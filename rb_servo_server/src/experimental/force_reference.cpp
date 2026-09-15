#include "rb_servo/experimental/force_reference.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rb_servo::experimental {
namespace {
bool positive(double value) { return std::isfinite(value) && value > 0; }
bool finite(const ForceReferenceState& state) {
    return state.position.allFinite() && state.velocity.allFinite() &&
           state.acceleration.allFinite();
}
Eigen::Vector3d limited(const Eigen::Vector3d& value, double limit) {
    const double norm = value.norm();
    return norm > limit ? Eigen::Vector3d(value * (limit / norm)) : value;
}
}

void ForceReference::validate(const ForceReferenceConfig& c, double dt) {
    if (!positive(dt) || !positive(c.sustained_force_n) ||
        !positive(c.recovery_force_n) || !std::isfinite(c.noise_force_n) || c.noise_force_n < 0 ||
        !(c.noise_force_n < c.recovery_force_n && c.recovery_force_n < c.sustained_force_n) ||
        !positive(c.mass_kg) || !positive(c.damping_ns_m) ||
        !positive(c.contact_stiffness_n_m) || !positive(c.max_velocity_m_s) ||
        !positive(c.max_acceleration_m_s2) || !positive(c.max_jerk_m_s3)) {
        throw std::invalid_argument("force reference requires finite explicit dynamics/limits and "
                                    "0 <= noise_force_n < recovery_force_n < sustained_force_n");
    }
}

ForceReference::ForceReference(const ForceReferenceConfig& c, double dt)
    : config_(c), dt_(dt), trajectory_(dt) {
    validate(c, dt);
    input_.control_interface = ruckig::ControlInterface::Velocity;
    input_.synchronization = ruckig::Synchronization::None;
    input_.max_velocity.fill(c.max_velocity_m_s);
    input_.max_acceleration.fill(c.max_acceleration_m_s2);
    input_.max_jerk.fill(c.max_jerk_m_s3);
    input_.target_acceleration.fill(0);
}

void ForceReference::clear() {
    state_ = {};
    initialized_ = false;
    last_step_valid_ = false;
    trajectory_.reset();
}

void ForceReference::reset(const ForceReferenceState& accepted) {
    if (!finite(accepted) ||
        accepted.velocity.cwiseAbs().maxCoeff() > config_.max_velocity_m_s ||
        accepted.acceleration.cwiseAbs().maxCoeff() > config_.max_acceleration_m_s2) {
        throw std::invalid_argument("force reference seed is non-finite or outside motion limits");
    }
    state_ = accepted;
    initialized_ = true;
    last_step_valid_ = false;
    trajectory_.reset();
}

ForceReferenceResult ForceReference::step(const Eigen::Vector3d& goal,
                                          const Eigen::Vector3d& goal_velocity,
                                          const Eigen::Vector3d& measured,
                                          const Eigen::Vector3d& wrench) {
    // Retained position-pursuit baseline. Its absolute position error can grow
    // while a moving goal is obstructed; stepVelocity deliberately has no such
    // error state. Do not use this baseline to claim release anti-windup.
    if (!goal.allFinite() || !goal_velocity.allFinite()) {
        last_step_valid_=false;
        ForceReferenceResult out; out.state=state_;out.reason="nonfinite_input";
        return out;
    }
    return stepVelocity(goal_velocity + (config_.damping_ns_m / config_.mass_kg) *
                        (goal - state_.position), measured, wrench);
}

ForceReferenceResult ForceReference::stepVelocity(const Eigen::Vector3d& requested,
                                                  const Eigen::Vector3d& measured,
                                                  const Eigen::Vector3d& wrench) {
    last_step_valid_=false;
    ForceReferenceResult out;
    out.state = state_;
    if (!initialized_) return out;
    if (!requested.allFinite() ||
        !measured.allFinite() || !wrench.allFinite()) {
        out.reason = "nonfinite_input";
        return out;
    }
    const auto& c = config_;
    // Project before applying the global speed budget: an unreachable normal
    // goal must not consume that budget and starve valid tangential motion.
    out.requested_velocity = requested;
    if (!out.requested_velocity.allFinite() || !std::isfinite(out.requested_velocity.norm())) {
        out.reason="invalid_velocity_intent";return out;
    }
    out.measured_force_n = wrench.norm();
    if (!std::isfinite(out.measured_force_n)) { out.reason="force_overflow";return out; }

    // A radial sensor noise allowance keeps the response independent of tool axes.
    // Predict the force at the already commanded pose, instead of treating delayed
    // measured contact as if no motion were still in flight. Prediction applies only
    // to resolved force: an undefined direction in the noise band carries no model.
    if (out.measured_force_n > c.noise_force_n) {
        const Eigen::Vector3d normal = wrench / out.measured_force_n;
        const double resolved = out.measured_force_n - c.noise_force_n;
        const double recovery = c.recovery_force_n - c.noise_force_n;
        // Uncertain normal directions cannot carry a full stiffness extrapolation
        // as the resolved force approaches zero. This continuous confidence tends
        // quadratically to zero; otherwise crossing the noise allowance by epsilon
        // could instantly predict K * in_flight_displacement newtons.
        // It weights the contact model, not the policy's motion authority.
        const double confidence = resolved * resolved / (resolved * resolved + recovery * recovery);
        const double magnitude = std::max(0.0, resolved -
            confidence * c.contact_stiffness_n_m * normal.dot(state_.position - measured));
        out.predicted_force = normal * magnitude;
    }
    const auto& force = out.predicted_force;
    Eigen::Vector3d velocity = out.requested_velocity + force / c.damping_ns_m;
    const double recovery = c.recovery_force_n - c.noise_force_n;
    const double square = force.squaredNorm();
    // Minimum-change projection onto a force recovery half-space. No normalized
    // sign switch occurs near zero: 0*v >= -recovery^2/(2D) is strictly feasible.
    // The boundary permits weak contact and requests unloading above recovery.
    // It is a velocity reference condition, not an instantaneous force certificate.
    const double lower = (square - recovery * recovery) / (2 * c.damping_ns_m);
    const double violation = lower - force.dot(velocity);
    if (!force.allFinite() || !velocity.allFinite() || !std::isfinite(violation)) {
        out.reason="prediction_overflow";
        return out;
    }
    if (violation > 0 && square > 0) {
        velocity += (violation / square) * force;
        out.constraint_active = true;
    }
    out.compliant_velocity = limited(velocity, c.max_velocity_m_s);

    // Backward Euler for M*v_dot + D*v = D*v_compliant, followed by a velocity
    // OTG from this same executed p/v/a. The OTG owns all integration, including
    // braking at velocity limits; clipping its position afterward would break C2.
    const Eigen::Vector3d next_velocity =
        (c.mass_kg * state_.velocity + dt_ * c.damping_ns_m * out.compliant_velocity) /
        (c.mass_kg + dt_ * c.damping_ns_m);
    for (int axis = 0; axis < 3; ++axis) {
        input_.current_position[axis] = state_.position[axis];
        input_.current_velocity[axis] = state_.velocity[axis];
        input_.current_acceleration[axis] = state_.acceleration[axis];
        input_.target_velocity[axis] = next_velocity[axis];
    }
    const auto result = trajectory_.update(input_, output_);
    if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
        out.reason = "velocity_trajectory_refused";
        return out;
    }
    ForceReferenceState next;
    for (int axis = 0; axis < 3; ++axis) {
        next.position[axis] = output_.new_position[axis];
        next.velocity[axis] = output_.new_velocity[axis];
        next.acceleration[axis] = output_.new_acceleration[axis];
    }
    if (!finite(next)) { out.reason = "nonfinite_result"; return out; }
    state_ = next;
    last_step_valid_ = true;
    out.state = state_;
    out.valid = true;
    out.reason = "tracking";
    return out;
}

bool ForceReference::sampleLastStep(double t, ForceReferenceState& out) const {
    if(!last_step_valid_||!std::isfinite(t)||t<0||t>dt_)return false;
    std::array<double,3> p{},v{},a{};
    output_.trajectory.at_time(output_.time-dt_+t,p,v,a);
    for(int axis=0;axis<3;++axis) {out.position[axis]=p[axis];out.velocity[axis]=v[axis];out.acceleration[axis]=a[axis];}
    return finite(out);
}

} // namespace rb_servo::experimental
