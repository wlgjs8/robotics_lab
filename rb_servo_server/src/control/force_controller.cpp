#include "rb_servo/control/force_controller.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <utility>

namespace rb_servo {
namespace {

using AxisValues = std::array<double, 6>;

std::atomic<uint64_t> g_next_controller_id{1};

AxisValues poseValues(const Pose6D& value) {
    return {value.x, value.y, value.z, value.rx, value.ry, value.rz};
}

AxisValues vecValues(const Vec6& value) {
    return {value.x, value.y, value.z, value.rx, value.ry, value.rz};
}

AxisValues wrenchValues(const Wrench6D& value) {
    return {value.fx, value.fy, value.fz, value.tx, value.ty, value.tz};
}

Pose6D toPose(const AxisValues& value) {
    return {value[0], value[1], value[2], value[3], value[4], value[5]};
}

Vec6 toVec(const AxisValues& value) {
    return {value[0], value[1], value[2], value[3], value[4], value[5]};
}

Wrench6D toWrench(const AxisValues& value) {
    return {value[0], value[1], value[2], value[3], value[4], value[5]};
}

bool allFinite(const AxisValues& value) {
    return std::all_of(value.begin(), value.end(), [](double item) {
        return std::isfinite(item);
    });
}

AxisValues enabledAxes(const ForceControlAxis& axis) {
    return {
        axis.x ? 1.0 : 0.0,
        axis.y ? 1.0 : 0.0,
        axis.z ? 1.0 : 0.0,
        axis.roll ? 1.0 : 0.0,
        axis.pitch ? 1.0 : 0.0,
        axis.yaw ? 1.0 : 0.0,
    };
}

double tighten(double server_limit, double command_limit) {
    return command_limit > 0.0 ? std::min(server_limit, command_limit) : server_limit;
}

struct AxisMotionLimits {
    double offset = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    double jerk = 0.0;
};

struct AccelerationInterval {
    double lower = 0.0;
    double upper = 0.0;
};

bool within(double value, double lower, double upper) {
    constexpr double kTolerance = 1e-12;
    return value >= lower - kTolerance && value <= upper + kTolerance;
}

// Mirror of the proven scalar normal-admittance braking envelope, generalized
// to a symmetric Cartesian axis. It answers whether a state can still brake
// before the positive offset/velocity boundary without violating jerk.
bool upperBrakingFeasible(
    double offset,
    double velocity,
    double acceleration,
    const AxisMotionLimits& limits,
    double dt_sec
) {
    if (!within(offset, -limits.offset, limits.offset) ||
        !within(velocity, -limits.velocity, limits.velocity)) {
        return false;
    }
    if (velocity <= 0.0) return true;

    const double jerk_step = limits.jerk * dt_sec;
    const auto ramp_to_zero_velocity_delta = [&](double candidate_acceleration) {
        if (candidate_acceleration >= 0.0) return 0.0;
        const double ramp_steps = std::ceil(-candidate_acceleration / jerk_step) - 1.0;
        const double negative_steps = std::max(0.0, ramp_steps);
        return dt_sec * (
            negative_steps * candidate_acceleration +
            jerk_step * negative_steps * (negative_steps + 1.0) * 0.5
        );
    };
    const double required_steps = std::ceil(
        2.0 * limits.acceleration / jerk_step +
        2.0 * limits.velocity / (limits.acceleration * dt_sec)
    ) + 4.0;
    constexpr double kMaxBrakingLookaheadSteps = 4096.0;
    if (!std::isfinite(required_steps) || required_steps > kMaxBrakingLookaheadSteps) {
        return false;
    }

    for (int i = 0; i < static_cast<int>(required_steps); ++i) {
        const double more_negative = std::max(
            -limits.acceleration, acceleration - jerk_step
        );
        const auto terminal_velocity = [&](double candidate_acceleration) {
            return velocity + candidate_acceleration * dt_sec +
                ramp_to_zero_velocity_delta(candidate_acceleration);
        };
        if (terminal_velocity(more_negative) >= 0.0) {
            acceleration = more_negative;
        } else {
            double infeasible = more_negative;
            double feasible = std::min(limits.acceleration, acceleration + jerk_step);
            if (terminal_velocity(feasible) < -1e-12) {
                acceleration = feasible;
            } else {
                for (int search = 0; search < 40; ++search) {
                    const double midpoint = 0.5 * (infeasible + feasible);
                    if (terminal_velocity(midpoint) >= 0.0) feasible = midpoint;
                    else infeasible = midpoint;
                }
                acceleration = feasible;
            }
        }
        velocity += acceleration * dt_sec;
        offset += velocity * dt_sec;
        if (velocity > 0.0 &&
            (velocity > limits.velocity + 1e-12 || offset > limits.offset + 1e-12)) {
            return false;
        }
        if (velocity <= 1e-12) return true;
    }
    return false;
}

bool lowerBrakingFeasible(
    double offset,
    double velocity,
    double acceleration,
    const AxisMotionLimits& limits,
    double dt_sec
) {
    return upperBrakingFeasible(-offset, -velocity, -acceleration, limits, dt_sec);
}

bool tightenToJerkLimitedEnvelope(
    double offset,
    double velocity,
    double acceleration,
    const AxisMotionLimits& limits,
    double dt_sec,
    AccelerationInterval* interval
) {
    const double search_margin = std::min(1e-6, limits.jerk * dt_sec * 1e-3);
    const auto upper_ok = [&](double candidate_acceleration) {
        const double next_velocity = velocity + candidate_acceleration * dt_sec;
        const double next_offset = offset + next_velocity * dt_sec;
        return upperBrakingFeasible(
            next_offset, next_velocity, candidate_acceleration, limits, dt_sec
        );
    };
    const auto lower_ok = [&](double candidate_acceleration) {
        const double next_velocity = velocity + candidate_acceleration * dt_sec;
        const double next_offset = offset + next_velocity * dt_sec;
        return lowerBrakingFeasible(
            next_offset, next_velocity, candidate_acceleration, limits, dt_sec
        );
    };

    if (!upper_ok(interval->upper)) {
        if (!upper_ok(interval->lower)) return false;
        double feasible = interval->lower;
        double infeasible = interval->upper;
        for (int i = 0; i < 40; ++i) {
            const double midpoint = 0.5 * (feasible + infeasible);
            if (upper_ok(midpoint)) feasible = midpoint;
            else infeasible = midpoint;
        }
        interval->upper = std::max(interval->lower, feasible - search_margin);
    }
    if (!lower_ok(interval->lower)) {
        if (!lower_ok(interval->upper)) return false;
        double infeasible = interval->lower;
        double feasible = interval->upper;
        for (int i = 0; i < 40; ++i) {
            const double midpoint = 0.5 * (infeasible + feasible);
            if (lower_ok(midpoint)) feasible = midpoint;
            else infeasible = midpoint;
        }
        interval->lower = std::min(interval->upper, feasible + search_margin);
    }
    return interval->lower <= interval->upper + 1e-12;
}

}  // namespace

ForceController::ForceController(ForceControlConfig config)
    : config_(std::move(config)),
      controller_id_(g_next_controller_id.fetch_add(1, std::memory_order_relaxed)) {}

void ForceController::reset() {
    state_ = ForceControllerState{};
    engaged_ = false;
    ++lifecycle_generation_;
    ++state_revision_;
}

void ForceController::engage() {
    reset();
    engaged_ = true;
}

void ForceController::release() {
    reset();
}

void ForceController::reject() {
    // Deliberately retain the last committed state. A rejected IK/safety result
    // must never advance the controller's internal integrators.
}

bool ForceController::commit(const ForceControllerProposal& proposal) {
    if (!proposal.valid || !engaged_ || proposal.controller_id != controller_id_ ||
        proposal.lifecycle_generation != lifecycle_generation_ ||
        proposal.base_state_revision != state_revision_) {
        return false;
    }
    state_ = proposal.state;
    ++state_revision_;
    return true;
}

ForceControllerProposal ForceController::propose(
    const Wrench6D& measured_external_wrench_tcp,
    const ForceControlCommand& command,
    const Vec6& measured_actual_twist_tcp,
    double dt_sec
) const {
    ForceControllerProposal proposal;
    proposal.state = state_;
    proposal.controller_id = controller_id_;
    proposal.lifecycle_generation = lifecycle_generation_;
    proposal.base_state_revision = state_revision_;

    if (!config_.enable) {
        proposal.reason = "force control disabled";
        return proposal;
    }
    if (!engaged_) {
        proposal.reason = "force controller is not engaged";
        return proposal;
    }
    if (command.mode != ForceControlMode::Admittance &&
        command.mode != ForceControlMode::ExternalForceSafety) {
        proposal.reason = "unsupported or inactive force-control mode";
        return proposal;
    }
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0 || dt_sec > config_.max_dt_sec) {
        proposal.reason = "invalid controller dt";
        return proposal;
    }

    const AxisValues measured = wrenchValues(measured_external_wrench_tcp);
    const AxisValues target = wrenchValues(command.target_wrench);
    const AxisValues actual_twist = vecValues(measured_actual_twist_tcp);
    if (!allFinite(measured) || !allFinite(target) || !allFinite(actual_twist)) {
        proposal.reason = "non-finite controller input";
        return proposal;
    }

    const AxisValues old_offset = poseValues(state_.offset_tcp);
    const AxisValues old_velocity = vecValues(state_.velocity_tcp);
    const AxisValues old_acceleration = vecValues(state_.acceleration_tcp);
    const AxisValues enabled = enabledAxes(command.enabled_axis);
    AxisValues next_offset = old_offset;
    AxisValues next_velocity = old_velocity;
    AxisValues next_acceleration = old_acceleration;
    AxisValues wrench_error{};

    const double pos_offset = tighten(config_.max_pos_offset_m, command.max_pos_offset_m);
    const double rot_offset = tighten(config_.max_rot_offset_rad, command.max_rot_offset_rad);
    const double pos_step = tighten(config_.max_pos_step_m, command.max_pos_step_m);
    const double rot_step = tighten(config_.max_rot_step_rad, command.max_rot_step_rad);

    for (std::size_t i = 0; i < 6; ++i) {
        if (enabled[i] == 0.0) continue;

        // The installed sensor reports the reaction wrench on the TCP.  A
        // positive correction must therefore oppose an excess measured
        // wrench, i.e. target - measured.  Remove the configured quiet-zone
        // without introducing a discontinuity at its boundary.
        const double raw_error = target[i] - measured[i];
        const double deadband = config_.wrench_deadband[i];
        wrench_error[i] = std::abs(raw_error) <= deadband
            ? 0.0
            : raw_error - std::copysign(deadband, raw_error);
        const double raw_acceleration =
            (wrench_error[i] - config_.damping[i] * old_velocity[i] -
             config_.stiffness[i] * old_offset[i]) /
            config_.virtual_mass[i];

        const bool translation = i < 3;
        const double acceleration_limit = translation
            ? config_.max_linear_acceleration_m_s2
            : config_.max_angular_acceleration_rad_s2;
        const double jerk_limit = translation
            ? config_.max_linear_jerk_m_s3
            : config_.max_angular_jerk_rad_s3;
        const double velocity_limit = translation
            ? config_.max_linear_velocity_m_s
            : config_.max_angular_velocity_rad_s;
        const double offset_limit = translation ? pos_offset : rot_offset;
        const double step_limit = translation ? pos_step : rot_step;

        const AxisMotionLimits limits{
            offset_limit,
            std::min(velocity_limit, step_limit / dt_sec),
            acceleration_limit,
            jerk_limit,
        };
        const double jerk_step = limits.jerk * dt_sec;
        AccelerationInterval interval{
            std::max(-limits.acceleration, old_acceleration[i] - jerk_step),
            std::min(limits.acceleration, old_acceleration[i] + jerk_step),
        };
        interval.lower = std::max(
            interval.lower, (-limits.velocity - old_velocity[i]) / dt_sec
        );
        interval.upper = std::min(
            interval.upper, (limits.velocity - old_velocity[i]) / dt_sec
        );
        interval.lower = std::max(
            interval.lower,
            ((-limits.offset - old_offset[i]) / dt_sec - old_velocity[i]) / dt_sec
        );
        interval.upper = std::min(
            interval.upper,
            ((limits.offset - old_offset[i]) / dt_sec - old_velocity[i]) / dt_sec
        );
        if (interval.lower > interval.upper + 1e-6 ||
            !tightenToJerkLimitedEnvelope(
                old_offset[i], old_velocity[i], old_acceleration[i],
                limits, dt_sec, &interval
            )) {
            proposal.reason = "Cartesian axis state is outside the jerk-limited motion envelope";
            return proposal;
        }
        if (interval.lower > interval.upper) {
            const double boundary = 0.5 * (interval.lower + interval.upper);
            interval.lower = boundary;
            interval.upper = boundary;
        }

        double acceleration = std::clamp(raw_acceleration, interval.lower, interval.upper);
        const bool axis_limited = std::abs(acceleration - raw_acceleration) > 1e-12;
        proposal.limit_axes[i] = axis_limited;
        proposal.saturated = proposal.saturated || axis_limited;
        double velocity = std::clamp(
            old_velocity[i] + acceleration * dt_sec,
            -limits.velocity,
            limits.velocity
        );
        double offset = std::clamp(
            old_offset[i] + velocity * dt_sec,
            -limits.offset,
            limits.offset
        );
        velocity = (offset - old_offset[i]) / dt_sec;
        acceleration = (velocity - old_velocity[i]) / dt_sec;
        if (std::abs(acceleration) > limits.acceleration + 1e-6 ||
            std::abs(acceleration - old_acceleration[i]) > jerk_step + 1e-6) {
            proposal.reason = "Cartesian axis numerical boundary reconciliation exceeded limits";
            return proposal;
        }
        next_offset[i] = offset;
        next_velocity[i] = velocity;
        next_acceleration[i] = acceleration;
    }

    double power_w = 0.0;
    for (std::size_t i = 0; i < 6; ++i) {
        if (enabled[i] != 0.0) power_w += measured[i] * actual_twist[i];
    }
    const double energy = std::max(0.0, state_.observed_energy_j + power_w * dt_sec);
    if (!std::isfinite(energy) || energy > config_.max_energy_j) {
        proposal.reason = "passivity energy limit exceeded";
        return proposal;
    }

    proposal.state.offset_tcp = toPose(next_offset);
    proposal.state.velocity_tcp = toVec(next_velocity);
    proposal.state.acceleration_tcp = toVec(next_acceleration);
    proposal.state.observed_energy_j = energy;
    proposal.wrench_error_tcp = toWrench(wrench_error);
    proposal.valid = true;
    proposal.limit_reason = proposal.saturated ? "jerk_limited_motion_envelope" : "";
    proposal.reason = proposal.saturated ? "bounded" : "ok";
    return proposal;
}

}  // namespace rb_servo
