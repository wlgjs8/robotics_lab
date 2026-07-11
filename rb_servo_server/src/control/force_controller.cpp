#include "rb_servo/control/force_controller.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
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

struct AxisState {
    double offset = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

struct JerkInterval {
    double lower = 0.0;
    double upper = 0.0;
};

double scaledTolerance(double scale) {
    return 128.0 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, std::abs(scale));
}

bool within(double value, double lower, double upper, double scale) {
    const double tolerance = scaledTolerance(scale);
    return value >= lower - tolerance && value <= upper + tolerance;
}

AxisState advanceWithJerk(
    const AxisState& state,
    double jerk,
    double dt_sec
) {
    AxisState next;
    next.acceleration = state.acceleration + jerk * dt_sec;
    // Preserve the controller's established semi-implicit discrete dynamics:
    // the new acceleration drives this tick's velocity and the new velocity
    // drives this tick's position.  Jerk is now the selected control input, so
    // all three states are produced by one consistent update instead of being
    // integrated and then reconciled with clamps.
    next.velocity = state.velocity + next.acceleration * dt_sec;
    next.offset = state.offset + next.velocity * dt_sec;
    return next;
}

// Mirror of the proven scalar normal-admittance braking envelope, generalized
// to a symmetric Cartesian axis. It answers whether a state can still brake
// before the positive offset/velocity boundary without violating jerk.
bool upperBrakingFeasible(
    AxisState state,
    const AxisMotionLimits& limits,
    double dt_sec
) {
    if (!within(state.offset, -limits.offset, limits.offset, limits.offset) ||
        !within(state.velocity, -limits.velocity, limits.velocity, limits.velocity) ||
        !within(
            state.acceleration,
            -limits.acceleration,
            limits.acceleration,
            limits.acceleration
        )) {
        return false;
    }
    if (state.velocity <= 0.0) return true;

    const double jerk_step = limits.jerk * dt_sec;
    const double required_steps = std::ceil(
        limits.acceleration / jerk_step +
        2.0 * limits.velocity / (limits.acceleration * dt_sec)
    ) + 4.0;
    constexpr double kMaxBrakingLookaheadSteps = 4096.0;
    if (!std::isfinite(required_steps) || required_steps > kMaxBrakingLookaheadSteps) {
        return false;
    }

    for (int i = 0; i < static_cast<int>(required_steps); ++i) {
        // Maximum inward jerk minimizes both the future outward velocity and
        // the distance travelled toward this boundary.  Reversing velocity is
        // sufficient for boundary viability; requiring acceleration to ramp
        // back to zero at the same instant unnecessarily collapses the viable
        // set at a velocity limit and caused the observed recenter fault.
        state.acceleration = std::max(
            -limits.acceleration, state.acceleration - jerk_step
        );
        state.velocity += state.acceleration * dt_sec;
        state.offset += state.velocity * dt_sec;
        if (state.velocity > 0.0 &&
            (!within(
                 state.velocity, -limits.velocity, limits.velocity, limits.velocity
             ) ||
             !within(state.offset, -limits.offset, limits.offset, limits.offset))) {
            return false;
        }
        if (state.velocity <= scaledTolerance(limits.velocity)) return true;
    }
    return false;
}

bool lowerBrakingFeasible(
    const AxisState& state,
    const AxisMotionLimits& limits,
    double dt_sec
) {
    return upperBrakingFeasible(
        {-state.offset, -state.velocity, -state.acceleration}, limits, dt_sec
    );
}

bool collapseTinyInterval(JerkInterval* interval, double jerk_limit) {
    const double tolerance = 64.0 * scaledTolerance(jerk_limit);
    if (interval->lower > interval->upper + tolerance) return false;
    if (interval->lower > interval->upper) {
        const double boundary = 0.5 * (interval->lower + interval->upper);
        interval->lower = boundary;
        interval->upper = boundary;
    }
    return true;
}

bool tightenToJerkLimitedEnvelope(
    const AxisState& state,
    const AxisMotionLimits& limits,
    double dt_sec,
    JerkInterval* interval
) {
    const auto upper_ok = [&](double candidate_jerk) {
        return upperBrakingFeasible(
            advanceWithJerk(state, candidate_jerk, dt_sec), limits, dt_sec
        );
    };
    const auto lower_ok = [&](double candidate_jerk) {
        return lowerBrakingFeasible(
            advanceWithJerk(state, candidate_jerk, dt_sec), limits, dt_sec
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
        interval->upper = feasible;
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
        interval->lower = feasible;
    }
    return collapseTinyInterval(interval, limits.jerk);
}

bool constrainNextState(
    const AxisState& state,
    const AxisMotionLimits& limits,
    double dt_sec,
    JerkInterval* interval
) {
    const double dt2 = dt_sec * dt_sec;
    const double dt3 = dt2 * dt_sec;
    interval->lower = std::max(
        interval->lower,
        (-limits.acceleration - state.acceleration) / dt_sec
    );
    interval->upper = std::min(
        interval->upper,
        (limits.acceleration - state.acceleration) / dt_sec
    );
    interval->lower = std::max(
        interval->lower,
        (-limits.velocity - state.velocity - state.acceleration * dt_sec) / dt2
    );
    interval->upper = std::min(
        interval->upper,
        (limits.velocity - state.velocity - state.acceleration * dt_sec) / dt2
    );
    interval->lower = std::max(
        interval->lower,
        (-limits.offset - state.offset - state.velocity * dt_sec -
         state.acceleration * dt2) / dt3
    );
    interval->upper = std::min(
        interval->upper,
        (limits.offset - state.offset - state.velocity * dt_sec -
         state.acceleration * dt2) / dt3
    );
    return collapseTinyInterval(interval, limits.jerk);
}

bool feasibleJerkInterval(
    const AxisState& state,
    const AxisMotionLimits& limits,
    double dt_sec,
    JerkInterval* interval
) {
    *interval = {-limits.jerk, limits.jerk};
    return constrainNextState(state, limits, dt_sec, interval) &&
        tightenToJerkLimitedEnvelope(state, limits, dt_sec, interval);
}

AxisMotionLimits softGovernorLimits(
    const AxisMotionLimits& hard,
    double dt_sec
) {
    // The soft envelope is the recursively invariant operating set.  The hard
    // configured limits remain untouched and provide deterministic recovery
    // authority if floating-point or a prior implementation leaves a state on
    // the exact viability boundary.  Margins are derived from one control step,
    // not hand-tuned filtering or an output moving average.
    const double velocity_margin = std::min(
        0.05 * hard.velocity,
        std::max(
            scaledTolerance(hard.velocity), 2.0 * hard.jerk * dt_sec * dt_sec
        )
    );
    const double offset_margin = std::min(
        0.05 * hard.offset,
        std::max(
            scaledTolerance(hard.offset),
            2.0 * hard.velocity * dt_sec + hard.acceleration * dt_sec * dt_sec
        )
    );
    AxisMotionLimits soft = hard;
    soft.velocity = std::max(0.5 * hard.velocity, hard.velocity - velocity_margin);
    soft.offset = std::max(0.5 * hard.offset, hard.offset - offset_margin);
    return soft;
}

double recoveryJerk(
    const AxisState& state,
    const AxisMotionLimits& soft
) {
    if (state.velocity > soft.velocity ||
        (state.velocity > 0.0 && state.acceleration > 0.0)) {
        return -soft.jerk;
    }
    if (state.velocity < -soft.velocity ||
        (state.velocity < 0.0 && state.acceleration < 0.0)) {
        return soft.jerk;
    }
    if (state.offset > soft.offset) return -soft.jerk;
    if (state.offset < -soft.offset) return soft.jerk;
    if (state.acceleration > 0.0) return -soft.jerk;
    if (state.acceleration < 0.0) return soft.jerk;
    return 0.0;
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
        const AxisState old_state{
            old_offset[i], old_velocity[i], old_acceleration[i]
        };
        const AxisMotionLimits soft_limits = softGovernorLimits(limits, dt_sec);
        JerkInterval interval;
        bool recovering = false;
        if (!feasibleJerkInterval(old_state, soft_limits, dt_sec, &interval)) {
            recovering = true;
            if (!feasibleJerkInterval(old_state, limits, dt_sec, &interval)) {
                proposal.reason =
                    "Cartesian axis state is outside the hard jerk-governed motion envelope";
                return proposal;
            }
        }

        const double unbounded_desired_jerk =
            (raw_acceleration - old_acceleration[i]) / dt_sec;
        const double desired_jerk = std::clamp(
            unbounded_desired_jerk,
            -limits.jerk,
            limits.jerk
        );
        const double governed_jerk = std::clamp(
            recovering ? recoveryJerk(old_state, soft_limits) : desired_jerk,
            interval.lower,
            interval.upper
        );
        const AxisState next_state = advanceWithJerk(
            old_state, governed_jerk, dt_sec
        );
        if (!within(
                next_state.acceleration,
                -limits.acceleration,
                limits.acceleration,
                limits.acceleration
            ) ||
            !within(
                next_state.velocity,
                -limits.velocity,
                limits.velocity,
                limits.velocity
            ) ||
            !within(
                next_state.offset,
                -limits.offset,
                limits.offset,
                limits.offset
            )) {
            proposal.reason = "Cartesian jerk governor produced a state outside hard limits";
            return proposal;
        }
        const bool axis_limited = recovering ||
            std::abs(governed_jerk - unbounded_desired_jerk) >
                scaledTolerance(limits.jerk);
        proposal.limit_axes[i] = axis_limited;
        proposal.saturated = proposal.saturated || axis_limited;
        next_offset[i] = next_state.offset;
        next_velocity[i] = next_state.velocity;
        next_acceleration[i] = next_state.acceleration;
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
