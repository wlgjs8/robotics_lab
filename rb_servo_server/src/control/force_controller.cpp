#include "rb_servo/control/force_controller.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <utility>

namespace rb_servo {

void ContactForceNormalEstimator::update(
    bool episode_active,
    const math::Vector3& force_control_stand
) {
    if (!episode_active) {
        reset();
        return;
    }
    if (episode_active_) return;
    episode_active_ = true;
    if (!force_control_stand.allFinite() ||
        !(force_control_stand.squaredNorm() > 0.0)) {
        valid_ = false;
        normal_stand_.setZero();
        return;
    }
    normal_stand_ = -force_control_stand.normalized();
    valid_ = true;
}

void ContactForceNormalEstimator::reset() {
    episode_active_ = false;
    valid_ = false;
    normal_stand_.setZero();
}

Pose6D removeComplianceOffsetFromMeasured(const Pose6D& measured_stand,
                                          const Pose6D& t_tcp_compliance_pose,
                                          const Pose6D& offset_tcp) {
    const pinocchio::SE3 c = math::se3FromPose(t_tcp_compliance_pose);
    const pinocchio::SE3 offset(
        math::exp3(math::Vector3(offset_tcp.rx, offset_tcp.ry, offset_tcp.rz)),
        math::Vector3(offset_tcp.x, offset_tcp.y, offset_tcp.z)
    );
    // applyForceCorrection composes: sent = command * (c * offset * c^-1).
    // The measured pose tracks `sent`, so right-multiply by the inverse of the
    // same TCP-local correction to recover the un-complied pose.
    const pinocchio::SE3 correction = c * offset * c.inverse();
    return math::poseFromSe3(
        math::se3FromPose(measured_stand) * correction.inverse()
    );
}

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

bool sharedFeasibleJerkScale(
    std::size_t begin,
    std::size_t end,
    const AxisValues& enabled,
    const AxisValues& desired_jerk,
    const std::array<JerkInterval, 6>& intervals,
    double* scale
) {
    double lower_scale = 0.0;
    double upper_scale = 1.0;
    std::size_t coupled_axis_count = 0;
    for (std::size_t i = begin; i < end; ++i) {
        if (enabled[i] == 0.0) continue;
        const double desired = desired_jerk[i];
        if (std::abs(desired) <=
            128.0 * std::numeric_limits<double>::epsilon()) {
            if (intervals[i].lower > 0.0 || intervals[i].upper < 0.0) {
                return false;
            }
            continue;
        }
        ++coupled_axis_count;
        const double first = intervals[i].lower / desired;
        const double second = intervals[i].upper / desired;
        lower_scale = std::max(lower_scale, std::min(first, second));
        upper_scale = std::min(upper_scale, std::max(first, second));
    }
    if (coupled_axis_count < 2 || lower_scale > upper_scale) return false;
    *scale = std::clamp(upper_scale, 0.0, 1.0);
    return *scale >= lower_scale;
}

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

double loadedBoundaryBrakingJerk(
    const AxisState& state,
    double load_direction,
    const AxisMotionLimits& limits,
    double dt_sec
) {
    AxisState aligned{
        load_direction * state.offset,
        load_direction * state.velocity,
        load_direction * state.acceleration,
    };
    const double jerk_step = limits.jerk * dt_sec;
    const auto ramp_to_zero_velocity_delta = [&](double candidate_acceleration) {
        if (candidate_acceleration >= 0.0) return 0.0;
        const double ramp_steps = std::ceil(
            -candidate_acceleration / jerk_step
        ) - 1.0;
        const double negative_steps = std::max(0.0, ramp_steps);
        return dt_sec * (
            negative_steps * candidate_acceleration +
            jerk_step * negative_steps * (negative_steps + 1.0) * 0.5
        );
    };
    const auto terminal_velocity = [&](double candidate_acceleration) {
        return aligned.velocity + candidate_acceleration * dt_sec +
            ramp_to_zero_velocity_delta(candidate_acceleration);
    };

    const double more_braking = std::max(
        -limits.acceleration, aligned.acceleration - jerk_step
    );
    double next_acceleration = more_braking;
    if (terminal_velocity(more_braking) < 0.0) {
        double reversing = more_braking;
        double non_reversing = std::min(
            limits.acceleration, aligned.acceleration + jerk_step
        );
        if (terminal_velocity(non_reversing) < 0.0) {
            next_acceleration = non_reversing;
        } else {
            for (int search = 0; search < 40; ++search) {
                const double midpoint = 0.5 * (reversing + non_reversing);
                if (terminal_velocity(midpoint) >= 0.0) {
                    non_reversing = midpoint;
                } else {
                    reversing = midpoint;
                }
            }
            next_acceleration = non_reversing;
        }
    }
    const double aligned_jerk = std::clamp(
        (next_acceleration - aligned.acceleration) / dt_sec,
        -limits.jerk,
        limits.jerk
    );
    return load_direction * aligned_jerk;
}

bool loadedStopFeasible(
    AxisState state,
    const AxisMotionLimits& limits,
    double dt_sec
) {
    const double velocity_tolerance = scaledTolerance(limits.velocity);
    const double acceleration_tolerance = scaledTolerance(limits.acceleration);
    if (!within(state.offset, -limits.offset, limits.offset, limits.offset) ||
        !within(state.velocity, 0.0, limits.velocity, limits.velocity) ||
        !within(
            state.acceleration,
            -limits.acceleration,
            limits.acceleration,
            limits.acceleration
        )) {
        return false;
    }
    if (state.velocity <= velocity_tolerance &&
        std::abs(state.acceleration) <= acceleration_tolerance) {
        return true;
    }

    const double jerk_step = limits.jerk * dt_sec;
    const double required_steps = std::ceil(
        2.0 * limits.acceleration / jerk_step +
        2.0 * limits.velocity / (limits.acceleration * dt_sec)
    ) + 8.0;
    constexpr double kMaxLoadedStopSteps = 4096.0;
    if (!std::isfinite(required_steps) || required_steps > kMaxLoadedStopSteps) {
        return false;
    }

    const auto ramp_to_zero_velocity_delta = [&](double candidate_acceleration) {
        if (candidate_acceleration >= 0.0) return 0.0;
        const double ramp_steps = std::ceil(
            -candidate_acceleration / jerk_step
        ) - 1.0;
        const double negative_steps = std::max(0.0, ramp_steps);
        return dt_sec * (
            negative_steps * candidate_acceleration +
            jerk_step * negative_steps * (negative_steps + 1.0) * 0.5
        );
    };

    bool ramping_to_hold = false;
    for (int i = 0; i < static_cast<int>(required_steps); ++i) {
        if (ramping_to_hold) {
            state.acceleration = std::min(
                0.0, state.acceleration + jerk_step
            );
            state.velocity += state.acceleration * dt_sec;
            state.offset += state.velocity * dt_sec;
            if (!within(state.offset, -limits.offset, limits.offset, limits.offset) ||
                !within(state.velocity, 0.0, limits.velocity, limits.velocity)) {
                return false;
            }
            if (std::abs(state.acceleration) <= acceleration_tolerance) {
                return state.velocity <= velocity_tolerance;
            }
            continue;
        }

        const double more_braking = std::max(
            -limits.acceleration, state.acceleration - jerk_step
        );
        const auto terminal_velocity = [&](double candidate_acceleration) {
            return state.velocity + candidate_acceleration * dt_sec +
                ramp_to_zero_velocity_delta(candidate_acceleration);
        };
        double next_acceleration = more_braking;
        if (terminal_velocity(more_braking) < 0.0) {
            double reversing = more_braking;
            double non_reversing = std::min(
                limits.acceleration, state.acceleration + jerk_step
            );
            if (terminal_velocity(non_reversing) < 0.0) return false;
            for (int search = 0; search < 40; ++search) {
                const double midpoint = 0.5 * (reversing + non_reversing);
                if (terminal_velocity(midpoint) >= 0.0) {
                    non_reversing = midpoint;
                } else {
                    reversing = midpoint;
                }
            }
            next_acceleration = non_reversing;
            ramping_to_hold = next_acceleration < -acceleration_tolerance;
        }
        state.acceleration = next_acceleration;
        state.velocity += state.acceleration * dt_sec;
        state.offset += state.velocity * dt_sec;
        if (!within(state.offset, -limits.offset, limits.offset, limits.offset) ||
            !within(state.velocity, 0.0, limits.velocity, limits.velocity)) {
            return false;
        }
        if (state.velocity <= velocity_tolerance &&
            std::abs(state.acceleration) <= acceleration_tolerance) {
            return true;
        }
    }
    return false;
}

bool loadedHoldStopFeasible(
    const AxisState& state,
    const AxisMotionLimits& limits,
    double dt_sec
) {
    const double offset_tolerance = scaledTolerance(limits.offset);
    if (state.offset < -offset_tolerance) return false;
    const double meaningful_return_velocity =
        2.0 * limits.jerk * dt_sec * dt_sec;
    if (state.velocity >= -meaningful_return_velocity) {
        return loadedStopFeasible(state, limits, dt_sec);
    }

    // The axis is already returning toward the command equilibrium when the
    // load reappears.  Measure the remaining return travel from the zero
    // boundary and mirror velocity/acceleration so the same one-sided stop
    // oracle proves that the return can brake before crossing that boundary.
    return loadedStopFeasible(
        {
            limits.offset - state.offset,
            -state.velocity,
            -state.acceleration,
        },
        limits,
        dt_sec
    );
}

bool tightenToLoadedHoldEnvelope(
    const AxisState& state,
    double load_direction,
    const AxisMotionLimits& limits,
    double dt_sec,
    JerkInterval* interval
) {
    AxisState aligned{
        load_direction * state.offset,
        load_direction * state.velocity,
        load_direction * state.acceleration,
    };
    JerkInterval aligned_interval = load_direction > 0.0
        ? *interval
        : JerkInterval{-interval->upper, -interval->lower};
    const auto feasible = [&](double aligned_jerk) {
        return loadedHoldStopFeasible(
            advanceWithJerk(aligned, aligned_jerk, dt_sec), limits, dt_sec
        );
    };
    const double braking_seed = std::clamp(
        load_direction * loadedBoundaryBrakingJerk(
            state, load_direction, limits, dt_sec
        ),
        aligned_interval.lower,
        aligned_interval.upper
    );
    double seed = braking_seed;
    bool seed_feasible = feasible(seed);
    if (!seed_feasible) {
        // At initial contact the braking action points opposite the applied
        // load and is intentionally infeasible for a unilateral loaded hold.
        // Search the small one-dimensional interval for a forward action that
        // preserves enough room to stop without ever reversing toward zero.
        constexpr int kSeedSamples = 16;
        for (int sample = 0; sample <= kSeedSamples && !seed_feasible; ++sample) {
            const double ratio = static_cast<double>(sample) / kSeedSamples;
            seed = aligned_interval.lower +
                ratio * (aligned_interval.upper - aligned_interval.lower);
            seed_feasible = feasible(seed);
        }
    }
    if (!seed_feasible) return false;

    if (!feasible(aligned_interval.lower)) {
        double infeasible = aligned_interval.lower;
        double feasible_bound = seed;
        for (int i = 0; i < 32; ++i) {
            const double midpoint = 0.5 * (infeasible + feasible_bound);
            if (feasible(midpoint)) feasible_bound = midpoint;
            else infeasible = midpoint;
        }
        aligned_interval.lower = feasible_bound;
    }
    if (!feasible(aligned_interval.upper)) {
        double feasible_bound = seed;
        double infeasible = aligned_interval.upper;
        for (int i = 0; i < 32; ++i) {
            const double midpoint = 0.5 * (feasible_bound + infeasible);
            if (feasible(midpoint)) feasible_bound = midpoint;
            else infeasible = midpoint;
        }
        aligned_interval.upper = feasible_bound;
    }
    if (!collapseTinyInterval(&aligned_interval, limits.jerk)) return false;
    *interval = load_direction > 0.0
        ? aligned_interval
        : JerkInterval{-aligned_interval.upper, -aligned_interval.lower};
    return true;
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

    // Preserve the established component deadbands while making release a
    // block decision.  A sibling axis that becomes quiet while another axis
    // is still loaded must not start its spring return independently.
    for (std::size_t i = 0; i < 6; ++i) {
        if (enabled[i] == 0.0) continue;
        const double raw_error = target[i] - measured[i];
        const double deadband = config_.wrench_deadband[i];
        wrench_error[i] = std::abs(raw_error) <= deadband
            ? 0.0
            : raw_error - std::copysign(deadband, raw_error);
    }
    // Layer-3 generic force limiter: above force_limit_n the TRANSLATION
    // response envelope opens proportionally to the excess force. The escape
    // direction needs no model or frozen normal — the per-axis wrench error IS
    // the drive; the wider envelope just lets the existing mass-damper follow
    // it fast enough that the contact force saturates near the limit instead
    // of climbing into the hard-limit latch. (The comparison uses the
    // deadband-shaved error norm, i.e. it is deadband-conservative.)
    double translation_velocity_boost = 0.0;
    double translation_dynamic_scale = 1.0;
    if (config_.force_limit_n > 0.0) {
        const double error_norm = std::sqrt(
            wrench_error[0] * wrench_error[0] +
            wrench_error[1] * wrench_error[1] +
            wrench_error[2] * wrench_error[2]);
        const double excess = error_norm - config_.force_limit_n;
        if (excess > 0.0) {
            const double headroom = std::max(
                0.0,
                config_.backoff_max_velocity_m_s - config_.max_linear_velocity_m_s);
            translation_velocity_boost = std::min(
                config_.backoff_gain_m_s_per_n * excess, headroom);
            translation_dynamic_scale = 1.0 +
                translation_velocity_boost /
                    std::max(1e-9, config_.max_linear_velocity_m_s);
        }
    }
    const auto block_loaded = [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            if (enabled[i] != 0.0 &&
                std::abs(wrench_error[i]) > scaledTolerance(1.0)) {
                return true;
            }
        }
        return false;
    };
    const auto block_residual = [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            if (enabled[i] != 0.0 &&
                (std::abs(old_offset[i]) > scaledTolerance(1.0) ||
                 std::abs(old_velocity[i]) > scaledTolerance(1.0) ||
                 std::abs(old_acceleration[i]) > scaledTolerance(1.0))) {
                return true;
            }
        }
        return false;
    };
    const auto quiet_axis_residual = [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            if (enabled[i] != 0.0 &&
                std::abs(wrench_error[i]) <= scaledTolerance(1.0) &&
                (std::abs(old_offset[i]) > scaledTolerance(1.0) ||
                 std::abs(old_velocity[i]) > scaledTolerance(1.0) ||
                 std::abs(old_acceleration[i]) > scaledTolerance(1.0))) {
                return true;
            }
        }
        return false;
    };
    const bool translation_loaded = block_loaded(0, 3);
    const bool rotation_loaded = block_loaded(3, 6);
    const bool translation_residual = block_residual(0, 3);
    const bool rotation_residual = block_residual(3, 6);
    const bool translation_recenter = config_.blockwise_release_recenter &&
        !translation_loaded && translation_residual;
    const bool rotation_recenter = config_.blockwise_release_recenter &&
        !rotation_loaded && rotation_residual;
    proposal.translation_recenter_deferred =
        config_.blockwise_release_recenter && translation_loaded &&
        quiet_axis_residual(0, 3);
    proposal.rotation_recenter_deferred =
        config_.blockwise_release_recenter && rotation_loaded &&
        quiet_axis_residual(3, 6);

    std::array<AxisMotionLimits, 6> axis_limits{};
    std::array<AxisState, 6> old_states{};
    std::array<JerkInterval, 6> jerk_intervals{};
    AxisValues unbounded_desired_jerk{};
    AxisValues governed_jerk{};
    std::array<bool, 6> recovering_axes{};
    std::array<bool, 6> loaded_return_braking_axes{};
    std::array<bool, 6> loaded_hold_deferred_axes{};

    for (std::size_t i = 0; i < 6; ++i) {
        if (enabled[i] == 0.0) continue;

        // The installed sensor reports the reaction wrench on the TCP.  A
        // positive correction must therefore oppose an excess measured
        // wrench. During a partial block release, retain damping but suppress
        // the quiet sibling's spring so it brakes and holds until the whole
        // translation/rotation block is released.
        const bool sibling_loaded = i < 3 ? translation_loaded : rotation_loaded;
        const bool defer_axis_spring = config_.blockwise_release_recenter &&
            sibling_loaded && std::abs(wrench_error[i]) <= scaledTolerance(1.0);
        const double effective_stiffness =
            defer_axis_spring ? 0.0 : config_.stiffness[i];
        const double raw_acceleration =
            (wrench_error[i] - config_.damping[i] * old_velocity[i] -
             effective_stiffness * old_offset[i]) /
            config_.virtual_mass[i];

        const bool translation = i < 3;
        const double acceleration_limit = translation
            ? config_.max_linear_acceleration_m_s2 * translation_dynamic_scale
            : config_.max_angular_acceleration_rad_s2;
        const double jerk_limit = translation
            ? config_.max_linear_jerk_m_s3 * translation_dynamic_scale
            : config_.max_angular_jerk_rad_s3;
        const double velocity_limit = translation
            ? config_.max_linear_velocity_m_s + translation_velocity_boost
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
        const double load_direction = std::copysign(1.0, wrench_error[i]);
        // Loaded hold also handles a contact that returns while the spring is
        // recentering.  Its feasibility oracle mirrors that existing return
        // motion and proves it can brake before crossing the zero-offset
        // boundary; no instantaneous velocity reversal is assumed.
        const bool loaded_same_direction =
            config_.stiffness[i] > scaledTolerance(1.0) &&
            std::abs(wrench_error[i]) > scaledTolerance(1.0) &&
            load_direction * old_state.offset >=
                -scaledTolerance(limits.offset);
        JerkInterval interval;
        bool recovering = false;
        bool loaded_hold_deferred = false;
        bool soft_feasible = feasibleJerkInterval(
            old_state, soft_limits, dt_sec, &interval
        );
        if (soft_feasible && loaded_same_direction) {
            JerkInterval loaded_interval = interval;
            if (tightenToLoadedHoldEnvelope(
                    old_state,
                    load_direction,
                    soft_limits,
                    dt_sec,
                    &loaded_interval
                )) {
                interval = loaded_interval;
            } else {
                JerkInterval hard_loaded_interval;
                const bool hard_motion_feasible = feasibleJerkInterval(
                    old_state, limits, dt_sec, &hard_loaded_interval
                );
                if (!hard_motion_feasible) {
                    proposal.reason =
                        "Cartesian axis state is outside the hard jerk-governed motion envelope";
                    return proposal;
                }
                if (tightenToLoadedHoldEnvelope(
                        old_state,
                        load_direction,
                        limits,
                        dt_sec,
                        &hard_loaded_interval
                    )) {
                    recovering = true;
                    interval = hard_loaded_interval;
                } else {
                    // A load can reappear so late in a stiffness-driven return
                    // that the current velocity/acceleration cannot be stopped
                    // before a small equilibrium crossing.  Likewise, noisy
                    // deadband transitions can leave acceleration that must
                    // pass through a brief return transient before a loaded
                    // hold is dynamically reachable.  Preserve the ordinary
                    // jerk-safe envelope and brake toward the loaded hold;
                    // faulting cannot make the physically unavoidable
                    // transient disappear.
                    loaded_hold_deferred = true;
                }
            }
        }
        if (!soft_feasible) {
            recovering = true;
            if (!feasibleJerkInterval(old_state, limits, dt_sec, &interval)) {
                proposal.reason =
                    "Cartesian axis state is outside the hard jerk-governed motion envelope";
                return proposal;
            }
            if (loaded_same_direction) {
                JerkInterval loaded_interval = interval;
                if (tightenToLoadedHoldEnvelope(
                        old_state,
                        load_direction,
                        limits,
                        dt_sec,
                        &loaded_interval
                    )) {
                    interval = loaded_interval;
                } else {
                    loaded_hold_deferred = true;
                }
            }
        }

        unbounded_desired_jerk[i] =
            (raw_acceleration - old_acceleration[i]) / dt_sec;
        const double desired_jerk = std::clamp(
            unbounded_desired_jerk[i],
            -limits.jerk,
            limits.jerk
        );
        const bool loaded_return_braking = loaded_same_direction &&
            load_direction * old_state.velocity <
                -2.0 * limits.jerk * dt_sec * dt_sec;
        const bool loaded_boundary_hold = loaded_same_direction &&
            (recovering || loaded_return_braking || loaded_hold_deferred);
        const double recovery_jerk = loaded_boundary_hold
            ? loadedBoundaryBrakingJerk(
                  old_state,
                  load_direction,
                  limits,
                  dt_sec
              )
            : recoveryJerk(old_state, soft_limits);
        governed_jerk[i] = std::clamp(
            (recovering || loaded_return_braking || loaded_hold_deferred)
                ? recovery_jerk
                : desired_jerk,
            interval.lower,
            interval.upper
        );
        axis_limits[i] = limits;
        old_states[i] = old_state;
        jerk_intervals[i] = interval;
        recovering_axes[i] = recovering;
        loaded_return_braking_axes[i] = loaded_return_braking;
        loaded_hold_deferred_axes[i] = loaded_hold_deferred;
    }

    // In a fully released block, scale the unconstrained jerk vector once for
    // all enabled axes. This retains the SMD return direction while still
    // intersecting every axis' recursively feasible jerk interval. A hard-
    // envelope recovery jerk is authoritative, however: replacing it with the
    // common SMD scale can make the next state recursively infeasible and fault
    // one tick later. Resume coupling only after every axis is back inside its
    // soft envelope. Translation and rotation remain separate blocks because
    // their units and configured dynamics are intentionally different.
    const auto couple_recenter = [&](std::size_t begin, std::size_t end, bool active) {
        if (!active) return false;
        for (std::size_t i = begin; i < end; ++i) {
            if (enabled[i] != 0.0 && recovering_axes[i]) return false;
        }
        double shared_scale = 0.0;
        if (!sharedFeasibleJerkScale(
                begin,
                end,
                enabled,
                unbounded_desired_jerk,
                jerk_intervals,
                &shared_scale
            )) {
            return false;
        }
        for (std::size_t i = begin; i < end; ++i) {
            if (enabled[i] == 0.0) continue;
            governed_jerk[i] = shared_scale * unbounded_desired_jerk[i];
        }
        return true;
    };
    proposal.translation_recenter_coupled =
        couple_recenter(0, 3, translation_recenter);
    proposal.rotation_recenter_coupled =
        couple_recenter(3, 6, rotation_recenter);

    for (std::size_t i = 0; i < 6; ++i) {
        if (enabled[i] == 0.0) continue;
        const AxisMotionLimits& limits = axis_limits[i];
        const AxisState next_state = advanceWithJerk(
            old_states[i], governed_jerk[i], dt_sec
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
        const bool axis_limited = recovering_axes[i] ||
            loaded_return_braking_axes[i] || loaded_hold_deferred_axes[i] ||
            std::abs(governed_jerk[i] - unbounded_desired_jerk[i]) >
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
