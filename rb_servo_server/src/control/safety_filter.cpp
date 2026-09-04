#include <cstdio>
#include "rb_servo/control/safety_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rb_servo {
namespace {

double maxAbsJointError(const JointArray& a, const JointArray& b) {
    double max_error = 0.0;
    for (int i = 0; i < kDof; ++i) {
        if (!std::isfinite(a[i]) || !std::isfinite(b[i])) {
            return std::numeric_limits<double>::infinity();
        }
        max_error = std::max(max_error, std::abs(a[i] - b[i]));
    }
    return max_error;
}

SafetyTrackingTelemetry makeTrackingTelemetry(
    const JointArray& previous_q_deg,
    const RobotState& state,
    const SafetyTrackingState& tracking_state
) {
    SafetyTrackingTelemetry telemetry;
    telemetry.tracking_error_source = tracking_state.source.empty()
        ? "actual"
        : tracking_state.source;
    telemetry.tracking_error_source_valid = tracking_state.source_valid;
    telemetry.tracking_error_reason = tracking_state.reason;
    telemetry.physical_command_actual_error_deg =
        maxAbsJointError(previous_q_deg, state.q_actual_deg);
    telemetry.controller_simulation_physical_motion_detected =
        tracking_state.controller_simulation_physical_motion_detected;
    telemetry.command_reference_tracking_error_deg = tracking_state.override_tracking_q
        ? maxAbsJointError(previous_q_deg, tracking_state.tracking_q_deg)
        : telemetry.physical_command_actual_error_deg;

    // THE TWO ERRORS, SEPARATED, because they blame different subsystems and the
    // latch can only report one number. `q_target_deg` is the BOX's own reference
    // (rbpodo sdata.jnt_ref) — comparing it against the measured joints is
    // controller-manager's `TrackingError`, a physical-anomaly check independent of
    // whatever we asked for. Comparing OUR command against the measured joints is
    // CM's `JointDeviation`, and it also goes large when the box simply stops
    // consuming what we send.
    telemetry.command_vs_actual_deg = telemetry.physical_command_actual_error_deg;
    telemetry.reference_valid = state.q_ref_valid;
    telemetry.reference_vs_actual_deg = state.q_ref_valid
        ? maxAbsJointError(state.q_target_deg, state.q_actual_deg)
        : 0.0;
    return telemetry;
}

void recordStageDelta(
    const JointArray& before,
    const JointArray& after,
    bool* clamped,
    double* max_delta_deg,
    int* limited_joint
) {
    *clamped = false;
    *max_delta_deg = 0.0;
    *limited_joint = -1;
    for (int i = 0; i < kDof; ++i) {
        const double delta = std::abs(after[i] - before[i]);
        if (delta > *max_delta_deg) {
            *max_delta_deg = delta;
            *limited_joint = i;
        }
    }
    constexpr double kClampEpsilonDeg = 1e-12;
    *clamped = *max_delta_deg > kClampEpsilonDeg;
    if (!*clamped) {
        *limited_joint = -1;
    }
}

}  // namespace

SafetyFilter::SafetyFilter(const SafetyConfig& config) : config_(config) {}

SafetyCheckResult SafetyFilter::filterJointTarget(
    const JointArray& desired_q_deg,
    const JointArray& previous_q_deg,
    const JointArray& previous_previous_q_deg,
    const RobotState& state,
    double dt_sec,
    const SafetyTrackingState& tracking_state
) const {
    SafetyCheckResult result;
    result.tracking = makeTrackingTelemetry(previous_q_deg, state, tracking_state);

    if (!tracking_state.source_valid) {
        result.ok = false;
        result.verdict = SafetyVerdict::RobotStateError;
        result.filtered_q_deg = previous_q_deg;
        result.reason = tracking_state.reason.empty()
            ? "controller_simulation_reference_state_unavailable"
            : tracking_state.reason;
        result.tracking.tracking_error_reason = result.reason;
        return result;
    }

    if (tracking_state.controller_simulation_physical_motion_fault) {
        result.ok = false;
        result.verdict = SafetyVerdict::TrackingError;
        result.filtered_q_deg = previous_q_deg;
        result.reason = tracking_state.reason.empty()
            ? "controller_simulation_physical_motion_detected"
            : tracking_state.reason;
        result.tracking.tracking_error_reason = result.reason;
        return result;
    }

    const SafetyVerdict state_verdict = checkState(state);
    if (state_verdict != SafetyVerdict::Ok) {
        result.ok = false;
        result.verdict = state_verdict;
        result.filtered_q_deg = previous_q_deg;
        result.reason = "robot state error or disconnected";
        return result;
    }

    if (hasTrackingError(previous_q_deg, state, tracking_state)) {
        if (!tracking_state.tracking_error_advisory) {
            result.ok = false;
            result.verdict = SafetyVerdict::TrackingError;
            result.filtered_q_deg = previous_q_deg;
            // NAME THE SUBSYSTEM, not just the symptom. The same threshold is
            // crossed by two very different failures, and the one that fired on
            // 2026-08-26 said "tracking error" while the arm was following its own
            // reference to 0.00 deg — the arm was fine and the BOX had stopped
            // taking our commands. Both numbers go in the message so the reader
            // does not have to open a CSV to tell them apart.
            const double cmd_err = result.tracking.command_vs_actual_deg;
            const double ref_err = result.tracking.reference_vs_actual_deg;
            const bool arm_is_following =
                result.tracking.reference_valid &&
                ref_err < config_.max_tracking_error_deg * 0.25;
            char detail[224];
            if (tracking_state.override_tracking_q) {
                result.tracking.latch_cause = "reference";
                std::snprintf(detail, sizeof(detail),
                              "reference tracking error exceeded threshold "
                              "(command-vs-actual %.2f deg, limit %.2f)",
                              cmd_err, config_.max_tracking_error_deg);
            } else if (arm_is_following) {
                // The arm tracks its own reference but not our command: the
                // COMMAND LINK is the problem, not the servo.
                result.tracking.latch_cause = "command_not_executed";
                std::snprintf(detail, sizeof(detail),
                              "the controller is NOT executing our commands: "
                              "command-vs-actual %.2f deg (limit %.2f) while the box's "
                              "own reference tracks the arm to %.2f deg - check the "
                              "servo stream (queue sync hold, send policy), not the arm",
                              cmd_err, config_.max_tracking_error_deg, ref_err);
            } else {
                result.tracking.latch_cause = "arm_not_following";
                std::snprintf(detail, sizeof(detail),
                              "the ARM is not following its own controller reference: "
                              "reference-vs-actual %.2f deg, command-vs-actual %.2f deg "
                              "(limit %.2f) - collision, overload or servo fault",
                              ref_err, cmd_err, config_.max_tracking_error_deg);
            }
            result.reason = detail;
            result.tracking.tracking_error_reason = result.reason;
            return result;
        }
        // controller-simulation advisory: the sim controller's reported jnt_ref does
        // not advance while its servo is disabled, so do NOT snap the streaming
        // command back to previous_q (that pins Cartesian motion after
        // max_tracking_error_deg). Report it and let the command accumulate. The
        // genuine controller_simulation_physical_motion_fault is handled above.
        result.tracking.tracking_error_reason =
            "reference tracking error (advisory, controller-simulation)";
    }

    result.clamp = clampMotionDetailed(
        desired_q_deg,
        previous_q_deg,
        previous_previous_q_deg,
        dt_sec
    );
    result.filtered_q_deg = result.clamp.q_after_accel_limit_deg;
    result.joint_limit_clamped = result.clamp.joint_limit_clamped;
    result.verdict = result.joint_limit_clamped ? SafetyVerdict::JointLimitClamped : SafetyVerdict::Ok;
    result.ok = true;
    return result;
}

JointArray SafetyFilter::clampMotion(
    const JointArray& desired_q_deg,
    const JointArray& previous_q_deg,
    const JointArray& previous_previous_q_deg,
    double dt_sec,
    bool* clamped
) const {
    const SafetyClampTelemetry telemetry = clampMotionDetailed(
        desired_q_deg,
        previous_q_deg,
        previous_previous_q_deg,
        dt_sec
    );
    if (clamped) *clamped = telemetry.joint_limit_clamped;
    return telemetry.q_after_accel_limit_deg;
}

SafetyClampTelemetry SafetyFilter::clampMotionDetailed(
    const JointArray& desired_q_deg,
    const JointArray& previous_q_deg,
    const JointArray& previous_previous_q_deg,
    double dt_sec
) const {
    SafetyClampTelemetry telemetry;
    telemetry.present = true;
    telemetry.q_before_safety_deg = desired_q_deg;
    telemetry.q_after_joint_limit_deg = clampJointLimits(desired_q_deg, nullptr);
    // Approach barrier runs INSIDE the joint-limit stage: it shapes how the command
    // reaches a bound, so its correction is reported as part of the joint-limit clamp
    // rather than as a new stage (keeps the existing 3-stage telemetry contract).
    telemetry.q_after_joint_limit_deg = applyJointLimitBarrier(
        telemetry.q_after_joint_limit_deg,
        previous_q_deg,
        dt_sec
    );
    telemetry.q_after_velocity_limit_deg = clampVelocity(
        telemetry.q_after_joint_limit_deg,
        previous_q_deg,
        dt_sec
    );
    telemetry.q_after_accel_limit_deg = clampAcceleration(
        telemetry.q_after_velocity_limit_deg,
        previous_q_deg,
        previous_previous_q_deg,
        dt_sec
    );
    recordStageDelta(
        telemetry.q_before_safety_deg,
        telemetry.q_after_joint_limit_deg,
        &telemetry.joint_limit_clamped,
        &telemetry.joint_limit_clamp_max_delta_deg,
        &telemetry.joint_limit_limited_joint
    );
    recordStageDelta(
        telemetry.q_after_joint_limit_deg,
        telemetry.q_after_velocity_limit_deg,
        &telemetry.velocity_clamped,
        &telemetry.velocity_clamp_max_delta_deg,
        &telemetry.velocity_limited_joint
    );
    recordStageDelta(
        telemetry.q_after_velocity_limit_deg,
        telemetry.q_after_accel_limit_deg,
        &telemetry.accel_clamped,
        &telemetry.accel_clamp_max_delta_deg,
        &telemetry.accel_limited_joint
    );
    return telemetry;
}

SafetyVerdict SafetyFilter::checkState(const RobotState& state) const {
    if (state.has_error) {
        return SafetyVerdict::RobotStateError;
    }
    if (state.connection_state == RobotConnectionState::Error ||
        state.connection_state == RobotConnectionState::Disconnected) {
        return SafetyVerdict::RobotStateError;
    }
    return SafetyVerdict::Ok;
}

bool SafetyFilter::hasTrackingError(
    const JointArray& previous_q_deg,
    const RobotState& state,
    const SafetyTrackingState& tracking_state
) const {
    const JointArray& observed_q_deg = tracking_state.override_tracking_q
        ? tracking_state.tracking_q_deg
        : state.q_actual_deg;
    for (int i = 0; i < kDof; ++i) {
        if (!std::isfinite(previous_q_deg[i]) || !std::isfinite(observed_q_deg[i])) {
            return true;
        }
        if (std::abs(previous_q_deg[i] - observed_q_deg[i]) > config_.max_tracking_error_deg) {
            return true;
        }
    }
    return false;
}

bool SafetyFilter::isStateSafe(const RobotState& state) const {
    return checkState(state) == SafetyVerdict::Ok;
}

bool SafetyFilter::shouldStopBothArms(
    const RobotState& left_state,
    const RobotState& right_state
) const {
    if (!config_.stop_both_arms_on_single_arm_error) {
        return false;
    }
    return !isStateSafe(left_state) || !isStateSafe(right_state);
}

JointArray SafetyFilter::clampJointLimits(const JointArray& q, bool* clamped) const {
    JointArray out = q;
    bool did_clamp = false;
    for (int i = 0; i < kDof; ++i) {
        const double before = out[i];
        // Command targets are intentionally not wrapped here; raw targets clamp
        // conservatively until continuous motion-safe unwrapping is implemented.
        out[i] = std::clamp(out[i], config_.q_min_deg[i], config_.q_max_deg[i]);
        did_clamp = did_clamp || (out[i] != before);
    }
    if (clamped) *clamped = did_clamp;
    return out;
}

JointArray SafetyFilter::applyJointLimitBarrier(
    const JointArray& q,
    const JointArray& q_prev,
    double dt_sec
) const {
    JointArray out = q;
    const JointLimitBarrierConfig& cfg = config_.joint_limit_barrier;
    if (!cfg.enable || dt_sec <= 0.0) return out;
    for (int i = 0; i < kDof; ++i) {
        const double lo = cfg.inherit_bounds ? config_.q_min_deg[i] : cfg.q_min_deg[i];
        const double hi = cfg.inherit_bounds ? config_.q_max_deg[i] : cfg.q_max_deg[i];
        const double band = cfg.d_slow_deg[i];
        const double a_brake = cfg.a_brake_deg_s2[i];
        if (!(band > 0.0) || !(a_brake > 0.0) || !(hi > lo)) continue;
        const double step = out[i] - q_prev[i];
        if (step == 0.0) continue;
        // Brake onto (bound - standoff), not onto the bound. A joint parked exactly
        // on its bound has its servo holding against the stop, and that rings: the
        // left elbow sat at 150.000 deg for 8 s while the encoder oscillated at
        // 17 Hz (2026-08-27). 0.057 deg of clearance was enough to remove the peak
        // entirely. The hard clamp still owns the bound; this only decides where the
        // barrier brings the command to rest.
        const double standoff = std::max(0.0, cfg.standoff_deg[i]);
        const double hi_eff = hi - standoff;
        const double lo_eff = lo + standoff;
        if (!(hi_eff > lo_eff)) continue;   // standoff wider than the range: inert
        // Only the direction that CLOSES on a bound is limited; retreating is free, so
        // the arm can always be commanded back out of the band -- including from
        // inside the standoff, where the closing margin below goes negative.
        const double margin = step > 0.0 ? hi_eff - q_prev[i] : q_prev[i] - lo_eff;
        if (!(margin < band)) continue;              // outside the engage band
        // The continuous braking law sqrt(2*a*margin) overshoots when discretised: the
        // step it permits exceeds the remaining margin once margin < 2*a*dt^2 (0.012 deg
        // at a=1500, dt=2 ms). Clamp to the margin itself so the joint asymptotes onto
        // the bound instead of stepping across it.
        const double allowed = margin > 0.0
            ? std::min(margin, std::sqrt(2.0 * a_brake * margin) * dt_sec)
            : 0.0;                                    // at/past the bound: no closing
        const double closing = std::abs(step);
        if (closing <= allowed) continue;
        out[i] = q_prev[i] + (step > 0.0 ? allowed : -allowed);
    }
    return out;
}

JointArray SafetyFilter::clampVelocity(
    const JointArray& q,
    const JointArray& q_prev,
    double dt_sec
) const {
    JointArray out = q;
    if (dt_sec <= 0.0) return q_prev;
    for (int i = 0; i < kDof; ++i) {
        const double max_step = config_.dq_max_deg_s[i] * dt_sec;
        out[i] = q_prev[i] + std::clamp(q[i] - q_prev[i], -max_step, max_step);
    }
    return out;
}

JointArray SafetyFilter::clampAcceleration(
    const JointArray& q,
    const JointArray& q_prev,
    const JointArray& q_prevprev,
    double dt_sec
) const {
    JointArray out = q;
    if (dt_sec <= 0.0) return q_prev;
    // Deceleration is allowed to be `ddq_max_decel_ratio` times harsher than
    // acceleration; the anti-overshoot guard below then lets the command lead the
    // target by at most `decel_overshoot_budget_deg` while that ramp runs.
    //
    // With ratio == 1.0 AND budget == 0.0 this is byte-identical to the legacy
    // filter, which bounded acceleration but NOT deceleration: the guard clipped the
    // accel-limited output straight back to the target, so a target that stopped hard
    // was passed through at unbounded jerk (measured -68,000 deg/s^2 against a
    // 3,000 deg/s^2 ceiling on 2026-08-26). Bounding deceleration necessarily costs
    // some lead -- the budget is what caps it.
    const double decel_ratio = std::max(1.0, config_.ddq_max_decel_ratio);
    const double overshoot_budget = std::max(0.0, config_.decel_overshoot_budget_deg);
    for (int i = 0; i < kDof; ++i) {
        const double prev_vel = (q_prev[i] - q_prevprev[i]) / dt_sec;
        const double desired_vel = (q[i] - q_prev[i]) / dt_sec;
        const double max_dv = config_.ddq_max_deg_s2[i] * dt_sec;
        // Shedding speed is a deceleration and gets the wider budget; building speed
        // keeps ddq_max. A sign reversal counts even at equal magnitude (+100 -> -100
        // still has to decelerate through zero first), which |desired| < |prev| alone
        // would miss. Starting from rest (prev_vel == 0) is acceleration, not shedding.
        const bool shedding_speed = std::abs(desired_vel) < std::abs(prev_vel) ||
                                    desired_vel * prev_vel < 0.0;
        const double dv_limit = shedding_speed ? max_dv * decel_ratio : max_dv;
        double vel = prev_vel + std::clamp(desired_vel - prev_vel, -dv_limit, dv_limit);
        // THE REVERSAL RULE (2026-09-04). The wide budget is for SHEDDING speed. A
        // reversal that crosses zero inside one tick used to keep the whole
        // decel_ratio*max_dv step, so the part of it that BUILDS speed in the new
        // direction was 4x what a start from rest is allowed. Every 9-12k deg/s^2
        // kick measured on 2026-09-04 was exactly ddq_max x decel_ratio on a reversal
        // tick (InitMotion resume 12,021 = 3000x4 on J6; ROI-face entry 9,385; the
        // wrist-singularity branch crawl 9,222-11,498 = 2300x4 / 3000x4). Shed to
        // zero at the wide budget, then build the opposite direction at ddq_max.
        if (prev_vel != 0.0 && vel * prev_vel < 0.0) {
            vel = std::copysign(std::min(std::abs(vel), max_dv), vel);
        }
        out[i] = q_prev[i] + vel * dt_sec;
        // Bounded anti-overshoot. `dir` is the direction from the previous command to
        // this tick's target; `lead` > 0 means the decel-limited output would land PAST
        // the target. Legacy behaviour was lead <= 0 (clip straight back to the target,
        // i.e. unbounded deceleration); the budget is how far past the target the
        // bounded-deceleration ramp is allowed to sit.
        //
        // Scope, stated honestly: this caps the lead measured against the CURRENT
        // target each tick. It does not cap the total excursion if the target stops dead
        // and the command coasts past it -- that excursion is v^2 / (2 * decel_ratio *
        // ddq_max), which is what bounding deceleration costs by construction. Size
        // decel_ratio against the worst commanded speed with that formula.
        const double dir = (q[i] >= q_prev[i]) ? 1.0 : -1.0;
        const double lead = (out[i] - q[i]) * dir;
        if (lead > overshoot_budget) {
            if (decel_ratio <= 1.0 && overshoot_budget <= 0.0) {
                // Legacy contract (ratio 1.0 + budget 0.0 == byte-identical old
                // filter): clip straight back to the target, i.e. deceleration
                // is unbounded. Kept so configs that never opted into bounded
                // deceleration are unchanged.
                out[i] = q[i];
            } else {
                // Rate-limited clip, NOT an assignment. The previous assignment
                // (`out = q +/- budget`) was a teleport: up to `budget` deg in
                // one 2 ms tick (0.5 deg = ~125,000 deg/s^2), bypassing every
                // ceiling above, and on a hovering target `dir` flips sign
                // tick-to-tick so the clip anchor alternated between q+budget
                // and q-budget -- a loop-rate square wave exactly when a
                // barrier had pinned the arm. Move TOWARD the budget boundary
                // at no more than the decel dv budget instead: the overshoot
                // cap is reached over a few ticks (jerk stays bounded) and the
                // hovering-target chatter amplitude collapses from +/-budget
                // to +/-dv_limit*dt.
                const double clip_target = q[i] + dir * overshoot_budget;
                const double clip_vel = (clip_target - q_prev[i]) / dt_sec;
                const double bounded_vel =
                    prev_vel + std::clamp(clip_vel - prev_vel, -dv_limit, dv_limit);
                // Never let the "clip" move the output FURTHER past the target
                // than the unclipped decel ramp already did.
                const double candidate = q_prev[i] + bounded_vel * dt_sec;
                if ((candidate - q[i]) * dir < lead) {
                    out[i] = candidate;
                }
            }
        }
        // A decel-bounded coast past a target sitting at a joint limit must not
        // carry the COMMAND past the hard limit (the legacy assignment clip
        // could, by up to the budget; clampJointLimits ran before this stage
        // and is not re-applied). Guarded on a non-degenerate range so callers
        // with a default-constructed config (all-zero limits) are unaffected.
        if (config_.q_min_deg[i] < config_.q_max_deg[i]) {
            out[i] = std::clamp(out[i], config_.q_min_deg[i], config_.q_max_deg[i]);
        }
    }
    return out;
}

}  // namespace rb_servo
