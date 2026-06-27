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
            result.reason = tracking_state.override_tracking_q
                ? "reference tracking error exceeded threshold"
                : "tracking error exceeded threshold";
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
    for (int i = 0; i < kDof; ++i) {
        const double prev_vel = (q_prev[i] - q_prevprev[i]) / dt_sec;
        const double desired_vel = (q[i] - q_prev[i]) / dt_sec;
        const double max_dv = config_.ddq_max_deg_s2[i] * dt_sec;
        const double vel = prev_vel + std::clamp(desired_vel - prev_vel, -max_dv, max_dv);
        out[i] = q_prev[i] + vel * dt_sec;
        // Do not let acceleration limiting overshoot the already velocity-limited target.
        // This prevents a late tick or direction change from pushing past the commanded pose.
        if (q[i] >= q_prev[i]) {
            out[i] = std::min(out[i], q[i]);
        } else {
            out[i] = std::max(out[i], q[i]);
        }
    }
    return out;
}

}  // namespace rb_servo
