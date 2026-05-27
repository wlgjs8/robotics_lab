#include "rb_servo/control/safety_filter.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {

SafetyFilter::SafetyFilter(const SafetyConfig& config) : config_(config) {}

SafetyCheckResult SafetyFilter::filterJointTarget(
    const JointArray& desired_q_deg,
    const JointArray& previous_q_deg,
    const JointArray& previous_previous_q_deg,
    const RobotState& state,
    double dt_sec
) const {
    SafetyCheckResult result;

    const SafetyVerdict state_verdict = checkState(state);
    if (state_verdict != SafetyVerdict::Ok) {
        result.ok = false;
        result.verdict = state_verdict;
        result.filtered_q_deg = previous_q_deg;
        return result;
    }

    if (hasTrackingError(previous_q_deg, state)) {
        result.ok = false;
        result.verdict = SafetyVerdict::TrackingError;
        result.filtered_q_deg = previous_q_deg;
        return result;
    }

    bool clamped = false;
    JointArray out = desired_q_deg;
    out = clampJointLimits(out, &clamped);
    out = clampVelocity(out, previous_q_deg, dt_sec);
    out = clampAcceleration(out, previous_q_deg, previous_previous_q_deg, dt_sec);

    result.filtered_q_deg = out;
    result.joint_limit_clamped = clamped;
    result.verdict = clamped ? SafetyVerdict::JointLimitClamped : SafetyVerdict::Ok;
    result.ok = true;
    return result;
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
    const RobotState& state
) const {
    for (int i = 0; i < kDof; ++i) {
        if (std::abs(previous_q_deg[i] - state.q_actual_deg[i]) > config_.max_tracking_error_deg) {
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
