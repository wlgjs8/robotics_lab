#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

struct SafetyCheckResult {
    JointArray filtered_q_deg{};
    SafetyVerdict verdict = SafetyVerdict::Ok;
    bool ok = true;
    bool joint_limit_clamped = false;
};

class SafetyFilter {
public:
    explicit SafetyFilter(const SafetyConfig& config);

    SafetyCheckResult filterJointTarget(
        const JointArray& desired_q_deg,
        const JointArray& previous_q_deg,
        const JointArray& previous_previous_q_deg,
        const RobotState& state,
        double dt_sec
    ) const;

    SafetyVerdict checkState(const RobotState& state) const;
    bool hasTrackingError(const JointArray& previous_q_deg, const RobotState& state) const;

    bool isStateSafe(const RobotState& state) const;

    bool shouldStopBothArms(
        const RobotState& left_state,
        const RobotState& right_state
    ) const;

private:
    JointArray clampJointLimits(const JointArray& q, bool* clamped) const;

    JointArray clampVelocity(
        const JointArray& q,
        const JointArray& q_prev,
        double dt_sec
    ) const;

    JointArray clampAcceleration(
        const JointArray& q,
        const JointArray& q_prev,
        const JointArray& q_prevprev,
        double dt_sec
    ) const;

private:
    SafetyConfig config_;
};

}  // namespace rb_servo
