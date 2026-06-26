#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/joint_smd_tracker.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

class TrajectoryFilter {
public:
    TrajectoryFilter(const ServoConfig& servo_config, const SafetyConfig& safety_config);

    JointArray computeJointTarget(
        const ArmCommand& command,
        const RobotState& state,
        const JointArray& previous_sent_target,
        double dt_sec
    );

private:
    JointArray holdTarget(const JointArray& previous_sent_target) const;

    JointArray filterJointTarget(
        const JointArray& raw_target,
        const JointArray& previous_sent_target,
        double dt_sec
    ) const;

    // safety.joint_target_smd profile path: smooth second-order tracking of
    // the commanded goal instead of the raw full-speed ramp.
    JointArray smdJointTarget(
        const JointArray& raw_target,
        const JointArray& previous_sent_target,
        double dt_sec
    );

private:
    ServoConfig servo_config_;
    SafetyConfig safety_config_;
    JointSmdTracker joint_smd_;
    // previous_sent_target from the PREVIOUS call (any mode) — used to estimate
    // the joint velocity when the SMD profile activates mid-motion.
    JointArray last_previous_sent_target_{};
    bool has_last_previous_sent_target_ = false;
};

}  // namespace rb_servo
