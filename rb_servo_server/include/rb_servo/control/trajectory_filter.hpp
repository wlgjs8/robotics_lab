#pragma once

#include "rb_servo/config/config.hpp"
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

    JointArray integrateJointVelocity(
        const JointArray& dq_target_deg_s,
        const JointArray& previous_sent_target,
        double dt_sec
    ) const;

private:
    ServoConfig servo_config_;
    SafetyConfig safety_config_;
};

}  // namespace rb_servo
