#include "rb_servo/control/trajectory_filter.hpp"

#include <algorithm>

namespace rb_servo {

TrajectoryFilter::TrajectoryFilter(const ServoConfig& servo_config, const SafetyConfig& safety_config)
    : servo_config_(servo_config), safety_config_(safety_config) {}

JointArray TrajectoryFilter::computeJointTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_sent_target,
    double dt_sec
) {
    switch (command.mode) {
        case ControlMode::Hold:
        case ControlMode::Idle:
        case ControlMode::ArmMotion:
        case ControlMode::DisarmMotion:
        case ControlMode::EmergencyStop:
        case ControlMode::ResetFault:
            return holdTarget(previous_sent_target);
        case ControlMode::JointTarget:
            return filterJointTarget(command.q_target_deg, previous_sent_target, dt_sec);
        case ControlMode::JointVelocity:
            return integrateJointVelocity(command.dq_target_deg_s, previous_sent_target, dt_sec);
        case ControlMode::TcpPoseTarget:
        case ControlMode::TcpDeltaStand:
        case ControlMode::TcpDeltaLocal:
            // Cartesian modes are intentionally deferred.
            return holdTarget(previous_sent_target);
    }
    return holdTarget(previous_sent_target);
}

JointArray TrajectoryFilter::holdTarget(const JointArray& previous_sent_target) const {
    return previous_sent_target;
}

JointArray TrajectoryFilter::filterJointTarget(
    const JointArray& raw_target,
    const JointArray& previous_sent_target,
    double dt_sec
) const {
    JointArray out = previous_sent_target;
    for (int i = 0; i < kDof; ++i) {
        const double max_step = safety_config_.dq_max_deg_s[i] * dt_sec;
        const double delta = std::clamp(raw_target[i] - previous_sent_target[i], -max_step, max_step);
        out[i] = previous_sent_target[i] + delta;
    }
    return out;
}

JointArray TrajectoryFilter::integrateJointVelocity(
    const JointArray& dq_target_deg_s,
    const JointArray& previous_sent_target,
    double dt_sec
) const {
    JointArray out = previous_sent_target;
    for (int i = 0; i < kDof; ++i) {
        const double dq = std::clamp(
            dq_target_deg_s[i],
            -safety_config_.dq_max_deg_s[i],
            safety_config_.dq_max_deg_s[i]
        );
        out[i] = previous_sent_target[i] + dq * dt_sec;
    }
    return out;
}

}  // namespace rb_servo
