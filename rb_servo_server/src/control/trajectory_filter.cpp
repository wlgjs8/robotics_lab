#include "rb_servo/control/trajectory_filter.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {
namespace {

bool nearlyEqualJointArray(const JointArray& a, const JointArray& b, double tol_deg) {
    for (int i = 0; i < kDof; ++i) {
        if (!(std::abs(a[i] - b[i]) <= tol_deg)) {
            return false;
        }
    }
    return true;
}

}  // namespace

TrajectoryFilter::TrajectoryFilter(const ServoConfig& servo_config, const SafetyConfig& safety_config)
    : servo_config_(servo_config),
      safety_config_(safety_config),
      joint_smd_(safety_config.joint_target_smd) {}

JointArray TrajectoryFilter::computeJointTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_sent_target,
    double dt_sec
) {
    JointArray out{};
    switch (command.mode) {
        case ControlMode::Hold:
        case ControlMode::Idle:
        case ControlMode::ArmMotion:
        case ControlMode::DisarmMotion:
        case ControlMode::EmergencyStop:
        case ControlMode::ResetFault:
        case ControlMode::SetSafetyFloorZ:
            joint_smd_.deactivate();
            out = holdTarget(previous_sent_target);
            break;
        case ControlMode::JointTarget:
            out = safety_config_.joint_target_smd.enable
                ? smdJointTarget(command.q_target_deg, previous_sent_target, dt_sec)
                : filterJointTarget(command.q_target_deg, previous_sent_target, dt_sec);
            break;
        case ControlMode::JointVelocity:
            joint_smd_.deactivate();
            out = integrateJointVelocity(command.dq_target_deg_s, previous_sent_target, dt_sec);
            break;
        case ControlMode::TcpPoseTarget:
        case ControlMode::TcpLinearMove:
        case ControlMode::TcpCircleMove:
        case ControlMode::TcpCircleTrack:
        case ControlMode::TcpDeltaStand:
        case ControlMode::TcpDeltaLocal:
        case ControlMode::TcpTwistStand:
        case ControlMode::TcpTwistLocal:
            // Cartesian modes are intentionally deferred.
            joint_smd_.deactivate();
            out = holdTarget(previous_sent_target);
            break;
        default:
            joint_smd_.deactivate();
            out = holdTarget(previous_sent_target);
            break;
    }
    last_previous_sent_target_ = previous_sent_target;
    has_last_previous_sent_target_ = true;
    return out;
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

JointArray TrajectoryFilter::smdJointTarget(
    const JointArray& raw_target,
    const JointArray& previous_sent_target,
    double dt_sec
) {
    // (Re)activate when the profile is fresh OR when somebody else moved the
    // sent target since our last step (e.g. the Cartesian path ran in between
    // — those modes bypass this filter entirely, so deactivate() may not have
    // been called). The internal state must always continue exactly from the
    // caller's previous_sent_target to stay jump-free.
    constexpr double kContinuityTolDeg = 1e-6;
    if (!joint_smd_.active() ||
        !nearlyEqualJointArray(joint_smd_.position(), previous_sent_target, kContinuityTolDeg)) {
        JointArray dq0{};
        if (has_last_previous_sent_target_ && dt_sec > 0.0 && std::isfinite(dt_sec)) {
            for (int i = 0; i < kDof; ++i) {
                dq0[i] = (previous_sent_target[i] - last_previous_sent_target_[i]) / dt_sec;
            }
        }
        joint_smd_.reset(previous_sent_target, dq0);
    }
    joint_smd_.setGoal(raw_target);
    return joint_smd_.step(dt_sec);
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
