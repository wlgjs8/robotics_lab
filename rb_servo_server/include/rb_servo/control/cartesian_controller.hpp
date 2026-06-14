#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/cartesian_trajectory_planner.hpp"
#include "rb_servo/core/types.hpp"
#include "rb_servo/kinematics/i_kinematics.hpp"

#include <memory>
#include <string>

namespace rb_servo {

struct CartesianArmTargetResult {
    SafetyVerdict verdict = SafetyVerdict::CartesianUnavailable;
    JointArray q_target_deg{};
    std::string reason;
    CartesianSolveTelemetry telemetry;
};

// Shared position-IK step: solve full IK toward target_tcp_stand SEEDED FROM
// THE PREVIOUSLY SENT JOINTS (feedforward chain — see the relay-limit-cycle
// note in the implementation). Used by TcpPoseTarget (CartesianController) and
// by the path primitives in CartesianServoController (TcpLinearMove), so both
// share the same smooth, measurement-decoupled IK behavior.
CartesianArmTargetResult solveIkArmTargetFromTcpStand(
    IKinematics& kinematics,
    const CartesianControlConfig& config,
    const ArmMountConfig& mount,
    ArmId arm_id,
    const Pose6D& target_tcp_stand,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode
);

class CartesianController {
public:
    CartesianController(
        const ArmMountConfig& left_mount,
        const ArmMountConfig& right_mount,
        const CartesianControlConfig& config,
        std::shared_ptr<IKinematics> kinematics
    );

    CartesianArmTargetResult computeArmJointTarget(
        const ArmCommand& command,
        const RobotState& state,
        const JointArray& previous_safe_sent_q_deg,
        RunMode run_mode
    );

    Pose6D applyTcpDeltaStand(
        const Pose6D& current_tcp_stand,
        const Pose6D& delta
    ) const;

    Pose6D applyTcpDeltaLocal(
        const Pose6D& current_tcp_stand,
        const Pose6D& delta
    ) const;

private:
    CartesianArmTargetResult solveIkFromTcpStandTarget(
        ArmId arm_id,
        const Pose6D& target_tcp_stand,
        const RobotState& state,
        const JointArray& previous_safe_sent_q_deg,
        RunMode run_mode
    );

private:
    ArmMountConfig left_mount_;
    ArmMountConfig right_mount_;
    CartesianControlConfig config_;
    std::shared_ptr<IKinematics> kinematics_;
};

}  // namespace rb_servo
