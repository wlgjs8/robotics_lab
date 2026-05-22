#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

class CartesianController {
public:
    CartesianController(
        const ArmMountConfig& left_mount,
        const ArmMountConfig& right_mount
    );

    JointArray computeArmJointTarget(
        const ArmCommand& command,
        const RobotState& state
    );

private:
    JointArray solveIkFromTcpStandTarget(
        ArmId arm_id,
        const Pose6D& target_tcp_stand,
        const RobotState& state
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
    ArmMountConfig left_mount_;
    ArmMountConfig right_mount_;
};

}  // namespace rb_servo
