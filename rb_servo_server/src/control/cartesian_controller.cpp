#include "rb_servo/control/cartesian_controller.hpp"

#include <stdexcept>

namespace rb_servo {

CartesianController::CartesianController(
    const ArmMountConfig& left_mount,
    const ArmMountConfig& right_mount
) : left_mount_(left_mount), right_mount_(right_mount) {}

JointArray CartesianController::computeArmJointTarget(
    const ArmCommand& command,
    const RobotState& state
) {
    (void)command;
    (void)state;
    // TODO: implement FK/IK based TCP control after joint servo_j is stable.
    throw std::runtime_error("CartesianController is not implemented yet");
}

JointArray CartesianController::solveIkFromTcpStandTarget(
    ArmId arm_id,
    const Pose6D& target_tcp_stand,
    const RobotState& state
) {
    (void)arm_id;
    (void)target_tcp_stand;
    (void)state;
    throw std::runtime_error("IK is not implemented yet");
}

Pose6D CartesianController::applyTcpDeltaStand(
    const Pose6D& current_tcp_stand,
    const Pose6D& delta
) const {
    Pose6D out = current_tcp_stand;
    out.x += delta.x;
    out.y += delta.y;
    out.z += delta.z;
    out.rx += delta.rx;
    out.ry += delta.ry;
    out.rz += delta.rz;
    return out;
}

Pose6D CartesianController::applyTcpDeltaLocal(
    const Pose6D& current_tcp_stand,
    const Pose6D& delta
) const {
    // TODO: apply delta in TCP local frame using SE(3), not simple addition.
    return applyTcpDeltaStand(current_tcp_stand, delta);
}

}  // namespace rb_servo
