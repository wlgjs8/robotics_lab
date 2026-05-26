#pragma once

#include <memory>
#include <string>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/cartesian_controller.hpp"
#include "rb_servo/control/cartesian_trajectory_planner.hpp"
#include "rb_servo/core/types.hpp"
#include "rb_servo/kinematics/i_kinematics.hpp"

namespace rb_servo {

struct CartesianServoPathState {
    bool active = false;
    bool done = false;
    uint64_t seq = 0;
    double elapsed_sec = 0.0;
    double duration_sec = 0.0;
    bool lease_enforced = false;
    uint64_t lease_expires_time_ns = 0;
    Pose6D start_tcp_stand;
    Pose6D target_tcp_stand;
    CartesianOrientationInterpolation orientation_mode = CartesianOrientationInterpolation::Constant;
};

struct CartesianTwistHoldState {
    bool orientation_hold_active = false;
    Pose6D hold_tcp_stand;
};

class CartesianServoController {
public:
    CartesianServoController(
        const ArmMountConfig& left_mount,
        const ArmMountConfig& right_mount,
        const CartesianControlConfig& config,
        std::shared_ptr<IKinematics> kinematics
    );

    CartesianArmTargetResult computeLinearMoveTarget(
        const ArmCommand& command,
        const RobotState& state,
        const JointArray& previous_safe_sent_q_deg,
        RunMode run_mode,
        double dt_sec,
        uint64_t command_seq,
        CartesianServoPathState* path_state
    );

    CartesianArmTargetResult computeTwistTarget(
        const ArmCommand& command,
        const RobotState& state,
        const JointArray& previous_safe_sent_q_deg,
        RunMode run_mode,
        double dt_sec,
        CartesianTwistHoldState* hold_state
    );

private:
    ArmMountConfig left_mount_;
    ArmMountConfig right_mount_;
    CartesianControlConfig config_;
    std::shared_ptr<IKinematics> kinematics_;
};

}  // namespace rb_servo
