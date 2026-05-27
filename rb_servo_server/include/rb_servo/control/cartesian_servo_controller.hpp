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

struct CartesianCircleMoveState {
    bool active = false;
    bool done = false;
    uint64_t seq = 0;
    double elapsed_sec = 0.0;
    double duration_sec = 0.0;
    bool lease_enforced = false;
    uint64_t lease_expires_time_ns = 0;
    TcpCircleMoveCommand command;
    Pose6D start_tcp_stand;
    Pose6D reference_tcp_stand;
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    double radius_m = 0.0;
    int axis1 = 0;
    int axis2 = 1;
};

struct CartesianTwistHoldState {
    bool orientation_hold_active = false;
    Pose6D hold_tcp_stand;
};

struct CartesianVelocityIntegratorState {
    bool valid = false;
    JointArray q_command_deg{};
    uint64_t last_seq = 0;
    ControlMode last_mode = ControlMode::Hold;
    uint64_t resets_total = 0;
    uint64_t clamps_total = 0;
    uint64_t divergence_total = 0;
    double max_command_actual_error_deg_observed = 0.0;
    std::string reset_reason;
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
        CartesianServoPathState* path_state,
        CartesianVelocityIntegratorState* velocity_integrator_state = nullptr
    );

    CartesianArmTargetResult computeTwistTarget(
        const ArmCommand& command,
        const RobotState& state,
        const JointArray& previous_safe_sent_q_deg,
        RunMode run_mode,
        double dt_sec,
        uint64_t command_seq,
        CartesianTwistHoldState* hold_state,
        CartesianVelocityIntegratorState* velocity_integrator_state = nullptr
    );

    CartesianArmTargetResult computeCircleMoveTarget(
        const ArmCommand& command,
        const RobotState& state,
        const JointArray& previous_safe_sent_q_deg,
        RunMode run_mode,
        double dt_sec,
        uint64_t command_seq,
        CartesianCircleMoveState* circle_state,
        CartesianVelocityIntegratorState* velocity_integrator_state = nullptr
    );

    void updateVelocityIntegratorAfterSafety(
        CartesianVelocityIntegratorState* velocity_integrator_state,
        const JointArray& safe_q_target_deg,
        bool was_sent_or_intended,
        bool target_was_clamped,
        const std::string& reset_reason
    );

private:
    ArmMountConfig left_mount_;
    ArmMountConfig right_mount_;
    CartesianControlConfig config_;
    std::shared_ptr<IKinematics> kinematics_;
};

}  // namespace rb_servo
