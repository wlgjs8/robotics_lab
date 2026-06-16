#pragma once

#include <memory>
#include <string>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/cartesian_controller.hpp"
#include "rb_servo/control/cartesian_trajectory_planner.hpp"
#include "rb_servo/control/smd_pose_tracker.hpp"
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
    // pose_track_smd smoothing of the per-tick path reference (same filter as
    // streaming TcpPoseTarget). Null when pose_track_smd.enable is false.
    std::shared_ptr<SmdPoseTracker> smd;
    // Previous commanded (post-filter) pose, for telemetry velocity norms.
    Pose6D last_commanded_tcp_stand;
    bool has_last_commanded = false;
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
    // twist_via_smd: running integrated pose goal + the SMD tracker. Created on
    // first use and reset to the current pose on lease/mode (re)entry (the whole
    // CartesianTwistHoldState is reset to {} then, clearing twist_smd to null).
    Pose6D twist_smd_goal{};
    std::shared_ptr<SmdPoseTracker> twist_smd;
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
    double command_reference_error_deg_observed = 0.0;
    double physical_command_actual_error_deg_observed = 0.0;
    std::string cartesian_servo_state_source = "actual";
    std::string cartesian_divergence_source = "actual";
    bool q_reference_for_servo_valid = false;
    std::string reset_reason;
};

struct CartesianServoStateContext {
    std::string servo_state_source = "actual";
    std::string divergence_source = "actual";
    bool q_reference_for_servo_valid = false;
    JointArray physical_q_actual_deg{};
    JointArray reference_q_deg{};
    JointArray divergence_q_deg{};
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
        CartesianVelocityIntegratorState* velocity_integrator_state = nullptr,
        const CartesianServoStateContext* state_context = nullptr
    );

    CartesianArmTargetResult computeTwistTarget(
        const ArmCommand& command,
        const RobotState& state,
        const JointArray& previous_safe_sent_q_deg,
        RunMode run_mode,
        double dt_sec,
        uint64_t command_seq,
        CartesianTwistHoldState* hold_state,
        CartesianVelocityIntegratorState* velocity_integrator_state = nullptr,
        const CartesianServoStateContext* state_context = nullptr
    );

    CartesianArmTargetResult computeCircleMoveTarget(
        const ArmCommand& command,
        const RobotState& state,
        const JointArray& previous_safe_sent_q_deg,
        RunMode run_mode,
        double dt_sec,
        uint64_t command_seq,
        CartesianCircleMoveState* circle_state,
        CartesianVelocityIntegratorState* velocity_integrator_state = nullptr,
        const CartesianServoStateContext* state_context = nullptr
    );

    void updateVelocityIntegratorAfterSafety(
        CartesianVelocityIntegratorState* velocity_integrator_state,
        const JointArray& safe_q_target_deg,
        bool was_sent_or_intended,
        bool target_was_clamped,
        const std::string& reset_reason
    );

    // Stand-frame floor plane (safety.floor_constraint Tier-2 assist): when the
    // commanded TCP is at/below z_min_m + soft_margin_m, the negative stand-frame
    // linear v_z of a streaming twist is zeroed so lateral motion slides along
    // the plane instead of stuttering against the Tier-1 joint-level hold.
    void setFloorConstraint(bool enabled, double z_min_m, double soft_margin_m);

private:
    ArmMountConfig left_mount_;
    ArmMountConfig right_mount_;
    CartesianControlConfig config_;
    std::shared_ptr<IKinematics> kinematics_;
    bool floor_enabled_ = false;
    double floor_z_min_m_ = 0.0;
    double floor_soft_margin_m_ = 0.0;
};

}  // namespace rb_servo
