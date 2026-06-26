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
        const CartesianServoStateContext* state_context = nullptr
    );

    // Stand-frame floor plane metadata used by Cartesian safety telemetry.
    void setFloorConstraint(
        bool enabled,
        double z_min_m,
        double soft_margin_m,
        std::vector<FloorCheckPointConfig> tcp_offset_points = {}
    );

private:
    ArmMountConfig left_mount_;
    ArmMountConfig right_mount_;
    CartesianControlConfig config_;
    std::shared_ptr<IKinematics> kinematics_;
    bool floor_enabled_ = false;
    double floor_z_min_m_ = 0.0;
    double floor_soft_margin_m_ = 0.0;
    std::vector<FloorCheckPointConfig> floor_tcp_offset_points_;
};

}  // namespace rb_servo
