#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/arm_worker.hpp"
#include "rb_servo/control/cartesian_servo_controller.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/joint_moving_average.hpp"
#include "rb_servo/control/smd_pose_tracker.hpp"
#include "rb_servo/control/self_collision.hpp"
#include "rb_servo/control/floor_constraint.hpp"
#include "rb_servo/control/fault_classifier.hpp"
#include "rb_servo/control/safety_filter.hpp"
#include "rb_servo/control/trajectory_filter.hpp"
#include "rb_servo/kinematics/i_kinematics.hpp"
#include "rb_servo/logging/servo_logger.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

class DualArmServoLoop {
public:
    DualArmServoLoop(
        std::unique_ptr<IRobotBackend> left_robot,
        std::unique_ptr<IRobotBackend> right_robot,
        const DualArmConfig& config,
        CommandBuffer* command_buffer,
        ServoLogger* logger,
        std::shared_ptr<IKinematics> kinematics = nullptr
    );

    ~DualArmServoLoop();

    bool start();
    void stop();

    bool isRunning() const;
    ServerMotionState motionState() const;
    bool faultLatched() const;
    SafetyVerdict latchedFaultReason() const;
    ServoTarget previousSentTarget() const;
    ServoSnapshot latestSnapshot() const;

private:
    struct LatchedCartesianTarget {
        uint64_t seq = 0;
        Pose6D target_tcp_stand;
        bool valid = false;
    };

    void loopMain();
    bool configureRealtimeForLoop();

    bool initializeRobots();
    bool initializeWorkers();
    bool readRobotStates(RobotState& left, RobotState& right);
    void populateTcpPose(RobotState& state, const ArmMountConfig& mount) const;
    ArmStartupValidationSnapshot validateStartupArm(const RobotState& state) const;
    StartupValidationSnapshot validateStartupStates(
        const RobotState& left,
        const RobotState& right
    ) const;
    bool startupValidationAllowsStart(const StartupValidationSnapshot& validation) const;
    bool readOnlyDiagnosticStartupEnabled() const;
    bool initializeStartupTargets(
        const RobotState& left,
        const RobotState& right
    );
    void logStartupValidation(
        const StartupValidationSnapshot& validation,
        const RobotState& left,
        const RobotState& right
    ) const;
    void storeStartupValidation(const StartupValidationSnapshot& validation);
    bool isValidRobotStateForStartup(const RobotState& state) const;
    bool isValidJointState(const RobotState& state) const;
    void clearLatchedCartesianTargets();
    void clearLatchedCartesianTarget(ArmId arm_id);
    void resetCartesianVelocityIntegrator(ArmId arm_id, const std::string& reason);
    void refreshCartesianVelocityIntegratorTelemetry(ArmId arm_id);
    DualArmCommand resolveCartesianDeltaCommand(
        const DualArmCommand& command,
        const RobotState& left_state,
        const RobotState& right_state
    );
    ArmCommand resolveArmCartesianDeltaCommand(
        const ArmCommand& command,
        const RobotState& state,
        uint64_t command_seq,
        LatchedCartesianTarget& latch
    );

    ServoTarget computeServoTarget(
        const RobotState& left_state,
        const RobotState& right_state,
        const DualArmCommand& command,
        double dt_sec,
        SafetyVerdict* command_verdict
    );

    ServoTarget applySafety(
        const ServoTarget& desired,
        const RobotState& left_state,
        const RobotState& right_state,
        double dt_sec,
        SafetyVerdict* verdict
    );

    // Dual-arm self-collision clearance for candidate joint targets (uses the
    // configured kinematics + mounts + safety.self_collision). checked=false if
    // link geometry is unavailable.
    SelfCollisionResult evaluateSelfCollision(
        const JointArray& left_q_deg,
        const JointArray& right_q_deg
    ) const;

    // Stand-frame floor plane constraint (safety.floor_constraint): FK the arm's
    // TCP for a candidate joint target. checked=false if kinematics/FK is
    // unavailable — the caller fails closed.
    FloorArmEvaluation evaluateFloorArm(ArmId arm, const JointArray& q_deg) const;
    // Effective (runtime-adjustable) floor plane height in meters.
    double effectiveFloorZ() const;
    // Tier-2 usability clamp: project a Cartesian target's stand z onto the floor
    // plane (no-op when the constraint is disabled or monitor_only).
    Pose6D clampPoseToFloor(const Pose6D& pose) const;

    DualSendResult sendTargets(
        const ServoTarget& target,
        uint64_t command_seq,
        uint64_t command_host_time_ns,
        const std::string& send_policy,
        uint64_t dispatch_start_ns,
        uint64_t deadline_ns
    );

    DualArmCommand makeHoldCommand(
        const RobotState& left_state,
        const RobotState& right_state,
        uint64_t now_ns
    ) const;

    bool commandRequestsResetFault(const DualArmCommand& command) const;
    bool commandRequestsSetSafetyFloorZ(const DualArmCommand& command) const;
    bool commandRequestsEmergencyStop(const DualArmCommand& command) const;
    bool commandRequestsArmMotion(const DualArmCommand& command) const;
    bool commandRequestsDisarmMotion(const DualArmCommand& command) const;
    bool commandRequestsMotion(const DualArmCommand& command) const;
    bool commandBlockedByReadOnly(const DualArmCommand& command) const;
    bool readOnlyMode() const;
    bool workerIoMode() const;
    bool rbpodoAsyncIoMode() const;
    bool workerBackedIoMode() const;
    bool motionAllowed() const;
    bool isRealMode() const;
    std::string currentSendPolicy() const;
    bool clearFaultLatch(RobotState& left_state, RobotState& right_state);
    void latchFault(
        SafetyVerdict verdict,
        const std::string& reason,
        const RobotState& left_state,
        const RobotState& right_state,
        const std::optional<FaultContext>& context = std::nullopt
    );
    void latchFault(
        SafetyVerdict verdict,
        const std::string& reason,
        const RobotState& left_state,
        const RobotState& right_state,
        const LatchedDualFaultContext& contexts
    );
    void setMotionState(ServerMotionState state);
    ServoTarget currentFaultHoldTarget() const;
    JointArray chooseSafeHoldTarget(const RobotState& state, const JointArray& previous_sent) const;
    double computeFilterDtSec(uint64_t actual_period_ns, uint64_t nominal_period_ns) const;

private:
    std::unique_ptr<IRobotBackend> left_robot_;
    std::unique_ptr<IRobotBackend> right_robot_;
    std::unique_ptr<ArmWorker> left_worker_;
    std::unique_ptr<ArmWorker> right_worker_;

    DualArmConfig config_;

    CommandBuffer* command_buffer_ = nullptr;
    ServoLogger* logger_ = nullptr;
    std::shared_ptr<IKinematics> kinematics_;
    bool kinematics_injected_ = false;

    TrajectoryFilter left_traj_filter_;
    TrajectoryFilter right_traj_filter_;
    SafetyFilter safety_filter_;

    std::atomic<bool> running_{false};
    std::atomic<bool> startup_complete_{false};
    std::atomic<bool> startup_ok_{false};
    std::thread thread_;

    uint64_t tick_ = 0;
    uint64_t last_loop_start_ns_ = 0;

    JointArray left_prev_sent_q_deg_{};
    JointArray right_prev_sent_q_deg_{};

    JointArray left_prevprev_sent_q_deg_{};
    JointArray right_prevprev_sent_q_deg_{};
    JointArray left_controller_sim_physical_baseline_q_deg_{};
    JointArray right_controller_sim_physical_baseline_q_deg_{};

    std::atomic<ServerMotionState> motion_state_{ServerMotionState::Disconnected};
    mutable std::mutex state_mutex_;
    std::atomic<bool> fault_latched_{false};
    std::atomic<SafetyVerdict> fault_verdict_{SafetyVerdict::Ok};
    std::atomic<SafetyVerdict> latched_fault_reason_{SafetyVerdict::Ok};
    std::string fault_reason_;
    std::optional<FaultContext> latched_fault_context_;
    std::optional<FaultContext> left_latched_fault_context_;
    std::optional<FaultContext> right_latched_fault_context_;
    JointArray left_fault_hold_q_deg_{};
    JointArray right_fault_hold_q_deg_{};
    CartesianSolveTelemetry left_last_cartesian_solve_;
    CartesianSolveTelemetry right_last_cartesian_solve_;
    SafetyTrackingTelemetry left_safety_tracking_;
    SafetyTrackingTelemetry right_safety_tracking_;
    SelfCollisionResult last_self_collision_{};
    // Floor plane constraint (safety.floor_constraint): runtime-adjustable plane
    // height (SetSafetyFloorZ, bounded by config runtime_min/max) + per-arm
    // telemetry of the last evaluated candidate targets.
    std::atomic<double> runtime_floor_z_m_{0.0};
    FloorArmEvaluation last_floor_left_{};
    FloorArmEvaluation last_floor_right_{};
    uint64_t floor_clamp_count_ = 0;
    std::string floor_last_set_reject_reason_;
    // Controller-sim tracking-error advisory (safety.controller_simulation_tracking_error_nonlatching).
    // Reset each tick in loopMain; set in applySafety when a reference/actual tracking
    // divergence is suppressed (not latched). Surfaced as published telemetry
    // (tracking_error_degraded) + a throttled WARN.
    bool tracking_error_degraded_this_tick_ = false;
    bool tracking_error_degraded_prev_tick_ = false;
    uint64_t last_tracking_error_degraded_warn_ns_ = 0;
    std::string last_tracking_error_degraded_reason_;
    LatchedCartesianTarget left_latched_cartesian_target_;
    LatchedCartesianTarget right_latched_cartesian_target_;
    CartesianServoPathState left_cartesian_servo_path_;
    CartesianServoPathState right_cartesian_servo_path_;
    CartesianCircleMoveState left_cartesian_circle_move_;
    CartesianCircleMoveState right_cartesian_circle_move_;
    CartesianTwistHoldState left_cartesian_twist_hold_;
    CartesianTwistHoldState right_cartesian_twist_hold_;
    CartesianVelocityIntegratorState left_cartesian_velocity_integrator_;
    CartesianVelocityIntegratorState right_cartesian_velocity_integrator_;
    SmdPoseTracker left_pose_track_smd_{PoseTrackSmdConfig{}};
    SmdPoseTracker right_pose_track_smd_{PoseTrackSmdConfig{}};
    JointMovingAverage left_output_ma_{0};
    JointMovingAverage right_output_ma_{0};
    StartupValidationSnapshot startup_validation_;
    ServoSnapshot latest_snapshot_;
};

}  // namespace rb_servo
