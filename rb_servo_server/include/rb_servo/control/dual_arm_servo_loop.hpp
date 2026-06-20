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
#include "rb_servo/control/collision_monitor.hpp"
#include "rb_servo/control/floor_constraint.hpp"
#include "rb_servo/control/roi_box.hpp"
#include "rb_servo/control/reach_constraint.hpp"
#include "rb_servo/control/fault_classifier.hpp"
#include "rb_servo/control/safety_filter.hpp"
#include "rb_servo/control/trajectory_filter.hpp"
#include "rb_servo/kinematics/i_kinematics.hpp"
#include "rb_servo/logging/servo_logger.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

// Per-arm direct-teaching (free-drive) lifecycle. Real-time servo control
// (move_servo_j streaming) and direct teaching are mutually exclusive controller
// regimes: entering free-drive while the controller is still executing servo
// motion is rejected with pendant error M151 ("Direct Teaching: Cannot run this
// function"). The arming state machine quiesces the servo stream and waits for
// the controller to report idle BEFORE issuing freedrive_teach_on, then confirms
// engagement via the controller's is_freedrive_mode flag.
enum class FreedriveStage {
    Off,       // normal servoing; no direct teaching
    Quiesce,   // ON requested: servo_j suppressed, waiting for controller idle
    Confirm,   // freedrive_teach_on issued, waiting for is_freedrive_mode == 1
    Active,    // direct teaching engaged and confirmed (hand-guiding)
    Exiting,   // OFF requested: freedrive_teach_off issued, waiting for is_freedrive_mode == 0
};

const char* toString(FreedriveStage stage);

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
        ControlMode left_mode,
        ControlMode right_mode,
        double dt_sec,
        SafetyVerdict* verdict
    );

    // Stand-frame floor plane constraint (safety.floor_constraint): FK the arm's
    // TCP for a candidate joint target. checked=false if kinematics/FK is
    // unavailable — the caller fails closed.
    FloorArmEvaluation evaluateFloorArm(ArmId arm, const JointArray& q_deg) const;
    // Effective (runtime-adjustable) floor plane height in meters.
    double effectiveFloorZ() const;
    // Tier-2 usability clamp: project a Cartesian target's stand z onto the floor
    // plane (no-op when the constraint is disabled or monitor_only).
    Pose6D clampPoseToFloor(const Pose6D& pose) const;

    // Stand-frame ROI box constraint (safety.roi_box): FK the arm's TCP for a
    // candidate joint target and evaluate it against the 6 box faces.
    // checked=false if kinematics/FK is unavailable — the caller fails closed.
    RoiArmEvaluation evaluateRoiArm(ArmId arm, const JointArray& q_deg) const;
    // Effective (runtime-adjustable) ROI box bounds in meters (stand frame).
    std::array<double, 3> effectiveRoiMin() const;
    std::array<double, 3> effectiveRoiMax() const;

    // Stand-frame reachable-shell constraint (safety.reach_constraint): FK the
    // arm's TCP for a candidate joint target and evaluate its radial distance from
    // the arm base against the inner/outer shells. checked=false if kinematics/FK
    // is unavailable — the caller fails closed.
    ReachArmEvaluation evaluateReachArm(ArmId arm, const JointArray& q_deg) const;
    // Tier-2 usability clamp: pull a Cartesian target's stand position inside the
    // ROI box (no-op when the constraint is disabled or monitor_only).
    Pose6D clampPoseToRoi(const Pose6D& pose) const;

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
    bool commandRequestsSetSafetyRoiBounds(const DualArmCommand& command) const;
    bool commandRequestsEmergencyStop(const DualArmCommand& command) const;
    bool commandRequestsArmMotion(const DualArmCommand& command) const;
    bool commandRequestsDisarmMotion(const DualArmCommand& command) const;
    bool commandRequestsFreedrive(const DualArmCommand& command) const;
    // Per-arm direct-teaching (free-drive) requests. Translates an inbound
    // Freedrive command into a stage transition (Off->Quiesce on ON,
    // Active/Quiesce/Confirm->exit on OFF) without touching the controller yet.
    // Fail-closed: a no-op unless config servo.allow_freedrive is set.
    void requestFreedrive(
        const DualArmCommand& command,
        const RobotState& left_state,
        const RobotState& right_state
    );
    // Per-tick advance of the per-arm free-drive state machine. Quiesces the servo
    // stream, waits for the controller to report idle, issues freedrive_teach_on,
    // confirms via is_freedrive_mode, and resyncs the held target on exit.
    void advanceFreedrive(const RobotState& left_state, const RobotState& right_state);
    // True while any arm is not Off (drives global servo_j send suppression and
    // motion-pipeline bypass — sends stop the moment an arm enters Quiesce).
    bool anyFreedriveActive() const;
    // True only while an arm is fully engaged (Active) — used for telemetry.
    bool anyFreedriveEngaged() const;
    // Issue freedrive_teach_on/off to one arm's backend (worker lifecycle in
    // worker/async I/O, direct otherwise). Returns the backend result.
    BackendResult<RobotState> sendFreedriveToBackend(ArmId arm_id, bool on);
    // Whether this arm's backend reports controller state usable to gate freedrive
    // (rbpodo). Non-rbpodo backends (mock/simulator) bypass the quiesce/confirm
    // waits and toggle immediately.
    bool freedriveUsesControllerSignals(ArmId arm_id) const;
    void resyncArmAfterFreedrive(ArmId arm_id, const RobotState& state);
    void abortFreedrive(ArmId arm_id, const RobotState& state, const std::string& reason);
    // Best-effort freedrive_teach_off for any arm still in (or arming) freedrive,
    // issued from stop() so a Ctrl-C/shutdown mid-teaching does not leave the
    // controller latched in freedrive_teach_on (which also blocks the pendant's
    // hardware direct-teaching button). Called after the loop thread is joined
    // but before the workers/backends are torn down.
    void teardownFreedriveOnStop();
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
    // Per-arm direct-teaching (free-drive) lifecycle. While either arm is not Off
    // the server suppresses servo_j to both controllers (send_policy=="freedrive")
    // and bypasses the motion pipeline. Owned by the loop thread; published as
    // atomics so currentSendPolicy()/telemetry can read them lock-free.
    std::atomic<FreedriveStage> left_freedrive_stage_{FreedriveStage::Off};
    std::atomic<FreedriveStage> right_freedrive_stage_{FreedriveStage::Off};
    // Hard deadline (steady ns) for the current Quiesce/Confirm/Exiting stage;
    // exceeding it aborts the transition. Loop-thread only.
    uint64_t left_freedrive_deadline_ns_ = 0;
    uint64_t right_freedrive_deadline_ns_ = 0;
    // Steady-ns timestamp the current stage was entered (settle-time fallback for
    // controllers that do not report a usable motion state). Loop-thread only.
    uint64_t left_freedrive_stage_entered_ns_ = 0;
    uint64_t right_freedrive_stage_entered_ns_ = 0;
    // Last free-drive abort/failure reason, surfaced in state telemetry. Guarded
    // by state_mutex_ on publish.
    std::string freedrive_note_;
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
    // URDF mesh self-collision (safety.self_collision.mesh): async monitor thread
    // off the servo_j path + the shared velocity-barrier config. Null in capsule
    // mode. last_collision_verdict_ caches the latest verdict for telemetry.
    std::unique_ptr<CollisionMonitor> collision_monitor_;
    CollisionMonitorConfig collision_monitor_cfg_{};
    CollisionVerdict last_collision_verdict_{};
    // Floor plane constraint (safety.floor_constraint): runtime-adjustable plane
    // height (SetSafetyFloorZ, bounded by config runtime_min/max) + per-arm
    // telemetry of the last evaluated candidate targets.
    std::atomic<double> runtime_floor_z_m_{0.0};
    FloorArmEvaluation last_floor_left_{};
    FloorArmEvaluation last_floor_right_{};
    uint64_t floor_clamp_count_ = 0;
    std::string floor_last_set_reject_reason_;
    // ROI box constraint (safety.roi_box): runtime-adjustable stand-frame bounds
    // (SetSafetyRoiBounds, bounded by config runtime_min/max per axis) + per-arm
    // telemetry of the last evaluated candidate targets.
    std::array<std::atomic<double>, 3> runtime_roi_min_m_{};
    std::array<std::atomic<double>, 3> runtime_roi_max_m_{};
    RoiArmEvaluation last_roi_left_{};
    RoiArmEvaluation last_roi_right_{};
    uint64_t roi_clamp_count_ = 0;
    std::string roi_last_set_reject_reason_;
    // Reachable-shell constraint (safety.reach_constraint): per-arm telemetry of the
    // last evaluated candidate targets (radial distance from the arm base vs the
    // inner/outer shells). No runtime-adjust command (shell radii are config-fixed).
    ReachArmEvaluation last_reach_left_{};
    ReachArmEvaluation last_reach_right_{};
    uint64_t reach_clamp_count_ = 0;
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
