#pragma once

#include <atomic>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/arm_worker.hpp"
#include "rb_servo/control/cartesian_servo_controller.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/joint_moving_average.hpp"
#include "rb_servo/control/smd_pose_tracker.hpp"
#include "rb_servo/control/self_collision.hpp"
#include "rb_servo/control/collision_monitor.hpp"
#include "rb_servo/control/init_motion_planner.hpp"
#include "rb_servo/control/floor_constraint.hpp"
#include "rb_servo/control/roi_box.hpp"
#include "rb_servo/control/reach_constraint.hpp"
#include "rb_servo/control/user_floor_constraint.hpp"
#include "rb_servo/control/fault_classifier.hpp"
#include "rb_servo/control/safety_filter.hpp"
#include "rb_servo/control/trajectory_filter.hpp"
#include "rb_servo/kinematics/i_kinematics.hpp"
#include "rb_servo/logging/servo_logger.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

class ScopePublisher;

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
        std::shared_ptr<IKinematics> kinematics = nullptr,
        ScopePublisher* scope_publisher = nullptr
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

    // Push the latest per-arm gripper open percent (0 closed .. 100 open) into the
    // safety gate so the TCP fingertip offset points interpolate between their
    // gripper-open and gripper-closed positions (see interpolateOffsetPoints). Fed
    // off the control loop by the StatePublisher's gripper-feedback read (non-blocking
    // atomic store). valid=false (stale/absent feedback, gripper disabled) makes the
    // gate fall back to the gripper-OPEN offsets — the conservative, larger envelope.
    void setGripperFeedback(ArmId arm, double percent, bool valid);

private:
    // Live gripper open percent per arm and its validity, set by setGripperFeedback
    // and read in the evaluate*Arm safety gates. Default OPEN (100) / invalid so the
    // pre-feedback behavior equals the legacy gripper-open offsets.
    std::atomic<double> gripper_percent_left_{100.0};
    std::atomic<double> gripper_percent_right_{100.0};
    std::atomic<bool> gripper_percent_valid_left_{false};
    std::atomic<bool> gripper_percent_valid_right_{false};
    // Effective gripper percent for offset interpolation: live percent when valid,
    // else 100 (open). Used by the evaluate*Arm gates.
    double effectiveGripperPercent(ArmId arm) const;
    // Per-constraint, per-arm scratch buffers holding the gripper-interpolated offset
    // points handed to the inline constraint evaluators each tick. mutable because
    // the evaluate*Arm methods are const; reused in place to avoid 500 Hz heap churn.
    mutable std::vector<FloorCheckPointConfig> floor_offset_scratch_left_;
    mutable std::vector<FloorCheckPointConfig> floor_offset_scratch_right_;
    mutable std::vector<FloorCheckPointConfig> roi_offset_scratch_left_;
    mutable std::vector<FloorCheckPointConfig> roi_offset_scratch_right_;
    mutable std::vector<FloorCheckPointConfig> reach_offset_scratch_left_;
    mutable std::vector<FloorCheckPointConfig> reach_offset_scratch_right_;
    mutable std::vector<FloorCheckPointConfig> user_floor_offset_scratch_left_;
    mutable std::vector<FloorCheckPointConfig> user_floor_offset_scratch_right_;
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

    // Collision-free JointTarget init_motion profile sequencer. When either arm's
    // JointTarget carries joint_target_profile=init_motion, this drives an async
    // plan (off the RT loop) from the current sent pose to the commanded init pose,
    // then REWRITES the command into ordinary direct JointTarget waypoints.
    // While planning it holds in place; on planning failure it holds and latches
    // the Failed status (fail-closed, never an un-planned motion). If the planner
    // is disabled it falls back to a direct JointTarget to the target.
    DualArmCommand applyInitMotionSequencer(
        DualArmCommand command,
        const RobotState& left_state,
        const RobotState& right_state
    );

    // Stand-frame floor plane constraint (safety.floor_constraint): FK the arm's
    // TCP for a candidate joint target. checked=false if kinematics/FK is
    // unavailable — the caller fails closed.
    FloorArmEvaluation evaluateFloorArm(ArmId arm, const JointArray& q_deg) const;
    // Effective (runtime-adjustable) floor plane height in meters.
    double effectiveFloorZ() const;
    // Effective stand-floor enforcement state: config opt-in AND the runtime toggle
    // (SetSafetyFloorEnabled). config.enable=false is a hard off (no runtime enable).
    bool floorConstraintActive() const;
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

    // Stand-frame user-defined tilted floor plane (safety.user_floor_constraint):
    // FK the arm's TCP for a candidate joint target and evaluate its signed distance
    // to the runtime plane. checked=false if kinematics/FK is unavailable — the
    // caller fails closed. The effective plane is read from the runtime atomics.
    UserFloorArmEvaluation evaluateUserFloorArm(ArmId arm, const JointArray& q_deg) const;
    bool userFloorActive() const;  // enabled at runtime AND constraint configured
    math::Vector3 effectiveUserFloorPoint() const;
    math::Vector3 effectiveUserFloorNormal() const;
    double effectiveUserFloorMargin() const;

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
    bool commandRequestsSetSafetyFloorEnabled(const DualArmCommand& command) const;
    bool commandRequestsSetSafetyRoiBounds(const DualArmCommand& command) const;
    bool commandRequestsSetUserSafetyFloorPlane(const DualArmCommand& command) const;
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
    ScopePublisher* scope_publisher_ = nullptr;
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

    // Collision-free InitMotion planner + sequencer state (safety.init_motion_planner).
    // The planner owns a PRIVATE CollisionMonitor (incl. the ground plane) and is null
    // when the feature is disabled (then InitMotion falls back to a direct JointTarget).
    std::unique_ptr<InitMotionPlanner> init_motion_planner_;
    enum class InitMotionStatus { Idle, Planning, Executing, Done, Failed };
    struct InitMotionExec {
        InitMotionStatus status = InitMotionStatus::Idle;
        bool has_target = false;
        JointArray target_left{};
        JointArray target_right{};
        std::future<InitMotionPlanResult> future;
        std::vector<std::pair<JointArray, JointArray>> waypoints;
        std::size_t index = 0;
        int escape_waypoints = 0;  // leading sub-threshold escape waypoints (follow precisely)
        std::string message;
        InitMotionPlanResult::FailMode fail_mode = InitMotionPlanResult::FailMode::None;
        double start_clear_m = std::numeric_limits<double>::quiet_NaN();
        double goal_clear_m = std::numeric_limits<double>::quiet_NaN();
        int tree_start_size = 0;
        int tree_goal_size = 0;
        int last_iterations = 0;
        double last_planning_time_s = 0.0;
        bool exec_timeout = false;
        bool exec_stalled = false;
        uint64_t start_ns = 0;  // steady-clock stamp when this sequence began (runaway bound)
        // Progress-aware execution timeout: the smallest max-joint distance-to-goal seen so
        // far and when it last improved. A move that keeps closing on the goal is NOT killed
        // by the wall-clock budget; a genuine stall (no progress for the stall window) fails
        // closed FAST instead of holding motion authority for the full budget.
        double best_dist_deg = 0.0;
        uint64_t last_progress_ns = 0;
        uint64_t last_exec_log_ns = 0;  // throttle for the streaming-progress diagnostic
    };
    InitMotionExec init_motion_exec_;

    // Collision-free TcpLinearMove (cartesian_control.linear_move.collision_free): decide
    // async whether the straight Cartesian path is clear (Straight -> run the exact MoveL)
    // or blocked (Detour -> stream an RRT joint detour to the IK'd goal). Reuses
    // init_motion_planner_ + pursueWaypoints.
    enum class LinearMoveStatus { Idle, Deciding, Straight, Detour, Done, Failed };
    struct LinearMoveExec {
        LinearMoveStatus status = LinearMoveStatus::Idle;
        bool has_target = false;
        bool left_active = false;
        bool right_active = false;
        bool slerp = false;
        Pose6D target_left{};
        Pose6D target_right{};
        std::future<InitMotionLinearResult> future;
        std::vector<std::pair<JointArray, JointArray>> waypoints;
        std::size_t index = 0;
        int escape_waypoints = 0;  // leading sub-threshold escape waypoints (follow precisely)
        std::string message;
        uint64_t start_ns = 0;
    };
    LinearMoveExec linear_move_exec_;

    // Collision-free TcpLinearMove sequencer (sibling of applyInitMotionSequencer): when
    // collision_free is enabled, decides Straight vs Detour for a TcpLinearMove and either
    // passes the command through (Straight, exact MoveL) or rewrites it to a streamed
    // JointTarget detour. Non-TcpLinearMove commands reset it.
    DualArmCommand applyCollisionFreeLinearMove(
        DualArmCommand command,
        const RobotState& left_state,
        const RobotState& right_state
    );

    // Pure-pursuit over a planned waypoint list using the current sent pose. Thin wrapper
    // around pursueWaypointsStep() that supplies the current sent pose and config tolerances:
    // advances `index` by projection (never stalls on corner-cutting), returns the farthest
    // waypoint within the execution lookahead (so the servo runs near max velocity), and
    // sets `done` at the final waypoint. Shared by InitMotion and the collision-free linear
    // detour.
    std::pair<JointArray, JointArray> pursueWaypoints(
        const std::vector<std::pair<JointArray, JointArray>>& waypoints,
        std::size_t& index,
        bool& done,
        int escape_count = 0
    ) const;
    CollisionVerdict last_collision_verdict_{};
    // Floor plane constraint (safety.floor_constraint): runtime-adjustable plane
    // height (SetSafetyFloorZ, bounded by config runtime_min/max) + per-arm
    // telemetry of the last evaluated candidate targets.
    std::atomic<double> runtime_floor_z_m_{0.0};
    // Runtime enforce on/off (SetSafetyFloorEnabled), seeded from config.enable.
    std::atomic<bool> runtime_floor_enabled_{false};
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
    // User-defined tilted floor plane (safety.user_floor_constraint): runtime
    // enable + plane (point/normal/margin), set via SetUserSafetyFloorPlane and
    // bounded by validateUserFloorPlaneRequest, plus per-arm telemetry of the last
    // evaluated candidate targets (signed distance to the plane).
    std::atomic<bool> runtime_user_floor_enabled_{false};
    std::array<std::atomic<double>, 3> runtime_user_floor_point_m_{};
    std::array<std::atomic<double>, 3> runtime_user_floor_normal_{};
    std::atomic<double> runtime_user_floor_margin_m_{0.0};
    UserFloorArmEvaluation last_user_floor_left_{};
    UserFloorArmEvaluation last_user_floor_right_{};
    uint64_t user_floor_clamp_count_ = 0;
    std::string user_floor_last_set_reject_reason_;
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
    SmdPoseTracker left_pose_track_smd_{PoseTrackSmdConfig{}};
    SmdPoseTracker right_pose_track_smd_{PoseTrackSmdConfig{}};
    JointMovingAverage left_output_ma_{0};
    JointMovingAverage right_output_ma_{0};
    // A/B/C separation telemetry (Patch 4), captured each tick from the SMD step
    // and the output-MA stage, merged into the per-arm cartesian_solve sample.
    struct AbcTelemetry {
        bool smd_active = false;
        std::optional<Pose6D> smd_ref_stand;
        std::optional<Pose6D> smd_goal_stand;
        SmdStepInfo smd_step_info;
        std::uint64_t smd_reanchor_count = 0;
        bool output_ma_present = false;
        JointArray q_target_before_output_ma_deg{};
        JointArray q_target_after_output_ma_deg{};
        bool safety_clamp_present = false;
        SafetyClampTelemetry safety_clamp;
    };
    AbcTelemetry left_abc_telemetry_;
    AbcTelemetry right_abc_telemetry_;
    static void mergeAbcTelemetry(CartesianSolveTelemetry& solve, const AbcTelemetry& abc,
                                  int output_ma_window);
    StartupValidationSnapshot startup_validation_;
    ServoSnapshot latest_snapshot_;
};

// Projection-based pure-pursuit step over a planned collision-free waypoint polyline.
// Given the current sent pose, it advances `index` by PROJECTING the pose onto the path
// (monotonic, never stalls — even when the lookahead chord cuts a corner and never comes
// within `waypoint_tol_deg` of an apex node), then returns the farthest forward waypoint
// within `lookahead_deg` of the current pose. `done` is set once the pose has settled
// within `waypoint_tol_deg` of the final waypoint. Stateless apart from the in/out
// `index`, so it is unit-testable in isolation (see test_init_motion_pursuit).
struct PursuitStep {
    JointArray left{};
    JointArray right{};
    bool done = false;
};
PursuitStep pursueWaypointsStep(
    const std::vector<std::pair<JointArray, JointArray>>& waypoints,
    const JointArray& cur_left,
    const JointArray& cur_right,
    std::size_t& index,
    double waypoint_tol_deg,
    double lookahead_deg,
    int escape_count = 0);

}  // namespace rb_servo
