#pragma once

#include <atomic>
#include <condition_variable>
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
#include "rb_servo/control/cartesian_chunk_follower.hpp"
#include "rb_servo/control/cartesian_servo_controller.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/delta_twist_follower.hpp"
#include "rb_servo/control/force_controller.hpp"
#include "rb_servo/control/joint_moving_average.hpp"
#include "rb_servo/control/normal_force_controller.hpp"
#include "rb_servo/control/realtime_timing.hpp"
#include "rb_servo/control/smd_pose_tracker.hpp"
#include "rb_servo/network/chunk_frame_receiver.hpp"
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
#include "rb_servo/sensor/ft_wrench_pipeline.hpp"

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

    // Wire the dedicated chunk-frame ingest (Ruckig chunk-follower reference
    // input). Must be called BEFORE start(); the receiver must outlive the loop.
    // nullptr (or a receiver with an empty bind) leaves every profile on the
    // pose_track_smd path even when ruckig_follower.enable is set.
    void setChunkFrameReceiver(ChunkFrameReceiver* receiver) { chunk_frame_receiver_ = receiver; }

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
    bool commandRequestsSetExternalBoxes(const DualArmCommand& command) const;
    bool commandRequestsSetUserSafetyFloorPlane(const DualArmCommand& command) const;
    bool applySetExternalBoxesCommand(const DualArmCommand& command);
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
    JointArray chooseSafeHoldTarget(
        ArmId arm_id,
        const RobotState& state,
        const JointArray& previous_sent
    ) const;
    double computeFilterDtSec(uint64_t actual_period_ns, uint64_t nominal_period_ns) const;
    void resetChunkFollowerEngageWait(ArmId arm_id);
    void clearChunkFollowerFaultRequests();
    void recordChunkFollowerFaultRequest(ArmId arm_id, const std::string& reason);
    bool latchChunkFollowerFaultRequests(const RobotState& left_state, const RobotState& right_state);
    void markSafetyIntervention(ArmId arm_id, uint64_t now_ns);
    bool safetyInterventionRecent(ArmId arm_id, uint64_t now_ns) const;
    bool updateForceRuntime(
        ArmId arm_id,
        const RobotState& state,
        double dt_sec,
        uint64_t now_ns,
        std::string* fault_reason
    );
    void invalidatePostInitTare(ArmId arm_id, uint64_t command_seq);
    void beginPostInitTare(ArmId arm_id, uint64_t now_ns);
    bool latchPayloadIdentificationInhibit(ArmId arm_id);
    void applyForceCorrection(ArmId arm_id, ArmCommand* command);
    void finishForceProposals(
        bool left_accepted,
        bool right_accepted,
        SafetyVerdict verdict
    );

private:
    std::unique_ptr<IRobotBackend> left_robot_;
    std::unique_ptr<IRobotBackend> right_robot_;
    std::unique_ptr<ArmWorker> left_worker_;
    std::unique_ptr<ArmWorker> right_worker_;

    DualArmConfig config_;

    struct ForceArmRuntime {
        ForceTorqueTelemetry ft;
        ForceControlTelemetry control;
        Pose6D contact_anchor;
        bool contact_anchor_valid = false;
        bool normal_contact_active = false;
        ContactForceNormalEstimator contact_force_normal_estimator;
        double contact_cartesian_normal_offset_m = 0.0;
        bool transverse_contact_active = false;
        bool rotational_contact_active = false;
        int enter_count = 0;
        int transverse_enter_count = 0;
        int rotational_enter_count = 0;
        int hard_limit_count = 0;
        uint64_t release_start_ns = 0;
        Pose6D release_hold_pose;
        bool release_hold_pending = false;
        bool release_hold_applied = false;
        bool release_hold_clear_requested = false;
        Pose6D previous_actual_pose;
        uint64_t previous_actual_pose_ns = 0;
        Pose6D sent_tcp_sample_older;
        Pose6D sent_tcp_sample_newer;
        uint64_t sent_tcp_sample_older_ns = 0;
        uint64_t sent_tcp_sample_newer_ns = 0;
        int sent_tcp_sample_count = 0;
        std::optional<NormalForceControllerProposal> pending_proposal;
        bool pending_proposal_applied = false;
        std::optional<ForceControllerProposal> pending_cartesian_proposal;
        bool pending_cartesian_proposal_applied = false;
        Wrench6D control_wrench_surface;
        Wrench6D control_wrench_compliance;
        Vec6 actual_twist_compliance;
        Pose6D compliance_frame_actual_stand;
        Pose6D previous_raw_compliance_target;
        Pose6D rolling_compliance_target;
        bool rolling_compliance_target_valid = false;
        std::string rolling_compliance_target_source = "unavailable";
        Pose6D pending_previous_raw_compliance_target;
        Pose6D pending_rolling_compliance_target;
        bool pending_rolling_compliance_target_valid = false;
        std::string pending_rolling_compliance_target_source = "unavailable";
        bool compliance_hold_target_this_tick = false;
        bool tare_valid = false;
        bool tare_waiting_for_init_completion = false;
        bool tare_collecting = false;
        uint64_t tare_not_before_ns = 0;
        uint64_t last_init_tare_command_seq = 0;
        uint64_t tare_generation = 0;
        bool payload_identification_inhibit = false;
        // Per-tick compliance frame (TCP-local) for the follower's compliance-
        // aware actual-lead compensation; valid once the frame is resolved.
        Pose6D t_tcp_compliance_pose;
        bool t_tcp_compliance_valid = false;
        // Deadband-filtered loading direction (stand frame) for the chunk
        // follower's wrench-gated loading projection. Valid only while the
        // pipeline is healthy and cartesian_admittance is active this tick.
        std::array<double, 3> follower_loading_reaction_stand{{0.0, 0.0, 0.0}};
        bool follower_loading_reaction_valid = false;
        // contact_force episode entry direction-consistency window: the force
        // direction at the first debounce sample; later samples must stay in a
        // 30 deg cone or the window restarts (transit residuals rotate, real
        // contact does not).
        std::array<double, 3> contact_entry_first_dir{{0.0, 0.0, 0.0}};
        bool contact_entry_first_dir_valid = false;
        // True when the chunk follower produced this tick's Cartesian target.
        // The rolling compliance equilibrium then adopts the follower output
        // verbatim instead of re-projecting policy deltas: the follower's
        // wrench-gated projection already removed contact loading, and running
        // BOTH projections let the equilibrium and the plan drift apart
        // (2026-07-18 14:16 run: an 11.6 mm plan-vs-equilibrium gap consumed
        // the actual-lead budget and latched at 21.4 mm with only ~0.3 deg of
        // real joint tracking error).
        bool chunk_follower_drove_this_tick = false;
        bool follower_contact_normal_owned = false;
    };
    FtWrenchPipeline left_ft_pipeline_;
    FtWrenchPipeline right_ft_pipeline_;
    NormalForceController left_normal_force_controller_;
    NormalForceController right_normal_force_controller_;
    ForceController left_cartesian_force_controller_;
    ForceController right_cartesian_force_controller_;
    ForceArmRuntime left_force_runtime_;
    ForceArmRuntime right_force_runtime_;
    uint64_t motion_epoch_ = 0;

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
    RealtimeTimingAccumulator realtime_timing_accumulator_;

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
    // External keep-out box feed liveness. When the boxes are ENFORCED
    // (external_boxes.enable && !monitor_only) the keep-out is useless without a live
    // producer feed (the box geometry stays parked/inert), and that loss is silent. To
    // keep it diagnosable we FAIL CLOSED: if no fresh SetExternalBoxes feed arrives
    // (producer never started, or it stopped), checkExternalBoxFeedOrAbort() aborts the
    // process with a loud reason instead of running on without keep-out. Inert when the
    // boxes are monitor_only or disabled (the current default config). Liveness itself
    // is stamped at RECEIVE time on the CommandBuffer
    // (CommandBuffer::lastExternalBoxReceiveNs), independent of the external-box side
    // slot, so control-loop scheduling cannot hide a live producer or false-abort.
    uint64_t external_box_enforce_start_ns_ = 0;    // first enforced tick (startup-grace anchor)
    void checkExternalBoxFeedOrAbort(uint64_t now_ns);

    // Collision-free InitMotion planner + sequencer state (safety.init_motion_planner).
    // The planner owns a PRIVATE CollisionMonitor (incl. the ground plane) and is null
    // when the feature is disabled (then InitMotion falls back to a direct JointTarget).
    std::unique_ptr<InitMotionPlanner> init_motion_planner_;
    enum class PlannerRequester { LeftInit = 0, RightInit = 1, Linear = 2 };
    struct PlannerJob {
        PlannerRequester requester = PlannerRequester::LeftInit;
        uint64_t generation = 0;
        bool is_linear = false;
        JointArray start_left{};
        JointArray start_right{};
        JointArray target_left{};
        JointArray target_right{};
        bool request_left = false;
        bool request_right = false;
        Pose6D lin_goal_left{};
        Pose6D lin_goal_right{};
        bool lin_left_active = false;
        bool lin_right_active = false;
        bool slerp = false;
        int lin_samples = 0;
    };
    struct PlannerResultSlot {
        uint64_t generation = 0;
        bool valid = false;
        bool is_linear = false;
        InitMotionPlanResult init_result{};
        InitMotionLinearResult linear_result{};
    };
    std::thread planner_worker_;
    std::mutex planner_mtx_;
    std::condition_variable planner_cv_;
    bool planner_stop_ = false;
    std::optional<PlannerJob> planner_pending_[3];
    PlannerResultSlot planner_result_[3];
    uint64_t planner_job_seq_ = 0;
    void plannerWorkerMain();
    uint64_t postPlannerJob(PlannerJob job);
    bool takePlannerResult(PlannerRequester requester, uint64_t generation, PlannerResultSlot& out);

    enum class InitMotionStatus { Idle, Planning, Executing, Done, Failed };
    struct InitMotionExec {
        InitMotionStatus status = InitMotionStatus::Idle;
        // Init Motion is a one-shot request, but CommandBuffer keeps the accepted
        // packet visible until its timeout.  Remember the request sequence so the
        // same cached packet cannot repeatedly re-anchor Hold to q_actual.
        bool request_seen = false;
        uint64_t request_seq = 0;
        bool has_target = false;
        bool left_active = false;
        bool right_active = false;
        JointArray target_left{};
        JointArray target_right{};
        uint64_t plan_generation = 0;
        std::vector<std::pair<JointArray, JointArray>> waypoints;
        std::size_t index = 0;
        int escape_waypoints = 0;  // leading sub-threshold escape waypoints (follow precisely)
        std::string message;
        InitMotionPlanResult::FailMode fail_mode = InitMotionPlanResult::FailMode::None;
        double start_clear_m = std::numeric_limits<double>::quiet_NaN();
        double goal_clear_m = std::numeric_limits<double>::quiet_NaN();
        double goal_self_min_clearance_m = std::numeric_limits<double>::quiet_NaN();
        double goal_external_min_clearance_m = std::numeric_limits<double>::quiet_NaN();
        std::string goal_nearest_pair_name_a;
        std::string goal_nearest_pair_name_b;
        std::string goal_nearest_pair_category;
        bool goal_nearest_pair_external = false;
        bool goal_nearest_pair_disabled_by_rule = false;
        double goal_nearest_pair_distance_m = std::numeric_limits<double>::quiet_NaN();
        double goal_clear_threshold_self_m = std::numeric_limits<double>::quiet_NaN();
        double goal_clear_threshold_external_m = std::numeric_limits<double>::quiet_NaN();
        double goal_clear_margin_deficit_m = std::numeric_limits<double>::quiet_NaN();
        double clear_threshold_m = std::numeric_limits<double>::quiet_NaN();
        double external_clear_threshold_m = std::numeric_limits<double>::quiet_NaN();
        std::string nearest_pair;
        double nearest_pair_distance_m = std::numeric_limits<double>::quiet_NaN();
        bool nearest_pair_external = false;
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
    InitMotionExec left_init_motion_exec_;
    InitMotionExec right_init_motion_exec_;

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
        uint64_t plan_generation = 0;
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
    // advances `index` by projection or far-endpoint proximity (never stalls on
    // corner-cutting or asymptotic endpoint convergence), returns the farthest waypoint
    // within the execution lookahead (so the servo runs near max velocity), and sets
    // `done` at the final waypoint. Shared by InitMotion and the collision-free linear
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
    // Last tick when the final safety gate actively modified/held each arm.
    // applyChunkFollowerStage runs before this tick's applySafety, so strict
    // follower divergence reads the previous safety tick through a short debounce.
    uint64_t left_safety_intervention_last_ns_ = 0;
    uint64_t right_safety_intervention_last_ns_ = 0;
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
    std::string left_pose_track_profile_name_;
    std::string right_pose_track_profile_name_;
    // Chunk-follower stage (per-profile opt-in replacement for the SMD step).
    // The default absolute-waypoint follower still prewarms Ruckig off the RT
    // path; delta_twist consumes local action deltas through a separate state.
    control::CartesianChunkFollower left_chunk_follower_{control::CartesianChunkFollowerConfig{}};
    control::CartesianChunkFollower right_chunk_follower_{control::CartesianChunkFollowerConfig{}};
    control::DeltaTwistFollower left_delta_twist_follower_{control::DeltaTwistFollowerConfig{}};
    control::DeltaTwistFollower right_delta_twist_follower_{control::DeltaTwistFollowerConfig{}};
    RuckigFollowerConfig left_chunk_follower_built_{};
    RuckigFollowerConfig right_chunk_follower_built_{};
    std::uint64_t left_chunk_submitted_wire_seq_ = 0;
    std::uint64_t left_chunk_submitted_recv_seq_ = 0;
    std::uint64_t right_chunk_submitted_wire_seq_ = 0;
    std::uint64_t right_chunk_submitted_recv_seq_ = 0;
    bool left_chunk_engage_waiting_ = false;
    bool right_chunk_engage_waiting_ = false;
    double left_chunk_engage_wait_start_sec_ = 0.0;
    double right_chunk_engage_wait_start_sec_ = 0.0;
    std::uint64_t left_chunk_follower_reanchor_count_ = 0;
    std::uint64_t right_chunk_follower_reanchor_count_ = 0;
    std::uint64_t left_chunk_follower_warm_resume_count_ = 0;
    std::uint64_t right_chunk_follower_warm_resume_count_ = 0;
    uint64_t left_chunk_follower_reanchor_log_ns_ = 0;
    uint64_t right_chunk_follower_reanchor_log_ns_ = 0;
    struct ChunkFollowerFaultRequest {
        bool active = false;
        ArmId arm = ArmId::Left;
        std::string reason;
    };
    ChunkFollowerFaultRequest left_chunk_follower_fault_request_;
    ChunkFollowerFaultRequest right_chunk_follower_fault_request_;
    ChunkFrameReceiver* chunk_frame_receiver_ = nullptr;
    ChunkFrameReceiver::Frame chunk_frame_cache_{};
    std::uint64_t chunk_frame_cache_recv_seq_ = 0;
    uint64_t last_chunk_follower_log_ns_ = 0;
    // Copy the newest chunk frame out of the receiver (at most one copy per
    // frame; try_lock, allocation-free). Called once per tick before the
    // Cartesian smoothing stage.
    void pollChunkFrames();
    // The SMD-stage drop-in: runs the Ruckig chunk-follower when the profile
    // enables it and a chunk feed is live. Legacy profiles fall back to
    // applyPoseTrackSmd with identical semantics; strict profiles hold and
    // request a ChunkFollowerFault on follower-regime interruptions.
    ArmCommand applyChunkFollowerStage(
        ArmId arm_id,
        const ArmCommand& command,
        const TcpPoseTargetProfileConfig& profile,
        control::CartesianChunkFollower* follower,
        RuckigFollowerConfig* built_cfg,
        std::uint64_t* submitted_wire_seq,
        std::uint64_t* submitted_recv_seq,
        SmdPoseTracker* smd_tracker,
        const ArmMountConfig& mount,
        const JointArray& previous_sent_q_deg,
        const Pose6D& actual_feedback_pose,
        double dt_sec
    );
    ArmCommand applyDeltaTwistFollowerStage(
        ArmId arm_id,
        const ArmCommand& command,
        const TcpPoseTargetProfileConfig& profile,
        control::DeltaTwistFollower* follower,
        RuckigFollowerConfig* built_cfg,
        std::uint64_t* submitted_wire_seq,
        std::uint64_t* submitted_recv_seq,
        SmdPoseTracker* smd_tracker,
        const ArmMountConfig& mount,
        const JointArray& previous_sent_q_deg,
        const Pose6D& actual_feedback_pose,
        const Pose6D& execution_feedback_pose,
        double dt_sec
    );
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
        std::string tcp_target_profile;
        bool tcp_target_profile_found = false;
        PoseTrackSmdConfig smd_profile;
        double max_smd_goal_lead_m = 0.0;
        double max_smd_goal_lead_rad = 0.0;
        bool output_ma_present = false;
        JointArray q_target_before_output_ma_deg{};
        JointArray q_target_after_output_ma_deg{};
        bool safety_clamp_present = false;
        SafetyClampTelemetry safety_clamp;
        // Chunk-follower stage telemetry (captured in apply*FollowerStage each
        // tick; merged into cartesian_solve).
        std::string follower_controller = "none";
        bool follower_active = false;
        std::uint64_t follower_wire_seq = 0;
        std::uint64_t follower_recv_seq = 0;
        int follower_step = -1;
        double follower_t_in_seg_sec = 0.0;
        double follower_duration_sec = 0.0;
        double follower_alpha = 1.0;
        bool follower_converged = false;
        bool follower_stall = false;
        bool follower_corner = false;
        std::optional<Pose6D> follower_pf_stand;
        std::optional<Pose6D> stage_tcp_target_stand;
        double follower_divergence_pos_m = 0.0;
        double follower_divergence_ang_rad = 0.0;
        double follower_projection_error_m = 0.0;
        double follower_projection_error_rad = 0.0;
        int follower_projection_error_count = 0;
        double follower_actual_lead_m = 0.0;
        double follower_actual_lead_rad = 0.0;
        int follower_actual_lead_error_count = 0;
        bool follower_loading_projection_active = false;
        double follower_contact_shift_m = 0.0;
        std::uint64_t follower_reanchor_count = 0;
        std::uint64_t follower_warm_resume_count = 0;
        bool safety_intervention_recent = false;
        double delta_twist_pending_linear_norm_m = 0.0;
        double delta_twist_pending_angular_norm_rad = 0.0;
        Vec6 delta_twist_step_delta{};
        double delta_twist_step_linear_norm_m = 0.0;
        double delta_twist_step_angular_norm_rad = 0.0;
        double delta_twist_step_yaw_rad = 0.0;
        Vec6 delta_twist_realized_delta{};
        double delta_twist_realized_linear_norm_m = 0.0;
        double delta_twist_realized_angular_norm_rad = 0.0;
        double delta_twist_realized_yaw_rad = 0.0;
        double delta_twist_realized_linear_ratio = 1.0;
        double delta_twist_realized_angular_ratio = 1.0;
        double delta_twist_realized_yaw_ratio = 1.0;
        double delta_twist_phase_sec = 0.0;
        int delta_twist_step_kind = 0;
        int delta_twist_normal_consumed = 0;
        int delta_twist_reserve_consumed = 0;
        double delta_twist_xi_ref_linear_norm_m_s = 0.0;
        double delta_twist_xi_ref_angular_norm_rad_s = 0.0;
        double delta_twist_xi_cmd_linear_norm_m_s = 0.0;
        double delta_twist_xi_cmd_angular_norm_rad_s = 0.0;
        bool delta_twist_saturated = false;
        double delta_twist_lead_linear_norm_m = 0.0;
        double delta_twist_lead_angular_norm_rad = 0.0;
        int delta_twist_feedback_source = 0;
        bool delta_twist_pending_clamped = false;
        bool delta_twist_residual_cleared_on_frame = false;
        bool delta_twist_min_time_to_go_used = false;
        double delta_twist_lin_feedback_cos = 1.0;
        double delta_twist_ang_feedback_cos = 1.0;
        bool delta_twist_xi_ref_clamped_norm = false;
        bool delta_twist_xi_cmd_clamped_norm = false;
        int delta_twist_frame_rows = 0;
        int delta_twist_normal_budget = 0;
        int delta_twist_total_budget = 0;
        int delta_twist_steps_remaining = 0;
        std::uint32_t delta_twist_clamp_mask = 0;
        Vec6 delta_twist_accel_cmd{};
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
// or by reaching the segment's far endpoint within `waypoint_tol_deg` on both arms
// (monotonic, never stalls when the lookahead chord cuts a corner or when an
// asymptotic tracker settles just short of an escape-head waypoint), then returns the
// farthest forward waypoint within `lookahead_deg` of the current pose. `done` is set
// once the pose has settled within `waypoint_tol_deg` of the final waypoint. Stateless
// apart from the in/out `index`, so it is unit-testable in isolation (see
// test_init_motion_pursuit).
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
