#pragma once

#include <optional>
#include <string>
#include <vector>
#include "rb_servo/core/types.hpp"

namespace rb_servo {

enum class ServoIoModel {
    Direct,
    Worker
};

struct BackendConfig {
    BackendType backend_type = BackendType::Rbpodo;
    RunMode run_mode = RunMode::Real;

    std::string name;
    std::string ip;
    std::string operation_mode = "real";
    std::string simulator_control_endpoint = "tcp://127.0.0.1:50200";
    // Deprecated compatibility alias. Keep synchronized with simulator_control_endpoint.
    std::string rbsim_control_endpoint = "tcp://127.0.0.1:50200";
    double rbsim_request_timeout_sec = 0.2;
    double rbsim_connect_timeout_sec = 0.2;
    double rbsim_read_timeout_sec = 0.2;
    double rbsim_send_timeout_sec = 0.2;
    double rbsim_stop_timeout_sec = 0.2;
    double rbsim_reset_timeout_sec = 0.2;

    double command_timeout_sec = 0.2;

    JointArray initial_q_deg{};

    double speed_bar = 0.1;

    double servo_t1_sec = 0.002;
    double servo_t2_sec = 0.05;
    double servo_gain = 1.0;
    double servo_alpha = 0.5;

    // Deprecated rbpodo aliases. Kept synchronized with canonical fields while
    // old configs migrate.
    double servo_time_sec = 0.002;
    double servo_lookahead_sec = 0.05;
    double servo_acc = 0.5;

    // rbpodo-only. When true, Cobot::disable_waiting_ack() makes command calls
    // return after socket send instead of waiting for controller ACK.
    bool disable_waiting_ack = false;
    bool allow_controller_simulation_diagnostics_suspect = false;
    bool controller_simulation_treat_unreliable_status_fields_as_unavailable = false;
    // Real (operation_mode: real) physical-motion opt-in mirror of the field above:
    // accept the same vendor-unreliable status fields (op_stat_self_collision shape,
    // robot_time) as UNAVAILABLE instead of latching diagnostics_suspect. Fail-closed,
    // gated by rbpodoSuspectDiagnosticsRealMotionGateOpen (needs operation_mode==real +
    // this config opt-in). Does NOT suppress EMS/SOS/soft-estop/collision_occur/
    // unknown-mode faults.
    bool allow_real_motion_with_suspect_diagnostics = false;
    bool allow_controller_simulation_init_error = false;

    // rbpodo controller-simulation only: tolerate up to N consecutive transient
    // readState misses (no CobotData frame within the read window) by holding the
    // last valid state and staying connected, instead of declaring the controller
    // disconnected on the first miss. A sustained outage still trips after N
    // consecutive misses. 0 = no tolerance (fail-closed, default). Ignored for
    // physical real operation (operation_mode != simulation).
    int max_consecutive_read_misses = 0;
};

struct ArmMountConfig {
    ArmId arm_id = ArmId::Left;
    Pose6D base_pose_in_stand;
};

struct IkSolverConfig {
    bool enable = true;
    int max_iterations = 50;
    double timeout_ms = 2.0;
    double damping = 0.001;
    double position_tolerance_m = 0.001;
    double orientation_tolerance_rad = 0.02;
    JointArray max_step_deg{2.0, 2.0, 2.0, 3.0, 3.0, 4.0};
    // Selective singularity-robust damping (Nakamura-Hanafusa). When the task
    // Jacobian's smallest singular value sigma_min drops below
    // singular_region_eps, damping on the (near-)singular direction is ramped
    // up to at most damping_max, trading Cartesian accuracy in the unachievable
    // direction for joint-space continuity. This stops the DLS step from
    // blowing up along a degenerate direction and flipping to a distant IK
    // branch (a ~5 deg single-tick joint jump for a ~0 mm Cartesian move near a
    // wrist singularity). Outside the singular region the base `damping` is used
    // unchanged, so well-conditioned tracking accuracy is preserved. Both <= 0
    // disables the ramp (pure constant `damping`).
    double singular_region_eps = 0.0;
    double damping_max = 0.0;
    // Observability guard: when > 0, flag the solve (telemetry
    // ik_branch_jump_suspected) if the returned solution differs from the seed
    // by more than this many degrees on any joint. On its own it does NOT alter
    // the solution; it surfaces branch-jump events for diagnosis. The clamp
    // fields below turn it into an actual correction.
    double max_solution_jump_deg = 0.0;
    // Branch-jump CLAMP (acts on the solution, not just observe). When
    // max_solution_jump_deg > 0 and a converged solution jumps more than that
    // from the seed:
    //   1) if branch_jump_damping_scale > 1 and branch_jump_max_retries > 0,
    //      re-solve the SAME tick with damping multiplied by the scale (escalating
    //      per retry) to pull the step back onto the local branch — the first
    //      attempt whose jump is within threshold wins;
    //   2) if it still exceeds the threshold, the OVERSHOOT policy decides:
    //      - branch_jump_rate_limit == true: scale the whole seed->solution joint
    //        delta so the largest per-joint step equals max_solution_jump_deg —
    //        the arm advances toward the solution along the same joint-space
    //        direction at a bounded joint speed (no freeze, no abrupt flip). The
    //        seed advances every tick, so there is no deadlock and the threshold
    //        doubles as the smoothness/lag knob (telemetry reason
    //        "branch_jump_rate_limited", ik_branch_jump_clamped stays false).
    //      - else if branch_jump_clamp_to_seed == true: return the SEED (zero
    //        motion this tick). WARNING: deadlocks under streaming targets — the
    //        frozen seed keeps re-exceeding the threshold, so the arm stays
    //        clamped until the target returns near the frozen pose. Prefer
    //        rate_limit. (telemetry ik_branch_jump_clamped).
    //      - else: return the flagged (most-damped) solution unchanged (allow;
    //        can be rough on high-gain motion).
    // branch_jump_rate_limit takes precedence over branch_jump_clamp_to_seed.
    // All default-off (scale <= 1 / retries <= 0 / rate_limit & clamp false) =>
    // pure observability, behavior unchanged.
    double branch_jump_damping_scale = 0.0;
    int branch_jump_max_retries = 0;
    bool branch_jump_clamp_to_seed = false;
    bool branch_jump_rate_limit = false;
};

struct KinematicsConfig {
    bool enable = false;
    std::string provider = "none";
    std::string urdf = "descriptions/urdf/rb3_730e.urdf";
    std::string base_link = "world";
    std::string tip_link = "tcp";
    std::vector<std::string> joint_names{
        "base_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint",
    };
    std::string q_units = "deg";
    bool publish_tcp = false;
    IkSolverConfig ik;
};

enum class ControllerSimulationTrackingErrorSource {
    Actual,
    Reference
};

enum class ControllerSimulationPhysicalMotionPolicy {
    WarnOnly,
    FaultLatch
};

enum class RbpodoAsyncStreamingMode {
    Disabled,
    SdkAckWorker,
    SocketSendSupervised
};

enum class RbpodoAsyncQueuePolicy {
    LatestWins
};

enum class RbpodoAsyncReferenceSupervisionPolicy {
    WarnOnly,
    FaultLatch
};

struct RbpodoAsyncAckSupervisionConfig {
    bool enable = true;
    double expected_ack_timeout_ms = 50.0;
    double missing_ack_fault_after_ms = 100.0;
    int max_consecutive_missing_ack = 10;
};

struct RbpodoAsyncReferenceSupervisionConfig {
    bool enable = true;
    double q_ref_update_timeout_ms = 50.0;
    double q_ref_target_tolerance_deg = 1.0;
    double q_ref_target_fault_after_ms = 100.0;
    double tcp_ref_update_timeout_ms = 50.0;
    double tcp_ref_target_tolerance_m = 0.02;
    double tcp_ref_target_fault_after_ms = 100.0;
    RbpodoAsyncReferenceSupervisionPolicy policy =
        RbpodoAsyncReferenceSupervisionPolicy::FaultLatch;
};

struct RbpodoAsyncDiagnosticsConfig {
    bool publish_per_command_jsonl = false;
};

struct RbpodoAsyncStreamingConfig {
    bool enable = false;
    RbpodoAsyncStreamingMode mode = RbpodoAsyncStreamingMode::Disabled;
    int rate_hz = 500;
    RbpodoAsyncQueuePolicy queue_policy = RbpodoAsyncQueuePolicy::LatestWins;
    double max_pending_age_ms = 10.0;
    RbpodoAsyncAckSupervisionConfig ack_supervision;
    RbpodoAsyncReferenceSupervisionConfig reference_supervision;
    RbpodoAsyncDiagnosticsConfig diagnostics;
};

enum class SelfCollisionFailPolicy {
    ClampToHold,
    FaultLatch,
};

// Extra collision primitive for the mesh self-collision monitor — geometry the
// URDF does not carry: wrist cameras, cable bundles, the work table/box. Attached
// to a named frame: an ARM frame (e.g. an attachment_site) makes it move with that
// arm (auto-classified left/right by ancestry); "stand"/"world" makes it a static
// obstacle paired against both arms. Shapes are coal primitives.
struct ExtraCollisionConfig {
    std::string name;
    std::string shape = "box";       // box | sphere | capsule | cylinder
    std::string parent_frame;        // URDF frame to attach to (arm frame or stand/world)
    std::array<double, 3> size_m{0.0, 0.0, 0.0};  // box: full extents (x,y,z)
    double radius_m = 0.0;           // sphere/capsule/cylinder
    double length_m = 0.0;           // capsule/cylinder (length between caps / height)
    std::array<double, 3> xyz_m{0.0, 0.0, 0.0};   // offset in the parent frame
    std::array<double, 3> rpy{0.0, 0.0, 0.0};      // orientation in the parent frame
};

// Server-side self-collision guard treating stand + left arm + right arm as one
// "self" (the rbpodo controller firmware does not populate op_stat_self_collision).
// The ONLY implementation is the async URDF-mesh CollisionMonitor (pinocchio +
// coal): every candidate joint target is checked against the dual-arm + stand
// collision geometry and refused (velocity barrier / fault) before it is sent.
// There is no capsule approximation path.
struct SelfCollisionConfig {
    // Master switch for the URDF-mesh self-collision guard. When true the servo
    // loop feeds every candidate target to the monitor and applies the shared
    // velocity barrier; a hard breach, or a stale/absent monitor, fails closed.
    bool enable = false;
    SelfCollisionFailPolicy fail_policy = SelfCollisionFailPolicy::ClampToHold;
    // Observe-only: still evaluate and publish clearance/violation telemetry, but
    // do NOT clamp or latch. For tuning d_hard/d_slow in simulation against a known
    // collision-free trajectory. Never use monitor_only as a real-motion safety
    // posture.
    bool monitor_only = false;

    // URDF mesh self-collision parameters (consumed only when enable is true). The
    // monitor runs on a separate thread (off the 2 ms servo_j path) + a velocity
    // barrier. Barrier params are a SINGLE shared set, common to every motion
    // primitive (TcpPoseTarget, TcpTwistLocal, ...) — speed adaptation comes from
    // the measured closing speed, so nothing is tuned per primitive.
    struct MeshConfig {
        std::string unified_urdf;        // stand+both-arms URDF (e.g. dual_rb3_730e_ver3.urdf)
        std::vector<std::string> package_dirs;  // resolve mesh "../../../meshes" paths
        std::string pika_gripper_mesh;   // optional; attached as a convex hull per arm
        std::string left_prefix = "dual_rb3_730e_left_";
        std::string right_prefix = "dual_rb3_730e_right_";
        std::string stand_frame = "stand";
        // arm geometry whose name contains any of these is NOT paired vs the stand
        std::vector<std::string> stand_ignore_arm_substrings{"link0"};
        // Left/right classification by kinematic-tree ancestry (robust to mesh/link
        // renaming). Default arm root frame = "<prefix>world" if left empty.
        std::string left_arm_root_frame;
        std::string right_arm_root_frame;
        // Intra-arm self-collision (an arm folding onto itself); adjacent links
        // (chain separation < intra_arm_min_chain_separation) are skipped.
        bool check_intra_arm = true;
        int intra_arm_min_chain_separation = 2;
        // Swept-volume guard: samples between consecutive evaluations (1 = endpoint
        // only). >=2 prevents fast motion tunneling a thin obstacle between ticks.
        int swept_samples = 2;  // 1=endpoint, >=2 sweeps (cost ~x per sample)
        // shared velocity-barrier params
        double d_hard_m = 0.005;
        double d_slow_m = 0.025;
        double a_brake_m_s2 = 4.0;
        double hyst_m = 0.005;
        double latency_s = 0.010;
        // Verdict older than this -> hold (recoverable, not a latch). Loose enough
        // to ride out normal OS scheduling jitter of the (non-RT) monitor thread;
        // the monitor normally refreshes every ~1.5 ms.
        double max_staleness_s = 0.050;
        int monitor_core = -1;
        int max_near_pairs = 8;
        // VISUALIZATION ONLY (does not affect the barrier): publish near-pair witness
        // segments to the GUI for any checked pair within this clearance. Decoupled
        // from d_slow so close-call markers stay visible even when the barrier band
        // (d_slow) is tuned tight. Effective threshold = max(this, d_slow).
        double viz_near_pairs_m = 0.06;
        // Extra collision primitives not in the URDF (wrist cameras, cables, table).
        std::vector<ExtraCollisionConfig> extra_collision;
    };
    MeshConfig mesh;
};

enum class FloorConstraintFailPolicy {
    ClampToHold,
    FaultLatch,
};

// One named extra floor-check point, expressed as an offset in the TCP frame.
struct FloorCheckPointConfig {
    std::string name;
    std::array<double, 3> offset_m{};
};

// Stand-frame floor plane constraint: the TCP of either arm must never go below
// z = z_min_m (meters, stand frame), regardless of motion primitive or run mode.
// Tier 1 (hard backstop) FK-checks every candidate joint target at the final
// safety gate; Tier 2 clamps Cartesian targets / negative stand v_z so streaming
// commands slide along the plane. z_min_m is runtime-adjustable via the leaseless
// SetSafetyFloorZ command, bounded to [runtime_min_z_m, runtime_max_z_m].
struct FloorConstraintConfig {
    bool enable = false;
    double z_min_m = 0.010;
    double runtime_min_z_m = 0.0;
    double runtime_max_z_m = 0.5;
    FloorConstraintFailPolicy fail_policy = FloorConstraintFailPolicy::ClampToHold;
    // Observe-only: publish per-arm tcp z / violation telemetry without clamping
    // or latching. Never use monitor_only as a real-motion safety posture.
    bool monitor_only = false;
    // Additional floor-check points expressed in the TCP frame (meters). The
    // TCP point is always checked; each entry adds one more point at
    // tcp_position + R_tcp * offset_m — e.g. the two PIKA gripper fingertips,
    // which dip below the TCP point when the tool rotates. The published
    // per-arm tcp_z_m becomes the LOWEST checked point's z.
    std::vector<FloorCheckPointConfig> tcp_offset_points;
};

// Joint-space SMD profile for the JointTarget primitive (the joint-space
// mirror of cartesian_control.pose_track_smd): the sent target follows the
// commanded goal as a second-order system (mass fixed at 1.0) stepped at the
// servo rate, per joint:
//   ddq = wn^2 * (goal - q) - 2 * zeta * wn * dq
// The per-joint velocity/accel clamps are the max-force saturation; inside the
// limits the zeta/fn dynamics are exact. These are PROFILE limits — the global
// safety.dq_max_deg_s / ddq_max_deg_s2 clamps still apply downstream as the
// outer hard safety bound. Disabled (default): JointTarget keeps the legacy
// full-speed rate-limited ramp at dq_max.
struct JointTargetSmdConfig {
    bool enable = false;
    double damping_ratio = 1.0;        // 1.0 = critical damping (no overshoot)
    double natural_frequency_hz = 0.4;
    JointArray max_velocity_deg_s{30.0, 30.0, 30.0, 45.0, 45.0, 60.0};
    JointArray max_accel_deg_s2{150.0, 150.0, 150.0, 250.0, 250.0, 350.0};
};

struct SafetyConfig {
    JointArray q_min_deg{};
    JointArray q_max_deg{};
    JointArray dq_max_deg_s{};
    JointArray ddq_max_deg_s2{};
    JointArray joint_wrap_period_deg{};

    double command_timeout_sec = 0.2;
    double max_tracking_error_deg = 10.0;

    // mock/rbsim can use SnapToActual for fast iteration. real should use FaultLatch.
    TrackingErrorPolicy tracking_error_policy = TrackingErrorPolicy::FaultLatch;

    bool stop_both_arms_on_single_arm_error = true;
    bool latch_fault_on_robot_state_error = true;
    bool joint_wrap_for_startup_validation = false;
    bool joint_wrap_for_motion_safety = false;
    ControllerSimulationTrackingErrorSource controller_simulation_tracking_error_source =
        ControllerSimulationTrackingErrorSource::Actual;
    ControllerSimulationPhysicalMotionPolicy controller_simulation_physical_motion_policy =
        ControllerSimulationPhysicalMotionPolicy::FaultLatch;
    double controller_simulation_physical_motion_threshold_deg = 0.05;
    // pgmode controller-sim only (opt-in, default false). When the controller-sim
    // motion gate is open, the reference/actual tracking-error divergence is treated
    // as ADVISORY (degraded telemetry + throttled WARN) instead of latching
    // SafetyVerdict::TrackingError. The diagnostics_suspect controller's reference
    // readback lags the commanded joints with no physical motion, so the latch is
    // spurious there. Inert in real mode (gate closed → keeps latching). Does NOT
    // affect the controller_simulation_physical_motion guard, which still latches.
    bool controller_simulation_tracking_error_nonlatching = false;
    SelfCollisionConfig self_collision;
    FloorConstraintConfig floor_constraint;
    JointTargetSmdConfig joint_target_smd;
};

inline constexpr JointArray rbpodoDefaultSafetyJointMinDeg() {
    return JointArray{-360.0, -360.0, -360.0, -360.0, -360.0, -360.0};
}

inline constexpr JointArray rbpodoDefaultSafetyJointMaxDeg() {
    return JointArray{360.0, 360.0, 360.0, 360.0, 360.0, 360.0};
}

struct ServoConfig {
    int rate_hz = 500;
    double command_timeout_sec = 0.2;
    ServoIoModel io_model = ServoIoModel::Direct;
    ControlMode startup_mode = ControlMode::Hold;
    bool send_servo_commands = true;
    bool allow_readonly_faulted_startup = false;
    bool allow_readonly_q_range_violation_startup = false;
    bool allow_readonly_wrong_mode_startup = false;
    bool allow_controller_simulation_motion = false;
    bool allow_controller_simulation_diagnostics_suspect = false;
    bool controller_simulation_treat_unreliable_status_fields_as_unavailable = false;
    // Real physical-motion (operation_mode: real) opt-in; propagated to both backends.
    bool allow_real_motion_with_suspect_diagnostics = false;
    bool controller_simulation_async_supervision_nonlatching = false;
    bool allow_controller_simulation_init_error = false;
    bool allow_controller_simulation_not_activated = false;
    // Per-arm direct-teaching (free-drive). Fail-closed opt-in: when false, the
    // server rejects every Freedrive command. Enable only on configs that accept
    // releasing servo_j authority for operator hand-guiding (see
    // docs/runbooks/freedrive_direct_teaching.md).
    bool allow_freedrive = false;

    bool enable_realtime_priority = true;
    int realtime_priority = 80;
    int cpu_core = -1;
    double worker_read_period_sec = 0.002;

    // Use actual period for logging, but cap filter dt so one late tick does not
    // create an unexpectedly large joint step.
    double filter_dt_min_ratio = 0.5;
    double filter_dt_max_ratio = 1.5;

    // Final-stage moving average over the last N sent joint targets (applied
    // after the safety filter; convex combination keeps limits intact).
    // 0/1 disables. 40 at 500 Hz = 80 ms boxcar, ~40 ms group delay.
    int output_moving_average_window = 0;

    double servo_t1_rate_match_tolerance_ratio = 0.2;
    bool allow_servo_t1_rate_mismatch = false;

    RbpodoAsyncStreamingConfig rbpodo_async_streaming;
};

struct NetworkConfig {
    std::string command_bind = "udp://127.0.0.1:50010";
    std::string state_pub_endpoint = "udp://127.0.0.1:50110";
    // Deprecated compatibility alias. Keep synchronized with the first endpoint.
    std::string state_pub_bind = "udp://127.0.0.1:50110";
    // Canonical UDP state destinations. The first entry mirrors state_pub_endpoint.
    std::vector<std::string> state_pub_endpoints{"udp://127.0.0.1:50110"};
    int state_pub_rate_hz = 20;
    std::vector<std::string> command_source_allowlist{"127.0.0.1/32"};
    double command_timeout_sec = 0.2;
    bool command_source_enforce_lease = false;
    double command_source_lease_timeout_sec = 1.0;
};

struct CommandSourceConfig {
    bool enforce_lease = false;
    double lease_timeout_sec = 1.0;
};

struct LoggingConfig {
    bool enable = true;
    std::string directory = "./logs";
    int flush_period_ms = 100;
    size_t queue_capacity = 4096;
};

struct ForceControlConfig {
    std::string provider = "null";
    bool enable = false;
    int update_rate_hz = 200;

    // Simple admittance fallback used before integrating mo_forcecontroller.
    double admittance_gain_pos = 0.0002;  // m / (N*s) applied as gain * error * dt
    double admittance_gain_rot = 0.0001;  // rad / (Nm*s)
    double force_lpf_alpha = 0.2;

    double max_pos_offset_m = 0.01;
    double max_rot_offset_rad = 0.1;
    double max_pos_step_m = 0.001;
    double max_rot_step_rad = 0.01;
};

struct LinearMoveConfig {
    double min_duration_sec = 0.05;
    double max_duration_sec = 10.0;
    double default_linear_speed_m_s = 0.03;
    double default_angular_speed_rad_s = 0.2;
    double constant_orientation_tolerance_rad = 0.005;
    LinearMoveOrientationMode default_orientation_mode = LinearMoveOrientationMode::Constant;
};

struct CircleMoveConfig {
    bool allow_in_simulation = true;
    bool allow_in_real = false;
    double max_diameter_m = 0.20;
    double min_period_sec = 3.0;
};

// Spring-Mass-Damper smoothing for the streaming TcpPoseTarget path: received
// command deltas integrate into a goal pose, and the published target follows
// that goal as a second-order system (mass fixed at 1.0) stepped at the servo
// rate. Translation and rotation are tunable independently.
struct PoseTrackSmdConfig {
    bool enable = false;
    double damping_ratio_linear = 1.0;
    double natural_frequency_linear_hz = 0.5;
    double damping_ratio_angular = 1.0;
    double natural_frequency_angular_hz = 0.5;
    // Saturation clamps applied as vector-norm limits each tick. Mass is fixed
    // at 1.0, so the accel clamp IS the max-force clamp. Inside the limits the
    // zeta/fn dynamics are exactly preserved. 0 = unlimited.
    double max_linear_velocity_m_s = 0.0;
    double max_linear_accel_m_s2 = 0.0;
    double max_angular_velocity_rad_s = 0.0;
    double max_angular_accel_rad_s2 = 0.0;
    // Velocity feedforward: damp on the velocity ERROR (goal_dot - x_dot) rather
    // than the absolute x_dot, so the error dynamics become
    //   e_ddot + 2*zeta*wn*e_dot + wn^2*e = goal_ddot.
    // A constant-velocity (ramp) goal then has ZERO steady-state lag, while jitter
    // (the goal_ddot term) still rolls off through the 2nd-order response. This
    // decouples natural_frequency from tracking accuracy: fn becomes a pure
    // smoothing dial that can be lowered for more smoothing without adding lag.
    // The goal velocity is estimated internally from the per-tick goal delta
    // (auto stand/body frame; valid because every caller integrates the goal once
    // per step()). Off by default = exact legacy 2nd-order SMD.
    bool velocity_feedforward = false;
};

enum class CartesianLimitPolicy {
    Clamp,
    Reject
};

enum class CartesianVelocityTargetIntegrationMode {
    MeasuredActual,
    MeasuredActualLookahead,
    PreviousCommand
};

enum class CartesianCommandActualErrorPolicy {
    Reset,
    Fault
};

enum class CartesianControllerSimulationStateSource {
    Actual,
    Reference
};

struct CartesianControlConfig {
    bool enable = true;
    bool allow_in_simulation = true;
    bool allow_in_real = false;
    bool allow_in_controller_simulation = false;
    bool enable_server_side_circle_track = false;
    bool enable_benchmark_primitives = false;
    double warn_ik_duration_us = 3000.0;
    double fail_ik_duration_us = 0.0;
    // Deprecated compatibility field. New control code uses path_kp_pos/path_kp_ori.
    double path_kp = 6.0;
    double path_kp_pos = 6.0;
    double path_kp_ori = 6.0;
    double twist_orientation_hold_kp = 6.0;
    double twist_angular_deadband_rad_s = 0.0001;
    // Route the streaming twist through the SMD pose tracker instead of velocity
    // IK + joint integration: integrate the (clamped) twist into a stand-frame
    // pose goal each tick, smooth it with the pose_track_smd filter (same as
    // streaming TcpPoseTarget), then position-IK the smoothed pose. Default off
    // (behavior-preserving). Uses pose_track_smd's zeta/fn for the filter.
    bool twist_via_smd_enable = false;
    // Anti-windup for the twist_via_smd goal integrator. Clamp the integrated
    // pose goal so it never LEADS the measured TCP by more than these budgets
    // (position m / orientation rad). In this feedforward chain the goal would
    // otherwise run away when the arm stalls (IK clamp / fault / can't track) and
    // lurch on recovery; clamping the goal to measured+budget is the
    // back-calculation feedback that bounds both. Normal tracking lead is only
    // the SMD lag (~mm / sub-deg), well under budget, so it never engages there.
    // Both default 0 = disabled (behavior-preserving); e.g. 0.05 m / 0.2 rad.
    double twist_smd_goal_max_lead_m = 0.0;
    double twist_smd_goal_max_lead_rad = 0.0;
    double velocity_damping = 0.01;
    double max_twist_linear_m_s = 0.03;
    double max_twist_angular_rad_s = 0.2;
    double max_linear_move_speed_m_s = 0.05;
    double max_angular_move_speed_rad_s = 0.3;
    std::optional<double> max_cartesian_step_m;
    std::optional<double> max_cartesian_step_rad;
    CartesianLimitPolicy exceed_limit_policy = CartesianLimitPolicy::Clamp;
    CartesianVelocityTargetIntegrationMode velocity_target_integration =
        CartesianVelocityTargetIntegrationMode::PreviousCommand;
    CartesianControllerSimulationStateSource controller_simulation_servo_state_source =
        CartesianControllerSimulationStateSource::Actual;
    CartesianControllerSimulationStateSource controller_simulation_divergence_source =
        CartesianControllerSimulationStateSource::Actual;
    double velocity_target_lookahead_sec = 0.04;
    JointArray max_command_actual_error_deg{5.0, 5.0, 5.0, 8.0, 8.0, 10.0};
    bool reset_velocity_integrator_on_mode_change = true;
    CartesianCommandActualErrorPolicy command_actual_error_policy =
        CartesianCommandActualErrorPolicy::Reset;
    LinearMoveConfig linear_move;
    CircleMoveConfig circle_move;
    PoseTrackSmdConfig pose_track_smd;
};

struct DualArmConfig {
    BackendConfig left_robot;
    BackendConfig right_robot;

    ArmMountConfig left_mount;
    ArmMountConfig right_mount;

    ServoConfig servo;
    SafetyConfig safety;
    NetworkConfig network;
    CommandSourceConfig command_source;
    LoggingConfig logging;
    ForceControlConfig force_control;
    CartesianControlConfig cartesian_control;
    KinematicsConfig kinematics;
};

DualArmConfig loadConfigFromYaml(const std::string& path);

}  // namespace rb_servo
