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
    // RB_ALLOW_REAL_ROBOT/MOTION + RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION). Does
    // NOT suppress EMS/SOS/soft-estop/collision_occur/unknown-mode faults.
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

// Static stand collision capsule (stand frame, meters). Derived from the stand
// URDF collision boxes (mo_robot_descriptions dual_rb3_730e_stand_ver3).
struct StandCapsuleConfig {
    std::string name;
    std::array<double, 3> p0_m{0.0, 0.0, 0.0};
    std::array<double, 3> p1_m{0.0, 0.0, 0.0};
    double radius_m = 0.0;
};

// A geometric collision capsule (segment endpoints + radius) in some frame.
// Used both as the FK output (stand frame) and as a generic capsule primitive.
struct ArmCapsule {
    std::array<double, 3> p0_m{0.0, 0.0, 0.0};
    std::array<double, 3> p1_m{0.0, 0.0, 0.0};
    double radius_m = 0.0;
};

// One arm collision capsule defined in a URDF LINK frame (link0..link6,
// attachment_site). The server FK-transforms p0_m/p1_m by that frame's placement
// (per arm joints + mount) each tick to get the stand-frame capsule it checks.
// Fit per collision hull from the RB3-730e URDF so the capsules follow the real
// link "dogleg" shape (link2/link4 ship multiple hulls) instead of a single fat
// straight capsule on the joint-origin skeleton. Regenerate with
// scripts/fit_arm_collision_capsules.py.
struct ArmCapsuleConfig {
    std::string frame;
    std::array<double, 3> p0_m{0.0, 0.0, 0.0};
    std::array<double, 3> p1_m{0.0, 0.0, 0.0};
    double radius_m = 0.0;
};

// RB3-730e per-link capsule template (both arms share it; FK'd per arm). Order
// matters: indices feed stand_ignore_bones. Indices 0..1 are link0/link1 (base,
// on/near the stand mount). Fit by scripts/fit_arm_collision_capsules.py.
inline std::vector<ArmCapsuleConfig> defaultRb3ArmCapsules() {
    return {
        {"link0",           {+0.0696, -0.0007, +0.0256}, {-0.0623, +0.0023, +0.0359}, 0.0566},
        {"link1",           {-0.0052, -0.0212, -0.0681}, {-0.0010, +0.0093, +0.0579}, 0.0647},
        {"link2",           {-0.0003, -0.1552, -0.0497}, {+0.0011, -0.0772, +0.0790}, 0.0804},
        {"link2",           {+0.0003, -0.1174, +0.0812}, {-0.0015, -0.1194, +0.2131}, 0.0376},
        {"link2",           {-0.0028, -0.0744, +0.2178}, {+0.0002, -0.1501, +0.3285}, 0.0706},
        {"link3",           {-0.0001, +0.0175, -0.0464}, {+0.0009, -0.0355, +0.0669}, 0.0624},
        {"link4",           {+0.0014, -0.1107, +0.2140}, {-0.0026, -0.0893, +0.3132}, 0.0405},
        {"link4",           {+0.0407, +0.0077, +0.0764}, {-0.0407, +0.0076, +0.0765}, 0.0490},
        {"link4",           {+0.0003, -0.1457, +0.3534}, {+0.0005, -0.0296, +0.3459}, 0.0525},
        {"link4",           {+0.0003, -0.1362, +0.2054}, {-0.0001, +0.0451, +0.1032}, 0.0513},
        {"link5",           {+0.0034, -0.0138, +0.0650}, {-0.0017, +0.0081, -0.0486}, 0.0399},
        {"link6",           {+0.0018, +0.0347, +0.0784}, {-0.0175, -0.0385, +0.0756}, 0.0307},
        // Pika gripper (attachment_site frame): body column, camera (+y nub),
        // jaw housing cross-bar (x), and fingertip bar ending ~3 mm past the tip.
        {"attachment_site", {+0.0000, +0.0000, +0.0000}, {+0.0000, +0.0000, +0.1000}, 0.0350},
        {"attachment_site", {+0.0000, +0.0607, +0.0810}, {+0.0000, +0.0940, +0.0810}, 0.0280},
        {"attachment_site", {-0.1039, +0.0000, +0.1305}, {+0.1039, +0.0000, +0.1305}, 0.0317},
        {"attachment_site", {-0.0594, +0.0000, +0.2286}, {+0.0594, +0.0000, +0.2286}, 0.0220},
    };
}

// Server-side self-collision guard treating stand + left arm + right arm as one
// "self" (the rbpodo controller firmware does not populate op_stat_self_collision).
// Each arm link is approximated as a capsule (segment between consecutive
// kinematic-chain points + radius); the stand as a static capsule list. Checked
// pairs are left<->right, left<->stand, right<->stand — NEVER intra-arm (adjacent
// links touch by construction). A candidate target is refused if any checked pair
// comes within margin_m of each other.
struct SelfCollisionConfig {
    bool enable = false;
    double margin_m = 0.05;
    // Capsule radius per bone (meters). Chain points are [base, j1..j6, tcp] (8
    // points -> 7 bones); index i is the bone from point i to point i+1. The last
    // radius is reused if more bones exist (e.g. a future gripper). Conservative
    // (slightly large) defaults; tune in simulation.
    std::array<double, 7> link_radius_m{0.10, 0.09, 0.08, 0.07, 0.06, 0.06, 0.06};
    SelfCollisionFailPolicy fail_policy = SelfCollisionFailPolicy::ClampToHold;
    // Observe-only: still evaluate and publish clearance/violation telemetry, but
    // do NOT clamp or latch. For tuning radii/margin in simulation against a known
    // collision-free trajectory. Never use monitor_only as a real-motion safety
    // posture.
    bool monitor_only = false;
    // Pair toggles. Arm<->stand checks additionally require a non-empty
    // stand_capsules list (empty list = stand checks are skipped, preserving the
    // arm-arm-only behavior of older configs).
    bool check_left_right = true;
    bool check_left_stand = true;
    bool check_right_stand = true;
    std::vector<StandCapsuleConfig> stand_capsules;
    // Per-link arm collision capsules (both arms share this template; FK'd per
    // arm each tick). Defaults follow the RB3-730e collision hulls so the capsules
    // hug the real link shape; override in YAML to retune. The legacy
    // link_radius_m skeleton model is unused when this is non-empty.
    std::vector<ArmCapsuleConfig> arm_capsules{defaultRb3ArmCapsules()};
    // Arm capsule indices excluded from the arm<->stand check (indices into
    // arm_capsules). Indices 0,1,2 are link0/link1/link2-shoulder: the arm is
    // bolted onto the stand shoulder plates here, so these capsules permanently
    // overlap the stand mount/shoulder capsules and would always self-trigger
    // (a structural, not avoidable, overlap). The reach links (link3+, index 3+)
    // stay checked against the whole stand. See scripts/fit_arm_collision_capsules.py
    // and the arm<->stand clearance probe.
    std::vector<int> stand_ignore_bones{0, 1, 2};

    // URDF mesh self-collision via the async CollisionMonitor (pinocchio + coal).
    // When mesh.enable is true the servo loop uses the mesh monitor (separate
    // thread, off the 2 ms servo_j path) + a velocity barrier INSTEAD of the
    // capsule path above; the capsule code stays compiled but is not evaluated.
    // Barrier params are a SINGLE shared set, common to every motion primitive
    // (TcpPoseTarget, TcpTwistLocal, ...) — speed adaptation comes from the
    // measured closing speed, so nothing is tuned per primitive.
    struct MeshConfig {
        bool enable = false;
        std::string unified_urdf;        // stand+both-arms URDF (e.g. dual_rb3_730e_ver3.urdf)
        std::vector<std::string> package_dirs;  // resolve mesh "../../../meshes" paths
        std::string pika_gripper_mesh;   // optional; attached as a convex hull per arm
        std::string left_prefix = "dual_rb3_730e_left_";
        std::string right_prefix = "dual_rb3_730e_right_";
        std::string stand_frame = "stand";
        // arm geometry whose name contains any of these is NOT paired vs the stand
        std::vector<std::string> stand_ignore_arm_substrings{"link0"};
        // shared velocity-barrier params
        double d_hard_m = 0.005;
        double d_slow_m = 0.025;
        double a_brake_m_s2 = 4.0;
        double hyst_m = 0.005;
        double latency_s = 0.005;
        // Verdict older than this -> hold (recoverable, not a latch). Loose enough
        // to ride out normal OS scheduling jitter of the (non-RT) monitor thread;
        // the monitor normally refreshes every ~1.5 ms.
        double max_staleness_s = 0.050;
        int monitor_core = -1;
        int max_near_pairs = 8;
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
