#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace rb_servo {

constexpr int kDof = 6;
using JointArray = std::array<double, kDof>;
using JointBoolArray = std::array<bool, kDof>;

struct JointRangeNormalization {
    double normalized_value_deg = 0.0;
    bool was_wrapped = false;
    bool equivalent_in_range = false;
};

inline bool jointValueInRange(double value_deg, double min_deg, double max_deg) {
    return std::isfinite(value_deg) &&
           std::isfinite(min_deg) &&
           std::isfinite(max_deg) &&
           value_deg >= min_deg &&
           value_deg <= max_deg;
}

inline JointRangeNormalization normalizeJointForRange(
    double value_deg,
    double min_deg,
    double max_deg,
    double wrap_period_deg
) {
    JointRangeNormalization result;
    result.normalized_value_deg = value_deg;
    result.equivalent_in_range = jointValueInRange(value_deg, min_deg, max_deg);
    if (result.equivalent_in_range) {
        return result;
    }

    if (!std::isfinite(value_deg) ||
        !std::isfinite(min_deg) ||
        !std::isfinite(max_deg) ||
        !std::isfinite(wrap_period_deg) ||
        wrap_period_deg <= 0.0 ||
        max_deg < min_deg) {
        return result;
    }

    double normalized = value_deg -
        std::floor((value_deg - min_deg) / wrap_period_deg) * wrap_period_deg;
    if (normalized < min_deg) {
        normalized += wrap_period_deg;
    }
    if (normalized >= min_deg + wrap_period_deg) {
        normalized -= wrap_period_deg;
    }

    constexpr double kWrapEpsilon = 1e-9;
    result.normalized_value_deg = normalized;
    result.was_wrapped = std::abs(normalized - value_deg) > kWrapEpsilon;
    result.equivalent_in_range =
        normalized >= min_deg - kWrapEpsilon &&
        normalized <= max_deg + kWrapEpsilon;
    return result;
}

struct Vec6 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
};

enum class ArmId { Left, Right };
enum class RunMode { Real, Simulation, Mock };
enum class BackendType {
    Rbpodo,
    Mock,
};

enum class ControlMode {
    Idle,
    Hold,
    ArmMotion,
    DisarmMotion,
    JointTarget,
    TcpPoseTarget,
    TcpLinearMove,
    EmergencyStop,
    ResetFault,
    SetSafetyFloorZ,
    // Leaseless runtime enable/disable of the stand-frame floor plane
    // (safety.floor_constraint). Only effective when the floor is opted in at config
    // (floor_constraint.enable=true); it toggles whether that floor is enforced.
    SetSafetyFloorEnabled,
    // Leaseless runtime adjustment of the stand-frame ROI box bounds
    // (safety.roi_box), bounded server-side to the configured runtime envelope.
    SetSafetyRoiBounds,
    // Leaseless runtime set/enable of the user-defined tilted floor plane
    // (safety.user_floor_constraint): carries a stand-frame point + unit normal +
    // margin + enable flag, validated server-side by validateUserFloorPlaneRequest.
    SetUserSafetyFloorPlane,
    // Per-arm direct-teaching (free-drive). Releases servo_j control on the
    // addressed arm's controller (freedrive_teach_on) so an operator can hand-
    // guide it, then re-acquires it (freedrive_teach_off) with a target resync.
    // Sticky server state; carried by ArmCommand.freedrive_on.
    Freedrive
};

enum class JointTargetProfile {
    Direct,
    InitMotion
};

enum class ServerMotionState {
    Disconnected,
    ConnectedHold,
    ArmedHold,
    Running,
    FaultLatched,
    EmergencyLatched
};

enum class ForceControlMode {
    Off,
    Admittance,
    Impedance,
    ExternalForceSafety
};

enum class RobotConnectionState {
    Disconnected,
    Connected,
    Error
};

enum class BackendAckPolicy {
    BackendDefault,
    Wait,
    Disabled
};

enum class RbpodoAsyncStreamingSupervisionState {
    Ok,
    Warning,
    Fault
};

enum class SafetyVerdict {
    Ok,
    JointLimitClamped,
    TrackingError,
    RobotStateError,
    SendFailure,
    EmergencyStop,
    FaultLatched,
    InvalidCommand,
    CartesianUnavailable,
    IkFailed,
    SelfCollision,
    FloorViolation,
    RoiViolation,
    UnknownError
};

enum class FaultDomain {
    None,
    SafetyPolicy,
    Backend,
    RobotState,
    Command,
    Kinematics,
    Emergency
};

enum class TrackingErrorPolicy {
    SnapToActual,
    FaultLatch
};

enum class LinearMoveOrientationMode {
    Constant,
    Slerp
};

struct Pose6D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    // Optional canonical orientation for state publication. RPY remains for
    // display and legacy command compatibility.
    std::optional<std::array<double, 4>> quaternion_xyzw;
};

struct Wrench6D {
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
};

struct RbpodoRawDiagnostics {
    double time_sec = 0.0;
    int real_vs_simulation_mode = 0;
    int init_state_info = 0;
    int init_error = 0;
    int op_stat_sos_flag = 0;
    int op_stat_ems_flag = 0;
    int op_stat_soft_estop_occur = 0;
    int op_stat_collision_occur = 0;
    int op_stat_self_collision = 0;
};

struct RbpodoDiagnosticsSnapshot {
    bool diagnostics_valid = true;
    bool diagnostics_suspect = false;
    std::string reason;
    std::string error_name;
    int stable_error_code = 0;
    RbpodoRawDiagnostics raw;
    std::vector<std::string> unavailable_fields;
};

struct ForceControlAxis {
    bool x = false;
    bool y = false;
    bool z = false;
    bool roll = false;
    bool pitch = false;
    bool yaw = false;
};

struct ForceControlCommand {
    ForceControlMode mode = ForceControlMode::Off;
    Wrench6D target_wrench;
    ForceControlAxis enabled_axis;

    // Force controller output clamps. They are applied before Cartesian IK.
    double max_pos_offset_m = 0.01;
    double max_rot_offset_rad = 0.1;
    double max_pos_step_m = 0.001;
    double max_rot_step_rad = 0.01;
};

struct RobotState {
    ArmId arm_id = ArmId::Left;

    uint64_t host_time_ns = 0;
    uint64_t robot_time_ns = 0;

    JointArray q_actual_deg{};
    JointArray q_target_deg{};
    JointArray dq_actual_deg_s{};
    bool q_actual_valid = false;
    bool q_ref_valid = false;
    std::string q_ref_source;
    std::string rbpodo_sdk_state_source;
    std::string rbpodo_state_decode_policy;
    bool has_valid_joint_state = false;

    // Legacy aliases for actual TCP FK from q_actual_deg.
    std::optional<Pose6D> tcp_base;
    std::optional<Pose6D> tcp_stand;
    std::optional<Pose6D> tcp_actual_base;
    std::optional<Pose6D> tcp_actual_stand;
    // Controller/internal reference TCP FK from q_target_deg / rbpodo jnt_ref.
    std::optional<Pose6D> tcp_ref_base;
    std::optional<Pose6D> tcp_ref_stand;
    // Commanded TCP FK from the joints actually sent this cycle (q_sent). Clean of
    // the controller's noisy jnt_ref readback; used for at-rest-stable display in
    // controller simulation.
    std::optional<Pose6D> tcp_command_stand;
    bool has_valid_tcp_pose = false;
    bool tcp_actual_valid = false;
    bool tcp_ref_valid = false;
    bool tcp_deferred = true;
    double fk_duration_us = 0.0;
    Wrench6D wrench_tcp;

    RobotConnectionState connection_state = RobotConnectionState::Disconnected;

    bool servo_enabled = false;
    // Controller motion state (rbpodo sdata.robot_state): 1 = Idle (no motion
    // command), 3 = executing motion, 0 = unknown/not reported. Gates direct-
    // teaching entry — freedrive_teach_on requires the controller to be idle.
    int controller_motion_state = 0;
    // Controller free-drive state (rbpodo sdata.is_freedrive_mode == 1). The
    // ground-truth confirmation that direct teaching actually engaged.
    bool controller_freedrive_on = false;
    bool has_error = false;
    std::optional<bool> fault_recoverable;
    std::string lifecycle_state;
    int error_code = 0;
    std::string motion_readiness_error_kind;
    std::string motion_readiness_error_name;
    std::string diagnostic_error_source;
    std::optional<RbpodoDiagnosticsSnapshot> rbpodo_diagnostics;
};

struct CartesianSolveTelemetry {
    bool attempted = false;
    bool success = false;
    std::string status = "not_attempted";
    std::string reason;
    double fk_duration_us = 0.0;
    double ik_duration_us = 0.0;
    int ik_iterations = 0;
    double position_error_m = 0.0;
    double orientation_error_rad = 0.0;
    // Conditioning / singularity-robust-damping diagnostics (last IK solve).
    double ik_min_singular_value = 0.0;
    double ik_applied_damping = 0.0;
    double ik_solution_jump_deg = 0.0;
    bool ik_branch_jump_suspected = false;
    bool ik_branch_jump_clamped = false;
    bool ik_timed_out = false;
    bool ik_warn_duration_exceeded = false;
    bool ik_fail_duration_exceeded = false;
    double warn_ik_duration_us = 0.0;
    double fail_ik_duration_us = 0.0;
    bool path_active = false;
    double path_s = 0.0;
    double path_position_error_m = 0.0;
    double path_orientation_error_rad = 0.0;
    double path_line_deviation_m = 0.0;
    bool path_done = false;
    double linear_move_duration_sec = 0.0;
    double linear_move_elapsed_sec = 0.0;
    std::string orientation_mode;
    bool floor_vz_clamped = false;
    std::string floor_lowest_point = "tcp";
    double floor_lowest_z_m = std::numeric_limits<double>::quiet_NaN();
    bool floor_goal_clamped = false;
    double goal_minus_measured_pos_m = 0.0;
    double goal_minus_measured_ori_rad = 0.0;
    std::string cartesian_velocity_integration_mode;
    std::string cartesian_servo_state_source = "actual";
    std::string cartesian_divergence_source = "actual";
    bool q_reference_for_servo_valid = false;
    bool q_integrator_valid = false;
    std::string integrator_reset_reason;
    uint64_t integrator_resets_total = 0;
    uint64_t integrator_clamps_total = 0;
    uint64_t integrator_divergence_total = 0;
    double max_command_actual_error_deg_observed = 0.0;
    double command_reference_error_deg_observed = 0.0;
    double physical_command_actual_error_deg_observed = 0.0;
    double velocity_target_lookahead_sec = 0.0;
    // --- A/B/C separation telemetry (Patch 4). Populated by the pose-track SMD path
    // and the final output moving-average stage; absent/false otherwise. Pure
    // telemetry — does not affect control. ---
    bool smd_active = false;
    std::optional<Pose6D> smd_goal_stand;   // integrated SMD goal (B input)
    std::optional<Pose6D> smd_ref_stand;    // SMD step output, BEFORE IK (B output)
    bool smd_velocity_feedforward_used = false;
    bool smd_linear_velocity_clipped = false;
    bool smd_linear_accel_clipped = false;
    bool smd_angular_velocity_clipped = false;
    bool smd_angular_accel_clipped = false;
    bool smd_goal_linear_velocity_ff_clipped = false;
    bool smd_goal_angular_velocity_ff_clipped = false;
    double smd_goal_linear_velocity_norm_m_s = 0.0;
    double smd_goal_angular_velocity_norm_rad_s = 0.0;
    uint64_t smd_reanchor_count = 0;
    // Final-stage output moving average (C). q_target before/after the boxcar.
    bool output_ma_present = false;
    int output_ma_window = 0;
    JointArray q_target_before_output_ma_deg{};
    JointArray q_target_after_output_ma_deg{};
};

struct SafetyTrackingTelemetry {
    std::string tracking_error_source = "actual";
    bool tracking_error_source_valid = true;
    std::string tracking_error_reason;
    double command_reference_tracking_error_deg = 0.0;
    double physical_command_actual_error_deg = 0.0;
    bool controller_simulation_physical_motion_detected = false;
};

struct ArmCommand {
    ArmId arm_id = ArmId::Left;

    uint64_t seq = 0;
    uint64_t host_time_ns = 0;

    ControlMode mode = ControlMode::Hold;

    JointArray q_target_deg{};
    JointTargetProfile joint_target_profile = JointTargetProfile::Direct;

    Pose6D tcp_target_stand;
    double linear_move_duration_sec = 0.0;
    double linear_move_linear_speed_m_s = 0.0;
    double linear_move_angular_speed_rad_s = 0.0;
    LinearMoveOrientationMode linear_move_orientation_mode = LinearMoveOrientationMode::Constant;

    ForceControlCommand force_control;

    double gripper_target = 0.0;
    bool has_gripper = false;
    double timeout_sec = 0.2;

    // Freedrive (direct-teaching) request payload. When mode == Freedrive, this
    // arm's sticky server free-drive state is set to freedrive_on. has_freedrive
    // is true only when the parser saw an explicit boolean (so a bare Freedrive
    // command without the flag is rejected rather than silently treated as off).
    bool freedrive_on = false;
    bool has_freedrive = false;

    // Parsed command validation flags. A command parser must set these true only
    // when the corresponding array was present and had the expected size.
    bool has_joint_target = false;
    bool has_tcp_target = false;
    bool has_linear_move_duration = false;
    bool has_linear_move_linear_speed = false;
    bool has_linear_move_angular_speed = false;
    bool has_linear_move_orientation_mode = false;
};

struct CommandSourceMetadata {
    std::string source_id;
    std::string session_id;
    std::string lease_token;
    std::optional<int> source_priority;
};

struct CommandSourceLeaseState {
    bool enforce_lease = false;
    bool active = false;
    bool command_requires_lease = false;
    bool command_has_lease = true;
    std::string source_id;
    std::string session_id;
    std::string lease_token;
    uint64_t acquired_time_ns = 0;
    uint64_t expires_time_ns = 0;
    std::string verdict = "Ok";
    std::string reason;
};

struct DualArmCommand {
    uint64_t seq = 0;
    uint64_t host_time_ns = 0;

    CommandSourceMetadata source;
    CommandSourceLeaseState lease;

    ArmCommand left;
    ArmCommand right;

    // SetSafetyFloorZ payload: requested stand-frame floor plane height (meters).
    double floor_z_m = 0.0;
    bool has_floor_z = false;

    // SetSafetyFloorEnabled payload: runtime enforce on/off for the stand floor.
    bool floor_enabled = false;
    bool has_floor_enabled = false;

    // SetSafetyRoiBounds payload (top-level: the ROI box is global, not per-arm):
    // requested stand-frame axis-aligned box bounds [min, max] in meters.
    std::array<double, 3> roi_min_m{};
    std::array<double, 3> roi_max_m{};
    bool has_roi_bounds = false;

    // SetUserSafetyFloorPlane payload (top-level: the plane is global, not per-arm):
    // a stand-frame point + unit normal defining the half-space n.(p-point) >= margin,
    // plus an enable flag (false turns the constraint off unconditionally).
    std::array<double, 3> user_floor_point_m{};
    std::array<double, 3> user_floor_normal{0.0, 0.0, 1.0};
    double user_floor_margin_m = 0.0;
    bool user_floor_enable = false;
    bool has_user_floor_plane = false;

    // AcquireLease / ReleaseLease packets are pure lease management. They must
    // never enter the command buffer: the buffer is latest-wins, so their
    // parsed Hold modes would overwrite an in-flight motion command.
    bool lease_admin_only = false;

    // Deprecated in v3. Commands are treated as coupled by default: if a packet
    // becomes stale, both arms hold. Per-arm command streams should use separate
    // timestamps in a future binary protocol.
    bool coupled_timeout = true;
};

struct ServoTarget {
    JointArray left_q_target_deg{};
    JointArray right_q_target_deg{};
};

struct BackendCallSnapshot {
    bool ok = true;
    bool accepted = true;
    std::string backend_error_kind = "None";
    std::string error_name = "None";
    std::string error_code;
    std::string error_message;
    double duration_us = 0.0;
    std::string state_after_source = "none";
    BackendAckPolicy ack_policy = BackendAckPolicy::BackendDefault;
    bool ack_observed = false;
    bool controller_acceptance_observed = false;
    double ack_wait_duration_us = 0.0;
    bool rbpodo_waiting_ack = false;
    std::string acceptance_semantics = "unknown";
};

struct ArmWorkerTelemetry {
    uint64_t worker_command_drops_total = 0;
    uint64_t worker_pending_overwrites_total = 0;
    uint64_t worker_last_dropped_seq = 0;
    uint64_t worker_last_enqueued_seq = 0;
    uint64_t worker_last_dispatched_seq = 0;
    uint64_t worker_last_completed_seq = 0;
    std::string worker_queue_policy = "latest_wins";
};

struct RbpodoAsyncStreamingTelemetry {
    uint64_t commands_enqueued_total = 0;
    uint64_t commands_sent_total = 0;
    uint64_t commands_acked_total = 0;
    uint64_t commands_socket_sent_total = 0;
    uint64_t commands_dropped_total = 0;
    uint64_t commands_overwritten_total = 0;
    uint64_t ack_timeout_count = 0;
    uint64_t missing_ack_count = 0;
    uint64_t q_ref_watchdog_miss_count = 0;
    uint64_t tcp_ref_watchdog_miss_count = 0;
    uint64_t last_command_seq = 0;
    uint64_t last_sent_seq = 0;
    uint64_t last_ack_seq = 0;
    uint64_t first_goal_command_send_ns = 0;
    uint64_t last_goal_command_send_ns = 0;
    uint64_t first_worker_send_ns = 0;
    uint64_t last_worker_send_ns = 0;
    uint64_t goal_window_commands_sent = 0;
    uint64_t goal_window_commands_acked = 0;
    uint64_t last_q_ref_update_host_time_ns = 0;
    uint64_t last_tcp_ref_update_host_time_ns = 0;
    uint64_t last_socket_send_host_time_ns = 0;
    double q_ref_update_age_ms = 0.0;
    double tcp_ref_update_age_ms = 0.0;
    double q_ref_target_error_deg_max = 0.0;
    double tcp_ref_target_error_m = 0.0;
    double last_async_send_duration_us = 0.0;
    double last_async_ack_duration_us = 0.0;
    double max_async_send_duration_us = 0.0;
    double max_async_ack_duration_us = 0.0;
    std::string last_controller_acceptance_semantics;
    std::string last_async_acceptance_semantics;
    std::string command_phase;
    std::string last_send_result;
    std::string last_ack_result;
    std::string last_failure;
    uint64_t worker_backlog = 0;
    double max_pending_age_ms_observed = 0.0;
    RbpodoAsyncStreamingSupervisionState supervision_state =
        RbpodoAsyncStreamingSupervisionState::Ok;
    RbpodoAsyncStreamingSupervisionState reference_supervision_state =
        RbpodoAsyncStreamingSupervisionState::Ok;
    std::string reference_supervision_reason;
    uint64_t reference_supervision_fault_count = 0;
};

struct BackendTransportTelemetry {
    uint64_t connect_attempts_total = 0;
    uint64_t connect_failures_total = 0;
    uint64_t connect_attempts_suppressed_total = 0;
    uint64_t connections_opened_total = 0;
    uint64_t reconnects_total = 0;
    uint64_t requests_total = 0;
    uint64_t read_syscalls_total = 0;
    uint64_t write_syscalls_total = 0;
    std::string last_connect_error_name;
    std::string last_connect_error_message;
    uint64_t next_connect_attempt_ns = 0;
    uint64_t next_connect_attempt_delay_ms = 0;
    std::string last_transport_error_kind;
};

struct LatchedFaultContextSnapshot {
    std::string verdict = "Ok";
    std::string domain = "None";
    std::string arm = "left";
    std::string backend_op = "ReadState";
    std::string backend_error_kind = "None";
    std::string backend_error_name = "None";
    std::string backend_error_code;
    bool retryable = false;
    bool recoverable = false;
    bool robot_fault = false;
    bool transport_fault = false;
    std::string state_after_source = "none";
    std::string reason;
};

struct JointRangeViolation {
    int joint = 0;
    double value_deg = 0.0;
    double min_deg = 0.0;
    double max_deg = 0.0;
};

struct JointRangeWrapped {
    int joint = 0;
    double raw_deg = 0.0;
    double normalized_deg = 0.0;
    double period_deg = 0.0;
};

struct ArmStartupValidationSnapshot {
    bool acquisition_ok = false;
    bool motion_ready = false;
    bool read_only_diagnostic = false;
    bool allowed_unsafe_startup = false;
    std::vector<std::string> invalid_reasons;
    std::vector<JointRangeViolation> q_range_violations;
    std::vector<JointRangeWrapped> q_range_wrapped;
    std::optional<JointArray> q_actual_normalized_for_safety_deg;
    std::string diagnostic_error_source;
};

struct StartupValidationSnapshot {
    bool acquisition_ok = false;
    bool motion_ready = false;
    bool read_only_diagnostic = false;
    bool allowed_unsafe_startup = false;
    ArmStartupValidationSnapshot left;
    ArmStartupValidationSnapshot right;
};

struct ServoSample {
    uint64_t tick = 0;
    uint64_t loop_start_time_ns = 0;
    uint64_t loop_end_time_ns = 0;

    RobotState left_state;
    RobotState right_state;

    DualArmCommand command;

    JointArray left_sent_q_deg{};
    JointArray right_sent_q_deg{};

    bool left_send_ok = false;
    bool right_send_ok = false;
    BackendCallSnapshot left_last_read;
    BackendCallSnapshot right_last_read;
    BackendCallSnapshot left_last_send;
    BackendCallSnapshot right_last_send;
    std::string left_send_error_kind;
    std::string left_send_error_name;
    std::string left_send_error_code;
    std::string left_send_error_message;
    std::string right_send_error_kind;
    std::string right_send_error_name;
    std::string right_send_error_code;
    std::string right_send_error_message;
    CartesianSolveTelemetry left_cartesian_solve;
    CartesianSolveTelemetry right_cartesian_solve;
    SafetyTrackingTelemetry left_safety_tracking;
    SafetyTrackingTelemetry right_safety_tracking;
    bool send_suppressed = false;
    std::string send_policy = "send_servo_j";
    uint64_t left_send_start_ns = 0;
    uint64_t left_send_end_ns = 0;
    uint64_t right_send_start_ns = 0;
    uint64_t right_send_end_ns = 0;
    double send_skew_us = 0.0;
    double left_send_duration_us = 0.0;
    double right_send_duration_us = 0.0;
    ArmWorkerTelemetry left_worker_telemetry;
    ArmWorkerTelemetry right_worker_telemetry;
    RbpodoAsyncStreamingTelemetry left_async_streaming;
    RbpodoAsyncStreamingTelemetry right_async_streaming;
    std::optional<BackendTransportTelemetry> left_transport_telemetry;
    std::optional<BackendTransportTelemetry> right_transport_telemetry;

    double period_ms = 0.0;
    double jitter_ms = 0.0;
    double filter_dt_ms = 0.0;

    SafetyVerdict safety_verdict = SafetyVerdict::Ok;
    ServerMotionState motion_state = ServerMotionState::Disconnected;
    bool fault_latched = false;
    bool async_supervision_degraded = false;
    bool tracking_error_degraded = false;
    std::string fault_reason;
    std::optional<LatchedFaultContextSnapshot> latched_fault_context;
    std::optional<LatchedFaultContextSnapshot> left_latched_fault_context;
    std::optional<LatchedFaultContextSnapshot> right_latched_fault_context;
};

// One near pair from the URDF mesh self-collision monitor: the two closest
// witness points (stand frame) on the two geometries + their signed clearance.
// Lets a viewer draw the close-call segments over the URDF meshes (the mesh-mode
// analogue of the capsule list above).
struct SelfCollisionNearPairViz {
    std::string name_a;
    std::string name_b;
    std::array<double, 3> p_a_m{};
    std::array<double, 3> p_b_m{};
    double clearance_m = 0.0;
    bool external = false;  // arm<->external obstacle (floor) vs robot self-collision
};

struct ServoSnapshot {
    uint64_t tick = 0;
    uint64_t loop_start_time_ns = 0;
    uint64_t loop_end_time_ns = 0;

    RobotState left_state;
    RobotState right_state;

    DualArmCommand command;

    JointArray left_sent_q_deg{};
    JointArray right_sent_q_deg{};
    JointArray left_prev_sent_q_deg{};
    JointArray right_prev_sent_q_deg{};

    double period_ms = 0.0;
    double jitter_ms = 0.0;
    double filter_dt_ms = 0.0;

    SafetyVerdict safety_verdict = SafetyVerdict::Ok;
    ServerMotionState motion_state = ServerMotionState::Disconnected;
    bool fault_latched = false;
    bool async_supervision_degraded = false;
    bool tracking_error_degraded = false;
    // Per-arm direct-teaching (free-drive) sticky state. While true, that arm's
    // controller is in freedrive_teach_on and the server sends no servo_j to
    // either controller (send_policy == "freedrive").
    bool left_freedrive_active = false;
    bool right_freedrive_active = false;
    // Per-arm free-drive lifecycle stage (off/arming_quiesce/arming_confirm/
    // active/exiting) and last abort/failure note, for operator telemetry.
    std::string left_freedrive_stage = "off";
    std::string right_freedrive_stage = "off";
    std::string freedrive_note;
    SafetyVerdict latched_fault_reason = SafetyVerdict::Ok;
    std::string fault_reason;

    // Dual-arm self-collision guard telemetry (safety.self_collision).
    bool self_collision_enabled = false;
    bool self_collision_checked = false;
    bool self_collision_violated = false;
    double self_collision_min_clearance_m = 0.0;
    double self_collision_margin_m = 0.0;
    int self_collision_left_bone = -1;
    std::string self_collision_pair;
    std::string self_collision_stand_capsule;
    int self_collision_right_bone = -1;
    // Closest bone-axis points of the min-clearance pair (stand frame), on the
    // members themselves; a = pair's first member (arm), b = second member
    // (other arm / stand). Their gap = min_clearance + both capsule radii.
    bool self_collision_has_closest_points = false;
    std::array<double, 3> self_collision_closest_point_a_m{};
    std::array<double, 3> self_collision_closest_point_b_m{};
    // URDF mesh self-collision (safety.self_collision): closest near pairs from
    // the async CollisionMonitor (stand frame). Empty when the guard is disabled
    // or no pair is within the slow zone.
    bool self_collision_mesh = false;
    std::vector<SelfCollisionNearPairViz> self_collision_near_pairs;

    // Stand-frame floor plane constraint telemetry (safety.floor_constraint).
    bool floor_constraint_enabled = false;
    bool floor_constraint_monitor_only = false;
    double floor_constraint_z_min_m = 0.0;         // effective (runtime) plane height
    double floor_constraint_config_z_min_m = 0.0;  // startup config value
    double floor_constraint_runtime_min_z_m = 0.0;
    double floor_constraint_runtime_max_z_m = 0.0;
    bool floor_constraint_left_checked = false;
    bool floor_constraint_left_violated = false;
    double floor_constraint_left_tcp_z_m = 0.0;  // lowest checked point z
    std::string floor_constraint_left_lowest_point;
    std::array<double, 3> floor_constraint_left_lowest_point_m{};  // lowest point xyz (stand)
    bool floor_constraint_right_checked = false;
    bool floor_constraint_right_violated = false;
    double floor_constraint_right_tcp_z_m = 0.0;  // lowest checked point z
    std::string floor_constraint_right_lowest_point;
    std::array<double, 3> floor_constraint_right_lowest_point_m{};  // lowest point xyz (stand)
    uint64_t floor_constraint_clamp_count = 0;
    std::string floor_constraint_last_set_reject_reason;

    // Stand-frame ROI box (workspace limit) telemetry (safety.roi_box).
    bool roi_box_enabled = false;
    bool roi_box_monitor_only = false;
    std::array<double, 3> roi_box_min_m{};         // effective (runtime) bounds
    std::array<double, 3> roi_box_max_m{};
    std::array<double, 3> roi_box_runtime_min_m{};  // SetSafetyRoiBounds envelope
    std::array<double, 3> roi_box_runtime_max_m{};
    bool roi_box_left_checked = false;
    bool roi_box_left_violated = false;
    double roi_box_left_min_margin_m = 0.0;  // closest face margin (>=0 inside)
    std::string roi_box_left_closest_face;
    bool roi_box_right_checked = false;
    bool roi_box_right_violated = false;
    double roi_box_right_min_margin_m = 0.0;
    std::string roi_box_right_closest_face;
    uint64_t roi_box_clamp_count = 0;
    std::string roi_box_last_set_reject_reason;

    // User-defined tilted floor plane (safety.user_floor_constraint).
    bool user_floor_constraint_enabled = false;
    bool user_floor_constraint_monitor_only = false;
    std::array<double, 3> user_floor_constraint_point_m{};      // effective (runtime) plane point
    std::array<double, 3> user_floor_constraint_normal{0.0, 0.0, 1.0};
    double user_floor_constraint_margin_m = 0.0;
    bool user_floor_constraint_left_checked = false;
    bool user_floor_constraint_left_violated = false;
    double user_floor_constraint_left_signed_dist_m = 0.0;  // lowest signed distance to plane
    std::string user_floor_constraint_left_lowest_point;
    std::array<double, 3> user_floor_constraint_left_lowest_point_m{};
    bool user_floor_constraint_right_checked = false;
    bool user_floor_constraint_right_violated = false;
    double user_floor_constraint_right_signed_dist_m = 0.0;
    std::string user_floor_constraint_right_lowest_point;
    std::array<double, 3> user_floor_constraint_right_lowest_point_m{};
    uint64_t user_floor_constraint_clamp_count = 0;
    std::string user_floor_constraint_last_set_reject_reason;

    std::optional<LatchedFaultContextSnapshot> latched_fault_context;
    std::optional<LatchedFaultContextSnapshot> left_latched_fault_context;
    std::optional<LatchedFaultContextSnapshot> right_latched_fault_context;

    bool left_send_ok = false;
    bool right_send_ok = false;
    BackendCallSnapshot left_last_read;
    BackendCallSnapshot right_last_read;
    BackendCallSnapshot left_last_send;
    BackendCallSnapshot right_last_send;
    std::string left_send_error_kind;
    std::string left_send_error_name;
    std::string left_send_error_code;
    std::string left_send_error_message;
    std::string right_send_error_kind;
    std::string right_send_error_name;
    std::string right_send_error_code;
    std::string right_send_error_message;
    CartesianSolveTelemetry left_cartesian_solve;
    CartesianSolveTelemetry right_cartesian_solve;
    SafetyTrackingTelemetry left_safety_tracking;
    SafetyTrackingTelemetry right_safety_tracking;
    bool send_suppressed = false;
    std::string send_policy = "send_servo_j";
    uint64_t left_send_start_ns = 0;
    uint64_t left_send_end_ns = 0;
    uint64_t right_send_start_ns = 0;
    uint64_t right_send_end_ns = 0;
    double send_skew_us = 0.0;
    double left_send_duration_us = 0.0;
    double right_send_duration_us = 0.0;
    ArmWorkerTelemetry left_worker_telemetry;
    ArmWorkerTelemetry right_worker_telemetry;
    RbpodoAsyncStreamingTelemetry left_async_streaming;
    RbpodoAsyncStreamingTelemetry right_async_streaming;
    std::optional<BackendTransportTelemetry> left_transport_telemetry;
    std::optional<BackendTransportTelemetry> right_transport_telemetry;
    StartupValidationSnapshot startup_validation;

    uint64_t logger_dropped_samples = 0;
};

std::string toString(ArmId arm_id);
std::string toString(ControlMode mode);
std::string toString(JointTargetProfile profile);
std::string toString(ServerMotionState state);
std::string toString(ForceControlMode mode);
std::string toString(BackendAckPolicy policy);
std::string toString(SafetyVerdict verdict);
std::string toString(FaultDomain domain);
std::string toString(TrackingErrorPolicy policy);
ControlMode controlModeFromString(const std::string& mode);
JointTargetProfile jointTargetProfileFromString(const std::string& value);
ForceControlMode forceControlModeFromString(const std::string& mode);
TrackingErrorPolicy trackingErrorPolicyFromString(const std::string& value);

}  // namespace rb_servo
