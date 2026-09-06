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
    // Leaseless, non-motion: average the F/T sensor's zero for the selected arm(s).
    // A tare is what makes any absolute force claim meaningful, and force control
    // REFUSES to cover an arm whose bias has never been established.
    TareForceSensor,
    // Leaseless runtime enable/disable of the stand-frame floor plane
    // (safety.floor_constraint). Only effective when the floor is opted in at config
    // (floor_constraint.enable=true); it toggles whether that floor is enforced.
    SetSafetyFloorEnabled,
    // Leaseless runtime adjustment of the stand-frame ROI box bounds
    // (safety.roi_box), bounded server-side to the configured runtime envelope.
    SetSafetyRoiBounds,
    // Leaseless runtime update of detected external keep-out box poses for the
    // async mesh CollisionMonitor. Adds obstacles only; it grants no motion authority.
    SetExternalBoxes,
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
    // The reach shell (safety.reach_constraint) — its OWN verdict since 2026-09-04.
    // It reported as RoiViolation until then, which cost an evening: the operator saw
    // "RoiViolation", checked the ROI box in the GUI, found the TCP well inside it,
    // and had no way left to see that a sphere centered on the shoulder was the thing
    // refusing to let the arm descend. The two constraints are different shapes in
    // different frames; they get different names.
    ReachViolation,
    ChunkFollowerFault,
    ExternalForceLimit,
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


// A 6-DOF wrench. Force [N], torque [Nm]. The FRAME AND THE REFERENCE POINT ARE
// NOT IN THIS TYPE — every field that holds one names both (see FtTelemetry).
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



struct RobotState {
    ArmId arm_id = ArmId::Left;

    uint64_t host_time_ns = 0;
    uint64_t robot_time_ns = 0;
    // Backend-owned sequence that advances only when a fresh state frame is
    // acquired. Cached/held states retain the sequence of their source frame.
    uint64_t acquisition_sequence = 0;

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
    // The controller's external F/T sensor reading, decoded from the rbpodo state
    // frame (sdata.eft_fx..eft_mz). N / Nm, IN THE SENSOR'S OWN AXES — orthogonal
    // but NOT right-handed on this unit, so this is NOT a TCP wrench and cannot be
    // turned into one by a rotation. FtPipeline applies the MEASURED basis
    // (force_torque.<arm>.axes) to map it. Zeros when no sensor is selected on the
    // controller or the state frame ended before these fields; eft_valid says which.
    Wrench6D eft_wrench;
    bool eft_valid = false;
    // The control box's OWN TCP kinematics, carried through uninterpreted
    // (rbpodo sdata.tcp_pos / tcp_ref: [x, y, z] mm + [rx, ry, rz] deg in the box's
    // base frame, with whatever tool the box has set). LOGGING ONLY (2026-09-04):
    // the box runs its calibrated DH (J1.d 165.5 mm on both RB5 boxes vs the
    // URDF's 169.2), and comparing this against the server's URDF FK of the same
    // joints is how it gets decided whether that 3.7 mm must be adopted.
    std::array<double, 6> box_tcp_pos{};
    std::array<double, 6> box_tcp_ref{};
    bool box_tcp_valid = false;
    // Our FK (tip, base frame) vs the box's own TCP report, once the box tool offset is
    // configured (kinematics.calibration.box_tool_offset_mm): the DH oracle. NaN = not
    // computable this tick.
    double fk_vs_box_tcp_mm = std::numeric_limits<double>::quiet_NaN();
    // The box's link-parameter (calibrated DH) answer, read once at connect and
    // repeated on every state frame (small, fixed size; no allocation). count 0 =
    // not read / not answered. Slot meaning is vendor firmware per kind; the
    // RB5-850E map is documented where it is logged (rbpodo_backend.cpp).
    std::array<double, 16> box_link_parameter{};
    int box_link_parameter_count = 0;

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

struct SafetyClampTelemetry {
    bool present = false;
    JointArray q_before_safety_deg{};
    JointArray q_after_joint_limit_deg{};
    JointArray q_after_velocity_limit_deg{};
    JointArray q_after_accel_limit_deg{};
    bool joint_limit_clamped = false;
    bool velocity_clamped = false;
    bool accel_clamped = false;
    double joint_limit_clamp_max_delta_deg = 0.0;
    double velocity_clamp_max_delta_deg = 0.0;
    double accel_clamp_max_delta_deg = 0.0;
    int joint_limit_limited_joint = -1;
    int velocity_limited_joint = -1;
    int accel_limited_joint = -1;
};

// Stable telemetry labels. The control-layer enum orders are checked at their
// producer; core does not depend on optimizer/worker headers.
inline constexpr std::array<const char*, 8> kPreviewWorkerStatusNames{
    "solved", "invalid_request", "source_mismatch", "preview_unavailable",
    "splice_unavailable", "solve_rejected", "late", "worker_exception"};
inline constexpr std::array<const char*, 8> kPreviewSolveStatusNames{
    "solved", "invalid_reference", "invalid_initial_state", "infeasible",
    "iteration_limit", "time_budget_exceeded", "numerical_failure", "tracking_budget_exceeded"};
inline constexpr std::array<const char*, 8> kPreviewResultCheckNames{
    "ready", "worker_rejected", "epoch_mismatch", "gate_mismatch",
    "source_mismatch", "parent_mismatch", "late", "invalid_timing"};
inline constexpr std::array<const char*, 8> kPreviewStagedCancelNames{
    "fold", "reset", "source", "parent", "contact", "expiry", "sample", "other"};
inline constexpr std::array<const char*, 5> kPreviewBrakeCauseNames{
    "expired", "contact", "backlog", "history", "other"};

enum class PreviewFoldCause : std::uint8_t { Unknown, Force, RoiFloor, GeometryHold };
inline constexpr const char* previewFoldCauseName(PreviewFoldCause cause) {
    switch (cause) {
        case PreviewFoldCause::Force: return "force";
        case PreviewFoldCause::RoiFloor: return "roi_floor";
        case PreviewFoldCause::GeometryHold: return "geometry_hold";
        default: return "unknown";
    }
}

// Current coordinator preview execution, independent of the raw follower and
// backend ACK. Status strings below are static literals, not RT allocations.
struct PreviewExecutionTelemetry {
    bool enabled = false;
    bool active = false;
    const char* status = "disabled";
    uint64_t sample_time_ns = 0;
    uint64_t epoch = 0;
    uint64_t plan_id = 0;
    uint64_t source_wire_seq = 0;
    uint64_t source_recv_seq = 0;
    double backlog_sec = 0.0;
    double rate = 1.0;
    double plan_age_sec = 0.0;
    double accepted_position_error_m = 0.0;
    double accepted_rotation_error_rad = 0.0;
    double solve_time_sec = 0.0;
    uint64_t submitted = 0;
    uint64_t accepted = 0;
    uint64_t rejected = 0;
    uint64_t expired = 0;
    uint64_t contact_guard_count = 0;

    // Latest observed result and cumulative counts survive lifecycle resets.
    // All status/reason pointers must be static literals. Result timing is in
    // monotonic seconds, IDs/counters remain uint64_t in JSON and CSV.
    uint64_t gate_revision = 0;
    uint64_t gauge_revision = 0;
    uint64_t parent_plan_id = 0;
    uint64_t request_id = 0;
    bool result_valid = false;
    bool result_solve_attempted = false;
    const char* last_worker_status = "not_observed";
    const char* last_solve_status = "not_observed";
    const char* last_admission_reason = "not_observed";
    // Worker-owned counts include completed results even if mailbox delivery
    // drops/coalesces them. Solve counts include only attempted QPs; admission
    // checks below count only results actually observed by the coordinator.
    std::array<uint64_t, 8> worker_status_counts{};
    std::array<uint64_t, 8> solve_status_counts{};
    std::array<uint64_t, 8> result_checks{};
    uint64_t result_request_id = 0;
    uint64_t result_epoch = 0;
    uint64_t result_gate_revision = 0;
    uint64_t result_gauge_revision = 0;
    uint64_t result_source_wire_seq = 0;
    uint64_t result_source_recv_seq = 0;
    uint64_t result_parent_plan_id = 0;
    uint64_t result_gauge_transported = 0;
    uint64_t staged_gauge_transported = 0;
    uint64_t gauge_transport_failed = 0;
    double result_generated_at_sec = 0.0;
    double result_splice_at_sec = 0.0;
    double result_valid_until_sec = 0.0;
    double result_completed_at_sec = 0.0;
    double result_observed_at_sec = 0.0;
    int solve_iterations = 0;
    bool solve_contact_constrained = false;
    bool solve_contact_decomposed = false;
    bool solve_contact_coupled_fallback = false;
    double solve_max_constraint_violation = 0.0;
    double solve_max_contact_velocity_violation_m_s = 0.0;
    bool solve_angular_norm_coupled = false;
    uint64_t solve_angular_norm_cuts = 0;
    double solve_max_angular_chart_velocity_norm = 0.0;
    double solve_max_angular_chart_acceleration_norm = 0.0;
    // Optimizer seed diagnostics are meaningful only if result_solve_attempted.
    // An earlier worker refusal may not have established a valid splice seed.
    double result_initial_linear_velocity_max_m_s = 0.0;
    double result_initial_linear_acceleration_max_m_s2 = 0.0;
    double result_initial_angular_velocity_norm_rad_s = 0.0;
    double result_initial_angular_acceleration_norm_rad_s2 = 0.0;

    uint64_t ready_not_staged = 0;
    uint64_t staged_identity_rejected = 0;
    uint64_t staged_expired = 0;
    uint64_t staged_sample_rejected = 0;
    uint64_t staged_contact_rejected = 0;
    std::array<uint64_t, 8> staged_cancel_counts{};
    const char* last_staged_cancel_reason = "none";
    double last_staged_cancel_time_sec = 0.0;
    uint64_t last_staged_cancel_request_id = 0;
    // Admission means the future plan became the active nominal trajectory;
    // it is distinct from backend enqueue/ACK and from staging a result.
    double last_admission_time_sec = 0.0;
    double last_admission_gap_sec = 0.0;
    uint64_t last_admitted_request_id = 0;
    uint64_t last_admitted_parent_plan_id = 0;

    std::array<uint64_t, 5> brake_counts{};
    const char* last_brake_reason = "none";
    double last_brake_start_time_sec = 0.0;
    double last_brake_origin_sec = 0.0;
    uint64_t angular_continuations_started = 0;
    uint64_t angular_brakes_started = 0;
    double last_contact_reject_time_sec = 0.0;
    double last_contact_reject_gate = 1.0;
    double last_contact_reject_closing_m_s = 0.0;
    double last_contact_reject_allowed_m_s = 0.0;
    std::array<double, 3> last_contact_reject_normal{};

    // One exact latest applied common-frame shift. The booked time/transform
    // distinguishes the previous safety decision from next-tick application.
    PreviewFoldCause fold_cause = PreviewFoldCause::Unknown;
    uint64_t fold_count = 0;
    uint64_t fold_force_count = 0;
    uint64_t fold_roi_floor_count = 0;
    uint64_t fold_geometry_hold_count = 0;
    uint64_t fold_unknown_count = 0;
    uint64_t fold_booked_time_ns = 0;
    uint64_t fold_applied_time_ns = 0;
    uint64_t fold_revision = 0;
    uint32_t fold_geometry_cause_mask = 0;
    std::array<double, 3> fold_translation_m{};
    std::array<double, 4> fold_quaternion_xyzw{0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> fold_booked_translation_m{};
    std::array<double, 4> fold_booked_quaternion_xyzw{0.0, 0.0, 0.0, 1.0};
    // Current tick's booked geometry correction, even if it is never applied
    // after a Hold/fault/reset. Separate from the latest applied fold above.
    bool pending_geometry_fold_valid = false;
    uint64_t pending_geometry_fold_time_ns = 0;
    uint32_t pending_geometry_fold_cause_mask = 0;
    std::array<double, 3> pending_geometry_fold_translation_m{};
    std::array<double, 4> pending_geometry_fold_quaternion_xyzw{0.0, 0.0, 0.0, 1.0};
    // Cumulative transform within epoch; differ consecutive samples to retain
    // all same-tick shifts even when only the latest fold event is displayed.
    std::array<double, 3> gauge_translation_m{};
    std::array<double, 4> gauge_quaternion_xyzw{0.0, 0.0, 0.0, 1.0};

    // Mailbox totals have their own producer; they must not be inferred from
    // submitted - accepted - rejected, which conflates several lifecycles.
    uint64_t request_invalid = 0;
    uint64_t request_mailbox_full = 0;
    uint64_t request_coalesced = 0;
    uint64_t result_publish_dropped = 0;
    uint64_t result_coalesced = 0;
};

struct CartesianSolveTelemetry {
    PreviewExecutionTelemetry preview_execution;
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
    bool ik_branch_jump_rate_limited = false;
    double ik_branch_jump_raw_deg = 0.0;
    double ik_branch_jump_limit_deg = 0.0;
    double ik_branch_jump_scale = 1.0;
    int ik_branch_jump_retry_count = 0;
    // Joint-limit diagnostics: when reason == "joint_limit", which joint (0-based) pinned
    // its position limit and that joint's signed margin to the limit in degrees (<= ~0 ==
    // saturated). worst_index is -1 when the solve was not joint-limited.
    int ik_joint_limit_worst_index = -1;
    double ik_joint_limit_worst_margin_deg = 0.0;
    // Joint-limit relief (ik.limit_relief_* / limit_avoidance_* / pinned_unconverged_*).
    // pinned = the returned solution RESTS on a limit; relief_weight < 1 = orientation
    // was traded for position; avoidance_step_deg = null-space push away from the bound;
    // lowpass_active = the loop damped this tick's solution because it was pinned AND
    // out of iterations. Together these say "the arm is at a bound, here is what it gave
    // up to keep moving" — the thing the log could not previously answer.
    bool ik_joint_limit_pinned = false;
    double ik_limit_relief_weight = 1.0;
    double ik_limit_avoidance_step_deg = 0.0;
    bool ik_pinned_lowpass_active = false;
    JointArray q_ik_seed_deg{};
    JointArray q_ik_raw_solution_deg{};
    JointArray q_ik_solution_deg{};
    JointArray q_ik_raw_delta_deg{};
    JointArray q_ik_delta_deg{};
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
    std::string cartesian_servo_state_source = "actual";
    std::string cartesian_divergence_source = "actual";
    bool q_reference_for_servo_valid = false;
    // --- A/B/C separation telemetry (Patch 4). Populated by the pose-track SMD path
    // and the final output moving-average stage; absent/false otherwise. Pure
    // telemetry — does not affect control. ---
    bool smd_active = false;
    std::string tcp_target_profile;
    bool tcp_target_profile_found = false;
    double smd_profile_nf_linear_hz = 0.0;
    double smd_profile_nf_angular_hz = 0.0;
    bool smd_profile_velocity_feedforward = false;
    double smd_profile_max_linear_velocity_m_s = 0.0;
    double smd_profile_max_linear_accel_m_s2 = 0.0;
    double smd_profile_max_angular_velocity_rad_s = 0.0;
    double smd_profile_max_angular_accel_rad_s2 = 0.0;
    double smd_profile_max_goal_lead_m = 0.0;
    double smd_profile_max_goal_lead_rad = 0.0;
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
    // THE RELEASE BRAKE / WALL BRAKE on the pose-track path (2026-09-04 pm).
    bool smd_release_braking = false;
    bool smd_wall_engaged = false;
    std::string smd_wall_name;                 // closest wall in band this tick
    double smd_wall_margin_m = std::numeric_limits<double>::quiet_NaN();
    double smd_wall_cap_m_s = -1.0;            // approach cap that acted; < 0 = none
    double smd_wall_clamp_m = 0.0;             // position correction this tick [m]
    // Chunk-follower stage (replaces the SMD step while active). One row per
    // tick: the active segment target / delta-follower state, window ids, and
    // flags — offline analysis joins these against command_tcp_target_stand
    // (raw producer command), tcp_command_stand (emitted setpoint), and
    // tcp_actual_stand (measured) on the same row.
    std::string follower_controller = "none";
    bool follower_active = false;
    uint64_t follower_wire_seq = 0;     // producer packet seq / flow chunk id
    uint64_t follower_recv_seq = 0;     // receiver-local accepted-frame count
    int follower_step = -1;             // absolute chunk index of the segment target
    double follower_t_in_seg_sec = 0.0; // time into the current 33ms segment
    double follower_duration_sec = 0.0; // ruckig T_opt of the segment (>= policy_dt)
    double follower_jerk_scale = 1.0; // selected fraction of configured per-axis jerk ceilings
    int follower_jerk_search_calculations = 0; // extra solves for the current segment
    std::array<double, 6> follower_axis_duration_sec{};
    std::array<double, 6> follower_target_velocity{};
    std::array<double, 6> follower_target_acceleration{};
    int follower_segments = 0;
    double follower_advance_gate = 1.0;
    double follower_plan_rate_gate = 1.0;
    double follower_core_gate = 1.0;    // policy period / stretched segment length (core time-stretch)
    double follower_leash_gate = 1.0;   // divergence leash on the plan clock
    int follower_solve_failures = 0;    // Ruckig-refused segment solves (served by ring-down)
    std::array<double, 3> follower_advance_direction{};
    bool follower_output_smd_reseeded = false;
    double follower_alpha = 1.0;        // sacrifice-ladder time dilation applied
    bool follower_converged = false;    // duration ≈ policy_dt (in-regime)
    bool follower_stall = false;        // ring-down (window exhausted, no fresh chunk)
    bool follower_corner = false;       // corner ring-down fired on some axis
    std::optional<Pose6D> follower_pf_stand;  // active segment target pose
    std::optional<Pose6D> stage_tcp_target_stand;  // pose-track stage output handed to IK
    bool follower_output_smd_active = false;
    double follower_output_smd_lag_m = 0.0;
    double follower_output_smd_lag_rad = 0.0;
    std::optional<Pose6D> follower_prefilter_stand;  // raw per-tick follower emission
    // Physical wall-time derivatives: stand linear / reference-body angular.
    std::optional<Vec6> follower_sample_velocity;
    std::optional<Vec6> follower_sample_acceleration;
    double follower_divergence_pos_m = 0.0;
    double follower_divergence_ang_rad = 0.0;
    double follower_projection_error_m = 0.0;
    double follower_projection_error_rad = 0.0;
    int follower_projection_error_count = 0;
    double follower_actual_lead_m = 0.0;
    double follower_actual_lead_rad = 0.0;
    int follower_actual_lead_error_count = 0;
    uint64_t follower_reanchor_count = 0;          // TOTAL re-anchors (all causes below)
    // Split by CAUSE, because the total cannot answer the question that matters:
    // was the plan re-anchored because the safety layer deliberately slowed the arm
    // (INTENDED -- the constraint is doing its job and the lead is its bounded, expected
    // consequence), or because the arm simply failed to track (UNINTENDED -- a genuine
    // fidelity loss that re-anchoring HIDES by skipping trajectory content)?
    // Only the unexplained kind consumes the lead re-anchor rate budget.
    uint64_t follower_divergence_reanchor_count = 0;        // sent target drifted from the plan
    uint64_t follower_divergence_excused_count = 0;         // soft divergence while the robot tracked its command
    // Dead-time tolerant robot-vs-command tracking (best match among the commands sent
    // in the last 50 ms): the question the divergence/lead gates are excused on.
    double follower_cmd_track_pos_m = 0.0;
    double follower_cmd_track_rad = 0.0;
    int follower_cmd_track_lag_ticks = -1;
    bool follower_cmd_tracks = false;
    uint64_t follower_lead_reanchor_explained_count = 0;    // lead while throttled/blocked/projected
    uint64_t follower_lead_reanchor_unexplained_count = 0;  // lead with no safety cause -> real
    uint64_t follower_warm_resume_count = 0;       // brief Hold resumes preserving chained p/v/a
    bool safety_intervention_recent = false;       // debounced signal seen by follower stage
    bool cartesian_solve_blocked_recent = false;   // debounced IK/Cartesian solve refusal seen by follower stage
    bool command_throttled_recent = false;         // debounced safety-clamp / branch-jump-rate-limit throttle
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
    // Final-stage output moving average (C). q_target before/after the boxcar.
    bool output_ma_present = false;
    int output_ma_window = 0;
    JointArray q_target_before_output_ma_deg{};
    JointArray q_target_after_output_ma_deg{};
    SafetyClampTelemetry safety_clamp;
};

// Shared producer/receiver diagnostics for the active whole-chunk frame. This
// lives only in ServoSample/CSV; state JSON remains intentionally unchanged.
struct ChunkFrameTelemetry {
    std::uint64_t wire_seq = 0;
    std::uint64_t recv_seq = 0;
    double policy_dt_sec = 0.0;
    int horizon = 0;
    int execute_steps = 0;
    int runway_steps = 0;
    double age_ms = 0.0;
    double interarrival_ms = 0.0;
    std::uint64_t inference_seq = 0;
    double inference_queue_wait_ms = 0.0;
    double inference_latency_ms = 0.0;
    double inference_ready_wait_ms = 0.0;
    double inference_period_ms = 0.0;
    double inference_period_jitter_ms = 0.0;
    std::uint64_t inference_stall_count = 0;
    std::uint64_t camera_bundle_seq = 0;
    double camera_bundle_age_ms = 0.0;
    double camera_max_skew_ms = 0.0;
    std::uint64_t camera_left_frame_number = 0;
    std::uint64_t camera_right_frame_number = 0;
    double camera_left_frame_age_ms = 0.0;
    double camera_right_frame_age_ms = 0.0;
    double camera_left_focus_score = 0.0;
    double camera_right_focus_score = 0.0;
};

// ---- F/T pipeline telemetry -------------------------------------------------
// One arm's wrench at every stage the pipeline produces, so a wrong number can be
// traced to the stage that made it wrong instead of being argued about. EVERY
// wrench field below names its FRAME and its REFERENCE POINT, because a wrench
// without both is not a measurement (controller-manager wiki/decisions/0027).
struct FtTelemetry {
    bool enabled = false;              // force_torque.<arm>.enable
    // The COLD sensor-presence verdict. A live RFT always jitters above its noise
    // floor, so a stream that never varies is an unplugged sensor. NOT fatal: the
    // arm runs without force sensing and every compensated channel below is pinned
    // to EXACT ZERO, so bias and tool-gravity subtraction cannot fabricate a force
    // nobody measured.
    bool connected = false;
    std::string connect_reason;        // why `connected` reads the way it does
    double liveness_force_pp_n = 0.0;  // peak-to-peak seen during the check [N]
    double liveness_torque_pp_nm = 0.0;
    // (1) RAW, axis-mapped only. SENSOR frame (flange-aligned after the basis map),
    //     torque about the SENSING REFERENCE ORIGIN. No bias, no gravity, no deadzone.
    Wrench6D raw_sensor;
    // The tool-gravity term this tick, SENSOR frame @SRO. Published because the TARE
    // MUST NOT DOUBLE-SUBTRACT IT: the box is told a zero payload, so `raw_sensor`
    // still CONTAINS gravity and a tare that averaged raw would fold the tare pose's
    // gravity into the bias. The tare averages `raw_sensor - gravity` instead.
    Wrench6D gravity_sensor;
    // (2) COMPENSATED (-bias -gravity), SENSOR frame @SRO, BEFORE the deadzone.
    Wrench6D comp_sensor_nodz;
    // (2') the same after the deadzone.
    Wrench6D comp_sensor;
    // (3) COMPENSATED, TCP reference point, TOOL axes, after the deadzone. THIS IS
    //     THE SURFACE THE FORCE LAW CONSUMES. The order is load-bearing:
    //     gravity/bias -> reference-point shift -> rotate into tool axes -> deadzone.
    Wrench6D comp_tcp;
    // (4) the same wrench rotated into STAND axes (torque still about the TCP). The
    //     frame the overlay integrates in and the frame an operator reads.
    Wrench6D comp_stand;
    // The bias in force at this instant, SENSOR frame — what the last tare stored.
    Wrench6D bias;
    bool bias_valid = false;           // a tare has run since the last invalidation
    std::string bias_source;           // "config" | "tare" | "none"
    std::uint64_t bias_generation = 0; // increments on every accepted tare
    std::string tare_state;            // "none" | "settling" | "accepted" | "rejected"
    std::string tare_reason;
    int tare_samples = 0;
    // Automatic-tare-on-InitMotion stage (force_torque.auto_tare_after_init_motion).
    //   "off"           - not configured for this arm
    //   "idle"          - configured, nothing pending
    //   "awaiting_init" - InitMotion requested; the zero is dropped and the tare waits
    //                     for the arm to reach the init pose
    //   "settling"      - init pose reached; waiting out settle_sec at rest
    // The collection itself then shows up in `tare_state` exactly like a manual tare.
    std::string auto_tare_stage = "off";
    std::string auto_tare_reason;      // why it last armed, fired or was dropped
    // The heavily low-passed STAND-frame force magnitude, as a mass. Exists to escape
    // the deadzone: the shipped 2 N/axis flattens ~204 g to exactly zero on every
    // compensated channel, and this is the range an operator wants to read.
    double load_force_n = 0.0;
    double load_mass_kg = 0.0;
    bool load_settled = false;
    // The effective configuration this arm ran with, echoed so a log can be read
    // without the yaml beside it.
    double tool_mass_kg = 0.0;
    std::array<double, 3> tool_com_mm{};
    std::array<double, 3> sensor_offset_mm{};
    std::array<double, 3> tcp_from_sro_mm{};
    double axes_determinant = 0.0;     // -1 on this unit: LEFT-HANDED, and correct
};

// ---- force-control (admittance overlay) telemetry ---------------------------
struct ForceControlTelemetry {
    bool enabled = false;              // force_control.enable
    bool covered = false;              // the overlay ran this tick
    std::string coverage_reason;       // why it did or did not
    // WHICH LAW RAN. "stream" (a plan driven into contact, soft, gate-bounded) or
    // "hold" (an operator pushing by hand, stiff, spring-bounded). They differ by 5x
    // in the rotation/translation stiffness RATIO, so an unlabelled deviation cannot
    // be judged against either.
    std::string law;
    bool compose_applied = false;      // the deviation reached the commanded target
    // Tick-entry overlay state, including a frozen deviation while uncovered.
    // Unlike deviation_m/rad below, these are populated even when the law cannot run.
    std::array<double, 3> reference_deviation_m{};
    std::array<double, 3> reference_deviation_rad{};
    bool reference_strip_enabled = false;  // valid Cartesian targets will be composed this tick
    std::uint64_t reference_reset_count = 0; // fresh InitMotion requests for this arm
    // THE DEVIATION from the nominal (followed) pose, STAND frame. Translation [m],
    // rotation as a rotation vector [rad].
    std::array<double, 3> deviation_m{};
    std::array<double, 3> deviation_rad{};
    double deviation_norm_m = 0.0;
    double deviation_norm_rad = 0.0;
    std::array<double, 3> velocity_m_s{};
    std::array<double, 3> velocity_rad_s{};
    // THE FENCE. `bounded` is latched while pinned: a silent saturation is a lie
    // about where the arm is being asked to go.
    bool bounded = false;
    // THE OSCILLATION GUARD (force_control.oscillation_*): compliance is frozen
    // because the deviation velocity reversed direction repeatedly at amplitude —
    // a limit cycle, not a push. trips is cumulative (read as steps).
    bool oscillation_frozen = false;
    uint64_t oscillation_trips = 0;
    // Coverage recovery progress (force_control.coverage_recover_sec): consecutive
    // healthy raw verdicts / ticks required. 0/0 when not recovering. This is the
    // number that used to be spliced into the coverage reason string, which made
    // the edge-logger print a WARN every tick.
    int coverage_recover_streak = 0;
    int coverage_recover_needed = 0;
    // The wrench the LAW actually consumed, after the contact-shock low-pass
    // (force_control.wrench_filter_hz; 0 = filter off, equals wrench_stand).
    // wrench_stand above stays the RAW measurement, so the pair shows exactly
    // what the filter removed.
    Wrench6D wrench_filtered_stand{};
    double wrench_filter_hz = 0.0;
    double fence_m = 0.0;
    double fence_rad = 0.0;
    // THE GATE: the fraction of the plan advance that survives along the direction
    // pushing INTO the measured wrench. 1 = free space, 0 = fully held.
    double gate_translation = 1.0;
    double gate_rotation = 1.0;
    double gate_force_n = 0.0;         // |F| the gate judged: PHYSICAL (pre-deadzone), filtered (2026-09-04)
    double gate_torque_nm = 0.0;
    bool gate_closed = false;          // < 0.02 on either channel
    // How much plan advance the gate actually removed this tick [m] / [rad].
    double gate_removed_m = 0.0;
    double gate_removed_rad = 0.0;
    // THE STREAM CHANNEL (absolute-target path, 2026-09-04): the slow |F| it is
    // judged on, whether the sustained-contact trigger is armed, and its fade.
    double gate_stream_translation = 1.0;
    double gate_stream_force_n = 0.0;
    bool gate_stream_armed = false;
    // CSV-only: exact gate snapshot consumed by pose-track SMD, BEFORE this
    // tick's force update. The legacy fields above describe the updated gate.
    bool smd_gate_sample_valid = false;
    bool smd_gate_armed = false;
    bool smd_gate_releasing = false;
    double smd_gate_translation = 1.0;
    std::array<double, 3> smd_gate_normal_stand{};
    std::array<double, 3> smd_gate_measured_force_stand_n{};
    double smd_gate_removed_velocity_m_s = 0.0;
    // The wrench that drove the law, STAND frame @TCP — the same numbers as
    // FtTelemetry::comp_stand, repeated here so one row explains one decision.
    Wrench6D wrench_stand;
    // IK on the deviated pose. A refusal HOLDS the last emitted pose (bounded by
    // construction) rather than letting the nominal stand, which would step by the
    // whole deviation.
    bool ik_refused = false;
    std::uint64_t ik_refused_total = 0;
    std::uint32_t ik_refused_streak = 0;
    // THE FOLD (force_control.fold_deviation). `folded` = this tick's deviation was
    // handed to the plan; `fold_sink` names where ("chunk_follower", "hold_nominal")
    // or why not ("declined: ..."); `fold_m/rad` is what moved THIS tick and
    // `absorbed_*` the running total for the run, i.e. how far force has moved the
    // plan away from where the source asked it to be.
    bool folded = false;
    std::string fold_sink;
    std::array<double, 3> fold_m{};
    std::array<double, 3> fold_rad{};
    std::array<double, 3> absorbed_m{};
    double absorbed_norm_m = 0.0;
    double absorbed_norm_rad = 0.0;
    // THE HAND-GUIDE LATCH (force_control.hold_engage_force_n): whether the hold law
    // integrated this tick, and the physical (pre-deadzone) |F| it judged.
    bool hold_engaged = true;
    double hold_force_n = 0.0;
};

struct SafetyTrackingTelemetry {
    std::string tracking_error_source = "actual";
    bool tracking_error_source_valid = true;
    std::string tracking_error_reason;
    double command_reference_tracking_error_deg = 0.0;
    double physical_command_actual_error_deg = 0.0;
    // ---- TWO ERRORS, AND THEY BLAME DIFFERENT THINGS -------------------------
    // The latch compares OUR COMMAND against the measured joints, which is
    // controller-manager's `JointDeviation`. CM keeps a SECOND, separate check —
    // its `TrackingError` — that compares the BOX's OWN reference (sdata.jnt_ref)
    // against the measured joints, and its comment says why: that one is "a
    // physical anomaly (collision / overload / servo fault), INDEPENDENT of our
    // command".
    //
    // Reporting only the first is what made the 2026-08-26 fault unreadable: it
    // said "tracking error" while the arm was tracking its own reference to
    // 0.00 deg. The arm was perfect; the BOX was not taking our commands. Both
    // numbers are published so the log names the right subsystem.
    //
    // command_vs_actual  large + reference_vs_actual small -> the LINK is broken:
    //                    the box is not consuming what we send.
    // reference_vs_actual large                            -> the ARM is in
    //                    trouble: collision, overload, servo fault.
    double command_vs_actual_deg = 0.0;      // our q_sent    vs measured
    double reference_vs_actual_deg = 0.0;    // box's q_ref   vs measured
    bool reference_valid = false;            // the box reported a usable q_ref
    // Which of the two the latch fired on, empty when it has not fired.
    std::string latch_cause;
    bool controller_simulation_physical_motion_detected = false;
};




struct ArmCommand {
    ArmId arm_id = ArmId::Left;

    uint64_t seq = 0;
    uint64_t host_time_ns = 0;

    ControlMode mode = ControlMode::Hold;

    JointArray q_target_deg{};
    JointTargetProfile joint_target_profile = JointTargetProfile::Direct;
    // Stable across retransmitted packets; zero retains legacy seq/goal semantics.
    uint64_t init_motion_request_id = 0;

    // Final-stop joint pose for the JointTarget arrival-decel taper. InitMotion pursuit
    // sets this to the terminal waypoint while q_target_deg leads by the pursuit
    // lookahead, so JointSmdTracker can ease into the true endpoint independently of the
    // cruise natural frequency. Absent (has_arrival_stop=false) => no taper (unchanged).
    JointArray arrival_stop_q_deg{};
    bool has_arrival_stop = false;

    Pose6D tcp_target_stand;
    double linear_move_duration_sec = 0.0;
    double linear_move_linear_speed_m_s = 0.0;
    double linear_move_angular_speed_rad_s = 0.0;
    LinearMoveOrientationMode linear_move_orientation_mode = LinearMoveOrientationMode::Constant;


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
    // Server-authored: a compliant Hold promoted to a TcpPoseTarget at its latched
    // nominal (force_control.hold_compliance). The follower stages treat it as a
    // Hold for the chunk plan's lifecycle (pause / bounded resume, never engage)
    // and only the pose-track SMD tracks the nominal (2026-09-06).
    bool compliant_hold = false;
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

struct ExternalBoxCommand {
    std::string label;                    // "green" | "gray" slot key
    std::array<double, 16> T_stand_box{};  // 4x4 row-major, box pose in stand frame
    bool enable = true;
};

struct DualArmCommand {
    uint64_t seq = 0;
    uint64_t host_time_ns = 0;

    CommandSourceMetadata source;
    CommandSourceLeaseState lease;

    ArmCommand left;
    ArmCommand right;

    std::string tcp_target_profile = "default";
    bool tcp_target_profile_provided = false;
    uint64_t client_send_monotonic_ns = 0;
    bool has_client_send_monotonic_ns = false;
    uint64_t input_sample_monotonic_ns = 0;
    bool has_input_sample_monotonic_ns = false;
    std::string action_source;
    std::string source_conditioning_mode;

    // TareForceSensor payload: which arm(s) to tare. Both false = both arms, which
    // is what an operator pressing one button means.
    bool tare_left = false;
    bool tare_right = false;
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

    // SetExternalBoxes payload (top-level: external keep-out boxes are global):
    // labeled stand-frame box poses supplied by the camera/perception worker.
    std::vector<ExternalBoxCommand> external_boxes;
    bool has_external_boxes = false;

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

struct CommandBufferReadTelemetry {
    std::string result = "unset";
    uint64_t pending_lifecycle_count = 0;
    uint64_t skipped_lifecycle_count = 0;
    bool external_boxes_pending = false;
    bool external_boxes_consumed = false;
    bool external_boxes_applied = false;
    uint64_t external_boxes_seq = 0;
    ControlMode external_boxes_left_mode = ControlMode::Hold;
    ControlMode external_boxes_right_mode = ControlMode::Hold;
    uint64_t external_boxes_host_time_ns = 0;
    double external_boxes_age_ms = std::numeric_limits<double>::quiet_NaN();
    double external_boxes_client_send_age_ms = std::numeric_limits<double>::quiet_NaN();

    uint64_t returned_seq = 0;
    ControlMode returned_left_mode = ControlMode::Hold;
    ControlMode returned_right_mode = ControlMode::Hold;
    uint64_t returned_host_time_ns = 0;
    double returned_age_ms = std::numeric_limits<double>::quiet_NaN();
    double returned_client_send_age_ms = std::numeric_limits<double>::quiet_NaN();

    uint64_t latest_seq = 0;
    ControlMode latest_left_mode = ControlMode::Hold;
    ControlMode latest_right_mode = ControlMode::Hold;
    uint64_t latest_host_time_ns = 0;
    double latest_age_ms = std::numeric_limits<double>::quiet_NaN();
    double latest_timeout_ms = std::numeric_limits<double>::quiet_NaN();
    bool latest_timeout_valid = false;
    bool latest_usable = false;
    double latest_client_send_age_ms = std::numeric_limits<double>::quiet_NaN();

    uint64_t lifecycle_seq = 0;
    ControlMode lifecycle_left_mode = ControlMode::Hold;
    ControlMode lifecycle_right_mode = ControlMode::Hold;
    uint64_t lifecycle_host_time_ns = 0;
    double lifecycle_age_ms = std::numeric_limits<double>::quiet_NaN();
    double lifecycle_timeout_ms = std::numeric_limits<double>::quiet_NaN();
    bool lifecycle_timeout_valid = false;
    bool lifecycle_usable = false;
};

struct ServoTarget {
    JointArray left_q_target_deg{};
    JointArray right_q_target_deg{};
};

// Timing around the vendor SDK's blocking CobotData::request_data() call.
// System-clock stamps can be correlated with passive packet-capture timestamps;
// steady-clock stamps remain suitable for in-process duration measurements.
// This is deliberately distinct from BackendTiming, which covers the entire
// readState() operation, including state mapping and fault classification.
struct BackendReadExchangeTiming {
    bool available = false;
    uint64_t exchange_sequence = 0;
    std::string source = "none";
    uint64_t request_data_call_start_steady_ns = 0;
    uint64_t request_data_call_start_system_ns = 0;
    uint64_t request_data_call_return_steady_ns = 0;
    uint64_t request_data_call_return_system_ns = 0;
    double request_data_call_duration_us = 0.0;
};

struct BackendCallSnapshot {
    bool ok = true;
    bool accepted = true;
    std::string backend_error_kind = "None";
    std::string error_name = "None";
    std::string error_code;
    std::string error_message;
    double duration_us = 0.0;
    BackendReadExchangeTiming read_exchange_timing;
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
    // Wire-side accounting. The loop enqueues at its own 500 Hz clock while the
    // worker dispatches at the box-locked cadence (~499.35 Hz under queue sync),
    // so `pending_overwrites` above counts setpoints that never reached the wire
    // and `repeated_sends` counts wire holds on an empty-mailbox cadence tick.
    // `last_wire_send_*_ns` bracket the worker's actual backend->sendServoJ()
    // call — distinct from the loop-side enqueue stamp the CSV logs as
    // left/right_send_start_ns.
    uint64_t worker_repeated_sends_total = 0;
    uint64_t worker_wire_dispatches_total = 0;
    uint64_t worker_last_wire_send_start_ns = 0;
    uint64_t worker_last_wire_send_end_ns = 0;
    // Setpoint rate conversion (servo.worker_setpoint_interpolation).
    // delay_setpoints is the interpolation latency in setpoints (~1 = +2 ms);
    // rebase/hold count the clamp events (see setpoint_interpolator.hpp).
    bool worker_interp_active = false;
    double worker_interp_delay_setpoints = 0.0;
    uint64_t worker_interp_rebase_total = 0;
    uint64_t worker_interp_hold_total = 0;
    std::string worker_queue_policy = "latest_wins";
};

// Per-tick observability for the combined geometric velocity projection
// (ROI/floor/reach/self-collision rows + the global per-joint ceiling inside
// solveVelocityProjection). Previously the only output was a 5 Hz stderr line
// behind RB_SELF_COLLISION_LOG; boundary chatter and release lunges could not
// be attributed from the CSV.
struct SafetyProjectionTelemetry {
    // CSV-only stage snapshots, raw joint degrees. Distinguish the geometric
    // solve from its subsequent release slew without changing either stage.
    bool joint_stage_trace_valid = false;
    std::array<JointArray, 2> requested_q_deg{};
    std::array<JointArray, 2> projected_q_deg{};
    std::array<JointArray, 2> released_q_deg{};
    bool active = false;                   // any constraint row engaged this tick
    int constraint_count = 0;              // rows handed to the Gauss-Seidel solve
    double left_correction_deg_s = 0.0;    // max joint-speed removed, per arm
    double right_correction_deg_s = 0.0;
    // What actually reached the wire after the release slew shaped it. The two
    // above are the SOLVER's answer and are unchanged by the slew, so they cannot
    // show whether the release was ramped -- these can. Equal to the solver value
    // whenever the correction is growing (the slew only bounds shrinking).
    double left_applied_correction_deg_s = 0.0;
    double right_applied_correction_deg_s = 0.0;
    bool ceiling_clamped = false;          // global per-joint velocity ceiling bound
    double min_margin_m = -1.0;            // min d_now across engaged rows; -1 = none
    // WHICH pair is actually near ITS OWN floor, and by how much. min_margin_m is a
    // raw clearance minimised across rows whose floors differ by 5x (arm<->arm 25 mm
    // vs intra-arm 5 mm), so it cannot answer that on its own: a run can read
    // "30 s below 25 mm" and be entirely normal intra-arm geometry. Headroom is
    // d_now - that row's own d_hard, so 0 means "at its floor" for every class, and
    // the pair name says which geometry to look at before tuning anything.
    double min_headroom_m = -1.0;          // min (d_now - d_hard) across collision rows
    double min_headroom_d_hard_m = -1.0;   // that row's own floor
    std::string min_headroom_pair;         // "geom_a <-> geom_b" of that row
    std::string min_headroom_class;        // self | intra_arm | external | external_box | gripper_gripper
    // Monitor liveness and class minima, EVERY tick the verdict is valid (2026-09-04;
    // the age used to be reported only on ticks with an engaged row, and the global
    // min clearance read the structural intra-arm pair all run). -1 / inf = none.
    double selfcol_verdict_age_ms = -1.0;  // age of the collision verdict; -1 = none
    double selfcol_eval_ms = -1.0;         // wall time of the monitor's last evaluation
    int selfcol_near_count = 0;            // pairs in the verdict's near list
    int selfcol_near_band_count = 0;       // of which inside their class engage band
    double selfcol_self_min_clearance_m = std::numeric_limits<double>::infinity();
    double selfcol_intra_arm_min_clearance_m = std::numeric_limits<double>::infinity();
    double selfcol_gripper_min_clearance_m = std::numeric_limits<double>::infinity();
    bool selfcol_gripper_excluded = false; // gripper<->gripper rows left to force control
    bool selfcol_stale = false;            // verdict older than max_staleness_s (hold)
    int sweeps = 0;                        // Gauss-Seidel sweeps the solve ran
    bool converged = false;                // and whether the last one changed nothing
    double tightest_dir_change_deg = 0.0;  // J direction jitter of the tightest collision row
    uint64_t self_collision_clamp_count = 0;  // ticks a collision row "blocked" an arm
    // REACH SHELL (safety.reach_constraint), 2026-09-04. Until today this layer had
    // no telemetry at all: not one of the 1255 servo-log columns, and nothing in the
    // published state either, while its verdict aliased to RoiViolation. Diagnosing
    // the 2026-09-04 blocked-descent runs meant recomputing the radius offline from
    // left_mount.base_pose_in_stand and the TCP columns to find out that a sphere was
    // holding the arm 81 mm above where the operator was pushing. These columns are
    // that reconstruction, done in the loop that already knows the answer.
    // margin = distance to the closest shell (>= 0 inside, < 0 outside); r_far =
    // radius of the most-exposed checked point (TCP + gripper-tip offsets), which is
    // the one the shell actually binds — it runs up to 58 mm past the TCP radius.
    bool left_reach_engaged = false;       // a reach row entered this tick's solve
    bool right_reach_engaged = false;
    double left_reach_margin_m = std::numeric_limits<double>::quiet_NaN();
    double right_reach_margin_m = std::numeric_limits<double>::quiet_NaN();
    double left_reach_r_far_m = std::numeric_limits<double>::quiet_NaN();
    double right_reach_r_far_m = std::numeric_limits<double>::quiet_NaN();
    std::string left_reach_shell;          // r_min | r_max (closest), empty = unchecked
    std::string right_reach_shell;
    uint64_t reach_clamp_count = 0;        // ticks a reach row "blocked" an arm
    // Safety plan gate (safety.plan_gate): the rate at which each arm's chunk
    // follower plan clock is currently allowed to advance. 1.0 = ungated.
    double left_plan_gate = 1.0;
    double right_plan_gate = 1.0;
    // THE HOLD FOLD (2026-09-05): the shortfall booked into each arm's plan this
    // tick [m] (0 = nothing booked), and whether a shortfall beyond the sanity
    // cap was declined this tick.
    double left_hold_fold_m = 0.0;
    double right_hold_fold_m = 0.0;
    bool left_hold_fold_capped = false;
    bool right_hold_fold_capped = false;
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

// Rainbow control-box command-queue occupancy, parsed from the "RBACK[<n>]"
// ACK the firmware returns on the COMMAND channel for every streamed command.
// n is the box's queue fill at the moment it received that command -- the only
// direct window into box-side scheduling depth. Firmware v8.7.3 reports a
// meaningful occupancy here.
//
// fill stays -1 until a token is actually parsed: absence is reported, never
// defaulted to 0, because 0 is itself a legal (empty-queue) reading.
struct RbpodoQueueAckTelemetry {
    bool observed = false;              // at least one RBACK parsed on this connection
    int fill = -1;                      // latest occupancy; -1 = never observed
    int fill_min = -1;
    int fill_max = -1;
    uint64_t sequence = 0;              // increments per parsed RBACK (freshness)
    uint64_t parsed_total = 0;          // RBACK tokens parsed since connect
    uint64_t drained_total = 0;         // responses drained since connect
    uint64_t malformed_total = 0;       // drained text containing "RBACK" that did not parse
    uint64_t drained_this_send = 0;     // responses drained by this send
    uint64_t parsed_this_send = 0;      // RBACK tokens parsed by this send
};

// The queue-sync CONTROLLER's own state, paired with RbpodoQueueAckTelemetry
// above: that one is the plant (what the box reported), this one is the loop
// closed around it. Without this the fill can be watched misbehaving with no way
// to tell whether the trim saturated, the integral wound up, or the law fell back
// to a non-tracking phase.
//
// This is a plain telemetry mirror of control/QueueSyncDecision. It is duplicated
// rather than reused because queue_sync_controller.hpp includes config.hpp, which
// includes THIS header -- taking the type directly would be a cycle.
struct QueueSyncTelemetry {
    bool enabled = false;               // queue_sync ran for this arm this tick
    double period_trim_us = 0.0;        // THE ACTUATOR: added to the send period
    double fill_lpf = 0.0;              // low-passed fill the law acts on
    double integral_us = 0.0;           // learns the host/box clock mismatch
    int last_fill = -1;                 // -1 = no RBACK observed yet
    bool fill_valid = false;
    std::string phase = "idle";         // idle | warmup | drain | track
    bool locked = false;                // Track, fill in tolerance, AND feedback recent
    int stale_cycles = 0;               // age of last_fill in cycles (0 = fresh this tick)
    uint64_t underrun_events = 0;       // CONFIRMED fill <= protect_fill
    uint64_t warn_events = 0;           // fill entered the warn band (once per episode)
    uint64_t dip_events = 0;            // a dip episode ended
    int dip_last_min = 0;               // last episode: minimum fill reached
    double dip_last_ms = 0.0;           // last episode: duration
    uint64_t stall_events = 0;          // no fresh RBACK for stall_cycles
    uint64_t highwater_events = 0;      // absurd backlog; box likely not consuming
    uint64_t redrain_events = 0;        // queue rebase forced a re-drain
    uint64_t no_consumption_events = 0; // fill rising faster than a trim can correct
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

struct InitMotionDiag {
    std::string status = "idle";
    std::string fail_mode = "none";
    std::string message;
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
    int tree_start = 0;
    int tree_goal = 0;
    int iterations = 0;
    double planning_time_s = 0.0;
    int waypoint_index = 0;
    int waypoint_count = 0;
    double dist_to_goal_deg = std::numeric_limits<double>::quiet_NaN();
    double clear_threshold_m = std::numeric_limits<double>::quiet_NaN();
    double external_clear_threshold_m = std::numeric_limits<double>::quiet_NaN();
    std::string nearest_pair;
    double nearest_pair_distance_m = std::numeric_limits<double>::quiet_NaN();
    bool nearest_pair_external = false;
};

struct InitMotionArmDiag {
    uint64_t request_id = 0;
    std::string status = "idle";
    std::string fail_mode = "none";
    std::string message;
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
    int waypoint_index = 0;
    int waypoint_count = 0;
    double dist_to_goal_deg = std::numeric_limits<double>::quiet_NaN();
    double clear_threshold_m = std::numeric_limits<double>::quiet_NaN();
    double external_clear_threshold_m = std::numeric_limits<double>::quiet_NaN();
    std::string nearest_pair;
    double nearest_pair_distance_m = std::numeric_limits<double>::quiet_NaN();
    bool nearest_pair_external = false;
};

struct CollisionPairPattern {
    std::string pattern_a;
    std::string pattern_b;
};

struct ServoSample {
    uint64_t tick = 0;
    uint64_t loop_start_time_ns = 0;
    uint64_t loop_end_time_ns = 0;
    // Wake/send jitter decomposition (steady clock, same epoch as loop_start_time_ns).
    // sched_wake_time_ns: the sleep_until target this tick was supposed to wake at.
    // prev_sleep_enter_time_ns: when the previous tick entered sleep_until (0 on first tick).
    uint64_t sched_wake_time_ns = 0;
    uint64_t prev_sleep_enter_time_ns = 0;

    RobotState left_state;
    RobotState right_state;
    // Persist the same per-tick F/T and force-control truth surface that is
    // published over UDP so supervised hardware tests remain auditable after
    // the live GUI/state consumers have exited.

    DualArmCommand command;
    CommandBufferReadTelemetry command_buffer_read;
    ChunkFrameTelemetry chunk_frame;

    JointArray left_sent_q_deg{};
    JointArray right_sent_q_deg{};

    RbpodoQueueAckTelemetry left_queue_ack;
    RbpodoQueueAckTelemetry right_queue_ack;
    QueueSyncTelemetry left_queue_sync;
    QueueSyncTelemetry right_queue_sync;

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
    FtTelemetry left_ft;
    FtTelemetry right_ft;
    ForceControlTelemetry left_force_control;
    ForceControlTelemetry right_force_control;
    SafetyTrackingTelemetry left_safety_tracking;
    SafetyTrackingTelemetry right_safety_tracking;
    SafetyProjectionTelemetry safety_projection;
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
    InitMotionDiag init_motion;
    InitMotionArmDiag init_motion_left;
    InitMotionArmDiag init_motion_right;
    std::string left_mode_before_init_sequencer;
    std::string right_mode_before_init_sequencer;
    std::string left_mode_after_init_sequencer;
    std::string right_mode_after_init_sequencer;
    std::string left_joint_target_profile_before_init_sequencer;
    std::string right_joint_target_profile_before_init_sequencer;
    std::string left_joint_target_profile_after_init_sequencer;
    std::string right_joint_target_profile_after_init_sequencer;
    std::string non_init_arm_preserved_mode;
    bool single_arm_freeze_other_arm = false;
    // URDF-mesh self-collision monitor (safety.self_collision): smallest clearance over
    // the active pairs this tick and the closest pair's name. Logged so a controller
    // op_stat_self_collision (code 1005) latch can be cross-checked against the server's
    // own mesh proximity in the CSV (real collision -> clearance trending to ~0 on a real
    // arm pair; firmware false-positive -> clearance stays comfortably positive).
    double self_collision_min_clearance_m = 0.0;
    std::string self_collision_pair;
    // "geom_a <-> geom_b" for min_clearance_m. self_collision_pair above is only a
    // side category ("left_right"/"left_stand"/"right_stand"/"all") and reads "all"
    // for every same-side pair, so it cannot name an arm folding onto itself.
    std::string self_collision_closest_pair;
};

struct RealtimeTimingRange {
    double last = 0.0;
    double p95 = 0.0;
    double max = 0.0;
};

struct ServoRealtimeTimingTelemetry {
    double target_rate_hz = 0.0;
    double observed_rate_hz = 0.0;
    double send_rate_hz = 0.0;
    RealtimeTimingRange period_ms;
    RealtimeTimingRange jitter_ms;
    RealtimeTimingRange wake_latency_us;
    RealtimeTimingRange pre_send_us;
    RealtimeTimingRange send_duration_us;
    uint64_t deadline_miss_count = 0;
    uint64_t catch_up_count = 0;
};

struct FeedbackRealtimeTimingTelemetry {
    double frame_rate_hz = 0.0;
    double fresh_rate_hz = 0.0;
    uint64_t held_count = 0;
    RealtimeTimingRange period_ms;
    RealtimeTimingRange jitter_ms;
    RealtimeTimingRange age_us;
    RealtimeTimingRange phase_us;
    bool freshness_reliable = false;
    bool robot_time_available = false;
    bool robot_time_monotonic = false;
};

struct RealtimeTimingTelemetry {
    double window_sec = 0.0;
    ServoRealtimeTimingTelemetry servo;
    FeedbackRealtimeTimingTelemetry left_feedback;
    FeedbackRealtimeTimingTelemetry right_feedback;
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
    bool external_box = false;  // arm<->runtime external keep-out box
    bool intra_arm = false;     // same-arm non-adjacent link pair
    bool gripper_gripper = false;  // cross-arm pair of two Pika hulls
    bool environment = false;      // arm<->cell structure (env_* geometry)
    // THIS PAIR'S OWN barrier thresholds, resolved by the same per-category selection
    // the monitor enforces with (collision_monitor.cpp: external_box -> external ->
    // intra_arm -> gripper_gripper -> self). Published because a consumer CANNOT derive
    // them: the near list is sorted by RAW clearance, so "nearest" is not "violating"
    // when the categories have different floors — on the RB5 the structural intra-arm
    // link3<->link5 pair sits at ~23 mm (floor 5 mm) and owns near[0] on 99% of ticks,
    // while an arm<->stand pair violating its own 40 mm floor ranks below it. A viewer
    // banding every pair against the single self d_hard_m therefore both mis-colors the
    // structural pairs red and names the wrong parts as the colliding ones.
    double d_hard_m = 0.0;
    double d_slow_m = 0.0;
    // Signed clearance rate of THIS pair, + = separating. Published so a viewer can
    // tell a pair the barrier is ACTING on from one that merely sits inside the band
    // forever: the RB5's shoulders park link1 at 82-84 mm against the 90 mm self slow
    // band on every tick, so "inside d_slow" alone paints nine permanent tubes that
    // never mean anything. The barrier only removes the CLOSING component, so closing
    // is the property that separates the two.
    double rate_m_s = 0.0;
};

struct ServoSnapshot {
    uint64_t tick = 0;
    uint64_t loop_start_time_ns = 0;
    uint64_t loop_end_time_ns = 0;

    RobotState left_state;
    RobotState right_state;
    uint64_t motion_epoch = 0;

    DualArmCommand command;

    JointArray left_sent_q_deg{};
    JointArray right_sent_q_deg{};
    JointArray left_prev_sent_q_deg{};
    JointArray right_prev_sent_q_deg{};

    double period_ms = 0.0;
    double jitter_ms = 0.0;
    double filter_dt_ms = 0.0;

    // Producer-side rolling cadence statistics. Absent for snapshots that did
    // not originate from the live servo loop (for example static unit-test or
    // compatibility snapshots).
    std::optional<RealtimeTimingTelemetry> realtime_timing;

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
    double self_collision_external_box_min_clearance_m = std::numeric_limits<double>::infinity();
    std::vector<double> self_collision_external_box_clearance_m;
    double self_collision_verdict_age_ms = -1.0;
    double self_collision_eval_ms = -1.0;
    int self_collision_near_count = 0;
    double self_collision_self_min_clearance_m = std::numeric_limits<double>::infinity();
    double self_collision_intra_arm_min_clearance_m = std::numeric_limits<double>::infinity();
    double self_collision_gripper_min_clearance_m = std::numeric_limits<double>::infinity();
    bool self_collision_gripper_excluded = false;
    uint64_t self_collision_clamp_count = 0;
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
    InitMotionDiag init_motion;
    InitMotionArmDiag init_motion_left;
    InitMotionArmDiag init_motion_right;

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
    // The same ROI evaluation run on the MEASURED joints instead of the commanded
    // target. The enforcing evaluation above answers "is the pose the policy asked
    // for outside the box"; the damper's whole job is to keep that from becoming
    // true of the arm, so a supervisor that wants to know where the ARM actually is
    // has to be told separately. Same offset points, same gripper-open
    // interpolation, same effective bounds -- so it can never disagree with the
    // layer that is doing the stopping.
    bool roi_box_left_measured_checked = false;
    bool roi_box_left_measured_violated = false;
    double roi_box_left_measured_min_margin_m = 0.0;
    std::string roi_box_left_measured_closest_face;
    bool roi_box_right_measured_checked = false;
    bool roi_box_right_measured_violated = false;
    double roi_box_right_measured_min_margin_m = 0.0;
    std::string roi_box_right_measured_closest_face;
    uint64_t roi_box_clamp_count = 0;
    std::string roi_box_last_set_reject_reason;

    // REACH SHELL (safety.reach_constraint), published 2026-09-04. The GUI drew this
    // limit from a STATIC asset (reach_envelope_rb5_850e.npz, r_max_recommended
    // 1.2526) while the server enforced whatever the config said — 0.980 that
    // evening, a 273 mm lie — so the operator could turn the overlay on and still
    // not see the surface the arm was actually stopping against. The enforced radii
    // are server state, so the viewer has to read them from here.
    // base_stand is the shell CENTER (the arm mount origin in the stand frame),
    // without which a radius cannot be drawn in the right place.
    bool reach_shell_enabled = false;
    bool reach_shell_monitor_only = false;
    double reach_shell_r_max_m = 0.0;
    double reach_shell_r_min_m = 0.0;
    double reach_shell_d_slow_m = 0.0;          // engage band: braking starts at r_max - d_slow
    std::array<double, 3> reach_shell_left_base_stand_m{};
    std::array<double, 3> reach_shell_right_base_stand_m{};
    bool reach_shell_left_checked = false;
    bool reach_shell_left_violated = false;
    double reach_shell_left_min_margin_m = 0.0;  // closest shell margin (>=0 inside)
    double reach_shell_left_r_far_m = 0.0;       // radius of the most-exposed checked point
    std::string reach_shell_left_closest_shell;  // r_min | r_max
    bool reach_shell_right_checked = false;
    bool reach_shell_right_violated = false;
    double reach_shell_right_min_margin_m = 0.0;
    double reach_shell_right_r_far_m = 0.0;
    std::string reach_shell_right_closest_shell;
    uint64_t reach_shell_clamp_count = 0;

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
    FtTelemetry left_ft;
    FtTelemetry right_ft;
    ForceControlTelemetry left_force_control;
    ForceControlTelemetry right_force_control;
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
std::string toString(BackendAckPolicy policy);
std::string toString(SafetyVerdict verdict);
std::string toString(FaultDomain domain);
std::string toString(TrackingErrorPolicy policy);
ControlMode controlModeFromString(const std::string& mode);
JointTargetProfile jointTargetProfileFromString(const std::string& value);
TrackingErrorPolicy trackingErrorPolicyFromString(const std::string& value);

}  // namespace rb_servo
