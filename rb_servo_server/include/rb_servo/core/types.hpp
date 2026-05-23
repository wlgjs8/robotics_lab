#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace rb_servo {

constexpr int kDof = 6;
using JointArray = std::array<double, kDof>;

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
    Simulator,
    Rbsim = Simulator  // Deprecated compatibility name.
};

enum class ControlMode {
    Idle,
    Hold,
    ArmMotion,
    DisarmMotion,
    JointTarget,
    JointVelocity,
    TcpPoseTarget,
    TcpDeltaStand,
    TcpDeltaLocal,
    EmergencyStop,
    ResetFault
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
    bool has_valid_joint_state = false;

    std::optional<Pose6D> tcp_base;
    std::optional<Pose6D> tcp_stand;
    bool has_valid_tcp_pose = false;
    bool tcp_deferred = true;
    Wrench6D wrench_tcp;

    RobotConnectionState connection_state = RobotConnectionState::Disconnected;

    bool servo_enabled = false;
    bool has_error = false;
    std::optional<bool> fault_recoverable;
    std::string lifecycle_state;
    int error_code = 0;
};

struct ArmCommand {
    ArmId arm_id = ArmId::Left;

    uint64_t seq = 0;
    uint64_t host_time_ns = 0;

    ControlMode mode = ControlMode::Hold;

    JointArray q_target_deg{};
    JointArray dq_target_deg_s{};

    Pose6D tcp_target_stand;
    Pose6D tcp_delta_stand;
    Pose6D tcp_delta_local;

    ForceControlCommand force_control;

    double gripper_target = 0.0;
    double timeout_sec = 0.2;

    // Parsed command validation flags. A command parser must set these true only
    // when the corresponding array was present and had the expected size.
    bool has_joint_target = false;
    bool has_joint_velocity = false;
    bool has_tcp_target = false;
    bool has_tcp_delta_stand = false;
    bool has_tcp_delta_local = false;
};

struct DualArmCommand {
    uint64_t seq = 0;
    uint64_t host_time_ns = 0;

    ArmCommand left;
    ArmCommand right;

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

    double period_ms = 0.0;
    double jitter_ms = 0.0;
    double filter_dt_ms = 0.0;

    SafetyVerdict safety_verdict = SafetyVerdict::Ok;
    ServerMotionState motion_state = ServerMotionState::Disconnected;
    bool fault_latched = false;
    std::string fault_reason;
    std::optional<LatchedFaultContextSnapshot> latched_fault_context;
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
    SafetyVerdict latched_fault_reason = SafetyVerdict::Ok;
    std::string fault_reason;
    std::optional<LatchedFaultContextSnapshot> latched_fault_context;

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

    uint64_t logger_dropped_samples = 0;
};

std::string toString(ArmId arm_id);
std::string toString(ControlMode mode);
std::string toString(ServerMotionState state);
std::string toString(ForceControlMode mode);
std::string toString(SafetyVerdict verdict);
std::string toString(FaultDomain domain);
std::string toString(TrackingErrorPolicy policy);
ControlMode controlModeFromString(const std::string& mode);
ForceControlMode forceControlModeFromString(const std::string& mode);
TrackingErrorPolicy trackingErrorPolicyFromString(const std::string& value);

}  // namespace rb_servo
