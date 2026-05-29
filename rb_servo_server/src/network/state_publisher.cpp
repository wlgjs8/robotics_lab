#include "rb_servo/network/state_publisher.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace rb_servo {
namespace {

constexpr int kStateSchemaVersion = 1;

struct UdpDestination {
    std::string endpoint;
    std::vector<char> storage;
    socklen_t len{0};

    const sockaddr* addr() const {
        return reinterpret_cast<const sockaddr*>(storage.data());
    }
};

std::chrono::nanoseconds publishPeriod(int state_pub_rate_hz) {
    const int rate_hz = state_pub_rate_hz > 0 ? state_pub_rate_hz : 20;
    return std::chrono::nanoseconds(1'000'000'000LL / rate_hz);
}

nlohmann::json jointArrayJson(const JointArray& joints) {
    nlohmann::json out = nlohmann::json::array();
    for (double value : joints) out.push_back(value);
    return out;
}

nlohmann::json optionalJointArrayJson(const std::optional<JointArray>& joints) {
    if (!joints.has_value()) return nullptr;
    return jointArrayJson(*joints);
}

nlohmann::json stringArrayJson(const std::vector<std::string>& values) {
    nlohmann::json out = nlohmann::json::array();
    for (const std::string& value : values) out.push_back(value);
    return out;
}

nlohmann::json qRangeViolationsJson(const std::vector<JointRangeViolation>& violations) {
    nlohmann::json out = nlohmann::json::array();
    for (const JointRangeViolation& violation : violations) {
        out.push_back({
            {"joint", violation.joint},
            {"value_deg", violation.value_deg},
            {"min_deg", violation.min_deg},
            {"max_deg", violation.max_deg},
        });
    }
    return out;
}

nlohmann::json qRangeWrappedJson(const std::vector<JointRangeWrapped>& wrapped_joints) {
    nlohmann::json out = nlohmann::json::array();
    for (const JointRangeWrapped& wrapped : wrapped_joints) {
        out.push_back({
            {"joint", wrapped.joint},
            {"raw_deg", wrapped.raw_deg},
            {"normalized_deg", wrapped.normalized_deg},
            {"period_deg", wrapped.period_deg},
        });
    }
    return out;
}

nlohmann::json optionalDoubleJson(const std::optional<double>& value) {
    if (!value.has_value()) return nullptr;
    return *value;
}

std::string orientationModeString(LinearMoveOrientationMode mode) {
    switch (mode) {
        case LinearMoveOrientationMode::Constant:
            return "constant";
        case LinearMoveOrientationMode::Slerp:
            return "slerp";
    }
    return "unknown";
}

std::string cartesianLimitPolicyString(CartesianLimitPolicy policy) {
    switch (policy) {
        case CartesianLimitPolicy::Clamp:
            return "clamp";
        case CartesianLimitPolicy::Reject:
            return "reject";
    }
    return "unknown";
}

std::string velocityIntegrationModeString(CartesianVelocityTargetIntegrationMode mode) {
    switch (mode) {
        case CartesianVelocityTargetIntegrationMode::MeasuredActual:
            return "measured_actual";
        case CartesianVelocityTargetIntegrationMode::MeasuredActualLookahead:
            return "measured_actual_lookahead";
        case CartesianVelocityTargetIntegrationMode::PreviousCommand:
            return "previous_command";
    }
    return "unknown";
}

std::string commandActualErrorPolicyString(CartesianCommandActualErrorPolicy policy) {
    switch (policy) {
        case CartesianCommandActualErrorPolicy::Reset:
            return "reset";
        case CartesianCommandActualErrorPolicy::Fault:
            return "fault";
    }
    return "unknown";
}

nlohmann::json cartesianControlSnapshotJson(const CartesianControlConfig& config) {
    return {
        {"schema", "robotics_lab.cartesian_control_snapshot.v1"},
        {"enable", config.enable},
        {"allow_in_simulation", config.allow_in_simulation},
        {"allow_in_real", config.allow_in_real},
        {"enable_benchmark_primitives", config.enable_benchmark_primitives},
        {"warn_ik_duration_us", config.warn_ik_duration_us},
        {"fail_ik_duration_us", config.fail_ik_duration_us},
        {"path_kp", config.path_kp},
        {"path_kp_pos", config.path_kp_pos},
        {"path_kp_ori", config.path_kp_ori},
        {"twist_orientation_hold_kp", config.twist_orientation_hold_kp},
        {"twist_angular_deadband_rad_s", config.twist_angular_deadband_rad_s},
        {"velocity_damping", config.velocity_damping},
        {"max_twist_linear_m_s", config.max_twist_linear_m_s},
        {"max_twist_angular_rad_s", config.max_twist_angular_rad_s},
        {"max_linear_move_speed_m_s", config.max_linear_move_speed_m_s},
        {"max_angular_move_speed_rad_s", config.max_angular_move_speed_rad_s},
        {"max_cartesian_step_m", optionalDoubleJson(config.max_cartesian_step_m)},
        {"max_cartesian_step_rad", optionalDoubleJson(config.max_cartesian_step_rad)},
        {"exceed_limit_policy", cartesianLimitPolicyString(config.exceed_limit_policy)},
        {"velocity_target_integration", velocityIntegrationModeString(config.velocity_target_integration)},
        {"velocity_target_lookahead_sec", config.velocity_target_lookahead_sec},
        {"max_command_actual_error_deg", jointArrayJson(config.max_command_actual_error_deg)},
        {"reset_velocity_integrator_on_mode_change", config.reset_velocity_integrator_on_mode_change},
        {"command_actual_error_policy", commandActualErrorPolicyString(config.command_actual_error_policy)},
        {"linear_move", {
            {"min_duration_sec", config.linear_move.min_duration_sec},
            {"max_duration_sec", config.linear_move.max_duration_sec},
            {"default_linear_speed_m_s", config.linear_move.default_linear_speed_m_s},
            {"default_angular_speed_rad_s", config.linear_move.default_angular_speed_rad_s},
            {"constant_orientation_tolerance_rad", config.linear_move.constant_orientation_tolerance_rad},
            {"default_orientation_mode", orientationModeString(config.linear_move.default_orientation_mode)},
        }},
        {"circle_move", {
            {"allow_in_simulation", config.circle_move.allow_in_simulation},
            {"allow_in_real", config.circle_move.allow_in_real},
            {"max_diameter_m", config.circle_move.max_diameter_m},
            {"min_period_sec", config.circle_move.min_period_sec},
        }},
    };
}

nlohmann::json kinematicsSnapshotJson(const KinematicsConfig& config) {
    return {
        {"schema", "robotics_lab.kinematics_snapshot.v1"},
        {"enable", config.enable},
        {"provider", config.provider},
        {"urdf", config.urdf},
        {"base_link", config.base_link},
        {"tip_link", config.tip_link},
        {"joint_names", stringArrayJson(config.joint_names)},
        {"q_units", config.q_units},
        {"publish_tcp", config.publish_tcp},
        {"ik", {
            {"max_iterations", config.ik.max_iterations},
            {"timeout_ms", config.ik.timeout_ms},
            {"damping", config.ik.damping},
            {"position_tolerance_m", config.ik.position_tolerance_m},
            {"orientation_tolerance_rad", config.ik.orientation_tolerance_rad},
            {"max_step_deg", jointArrayJson(config.ik.max_step_deg)},
        }},
    };
}

nlohmann::json quaternionJson(const std::optional<std::array<double, 4>>& quaternion_xyzw) {
    if (!quaternion_xyzw) return nullptr;
    const auto& q = *quaternion_xyzw;
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!std::isfinite(norm) || norm <= 0.0) return nullptr;
    return {
        q[0] / norm,
        q[1] / norm,
        q[2] / norm,
        q[3] / norm,
    };
}

nlohmann::json poseJson(const Pose6D& pose) {
    nlohmann::json out = {
        {"x", pose.x},
        {"y", pose.y},
        {"z", pose.z},
        {"rx", pose.rx},
        {"ry", pose.ry},
        {"rz", pose.rz},
    };
    const nlohmann::json quaternion_xyzw = quaternionJson(pose.quaternion_xyzw);
    if (!quaternion_xyzw.is_null()) {
        out["quaternion_xyzw"] = quaternion_xyzw;
        out["qx"] = quaternion_xyzw.at(0);
        out["qy"] = quaternion_xyzw.at(1);
        out["qz"] = quaternion_xyzw.at(2);
        out["qw"] = quaternion_xyzw.at(3);
    }
    return out;
}

nlohmann::json optionalPoseJson(const std::optional<Pose6D>& pose) {
    if (!pose) return nullptr;
    return poseJson(*pose);
}

nlohmann::json optionalBoolJson(const std::optional<bool>& value) {
    if (!value.has_value()) return nullptr;
    return *value;
}

nlohmann::json optionalStringJson(const std::string& value) {
    if (value.empty()) return nullptr;
    return value;
}

nlohmann::json rbpodoTimeJson(double value) {
    if (!std::isfinite(value)) return nullptr;
    return value;
}

nlohmann::json rbpodoRawDiagnosticsJson(const RbpodoRawDiagnostics& raw) {
    return {
        {"time", rbpodoTimeJson(raw.time_sec)},
        {"real_vs_simulation_mode", raw.real_vs_simulation_mode},
        {"init_state_info", raw.init_state_info},
        {"init_error", raw.init_error},
        {"op_stat_sos_flag", raw.op_stat_sos_flag},
        {"op_stat_ems_flag", raw.op_stat_ems_flag},
        {"op_stat_soft_estop_occur", raw.op_stat_soft_estop_occur},
        {"op_stat_collision_occur", raw.op_stat_collision_occur},
        {"op_stat_self_collision", raw.op_stat_self_collision},
    };
}

nlohmann::json optionalRbpodoDiagnosticsJson(
    const std::optional<RbpodoDiagnosticsSnapshot>& diagnostics
) {
    if (!diagnostics.has_value()) return nullptr;
    return {
        {"diagnostics_valid", diagnostics->diagnostics_valid},
        {"diagnostics_suspect", diagnostics->diagnostics_suspect},
        {"reason", optionalStringJson(diagnostics->reason)},
        {"error_name", optionalStringJson(diagnostics->error_name)},
        {"stable_error_code", diagnostics->stable_error_code},
        {"raw", rbpodoRawDiagnosticsJson(diagnostics->raw)},
    };
}

double ageUs(uint64_t newer_ns, uint64_t older_ns) {
    if (newer_ns == 0 || older_ns == 0 || newer_ns < older_ns) return 0.0;
    return static_cast<double>(newer_ns - older_ns) / 1000.0;
}

bool sendWithinPeriod(uint64_t loop_start_ns, double period_ms, uint64_t send_end_ns) {
    if (loop_start_ns == 0 || send_end_ns == 0 || period_ms <= 0.0) return false;
    const auto period_ns = static_cast<uint64_t>(period_ms * 1'000'000.0);
    return send_end_ns <= loop_start_ns + period_ns;
}

bool sendPeriodOverrun(uint64_t loop_start_ns, double period_ms, uint64_t send_end_ns) {
    if (loop_start_ns == 0 || send_end_ns == 0 || period_ms <= 0.0) return false;
    return !sendWithinPeriod(loop_start_ns, period_ms, send_end_ns);
}

nlohmann::json backendCallJson(const BackendCallSnapshot& call, bool send_call) {
    nlohmann::json out = {
        {"backend_error_kind", call.backend_error_kind},
        {"error_name", call.error_name},
        {"error_code", call.error_code},
        {"duration_us", call.duration_us},
    };
    if (send_call) {
        out["accepted"] = call.accepted;
        out["state_after_source"] = call.state_after_source;
        out["ack_policy"] = toString(call.ack_policy);
        out["ack_observed"] = call.ack_observed;
        out["controller_acceptance_observed"] = call.controller_acceptance_observed;
        out["ack_wait_duration_us"] = call.ack_wait_duration_us;
        out["rbpodo_waiting_ack"] = call.rbpodo_waiting_ack;
        out["send_acceptance_semantics"] = call.acceptance_semantics;
    } else {
        out["ok"] = call.ok;
    }
    return out;
}

nlohmann::json startupArmValidationJson(const ArmStartupValidationSnapshot& validation) {
    return {
        {"acquisition_ok", validation.acquisition_ok},
        {"motion_ready", validation.motion_ready},
        {"read_only_diagnostic", validation.read_only_diagnostic},
        {"allowed_unsafe_startup", validation.allowed_unsafe_startup},
        {"invalid_reasons", stringArrayJson(validation.invalid_reasons)},
        {"q_range_violations", qRangeViolationsJson(validation.q_range_violations)},
        {"q_range_wrapped", qRangeWrappedJson(validation.q_range_wrapped)},
        {"q_actual_normalized_for_safety_deg", optionalJointArrayJson(validation.q_actual_normalized_for_safety_deg)},
        {"diagnostic_error_source", optionalStringJson(validation.diagnostic_error_source)},
    };
}

nlohmann::json startupValidationJson(const StartupValidationSnapshot& validation) {
    return {
        {"acquisition_ok", validation.acquisition_ok},
        {"motion_ready", validation.motion_ready},
        {"read_only_diagnostic", validation.read_only_diagnostic},
        {"allowed_unsafe_startup", validation.allowed_unsafe_startup},
        {"left", startupArmValidationJson(validation.left)},
        {"right", startupArmValidationJson(validation.right)},
    };
}

uint64_t workerReadPeriodNs(const ServoConfig& config, bool enabled) {
    if (!enabled || config.worker_read_period_sec <= 0.0 || !std::isfinite(config.worker_read_period_sec)) {
        return 0;
    }
    const double period_ns = config.worker_read_period_sec * 1'000'000'000.0;
    if (!std::isfinite(period_ns) ||
        period_ns >= static_cast<double>(std::numeric_limits<uint64_t>::max())) {
        return std::numeric_limits<uint64_t>::max();
    }
    return static_cast<uint64_t>(std::llround(period_ns));
}

double workerReadRateHz(const ServoConfig& config, bool enabled) {
    if (!enabled || config.worker_read_period_sec <= 0.0 || !std::isfinite(config.worker_read_period_sec)) {
        return 0.0;
    }
    return 1.0 / config.worker_read_period_sec;
}

nlohmann::json workerTelemetryJson(
    const ArmWorkerTelemetry& telemetry,
    bool enabled,
    const ServoConfig& config,
    double state_age_us
) {
    return {
        {"enabled", enabled},
        {"queue_policy", telemetry.worker_queue_policy},
        {"read_period_ns", workerReadPeriodNs(config, enabled)},
        {"read_period_sec", enabled ? config.worker_read_period_sec : 0.0},
        {"read_rate_hz", workerReadRateHz(config, enabled)},
        {"state_age_us", enabled ? state_age_us : 0.0},
        {"command_drops_total", telemetry.worker_command_drops_total},
        {"pending_overwrites_total", telemetry.worker_pending_overwrites_total},
        {"last_dropped_seq", telemetry.worker_last_dropped_seq},
        {"last_enqueued_seq", telemetry.worker_last_enqueued_seq},
        {"last_dispatched_seq", telemetry.worker_last_dispatched_seq},
        {"last_completed_seq", telemetry.worker_last_completed_seq},
    };
}

nlohmann::json transportTelemetryJson(const std::optional<BackendTransportTelemetry>& telemetry) {
    if (!telemetry) return nullptr;
    return {
        {"connect_attempts_total", telemetry->connect_attempts_total},
        {"connect_failures_total", telemetry->connect_failures_total},
        {"connect_attempts_suppressed_total", telemetry->connect_attempts_suppressed_total},
        {"connections_opened_total", telemetry->connections_opened_total},
        {"reconnects_total", telemetry->reconnects_total},
        {"requests_total", telemetry->requests_total},
        {"read_syscalls_total", telemetry->read_syscalls_total},
        {"write_syscalls_total", telemetry->write_syscalls_total},
        {"last_connect_error_name", telemetry->last_connect_error_name},
        {"last_connect_error_message", telemetry->last_connect_error_message},
        {"next_connect_attempt_ns", telemetry->next_connect_attempt_ns},
        {"next_connect_attempt_delay_ms", telemetry->next_connect_attempt_delay_ms},
        {"last_transport_error_kind", telemetry->last_transport_error_kind},
    };
}

nlohmann::json cartesianSolveJson(const CartesianSolveTelemetry& telemetry) {
    return {
        {"attempted", telemetry.attempted},
        {"success", telemetry.success},
        {"status", telemetry.status},
        {"reason", telemetry.reason},
        {"fk_duration_us", telemetry.fk_duration_us},
        {"ik_duration_us", telemetry.ik_duration_us},
        {"ik_iterations", telemetry.ik_iterations},
        {"position_error_m", telemetry.position_error_m},
        {"orientation_error_rad", telemetry.orientation_error_rad},
        {"ik_status", telemetry.status},
        {"ik_reason", telemetry.reason},
        {"ik_timed_out", telemetry.ik_timed_out},
        {"ik_warn_duration_exceeded", telemetry.ik_warn_duration_exceeded},
        {"ik_fail_duration_exceeded", telemetry.ik_fail_duration_exceeded},
        {"warn_ik_duration_us", telemetry.warn_ik_duration_us},
        {"fail_ik_duration_us", telemetry.fail_ik_duration_us},
        {"path_active", telemetry.path_active},
        {"path_s", telemetry.path_s},
        {"path_position_error_m", telemetry.path_position_error_m},
        {"path_orientation_error_rad", telemetry.path_orientation_error_rad},
        {"path_line_deviation_m", telemetry.path_line_deviation_m},
        {"path_done", telemetry.path_done},
        {"path_completion_hold", telemetry.path_done && !telemetry.path_active},
        {"path_elapsed_sec", telemetry.linear_move_elapsed_sec},
        {"linear_move_duration_sec", telemetry.linear_move_duration_sec},
        {"linear_move_elapsed_sec", telemetry.linear_move_elapsed_sec},
        {"orientation_mode", telemetry.orientation_mode},
        {"twist_clamped", telemetry.twist_clamped},
        {"requested_twist_linear_norm_m_s", telemetry.requested_twist_linear_norm_m_s},
        {"requested_twist_angular_norm_rad_s", telemetry.requested_twist_angular_norm_rad_s},
        {"applied_twist_linear_norm_m_s", telemetry.applied_twist_linear_norm_m_s},
        {"applied_twist_angular_norm_rad_s", telemetry.applied_twist_angular_norm_rad_s},
        {"cartesian_velocity_integration_mode", telemetry.cartesian_velocity_integration_mode},
        {"q_integrator_valid", telemetry.q_integrator_valid},
        {"integrator_reset_reason", telemetry.integrator_reset_reason},
        {"integrator_resets_total", telemetry.integrator_resets_total},
        {"integrator_clamps_total", telemetry.integrator_clamps_total},
        {"integrator_divergence_total", telemetry.integrator_divergence_total},
        {"max_command_actual_error_deg_observed", telemetry.max_command_actual_error_deg_observed},
        {"velocity_target_lookahead_sec", telemetry.velocity_target_lookahead_sec},
        {"circle_active", telemetry.circle_active},
        {"circle_phase", telemetry.circle_phase},
        {"circle_repeat_index", telemetry.circle_repeat_index},
        {"circle_radius_m", telemetry.circle_radius_m},
        {"circle_period_sec", telemetry.circle_period_sec},
        {"circle_position_error_m", telemetry.circle_position_error_m},
        {"circle_orientation_error_rad", telemetry.circle_orientation_error_rad},
        {"circle_done", telemetry.circle_done},
    };
}

nlohmann::json faultContextDetailJson(const LatchedFaultContextSnapshot& context) {
    return {
        {"verdict", context.verdict},
        {"domain", context.domain},
        {"arm", context.arm},
        {"backend_op", context.backend_op},
        {"backend_error_kind", context.backend_error_kind},
        {"backend_error_name", context.backend_error_name},
        {"backend_error_code", context.backend_error_code},
        {"retryable", context.retryable},
        {"recoverable", context.recoverable},
        {"robot_fault", context.robot_fault},
        {"transport_fault", context.transport_fault},
        {"state_after_source", context.state_after_source},
        {"reason", context.reason},
    };
}

nlohmann::json optionalFaultContextDetailJson(
    const std::optional<LatchedFaultContextSnapshot>& context
) {
    if (!context.has_value()) return nullptr;
    return faultContextDetailJson(*context);
}

nlohmann::json faultContextJson(const ServoSnapshot& snapshot) {
    nlohmann::json out = {
        {"latched", snapshot.fault_latched},
        {"motion_state", toString(snapshot.motion_state)},
        {"safety_verdict", toString(snapshot.safety_verdict)},
        {"latched_fault_reason", toString(snapshot.latched_fault_reason)},
        {"reason", snapshot.fault_reason},
        {"top_level", optionalFaultContextDetailJson(snapshot.latched_fault_context)},
        {"left", optionalFaultContextDetailJson(snapshot.left_latched_fault_context)},
        {"right", optionalFaultContextDetailJson(snapshot.right_latched_fault_context)},
    };
    if (!snapshot.latched_fault_context.has_value()) {
        out["verdict"] = nullptr;
        out["domain"] = nullptr;
        out["arm"] = nullptr;
        out["backend_op"] = nullptr;
        out["backend_error_kind"] = nullptr;
        out["backend_error_name"] = nullptr;
        out["backend_error_code"] = nullptr;
        out["retryable"] = nullptr;
        out["recoverable"] = nullptr;
        out["robot_fault"] = nullptr;
        out["transport_fault"] = nullptr;
        out["state_after_source"] = nullptr;
        return out;
    }

    const LatchedFaultContextSnapshot& context = *snapshot.latched_fault_context;
    const nlohmann::json detail = faultContextDetailJson(context);
    out.update(detail);
    return out;
}

nlohmann::json commandSourceJson(const ServoSnapshot& snapshot, const DualArmConfig& config) {
    const CommandSourceLeaseState& lease = snapshot.command.lease;
    const bool lease_expired = lease.active &&
        lease.expires_time_ns > 0 &&
        snapshot.loop_end_time_ns > lease.expires_time_ns;
    const bool active = lease.active && !lease_expired;
    return {
        {"source_id", optionalStringJson(snapshot.command.source.source_id)},
        {"session_id", optionalStringJson(snapshot.command.source.session_id)},
        {"lease_token", optionalStringJson(snapshot.command.source.lease_token)},
        {"source_priority", snapshot.command.source.source_priority.has_value()
            ? nlohmann::json(*snapshot.command.source.source_priority)
            : nlohmann::json(nullptr)},
        {"enforce_lease", config.command_source.enforce_lease || lease.enforce_lease},
        {"lease_timeout_sec", config.command_source.lease_timeout_sec},
        {"active", active},
        {"expired", lease_expired},
        {"active_source_id", optionalStringJson(lease.source_id)},
        {"active_session_id", optionalStringJson(lease.session_id)},
        {"active_lease_token", optionalStringJson(lease.lease_token)},
        {"acquired_time_ns", lease.acquired_time_ns},
        {"expires_time_ns", lease.expires_time_ns},
        {"command_requires_lease", lease.command_requires_lease},
        {"command_has_lease", lease.command_has_lease},
        {"verdict", lease_expired ? "Expired" : lease.verdict},
        {"reason", lease_expired ? "command source lease expired" : lease.reason},
    };
}

std::string runModeString(RunMode mode) {
    switch (mode) {
        case RunMode::Real: return "real";
        case RunMode::Simulation: return "simulation";
        case RunMode::Mock: return "mock";
    }
    return "unknown";
}

std::string backendTypeString(BackendType backend_type) {
    switch (backend_type) {
        case BackendType::Rbpodo: return "rbpodo";
        case BackendType::Mock: return "mock";
        case BackendType::Simulator: return "simulator";
        case BackendType::RbscriptTcp: return "rbscript_tcp";
    }
    return "unknown";
}

std::string observedModeString(const DualArmConfig& config) {
    if (config.left_robot.run_mode == config.right_robot.run_mode) {
        return runModeString(config.left_robot.run_mode);
    }
    return "mixed";
}

std::string observedBackendString(const DualArmConfig& config) {
    if (config.left_robot.backend_type == config.right_robot.backend_type) {
        return backendTypeString(config.left_robot.backend_type);
    }
    return "mixed";
}

nlohmann::json armStateJson(
    const RobotState& state,
    const ArmCommand& command,
    const JointArray& sent_q_deg,
    const JointArray& previous_sent_q_deg,
    bool send_ok,
    const BackendCallSnapshot& last_read,
    const BackendCallSnapshot& last_send,
    const std::string& send_error_kind,
    const std::string& send_error_name,
    const std::string& send_error_code,
    const std::string& send_error_message,
    uint64_t send_start_ns,
    uint64_t send_end_ns,
    double send_duration_us,
    double state_age_us,
    double send_result_age_us,
    bool send_within_period,
    bool send_period_overrun,
    double worker_loop_read_duration_us,
    const ArmWorkerTelemetry& worker_telemetry,
    const std::optional<BackendTransportTelemetry>& transport_telemetry,
    bool worker_enabled,
    const ServoConfig& servo_config,
    const CartesianSolveTelemetry& cartesian_solve,
    const ArmStartupValidationSnapshot& startup_validation
) {
    return {
        {"mode", toString(command.mode)},
        {"q_actual_deg", jointArrayJson(state.q_actual_deg)},
        {"q_sent_deg", jointArrayJson(sent_q_deg)},
        {"q_previous_sent_deg", jointArrayJson(previous_sent_q_deg)},
        {"send_ok", send_ok},
        {"send_error_kind", send_error_kind},
        {"send_error_name", send_error_name},
        {"send_error_code", send_error_code},
        {"send_error_message", send_error_message},
        {"send_start_ns", send_start_ns},
        {"send_end_ns", send_end_ns},
        {"send_duration_us", send_duration_us},
        {"state_age_us", state_age_us},
        {"send_result_age_us", send_result_age_us},
        {"send_within_period", send_within_period},
        {"send_period_overrun", send_period_overrun},
        {"send_command_deadline_missed", nlohmann::json(nullptr)},
        {"send_deadline_hit", send_within_period},
        {"send_deadline_hit_deprecated_alias_for", "send_within_period"},
        {"worker_loop_read_duration_us", worker_loop_read_duration_us},
        {"worker", workerTelemetryJson(worker_telemetry, worker_enabled, servo_config, state_age_us)},
        {"transport", transportTelemetryJson(transport_telemetry)},
        {"has_valid_joint_state", state.has_valid_joint_state},
        {"startup_acquisition_ok", startup_validation.acquisition_ok},
        {"startup_motion_ready", startup_validation.motion_ready},
        {"startup_invalid_reasons", stringArrayJson(startup_validation.invalid_reasons)},
        {"q_range_violations", qRangeViolationsJson(startup_validation.q_range_violations)},
        {"q_range_wrapped", qRangeWrappedJson(startup_validation.q_range_wrapped)},
        {"q_actual_normalized_for_safety_deg", optionalJointArrayJson(startup_validation.q_actual_normalized_for_safety_deg)},
        {"read_only_diagnostic", startup_validation.read_only_diagnostic},
        {"allowed_unsafe_startup", startup_validation.allowed_unsafe_startup},
        {"diagnostic_error_source", optionalStringJson(startup_validation.diagnostic_error_source)},
        {"connection_state", state.connection_state == RobotConnectionState::Connected
            ? "Connected"
            : state.connection_state == RobotConnectionState::Error ? "Error" : "Disconnected"},
        {"has_error", state.has_error},
        {"servo_enabled", state.servo_enabled},
        {"fault_recoverable", optionalBoolJson(state.fault_recoverable)},
        {"lifecycle_state", optionalStringJson(state.lifecycle_state)},
        {"motion_readiness_error_kind", optionalStringJson(state.motion_readiness_error_kind)},
        {"motion_readiness_error_name", optionalStringJson(state.motion_readiness_error_name)},
        {"rbpodo_diagnostics", optionalRbpodoDiagnosticsJson(state.rbpodo_diagnostics)},
        {"last_read", backendCallJson(last_read, false)},
        {"last_send", backendCallJson(last_send, true)},
        {"robot_time_ns", state.robot_time_ns},
        {"host_time_ns", state.host_time_ns},
        {"error_code", state.error_code},
        {"tcp_stand", optionalPoseJson(state.tcp_stand)},
        {"tcp_base", optionalPoseJson(state.tcp_base)},
        {"has_valid_tcp_pose", state.has_valid_tcp_pose},
        {"tcp_deferred", state.tcp_deferred},
        {"fk_duration_us", state.fk_duration_us},
        {"cartesian_solve", cartesianSolveJson(cartesian_solve)},
    };
}

bool resolveUdpEndpoint(const std::string& endpoint, UdpDestination* destination) {
    std::string host;
    int port = 0;
    if (!StatePublisher::parseUdpEndpointUri(endpoint, &host, &port)) {
        std::cerr << "[ERROR] StatePublisher only supports udp://host:port endpoints, got "
                  << endpoint << "\n";
        return false;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    addrinfo* results = nullptr;
    const std::string port_string = std::to_string(port);
    const int gai = ::getaddrinfo(host.c_str(), port_string.c_str(), &hints, &results);
    if (gai != 0 || results == nullptr) {
        std::cerr << "[ERROR] StatePublisher failed to resolve host '" << host
                  << "': " << ::gai_strerror(gai) << "\n";
        return false;
    }

    for (addrinfo* item = results; item != nullptr; item = item->ai_next) {
        if (!item->ai_addr || item->ai_addrlen <= 0) continue;
        destination->endpoint = endpoint;
        destination->storage.assign(
            reinterpret_cast<const char*>(item->ai_addr),
            reinterpret_cast<const char*>(item->ai_addr) + item->ai_addrlen
        );
        destination->len = static_cast<socklen_t>(item->ai_addrlen);
        ::freeaddrinfo(results);
        return true;
    }
    ::freeaddrinfo(results);

    std::cerr << "[ERROR] StatePublisher found no UDP address for host '" << host << "'\n";
    return false;
}

std::vector<UdpDestination> resolveUdpDestinations(const NetworkConfig& config) {
    std::vector<std::string> endpoints = config.state_pub_endpoints;
    if (endpoints.empty()) {
        endpoints.push_back(config.state_pub_bind);
    }
    std::vector<UdpDestination> destinations;
    destinations.reserve(endpoints.size());
    for (const std::string& endpoint : endpoints) {
        UdpDestination destination;
        if (!resolveUdpEndpoint(endpoint, &destination)) {
            continue;
        }
        destinations.push_back(std::move(destination));
    }
    return destinations;
}

bool validateUdpEndpointSyntax(const NetworkConfig& config) {
    std::vector<std::string> endpoints = config.state_pub_endpoints;
    if (endpoints.empty()) {
        endpoints.push_back(config.state_pub_bind);
    }
    for (const std::string& endpoint : endpoints) {
        if (!StatePublisher::parseUdpEndpointUri(endpoint, nullptr, nullptr)) {
            std::cerr << "[ERROR] StatePublisher only supports udp://host:port endpoints, got "
                      << endpoint << "\n";
            return false;
        }
    }
    return true;
}

void normalizeStatePublisherNetworkConfig(NetworkConfig* config) {
    if (!config) return;
    if (config->state_pub_bind != config->state_pub_endpoint) {
        config->state_pub_endpoint = config->state_pub_bind;
        config->state_pub_endpoints = {config->state_pub_bind};
        return;
    }
    if (config->state_pub_endpoints.empty() || config->state_pub_endpoints.front() != config->state_pub_endpoint) {
        config->state_pub_endpoints = {config->state_pub_endpoint};
    }
}

}  // namespace

StatePublisher::StatePublisher(const DualArmConfig& config, SnapshotProvider provider)
    : config_(config), snapshot_provider_(std::move(provider)) {
    normalizeStatePublisherNetworkConfig(&config_.network);
}

StatePublisher::StatePublisher(const NetworkConfig& config) {
    config_.network = config;
    normalizeStatePublisherNetworkConfig(&config_.network);
    config_.command_source.enforce_lease = config.command_source_enforce_lease;
    config_.command_source.lease_timeout_sec = config.command_source_lease_timeout_sec;
}

StatePublisher::~StatePublisher() {
    stop();
}

void StatePublisher::updateSnapshot(const ServoSnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_snapshot_ = snapshot;
}

std::string StatePublisher::serializeSnapshot(const ServoSnapshot& snapshot) const {
    nlohmann::json message;
    message["schema_version"] = kStateSchemaVersion;
    message["tick"] = snapshot.tick;
    message["loop_start_time_ns"] = snapshot.loop_start_time_ns;
    message["loop_end_time_ns"] = snapshot.loop_end_time_ns;
    message["host_time_ns"] = snapshot.loop_end_time_ns;
    message["period_ms"] = snapshot.period_ms;
    message["jitter_ms"] = snapshot.jitter_ms;
    message["filter_dt_ms"] = snapshot.filter_dt_ms;
    message["command_seq"] = snapshot.command.seq;
    message["command_source"] = commandSourceJson(snapshot, config_);
    message["observed_mode"] = observedModeString(config_);
    message["observed_backend"] = observedBackendString(config_);
    message["cartesian_control_snapshot"] = cartesianControlSnapshotJson(config_.cartesian_control);
    message["kinematics_snapshot"] = kinematicsSnapshotJson(config_.kinematics);
    message["startup_validation"] = startupValidationJson(snapshot.startup_validation);
    const bool worker_enabled = config_.servo.io_model == ServoIoModel::Worker;

    message["left"] = armStateJson(
        snapshot.left_state,
        snapshot.command.left,
        snapshot.left_sent_q_deg,
        snapshot.left_prev_sent_q_deg,
        snapshot.left_send_ok,
        snapshot.left_last_read,
        snapshot.left_last_send,
        snapshot.left_send_error_kind,
        snapshot.left_send_error_name,
        snapshot.left_send_error_code,
        snapshot.left_send_error_message,
        snapshot.left_send_start_ns,
        snapshot.left_send_end_ns,
        snapshot.left_send_duration_us,
        ageUs(snapshot.loop_end_time_ns, snapshot.left_state.host_time_ns),
        ageUs(snapshot.loop_end_time_ns, snapshot.left_send_end_ns),
        sendWithinPeriod(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.left_send_end_ns),
        sendPeriodOverrun(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.left_send_end_ns),
        worker_enabled ? snapshot.left_last_read.duration_us : 0.0,
        snapshot.left_worker_telemetry,
        snapshot.left_transport_telemetry,
        worker_enabled,
        config_.servo,
        snapshot.left_cartesian_solve,
        snapshot.startup_validation.left
    );
    message["right"] = armStateJson(
        snapshot.right_state,
        snapshot.command.right,
        snapshot.right_sent_q_deg,
        snapshot.right_prev_sent_q_deg,
        snapshot.right_send_ok,
        snapshot.right_last_read,
        snapshot.right_last_send,
        snapshot.right_send_error_kind,
        snapshot.right_send_error_name,
        snapshot.right_send_error_code,
        snapshot.right_send_error_message,
        snapshot.right_send_start_ns,
        snapshot.right_send_end_ns,
        snapshot.right_send_duration_us,
        ageUs(snapshot.loop_end_time_ns, snapshot.right_state.host_time_ns),
        ageUs(snapshot.loop_end_time_ns, snapshot.right_send_end_ns),
        sendWithinPeriod(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.right_send_end_ns),
        sendPeriodOverrun(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.right_send_end_ns),
        worker_enabled ? snapshot.right_last_read.duration_us : 0.0,
        snapshot.right_worker_telemetry,
        snapshot.right_transport_telemetry,
        worker_enabled,
        config_.servo,
        snapshot.right_cartesian_solve,
        snapshot.startup_validation.right
    );
    message["last_cartesian_solve"] = {
        {"left", cartesianSolveJson(snapshot.left_cartesian_solve)},
        {"right", cartesianSolveJson(snapshot.right_cartesian_solve)},
    };

    message["send_skew_us"] = snapshot.send_skew_us;
    message["dispatch_skew_us"] = snapshot.send_skew_us;
    const bool left_send_within_period =
        sendWithinPeriod(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.left_send_end_ns);
    const bool right_send_within_period =
        sendWithinPeriod(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.right_send_end_ns);
    const bool left_send_period_overrun =
        sendPeriodOverrun(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.left_send_end_ns);
    const bool right_send_period_overrun =
        sendPeriodOverrun(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.right_send_end_ns);
    message["send_within_period"] = left_send_within_period && right_send_within_period;
    message["send_period_overrun"] = left_send_period_overrun || right_send_period_overrun;
    message["send_command_deadline_missed"] = nullptr;
    message["send_deadline_hit"] = message["send_within_period"];
    message["send_deadline_hit_deprecated_alias_for"] = "send_within_period";
    message["send_suppressed"] = snapshot.send_suppressed;
    message["send_policy"] = snapshot.send_policy;
    message["safety_verdict"] = toString(snapshot.safety_verdict);
    message["motion_state"] = toString(snapshot.motion_state);
    message["fault_latched"] = snapshot.fault_latched;
    message["latched_fault_reason"] = toString(snapshot.latched_fault_reason);
    message["fault_reason"] = snapshot.fault_reason;
    message["fault_context"] = faultContextJson(snapshot);
    message["logger_dropped_samples"] = snapshot.logger_dropped_samples;
    message["logger_health"] = {
        {"dropped_samples", snapshot.logger_dropped_samples},
        {"ok", snapshot.logger_dropped_samples == 0},
    };
    message["mount_transform_deferred"] = false;
    message["mounts"] = {
        {"left", {
            {"frame", "stand"},
            {"base_pose_in_stand", poseJson(config_.left_mount.base_pose_in_stand)},
        }},
        {"right", {
            {"frame", "stand"},
            {"base_pose_in_stand", poseJson(config_.right_mount.base_pose_in_stand)},
        }},
    };
    message["tcp_fields_deferred"] =
        snapshot.left_state.tcp_deferred || snapshot.right_state.tcp_deferred;
    return message.dump();
}

bool StatePublisher::start() {
    if (running_) return true;

    if (!validateUdpEndpointSyntax(config_.network)) {
        return false;
    }

    running_ = true;
    thread_ = std::thread(&StatePublisher::threadMain, this);
    return true;
}

void StatePublisher::stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void StatePublisher::threadMain() {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "[ERROR] StatePublisher socket failed: " << std::strerror(errno) << "\n";
        running_ = false;
        return;
    }

    const auto publish_period = publishPeriod(config_.network.state_pub_rate_hz);
    std::vector<UdpDestination> destinations;
    auto next_resolve = std::chrono::steady_clock::time_point{};
    bool send_warned = false;
    while (running_) {
        const auto now = std::chrono::steady_clock::now();
        if (destinations.empty() || now >= next_resolve) {
            destinations = resolveUdpDestinations(config_.network);
            next_resolve = now + std::chrono::seconds(1);
        }

        ServoSnapshot snapshot;
        if (snapshot_provider_) {
            snapshot = snapshot_provider_();
            updateSnapshot(snapshot);
        } else {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            snapshot = latest_snapshot_;
        }

        const std::string payload = serializeSnapshot(snapshot);
        for (const UdpDestination& destination : destinations) {
            const ssize_t sent = ::sendto(
                fd,
                payload.data(),
                payload.size(),
                0,
                destination.addr(),
                destination.len
            );
            if (sent < 0 && !send_warned) {
                std::cerr << "[WARN] StatePublisher send failed to "
                          << destination.endpoint << ": " << std::strerror(errno) << "\n";
                send_warned = true;
            }
        }
        std::this_thread::sleep_for(publish_period);
    }

    ::close(fd);
}

bool StatePublisher::parseUdpEndpointUri(const std::string& endpoint, std::string* host, int* port) {
    constexpr const char* prefix = "udp://";
    if (endpoint.rfind(prefix, 0) != 0) return false;

    const std::string rest = endpoint.substr(std::strlen(prefix));
    if (rest.empty()) return false;

    const auto colon = rest.rfind(':');
    if (colon == std::string::npos || colon + 1 >= rest.size()) return false;

    const std::string parsed_host = rest.substr(0, colon);
    if (parsed_host.empty() || parsed_host == "0.0.0.0") return false;

    int parsed_port = 0;
    std::string port_tail;
    try {
        size_t consumed = 0;
        parsed_port = std::stoi(rest.substr(colon + 1), &consumed);
        port_tail = rest.substr(colon + 1 + consumed);
    } catch (const std::exception&) {
        return false;
    }
    if (!port_tail.empty()) return false;
    if (parsed_port <= 0 || parsed_port > 65535) return false;

    // Hostname-capable by design: Docker Compose service names such as
    // rb_servo_gui are resolved in threadMain with getaddrinfo(). Static
    // container IPs are not required for cross-container UDP state delivery.
    if (host) *host = parsed_host == "localhost" ? "127.0.0.1" : parsed_host;
    if (port) *port = parsed_port;
    return true;
}

bool StatePublisher::parseEndpoint(std::string* host, int* port) const {
    return parseUdpEndpointUri(config_.network.state_pub_bind, host, port);
}

}  // namespace rb_servo
