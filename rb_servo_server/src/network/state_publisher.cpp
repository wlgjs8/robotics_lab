#include "rb_servo/network/state_publisher.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <set>
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

bool finiteJointArray(const JointArray& joints) {
    return std::all_of(joints.begin(), joints.end(), [](double value) {
        return std::isfinite(value);
    });
}

bool qActualValidForPublication(const RobotState& state) {
    return state.q_actual_valid ||
        (state.has_valid_joint_state && finiteJointArray(state.q_actual_deg));
}

bool qRefValidForPublication(const RobotState& state) {
    return state.q_ref_valid ||
        (state.has_valid_joint_state && finiteJointArray(state.q_target_deg));
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

nlohmann::json finiteDoubleJson(double value) {
    if (!std::isfinite(value)) return nullptr;
    return value;
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

std::string controllerSimulationStateSourceString(CartesianControllerSimulationStateSource source) {
    switch (source) {
        case CartesianControllerSimulationStateSource::Actual:
            return "actual";
        case CartesianControllerSimulationStateSource::Reference:
            return "reference";
    }
    return "unknown";
}

std::string controllerSimulationTrackingSourceString(ControllerSimulationTrackingErrorSource source) {
    switch (source) {
        case ControllerSimulationTrackingErrorSource::Actual:
            return "actual";
        case ControllerSimulationTrackingErrorSource::Reference:
            return "reference";
    }
    return "unknown";
}

std::string controllerSimulationPhysicalMotionPolicyString(
    ControllerSimulationPhysicalMotionPolicy policy
) {
    switch (policy) {
        case ControllerSimulationPhysicalMotionPolicy::WarnOnly:
            return "warn_only";
        case ControllerSimulationPhysicalMotionPolicy::FaultLatch:
            return "fault_latch";
    }
    return "unknown";
}

std::string rbpodoAsyncStreamingModeString(RbpodoAsyncStreamingMode mode) {
    switch (mode) {
        case RbpodoAsyncStreamingMode::Disabled:
            return "disabled";
        case RbpodoAsyncStreamingMode::SdkAckWorker:
            return "sdk_ack_worker";
        case RbpodoAsyncStreamingMode::SocketSendSupervised:
            return "socket_send_supervised";
    }
    return "unknown";
}

std::string rbpodoAsyncQueuePolicyString(RbpodoAsyncQueuePolicy policy) {
    switch (policy) {
        case RbpodoAsyncQueuePolicy::LatestWins:
            return "latest_wins";
    }
    return "unknown";
}

std::string rbpodoAsyncSupervisionStateString(RbpodoAsyncStreamingSupervisionState state) {
    switch (state) {
        case RbpodoAsyncStreamingSupervisionState::Ok:
            return "ok";
        case RbpodoAsyncStreamingSupervisionState::Warning:
            return "warning";
        case RbpodoAsyncStreamingSupervisionState::Fault:
            return "fault";
    }
    return "unknown";
}

std::string rbpodoAsyncAcceptanceSemantics(
    const RbpodoAsyncStreamingConfig& config,
    const RbpodoAsyncStreamingTelemetry& telemetry
) {
    if (config.mode == RbpodoAsyncStreamingMode::SocketSendSupervised) {
        return "socket_send_only";
    }
    if (!telemetry.last_controller_acceptance_semantics.empty()) {
        return telemetry.last_controller_acceptance_semantics;
    }
    if (config.mode == RbpodoAsyncStreamingMode::SdkAckWorker) {
        return "controller_ack_observed";
    }
    return "disabled";
}

nlohmann::json rbpodoAsyncStreamingJson(
    const RbpodoAsyncStreamingConfig& config,
    const RbpodoAsyncStreamingTelemetry& telemetry
) {
    return {
        {"enabled", config.enable},
        {"mode", rbpodoAsyncStreamingModeString(config.mode)},
        {"queue_policy", rbpodoAsyncQueuePolicyString(config.queue_policy)},
        {"commands_enqueued_total", telemetry.commands_enqueued_total},
        {"commands_sent_total", telemetry.commands_sent_total},
        {"commands_acked_total", telemetry.commands_acked_total},
        {"commands_socket_sent_total", telemetry.commands_socket_sent_total},
        {"commands_dropped_total", telemetry.commands_dropped_total},
        {"commands_overwritten_total", telemetry.commands_overwritten_total},
        {"ack_timeout_count", telemetry.ack_timeout_count},
        {"missing_ack_count", telemetry.missing_ack_count},
        {"q_ref_watchdog_miss_count", telemetry.q_ref_watchdog_miss_count},
        {"tcp_ref_watchdog_miss_count", telemetry.tcp_ref_watchdog_miss_count},
        {"last_command_seq", telemetry.last_command_seq},
        {"last_sent_seq", telemetry.last_sent_seq},
        {"last_ack_seq", telemetry.last_ack_seq},
        {"first_goal_command_send_ns", telemetry.first_goal_command_send_ns},
        {"last_goal_command_send_ns", telemetry.last_goal_command_send_ns},
        {"first_worker_send_ns", telemetry.first_worker_send_ns},
        {"last_worker_send_ns", telemetry.last_worker_send_ns},
        {"goal_window_commands_sent", telemetry.goal_window_commands_sent},
        {"goal_window_commands_acked", telemetry.goal_window_commands_acked},
        {"last_q_ref_update_host_time_ns", telemetry.last_q_ref_update_host_time_ns},
        {"last_tcp_ref_update_host_time_ns", telemetry.last_tcp_ref_update_host_time_ns},
        {"last_socket_send_host_time_ns", telemetry.last_socket_send_host_time_ns},
        {"q_ref_update_age_ms", telemetry.q_ref_update_age_ms},
        {"tcp_ref_update_age_ms", telemetry.tcp_ref_update_age_ms},
        {"q_ref_target_error_deg_max", telemetry.q_ref_target_error_deg_max},
        {"tcp_ref_target_error_m", telemetry.tcp_ref_target_error_m},
        {"last_async_send_duration_us", telemetry.last_async_send_duration_us},
        {"last_async_ack_duration_us", telemetry.last_async_ack_duration_us},
        {"max_async_send_duration_us", telemetry.max_async_send_duration_us},
        {"max_async_ack_duration_us", telemetry.max_async_ack_duration_us},
        {"last_controller_acceptance_semantics",
            rbpodoAsyncAcceptanceSemantics(config, telemetry)},
        {"last_async_acceptance_semantics",
            telemetry.last_async_acceptance_semantics.empty()
                ? rbpodoAsyncAcceptanceSemantics(config, telemetry)
                : telemetry.last_async_acceptance_semantics},
        {"command_phase", telemetry.command_phase},
        {"last_send_result", telemetry.last_send_result},
        {"last_ack_result", telemetry.last_ack_result},
        {"last_failure", telemetry.last_failure},
        {"worker_backlog", telemetry.worker_backlog},
        {"async_worker_backlog", telemetry.worker_backlog},
        {"max_pending_age_ms_observed", telemetry.max_pending_age_ms_observed},
        {"supervision_state", rbpodoAsyncSupervisionStateString(telemetry.supervision_state)},
        {"async_supervision_state", rbpodoAsyncSupervisionStateString(telemetry.supervision_state)},
        {"reference_supervision_state",
            rbpodoAsyncSupervisionStateString(telemetry.reference_supervision_state)},
        {"reference_supervision_reason", telemetry.reference_supervision_reason},
        {"reference_supervision_fault_count", telemetry.reference_supervision_fault_count},
    };
}

nlohmann::json cartesianControlSnapshotJson(const CartesianControlConfig& config) {
    return {
        {"schema", "robotics_lab.cartesian_control_snapshot.v1"},
        {"enable", config.enable},
        {"allow_in_simulation", config.allow_in_simulation},
        {"allow_in_real", config.allow_in_real},
        {"allow_in_controller_simulation", config.allow_in_controller_simulation},
        {"warn_ik_duration_us", config.warn_ik_duration_us},
        {"fail_ik_duration_us", config.fail_ik_duration_us},
        {"path_kp", config.path_kp},
        {"path_kp_pos", config.path_kp_pos},
        {"path_kp_ori", config.path_kp_ori},
        {"velocity_damping", config.velocity_damping},
        {"max_linear_move_speed_m_s", config.max_linear_move_speed_m_s},
        {"max_angular_move_speed_rad_s", config.max_angular_move_speed_rad_s},
        {"max_cartesian_step_m", optionalDoubleJson(config.max_cartesian_step_m)},
        {"max_cartesian_step_rad", optionalDoubleJson(config.max_cartesian_step_rad)},
        {"exceed_limit_policy", cartesianLimitPolicyString(config.exceed_limit_policy)},
        {"controller_simulation_servo_state_source",
            controllerSimulationStateSourceString(config.controller_simulation_servo_state_source)},
        {"controller_simulation_divergence_source",
            controllerSimulationStateSourceString(config.controller_simulation_divergence_source)},
        {"linear_move", {
            {"min_duration_sec", config.linear_move.min_duration_sec},
            {"max_duration_sec", config.linear_move.max_duration_sec},
            {"default_linear_speed_m_s", config.linear_move.default_linear_speed_m_s},
            {"default_angular_speed_rad_s", config.linear_move.default_angular_speed_rad_s},
            {"constant_orientation_tolerance_rad", config.linear_move.constant_orientation_tolerance_rad},
            {"default_orientation_mode", orientationModeString(config.linear_move.default_orientation_mode)},
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

// Self-collision geometry manifest: the EXACT inputs the async CollisionMonitor
// builds its coal geometry from, so the GUI loads the same unified URDF (collision
// meshes), attaches the same Pika gripper hull, and adds the same extra primitives
// — instead of hardcoding a viewer-side URDF. Mirrors collision_monitor.cpp's
// geometry build (gripper hull at "<prefix>attachment_site" rotated +90° about Z,
// STL in mm so scale 0.001). Static (config-derived); the GUI reads it once.
nlohmann::json selfCollisionManifestJson(
    const SelfCollisionConfig& sc, const std::vector<std::string>& joint_names) {
    const auto& m = sc.mesh;
    nlohmann::json extra = nlohmann::json::array();
    for (const auto& e : m.extra_collision) {
        extra.push_back({
            {"name", e.name},
            {"shape", e.shape},
            {"parent_frame", e.parent_frame},
            {"size_m", e.size_m},
            {"radius_m", e.radius_m},
            {"length_m", e.length_m},
            {"xyz_m", e.xyz_m},
            {"rpy", e.rpy},
        });
    }
    return {
        {"schema", "robotics_lab.self_collision_manifest.v1"},
        {"unified_urdf", m.unified_urdf},
        {"package_dirs", stringArrayJson(m.package_dirs)},
        {"pika_gripper_mesh", m.pika_gripper_mesh},
        // Articulated gripper meshes (when set, the monitor checks a static base hull +
        // two movable finger hulls instead of the single hull above). The GUI checkgeom
        // overlay mirrors this and moves the fingers by the live jaw percent.
        {"pika_gripper_base_mesh", m.pika_gripper_base_mesh},
        {"pika_finger_left_mesh", m.pika_finger_left_mesh},
        {"pika_finger_right_mesh", m.pika_finger_right_mesh},
        {"gripper_finger_travel_m", m.gripper_finger_travel_m},
        {"left_prefix", m.left_prefix},
        {"right_prefix", m.right_prefix},
        {"stand_frame", m.stand_frame},
        // Base actuated-joint names (command order); the GUI forms the unified-URDF
        // joint names as "<left_prefix>+name" / "<right_prefix>+name", exactly as
        // collision_monitor.cpp builds left_joints/right_joints.
        {"joint_names", stringArrayJson(joint_names)},
        // How collision_monitor.cpp attaches the Pika hull (mirror exactly).
        {"gripper_attach", {
            {"frame_suffix", "attachment_site"},  // attaches at "<prefix>attachment_site"
            {"object_suffix", "pika_gripper"},    // coal object name "<prefix>pika_gripper"
            {"xyz_m", std::array<double, 3>{0.0, 0.0, 0.0}},
            {"rpy", std::array<double, 3>{0.0, 0.0, M_PI / 2.0}},  // +90° about Z
            {"mesh_scale", 0.001},                // STL in mm -> m
        }},
        {"extra_collision", std::move(extra)},
        // Clearance thresholds so the viewer colors near pairs consistently. The
        // external_* set governs arm<->floor (external) pairs; the GUI bands those
        // against it and draws them in a distinct color.
        {"d_hard_m", m.d_hard_m},
        {"d_slow_m", m.d_slow_m},
        {"external_d_hard_m", m.external.d_hard_m},
        {"external_d_slow_m", m.external.d_slow_m},
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
        {"unavailable_fields", stringArrayJson(diagnostics->unavailable_fields)},
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
    const SafetyClampTelemetry& clamp = telemetry.safety_clamp;
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
        {"ik_min_singular_value", telemetry.ik_min_singular_value},
        {"ik_applied_damping", telemetry.ik_applied_damping},
        {"ik_solution_jump_deg", telemetry.ik_solution_jump_deg},
        {"ik_branch_jump_suspected", telemetry.ik_branch_jump_suspected},
        {"ik_branch_jump_clamped", telemetry.ik_branch_jump_clamped},
        {"ik_branch_jump_rate_limited", telemetry.ik_branch_jump_rate_limited},
        {"ik_branch_jump_raw_deg", telemetry.ik_branch_jump_raw_deg},
        {"ik_branch_jump_limit_deg", telemetry.ik_branch_jump_limit_deg},
        {"ik_branch_jump_scale", telemetry.ik_branch_jump_scale},
        {"ik_branch_jump_retry_count", telemetry.ik_branch_jump_retry_count},
        {"q_ik_seed_deg", jointArrayJson(telemetry.q_ik_seed_deg)},
        {"q_ik_raw_solution_deg", jointArrayJson(telemetry.q_ik_raw_solution_deg)},
        {"q_ik_solution_deg", jointArrayJson(telemetry.q_ik_solution_deg)},
        {"q_ik_raw_delta_deg", jointArrayJson(telemetry.q_ik_raw_delta_deg)},
        {"q_ik_delta_deg", jointArrayJson(telemetry.q_ik_delta_deg)},
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
        {"floor_vz_clamped", telemetry.floor_vz_clamped},
        {"floor_lowest_point", telemetry.floor_lowest_point},
        {"floor_lowest_z_m", telemetry.floor_lowest_z_m},
        {"floor_goal_clamped", telemetry.floor_goal_clamped},
        {"goal_minus_measured_pos_m", telemetry.goal_minus_measured_pos_m},
        {"goal_minus_measured_ori_rad", telemetry.goal_minus_measured_ori_rad},
        {"cartesian_servo_state_source", telemetry.cartesian_servo_state_source},
        {"cartesian_divergence_source", telemetry.cartesian_divergence_source},
        {"q_reference_for_servo_valid", telemetry.q_reference_for_servo_valid},
        // A/B/C separation telemetry (Patch 4). Absent/false on non-SMD paths.
        {"smd_active", telemetry.smd_active},
        {"tcp_target_profile", telemetry.tcp_target_profile},
        {"tcp_target_profile_found", telemetry.tcp_target_profile_found},
        {"smd_profile_nf_linear_hz", telemetry.smd_profile_nf_linear_hz},
        {"smd_profile_nf_angular_hz", telemetry.smd_profile_nf_angular_hz},
        {"smd_profile_velocity_feedforward", telemetry.smd_profile_velocity_feedforward},
        {"smd_profile_max_linear_velocity_m_s", telemetry.smd_profile_max_linear_velocity_m_s},
        {"smd_profile_max_linear_accel_m_s2", telemetry.smd_profile_max_linear_accel_m_s2},
        {"smd_profile_max_angular_velocity_rad_s", telemetry.smd_profile_max_angular_velocity_rad_s},
        {"smd_profile_max_angular_accel_rad_s2", telemetry.smd_profile_max_angular_accel_rad_s2},
        {"smd_profile_max_goal_lead_m", telemetry.smd_profile_max_goal_lead_m},
        {"smd_profile_max_goal_lead_rad", telemetry.smd_profile_max_goal_lead_rad},
        {"smd_goal_stand", optionalPoseJson(telemetry.smd_goal_stand)},
        {"smd_ref_stand", optionalPoseJson(telemetry.smd_ref_stand)},
        {"smd_velocity_feedforward_used", telemetry.smd_velocity_feedforward_used},
        {"smd_linear_velocity_clipped", telemetry.smd_linear_velocity_clipped},
        {"smd_linear_accel_clipped", telemetry.smd_linear_accel_clipped},
        {"smd_angular_velocity_clipped", telemetry.smd_angular_velocity_clipped},
        {"smd_angular_accel_clipped", telemetry.smd_angular_accel_clipped},
        {"smd_goal_linear_velocity_ff_clipped", telemetry.smd_goal_linear_velocity_ff_clipped},
        {"smd_goal_angular_velocity_ff_clipped", telemetry.smd_goal_angular_velocity_ff_clipped},
        {"smd_goal_linear_velocity_norm_m_s", telemetry.smd_goal_linear_velocity_norm_m_s},
        {"smd_goal_angular_velocity_norm_rad_s", telemetry.smd_goal_angular_velocity_norm_rad_s},
        {"smd_reanchor_count", telemetry.smd_reanchor_count},
        {"safety_intervention_recent", telemetry.safety_intervention_recent},
        {"delta_twist_safety_intervention_recent", telemetry.delta_twist_safety_intervention_recent},
        {"delta_twist_block_on_safety_intervention_recent",
         telemetry.delta_twist_block_on_safety_intervention_recent},
        {"delta_twist_soft_intervention_ignored_for_block",
         telemetry.delta_twist_soft_intervention_ignored_for_block},
        {"surface_min_tip_dist_m", finiteDoubleJson(telemetry.surface_min_tip_dist_m)},
        {"surface_active", telemetry.surface_active},
        {"surface_close_soon", telemetry.surface_close_soon},
        {"surface_mode", telemetry.surface_mode},
        {"grasp_close_soon", telemetry.grasp_close_soon},
        {"grasp_close_soon_source", telemetry.grasp_close_soon_source},
        {"grasp_close_soon_steps_ahead", telemetry.grasp_close_soon_steps_ahead},
        {"grasp_phase", telemetry.grasp_phase},
        {"grasp_dwell_active", telemetry.grasp_dwell_active},
        {"grasp_dwell_elapsed_sec", finiteDoubleJson(telemetry.grasp_dwell_elapsed_sec)},
        {"grasp_dwell_fallback_triggered", telemetry.grasp_dwell_fallback_triggered},
        {"grasp_dwell_reason", telemetry.grasp_dwell_reason},
        {"gripper_policy_cmd", optionalDoubleJson(telemetry.gripper_policy_cmd)},
        {"gripper_effective_cmd", optionalDoubleJson(telemetry.gripper_effective_cmd)},
        {"gripper_close_soon", telemetry.gripper_close_soon},
        {"grip_horizon_available", telemetry.grip_horizon_available},
        {"grip_horizon_len", telemetry.grip_horizon_len},
        {"grip_horizon_current_index", telemetry.grip_horizon_current_index},
        {"grip_horizon_min", finiteDoubleJson(telemetry.grip_horizon_min)},
        {"grip_horizon_max", finiteDoubleJson(telemetry.grip_horizon_max)},
        {"grip_horizon_argmin", telemetry.grip_horizon_argmin},
        {"grip_horizon_argmax", telemetry.grip_horizon_argmax},
        {"grip_horizon_first_close_index", telemetry.grip_horizon_first_close_index},
        {"grip_horizon_first_close_steps_ahead", telemetry.grip_horizon_first_close_steps_ahead},
        {"grip_horizon_close_threshold", finiteDoubleJson(telemetry.grip_horizon_close_threshold)},
        {"grip_horizon_close_is_greater", telemetry.grip_horizon_close_is_greater},
        {"output_ma_present", telemetry.output_ma_present},
        {"output_ma_window", telemetry.output_ma_window},
        {"output_ma_active", telemetry.output_ma_present && telemetry.output_ma_window > 1},
        {"q_target_before_output_ma_deg",
         telemetry.output_ma_present ? jointArrayJson(telemetry.q_target_before_output_ma_deg)
                                     : nlohmann::json(nullptr)},
        {"q_target_after_output_ma_deg",
         telemetry.output_ma_present ? jointArrayJson(telemetry.q_target_after_output_ma_deg)
                                     : nlohmann::json(nullptr)},
        {"safety_clamp_present", clamp.present},
        {"q_before_safety_deg", clamp.present ? jointArrayJson(clamp.q_before_safety_deg)
                                             : nlohmann::json(nullptr)},
        {"q_after_joint_limit_deg", clamp.present ? jointArrayJson(clamp.q_after_joint_limit_deg)
                                                  : nlohmann::json(nullptr)},
        {"q_after_velocity_limit_deg", clamp.present ? jointArrayJson(clamp.q_after_velocity_limit_deg)
                                                     : nlohmann::json(nullptr)},
        {"q_after_accel_limit_deg", clamp.present ? jointArrayJson(clamp.q_after_accel_limit_deg)
                                                  : nlohmann::json(nullptr)},
        {"safety_joint_limit_clamped", clamp.joint_limit_clamped},
        {"safety_velocity_clamped", clamp.velocity_clamped},
        {"safety_accel_clamped", clamp.accel_clamped},
        {"safety_joint_limit_clamp_max_delta_deg", clamp.joint_limit_clamp_max_delta_deg},
        {"safety_velocity_clamp_max_delta_deg", clamp.velocity_clamp_max_delta_deg},
        {"safety_accel_clamp_max_delta_deg", clamp.accel_clamp_max_delta_deg},
        {"safety_joint_limit_limited_joint", clamp.joint_limit_limited_joint},
        {"safety_velocity_limited_joint", clamp.velocity_limited_joint},
        {"safety_accel_limited_joint", clamp.accel_limited_joint},
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

std::string lowerAscii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

struct TcpPublication {
    std::optional<Pose6D> actual_base;
    std::optional<Pose6D> actual_stand;
    std::optional<Pose6D> ref_base;
    std::optional<Pose6D> ref_stand;
    std::optional<Pose6D> command_stand;
    bool actual_valid = false;
    bool ref_valid = false;
};

TcpPublication tcpPublicationForState(const RobotState& state) {
    TcpPublication out;
    out.actual_base = state.tcp_actual_base.has_value() ? state.tcp_actual_base : state.tcp_base;
    out.actual_stand = state.tcp_actual_stand.has_value() ? state.tcp_actual_stand : state.tcp_stand;
    out.ref_base = state.tcp_ref_base;
    out.ref_stand = state.tcp_ref_stand;
    out.command_stand = state.tcp_command_stand;
    out.actual_valid =
        (state.tcp_actual_valid || state.has_valid_tcp_pose) &&
        out.actual_base.has_value() &&
        out.actual_stand.has_value();
    out.ref_valid =
        state.tcp_ref_valid &&
        out.ref_base.has_value() &&
        out.ref_stand.has_value();
    return out;
}

bool isRbpodoControllerSimulation(const BackendConfig& backend_config) {
    if (backend_config.backend_type != BackendType::Rbpodo) return false;
    const std::string operation_mode = lowerAscii(backend_config.operation_mode);
    return operation_mode == "simulation" || operation_mode == "sim";
}


bool isStreamingCartesianMode(ControlMode mode) {
    return mode == ControlMode::TcpLinearMove;
}

bool isCartesianMode(ControlMode mode) {
    return mode == ControlMode::TcpPoseTarget ||
           mode == ControlMode::TcpLinearMove;
}

bool controllerSimulationCartesianGateOpen(
    const CartesianControlConfig& cartesian_config,
    const ServoConfig& servo_config,
    const BackendConfig& backend_config
) {
    return cartesian_config.enable &&
        cartesian_config.allow_in_controller_simulation &&
        servo_config.allow_controller_simulation_motion &&
        backend_config.backend_type == BackendType::Rbpodo &&
        backend_config.run_mode == RunMode::Real &&
        isRbpodoControllerSimulation(backend_config);
}

std::string cartesianGateUnavailableReason(
    const CartesianControlConfig& cartesian_config,
    const ServoConfig& servo_config,
    const BackendConfig& backend_config,
    ControlMode command_mode
) {
    // Mirror of cartesianAvailabilityForArm: real/sim execution gating is
    // retired — only the feature flags below can make Cartesian unavailable.
    (void)servo_config;
    (void)backend_config;
    if (!cartesian_config.enable) {
        return "cartesian_control_unavailable_disabled";
    }
    return "";
}

nlohmann::json cartesianGateJson(
    const CartesianControlConfig& cartesian_config,
    const ServoConfig& servo_config,
    const SafetyConfig& safety_config,
    const BackendConfig& backend_config,
    ControlMode command_mode,
    const CartesianSolveTelemetry& cartesian_solve
) {
    std::string unavailable_reason = cartesianGateUnavailableReason(
        cartesian_config,
        servo_config,
        backend_config,
        command_mode
    );
    if (!cartesian_solve.reason.empty() &&
        cartesian_solve.reason.rfind("cartesian_control_unavailable", 0) == 0) {
        unavailable_reason = cartesian_solve.reason;
    }
    const bool controller_sim_cartesian_enabled =
        unavailable_reason.empty() &&
        isCartesianMode(command_mode) &&
        controllerSimulationCartesianGateOpen(cartesian_config, servo_config, backend_config);
    const std::string streaming_unavailable_reason = cartesianGateUnavailableReason(
        cartesian_config,
        servo_config,
        backend_config,
        ControlMode::TcpLinearMove
    );
    const bool controller_sim_streaming_cartesian_available =
        streaming_unavailable_reason.empty() &&
        controllerSimulationCartesianGateOpen(cartesian_config, servo_config, backend_config);
    const bool physical_motion_expected = isRbpodoControllerSimulation(backend_config)
        ? false
        : backend_config.run_mode == RunMode::Real;
    // True when physical-real Cartesian motion is open. Stays false in
    // controller-simulation operation.
    const bool streaming_cartesian_physical_real_enabled =
        backend_config.backend_type == BackendType::Rbpodo &&
        backend_config.run_mode == RunMode::Real &&
        !isRbpodoControllerSimulation(backend_config) &&
        cartesian_config.allow_in_real;
    return {
        {"run_mode", runModeString(backend_config.run_mode)},
        {"backend_type", backendTypeString(backend_config.backend_type)},
        {"operation_mode", backend_config.operation_mode},
        {"allow_in_simulation", cartesian_config.allow_in_simulation},
        {"allow_in_real", cartesian_config.allow_in_real},
        {"allow_in_controller_simulation", cartesian_config.allow_in_controller_simulation},
        {"allow_controller_simulation_motion", servo_config.allow_controller_simulation_motion},
        {"controller_simulation_servo_state_source",
            controllerSimulationStateSourceString(cartesian_config.controller_simulation_servo_state_source)},
        {"controller_simulation_divergence_source",
            controllerSimulationStateSourceString(cartesian_config.controller_simulation_divergence_source)},
        {"controller_simulation_tracking_error_source",
            controllerSimulationTrackingSourceString(safety_config.controller_simulation_tracking_error_source)},
        {"controller_simulation_physical_motion_policy",
            controllerSimulationPhysicalMotionPolicyString(safety_config.controller_simulation_physical_motion_policy)},
        {"controller_simulation_physical_motion_threshold_deg",
            safety_config.controller_simulation_physical_motion_threshold_deg},
        {"physical_motion_expected", physical_motion_expected},
        {"controller_simulation_cartesian_enabled", controller_sim_cartesian_enabled},
        {"controller_simulation_cartesian_enabled_for_current_command", controller_sim_cartesian_enabled},
        {"controller_simulation_streaming_cartesian_available",
            controller_sim_streaming_cartesian_available},
        {"controller_simulation_streaming_cartesian_unavailable_reason",
            streaming_unavailable_reason.empty()
                ? nlohmann::json(nullptr)
                : nlohmann::json(streaming_unavailable_reason)},
        {"streaming_cartesian_physical_real_enabled", streaming_cartesian_physical_real_enabled},
        {"current_command_is_cartesian", isCartesianMode(command_mode)},
        {"current_command_is_streaming_cartesian", isStreamingCartesianMode(command_mode)},
        {"cartesian_available", unavailable_reason.empty()},
        {"cartesian_unavailable_reason", unavailable_reason.empty()
            ? nlohmann::json(nullptr)
            : nlohmann::json(unavailable_reason)},
    };
}

bool controllerSimulationDiagnosticOverrideActive(
    const ServoConfig& servo_config,
    const BackendConfig& backend_config,
    const ArmStartupValidationSnapshot& startup_validation
) {
    const bool diagnostics_suspect_active =
        servo_config.allow_controller_simulation_diagnostics_suspect &&
        startup_validation.diagnostic_error_source == "rbpodo_diagnostics_suspect";
    const bool init_error_active =
        servo_config.allow_controller_simulation_init_error &&
        startup_validation.diagnostic_error_source == "rbpodo_init_error";
    return servo_config.send_servo_commands &&
        servo_config.allow_controller_simulation_motion &&
        isRbpodoControllerSimulation(backend_config) &&
        startup_validation.allowed_unsafe_startup &&
        (diagnostics_suspect_active || init_error_active);
}

std::string tcpTrackingSource(
    const TcpPublication& tcp,
    const BackendConfig& backend_config
) {
    if (isRbpodoControllerSimulation(backend_config) && tcp.ref_valid) {
        return "tcp_ref_stand";
    }
    if (tcp.actual_valid) {
        return "tcp_actual_stand";
    }
    if (tcp.ref_valid) {
        return "tcp_ref_stand";
    }
    return "none";
}

std::string tcpTrackingSourceRecommendation(
    const TcpPublication& tcp,
    const BackendConfig& backend_config
) {
    if (isRbpodoControllerSimulation(backend_config)) {
        if (tcp.ref_valid) {
            return "reference_for_controller_simulation";
        }
        return tcp.actual_valid
            ? "actual_fallback_reference_unavailable"
            : "unavailable";
    }
    return tcp.actual_valid ? "actual" : "unavailable";
}

nlohmann::json controllerSimulationModeJson(
    const TcpPublication& tcp,
    const BackendConfig& backend_config,
    bool diagnostic_override_active
) {
    if (!isRbpodoControllerSimulation(backend_config)) return nullptr;
    const bool use_ref = tcp.ref_valid;
    const std::string recommended_pose = use_ref
        ? "tcp_ref_stand"
        : (tcp.actual_valid ? "tcp_actual_stand" : "none");
    return {
        {"operation_mode", backend_config.operation_mode},
        {"recommended_tracking_pose", recommended_pose},
        {"physical_motion_expected", false},
        {"controller_simulation_diagnostic_override_active", diagnostic_override_active},
        {"tcp_ref_valid", tcp.ref_valid},
        {"reason", use_ref
            ? "reference_for_controller_simulation"
            : "reference_unavailable_actual_fallback"},
    };
}

nlohmann::json gripperFeedbackJson(const GripperArmFeedback& fb) {
    // Stamped from the latest gripper_state.v1 feedback the bridge cached. When no
    // fresh feedback exists (disabled / never received / stale) -> {valid:false}.
    if (!fb.valid) {
        return nlohmann::json{{"valid", false}, {"stale", true}, {"ok", false}};
    }
    return nlohmann::json{
        {"valid", true},
        {"stale", false},
        {"percent", fb.percent},
        {"target_percent", fb.target_percent},
        {"moving", fb.moving},
        {"ok", fb.ok},
        {"fault", fb.fault.empty() ? nlohmann::json(nullptr) : nlohmann::json(fb.fault)},
    };
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
    const RbpodoAsyncStreamingTelemetry& async_streaming_telemetry,
    const std::optional<BackendTransportTelemetry>& transport_telemetry,
    bool worker_enabled,
    const ServoConfig& servo_config,
    const BackendConfig& backend_config,
    const CartesianControlConfig& cartesian_config,
    const SafetyConfig& safety_config,
    const CartesianSolveTelemetry& cartesian_solve,
    const SafetyTrackingTelemetry& safety_tracking,
    const ArmStartupValidationSnapshot& startup_validation,
    const GripperArmFeedback& gripper_feedback
) {
    const TcpPublication tcp = tcpPublicationForState(state);
    const bool diagnostic_override_active = controllerSimulationDiagnosticOverrideActive(
        servo_config,
        backend_config,
        startup_validation
    );
    const nlohmann::json cartesian_gate =
        cartesianGateJson(
            cartesian_config,
            servo_config,
            safety_config,
            backend_config,
            command.mode,
            cartesian_solve
    );
    return {
        {"mode", toString(command.mode)},
        {"gripper", gripperFeedbackJson(gripper_feedback)},
        {"q_actual_deg", jointArrayJson(state.q_actual_deg)},
        {"q_target_deg", jointArrayJson(state.q_target_deg)},
        {"q_ref_deg", jointArrayJson(state.q_target_deg)},
        {"q_actual_valid", qActualValidForPublication(state)},
        {"q_ref_valid", qRefValidForPublication(state)},
        {"q_ref_source", optionalStringJson(state.q_ref_source)},
        {"rbpodo_sdk_state_source", optionalStringJson(state.rbpodo_sdk_state_source)},
        {"rbpodo_state_decode_policy", optionalStringJson(state.rbpodo_state_decode_policy)},
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
        {"async_streaming", rbpodoAsyncStreamingJson(
            servo_config.rbpodo_async_streaming,
            async_streaming_telemetry
        )},
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
        {"tcp_stand", optionalPoseJson(tcp.actual_stand)},
        {"tcp_base", optionalPoseJson(tcp.actual_base)},
        {"tcp_actual_stand", optionalPoseJson(tcp.actual_stand)},
        {"tcp_actual_base", optionalPoseJson(tcp.actual_base)},
        {"tcp_ref_stand", optionalPoseJson(tcp.ref_stand)},
        {"tcp_ref_base", optionalPoseJson(tcp.ref_base)},
        {"tcp_command_stand", optionalPoseJson(tcp.command_stand)},
        {"has_valid_tcp_pose", tcp.actual_valid},
        {"tcp_actual_valid", tcp.actual_valid},
        {"tcp_ref_valid", tcp.ref_valid},
        {"tcp_tracking_source", tcpTrackingSource(tcp, backend_config)},
        {"tcp_tracking_source_recommendation", tcpTrackingSourceRecommendation(tcp, backend_config)},
        {"tracking_error_source", safety_tracking.tracking_error_source},
        {"tracking_error_source_valid", safety_tracking.tracking_error_source_valid},
        {"tracking_error_reason", optionalStringJson(safety_tracking.tracking_error_reason)},
        {"command_reference_tracking_error_deg",
            safety_tracking.command_reference_tracking_error_deg},
        {"physical_command_actual_error_deg",
            safety_tracking.physical_command_actual_error_deg},
        {"controller_simulation_physical_motion_detected",
            safety_tracking.controller_simulation_physical_motion_detected},
        {"controller_simulation_diagnostic_override_active", diagnostic_override_active},
        {"cartesian_available", cartesian_gate.at("cartesian_available")},
        {"cartesian_unavailable_reason", cartesian_gate.at("cartesian_unavailable_reason")},
        {"cartesian_gate", cartesian_gate},
        {"controller_simulation_cartesian_enabled",
            cartesian_gate.at("controller_simulation_cartesian_enabled")},
        {"controller_simulation_cartesian_enabled_for_current_command",
            cartesian_gate.at("controller_simulation_cartesian_enabled_for_current_command")},
        {"controller_simulation_streaming_cartesian_available",
            cartesian_gate.at("controller_simulation_streaming_cartesian_available")},
        {"streaming_cartesian_physical_real_enabled",
            cartesian_gate.at("streaming_cartesian_physical_real_enabled")},
        {"physical_motion_expected", isRbpodoControllerSimulation(backend_config)
            ? nlohmann::json(false)
            : (backend_config.run_mode == RunMode::Real
                ? nlohmann::json(true)
                : nlohmann::json(nullptr))},
        {"controller_simulation_mode", controllerSimulationModeJson(
            tcp,
            backend_config,
            diagnostic_override_active
        )},
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
    std::vector<UdpDestination> destinations;
    destinations.reserve(config.state_pub_endpoints.size());
    for (const std::string& endpoint : config.state_pub_endpoints) {
        UdpDestination destination;
        if (!resolveUdpEndpoint(endpoint, &destination)) {
            continue;
        }
        destinations.push_back(std::move(destination));
    }
    return destinations;
}

bool validateUdpEndpointSyntax(const NetworkConfig& config) {
    if (config.state_pub_endpoints.empty()) {
        std::cerr << "[ERROR] StatePublisher requires at least one state_pub_endpoints entry\n";
        return false;
    }
    for (const std::string& endpoint : config.state_pub_endpoints) {
        if (!StatePublisher::parseUdpEndpointUri(endpoint, nullptr, nullptr)) {
            std::cerr << "[ERROR] StatePublisher only supports udp://host:port endpoints, got "
                      << endpoint << "\n";
            return false;
        }
    }
    return true;
}

std::vector<std::string> uniqueEndpoints(const std::vector<std::string>& endpoints) {
    std::vector<std::string> unique;
    unique.reserve(endpoints.size());
    std::set<std::string> seen;
    for (const std::string& endpoint : endpoints) {
        if (seen.insert(endpoint).second) {
            unique.push_back(endpoint);
        } else {
            std::cerr << "[WARN] duplicate StatePublisher endpoint ignored: "
                      << endpoint << "\n";
        }
    }
    return unique;
}

void normalizeStatePublisherNetworkConfig(NetworkConfig* config) {
    if (!config) return;
    const NetworkConfig defaults;
    const bool endpoint_list_is_default =
        config->state_pub_endpoints.size() == 1 &&
        config->state_pub_endpoints.front() == defaults.state_pub_endpoint;

    if (config->state_pub_endpoints.empty()) {
        const std::string fallback =
            config->state_pub_endpoint.empty() ? config->state_pub_bind : config->state_pub_endpoint;
        config->state_pub_endpoints = {fallback};
    } else if (endpoint_list_is_default && config->state_pub_endpoint != defaults.state_pub_endpoint) {
        config->state_pub_endpoints = {config->state_pub_endpoint};
    } else if (
        endpoint_list_is_default &&
        config->state_pub_endpoint == defaults.state_pub_endpoint &&
        config->state_pub_bind != defaults.state_pub_bind
    ) {
        config->state_pub_endpoints = {config->state_pub_bind};
    } else {
        config->state_pub_endpoints = uniqueEndpoints(config->state_pub_endpoints);
    }

    if (config->state_pub_endpoints.empty()) {
        config->state_pub_endpoints = {defaults.state_pub_endpoint};
        return;
    }
    config->state_pub_endpoint = config->state_pub_endpoints.front();
    config->state_pub_bind = config->state_pub_endpoint;
}

}  // namespace

StatePublisher::StatePublisher(const DualArmConfig& config, SnapshotProvider provider,
                               GripperFeedbackSink gripper_sink)
    : config_(config),
      snapshot_provider_(std::move(provider)),
      gripper_feedback_sink_(std::move(gripper_sink)) {
    normalizeStatePublisherNetworkConfig(&config_.network);
    if (config_.gripper.enable) {
        gripper_bridge_ = std::make_unique<GripperBridge>(config_.gripper);
    }
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
    message["async_streaming_enabled"] = config_.servo.rbpodo_async_streaming.enable;
    message["async_streaming_mode"] =
        rbpodoAsyncStreamingModeString(config_.servo.rbpodo_async_streaming.mode);
    message["async_streaming_policy"] =
        rbpodoAsyncQueuePolicyString(config_.servo.rbpodo_async_streaming.queue_policy);
    message["cartesian_control_snapshot"] = cartesianControlSnapshotJson(config_.cartesian_control);
    message["kinematics_snapshot"] = kinematicsSnapshotJson(config_.kinematics);
    message["startup_validation"] = startupValidationJson(snapshot.startup_validation);
    const bool worker_enabled =
        config_.servo.io_model == ServoIoModel::Worker ||
        config_.servo.rbpodo_async_streaming.enable;

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
        snapshot.left_async_streaming,
        snapshot.left_transport_telemetry,
        worker_enabled,
        config_.servo,
        config_.left_robot,
        config_.cartesian_control,
        config_.safety,
        snapshot.left_cartesian_solve,
        snapshot.left_safety_tracking,
        snapshot.startup_validation.left,
        gripper_bridge_ ? gripper_bridge_->latest(ArmId::Left) : GripperArmFeedback{}
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
        snapshot.right_async_streaming,
        snapshot.right_transport_telemetry,
        worker_enabled,
        config_.servo,
        config_.right_robot,
        config_.cartesian_control,
        config_.safety,
        snapshot.right_cartesian_solve,
        snapshot.right_safety_tracking,
        snapshot.startup_validation.right,
        gripper_bridge_ ? gripper_bridge_->latest(ArmId::Right) : GripperArmFeedback{}
    );
    message["last_cartesian_solve"] = {
        {"left", cartesianSolveJson(snapshot.left_cartesian_solve)},
        {"right", cartesianSolveJson(snapshot.right_cartesian_solve)},
    };
    message["init_motion"] = {
        {"status", snapshot.init_motion.status},
        {"fail_mode", snapshot.init_motion.fail_mode},
        {"message", snapshot.init_motion.message},
        {"start_clear_m", finiteDoubleJson(snapshot.init_motion.start_clear_m)},
        {"goal_clear_m", finiteDoubleJson(snapshot.init_motion.goal_clear_m)},
        {"goal_self_min_clearance_m", finiteDoubleJson(snapshot.init_motion.goal_self_min_clearance_m)},
        {"goal_external_min_clearance_m", finiteDoubleJson(snapshot.init_motion.goal_external_min_clearance_m)},
        {"goal_nearest_pair_name_a", snapshot.init_motion.goal_nearest_pair_name_a},
        {"goal_nearest_pair_name_b", snapshot.init_motion.goal_nearest_pair_name_b},
        {"goal_nearest_pair_category", snapshot.init_motion.goal_nearest_pair_category},
        {"goal_nearest_pair_external", snapshot.init_motion.goal_nearest_pair_external},
        {"goal_nearest_pair_disabled_by_rule", snapshot.init_motion.goal_nearest_pair_disabled_by_rule},
        {"goal_nearest_pair_distance_m", finiteDoubleJson(snapshot.init_motion.goal_nearest_pair_distance_m)},
        {"goal_clear_threshold_self_m", finiteDoubleJson(snapshot.init_motion.goal_clear_threshold_self_m)},
        {"goal_clear_threshold_external_m", finiteDoubleJson(snapshot.init_motion.goal_clear_threshold_external_m)},
        {"goal_clear_margin_deficit_m", finiteDoubleJson(snapshot.init_motion.goal_clear_margin_deficit_m)},
        {"tree_start", snapshot.init_motion.tree_start},
        {"tree_goal", snapshot.init_motion.tree_goal},
        {"iterations", snapshot.init_motion.iterations},
        {"planning_time_s", finiteDoubleJson(snapshot.init_motion.planning_time_s)},
        {"waypoint_index", snapshot.init_motion.waypoint_index},
        {"waypoint_count", snapshot.init_motion.waypoint_count},
        {"dist_to_goal_deg", finiteDoubleJson(snapshot.init_motion.dist_to_goal_deg)},
        {"clear_threshold_m", finiteDoubleJson(snapshot.init_motion.clear_threshold_m)},
        {"external_clear_threshold_m", finiteDoubleJson(snapshot.init_motion.external_clear_threshold_m)},
        {"nearest_pair", snapshot.init_motion.nearest_pair},
        {"nearest_pair_distance_m", finiteDoubleJson(snapshot.init_motion.nearest_pair_distance_m)},
        {"nearest_pair_external", snapshot.init_motion.nearest_pair_external},
        {"left", {
            {"status", snapshot.init_motion_left.status},
            {"fail_mode", snapshot.init_motion_left.fail_mode},
            {"message", snapshot.init_motion_left.message},
            {"goal_self_min_clearance_m", finiteDoubleJson(snapshot.init_motion_left.goal_self_min_clearance_m)},
            {"goal_external_min_clearance_m", finiteDoubleJson(snapshot.init_motion_left.goal_external_min_clearance_m)},
            {"goal_nearest_pair_name_a", snapshot.init_motion_left.goal_nearest_pair_name_a},
            {"goal_nearest_pair_name_b", snapshot.init_motion_left.goal_nearest_pair_name_b},
            {"goal_nearest_pair_category", snapshot.init_motion_left.goal_nearest_pair_category},
            {"goal_nearest_pair_external", snapshot.init_motion_left.goal_nearest_pair_external},
            {"goal_nearest_pair_disabled_by_rule", snapshot.init_motion_left.goal_nearest_pair_disabled_by_rule},
            {"goal_nearest_pair_distance_m", finiteDoubleJson(snapshot.init_motion_left.goal_nearest_pair_distance_m)},
            {"goal_clear_threshold_self_m", finiteDoubleJson(snapshot.init_motion_left.goal_clear_threshold_self_m)},
            {"goal_clear_threshold_external_m", finiteDoubleJson(snapshot.init_motion_left.goal_clear_threshold_external_m)},
            {"goal_clear_margin_deficit_m", finiteDoubleJson(snapshot.init_motion_left.goal_clear_margin_deficit_m)},
            {"waypoint_index", snapshot.init_motion_left.waypoint_index},
            {"waypoint_count", snapshot.init_motion_left.waypoint_count},
            {"dist_to_goal_deg", finiteDoubleJson(snapshot.init_motion_left.dist_to_goal_deg)},
            {"clear_threshold_m", finiteDoubleJson(snapshot.init_motion_left.clear_threshold_m)},
            {"external_clear_threshold_m", finiteDoubleJson(snapshot.init_motion_left.external_clear_threshold_m)},
            {"nearest_pair", snapshot.init_motion_left.nearest_pair},
            {"nearest_pair_distance_m", finiteDoubleJson(snapshot.init_motion_left.nearest_pair_distance_m)},
            {"nearest_pair_external", snapshot.init_motion_left.nearest_pair_external},
        }},
        {"right", {
            {"status", snapshot.init_motion_right.status},
            {"fail_mode", snapshot.init_motion_right.fail_mode},
            {"message", snapshot.init_motion_right.message},
            {"goal_self_min_clearance_m", finiteDoubleJson(snapshot.init_motion_right.goal_self_min_clearance_m)},
            {"goal_external_min_clearance_m", finiteDoubleJson(snapshot.init_motion_right.goal_external_min_clearance_m)},
            {"goal_nearest_pair_name_a", snapshot.init_motion_right.goal_nearest_pair_name_a},
            {"goal_nearest_pair_name_b", snapshot.init_motion_right.goal_nearest_pair_name_b},
            {"goal_nearest_pair_category", snapshot.init_motion_right.goal_nearest_pair_category},
            {"goal_nearest_pair_external", snapshot.init_motion_right.goal_nearest_pair_external},
            {"goal_nearest_pair_disabled_by_rule", snapshot.init_motion_right.goal_nearest_pair_disabled_by_rule},
            {"goal_nearest_pair_distance_m", finiteDoubleJson(snapshot.init_motion_right.goal_nearest_pair_distance_m)},
            {"goal_clear_threshold_self_m", finiteDoubleJson(snapshot.init_motion_right.goal_clear_threshold_self_m)},
            {"goal_clear_threshold_external_m", finiteDoubleJson(snapshot.init_motion_right.goal_clear_threshold_external_m)},
            {"goal_clear_margin_deficit_m", finiteDoubleJson(snapshot.init_motion_right.goal_clear_margin_deficit_m)},
            {"waypoint_index", snapshot.init_motion_right.waypoint_index},
            {"waypoint_count", snapshot.init_motion_right.waypoint_count},
            {"dist_to_goal_deg", finiteDoubleJson(snapshot.init_motion_right.dist_to_goal_deg)},
            {"clear_threshold_m", finiteDoubleJson(snapshot.init_motion_right.clear_threshold_m)},
            {"external_clear_threshold_m", finiteDoubleJson(snapshot.init_motion_right.external_clear_threshold_m)},
            {"nearest_pair", snapshot.init_motion_right.nearest_pair},
            {"nearest_pair_distance_m", finiteDoubleJson(snapshot.init_motion_right.nearest_pair_distance_m)},
            {"nearest_pair_external", snapshot.init_motion_right.nearest_pair_external},
        }},
        {"aggregate", {
            {"status", snapshot.init_motion.status},
            {"fail_mode", snapshot.init_motion.fail_mode},
            {"message", snapshot.init_motion.message},
            {"goal_self_min_clearance_m", finiteDoubleJson(snapshot.init_motion.goal_self_min_clearance_m)},
            {"goal_external_min_clearance_m", finiteDoubleJson(snapshot.init_motion.goal_external_min_clearance_m)},
            {"goal_nearest_pair_name_a", snapshot.init_motion.goal_nearest_pair_name_a},
            {"goal_nearest_pair_name_b", snapshot.init_motion.goal_nearest_pair_name_b},
            {"goal_nearest_pair_category", snapshot.init_motion.goal_nearest_pair_category},
            {"goal_nearest_pair_external", snapshot.init_motion.goal_nearest_pair_external},
            {"goal_nearest_pair_disabled_by_rule", snapshot.init_motion.goal_nearest_pair_disabled_by_rule},
            {"goal_nearest_pair_distance_m", finiteDoubleJson(snapshot.init_motion.goal_nearest_pair_distance_m)},
            {"goal_clear_threshold_self_m", finiteDoubleJson(snapshot.init_motion.goal_clear_threshold_self_m)},
            {"goal_clear_threshold_external_m", finiteDoubleJson(snapshot.init_motion.goal_clear_threshold_external_m)},
            {"goal_clear_margin_deficit_m", finiteDoubleJson(snapshot.init_motion.goal_clear_margin_deficit_m)},
            {"waypoint_index", snapshot.init_motion.waypoint_index},
            {"waypoint_count", snapshot.init_motion.waypoint_count},
            {"dist_to_goal_deg", finiteDoubleJson(snapshot.init_motion.dist_to_goal_deg)},
            {"clear_threshold_m", finiteDoubleJson(snapshot.init_motion.clear_threshold_m)},
            {"external_clear_threshold_m", finiteDoubleJson(snapshot.init_motion.external_clear_threshold_m)},
            {"nearest_pair", snapshot.init_motion.nearest_pair},
            {"nearest_pair_distance_m", finiteDoubleJson(snapshot.init_motion.nearest_pair_distance_m)},
            {"nearest_pair_external", snapshot.init_motion.nearest_pair_external},
        }},
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
    {
        nlohmann::json self_collision;
        self_collision["enabled"] = snapshot.self_collision_enabled;
        self_collision["checked"] = snapshot.self_collision_checked;
        self_collision["violated"] = snapshot.self_collision_violated;
        self_collision["margin_m"] = snapshot.self_collision_margin_m;
        if (snapshot.self_collision_checked &&
            std::isfinite(snapshot.self_collision_min_clearance_m)) {
            self_collision["min_clearance_m"] = snapshot.self_collision_min_clearance_m;
        } else {
            self_collision["min_clearance_m"] = nullptr;
        }
        if (std::isfinite(snapshot.self_collision_external_box_min_clearance_m)) {
            self_collision["external_box_min_clearance_m"] =
                snapshot.self_collision_external_box_min_clearance_m;
        } else {
            self_collision["external_box_min_clearance_m"] = nullptr;
        }
        {
            nlohmann::json arr = nlohmann::json::array();
            for (const double clearance : snapshot.self_collision_external_box_clearance_m) {
                arr.push_back(std::isfinite(clearance)
                    ? nlohmann::json(clearance)
                    : nlohmann::json(nullptr));
            }
            self_collision["external_box_clearance_m"] = std::move(arr);
        }
        self_collision["left_bone"] = snapshot.self_collision_left_bone;
        self_collision["right_bone"] = snapshot.self_collision_right_bone;
        self_collision["pair"] = snapshot.self_collision_pair.empty()
            ? nlohmann::json(nullptr)
            : nlohmann::json(snapshot.self_collision_pair);
        self_collision["stand_capsule"] = snapshot.self_collision_stand_capsule.empty()
            ? nlohmann::json(nullptr)
            : nlohmann::json(snapshot.self_collision_stand_capsule);
        // Closest bone-axis points (stand frame) of the min-clearance pair —
        // on the members themselves — for GUI witness-point markers.
        if (snapshot.self_collision_has_closest_points) {
            self_collision["closest_point_a_m"] = snapshot.self_collision_closest_point_a_m;
            self_collision["closest_point_b_m"] = snapshot.self_collision_closest_point_b_m;
        } else {
            self_collision["closest_point_a_m"] = nullptr;
            self_collision["closest_point_b_m"] = nullptr;
        }
        // URDF mesh self-collision: closest near pairs (witness segments) so the
        // viewer can draw the mesh-based close calls (witness points + clearance).
        self_collision["mesh"] = snapshot.self_collision_mesh;
        if (snapshot.self_collision_mesh && !snapshot.self_collision_near_pairs.empty()) {
            nlohmann::json arr = nlohmann::json::array();
            for (const auto& p : snapshot.self_collision_near_pairs) {
                nlohmann::json entry;
                entry["name_a"] = p.name_a;
                entry["name_b"] = p.name_b;
                entry["p_a_m"] = p.p_a_m;
                entry["p_b_m"] = p.p_b_m;
                entry["clearance_m"] = p.clearance_m;
                entry["external"] = p.external;
                entry["external_box"] = p.external_box;
                arr.push_back(std::move(entry));
            }
            self_collision["near_pairs"] = std::move(arr);
        } else {
            self_collision["near_pairs"] = nullptr;
        }
        // Static collision-geometry manifest so the GUI mirrors exactly what the
        // monitor checks (single source of truth: the server config, not a
        // hardcoded viewer URDF). Present only when the guard is enabled.
        if (config_.safety.self_collision.enable) {
            self_collision["manifest"] = selfCollisionManifestJson(
                config_.safety.self_collision, config_.kinematics.joint_names);
        } else {
            self_collision["manifest"] = nullptr;
        }
        message["self_collision"] = self_collision;
    }
    {
        nlohmann::json floor;
        floor["enabled"] = snapshot.floor_constraint_enabled;
        floor["monitor_only"] = snapshot.floor_constraint_monitor_only;
        floor["z_min_m"] = snapshot.floor_constraint_z_min_m;
        floor["config_z_min_m"] = snapshot.floor_constraint_config_z_min_m;
        floor["runtime_min_z_m"] = snapshot.floor_constraint_runtime_min_z_m;
        floor["runtime_max_z_m"] = snapshot.floor_constraint_runtime_max_z_m;
        const auto arm_json = [](bool checked, bool violated, double tcp_z_m,
                                 const std::string& lowest_point,
                                 const std::array<double, 3>& lowest_point_m) {
            nlohmann::json arm;
            arm["checked"] = checked;
            arm["violated"] = violated;
            if (checked && std::isfinite(tcp_z_m)) {
                // Lowest checked point's z (TCP, or a configured TCP-frame
                // offset point such as a gripper fingertip).
                arm["tcp_z_m"] = tcp_z_m;
                arm["lowest_point"] = lowest_point.empty() ? "tcp" : lowest_point;
                // Full stand-frame xyz of that lowest point, so the GUI can capture
                // the true floor-contact sample when fitting a user floor plane.
                arm["lowest_point_m"] = nlohmann::json::array(
                    {lowest_point_m[0], lowest_point_m[1], lowest_point_m[2]});
            } else {
                arm["tcp_z_m"] = nullptr;
                arm["lowest_point"] = nullptr;
                arm["lowest_point_m"] = nullptr;
            }
            return arm;
        };
        floor["left"] = arm_json(
            snapshot.floor_constraint_left_checked,
            snapshot.floor_constraint_left_violated,
            snapshot.floor_constraint_left_tcp_z_m,
            snapshot.floor_constraint_left_lowest_point,
            snapshot.floor_constraint_left_lowest_point_m);
        floor["right"] = arm_json(
            snapshot.floor_constraint_right_checked,
            snapshot.floor_constraint_right_violated,
            snapshot.floor_constraint_right_tcp_z_m,
            snapshot.floor_constraint_right_lowest_point,
            snapshot.floor_constraint_right_lowest_point_m);
        floor["clamp_count"] = snapshot.floor_constraint_clamp_count;
        if (snapshot.floor_constraint_last_set_reject_reason.empty()) {
            floor["last_set_reject_reason"] = nullptr;
        } else {
            floor["last_set_reject_reason"] = snapshot.floor_constraint_last_set_reject_reason;
        }
        message["floor_constraint"] = floor;
    }
    {
        nlohmann::json roi;
        roi["enabled"] = snapshot.roi_box_enabled;
        roi["monitor_only"] = snapshot.roi_box_monitor_only;
        const auto vec3 = [](const std::array<double, 3>& v) {
            return nlohmann::json::array({v[0], v[1], v[2]});
        };
        roi["min_m"] = vec3(snapshot.roi_box_min_m);          // effective (runtime)
        roi["max_m"] = vec3(snapshot.roi_box_max_m);
        roi["runtime_min_m"] = vec3(snapshot.roi_box_runtime_min_m);
        roi["runtime_max_m"] = vec3(snapshot.roi_box_runtime_max_m);
        const auto arm_json = [](bool checked, bool violated, double min_margin_m,
                                 const std::string& closest_face) {
            nlohmann::json arm;
            arm["checked"] = checked;
            arm["violated"] = violated;
            if (checked && std::isfinite(min_margin_m)) {
                arm["min_margin_m"] = min_margin_m;
                arm["closest_face"] = closest_face.empty() ? "?" : closest_face;
            } else {
                arm["min_margin_m"] = nullptr;
                arm["closest_face"] = nullptr;
            }
            return arm;
        };
        roi["left"] = arm_json(
            snapshot.roi_box_left_checked, snapshot.roi_box_left_violated,
            snapshot.roi_box_left_min_margin_m, snapshot.roi_box_left_closest_face);
        roi["right"] = arm_json(
            snapshot.roi_box_right_checked, snapshot.roi_box_right_violated,
            snapshot.roi_box_right_min_margin_m, snapshot.roi_box_right_closest_face);
        roi["clamp_count"] = snapshot.roi_box_clamp_count;
        if (snapshot.roi_box_last_set_reject_reason.empty()) {
            roi["last_set_reject_reason"] = nullptr;
        } else {
            roi["last_set_reject_reason"] = snapshot.roi_box_last_set_reject_reason;
        }
        message["roi_box"] = roi;
    }
    {
        nlohmann::json uf;
        uf["enabled"] = snapshot.user_floor_constraint_enabled;
        uf["monitor_only"] = snapshot.user_floor_constraint_monitor_only;
        const auto vec3 = [](const std::array<double, 3>& v) {
            return nlohmann::json::array({v[0], v[1], v[2]});
        };
        uf["point_m"] = vec3(snapshot.user_floor_constraint_point_m);   // effective (runtime)
        uf["normal"] = vec3(snapshot.user_floor_constraint_normal);
        uf["margin_m"] = snapshot.user_floor_constraint_margin_m;
        const auto arm_json = [&](bool checked, bool violated, double signed_dist_m,
                                  const std::string& lowest_point,
                                  const std::array<double, 3>& lowest_point_m) {
            nlohmann::json arm;
            arm["checked"] = checked;
            arm["violated"] = violated;
            if (checked && std::isfinite(signed_dist_m)) {
                arm["signed_dist_m"] = signed_dist_m;
                arm["lowest_point"] = lowest_point.empty() ? "tcp" : lowest_point;
                arm["lowest_point_m"] = vec3(lowest_point_m);
            } else {
                arm["signed_dist_m"] = nullptr;
                arm["lowest_point"] = nullptr;
                arm["lowest_point_m"] = nullptr;
            }
            return arm;
        };
        uf["left"] = arm_json(
            snapshot.user_floor_constraint_left_checked,
            snapshot.user_floor_constraint_left_violated,
            snapshot.user_floor_constraint_left_signed_dist_m,
            snapshot.user_floor_constraint_left_lowest_point,
            snapshot.user_floor_constraint_left_lowest_point_m);
        uf["right"] = arm_json(
            snapshot.user_floor_constraint_right_checked,
            snapshot.user_floor_constraint_right_violated,
            snapshot.user_floor_constraint_right_signed_dist_m,
            snapshot.user_floor_constraint_right_lowest_point,
            snapshot.user_floor_constraint_right_lowest_point_m);
        uf["clamp_count"] = snapshot.user_floor_constraint_clamp_count;
        if (snapshot.user_floor_constraint_last_set_reject_reason.empty()) {
            uf["last_set_reject_reason"] = nullptr;
        } else {
            uf["last_set_reject_reason"] = snapshot.user_floor_constraint_last_set_reject_reason;
        }
        message["user_floor_constraint"] = uf;
    }
    message["motion_state"] = toString(snapshot.motion_state);
    message["fault_latched"] = snapshot.fault_latched;
    message["async_supervision_degraded"] = snapshot.async_supervision_degraded;
    message["tracking_error_degraded"] = snapshot.tracking_error_degraded;
    message["freedrive"] = {
        {"left_active", snapshot.left_freedrive_active},
        {"right_active", snapshot.right_freedrive_active},
        {"any_active", snapshot.left_freedrive_active || snapshot.right_freedrive_active},
        {"left_stage", snapshot.left_freedrive_stage},
        {"right_stage", snapshot.right_freedrive_stage},
        {"note", snapshot.freedrive_note},
    };
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

    if (gripper_bridge_ && !gripper_bridge_->start()) {
        std::cerr << "[ERROR] StatePublisher failed to start gripper bridge\n";
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
    if (gripper_bridge_) gripper_bridge_->stop();
}

void StatePublisher::threadMain() {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "[ERROR] StatePublisher socket failed: " << std::strerror(errno) << "\n";
        running_ = false;
        return;
    }

    const auto publish_period = publishPeriod(config_.network.state_pub_rate_hz);
    const std::vector<UdpDestination> destinations = resolveUdpDestinations(config_.network);
    if (destinations.empty()) {
        std::cerr << "[ERROR] StatePublisher resolved no UDP state destinations\n";
        ::close(fd);
        running_ = false;
        return;
    }
    std::set<std::string> send_warned_endpoints;
    while (running_) {
        ServoSnapshot snapshot;
        if (snapshot_provider_) {
            snapshot = snapshot_provider_();
            updateSnapshot(snapshot);
        } else {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            snapshot = latest_snapshot_;
        }

        // Forward the arbitrated gripper setpoint to gripper_server (rate-limited
        // inside the bridge). Off the RT loop, non-blocking.
        if (gripper_bridge_) gripper_bridge_->forward(snapshot.command);

        // Push the latest gripper open percent into the control loop's safety gate so
        // the TCP fingertip offset points track the live jaw geometry. Non-blocking
        // (atomic store on the consumer). invalid feedback -> gate falls back to open.
        if (gripper_feedback_sink_) {
            const GripperArmFeedback left_fb =
                gripper_bridge_ ? gripper_bridge_->latest(ArmId::Left) : GripperArmFeedback{};
            const GripperArmFeedback right_fb =
                gripper_bridge_ ? gripper_bridge_->latest(ArmId::Right) : GripperArmFeedback{};
            gripper_feedback_sink_(ArmId::Left, left_fb.percent, left_fb.valid);
            gripper_feedback_sink_(ArmId::Right, right_fb.percent, right_fb.valid);
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
            if (sent < 0 && send_warned_endpoints.insert(destination.endpoint).second) {
                std::cerr << "[WARN] StatePublisher send failed to "
                          << destination.endpoint << ": " << std::strerror(errno) << "\n";
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
    if (!config_.network.state_pub_endpoints.empty()) {
        return parseUdpEndpointUri(config_.network.state_pub_endpoints.front(), host, port);
    }
    return parseUdpEndpointUri(config_.network.state_pub_endpoint, host, port);
}

}  // namespace rb_servo
