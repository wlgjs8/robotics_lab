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
#include <stdexcept>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace rb_servo {
namespace {

constexpr int kStateSchemaVersion = 1;

std::chrono::nanoseconds publishPeriod(int state_pub_rate_hz) {
    const int rate_hz = state_pub_rate_hz > 0 ? state_pub_rate_hz : 20;
    return std::chrono::nanoseconds(1'000'000'000LL / rate_hz);
}

nlohmann::json jointArrayJson(const JointArray& joints) {
    nlohmann::json out = nlohmann::json::array();
    for (double value : joints) out.push_back(value);
    return out;
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

double ageUs(uint64_t newer_ns, uint64_t older_ns) {
    if (newer_ns == 0 || older_ns == 0 || newer_ns < older_ns) return 0.0;
    return static_cast<double>(newer_ns - older_ns) / 1000.0;
}

bool sendDeadlineHit(uint64_t loop_start_ns, double period_ms, uint64_t send_end_ns) {
    if (loop_start_ns == 0 || send_end_ns == 0 || period_ms <= 0.0) return false;
    const auto period_ns = static_cast<uint64_t>(period_ms * 1'000'000.0);
    return send_end_ns <= loop_start_ns + period_ns;
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
    } else {
        out["ok"] = call.ok;
    }
    return out;
}

nlohmann::json workerTelemetryJson(const ArmWorkerTelemetry& telemetry, bool enabled) {
    return {
        {"enabled", enabled},
        {"queue_policy", telemetry.worker_queue_policy},
        {"command_drops_total", telemetry.worker_command_drops_total},
        {"pending_overwrites_total", telemetry.worker_pending_overwrites_total},
        {"last_dropped_seq", telemetry.worker_last_dropped_seq},
        {"last_enqueued_seq", telemetry.worker_last_enqueued_seq},
        {"last_dispatched_seq", telemetry.worker_last_dispatched_seq},
        {"last_completed_seq", telemetry.worker_last_completed_seq},
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
        {"ik_status", telemetry.status},
        {"ik_reason", telemetry.reason},
        {"ik_timed_out", telemetry.ik_timed_out},
        {"ik_warn_duration_exceeded", telemetry.ik_warn_duration_exceeded},
        {"ik_fail_duration_exceeded", telemetry.ik_fail_duration_exceeded},
        {"warn_ik_duration_us", telemetry.warn_ik_duration_us},
        {"fail_ik_duration_us", telemetry.fail_ik_duration_us},
    };
}

nlohmann::json faultContextJson(const ServoSnapshot& snapshot) {
    nlohmann::json out = {
        {"latched", snapshot.fault_latched},
        {"motion_state", toString(snapshot.motion_state)},
        {"safety_verdict", toString(snapshot.safety_verdict)},
        {"latched_fault_reason", toString(snapshot.latched_fault_reason)},
        {"reason", snapshot.fault_reason},
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
    out["verdict"] = context.verdict;
    out["domain"] = context.domain;
    out["arm"] = context.arm;
    out["backend_op"] = context.backend_op;
    out["backend_error_kind"] = context.backend_error_kind;
    out["backend_error_name"] = context.backend_error_name;
    out["backend_error_code"] = context.backend_error_code;
    out["retryable"] = context.retryable;
    out["recoverable"] = context.recoverable;
    out["robot_fault"] = context.robot_fault;
    out["transport_fault"] = context.transport_fault;
    out["state_after_source"] = context.state_after_source;
    out["reason"] = context.reason;
    return out;
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
    bool send_deadline_hit,
    double worker_loop_read_duration_us,
    const ArmWorkerTelemetry& worker_telemetry,
    bool worker_enabled,
    const CartesianSolveTelemetry& cartesian_solve
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
        {"send_deadline_hit", send_deadline_hit},
        {"worker_loop_read_duration_us", worker_loop_read_duration_us},
        {"worker", workerTelemetryJson(worker_telemetry, worker_enabled)},
        {"has_valid_joint_state", state.has_valid_joint_state},
        {"connection_state", state.connection_state == RobotConnectionState::Connected
            ? "Connected"
            : state.connection_state == RobotConnectionState::Error ? "Error" : "Disconnected"},
        {"has_error", state.has_error},
        {"servo_enabled", state.servo_enabled},
        {"fault_recoverable", optionalBoolJson(state.fault_recoverable)},
        {"lifecycle_state", optionalStringJson(state.lifecycle_state)},
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

}  // namespace

StatePublisher::StatePublisher(const DualArmConfig& config, SnapshotProvider provider)
    : config_(config), snapshot_provider_(std::move(provider)) {}

StatePublisher::StatePublisher(const NetworkConfig& config) {
    config_.network = config;
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
    message["observed_mode"] = observedModeString(config_);
    message["observed_backend"] = observedBackendString(config_);
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
        sendDeadlineHit(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.left_send_end_ns),
        worker_enabled ? snapshot.left_last_read.duration_us : 0.0,
        snapshot.left_worker_telemetry,
        worker_enabled,
        snapshot.left_cartesian_solve
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
        sendDeadlineHit(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.right_send_end_ns),
        worker_enabled ? snapshot.right_last_read.duration_us : 0.0,
        snapshot.right_worker_telemetry,
        worker_enabled,
        snapshot.right_cartesian_solve
    );
    message["last_cartesian_solve"] = {
        {"left", cartesianSolveJson(snapshot.left_cartesian_solve)},
        {"right", cartesianSolveJson(snapshot.right_cartesian_solve)},
    };

    message["send_skew_us"] = snapshot.send_skew_us;
    message["dispatch_skew_us"] = snapshot.send_skew_us;
    message["send_deadline_hit"] =
        sendDeadlineHit(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.left_send_end_ns) &&
        sendDeadlineHit(snapshot.loop_start_time_ns, snapshot.period_ms, snapshot.right_send_end_ns);
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

    std::string host;
    int port = 0;
    if (!parseEndpoint(&host, &port)) {
        std::cerr << "[ERROR] StatePublisher only supports udp://host:port endpoints, got "
                  << config_.network.state_pub_bind << "\n";
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
    std::string host;
    int port = 0;
    if (!parseEndpoint(&host, &port)) {
        running_ = false;
        return;
    }

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "[ERROR] StatePublisher socket failed: " << std::strerror(errno) << "\n";
        running_ = false;
        return;
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
        ::close(fd);
        running_ = false;
        return;
    }

    std::vector<char> dest_storage;
    const sockaddr* dest_addr = nullptr;
    socklen_t dest_len = 0;
    for (addrinfo* item = results; item != nullptr; item = item->ai_next) {
        if (!item->ai_addr || item->ai_addrlen <= 0) continue;
        dest_storage.assign(
            reinterpret_cast<const char*>(item->ai_addr),
            reinterpret_cast<const char*>(item->ai_addr) + item->ai_addrlen
        );
        dest_addr = reinterpret_cast<const sockaddr*>(dest_storage.data());
        dest_len = static_cast<socklen_t>(item->ai_addrlen);
        break;
    }
    ::freeaddrinfo(results);

    if (!dest_addr || dest_len == 0) {
        std::cerr << "[ERROR] StatePublisher found no UDP address for host '" << host << "'\n";
        ::close(fd);
        running_ = false;
        return;
    }

    const auto publish_period = publishPeriod(config_.network.state_pub_rate_hz);
    bool send_warned = false;
    while (running_) {
        ServoSnapshot snapshot;
        if (snapshot_provider_) {
            snapshot = snapshot_provider_();
            updateSnapshot(snapshot);
        } else {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            snapshot = latest_snapshot_;
        }

        const std::string payload = serializeSnapshot(snapshot);
        const ssize_t sent = ::sendto(
            fd,
            payload.data(),
            payload.size(),
            0,
            dest_addr,
            dest_len
        );
        if (sent < 0 && !send_warned) {
            std::cerr << "[WARN] StatePublisher send failed: " << std::strerror(errno) << "\n";
            send_warned = true;
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
