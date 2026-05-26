#include "rb_servo/robot/rbsim_backend.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "rb_servo/core/clock.hpp"

namespace rb_servo {
namespace {

using json = nlohmann::json;

constexpr uint64_t kInitialReconnectBackoffNs = 50ULL * 1000ULL * 1000ULL;
constexpr uint64_t kMaxReconnectBackoffNs = 1000ULL * 1000ULL * 1000ULL;

struct TcpEndpoint {
    std::string host;
    int port = 0;
};

TcpEndpoint parseTcpEndpoint(const std::string& endpoint) {
    constexpr const char* kPrefix = "tcp://";
    if (endpoint.rfind(kPrefix, 0) != 0) {
        throw std::runtime_error("RbsimBackend endpoint must use tcp://host:port: " + endpoint);
    }

    const std::string rest = endpoint.substr(std::strlen(kPrefix));
    const auto colon = rest.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= rest.size()) {
        throw std::runtime_error("Invalid RbsimBackend endpoint: " + endpoint);
    }

    TcpEndpoint out;
    out.host = rest.substr(0, colon);
    out.port = std::stoi(rest.substr(colon + 1));
    if (out.port <= 0 || out.port > 65535) {
        throw std::runtime_error("Invalid RbsimBackend endpoint port: " + endpoint);
    }
    return out;
}

timeval timeoutValue(double seconds) {
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(seconds);
    tv.tv_usec = static_cast<suseconds_t>((seconds - static_cast<double>(tv.tv_sec)) * 1000000.0);
    return tv;
}

bool jsonBool(const json& object, const char* key, bool fallback) {
    const auto it = object.find(key);
    return it != object.end() && it->is_boolean() ? it->get<bool>() : fallback;
}

int jsonInt(const json& object, const char* key, int fallback) {
    const auto it = object.find(key);
    return it != object.end() && it->is_number_integer() ? it->get<int>() : fallback;
}

uint64_t jsonUint64(const json& object, const char* key, uint64_t fallback) {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_number_unsigned()) return fallback;
    return it->get<uint64_t>();
}

bool readJointArray(const json& object, const char* key, JointArray* out) {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_array() || it->size() != static_cast<size_t>(kDof)) return false;
    JointArray parsed{};
    for (int i = 0; i < kDof; ++i) {
        const json& item = (*it)[static_cast<size_t>(i)];
        if (!item.is_number()) return false;
        const double value = item.get<double>();
        if (!std::isfinite(value)) return false;
        parsed[static_cast<size_t>(i)] = value;
    }
    *out = parsed;
    return true;
}

RobotConnectionState parseConnectionState(const json& state) {
    const auto it = state.find("connection_state");
    if (it == state.end() || !it->is_string()) return RobotConnectionState::Error;
    const std::string value = it->get<std::string>();
    if (value == "Connected") return RobotConnectionState::Connected;
    if (value == "Disconnected") return RobotConnectionState::Disconnected;
    return RobotConnectionState::Error;
}

bool mapState(const json& state, ArmId arm_id, RobotState* out) {
    if (!state.is_object()) return false;

    RobotState mapped;
    mapped.arm_id = arm_id;
    mapped.host_time_ns = nowSteadyNs();
    mapped.robot_time_ns = jsonUint64(state, "robot_time_ns", 0);
    if (!readJointArray(state, "q_actual_deg", &mapped.q_actual_deg)) return false;
    if (!readJointArray(state, "q_target_deg", &mapped.q_target_deg)) return false;
    if (!readJointArray(state, "dq_actual_deg_s", &mapped.dq_actual_deg_s)) return false;
    mapped.has_valid_joint_state = jsonBool(state, "has_valid_joint_state", false);
    if (jsonBool(state, "stale_state", false)) {
        mapped.has_valid_joint_state = false;
    }
    mapped.connection_state = parseConnectionState(state);
    mapped.servo_enabled = jsonBool(state, "servo_enabled", false);
    mapped.has_error = jsonBool(state, "has_error", false);
    mapped.error_code = jsonInt(state, "error_code", 0);

    *out = mapped;
    return true;
}

json jointArrayJson(const JointArray& joints) {
    json out = json::array();
    for (double q : joints) out.push_back(q);
    return out;
}

double timeoutForOperation(const BackendConfig& config, const std::string& op) {
    if (op == "connect" || op == "initialize") return config.rbsim_connect_timeout_sec;
    if (op == "read_state") return config.rbsim_read_timeout_sec;
    if (op == "send_servo_j") return config.rbsim_send_timeout_sec;
    if (op == "stop") return config.rbsim_stop_timeout_sec;
    if (op == "reset_fault") return config.rbsim_reset_timeout_sec;
    return config.rbsim_request_timeout_sec;
}

std::string simulatorEndpoint(const BackendConfig& config) {
    const BackendConfig defaults;
    if (config.simulator_control_endpoint == defaults.simulator_control_endpoint &&
        config.rbsim_control_endpoint != defaults.rbsim_control_endpoint) {
        return config.rbsim_control_endpoint;
    }
    return config.simulator_control_endpoint;
}

bool closesPersistentTransport(BackendErrorKind kind) {
    return kind == BackendErrorKind::TransportConnectFailed ||
           kind == BackendErrorKind::TransportWriteFailed ||
           kind == BackendErrorKind::TransportReadFailed ||
           kind == BackendErrorKind::TransportTimeout ||
           kind == BackendErrorKind::ProtocolError;
}

template <typename T>
BackendResult<T> okResult(BackendOp op, const T& value, const BackendTiming& timing = BackendTiming{}) {
    BackendResult<T> result;
    result.ok = true;
    result.op = op;
    result.value = value;
    result.error = noBackendError();
    result.timing = timing;
    return result;
}

template <typename T>
BackendResult<T> failedResult(BackendOp op, const BackendError& error, const BackendTiming& timing = BackendTiming{}) {
    BackendResult<T> result;
    result.ok = false;
    result.op = op;
    result.error = error;
    result.timing = timing;
    return result;
}

std::string jsonCodeString(const json& object, const char* key) {
    const auto it = object.find(key);
    if (it == object.end()) return "";
    if (it->is_string()) return it->get<std::string>();
    if (it->is_number_integer()) return std::to_string(it->get<int>());
    if (it->is_number_unsigned()) return std::to_string(it->get<uint64_t>());
    return "";
}

std::string jsonString(const json& object, const char* key, const std::string& fallback = "") {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_string()) return fallback;
    return it->get<std::string>();
}

std::optional<bool> jsonOptionalBool(const json& object, const char* key) {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_boolean()) return std::nullopt;
    return it->get<bool>();
}

BackendErrorKind kindFromString(const std::string& kind) {
    if (kind == "TransportConnectFailed") return BackendErrorKind::TransportConnectFailed;
    if (kind == "TransportWriteFailed") return BackendErrorKind::TransportWriteFailed;
    if (kind == "TransportReadFailed") return BackendErrorKind::TransportReadFailed;
    if (kind == "TransportTimeout") return BackendErrorKind::TransportTimeout;
    if (kind == "ProtocolError") return BackendErrorKind::ProtocolError;
    if (kind == "UnsupportedSchema") return BackendErrorKind::UnsupportedSchema;
    if (kind == "WrongArm") return BackendErrorKind::WrongArm;
    if (kind == "WrongEndpoint") return BackendErrorKind::WrongEndpoint;
    if (kind == "UnknownArm") return BackendErrorKind::UnknownArm;
    if (kind == "RobotDisconnected") return BackendErrorKind::RobotDisconnected;
    if (kind == "RobotNotInitialized") return BackendErrorKind::RobotNotInitialized;
    if (kind == "ServoDisabled") return BackendErrorKind::ServoDisabled;
    if (kind == "WrongMode") return BackendErrorKind::WrongMode;
    if (kind == "RobotFault") return BackendErrorKind::RobotFault;
    if (kind == "InvalidJointState") return BackendErrorKind::InvalidJointState;
    if (kind == "InvalidTarget") return BackendErrorKind::InvalidTarget;
    if (kind == "ControllerRejected") return BackendErrorKind::ControllerRejected;
    if (kind == "CommandTimeout") return BackendErrorKind::CommandTimeout;
    if (kind == "DependencyUnavailable") return BackendErrorKind::DependencyUnavailable;
    if (kind == "SuppressedByPolicy") return BackendErrorKind::SuppressedByPolicy;
    if (kind == "Unknown") return BackendErrorKind::Unknown;
    return BackendErrorKind::Unknown;
}

BackendErrorKind fallbackKindFromRbsimName(const std::string& name) {
    if (name == "unsupported_schema_version") return BackendErrorKind::UnsupportedSchema;
    if (name == "wrong_arm") return BackendErrorKind::WrongArm;
    if (name == "wrong_endpoint") return BackendErrorKind::WrongEndpoint;
    if (name == "unknown_arm") return BackendErrorKind::UnknownArm;
    if (name == "disconnected") return BackendErrorKind::RobotDisconnected;
    if (name == "not_initialized") return BackendErrorKind::RobotNotInitialized;
    if (name == "servo_disabled") return BackendErrorKind::ServoDisabled;
    if (name == "fault_latched") return BackendErrorKind::RobotFault;
    if (name == "invalid_joint_state") return BackendErrorKind::InvalidJointState;
    if (name == "send_failure_injected") return BackendErrorKind::TransportWriteFailed;
    if (name == "read_failure_injected") return BackendErrorKind::TransportReadFailed;
    if (name == "stop_failure_injected") return BackendErrorKind::TransportWriteFailed;
    if (name == "reset_failure_injected") return BackendErrorKind::TransportWriteFailed;
    if (name == "state_rejected") return BackendErrorKind::ControllerRejected;
    if (name == "bad_request" || name == "invalid_json" || name == "unknown_operation") {
        return BackendErrorKind::ProtocolError;
    }
    return BackendErrorKind::ProtocolError;
}

BackendErrorKind errorKindFromResponse(const json& error) {
    const std::string explicit_kind = jsonString(error, "kind");
    if (!explicit_kind.empty()) {
        const BackendErrorKind kind = kindFromString(explicit_kind);
        if (kind != BackendErrorKind::Unknown || explicit_kind == "Unknown") return kind;
    }
    return fallbackKindFromRbsimName(jsonString(error, "name"));
}

bool hasMappedResponseState(const RobotState& state) {
    return state.host_time_ns != 0;
}

BackendError protocolErrorFromResponse(const std::string& op, const json& response) {
    const auto error_it = response.find("error");
    if (error_it == response.end() || !error_it->is_object()) {
        return backendError(BackendErrorKind::ProtocolError, "rbsim " + op + " failed without an error object");
    }
    const std::string name = jsonString(*error_it, "name", "ProtocolError");
    const std::string message = jsonString(*error_it, "message", "rbsim " + op + " failed");
    return backendError(
        errorKindFromResponse(*error_it),
        message,
        jsonCodeString(*error_it, "code"),
        name,
        jsonOptionalBool(*error_it, "retryable"),
        jsonOptionalBool(*error_it, "recoverable")
    );
}

}  // namespace

struct JsonLineRequestResult {
    bool ok = false;
    std::string line;
    BackendError error = noBackendError();
};

class JsonLineTcpClient {
public:
    explicit JsonLineTcpClient(std::string endpoint)
        : endpoint_text_(std::move(endpoint)), endpoint_(parseTcpEndpoint(endpoint_text_)) {}

    ~JsonLineTcpClient() {
        close();
    }

    JsonLineRequestResult request(const std::string& line, double timeout_sec, const std::string& op) {
        counters_.requests_total += 1;
        const BackendError connect_error = connectIfNeeded(timeout_sec);
        if (connect_error.kind != BackendErrorKind::None) {
            return JsonLineRequestResult{false, {}, connect_error};
        }

        setSocketTimeouts(timeout_sec);
        const BackendError write_error = sendAll(line, op);
        if (write_error.kind != BackendErrorKind::None) {
            closeForTransportError(write_error.kind);
            return JsonLineRequestResult{false, {}, write_error};
        }

        std::string response_line;
        const BackendError read_error = recvLineBuffered(&response_line, op);
        if (read_error.kind != BackendErrorKind::None) {
            closeForTransportError(read_error.kind);
            return JsonLineRequestResult{false, {}, read_error};
        }

        JsonLineRequestResult result;
        result.ok = true;
        result.line = std::move(response_line);
        return result;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        read_buffer_.clear();
    }

    void closeForBackendError(BackendErrorKind kind) {
        if (!closesPersistentTransport(kind)) return;
        closeForTransportError(kind);
    }

    RbsimTransportCounters counters() const {
        RbsimTransportCounters out = counters_;
        const uint64_t now = nowSteadyNs();
        out.next_connect_attempt_ns = next_allowed_connect_attempt_ns_;
        out.next_connect_attempt_delay_ms =
            next_allowed_connect_attempt_ns_ > now
                ? ceilDiv(next_allowed_connect_attempt_ns_ - now, 1000ULL * 1000ULL)
                : 0;
        return out;
    }

private:
    BackendError connectIfNeeded(double timeout_sec) {
        if (fd_ >= 0) return noBackendError();

        const uint64_t now = nowSteadyNs();
        if (next_allowed_connect_attempt_ns_ > now) {
            counters_.connect_attempts_suppressed_total += 1;
            const uint64_t remaining_ms = ceilDiv(next_allowed_connect_attempt_ns_ - now, 1000ULL * 1000ULL);
            const std::string message =
                "rbsim connect suppressed by reconnect backoff for " + endpoint_text_ +
                "; next retry in " + std::to_string(remaining_ms) + " ms";
            recordConnectError("rbsim_connect_backoff", message);
            return backendError(
                BackendErrorKind::TransportConnectFailed,
                message,
                "",
                "rbsim_connect_backoff",
                true,
                true
            );
        }

        counters_.connect_attempts_total += 1;

        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;

        addrinfo* results = nullptr;
        const std::string port = std::to_string(endpoint_.port);
        const int gai = ::getaddrinfo(endpoint_.host.c_str(), port.c_str(), &hints, &results);
        if (gai != 0 || results == nullptr) {
            const std::string message = "rbsim resolve failed for " + endpoint_.host + ": " + ::gai_strerror(gai);
            std::cerr << "[ERROR] RbsimBackend " << message << "\n";
            noteConnectFailure("rbsim_resolve_failed", message);
            return backendError(
                BackendErrorKind::TransportConnectFailed,
                message,
                "",
                "rbsim_resolve_failed",
                true,
                true
            );
        }

        int opened_fd = -1;
        std::string last_connect_error = "no address candidates";
        for (addrinfo* item = results; item != nullptr; item = item->ai_next) {
            opened_fd = ::socket(item->ai_family, item->ai_socktype, item->ai_protocol);
            if (opened_fd < 0) {
                last_connect_error = std::string("socket failed: ") + std::strerror(errno);
                continue;
            }

            setSocketTimeouts(opened_fd, timeout_sec);
            if (::connect(opened_fd, item->ai_addr, item->ai_addrlen) == 0) break;

            last_connect_error = std::string("connect failed: ") + std::strerror(errno);
            ::close(opened_fd);
            opened_fd = -1;
        }

        ::freeaddrinfo(results);
        if (opened_fd < 0) {
            const std::string message = "rbsim connect failed for " + endpoint_text_ + ": " + last_connect_error;
            noteConnectFailure("rbsim_connect_failed", message);
            return backendError(
                BackendErrorKind::TransportConnectFailed,
                message,
                "",
                "rbsim_connect_failed",
                true,
                true
            );
        }

        configureConnectedSocket(opened_fd);
        if (counters_.connections_opened_total > 0) {
            counters_.reconnects_total += 1;
        }
        counters_.connections_opened_total += 1;
        current_connect_backoff_ns_ = kInitialReconnectBackoffNs;
        next_allowed_connect_attempt_ns_ = nowSteadyNs();
        counters_.next_connect_attempt_ns = next_allowed_connect_attempt_ns_;
        fd_ = opened_fd;
        read_buffer_.clear();
        return noBackendError();
    }

    void setSocketTimeouts(double timeout_sec) {
        if (fd_ >= 0) setSocketTimeouts(fd_, timeout_sec);
    }

    static void setSocketTimeouts(int fd, double timeout_sec) {
        timeval tv = timeoutValue(timeout_sec);
        (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }

    void configureConnectedSocket(int fd) const {
        const int enabled = 1;
        if (::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) != 0) {
            std::cerr << "[WARN] RbsimBackend failed to set TCP_NODELAY for "
                      << endpoint_text_ << ": " << std::strerror(errno) << "\n";
        }
        if (::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled)) != 0) {
            std::cerr << "[WARN] RbsimBackend failed to set SO_KEEPALIVE for "
                      << endpoint_text_ << ": " << std::strerror(errno) << "\n";
        }
#ifdef __linux__
#ifdef TCP_KEEPIDLE
        setTcpKeepaliveInt(fd, TCP_KEEPIDLE, 5, "TCP_KEEPIDLE");
#endif
#ifdef TCP_KEEPINTVL
        setTcpKeepaliveInt(fd, TCP_KEEPINTVL, 2, "TCP_KEEPINTVL");
#endif
#ifdef TCP_KEEPCNT
        setTcpKeepaliveInt(fd, TCP_KEEPCNT, 3, "TCP_KEEPCNT");
#endif
#endif
    }

    void setTcpKeepaliveInt(int fd, int option, int value, const char* option_name) const {
        if (::setsockopt(fd, IPPROTO_TCP, option, &value, sizeof(value)) != 0) {
            std::cerr << "[WARN] RbsimBackend failed to set " << option_name << " for "
                      << endpoint_text_ << ": " << std::strerror(errno) << "\n";
        }
    }

    BackendError sendAll(const std::string& payload, const std::string& op) {
        const char* data = payload.data();
        size_t remaining = payload.size();
        while (remaining > 0) {
#ifdef MSG_NOSIGNAL
            constexpr int kSendFlags = MSG_NOSIGNAL;
#else
            constexpr int kSendFlags = 0;
#endif
            const ssize_t sent = ::send(fd_, data, remaining, kSendFlags);
            counters_.write_syscalls_total += 1;
            if (sent > 0) {
                data += sent;
                remaining -= static_cast<size_t>(sent);
                continue;
            }
            if (sent < 0 && errno == EINTR) continue;
            if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                return backendError(BackendErrorKind::TransportTimeout, "rbsim write timed out for " + op);
            }
            return backendError(BackendErrorKind::TransportWriteFailed, "rbsim write failed for " + op);
        }
        return noBackendError();
    }

    BackendError recvLineBuffered(std::string* line, const std::string& op) {
        constexpr size_t kMaxResponseBytes = 64 * 1024;
        constexpr size_t kReadChunkBytes = 4096;
        line->clear();

        while (read_buffer_.size() <= kMaxResponseBytes) {
            const auto newline = read_buffer_.find('\n');
            if (newline != std::string::npos) {
                *line = read_buffer_.substr(0, newline);
                read_buffer_.erase(0, newline + 1);
                return noBackendError();
            }

            char chunk[kReadChunkBytes];
            const ssize_t received = ::recv(fd_, chunk, sizeof(chunk), 0);
            counters_.read_syscalls_total += 1;
            if (received > 0) {
                read_buffer_.append(chunk, static_cast<size_t>(received));
                continue;
            }
            if (received < 0 && errno == EINTR) continue;
            if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                return backendError(BackendErrorKind::TransportTimeout, "rbsim read timed out for " + op);
            }
            return backendError(BackendErrorKind::TransportReadFailed, "rbsim read failed for " + op);
        }

        return backendError(BackendErrorKind::ProtocolError, "rbsim " + op + " response exceeded JSON-line limit");
    }

    void closeForTransportError(BackendErrorKind kind) {
        counters_.last_transport_error_kind = kind;
        close();
        scheduleReconnectBackoff();
    }

    void noteConnectFailure(const std::string& name, const std::string& message) {
        counters_.connect_failures_total += 1;
        counters_.last_transport_error_kind = BackendErrorKind::TransportConnectFailed;
        recordConnectError(name, message);
        scheduleReconnectBackoff();
    }

    void recordConnectError(const std::string& name, const std::string& message) {
        counters_.last_connect_error_name = name;
        counters_.last_connect_error_message = message;
    }

    void scheduleReconnectBackoff() {
        const uint64_t now = nowSteadyNs();
        next_allowed_connect_attempt_ns_ = now + current_connect_backoff_ns_;
        counters_.next_connect_attempt_ns = next_allowed_connect_attempt_ns_;
        if (current_connect_backoff_ns_ < kMaxReconnectBackoffNs) {
            current_connect_backoff_ns_ =
                std::min<uint64_t>(current_connect_backoff_ns_ * 2ULL, kMaxReconnectBackoffNs);
        }
    }

    static uint64_t ceilDiv(uint64_t numerator, uint64_t denominator) {
        return denominator == 0 ? 0 : (numerator + denominator - 1ULL) / denominator;
    }

    std::string endpoint_text_;
    TcpEndpoint endpoint_;
    int fd_ = -1;
    std::string read_buffer_;
    RbsimTransportCounters counters_;
    uint64_t current_connect_backoff_ns_ = kInitialReconnectBackoffNs;
    uint64_t next_allowed_connect_attempt_ns_ = 0;
};

RbsimBackend::RbsimBackend(ArmId arm_id, const BackendConfig& config)
    : arm_id_(arm_id), config_(config) {}

RbsimBackend::~RbsimBackend() = default;

BackendResult<RobotState> RbsimBackend::connect() {
    BackendResult<RobotState> result = controlRequest("connect", BackendOp::Connect, json::object(), true);
    connected_ = result.ok && result.value.connection_state == RobotConnectionState::Connected;
    if (!connected_ && result.ok) {
        result.ok = false;
        result.error = backendError(BackendErrorKind::RobotDisconnected, "rbsim connect did not report Connected state");
    }
    return result;
}

BackendResult<RobotState> RbsimBackend::initialize() {
    json params;
    params["enable_servo"] = true;
    BackendResult<RobotState> result = controlRequest("initialize", BackendOp::Initialize, params, true);
    if (!result.ok) return result;
    if (result.value.connection_state != RobotConnectionState::Connected) {
        result.ok = false;
        result.error = backendError(BackendErrorKind::RobotDisconnected, "rbsim initialize did not report Connected state");
    } else if (!result.value.has_valid_joint_state) {
        result.ok = false;
        result.error = backendError(BackendErrorKind::InvalidJointState, "rbsim initialize returned invalid joint state");
    } else if (result.value.has_error) {
        result.ok = false;
        result.error = backendError(
            BackendErrorKind::RobotFault,
            "rbsim initialize returned robot error",
            std::to_string(result.value.error_code),
            "RobotFault"
        );
    }
    return result;
}

BackendResult<RobotState> RbsimBackend::readState() {
    return controlRequest("read_state", BackendOp::ReadState, json::object(), true);
}

SendServoJResult RbsimBackend::sendServoJ(const SendServoJRequest& request) {
    json params;
    params["q_target_deg"] = jointArrayJson(request.q_target_deg);
    BackendResult<RobotState> result = controlRequest("send_servo_j", BackendOp::SendServoJ, params, false);
    if (!result.ok) {
        if (hasMappedResponseState(result.value)) {
            return rejectedSend(request, result.error, result.timing, result.value, "response");
        }
        return rejectedSend(request, result.error, result.timing);
    }
    return acceptedSend(request, result.timing, result.value, "response");
}

BackendResult<RobotState> RbsimBackend::stop() {
    return controlRequest("stop", BackendOp::Stop, json::object(), false);
}

BackendResult<RobotState> RbsimBackend::resetFault() {
    BackendResult<RobotState> result = controlRequest("reset_fault", BackendOp::ResetFault, json::object(), false);
    if (!result.ok) return result;
    BackendResult<RobotState> initialized = initialize();
    initialized.op = BackendOp::ResetFault;
    return initialized;
}

bool RbsimBackend::isConnected() const {
    return connected_;
}

ArmId RbsimBackend::armId() const {
    return arm_id_;
}

std::string RbsimBackend::name() const {
    return config_.name;
}

RbsimTransportCounters RbsimBackend::transportCounters() const {
    return client_ ? client_->counters() : RbsimTransportCounters{};
}

BackendResult<RobotState> RbsimBackend::controlRequest(
    const std::string& op,
    BackendOp backend_op,
    const json& params,
    bool require_state
) {
    const uint64_t start = nowSteadyNs();
    try {
        if (!client_) {
            client_ = std::make_unique<JsonLineTcpClient>(simulatorEndpoint(config_));
        }
    } catch (const std::exception& exc) {
        return failedResult<RobotState>(
            backend_op,
            backendError(BackendErrorKind::WrongEndpoint, exc.what()),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    json request;
    request["schema_version"] = "rbsim.v1";
    request["request_id"] = config_.name + "-" + std::to_string(++request_seq_);
    request["op"] = op;
    request["arm"] = toString(arm_id_);
    request["params"] = params;

    const std::string line = request.dump() + "\n";
    const JsonLineRequestResult transport = client_->request(line, timeoutForOperation(config_, op), op);
    if (!transport.ok) {
        connected_ = false;
        return failedResult<RobotState>(
            backend_op,
            transport.error,
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    json response;
    try {
        response = json::parse(transport.line);
    } catch (const json::exception& exc) {
        std::cerr << "[ERROR] RbsimBackend invalid JSON response: " << exc.what() << "\n";
        client_->closeForBackendError(BackendErrorKind::ProtocolError);
        connected_ = false;
        return failedResult<RobotState>(
            backend_op,
            backendError(BackendErrorKind::ProtocolError, exc.what()),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    if (!jsonBool(response, "ok", false)) {
        const BackendError error = protocolErrorFromResponse(op, response);
        const bool closed_transport = closesPersistentTransport(error.kind);
        client_->closeForBackendError(error.kind);
        if (closed_transport) {
            connected_ = false;
        }
        RobotState response_state;
        const auto state_it = response.find("state");
        if (state_it != response.end() && mapState(*state_it, arm_id_, &response_state)) {
            connected_ = !closed_transport && response_state.connection_state == RobotConnectionState::Connected;
            BackendResult<RobotState> failed =
                failedResult<RobotState>(backend_op, error, makeBackendTiming(start, nowSteadyNs()));
            failed.value = response_state;
            std::cerr << "[WARN] RbsimBackend " << op << " failed for "
                      << toString(arm_id_) << ": " << error.name << " (" << error.code
                      << "): " << error.message << "\n";
            return failed;
        }
        std::cerr << "[WARN] RbsimBackend " << op << " failed for "
                  << toString(arm_id_) << ": " << error.name << " (" << error.code
                  << "): " << error.message << "\n";
        return failedResult<RobotState>(backend_op, error, makeBackendTiming(start, nowSteadyNs()));
    }

    const auto state_it = response.find("state");
    RobotState state;
    state.arm_id = arm_id_;
    state.host_time_ns = nowSteadyNs();
    state.connection_state = connected_ ? RobotConnectionState::Connected : RobotConnectionState::Disconnected;
    if (state_it != response.end()) {
        if (!mapState(*state_it, arm_id_, &state)) {
            std::cerr << "[ERROR] RbsimBackend " << op << " response missing valid state\n";
            return failedResult<RobotState>(
                backend_op,
                backendError(BackendErrorKind::UnsupportedSchema, "rbsim " + op + " response contained invalid state"),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        connected_ = state.connection_state == RobotConnectionState::Connected;
    } else if (require_state) {
        std::cerr << "[ERROR] RbsimBackend " << op << " response missing state\n";
        return failedResult<RobotState>(
            backend_op,
            backendError(BackendErrorKind::UnsupportedSchema, "rbsim " + op + " response missing state"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    return okResult(backend_op, state, makeBackendTiming(start, nowSteadyNs()));
}

}  // namespace rb_servo
