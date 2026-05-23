#include "rb_servo/robot/rbsim_backend.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>

#include "rb_servo/core/clock.hpp"

namespace rb_servo {
namespace {

using json = nlohmann::json;

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

int openTcpConnection(const TcpEndpoint& endpoint, double timeout_sec) {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* results = nullptr;
    const std::string port = std::to_string(endpoint.port);
    const int gai = ::getaddrinfo(endpoint.host.c_str(), port.c_str(), &hints, &results);
    if (gai != 0 || results == nullptr) {
        std::cerr << "[ERROR] RbsimBackend resolve failed for " << endpoint.host
                  << ": " << ::gai_strerror(gai) << "\n";
        return -1;
    }

    int fd = -1;
    for (addrinfo* item = results; item != nullptr; item = item->ai_next) {
        fd = ::socket(item->ai_family, item->ai_socktype, item->ai_protocol);
        if (fd < 0) continue;

        timeval tv = timeoutValue(timeout_sec);
        (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        if (::connect(fd, item->ai_addr, item->ai_addrlen) == 0) break;
        ::close(fd);
        fd = -1;
    }

    ::freeaddrinfo(results);
    return fd;
}

bool sendAll(int fd, const std::string& payload) {
    const char* data = payload.data();
    size_t remaining = payload.size();
    while (remaining > 0) {
        const ssize_t sent = ::send(fd, data, remaining, 0);
        if (sent <= 0) return false;
        data += sent;
        remaining -= static_cast<size_t>(sent);
    }
    return true;
}

bool recvLine(int fd, std::string* line) {
    line->clear();
    constexpr size_t kMaxResponseBytes = 64 * 1024;
    while (line->size() < kMaxResponseBytes) {
        char c = '\0';
        const ssize_t received = ::recv(fd, &c, 1, 0);
        if (received <= 0) return false;
        if (c == '\n') return true;
        line->push_back(c);
    }
    return false;
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

RbsimBackend::RbsimBackend(ArmId arm_id, const BackendConfig& config)
    : arm_id_(arm_id), config_(config) {}

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

BackendResult<RobotState> RbsimBackend::controlRequest(
    const std::string& op,
    BackendOp backend_op,
    const json& params,
    bool require_state
) {
    const uint64_t start = nowSteadyNs();
    TcpEndpoint endpoint;
    try {
        endpoint = parseTcpEndpoint(simulatorEndpoint(config_));
    } catch (const std::exception& exc) {
        return failedResult<RobotState>(
            backend_op,
            backendError(BackendErrorKind::WrongEndpoint, exc.what()),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    const int fd = openTcpConnection(endpoint, timeoutForOperation(config_, op));
    if (fd < 0) {
        connected_ = false;
        return failedResult<RobotState>(
            backend_op,
            backendError(BackendErrorKind::TransportConnectFailed, "rbsim connect failed for " + simulatorEndpoint(config_)),
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
    std::string response_line;
    const bool write_ok = sendAll(fd, line);
    const bool read_ok = write_ok && recvLine(fd, &response_line);
    ::close(fd);
    if (!write_ok) {
        connected_ = false;
        return failedResult<RobotState>(
            backend_op,
            backendError(BackendErrorKind::TransportWriteFailed, "rbsim write failed for " + op),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    if (!read_ok) {
        connected_ = false;
        return failedResult<RobotState>(
            backend_op,
            backendError(BackendErrorKind::TransportReadFailed, "rbsim read failed for " + op),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    json response;
    try {
        response = json::parse(response_line);
    } catch (const json::exception& exc) {
        std::cerr << "[ERROR] RbsimBackend invalid JSON response: " << exc.what() << "\n";
        return failedResult<RobotState>(
            backend_op,
            backendError(BackendErrorKind::ProtocolError, exc.what()),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    if (!jsonBool(response, "ok", false)) {
        const BackendError error = protocolErrorFromResponse(op, response);
        RobotState response_state;
        const auto state_it = response.find("state");
        if (state_it != response.end() && mapState(*state_it, arm_id_, &response_state)) {
            connected_ = response_state.connection_state == RobotConnectionState::Connected;
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
