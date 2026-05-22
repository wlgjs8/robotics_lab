#include "rb_servo/robot/rbsim_backend.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
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

}  // namespace

RbsimBackend::RbsimBackend(ArmId arm_id, const BackendConfig& config)
    : arm_id_(arm_id), config_(config) {}

bool RbsimBackend::connect() {
    RobotState state;
    connected_ = controlRequest("connect", json::object(), &state) &&
                 state.connection_state == RobotConnectionState::Connected;
    return connected_;
}

bool RbsimBackend::initialize() {
    json params;
    params["enable_servo"] = true;
    RobotState state;
    return controlRequest("initialize", params, &state) &&
           state.connection_state == RobotConnectionState::Connected &&
           state.has_valid_joint_state &&
           !state.has_error;
}

bool RbsimBackend::readState(RobotState& out_state) {
    return controlRequest("read_state", json::object(), &out_state);
}

bool RbsimBackend::sendServoJ(const JointArray& q_target_deg) {
    json params;
    params["q_target_deg"] = jointArrayJson(q_target_deg);
    return controlRequest("send_servo_j", params, nullptr);
}

bool RbsimBackend::stop() {
    return controlRequest("stop", json::object(), nullptr);
}

bool RbsimBackend::resetFault() {
    RobotState reset_state;
    if (!controlRequest("reset_fault", json::object(), &reset_state)) {
        return false;
    }
    return initialize();
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

bool RbsimBackend::controlRequest(const std::string& op, const json& params, RobotState* out_state) {
    const TcpEndpoint endpoint = parseTcpEndpoint(config_.rbsim_control_endpoint);
    const int fd = openTcpConnection(endpoint, timeoutForOperation(config_, op));
    if (fd < 0) {
        connected_ = false;
        return false;
    }

    json request;
    request["schema_version"] = "rbsim.v1";
    request["request_id"] = config_.name + "-" + std::to_string(++request_seq_);
    request["op"] = op;
    request["arm"] = toString(arm_id_);
    request["params"] = params;

    const std::string line = request.dump() + "\n";
    std::string response_line;
    const bool io_ok = sendAll(fd, line) && recvLine(fd, &response_line);
    ::close(fd);
    if (!io_ok) {
        connected_ = false;
        return false;
    }

    json response;
    try {
        response = json::parse(response_line);
    } catch (const json::exception& exc) {
        std::cerr << "[ERROR] RbsimBackend invalid JSON response: " << exc.what() << "\n";
        return false;
    }

    if (!jsonBool(response, "ok", false)) {
        const auto error_it = response.find("error");
        if (error_it != response.end() && error_it->is_object()) {
            const std::string name = error_it->value("name", "unknown");
            const int code = error_it->value("code", 0);
            std::cerr << "[WARN] RbsimBackend " << op << " failed for "
                      << toString(arm_id_) << ": " << name << " (" << code << ")\n";
        }
        return false;
    }

    const auto state_it = response.find("state");
    if (out_state) {
        if (state_it == response.end() || !mapState(*state_it, arm_id_, out_state)) {
            std::cerr << "[ERROR] RbsimBackend " << op << " response missing valid state\n";
            return false;
        }
        connected_ = out_state->connection_state == RobotConnectionState::Connected;
    } else if (state_it != response.end() && state_it->is_object()) {
        RobotState mapped;
        if (mapState(*state_it, arm_id_, &mapped)) {
            connected_ = mapped.connection_state == RobotConnectionState::Connected;
        }
    }

    return true;
}

}  // namespace rb_servo
