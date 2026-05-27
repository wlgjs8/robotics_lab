#include "rb_servo/robot/rbscript_tcp_backend.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "rb_servo/core/clock.hpp"

namespace rb_servo {
namespace {

constexpr uint64_t kInitialReconnectBackoffNs = 50ULL * 1000ULL * 1000ULL;
constexpr uint64_t kMaxReconnectBackoffNs = 1000ULL * 1000ULL * 1000ULL;

bool envIsOne(const char* name) {
    const char* value = std::getenv(name);
    return value && std::string(value) == "1";
}

bool finiteJointArray(const JointArray& joints) {
    return std::all_of(joints.begin(), joints.end(), [](double value) {
        return std::isfinite(value);
    });
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

RobotState basicState(ArmId arm_id, bool connected) {
    RobotState state;
    state.arm_id = arm_id;
    state.host_time_ns = nowSteadyNs();
    state.connection_state = connected
        ? RobotConnectionState::Connected
        : RobotConnectionState::Disconnected;
    state.has_valid_joint_state = false;
    state.servo_enabled = false;
    state.lifecycle_state = connected
        ? "rbscript_tcp_data_port_not_implemented"
        : "disconnected";
    return state;
}

RobotState stateFromParsedData(ArmId arm_id, const RbscriptDataState& parsed) {
    RobotState state;
    state.arm_id = arm_id;
    state.host_time_ns = nowSteadyNs();
    state.robot_time_ns = parsed.robot_time_ns;
    state.q_actual_deg = parsed.q_actual_deg;
    state.q_target_deg = parsed.q_target_deg;
    state.has_valid_joint_state = true;
    state.connection_state = RobotConnectionState::Connected;
    state.servo_enabled = parsed.servo_enabled;
    state.has_error = parsed.has_error;
    state.error_code = parsed.error_code;
    state.lifecycle_state = parsed.lifecycle_state.empty()
        ? "rbscript_tcp_data_valid"
        : parsed.lifecycle_state;
    return state;
}

timeval timeoutValue(double seconds) {
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(seconds);
    tv.tv_usec = static_cast<suseconds_t>((seconds - static_cast<double>(tv.tv_sec)) * 1000000.0);
    return tv;
}

void setCloseOnExec(int fd) {
    const int flags = ::fcntl(fd, F_GETFD, 0);
    if (flags >= 0) {
        (void)::fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
    }
}

bool setNonblocking(int fd, bool enabled) {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) return false;
    int next = flags;
    if (enabled) {
        next |= O_NONBLOCK;
    } else {
        next &= ~O_NONBLOCK;
    }
    return ::fcntl(fd, F_SETFL, next) == 0;
}

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

BackendError errnoError(BackendErrorKind kind, const std::string& prefix, const std::string& name) {
    return backendError(
        kind,
        prefix + ": " + std::strerror(errno),
        std::to_string(errno),
        name
    );
}

class CommandTcpClient {
public:
    CommandTcpClient(std::string host, int port)
        : host_(std::move(host)), port_(port) {}

    ~CommandTcpClient() {
        closeSocket();
    }

    BackendError connectIfNeeded(double timeout_sec) {
        if (fd_ >= 0) return noBackendError();

        const uint64_t now_ns = nowSteadyNs();
        if (next_connect_attempt_ns_ != 0 && now_ns < next_connect_attempt_ns_) {
            counters_.connect_attempts_suppressed_total += 1;
            counters_.last_transport_error_kind = BackendErrorKind::TransportConnectFailed;
            return backendError(
                BackendErrorKind::TransportConnectFailed,
                "rbscript_tcp connect attempt suppressed by reconnect backoff",
                "",
                "rbscript_tcp_connect_backoff"
            );
        }

        counters_.connect_attempts_total += 1;

        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;
        addrinfo* results = nullptr;
        const std::string port_text = std::to_string(port_);
        const int gai = ::getaddrinfo(host_.c_str(), port_text.c_str(), &hints, &results);
        if (gai != 0 || results == nullptr) {
            counters_.connect_failures_total += 1;
            counters_.last_connect_error_name = "rbscript_tcp_resolve_failed";
            counters_.last_connect_error_message = ::gai_strerror(gai);
            scheduleBackoff();
            return backendError(
                BackendErrorKind::TransportConnectFailed,
                "rbscript_tcp failed to resolve command endpoint " + host_ + ":" + port_text +
                    ": " + ::gai_strerror(gai),
                std::to_string(gai),
                "rbscript_tcp_resolve_failed"
            );
        }

        BackendError last_error = backendError(
            BackendErrorKind::TransportConnectFailed,
            "rbscript_tcp connect failed",
            "",
            "rbscript_tcp_connect_failed"
        );
        for (addrinfo* item = results; item != nullptr; item = item->ai_next) {
            if (!item->ai_addr || item->ai_addrlen <= 0) continue;
            const int fd = ::socket(item->ai_family, item->ai_socktype, item->ai_protocol);
            if (fd < 0) {
                last_error = errnoError(
                    BackendErrorKind::TransportConnectFailed,
                    "rbscript_tcp socket creation failed",
                    "rbscript_tcp_socket_failed"
                );
                continue;
            }
            setCloseOnExec(fd);
            configureSocket(fd, timeout_sec);
            if (!connectFd(fd, item->ai_addr, item->ai_addrlen, timeout_sec, &last_error)) {
                ::close(fd);
                continue;
            }
            fd_ = fd;
            reconnect_backoff_ns_ = kInitialReconnectBackoffNs;
            next_connect_attempt_ns_ = 0;
            counters_.connections_opened_total += 1;
            if (has_connected_before_) counters_.reconnects_total += 1;
            has_connected_before_ = true;
            ::freeaddrinfo(results);
            return noBackendError();
        }
        ::freeaddrinfo(results);
        counters_.connect_failures_total += 1;
        counters_.last_connect_error_name = last_error.name;
        counters_.last_connect_error_message = last_error.message;
        counters_.last_transport_error_kind = BackendErrorKind::TransportConnectFailed;
        scheduleBackoff();
        return last_error;
    }

    BackendError sendCommand(const std::string& command, double timeout_sec) {
        counters_.command_send_total += 1;
        return sendRaw(command, timeout_sec);
    }

    BackendError sendRaw(const std::string& command, double timeout_sec) {
        const BackendError connect_error = connectIfNeeded(timeout_sec);
        if (connect_error.kind != BackendErrorKind::None) return connect_error;
        setSocketTimeouts(timeout_sec);
        const BackendError write_error = sendAll(command);
        if (write_error.kind != BackendErrorKind::None) {
            closeForTransportError(write_error.kind);
        }
        return write_error;
    }

    BackendError readLine(double timeout_sec, std::string* response) {
        setSocketTimeouts(timeout_sec);
        BackendError read_error = recvLine(response, "data response");
        if (read_error.kind != BackendErrorKind::None) {
            closeForTransportError(read_error.kind);
        }
        return read_error;
    }

    BackendError readAck(double timeout_sec, std::string* response) {
        setSocketTimeouts(timeout_sec);
        BackendError read_error = recvLine(response, "command ACK");
        if (read_error.kind != BackendErrorKind::None) {
            closeForTransportError(read_error.kind);
            counters_.ack_error_total += 1;
            return read_error;
        }
        const std::string text = lower(*response);
        if (text.find("the command was executed") != std::string::npos ||
            text.find("command was executed") != std::string::npos ||
            text.find("executed") != std::string::npos) {
            counters_.ack_success_total += 1;
            return noBackendError();
        }
        counters_.ack_error_total += 1;
        if (text.find("not allowed") != std::string::npos ||
            text.find("error") != std::string::npos ||
            text.find("fail") != std::string::npos) {
            return backendError(
                BackendErrorKind::ControllerRejected,
                "rbscript_tcp controller rejected command: " + *response,
                "",
                "rbscript_tcp_controller_rejected"
            );
        }
        return backendError(
            BackendErrorKind::ProtocolError,
            "rbscript_tcp could not classify controller ACK: " + *response,
            "",
            "rbscript_tcp_unrecognized_ack"
        );
    }

    bool connected() const {
        return fd_ >= 0;
    }

    void closeForTransportError(BackendErrorKind kind) {
        counters_.last_transport_error_kind = kind;
        closeSocket();
        scheduleBackoff();
    }

    void closeSocket() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    RbscriptTcpTransportCounters counters() const {
        RbscriptTcpTransportCounters out = counters_;
        out.next_connect_attempt_ns = next_connect_attempt_ns_;
        if (next_connect_attempt_ns_ > 0) {
            const uint64_t now_ns = nowSteadyNs();
            if (next_connect_attempt_ns_ > now_ns) {
                out.next_connect_attempt_delay_ms = (next_connect_attempt_ns_ - now_ns) / 1'000'000ULL;
            }
        }
        return out;
    }

private:
    void configureSocket(int fd, double timeout_sec) {
        const int one = 1;
        (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        (void)::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
        const timeval tv = timeoutValue(timeout_sec);
        (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    void setSocketTimeouts(double timeout_sec) {
        if (fd_ < 0) return;
        const timeval tv = timeoutValue(timeout_sec);
        (void)::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        (void)::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    bool connectFd(int fd, const sockaddr* address, socklen_t len, double timeout_sec, BackendError* error) {
        if (!setNonblocking(fd, true)) {
            *error = errnoError(
                BackendErrorKind::TransportConnectFailed,
                "rbscript_tcp failed to set nonblocking connect",
                "rbscript_tcp_nonblocking_failed"
            );
            return false;
        }
        int rc = ::connect(fd, address, len);
        if (rc == 0) {
            (void)setNonblocking(fd, false);
            return true;
        }
        if (errno != EINPROGRESS) {
            *error = errnoError(
                BackendErrorKind::TransportConnectFailed,
                "rbscript_tcp connect failed",
                "rbscript_tcp_connect_failed"
            );
            return false;
        }
        fd_set write_set;
        FD_ZERO(&write_set);
        FD_SET(fd, &write_set);
        timeval tv = timeoutValue(timeout_sec);
        rc = ::select(fd + 1, nullptr, &write_set, nullptr, &tv);
        if (rc == 0) {
            *error = backendError(
                BackendErrorKind::TransportTimeout,
                "rbscript_tcp connect timed out",
                "",
                "rbscript_tcp_connect_timeout"
            );
            return false;
        }
        if (rc < 0) {
            *error = errnoError(
                BackendErrorKind::TransportConnectFailed,
                "rbscript_tcp connect select failed",
                "rbscript_tcp_connect_select_failed"
            );
            return false;
        }
        int socket_error = 0;
        socklen_t socket_error_len = sizeof(socket_error);
        if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_len) != 0 || socket_error != 0) {
            errno = socket_error;
            *error = errnoError(
                BackendErrorKind::TransportConnectFailed,
                "rbscript_tcp connect failed",
                "rbscript_tcp_connect_failed"
            );
            return false;
        }
        (void)setNonblocking(fd, false);
        return true;
    }

    BackendError sendAll(const std::string& command) {
        std::size_t offset = 0;
        while (offset < command.size()) {
#ifdef MSG_NOSIGNAL
            constexpr int kSendFlags = MSG_NOSIGNAL;
#else
            constexpr int kSendFlags = 0;
#endif
            const ssize_t n = ::send(fd_, command.data() + offset, command.size() - offset, kSendFlags);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    return backendError(
                        BackendErrorKind::TransportTimeout,
                        "rbscript_tcp timed out sending command",
                        std::to_string(errno),
                        "rbscript_tcp_send_timeout"
                    );
                }
                return errnoError(
                    BackendErrorKind::TransportWriteFailed,
                    "rbscript_tcp failed to send command",
                    "rbscript_tcp_send_failed"
                );
            }
            if (n == 0) {
                return backendError(
                    BackendErrorKind::TransportWriteFailed,
                    "rbscript_tcp socket closed during send",
                    "",
                    "rbscript_tcp_send_closed"
                );
            }
            counters_.write_syscalls_total += 1;
            offset += static_cast<std::size_t>(n);
        }
        return noBackendError();
    }

    BackendError recvLine(std::string* line, const std::string& context) {
        line->clear();
        char ch = '\0';
        while (line->size() < 4096) {
            const ssize_t n = ::recv(fd_, &ch, 1, 0);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    return backendError(
                        BackendErrorKind::TransportTimeout,
                        "rbscript_tcp timed out waiting for " + context,
                        std::to_string(errno),
                        context == "command ACK" ? "rbscript_tcp_ack_timeout" : "rbscript_tcp_data_timeout"
                    );
                }
                return errnoError(
                    BackendErrorKind::TransportReadFailed,
                    "rbscript_tcp failed to read " + context,
                    context == "command ACK" ? "rbscript_tcp_ack_read_failed" : "rbscript_tcp_data_read_failed"
                );
            }
            if (n == 0) {
                return backendError(
                    BackendErrorKind::TransportReadFailed,
                    "rbscript_tcp socket closed before " + context,
                    "",
                    context == "command ACK" ? "rbscript_tcp_ack_closed" : "rbscript_tcp_data_closed"
                );
            }
            counters_.read_syscalls_total += 1;
            if (ch == '\n') return noBackendError();
            if (ch != '\r') line->push_back(ch);
        }
        return backendError(
            BackendErrorKind::ProtocolError,
            "rbscript_tcp " + context + " exceeded maximum line length",
            "",
            context == "command ACK" ? "rbscript_tcp_ack_too_long" : "rbscript_tcp_data_too_long"
        );
    }

    void scheduleBackoff() {
        next_connect_attempt_ns_ = nowSteadyNs() + reconnect_backoff_ns_;
        reconnect_backoff_ns_ = std::min<uint64_t>(reconnect_backoff_ns_ * 2ULL, kMaxReconnectBackoffNs);
    }

    std::string host_;
    int port_ = 0;
    int fd_ = -1;
    bool has_connected_before_ = false;
    uint64_t reconnect_backoff_ns_ = kInitialReconnectBackoffNs;
    uint64_t next_connect_attempt_ns_ = 0;
    RbscriptTcpTransportCounters counters_;
};

}  // namespace

namespace {

bool jsonJointArray(const nlohmann::json& value, JointArray* out) {
    if (!value.is_array() || value.size() != static_cast<std::size_t>(kDof)) {
        return false;
    }
    JointArray joints{};
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (!value.at(i).is_number()) return false;
        joints[i] = value.at(i).get<double>();
        if (!std::isfinite(joints[i])) return false;
    }
    *out = joints;
    return true;
}

}  // namespace

std::string formatRbscriptServoJ(const JointArray& q_target_deg, const BackendConfig& config) {
    std::ostringstream out;
    out << std::setprecision(12);
    out << "move_servo_j(jnt[";
    for (std::size_t i = 0; i < q_target_deg.size(); ++i) {
        if (i > 0) out << ",";
        out << q_target_deg[i];
    }
    out << "],"
        << config.script_t1_sec << ","
        << config.script_t2_sec << ","
        << config.script_gain << ","
        << config.script_alpha << ")\n";
    return out.str();
}

std::string formatRbscriptReqdata() {
    return "reqdata\n";
}

RbscriptDataParseResult parseRbscriptDataPayload(const std::string& payload) {
    RbscriptDataParseResult result;
    if (payload.empty()) {
        result.error = backendError(
            BackendErrorKind::ProtocolError,
            "rbscript_tcp data response was empty",
            "",
            "rbscript_tcp_data_empty"
        );
        return result;
    }

    nlohmann::json doc;
    try {
        doc = nlohmann::json::parse(payload);
    } catch (const nlohmann::json::exception& ex) {
        result.error = backendError(
            BackendErrorKind::ProtocolError,
            std::string("rbscript_tcp data response is not a recognized JSON state fixture: ") + ex.what(),
            "",
            "rbscript_tcp_data_unrecognized_format"
        );
        return result;
    }

    if (!doc.is_object() ||
        !doc.contains("schema") ||
        !doc.at("schema").is_string() ||
        doc.at("schema").get<std::string>() != "rbscript_tcp_state_v1") {
        result.error = backendError(
            BackendErrorKind::UnsupportedSchema,
            "rbscript_tcp data response schema is not supported; refusing to guess Rainbow binary offsets",
            "",
            "rbscript_tcp_data_unsupported_schema"
        );
        return result;
    }

    JointArray q_actual{};
    if (!doc.contains("q_actual_deg") || !jsonJointArray(doc.at("q_actual_deg"), &q_actual)) {
        result.error = backendError(
            BackendErrorKind::InvalidJointState,
            "rbscript_tcp data response lacks six finite q_actual_deg values in degrees",
            "",
            "rbscript_tcp_data_invalid_q_actual"
        );
        return result;
    }

    JointArray q_target = q_actual;
    bool has_q_target = false;
    if (doc.contains("q_target_deg")) {
        if (!jsonJointArray(doc.at("q_target_deg"), &q_target)) {
            result.error = backendError(
                BackendErrorKind::InvalidJointState,
                "rbscript_tcp data response has invalid q_target_deg values",
                "",
                "rbscript_tcp_data_invalid_q_target"
            );
            return result;
        }
        has_q_target = true;
    }

    double robot_time_sec = 0.0;
    if (doc.contains("robot_time_sec")) {
        if (!doc.at("robot_time_sec").is_number()) {
            result.error = backendError(
                BackendErrorKind::ProtocolError,
                "rbscript_tcp data response robot_time_sec is not numeric",
                "",
                "rbscript_tcp_data_invalid_robot_time"
            );
            return result;
        }
        robot_time_sec = doc.at("robot_time_sec").get<double>();
        if (!std::isfinite(robot_time_sec) || robot_time_sec < 0.0) {
            result.error = backendError(
                BackendErrorKind::ProtocolError,
                "rbscript_tcp data response robot_time_sec is not finite and nonnegative",
                "",
                "rbscript_tcp_data_invalid_robot_time"
            );
            return result;
        }
    }

    int error_code = 0;
    if (doc.contains("error_code")) {
        if (!doc.at("error_code").is_number_integer()) {
            result.error = backendError(
                BackendErrorKind::ProtocolError,
                "rbscript_tcp data response error_code is not an integer",
                "",
                "rbscript_tcp_data_invalid_error_code"
            );
            return result;
        }
        error_code = doc.at("error_code").get<int>();
    }
    bool has_error = error_code != 0;
    if (doc.contains("has_error")) {
        if (!doc.at("has_error").is_boolean()) {
            result.error = backendError(
                BackendErrorKind::ProtocolError,
                "rbscript_tcp data response has_error is not boolean",
                "",
                "rbscript_tcp_data_invalid_has_error"
            );
            return result;
        }
        has_error = doc.at("has_error").get<bool>();
    }
    bool servo_enabled = false;
    if (doc.contains("servo_enabled")) {
        if (!doc.at("servo_enabled").is_boolean()) {
            result.error = backendError(
                BackendErrorKind::ProtocolError,
                "rbscript_tcp data response servo_enabled is not boolean",
                "",
                "rbscript_tcp_data_invalid_servo_enabled"
            );
            return result;
        }
        servo_enabled = doc.at("servo_enabled").get<bool>();
    }

    result.ok = true;
    result.error = noBackendError();
    result.state.q_actual_deg = q_actual;
    result.state.q_target_deg = q_target;
    result.state.has_q_target_deg = has_q_target;
    result.state.robot_time_ns = static_cast<uint64_t>(robot_time_sec * 1'000'000'000.0);
    result.state.servo_enabled = servo_enabled;
    result.state.has_error = has_error;
    result.state.error_code = error_code;
    if (doc.contains("lifecycle_state") && doc.at("lifecycle_state").is_string()) {
        result.state.lifecycle_state = doc.at("lifecycle_state").get<std::string>();
    } else {
        result.state.lifecycle_state = has_error
            ? std::string("rbscript_tcp_robot_error")
            : (servo_enabled ? std::string("rbscript_tcp_servo_enabled")
                             : std::string("rbscript_tcp_state_valid_not_servo_enabled"));
    }
    return result;
}

struct RbscriptTcpBackend::Impl {
    ArmId arm_id = ArmId::Left;
    BackendConfig config;
    std::unique_ptr<CommandTcpClient> command_client;
    std::unique_ptr<CommandTcpClient> data_client;
    RbscriptTcpTransportCounters data_counters;
};

RbscriptTcpBackend::RbscriptTcpBackend(ArmId arm_id, const BackendConfig& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->arm_id = arm_id;
    impl_->config = config;
    impl_->command_client = std::make_unique<CommandTcpClient>(config.ip, config.command_port);
    impl_->data_client = std::make_unique<CommandTcpClient>(config.ip, config.data_port);
}

RbscriptTcpBackend::~RbscriptTcpBackend() = default;

BackendResult<RobotState> RbscriptTcpBackend::connect() {
    const uint64_t start = nowSteadyNs();
    if (impl_->config.run_mode == RunMode::Real) {
        if (!envIsOne("RB_ALLOW_REAL_ROBOT") || !envIsOne("RB_ALLOW_RBSCRIPT_TCP")) {
            return failedResult<RobotState>(
                BackendOp::Connect,
                backendError(
                    BackendErrorKind::SuppressedByPolicy,
                    "Refusing rbscript_tcp real connection. Set RB_ALLOW_REAL_ROBOT=1 and RB_ALLOW_RBSCRIPT_TCP=1.",
                    "",
                    "rbscript_tcp_real_connection_gate_closed"
                ),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
    }
    if (impl_->config.ip.empty()) {
        return failedResult<RobotState>(
            BackendOp::Connect,
            backendError(
                BackendErrorKind::WrongEndpoint,
                "RbscriptTcpBackend requires a non-empty controller ip",
                "",
                "rbscript_tcp_missing_controller_ip"
            ),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    const BackendError error = impl_->command_client->connectIfNeeded(impl_->config.connect_timeout_sec);
    if (error.kind != BackendErrorKind::None) {
        return failedResult<RobotState>(
            BackendOp::Connect,
            error,
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    return okResult(
        BackendOp::Connect,
        basicState(impl_->arm_id, true),
        makeBackendTiming(start, nowSteadyNs())
    );
}

BackendResult<RobotState> RbscriptTcpBackend::initialize() {
    const uint64_t start = nowSteadyNs();
    if (!isConnected()) {
        return failedResult<RobotState>(
            BackendOp::Initialize,
            backendError(
                BackendErrorKind::RobotDisconnected,
                "rbscript_tcp backend is not connected",
                "",
                "rbscript_tcp_not_connected"
            ),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    return okResult(
        BackendOp::Initialize,
        basicState(impl_->arm_id, true),
        makeBackendTiming(start, nowSteadyNs())
    );
}

BackendResult<RobotState> RbscriptTcpBackend::readState() {
    const uint64_t start = nowSteadyNs();
    if (impl_->config.run_mode == RunMode::Real) {
        if (!envIsOne("RB_ALLOW_REAL_ROBOT") || !envIsOne("RB_ALLOW_RBSCRIPT_TCP")) {
            return failedResult<RobotState>(
                BackendOp::ReadState,
                backendError(
                    BackendErrorKind::SuppressedByPolicy,
                    "Refusing rbscript_tcp real state connection. Set RB_ALLOW_REAL_ROBOT=1 and RB_ALLOW_RBSCRIPT_TCP=1.",
                    "",
                    "rbscript_tcp_real_state_gate_closed"
                ),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
    }
    if (impl_->config.ip.empty()) {
        return failedResult<RobotState>(
            BackendOp::ReadState,
            backendError(
                BackendErrorKind::WrongEndpoint,
                "RbscriptTcpBackend requires a non-empty controller ip for data port state",
                "",
                "rbscript_tcp_missing_data_controller_ip"
            ),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    impl_->data_counters.data_requests_total += 1;
    const std::string request = formatRbscriptReqdata();
    BackendError send_error = noBackendError();
    {
        send_error = impl_->data_client->sendRaw(request, impl_->config.read_timeout_sec);
    }
    if (send_error.kind != BackendErrorKind::None) {
        if (send_error.kind == BackendErrorKind::TransportTimeout) {
            impl_->data_counters.data_timeouts_total += 1;
        }
        return failedResult<RobotState>(
            BackendOp::ReadState,
            send_error,
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    std::string response;
    BackendError read_error = impl_->data_client->readLine(impl_->config.read_timeout_sec, &response);
    if (read_error.kind != BackendErrorKind::None) {
        if (read_error.kind == BackendErrorKind::TransportTimeout) {
            impl_->data_counters.data_timeouts_total += 1;
        }
        return failedResult<RobotState>(
            BackendOp::ReadState,
            read_error,
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    RbscriptDataParseResult parsed = parseRbscriptDataPayload(response);
    if (!parsed.ok) {
        impl_->data_counters.data_parse_failures_total += 1;
        return failedResult<RobotState>(
            BackendOp::ReadState,
            parsed.error,
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    impl_->data_counters.data_success_total += 1;
    return okResult(
        BackendOp::ReadState,
        stateFromParsedData(impl_->arm_id, parsed.state),
        makeBackendTiming(start, nowSteadyNs())
    );
}

SendServoJResult RbscriptTcpBackend::sendServoJ(const SendServoJRequest& request) {
    const uint64_t start = nowSteadyNs();
    if (impl_->config.run_mode == RunMode::Real) {
        if (!envIsOne("RB_ALLOW_REAL_MOTION") || !envIsOne("RB_ALLOW_RBSCRIPT_TCP_MOTION")) {
            return rejectedSend(
                request,
                backendError(
                    BackendErrorKind::SuppressedByPolicy,
                    "RbscriptTcpBackend refused servo_j without RB_ALLOW_REAL_MOTION=1 and RB_ALLOW_RBSCRIPT_TCP_MOTION=1",
                    "",
                    "rbscript_tcp_motion_gate_closed"
                ),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
    }
    if (!finiteJointArray(request.q_target_deg)) {
        return rejectedSend(
            request,
            backendError(
                BackendErrorKind::InvalidTarget,
                "RbscriptTcpBackend refused non-finite servo_j target",
                "",
                "rbscript_tcp_invalid_servo_j_target"
            ),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    const std::string command = formatRbscriptServoJ(request.q_target_deg, impl_->config);
    BackendError send_error = impl_->command_client->sendCommand(command, impl_->config.command_timeout_sec);
    if (send_error.kind != BackendErrorKind::None) {
        return rejectedSend(request, send_error, makeBackendTiming(start, nowSteadyNs()));
    }
    if (impl_->config.disable_waiting_ack) {
        SendServoJResult result = acceptedSend(request, makeBackendTiming(start, nowSteadyNs()));
        result.ack_policy = BackendAckPolicy::Disabled;
        result.ack_observed = false;
        return result;
    }

    std::string response;
    BackendError ack_error = impl_->command_client->readAck(impl_->config.command_timeout_sec, &response);
    if (ack_error.kind != BackendErrorKind::None) {
        SendServoJResult result = rejectedSend(request, ack_error, makeBackendTiming(start, nowSteadyNs()));
        result.ack_policy = BackendAckPolicy::Wait;
        result.ack_observed = ack_error.kind == BackendErrorKind::ControllerRejected ||
            ack_error.kind == BackendErrorKind::ProtocolError;
        return result;
    }
    SendServoJResult result = acceptedSend(request, makeBackendTiming(start, nowSteadyNs()));
    result.ack_policy = BackendAckPolicy::Wait;
    result.ack_observed = true;
    return result;
}

BackendResult<RobotState> RbscriptTcpBackend::stop() {
    const uint64_t start = nowSteadyNs();
    return failedResult<RobotState>(
        BackendOp::Stop,
        backendError(
            BackendErrorKind::DependencyUnavailable,
            "No verified rbscript_tcp stop/hold command is wired; operator intervention is required.",
            "",
            "rbscript_tcp_stop_unverified"
        ),
        makeBackendTiming(start, nowSteadyNs())
    );
}

BackendResult<RobotState> RbscriptTcpBackend::resetFault() {
    const uint64_t start = nowSteadyNs();
    return failedResult<RobotState>(
        BackendOp::ResetFault,
        backendError(
            BackendErrorKind::DependencyUnavailable,
            "No verified rbscript_tcp fault reset command is wired; operator intervention is required.",
            "",
            "rbscript_tcp_reset_fault_unverified"
        ),
        makeBackendTiming(start, nowSteadyNs())
    );
}

bool RbscriptTcpBackend::isConnected() const {
    return impl_->command_client && impl_->command_client->connected();
}

ArmId RbscriptTcpBackend::armId() const {
    return impl_->arm_id;
}

std::string RbscriptTcpBackend::name() const {
    return impl_->config.name;
}

std::optional<BackendTransportTelemetry> RbscriptTcpBackend::transportTelemetry() const {
    const RbscriptTcpTransportCounters counters = transportCounters();
    BackendTransportTelemetry out;
    out.connect_attempts_total = counters.connect_attempts_total;
    out.connect_failures_total = counters.connect_failures_total;
    out.connect_attempts_suppressed_total = counters.connect_attempts_suppressed_total;
    out.connections_opened_total = counters.connections_opened_total;
    out.reconnects_total = counters.reconnects_total;
    out.requests_total = counters.command_send_total + counters.data_requests_total;
    out.read_syscalls_total = counters.read_syscalls_total;
    out.write_syscalls_total = counters.write_syscalls_total;
    out.last_connect_error_name = counters.last_connect_error_name;
    out.last_connect_error_message = counters.last_connect_error_message;
    out.next_connect_attempt_ns = counters.next_connect_attempt_ns;
    out.next_connect_attempt_delay_ms = counters.next_connect_attempt_delay_ms;
    if (counters.last_transport_error_kind.has_value()) {
        out.last_transport_error_kind = toString(*counters.last_transport_error_kind);
    }
    return out;
}

RbscriptTcpTransportCounters RbscriptTcpBackend::transportCounters() const {
    RbscriptTcpTransportCounters out =
        impl_->command_client ? impl_->command_client->counters() : RbscriptTcpTransportCounters{};
    if (impl_->data_client) {
        const RbscriptTcpTransportCounters data = impl_->data_client->counters();
        out.connections_opened_total += data.connections_opened_total;
        out.reconnects_total += data.reconnects_total;
        out.connect_failures_total += data.connect_failures_total;
        out.connect_attempts_suppressed_total += data.connect_attempts_suppressed_total;
        out.connect_attempts_total += data.connect_attempts_total;
        out.read_syscalls_total += data.read_syscalls_total;
        out.write_syscalls_total += data.write_syscalls_total;
        if (!data.last_connect_error_name.empty()) {
            out.last_connect_error_name = data.last_connect_error_name;
            out.last_connect_error_message = data.last_connect_error_message;
        }
        if (data.next_connect_attempt_ns > out.next_connect_attempt_ns) {
            out.next_connect_attempt_ns = data.next_connect_attempt_ns;
            out.next_connect_attempt_delay_ms = data.next_connect_attempt_delay_ms;
        }
        if (data.last_transport_error_kind.has_value()) {
            out.last_transport_error_kind = data.last_transport_error_kind;
        }
    }
    out.data_requests_total = impl_->data_counters.data_requests_total;
    out.data_success_total = impl_->data_counters.data_success_total;
    out.data_parse_failures_total = impl_->data_counters.data_parse_failures_total;
    out.data_timeouts_total = impl_->data_counters.data_timeouts_total;
    return out;
}

}  // namespace rb_servo
