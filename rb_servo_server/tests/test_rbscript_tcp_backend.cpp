#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <stdexcept>
#include <thread>

#include "rb_servo/robot/rbscript_tcp_backend.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

class EnvGuard {
public:
    EnvGuard(const char* name, const char* value) : name_(name) {
        const char* previous = std::getenv(name);
        if (previous) {
            had_previous_ = true;
            previous_ = previous;
        }
        if (value) {
            ::setenv(name, value, 1);
        } else {
            ::unsetenv(name);
        }
    }

    ~EnvGuard() {
        if (had_previous_) {
            ::setenv(name_.c_str(), previous_.c_str(), 1);
        } else {
            ::unsetenv(name_.c_str());
        }
    }

private:
    std::string name_;
    bool had_previous_ = false;
    std::string previous_;
};

class FakeCommandServer {
public:
    explicit FakeCommandServer(std::optional<std::string> ack, int hold_ms = 0)
        : ack_(std::move(ack)), hold_ms_(hold_ms) {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0) {
            throw std::runtime_error("socket failed");
        }
        const int one = 1;
        (void)::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            throw std::runtime_error(std::string("bind failed: ") + std::strerror(errno));
        }
        if (::listen(listen_fd_, 1) != 0) {
            throw std::runtime_error("listen failed");
        }
        socklen_t len = sizeof(addr);
        if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
            throw std::runtime_error("getsockname failed");
        }
        port_ = ntohs(addr.sin_port);
        thread_ = std::thread([this]() { run(); });
    }

    ~FakeCommandServer() {
        if (listen_fd_ >= 0) {
            ::shutdown(listen_fd_, SHUT_RDWR);
            ::close(listen_fd_);
            listen_fd_ = -1;
        }
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    int port() const { return port_; }

    std::string command() {
        if (thread_.joinable()) {
            thread_.join();
        }
        return command_;
    }

private:
    void run() {
        const int client_fd = ::accept(listen_fd_, nullptr, nullptr);
        if (client_fd < 0) return;
        char ch = '\0';
        while (command_.size() < 4096) {
            const ssize_t n = ::recv(client_fd, &ch, 1, 0);
            if (n <= 0) break;
            command_.push_back(ch);
            if (ch == '\n') break;
        }
        if (hold_ms_ > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms_));
        }
        if (ack_.has_value()) {
            const std::string ack_line = *ack_ + "\n";
            (void)::send(client_fd, ack_line.data(), ack_line.size(), 0);
        }
        ::shutdown(client_fd, SHUT_RDWR);
        ::close(client_fd);
    }

    int listen_fd_ = -1;
    int port_ = 0;
    std::optional<std::string> ack_;
    int hold_ms_ = 0;
    std::thread thread_;
    std::string command_;
};

rb_servo::BackendConfig testConfig(int port) {
    rb_servo::BackendConfig cfg;
    cfg.backend_type = rb_servo::BackendType::RbscriptTcp;
    cfg.run_mode = rb_servo::RunMode::Mock;
    cfg.name = "fake_rbscript_tcp";
    cfg.ip = "127.0.0.1";
    cfg.command_port = port;
    cfg.data_port = 5001;
    cfg.command_timeout_sec = 0.05;
    cfg.read_timeout_sec = 0.05;
    cfg.connect_timeout_sec = 0.2;
    cfg.disable_waiting_ack = false;
    cfg.script_t1_sec = 0.008;
    cfg.script_t2_sec = 0.05;
    cfg.script_gain = 1.0;
    cfg.script_alpha = 0.5;
    return cfg;
}

rb_servo::BackendConfig testDataConfig(int data_port) {
    rb_servo::BackendConfig cfg = testConfig(1);
    cfg.data_port = data_port;
    return cfg;
}

rb_servo::SendServoJRequest request() {
    rb_servo::SendServoJRequest req;
    req.q_target_deg = {1.0, -2.5, 3.25, 4.0, -5.0, 6.125};
    req.command_seq = 42;
    return req;
}

bool testFormatServoJ() {
    const rb_servo::BackendConfig cfg = testConfig(5000);
    const std::string command = rb_servo::formatRbscriptServoJ(request().q_target_deg, cfg);
    RB_CHECK(command.find("move_servo_j(jnt[1,-2.5,3.25,4,-5,6.125],0.008,0.05,1,0.5)") == 0);
    RB_CHECK(command.back() == '\n');
    return true;
}

std::string validStateFixture() {
    return R"({"schema":"rbscript_tcp_state_v1","robot_time_sec":1.25,"q_actual_deg":[1,2,3,4,5,6],"q_target_deg":[1.1,2.1,3.1,4.1,5.1,6.1],"servo_enabled":true,"has_error":false,"error_code":0,"lifecycle_state":"fixture_ready"})";
}

bool testReqdataFormat() {
    RB_CHECK(rb_servo::formatRbscriptReqdata() == "reqdata\n");
    return true;
}

bool testParseDataFixture() {
    const rb_servo::RbscriptDataParseResult result =
        rb_servo::parseRbscriptDataPayload(validStateFixture());
    RB_CHECK(result.ok);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(result.state.q_actual_deg[0] == 1.0);
    RB_CHECK(result.state.q_actual_deg[5] == 6.0);
    RB_CHECK(result.state.has_q_target_deg);
    RB_CHECK(std::fabs(result.state.q_target_deg[2] - 3.1) < 1e-9);
    RB_CHECK(result.state.robot_time_ns == 1250000000ULL);
    RB_CHECK(result.state.servo_enabled);
    RB_CHECK(!result.state.has_error);
    RB_CHECK(result.state.lifecycle_state == "fixture_ready");
    return true;
}

bool testParseRejectsShortData() {
    const rb_servo::RbscriptDataParseResult result =
        rb_servo::parseRbscriptDataPayload("");
    RB_CHECK(!result.ok);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::ProtocolError);
    return true;
}

bool testParseRejectsUnknownFormat() {
    const rb_servo::RbscriptDataParseResult result =
        rb_servo::parseRbscriptDataPayload("not a documented state payload");
    RB_CHECK(!result.ok);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::ProtocolError);
    return true;
}

bool testParseRejectsUnknownSchema() {
    const rb_servo::RbscriptDataParseResult result =
        rb_servo::parseRbscriptDataPayload(R"({"schema":"rainbow_binary_blob","q_actual_deg":[1,2,3,4,5,6]})");
    RB_CHECK(!result.ok);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::UnsupportedSchema);
    return true;
}

bool testParseRejectsInvalidJointData() {
    const rb_servo::RbscriptDataParseResult result =
        rb_servo::parseRbscriptDataPayload(R"({"schema":"rbscript_tcp_state_v1","q_actual_deg":[1,2,3,4,5]})");
    RB_CHECK(!result.ok);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::InvalidJointState);

    const rb_servo::RbscriptDataParseResult nan_result =
        rb_servo::parseRbscriptDataPayload(R"({"schema":"rbscript_tcp_state_v1","q_actual_deg":[1,2,3,4,5,NaN]})");
    RB_CHECK(!nan_result.ok);
    RB_CHECK(nan_result.error.kind == rb_servo::BackendErrorKind::ProtocolError);
    return true;
}

bool testReadStateFromDataPortFixture() {
    FakeCommandServer data_server(validStateFixture());
    rb_servo::RbscriptTcpBackend backend(rb_servo::ArmId::Left, testDataConfig(data_server.port()));
    const auto result = backend.readState();
    RB_CHECK(result.ok);
    RB_CHECK(result.value.connection_state == rb_servo::RobotConnectionState::Connected);
    RB_CHECK(result.value.has_valid_joint_state);
    RB_CHECK(result.value.servo_enabled);
    RB_CHECK(!result.value.has_error);
    RB_CHECK(result.value.error_code == 0);
    RB_CHECK(result.value.lifecycle_state == "fixture_ready");
    RB_CHECK(result.value.robot_time_ns == 1250000000ULL);
    RB_CHECK(result.value.q_actual_deg[0] == 1.0);
    RB_CHECK(std::fabs(result.value.q_target_deg[5] - 6.1) < 1e-9);
    RB_CHECK(data_server.command() == "reqdata\n");
    const auto counters = backend.transportCounters();
    RB_CHECK(counters.data_requests_total == 1);
    RB_CHECK(counters.data_success_total == 1);
    RB_CHECK(counters.data_parse_failures_total == 0);
    return true;
}

bool testReadStateParseFailure() {
    FakeCommandServer data_server("not-json");
    rb_servo::RbscriptTcpBackend backend(rb_servo::ArmId::Left, testDataConfig(data_server.port()));
    const auto result = backend.readState();
    RB_CHECK(!result.ok);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::ProtocolError);
    RB_CHECK(data_server.command() == "reqdata\n");
    const auto counters = backend.transportCounters();
    RB_CHECK(counters.data_requests_total == 1);
    RB_CHECK(counters.data_success_total == 0);
    RB_CHECK(counters.data_parse_failures_total == 1);
    return true;
}

bool testReadStateOkWhenServoDisabled() {
    FakeCommandServer data_server(
        R"({"schema":"rbscript_tcp_state_v1","q_actual_deg":[-1,-2,-3,-4,-5,-6],"servo_enabled":false})"
    );
    rb_servo::RbscriptTcpBackend backend(rb_servo::ArmId::Left, testDataConfig(data_server.port()));
    const auto result = backend.readState();
    RB_CHECK(result.ok);
    RB_CHECK(result.value.has_valid_joint_state);
    RB_CHECK(!result.value.servo_enabled);
    RB_CHECK(result.value.lifecycle_state == "rbscript_tcp_state_valid_not_servo_enabled");
    RB_CHECK(result.value.q_actual_deg[0] == -1.0);
    RB_CHECK(result.value.q_target_deg[0] == -1.0);
    return true;
}

bool testReadStateTimeout() {
    FakeCommandServer data_server(std::nullopt, 200);
    rb_servo::BackendConfig cfg = testDataConfig(data_server.port());
    cfg.read_timeout_sec = 0.03;
    rb_servo::RbscriptTcpBackend backend(rb_servo::ArmId::Left, cfg);
    const auto result = backend.readState();
    RB_CHECK(!result.ok);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::TransportTimeout);
    RB_CHECK(data_server.command() == "reqdata\n");
    const auto counters = backend.transportCounters();
    RB_CHECK(counters.data_requests_total == 1);
    RB_CHECK(counters.data_timeouts_total == 1);
    return true;
}

bool testSuccessAckAccepted() {
    FakeCommandServer server("The command was executed");
    rb_servo::RbscriptTcpBackend backend(rb_servo::ArmId::Left, testConfig(server.port()));
    RB_CHECK(backend.connect().ok);
    const rb_servo::SendServoJResult result = backend.sendServoJ(request());
    RB_CHECK(result.accepted);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(result.ack_policy == rb_servo::BackendAckPolicy::Wait);
    RB_CHECK(result.ack_observed);
    RB_CHECK(server.command().find("move_servo_j(jnt[1,-2.5,3.25,4,-5,6.125]") != std::string::npos);
    const auto counters = backend.transportCounters();
    RB_CHECK(counters.command_send_total == 1);
    RB_CHECK(counters.ack_success_total == 1);
    return true;
}

bool testErrorAckRejected() {
    FakeCommandServer server("The command is not allowed");
    rb_servo::RbscriptTcpBackend backend(rb_servo::ArmId::Left, testConfig(server.port()));
    RB_CHECK(backend.connect().ok);
    const rb_servo::SendServoJResult result = backend.sendServoJ(request());
    RB_CHECK(!result.accepted);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::ControllerRejected);
    RB_CHECK(result.ack_policy == rb_servo::BackendAckPolicy::Wait);
    RB_CHECK(result.ack_observed);
    RB_CHECK(backend.transportCounters().ack_error_total == 1);
    return true;
}

bool testTimeoutRejected() {
    FakeCommandServer server(std::nullopt, 200);
    rb_servo::BackendConfig cfg = testConfig(server.port());
    cfg.command_timeout_sec = 0.03;
    rb_servo::RbscriptTcpBackend backend(rb_servo::ArmId::Left, cfg);
    RB_CHECK(backend.connect().ok);
    const rb_servo::SendServoJResult result = backend.sendServoJ(request());
    RB_CHECK(!result.accepted);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::TransportTimeout);
    RB_CHECK(result.ack_policy == rb_servo::BackendAckPolicy::Wait);
    RB_CHECK(!result.ack_observed);
    return true;
}

bool testDisableAckSendsWithoutWaiting() {
    FakeCommandServer server(std::nullopt, 100);
    rb_servo::BackendConfig cfg = testConfig(server.port());
    cfg.disable_waiting_ack = true;
    rb_servo::RbscriptTcpBackend backend(rb_servo::ArmId::Left, cfg);
    RB_CHECK(backend.connect().ok);
    const rb_servo::SendServoJResult result = backend.sendServoJ(request());
    RB_CHECK(result.accepted);
    RB_CHECK(result.ack_policy == rb_servo::BackendAckPolicy::Disabled);
    RB_CHECK(!result.ack_observed);
    RB_CHECK(server.command().find("move_servo_j") != std::string::npos);
    RB_CHECK(backend.transportCounters().ack_success_total == 0);
    RB_CHECK(backend.transportCounters().ack_error_total == 0);
    return true;
}

bool testRealGatesRejectWithoutEnv() {
    EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", nullptr);
    EnvGuard rbscript_gate("RB_ALLOW_RBSCRIPT_TCP", nullptr);
    EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", nullptr);
    EnvGuard rbscript_motion_gate("RB_ALLOW_RBSCRIPT_TCP_MOTION", nullptr);
    rb_servo::BackendConfig cfg = testConfig(5000);
    cfg.run_mode = rb_servo::RunMode::Real;
    rb_servo::RbscriptTcpBackend backend(rb_servo::ArmId::Left, cfg);
    const auto connect_result = backend.connect();
    RB_CHECK(!connect_result.ok);
    RB_CHECK(connect_result.error.kind == rb_servo::BackendErrorKind::SuppressedByPolicy);

    cfg.run_mode = rb_servo::RunMode::Real;
    rb_servo::RbscriptTcpBackend real_motion_backend(rb_servo::ArmId::Left, cfg);
    const rb_servo::SendServoJResult send_result = real_motion_backend.sendServoJ(request());
    RB_CHECK(!send_result.accepted);
    RB_CHECK(send_result.error.kind == rb_servo::BackendErrorKind::SuppressedByPolicy);
    return true;
}

bool testInvalidTargetRejected() {
    rb_servo::BackendConfig cfg = testConfig(5000);
    rb_servo::RbscriptTcpBackend backend(rb_servo::ArmId::Left, cfg);
    rb_servo::SendServoJRequest req = request();
    req.q_target_deg[3] = std::numeric_limits<double>::quiet_NaN();
    const rb_servo::SendServoJResult result = backend.sendServoJ(req);
    RB_CHECK(!result.accepted);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::InvalidTarget);
    return true;
}

}  // namespace

int main() {
    if (!testFormatServoJ()) return 1;
    if (!testReqdataFormat()) return 1;
    if (!testParseDataFixture()) return 1;
    if (!testParseRejectsShortData()) return 1;
    if (!testParseRejectsUnknownFormat()) return 1;
    if (!testParseRejectsUnknownSchema()) return 1;
    if (!testParseRejectsInvalidJointData()) return 1;
    if (!testReadStateFromDataPortFixture()) return 1;
    if (!testReadStateParseFailure()) return 1;
    if (!testReadStateOkWhenServoDisabled()) return 1;
    if (!testReadStateTimeout()) return 1;
    if (!testSuccessAckAccepted()) return 1;
    if (!testErrorAckRejected()) return 1;
    if (!testTimeoutRejected()) return 1;
    if (!testDisableAckSendsWithoutWaiting()) return 1;
    if (!testRealGatesRejectWithoutEnv()) return 1;
    if (!testInvalidTargetRejected()) return 1;
    return 0;
}
