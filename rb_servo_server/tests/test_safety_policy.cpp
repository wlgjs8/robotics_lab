#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <atomic>
#include <mutex>
#include <optional>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/dual_arm_servo_loop.hpp"
#include "rb_servo/control/safety_filter.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/kinematics/i_kinematics.hpp"
#include "rb_servo/logging/servo_logger.hpp"
#include "rb_servo/network/command_server.hpp"
#include "rb_servo/network/state_publisher.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"
#include "rb_servo/robot/rbsim_backend.hpp"

#include <nlohmann/json.hpp>

namespace {

constexpr double kEpsilon = 1e-9;

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

rb_servo::JointArray joints(double value) {
    rb_servo::JointArray out{};
    out.fill(value);
    return out;
}

bool sameJointArray(const rb_servo::JointArray& a, const rb_servo::JointArray& b) {
    for (int i = 0; i < rb_servo::kDof; ++i) {
        if (std::abs(a[i] - b[i]) > kEpsilon) return false;
    }
    return true;
}

bool contains(const std::string& text, const std::string& needle) {
    return text.find(needle) != std::string::npos;
}

bool jsonArrayHasSixFinite(const nlohmann::json& value) {
    if (!value.is_array() || value.size() != static_cast<size_t>(rb_servo::kDof)) return false;
    for (const auto& item : value) {
        if (!item.is_number()) return false;
        if (!std::isfinite(item.get<double>())) return false;
    }
    return true;
}

class TestBackend final : public rb_servo::IRobotBackend {
public:
    TestBackend(
        rb_servo::ArmId arm_id,
        rb_servo::JointArray initial,
        bool fail_send,
        rb_servo::BackendErrorKind send_error_kind = rb_servo::BackendErrorKind::ControllerRejected
    ) : arm_id_(arm_id),
        q_actual_(initial),
        q_target_(initial),
        fail_send_(fail_send),
        send_error_kind_(send_error_kind) {}

    rb_servo::BackendResult<rb_servo::RobotState> connect() override {
        connected_ = true;
        return result(rb_servo::BackendOp::Connect, currentState(), true);
    }

    rb_servo::BackendResult<rb_servo::RobotState> initialize() override {
        initialized_ = true;
        return result(rb_servo::BackendOp::Initialize, currentState(), true);
    }

    rb_servo::BackendResult<rb_servo::RobotState> readState() override {
        ++read_count_;
        if (!read_ok_) {
            return result(
                rb_servo::BackendOp::ReadState,
                currentState(),
                false,
                rb_servo::backendError(rb_servo::BackendErrorKind::TransportReadFailed, "test read failed")
            );
        }
        return result(rb_servo::BackendOp::ReadState, currentState(), true);
    }

    rb_servo::SendServoJResult sendServoJ(const rb_servo::SendServoJRequest& request) override {
        ++send_count_;
        if (fail_send_) {
            std::optional<rb_servo::RobotState> state_after;
            std::string state_after_source = "none";
            if (send_error_kind_ == rb_servo::BackendErrorKind::RobotFault) {
                has_error_ = true;
                error_code_ = 2222;
                state_after = currentState();
                state_after_source = "cache";
            }
            return rb_servo::rejectedSend(
                request,
                rb_servo::backendError(
                    send_error_kind_,
                    send_error_kind_ == rb_servo::BackendErrorKind::SuppressedByPolicy
                        ? "test send suppressed by policy"
                        : "test send failed",
                    send_error_kind_ == rb_servo::BackendErrorKind::RobotFault ? "2222" : "",
                    ""
                ),
                {},
                state_after,
                state_after_source
            );
        }
        q_target_ = request.q_target_deg;
        q_actual_ = request.q_target_deg;
        return rb_servo::acceptedSend(request, {}, currentState(), "cache");
    }

    rb_servo::BackendResult<rb_servo::RobotState> stop() override {
        return result(rb_servo::BackendOp::Stop, currentState(), true);
    }
    rb_servo::BackendResult<rb_servo::RobotState> resetFault() override {
        ++reset_count_;
        if (invalidate_joint_state_on_reset_) {
            valid_joint_state_ = false;
        }
        return result(
            rb_servo::BackendOp::ResetFault,
            currentState(),
            reset_ok_,
            rb_servo::backendError(rb_servo::BackendErrorKind::ControllerRejected, "test reset failed")
        );
    }
    bool isConnected() const override { return connected_; }
    rb_servo::ArmId armId() const override { return arm_id_; }
    std::string name() const override { return "test"; }

    void setValidJointState(bool valid) { valid_joint_state_ = valid; }
    void setReadOk(bool ok) { read_ok_ = ok; }
    void setConnected(bool connected) { connected_ = connected; }
    void setHasError(bool has_error) { has_error_ = has_error; }
    void setResetOk(bool ok) { reset_ok_ = ok; }
    void setInvalidateJointStateOnReset(bool invalidate) { invalidate_joint_state_on_reset_ = invalidate; }
    int readCount() const { return read_count_; }
    int resetCount() const { return reset_count_; }
    int sendCount() const { return send_count_; }

private:
    rb_servo::RobotState currentState() const {
        rb_servo::RobotState state;
        state.arm_id = arm_id_;
        state.host_time_ns = rb_servo::nowSteadyNs();
        state.q_actual_deg = q_actual_;
        state.q_target_deg = q_target_;
        state.has_valid_joint_state = valid_joint_state_;
        state.connection_state = connected_
            ? rb_servo::RobotConnectionState::Connected
            : rb_servo::RobotConnectionState::Disconnected;
        state.servo_enabled = initialized_;
        state.has_error = has_error_;
        state.error_code = error_code_;
        return state;
    }

    rb_servo::BackendResult<rb_servo::RobotState> result(
        rb_servo::BackendOp op,
        const rb_servo::RobotState& state,
        bool ok,
        const rb_servo::BackendError& error = rb_servo::noBackendError()
    ) const {
        rb_servo::BackendResult<rb_servo::RobotState> out;
        out.ok = ok;
        out.op = op;
        out.value = state;
        out.error = ok ? rb_servo::noBackendError() : error;
        return out;
    }

    rb_servo::ArmId arm_id_;
    rb_servo::JointArray q_actual_{};
    rb_servo::JointArray q_target_{};
    bool fail_send_ = false;
    rb_servo::BackendErrorKind send_error_kind_ = rb_servo::BackendErrorKind::ControllerRejected;
    bool valid_joint_state_ = true;
    bool read_ok_ = true;
    bool reset_ok_ = true;
    bool invalidate_joint_state_on_reset_ = false;
    bool has_error_ = false;
    int error_code_ = 0;
    bool connected_ = false;
    bool initialized_ = false;
    int read_count_ = 0;
    int reset_count_ = 0;
    int send_count_ = 0;
};

class FakeCartesianKinematics final : public rb_servo::IKinematics {
public:
    rb_servo::Pose6D computeTcpBase(const rb_servo::JointArray& q_deg) const override {
        return {q_deg[0] / 100.0, q_deg[1] / 100.0, q_deg[2] / 100.0, 0.0, 0.0, 0.0};
    }

    rb_servo::Pose6D computeTcpStand(
        rb_servo::ArmId arm,
        const rb_servo::JointArray& q_deg,
        const rb_servo::ArmMountConfig& mount
    ) const override {
        (void)arm;
        const rb_servo::Pose6D tcp_base = computeTcpBase(q_deg);
        return {
            mount.base_pose_in_stand.x + tcp_base.x,
            mount.base_pose_in_stand.y + tcp_base.y,
            mount.base_pose_in_stand.z + tcp_base.z,
            0.0,
            0.0,
            0.0,
        };
    }

    rb_servo::IkResult solveIk(
        rb_servo::ArmId arm,
        const rb_servo::Pose6D& target_tcp_stand,
        const rb_servo::JointArray& seed_q_deg,
        const rb_servo::ArmMountConfig& mount
    ) const override {
        (void)arm;
        rb_servo::IkResult result;
        result.q_solution_deg = seed_q_deg;
        if (fail_) {
            result.success = false;
            result.q_solution_deg = joints(999.0);
            result.reason = "injected_failure";
            return result;
        }
        result.success = true;
        result.q_solution_deg[0] = (target_tcp_stand.x - mount.base_pose_in_stand.x) * 100.0;
        result.q_solution_deg[1] = (target_tcp_stand.y - mount.base_pose_in_stand.y) * 100.0;
        result.q_solution_deg[2] = (target_tcp_stand.z - mount.base_pose_in_stand.z) * 100.0;
        return result;
    }

    void setFail(bool fail) { fail_ = fail; }

private:
    bool fail_ = false;
};

class ScriptedRbsimServer {
public:
    ScriptedRbsimServer() {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0) throw std::runtime_error("socket failed");

        int reuse = 1;
        (void)::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            ::close(listen_fd_);
            throw std::runtime_error("bind failed");
        }
        if (::listen(listen_fd_, 8) != 0) {
            ::close(listen_fd_);
            throw std::runtime_error("listen failed");
        }

        sockaddr_in bound{};
        socklen_t len = sizeof(bound);
        if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&bound), &len) != 0) {
            ::close(listen_fd_);
            throw std::runtime_error("getsockname failed");
        }
        port_ = ntohs(bound.sin_port);
        thread_ = std::thread(&ScriptedRbsimServer::run, this);
    }

    ~ScriptedRbsimServer() {
        running_ = false;
        if (listen_fd_ >= 0) {
            ::shutdown(listen_fd_, SHUT_RDWR);
            ::close(listen_fd_);
            listen_fd_ = -1;
        }
        if (thread_.joinable()) thread_.join();
    }

    std::string endpoint() const {
        return "tcp://127.0.0.1:" + std::to_string(port_);
    }

    void failNextSend(const std::string& arm = "left") {
        std::lock_guard<std::mutex> lock(state_mutex_);
        armState(arm).fail_next_send = true;
    }

    void setSendFailure(const std::string& arm, bool enabled) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        armState(arm).send_failure = enabled;
    }

    void failNextRead(const std::string& arm = "left") {
        std::lock_guard<std::mutex> lock(state_mutex_);
        armState(arm).fail_next_read = true;
    }

    void setStopFailure(const std::string& arm, bool enabled) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        armState(arm).stop_failure = enabled;
    }

    void setResetFailure(const std::string& arm, bool enabled) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        armState(arm).reset_failure = enabled;
    }

    void setJointValidity(const std::string& arm, bool valid) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        armState(arm).has_valid_joint_state = valid;
    }

    void setTrackingBias(const std::string& arm, double bias_deg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        armState(arm).tracking_bias_deg = bias_deg;
    }

    void disconnectArm(const std::string& arm) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ArmRuntime& state = armState(arm);
        state.connected = false;
        state.initialized = false;
        state.servo_enabled = false;
        state.stopped = true;
    }

    void setFault(const std::string& arm, int error_code) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ArmRuntime& state = armState(arm);
        state.has_error = true;
        state.error_code = error_code;
        state.servo_enabled = false;
        state.stopped = true;
    }

private:
    struct ArmRuntime {
        bool connected = false;
        bool initialized = false;
        bool servo_enabled = false;
        bool stopped = false;
        bool has_error = false;
        bool has_valid_joint_state = true;
        bool send_failure = false;
        bool fail_next_send = false;
        bool fail_next_read = false;
        bool stop_failure = false;
        bool reset_failure = false;
        int error_code = 0;
        double tracking_bias_deg = 0.0;
        uint64_t robot_time_ns = 0;
        std::vector<double> q_actual{0, -30, 80, 0, 60, 0};
        std::vector<double> q_target{0, -30, 80, 0, 60, 0};
    };

    ArmRuntime& armState(const std::string& arm) {
        return arm == "right" ? right_ : left_;
    }

    void run() {
        while (running_) {
            const int client = ::accept(listen_fd_, nullptr, nullptr);
            if (client < 0) continue;
            handleClient(client);
            ::close(client);
        }
    }

    void handleClient(int client) {
        std::string line;
        char c = '\0';
        while (::recv(client, &c, 1, 0) == 1) {
            if (c == '\n') break;
            line.push_back(c);
        }

        nlohmann::json request = nlohmann::json::parse(line);
        const std::string op = request.value("op", "");
        const std::string arm = request.value("arm", "left");
        const std::string request_id = request.value("request_id", "");

        nlohmann::json response;
        response["schema_version"] = "rbsim.v1";
        response["request_id"] = request_id;
        response["server_time_ns"] = rb_servo::nowSteadyNs();
        response["arm"] = arm;

        std::lock_guard<std::mutex> lock(state_mutex_);
        ArmRuntime& state = armState(arm);
        const auto stateJson = [&]() {
            std::vector<double> q_actual = state.q_actual;
            q_actual[0] += state.tracking_bias_deg;
            return nlohmann::json{
                {"arm", arm},
                {"q_actual_deg", q_actual},
                {"q_target_deg", state.q_target},
                {"dq_actual_deg_s", std::vector<double>{0, 0, 0, 0, 0, 0}},
                {"has_valid_joint_state", state.has_valid_joint_state},
                {"connection_state", state.connected ? "Connected" : "Disconnected"},
                {"servo_enabled", state.servo_enabled},
                {"has_error", state.has_error},
                {"error_code", state.error_code},
                {"robot_time_ns", state.robot_time_ns},
                {"lifecycle_state", state.has_error ? "faulted" : state.stopped ? "stopped" : state.servo_enabled ? "servo_enabled" : "connected"},
            };
        };
        if (op == "read_state" && state.fail_next_read) {
            state.fail_next_read = false;
            response["ok"] = false;
            response["error"] = {
                {"kind", "TransportReadFailed"},
                {"name", "read_failure_injected"},
                {"message", "read failure injected"},
                {"code", 2104},
                {"retryable", true},
                {"recoverable", true},
            };
            sendResponse(client, response);
            return;
        }

        if (op == "send_servo_j" && (state.send_failure || state.fail_next_send)) {
            state.fail_next_send = false;
            response["ok"] = false;
            response["error"] = {
                {"kind", "TransportWriteFailed"},
                {"name", "send_failure_injected"},
                {"message", "send failure injected"},
                {"code", 2101},
                {"retryable", true},
                {"recoverable", true},
            };
            sendResponse(client, response);
            return;
        }

        if (op == "send_servo_j" && state.has_error) {
            response["ok"] = false;
            response["error"] = {
                {"kind", "RobotFault"},
                {"name", "fault_latched"},
                {"message", "fault latched"},
                {"code", state.error_code},
                {"retryable", false},
                {"recoverable", true},
            };
            response["state"] = stateJson();
            sendResponse(client, response);
            return;
        }

        if (op == "connect") {
            state.connected = true;
            state.stopped = false;
        } else if (op == "initialize") {
            state.connected = true;
            state.initialized = true;
            state.servo_enabled = request.value("params", nlohmann::json::object()).value("enable_servo", true);
            state.stopped = false;
        } else if (op == "send_servo_j") {
            state.q_target = request.at("params").at("q_target_deg").get<std::vector<double>>();
            state.q_actual = state.q_target;
            state.stopped = false;
        } else if (op == "stop") {
            if (state.stop_failure) {
                response["ok"] = false;
                response["error"] = {
                    {"kind", "TransportWriteFailed"},
                    {"name", "stop_failure_injected"},
                    {"message", "stop failure injected"},
                    {"code", 2102},
                    {"retryable", true},
                    {"recoverable", true},
                };
                sendResponse(client, response);
                return;
            }
            state.q_target = state.q_actual;
            state.servo_enabled = false;
            state.stopped = true;
        } else if (op == "reset_fault") {
            if (state.reset_failure) {
                response["ok"] = false;
                response["error"] = {
                    {"kind", "TransportWriteFailed"},
                    {"name", "reset_failure_injected"},
                    {"message", "reset failure injected"},
                    {"code", 2103},
                    {"retryable", true},
                    {"recoverable", true},
                };
                sendResponse(client, response);
                return;
            }
            state.initialized = false;
            state.servo_enabled = false;
            state.has_error = false;
            state.error_code = 0;
            state.stopped = true;
        } else if (op != "read_state") {
            response["ok"] = false;
            response["error"] = {{"name", "unknown_operation"}, {"message", op}, {"code", 4003}};
            sendResponse(client, response);
            return;
        }

        state.robot_time_ns += 5000000;
        response["ok"] = true;
        response["state"] = stateJson();
        sendResponse(client, response);
    }

    void sendResponse(int client, const nlohmann::json& response) {
        const std::string payload = response.dump() + "\n";
        (void)::send(client, payload.data(), payload.size(), 0);
    }

    int listen_fd_ = -1;
    int port_ = 0;
    std::atomic<bool> running_{true};
    std::thread thread_;
    std::mutex state_mutex_;
    ArmRuntime left_;
    ArmRuntime right_;
};

rb_servo::DualArmConfig testConfig() {
    rb_servo::DualArmConfig cfg;
    cfg.left_robot.run_mode = rb_servo::RunMode::Mock;
    cfg.right_robot.run_mode = rb_servo::RunMode::Mock;
    cfg.servo.rate_hz = 200;
    cfg.servo.command_timeout_sec = 0.2;
    cfg.servo.enable_realtime_priority = false;
    cfg.servo.filter_dt_min_ratio = 0.5;
    cfg.servo.filter_dt_max_ratio = 1.5;
    cfg.safety.q_min_deg = joints(-180.0);
    cfg.safety.q_max_deg = joints(180.0);
    cfg.safety.dq_max_deg_s = joints(10000.0);
    cfg.safety.ddq_max_deg_s2 = joints(100000.0);
    cfg.safety.max_tracking_error_deg = 1000.0;
    cfg.safety.tracking_error_policy = rb_servo::TrackingErrorPolicy::SnapToActual;
    cfg.safety.stop_both_arms_on_single_arm_error = false;
    cfg.safety.latch_fault_on_robot_state_error = true;
    return cfg;
}

rb_servo::DualArmCommand command(rb_servo::ControlMode mode) {
    rb_servo::DualArmCommand cmd;
    cmd.seq = 1;
    cmd.host_time_ns = rb_servo::nowSteadyNs();
    cmd.left.arm_id = rb_servo::ArmId::Left;
    cmd.right.arm_id = rb_servo::ArmId::Right;
    cmd.left.mode = mode;
    cmd.right.mode = mode;
    cmd.left.timeout_sec = 0.2;
    cmd.right.timeout_sec = 0.2;
    return cmd;
}

void sleepTicks() {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
}

template <typename Predicate>
bool waitUntil(Predicate predicate, std::chrono::milliseconds timeout = std::chrono::milliseconds(500)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return predicate();
}

rb_servo::BackendConfig rbsimBackendConfig(
    rb_servo::ArmId arm_id,
    const std::string& endpoint,
    const std::string& suffix = ""
) {
    rb_servo::BackendConfig cfg;
    cfg.backend_type = rb_servo::BackendType::Rbsim;
    cfg.run_mode = rb_servo::RunMode::Simulation;
    cfg.name = std::string(arm_id == rb_servo::ArmId::Left ? "left_rbsim" : "right_rbsim") + suffix;
    cfg.rbsim_control_endpoint = endpoint;
    cfg.rbsim_request_timeout_sec = 0.2;
    cfg.rbsim_connect_timeout_sec = 0.2;
    cfg.rbsim_read_timeout_sec = 0.2;
    cfg.rbsim_send_timeout_sec = 0.2;
    cfg.rbsim_stop_timeout_sec = 0.2;
    cfg.rbsim_reset_timeout_sec = 0.2;
    return cfg;
}

std::unique_ptr<rb_servo::RbsimBackend> makeRbsimBackend(
    rb_servo::ArmId arm_id,
    const std::string& endpoint,
    const std::string& suffix = ""
) {
    return std::make_unique<rb_servo::RbsimBackend>(
        arm_id,
        rbsimBackendConfig(arm_id, endpoint, suffix)
    );
}

int reserveLoopbackUdpPort() {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return -1;
    }

    socklen_t len = sizeof(addr);
    if (::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        ::close(fd);
        return -1;
    }
    const int port = ntohs(addr.sin_port);
    ::close(fd);
    return port;
}

bool sendUdpJson(const std::string& host, int port, const std::string& payload) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        ::close(fd);
        return false;
    }
    const ssize_t sent = ::sendto(
        fd,
        payload.data(),
        payload.size(),
        0,
        reinterpret_cast<sockaddr*>(&addr),
        sizeof(addr)
    );
    ::close(fd);
    return sent == static_cast<ssize_t>(payload.size());
}

class EnvVarGuard {
public:
    explicit EnvVarGuard(const char* name)
        : name_(name) {
        const char* value = std::getenv(name_.c_str());
        if (value) {
            had_value_ = true;
            old_value_ = value;
        }
    }

    ~EnvVarGuard() {
        if (had_value_) {
            setenv(name_.c_str(), old_value_.c_str(), 1);
        } else {
            unsetenv(name_.c_str());
        }
    }

    void set(const char* value) const { setenv(name_.c_str(), value, 1); }
    void unset() const { unsetenv(name_.c_str()); }

private:
    std::string name_;
    bool had_value_ = false;
    std::string old_value_;
};

bool testSimulatorConfigParsesCanonicalAndAliases() {
    const std::filesystem::path config_path =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "config" / "dual_simulator.yaml";
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(config_path.string());

    RB_CHECK(cfg.left_robot.backend_type == rb_servo::BackendType::Simulator);
    RB_CHECK(cfg.right_robot.backend_type == rb_servo::BackendType::Simulator);
    RB_CHECK(cfg.left_robot.run_mode == rb_servo::RunMode::Simulation);
    RB_CHECK(cfg.right_robot.run_mode == rb_servo::RunMode::Simulation);
    RB_CHECK(cfg.left_robot.simulator_control_endpoint == "tcp://127.0.0.1:50200");
    RB_CHECK(cfg.right_robot.simulator_control_endpoint == "tcp://127.0.0.1:50210");
    RB_CHECK(cfg.left_robot.rbsim_control_endpoint == "tcp://127.0.0.1:50200");
    RB_CHECK(cfg.right_robot.rbsim_control_endpoint == "tcp://127.0.0.1:50210");
    RB_CHECK(std::abs(cfg.left_robot.rbsim_connect_timeout_sec - 0.2) < kEpsilon);
    RB_CHECK(std::abs(cfg.left_robot.rbsim_read_timeout_sec - 0.2) < kEpsilon);
    RB_CHECK(std::abs(cfg.left_robot.rbsim_send_timeout_sec - 0.2) < kEpsilon);
    RB_CHECK(std::abs(cfg.left_robot.rbsim_stop_timeout_sec - 0.2) < kEpsilon);
    RB_CHECK(std::abs(cfg.left_robot.rbsim_reset_timeout_sec - 0.2) < kEpsilon);
    RB_CHECK(cfg.network.command_bind == "udp://127.0.0.1:50010");
    RB_CHECK(cfg.network.state_pub_endpoint == "udp://127.0.0.1:50110");
    RB_CHECK(cfg.network.state_pub_bind == "udp://127.0.0.1:50110");
    RB_CHECK(cfg.network.state_pub_rate_hz == 20);

    const std::filesystem::path compose_config_path =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "config" / "dual_simulator_compose.yaml";
    const rb_servo::DualArmConfig compose_cfg = rb_servo::loadConfigFromYaml(compose_config_path.string());
    RB_CHECK(compose_cfg.left_robot.simulator_control_endpoint == "tcp://rb_simulator_left:50200");
    RB_CHECK(compose_cfg.right_robot.simulator_control_endpoint == "tcp://rb_simulator_right:50200");

    const std::string alias_path = "/tmp/rb-servo-simulator-alias-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(alias_path);
        file << "left_robot:\n"
             << "  backend_type: rbsim_local\n"
             << "  run_mode: rbsim_local\n"
             << "  rbsim_control_endpoint: \"tcp://127.0.0.1:50200\"\n"
             << "right_robot:\n"
             << "  backend_type: mock\n"
             << "  run_mode: mock\n";
    }

    std::ostringstream warnings;
    auto* const old_cerr = std::cerr.rdbuf(warnings.rdbuf());
    const rb_servo::DualArmConfig alias_cfg = rb_servo::loadConfigFromYaml(alias_path);
    std::cerr.rdbuf(old_cerr);
    ::unlink(alias_path.c_str());

    RB_CHECK(alias_cfg.left_robot.backend_type == rb_servo::BackendType::Simulator);
    RB_CHECK(alias_cfg.left_robot.run_mode == rb_servo::RunMode::Simulation);
    RB_CHECK(alias_cfg.left_robot.simulator_control_endpoint == "tcp://127.0.0.1:50200");
    RB_CHECK(warnings.str().find("deprecated") != std::string::npos);

    const std::string path = "/tmp/rb-servo-simulator-exposure-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(path);
        file << "left_robot:\n"
             << "  backend_type: simulator\n"
             << "  run_mode: simulation\n"
             << "  simulator_control_endpoint: \"tcp://0.0.0.0:50200\"\n"
             << "right_robot:\n"
             << "  backend_type: mock\n"
             << "  run_mode: mock\n";
    }

    bool rejected = false;
    try {
        (void)rb_servo::loadConfigFromYaml(path);
    } catch (const std::exception&) {
        rejected = true;
    }
    ::unlink(path.c_str());

    RB_CHECK(rejected);

    const std::string real_sim_path = "/tmp/rb-servo-real-simulator-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(real_sim_path);
        file << "left_robot:\n"
             << "  backend_type: simulator\n"
             << "  run_mode: real\n"
             << "  simulator_control_endpoint: \"tcp://127.0.0.1:50200\"\n"
             << "right_robot:\n"
             << "  backend_type: mock\n"
             << "  run_mode: mock\n";
    }

    bool real_sim_rejected = false;
    try {
        (void)rb_servo::loadConfigFromYaml(real_sim_path);
    } catch (const std::exception&) {
        real_sim_rejected = true;
    }
    ::unlink(real_sim_path.c_str());

    RB_CHECK(real_sim_rejected);
    return true;
}

bool testRbsimBackendMapsStateAndFailureResponses() {
    std::unique_ptr<ScriptedRbsimServer> server;
    try {
        server = std::make_unique<ScriptedRbsimServer>();
    } catch (const std::exception& exc) {
        std::cerr << "[SKIP] rbsim socket fixture unavailable: " << exc.what() << "\n";
        return true;
    }

    rb_servo::BackendConfig cfg;
    cfg.backend_type = rb_servo::BackendType::Rbsim;
    cfg.run_mode = rb_servo::RunMode::Simulation;
    cfg.name = "left_rbsim_test";
    cfg.rbsim_control_endpoint = server->endpoint();
    cfg.rbsim_request_timeout_sec = 0.2;

    rb_servo::RbsimBackend backend(rb_servo::ArmId::Left, cfg);
    RB_CHECK(backend.connect().ok);
    RB_CHECK(backend.isConnected());
    RB_CHECK(backend.initialize().ok);

    rb_servo::BackendResult<rb_servo::RobotState> state_result = backend.readState();
    RB_CHECK(state_result.ok);
    rb_servo::RobotState state = state_result.value;
    RB_CHECK(state.arm_id == rb_servo::ArmId::Left);
    RB_CHECK(state.connection_state == rb_servo::RobotConnectionState::Connected);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.servo_enabled);
    RB_CHECK(!state.has_error);
    RB_CHECK(state.error_code == 0);
    RB_CHECK(state.robot_time_ns > 0);
    RB_CHECK(state.q_actual_deg[1] == -30.0);
    RB_CHECK(state.q_target_deg[2] == 80.0);

    rb_servo::JointArray target = state.q_actual_deg;
    target[0] += 1.25;
    target[5] -= 2.5;
    rb_servo::SendServoJRequest request;
    request.q_target_deg = target;
    RB_CHECK(backend.sendServoJ(request).accepted);
    state_result = backend.readState();
    RB_CHECK(state_result.ok);
    state = state_result.value;
    RB_CHECK(sameJointArray(state.q_target_deg, target));

    server->failNextSend();
    target[0] += 5.0;
    request.q_target_deg = target;
    const rb_servo::SendServoJResult rejected = backend.sendServoJ(request);
    RB_CHECK(!rejected.accepted);
    RB_CHECK(rejected.error.kind == rb_servo::BackendErrorKind::TransportWriteFailed);
    RB_CHECK(rejected.error.name == "send_failure_injected");
    RB_CHECK(rejected.error.code == "2101");
    RB_CHECK(rejected.error.message == "send failure injected");
    RB_CHECK(rejected.error.transport_fault);
    RB_CHECK(!rejected.error.robot_fault);
    RB_CHECK(!rejected.state_after.has_value());
    RB_CHECK(rejected.state_after_source == "none");
    state_result = backend.readState();
    RB_CHECK(state_result.ok);
    state = state_result.value;
    RB_CHECK(state.q_target_deg[0] == 1.25);

    server->setFault("left", 2222);
    target[0] += 5.0;
    request.q_target_deg = target;
    const rb_servo::SendServoJResult faulted = backend.sendServoJ(request);
    RB_CHECK(!faulted.accepted);
    RB_CHECK(faulted.error.kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(faulted.error.name == "fault_latched");
    RB_CHECK(faulted.error.code == "2222");
    RB_CHECK(faulted.error.robot_fault);
    RB_CHECK(!faulted.error.transport_fault);
    RB_CHECK(faulted.state_after.has_value());
    RB_CHECK(faulted.state_after_source == "response");
    RB_CHECK(faulted.state_after->has_error);
    RB_CHECK(faulted.state_after->error_code == 2222);

    RB_CHECK(backend.stop().ok);
    RB_CHECK(backend.resetFault().ok);
    return true;
}

bool testRbsimInvalidJointStateLatchesAndHoldsPreviousTarget() {
    std::unique_ptr<ScriptedRbsimServer> server;
    try {
        server = std::make_unique<ScriptedRbsimServer>();
    } catch (const std::exception& exc) {
        std::cerr << "[SKIP] rbsim socket fixture unavailable: " << exc.what() << "\n";
        return true;
    }

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.stop_both_arms_on_single_arm_error = true;
    rb_servo::DualArmServoLoop loop(
        makeRbsimBackend(rb_servo::ArmId::Left, server->endpoint(), "_invalid_state"),
        makeRbsimBackend(rb_servo::ArmId::Right, server->endpoint(), "_invalid_state"),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    const rb_servo::ServoTarget initial = loop.previousSentTarget();
    server->setJointValidity("left", false);
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(snapshot.left_state.connection_state == rb_servo::RobotConnectionState::Connected);
    RB_CHECK(!snapshot.left_state.has_valid_joint_state);
    RB_CHECK(snapshot.right_state.has_valid_joint_state);
    RB_CHECK(sameJointArray(loop.previousSentTarget().left_q_target_deg, initial.left_q_target_deg));
    RB_CHECK(sameJointArray(loop.previousSentTarget().right_q_target_deg, initial.right_q_target_deg));
    loop.stop();
    return true;
}

bool testRbsimPerArmDisconnectLatchesAndPublishesTruthfulSnapshot() {
    std::unique_ptr<ScriptedRbsimServer> server;
    try {
        server = std::make_unique<ScriptedRbsimServer>();
    } catch (const std::exception& exc) {
        std::cerr << "[SKIP] rbsim socket fixture unavailable: " << exc.what() << "\n";
        return true;
    }

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        makeRbsimBackend(rb_servo::ArmId::Left, server->endpoint(), "_disconnect"),
        makeRbsimBackend(rb_servo::ArmId::Right, server->endpoint(), "_disconnect"),
        testConfig(),
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    server->disconnectArm("right");
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(snapshot.left_state.connection_state == rb_servo::RobotConnectionState::Connected);
    RB_CHECK(snapshot.right_state.connection_state == rb_servo::RobotConnectionState::Disconnected);
    RB_CHECK(!snapshot.right_state.servo_enabled);
    loop.stop();
    return true;
}

bool testRbsimReadRobotFaultUsesFaultClassifierReason() {
    std::unique_ptr<ScriptedRbsimServer> server;
    try {
        server = std::make_unique<ScriptedRbsimServer>();
    } catch (const std::exception& exc) {
        std::cerr << "[SKIP] rbsim socket fixture unavailable: " << exc.what() << "\n";
        return true;
    }

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        makeRbsimBackend(rb_servo::ArmId::Left, server->endpoint(), "_read_robot_fault"),
        makeRbsimBackend(rb_servo::ArmId::Right, server->endpoint(), "_read_robot_fault"),
        testConfig(),
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    server->setFault("left", 2222);
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(snapshot.left_state.has_error);
    RB_CHECK(snapshot.left_state.error_code == 2222);
    RB_CHECK(contains(snapshot.fault_reason, "robot/controller fault"));
    RB_CHECK(!contains(snapshot.fault_reason, "transport failure"));
    loop.stop();
    return true;
}

bool testRbsimReadFailureLatchesFaultWithoutAdvancingHold() {
    std::unique_ptr<ScriptedRbsimServer> server;
    try {
        server = std::make_unique<ScriptedRbsimServer>();
    } catch (const std::exception& exc) {
        std::cerr << "[SKIP] rbsim socket fixture unavailable: " << exc.what() << "\n";
        return true;
    }

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        makeRbsimBackend(rb_servo::ArmId::Left, server->endpoint(), "_read_failure"),
        makeRbsimBackend(rb_servo::ArmId::Right, server->endpoint(), "_read_failure"),
        testConfig(),
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    const rb_servo::ServoTarget initial = loop.previousSentTarget();
    server->failNextRead("left");
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(sameJointArray(loop.previousSentTarget().left_q_target_deg, initial.left_q_target_deg));
    RB_CHECK(sameJointArray(loop.previousSentTarget().right_q_target_deg, initial.right_q_target_deg));
    loop.stop();
    return true;
}

bool testRbsimSendFailureLatchesAndSnapshotsSendResult() {
    std::unique_ptr<ScriptedRbsimServer> server;
    try {
        server = std::make_unique<ScriptedRbsimServer>();
    } catch (const std::exception& exc) {
        std::cerr << "[SKIP] rbsim socket fixture unavailable: " << exc.what() << "\n";
        return true;
    }

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.stop_both_arms_on_single_arm_error = true;
    rb_servo::DualArmServoLoop loop(
        makeRbsimBackend(rb_servo::ArmId::Left, server->endpoint(), "_send_failure"),
        makeRbsimBackend(rb_servo::ArmId::Right, server->endpoint(), "_send_failure"),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(4.0);
    target.right.q_target_deg = joints(4.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    server->setSendFailure("left", true);
    buffer.setCommand(target);
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::SendFailure);
    RB_CHECK(!snapshot.left_send_ok);
    RB_CHECK(snapshot.right_send_ok);
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::FaultLatched);
    loop.stop();
    return true;
}

bool testRbsimResetFailureKeepsEmergencyLatch() {
    std::unique_ptr<ScriptedRbsimServer> server;
    try {
        server = std::make_unique<ScriptedRbsimServer>();
    } catch (const std::exception& exc) {
        std::cerr << "[SKIP] rbsim socket fixture unavailable: " << exc.what() << "\n";
        return true;
    }

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        makeRbsimBackend(rb_servo::ArmId::Left, server->endpoint(), "_reset_failure"),
        makeRbsimBackend(rb_servo::ArmId::Right, server->endpoint(), "_reset_failure"),
        testConfig(),
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::EmergencyStop));
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::EmergencyStop);

    server->setResetFailure("left", true);
    buffer.setCommand(command(rb_servo::ControlMode::ResetFault));
    sleepTicks();
    RB_CHECK(loop.faultLatched());
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::EmergencyStop);
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::EmergencyLatched);
    loop.stop();
    return true;
}

bool testRbsimStopFailureDoesNotReportStoppedState() {
    std::unique_ptr<ScriptedRbsimServer> server;
    try {
        server = std::make_unique<ScriptedRbsimServer>();
    } catch (const std::exception& exc) {
        std::cerr << "[SKIP] rbsim socket fixture unavailable: " << exc.what() << "\n";
        return true;
    }

    rb_servo::RbsimBackend backend(
        rb_servo::ArmId::Left,
        rbsimBackendConfig(rb_servo::ArmId::Left, server->endpoint(), "_stop_failure")
    );
    RB_CHECK(backend.connect().ok);
    RB_CHECK(backend.initialize().ok);
    server->setStopFailure("left", true);
    const rb_servo::BackendResult<rb_servo::RobotState> stop_result = backend.stop();
    RB_CHECK(!stop_result.ok);
    RB_CHECK(stop_result.error.name == "stop_failure_injected");
    RB_CHECK(stop_result.error.code == "2102");

    const rb_servo::BackendResult<rb_servo::RobotState> state_result = backend.readState();
    RB_CHECK(state_result.ok);
    const rb_servo::RobotState state = state_result.value;
    RB_CHECK(state.connection_state == rb_servo::RobotConnectionState::Connected);
    RB_CHECK(state.servo_enabled);
    return true;
}

bool testRbsimTrackingErrorFaultLatchHoldsPreviousTarget() {
    std::unique_ptr<ScriptedRbsimServer> server;
    try {
        server = std::make_unique<ScriptedRbsimServer>();
    } catch (const std::exception& exc) {
        std::cerr << "[SKIP] rbsim socket fixture unavailable: " << exc.what() << "\n";
        return true;
    }

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.max_tracking_error_deg = 2.0;
    cfg.safety.tracking_error_policy = rb_servo::TrackingErrorPolicy::FaultLatch;
    rb_servo::DualArmServoLoop loop(
        makeRbsimBackend(rb_servo::ArmId::Left, server->endpoint(), "_tracking"),
        makeRbsimBackend(rb_servo::ArmId::Right, server->endpoint(), "_tracking"),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    const rb_servo::ServoTarget initial = loop.previousSentTarget();
    server->setTrackingBias("left", 12.0);
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::TrackingError);
    RB_CHECK(std::abs(snapshot.left_state.q_actual_deg[0] - (initial.left_q_target_deg[0] + 12.0)) < kEpsilon);
    RB_CHECK(sameJointArray(loop.previousSentTarget().left_q_target_deg, initial.left_q_target_deg));
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::FaultLatched);
    loop.stop();
    return true;
}

bool testCommandValidation() {
    rb_servo::NetworkConfig network;
    network.command_timeout_sec = 0.35;
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    rb_servo::DualArmCommand out;
    const uint64_t now = rb_servo::nowSteadyNs();

    RB_CHECK(!server.parseMessage("{", now, &out));
    RB_CHECK(!server.parseMessage(R"({"mode":"Hold"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"Unknown"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"JointTarget"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"JointTarget","q_target_deg":[0,0,0,0,0]})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"JointTarget","q_target_deg":[0,0,0,0,0,"bad"]})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"JointTarget","timeout_sec":0,"q_target_deg":[0,0,0,0,0,0]})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"JointVelocity"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpPoseTarget"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpDeltaStand"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpDeltaLocal"})", now, &out));

    RB_CHECK(server.parseMessage(R"({"seq":1,"mode":"EmergencyStop"})", now, &out));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::EmergencyStop);
    RB_CHECK(server.parseMessage(R"({"seq":2,"mode":"ArmMotion"})", now, &out));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::ArmMotion);

    RB_CHECK(server.parseMessage(R"({"seq":3,"mode":"JointTarget","q_target_deg":[1,2,3,4,5,6]})", now, &out));
    RB_CHECK(out.left.has_joint_target);
    RB_CHECK(out.right.has_joint_target);
    RB_CHECK(std::abs(out.left.timeout_sec - 0.35) < kEpsilon);
    return true;
}

bool testCartesianCommandParser() {
    rb_servo::NetworkConfig network;
    network.command_timeout_sec = 0.35;
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    rb_servo::DualArmCommand out;
    const uint64_t now = rb_servo::nowSteadyNs();

    RB_CHECK(!server.parseMessage(R"({"schema_version":2,"seq":1,"mode":"TcpPoseTarget","tcp_target_stand":[0,0,0,0,0,0]})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"schema_version":1,"seq":1,"mode":"TcpPoseTarget"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"schema_version":1,"seq":1,"mode":"TcpPoseTarget","tcp_target_stand":[0,0,0,0,0]})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"schema_version":1,"seq":1,"mode":"TcpPoseTarget","tcp_target_stand":[0,0,0,0,0,1e999]})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"schema_version":1,"seq":1,"mode":"TcpPoseTarget","left":{"tcp_target_stand":[0,0,0,0,0,0]},"right":{}})", now, &out));

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":1,"mode":"TcpPoseTarget","host_time_ns":123456789,"timeout_sec":0.2,"left":{"tcp_target_stand":[0.3,0.1,0.5,0,3.14,0]},"right":{"tcp_target_stand":[0.3,-0.1,0.5,0,3.14,0]}})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::TcpPoseTarget);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::TcpPoseTarget);
    RB_CHECK(out.left.has_tcp_target);
    RB_CHECK(out.right.has_tcp_target);
    RB_CHECK(std::abs(out.left.tcp_target_stand.x - 0.3) < kEpsilon);
    RB_CHECK(std::abs(out.right.tcp_target_stand.y + 0.1) < kEpsilon);
    RB_CHECK(!out.left.has_tcp_delta_stand);
    RB_CHECK(!out.left.has_tcp_delta_local);

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":2,"mode":"TcpDeltaStand","timeout_sec":0.2,"left":{"tcp_delta_stand":[0.01,0,0,0,0,0]},"right":{"tcp_delta_stand":[0,-0.01,0,0,0,0]}})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::TcpDeltaStand);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::TcpDeltaStand);
    RB_CHECK(out.left.has_tcp_delta_stand);
    RB_CHECK(out.right.has_tcp_delta_stand);
    RB_CHECK(!out.left.has_tcp_delta_local);
    RB_CHECK(std::abs(out.left.tcp_delta_stand.x - 0.01) < kEpsilon);
    RB_CHECK(std::abs(out.right.tcp_delta_stand.y + 0.01) < kEpsilon);

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":3,"mode":"TcpDeltaLocal","timeout_sec":0.2,"left":{"tcp_delta_local":[0,0,0.01,0,0,0]},"right":{"tcp_delta_local":[0,0,-0.01,0,0,0]}})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::TcpDeltaLocal);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::TcpDeltaLocal);
    RB_CHECK(out.left.has_tcp_delta_local);
    RB_CHECK(out.right.has_tcp_delta_local);
    RB_CHECK(!out.left.has_tcp_delta_stand);
    RB_CHECK(std::abs(out.left.tcp_delta_local.z - 0.01) < kEpsilon);
    RB_CHECK(std::abs(out.right.tcp_delta_local.z + 0.01) < kEpsilon);
    return true;
}

bool testCommandSequenceRequiredAndMonotonic() {
    rb_servo::NetworkConfig network;
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    rb_servo::DualArmCommand out;
    const uint64_t now = rb_servo::nowSteadyNs();

    RB_CHECK(!server.parseMessage(R"({"mode":"Hold"})", now, &out));
    RB_CHECK(server.parseMessage(R"({"seq":10,"mode":"Hold"})", now, &out));
    RB_CHECK(out.seq == 10);
    RB_CHECK(!server.parseMessage(R"({"seq":10,"mode":"EmergencyStop"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":9,"mode":"EmergencyStop"})", now, &out));
    RB_CHECK(server.parseMessage(R"({"seq":11,"mode":"EmergencyStop"})", now, &out));
    RB_CHECK(out.seq == 11);
    RB_CHECK(out.left.mode == rb_servo::ControlMode::EmergencyStop);
    return true;
}

bool testCommandSourceAllowlistMatching() {
    rb_servo::NetworkConfig network;
    network.command_source_allowlist = {"127.0.0.1/32", "192.168.10.0/24", "10.1.2.3"};
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);

    RB_CHECK(server.acceptsCommandSource("127.0.0.1"));
    RB_CHECK(server.acceptsCommandSource("192.168.10.42"));
    RB_CHECK(server.acceptsCommandSource("10.1.2.3"));
    RB_CHECK(!server.acceptsCommandSource("192.168.11.42"));
    RB_CHECK(!server.acceptsCommandSource("10.1.2.4"));
    RB_CHECK(!server.acceptsCommandSource("not-an-ip"));
    return true;
}

bool testUdpCommandIngressAllowsOnlyTrustedSources() {
    const int accepted_port = reserveLoopbackUdpPort();
    if (accepted_port <= 0) {
        std::cerr << "SKIP testUdpCommandIngressAllowsOnlyTrustedSources: loopback UDP unavailable\n";
        return true;
    }

    rb_servo::NetworkConfig accepted_network;
    accepted_network.command_bind = "udp://127.0.0.1:" + std::to_string(accepted_port);
    accepted_network.command_source_allowlist = {"127.0.0.1/32"};
    rb_servo::CommandBuffer accepted_buffer;
    rb_servo::CommandServer accepted_server(accepted_network, &accepted_buffer);
    RB_CHECK(accepted_server.start());
    RB_CHECK(sendUdpJson(
        "127.0.0.1",
        accepted_port,
        R"({"seq":1,"mode":"JointTarget","timeout_sec":1.0,"q_target_deg":[1,2,3,4,5,6]})"
    ));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    rb_servo::DualArmCommand latest = accepted_buffer.latestOrHold(rb_servo::nowSteadyNs());
    accepted_server.stop();
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(latest.left.has_joint_target);
    RB_CHECK(std::abs(latest.left.q_target_deg[0] - 1.0) < kEpsilon);

    const int rejected_port = reserveLoopbackUdpPort();
    RB_CHECK(rejected_port > 0);
    rb_servo::NetworkConfig rejected_network;
    rejected_network.command_bind = "udp://127.0.0.1:" + std::to_string(rejected_port);
    rejected_network.command_source_allowlist = {"10.0.0.0/8"};
    rb_servo::CommandBuffer rejected_buffer;
    rb_servo::CommandServer rejected_server(rejected_network, &rejected_buffer);
    RB_CHECK(rejected_server.start());
    RB_CHECK(sendUdpJson("127.0.0.1", rejected_port, R"({"seq":1,"mode":"EmergencyStop"})"));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    latest = rejected_buffer.latestOrHold(rb_servo::nowSteadyNs());
    rejected_server.stop();
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::Hold);
    return true;
}

bool testCommandSourceAllowlistConfigValidation() {
    const std::string ok_path = "/tmp/rb-servo-allowlist-ok-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(ok_path);
        file << "network:\n"
             << "  command_source_allowlist: [\"127.0.0.1/32\", \"192.168.0.0/16\"]\n";
    }
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(ok_path);
    ::unlink(ok_path.c_str());
    RB_CHECK(cfg.network.command_source_allowlist.size() == 2);
    RB_CHECK(cfg.network.command_source_allowlist[0] == "127.0.0.1/32");
    RB_CHECK(cfg.network.command_source_allowlist[1] == "192.168.0.0/16");

    const std::string empty_path = "/tmp/rb-servo-allowlist-empty-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(empty_path);
        file << "network:\n"
             << "  command_source_allowlist: []\n";
    }
    bool rejected = false;
    try {
        (void)rb_servo::loadConfigFromYaml(empty_path);
    } catch (const std::exception&) {
        rejected = true;
    }
    ::unlink(empty_path.c_str());
    RB_CHECK(rejected);

    const std::string invalid_path = "/tmp/rb-servo-allowlist-invalid-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(invalid_path);
        file << "network:\n"
             << "  command_source_allowlist: [\"0.0.0.0/0\"]\n";
    }
    rejected = false;
    try {
        (void)rb_servo::loadConfigFromYaml(invalid_path);
    } catch (const std::exception&) {
        rejected = true;
    }
    ::unlink(invalid_path.c_str());
    RB_CHECK(rejected);
    return true;
}

bool testCommandBufferInvalidTimeoutHolds() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.host_time_ns = rb_servo::nowSteadyNs();
    target.left.q_target_deg = joints(12.0);
    target.right.q_target_deg = joints(12.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    target.left.timeout_sec = -1.0;
    target.right.timeout_sec = 0.2;
    buffer.setCommand(target);

    rb_servo::DualArmCommand latest = buffer.latestOrHold(target.host_time_ns + 1);
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::Hold);

    target.left.timeout_sec = 0.1;
    target.right.timeout_sec = 0.1;
    buffer.setCommand(target);
    latest = buffer.latestOrHold(target.host_time_ns + 200'000'000ULL);
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::Hold);
    return true;
}

bool testCoupledTimeoutUsesEarliestArmTimeout() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.host_time_ns = rb_servo::nowSteadyNs();
    target.left.timeout_sec = 0.2;
    target.right.timeout_sec = 0.05;
    target.coupled_timeout = false;
    target.left.q_target_deg = joints(4.0);
    target.right.q_target_deg = joints(4.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);

    rb_servo::DualArmCommand latest = buffer.latestOrHold(target.host_time_ns + 100'000'000ULL);
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::Hold);
    return true;
}

bool testLifecycleCommandSurvivesMotionOverwrite() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmCommand arm = command(rb_servo::ControlMode::ArmMotion);
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(3.0);
    target.right.q_target_deg = joints(3.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;

    const uint64_t now = rb_servo::nowSteadyNs();
    arm.host_time_ns = now;
    target.host_time_ns = now + 1;

    buffer.setCommand(arm);
    buffer.setCommand(target);

    rb_servo::DualArmCommand first = buffer.latestOrHold(now + 2);
    RB_CHECK(first.left.mode == rb_servo::ControlMode::ArmMotion);
    RB_CHECK(first.right.mode == rb_servo::ControlMode::ArmMotion);

    rb_servo::DualArmCommand second = buffer.latestOrHold(now + 3);
    RB_CHECK(second.left.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(second.right.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(sameJointArray(second.left.q_target_deg, joints(3.0)));
    return true;
}

bool testOversizedUdpPacketDoesNotUpdateCommandBuffer() {
    const int port = reserveLoopbackUdpPort();
    if (port <= 0) {
        std::cerr << "SKIP testOversizedUdpPacketDoesNotUpdateCommandBuffer: loopback UDP unavailable\n";
        return true;
    }

    rb_servo::NetworkConfig network;
    network.command_bind = "udp://127.0.0.1:" + std::to_string(port);
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    RB_CHECK(server.start());

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    RB_CHECK(fd >= 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    RB_CHECK(::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) == 1);
    const std::string oversized(9000, 'x');
    const ssize_t sent = ::sendto(
        fd,
        oversized.data(),
        oversized.size(),
        0,
        reinterpret_cast<sockaddr*>(&addr),
        sizeof(addr)
    );
    ::close(fd);
    RB_CHECK(sent == static_cast<ssize_t>(oversized.size()));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const rb_servo::DualArmCommand latest = buffer.latestOrHold(rb_servo::nowSteadyNs());
    server.stop();
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::Hold);
    return true;
}

bool testCommandServerStartFailsOnInvalidBind() {
    rb_servo::NetworkConfig network;
    network.command_bind = "udp://invalid-host:50010";
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    RB_CHECK(!server.start());
    server.stop();
    return true;
}

bool testCommandServerStartFailsOnPortConflict() {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "SKIP testCommandServerStartFailsOnPortConflict: loopback UDP unavailable\n";
        return true;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "bind test socket failed: " << std::strerror(errno) << "\n";
        ::close(fd);
        return false;
    }

    socklen_t len = sizeof(addr);
    RB_CHECK(::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) == 0);
    const int port = ntohs(addr.sin_port);

    rb_servo::NetworkConfig network;
    network.command_bind = "udp://127.0.0.1:" + std::to_string(port);
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    const bool started = server.start();
    server.stop();
    ::close(fd);
    RB_CHECK(!started);
    return true;
}

bool testSecondCommandServerStartFailsOnSamePort() {
    const int port = reserveLoopbackUdpPort();
    if (port <= 0) {
        std::cerr << "SKIP testSecondCommandServerStartFailsOnSamePort: loopback UDP unavailable\n";
        return true;
    }

    rb_servo::NetworkConfig network;
    network.command_bind = "udp://127.0.0.1:" + std::to_string(port);
    rb_servo::CommandBuffer first_buffer;
    rb_servo::CommandBuffer second_buffer;
    rb_servo::CommandServer first(network, &first_buffer);
    rb_servo::CommandServer second(network, &second_buffer);

    RB_CHECK(first.start());
    const bool second_started = second.start();
    second.stop();
    first.stop();
    RB_CHECK(!second_started);
    return true;
}

bool testRealModeTcpStatePublisherExposureRequiresOverride() {
    const char* old_allow_real = std::getenv("RB_ALLOW_REAL_ROBOT");
    const char* old_allow_motion = std::getenv("RB_ALLOW_REAL_MOTION");
    const char* old_allow_network = std::getenv("RB_ALLOW_NETWORK_EXPOSURE");
    const std::string saved_allow_real = old_allow_real ? old_allow_real : "";
    const std::string saved_allow_motion = old_allow_motion ? old_allow_motion : "";
    const std::string saved_allow_network = old_allow_network ? old_allow_network : "";

    setenv("RB_ALLOW_REAL_ROBOT", "1", 1);
    setenv("RB_ALLOW_REAL_MOTION", "1", 1);
    unsetenv("RB_ALLOW_NETWORK_EXPOSURE");

    const std::string path = "/tmp/rb-servo-real-exposure-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(path);
        file << "left_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "network:\n"
             << "  command_bind: \"udp://127.0.0.1:50010\"\n"
             << "  state_pub_endpoint: \"tcp://0.0.0.0:50110\"\n";
    }

    bool rejected = false;
    try {
        (void)rb_servo::loadConfigFromYaml(path);
    } catch (const std::exception&) {
        rejected = true;
    }
    ::unlink(path.c_str());

    if (old_allow_real) {
        setenv("RB_ALLOW_REAL_ROBOT", saved_allow_real.c_str(), 1);
    } else {
        unsetenv("RB_ALLOW_REAL_ROBOT");
    }
    if (old_allow_motion) {
        setenv("RB_ALLOW_REAL_MOTION", saved_allow_motion.c_str(), 1);
    } else {
        unsetenv("RB_ALLOW_REAL_MOTION");
    }
    if (old_allow_network) {
        setenv("RB_ALLOW_NETWORK_EXPOSURE", saved_allow_network.c_str(), 1);
    } else {
        unsetenv("RB_ALLOW_NETWORK_EXPOSURE");
    }

    RB_CHECK(rejected);
    return true;
}

bool testRealModeReadOnlyAndMotionEnvGates() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.unset();
    allow_motion.unset();

    const std::string read_only_path = "/tmp/rb-servo-real-read-only-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(read_only_path);
        file << "left_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "servo:\n"
             << "  enable_realtime_priority: true\n"
             << "  send_servo_commands: false\n"
             << "safety:\n"
             << "  tracking_error_policy: fault_latch\n";
    }

    bool rejected_without_robot = false;
    try {
        (void)rb_servo::loadConfigFromYaml(read_only_path);
    } catch (const std::exception&) {
        rejected_without_robot = true;
    }
    RB_CHECK(rejected_without_robot);

    allow_real.set("1");
    const rb_servo::DualArmConfig read_only_cfg = rb_servo::loadConfigFromYaml(read_only_path);
    RB_CHECK(!read_only_cfg.servo.send_servo_commands);
    ::unlink(read_only_path.c_str());

    const std::string motion_path = "/tmp/rb-servo-real-motion-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(motion_path);
        file << "left_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "servo:\n"
             << "  enable_realtime_priority: true\n"
             << "  send_servo_commands: true\n"
             << "safety:\n"
             << "  tracking_error_policy: fault_latch\n";
    }

    bool rejected_without_motion = false;
    try {
        (void)rb_servo::loadConfigFromYaml(motion_path);
    } catch (const std::exception&) {
        rejected_without_motion = true;
    }
    RB_CHECK(rejected_without_motion);

    allow_motion.set("1");
    const rb_servo::DualArmConfig motion_cfg = rb_servo::loadConfigFromYaml(motion_path);
    RB_CHECK(motion_cfg.servo.send_servo_commands);
    ::unlink(motion_path.c_str());
    return true;
}

bool testSafetyFilterVelocityClampMaxStep() {
    rb_servo::SafetyConfig cfg;
    cfg.q_min_deg = joints(-180.0);
    cfg.q_max_deg = joints(180.0);
    cfg.dq_max_deg_s = joints(10.0);
    cfg.ddq_max_deg_s2 = joints(100000.0);
    cfg.max_tracking_error_deg = 1000.0;
    rb_servo::SafetyFilter filter(cfg);

    rb_servo::RobotState state;
    state.connection_state = rb_servo::RobotConnectionState::Connected;
    state.has_valid_joint_state = true;
    state.q_actual_deg = joints(0.0);

    const rb_servo::SafetyCheckResult result = filter.filterJointTarget(
        joints(100.0),
        joints(0.0),
        joints(0.0),
        state,
        0.01
    );

    RB_CHECK(result.ok);
    for (double q : result.filtered_q_deg) {
        RB_CHECK(q <= 0.1 + kEpsilon);
        RB_CHECK(q >= -kEpsilon);
    }
    return true;
}

bool testSafetyFilterAccelerationClampDoesNotOvershoot() {
    rb_servo::SafetyConfig cfg;
    cfg.q_min_deg = joints(-180.0);
    cfg.q_max_deg = joints(180.0);
    cfg.dq_max_deg_s = joints(1000.0);
    cfg.ddq_max_deg_s2 = joints(100.0);
    cfg.max_tracking_error_deg = 1000.0;
    rb_servo::SafetyFilter filter(cfg);

    rb_servo::RobotState state;
    state.connection_state = rb_servo::RobotConnectionState::Connected;
    state.has_valid_joint_state = true;
    state.q_actual_deg = joints(0.0);

    const rb_servo::SafetyCheckResult result = filter.filterJointTarget(
        joints(0.005),
        joints(0.0),
        joints(0.0),
        state,
        0.01
    );

    RB_CHECK(result.ok);
    for (double q : result.filtered_q_deg) {
        RB_CHECK(q <= 0.005 + kEpsilon);
        RB_CHECK(q >= -kEpsilon);
    }
    return true;
}

bool testRobotStateErrorRealPolicyLatchesFault() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Real;
    cfg.safety.latch_fault_on_robot_state_error = false;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    TestBackend* left_raw = left.get();
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    left_raw->setHasError(true);
    sleepTicks();
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::FaultLatched);
    return true;
}

bool testLatestSnapshotContainsSendTimingAndPreviousTargets() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.dq_max_deg_s = joints(10000.0);
    cfg.safety.ddq_max_deg_s2 = joints(100000.0);
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(2.0);
    target.right.q_target_deg = joints(2.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(snapshot.tick > 0);
    RB_CHECK(snapshot.loop_end_time_ns >= snapshot.loop_start_time_ns);
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::Running);
    RB_CHECK(snapshot.left_send_ok);
    RB_CHECK(snapshot.right_send_ok);
    RB_CHECK(snapshot.left_send_start_ns > 0);
    RB_CHECK(snapshot.left_send_end_ns >= snapshot.left_send_start_ns);
    RB_CHECK(snapshot.right_send_start_ns > 0);
    RB_CHECK(snapshot.right_send_end_ns >= snapshot.right_send_start_ns);
    RB_CHECK(snapshot.left_send_duration_us >= 0.0);
    RB_CHECK(snapshot.right_send_duration_us >= 0.0);
    RB_CHECK(sameJointArray(snapshot.left_prev_sent_q_deg, joints(2.0)));
    RB_CHECK(sameJointArray(snapshot.right_prev_sent_q_deg, joints(2.0)));
    return true;
}

bool testReadOnlyModeSuppressesSendsAndBlocksMotionCommands() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.send_servo_commands = false;
    cfg.safety.dq_max_deg_s = joints(10000.0);
    cfg.safety.ddq_max_deg_s2 = joints(100000.0);
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* left_backend = left.get();
    TestBackend* right_backend = right.get();
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    sleepTicks();
    uint64_t tick_before = loop.latestSnapshot().tick;
    RB_CHECK(left_backend->sendCount() == 0);
    RB_CHECK(right_backend->sendCount() == 0);

    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot current = loop.latestSnapshot();
        return current.command.left.mode == rb_servo::ControlMode::ArmMotion &&
               current.safety_verdict == rb_servo::SafetyVerdict::InvalidCommand;
    }));
    rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.tick > tick_before);
    RB_CHECK(snapshot.send_suppressed);
    RB_CHECK(snapshot.send_policy == "read_only");
    RB_CHECK(snapshot.left_send_ok);
    RB_CHECK(snapshot.right_send_ok);
    RB_CHECK(snapshot.left_send_start_ns == 0);
    RB_CHECK(snapshot.right_send_start_ns == 0);
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::ConnectedHold);
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::InvalidCommand);

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(9.0);
    target.right.q_target_deg = joints(9.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot current = loop.latestSnapshot();
        return current.command.left.mode == rb_servo::ControlMode::JointTarget &&
               current.safety_verdict == rb_servo::SafetyVerdict::InvalidCommand;
    }));
    snapshot = loop.latestSnapshot();
    RB_CHECK(left_backend->sendCount() == 0);
    RB_CHECK(right_backend->sendCount() == 0);
    RB_CHECK(snapshot.send_suppressed);
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::InvalidCommand);
    RB_CHECK(sameJointArray(loop.previousSentTarget().left_q_target_deg, initial));
    RB_CHECK(sameJointArray(loop.previousSentTarget().right_q_target_deg, initial));

    buffer.setCommand(command(rb_servo::ControlMode::EmergencyStop));
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));
    buffer.setCommand(command(rb_servo::ControlMode::ResetFault));
    sleepTicks();
    RB_CHECK(left_backend->resetCount() == 0);
    RB_CHECK(right_backend->resetCount() == 0);
    RB_CHECK(loop.faultLatched());
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::EmergencyLatched);
    RB_CHECK(left_backend->sendCount() == 0);
    RB_CHECK(right_backend->sendCount() == 0);

    loop.stop();
    return true;
}


bool testStatePublisherAcceptsDockerServiceHostnameEndpoint() {
    std::string host;
    int port = 0;
    RB_CHECK(rb_servo::StatePublisher::parseUdpEndpointUri("udp://rb_servo_gui:50110", &host, &port));
    RB_CHECK(host == "rb_servo_gui");
    RB_CHECK(port == 50110);
    RB_CHECK(!rb_servo::StatePublisher::parseUdpEndpointUri("udp://0.0.0.0:50110", &host, &port));
    RB_CHECK(!rb_servo::StatePublisher::parseUdpEndpointUri("tcp://rb_servo_gui:50110", &host, &port));
    return true;
}

bool testStatePublisherSerializesServoSnapshotSchema() {
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_mount.base_pose_in_stand = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    cfg.right_mount.base_pose_in_stand = {-0.1, -0.2, 0.3, -0.4, -0.5, 0.6};

    rb_servo::ServoSnapshot snapshot;
    snapshot.tick = 123;
    snapshot.loop_start_time_ns = 1'000;
    snapshot.loop_end_time_ns = 2'000;
    snapshot.period_ms = 5.0;
    snapshot.jitter_ms = 0.1;
    snapshot.filter_dt_ms = 5.0;
    snapshot.command.seq = 42;
    snapshot.command.left.mode = rb_servo::ControlMode::JointTarget;
    snapshot.command.right.mode = rb_servo::ControlMode::Hold;
    snapshot.left_state.arm_id = rb_servo::ArmId::Left;
    snapshot.right_state.arm_id = rb_servo::ArmId::Right;
    snapshot.left_state.host_time_ns = 11'000;
    snapshot.right_state.host_time_ns = 12'000;
    snapshot.left_state.robot_time_ns = 21'000;
    snapshot.right_state.robot_time_ns = 22'000;
    snapshot.left_state.q_actual_deg = joints(1.0);
    snapshot.right_state.q_actual_deg = joints(2.0);
    snapshot.left_state.has_valid_joint_state = true;
    snapshot.right_state.has_valid_joint_state = true;
    snapshot.left_state.connection_state = rb_servo::RobotConnectionState::Connected;
    snapshot.right_state.connection_state = rb_servo::RobotConnectionState::Connected;
    snapshot.left_sent_q_deg = joints(3.0);
    snapshot.right_sent_q_deg = joints(4.0);
    snapshot.left_prev_sent_q_deg = joints(5.0);
    snapshot.right_prev_sent_q_deg = joints(6.0);
    snapshot.left_send_ok = true;
    snapshot.right_send_ok = true;
    snapshot.left_send_start_ns = 10;
    snapshot.left_send_end_ns = 20;
    snapshot.right_send_start_ns = 30;
    snapshot.right_send_end_ns = 40;
    snapshot.send_skew_us = 20.0;
    snapshot.send_suppressed = true;
    snapshot.send_policy = "read_only";
    snapshot.left_send_duration_us = 10.0;
    snapshot.right_send_duration_us = 10.0;
    snapshot.safety_verdict = rb_servo::SafetyVerdict::Ok;
    snapshot.motion_state = rb_servo::ServerMotionState::Running;
    snapshot.fault_latched = false;
    snapshot.latched_fault_reason = rb_servo::SafetyVerdict::Ok;
    snapshot.fault_reason = "";
    snapshot.logger_dropped_samples = 0;

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));

    const char* top_keys[] = {
        "schema_version", "tick", "host_time_ns", "loop_start_time_ns", "loop_end_time_ns",
        "period_ms", "jitter_ms", "filter_dt_ms", "command_seq", "left", "right",
        "send_skew_us", "send_suppressed", "send_policy", "safety_verdict", "motion_state", "fault_latched",
        "latched_fault_reason", "fault_reason", "logger_dropped_samples", "logger_health",
        "mount_transform_deferred", "mounts", "tcp_fields_deferred"
    };
    for (const char* key : top_keys) {
        RB_CHECK(json.contains(key));
    }
    const char* arm_keys[] = {
        "mode", "q_actual_deg", "q_sent_deg", "q_previous_sent_deg", "send_ok",
        "send_start_ns", "send_end_ns", "send_duration_us", "has_valid_joint_state",
        "connection_state", "robot_time_ns", "host_time_ns", "error_code",
        "tcp_stand", "tcp_base", "tcp_deferred"
    };
    for (const char* arm_name : {"left", "right"}) {
        for (const char* key : arm_keys) {
            RB_CHECK(json.at(arm_name).contains(key));
        }
    }

    RB_CHECK(json.at("schema_version").get<int>() == 1);
    RB_CHECK(json.at("tick").get<uint64_t>() == 123);
    RB_CHECK(json.at("host_time_ns").get<uint64_t>() == 2'000);
    RB_CHECK(json.at("loop_start_time_ns").get<uint64_t>() == 1'000);
    RB_CHECK(json.at("loop_end_time_ns").get<uint64_t>() == 2'000);
    RB_CHECK(json.at("period_ms").get<double>() == 5.0);
    RB_CHECK(json.at("jitter_ms").get<double>() == 0.1);
    RB_CHECK(json.at("filter_dt_ms").get<double>() == 5.0);
    RB_CHECK(json.at("command_seq").get<uint64_t>() == 42);
    RB_CHECK(json.at("left").at("mode").get<std::string>() == "JointTarget");
    RB_CHECK(json.at("right").at("mode").get<std::string>() == "Hold");
    RB_CHECK(jsonArrayHasSixFinite(json.at("left").at("q_actual_deg")));
    RB_CHECK(jsonArrayHasSixFinite(json.at("right").at("q_actual_deg")));
    RB_CHECK(jsonArrayHasSixFinite(json.at("left").at("q_sent_deg")));
    RB_CHECK(jsonArrayHasSixFinite(json.at("right").at("q_sent_deg")));
    RB_CHECK(jsonArrayHasSixFinite(json.at("left").at("q_previous_sent_deg")));
    RB_CHECK(jsonArrayHasSixFinite(json.at("right").at("q_previous_sent_deg")));
    RB_CHECK(json.at("left").at("send_ok").get<bool>());
    RB_CHECK(json.at("right").at("send_ok").get<bool>());
    RB_CHECK(json.at("left").at("send_start_ns").get<uint64_t>() == 10);
    RB_CHECK(json.at("left").at("send_end_ns").get<uint64_t>() == 20);
    RB_CHECK(json.at("right").at("send_start_ns").get<uint64_t>() == 30);
    RB_CHECK(json.at("right").at("send_end_ns").get<uint64_t>() == 40);
    RB_CHECK(json.at("left").at("host_time_ns").get<uint64_t>() == 11'000);
    RB_CHECK(json.at("right").at("robot_time_ns").get<uint64_t>() == 22'000);
    RB_CHECK(json.at("send_skew_us").get<double>() == 20.0);
    RB_CHECK(json.at("send_suppressed").get<bool>());
    RB_CHECK(json.at("send_policy").get<std::string>() == "read_only");
    RB_CHECK(json.at("left").at("send_duration_us").get<double>() == 10.0);
    RB_CHECK(json.at("right").at("send_duration_us").get<double>() == 10.0);
    RB_CHECK(json.at("safety_verdict").get<std::string>() == "Ok");
    RB_CHECK(json.at("motion_state").get<std::string>() == "Running");
    RB_CHECK(!json.at("fault_latched").get<bool>());
    RB_CHECK(json.at("latched_fault_reason").get<std::string>() == "Ok");
    RB_CHECK(json.at("fault_reason").get<std::string>().empty());
    RB_CHECK(json.at("logger_dropped_samples").get<uint64_t>() == 0);
    RB_CHECK(json.at("logger_health").at("ok").get<bool>());
    RB_CHECK(!json.at("mount_transform_deferred").get<bool>());
    RB_CHECK(json.at("mounts").at("left").at("frame").get<std::string>() == "stand");
    RB_CHECK(json.at("mounts").at("right").at("base_pose_in_stand").at("x").get<double>() == -0.1);
    RB_CHECK(json.at("tcp_fields_deferred").get<bool>());
    RB_CHECK(json.at("left").at("tcp_stand").is_null());
    RB_CHECK(json.at("left").at("tcp_base").is_null());
    RB_CHECK(json.at("left").at("tcp_deferred").get<bool>());
    RB_CHECK(json.at("right").at("tcp_stand").is_null());
    RB_CHECK(json.at("right").at("tcp_base").is_null());
    RB_CHECK(json.at("right").at("tcp_deferred").get<bool>());
    return true;
}

bool testStatePublisherUsesLatestSnapshotWithoutBackendReadsAndDoesNotStallLoop() {
    rb_servo::DualArmConfig cfg = testConfig();
    const int port = reserveLoopbackUdpPort();
    if (port <= 0) {
        std::cerr << "SKIP testStatePublisherUsesLatestSnapshotWithoutBackendReadsAndDoesNotStallLoop: loopback UDP unavailable\n";
        return true;
    }
    cfg.network.state_pub_bind = "udp://127.0.0.1:" + std::to_string(port);
    cfg.servo.rate_hz = 200;
    cfg.servo.enable_realtime_priority = false;

    int provider_calls = 0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* left_raw = left.get();
    TestBackend* right_raw = right.get();

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    sleepTicks();
    const uint64_t tick_before = loop.latestSnapshot().tick;

    rb_servo::StatePublisher publisher(cfg, [&loop, &provider_calls]() {
        ++provider_calls;
        return loop.latestSnapshot();
    });

    RB_CHECK(publisher.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    publisher.stop();
    const uint64_t tick_after = loop.latestSnapshot().tick;
    RB_CHECK(tick_after > tick_before + 5);
    RB_CHECK(provider_calls > 0);

    loop.stop();
    const int left_reads_after_loop_stop = left_raw->readCount();
    const int right_reads_after_loop_stop = right_raw->readCount();

    provider_calls = 0;
    RB_CHECK(publisher.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    publisher.stop();
    RB_CHECK(provider_calls > 0);
    RB_CHECK(left_raw->readCount() == left_reads_after_loop_stop);
    RB_CHECK(right_raw->readCount() == right_reads_after_loop_stop);
    return true;
}

bool testLoggerZeroCapacityDropsWithoutBlocking() {
    rb_servo::LoggingConfig cfg;
    cfg.enable = true;
    cfg.directory = "/tmp/rb-servo-logger-test-" + std::to_string(getpid());
    cfg.queue_capacity = 0;
    cfg.flush_period_ms = 1;

    rb_servo::ServoLogger logger(cfg);
    RB_CHECK(logger.start());
    rb_servo::ServoSample sample;
    logger.push(sample);
    logger.stop();
    std::filesystem::remove_all(cfg.directory);
    RB_CHECK(logger.droppedSamples() == 1);
    return true;
}

bool testInvalidStartupRobotStateFailsStart() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    left->setValidJointState(false);

    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(!loop.start());
    return true;
}

bool testEmergencyWinsAndResetDoesNotRun() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ConnectedHold);

    rb_servo::DualArmCommand mixed = command(rb_servo::ControlMode::ResetFault);
    mixed.right.mode = rb_servo::ControlMode::EmergencyStop;
    buffer.setCommand(mixed);
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::EmergencyLatched);
    RB_CHECK(loop.faultLatched());
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::EmergencyStop);

    buffer.setCommand(command(rb_servo::ControlMode::ResetFault));
    sleepTicks();
    RB_CHECK(!loop.faultLatched());
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ConnectedHold);

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(5.0);
    target.right.q_target_deg = joints(5.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ConnectedHold);

    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    buffer.setCommand(target);
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::Running);

    buffer.setCommand(command(rb_servo::ControlMode::ResetFault));
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ConnectedHold);
    loop.stop();
    return true;
}

bool testResetFaultFailureKeepsFaultLatched() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* left_backend = left.get();
    TestBackend* right_backend = right.get();
    left_backend->setResetOk(false);
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::EmergencyStop));
    sleepTicks();
    RB_CHECK(loop.faultLatched());
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::EmergencyLatched);

    buffer.setCommand(command(rb_servo::ControlMode::ResetFault));
    sleepTicks();
    RB_CHECK(left_backend->resetCount() > 0);
    RB_CHECK(right_backend->resetCount() > 0);
    RB_CHECK(loop.faultLatched());
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::EmergencyLatched);
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::EmergencyStop);
    loop.stop();
    return true;
}

bool testResetFaultRequiresFreshValidState() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* left_backend = left.get();
    left_backend->setInvalidateJointStateOnReset(true);
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::EmergencyStop));
    sleepTicks();
    RB_CHECK(loop.faultLatched());

    buffer.setCommand(command(rb_servo::ControlMode::ResetFault));
    sleepTicks();
    RB_CHECK(left_backend->resetCount() > 0);
    RB_CHECK(loop.faultLatched());
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::EmergencyLatched);
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::EmergencyStop);
    loop.stop();
    return true;
}

bool testDisarmAndCartesianHoldPreviousTarget() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand cartesian = command(rb_servo::ControlMode::TcpPoseTarget);
    cartesian.left.has_tcp_target = true;
    cartesian.right.has_tcp_target = true;
    cartesian.left.tcp_target_stand.x = 0.1;
    cartesian.right.tcp_target_stand.x = 0.1;
    buffer.setCommand(cartesian);
    sleepTicks();
    RB_CHECK(sameJointArray(loop.previousSentTarget().left_q_target_deg, initial));
    RB_CHECK(sameJointArray(loop.previousSentTarget().right_q_target_deg, initial));
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ArmedHold);
    rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::ArmedHold);
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable);

    buffer.setCommand(command(rb_servo::ControlMode::DisarmMotion));
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ConnectedHold);
    loop.stop();
    return true;
}

bool testCartesianPoseTargetUsesIkInSimulation() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.right_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.left_mount.arm_id = rb_servo::ArmId::Left;
    cfg.right_mount.arm_id = rb_servo::ArmId::Right;
    cfg.kinematics.enable = true;
    cfg.kinematics.ik.enable = true;
    cfg.kinematics.publish_tcp = true;
    const rb_servo::JointArray initial = joints(0.0);
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand mixed = command(rb_servo::ControlMode::Hold);
    mixed.left.mode = rb_servo::ControlMode::TcpPoseTarget;
    mixed.left.has_tcp_target = true;
    mixed.left.tcp_target_stand = {0.04, 0.02, 0.01, 0.0, 0.0, 0.0};
    mixed.right.mode = rb_servo::ControlMode::JointTarget;
    mixed.right.has_joint_target = true;
    mixed.right.q_target_deg = joints(3.0);
    buffer.setCommand(mixed);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        return snapshot.command.left.mode == rb_servo::ControlMode::TcpPoseTarget &&
               snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok &&
               snapshot.motion_state == rb_servo::ServerMotionState::Running &&
               std::abs(previous.left_q_target_deg[0] - 4.0) < kEpsilon &&
               std::abs(previous.left_q_target_deg[1] - 2.0) < kEpsilon &&
               std::abs(previous.left_q_target_deg[2] - 1.0) < kEpsilon &&
               sameJointArray(previous.right_q_target_deg, joints(3.0));
    }));
    const rb_servo::ServoTarget previous = loop.previousSentTarget();
    loop.stop();

    RB_CHECK(std::abs(previous.left_q_target_deg[0] - 4.0) < kEpsilon);
    RB_CHECK(std::abs(previous.left_q_target_deg[1] - 2.0) < kEpsilon);
    RB_CHECK(std::abs(previous.left_q_target_deg[2] - 1.0) < kEpsilon);
    RB_CHECK(sameJointArray(previous.right_q_target_deg, joints(3.0)));
    return true;
}

bool testCartesianDeltaStandAndLocalUseIkInSimulation() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.right_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.left_mount.arm_id = rb_servo::ArmId::Left;
    cfg.right_mount.arm_id = rb_servo::ArmId::Right;
    cfg.kinematics.enable = true;
    cfg.kinematics.ik.enable = true;
    cfg.kinematics.publish_tcp = true;
    const rb_servo::JointArray initial = joints(0.0);
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand stand_delta = command(rb_servo::ControlMode::TcpDeltaStand);
    stand_delta.left.has_tcp_delta_stand = true;
    stand_delta.right.has_tcp_delta_stand = true;
    stand_delta.left.tcp_delta_stand = {0.02, 0.0, 0.0, 0.0, 0.0, 0.0};
    stand_delta.right.tcp_delta_stand = {0.0, 0.03, 0.0, 0.0, 0.0, 0.0};
    buffer.setCommand(stand_delta);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        return previous.left_q_target_deg[0] >= 2.0 - kEpsilon &&
               previous.right_q_target_deg[1] >= 3.0 - kEpsilon;
    }));
    const rb_servo::ServoTarget after_stand_delta = loop.previousSentTarget();

    rb_servo::DualArmCommand local_delta = command(rb_servo::ControlMode::TcpDeltaLocal);
    local_delta.left.has_tcp_delta_local = true;
    local_delta.right.has_tcp_delta_local = true;
    local_delta.left.tcp_delta_local = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0};
    local_delta.right.tcp_delta_local = {0.0, 0.01, 0.0, 0.0, 0.0, 0.0};
    buffer.setCommand(local_delta);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        return previous.left_q_target_deg[0] >= after_stand_delta.left_q_target_deg[0] + 1.0 - kEpsilon &&
               previous.right_q_target_deg[1] >= after_stand_delta.right_q_target_deg[1] + 1.0 - kEpsilon;
    }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok);
    return true;
}

bool testCartesianIkFailureHoldsPreviousSafeTarget() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.right_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.kinematics.enable = true;
    cfg.kinematics.ik.enable = true;
    cfg.kinematics.publish_tcp = true;
    const rb_servo::JointArray initial = joints(0.0);
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    kinematics->setFail(true);

    rb_servo::DualArmCommand cartesian = command(rb_servo::ControlMode::TcpPoseTarget);
    cartesian.left.has_tcp_target = true;
    cartesian.right.has_tcp_target = true;
    cartesian.left.tcp_target_stand = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
    cartesian.right.tcp_target_stand = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
    buffer.setCommand(cartesian);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.command.left.mode == rb_servo::ControlMode::TcpPoseTarget &&
               snapshot.safety_verdict == rb_servo::SafetyVerdict::IkFailed;
    }));
    const rb_servo::ServoTarget previous = loop.previousSentTarget();
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();

    RB_CHECK(sameJointArray(previous.left_q_target_deg, initial));
    RB_CHECK(sameJointArray(previous.right_q_target_deg, initial));
    RB_CHECK(sameJointArray(snapshot.left_sent_q_deg, initial));
    RB_CHECK(sameJointArray(snapshot.right_sent_q_deg, initial));
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::ArmedHold);
    return true;
}

bool testCartesianRealModeBlockedByDefault() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Real;
    cfg.right_robot.run_mode = rb_servo::RunMode::Real;
    cfg.kinematics.enable = true;
    cfg.kinematics.ik.enable = true;
    cfg.kinematics.publish_tcp = true;
    cfg.cartesian_control.allow_in_real = false;
    const rb_servo::JointArray initial = joints(0.0);
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand cartesian = command(rb_servo::ControlMode::TcpPoseTarget);
    cartesian.left.has_tcp_target = true;
    cartesian.right.has_tcp_target = true;
    cartesian.left.tcp_target_stand = {0.04, 0.0, 0.0, 0.0, 0.0, 0.0};
    cartesian.right.tcp_target_stand = {0.04, 0.0, 0.0, 0.0, 0.0, 0.0};
    buffer.setCommand(cartesian);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.command.left.mode == rb_servo::ControlMode::TcpPoseTarget &&
               snapshot.safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable;
    }));
    const rb_servo::ServoTarget previous = loop.previousSentTarget();
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();

    RB_CHECK(sameJointArray(previous.left_q_target_deg, initial));
    RB_CHECK(sameJointArray(previous.right_q_target_deg, initial));
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::ArmedHold);
    return true;
}

bool testInvalidMotionCommandDoesNotReportRunning() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand missing_payload = command(rb_servo::ControlMode::JointTarget);
    buffer.setCommand(missing_payload);
    sleepTicks();
    RB_CHECK(sameJointArray(loop.previousSentTarget().left_q_target_deg, initial));
    RB_CHECK(sameJointArray(loop.previousSentTarget().right_q_target_deg, initial));
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ArmedHold);
    rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::ArmedHold);
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::InvalidCommand);
    loop.stop();
    return true;
}

bool testJointLimitClamp() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.q_max_deg = joints(5.0);
    cfg.safety.q_min_deg = joints(-5.0);
    cfg.safety.dq_max_deg_s = joints(100000.0);
    cfg.safety.ddq_max_deg_s2 = joints(1000000.0);
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(100.0);
    target.right.q_target_deg = joints(100.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();
    loop.stop();

    const rb_servo::ServoTarget previous = loop.previousSentTarget();
    for (double q : previous.left_q_target_deg) RB_CHECK(q <= 5.0 + kEpsilon);
    for (double q : previous.right_q_target_deg) RB_CHECK(q <= 5.0 + kEpsilon);
    return true;
}

bool testSendFailureDoesNotAdvancePreviousTarget() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, true),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(7.0);
    target.right.q_target_deg = joints(7.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();
    loop.stop();

    const rb_servo::ServoTarget previous = loop.previousSentTarget();
    RB_CHECK(sameJointArray(previous.left_q_target_deg, initial));
    RB_CHECK(!sameJointArray(previous.right_q_target_deg, initial));
    return true;
}

bool testStopBothOnSendFailureLatchesFault() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.stop_both_arms_on_single_arm_error = true;
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, true),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(7.0);
    target.right.q_target_deg = joints(7.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();

    RB_CHECK(loop.faultLatched());
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::SendFailure);
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::FaultLatched);
    loop.stop();
    return true;
}

bool testRobotFaultSendClassifiesAsRobotStateFault() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.stop_both_arms_on_single_arm_error = true;
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Left,
            initial,
            true,
            rb_servo::BackendErrorKind::RobotFault
        ),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(7.0);
    target.right.q_target_deg = joints(7.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(snapshot.left_state.has_error);
    RB_CHECK(snapshot.left_state.error_code == 2222);
    RB_CHECK(!snapshot.left_send_ok);
    RB_CHECK(snapshot.left_send_error_kind == "RobotFault");
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::FaultLatched);
    loop.stop();
    return true;
}

bool testSuppressedByPolicySendDoesNotLatchFault() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.stop_both_arms_on_single_arm_error = true;
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Left,
            initial,
            true,
            rb_servo::BackendErrorKind::SuppressedByPolicy
        ),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(7.0);
    target.right.q_target_deg = joints(7.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(!snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(!snapshot.left_send_ok);
    RB_CHECK(snapshot.left_send_error_kind == "SuppressedByPolicy");
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::Ok);
    loop.stop();
    return true;
}

}  // namespace

int main() {
    if (!testCommandValidation()) return 1;
    if (!testSimulatorConfigParsesCanonicalAndAliases()) return 1;
    if (!testRbsimBackendMapsStateAndFailureResponses()) return 1;
    if (!testRbsimInvalidJointStateLatchesAndHoldsPreviousTarget()) return 1;
    if (!testRbsimPerArmDisconnectLatchesAndPublishesTruthfulSnapshot()) return 1;
    if (!testRbsimReadRobotFaultUsesFaultClassifierReason()) return 1;
    if (!testRbsimReadFailureLatchesFaultWithoutAdvancingHold()) return 1;
    if (!testRbsimSendFailureLatchesAndSnapshotsSendResult()) return 1;
    if (!testRbsimResetFailureKeepsEmergencyLatch()) return 1;
    if (!testRbsimStopFailureDoesNotReportStoppedState()) return 1;
    if (!testRbsimTrackingErrorFaultLatchHoldsPreviousTarget()) return 1;
    if (!testCommandSequenceRequiredAndMonotonic()) return 1;
    if (!testCartesianCommandParser()) return 1;
    if (!testCommandSourceAllowlistMatching()) return 1;
    if (!testUdpCommandIngressAllowsOnlyTrustedSources()) return 1;
    if (!testCommandSourceAllowlistConfigValidation()) return 1;
    if (!testCommandBufferInvalidTimeoutHolds()) return 1;
    if (!testCoupledTimeoutUsesEarliestArmTimeout()) return 1;
    if (!testLifecycleCommandSurvivesMotionOverwrite()) return 1;
    if (!testOversizedUdpPacketDoesNotUpdateCommandBuffer()) return 1;
    if (!testCommandServerStartFailsOnInvalidBind()) return 1;
    if (!testCommandServerStartFailsOnPortConflict()) return 1;
    if (!testSecondCommandServerStartFailsOnSamePort()) return 1;
    if (!testRealModeTcpStatePublisherExposureRequiresOverride()) return 1;
    if (!testRealModeReadOnlyAndMotionEnvGates()) return 1;
    if (!testSafetyFilterVelocityClampMaxStep()) return 1;
    if (!testSafetyFilterAccelerationClampDoesNotOvershoot()) return 1;
    if (!testRobotStateErrorRealPolicyLatchesFault()) return 1;
    if (!testLatestSnapshotContainsSendTimingAndPreviousTargets()) return 1;
    if (!testReadOnlyModeSuppressesSendsAndBlocksMotionCommands()) return 1;
    if (!testStatePublisherAcceptsDockerServiceHostnameEndpoint()) return 1;
    if (!testStatePublisherSerializesServoSnapshotSchema()) return 1;
    if (!testStatePublisherUsesLatestSnapshotWithoutBackendReadsAndDoesNotStallLoop()) return 1;
    if (!testLoggerZeroCapacityDropsWithoutBlocking()) return 1;
    if (!testInvalidStartupRobotStateFailsStart()) return 1;
    if (!testEmergencyWinsAndResetDoesNotRun()) return 1;
    if (!testResetFaultFailureKeepsFaultLatched()) return 1;
    if (!testResetFaultRequiresFreshValidState()) return 1;
    if (!testDisarmAndCartesianHoldPreviousTarget()) return 1;
    if (!testCartesianPoseTargetUsesIkInSimulation()) return 1;
    if (!testCartesianDeltaStandAndLocalUseIkInSimulation()) return 1;
    if (!testCartesianIkFailureHoldsPreviousSafeTarget()) return 1;
    if (!testCartesianRealModeBlockedByDefault()) return 1;
    if (!testInvalidMotionCommandDoesNotReportRunning()) return 1;
    if (!testJointLimitClamp()) return 1;
    if (!testSendFailureDoesNotAdvancePreviousTarget()) return 1;
    if (!testStopBothOnSendFailureLatchesFault()) return 1;
    if (!testRobotFaultSendClassifiesAsRobotStateFault()) return 1;
    if (!testSuppressedByPolicySendDoesNotLatchFault()) return 1;
    return 0;
}
