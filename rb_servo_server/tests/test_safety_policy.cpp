#include <chrono>
#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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
#include "rb_servo/control/cartesian_controller.hpp"
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

bool containsValue(const std::vector<std::string>& values, const std::string& needle) {
    return std::find(values.begin(), values.end(), needle) != values.end();
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
        rb_servo::BackendErrorKind send_error_kind = rb_servo::BackendErrorKind::ControllerRejected,
        std::optional<rb_servo::JointArray> initial_target = std::nullopt,
        bool accept_send_without_state_update = false
    ) : arm_id_(arm_id),
        q_actual_(initial),
        q_target_(initial_target.value_or(initial)),
        fail_send_(fail_send),
        send_error_kind_(send_error_kind),
        accept_send_without_state_update_(accept_send_without_state_update) {}

    rb_servo::BackendResult<rb_servo::RobotState> connect() override {
        connected_ = true;
        return result(rb_servo::BackendOp::Connect, currentState(), true);
    }

    rb_servo::BackendResult<rb_servo::RobotState> initialize() override {
        initialized_ = true;
        return result(rb_servo::BackendOp::Initialize, currentState(), true);
    }

    rb_servo::BackendResult<rb_servo::RobotState> readState() override {
        const int read_sleep_ms = read_sleep_ms_.load();
        if (read_sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(read_sleep_ms));
        }
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
        const int send_sleep_ms = send_sleep_ms_.load();
        if (send_sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(send_sleep_ms));
        }
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
        if (!accept_send_without_state_update_) {
            q_actual_ = request.q_target_deg;
        }
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
    void setErrorCode(int error_code) { error_code_ = error_code; }
    void setMotionReadinessError(
        const std::string& kind,
        const std::string& name,
        const std::string& source
    ) {
        motion_readiness_error_kind_ = kind;
        motion_readiness_error_name_ = name;
        diagnostic_error_source_ = source;
    }
    void setResetOk(bool ok) { reset_ok_ = ok; }
    void setInvalidateJointStateOnReset(bool invalidate) { invalidate_joint_state_on_reset_ = invalidate; }
    void setReadSleepMs(int sleep_ms) { read_sleep_ms_.store(sleep_ms); }
    void setSendSleepMs(int sleep_ms) { send_sleep_ms_.store(sleep_ms); }
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
        state.motion_readiness_error_kind = motion_readiness_error_kind_;
        state.motion_readiness_error_name = motion_readiness_error_name_;
        state.diagnostic_error_source = diagnostic_error_source_;
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
    bool accept_send_without_state_update_ = false;
    bool valid_joint_state_ = true;
    bool read_ok_ = true;
    bool reset_ok_ = true;
    bool invalidate_joint_state_on_reset_ = false;
    bool has_error_ = false;
    int error_code_ = 0;
    std::string motion_readiness_error_kind_;
    std::string motion_readiness_error_name_;
    std::string diagnostic_error_source_;
    bool connected_ = false;
    bool initialized_ = false;
    std::atomic<int> read_sleep_ms_{0};
    std::atomic<int> send_sleep_ms_{0};
    int read_count_ = 0;
    int reset_count_ = 0;
    int send_count_ = 0;
};

class FakeCartesianKinematics final : public rb_servo::IKinematics {
public:
    rb_servo::Pose6D computeTcpBase(const rb_servo::JointArray& q_deg) const override {
        return {
            q_deg[0] / 100.0,
            q_deg[1] / 100.0,
            q_deg[2] / 100.0,
            0.0,
            0.0,
            orientation_from_joint_ ? q_deg[5] / 100.0 : 0.0,
        };
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
            tcp_base.rz,
        };
    }

    rb_servo::IkResult solveIk(
        rb_servo::ArmId arm,
        const rb_servo::Pose6D& target_tcp_stand,
        const rb_servo::JointArray& seed_q_deg,
        const rb_servo::ArmMountConfig& mount
    ) const override {
        (void)arm;
        if (arm == rb_servo::ArmId::Left) {
            last_left_target_ = target_tcp_stand;
        } else {
            last_right_target_ = target_tcp_stand;
        }
        rb_servo::IkResult result;
        result.q_solution_deg = seed_q_deg;
        if (fail_) {
            result.success = false;
            result.q_solution_deg = joints(999.0);
            result.reason = "injected_failure";
            result.duration_us = 125.0;
            result.iterations = 7;
            return result;
        }
        result.success = true;
        result.q_solution_deg[0] = (target_tcp_stand.x - mount.base_pose_in_stand.x) * 100.0;
        result.q_solution_deg[1] = (target_tcp_stand.y - mount.base_pose_in_stand.y) * 100.0;
        result.q_solution_deg[2] = (target_tcp_stand.z - mount.base_pose_in_stand.z) * 100.0;
        result.position_error_m = position_error_m_;
        result.orientation_error_rad = orientation_error_rad_;
        if (orientation_from_joint_) {
            result.q_solution_deg[5] = (target_tcp_stand.rz + orientation_solve_bias_rad_) * 100.0;
        }
        result.duration_us = 125.0;
        result.iterations = 7;
        return result;
    }

    rb_servo::CartesianVelocityResult solveCartesianVelocity(
        rb_servo::ArmId arm,
        const rb_servo::JointArray& q_deg,
        const rb_servo::ArmMountConfig& mount,
        const rb_servo::Vec6& tcp_twist_local,
        double damping
    ) const override {
        (void)q_deg;
        (void)mount;
        (void)damping;
        if (arm == rb_servo::ArmId::Left) {
            last_left_twist_ = tcp_twist_local;
        } else {
            last_right_twist_ = tcp_twist_local;
        }
        rb_servo::CartesianVelocityResult result;
        if (fail_) {
            result.success = false;
            result.reason = "injected_failure";
            return result;
        }
        result.success = true;
        result.qdot_deg_s[0] = tcp_twist_local.x * 100.0;
        result.qdot_deg_s[1] = tcp_twist_local.y * 100.0;
        result.qdot_deg_s[2] = tcp_twist_local.z * 100.0;
        if (orientation_from_joint_) {
            result.qdot_deg_s[5] = tcp_twist_local.rz * 100.0;
        }
        return result;
    }

    void setFail(bool fail) { fail_ = fail; }
    void setOrientationFromJoint(bool enabled) { orientation_from_joint_ = enabled; }
    void setOrientationSolveBiasRad(double bias_rad) { orientation_solve_bias_rad_ = bias_rad; }
    void setSolveError(double position_error_m, double orientation_error_rad) {
        position_error_m_ = position_error_m;
        orientation_error_rad_ = orientation_error_rad;
    }
    std::optional<rb_servo::Pose6D> lastLeftTarget() const { return last_left_target_; }
    std::optional<rb_servo::Pose6D> lastRightTarget() const { return last_right_target_; }
    std::optional<rb_servo::Vec6> lastLeftTwist() const { return last_left_twist_; }
    std::optional<rb_servo::Vec6> lastRightTwist() const { return last_right_twist_; }

private:
    bool fail_ = false;
    bool orientation_from_joint_ = false;
    double orientation_solve_bias_rad_ = 0.0;
    double position_error_m_ = 0.0;
    double orientation_error_rad_ = 0.0;
    mutable std::optional<rb_servo::Pose6D> last_left_target_;
    mutable std::optional<rb_servo::Pose6D> last_right_target_;
    mutable std::optional<rb_servo::Vec6> last_left_twist_;
    mutable std::optional<rb_servo::Vec6> last_right_twist_;
};

class ScriptedRbsimServer {
public:
    explicit ScriptedRbsimServer(int requested_port = 0) {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0) throw std::runtime_error("socket failed");

        int reuse = 1;
        (void)::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(static_cast<uint16_t>(requested_port));
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
        std::lock_guard<std::mutex> lock(client_threads_mutex_);
        for (std::thread& client_thread : client_threads_) {
            if (client_thread.joinable()) client_thread.join();
        }
    }

    std::string endpoint() const {
        return "tcp://127.0.0.1:" + std::to_string(port_);
    }

    int connectionCount() const {
        return connection_count_.load();
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

    void dropNextRequest(const std::string& arm = "left") {
        std::lock_guard<std::mutex> lock(state_mutex_);
        armState(arm).drop_next_request = true;
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
        bool drop_next_request = false;
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
            connection_count_.fetch_add(1);
            std::lock_guard<std::mutex> lock(client_threads_mutex_);
            client_threads_.emplace_back(&ScriptedRbsimServer::handleClient, this, client);
        }
    }

    void handleClient(int client) {
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        (void)::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        std::string line;
        char c = '\0';
        while (running_) {
            line.clear();
            while (running_) {
                const ssize_t received = ::recv(client, &c, 1, 0);
                if (received == 1) {
                    if (c == '\n') break;
                    line.push_back(c);
                    continue;
                }
                if (received == 0) {
                    ::close(client);
                    return;
                }
                if (errno == EINTR) continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                ::close(client);
                return;
            }
            if (line.empty() && !running_) break;
            if (!handleRequest(client, line)) break;
        }
        ::close(client);
    }

    bool handleRequest(int client, const std::string& line) {
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
        if (state.drop_next_request) {
            state.drop_next_request = false;
            return false;
        }
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
            return true;
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
            return true;
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
            return true;
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
                return true;
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
                return true;
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
            return true;
        }

        state.robot_time_ns += 5000000;
        response["ok"] = true;
        response["state"] = stateJson();
        sendResponse(client, response);
        return true;
    }

    void sendResponse(int client, const nlohmann::json& response) {
        const std::string payload = response.dump() + "\n";
        (void)::send(client, payload.data(), payload.size(), 0);
    }

    int listen_fd_ = -1;
    int port_ = 0;
    std::atomic<bool> running_{true};
    std::atomic<int> connection_count_{0};
    std::thread thread_;
    std::mutex client_threads_mutex_;
    std::vector<std::thread> client_threads_;
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

rb_servo::DualArmConfig rbpodoControllerSimulationConfig() {
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.backend_type = rb_servo::BackendType::Rbpodo;
    cfg.left_robot.run_mode = rb_servo::RunMode::Real;
    cfg.left_robot.operation_mode = "simulation";
    cfg.right_robot.backend_type = rb_servo::BackendType::Rbpodo;
    cfg.right_robot.run_mode = rb_servo::RunMode::Real;
    cfg.right_robot.operation_mode = "simulation";
    cfg.servo.send_servo_commands = true;
    cfg.servo.allow_controller_simulation_motion = true;
    cfg.safety.tracking_error_policy = rb_servo::TrackingErrorPolicy::FaultLatch;
    cfg.safety.stop_both_arms_on_single_arm_error = true;
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

void configureCartesianLoopTest(rb_servo::DualArmConfig* cfg) {
    cfg->left_mount.arm_id = rb_servo::ArmId::Left;
    cfg->right_mount.arm_id = rb_servo::ArmId::Right;
    cfg->kinematics.enable = true;
    cfg->kinematics.ik.enable = true;
    cfg->kinematics.publish_tcp = true;
    cfg->cartesian_control.max_twist_linear_m_s = 10.0;
    cfg->cartesian_control.max_twist_angular_rad_s = 10.0;
}

rb_servo::DualArmCommand leftTcpTwistStandCommand() {
    rb_servo::DualArmCommand twist = command(rb_servo::ControlMode::Hold);
    twist.left.mode = rb_servo::ControlMode::TcpTwistStand;
    twist.left.has_tcp_twist_stand = true;
    twist.left.tcp_twist_stand = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0};
    twist.right.mode = rb_servo::ControlMode::Hold;
    return twist;
}

bool runLeftTcpTwistStandCase(
    rb_servo::DualArmConfig cfg,
    rb_servo::ServoSnapshot* snapshot,
    bool* left_twist_observed,
    bool accept_send_without_state_update = false,
    bool wait_for_fault = false
) {
    rb_servo::CommandBuffer buffer;
    const rb_servo::JointArray initial = joints(0.0);
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Left,
            initial,
            false,
            rb_servo::BackendErrorKind::ControllerRejected,
            std::nullopt,
            accept_send_without_state_update
        ),
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Right,
            initial,
            false,
            rb_servo::BackendErrorKind::ControllerRejected,
            std::nullopt,
            accept_send_without_state_update
        ),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );
    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    buffer.setCommand(leftTcpTwistStandCommand());
    RB_CHECK(waitUntil([&] {
        *snapshot = loop.latestSnapshot();
        if (wait_for_fault) {
            return snapshot->fault_latched &&
                   snapshot->safety_verdict == rb_servo::SafetyVerdict::FaultLatched;
        }
        return snapshot->command.left.mode == rb_servo::ControlMode::TcpTwistStand &&
               (snapshot->safety_verdict == rb_servo::SafetyVerdict::Ok ||
                snapshot->safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable ||
                snapshot->safety_verdict == rb_servo::SafetyVerdict::FaultLatched);
    }, std::chrono::milliseconds(1000)));
    *left_twist_observed = kinematics->lastLeftTwist().has_value();
    loop.stop();
    return true;
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

int reserveLoopbackTcpPort() {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    int reuse = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

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
    RB_CHECK(server->connectionCount() == 1);

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
    for (int i = 0; i < 8; ++i) {
        target[0] += 0.1;
        request.q_target_deg = target;
        RB_CHECK(backend.sendServoJ(request).accepted);
        RB_CHECK(backend.readState().ok);
    }
    RB_CHECK(server->connectionCount() == 1);
    rb_servo::RbsimTransportCounters counters = backend.transportCounters();
    RB_CHECK(counters.connect_attempts_total == 1);
    RB_CHECK(counters.connect_failures_total == 0);
    RB_CHECK(counters.connect_attempts_suppressed_total == 0);
    RB_CHECK(counters.connections_opened_total == 1);
    RB_CHECK(counters.reconnects_total == 0);
    RB_CHECK(counters.requests_total >= 20);
    RB_CHECK(counters.read_syscalls_total < counters.requests_total * 2);
    RB_CHECK(counters.next_connect_attempt_delay_ms == 0);
    const double last_accepted_target0 = target[0];

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
    RB_CHECK(!state_result.ok);
    RB_CHECK(state_result.error.kind == rb_servo::BackendErrorKind::TransportConnectFailed);
    RB_CHECK(state_result.error.name == "rbsim_connect_backoff");
    RB_CHECK(state_result.error.retryable);
    RB_CHECK(state_result.error.transport_fault);
    counters = backend.transportCounters();
    RB_CHECK(counters.connect_attempts_suppressed_total >= 1);
    RB_CHECK(counters.last_connect_error_name == "rbsim_connect_backoff");
    RB_CHECK(counters.next_connect_attempt_delay_ms > 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    state_result = backend.readState();
    RB_CHECK(state_result.ok);
    state = state_result.value;
    RB_CHECK(state.q_target_deg[0] == last_accepted_target0);
    RB_CHECK(server->connectionCount() == 2);
    counters = backend.transportCounters();
    RB_CHECK(counters.connections_opened_total == 2);
    RB_CHECK(counters.reconnects_total == 1);
    RB_CHECK(counters.connect_attempts_total == 2);
    RB_CHECK(counters.connect_failures_total == 0);
    RB_CHECK(counters.next_connect_attempt_delay_ms == 0);
    RB_CHECK(counters.last_transport_error_kind.has_value());
    RB_CHECK(*counters.last_transport_error_kind == rb_servo::BackendErrorKind::TransportWriteFailed);

    server->setFault("left", 2222);
    const int connections_before_fault = server->connectionCount();
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
    RB_CHECK(server->connectionCount() == connections_before_fault);
    RB_CHECK(backend.transportCounters().connections_opened_total == counters.connections_opened_total);

    RB_CHECK(backend.stop().ok);
    RB_CHECK(backend.resetFault().ok);
    RB_CHECK(server->connectionCount() == connections_before_fault);
    return true;
}

bool testRbsimPersistentTransportReconnectsAfterSocketDrop() {
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
    cfg.name = "left_rbsim_reconnect_test";
    cfg.rbsim_control_endpoint = server->endpoint();
    cfg.rbsim_request_timeout_sec = 0.2;

    rb_servo::RbsimBackend backend(rb_servo::ArmId::Left, cfg);
    RB_CHECK(backend.connect().ok);
    RB_CHECK(backend.initialize().ok);
    RB_CHECK(server->connectionCount() == 1);

    server->dropNextRequest("left");
    const rb_servo::BackendResult<rb_servo::RobotState> dropped = backend.readState();
    RB_CHECK(!dropped.ok);
    RB_CHECK(dropped.error.kind == rb_servo::BackendErrorKind::TransportReadFailed);
    rb_servo::RbsimTransportCounters counters = backend.transportCounters();
    RB_CHECK(counters.connections_opened_total == 1);
    RB_CHECK(counters.reconnects_total == 0);
    RB_CHECK(counters.last_transport_error_kind.has_value());
    RB_CHECK(*counters.last_transport_error_kind == rb_servo::BackendErrorKind::TransportReadFailed);
    RB_CHECK(counters.next_connect_attempt_delay_ms > 0);

    const rb_servo::BackendResult<rb_servo::RobotState> suppressed = backend.readState();
    RB_CHECK(!suppressed.ok);
    RB_CHECK(suppressed.error.kind == rb_servo::BackendErrorKind::TransportConnectFailed);
    RB_CHECK(suppressed.error.name == "rbsim_connect_backoff");
    counters = backend.transportCounters();
    RB_CHECK(counters.connect_attempts_suppressed_total >= 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    const rb_servo::BackendResult<rb_servo::RobotState> recovered = backend.readState();
    RB_CHECK(recovered.ok);
    RB_CHECK(server->connectionCount() == 2);
    counters = backend.transportCounters();
    RB_CHECK(counters.connections_opened_total == 2);
    RB_CHECK(counters.reconnects_total == 1);
    return true;
}

bool testRbsimReconnectBackoffSuppressesStormAndResetsAfterSuccess() {
    const int port = reserveLoopbackTcpPort();
    if (port <= 0) {
        std::cerr << "[SKIP] loopback TCP port fixture unavailable\n";
        return true;
    }

    rb_servo::BackendConfig cfg;
    cfg.backend_type = rb_servo::BackendType::Rbsim;
    cfg.run_mode = rb_servo::RunMode::Simulation;
    cfg.name = "left_rbsim_backoff_test";
    cfg.rbsim_control_endpoint = "tcp://127.0.0.1:" + std::to_string(port);
    cfg.simulator_control_endpoint = cfg.rbsim_control_endpoint;
    cfg.rbsim_request_timeout_sec = 0.05;
    cfg.rbsim_connect_timeout_sec = 0.05;
    cfg.rbsim_read_timeout_sec = 0.05;

    rb_servo::RbsimBackend backend(rb_servo::ArmId::Left, cfg);
    rb_servo::BackendResult<rb_servo::RobotState> first = backend.readState();
    RB_CHECK(!first.ok);
    RB_CHECK(first.error.kind == rb_servo::BackendErrorKind::TransportConnectFailed);
    RB_CHECK(first.error.name == "rbsim_connect_failed");
    RB_CHECK(first.error.retryable);
    RB_CHECK(first.error.transport_fault);

    rb_servo::RbsimTransportCounters counters = backend.transportCounters();
    RB_CHECK(counters.connect_attempts_total == 1);
    RB_CHECK(counters.connect_failures_total == 1);
    RB_CHECK(counters.connect_attempts_suppressed_total == 0);
    RB_CHECK(counters.connections_opened_total == 0);
    RB_CHECK(counters.last_connect_error_name == "rbsim_connect_failed");
    RB_CHECK(!counters.last_connect_error_message.empty());
    RB_CHECK(counters.next_connect_attempt_delay_ms > 0);
    RB_CHECK(counters.next_connect_attempt_delay_ms <= 50);

    rb_servo::BackendResult<rb_servo::RobotState> second = backend.readState();
    RB_CHECK(!second.ok);
    RB_CHECK(second.error.kind == rb_servo::BackendErrorKind::TransportConnectFailed);
    RB_CHECK(second.error.name == "rbsim_connect_backoff");
    RB_CHECK(second.error.retryable);
    RB_CHECK(second.error.transport_fault);

    rb_servo::RbsimTransportCounters after_suppressed = backend.transportCounters();
    RB_CHECK(after_suppressed.connect_attempts_total == counters.connect_attempts_total);
    RB_CHECK(after_suppressed.connect_failures_total == counters.connect_failures_total);
    RB_CHECK(after_suppressed.connect_attempts_suppressed_total == counters.connect_attempts_suppressed_total + 1);
    RB_CHECK(after_suppressed.last_connect_error_name == "rbsim_connect_backoff");
    RB_CHECK(contains(after_suppressed.last_connect_error_message, "next retry"));
    RB_CHECK(after_suppressed.next_connect_attempt_delay_ms <= counters.next_connect_attempt_delay_ms);

    std::unique_ptr<ScriptedRbsimServer> server;
    try {
        server = std::make_unique<ScriptedRbsimServer>(port);
    } catch (const std::exception& exc) {
        std::cerr << "[SKIP] rbsim fixed-port fixture unavailable: " << exc.what() << "\n";
        return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    rb_servo::BackendResult<rb_servo::RobotState> recovered = backend.readState();
    RB_CHECK(recovered.ok);
    RB_CHECK(server->connectionCount() == 1);

    rb_servo::RbsimTransportCounters recovered_counters = backend.transportCounters();
    RB_CHECK(recovered_counters.connect_attempts_total == after_suppressed.connect_attempts_total + 1);
    RB_CHECK(recovered_counters.connect_failures_total == after_suppressed.connect_failures_total);
    RB_CHECK(recovered_counters.connections_opened_total == 1);
    RB_CHECK(recovered_counters.reconnects_total == 0);
    RB_CHECK(recovered_counters.next_connect_attempt_delay_ms == 0);
    RB_CHECK(recovered_counters.read_syscalls_total > 0);
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
    RB_CHECK(snapshot.right_send_ok || snapshot.right_send_error_kind == "SuppressedByPolicy");
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

    rb_servo::BackendResult<rb_servo::RobotState> state_result = backend.readState();
    RB_CHECK(!state_result.ok);
    RB_CHECK(state_result.error.name == "rbsim_connect_backoff");
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    state_result = backend.readState();
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
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpLinearMove"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpCircleMove"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpDeltaStand"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpDeltaLocal"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpTwistStand"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpTwistLocal"})", now, &out));

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
    RB_CHECK(!out.left.tcp_target_stand.quaternion_xyzw.has_value());
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

    RB_CHECK(!server.parseMessage(
        R"({"schema_version":1,"seq":4,"mode":"TcpPoseTarget","timeout_sec":0.2,"left":{"tcp_target_stand":{"x":0.3,"y":0.1,"z":0.5,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,0,0]}},"right":{"tcp_target_stand":{"x":0.3,"y":-0.1,"z":0.5,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,0,1]}}})",
        now,
        &out
    ));

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":4,"mode":"TcpPoseTarget","timeout_sec":0.2,"left":{"tcp_target_stand":{"x":0.3,"y":0.1,"z":0.5,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,1,0]}},"right":{"tcp_target_stand":{"x":0.3,"y":-0.1,"z":0.5,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,0,1]}}})",
        now,
        &out
    ));
    RB_CHECK(out.left.has_tcp_target);
    RB_CHECK(out.left.tcp_target_stand.quaternion_xyzw.has_value());
    RB_CHECK(std::abs(out.left.tcp_target_stand.quaternion_xyzw->at(2) - 1.0) < kEpsilon);
    RB_CHECK(out.right.tcp_target_stand.quaternion_xyzw.has_value());
    RB_CHECK(std::abs(out.right.tcp_target_stand.quaternion_xyzw->at(3) - 1.0) < kEpsilon);

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":5,"mode":"TcpLinearMove","timeout_sec":0.2,"left":{"target_tcp_stand":{"x":0.35,"y":0.11,"z":0.55,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,0.70710678118,0.70710678118]},"duration_sec":2.0,"orientation_mode":"slerp"},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::TcpLinearMove);
    RB_CHECK(out.left.has_tcp_target);
    RB_CHECK(out.left.has_linear_move_duration);
    RB_CHECK(out.left.has_linear_move_orientation_mode);
    RB_CHECK(out.left.linear_move_orientation_mode == rb_servo::LinearMoveOrientationMode::Slerp);
    RB_CHECK(std::abs(out.left.tcp_target_stand.x - 0.35) < kEpsilon);
    RB_CHECK(out.left.tcp_target_stand.quaternion_xyzw.has_value());
    RB_CHECK(std::abs(out.left.tcp_target_stand.quaternion_xyzw->at(2) - 0.70710678118) < kEpsilon);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::Hold);

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":6,"mode":"TcpLinearMove","timeout_sec":0.2,"left":{"target_tcp_stand":{"x":0.35,"y":0.11,"z":0.55,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,0,1]},"linear_speed_m_s":0.05},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::TcpLinearMove);
    RB_CHECK(out.left.has_linear_move_linear_speed);
    RB_CHECK(std::abs(out.left.linear_move_linear_speed_m_s - 0.05) < kEpsilon);

    RB_CHECK(!server.parseMessage(
        R"({"schema_version":1,"seq":7,"mode":"TcpLinearMove","timeout_sec":0.2,"left":{"target_tcp_stand":{"x":0.35,"y":0.11,"z":0.55,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,0,1]},"duration_sec":0.0},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(!server.parseMessage(
        R"({"schema_version":1,"seq":8,"mode":"TcpLinearMove","timeout_sec":0.2,"left":{"target_tcp_stand":{"x":0.35,"y":0.11,"z":0.55,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,0,1]},"linear_speed_m_s":-0.05},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(!server.parseMessage(
        R"({"schema_version":1,"seq":9,"mode":"TcpLinearMove","timeout_sec":0.2,"left":{"target_tcp_stand":{"x":0.35,"y":0.11,"z":0.55,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,0,1]},"duration_sec":1.0,"orientation_mode":"bad"},"right":{"mode":"Hold"}})",
        now,
        &out
    ));

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":10,"mode":"Hold","timeout_sec":0.2,"left":{"mode":"TcpTwistLocal","tcp_twist_local":[0.02,0,0,0,0,0]},"right":{"mode":"TcpTwistStand","tcp_twist_stand":[0,0.01,0,0,0,0.1]}})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::TcpTwistLocal);
    RB_CHECK(out.left.has_tcp_twist_local);
    RB_CHECK(std::abs(out.left.tcp_twist_local.x - 0.02) < kEpsilon);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::TcpTwistStand);
    RB_CHECK(out.right.has_tcp_twist_stand);
    RB_CHECK(std::abs(out.right.tcp_twist_stand.y - 0.01) < kEpsilon);
    RB_CHECK(std::abs(out.right.tcp_twist_stand.rz - 0.1) < kEpsilon);

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":11,"mode":"Hold","timeout_sec":0.2,"left":{"mode":"TcpCircleMove","plane":"xy","diameter_m":0.15,"period_sec":4.0,"repeat":2,"center_mode":"start_on_circle","orientation_mode":"constant","frame":"stand"},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::TcpCircleMove);
    RB_CHECK(out.left.has_tcp_circle_move);
    RB_CHECK(out.left.tcp_circle_move.plane == rb_servo::TcpCirclePlane::XY);
    RB_CHECK(std::abs(out.left.tcp_circle_move.diameter_m - 0.15) < kEpsilon);
    RB_CHECK(std::abs(out.left.tcp_circle_move.period_sec - 4.0) < kEpsilon);
    RB_CHECK(out.left.tcp_circle_move.repeat == 2);
    RB_CHECK(out.left.tcp_circle_move.center_mode == rb_servo::TcpCircleCenterMode::StartOnCircle);
    RB_CHECK(out.left.tcp_circle_move.orientation_mode == rb_servo::LinearMoveOrientationMode::Constant);
    RB_CHECK(out.left.tcp_circle_move.frame == rb_servo::TcpCircleFrame::Stand);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::Hold);

    RB_CHECK(!server.parseMessage(
        R"({"schema_version":1,"seq":12,"mode":"TcpCircleMove","timeout_sec":0.2,"left":{"plane":"bad","diameter_m":0.15,"period_sec":4.0,"repeat":1},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(!server.parseMessage(
        R"({"schema_version":1,"seq":13,"mode":"TcpCircleMove","timeout_sec":0.2,"left":{"plane":"xy","diameter_m":0.0,"period_sec":4.0,"repeat":1},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(!server.parseMessage(
        R"({"schema_version":1,"seq":14,"mode":"TcpCircleMove","timeout_sec":0.2,"left":{"plane":"xy","diameter_m":0.15,"period_sec":0.0,"repeat":1},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(!server.parseMessage(
        R"({"schema_version":1,"seq":15,"mode":"TcpCircleMove","timeout_sec":0.2,"left":{"plane":"xy","diameter_m":0.15,"period_sec":4.0,"repeat":0},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    return true;
}

bool testCartesianControllerUsesQuaternionPoseOrientation() {
    rb_servo::DualArmConfig cfg = testConfig();
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::CartesianController controller(
        cfg.left_mount,
        cfg.right_mount,
        cfg.cartesian_control,
        kinematics
    );

    const double s = std::sin(M_PI / 4.0);
    const double c = std::cos(M_PI / 4.0);
    rb_servo::Pose6D current;
    current.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, s, c};
    const rb_servo::Pose6D translated = controller.applyTcpDeltaLocal(
        current,
        rb_servo::Pose6D{0.01, 0.0, 0.0, 0.0, 0.0, 0.0}
    );
    RB_CHECK(std::abs(translated.x) < 1e-9);
    RB_CHECK(std::abs(translated.y - 0.01) < 1e-9);
    RB_CHECK(translated.quaternion_xyzw.has_value());
    const auto& translated_q = *translated.quaternion_xyzw;
    const double dot = translated_q[2] * s + translated_q[3] * c;
    RB_CHECK(std::abs(std::abs(dot) - 1.0) < 1e-9);

    rb_servo::NetworkConfig network;
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    rb_servo::DualArmCommand parsed;
    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":1,"mode":"TcpPoseTarget","timeout_sec":0.2,"left":{"tcp_target_stand":{"x":0.3,"y":0.1,"z":0.5,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,1,0]}},"right":{"tcp_target_stand":{"x":0.3,"y":-0.1,"z":0.5,"rx":0,"ry":0,"rz":0,"quaternion_xyzw":[0,0,0,1]}}})",
        rb_servo::nowSteadyNs(),
        &parsed
    ));

    rb_servo::RobotState state;
    state.arm_id = rb_servo::ArmId::Left;
    state.has_valid_joint_state = true;
    state.q_actual_deg = joints(0.0);
    const rb_servo::CartesianArmTargetResult result = controller.computeArmJointTarget(
        parsed.left,
        state,
        joints(0.0),
        rb_servo::RunMode::Simulation
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    const std::optional<rb_servo::Pose6D> target = kinematics->lastLeftTarget();
    RB_CHECK(target.has_value());
    RB_CHECK(target->quaternion_xyzw.has_value());
    RB_CHECK(std::abs(target->quaternion_xyzw->at(2) - 1.0) < kEpsilon);
    RB_CHECK(std::abs(target->quaternion_xyzw->at(3)) < kEpsilon);
    return true;
}

bool testCartesianControllerPureTranslationsPreserveQuaternionOrientation() {
    rb_servo::DualArmConfig cfg = testConfig();
    rb_servo::CartesianController controller(
        cfg.left_mount,
        cfg.right_mount,
        cfg.cartesian_control,
        std::make_shared<FakeCartesianKinematics>()
    );

    const double s = std::sin(M_PI / 4.0);
    const double c = std::cos(M_PI / 4.0);
    rb_servo::Pose6D current;
    current.x = 0.3;
    current.y = -0.2;
    current.z = 0.7;
    current.rx = 0.0;
    current.ry = 0.0;
    current.rz = 0.0;
    current.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, s, c};
    const rb_servo::Pose6D delta{0.01, -0.02, 0.03, 0.0, 0.0, 0.0};

    const auto same_orientation = [&](const rb_servo::Pose6D& pose) {
        if (!pose.quaternion_xyzw.has_value()) return false;
        const auto& q = *pose.quaternion_xyzw;
        const double dot = q[2] * s + q[3] * c;
        return std::abs(std::abs(dot) - 1.0) < 1e-9;
    };

    RB_CHECK(same_orientation(controller.applyTcpDeltaStand(current, delta)));
    RB_CHECK(same_orientation(controller.applyTcpDeltaLocal(current, delta)));
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

bool testCommandSourceMetadataAndLeaseEnforcement() {
    rb_servo::NetworkConfig permissive_network;
    permissive_network.command_timeout_sec = 0.35;
    rb_servo::CommandBuffer permissive_buffer;
    rb_servo::CommandServer permissive(permissive_network, &permissive_buffer);
    rb_servo::DualArmCommand out;
    const uint64_t now = rb_servo::nowSteadyNs();

    RB_CHECK(permissive.parseMessage(
        R"({"seq":1,"mode":"ArmMotion","source_id":"gui","session_id":"gui-session","lease_token":"gui-token","source_priority":10})",
        now,
        &out
    ));
    RB_CHECK(out.source.source_id == "gui");
    RB_CHECK(out.source.session_id == "gui-session");
    RB_CHECK(out.source.lease_token == "gui-token");
    RB_CHECK(out.source.source_priority.has_value());
    RB_CHECK(*out.source.source_priority == 10);
    RB_CHECK(out.lease.active);
    RB_CHECK(out.lease.source_id == "gui");
    RB_CHECK(out.lease.session_id == "gui-session");
    RB_CHECK(out.lease.lease_token == "gui-token");

    RB_CHECK(permissive.parseMessage(R"({"seq":1,"mode":"Hold","source_id":"policy","session_id":"policy-session"})", now + 1, &out));
    RB_CHECK(out.source.source_id == "policy");

    rb_servo::NetworkConfig enforcing_network;
    enforcing_network.command_source_enforce_lease = true;
    enforcing_network.command_source_lease_timeout_sec = 1.0;
    rb_servo::CommandBuffer enforcing_buffer;
    rb_servo::CommandServer enforcing(enforcing_network, &enforcing_buffer);

    RB_CHECK(enforcing.parseMessage(
        R"({"seq":1,"mode":"AcquireLease","source_id":"gui","session_id":"gui-session"})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(out.lease.active);
    RB_CHECK(out.lease.enforce_lease);
    RB_CHECK(out.lease.source_id == "gui");
    RB_CHECK(!out.lease.lease_token.empty());

    RB_CHECK(enforcing.parseMessage(
        R"({"seq":2,"mode":"JointTarget","source_id":"gui","session_id":"gui-session","q_target_deg":[1,2,3,4,5,6]})",
        now + 1,
        &out
    ));
    RB_CHECK(out.lease.command_requires_lease);
    RB_CHECK(out.lease.command_has_lease);

    RB_CHECK(!enforcing.parseMessage(
        R"({"seq":1,"mode":"JointTarget","source_id":"policy","session_id":"policy-session","q_target_deg":[1,2,3,4,5,6]})",
        now + 2,
        &out
    ));
    RB_CHECK(contains(enforcing.lastRejectReason(), "command_source_lease_required"));

    RB_CHECK(enforcing.parseMessage(
        R"({"seq":1,"mode":"EmergencyStop","source_id":"policy","session_id":"policy-session"})",
        now + 3,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::EmergencyStop);

    RB_CHECK(enforcing.parseMessage(
        R"({"seq":2,"mode":"ArmMotion","source_id":"policy","session_id":"policy-session"})",
        now + 1'100'000'000ULL,
        &out
    ));
    RB_CHECK(out.lease.active);
    RB_CHECK(out.lease.source_id == "policy");

    rb_servo::CommandBuffer legacy_buffer;
    rb_servo::CommandServer legacy_enforcing(enforcing_network, &legacy_buffer);
    RB_CHECK(legacy_enforcing.parseMessage(R"({"seq":1,"mode":"ArmMotion"})", now, &out));
    RB_CHECK(out.lease.active);
    RB_CHECK(out.lease.source_id.empty());
    RB_CHECK(legacy_enforcing.parseMessage(
        R"({"seq":2,"mode":"JointTarget","q_target_deg":[1,2,3,4,5,6]})",
        now + 1,
        &out
    ));
    RB_CHECK(out.lease.command_has_lease);
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
             << "  ip: 172.28.60.200\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "  ip: 172.28.60.201\n"
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
             << "  ip: 172.28.60.200\n"
             << "  servo_t1_sec: 0.005\n"
             << "  servo_t2_sec: 0.05\n"
             << "  servo_alpha: 0.5\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "  ip: 172.28.60.201\n"
             << "  servo_t1_sec: 0.005\n"
             << "  servo_t2_sec: 0.05\n"
             << "  servo_alpha: 0.5\n"
             << "servo:\n"
             << "  enable_realtime_priority: true\n"
             << "  send_servo_commands: false\n"
             << "  allow_readonly_faulted_startup: true\n"
             << "  allow_readonly_q_range_violation_startup: true\n"
             << "  allow_readonly_wrong_mode_startup: true\n"
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
    RB_CHECK(read_only_cfg.servo.allow_readonly_faulted_startup);
    RB_CHECK(read_only_cfg.servo.allow_readonly_q_range_violation_startup);
    RB_CHECK(read_only_cfg.servo.allow_readonly_wrong_mode_startup);
    ::unlink(read_only_path.c_str());

    const std::string motion_path = "/tmp/rb-servo-real-motion-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(motion_path);
        file << "left_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "  ip: 172.28.60.200\n"
             << "  servo_t1_sec: 0.005\n"
             << "  servo_t2_sec: 0.05\n"
             << "  servo_alpha: 0.5\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "  ip: 172.28.60.201\n"
             << "  servo_t1_sec: 0.005\n"
             << "  servo_t2_sec: 0.05\n"
             << "  servo_alpha: 0.5\n"
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

    const std::string diagnostic_motion_path =
        "/tmp/rb-servo-read-only-diagnostic-motion-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(diagnostic_motion_path);
        file << "left_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "  ip: 172.28.60.200\n"
             << "  servo_t1_sec: 0.005\n"
             << "  servo_t2_sec: 0.05\n"
             << "  servo_alpha: 0.5\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "  ip: 172.28.60.201\n"
             << "  servo_t1_sec: 0.005\n"
             << "  servo_t2_sec: 0.05\n"
             << "  servo_alpha: 0.5\n"
             << "servo:\n"
             << "  enable_realtime_priority: true\n"
             << "  send_servo_commands: true\n"
             << "  allow_readonly_faulted_startup: true\n"
             << "safety:\n"
             << "  tracking_error_policy: fault_latch\n";
    }
    bool rejected_motion_diagnostic = false;
    try {
        (void)rb_servo::loadConfigFromYaml(diagnostic_motion_path);
    } catch (const std::exception& exc) {
        rejected_motion_diagnostic = contains(exc.what(), "allow_readonly");
    }
    ::unlink(diagnostic_motion_path.c_str());
    RB_CHECK(rejected_motion_diagnostic);
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
    RB_CHECK(!snapshot.left_send_ok);
    RB_CHECK(!snapshot.right_send_ok);
    RB_CHECK(snapshot.left_last_send.backend_error_kind == "SuppressedByPolicy");
    RB_CHECK(snapshot.right_last_send.backend_error_kind == "SuppressedByPolicy");
    RB_CHECK(!snapshot.left_last_send.accepted);
    RB_CHECK(!snapshot.right_last_send.accepted);
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
    RB_CHECK(snapshot.send_policy == "read_only");
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
    snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.send_policy == "emergency_latched");
    RB_CHECK(snapshot.send_suppressed);

    loop.stop();
    return true;
}

bool testWorkerIoModeDispatchesThroughArmWorkers() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.io_model = rb_servo::ServoIoModel::Worker;
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
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] { return loop.motionState() == rb_servo::ServerMotionState::ArmedHold; }));

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(3.0);
    target.right.q_target_deg = joints(3.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);

    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        return snapshot.left_send_ok &&
               snapshot.right_send_ok &&
               sameJointArray(previous.left_q_target_deg, joints(3.0)) &&
               sameJointArray(previous.right_q_target_deg, joints(3.0));
    }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(snapshot.send_policy == "send_servo_j");
    RB_CHECK(snapshot.left_last_send.accepted);
    RB_CHECK(snapshot.right_last_send.accepted);
    RB_CHECK(left_backend->sendCount() > 0);
    RB_CHECK(right_backend->sendCount() > 0);
    RB_CHECK(left_backend->readCount() > 0);
    RB_CHECK(right_backend->readCount() > 0);
    return true;
}

bool testWorkerIoModeTimesOutMissingSendResultByDeadline() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.io_model = rb_servo::ServoIoModel::Worker;
    cfg.servo.rate_hz = 2;
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
    left_backend->setSendSleepMs(800);
    right_backend->setSendSleepMs(800);
    rb_servo::DualArmCommand arm_motion = command(rb_servo::ControlMode::ArmMotion);
    arm_motion.left.timeout_sec = 2.0;
    arm_motion.right.timeout_sec = 2.0;
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil(
        [&] { return loop.motionState() == rb_servo::ServerMotionState::ArmedHold; },
        std::chrono::milliseconds(1500)
    ));

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(4.0);
    target.right.q_target_deg = joints(4.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    target.left.timeout_sec = 0.6;
    target.right.timeout_sec = 0.6;
    buffer.setCommand(target);

    const bool saw_timeout = waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.left_last_send.backend_error_kind == "CommandTimeout" &&
               snapshot.right_last_send.backend_error_kind == "CommandTimeout";
    }, std::chrono::milliseconds(3000));
    RB_CHECK(saw_timeout);

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(!snapshot.left_send_ok);
    RB_CHECK(!snapshot.right_send_ok);
    RB_CHECK(snapshot.left_last_send.error_name == "arm_worker_send_result_timeout");
    RB_CHECK(snapshot.right_last_send.error_name == "arm_worker_send_result_timeout");
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::SendFailure ||
             snapshot.safety_verdict == rb_servo::SafetyVerdict::FaultLatched);
    return true;
}

bool testWorkerIoModeReportsMixedTimeoutAndAcceptedArm() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.io_model = rb_servo::ServoIoModel::Worker;
    cfg.servo.rate_hz = 2;
    cfg.safety.stop_both_arms_on_single_arm_error = true;
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
    left_backend->setSendSleepMs(800);
    rb_servo::DualArmCommand arm_motion = command(rb_servo::ControlMode::ArmMotion);
    arm_motion.left.timeout_sec = 2.0;
    arm_motion.right.timeout_sec = 2.0;
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil(
        [&] { return loop.motionState() == rb_servo::ServerMotionState::ArmedHold; },
        std::chrono::milliseconds(1500)
    ));

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(5.0);
    target.right.q_target_deg = joints(5.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    target.left.timeout_sec = 0.6;
    target.right.timeout_sec = 0.6;
    buffer.setCommand(target);

    std::optional<rb_servo::ServoSnapshot> mixed_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (snapshot.left_last_send.backend_error_kind == "CommandTimeout" &&
            snapshot.right_last_send.accepted) {
            mixed_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(3000)));

    loop.stop();
    RB_CHECK(mixed_snapshot.has_value());
    RB_CHECK(!mixed_snapshot->left_send_ok);
    RB_CHECK(mixed_snapshot->right_send_ok);
    RB_CHECK(mixed_snapshot->left_last_send.error_name == "arm_worker_send_result_timeout");
    RB_CHECK(mixed_snapshot->right_last_send.backend_error_kind == "None");
    RB_CHECK(mixed_snapshot->fault_latched);
    RB_CHECK(mixed_snapshot->latched_fault_reason == rb_servo::SafetyVerdict::SendFailure);
    RB_CHECK(left_backend->sendCount() > 0);
    RB_CHECK(right_backend->sendCount() > 0);
    return true;
}

bool testWorkerIoModeClassifiesStaleStateExplicitly() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.io_model = rb_servo::ServoIoModel::Worker;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* left_backend = left.get();
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    left_backend->setReadSleepMs(2000);
    const bool saw_stale = waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.left_last_read.backend_error_kind == "TransportTimeout" &&
               snapshot.left_last_read.error_name == "arm_worker_state_stale";
    }, std::chrono::milliseconds(2500));
    RB_CHECK(saw_stale);

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::FaultLatched ||
             snapshot.safety_verdict == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::RobotStateError);
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
    cfg.left_robot.backend_type = rb_servo::BackendType::Mock;
    cfg.right_robot.backend_type = rb_servo::BackendType::Mock;
    cfg.left_mount.base_pose_in_stand = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    cfg.right_mount.base_pose_in_stand = {-0.1, -0.2, 0.3, -0.4, -0.5, 0.6};
    cfg.cartesian_control.path_kp_pos = 7.5;
    cfg.cartesian_control.path_kp_ori = 8.5;
    cfg.cartesian_control.max_twist_linear_m_s = 0.045;
    cfg.cartesian_control.max_cartesian_step_m = 0.012;
    cfg.cartesian_control.exceed_limit_policy = rb_servo::CartesianLimitPolicy::Reject;
    cfg.cartesian_control.linear_move.default_linear_speed_m_s = 0.022;
    cfg.cartesian_control.linear_move.default_orientation_mode = rb_servo::LinearMoveOrientationMode::Slerp;
    cfg.kinematics.enable = true;
    cfg.kinematics.provider = "pinocchio";
    cfg.kinematics.base_link = "left_base";
    cfg.kinematics.tip_link = "left_tcp";
    cfg.kinematics.publish_tcp = true;
    cfg.kinematics.ik.damping = 0.004;
    cfg.kinematics.ik.max_iterations = 31;

    rb_servo::ServoSnapshot snapshot;
    snapshot.tick = 123;
    snapshot.loop_start_time_ns = 1'000;
    snapshot.loop_end_time_ns = 2'000;
    snapshot.period_ms = 5.0;
    snapshot.jitter_ms = 0.1;
    snapshot.filter_dt_ms = 5.0;
    snapshot.command.seq = 42;
    snapshot.command.source.source_id = "rb_gui";
    snapshot.command.source.session_id = "session-1";
    snapshot.command.source.lease_token = "lease-token";
    snapshot.command.lease.enforce_lease = true;
    snapshot.command.lease.active = true;
    snapshot.command.lease.source_id = "rb_gui";
    snapshot.command.lease.session_id = "session-1";
    snapshot.command.lease.lease_token = "lease-token";
    snapshot.command.lease.acquired_time_ns = 1'500;
    snapshot.command.lease.expires_time_ns = 3'000;
    snapshot.command.lease.command_requires_lease = true;
    snapshot.command.lease.command_has_lease = true;
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
    snapshot.left_state.servo_enabled = true;
    snapshot.right_state.servo_enabled = true;
    snapshot.left_state.fk_duration_us = 11.0;
    snapshot.right_state.fk_duration_us = 12.0;
    snapshot.left_state.has_error = true;
    snapshot.left_state.error_code = 2222;
    snapshot.left_state.fault_recoverable = true;
    snapshot.left_state.lifecycle_state = "faulted";
    snapshot.left_state.motion_readiness_error_kind = "RobotFault";
    snapshot.left_state.motion_readiness_error_name = "rbpodo_robot_fault";
    snapshot.left_state.diagnostic_error_source = "rbpodo_robot_fault";
    snapshot.left_sent_q_deg = joints(3.0);
    snapshot.right_sent_q_deg = joints(4.0);
    snapshot.left_prev_sent_q_deg = joints(5.0);
    snapshot.right_prev_sent_q_deg = joints(6.0);
    snapshot.left_send_ok = false;
    snapshot.right_send_ok = true;
    snapshot.left_last_read.ok = false;
    snapshot.left_last_read.backend_error_kind = "RobotFault";
    snapshot.left_last_read.error_name = "fault_latched";
    snapshot.left_last_read.error_code = "2222";
    snapshot.left_last_read.duration_us = 12.0;
    snapshot.right_last_read.ok = true;
    snapshot.left_last_send.ok = true;
    snapshot.left_last_send.accepted = false;
    snapshot.left_last_send.backend_error_kind = "SuppressedByPolicy";
    snapshot.left_last_send.error_name = "read_only";
    snapshot.left_last_send.duration_us = 0.0;
    snapshot.left_last_send.state_after_source = "none";
    snapshot.left_last_send.ack_policy = rb_servo::BackendAckPolicy::Disabled;
    snapshot.left_last_send.ack_observed = false;
    snapshot.left_last_send.controller_acceptance_observed = false;
    snapshot.left_last_send.ack_wait_duration_us = 0.0;
    snapshot.left_last_send.rbpodo_waiting_ack = false;
    snapshot.left_last_send.acceptance_semantics = "not_sent";
    snapshot.right_last_send.ok = true;
    snapshot.right_last_send.accepted = true;
    snapshot.right_last_send.backend_error_kind = "None";
    snapshot.right_last_send.error_name = "None";
    snapshot.right_last_send.duration_us = 10.0;
    snapshot.right_last_send.state_after_source = "response";
    snapshot.right_last_send.ack_policy = rb_servo::BackendAckPolicy::Wait;
    snapshot.right_last_send.ack_observed = true;
    snapshot.right_last_send.controller_acceptance_observed = true;
    snapshot.right_last_send.ack_wait_duration_us = 123.0;
    snapshot.right_last_send.rbpodo_waiting_ack = true;
    snapshot.right_last_send.acceptance_semantics = "controller_ack_observed";
    snapshot.left_cartesian_solve.attempted = true;
    snapshot.left_cartesian_solve.success = true;
    snapshot.left_cartesian_solve.status = "ok";
    snapshot.left_cartesian_solve.reason = "";
    snapshot.left_cartesian_solve.fk_duration_us = 11.0;
    snapshot.left_cartesian_solve.ik_duration_us = 125.0;
    snapshot.left_cartesian_solve.ik_iterations = 7;
    snapshot.left_cartesian_solve.position_error_m = 0.001;
    snapshot.left_cartesian_solve.orientation_error_rad = 0.002;
    snapshot.left_cartesian_solve.path_active = true;
    snapshot.left_cartesian_solve.path_s = 0.5;
    snapshot.left_cartesian_solve.path_position_error_m = 0.003;
    snapshot.left_cartesian_solve.path_orientation_error_rad = 0.004;
    snapshot.left_cartesian_solve.path_line_deviation_m = 0.0005;
    snapshot.left_cartesian_solve.path_done = false;
    snapshot.left_cartesian_solve.linear_move_duration_sec = 2.0;
    snapshot.left_cartesian_solve.linear_move_elapsed_sec = 1.0;
    snapshot.left_cartesian_solve.orientation_mode = "constant";
    snapshot.left_cartesian_solve.warn_ik_duration_us = 3000.0;
    snapshot.right_cartesian_solve.attempted = true;
    snapshot.right_cartesian_solve.success = false;
    snapshot.right_cartesian_solve.status = "failed";
    snapshot.right_cartesian_solve.reason = "timeout";
    snapshot.right_cartesian_solve.fk_duration_us = 12.0;
    snapshot.right_cartesian_solve.ik_duration_us = 5000.0;
    snapshot.right_cartesian_solve.ik_iterations = 50;
    snapshot.right_cartesian_solve.position_error_m = 0.03;
    snapshot.right_cartesian_solve.orientation_error_rad = 0.04;
    snapshot.right_cartesian_solve.ik_timed_out = true;
    snapshot.right_cartesian_solve.ik_warn_duration_exceeded = true;
    snapshot.right_cartesian_solve.warn_ik_duration_us = 3000.0;
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
    snapshot.startup_validation.acquisition_ok = true;
    snapshot.startup_validation.motion_ready = false;
    snapshot.startup_validation.read_only_diagnostic = true;
    snapshot.startup_validation.allowed_unsafe_startup = true;
    snapshot.startup_validation.left.acquisition_ok = true;
    snapshot.startup_validation.left.motion_ready = false;
    snapshot.startup_validation.left.read_only_diagnostic = true;
    snapshot.startup_validation.left.allowed_unsafe_startup = true;
    snapshot.startup_validation.left.invalid_reasons = {"robot_fault", "q_range_violation"};
    snapshot.startup_validation.left.diagnostic_error_source = "error_code:2222";
    snapshot.startup_validation.left.q_range_violations.push_back({3, 250.0, -180.0, 180.0});
    snapshot.startup_validation.right.acquisition_ok = true;
    snapshot.startup_validation.right.motion_ready = true;
    snapshot.startup_validation.right.read_only_diagnostic = true;
    snapshot.logger_dropped_samples = 0;

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));

    const char* top_keys[] = {
        "schema_version", "tick", "host_time_ns", "loop_start_time_ns", "loop_end_time_ns",
        "period_ms", "jitter_ms", "filter_dt_ms", "command_seq", "command_source", "observed_mode", "observed_backend",
        "cartesian_control_snapshot", "kinematics_snapshot", "startup_validation", "left", "right",
        "send_skew_us", "send_within_period", "send_period_overrun", "send_command_deadline_missed",
        "send_deadline_hit", "send_deadline_hit_deprecated_alias_for",
        "send_suppressed", "send_policy", "safety_verdict", "motion_state", "fault_latched",
        "latched_fault_reason", "fault_reason", "logger_dropped_samples", "logger_health",
        "fault_context", "mount_transform_deferred", "mounts", "tcp_fields_deferred",
        "last_cartesian_solve"
    };
    for (const char* key : top_keys) {
        RB_CHECK(json.contains(key));
    }
    const char* arm_keys[] = {
        "mode", "q_actual_deg", "q_sent_deg", "q_previous_sent_deg", "send_ok",
        "send_start_ns", "send_end_ns", "send_duration_us", "has_valid_joint_state",
        "startup_acquisition_ok", "startup_motion_ready", "startup_invalid_reasons",
        "q_range_violations", "read_only_diagnostic", "allowed_unsafe_startup",
        "diagnostic_error_source", "connection_state", "has_error", "servo_enabled",
        "fault_recoverable", "lifecycle_state", "motion_readiness_error_kind",
        "motion_readiness_error_name",
        "last_read", "last_send", "robot_time_ns", "host_time_ns", "error_code", "state_age_us",
        "send_within_period", "send_period_overrun", "send_command_deadline_missed",
        "send_deadline_hit", "send_deadline_hit_deprecated_alias_for",
        "tcp_stand", "tcp_base", "tcp_deferred", "fk_duration_us", "cartesian_solve", "worker",
        "transport", "cartesian_available", "cartesian_unavailable_reason", "cartesian_gate",
        "controller_simulation_cartesian_enabled", "streaming_cartesian_physical_real_enabled",
        "tracking_error_source", "tracking_error_source_valid", "tracking_error_reason",
        "command_reference_tracking_error_deg", "physical_command_actual_error_deg",
        "controller_simulation_physical_motion_detected"
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
    RB_CHECK(json.at("command_source").at("source_id").get<std::string>() == "rb_gui");
    RB_CHECK(json.at("command_source").at("session_id").get<std::string>() == "session-1");
    RB_CHECK(json.at("command_source").at("active").get<bool>());
    RB_CHECK(json.at("command_source").at("command_requires_lease").get<bool>());
    RB_CHECK(json.at("command_source").at("command_has_lease").get<bool>());
    RB_CHECK(json.at("command_source").at("active_source_id").get<std::string>() == "rb_gui");
    RB_CHECK(json.at("observed_mode").get<std::string>() == "mock");
    RB_CHECK(json.at("observed_backend").get<std::string>() == "mock");
    RB_CHECK(json.at("cartesian_control_snapshot").at("schema").get<std::string>() == "robotics_lab.cartesian_control_snapshot.v1");
    RB_CHECK(json.at("cartesian_control_snapshot").at("enable").get<bool>());
    RB_CHECK(json.at("cartesian_control_snapshot").at("allow_in_simulation").get<bool>());
    RB_CHECK(!json.at("cartesian_control_snapshot").at("allow_in_real").get<bool>());
    RB_CHECK(!json.at("cartesian_control_snapshot").at("allow_in_controller_simulation").get<bool>());
    RB_CHECK(json.at("cartesian_control_snapshot").at("path_kp_pos").get<double>() == 7.5);
    RB_CHECK(json.at("cartesian_control_snapshot").at("path_kp_ori").get<double>() == 8.5);
    RB_CHECK(json.at("cartesian_control_snapshot").at("max_twist_linear_m_s").get<double>() == 0.045);
    RB_CHECK(json.at("cartesian_control_snapshot").at("max_cartesian_step_m").get<double>() == 0.012);
    RB_CHECK(json.at("cartesian_control_snapshot").at("max_cartesian_step_rad").is_null());
    RB_CHECK(json.at("cartesian_control_snapshot").at("exceed_limit_policy").get<std::string>() == "reject");
    RB_CHECK(json.at("cartesian_control_snapshot").at("linear_move").at("default_linear_speed_m_s").get<double>() == 0.022);
    RB_CHECK(json.at("cartesian_control_snapshot").at("linear_move").at("default_orientation_mode").get<std::string>() == "slerp");
    RB_CHECK(json.at("kinematics_snapshot").at("schema").get<std::string>() == "robotics_lab.kinematics_snapshot.v1");
    RB_CHECK(json.at("kinematics_snapshot").at("enable").get<bool>());
    RB_CHECK(json.at("kinematics_snapshot").at("provider").get<std::string>() == "pinocchio");
    RB_CHECK(json.at("kinematics_snapshot").at("base_link").get<std::string>() == "left_base");
    RB_CHECK(json.at("kinematics_snapshot").at("tip_link").get<std::string>() == "left_tcp");
    RB_CHECK(json.at("kinematics_snapshot").at("publish_tcp").get<bool>());
    RB_CHECK(json.at("kinematics_snapshot").at("joint_names").size() == 6);
    RB_CHECK(json.at("kinematics_snapshot").at("ik").at("damping").get<double>() == 0.004);
    RB_CHECK(json.at("kinematics_snapshot").at("ik").at("max_iterations").get<int>() == 31);
    RB_CHECK(json.at("left").at("mode").get<std::string>() == "JointTarget");
    RB_CHECK(json.at("right").at("mode").get<std::string>() == "Hold");
    RB_CHECK(json.at("left").at("cartesian_gate").at("backend_type").get<std::string>() == "mock");
    RB_CHECK(!json.at("left").at("cartesian_gate").at("allow_in_controller_simulation").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_gate").at("controller_simulation_tracking_error_source").get<std::string>() ==
             "actual");
    RB_CHECK(json.at("left").at("cartesian_gate").at("controller_simulation_physical_motion_policy").get<std::string>() ==
             "fault_latch");
    RB_CHECK(json.at("left").at("tracking_error_source").get<std::string>() == "actual");
    RB_CHECK(json.at("left").at("tracking_error_source_valid").get<bool>());
    RB_CHECK(!json.at("left").at("streaming_cartesian_physical_real_enabled").get<bool>());
    RB_CHECK(jsonArrayHasSixFinite(json.at("left").at("q_actual_deg")));
    RB_CHECK(jsonArrayHasSixFinite(json.at("right").at("q_actual_deg")));
    RB_CHECK(jsonArrayHasSixFinite(json.at("left").at("q_sent_deg")));
    RB_CHECK(jsonArrayHasSixFinite(json.at("right").at("q_sent_deg")));
    RB_CHECK(jsonArrayHasSixFinite(json.at("left").at("q_previous_sent_deg")));
    RB_CHECK(jsonArrayHasSixFinite(json.at("right").at("q_previous_sent_deg")));
    RB_CHECK(!json.at("left").at("send_ok").get<bool>());
    RB_CHECK(json.at("right").at("send_ok").get<bool>());
    RB_CHECK(json.at("left").at("has_error").get<bool>());
    RB_CHECK(json.at("startup_validation").at("acquisition_ok").get<bool>());
    RB_CHECK(!json.at("startup_validation").at("motion_ready").get<bool>());
    RB_CHECK(json.at("startup_validation").at("read_only_diagnostic").get<bool>());
    RB_CHECK(json.at("startup_validation").at("allowed_unsafe_startup").get<bool>());
    RB_CHECK(!json.at("startup_validation").at("left").at("motion_ready").get<bool>());
    RB_CHECK(json.at("left").at("startup_acquisition_ok").get<bool>());
    RB_CHECK(!json.at("left").at("startup_motion_ready").get<bool>());
    RB_CHECK(json.at("left").at("read_only_diagnostic").get<bool>());
    RB_CHECK(json.at("left").at("allowed_unsafe_startup").get<bool>());
    RB_CHECK(json.at("left").at("startup_invalid_reasons").size() == 2);
    RB_CHECK(json.at("left").at("q_range_violations").size() == 1);
    RB_CHECK(json.at("left").at("q_range_violations").at(0).at("joint").get<int>() == 3);
    RB_CHECK(json.at("left").at("diagnostic_error_source").get<std::string>() == "error_code:2222");
    RB_CHECK(json.at("right").at("startup_motion_ready").get<bool>());
    RB_CHECK(json.at("right").at("q_range_violations").empty());
    RB_CHECK(json.at("left").at("servo_enabled").get<bool>());
    RB_CHECK(json.at("left").at("fault_recoverable").get<bool>());
    RB_CHECK(json.at("left").at("lifecycle_state").get<std::string>() == "faulted");
    RB_CHECK(json.at("left").at("motion_readiness_error_kind").get<std::string>() == "RobotFault");
    RB_CHECK(json.at("left").at("motion_readiness_error_name").get<std::string>() == "rbpodo_robot_fault");
    RB_CHECK(json.at("right").at("fault_recoverable").is_null());
    RB_CHECK(json.at("right").at("lifecycle_state").is_null());
    RB_CHECK(json.at("right").at("motion_readiness_error_kind").is_null());
    RB_CHECK(!json.at("left").at("last_read").at("ok").get<bool>());
    RB_CHECK(json.at("left").at("last_read").at("backend_error_kind").get<std::string>() == "RobotFault");
    RB_CHECK(json.at("left").at("last_read").at("error_name").get<std::string>() == "fault_latched");
    RB_CHECK(json.at("left").at("last_read").at("error_code").get<std::string>() == "2222");
    RB_CHECK(!json.at("left").at("last_send").at("accepted").get<bool>());
    RB_CHECK(json.at("left").at("last_send").at("backend_error_kind").get<std::string>() == "SuppressedByPolicy");
    RB_CHECK(json.at("left").at("last_send").at("error_name").get<std::string>() == "read_only");
    RB_CHECK(json.at("left").at("last_send").at("state_after_source").get<std::string>() == "none");
    RB_CHECK(json.at("left").at("last_send").at("ack_policy").get<std::string>() == "disabled");
    RB_CHECK(!json.at("left").at("last_send").at("ack_observed").get<bool>());
    RB_CHECK(!json.at("left").at("last_send").at("controller_acceptance_observed").get<bool>());
    RB_CHECK(json.at("left").at("last_send").at("send_acceptance_semantics").get<std::string>() == "not_sent");
    RB_CHECK(json.at("right").at("last_send").at("accepted").get<bool>());
    RB_CHECK(json.at("right").at("last_send").at("state_after_source").get<std::string>() == "response");
    RB_CHECK(json.at("right").at("last_send").at("ack_policy").get<std::string>() == "wait");
    RB_CHECK(json.at("right").at("last_send").at("ack_observed").get<bool>());
    RB_CHECK(json.at("right").at("last_send").at("controller_acceptance_observed").get<bool>());
    RB_CHECK(json.at("right").at("last_send").at("ack_wait_duration_us").get<double>() == 123.0);
    RB_CHECK(json.at("right").at("last_send").at("rbpodo_waiting_ack").get<bool>());
    RB_CHECK(json.at("right").at("last_send").at("send_acceptance_semantics").get<std::string>() == "controller_ack_observed");
    RB_CHECK(json.at("left").at("send_start_ns").get<uint64_t>() == 10);
    RB_CHECK(json.at("left").at("send_end_ns").get<uint64_t>() == 20);
    RB_CHECK(json.at("right").at("send_start_ns").get<uint64_t>() == 30);
    RB_CHECK(json.at("right").at("send_end_ns").get<uint64_t>() == 40);
    RB_CHECK(json.at("left").at("host_time_ns").get<uint64_t>() == 11'000);
    RB_CHECK(json.at("right").at("robot_time_ns").get<uint64_t>() == 22'000);
    RB_CHECK(json.at("send_skew_us").get<double>() == 20.0);
    RB_CHECK(json.at("send_within_period").get<bool>());
    RB_CHECK(!json.at("send_period_overrun").get<bool>());
    RB_CHECK(json.at("send_command_deadline_missed").is_null());
    RB_CHECK(json.at("send_deadline_hit").get<bool>());
    RB_CHECK(json.at("send_deadline_hit_deprecated_alias_for").get<std::string>() == "send_within_period");
    RB_CHECK(json.at("send_suppressed").get<bool>());
    RB_CHECK(json.at("send_policy").get<std::string>() == "read_only");
    RB_CHECK(json.at("left").at("send_duration_us").get<double>() == 10.0);
    RB_CHECK(json.at("right").at("send_duration_us").get<double>() == 10.0);
    RB_CHECK(json.at("left").at("send_within_period").get<bool>());
    RB_CHECK(!json.at("left").at("send_period_overrun").get<bool>());
    RB_CHECK(json.at("left").at("send_command_deadline_missed").is_null());
    RB_CHECK(json.at("left").at("send_deadline_hit").get<bool>());
    RB_CHECK(json.at("left").at("send_deadline_hit_deprecated_alias_for").get<std::string>() == "send_within_period");
    RB_CHECK(json.at("safety_verdict").get<std::string>() == "Ok");
    RB_CHECK(json.at("motion_state").get<std::string>() == "Running");
    RB_CHECK(!json.at("fault_latched").get<bool>());
    RB_CHECK(json.at("latched_fault_reason").get<std::string>() == "Ok");
    RB_CHECK(json.at("fault_reason").get<std::string>().empty());
    RB_CHECK(!json.at("fault_context").at("latched").get<bool>());
    RB_CHECK(json.at("fault_context").at("motion_state").get<std::string>() == "Running");
    RB_CHECK(json.at("fault_context").at("backend_error_kind").is_null());
    RB_CHECK(json.at("fault_context").at("top_level").is_null());
    RB_CHECK(json.at("fault_context").at("left").is_null());
    RB_CHECK(json.at("fault_context").at("right").is_null());
    RB_CHECK(json.at("logger_dropped_samples").get<uint64_t>() == 0);
    RB_CHECK(json.at("logger_health").at("ok").get<bool>());
    RB_CHECK(!json.at("mount_transform_deferred").get<bool>());
    RB_CHECK(json.at("mounts").at("left").at("frame").get<std::string>() == "stand");
    RB_CHECK(json.at("mounts").at("right").at("base_pose_in_stand").at("x").get<double>() == -0.1);
    RB_CHECK(json.at("tcp_fields_deferred").get<bool>());
    RB_CHECK(json.at("left").at("tcp_stand").is_null());
    RB_CHECK(json.at("left").at("tcp_base").is_null());
    RB_CHECK(json.at("left").at("tcp_deferred").get<bool>());
    RB_CHECK(json.at("left").at("fk_duration_us").get<double>() == 11.0);
    RB_CHECK(json.at("left").at("cartesian_solve").at("attempted").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_solve").at("status").get<std::string>() == "ok");
    RB_CHECK(json.at("left").at("cartesian_solve").at("ik_status").get<std::string>() == "ok");
    RB_CHECK(json.at("left").at("cartesian_solve").at("ik_duration_us").get<double>() == 125.0);
    RB_CHECK(json.at("left").at("cartesian_solve").at("ik_iterations").get<int>() == 7);
    RB_CHECK(json.at("left").at("cartesian_solve").at("position_error_m").get<double>() == 0.001);
    RB_CHECK(json.at("left").at("cartesian_solve").at("orientation_error_rad").get<double>() == 0.002);
    RB_CHECK(json.at("left").at("cartesian_solve").at("path_active").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_solve").at("path_s").get<double>() == 0.5);
    RB_CHECK(json.at("left").at("cartesian_solve").at("path_position_error_m").get<double>() == 0.003);
    RB_CHECK(json.at("left").at("cartesian_solve").at("path_orientation_error_rad").get<double>() == 0.004);
    RB_CHECK(json.at("left").at("cartesian_solve").at("path_line_deviation_m").get<double>() == 0.0005);
    RB_CHECK(!json.at("left").at("cartesian_solve").at("path_done").get<bool>());
    RB_CHECK(!json.at("left").at("cartesian_solve").at("path_completion_hold").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_solve").at("path_elapsed_sec").get<double>() == 1.0);
    RB_CHECK(json.at("left").at("cartesian_solve").at("linear_move_duration_sec").get<double>() == 2.0);
    RB_CHECK(json.at("left").at("cartesian_solve").at("linear_move_elapsed_sec").get<double>() == 1.0);
    RB_CHECK(json.at("left").at("cartesian_solve").at("orientation_mode").get<std::string>() == "constant");
    RB_CHECK(json.at("right").at("tcp_stand").is_null());
    RB_CHECK(json.at("right").at("tcp_base").is_null());
    RB_CHECK(json.at("right").at("tcp_deferred").get<bool>());
    RB_CHECK(json.at("right").at("fk_duration_us").get<double>() == 12.0);
    RB_CHECK(json.at("right").at("cartesian_solve").at("ik_timed_out").get<bool>());
    RB_CHECK(json.at("right").at("cartesian_solve").at("ik_warn_duration_exceeded").get<bool>());
    RB_CHECK(json.at("last_cartesian_solve").at("right").at("ik_reason").get<std::string>() == "timeout");
    RB_CHECK(json.at("last_cartesian_solve").at("right").at("orientation_error_rad").get<double>() == 0.04);
    RB_CHECK(!json.at("left").at("worker").at("enabled").get<bool>());
    RB_CHECK(json.at("left").at("worker").at("queue_policy").get<std::string>() == "latest_wins");
    RB_CHECK(json.at("left").at("worker").at("read_period_ns").get<uint64_t>() == 0);
    RB_CHECK(json.at("left").at("worker").at("read_period_sec").get<double>() == 0.0);
    RB_CHECK(json.at("left").at("worker").at("read_rate_hz").get<double>() == 0.0);
    RB_CHECK(json.at("left").at("worker").at("state_age_us").get<double>() == 0.0);
    RB_CHECK(json.at("left").at("worker").at("command_drops_total").get<uint64_t>() == 0);
    RB_CHECK(json.at("left").at("worker").at("pending_overwrites_total").get<uint64_t>() == 0);
    RB_CHECK(json.at("left").at("worker").at("last_dropped_seq").get<uint64_t>() == 0);
    RB_CHECK(json.at("left").at("worker").at("last_enqueued_seq").get<uint64_t>() == 0);
    RB_CHECK(json.at("left").at("worker").at("last_dispatched_seq").get<uint64_t>() == 0);
    RB_CHECK(json.at("left").at("worker").at("last_completed_seq").get<uint64_t>() == 0);
    RB_CHECK(json.at("left").at("transport").is_null());

    snapshot.fault_latched = true;
    snapshot.motion_state = rb_servo::ServerMotionState::FaultLatched;
    snapshot.latched_fault_reason = rb_servo::SafetyVerdict::RobotStateError;
    snapshot.fault_reason = "robot/controller fault during ReadState on left";
    rb_servo::LatchedFaultContextSnapshot latched_context;
    latched_context.verdict = "RobotStateError";
    latched_context.domain = "RobotState";
    latched_context.arm = "left";
    latched_context.backend_op = "ReadState";
    latched_context.backend_error_kind = "RobotFault";
    latched_context.backend_error_name = "fault_latched";
    latched_context.backend_error_code = "2222";
    latched_context.retryable = false;
    latched_context.recoverable = true;
    latched_context.robot_fault = true;
    latched_context.transport_fault = false;
    latched_context.state_after_source = "response";
    latched_context.reason = "robot/controller fault during ReadState on left";
    snapshot.latched_fault_context = latched_context;
    snapshot.left_latched_fault_context = latched_context;
    rb_servo::LatchedFaultContextSnapshot right_latched_context;
    right_latched_context.verdict = "SendFailure";
    right_latched_context.domain = "Backend";
    right_latched_context.arm = "right";
    right_latched_context.backend_op = "SendServoJ";
    right_latched_context.backend_error_kind = "TransportTimeout";
    right_latched_context.backend_error_name = "send_timeout";
    right_latched_context.retryable = true;
    right_latched_context.recoverable = true;
    right_latched_context.transport_fault = true;
    right_latched_context.state_after_source = "none";
    right_latched_context.reason = "transport failure during SendServoJ on right";
    snapshot.right_latched_fault_context = right_latched_context;

    const nlohmann::json latched_json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    RB_CHECK(latched_json.at("fault_context").at("latched").get<bool>());
    RB_CHECK(latched_json.at("fault_context").at("verdict").get<std::string>() == "RobotStateError");
    RB_CHECK(latched_json.at("fault_context").at("domain").get<std::string>() == "RobotState");
    RB_CHECK(latched_json.at("fault_context").at("arm").get<std::string>() == "left");
    RB_CHECK(latched_json.at("fault_context").at("backend_op").get<std::string>() == "ReadState");
    RB_CHECK(latched_json.at("fault_context").at("backend_error_kind").get<std::string>() == "RobotFault");
    RB_CHECK(latched_json.at("fault_context").at("backend_error_name").get<std::string>() == "fault_latched");
    RB_CHECK(latched_json.at("fault_context").at("backend_error_code").get<std::string>() == "2222");
    RB_CHECK(!latched_json.at("fault_context").at("retryable").get<bool>());
    RB_CHECK(latched_json.at("fault_context").at("recoverable").get<bool>());
    RB_CHECK(latched_json.at("fault_context").at("robot_fault").get<bool>());
    RB_CHECK(!latched_json.at("fault_context").at("transport_fault").get<bool>());
    RB_CHECK(latched_json.at("fault_context").at("state_after_source").get<std::string>() == "response");
    RB_CHECK(latched_json.at("fault_context").at("top_level").at("backend_error_kind").get<std::string>() == "RobotFault");
    RB_CHECK(latched_json.at("fault_context").at("left").at("backend_error_kind").get<std::string>() == "RobotFault");
    RB_CHECK(latched_json.at("fault_context").at("left").at("arm").get<std::string>() == "left");
    RB_CHECK(latched_json.at("fault_context").at("right").at("backend_error_kind").get<std::string>() == "TransportTimeout");
    RB_CHECK(latched_json.at("fault_context").at("right").at("arm").get<std::string>() == "right");
    RB_CHECK(latched_json.at("fault_context").at("right").at("transport_fault").get<bool>());
    RB_CHECK(latched_json.at("left").at("last_send").at("backend_error_kind").get<std::string>() == "SuppressedByPolicy");

    rb_servo::DualArmConfig worker_cfg = cfg;
    worker_cfg.servo.io_model = rb_servo::ServoIoModel::Worker;
    worker_cfg.servo.worker_read_period_sec = 0.025;
    rb_servo::ServoSnapshot worker_snapshot = snapshot;
    worker_snapshot.loop_end_time_ns = 20'000;
    worker_snapshot.left_worker_telemetry.worker_command_drops_total = 3;
    worker_snapshot.left_worker_telemetry.worker_pending_overwrites_total = 3;
    worker_snapshot.left_worker_telemetry.worker_last_dropped_seq = 1201;
    worker_snapshot.left_worker_telemetry.worker_last_enqueued_seq = 1204;
    worker_snapshot.left_worker_telemetry.worker_last_dispatched_seq = 1204;
    worker_snapshot.left_worker_telemetry.worker_last_completed_seq = 1204;
    rb_servo::BackendTransportTelemetry transport;
    transport.connect_attempts_total = 2;
    transport.connect_failures_total = 1;
    transport.connect_attempts_suppressed_total = 3;
    transport.connections_opened_total = 1;
    transport.reconnects_total = 1;
    transport.requests_total = 12;
    transport.read_syscalls_total = 13;
    transport.write_syscalls_total = 14;
    transport.last_connect_error_name = "rbsim_connect_backoff";
    transport.last_connect_error_message = "next retry in 50 ms";
    transport.next_connect_attempt_ns = 123456;
    transport.next_connect_attempt_delay_ms = 50;
    transport.last_transport_error_kind = "TransportReadFailed";
    worker_snapshot.left_transport_telemetry = transport;
    rb_servo::StatePublisher worker_publisher(worker_cfg);
    const nlohmann::json worker_json =
        nlohmann::json::parse(worker_publisher.serializeSnapshot(worker_snapshot));
    const nlohmann::json& worker = worker_json.at("left").at("worker");
    RB_CHECK(worker.at("enabled").get<bool>());
    RB_CHECK(worker.at("queue_policy").get<std::string>() == "latest_wins");
    RB_CHECK(worker.at("read_period_ns").get<uint64_t>() == 25'000'000);
    RB_CHECK(worker.at("read_period_sec").get<double>() == 0.025);
    RB_CHECK(worker.at("read_rate_hz").get<double>() == 40.0);
    RB_CHECK(worker.at("state_age_us").get<double>() == 9.0);
    RB_CHECK(worker.at("command_drops_total").get<uint64_t>() == 3);
    RB_CHECK(worker.at("pending_overwrites_total").get<uint64_t>() == 3);
    RB_CHECK(worker.at("last_dropped_seq").get<uint64_t>() == 1201);
    RB_CHECK(worker.at("last_enqueued_seq").get<uint64_t>() == 1204);
    RB_CHECK(worker.at("last_dispatched_seq").get<uint64_t>() == 1204);
    RB_CHECK(worker.at("last_completed_seq").get<uint64_t>() == 1204);
    const nlohmann::json& transport_json = worker_json.at("left").at("transport");
    RB_CHECK(transport_json.at("connect_attempts_total").get<uint64_t>() == 2);
    RB_CHECK(transport_json.at("connect_failures_total").get<uint64_t>() == 1);
    RB_CHECK(transport_json.at("connect_attempts_suppressed_total").get<uint64_t>() == 3);
    RB_CHECK(transport_json.at("connections_opened_total").get<uint64_t>() == 1);
    RB_CHECK(transport_json.at("reconnects_total").get<uint64_t>() == 1);
    RB_CHECK(transport_json.at("requests_total").get<uint64_t>() == 12);
    RB_CHECK(transport_json.at("read_syscalls_total").get<uint64_t>() == 13);
    RB_CHECK(transport_json.at("write_syscalls_total").get<uint64_t>() == 14);
    RB_CHECK(transport_json.at("last_connect_error_name").get<std::string>() == "rbsim_connect_backoff");
    RB_CHECK(transport_json.at("last_connect_error_message").get<std::string>() == "next retry in 50 ms");
    RB_CHECK(transport_json.at("next_connect_attempt_ns").get<uint64_t>() == 123456);
    RB_CHECK(transport_json.at("next_connect_attempt_delay_ms").get<uint64_t>() == 50);
    RB_CHECK(transport_json.at("last_transport_error_kind").get<std::string>() == "TransportReadFailed");
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

bool testStatePublisherUsesConfiguredPublishRate() {
    const int port = reserveLoopbackUdpPort();
    if (port <= 0) {
        std::cerr << "SKIP testStatePublisherUsesConfiguredPublishRate: loopback UDP unavailable\n";
        return true;
    }

    rb_servo::DualArmConfig cfg = testConfig();
    cfg.network.state_pub_bind = "udp://127.0.0.1:" + std::to_string(port);
    cfg.network.state_pub_endpoint = cfg.network.state_pub_bind;
    cfg.network.state_pub_rate_hz = 100;

    std::atomic<int> provider_calls{0};
    rb_servo::StatePublisher publisher(cfg, [&provider_calls]() {
        ++provider_calls;
        rb_servo::ServoSnapshot snapshot;
        snapshot.tick = static_cast<uint64_t>(provider_calls.load());
        return snapshot;
    });

    RB_CHECK(publisher.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(180));
    publisher.stop();

    const int calls = provider_calls.load();
    RB_CHECK(calls >= 8);
    RB_CHECK(calls <= 30);
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

bool testReadOnlyDiagnosticStartupAllowsFaultedStateAndPublishesUnsafeSnapshot() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.send_servo_commands = false;
    cfg.servo.allow_readonly_faulted_startup = true;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    right->setHasError(true);
    right->setErrorCode(1977952848);

    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    RB_CHECK(waitUntil([&] { return loop.latestSnapshot().loop_end_time_ns > 0; }));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.right_state.has_error);
    RB_CHECK(snapshot.right_state.error_code == 1977952848);
    RB_CHECK(snapshot.send_suppressed);
    RB_CHECK(snapshot.send_policy == "read_only");
    RB_CHECK(snapshot.startup_validation.acquisition_ok);
    RB_CHECK(!snapshot.startup_validation.motion_ready);
    RB_CHECK(snapshot.startup_validation.read_only_diagnostic);
    RB_CHECK(snapshot.startup_validation.allowed_unsafe_startup);
    RB_CHECK(snapshot.startup_validation.right.acquisition_ok);
    RB_CHECK(!snapshot.startup_validation.right.motion_ready);
    RB_CHECK(snapshot.startup_validation.right.allowed_unsafe_startup);
    RB_CHECK(containsValue(snapshot.startup_validation.right.invalid_reasons, "robot_fault"));
    RB_CHECK(snapshot.startup_validation.right.diagnostic_error_source == "error_code:1977952848");
    loop.stop();
    return true;
}

bool testMotionStartupRejectsFaultedState() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.send_servo_commands = true;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    left->setHasError(true);
    left->setErrorCode(2222);

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

bool testRbpodoControllerSimulationMotionRequiresExplicitGate() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    EnvVarGuard allow_controller_sim("RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION");
    EnvVarGuard pgmode_confirmed("RB_RBPODO_PGMODE_SIMULATION_CONFIRMED");
    allow_real.unset();
    allow_motion.unset();
    allow_controller_sim.unset();
    pgmode_confirmed.unset();

    rb_servo::CommandBuffer buffer;
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();

    {
        rb_servo::DualArmServoLoop loop(
            std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    allow_real.set("1");
    allow_motion.set("1");
    allow_controller_sim.set("1");
    {
        rb_servo::DualArmServoLoop loop(
            std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    pgmode_confirmed.set("1");
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );
    RB_CHECK(loop.start());
    sleepTicks();
    RB_CHECK(!loop.latestSnapshot().fault_latched);
    loop.stop();
    return true;
}

bool testRbpodoControllerSimulationDiagnosticOverrideIsNarrow() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    EnvVarGuard allow_controller_sim("RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION");
    EnvVarGuard allow_diag("RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM");
    EnvVarGuard pgmode_confirmed("RB_RBPODO_PGMODE_SIMULATION_CONFIRMED");
    allow_real.set("1");
    allow_motion.set("1");
    allow_controller_sim.set("1");
    allow_diag.unset();
    pgmode_confirmed.set("1");

    const rb_servo::JointArray initial = joints(0.0);
    const auto diagnostic_backend = [&]() {
        auto backend = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
        backend->setHasError(true);
        backend->setErrorCode(-2001);
        backend->setMotionReadinessError(
            "RobotFault",
            "rbpodo_diagnostics_suspect",
            "rbpodo_diagnostics_suspect"
        );
        return backend;
    };
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();

    {
        rb_servo::DualArmServoLoop loop(
            diagnostic_backend(),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    cfg.servo.allow_controller_simulation_diagnostics_suspect = true;
    {
        rb_servo::DualArmServoLoop loop(
            diagnostic_backend(),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    allow_diag.set("1");
    auto left = diagnostic_backend();
    TestBackend* left_raw = left.get();
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );
    RB_CHECK(loop.start());
    RB_CHECK(waitUntil([&] { return loop.latestSnapshot().loop_end_time_ns > 0; }));
    RB_CHECK(left_raw->sendCount() > 0);
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.left_state.has_error);
    RB_CHECK(snapshot.left_state.diagnostic_error_source == "rbpodo_diagnostics_suspect");
    RB_CHECK(snapshot.startup_validation.allowed_unsafe_startup);
    RB_CHECK(snapshot.startup_validation.left.allowed_unsafe_startup);
    RB_CHECK(containsValue(snapshot.startup_validation.left.invalid_reasons, "robot_fault"));

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    RB_CHECK(json.at("left").at("controller_simulation_diagnostic_override_active").get<bool>());
    RB_CHECK(!json.at("left").at("physical_motion_expected").get<bool>());
    RB_CHECK(json.at("left")
        .at("controller_simulation_mode")
        .at("controller_simulation_diagnostic_override_active")
        .get<bool>());
    loop.stop();
    return true;
}

bool testRbpodoControllerSimulationDiagnosticOverrideRejectsHardFaults() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    EnvVarGuard allow_controller_sim("RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION");
    EnvVarGuard allow_diag("RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM");
    EnvVarGuard pgmode_confirmed("RB_RBPODO_PGMODE_SIMULATION_CONFIRMED");
    allow_real.set("1");
    allow_motion.set("1");
    allow_controller_sim.set("1");
    allow_diag.set("1");
    pgmode_confirmed.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.allow_controller_simulation_diagnostics_suspect = true;

    {
        rb_servo::JointArray non_finite = joints(0.0);
        non_finite[0] = std::numeric_limits<double>::quiet_NaN();
        auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, non_finite, false);
        left->setHasError(true);
        left->setMotionReadinessError(
            "RobotFault",
            "rbpodo_diagnostics_suspect",
            "rbpodo_diagnostics_suspect"
        );
        rb_servo::DualArmServoLoop loop(
            std::move(left),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(0.0), false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    {
        rb_servo::JointArray out_of_range = joints(0.0);
        out_of_range[2] = 250.0;
        auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, out_of_range, false);
        left->setHasError(true);
        left->setMotionReadinessError(
            "RobotFault",
            "rbpodo_diagnostics_suspect",
            "rbpodo_diagnostics_suspect"
        );
        rb_servo::DualArmServoLoop loop(
            std::move(left),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(0.0), false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    {
        auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, joints(0.0), false);
        left->setHasError(true);
        left->setErrorCode(1002);
        left->setMotionReadinessError(
            "RobotFault",
            "rbpodo_ems_flag",
            "rbpodo_ems_flag"
        );
        rb_servo::DualArmServoLoop loop(
            std::move(left),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(0.0), false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    cfg.left_robot.operation_mode = "real";
    {
        auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, joints(0.0), false);
        left->setHasError(true);
        left->setMotionReadinessError(
            "RobotFault",
            "rbpodo_diagnostics_suspect",
            "rbpodo_diagnostics_suspect"
        );
        rb_servo::DualArmServoLoop loop(
            std::move(left),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(0.0), false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    return true;
}

bool testReadOnlyDiagnosticStartupAllowsRangeViolationOnlyWhenConfigured() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.send_servo_commands = false;
    rb_servo::JointArray out_of_range = joints(0.0);
    out_of_range[2] = 250.0;
    {
        rb_servo::DualArmServoLoop loop(
            std::make_unique<TestBackend>(rb_servo::ArmId::Left, out_of_range, false),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(0.0), false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    cfg.servo.allow_readonly_q_range_violation_startup = true;
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, out_of_range, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(0.0), false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    RB_CHECK(waitUntil([&] { return loop.latestSnapshot().loop_end_time_ns > 0; }));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.startup_validation.acquisition_ok);
    RB_CHECK(!snapshot.startup_validation.motion_ready);
    RB_CHECK(snapshot.startup_validation.allowed_unsafe_startup);
    RB_CHECK(containsValue(snapshot.startup_validation.left.invalid_reasons, "q_range_violation"));
    RB_CHECK(snapshot.startup_validation.left.q_range_violations.size() == 1);
    RB_CHECK(snapshot.startup_validation.left.q_range_violations.front().joint == 3);
    RB_CHECK(snapshot.startup_validation.left.q_range_violations.front().value_deg == 250.0);
    RB_CHECK(snapshot.send_suppressed);
    RB_CHECK(snapshot.send_policy == "read_only");
    loop.stop();
    return true;
}

bool testMotionStartupRejectsRangeViolation() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.send_servo_commands = true;
    rb_servo::JointArray out_of_range = joints(0.0);
    out_of_range[2] = 250.0;
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, out_of_range, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(0.0), false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(!loop.start());
    return true;
}

bool testReadOnlyDiagnosticStartupAllowsWrongModeOnlyWhenConfigured() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.send_servo_commands = false;
    const rb_servo::JointArray initial = joints(0.0);
    {
        auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
        auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
        left->setMotionReadinessError(
            "WrongMode",
            "rbpodo_wrong_operation_mode",
            "rbpodo_wrong_operation_mode"
        );
        rb_servo::DualArmServoLoop loop(
            std::move(left),
            std::move(right),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    cfg.servo.allow_readonly_wrong_mode_startup = true;
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    left->setMotionReadinessError(
        "WrongMode",
        "rbpodo_wrong_operation_mode",
        "rbpodo_wrong_operation_mode"
    );
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    RB_CHECK(waitUntil([&] { return loop.latestSnapshot().loop_end_time_ns > 0; }));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(!snapshot.startup_validation.motion_ready);
    RB_CHECK(snapshot.startup_validation.allowed_unsafe_startup);
    RB_CHECK(containsValue(snapshot.startup_validation.left.invalid_reasons, "wrong_mode"));
    RB_CHECK(snapshot.startup_validation.left.diagnostic_error_source == "rbpodo_wrong_operation_mode");
    RB_CHECK(snapshot.send_suppressed);
    RB_CHECK(snapshot.send_policy == "read_only");
    loop.stop();
    return true;
}

bool testMotionStartupRejectsWrongMode() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.send_servo_commands = true;
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, joints(0.0), false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(0.0), false);
    left->setMotionReadinessError(
        "WrongMode",
        "rbpodo_wrong_operation_mode",
        "rbpodo_wrong_operation_mode"
    );
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

bool testReadOnlyDiagnosticStartupRejectsAcquisitionFailure() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.send_servo_commands = false;
    cfg.servo.allow_readonly_faulted_startup = true;
    cfg.servo.allow_readonly_q_range_violation_startup = true;
    cfg.servo.allow_readonly_wrong_mode_startup = true;
    rb_servo::JointArray non_finite = joints(0.0);
    non_finite[0] = std::numeric_limits<double>::quiet_NaN();
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, non_finite, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(0.0), false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(!loop.start());
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
    RB_CHECK(!loop.latestSnapshot().latched_fault_context.has_value());

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
    kinematics->setSolveError(0.0015, 0.0025);
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
               snapshot.left_cartesian_solve.attempted &&
               snapshot.left_cartesian_solve.status == "ok" &&
               snapshot.left_cartesian_solve.ik_duration_us == 125.0 &&
               snapshot.left_cartesian_solve.ik_iterations == 7 &&
               snapshot.left_cartesian_solve.position_error_m == 0.0015 &&
               snapshot.left_cartesian_solve.orientation_error_rad == 0.0025 &&
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

bool testTcpLinearMoveUsesIkInSimulationOnly() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.right_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.left_mount.arm_id = rb_servo::ArmId::Left;
    cfg.right_mount.arm_id = rb_servo::ArmId::Right;
    cfg.kinematics.enable = true;
    cfg.kinematics.ik.enable = true;
    cfg.kinematics.publish_tcp = true;
    cfg.cartesian_control.max_linear_move_speed_m_s = 1.0;
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

    rb_servo::DualArmCommand linear = command(rb_servo::ControlMode::Hold);
    linear.left.mode = rb_servo::ControlMode::TcpLinearMove;
    linear.left.has_tcp_target = true;
    linear.left.linear_move_duration_sec = 1.0;
    linear.left.has_linear_move_duration = true;
    linear.left.tcp_target_stand = {0.05, 0.02, 0.01, 0.0, 0.0, 0.0};
    linear.left.tcp_target_stand.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
    linear.right.mode = rb_servo::ControlMode::Hold;
    buffer.setCommand(linear);
    double max_orientation_error = 0.0;
    double max_line_deviation = 0.0;
    double observed_duration = 0.0;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        observed_duration = snapshot.left_cartesian_solve.linear_move_duration_sec;
        max_orientation_error = std::max(
            max_orientation_error,
            snapshot.left_cartesian_solve.path_orientation_error_rad
        );
        max_line_deviation = std::max(
            max_line_deviation,
            snapshot.left_cartesian_solve.path_line_deviation_m
        );
        return snapshot.command.left.mode == rb_servo::ControlMode::TcpLinearMove &&
               snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok &&
               snapshot.left_cartesian_solve.status == "ok" &&
               snapshot.left_cartesian_solve.path_s > 0.0 &&
               kinematics->lastLeftTwist().has_value();
    }, std::chrono::milliseconds(1000)));
    RB_CHECK(std::abs(observed_duration - 1.0) < 1e-9);
    rb_servo::ServoSnapshot continued_snapshot;
    const bool continued = waitUntil([&] {
        continued_snapshot = loop.latestSnapshot();
        return continued_snapshot.command.left.mode == rb_servo::ControlMode::Hold &&
               continued_snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok &&
               continued_snapshot.left_cartesian_solve.status == "ok" &&
               continued_snapshot.left_cartesian_solve.path_active &&
               continued_snapshot.left_cartesian_solve.path_s > 0.2 &&
               continued_snapshot.left_cartesian_solve.path_s < 1.0;
    }, std::chrono::milliseconds(1000));
    RB_CHECK(continued);
    RB_CHECK(std::abs(continued_snapshot.left_cartesian_solve.linear_move_duration_sec - 1.0) < 1e-9);
    RB_CHECK(max_orientation_error < 1e-9);
    RB_CHECK(max_line_deviation < 2e-3);
    RB_CHECK(std::abs(kinematics->lastLeftTwist()->rz) < 1e-9);
    rb_servo::ServoSnapshot done_snapshot;
    RB_CHECK(waitUntil([&] {
        done_snapshot = loop.latestSnapshot();
        return done_snapshot.command.left.mode == rb_servo::ControlMode::Hold &&
               done_snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok &&
               done_snapshot.left_cartesian_solve.status == "ok" &&
               done_snapshot.left_cartesian_solve.path_done &&
               !done_snapshot.left_cartesian_solve.path_active &&
               done_snapshot.left_cartesian_solve.path_s >= 1.0;
    }, std::chrono::milliseconds(1500)));
    sleepTicks();
    const rb_servo::ServoSnapshot held_done_snapshot = loop.latestSnapshot();
    RB_CHECK(held_done_snapshot.command.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(held_done_snapshot.left_cartesian_solve.status == "ok");
    RB_CHECK(held_done_snapshot.left_cartesian_solve.path_done);
    RB_CHECK(!held_done_snapshot.left_cartesian_solve.path_active);
    RB_CHECK(held_done_snapshot.left_cartesian_solve.path_s >= 1.0);
    rb_servo::DualArmCommand replacement = command(rb_servo::ControlMode::Hold);
    replacement.seq = 2;
    replacement.left.mode = rb_servo::ControlMode::TcpLinearMove;
    replacement.left.has_tcp_target = true;
    replacement.left.linear_move_duration_sec = 0.5;
    replacement.left.has_linear_move_duration = true;
    replacement.left.tcp_target_stand = {0.08, 0.02, 0.01, 0.0, 0.0, 0.0};
    replacement.left.tcp_target_stand.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
    replacement.right.mode = rb_servo::ControlMode::Hold;
    buffer.setCommand(replacement);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.command.left.mode == rb_servo::ControlMode::TcpLinearMove &&
               snapshot.left_cartesian_solve.status == "ok" &&
               std::abs(snapshot.left_cartesian_solve.linear_move_duration_sec - 0.5) < 1e-9 &&
               snapshot.left_cartesian_solve.path_s < continued_snapshot.left_cartesian_solve.path_s;
    }));
    rb_servo::DualArmCommand hold = command(rb_servo::ControlMode::Hold);
    hold.seq = 3;
    buffer.setCommand(hold);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.command.left.mode == rb_servo::ControlMode::Hold &&
               snapshot.left_cartesian_solve.status == "not_attempted" &&
               !snapshot.left_cartesian_solve.path_active;
    }));
    loop.stop();

    rb_servo::CommandBuffer real_buffer;
    rb_servo::DualArmConfig real_cfg = cfg;
    real_cfg.left_robot.run_mode = rb_servo::RunMode::Real;
    real_cfg.right_robot.run_mode = rb_servo::RunMode::Real;
    real_cfg.cartesian_control.allow_in_real = true;
    auto real_kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop real_loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        real_cfg,
        &real_buffer,
        nullptr,
        real_kinematics
    );
    RB_CHECK(real_loop.start());
    real_buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    linear.seq = 2;
    linear.host_time_ns = rb_servo::nowSteadyNs();
    real_buffer.setCommand(linear);
    sleepTicks();
    const rb_servo::ServoSnapshot real_snapshot = real_loop.latestSnapshot();
    RB_CHECK(real_snapshot.safety_verdict == rb_servo::SafetyVerdict::InvalidCommand ||
             real_snapshot.safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable);
    RB_CHECK(!real_kinematics->lastLeftTwist().has_value());
    real_loop.stop();
    return true;
}

bool testStreamingCartesianSimulationStillAvailable() {
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.right_robot.run_mode = rb_servo::RunMode::Simulation;
    configureCartesianLoopTest(&cfg);

    rb_servo::ServoSnapshot snapshot;
    bool twist_observed = false;
    RB_CHECK(runLeftTcpTwistStandCase(cfg, &snapshot, &twist_observed));
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(twist_observed);
    return true;
}

bool testRbpodoControllerSimulationStartupReferenceSource() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    EnvVarGuard allow_controller_sim("RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION");
    EnvVarGuard pgmode_confirmed("RB_RBPODO_PGMODE_SIMULATION_CONFIRMED");

    allow_real.set("1");
    allow_motion.set("1");
    allow_controller_sim.set("1");
    pgmode_confirmed.set("1");

    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.safety.controller_simulation_tracking_error_source =
        rb_servo::ControllerSimulationTrackingErrorSource::Reference;
    cfg.safety.max_tracking_error_deg = 10.0;

    const rb_servo::JointArray actual = joints(0.0);
    rb_servo::JointArray reference = joints(20.0);

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Left,
            actual,
            false,
            rb_servo::BackendErrorKind::ControllerRejected,
            reference,
            true
        ),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, actual, false),
        cfg,
        &buffer,
        nullptr
    );
    RB_CHECK(loop.start());
    sleepTicks();
    rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    rb_servo::ServoTarget previous = loop.previousSentTarget();
    RB_CHECK(!snapshot.fault_latched);
    RB_CHECK(sameJointArray(previous.left_q_target_deg, reference));
    RB_CHECK(sameJointArray(previous.right_q_target_deg, actual));
    loop.stop();

    rb_servo::CommandBuffer physical_motion_buffer;
    rb_servo::DualArmServoLoop physical_motion_loop(
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Left,
            actual,
            false,
            rb_servo::BackendErrorKind::ControllerRejected,
            reference,
            false
        ),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, actual, false),
        cfg,
        &physical_motion_buffer,
        nullptr
    );
    RB_CHECK(physical_motion_loop.start());
    RB_CHECK(waitUntil([&] {
        snapshot = physical_motion_loop.latestSnapshot();
        return snapshot.fault_latched &&
               snapshot.latched_fault_reason == rb_servo::SafetyVerdict::TrackingError &&
               snapshot.fault_reason == "controller_simulation_physical_motion_detected";
    }, std::chrono::milliseconds(1000)));
    physical_motion_loop.stop();

    rb_servo::DualArmConfig physical_real_cfg = cfg;
    physical_real_cfg.left_robot.operation_mode = "real";
    physical_real_cfg.right_robot.operation_mode = "real";
    rb_servo::CommandBuffer physical_real_buffer;
    rb_servo::DualArmServoLoop physical_real_loop(
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Left,
            actual,
            false,
            rb_servo::BackendErrorKind::ControllerRejected,
            reference,
            true
        ),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, actual, false),
        physical_real_cfg,
        &physical_real_buffer,
        nullptr
    );
    RB_CHECK(physical_real_loop.start());
    sleepTicks();
    previous = physical_real_loop.previousSentTarget();
    snapshot = physical_real_loop.latestSnapshot();
    RB_CHECK(!snapshot.fault_latched);
    RB_CHECK(sameJointArray(previous.left_q_target_deg, actual));
    physical_real_loop.stop();

    rb_servo::JointArray invalid_reference = reference;
    invalid_reference[0] = std::numeric_limits<double>::quiet_NaN();
    rb_servo::CommandBuffer invalid_buffer;
    rb_servo::DualArmServoLoop invalid_loop(
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Left,
            actual,
            false,
            rb_servo::BackendErrorKind::ControllerRejected,
            invalid_reference,
            true
        ),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, actual, false),
        cfg,
        &invalid_buffer,
        nullptr
    );
    RB_CHECK(!invalid_loop.start());
    return true;
}

bool testRbpodoControllerSimulationStreamingCartesianGate() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    EnvVarGuard allow_controller_sim("RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION");
    EnvVarGuard allow_controller_sim_cartesian("RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN");
    EnvVarGuard allow_real_cartesian("RB_ALLOW_REAL_CARTESIAN");
    EnvVarGuard pgmode_confirmed("RB_RBPODO_PGMODE_SIMULATION_CONFIRMED");

    allow_real.set("1");
    allow_motion.set("1");
    allow_controller_sim.set("1");
    allow_controller_sim_cartesian.set("1");
    allow_real_cartesian.unset();
    pgmode_confirmed.set("1");

    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    configureCartesianLoopTest(&cfg);
    cfg.cartesian_control.allow_in_controller_simulation = true;
    cfg.cartesian_control.controller_simulation_servo_state_source =
        rb_servo::CartesianControllerSimulationStateSource::Reference;
    cfg.cartesian_control.controller_simulation_divergence_source =
        rb_servo::CartesianControllerSimulationStateSource::Reference;
    cfg.safety.controller_simulation_tracking_error_source =
        rb_servo::ControllerSimulationTrackingErrorSource::Reference;

    rb_servo::ServoSnapshot snapshot;
    bool twist_observed = false;
    RB_CHECK(runLeftTcpTwistStandCase(cfg, &snapshot, &twist_observed, true));
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(twist_observed);
    RB_CHECK(snapshot.left_safety_tracking.tracking_error_source == "reference");
    RB_CHECK(snapshot.left_safety_tracking.tracking_error_source_valid);
    RB_CHECK(!snapshot.left_safety_tracking.controller_simulation_physical_motion_detected);
    RB_CHECK(snapshot.left_cartesian_solve.cartesian_servo_state_source == "reference");
    RB_CHECK(snapshot.left_cartesian_solve.cartesian_divergence_source == "reference");
    RB_CHECK(snapshot.left_cartesian_solve.q_reference_for_servo_valid);

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    RB_CHECK(json.at("left").at("cartesian_available").get<bool>());
    RB_CHECK(json.at("left").at("controller_simulation_cartesian_enabled").get<bool>());
    RB_CHECK(!json.at("left").at("streaming_cartesian_physical_real_enabled").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_gate").at("allow_in_controller_simulation").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_gate").at("controller_simulation_servo_state_source").get<std::string>() ==
             "reference");
    RB_CHECK(json.at("left").at("cartesian_gate").at("controller_simulation_tracking_error_source").get<std::string>() ==
             "reference");
    RB_CHECK(json.at("left").at("tracking_error_source").get<std::string>() == "reference");
    RB_CHECK(json.at("left").at("tracking_error_source_valid").get<bool>());
    RB_CHECK(!json.at("left").at("controller_simulation_physical_motion_detected").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_solve").at("cartesian_servo_state_source").get<std::string>() ==
             "reference");
    RB_CHECK(json.at("left").at("cartesian_solve").at("cartesian_divergence_source").get<std::string>() ==
             "reference");
    RB_CHECK(json.at("left").at("cartesian_solve").at("q_reference_for_servo_valid").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_gate").at("env_RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN").get<bool>());
    RB_CHECK(!json.at("left").at("cartesian_gate").at("physical_motion_expected").get<bool>());

    rb_servo::ServoSnapshot physical_motion_snapshot;
    bool physical_motion_twist_observed = false;
    RB_CHECK(runLeftTcpTwistStandCase(
        cfg,
        &physical_motion_snapshot,
        &physical_motion_twist_observed,
        false,
        true
    ));
    RB_CHECK(physical_motion_snapshot.fault_latched);
    RB_CHECK(physical_motion_snapshot.latched_fault_reason == rb_servo::SafetyVerdict::TrackingError);
    RB_CHECK(physical_motion_snapshot.fault_reason == "controller_simulation_physical_motion_detected");
    RB_CHECK(physical_motion_snapshot.left_safety_tracking.controller_simulation_physical_motion_detected);

    allow_controller_sim_cartesian.unset();
    rb_servo::ServoSnapshot missing_env_snapshot;
    bool missing_env_twist_observed = false;
    RB_CHECK(runLeftTcpTwistStandCase(cfg, &missing_env_snapshot, &missing_env_twist_observed));
    RB_CHECK(missing_env_snapshot.safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable);
    RB_CHECK(missing_env_snapshot.left_cartesian_solve.reason ==
             "cartesian_control_unavailable_controller_sim_env");
    RB_CHECK(!missing_env_twist_observed);
    allow_controller_sim_cartesian.set("1");

    rb_servo::DualArmConfig physical_mode_cfg = cfg;
    physical_mode_cfg.left_robot.operation_mode = "real";
    physical_mode_cfg.right_robot.operation_mode = "real";
    rb_servo::ServoSnapshot physical_mode_snapshot;
    bool physical_mode_twist_observed = false;
    RB_CHECK(runLeftTcpTwistStandCase(physical_mode_cfg, &physical_mode_snapshot, &physical_mode_twist_observed));
    RB_CHECK(physical_mode_snapshot.safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable);
    RB_CHECK(physical_mode_snapshot.left_cartesian_solve.reason ==
             "cartesian_control_unavailable_operation_mode");
    RB_CHECK(!physical_mode_twist_observed);

    rb_servo::DualArmConfig rbscript_cfg = cfg;
    rbscript_cfg.left_robot.backend_type = rb_servo::BackendType::RbscriptTcp;
    rbscript_cfg.right_robot.backend_type = rb_servo::BackendType::RbscriptTcp;
    rb_servo::ServoSnapshot rbscript_snapshot;
    bool rbscript_twist_observed = false;
    RB_CHECK(runLeftTcpTwistStandCase(rbscript_cfg, &rbscript_snapshot, &rbscript_twist_observed));
    RB_CHECK(rbscript_snapshot.safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable);
    RB_CHECK(rbscript_snapshot.left_cartesian_solve.reason ==
             "cartesian_control_unavailable_backend");
    RB_CHECK(!rbscript_twist_observed);

    rb_servo::DualArmConfig config_closed_cfg = cfg;
    config_closed_cfg.cartesian_control.allow_in_controller_simulation = false;
    rb_servo::ServoSnapshot config_closed_snapshot;
    bool config_closed_twist_observed = false;
    RB_CHECK(runLeftTcpTwistStandCase(config_closed_cfg, &config_closed_snapshot, &config_closed_twist_observed));
    RB_CHECK(config_closed_snapshot.safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable);
    RB_CHECK(config_closed_snapshot.left_cartesian_solve.reason ==
             "cartesian_control_unavailable_controller_sim_config");
    RB_CHECK(!config_closed_twist_observed);

    rb_servo::DualArmConfig physical_real_cfg = cfg;
    physical_real_cfg.left_robot.operation_mode = "real";
    physical_real_cfg.right_robot.operation_mode = "real";
    physical_real_cfg.cartesian_control.allow_in_real = true;
    allow_real_cartesian.set("1");
    rb_servo::ServoSnapshot physical_real_snapshot;
    bool physical_real_twist_observed = false;
    RB_CHECK(runLeftTcpTwistStandCase(physical_real_cfg, &physical_real_snapshot, &physical_real_twist_observed));
    RB_CHECK(physical_real_snapshot.safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable);
    RB_CHECK(physical_real_snapshot.left_cartesian_solve.reason ==
             "cartesian_control_unavailable_physical_real_blocked");
    RB_CHECK(!physical_real_twist_observed);

    return true;
}

bool testTcpCircleMoveSimulationOnlyAndConfigGated() {
    rb_servo::CommandBuffer disabled_buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.right_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.left_mount.arm_id = rb_servo::ArmId::Left;
    cfg.right_mount.arm_id = rb_servo::ArmId::Right;
    cfg.kinematics.enable = true;
    cfg.kinematics.ik.enable = true;
    cfg.kinematics.publish_tcp = true;
    cfg.cartesian_control.enable_benchmark_primitives = false;
    cfg.cartesian_control.circle_move.min_period_sec = 0.1;
    cfg.cartesian_control.max_twist_linear_m_s = 10.0;
    cfg.cartesian_control.max_twist_angular_rad_s = 10.0;
    const rb_servo::JointArray initial = joints(0.0);

    auto disabled_kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop disabled_loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &disabled_buffer,
        nullptr,
        disabled_kinematics
    );
    RB_CHECK(disabled_loop.start());
    disabled_buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand circle = command(rb_servo::ControlMode::Hold);
    circle.left.mode = rb_servo::ControlMode::TcpCircleMove;
    circle.left.has_tcp_circle_move = true;
    circle.left.tcp_circle_move.plane = rb_servo::TcpCirclePlane::XY;
    circle.left.tcp_circle_move.diameter_m = 0.10;
    circle.left.tcp_circle_move.period_sec = 0.50;
    circle.left.tcp_circle_move.repeat = 1;
    circle.right.mode = rb_servo::ControlMode::Hold;
    disabled_buffer.setCommand(circle);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = disabled_loop.latestSnapshot();
        return snapshot.command.left.mode == rb_servo::ControlMode::TcpCircleMove &&
               snapshot.safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable &&
               snapshot.left_cartesian_solve.reason == "tcp_circle_move_benchmark_primitives_disabled";
    }));
    RB_CHECK(!disabled_kinematics->lastLeftTwist().has_value());
    disabled_loop.stop();

    rb_servo::CommandBuffer enabled_buffer;
    rb_servo::DualArmConfig enabled_cfg = cfg;
    enabled_cfg.cartesian_control.enable_benchmark_primitives = true;
    auto enabled_kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop enabled_loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        enabled_cfg,
        &enabled_buffer,
        nullptr,
        enabled_kinematics
    );
    RB_CHECK(enabled_loop.start());
    enabled_buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    enabled_buffer.setCommand(circle);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = enabled_loop.latestSnapshot();
        return snapshot.command.left.mode == rb_servo::ControlMode::TcpCircleMove &&
               snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok &&
               snapshot.left_cartesian_solve.status == "ok" &&
               snapshot.left_cartesian_solve.circle_active &&
               enabled_kinematics->lastLeftTwist().has_value();
    }));
    const rb_servo::ServoSnapshot enabled_snapshot = enabled_loop.latestSnapshot();
    RB_CHECK(std::abs(enabled_snapshot.left_cartesian_solve.circle_radius_m - 0.05) < 1e-9);
    enabled_loop.stop();

    rb_servo::CommandBuffer real_buffer;
    rb_servo::DualArmConfig real_cfg = enabled_cfg;
    real_cfg.left_robot.run_mode = rb_servo::RunMode::Real;
    real_cfg.right_robot.run_mode = rb_servo::RunMode::Real;
    auto real_kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop real_loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        real_cfg,
        &real_buffer,
        nullptr,
        real_kinematics
    );
    RB_CHECK(real_loop.start());
    real_buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    real_buffer.setCommand(circle);
    sleepTicks();
    const rb_servo::ServoSnapshot real_snapshot = real_loop.latestSnapshot();
    RB_CHECK(real_snapshot.safety_verdict == rb_servo::SafetyVerdict::InvalidCommand ||
             real_snapshot.safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable);
    RB_CHECK(!real_kinematics->lastLeftTwist().has_value());
    real_loop.stop();
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
    stand_delta.seq = 2;
    stand_delta.host_time_ns = rb_servo::nowSteadyNs();
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
    local_delta.seq = 3;
    local_delta.host_time_ns = rb_servo::nowSteadyNs();
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

bool testCartesianDeltaCommandsAreOneShotPerSeq() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.right_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.left_mount.arm_id = rb_servo::ArmId::Left;
    cfg.right_mount.arm_id = rb_servo::ArmId::Right;
    cfg.kinematics.enable = true;
    cfg.kinematics.ik.enable = true;
    cfg.kinematics.publish_tcp = true;
    rb_servo::JointArray initial = joints(0.0);
    initial[5] = 10.0;
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    kinematics->setOrientationFromJoint(true);
    kinematics->setOrientationSolveBiasRad(0.001);
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

    rb_servo::DualArmCommand local_delta = command(rb_servo::ControlMode::TcpDeltaLocal);
    local_delta.seq = 10;
    local_delta.host_time_ns = rb_servo::nowSteadyNs();
    local_delta.left.has_tcp_delta_local = true;
    local_delta.right.has_tcp_delta_local = true;
    local_delta.left.tcp_delta_local = {0.0, 0.0, 0.01, 0.0, 0.0, 0.0};
    local_delta.right.tcp_delta_local = {0.0, 0.0, 0.02, 0.0, 0.0, 0.0};
    buffer.setCommand(local_delta);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        return previous.left_q_target_deg[2] >= 1.0 - kEpsilon &&
               previous.right_q_target_deg[2] >= 2.0 - kEpsilon;
    }));
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    rb_servo::ServoTarget after_same_seq = loop.previousSentTarget();
    RB_CHECK(std::abs(after_same_seq.left_q_target_deg[2] - 1.0) < 1e-6);
    RB_CHECK(std::abs(after_same_seq.right_q_target_deg[2] - 2.0) < 1e-6);
    RB_CHECK(std::abs(after_same_seq.left_q_target_deg[5] - 10.1) < 1e-6);
    RB_CHECK(std::abs(after_same_seq.right_q_target_deg[5] - 10.1) < 1e-6);

    rb_servo::ServoSnapshot same_seq_snapshot = loop.latestSnapshot();
    RB_CHECK(same_seq_snapshot.command.seq == 10);
    RB_CHECK(same_seq_snapshot.command.left.mode == rb_servo::ControlMode::TcpPoseTarget);
    RB_CHECK(same_seq_snapshot.command.left.has_tcp_target);
    RB_CHECK(std::abs(same_seq_snapshot.command.left.tcp_target_stand.z - 0.01) < 1e-6);
    RB_CHECK(std::abs(same_seq_snapshot.command.left.tcp_target_stand.rz - 0.1) < 1e-6);

    rb_servo::DualArmCommand next_delta = local_delta;
    next_delta.seq = 11;
    next_delta.host_time_ns = rb_servo::nowSteadyNs();
    buffer.setCommand(next_delta);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        return previous.left_q_target_deg[2] >= 2.0 - kEpsilon &&
               previous.right_q_target_deg[2] >= 4.0 - kEpsilon;
    }));
    const rb_servo::ServoTarget after_new_seq = loop.previousSentTarget();
    loop.stop();
    RB_CHECK(std::abs(after_new_seq.left_q_target_deg[2] - 2.0) < 1e-6);
    RB_CHECK(std::abs(after_new_seq.right_q_target_deg[2] - 4.0) < 1e-6);
    RB_CHECK(std::abs(after_new_seq.left_q_target_deg[5] - 10.2) < 1e-6);
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
    RB_CHECK(snapshot.left_cartesian_solve.attempted);
    RB_CHECK(snapshot.left_cartesian_solve.status == "failed");
    RB_CHECK(snapshot.left_cartesian_solve.reason == "injected_failure");
    RB_CHECK(snapshot.left_cartesian_solve.ik_duration_us == 125.0);
    RB_CHECK(snapshot.left_cartesian_solve.ik_iterations == 7);
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::ArmedHold);
    return true;
}

bool testCartesianIkDurationBudgetFailureIsSimulatorOnly() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.right_robot.run_mode = rb_servo::RunMode::Simulation;
    cfg.kinematics.enable = true;
    cfg.kinematics.ik.enable = true;
    cfg.kinematics.publish_tcp = true;
    cfg.cartesian_control.fail_ik_duration_us = 1.0;
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
               snapshot.safety_verdict == rb_servo::SafetyVerdict::IkFailed &&
               snapshot.left_cartesian_solve.ik_fail_duration_exceeded &&
               snapshot.left_cartesian_solve.reason == "ik_duration_budget_exceeded";
    }));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();

    RB_CHECK(snapshot.left_cartesian_solve.ik_duration_us == 125.0);
    RB_CHECK(snapshot.left_cartesian_solve.fail_ik_duration_us == 1.0);
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
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.latched_fault_context.has_value() &&
               snapshot.latched_fault_context->verdict == "SendFailure";
    }));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.latched_fault_context.has_value());
    RB_CHECK(snapshot.latched_fault_context->domain == "Backend");
    RB_CHECK(
        snapshot.latched_fault_context->backend_error_kind == "ControllerRejected" ||
        snapshot.latched_fault_context->backend_error_kind == "TransportWriteFailed" ||
        snapshot.latched_fault_context->backend_error_kind == "TransportTimeout"
    );
    loop.stop();
    return true;
}

bool testRobotFaultSendClassifiesAsRobotStateFault() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.stop_both_arms_on_single_arm_error = true;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left,
        initial,
        true,
        rb_servo::BackendErrorKind::RobotFault
    );
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
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(7.0);
    target.right.q_target_deg = joints(7.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));

    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot current = loop.latestSnapshot();
        return current.latched_fault_context.has_value() &&
               current.latched_fault_context->backend_error_kind == "RobotFault";
    }));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    const int left_send_count_at_latch = left_backend->sendCount();
    const int right_send_count_at_latch = right_backend->sendCount();
    sleepTicks();
    const rb_servo::ServoSnapshot suppressed = loop.latestSnapshot();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(snapshot.latched_fault_context.has_value());
    RB_CHECK(snapshot.latched_fault_context->verdict == "RobotStateError");
    RB_CHECK(snapshot.latched_fault_context->domain == "RobotState");
    RB_CHECK(snapshot.latched_fault_context->arm == "left");
    RB_CHECK(snapshot.latched_fault_context->backend_op == "SendServoJ");
    RB_CHECK(snapshot.latched_fault_context->backend_error_kind == "RobotFault");
    RB_CHECK(snapshot.latched_fault_context->backend_error_code == "2222");
    RB_CHECK(snapshot.latched_fault_context->recoverable);
    RB_CHECK(!snapshot.latched_fault_context->retryable);
    RB_CHECK(snapshot.latched_fault_context->robot_fault);
    RB_CHECK(!snapshot.latched_fault_context->transport_fault);
    RB_CHECK(snapshot.latched_fault_context->state_after_source == "cache");
    RB_CHECK(snapshot.left_state.has_error);
    RB_CHECK(snapshot.left_state.error_code == 2222);
    RB_CHECK(!snapshot.left_send_ok);
    RB_CHECK(
        snapshot.left_send_error_kind == "RobotFault" ||
        snapshot.left_last_send.backend_error_kind == "SuppressedByPolicy"
    );
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::FaultLatched);
    RB_CHECK(left_backend->sendCount() == left_send_count_at_latch);
    RB_CHECK(right_backend->sendCount() == right_send_count_at_latch);
    RB_CHECK(suppressed.send_policy == "fault_latched");
    RB_CHECK(suppressed.send_suppressed);
    RB_CHECK(!suppressed.left_last_send.accepted);
    RB_CHECK(suppressed.left_last_send.backend_error_kind == "SuppressedByPolicy");
    RB_CHECK(suppressed.latched_fault_context.has_value());
    RB_CHECK(suppressed.latched_fault_context->backend_error_kind == "RobotFault");
    RB_CHECK(suppressed.latched_fault_context->backend_error_code == "2222");
    RB_CHECK(suppressed.left_last_read.backend_error_kind == "RobotFault");
    loop.stop();
    return true;
}

bool testDualArmSendFaultLatchPreservesPerArmContexts() {
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
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Right,
            initial,
            true,
            rb_servo::BackendErrorKind::TransportTimeout
        ),
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

    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot current = loop.latestSnapshot();
        return current.fault_latched &&
               current.latched_fault_context.has_value() &&
               current.left_latched_fault_context.has_value() &&
               current.right_latched_fault_context.has_value();
    }));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();

    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(snapshot.latched_fault_context->arm == "left");
    RB_CHECK(snapshot.latched_fault_context->backend_error_kind == "RobotFault");
    RB_CHECK(snapshot.left_latched_fault_context->arm == "left");
    RB_CHECK(snapshot.left_latched_fault_context->verdict == "RobotStateError");
    RB_CHECK(snapshot.left_latched_fault_context->backend_error_kind == "RobotFault");
    RB_CHECK(snapshot.left_latched_fault_context->backend_error_code == "2222");
    RB_CHECK(snapshot.right_latched_fault_context->arm == "right");
    RB_CHECK(snapshot.right_latched_fault_context->verdict == "SendFailure");
    RB_CHECK(snapshot.right_latched_fault_context->domain == "Backend");
    RB_CHECK(snapshot.right_latched_fault_context->backend_error_kind == "TransportTimeout");
    RB_CHECK(snapshot.right_latched_fault_context->transport_fault);
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
    if (!testRbsimPersistentTransportReconnectsAfterSocketDrop()) return 1;
    if (!testRbsimReconnectBackoffSuppressesStormAndResetsAfterSuccess()) return 1;
    if (!testRbsimInvalidJointStateLatchesAndHoldsPreviousTarget()) return 1;
    if (!testRbsimPerArmDisconnectLatchesAndPublishesTruthfulSnapshot()) return 1;
    if (!testRbsimReadRobotFaultUsesFaultClassifierReason()) return 1;
    if (!testRbsimReadFailureLatchesFaultWithoutAdvancingHold()) return 1;
    if (!testRbsimSendFailureLatchesAndSnapshotsSendResult()) return 1;
    if (!testRbsimResetFailureKeepsEmergencyLatch()) return 1;
    if (!testRbsimStopFailureDoesNotReportStoppedState()) return 1;
    if (!testRbsimTrackingErrorFaultLatchHoldsPreviousTarget()) return 1;
    if (!testCommandSequenceRequiredAndMonotonic()) return 1;
    if (!testCommandSourceMetadataAndLeaseEnforcement()) return 1;
    if (!testCartesianCommandParser()) return 1;
    if (!testCartesianControllerUsesQuaternionPoseOrientation()) return 1;
    if (!testCartesianControllerPureTranslationsPreserveQuaternionOrientation()) return 1;
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
    if (!testWorkerIoModeDispatchesThroughArmWorkers()) return 1;
    if (!testWorkerIoModeTimesOutMissingSendResultByDeadline()) return 1;
    if (!testWorkerIoModeReportsMixedTimeoutAndAcceptedArm()) return 1;
    if (!testWorkerIoModeClassifiesStaleStateExplicitly()) return 1;
    if (!testStatePublisherAcceptsDockerServiceHostnameEndpoint()) return 1;
    if (!testStatePublisherSerializesServoSnapshotSchema()) return 1;
    if (!testStatePublisherUsesLatestSnapshotWithoutBackendReadsAndDoesNotStallLoop()) return 1;
    if (!testStatePublisherUsesConfiguredPublishRate()) return 1;
    if (!testLoggerZeroCapacityDropsWithoutBlocking()) return 1;
    if (!testReadOnlyDiagnosticStartupAllowsFaultedStateAndPublishesUnsafeSnapshot()) return 1;
    if (!testMotionStartupRejectsFaultedState()) return 1;
    if (!testRbpodoControllerSimulationMotionRequiresExplicitGate()) return 1;
    if (!testRbpodoControllerSimulationDiagnosticOverrideIsNarrow()) return 1;
    if (!testRbpodoControllerSimulationDiagnosticOverrideRejectsHardFaults()) return 1;
    if (!testReadOnlyDiagnosticStartupAllowsRangeViolationOnlyWhenConfigured()) return 1;
    if (!testMotionStartupRejectsRangeViolation()) return 1;
    if (!testReadOnlyDiagnosticStartupAllowsWrongModeOnlyWhenConfigured()) return 1;
    if (!testMotionStartupRejectsWrongMode()) return 1;
    if (!testReadOnlyDiagnosticStartupRejectsAcquisitionFailure()) return 1;
    if (!testInvalidStartupRobotStateFailsStart()) return 1;
    if (!testEmergencyWinsAndResetDoesNotRun()) return 1;
    if (!testResetFaultFailureKeepsFaultLatched()) return 1;
    if (!testResetFaultRequiresFreshValidState()) return 1;
    if (!testDisarmAndCartesianHoldPreviousTarget()) return 1;
    if (!testCartesianPoseTargetUsesIkInSimulation()) return 1;
    if (!testTcpLinearMoveUsesIkInSimulationOnly()) return 1;
    if (!testStreamingCartesianSimulationStillAvailable()) return 1;
    if (!testRbpodoControllerSimulationStartupReferenceSource()) return 1;
    if (!testRbpodoControllerSimulationStreamingCartesianGate()) return 1;
    if (!testTcpCircleMoveSimulationOnlyAndConfigGated()) return 1;
    if (!testCartesianDeltaStandAndLocalUseIkInSimulation()) return 1;
    if (!testCartesianDeltaCommandsAreOneShotPerSeq()) return 1;
    if (!testCartesianIkFailureHoldsPreviousSafeTarget()) return 1;
    if (!testCartesianIkDurationBudgetFailureIsSimulatorOnly()) return 1;
    if (!testCartesianRealModeBlockedByDefault()) return 1;
    if (!testInvalidMotionCommandDoesNotReportRunning()) return 1;
    if (!testJointLimitClamp()) return 1;
    if (!testSendFailureDoesNotAdvancePreviousTarget()) return 1;
    if (!testStopBothOnSendFailureLatchesFault()) return 1;
    if (!testRobotFaultSendClassifiesAsRobotStateFault()) return 1;
    if (!testDualArmSendFaultLatchPreservesPerArmContexts()) return 1;
    if (!testSuppressedByPolicySendDoesNotLatchFault()) return 1;
    return 0;
}
