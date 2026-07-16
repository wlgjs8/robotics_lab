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

void applyIntentionalNarrowJointRangeForViolationTest(rb_servo::SafetyConfig* safety) {
    safety->q_min_deg = joints(-180.0);
    safety->q_max_deg = joints(180.0);
}

rb_servo::SafetyConfig rbpodoRawControllerSafetyConfigForTests() {
    rb_servo::SafetyConfig cfg;
    cfg.q_min_deg = rb_servo::rbpodoDefaultSafetyJointMinDeg();
    cfg.q_max_deg = rb_servo::rbpodoDefaultSafetyJointMaxDeg();
    return cfg;
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

std::vector<std::string> splitCommaSeparated(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    if (!line.empty() && line.back() == ',') {
        fields.push_back("");
    }
    return fields;
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
        advanceRobotTimeOnRead();
        if (!read_ok_) {
            return result(
                rb_servo::BackendOp::ReadState,
                currentState(),
                false,
                rb_servo::backendError(rb_servo::BackendErrorKind::TransportReadFailed, "test read failed")
            );
        }
        acquisition_sequence_.fetch_add(1);
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
        if (!freeze_reference_on_send_) {
            q_target_ = request.q_target_deg;
        }
        if (!accept_send_without_state_update_) {
            q_actual_ = request.q_target_deg;
        }
        rb_servo::SendServoJResult result =
            rb_servo::acceptedSend(request, {}, currentState(), "cache");
        result.ack_policy = rb_servo::BackendAckPolicy::Wait;
        result.ack_observed = true;
        result.controller_acceptance_observed = true;
        result.rbpodo_waiting_ack = true;
        result.acceptance_semantics = "controller_ack_observed";
        return result;
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
    rb_servo::BackendResult<rb_servo::RobotState> setFreedrive(bool on) override {
        if (on) {
            ++freedrive_on_count_;
            if (freedrive_on_fail_.load()) {
                return result(
                    rb_servo::BackendOp::SetFreedrive,
                    currentState(),
                    false,
                    rb_servo::backendError(
                        rb_servo::BackendErrorKind::ControllerRejected,
                        "test teach_on rejected (M151)", "151", ""
                    )
                );
            }
        } else {
            ++freedrive_off_count_;
        }
        // Simulate the controller engaging/releasing gravity-compensation.
        controller_freedrive_on_.store(on);
        return result(rb_servo::BackendOp::SetFreedrive, currentState(), true);
    }
    bool isConnected() const override { return connected_; }
    rb_servo::ArmId armId() const override { return arm_id_; }
    std::string name() const override { return "test"; }

    void setControllerMotionState(int s) { controller_motion_state_.store(s); }
    void setControllerFreedriveOn(bool on) { controller_freedrive_on_.store(on); }
    void setFreedriveOnFail(bool fail) { freedrive_on_fail_.store(fail); }
    int freedriveOnCount() const { return freedrive_on_count_.load(); }
    int freedriveOffCount() const { return freedrive_off_count_.load(); }
    bool controllerFreedriveOn() const { return controller_freedrive_on_.load(); }

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
    void setServoEnabled(bool enabled) { servo_enabled_override_ = enabled; }
    void setResetOk(bool ok) { reset_ok_ = ok; }
    void setInvalidateJointStateOnReset(bool invalidate) { invalidate_joint_state_on_reset_ = invalidate; }
    void setReadSleepMs(int sleep_ms) { read_sleep_ms_.store(sleep_ms); }
    void setSendSleepMs(int sleep_ms) { send_sleep_ms_.store(sleep_ms); }
    void setQRefValid(bool valid) { q_ref_valid_ = valid; }
    void setFreezeReferenceOnSend(bool freeze) { freeze_reference_on_send_ = freeze; }
    void setAcceptSendWithoutStateUpdate(bool accept) { accept_send_without_state_update_ = accept; }
    void setActualJoints(const rb_servo::JointArray& q_actual) { q_actual_ = q_actual; }
    void setRobotTimeNs(uint64_t robot_time_ns) { robot_time_ns_.store(robot_time_ns); }
    void setAdvanceRobotTimeOnRead(bool advance) { advance_robot_time_on_read_ = advance; }
    void setEftWrench(const rb_servo::Wrench6D& wrench) {
        eft_wrench_ = wrench;
        eft_valid_ = true;
    }
    int readCount() const { return read_count_; }
    int resetCount() const { return reset_count_; }
    int sendCount() const { return send_count_.load(); }

private:
    void advanceRobotTimeOnRead() {
        if (advance_robot_time_on_read_) {
            robot_time_ns_.fetch_add(5'000'000);
        }
    }

    rb_servo::RobotState currentState() const {
        rb_servo::RobotState state;
        state.arm_id = arm_id_;
        state.host_time_ns = rb_servo::nowSteadyNs();
        state.robot_time_ns = robot_time_ns_.load();
        state.acquisition_sequence = acquisition_sequence_.load();
        state.q_actual_deg = q_actual_;
        state.q_target_deg = q_target_;
        state.q_actual_valid = valid_joint_state_;
        state.q_ref_valid = q_ref_valid_ && valid_joint_state_;
        state.q_ref_source = "test.q_ref";
        state.eft_wrench = eft_wrench_;
        state.eft_valid = eft_valid_;
        state.has_valid_joint_state = valid_joint_state_;
        state.connection_state = connected_
            ? rb_servo::RobotConnectionState::Connected
            : rb_servo::RobotConnectionState::Disconnected;
        state.servo_enabled = servo_enabled_override_.value_or(initialized_);
        state.controller_motion_state = controller_motion_state_.load();
        state.controller_freedrive_on = controller_freedrive_on_.load();
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
    bool q_ref_valid_ = true;
    bool freeze_reference_on_send_ = false;
    bool advance_robot_time_on_read_ = true;
    bool read_ok_ = true;
    bool reset_ok_ = true;
    bool invalidate_joint_state_on_reset_ = false;
    bool has_error_ = false;
    int error_code_ = 0;
    std::string motion_readiness_error_kind_;
    std::string motion_readiness_error_name_;
    std::string diagnostic_error_source_;
    std::optional<bool> servo_enabled_override_;
    bool connected_ = false;
    bool initialized_ = false;
    std::atomic<int> read_sleep_ms_{0};
    std::atomic<int> send_sleep_ms_{0};
    // Free-drive controller signal simulation. controller_motion_state_: 1=Idle,
    // 3=Moving, 0=unknown. controller_freedrive_on_ mirrors is_freedrive_mode.
    std::atomic<int> controller_motion_state_{0};
    std::atomic<bool> controller_freedrive_on_{false};
    std::atomic<bool> freedrive_on_fail_{false};
    std::atomic<int> freedrive_on_count_{0};
    std::atomic<int> freedrive_off_count_{0};
    mutable std::atomic<uint64_t> robot_time_ns_{0};
    mutable std::atomic<uint64_t> acquisition_sequence_{0};
    rb_servo::Wrench6D eft_wrench_{};
    bool eft_valid_ = false;
    int read_count_ = 0;
    int reset_count_ = 0;
    std::atomic<int> send_count_{0};
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
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            if (arm == rb_servo::ArmId::Left) {
                last_left_target_ = target_tcp_stand;
            } else {
                last_right_target_ = target_tcp_stand;
            }
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

    void setFail(bool fail) { fail_ = fail; }
    void setOrientationFromJoint(bool enabled) { orientation_from_joint_ = enabled; }
    void setOrientationSolveBiasRad(double bias_rad) { orientation_solve_bias_rad_ = bias_rad; }
    void setSolveError(double position_error_m, double orientation_error_rad) {
        position_error_m_ = position_error_m;
        orientation_error_rad_ = orientation_error_rad;
    }
    std::optional<rb_servo::Pose6D> lastLeftTarget() const {
        std::lock_guard<std::mutex> lock(target_mutex_);
        return last_left_target_;
    }
    std::optional<rb_servo::Pose6D> lastRightTarget() const {
        std::lock_guard<std::mutex> lock(target_mutex_);
        return last_right_target_;
    }

private:
    bool fail_ = false;
    bool orientation_from_joint_ = false;
    double orientation_solve_bias_rad_ = 0.0;
    double position_error_m_ = 0.0;
    double orientation_error_rad_ = 0.0;
    mutable std::mutex target_mutex_;
    mutable std::optional<rb_servo::Pose6D> last_left_target_;
    mutable std::optional<rb_servo::Pose6D> last_right_target_;
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
    cfg.safety.q_min_deg = rb_servo::rbpodoDefaultSafetyJointMinDeg();
    cfg.safety.q_max_deg = rb_servo::rbpodoDefaultSafetyJointMaxDeg();
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

rb_servo::DualArmConfig rbpodoPhysicalRealConfig() {
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.left_robot.operation_mode = "real";
    cfg.right_robot.operation_mode = "real";
    cfg.servo.allow_controller_simulation_motion = false;
    return cfg;
}

void enableRbpodoAsyncStreaming(
    rb_servo::DualArmConfig* cfg,
    rb_servo::RbpodoAsyncStreamingMode mode =
        rb_servo::RbpodoAsyncStreamingMode::SdkAckWorker
) {
    cfg->servo.rbpodo_async_streaming.enable = true;
    cfg->servo.rbpodo_async_streaming.mode = mode;
    cfg->servo.rbpodo_async_streaming.rate_hz = cfg->servo.rate_hz;
    cfg->servo.rbpodo_async_streaming.ack_supervision.max_consecutive_missing_ack = 2;
    cfg->servo.rbpodo_async_streaming.ack_supervision.missing_ack_fault_after_ms = 20.0;
    cfg->servo.rbpodo_async_streaming.reference_supervision.q_ref_update_timeout_ms = 20.0;
    cfg->servo.rbpodo_async_streaming.reference_supervision.q_ref_target_tolerance_deg = 0.5;
    cfg->servo.rbpodo_async_streaming.reference_supervision.q_ref_target_fault_after_ms = 20.0;
    cfg->servo.rbpodo_async_streaming.reference_supervision.tcp_ref_update_timeout_ms = 20.0;
    cfg->servo.rbpodo_async_streaming.reference_supervision.tcp_ref_target_tolerance_m = 0.02;
    cfg->servo.rbpodo_async_streaming.reference_supervision.tcp_ref_target_fault_after_ms = 20.0;
    cfg->servo.rbpodo_async_streaming.reference_supervision.policy =
        rb_servo::RbpodoAsyncReferenceSupervisionPolicy::FaultLatch;
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

rb_servo::DualArmCommand freedriveCommand(
    uint64_t seq,
    std::optional<bool> left_on,
    std::optional<bool> right_on
) {
    rb_servo::DualArmCommand cmd;
    cmd.seq = seq;
    cmd.host_time_ns = rb_servo::nowSteadyNs();
    cmd.left.arm_id = rb_servo::ArmId::Left;
    cmd.right.arm_id = rb_servo::ArmId::Right;
    cmd.left.timeout_sec = 0.2;
    cmd.right.timeout_sec = 0.2;
    if (left_on.has_value()) {
        cmd.left.mode = rb_servo::ControlMode::Freedrive;
        cmd.left.has_freedrive = true;
        cmd.left.freedrive_on = *left_on;
    } else {
        cmd.left.mode = rb_servo::ControlMode::Hold;
    }
    if (right_on.has_value()) {
        cmd.right.mode = rb_servo::ControlMode::Freedrive;
        cmd.right.has_freedrive = true;
        cmd.right.freedrive_on = *right_on;
    } else {
        cmd.right.mode = rb_servo::ControlMode::Hold;
    }
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
}

std::vector<rb_servo::ControlMode> nonStreamingCartesianModes() {
    return {
        rb_servo::ControlMode::TcpPoseTarget,
    };
}

rb_servo::DualArmCommand leftNonStreamingCartesianCommand(rb_servo::ControlMode mode) {
    rb_servo::DualArmCommand cartesian = command(rb_servo::ControlMode::Hold);
    cartesian.seq = 20 + static_cast<uint64_t>(mode);
    cartesian.host_time_ns = rb_servo::nowSteadyNs();
    cartesian.left.mode = mode;
    cartesian.right.mode = rb_servo::ControlMode::Hold;
    switch (mode) {
        case rb_servo::ControlMode::TcpPoseTarget:
            cartesian.left.has_tcp_target = true;
            cartesian.left.tcp_target_stand = {0.04, 0.02, 0.01, 0.0, 0.0, 0.0};
            break;
        default:
            break;
    }
    return cartesian;
}

bool runLeftNonStreamingCartesianCase(
    rb_servo::DualArmConfig cfg,
    rb_servo::ControlMode mode,
    rb_servo::ServoSnapshot* snapshot,
    bool* left_ik_observed,
    bool accept_send_without_state_update = false
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
    const rb_servo::DualArmCommand cartesian = leftNonStreamingCartesianCommand(mode);
    buffer.setCommand(cartesian);
    RB_CHECK(waitUntil([&] {
        *snapshot = loop.latestSnapshot();
        return snapshot->command.seq == cartesian.seq &&
               snapshot->left_cartesian_solve.attempted &&
               (snapshot->safety_verdict == rb_servo::SafetyVerdict::Ok ||
                snapshot->safety_verdict == rb_servo::SafetyVerdict::CartesianUnavailable ||
                snapshot->safety_verdict == rb_servo::SafetyVerdict::IkFailed);
    }, std::chrono::milliseconds(1000)));
    *left_ik_observed = kinematics->lastLeftTarget().has_value();
    loop.stop();
    return true;
}

bool checkPublishedLeftCartesianGate(
    const rb_servo::DualArmConfig& cfg,
    const rb_servo::ServoSnapshot& snapshot,
    bool expected_available,
    bool expected_controller_sim_enabled,
    const std::string& expected_reason
) {
    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    RB_CHECK(json.at("left").at("cartesian_available").get<bool>() == expected_available);
    RB_CHECK(json.at("left").at("controller_simulation_cartesian_enabled").get<bool>() ==
             expected_controller_sim_enabled);
    RB_CHECK(json.at("left").at("controller_simulation_cartesian_enabled_for_current_command").get<bool>() ==
             expected_controller_sim_enabled);
    RB_CHECK(json.at("left").at("cartesian_gate").at("cartesian_available").get<bool>() ==
             expected_available);
    RB_CHECK(json.at("left").at("cartesian_gate")
                 .at("controller_simulation_cartesian_enabled_for_current_command")
                 .get<bool>() == expected_controller_sim_enabled);
    if (expected_controller_sim_enabled) {
        RB_CHECK(!json.at("left").at("physical_motion_expected").get<bool>());
        RB_CHECK(!json.at("left").at("cartesian_gate").at("physical_motion_expected").get<bool>());
    }
    if (expected_reason.empty()) {
        RB_CHECK(json.at("left").at("cartesian_unavailable_reason").is_null());
        RB_CHECK(json.at("left").at("cartesian_gate").at("cartesian_unavailable_reason").is_null());
    } else {
        RB_CHECK(json.at("left").at("cartesian_unavailable_reason").get<std::string>() ==
                 expected_reason);
        RB_CHECK(json.at("left").at("cartesian_gate")
                     .at("cartesian_unavailable_reason")
                     .get<std::string>() == expected_reason);
    }
    return true;
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

std::string makeDualArmChunkFramePacket(uint64_t seq, int horizon) {
    nlohmann::json left = nlohmann::json::array();
    for (int i = 0; i < horizon; ++i) {
        left.push_back({
            0.0005 * static_cast<double>(i),
            0.0002 * static_cast<double>(i),
            0.0001 * static_cast<double>(i),
            0.0,
            0.0,
            0.0,
            1.0,
            20.0,
        });
    }
    nlohmann::json packet = {
        {"schema", "robotics_lab.chunk_overlay.v2"},
        {"schema_version", "robotics_lab.chunk_overlay.v2"},
        {"host_time_ns", 123456789},
        {"seq", seq},
        {"policy_dt_sec", 0.0334},
        {"horizon", horizon},
        {"left", left},
        {"right", left},
    };
    return packet.dump();
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

std::string writeRbpodoAsyncConfig(
    const std::string& suffix,
    const std::string& left_operation_mode = "simulation",
    const std::string& right_operation_mode = "simulation",
    const std::string& async_mode = "sdk_ack_worker",
    bool disable_waiting_ack = false
) {
    const std::string path =
        "/tmp/rb-servo-rbpodo-async-" + suffix + "-" + std::to_string(getpid()) + ".yaml";
    std::ofstream file(path);
    file << "left_robot:\n"
         << "  backend_type: rbpodo\n"
         << "  run_mode: real\n"
         << "  operation_mode: " << left_operation_mode << "\n"
         << "  ip: 127.0.0.1\n"
         << "  servo_t1_sec: 0.002\n"
         << "  servo_t2_sec: 0.05\n"
         << "  servo_gain: 1.0\n"
         << "  servo_alpha: 0.5\n"
         << "  disable_waiting_ack: " << (disable_waiting_ack ? "true" : "false") << "\n"
         << "right_robot:\n"
         << "  backend_type: rbpodo\n"
         << "  run_mode: real\n"
         << "  operation_mode: " << right_operation_mode << "\n"
         << "  ip: 127.0.0.2\n"
         << "  servo_t1_sec: 0.002\n"
         << "  servo_t2_sec: 0.05\n"
         << "  servo_gain: 1.0\n"
         << "  servo_alpha: 0.5\n"
         << "  disable_waiting_ack: " << (disable_waiting_ack ? "true" : "false") << "\n"
         << "servo:\n"
         << "  rate_hz: 500\n"
         << "  send_servo_commands: true\n"
         << "  allow_controller_simulation_motion: true\n"
         << "  enable_realtime_priority: true\n"
         << "  rbpodo_async_streaming:\n"
         << "    enable: true\n"
         << "    mode: " << async_mode << "\n"
         << "    rate_hz: 500\n"
         << "safety:\n"
         << "  tracking_error_policy: fault_latch\n"
         << "  stop_both_arms_on_single_arm_error: true\n"
         << "  latch_fault_on_robot_state_error: true\n";
    return path;
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
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpPoseTarget"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"TcpLinearMove"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"seq":1,"mode":"InitMotion","q_target_deg":[0,0,0,0,0,0]})", now, &out));

    RB_CHECK(server.parseMessage(R"({"seq":1,"mode":"EmergencyStop"})", now, &out));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::EmergencyStop);
    RB_CHECK(server.parseMessage(R"({"seq":2,"mode":"ArmMotion"})", now, &out));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::ArmMotion);

    RB_CHECK(server.parseMessage(R"({"seq":3,"mode":"JointTarget","q_target_deg":[1,2,3,4,5,6]})", now, &out));
    RB_CHECK(out.left.has_joint_target);
    RB_CHECK(out.right.has_joint_target);
    RB_CHECK(out.left.joint_target_profile == rb_servo::JointTargetProfile::Direct);
    RB_CHECK(std::abs(out.left.timeout_sec - 0.35) < kEpsilon);
    return true;
}

bool testSetExternalBoxesCommandParser() {
    rb_servo::NetworkConfig network;
    network.command_source_enforce_lease = true;
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    rb_servo::DualArmCommand out;
    const uint64_t now = rb_servo::nowSteadyNs();

    RB_CHECK(server.parseMessage(
        R"({"seq":1,"mode":"AcquireLease","source_id":"policy_runner","session_id":"policy-session","lease_token":"policy-token"})",
        now,
        &out
    ));
    RB_CHECK(server.parseMessage(
        R"({"seq":1,"mode":"set_external_boxes","source_id":"camera_worker","session_id":"camera-session","boxes":[{"label":"green","T":[1,0,0,0.11,0,1,0,0.22,0,0,1,0.33,0,0,0,1]},{"label":"gray","T":[0,-1,0,0.44,1,0,0,0.55,0,0,1,0.66,0,0,0,1],"enable":false}]})",
        now + 1,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::SetExternalBoxes);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::SetExternalBoxes);
    RB_CHECK(out.has_external_boxes);
    RB_CHECK(out.external_boxes.size() == 2);
    RB_CHECK(out.external_boxes[0].label == "green");
    RB_CHECK(out.external_boxes[0].enable);
    RB_CHECK(std::abs(out.external_boxes[0].T_stand_box[3] - 0.11) < kEpsilon);
    RB_CHECK(out.external_boxes[1].label == "gray");
    RB_CHECK(!out.external_boxes[1].enable);
    RB_CHECK(std::abs(out.external_boxes[1].T_stand_box[7] - 0.55) < kEpsilon);
    RB_CHECK(!out.lease.command_requires_lease);
    return true;
}

// The external-box feed-liveness watchdog measures RECEIVE time on the
// CommandBuffer, not apply time. Encode that property directly: the receive stamp
// is independent from motion commands and is only advanced by noteExternalBoxReceived().
bool testExternalBoxReceiveStampSurvivesMotionFlood() {
    rb_servo::CommandBuffer buffer;
    RB_CHECK(buffer.lastExternalBoxReceiveNs() == 0);  // no feed yet

    const uint64_t t1 = 1'000'000'000ULL;
    buffer.noteExternalBoxReceived(t1);
    RB_CHECK(buffer.lastExternalBoxReceiveNs() == t1);

    // Saturate the latest-command slot the way flow-infer would at command_rate_hz.
    rb_servo::DualArmCommand motion;
    motion.left.timeout_sec = 1.0;
    motion.right.timeout_sec = 1.0;
    for (int i = 0; i < 1000; ++i) {
        motion.host_time_ns = t1 + static_cast<uint64_t>(i);
        buffer.setCommand(motion);
    }
    // The motion flood must NOT touch the external-box receive stamp.
    RB_CHECK(buffer.lastExternalBoxReceiveNs() == t1);

    const uint64_t t2 = t1 + 2'000'000'000ULL;
    buffer.noteExternalBoxReceived(t2);
    RB_CHECK(buffer.lastExternalBoxReceiveNs() == t2);
    return true;
}

rb_servo::DualArmCommand externalBoxesCommand(uint64_t seq, uint64_t host_time_ns, double x_m) {
    rb_servo::DualArmCommand cmd = command(rb_servo::ControlMode::SetExternalBoxes);
    cmd.seq = seq;
    cmd.host_time_ns = host_time_ns;
    cmd.source.source_id = "stereo_worker";
    cmd.source.session_id = "camera-session";
    cmd.has_external_boxes = true;
    rb_servo::ExternalBoxCommand box;
    box.label = "green";
    box.T_stand_box = {
        1.0, 0.0, 0.0, x_m,
        0.0, 1.0, 0.0, 0.22,
        0.0, 0.0, 1.0, 0.33,
        0.0, 0.0, 0.0, 1.0
    };
    box.enable = true;
    cmd.external_boxes.push_back(box);
    return cmd;
}

bool testExternalBoxesDoNotDisplaceMotionLatest() {
    rb_servo::CommandBuffer buffer;
    const uint64_t now = rb_servo::nowSteadyNs();
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.seq = 10;
    target.host_time_ns = now;
    target.left.q_target_deg = joints(3.0);
    target.right.q_target_deg = joints(3.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    rb_servo::DualArmCommand boxes = externalBoxesCommand(11, now + 1, 0.11);

    buffer.setCommand(target);
    buffer.setCommand(boxes);

    rb_servo::CommandBufferReadTelemetry telemetry;
    rb_servo::DualArmCommand latest = buffer.latestOrHold(now + 2, &telemetry);
    RB_CHECK(latest.seq == target.seq);
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(telemetry.external_boxes_pending);
    RB_CHECK(!telemetry.external_boxes_consumed);
    RB_CHECK(telemetry.latest_seq == target.seq);
    RB_CHECK(telemetry.latest_right_mode == rb_servo::ControlMode::JointTarget);

    std::optional<rb_servo::DualArmCommand> consumed =
        buffer.consumeLatestExternalBoxes(now + 3, &telemetry);
    RB_CHECK(consumed.has_value());
    RB_CHECK(consumed->seq == boxes.seq);
    RB_CHECK(consumed->right.mode == rb_servo::ControlMode::SetExternalBoxes);
    RB_CHECK(telemetry.external_boxes_consumed);
    RB_CHECK(telemetry.external_boxes_seq == boxes.seq);

    latest = buffer.latestOrHold(now + 4);
    RB_CHECK(latest.seq == target.seq);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(!buffer.consumeLatestExternalBoxes(now + 5).has_value());
    return true;
}

bool testExternalBoxesLatestOnlySideSlot() {
    rb_servo::CommandBuffer buffer;
    const uint64_t now = rb_servo::nowSteadyNs();
    rb_servo::DualArmCommand first = externalBoxesCommand(20, now, 0.20);
    rb_servo::DualArmCommand second = externalBoxesCommand(21, now + 1, 0.21);

    buffer.setCommand(first);
    buffer.setCommand(second);

    rb_servo::CommandBufferReadTelemetry telemetry;
    rb_servo::DualArmCommand latest = buffer.latestOrHold(now + 2, &telemetry);
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(telemetry.external_boxes_pending);

    std::optional<rb_servo::DualArmCommand> consumed =
        buffer.consumeLatestExternalBoxes(now + 3, &telemetry);
    RB_CHECK(consumed.has_value());
    RB_CHECK(consumed->seq == second.seq);
    RB_CHECK(std::abs(consumed->external_boxes[0].T_stand_box[3] - 0.21) < kEpsilon);
    RB_CHECK(!buffer.consumeLatestExternalBoxes(now + 4).has_value());
    return true;
}

bool testParsedExternalBoxesDoNotDisplaceMotionLatest() {
    rb_servo::NetworkConfig network;
    network.command_source_enforce_lease = false;
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    rb_servo::DualArmCommand parsed_boxes;
    const uint64_t now = rb_servo::nowSteadyNs();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.seq = 30;
    target.host_time_ns = now;
    target.left.q_target_deg = joints(4.0);
    target.right.q_target_deg = joints(4.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);

    RB_CHECK(server.parseMessage(
        R"({"seq":31,"mode":"set_external_boxes","source_id":"stereo_worker","session_id":"camera-session","boxes":[{"label":"green","T":[1,0,0,0.31,0,1,0,0.22,0,0,1,0.33,0,0,0,1]}]})",
        now + 1,
        &parsed_boxes
    ));
    buffer.setCommand(parsed_boxes);

    rb_servo::CommandBufferReadTelemetry telemetry;
    rb_servo::DualArmCommand latest = buffer.latestOrHold(now + 2, &telemetry);
    RB_CHECK(latest.seq == target.seq);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(telemetry.external_boxes_pending);
    std::optional<rb_servo::DualArmCommand> consumed =
        buffer.consumeLatestExternalBoxes(now + 3, &telemetry);
    RB_CHECK(consumed.has_value());
    RB_CHECK(consumed->seq == parsed_boxes.seq);
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
    RB_CHECK(out.tcp_target_profile == "default");

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

    rb_servo::CartesianControlConfig cartesian_profiles;
    cartesian_profiles.tcp_pose_target_profile_default = "umi_large_smooth";
    for (const std::string& name : {"spacemouse_precise", "umi_large_smooth", "flow_infer_smooth"}) {
        rb_servo::TcpPoseTargetProfileConfig profile;
        profile.name = name;
        cartesian_profiles.tcp_pose_target_profiles.push_back(profile);
    }
    rb_servo::CommandBuffer profiled_buffer;
    rb_servo::CommandServer profiled_server(network, &profiled_buffer, cartesian_profiles);
    RB_CHECK(profiled_server.parseMessage(
        R"({"schema_version":1,"seq":1,"mode":"TcpPoseTarget","timeout_sec":0.2,"left":{"tcp_target_stand":[0.3,0.1,0.5,0,0,0]},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(out.tcp_target_profile == "umi_large_smooth");
    RB_CHECK(!out.tcp_target_profile_provided);
    RB_CHECK(profiled_server.parseMessage(
        R"({"schema_version":1,"seq":2,"mode":"TcpPoseTarget","tcp_target_profile":"spacemouse_precise","timeout_sec":0.2,"left":{"tcp_target_stand":[0.3,0.1,0.5,0,0,0]},"right":{"mode":"Hold"},"client_send_monotonic_ns":123,"input_sample_monotonic_ns":100})",
        now,
        &out
    ));
    RB_CHECK(out.tcp_target_profile == "spacemouse_precise");
    RB_CHECK(out.tcp_target_profile_provided);
    RB_CHECK(out.has_client_send_monotonic_ns);
    RB_CHECK(out.client_send_monotonic_ns == 123);
    RB_CHECK(out.has_input_sample_monotonic_ns);
    RB_CHECK(out.input_sample_monotonic_ns == 100);
    RB_CHECK(!profiled_server.parseMessage(
        R"({"schema_version":1,"seq":3,"mode":"TcpPoseTarget","tcp_target_profile":"unknown","timeout_sec":0.2,"left":{"tcp_target_stand":[0.3,0.1,0.5,0,0,0]},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(profiled_server.lastRejectReason() == "unknown_tcp_target_profile");

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
        R"({"schema_version":1,"seq":10,"mode":"JointTarget","timeout_sec":0.2,"left":{"mode":"JointTarget","q_target_deg":[1,2,3,4,5,6],"joint_target_profile":"init_motion"},"right":{"mode":"JointTarget","q_target_deg":[6,5,4,3,2,1],"joint_target_profile":"direct"}})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(out.left.has_joint_target);
    RB_CHECK(out.right.has_joint_target);
    RB_CHECK(out.left.joint_target_profile == rb_servo::JointTargetProfile::InitMotion);
    RB_CHECK(out.right.joint_target_profile == rb_servo::JointTargetProfile::Direct);
    RB_CHECK(std::abs(out.left.q_target_deg[0] - 1.0) < kEpsilon);
    RB_CHECK(std::abs(out.right.q_target_deg[5] - 1.0) < kEpsilon);

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":11,"mode":"JointTarget","timeout_sec":0.2,"left":{"mode":"JointTarget","q_target_deg":[1,2,3,4,5,6],"joint_target_profile":"payload_identification"},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(out.left.joint_target_profile ==
        rb_servo::JointTargetProfile::PayloadIdentification);
    RB_CHECK(rb_servo::toString(out.left.joint_target_profile) ==
        "payload_identification");

    RB_CHECK(!server.parseMessage(
        R"({"schema_version":1,"seq":12,"mode":"JointTarget","timeout_sec":0.2,"left":{"mode":"JointTarget","q_target_deg":[1,2,3,4,5,6],"joint_target_profile":"bad"},"right":{"mode":"Hold"}})",
        now,
        &out
    ));

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":13,"mode":"Hold","timeout_sec":0.2,"left":{"mode":"TcpPoseTarget","tcp_target_stand":[0.2,0.0,0.4,0,0,0]},"right":{"mode":"Hold"}})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::TcpPoseTarget);
    RB_CHECK(out.left.has_tcp_target);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(std::abs(out.left.tcp_target_stand.x - 0.2) < kEpsilon);
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
             << "  servo_t1_sec: 0.002\n"
             << "  servo_t2_sec: 0.05\n"
             << "  servo_alpha: 0.5\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "  ip: 172.28.60.201\n"
             << "  servo_t1_sec: 0.002\n"
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

    // Real/sim env gates retired: real read-only configs load without envs.
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
             << "  servo_t1_sec: 0.002\n"
             << "  servo_t2_sec: 0.05\n"
             << "  servo_alpha: 0.5\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "  ip: 172.28.60.201\n"
             << "  servo_t1_sec: 0.002\n"
             << "  servo_t2_sec: 0.05\n"
             << "  servo_alpha: 0.5\n"
             << "servo:\n"
             << "  enable_realtime_priority: true\n"
             << "  send_servo_commands: true\n"
             << "safety:\n"
             << "  tracking_error_policy: fault_latch\n";
    }

    // Real/sim env gates retired: real motion configs load without envs.
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
             << "  servo_t1_sec: 0.002\n"
             << "  servo_t2_sec: 0.05\n"
             << "  servo_alpha: 0.5\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "  ip: 172.28.60.201\n"
             << "  servo_t1_sec: 0.002\n"
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
    rb_servo::SafetyConfig cfg = rbpodoRawControllerSafetyConfigForTests();
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
    RB_CHECK(result.clamp.present);
    RB_CHECK(!result.clamp.joint_limit_clamped);
    RB_CHECK(result.clamp.velocity_clamped);
    RB_CHECK(!result.clamp.accel_clamped);
    RB_CHECK(result.clamp.velocity_limited_joint >= 0);
    RB_CHECK(std::abs(result.clamp.velocity_clamp_max_delta_deg - 99.9) < kEpsilon);
    for (double q : result.filtered_q_deg) {
        RB_CHECK(q <= 0.1 + kEpsilon);
        RB_CHECK(q >= -kEpsilon);
    }
    for (int i = 0; i < rb_servo::kDof; ++i) {
        RB_CHECK(std::abs(result.clamp.q_before_safety_deg[i] - 100.0) < kEpsilon);
        RB_CHECK(std::abs(result.clamp.q_after_joint_limit_deg[i] - 100.0) < kEpsilon);
        RB_CHECK(std::abs(result.clamp.q_after_velocity_limit_deg[i] - 0.1) < kEpsilon);
        RB_CHECK(std::abs(result.clamp.q_after_accel_limit_deg[i] - result.filtered_q_deg[i]) < kEpsilon);
    }
    return true;
}

bool testSafetyFilterAccelerationClampDoesNotOvershoot() {
    rb_servo::SafetyConfig cfg = rbpodoRawControllerSafetyConfigForTests();
    cfg.dq_max_deg_s = joints(1000.0);
    cfg.ddq_max_deg_s2 = joints(100.0);
    cfg.max_tracking_error_deg = 1000.0;
    rb_servo::SafetyFilter filter(cfg);

    rb_servo::RobotState state;
    state.connection_state = rb_servo::RobotConnectionState::Connected;
    state.has_valid_joint_state = true;
    state.q_actual_deg = joints(0.0);

    const rb_servo::SafetyCheckResult result = filter.filterJointTarget(
        joints(0.02),
        joints(0.0),
        joints(0.0),
        state,
        0.01
    );

    RB_CHECK(result.ok);
    RB_CHECK(result.clamp.present);
    RB_CHECK(!result.clamp.joint_limit_clamped);
    RB_CHECK(!result.clamp.velocity_clamped);
    RB_CHECK(result.clamp.accel_clamped);
    RB_CHECK(result.clamp.accel_limited_joint >= 0);
    RB_CHECK(result.clamp.accel_clamp_max_delta_deg > 0.0);
    for (double q : result.filtered_q_deg) {
        RB_CHECK(q <= 0.02 + kEpsilon);
        RB_CHECK(q >= -kEpsilon);
    }
    for (int i = 0; i < rb_servo::kDof; ++i) {
        RB_CHECK(std::abs(result.clamp.q_before_safety_deg[i] - 0.02) < kEpsilon);
        RB_CHECK(std::abs(result.clamp.q_after_joint_limit_deg[i] - 0.02) < kEpsilon);
        RB_CHECK(std::abs(result.clamp.q_after_velocity_limit_deg[i] - 0.02) < kEpsilon);
        RB_CHECK(std::abs(result.clamp.q_after_accel_limit_deg[i] - result.filtered_q_deg[i]) < kEpsilon);
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
    cfg.servo.rate_hz = 50;
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
    }, std::chrono::milliseconds(1000)));
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
    }, std::chrono::milliseconds(1000)));
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

bool testRbpodoAsyncConfigRejectsPhysicalRealAndMissingRealEnv() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.unset();

    // Real/sim env gates retired: async streaming configs load without envs.
    const std::string missing_env_path = writeRbpodoAsyncConfig("missing-real-env");
    (void)rb_servo::loadConfigFromYaml(missing_env_path);
    ::unlink(missing_env_path.c_str());

    allow_motion.set("1");
    const std::string operation_real_path =
        writeRbpodoAsyncConfig("operation-real", "real", "simulation");
    bool physical_real_rejected = false;
    try {
        (void)rb_servo::loadConfigFromYaml(operation_real_path);
    } catch (const std::exception& exc) {
        physical_real_rejected = contains(exc.what(), "operation_mode=simulation");
    }
    ::unlink(operation_real_path.c_str());
    RB_CHECK(physical_real_rejected);
    return true;
}

bool testRbpodoAsyncServoLoopDoesNotBlockOnSlowAckWorker() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.rate_hz = 20;
    cfg.servo.worker_read_period_sec = 0.005;
    enableRbpodoAsyncStreaming(&cfg);
    cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_update_timeout_ms = 200.0;
    cfg.servo.rbpodo_async_streaming.ack_supervision.missing_ack_fault_after_ms = 200.0;
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
    left_backend->setSendSleepMs(80);
    right_backend->setSendSleepMs(80);
    rb_servo::DualArmCommand arm_motion = command(rb_servo::ControlMode::ArmMotion);
    arm_motion.left.timeout_sec = 1.0;
    arm_motion.right.timeout_sec = 1.0;
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil(
        [&] { return loop.motionState() == rb_servo::ServerMotionState::ArmedHold; },
        std::chrono::milliseconds(1000)
    ));

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.seq = 8801;
    target.left.q_target_deg = joints(8.0);
    target.right.q_target_deg = joints(8.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    target.left.timeout_sec = 1.0;
    target.right.timeout_sec = 1.0;
    buffer.setCommand(target);

    std::optional<rb_servo::ServoSnapshot> async_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (snapshot.command.seq == 8801 &&
            snapshot.left_last_send.acceptance_semantics == "async_enqueued" &&
            snapshot.right_last_send.acceptance_semantics == "async_enqueued") {
            async_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1000)));

    RB_CHECK(async_snapshot.has_value());
    RB_CHECK(async_snapshot->left_send_ok);
    RB_CHECK(async_snapshot->right_send_ok);
    RB_CHECK(async_snapshot->left_send_duration_us < 50'000.0);
    RB_CHECK(async_snapshot->right_send_duration_us < 50'000.0);
    loop.stop();
    RB_CHECK(async_snapshot->left_async_streaming.commands_enqueued_total > 0);
    RB_CHECK(left_backend->sendCount() > 0);
    RB_CHECK(right_backend->sendCount() > 0);
    return true;
}

bool testRbpodoAsyncSupervisionFaultLatchesServoLoop() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.rate_hz = 50;
    cfg.servo.worker_read_period_sec = 0.005;
    enableRbpodoAsyncStreaming(&cfg);
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Left,
            initial,
            true,
            rb_servo::BackendErrorKind::TransportTimeout
        ),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());

    std::optional<rb_servo::ServoSnapshot> fault_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (snapshot.fault_latched &&
            snapshot.latched_fault_reason == rb_servo::SafetyVerdict::SendFailure &&
            snapshot.left_async_streaming.supervision_state ==
                rb_servo::RbpodoAsyncStreamingSupervisionState::Fault) {
            fault_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1500)));

    loop.stop();
    RB_CHECK(fault_snapshot.has_value());
    RB_CHECK(fault_snapshot->left_async_streaming.commands_dropped_total > 0);
    RB_CHECK(fault_snapshot->left_async_streaming.ack_timeout_count > 0);
    RB_CHECK(fault_snapshot->fault_reason == "rbpodo async streaming supervision fault");
    return true;
}

bool testRbpodoAsyncSupervisionFaultIsAdvisoryInControllerSimulationWhenFlagEnabled() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.rate_hz = 50;
    cfg.servo.worker_read_period_sec = 0.005;
    cfg.servo.controller_simulation_async_supervision_nonlatching = true;
    enableRbpodoAsyncStreaming(&cfg);
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left,
        initial,
        true,
        rb_servo::BackendErrorKind::TransportTimeout
    );
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* left_backend = left.get();
    TestBackend* right_backend = right.get();
    rb_servo::DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr);

    RB_CHECK(loop.start());

    std::optional<rb_servo::ServoSnapshot> degraded_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (!snapshot.fault_latched &&
            snapshot.async_supervision_degraded &&
            snapshot.send_policy == "send_servo_j" &&
            snapshot.left_async_streaming.supervision_state ==
                rb_servo::RbpodoAsyncStreamingSupervisionState::Fault) {
            degraded_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1500)));

    const int left_send_count = left_backend->sendCount();
    const int right_send_count = right_backend->sendCount();
    RB_CHECK(waitUntil([&] {
        return !loop.faultLatched() &&
               left_backend->sendCount() > left_send_count &&
               right_backend->sendCount() > right_send_count;
    }, std::chrono::milliseconds(500)));
    loop.stop();
    RB_CHECK(degraded_snapshot.has_value());
    RB_CHECK(degraded_snapshot->left_async_streaming.commands_dropped_total > 0);
    RB_CHECK(degraded_snapshot->left_async_streaming.ack_timeout_count > 0);
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::Ok);
    return true;
}

bool testRbpodoAsyncSupervisionFlagDoesNotBypassPhysicalRealLatch() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoPhysicalRealConfig();
    cfg.servo.rate_hz = 50;
    cfg.servo.worker_read_period_sec = 0.005;
    cfg.servo.controller_simulation_async_supervision_nonlatching = true;
    enableRbpodoAsyncStreaming(&cfg);
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Left,
            initial,
            true,
            rb_servo::BackendErrorKind::TransportTimeout
        ),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());

    std::optional<rb_servo::ServoSnapshot> fault_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (snapshot.fault_latched &&
            !snapshot.async_supervision_degraded &&
            snapshot.latched_fault_reason == rb_servo::SafetyVerdict::SendFailure &&
            snapshot.left_async_streaming.supervision_state ==
                rb_servo::RbpodoAsyncStreamingSupervisionState::Fault) {
            fault_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1500)));

    loop.stop();
    RB_CHECK(fault_snapshot.has_value());
    RB_CHECK(fault_snapshot->fault_reason == "rbpodo async streaming supervision fault");
    return true;
}

bool testRbpodoAsyncHoldStreamsServoJWithoutLatch() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.rate_hz = 50;
    cfg.servo.worker_read_period_sec = 0.005;
    enableRbpodoAsyncStreaming(&cfg);
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    std::optional<rb_servo::ServoSnapshot> hold_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (!snapshot.fault_latched &&
            snapshot.command.left.mode == rb_servo::ControlMode::Hold &&
            snapshot.command.right.mode == rb_servo::ControlMode::Hold &&
            snapshot.send_policy == "send_servo_j" &&
            !snapshot.send_suppressed &&
            snapshot.left_async_streaming.commands_enqueued_total > 2 &&
            snapshot.right_async_streaming.commands_enqueued_total > 2 &&
            snapshot.left_async_streaming.supervision_state !=
                rb_servo::RbpodoAsyncStreamingSupervisionState::Fault &&
            snapshot.right_async_streaming.supervision_state !=
                rb_servo::RbpodoAsyncStreamingSupervisionState::Fault) {
            hold_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1000)));

    loop.stop();
    RB_CHECK(hold_snapshot.has_value());
    RB_CHECK(hold_snapshot->left_send_ok);
    RB_CHECK(hold_snapshot->right_send_ok);
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::Ok);
    return true;
}

bool testRealHoldFreezesLastReferenceNotDriftedActual() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoPhysicalRealConfig();
    cfg.servo.rate_hz = 100;
    cfg.safety.dq_max_deg_s = joints(10000.0);
    cfg.safety.ddq_max_deg_s2 = joints(100000.0);
    cfg.safety.max_tracking_error_deg = 1000.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* left_backend = left.get();
    TestBackend* right_backend = right.get();
    rb_servo::DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr);

    RB_CHECK(loop.start());
    rb_servo::DualArmCommand arm_motion = command(rb_servo::ControlMode::ArmMotion);
    arm_motion.left.timeout_sec = 1.0;
    arm_motion.right.timeout_sec = 1.0;
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil(
        [&] { return loop.motionState() == rb_servo::ServerMotionState::ArmedHold; },
        std::chrono::milliseconds(1000)
    ));

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.seq = 9101;
    target.left.q_target_deg = joints(6.0);
    target.right.q_target_deg = joints(6.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    target.left.timeout_sec = 1.0;
    target.right.timeout_sec = 1.0;
    buffer.setCommand(target);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        return loop.motionState() == rb_servo::ServerMotionState::Running &&
               sameJointArray(previous.left_q_target_deg, joints(6.0)) &&
               sameJointArray(previous.right_q_target_deg, joints(6.0));
    }, std::chrono::milliseconds(1000)));

    const rb_servo::JointArray drifted_actual = joints(2.0);
    left_backend->setAcceptSendWithoutStateUpdate(true);
    right_backend->setAcceptSendWithoutStateUpdate(true);
    left_backend->setActualJoints(drifted_actual);
    right_backend->setActualJoints(drifted_actual);

    // Hold must FREEZE at the last commanded reference (held_reference), NOT chase
    // the drifted measured actual: commanding the live actual every tick leaves the
    // servo with zero error and no restoring torque, so the arm creeps to its
    // gravity-settled pose (the startup/idle drift this guards against).
    const rb_servo::JointArray held_reference = joints(6.0);
    rb_servo::DualArmCommand hold = command(rb_servo::ControlMode::Hold);
    hold.seq = 9102;
    hold.left.timeout_sec = 1.0;
    hold.right.timeout_sec = 1.0;
    buffer.setCommand(hold);
    std::optional<rb_servo::ServoSnapshot> hold_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        if (snapshot.command.seq == 9102 &&
            snapshot.motion_state == rb_servo::ServerMotionState::ArmedHold &&
            snapshot.send_policy == "send_servo_j" &&
            !snapshot.send_suppressed &&
            !snapshot.fault_latched &&
            sameJointArray(previous.left_q_target_deg, held_reference) &&
            sameJointArray(previous.right_q_target_deg, held_reference)) {
            hold_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1000)));

    RB_CHECK(hold_snapshot.has_value());
    RB_CHECK(sameJointArray(hold_snapshot->left_sent_q_deg, held_reference));
    RB_CHECK(sameJointArray(hold_snapshot->right_sent_q_deg, held_reference));

    arm_motion.seq = 9103;
    arm_motion.host_time_ns = rb_servo::nowSteadyNs();
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        return snapshot.motion_state == rb_servo::ServerMotionState::ArmedHold &&
               !snapshot.fault_latched &&
               snapshot.send_policy == "send_servo_j" &&
               sameJointArray(previous.left_q_target_deg, held_reference) &&
               sameJointArray(previous.right_q_target_deg, held_reference);
    }, std::chrono::milliseconds(1000)));

    loop.stop();
    RB_CHECK(left_backend->resetCount() == 0);
    RB_CHECK(right_backend->resetCount() == 0);
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::Ok);
    return true;
}

bool testCompletedInitMotionCachedPacketDoesNotReanchorHoldToActual() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.rate_hz = 100;
    cfg.safety.self_collision.enable = true;
    cfg.safety.self_collision.monitor_only = true;
    const std::filesystem::path repo_root =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::filesystem::path unified_urdf_dir =
        repo_root.parent_path() /
        "mo_robot_descriptions/mo_robot_descriptions/robots/urdf/dual_rb3_730e";
    cfg.safety.self_collision.mesh.unified_urdf =
        (unified_urdf_dir / "dual_rb3_730e_ver5.urdf").string();
    cfg.safety.self_collision.mesh.package_dirs = {unified_urdf_dir.string()};
    if (!std::filesystem::is_regular_file(
            cfg.safety.self_collision.mesh.unified_urdf)) {
        std::cout << "SKIP: cached Init Motion Hold regression requires unified URDF ("
                  << cfg.safety.self_collision.mesh.unified_urdf << ")\n";
        return true;
    }
    cfg.safety.init_motion_planner.enable = true;
    cfg.safety.init_motion_planner.noop_tol_deg = 1.5;
    cfg.safety.init_motion_planner.waypoint_tol_deg = 1.5;
    cfg.safety.max_tracking_error_deg = 1000.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* left_backend = left.get();
    TestBackend* right_backend = right.get();
    rb_servo::DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr);

    RB_CHECK(loop.start());
    rb_servo::DualArmCommand arm_motion = command(rb_servo::ControlMode::ArmMotion);
    arm_motion.left.timeout_sec = 1.0;
    arm_motion.right.timeout_sec = 1.0;
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil(
        [&] { return loop.motionState() == rb_servo::ServerMotionState::ArmedHold; },
        std::chrono::milliseconds(1000)
    ));

    rb_servo::DualArmCommand init = command(rb_servo::ControlMode::JointTarget);
    init.seq = 9201;
    init.left.q_target_deg = initial;
    init.right.q_target_deg = initial;
    init.left.has_joint_target = true;
    init.right.has_joint_target = true;
    init.left.joint_target_profile = rb_servo::JointTargetProfile::InitMotion;
    init.right.joint_target_profile = rb_servo::JointTargetProfile::InitMotion;
    init.left.timeout_sec = 1.0;
    init.right.timeout_sec = 1.0;
    buffer.setCommand(init);

    // The first observation is a measured no-op.  Completion must immediately
    // hand command ownership to Hold even while CommandBuffer still returns the
    // cached Init Motion packet.
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.command.seq == init.seq &&
               snapshot.init_motion.status == "done" &&
               snapshot.command.left.mode == rb_servo::ControlMode::Hold &&
               snapshot.command.right.mode == rb_servo::ControlMode::Hold &&
               !snapshot.command.left.has_joint_target &&
               !snapshot.command.right.has_joint_target;
    }, std::chrono::milliseconds(1000)));

    // Simulate a hand load moving measured joints inside the no-op tolerance.
    // The old implementation treated every cached tick as a fresh request and
    // repeatedly snapped previous_sent to this q_actual value.
    left_backend->setAcceptSendWithoutStateUpdate(true);
    right_backend->setAcceptSendWithoutStateUpdate(true);
    left_backend->setActualJoints(joints(0.5));
    right_backend->setActualJoints(joints(0.5));

    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.command.seq == init.seq &&
               sameJointArray(snapshot.left_state.q_actual_deg, joints(0.5)) &&
               sameJointArray(snapshot.right_state.q_actual_deg, joints(0.5)) &&
               snapshot.command.left.mode == rb_servo::ControlMode::Hold &&
               snapshot.command.right.mode == rb_servo::ControlMode::Hold;
    }, std::chrono::milliseconds(1000)));
    sleepTicks();

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    const rb_servo::ServoTarget previous = loop.previousSentTarget();
    loop.stop();
    RB_CHECK(snapshot.command.seq == init.seq);
    RB_CHECK(snapshot.command.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(snapshot.command.right.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(sameJointArray(snapshot.left_sent_q_deg, initial));
    RB_CHECK(sameJointArray(snapshot.right_sent_q_deg, initial));
    RB_CHECK(sameJointArray(previous.left_q_target_deg, initial));
    RB_CHECK(sameJointArray(previous.right_q_target_deg, initial));
    RB_CHECK(!snapshot.fault_latched);
    return true;
}

bool testGripperSetpointSurvivesHoldRewrite() {
    // A SpaceMouse button-only gripper press (cap neutral) arrives as a Hold/Hold
    // command carrying gripper_target. The explicit-dual-hold rewrite path must
    // PRESERVE that gripper setpoint (gripper is forwarded from snapshot.command),
    // not drop it via makeHoldCommand — otherwise the gripper never actuates.
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoPhysicalRealConfig();
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );
    RB_CHECK(loop.start());

    rb_servo::DualArmCommand grip = command(rb_servo::ControlMode::Hold);
    grip.seq = 5501;
    grip.host_time_ns = rb_servo::nowSteadyNs();
    grip.left.timeout_sec = 1.0;
    grip.right.timeout_sec = 1.0;
    grip.left.has_gripper = true;
    grip.left.gripper_target = 10.0;    // left button -> close
    grip.right.has_gripper = true;
    grip.right.gripper_target = 100.0;  // right button -> open
    buffer.setCommand(grip);

    std::optional<rb_servo::ServoSnapshot> snap;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot s = loop.latestSnapshot();
        if (s.command.seq == 5501 &&
            s.command.left.has_gripper && s.command.right.has_gripper) {
            snap = s;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1000)));
    loop.stop();

    RB_CHECK(snap.has_value());
    RB_CHECK(snap->command.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(std::abs(snap->command.left.gripper_target - 10.0) < kEpsilon);
    RB_CHECK(std::abs(snap->command.right.gripper_target - 100.0) < kEpsilon);
    return true;
}

bool testEmergencyStopStillRequiresResetAfterArmMotion() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoPhysicalRealConfig();
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* left_backend = left.get();
    TestBackend* right_backend = right.get();
    rb_servo::DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr);

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    buffer.setCommand(command(rb_servo::ControlMode::EmergencyStop));
    RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::EmergencyLatched);
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::EmergencyStop);

    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::EmergencyLatched);
    RB_CHECK(snapshot.send_policy == "emergency_latched");
    RB_CHECK(snapshot.send_suppressed);
    RB_CHECK(left_backend->resetCount() == 0);
    RB_CHECK(right_backend->resetCount() == 0);
    return true;
}

bool testPhysicalRealSendFailureStillHardLatches() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoPhysicalRealConfig();
    cfg.safety.stop_both_arms_on_single_arm_error = false;
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(
            rb_servo::ArmId::Left,
            initial,
            true,
            rb_servo::BackendErrorKind::TransportTimeout
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
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::FaultLatched);
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::SendFailure);

    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::FaultLatched);
    RB_CHECK(snapshot.send_policy == "fault_latched");
    RB_CHECK(snapshot.send_suppressed);
    return true;
}

// In pgmode controller-sim with the flag enabled, a sustained command-vs-actual
// tracking divergence (the diagnostics_suspect controller never reports following
// the command) must NOT latch: it stays advisory (tracking_error_degraded=true),
// keeps following/sending, and never trips fault_latched.
bool testRbpodoTrackingErrorIsAdvisoryInControllerSimulationWhenFlagEnabled() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.rate_hz = 50;
    cfg.servo.worker_read_period_sec = 0.005;
    cfg.safety.max_tracking_error_deg = 2.0;
    cfg.safety.controller_simulation_tracking_error_nonlatching = true;
    const rb_servo::JointArray initial = joints(0.0);
    // accept_send_without_state_update=true freezes q_actual at initial while the
    // commanded target advances, opening a > max_tracking_error_deg gap (the
    // controller-not-following case) without any genuine physical motion.
    auto left = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left, initial, false,
        rb_servo::BackendErrorKind::ControllerRejected, std::nullopt, true);
    auto right = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right, initial, false,
        rb_servo::BackendErrorKind::ControllerRejected, std::nullopt, true);
    TestBackend* left_backend = left.get();
    TestBackend* right_backend = right.get();
    rb_servo::DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr);

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(30.0);
    target.right.q_target_deg = joints(30.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);

    std::optional<rb_servo::ServoSnapshot> degraded_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (!snapshot.fault_latched && snapshot.tracking_error_degraded) {
            degraded_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1500)));

    // Stays live: keeps issuing servo sends and never latches while degraded.
    const int left_send_count = left_backend->sendCount();
    const int right_send_count = right_backend->sendCount();
    RB_CHECK(waitUntil([&] {
        return !loop.faultLatched() &&
               left_backend->sendCount() > left_send_count &&
               right_backend->sendCount() > right_send_count;
    }, std::chrono::milliseconds(500)));
    loop.stop();
    RB_CHECK(degraded_snapshot.has_value());
    RB_CHECK(!loop.faultLatched());
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::Ok);
    return true;
}

// Fail-closed: the same flag set in a physical-real config (gate closed) must NOT
// suppress the tracking-error latch. Real mode keeps latching TrackingError.
bool testRbpodoTrackingErrorFlagDoesNotBypassPhysicalRealLatch() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoPhysicalRealConfig();
    cfg.servo.rate_hz = 50;
    cfg.servo.worker_read_period_sec = 0.005;
    cfg.safety.max_tracking_error_deg = 2.0;
    cfg.safety.controller_simulation_tracking_error_nonlatching = true;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left, initial, false,
        rb_servo::BackendErrorKind::ControllerRejected, std::nullopt, true);
    auto right = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right, initial, false,
        rb_servo::BackendErrorKind::ControllerRejected, std::nullopt, true);
    rb_servo::DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr);

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(30.0);
    target.right.q_target_deg = joints(30.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);

    std::optional<rb_servo::ServoSnapshot> fault_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (snapshot.fault_latched &&
            !snapshot.tracking_error_degraded &&
            snapshot.latched_fault_reason == rb_servo::SafetyVerdict::TrackingError) {
            fault_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1500)));

    loop.stop();
    RB_CHECK(fault_snapshot.has_value());
    return true;
}

bool testRbpodoAsyncReferenceSupervisionQRefUpdatesOk() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.rate_hz = 50;
    cfg.servo.worker_read_period_sec = 0.005;
    enableRbpodoAsyncStreaming(
        &cfg,
        rb_servo::RbpodoAsyncStreamingMode::SocketSendSupervised
    );
    cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_update_timeout_ms = 100.0;
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );
    RB_CHECK(loop.start());

    rb_servo::DualArmCommand arm_motion = command(rb_servo::ControlMode::ArmMotion);
    arm_motion.left.timeout_sec = 1.0;
    arm_motion.right.timeout_sec = 1.0;
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil(
        [&] { return loop.motionState() == rb_servo::ServerMotionState::ArmedHold; },
        std::chrono::milliseconds(1000)
    ));

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.seq = 8901;
    target.left.q_target_deg = joints(4.0);
    target.right.q_target_deg = joints(4.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    target.left.timeout_sec = 1.0;
    target.right.timeout_sec = 1.0;
    buffer.setCommand(target);

    std::optional<rb_servo::ServoSnapshot> ok_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (snapshot.command.seq == 8901 &&
            !snapshot.fault_latched &&
            snapshot.left_async_streaming.reference_supervision_state ==
                rb_servo::RbpodoAsyncStreamingSupervisionState::Ok &&
            snapshot.left_async_streaming.q_ref_target_error_deg_max <= 0.5) {
            ok_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1000)));

    loop.stop();
    RB_CHECK(ok_snapshot.has_value());
    RB_CHECK(ok_snapshot->left_async_streaming.last_q_ref_update_host_time_ns > 0);
    RB_CHECK(ok_snapshot->left_async_streaming.reference_supervision_fault_count == 0);
    return true;
}

bool testRbpodoAsyncReferenceSupervisionQRefStopsFaultsSocketSend() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.rate_hz = 100;
    cfg.servo.worker_read_period_sec = 0.005;
    enableRbpodoAsyncStreaming(
        &cfg,
        rb_servo::RbpodoAsyncStreamingMode::SocketSendSupervised
    );
    cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_update_timeout_ms = 20.0;
    cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_target_fault_after_ms = 500.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    left->setAdvanceRobotTimeOnRead(false);
    right->setAdvanceRobotTimeOnRead(false);
    rb_servo::DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr);
    RB_CHECK(loop.start());

    std::optional<rb_servo::ServoSnapshot> fault_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (snapshot.fault_latched) {
            fault_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1000)));

    loop.stop();
    RB_CHECK(fault_snapshot.has_value());
    if (fault_snapshot->left_async_streaming.reference_supervision_reason !=
        "async_q_ref_update_timeout") {
        std::cerr << "unexpected q_ref stale supervision reason: "
                  << fault_snapshot->left_async_streaming.reference_supervision_reason
                  << " left_state="
                  << static_cast<int>(fault_snapshot->left_async_streaming.reference_supervision_state)
                  << " left_failure=" << fault_snapshot->left_async_streaming.last_failure
                  << " fault_reason=" << fault_snapshot->fault_reason
                  << " latched="
                  << static_cast<int>(fault_snapshot->latched_fault_reason)
                  << " right_reason="
                  << fault_snapshot->right_async_streaming.reference_supervision_reason
                  << " right_state="
                  << static_cast<int>(fault_snapshot->right_async_streaming.reference_supervision_state)
                  << " right_failure=" << fault_snapshot->right_async_streaming.last_failure
                  << "\n";
    }
    RB_CHECK(fault_snapshot->latched_fault_reason == rb_servo::SafetyVerdict::SendFailure);
    RB_CHECK(
        fault_snapshot->left_async_streaming.reference_supervision_state ==
        rb_servo::RbpodoAsyncStreamingSupervisionState::Fault
    );
    RB_CHECK(
        fault_snapshot->left_async_streaming.reference_supervision_reason ==
        "async_q_ref_update_timeout"
    );
    RB_CHECK(fault_snapshot->left_async_streaming.q_ref_watchdog_miss_count > 0);
    RB_CHECK(fault_snapshot->left_async_streaming.reference_supervision_fault_count > 0);
    return true;
}

bool testRbpodoAsyncReferenceSupervisionTargetErrorFaultsSocketSend() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.rate_hz = 100;
    cfg.servo.worker_read_period_sec = 0.005;
    enableRbpodoAsyncStreaming(
        &cfg,
        rb_servo::RbpodoAsyncStreamingMode::SocketSendSupervised
    );
    cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_update_timeout_ms = 500.0;
    cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_target_tolerance_deg = 0.5;
    cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_target_fault_after_ms = 20.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left,
        initial,
        false,
        rb_servo::BackendErrorKind::ControllerRejected,
        std::nullopt,
        true
    );
    auto right = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right,
        initial,
        false,
        rb_servo::BackendErrorKind::ControllerRejected,
        std::nullopt,
        true
    );
    left->setFreezeReferenceOnSend(true);
    right->setFreezeReferenceOnSend(true);
    rb_servo::DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr);
    RB_CHECK(loop.start());

    rb_servo::DualArmCommand arm_motion = command(rb_servo::ControlMode::ArmMotion);
    arm_motion.left.timeout_sec = 1.0;
    arm_motion.right.timeout_sec = 1.0;
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil(
        [&] { return loop.motionState() == rb_servo::ServerMotionState::ArmedHold; },
        std::chrono::milliseconds(1000)
    ));

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.seq = 8902;
    target.left.q_target_deg = joints(5.0);
    target.right.q_target_deg = joints(5.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    target.left.timeout_sec = 1.0;
    target.right.timeout_sec = 1.0;
    buffer.setCommand(target);

    std::optional<rb_servo::ServoSnapshot> fault_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (snapshot.fault_latched &&
            snapshot.latched_fault_reason == rb_servo::SafetyVerdict::SendFailure &&
            snapshot.left_async_streaming.reference_supervision_state ==
                rb_servo::RbpodoAsyncStreamingSupervisionState::Fault &&
            snapshot.left_async_streaming.reference_supervision_reason ==
                "async_q_ref_target_error") {
            fault_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1500)));

    loop.stop();
    RB_CHECK(fault_snapshot.has_value());
    RB_CHECK(fault_snapshot->left_async_streaming.q_ref_target_error_deg_max > 0.5);
    RB_CHECK(!fault_snapshot->left_safety_tracking.controller_simulation_physical_motion_detected);
    return true;
}

bool testRbpodoAsyncReferenceSupervisionTargetErrorIsAdvisoryWhenFlagEnabled() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.rate_hz = 100;
    cfg.servo.worker_read_period_sec = 0.005;
    cfg.servo.controller_simulation_async_supervision_nonlatching = true;
    enableRbpodoAsyncStreaming(
        &cfg,
        rb_servo::RbpodoAsyncStreamingMode::SocketSendSupervised
    );
    cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_update_timeout_ms = 500.0;
    cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_target_tolerance_deg = 0.5;
    cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_target_fault_after_ms = 20.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left,
        initial,
        false,
        rb_servo::BackendErrorKind::ControllerRejected,
        std::nullopt,
        true
    );
    auto right = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right,
        initial,
        false,
        rb_servo::BackendErrorKind::ControllerRejected,
        std::nullopt,
        true
    );
    TestBackend* left_backend = left.get();
    TestBackend* right_backend = right.get();
    left_backend->setFreezeReferenceOnSend(true);
    right_backend->setFreezeReferenceOnSend(true);
    rb_servo::DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr);
    RB_CHECK(loop.start());

    rb_servo::DualArmCommand arm_motion = command(rb_servo::ControlMode::ArmMotion);
    arm_motion.left.timeout_sec = 1.0;
    arm_motion.right.timeout_sec = 1.0;
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil(
        [&] { return loop.motionState() == rb_servo::ServerMotionState::ArmedHold; },
        std::chrono::milliseconds(1000)
    ));

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.seq = 8903;
    target.left.q_target_deg = joints(5.0);
    target.right.q_target_deg = joints(5.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    target.left.timeout_sec = 1.0;
    target.right.timeout_sec = 1.0;
    buffer.setCommand(target);

    std::optional<rb_servo::ServoSnapshot> degraded_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (snapshot.command.seq == 8903 &&
            !snapshot.fault_latched &&
            snapshot.async_supervision_degraded &&
            snapshot.send_policy == "send_servo_j" &&
            snapshot.left_async_streaming.reference_supervision_state ==
                rb_servo::RbpodoAsyncStreamingSupervisionState::Fault &&
            snapshot.left_async_streaming.reference_supervision_reason ==
                "async_q_ref_target_error") {
            degraded_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1500)));

    const int left_send_count = left_backend->sendCount();
    const int right_send_count = right_backend->sendCount();
    RB_CHECK(waitUntil([&] {
        return !loop.faultLatched() &&
               left_backend->sendCount() > left_send_count &&
               right_backend->sendCount() > right_send_count;
    }, std::chrono::milliseconds(500)));
    loop.stop();
    RB_CHECK(degraded_snapshot.has_value());
    RB_CHECK(degraded_snapshot->left_async_streaming.q_ref_target_error_deg_max > 0.5);
    RB_CHECK(degraded_snapshot->left_async_streaming.reference_supervision_fault_count > 0);
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::Ok);
    return true;
}

bool testRbpodoAsyncReferenceSupervisionInvalidQRefFaults() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.rate_hz = 100;
    cfg.servo.worker_read_period_sec = 0.005;
    enableRbpodoAsyncStreaming(
        &cfg,
        rb_servo::RbpodoAsyncStreamingMode::SocketSendSupervised
    );
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    left->setQRefValid(false);
    right->setQRefValid(false);
    rb_servo::DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr);
    RB_CHECK(loop.start());

    std::optional<rb_servo::ServoSnapshot> fault_snapshot;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        if (snapshot.fault_latched &&
            snapshot.latched_fault_reason == rb_servo::SafetyVerdict::SendFailure &&
            snapshot.left_async_streaming.commands_enqueued_total > 0 &&
            snapshot.left_async_streaming.reference_supervision_state ==
                rb_servo::RbpodoAsyncStreamingSupervisionState::Fault &&
            snapshot.left_async_streaming.reference_supervision_reason ==
                "async_reference_q_ref_invalid") {
            fault_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1000)));

    loop.stop();
    RB_CHECK(fault_snapshot.has_value());
    RB_CHECK(fault_snapshot->left_async_streaming.reference_supervision_fault_count > 0);
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
    cfg.cartesian_control.max_linear_move_speed_m_s = 0.045;
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
    snapshot.left_state.acquisition_sequence = 31;
    snapshot.right_state.acquisition_sequence = 32;
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
    snapshot.left_last_read.read_exchange_timing.available = true;
    snapshot.left_last_read.read_exchange_timing.exchange_sequence = 77;
    snapshot.left_last_read.read_exchange_timing.source = "rbpodo_sdk_request_data";
    snapshot.left_last_read.read_exchange_timing.request_data_call_start_steady_ns = 100'000;
    snapshot.left_last_read.read_exchange_timing.request_data_call_start_system_ns = 200'000;
    snapshot.left_last_read.read_exchange_timing.request_data_call_return_steady_ns = 112'000;
    snapshot.left_last_read.read_exchange_timing.request_data_call_return_system_ns = 212'000;
    snapshot.left_last_read.read_exchange_timing.request_data_call_duration_us = 12.0;
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
    snapshot.left_cartesian_solve.ik_branch_jump_rate_limited = true;
    snapshot.left_cartesian_solve.ik_branch_jump_raw_deg = 18.0;
    snapshot.left_cartesian_solve.ik_branch_jump_limit_deg = 4.0;
    snapshot.left_cartesian_solve.ik_branch_jump_scale = 4.0 / 18.0;
    snapshot.left_cartesian_solve.ik_branch_jump_retry_count = 2;
    snapshot.left_cartesian_solve.q_ik_seed_deg = joints(1.0);
    snapshot.left_cartesian_solve.q_ik_raw_solution_deg = joints(19.0);
    snapshot.left_cartesian_solve.q_ik_solution_deg = joints(5.0);
    snapshot.left_cartesian_solve.q_ik_raw_delta_deg = joints(18.0);
    snapshot.left_cartesian_solve.q_ik_delta_deg = joints(4.0);
    snapshot.left_cartesian_solve.safety_clamp.present = true;
    snapshot.left_cartesian_solve.safety_clamp.q_before_safety_deg = joints(6.0);
    snapshot.left_cartesian_solve.safety_clamp.q_after_joint_limit_deg = joints(6.0);
    snapshot.left_cartesian_solve.safety_clamp.q_after_velocity_limit_deg = joints(5.0);
    snapshot.left_cartesian_solve.safety_clamp.q_after_accel_limit_deg = joints(4.5);
    snapshot.left_cartesian_solve.safety_clamp.velocity_clamped = true;
    snapshot.left_cartesian_solve.safety_clamp.accel_clamped = true;
    snapshot.left_cartesian_solve.safety_clamp.velocity_clamp_max_delta_deg = 1.0;
    snapshot.left_cartesian_solve.safety_clamp.accel_clamp_max_delta_deg = 0.5;
    snapshot.left_cartesian_solve.safety_clamp.velocity_limited_joint = 0;
    snapshot.left_cartesian_solve.safety_clamp.accel_limited_joint = 1;
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
    snapshot.left_force_torque.gravity_tcp = {1.0, 2.0, 3.0};
    snapshot.left_force_torque.t_tcp_sensor = {
        0.0, 0.0, -0.202642, 0.0, 0.0, 1.5707963267948966,
    };
    snapshot.left_force_torque.payload_identification_inhibit = true;
    snapshot.left_force_torque.joint_target_profile = "payload_identification";
    snapshot.payload_identification_config.enable = true;
    snapshot.payload_identification_config.wrench_convention = "sensor_reaction";
    snapshot.payload_identification_config.min_poses = 7;
    snapshot.payload_identification_config.arrival_tolerance_deg = 0.4;
    snapshot.payload_identification_config.settle_sec = 0.6;
    snapshot.payload_identification_config.samples_per_pose = 250;
    snapshot.payload_identification_config.max_force_stddev_n = 0.8;
    snapshot.payload_identification_config.max_torque_stddev_nm = 0.09;
    snapshot.payload_identification_config.max_force_fit_rms_n = 1.1;
    snapshot.payload_identification_config.max_torque_fit_rms_nm = 0.12;
    snapshot.payload_identification_config.max_design_condition_number = 1.0e5;
    snapshot.logger_dropped_samples = 0;

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));

    const char* top_keys[] = {
        "schema_version", "tick", "host_time_ns", "loop_start_time_ns", "loop_end_time_ns",
        "period_ms", "jitter_ms", "filter_dt_ms", "command_seq", "command_source", "observed_mode", "observed_backend",
        "cartesian_control_snapshot", "kinematics_snapshot", "force_torque",
        "startup_validation", "left", "right",
        "send_skew_us", "send_within_period", "send_period_overrun", "send_command_deadline_missed",
        "send_deadline_hit", "send_deadline_hit_deprecated_alias_for",
        "send_suppressed", "send_policy", "safety_verdict", "motion_state", "fault_latched",
        "async_supervision_degraded", "tracking_error_degraded", "latched_fault_reason", "fault_reason",
        "logger_dropped_samples", "logger_health",
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
        "last_read", "last_send", "robot_time_ns", "host_time_ns", "acquisition_sequence",
        "error_code", "state_age_us",
        "send_within_period", "send_period_overrun", "send_command_deadline_missed",
        "send_deadline_hit", "send_deadline_hit_deprecated_alias_for",
        "tcp_stand", "tcp_base", "tcp_deferred", "fk_duration_us", "cartesian_solve", "worker",
        "transport", "cartesian_available", "cartesian_unavailable_reason", "cartesian_gate",
        "controller_simulation_cartesian_enabled", "streaming_cartesian_physical_real_enabled",
        "controller_simulation_cartesian_enabled_for_current_command",
        "controller_simulation_streaming_cartesian_available",
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
    RB_CHECK(
        json.at("left").at("force_torque").at("t_tcp_sensor").at(2).get<double>()
        == -0.202642
    );
    RB_CHECK(
        json.at("left").at("force_torque").at("t_tcp_sensor").at(5).get<double>()
        == 1.5707963267948966
    );
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
    RB_CHECK(json.at("cartesian_control_snapshot").at("max_linear_move_speed_m_s").get<double>() == 0.045);
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
    const auto& payload_identification =
        json.at("force_torque").at("payload_identification");
    RB_CHECK(payload_identification.at("enable").get<bool>());
    RB_CHECK(payload_identification.at("wrench_convention").get<std::string>() ==
        "sensor_reaction");
    RB_CHECK(payload_identification.at("min_poses").get<int>() == 7);
    RB_CHECK(payload_identification.at("arrival_tolerance_deg").get<double>() == 0.4);
    RB_CHECK(payload_identification.at("settle_sec").get<double>() == 0.6);
    RB_CHECK(payload_identification.at("samples_per_pose").get<int>() == 250);
    RB_CHECK(payload_identification.at("max_force_stddev_n").get<double>() == 0.8);
    RB_CHECK(payload_identification.at("max_torque_stddev_nm").get<double>() == 0.09);
    RB_CHECK(payload_identification.at("max_force_fit_rms_n").get<double>() == 1.1);
    RB_CHECK(payload_identification.at("max_torque_fit_rms_nm").get<double>() == 0.12);
    RB_CHECK(payload_identification.at("max_design_condition_number").get<double>() == 1.0e5);
    const auto& left_force_torque = json.at("left").at("force_torque");
    RB_CHECK(left_force_torque.at("gravity_tcp").size() == 3);
    RB_CHECK(left_force_torque.at("gravity_tcp").at(0).get<double>() == 1.0);
    RB_CHECK(left_force_torque.at("gravity_tcp").at(1).get<double>() == 2.0);
    RB_CHECK(left_force_torque.at("gravity_tcp").at(2).get<double>() == 3.0);
    RB_CHECK(left_force_torque.at("payload_identification_inhibit").get<bool>());
    RB_CHECK(left_force_torque.at("joint_target_profile").get<std::string>() ==
        "payload_identification");
    RB_CHECK(json.at("left").at("mode").get<std::string>() == "JointTarget");
    RB_CHECK(json.at("right").at("mode").get<std::string>() == "Hold");
    RB_CHECK(json.at("left").at("cartesian_gate").at("backend_type").get<std::string>() == "mock");
    RB_CHECK(!json.at("left").at("cartesian_gate").at("allow_in_controller_simulation").get<bool>());
    RB_CHECK(!json.at("left").at("cartesian_gate").at("allow_controller_simulation_motion").get<bool>());
    RB_CHECK(!json.at("left").at("cartesian_gate").at("controller_simulation_streaming_cartesian_available").get<bool>());
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
    const auto& reqdata_exchange = json.at("left").at("last_read").at("reqdata_exchange");
    RB_CHECK(reqdata_exchange.at("available").get<bool>());
    RB_CHECK(reqdata_exchange.at("sequence").get<uint64_t>() == 77);
    RB_CHECK(reqdata_exchange.at("source").get<std::string>() == "rbpodo_sdk_request_data");
    RB_CHECK(reqdata_exchange.at("call_start_steady_ns").get<uint64_t>() == 100'000);
    RB_CHECK(reqdata_exchange.at("call_start_system_ns").get<uint64_t>() == 200'000);
    RB_CHECK(reqdata_exchange.at("call_return_steady_ns").get<uint64_t>() == 112'000);
    RB_CHECK(reqdata_exchange.at("call_return_system_ns").get<uint64_t>() == 212'000);
    RB_CHECK(reqdata_exchange.at("call_duration_us").get<double>() == 12.0);
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
    RB_CHECK(json.at("left").at("acquisition_sequence").get<uint64_t>() == 31);
    RB_CHECK(json.at("right").at("acquisition_sequence").get<uint64_t>() == 32);
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
    RB_CHECK(json.at("left").at("cartesian_solve").at("ik_branch_jump_rate_limited").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_solve").at("ik_branch_jump_raw_deg").get<double>() == 18.0);
    RB_CHECK(json.at("left").at("cartesian_solve").at("ik_branch_jump_limit_deg").get<double>() == 4.0);
    RB_CHECK(json.at("left").at("cartesian_solve").at("ik_branch_jump_retry_count").get<int>() == 2);
    RB_CHECK(jsonArrayHasSixFinite(json.at("left").at("cartesian_solve").at("q_ik_seed_deg")));
    RB_CHECK(json.at("left").at("cartesian_solve").at("q_ik_delta_deg").at(0).get<double>() == 4.0);
    RB_CHECK(json.at("left").at("cartesian_solve").at("safety_clamp_present").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_solve").at("safety_velocity_clamped").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_solve").at("safety_accel_clamped").get<bool>());
    RB_CHECK(json.at("left").at("cartesian_solve").at("q_after_velocity_limit_deg").at(0).get<double>() == 5.0);
    RB_CHECK(json.at("left").at("cartesian_solve").at("safety_velocity_limited_joint").get<int>() == 0);
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
    worker_cfg.servo.rbpodo_async_streaming.enable = true;
    worker_cfg.servo.rbpodo_async_streaming.mode =
        rb_servo::RbpodoAsyncStreamingMode::SocketSendSupervised;
    rb_servo::ServoSnapshot worker_snapshot = snapshot;
    worker_snapshot.loop_end_time_ns = 20'000;
    worker_snapshot.left_worker_telemetry.worker_command_drops_total = 3;
    worker_snapshot.left_worker_telemetry.worker_pending_overwrites_total = 3;
    worker_snapshot.left_worker_telemetry.worker_last_dropped_seq = 1201;
    worker_snapshot.left_worker_telemetry.worker_last_enqueued_seq = 1204;
    worker_snapshot.left_worker_telemetry.worker_last_dispatched_seq = 1204;
    worker_snapshot.left_worker_telemetry.worker_last_completed_seq = 1204;
    worker_snapshot.left_async_streaming.commands_enqueued_total = 10;
    worker_snapshot.left_async_streaming.commands_sent_total = 9;
    worker_snapshot.left_async_streaming.commands_acked_total = 0;
    worker_snapshot.left_async_streaming.commands_socket_sent_total = 9;
    worker_snapshot.left_async_streaming.commands_overwritten_total = 1;
    worker_snapshot.left_async_streaming.commands_dropped_total = 1;
    worker_snapshot.left_async_streaming.ack_timeout_count = 2;
    worker_snapshot.left_async_streaming.last_command_seq = 1300;
    worker_snapshot.left_async_streaming.last_sent_seq = 1299;
    worker_snapshot.left_async_streaming.last_q_ref_update_host_time_ns = 1301;
    worker_snapshot.left_async_streaming.last_tcp_ref_update_host_time_ns = 1302;
    worker_snapshot.left_async_streaming.q_ref_update_age_ms = 4.5;
    worker_snapshot.left_async_streaming.tcp_ref_update_age_ms = 5.5;
    worker_snapshot.left_async_streaming.q_ref_target_error_deg_max = 0.25;
    worker_snapshot.left_async_streaming.tcp_ref_target_error_m = 0.012;
    worker_snapshot.left_async_streaming.last_async_send_duration_us = 2500.0;
    worker_snapshot.left_async_streaming.last_async_ack_duration_us = 0.0;
    worker_snapshot.left_async_streaming.last_async_acceptance_semantics = "socket_send_only";
    worker_snapshot.left_async_streaming.worker_backlog = 1;
    worker_snapshot.left_async_streaming.supervision_state =
        rb_servo::RbpodoAsyncStreamingSupervisionState::Warning;
    worker_snapshot.left_async_streaming.reference_supervision_state =
        rb_servo::RbpodoAsyncStreamingSupervisionState::Warning;
    worker_snapshot.left_async_streaming.reference_supervision_reason =
        "async_q_ref_target_error";
    worker_snapshot.left_async_streaming.reference_supervision_fault_count = 1;
    rb_servo::BackendTransportTelemetry transport;
    transport.connect_attempts_total = 2;
    transport.connect_failures_total = 1;
    transport.connect_attempts_suppressed_total = 3;
    transport.connections_opened_total = 1;
    transport.reconnects_total = 1;
    transport.requests_total = 12;
    transport.read_syscalls_total = 13;
    transport.write_syscalls_total = 14;
    transport.last_connect_error_name = "connect_backoff";
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
    const nlohmann::json& async = worker_json.at("left").at("async_streaming");
    RB_CHECK(async.at("enabled").get<bool>());
    RB_CHECK(async.at("mode").get<std::string>() == "socket_send_supervised");
    RB_CHECK(async.at("commands_enqueued_total").get<uint64_t>() == 10);
    RB_CHECK(async.at("commands_sent_total").get<uint64_t>() == 9);
    RB_CHECK(async.at("commands_acked_total").get<uint64_t>() == 0);
    RB_CHECK(async.at("commands_socket_sent_total").get<uint64_t>() == 9);
    RB_CHECK(async.at("commands_overwritten_total").get<uint64_t>() == 1);
    RB_CHECK(async.at("commands_dropped_total").get<uint64_t>() == 1);
    RB_CHECK(async.at("ack_timeout_count").get<uint64_t>() == 2);
    RB_CHECK(async.at("last_command_seq").get<uint64_t>() == 1300);
    RB_CHECK(async.at("last_sent_seq").get<uint64_t>() == 1299);
    RB_CHECK(async.at("last_q_ref_update_host_time_ns").get<uint64_t>() == 1301);
    RB_CHECK(async.at("last_tcp_ref_update_host_time_ns").get<uint64_t>() == 1302);
    RB_CHECK(async.at("q_ref_update_age_ms").get<double>() == 4.5);
    RB_CHECK(async.at("tcp_ref_update_age_ms").get<double>() == 5.5);
    RB_CHECK(async.at("q_ref_target_error_deg_max").get<double>() == 0.25);
    RB_CHECK(async.at("tcp_ref_target_error_m").get<double>() == 0.012);
    RB_CHECK(async.at("last_async_send_duration_us").get<double>() == 2500.0);
    RB_CHECK(async.at("last_async_ack_duration_us").get<double>() == 0.0);
    RB_CHECK(async.at("last_async_acceptance_semantics").get<std::string>() == "socket_send_only");
    RB_CHECK(async.at("async_worker_backlog").get<uint64_t>() == 1);
    RB_CHECK(async.at("async_supervision_state").get<std::string>() == "warning");
    RB_CHECK(async.at("reference_supervision_state").get<std::string>() == "warning");
    RB_CHECK(async.at("reference_supervision_reason").get<std::string>() == "async_q_ref_target_error");
    RB_CHECK(async.at("reference_supervision_fault_count").get<uint64_t>() == 1);
    const nlohmann::json& transport_json = worker_json.at("left").at("transport");
    RB_CHECK(transport_json.at("connect_attempts_total").get<uint64_t>() == 2);
    RB_CHECK(transport_json.at("connect_failures_total").get<uint64_t>() == 1);
    RB_CHECK(transport_json.at("connect_attempts_suppressed_total").get<uint64_t>() == 3);
    RB_CHECK(transport_json.at("connections_opened_total").get<uint64_t>() == 1);
    RB_CHECK(transport_json.at("reconnects_total").get<uint64_t>() == 1);
    RB_CHECK(transport_json.at("requests_total").get<uint64_t>() == 12);
    RB_CHECK(transport_json.at("read_syscalls_total").get<uint64_t>() == 13);
    RB_CHECK(transport_json.at("write_syscalls_total").get<uint64_t>() == 14);
    RB_CHECK(transport_json.at("last_connect_error_name").get<std::string>() == "connect_backoff");
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

bool testServoLoggerAppendsTcpPoseTargetDebugColumns() {
    rb_servo::LoggingConfig cfg;
    cfg.enable = true;
    cfg.directory = "/tmp/rb-servo-logger-columns-test-" + std::to_string(getpid());
    cfg.queue_capacity = 4;
    cfg.flush_period_ms = 1;
    std::filesystem::remove_all(cfg.directory);

    rb_servo::ServoLogger logger(cfg);
    RB_CHECK(logger.start());

    rb_servo::ServoSample sample;
    sample.tick = 1;
    sample.loop_start_time_ns = 1000;
    sample.loop_end_time_ns = 3000;
    sample.period_ms = 2.0;
    sample.command.seq = 42;
    sample.chunk_frame.wire_seq = 70;
    sample.chunk_frame.recv_seq = 71;
    sample.chunk_frame.policy_dt_sec = 0.0334;
    sample.chunk_frame.horizon = 16;
    sample.chunk_frame.execute_steps = 12;
    sample.chunk_frame.runway_steps = 4;
    sample.chunk_frame.age_ms = 8.5;
    sample.chunk_frame.interarrival_ms = 401.0;
    sample.chunk_frame.inference_seq = 72;
    sample.chunk_frame.inference_latency_ms = 77.5;
    sample.chunk_frame.camera_bundle_seq = 73;
    sample.chunk_frame.camera_right_focus_score = 45.5;
    sample.left_cartesian_solve.tcp_target_profile = "umi_large_smooth";
    sample.left_cartesian_solve.smd_profile_nf_linear_hz = 1.0;
    sample.left_cartesian_solve.smd_profile_nf_angular_hz = 1.1;
    sample.left_cartesian_solve.smd_profile_velocity_feedforward = true;
    sample.left_cartesian_solve.smd_profile_max_linear_velocity_m_s = 0.35;
    sample.left_cartesian_solve.smd_profile_max_linear_accel_m_s2 = 0.8;
    sample.left_cartesian_solve.smd_profile_max_angular_velocity_rad_s = 1.3;
    sample.left_cartesian_solve.smd_profile_max_angular_accel_rad_s2 = 5.0;
    sample.left_cartesian_solve.delta_twist_frame_rows = 16;
    sample.left_cartesian_solve.delta_twist_normal_budget = 12;
    sample.left_cartesian_solve.delta_twist_total_budget = 14;
    sample.left_cartesian_solve.delta_twist_steps_remaining = 9;
    sample.left_cartesian_solve.delta_twist_clamp_mask = 65;
    sample.left_cartesian_solve.delta_twist_accel_cmd =
        rb_servo::Vec6{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    sample.left_cartesian_solve.ik_branch_jump_rate_limited = true;
    sample.left_cartesian_solve.ik_branch_jump_raw_deg = 12.0;
    sample.left_cartesian_solve.ik_branch_jump_limit_deg = 4.0;
    sample.left_cartesian_solve.ik_branch_jump_scale = 0.3333333333333333;
    sample.left_cartesian_solve.ik_branch_jump_retry_count = 3;
    sample.left_cartesian_solve.q_ik_seed_deg = joints(1.0);
    sample.left_cartesian_solve.q_ik_raw_solution_deg = joints(13.0);
    sample.left_cartesian_solve.q_ik_solution_deg = joints(5.0);
    sample.left_cartesian_solve.q_ik_raw_delta_deg = joints(12.0);
    sample.left_cartesian_solve.q_ik_delta_deg = joints(4.0);
    sample.left_cartesian_solve.safety_clamp.present = true;
    sample.left_cartesian_solve.safety_clamp.q_before_safety_deg = joints(6.0);
    sample.left_cartesian_solve.safety_clamp.q_after_joint_limit_deg = joints(6.0);
    sample.left_cartesian_solve.safety_clamp.q_after_velocity_limit_deg = joints(5.0);
    sample.left_cartesian_solve.safety_clamp.q_after_accel_limit_deg = joints(4.5);
    sample.left_cartesian_solve.safety_clamp.velocity_clamped = true;
    sample.left_cartesian_solve.safety_clamp.accel_clamped = true;
    sample.left_cartesian_solve.safety_clamp.velocity_clamp_max_delta_deg = 1.0;
    sample.left_cartesian_solve.safety_clamp.accel_clamp_max_delta_deg = 0.5;
    sample.left_cartesian_solve.safety_clamp.velocity_limited_joint = 0;
    sample.left_cartesian_solve.safety_clamp.accel_limited_joint = 1;
    sample.left_mode_before_init_sequencer = "JointTarget";
    sample.right_mode_before_init_sequencer = "TcpPoseTarget";
    sample.left_mode_after_init_sequencer = "JointTarget";
    sample.right_mode_after_init_sequencer = "TcpPoseTarget";
    sample.left_joint_target_profile_before_init_sequencer = "init_motion";
    sample.right_joint_target_profile_before_init_sequencer = "direct";
    sample.left_joint_target_profile_after_init_sequencer = "direct";
    sample.right_joint_target_profile_after_init_sequencer = "direct";
    sample.init_motion_left.status = "executing";
    sample.init_motion_right.status = "idle";
    sample.init_motion.status = "executing";
    sample.init_motion_left.waypoint_index = 2;
    sample.init_motion_left.waypoint_count = 7;
    sample.init_motion_left.dist_to_goal_deg = 3.5;
    sample.init_motion_left.clear_threshold_m = 0.023;
    sample.init_motion_left.external_clear_threshold_m = 0.0075;
    sample.init_motion_left.nearest_pair = "left_link <-> stand";
    sample.init_motion_left.nearest_pair_distance_m = 0.018;
    sample.init_motion_left.goal_nearest_pair_name_a = "left_link0";
    sample.init_motion_left.goal_nearest_pair_name_b = "left_link2";
    sample.init_motion_left.goal_nearest_pair_category = "intra-arm";
    sample.init_motion_left.goal_nearest_pair_distance_m = 0.0188;
    sample.init_motion_left.goal_clear_threshold_self_m = 0.023;
    sample.init_motion_left.goal_clear_margin_deficit_m = 0.0042;
    sample.init_motion.clear_threshold_m = 0.023;
    sample.init_motion.external_clear_threshold_m = 0.0075;
    sample.init_motion.nearest_pair = "left_link <-> stand";
    sample.init_motion.nearest_pair_distance_m = 0.018;
    sample.init_motion.goal_nearest_pair_name_a = "left_link0";
    sample.init_motion.goal_nearest_pair_name_b = "left_link2";
    sample.init_motion.goal_nearest_pair_category = "intra-arm";
    sample.init_motion.goal_nearest_pair_distance_m = 0.0188;
    sample.init_motion.goal_clear_threshold_self_m = 0.023;
    sample.init_motion.goal_clear_margin_deficit_m = 0.0042;
    sample.non_init_arm_preserved_mode = "TcpPoseTarget";
    sample.single_arm_freeze_other_arm = false;
    sample.left_force_torque.enabled = true;
    sample.left_force_torque.source = "rbpodo_eft";
    sample.left_force_torque.source_assurance = "controller_frame_only";
    sample.left_force_torque.raw_sensor_wrench.fz = -22.5;
    sample.left_force_torque.t_tcp_sensor = {
        0.0, 0.0, -0.202642, 0.0, 0.0, 1.5707963267948966,
    };
    sample.left_force_torque.control_external_wrench.fz = 7.25;
    sample.left_force_torque.healthy = true;
    sample.left_force_torque.freshness_value = 1234;
    sample.left_force_torque.freshness_advanced = true;
    sample.left_force_torque.reason = "ok";
    sample.left_force_torque.auto_tare_enabled = true;
    sample.left_force_torque.tare_valid = true;
    sample.left_force_torque.tare_state = "accepted";
    sample.left_force_torque.tare_sample_count = 500;
    sample.left_force_torque.tare_generation = 3;
    sample.left_force_torque.tare_reason = "accepted";
    sample.left_force_torque.residual_tare_tcp.fz = 23.5;
    sample.left_force_torque.gravity_tcp = {1.25, -2.5, -9.0};
    sample.left_force_torque.payload_identification_inhibit = true;
    sample.left_force_torque.joint_target_profile = "payload_identification";
    sample.left_force_control.enabled = true;
    sample.left_force_control.operating_mode = "monitor";
    sample.left_force_control.state = "release_braking";
    sample.left_force_control.compliance_frame = "sensor_origin";
    sample.left_force_control.compliance_frame_pose_valid = true;
    sample.left_force_control.compliance_frame_actual_stand = {
        0.31, -0.12, 0.44, 0.0, 0.0, 1.5707963267948966,
    };
    sample.left_force_control.measured_force_n = 6.75;
    sample.left_force_control.fast_normal_force_n = 7.25;
    sample.left_force_control.fast_force_norm_n = 8.5;
    sample.left_force_control.fast_torque_norm_nm = 0.75;
    sample.left_force_control.contact_threshold_exceeded = true;
    sample.left_force_control.hard_limit_exceeded = false;
    sample.left_force_control.target_force_n = 2.0;
    sample.left_force_control.compliance_active = true;
    sample.left_force_control.normal_contact_active = false;
    sample.left_force_control.transverse_contact_active = true;
    sample.left_force_control.rotational_contact_active = true;
    sample.left_force_control.loading_projection_active = true;
    sample.left_force_control.control_wrench_surface.fx = 1.25;
    sample.left_force_control.control_wrench_compliance.fy = 2.5;
    sample.left_force_control.wrench_error_compliance.tz = 0.4;
    sample.left_force_control.compliance_offset_surface.ry = 0.02;
    sample.left_force_control.accepted_policy_delta_surface = {
        0.001, 0.002, 0.003, 0.01, 0.02, 0.03,
    };
    sample.left_force_control.compliance_equilibrium_stand = {
        0.41, -0.22, 0.33, 0.1, -0.2, 0.3,
    };
    sample.left_force_control.compliance_equilibrium_source = "policy_target";
    sample.left_force_control.compliance_recenter_active = true;
    sample.left_force_control.compliance_translation_recenter_coupled = true;
    sample.left_force_control.compliance_rotation_recenter_coupled = true;
    sample.left_force_control.compliance_translation_recenter_deferred = false;
    sample.left_force_control.compliance_rotation_recenter_deferred = true;
    sample.left_force_control.compliance_limit_axes = {
        true, false, false, false, true, false,
    };
    sample.left_force_control.compliance_limit_reason =
        "jerk_limited_motion_envelope";
    sample.left_force_control.motion_epoch = 9;
    sample.left_last_read.read_exchange_timing.available = true;
    sample.left_last_read.read_exchange_timing.exchange_sequence = 91;
    sample.left_last_read.read_exchange_timing.source = "rbpodo_sdk_request_data";
    sample.left_last_read.read_exchange_timing.request_data_call_start_steady_ns = 1'000'000;
    sample.left_last_read.read_exchange_timing.request_data_call_start_system_ns = 2'000'000;
    sample.left_last_read.read_exchange_timing.request_data_call_return_steady_ns = 1'010'000;
    sample.left_last_read.read_exchange_timing.request_data_call_return_system_ns = 2'010'000;
    sample.left_last_read.read_exchange_timing.request_data_call_duration_us = 10.0;
    logger.push(sample);

    const std::filesystem::path latest = std::filesystem::path(cfg.directory) / "servo_log.csv";
    std::string header_line;
    std::string row_line;
    const bool wrote_row = waitUntil([&] {
        std::ifstream in(latest);
        if (!in) return false;
        if (!std::getline(in, header_line)) return false;
        return static_cast<bool>(std::getline(in, row_line));
    }, std::chrono::milliseconds(1000));
    logger.stop();
    RB_CHECK(wrote_row);

    const std::vector<std::string> header = splitCommaSeparated(header_line);
    const std::vector<std::string> row = splitCommaSeparated(row_line);
    RB_CHECK(header.size() == row.size());
    const auto index_of = [&](const std::string& name) {
        const auto it = std::find(header.begin(), header.end(), name);
        return it == header.end()
            ? static_cast<std::size_t>(header.size())
            : static_cast<std::size_t>(std::distance(header.begin(), it));
    };
    const std::size_t old_last = index_of("right_q_actual_jerk_deg_s3_5");
    const std::size_t reqdata_available = index_of("left_reqdata_timing_available");
    const std::size_t reqdata_sequence = index_of("left_reqdata_exchange_sequence");
    const std::size_t reqdata_source = index_of("left_reqdata_timing_source");
    const std::size_t reqdata_start_system = index_of("left_reqdata_call_start_system_ns");
    const std::size_t reqdata_return_system = index_of("left_reqdata_call_return_system_ns");
    const std::size_t reqdata_duration = index_of("left_reqdata_call_duration_us");
    const std::size_t chunk_wire_seq = index_of("chunk_frame_wire_seq");
    const std::size_t chunk_inference_latency = index_of("chunk_inference_latency_ms");
    const std::size_t chunk_camera_focus = index_of("chunk_camera_right_focus_score");
    const std::size_t delta_frame_rows = index_of("left_delta_twist_frame_rows");
    const std::size_t delta_clamp_mask = index_of("left_delta_twist_clamp_mask");
    const std::size_t delta_accel_rz = index_of("left_delta_twist_accel_cmd_rz_rad_s2");
    const std::size_t branch_rate = index_of("left_cart_branch_jump_rate_limited");
    const std::size_t raw_jump = index_of("left_cart_branch_jump_raw_deg");
    const std::size_t q_seed = index_of("left_q_ik_seed_deg_0");
    const std::size_t clamp_present = index_of("left_safety_clamp_present");
    const std::size_t velocity_clamped = index_of("left_safety_velocity_clamped");
    const std::size_t accel_joint = index_of("left_safety_accel_limited_joint");
    const std::size_t profile_name = index_of("left_tcp_target_profile");
    const std::size_t profile_nf_linear = index_of("left_smd_profile_nf_linear_hz");
    const std::size_t profile_velocity_ff = index_of("left_smd_profile_velocity_feedforward");
    const std::size_t profile_max_angular_accel = index_of("left_smd_profile_max_angular_accel_rad_s2");
    const std::size_t init_left_status = index_of("init_motion_left_status");
    const std::size_t init_right_status = index_of("init_motion_right_status");
    const std::size_t before_left = index_of("left_mode_before_init_sequencer");
    const std::size_t after_right = index_of("right_mode_after_init_sequencer");
    const std::size_t before_left_profile = index_of("left_joint_target_profile_before_init_sequencer");
    const std::size_t after_left_profile = index_of("left_joint_target_profile_after_init_sequencer");
    const std::size_t preserved_mode = index_of("non_init_arm_preserved_mode");
    const std::size_t freeze_other = index_of("single_arm_freeze_other_arm");
    const std::size_t init_left_thresh = index_of("init_motion_left_clear_threshold_m");
    const std::size_t init_left_pair = index_of("init_motion_left_nearest_pair");
    const std::size_t init_left_pair_dist = index_of("init_motion_left_nearest_pair_distance_m");
    const std::size_t init_left_goal_pair_a = index_of("init_motion_left_goal_nearest_pair_a");
    const std::size_t init_left_goal_category = index_of("init_motion_left_goal_pair_category");
    const std::size_t init_left_goal_deficit = index_of("init_motion_left_goal_margin_deficit_m");
    const std::size_t ft_raw_fz = index_of("left_ft_raw_sensor_fz_n");
    const std::size_t ft_transform_z = index_of("left_ft_t_tcp_sensor_z_m");
    const std::size_t ft_transform_rz = index_of("left_ft_t_tcp_sensor_rz_rad");
    const std::size_t ft_control_fz = index_of("left_ft_control_external_fz_n");
    const std::size_t ft_healthy = index_of("left_ft_healthy");
    const std::size_t ft_freshness = index_of("left_ft_freshness_value");
    const std::size_t ft_tare_valid = index_of("left_ft_tare_valid");
    const std::size_t ft_tare_state = index_of("left_ft_tare_state");
    const std::size_t ft_tare_generation = index_of("left_ft_tare_generation");
    const std::size_t ft_tare_fz = index_of("left_ft_residual_tare_tcp_fz_n");
    const std::size_t ft_gravity_x = index_of("left_ft_gravity_tcp_x_m_s2");
    const std::size_t ft_gravity_y = index_of("left_ft_gravity_tcp_y_m_s2");
    const std::size_t ft_gravity_z = index_of("left_ft_gravity_tcp_z_m_s2");
    const std::size_t ft_payload_inhibit =
        index_of("left_ft_payload_identification_inhibit");
    const std::size_t ft_joint_target_profile =
        index_of("left_ft_joint_target_profile");
    const std::size_t force_mode = index_of("left_force_control_operating_mode");
    const std::size_t force_state = index_of("left_force_control_state");
    const std::size_t compliance_frame =
        index_of("left_force_control_compliance_frame");
    const std::size_t compliance_frame_valid =
        index_of("left_force_control_compliance_frame_pose_valid");
    const std::size_t compliance_frame_x =
        index_of("left_force_control_compliance_frame_actual_stand_x_m");
    const std::size_t compliance_frame_rz =
        index_of("left_force_control_compliance_frame_actual_stand_rz_rad");
    const std::size_t compliance_active = index_of("left_force_control_compliance_active");
    const std::size_t normal_contact =
        index_of("left_force_control_normal_contact_active");
    const std::size_t transverse_contact =
        index_of("left_force_control_transverse_contact_active");
    const std::size_t rotational_contact =
        index_of("left_force_control_rotational_contact_active");
    const std::size_t loading_projection = index_of("left_force_control_loading_projection_active");
    const std::size_t compliance_wrench_fx =
        index_of("left_force_control_control_wrench_surface_fx_n");
    const std::size_t compliance_frame_wrench_fy =
        index_of("left_force_control_control_wrench_compliance_fy_n");
    const std::size_t compliance_frame_error_tz =
        index_of("left_force_control_wrench_error_compliance_tz_nm");
    const std::size_t compliance_offset_ry =
        index_of("left_force_control_compliance_offset_surface_dry_rad");
    const std::size_t accepted_policy_rz =
        index_of("left_force_control_accepted_policy_delta_surface_drz_rad");
    const std::size_t equilibrium_x =
        index_of("left_force_control_compliance_equilibrium_stand_x_m");
    const std::size_t equilibrium_rz =
        index_of("left_force_control_compliance_equilibrium_stand_rz_rad");
    const std::size_t equilibrium_source =
        index_of("left_force_control_compliance_equilibrium_source");
    const std::size_t recenter_active =
        index_of("left_force_control_compliance_recenter_active");
    const std::size_t translation_recenter_coupled =
        index_of("left_force_control_compliance_translation_recenter_coupled");
    const std::size_t rotation_recenter_coupled =
        index_of("left_force_control_compliance_rotation_recenter_coupled");
    const std::size_t translation_recenter_deferred =
        index_of("left_force_control_compliance_translation_recenter_deferred");
    const std::size_t rotation_recenter_deferred =
        index_of("left_force_control_compliance_rotation_recenter_deferred");
    const std::size_t limit_axis_x = index_of("left_force_control_limit_axis_x");
    const std::size_t limit_axis_pitch =
        index_of("left_force_control_limit_axis_pitch");
    const std::size_t limit_reason = index_of("left_force_control_limit_reason");
    const std::size_t measured_force = index_of("left_force_control_measured_normal_force_n");
    const std::size_t fast_normal_force = index_of("left_force_control_fast_normal_force_n");
    const std::size_t fast_force_norm = index_of("left_force_control_fast_force_norm_n");
    const std::size_t fast_torque_norm = index_of("left_force_control_fast_torque_norm_nm");
    const std::size_t contact_threshold = index_of("left_force_control_contact_threshold_exceeded");
    const std::size_t hard_threshold = index_of("left_force_control_hard_limit_threshold_exceeded");
    const std::size_t hard_count = index_of("left_force_control_hard_limit_sample_count");
    const std::size_t hard_limit = index_of("left_force_control_hard_limit_exceeded");
    const std::size_t motion_epoch = index_of("left_force_control_motion_epoch");
    RB_CHECK(old_last < header.size());
    RB_CHECK(reqdata_available < header.size());
    RB_CHECK(reqdata_sequence < header.size());
    RB_CHECK(reqdata_source < header.size());
    RB_CHECK(reqdata_start_system < header.size());
    RB_CHECK(reqdata_return_system < header.size());
    RB_CHECK(reqdata_duration < header.size());
    RB_CHECK(chunk_wire_seq < header.size());
    RB_CHECK(chunk_inference_latency < header.size());
    RB_CHECK(chunk_camera_focus < header.size());
    RB_CHECK(delta_frame_rows < old_last);
    RB_CHECK(delta_clamp_mask < old_last);
    RB_CHECK(delta_accel_rz < old_last);
    RB_CHECK(profile_name < old_last);
    RB_CHECK(profile_nf_linear < old_last);
    RB_CHECK(profile_velocity_ff < old_last);
    RB_CHECK(profile_max_angular_accel < old_last);
    RB_CHECK(branch_rate > old_last);
    RB_CHECK(raw_jump > old_last);
    RB_CHECK(q_seed > old_last);
    RB_CHECK(clamp_present > old_last);
    RB_CHECK(velocity_clamped > old_last);
    RB_CHECK(accel_joint > old_last);
    RB_CHECK(init_left_status > old_last);
    RB_CHECK(init_right_status > old_last);
    RB_CHECK(before_left > old_last);
    RB_CHECK(after_right > old_last);
    RB_CHECK(before_left_profile > old_last);
    RB_CHECK(after_left_profile > old_last);
    RB_CHECK(preserved_mode > old_last);
    RB_CHECK(freeze_other > old_last);
    RB_CHECK(init_left_thresh > old_last);
    RB_CHECK(init_left_pair > old_last);
    RB_CHECK(init_left_pair_dist > old_last);
    RB_CHECK(init_left_goal_pair_a > old_last);
    RB_CHECK(init_left_goal_category > old_last);
    RB_CHECK(init_left_goal_deficit > old_last);
    RB_CHECK(ft_raw_fz > init_left_goal_deficit);
    RB_CHECK(ft_transform_z > ft_raw_fz);
    RB_CHECK(ft_transform_rz > ft_transform_z);
    RB_CHECK(ft_control_fz > init_left_goal_deficit);
    RB_CHECK(ft_healthy > init_left_goal_deficit);
    RB_CHECK(ft_freshness > init_left_goal_deficit);
    RB_CHECK(ft_tare_valid > init_left_goal_deficit);
    RB_CHECK(ft_tare_state > init_left_goal_deficit);
    RB_CHECK(ft_tare_generation > init_left_goal_deficit);
    RB_CHECK(ft_tare_fz > init_left_goal_deficit);
    RB_CHECK(ft_gravity_x > init_left_goal_deficit);
    RB_CHECK(ft_gravity_y > ft_gravity_x);
    RB_CHECK(ft_gravity_z > ft_gravity_y);
    RB_CHECK(ft_payload_inhibit > ft_gravity_z);
    RB_CHECK(ft_joint_target_profile > ft_payload_inhibit);
    RB_CHECK(force_mode > init_left_goal_deficit);
    RB_CHECK(force_state > init_left_goal_deficit);
    RB_CHECK(compliance_frame > force_state);
    RB_CHECK(compliance_frame_valid > compliance_frame);
    RB_CHECK(compliance_frame_x > compliance_frame_valid);
    RB_CHECK(compliance_frame_rz > compliance_frame_x);
    RB_CHECK(compliance_active > force_state);
    RB_CHECK(normal_contact > force_state);
    RB_CHECK(transverse_contact > normal_contact);
    RB_CHECK(rotational_contact > transverse_contact);
    RB_CHECK(loading_projection > compliance_active);
    RB_CHECK(compliance_wrench_fx > loading_projection);
    RB_CHECK(compliance_frame_wrench_fy > compliance_wrench_fx);
    RB_CHECK(compliance_frame_error_tz > compliance_frame_wrench_fy);
    RB_CHECK(compliance_offset_ry > compliance_wrench_fx);
    RB_CHECK(accepted_policy_rz > compliance_offset_ry);
    RB_CHECK(equilibrium_x > accepted_policy_rz);
    RB_CHECK(equilibrium_rz > equilibrium_x);
    RB_CHECK(equilibrium_source > equilibrium_rz);
    RB_CHECK(recenter_active > equilibrium_source);
    RB_CHECK(limit_axis_x > recenter_active);
    RB_CHECK(limit_axis_pitch > limit_axis_x);
    RB_CHECK(limit_reason > limit_axis_pitch);
    RB_CHECK(measured_force > init_left_goal_deficit);
    RB_CHECK(fast_normal_force > measured_force);
    RB_CHECK(fast_force_norm > fast_normal_force);
    RB_CHECK(fast_torque_norm > fast_force_norm);
    RB_CHECK(contact_threshold > fast_torque_norm);
    RB_CHECK(hard_threshold > contact_threshold);
    RB_CHECK(hard_count > hard_threshold);
    RB_CHECK(hard_limit > hard_count);
    RB_CHECK(motion_epoch > init_left_goal_deficit);
    RB_CHECK(row.at(branch_rate) == "1");
    RB_CHECK(row.at(reqdata_available) == "1");
    RB_CHECK(row.at(reqdata_sequence) == "91");
    RB_CHECK(row.at(reqdata_source) == "rbpodo_sdk_request_data");
    RB_CHECK(row.at(reqdata_start_system) == "2000000");
    RB_CHECK(row.at(reqdata_return_system) == "2010000");
    RB_CHECK(row.at(reqdata_duration) == "10");
    RB_CHECK(row.at(chunk_wire_seq) == "70");
    RB_CHECK(row.at(chunk_inference_latency) == "77.5");
    RB_CHECK(row.at(chunk_camera_focus) == "45.5");
    RB_CHECK(row.at(delta_frame_rows) == "16");
    RB_CHECK(row.at(delta_clamp_mask) == "65");
    RB_CHECK(row.at(delta_accel_rz) == "6");
    RB_CHECK(row.at(raw_jump) == "12");
    RB_CHECK(row.at(q_seed) == "1");
    RB_CHECK(row.at(clamp_present) == "1");
    RB_CHECK(row.at(velocity_clamped) == "1");
    RB_CHECK(row.at(accel_joint) == "1");
    RB_CHECK(row.at(profile_name) == "umi_large_smooth");
    RB_CHECK(row.at(profile_nf_linear) == "1");
    RB_CHECK(row.at(profile_velocity_ff) == "1");
    RB_CHECK(row.at(profile_max_angular_accel) == "5");
    RB_CHECK(row.at(init_left_status) == "executing");
    RB_CHECK(row.at(init_right_status) == "idle");
    RB_CHECK(row.at(before_left) == "JointTarget");
    RB_CHECK(row.at(after_right) == "TcpPoseTarget");
    RB_CHECK(row.at(before_left_profile) == "init_motion");
    RB_CHECK(row.at(after_left_profile) == "direct");
    RB_CHECK(row.at(preserved_mode) == "TcpPoseTarget");
    RB_CHECK(row.at(freeze_other) == "0");
    RB_CHECK(row.at(init_left_thresh) == "0.023");
    RB_CHECK(row.at(init_left_pair) == "left_link <-> stand");
    RB_CHECK(row.at(init_left_pair_dist) == "0.018");
    RB_CHECK(row.at(init_left_goal_pair_a) == "left_link0");
    RB_CHECK(row.at(init_left_goal_category) == "intra-arm");
    RB_CHECK(row.at(init_left_goal_deficit) == "0.0042");
    RB_CHECK(row.at(ft_raw_fz) == "-22.5");
    RB_CHECK(row.at(ft_transform_z) == "-0.202642");
    RB_CHECK(row.at(ft_transform_rz) == "1.5708");
    RB_CHECK(row.at(ft_control_fz) == "7.25");
    RB_CHECK(row.at(ft_healthy) == "1");
    RB_CHECK(row.at(ft_freshness) == "1234");
    RB_CHECK(row.at(ft_tare_valid) == "1");
    RB_CHECK(row.at(ft_tare_state) == "accepted");
    RB_CHECK(row.at(ft_tare_generation) == "3");
    RB_CHECK(row.at(ft_tare_fz) == "23.5");
    RB_CHECK(row.at(ft_gravity_x) == "1.25");
    RB_CHECK(row.at(ft_gravity_y) == "-2.5");
    RB_CHECK(row.at(ft_gravity_z) == "-9");
    RB_CHECK(row.at(ft_payload_inhibit) == "1");
    RB_CHECK(row.at(ft_joint_target_profile) == "payload_identification");
    RB_CHECK(row.at(compliance_frame) == "sensor_origin");
    RB_CHECK(row.at(compliance_frame_valid) == "1");
    RB_CHECK(row.at(compliance_frame_x) == "0.31");
    RB_CHECK(row.at(compliance_frame_rz) == "1.5708");
    RB_CHECK(row.at(compliance_frame_wrench_fy) == "2.5");
    RB_CHECK(row.at(compliance_frame_error_tz) == "0.4");
    RB_CHECK(row.at(force_mode) == "monitor");
    RB_CHECK(row.at(force_state) == "release_braking");
    RB_CHECK(row.at(compliance_active) == "1");
    RB_CHECK(row.at(normal_contact) == "0");
    RB_CHECK(row.at(transverse_contact) == "1");
    RB_CHECK(row.at(rotational_contact) == "1");
    RB_CHECK(row.at(loading_projection) == "1");
    RB_CHECK(row.at(compliance_wrench_fx) == "1.25");
    RB_CHECK(row.at(compliance_offset_ry) == "0.02");
    RB_CHECK(row.at(accepted_policy_rz) == "0.03");
    RB_CHECK(row.at(equilibrium_x) == "0.41");
    RB_CHECK(row.at(equilibrium_rz) == "0.3");
    RB_CHECK(row.at(equilibrium_source) == "policy_target");
    RB_CHECK(row.at(recenter_active) == "1");
    RB_CHECK(row.at(translation_recenter_coupled) == "1");
    RB_CHECK(row.at(rotation_recenter_coupled) == "1");
    RB_CHECK(row.at(translation_recenter_deferred) == "0");
    RB_CHECK(row.at(rotation_recenter_deferred) == "1");
    RB_CHECK(row.at(limit_axis_x) == "1");
    RB_CHECK(row.at(limit_axis_pitch) == "1");
    RB_CHECK(row.at(limit_reason) == "jerk_limited_motion_envelope");
    RB_CHECK(row.at(measured_force) == "6.75");
    RB_CHECK(row.at(fast_normal_force) == "7.25");
    RB_CHECK(row.at(fast_force_norm) == "8.5");
    RB_CHECK(row.at(fast_torque_norm) == "0.75");
    RB_CHECK(row.at(contact_threshold) == "1");
    RB_CHECK(row.at(hard_threshold) == "0");
    RB_CHECK(row.at(hard_count) == "0");
    RB_CHECK(row.at(hard_limit) == "0");
    RB_CHECK(row.at(motion_epoch) == "9");
    std::filesystem::remove_all(cfg.directory);
    return true;
}

bool testPayloadIdentificationProfileLatchesForceMotionInhibit() {
    rb_servo::DualArmConfig cfg = testConfig();
    configureCartesianLoopTest(&cfg);
    const std::filesystem::path repo_root =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const std::filesystem::path unified_urdf_dir =
        repo_root.parent_path() /
        "mo_robot_descriptions/mo_robot_descriptions/robots/urdf/dual_rb3_730e";
    cfg.safety.self_collision.enable = true;
    cfg.safety.self_collision.monitor_only = true;
    cfg.safety.self_collision.mesh.unified_urdf =
        (unified_urdf_dir / "dual_rb3_730e_ver5.urdf").string();
    cfg.safety.self_collision.mesh.package_dirs = {unified_urdf_dir.string()};
    cfg.safety.init_motion_planner.enable = true;
    cfg.safety.init_motion_planner.noop_tol_deg = 1.5;
    cfg.safety.init_motion_planner.waypoint_tol_deg = 1.5;
    cfg.servo.send_at_tick_start = false;
    cfg.servo.command_timeout_sec = 0.03;
    cfg.safety.floor_constraint.enable = true;
    cfg.safety.floor_constraint.z_min_m = -1.0;
    cfg.force_torque.source = "rbpodo_eft";
    cfg.force_torque.payload_identification.enable = true;
    cfg.force_torque.left.enable = true;
    cfg.force_torque.left.frame_configured = true;
    cfg.force_torque.left.freshness_source = "sequence";
    cfg.force_torque.left.control_lpf_alpha = 1.0;
    cfg.force_torque.right.enable = true;
    cfg.force_torque.right.frame_configured = true;
    cfg.force_torque.right.auto_tare_after_init_motion = true;
    cfg.force_torque.right.auto_tare_settle_sec = 0.001;
    cfg.force_torque.right.residual_tare_min_samples = 3;
    cfg.force_torque.right.residual_tare_max_force_stddev_n = 0.1;
    cfg.force_torque.right.residual_tare_max_torque_stddev_nm = 0.01;
    cfg.force_control.provider = "project_native";
    cfg.force_control.enable = true;
    cfg.force_control.operating_mode = "cartesian_admittance";
    cfg.force_control.update_rate_hz = cfg.servo.rate_hz;
    cfg.force_control.left.enable = true;
    cfg.force_control.left.contact_enter_force_n = 4.0;
    cfg.force_control.left.contact_release_force_n = 2.0;
    cfg.force_control.left.hard_normal_force_n = 100.0;
    cfg.force_control.left.hard_force_norm_n = 200.0;
    cfg.force_control.left.hard_torque_norm_nm = 100.0;
    cfg.force_control.left.debounce_samples = 1;
    cfg.force_control.left.transverse_contact_enter_force_n = 4.0;
    cfg.force_control.left.transverse_contact_release_force_n = 2.0;
    cfg.force_control.left.torque_contact_enter_nm = 0.5;
    cfg.force_control.left.torque_contact_release_nm = 0.2;
    cfg.force_control.left.compliance_axes = {true, true, true, true, true, true};
    cfg.force_control.virtual_mass = {1.0, 1.0, 1.0, 0.5, 0.5, 0.5};
    cfg.force_control.damping = {20.0, 20.0, 20.0, 5.0, 5.0, 5.0};
    cfg.force_control.stiffness = {100.0, 100.0, 100.0, 20.0, 20.0, 20.0};
    cfg.force_control.wrench_deadband = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    cfg.force_control.max_energy_j = 100.0;

    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    left->setEftWrench(rb_servo::Wrench6D{8.0, -7.0, 6.0, 1.0, -1.0, 1.0});
    right->setEftWrench(rb_servo::Wrench6D{});
    rb_servo::CommandBuffer buffer;
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::move(left), std::move(right), cfg, &buffer, nullptr, kinematics);

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] {
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));
    RB_CHECK(waitUntil([&] {
        return loop.latestSnapshot().left_force_control.proposal_committed;
    }, std::chrono::milliseconds(2000)));
    RB_CHECK(waitUntil([&] {
        const auto snapshot = loop.latestSnapshot();
        return snapshot.right_force_torque.healthy &&
            !snapshot.right_force_torque.stale;
    }));

    const auto rejected_as_two_arm_hold = [
        &buffer,
        &loop
    ](const rb_servo::DualArmCommand& malformed) {
        buffer.setCommand(malformed);
        rb_servo::ServoSnapshot rejected;
        const bool observed = waitUntil([&] {
            rejected = loop.latestSnapshot();
            return rejected.command.seq == malformed.seq &&
                rejected.safety_verdict == rb_servo::SafetyVerdict::InvalidCommand;
        });
        const bool held = observed &&
            rejected.command.left.mode == rb_servo::ControlMode::Hold &&
            rejected.command.right.mode == rb_servo::ControlMode::Hold &&
            !rejected.command.left.has_joint_target &&
            !rejected.command.right.has_joint_target &&
            !rejected.left_force_torque.payload_identification_inhibit &&
            !rejected.right_force_torque.payload_identification_inhibit &&
            !rejected.fault_latched;
        if (!held) {
            std::cerr
                << "payload-identification malformed rejection mismatch: observed="
                << observed << " seq=" << rejected.command.seq
                << " verdict=" << rb_servo::toString(rejected.safety_verdict)
                << " modes=" << rb_servo::toString(rejected.command.left.mode) << "/"
                << rb_servo::toString(rejected.command.right.mode)
                << " targets=" << rejected.command.left.has_joint_target << "/"
                << rejected.command.right.has_joint_target
                << " inhibits="
                << rejected.left_force_torque.payload_identification_inhibit << "/"
                << rejected.right_force_torque.payload_identification_inhibit
                << " fault=" << rejected.fault_latched << "\n";
        }
        return held;
    };

    // The server, not only rb_gui, owns the exclusive one-arm packet shape.
    // Two calibration profiles must not latch either arm or move either target.
    rb_servo::DualArmCommand both_identify = command(rb_servo::ControlMode::Hold);
    both_identify.seq = 42;
    both_identify.host_time_ns = rb_servo::nowSteadyNs();
    for (rb_servo::ArmCommand* arm : {&both_identify.left, &both_identify.right}) {
        arm->mode = rb_servo::ControlMode::JointTarget;
        arm->has_joint_target = true;
        arm->q_target_deg = initial;
        arm->joint_target_profile =
            rb_servo::JointTargetProfile::PayloadIdentification;
    }
    RB_CHECK(rejected_as_two_arm_hold(both_identify));

    // A valid profile combined with any peer motion is likewise an atomic
    // rejection; the peer may only be a payload-free Hold.
    rb_servo::DualArmCommand identify_with_peer_motion =
        command(rb_servo::ControlMode::Hold);
    identify_with_peer_motion.seq = 43;
    identify_with_peer_motion.host_time_ns = rb_servo::nowSteadyNs();
    identify_with_peer_motion.left.mode = rb_servo::ControlMode::JointTarget;
    identify_with_peer_motion.left.has_joint_target = true;
    identify_with_peer_motion.left.q_target_deg = joints(0.5);
    identify_with_peer_motion.right.mode = rb_servo::ControlMode::JointTarget;
    identify_with_peer_motion.right.has_joint_target = true;
    identify_with_peer_motion.right.q_target_deg = initial;
    identify_with_peer_motion.right.joint_target_profile =
        rb_servo::JointTargetProfile::PayloadIdentification;
    RB_CHECK(rejected_as_two_arm_hold(identify_with_peer_motion));

    rb_servo::DualArmCommand identify = command(rb_servo::ControlMode::Hold);
    identify.seq = 44;
    identify.host_time_ns = rb_servo::nowSteadyNs();
    identify.right.mode = rb_servo::ControlMode::JointTarget;
    identify.right.has_joint_target = true;
    identify.right.q_target_deg = initial;
    identify.right.joint_target_profile =
        rb_servo::JointTargetProfile::PayloadIdentification;
    identify.left.timeout_sec = 0.03;
    identify.right.timeout_sec = 0.03;
    buffer.setCommand(identify);

    rb_servo::ServoSnapshot snapshot;
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.command.seq == identify.seq &&
            snapshot.right_force_torque.payload_identification_inhibit &&
            snapshot.right_force_torque.joint_target_profile ==
                "payload_identification";
    }));
    RB_CHECK(!snapshot.right_force_torque.tare_valid);
    RB_CHECK(snapshot.right_force_control.state ==
        "payload_identification_inhibited");
    RB_CHECK(!snapshot.right_force_control.proposal_committed);
    RB_CHECK(!snapshot.left_force_control.proposal_committed);

    // A stale/expired acquisition packet becoming Hold is not a release event.
    // Only an accepted later InitMotion tare may clear this latch.
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.command.right.mode == rb_servo::ControlMode::Hold &&
            snapshot.right_force_torque.payload_identification_inhibit;
    }, std::chrono::milliseconds(500)));
    RB_CHECK(!snapshot.fault_latched);

    rb_servo::DualArmCommand retare = command(rb_servo::ControlMode::Hold);
    retare.seq = 45;
    retare.host_time_ns = rb_servo::nowSteadyNs();
    retare.right.mode = rb_servo::ControlMode::JointTarget;
    retare.right.has_joint_target = true;
    retare.right.q_target_deg = initial;
    retare.right.joint_target_profile = rb_servo::JointTargetProfile::InitMotion;
    retare.left.timeout_sec = 0.5;
    retare.right.timeout_sec = 0.5;
    buffer.setCommand(retare);
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.right_force_torque.tare_valid &&
            snapshot.right_force_torque.tare_state == "accepted" &&
            !snapshot.right_force_torque.payload_identification_inhibit;
    }, std::chrono::milliseconds(1000)));
    RB_CHECK(!snapshot.fault_latched);
    loop.stop();

    // Admission failure must also be a pure two-arm joint hold for that packet;
    // it must not fall through to either arm's Cartesian force-Hold promotion.
    cfg.force_torque.payload_identification.enable = false;
    auto rejected_left = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left, initial, false);
    auto rejected_right = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right, initial, false);
    rejected_left->setEftWrench(
        rb_servo::Wrench6D{8.0, -7.0, 6.0, 1.0, -1.0, 1.0});
    rb_servo::CommandBuffer rejected_buffer;
    rb_servo::DualArmServoLoop rejected_loop(
        std::move(rejected_left),
        std::move(rejected_right),
        cfg,
        &rejected_buffer,
        nullptr,
        std::make_shared<FakeCartesianKinematics>()
    );
    RB_CHECK(rejected_loop.start());
    rejected_buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] {
        return rejected_loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));
    RB_CHECK(waitUntil([&] {
        return rejected_loop.latestSnapshot().left_force_control.proposal_committed;
    }, std::chrono::milliseconds(2000)));
    identify.seq = 45;
    identify.host_time_ns = rb_servo::nowSteadyNs();
    rejected_buffer.setCommand(identify);
    RB_CHECK(waitUntil([&] {
        snapshot = rejected_loop.latestSnapshot();
        return snapshot.command.seq == identify.seq &&
            snapshot.safety_verdict == rb_servo::SafetyVerdict::InvalidCommand;
    }));
    RB_CHECK(!snapshot.right_force_torque.payload_identification_inhibit);
    RB_CHECK(!snapshot.left_force_control.proposal_committed);
    RB_CHECK(!snapshot.right_force_control.proposal_committed);
    RB_CHECK(!snapshot.fault_latched);
    rejected_loop.stop();
    return true;
}

bool testPayloadIdentificationRetainsRawHardLimitGuard() {
    rb_servo::DualArmConfig cfg = testConfig();
    configureCartesianLoopTest(&cfg);
    cfg.force_torque.source = "rbpodo_eft";
    cfg.force_torque.payload_identification.enable = true;
    cfg.force_torque.right.enable = true;
    cfg.force_torque.right.frame_configured = true;
    cfg.force_torque.right.auto_tare_after_init_motion = true;
    cfg.force_control.provider = "project_native";
    cfg.force_control.enable = true;
    cfg.force_control.operating_mode = "cartesian_admittance";
    cfg.force_control.right.enable = true;
    cfg.force_control.right.surface_source = "none";
    cfg.force_control.right.hard_normal_force_n = 5.0;
    cfg.force_control.right.hard_force_norm_n = 5.0;
    cfg.force_control.right.hard_torque_norm_nm = 5.0;
    cfg.force_control.right.hard_limit_debounce_samples = 2;

    rb_servo::Wrench6D hard_wrench;
    hard_wrench.fz = -10.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    right->setEftWrench(hard_wrench);
    rb_servo::CommandBuffer buffer;
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::move(left), std::move(right), cfg, &buffer, nullptr, kinematics);

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] {
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));
    RB_CHECK(waitUntil([&] {
        return loop.latestSnapshot().right_force_torque.healthy;
    }));

    rb_servo::DualArmCommand identify = command(rb_servo::ControlMode::Hold);
    identify.seq = 46;
    identify.host_time_ns = rb_servo::nowSteadyNs();
    identify.right.mode = rb_servo::ControlMode::JointTarget;
    identify.right.has_joint_target = true;
    identify.right.q_target_deg = initial;
    identify.right.joint_target_profile =
        rb_servo::JointTargetProfile::PayloadIdentification;
    buffer.setCommand(identify);

    rb_servo::ServoSnapshot snapshot;
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.fault_latched;
    }, std::chrono::milliseconds(1000)));
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::ExternalForceLimit ||
        snapshot.safety_verdict == rb_servo::SafetyVerdict::FaultLatched);
    RB_CHECK(snapshot.right_force_torque.payload_identification_inhibit);
    RB_CHECK(snapshot.right_force_control.hard_limit_threshold_exceeded);
    RB_CHECK(snapshot.right_force_control.hard_limit_exceeded);
    RB_CHECK(snapshot.right_force_control.fast_force_norm_n >= 10.0);
    loop.stop();
    return true;
}

bool testForceRuntimeUsesTimestampAfterBackendStateRead() {
    rb_servo::DualArmConfig cfg = testConfig();
    configureCartesianLoopTest(&cfg);
    cfg.force_torque.source = "rbpodo_eft";
    cfg.force_torque.left.enable = true;
    cfg.force_torque.left.frame_configured = true;
    cfg.force_torque.right.enable = true;
    cfg.force_torque.right.frame_configured = true;
    cfg.force_control.provider = "project_native";
    cfg.force_control.enable = true;
    cfg.force_control.operating_mode = "monitor";
    cfg.force_control.left.enable = true;
    cfg.force_control.right.enable = true;
    cfg.force_control.left.contact_enter_force_n = 2.0;
    cfg.force_control.right.contact_enter_force_n = 2.0;
    cfg.force_control.left.hard_normal_force_n = 10.0;
    cfg.force_control.right.hard_normal_force_n = 10.0;
    cfg.force_control.left.hard_force_norm_n = 20.0;
    cfg.force_control.right.hard_force_norm_n = 20.0;
    cfg.force_control.left.hard_torque_norm_nm = 10.0;
    cfg.force_control.right.hard_torque_norm_nm = 10.0;

    rb_servo::Wrench6D eft;
    eft.fx = 1.0;
    eft.fy = -2.0;
    // With an outward +Z floor normal, a -Z sensor reaction is positive
    // compression and must cross the monitor contact threshold.
    eft.fz = -3.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    left->setEftWrench(eft);
    right->setEftWrench(eft);
    // Real rbpodo firmware may report robot_time_ns=0. A fresh backend frame
    // must still qualify EFT freshness through acquisition_sequence.
    left->setAdvanceRobotTimeOnRead(false);
    right->setAdvanceRobotTimeOnRead(false);

    rb_servo::CommandBuffer buffer;
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    rb_servo::ServoSnapshot snapshot;
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.left_force_torque.healthy && snapshot.right_force_torque.healthy &&
            snapshot.left_force_control.state == "monitoring" &&
            snapshot.right_force_control.state == "monitoring";
    }));
    RB_CHECK(!snapshot.left_force_torque.stale);
    RB_CHECK(!snapshot.right_force_torque.stale);
    RB_CHECK(snapshot.left_force_torque.reason == "ok");
    RB_CHECK(snapshot.right_force_torque.reason == "ok");
    RB_CHECK(snapshot.left_force_torque.freshness_value > 0);
    RB_CHECK(snapshot.right_force_torque.freshness_value > 0);
    RB_CHECK(std::abs(snapshot.right_force_torque.wrench_tcp.fz + 3.0) < 1e-12);
    RB_CHECK(std::abs(snapshot.right_force_control.measured_force_n - 3.0) < 1e-12);
    RB_CHECK(std::abs(snapshot.right_force_control.fast_normal_force_n - 3.0) < 1e-12);
    RB_CHECK(std::abs(snapshot.right_force_control.fast_force_norm_n - std::sqrt(14.0)) < 1e-12);
    RB_CHECK(snapshot.right_force_control.contact_threshold_exceeded);
    RB_CHECK(!snapshot.right_force_control.hard_limit_exceeded);
    RB_CHECK(snapshot.left_force_control.state == "monitoring");
    RB_CHECK(snapshot.right_force_control.state == "monitoring");
    RB_CHECK(!snapshot.fault_latched);
    loop.stop();
    return true;
}

bool testForceHardLimitRequiresConsecutiveFreshSamples() {
    rb_servo::DualArmConfig cfg = testConfig();
    configureCartesianLoopTest(&cfg);
    cfg.servo.send_at_tick_start = false;
    cfg.safety.floor_constraint.enable = true;
    cfg.safety.floor_constraint.z_min_m = -1.0;
    cfg.force_torque.source = "rbpodo_eft";
    cfg.force_torque.right.enable = true;
    cfg.force_torque.right.frame_configured = true;
    cfg.force_torque.right.freshness_source = "sequence";
    cfg.force_torque.right.control_lpf_alpha = 1.0;
    cfg.force_control.provider = "project_native";
    cfg.force_control.enable = true;
    cfg.force_control.operating_mode = "guarded_admittance";
    cfg.force_control.update_rate_hz = cfg.servo.rate_hz;
    cfg.force_control.right.enable = true;
    cfg.force_control.right.target_force_n = 2.0;
    cfg.force_control.right.contact_enter_force_n = 4.0;
    cfg.force_control.right.contact_release_force_n = 1.0;
    cfg.force_control.right.force_deadband_n = 0.5;
    cfg.force_control.right.hard_normal_force_n = 5.0;
    cfg.force_control.right.hard_force_norm_n = 5.0;
    cfg.force_control.right.hard_torque_norm_nm = 5.0;
    cfg.force_control.right.debounce_samples = 1;
    cfg.force_control.right.hard_limit_debounce_samples = 50;
    cfg.force_control.right.release_dwell_sec = 0.1;
    cfg.force_control.normal_admittance.max_energy_j = 100.0;

    rb_servo::Wrench6D contact_wrench;
    contact_wrench.fz = -10.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left, initial, false);
    auto right_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right, initial, false);
    right_backend->setEftWrench(contact_wrench);
    rb_servo::CommandBuffer buffer;
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::move(left_backend),
        std::move(right_backend),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    rb_servo::ServoSnapshot snapshot;
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.right_force_control.hard_limit_threshold_exceeded &&
            snapshot.right_force_control.hard_limit_sample_count > 0 &&
            snapshot.right_force_control.hard_limit_sample_count < 50;
    }));
    RB_CHECK(!snapshot.right_force_control.hard_limit_exceeded);
    RB_CHECK(!snapshot.fault_latched);

    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.fault_latched;
    }, std::chrono::milliseconds(1000)));
    RB_CHECK(snapshot.right_force_control.hard_limit_sample_count >= 50);
    RB_CHECK(snapshot.right_force_control.hard_limit_exceeded);
    RB_CHECK(snapshot.fault_reason == "external force/torque hard limit exceeded");
    loop.stop();
    return true;
}

bool testGuardedAdmittanceContinuesAcrossUpstreamHold() {
    rb_servo::DualArmConfig cfg = testConfig();
    configureCartesianLoopTest(&cfg);
    cfg.servo.send_at_tick_start = false;
    cfg.safety.floor_constraint.enable = true;
    cfg.safety.floor_constraint.z_min_m = -1.0;
    cfg.force_torque.source = "rbpodo_eft";
    cfg.force_torque.right.enable = true;
    cfg.force_torque.right.frame_configured = true;
    cfg.force_torque.right.freshness_source = "sequence";
    cfg.force_torque.right.control_lpf_alpha = 1.0;
    cfg.force_control.provider = "project_native";
    cfg.force_control.enable = true;
    cfg.force_control.operating_mode = "guarded_admittance";
    cfg.force_control.update_rate_hz = cfg.servo.rate_hz;
    cfg.force_control.right.enable = true;
    cfg.force_control.right.target_force_n = 2.0;
    cfg.force_control.right.contact_enter_force_n = 6.0;
    cfg.force_control.right.contact_release_force_n = 1.0;
    cfg.force_control.right.force_deadband_n = 0.5;
    cfg.force_control.right.hard_normal_force_n = 100.0;
    cfg.force_control.right.hard_force_norm_n = 100.0;
    cfg.force_control.right.hard_torque_norm_nm = 10.0;
    cfg.force_control.right.debounce_samples = 1;
    cfg.force_control.right.release_dwell_sec = 0.1;
    cfg.force_control.normal_admittance.max_energy_j = 100.0;

    rb_servo::Wrench6D contact_wrench;
    contact_wrench.fz = -10.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left, initial, false);
    auto right_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right, initial, false);
    right_backend->setEftWrench(contact_wrench);
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        std::move(left_backend),
        std::move(right_backend),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] {
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));
    rb_servo::DualArmCommand hold = command(rb_servo::ControlMode::Hold);
    hold.seq = 2;
    hold.host_time_ns = rb_servo::nowSteadyNs();
    buffer.setCommand(hold);

    rb_servo::ServoSnapshot snapshot;
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.command.seq == hold.seq &&
            snapshot.command.right.mode == rb_servo::ControlMode::Hold &&
            snapshot.right_force_control.contact_active &&
            snapshot.right_force_control.proposal_committed &&
            snapshot.right_force_control.correction_m > 0.0 &&
            kinematics->lastRightTarget().has_value() &&
            kinematics->lastRightTarget()->z > 0.0;
    }, std::chrono::milliseconds(1000)));
    RB_CHECK(!snapshot.fault_latched);
    RB_CHECK(snapshot.right_force_control.state == "regulating");
    loop.stop();
    return true;
}

bool testCartesianTransverseComplianceKeepsSoftContactInCurrentMotionEpoch() {
    rb_servo::DualArmConfig cfg = testConfig();
    configureCartesianLoopTest(&cfg);
    cfg.servo.send_at_tick_start = false;
    cfg.servo.command_timeout_sec = 5.0;
    cfg.safety.floor_constraint.enable = true;
    cfg.safety.floor_constraint.z_min_m = -1.0;
    cfg.force_torque.source = "rbpodo_eft";
    cfg.force_torque.right.enable = true;
    cfg.force_torque.right.frame_configured = true;
    cfg.force_torque.right.freshness_source = "sequence";
    cfg.force_torque.right.control_lpf_alpha = 1.0;
    cfg.force_torque.right.t_tcp_sensor = {
        0.0, 0.0, -0.2, 0.0, 0.0, 1.5707963267948966,
    };
    cfg.force_control.provider = "project_native";
    cfg.force_control.enable = true;
    cfg.force_control.operating_mode = "cartesian_admittance";
    cfg.force_control.update_rate_hz = cfg.servo.rate_hz;
    cfg.force_control.right.enable = true;
    cfg.force_control.right.compliance_frame = "sensor_origin";
    cfg.force_control.right.target_force_n = 2.0;
    cfg.force_control.right.contact_enter_force_n = 6.0;
    cfg.force_control.right.contact_release_force_n = 3.0;
    cfg.force_control.right.force_deadband_n = 0.5;
    cfg.force_control.right.hard_normal_force_n = 100.0;
    cfg.force_control.right.hard_force_norm_n = 100.0;
    cfg.force_control.right.hard_torque_norm_nm = 100.0;
    cfg.force_control.right.debounce_samples = 1;
    cfg.force_control.right.release_dwell_sec = 0.02;
    cfg.force_control.right.transverse_contact_enter_force_n = 5.0;
    cfg.force_control.right.transverse_contact_release_force_n = 4.0;
    cfg.force_control.right.torque_contact_enter_nm = 0.9;
    cfg.force_control.right.torque_contact_release_nm = 0.7;
    cfg.force_control.right.compliance_axes = {true, true, true, true, true, true};
    cfg.force_control.virtual_mass = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    cfg.force_control.damping = {20.0, 20.0, 20.0, 5.0, 5.0, 5.0};
    cfg.force_control.stiffness = {30.0, 30.0, 0.0, 5.0, 5.0, 5.0};
    cfg.force_control.wrench_deadband = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    cfg.force_control.max_energy_j = 100.0;
    cfg.force_control.normal_admittance.max_energy_j = 100.0;

    rb_servo::Wrench6D contact_wrench;
    contact_wrench.fx = 8.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left, initial, false);
    auto right_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right, initial, false);
    TestBackend* right_backend_ptr = right_backend.get();
    right_backend_ptr->setEftWrench(contact_wrench);
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        std::move(left_backend), std::move(right_backend), cfg, &buffer,
        nullptr, kinematics
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] {
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::TcpPoseTarget);
    target.seq = 2;
    target.host_time_ns = rb_servo::nowSteadyNs();
    target.left.timeout_sec = 5.0;
    target.right.timeout_sec = 5.0;
    target.left.has_tcp_target = true;
    target.right.has_tcp_target = true;
    target.right.tcp_target_stand.x = 0.05;
    target.right.tcp_target_stand.y = 0.02;
    target.right.tcp_target_stand.z = -0.05;
    buffer.setCommand(target);

    rb_servo::ServoSnapshot snapshot;
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.right_force_control.contact_active &&
            snapshot.right_force_control.compliance_frame == "sensor_origin" &&
            snapshot.right_force_control.transverse_contact_active &&
            !snapshot.right_force_control.normal_contact_active &&
            snapshot.right_force_control.compliance_active &&
            !snapshot.right_force_control.normal_regulating &&
            snapshot.right_force_control.transverse_regulating &&
            snapshot.right_force_control.proposal_committed &&
            snapshot.right_force_control.compliance_offset_surface.x < 0.0;
    }, std::chrono::milliseconds(1500)));
    const uint64_t soft_contact_epoch = snapshot.motion_epoch;
    RB_CHECK(!snapshot.fault_latched);
    RB_CHECK(snapshot.right_force_control.state == "regulating");
    RB_CHECK(std::abs(
        snapshot.right_force_control.control_wrench_compliance.fx - 8.0
    ) < 1e-6);
    RB_CHECK(std::abs(
        snapshot.right_force_control.control_wrench_compliance.fy
    ) < 1e-6);
    RB_CHECK(std::abs(
        snapshot.right_force_control.control_wrench_surface.fx
    ) < 1e-6);
    RB_CHECK(std::abs(
        snapshot.right_force_control.control_wrench_surface.fy - 8.0
    ) < 1e-6);
    RB_CHECK(snapshot.right_force_control.correction_m == 0.0);
    RB_CHECK(snapshot.right_force_control.velocity_m_s == 0.0);
    RB_CHECK(snapshot.right_force_control.acceleration_m_s2 == 0.0);
    const double equilibrium_y_before_loading =
        snapshot.right_force_control.compliance_equilibrium_stand.y;

    target.seq = 3;
    target.host_time_ns = rb_servo::nowSteadyNs();
    target.right.tcp_target_stand.y = 0.10;
    buffer.setCommand(target);
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.command.seq == target.seq &&
            snapshot.right_force_control.proposal_committed;
    }, std::chrono::milliseconds(1000)));
    RB_CHECK(snapshot.right_force_control.compliance_equilibrium_source ==
             "policy_target");
    RB_CHECK(snapshot.right_force_control.compliance_equilibrium_stand.y <=
             equilibrium_y_before_loading + 1e-6);
    RB_CHECK(snapshot.motion_epoch == soft_contact_epoch);

    right_backend_ptr->setEftWrench(rb_servo::Wrench6D{});
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return !snapshot.right_force_control.contact_active;
    }, std::chrono::milliseconds(1500)));
    RB_CHECK(snapshot.motion_epoch == soft_contact_epoch);
    RB_CHECK(!snapshot.fault_latched);
    loop.stop();
    return true;
}

bool testTcpOriginRotationalComplianceKeepsTcpTranslationFixed() {
    rb_servo::DualArmConfig cfg = testConfig();
    configureCartesianLoopTest(&cfg);
    cfg.servo.send_at_tick_start = false;
    cfg.servo.command_timeout_sec = 5.0;
    cfg.safety.floor_constraint.enable = true;
    cfg.safety.floor_constraint.z_min_m = -1.0;
    cfg.force_torque.source = "rbpodo_eft";
    cfg.force_torque.right.enable = true;
    cfg.force_torque.right.frame_configured = true;
    cfg.force_torque.right.freshness_source = "sequence";
    cfg.force_torque.right.control_lpf_alpha = 1.0;
    cfg.force_torque.right.t_tcp_sensor = {
        0.0, 0.0, -0.2, 0.0, 0.0, 1.5707963267948966,
    };
    cfg.force_control.provider = "project_native";
    cfg.force_control.enable = true;
    cfg.force_control.operating_mode = "cartesian_admittance";
    cfg.force_control.update_rate_hz = cfg.servo.rate_hz;
    cfg.force_control.right.enable = true;
    cfg.force_control.right.compliance_frame = "tcp_origin";
    cfg.force_control.right.contact_enter_force_n = 100.0;
    cfg.force_control.right.contact_release_force_n = 90.0;
    cfg.force_control.right.hard_normal_force_n = 100.0;
    cfg.force_control.right.hard_force_norm_n = 100.0;
    cfg.force_control.right.hard_torque_norm_nm = 100.0;
    cfg.force_control.right.debounce_samples = 1;
    cfg.force_control.right.transverse_contact_enter_force_n = 100.0;
    cfg.force_control.right.transverse_contact_release_force_n = 90.0;
    cfg.force_control.right.torque_contact_enter_nm = 0.5;
    cfg.force_control.right.torque_contact_release_nm = 0.2;
    cfg.force_control.right.compliance_axes = {true, true, true, true, true, true};
    cfg.force_control.virtual_mass = {1.0, 1.0, 1.0, 0.5, 0.5, 0.5};
    cfg.force_control.damping = {20.0, 20.0, 20.0, 5.0, 5.0, 5.0};
    cfg.force_control.stiffness = {30.0, 30.0, 30.0, 5.0, 5.0, 5.0};
    cfg.force_control.wrench_deadband = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    cfg.force_control.max_energy_j = 100.0;

    rb_servo::Wrench6D torque_wrench;
    torque_wrench.tx = 2.0;
    const rb_servo::JointArray initial = joints(0.0);
    auto left_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left, initial, false);
    auto right_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right, initial, false);
    right_backend->setEftWrench(torque_wrench);
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        std::move(left_backend), std::move(right_backend), cfg, &buffer,
        nullptr, kinematics
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] {
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::TcpPoseTarget);
    target.seq = 2;
    target.host_time_ns = rb_servo::nowSteadyNs();
    target.left.timeout_sec = 5.0;
    target.right.timeout_sec = 5.0;
    target.left.has_tcp_target = true;
    target.right.has_tcp_target = true;
    target.right.tcp_target_stand = {0.05, 0.02, -0.05, 0.0, 0.0, 0.0};
    buffer.setCommand(target);

    rb_servo::ServoSnapshot snapshot;
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        const rb_servo::Pose6D& offset =
            snapshot.right_force_control.compliance_offset_surface;
        return snapshot.right_force_control.compliance_frame == "tcp_origin" &&
            snapshot.right_force_control.rotational_contact_active &&
            snapshot.right_force_control.proposal_committed &&
            std::abs(offset.rx) > 1e-4 &&
            kinematics->lastRightTarget().has_value();
    }, std::chrono::milliseconds(1500)));

    const std::optional<rb_servo::Pose6D> corrected_target =
        kinematics->lastRightTarget();
    RB_CHECK(corrected_target.has_value());
    const rb_servo::Pose6D& equilibrium =
        snapshot.right_force_control.compliance_equilibrium_stand;
    RB_CHECK(snapshot.right_force_control.compliance_frame_pose_valid);
    RB_CHECK(std::abs(
        snapshot.right_force_control.compliance_frame_actual_stand.rz -
        1.5707963267948966
    ) < 1e-9);
    RB_CHECK(std::abs(corrected_target->x - equilibrium.x) < 1e-9);
    RB_CHECK(std::abs(corrected_target->y - equilibrium.y) < 1e-9);
    RB_CHECK(std::abs(corrected_target->z - equilibrium.z) < 1e-9);
    RB_CHECK(std::hypot(corrected_target->rx, corrected_target->ry) > 1e-4);
    RB_CHECK(!snapshot.fault_latched);
    loop.stop();
    return true;
}

bool testCartesianHoldComplianceUsesFixedSixAxisEquilibriumAndRecenters() {
    rb_servo::DualArmConfig cfg = testConfig();
    configureCartesianLoopTest(&cfg);
    cfg.servo.send_at_tick_start = false;
    cfg.servo.command_timeout_sec = 10.0;
    cfg.safety.floor_constraint.enable = true;
    cfg.safety.floor_constraint.z_min_m = -1.0;
    cfg.force_torque.source = "rbpodo_eft";
    cfg.force_torque.right.enable = true;
    cfg.force_torque.right.frame_configured = true;
    cfg.force_torque.right.freshness_source = "sequence";
    cfg.force_torque.right.control_lpf_alpha = 1.0;
    cfg.force_control.provider = "project_native";
    cfg.force_control.enable = true;
    cfg.force_control.operating_mode = "cartesian_admittance";
    cfg.force_control.update_rate_hz = cfg.servo.rate_hz;
    cfg.force_control.right.enable = true;
    cfg.force_control.right.contact_enter_force_n = 4.0;
    cfg.force_control.right.contact_release_force_n = 2.0;
    cfg.force_control.right.hard_normal_force_n = 100.0;
    cfg.force_control.right.hard_force_norm_n = 200.0;
    cfg.force_control.right.hard_torque_norm_nm = 100.0;
    cfg.force_control.right.debounce_samples = 1;
    cfg.force_control.right.transverse_contact_enter_force_n = 4.0;
    cfg.force_control.right.transverse_contact_release_force_n = 2.0;
    cfg.force_control.right.torque_contact_enter_nm = 0.5;
    cfg.force_control.right.torque_contact_release_nm = 0.2;
    cfg.force_control.right.compliance_axes = {true, true, true, true, true, true};
    cfg.force_control.virtual_mass = {1.0, 1.0, 1.0, 0.5, 0.5, 0.5};
    cfg.force_control.damping = {20.0, 20.0, 20.0, 5.0, 5.0, 5.0};
    cfg.force_control.stiffness = {100.0, 100.0, 100.0, 20.0, 20.0, 20.0};
    cfg.force_control.wrench_deadband = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    cfg.force_control.max_linear_velocity_m_s = 0.03;
    cfg.force_control.max_angular_velocity_rad_s = 0.1;
    cfg.force_control.max_linear_acceleration_m_s2 = 0.5;
    cfg.force_control.max_angular_acceleration_rad_s2 = 1.0;
    cfg.force_control.max_linear_jerk_m_s3 = 5.0;
    cfg.force_control.max_angular_jerk_rad_s3 = 10.0;
    cfg.force_control.max_energy_j = 100.0;

    rb_servo::Wrench6D wrench{8.0, -7.0, 6.0, 1.0, -1.0, 1.0};
    const rb_servo::JointArray initial = joints(0.0);
    auto left_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left, initial, false);
    auto right_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right, initial, false);
    TestBackend* right_backend_ptr = right_backend.get();
    right_backend_ptr->setEftWrench(wrench);
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    kinematics->setOrientationFromJoint(true);
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        std::move(left_backend), std::move(right_backend), cfg, &buffer,
        nullptr, kinematics
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] {
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));

    rb_servo::ServoSnapshot snapshot;
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        const rb_servo::Pose6D& offset =
            snapshot.right_force_control.compliance_offset_surface;
        return snapshot.right_force_control.proposal_committed &&
            snapshot.right_force_control.compliance_equilibrium_source ==
                "hold_anchor" &&
            offset.x < 0.0 && offset.y > 0.0 && offset.z < 0.0 &&
            offset.rx < 0.0 && offset.ry > 0.0 && offset.rz < 0.0;
    }, std::chrono::milliseconds(2000)));
    RB_CHECK(!snapshot.fault_latched);
    const rb_servo::Pose6D equilibrium =
        snapshot.right_force_control.compliance_equilibrium_stand;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    snapshot = loop.latestSnapshot();
    const rb_servo::Pose6D& held_equilibrium =
        snapshot.right_force_control.compliance_equilibrium_stand;
    RB_CHECK(std::abs(held_equilibrium.x - equilibrium.x) <= 1e-9);
    RB_CHECK(std::abs(held_equilibrium.y - equilibrium.y) <= 1e-9);
    RB_CHECK(std::abs(held_equilibrium.z - equilibrium.z) <= 1e-9);
    RB_CHECK(std::abs(snapshot.right_force_control.compliance_offset_surface.x) <=
             cfg.force_control.max_pos_offset_m + 1e-9);
    RB_CHECK(std::abs(snapshot.right_force_control.compliance_offset_surface.y) <=
             cfg.force_control.max_pos_offset_m + 1e-9);
    RB_CHECK(std::abs(snapshot.right_force_control.compliance_offset_surface.z) <=
             cfg.force_control.max_pos_offset_m + 1e-9);

    right_backend_ptr->setEftWrench(rb_servo::Wrench6D{});
    const bool recentered = waitUntil([&] {
        snapshot = loop.latestSnapshot();
        const rb_servo::Pose6D& offset =
            snapshot.right_force_control.compliance_offset_surface;
        const rb_servo::Vec6& velocity =
            snapshot.right_force_control.compliance_velocity_surface;
        return std::abs(offset.x) < 1e-4 && std::abs(offset.y) < 1e-4 &&
            std::abs(offset.z) < 1e-4 && std::abs(offset.rx) < 1e-4 &&
            std::abs(offset.ry) < 1e-4 && std::abs(offset.rz) < 1e-4 &&
            std::abs(velocity.x) < 1e-4 && std::abs(velocity.y) < 1e-4 &&
            std::abs(velocity.z) < 1e-4;
    }, std::chrono::milliseconds(4000));
    if (!recentered) {
        const rb_servo::Pose6D& offset =
            snapshot.right_force_control.compliance_offset_surface;
        const rb_servo::Vec6& velocity =
            snapshot.right_force_control.compliance_velocity_surface;
        std::cerr << "cartesian recenter timeout offset=["
                  << offset.x << ',' << offset.y << ',' << offset.z << ','
                  << offset.rx << ',' << offset.ry << ',' << offset.rz
                  << "] velocity=[" << velocity.x << ',' << velocity.y << ','
                  << velocity.z << ',' << velocity.rx << ',' << velocity.ry
                  << ',' << velocity.rz << "] state="
                  << snapshot.right_force_control.state << '\n';
    }
    RB_CHECK(recentered);
    RB_CHECK(snapshot.right_force_control.compliance_equilibrium_source ==
             "hold_anchor");
    RB_CHECK(!snapshot.fault_latched);
    loop.stop();
    return true;
}

bool testGuardedAdmittanceBrakesAndHoldsMeasuredPoseOnRelease() {
    rb_servo::DualArmConfig cfg = testConfig();
    configureCartesianLoopTest(&cfg);
    cfg.servo.send_at_tick_start = false;
    cfg.servo.command_timeout_sec = 5.0;
    cfg.safety.floor_constraint.enable = true;
    cfg.safety.floor_constraint.z_min_m = -1.0;
    cfg.force_torque.source = "rbpodo_eft";
    cfg.force_torque.right.enable = true;
    cfg.force_torque.right.frame_configured = true;
    cfg.force_torque.right.freshness_source = "sequence";
    cfg.force_torque.right.control_lpf_alpha = 1.0;
    cfg.force_control.provider = "project_native";
    cfg.force_control.enable = true;
    cfg.force_control.operating_mode = "guarded_admittance";
    cfg.force_control.update_rate_hz = cfg.servo.rate_hz;
    cfg.force_control.left.enable = false;
    cfg.force_control.right.enable = true;
    cfg.force_control.right.target_force_n = 2.0;
    cfg.force_control.right.contact_enter_force_n = 6.0;
    cfg.force_control.right.contact_release_force_n = 3.0;
    cfg.force_control.right.force_deadband_n = 0.5;
    cfg.force_control.right.hard_normal_force_n = 100.0;
    cfg.force_control.right.hard_force_norm_n = 100.0;
    cfg.force_control.right.hard_torque_norm_nm = 10.0;
    cfg.force_control.right.debounce_samples = 1;
    cfg.force_control.right.release_dwell_sec = 0.2;
    cfg.force_control.right.release_velocity_threshold_m_s = 0.002;
    cfg.force_control.normal_admittance.virtual_mass_kg = 8.0;
    cfg.force_control.normal_admittance.damping_n_s_m = 160.0;
    cfg.force_control.normal_admittance.stiffness_n_m = 0.0;
    cfg.force_control.normal_admittance.max_normal_velocity_m_s = 0.015;
    cfg.force_control.normal_admittance.max_normal_acceleration_m_s2 = 0.12;
    cfg.force_control.normal_admittance.max_normal_jerk_m_s3 = 0.8;
    cfg.force_control.normal_admittance.max_energy_j = 100.0;

    rb_servo::Wrench6D contact_wrench;
    contact_wrench.fz = -6.84;
    const rb_servo::JointArray initial = joints(0.0);
    auto left_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left, initial, false);
    auto right_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right, initial, false);
    TestBackend* right_backend_ptr = right_backend.get();
    right_backend_ptr->setEftWrench(contact_wrench);
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmServoLoop loop(
        std::move(left_backend),
        std::move(right_backend),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] {
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));

    rb_servo::DualArmCommand stale_target = command(rb_servo::ControlMode::TcpPoseTarget);
    stale_target.seq = 2;
    stale_target.host_time_ns = rb_servo::nowSteadyNs();
    stale_target.left.timeout_sec = 5.0;
    stale_target.right.timeout_sec = 5.0;
    stale_target.left.has_tcp_target = true;
    stale_target.right.has_tcp_target = true;
    stale_target.right.tcp_target_stand.z = -0.2;
    buffer.setCommand(stale_target);

    rb_servo::ServoSnapshot snapshot;
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.right_force_control.state == "regulating" &&
            snapshot.right_force_control.proposal_committed &&
            snapshot.right_force_control.velocity_m_s > 0.002;
    }, std::chrono::milliseconds(1500)));
    const uint64_t contact_epoch = snapshot.motion_epoch;

    // Replay the measured force tail that previously could not release:
    // contact enters above 6 N, then settles through 2.4 N toward the 2 N
    // commanded force. Both values must remain inside the 3 N release band.
    rb_servo::Wrench6D release_tail_wrench;
    release_tail_wrench.fz = -2.4;
    right_backend_ptr->setEftWrench(release_tail_wrench);
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.right_force_control.state == "release_braking" &&
            snapshot.right_force_control.contact_active &&
            snapshot.right_force_control.proposal_committed;
    }, std::chrono::milliseconds(1500)));

    // Noise below the release threshold must not bounce back to regulating or
    // reset the brake/dwell state.
    rb_servo::Wrench6D release_noise_wrench;
    release_noise_wrench.fz = -2.9;
    right_backend_ptr->setEftWrench(release_noise_wrench);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.right_force_control.state == "release_braking");
    RB_CHECK(snapshot.right_force_control.contact_active);

    rb_servo::Wrench6D target_wrench;
    target_wrench.fz = -2.0;
    right_backend_ptr->setEftWrench(target_wrench);
    RB_CHECK(waitUntil([&] {
        snapshot = loop.latestSnapshot();
        return snapshot.right_force_control.state == "release_hold" &&
            !snapshot.right_force_control.contact_active &&
            snapshot.motion_epoch > contact_epoch;
    }, std::chrono::milliseconds(3000)));
    RB_CHECK(snapshot.right_state.tcp_actual_valid);
    RB_CHECK(snapshot.right_state.tcp_actual_stand.has_value());
    RB_CHECK(kinematics->lastRightTarget().has_value());
    RB_CHECK(std::abs(
        kinematics->lastRightTarget()->z -
        snapshot.right_state.tcp_actual_stand->z
    ) < 1e-9);
    RB_CHECK(std::abs(kinematics->lastRightTarget()->z + 0.2) > 0.1);
    RB_CHECK(!snapshot.fault_latched);

    // The stale raw TcpPoseTarget remains buffered and may be returned for many
    // servo ticks before flow-infer observes the release epoch. It must not
    // clear the measured-pose hold or resume its inward z target.
    const int sends_at_release = right_backend_ptr->sendCount();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    snapshot = loop.latestSnapshot();
    RB_CHECK(right_backend_ptr->sendCount() > sends_at_release + 2);
    RB_CHECK(snapshot.right_force_control.state == "release_hold");
    RB_CHECK(kinematics->lastRightTarget().has_value());
    RB_CHECK(std::abs(kinematics->lastRightTarget()->z + 0.2) > 0.1);

    rb_servo::DualArmCommand expiring_stale = stale_target;
    expiring_stale.seq = 3;
    expiring_stale.host_time_ns = rb_servo::nowSteadyNs();
    expiring_stale.left.timeout_sec = 0.02;
    expiring_stale.right.timeout_sec = 0.02;
    expiring_stale.right.tcp_target_stand.z = -0.3;
    buffer.setCommand(expiring_stale);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.right_force_control.state == "release_hold");
    RB_CHECK(kinematics->lastRightTarget().has_value());
    RB_CHECK(std::abs(kinematics->lastRightTarget()->z + 0.3) > 0.1);

    // A raw Hold is the epoch/re-anchor acknowledgement. The server still sends
    // the measured release pose for that accepted tick, then admits a later
    // post-release Cartesian command.
    rb_servo::DualArmCommand release_ack = command(rb_servo::ControlMode::Hold);
    release_ack.seq = 4;
    release_ack.host_time_ns = rb_servo::nowSteadyNs();
    release_ack.left.timeout_sec = 5.0;
    release_ack.right.timeout_sec = 5.0;
    buffer.setCommand(release_ack);
    RB_CHECK(waitUntil([&] {
        return loop.latestSnapshot().right_force_control.state == "armed";
    }));

    rb_servo::DualArmCommand post_release = command(rb_servo::ControlMode::TcpPoseTarget);
    post_release.seq = 5;
    post_release.host_time_ns = rb_servo::nowSteadyNs();
    post_release.left.timeout_sec = 5.0;
    post_release.right.timeout_sec = 5.0;
    post_release.left.has_tcp_target = true;
    post_release.right.has_tcp_target = true;
    post_release.right.tcp_target_stand.z = 0.1;
    buffer.setCommand(post_release);
    RB_CHECK(waitUntil([&] {
        return kinematics->lastRightTarget().has_value() &&
            std::abs(kinematics->lastRightTarget()->z - 0.1) < 1e-6;
    }));
    loop.stop();
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

bool testRbpodoControllerSimulationMotionRequiresConfigAndRealEnvGates() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.unset();
    allow_motion.unset();

    rb_servo::CommandBuffer buffer;
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();

    // Real/sim env gates retired: controller-sim motion starts without envs;
    // only the config opt-in below still gates startup.
    {
        rb_servo::DualArmServoLoop loop(
            std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(loop.start());
        loop.stop();
    }

    allow_real.set("1");
    allow_motion.set("1");
    rb_servo::DualArmConfig config_closed_cfg = cfg;
    config_closed_cfg.servo.allow_controller_simulation_motion = false;
    {
        rb_servo::DualArmServoLoop loop(
            std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
            config_closed_cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

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
    allow_real.set("1");
    allow_motion.set("1");

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

bool testRbpodoControllerSimulationNotActivatedDiagnosticOverrideRequiresGate() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    const rb_servo::JointArray initial = joints(0.0);
    const auto not_activated_diagnostic_backend = [&](rb_servo::ArmId arm) {
        auto backend = std::make_unique<TestBackend>(arm, initial, false);
        backend->setHasError(true);
        backend->setErrorCode(-2001);
        backend->setServoEnabled(false);
        backend->setMotionReadinessError(
            "RobotFault",
            "rbpodo_diagnostics_suspect",
            "rbpodo_diagnostics_suspect"
        );
        return backend;
    };

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.allow_controller_simulation_diagnostics_suspect = true;

    {
        rb_servo::DualArmServoLoop loop(
            not_activated_diagnostic_backend(rb_servo::ArmId::Left),
            not_activated_diagnostic_backend(rb_servo::ArmId::Right),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    cfg.servo.allow_controller_simulation_not_activated = true;
    auto left = not_activated_diagnostic_backend(rb_servo::ArmId::Left);
    auto right = not_activated_diagnostic_backend(rb_servo::ArmId::Right);
    TestBackend* left_raw = left.get();
    TestBackend* right_raw = right.get();
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr
    );
    RB_CHECK(loop.start());
    RB_CHECK(waitUntil([&] { return loop.latestSnapshot().loop_end_time_ns > 0; }));
    RB_CHECK(left_raw->sendCount() > 0);
    RB_CHECK(right_raw->sendCount() > 0);

    rb_servo::DualArmCommand arm_motion = command(rb_servo::ControlMode::ArmMotion);
    arm_motion.seq = 2;
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil([&] {
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));

    const int left_sends_before_target = left_raw->sendCount();
    const int right_sends_before_target = right_raw->sendCount();
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.seq = 3;
    target.left.q_target_deg = joints(4.0);
    target.right.q_target_deg = joints(4.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    RB_CHECK(waitUntil([&] {
        return left_raw->sendCount() > left_sends_before_target &&
            right_raw->sendCount() > right_sends_before_target;
    }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(!snapshot.fault_latched);
    RB_CHECK(snapshot.left_send_ok);
    RB_CHECK(snapshot.right_send_ok);
    RB_CHECK(snapshot.left_state.has_error);
    RB_CHECK(snapshot.right_state.has_error);
    RB_CHECK(!snapshot.left_state.servo_enabled);
    RB_CHECK(!snapshot.right_state.servo_enabled);
    RB_CHECK(snapshot.left_state.diagnostic_error_source == "rbpodo_diagnostics_suspect");
    RB_CHECK(snapshot.right_state.diagnostic_error_source == "rbpodo_diagnostics_suspect");
    RB_CHECK(snapshot.startup_validation.allowed_unsafe_startup);
    RB_CHECK(snapshot.startup_validation.left.allowed_unsafe_startup);
    RB_CHECK(snapshot.startup_validation.right.allowed_unsafe_startup);
    RB_CHECK(containsValue(snapshot.startup_validation.left.invalid_reasons, "robot_fault"));
    RB_CHECK(containsValue(snapshot.startup_validation.left.invalid_reasons, "servo_disabled"));
    RB_CHECK(containsValue(snapshot.startup_validation.right.invalid_reasons, "robot_fault"));
    RB_CHECK(containsValue(snapshot.startup_validation.right.invalid_reasons, "servo_disabled"));

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    RB_CHECK(json.at("left").at("controller_simulation_diagnostic_override_active").get<bool>());
    RB_CHECK(json.at("right").at("controller_simulation_diagnostic_override_active").get<bool>());
    RB_CHECK(!json.at("left").at("physical_motion_expected").get<bool>());
    RB_CHECK(!json.at("right").at("physical_motion_expected").get<bool>());
    loop.stop();

    rb_servo::DualArmConfig physical_real_cfg = cfg;
    physical_real_cfg.left_robot.operation_mode = "real";
    physical_real_cfg.right_robot.operation_mode = "real";
    rb_servo::DualArmServoLoop physical_real_loop(
        not_activated_diagnostic_backend(rb_servo::ArmId::Left),
        not_activated_diagnostic_backend(rb_servo::ArmId::Right),
        physical_real_cfg,
        &buffer,
        nullptr
    );
    RB_CHECK(!physical_real_loop.start());
    return true;
}

bool testRbpodoControllerSimulationDiagnosticOverrideRejectsHardFaults() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

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
        out_of_range[2] = 370.0;
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

bool testRbpodoControllerSimulationInitErrorOverrideIsNarrow() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    const rb_servo::JointArray initial = joints(0.0);
    const auto init_error_backend = [&](rb_servo::ArmId arm) {
        auto backend = std::make_unique<TestBackend>(arm, initial, false);
        backend->setHasError(true);
        backend->setErrorCode(187);
        backend->setServoEnabled(false);
        backend->setMotionReadinessError(
            "RobotFault",
            "rbpodo_init_error",
            "rbpodo_init_error"
        );
        return backend;
    };
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();

    {
        rb_servo::DualArmServoLoop loop(
            init_error_backend(rb_servo::ArmId::Left),
            std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(!loop.start());
    }

    cfg.servo.allow_controller_simulation_init_error = true;
    auto left = init_error_backend(rb_servo::ArmId::Left);
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

    rb_servo::DualArmCommand arm_motion = command(rb_servo::ControlMode::ArmMotion);
    arm_motion.seq = 2;
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil([&] {
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));

    const int sends_before_target = left_raw->sendCount();
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.seq = 3;
    target.left.q_target_deg = joints(4.0);
    target.right.q_target_deg = joints(4.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    RB_CHECK(waitUntil([&] { return left_raw->sendCount() > sends_before_target; }));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(!snapshot.fault_latched);
    RB_CHECK(snapshot.left_send_ok);
    RB_CHECK(snapshot.left_state.has_error);
    RB_CHECK(!snapshot.left_state.servo_enabled);
    RB_CHECK(snapshot.left_state.diagnostic_error_source == "rbpodo_init_error");
    RB_CHECK(snapshot.startup_validation.allowed_unsafe_startup);
    RB_CHECK(snapshot.startup_validation.left.allowed_unsafe_startup);
    RB_CHECK(containsValue(snapshot.startup_validation.left.invalid_reasons, "robot_fault"));
    RB_CHECK(containsValue(snapshot.startup_validation.left.invalid_reasons, "servo_disabled"));

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    RB_CHECK(json.at("left").at("controller_simulation_diagnostic_override_active").get<bool>());
    RB_CHECK(!json.at("left").at("physical_motion_expected").get<bool>());
    RB_CHECK(json.at("left")
        .at("controller_simulation_mode")
        .at("controller_simulation_diagnostic_override_active")
        .get<bool>());
    loop.stop();

    rb_servo::DualArmConfig physical_real_cfg = cfg;
    physical_real_cfg.left_robot.operation_mode = "real";
    physical_real_cfg.right_robot.operation_mode = "real";
    rb_servo::DualArmServoLoop physical_real_loop(
        init_error_backend(rb_servo::ArmId::Left),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        physical_real_cfg,
        &buffer,
        nullptr
    );
    RB_CHECK(!physical_real_loop.start());
    return true;
}

bool testRbpodoControllerSimulationInitErrorOverrideRejectsOtherInvalidReasons() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    cfg.servo.allow_controller_simulation_init_error = true;

    {
        rb_servo::JointArray out_of_range = joints(0.0);
        out_of_range[2] = 370.0;
        auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, out_of_range, false);
        left->setHasError(true);
        left->setErrorCode(187);
        left->setServoEnabled(false);
        left->setMotionReadinessError(
            "RobotFault",
            "rbpodo_init_error",
            "rbpodo_init_error"
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
        left->setErrorCode(187);
        left->setServoEnabled(true);
        left->setMotionReadinessError(
            "RobotFault",
            "rbpodo_init_error",
            "rbpodo_init_error"
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
    applyIntentionalNarrowJointRangeForViolationTest(&cfg.safety);
    cfg.servo.send_servo_commands = false;
    rb_servo::JointArray out_of_range = joints(0.0);
    out_of_range[2] = 270.0;
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
    RB_CHECK(snapshot.startup_validation.left.q_range_violations.front().value_deg == 270.0);
    RB_CHECK(snapshot.send_suppressed);
    RB_CHECK(snapshot.send_policy == "read_only");
    loop.stop();
    return true;
}

bool testStartupPreservesRawControllerJointValuesInsideConfiguredRange() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.servo.send_servo_commands = true;
    rb_servo::JointArray raw = joints(0.0);
    raw[0] = 270.0;
    raw[2] = -317.0;
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, raw, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(0.0), false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    RB_CHECK(waitUntil([&] { return loop.latestSnapshot().loop_end_time_ns > 0; }));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(snapshot.startup_validation.acquisition_ok);
    RB_CHECK(snapshot.startup_validation.motion_ready);
    RB_CHECK(snapshot.startup_validation.left.q_range_violations.empty());
    RB_CHECK(snapshot.startup_validation.left.q_range_wrapped.empty());
    RB_CHECK(!snapshot.startup_validation.left.q_actual_normalized_for_safety_deg.has_value());
    RB_CHECK(sameJointArray(snapshot.left_state.q_actual_deg, raw));
    return true;
}

bool testMotionStartupRejectsRangeViolation() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    applyIntentionalNarrowJointRangeForViolationTest(&cfg.safety);
    cfg.servo.send_servo_commands = true;
    rb_servo::JointArray out_of_range = joints(0.0);
    out_of_range[2] = 270.0;
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
    // ArmMotion is no longer required: a JointTarget executes straight from ConnectedHold
    // (after the fault reset) -> Running, without a preceding ArmMotion.
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::Running);

    // ArmMotion remains accepted as a no-op state label; motion keeps running.
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

bool testControllerSimulationResetFaultResyncUsesReference() {
    const auto run_case = [](rb_servo::DualArmConfig cfg, bool expect_reference) {
        rb_servo::CommandBuffer buffer;
        const rb_servo::JointArray actual = joints(0.0);
        rb_servo::DualArmServoLoop loop(
            std::make_unique<TestBackend>(
                rb_servo::ArmId::Left,
                actual,
                false,
                rb_servo::BackendErrorKind::ControllerRejected,
                std::nullopt,
                true
            ),
            std::make_unique<TestBackend>(
                rb_servo::ArmId::Right,
                actual,
                false,
                rb_servo::BackendErrorKind::ControllerRejected,
                std::nullopt,
                true
            ),
            cfg,
            &buffer,
            nullptr
        );
        RB_CHECK(loop.start());
        rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
        target.left.q_target_deg = joints(7.0);
        target.right.q_target_deg = joints(7.0);
        target.left.has_joint_target = true;
        target.right.has_joint_target = true;
        buffer.setCommand(target);
        RB_CHECK(waitUntil([&] {
            return sameJointArray(loop.previousSentTarget().left_q_target_deg, joints(7.0));
        }));

        buffer.setCommand(command(rb_servo::ControlMode::EmergencyStop));
        RB_CHECK(waitUntil([&] { return loop.faultLatched(); }));
        buffer.setCommand(command(rb_servo::ControlMode::ResetFault));
        RB_CHECK(waitUntil([&] { return !loop.faultLatched(); }));
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        loop.stop();
        RB_CHECK(sameJointArray(
            previous.left_q_target_deg,
            expect_reference ? joints(7.0) : actual
        ));
        RB_CHECK(sameJointArray(
            previous.right_q_target_deg,
            expect_reference ? joints(7.0) : actual
        ));
        return true;
    };

    RB_CHECK(run_case(rbpodoControllerSimulationConfig(), true));
    RB_CHECK(run_case(rbpodoPhysicalRealConfig(), false));
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

bool testChunkFollowerDeactivatesAcrossBothArmJointTargetGap() {
    const int port = reserveLoopbackUdpPort();
    if (port <= 0) {
        std::cerr << "SKIP testChunkFollowerDeactivatesAcrossBothArmJointTargetGap: "
                  << "loopback UDP unavailable\n";
        return true;
    }

    rb_servo::ChunkFrameReceiver receiver("udp://127.0.0.1:" + std::to_string(port));
    RB_CHECK(receiver.start());
    RB_CHECK(sendUdpJson("127.0.0.1", port, makeDualArmChunkFramePacket(77, 24)));
    RB_CHECK(waitUntil([&] { return receiver.latestSeq() > 0; }, std::chrono::milliseconds(500)));

    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = rbpodoPhysicalRealConfig();
    configureCartesianLoopTest(&cfg);
    cfg.cartesian_control.allow_in_real = true;
    cfg.cartesian_control.tcp_pose_target_profile_default = "default";
    rb_servo::TcpPoseTargetProfileConfig profile;
    profile.name = "default";
    profile.pose_track_smd = cfg.cartesian_control.pose_track_smd;
    profile.ruckig_follower.enable = true;
    profile.ruckig_follower.fallback_policy = rb_servo::RuckigFollowerFallbackPolicy::Fault;
    cfg.cartesian_control.tcp_pose_target_profiles = {profile};

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
    loop.setChunkFrameReceiver(&receiver);

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand cartesian = command(rb_servo::ControlMode::TcpPoseTarget);
    cartesian.seq = 77;
    cartesian.host_time_ns = rb_servo::nowSteadyNs();
    cartesian.left.timeout_sec = 5.0;
    cartesian.right.timeout_sec = 5.0;
    cartesian.left.has_tcp_target = true;
    cartesian.right.has_tcp_target = true;
    cartesian.left.tcp_target_stand = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    cartesian.right.tcp_target_stand = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    cartesian.left.tcp_target_stand.quaternion_xyzw =
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
    cartesian.right.tcp_target_stand.quaternion_xyzw =
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
    buffer.setCommand(cartesian);

    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.command.seq == cartesian.seq &&
               snapshot.left_cartesian_solve.follower_active &&
               snapshot.right_cartesian_solve.follower_active &&
               snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok &&
               !snapshot.fault_latched;
    }, std::chrono::milliseconds(1000)));

    rb_servo::DualArmCommand joint = command(rb_servo::ControlMode::JointTarget);
    joint.seq = 78;
    joint.host_time_ns = rb_servo::nowSteadyNs();
    joint.left.timeout_sec = 5.0;
    joint.right.timeout_sec = 5.0;
    joint.left.has_joint_target = true;
    joint.right.has_joint_target = true;
    joint.left.q_target_deg = joints(0.0);
    joint.right.q_target_deg = joints(0.0);
    buffer.setCommand(joint);

    const uint64_t joint_start_tick = loop.latestSnapshot().tick;
    const uint64_t min_gap_ticks = static_cast<uint64_t>(
        cfg.servo.rate_hz * (profile.ruckig_follower.chunk_feed_timeout_sec + 0.1)
    );
    rb_servo::ServoSnapshot gap_snapshot;
    RB_CHECK(waitUntil([&] {
        gap_snapshot = loop.latestSnapshot();
        return gap_snapshot.command.seq == joint.seq &&
               gap_snapshot.tick >= joint_start_tick + min_gap_ticks &&
               !gap_snapshot.fault_latched;
    }, std::chrono::milliseconds(4000)));

    rb_servo::DualArmCommand resumed = cartesian;
    resumed.seq = 79;
    resumed.host_time_ns = rb_servo::nowSteadyNs();
    buffer.setCommand(resumed);

    const uint64_t resume_start_tick = loop.latestSnapshot().tick;
    rb_servo::ServoSnapshot resumed_snapshot;
    RB_CHECK(waitUntil([&] {
        resumed_snapshot = loop.latestSnapshot();
        return resumed_snapshot.command.seq == resumed.seq &&
               resumed_snapshot.tick >= resume_start_tick + 5;
    }, std::chrono::milliseconds(500)));
    const bool fault_latched = loop.faultLatched();
    const rb_servo::SafetyVerdict latched_reason = loop.latchedFaultReason();
    loop.stop();
    receiver.stop();

    RB_CHECK(resumed_snapshot.safety_verdict != rb_servo::SafetyVerdict::ChunkFollowerFault);
    RB_CHECK(!resumed_snapshot.fault_latched);
    RB_CHECK(!fault_latched);
    RB_CHECK(latched_reason != rb_servo::SafetyVerdict::ChunkFollowerFault);
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
               // Linear move now runs the same position-IK feedforward chain
               // as TcpPoseTarget (no velocity-IK on the measured state).
               kinematics->lastLeftTarget().has_value();
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
    RB_CHECK(std::abs(kinematics->lastLeftTarget()->rz) < 1e-9);
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
    // Real/sim gating retired: TcpLinearMove also computes in real run mode.
    RB_CHECK(real_snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok);
    real_loop.stop();
    return true;
}

bool testRbpodoControllerSimulationStartupReferenceSource() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");

    allow_real.set("1");
    allow_motion.set("1");

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

bool testRbpodoControllerSimulationNonStreamingCartesianGate() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    EnvVarGuard allow_real_cartesian("RB_ALLOW_REAL_CARTESIAN");

    allow_real.set("1");
    allow_motion.set("1");
    allow_real_cartesian.unset();

    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    configureCartesianLoopTest(&cfg);
    cfg.servo.send_at_tick_start = true;
    cfg.cartesian_control.allow_in_controller_simulation = true;
    cfg.cartesian_control.allow_in_real = false;
    cfg.cartesian_control.controller_simulation_servo_state_source =
        rb_servo::CartesianControllerSimulationStateSource::Reference;
    cfg.cartesian_control.controller_simulation_divergence_source =
        rb_servo::CartesianControllerSimulationStateSource::Reference;

    for (const rb_servo::ControlMode mode : nonStreamingCartesianModes()) {
        rb_servo::ServoSnapshot snapshot;
        bool ik_observed = false;
        RB_CHECK(runLeftNonStreamingCartesianCase(cfg, mode, &snapshot, &ik_observed, true));
        RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok);
        RB_CHECK(snapshot.left_cartesian_solve.status == "ok");
        RB_CHECK(snapshot.left_cartesian_solve.cartesian_servo_state_source == "reference");
        RB_CHECK(snapshot.left_cartesian_solve.cartesian_divergence_source == "reference");
        RB_CHECK(snapshot.left_cartesian_solve.q_reference_for_servo_valid);
        RB_CHECK(ik_observed);
        RB_CHECK(checkPublishedLeftCartesianGate(cfg, snapshot, true, true, ""));
    }

    // Real/sim env gates retired: physical real works without
    // RB_ALLOW_REAL_CARTESIAN (the env-closed block below is the same case).
    rb_servo::DualArmConfig physical_real_cfg = cfg;
    physical_real_cfg.left_robot.operation_mode = "real";
    physical_real_cfg.right_robot.operation_mode = "real";
    physical_real_cfg.cartesian_control.allow_in_real = true;
    allow_real_cartesian.unset();
    for (const rb_servo::ControlMode mode : nonStreamingCartesianModes()) {
        rb_servo::ServoSnapshot snapshot;
        bool ik_observed = false;
        RB_CHECK(runLeftNonStreamingCartesianCase(
            physical_real_cfg,
            mode,
            &snapshot,
            &ik_observed,
            true
        ));
        RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok);
        RB_CHECK(snapshot.left_cartesian_solve.status == "ok");
        RB_CHECK(snapshot.left_cartesian_solve.cartesian_servo_state_source == "actual");
        RB_CHECK(ik_observed);
        RB_CHECK(checkPublishedLeftCartesianGate(physical_real_cfg, snapshot, true, false, ""));
    }
    allow_real_cartesian.unset();

    rb_servo::DualArmConfig simulation_cfg = testConfig();
    simulation_cfg.left_robot.run_mode = rb_servo::RunMode::Simulation;
    simulation_cfg.right_robot.run_mode = rb_servo::RunMode::Simulation;
    configureCartesianLoopTest(&simulation_cfg);
    for (const rb_servo::ControlMode mode : nonStreamingCartesianModes()) {
        rb_servo::ServoSnapshot snapshot;
        bool ik_observed = false;
        RB_CHECK(runLeftNonStreamingCartesianCase(simulation_cfg, mode, &snapshot, &ik_observed));
        RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok);
        RB_CHECK(snapshot.left_cartesian_solve.status == "ok");
        RB_CHECK(ik_observed);
        RB_CHECK(checkPublishedLeftCartesianGate(simulation_cfg, snapshot, true, false, ""));
    }

    return true;
}

bool testRbpodoControllerSimulationCartesianIkSeedUsesReference() {
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    configureCartesianLoopTest(&cfg);
    cfg.cartesian_control.allow_in_controller_simulation = true;
    cfg.cartesian_control.allow_in_real = false;
    cfg.cartesian_control.controller_simulation_servo_state_source =
        rb_servo::CartesianControllerSimulationStateSource::Reference;
    cfg.cartesian_control.controller_simulation_divergence_source =
        rb_servo::CartesianControllerSimulationStateSource::Reference;

    rb_servo::CommandBuffer buffer;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left,
        initial,
        false,
        rb_servo::BackendErrorKind::ControllerRejected,
        initial,
        true
    );
    auto right = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right,
        initial,
        false,
        rb_servo::BackendErrorKind::ControllerRejected,
        initial,
        true
    );
    // Model a controller reference that is delayed relative to accepted sends.
    // This is the pairing that previously made every later IK solve seed from
    // the newer sent target while its TCP feedback still came from q_ref=0.
    left->setFreezeReferenceOnSend(true);
    right->setFreezeReferenceOnSend(true);

    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );
    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    rb_servo::DualArmCommand cartesian =
        leftNonStreamingCartesianCommand(rb_servo::ControlMode::TcpPoseTarget);
    cartesian.left.timeout_sec = 1.0;
    cartesian.right.timeout_sec = 1.0;
    buffer.setCommand(cartesian);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoTarget sent = loop.previousSentTarget();
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.command.seq == cartesian.seq &&
               snapshot.left_cartesian_solve.status == "ok" &&
               sent.left_q_target_deg[0] > 3.9;
    }, std::chrono::milliseconds(1000)));
    sleepTicks();
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    RB_CHECK(snapshot.left_cartesian_solve.cartesian_servo_state_source == "reference");
    RB_CHECK(std::abs(snapshot.left_cartesian_solve.q_ik_seed_deg[0]) < kEpsilon);
    RB_CHECK(loop.previousSentTarget().left_q_target_deg[0] > 3.9);
    RB_CHECK(!snapshot.fault_latched);
    loop.stop();
    return true;
}

bool testControllerSimulationTickStartSendHistoryStaysBounded() {
    rb_servo::DualArmConfig cfg = rbpodoControllerSimulationConfig();
    configureCartesianLoopTest(&cfg);
    cfg.servo.rate_hz = 500;
    cfg.servo.send_at_tick_start = true;
    cfg.cartesian_control.allow_in_controller_simulation = true;
    cfg.cartesian_control.allow_in_real = false;
    cfg.cartesian_control.controller_simulation_servo_state_source =
        rb_servo::CartesianControllerSimulationStateSource::Reference;
    cfg.cartesian_control.controller_simulation_divergence_source =
        rb_servo::CartesianControllerSimulationStateSource::Reference;
    // Match the order of magnitude of the tracked pgmode stack. With the old
    // two-send-old bookkeeping, this acceleration-limited history formed an
    // alternating recurrence and quickly escaped the 4 degree IK target.
    cfg.safety.dq_max_deg_s = joints(170.0);
    cfg.safety.ddq_max_deg_s2 = joints(1500.0);

    rb_servo::CommandBuffer buffer;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left,
        initial,
        false,
        rb_servo::BackendErrorKind::ControllerRejected,
        initial,
        true
    );
    auto right = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right,
        initial,
        false,
        rb_servo::BackendErrorKind::ControllerRejected,
        initial,
        true
    );
    // A delayed/frozen q_ref models the physical control-box pgmode response;
    // accepted sends advance only the loop's sent-target history.
    left->setFreezeReferenceOnSend(true);
    right->setFreezeReferenceOnSend(true);

    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::move(left), std::move(right), cfg, &buffer, nullptr, kinematics);
    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    rb_servo::DualArmCommand cartesian =
        leftNonStreamingCartesianCommand(rb_servo::ControlMode::TcpPoseTarget);
    cartesian.left.timeout_sec = 1.0;
    cartesian.right.timeout_sec = 1.0;
    buffer.setCommand(cartesian);

    double maximum_abs_sent_deg = 0.0;
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoTarget sent = loop.previousSentTarget();
        maximum_abs_sent_deg = std::max(
            maximum_abs_sent_deg, std::abs(sent.left_q_target_deg[0]));
        return sent.left_q_target_deg[0] > 0.1;
    }, std::chrono::milliseconds(500)));
    const auto sample_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    while (std::chrono::steady_clock::now() < sample_deadline) {
        const rb_servo::ServoTarget sent = loop.previousSentTarget();
        maximum_abs_sent_deg = std::max(
            maximum_abs_sent_deg, std::abs(sent.left_q_target_deg[0]));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();

    RB_CHECK(maximum_abs_sent_deg <= 4.01);
    RB_CHECK(!snapshot.fault_latched);
    RB_CHECK(snapshot.left_cartesian_solve.cartesian_servo_state_source == "reference");
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

// Regression: a single arm's Cartesian/IK failure must NOT freeze the healthy arm.
// Left streams a TcpPoseTarget whose IK fails every tick (mirrors a flow pose driving
// the elbow into the URDF joint limit); right runs an ordinary JointTarget. The right
// arm must keep advancing toward its goal while the aggregate verdict reports IkFailed.
// (Previously the loop blanket-held BOTH arms on any non-Ok command verdict, so a
// single-arm InitMotion or any dual-arm move stalled when the opposite arm's IK failed.)
bool testSingleArmIkFailureDoesNotFreezeHealthyArm() {
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
    // Left Cartesian IK fails on every tick; right does not use IK (JointTarget).
    kinematics->setFail(true);

    rb_servo::DualArmCommand mixed = command(rb_servo::ControlMode::JointTarget);
    mixed.seq = 42;
    mixed.host_time_ns = rb_servo::nowSteadyNs();
    mixed.left.mode = rb_servo::ControlMode::TcpPoseTarget;
    mixed.left.has_tcp_target = true;
    mixed.left.tcp_target_stand = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
    mixed.right.mode = rb_servo::ControlMode::JointTarget;
    mixed.right.has_joint_target = true;
    mixed.right.q_target_deg = joints(20.0);
    buffer.setCommand(mixed);

    // The right arm must ramp toward its JointTarget goal even though left IK fails.
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.command.seq == mixed.seq &&
               snapshot.safety_verdict == rb_servo::SafetyVerdict::IkFailed &&
               snapshot.right_sent_q_deg[0] > 1.0;
    }, std::chrono::milliseconds(1500)));

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();

    // Aggregate verdict still surfaces the left-arm IK failure for the operator...
    RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::IkFailed);
    RB_CHECK(snapshot.left_cartesian_solve.attempted);
    RB_CHECK(snapshot.left_cartesian_solve.status == "failed");
    // ...the failed left arm holds at its previous safe target...
    RB_CHECK(sameJointArray(snapshot.left_sent_q_deg, initial));
    // ...and the healthy right arm advanced toward its goal (not frozen at prev_sent).
    RB_CHECK(snapshot.right_sent_q_deg[0] > 1.0);
    RB_CHECK(snapshot.right_sent_q_deg[0] <= 20.0 + 1e-6);
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
    // Real/sim gating retired: TcpPoseTarget runs in real even with
    // allow_in_real=false (the flag no longer blocks execution).
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        return snapshot.command.left.mode == rb_servo::ControlMode::TcpPoseTarget &&
               snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok;
    }));
    loop.stop();
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

bool testFreedriveArmingQuiescesUntilIdleThenEngages() {
    rb_servo::DualArmConfig cfg = rbpodoPhysicalRealConfig();
    cfg.servo.allow_freedrive = true;
    rb_servo::CommandBuffer buffer;
    const rb_servo::JointArray initial = joints(0.0);
    auto left_backend = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right_backend = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* left = left_backend.get();
    TestBackend* right = right_backend.get();
    // Both controllers initially report actively executing servo motion (3).
    left->setControllerMotionState(3);
    right->setControllerMotionState(3);
    rb_servo::DualArmServoLoop loop(
        std::move(left_backend), std::move(right_backend), cfg, &buffer, nullptr);
    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    // Request right-arm direct teaching while the controller is still "moving".
    buffer.setCommand(freedriveCommand(2, std::nullopt, true));
    rb_servo::ServoSnapshot snap;
    RB_CHECK(waitUntil([&] {
        snap = loop.latestSnapshot();
        return snap.right_freedrive_stage == "arming_quiesce";
    }, std::chrono::milliseconds(500)));
    // Core M151 fix: teach_on must NOT be issued while the controller reports
    // motion, and servo_j must already be suppressed so the controller can settle.
    snap = loop.latestSnapshot();
    RB_CHECK(right->freedriveOnCount() == 0);
    RB_CHECK(snap.send_policy == "freedrive");
    RB_CHECK(snap.right_freedrive_active == false);

    // Controller settles to Idle (1) -> teach_on issued, engagement confirmed.
    right->setControllerMotionState(1);
    RB_CHECK(waitUntil([&] {
        snap = loop.latestSnapshot();
        return snap.right_freedrive_stage == "active";
    }, std::chrono::milliseconds(1000)));
    RB_CHECK(right->freedriveOnCount() == 1);
    RB_CHECK(right->controllerFreedriveOn());
    RB_CHECK(snap.right_freedrive_active == true);
    RB_CHECK(snap.left_freedrive_stage == "off");

    // Exit: teach_off + resync -> back to off.
    buffer.setCommand(freedriveCommand(3, std::nullopt, false));
    RB_CHECK(waitUntil([&] {
        snap = loop.latestSnapshot();
        return snap.right_freedrive_stage == "off";
    }, std::chrono::milliseconds(1000)));
    RB_CHECK(right->freedriveOffCount() >= 1);
    RB_CHECK(!right->controllerFreedriveOn());
    RB_CHECK(snap.right_freedrive_active == false);
    loop.stop();
    return true;
}

bool testFreedriveTeachOnFailureAbortsAndReleases() {
    rb_servo::DualArmConfig cfg = rbpodoPhysicalRealConfig();
    cfg.servo.allow_freedrive = true;
    rb_servo::CommandBuffer buffer;
    const rb_servo::JointArray initial = joints(0.0);
    auto left_backend = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right_backend = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    TestBackend* right = right_backend.get();
    right->setControllerMotionState(1);   // idle: machine proceeds straight to teach_on
    right->setFreedriveOnFail(true);      // controller rejects teach_on (e.g. M151)
    rb_servo::DualArmServoLoop loop(
        std::move(left_backend), std::move(right_backend), cfg, &buffer, nullptr);
    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    buffer.setCommand(freedriveCommand(2, std::nullopt, true));
    RB_CHECK(waitUntil([&] {
        return right->freedriveOnCount() >= 1;
    }, std::chrono::milliseconds(1000)));
    // Cancel so the stale ON command does not re-arm; the arm must settle to off.
    buffer.setCommand(freedriveCommand(4, std::nullopt, false));
    rb_servo::ServoSnapshot snap;
    RB_CHECK(waitUntil([&] {
        snap = loop.latestSnapshot();
        return snap.right_freedrive_stage == "off";
    }, std::chrono::milliseconds(1000)));
    // teach_on failure must not leave the arm engaged; an abort note is surfaced.
    RB_CHECK(snap.right_freedrive_active == false);
    RB_CHECK(!right->controllerFreedriveOn());
    RB_CHECK(!snap.freedrive_note.empty());
    loop.stop();
    return true;
}

bool testFreedriveExitReanchorsCartesianComplianceToTaughtPose() {
    rb_servo::DualArmConfig cfg = rbpodoPhysicalRealConfig();
    configureCartesianLoopTest(&cfg);
    cfg.servo.allow_freedrive = true;
    cfg.servo.send_at_tick_start = false;
    cfg.servo.command_timeout_sec = 5.0;
    cfg.cartesian_control.allow_in_real = true;
    cfg.safety.floor_constraint.enable = true;
    cfg.safety.floor_constraint.z_min_m = -1.0;
    cfg.force_torque.source = "rbpodo_eft";
    cfg.force_torque.right.enable = true;
    cfg.force_torque.right.frame_configured = true;
    cfg.force_torque.right.freshness_source = "sequence";
    cfg.force_torque.right.control_lpf_alpha = 1.0;
    cfg.force_control.provider = "project_native";
    cfg.force_control.enable = true;
    cfg.force_control.operating_mode = "cartesian_admittance";
    cfg.force_control.allow_in_real = true;
    cfg.force_control.supervised_experimental_real = true;
    cfg.force_control.update_rate_hz = cfg.servo.rate_hz;
    cfg.force_control.right.enable = true;
    cfg.force_control.right.surface_source = "none";
    cfg.force_control.right.compliance_frame = "tcp_origin";
    cfg.force_control.right.hard_normal_force_n = 100.0;
    cfg.force_control.right.hard_force_norm_n = 100.0;
    cfg.force_control.right.hard_torque_norm_nm = 100.0;
    cfg.force_control.right.compliance_axes = {true, true, true, true, true, true};
    cfg.force_control.virtual_mass = {1.0, 1.0, 1.0, 0.5, 0.5, 0.5};
    cfg.force_control.damping = {20.0, 20.0, 20.0, 5.0, 5.0, 5.0};
    cfg.force_control.stiffness = {100.0, 100.0, 100.0, 20.0, 20.0, 20.0};
    cfg.force_control.wrench_deadband = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    cfg.force_control.max_energy_j = 100.0;

    rb_servo::CommandBuffer buffer;
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::JointArray taught = initial;
    taught[0] = 10.0;
    taught[1] = 20.0;
    taught[2] = 30.0;
    auto left_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Left, initial, false);
    auto right_backend = std::make_unique<TestBackend>(
        rb_servo::ArmId::Right, initial, false);
    TestBackend* right = right_backend.get();
    right->setControllerMotionState(1);
    right->setEftWrench(rb_servo::Wrench6D{});
    auto kinematics = std::make_shared<FakeCartesianKinematics>();
    const rb_servo::Pose6D taught_tcp = kinematics->computeTcpStand(
        rb_servo::ArmId::Right, taught, cfg.right_mount);
    rb_servo::DualArmServoLoop loop(
        std::move(left_backend), std::move(right_backend), cfg, &buffer,
        nullptr, kinematics);

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    RB_CHECK(waitUntil([&] {
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold;
    }));
    rb_servo::DualArmCommand hold = command(rb_servo::ControlMode::Hold);
    hold.seq = 2;
    hold.host_time_ns = rb_servo::nowSteadyNs();
    hold.left.timeout_sec = 5.0;
    hold.right.timeout_sec = 5.0;
    buffer.setCommand(hold);

    rb_servo::ServoSnapshot snap;
    RB_CHECK(waitUntil([&] {
        snap = loop.latestSnapshot();
        return snap.right_force_control.proposal_committed &&
            snap.right_force_control.compliance_equilibrium_source == "hold_anchor";
    }, std::chrono::milliseconds(1500)));
    const uint64_t epoch_before_teaching = snap.motion_epoch;

    buffer.setCommand(freedriveCommand(3, std::nullopt, true));
    RB_CHECK(waitUntil([&] {
        snap = loop.latestSnapshot();
        return snap.right_freedrive_stage == "active";
    }, std::chrono::milliseconds(1000)));
    right->setActualJoints(taught);
    RB_CHECK(waitUntil([&] {
        snap = loop.latestSnapshot();
        return std::abs(snap.right_state.q_actual_deg[0] - taught[0]) < 1e-9 &&
            std::abs(snap.right_state.q_actual_deg[1] - taught[1]) < 1e-9 &&
            std::abs(snap.right_state.q_actual_deg[2] - taught[2]) < 1e-9;
    }, std::chrono::milliseconds(500)));

    buffer.setCommand(freedriveCommand(4, std::nullopt, false));
    RB_CHECK(waitUntil([&] {
        snap = loop.latestSnapshot();
        return snap.right_freedrive_stage == "off" &&
            snap.motion_epoch > epoch_before_teaching;
    }, std::chrono::milliseconds(1000)));
    RB_CHECK(snap.right_force_control.compliance_equilibrium_source == "hold_anchor");
    RB_CHECK(std::abs(
        snap.right_force_control.compliance_equilibrium_stand.x - taught_tcp.x) < 1e-9);
    RB_CHECK(std::abs(
        snap.right_force_control.compliance_equilibrium_stand.y - taught_tcp.y) < 1e-9);
    RB_CHECK(std::abs(
        snap.right_force_control.compliance_equilibrium_stand.z - taught_tcp.z) < 1e-9);
    RB_CHECK(!snap.right_force_control.compliance_active);
    RB_CHECK(!snap.right_force_control.proposal_committed);

    // Direct teaching deliberately disarms motion. Re-arm as the operator/GUI
    // would; Cartesian force Hold must resume around the taught pose, not the
    // pre-teaching equilibrium.
    rb_servo::DualArmCommand rearm = command(rb_servo::ControlMode::ArmMotion);
    rearm.seq = 5;
    rearm.host_time_ns = rb_servo::nowSteadyNs();
    buffer.setCommand(rearm);
    RB_CHECK(waitUntil([&] {
        snap = loop.latestSnapshot();
        return loop.motionState() == rb_servo::ServerMotionState::ArmedHold &&
            snap.right_force_control.proposal_committed;
    }, std::chrono::milliseconds(1500)));
    sleepTicks();
    snap = loop.latestSnapshot();
    for (std::size_t i = 0; i < taught.size(); ++i) {
        RB_CHECK(std::abs(snap.right_state.q_actual_deg[i] - taught[i]) < 1e-6);
        RB_CHECK(std::abs(snap.right_sent_q_deg[i] - taught[i]) < 1e-6);
    }
    RB_CHECK(!snap.fault_latched);
    loop.stop();
    return true;
}

int main() {
    if (!testCommandValidation()) return 1;
    if (!testSetExternalBoxesCommandParser()) return 1;
    if (!testExternalBoxReceiveStampSurvivesMotionFlood()) return 1;
    if (!testExternalBoxesDoNotDisplaceMotionLatest()) return 1;
    if (!testExternalBoxesLatestOnlySideSlot()) return 1;
    if (!testParsedExternalBoxesDoNotDisplaceMotionLatest()) return 1;
    if (!testFreedriveArmingQuiescesUntilIdleThenEngages()) return 1;
    if (!testFreedriveTeachOnFailureAbortsAndReleases()) return 1;
    if (!testFreedriveExitReanchorsCartesianComplianceToTaughtPose()) return 1;
    if (!testCommandSequenceRequiredAndMonotonic()) return 1;
    if (!testCommandSourceMetadataAndLeaseEnforcement()) return 1;
    if (!testCartesianCommandParser()) return 1;
    if (!testCartesianControllerUsesQuaternionPoseOrientation()) return 1;
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
    if (!testRbpodoAsyncConfigRejectsPhysicalRealAndMissingRealEnv()) return 1;
    if (!testRbpodoAsyncServoLoopDoesNotBlockOnSlowAckWorker()) return 1;
    if (!testRbpodoAsyncSupervisionFaultLatchesServoLoop()) return 1;
    if (!testRbpodoAsyncSupervisionFaultIsAdvisoryInControllerSimulationWhenFlagEnabled()) return 1;
    if (!testRbpodoAsyncSupervisionFlagDoesNotBypassPhysicalRealLatch()) return 1;
    if (!testRbpodoAsyncHoldStreamsServoJWithoutLatch()) return 1;
    if (!testRealHoldFreezesLastReferenceNotDriftedActual()) return 1;
    if (!testCompletedInitMotionCachedPacketDoesNotReanchorHoldToActual()) return 1;
    if (!testGripperSetpointSurvivesHoldRewrite()) return 1;
    if (!testEmergencyStopStillRequiresResetAfterArmMotion()) return 1;
    if (!testPhysicalRealSendFailureStillHardLatches()) return 1;
    if (!testRbpodoTrackingErrorIsAdvisoryInControllerSimulationWhenFlagEnabled()) return 1;
    if (!testRbpodoTrackingErrorFlagDoesNotBypassPhysicalRealLatch()) return 1;
    if (!testRbpodoAsyncReferenceSupervisionQRefUpdatesOk()) return 1;
    if (!testRbpodoAsyncReferenceSupervisionQRefStopsFaultsSocketSend()) return 1;
    if (!testRbpodoAsyncReferenceSupervisionTargetErrorFaultsSocketSend()) return 1;
    if (!testRbpodoAsyncReferenceSupervisionTargetErrorIsAdvisoryWhenFlagEnabled()) return 1;
    if (!testRbpodoAsyncReferenceSupervisionInvalidQRefFaults()) return 1;
    if (!testStatePublisherAcceptsDockerServiceHostnameEndpoint()) return 1;
    if (!testStatePublisherSerializesServoSnapshotSchema()) return 1;
    if (!testStatePublisherUsesLatestSnapshotWithoutBackendReadsAndDoesNotStallLoop()) return 1;
    if (!testStatePublisherUsesConfiguredPublishRate()) return 1;
    if (!testLoggerZeroCapacityDropsWithoutBlocking()) return 1;
    if (!testServoLoggerAppendsTcpPoseTargetDebugColumns()) return 1;
    if (!testPayloadIdentificationProfileLatchesForceMotionInhibit()) return 1;
    if (!testPayloadIdentificationRetainsRawHardLimitGuard()) return 1;
    if (!testForceRuntimeUsesTimestampAfterBackendStateRead()) return 1;
    if (!testForceHardLimitRequiresConsecutiveFreshSamples()) return 1;
    if (!testGuardedAdmittanceContinuesAcrossUpstreamHold()) return 1;
    if (!testCartesianTransverseComplianceKeepsSoftContactInCurrentMotionEpoch()) return 1;
    if (!testTcpOriginRotationalComplianceKeepsTcpTranslationFixed()) return 1;
    if (!testCartesianHoldComplianceUsesFixedSixAxisEquilibriumAndRecenters()) return 1;
    if (!testGuardedAdmittanceBrakesAndHoldsMeasuredPoseOnRelease()) return 1;
    if (!testReadOnlyDiagnosticStartupAllowsFaultedStateAndPublishesUnsafeSnapshot()) return 1;
    if (!testMotionStartupRejectsFaultedState()) return 1;
    if (!testRbpodoControllerSimulationMotionRequiresConfigAndRealEnvGates()) return 1;
    if (!testRbpodoControllerSimulationDiagnosticOverrideIsNarrow()) return 1;
    if (!testRbpodoControllerSimulationNotActivatedDiagnosticOverrideRequiresGate()) return 1;
    if (!testRbpodoControllerSimulationDiagnosticOverrideRejectsHardFaults()) return 1;
    if (!testRbpodoControllerSimulationInitErrorOverrideIsNarrow()) return 1;
    if (!testRbpodoControllerSimulationInitErrorOverrideRejectsOtherInvalidReasons()) return 1;
    if (!testReadOnlyDiagnosticStartupAllowsRangeViolationOnlyWhenConfigured()) return 1;
    if (!testStartupPreservesRawControllerJointValuesInsideConfiguredRange()) return 1;
    if (!testMotionStartupRejectsRangeViolation()) return 1;
    if (!testReadOnlyDiagnosticStartupAllowsWrongModeOnlyWhenConfigured()) return 1;
    if (!testMotionStartupRejectsWrongMode()) return 1;
    if (!testReadOnlyDiagnosticStartupRejectsAcquisitionFailure()) return 1;
    if (!testInvalidStartupRobotStateFailsStart()) return 1;
    if (!testEmergencyWinsAndResetDoesNotRun()) return 1;
    if (!testResetFaultFailureKeepsFaultLatched()) return 1;
    if (!testResetFaultRequiresFreshValidState()) return 1;
    if (!testControllerSimulationResetFaultResyncUsesReference()) return 1;
    if (!testDisarmAndCartesianHoldPreviousTarget()) return 1;
    if (!testCartesianPoseTargetUsesIkInSimulation()) return 1;
    if (!testChunkFollowerDeactivatesAcrossBothArmJointTargetGap()) return 1;
    if (!testTcpLinearMoveUsesIkInSimulationOnly()) return 1;
    if (!testRbpodoControllerSimulationStartupReferenceSource()) return 1;
    if (!testRbpodoControllerSimulationNonStreamingCartesianGate()) return 1;
    if (!testRbpodoControllerSimulationCartesianIkSeedUsesReference()) return 1;
    if (!testControllerSimulationTickStartSendHistoryStaysBounded()) return 1;
    if (!testCartesianIkFailureHoldsPreviousSafeTarget()) return 1;
    if (!testSingleArmIkFailureDoesNotFreezeHealthyArm()) return 1;
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
