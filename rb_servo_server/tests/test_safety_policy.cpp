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
    int readCount() const { return read_count_; }
    int resetCount() const { return reset_count_; }
    int sendCount() const { return send_count_; }

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
        state.q_actual_deg = q_actual_;
        state.q_target_deg = q_target_;
        state.q_actual_valid = valid_joint_state_;
        state.q_ref_valid = q_ref_valid_ && valid_joint_state_;
        state.q_ref_source = "test.q_ref";
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

    void setFail(bool fail) { fail_ = fail; }
    void setOrientationFromJoint(bool enabled) { orientation_from_joint_ = enabled; }
    void setOrientationSolveBiasRad(double bias_rad) { orientation_solve_bias_rad_ = bias_rad; }
    void setSolveError(double position_error_m, double orientation_error_rad) {
        position_error_m_ = position_error_m;
        orientation_error_rad_ = orientation_error_rad;
    }
    std::optional<rb_servo::Pose6D> lastLeftTarget() const { return last_left_target_; }
    std::optional<rb_servo::Pose6D> lastRightTarget() const { return last_right_target_; }

private:
    bool fail_ = false;
    bool orientation_from_joint_ = false;
    double orientation_solve_bias_rad_ = 0.0;
    double position_error_m_ = 0.0;
    double orientation_error_rad_ = 0.0;
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

    RB_CHECK(!server.parseMessage(
        R"({"schema_version":1,"seq":11,"mode":"JointTarget","timeout_sec":0.2,"left":{"mode":"JointTarget","q_target_deg":[1,2,3,4,5,6],"joint_target_profile":"bad"},"right":{"mode":"Hold"}})",
        now,
        &out
    ));

    RB_CHECK(server.parseMessage(
        R"({"schema_version":1,"seq":12,"mode":"Hold","timeout_sec":0.2,"left":{"mode":"TcpPoseTarget","tcp_target_stand":[0.2,0.0,0.4,0,0,0]},"right":{"mode":"Hold"}})",
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
    for (double q : result.filtered_q_deg) {
        RB_CHECK(q <= 0.1 + kEpsilon);
        RB_CHECK(q >= -kEpsilon);
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

bool testRealHoldStreamsCurrentActualAndArmMotionReanchors() {
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
            sameJointArray(previous.left_q_target_deg, drifted_actual) &&
            sameJointArray(previous.right_q_target_deg, drifted_actual)) {
            hold_snapshot = snapshot;
            return true;
        }
        return false;
    }, std::chrono::milliseconds(1000)));

    RB_CHECK(hold_snapshot.has_value());
    RB_CHECK(sameJointArray(hold_snapshot->left_sent_q_deg, drifted_actual));
    RB_CHECK(sameJointArray(hold_snapshot->right_sent_q_deg, drifted_actual));

    arm_motion.seq = 9103;
    arm_motion.host_time_ns = rb_servo::nowSteadyNs();
    buffer.setCommand(arm_motion);
    RB_CHECK(waitUntil([&] {
        const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
        const rb_servo::ServoTarget previous = loop.previousSentTarget();
        return snapshot.motion_state == rb_servo::ServerMotionState::ArmedHold &&
               !snapshot.fault_latched &&
               snapshot.send_policy == "send_servo_j" &&
               sameJointArray(previous.left_q_target_deg, drifted_actual) &&
               sameJointArray(previous.right_q_target_deg, drifted_actual);
    }, std::chrono::milliseconds(1000)));

    loop.stop();
    RB_CHECK(left_backend->resetCount() == 0);
    RB_CHECK(right_backend->resetCount() == 0);
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::Ok);
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
        "last_read", "last_send", "robot_time_ns", "host_time_ns", "error_code", "state_age_us",
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
    cfg.cartesian_control.allow_in_controller_simulation = true;
    cfg.cartesian_control.allow_in_real = false;

    for (const rb_servo::ControlMode mode : nonStreamingCartesianModes()) {
        rb_servo::ServoSnapshot snapshot;
        bool ik_observed = false;
        RB_CHECK(runLeftNonStreamingCartesianCase(cfg, mode, &snapshot, &ik_observed, true));
        RB_CHECK(snapshot.safety_verdict == rb_servo::SafetyVerdict::Ok);
        RB_CHECK(snapshot.left_cartesian_solve.status == "ok");
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

int main() {
    if (!testCommandValidation()) return 1;
    if (!testFreedriveArmingQuiescesUntilIdleThenEngages()) return 1;
    if (!testFreedriveTeachOnFailureAbortsAndReleases()) return 1;
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
    if (!testRealHoldStreamsCurrentActualAndArmMotionReanchors()) return 1;
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
    if (!testDisarmAndCartesianHoldPreviousTarget()) return 1;
    if (!testCartesianPoseTargetUsesIkInSimulation()) return 1;
    if (!testTcpLinearMoveUsesIkInSimulationOnly()) return 1;
    if (!testRbpodoControllerSimulationStartupReferenceSource()) return 1;
    if (!testRbpodoControllerSimulationNonStreamingCartesianGate()) return 1;
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
