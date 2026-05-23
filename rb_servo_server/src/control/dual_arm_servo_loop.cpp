#include "rb_servo/control/dual_arm_servo_loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>

#include "rb_servo/control/cartesian_controller.hpp"
#include "rb_servo/control/fault_classifier.hpp"
#include "rb_servo/control/servo_dispatcher.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/core/realtime.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"

namespace rb_servo {
namespace {
bool isCartesianMode(ControlMode mode) {
    return mode == ControlMode::TcpPoseTarget ||
           mode == ControlMode::TcpDeltaStand ||
           mode == ControlMode::TcpDeltaLocal;
}

bool isMotionMode(ControlMode mode) {
    return mode == ControlMode::JointTarget ||
           mode == ControlMode::JointVelocity ||
           mode == ControlMode::TcpPoseTarget ||
           mode == ControlMode::TcpDeltaStand ||
           mode == ControlMode::TcpDeltaLocal;
}

bool isReadOnlyBlockedMode(ControlMode mode) {
    return mode == ControlMode::ArmMotion || isMotionMode(mode);
}

bool isCommandModeMissingPayload(const ArmCommand& command) {
    switch (command.mode) {
        case ControlMode::JointTarget:
            return !command.has_joint_target;
        case ControlMode::JointVelocity:
            return !command.has_joint_velocity;
        case ControlMode::TcpPoseTarget:
            return !command.has_tcp_target;
        case ControlMode::TcpDeltaStand:
            return !command.has_tcp_delta_stand;
        case ControlMode::TcpDeltaLocal:
            return !command.has_tcp_delta_local;
        default:
            return false;
    }
}

std::shared_ptr<IKinematics> makeKinematicsProvider(const DualArmConfig& config) {
    if (!config.kinematics.enable || (!config.kinematics.publish_tcp && !config.kinematics.ik.enable)) {
        return nullptr;
    }
    if (config.kinematics.provider != "pinocchio") {
        return nullptr;
    }
    if (!PinocchioKinematics::isAvailable()) {
        std::cerr << "[WARN] FK TCP publish deferred: Pinocchio kinematics is not available in this build\n";
        return nullptr;
    }
    try {
        return std::make_shared<PinocchioKinematics>(config.kinematics);
    } catch (const std::exception& exc) {
        std::cerr << "[WARN] FK TCP publish deferred: failed to initialize kinematics: "
                  << exc.what() << "\n";
        return nullptr;
    }
}

bool envFlagEnabled(const char* name) {
    const char* value = std::getenv(name);
    return value && std::string(value) == "1";
}

RunMode runModeForArm(const DualArmConfig& config, ArmId arm_id) {
    return arm_id == ArmId::Left
        ? config.left_robot.run_mode
        : config.right_robot.run_mode;
}

BackendCallSnapshot readCallSnapshot(
    const BackendResult<RobotState>& result,
    const FaultContext& classified
) {
    BackendCallSnapshot snapshot;
    snapshot.ok = classified.verdict == SafetyVerdict::Ok;
    snapshot.accepted = snapshot.ok;
    const BackendError& error = snapshot.ok ? result.error : classified.backend_error;
    snapshot.backend_error_kind = toString(error.kind);
    snapshot.error_name = error.name;
    snapshot.error_code = error.code;
    snapshot.error_message = error.message;
    snapshot.duration_us = result.timing.duration_us;
    return snapshot;
}

BackendCallSnapshot sendCallSnapshot(const SendServoJResult& result) {
    BackendCallSnapshot snapshot;
    snapshot.ok = result.accepted || result.error.kind == BackendErrorKind::SuppressedByPolicy;
    snapshot.accepted = result.accepted;
    snapshot.backend_error_kind = toString(result.error.kind);
    snapshot.error_name = result.error.name;
    snapshot.error_code = result.error.code;
    snapshot.error_message = result.error.message;
    snapshot.duration_us = result.timing.duration_us;
    snapshot.state_after_source = result.state_after_source;
    return snapshot;
}

BackendError suppressedSendError(const std::string& send_policy) {
    return backendError(
        BackendErrorKind::SuppressedByPolicy,
        "regular servo_j suppressed by send_policy=" + send_policy,
        "",
        send_policy
    );
}
}

DualArmServoLoop::DualArmServoLoop(
    std::unique_ptr<IRobotBackend> left_robot,
    std::unique_ptr<IRobotBackend> right_robot,
    const DualArmConfig& config,
    CommandBuffer* command_buffer,
    ServoLogger* logger,
    std::shared_ptr<IKinematics> kinematics
) : left_robot_(std::move(left_robot)),
    right_robot_(std::move(right_robot)),
    config_(config),
    command_buffer_(command_buffer),
    logger_(logger),
    kinematics_(nullptr),
    kinematics_injected_(kinematics != nullptr),
    left_traj_filter_(config.servo, config.safety),
    right_traj_filter_(config.servo, config.safety),
    safety_filter_(config.safety) {
    kinematics_ = kinematics ? std::move(kinematics) : makeKinematicsProvider(config);
}

DualArmServoLoop::~DualArmServoLoop() {
    stop();
}

bool DualArmServoLoop::start() {
    if (running_) return true;
    if (!initializeRobots()) {
        return false;
    }
    running_ = true;
    startup_complete_ = false;
    startup_ok_ = false;
    thread_ = std::thread(&DualArmServoLoop::loopMain, this);
    for (int i = 0; i < 100; ++i) {
        if (startup_complete_.load()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!startup_complete_.load() || !startup_ok_.load()) {
        stop();
        return false;
    }
    return true;
}

void DualArmServoLoop::stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
    if (left_robot_) left_robot_->stop();
    if (right_robot_) right_robot_->stop();
}

bool DualArmServoLoop::isRunning() const {
    return running_;
}

ServerMotionState DualArmServoLoop::motionState() const {
    return motion_state_.load();
}

bool DualArmServoLoop::faultLatched() const {
    return fault_latched_.load();
}

SafetyVerdict DualArmServoLoop::latchedFaultReason() const {
    return latched_fault_reason_.load();
}

ServoTarget DualArmServoLoop::previousSentTarget() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ServoTarget target;
    target.left_q_target_deg = left_prev_sent_q_deg_;
    target.right_q_target_deg = right_prev_sent_q_deg_;
    return target;
}

ServoSnapshot DualArmServoLoop::latestSnapshot() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_snapshot_;
}

bool DualArmServoLoop::initializeRobots() {
    const BackendResult<RobotState> left_connect = left_robot_->connect();
    const BackendResult<RobotState> right_connect = right_robot_->connect();
    if (!left_connect.ok || !right_connect.ok) {
        std::cerr << "[ERROR] failed to connect robots"
                  << " left=" << left_connect.error.name << ":" << left_connect.error.message
                  << " right=" << right_connect.error.name << ":" << right_connect.error.message << "\n";
        return false;
    }
    const BackendResult<RobotState> left_init = left_robot_->initialize();
    const BackendResult<RobotState> right_init = right_robot_->initialize();
    if (!left_init.ok || !right_init.ok) {
        std::cerr << "[ERROR] failed to initialize robots"
                  << " left=" << left_init.error.name << ":" << left_init.error.message
                  << " right=" << right_init.error.name << ":" << right_init.error.message << "\n";
        return false;
    }

    RobotState left, right;
    if (!readRobotStates(left, right) ||
        !isValidRobotStateForStartup(left) ||
        !isValidRobotStateForStartup(right)) {
        std::cerr << "[ERROR] invalid robot startup state\n";
        return false;
    }
    left_prev_sent_q_deg_ = left.q_actual_deg;
    left_prevprev_sent_q_deg_ = left.q_actual_deg;
    right_prev_sent_q_deg_ = right.q_actual_deg;
    right_prevprev_sent_q_deg_ = right.q_actual_deg;
    left_fault_hold_q_deg_ = left.q_actual_deg;
    right_fault_hold_q_deg_ = right.q_actual_deg;
    setMotionState(ServerMotionState::ConnectedHold);
    if (readOnlyMode()) {
        std::cerr << "[INFO] servo send policy: read_only; backend sendServoJ calls are suppressed\n";
    }
    return true;
}

void DualArmServoLoop::loopMain() {
    if (!configureRealtimeForLoop()) {
        startup_ok_ = false;
        startup_complete_ = true;
        running_ = false;
        return;
    }
    startup_ok_ = true;
    startup_complete_ = true;

    const int rate_hz = config_.servo.rate_hz > 0 ? config_.servo.rate_hz : 200;
    const auto period = std::chrono::nanoseconds(static_cast<long long>(1'000'000'000LL / rate_hz));
    auto next_tick = std::chrono::steady_clock::now();
    last_loop_start_ns_ = 0;

    while (running_) {
        next_tick += period;
        const uint64_t loop_start = nowSteadyNs();
        const uint64_t nominal_period_ns = static_cast<uint64_t>(period.count());
        const uint64_t actual_period_ns = last_loop_start_ns_ == 0
            ? nominal_period_ns
            : loop_start - last_loop_start_ns_;
        const double filter_dt_sec = computeFilterDtSec(actual_period_ns, nominal_period_ns);
        last_loop_start_ns_ = loop_start;

        const BackendResult<RobotState> left_state_result = left_robot_
            ? left_robot_->readState()
            : failedReadState(backendError(BackendErrorKind::RobotDisconnected, "left backend unavailable"));
        const BackendResult<RobotState> right_state_result = right_robot_
            ? right_robot_->readState()
            : failedReadState(backendError(BackendErrorKind::RobotDisconnected, "right backend unavailable"));

        RobotState left_state = left_state_result.value;
        RobotState right_state = right_state_result.value;
        left_state.arm_id = ArmId::Left;
        right_state.arm_id = ArmId::Right;
        populateTcpPose(left_state, config_.left_mount);
        populateTcpPose(right_state, config_.right_mount);

        const FaultContext left_read_fault = classifyReadStateResult(left_state_result, ArmId::Left);
        const FaultContext right_read_fault = classifyReadStateResult(right_state_result, ArmId::Right);
        const FaultContext read_fault = left_read_fault.verdict != SafetyVerdict::Ok
            ? left_read_fault
            : right_read_fault;
        const bool state_ok = read_fault.verdict == SafetyVerdict::Ok;

        DualArmCommand command = command_buffer_
            ? command_buffer_->latestOrHold(loop_start)
            : makeHoldCommand(left_state, right_state, loop_start);
        const bool read_only_command_blocked = readOnlyMode() && commandBlockedByReadOnly(command);

        if (read_only_command_blocked) {
            setMotionState(ServerMotionState::ConnectedHold);
        } else if (commandRequestsEmergencyStop(command)) {
            latchFault(SafetyVerdict::EmergencyStop, "EmergencyStop command", left_state, right_state);
            command = makeHoldCommand(left_state, right_state, loop_start);
        } else if (commandRequestsResetFault(command)) {
            if (readOnlyMode()) {
                command = makeHoldCommand(left_state, right_state, loop_start);
            } else if (fault_latched_.load()) {
                clearFaultLatch(left_state, right_state);
            } else {
                setMotionState(ServerMotionState::ConnectedHold);
            }
            command = makeHoldCommand(left_state, right_state, loop_start);
        } else if (commandRequestsDisarmMotion(command)) {
            setMotionState(ServerMotionState::ConnectedHold);
            command = makeHoldCommand(left_state, right_state, loop_start);
        } else if (commandRequestsArmMotion(command)) {
            if (!fault_latched_.load()) {
                setMotionState(ServerMotionState::ArmedHold);
            }
            command = makeHoldCommand(left_state, right_state, loop_start);
        } else if (commandRequestsMotion(command) && !motionAllowed()) {
            command = makeHoldCommand(left_state, right_state, loop_start);
        }

        ServoTarget safe_target;
        SafetyVerdict safety_verdict = SafetyVerdict::Ok;

        if (!state_ok || !isValidJointState(left_state) || !isValidJointState(right_state)) {
            safety_verdict = state_ok ? SafetyVerdict::RobotStateError : read_fault.verdict;
            if (isRealMode() || config_.safety.latch_fault_on_robot_state_error) {
                const std::string reason = state_ok || read_fault.reason.empty()
                    ? "robot state read failed or invalid"
                    : read_fault.reason;
                latchFault(safety_verdict, reason, left_state, right_state);
                safe_target = currentFaultHoldTarget();
                safety_verdict = SafetyVerdict::FaultLatched;
            } else {
                safe_target.left_q_target_deg = left_prev_sent_q_deg_;
                safe_target.right_q_target_deg = right_prev_sent_q_deg_;
            }
        } else if (fault_latched_.load()) {
            safe_target = currentFaultHoldTarget();
            safety_verdict = SafetyVerdict::FaultLatched;
        } else if (read_only_command_blocked) {
            safe_target.left_q_target_deg = left_prev_sent_q_deg_;
            safe_target.right_q_target_deg = right_prev_sent_q_deg_;
            safety_verdict = SafetyVerdict::InvalidCommand;
        } else {
            SafetyVerdict command_verdict = SafetyVerdict::Ok;
            const bool motion_requested = commandRequestsMotion(command);
            ServoTarget desired = computeServoTarget(left_state, right_state, command, filter_dt_sec, &command_verdict);

            if (command_verdict != SafetyVerdict::Ok) {
                // Missing payload, unsupported Cartesian/IK, or other command generation failure.
                // Do not synthesize a new target or report Running for a held/rejected command.
                safe_target.left_q_target_deg = left_prev_sent_q_deg_;
                safe_target.right_q_target_deg = right_prev_sent_q_deg_;
                safety_verdict = command_verdict;
                if (motion_requested) {
                    setMotionState(ServerMotionState::ArmedHold);
                }
            } else {
                safe_target = applySafety(desired, left_state, right_state, filter_dt_sec, &safety_verdict);
                if (motion_requested) {
                    if (safety_verdict == SafetyVerdict::Ok ||
                        safety_verdict == SafetyVerdict::JointLimitClamped) {
                        setMotionState(ServerMotionState::Running);
                    } else if (!fault_latched_.load()) {
                        setMotionState(ServerMotionState::ArmedHold);
                    }
                }
            }
        }

        const ServoTarget attempted_target = safe_target;
        const bool fault_latched_before_send = fault_latched_.load();
        const std::string send_policy = currentSendPolicy();
        const bool send_suppressed = send_policy != "send_servo_j";
        DualSendResult dual_send_result = sendTargets(
            attempted_target,
            command.seq,
            send_policy,
            loop_start,
            loop_start + nominal_period_ns
        );
        const SendServoJResult& left_send_result = dual_send_result.left.result;
        const SendServoJResult& right_send_result = dual_send_result.right.result;
        const uint64_t left_send_start_ns = dual_send_result.left.dispatch_timing.start_ns;
        const uint64_t left_send_end_ns = dual_send_result.left.dispatch_timing.end_ns;
        const uint64_t right_send_start_ns = dual_send_result.right.dispatch_timing.start_ns;
        const uint64_t right_send_end_ns = dual_send_result.right.dispatch_timing.end_ns;
        const bool left_ok = left_send_result.accepted;
        const bool right_ok = right_send_result.accepted;
        if (left_send_result.state_after.has_value()) {
            left_state = *left_send_result.state_after;
            populateTcpPose(left_state, config_.left_mount);
        }
        if (right_send_result.state_after.has_value()) {
            right_state = *right_send_result.state_after;
            populateTcpPose(right_state, config_.right_mount);
        }

        const FaultContext send_fault = classifyDualSendResult(dual_send_result);
        if (send_fault.verdict != SafetyVerdict::Ok) {
            safety_verdict = send_fault.verdict;
            if (isRealMode() || config_.safety.stop_both_arms_on_single_arm_error) {
                std::string reason = send_fault.reason.empty()
                    ? "sendServoJ failed"
                    : send_fault.reason;
                latchFault(send_fault.verdict, reason, left_state, right_state);
                safe_target = currentFaultHoldTarget();
                safety_verdict = SafetyVerdict::FaultLatched;
            }
        }

        const uint64_t loop_end = nowSteadyNs();

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (left_ok && !fault_latched_before_send && !send_suppressed) {
                left_prevprev_sent_q_deg_ = left_prev_sent_q_deg_;
                left_prev_sent_q_deg_ = attempted_target.left_q_target_deg;
            }
            if (right_ok && !fault_latched_before_send && !send_suppressed) {
                right_prevprev_sent_q_deg_ = right_prev_sent_q_deg_;
                right_prev_sent_q_deg_ = attempted_target.right_q_target_deg;
            }
        }

        ServoSample sample;
        sample.tick = tick_++;
        sample.loop_start_time_ns = loop_start;
        sample.loop_end_time_ns = loop_end;
        sample.left_state = left_state;
        sample.right_state = right_state;
        sample.command = command;
        sample.left_sent_q_deg = attempted_target.left_q_target_deg;
        sample.right_sent_q_deg = attempted_target.right_q_target_deg;
        sample.left_send_ok = left_ok;
        sample.right_send_ok = right_ok;
        sample.left_last_read = readCallSnapshot(left_state_result, left_read_fault);
        sample.right_last_read = readCallSnapshot(right_state_result, right_read_fault);
        sample.left_last_send = sendCallSnapshot(left_send_result);
        sample.right_last_send = sendCallSnapshot(right_send_result);
        if (!left_ok) {
            sample.left_send_error_kind = toString(left_send_result.error.kind);
            sample.left_send_error_name = left_send_result.error.name;
            sample.left_send_error_code = left_send_result.error.code;
            sample.left_send_error_message = left_send_result.error.message;
        }
        if (!right_ok) {
            sample.right_send_error_kind = toString(right_send_result.error.kind);
            sample.right_send_error_name = right_send_result.error.name;
            sample.right_send_error_code = right_send_result.error.code;
            sample.right_send_error_message = right_send_result.error.message;
        }
        sample.send_suppressed = send_suppressed;
        sample.send_policy = send_policy;
        sample.left_send_start_ns = left_send_start_ns;
        sample.left_send_end_ns = left_send_end_ns;
        sample.right_send_start_ns = right_send_start_ns;
        sample.right_send_end_ns = right_send_end_ns;
        sample.send_skew_us = dual_send_result.left_right_start_skew_us;
        if (left_send_end_ns >= left_send_start_ns && left_send_start_ns > 0) {
            sample.left_send_duration_us = static_cast<double>(left_send_end_ns - left_send_start_ns) / 1000.0;
        }
        if (right_send_end_ns >= right_send_start_ns && right_send_start_ns > 0) {
            sample.right_send_duration_us = static_cast<double>(right_send_end_ns - right_send_start_ns) / 1000.0;
        }
        sample.period_ms = nsToMs(actual_period_ns);
        sample.filter_dt_ms = filter_dt_sec * 1000.0;
        sample.jitter_ms = nsToMs(actual_period_ns > nominal_period_ns
            ? actual_period_ns - nominal_period_ns
            : nominal_period_ns - actual_period_ns);
        sample.safety_verdict = safety_verdict;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            sample.fault_latched = fault_latched_.load();
            sample.motion_state = motion_state_.load();
            sample.fault_reason = fault_reason_;

            latest_snapshot_.tick = sample.tick;
            latest_snapshot_.loop_start_time_ns = loop_start;
            latest_snapshot_.loop_end_time_ns = loop_end;
            latest_snapshot_.left_state = left_state;
            latest_snapshot_.right_state = right_state;
            latest_snapshot_.command = command;
            latest_snapshot_.left_sent_q_deg = attempted_target.left_q_target_deg;
            latest_snapshot_.right_sent_q_deg = attempted_target.right_q_target_deg;
            latest_snapshot_.left_prev_sent_q_deg = left_prev_sent_q_deg_;
            latest_snapshot_.right_prev_sent_q_deg = right_prev_sent_q_deg_;
            latest_snapshot_.period_ms = sample.period_ms;
            latest_snapshot_.jitter_ms = sample.jitter_ms;
            latest_snapshot_.filter_dt_ms = sample.filter_dt_ms;
            latest_snapshot_.safety_verdict = safety_verdict;
            latest_snapshot_.motion_state = sample.motion_state;
            latest_snapshot_.fault_latched = sample.fault_latched;
            latest_snapshot_.latched_fault_reason = latched_fault_reason_.load();
            latest_snapshot_.fault_reason = fault_reason_;
            latest_snapshot_.left_send_ok = left_ok;
            latest_snapshot_.right_send_ok = right_ok;
            latest_snapshot_.left_last_read = sample.left_last_read;
            latest_snapshot_.right_last_read = sample.right_last_read;
            latest_snapshot_.left_last_send = sample.left_last_send;
            latest_snapshot_.right_last_send = sample.right_last_send;
            latest_snapshot_.left_send_error_kind = sample.left_send_error_kind;
            latest_snapshot_.left_send_error_name = sample.left_send_error_name;
            latest_snapshot_.left_send_error_code = sample.left_send_error_code;
            latest_snapshot_.left_send_error_message = sample.left_send_error_message;
            latest_snapshot_.right_send_error_kind = sample.right_send_error_kind;
            latest_snapshot_.right_send_error_name = sample.right_send_error_name;
            latest_snapshot_.right_send_error_code = sample.right_send_error_code;
            latest_snapshot_.right_send_error_message = sample.right_send_error_message;
            latest_snapshot_.send_suppressed = send_suppressed;
            latest_snapshot_.send_policy = send_policy;
            latest_snapshot_.left_send_start_ns = left_send_start_ns;
            latest_snapshot_.left_send_end_ns = left_send_end_ns;
            latest_snapshot_.right_send_start_ns = right_send_start_ns;
            latest_snapshot_.right_send_end_ns = right_send_end_ns;
            latest_snapshot_.send_skew_us = sample.send_skew_us;
            latest_snapshot_.left_send_duration_us = sample.left_send_duration_us;
            latest_snapshot_.right_send_duration_us = sample.right_send_duration_us;
            latest_snapshot_.logger_dropped_samples = logger_ ? logger_->droppedSamples() : 0;
        }

        if (logger_) {
            logger_->push(sample);
        }

        std::this_thread::sleep_until(next_tick);
    }
}

bool DualArmServoLoop::configureRealtimeForLoop() {
    bool ok = true;
    if (config_.servo.enable_realtime_priority) {
        ok = lockMemory() && ok;
        ok = setCurrentThreadRealtimePriority(config_.servo.realtime_priority) && ok;
    }
    if (config_.servo.cpu_core >= 0) {
        ok = pinCurrentThreadToCpu(config_.servo.cpu_core) && ok;
    }

    if (!ok && isRealMode()) {
        std::cerr << "[ERROR] realtime setup failed in real mode\n";
        return false;
    }
    return true;
}

bool DualArmServoLoop::readRobotStates(RobotState& left, RobotState& right) {
    const BackendResult<RobotState> left_result = left_robot_
        ? left_robot_->readState()
        : BackendResult<RobotState>{};
    const BackendResult<RobotState> right_result = right_robot_
        ? right_robot_->readState()
        : BackendResult<RobotState>{};
    if (left_result.ok) {
        left = left_result.value;
    } else {
        left = left_result.value;
        left.arm_id = ArmId::Left;
    }
    if (right_result.ok) {
        right = right_result.value;
    } else {
        right = right_result.value;
        right.arm_id = ArmId::Right;
    }
    populateTcpPose(left, config_.left_mount);
    populateTcpPose(right, config_.right_mount);
    return left_result.ok && right_result.ok;
}

void DualArmServoLoop::populateTcpPose(RobotState& state, const ArmMountConfig& mount) const {
    state.tcp_base.reset();
    state.tcp_stand.reset();
    state.has_valid_tcp_pose = false;
    const bool publish_tcp = config_.kinematics.publish_tcp || kinematics_injected_;
    state.tcp_deferred = kinematics_ == nullptr || !publish_tcp;

    if (!kinematics_ || !publish_tcp) {
        return;
    }
    if (!isValidJointState(state)) {
        return;
    }

    try {
        state.tcp_base = kinematics_->computeTcpBase(state.q_actual_deg);
        state.tcp_stand = kinematics_->computeTcpStand(state.arm_id, state.q_actual_deg, mount);
        state.has_valid_tcp_pose = true;
        state.tcp_deferred = false;
    } catch (const std::exception& exc) {
        std::cerr << "[WARN] FK TCP publish invalid for "
                  << (state.arm_id == ArmId::Left ? "left" : "right")
                  << " arm: " << exc.what() << "\n";
    }
}

bool DualArmServoLoop::isValidJointState(const RobotState& state) const {
    if (state.connection_state != RobotConnectionState::Connected) return false;
    if (!state.has_valid_joint_state) return false;
    if (state.has_error) return false;
    for (int i = 0; i < kDof; ++i) {
        const double q = state.q_actual_deg[i];
        if (!std::isfinite(q)) return false;
        if (q < config_.safety.q_min_deg[i] || q > config_.safety.q_max_deg[i]) return false;
    }
    return true;
}

bool DualArmServoLoop::isValidRobotStateForStartup(const RobotState& state) const {
    return isValidJointState(state);
}

ServoTarget DualArmServoLoop::computeServoTarget(
    const RobotState& left_state,
    const RobotState& right_state,
    const DualArmCommand& command,
    double dt_sec,
    SafetyVerdict* command_verdict
) {
    if (command_verdict) *command_verdict = SafetyVerdict::Ok;
    ServoTarget target;

    if (isCommandModeMissingPayload(command.left) || isCommandModeMissingPayload(command.right)) {
        if (command_verdict) *command_verdict = SafetyVerdict::InvalidCommand;
        target.left_q_target_deg = left_prev_sent_q_deg_;
        target.right_q_target_deg = right_prev_sent_q_deg_;
        return target;
    }

    if (isCartesianMode(command.left.mode) || isCartesianMode(command.right.mode)) {
        bool cartesian_available =
            config_.cartesian_control.enable &&
            config_.kinematics.enable &&
            config_.kinematics.ik.enable &&
            kinematics_ != nullptr;
        if (cartesian_available) {
            for (const ArmCommand* arm_command : {&command.left, &command.right}) {
                if (!isCartesianMode(arm_command->mode)) continue;
                const RunMode run_mode = runModeForArm(config_, arm_command->arm_id);
                if (run_mode == RunMode::Simulation) {
                    cartesian_available = config_.cartesian_control.allow_in_simulation;
                } else if (run_mode == RunMode::Real) {
                    cartesian_available =
                        config_.cartesian_control.allow_in_real &&
                        envFlagEnabled("RB_ALLOW_REAL_CARTESIAN");
                } else {
                    cartesian_available = false;
                }
                if (!cartesian_available) break;
            }
        }
        if (!cartesian_available) {
            if (command_verdict) *command_verdict = SafetyVerdict::CartesianUnavailable;
            target.left_q_target_deg = left_prev_sent_q_deg_;
            target.right_q_target_deg = right_prev_sent_q_deg_;
            return target;
        }

        CartesianController cartesian(
            config_.left_mount,
            config_.right_mount,
            kinematics_
        );

        const CartesianArmTargetResult left_cartesian_result = isCartesianMode(command.left.mode)
            ? cartesian.computeArmJointTarget(command.left, left_state, left_prev_sent_q_deg_)
            : CartesianArmTargetResult{
                SafetyVerdict::Ok,
                left_traj_filter_.computeJointTarget(command.left, left_state, left_prev_sent_q_deg_, dt_sec),
                ""
            };
        target.left_q_target_deg = left_cartesian_result.q_target_deg;

        const CartesianArmTargetResult right_cartesian_result = isCartesianMode(command.right.mode)
            ? cartesian.computeArmJointTarget(command.right, right_state, right_prev_sent_q_deg_)
            : CartesianArmTargetResult{
                SafetyVerdict::Ok,
                right_traj_filter_.computeJointTarget(command.right, right_state, right_prev_sent_q_deg_, dt_sec),
                ""
            };
        target.right_q_target_deg = right_cartesian_result.q_target_deg;

        if (left_cartesian_result.verdict != SafetyVerdict::Ok ||
            right_cartesian_result.verdict != SafetyVerdict::Ok) {
            SafetyVerdict verdict = SafetyVerdict::CartesianUnavailable;
            if (left_cartesian_result.verdict == SafetyVerdict::IkFailed ||
                right_cartesian_result.verdict == SafetyVerdict::IkFailed) {
                verdict = SafetyVerdict::IkFailed;
            }
            if (command_verdict) *command_verdict = verdict;
            if (left_cartesian_result.verdict != SafetyVerdict::Ok) {
                target.left_q_target_deg = left_prev_sent_q_deg_;
            }
            if (right_cartesian_result.verdict != SafetyVerdict::Ok) {
                target.right_q_target_deg = right_prev_sent_q_deg_;
            }
        }
        return target;
    }

    target.left_q_target_deg = left_traj_filter_.computeJointTarget(
        command.left,
        left_state,
        left_prev_sent_q_deg_,
        dt_sec
    );
    target.right_q_target_deg = right_traj_filter_.computeJointTarget(
        command.right,
        right_state,
        right_prev_sent_q_deg_,
        dt_sec
    );
    return target;
}

ServoTarget DualArmServoLoop::applySafety(
    const ServoTarget& desired,
    const RobotState& left_state,
    const RobotState& right_state,
    double dt_sec,
    SafetyVerdict* verdict
) {
    ServoTarget out;
    const SafetyCheckResult left_result = safety_filter_.filterJointTarget(
        desired.left_q_target_deg,
        left_prev_sent_q_deg_,
        left_prevprev_sent_q_deg_,
        left_state,
        dt_sec
    );
    const SafetyCheckResult right_result = safety_filter_.filterJointTarget(
        desired.right_q_target_deg,
        right_prev_sent_q_deg_,
        right_prevprev_sent_q_deg_,
        right_state,
        dt_sec
    );

    out.left_q_target_deg = left_result.filtered_q_deg;
    out.right_q_target_deg = right_result.filtered_q_deg;

    SafetyVerdict combined = SafetyVerdict::Ok;
    if (!left_result.ok) combined = left_result.verdict;
    if (!right_result.ok && combined == SafetyVerdict::Ok) combined = right_result.verdict;
    if ((left_result.joint_limit_clamped || right_result.joint_limit_clamped) && combined == SafetyVerdict::Ok) {
        combined = SafetyVerdict::JointLimitClamped;
    }

    if (combined == SafetyVerdict::TrackingError) {
        if (config_.safety.tracking_error_policy == TrackingErrorPolicy::SnapToActual) {
            // 개발/mock/rbsim용 복구 정책: 현재 실제 자세를 새 안전 기준점으로 삼고 그 자리에서 멈춘다.
            out.left_q_target_deg = left_state.q_actual_deg;
            out.right_q_target_deg = right_state.q_actual_deg;
        } else {
            latchFault(SafetyVerdict::TrackingError, "tracking error exceeded threshold", left_state, right_state);
            out = currentFaultHoldTarget();
            combined = SafetyVerdict::FaultLatched;
        }
    } else if (combined == SafetyVerdict::RobotStateError) {
        if (config_.safety.latch_fault_on_robot_state_error) {
            latchFault(SafetyVerdict::RobotStateError, "robot state error or disconnected", left_state, right_state);
            out = currentFaultHoldTarget();
            combined = SafetyVerdict::FaultLatched;
        } else {
            out.left_q_target_deg = left_prev_sent_q_deg_;
            out.right_q_target_deg = right_prev_sent_q_deg_;
        }
    }

    if (verdict) *verdict = combined;
    return out;
}

DualSendResult DualArmServoLoop::sendTargets(
    const ServoTarget& target,
    uint64_t command_seq,
    const std::string& send_policy,
    uint64_t dispatch_start_ns,
    uint64_t deadline_ns
) {
    ServoDispatchRequest dispatch_request;
    dispatch_request.left.q_target_deg = target.left_q_target_deg;
    dispatch_request.right.q_target_deg = target.right_q_target_deg;
    dispatch_request.seq = command_seq;
    dispatch_request.dispatch_start_ns = dispatch_start_ns;
    dispatch_request.deadline_ns = deadline_ns;

    if (send_policy != "send_servo_j") {
        const uint64_t suppressed_time_ns = nowSteadyNs();
        const BackendTiming timing = makeBackendTiming(suppressed_time_ns, suppressed_time_ns);
        const BackendError error = suppressedSendError(send_policy);
        dispatch_request.left.command_seq = command_seq;
        dispatch_request.left.host_time_ns = suppressed_time_ns;
        dispatch_request.left.deadline_ns = deadline_ns;
        dispatch_request.right.command_seq = command_seq;
        dispatch_request.right.host_time_ns = suppressed_time_ns;
        dispatch_request.right.deadline_ns = deadline_ns;

        DualSendResult result;
        result.left.arm_id = ArmId::Left;
        result.left.request = dispatch_request.left;
        result.left.result = rejectedSend(dispatch_request.left, error, timing);
        result.right.arm_id = ArmId::Right;
        result.right.request = dispatch_request.right;
        result.right.result = rejectedSend(dispatch_request.right, error, timing);
        result.dispatch_start_ns = suppressed_time_ns;
        result.dispatch_end_ns = suppressed_time_ns;
        result.timing = timing;
        return result;
    }

    return ServoDispatcher::dispatchDirectSequential(*left_robot_, *right_robot_, dispatch_request);
}

DualArmCommand DualArmServoLoop::makeHoldCommand(
    const RobotState& left_state,
    const RobotState& right_state,
    uint64_t now_ns
) const {
    DualArmCommand cmd;
    cmd.host_time_ns = now_ns;
    cmd.left.arm_id = ArmId::Left;
    cmd.right.arm_id = ArmId::Right;
    cmd.left.mode = ControlMode::Hold;
    cmd.right.mode = ControlMode::Hold;
    cmd.left.q_target_deg = chooseSafeHoldTarget(left_state, left_prev_sent_q_deg_);
    cmd.right.q_target_deg = chooseSafeHoldTarget(right_state, right_prev_sent_q_deg_);
    return cmd;
}

bool DualArmServoLoop::commandRequestsResetFault(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::ResetFault || command.right.mode == ControlMode::ResetFault;
}

bool DualArmServoLoop::commandRequestsEmergencyStop(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::EmergencyStop || command.right.mode == ControlMode::EmergencyStop;
}

bool DualArmServoLoop::commandRequestsArmMotion(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::ArmMotion || command.right.mode == ControlMode::ArmMotion;
}

bool DualArmServoLoop::commandRequestsDisarmMotion(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::DisarmMotion || command.right.mode == ControlMode::DisarmMotion;
}

bool DualArmServoLoop::commandRequestsMotion(const DualArmCommand& command) const {
    return isMotionMode(command.left.mode) || isMotionMode(command.right.mode);
}

bool DualArmServoLoop::commandBlockedByReadOnly(const DualArmCommand& command) const {
    return isReadOnlyBlockedMode(command.left.mode) || isReadOnlyBlockedMode(command.right.mode);
}

bool DualArmServoLoop::readOnlyMode() const {
    return !config_.servo.send_servo_commands;
}

bool DualArmServoLoop::motionAllowed() const {
    const ServerMotionState state = motion_state_.load();
    return state == ServerMotionState::ArmedHold || state == ServerMotionState::Running;
}

bool DualArmServoLoop::isRealMode() const {
    return config_.left_robot.run_mode == RunMode::Real || config_.right_robot.run_mode == RunMode::Real;
}

std::string DualArmServoLoop::currentSendPolicy() const {
    const ServerMotionState state = motion_state_.load();
    if (state == ServerMotionState::EmergencyLatched) {
        return "emergency_latched";
    }
    if (fault_latched_.load() || state == ServerMotionState::FaultLatched) {
        return "fault_latched";
    }
    if (readOnlyMode()) {
        return "read_only";
    }
    return "send_servo_j";
}

bool DualArmServoLoop::clearFaultLatch(RobotState& left_state, RobotState& right_state) {
    const BackendResult<RobotState> left_reset = left_robot_
        ? left_robot_->resetFault()
        : BackendResult<RobotState>{};
    const BackendResult<RobotState> right_reset = right_robot_
        ? right_robot_->resetFault()
        : BackendResult<RobotState>{};
    const bool left_reset_ok = left_reset.ok;
    const bool right_reset_ok = right_reset.ok;
    if (!left_reset_ok || !right_reset_ok) {
        std::cerr << "[WARN] fault latch remains: backend resetFault failed"
                  << " left=" << left_reset.error.name << ":" << left_reset.error.message
                  << " right=" << right_reset.error.name << ":" << right_reset.error.message << "\n";
        return false;
    }

    RobotState fresh_left;
    RobotState fresh_right;
    if (!readRobotStates(fresh_left, fresh_right) ||
        !isValidJointState(fresh_left) ||
        !isValidJointState(fresh_right)) {
        std::cerr << "[WARN] fault latch remains: reset did not produce fresh valid robot state\n";
        return false;
    }

    left_state = fresh_left;
    right_state = fresh_right;

    std::lock_guard<std::mutex> lock(state_mutex_);
    fault_latched_.store(false);
    fault_verdict_.store(SafetyVerdict::Ok);
    latched_fault_reason_.store(SafetyVerdict::Ok);
    fault_reason_.clear();
    left_prev_sent_q_deg_ = chooseSafeHoldTarget(left_state, left_prev_sent_q_deg_);
    right_prev_sent_q_deg_ = chooseSafeHoldTarget(right_state, right_prev_sent_q_deg_);
    left_prevprev_sent_q_deg_ = left_prev_sent_q_deg_;
    right_prevprev_sent_q_deg_ = right_prev_sent_q_deg_;
    left_fault_hold_q_deg_ = left_prev_sent_q_deg_;
    right_fault_hold_q_deg_ = right_prev_sent_q_deg_;
    setMotionState(ServerMotionState::ConnectedHold);
    std::cerr << "[INFO] fault latch cleared\n";
    return true;
}

void DualArmServoLoop::latchFault(
    SafetyVerdict verdict,
    const std::string& reason,
    const RobotState& left_state,
    const RobotState& right_state
) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (fault_latched_.load()) return;
    fault_latched_.store(true);
    fault_verdict_.store(verdict);
    latched_fault_reason_.store(verdict);
    fault_reason_ = reason;
    left_fault_hold_q_deg_ = chooseSafeHoldTarget(left_state, left_prev_sent_q_deg_);
    right_fault_hold_q_deg_ = chooseSafeHoldTarget(right_state, right_prev_sent_q_deg_);
    setMotionState(verdict == SafetyVerdict::EmergencyStop
        ? ServerMotionState::EmergencyLatched
        : ServerMotionState::FaultLatched);
    std::cerr << "[WARN] fault latched: " << toString(verdict) << " - " << reason << "\n";
}

void DualArmServoLoop::setMotionState(ServerMotionState state) {
    motion_state_ = state;
}

ServoTarget DualArmServoLoop::currentFaultHoldTarget() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ServoTarget target;
    target.left_q_target_deg = left_fault_hold_q_deg_;
    target.right_q_target_deg = right_fault_hold_q_deg_;
    return target;
}

JointArray DualArmServoLoop::chooseSafeHoldTarget(
    const RobotState& state,
    const JointArray& previous_sent
) const {
    if (isValidJointState(state)) {
        return state.q_actual_deg;
    }
    return previous_sent;
}

double DualArmServoLoop::computeFilterDtSec(uint64_t actual_period_ns, uint64_t nominal_period_ns) const {
    const double nominal_dt = nsToSec(nominal_period_ns);
    const double actual_dt = nsToSec(actual_period_ns);
    const double min_ratio = std::max(0.0, config_.servo.filter_dt_min_ratio);
    const double max_ratio = std::max(min_ratio, config_.servo.filter_dt_max_ratio);
    return std::clamp(actual_dt, nominal_dt * min_ratio, nominal_dt * max_ratio);
}

}  // namespace rb_servo
