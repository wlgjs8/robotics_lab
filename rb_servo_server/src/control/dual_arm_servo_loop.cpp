#include "rb_servo/control/dual_arm_servo_loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>

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
    if (!config.kinematics.enable || !config.kinematics.publish_tcp) {
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
    kinematics_(kinematics ? std::move(kinematics) : makeKinematicsProvider(config)),
    left_traj_filter_(config.servo, config.safety),
    right_traj_filter_(config.servo, config.safety),
    safety_filter_(config.safety) {}

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
    if (!left_robot_->connect() || !right_robot_->connect()) {
        std::cerr << "[ERROR] failed to connect robots\n";
        return false;
    }
    if (!left_robot_->initialize() || !right_robot_->initialize()) {
        std::cerr << "[ERROR] failed to initialize robots\n";
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

        RobotState left_state;
        RobotState right_state;
        const bool state_ok = readRobotStates(left_state, right_state);

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
            safety_verdict = SafetyVerdict::RobotStateError;
            if (isRealMode() || config_.safety.latch_fault_on_robot_state_error) {
                latchFault(SafetyVerdict::RobotStateError, "robot state read failed or invalid", left_state, right_state);
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

        bool left_ok = false;
        bool right_ok = false;
        uint64_t left_send_start_ns = 0;
        uint64_t left_send_end_ns = 0;
        uint64_t right_send_start_ns = 0;
        uint64_t right_send_end_ns = 0;
        const ServoTarget attempted_target = safe_target;
        const bool fault_latched_before_send = fault_latched_.load();
        const bool send_suppressed = readOnlyMode();
        const std::string send_policy = send_suppressed ? "read_only" : "send_servo_j";
        sendTargets(
            attempted_target,
            &left_ok,
            &right_ok,
            &left_send_start_ns,
            &left_send_end_ns,
            &right_send_start_ns,
            &right_send_end_ns
        );
        if (!left_ok || !right_ok) {
            safety_verdict = SafetyVerdict::SendFailure;
            if (isRealMode() || config_.safety.stop_both_arms_on_single_arm_error) {
                latchFault(SafetyVerdict::SendFailure, "sendServoJ failed", left_state, right_state);
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
        sample.send_suppressed = send_suppressed;
        sample.send_policy = send_policy;
        sample.left_send_start_ns = left_send_start_ns;
        sample.left_send_end_ns = left_send_end_ns;
        sample.right_send_start_ns = right_send_start_ns;
        sample.right_send_end_ns = right_send_end_ns;
        if (left_send_start_ns > 0 && right_send_start_ns > 0) {
            const uint64_t skew_ns = left_send_start_ns > right_send_start_ns
                ? left_send_start_ns - right_send_start_ns
                : right_send_start_ns - left_send_start_ns;
            sample.send_skew_us = static_cast<double>(skew_ns) / 1000.0;
        }
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
    const bool left_ok = left_robot_ && left_robot_->readState(left);
    const bool right_ok = right_robot_ && right_robot_->readState(right);
    populateTcpPose(left, config_.left_mount);
    populateTcpPose(right, config_.right_mount);
    return left_ok && right_ok;
}

void DualArmServoLoop::populateTcpPose(RobotState& state, const ArmMountConfig& mount) const {
    state.tcp_base.reset();
    state.tcp_stand.reset();
    state.has_valid_tcp_pose = false;
    state.tcp_deferred = kinematics_ == nullptr;

    if (!kinematics_) {
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
        if (command_verdict) *command_verdict = SafetyVerdict::CartesianUnavailable;
        target.left_q_target_deg = left_prev_sent_q_deg_;
        target.right_q_target_deg = right_prev_sent_q_deg_;
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

void DualArmServoLoop::sendTargets(
    const ServoTarget& target,
    bool* left_ok,
    bool* right_ok,
    uint64_t* left_send_start_ns,
    uint64_t* left_send_end_ns,
    uint64_t* right_send_start_ns,
    uint64_t* right_send_end_ns
) {
    if (readOnlyMode()) {
        if (left_ok) *left_ok = true;
        if (right_ok) *right_ok = true;
        if (left_send_start_ns) *left_send_start_ns = 0;
        if (left_send_end_ns) *left_send_end_ns = 0;
        if (right_send_start_ns) *right_send_start_ns = 0;
        if (right_send_end_ns) *right_send_end_ns = 0;
        return;
    }

    if (left_send_start_ns) *left_send_start_ns = nowSteadyNs();
    const bool left_result = left_robot_->sendServoJ(target.left_q_target_deg);
    if (left_send_end_ns) *left_send_end_ns = nowSteadyNs();
    if (left_ok) *left_ok = left_result;

    if (right_send_start_ns) *right_send_start_ns = nowSteadyNs();
    const bool right_result = right_robot_->sendServoJ(target.right_q_target_deg);
    if (right_send_end_ns) *right_send_end_ns = nowSteadyNs();
    if (right_ok) *right_ok = right_result;
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

bool DualArmServoLoop::clearFaultLatch(RobotState& left_state, RobotState& right_state) {
    const bool left_reset_ok = left_robot_ && left_robot_->resetFault();
    const bool right_reset_ok = right_robot_ && right_robot_->resetFault();
    if (!left_reset_ok || !right_reset_ok) {
        std::cerr << "[WARN] fault latch remains: backend resetFault failed"
                  << " left=" << left_reset_ok
                  << " right=" << right_reset_ok << "\n";
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
