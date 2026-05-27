#include "rb_servo/robot/mock_backend.hpp"

#include <algorithm>
#include "rb_servo/core/clock.hpp"

namespace rb_servo {

MockBackend::MockBackend(ArmId arm_id, const BackendConfig& config)
    : arm_id_(arm_id), config_(config) {}

namespace {

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

}  // namespace

BackendResult<RobotState> MockBackend::connect() {
    const uint64_t start = nowSteadyNs();
    connected_ = true;
    return okResult(BackendOp::Connect, readState().value, makeBackendTiming(start, nowSteadyNs()));
}

BackendResult<RobotState> MockBackend::initialize() {
    const uint64_t start = nowSteadyNs();
    if (!connected_) {
        return failedResult<RobotState>(
            BackendOp::Initialize,
            backendError(BackendErrorKind::RobotDisconnected, "mock backend is not connected"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    q_actual_deg_ = config_.initial_q_deg;
    q_target_deg_ = config_.initial_q_deg;
    initialized_ = true;
    last_update_time_ns_ = nowSteadyNs();
    return okResult(BackendOp::Initialize, readState().value, makeBackendTiming(start, nowSteadyNs()));
}

BackendResult<RobotState> MockBackend::readState() {
    const uint64_t start = nowSteadyNs();
    const uint64_t now = nowSteadyNs();
    const double dt = last_update_time_ns_ == 0 ? 0.005 : nsToSec(now - last_update_time_ns_);
    last_update_time_ns_ = now;

    const double tau = 0.04;
    const double alpha = std::clamp(dt / tau, 0.0, 1.0);
    for (int i = 0; i < kDof; ++i) {
        q_actual_deg_[i] += alpha * (q_target_deg_[i] - q_actual_deg_[i]);
    }

    RobotState state;
    state.arm_id = arm_id_;
    state.host_time_ns = now;
    state.q_actual_deg = q_actual_deg_;
    state.q_target_deg = q_target_deg_;
    state.has_valid_joint_state = connected_;
    state.connection_state = connected_ ? RobotConnectionState::Connected : RobotConnectionState::Disconnected;
    state.servo_enabled = initialized_;
    state.has_error = false;
    state.error_code = 0;
    if (!connected_) {
        return failedResult<RobotState>(
            BackendOp::ReadState,
            backendError(BackendErrorKind::RobotDisconnected, "mock backend is not connected"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    return okResult(BackendOp::ReadState, state, makeBackendTiming(start, nowSteadyNs()));
}

SendServoJResult MockBackend::sendServoJ(const SendServoJRequest& request) {
    const uint64_t start = nowSteadyNs();
    const BackendTiming timing = makeBackendTiming(start, nowSteadyNs());
    if (!connected_) {
        return rejectedSend(
            request,
            backendError(BackendErrorKind::RobotDisconnected, "mock backend is not connected"),
            timing
        );
    }
    if (!initialized_) {
        return rejectedSend(
            request,
            backendError(BackendErrorKind::RobotNotInitialized, "mock backend is not initialized"),
            timing
        );
    }
    q_target_deg_ = request.q_target_deg;
    return acceptedSend(request, timing, readState().value, "cache");
}

BackendResult<RobotState> MockBackend::stop() {
    const uint64_t start = nowSteadyNs();
    q_target_deg_ = q_actual_deg_;
    return okResult(BackendOp::Stop, readState().value, makeBackendTiming(start, nowSteadyNs()));
}

BackendResult<RobotState> MockBackend::resetFault() {
    const uint64_t start = nowSteadyNs();
    return okResult(BackendOp::ResetFault, readState().value, makeBackendTiming(start, nowSteadyNs()));
}

bool MockBackend::isConnected() const {
    return connected_;
}

ArmId MockBackend::armId() const {
    return arm_id_;
}

std::string MockBackend::name() const {
    return config_.name;
}

}  // namespace rb_servo
