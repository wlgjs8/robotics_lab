#include "rb_servo/robot/backend_result.hpp"

#include <utility>

namespace rb_servo {
namespace {

struct ErrorPolicy {
    bool retryable = false;
    bool recoverable = false;
    bool robot_fault = false;
    bool transport_fault = false;
};

ErrorPolicy defaultPolicy(BackendErrorKind kind) {
    switch (kind) {
        case BackendErrorKind::None:
            return {};
        case BackendErrorKind::TransportConnectFailed:
        case BackendErrorKind::TransportWriteFailed:
        case BackendErrorKind::TransportReadFailed:
        case BackendErrorKind::TransportTimeout:
            return {true, true, false, true};
        case BackendErrorKind::RobotDisconnected:
            return {true, true, false, false};
        case BackendErrorKind::RobotFault:
            return {false, true, true, false};
        case BackendErrorKind::InvalidJointState:
            return {true, true, false, false};
        case BackendErrorKind::CommandTimeout:
            return {true, true, false, false};
        case BackendErrorKind::DependencyUnavailable:
            return {false, false, false, false};
        case BackendErrorKind::SuppressedByPolicy:
            return {false, true, false, false};
        case BackendErrorKind::ProtocolError:
        case BackendErrorKind::UnsupportedSchema:
        case BackendErrorKind::WrongArm:
        case BackendErrorKind::WrongEndpoint:
        case BackendErrorKind::UnknownArm:
        case BackendErrorKind::RobotNotInitialized:
        case BackendErrorKind::ServoDisabled:
        case BackendErrorKind::WrongMode:
        case BackendErrorKind::InvalidTarget:
        case BackendErrorKind::ControllerRejected:
            return {false, true, false, false};
        case BackendErrorKind::Unknown:
            return {false, false, false, false};
    }
    return {false, false, false, false};
}

std::string normalizedStateAfterSource(
    const std::optional<RobotState>& state_after,
    const std::string& requested_source
) {
    if (!state_after.has_value()) {
        return "none";
    }
    if (requested_source == "response" || requested_source == "cache") {
        return requested_source;
    }
    return "none";
}

}  // namespace

std::string toString(BackendOp op) {
    switch (op) {
        case BackendOp::Connect: return "Connect";
        case BackendOp::Initialize: return "Initialize";
        case BackendOp::ReadState: return "ReadState";
        case BackendOp::SendServoJ: return "SendServoJ";
        case BackendOp::Stop: return "Stop";
        case BackendOp::ResetFault: return "ResetFault";
    }
    return "Unknown";
}

std::string toString(BackendErrorKind kind) {
    switch (kind) {
        case BackendErrorKind::None: return "None";
        case BackendErrorKind::TransportConnectFailed: return "TransportConnectFailed";
        case BackendErrorKind::TransportWriteFailed: return "TransportWriteFailed";
        case BackendErrorKind::TransportReadFailed: return "TransportReadFailed";
        case BackendErrorKind::TransportTimeout: return "TransportTimeout";
        case BackendErrorKind::ProtocolError: return "ProtocolError";
        case BackendErrorKind::UnsupportedSchema: return "UnsupportedSchema";
        case BackendErrorKind::WrongArm: return "WrongArm";
        case BackendErrorKind::WrongEndpoint: return "WrongEndpoint";
        case BackendErrorKind::UnknownArm: return "UnknownArm";
        case BackendErrorKind::RobotDisconnected: return "RobotDisconnected";
        case BackendErrorKind::RobotNotInitialized: return "RobotNotInitialized";
        case BackendErrorKind::ServoDisabled: return "ServoDisabled";
        case BackendErrorKind::WrongMode: return "WrongMode";
        case BackendErrorKind::RobotFault: return "RobotFault";
        case BackendErrorKind::InvalidJointState: return "InvalidJointState";
        case BackendErrorKind::InvalidTarget: return "InvalidTarget";
        case BackendErrorKind::ControllerRejected: return "ControllerRejected";
        case BackendErrorKind::CommandTimeout: return "CommandTimeout";
        case BackendErrorKind::DependencyUnavailable: return "DependencyUnavailable";
        case BackendErrorKind::SuppressedByPolicy: return "SuppressedByPolicy";
        case BackendErrorKind::Unknown: return "Unknown";
    }
    return "Unknown";
}

BackendError noBackendError() {
    return BackendError{};
}

BackendError backendError(
    BackendErrorKind kind,
    std::string message,
    std::string code,
    std::string name,
    std::optional<bool> retryable,
    std::optional<bool> recoverable,
    std::optional<bool> robot_fault,
    std::optional<bool> transport_fault
) {
    const ErrorPolicy policy = defaultPolicy(kind);
    BackendError error;
    error.kind = kind;
    error.code = std::move(code);
    error.name = name.empty() ? toString(kind) : std::move(name);
    error.message = std::move(message);
    error.retryable = retryable.value_or(policy.retryable);
    error.recoverable = recoverable.value_or(policy.recoverable);
    error.robot_fault = robot_fault.value_or(policy.robot_fault);
    error.transport_fault = transport_fault.value_or(policy.transport_fault);
    return error;
}

BackendTiming makeBackendTiming(uint64_t start_ns, uint64_t end_ns) {
    BackendTiming timing;
    timing.start_ns = start_ns;
    timing.end_ns = end_ns;
    if (end_ns >= start_ns) {
        timing.duration_us = static_cast<double>(end_ns - start_ns) / 1000.0;
    }
    return timing;
}

BackendResult<RobotState> okReadState(
    const RobotState& state,
    const BackendTiming& timing
) {
    BackendResult<RobotState> result;
    result.ok = true;
    result.op = BackendOp::ReadState;
    result.value = state;
    result.error = noBackendError();
    result.timing = timing;
    return result;
}

BackendResult<RobotState> failedReadState(
    const BackendError& error,
    const BackendTiming& timing
) {
    BackendResult<RobotState> result;
    result.ok = false;
    result.op = BackendOp::ReadState;
    result.error = error;
    result.timing = timing;
    return result;
}

SendServoJResult acceptedSend(
    const SendServoJRequest& request,
    const BackendTiming& timing,
    std::optional<RobotState> state_after,
    std::string state_after_source
) {
    SendServoJResult result;
    result.accepted = true;
    result.error = noBackendError();
    result.timing = timing;
    result.state_after_source = normalizedStateAfterSource(state_after, state_after_source);
    if (result.state_after_source == "none") {
        state_after.reset();
    }
    result.state_after = std::move(state_after);
    result.requested_q_deg = request.q_target_deg;
    return result;
}

SendServoJResult rejectedSend(
    const SendServoJRequest& request,
    const BackendError& error,
    const BackendTiming& timing,
    std::optional<RobotState> state_after,
    std::string state_after_source
) {
    SendServoJResult result;
    result.accepted = false;
    result.error = error;
    result.timing = timing;
    result.state_after_source = normalizedStateAfterSource(state_after, state_after_source);
    if (result.state_after_source == "none") {
        state_after.reset();
    }
    result.state_after = std::move(state_after);
    result.requested_q_deg = request.q_target_deg;
    return result;
}

}  // namespace rb_servo
