#include "rb_servo/control/fault_classifier.hpp"

#include <cstdlib>
#include <string>

namespace rb_servo {
namespace {

bool isTransportError(BackendErrorKind kind) {
    return kind == BackendErrorKind::TransportConnectFailed ||
           kind == BackendErrorKind::TransportWriteFailed ||
           kind == BackendErrorKind::TransportReadFailed ||
           kind == BackendErrorKind::TransportTimeout;
}

bool isRobotStateError(BackendErrorKind kind) {
    return kind == BackendErrorKind::RobotDisconnected ||
           kind == BackendErrorKind::RobotNotInitialized ||
           kind == BackendErrorKind::ServoDisabled ||
           kind == BackendErrorKind::WrongMode ||
           kind == BackendErrorKind::RobotFault ||
           kind == BackendErrorKind::InvalidJointState;
}

bool isEndpointError(BackendErrorKind kind) {
    return kind == BackendErrorKind::WrongArm ||
           kind == BackendErrorKind::WrongEndpoint ||
           kind == BackendErrorKind::UnknownArm;
}

int parseErrorCode(const std::string& code) {
    if (code.empty()) return 0;
    char* end = nullptr;
    const long parsed = std::strtol(code.c_str(), &end, 10);
    if (end == code.c_str() || (end != nullptr && *end != '\0')) return 0;
    return static_cast<int>(parsed);
}

std::string armName(ArmId arm) {
    return toString(arm);
}

std::string backendReason(
    BackendOp op,
    ArmId arm,
    const BackendError& error,
    const std::string& prefix
) {
    std::string reason = prefix + " during " + toString(op) + " on " + armName(arm);
    reason += ": " + toString(error.kind);
    if (!error.name.empty() && error.name != toString(error.kind)) {
        reason += " (" + error.name + ")";
    }
    if (!error.message.empty()) {
        reason += ": " + error.message;
    }
    if (!error.code.empty()) {
        reason += " code=" + error.code;
    }
    return reason;
}

FaultContext okContext(BackendOp op, ArmId arm) {
    FaultContext context;
    context.backend_op = op;
    context.arm = arm;
    return context;
}

FaultContext classifyBackendError(
    BackendOp op,
    ArmId arm,
    const BackendError& error,
    const std::optional<RobotState>& state_after
) {
    FaultContext context;
    context.arm = arm;
    context.backend_op = op;
    context.backend_error = error;
    context.recoverable = error.recoverable;
    context.retryable = error.retryable;
    context.state_after = state_after;
    context.robot_error_code = state_after.has_value()
        ? state_after->error_code
        : parseErrorCode(error.code);

    if (error.kind == BackendErrorKind::None) {
        context.verdict = SafetyVerdict::Ok;
        context.domain = FaultDomain::None;
        return context;
    }

    if (error.kind == BackendErrorKind::SuppressedByPolicy) {
        context.verdict = SafetyVerdict::Ok;
        context.domain = FaultDomain::SafetyPolicy;
        context.suppress_regular_servo = true;
        context.reason = backendReason(op, arm, error, "backend operation suppressed by safety policy");
        return context;
    }

    if (error.kind == BackendErrorKind::RobotFault) {
        context.verdict = SafetyVerdict::RobotStateError;
        context.domain = FaultDomain::RobotState;
        context.reason = backendReason(op, arm, error, "robot/controller fault");
        return context;
    }

    if (isTransportError(error.kind)) {
        context.verdict = op == BackendOp::SendServoJ
            ? SafetyVerdict::SendFailure
            : SafetyVerdict::RobotStateError;
        context.domain = FaultDomain::Backend;
        context.reason = backendReason(op, arm, error, "transport failure");
        return context;
    }

    if (isRobotStateError(error.kind)) {
        context.verdict = SafetyVerdict::RobotStateError;
        context.domain = FaultDomain::RobotState;
        context.reason = backendReason(op, arm, error, "robot state fault");
        return context;
    }

    if (isEndpointError(error.kind)) {
        context.verdict = SafetyVerdict::RobotStateError;
        context.domain = FaultDomain::Backend;
        context.reason = backendReason(op, arm, error, "backend endpoint/arm mismatch");
        return context;
    }

    if (error.kind == BackendErrorKind::InvalidTarget) {
        context.verdict = op == BackendOp::SendServoJ
            ? SafetyVerdict::InvalidCommand
            : SafetyVerdict::RobotStateError;
        context.domain = op == BackendOp::SendServoJ ? FaultDomain::Command : FaultDomain::Backend;
        context.reason = backendReason(op, arm, error, "invalid command target");
        return context;
    }

    if (error.kind == BackendErrorKind::ControllerRejected ||
        error.kind == BackendErrorKind::CommandTimeout) {
        context.verdict = op == BackendOp::SendServoJ
            ? SafetyVerdict::SendFailure
            : SafetyVerdict::RobotStateError;
        context.domain = FaultDomain::Backend;
        context.reason = backendReason(op, arm, error, "backend/controller rejected operation");
        return context;
    }

    context.verdict = op == BackendOp::SendServoJ
        ? SafetyVerdict::SendFailure
        : SafetyVerdict::RobotStateError;
    context.domain = FaultDomain::Backend;
    context.reason = backendReason(op, arm, error, "backend error");
    return context;
}

bool isFailure(const FaultContext& context) {
    return context.verdict != SafetyVerdict::Ok;
}

}  // namespace

FaultContext classifyReadStateResult(
    const BackendResult<RobotState>& result,
    ArmId arm
) {
    if (result.ok) {
        return okContext(BackendOp::ReadState, arm);
    }
    return classifyBackendError(
        BackendOp::ReadState,
        arm,
        result.error,
        result.value.has_valid_joint_state || result.value.has_error
            ? std::optional<RobotState>(result.value)
            : std::nullopt
    );
}

FaultContext classifySendServoJResult(
    const SendServoJResult& result,
    ArmId arm
) {
    if (result.accepted) {
        FaultContext context = okContext(BackendOp::SendServoJ, arm);
        context.state_after = result.state_after;
        return context;
    }
    return classifyBackendError(
        BackendOp::SendServoJ,
        arm,
        result.error,
        result.state_after
    );
}

FaultContext classifyDualSendResult(const DualSendResult& result) {
    const FaultContext left = classifySendServoJResult(result.left.result, result.left.arm_id);
    const FaultContext right = classifySendServoJResult(result.right.result, result.right.arm_id);
    if (isFailure(left)) return left;
    if (isFailure(right)) return right;
    if (left.suppress_regular_servo) return left;
    if (right.suppress_regular_servo) return right;
    return left;
}

FaultContext classifyCommandValidation(
    SafetyVerdict verdict,
    ArmId arm,
    const std::string& reason
) {
    FaultContext context;
    context.verdict = verdict;
    context.arm = arm;
    context.domain = FaultDomain::Command;
    context.reason = reason;
    context.suppress_regular_servo = verdict != SafetyVerdict::Ok;
    if (verdict == SafetyVerdict::EmergencyStop) {
        context.domain = FaultDomain::Emergency;
    } else if (verdict == SafetyVerdict::IkFailed ||
               verdict == SafetyVerdict::CartesianUnavailable) {
        context.domain = FaultDomain::Kinematics;
    }
    return context;
}

FaultContext classifyIkFailure(
    ArmId arm,
    const std::string& reason
) {
    FaultContext context;
    context.verdict = SafetyVerdict::IkFailed;
    context.domain = FaultDomain::Kinematics;
    context.arm = arm;
    context.reason = reason.empty() ? "IK failed" : reason;
    context.suppress_regular_servo = true;
    return context;
}

}  // namespace rb_servo
