#pragma once

#include <optional>
#include <string>

#include "rb_servo/core/types.hpp"
#include "rb_servo/robot/backend_result.hpp"

namespace rb_servo {

struct FaultContext {
    SafetyVerdict verdict = SafetyVerdict::Ok;
    FaultDomain domain = FaultDomain::None;
    ArmId arm = ArmId::Left;
    BackendOp backend_op = BackendOp::ReadState;
    BackendError backend_error = noBackendError();
    int robot_error_code = 0;
    std::string reason;
    bool recoverable = false;
    bool retryable = false;
    bool suppress_regular_servo = false;
    std::optional<RobotState> state_after;
    std::string state_after_source = "none";
};

FaultContext classifyReadStateResult(
    const BackendResult<RobotState>& result,
    ArmId arm
);

FaultContext classifySendServoJResult(
    const SendServoJResult& result,
    ArmId arm
);

FaultContext classifyDualSendResult(const DualSendResult& result);

FaultContext classifyCommandValidation(
    SafetyVerdict verdict,
    ArmId arm,
    const std::string& reason
);

FaultContext classifyIkFailure(
    ArmId arm,
    const std::string& reason
);

}  // namespace rb_servo
