#include <iostream>
#include <string>

#include "rb_servo/control/fault_classifier.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

rb_servo::SendServoJRequest request() {
    rb_servo::SendServoJRequest out;
    out.command_seq = 7;
    out.q_target_deg = {1.0, -29.0, 80.0, 0.0, 60.0, 0.0};
    return out;
}

bool contains(const std::string& text, const std::string& needle) {
    return text.find(needle) != std::string::npos;
}

bool testFaultDomainToString() {
    RB_CHECK(rb_servo::toString(rb_servo::FaultDomain::None) == "None");
    RB_CHECK(rb_servo::toString(rb_servo::FaultDomain::SafetyPolicy) == "SafetyPolicy");
    RB_CHECK(rb_servo::toString(rb_servo::FaultDomain::Backend) == "Backend");
    RB_CHECK(rb_servo::toString(rb_servo::FaultDomain::RobotState) == "RobotState");
    RB_CHECK(rb_servo::toString(rb_servo::FaultDomain::Command) == "Command");
    RB_CHECK(rb_servo::toString(rb_servo::FaultDomain::Kinematics) == "Kinematics");
    RB_CHECK(rb_servo::toString(rb_servo::FaultDomain::Emergency) == "Emergency");
    return true;
}

bool testRobotFaultSendIsRobotStateFault() {
    rb_servo::RobotState state_after;
    state_after.arm_id = rb_servo::ArmId::Left;
    state_after.connection_state = rb_servo::RobotConnectionState::Connected;
    state_after.has_valid_joint_state = true;
    state_after.has_error = true;
    state_after.error_code = 2222;

    const rb_servo::SendServoJResult result = rb_servo::rejectedSend(
        request(),
        rb_servo::backendError(
            rb_servo::BackendErrorKind::RobotFault,
            "controller fault latched",
            "2222",
            "fault_latched"
        ),
        {},
        state_after,
        "response"
    );

    const rb_servo::FaultContext context =
        rb_servo::classifySendServoJResult(result, rb_servo::ArmId::Left);
    RB_CHECK(context.verdict == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(context.domain == rb_servo::FaultDomain::RobotState);
    RB_CHECK(context.backend_error.kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(context.robot_error_code == 2222);
    RB_CHECK(context.state_after.has_value());
    RB_CHECK(contains(context.reason, "robot/controller fault"));
    RB_CHECK(!contains(context.reason, "transport failure"));
    return true;
}

bool testTransportSendFailureStaysSendFailure() {
    const rb_servo::SendServoJResult result = rb_servo::rejectedSend(
        request(),
        rb_servo::backendError(
            rb_servo::BackendErrorKind::TransportWriteFailed,
            "socket write failed",
            "2101",
            "send_failure_injected"
        )
    );

    const rb_servo::FaultContext context =
        rb_servo::classifySendServoJResult(result, rb_servo::ArmId::Left);
    RB_CHECK(context.verdict == rb_servo::SafetyVerdict::SendFailure);
    RB_CHECK(context.domain == rb_servo::FaultDomain::Backend);
    RB_CHECK(context.backend_error.transport_fault);
    RB_CHECK(context.retryable);
    RB_CHECK(context.recoverable);
    RB_CHECK(contains(context.reason, "transport failure"));
    return true;
}

bool testSuppressedByPolicyIsNotFailure() {
    const rb_servo::SendServoJResult result = rb_servo::rejectedSend(
        request(),
        rb_servo::backendError(
            rb_servo::BackendErrorKind::SuppressedByPolicy,
            "motion gate closed",
            "",
            "rbpodo_motion_gate_closed"
        )
    );

    const rb_servo::FaultContext context =
        rb_servo::classifySendServoJResult(result, rb_servo::ArmId::Right);
    RB_CHECK(context.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(context.domain == rb_servo::FaultDomain::SafetyPolicy);
    RB_CHECK(context.suppress_regular_servo);
    RB_CHECK(!context.backend_error.robot_fault);
    RB_CHECK(!context.backend_error.transport_fault);
    return true;
}

bool testDualSendPrefersRealFailureOverPolicySuppression() {
    rb_servo::DualSendResult dual;
    dual.left.arm_id = rb_servo::ArmId::Left;
    dual.left.request = request();
    dual.left.result = rb_servo::rejectedSend(
        dual.left.request,
        rb_servo::backendError(rb_servo::BackendErrorKind::SuppressedByPolicy, "motion gate closed")
    );
    dual.right.arm_id = rb_servo::ArmId::Right;
    dual.right.request = request();
    dual.right.result = rb_servo::rejectedSend(
        dual.right.request,
        rb_servo::backendError(rb_servo::BackendErrorKind::TransportTimeout, "send timed out")
    );

    const rb_servo::FaultContext context = rb_servo::classifyDualSendResult(dual);
    RB_CHECK(context.arm == rb_servo::ArmId::Right);
    RB_CHECK(context.verdict == rb_servo::SafetyVerdict::SendFailure);
    RB_CHECK(context.domain == rb_servo::FaultDomain::Backend);
    return true;
}

bool testCommandAndIkClassifiers() {
    const rb_servo::FaultContext invalid = rb_servo::classifyCommandValidation(
        rb_servo::SafetyVerdict::InvalidCommand,
        rb_servo::ArmId::Left,
        "missing joint target"
    );
    RB_CHECK(invalid.verdict == rb_servo::SafetyVerdict::InvalidCommand);
    RB_CHECK(invalid.domain == rb_servo::FaultDomain::Command);
    RB_CHECK(invalid.suppress_regular_servo);

    const rb_servo::FaultContext ik =
        rb_servo::classifyIkFailure(rb_servo::ArmId::Right, "IK solver returned no solution");
    RB_CHECK(ik.verdict == rb_servo::SafetyVerdict::IkFailed);
    RB_CHECK(ik.domain == rb_servo::FaultDomain::Kinematics);
    RB_CHECK(ik.suppress_regular_servo);
    return true;
}

}  // namespace

int main() {
    if (!testFaultDomainToString()) return 1;
    if (!testRobotFaultSendIsRobotStateFault()) return 1;
    if (!testTransportSendFailureStaysSendFailure()) return 1;
    if (!testSuppressedByPolicyIsNotFailure()) return 1;
    if (!testDualSendPrefersRealFailureOverPolicySuppression()) return 1;
    if (!testCommandAndIkClassifiers()) return 1;
    return 0;
}
