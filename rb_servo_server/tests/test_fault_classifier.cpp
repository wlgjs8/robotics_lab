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
    RB_CHECK(context.state_after_source == "response");
    RB_CHECK(contains(context.reason, "robot/controller fault"));
    RB_CHECK(!contains(context.reason, "transport failure"));
    return true;
}

bool testRobotFaultReadIsRobotStateFault() {
    rb_servo::RobotState faulted;
    faulted.arm_id = rb_servo::ArmId::Left;
    faulted.connection_state = rb_servo::RobotConnectionState::Connected;
    faulted.has_valid_joint_state = true;
    faulted.has_error = true;
    faulted.error_code = 2222;

    rb_servo::BackendResult<rb_servo::RobotState> result;
    result.ok = true;
    result.op = rb_servo::BackendOp::ReadState;
    result.value = faulted;

    const rb_servo::FaultContext context =
        rb_servo::classifyReadStateResult(result, rb_servo::ArmId::Left);
    RB_CHECK(context.verdict == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(context.domain == rb_servo::FaultDomain::RobotState);
    RB_CHECK(context.backend_error.kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(context.robot_error_code == 2222);
    RB_CHECK(context.state_after.has_value());
    RB_CHECK(context.state_after_source == "response");
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
    RB_CHECK(context.state_after_source == "none");
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

bool testDualSendContextsPreserveBothArmFailures() {
    rb_servo::RobotState left_state_after;
    left_state_after.arm_id = rb_servo::ArmId::Left;
    left_state_after.connection_state = rb_servo::RobotConnectionState::Connected;
    left_state_after.has_valid_joint_state = true;
    left_state_after.has_error = true;
    left_state_after.error_code = 2222;

    rb_servo::DualSendResult dual;
    dual.left.arm_id = rb_servo::ArmId::Left;
    dual.left.request = request();
    dual.left.result = rb_servo::rejectedSend(
        dual.left.request,
        rb_servo::backendError(
            rb_servo::BackendErrorKind::RobotFault,
            "controller fault latched",
            "2222",
            "fault_latched"
        ),
        {},
        left_state_after,
        "response"
    );
    dual.right.arm_id = rb_servo::ArmId::Right;
    dual.right.request = request();
    dual.right.result = rb_servo::rejectedSend(
        dual.right.request,
        rb_servo::backendError(
            rb_servo::BackendErrorKind::TransportTimeout,
            "send timed out",
            "",
            "send_timeout"
        )
    );

    const rb_servo::LatchedDualFaultContext contexts =
        rb_servo::classifyDualSendResultContexts(dual);
    RB_CHECK(contexts.top_level.has_value());
    RB_CHECK(contexts.left.has_value());
    RB_CHECK(contexts.right.has_value());
    RB_CHECK(contexts.top_level->arm == rb_servo::ArmId::Left);
    RB_CHECK(contexts.top_level->backend_error.kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(contexts.left->verdict == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(contexts.left->domain == rb_servo::FaultDomain::RobotState);
    RB_CHECK(contexts.left->backend_error.kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(contexts.right->verdict == rb_servo::SafetyVerdict::SendFailure);
    RB_CHECK(contexts.right->domain == rb_servo::FaultDomain::Backend);
    RB_CHECK(contexts.right->backend_error.kind == rb_servo::BackendErrorKind::TransportTimeout);
    return true;
}

bool testDualSendContextsPreserveSuppressionBesideFailure() {
    rb_servo::DualSendResult dual;
    dual.left.arm_id = rb_servo::ArmId::Left;
    dual.left.request = request();
    dual.left.result = rb_servo::rejectedSend(
        dual.left.request,
        rb_servo::backendError(
            rb_servo::BackendErrorKind::SuppressedByPolicy,
            "motion gate closed",
            "",
            "rbpodo_motion_gate_closed"
        )
    );
    dual.right.arm_id = rb_servo::ArmId::Right;
    dual.right.request = request();
    dual.right.result = rb_servo::rejectedSend(
        dual.right.request,
        rb_servo::backendError(
            rb_servo::BackendErrorKind::TransportWriteFailed,
            "socket write failed",
            "",
            "send_failure_injected"
        )
    );

    const rb_servo::LatchedDualFaultContext contexts =
        rb_servo::classifyDualSendResultContexts(dual);
    RB_CHECK(contexts.top_level.has_value());
    RB_CHECK(contexts.left.has_value());
    RB_CHECK(contexts.right.has_value());
    RB_CHECK(contexts.top_level->arm == rb_servo::ArmId::Right);
    RB_CHECK(contexts.top_level->verdict == rb_servo::SafetyVerdict::SendFailure);
    RB_CHECK(contexts.left->verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(contexts.left->domain == rb_servo::FaultDomain::SafetyPolicy);
    RB_CHECK(contexts.left->suppress_regular_servo);
    RB_CHECK(contexts.right->backend_error.kind == rb_servo::BackendErrorKind::TransportWriteFailed);
    return true;
}

bool testDualSendContextsEmptyWhenBothArmsOk() {
    rb_servo::DualSendResult dual;
    dual.left.arm_id = rb_servo::ArmId::Left;
    dual.left.request = request();
    dual.left.result = rb_servo::acceptedSend(dual.left.request);
    dual.right.arm_id = rb_servo::ArmId::Right;
    dual.right.request = request();
    dual.right.result = rb_servo::acceptedSend(dual.right.request);

    const rb_servo::LatchedDualFaultContext contexts =
        rb_servo::classifyDualSendResultContexts(dual);
    RB_CHECK(!contexts.top_level.has_value());
    RB_CHECK(!contexts.left.has_value());
    RB_CHECK(!contexts.right.has_value());
    const rb_servo::FaultContext top = rb_servo::classifyDualSendResult(dual);
    RB_CHECK(top.verdict == rb_servo::SafetyVerdict::Ok);
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

// A servo_j request that expired before the worker dispatch is the client
// command aging out mid-tick -- the loop's own freshness gate produces a
// graceful stale-hold for the same event one tick later, so the classifier
// must NOT latch it (measured 2026-08-26: four runs ended by this 1-tick race
// at command age 149.7-149.9 / 150 ms). Every other CommandTimeout, and the
// lifecycle expiry, still fails.
bool testWorkerExpiredServoJIsNotAFault() {
    const rb_servo::SendServoJResult expired = rb_servo::rejectedSend(
        request(),
        rb_servo::backendError(
            rb_servo::BackendErrorKind::CommandTimeout,
            "servo_j request expired before arm worker dispatch",
            "",
            "arm_worker_command_expired"
        ),
        {},
        std::nullopt,
        "none"
    );
    const rb_servo::FaultContext graceful =
        rb_servo::classifySendServoJResult(expired, rb_servo::ArmId::Left);
    RB_CHECK(graceful.verdict == rb_servo::SafetyVerdict::Ok);

    const rb_servo::SendServoJResult other_timeout = rb_servo::rejectedSend(
        request(),
        rb_servo::backendError(
            rb_servo::BackendErrorKind::CommandTimeout,
            "controller ack timed out",
            "",
            "rbpodo_ack_timeout"
        ),
        {},
        std::nullopt,
        "none"
    );
    const rb_servo::FaultContext still_fails =
        rb_servo::classifySendServoJResult(other_timeout, rb_servo::ArmId::Left);
    RB_CHECK(still_fails.verdict == rb_servo::SafetyVerdict::SendFailure);
    return true;
}

int main() {
    if (!testFaultDomainToString()) return 1;
    if (!testWorkerExpiredServoJIsNotAFault()) return 1;
    if (!testRobotFaultSendIsRobotStateFault()) return 1;
    if (!testRobotFaultReadIsRobotStateFault()) return 1;
    if (!testTransportSendFailureStaysSendFailure()) return 1;
    if (!testSuppressedByPolicyIsNotFailure()) return 1;
    if (!testDualSendPrefersRealFailureOverPolicySuppression()) return 1;
    if (!testDualSendContextsPreserveBothArmFailures()) return 1;
    if (!testDualSendContextsPreserveSuppressionBesideFailure()) return 1;
    if (!testDualSendContextsEmptyWhenBothArmsOk()) return 1;
    if (!testCommandAndIkClassifiers()) return 1;
    return 0;
}
