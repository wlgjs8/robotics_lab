#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "rb_servo/robot/backend_result.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

bool testToStringMappings() {
    const std::vector<std::pair<rb_servo::BackendOp, std::string>> ops = {
        {rb_servo::BackendOp::Connect, "Connect"},
        {rb_servo::BackendOp::Initialize, "Initialize"},
        {rb_servo::BackendOp::ReadState, "ReadState"},
        {rb_servo::BackendOp::SendServoJ, "SendServoJ"},
        {rb_servo::BackendOp::Stop, "Stop"},
        {rb_servo::BackendOp::ResetFault, "ResetFault"},
    };
    for (const auto& item : ops) {
        RB_CHECK(rb_servo::toString(item.first) == item.second);
    }

    const std::vector<std::pair<rb_servo::BackendErrorKind, std::string>> kinds = {
        {rb_servo::BackendErrorKind::None, "None"},
        {rb_servo::BackendErrorKind::TransportConnectFailed, "TransportConnectFailed"},
        {rb_servo::BackendErrorKind::TransportWriteFailed, "TransportWriteFailed"},
        {rb_servo::BackendErrorKind::TransportReadFailed, "TransportReadFailed"},
        {rb_servo::BackendErrorKind::TransportTimeout, "TransportTimeout"},
        {rb_servo::BackendErrorKind::ProtocolError, "ProtocolError"},
        {rb_servo::BackendErrorKind::UnsupportedSchema, "UnsupportedSchema"},
        {rb_servo::BackendErrorKind::WrongArm, "WrongArm"},
        {rb_servo::BackendErrorKind::WrongEndpoint, "WrongEndpoint"},
        {rb_servo::BackendErrorKind::UnknownArm, "UnknownArm"},
        {rb_servo::BackendErrorKind::RobotDisconnected, "RobotDisconnected"},
        {rb_servo::BackendErrorKind::RobotNotInitialized, "RobotNotInitialized"},
        {rb_servo::BackendErrorKind::ServoDisabled, "ServoDisabled"},
        {rb_servo::BackendErrorKind::WrongMode, "WrongMode"},
        {rb_servo::BackendErrorKind::RobotFault, "RobotFault"},
        {rb_servo::BackendErrorKind::InvalidJointState, "InvalidJointState"},
        {rb_servo::BackendErrorKind::InvalidTarget, "InvalidTarget"},
        {rb_servo::BackendErrorKind::ControllerRejected, "ControllerRejected"},
        {rb_servo::BackendErrorKind::CommandTimeout, "CommandTimeout"},
        {rb_servo::BackendErrorKind::DependencyUnavailable, "DependencyUnavailable"},
        {rb_servo::BackendErrorKind::SuppressedByPolicy, "SuppressedByPolicy"},
        {rb_servo::BackendErrorKind::Unknown, "Unknown"},
    };
    for (const auto& item : kinds) {
        RB_CHECK(rb_servo::toString(item.first) == item.second);
    }
    return true;
}

bool testTimingDuration() {
    const rb_servo::BackendTiming timing = rb_servo::makeBackendTiming(1000, 3500);
    RB_CHECK(timing.start_ns == 1000);
    RB_CHECK(timing.end_ns == 3500);
    RB_CHECK(std::abs(timing.duration_us - 2.5) < 1e-12);

    const rb_servo::BackendTiming reversed = rb_servo::makeBackendTiming(3500, 1000);
    RB_CHECK(reversed.duration_us == 0.0);
    return true;
}

bool testErrorFlags() {
    const rb_servo::BackendError robot_fault =
        rb_servo::backendError(rb_servo::BackendErrorKind::RobotFault, "robot reported a fault");
    RB_CHECK(robot_fault.robot_fault);
    RB_CHECK(!robot_fault.transport_fault);
    RB_CHECK(!robot_fault.retryable);
    RB_CHECK(robot_fault.recoverable);

    const rb_servo::BackendError timeout =
        rb_servo::backendError(rb_servo::BackendErrorKind::TransportTimeout, "read timed out");
    RB_CHECK(!timeout.robot_fault);
    RB_CHECK(timeout.transport_fault);
    RB_CHECK(timeout.retryable);
    RB_CHECK(timeout.recoverable);

    const rb_servo::BackendError suppressed =
        rb_servo::backendError(rb_servo::BackendErrorKind::SuppressedByPolicy, "real motion gate closed");
    RB_CHECK(!suppressed.robot_fault);
    RB_CHECK(!suppressed.transport_fault);
    RB_CHECK(!suppressed.retryable);
    RB_CHECK(suppressed.recoverable);
    return true;
}

bool testReadStateHelpers() {
    rb_servo::RobotState state;
    state.arm_id = rb_servo::ArmId::Right;
    state.has_valid_joint_state = true;

    const rb_servo::BackendTiming timing = rb_servo::makeBackendTiming(10, 2010);
    const rb_servo::BackendResult<rb_servo::RobotState> ok = rb_servo::okReadState(state, timing);
    RB_CHECK(ok.ok);
    RB_CHECK(ok.op == rb_servo::BackendOp::ReadState);
    RB_CHECK(ok.value.arm_id == rb_servo::ArmId::Right);
    RB_CHECK(ok.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(std::abs(ok.timing.duration_us - 2.0) < 1e-12);

    const rb_servo::BackendError error =
        rb_servo::backendError(rb_servo::BackendErrorKind::TransportReadFailed, "socket read failed");
    const rb_servo::BackendResult<rb_servo::RobotState> failed = rb_servo::failedReadState(error, timing);
    RB_CHECK(!failed.ok);
    RB_CHECK(failed.op == rb_servo::BackendOp::ReadState);
    RB_CHECK(failed.error.kind == rb_servo::BackendErrorKind::TransportReadFailed);
    RB_CHECK(failed.error.transport_fault);
    return true;
}

bool testSendHelpersKeepStateAfterExplicit() {
    rb_servo::SendServoJRequest request;
    request.command_seq = 42;
    request.q_target_deg = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    rb_servo::RobotState state_after;
    state_after.arm_id = rb_servo::ArmId::Left;
    state_after.q_target_deg = request.q_target_deg;

    const rb_servo::SendServoJResult accepted = rb_servo::acceptedSend(
        request,
        rb_servo::makeBackendTiming(100, 2100),
        state_after,
        "response"
    );
    RB_CHECK(accepted.accepted);
    RB_CHECK(accepted.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(accepted.state_after.has_value());
    RB_CHECK(accepted.state_after_source == "response");
    RB_CHECK(accepted.requested_q_deg[5] == 6.0);

    const rb_servo::SendServoJResult rejected = rb_servo::rejectedSend(
        request,
        rb_servo::backendError(rb_servo::BackendErrorKind::SuppressedByPolicy, "motion disabled by gate")
    );
    RB_CHECK(!rejected.accepted);
    RB_CHECK(rejected.error.kind == rb_servo::BackendErrorKind::SuppressedByPolicy);
    RB_CHECK(!rejected.error.transport_fault);
    RB_CHECK(!rejected.error.robot_fault);
    RB_CHECK(!rejected.state_after.has_value());
    RB_CHECK(rejected.state_after_source == "none");
    RB_CHECK(rejected.requested_q_deg[0] == 1.0);
    return true;
}

}  // namespace

int main() {
    if (!testToStringMappings()) return 1;
    if (!testTimingDuration()) return 1;
    if (!testErrorFlags()) return 1;
    if (!testReadStateHelpers()) return 1;
    if (!testSendHelpersKeepStateAfterExplicit()) return 1;
    return 0;
}
