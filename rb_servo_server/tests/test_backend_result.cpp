#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rb_servo/robot/backend_result.hpp"
#include "rb_servo/robot/rbpodo_backend.hpp"

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
    RB_CHECK(accepted.ack_policy == rb_servo::BackendAckPolicy::BackendDefault);
    RB_CHECK(!accepted.ack_observed);
    RB_CHECK(!accepted.controller_acceptance_observed);
    RB_CHECK(accepted.ack_wait_duration_us == 0.0);
    RB_CHECK(!accepted.rbpodo_waiting_ack);
    RB_CHECK(accepted.acceptance_semantics == "unknown");

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
    RB_CHECK(rejected.acceptance_semantics == "unknown");
    return true;
}

rb_servo::BackendConfig rbpodoConfig(std::string operation_mode = "real") {
    rb_servo::BackendConfig config;
    config.backend_type = rb_servo::BackendType::Rbpodo;
    config.run_mode = rb_servo::RunMode::Real;
    config.operation_mode = std::move(operation_mode);
    return config;
}

rb_servo::RbpodoSystemStateSnapshot rbpodoSnapshot() {
    rb_servo::RbpodoSystemStateSnapshot snapshot;
    snapshot.q_actual_deg = {1.0, -2.0, 3.0, -4.0, 5.0, -6.0};
    snapshot.q_target_deg = snapshot.q_actual_deg;
    snapshot.robot_time_sec = 12.5;
    snapshot.real_vs_simulation_mode = 0;
    snapshot.init_state_info = 6;
    return snapshot;
}

bool testRbpodoReadStateAcceptsServoDisabledJointFeedback() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.init_state_info = 4;

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.connection_state == rb_servo::RobotConnectionState::Connected);
    RB_CHECK(!state.servo_enabled);
    RB_CHECK(!state.has_error);
    RB_CHECK(state.lifecycle_state == "connected_not_motion_ready");
    RB_CHECK(!rb_servo::rbpodoStateAcquisitionError(state).has_value());
    const rb_servo::BackendResult<rb_servo::RobotState> read_result = rb_servo::okReadState(state);
    RB_CHECK(read_result.ok);
    RB_CHECK(!read_result.value.servo_enabled);

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::ServoDisabled);
    RB_CHECK(readiness->name == "rbpodo_servo_disabled");
    return true;
}

bool testRbpodoServoDisabledSendRejectsWithoutTransportWriteFailure() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.init_state_info = 4;
    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Right, snapshot);
    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());

    rb_servo::SendServoJRequest request;
    request.command_seq = 77;
    request.q_target_deg = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    const rb_servo::SendServoJResult result =
        rb_servo::rejectedSend(request, *readiness, {}, state, "cache");
    RB_CHECK(!result.accepted);
    RB_CHECK(result.error.kind == rb_servo::BackendErrorKind::ServoDisabled);
    RB_CHECK(result.error.kind != rb_servo::BackendErrorKind::TransportWriteFailed);
    RB_CHECK(result.state_after.has_value());
    RB_CHECK(!result.state_after->servo_enabled);
    RB_CHECK(result.state_after_source == "cache");
    return true;
}

bool testRbpodoFaultedJointFeedbackIsReadableButNotMotionReady() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.op_stat_collision_occur = 1234;

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.has_error);
    // op_stat_collision_occur = 1234 has low 2 bits = 2 (spec: only lower 2 bits valid,
    // value 0/1). That is malformed, NOT an external collision (which is low-bits == 1,
    // caught by the masked check). The raw-value collision fallback was removed (it only
    // false-positived on reserved bits), so a malformed value now surfaces via the generic
    // diagnostics_suspect path: error_code = kRbpodoDiagnosticsSuspectCode (-2001), lifecycle
    // "diagnostics_suspect". Still readable, still fault-closed (not motion-ready).
    RB_CHECK(state.error_code == -2001);
    RB_CHECK(state.lifecycle_state == "diagnostics_suspect");
    RB_CHECK(!rb_servo::rbpodoStateAcquisitionError(state).has_value());

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::RobotFault);
    return true;
}

bool testRbpodoWrongModeIsMotionReadinessNotReadFailure() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.real_vs_simulation_mode = 1;

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.servo_enabled);
    RB_CHECK(!rb_servo::rbpodoStateAcquisitionError(state).has_value());

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig("real"), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::WrongMode);
    RB_CHECK(readiness->name == "rbpodo_wrong_operation_mode");
    return true;
}

bool testRbpodoInvalidJointStateFailsAcquisition() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.q_actual_deg[2] = std::numeric_limits<double>::quiet_NaN();

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(!state.has_valid_joint_state);

    const std::optional<rb_servo::BackendError> acquisition =
        rb_servo::rbpodoStateAcquisitionError(state);
    RB_CHECK(acquisition.has_value());
    RB_CHECK(acquisition->kind == rb_servo::BackendErrorKind::InvalidJointState);

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::InvalidJointState);
    return true;
}

}  // namespace

int main() {
    if (!testToStringMappings()) return 1;
    if (!testTimingDuration()) return 1;
    if (!testErrorFlags()) return 1;
    if (!testReadStateHelpers()) return 1;
    if (!testSendHelpersKeepStateAfterExplicit()) return 1;
    if (!testRbpodoReadStateAcceptsServoDisabledJointFeedback()) return 1;
    if (!testRbpodoServoDisabledSendRejectsWithoutTransportWriteFailure()) return 1;
    if (!testRbpodoFaultedJointFeedbackIsReadableButNotMotionReady()) return 1;
    if (!testRbpodoWrongModeIsMotionReadinessNotReadFailure()) return 1;
    if (!testRbpodoInvalidJointStateFailsAcquisition()) return 1;
    return 0;
}
