#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "rb_servo/core/types.hpp"

namespace rb_servo {

enum class BackendOp {
    Connect,
    Initialize,
    ReadState,
    SendServoJ,
    Stop,
    ResetFault,
    SetFreedrive
};

enum class BackendErrorKind {
    None,
    TransportConnectFailed,
    TransportWriteFailed,
    TransportReadFailed,
    TransportTimeout,
    ProtocolError,
    UnsupportedSchema,
    WrongArm,
    WrongEndpoint,
    UnknownArm,
    RobotDisconnected,
    RobotNotInitialized,
    ServoDisabled,
    WrongMode,
    RobotFault,
    InvalidJointState,
    InvalidTarget,
    ControllerRejected,
    CommandTimeout,
    DependencyUnavailable,
    SuppressedByPolicy,
    Unknown
};

struct BackendError {
    BackendErrorKind kind = BackendErrorKind::None;
    std::string code;
    std::string name = "None";
    std::string message;
    bool retryable = false;
    bool recoverable = false;
    bool robot_fault = false;
    bool transport_fault = false;
};

struct BackendTiming {
    uint64_t start_ns = 0;
    uint64_t end_ns = 0;
    double duration_us = 0.0;
};

template <typename T>
struct BackendResult {
    bool ok = false;
    BackendOp op = BackendOp::ReadState;
    T value{};
    BackendError error;
    BackendTiming timing;
    BackendReadExchangeTiming read_exchange_timing;
};

struct SendServoJRequest {
    JointArray q_target_deg{};
    uint64_t command_seq = 0;
    uint64_t host_time_ns = 0;
    uint64_t deadline_ns = 0;
};

struct SendServoJResult {
    bool accepted = false;
    BackendError error;
    BackendTiming timing;
    std::optional<RobotState> state_after;
    std::string state_after_source = "none";
    BackendAckPolicy ack_policy = BackendAckPolicy::BackendDefault;
    bool ack_observed = false;
    bool controller_acceptance_observed = false;
    double ack_wait_duration_us = 0.0;
    bool rbpodo_waiting_ack = false;
    std::string acceptance_semantics = "unknown";
    JointArray requested_q_deg{};
    RbpodoQueueAckTelemetry queue_ack;
};

struct ArmSendResult {
    ArmId arm_id = ArmId::Left;
    SendServoJRequest request;
    SendServoJResult result;
    BackendTiming dispatch_timing;
};

struct DualSendResult {
    ArmSendResult left;
    ArmSendResult right;
    BackendTiming timing;
    uint64_t dispatch_start_ns = 0;
    uint64_t dispatch_end_ns = 0;
    double left_right_start_skew_us = 0.0;
    double left_right_end_skew_us = 0.0;

    bool any_transport_failure() const;
    bool any_robot_fault() const;
};

std::string toString(BackendOp op);
std::string toString(BackendErrorKind kind);

BackendError noBackendError();
BackendError backendError(
    BackendErrorKind kind,
    std::string message,
    std::string code = "",
    std::string name = "",
    std::optional<bool> retryable = std::nullopt,
    std::optional<bool> recoverable = std::nullopt,
    std::optional<bool> robot_fault = std::nullopt,
    std::optional<bool> transport_fault = std::nullopt
);
BackendTiming makeBackendTiming(uint64_t start_ns, uint64_t end_ns);

BackendResult<RobotState> okReadState(
    const RobotState& state,
    const BackendTiming& timing = BackendTiming{}
);
BackendResult<RobotState> failedReadState(
    const BackendError& error,
    const BackendTiming& timing = BackendTiming{}
);

SendServoJResult acceptedSend(
    const SendServoJRequest& request,
    const BackendTiming& timing = BackendTiming{},
    std::optional<RobotState> state_after = std::nullopt,
    std::string state_after_source = "none"
);
SendServoJResult rejectedSend(
    const SendServoJRequest& request,
    const BackendError& error,
    const BackendTiming& timing = BackendTiming{},
    std::optional<RobotState> state_after = std::nullopt,
    std::string state_after_source = "none"
);

}  // namespace rb_servo
