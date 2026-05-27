#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "rb_servo/config/config.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

struct RbscriptTcpTransportCounters {
    uint64_t connections_opened_total = 0;
    uint64_t reconnects_total = 0;
    uint64_t command_send_total = 0;
    uint64_t ack_success_total = 0;
    uint64_t ack_error_total = 0;
    uint64_t connect_failures_total = 0;
    uint64_t connect_attempts_suppressed_total = 0;
    uint64_t connect_attempts_total = 0;
    uint64_t read_syscalls_total = 0;
    uint64_t write_syscalls_total = 0;
    uint64_t data_requests_total = 0;
    uint64_t data_success_total = 0;
    uint64_t data_parse_failures_total = 0;
    uint64_t data_timeouts_total = 0;
    std::string last_connect_error_name;
    std::string last_connect_error_message;
    uint64_t next_connect_attempt_ns = 0;
    uint64_t next_connect_attempt_delay_ms = 0;
    std::optional<BackendErrorKind> last_transport_error_kind;
};

struct RbscriptDataState {
    JointArray q_actual_deg{};
    JointArray q_target_deg{};
    bool has_q_target_deg = false;
    uint64_t robot_time_ns = 0;
    bool servo_enabled = false;
    bool has_error = false;
    int error_code = 0;
    std::string lifecycle_state;
};

struct RbscriptDataParseResult {
    bool ok = false;
    RbscriptDataState state;
    BackendError error;
};

std::string formatRbscriptServoJ(const JointArray& q_target_deg, const BackendConfig& config);
std::string formatRbscriptReqdata();
RbscriptDataParseResult parseRbscriptDataPayload(const std::string& payload);

class RbscriptTcpBackend final : public IRobotBackend {
public:
    RbscriptTcpBackend(ArmId arm_id, const BackendConfig& config);
    ~RbscriptTcpBackend() override;

    BackendResult<RobotState> connect() override;
    BackendResult<RobotState> initialize() override;

    BackendResult<RobotState> readState() override;
    SendServoJResult sendServoJ(const SendServoJRequest& request) override;

    BackendResult<RobotState> stop() override;
    BackendResult<RobotState> resetFault() override;

    bool isConnected() const override;
    ArmId armId() const override;
    std::string name() const override;
    std::optional<BackendTransportTelemetry> transportTelemetry() const override;
    RbscriptTcpTransportCounters transportCounters() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rb_servo
