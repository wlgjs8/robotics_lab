#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include "rb_servo/config/config.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

class JsonLineTcpClient;

struct RbsimTransportCounters {
    uint64_t connect_attempts_total = 0;
    uint64_t connect_failures_total = 0;
    uint64_t connect_attempts_suppressed_total = 0;
    uint64_t connections_opened_total = 0;
    uint64_t reconnects_total = 0;
    uint64_t requests_total = 0;
    uint64_t read_syscalls_total = 0;
    uint64_t write_syscalls_total = 0;
    std::string last_connect_error_name;
    std::string last_connect_error_message;
    uint64_t next_connect_attempt_ns = 0;
    uint64_t next_connect_attempt_delay_ms = 0;
    std::optional<BackendErrorKind> last_transport_error_kind;
};

class RbsimBackend final : public IRobotBackend {
public:
    RbsimBackend(ArmId arm_id, const BackendConfig& config);
    ~RbsimBackend() override;

    BackendResult<RobotState> connect() override;
    BackendResult<RobotState> initialize() override;

    BackendResult<RobotState> readState() override;
    SendServoJResult sendServoJ(const SendServoJRequest& request) override;

    BackendResult<RobotState> stop() override;
    BackendResult<RobotState> resetFault() override;

    bool isConnected() const override;
    ArmId armId() const override;
    std::string name() const override;
    RbsimTransportCounters transportCounters() const;

private:
    BackendResult<RobotState> controlRequest(
        const std::string& op,
        BackendOp backend_op,
        const nlohmann::json& params,
        bool require_state
    );

    ArmId arm_id_;
    BackendConfig config_;
    bool connected_ = false;
    uint64_t request_seq_ = 0;
    std::unique_ptr<JsonLineTcpClient> client_;
};

}  // namespace rb_servo
