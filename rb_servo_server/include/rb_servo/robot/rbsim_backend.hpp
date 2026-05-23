#pragma once

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

#include "rb_servo/config/config.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

class RbsimBackend final : public IRobotBackend {
public:
    RbsimBackend(ArmId arm_id, const BackendConfig& config);

    BackendResult<RobotState> connect() override;
    BackendResult<RobotState> initialize() override;

    BackendResult<RobotState> readState() override;
    SendServoJResult sendServoJ(const SendServoJRequest& request) override;

    BackendResult<RobotState> stop() override;
    BackendResult<RobotState> resetFault() override;

    bool isConnected() const override;
    ArmId armId() const override;
    std::string name() const override;

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
};

}  // namespace rb_servo
