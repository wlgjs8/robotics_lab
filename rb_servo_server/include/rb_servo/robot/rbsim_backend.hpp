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

    bool connect() override;
    bool initialize() override;

    bool readState(RobotState& out_state) override;
    bool sendServoJ(const JointArray& q_target_deg) override;

    bool stop() override;
    bool resetFault() override;

    bool isConnected() const override;
    ArmId armId() const override;
    std::string name() const override;

private:
    bool controlRequest(const std::string& op, const nlohmann::json& params, RobotState* out_state);

    ArmId arm_id_;
    BackendConfig config_;
    bool connected_ = false;
    uint64_t request_seq_ = 0;
};

}  // namespace rb_servo
