#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

class MockBackend final : public IRobotBackend {
public:
    MockBackend(ArmId arm_id, const BackendConfig& config);

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
    ArmId arm_id_;
    BackendConfig config_;

    bool connected_ = false;
    bool initialized_ = false;

    JointArray q_actual_deg_{};
    JointArray q_target_deg_{};

    uint64_t last_update_time_ns_ = 0;
};

}  // namespace rb_servo
