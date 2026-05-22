#pragma once

#include <memory>
#include <string>

#include "rb_servo/config/config.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

class RbpodoBackend final : public IRobotBackend {
public:
    RbpodoBackend(ArmId arm_id, const BackendConfig& config);
    ~RbpodoBackend() override;

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
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rb_servo
