#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

class MockBackend final : public IRobotBackend {
public:
    MockBackend(ArmId arm_id, const BackendConfig& config);

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
    ArmId arm_id_;
    BackendConfig config_;

    bool connected_ = false;
    bool initialized_ = false;

    JointArray q_actual_deg_{};
    JointArray q_target_deg_{};

    uint64_t last_update_time_ns_ = 0;
};

}  // namespace rb_servo
