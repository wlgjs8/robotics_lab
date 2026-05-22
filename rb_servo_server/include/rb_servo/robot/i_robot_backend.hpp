#pragma once

#include <string>
#include "rb_servo/core/types.hpp"

namespace rb_servo {

class IRobotBackend {
public:
    virtual ~IRobotBackend() = default;

    virtual bool connect() = 0;
    virtual bool initialize() = 0;

    virtual bool readState(RobotState& out_state) = 0;
    virtual bool sendServoJ(const JointArray& q_target_deg) = 0;

    virtual bool stop() = 0;
    virtual bool resetFault() = 0;

    virtual bool isConnected() const = 0;
    virtual ArmId armId() const = 0;
    virtual std::string name() const = 0;
};

}  // namespace rb_servo
