#pragma once

#include <string>
#include "rb_servo/core/types.hpp"
#include "rb_servo/robot/backend_result.hpp"

namespace rb_servo {

class IRobotBackend {
public:
    virtual ~IRobotBackend() = default;

    virtual BackendResult<RobotState> connect() = 0;
    virtual BackendResult<RobotState> initialize() = 0;

    virtual BackendResult<RobotState> readState() = 0;
    virtual SendServoJResult sendServoJ(const SendServoJRequest& request) = 0;

    virtual BackendResult<RobotState> stop() = 0;
    virtual BackendResult<RobotState> resetFault() = 0;

    virtual bool isConnected() const = 0;
    virtual ArmId armId() const = 0;
    virtual std::string name() const = 0;
    virtual std::optional<BackendTransportTelemetry> transportTelemetry() const { return std::nullopt; }
};

}  // namespace rb_servo
