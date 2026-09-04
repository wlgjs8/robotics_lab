#pragma once

#include <string>
#include <vector>
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

    // Per-arm direct-teaching (free-drive). on=true releases servo_j control so
    // the arm can be hand-guided (controller freedrive_teach_on); on=false re-
    // acquires it. The default is a benign no-op success for hardware-free
    // backends (mock/simulator) so the server-side sticky state machine and GUI
    // can be exercised; real controllers (RbpodoBackend) override it.
    virtual BackendResult<RobotState> setFreedrive(bool /*on*/) {
        BackendResult<RobotState> result;
        result.ok = true;
        result.op = BackendOp::SetFreedrive;
        return result;
    }

    virtual bool isConnected() const = 0;
    virtual ArmId armId() const = 0;
    virtual std::string name() const = 0;
    virtual std::optional<BackendTransportTelemetry> transportTelemetry() const { return std::nullopt; }
    // The controller's own link-parameter (calibrated DH) table if the backend
    // read one at connect; empty for hardware-free backends. Logging only.
    virtual std::vector<double> boxLinkParameter() const { return {}; }
};

}  // namespace rb_servo
