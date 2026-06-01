#pragma once

#include <cstdint>

#include "rb_servo/control/arm_worker.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

struct ServoDispatchRequest {
    SendServoJRequest left;
    SendServoJRequest right;
    uint64_t seq = 0;
    uint64_t dispatch_start_ns = 0;
    uint64_t deadline_ns = 0;
};

class ServoDispatcher {
public:
    static DualSendResult dispatchDirectSequential(
        IRobotBackend& left,
        IRobotBackend& right,
        const ServoDispatchRequest& request
    );

    static DualSendResult dispatchWorker(
        ArmWorker& left,
        ArmWorker& right,
        const ServoDispatchRequest& request
    );

    static DualSendResult dispatchRbpodoAsync(
        ArmWorker& left,
        ArmWorker& right,
        const ServoDispatchRequest& request
    );

    static bool armDeadlineMissed(const ArmSendResult& result);
    static bool deadlineMissed(const DualSendResult& result);
};

}  // namespace rb_servo
