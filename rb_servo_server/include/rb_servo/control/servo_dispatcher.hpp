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

    // Withhold this arm's servo_j for this tick, dropping its box command queue
    // by exactly one. Set by BoxQueueCadence's per-arm level trim; see
    // box_queue_cadence.hpp for why this is safe and why it must drop a waypoint
    // rather than hold the trajectory back.
    //
    // Honoured by dispatchDirectSequential and dispatchRbpodoAsync. Direct is the
    // one that matters: stack_real.yaml runs io_model: direct with
    // rbpodo_async_streaming disabled (config validation rejects async in real
    // mode), so real motion never reaches the async path. An earlier version of
    // this comment claimed only the async path streams; it shipped a level trim
    // that incremented its counters 2791 times in a 64 s run and withheld exactly
    // zero sends. If a new dispatch path is added, honour these there too.
    bool skip_left = false;
    bool skip_right = false;
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
