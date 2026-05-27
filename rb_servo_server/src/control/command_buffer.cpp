#include "rb_servo/control/command_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {
namespace {
DualArmCommand makeHold(uint64_t now_ns) {
    DualArmCommand hold;
    hold.host_time_ns = now_ns;
    hold.left.arm_id = ArmId::Left;
    hold.right.arm_id = ArmId::Right;
    hold.left.mode = ControlMode::Hold;
    hold.right.mode = ControlMode::Hold;
    return hold;
}

bool validTimeout(double timeout_sec) {
    return timeout_sec > 0.0 && std::isfinite(timeout_sec);
}

bool isLifecycleMode(ControlMode mode) {
    return mode == ControlMode::ArmMotion ||
           mode == ControlMode::DisarmMotion ||
           mode == ControlMode::EmergencyStop ||
           mode == ControlMode::ResetFault;
}

bool isLifecycleCommand(const DualArmCommand& command) {
    return isLifecycleMode(command.left.mode) || isLifecycleMode(command.right.mode);
}

bool isUsableCommand(const DualArmCommand& command, uint64_t now_ns) {
    if (!validTimeout(command.left.timeout_sec) || !validTimeout(command.right.timeout_sec)) {
        return false;
    }
    const double timeout = std::min(command.left.timeout_sec, command.right.timeout_sec);
    const uint64_t timeout_ns = static_cast<uint64_t>(timeout * 1e9);
    return command.host_time_ns == 0 || now_ns <= command.host_time_ns + timeout_ns;
}
}  // namespace

void CommandBuffer::setCommand(const DualArmCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (isLifecycleCommand(command)) {
        constexpr size_t kMaxPendingLifecycleCommands = 16;
        if (pending_lifecycle_commands_.size() >= kMaxPendingLifecycleCommands) {
            pending_lifecycle_commands_.pop_front();
        }
        pending_lifecycle_commands_.push_back(command);
        latest_command_ = makeHold(command.host_time_ns);
    } else {
        latest_command_ = command;
    }
}

DualArmCommand CommandBuffer::latestOrHold(uint64_t now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!pending_lifecycle_commands_.empty()) {
        DualArmCommand command = pending_lifecycle_commands_.front();
        pending_lifecycle_commands_.pop_front();
        if (isUsableCommand(command, now_ns)) {
            return command;
        }
    }

    if (!latest_command_.has_value()) {
        return makeHold(now_ns);
    }

    DualArmCommand cmd = *latest_command_;
    if (!isUsableCommand(cmd, now_ns)) {
        return makeHold(now_ns);
    }
    return cmd;
}

}  // namespace rb_servo
