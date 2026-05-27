#pragma once

#include <deque>
#include <mutex>
#include <optional>

#include "rb_servo/core/types.hpp"

namespace rb_servo {

class CommandBuffer {
public:
    void setCommand(const DualArmCommand& command);
    DualArmCommand latestOrHold(uint64_t now_ns);

private:
    mutable std::mutex mutex_;
    std::optional<DualArmCommand> latest_command_;
    std::deque<DualArmCommand> pending_lifecycle_commands_;
};

}  // namespace rb_servo
