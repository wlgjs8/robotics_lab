#pragma once

#include <deque>
#include <mutex>
#include <optional>

#include "rb_servo/core/types.hpp"

namespace rb_servo {

class CommandBuffer {
public:
    void setCommand(const DualArmCommand& command);
    // Lease-admin packets (AcquireLease/ReleaseLease) must not displace the
    // buffered motion command, but their lease grant/clear still has to reach
    // the published state (the lease readback is snapshot.command.lease).
    // Update only the lease snapshot of the buffered command, synthesizing a
    // non-expiring Hold when the buffer is empty (acquire at startup) or the
    // buffered command has already EXPIRED at now_ns (re-acquire after idle
    // teleop) — an expired carrier would hide the grant from the readback.
    void updateLease(const CommandSourceLeaseState& lease, uint64_t now_ns);
    DualArmCommand latestOrHold(uint64_t now_ns);

private:
    mutable std::mutex mutex_;
    std::optional<DualArmCommand> latest_command_;
    std::deque<DualArmCommand> pending_lifecycle_commands_;
};

}  // namespace rb_servo
