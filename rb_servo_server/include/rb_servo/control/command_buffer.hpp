#pragma once

#include <atomic>
#include <cstdint>
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
    DualArmCommand latestOrHold(
        uint64_t now_ns,
        CommandBufferReadTelemetry* telemetry = nullptr
    );
    std::optional<DualArmCommand> consumeLatestExternalBoxes(
        uint64_t now_ns,
        CommandBufferReadTelemetry* telemetry = nullptr
    );

    // External-box keep-out feed liveness. Stamped on the network RECEIVE thread
    // for every accepted SetExternalBoxes packet. The box payload itself uses a
    // side slot so it cannot displace motion; this receive stamp is still kept
    // separately so the servo-side liveness watchdog measures true producer
    // aliveness. Read on the control thread. 0 means no feed received yet.
    void noteExternalBoxReceived(uint64_t receive_time_ns);
    uint64_t lastExternalBoxReceiveNs() const;

private:
    mutable std::mutex mutex_;
    std::optional<DualArmCommand> latest_command_;
    std::optional<DualArmCommand> pending_external_boxes_command_;
    std::deque<DualArmCommand> pending_lifecycle_commands_;
    std::atomic<uint64_t> last_external_box_receive_ns_{0};
};

}  // namespace rb_servo
