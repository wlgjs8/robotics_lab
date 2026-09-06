#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"
#include "rb_servo/network/gripper_bridge.hpp"

namespace rb_servo {

class StatePublisher {
public:
    using SnapshotProvider = std::function<ServoSnapshot()>;
    // Sink for the latest per-arm gripper feedback (open percent + validity), pushed
    // each publish cycle so the control loop's safety gate can interpolate the TCP
    // fingertip offset points. Non-blocking on the consumer side (atomic store).
    using GripperFeedbackSink = std::function<void(ArmId, double percent, bool valid)>;

    explicit StatePublisher(const DualArmConfig& config, SnapshotProvider provider = {},
                            GripperFeedbackSink gripper_sink = {});
    explicit StatePublisher(const NetworkConfig& config);
    ~StatePublisher();

    void updateSnapshot(const ServoSnapshot& snapshot);
    std::string serializeSnapshot(const ServoSnapshot& snapshot) const;

    bool start();
    void stop();

    static bool parseUdpEndpointUri(const std::string& endpoint, std::string* host, int* port);

private:
    void threadMain();
    bool parseEndpoint(std::string* host, int* port) const;

private:
    DualArmConfig config_;
    SnapshotProvider snapshot_provider_;
    GripperFeedbackSink gripper_feedback_sink_;

    mutable std::mutex snapshot_mutex_;
    ServoSnapshot latest_snapshot_;

    // Out-of-process gripper bridge (null when gripper.enable=false). Forwarded
    // each publish cycle and stamped into the per-arm state JSON. All gripper I/O
    // runs on this publisher thread + the bridge's own receive thread, never the
    // RT control loop.
    std::unique_ptr<GripperBridge> gripper_bridge_;

    // Publisher-thread failures, copied into later successful state packets.
    // Atomics also permit passive serializeSnapshot callers while publishing.
    std::atomic<uint64_t> publication_oversize_dropped_total_{0};
    std::atomic<uint64_t> publication_send_errors_total_{0};
    std::atomic<uint64_t> publication_last_error_time_ns_{0};
    std::atomic<int> publication_last_error_code_{0};

    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace rb_servo
