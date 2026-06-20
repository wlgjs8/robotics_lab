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

    explicit StatePublisher(const DualArmConfig& config, SnapshotProvider provider = {});
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

    mutable std::mutex snapshot_mutex_;
    ServoSnapshot latest_snapshot_;

    // Out-of-process gripper bridge (null when gripper.enable=false). Forwarded
    // each publish cycle and stamped into the per-arm state JSON. All gripper I/O
    // runs on this publisher thread + the bridge's own receive thread, never the
    // RT control loop.
    std::unique_ptr<GripperBridge> gripper_bridge_;

    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace rb_servo
