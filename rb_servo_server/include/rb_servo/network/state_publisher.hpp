#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

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

    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace rb_servo
