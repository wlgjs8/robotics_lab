#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

struct ScopeSample {
    uint64_t t_host_ns = 0;
    uint64_t l_robot_ns = 0;
    uint64_t r_robot_ns = 0;
    JointArray l_sent{};
    JointArray r_sent{};
    JointArray l_ref{};
    JointArray r_ref{};
    JointArray l_actual{};
    JointArray r_actual{};
};

class ScopePublisher {
public:
    ScopePublisher(const ScopeConfig& scope_config, const NetworkConfig& network_config);
    ~ScopePublisher();

    bool start();
    void stop();

    void push(const ScopeSample& sample);
    uint64_t droppedSamples() const;

    static std::string serializeBatch(const std::vector<ScopeSample>& samples);

private:
    void threadMain();
    std::vector<ScopeSample> drainPending();

private:
    ScopeConfig scope_config_;
    NetworkConfig network_config_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<ScopeSample> ring_;
    size_t head_ = 0;
    size_t size_ = 0;
    std::atomic<uint64_t> dropped_samples_{0};
};

}  // namespace rb_servo
