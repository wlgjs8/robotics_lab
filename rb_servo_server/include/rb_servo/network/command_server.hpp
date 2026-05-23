#pragma once

#include <atomic>
#include <future>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/command_buffer.hpp"

namespace rb_servo {

class CommandServer {
public:
    CommandServer(
        const NetworkConfig& config,
        CommandBuffer* command_buffer
    );

    ~CommandServer();

    bool start();
    void stop();

    bool parseMessage(
        const std::string& message,
        uint64_t receive_time_ns,
        DualArmCommand* out_command
    );
    bool acceptsCommandSource(const std::string& source_ip) const;
    std::string lastRejectReason() const;

private:
    void threadMain(std::promise<bool> startup_result);
    CommandSourceLeaseState currentLeaseState(uint64_t now_ns) const;

private:
    NetworkConfig config_;
    CommandSourceConfig command_source_config_;
    CommandBuffer* command_buffer_ = nullptr;
    std::unordered_map<std::string, uint64_t> last_accepted_seq_by_source_;
    CommandSourceLeaseState active_lease_;
    uint64_t lease_counter_ = 0;
    std::string last_reject_reason_;

    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace rb_servo
