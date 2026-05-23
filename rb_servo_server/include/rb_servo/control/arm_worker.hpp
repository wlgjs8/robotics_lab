#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

struct ArmWorkerOptions {
    uint64_t read_period_ns = 1'000'000;
};

class ArmWorker {
public:
    explicit ArmWorker(std::unique_ptr<IRobotBackend> backend, ArmWorkerOptions options = {});
    ~ArmWorker();

    ArmWorker(const ArmWorker&) = delete;
    ArmWorker& operator=(const ArmWorker&) = delete;
    ArmWorker(ArmWorker&&) = delete;
    ArmWorker& operator=(ArmWorker&&) = delete;

    bool start();
    void stop();

    BackendResult<RobotState> latestState(uint64_t max_age_ns) const;
    void enqueueServoJ(SendServoJRequest request);
    std::optional<ArmSendResult> lastSendResult() const;

    ArmId armId() const;
    std::string name() const;

private:
    void run();
    void storeReadResult(const BackendResult<RobotState>& result, uint64_t observed_ns);
    void storeSendResult(
        const SendServoJRequest& request,
        const SendServoJResult& result,
        const BackendTiming& dispatch_timing
    );
    void storeSendResultLocked(
        const SendServoJRequest& request,
        const SendServoJResult& result,
        const BackendTiming& dispatch_timing
    );
    bool isExpired(const SendServoJRequest& request, uint64_t now_ns) const;
    SendServoJResult expiredResult(const SendServoJRequest& request, uint64_t now_ns) const;
    SendServoJResult notRunningResult(const SendServoJRequest& request, uint64_t now_ns) const;

    std::unique_ptr<IRobotBackend> backend_;
    ArmWorkerOptions options_;
    ArmId arm_id_;
    std::string name_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_requested_ = false;
    bool running_ = false;
    std::optional<SendServoJRequest> pending_servo_j_;
    std::optional<BackendResult<RobotState>> latest_state_;
    uint64_t latest_state_observed_ns_ = 0;
    std::optional<ArmSendResult> last_send_result_;

    std::thread thread_;
};

}  // namespace rb_servo
