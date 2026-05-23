#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

struct ArmWorkerOptions {
    uint64_t read_period_ns = 1'000'000;
    std::size_t lifecycle_queue_capacity = 4;
};

enum class ArmWorkerCommandKind {
    ServoJ,
    ResetFault,
    Stop
};

struct ArmWorkerCommand {
    ArmWorkerCommandKind kind = ArmWorkerCommandKind::ServoJ;
    SendServoJRequest servo_j;
    uint64_t command_seq = 0;
    uint64_t host_time_ns = 0;
    uint64_t deadline_ns = 0;
};

struct ArmWorkerLifecycleResult {
    ArmId arm_id = ArmId::Left;
    ArmWorkerCommand command;
    BackendResult<RobotState> result;
    BackendTiming dispatch_timing;
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
    BackendResult<RobotState> enqueueLifecycleCommand(ArmWorkerCommand command);
    BackendResult<RobotState> resetFault(uint64_t command_seq = 0, uint64_t deadline_ns = 0);
    std::optional<ArmWorkerLifecycleResult> lastLifecycleResult() const;
    std::optional<ArmWorkerLifecycleResult> waitForLifecycleResult(
        const ArmWorkerCommand& command,
        uint64_t not_before_ns,
        uint64_t wait_until_ns
    );
    std::optional<ArmSendResult> lastSendResult() const;
    std::optional<ArmSendResult> waitForSendResult(
        const SendServoJRequest& request,
        uint64_t not_before_ns,
        uint64_t wait_until_ns
    );
    ArmWorkerTelemetry telemetry() const;

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
    SendServoJResult deadlineMissedResult(
        const SendServoJRequest& request,
        const BackendTiming& dispatch_timing
    ) const;
    SendServoJResult notRunningResult(const SendServoJRequest& request, uint64_t now_ns) const;
    BackendResult<RobotState> lifecycleNotRunningResult(
        const ArmWorkerCommand& command,
        uint64_t now_ns
    ) const;
    BackendResult<RobotState> lifecycleQueueFullResult(
        const ArmWorkerCommand& command,
        uint64_t now_ns
    ) const;
    BackendResult<RobotState> lifecycleExpiredResult(
        const ArmWorkerCommand& command,
        uint64_t now_ns
    ) const;
    BackendResult<RobotState> lifecycleTimeoutResult(
        const ArmWorkerCommand& command,
        uint64_t start_ns,
        uint64_t end_ns
    ) const;
    BackendResult<RobotState> executeLifecycleCommand(
        const ArmWorkerCommand& command,
        bool& backend_ready
    );
    void storeLifecycleResult(
        const ArmWorkerCommand& command,
        const BackendResult<RobotState>& result,
        const BackendTiming& dispatch_timing
    );
    void storeLifecycleResultLocked(
        const ArmWorkerCommand& command,
        const BackendResult<RobotState>& result,
        const BackendTiming& dispatch_timing
    );
    void stopBackendBeforeExit(bool backend_ready);

    std::unique_ptr<IRobotBackend> backend_;
    ArmWorkerOptions options_;
    ArmId arm_id_;
    std::string name_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_requested_ = false;
    bool running_ = false;
    std::optional<SendServoJRequest> pending_servo_j_;
    std::deque<ArmWorkerCommand> lifecycle_queue_;
    std::optional<BackendResult<RobotState>> latest_state_;
    uint64_t latest_state_observed_ns_ = 0;
    std::optional<ArmSendResult> last_send_result_;
    std::optional<ArmWorkerLifecycleResult> last_lifecycle_result_;
    ArmWorkerTelemetry telemetry_;

    std::thread thread_;
};

}  // namespace rb_servo
