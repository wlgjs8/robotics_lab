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

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/queue_sync_controller.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

struct ArmWorkerOptions {
    uint64_t read_period_ns = 10'000'000;
    // Cadence ownership. 0 (default) keeps the legacy event-driven behaviour:
    // the worker dispatches whatever the servo loop enqueued as soon as it can,
    // and the loop owns the send rhythm.
    //
    // Non-zero makes THIS worker own its arm's send cadence: it sends once per
    // send_period_ns (plus the queue-sync trim) from the latest-wins mailbox the
    // servo loop keeps refreshing. That is what queue sync requires -- holding a
    // control box's queue at a fixed depth means matching that box's clock, and
    // a single loop has one period for two boxes running two different clocks.
    uint64_t send_period_ns = 0;
    QueueSyncConfig queue_sync;
    // RT scheduling for THIS worker's thread, applied once at thread entry.
    // 0 / -1 = leave it alone. See ServoConfig::worker_realtime_priority.
    int realtime_priority = 0;
    int cpu_core = -1;
    std::size_t lifecycle_queue_capacity = 4;
    bool rbpodo_async_streaming_enabled = false;
    RbpodoAsyncStreamingMode rbpodo_async_streaming_mode =
        RbpodoAsyncStreamingMode::Disabled;
    double rbpodo_async_max_pending_age_ms = 10.0;
    bool controller_simulation_timing_reject_tolerance_enabled = false;
    RbpodoAsyncAckSupervisionConfig rbpodo_async_ack_supervision;
    RbpodoAsyncReferenceSupervisionConfig rbpodo_async_reference_supervision;
};

enum class ArmWorkerCommandKind {
    ServoJ,
    ResetFault,
    Stop,
    SetFreedrive
};

struct ArmWorkerCommand {
    ArmWorkerCommandKind kind = ArmWorkerCommandKind::ServoJ;
    SendServoJRequest servo_j;
    uint64_t command_seq = 0;
    uint64_t host_time_ns = 0;
    uint64_t deadline_ns = 0;
    bool freedrive_on = false;  // SetFreedrive payload.
};

struct ArmWorkerLifecycleResult {
    ArmId arm_id = ArmId::Left;
    ArmWorkerCommand command;
    BackendResult<RobotState> result;
    BackendTiming dispatch_timing;
};

struct ArmWorkerStartupTelemetry {
    ArmId arm_id = ArmId::Left;
    std::string backend_name;
    std::string phase = "not_started";
    uint64_t start_time_ns = 0;
    uint64_t phase_time_ns = 0;
    std::string last_op;
    bool last_result_ok = false;
    std::string last_error_name = "None";
    std::string last_error_kind = "None";
    std::string last_error_message;
    bool latest_state_present = false;
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
    ArmSendResult enqueueAsyncServoJ(SendServoJRequest request);
    BackendResult<RobotState> enqueueLifecycleCommand(ArmWorkerCommand command);
    BackendResult<RobotState> resetFault(uint64_t command_seq = 0, uint64_t deadline_ns = 0);
    BackendResult<RobotState> setFreedrive(bool on, uint64_t command_seq = 0, uint64_t deadline_ns = 0);
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
    ArmWorkerStartupTelemetry startupTelemetry() const;
    RbpodoAsyncStreamingTelemetry asyncStreamingTelemetry() const;
    // Latest queue-sync decision for this arm. Meaningful only when this worker
    // owns its cadence (options.send_period_ns > 0) and queue_sync.enable is on;
    // otherwise it stays at its neutral default so a consumer cannot mistake an
    // inactive controller for a locked one.
    QueueSyncDecision queueSyncDecision() const;
    // Control-box queue occupancy from the last REAL send. Kept separate from
    // lastSendResult(): in cadence mode the loop enqueues every tick and the
    // enqueue path overwrites last_send_result_, so reading queue_ack from there
    // returns a default (fill = -1) almost always.
    RbpodoQueueAckTelemetry latestQueueAck() const;
    std::optional<BackendTransportTelemetry> transportTelemetry() const;

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
    void storeImmediateAsyncResultLocked(
        const SendServoJRequest& request,
        const SendServoJResult& result,
        const BackendTiming& dispatch_timing
    );
    void updateAsyncSendTelemetryLocked(
        const SendServoJRequest& request,
        const SendServoJResult& result,
        const BackendTiming& dispatch_timing
    );
    void updateAsyncReferenceSupervisionLocked(
        const BackendResult<RobotState>& result,
        uint64_t observed_ns
    );
    void noteAsyncDropLocked(
        uint64_t seq,
        const std::string& reason,
        RbpodoAsyncStreamingSupervisionState state =
            RbpodoAsyncStreamingSupervisionState::Warning
    );
    void updateStartupPhase(const std::string& phase);
    void updateStartupResultPhase(
        const std::string& phase,
        const BackendResult<RobotState>& result
    );
    bool asyncStreamingEnabled() const;
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

    // Cadence + queue sync. Touched only on the worker thread except for the
    // decision snapshot, which is published under mutex_ for telemetry readers.
    QueueSyncController queue_sync_;
    QueueSyncDecision queue_sync_decision_;
    RbpodoQueueAckTelemetry latest_queue_ack_;
    // Last setpoint actually handed to the box. In cadence mode the worker must
    // put a command on the wire EVERY tick: the box drains its FIFO on its own
    // clock, so a skipped send is a queue entry it never receives. Repeating the
    // previous setpoint is a hold, which is what a 500 Hz servo stream does
    // anyway; skipping starves the queue instead (measured: 231 Hz of actual
    // sends against a 500 Hz loop, and the fill drained to the protect floor).
    std::optional<SendServoJRequest> last_servo_j_;
    uint64_t repeated_sends_total_ = 0;

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
    ArmWorkerStartupTelemetry startup_telemetry_;
    RbpodoAsyncStreamingTelemetry async_telemetry_;
    std::optional<SendServoJRequest> last_async_sent_request_;
    std::optional<JointArray> last_async_q_ref_deg_;
    uint64_t last_async_reference_fault_sample_ns_ = 0;
    uint64_t consecutive_async_timing_rejects_ = 0;

    std::thread thread_;
};

}  // namespace rb_servo
