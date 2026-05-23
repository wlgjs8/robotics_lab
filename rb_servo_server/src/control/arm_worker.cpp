#include "rb_servo/control/arm_worker.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "rb_servo/core/clock.hpp"

namespace rb_servo {
namespace {

std::chrono::nanoseconds readPeriod(const ArmWorkerOptions& options) {
    const uint64_t period_ns = options.read_period_ns == 0 ? 1'000'000 : options.read_period_ns;
    return std::chrono::nanoseconds(period_ns);
}

BackendResult<RobotState> lifecycleFailureAsReadResult(
    const BackendResult<RobotState>& lifecycle_result
) {
    return failedReadState(lifecycle_result.error, lifecycle_result.timing);
}

BackendOp backendOpForCommand(ArmWorkerCommandKind kind) {
    switch (kind) {
        case ArmWorkerCommandKind::ResetFault:
            return BackendOp::ResetFault;
        case ArmWorkerCommandKind::Stop:
            return BackendOp::Stop;
        case ArmWorkerCommandKind::ServoJ:
            return BackendOp::SendServoJ;
    }
    return BackendOp::ReadState;
}

BackendResult<RobotState> failedLifecycleResult(
    const ArmWorkerCommand& command,
    const BackendError& error,
    const BackendTiming& timing
) {
    BackendResult<RobotState> result;
    result.ok = false;
    result.op = backendOpForCommand(command.kind);
    result.error = error;
    result.timing = timing;
    return result;
}

BackendResult<RobotState> acceptedLifecycleEnqueueResult(
    const ArmWorkerCommand& command,
    const std::optional<BackendResult<RobotState>>& latest_state,
    const BackendTiming& timing
) {
    BackendResult<RobotState> result;
    result.ok = true;
    result.op = backendOpForCommand(command.kind);
    result.error = noBackendError();
    result.timing = timing;
    if (latest_state.has_value()) {
        result.value = latest_state->value;
    }
    return result;
}

bool sendResultMatches(
    const std::optional<ArmSendResult>& result,
    const SendServoJRequest& request,
    uint64_t not_before_ns
) {
    return result.has_value() &&
           result->request.command_seq == request.command_seq &&
           result->request.deadline_ns == request.deadline_ns &&
           result->request.host_time_ns == request.host_time_ns &&
           result->dispatch_timing.end_ns >= not_before_ns;
}

bool lifecycleResultMatches(
    const std::optional<ArmWorkerLifecycleResult>& result,
    const ArmWorkerCommand& command,
    uint64_t not_before_ns
) {
    return result.has_value() &&
           result->command.kind == command.kind &&
           result->command.command_seq == command.command_seq &&
           result->command.deadline_ns == command.deadline_ns &&
           result->command.host_time_ns == command.host_time_ns &&
           result->dispatch_timing.end_ns >= not_before_ns;
}

std::size_t lifecycleQueueCapacity(const ArmWorkerOptions& options) {
    return options.lifecycle_queue_capacity == 0 ? 1 : options.lifecycle_queue_capacity;
}

}  // namespace

ArmWorker::ArmWorker(std::unique_ptr<IRobotBackend> backend, ArmWorkerOptions options)
    : backend_(std::move(backend)), options_(options) {
    if (!backend_) {
        throw std::invalid_argument("ArmWorker requires a backend");
    }
    arm_id_ = backend_->armId();
    name_ = backend_->name();
}

ArmWorker::~ArmWorker() {
    stop();
}

bool ArmWorker::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_ || thread_.joinable()) {
        return false;
    }
    stop_requested_ = false;
    running_ = true;
    thread_ = std::thread(&ArmWorker::run, this);
    return true;
}

void ArmWorker::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ && !thread_.joinable()) {
            return;
        }
        stop_requested_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
}

BackendResult<RobotState> ArmWorker::latestState(uint64_t max_age_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_state_.has_value()) {
        const uint64_t now = nowSteadyNs();
        return failedReadState(
            backendError(
                BackendErrorKind::RobotDisconnected,
                "arm worker has no state sample",
                "",
                "arm_worker_no_state"
            ),
            makeBackendTiming(now, now)
        );
    }

    if (max_age_ns > 0 && latest_state_observed_ns_ > 0) {
        const uint64_t now = nowSteadyNs();
        if (now >= latest_state_observed_ns_ && now - latest_state_observed_ns_ > max_age_ns) {
            return failedReadState(
                backendError(
                    BackendErrorKind::TransportTimeout,
                    "arm worker state sample is stale",
                    "",
                    "arm_worker_state_stale"
                ),
                makeBackendTiming(latest_state_observed_ns_, now)
            );
        }
    }

    return *latest_state_;
}

void ArmWorker::enqueueServoJ(SendServoJRequest request) {
    const uint64_t now = nowSteadyNs();
    std::lock_guard<std::mutex> lock(mutex_);

    if (!running_) {
        const SendServoJResult result = notRunningResult(request, now);
        storeSendResultLocked(request, result, makeBackendTiming(now, now));
        return;
    }

    if (isExpired(request, now)) {
        const SendServoJResult result = expiredResult(request, now);
        storeSendResultLocked(request, result, makeBackendTiming(now, now));
        return;
    }

    if (pending_servo_j_.has_value() &&
        pending_servo_j_->command_seq != request.command_seq) {
        ++telemetry_.worker_command_drops_total;
        ++telemetry_.worker_pending_overwrites_total;
        telemetry_.worker_last_dropped_seq = pending_servo_j_->command_seq;
    }
    telemetry_.worker_last_enqueued_seq = request.command_seq;
    pending_servo_j_ = std::move(request);
    cv_.notify_one();
}

BackendResult<RobotState> ArmWorker::enqueueLifecycleCommand(ArmWorkerCommand command) {
    const uint64_t now = nowSteadyNs();
    std::lock_guard<std::mutex> lock(mutex_);
    if (command.kind == ArmWorkerCommandKind::ServoJ) {
        return failedLifecycleResult(
            command,
            backendError(
                BackendErrorKind::SuppressedByPolicy,
                "servo_j must use the arm worker servo latest-wins slot",
                "",
                "arm_worker_lifecycle_wrong_command_kind"
            ),
            makeBackendTiming(now, now)
        );
    }
    if (command.host_time_ns == 0) {
        command.host_time_ns = now;
    }
    if (!running_) {
        return lifecycleNotRunningResult(command, now);
    }
    if (command.deadline_ns > 0 && now >= command.deadline_ns) {
        return lifecycleExpiredResult(command, now);
    }
    if (lifecycle_queue_.size() >= lifecycleQueueCapacity(options_)) {
        return lifecycleQueueFullResult(command, now);
    }

    lifecycle_queue_.push_back(command);
    cv_.notify_one();
    return acceptedLifecycleEnqueueResult(command, latest_state_, makeBackendTiming(now, now));
}

BackendResult<RobotState> ArmWorker::resetFault(uint64_t command_seq, uint64_t deadline_ns) {
    const uint64_t now = nowSteadyNs();
    ArmWorkerCommand command;
    command.kind = ArmWorkerCommandKind::ResetFault;
    command.command_seq = command_seq;
    command.host_time_ns = now;
    command.deadline_ns = deadline_ns == 0 ? now + 1'000'000'000 : deadline_ns;

    const BackendResult<RobotState> enqueue_result = enqueueLifecycleCommand(command);
    if (!enqueue_result.ok) {
        return enqueue_result;
    }

    const std::optional<ArmWorkerLifecycleResult> result =
        waitForLifecycleResult(command, now, command.deadline_ns);
    if (!result.has_value()) {
        return lifecycleTimeoutResult(command, now, nowSteadyNs());
    }
    return result->result;
}

std::optional<ArmWorkerLifecycleResult> ArmWorker::lastLifecycleResult() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_lifecycle_result_;
}

std::optional<ArmWorkerLifecycleResult> ArmWorker::waitForLifecycleResult(
    const ArmWorkerCommand& command,
    uint64_t not_before_ns,
    uint64_t wait_until_ns
) {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto matches = [this, &command, not_before_ns] {
        return lifecycleResultMatches(last_lifecycle_result_, command, not_before_ns);
    };

    while (!matches()) {
        if (wait_until_ns == 0) {
            return std::nullopt;
        }

        const uint64_t now = nowSteadyNs();
        if (now >= wait_until_ns) {
            return std::nullopt;
        }

        cv_.wait_for(lock, std::chrono::nanoseconds(wait_until_ns - now), matches);
    }

    return last_lifecycle_result_;
}

std::optional<ArmSendResult> ArmWorker::lastSendResult() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_send_result_;
}

std::optional<ArmSendResult> ArmWorker::waitForSendResult(
    const SendServoJRequest& request,
    uint64_t not_before_ns,
    uint64_t wait_until_ns
) {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto matches = [this, &request, not_before_ns] {
        return sendResultMatches(last_send_result_, request, not_before_ns);
    };

    while (!matches()) {
        if (wait_until_ns == 0) {
            return std::nullopt;
        }

        const uint64_t now = nowSteadyNs();
        if (now >= wait_until_ns) {
            return std::nullopt;
        }

        cv_.wait_for(lock, std::chrono::nanoseconds(wait_until_ns - now), matches);
    }

    return last_send_result_;
}

ArmWorkerTelemetry ArmWorker::telemetry() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_;
}

ArmId ArmWorker::armId() const {
    return arm_id_;
}

std::string ArmWorker::name() const {
    return name_;
}

void ArmWorker::run() {
    bool backend_ready = false;

    const BackendResult<RobotState> connect_result = backend_->connect();
    if (!connect_result.ok) {
        storeReadResult(lifecycleFailureAsReadResult(connect_result), nowSteadyNs());
    } else {
        const BackendResult<RobotState> initialize_result = backend_->initialize();
        if (!initialize_result.ok) {
            storeReadResult(lifecycleFailureAsReadResult(initialize_result), nowSteadyNs());
        } else {
            backend_ready = true;
        }
    }

    while (true) {
        std::optional<SendServoJRequest> command;
        std::optional<ArmWorkerCommand> lifecycle_command;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (stop_requested_) {
                running_ = false;
                pending_servo_j_.reset();
                lifecycle_queue_.clear();
                lock.unlock();
                stopBackendBeforeExit(backend_ready);
                return;
            }
            if (!lifecycle_queue_.empty()) {
                lifecycle_command = lifecycle_queue_.front();
                lifecycle_queue_.pop_front();
            } else if (pending_servo_j_.has_value()) {
                command = pending_servo_j_;
                telemetry_.worker_last_dispatched_seq = command->command_seq;
                pending_servo_j_.reset();
            }
        }

        const uint64_t now = nowSteadyNs();
        if (lifecycle_command.has_value()) {
            if (lifecycle_command->deadline_ns > 0 && now >= lifecycle_command->deadline_ns) {
                const BackendResult<RobotState> result = lifecycleExpiredResult(*lifecycle_command, now);
                storeLifecycleResult(*lifecycle_command, result, makeBackendTiming(now, now));
            } else {
                const uint64_t dispatch_start_ns = nowSteadyNs();
                BackendResult<RobotState> result =
                    executeLifecycleCommand(*lifecycle_command, backend_ready);
                const uint64_t dispatch_end_ns = nowSteadyNs();
                const BackendTiming dispatch_timing =
                    makeBackendTiming(dispatch_start_ns, dispatch_end_ns);
                if (lifecycle_command->deadline_ns > 0 &&
                    dispatch_end_ns > lifecycle_command->deadline_ns) {
                    result = lifecycleTimeoutResult(
                        *lifecycle_command,
                        dispatch_start_ns,
                        dispatch_end_ns
                    );
                } else if (result.timing.start_ns == 0 && result.timing.end_ns == 0) {
                    result.timing = dispatch_timing;
                }
                storeLifecycleResult(*lifecycle_command, result, dispatch_timing);
            }
        } else if (command.has_value()) {
            if (isExpired(*command, now)) {
                const SendServoJResult result = expiredResult(*command, now);
                storeSendResult(*command, result, makeBackendTiming(now, now));
            } else if (!backend_ready) {
                const SendServoJResult result = rejectedSend(
                    *command,
                    backendError(
                        BackendErrorKind::SuppressedByPolicy,
                        "servo_j request suppressed because arm backend is not ready",
                        "",
                        "arm_worker_backend_not_ready"
                    ),
                    makeBackendTiming(now, now)
                );
                storeSendResult(*command, result, makeBackendTiming(now, now));
            } else {
                const uint64_t send_start_ns = nowSteadyNs();
                SendServoJResult result = backend_->sendServoJ(*command);
                const uint64_t send_end_ns = nowSteadyNs();
                const BackendTiming dispatch_timing = makeBackendTiming(send_start_ns, send_end_ns);
                if (isExpired(*command, send_end_ns)) {
                    result = deadlineMissedResult(*command, dispatch_timing);
                }
                storeSendResult(*command, result, dispatch_timing);
            }
        }

        const BackendResult<RobotState> read_result = backend_->readState();
        storeReadResult(read_result, nowSteadyNs());

        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_requested_) {
            running_ = false;
            pending_servo_j_.reset();
            lifecycle_queue_.clear();
            lock.unlock();
            stopBackendBeforeExit(backend_ready);
            return;
        }
        cv_.wait_for(lock, readPeriod(options_), [this] {
            return stop_requested_ || !lifecycle_queue_.empty() || pending_servo_j_.has_value();
        });
    }
}

void ArmWorker::storeReadResult(const BackendResult<RobotState>& result, uint64_t observed_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_state_ = result;
    latest_state_observed_ns_ = observed_ns;
}

void ArmWorker::storeSendResult(
    const SendServoJRequest& request,
    const SendServoJResult& result,
    const BackendTiming& dispatch_timing
) {
    std::lock_guard<std::mutex> lock(mutex_);
    storeSendResultLocked(request, result, dispatch_timing);
}

void ArmWorker::storeSendResultLocked(
    const SendServoJRequest& request,
    const SendServoJResult& result,
    const BackendTiming& dispatch_timing
) {
    ArmSendResult arm_result;
    arm_result.arm_id = arm_id_;
    arm_result.request = request;
    arm_result.result = result;
    if (arm_result.result.timing.start_ns == 0 && arm_result.result.timing.end_ns == 0) {
        arm_result.result.timing = dispatch_timing;
    }
    arm_result.dispatch_timing = dispatch_timing;
    last_send_result_ = std::move(arm_result);
    telemetry_.worker_last_completed_seq = request.command_seq;
    cv_.notify_all();
}

BackendResult<RobotState> ArmWorker::executeLifecycleCommand(
    const ArmWorkerCommand& command,
    bool& backend_ready
) {
    switch (command.kind) {
        case ArmWorkerCommandKind::ResetFault: {
            BackendResult<RobotState> result = backend_->resetFault();
            if (result.ok) {
                backend_ready = true;
            }
            return result;
        }
        case ArmWorkerCommandKind::Stop: {
            BackendResult<RobotState> result = backend_->stop();
            if (result.ok) {
                backend_ready = false;
            }
            return result;
        }
        case ArmWorkerCommandKind::ServoJ:
            break;
    }

    const uint64_t now = nowSteadyNs();
    return failedLifecycleResult(
        command,
        backendError(
            BackendErrorKind::SuppressedByPolicy,
            "servo_j must use the arm worker servo latest-wins slot",
            "",
            "arm_worker_lifecycle_wrong_command_kind"
        ),
        makeBackendTiming(now, now)
    );
}

void ArmWorker::storeLifecycleResult(
    const ArmWorkerCommand& command,
    const BackendResult<RobotState>& result,
    const BackendTiming& dispatch_timing
) {
    std::lock_guard<std::mutex> lock(mutex_);
    storeLifecycleResultLocked(command, result, dispatch_timing);
}

void ArmWorker::storeLifecycleResultLocked(
    const ArmWorkerCommand& command,
    const BackendResult<RobotState>& result,
    const BackendTiming& dispatch_timing
) {
    ArmWorkerLifecycleResult lifecycle_result;
    lifecycle_result.arm_id = arm_id_;
    lifecycle_result.command = command;
    lifecycle_result.result = result;
    if (lifecycle_result.result.timing.start_ns == 0 &&
        lifecycle_result.result.timing.end_ns == 0) {
        lifecycle_result.result.timing = dispatch_timing;
    }
    lifecycle_result.dispatch_timing = dispatch_timing;
    latest_state_ = lifecycle_result.result;
    latest_state_observed_ns_ = dispatch_timing.end_ns;
    last_lifecycle_result_ = std::move(lifecycle_result);
    cv_.notify_all();
}

bool ArmWorker::isExpired(const SendServoJRequest& request, uint64_t now_ns) const {
    return request.deadline_ns > 0 && now_ns >= request.deadline_ns;
}

SendServoJResult ArmWorker::expiredResult(
    const SendServoJRequest& request,
    uint64_t now_ns
) const {
    return rejectedSend(
        request,
        backendError(
            BackendErrorKind::CommandTimeout,
            "servo_j request expired before arm worker dispatch",
            "",
            "arm_worker_command_expired"
        ),
        makeBackendTiming(now_ns, now_ns)
    );
}

SendServoJResult ArmWorker::deadlineMissedResult(
    const SendServoJRequest& request,
    const BackendTiming& dispatch_timing
) const {
    return rejectedSend(
        request,
        backendError(
            BackendErrorKind::CommandTimeout,
            "servo_j request completed after its command deadline",
            "",
            "arm_worker_send_result_timeout"
        ),
        dispatch_timing
    );
}

SendServoJResult ArmWorker::notRunningResult(
    const SendServoJRequest& request,
    uint64_t now_ns
) const {
    return rejectedSend(
        request,
        backendError(
            BackendErrorKind::SuppressedByPolicy,
            "servo_j request suppressed because arm worker is not running",
            "",
            "arm_worker_not_running"
        ),
        makeBackendTiming(now_ns, now_ns)
    );
}

BackendResult<RobotState> ArmWorker::lifecycleNotRunningResult(
    const ArmWorkerCommand& command,
    uint64_t now_ns
) const {
    return failedLifecycleResult(
        command,
        backendError(
            BackendErrorKind::SuppressedByPolicy,
            "lifecycle request suppressed because arm worker is not running",
            "",
            "arm_worker_lifecycle_not_running"
        ),
        makeBackendTiming(now_ns, now_ns)
    );
}

BackendResult<RobotState> ArmWorker::lifecycleQueueFullResult(
    const ArmWorkerCommand& command,
    uint64_t now_ns
) const {
    return failedLifecycleResult(
        command,
        backendError(
            BackendErrorKind::SuppressedByPolicy,
            "lifecycle request rejected because arm worker lifecycle queue is full",
            "",
            "arm_worker_lifecycle_queue_full",
            true,
            true
        ),
        makeBackendTiming(now_ns, now_ns)
    );
}

BackendResult<RobotState> ArmWorker::lifecycleExpiredResult(
    const ArmWorkerCommand& command,
    uint64_t now_ns
) const {
    return failedLifecycleResult(
        command,
        backendError(
            BackendErrorKind::CommandTimeout,
            "lifecycle request expired before arm worker dispatch",
            "",
            "arm_worker_lifecycle_command_expired"
        ),
        makeBackendTiming(now_ns, now_ns)
    );
}

BackendResult<RobotState> ArmWorker::lifecycleTimeoutResult(
    const ArmWorkerCommand& command,
    uint64_t start_ns,
    uint64_t end_ns
) const {
    return failedLifecycleResult(
        command,
        backendError(
            BackendErrorKind::CommandTimeout,
            "arm worker did not publish lifecycle result before command deadline",
            "",
            "arm_worker_lifecycle_result_timeout"
        ),
        makeBackendTiming(start_ns, end_ns)
    );
}

void ArmWorker::stopBackendBeforeExit(bool backend_ready) {
    if (!backend_ready) {
        return;
    }
    (void)backend_->stop();
}

}  // namespace rb_servo
