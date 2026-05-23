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

    pending_servo_j_ = std::move(request);
    cv_.notify_one();
}

std::optional<ArmSendResult> ArmWorker::lastSendResult() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_send_result_;
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
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (stop_requested_) {
                running_ = false;
                pending_servo_j_.reset();
                return;
            }
            if (pending_servo_j_.has_value()) {
                command = pending_servo_j_;
                pending_servo_j_.reset();
            }
        }

        const uint64_t now = nowSteadyNs();
        if (command.has_value()) {
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
                const SendServoJResult result = backend_->sendServoJ(*command);
                const uint64_t send_end_ns = nowSteadyNs();
                storeSendResult(*command, result, makeBackendTiming(send_start_ns, send_end_ns));
            }
        }

        const BackendResult<RobotState> read_result = backend_->readState();
        storeReadResult(read_result, nowSteadyNs());

        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_requested_) {
            running_ = false;
            pending_servo_j_.reset();
            return;
        }
        cv_.wait_for(lock, readPeriod(options_), [this] {
            return stop_requested_ || pending_servo_j_.has_value();
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
    arm_result.dispatch_timing = dispatch_timing;
    last_send_result_ = std::move(arm_result);
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

}  // namespace rb_servo
