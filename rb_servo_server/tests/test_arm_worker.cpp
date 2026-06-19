#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rb_servo/control/arm_worker.hpp"
#include "rb_servo/core/clock.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

rb_servo::JointArray joints(double value) {
    rb_servo::JointArray out{};
    out.fill(value);
    return out;
}

bool sameJointArray(const rb_servo::JointArray& a, const rb_servo::JointArray& b) {
    for (int i = 0; i < rb_servo::kDof; ++i) {
        if (std::abs(a[i] - b[i]) > 1e-9) return false;
    }
    return true;
}

rb_servo::RobotState validState(rb_servo::ArmId arm_id, const rb_servo::JointArray& q) {
    rb_servo::RobotState state;
    state.arm_id = arm_id;
    state.host_time_ns = rb_servo::nowSteadyNs();
    state.q_actual_deg = q;
    state.q_target_deg = q;
    state.has_valid_joint_state = true;
    state.connection_state = rb_servo::RobotConnectionState::Connected;
    state.servo_enabled = true;
    return state;
}

rb_servo::BackendResult<rb_servo::RobotState> backendStateResult(
    rb_servo::BackendOp op,
    const rb_servo::RobotState& state
) {
    rb_servo::BackendResult<rb_servo::RobotState> result;
    result.ok = true;
    result.op = op;
    result.value = state;
    result.error = rb_servo::noBackendError();
    return result;
}

class WorkerTestBackend final : public rb_servo::IRobotBackend {
public:
    explicit WorkerTestBackend(
        rb_servo::ArmId arm_id,
        rb_servo::BackendErrorKind send_error_kind = rb_servo::BackendErrorKind::None,
        bool block_first_read = false,
        bool socket_send_only = false,
        bool block_initialize = false,
        bool block_connect = false,
        std::chrono::milliseconds send_delay = std::chrono::milliseconds(0)
    ) : arm_id_(arm_id),
        send_error_kind_(send_error_kind),
        block_first_read_(block_first_read),
        socket_send_only_(socket_send_only),
        block_initialize_(block_initialize),
        block_connect_(block_connect),
        send_delay_(send_delay),
        state_(validState(arm_id, joints(0.0))) {}

    rb_servo::BackendResult<rb_servo::RobotState> connect() override {
        std::unique_lock<std::mutex> lock(mutex_);
        if (block_connect_) {
            connect_waiting_ = true;
            cv_.notify_all();
            cv_.wait(lock, [this] {
                return connect_released_;
            });
        }
        connected_ = true;
        ++connect_count_;
        return backendStateResult(rb_servo::BackendOp::Connect, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> initialize() override {
        std::unique_lock<std::mutex> lock(mutex_);
        if (block_initialize_) {
            initialize_waiting_ = true;
            cv_.notify_all();
            cv_.wait(lock, [this] {
                return initialize_released_;
            });
        }
        initialized_ = true;
        ++initialize_count_;
        return backendStateResult(rb_servo::BackendOp::Initialize, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> readState() override {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ++read_count_;
            if (block_first_read_ && read_count_ == 1) {
                first_read_waiting_ = true;
                cv_.notify_all();
                cv_.wait(lock, [this] {
                    return first_read_released_;
                });
            }
            state_.host_time_ns = rb_servo::nowSteadyNs();
        }
        cv_.notify_all();
        std::lock_guard<std::mutex> lock(mutex_);
        return backendStateResult(rb_servo::BackendOp::ReadState, state_);
    }

    rb_servo::SendServoJResult sendServoJ(const rb_servo::SendServoJRequest& request) override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++send_count_;
            last_request_ = request;
            if (send_error_kind_ == rb_servo::BackendErrorKind::None) {
                state_.q_actual_deg = request.q_target_deg;
                state_.q_target_deg = request.q_target_deg;
                state_.host_time_ns = rb_servo::nowSteadyNs();
            }
        }
        cv_.notify_all();
        if (send_delay_.count() > 0) {
            std::this_thread::sleep_for(send_delay_);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (send_error_kind_ == rb_servo::BackendErrorKind::None) {
            rb_servo::SendServoJResult result =
                rb_servo::acceptedSend(request, {}, state_, "cache");
            if (socket_send_only_) {
                result.ack_policy = rb_servo::BackendAckPolicy::Disabled;
                result.ack_observed = false;
                result.controller_acceptance_observed = false;
                result.ack_wait_duration_us = 0.0;
                result.rbpodo_waiting_ack = false;
                result.acceptance_semantics = "socket_send_only";
            } else {
                result.ack_policy = rb_servo::BackendAckPolicy::Wait;
                result.ack_observed = true;
                result.controller_acceptance_observed = true;
                result.ack_wait_duration_us = result.timing.duration_us;
                result.rbpodo_waiting_ack = true;
                result.acceptance_semantics = "controller_ack_observed";
            }
            return result;
        }
        return rb_servo::rejectedSend(
            request,
            rb_servo::backendError(
                send_error_kind_,
                "test backend send failure",
                "",
                "test_backend_send_failure"
            )
        );
    }

    rb_servo::BackendResult<rb_servo::RobotState> stop() override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stop_count_;
        return backendStateResult(rb_servo::BackendOp::Stop, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> resetFault() override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++reset_count_;
        state_.has_error = false;
        state_.error_code = 0;
        return backendStateResult(rb_servo::BackendOp::ResetFault, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> setFreedrive(bool on) override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++freedrive_count_;
        last_freedrive_on_ = on;
        return backendStateResult(rb_servo::BackendOp::SetFreedrive, state_);
    }

    bool isConnected() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return connected_;
    }

    rb_servo::ArmId armId() const override { return arm_id_; }
    std::string name() const override { return "worker_test_backend"; }

    bool waitForReads(int expected, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this, expected] {
            return read_count_ >= expected;
        });
    }

    bool waitForSends(int expected, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this, expected] {
            return send_count_ >= expected;
        });
    }

    bool waitForResets(int expected, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this, expected] {
            return reset_count_ >= expected;
        });
    }

    bool waitForFirstReadEntered(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this] {
            return first_read_waiting_;
        });
    }

    bool waitForInitializeEntered(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this] {
            return initialize_waiting_;
        });
    }

    bool waitForConnectEntered(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this] {
            return connect_waiting_;
        });
    }

    void releaseConnect() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            connect_released_ = true;
        }
        cv_.notify_all();
    }

    void releaseFirstRead() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            first_read_released_ = true;
        }
        cv_.notify_all();
    }

    void releaseInitialize() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            initialize_released_ = true;
        }
        cv_.notify_all();
    }

    int connectCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return connect_count_;
    }

    int initializeCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return initialize_count_;
    }

    int sendCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return send_count_;
    }

    int resetCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return reset_count_;
    }

    int stopCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stop_count_;
    }

    int freedriveCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return freedrive_count_;
    }

    std::optional<bool> lastFreedriveOn() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_freedrive_on_;
    }

    std::optional<rb_servo::SendServoJRequest> lastRequest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_request_;
    }

private:
    rb_servo::ArmId arm_id_;
    rb_servo::BackendErrorKind send_error_kind_;
    bool block_first_read_ = false;
    bool socket_send_only_ = false;
    bool block_initialize_ = false;
    bool block_connect_ = false;
    std::chrono::milliseconds send_delay_{0};
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool connected_ = false;
    bool initialized_ = false;
    bool connect_waiting_ = false;
    bool connect_released_ = false;
    bool initialize_waiting_ = false;
    bool initialize_released_ = false;
    bool first_read_waiting_ = false;
    bool first_read_released_ = false;
    int connect_count_ = 0;
    int initialize_count_ = 0;
    int read_count_ = 0;
    int send_count_ = 0;
    int reset_count_ = 0;
    int stop_count_ = 0;
    int freedrive_count_ = 0;
    std::optional<bool> last_freedrive_on_;
    rb_servo::RobotState state_;
    std::optional<rb_servo::SendServoJRequest> last_request_;
};

rb_servo::SendServoJRequest request(uint64_t seq, const rb_servo::JointArray& q, uint64_t deadline_ns) {
    rb_servo::SendServoJRequest out;
    out.command_seq = seq;
    out.host_time_ns = rb_servo::nowSteadyNs();
    out.deadline_ns = deadline_ns;
    out.q_target_deg = q;
    return out;
}

rb_servo::ArmWorkerOptions asyncWorkerOptions(rb_servo::RbpodoAsyncStreamingMode mode) {
    rb_servo::ArmWorkerOptions options;
    options.read_period_ns = 1'000'000;
    options.rbpodo_async_streaming_enabled = true;
    options.rbpodo_async_streaming_mode = mode;
    options.rbpodo_async_max_pending_age_ms = 10.0;
    options.rbpodo_async_ack_supervision.max_consecutive_missing_ack = 2;
    return options;
}

bool testSuccessfulReadLoop() {
    auto backend = std::make_unique<WorkerTestBackend>(rb_servo::ArmId::Left);
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorkerOptions options;
    options.read_period_ns = 1'000'000;
    rb_servo::ArmWorker worker(std::move(backend), options);

    RB_CHECK(worker.armId() == rb_servo::ArmId::Left);
    RB_CHECK(worker.name() == "worker_test_backend");
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));

    const rb_servo::BackendResult<rb_servo::RobotState> state = worker.latestState(1'000'000'000);
    RB_CHECK(state.ok);
    RB_CHECK(state.op == rb_servo::BackendOp::ReadState);
    RB_CHECK(state.value.arm_id == rb_servo::ArmId::Left);
    RB_CHECK(state.value.has_valid_joint_state);
    RB_CHECK(raw_backend->connectCount() == 1);
    RB_CHECK(raw_backend->initializeCount() == 1);
    worker.stop();
    return true;
}

bool testInitializeStateIsPublishedBeforeFirstReadCompletes() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        true
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));

    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForFirstReadEntered(std::chrono::milliseconds(200)));

    const rb_servo::BackendResult<rb_servo::RobotState> state =
        worker.latestState(1'000'000'000);
    RB_CHECK(state.ok);
    RB_CHECK(state.op == rb_servo::BackendOp::Initialize);
    RB_CHECK(state.value.arm_id == rb_servo::ArmId::Left);
    RB_CHECK(state.value.has_valid_joint_state);
    RB_CHECK(raw_backend->connectCount() == 1);
    RB_CHECK(raw_backend->initializeCount() == 1);

    raw_backend->releaseFirstRead();
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));
    worker.stop();
    return true;
}

bool testConnectStateIsPublishedBeforeInitializeCompletes() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        false,
        false,
        true
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));

    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForInitializeEntered(std::chrono::milliseconds(200)));

    rb_servo::ArmWorkerStartupTelemetry startup = worker.startupTelemetry();
    RB_CHECK(startup.phase == "initialize_entered");
    RB_CHECK(!startup.latest_state_present || startup.last_op == "Connect");

    const rb_servo::BackendResult<rb_servo::RobotState> state =
        worker.latestState(1'000'000'000);
    RB_CHECK(state.ok);
    RB_CHECK(state.op == rb_servo::BackendOp::Connect);
    RB_CHECK(state.value.arm_id == rb_servo::ArmId::Left);
    RB_CHECK(state.value.has_valid_joint_state);
    RB_CHECK(raw_backend->connectCount() == 1);
    RB_CHECK(raw_backend->initializeCount() == 0);

    raw_backend->releaseInitialize();
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));
    worker.stop();
    return true;
}

bool testStartupTelemetryReportsBlockedConnect() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        false,
        false,
        false,
        true
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));

    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForConnectEntered(std::chrono::milliseconds(200)));

    rb_servo::ArmWorkerStartupTelemetry startup = worker.startupTelemetry();
    RB_CHECK(startup.phase == "connect_entered");
    RB_CHECK(!startup.latest_state_present);
    RB_CHECK(startup.backend_name == "worker_test_backend");

    raw_backend->releaseConnect();
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));
    worker.stop();
    return true;
}

bool testDefaultReadPeriodIsConservative() {
    rb_servo::ArmWorkerOptions options;
    RB_CHECK(options.read_period_ns == 10'000'000);
    return true;
}

bool testLatestStateReportsStaleSample() {
    auto backend = std::make_unique<WorkerTestBackend>(rb_servo::ArmId::Left);
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));
    worker.stop();

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const rb_servo::BackendResult<rb_servo::RobotState> state = worker.latestState(1);
    RB_CHECK(!state.ok);
    RB_CHECK(state.op == rb_servo::BackendOp::ReadState);
    RB_CHECK(state.error.kind == rb_servo::BackendErrorKind::TransportTimeout);
    RB_CHECK(state.error.name == "arm_worker_state_stale");
    return true;
}

bool testSendRequestAccepted() {
    auto backend = std::make_unique<WorkerTestBackend>(rb_servo::ArmId::Right);
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));

    const rb_servo::JointArray target = joints(12.0);
    worker.enqueueServoJ(request(42, target, rb_servo::nowSteadyNs() + 1'000'000'000));
    RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));

    const std::optional<rb_servo::ArmSendResult> last = worker.lastSendResult();
    RB_CHECK(last.has_value());
    RB_CHECK(last->arm_id == rb_servo::ArmId::Right);
    RB_CHECK(last->request.command_seq == 42);
    RB_CHECK(last->result.accepted);
    RB_CHECK(last->result.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(sameJointArray(last->result.requested_q_deg, target));

    const std::optional<rb_servo::SendServoJRequest> backend_request = raw_backend->lastRequest();
    RB_CHECK(backend_request.has_value());
    RB_CHECK(backend_request->command_seq == 42);
    RB_CHECK(sameJointArray(backend_request->q_target_deg, target));
    worker.stop();
    return true;
}

bool testExpiredCommandDropped() {
    auto backend = std::make_unique<WorkerTestBackend>(rb_servo::ArmId::Left);
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));

    worker.enqueueServoJ(request(7, joints(3.0), rb_servo::nowSteadyNs() - 1));
    const std::optional<rb_servo::ArmSendResult> last = worker.lastSendResult();
    RB_CHECK(last.has_value());
    RB_CHECK(last->request.command_seq == 7);
    RB_CHECK(!last->result.accepted);
    RB_CHECK(last->result.error.kind == rb_servo::BackendErrorKind::CommandTimeout);
    RB_CHECK(last->result.error.name == "arm_worker_command_expired");
    RB_CHECK(raw_backend->sendCount() == 0);
    worker.stop();
    return true;
}

bool testBackendSendFailurePreserved() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::TransportWriteFailed
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));

    worker.enqueueServoJ(request(8, joints(4.0), rb_servo::nowSteadyNs() + 1'000'000'000));
    RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));

    const std::optional<rb_servo::ArmSendResult> last = worker.lastSendResult();
    RB_CHECK(last.has_value());
    RB_CHECK(last->request.command_seq == 8);
    RB_CHECK(!last->result.accepted);
    RB_CHECK(last->result.error.kind == rb_servo::BackendErrorKind::TransportWriteFailed);
    RB_CHECK(last->result.error.name == "test_backend_send_failure");
    RB_CHECK(last->result.error.transport_fault);
    worker.stop();
    return true;
}

bool testLatestQueuedCommandWins() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        true
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForFirstReadEntered(std::chrono::milliseconds(200)));

    worker.enqueueServoJ(request(11, joints(11.0), rb_servo::nowSteadyNs() + 1'000'000'000));
    rb_servo::ArmWorkerTelemetry telemetry = worker.telemetry();
    RB_CHECK(telemetry.worker_queue_policy == "latest_wins");
    RB_CHECK(telemetry.worker_command_drops_total == 0);
    RB_CHECK(telemetry.worker_pending_overwrites_total == 0);
    RB_CHECK(telemetry.worker_last_enqueued_seq == 11);
    RB_CHECK(telemetry.worker_last_dispatched_seq == 0);
    RB_CHECK(telemetry.worker_last_completed_seq == 0);

    worker.enqueueServoJ(request(12, joints(12.0), rb_servo::nowSteadyNs() + 1'000'000'000));
    telemetry = worker.telemetry();
    RB_CHECK(telemetry.worker_command_drops_total == 1);
    RB_CHECK(telemetry.worker_pending_overwrites_total == 1);
    RB_CHECK(telemetry.worker_last_dropped_seq == 11);
    RB_CHECK(telemetry.worker_last_enqueued_seq == 12);
    RB_CHECK(telemetry.worker_last_dispatched_seq == 0);
    RB_CHECK(telemetry.worker_last_completed_seq == 0);

    raw_backend->releaseFirstRead();

    RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));
    const std::optional<rb_servo::ArmSendResult> last = worker.lastSendResult();
    RB_CHECK(last.has_value());
    RB_CHECK(last->request.command_seq == 12);
    RB_CHECK(last->result.accepted);
    RB_CHECK(raw_backend->sendCount() == 1);
    telemetry = worker.telemetry();
    RB_CHECK(telemetry.worker_command_drops_total == 1);
    RB_CHECK(telemetry.worker_pending_overwrites_total == 1);
    RB_CHECK(telemetry.worker_last_dropped_seq == 11);
    RB_CHECK(telemetry.worker_last_enqueued_seq == 12);
    RB_CHECK(telemetry.worker_last_dispatched_seq == 12);
    RB_CHECK(telemetry.worker_last_completed_seq == 12);
    worker.stop();
    return true;
}

bool testNoDropCountedAfterImmediateDispatch() {
    auto backend = std::make_unique<WorkerTestBackend>(rb_servo::ArmId::Right);
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));

    worker.enqueueServoJ(request(20, joints(20.0), rb_servo::nowSteadyNs() + 1'000'000'000));
    RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));
    rb_servo::ArmWorkerTelemetry telemetry = worker.telemetry();
    RB_CHECK(telemetry.worker_command_drops_total == 0);
    RB_CHECK(telemetry.worker_pending_overwrites_total == 0);
    RB_CHECK(telemetry.worker_last_enqueued_seq == 20);
    RB_CHECK(telemetry.worker_last_dispatched_seq == 20);
    RB_CHECK(telemetry.worker_last_completed_seq == 20);

    worker.enqueueServoJ(request(21, joints(21.0), rb_servo::nowSteadyNs() + 1'000'000'000));
    RB_CHECK(raw_backend->waitForSends(2, std::chrono::milliseconds(200)));
    telemetry = worker.telemetry();
    RB_CHECK(telemetry.worker_command_drops_total == 0);
    RB_CHECK(telemetry.worker_pending_overwrites_total == 0);
    RB_CHECK(telemetry.worker_last_dropped_seq == 0);
    RB_CHECK(telemetry.worker_last_enqueued_seq == 21);
    RB_CHECK(telemetry.worker_last_dispatched_seq == 21);
    RB_CHECK(telemetry.worker_last_completed_seq == 21);

    worker.stop();
    return true;
}

bool testResetFaultUsesLifecycleQueue() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        true
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForFirstReadEntered(std::chrono::milliseconds(200)));

    worker.enqueueServoJ(request(30, joints(30.0), rb_servo::nowSteadyNs() + 1'000'000'000));
    const uint64_t lifecycle_host_time = rb_servo::nowSteadyNs();
    const uint64_t lifecycle_deadline = lifecycle_host_time + 1'000'000'000;
    const rb_servo::ArmWorkerCommand reset_command{
        rb_servo::ArmWorkerCommandKind::ResetFault,
        {},
        31,
        lifecycle_host_time,
        lifecycle_deadline
    };
    const rb_servo::BackendResult<rb_servo::RobotState> enqueued =
        worker.enqueueLifecycleCommand(reset_command);
    RB_CHECK(enqueued.ok);
    RB_CHECK(raw_backend->resetCount() == 0);
    RB_CHECK(raw_backend->sendCount() == 0);

    raw_backend->releaseFirstRead();
    RB_CHECK(raw_backend->waitForResets(1, std::chrono::milliseconds(200)));
    const std::optional<rb_servo::ArmWorkerLifecycleResult> lifecycle =
        worker.waitForLifecycleResult(
            reset_command,
            enqueued.timing.start_ns,
            rb_servo::nowSteadyNs() + 1'000'000'000
        );
    RB_CHECK(lifecycle.has_value());
    RB_CHECK(lifecycle->command.kind == rb_servo::ArmWorkerCommandKind::ResetFault);
    RB_CHECK(lifecycle->command.command_seq == 31);
    RB_CHECK(lifecycle->result.ok);
    RB_CHECK(lifecycle->result.op == rb_servo::BackendOp::ResetFault);
    RB_CHECK(raw_backend->resetCount() == 1);
    worker.stop();
    return true;
}

bool testSetFreedriveUsesLifecycleQueue() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        true
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForFirstReadEntered(std::chrono::milliseconds(200)));
    RB_CHECK(raw_backend->freedriveCount() == 0);

    raw_backend->releaseFirstRead();
    const uint64_t deadline = rb_servo::nowSteadyNs() + 1'000'000'000;
    const rb_servo::BackendResult<rb_servo::RobotState> on_result =
        worker.setFreedrive(true, 50, deadline);
    RB_CHECK(on_result.ok);
    RB_CHECK(on_result.op == rb_servo::BackendOp::SetFreedrive);
    RB_CHECK(raw_backend->freedriveCount() == 1);
    RB_CHECK(raw_backend->lastFreedriveOn().value_or(false) == true);

    const rb_servo::BackendResult<rb_servo::RobotState> off_result =
        worker.setFreedrive(false, 51, rb_servo::nowSteadyNs() + 1'000'000'000);
    RB_CHECK(off_result.ok);
    RB_CHECK(raw_backend->freedriveCount() == 2);
    RB_CHECK(raw_backend->lastFreedriveOn().value_or(true) == false);
    worker.stop();
    return true;
}

bool testLifecycleQueueFullIsExplicit() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        true
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorkerOptions options;
    options.lifecycle_queue_capacity = 2;
    rb_servo::ArmWorker worker(std::move(backend), options);
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForFirstReadEntered(std::chrono::milliseconds(200)));

    const uint64_t deadline = rb_servo::nowSteadyNs() + 1'000'000'000;
    rb_servo::ArmWorkerCommand first;
    first.kind = rb_servo::ArmWorkerCommandKind::ResetFault;
    first.command_seq = 40;
    first.deadline_ns = deadline;
    rb_servo::ArmWorkerCommand second = first;
    second.command_seq = 41;
    rb_servo::ArmWorkerCommand third = first;
    third.command_seq = 42;

    RB_CHECK(worker.enqueueLifecycleCommand(first).ok);
    RB_CHECK(worker.enqueueLifecycleCommand(second).ok);
    const rb_servo::BackendResult<rb_servo::RobotState> rejected =
        worker.enqueueLifecycleCommand(third);
    RB_CHECK(!rejected.ok);
    RB_CHECK(rejected.op == rb_servo::BackendOp::ResetFault);
    RB_CHECK(rejected.error.kind == rb_servo::BackendErrorKind::SuppressedByPolicy);
    RB_CHECK(rejected.error.name == "arm_worker_lifecycle_queue_full");
    RB_CHECK(raw_backend->resetCount() == 0);

    raw_backend->releaseFirstRead();
    worker.stop();
    return true;
}

bool testStopJoinsThreadAndRejectsNewCommand() {
    auto backend = std::make_unique<WorkerTestBackend>(rb_servo::ArmId::Right);
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));
    worker.stop();

    worker.enqueueServoJ(request(9, joints(5.0), rb_servo::nowSteadyNs() + 1'000'000'000));
    const std::optional<rb_servo::ArmSendResult> last = worker.lastSendResult();
    RB_CHECK(last.has_value());
    RB_CHECK(last->request.command_seq == 9);
    RB_CHECK(!last->result.accepted);
    RB_CHECK(last->result.error.kind == rb_servo::BackendErrorKind::SuppressedByPolicy);
    RB_CHECK(last->result.error.name == "arm_worker_not_running");
    RB_CHECK(raw_backend->sendCount() == 0);

    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(2, std::chrono::milliseconds(200)));
    worker.stop();
    return true;
}

bool testNoDeadlockOnDestruction() {
    {
        auto backend = std::make_unique<WorkerTestBackend>(rb_servo::ArmId::Left);
        WorkerTestBackend* raw_backend = backend.get();
        rb_servo::ArmWorker worker(std::move(backend));
        RB_CHECK(worker.start());
        RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));
        worker.enqueueServoJ(request(10, joints(6.0), rb_servo::nowSteadyNs() + 1'000'000'000));
        RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));
    }
    return true;
}

bool testStopWithPendingCommandBehindReadDoesNotDeadlock() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        true
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(std::move(backend));
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForFirstReadEntered(std::chrono::milliseconds(200)));

    worker.enqueueServoJ(request(13, joints(13.0), rb_servo::nowSteadyNs() + 1'000'000'000));
    bool stopped = false;
    std::thread stopper([&] {
        worker.stop();
        stopped = true;
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    raw_backend->releaseFirstRead();
    stopper.join();

    RB_CHECK(stopped);
    RB_CHECK(raw_backend->sendCount() == 0);
    return true;
}

bool testAsyncLatestWinsOverwriteTelemetry() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        true
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(
        std::move(backend),
        asyncWorkerOptions(rb_servo::RbpodoAsyncStreamingMode::SdkAckWorker)
    );
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForFirstReadEntered(std::chrono::milliseconds(200)));

    const uint64_t deadline = rb_servo::nowSteadyNs() + 1'000'000'000;
    const rb_servo::ArmSendResult first =
        worker.enqueueAsyncServoJ(request(1011, joints(11.0), deadline));
    RB_CHECK(first.result.accepted);
    RB_CHECK(first.result.acceptance_semantics == "async_enqueued");

    const rb_servo::ArmSendResult second =
        worker.enqueueAsyncServoJ(request(1012, joints(12.0), deadline));
    RB_CHECK(second.result.accepted);

    rb_servo::RbpodoAsyncStreamingTelemetry async = worker.asyncStreamingTelemetry();
    RB_CHECK(async.commands_enqueued_total == 2);
    RB_CHECK(async.commands_overwritten_total == 1);
    RB_CHECK(async.commands_dropped_total == 1);
    RB_CHECK(async.last_command_seq == 1012);
    RB_CHECK(async.worker_backlog == 1);

    raw_backend->releaseFirstRead();
    RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));
    worker.stop();

    const std::optional<rb_servo::SendServoJRequest> last_request = raw_backend->lastRequest();
    RB_CHECK(last_request.has_value());
    RB_CHECK(last_request->command_seq == 1012);
    RB_CHECK(raw_backend->sendCount() == 1);
    async = worker.asyncStreamingTelemetry();
    RB_CHECK(async.commands_sent_total == 1);
    RB_CHECK(async.commands_acked_total == 1);
    RB_CHECK(async.last_sent_seq == 1012);
    RB_CHECK(async.last_ack_seq == 1012);
    RB_CHECK(async.last_async_acceptance_semantics == "controller_ack_observed");
    return true;
}

bool testAsyncSdkAckWorkerRecordsAckObservedResults() {
    auto backend = std::make_unique<WorkerTestBackend>(rb_servo::ArmId::Right);
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(
        std::move(backend),
        asyncWorkerOptions(rb_servo::RbpodoAsyncStreamingMode::SdkAckWorker)
    );
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));

    const uint64_t deadline = rb_servo::nowSteadyNs() + 1'000'000'000;
    const rb_servo::ArmSendResult enqueue =
        worker.enqueueAsyncServoJ(request(1020, joints(20.0), deadline));
    RB_CHECK(enqueue.result.accepted);
    RB_CHECK(enqueue.result.controller_acceptance_observed == false);
    RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));

    rb_servo::RbpodoAsyncStreamingTelemetry async = worker.asyncStreamingTelemetry();
    worker.stop();
    RB_CHECK(async.commands_enqueued_total == 1);
    RB_CHECK(async.commands_sent_total == 1);
    RB_CHECK(async.commands_acked_total == 1);
    RB_CHECK(async.commands_socket_sent_total == 0);
    RB_CHECK(async.last_ack_seq == 1020);
    RB_CHECK(async.last_ack_result == "controller_ack_observed");
    RB_CHECK(async.last_async_ack_duration_us >= 0.0);
    RB_CHECK(async.supervision_state == rb_servo::RbpodoAsyncStreamingSupervisionState::Ok);
    return true;
}

bool testAsyncTimingRejectFaultsImmediatelyByDefault() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        false,
        false,
        false,
        false,
        std::chrono::milliseconds(20)
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorkerOptions options =
        asyncWorkerOptions(rb_servo::RbpodoAsyncStreamingMode::SdkAckWorker);
    options.controller_simulation_timing_reject_tolerance_enabled = false;
    rb_servo::ArmWorker worker(std::move(backend), options);
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));

    rb_servo::SendServoJRequest req =
        request(1025, joints(25.0), rb_servo::nowSteadyNs() + 5'000'000);
    const rb_servo::ArmSendResult enqueue = worker.enqueueAsyncServoJ(req);
    RB_CHECK(enqueue.result.accepted);
    RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));
    const std::optional<rb_servo::ArmSendResult> last =
        worker.waitForSendResult(req, enqueue.dispatch_timing.end_ns + 1, rb_servo::nowSteadyNs() + 500'000'000);
    const rb_servo::RbpodoAsyncStreamingTelemetry async = worker.asyncStreamingTelemetry();
    worker.stop();

    RB_CHECK(last.has_value());
    RB_CHECK(!last->result.accepted);
    RB_CHECK(last->result.error.kind == rb_servo::BackendErrorKind::CommandTimeout);
    RB_CHECK(last->result.error.name == "arm_worker_send_result_timeout");
    RB_CHECK(async.ack_timeout_count == 1);
    RB_CHECK(async.supervision_state == rb_servo::RbpodoAsyncStreamingSupervisionState::Fault);
    return true;
}

bool testAsyncControllerSimulationTimingRejectUsesConsecutiveTolerance() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Right,
        rb_servo::BackendErrorKind::None,
        false,
        false,
        false,
        false,
        std::chrono::milliseconds(20)
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorkerOptions options =
        asyncWorkerOptions(rb_servo::RbpodoAsyncStreamingMode::SdkAckWorker);
    options.controller_simulation_timing_reject_tolerance_enabled = true;
    options.rbpodo_async_ack_supervision.max_consecutive_missing_ack = 2;
    rb_servo::ArmWorker worker(std::move(backend), options);
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));

    rb_servo::SendServoJRequest first =
        request(1026, joints(26.0), rb_servo::nowSteadyNs() + 5'000'000);
    const rb_servo::ArmSendResult first_enqueue = worker.enqueueAsyncServoJ(first);
    RB_CHECK(first_enqueue.result.accepted);
    RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));
    const std::optional<rb_servo::ArmSendResult> first_last =
        worker.waitForSendResult(
            first,
            first_enqueue.dispatch_timing.end_ns + 1,
            rb_servo::nowSteadyNs() + 500'000'000
        );
    RB_CHECK(first_last.has_value());
    RB_CHECK(!first_last->result.accepted);
    rb_servo::RbpodoAsyncStreamingTelemetry async = worker.asyncStreamingTelemetry();
    RB_CHECK(async.ack_timeout_count == 1);
    RB_CHECK(async.supervision_state != rb_servo::RbpodoAsyncStreamingSupervisionState::Fault);

    rb_servo::SendServoJRequest second =
        request(1027, joints(27.0), rb_servo::nowSteadyNs() + 5'000'000);
    const rb_servo::ArmSendResult second_enqueue = worker.enqueueAsyncServoJ(second);
    RB_CHECK(second_enqueue.result.accepted);
    RB_CHECK(raw_backend->waitForSends(2, std::chrono::milliseconds(200)));
    const std::optional<rb_servo::ArmSendResult> second_last =
        worker.waitForSendResult(
            second,
            second_enqueue.dispatch_timing.end_ns + 1,
            rb_servo::nowSteadyNs() + 500'000'000
        );
    async = worker.asyncStreamingTelemetry();
    worker.stop();

    RB_CHECK(second_last.has_value());
    RB_CHECK(!second_last->result.accepted);
    RB_CHECK(second_last->result.error.kind == rb_servo::BackendErrorKind::CommandTimeout);
    RB_CHECK(second_last->result.error.name == "arm_worker_send_result_timeout");
    RB_CHECK(async.ack_timeout_count == 2);
    RB_CHECK(async.supervision_state == rb_servo::RbpodoAsyncStreamingSupervisionState::Fault);
    return true;
}

bool testAsyncSocketSendSupervisedRecordsSocketSendOnly() {
    auto backend = std::make_unique<WorkerTestBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        false,
        true
    );
    WorkerTestBackend* raw_backend = backend.get();
    rb_servo::ArmWorker worker(
        std::move(backend),
        asyncWorkerOptions(rb_servo::RbpodoAsyncStreamingMode::SocketSendSupervised)
    );
    RB_CHECK(worker.start());
    RB_CHECK(raw_backend->waitForReads(1, std::chrono::milliseconds(200)));

    const uint64_t deadline = rb_servo::nowSteadyNs() + 1'000'000'000;
    const rb_servo::ArmSendResult enqueue =
        worker.enqueueAsyncServoJ(request(1030, joints(30.0), deadline));
    RB_CHECK(enqueue.result.accepted);
    RB_CHECK(enqueue.result.ack_policy == rb_servo::BackendAckPolicy::Disabled);
    RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));

    const rb_servo::RbpodoAsyncStreamingTelemetry async = worker.asyncStreamingTelemetry();
    worker.stop();
    RB_CHECK(async.commands_enqueued_total == 1);
    RB_CHECK(async.commands_sent_total == 1);
    RB_CHECK(async.commands_socket_sent_total == 1);
    RB_CHECK(async.commands_acked_total == 0);
    RB_CHECK(async.last_ack_seq == 0);
    RB_CHECK(async.last_async_acceptance_semantics == "socket_send_only");
    RB_CHECK(async.last_controller_acceptance_semantics == "socket_send_only");
    return true;
}

}  // namespace

int main() {
    if (!testSuccessfulReadLoop()) return 1;
    if (!testInitializeStateIsPublishedBeforeFirstReadCompletes()) return 1;
    if (!testConnectStateIsPublishedBeforeInitializeCompletes()) return 1;
    if (!testStartupTelemetryReportsBlockedConnect()) return 1;
    if (!testDefaultReadPeriodIsConservative()) return 1;
    if (!testLatestStateReportsStaleSample()) return 1;
    if (!testSendRequestAccepted()) return 1;
    if (!testExpiredCommandDropped()) return 1;
    if (!testBackendSendFailurePreserved()) return 1;
    if (!testLatestQueuedCommandWins()) return 1;
    if (!testNoDropCountedAfterImmediateDispatch()) return 1;
    if (!testResetFaultUsesLifecycleQueue()) return 1;
    if (!testSetFreedriveUsesLifecycleQueue()) return 1;
    if (!testLifecycleQueueFullIsExplicit()) return 1;
    if (!testStopJoinsThreadAndRejectsNewCommand()) return 1;
    if (!testNoDeadlockOnDestruction()) return 1;
    if (!testStopWithPendingCommandBehindReadDoesNotDeadlock()) return 1;
    if (!testAsyncLatestWinsOverwriteTelemetry()) return 1;
    if (!testAsyncSdkAckWorkerRecordsAckObservedResults()) return 1;
    if (!testAsyncTimingRejectFaultsImmediatelyByDefault()) return 1;
    if (!testAsyncControllerSimulationTimingRejectUsesConsecutiveTolerance()) return 1;
    if (!testAsyncSocketSendSupervisedRecordsSocketSendOnly()) return 1;
    return 0;
}
