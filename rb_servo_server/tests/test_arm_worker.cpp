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
        bool block_first_read = false
    ) : arm_id_(arm_id),
        send_error_kind_(send_error_kind),
        block_first_read_(block_first_read),
        state_(validState(arm_id, joints(0.0))) {}

    rb_servo::BackendResult<rb_servo::RobotState> connect() override {
        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = true;
        ++connect_count_;
        return backendStateResult(rb_servo::BackendOp::Connect, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> initialize() override {
        std::lock_guard<std::mutex> lock(mutex_);
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

        std::lock_guard<std::mutex> lock(mutex_);
        if (send_error_kind_ == rb_servo::BackendErrorKind::None) {
            return rb_servo::acceptedSend(request, {}, state_, "cache");
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
        return backendStateResult(rb_servo::BackendOp::Stop, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> resetFault() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return backendStateResult(rb_servo::BackendOp::ResetFault, state_);
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

    bool waitForFirstReadEntered(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this] {
            return first_read_waiting_;
        });
    }

    void releaseFirstRead() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            first_read_released_ = true;
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

    std::optional<rb_servo::SendServoJRequest> lastRequest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_request_;
    }

private:
    rb_servo::ArmId arm_id_;
    rb_servo::BackendErrorKind send_error_kind_;
    bool block_first_read_ = false;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool connected_ = false;
    bool initialized_ = false;
    bool first_read_waiting_ = false;
    bool first_read_released_ = false;
    int connect_count_ = 0;
    int initialize_count_ = 0;
    int read_count_ = 0;
    int send_count_ = 0;
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
    worker.enqueueServoJ(request(12, joints(12.0), rb_servo::nowSteadyNs() + 1'000'000'000));
    raw_backend->releaseFirstRead();

    RB_CHECK(raw_backend->waitForSends(1, std::chrono::milliseconds(200)));
    const std::optional<rb_servo::ArmSendResult> last = worker.lastSendResult();
    RB_CHECK(last.has_value());
    RB_CHECK(last->request.command_seq == 12);
    RB_CHECK(last->result.accepted);
    RB_CHECK(raw_backend->sendCount() == 1);
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

}  // namespace

int main() {
    if (!testSuccessfulReadLoop()) return 1;
    if (!testLatestStateReportsStaleSample()) return 1;
    if (!testSendRequestAccepted()) return 1;
    if (!testExpiredCommandDropped()) return 1;
    if (!testBackendSendFailurePreserved()) return 1;
    if (!testLatestQueuedCommandWins()) return 1;
    if (!testStopJoinsThreadAndRejectsNewCommand()) return 1;
    if (!testNoDeadlockOnDestruction()) return 1;
    if (!testStopWithPendingCommandBehindReadDoesNotDeadlock()) return 1;
    return 0;
}
