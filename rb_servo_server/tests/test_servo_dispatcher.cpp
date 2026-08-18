#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rb_servo/control/servo_dispatcher.hpp"
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

class DispatchBackend final : public rb_servo::IRobotBackend {
public:
    DispatchBackend(
        rb_servo::ArmId arm_id,
        rb_servo::BackendErrorKind send_error_kind = rb_servo::BackendErrorKind::None,
        std::chrono::microseconds send_delay = std::chrono::microseconds(0)
    ) : arm_id_(arm_id),
        send_error_kind_(send_error_kind),
        send_delay_(send_delay),
        state_(validState(arm_id, joints(0.0))) {}

    rb_servo::BackendResult<rb_servo::RobotState> connect() override {
        return backendStateResult(rb_servo::BackendOp::Connect, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> initialize() override {
        return backendStateResult(rb_servo::BackendOp::Initialize, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> readState() override {
        return backendStateResult(rb_servo::BackendOp::ReadState, state_);
    }

    rb_servo::SendServoJResult sendServoJ(const rb_servo::SendServoJRequest& request) override {
        last_request_ = request;
        ++send_count_;
        if (send_delay_.count() > 0) {
            std::this_thread::sleep_for(send_delay_);
        }
        if (send_error_kind_ == rb_servo::BackendErrorKind::None) {
            state_.q_target_deg = request.q_target_deg;
            state_.q_actual_deg = request.q_target_deg;
            return rb_servo::acceptedSend(request, {}, state_, "cache");
        }

        if (send_error_kind_ == rb_servo::BackendErrorKind::RobotFault) {
            state_.has_error = true;
            state_.error_code = 2222;
        }
        return rb_servo::rejectedSend(
            request,
            rb_servo::backendError(
                send_error_kind_,
                "test dispatch failure",
                send_error_kind_ == rb_servo::BackendErrorKind::RobotFault ? "2222" : "",
                "test_dispatch_failure"
            ),
            {},
            send_error_kind_ == rb_servo::BackendErrorKind::RobotFault
                ? std::optional<rb_servo::RobotState>(state_)
                : std::nullopt,
            send_error_kind_ == rb_servo::BackendErrorKind::RobotFault ? "cache" : "none"
        );
    }

    rb_servo::BackendResult<rb_servo::RobotState> stop() override {
        return backendStateResult(rb_servo::BackendOp::Stop, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> resetFault() override {
        state_.has_error = false;
        state_.error_code = 0;
        return backendStateResult(rb_servo::BackendOp::ResetFault, state_);
    }

    bool isConnected() const override { return true; }
    rb_servo::ArmId armId() const override { return arm_id_; }
    std::string name() const override { return "dispatch_test_backend"; }

    const rb_servo::SendServoJRequest& lastRequest() const { return last_request_; }
    // Backend-level send tally. A withheld send must not reach the backend at
    // all, so this counter -- not the returned result -- is what proves a skip.
    int sendCount() const { return send_count_; }

private:
    rb_servo::ArmId arm_id_;
    rb_servo::BackendErrorKind send_error_kind_;
    std::chrono::microseconds send_delay_;
    rb_servo::RobotState state_;
    rb_servo::SendServoJRequest last_request_;
    int send_count_ = 0;
};

class WorkerDispatchBackend final : public rb_servo::IRobotBackend {
public:
    WorkerDispatchBackend(
        rb_servo::ArmId arm_id,
        rb_servo::BackendErrorKind send_error_kind = rb_servo::BackendErrorKind::None,
        std::chrono::milliseconds send_delay = std::chrono::milliseconds(0)
    ) : arm_id_(arm_id),
        send_error_kind_(send_error_kind),
        send_delay_(send_delay),
        state_(validState(arm_id, joints(0.0))) {}

    rb_servo::BackendResult<rb_servo::RobotState> connect() override {
        return backendStateResult(rb_servo::BackendOp::Connect, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> initialize() override {
        return backendStateResult(rb_servo::BackendOp::Initialize, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> readState() override {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.host_time_ns = rb_servo::nowSteadyNs();
        return backendStateResult(rb_servo::BackendOp::ReadState, state_);
    }

    rb_servo::SendServoJResult sendServoJ(const rb_servo::SendServoJRequest& request) override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++send_count_;
            last_request_ = request;
        }
        cv_.notify_all();

        if (send_delay_.count() > 0) {
            std::this_thread::sleep_for(send_delay_);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (send_error_kind_ == rb_servo::BackendErrorKind::None) {
            state_.q_target_deg = request.q_target_deg;
            state_.q_actual_deg = request.q_target_deg;
            state_.host_time_ns = rb_servo::nowSteadyNs();
            return rb_servo::acceptedSend(request, {}, state_, "cache");
        }

        return rb_servo::rejectedSend(
            request,
            rb_servo::backendError(
                send_error_kind_,
                "worker dispatch test failure",
                "",
                "worker_dispatch_failure"
            )
        );
    }

    rb_servo::BackendResult<rb_servo::RobotState> stop() override {
        return backendStateResult(rb_servo::BackendOp::Stop, state_);
    }

    rb_servo::BackendResult<rb_servo::RobotState> resetFault() override {
        return backendStateResult(rb_servo::BackendOp::ResetFault, state_);
    }

    bool isConnected() const override { return true; }
    rb_servo::ArmId armId() const override { return arm_id_; }
    std::string name() const override { return "worker_dispatch_backend"; }

    bool waitForSends(int expected, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this, expected] {
            return send_count_ >= expected;
        });
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
    std::chrono::milliseconds send_delay_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    rb_servo::RobotState state_;
    int send_count_ = 0;
    std::optional<rb_servo::SendServoJRequest> last_request_;
};

bool waitForWorkerState(rb_servo::ArmWorker& worker) {
    for (int i = 0; i < 200; ++i) {
        if (worker.latestState(1'000'000'000).ok) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

rb_servo::ServoDispatchRequest workerRequest(uint64_t seq, uint64_t host_time_ns, uint64_t deadline_ns) {
    rb_servo::ServoDispatchRequest request;
    request.seq = seq;
    request.dispatch_start_ns = host_time_ns;
    request.deadline_ns = deadline_ns;
    request.left.host_time_ns = host_time_ns;
    request.right.host_time_ns = host_time_ns;
    request.left.q_target_deg = joints(1.0);
    request.right.q_target_deg = joints(2.0);
    return request;
}

rb_servo::ArmWorkerOptions asyncWorkerOptions() {
    rb_servo::ArmWorkerOptions options;
    options.read_period_ns = 1'000'000;
    options.rbpodo_async_streaming_enabled = true;
    options.rbpodo_async_streaming_mode = rb_servo::RbpodoAsyncStreamingMode::SdkAckWorker;
    return options;
}

bool testDirectSequentialPopulatesTimingAndPreservesResults() {
    DispatchBackend left(rb_servo::ArmId::Left, rb_servo::BackendErrorKind::None, std::chrono::microseconds(200));
    DispatchBackend right(rb_servo::ArmId::Right, rb_servo::BackendErrorKind::TransportWriteFailed);

    rb_servo::ServoDispatchRequest request;
    request.seq = 42;
    request.dispatch_start_ns = 1000;
    request.deadline_ns = 9000;
    request.left.q_target_deg = joints(1.0);
    request.right.q_target_deg = joints(2.0);

    const rb_servo::DualSendResult result =
        rb_servo::ServoDispatcher::dispatchDirectSequential(left, right, request);

    RB_CHECK(result.dispatch_start_ns == 1000);
    RB_CHECK(result.dispatch_end_ns >= result.dispatch_start_ns);
    RB_CHECK(result.timing.start_ns == result.dispatch_start_ns);
    RB_CHECK(result.timing.end_ns == result.dispatch_end_ns);
    RB_CHECK(result.timing.duration_us >= 0.0);
    RB_CHECK(result.left.dispatch_timing.start_ns > 0);
    RB_CHECK(result.left.dispatch_timing.end_ns >= result.left.dispatch_timing.start_ns);
    RB_CHECK(result.left.dispatch_timing.duration_us > 0.0);
    RB_CHECK(result.left.result.timing.duration_us > 0.0);
    RB_CHECK(result.right.dispatch_timing.start_ns > 0);
    RB_CHECK(result.right.dispatch_timing.end_ns >= result.right.dispatch_timing.start_ns);
    RB_CHECK(result.left_right_start_skew_us > 0.0);
    RB_CHECK(result.left_right_end_skew_us >= 0.0);

    RB_CHECK(result.left.arm_id == rb_servo::ArmId::Left);
    RB_CHECK(result.right.arm_id == rb_servo::ArmId::Right);
    RB_CHECK(result.left.result.accepted);
    RB_CHECK(!result.right.result.accepted);
    RB_CHECK(result.right.result.error.kind == rb_servo::BackendErrorKind::TransportWriteFailed);
    RB_CHECK(result.any_transport_failure());
    RB_CHECK(!result.any_robot_fault());

    RB_CHECK(result.left.request.command_seq == 42);
    RB_CHECK(result.right.request.command_seq == 42);
    RB_CHECK(result.left.request.deadline_ns == 9000);
    RB_CHECK(result.right.request.deadline_ns == 9000);
    RB_CHECK(result.left.request.host_time_ns == 1000);
    RB_CHECK(result.right.request.host_time_ns == 1000);
    RB_CHECK(result.left.request.host_time_ns == left.lastRequest().host_time_ns);
    RB_CHECK(result.right.request.host_time_ns == right.lastRequest().host_time_ns);
    RB_CHECK(sameJointArray(result.left.request.q_target_deg, joints(1.0)));
    RB_CHECK(sameJointArray(result.right.request.q_target_deg, joints(2.0)));
    RB_CHECK(sameJointArray(result.left.result.requested_q_deg, joints(1.0)));
    RB_CHECK(sameJointArray(result.right.result.requested_q_deg, joints(2.0)));
    return true;
}

bool testWorkerDispatchBothAccepted() {
    auto left_backend = std::make_unique<WorkerDispatchBackend>(rb_servo::ArmId::Left);
    auto right_backend = std::make_unique<WorkerDispatchBackend>(rb_servo::ArmId::Right);
    WorkerDispatchBackend* left_raw = left_backend.get();
    WorkerDispatchBackend* right_raw = right_backend.get();
    rb_servo::ArmWorker left(std::move(left_backend));
    rb_servo::ArmWorker right(std::move(right_backend));
    RB_CHECK(left.start());
    RB_CHECK(right.start());
    RB_CHECK(waitForWorkerState(left));
    RB_CHECK(waitForWorkerState(right));

    const uint64_t host_time_ns = rb_servo::nowSteadyNs();
    const rb_servo::DualSendResult result = rb_servo::ServoDispatcher::dispatchWorker(
        left,
        right,
        workerRequest(100, host_time_ns, host_time_ns + 500'000'000)
    );

    left.stop();
    right.stop();
    RB_CHECK(result.left.result.accepted);
    RB_CHECK(result.right.result.accepted);
    RB_CHECK(result.left.request.command_seq == 100);
    RB_CHECK(result.right.request.command_seq == 100);
    RB_CHECK(result.left.request.host_time_ns == host_time_ns);
    RB_CHECK(result.right.request.host_time_ns == host_time_ns);
    RB_CHECK(result.left.result.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(result.right.result.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(!rb_servo::ServoDispatcher::deadlineMissed(result));
    RB_CHECK(result.left_right_start_skew_us < 10'000.0);
    RB_CHECK(left_raw->sendCount() == 1);
    RB_CHECK(right_raw->sendCount() == 1);
    return true;
}

bool testWorkerDispatchSlowArmDoesNotHideFastArm() {
    auto left_backend = std::make_unique<WorkerDispatchBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        std::chrono::milliseconds(60)
    );
    auto right_backend = std::make_unique<WorkerDispatchBackend>(rb_servo::ArmId::Right);
    WorkerDispatchBackend* left_raw = left_backend.get();
    WorkerDispatchBackend* right_raw = right_backend.get();
    rb_servo::ArmWorker left(std::move(left_backend));
    rb_servo::ArmWorker right(std::move(right_backend));
    RB_CHECK(left.start());
    RB_CHECK(right.start());
    RB_CHECK(waitForWorkerState(left));
    RB_CHECK(waitForWorkerState(right));

    const uint64_t host_time_ns = rb_servo::nowSteadyNs();
    const rb_servo::DualSendResult result = rb_servo::ServoDispatcher::dispatchWorker(
        left,
        right,
        workerRequest(101, host_time_ns, host_time_ns + 20'000'000)
    );

    RB_CHECK(left_raw->waitForSends(1, std::chrono::milliseconds(50)));
    RB_CHECK(right_raw->sendCount() == 1);
    left.stop();
    right.stop();
    RB_CHECK(!result.left.result.accepted);
    RB_CHECK(result.left.result.error.kind == rb_servo::BackendErrorKind::CommandTimeout);
    RB_CHECK(result.right.result.accepted);
    RB_CHECK(result.right.result.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(rb_servo::ServoDispatcher::armDeadlineMissed(result.left));
    RB_CHECK(rb_servo::ServoDispatcher::deadlineMissed(result));
    RB_CHECK(result.left_right_start_skew_us < 10'000.0);
    return true;
}

bool testWorkerDispatchTransportFailureDoesNotHideAcceptedArm() {
    auto left_backend = std::make_unique<WorkerDispatchBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::TransportWriteFailed
    );
    auto right_backend = std::make_unique<WorkerDispatchBackend>(rb_servo::ArmId::Right);
    rb_servo::ArmWorker left(std::move(left_backend));
    rb_servo::ArmWorker right(std::move(right_backend));
    RB_CHECK(left.start());
    RB_CHECK(right.start());
    RB_CHECK(waitForWorkerState(left));
    RB_CHECK(waitForWorkerState(right));

    const uint64_t host_time_ns = rb_servo::nowSteadyNs();
    const rb_servo::DualSendResult result = rb_servo::ServoDispatcher::dispatchWorker(
        left,
        right,
        workerRequest(102, host_time_ns, host_time_ns + 500'000'000)
    );

    left.stop();
    right.stop();
    RB_CHECK(!result.left.result.accepted);
    RB_CHECK(result.left.result.error.kind == rb_servo::BackendErrorKind::TransportWriteFailed);
    RB_CHECK(result.left.result.error.transport_fault);
    RB_CHECK(result.right.result.accepted);
    RB_CHECK(result.right.result.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(result.any_transport_failure());
    RB_CHECK(!result.any_robot_fault());
    return true;
}

bool testDirectSequentialDeadlineMissFlag() {
    DispatchBackend left(rb_servo::ArmId::Left, rb_servo::BackendErrorKind::None, std::chrono::microseconds(2000));
    DispatchBackend right(rb_servo::ArmId::Right);
    rb_servo::ServoDispatchRequest request;
    request.seq = 103;
    request.dispatch_start_ns = rb_servo::nowSteadyNs();
    request.deadline_ns = request.dispatch_start_ns + 1000;
    request.left.host_time_ns = request.dispatch_start_ns;
    request.right.host_time_ns = request.dispatch_start_ns;
    request.left.q_target_deg = joints(1.0);
    request.right.q_target_deg = joints(2.0);

    const rb_servo::DualSendResult result =
        rb_servo::ServoDispatcher::dispatchDirectSequential(left, right, request);

    RB_CHECK(result.left.result.accepted);
    RB_CHECK(result.right.result.accepted);
    RB_CHECK(rb_servo::ServoDispatcher::armDeadlineMissed(result.left));
    RB_CHECK(rb_servo::ServoDispatcher::deadlineMissed(result));
    return true;
}

bool testRobotFaultHelper() {
    DispatchBackend left(rb_servo::ArmId::Left);
    DispatchBackend right(rb_servo::ArmId::Right, rb_servo::BackendErrorKind::RobotFault);

    rb_servo::ServoDispatchRequest request;
    request.seq = 7;
    request.left.q_target_deg = joints(3.0);
    request.right.q_target_deg = joints(4.0);

    const rb_servo::DualSendResult result =
        rb_servo::ServoDispatcher::dispatchDirectSequential(left, right, request);

    RB_CHECK(!result.any_transport_failure());
    RB_CHECK(result.any_robot_fault());
    RB_CHECK(!result.right.result.accepted);
    RB_CHECK(result.right.result.state_after.has_value());
    RB_CHECK(result.right.result.state_after_source == "cache");
    return true;
}

bool testRbpodoAsyncDispatchDoesNotWaitForSlowAckWorker() {
    auto left_backend = std::make_unique<WorkerDispatchBackend>(
        rb_servo::ArmId::Left,
        rb_servo::BackendErrorKind::None,
        std::chrono::milliseconds(120)
    );
    auto right_backend = std::make_unique<WorkerDispatchBackend>(
        rb_servo::ArmId::Right,
        rb_servo::BackendErrorKind::None,
        std::chrono::milliseconds(120)
    );
    WorkerDispatchBackend* left_raw = left_backend.get();
    WorkerDispatchBackend* right_raw = right_backend.get();
    rb_servo::ArmWorker left(std::move(left_backend), asyncWorkerOptions());
    rb_servo::ArmWorker right(std::move(right_backend), asyncWorkerOptions());
    RB_CHECK(left.start());
    RB_CHECK(right.start());
    RB_CHECK(waitForWorkerState(left));
    RB_CHECK(waitForWorkerState(right));

    const uint64_t host_time_ns = rb_servo::nowSteadyNs();
    const auto started = std::chrono::steady_clock::now();
    const rb_servo::DualSendResult result = rb_servo::ServoDispatcher::dispatchRbpodoAsync(
        left,
        right,
        workerRequest(104, host_time_ns, host_time_ns + 500'000'000)
    );
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started
    );

    RB_CHECK(elapsed.count() < 50);
    RB_CHECK(result.left.result.accepted);
    RB_CHECK(result.right.result.accepted);
    RB_CHECK(result.left.result.acceptance_semantics == "async_enqueued");
    RB_CHECK(result.right.result.acceptance_semantics == "async_enqueued");
    RB_CHECK(result.left.dispatch_timing.duration_us < 50'000.0);
    RB_CHECK(result.right.dispatch_timing.duration_us < 50'000.0);
    RB_CHECK(left_raw->waitForSends(1, std::chrono::milliseconds(500)));
    RB_CHECK(right_raw->waitForSends(1, std::chrono::milliseconds(500)));
    left.stop();
    right.stop();
    return true;
}


// A skip must reach exactly one box. This is the whole mechanism of the per-arm
// queue level trim: the skipped arm's queue drops by one because we did not
// append, and the other arm is untouched.
bool testRbpodoAsyncSkipWithholdsOnlyTheSkippedArm() {
    auto left_backend = std::make_unique<WorkerDispatchBackend>(rb_servo::ArmId::Left);
    auto right_backend = std::make_unique<WorkerDispatchBackend>(rb_servo::ArmId::Right);
    WorkerDispatchBackend* left_raw = left_backend.get();
    WorkerDispatchBackend* right_raw = right_backend.get();
    rb_servo::ArmWorker left(std::move(left_backend), asyncWorkerOptions());
    rb_servo::ArmWorker right(std::move(right_backend), asyncWorkerOptions());
    RB_CHECK(left.start());
    RB_CHECK(right.start());
    RB_CHECK(waitForWorkerState(left));
    RB_CHECK(waitForWorkerState(right));

    const uint64_t host_time_ns = rb_servo::nowSteadyNs();
    rb_servo::ServoDispatchRequest request =
        workerRequest(105, host_time_ns, host_time_ns + 500'000'000);
    request.skip_right = true;
    const rb_servo::DualSendResult result =
        rb_servo::ServoDispatcher::dispatchRbpodoAsync(left, right, request);

    RB_CHECK(left_raw->waitForSends(1, std::chrono::milliseconds(500)));
    // Give the right worker the same wall-clock chance to send before asserting
    // that it did not, so this cannot pass merely by being fast.
    RB_CHECK(!right_raw->waitForSends(1, std::chrono::milliseconds(100)));
    RB_CHECK(right_raw->sendCount() == 0);

    // Reported accepted on purpose: nothing failed, we chose not to transmit.
    // Marking it rejected would latch a send fault on a healthy arm.
    RB_CHECK(result.right.result.accepted);
    RB_CHECK(result.right.result.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(result.right.result.acceptance_semantics == "skipped_queue_level");
    RB_CHECK(result.right.arm_id == rb_servo::ArmId::Right);
    // A command that was never sent gets no RBACK: the cadence controller has to
    // see "not observed", never a stale depth.
    RB_CHECK(!result.right.result.box_queue_fill.has_value());
    RB_CHECK(result.right.result.box_queue_fill_samples == 0);

    RB_CHECK(result.left.result.accepted);
    RB_CHECK(result.left.result.acceptance_semantics == "async_enqueued");

    left.stop();
    right.stop();
    return true;
}

// Default-constructed requests must never withhold anything: the skip is opt-in
// from the cadence controller, not a state the dispatcher can fall into.
bool testRbpodoAsyncSendsBothArmsWhenNoSkipRequested() {
    auto left_backend = std::make_unique<WorkerDispatchBackend>(rb_servo::ArmId::Left);
    auto right_backend = std::make_unique<WorkerDispatchBackend>(rb_servo::ArmId::Right);
    WorkerDispatchBackend* left_raw = left_backend.get();
    WorkerDispatchBackend* right_raw = right_backend.get();
    rb_servo::ArmWorker left(std::move(left_backend), asyncWorkerOptions());
    rb_servo::ArmWorker right(std::move(right_backend), asyncWorkerOptions());
    RB_CHECK(left.start());
    RB_CHECK(right.start());
    RB_CHECK(waitForWorkerState(left));
    RB_CHECK(waitForWorkerState(right));

    const uint64_t host_time_ns = rb_servo::nowSteadyNs();
    const rb_servo::ServoDispatchRequest request =
        workerRequest(106, host_time_ns, host_time_ns + 500'000'000);
    RB_CHECK(!request.skip_left);
    RB_CHECK(!request.skip_right);
    rb_servo::ServoDispatcher::dispatchRbpodoAsync(left, right, request);

    RB_CHECK(left_raw->waitForSends(1, std::chrono::milliseconds(500)));
    RB_CHECK(right_raw->waitForSends(1, std::chrono::milliseconds(500)));
    left.stop();
    right.stop();
    return true;
}


// THE path that matters: stack_real.yaml runs io_model: direct, so this is where
// a real skip actually withholds a servo_j. The async-only version of this
// feature shipped once and did nothing on hardware -- 2791 counted skips, zero
// withheld sends, 100% RBACK observation. Keep this test.
bool testDirectSequentialSkipWithholdsOnlyTheSkippedArm() {
    DispatchBackend left(rb_servo::ArmId::Left);
    DispatchBackend right(rb_servo::ArmId::Right);

    rb_servo::ServoDispatchRequest request;
    request.seq = 200;
    request.left.q_target_deg = joints(1.0);
    request.right.q_target_deg = joints(2.0);
    request.dispatch_start_ns = rb_servo::nowSteadyNs();
    request.skip_right = true;

    const rb_servo::DualSendResult result =
        rb_servo::ServoDispatcher::dispatchDirectSequential(left, right, request);

    RB_CHECK(left.sendCount() == 1);
    RB_CHECK(right.sendCount() == 0);
    RB_CHECK(result.right.result.accepted);
    RB_CHECK(result.right.result.error.kind == rb_servo::BackendErrorKind::None);
    RB_CHECK(result.right.result.acceptance_semantics == "skipped_queue_level");
    RB_CHECK(!result.right.result.box_queue_fill.has_value());
    RB_CHECK(result.right.arm_id == rb_servo::ArmId::Right);
    RB_CHECK(result.left.result.accepted);
    RB_CHECK(result.left.result.acceptance_semantics != "skipped_queue_level");

    // The other arm's skip must be independent, not a shared flag.
    DispatchBackend left2(rb_servo::ArmId::Left);
    DispatchBackend right2(rb_servo::ArmId::Right);
    rb_servo::ServoDispatchRequest left_skip = request;
    left_skip.skip_right = false;
    left_skip.skip_left = true;
    rb_servo::ServoDispatcher::dispatchDirectSequential(left2, right2, left_skip);
    RB_CHECK(left2.sendCount() == 0);
    RB_CHECK(right2.sendCount() == 1);
    return true;
}

}  // namespace

int main() {
    if (!testDirectSequentialPopulatesTimingAndPreservesResults()) return 1;
    if (!testWorkerDispatchBothAccepted()) return 1;
    if (!testWorkerDispatchSlowArmDoesNotHideFastArm()) return 1;
    if (!testWorkerDispatchTransportFailureDoesNotHideAcceptedArm()) return 1;
    if (!testDirectSequentialDeadlineMissFlag()) return 1;
    if (!testRobotFaultHelper()) return 1;
    if (!testRbpodoAsyncDispatchDoesNotWaitForSlowAckWorker()) return 1;
    if (!testRbpodoAsyncSkipWithholdsOnlyTheSkippedArm()) return 1;
    if (!testRbpodoAsyncSendsBothArmsWhenNoSkipRequested()) return 1;
    if (!testDirectSequentialSkipWithholdsOnlyTheSkippedArm()) return 1;
    return 0;
}
