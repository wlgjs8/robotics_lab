#include "rb_servo/control/servo_dispatcher.hpp"

#include <optional>

#include "rb_servo/core/clock.hpp"

namespace rb_servo {
namespace {

double absDurationUs(uint64_t a_ns, uint64_t b_ns) {
    const uint64_t diff_ns = a_ns > b_ns ? a_ns - b_ns : b_ns - a_ns;
    return static_cast<double>(diff_ns) / 1000.0;
}

bool hasTransportFailure(const ArmSendResult& result) {
    return !result.result.accepted && result.result.error.transport_fault;
}

bool hasRobotFault(const ArmSendResult& result) {
    return !result.result.accepted && result.result.error.robot_fault;
}

bool matchesRequest(
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

uint64_t requestHostTime(uint64_t requested_host_time_ns, uint64_t fallback_ns) {
    return requested_host_time_ns > 0 ? requested_host_time_ns : fallback_ns;
}

SendServoJResult withDispatchTiming(SendServoJResult result, const BackendTiming& timing) {
    if (result.timing.start_ns == 0 && result.timing.end_ns == 0) {
        result.timing = timing;
    }
    return result;
}

ArmSendResult timeoutResult(
    ArmId arm_id,
    const SendServoJRequest& request,
    uint64_t start_ns,
    uint64_t end_ns
) {
    ArmSendResult result;
    result.arm_id = arm_id;
    result.request = request;
    result.dispatch_timing = makeBackendTiming(start_ns, end_ns);
    result.result = rejectedSend(
        request,
        backendError(
            BackendErrorKind::CommandTimeout,
            "arm worker did not publish send result before command deadline",
            "",
            "arm_worker_send_result_timeout"
        ),
        result.dispatch_timing
    );
    return result;
}

// A withheld send: the per-arm queue level trim chose not to transmit this tick,
// so this arm's box command queue drops by one. Reported as ACCEPTED on purpose: nothing failed, we chose not
// to transmit, and marking it rejected would latch a send fault and poison the
// backend-error telemetry. The distinct acceptance_semantics is how a skip is
// told apart from a real send downstream and in the CSV. box_queue_fill stays
// nullopt because a command that was never sent gets no RBACK, and the cadence
// controller must see "not observed" rather than a stale depth.
SendServoJResult skippedSend(const SendServoJRequest& request) {
    SendServoJResult result = acceptedSend(request);
    result.acceptance_semantics = "skipped_queue_level";
    result.ack_observed = false;
    result.controller_acceptance_observed = false;
    result.box_queue_fill = std::nullopt;
    result.box_queue_fill_samples = 0;
    result.requested_q_deg = request.q_target_deg;
    return result;
}

ArmSendResult skippedResult(
    ArmId arm_id,
    const SendServoJRequest& request,
    uint64_t start_ns,
    uint64_t end_ns
) {
    ArmSendResult result;
    result.arm_id = arm_id;
    result.request = request;
    result.dispatch_timing = makeBackendTiming(start_ns, end_ns);
    result.result = skippedSend(request);
    result.result.timing = result.dispatch_timing;
    return result;
}

}  // namespace

bool DualSendResult::any_transport_failure() const {
    return hasTransportFailure(left) || hasTransportFailure(right);
}

bool DualSendResult::any_robot_fault() const {
    return hasRobotFault(left) || hasRobotFault(right);
}

bool ServoDispatcher::armDeadlineMissed(const ArmSendResult& result) {
    return result.request.deadline_ns > 0 &&
           result.dispatch_timing.end_ns > result.request.deadline_ns;
}

bool ServoDispatcher::deadlineMissed(const DualSendResult& result) {
    return armDeadlineMissed(result.left) || armDeadlineMissed(result.right);
}

DualSendResult ServoDispatcher::dispatchDirectSequential(
    IRobotBackend& left_backend,
    IRobotBackend& right_backend,
    const ServoDispatchRequest& request
) {
    DualSendResult result;
    result.dispatch_start_ns = request.dispatch_start_ns > 0
        ? request.dispatch_start_ns
        : nowSteadyNs();

    SendServoJRequest left_request = request.left;
    left_request.command_seq = request.seq;
    left_request.deadline_ns = request.deadline_ns;
    left_request.host_time_ns = requestHostTime(left_request.host_time_ns, result.dispatch_start_ns);

    SendServoJRequest right_request = request.right;
    right_request.command_seq = request.seq;
    right_request.deadline_ns = request.deadline_ns;
    right_request.host_time_ns = requestHostTime(right_request.host_time_ns, result.dispatch_start_ns);

    // Per-arm queue level trim. This is the path real motion uses (io_model:
    // direct; rbpodo_async_streaming is controller-sim only and config validation
    // rejects it in real mode), so a skip that is not honoured HERE is not
    // honoured at all.
    const uint64_t left_start_ns = nowSteadyNs();
    SendServoJResult left_send = request.skip_left
        ? skippedSend(left_request)
        : left_backend.sendServoJ(left_request);
    const uint64_t left_end_ns = nowSteadyNs();
    const BackendTiming left_timing = makeBackendTiming(left_start_ns, left_end_ns);
    left_send = withDispatchTiming(left_send, left_timing);

    const uint64_t right_start_ns = nowSteadyNs();
    SendServoJResult right_send = request.skip_right
        ? skippedSend(right_request)
        : right_backend.sendServoJ(right_request);
    const uint64_t right_end_ns = nowSteadyNs();
    const BackendTiming right_timing = makeBackendTiming(right_start_ns, right_end_ns);
    right_send = withDispatchTiming(right_send, right_timing);

    result.dispatch_end_ns = nowSteadyNs();
    result.timing = makeBackendTiming(result.dispatch_start_ns, result.dispatch_end_ns);

    result.left.arm_id = ArmId::Left;
    result.left.request = left_request;
    result.left.result = left_send;
    result.left.dispatch_timing = left_timing;

    result.right.arm_id = ArmId::Right;
    result.right.request = right_request;
    result.right.result = right_send;
    result.right.dispatch_timing = right_timing;

    result.left_right_start_skew_us = absDurationUs(left_start_ns, right_start_ns);
    result.left_right_end_skew_us = absDurationUs(left_end_ns, right_end_ns);
    return result;
}

DualSendResult ServoDispatcher::dispatchWorker(
    ArmWorker& left_worker,
    ArmWorker& right_worker,
    const ServoDispatchRequest& request
) {
    DualSendResult result;
    result.dispatch_start_ns = request.dispatch_start_ns > 0
        ? request.dispatch_start_ns
        : nowSteadyNs();

    SendServoJRequest left_request = request.left;
    left_request.command_seq = request.seq;
    left_request.deadline_ns = request.deadline_ns;
    left_request.host_time_ns = requestHostTime(left_request.host_time_ns, result.dispatch_start_ns);

    SendServoJRequest right_request = request.right;
    right_request.command_seq = request.seq;
    right_request.deadline_ns = request.deadline_ns;
    right_request.host_time_ns = requestHostTime(right_request.host_time_ns, result.dispatch_start_ns);

    // Honour the level trim here too. This path is not what real motion uses, but
    // leaving one dispatch path silently ignoring a skip is exactly the bug that
    // shipped once already (see ServoDispatchRequest::skip_left).
    const uint64_t left_enqueue_ns = nowSteadyNs();
    if (!request.skip_left) left_worker.enqueueServoJ(left_request);

    const uint64_t right_enqueue_ns = nowSteadyNs();
    if (!request.skip_right) right_worker.enqueueServoJ(right_request);

    std::optional<ArmSendResult> left_result =
        request.skip_left
            ? std::optional<ArmSendResult>(
                  skippedResult(ArmId::Left, left_request, left_enqueue_ns, nowSteadyNs()))
            : left_worker.waitForSendResult(left_request, left_enqueue_ns, request.deadline_ns);
    std::optional<ArmSendResult> right_result =
        request.skip_right
            ? std::optional<ArmSendResult>(
                  skippedResult(ArmId::Right, right_request, right_enqueue_ns, nowSteadyNs()))
            : right_worker.waitForSendResult(right_request, right_enqueue_ns, request.deadline_ns);

    if (!left_result.has_value() && !request.skip_left) {
        const std::optional<ArmSendResult> candidate = left_worker.lastSendResult();
        if (matchesRequest(candidate, left_request, left_enqueue_ns)) {
            left_result = candidate;
        }
    }
    if (!right_result.has_value() && !request.skip_right) {
        const std::optional<ArmSendResult> candidate = right_worker.lastSendResult();
        if (matchesRequest(candidate, right_request, right_enqueue_ns)) {
            right_result = candidate;
        }
    }

    const uint64_t end_ns = nowSteadyNs();
    result.left = left_result.value_or(timeoutResult(ArmId::Left, left_request, left_enqueue_ns, end_ns));
    result.right = right_result.value_or(timeoutResult(ArmId::Right, right_request, right_enqueue_ns, end_ns));
    result.dispatch_end_ns = end_ns;
    result.timing = makeBackendTiming(result.dispatch_start_ns, result.dispatch_end_ns);
    result.left_right_start_skew_us = absDurationUs(left_enqueue_ns, right_enqueue_ns);
    result.left_right_end_skew_us =
        absDurationUs(result.left.dispatch_timing.end_ns, result.right.dispatch_timing.end_ns);
    return result;
}

DualSendResult ServoDispatcher::dispatchRbpodoAsync(
    ArmWorker& left_worker,
    ArmWorker& right_worker,
    const ServoDispatchRequest& request
) {
    DualSendResult result;
    result.dispatch_start_ns = request.dispatch_start_ns > 0
        ? request.dispatch_start_ns
        : nowSteadyNs();

    SendServoJRequest left_request = request.left;
    left_request.command_seq = request.seq;
    left_request.deadline_ns = request.deadline_ns;
    left_request.host_time_ns = requestHostTime(left_request.host_time_ns, result.dispatch_start_ns);

    SendServoJRequest right_request = request.right;
    right_request.command_seq = request.seq;
    right_request.deadline_ns = request.deadline_ns;
    right_request.host_time_ns = requestHostTime(right_request.host_time_ns, result.dispatch_start_ns);

    const uint64_t left_enqueue_ns = nowSteadyNs();
    result.left = request.skip_left
        ? skippedResult(ArmId::Left, left_request, left_enqueue_ns, nowSteadyNs())
        : left_worker.enqueueAsyncServoJ(left_request);

    const uint64_t right_enqueue_ns = nowSteadyNs();
    result.right = request.skip_right
        ? skippedResult(ArmId::Right, right_request, right_enqueue_ns, nowSteadyNs())
        : right_worker.enqueueAsyncServoJ(right_request);

    result.dispatch_end_ns = nowSteadyNs();
    result.timing = makeBackendTiming(result.dispatch_start_ns, result.dispatch_end_ns);
    result.left_right_start_skew_us = absDurationUs(left_enqueue_ns, right_enqueue_ns);
    result.left_right_end_skew_us =
        absDurationUs(result.left.dispatch_timing.end_ns, result.right.dispatch_timing.end_ns);
    return result;
}

}  // namespace rb_servo
