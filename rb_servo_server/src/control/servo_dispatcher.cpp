#include "rb_servo/control/servo_dispatcher.hpp"

#include <chrono>
#include <optional>
#include <thread>

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
    const SendServoJRequest& request
) {
    return result.has_value() &&
           result->request.command_seq == request.command_seq &&
           result->request.deadline_ns == request.deadline_ns;
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

}  // namespace

bool DualSendResult::any_transport_failure() const {
    return hasTransportFailure(left) || hasTransportFailure(right);
}

bool DualSendResult::any_robot_fault() const {
    return hasRobotFault(left) || hasRobotFault(right);
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

    SendServoJRequest right_request = request.right;
    right_request.command_seq = request.seq;
    right_request.deadline_ns = request.deadline_ns;

    const uint64_t left_start_ns = nowSteadyNs();
    left_request.host_time_ns = left_start_ns;
    const SendServoJResult left_send = left_backend.sendServoJ(left_request);
    const uint64_t left_end_ns = nowSteadyNs();

    const uint64_t right_start_ns = nowSteadyNs();
    right_request.host_time_ns = right_start_ns;
    const SendServoJResult right_send = right_backend.sendServoJ(right_request);
    const uint64_t right_end_ns = nowSteadyNs();

    result.dispatch_end_ns = nowSteadyNs();
    result.timing = makeBackendTiming(result.dispatch_start_ns, result.dispatch_end_ns);

    result.left.arm_id = ArmId::Left;
    result.left.request = left_request;
    result.left.result = left_send;
    result.left.dispatch_timing = makeBackendTiming(left_start_ns, left_end_ns);

    result.right.arm_id = ArmId::Right;
    result.right.request = right_request;
    result.right.result = right_send;
    result.right.dispatch_timing = makeBackendTiming(right_start_ns, right_end_ns);

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

    SendServoJRequest right_request = request.right;
    right_request.command_seq = request.seq;
    right_request.deadline_ns = request.deadline_ns;

    const uint64_t left_enqueue_ns = nowSteadyNs();
    left_request.host_time_ns = left_enqueue_ns;
    left_worker.enqueueServoJ(left_request);

    const uint64_t right_enqueue_ns = nowSteadyNs();
    right_request.host_time_ns = right_enqueue_ns;
    right_worker.enqueueServoJ(right_request);

    std::optional<ArmSendResult> left_result;
    std::optional<ArmSendResult> right_result;
    while (true) {
        if (!left_result.has_value()) {
            const std::optional<ArmSendResult> candidate = left_worker.lastSendResult();
            if (matchesRequest(candidate, left_request)) {
                left_result = candidate;
            }
        }
        if (!right_result.has_value()) {
            const std::optional<ArmSendResult> candidate = right_worker.lastSendResult();
            if (matchesRequest(candidate, right_request)) {
                right_result = candidate;
            }
        }
        if (left_result.has_value() && right_result.has_value()) {
            break;
        }

        const uint64_t now = nowSteadyNs();
        if (request.deadline_ns > 0 && now >= request.deadline_ns) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
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

}  // namespace rb_servo
