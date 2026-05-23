#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include "rb_servo/control/servo_dispatcher.hpp"

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

private:
    rb_servo::ArmId arm_id_;
    rb_servo::BackendErrorKind send_error_kind_;
    std::chrono::microseconds send_delay_;
    rb_servo::RobotState state_;
    rb_servo::SendServoJRequest last_request_;
};

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
    RB_CHECK(result.left.request.host_time_ns == left.lastRequest().host_time_ns);
    RB_CHECK(result.right.request.host_time_ns == right.lastRequest().host_time_ns);
    RB_CHECK(sameJointArray(result.left.request.q_target_deg, joints(1.0)));
    RB_CHECK(sameJointArray(result.right.request.q_target_deg, joints(2.0)));
    RB_CHECK(sameJointArray(result.left.result.requested_q_deg, joints(1.0)));
    RB_CHECK(sameJointArray(result.right.result.requested_q_deg, joints(2.0)));
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

}  // namespace

int main() {
    if (!testDirectSequentialPopulatesTimingAndPreservesResults()) return 1;
    if (!testRobotFaultHelper()) return 1;
    return 0;
}
