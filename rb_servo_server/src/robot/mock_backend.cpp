#include "rb_servo/robot/mock_backend.hpp"

#include <algorithm>
#include "rb_servo/core/clock.hpp"

namespace rb_servo {

MockBackend::MockBackend(ArmId arm_id, const BackendConfig& config)
    : arm_id_(arm_id), config_(config) {}

bool MockBackend::connect() {
    connected_ = true;
    return true;
}

bool MockBackend::initialize() {
    q_actual_deg_ = config_.initial_q_deg;
    q_target_deg_ = config_.initial_q_deg;
    initialized_ = true;
    last_update_time_ns_ = nowSteadyNs();
    return true;
}

bool MockBackend::readState(RobotState& out_state) {
    const uint64_t now = nowSteadyNs();
    const double dt = last_update_time_ns_ == 0 ? 0.005 : nsToSec(now - last_update_time_ns_);
    last_update_time_ns_ = now;

    const double tau = 0.04;
    const double alpha = std::clamp(dt / tau, 0.0, 1.0);
    for (int i = 0; i < kDof; ++i) {
        q_actual_deg_[i] += alpha * (q_target_deg_[i] - q_actual_deg_[i]);
    }

    out_state.arm_id = arm_id_;
    out_state.host_time_ns = now;
    out_state.q_actual_deg = q_actual_deg_;
    out_state.q_target_deg = q_target_deg_;
    out_state.has_valid_joint_state = true;
    out_state.connection_state = connected_ ? RobotConnectionState::Connected : RobotConnectionState::Disconnected;
    out_state.servo_enabled = initialized_;
    out_state.has_error = false;
    out_state.error_code = 0;
    return true;
}

bool MockBackend::sendServoJ(const JointArray& q_target_deg) {
    q_target_deg_ = q_target_deg;
    return connected_ && initialized_;
}

bool MockBackend::stop() {
    q_target_deg_ = q_actual_deg_;
    return true;
}

bool MockBackend::resetFault() {
    return true;
}

bool MockBackend::isConnected() const {
    return connected_;
}

ArmId MockBackend::armId() const {
    return arm_id_;
}

std::string MockBackend::name() const {
    return config_.name;
}

}  // namespace rb_servo
