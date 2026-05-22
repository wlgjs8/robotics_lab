#include "rb_servo/robot/rbpodo_backend.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "rb_servo/core/clock.hpp"

#ifdef RB_SERVO_ENABLE_RBPODO
// TODO: include actual rbpodo headers here.
// #include <rbpodo/rbpodo.hpp>
#endif

namespace rb_servo {

struct RbpodoBackend::Impl {
    ArmId arm_id;
    BackendConfig config;
    bool connected = false;

#ifdef RB_SERVO_ENABLE_RBPODO
    // TODO: store rbpodo Cobot object here.
#endif
};

RbpodoBackend::RbpodoBackend(ArmId arm_id, const BackendConfig& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->arm_id = arm_id;
    impl_->config = config;
}

RbpodoBackend::~RbpodoBackend() = default;

bool RbpodoBackend::connect() {
#ifndef RB_SERVO_ENABLE_RBPODO
    std::cerr << "[ERROR] RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF.\n";
    return false;
#else
    if (impl_->config.run_mode == RunMode::Real) {
        const char* allow = std::getenv("RB_ALLOW_REAL_ROBOT");
        if (!allow || std::string(allow) != "1") {
            throw std::runtime_error("Refusing real robot mode. Set RB_ALLOW_REAL_ROBOT=1.");
        }
    }
    std::cerr << "[ERROR] RbpodoBackend state/servo implementation is incomplete; refusing to connect.\n";
    return false;
#endif
}

bool RbpodoBackend::initialize() {
#ifndef RB_SERVO_ENABLE_RBPODO
    return false;
#else
    // TODO:
    // - set operation mode real/simulation
    // - speed bar
    // - disable waiting ACK if supported by current rbpodo version
    return impl_->connected;
#endif
}

bool RbpodoBackend::readState(RobotState& out_state) {
#ifndef RB_SERVO_ENABLE_RBPODO
    (void)out_state;
    return false;
#else
    out_state.arm_id = impl_->arm_id;
    out_state.host_time_ns = nowSteadyNs();
    out_state.connection_state = impl_->connected ? RobotConnectionState::Connected : RobotConnectionState::Disconnected;
    out_state.has_valid_joint_state = false;
    // TODO: read q_actual, q_target, error state through rbpodo data channel.
    return false;
#endif
}

bool RbpodoBackend::sendServoJ(const JointArray& q_target_deg) {
#ifndef RB_SERVO_ENABLE_RBPODO
    (void)q_target_deg;
    return false;
#else
    // TODO: call rbpodo move_servo_j with config parameters.
    (void)q_target_deg;
    return false;
#endif
}

bool RbpodoBackend::stop() {
#ifndef RB_SERVO_ENABLE_RBPODO
    return false;
#else
    // TODO: call stop/hold method.
    return false;
#endif
}

bool RbpodoBackend::resetFault() {
#ifndef RB_SERVO_ENABLE_RBPODO
    return false;
#else
    // TODO: call reset fault/recover if available.
    return false;
#endif
}

bool RbpodoBackend::isConnected() const {
    return impl_->connected;
}

ArmId RbpodoBackend::armId() const {
    return impl_->arm_id;
}

std::string RbpodoBackend::name() const {
    return impl_->config.name;
}

}  // namespace rb_servo
