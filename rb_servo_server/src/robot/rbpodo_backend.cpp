#include "rb_servo/robot/rbpodo_backend.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rb_servo/core/clock.hpp"

#ifdef RB_SERVO_ENABLE_RBPODO
#include <rbpodo/rbpodo.hpp>
#endif

namespace rb_servo {
namespace {

bool envIsOne(const char* name) {
    const char* value = std::getenv(name);
    return value && std::string(value) == "1";
}

bool finiteJointArray(const JointArray& joints) {
    return std::all_of(joints.begin(), joints.end(), [](double value) {
        return std::isfinite(value);
    });
}

template <typename T>
BackendResult<T> okResult(BackendOp op, const T& value, const BackendTiming& timing = BackendTiming{}) {
    BackendResult<T> result;
    result.ok = true;
    result.op = op;
    result.value = value;
    result.error = noBackendError();
    result.timing = timing;
    return result;
}

template <typename T>
BackendResult<T> failedResult(BackendOp op, const BackendError& error, const BackendTiming& timing = BackendTiming{}) {
    BackendResult<T> result;
    result.ok = false;
    result.op = op;
    result.error = error;
    result.timing = timing;
    return result;
}

RobotState basicState(ArmId arm_id, bool connected) {
    RobotState state;
    state.arm_id = arm_id;
    state.host_time_ns = nowSteadyNs();
    state.connection_state = connected
        ? RobotConnectionState::Connected
        : RobotConnectionState::Disconnected;
    state.has_valid_joint_state = false;
    return state;
}

#ifdef RB_SERVO_ENABLE_RBPODO
constexpr double kDefaultCommandTimeoutSec = 0.5;
constexpr double kDefaultStateTimeoutSec = 0.2;

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

rb::podo::OperationMode operationModeFromConfig(const BackendConfig& config) {
    const std::string operation_mode = lower(config.operation_mode);
    if (operation_mode == "simulation" || operation_mode == "sim") {
        return rb::podo::OperationMode::Simulation;
    }
    return rb::podo::OperationMode::Real;
}

int firstNonZeroErrorCode(const rb::podo::SystemState& state) {
    if (state.sdata.op_stat_sos_flag != 0) return state.sdata.op_stat_sos_flag;
    if (state.sdata.init_error != 0) return state.sdata.init_error;
    if (state.sdata.op_stat_ems_flag != 0) return state.sdata.op_stat_ems_flag;
    if (state.sdata.op_stat_soft_estop_occur != 0) return state.sdata.op_stat_soft_estop_occur;
    if (state.sdata.op_stat_collision_occur != 0) return state.sdata.op_stat_collision_occur;
    if (state.sdata.op_stat_self_collision != 0) return state.sdata.op_stat_self_collision;
    return 0;
}

void fillRobotStateFromSystemState(
    ArmId arm_id,
    const rb::podo::SystemState& rb_state,
    RobotState* out_state
) {
    out_state->arm_id = arm_id;
    out_state->host_time_ns = nowSteadyNs();
    out_state->robot_time_ns = std::isfinite(static_cast<double>(rb_state.sdata.time)) && rb_state.sdata.time >= 0.0f
        ? static_cast<uint64_t>(static_cast<double>(rb_state.sdata.time) * 1'000'000'000.0)
        : 0;

    for (int i = 0; i < kDof; ++i) {
        out_state->q_actual_deg[static_cast<std::size_t>(i)] = rb_state.sdata.jnt_ang[i];
        out_state->q_target_deg[static_cast<std::size_t>(i)] = rb_state.sdata.jnt_ref[i];
        out_state->dq_actual_deg_s[static_cast<std::size_t>(i)] = 0.0;
    }

    const int error_code = firstNonZeroErrorCode(rb_state);
    out_state->connection_state = RobotConnectionState::Connected;
    out_state->servo_enabled = rb_state.sdata.init_state_info == 6;
    out_state->has_error = error_code != 0;
    out_state->error_code = error_code;
    out_state->has_valid_joint_state =
        finiteJointArray(out_state->q_actual_deg) &&
        finiteJointArray(out_state->q_target_deg);
}
#endif

}  // namespace

struct RbpodoBackend::Impl {
    ArmId arm_id;
    BackendConfig config;
    bool connected = false;

#ifdef RB_SERVO_ENABLE_RBPODO
    std::unique_ptr<rb::podo::Cobot<>> robot;
    std::unique_ptr<rb::podo::CobotData> data_channel;
#endif
};

RbpodoBackend::RbpodoBackend(ArmId arm_id, const BackendConfig& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->arm_id = arm_id;
    impl_->config = config;
}

RbpodoBackend::~RbpodoBackend() = default;

BackendResult<RobotState> RbpodoBackend::connect() {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    std::cerr << "[ERROR] RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF.\n";
    return failedResult<RobotState>(
        BackendOp::Connect,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    if (impl_->config.run_mode == RunMode::Real) {
        if (!envIsOne("RB_ALLOW_REAL_ROBOT")) {
            impl_->connected = false;
            return failedResult<RobotState>(
                BackendOp::Connect,
                backendError(BackendErrorKind::SuppressedByPolicy, "Refusing real robot mode. Set RB_ALLOW_REAL_ROBOT=1."),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
    }
    if (impl_->config.ip.empty()) {
        std::cerr << "[ERROR] RbpodoBackend requires a non-empty controller ip for "
                  << impl_->config.name << "\n";
        return failedResult<RobotState>(
            BackendOp::Connect,
            backendError(BackendErrorKind::WrongEndpoint, "RbpodoBackend requires a non-empty controller ip"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    try {
        impl_->robot = std::make_unique<rb::podo::Cobot<>>(impl_->config.ip);
        impl_->data_channel = std::make_unique<rb::podo::CobotData>(impl_->config.ip);
        const auto state = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
        if (!state) {
            std::cerr << "[ERROR] RbpodoBackend connected sockets but did not receive state from "
                      << impl_->config.ip << "\n";
            impl_->robot.reset();
            impl_->data_channel.reset();
            impl_->connected = false;
            return failedResult<RobotState>(
                BackendOp::Connect,
                backendError(BackendErrorKind::TransportReadFailed, "rbpodo connected sockets but did not receive state"),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        impl_->connected = true;
        std::cerr << "[INFO] RbpodoBackend connected to " << impl_->config.ip
                  << " for " << impl_->config.name << "\n";
        RobotState mapped;
        fillRobotStateFromSystemState(impl_->arm_id, *state, &mapped);
        return okResult(BackendOp::Connect, mapped, makeBackendTiming(start, nowSteadyNs()));
    } catch (const std::exception& exc) {
        std::cerr << "[ERROR] RbpodoBackend connect failed for " << impl_->config.ip
                  << ": " << exc.what() << "\n";
        impl_->robot.reset();
        impl_->data_channel.reset();
        impl_->connected = false;
        return failedResult<RobotState>(
            BackendOp::Connect,
            backendError(BackendErrorKind::TransportConnectFailed, exc.what()),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
#endif
}

BackendResult<RobotState> RbpodoBackend::initialize() {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return failedResult<RobotState>(
        BackendOp::Initialize,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    if (!impl_->connected || !impl_->data_channel) {
        return failedResult<RobotState>(
            BackendOp::Initialize,
            backendError(BackendErrorKind::RobotDisconnected, "rbpodo backend is not connected"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    try {
        const auto state = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
        if (!state) {
            std::cerr << "[ERROR] RbpodoBackend initialize failed: no state from "
                      << impl_->config.ip << "\n";
            return failedResult<RobotState>(
                BackendOp::Initialize,
                backendError(BackendErrorKind::TransportReadFailed, "rbpodo initialize failed: no state"),
                makeBackendTiming(start, nowSteadyNs())
            );
        }

        if (impl_->config.run_mode == RunMode::Real && envIsOne("RB_ALLOW_REAL_MOTION")) {
            rb::podo::ResponseCollector responses;
            auto ret = impl_->robot->set_operation_mode(
                responses,
                operationModeFromConfig(impl_->config),
                kDefaultCommandTimeoutSec,
                true
            );
            if (!ret.is_success()) {
                std::cerr << "[ERROR] RbpodoBackend set_operation_mode failed for "
                          << impl_->config.name << "\n";
                return failedResult<RobotState>(
                    BackendOp::Initialize,
                    backendError(BackendErrorKind::ControllerRejected, "rbpodo set_operation_mode failed"),
                    makeBackendTiming(start, nowSteadyNs())
                );
            }
            responses.clear();
            ret = impl_->robot->set_speed_bar(
                responses,
                impl_->config.speed_bar,
                kDefaultCommandTimeoutSec,
                true
            );
            if (!ret.is_success()) {
                std::cerr << "[ERROR] RbpodoBackend set_speed_bar failed for "
                          << impl_->config.name << "\n";
                return failedResult<RobotState>(
                    BackendOp::Initialize,
                    backendError(BackendErrorKind::ControllerRejected, "rbpodo set_speed_bar failed"),
                    makeBackendTiming(start, nowSteadyNs())
                );
            }
        }
        RobotState mapped;
        fillRobotStateFromSystemState(impl_->arm_id, *state, &mapped);
        return okResult(BackendOp::Initialize, mapped, makeBackendTiming(start, nowSteadyNs()));
    } catch (const std::exception& exc) {
        std::cerr << "[ERROR] RbpodoBackend initialize failed for " << impl_->config.name
                  << ": " << exc.what() << "\n";
        return failedResult<RobotState>(
            BackendOp::Initialize,
            backendError(BackendErrorKind::Unknown, exc.what()),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
#endif
}

BackendResult<RobotState> RbpodoBackend::readState() {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return failedResult<RobotState>(
        BackendOp::ReadState,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    RobotState out_state = basicState(impl_->arm_id, impl_->connected);
    out_state.arm_id = impl_->arm_id;
    out_state.host_time_ns = nowSteadyNs();
    out_state.connection_state = impl_->connected
        ? RobotConnectionState::Connected
        : RobotConnectionState::Disconnected;
    out_state.has_valid_joint_state = false;
    if (!impl_->connected || !impl_->data_channel) {
        return failedResult<RobotState>(
            BackendOp::ReadState,
            backendError(BackendErrorKind::RobotDisconnected, "rbpodo backend is not connected"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    try {
        const auto state = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
        if (!state) {
            out_state.connection_state = RobotConnectionState::Disconnected;
            impl_->connected = false;
            return failedResult<RobotState>(
                BackendOp::ReadState,
                backendError(BackendErrorKind::TransportReadFailed, "rbpodo readState returned no state"),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        fillRobotStateFromSystemState(impl_->arm_id, *state, &out_state);
        return okResult(BackendOp::ReadState, out_state, makeBackendTiming(start, nowSteadyNs()));
    } catch (const std::exception& exc) {
        std::cerr << "[WARN] RbpodoBackend readState failed for " << impl_->config.name
                  << ": " << exc.what() << "\n";
        out_state.connection_state = RobotConnectionState::Error;
        out_state.has_error = true;
        out_state.error_code = -1;
        return failedResult<RobotState>(
            BackendOp::ReadState,
            backendError(BackendErrorKind::Unknown, exc.what()),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
#endif
}

SendServoJResult RbpodoBackend::sendServoJ(const SendServoJRequest& request) {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return rejectedSend(
        request,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    if (!impl_->connected || !impl_->robot) {
        return rejectedSend(
            request,
            backendError(BackendErrorKind::RobotDisconnected, "rbpodo backend is not connected"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    if (impl_->config.run_mode == RunMode::Real && !envIsOne("RB_ALLOW_REAL_MOTION")) {
        std::cerr << "[ERROR] RbpodoBackend refused servo_j without RB_ALLOW_REAL_MOTION=1\n";
        return rejectedSend(
            request,
            backendError(BackendErrorKind::SuppressedByPolicy, "RbpodoBackend refused servo_j without RB_ALLOW_REAL_MOTION=1"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    if (!finiteJointArray(request.q_target_deg)) {
        std::cerr << "[ERROR] RbpodoBackend refused non-finite servo_j target\n";
        return rejectedSend(
            request,
            backendError(BackendErrorKind::InvalidTarget, "RbpodoBackend refused non-finite servo_j target"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    try {
        rb::podo::ResponseCollector responses;
        const auto ret = impl_->robot->move_servo_j(
            responses,
            request.q_target_deg,
            impl_->config.servo_time_sec,
            impl_->config.servo_lookahead_sec,
            impl_->config.servo_gain,
            impl_->config.servo_acc,
            kDefaultCommandTimeoutSec,
            true
        );
        if (!ret.is_success()) {
            std::cerr << "[WARN] RbpodoBackend move_servo_j was not accepted for "
                      << impl_->config.name << "\n";
            return rejectedSend(
                request,
                backendError(BackendErrorKind::ControllerRejected, "rbpodo move_servo_j was not accepted"),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        if (responses.has_error()) {
            std::cerr << "[WARN] RbpodoBackend move_servo_j response contained an error for "
                      << impl_->config.name << ": " << responses << "\n";
            return rejectedSend(
                request,
                backendError(BackendErrorKind::ControllerRejected, "rbpodo move_servo_j response contained an error"),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        return acceptedSend(request, makeBackendTiming(start, nowSteadyNs()));
    } catch (const std::exception& exc) {
        std::cerr << "[WARN] RbpodoBackend move_servo_j failed for " << impl_->config.name
                  << ": " << exc.what() << "\n";
        return rejectedSend(
            request,
            backendError(BackendErrorKind::Unknown, exc.what()),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
#endif
}

BackendResult<RobotState> RbpodoBackend::stop() {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return failedResult<RobotState>(
        BackendOp::Stop,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    // No verified controller-level hold/stop API is wired for P1-B.
    // task_stop exists in rbpodo, but it stops task programs and is not treated
    // here as a safe servo hold primitive.
    return failedResult<RobotState>(
        BackendOp::Stop,
        backendError(BackendErrorKind::DependencyUnavailable, "No verified rbpodo hold/stop API is wired"),
        makeBackendTiming(start, nowSteadyNs())
    );
#endif
}

BackendResult<RobotState> RbpodoBackend::resetFault() {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return failedResult<RobotState>(
        BackendOp::ResetFault,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    // No verified fault-reset API is exposed by the inspected rbpodo headers.
    return failedResult<RobotState>(
        BackendOp::ResetFault,
        backendError(BackendErrorKind::DependencyUnavailable, "No verified rbpodo fault-reset API is exposed"),
        makeBackendTiming(start, nowSteadyNs())
    );
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
