#include "rb_servo/control/cartesian_servo_controller.hpp"

#include "rb_servo/kinematics/ik_solver.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rb_servo {
namespace {

CartesianOrientationInterpolation plannerOrientationMode(LinearMoveOrientationMode mode) {
    return mode == LinearMoveOrientationMode::Slerp
        ? CartesianOrientationInterpolation::Slerp
        : CartesianOrientationInterpolation::Constant;
}

std::string orientationModeName(CartesianOrientationInterpolation mode) {
    return mode == CartesianOrientationInterpolation::Slerp ? "slerp" : "constant";
}

Vec6 referenceVelocityLocal(
    const CartesianServoPathState& path,
    const Pose6D& reference,
    double s
) {
    Vec6 out;
    if (path.done || path.duration_sec <= 0.0) return out;
    const Vec6 velocity_stand{
        (path.target_tcp_stand.x - path.start_tcp_stand.x) / path.duration_sec,
        (path.target_tcp_stand.y - path.start_tcp_stand.y) / path.duration_sec,
        (path.target_tcp_stand.z - path.start_tcp_stand.z) / path.duration_sec,
        0.0,
        0.0,
        0.0,
    };
    const Vec6 velocity_local = math::twistStandToLocal(velocity_stand, reference);
    out.x = velocity_local.x;
    out.y = velocity_local.y;
    out.z = velocity_local.z;

    if (path.orientation_mode == CartesianOrientationInterpolation::Slerp) {
        const double ds = std::min(1.0 - s, std::max(1e-4, 1e-3));
        if (ds > 0.0) {
            const double dt = ds * path.duration_sec;
            const Pose6D next = LinearCartesianPlanner{}.sample(
                CartesianTrajectoryRequest{
                    path.start_tcp_stand,
                    path.target_tcp_stand,
                    path.orientation_mode,
                },
                s + ds
            );
            const Vec6 delta = math::bodyErrorLocal(reference, next);
            out.rx = delta.rx / dt;
            out.ry = delta.ry / dt;
            out.rz = delta.rz / dt;
        }
    }
    return out;
}

bool finiteVec6(const Vec6& value) {
    return std::isfinite(value.x) &&
           std::isfinite(value.y) &&
           std::isfinite(value.z) &&
           std::isfinite(value.rx) &&
           std::isfinite(value.ry) &&
           std::isfinite(value.rz);
}

std::string integrationModeName(CartesianVelocityTargetIntegrationMode mode) {
    switch (mode) {
        case CartesianVelocityTargetIntegrationMode::MeasuredActual:
            return "measured_actual";
        case CartesianVelocityTargetIntegrationMode::MeasuredActualLookahead:
            return "measured_actual_lookahead";
        case CartesianVelocityTargetIntegrationMode::PreviousCommand:
            return "previous_command";
    }
    return "unknown";
}

bool isTwistMode(ControlMode mode) {
    return mode == ControlMode::TcpTwistStand || mode == ControlMode::TcpTwistLocal;
}

bool sameVelocityIntegrationCategory(ControlMode a, ControlMode b) {
    if (isTwistMode(a) && isTwistMode(b)) return true;
    return a == ControlMode::TcpLinearMove && b == ControlMode::TcpLinearMove;
}

void resetVelocityIntegrator(
    CartesianVelocityIntegratorState* state,
    const JointArray& q_actual_deg,
    ControlMode mode,
    uint64_t seq,
    const std::string& reason
) {
    if (!state) return;
    state->valid = true;
    state->q_command_deg = q_actual_deg;
    state->last_mode = mode;
    state->last_seq = seq;
    state->reset_reason = reason;
    ++state->resets_total;
}

bool commandActualDiverged(
    const JointArray& q_command_deg,
    const JointArray& q_actual_deg,
    const JointArray& max_error_deg,
    double* max_observed
) {
    bool diverged = false;
    double observed = 0.0;
    for (int i = 0; i < kDof; ++i) {
        const double error = std::abs(q_command_deg[i] - q_actual_deg[i]);
        observed = std::max(observed, error);
        if (error > max_error_deg[i]) {
            diverged = true;
        }
    }
    if (max_observed) *max_observed = observed;
    return diverged;
}

void populateIntegratorTelemetry(
    CartesianSolveTelemetry* telemetry,
    const CartesianControlConfig& config,
    const CartesianVelocityIntegratorState* state
) {
    if (!telemetry) return;
    telemetry->cartesian_velocity_integration_mode =
        integrationModeName(config.velocity_target_integration);
    telemetry->velocity_target_lookahead_sec = config.velocity_target_lookahead_sec;
    if (!state) return;
    telemetry->q_integrator_valid = state->valid;
    telemetry->integrator_reset_reason = state->reset_reason;
    telemetry->integrator_resets_total = state->resets_total;
    telemetry->integrator_clamps_total = state->clamps_total;
    telemetry->integrator_divergence_total = state->divergence_total;
    telemetry->max_command_actual_error_deg_observed =
        state->max_command_actual_error_deg_observed;
}

CartesianArmTargetResult integrateVelocityTarget(
    const CartesianArmTargetResult& input,
    const CartesianControlConfig& config,
    const JointArray& q_actual_deg,
    const CartesianVelocityResult& velocity,
    double dt_sec,
    ControlMode mode,
    uint64_t seq,
    CartesianVelocityIntegratorState* state
) {
    CartesianArmTargetResult result = input;
    populateIntegratorTelemetry(&result.telemetry, config, state);

    const double safe_dt_sec = std::max(0.0, dt_sec);
    double integration_dt_sec = safe_dt_sec;
    JointArray base_q = q_actual_deg;

    if (config.velocity_target_integration == CartesianVelocityTargetIntegrationMode::MeasuredActualLookahead) {
        integration_dt_sec = config.velocity_target_lookahead_sec;
    } else if (config.velocity_target_integration == CartesianVelocityTargetIntegrationMode::PreviousCommand) {
        if (!state) {
            base_q = q_actual_deg;
        } else if (!state->valid) {
            resetVelocityIntegrator(state, q_actual_deg, mode, seq, "seed_from_actual");
        } else if (config.reset_velocity_integrator_on_mode_change &&
                   !sameVelocityIntegrationCategory(state->last_mode, mode)) {
            resetVelocityIntegrator(state, q_actual_deg, mode, seq, "mode_change");
        }

        if (state) {
            double observed_error = 0.0;
            if (commandActualDiverged(
                    state->q_command_deg,
                    q_actual_deg,
                    config.max_command_actual_error_deg,
                    &observed_error)) {
                ++state->divergence_total;
                state->max_command_actual_error_deg_observed =
                    std::max(state->max_command_actual_error_deg_observed, observed_error);
                if (config.command_actual_error_policy == CartesianCommandActualErrorPolicy::Fault) {
                    state->valid = false;
                    state->reset_reason = "command_actual_divergence_fault";
                    populateIntegratorTelemetry(&result.telemetry, config, state);
                    result.verdict = SafetyVerdict::TrackingError;
                    result.reason = "cartesian_velocity_integrator_divergence";
                    result.telemetry.status = "failed";
                    result.telemetry.reason = result.reason;
                    return result;
                }
                resetVelocityIntegrator(state, q_actual_deg, mode, seq, "command_actual_divergence_reset");
            } else {
                state->max_command_actual_error_deg_observed =
                    std::max(state->max_command_actual_error_deg_observed, observed_error);
            }
            state->last_mode = mode;
            state->last_seq = seq;
            base_q = state->q_command_deg;
        }
    }

    JointArray q_next = base_q;
    for (int i = 0; i < kDof; ++i) {
        q_next[i] += velocity.qdot_deg_s[i] * integration_dt_sec;
        if (!std::isfinite(q_next[i])) {
            result.verdict = SafetyVerdict::IkFailed;
            result.reason = "non_finite_cartesian_servo_joint_target";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }
    }

    result.q_target_deg = q_next;
    populateIntegratorTelemetry(&result.telemetry, config, state);
    return result;
}

double linearNorm(const Vec6& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

double angularNorm(const Vec6& value) {
    return std::sqrt(value.rx * value.rx + value.ry * value.ry + value.rz * value.rz);
}

void scaleLinear(Vec6* value, double scale) {
    value->x *= scale;
    value->y *= scale;
    value->z *= scale;
}

void scaleAngular(Vec6* value, double scale) {
    value->rx *= scale;
    value->ry *= scale;
    value->rz *= scale;
}

bool limitTwist(
    Vec6* value,
    double max_linear_m_s,
    double max_angular_rad_s,
    CartesianLimitPolicy policy,
    bool* clamped
) {
    const double lin = linearNorm(*value);
    const double ang = angularNorm(*value);
    const bool linear_exceeded = lin > max_linear_m_s + 1e-12;
    const bool angular_exceeded = ang > max_angular_rad_s + 1e-12;
    if (!linear_exceeded && !angular_exceeded) return true;
    if (policy == CartesianLimitPolicy::Reject) return false;
    if (linear_exceeded && lin > 0.0) {
        scaleLinear(value, max_linear_m_s / lin);
    }
    if (angular_exceeded && ang > 0.0) {
        scaleAngular(value, max_angular_rad_s / ang);
    }
    if (clamped) *clamped = true;
    return true;
}

const ArmMountConfig& mountForArm(
    ArmId arm_id,
    const ArmMountConfig& left_mount,
    const ArmMountConfig& right_mount
) {
    return arm_id == ArmId::Left ? left_mount : right_mount;
}

}  // namespace

CartesianServoController::CartesianServoController(
    const ArmMountConfig& left_mount,
    const ArmMountConfig& right_mount,
    const CartesianControlConfig& config,
    std::shared_ptr<IKinematics> kinematics
) : left_mount_(left_mount), right_mount_(right_mount), config_(config), kinematics_(std::move(kinematics)) {}

CartesianArmTargetResult CartesianServoController::computeLinearMoveTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode,
    double dt_sec,
    uint64_t command_seq,
    CartesianServoPathState* path_state,
    CartesianVelocityIntegratorState* velocity_integrator_state
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;
    result.telemetry.attempted = true;
    result.telemetry.fk_duration_us = state.fk_duration_us;
    result.telemetry.warn_ik_duration_us = config_.warn_ik_duration_us;
    result.telemetry.fail_ik_duration_us = config_.fail_ik_duration_us;

    if (run_mode != RunMode::Simulation) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "tcp_linear_move_simulation_only";
        result.telemetry.status = "unavailable";
        result.telemetry.reason = result.reason;
        return result;
    }
    if (!kinematics_) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "kinematics_unavailable";
        result.telemetry.status = "unavailable";
        result.telemetry.reason = result.reason;
        return result;
    }
    const bool continuing_active_path = command_seq == 0 && path_state && path_state->active;
    if (!path_state ||
        (!continuing_active_path && !command.has_tcp_target) ||
        !state.tcp_stand ||
        !state.has_valid_tcp_pose ||
        !state.has_valid_joint_state ||
        (!continuing_active_path && !ik_solver::isFinitePose(command.tcp_target_stand))) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "tcp_pose_unavailable";
        result.telemetry.status = "unavailable";
        result.telemetry.reason = result.reason;
        return result;
    }

    if (!continuing_active_path && !command.has_linear_move_duration && !command.has_linear_move_linear_speed) {
        result.verdict = SafetyVerdict::InvalidCommand;
        result.reason = "tcp_linear_move_timing_required";
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        return result;
    }

    if (!continuing_active_path && (!path_state->active || path_state->seq != command_seq)) {
        *path_state = CartesianServoPathState{};
        path_state->active = true;
        path_state->seq = command_seq;
        path_state->start_tcp_stand = *state.tcp_stand;
        path_state->target_tcp_stand = command.tcp_target_stand;
        path_state->orientation_mode = plannerOrientationMode(
            command.has_linear_move_orientation_mode
                ? command.linear_move_orientation_mode
                : config_.linear_move.default_orientation_mode
        );

        const double orientation_distance = math::orientationDistanceRad(
            path_state->start_tcp_stand,
            path_state->target_tcp_stand
        );
        if (path_state->orientation_mode == CartesianOrientationInterpolation::Constant &&
            orientation_distance > config_.linear_move.constant_orientation_tolerance_rad) {
            *path_state = CartesianServoPathState{};
            result.verdict = SafetyVerdict::InvalidCommand;
            result.reason = "tcp_linear_move_constant_orientation_mismatch";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }

        const double position_distance =
            math::positionDistance(path_state->start_tcp_stand, path_state->target_tcp_stand);
        double duration_sec = 0.0;
        if (command.has_linear_move_duration) {
            duration_sec = command.linear_move_duration_sec;
        } else {
            double linear_speed = command.has_linear_move_linear_speed
                ? command.linear_move_linear_speed_m_s
                : config_.linear_move.default_linear_speed_m_s;
            if (linear_speed > config_.max_linear_move_speed_m_s + 1e-12) {
                if (config_.exceed_limit_policy == CartesianLimitPolicy::Reject) {
                    *path_state = CartesianServoPathState{};
                    result.verdict = SafetyVerdict::InvalidCommand;
                    result.reason = "tcp_linear_move_linear_speed_limit_exceeded";
                    result.telemetry.status = "failed";
                    result.telemetry.reason = result.reason;
                    return result;
                }
                linear_speed = config_.max_linear_move_speed_m_s;
            }
            duration_sec = position_distance / linear_speed;
        }
        if (path_state->orientation_mode == CartesianOrientationInterpolation::Slerp) {
            double angular_speed = command.has_linear_move_angular_speed
                ? command.linear_move_angular_speed_rad_s
                : config_.linear_move.default_angular_speed_rad_s;
            if (angular_speed > config_.max_angular_move_speed_rad_s + 1e-12) {
                if (config_.exceed_limit_policy == CartesianLimitPolicy::Reject) {
                    *path_state = CartesianServoPathState{};
                    result.verdict = SafetyVerdict::InvalidCommand;
                    result.reason = "tcp_linear_move_angular_speed_limit_exceeded";
                    result.telemetry.status = "failed";
                    result.telemetry.reason = result.reason;
                    return result;
                }
                angular_speed = config_.max_angular_move_speed_rad_s;
            }
            duration_sec = std::max(duration_sec, orientation_distance / angular_speed);
        }
        if (!std::isfinite(duration_sec) || duration_sec <= 0.0) {
            *path_state = CartesianServoPathState{};
            result.verdict = SafetyVerdict::InvalidCommand;
            result.reason = "tcp_linear_move_invalid_duration";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }
        const double limited_linear_duration = position_distance / config_.max_linear_move_speed_m_s;
        const double limited_angular_duration =
            path_state->orientation_mode == CartesianOrientationInterpolation::Slerp
                ? orientation_distance / config_.max_angular_move_speed_rad_s
                : 0.0;
        const double speed_limited_duration =
            std::max(duration_sec, std::max(limited_linear_duration, limited_angular_duration));
        if (speed_limited_duration > duration_sec + 1e-12 &&
            config_.exceed_limit_policy == CartesianLimitPolicy::Reject) {
            *path_state = CartesianServoPathState{};
            result.verdict = SafetyVerdict::InvalidCommand;
            result.reason = "tcp_linear_move_speed_limit_exceeded";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }
        path_state->duration_sec = std::max(speed_limited_duration, config_.linear_move.min_duration_sec);
        if (path_state->duration_sec > config_.linear_move.max_duration_sec) {
            *path_state = CartesianServoPathState{};
            result.verdict = SafetyVerdict::InvalidCommand;
            result.reason = "tcp_linear_move_duration_exceeds_max";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }
    }

    if (!path_state->done) {
        path_state->elapsed_sec = std::min(path_state->elapsed_sec + std::max(0.0, dt_sec), path_state->duration_sec);
        path_state->done = path_state->elapsed_sec >= path_state->duration_sec - 1e-12;
    }
    const double path_s = path_state->duration_sec > 0.0
        ? std::clamp(path_state->elapsed_sec / path_state->duration_sec, 0.0, 1.0)
        : 1.0;
    const Pose6D reference = LinearCartesianPlanner{}.sample(
        CartesianTrajectoryRequest{
            path_state->start_tcp_stand,
            path_state->target_tcp_stand,
            path_state->orientation_mode,
        },
        path_s
    );
    const Vec6 error = math::bodyErrorLocal(*state.tcp_stand, reference);
    const Vec6 v_ref = referenceVelocityLocal(*path_state, reference, path_s);
    Vec6 v_cmd{
        v_ref.x + config_.path_kp_pos * error.x,
        v_ref.y + config_.path_kp_pos * error.y,
        v_ref.z + config_.path_kp_pos * error.z,
        v_ref.rx + config_.path_kp_ori * error.rx,
        v_ref.ry + config_.path_kp_ori * error.ry,
        v_ref.rz + config_.path_kp_ori * error.rz,
    };
    if (!finiteVec6(v_cmd)) {
        result.verdict = SafetyVerdict::IkFailed;
        result.reason = "non_finite_cartesian_servo_command";
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        return result;
    }
    double max_linear_velocity = config_.max_linear_move_speed_m_s;
    double max_angular_velocity = config_.max_angular_move_speed_rad_s;
    if (dt_sec > 0.0 && config_.max_cartesian_step_m.has_value()) {
        max_linear_velocity = std::min(max_linear_velocity, *config_.max_cartesian_step_m / dt_sec);
    }
    if (dt_sec > 0.0 && config_.max_cartesian_step_rad.has_value()) {
        max_angular_velocity = std::min(max_angular_velocity, *config_.max_cartesian_step_rad / dt_sec);
    }
    if (!limitTwist(
            &v_cmd,
            max_linear_velocity,
            max_angular_velocity,
            config_.exceed_limit_policy,
            nullptr)) {
        result.verdict = SafetyVerdict::InvalidCommand;
        result.reason = "cartesian_linear_move_limit_exceeded";
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        return result;
    }

    const CartesianVelocityResult velocity = kinematics_->solveCartesianVelocity(
        command.arm_id,
        state.q_actual_deg,
        mountForArm(command.arm_id, left_mount_, right_mount_),
        v_cmd,
        config_.velocity_damping
    );
    if (!velocity.success) {
        result.verdict = SafetyVerdict::IkFailed;
        result.reason = velocity.reason.empty() ? "cartesian_velocity_solve_failed" : velocity.reason;
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        return result;
    }

    result.verdict = SafetyVerdict::Ok;
    result.telemetry.success = true;
    result.telemetry.status = "ok";
    result.telemetry.path_active = path_state->active && !path_state->done;
    result.telemetry.path_s = path_s;
    result.telemetry.path_position_error_m = std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
    result.telemetry.path_orientation_error_rad = std::sqrt(error.rx * error.rx + error.ry * error.ry + error.rz * error.rz);
    result.telemetry.path_line_deviation_m = math::lineDeviation(
        path_state->start_tcp_stand,
        path_state->target_tcp_stand,
        *state.tcp_stand
    );
    result.telemetry.path_done = path_state->done;
    result.telemetry.linear_move_duration_sec = path_state->duration_sec;
    result.telemetry.linear_move_elapsed_sec = path_state->elapsed_sec;
    result.telemetry.orientation_mode = orientationModeName(path_state->orientation_mode);
    result.telemetry.position_error_m = result.telemetry.path_position_error_m;
    result.telemetry.orientation_error_rad = result.telemetry.path_orientation_error_rad;
    return integrateVelocityTarget(
        result,
        config_,
        state.q_actual_deg,
        velocity,
        dt_sec,
        ControlMode::TcpLinearMove,
        command_seq,
        velocity_integrator_state
    );
}

CartesianArmTargetResult CartesianServoController::computeTwistTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode,
    double dt_sec,
    uint64_t command_seq,
    CartesianTwistHoldState* hold_state,
    CartesianVelocityIntegratorState* velocity_integrator_state
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;
    result.telemetry.attempted = true;
    result.telemetry.fk_duration_us = state.fk_duration_us;
    result.telemetry.warn_ik_duration_us = config_.warn_ik_duration_us;
    result.telemetry.fail_ik_duration_us = config_.fail_ik_duration_us;

    if (run_mode != RunMode::Simulation) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "tcp_twist_simulation_only";
        result.telemetry.status = "unavailable";
        result.telemetry.reason = result.reason;
        return result;
    }
    if (!kinematics_) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "kinematics_unavailable";
        result.telemetry.status = "unavailable";
        result.telemetry.reason = result.reason;
        return result;
    }
    if (!state.tcp_stand || !state.has_valid_tcp_pose || !state.has_valid_joint_state || !hold_state) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "tcp_pose_unavailable";
        result.telemetry.status = "unavailable";
        result.telemetry.reason = result.reason;
        return result;
    }

    Vec6 requested;
    if (command.mode == ControlMode::TcpTwistLocal) {
        if (!command.has_tcp_twist_local) {
            result.verdict = SafetyVerdict::CartesianUnavailable;
            result.reason = "missing_tcp_twist_local";
            result.telemetry.status = "unavailable";
            result.telemetry.reason = result.reason;
            return result;
        }
        requested = command.tcp_twist_local;
    } else if (command.mode == ControlMode::TcpTwistStand) {
        if (!command.has_tcp_twist_stand) {
            result.verdict = SafetyVerdict::CartesianUnavailable;
            result.reason = "missing_tcp_twist_stand";
            result.telemetry.status = "unavailable";
            result.telemetry.reason = result.reason;
            return result;
        }
        requested = math::twistStandToLocal(command.tcp_twist_stand, *state.tcp_stand);
    } else {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "not_tcp_twist_mode";
        result.telemetry.status = "not_attempted";
        result.telemetry.reason = result.reason;
        return result;
    }
    if (!finiteVec6(requested)) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "invalid_tcp_twist";
        result.telemetry.status = "unavailable";
        result.telemetry.reason = result.reason;
        return result;
    }
    result.telemetry.requested_twist_linear_norm_m_s = linearNorm(requested);
    result.telemetry.requested_twist_angular_norm_rad_s = angularNorm(requested);
    bool twist_clamped = false;
    if (!limitTwist(
            &requested,
            config_.max_twist_linear_m_s,
            config_.max_twist_angular_rad_s,
            config_.exceed_limit_policy,
            &twist_clamped)) {
        result.verdict = SafetyVerdict::InvalidCommand;
        result.reason = "cartesian_twist_limit_exceeded";
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        result.telemetry.applied_twist_linear_norm_m_s = 0.0;
        result.telemetry.applied_twist_angular_norm_rad_s = 0.0;
        return result;
    }

    Vec6 orientation_error;
    if (angularNorm(requested) <= config_.twist_angular_deadband_rad_s) {
        if (!hold_state->orientation_hold_active) {
            hold_state->hold_tcp_stand = *state.tcp_stand;
            hold_state->orientation_hold_active = true;
        }
        Pose6D orientation_reference = hold_state->hold_tcp_stand;
        orientation_reference.x = state.tcp_stand->x;
        orientation_reference.y = state.tcp_stand->y;
        orientation_reference.z = state.tcp_stand->z;
        orientation_error = math::bodyErrorLocal(*state.tcp_stand, orientation_reference);
        requested.rx = config_.twist_orientation_hold_kp * orientation_error.rx;
        requested.ry = config_.twist_orientation_hold_kp * orientation_error.ry;
        requested.rz = config_.twist_orientation_hold_kp * orientation_error.rz;
    } else {
        hold_state->orientation_hold_active = false;
    }
    if (!limitTwist(
            &requested,
            config_.max_twist_linear_m_s,
            config_.max_twist_angular_rad_s,
            config_.exceed_limit_policy,
            &twist_clamped)) {
        result.verdict = SafetyVerdict::InvalidCommand;
        result.reason = "cartesian_twist_limit_exceeded";
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        result.telemetry.applied_twist_linear_norm_m_s = 0.0;
        result.telemetry.applied_twist_angular_norm_rad_s = 0.0;
        return result;
    }
    result.telemetry.twist_clamped = twist_clamped;
    result.telemetry.applied_twist_linear_norm_m_s = linearNorm(requested);
    result.telemetry.applied_twist_angular_norm_rad_s = angularNorm(requested);

    const CartesianVelocityResult velocity = kinematics_->solveCartesianVelocity(
        command.arm_id,
        state.q_actual_deg,
        mountForArm(command.arm_id, left_mount_, right_mount_),
        requested,
        config_.velocity_damping
    );
    if (!velocity.success) {
        result.verdict = SafetyVerdict::IkFailed;
        result.reason = velocity.reason.empty() ? "cartesian_velocity_solve_failed" : velocity.reason;
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        return result;
    }

    result.verdict = SafetyVerdict::Ok;
    result.telemetry.success = true;
    result.telemetry.status = "ok";
    result.telemetry.path_orientation_error_rad =
        std::sqrt(orientation_error.rx * orientation_error.rx +
                  orientation_error.ry * orientation_error.ry +
                  orientation_error.rz * orientation_error.rz);
    result.telemetry.orientation_error_rad = result.telemetry.path_orientation_error_rad;
    return integrateVelocityTarget(
        result,
        config_,
        state.q_actual_deg,
        velocity,
        dt_sec,
        command.mode,
        command_seq,
        velocity_integrator_state
    );
}

void CartesianServoController::updateVelocityIntegratorAfterSafety(
    CartesianVelocityIntegratorState* velocity_integrator_state,
    const JointArray& safe_q_target_deg,
    bool was_sent_or_intended,
    bool target_was_clamped,
    const std::string& reset_reason
) {
    if (!velocity_integrator_state ||
        config_.velocity_target_integration != CartesianVelocityTargetIntegrationMode::PreviousCommand) {
        return;
    }
    if (!was_sent_or_intended) {
        if (velocity_integrator_state->valid) {
            velocity_integrator_state->valid = false;
            velocity_integrator_state->reset_reason = reset_reason;
            ++velocity_integrator_state->resets_total;
        }
        return;
    }
    if (!velocity_integrator_state->valid) {
        return;
    }
    velocity_integrator_state->q_command_deg = safe_q_target_deg;
    if (target_was_clamped) {
        ++velocity_integrator_state->clamps_total;
    }
}

}  // namespace rb_servo
