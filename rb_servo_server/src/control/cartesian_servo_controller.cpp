#include "rb_servo/control/cartesian_servo_controller.hpp"

#include "rb_servo/kinematics/ik_solver.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <utility>

namespace rb_servo {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

CartesianOrientationInterpolation plannerOrientationMode(LinearMoveOrientationMode mode) {
    return mode == LinearMoveOrientationMode::Slerp
        ? CartesianOrientationInterpolation::Slerp
        : CartesianOrientationInterpolation::Constant;
}

std::string orientationModeName(CartesianOrientationInterpolation mode) {
    return mode == CartesianOrientationInterpolation::Slerp ? "slerp" : "constant";
}

// Quintic (10t^3 - 15t^4 + 6t^5) time scaling for TcpLinearMove: zero velocity
// AND zero acceleration at both endpoints, so the path eases in/out instead of
// the velocity step of a constant-rate profile. Peak ds/dt is 15/8 of the mean
// rate; the duration speed limits account for that ratio.
constexpr double kQuinticPeakRateRatio = 15.0 / 8.0;
// Completion tolerances for the smoothed (pose_track_smd) reference: the move
// reports done only after the filter output has settled on the latched goal.
constexpr double kLinearMoveSettlePositionM = 5e-4;
constexpr double kLinearMoveSettleOrientationRad = 5e-3;

double quinticTimeScaling(double tau) {
    tau = std::clamp(tau, 0.0, 1.0);
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
}

bool finiteJoints(const JointArray& joints) {
    for (double joint : joints) {
        if (!std::isfinite(joint)) return false;
    }
    return true;
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
    if (a == ControlMode::TcpCircleMove && b == ControlMode::TcpCircleMove) return true;
    return a == ControlMode::TcpLinearMove && b == ControlMode::TcpLinearMove;
}

void resetVelocityIntegrator(
    CartesianVelocityIntegratorState* state,
    const JointArray& q_servo_state_deg,
    ControlMode mode,
    uint64_t seq,
    const std::string& reason
) {
    if (!state) return;
    state->valid = true;
    state->q_command_deg = q_servo_state_deg;
    state->last_mode = mode;
    state->last_seq = seq;
    state->reset_reason = reason;
    ++state->resets_total;
}

double maxJointErrorDeg(
    const JointArray& q_command_deg,
    const JointArray& q_state_deg
) {
    double observed = 0.0;
    for (int i = 0; i < kDof; ++i) {
        observed = std::max(observed, std::abs(q_command_deg[i] - q_state_deg[i]));
    }
    return observed;
}

bool commandStateDiverged(
    const JointArray& q_command_deg,
    const JointArray& q_state_deg,
    const JointArray& max_error_deg,
    double* max_observed
) {
    bool diverged = false;
    double observed = maxJointErrorDeg(q_command_deg, q_state_deg);
    for (int i = 0; i < kDof; ++i) {
        const double error = std::abs(q_command_deg[i] - q_state_deg[i]);
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
    telemetry->cartesian_servo_state_source = state->cartesian_servo_state_source;
    telemetry->cartesian_divergence_source = state->cartesian_divergence_source;
    telemetry->q_reference_for_servo_valid = state->q_reference_for_servo_valid;
    telemetry->q_integrator_valid = state->valid;
    telemetry->integrator_reset_reason = state->reset_reason;
    telemetry->integrator_resets_total = state->resets_total;
    telemetry->integrator_clamps_total = state->clamps_total;
    telemetry->integrator_divergence_total = state->divergence_total;
    telemetry->max_command_actual_error_deg_observed =
        state->max_command_actual_error_deg_observed;
    telemetry->command_reference_error_deg_observed =
        state->command_reference_error_deg_observed;
    telemetry->physical_command_actual_error_deg_observed =
        state->physical_command_actual_error_deg_observed;
}

CartesianArmTargetResult integrateVelocityTarget(
    const CartesianArmTargetResult& input,
    const CartesianControlConfig& config,
    const JointArray& q_servo_state_deg,
    const CartesianVelocityResult& velocity,
    double dt_sec,
    ControlMode mode,
    uint64_t seq,
    CartesianVelocityIntegratorState* state,
    const CartesianServoStateContext* context
) {
    CartesianArmTargetResult result = input;

    const JointArray& q_physical_actual_deg =
        context ? context->physical_q_actual_deg : q_servo_state_deg;
    const JointArray& q_reference_deg =
        context ? context->reference_q_deg : q_servo_state_deg;
    const JointArray& q_divergence_deg =
        context ? context->divergence_q_deg : q_servo_state_deg;
    if (state && context) {
        state->cartesian_servo_state_source = context->servo_state_source;
        state->cartesian_divergence_source = context->divergence_source;
        state->q_reference_for_servo_valid = context->q_reference_for_servo_valid;
    }
    populateIntegratorTelemetry(&result.telemetry, config, state);

    const double safe_dt_sec = std::max(0.0, dt_sec);
    double integration_dt_sec = safe_dt_sec;
    JointArray base_q = q_servo_state_deg;

    if (config.velocity_target_integration == CartesianVelocityTargetIntegrationMode::MeasuredActualLookahead) {
        integration_dt_sec = config.velocity_target_lookahead_sec;
    } else if (config.velocity_target_integration == CartesianVelocityTargetIntegrationMode::PreviousCommand) {
        if (!state) {
            base_q = q_servo_state_deg;
        } else if (!state->valid) {
            resetVelocityIntegrator(state, q_servo_state_deg, mode, seq, "seed_from_servo_state");
        } else if (config.reset_velocity_integrator_on_mode_change &&
                   !sameVelocityIntegrationCategory(state->last_mode, mode)) {
            resetVelocityIntegrator(state, q_servo_state_deg, mode, seq, "mode_change");
        }

        if (state) {
            const double physical_error =
                maxJointErrorDeg(state->q_command_deg, q_physical_actual_deg);
            state->physical_command_actual_error_deg_observed =
                std::max(state->physical_command_actual_error_deg_observed, physical_error);
            state->max_command_actual_error_deg_observed =
                std::max(state->max_command_actual_error_deg_observed, physical_error);
            if (context && context->q_reference_for_servo_valid) {
                const double reference_error =
                    maxJointErrorDeg(state->q_command_deg, q_reference_deg);
                state->command_reference_error_deg_observed =
                    std::max(state->command_reference_error_deg_observed, reference_error);
            }

            double observed_divergence_error = 0.0;
            if (commandStateDiverged(
                    state->q_command_deg,
                    q_divergence_deg,
                    config.max_command_actual_error_deg,
                    &observed_divergence_error)) {
                ++state->divergence_total;
                if (config.command_actual_error_policy == CartesianCommandActualErrorPolicy::Fault) {
                    state->valid = false;
                    state->reset_reason = context && context->divergence_source == "reference"
                        ? "command_reference_divergence_fault"
                        : "command_actual_divergence_fault";
                    populateIntegratorTelemetry(&result.telemetry, config, state);
                    result.verdict = SafetyVerdict::TrackingError;
                    result.reason = "cartesian_velocity_integrator_divergence";
                    result.telemetry.status = "failed";
                    result.telemetry.reason = result.reason;
                    return result;
                }
                resetVelocityIntegrator(
                    state,
                    q_servo_state_deg,
                    mode,
                    seq,
                    context && context->divergence_source == "reference"
                        ? "command_reference_divergence_reset"
                        : "command_actual_divergence_reset"
                );
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

bool circleAxes(TcpCirclePlane plane, int* axis1, int* axis2) {
    if (!axis1 || !axis2) return false;
    switch (plane) {
        case TcpCirclePlane::XY:
            *axis1 = 0;
            *axis2 = 1;
            return true;
        case TcpCirclePlane::XZ:
            *axis1 = 0;
            *axis2 = 2;
            return true;
        case TcpCirclePlane::YZ:
            *axis1 = 1;
            *axis2 = 2;
            return true;
    }
    return false;
}

double poseAxisValue(const Pose6D& pose, int axis) {
    switch (axis) {
        case 0:
            return pose.x;
        case 1:
            return pose.y;
        default:
            return pose.z;
    }
}

void setPoseAxisValue(Pose6D* pose, int axis, double value) {
    if (!pose) return;
    switch (axis) {
        case 0:
            pose->x = value;
            break;
        case 1:
            pose->y = value;
            break;
        default:
            pose->z = value;
            break;
    }
}

void setVecAxisValue(Vec6* vec, int axis, double value) {
    if (!vec) return;
    switch (axis) {
        case 0:
            vec->x = value;
            break;
        case 1:
            vec->y = value;
            break;
        default:
            vec->z = value;
            break;
    }
}

}  // namespace

CartesianServoController::CartesianServoController(
    const ArmMountConfig& left_mount,
    const ArmMountConfig& right_mount,
    const CartesianControlConfig& config,
    std::shared_ptr<IKinematics> kinematics
) : left_mount_(left_mount), right_mount_(right_mount), config_(config), kinematics_(std::move(kinematics)) {}

void CartesianServoController::setFloorConstraint(bool enabled, double z_min_m, double soft_margin_m) {
    floor_enabled_ = enabled && std::isfinite(z_min_m) && std::isfinite(soft_margin_m);
    floor_z_min_m_ = z_min_m;
    floor_soft_margin_m_ = std::max(soft_margin_m, 0.0);
}

CartesianArmTargetResult CartesianServoController::computeLinearMoveTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode,
    double dt_sec,
    uint64_t command_seq,
    CartesianServoPathState* path_state,
    CartesianVelocityIntegratorState* velocity_integrator_state,
    const CartesianServoStateContext* state_context
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;
    result.telemetry.attempted = true;
    result.telemetry.fk_duration_us = state.fk_duration_us;
    result.telemetry.warn_ik_duration_us = config_.warn_ik_duration_us;
    result.telemetry.fail_ik_duration_us = config_.fail_ik_duration_us;

    // Real/sim gating retired: linear move computes in every run mode.
    // The velocity-integrator plumbing is unused since the move switched to
    // the position-IK feedforward chain (kept in the signature for ABI/call
    // compatibility with the servo loop).
    (void)velocity_integrator_state;
    (void)state_context;
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
        // Anchor the path at the COMMAND-side pose (FK of the previously sent
        // joints) when available, falling back to the measured TCP: the whole
        // chain stays feedforward and there is no measured-vs-commanded jump
        // at path start (the physical state lags the command by the
        // controller servo lag).
        path_state->start_tcp_stand = finiteJoints(previous_safe_sent_q_deg)
            ? kinematics_->computeTcpStand(
                  command.arm_id,
                  previous_safe_sent_q_deg,
                  mountForArm(command.arm_id, left_mount_, right_mount_))
            : *state.tcp_stand;
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
        // Quintic profile: peak rate = kQuinticPeakRateRatio * mean rate, so
        // the speed-limited duration keeps the PEAK inside the configured max.
        const double limited_linear_duration =
            position_distance * kQuinticPeakRateRatio / config_.max_linear_move_speed_m_s;
        const double limited_angular_duration =
            path_state->orientation_mode == CartesianOrientationInterpolation::Slerp
                ? orientation_distance * kQuinticPeakRateRatio / config_.max_angular_move_speed_rad_s
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
        // Smooth the moving path reference with the same pose_track_smd filter
        // streaming TcpPoseTarget uses, anchored at the command-side start.
        if (config_.pose_track_smd.enable) {
            path_state->smd = std::make_shared<SmdPoseTracker>(config_.pose_track_smd);
            path_state->smd->reset(path_state->start_tcp_stand);
        }
    }

    if (!path_state->done) {
        path_state->elapsed_sec = std::min(path_state->elapsed_sec + std::max(0.0, dt_sec), path_state->duration_sec);
    }
    const double tau = path_state->duration_sec > 0.0
        ? std::clamp(path_state->elapsed_sec / path_state->duration_sec, 0.0, 1.0)
        : 1.0;
    const double path_s = quinticTimeScaling(tau);
    const Pose6D reference = LinearCartesianPlanner{}.sample(
        CartesianTrajectoryRequest{
            path_state->start_tcp_stand,
            path_state->target_tcp_stand,
            path_state->orientation_mode,
        },
        path_s
    );

    // Feedforward chain, same stack as streaming TcpPoseTarget: the quintic
    // path reference runs through the pose_track_smd filter, then position-IK
    // toward the filtered pose seeded from the previously SENT joints. The
    // measured state no longer feeds the per-tick command — the previous
    // v_ref + Kp * bodyError(measured, reference) velocity servo re-injected
    // encoder noise and the controller's stair-stepped state updates into
    // every tick, which is what made TcpLinearMove judder while TcpPoseTarget
    // ran smoothly. Measured pose is kept for tracking TELEMETRY only.
    Pose6D commanded = reference;
    if (path_state->smd) {
        path_state->smd->updateGoalFromCommand(reference);
        commanded = path_state->smd->step(std::max(0.0, dt_sec));
    }

    // done = time profile finished AND the smoothing filter settled on the
    // latched goal (with the filter disabled it settles instantly at s=1).
    const bool time_done = path_state->elapsed_sec >= path_state->duration_sec - 1e-12;
    const bool smd_settled = !path_state->smd ||
        (math::positionDistance(commanded, path_state->target_tcp_stand) <= kLinearMoveSettlePositionM &&
         math::orientationDistanceRad(commanded, path_state->target_tcp_stand) <= kLinearMoveSettleOrientationRad);
    path_state->done = time_done && smd_settled;

    // Command-side velocity norms for telemetry (post-filter pose step / dt).
    double commanded_linear_norm_m_s = 0.0;
    double commanded_angular_norm_rad_s = 0.0;
    if (path_state->has_last_commanded && dt_sec > 0.0) {
        commanded_linear_norm_m_s =
            math::positionDistance(path_state->last_commanded_tcp_stand, commanded) / dt_sec;
        commanded_angular_norm_rad_s =
            math::orientationDistanceRad(path_state->last_commanded_tcp_stand, commanded) / dt_sec;
    }
    path_state->last_commanded_tcp_stand = commanded;
    path_state->has_last_commanded = true;

    const Vec6 error = math::bodyErrorLocal(*state.tcp_stand, reference);

    CartesianArmTargetResult ik_result = solveIkArmTargetFromTcpStand(
        *kinematics_,
        config_,
        mountForArm(command.arm_id, left_mount_, right_mount_),
        command.arm_id,
        commanded,
        state,
        previous_safe_sent_q_deg,
        run_mode
    );
    ik_result.telemetry.requested_twist_linear_norm_m_s = commanded_linear_norm_m_s;
    ik_result.telemetry.requested_twist_angular_norm_rad_s = commanded_angular_norm_rad_s;
    ik_result.telemetry.applied_twist_linear_norm_m_s = commanded_linear_norm_m_s;
    ik_result.telemetry.applied_twist_angular_norm_rad_s = commanded_angular_norm_rad_s;
    ik_result.telemetry.path_active = path_state->active && !path_state->done;
    ik_result.telemetry.path_s = path_s;
    ik_result.telemetry.path_position_error_m = std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
    ik_result.telemetry.path_orientation_error_rad = std::sqrt(error.rx * error.rx + error.ry * error.ry + error.rz * error.rz);
    ik_result.telemetry.path_line_deviation_m = math::lineDeviation(
        path_state->start_tcp_stand,
        path_state->target_tcp_stand,
        *state.tcp_stand
    );
    ik_result.telemetry.path_done = path_state->done;
    ik_result.telemetry.linear_move_duration_sec = path_state->duration_sec;
    ik_result.telemetry.linear_move_elapsed_sec = path_state->elapsed_sec;
    ik_result.telemetry.orientation_mode = orientationModeName(path_state->orientation_mode);
    return ik_result;
}

namespace {

// Env-gated per-tick CSV dump of the model-output -> joint twist pipeline
// (set RB_TWIST_PIPELINE_CSV=/path/to.csv). One row per arm per tick:
//   model output twist -> applied twist (clamp + orientation hold + floor) ->
//   [SMD: smoothed pose goal in the twist_via_smd path; nan otherwise] ->
//   qdot (velocity IK; nan in the twist_via_smd path) -> q_target (joint
//   command) -> IK conditioning diagnostics (sigma_min, applied damping,
//   per-tick joint jump vs seed, branch-jump flag).
void logTwistPipelineCsv(const ArmCommand& command,
                         const Vec6& model_out, const Vec6& applied,
                         const Pose6D* smd_pose, const JointArray* qdot,
                         const JointArray& q_target,
                         const CartesianSolveTelemetry& telem, uint64_t seq) {
    static const char* csv_path = std::getenv("RB_TWIST_PIPELINE_CSV");
    if (csv_path == nullptr || csv_path[0] == '\0') return;
    static std::mutex mtx;
    static std::ofstream ofs;
    static bool header_written = false;
    std::lock_guard<std::mutex> lock(mtx);
    if (!header_written) {
        ofs.open(csv_path, std::ios::out | std::ios::trunc);
        ofs << std::setprecision(9);
        ofs << "host_ns,t_sec,seq,arm,mode,"
               "mo_vx,mo_vy,mo_vz,mo_wx,mo_wy,mo_wz,"
               "ap_vx,ap_vy,ap_vz,ap_wx,ap_wy,ap_wz,"
               "smd_x,smd_y,smd_z,smd_rx,smd_ry,smd_rz,"
               "qd0,qd1,qd2,qd3,qd4,qd5,"
               "qc0,qc1,qc2,qc3,qc4,qc5,"
               "ik_sigma_min,ik_lambda,ik_jump_deg,ik_branch_jump,ik_branch_jump_clamped,"
               "smd_goal_clamped\n";
        header_written = true;
    }
    if (!ofs.good()) return;
    const long long host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    static long long first_ns = host_ns;          // first logged tick -> t_sec origin
    const double t_sec = static_cast<double>(host_ns - first_ns) / 1e9;
    ofs << host_ns << ',' << t_sec << ',' << seq << ','
        << (command.arm_id == ArmId::Left ? "left" : "right") << ','
        << toString(command.mode) << ','
        << model_out.x << ',' << model_out.y << ',' << model_out.z << ','
        << model_out.rx << ',' << model_out.ry << ',' << model_out.rz << ','
        << applied.x << ',' << applied.y << ',' << applied.z << ','
        << applied.rx << ',' << applied.ry << ',' << applied.rz << ',';
    if (smd_pose != nullptr) {
        ofs << smd_pose->x << ',' << smd_pose->y << ',' << smd_pose->z << ','
            << smd_pose->rx << ',' << smd_pose->ry << ',' << smd_pose->rz << ',';
    } else {
        ofs << "nan,nan,nan,nan,nan,nan,";
    }
    if (qdot != nullptr) {
        for (int i = 0; i < kDof; ++i) ofs << (*qdot)[i] << ',';
    } else {
        ofs << "nan,nan,nan,nan,nan,nan,";
    }
    for (int i = 0; i < kDof; ++i) ofs << q_target[i] << ',';
    ofs << telem.ik_min_singular_value << ',' << telem.ik_applied_damping << ','
        << telem.ik_solution_jump_deg << ',' << (telem.ik_branch_jump_suspected ? 1 : 0)
        << ',' << (telem.ik_branch_jump_clamped ? 1 : 0)
        << ',' << (telem.twist_smd_goal_clamped ? 1 : 0)
        << '\n';
}

}  // namespace

CartesianArmTargetResult CartesianServoController::computeTwistTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode,
    double dt_sec,
    uint64_t command_seq,
    CartesianTwistHoldState* hold_state,
    CartesianVelocityIntegratorState* velocity_integrator_state,
    const CartesianServoStateContext* state_context
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;
    result.telemetry.attempted = true;
    result.telemetry.fk_duration_us = state.fk_duration_us;
    result.telemetry.warn_ik_duration_us = config_.warn_ik_duration_us;
    result.telemetry.fail_ik_duration_us = config_.fail_ik_duration_us;

    // Real/sim gating retired: streaming twist computes in every run mode.
    (void)run_mode;
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
    const Vec6 model_out_twist = requested;  // received model twist, pre clamp/hold
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
    // Tier-2 floor assist: zero a downward stand-frame v_z when the commanded
    // TCP is at/below the plane (+ soft margin) so lateral motion keeps sliding
    // along the plane. `requested` is a local twist here — rotate to stand,
    // clamp only the linear z component, rotate back.
    if (floor_enabled_ && state.tcp_stand->z <= floor_z_min_m_ + floor_soft_margin_m_) {
        Vec6 stand_twist = math::twistLocalToStand(requested, *state.tcp_stand);
        if (stand_twist.z < 0.0) {
            stand_twist.z = 0.0;
            requested = math::twistStandToLocal(stand_twist, *state.tcp_stand);
            result.telemetry.floor_vz_clamped = true;
        }
    }

    result.telemetry.twist_clamped = twist_clamped;
    result.telemetry.applied_twist_linear_norm_m_s = linearNorm(requested);
    result.telemetry.applied_twist_angular_norm_rad_s = angularNorm(requested);

    // twist_via_smd: integrate the clamped/filtered local twist into a stand-frame
    // pose goal, smooth it through the SMD pose tracker (same as streaming
    // TcpPoseTarget), then position-IK the smoothed pose -> joint. Bypasses the
    // velocity-IK + joint integrator below.
    if (config_.twist_via_smd_enable) {
        const double smd_dt = std::max(0.0, dt_sec);
        if (!hold_state->twist_smd) {
            hold_state->twist_smd = std::make_shared<SmdPoseTracker>(config_.pose_track_smd);
            hold_state->twist_smd->reset(*state.tcp_stand);
            hold_state->twist_smd_goal = *state.tcp_stand;
            hold_state->twist_smd->updateGoalFromCommand(hold_state->twist_smd_goal);
        }
        Pose6D twist_delta;
        twist_delta.x = requested.x * smd_dt;
        twist_delta.y = requested.y * smd_dt;
        twist_delta.z = requested.z * smd_dt;
        twist_delta.rx = requested.rx * smd_dt;
        twist_delta.ry = requested.ry * smd_dt;
        twist_delta.rz = requested.rz * smd_dt;
        hold_state->twist_smd_goal =
            math::composeDeltaLocal(hold_state->twist_smd_goal, twist_delta);

        // Anti-windup (back-calculation feedback for this feedforward integrator):
        // clamp the integrated goal so it never LEADS the measured TCP by more
        // than the configured budget. Without it, a stalled arm (IK clamp / fault
        // / can't track) lets the goal run away unbounded, then lurch on recovery.
        // During healthy tracking the lead is just the SMD lag (~mm / sub-deg), so
        // this never engages. Position and orientation are clamped independently;
        // both budgets <= 0 disable it (behavior-preserving).
        bool goal_clamped = false;
        const Pose6D& measured_tcp_stand = *state.tcp_stand;
        Pose6D& smd_goal = hold_state->twist_smd_goal;
        if (config_.twist_smd_goal_max_lead_m > 0.0) {
            const double pos_lead = math::positionDistance(measured_tcp_stand, smd_goal);
            if (pos_lead > config_.twist_smd_goal_max_lead_m) {
                const double s = config_.twist_smd_goal_max_lead_m / pos_lead;
                smd_goal.x = measured_tcp_stand.x + (smd_goal.x - measured_tcp_stand.x) * s;
                smd_goal.y = measured_tcp_stand.y + (smd_goal.y - measured_tcp_stand.y) * s;
                smd_goal.z = measured_tcp_stand.z + (smd_goal.z - measured_tcp_stand.z) * s;
                goal_clamped = true;
            }
        }
        if (config_.twist_smd_goal_max_lead_rad > 0.0) {
            const double ori_lead = math::orientationDistanceRad(measured_tcp_stand, smd_goal);
            if (ori_lead > config_.twist_smd_goal_max_lead_rad) {
                const double s = config_.twist_smd_goal_max_lead_rad / ori_lead;
                // Slerp orientation from measured toward goal by s; keep the
                // already position-clamped translation.
                const Pose6D ori_clamped =
                    math::interpolateLinear(measured_tcp_stand, smd_goal, true, s);
                smd_goal.rx = ori_clamped.rx;
                smd_goal.ry = ori_clamped.ry;
                smd_goal.rz = ori_clamped.rz;
                goal_clamped = true;
            }
        }

        hold_state->twist_smd->updateGoalFromCommand(hold_state->twist_smd_goal);
        const Pose6D smd_pose = hold_state->twist_smd->step(smd_dt);

        CartesianArmTargetResult smd_result = solveIkArmTargetFromTcpStand(
            *kinematics_,
            config_,
            mountForArm(command.arm_id, left_mount_, right_mount_),
            command.arm_id,
            smd_pose,
            state,
            previous_safe_sent_q_deg,
            run_mode
        );
        smd_result.telemetry.requested_twist_linear_norm_m_s =
            result.telemetry.requested_twist_linear_norm_m_s;
        smd_result.telemetry.requested_twist_angular_norm_rad_s =
            result.telemetry.requested_twist_angular_norm_rad_s;
        smd_result.telemetry.applied_twist_linear_norm_m_s = linearNorm(requested);
        smd_result.telemetry.applied_twist_angular_norm_rad_s = angularNorm(requested);
        smd_result.telemetry.twist_clamped = twist_clamped;
        smd_result.telemetry.twist_smd_goal_clamped = goal_clamped;
        logTwistPipelineCsv(command, model_out_twist, requested, &smd_pose, nullptr,
                            smd_result.q_target_deg, smd_result.telemetry, command_seq);
        return smd_result;
    }

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
    CartesianArmTargetResult final_result = integrateVelocityTarget(
        result,
        config_,
        state.q_actual_deg,
        velocity,
        dt_sec,
        command.mode,
        command_seq,
        velocity_integrator_state,
        state_context
    );
    logTwistPipelineCsv(command, model_out_twist, requested, nullptr, &velocity.qdot_deg_s,
                        final_result.q_target_deg, final_result.telemetry, command_seq);
    return final_result;
}

CartesianArmTargetResult CartesianServoController::computeCircleMoveTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode,
    double dt_sec,
    uint64_t command_seq,
    CartesianCircleMoveState* circle_state,
    CartesianVelocityIntegratorState* velocity_integrator_state,
    const CartesianServoStateContext* state_context
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;
    result.telemetry.attempted = true;
    result.telemetry.fk_duration_us = state.fk_duration_us;
    result.telemetry.warn_ik_duration_us = config_.warn_ik_duration_us;
    result.telemetry.fail_ik_duration_us = config_.fail_ik_duration_us;

    // Real/sim gating retired: circle move computes in every run mode.
    if (!config_.enable_benchmark_primitives ||
        !config_.circle_move.allow_in_simulation ||
        config_.circle_move.allow_in_real) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "tcp_circle_move_benchmark_primitives_disabled";
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
    if (!circle_state || !state.tcp_stand || !state.has_valid_tcp_pose || !state.has_valid_joint_state) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "tcp_pose_unavailable";
        result.telemetry.status = "unavailable";
        result.telemetry.reason = result.reason;
        return result;
    }
    if (!command.has_tcp_circle_move) {
        result.verdict = SafetyVerdict::InvalidCommand;
        result.reason = "missing_tcp_circle_move";
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        return result;
    }

    const bool continuing_active_circle = command_seq == 0 && circle_state->active;
    if (!continuing_active_circle && (!circle_state->active || circle_state->seq != command_seq)) {
        if (command.tcp_circle_move.frame != TcpCircleFrame::Stand ||
            command.tcp_circle_move.center_mode != TcpCircleCenterMode::StartOnCircle ||
            command.tcp_circle_move.orientation_mode != LinearMoveOrientationMode::Constant) {
            result.verdict = SafetyVerdict::InvalidCommand;
            result.reason = "tcp_circle_move_unsupported_option";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }
        if (!std::isfinite(command.tcp_circle_move.diameter_m) ||
            !std::isfinite(command.tcp_circle_move.period_sec) ||
            !std::isfinite(command.tcp_circle_move.phase_advance_sec) ||
            command.tcp_circle_move.diameter_m <= 0.0 ||
            command.tcp_circle_move.period_sec <= 0.0 ||
            command.tcp_circle_move.repeat <= 0 ||
            command.tcp_circle_move.phase_advance_sec < 0.0 ||
            command.tcp_circle_move.phase_advance_sec >
                0.25 * command.tcp_circle_move.period_sec + 1e-12) {
            result.verdict = SafetyVerdict::InvalidCommand;
            result.reason = "tcp_circle_move_invalid_parameter";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }
        if (command.tcp_circle_move.diameter_m > config_.circle_move.max_diameter_m + 1e-12) {
            result.verdict = SafetyVerdict::InvalidCommand;
            result.reason = "tcp_circle_move_diameter_limit_exceeded";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }
        if (command.tcp_circle_move.period_sec < config_.circle_move.min_period_sec - 1e-12) {
            result.verdict = SafetyVerdict::InvalidCommand;
            result.reason = "tcp_circle_move_period_limit_exceeded";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }

        int axis1 = 0;
        int axis2 = 1;
        if (!circleAxes(command.tcp_circle_move.plane, &axis1, &axis2)) {
            result.verdict = SafetyVerdict::InvalidCommand;
            result.reason = "tcp_circle_move_invalid_plane";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }

        *circle_state = CartesianCircleMoveState{};
        circle_state->active = true;
        circle_state->seq = command_seq;
        circle_state->command = command.tcp_circle_move;
        circle_state->start_tcp_stand = *state.tcp_stand;
        circle_state->reference_tcp_stand = *state.tcp_stand;
        circle_state->radius_m = 0.5 * command.tcp_circle_move.diameter_m;
        circle_state->duration_sec = command.tcp_circle_move.period_sec *
            static_cast<double>(command.tcp_circle_move.repeat);
        circle_state->axis1 = axis1;
        circle_state->axis2 = axis2;
        circle_state->center_x = state.tcp_stand->x;
        circle_state->center_y = state.tcp_stand->y;
        circle_state->center_z = state.tcp_stand->z;
        switch (axis1) {
            case 0:
                circle_state->center_x -= circle_state->radius_m;
                break;
            case 1:
                circle_state->center_y -= circle_state->radius_m;
                break;
            default:
                circle_state->center_z -= circle_state->radius_m;
                break;
        }
    }

    if (!circle_state->done) {
        circle_state->elapsed_sec = std::min(
            circle_state->elapsed_sec + std::max(0.0, dt_sec),
            circle_state->duration_sec
        );
        circle_state->done = circle_state->elapsed_sec >= circle_state->duration_sec - 1e-12;
    }

    const TcpCircleMoveCommand& circle = circle_state->command;
    const double radius = circle_state->radius_m;
    const double omega = 2.0 * kPi / circle.period_sec;
    const double reference_elapsed_sec = std::min(
        circle_state->elapsed_sec + circle.phase_advance_sec,
        circle_state->duration_sec
    );
    const double theta = omega * reference_elapsed_sec;
    Pose6D reference = circle_state->start_tcp_stand;
    reference.x = circle_state->center_x;
    reference.y = circle_state->center_y;
    reference.z = circle_state->center_z;
    setPoseAxisValue(&reference, circle_state->axis1, poseAxisValue(reference, circle_state->axis1) + radius * std::cos(theta));
    setPoseAxisValue(&reference, circle_state->axis2, poseAxisValue(reference, circle_state->axis2) + radius * std::sin(theta));
    circle_state->reference_tcp_stand = reference;

    const Vec6 error = math::bodyErrorLocal(*state.tcp_stand, reference);
    Vec6 v_ref_stand;
    setVecAxisValue(&v_ref_stand, circle_state->axis1, -radius * omega * std::sin(theta));
    setVecAxisValue(&v_ref_stand, circle_state->axis2, radius * omega * std::cos(theta));
    const Vec6 v_ref_local = math::twistStandToLocal(v_ref_stand, *state.tcp_stand);
    Vec6 v_cmd{
        v_ref_local.x + config_.path_kp_pos * error.x,
        v_ref_local.y + config_.path_kp_pos * error.y,
        v_ref_local.z + config_.path_kp_pos * error.z,
        v_ref_local.rx + config_.path_kp_ori * error.rx,
        v_ref_local.ry + config_.path_kp_ori * error.ry,
        v_ref_local.rz + config_.path_kp_ori * error.rz,
    };
    if (!finiteVec6(v_cmd)) {
        result.verdict = SafetyVerdict::IkFailed;
        result.reason = "non_finite_cartesian_servo_command";
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        return result;
    }
    result.telemetry.requested_twist_linear_norm_m_s = linearNorm(v_cmd);
    result.telemetry.requested_twist_angular_norm_rad_s = angularNorm(v_cmd);
    bool twist_clamped = false;
    if (!limitTwist(
            &v_cmd,
            config_.max_twist_linear_m_s,
            config_.max_twist_angular_rad_s,
            config_.exceed_limit_policy,
            &twist_clamped)) {
        result.verdict = SafetyVerdict::InvalidCommand;
        result.reason = "cartesian_circle_move_limit_exceeded";
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        result.telemetry.applied_twist_linear_norm_m_s = 0.0;
        result.telemetry.applied_twist_angular_norm_rad_s = 0.0;
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

    const double path_s = circle_state->duration_sec > 0.0
        ? std::clamp(circle_state->elapsed_sec / circle_state->duration_sec, 0.0, 1.0)
        : 1.0;
    const int repeat_index = std::min(
        circle.repeat - 1,
        static_cast<int>(std::floor(circle_state->elapsed_sec / circle.period_sec))
    );
    result.verdict = SafetyVerdict::Ok;
    result.telemetry.success = true;
    result.telemetry.status = "ok";
    result.telemetry.path_active = circle_state->active && !circle_state->done;
    result.telemetry.path_s = path_s;
    result.telemetry.path_done = circle_state->done;
    result.telemetry.linear_move_duration_sec = circle_state->duration_sec;
    result.telemetry.linear_move_elapsed_sec = circle_state->elapsed_sec;
    result.telemetry.orientation_mode = "constant";
    result.telemetry.twist_clamped = twist_clamped;
    result.telemetry.applied_twist_linear_norm_m_s = linearNorm(v_cmd);
    result.telemetry.applied_twist_angular_norm_rad_s = angularNorm(v_cmd);
    result.telemetry.path_position_error_m =
        std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
    result.telemetry.path_orientation_error_rad =
        std::sqrt(error.rx * error.rx + error.ry * error.ry + error.rz * error.rz);
    result.telemetry.position_error_m = result.telemetry.path_position_error_m;
    result.telemetry.orientation_error_rad = result.telemetry.path_orientation_error_rad;
    result.telemetry.circle_active = result.telemetry.path_active;
    result.telemetry.circle_phase = theta;
    result.telemetry.circle_repeat_index = repeat_index;
    result.telemetry.circle_radius_m = radius;
    result.telemetry.circle_period_sec = circle.period_sec;
    result.telemetry.circle_position_error_m = result.telemetry.path_position_error_m;
    result.telemetry.circle_orientation_error_rad = result.telemetry.path_orientation_error_rad;
    result.telemetry.circle_done = circle_state->done;

    return integrateVelocityTarget(
        result,
        config_,
        state.q_actual_deg,
        velocity,
        dt_sec,
        ControlMode::TcpCircleMove,
        command_seq,
        velocity_integrator_state,
        state_context
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
