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

void CartesianServoController::setFloorConstraint(
    bool enabled,
    double z_min_m,
    double soft_margin_m,
    std::vector<FloorCheckPointConfig> tcp_offset_points
) {
    floor_enabled_ = enabled && std::isfinite(z_min_m) && std::isfinite(soft_margin_m);
    floor_z_min_m_ = z_min_m;
    floor_soft_margin_m_ = std::max(soft_margin_m, 0.0);
    floor_tcp_offset_points_ = std::move(tcp_offset_points);
}

CartesianArmTargetResult CartesianServoController::computeLinearMoveTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode,
    double dt_sec,
    uint64_t command_seq,
    CartesianServoPathState* path_state,
    const CartesianServoStateContext* state_context
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;
    result.telemetry.attempted = true;
    result.telemetry.fk_duration_us = state.fk_duration_us;
    result.telemetry.warn_ik_duration_us = config_.warn_ik_duration_us;
    result.telemetry.fail_ik_duration_us = config_.fail_ik_duration_us;

    // Real/sim gating retired: linear move computes in every run mode.
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

}  // namespace rb_servo
