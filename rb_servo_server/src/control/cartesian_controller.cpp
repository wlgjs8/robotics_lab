#include "rb_servo/control/cartesian_controller.hpp"

#include "rb_servo/kinematics/ik_solver.hpp"
#include "rb_servo/math/se3.hpp"

#include <cmath>
#include <utility>

namespace rb_servo {
namespace {

bool isFiniteJoints(const JointArray& joints) {
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

CartesianController::CartesianController(
    const ArmMountConfig& left_mount,
    const ArmMountConfig& right_mount,
    const CartesianControlConfig& config,
    std::shared_ptr<IKinematics> kinematics
) : left_mount_(left_mount), right_mount_(right_mount), config_(config), kinematics_(std::move(kinematics)) {}

CartesianArmTargetResult CartesianController::computeArmJointTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;
    result.telemetry.attempted = true;
    result.telemetry.fk_duration_us = state.fk_duration_us;
    result.telemetry.warn_ik_duration_us = config_.warn_ik_duration_us;
    result.telemetry.fail_ik_duration_us = config_.fail_ik_duration_us;

    if (!kinematics_) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "kinematics_unavailable";
        result.telemetry.status = "unavailable";
        result.telemetry.reason = result.reason;
        return result;
    }

    if (!state.has_valid_joint_state || !isFiniteJoints(state.q_actual_deg)) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "invalid_joint_state";
        result.telemetry.status = "unavailable";
        result.telemetry.reason = result.reason;
        return result;
    }

    Pose6D target_tcp_stand;
    switch (command.mode) {
        case ControlMode::TcpPoseTarget:
            if (!command.has_tcp_target || !ik_solver::isFinitePose(command.tcp_target_stand)) {
                result.verdict = SafetyVerdict::CartesianUnavailable;
                result.reason = "invalid_tcp_target";
                result.telemetry.status = "unavailable";
                result.telemetry.reason = result.reason;
                return result;
            }
            target_tcp_stand = command.tcp_target_stand;
            break;
        case ControlMode::TcpLinearMove:
            // Real/sim gating retired: linear move computes in every run mode.
            if (!command.has_tcp_target || !state.tcp_stand || !state.has_valid_tcp_pose ||
                !ik_solver::isFinitePose(command.tcp_target_stand)) {
                result.verdict = SafetyVerdict::CartesianUnavailable;
                result.reason = "tcp_pose_unavailable";
                result.telemetry.status = "unavailable";
                result.telemetry.reason = result.reason;
                return result;
            }
            target_tcp_stand = LinearCartesianPlanner{}.sample(
                CartesianTrajectoryRequest{
                    *state.tcp_stand,
                    command.tcp_target_stand,
                    CartesianOrientationInterpolation::Slerp,
                },
                1.0
            );
            break;
        case ControlMode::TcpDeltaStand:
            if (!command.has_tcp_delta_stand || !state.tcp_stand || !state.has_valid_tcp_pose ||
                !ik_solver::isFinitePose(command.tcp_delta_stand)) {
                result.verdict = SafetyVerdict::CartesianUnavailable;
                result.reason = "tcp_pose_unavailable";
                result.telemetry.status = "unavailable";
                result.telemetry.reason = result.reason;
                return result;
            }
            target_tcp_stand = applyTcpDeltaStand(*state.tcp_stand, command.tcp_delta_stand);
            break;
        case ControlMode::TcpDeltaLocal:
            if (!command.has_tcp_delta_local || !state.tcp_stand || !state.has_valid_tcp_pose ||
                !ik_solver::isFinitePose(command.tcp_delta_local)) {
                result.verdict = SafetyVerdict::CartesianUnavailable;
                result.reason = "tcp_pose_unavailable";
                result.telemetry.status = "unavailable";
                result.telemetry.reason = result.reason;
                return result;
            }
            target_tcp_stand = applyTcpDeltaLocal(*state.tcp_stand, command.tcp_delta_local);
            break;
        default:
            result.verdict = SafetyVerdict::CartesianUnavailable;
            result.reason = "not_cartesian_mode";
            result.telemetry.status = "not_attempted";
            result.telemetry.reason = result.reason;
            return result;
    }

    return solveIkFromTcpStandTarget(command.arm_id, target_tcp_stand, state, previous_safe_sent_q_deg, run_mode);
}

CartesianArmTargetResult solveIkArmTargetFromTcpStand(
    IKinematics& kinematics,
    const CartesianControlConfig& config,
    const ArmMountConfig& mount,
    ArmId arm_id,
    const Pose6D& target_tcp_stand,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;
    result.telemetry.attempted = true;
    result.telemetry.fk_duration_us = state.fk_duration_us;
    result.telemetry.warn_ik_duration_us = config.warn_ik_duration_us;
    result.telemetry.fail_ik_duration_us = config.fail_ik_duration_us;

    // Seed from the previously SENT target, not the measured joint state: the
    // physical state lags the command by the controller servo lag (~100 ms),
    // so an actual-state seed (a) sits far from the new solution during fast
    // motion (more DLS iterations under max_step_deg), and (b) feeds the
    // robot's physical response back into the next command, which combined
    // with the IK tolerance dead zone produced a 3-5 Hz relay limit cycle in
    // 500 Hz streaming TcpPoseTarget teleop. The previous sent target is one
    // tick away from the new solution and keeps the chain feedforward.
    const JointArray seed_q_deg = isFiniteJoints(previous_safe_sent_q_deg)
        ? previous_safe_sent_q_deg
        : state.q_actual_deg;
    const IkResult ik = kinematics.solveIk(
        arm_id,
        target_tcp_stand,
        seed_q_deg,
        mount
    );
    result.telemetry.ik_duration_us = ik.duration_us;
    result.telemetry.ik_iterations = ik.iterations;
    result.telemetry.position_error_m = ik.position_error_m;
    result.telemetry.orientation_error_rad = ik.orientation_error_rad;
    result.telemetry.ik_timed_out = ik.timed_out || ik.reason == ik_solver::kReasonTimeout;
    result.telemetry.ik_warn_duration_exceeded =
        config.warn_ik_duration_us > 0.0 && ik.duration_us > config.warn_ik_duration_us;
    result.telemetry.ik_fail_duration_exceeded =
        run_mode == RunMode::Simulation &&
        config.fail_ik_duration_us > 0.0 &&
        ik.duration_us > config.fail_ik_duration_us;
    if (!ik.success) {
        result.verdict = ik.reason == ik_solver::kReasonKinematicsUnavailable
            ? SafetyVerdict::CartesianUnavailable
            : SafetyVerdict::IkFailed;
        result.reason = ik.reason;
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        return result;
    }

    if (result.telemetry.ik_fail_duration_exceeded) {
        result.verdict = SafetyVerdict::IkFailed;
        result.reason = "ik_duration_budget_exceeded";
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        return result;
    }

    if (!isFiniteJoints(ik.q_solution_deg)) {
        result.verdict = SafetyVerdict::IkFailed;
        result.reason = "non_finite_ik_solution";
        result.telemetry.status = "failed";
        result.telemetry.reason = result.reason;
        return result;
    }

    result.verdict = SafetyVerdict::Ok;
    result.q_target_deg = ik.q_solution_deg;
    result.telemetry.success = true;
    result.telemetry.status = "ok";
    result.telemetry.reason = ik.reason;
    return result;
}

CartesianArmTargetResult CartesianController::solveIkFromTcpStandTarget(
    ArmId arm_id,
    const Pose6D& target_tcp_stand,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode
) {
    return solveIkArmTargetFromTcpStand(
        *kinematics_,
        config_,
        mountForArm(arm_id, left_mount_, right_mount_),
        arm_id,
        target_tcp_stand,
        state,
        previous_safe_sent_q_deg,
        run_mode
    );
}

Pose6D CartesianController::applyTcpDeltaStand(
    const Pose6D& current_tcp_stand,
    const Pose6D& delta
) const {
    return math::composeDeltaStand(current_tcp_stand, delta);
}

Pose6D CartesianController::applyTcpDeltaLocal(
    const Pose6D& current_tcp_stand,
    const Pose6D& delta
) const {
    return math::composeDeltaLocal(current_tcp_stand, delta);
}

}  // namespace rb_servo
