#include "rb_servo/control/cartesian_controller.hpp"

#include "rb_servo/kinematics/ik_solver.hpp"
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

JointArray jointDelta(const JointArray& q, const JointArray& seed) {
    JointArray out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = q[i] - seed[i];
    }
    return out;
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
    result.telemetry.ik_min_singular_value = ik.min_singular_value;
    result.telemetry.ik_applied_damping = ik.applied_damping;
    result.telemetry.ik_solution_jump_deg = ik.solution_jump_deg;
    result.telemetry.ik_branch_jump_suspected = ik.branch_jump_suspected;
    result.telemetry.ik_branch_jump_clamped = ik.branch_jump_clamped;
    result.telemetry.ik_branch_jump_rate_limited = ik.branch_jump_rate_limited;
    result.telemetry.ik_branch_jump_raw_deg =
        ik.branch_jump_details_valid ? ik.raw_solution_jump_deg : ik.solution_jump_deg;
    result.telemetry.ik_branch_jump_limit_deg = ik.branch_jump_limit_deg;
    result.telemetry.ik_branch_jump_scale = ik.branch_jump_scale;
    result.telemetry.ik_branch_jump_retry_count = ik.branch_jump_retry_count;
    result.telemetry.ik_joint_limit_worst_index = ik.joint_limit_worst_index;
    result.telemetry.ik_joint_limit_worst_margin_deg = ik.joint_limit_worst_margin_deg;
    result.telemetry.ik_joint_limit_pinned = ik.joint_limit_pinned;
    result.telemetry.ik_limit_relief_weight = ik.limit_relief_weight;
    result.telemetry.ik_limit_avoidance_step_deg = ik.limit_avoidance_step_deg;
    result.telemetry.q_ik_seed_deg = ik.branch_jump_details_valid ? ik.q_seed_deg : seed_q_deg;
    result.telemetry.q_ik_raw_solution_deg =
        ik.branch_jump_details_valid ? ik.q_raw_solution_deg : ik.q_solution_deg;
    result.telemetry.q_ik_solution_deg = ik.q_solution_deg;
    result.telemetry.q_ik_raw_delta_deg = ik.branch_jump_details_valid
        ? ik.q_raw_delta_deg
        : jointDelta(ik.q_solution_deg, seed_q_deg);
    result.telemetry.q_ik_delta_deg = ik.branch_jump_details_valid
        ? ik.q_solution_delta_deg
        : jointDelta(ik.q_solution_deg, seed_q_deg);
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

}  // namespace rb_servo
