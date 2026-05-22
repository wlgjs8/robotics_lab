#include "rb_servo/control/cartesian_controller.hpp"

#include "rb_servo/kinematics/ik_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace rb_servo {
namespace {

using Matrix3 = std::array<std::array<double, 3>, 3>;
using Vector3 = std::array<double, 3>;

struct Transform {
    Matrix3 r{};
    Vector3 t{};
};

Matrix3 identityMatrix() {
    return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
}

Matrix3 addMatrix(const Matrix3& a, const Matrix3& b) {
    Matrix3 out{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out[row][col] = a[row][col] + b[row][col];
        }
    }
    return out;
}

Matrix3 scaleMatrix(const Matrix3& a, double scale) {
    Matrix3 out{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out[row][col] = a[row][col] * scale;
        }
    }
    return out;
}

Matrix3 multiplyMatrix(const Matrix3& a, const Matrix3& b) {
    Matrix3 out{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out[row][col] = a[row][0] * b[0][col] +
                            a[row][1] * b[1][col] +
                            a[row][2] * b[2][col];
        }
    }
    return out;
}

Vector3 multiplyMatrixVector(const Matrix3& a, const Vector3& v) {
    return {
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    };
}

Vector3 addVector(const Vector3& a, const Vector3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Matrix3 skew(const Vector3& v) {
    return {{{0.0, -v[2], v[1]}, {v[2], 0.0, -v[0]}, {-v[1], v[0], 0.0}}};
}

Matrix3 rotationFromRpy(double rx, double ry, double rz) {
    const double cr = std::cos(rx);
    const double sr = std::sin(rx);
    const double cp = std::cos(ry);
    const double sp = std::sin(ry);
    const double cy = std::cos(rz);
    const double sy = std::sin(rz);

    return {{
        {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp, cp * sr, cp * cr},
    }};
}

Transform transformFromPose(const Pose6D& pose) {
    return {rotationFromRpy(pose.rx, pose.ry, pose.rz), {pose.x, pose.y, pose.z}};
}

Pose6D poseFromTransform(const Transform& transform) {
    Pose6D pose;
    pose.x = transform.t[0];
    pose.y = transform.t[1];
    pose.z = transform.t[2];

    const double sy = -transform.r[2][0];
    pose.ry = std::asin(std::clamp(sy, -1.0, 1.0));
    const double cy = std::cos(pose.ry);
    if (std::abs(cy) > 1e-9) {
        pose.rx = std::atan2(transform.r[2][1], transform.r[2][2]);
        pose.rz = std::atan2(transform.r[1][0], transform.r[0][0]);
    } else {
        pose.rx = 0.0;
        pose.rz = std::atan2(-transform.r[0][1], transform.r[1][1]);
    }
    return pose;
}

Transform multiplyTransform(const Transform& a, const Transform& b) {
    Transform out;
    out.r = multiplyMatrix(a.r, b.r);
    out.t = addVector(multiplyMatrixVector(a.r, b.t), a.t);
    return out;
}

Transform expDelta(const Pose6D& delta) {
    const Vector3 v{delta.x, delta.y, delta.z};
    const Vector3 omega{delta.rx, delta.ry, delta.rz};
    const double theta2 = omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2];
    const double theta = std::sqrt(theta2);
    const Matrix3 omega_hat = skew(omega);
    const Matrix3 omega_hat2 = multiplyMatrix(omega_hat, omega_hat);

    double a = 1.0;
    double b = 0.5;
    double c = 1.0 / 6.0;
    if (theta > 1e-9) {
        a = std::sin(theta) / theta;
        b = (1.0 - std::cos(theta)) / theta2;
        c = (theta - std::sin(theta)) / (theta2 * theta);
    }

    const Matrix3 identity = identityMatrix();
    const Matrix3 r = addMatrix(addMatrix(identity, scaleMatrix(omega_hat, a)), scaleMatrix(omega_hat2, b));
    const Matrix3 v_matrix = addMatrix(addMatrix(identity, scaleMatrix(omega_hat, b)), scaleMatrix(omega_hat2, c));

    Transform out;
    out.r = r;
    out.t = multiplyMatrixVector(v_matrix, v);
    return out;
}

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
    std::shared_ptr<IKinematics> kinematics
) : left_mount_(left_mount), right_mount_(right_mount), kinematics_(std::move(kinematics)) {}

CartesianArmTargetResult CartesianController::computeArmJointTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;

    if (!kinematics_) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "kinematics_unavailable";
        return result;
    }

    if (!state.has_valid_joint_state || !isFiniteJoints(state.q_actual_deg)) {
        result.verdict = SafetyVerdict::CartesianUnavailable;
        result.reason = "invalid_joint_state";
        return result;
    }

    Pose6D target_tcp_stand;
    switch (command.mode) {
        case ControlMode::TcpPoseTarget:
            if (!command.has_tcp_target || !ik_solver::isFinitePose(command.tcp_target_stand)) {
                result.verdict = SafetyVerdict::CartesianUnavailable;
                result.reason = "invalid_tcp_target";
                return result;
            }
            target_tcp_stand = command.tcp_target_stand;
            break;
        case ControlMode::TcpDeltaStand:
            if (!command.has_tcp_delta_stand || !state.tcp_stand || !state.has_valid_tcp_pose ||
                !ik_solver::isFinitePose(command.tcp_delta_stand)) {
                result.verdict = SafetyVerdict::CartesianUnavailable;
                result.reason = "tcp_pose_unavailable";
                return result;
            }
            target_tcp_stand = applyTcpDeltaStand(*state.tcp_stand, command.tcp_delta_stand);
            break;
        case ControlMode::TcpDeltaLocal:
            if (!command.has_tcp_delta_local || !state.tcp_stand || !state.has_valid_tcp_pose ||
                !ik_solver::isFinitePose(command.tcp_delta_local)) {
                result.verdict = SafetyVerdict::CartesianUnavailable;
                result.reason = "tcp_pose_unavailable";
                return result;
            }
            target_tcp_stand = applyTcpDeltaLocal(*state.tcp_stand, command.tcp_delta_local);
            break;
        default:
            result.verdict = SafetyVerdict::CartesianUnavailable;
            result.reason = "not_cartesian_mode";
            return result;
    }

    return solveIkFromTcpStandTarget(command.arm_id, target_tcp_stand, state, previous_safe_sent_q_deg);
}

CartesianArmTargetResult CartesianController::solveIkFromTcpStandTarget(
    ArmId arm_id,
    const Pose6D& target_tcp_stand,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;

    const JointArray seed_q_deg = isFiniteJoints(state.q_actual_deg)
        ? state.q_actual_deg
        : previous_safe_sent_q_deg;
    const IkResult ik = kinematics_->solveIk(
        arm_id,
        target_tcp_stand,
        seed_q_deg,
        mountForArm(arm_id, left_mount_, right_mount_)
    );
    if (!ik.success) {
        result.verdict = ik.reason == ik_solver::kReasonKinematicsUnavailable
            ? SafetyVerdict::CartesianUnavailable
            : SafetyVerdict::IkFailed;
        result.reason = ik.reason;
        return result;
    }

    if (!isFiniteJoints(ik.q_solution_deg)) {
        result.verdict = SafetyVerdict::IkFailed;
        result.reason = "non_finite_ik_solution";
        return result;
    }

    result.verdict = SafetyVerdict::Ok;
    result.q_target_deg = ik.q_solution_deg;
    return result;
}

Pose6D CartesianController::applyTcpDeltaStand(
    const Pose6D& current_tcp_stand,
    const Pose6D& delta
) const {
    return poseFromTransform(multiplyTransform(expDelta(delta), transformFromPose(current_tcp_stand)));
}

Pose6D CartesianController::applyTcpDeltaLocal(
    const Pose6D& current_tcp_stand,
    const Pose6D& delta
) const {
    return poseFromTransform(multiplyTransform(transformFromPose(current_tcp_stand), expDelta(delta)));
}

}  // namespace rb_servo
