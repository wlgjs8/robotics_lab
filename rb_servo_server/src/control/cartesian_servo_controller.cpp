#include "rb_servo/control/cartesian_servo_controller.hpp"

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

constexpr double kPi = 3.14159265358979323846;
constexpr double kConstantOrientationToleranceRad = 1e-6;

Matrix3 identityMatrix() {
    return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
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

Matrix3 transposeMatrix(const Matrix3& a) {
    Matrix3 out{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out[row][col] = a[col][row];
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

Vector3 subtractVector(const Vector3& a, const Vector3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double normVector(const Vector3& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double dotVector(const Vector3& a, const Vector3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
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

Matrix3 rotationFromQuaternionOrRpy(const Pose6D& pose) {
    if (pose.quaternion_xyzw.has_value()) {
        const auto& q = *pose.quaternion_xyzw;
        const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (std::isfinite(norm) && norm > 0.0) {
            const double x = q[0] / norm;
            const double y = q[1] / norm;
            const double z = q[2] / norm;
            const double w = q[3] / norm;
            return {{
                {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)},
                {2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)},
                {2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)},
            }};
        }
    }
    return rotationFromRpy(pose.rx, pose.ry, pose.rz);
}

Transform transformFromPose(const Pose6D& pose) {
    return {rotationFromQuaternionOrRpy(pose), {pose.x, pose.y, pose.z}};
}

Vector3 logRotation(const Matrix3& r) {
    const double cos_theta = std::clamp((r[0][0] + r[1][1] + r[2][2] - 1.0) * 0.5, -1.0, 1.0);
    const double theta = std::acos(cos_theta);
    if (theta < 1e-9) {
        return {
            0.5 * (r[2][1] - r[1][2]),
            0.5 * (r[0][2] - r[2][0]),
            0.5 * (r[1][0] - r[0][1]),
        };
    }
    const double scale = theta / (2.0 * std::sin(theta));
    return {
        scale * (r[2][1] - r[1][2]),
        scale * (r[0][2] - r[2][0]),
        scale * (r[1][0] - r[0][1]),
    };
}

Vec6 bodyError(const Pose6D& current, const Pose6D& reference) {
    const Transform current_tf = transformFromPose(current);
    const Transform ref_tf = transformFromPose(reference);
    const Matrix3 current_r_t = transposeMatrix(current_tf.r);
    const Vector3 p_error_local = multiplyMatrixVector(
        current_r_t,
        subtractVector(ref_tf.t, current_tf.t)
    );
    const Vector3 r_error = logRotation(multiplyMatrix(current_r_t, ref_tf.r));
    return {p_error_local[0], p_error_local[1], p_error_local[2], r_error[0], r_error[1], r_error[2]};
}

double orientationDistanceRad(const Pose6D& start, const Pose6D& target) {
    const Transform start_tf = transformFromPose(start);
    const Transform target_tf = transformFromPose(target);
    return normVector(logRotation(multiplyMatrix(transposeMatrix(start_tf.r), target_tf.r)));
}

double positionDistance(const Pose6D& start, const Pose6D& target) {
    return normVector(Vector3{
        target.x - start.x,
        target.y - start.y,
        target.z - start.z,
    });
}

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
    const Transform reference_tf = transformFromPose(reference);
    const Vector3 velocity_stand{
        (path.target_tcp_stand.x - path.start_tcp_stand.x) / path.duration_sec,
        (path.target_tcp_stand.y - path.start_tcp_stand.y) / path.duration_sec,
        (path.target_tcp_stand.z - path.start_tcp_stand.z) / path.duration_sec,
    };
    const Vector3 velocity_local = multiplyMatrixVector(transposeMatrix(reference_tf.r), velocity_stand);
    out.x = velocity_local[0];
    out.y = velocity_local[1];
    out.z = velocity_local[2];

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
            const Vec6 delta = bodyError(reference, next);
            out.rx = delta.rx / dt;
            out.ry = delta.ry / dt;
            out.rz = delta.rz / dt;
        }
    }
    return out;
}

double lineDeviation(
    const Pose6D& start,
    const Pose6D& target,
    const Pose6D& current
) {
    const Vector3 a{start.x, start.y, start.z};
    const Vector3 b{target.x, target.y, target.z};
    const Vector3 p{current.x, current.y, current.z};
    const Vector3 ab = subtractVector(b, a);
    const Vector3 ap = subtractVector(p, a);
    const double ab2 = dotVector(ab, ab);
    if (ab2 <= 1e-12) return normVector(ap);
    const double u = std::clamp(dotVector(ap, ab) / ab2, 0.0, 1.0);
    const Vector3 projection{a[0] + u * ab[0], a[1] + u * ab[1], a[2] + u * ab[2]};
    return normVector(subtractVector(p, projection));
}

bool finiteVec6(const Vec6& value) {
    return std::isfinite(value.x) &&
           std::isfinite(value.y) &&
           std::isfinite(value.z) &&
           std::isfinite(value.rx) &&
           std::isfinite(value.ry) &&
           std::isfinite(value.rz);
}

bool angularZero(const Vec6& value) {
    return std::abs(value.rx) < 1e-12 &&
           std::abs(value.ry) < 1e-12 &&
           std::abs(value.rz) < 1e-12;
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

Vec6 twistStandToLocal(const Vec6& twist_stand, const Pose6D& current_tcp_stand) {
    const Transform current_tf = transformFromPose(current_tcp_stand);
    const Matrix3 stand_to_local = transposeMatrix(current_tf.r);
    const Vector3 linear = multiplyMatrixVector(
        stand_to_local,
        Vector3{twist_stand.x, twist_stand.y, twist_stand.z}
    );
    const Vector3 angular = multiplyMatrixVector(
        stand_to_local,
        Vector3{twist_stand.rx, twist_stand.ry, twist_stand.rz}
    );
    return {linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]};
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
    CartesianServoPathState* path_state
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

        const double orientation_distance = orientationDistanceRad(
            path_state->start_tcp_stand,
            path_state->target_tcp_stand
        );
        if (path_state->orientation_mode == CartesianOrientationInterpolation::Constant &&
            orientation_distance > kConstantOrientationToleranceRad) {
            *path_state = CartesianServoPathState{};
            result.verdict = SafetyVerdict::InvalidCommand;
            result.reason = "tcp_linear_move_constant_orientation_mismatch";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }

        const double position_distance = positionDistance(path_state->start_tcp_stand, path_state->target_tcp_stand);
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
    const Vec6 error = bodyError(*state.tcp_stand, reference);
    const Vec6 v_ref = referenceVelocityLocal(*path_state, reference, path_s);
    Vec6 v_cmd{
        v_ref.x + config_.path_kp * error.x,
        v_ref.y + config_.path_kp * error.y,
        v_ref.z + config_.path_kp * error.z,
        v_ref.rx + config_.path_kp * error.rx,
        v_ref.ry + config_.path_kp * error.ry,
        v_ref.rz + config_.path_kp * error.rz,
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

    JointArray q_next = state.q_actual_deg;
    for (int i = 0; i < kDof; ++i) {
        q_next[i] += velocity.qdot_deg_s[i] * std::max(0.0, dt_sec);
        if (!std::isfinite(q_next[i])) {
            result.verdict = SafetyVerdict::IkFailed;
            result.reason = "non_finite_cartesian_servo_joint_target";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }
    }

    result.verdict = SafetyVerdict::Ok;
    result.q_target_deg = q_next;
    result.telemetry.success = true;
    result.telemetry.status = "ok";
    result.telemetry.path_active = path_state->active && !path_state->done;
    result.telemetry.path_s = path_s;
    result.telemetry.path_position_error_m = std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
    result.telemetry.path_orientation_error_rad = std::sqrt(error.rx * error.rx + error.ry * error.ry + error.rz * error.rz);
    result.telemetry.path_line_deviation_m = lineDeviation(
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
    return result;
}

CartesianArmTargetResult CartesianServoController::computeTwistTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_safe_sent_q_deg,
    RunMode run_mode,
    double dt_sec,
    CartesianTwistHoldState* hold_state
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
        requested = twistStandToLocal(command.tcp_twist_stand, *state.tcp_stand);
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
    if (angularZero(requested)) {
        if (!hold_state->orientation_hold_active) {
            hold_state->hold_tcp_stand = *state.tcp_stand;
            hold_state->orientation_hold_active = true;
        }
        Pose6D orientation_reference = hold_state->hold_tcp_stand;
        orientation_reference.x = state.tcp_stand->x;
        orientation_reference.y = state.tcp_stand->y;
        orientation_reference.z = state.tcp_stand->z;
        orientation_error = bodyError(*state.tcp_stand, orientation_reference);
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

    JointArray q_next = state.q_actual_deg;
    for (int i = 0; i < kDof; ++i) {
        q_next[i] += velocity.qdot_deg_s[i] * std::max(0.0, dt_sec);
        if (!std::isfinite(q_next[i])) {
            result.verdict = SafetyVerdict::IkFailed;
            result.reason = "non_finite_cartesian_servo_joint_target";
            result.telemetry.status = "failed";
            result.telemetry.reason = result.reason;
            return result;
        }
    }

    result.verdict = SafetyVerdict::Ok;
    result.q_target_deg = q_next;
    result.telemetry.success = true;
    result.telemetry.status = "ok";
    result.telemetry.path_orientation_error_rad =
        std::sqrt(orientation_error.rx * orientation_error.rx +
                  orientation_error.ry * orientation_error.ry +
                  orientation_error.rz * orientation_error.rz);
    result.telemetry.orientation_error_rad = result.telemetry.path_orientation_error_rad;
    return result;
}

}  // namespace rb_servo
