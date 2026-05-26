#include "rb_servo/control/cartesian_trajectory_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace rb_servo {
namespace {

std::array<double, 4> normalizeQuaternionXyzw(const std::array<double, 4>& q) {
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!std::isfinite(norm) || norm <= 0.0) {
        return {0.0, 0.0, 0.0, 1.0};
    }
    return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}

std::array<double, 4> quaternionFromRpy(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    return normalizeQuaternionXyzw({
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    });
}

std::array<double, 4> poseQuaternionXyzw(const Pose6D& pose) {
    if (pose.quaternion_xyzw.has_value()) {
        return normalizeQuaternionXyzw(*pose.quaternion_xyzw);
    }
    return quaternionFromRpy(pose.rx, pose.ry, pose.rz);
}

void setPoseOrientationFromQuaternion(Pose6D* pose, const std::array<double, 4>& quaternion_xyzw) {
    const auto q = normalizeQuaternionXyzw(quaternion_xyzw);
    const double x = q[0];
    const double y = q[1];
    const double z = q[2];
    const double w = q[3];

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    pose->rx = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        pose->ry = std::copysign(3.14159265358979323846 / 2.0, sinp);
    } else {
        pose->ry = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    pose->rz = std::atan2(siny_cosp, cosy_cosp);
    pose->quaternion_xyzw = q;
}

double dotQuaternion(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

std::array<double, 4> slerpQuaternionXyzw(
    const std::array<double, 4>& start,
    const std::array<double, 4>& target,
    double s
) {
    const auto q0 = normalizeQuaternionXyzw(start);
    auto q1 = normalizeQuaternionXyzw(target);
    double dot = dotQuaternion(q0, q1);
    if (dot < 0.0) {
        for (double& value : q1) value = -value;
        dot = -dot;
    }
    dot = std::clamp(dot, -1.0, 1.0);

    if (dot > 0.9995) {
        return normalizeQuaternionXyzw({
            q0[0] + s * (q1[0] - q0[0]),
            q0[1] + s * (q1[1] - q0[1]),
            q0[2] + s * (q1[2] - q0[2]),
            q0[3] + s * (q1[3] - q0[3]),
        });
    }

    const double theta_0 = std::acos(dot);
    const double sin_theta_0 = std::sin(theta_0);
    const double theta = theta_0 * s;
    const double sin_theta = std::sin(theta);
    const double scale0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
    const double scale1 = sin_theta / sin_theta_0;

    return normalizeQuaternionXyzw({
        scale0 * q0[0] + scale1 * q1[0],
        scale0 * q0[1] + scale1 * q1[1],
        scale0 * q0[2] + scale1 * q1[2],
        scale0 * q0[3] + scale1 * q1[3],
    });
}

}  // namespace

Pose6D LinearCartesianPlanner::sample(
    const CartesianTrajectoryRequest& request,
    double s
) const {
    const double clamped_s = std::clamp(s, 0.0, 1.0);
    Pose6D out;
    out.x = request.start_tcp_stand.x + clamped_s * (request.target_tcp_stand.x - request.start_tcp_stand.x);
    out.y = request.start_tcp_stand.y + clamped_s * (request.target_tcp_stand.y - request.start_tcp_stand.y);
    out.z = request.start_tcp_stand.z + clamped_s * (request.target_tcp_stand.z - request.start_tcp_stand.z);

    const std::array<double, 4> q = request.orientation_mode == CartesianOrientationInterpolation::Constant
        ? poseQuaternionXyzw(request.start_tcp_stand)
        : slerpQuaternionXyzw(
            poseQuaternionXyzw(request.start_tcp_stand),
            poseQuaternionXyzw(request.target_tcp_stand),
            clamped_s
        );
    setPoseOrientationFromQuaternion(&out, q);
    return out;
}

}  // namespace rb_servo
