#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pinocchio/spatial/explog.hpp>

namespace rb_servo::math {
namespace {

Vector3 translationFromPose(const Pose6D& pose) {
    return Vector3(pose.x, pose.y, pose.z);
}

double vectorNorm(const Vector3& value) {
    const double norm = value.norm();
    return std::isfinite(norm) ? norm : std::numeric_limits<double>::infinity();
}

pinocchio::SE3 deltaSe3FromPose(const Pose6D& delta) {
    pinocchio::Motion motion;
    motion.linear() = Vector3(delta.x, delta.y, delta.z);
    motion.angular() = Vector3(delta.rx, delta.ry, delta.rz);
    return pinocchio::exp6(motion);
}

Eigen::Quaterniond quaternionFromPose(const Pose6D& pose) {
    Eigen::Quaterniond q(rotationFromPose(pose));
    q.normalize();
    return q;
}

}  // namespace

Eigen::Quaterniond quaternionFromXyzw(const std::array<double, 4>& xyzw) {
    const double norm = std::sqrt(
        xyzw[0] * xyzw[0] +
        xyzw[1] * xyzw[1] +
        xyzw[2] * xyzw[2] +
        xyzw[3] * xyzw[3]
    );
    if (!std::isfinite(norm) || norm <= 0.0) {
        throw std::runtime_error("Pose6D quaternion_xyzw is non-finite or zero length");
    }
    Eigen::Quaterniond q(xyzw[3] / norm, xyzw[0] / norm, xyzw[1] / norm, xyzw[2] / norm);
    q.normalize();
    return q;
}

std::array<double, 4> quaternionToXyzw(const Eigen::Quaterniond& q_in) {
    Eigen::Quaterniond q = q_in;
    q.normalize();
    if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || !std::isfinite(q.w())) {
        throw std::runtime_error("Eigen quaternion is non-finite");
    }
    return std::array<double, 4>{q.x(), q.y(), q.z(), q.w()};
}

Matrix3 rotationFromPose(const Pose6D& pose) {
    if (pose.quaternion_xyzw.has_value()) {
        return quaternionFromXyzw(*pose.quaternion_xyzw).toRotationMatrix();
    }

    const Eigen::AngleAxisd roll(pose.rx, Vector3::UnitX());
    const Eigen::AngleAxisd pitch(pose.ry, Vector3::UnitY());
    const Eigen::AngleAxisd yaw(pose.rz, Vector3::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
}

pinocchio::SE3 se3FromPose(const Pose6D& pose) {
    return pinocchio::SE3(rotationFromPose(pose), translationFromPose(pose));
}

Pose6D poseFromSe3(const pinocchio::SE3& transform) {
    const Eigen::Vector3d ypr = transform.rotation().eulerAngles(2, 1, 0);
    Eigen::Quaterniond quaternion(transform.rotation());
    quaternion.normalize();

    Pose6D pose;
    pose.x = transform.translation().x();
    pose.y = transform.translation().y();
    pose.z = transform.translation().z();
    pose.rx = ypr.z();
    pose.ry = ypr.y();
    pose.rz = ypr.x();
    pose.quaternion_xyzw = quaternionToXyzw(quaternion);
    return pose;
}

Vector3 log3(const Matrix3& rotation) {
    return pinocchio::log3(rotation);
}

Matrix3 exp3(const Vector3& omega) {
    return pinocchio::exp3(omega);
}

Vector6 log6Local(const pinocchio::SE3& current, const pinocchio::SE3& target) {
    return pinocchio::log6(current.actInv(target)).toVector();
}

Pose6D composeDeltaStand(const Pose6D& current_tcp_stand, const Pose6D& delta) {
    return poseFromSe3(deltaSe3FromPose(delta) * se3FromPose(current_tcp_stand));
}

Pose6D composeDeltaLocal(const Pose6D& current_tcp_stand, const Pose6D& delta) {
    return poseFromSe3(se3FromPose(current_tcp_stand) * deltaSe3FromPose(delta));
}

Pose6D interpolateLinear(
    const Pose6D& start_tcp_stand,
    const Pose6D& target_tcp_stand,
    bool slerp_orientation,
    double s
) {
    const double clamped_s = std::clamp(s, 0.0, 1.0);
    const Vector3 start_t = translationFromPose(start_tcp_stand);
    const Vector3 target_t = translationFromPose(target_tcp_stand);
    const Vector3 t = start_t + clamped_s * (target_t - start_t);

    Eigen::Quaterniond q = quaternionFromPose(start_tcp_stand);
    if (slerp_orientation) {
        q = q.slerp(clamped_s, quaternionFromPose(target_tcp_stand));
    }
    q.normalize();

    return poseFromSe3(pinocchio::SE3(q.toRotationMatrix(), t));
}

Vec6 bodyErrorLocal(const Pose6D& current_tcp_stand, const Pose6D& reference_tcp_stand) {
    const Vector6 error = log6Local(se3FromPose(current_tcp_stand), se3FromPose(reference_tcp_stand));
    return {
        error[0],
        error[1],
        error[2],
        error[3],
        error[4],
        error[5],
    };
}

Vec6 twistStandToLocal(const Vec6& twist_stand, const Pose6D& current_tcp_stand) {
    const Matrix3 stand_to_local = rotationFromPose(current_tcp_stand).transpose();
    const Vector3 linear = stand_to_local * Vector3(twist_stand.x, twist_stand.y, twist_stand.z);
    const Vector3 angular = stand_to_local * Vector3(twist_stand.rx, twist_stand.ry, twist_stand.rz);
    return {linear.x(), linear.y(), linear.z(), angular.x(), angular.y(), angular.z()};
}

double orientationDistanceRad(const Pose6D& start_tcp_stand, const Pose6D& target_tcp_stand) {
    const Matrix3 relative = rotationFromPose(start_tcp_stand).transpose() * rotationFromPose(target_tcp_stand);
    return vectorNorm(log3(relative));
}

double positionDistance(const Pose6D& start_tcp_stand, const Pose6D& target_tcp_stand) {
    return vectorNorm(translationFromPose(target_tcp_stand) - translationFromPose(start_tcp_stand));
}

double lineDeviation(
    const Pose6D& start_tcp_stand,
    const Pose6D& target_tcp_stand,
    const Pose6D& current_tcp_stand
) {
    const Vector3 a = translationFromPose(start_tcp_stand);
    const Vector3 b = translationFromPose(target_tcp_stand);
    const Vector3 p = translationFromPose(current_tcp_stand);
    const Vector3 ab = b - a;
    const Vector3 ap = p - a;
    const double ab2 = ab.squaredNorm();
    if (ab2 <= 1e-12) return vectorNorm(ap);
    const double u = std::clamp(ap.dot(ab) / ab2, 0.0, 1.0);
    return vectorNorm(p - (a + u * ab));
}

}  // namespace rb_servo::math
