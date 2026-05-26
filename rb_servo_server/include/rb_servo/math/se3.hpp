#pragma once

#include "rb_servo/core/types.hpp"

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/spatial/se3.hpp>

namespace rb_servo::math {

using Matrix3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;
using Vector6 = Eigen::Matrix<double, 6, 1>;

Eigen::Quaterniond quaternionFromXyzw(const std::array<double, 4>& xyzw);
std::array<double, 4> quaternionToXyzw(const Eigen::Quaterniond& q);

Matrix3 rotationFromPose(const Pose6D& pose);
pinocchio::SE3 se3FromPose(const Pose6D& pose);
Pose6D poseFromSe3(const pinocchio::SE3& transform);

Vector3 log3(const Matrix3& rotation);
Matrix3 exp3(const Vector3& omega);
Vector6 log6Local(const pinocchio::SE3& current, const pinocchio::SE3& target);

Pose6D composeDeltaStand(const Pose6D& current_tcp_stand, const Pose6D& delta);
Pose6D composeDeltaLocal(const Pose6D& current_tcp_stand, const Pose6D& delta);
Pose6D interpolateLinear(
    const Pose6D& start_tcp_stand,
    const Pose6D& target_tcp_stand,
    bool slerp_orientation,
    double s
);

Vec6 bodyErrorLocal(const Pose6D& current_tcp_stand, const Pose6D& reference_tcp_stand);
Vec6 twistStandToLocal(const Vec6& twist_stand, const Pose6D& current_tcp_stand);

double orientationDistanceRad(const Pose6D& start_tcp_stand, const Pose6D& target_tcp_stand);
double positionDistance(const Pose6D& start_tcp_stand, const Pose6D& target_tcp_stand);
double lineDeviation(
    const Pose6D& start_tcp_stand,
    const Pose6D& target_tcp_stand,
    const Pose6D& current_tcp_stand
);

}  // namespace rb_servo::math
