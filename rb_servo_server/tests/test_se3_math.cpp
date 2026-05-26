#include <array>
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "rb_servo/math/se3.hpp"

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

bool approx(double a, double b, double tolerance) {
    return std::abs(a - b) <= tolerance;
}

bool vectorApprox(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    double tolerance
) {
    return (a - b).norm() <= tolerance;
}

bool finite3(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

bool finite6(const Eigen::Matrix<double, 6, 1>& value) {
    return value.array().isFinite().all();
}

rb_servo::Pose6D poseFromRpy(double roll, double pitch, double yaw) {
    rb_servo::Pose6D pose;
    pose.rx = roll;
    pose.ry = pitch;
    pose.rz = yaw;
    return pose;
}

rb_servo::Pose6D poseFromQuaternion(const Eigen::Quaterniond& q) {
    rb_servo::Pose6D pose;
    pose.quaternion_xyzw = rb_servo::math::quaternionToXyzw(q);
    return pose;
}

bool testLogNearZero() {
    const Eigen::Vector3d omega(1e-9, 0.0, 0.0);
    const Eigen::Vector3d logged = rb_servo::math::log3(rb_servo::math::exp3(omega));
    RB_CHECK(finite3(logged));
    RB_CHECK(logged.norm() < 1e-8);
    return true;
}

bool testLogAtNinetyDegrees() {
    const Eigen::Vector3d axis = Eigen::Vector3d(0.0, 0.0, 1.0);
    const Eigen::Vector3d expected = axis * (kPi * 0.5);
    const Eigen::Vector3d logged = rb_servo::math::log3(rb_servo::math::exp3(expected));
    RB_CHECK(finite3(logged));
    RB_CHECK(vectorApprox(logged, expected, 1e-10));
    return true;
}

bool testLogNearPi() {
    const std::array<Eigen::Vector3d, 4> axes{
        Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d(1.0, 2.0, -3.0).normalized(),
    };
    const double angle = kPi - 1e-6;
    for (const Eigen::Vector3d& axis : axes) {
        const Eigen::Vector3d logged = rb_servo::math::log3(rb_servo::math::exp3(axis * angle));
        RB_CHECK(finite3(logged));
        RB_CHECK(approx(logged.norm(), angle, 1e-8));
    }
    return true;
}

bool testLogExactlyPi() {
    const std::array<Eigen::Vector3d, 3> axes{
        Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ(),
    };
    for (const Eigen::Vector3d& axis : axes) {
        const Eigen::Vector3d logged = rb_servo::math::log3(rb_servo::math::exp3(axis * kPi));
        RB_CHECK(finite3(logged));
        RB_CHECK(approx(logged.norm(), kPi, 1e-10));
    }
    return true;
}

bool testQuaternionSignInvariance() {
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-0.35, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.45, Eigen::Vector3d::UnitX())
    );
    rb_servo::Pose6D positive = poseFromQuaternion(q);
    rb_servo::Pose6D negative;
    negative.quaternion_xyzw = std::array<double, 4>{-q.x(), -q.y(), -q.z(), -q.w()};
    RB_CHECK((rb_servo::math::rotationFromPose(positive) - rb_servo::math::rotationFromPose(negative)).norm() < 1e-12);
    return true;
}

bool testPoseQuaternionXyzwConvention() {
    const double angle = 1.0;
    rb_servo::Pose6D pose;
    pose.quaternion_xyzw = std::array<double, 4>{std::sin(angle * 0.5), 0.0, 0.0, std::cos(angle * 0.5)};

    const Eigen::Vector3d logged = rb_servo::math::log3(rb_servo::math::rotationFromPose(pose));
    RB_CHECK(finite3(logged));
    RB_CHECK(vectorApprox(logged, Eigen::Vector3d(angle, 0.0, 0.0), 1e-12));

    const Eigen::Quaterniond q = rb_servo::math::quaternionFromXyzw(*pose.quaternion_xyzw);
    RB_CHECK(approx(q.x(), std::sin(angle * 0.5), 1e-12));
    RB_CHECK(approx(q.y(), 0.0, 1e-12));
    RB_CHECK(approx(q.z(), 0.0, 1e-12));
    RB_CHECK(approx(q.w(), std::cos(angle * 0.5), 1e-12));
    return true;
}

bool testPoseQuaternionAndRpyConsistency() {
    const double roll = 0.31;
    const double pitch = -0.42;
    const double yaw = 0.53;
    const Eigen::Matrix3d rotation = (
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
    ).toRotationMatrix();
    const rb_servo::Pose6D rpy_pose = poseFromRpy(roll, pitch, yaw);
    const rb_servo::Pose6D quaternion_pose = poseFromQuaternion(Eigen::Quaterniond(rotation));
    RB_CHECK((rb_servo::math::rotationFromPose(rpy_pose) - rotation).norm() < 1e-12);
    RB_CHECK((rb_servo::math::rotationFromPose(quaternion_pose) - rotation).norm() < 1e-12);
    RB_CHECK((rb_servo::math::rotationFromPose(rpy_pose) - rb_servo::math::rotationFromPose(quaternion_pose)).norm() < 1e-12);
    return true;
}

bool assertBodyErrorMatchesPinocchioLog6(
    const pinocchio::SE3& current,
    const pinocchio::SE3& target,
    double tolerance
) {
    const Eigen::Matrix<double, 6, 1> expected = pinocchio::log6(current.actInv(target)).toVector();
    const rb_servo::Vec6 error = rb_servo::math::bodyErrorLocal(
        rb_servo::math::poseFromSe3(current),
        rb_servo::math::poseFromSe3(target)
    );
    Eigen::Matrix<double, 6, 1> actual;
    actual << error.x, error.y, error.z, error.rx, error.ry, error.rz;
    RB_CHECK(finite6(actual));
    RB_CHECK((actual - expected).norm() < tolerance);
    RB_CHECK((rb_servo::math::log6Local(current, target) - expected).norm() < tolerance);
    return true;
}

bool testBodyErrorMatchesPinocchioLog6() {
    const pinocchio::SE3 small_current(
        rb_servo::math::exp3(Eigen::Vector3d(0.001, -0.002, 0.003)),
        Eigen::Vector3d(0.2, -0.1, 0.3)
    );
    const pinocchio::SE3 small_target(
        rb_servo::math::exp3(Eigen::Vector3d(0.002, -0.001, 0.004)),
        Eigen::Vector3d(0.201, -0.099, 0.302)
    );
    RB_CHECK(assertBodyErrorMatchesPinocchioLog6(small_current, small_target, 1e-12));

    const pinocchio::SE3 current(
        rb_servo::math::exp3(Eigen::Vector3d(0.15, -0.05, 0.21)),
        Eigen::Vector3d(0.2, -0.1, 0.3)
    );
    const pinocchio::SE3 target(
        rb_servo::math::exp3(Eigen::Vector3d(-0.12, 0.18, 0.04)),
        Eigen::Vector3d(0.23, -0.04, 0.27)
    );
    RB_CHECK(assertBodyErrorMatchesPinocchioLog6(current, target, 1e-12));
    return true;
}

bool testTwistStandToLocalUsesRotationTranspose() {
    rb_servo::Pose6D tcp_stand;
    tcp_stand.quaternion_xyzw = std::array<double, 4>{
        0.0,
        0.0,
        std::sin(kPi * 0.25),
        std::cos(kPi * 0.25),
    };
    const rb_servo::Vec6 stand_twist{1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    const rb_servo::Vec6 local_twist = rb_servo::math::twistStandToLocal(stand_twist, tcp_stand);

    RB_CHECK(approx(local_twist.x, 0.0, 1e-12));
    RB_CHECK(approx(local_twist.y, -1.0, 1e-12));
    RB_CHECK(approx(local_twist.z, 0.0, 1e-12));
    RB_CHECK(approx(local_twist.rx, 0.0, 1e-12));
    RB_CHECK(approx(local_twist.ry, 0.0, 1e-12));
    RB_CHECK(approx(local_twist.rz, 1.0, 1e-12));
    return true;
}

}  // namespace

int main() {
    if (!testLogNearZero()) return 1;
    if (!testLogAtNinetyDegrees()) return 1;
    if (!testLogNearPi()) return 1;
    if (!testLogExactlyPi()) return 1;
    if (!testQuaternionSignInvariance()) return 1;
    if (!testPoseQuaternionXyzwConvention()) return 1;
    if (!testPoseQuaternionAndRpyConsistency()) return 1;
    if (!testBodyErrorMatchesPinocchioLog6()) return 1;
    if (!testTwistStandToLocalUsesRotationTranspose()) return 1;
    return 0;
}
