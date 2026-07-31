#include "rb_servo/control/follower_output_smd.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo::control {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kReseedPositionM = 0.05;
constexpr double kReseedOrientationRad = 0.10;

Eigen::Vector3d positionOf(const Pose6D& pose) {
    return {pose.x, pose.y, pose.z};
}

Eigen::Quaterniond rotationOf(const Pose6D& pose) {
    return Eigen::Quaterniond(math::rotationFromPose(pose)).normalized();
}

}  // namespace

FollowerOutputSmd::FollowerOutputSmd(const FollowerOutputSmdConfig& config)
    : config_(config) {}

void FollowerOutputSmd::reset(const Pose6D& pose, const Vec6& xi) {
    position_ = positionOf(pose);
    velocity_ = Eigen::Vector3d(xi.x, xi.y, xi.z);
    rotation_ = rotationOf(pose);
    angular_velocity_ = Eigen::Vector3d(xi.rx, xi.ry, xi.rz);
    velocity_ff_ = velocity_;
    angular_velocity_ff_ = angular_velocity_;
    lag_pos_m_ = 0.0;
    lag_ang_rad_ = 0.0;
    active_ = true;
}

Pose6D FollowerOutputSmd::step(
    const Pose6D& reference,
    const Vec6& xi_ref,
    double dt_sec
) {
    if (!active_) {
        reset(reference, xi_ref);
        return currentPose();
    }

    const Eigen::Vector3d reference_position = positionOf(reference);
    const Eigen::Quaterniond reference_rotation = rotationOf(reference);
    const Eigen::Vector3d orientation_error = math::log3(
        (rotation_.conjugate() * reference_rotation).toRotationMatrix());

    // A stale output state must never pull the command back toward an old pose.
    // Snap to the live pre-filter reference and inherit its chained velocity.
    if ((reference_position - position_).norm() > kReseedPositionM ||
        orientation_error.norm() > kReseedOrientationRad) {
        reset(reference, xi_ref);
        return currentPose();
    }

    const double dt = std::max(0.0, dt_sec);
    if (dt <= 0.0 || !std::isfinite(dt)) {
        const Pose6D out = currentPose();
        lag_pos_m_ = math::positionDistance(reference, out);
        lag_ang_rad_ = math::orientationDistanceRad(reference, out);
        return out;
    }

    const Eigen::Vector3d reference_velocity(xi_ref.x, xi_ref.y, xi_ref.z);
    const Eigen::Vector3d reference_angular_velocity(xi_ref.rx, xi_ref.ry, xi_ref.rz);
    const double wn_linear = kTwoPi * config_.nf_linear_hz;
    const double wn_angular = kTwoPi * config_.nf_angular_hz;

    Eigen::Vector3d damping_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d damping_angular_velocity = Eigen::Vector3d::Zero();
    if (config_.velocity_ff) {
        const double ff_linear_hz = config_.velocity_ff_lpf_hz > 0.0
            ? config_.velocity_ff_lpf_hz
            : config_.nf_linear_hz;
        const double ff_angular_hz = config_.velocity_ff_lpf_hz > 0.0
            ? config_.velocity_ff_lpf_hz
            : config_.nf_angular_hz;
        velocity_ff_ += kTwoPi * ff_linear_hz * (reference_velocity - velocity_ff_) * dt;
        angular_velocity_ff_ +=
            kTwoPi * ff_angular_hz *
            (reference_angular_velocity - angular_velocity_ff_) * dt;
        damping_velocity = velocity_ff_;
        damping_angular_velocity = angular_velocity_ff_;
    }

    const Eigen::Vector3d linear_acceleration =
        wn_linear * wn_linear * (reference_position - position_) +
        2.0 * config_.damping_ratio * wn_linear * (damping_velocity - velocity_);
    velocity_ += linear_acceleration * dt;
    position_ += velocity_ * dt;

    const Eigen::Vector3d angular_acceleration =
        wn_angular * wn_angular * orientation_error +
        2.0 * config_.damping_ratio * wn_angular *
            (damping_angular_velocity - angular_velocity_);
    angular_velocity_ += angular_acceleration * dt;
    rotation_ = (rotation_ *
                 Eigen::Quaterniond(math::exp3(angular_velocity_ * dt))).normalized();

    const Pose6D out = currentPose();
    lag_pos_m_ = math::positionDistance(reference, out);
    lag_ang_rad_ = math::orientationDistanceRad(reference, out);
    return out;
}

Pose6D FollowerOutputSmd::currentPose() const {
    return math::poseFromSe3(pinocchio::SE3(rotation_.toRotationMatrix(), position_));
}

}  // namespace rb_servo::control
