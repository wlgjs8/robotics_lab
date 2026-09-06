#include "rb_servo/control/follower_output_smd.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo::control {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;
// RESEED BOUNDS (2026-09-06: 0.05 m / 0.10 rad -> 0.10 m / 0.30 rad). A reseed is a
// SNAP of the emitted target onto the reference, i.e. a kick by the whole lag. It
// exists for genuine reference jumps (re-anchor, cold seed), not for lag the tracker
// is in the middle of closing. At 0.10 rad it sat exactly on the plan-vs-sent soft
// bound the divergence leash floors at, and servo_log_20260906_131740.csv 56.228 s
// crossed it by 0.0003 rad with 29 mm of ordinary acceleration lag: a 28 mm one-tick
// step, 7,500 deg/s^2 on the wire. The bounds now equal the HARD divergence latch
// (kPoseTrackDivergenceHard*), above which the follower faults anyway.
constexpr double kReseedPositionM = 0.10;
constexpr double kReseedOrientationRad = 0.30;

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
    double dt_sec,
    const Vec6* xi_dot_ref,
    bool reference_body_derivatives
) {
    reseeded_last_step_ = false;
    if (!active_) {
        reset(reference, xi_ref);
        reseeded_last_step_ = true;
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
        reseeded_last_step_ = true;
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
    Eigen::Vector3d accel_ff = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_accel_ff = Eigen::Vector3d::Zero();
    const bool profile_ff = config_.profile_feedforward && xi_dot_ref != nullptr;
    if (profile_ff) {
        // Exact profile feed-forward: the sampled v and a of the jerk-limited plan.
        // No low-pass on v (the sample is continuous, unlike the chained end-state
        // velocity the legacy path had to filter), and a enters as a term of its own,
        // so the tracker's error dynamics see only what the profile does not carry.
        velocity_ff_ = reference_velocity;
        // The follower reports angular derivatives in the REFERENCE body;
        // this integrator stores them in its own current output body. These
        // coincide only at zero orientation error. Transport both derivatives,
        // including the rotating-output-frame term in angular acceleration.
        const Eigen::Matrix3d reference_to_output = reference_body_derivatives
            ? Eigen::Matrix3d(rotation_.conjugate().toRotationMatrix() * reference_rotation.toRotationMatrix())
            : Eigen::Matrix3d::Identity();
        angular_velocity_ff_ = reference_to_output * reference_angular_velocity;
        damping_velocity = velocity_ff_;
        damping_angular_velocity = angular_velocity_ff_;
        accel_ff = Eigen::Vector3d(xi_dot_ref->x, xi_dot_ref->y, xi_dot_ref->z);
        angular_accel_ff = reference_to_output *
            Eigen::Vector3d(xi_dot_ref->rx, xi_dot_ref->ry, xi_dot_ref->rz) -
            (reference_body_derivatives ? Eigen::Vector3d(angular_velocity_.cross(angular_velocity_ff_))
                                        : Eigen::Vector3d::Zero());
    } else if (config_.velocity_ff) {
        const double ff_linear_hz = config_.velocity_ff_lpf_hz > 0.0
            ? config_.velocity_ff_lpf_hz
            : config_.nf_linear_hz;
        const double ff_angular_hz = config_.velocity_ff_lpf_hz > 0.0
            ? config_.velocity_ff_lpf_hz
            : config_.nf_angular_hz;
        velocity_ff_ += kTwoPi * ff_linear_hz * (reference_velocity - velocity_ff_) * dt;
        const Eigen::Vector3d angular_input = reference_body_derivatives
            ? Eigen::Vector3d(rotation_.conjugate() * reference_rotation * reference_angular_velocity)
            : reference_angular_velocity;
        angular_velocity_ff_ +=
            kTwoPi * ff_angular_hz *
            (angular_input - angular_velocity_ff_) * dt;
        damping_velocity = config_.velocity_ff_linear_gain * velocity_ff_;
        damping_angular_velocity = angular_velocity_ff_;
    }

    const Eigen::Vector3d linear_acceleration = accel_ff +
        wn_linear * wn_linear * (reference_position - position_) +
        2.0 * config_.damping_ratio * wn_linear * (damping_velocity - velocity_);
    velocity_ += linear_acceleration * dt;
    position_ += velocity_ * dt;

    const Eigen::Vector3d angular_acceleration = angular_accel_ff +
        wn_angular * wn_angular * orientation_error +
        2.0 * config_.damping_ratio * wn_angular *
            (damping_angular_velocity - angular_velocity_);
    angular_velocity_ += angular_acceleration * dt;
    if (reference_body_derivatives && !profile_ff && config_.velocity_ff) {
        const Eigen::Quaterniond angular_increment(math::exp3(angular_velocity_ * dt));
        rotation_ = (rotation_ * angular_increment).normalized();
        // The retained LPF state describes a physical angular-velocity vector.
        // Express it in the NEXT output body before the next filter update.
        // A common stand-frame force fold leaves these body components intact.
        angular_velocity_ff_ = angular_increment.conjugate() * angular_velocity_ff_;
    } else {
        rotation_ = (rotation_ *
                     Eigen::Quaterniond(math::exp3(angular_velocity_ * dt))).normalized();
    }

    const Pose6D out = currentPose();
    lag_pos_m_ = math::positionDistance(reference, out);
    lag_ang_rad_ = math::orientationDistanceRad(reference, out);
    return out;
}

void FollowerOutputSmd::shift(const Eigen::Vector3d& dp_stand,
                              const Eigen::Quaterniond& dR_stand) {
    if (!active_) return;
    position_ += dp_stand;
    rotation_ = (dR_stand * rotation_).normalized();
}

Pose6D FollowerOutputSmd::currentPose() const {
    return math::poseFromSe3(pinocchio::SE3(rotation_.toRotationMatrix(), position_));
}

}  // namespace rb_servo::control
