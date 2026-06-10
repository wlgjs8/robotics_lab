#include "rb_servo/control/smd_pose_tracker.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

Eigen::Vector3d positionOf(const Pose6D& pose) {
    return {pose.x, pose.y, pose.z};
}

Eigen::Quaterniond rotationOf(const Pose6D& pose) {
    return Eigen::Quaterniond(math::rotationFromPose(pose)).normalized();
}

Pose6D poseFrom(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation) {
    return math::poseFromSe3(pinocchio::SE3(rotation.toRotationMatrix(), position));
}

// Direction-preserving norm clamp; max_norm <= 0 means unlimited.
Eigen::Vector3d clampNorm(const Eigen::Vector3d& value, double max_norm) {
    if (max_norm <= 0.0) return value;
    const double norm = value.norm();
    if (norm <= max_norm) return value;
    return value * (max_norm / norm);
}

}  // namespace

SmdPoseTracker::SmdPoseTracker(const PoseTrackSmdConfig& config) : config_(config) {}

void SmdPoseTracker::reset(const Pose6D& pose) {
    position_ = positionOf(pose);
    velocity_.setZero();
    rotation_ = rotationOf(pose);
    angular_velocity_.setZero();
    goal_position_ = position_;
    goal_rotation_ = rotation_;
    previous_command_.reset();
    active_ = true;
}

void SmdPoseTracker::deactivate() {
    active_ = false;
    previous_command_.reset();
}

void SmdPoseTracker::updateGoalFromCommand(const Pose6D& command_pose) {
    if (!active_) return;
    if (!previous_command_.has_value()) {
        // First command after (re)activation only latches the reference so an
        // engagement offset between the commanded pose and the current state
        // cannot inject a jump; deltas integrate from here.
        previous_command_ = command_pose;
        return;
    }
    const Eigen::Vector3d previous_position = positionOf(*previous_command_);
    const Eigen::Quaterniond previous_rotation = rotationOf(*previous_command_);
    const Eigen::Vector3d command_position = positionOf(command_pose);
    const Eigen::Quaterniond command_rotation = rotationOf(command_pose);

    goal_position_ += command_position - previous_position;
    // World-frame (left) rotation delta, matching the stand-frame translation
    // delta semantics above.
    goal_rotation_ = (command_rotation * previous_rotation.conjugate() * goal_rotation_).normalized();
    previous_command_ = command_pose;
}

Pose6D SmdPoseTracker::step(double dt_sec) {
    const double dt = std::max(0.0, dt_sec);
    if (!active_ || dt <= 0.0) {
        return poseFrom(position_, rotation_);
    }

    // Mass is fixed at 1.0; x_ddot = wn^2 * error - 2 * zeta * wn * x_dot.
    // The accel clamp is therefore a max-FORCE clamp, and the velocity clamp
    // caps the tracking speed; inside the limits the zeta/fn dynamics are
    // untouched.
    const double wn_lin = kTwoPi * config_.natural_frequency_linear_hz;
    const Eigen::Vector3d linear_accel = clampNorm(
        wn_lin * wn_lin * (goal_position_ - position_) -
            2.0 * config_.damping_ratio_linear * wn_lin * velocity_,
        config_.max_linear_accel_m_s2
    );
    // Semi-implicit Euler keeps the discrete system stable well past the
    // frequencies reachable at the 500 Hz servo rate.
    velocity_ = clampNorm(velocity_ + linear_accel * dt, config_.max_linear_velocity_m_s);
    position_ += velocity_ * dt;

    const double wn_ang = kTwoPi * config_.natural_frequency_angular_hz;
    const Eigen::Vector3d orientation_error = math::log3(
        (rotation_.conjugate() * goal_rotation_).toRotationMatrix()
    );
    const Eigen::Vector3d angular_accel = clampNorm(
        wn_ang * wn_ang * orientation_error -
            2.0 * config_.damping_ratio_angular * wn_ang * angular_velocity_,
        config_.max_angular_accel_rad_s2
    );
    angular_velocity_ = clampNorm(
        angular_velocity_ + angular_accel * dt, config_.max_angular_velocity_rad_s);
    rotation_ = (rotation_ * Eigen::Quaterniond(math::exp3(angular_velocity_ * dt))).normalized();

    return poseFrom(position_, rotation_);
}

Pose6D SmdPoseTracker::goalPose() const {
    return poseFrom(goal_position_, goal_rotation_);
}

}  // namespace rb_servo
