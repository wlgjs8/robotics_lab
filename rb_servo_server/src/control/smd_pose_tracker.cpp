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

// Direction-preserving norm clamp; max_norm <= 0 means unlimited. Sets
// `clipped` when the input norm exceeded the limit (telemetry only).
Eigen::Vector3d clampNorm(const Eigen::Vector3d& value, double max_norm, bool* clipped = nullptr) {
    if (max_norm <= 0.0) return value;
    const double norm = value.norm();
    if (norm <= max_norm) return value;
    if (clipped) *clipped = true;
    return value * (max_norm / norm);
}

}  // namespace

SmdPoseTracker::SmdPoseTracker(const PoseTrackSmdConfig& config) : config_(config) {}

void SmdPoseTracker::reset(const Pose6D& pose) {
    if (active_) ++reanchor_count_;  // reset() while active == a genuine re-anchor
    position_ = positionOf(pose);
    velocity_.setZero();
    rotation_ = rotationOf(pose);
    angular_velocity_.setZero();
    goal_position_ = position_;
    goal_rotation_ = rotation_;
    previous_goal_position_ = position_;
    previous_goal_rotation_ = rotation_;
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

    SmdStepInfo info;
    info.velocity_feedforward_used = config_.velocity_feedforward;

    // Optional velocity feedforward. The damping term acts on the velocity ERROR
    // (x_dot - goal_dot) instead of the absolute x_dot, zeroing steady-state lag
    // for ramp goals. The goal velocity is always estimated from the goal delta
    // accrued since the previous step(). The estimate is clamped to the max
    // tracking velocity so a sparse update can never inject a feedforward spike.
    Eigen::Vector3d goal_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_angular_velocity = Eigen::Vector3d::Zero();
    if (config_.velocity_feedforward) {
        goal_linear_velocity = (goal_position_ - previous_goal_position_) / dt;
        // Body-frame angular velocity of the goal (so(3) log of the goal
        // rotation delta); coincides with the state body frame when the
        // tracking error is small (the regime feedforward operates in).
        goal_angular_velocity =
            math::log3((previous_goal_rotation_.conjugate() * goal_rotation_).toRotationMatrix()) / dt;
        goal_linear_velocity = clampNorm(goal_linear_velocity, config_.max_linear_velocity_m_s,
                                         &info.goal_linear_velocity_ff_clipped);
        goal_angular_velocity = clampNorm(goal_angular_velocity, config_.max_angular_velocity_rad_s,
                                          &info.goal_angular_velocity_ff_clipped);
    }
    info.goal_linear_velocity = goal_linear_velocity;
    info.goal_angular_velocity = goal_angular_velocity;
    previous_goal_position_ = goal_position_;
    previous_goal_rotation_ = goal_rotation_;

    // Mass is fixed at 1.0; x_ddot = wn^2 * error - 2 * zeta * wn * (x_dot - goal_dot).
    // The accel clamp is therefore a max-FORCE clamp, and the velocity clamp
    // caps the tracking speed; inside the limits the zeta/fn dynamics are
    // untouched. Without feedforward goal_dot is zero (legacy behavior).
    const double wn_lin = kTwoPi * config_.natural_frequency_linear_hz;
    const Eigen::Vector3d linear_accel = clampNorm(
        wn_lin * wn_lin * (goal_position_ - position_) -
            2.0 * config_.damping_ratio_linear * wn_lin * (velocity_ - goal_linear_velocity),
        config_.max_linear_accel_m_s2, &info.linear_accel_clipped
    );
    // Semi-implicit Euler keeps the discrete system stable well past the
    // frequencies reachable at the 500 Hz servo rate.
    velocity_ = clampNorm(velocity_ + linear_accel * dt, config_.max_linear_velocity_m_s,
                          &info.linear_velocity_clipped);
    position_ += velocity_ * dt;

    const double wn_ang = kTwoPi * config_.natural_frequency_angular_hz;
    const Eigen::Vector3d orientation_error = math::log3(
        (rotation_.conjugate() * goal_rotation_).toRotationMatrix()
    );
    const Eigen::Vector3d angular_accel = clampNorm(
        wn_ang * wn_ang * orientation_error -
            2.0 * config_.damping_ratio_angular * wn_ang * (angular_velocity_ - goal_angular_velocity),
        config_.max_angular_accel_rad_s2, &info.angular_accel_clipped
    );
    angular_velocity_ = clampNorm(
        angular_velocity_ + angular_accel * dt, config_.max_angular_velocity_rad_s,
        &info.angular_velocity_clipped);
    rotation_ = (rotation_ * Eigen::Quaterniond(math::exp3(angular_velocity_ * dt))).normalized();

    last_step_info_ = info;
    return poseFrom(position_, rotation_);
}

Pose6D SmdPoseTracker::goalPose() const {
    return poseFrom(goal_position_, goal_rotation_);
}

Pose6D SmdPoseTracker::currentPose() const {
    return poseFrom(position_, rotation_);
}

bool SmdPoseTracker::driftedFrom(const Pose6D& reference, double pos_tol_m, double ang_tol_rad) const {
    const double pos_drift = (positionOf(reference) - position_).norm();
    if (pos_drift > pos_tol_m) return true;
    const double ang_drift =
        math::log3((rotation_.conjugate() * rotationOf(reference)).toRotationMatrix()).norm();
    return ang_drift > ang_tol_rad;
}

}  // namespace rb_servo
