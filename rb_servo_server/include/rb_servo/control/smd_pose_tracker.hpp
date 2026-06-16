#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rb_servo {

// Spring-Mass-Damper pose tracking filter for streaming TcpPoseTarget teleop.
//
// Incoming command poses feed a goal-pose DELTA INTEGRATOR (the first command
// after (re)activation only latches the reference; subsequent command deltas
// are accumulated onto the goal), and the published pose follows that goal as
// a critically-dampable second-order system stepped at the servo rate:
//
//   x_ddot = wn^2 * (goal - x) - 2 * zeta * wn * x_dot      (mass fixed at 1.0)
//
// Translation integrates in stand frame; rotation integrates the body-frame
// angular state via so(3) log/exp. Translation and rotation have independent
// damping ratio / natural frequency (see PoseTrackSmdConfig in config.hpp).
class SmdPoseTracker {
public:
    explicit SmdPoseTracker(const PoseTrackSmdConfig& config);

    bool active() const { return active_; }

    // Initialize the filter and the goal integrator at `pose` with zero
    // velocity. The next updateGoalFromCommand() call only latches the
    // command reference (no jump), deltas integrate from there.
    void reset(const Pose6D& pose);

    void deactivate();

    // Feed one received command pose into the goal delta integrator.
    void updateGoalFromCommand(const Pose6D& command_pose);

    // Advance the SMD state by dt toward the integrated goal and return the
    // smoothed pose to publish. Requires active().
    Pose6D step(double dt_sec);

    Pose6D goalPose() const;

private:
    PoseTrackSmdConfig config_;
    bool active_ = false;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();  // body frame
    Eigen::Vector3d goal_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond goal_rotation_ = Eigen::Quaterniond::Identity();
    // Goal pose at the previous step(), used to estimate the goal velocity for
    // the optional velocity feedforward term (config_.velocity_feedforward).
    Eigen::Vector3d previous_goal_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond previous_goal_rotation_ = Eigen::Quaterniond::Identity();
    std::optional<Pose6D> previous_command_;
};

}  // namespace rb_servo
