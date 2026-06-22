#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rb_servo {

// Per-step SMD diagnostics (A/B/C telemetry separation, Patch 4). Filled by every
// step(); read via lastStepInfo(). Pure telemetry — does not affect dynamics.
struct SmdStepInfo {
    bool linear_velocity_clipped = false;
    bool linear_accel_clipped = false;
    bool angular_velocity_clipped = false;
    bool angular_accel_clipped = false;
    // Whether the feedforward goal-velocity estimate itself was norm-clamped to the
    // max tracking velocity (VFF spike guard) — distinct from the state vel/accel
    // clips above. Important for verifying command_twist / VFF spikes.
    bool goal_linear_velocity_ff_clipped = false;
    bool goal_angular_velocity_ff_clipped = false;
    bool velocity_feedforward_used = false;
    // Effective feedforward source THIS step: "finite_difference" | "command_twist"
    // | "none" (ff off). When configured command_twist but the command twist was
    // missing/non-finite, this is "finite_difference" and command_twist_fallback=true.
    std::string velocity_feedforward_source = "none";
    bool command_twist_fallback = false;
    // Estimated goal velocity actually applied as feedforward (stand/body frame).
    Eigen::Vector3d goal_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_angular_velocity = Eigen::Vector3d::Zero();
};

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

    // Optional conditioned goal twist accompanying the current TcpPoseTarget
    // command (Patch 5). Used as the velocity feedforward source when
    // config.velocity_feedforward_source is "command_twist"/"auto"; ignored for
    // "finite_difference". Sticky (held across servo ticks until the next command).
    // linear = stand-frame m/s, angular = body-frame rad/s. Pass std::nullopt to
    // clear. Non-finite values are ignored (step() falls back to finite diff).
    void setCommandTwist(const std::optional<Eigen::Vector3d>& linear,
                         const std::optional<Eigen::Vector3d>& angular);

    // Advance the SMD state by dt toward the integrated goal and return the
    // smoothed pose to publish. Requires active().
    Pose6D step(double dt_sec);

    // Diagnostics from the most recent step() (clip flags, feedforward source,
    // applied goal-velocity estimate). Pure telemetry.
    const SmdStepInfo& lastStepInfo() const { return last_step_info_; }

    // Number of re-anchors (reset() while already active) since construction.
    std::uint64_t reanchorCount() const { return reanchor_count_; }

    Pose6D goalPose() const;

    // The SMD's currently-tracked (published) pose.
    Pose6D currentPose() const;

    // True if the tracker's held pose has drifted from `reference` beyond either
    // tolerance. Used to detect that another control path (JointTarget, fault hold,
    // command gap) moved the robot while the tracker stayed active with stale state,
    // so the caller can re-anchor before integrating the next command delta.
    bool driftedFrom(const Pose6D& reference, double pos_tol_m, double ang_tol_rad) const;

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
    // Patch 5: optional conditioned command twist for velocity feedforward.
    std::optional<Eigen::Vector3d> command_linear_velocity_;
    std::optional<Eigen::Vector3d> command_angular_velocity_;
    // Patch 4: diagnostics.
    SmdStepInfo last_step_info_;
    std::uint64_t reanchor_count_ = 0;
};

}  // namespace rb_servo
