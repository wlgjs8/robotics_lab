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
    // clips above.
    bool goal_linear_velocity_ff_clipped = false;
    bool goal_angular_velocity_ff_clipped = false;
    bool velocity_feedforward_used = false;
    // Estimated goal velocity actually applied as feedforward (stand/body frame).
    Eigen::Vector3d goal_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_angular_velocity = Eigen::Vector3d::Zero();
    // Manipulability velocity scale applied this step (1.0 = none, < 1 = slowed near
    // a singularity). Telemetry only.
    double singularity_velocity_scale = 1.0;
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

    // reset() that carries a velocity: the filter state starts at `pose` MOVING with
    // `stand_twist` (x/y/z m/s in the stand frame, rx/ry/rz rad/s in the stand
    // frame, converted to the body-frame angular state here), clamped to the
    // profile's velocity caps. For a hand-off from a driver that was mid-motion
    // (chunk follower -> pose-track SMD): the SMD then decelerates on its own caps
    // instead of stopping the arm in one tick. The goal integrator still latches at
    // `pose`, so with a stationary command the state settles back onto it.
    void reset(const Pose6D& pose, const Vec6& stand_twist);

    // Pin the filter state AND the goal integrator at `pose` with zero velocity,
    // re-latching the command reference so the next updateGoalFromCommand()
    // integrates nothing. Same end state as reset(), with two differences that
    // matter to a caller that pins EVERY tick for as long as some other stage
    // holds the output:
    //   - it does not count a re-anchor (reanchorCount() stays a count of
    //     genuine ones, not of hold ticks), and
    //   - it keeps the retained min-singular sample, because a hold does not
    //     move the arm and so does not invalidate the pose context.
    // Use reset() for a real re-anchor and this for a held output.
    void holdAt(const Pose6D& pose);

    void deactivate();

    // Feed one received command pose into the goal delta integrator.
    void updateGoalFromCommand(const Pose6D& command_pose);

    // Advance the SMD state by dt toward the integrated goal and return the
    // smoothed pose to publish. Requires active().
    Pose6D step(double dt_sec);

    // Diagnostics from the most recent step() (clip flags, feedforward source,
    // applied goal-velocity estimate). Pure telemetry.
    const SmdStepInfo& lastStepInfo() const { return last_step_info_; }

    // Number of re-anchors (reset() while already active) since construction.
    std::uint64_t reanchorCount() const { return reanchor_count_; }

    // Feed the most recent IK solve's task-Jacobian min singular value so the NEXT
    // step() can scale the tracking velocity down near a singularity (manipulability
    // guard, config singularity_scale_*). Pure velocity scaling: never touches IK
    // damping/iterations.
    //
    // A sample <= 0 means "this solve did not measure sigma" — the IK converged before
    // taking a DLS step (common in healthy poses: ~41% of solves) or it failed outright.
    // Such a sample is NOT a reading of 'well conditioned'; it carries no information, so
    // it must not overwrite the last real measurement. Overwriting it was a live bug: an
    // IK max_iterations failure fed 0.0, the guard read that as unknown, and the velocity
    // cap snapped back to FULL SPEED on exactly the deepest-singular ticks. Measured
    // 2026-08-14 (logs/servo_log_20260814_100223.csv, left arm): during two singularity
    // events the scale alternated 0.12 <-> 1.0 at 6-18 Hz on 38-43% of ticks. Holding the
    // last real sample keeps the guard engaged until the arm actually backs out.
    //
    // This is not free in healthy operation: replaying that run's sigma, 16% of normal
    // teleop ticks change (they sit inside the ramp and used to be spuriously released to
    // full speed by an unmeasured neighbour), mean scale 0.746 -> 0.717 on those ticks.
    // That is the guard doing what it was configured to do, not a regression.
    void setMinSingular(double sigma) {
        if (sigma > 0.0) last_min_singular_ = sigma;
    }

    Pose6D goalPose() const;

    // The SMD's currently-tracked (published) pose.
    Pose6D currentPose() const;

    // THE FORCE GATE ON THIS PATH (2026-09-04). Overwrite the published translation
    // with `position_stand` (the caller has already cut the advance INTO the contact
    // out of this tick's step) and drop the velocity component that still points
    // into it, so the next step does not carry that momentum back in. The goal is
    // NOT touched: this is a hold of the STATE, not a shift of the goal, which is
    // what an absolute-target source (UMI teleop, a policy's absolute setpoints)
    // needs - a shifted goal would leave the arm permanently offset from the source
    // after the contact is released. Tangential motion (sliding along the contact)
    // and the retreating component pass untouched.
    void constrainTranslation(const Eigen::Vector3d& position_stand,
                              const Eigen::Vector3d& outward_normal_stand);

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
    // Patch 4: diagnostics.
    SmdStepInfo last_step_info_;
    std::uint64_t reanchor_count_ = 0;
    double last_min_singular_ = -1.0;  // < 0 = unknown -> no singularity velocity scaling
};

}  // namespace rb_servo
