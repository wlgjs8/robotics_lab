#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rb_servo::control {

// Continuous 500 Hz post-follower pose conditioner. It deliberately has no
// command-goal integrator, goal lead, or chunk-boundary concept: the follower's
// emitted pose is the reference every tick and all follower bookkeeping remains
// on that pre-filter stream.
//
// This stage was introduced after the 2026-07-31 measurements showed policy
// knots roughly 2x rougher than demonstrations, 16--42x the teleop joint-command
// 5--20 Hz acceleration energy, and stand modes near 13 Hz (right) / 17 Hz
// (left). A per-chunk FIR instead created a 5.74 Hz boundary comb, so continuity
// across frame swaps is a defining property here.
//
// Velocity feedforward is low-passed before entering the damping term. For
// zeta=1 and w_lpf=wn this gives
//
//   H(s) = wn^2 (3s + wn) / (s + wn)^3,
//
// which has zero steady-state ramp lag and retains -40 dB/dec asymptotic
// rolloff. Feeding raw reference velocity would degrade that to -20 dB/dec
// (about 0.54 gain at 13 Hz for nf=3.5 Hz), defeating the output filter.
class FollowerOutputSmd {
public:
    explicit FollowerOutputSmd(const FollowerOutputSmdConfig& config);

    // Seed pose and filtered velocity from the same follower state. The FF LPF
    // states are also set to xi, so engagement does not manufacture a transient.
    void reset(const Pose6D& pose, const Vec6& xi);

    Pose6D step(const Pose6D& reference, const Vec6& xi_ref, double dt_sec);

    // THE FORCE OVERLAY'S FOLD, on this stage's own state: translate the filtered
    // position and left-compose the filtered orientation by the displacement the
    // follower just absorbed (CartesianChunkFollower::absorbOffset), velocities
    // untouched. Reference and state move together, so the step() that follows
    // sees the same error and the same velocity it would have without the fold -
    // the emitted pose shifts by exactly the displacement and nothing else changes.
    void shift(const Eigen::Vector3d& dp_stand, const Eigen::Quaterniond& dR_stand);

    bool active() const { return active_; }
    void deactivate() { active_ = false; }

    // Last-step distance from the pre-filter reference to the emitted pose.
    double lagPos() const { return lag_pos_m_; }
    double lagAng() const { return lag_ang_rad_; }

private:
    Pose6D currentPose() const;

    FollowerOutputSmdConfig config_;
    bool active_ = false;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ff_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_ff_ = Eigen::Vector3d::Zero();
    double lag_pos_m_ = 0.0;
    double lag_ang_rad_ = 0.0;
};

}  // namespace rb_servo::control
