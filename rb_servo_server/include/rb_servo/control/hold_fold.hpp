#pragma once
// THE HOLD FOLD (2026-09-05). While a safety projection row (self-collision, ROI
// face, reach shell, floor) or the IK branch-jump throttle holds an arm back, the
// chunk follower's plan must not run ahead of the arm. Measured on the bimanual
// rollouts of 2026-09-04 23:40-23:54: the plan-minus-sent gap grew to 12-39 mm in
// 0.1-0.4 s during such holds, and the tick the IK entered branch_jump_rate_limited
// on that runaway target carried a 7.1-12.0k deg/s^2 kick (11 of 11 episodes with
// a gap over 10 mm; every episode with a gap under 1 mm stayed below 850). The
// collision fold of 2026-09-04 am booked only what the collision ROWS removed
// (q_req -> q_final); the velocity/acceleration clamps that cut the IK's chase of
// the runaway target were not booked, so the gap kept growing.
//
// This helper computes the WHOLE shortfall between the pose the plan emitted and
// the pose the arm was actually commanded to (overlay-stripped FK of the sent
// joints), as the (dp, dR) a CartesianChunkFollower::absorbOffset booking needs.
// Below `min_step` it is noise (IK residual) and nothing is booked; above
// `max_step` it is not a hold but a snap somewhere else, and folding it would
// hide a fault - the caller logs and declines.
#include "rb_servo/core/types.hpp"
#include "rb_servo/math/se3.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace rb_servo {
namespace control {

struct HoldFoldLimits {
    double min_step_m = 1e-5;
    double min_step_rad = 1e-5;
    double max_step_m = 0.03;
    double max_step_rad = 0.2;
};

struct HoldFoldDelta {
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();            // achieved - emitted, stand
    Eigen::Quaterniond dR = Eigen::Quaterniond::Identity();  // R_achieved * R_emitted^T
    double dist_m = 0.0;
    double angle_rad = 0.0;
};

// Returns true when a fold should be booked. `capped` (optional) reports a shortfall
// beyond the sanity cap: the caller declines the fold and says so.
inline bool computeHoldFold(const Pose6D& emitted, const Pose6D& achieved,
                            const HoldFoldLimits& lim, HoldFoldDelta* out, bool* capped) {
    if (capped != nullptr) *capped = false;
    if (out == nullptr) return false;
    const Eigen::Vector3d pe(emitted.x, emitted.y, emitted.z);
    const Eigen::Vector3d pa(achieved.x, achieved.y, achieved.z);
    if (!pe.allFinite() || !pa.allFinite()) return false;
    HoldFoldDelta d;
    d.dp = pa - pe;
    d.dist_m = d.dp.norm();
    const math::Matrix3 Re = math::rotationFromPose(emitted);
    const math::Matrix3 Ra = math::rotationFromPose(achieved);
    d.dR = Eigen::Quaterniond(Ra * Re.transpose()).normalized();
    d.angle_rad = std::abs(Eigen::AngleAxisd(d.dR).angle());
    if (!std::isfinite(d.dist_m) || !std::isfinite(d.angle_rad)) return false;
    if (d.dist_m > lim.max_step_m || d.angle_rad > lim.max_step_rad) {
        if (capped != nullptr) *capped = true;
        return false;
    }
    if (d.dist_m < lim.min_step_m && d.angle_rad < lim.min_step_rad) return false;
    *out = d;
    return true;
}

}  // namespace control
}  // namespace rb_servo
