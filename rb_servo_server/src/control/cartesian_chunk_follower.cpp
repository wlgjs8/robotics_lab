#include "rb_servo/control/cartesian_chunk_follower.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace rb_servo::control {

namespace {

std::array<AxisLimit, 6> makeLimits(const CartesianChunkFollowerConfig& cfg) {
  return {cfg.lin, cfg.lin, cfg.lin, cfg.ang, cfg.ang, cfg.ang};
}

Eigen::Vector3d positionOf(const Pose6D& p) { return {p.x, p.y, p.z}; }

int sgn(double x, double deadband) { return (x > deadband) - (x < -deadband); }

int flankIndex(std::size_t k, int off, std::size_t n) {
  return std::clamp(static_cast<int>(k) + off, 0, static_cast<int>(n) - 1);
}

}  // namespace

CartesianChunkFollower::CartesianChunkFollower(const CartesianChunkFollowerConfig& cfg)
    : cfg_(cfg),
      limits_(makeLimits(cfg)),
      core_(limits_, 1.0 / 30.0, cfg.guard),
      window_(cfg.window) {}

void CartesianChunkFollower::reconfigure(const CartesianChunkFollowerConfig& cfg) {
  cfg_ = cfg;
  limits_ = makeLimits(cfg);
  core_.reconfigure(limits_, seg_dt_, cfg.guard);
  window_ = ChunkWindow(cfg.window);
  deactivate();
}

void CartesianChunkFollower::deactivate() {
  active_ = false;
  hold_paused_ = false;
  hold_pause_start_sec_ = 0.0;
  have_segment_ = false;
  window_.deactivate();
  diag_.consecutive_projection_errors = 0;
  diag_.consecutive_actual_lead_errors = 0;
  diag_.infeasible_fault = false;
  diag_.actual_lead_fault = false;
  actual_lead_checked_segment_ = -1;
  loading_dir_valid_ = false;
  contact_normal_owned_ = false;
  prev_loading_dir_valid_ = false;
  contact_shift_.setZero();
  diag_.loading_projection_active = false;
  diag_.contact_shift_m = 0.0;
}

void CartesianChunkFollower::pauseForHold(double now_sec) {
  if (!active_ || hold_paused_) return;
  hold_paused_ = true;
  hold_pause_start_sec_ = now_sec;
}

bool CartesianChunkFollower::expireHoldPause(double now_sec, double grace_sec) {
  if (!hold_paused_) return false;
  const double elapsed_sec = now_sec - hold_pause_start_sec_;
  if (!std::isfinite(now_sec) || !std::isfinite(grace_sec) || grace_sec <= 0.0 ||
      !std::isfinite(elapsed_sec) || elapsed_sec < 0.0 || elapsed_sec > grace_sec) {
    deactivate();
    return true;
  }
  return false;
}

HoldResumeResult CartesianChunkFollower::resumeFromHold(
    const Pose6D& live_reference,
    double now_sec,
    double grace_sec,
    double position_tolerance_m,
    double orientation_tolerance_rad) {
  if (!hold_paused_) return HoldResumeResult::NotPaused;
  const double elapsed_sec = now_sec - hold_pause_start_sec_;
  if (!std::isfinite(now_sec) || !std::isfinite(grace_sec) || grace_sec <= 0.0 ||
      !std::isfinite(elapsed_sec) || elapsed_sec < 0.0 || elapsed_sec > grace_sec) {
    deactivate();
    return HoldResumeResult::GraceExpired;
  }

  hold_paused_ = false;
  hold_pause_start_sec_ = 0.0;
  if (math::positionDistance(last_pose_, live_reference) > position_tolerance_m ||
      math::orientationDistanceRad(last_pose_, live_reference) >
          orientation_tolerance_rad) {
    // Leave the follower active so the servo loop's existing strict-divergence
    // policy can choose its already-audited safety reanchor or cold/fault path.
    return HoldResumeResult::Diverged;
  }
  return HoldResumeResult::WarmResumed;
}

void CartesianChunkFollower::setExternalReaction(
    const Eigen::Vector3d& loading_dir_stand,
    bool valid,
    bool contact_normal_owned) {
  loading_dir_stand_ = loading_dir_stand;
  loading_dir_valid_ = valid;
  contact_normal_owned_ = valid && contact_normal_owned;
}

void CartesianChunkFollower::reanchor(const Pose6D& reference) {
  hold_paused_ = false;
  hold_pause_start_sec_ = 0.0;
  R0_ref_ = Eigen::Quaterniond(math::rotationFromPose(reference)).normalized();
  std::array<double, 6> p0{reference.x, reference.y, reference.z, 0.0, 0.0, 0.0};
  core_.seed(p0);        // zero velocity/acceleration; keep the active window untouched
  have_segment_ = false; // force a fresh BVP solve from the new anchor on the next tick
  t_in_seg_ = 0.0;
  last_pose_ = reference;
  active_ = true;
}

void CartesianChunkFollower::submitFrame(const ChunkFrame& frame,
                                         const Pose6D& current_pose) {
  if (!window_.activate(frame)) return;  // frame too short; keep prior state

  seg_dt_ = window_.policyDt();
  core_.setDt(seg_dt_);

  if (!active_) {
    // Cold start: anchor the tangent at the current orientation and seed the
    // chained state at the current pose with zero velocity (no handoff jump).
    R0_ref_ = Eigen::Quaterniond(math::rotationFromPose(current_pose)).normalized();
    std::array<double, 6> p0{current_pose.x, current_pose.y, current_pose.z, 0, 0, 0};
    core_.seed(p0);            // v0 = a0 = 0
    have_segment_ = false;     // force a solve on the next tick
    t_in_seg_ = 0.0;
    last_pose_ = current_pose;
    current_grip_ = window_.gripAt(window_.index());
    active_ = true;
    contact_shift_.setZero();  // fresh anchor: no accumulated plan shift
  }
  // else: preempt — window frame swapped, chained core state kept. The next
  // boundary builds toward the new frame's poses from the current state.
}

void CartesianChunkFollower::submitDeltaFrame(const ChunkFrame& frame,
                                               const Pose6D& current_pose) {
  if (frame.delta.empty() || frame.delta.size() != frame.grip.size()) return;
  ChunkFrame integrated = frame;
  integrated.pose.clear();
  integrated.pose.reserve(frame.delta.size());
  Pose6D cursor = active_ ? last_pose_ : current_pose;
  for (const Vec6& d : frame.delta) {
    Pose6D delta;
    delta.x = d.x; delta.y = d.y; delta.z = d.z;
    delta.rx = d.rx; delta.ry = d.ry; delta.rz = d.rz;
    cursor = math::composeDeltaLocal(cursor, delta);
    integrated.pose.push_back(cursor);
  }
  // The new knots were just integrated from the EMITTED pose (which already
  // sits where the loading projection left it), so the accumulated plan shift
  // is baked into the new plan: clear the debt for the fresh window.
  contact_shift_.setZero();
  submitFrame(integrated, current_pose);
}

void CartesianChunkFollower::updateActualLead(const Pose6D& actual_pose) {
  diag_.actual_lead_m = math::positionDistance(last_pose_, actual_pose);
  diag_.actual_lead_rad = math::orientationDistanceRad(last_pose_, actual_pose);
  if (actual_lead_checked_segment_ == diag_.segments) return;
  actual_lead_checked_segment_ = diag_.segments;
  const bool violated =
      cfg_.max_actual_lead_m > 0.0 && cfg_.max_actual_lead_rad > 0.0 &&
      (diag_.actual_lead_m > cfg_.max_actual_lead_m ||
       diag_.actual_lead_rad > cfg_.max_actual_lead_rad);
  diag_.consecutive_actual_lead_errors =
      violated ? diag_.consecutive_actual_lead_errors + 1 : 0;
  diag_.actual_lead_fault = cfg_.max_consecutive_actual_lead_errors > 0 &&
      diag_.consecutive_actual_lead_errors >= cfg_.max_consecutive_actual_lead_errors;
}

Pose6D CartesianChunkFollower::tick(double dt_tick) {
  if (!active_ || hold_paused_) return last_pose_;

  t_in_seg_ += dt_tick;
  if (!have_segment_ || t_in_seg_ >= seg_dt_) {
    t_in_seg_ = have_segment_ ? std::max(0.0, t_in_seg_ - seg_dt_) : 0.0;
    stepToNextSegment();
  }
  last_pose_ = sampleAt(std::min(t_in_seg_, seg_dt_));
  return last_pose_;
}

void CartesianChunkFollower::stepToNextSegment() {
  // Re-linearize the orientation tangent onto the state chained from the
  // PREVIOUS solve, and reset the rotation axes to 0 for the new segment. Must
  // happen BEFORE the next solve so this segment's samples use the new R0_ref_.
  if (have_segment_) relinearizeAndReseed();

  seg_dt_ = window_.policyDt();
  core_.setDt(seg_dt_);

  // Consume the budgeted window first; when the next chunk is late, keep
  // consuming into the reserve tail (central diffs clamp at the data edge, so
  // the follower decelerates into the final waypoint) instead of stalling.
  if (window_.active() && window_.hasTailStep()) {
    const std::size_t k = window_.index();
    current_grip_ = window_.gripAt(k);       // gripper phase-locked to consumed step
    // Wrench-gated loading projection: before solving toward this segment's
    // knot, remove the component of the remaining plan advance that presses
    // into a loaded contact (same projection convention as the servo loop's
    // policy-delta path). The removal accumulates as a plan shift applied to
    // every knot in buildSample, so the chained core state stays continuous,
    // tangential motion passes through, and release causes no snap-back.
    diag_.loading_projection_active = false;
    if (loading_dir_valid_) {
      const double r2 = loading_dir_stand_.squaredNorm();
      // Direction-consistency gate: only project when the loading direction
      // agrees with the previous segment's (dot > 0). A real environmental
      // contact pushes in a stable direction; a grasped payload's inertial
      // wrench flips sign with the motion and must never yank the plan.
      const bool consistent = prev_loading_dir_valid_ &&
          loading_dir_stand_.dot(prev_loading_dir_) > 0.0;
      // Quasi-static gate: during fast transit the measured wrench carries a
      // real m*a inertial component the gravity map cannot remove; a real
      // surface press happens at near-zero plan acceleration. Above the bound
      // the projection stands down (baseline follower; hard limits still own
      // impact protection).
      const auto& a0 = core_.a0();
      const double plan_accel =
          std::sqrt(a0[0] * a0[0] + a0[1] * a0[1] + a0[2] * a0[2]);
      const bool quasi_static =
          plan_accel <= cfg_.loading_projection_max_accel_m_s2;
      // A debounced contact_force episode is itself evidence of real contact,
      // so only that episode-scoped ownership may bypass the quasi-static gate.
      // Direction consistency and the per-segment removal clamp remain active.
      if (r2 > 1e-12 && consistent &&
          (quasi_static || contact_normal_owned_)) {
        const auto& p0 = core_.p0();
        const Eigen::Vector3d advance =
            positionOf(window_.poseAt(k)) + contact_shift_ -
            Eigen::Vector3d(p0[0], p0[1], p0[2]);
        const double loading = loading_dir_stand_.dot(advance);
        if (loading > 0.0 ||
            (contact_normal_owned_ && std::abs(loading) > 0.0)) {
          Eigen::Vector3d removal = loading_dir_stand_ * (loading / r2);
          // Bound the per-segment plan pull to one segment of travel at the
          // linear velocity limit: the plan cannot have been advancing faster,
          // so a larger removal only encodes stale lag, not fresh loading.
          const double max_step = cfg_.lin.v_max * seg_dt_;
          const double removal_norm = removal.norm();
          if (removal_norm > max_step && removal_norm > 1e-12) {
            removal *= max_step / removal_norm;
          }
          contact_shift_ -= removal;
          diag_.loading_projection_active = true;
        }
      }
      prev_loading_dir_ = loading_dir_stand_;
      prev_loading_dir_valid_ = r2 > 1e-12;
    } else {
      prev_loading_dir_valid_ = false;
    }
    diag_.contact_shift_m = contact_shift_.norm();
    const BoundarySample<6> sample = buildSample(k);
    diag_.last_solve = core_.solve(sample);
    const auto& realized = core_.p0();
    double pos_sq = 0.0;
    double ang_sq = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double ep = sample.pf[axis] - realized[axis];
      const double er = sample.pf[axis + 3] - realized[axis + 3];
      pos_sq += ep * ep;
      ang_sq += er * er;
    }
    diag_.projection_error_m = std::sqrt(pos_sq);
    diag_.projection_error_rad = std::sqrt(ang_sq);
    const bool projection_violated =
        cfg_.max_projection_error_m > 0.0 && cfg_.max_projection_error_rad > 0.0 &&
        (diag_.projection_error_m > cfg_.max_projection_error_m ||
         diag_.projection_error_rad > cfg_.max_projection_error_rad);
    diag_.consecutive_projection_errors =
        projection_violated ? diag_.consecutive_projection_errors + 1 : 0;
    diag_.infeasible_fault = cfg_.max_consecutive_projection_errors > 0 &&
        diag_.consecutive_projection_errors >= cfg_.max_consecutive_projection_errors;
    diag_.stall = false;
    diag_.seg_target_stand = tangentPose(sample.pf);
    diag_.seg_step_index = static_cast<int>(k);
    diag_.seg_wire_seq = window_.wireSeq();
    diag_.seg_recv_seq = window_.recvSeq();
    window_.advance();
    ++diag_.segments;
  } else {
    // Stall / window exhausted with no fresh chunk: ring velocity down to zero
    // at the current pose (jerk-limited hold), and flag the stall.
    diag_.last_solve = ringDown();
    diag_.stall = true;
    diag_.loading_projection_active = false;
    diag_.contact_shift_m = contact_shift_.norm();
    diag_.projection_error_m = 0.0;
    diag_.projection_error_rad = 0.0;
    diag_.consecutive_projection_errors = 0;
    ++diag_.stall_count;
    diag_.seg_target_stand = tangentPose(core_.p0());
    diag_.seg_step_index = -1;
    diag_.seg_wire_seq = window_.wireSeq();
    diag_.seg_recv_seq = window_.recvSeq();
  }
  have_segment_ = true;
}

BoundarySample<6> CartesianChunkFollower::buildSample(std::size_t k) const {
  const std::size_t n = window_.horizon();
  const Pose6D pm1 = window_.poseAt(flankIndex(k, -1, n));
  const Pose6D pk = window_.poseAt(k);
  const Pose6D pp1 = window_.poseAt(flankIndex(k, +1, n));

  // Per-axis values: 0-2 stand position, 3-5 rotation-vector about R0_ref_.
  const Eigen::Matrix3d Rt = R0_ref_.conjugate().toRotationMatrix();
  auto rotvec = [&](const Pose6D& p) {
    return math::log3(Rt * math::rotationFromPose(p));
  };
  std::array<double, 6> vm1{}, vk{}, vp1{};
  const Eigen::Vector3d posm1 = positionOf(pm1), posk = positionOf(pk), posp1 = positionOf(pp1);
  const Eigen::Vector3d rm1 = rotvec(pm1), rk = rotvec(pk), rp1 = rotvec(pp1);
  for (int d = 0; d < 3; ++d) {
    // Positions carry the accumulated loading-projection shift; a constant
    // shift leaves the finite-difference velocity/accel targets unchanged.
    vm1[d] = posm1[d] + contact_shift_[d];
    vk[d] = posk[d] + contact_shift_[d];
    vp1[d] = posp1[d] + contact_shift_[d];
    vm1[d + 3] = rm1[d]; vk[d + 3] = rk[d]; vp1[d + 3] = rp1[d];
  }

  const double dt = seg_dt_;
  BoundarySample<6> s;
  for (int a = 0; a < 6; ++a) {
    const double d_k = vk[a] - vm1[a];
    const double d_kp1 = vp1[a] - vk[a];
    s.pf[a] = vk[a];
    s.vf[a] = (d_k + d_kp1) / (2 * dt);
    s.af[a] = (d_kp1 - d_k) / (dt * dt);
    const double deadband =
        std::max(0.0, a < 3 ? cfg_.guard.corner_deadband_lin_m
                            : cfg_.guard.corner_deadband_ang_rad);
    s.sign_dk[a] = sgn(d_k, deadband);
    s.sign_dkp1[a] = sgn(d_kp1, deadband);
  }
  return s;
}

SegmentSolve CartesianChunkFollower::ringDown() {
  // Target = current chained pose, zero velocity/accel → jerk-limited stop.
  BoundarySample<6> s;
  const auto& p0 = core_.p0();
  for (int a = 0; a < 6; ++a) {
    s.pf[a] = p0[a];
    s.vf[a] = 0.0;
    s.af[a] = 0.0;
    s.sign_dk[a] = 0;
    s.sign_dkp1[a] = 0;
  }
  return core_.solve(s);
}

void CartesianChunkFollower::relinearizeAndReseed() {
  std::array<double, 6> p0 = core_.p0();
  const Eigen::Vector3d theta(p0[3], p0[4], p0[5]);
  R0_ref_ = (R0_ref_ * Eigen::Quaterniond(math::exp3(theta))).normalized();
  p0[3] = p0[4] = p0[5] = 0.0;  // rotation measured from the new tangent
  core_.seed(p0, core_.v0(), core_.a0());
}

Pose6D CartesianChunkFollower::tangentPose(const std::array<double, 6>& axes) const {
  const Eigen::Quaterniond q =
      (R0_ref_ * Eigen::Quaterniond(math::exp3(Eigen::Vector3d(axes[3], axes[4], axes[5]))))
          .normalized();
  return math::poseFromSe3(
      pinocchio::SE3(q.toRotationMatrix(), Eigen::Vector3d(axes[0], axes[1], axes[2])));
}

Pose6D CartesianChunkFollower::sampleAt(double t) const {
  std::array<double, 6> p{}, v{}, a{};
  if (!core_.sample(t, p, v, a)) return last_pose_;
  const Eigen::Quaterniond q =
      (R0_ref_ * Eigen::Quaterniond(math::exp3(Eigen::Vector3d(p[3], p[4], p[5])))).normalized();
  // Build via SE3 so rx/ry/rz stay in sync with the canonical quaternion
  // (downstream IK prefers the quaternion; telemetry prints rpy).
  Pose6D out = math::poseFromSe3(
      pinocchio::SE3(q.toRotationMatrix(), Eigen::Vector3d(p[0], p[1], p[2])));
  return out;
}

}  // namespace rb_servo::control
