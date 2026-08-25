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

void CartesianChunkFollower::setAdvanceGate(double gate, const Eigen::Vector3d& dir) {
  advance_gate_ = gate < 0.0 ? 0.0 : (gate > 1.0 ? 1.0 : gate);
  into_contact_dir_ = dir;
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
    plan_shift_.setZero();     // fresh anchor: no accumulated hold-back
    have_segment_ = false;     // force a solve on the next tick
    t_in_seg_ = 0.0;
    last_pose_ = current_pose;
    current_grip_ = window_.gripAt(window_.index());
    active_ = true;
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
  // Anchor the integration at the CHAINED PLAN STATE, not at the pose we happen to be
  // emitting right now.
  //
  // ChunkFollowerSegment::solve() advances p0_/v0_/a0_ to the END of the segment it just
  // solved (chunk_follower_core.hpp:160), while last_pose_ = sampleAt(t_in_seg_) is the
  // MID-segment sample. A frame can land at any t_in_seg_, so anchoring at last_pose_ made
  // the first new knot land at  last_pose_ + d, i.e. only
  //     d - d_prev * (1 - t_in_seg_/seg_dt)
  // ahead of where the next solve actually starts (core_.p0()). Submitting early in a
  // segment therefore produced a near-zero — or, when d_prev > d, NEGATIVE — first advance:
  // a stall/reversal at every chunk swap on a perfectly constant-velocity delta stream,
  // with no curvature in the input to blame. Measured on hardware as a ~0.70x displacement
  // notch one row after every swap (outputs/sweep/20260729_0840*_fixedstep_command.jsonl,
  // n~880/row) plus an 11% hold rate on the last executed row, i.e. visible juddering that
  // blurs the wrist cameras the policy is conditioned on.
  //
  // core_.p0() is exactly where the next solve begins, so knot[0] = p0 (+) d is a true
  // one-step advance regardless of submit phase.
  Pose6D cursor = active_ ? tangentPose(core_.p0()) : current_pose;
  for (const Vec6& d : frame.delta) {
    Pose6D delta;
    delta.x = d.x; delta.y = d.y; delta.z = d.z;
    delta.rx = d.rx; delta.ry = d.ry; delta.rz = d.rz;
    cursor = math::composeDeltaLocal(cursor, delta);
    integrated.pose.push_back(cursor);
  }
  // The new knots were integrated from the EMITTED pose, which already sits where
  // the gate left it, so the hold-back is baked into the fresh plan: clear the debt.
  plan_shift_.setZero();
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

std::optional<Vec6> CartesianChunkFollower::currentVelocity() const {
  if (!active_ || hold_paused_ || !have_segment_ || !core_.hasState()) {
    return std::nullopt;
  }
  const auto& v = core_.v0();
  return Vec6{v[0], v[1], v[2], v[3], v[4], v[5]};
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
    // THE GATE, AT THE SEGMENT BOUNDARY. This is our input-period equivalent of the
    // controller-manager follow path's per-delta gate: one knot is consumed per
    // segment, so this is where an advance exists to attenuate.
    if (advance_gate_ < 1.0 && into_contact_dir_.squaredNorm() > 1e-18) {
      const auto& p0 = core_.p0();
      const Eigen::Vector3d advance = positionOf(window_.poseAt(k)) + plan_shift_ -
                                      Eigen::Vector3d(p0[0], p0[1], p0[2]);
      const double proj = advance.dot(into_contact_dir_);
      // proj < 0 means the advance drives AGAINST the reported force, i.e. deeper
      // into the contact. Tangential and retreating advances are left alone.
      if (proj < 0.0) {
        Eigen::Vector3d removal = (1.0 - advance_gate_) * proj * into_contact_dir_;
        // Bound the per-segment hold-back to one segment of travel at the linear
        // velocity limit: the plan cannot have been advancing faster than that, so a
        // larger removal encodes stale lag rather than fresh contact.
        const double max_step = cfg_.lin.v_max * seg_dt_;
        const double removal_norm = removal.norm();
        if (removal_norm > max_step && removal_norm > 1e-12) {
          removal *= max_step / removal_norm;
        }
        plan_shift_ -= removal;
      }
    }
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
    // Positions carry the accumulated gate hold-back. A CONSTANT shift leaves the
    // finite-difference velocity/accel targets unchanged, so holding the plan back
    // does not also distort the dynamics it is solved against.
    vm1[d] = posm1[d] + plan_shift_[d];
    vk[d] = posk[d] + plan_shift_[d];
    vp1[d] = posp1[d] + plan_shift_[d];
    vm1[d + 3] = rm1[d]; vk[d + 3] = rk[d]; vp1[d + 3] = rp1[d];
  }

  const double dt = seg_dt_;
  // At the window HEAD there is no backward neighbour: flankIndex(k,-1,n) clamps to k itself, so a
  // naive d_k would be 0 -- which halves the central-difference vf, injects a full-magnitude spurious
  // af (+d_kp1/dt^2), and zeroes sign_dk so the corner test can never fire. ChunkWindow::activate()
  // rewinds the cursor to the head on EVERY submitted frame, so that notch landed on row 0 of every
  // chunk, i.e. at the replan rate. Measured on logs/servo_log_20260724_160210.csv, emitted follower
  // speed by consumed row index (right arm): 20.4 / 27.6 / 42.4 / 47.9 / 45.9 mm/s -- row0 was 0.43x
  // the steady state, and with CHUNK_EXECUTE_STEPS=5 three of five executed rows were still ramping
  // (~23% average speed loss plus a periodic lurch). Fall back to the plain FORWARD difference and
  // claim no curvature; sign_dk then matches sign_dkp1, so no phantom corner either.
  // The TAIL clamp (d_kp1 == 0 at k == n-1) is deliberately left as-is: decelerating into the last
  // published knot is the desired ring-down behaviour when the window runs dry.
  const bool has_prev = k > 0;
  BoundarySample<6> s;
  for (int a = 0; a < 6; ++a) {
    const double d_kp1 = vp1[a] - vk[a];
    const double d_k = has_prev ? (vk[a] - vm1[a]) : d_kp1;
    s.pf[a] = vk[a];
    s.vf[a] = (d_k + d_kp1) / (2 * dt);
    s.af[a] = has_prev ? (d_kp1 - d_k) / (dt * dt) : 0.0;
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
