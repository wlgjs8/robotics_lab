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

int sgn(double x) { return (x > 1e-12) - (x < -1e-12); }

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
  have_segment_ = false;
  window_.deactivate();
}

void CartesianChunkFollower::reanchor(const Pose6D& reference) {
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
  }
  // else: preempt — window frame swapped, chained core state kept. The next
  // boundary builds toward the new frame's poses from the current state.
}

Pose6D CartesianChunkFollower::tick(double dt_tick) {
  if (!active_) return last_pose_;

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
    const BoundarySample<6> sample = buildSample(k);
    diag_.last_solve = core_.solve(sample);
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
    vm1[d] = posm1[d]; vk[d] = posk[d]; vp1[d] = posp1[d];
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
    s.sign_dk[a] = sgn(d_k);
    s.sign_dkp1[a] = sgn(d_kp1);
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
