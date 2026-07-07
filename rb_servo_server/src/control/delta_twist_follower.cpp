#include "rb_servo/control/delta_twist_follower.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace rb_servo::control {
namespace {

constexpr double kDefaultPolicyDt = 1.0 / 30.0;

double clampAbs(double value, double limit) {
  if (!std::isfinite(value)) return 0.0;
  const double lim = std::max(0.0, limit);
  return std::clamp(value, -lim, lim);
}

double linearNorm(const Vec6& v) {
  const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  return std::isfinite(n) ? n : std::numeric_limits<double>::infinity();
}

double angularNorm(const Vec6& v) {
  const double n = std::sqrt(v.rx * v.rx + v.ry * v.ry + v.rz * v.rz);
  return std::isfinite(n) ? n : std::numeric_limits<double>::infinity();
}

bool scaleLinearTo(Vec6* v, double max_norm) {
  if (!v) return false;
  const double n = linearNorm(*v);
  if (n <= max_norm || max_norm <= 0.0) return false;
  const double s = max_norm / n;
  v->x *= s;
  v->y *= s;
  v->z *= s;
  return true;
}

bool scaleAngularTo(Vec6* v, double max_norm) {
  if (!v) return false;
  const double n = angularNorm(*v);
  if (n <= max_norm || max_norm <= 0.0) return false;
  const double s = max_norm / n;
  v->rx *= s;
  v->ry *= s;
  v->rz *= s;
  return true;
}

bool hasLinearOrAngularNorm(const Vec6& v, double eps = 1e-9) {
  return linearNorm(v) > eps || angularNorm(v) > eps;
}

double robustRatio(double numerator, double denominator) {
  constexpr double kEps = 1e-9;
  if (!std::isfinite(numerator) || !std::isfinite(denominator)) return 0.0;
  if (std::abs(denominator) < kEps) {
    return std::abs(numerator) < kEps ? 1.0 : 0.0;
  }
  const double ratio = numerator / denominator;
  return std::isfinite(ratio) ? ratio : 0.0;
}

bool clampAxes(Vec6* v, const AxisLimit& lin, const AxisLimit& ang, double AxisLimit::* member) {
  if (!v) return false;
  const double lin_lim = lin.*member;
  const double ang_lim = ang.*member;
  const Vec6 before = *v;
  v->x = clampAbs(v->x, lin_lim);
  v->y = clampAbs(v->y, lin_lim);
  v->z = clampAbs(v->z, lin_lim);
  v->rx = clampAbs(v->rx, ang_lim);
  v->ry = clampAbs(v->ry, ang_lim);
  v->rz = clampAbs(v->rz, ang_lim);
  return before.x != v->x || before.y != v->y || before.z != v->z ||
         before.rx != v->rx || before.ry != v->ry || before.rz != v->rz;
}

Vec6 add(const Vec6& a, const Vec6& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z, a.rx + b.rx, a.ry + b.ry, a.rz + b.rz};
}

Vec6 sub(const Vec6& a, const Vec6& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z, a.rx - b.rx, a.ry - b.ry, a.rz - b.rz};
}

Vec6 mul(const Vec6& a, double s) {
  return {a.x * s, a.y * s, a.z * s, a.rx * s, a.ry * s, a.rz * s};
}

Pose6D deltaPose(const Vec6& delta) {
  Pose6D p;
  p.x = delta.x;
  p.y = delta.y;
  p.z = delta.z;
  p.rx = delta.rx;
  p.ry = delta.ry;
  p.rz = delta.rz;
  return p;
}

bool finiteDelta(const Vec6& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
         std::isfinite(v.rx) && std::isfinite(v.ry) && std::isfinite(v.rz);
}

bool finitePose(const Pose6D& pose) {
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z)) {
    return false;
  }
  try {
    const auto R = math::rotationFromPose(pose);
    return R.allFinite();
  } catch (...) {
    return false;
  }
}

Vec6 localDelta(const Pose6D& prev, const Pose6D& cur) {
  const auto R_prev = math::rotationFromPose(prev);
  const auto R_cur = math::rotationFromPose(cur);
  const math::Vector3 dp(cur.x - prev.x, cur.y - prev.y, cur.z - prev.z);
  const math::Vector3 pos_local = R_prev.transpose() * dp;
  const math::Vector3 rot_local = math::log3(R_prev.transpose() * R_cur);
  return {pos_local.x(), pos_local.y(), pos_local.z(),
          rot_local.x(), rot_local.y(), rot_local.z()};
}

bool feedbackDeltaPlausible(const Vec6& delta, const DeltaTwistFollowerConfig& cfg) {
  if (!finiteDelta(delta)) return false;
  const double lin_limit = std::max({3.0 * cfg.max_residual_m, 3.0 * cfg.max_lead_m, 0.25});
  const double ang_limit = std::max({3.0 * cfg.max_residual_rad, 3.0 * cfg.max_lead_rad, 1.0});
  return linearNorm(delta) <= lin_limit && angularNorm(delta) <= ang_limit;
}

}  // namespace

DeltaTwistFollower::DeltaTwistFollower(const DeltaTwistFollowerConfig& cfg)
    : cfg_(cfg) {}

void DeltaTwistFollower::reconfigure(const DeltaTwistFollowerConfig& cfg) {
  cfg_ = cfg;
  deactivate();
}

void DeltaTwistFollower::deactivate() {
  active_ = false;
  delta_.clear();
  grip_.clear();
  index_ = 0;
  consumed_ = 0;
  normal_eff_ = 0;
  total_eff_ = 0;
  stale_residual_age_ = 0.0;
  t_in_seg_ = 0.0;
  pending_delta_ = Vec6{};
  step_delta_ = Vec6{};
  realized_delta_ = Vec6{};
  xi_ref_ = Vec6{};
  xi_cmd_ = Vec6{};
  accel_cmd_ = Vec6{};
  lead_delta_ = Vec6{};
  has_feedback_pose_ = false;
  diag_ = DeltaTwistFollowerDiag{};
}

bool DeltaTwistFollower::hasConsumableStep() const {
  return active_ && consumed_ < total_eff_ && index_ < delta_.size();
}

double DeltaTwistFollower::gripAt(std::size_t k) const {
  if (grip_.empty()) return current_grip_;
  return grip_[std::min(k, grip_.size() - 1)];
}

void DeltaTwistFollower::submitFrame(const ChunkFrame& frame, const Pose6D& current_pose) {
  if (frame.delta.empty()) {
    deactivate();
    return;
  }

  policy_dt_ = frame.policy_dt > 1e-6 ? frame.policy_dt : kDefaultPolicyDt;
  recv_time_ = frame.recv_time;
  wire_seq_ = frame.wire_seq;
  recv_seq_ = frame.recv_seq;
  delta_ = frame.delta;
  grip_ = frame.grip;

  const int L = std::max(0, cfg_.discard_head_steps);
  const int n = static_cast<int>(delta_.size());
  if (L >= n || cfg_.consume_steps < 1) {
    deactivate();
    return;
  }
  index_ = static_cast<std::size_t>(L);
  consumed_ = 0;
  normal_eff_ = std::min(std::max(0, cfg_.consume_steps), n - L);
  total_eff_ = std::min(
      std::max(0, cfg_.consume_steps) + std::max(0, cfg_.reserve_steps),
      n - L);
  if (normal_eff_ < 1 || total_eff_ < 1) {
    deactivate();
    return;
  }
  t_in_seg_ = 0.0;
  stale_residual_age_ = 0.0;
  clampPendingResidual();

  if (!active_) {
    command_pose_ = current_pose;
    last_pose_ = current_pose;
    last_feedback_pose_ = current_pose;
    has_feedback_pose_ = false;
    xi_cmd_ = Vec6{};
    accel_cmd_ = Vec6{};
    pending_delta_ = Vec6{};
    step_delta_ = Vec6{};
    realized_delta_ = Vec6{};
    xi_ref_ = Vec6{};
    lead_delta_ = Vec6{};
    current_grip_ = gripAt(index_);
    diag_ = DeltaTwistFollowerDiag{};
    active_ = true;
  }

  // A delta row is a per-frame local displacement. Consume the first available
  // row immediately so the first servo tick starts moving toward the model's
  // current action instead of waiting one policy period.
  consumeNextStep();
  updateDiagState();
}

void DeltaTwistFollower::reanchor(const Pose6D& reference) {
  command_pose_ = reference;
  last_pose_ = reference;
  last_feedback_pose_ = reference;
  has_feedback_pose_ = false;
  xi_cmd_ = Vec6{};
  accel_cmd_ = Vec6{};
  pending_delta_ = Vec6{};
  step_delta_ = Vec6{};
  realized_delta_ = Vec6{};
  xi_ref_ = Vec6{};
  lead_delta_ = Vec6{};
  t_in_seg_ = 0.0;
  stale_residual_age_ = 0.0;
  diag_.seg_target_stand = reference;
  updateDiagState();
  if (total_eff_ > 0) active_ = true;
}

void DeltaTwistFollower::setFeedbackPose(const Pose6D& feedback_pose) {
  if (!finitePose(feedback_pose)) {
    return;
  }
  if (!has_feedback_pose_) {
    last_feedback_pose_ = feedback_pose;
    has_feedback_pose_ = true;
    return;
  }

  Vec6 realized{};
  try {
    realized = localDelta(last_feedback_pose_, feedback_pose);
  } catch (...) {
    last_feedback_pose_ = feedback_pose;
    return;
  }
  last_feedback_pose_ = feedback_pose;
  if (!feedbackDeltaPlausible(realized, cfg_)) {
    return;
  }
  realized_delta_ = add(realized_delta_, realized);
  pending_delta_ = sub(pending_delta_, realized);
  clampPendingResidual();
  updateDiagState();
}

bool DeltaTwistFollower::consumeNextStep() {
  if (!hasConsumableStep()) {
    diag_.stall = true;
    ++diag_.stall_count;
    diag_.seg_step_index = -1;
    diag_.step_phase = hasLinearOrAngularNorm(pending_delta_) ? DeltaTwistStepPhase::ResidualDrain
                                                             : DeltaTwistStepPhase::Ringdown;
    return false;
  }

  const Vec6 step = finiteDelta(delta_[index_]) ? delta_[index_] : Vec6{};
  step_delta_ = step;
  realized_delta_ = Vec6{};
  pending_delta_ = add(pending_delta_, step);
  clampPendingResidual();
  current_grip_ = gripAt(index_);
  diag_.stall = false;
  diag_.seg_step_index = static_cast<int>(index_);
  diag_.seg_wire_seq = wire_seq_;
  diag_.seg_recv_seq = recv_seq_;
  diag_.step_phase = consumed_ < normal_eff_ ? DeltaTwistStepPhase::Normal
                                             : DeltaTwistStepPhase::Reserve;
  ++index_;
  ++consumed_;
  ++diag_.segments;
  stale_residual_age_ = 0.0;
  return true;
}

void DeltaTwistFollower::clampPendingResidual() {
  (void)scaleLinearTo(&pending_delta_, cfg_.max_residual_m);
  (void)scaleAngularTo(&pending_delta_, cfg_.max_residual_rad);
}

void DeltaTwistFollower::updateDiagState() {
  diag_.pending_delta = pending_delta_;
  diag_.step_delta = step_delta_;
  diag_.realized_delta = realized_delta_;
  diag_.xi_ref = xi_ref_;
  diag_.xi_cmd = xi_cmd_;
  diag_.accel_cmd = accel_cmd_;
  diag_.lead_delta = lead_delta_;
  diag_.realized_linear_ratio = robustRatio(linearNorm(realized_delta_), linearNorm(step_delta_));
  diag_.realized_angular_ratio = robustRatio(angularNorm(realized_delta_), angularNorm(step_delta_));
  diag_.realized_yaw_ratio = robustRatio(realized_delta_.rz, step_delta_.rz);
  diag_.normal_consumed = std::min(consumed_, normal_eff_);
  diag_.reserve_consumed = std::max(0, consumed_ - normal_eff_);
  diag_.seg_target_stand = command_pose_;
}

void DeltaTwistFollower::clampCommandLead() {
  if (!has_feedback_pose_) {
    lead_delta_ = Vec6{};
    return;
  }
  Vec6 lead{};
  try {
    lead = localDelta(last_feedback_pose_, command_pose_);
  } catch (...) {
    lead_delta_ = Vec6{};
    return;
  }
  bool clamped = false;
  clamped = scaleLinearTo(&lead, cfg_.max_lead_m) || clamped;
  clamped = scaleAngularTo(&lead, cfg_.max_lead_rad) || clamped;
  lead_delta_ = lead;
  if (clamped) {
    command_pose_ = math::composeDeltaLocal(last_feedback_pose_, deltaPose(lead));
  }
}

Pose6D DeltaTwistFollower::tick(double dt_tick) {
  if (!active_) return last_pose_;
  if (!std::isfinite(dt_tick) || dt_tick <= 0.0) return last_pose_;

  t_in_seg_ += dt_tick;
  bool consumed_step = false;
  while (t_in_seg_ >= policy_dt_) {
    t_in_seg_ -= policy_dt_;
    consumed_step = consumeNextStep() || consumed_step;
  }
  if (!consumed_step && diag_.stall) {
    stale_residual_age_ += dt_tick;
    if (cfg_.stale_residual_timeout_sec > 0.0 &&
        stale_residual_age_ > cfg_.stale_residual_timeout_sec) {
      pending_delta_ = Vec6{};
    }
  }

  clampPendingResidual();
  const double remaining = std::max(policy_dt_ - t_in_seg_, dt_tick);
  const int extra_steps = std::max(0, cfg_.residual_drain_steps - 1);
  const double horizon = std::max(remaining + static_cast<double>(extra_steps) * policy_dt_, dt_tick);
  Vec6 xi_ref = mul(pending_delta_, 1.0 / horizon);
  bool saturated = clampAxes(&xi_ref, cfg_.lin, cfg_.ang, &AxisLimit::v_max);
  xi_ref_ = xi_ref;

  Vec6 a_des = mul(sub(xi_ref, xi_cmd_), 1.0 / std::max(cfg_.tau_sec, dt_tick));
  saturated = clampAxes(&a_des, cfg_.lin, cfg_.ang, &AxisLimit::a_max) || saturated;

  Vec6 j_des = mul(sub(a_des, accel_cmd_), 1.0 / dt_tick);
  saturated = clampAxes(&j_des, cfg_.lin, cfg_.ang, &AxisLimit::j_max) || saturated;

  accel_cmd_ = add(accel_cmd_, mul(j_des, dt_tick));
  saturated = clampAxes(&accel_cmd_, cfg_.lin, cfg_.ang, &AxisLimit::a_max) || saturated;
  xi_cmd_ = add(xi_cmd_, mul(accel_cmd_, dt_tick));
  saturated = clampAxes(&xi_cmd_, cfg_.lin, cfg_.ang, &AxisLimit::v_max) || saturated;

  const Vec6 delta_exec = mul(xi_cmd_, dt_tick);
  command_pose_ = math::composeDeltaLocal(command_pose_, deltaPose(delta_exec));
  clampCommandLead();
  last_pose_ = command_pose_;

  clampPendingResidual();
  if (diag_.stall) {
    diag_.step_phase = hasLinearOrAngularNorm(pending_delta_) ? DeltaTwistStepPhase::ResidualDrain
                                                             : DeltaTwistStepPhase::Ringdown;
  }

  diag_.saturated = saturated;
  diag_.last_solve.result = ruckig::Result::Working;
  diag_.last_solve.duration = policy_dt_;
  diag_.last_solve.alpha = 1.0;
  diag_.last_solve.converged = !saturated;
  diag_.last_solve.corner = false;
  updateDiagState();
  return last_pose_;
}

}  // namespace rb_servo::control
