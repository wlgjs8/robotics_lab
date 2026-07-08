#include "rb_servo/control/delta_twist_follower.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace rb_servo::control {
namespace {

constexpr double kDefaultPolicyDt = 1.0 / 30.0;
constexpr double kFeedbackCosDefault = 1.0;

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

bool closeSoonForGrasp(
    const GraspCommitConfig& cfg,
    const std::vector<double>& future_grip_window
) {
  if (!cfg.enable || cfg.close_lookahead_steps <= 0 || future_grip_window.empty()) {
    return false;
  }
  const std::size_t n = std::min(
      static_cast<std::size_t>(cfg.close_lookahead_steps),
      future_grip_window.size()
  );
  bool found = false;
  double extreme = cfg.close_is_greater
      ? -std::numeric_limits<double>::infinity()
      : std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    const double value = future_grip_window[i];
    if (!std::isfinite(value)) continue;
    found = true;
    if (cfg.close_is_greater) {
      extreme = std::max(extreme, value);
    } else {
      extreme = std::min(extreme, value);
    }
  }
  if (!found) return false;
  return cfg.close_is_greater ? extreme >= cfg.close_threshold
                              : extreme <= cfg.close_threshold;
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

bool clampLinearNorm(Vec6* v, double max_norm) {
  if (!v) return false;
  const double n = linearNorm(*v);
  if (!std::isfinite(n) || max_norm <= 0.0) {
    const bool changed = v->x != 0.0 || v->y != 0.0 || v->z != 0.0;
    v->x = 0.0;
    v->y = 0.0;
    v->z = 0.0;
    return changed;
  }
  if (n <= max_norm) return false;
  const double s = max_norm / n;
  v->x *= s;
  v->y *= s;
  v->z *= s;
  return true;
}

bool clampAngularNorm(Vec6* v, double max_norm) {
  if (!v) return false;
  const double n = angularNorm(*v);
  if (!std::isfinite(n) || max_norm <= 0.0) {
    const bool changed = v->rx != 0.0 || v->ry != 0.0 || v->rz != 0.0;
    v->rx = 0.0;
    v->ry = 0.0;
    v->rz = 0.0;
    return changed;
  }
  if (n <= max_norm) return false;
  const double s = max_norm / n;
  v->rx *= s;
  v->ry *= s;
  v->rz *= s;
  return true;
}

bool clampVec6Norm(Vec6* v, const AxisLimit& lin, const AxisLimit& ang, double AxisLimit::* member) {
  if (!v) return false;
  bool clamped = false;
  clamped = clampLinearNorm(v, lin.*member) || clamped;
  clamped = clampAngularNorm(v, ang.*member) || clamped;
  return clamped;
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

double smoothstep01(double value) {
  const double t = std::clamp(value, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
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

double linearDot(const Vec6& a, const Vec6& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

double angularDot(const Vec6& a, const Vec6& b) {
  return a.rx * b.rx + a.ry * b.ry + a.rz * b.rz;
}

void subtractProjectedLinear(Vec6* pending, const Vec6& realized, double* cos_out) {
  constexpr double kEps = 1e-9;
  if (!pending) return;
  if (cos_out) *cos_out = kFeedbackCosDefault;
  const double p_norm = linearNorm(*pending);
  const double r_norm = linearNorm(realized);
  if (p_norm <= kEps || r_norm <= kEps || !std::isfinite(p_norm) || !std::isfinite(r_norm)) {
    return;
  }
  const double dot = linearDot(*pending, realized);
  if (cos_out) *cos_out = std::clamp(dot / (p_norm * r_norm), -1.0, 1.0);
  const double projected_amount = std::clamp(dot / p_norm, 0.0, p_norm);
  pending->x -= pending->x / p_norm * projected_amount;
  pending->y -= pending->y / p_norm * projected_amount;
  pending->z -= pending->z / p_norm * projected_amount;
}

void subtractProjectedAngular(Vec6* pending, const Vec6& realized, double* cos_out) {
  constexpr double kEps = 1e-9;
  if (!pending) return;
  if (cos_out) *cos_out = kFeedbackCosDefault;
  const double p_norm = angularNorm(*pending);
  const double r_norm = angularNorm(realized);
  if (p_norm <= kEps || r_norm <= kEps || !std::isfinite(p_norm) || !std::isfinite(r_norm)) {
    return;
  }
  const double dot = angularDot(*pending, realized);
  if (cos_out) *cos_out = std::clamp(dot / (p_norm * r_norm), -1.0, 1.0);
  const double projected_amount = std::clamp(dot / p_norm, 0.0, p_norm);
  pending->rx -= pending->rx / p_norm * projected_amount;
  pending->ry -= pending->ry / p_norm * projected_amount;
  pending->rz -= pending->rz / p_norm * projected_amount;
}

}  // namespace

DeltaTwistFollower::DeltaTwistFollower(const DeltaTwistFollowerConfig& cfg)
    : cfg_(cfg),
      surface_projector_(cfg.surface_action_projector) {}

void DeltaTwistFollower::reconfigure(const DeltaTwistFollowerConfig& cfg) {
  cfg_ = cfg;
  surface_projector_.reconfigure(cfg.surface_action_projector);
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
  projected_step_delta_ = Vec6{};
  realized_delta_ = Vec6{};
  xi_ref_ = Vec6{};
  xi_cmd_ = Vec6{};
  accel_cmd_ = Vec6{};
  lead_delta_ = Vec6{};
  feedback_source_ = static_cast<int>(DeltaTwistFeedbackSource::None);
  pending_clamped_ = false;
  residual_cleared_on_frame_ = false;
  min_time_to_go_used_ = false;
  lin_feedback_cos_ = kFeedbackCosDefault;
  ang_feedback_cos_ = kFeedbackCosDefault;
  xi_ref_clamped_norm_ = false;
  xi_cmd_clamped_norm_ = false;
  blocked_ = false;
  block_reason_mask_ = 0;
  block_elapsed_sec_ = 0.0;
  block_requires_fresh_chunk_ = false;
  residual_cleared_by_block_ = false;
  step_consumed_this_tick_ = false;
  has_feedback_pose_ = false;
  has_projection_reference_ = false;
  grasp_commit_ = GraspCommitArmCommand{};
  previous_grasp_phase_ = GraspPhase::Normal;
  has_grasp_lift_start_ = false;
  surface_projection_ = SurfaceProjectionResult{};
  diag_ = DeltaTwistFollowerDiag{};
}

bool DeltaTwistFollower::hasConsumableStep() const {
  return active_ && consumed_ < total_eff_ && index_ < delta_.size();
}

double DeltaTwistFollower::gripAt(std::size_t k) const {
  if (grip_.empty()) return current_grip_;
  return grip_[std::min(k, grip_.size() - 1)];
}

std::vector<double> DeltaTwistFollower::futureGripWindow(std::size_t k) const {
  std::vector<double> window;
  const int lookahead = std::max(
      cfg_.surface_action_projector.close_lookahead_steps,
      cfg_.grasp_commit.close_lookahead_steps
  );
  if (lookahead <= 0 || grip_.empty()) return window;
  const std::size_t end = std::min(grip_.size(), k + static_cast<std::size_t>(lookahead));
  window.reserve(end > k ? end - k : 0);
  for (std::size_t i = k; i < end; ++i) {
    window.push_back(gripAt(i));
  }
  return window;
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
  residual_cleared_on_frame_ = false;
  residual_cleared_by_block_ = false;
  block_requires_fresh_chunk_ = false;
  step_consumed_this_tick_ = false;
  if (!blocked_) {
    block_elapsed_sec_ = 0.0;
    block_reason_mask_ = 0;
  }
  pending_clamped_ = false;
  if (finitePose(current_pose)) {
    projection_reference_pose_ = current_pose;
    has_projection_reference_ = true;
  }
  if (cfg_.clear_residual_on_new_frame) {
    // OpenPI chunks are closed-loop replans from the current observation. Carrying
    // residual from an older chunk double-counts stale action and causes catch-up
    // arcs; keep xi_cmd_/accel_cmd_ for velocity continuity, but clear backlog.
    pending_delta_ = Vec6{};
    realized_delta_ = Vec6{};
    step_delta_ = Vec6{};
    projected_step_delta_ = Vec6{};
    surface_projection_ = SurfaceProjectionResult{};
    stale_residual_age_ = 0.0;
    residual_cleared_on_frame_ = true;
  } else {
    pending_clamped_ = clampPendingResidual() || pending_clamped_;
  }

  if (!active_) {
    command_pose_ = current_pose;
    last_pose_ = current_pose;
    last_feedback_pose_ = current_pose;
    projection_reference_pose_ = current_pose;
    has_feedback_pose_ = false;
    has_projection_reference_ = finitePose(current_pose);
    xi_cmd_ = Vec6{};
    accel_cmd_ = Vec6{};
    pending_delta_ = Vec6{};
    step_delta_ = Vec6{};
    projected_step_delta_ = Vec6{};
    realized_delta_ = Vec6{};
    xi_ref_ = Vec6{};
    lead_delta_ = Vec6{};
    feedback_source_ = static_cast<int>(DeltaTwistFeedbackSource::None);
    min_time_to_go_used_ = false;
    lin_feedback_cos_ = kFeedbackCosDefault;
    ang_feedback_cos_ = kFeedbackCosDefault;
    xi_ref_clamped_norm_ = false;
    xi_cmd_clamped_norm_ = false;
    current_grip_ = gripAt(index_);
    surface_projection_ = SurfaceProjectionResult{};
    diag_ = DeltaTwistFollowerDiag{};
    active_ = true;
  }

  // A delta row is a per-frame local displacement. Consume the first available
  // row immediately so the first servo tick starts moving toward the model's
  // current action instead of waiting one policy period.
  if (blocked_) {
    updateDiagState();
    return;
  }
  step_consumed_this_tick_ = consumeNextStep();
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
  projected_step_delta_ = Vec6{};
  realized_delta_ = Vec6{};
  xi_ref_ = Vec6{};
  lead_delta_ = Vec6{};
  t_in_seg_ = 0.0;
  stale_residual_age_ = 0.0;
  feedback_source_ = static_cast<int>(DeltaTwistFeedbackSource::None);
  projection_reference_pose_ = reference;
  has_projection_reference_ = finitePose(reference);
  pending_clamped_ = false;
  residual_cleared_on_frame_ = true;
  min_time_to_go_used_ = false;
  lin_feedback_cos_ = kFeedbackCosDefault;
  ang_feedback_cos_ = kFeedbackCosDefault;
  xi_ref_clamped_norm_ = false;
  xi_cmd_clamped_norm_ = false;
  blocked_ = false;
  block_reason_mask_ = 0;
  block_elapsed_sec_ = 0.0;
  block_requires_fresh_chunk_ = false;
  residual_cleared_by_block_ = false;
  step_consumed_this_tick_ = false;
  surface_projection_ = SurfaceProjectionResult{};
  grasp_commit_ = GraspCommitArmCommand{};
  previous_grasp_phase_ = GraspPhase::Normal;
  has_grasp_lift_start_ = false;
  diag_.seg_target_stand = reference;
  updateDiagState();
  if (total_eff_ > 0) active_ = true;
}

void DeltaTwistFollower::clearResidualForCommit() {
  clearPendingResidual();
}

void DeltaTwistFollower::clearPendingResidual() {
  pending_delta_ = Vec6{};
  stale_residual_age_ = 0.0;
  pending_clamped_ = false;
  residual_cleared_on_frame_ = true;
}

void DeltaTwistFollower::setBlocked(bool blocked, int reason_mask) {
  blocked_ = blocked;
  block_reason_mask_ = blocked ? reason_mask : 0;
  if (!blocked && !block_requires_fresh_chunk_) {
    block_elapsed_sec_ = 0.0;
    residual_cleared_by_block_ = false;
  }
  if (blocked) {
    step_consumed_this_tick_ = false;
  }
  updateDiagState();
}

void DeltaTwistFollower::ringDown(double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) return;
  xi_ref_ = Vec6{};
  xi_ref_clamped_norm_ = false;
  xi_cmd_clamped_norm_ = false;
  bool saturated = false;

  Vec6 a_des = mul(sub(Vec6{}, xi_cmd_), 1.0 / std::max(cfg_.tau_sec, dt));
  saturated = clampVec6Norm(&a_des, cfg_.lin, cfg_.ang, &AxisLimit::a_max) || saturated;
  Vec6 j_des = mul(sub(a_des, accel_cmd_), 1.0 / dt);
  saturated = clampVec6Norm(&j_des, cfg_.lin, cfg_.ang, &AxisLimit::j_max) || saturated;
  accel_cmd_ = add(accel_cmd_, mul(j_des, dt));
  saturated = clampVec6Norm(&accel_cmd_, cfg_.lin, cfg_.ang, &AxisLimit::a_max) || saturated;
  xi_cmd_ = add(xi_cmd_, mul(accel_cmd_, dt));
  xi_cmd_clamped_norm_ = clampVec6Norm(&xi_cmd_, cfg_.lin, cfg_.ang, &AxisLimit::v_max);
  saturated = xi_cmd_clamped_norm_ || saturated;

  if (linearNorm(xi_cmd_) < 1e-8 && angularNorm(xi_cmd_) < 1e-8) {
    xi_cmd_ = Vec6{};
    accel_cmd_ = Vec6{};
  }
  diag_.saturated = saturated;
}

void DeltaTwistFollower::pauseBlocked(const Pose6D& feedback_or_reference_pose, double dt) {
  if (!std::isfinite(dt) || dt < 0.0) dt = 0.0;
  blocked_ = true;
  step_consumed_this_tick_ = false;
  block_elapsed_sec_ += dt;
  if (cfg_.block_requires_fresh_chunk_sec > 0.0 &&
      block_elapsed_sec_ >= cfg_.block_requires_fresh_chunk_sec) {
    block_requires_fresh_chunk_ = true;
  }
  if (cfg_.block_clear_residual) {
    const bool had_residual = hasLinearOrAngularNorm(pending_delta_);
    clearPendingResidual();
    residual_cleared_by_block_ = residual_cleared_by_block_ || had_residual;
  }
  if (finitePose(feedback_or_reference_pose)) {
    last_feedback_pose_ = feedback_or_reference_pose;
    projection_reference_pose_ = feedback_or_reference_pose;
    command_pose_ = feedback_or_reference_pose;
    last_pose_ = feedback_or_reference_pose;
    has_feedback_pose_ = true;
    has_projection_reference_ = true;
  }
  ringDown(dt);
  diag_.stall = true;
  diag_.step_phase = DeltaTwistStepPhase::Ringdown;
  diag_.last_solve.result = ruckig::Result::Working;
  diag_.last_solve.duration = policy_dt_;
  diag_.last_solve.alpha = 1.0;
  diag_.last_solve.converged = !diag_.saturated;
  diag_.last_solve.corner = false;
  updateDiagState();
}

Pose6D DeltaTwistFollower::liftPoseFromProgress(double progress) const {
  Pose6D pose = grasp_lift_start_pose_;
  pose.z += grasp_commit_.lift_height_m * smoothstep01(progress);
  return pose;
}

void DeltaTwistFollower::setGraspCommitCommand(const GraspCommitArmCommand& command) {
  grasp_commit_ = command;
  if (grasp_commit_.clear_residual) {
    clearResidualForCommit();
  }
  if (grasp_commit_.gripper_override_active) {
    current_grip_ = grasp_commit_.gripper_target;
  }

  const bool entering_hold =
      (previous_grasp_phase_ != GraspPhase::ClosingHold &&
       grasp_commit_.phase == GraspPhase::ClosingHold) ||
      (previous_grasp_phase_ != GraspPhase::ResumeWaitFreshChunk &&
       grasp_commit_.phase == GraspPhase::ResumeWaitFreshChunk);
  if (entering_hold && has_feedback_pose_) {
    command_pose_ = last_feedback_pose_;
    last_pose_ = command_pose_;
  }

  if (previous_grasp_phase_ != GraspPhase::LiftOut &&
      grasp_commit_.phase == GraspPhase::LiftOut) {
    grasp_lift_start_pose_ = has_feedback_pose_ ? last_feedback_pose_ : command_pose_;
    has_grasp_lift_start_ = true;
  } else if (grasp_commit_.phase != GraspPhase::LiftOut) {
    has_grasp_lift_start_ = false;
  }
  previous_grasp_phase_ = grasp_commit_.phase;
  updateDiagState();
}

void DeltaTwistFollower::setFeedbackPose(
    const Pose6D& feedback_pose,
    DeltaTwistFeedbackSource source
) {
  if (!finitePose(feedback_pose)) {
    return;
  }
  projection_reference_pose_ = feedback_pose;
  has_projection_reference_ = true;
  feedback_source_ = static_cast<int>(source);
  lin_feedback_cos_ = kFeedbackCosDefault;
  ang_feedback_cos_ = kFeedbackCosDefault;
  if (!has_feedback_pose_) {
    last_feedback_pose_ = feedback_pose;
    has_feedback_pose_ = true;
    return;
  }
  if (blocked_ || block_requires_fresh_chunk_) {
    last_feedback_pose_ = feedback_pose;
    has_feedback_pose_ = true;
    updateDiagState();
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
  // Residual anti-windup is based on execution feedback. If feedback moves
  // opposite to the pending local action, blind subtraction grows residual and
  // drives catch-up orbits, so only subtract the realized projection along the
  // current pending direction while keeping raw realized_delta_ for telemetry.
  subtractProjectedLinear(&pending_delta_, realized, &lin_feedback_cos_);
  subtractProjectedAngular(&pending_delta_, realized, &ang_feedback_cos_);
  pending_clamped_ = clampPendingResidual() || pending_clamped_;
  updateDiagState();
}

bool DeltaTwistFollower::consumeNextStep() {
  if (blocked_ || block_requires_fresh_chunk_) {
    diag_.stall = true;
    diag_.step_phase = DeltaTwistStepPhase::Ringdown;
    return false;
  }
  if (!hasConsumableStep()) {
    diag_.stall = true;
    ++diag_.stall_count;
    diag_.seg_step_index = -1;
    diag_.step_phase = hasLinearOrAngularNorm(pending_delta_) ? DeltaTwistStepPhase::ResidualDrain
                                                             : DeltaTwistStepPhase::Ringdown;
    return false;
  }

  const Vec6 raw_step = finiteDelta(delta_[index_]) ? delta_[index_] : Vec6{};
  const Pose6D projection_reference =
      has_projection_reference_ ? projection_reference_pose_ : command_pose_;
  std::vector<double> grip_window = futureGripWindow(index_);
  SurfaceProjectionResult surface = surface_projector_.project(
      arm_id_,
      projection_reference,
      raw_step,
      grip_window,
      current_grip_,
      consumed_ < normal_eff_ ? SurfacePhase::Normal : SurfacePhase::Reserve
  );
  surface.close_soon = surface.close_soon || closeSoonForGrasp(cfg_.grasp_commit, grip_window);
  Vec6 projected_step = surface.projected_delta_local;
  if (!finiteDelta(projected_step)) {
    projected_step = Vec6{};
    surface.projected_delta_local = Vec6{};
    surface.discarded_delta_local = raw_step;
    surface.active = true;
  }
  step_delta_ = raw_step;
  if (grasp_commit_.clear_residual) {
    clearResidualForCommit();
  }
  if (grasp_commit_.drop_policy_delta) {
    projected_step = Vec6{};
    surface.discarded_delta_local = raw_step;
  } else if (grasp_commit_.phase == GraspPhase::PreGraspCommit) {
    const Vec6 before_commit = projected_step;
    const double t_scale = std::clamp(grasp_commit_.translation_scale, 0.0, 1.0);
    const double r_scale = std::clamp(grasp_commit_.angular_scale, 0.0, 1.0);
    projected_step.x *= t_scale;
    projected_step.y *= t_scale;
    projected_step.z *= t_scale;
    projected_step.rx *= r_scale;
    projected_step.ry *= r_scale;
    projected_step.rz *= r_scale;
    surface.discarded_delta_local = add(surface.discarded_delta_local, sub(before_commit, projected_step));
  }
  projected_step_delta_ = projected_step;
  surface_projection_ = surface;
  realized_delta_ = Vec6{};
  pending_delta_ = add(pending_delta_, projected_step);
  pending_clamped_ = clampPendingResidual() || pending_clamped_;
  if (grasp_commit_.gripper_override_active) {
    current_grip_ = grasp_commit_.gripper_target;
  } else if (!grasp_commit_.freeze_gripper) {
    current_grip_ = gripAt(index_);
  }
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

bool DeltaTwistFollower::clampPendingResidual() {
  bool clamped = false;
  clamped = scaleLinearTo(&pending_delta_, cfg_.max_residual_m) || clamped;
  clamped = scaleAngularTo(&pending_delta_, cfg_.max_residual_rad) || clamped;
  if (cfg_.max_residual_m > 0.0 && linearNorm(pending_delta_) >= 0.999 * cfg_.max_residual_m) {
    clamped = true;
  }
  if (cfg_.max_residual_rad > 0.0 && angularNorm(pending_delta_) >= 0.999 * cfg_.max_residual_rad) {
    clamped = true;
  }
  return clamped;
}

void DeltaTwistFollower::updateDiagState() {
  diag_.pending_delta = pending_delta_;
  diag_.step_delta = step_delta_;
  diag_.projected_step_delta = projected_step_delta_;
  diag_.realized_delta = realized_delta_;
  diag_.xi_ref = xi_ref_;
  diag_.xi_cmd = xi_cmd_;
  diag_.accel_cmd = accel_cmd_;
  diag_.lead_delta = lead_delta_;
  diag_.feedback_source = feedback_source_;
  diag_.pending_clamped = pending_clamped_;
  diag_.residual_cleared_on_frame = residual_cleared_on_frame_;
  diag_.min_time_to_go_used = min_time_to_go_used_;
  diag_.lin_feedback_cos = lin_feedback_cos_;
  diag_.ang_feedback_cos = ang_feedback_cos_;
  diag_.xi_ref_clamped_norm = xi_ref_clamped_norm_;
  diag_.xi_cmd_clamped_norm = xi_cmd_clamped_norm_;
  diag_.blocked = blocked_;
  diag_.block_reason = block_reason_mask_;
  diag_.block_elapsed_sec = block_elapsed_sec_;
  diag_.block_requires_fresh_chunk = block_requires_fresh_chunk_;
  diag_.residual_cleared_by_block = residual_cleared_by_block_;
  diag_.step_consumed_this_tick = step_consumed_this_tick_;
  diag_.realized_linear_ratio = robustRatio(linearNorm(realized_delta_), linearNorm(step_delta_));
  diag_.realized_angular_ratio = robustRatio(angularNorm(realized_delta_), angularNorm(step_delta_));
  diag_.realized_yaw_ratio = robustRatio(realized_delta_.rz, step_delta_.rz);
  diag_.normal_consumed = std::min(consumed_, normal_eff_);
  diag_.reserve_consumed = std::max(0, consumed_ - normal_eff_);
  diag_.seg_target_stand = command_pose_;
  diag_.surface_projection = surface_projection_;
  diag_.grasp_commit = grasp_commit_;
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

  pending_clamped_ = false;
  min_time_to_go_used_ = false;
  xi_ref_clamped_norm_ = false;
  xi_cmd_clamped_norm_ = false;
  step_consumed_this_tick_ = false;

  if (blocked_ || block_requires_fresh_chunk_) {
    if (cfg_.block_clear_residual) {
      const bool had_residual = hasLinearOrAngularNorm(pending_delta_);
      clearPendingResidual();
      residual_cleared_by_block_ = residual_cleared_by_block_ || had_residual;
    }
    ringDown(dt_tick);
    if (has_feedback_pose_) {
      command_pose_ = last_feedback_pose_;
      last_pose_ = command_pose_;
    }
    diag_.stall = true;
    diag_.step_phase = DeltaTwistStepPhase::Ringdown;
    diag_.last_solve.result = ruckig::Result::Working;
    diag_.last_solve.duration = policy_dt_;
    diag_.last_solve.alpha = 1.0;
    diag_.last_solve.converged = !diag_.saturated;
    diag_.last_solve.corner = false;
    updateDiagState();
    residual_cleared_on_frame_ = false;
    return last_pose_;
  }

  if (grasp_commit_.clear_residual) {
    clearResidualForCommit();
  }
  if (grasp_commit_.gripper_override_active) {
    current_grip_ = grasp_commit_.gripper_target;
  }

  t_in_seg_ += dt_tick;
  bool consumed_step = false;
  while (t_in_seg_ >= policy_dt_) {
    t_in_seg_ -= policy_dt_;
    const bool did_consume = consumeNextStep();
    step_consumed_this_tick_ = step_consumed_this_tick_ || did_consume;
    consumed_step = did_consume || consumed_step;
  }
  if (!consumed_step && diag_.stall) {
    stale_residual_age_ += dt_tick;
    if (cfg_.stale_residual_timeout_sec > 0.0 &&
        stale_residual_age_ > cfg_.stale_residual_timeout_sec) {
      pending_delta_ = Vec6{};
    }
  }

  pending_clamped_ = clampPendingResidual() || pending_clamped_;

  if (grasp_commit_.hold_pose || grasp_commit_.lift_active) {
    pending_delta_ = Vec6{};
    xi_ref_ = Vec6{};
    xi_cmd_ = Vec6{};
    accel_cmd_ = Vec6{};
    lead_delta_ = Vec6{};
    if (grasp_commit_.lift_active) {
      if (!has_grasp_lift_start_) {
        grasp_lift_start_pose_ = has_feedback_pose_ ? last_feedback_pose_ : command_pose_;
        has_grasp_lift_start_ = true;
      }
      command_pose_ = liftPoseFromProgress(grasp_commit_.lift_progress);
    } else if (has_feedback_pose_) {
      command_pose_ = last_feedback_pose_;
    }
    last_pose_ = command_pose_;
    diag_.saturated = false;
    diag_.last_solve.result = ruckig::Result::Working;
    diag_.last_solve.duration = policy_dt_;
    diag_.last_solve.alpha = 1.0;
    diag_.last_solve.converged = true;
    diag_.last_solve.corner = false;
    updateDiagState();
    residual_cleared_on_frame_ = false;
    return last_pose_;
  }

  const double remaining = std::max(policy_dt_ - t_in_seg_, dt_tick);
  const int extra_steps = std::max(0, cfg_.residual_drain_steps - 1);
  const double raw_horizon = std::max(
      remaining + static_cast<double>(extra_steps) * policy_dt_,
      dt_tick);
  const double min_tgo = std::max(cfg_.min_time_to_go_sec, 0.5 * policy_dt_);
  min_time_to_go_used_ = raw_horizon < min_tgo;
  const double horizon = std::max(raw_horizon, min_tgo);
  Vec6 xi_ref = mul(pending_delta_, 1.0 / horizon);
  bool saturated = clampVec6Norm(&xi_ref, cfg_.lin, cfg_.ang, &AxisLimit::v_max);
  xi_ref_clamped_norm_ = saturated;
  xi_ref_ = xi_ref;

  Vec6 a_des = mul(sub(xi_ref, xi_cmd_), 1.0 / std::max(cfg_.tau_sec, dt_tick));
  saturated = clampVec6Norm(&a_des, cfg_.lin, cfg_.ang, &AxisLimit::a_max) || saturated;

  Vec6 j_des = mul(sub(a_des, accel_cmd_), 1.0 / dt_tick);
  saturated = clampVec6Norm(&j_des, cfg_.lin, cfg_.ang, &AxisLimit::j_max) || saturated;

  accel_cmd_ = add(accel_cmd_, mul(j_des, dt_tick));
  saturated = clampVec6Norm(&accel_cmd_, cfg_.lin, cfg_.ang, &AxisLimit::a_max) || saturated;
  xi_cmd_ = add(xi_cmd_, mul(accel_cmd_, dt_tick));
  xi_cmd_clamped_norm_ = clampVec6Norm(&xi_cmd_, cfg_.lin, cfg_.ang, &AxisLimit::v_max);
  saturated = xi_cmd_clamped_norm_ || saturated;

  const Vec6 delta_exec = mul(xi_cmd_, dt_tick);
  command_pose_ = math::composeDeltaLocal(command_pose_, deltaPose(delta_exec));
  clampCommandLead();
  last_pose_ = command_pose_;

  pending_clamped_ = clampPendingResidual() || pending_clamped_;
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
  residual_cleared_on_frame_ = false;
  return last_pose_;
}

}  // namespace rb_servo::control
