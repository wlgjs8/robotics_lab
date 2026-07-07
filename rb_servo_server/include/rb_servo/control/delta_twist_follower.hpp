// delta_twist_follower.hpp - local action-delta follower for chunk overlays.
//
// This controller consumes the conditioned model action rows directly:
//   [dx, dy, dz, drx, dry, drz]
// Each row is a per-policy-frame local/body displacement, not a velocity. The
// controller drains those deltas through an internal jerk-limited body twist and
// emits absolute stand-frame TcpPoseTarget poses at servo rate.

#pragma once

#include "rb_servo/control/chunk_follower_core.hpp"
#include "rb_servo/control/chunk_window.hpp"
#include "rb_servo/core/types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rb_servo::control {

struct DeltaTwistFollowerConfig {
  AxisLimit lin{1.0, 6.0, 30.0};
  AxisLimit ang{2.0, 12.0, 60.0};
  int discard_head_steps{0};
  int consume_steps{8};
  int reserve_steps{1};
  double tau_sec{0.015};
  int residual_drain_steps{1};
  double max_residual_m{0.030};
  double max_residual_rad{0.35};
  double max_lead_m{0.060};
  double max_lead_rad{0.30};
  double stale_residual_timeout_sec{0.15};
};

enum class DeltaTwistStepPhase {
  Inactive,
  Normal,
  Reserve,
  ResidualDrain,
  Ringdown,
};

struct DeltaTwistFollowerDiag {
  SegmentSolve last_solve{};
  bool stall{false};
  int stall_count{0};
  int segments{0};
  Pose6D seg_target_stand{};
  int seg_step_index{-1};
  std::uint64_t seg_wire_seq{0};
  std::uint64_t seg_recv_seq{0};
  Vec6 pending_delta{};
  Vec6 step_delta{};
  Vec6 realized_delta{};
  Vec6 xi_ref{};
  Vec6 xi_cmd{};
  Vec6 accel_cmd{};
  Vec6 lead_delta{};
  double realized_linear_ratio{1.0};
  double realized_angular_ratio{1.0};
  double realized_yaw_ratio{1.0};
  bool saturated{false};
  DeltaTwistStepPhase step_phase{DeltaTwistStepPhase::Inactive};
  int normal_consumed{0};
  int reserve_consumed{0};
};

class DeltaTwistFollower {
 public:
  explicit DeltaTwistFollower(const DeltaTwistFollowerConfig& cfg);

  void reconfigure(const DeltaTwistFollowerConfig& cfg);
  void submitFrame(const ChunkFrame& frame, const Pose6D& current_pose);
  bool active() const { return active_; }
  void deactivate();
  void reanchor(const Pose6D& reference);
  void setFeedbackPose(const Pose6D& feedback_pose);
  Pose6D tick(double dt_tick);

  double currentGrip() const { return current_grip_; }
  const Pose6D& lastPose() const { return last_pose_; }
  const DeltaTwistFollowerDiag& diag() const { return diag_; }
  double tInSegment() const { return t_in_seg_; }
  double ageSince(double now) const { return active_ ? now - recv_time_ : 0.0; }
  std::uint64_t windowWireSeq() const { return wire_seq_; }
  std::uint64_t windowRecvSeq() const { return recv_seq_; }
  std::size_t windowIndex() const { return index_; }
  int windowConsumed() const { return consumed_; }
  const Vec6& pendingDelta() const { return pending_delta_; }
  const Vec6& xiCommand() const { return xi_cmd_; }

 private:
  bool consumeNextStep();
  bool hasConsumableStep() const;
  double gripAt(std::size_t k) const;
  void clampPendingResidual();
  void clampCommandLead();
  void updateDiagState();

  DeltaTwistFollowerConfig cfg_;
  double policy_dt_{1.0 / 30.0};
  double recv_time_{0.0};
  std::uint64_t wire_seq_{0};
  std::uint64_t recv_seq_{0};
  std::vector<Vec6> delta_;
  std::vector<double> grip_;
  std::size_t index_{0};
  int consumed_{0};
  int normal_eff_{0};
  int total_eff_{0};
  double stale_residual_age_{0.0};

  Pose6D command_pose_{};
  Pose6D last_pose_{};
  Pose6D last_feedback_pose_{};
  bool has_feedback_pose_{false};
  Vec6 xi_cmd_{};
  Vec6 accel_cmd_{};
  Vec6 pending_delta_{};
  Vec6 step_delta_{};
  Vec6 realized_delta_{};
  Vec6 xi_ref_{};
  Vec6 lead_delta_{};
  double current_grip_{0.0};
  double t_in_seg_{0.0};
  bool active_{false};
  DeltaTwistFollowerDiag diag_{};
};

}  // namespace rb_servo::control
