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
#include "rb_servo/control/grasp_commit_coordinator.hpp"
#include "rb_servo/control/surface_action_projector.hpp"
#include "rb_servo/core/types.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
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
  bool clear_residual_on_new_frame{true};
  double min_time_to_go_sec{0.015};
  double max_residual_m{0.030};
  double max_residual_rad{0.35};
  double max_lead_m{0.060};
  double max_lead_rad{0.30};
  double stale_residual_timeout_sec{0.15};
  bool pause_on_safety_block{true};
  double block_requires_fresh_chunk_sec{0.050};
  bool block_clear_residual{true};
  SurfaceActionProjectorConfig surface_action_projector{};
  GraspCommitConfig grasp_commit{};
};

enum class DeltaTwistFeedbackSource {
  None = 0,
  Actual = 1,
  PreviousSentFk = 2,
};

enum class DeltaTwistStepPhase {
  Inactive,
  Normal,
  Reserve,
  ResidualDrain,
  Ringdown,
};

enum class GraspCloseSoonSource {
  None = 0,
  MotionOverlayGrip = 1,
  FullGripHorizon = 2,
  NearFloorDwellFallback = 3,
};

struct GripHorizonSummary {
  bool available{false};
  int len{0};
  int current_index{-1};
  double min{std::numeric_limits<double>::quiet_NaN()};
  double max{std::numeric_limits<double>::quiet_NaN()};
  int argmin{-1};
  int argmax{-1};
  int first_close_index{-1};
  int first_close_steps_ahead{-1};
  double close_threshold{std::numeric_limits<double>::quiet_NaN()};
  bool close_is_greater{false};
  int close_soon_source{static_cast<int>(GraspCloseSoonSource::None)};
  int close_soon_steps_ahead{-1};
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
  Vec6 projected_step_delta{};
  Vec6 realized_delta{};
  Vec6 xi_ref{};
  Vec6 xi_cmd{};
  Vec6 accel_cmd{};
  Vec6 lead_delta{};
  int feedback_source{0};
  bool pending_clamped{false};
  bool residual_cleared_on_frame{false};
  bool min_time_to_go_used{false};
  double lin_feedback_cos{1.0};
  double ang_feedback_cos{1.0};
  bool xi_ref_clamped_norm{false};
  bool xi_cmd_clamped_norm{false};
  bool blocked{false};
  int block_reason{0};
  double block_elapsed_sec{0.0};
  bool block_requires_fresh_chunk{false};
  bool residual_cleared_by_block{false};
  bool step_consumed_this_tick{false};
  double realized_linear_ratio{1.0};
  double realized_angular_ratio{1.0};
  double realized_yaw_ratio{1.0};
  bool saturated{false};
  DeltaTwistStepPhase step_phase{DeltaTwistStepPhase::Inactive};
  int normal_consumed{0};
  int reserve_consumed{0};
  SurfaceProjectionResult surface_projection{};
  GripHorizonSummary grip_horizon{};
  GraspCommitArmCommand grasp_commit{};
};

class DeltaTwistFollower {
 public:
  explicit DeltaTwistFollower(const DeltaTwistFollowerConfig& cfg);

  void reconfigure(const DeltaTwistFollowerConfig& cfg);
  void setArmId(ArmId arm_id) { arm_id_ = arm_id; }
  void submitFrame(const ChunkFrame& frame, const Pose6D& current_pose);
  bool active() const { return active_; }
  void deactivate();
  void reanchor(const Pose6D& reference);
  void pauseBlocked(const Pose6D& feedback_or_reference_pose, double dt);
  void clearPendingResidual();
  void ringDown(double dt);
  void setBlocked(bool blocked, int reason_mask = 0);
  void setFeedbackPose(
      const Pose6D& feedback_pose,
      DeltaTwistFeedbackSource source = DeltaTwistFeedbackSource::PreviousSentFk
  );
  void setGraspCommitCommand(const GraspCommitArmCommand& command);
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
  std::vector<double> futureGripWindow(std::size_t k) const;
  GripHorizonSummary summarizeGripHorizon(std::size_t k) const;
  bool clampPendingResidual();
  void clampCommandLead();
  void clearResidualForCommit();
  Pose6D liftPoseFromProgress(double progress) const;
  void updateDiagState();

  DeltaTwistFollowerConfig cfg_;
  ArmId arm_id_{ArmId::Left};
  SurfaceActionProjector surface_projector_;
  double policy_dt_{1.0 / 30.0};
  double recv_time_{0.0};
  std::uint64_t wire_seq_{0};
  std::uint64_t recv_seq_{0};
  std::vector<Vec6> delta_;
  std::vector<double> grip_;
  std::vector<double> grip_horizon_;
  bool full_grip_horizon_available_{false};
  std::size_t index_{0};
  int consumed_{0};
  int normal_eff_{0};
  int total_eff_{0};
  double stale_residual_age_{0.0};

  Pose6D command_pose_{};
  Pose6D last_pose_{};
  Pose6D last_feedback_pose_{};
  Pose6D projection_reference_pose_{};
  bool has_feedback_pose_{false};
  bool has_projection_reference_{false};
  Vec6 xi_cmd_{};
  Vec6 accel_cmd_{};
  Vec6 pending_delta_{};
  Vec6 step_delta_{};
  Vec6 projected_step_delta_{};
  Vec6 realized_delta_{};
  Vec6 xi_ref_{};
  Vec6 lead_delta_{};
  int feedback_source_{0};
  bool pending_clamped_{false};
  bool residual_cleared_on_frame_{false};
  bool min_time_to_go_used_{false};
  double lin_feedback_cos_{1.0};
  double ang_feedback_cos_{1.0};
  bool xi_ref_clamped_norm_{false};
  bool xi_cmd_clamped_norm_{false};
  bool blocked_{false};
  int block_reason_mask_{0};
  double block_elapsed_sec_{0.0};
  bool block_requires_fresh_chunk_{false};
  bool residual_cleared_by_block_{false};
  bool step_consumed_this_tick_{false};
  double current_grip_{0.0};
  double t_in_seg_{0.0};
  bool active_{false};
  SurfaceProjectionResult surface_projection_{};
  GraspCommitArmCommand grasp_commit_{};
  GraspPhase previous_grasp_phase_{GraspPhase::Normal};
  Pose6D grasp_lift_start_pose_{};
  bool has_grasp_lift_start_{false};
  GripHorizonSummary grip_horizon_summary_{};
  DeltaTwistFollowerDiag diag_{};
};

}  // namespace rb_servo::control
