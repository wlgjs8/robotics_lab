// cartesian_chunk_follower.hpp — servo-loop-facing Cartesian chunk follower.
//
// Wraps the pure ChunkFollowerSegment<6> (3 stand-position + 3 orientation axes)
// and a ChunkWindow, and adds the Cartesian-specific glue:
//   * orientation as a rotation-vector in a per-segment tangent about R0_ref,
//     re-linearized every 33 ms (log3/exp3 from math/se3),
//   * receding-horizon tick driving (consume the first dt of each solution,
//     re-solve at each 33 ms boundary, preempt on a fresh chunk),
//   * stall ring-down (zero-velocity target) when the window is exhausted with
//     no fresh chunk, and per-tick gripper phase-locked to the consumed step.
//
// Mirrors the SmdPoseTracker lifecycle so it is a drop-in for the SMD stage at
// applyPoseTrackSmd: submitFrame() ~ reset/updateGoal, tick() ~ step().

#pragma once

#include "rb_servo/control/chunk_follower_core.hpp"
#include "rb_servo/control/chunk_window.hpp"
#include "rb_servo/core/types.hpp"

#include <Eigen/Geometry>

namespace rb_servo::control {

struct CartesianChunkFollowerConfig {
  AxisLimit lin{1.0, 6.0, 30.0};   // linear per-axis limits (m, m/s, m/s^2, m/s^3)
  AxisLimit ang{2.0, 12.0, 60.0};  // angular per-axis limits (rad ...)
  ChunkWindowConfig window{};
  GuardConfig guard{};
  double max_projection_error_m{0.0};
  double max_projection_error_rad{0.0};
  int max_consecutive_projection_errors{0};
  double max_actual_lead_m{0.0};
  double max_actual_lead_rad{0.0};
  int max_consecutive_actual_lead_errors{0};
  // Quasi-static gate: loading projection only runs while the plan's linear
  // acceleration is below this (fast-transit inertial wrench is not contact).
  double loading_projection_max_accel_m_s2{0.5};
};

struct FollowerDiag {
  SegmentSolve last_solve{};
  bool stall{false};
  int stall_count{0};
  int segments{0};
  // Active-segment telemetry (updated at each 33ms boundary): the chunk step
  // this segment is driving toward, as an absolute stand pose, plus indices.
  Pose6D seg_target_stand{};
  int seg_step_index{-1};        // absolute chunk index (-1 = ring-down/no data)
  std::uint64_t seg_wire_seq{0}; // producer packet seq / flow chunk id
  std::uint64_t seg_recv_seq{0}; // receiver-local accepted-frame count
  double projection_error_m{0.0};
  double projection_error_rad{0.0};
  int consecutive_projection_errors{0};
  double actual_lead_m{0.0};
  double actual_lead_rad{0.0};
  int consecutive_actual_lead_errors{0};
  bool infeasible_fault{false};
  bool actual_lead_fault{false};
  // Wrench-gated loading projection (contact-aware following): true when the
  // last segment boundary removed a loading component from the plan advance;
  // contact_shift_m is the accumulated plan shift magnitude (resets per delta
  // frame because delta frames re-anchor their knots at the emitted pose).
  bool loading_projection_active{false};
  double contact_shift_m{0.0};
};

enum class HoldResumeResult {
  NotPaused,
  WarmResumed,
  GraceExpired,
  Diverged,
};

class CartesianChunkFollower {
 public:
  explicit CartesianChunkFollower(const CartesianChunkFollowerConfig& cfg);

  // Apply a new config in place (profile change): swaps limits/window/guards,
  // deactivates, and keeps the prewarmed ruckig OTG (no reconstruction cost).
  void reconfigure(const CartesianChunkFollowerConfig& cfg);

  // Deliver a chunk frame. Cold start (inactive): seed the chained state from
  // current_pose with zero velocity and anchor the orientation tangent there.
  // Already active: preempt — swap the window frame but KEEP the chained state
  // (receding-horizon, no discontinuity at the chunk seam).
  void submitFrame(const ChunkFrame& frame, const Pose6D& current_pose);
  // Delta-preview input: integrate each local/body delta on the server using
  // the canonical Eigen/Pinocchio SE(3) path, then feed the resulting absolute
  // knots to the same Ruckig receding-horizon follower.
  void submitDeltaFrame(const ChunkFrame& frame, const Pose6D& current_pose);
  void updateActualLead(const Pose6D& actual_pose);

  // Wrench-gated loading projection (contact-aware following). Pass the
  // deadband-filtered LOADING direction in the stand frame — the same
  // convention as the servo loop's policy-delta projection: the negation of
  // the measured external force error, i.e. the direction that presses INTO
  // the contact. While valid, each segment boundary removes the component of
  // the plan advance along this direction (accumulated as a plan shift), so
  // the follower never integrates into a loaded contact; tangential motion
  // passes through and contact release causes no snap-back. Invalid (the
  // default) is a strict no-op: behavior is identical to the pre-projection
  // follower, so an unhealthy wrench pipeline degrades safely.
  void setExternalReaction(const Eigen::Vector3d& loading_dir_stand,
                           bool valid,
                           bool contact_normal_owned = false);

  bool active() const { return active_; }
  bool holdPaused() const { return hold_paused_; }
  void deactivate();

  // Freeze an active chunk during a brief upstream Hold. No segment/window
  // time advances until resumeFromHold succeeds. Non-Hold mode changes still
  // use deactivate() and discard the window.
  void pauseForHold(double now_sec);
  bool expireHoldPause(double now_sec, double grace_sec);
  HoldResumeResult resumeFromHold(const Pose6D& live_reference,
                                  double now_sec,
                                  double grace_sec,
                                  double position_tolerance_m,
                                  double orientation_tolerance_rad);

  // Strict-divergence recovery when an external safety constraint already
  // explained the plan-vs-sent split. Keeps the active chunk window and consume
  // pointer intact, but reseeds the chained state at the live sent-pose reference.
  void reanchor(const Pose6D& reference);

  // Advance by one servo tick and return the smoothed stand-frame pose setpoint.
  // Crosses 33 ms segment boundaries internally (solve next BVP / ring down).
  Pose6D tick(double dt_tick);

  double currentGrip() const { return current_grip_; }
  const Pose6D& lastPose() const { return last_pose_; }
  const FollowerDiag& diag() const { return diag_; }
  double tInSegment() const { return t_in_seg_; }
  std::uint64_t windowWireSeq() const { return window_.wireSeq(); }
  std::uint64_t windowRecvSeq() const { return window_.recvSeq(); }
  std::size_t windowIndex() const { return window_.index(); }
  int windowConsumed() const { return window_.consumed(); }
  // Seconds since the active frame was received (feed-liveness watchdog input).
  double ageSince(double now) const { return active_ ? now - window_.recvTime() : 0.0; }

 private:
  void stepToNextSegment();
  BoundarySample<6> buildSample(std::size_t k) const;
  SegmentSolve ringDown();
  void relinearizeAndReseed();      // roll R0_ref to the chained orientation, reset rot axes
  Pose6D sampleAt(double t) const;
  // Convert a 6-axis point (pos + tangent rotation-vector) to a stand pose.
  Pose6D tangentPose(const std::array<double, 6>& axes) const;

  CartesianChunkFollowerConfig cfg_;
  std::array<AxisLimit, 6> limits_{};
  ChunkFollowerSegment<6> core_;
  ChunkWindow window_;

  Eigen::Quaterniond R0_ref_{Eigen::Quaterniond::Identity()};  // current segment tangent anchor
  double seg_dt_{1.0 / 30.0};
  double t_in_seg_{0.0};
  bool have_segment_{false};
  bool active_{false};
  bool hold_paused_{false};
  double hold_pause_start_sec_{0.0};

  double current_grip_{0.0};
  Pose6D last_pose_{};
  FollowerDiag diag_{};
  int actual_lead_checked_segment_{-1};

  // Loading-projection state (see setExternalReaction).
  Eigen::Vector3d loading_dir_stand_{Eigen::Vector3d::Zero()};
  bool loading_dir_valid_{false};
  bool contact_normal_owned_{false};
  Eigen::Vector3d contact_shift_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_loading_dir_{Eigen::Vector3d::Zero()};
  bool prev_loading_dir_valid_{false};
};

}  // namespace rb_servo::control
