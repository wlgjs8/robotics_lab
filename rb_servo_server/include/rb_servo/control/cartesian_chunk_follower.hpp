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
#include <cstddef>
#include <cstdint>

#include "rb_servo/control/chunk_follower_core.hpp"
#include "rb_servo/control/chunk_window.hpp"
#include "rb_servo/core/types.hpp"

#include <Eigen/Geometry>

#include <optional>

namespace rb_servo::control {

// Sliding-window rate budget over a caller-owned ring of timestamps. Records `now_ns`
// and reports whether MORE than `max_in_window` stamps now fall inside `window_sec`.
//
// Used by the chunk follower's actual-lead gate: a lead breach RE-ANCHORS the plan
// (removing the lead, and with it the catch-up lunge) and only latches once re-anchoring
// has demonstrably stopped helping. It is a RATE limit, not a lifetime count, so a long
// healthy run never accumulates its way into a latch.
//
// window_sec <= 0 or max_in_window <= 0 disables the budget: every call reports spent,
// i.e. the legacy latch-on-first-breach behavior.
bool recordAndCheckRateBudget(
    std::uint64_t* ring,
    std::size_t capacity,
    std::size_t& head,
    std::uint64_t now_ns,
    double window_sec,
    int max_in_window);


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

  // THE FORCE GATE ON THE PLAN ADVANCE (ported from controller-manager's follow
  // path). As the contact force rises, the fraction of each segment's advance that
  // survives ALONG THE DIRECTION PUSHING INTO THE WRENCH falls toward zero; the
  // tangential and retreating components pass at FULL authority, so sliding along a
  // contact and backing out of it are never throttled.
  //
  // *** THIS IS WHAT BOUNDS THE CONTACT FORCE UNDER A STREAMING PLAN. *** A spring
  // (k > 0) alone does not: |d| <= F/k holds only at a true static equilibrium, and
  // while the plan keeps advancing into a surface there is no equilibrium at all —
  // the nominal walks into the workpiece and the deviation, and the force, grow
  // without bound. controller-manager measured that as 961 N in 40 s.
  //
  // The removal accumulates as a PLAN SHIFT applied to every knot in the window, so
  // the chained core state stays continuous, the plan is not rewritten, and contact
  // release causes no snap-back. `gate` outside [0,1] is clamped; gate >= 1 with any
  // direction is a strict no-op.
  void setAdvanceGate(double gate, const Eigen::Vector3d& into_contact_dir_stand);

  // The accumulated plan shift [m] — how far the gate has held this plan back.
  double planShift() const { return plan_shift_.norm(); }

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
  // Chained Ruckig end-state velocity for the active segment. Axes 0--2 are
  // stand-frame translation; axes 3--5 are the unscaled R0_ref-local tangent
  // coordinates used by tangentPose() (q = R0_ref * exp(theta)). A fresh engage
  // has only a zero-velocity seed and no solved segment, so it reports no valid
  // velocity.
  std::optional<Vec6> currentVelocity() const;
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

  // The force gate on the plan advance (see setAdvanceGate).
  double advance_gate_{1.0};
  Eigen::Vector3d into_contact_dir_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plan_shift_{Eigen::Vector3d::Zero()};
};

}  // namespace rb_servo::control
