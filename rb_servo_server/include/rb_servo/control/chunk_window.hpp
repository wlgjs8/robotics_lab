// chunk_window.hpp — receding-horizon window manager over a VLA action chunk.
//
// Owns one active (smoothed) chunk frame and the L/C/R bookkeeping:
//   * discard the front L steps (already-past inference latency),
//   * consume C steps before a fresh chunk preempts,
//   * keep R steps of reserve so central-difference vf/af has a real k+1 neighbor.
// Enforces the hard invariant L + C + R <= horizon (clamps C, never over-reads).
// Smoothing happens HERE, before the follower takes differences (vf/af amplify
// jitter). The follower (cartesian_chunk_follower) pulls flanking smoothed poses
// and builds the BVP; this class does not touch ruckig or orientation tangents.

#pragma once

#include "rb_servo/core/types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rb_servo::control {

// One chunk as received on the servo side (absolute stand-frame waypoints,
// measured-anchored at idx0 by the producer). Optional delta rows preserve the
// conditioned local/body-frame model action used to generate those waypoints;
// current followers still consume pose/grip.
struct ChunkFrame {
  std::vector<Pose6D> pose;      // absolute stand-frame waypoints
  std::vector<double> grip;      // per-waypoint gripper target (producer units)
  std::vector<Vec6> delta;       // optional conditioned per-frame local/body deltas
  double policy_dt{1.0 / 30.0};  // per-step wall clock
  std::uint64_t wire_seq{0};     // producer packet seq / flow chunk id
  std::uint64_t recv_seq{0};     // receiver-local accepted-frame count
  double recv_time{0.0};         // servo receive-time stamp (feed-liveness)
};

struct ChunkWindowConfig {
  int discard_head_L{6};   // front steps to drop (measured inference latency)
  int consume_C{8};        // steps consumed before preemption (replan cadence)
  int reserve_R{2};        // lookahead reserve for central difference (>=1)
  int smoothing_window{3}; // odd; <=1 disables. Applied before differences.
};

class ChunkWindow {
 public:
  explicit ChunkWindow(ChunkWindowConfig cfg) : cfg_(cfg) {}

  // Activate a new (preempting) chunk frame: smooth it, discard the head L,
  // reset the consume pointer. Clamps the effective consume count so that
  // L + C_eff + R <= horizon. Returns false if the frame is too short to yield
  // even one consumable step with reserve.
  bool activate(const ChunkFrame& frame);

  bool active() const { return active_; }
  void deactivate() { active_ = false; }

  // A consumable step k exists (consumed < C_eff) with a real forward neighbor
  // k+1 for the central difference.
  bool hasStep() const;
  // Tail consumption: a step still exists in the data (k <= N-1) even past the
  // consume budget / without a strict forward neighbor. Central differences
  // clamp at the edge (d_{k+1} -> 0), so tail steps decelerate INTO the final
  // waypoint — used when the next chunk is late, instead of stalling early.
  bool hasTailStep() const { return active_ && k_ < pose_.size(); }
  // The consume budget C is spent — a fresh chunk should preempt.
  bool windowExhausted() const { return consumed_ >= consume_eff_; }

  std::size_t index() const { return k_; }   // current consumed absolute index
  void advance() { ++k_; ++consumed_; }

  const Pose6D& poseAt(std::size_t k) const;  // smoothed, clamped to [0, N-1]
  double gripAt(std::size_t k) const;

  double policyDt() const { return dt_; }
  std::uint64_t wireSeq() const { return wire_seq_; }
  std::uint64_t recvSeq() const { return recv_seq_; }
  double recvTime() const { return recv_; }
  int consumed() const { return consumed_; }
  int consumeBudget() const { return consume_eff_; }
  std::size_t horizon() const { return pose_.size(); }

 private:
  ChunkWindowConfig cfg_;
  std::vector<Pose6D> pose_;   // smoothed absolute waypoints
  std::vector<double> grip_;
  double dt_{1.0 / 30.0};
  std::uint64_t wire_seq_{0};
  std::uint64_t recv_seq_{0};
  double recv_{0.0};
  std::size_t k_{0};           // current consumed absolute index
  int consumed_{0};
  int consume_eff_{0};         // C clamped to fit L + C + R <= horizon
  bool active_{false};
};

}  // namespace rb_servo::control
