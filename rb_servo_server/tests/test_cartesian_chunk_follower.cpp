// test_cartesian_chunk_follower.cpp — the servo-loop-facing follower: receding
// horizon tick driving, per-tick continuity, orientation on SO(3), stall
// ring-down, and chunk preemption continuity.

#include "rb_servo/control/cartesian_chunk_follower.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <vector>

using rb_servo::control::CartesianChunkFollower;
using rb_servo::control::CartesianChunkFollowerConfig;
using rb_servo::control::ChunkFrame;
using rb_servo::control::HoldResumeResult;
using rb_servo::Pose6D;
using rb_servo::Vec6;

static int g_failures = 0;
static void check(bool ok, const char* name) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

static constexpr double TICK = 0.002;   // 500 Hz
static constexpr double SEG = 1.0 / 30.0;

// A gentle straight-line reference with a slow constant rotation about z.
static ChunkFrame makeRef(int n, double vx = 0.03, double wz = 0.2) {
  ChunkFrame f;
  f.policy_dt = SEG;
  f.wire_seq = 55;
  f.recv_seq = 20;
  f.recv_time = 0.0;
  for (int i = 0; i < n; ++i) {
    const double t = i * SEG;
    Pose6D p;
    p.x = vx * t;
    p.y = 0.0;
    p.z = 0.2;
    const double ang = wz * t;
    // quaternion for rotation ang about z:
    p.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, std::sin(ang / 2), std::cos(ang / 2)};
    f.pose.push_back(p);
    f.grip.push_back(20.0 + i);
  }
  return f;
}

static double quatNorm(const Pose6D& p) {
  const auto& q = *p.quaternion_xyzw;
  return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

static bool poseNear(const Pose6D& a, const Pose6D& b, double pos_tol, double ang_tol) {
  return rb_servo::math::positionDistance(a, b) < pos_tol &&
         rb_servo::math::orientationDistanceRad(a, b) < ang_tol;
}

static Pose6D poseWithXAndYaw(double x, double yaw_rad) {
  Pose6D p;
  p.x = x;
  p.y = 0.0;
  p.z = 0.2;
  p.quaternion_xyzw =
      std::array<double, 4>{0.0, 0.0, std::sin(yaw_rad / 2), std::cos(yaw_rad / 2)};
  return p;
}

static ChunkFrame makeLinearReverseFrame(double x_mid) {
  ChunkFrame f;
  f.policy_dt = SEG;
  f.pose.assign(6, poseWithXAndYaw(0.0, 0.0));
  f.pose[1] = poseWithXAndYaw(x_mid, 0.0);
  f.grip.assign(f.pose.size(), 20.0);
  return f;
}

static ChunkFrame makeAngularReverseFrame(double yaw_mid) {
  ChunkFrame f;
  f.policy_dt = SEG;
  f.pose.assign(6, poseWithXAndYaw(0.0, 0.0));
  f.pose[1] = poseWithXAndYaw(0.0, yaw_mid);
  f.grip.assign(f.pose.size(), 20.0);
  return f;
}

static bool firstSegmentCorner(const CartesianChunkFollowerConfig& cfg,
                               const ChunkFrame& frame,
                               bool* solved) {
  CartesianChunkFollower f(cfg);
  f.submitFrame(frame, frame.pose[static_cast<std::size_t>(cfg.window.discard_head_L)]);
  f.tick(TICK);
  if (solved) {
    *solved = f.diag().last_solve.result == ruckig::Result::Working;
  }
  return f.diag().last_solve.corner;
}

int main() {
  CartesianChunkFollowerConfig cfg;
  cfg.window = {/*L*/ 2, /*C*/ 10, /*R*/ 2, /*smooth*/ 3};

  // -- Test 1: activation, per-tick continuity, forward progress, SO(3). ------
  std::printf("Test 1: tick continuity + progress + orientation\n");
  {
    CartesianChunkFollower f(cfg);
    ChunkFrame frame = makeRef(16);
    const Pose6D start = frame.pose[cfg.window.discard_head_L];  // first consumed
    f.submitFrame(frame, start);
    check(f.active(), "active after submitFrame");

    Pose6D prev = f.tick(TICK);
    bool continuous = true, quat_ok = true;
    double max_jump = 0.0;
    for (int i = 0; i < 120; ++i) {  // ~7 segments
      Pose6D cur = f.tick(TICK);
      const double jump = std::sqrt(std::pow(cur.x - prev.x, 2) +
                                    std::pow(cur.y - prev.y, 2) +
                                    std::pow(cur.z - prev.z, 2));
      max_jump = std::max(max_jump, jump);
      continuous &= jump < 5e-3;          // ≤ v_max·dt with margin
      quat_ok &= std::fabs(quatNorm(cur) - 1.0) < 1e-6;
      prev = cur;
    }
    check(continuous, "per-tick position jump bounded (C0/C1 continuous)");
    check(quat_ok, "orientation stays a unit quaternion");
    check(prev.x > start.x + 1e-4, "setpoint progresses forward along the path");
    check(prev.z > 0.15 && prev.z < 0.25, "off-axis (z) stays put");
    std::printf("    max per-tick jump = %.5f m, final x = %.4f (start %.4f)\n",
                max_jump, prev.x, start.x);
  }

  // -- Test: brief Hold freezes time and preserves the chained velocity. -----
  std::printf("Test: warm resume preserves frozen chained state\n");
  {
    CartesianChunkFollower f(cfg);
    ChunkFrame frame = makeRef(18, 0.20, 0.0);
    const Pose6D start = frame.pose[cfg.window.discard_head_L];
    f.submitFrame(frame, start);
    Pose6D before_prev = start;
    Pose6D before = start;
    for (int i = 0; i < 100; ++i) {
      before_prev = before;
      before = f.tick(TICK);
    }
    const double travel_before = rb_servo::math::positionDistance(before_prev, before);
    const std::size_t index_before = f.windowIndex();
    const int consumed_before = f.windowConsumed();
    const double phase_before = f.tInSegment();

    f.pauseForHold(10.0);
    check(f.holdPaused(), "brief Hold marks the active follower paused");
    bool frozen = true;
    for (int i = 0; i < 180; ++i) {
      frozen &= poseNear(f.tick(TICK), before, 1e-12, 1e-12);
    }
    check(frozen, "Hold ticks do not advance the emitted pose");
    check(f.windowIndex() == index_before && f.windowConsumed() == consumed_before &&
              std::fabs(f.tInSegment() - phase_before) < 1e-12,
          "Hold ticks freeze window consumption and segment time");
    check(f.resumeFromHold(before, 10.36, 0.5, 0.05, 0.10) ==
              HoldResumeResult::WarmResumed,
          "gap inside grace window warm-resumes");
    const Pose6D after = f.tick(TICK);
    const double travel_after = rb_servo::math::positionDistance(before, after);
    check(travel_after <= cfg.lin.v_max * TICK + 1e-9,
          "first resumed tick stays within one velocity-limited tick");
    check(travel_before > 1e-7 && travel_after > 0.25 * travel_before,
          "warm resume retains mid-motion velocity instead of cold-seeding at zero");
  }

  // -- Test: a Hold beyond the grace window returns to cold-start semantics. -
  std::printf("Test: expired Hold gap cold-starts\n");
  {
    CartesianChunkFollower f(cfg);
    ChunkFrame frame = makeRef(18, 0.20, 0.0);
    f.submitFrame(frame, frame.pose[cfg.window.discard_head_L]);
    for (int i = 0; i < 100; ++i) f.tick(TICK);
    const Pose6D frozen = f.lastPose();
    f.pauseForHold(20.0);
    check(f.resumeFromHold(frozen, 20.51, 0.5, 0.05, 0.10) ==
              HoldResumeResult::GraceExpired,
          "gap beyond grace reports expiry");
    check(!f.active() && !f.holdPaused(), "expired Hold discards the active window");
    f.submitFrame(frame, frozen);
    const Pose6D cold_first = f.tick(TICK);
    check(poseNear(cold_first, frozen, 1e-12, 1e-12),
          "next frame cold-starts from the live reference with zero state");
  }

  // -- Test 2: stall ring-down when the window is exhausted. ------------------
  std::printf("Test 2: stall ring-down\n");
  {
    CartesianChunkFollower f(cfg);
    ChunkFrame frame = makeRef(16);
    f.submitFrame(frame, frame.pose[cfg.window.discard_head_L]);
    // Consume well past C so the window exhausts and the follower stalls.
    for (int i = 0; i < 400; ++i) f.tick(TICK);
    check(f.diag().stall, "stall flagged after window exhaustion");
    check(f.diag().stall_count > 0, "stall count incremented");
    // After stalling, velocity rings to ~0: consecutive setpoints converge.
    Pose6D a = f.tick(TICK);
    for (int i = 0; i < 60; ++i) f.tick(TICK);
    Pose6D b = f.tick(TICK);
    Pose6D c = f.tick(TICK);
    const double resid = std::fabs(c.x - b.x) + std::fabs(c.y - b.y) + std::fabs(c.z - b.z);
    check(resid < 1e-5, "setpoint is stationary after ring-down (velocity → 0)");
    (void)a;
  }

  // -- Test 3: preemption keeps continuity (no jump at the chunk seam). -------
  std::printf("Test 3: chunk preemption continuity\n");
  {
    CartesianChunkFollower f(cfg);
    ChunkFrame frame1 = makeRef(16);
    f.submitFrame(frame1, frame1.pose[cfg.window.discard_head_L]);
    for (int i = 0; i < 40; ++i) f.tick(TICK);
    const Pose6D before = f.tick(TICK);

    // A fresh chunk arrives (continuing the path from where we are).
    ChunkFrame frame2 = makeRef(16, 0.03, 0.2);
    for (auto& p : frame2.pose) { p.x += before.x; }  // re-anchored ahead
    frame2.wire_seq = 56;
    frame2.recv_seq = 21;
    f.submitFrame(frame2, before);  // active → preempt, keep chained state
    const Pose6D after = f.tick(TICK);
    const double seam = std::sqrt(std::pow(after.x - before.x, 2) +
                                  std::pow(after.y - before.y, 2) +
                                  std::pow(after.z - before.z, 2));
    check(f.active(), "still active after preemption");
    check(seam < 5e-3, "no position jump across the preemption seam");
    std::printf("    seam jump = %.6f m\n", seam);
  }

  // -- Test 4: producer wire seq and receiver seq both reach FollowerDiag. ----
  std::printf("Test 4: seq propagation\n");
  {
    CartesianChunkFollower f(cfg);
    ChunkFrame frame = makeRef(16);
    frame.wire_seq = 1234;
    frame.recv_seq = 7;
    f.submitFrame(frame, frame.pose[cfg.window.discard_head_L]);
    f.tick(TICK);
    check(f.diag().seg_wire_seq == 1234, "diag carries producer wire seq");
    check(f.diag().seg_recv_seq == 7, "diag carries receiver-local seq");
  }

  // -- Test 5: strict-divergence reanchor keeps the active chunk window. -------
  std::printf("Test 5: reanchor keeps window and restarts from reference\n");
  {
    CartesianChunkFollower f(cfg);
    ChunkFrame frame = makeRef(18);
    frame.wire_seq = 2222;
    frame.recv_seq = 33;
    f.submitFrame(frame, frame.pose[cfg.window.discard_head_L]);
    for (int i = 0; i < 80; ++i) f.tick(TICK);  // a few 33 ms segments

    const std::uint64_t wire_before = f.windowWireSeq();
    const std::uint64_t recv_before = f.windowRecvSeq();
    const std::size_t index_before = f.windowIndex();
    const int consumed_before = f.windowConsumed();
    const int segments_before = f.diag().segments;

    Pose6D ref;
    ref.x = -0.04;
    ref.y = 0.015;
    ref.z = 0.18;
    const double ang = 0.35;
    ref.quaternion_xyzw =
        std::array<double, 4>{0.0, 0.0, std::sin(ang / 2), std::cos(ang / 2)};

    f.reanchor(ref);
    check(f.active(), "active stays true immediately after reanchor");
    check(f.windowWireSeq() == wire_before && f.windowRecvSeq() == recv_before,
          "reanchor keeps active window seq ids");
    check(f.windowIndex() == index_before && f.windowConsumed() == consumed_before,
          "reanchor keeps window consume pointer");
    check(poseNear(f.lastPose(), ref, 1e-12, 1e-12), "lastPose equals reference immediately");

    const Pose6D first = f.tick(TICK);
    check(poseNear(first, ref, 1e-9, 1e-9), "next tick starts at reanchor reference");
    check(f.diag().segments == segments_before + 1, "reanchor forces a fresh segment solve");
    check(f.diag().seg_step_index == static_cast<int>(index_before),
          "fresh segment uses unchanged window index");
  }

  // -- Test 6: corner sign deadbands ignore encoder-noise reversals. ----------
  std::printf("Test 6: corner deadband\n");
  {
    CartesianChunkFollowerConfig deadband_cfg;
    deadband_cfg.window = {/*L*/ 1, /*C*/ 3, /*R*/ 1, /*smooth*/ 1};

    bool solved = false;
    const bool linear_sub =
        firstSegmentCorner(deadband_cfg, makeLinearReverseFrame(2.0e-4), &solved);
    check(solved, "linear sub-threshold reversal segment solves");
    check(!linear_sub, "linear reversal below 3e-4 m is not a corner");

    const bool linear_supra =
        firstSegmentCorner(deadband_cfg, makeLinearReverseFrame(4.0e-4), &solved);
    check(solved, "linear supra-threshold reversal segment solves");
    check(linear_supra, "linear reversal above 3e-4 m is a corner");

    const bool angular_sub =
        firstSegmentCorner(deadband_cfg, makeAngularReverseFrame(4.0e-4), &solved);
    check(solved, "angular sub-threshold reversal segment solves");
    check(!angular_sub, "angular reversal below 5e-4 rad is not a corner");

    const bool angular_supra =
        firstSegmentCorner(deadband_cfg, makeAngularReverseFrame(6.0e-4), &solved);
    check(solved, "angular supra-threshold reversal segment solves");
    check(angular_supra, "angular reversal above 5e-4 rad is a corner");

    // The deadbands are config-driven, not hard-coded: widening the angular
    // deadband must reclassify a reversal that the default would have flagged.
    // This is the knob for the measured rotational-noise firing rate (2026-07-31:
    // rotation axes alone tripped the guard on 17-38% of segments).
    CartesianChunkFollowerConfig wide_cfg = deadband_cfg;
    wide_cfg.guard.corner_deadband_ang_rad = 5.0e-3;   // 0.29 deg
    const bool angular_widened =
        firstSegmentCorner(wide_cfg, makeAngularReverseFrame(6.0e-4), &solved);
    check(solved, "widened-deadband reversal segment solves");
    check(!angular_widened,
          "widening corner_deadband_ang_rad reclassifies the same reversal as noise");

    CartesianChunkFollowerConfig tight_cfg = deadband_cfg;
    tight_cfg.guard.corner_deadband_lin_m = 1.0e-4;    // 0.1 mm
    const bool linear_tightened =
        firstSegmentCorner(tight_cfg, makeLinearReverseFrame(2.0e-4), &solved);
    check(solved, "tightened-deadband reversal segment solves");
    check(linear_tightened,
          "tightening corner_deadband_lin_m promotes a sub-default reversal to a corner");
  }

  // -- Test 6b: corner_velocity_scale is the configured ring-down. -------------
  std::printf("Test 6b: corner_velocity_scale\n");
  {
    // The scale multiplies the TARGET velocity, i.e. the speed the segment carries
    // at its far end -- so probe the last tick of the corner segment, not the first
    // (the segment starts from the seeded zero-velocity state either way).
    // makeLinearReverseFrame is SYMMETRIC (d_k = -d_kp1), so its central-difference
    // vf is exactly zero and no scale factor is observable. Use an ASYMMETRIC
    // reversal (+4 mm then -3 mm) so vf != 0 while both flanks clear the deadband.
    ChunkFrame asym;
    asym.policy_dt = SEG;
    asym.pose = {poseWithXAndYaw(0.0, 0.0), poseWithXAndYaw(4.0e-3, 0.0),
                 poseWithXAndYaw(1.0e-3, 0.0), poseWithXAndYaw(1.0e-3, 0.0),
                 poseWithXAndYaw(1.0e-3, 0.0), poseWithXAndYaw(1.0e-3, 0.0)};
    asym.grip.assign(asym.pose.size(), 20.0);

    auto exitSpeed = [&asym](double scale) {
      CartesianChunkFollowerConfig c;
      c.window = {/*L*/ 1, /*C*/ 3, /*R*/ 1, /*smooth*/ 1};
      c.guard.corner_velocity_scale = scale;
      const ChunkFrame& frame = asym;
      CartesianChunkFollower f(c);
      f.submitFrame(frame, frame.pose[static_cast<std::size_t>(c.window.discard_head_L)]);
      const int ticks = static_cast<int>(SEG / TICK);   // one full segment
      Pose6D prev{}, cur{};
      for (int i = 0; i < ticks; ++i) {
        prev = cur;
        cur = f.tick(TICK);
      }
      return std::hypot(cur.x - prev.x, cur.y - prev.y) / TICK;
    };
    const double v_hard = exitSpeed(0.0);      // full stop at the reversal
    const double v_default = exitSpeed(0.25);
    const double v_soft = exitSpeed(1.0);      // no velocity cut
    std::printf("    corner exit speed: scale0=%.6f scale0.25=%.6f scale1=%.6f m/s\n",
                v_hard, v_default, v_soft);
    check(v_soft > v_default, "corner_velocity_scale=1.0 exits the corner faster than 0.25");
    check(v_default > v_hard, "corner_velocity_scale=0.25 exits faster than a full stop");
  }

  // -- Test 7: delta-preview integrates local rows and faults on persistent slip.
  std::printf("Test 7: delta-preview integration + projection guard\n");
  {
    CartesianChunkFollowerConfig preview_cfg;
    preview_cfg.window = {/*L*/ 0, /*C*/ 3, /*R*/ 1, /*smooth*/ 1};
    preview_cfg.max_projection_error_m = 1e-6;
    preview_cfg.max_projection_error_rad = 1e-6;
    preview_cfg.max_consecutive_projection_errors = 2;
    preview_cfg.max_actual_lead_m = 0.001;
    preview_cfg.max_actual_lead_rad = 0.001;
    preview_cfg.max_consecutive_actual_lead_errors = 2;
    CartesianChunkFollower f(preview_cfg);
    ChunkFrame frame;
    frame.policy_dt = 1.0 / 30.0;
    frame.wire_seq = 909;
    frame.recv_seq = 44;
    for (int i = 0; i < 4; ++i) {
      frame.delta.push_back(Vec6{0.01, 0.0, 0.0, 0.0, 0.0, 0.0});
      frame.grip.push_back(20.0 + i);
    }
    Pose6D start;
    start.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
    f.submitDeltaFrame(frame, start);
    check(f.active(), "delta frame activates the shared Ruckig follower");
    for (int i = 0; i < 40; ++i) f.tick(TICK);
    check(f.diag().seg_target_stand.x > 0.02,
          "local delta rows integrate into cumulative forward absolute knots");
    check(f.diag().infeasible_fault, "persistent projection error reaches configured fault count");

    Pose6D far_actual = start;
    far_actual.x = -1.0;
    f.updateActualLead(far_actual);
    for (int i = 0; i < 20; ++i) f.tick(TICK);
    f.updateActualLead(far_actual);
    check(f.diag().actual_lead_fault, "actual lead reaches configured policy-frame fault count");
  }

  // -- Test: chunk-swap head behaviour (fix + characterization). --------------
  // (a) FIXED: flankIndex(k,-1,n) clamps to k at the window head, so d_k used to be
  //     0 there -- halving the central-difference vf, injecting a spurious
  //     af = +d_kp1/dt^2, and zeroing sign_dk so the corner test could never fire.
  //     buildSample now falls back to the forward difference at the head.
  // (b) FIXED (was: 0.33-0.54x travel loss for ~3 segments after every swap, with
  //     last_solve.duration/dt spiking to ~5 and the motion transiently reversing, on a
  //     PERFECTLY constant-velocity delta stream). Root cause: submitDeltaFrame anchored
  //     the integration at last_pose_ (the MID-segment emitted sample) while the next
  //     solve starts from the Ruckig chained state p0_, which solve() leaves at the END
  //     of the current segment -- so the first new knot was only
  //     d - d_prev*(1 - t_in_seg/seg_dt) ahead of the solve's start, i.e. ~0 (or negative)
  //     when the frame landed early in a segment. Now anchored at core_.p0().
  //     Hardware corroboration of the pre-fix behaviour:
  //     logs/servo_log_20260724_160210.csv emitted follower speed by consumed row
  //     index (right arm) = 20.4 / 27.6 / 42.4 / 47.9 / 45.9 mm/s, and
  //     outputs/sweep/20260729_0840*_fixedstep_command.jsonl showed a 0.70x displacement
  //     notch one row after each swap (n~880/row).
  std::printf("Test: chunk-swap head (fix + known-defect characterization)\n");
  {
    CartesianChunkFollowerConfig hcfg;
    hcfg.window = {/*L*/ 0, /*C*/ 10, /*R*/ 2, /*smooth*/ 1};
    const double vx = 0.03;  // constant speed -> zero reference curvature
    const double step = vx * SEG;
    auto makeDeltaFrame = [&](int n, std::uint64_t wseq) {
      ChunkFrame fr;
      fr.policy_dt = SEG;
      fr.wire_seq = wseq;
      fr.recv_seq = 20;
      fr.recv_time = 0.0;
      for (int i = 0; i < n; ++i) {
        Vec6 d{};
        d.x = step;
        fr.delta.push_back(d);
        fr.grip.push_back(0.0);
      }
      return fr;
    };

    CartesianChunkFollower f(hcfg);
    Pose6D origin;
    origin.x = 0.0; origin.y = 0.0; origin.z = 0.2;
    origin.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
    f.submitDeltaFrame(makeDeltaFrame(40, 55), origin);
    for (int i = 0; i < 250; ++i) f.tick(TICK);

    const int n3 = static_cast<int>(3 * SEG / TICK);
    Pose6D a = f.tick(TICK);
    Pose6D b = a;
    for (int i = 0; i < n3; ++i) b = f.tick(TICK);
    const double steady = b.x - a.x;
    check(steady > 0.8 * 3 * step, "steady state tracks the constant delta stream");

    const Pose6D at = f.tick(TICK);
    f.submitDeltaFrame(makeDeltaFrame(40, 56), at);
    const bool head_corner = f.diag().last_solve.corner;
    Pose6D c = at;
    for (int i = 0; i < n3; ++i) c = f.tick(TICK);
    const double swap = c.x - at.x;
    const double ratio = steady > 1e-12 ? swap / steady : 0.0;
    std::printf("    3-seg travel: steady=%.6f m  across-swap=%.6f m  ratio=%.3f\n",
                steady, swap, ratio);
    check(!head_corner, "fresh-chunk head reports no phantom corner (flank-clamp fix)");
    check(ratio > 0.85, "chunk swap preserves travel on a constant-velocity delta stream");

    // (c) The defect was PHASE-dependent: submitting early in a segment lost the most
    //     travel, so a single submit phase can pass by luck. Sweep the whole segment and
    //     require every phase to keep the travel AND never reverse.
    double worst = 1.0;
    int worst_phase = -1;
    const int ticks_per_seg = static_cast<int>(SEG / TICK);
    for (int phase = 0; phase < ticks_per_seg; ++phase) {
      CartesianChunkFollower g(hcfg);
      g.submitDeltaFrame(makeDeltaFrame(40, 55), origin);
      for (int i = 0; i < 250; ++i) g.tick(TICK);
      // measure this instance's own steady rate over 3 segments
      Pose6D s0 = g.tick(TICK);
      Pose6D s1 = s0;
      for (int i = 0; i < n3; ++i) s1 = g.tick(TICK);
      const double steady_i = s1.x - s0.x;
      // land the swap `phase` ticks into a segment
      Pose6D at_i = s1;
      for (int i = 0; i < phase; ++i) at_i = g.tick(TICK);
      g.submitDeltaFrame(makeDeltaFrame(40, 56), at_i);
      Pose6D prev = at_i, cur = at_i;
      double min_tick_dx = 1.0;
      for (int i = 0; i < n3; ++i) {
        cur = g.tick(TICK);
        min_tick_dx = std::min(min_tick_dx, cur.x - prev.x);
        prev = cur;
      }
      const double r = steady_i > 1e-12 ? (cur.x - at_i.x) / steady_i : 0.0;
      if (r < worst) { worst = r; worst_phase = phase; }
      if (min_tick_dx < -1e-9) {
        std::printf("    phase %d/%d REVERSED (min per-tick dx=%.3e m)\n",
                    phase, ticks_per_seg, min_tick_dx);
        check(false, "chunk swap never reverses the motion");
        break;
      }
    }
    std::printf("    worst-phase ratio over %d submit phases: %.3f (phase %d)\n",
                ticks_per_seg, worst, worst_phase);
    check(worst > 0.85, "chunk-swap travel is preserved at EVERY submit phase");
  }

  // -- Test: a held robot must not be outrun by the plan. --------------------
  // The floor/ROI/self-collision stage clamps an arm to its previous sent joints, but it runs
  // AFTER command generation, so the follower used to keep integrating deltas against a robot
  // that was standing still. Measured on servo_log_20260729_165037.csv: eight RoiViolation
  // episodes (right gripper tip crossing roi_box y=-0.150) grew actual_lead from 1.3 mm to
  // 40.7 mm / 4.5 deg and ended the rollout in delta_preview_actual_lead_fault -- and each time
  // the verdict flipped back to Ok the arm lunged to close that gap. This locks the invariant
  // that the pause actually bounds the divergence.
  // NOTE (2026-08-26): the CALLER changed. dual_arm_servo_loop used to pause on
  // safetyInterventionRecent(), i.e. on any velocity-damper projection. That froze and
  // warm-resumed the plan six times in 1.8 s on a barrier that was merely SLOWING the arm
  // (servo_log_20260826_065624.csv, left arm), and the resulting ~5 Hz plan discontinuity
  // was itself the shake. The pause is now driven only by cartesianSolveBlockedRecent(),
  // i.e. an arm actually HELD at prev_sent; a damped-but-moving arm keeps its plan and its
  // lead is bounded by the re-anchor instead. The follower-level invariant tested here is
  // unchanged -- only who calls it.

  // -- Test: a held robot must not be outrun by the plan. --------------------
  // The floor/ROI/self-collision stage clamps an arm to its previous sent joints, but it runs
  // AFTER command generation, so the follower used to keep integrating deltas against a robot
  // that was standing still. Measured on servo_log_20260729_165037.csv: eight RoiViolation
  // episodes (right gripper tip crossing roi_box y=-0.150) grew actual_lead from 1.3 mm to
  // 40.7 mm / 4.5 deg and ended the rollout in delta_preview_actual_lead_fault -- and each time
  // the verdict flipped back to Ok the arm lunged to close that gap. This locks the invariant
  // that the pause actually bounds the divergence.
  // NOTE (2026-08-26): the CALLER changed. dual_arm_servo_loop used to pause on
  // safetyInterventionRecent(), i.e. on any velocity-damper projection. That froze and
  // warm-resumed the plan six times in 1.8 s on a barrier that was merely SLOWING the arm
  // (servo_log_20260826_065624.csv, left arm), and the resulting ~5 Hz plan discontinuity
  // was itself the shake. The pause is now driven only by cartesianSolveBlockedRecent(),
  // i.e. an arm actually HELD at prev_sent; a damped-but-moving arm keeps its plan and its
  // lead is bounded by the re-anchor instead. The follower-level invariant tested here is
  // unchanged -- only who calls it.

  // -- Test: lead re-anchor rate budget ---------------------------------------
  // A lead breach RE-ANCHORS the plan (which removes the lead, and with it the catch-up
  // lunge that used to follow) and only latches once re-anchoring has demonstrably
  // stopped helping. Measured 2026-08-26 on servo_log_20260826_070506.csv: BOTH lead
  // budgets are already saturated -- right-arm angular lead p99.9 = 0.0855 rad of a
  // 0.0873 limit (98%), left-arm positional peak 0.0354 m of 0.035 (101%) -- so raising a
  // threshold only moves the latch to the next gate. Recovering, and giving up only on
  // repetition, is what converges. That same run produced ONE legitimate re-anchor in
  // 122 s, which is how 5-in-2-s was sized.
  std::printf("Test: lead re-anchor rate budget\n");
  {
    constexpr std::size_t CAP = 32;
    std::uint64_t ring[CAP] = {};
    std::size_t head = 0;
    const std::uint64_t SEC = 1000000000ULL;
    const std::uint64_t t0 = 1000 * SEC;
    const double window = 2.0;
    const int budget = 5;
    // `budget` recoveries inside the window are allowed; the next one spends it.
    for (int n = 1; n <= budget; ++n) {
          check(!rb_servo::control::recordAndCheckRateBudget(ring, CAP, head, t0 + n * SEC / 10, window, budget), "lead re-anchor budget");
    }
        check(rb_servo::control::recordAndCheckRateBudget(ring, CAP, head, t0 + 6 * SEC / 10, window, budget), "lead re-anchor budget");
    // RATE limit, not a lifetime count: once the window has passed the budget is back, so
    // a long healthy run never accumulates its way into a latch.
        check(!rb_servo::control::recordAndCheckRateBudget(ring, CAP, head, t0 + 60 * SEC, window, budget), "lead re-anchor budget");
    // A separate ring is a separate budget -- one arm's trouble must not latch the other.
    std::uint64_t other[CAP] = {};
    std::size_t other_head = 0;
        check(!rb_servo::control::recordAndCheckRateBudget(other, CAP, other_head, t0 + 6 * SEC / 10, window, budget), "lead re-anchor budget");
    // Disabled either way => legacy latch-on-first-breach.
    std::uint64_t off[CAP] = {};
    std::size_t off_head = 0;
        check(rb_servo::control::recordAndCheckRateBudget(off, CAP, off_head, t0, 0.0, budget), "lead re-anchor budget");
        check(rb_servo::control::recordAndCheckRateBudget(off, CAP, off_head, t0, window, 0), "lead re-anchor budget");
        check(rb_servo::control::recordAndCheckRateBudget(nullptr, 0, off_head, t0, window, budget), "lead re-anchor budget");
  }

  std::printf("Test: safety-hold pause bounds plan-vs-actual divergence\n");
  {
    CartesianChunkFollowerConfig hcfg;
    hcfg.window = {/*L*/ 0, /*C*/ 10, /*R*/ 2, /*smooth*/ 1};
    const double vx = 0.03;
    const double step = vx * SEG;
    auto deltaFrame = [&](int n, std::uint64_t wseq) {
      ChunkFrame fr;
      fr.policy_dt = SEG;
      fr.wire_seq = wseq;
      fr.recv_seq = 20;
      fr.recv_time = 0.0;
      for (int i = 0; i < n; ++i) {
        Vec6 d{};
        d.x = step;
        fr.delta.push_back(d);
        fr.grip.push_back(0.0);
      }
      return fr;
    };
    Pose6D origin;
    origin.x = 0.0; origin.y = 0.0; origin.z = 0.2;
    origin.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};

    // Drive both instances identically, then stall the "robot" at a fixed pose for 300 ms
    // (the longest measured RoiViolation episode was 92 ms).
    const int stalled_ticks = static_cast<int>(0.300 / TICK);
    double lead_running = 0.0, lead_paused = 0.0;
    for (int paused = 0; paused < 2; ++paused) {
      CartesianChunkFollower f(hcfg);
      f.submitDeltaFrame(deltaFrame(60, 55), origin);
      for (int i = 0; i < 200; ++i) f.tick(TICK);
      const Pose6D stuck = f.tick(TICK);  // the pose the clamped robot is frozen at
      if (paused) f.pauseForHold(100.0);
      for (int i = 0; i < stalled_ticks; ++i) {
        f.tick(TICK);
        f.updateActualLead(stuck);  // robot is held by the safety layer: actual never moves
      }
      (paused ? lead_paused : lead_running) = f.diag().actual_lead_m;
    }
    std::printf("    lead after 300 ms held: running=%.1f mm  paused=%.1f mm\n",
                lead_running * 1000.0, lead_paused * 1000.0);
    check(lead_running > 0.005, "unpaused follower does outrun a held robot (defect reproduced)");
    check(lead_paused < 1e-9, "safety-hold pause freezes the plan at the held pose");
  }

  // -- Measurement: synthetic streams do NOT reproduce the hardware head effect. -------
  // Hardware (servo_log_20260729_184823.csv, right arm, n~7300/row) shows the FIRST segment of
  // each new chunk missing its one-policy-step deadline 34% of the time, decaying monotonically
  // 34/22/19/16/12/6% over rows 0..5 (duration/seg_dt p90 1.76 -> 1.00). Position is continuous
  // across the swap (the anchor fix above); the velocity/acceleration boundary state is not.
  //
  // NEGATIVE RESULT, kept so nobody re-derives it: neither a smoothly-continued curved stream
  // nor one with injected replan disagreement reproduces that HEAD CONCENTRATION. Sweeping both
  // curvature and mismatch, every operating point degrades all rows EQUALLY (e.g. at 42% the
  // split is r0..r4 = 42/42/42/42/42), with a sharp cliff between feasible and saturated. So a
  // synthetic chunk cannot validate a change to the head boundary condition (e.g. seeding af
  // from the forward second difference instead of 0) -- that needs REAL recorded chunks replayed
  // through the follower. Capture them with FLOW_INFER_PRINT_CHUNK=1.
  std::printf("Test: chunk-swap feasibility is uniform on synthetic streams (negative result)\n");
  {
    CartesianChunkFollowerConfig hcfg;
    hcfg.window = {/*L*/ 0, /*C*/ 10, /*R*/ 2, /*smooth*/ 1};
    const double step_m = 0.03 * SEG;
    const double dphi = 0.03;
    const double mismatch = 0.065;   // the operating point whose overall rate is closest to hw
    const int EXECUTE = 5;
    auto curved = [&](int n, double phase0, std::uint64_t wseq) {
      ChunkFrame fr;
      fr.policy_dt = SEG;
      fr.wire_seq = wseq;
      fr.recv_seq = 20;
      fr.recv_time = 0.0;
      for (int i = 0; i < n; ++i) {
        const double ph = phase0 + i * dphi;
        Vec6 d{};
        d.x = step_m * std::cos(ph);
        d.y = step_m * std::sin(ph);
        fr.delta.push_back(d);
        fr.grip.push_back(0.0);
      }
      return fr;
    };
    CartesianChunkFollower f(hcfg);
    Pose6D origin;
    origin.x = 0.0; origin.y = 0.0; origin.z = 0.2;
    origin.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
    std::vector<std::vector<double>> ratio_by_row(EXECUTE + 1);
    int segments_seen = f.diag().segments;
    double phase = 0.0;
    std::uint64_t seq = 55;
    f.submitDeltaFrame(curved(40, phase, seq), origin);
    const int ticks_per_seg = static_cast<int>(SEG / TICK);
    for (int chunk = 0; chunk < 400; ++chunk) {
      for (int s = 0; s < EXECUTE; ++s) {
        for (int t = 0; t < ticks_per_seg; ++t) {
          f.tick(TICK);
          if (f.diag().segments != segments_seen) {
            segments_seen = f.diag().segments;
            const int row = f.diag().seg_step_index;
            if (row >= 0 && row <= EXECUTE && chunk > 2) {
              ratio_by_row[row].push_back(f.diag().last_solve.duration / SEG);
            }
          }
        }
      }
      phase += EXECUTE * dphi + mismatch * std::sin(2.399963 * chunk);
      f.submitDeltaFrame(curved(40, phase, ++seq), f.lastPose());
    }
    auto over105 = [](const std::vector<double>& v) {
      if (v.empty()) return 0.0;
      return 100.0 * static_cast<double>(std::count_if(
          v.begin(), v.end(), [](double x) { return x > 1.05; })) / v.size();
    };
    std::printf("    row>1.05%%:");
    double head = 0.0, tail_max = 0.0;
    for (int r = 0; r <= EXECUTE; ++r) {
      if (ratio_by_row[r].size() < 20) continue;
      const double o = over105(ratio_by_row[r]);
      std::printf(" r%d=%.0f", r, o);
      if (r == 0) head = o; else tail_max = std::max(tail_max, o);
    }
    std::printf("   (hardware for contrast: 34/22/19/16/12/6)\n");
    check(head > 5.0, "synthetic swap does stress the solver");
    check(std::fabs(head - tail_max) < 10.0,
          "synthetic difficulty is uniform across rows, unlike hardware");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
