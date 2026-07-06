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

using rb_servo::control::CartesianChunkFollower;
using rb_servo::control::CartesianChunkFollowerConfig;
using rb_servo::control::ChunkFrame;
using rb_servo::Pose6D;

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

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
