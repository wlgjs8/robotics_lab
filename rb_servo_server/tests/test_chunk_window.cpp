// test_chunk_window.cpp — L/C/R bookkeeping, invariant clamp, smoothing.

#include "rb_servo/control/chunk_window.hpp"

#include <array>
#include <cmath>
#include <cstdio>

using rb_servo::control::ChunkFrame;
using rb_servo::control::ChunkWindow;
using rb_servo::control::ChunkWindowConfig;
using rb_servo::Pose6D;

static int g_failures = 0;
static void check(bool ok, const char* name) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

static ChunkFrame makeFrame(int n, double step = 0.01) {
  ChunkFrame f;
  f.policy_dt = 1.0 / 30.0;
  f.wire_seq = 55;
  f.recv_seq = 20;
  f.recv_time = 0.0;
  for (int i = 0; i < n; ++i) {
    Pose6D p;
    p.x = step * i;
    p.y = 0.0;
    p.z = 0.2;
    p.quaternion_xyzw = std::array<double, 4>{0, 0, 0, 1};
    f.pose.push_back(p);
    f.grip.push_back(static_cast<double>(i));
  }
  return f;
}

int main() {
  // -- Test 0: the shipped consume ceiling covers BOTH execute settings. ------
  // policy_runner publishes n = min(EXECUTE_STEPS + RUNWAY_STEPS, horizon), and
  // activate() clamps consume to what the frame can supply:
  // c_eff = min(consume_steps, n - L - R). stack_real ships consume 8 /
  // reserve 4 / discard_head 0, so one value serves EXECUTE_STEPS 4 and 8 and
  // an A/B needs no config edit. At the previous consume 5 an 8-step execution
  // window was silently capped at 5 -- measured 2026-08-27, where the policy
  // predicted 24 steps, only rows 0-3 ever ran, and the right arm's per-step
  // motion GREW across that window (4.58 -> 5.19 mm): the depth was in the rows
  // being cut off.
  std::printf("Test 0: shipped consume ceiling vs EXECUTE_STEPS\n");
  {
    const ChunkWindowConfig shipped{/*L*/ 0, /*C*/ 8, /*R*/ 4, /*smooth*/ 1};
    ChunkWindow four(shipped);
    check(four.activate(makeFrame(4 + 4)), "EXECUTE_STEPS=4 -> n=8 activates");
    check(four.consumeBudget() == 4, "n=8, R=4 -> consume clamped to 4");

    ChunkWindow eight(shipped);
    check(eight.activate(makeFrame(8 + 4)), "EXECUTE_STEPS=8 -> n=12 activates");
    check(eight.consumeBudget() == 8, "n=12, R=4 -> the full 8 is consumed");
    int steps = 0;
    while (eight.hasStep()) { eight.advance(); ++steps; }
    check(steps == 8, "all 8 execution rows really are consumed");

    // The old ceiling is what capped the 8-step window.
    ChunkWindow old(ChunkWindowConfig{/*L*/ 0, /*C*/ 5, /*R*/ 4, /*smooth*/ 1});
    check(old.activate(makeFrame(8 + 4)), "n=12 activates at the old ceiling too");
    check(old.consumeBudget() == 5, "consume 5 caps an 8-step execution window");
  }

  // -- Test 1: nominal window, head discard, consume budget, exhaustion. ------
  std::printf("Test 1: nominal L/C/R\n");
  {
    ChunkWindow w(ChunkWindowConfig{/*L*/ 6, /*C*/ 8, /*R*/ 2, /*smooth*/ 1});
    check(w.activate(makeFrame(16)), "activate a 16-step frame");
    check(w.wireSeq() == 55 && w.recvSeq() == 20, "wire/recv seqs preserved distinctly");
    check(w.consumeBudget() == 8, "C not clamped (6+8+2=16)");
    check(w.index() == 6, "consume pointer starts at L=6 (head discarded)");
    int steps = 0;
    while (w.hasStep()) { w.advance(); ++steps; }
    check(steps == 8, "exactly C=8 steps consumed");
    check(w.windowExhausted(), "window exhausted after C steps");
  }

  // -- Test 2: invariant clamp L + C + R <= horizon. --------------------------
  std::printf("Test 2: invariant clamp\n");
  {
    ChunkWindow w(ChunkWindowConfig{/*L*/ 8, /*C*/ 16, /*R*/ 2, /*smooth*/ 1});
    check(w.activate(makeFrame(16)), "activate");
    check(w.consumeBudget() == 6, "C clamped to horizon-L-R = 6 (was 16)");
  }

  // -- Test 3: too-short frame rejected. --------------------------------------
  std::printf("Test 3: too-short frame\n");
  {
    ChunkWindow w(ChunkWindowConfig{/*L*/ 6, /*C*/ 8, /*R*/ 2, /*smooth*/ 1});
    check(!w.activate(makeFrame(5)), "5-step frame rejected (no room for L+R+1)");
    check(!w.active(), "window inactive after rejected activate");
  }

  // -- Test 4: smoothing pulls a spike toward its neighbors. ------------------
  std::printf("Test 4: smoothing\n");
  {
    ChunkFrame f = makeFrame(16, 0.0);  // all x=0 ...
    f.pose[8].x = 0.10;                  // ... except a spike at index 8
    ChunkWindow w(ChunkWindowConfig{/*L*/ 2, /*C*/ 8, /*R*/ 2, /*smooth*/ 3});
    check(w.activate(f), "activate spiky frame");
    const double sx = w.poseAt(8).x;
    check(sx < 0.10 - 1e-6 && sx > 0.0, "window-3 average pulls the spike down (0<x<0.10)");
    check(std::fabs(sx - 0.10 / 3.0) < 1e-9, "3-neighbor mean = spike/3");
  }

  // -- Test 5: poseAt clamps out-of-range indices. ---------------------------
  std::printf("Test 5: poseAt clamp\n");
  {
    ChunkWindow w(ChunkWindowConfig{/*L*/ 2, /*C*/ 8, /*R*/ 2, /*smooth*/ 1});
    w.activate(makeFrame(16, 0.01));
    check(std::fabs(w.poseAt(999).x - w.poseAt(15).x) < 1e-12, "index above N clamps to last");
    check(w.gripAt(999) == 15.0, "gripAt clamps to last");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
