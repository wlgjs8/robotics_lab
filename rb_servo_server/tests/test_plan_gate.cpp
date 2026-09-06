// test_plan_gate.cpp — the safety plan-gate law: what closes it, what must not,
// and how fast it moves. See include/rb_servo/control/plan_gate.hpp for the
// measured rationale behind each of these.

#include "rb_servo/control/plan_gate.hpp"

#include <cmath>
#include <cstdio>

using rb_servo::JointArray;
using rb_servo::SafetyPlanGateConfig;
using rb_servo::control::planGateStep;
using rb_servo::control::ikThrottlePlanGateStep;
using rb_servo::control::lowpassEngagementStep;
using rb_servo::control::planLeashGate;
using rb_servo::control::PlanLeashParams;
using rb_servo::SafetyPlanGateConfig;

static int g_failures = 0;
static void check(bool ok, const char* name) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

static SafetyPlanGateConfig shipped() {
  SafetyPlanGateConfig c;  // stack_real.yaml values
  c.enable = true;
  c.release_alpha = 0.02;
  c.attack_alpha = 0.1;
  c.deadband_deg = 0.05;
  c.min_gate = 0.0;
  return c;
}

// A 0.1 deg step on one joint: comfortably above the deadband, and the size of a
// real streaming step (0.1 deg / 2 ms = 50 deg/s).
static JointArray zeros() { return JointArray{0, 0, 0, 0, 0, 0}; }

// -- Test L: the divergence leash (2026-09-06). ------------------------------
static void testPlanLeash() {
  std::printf("Test L: divergence leash ramps the plan clock, never stops it\n");
  PlanLeashParams p;  // stack_real.yaml values
  p.start_m = 0.010; p.start_rad = 0.0349; p.full_m = 0.050; p.full_rad = 0.10; p.min_gate = 0.25;
  check(planLeashGate(0.0, 0.0, p) == 1.0, "no divergence -> 1.0");
  check(planLeashGate(0.010, 0.0349, p) == 1.0, "at start -> still 1.0");
  const double mid = planLeashGate(0.030, 0.0, p);
  check(std::fabs(mid - 0.5) < 1e-9, "halfway between start and full -> 0.5");
  check(std::fabs(planLeashGate(0.050, 0.0, p) - 0.25) < 1e-9, "at full -> min_gate");
  check(std::fabs(planLeashGate(0.500, 0.0, p) - 0.25) < 1e-9, "far beyond full -> min_gate, never 0");
  check(std::fabs(planLeashGate(0.0, 0.10, p) - 0.25) < 1e-9, "orientation alone reaches min_gate");
  check(std::fabs(planLeashGate(0.030, 0.06745, p) - 0.5) < 1e-6, "both axes halfway -> 0.5 (min of the two)");
  check(std::fabs(planLeashGate(0.012, 0.09, p) - planLeashGate(0.0, 0.09, p)) < 1e-12,
        "the more restrictive axis wins");
  bool mono = true;
  double prev = 1.0;
  for (double e = 0.0; e < 0.08; e += 0.0005) {
    const double g = planLeashGate(e, 0.0, p);
    mono &= g <= prev + 1e-12;
    prev = g;
  }
  check(mono, "monotone non-increasing in divergence");
  PlanLeashParams degenerate = p;
  degenerate.full_m = degenerate.start_m;
  check(planLeashGate(0.011, 0.0, degenerate) == 0.25 && planLeashGate(0.009, 0.0, degenerate) == 1.0,
        "a degenerate ramp (full == start) steps at start");
}

int main() {
  testPlanLeash();
  const SafetyPlanGateConfig cfg = shipped();

  // -- Test 1: nothing removed -> the gate opens, never closes. ---------------
  std::printf("Test 1: unobstructed motion never closes the gate\n");
  {
    const JointArray prev = zeros();
    const JointArray req{0.10, 0, 0, 0, 0, 0};
    double g = 1.0;
    for (int i = 0; i < 500; ++i) g = planGateStep(g, req, req, prev, cfg);
    check(g == 1.0, "requested == final holds the gate at 1.0");

    g = 0.2;  // recovering from an earlier obstruction
    const double after_one = planGateStep(g, req, req, prev, cfg);
    check(after_one > g, "a recovering gate releases upward");
    for (int i = 0; i < 500; ++i) g = planGateStep(g, req, req, prev, cfg);
    check(g > 0.999, "release converges to 1.0");
  }

  // -- Test 2: THE FIX. A clamped joint alone must not close the gate. --------
  // This is the J3-approach-barrier case that cost the left arm 9.42 s of plan
  // time: the barrier parks one joint at its standoff, so on that joint
  // final == prev forever. The caller now passes the post-clamp target as
  // `requested_q`, so requested == final and the gate stays open. The test
  // states the contract at the law's own boundary: identical inputs, open gate,
  // no matter how hard the clamp is holding.
  std::printf("Test 2: a held joint does not close the gate (barrier case)\n");
  {
    const JointArray prev{0, 0, 149.90, 0, 0, 0};
    // Post-clamp target: J3 pinned at the standoff, the other joints still moving.
    const JointArray post_clamp{0.08, 0.06, 149.90, 0.05, 0, 0};
    double g = 1.0;
    for (int i = 0; i < 200; ++i) {
      g = planGateStep(g, post_clamp, post_clamp, prev, cfg);
    }
    check(g == 1.0, "J3 pinned at the 149.90 standoff leaves the gate wide open");

    // And the regression it replaces: had the PRE-clamp target been passed (J3
    // still driving past the bound), the gate would have closed on it.
    const JointArray pre_clamp{0.08, 0.06, 150.40, 0.05, 0, 0};
    double g_old = 1.0;
    for (int i = 0; i < 200; ++i) {
      g_old = planGateStep(g_old, pre_clamp, post_clamp, prev, cfg);
    }
    check(g_old < 0.6, "the old pre-clamp input would have throttled the plan hard");
  }

  // -- Test 3: a real obstruction still closes it. ----------------------------
  std::printf("Test 3: an obstruction still closes the gate\n");
  {
    const JointArray prev = zeros();
    const JointArray req{0.10, 0, 0, 0, 0, 0};
    const JointArray blocked = zeros();          // projection removed all of it
    double g = 1.0;
    for (int i = 0; i < 200; ++i) g = planGateStep(g, req, blocked, prev, cfg);
    check(g < 0.001, "a fully blocked step drives the gate to ~0 (plan freezes)");

    const JointArray half{0.05, 0, 0, 0, 0, 0};  // half removed
    double h = 1.0;
    for (int i = 0; i < 300; ++i) h = planGateStep(h, req, half, prev, cfg);
    check(std::fabs(h - 0.5) < 0.01, "a half-removed step settles the gate at ~0.5");
  }

  // -- Test 4: THE FIX. Attack is rate-limited, not a one-tick step. ----------
  // The old law did `if (instant < g) g = instant`, so one tick could cut the
  // plan clock of all six joints by 88% (measured max |dgate|/tick = 0.88).
  std::printf("Test 4: attack is rate-limited\n");
  {
    const JointArray prev = zeros();
    const JointArray req{0.10, 0, 0, 0, 0, 0};
    const JointArray blocked = zeros();
    double g = 1.0;
    double worst_step = 0.0;
    for (int i = 0; i < 200; ++i) {
      const double next = planGateStep(g, req, blocked, prev, cfg);
      worst_step = std::max(worst_step, std::fabs(next - g));
      g = next;
    }
    check(worst_step <= cfg.attack_alpha + 1e-12,
          "no single tick moves the gate by more than attack_alpha");
    check(worst_step < 0.11, "worst tick is 0.1, not the old 0.88");

    // Still fast enough to matter: 63% closed in ~20 ms, 95% in ~60 ms at 500 Hz.
    double f = 1.0;
    for (int i = 0; i < 10; ++i) f = planGateStep(f, req, blocked, prev, cfg);
    check(f < 0.37, "63% closed within 10 ticks (20 ms)");
    for (int i = 0; i < 20; ++i) f = planGateStep(f, req, blocked, prev, cfg);
    check(f < 0.05, "95% closed within 30 ticks (60 ms)");
  }

  // -- Test 5: THE FIX. The ratio is directional, not two independent maxima. --
  std::printf("Test 5: directional ratio, not cross-joint maxima\n");
  {
    const JointArray prev = zeros();
    // J1 requests the largest step; J5 requests a smaller one and keeps it.
    const JointArray req{0.10, 0, 0, 0, 0, 0.04};
    const JointArray got{0.00, 0, 0, 0, 0, 0.04};  // J1 fully blocked
    double g = 1.0;
    for (int i = 0; i < 300; ++i) g = planGateStep(g, req, got, prev, cfg);
    // Directional: (0*0.10 + 0.04*0.04) / (0.10^2 + 0.04^2) = 0.0016/0.0116 = 0.138.
    check(std::fabs(g - 0.1379) < 0.01,
          "surviving fraction is measured along the requested direction");
    // The old form would have compared max|req|=0.10 against max|got|=0.04 -> 0.40,
    // i.e. J1's request over J5's realization. Confirm the two really differ.
    check(std::fabs(g - 0.40) > 0.2, "differs from the old cross-joint max ratio");

    // A step redirected 90 degrees made no progress and must read as blocked,
    // even though its magnitude is unchanged.
    const JointArray sideways{0, 0.10, 0, 0, 0, 0};
    const JointArray fwd{0.10, 0, 0, 0, 0, 0};
    double s = 1.0;
    for (int i = 0; i < 300; ++i) s = planGateStep(s, fwd, sideways, prev, cfg);
    check(s < 0.01, "an orthogonal step scores 0, not 1 as a norm ratio would");
  }

  // -- Test 6: deadband keeps idle/hold ticks from driving the gate. ----------
  std::printf("Test 6: deadband\n");
  {
    const JointArray prev = zeros();
    const JointArray tiny{0.001, 0, 0, 0, 0, 0};  // below the 0.05 deg deadband
    const JointArray held = zeros();              // and fully held
    double g = 0.3;
    const double next = planGateStep(g, tiny, held, prev, cfg);
    check(next > g, "a sub-deadband step only releases, never attacks");
  }

  // -- Test 7: min_gate floors the gate. --------------------------------------
  std::printf("Test 7: min_gate floor\n");
  {
    SafetyPlanGateConfig floored = shipped();
    floored.min_gate = 0.25;
    const JointArray prev = zeros();
    const JointArray req{0.10, 0, 0, 0, 0, 0};
    const JointArray blocked = zeros();
    double g = 1.0;
    for (int i = 0; i < 500; ++i) g = planGateStep(g, req, blocked, prev, floored);
    check(g > 0.2499 && g < 0.2501, "a fully blocked step settles at min_gate");
  }


  // -- IK-THROTTLE PACING: only a SUSTAINED throttle paces the plan ------------
  //
  // The persistence condition is the design. Feeding transient clamps (the joint-limit
  // barrier, the acceleration clamp) into this gate was measured and reverted: it
  // produced a 4.8 Hz ripple at 2.1-2.8x the baseline tremble. A short throttle must
  // therefore leave the plan alone.
  std::printf("Test: IK-throttle pacing needs persistence\n");
  {
    SafetyPlanGateConfig c = shipped();
    c.ik_throttle_min_ticks = 25;   // 50 ms at 500 Hz

    // A 10-tick throttle: shorter than the threshold, so the gate must stay open.
    double g = 1.0;
    for (int i = 1; i <= 10; ++i) g = ikThrottlePlanGateStep(g, i, true, 0.14, c);
    check(g > 0.999, "a throttle shorter than min_ticks does not pace the plan");

    // Sustained: the gate walks down toward the achieved ratio.
    g = 1.0;
    for (int i = 1; i <= 300; ++i) g = ikThrottlePlanGateStep(g, i, true, 0.14, c);
    check(g < 0.20, "a sustained throttle paces the plan toward the achieved ratio");
    check(g >= c.min_gate, "and never below min_gate");

    // NEVER ZERO. The whole point of pacing rather than freezing is that the plan keeps
    // moving; a zero gate is the stop-go the freeze alternative was rejected for.
    SafetyPlanGateConfig floored = c;
    floored.min_gate = 0.05;
    double gz = 1.0;
    for (int i = 1; i <= 5000; ++i) gz = ikThrottlePlanGateStep(gz, i, true, 0.0, floored);
    check(gz >= floored.min_gate - 1e-12, "a zero achieved ratio still floors at min_gate");

    // Release: once the throttle clears, the gate recovers to 1.
    double gr = 0.14;
    for (int i = 0; i < 1000; ++i) gr = ikThrottlePlanGateStep(gr, 0, false, 1.0, c);
    check(gr > 0.99, "the gate reopens once the throttle clears");

    // Off by default: min_ticks 0 means the law is inert no matter what is fed to it.
    SafetyPlanGateConfig off = shipped();
    double go = 1.0;
    for (int i = 1; i <= 500; ++i) go = ikThrottlePlanGateStep(go, i, true, 0.01, off);
    check(off.ik_throttle_min_ticks == 0 && go > 0.999, "default-off leaves the plan unpaced");
  }

  // -- LOW-PASS ENGAGEMENT: the release is a ramp, not a step -----------------
  //
  // The instant release put a median 4,482 deg/s^2 into the command against 83
  // elsewhere (2026-08-28, 54x, peak 184,224 = 122x ddq_max) by reversing a joint
  // between two 2 ms samples.
  std::printf("Test: low-pass engagement ramps out\n");
  {
    const double dt = 0.002;
    // Attack is immediate: the tick the gate closes, the filter is fully engaged.
    check(lowpassEngagementStep(0.0, true, 0.060, dt) == 1.0, "engagement attacks in one tick");

    // Release decays with the configured constant: one tau (60 ms = 30 ticks) leaves
    // e^-1, two tau leaves e^-2.
    double e = 1.0;
    for (int i = 0; i < 30; ++i) e = lowpassEngagementStep(e, false, 0.060, dt);
    check(e > 0.35 && e < 0.39, "one tau of release is ~37% engaged");
    for (int i = 0; i < 30; ++i) e = lowpassEngagementStep(e, false, 0.060, dt);
    check(e > 0.12 && e < 0.15, "two tau of release is ~14% engaged");
    // Monotone, never negative, and it does not resurrect itself.
    bool monotone = true;
    double prev = e;
    for (int i = 0; i < 500; ++i) {
      e = lowpassEngagementStep(e, false, 0.060, dt);
      monotone = monotone && e <= prev + 1e-15 && e >= 0.0;
      prev = e;
    }
    check(monotone, "release is monotone, bounded, and does not resurrect");

    // release_sec <= 0 reproduces the instant release this replaces, so the old
    // behaviour stays reachable and the knob is a real A/B.
    check(lowpassEngagementStep(1.0, false, 0.0, dt) == 0.0, "release_sec 0 = instant release");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");

  return g_failures == 0 ? 0 : 1;
}
