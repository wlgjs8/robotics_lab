// test_projection_release.cpp — the asymmetric release slew on the geometric
// projection correction. See include/rb_servo/control/projection_release.hpp for
// the measurements (servo_log_20260828_004539.csv, left-arm <-> stand at
// t=223.37-224.01 s) that each case below reproduces.

#include "rb_servo/control/projection_release.hpp"

#include <cmath>
#include <cstdio>

using rb_servo::JointArray;
using rb_servo::kDof;
using rb_servo::control::projectionReleaseStep;

static int g_failures = 0;
static void check(bool ok, const char* name) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

constexpr double kDt = 0.002;         // 500 Hz
constexpr double kSlew = 400.0;       // deg/s^2, the shipped value

static JointArray zeros() { return JointArray{0, 0, 0, 0, 0, 0}; }

// A correction expressed as a velocity (deg/s) is a per-tick displacement of
// vel * dt degrees; the log reports the velocity, so the tests speak in it.
static JointArray targetFor(double correction_deg_s) {
  JointArray q = zeros();
  q[0] = -correction_deg_s * kDt;   // final = requested - correction
  return q;
}
static double correctionOf(const JointArray& requested, const JointArray& final_q) {
  return (requested[0] - final_q[0]) / kDt;
}

int main() {
  const JointArray requested = zeros();

  // -- Test 1: engaging is INSTANT. A safety damper may never be slowed down. ---
  std::printf("Test 1: engage is instantaneous\n");
  {
    JointArray prev = zeros();
    JointArray q = targetFor(12.09);
    projectionReleaseStep(requested, q, prev, kDt, kSlew);
    check(std::fabs(correctionOf(requested, q) - 12.09) < 1e-9,
          "a correction appearing from nothing is applied in full on the first tick");

    // ... and growing further is also instant.
    JointArray q2 = targetFor(20.0);
    projectionReleaseStep(requested, q2, prev, kDt, kSlew);
    check(std::fabs(correctionOf(requested, q2) - 20.0) < 1e-9,
          "growing an existing correction is instantaneous too");
  }

  // -- Test 2: THE STEP RELEASE. 12.09 -> 0 in one tick becomes a ramp. ---------
  std::printf("Test 2: step release becomes a bounded ramp\n");
  {
    JointArray prev = zeros();
    JointArray engage = targetFor(12.09);
    projectionReleaseStep(requested, engage, prev, kDt, kSlew);

    // The solver now asks for nothing at all -- the rows disengaged.
    JointArray q = requested;
    projectionReleaseStep(requested, q, prev, kDt, kSlew);
    const double after_one = correctionOf(requested, q);
    check(after_one > 11.0,
          "one tick after full release the correction is still ~12, not 0");
    check(std::fabs((12.09 - after_one) - kSlew * kDt) < 1e-9,
          "the per-tick drop is exactly slew*dt");

    // It does reach zero, and in the intended time.
    int ticks = 1;
    while (correctionOf(requested, q) > 1e-9 && ticks < 10000) {
      JointArray next = requested;
      projectionReleaseStep(requested, next, prev, kDt, kSlew);
      q = next;
      ++ticks;
    }
    const double ms = ticks * kDt * 1000.0;
    check(ms > 20.0 && ms < 45.0, "12.09 deg/s releases over ~30 ms, not one tick");
    check(correctionOf(requested, q) == 0.0, "and it does reach exactly zero");
  }

  // -- Test 3: THE DROPOUTS. 12.28 -> 3.93 -> 12.36 stops being a hole. --------
  // Measured 40 times on the left arm and 25 on the right: the solve momentarily
  // returns a third of the correction and is back the next tick, each one a
  // ~4,280 deg/s^2 injection.
  std::printf("Test 3: single-tick dropouts are absorbed\n");
  {
    JointArray prev = zeros();
    JointArray a = targetFor(12.28);
    projectionReleaseStep(requested, a, prev, kDt, kSlew);
    JointArray b = targetFor(3.93);                 // the dropout tick
    projectionReleaseStep(requested, b, prev, kDt, kSlew);
    const double held = correctionOf(requested, b);
    check(held > 11.0, "the dropout tick holds ~12.28 instead of collapsing to 3.93");
    check(12.28 - held <= kSlew * kDt + 1e-9, "and it falls by at most slew*dt");
    JointArray c = targetFor(12.36);                // solver recovers
    projectionReleaseStep(requested, c, prev, kDt, kSlew);
    check(std::fabs(correctionOf(requested, c) - 12.36) < 1e-9,
          "recovery is instantaneous (growing is never limited)");
  }

  // -- Test 4: a sign flip decays through zero rather than snapping across. ----
  std::printf("Test 4: sign reversal\n");
  {
    JointArray prev = zeros();
    JointArray a = targetFor(10.0);
    projectionReleaseStep(requested, a, prev, kDt, kSlew);
    JointArray b = targetFor(-10.0);   // solver wants the opposite correction
    projectionReleaseStep(requested, b, prev, kDt, kSlew);
    const double c = correctionOf(requested, b);
    check(c > 0.0 && c < 10.0,
          "the old direction decays first; it does not jump to the opposite sign");
    check(10.0 - c <= kSlew * kDt + 1e-9, "bounded by the same slew");
  }

  // -- Test 5: disabled (0) is byte-identical to the legacy instant release. ---
  std::printf("Test 5: slew 0 preserves legacy behaviour\n");
  {
    JointArray prev = zeros();
    JointArray a = targetFor(12.09);
    projectionReleaseStep(requested, a, prev, kDt, 0.0);
    JointArray q = requested;
    projectionReleaseStep(requested, q, prev, kDt, 0.0);
    check(correctionOf(requested, q) == 0.0, "disabled: the correction drops to 0 at once");
    for (int j = 0; j < kDof; ++j) {
      if (prev[j] != 0.0) { check(false, "disabled still tracks prev"); break; }
    }
    check(true, "disabled records prev so enabling mid-run cannot release a stale value");
  }

  // -- Test 6: untouched joints are left exactly alone. ------------------------
  std::printf("Test 6: no correction, no change\n");
  {
    JointArray prev = zeros();
    JointArray q{1.0, -2.0, 3.0, -4.0, 5.0, -6.0};
    const JointArray req = q;   // requested == final: the projection did nothing
    projectionReleaseStep(req, q, prev, kDt, kSlew);
    bool same = true;
    for (int j = 0; j < kDof; ++j) same = same && (q[j] == req[j]);
    check(same, "a tick with no correction passes the target through untouched");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
