// chunk_follower_core_test.cpp — standalone unit test for chunk_follower_core.hpp.
// Compiles like the spike (no full rb_servo build); graduates to tests/ + CMake
// in Stage 0.
//
//   g++ -std=c++17 -O2 -I/opt/ros/humble/include -Irb_servo_server/include \
//       rb_servo_server/tools/chunk_follower_core_test.cpp \
//       -L/opt/ros/humble/lib/x86_64-linux-gnu -lruckig \
//       -Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu -o /tmp/cf_core_test && /tmp/cf_core_test

#include "rb_servo/control/chunk_follower_core.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

using rb_servo::control::AxisLimit;
using rb_servo::control::BoundarySample;
using rb_servo::control::ChunkFollowerSegment;
using rb_servo::control::dvCapacity;

static constexpr double DT = 1.0 / 30.0;
static int g_failures = 0;

static void check(bool ok, const char* name) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

static int sgn(double x) { return (x > 1e-12) - (x < -1e-12); }

// Build a central-difference BoundarySample of a 3D reference at grid index k.
template <std::size_t N>
static BoundarySample<N> sampleRef(const std::function<std::array<double, N>(double)>& p,
                                   double tk) {
  const auto pkm1 = p(tk - DT), pk = p(tk), pkp1 = p(tk + DT);
  BoundarySample<N> s;
  for (std::size_t d = 0; d < N; ++d) {
    const double d_k = pk[d] - pkm1[d];
    const double d_kp1 = pkp1[d] - pk[d];
    s.pf[d] = pk[d];
    s.vf[d] = (d_k + d_kp1) / (2 * DT);
    s.af[d] = (d_kp1 - d_k) / (DT * DT);
    s.sign_dk[d] = sgn(d_k);
    s.sign_dkp1[d] = sgn(d_kp1);
  }
  return s;
}

int main() {
  std::array<AxisLimit, 3> lim{};  // defaults v=1,a=6,j=30

  // -- Test 1: dvCapacity is the jerk branch at dt and matches j·dt²/4. --------
  std::printf("Test 1: dvCapacity jerk regime\n");
  {
    const double cap = dvCapacity(6.0, 30.0, DT);
    check(std::fabs(cap - 0.25 * 30.0 * DT * DT) < 1e-12, "cap = j·dt²/4 (jerk-limited at dt)");
    check(dvCapacity(6.0, 30.0, 1.0) > dvCapacity(6.0, 30.0, DT), "cap grows with T");
  }

  // -- Test 2: on-reference multi-segment run: solves, converges, C² seams. ----
  std::printf("Test 2: on-reference chaining (converge + C² continuity)\n");
  {
    // gentle smooth 3D curve well inside the Δv capacity (~0.0083 m/s/step).
    std::function<std::array<double, 3>(double)> ref = [](double t) {
      return std::array<double, 3>{0.20 * std::sin(0.8 * t),
                                   0.15 * std::sin(0.6 * t),
                                   0.05 * t};
    };
    ChunkFollowerSegment<3> seg(lim, DT);
    auto s0 = sampleRef<3>(ref, 0.0);
    seg.seed(s0.pf, s0.vf, s0.af);  // start exactly on reference

    bool all_work = true, all_converge = true, all_c2 = true, track_ok = true;
    std::array<double, 3> prev_end_p{}, prev_end_v{}, prev_end_a{};
    bool have_prev = false;

    for (int k = 1; k <= 40; ++k) {
      auto s = sampleRef<3>(ref, k * DT);
      auto r = seg.solve(s);
      all_work &= (r.result == ruckig::Result::Working);
      all_converge &= r.converged;

      std::array<double, 3> sp0, sv0, sa0, spd, svd, sad;
      seg.sample(0.0, sp0, sv0, sa0);   // this segment's start
      seg.sample(DT, spd, svd, sad);    // this segment's end (=next chained start)

      if (have_prev) {
        for (int d = 0; d < 3; ++d) {
          all_c2 &= std::fabs(sp0[d] - prev_end_p[d]) < 1e-9;
          all_c2 &= std::fabs(sv0[d] - prev_end_v[d]) < 1e-9;
          all_c2 &= std::fabs(sa0[d] - prev_end_a[d]) < 1e-9;  // accel continuity
        }
      }
      // tracks the reference: end-of-segment position ≈ ref(k·dt)
      const auto pr = ref(k * DT);
      for (int d = 0; d < 3; ++d) track_ok &= std::fabs(spd[d] - pr[d]) < 5e-3;

      prev_end_p = spd; prev_end_v = svd; prev_end_a = sad; have_prev = true;
    }
    check(all_work, "every on-reference segment solves (Working)");
    check(all_converge, "every on-reference segment converges (T_opt ≈ dt)");
    check(all_c2, "C² continuous at seams: p, v, AND a match (chaining wired)");
    check(track_ok, "accumulated setpoint tracks the reference (<5mm)");
  }

  // -- Test 3: over-fast target → ladder dilates α<1 and still solves. ---------
  std::printf("Test 3: sacrifice ladder (over-fast target)\n");
  {
    ChunkFollowerSegment<3> seg(lim, DT);
    seg.seed({0, 0, 0}, {0, 0, 0}, {0, 0, 0});
    BoundarySample<3> s;                 // demand a huge Δv the jerk cap can't meet
    s.pf = {0.02, 0, 0}; s.vf = {0.5, 0, 0}; s.af = {0, 0, 0};
    s.sign_dk = {1, 0, 0}; s.sign_dkp1 = {1, 0, 0};
    const double a = seg.predictAlpha(s);
    check(a < 1.0, "ladder picks α<1 for an over-fast target");
    auto r = seg.solve(s);
    check(r.result == ruckig::Result::Working, "over-fast segment still produces a valid trajectory");
    check(!r.converged, "over-fast segment is correctly flagged non-converged (T slips)");
  }

  // -- Test 4: forward-safe predictor rejects an over-fast START velocity. -----
  std::printf("Test 4: forward-safe invariant\n");
  {
    ChunkFollowerSegment<3> seg(lim, DT);
    seg.seed({0, 0, 0}, {0.9, 0, 0}, {0, 0, 0});  // v0 large, near a close target
    BoundarySample<3> s;
    s.pf = {0.005, 0, 0}; s.vf = {0.0, 0, 0}; s.af = {0, 0, 0};
    s.sign_dk = {1, 0, 0}; s.sign_dkp1 = {1, 0, 0};
    check(!seg.forwardSafe(s), "forward-safe rejects v0 too high for the remaining distance");
    seg.seed({0, 0, 0}, {0.02, 0, 0}, {0, 0, 0});   // modest v0
    check(seg.forwardSafe(s), "forward-safe accepts a modest v0");
  }

  // -- Test 5: corner (direction reversal) rings af→0 and flags corner. -------
  std::printf("Test 5: corner ring-down\n");
  {
    ChunkFollowerSegment<3> seg(lim, DT);
    seg.seed({0, 0, 0}, {0.05, 0, 0}, {0, 0, 0});
    BoundarySample<3> s;                 // d_k > 0, d_{k+1} < 0  → reversal on x
    s.pf = {0.001, 0, 0}; s.vf = {0.0, 0, 0}; s.af = {2.0, 0, 0};
    s.sign_dk = {1, 0, 0}; s.sign_dkp1 = {-1, 0, 0};
    auto r = seg.solve(s);
    check(r.corner, "corner detected on the reversing axis");
    check(r.result == ruckig::Result::Working, "corner segment solves");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
