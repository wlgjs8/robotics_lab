// test_command_tracking_window.cpp -- the dead-time tolerant "robot tracks its
// command" test used to excuse the chunk follower's plan-vs-command watchdogs.

#include "rb_servo/control/command_tracking_window.hpp"

#include <array>
#include <cmath>
#include <cstdio>

using rb_servo::Pose6D;
using rb_servo::control::CommandTrackingMatch;
using rb_servo::control::CommandTrackingWindow;

static int g_failures = 0;
static void check(bool ok, const char* name) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

static Pose6D poseAtYaw(double x_m, double yaw_rad) {
  Pose6D p;
  p.x = x_m; p.y = 0.0; p.z = 0.3;
  p.rx = 0.0; p.ry = 0.0; p.rz = yaw_rad;
  p.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, std::sin(0.5 * yaw_rad), std::cos(0.5 * yaw_rad)};
  return p;
}

int main() {
  constexpr double kPosTol = 0.0175;                  // half of a 35 mm lead budget
  constexpr double kAngTol = 0.5 * 0.08726646259;     // half of 5 deg
  constexpr double kTick = 0.002;

  std::printf("Test 1: empty window never tracks\n");
  {
    CommandTrackingWindow<25> w;
    const CommandTrackingMatch m = w.match(poseAtYaw(0.0, 0.0), kPosTol, kAngTol);
    check(!m.within, "empty window -> not tracking");
    check(m.lag_ticks == -1, "empty window reports lag -1");
  }

  std::printf("Test 2: a fast wrist turn tracked one dead time late matches at that lag\n");
  {
    // 1.4 rad/s yaw (80 deg/s): the naive sent_now-vs-actual_now gap after an 11-tick
    // (22 ms) dead time is 1.4 * 0.022 = 0.031 rad = 1.8 deg -- inside a 2.5 deg tol
    // here, but the point is the windowed match is EXACT at the dead time regardless
    // of speed. Use 4 rad/s so the naive gap (5 deg) is clearly outside tolerance.
    const double omega = 4.0;
    const int dead_ticks = 11;
    CommandTrackingWindow<25> w;
    bool ok = true;
    int last_lag = -1;
    double naive_gap = 0.0;
    for (int t = 0; t < 200; ++t) {
      const Pose6D sent = poseAtYaw(0.0, omega * kTick * t);
      w.record(sent);
      const int t_act = t - dead_ticks;
      if (t_act < 0) continue;
      const Pose6D actual = poseAtYaw(0.0, omega * kTick * t_act);
      const CommandTrackingMatch m = w.match(actual, kPosTol, kAngTol);
      ok &= m.within;
      last_lag = m.lag_ticks;
      naive_gap = rb_servo::math::orientationDistanceRad(sent, actual);
    }
    check(ok, "tracking arm always finds a match while turning at 4 rad/s");
    check(last_lag == dead_ticks, "the match is the command sent one dead time ago");
    check(naive_gap > kAngTol, "the naive sent_now-vs-actual_now gap would have failed");
  }

  std::printf("Test 3: a pinned arm stops matching within one window\n");
  {
    // The command keeps advancing at 0.45 m/s while the arm stays put.
    CommandTrackingWindow<25> w;
    const Pose6D pinned = poseAtYaw(0.0, 0.0);
    int first_fail_tick = -1;
    for (int t = 0; t < 200; ++t) {
      w.record(poseAtYaw(0.45 * kTick * t, 0.0));
      const CommandTrackingMatch m = w.match(pinned, kPosTol, kAngTol);
      if (!m.within && first_fail_tick < 0) first_fail_tick = t;
    }
    // The oldest entry leaves the window after 25 ticks; by then the newest command is
    // 25 * 0.9 mm = 22.5 mm away and the oldest retained one 0.9 mm less -- past 17.5 mm.
    check(first_fail_tick > 0, "pinned arm is eventually reported as not tracking");
    check(first_fail_tick <= 45, "…and within about one window of the command leaving tolerance");
    std::printf("    first not-tracking tick = %d\n", first_fail_tick);
  }

  std::printf("Test 4: the score is joint (position AND orientation)\n");
  {
    CommandTrackingWindow<8> w;
    w.record(poseAtYaw(0.0, 0.0));
    const CommandTrackingMatch pos_off = w.match(poseAtYaw(0.02, 0.0), kPosTol, kAngTol);
    const CommandTrackingMatch ang_off = w.match(poseAtYaw(0.0, 0.05), kPosTol, kAngTol);
    const CommandTrackingMatch both_in = w.match(poseAtYaw(0.01, 0.02), kPosTol, kAngTol);
    check(!pos_off.within, "20 mm off with orientation exact -> not tracking");
    check(!ang_off.within, "2.9 deg off with position exact -> not tracking");
    check(both_in.within, "10 mm / 1.1 deg off -> tracking");
    check(std::fabs(both_in.pos_m - 0.01) < 1e-9 && std::fabs(both_in.ang_rad - 0.02) < 1e-9,
          "reported distances are those of the matched entry");
  }

  std::printf("Test 5: zero tolerance is fail-closed\n");
  {
    CommandTrackingWindow<4> w;
    w.record(poseAtYaw(0.0, 0.0));
    check(!w.match(poseAtYaw(1e-6, 0.0), 0.0, kAngTol).within, "zero position tolerance refuses a 1 um gap");
    check(w.match(poseAtYaw(0.0, 0.0), 0.0, 0.0).within, "…but accepts an exact match");
  }

  std::printf("Test 6: the ring wraps and keeps only the newest Capacity entries\n");
  {
    CommandTrackingWindow<4> w;
    for (int t = 0; t < 10; ++t) w.record(poseAtYaw(0.1 * t, 0.0));
    check(w.size() == 4, "size saturates at capacity");
    check(!w.match(poseAtYaw(0.0, 0.0), kPosTol, kAngTol).within, "the oldest entries are gone");
    const CommandTrackingMatch m = w.match(poseAtYaw(0.6, 0.0), kPosTol, kAngTol);
    check(m.within && m.lag_ticks == 3, "the oldest retained entry is lag Capacity-1");
  }

  std::printf("%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
  return g_failures == 0 ? 0 : 1;
}
