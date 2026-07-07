// test_delta_twist_follower.cpp - local per-frame action delta follower.

#include "rb_servo/control/delta_twist_follower.hpp"
#include "rb_servo/math/se3.hpp"

#include <array>
#include <cmath>
#include <cstdio>

using rb_servo::Pose6D;
using rb_servo::Vec6;
using rb_servo::control::AxisLimit;
using rb_servo::control::ChunkFrame;
using rb_servo::control::DeltaTwistFollower;
using rb_servo::control::DeltaTwistFollowerConfig;

static int g_failures = 0;

static void check(bool ok, const char* name) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

static constexpr double TICK = 0.002;
static constexpr double POLICY_DT = 1.0 / 30.0;

static Pose6D identityPose() {
  Pose6D p;
  p.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
  return p;
}

static bool finitePose(const Pose6D& p) {
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
  if (!p.quaternion_xyzw.has_value()) return false;
  const auto& q = *p.quaternion_xyzw;
  return std::isfinite(q[0]) && std::isfinite(q[1]) && std::isfinite(q[2]) && std::isfinite(q[3]);
}

static double quatNorm(const Pose6D& p) {
  const auto& q = *p.quaternion_xyzw;
  return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

static double linearNorm(const Vec6& v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

static DeltaTwistFollowerConfig fastConfig() {
  DeltaTwistFollowerConfig cfg;
  cfg.lin = AxisLimit{2.0, 1000.0, 1000000.0};
  cfg.ang = AxisLimit{10.0, 5000.0, 1000000.0};
  cfg.consume_steps = 6;
  cfg.tau_sec = TICK;
  cfg.max_residual_m = 0.50;
  cfg.max_residual_rad = 1.0;
  cfg.max_lead_m = 0.50;
  cfg.max_lead_rad = 1.0;
  return cfg;
}

static ChunkFrame makeFrame(std::initializer_list<Vec6> deltas) {
  ChunkFrame f;
  f.policy_dt = POLICY_DT;
  f.wire_seq = 44;
  f.recv_seq = 9;
  f.recv_time = 10.0;
  int i = 0;
  for (const Vec6& delta : deltas) {
    f.delta.push_back(delta);
    f.grip.push_back(20.0 + static_cast<double>(i));
    ++i;
  }
  return f;
}

static Pose6D tickFor(DeltaTwistFollower& follower, double seconds) {
  Pose6D out = follower.lastPose();
  const int ticks = static_cast<int>(std::ceil(seconds / TICK));
  for (int i = 0; i < ticks; ++i) {
    out = follower.tick(TICK);
  }
  return out;
}

int main() {
  std::printf("Test 1: cold submit + translation delta\n");
  {
    DeltaTwistFollower follower(fastConfig());
    follower.submitFrame(makeFrame({Vec6{0.01, 0.0, 0.0, 0.0, 0.0, 0.0}}), identityPose());
    check(follower.active(), "active after one delta row");
    const Pose6D out = tickFor(follower, POLICY_DT);
    check(finitePose(out), "output pose finite");
    check(std::fabs(quatNorm(out) - 1.0) < 1e-9, "output quaternion normalized");
    check(out.x > 0.006 && out.x < 0.012, "x increased by local per-frame delta without dt scaling");
    check(std::fabs(out.y) < 1e-6 && std::fabs(out.z) < 1e-6, "off-axis translation stays near zero");
  }

  std::printf("Test 2: local rotvec delta\n");
  {
    DeltaTwistFollower follower(fastConfig());
    follower.submitFrame(makeFrame({Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.1}}), identityPose());
    const Pose6D out = tickFor(follower, POLICY_DT);
    const rb_servo::math::Vector3 logged = rb_servo::math::log3(rb_servo::math::rotationFromPose(out));
    check(logged.z() > 0.06 && logged.z() < 0.12, "positive yaw follows requested rotvec magnitude");
    check(std::fabs(logged.x()) < 1e-6 && std::fabs(logged.y()) < 1e-6, "rotation stays on requested axis");
  }

  std::printf("Test 3: consume_steps prevents stale tail consumption\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.consume_steps = 1;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(makeFrame({
        Vec6{0.01, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.01, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.01, 0.0, 0.0, 0.0, 0.0, 0.0},
    }), identityPose());
    tickFor(follower, POLICY_DT * 1.2);
    check(follower.diag().stall, "stall flagged after consume budget is spent");
    check(follower.windowConsumed() == 1, "only configured consume budget was consumed");
    check(follower.windowIndex() == 1, "horizon tail rows were not consumed indefinitely");
  }

  std::printf("Test 4: reanchor clears old residual\n");
  {
    DeltaTwistFollower follower(fastConfig());
    follower.submitFrame(makeFrame({Vec6{0.05, 0.0, 0.0, 0.0, 0.0, 0.0}}), identityPose());
    tickFor(follower, POLICY_DT * 0.5);
    Pose6D ref = identityPose();
    ref.x = 0.25;
    ref.y = -0.04;
    ref.z = 0.10;
    follower.reanchor(ref);
    const Pose6D out = follower.tick(TICK);
    check(rb_servo::math::positionDistance(out, ref) < 1e-5, "next output starts near reanchor reference");
    check(linearNorm(follower.diag().pending_delta) < 1e-9, "old pending delta cleared on reanchor");
  }

  std::printf("Test 5: missing delta rows do not activate\n");
  {
    DeltaTwistFollower follower(fastConfig());
    ChunkFrame f;
    f.policy_dt = POLICY_DT;
    f.grip.push_back(1.0);
    follower.submitFrame(f, identityPose());
    check(!follower.active(), "frame without deltas rejected");
  }

  std::printf("Test 6: residual clamp bounds huge backlog\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.max_lead_m = 0.20;
    cfg.max_residual_m = 0.005;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(makeFrame({Vec6{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}}), identityPose());
    follower.tick(TICK);
    check(linearNorm(follower.diag().pending_delta) <= cfg.max_residual_m + 1e-12,
          "pending residual is clamped after huge delta");
    check(finitePose(follower.lastPose()), "huge delta still emits finite pose");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
