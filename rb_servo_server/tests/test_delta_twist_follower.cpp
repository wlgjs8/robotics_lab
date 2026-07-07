// test_delta_twist_follower.cpp - local per-frame action delta follower.

#include "rb_servo/control/delta_twist_follower.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <initializer_list>

using rb_servo::Pose6D;
using rb_servo::Vec6;
using rb_servo::control::AxisLimit;
using rb_servo::control::ChunkFrame;
using rb_servo::control::DeltaTwistFollower;
using rb_servo::control::DeltaTwistFollowerConfig;
using rb_servo::control::DeltaTwistStepPhase;

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

static Pose6D yawPose(double yaw_rad) {
  Pose6D p = identityPose();
  p.quaternion_xyzw.reset();
  p.rz = yaw_rad;
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

static double angularNorm(const Vec6& v) {
  return std::sqrt(v.rx * v.rx + v.ry * v.ry + v.rz * v.rz);
}

static bool finiteVec6(const Vec6& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
         std::isfinite(v.rx) && std::isfinite(v.ry) && std::isfinite(v.rz);
}

static DeltaTwistFollowerConfig fastConfig() {
  DeltaTwistFollowerConfig cfg;
  cfg.lin = AxisLimit{5.0, 1000.0, 1000000.0};
  cfg.ang = AxisLimit{20.0, 5000.0, 1000000.0};
  cfg.consume_steps = 6;
  cfg.reserve_steps = 2;
  cfg.tau_sec = TICK;
  cfg.residual_drain_steps = 1;
  cfg.max_residual_m = 0.50;
  cfg.max_residual_rad = 1.0;
  cfg.max_lead_m = 0.50;
  cfg.max_lead_rad = 1.0;
  cfg.stale_residual_timeout_sec = 10.0;
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

static Pose6D tickOpenLoop(DeltaTwistFollower& follower, double seconds) {
  Pose6D out = follower.lastPose();
  const int ticks = static_cast<int>(std::ceil(seconds / TICK));
  for (int i = 0; i < ticks; ++i) {
    out = follower.tick(TICK);
  }
  return out;
}

static Pose6D tickPerfectFeedback(DeltaTwistFollower& follower, double seconds) {
  Pose6D out = follower.lastPose();
  follower.setFeedbackPose(out);
  const int ticks = static_cast<int>(std::ceil(seconds / TICK));
  for (int i = 0; i < ticks; ++i) {
    out = follower.tick(TICK);
    follower.setFeedbackPose(out);
  }
  return out;
}

int main() {
  std::printf("Test A: reserve_steps are consumed, then stale tail is blocked\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.consume_steps = 6;
    cfg.reserve_steps = 2;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(makeFrame({
        Vec6{0.001, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.001, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.001, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.001, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.001, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.001, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.001, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.001, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.100, 0.0, 0.0, 0.0, 0.0, 0.0},
    }), identityPose());
    check(follower.diag().seg_step_index == 0, "step 0 consumed at submit");
    check(follower.diag().step_phase == DeltaTwistStepPhase::Normal, "step 0 is normal phase");
    for (int expected = 1; expected <= 7; ++expected) {
      tickPerfectFeedback(follower, POLICY_DT);
      check(follower.diag().seg_step_index == expected, "expected next normal/reserve row consumed");
      const DeltaTwistStepPhase expected_phase =
          expected < 6 ? DeltaTwistStepPhase::Normal : DeltaTwistStepPhase::Reserve;
      check(follower.diag().step_phase == expected_phase, "step phase matches normal/reserve budget");
    }
    tickPerfectFeedback(follower, POLICY_DT);
    check(follower.diag().stall, "stall flagged after normal+reserve budget is spent");
    check(follower.diag().seg_step_index == -1, "follower_step is -1 after budget exhaustion");
    check(follower.windowConsumed() == 8, "only normal+reserve rows were consumed");
    check(follower.windowIndex() == 8, "rows beyond reserve were not consumed");
    check(follower.diag().normal_consumed == 6, "normal consumed count recorded");
    check(follower.diag().reserve_consumed == 2, "reserve consumed count recorded");
  }

  std::printf("Test B: fresh frame resets phase without zeroing velocity\n");
  {
    DeltaTwistFollower follower(fastConfig());
    follower.submitFrame(makeFrame({
        Vec6{0.03, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.03, 0.0, 0.0, 0.0, 0.0, 0.0},
    }), identityPose());
    tickOpenLoop(follower, POLICY_DT * 0.85);
    const double xi_before = linearNorm(follower.xiCommand());
    ChunkFrame fresh = makeFrame({
        Vec6{0.01, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.01, 0.0, 0.0, 0.0, 0.0, 0.0},
    });
    fresh.wire_seq = 45;
    follower.submitFrame(fresh, identityPose());
    check(follower.tInSegment() == 0.0, "fresh frame resets local policy-frame phase");
    check(follower.diag().residual_cleared_on_frame, "fresh frame reports residual clear");
    check(linearNorm(follower.pendingDelta()) < 0.015, "old residual is cleared before adding new first step");
    check(linearNorm(follower.xiCommand()) >= 0.5 * xi_before, "submitFrame does not force xi_cmd to zero");
    tickOpenLoop(follower, POLICY_DT * 0.50);
    check(follower.diag().seg_step_index == 0, "new step 0 remains active halfway through policy_dt");
    tickOpenLoop(follower, POLICY_DT * 0.60);
    check(follower.diag().seg_step_index == 1, "new step 1 is consumed after roughly one policy_dt");
  }

  std::printf("Test C: norm clamp preserves linear/angular direction\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.lin = AxisLimit{0.18, 1000.0, 1000000.0};
    cfg.ang = AxisLimit{0.60, 1000.0, 1000000.0};
    cfg.max_residual_m = 100.0;
    cfg.max_residual_rad = 100.0;
    cfg.max_lead_m = 100.0;
    cfg.max_lead_rad = 100.0;

    DeltaTwistFollower linear_x(cfg);
    linear_x.submitFrame(makeFrame({Vec6{10.0, 0.0, 0.0, 0.0, 0.0, 0.0}}), identityPose());
    (void)linear_x.tick(TICK);
    check(linearNorm(linear_x.diag().xi_ref) <= cfg.lin.v_max + 1e-12, "linear xi_ref norm respects configured limit");
    check(linear_x.diag().xi_ref.x > 0.17, "x direction preserved by norm clamp");
    check(std::fabs(linear_x.diag().xi_ref.y) < 1e-12, "no off-axis y from x-only clamp");

    DeltaTwistFollower linear_xy(cfg);
    linear_xy.submitFrame(makeFrame({Vec6{10.0, 10.0, 0.0, 0.0, 0.0, 0.0}}), identityPose());
    (void)linear_xy.tick(TICK);
    check(linearNorm(linear_xy.diag().xi_ref) <= cfg.lin.v_max + 1e-12, "diagonal linear norm has no sqrt(3) overshoot");
    check(std::fabs(linear_xy.diag().xi_ref.x - linear_xy.diag().xi_ref.y) < 1e-12, "diagonal linear direction ratio preserved");

    DeltaTwistFollower angular_xy(cfg);
    angular_xy.submitFrame(makeFrame({Vec6{0.0, 0.0, 0.0, 10.0, 10.0, 0.0}}), identityPose());
    (void)angular_xy.tick(TICK);
    check(angularNorm(angular_xy.diag().xi_ref) <= cfg.ang.v_max + 1e-12, "diagonal angular norm respects configured limit");
    check(std::fabs(angular_xy.diag().xi_ref.rx - angular_xy.diag().xi_ref.ry) < 1e-12, "diagonal angular direction ratio preserved");
  }

  std::printf("Test D: min time-to-go prevents end-frame spike\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.lin = AxisLimit{10.0, 1000.0, 1000000.0};
    cfg.min_time_to_go_sec = 0.020;
    cfg.max_lead_m = 100.0;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(makeFrame({Vec6{0.01, 0.0, 0.0, 0.0, 0.0, 0.0}}), identityPose());
    for (int i = 0; i < 16; ++i) {
      (void)follower.tick(TICK);
    }
    check(follower.diag().min_time_to_go_used, "min time-to-go floor is reported near frame end");
    check(std::fabs(follower.diag().xi_ref.x - 0.5) < 0.05, "xi_ref uses pending_delta/min_time_to_go instead of end-frame spike");
    check(linearNorm(follower.diag().xi_ref) < 1.0, "xi_ref stays below catch-up spike scale");
  }

  std::printf("Test E: area matching translation with perfect feedback\n");
  {
    DeltaTwistFollower follower(fastConfig());
    follower.submitFrame(makeFrame({Vec6{0.01, 0.0, 0.0, 0.0, 0.0, 0.0}}), identityPose());
    const Pose6D out = tickPerfectFeedback(follower, POLICY_DT);
    check(finitePose(out), "output pose finite");
    check(std::fabs(quatNorm(out) - 1.0) < 1e-9, "output quaternion normalized");
    check(out.x > 0.008 && out.x < 0.012, "translation area is close to requested 10 mm frame delta");
    check(std::fabs(out.y) < 1e-6 && std::fabs(out.z) < 1e-6, "off-axis translation stays near zero");
  }

  std::printf("Test F: area matching rotation with perfect feedback\n");
  {
    DeltaTwistFollower follower(fastConfig());
    follower.submitFrame(makeFrame({Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.1}}), identityPose());
    const Pose6D out = tickPerfectFeedback(follower, POLICY_DT);
    const rb_servo::math::Vector3 logged = rb_servo::math::log3(rb_servo::math::rotationFromPose(out));
    check(logged.z() > 0.08 && logged.z() < 0.12, "positive yaw roughly matches requested rotvec delta");
    check(std::fabs(logged.x()) < 1e-6 && std::fabs(logged.y()) < 1e-6, "rotation stays on requested axis");
  }

  std::printf("Test G: pending residual is reduced by feedback, not command alone\n");
  {
    DeltaTwistFollower follower(fastConfig());
    const Pose6D start = identityPose();
    follower.submitFrame(makeFrame({Vec6{0.02, 0.0, 0.0, 0.0, 0.0, 0.0}}), start);
    follower.setFeedbackPose(start);
    const Pose6D open_loop_out = tickOpenLoop(follower, POLICY_DT * 0.5);
    const double residual_without_feedback = linearNorm(follower.pendingDelta());
    check(residual_without_feedback > 0.015, "pending residual does not disappear from command integration alone");
    follower.setFeedbackPose(open_loop_out);
    const double residual_with_feedback = linearNorm(follower.pendingDelta());
    check(residual_with_feedback < residual_without_feedback, "pending residual decreases after realized feedback delta");
    check(finiteVec6(follower.diag().step_delta), "telemetry step delta is finite");
    check(finiteVec6(follower.diag().realized_delta), "telemetry realized delta is finite");
    check(follower.diag().step_delta.x > 0.019, "telemetry records requested frame delta");
    check(follower.diag().realized_delta.x > 0.0, "telemetry records realized feedback delta");
    check(std::isfinite(follower.diag().realized_linear_ratio), "linear realized ratio is finite");
  }

  std::printf("Test G2: feedback projection does not increase residual when feedback opposes pending\n");
  {
    DeltaTwistFollower linear_follower(fastConfig());
    const Pose6D start = identityPose();
    linear_follower.submitFrame(makeFrame({Vec6{0.02, 0.0, 0.0, 0.0, 0.0, 0.0}}), start);
    linear_follower.setFeedbackPose(start);
    Pose6D moved_back = start;
    moved_back.x = -0.005;
    const double linear_before = linearNorm(linear_follower.pendingDelta());
    linear_follower.setFeedbackPose(moved_back);
    check(linearNorm(linear_follower.pendingDelta()) <= linear_before + 1e-12, "opposing linear feedback does not grow pending residual");
    check(linear_follower.diag().lin_feedback_cos < -0.9, "linear feedback cosine records opposing direction");

    DeltaTwistFollower angular_follower(fastConfig());
    angular_follower.submitFrame(makeFrame({Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.10}}), start);
    angular_follower.setFeedbackPose(start);
    const double angular_before = angularNorm(angular_follower.pendingDelta());
    angular_follower.setFeedbackPose(yawPose(-0.02));
    check(angularNorm(angular_follower.pendingDelta()) <= angular_before + 1e-12, "opposing angular feedback does not grow pending residual");
    check(angular_follower.diag().ang_feedback_cos < -0.9, "angular feedback cosine records opposing direction");
  }

  std::printf("Test G3: telemetry ratios are robust near zero request\n");
  {
    DeltaTwistFollower follower(fastConfig());
    const Pose6D start = identityPose();
    follower.submitFrame(makeFrame({Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}), start);
    follower.setFeedbackPose(start);
    check(follower.diag().realized_linear_ratio == 1.0, "zero request and zero realized reports ratio 1");
    Pose6D moved = start;
    moved.x = 0.001;
    follower.setFeedbackPose(moved);
    check(std::isfinite(follower.diag().realized_linear_ratio), "zero denominator ratio stays finite");
    check(follower.diag().realized_linear_ratio == 0.0, "zero request with nonzero realized reports ratio 0");
  }

  std::printf("Test H: command lead clamp limits pose ahead of feedback\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.max_lead_m = 0.012;
    cfg.max_lead_rad = 0.04;
    cfg.max_residual_m = 0.50;
    cfg.max_residual_rad = 1.0;
    DeltaTwistFollower follower(cfg);
    const Pose6D feedback = identityPose();
    follower.submitFrame(makeFrame({Vec6{0.20, 0.0, 0.0, 0.0, 0.0, 0.5}}), feedback);
    follower.setFeedbackPose(feedback);
    double max_lead_m = 0.0;
    double max_lead_rad = 0.0;
    for (int i = 0; i < 80; ++i) {
      follower.setFeedbackPose(feedback);
      (void)follower.tick(TICK);
      const double lead_m = rb_servo::math::positionDistance(feedback, follower.lastPose());
      const double lead_rad = rb_servo::math::orientationDistanceRad(feedback, follower.lastPose());
      max_lead_m = std::max(max_lead_m, lead_m);
      max_lead_rad = std::max(max_lead_rad, lead_rad);
    }
    check(max_lead_m <= cfg.max_lead_m + 1e-9, "linear command lead stays clamped");
    check(max_lead_rad <= cfg.max_lead_rad + 1e-9, "angular command lead stays clamped");
  }

  std::printf("Test I: missing delta rows do not activate\n");
  {
    DeltaTwistFollower follower(fastConfig());
    ChunkFrame f;
    f.policy_dt = POLICY_DT;
    f.grip.push_back(1.0);
    follower.submitFrame(f, identityPose());
    check(!follower.active(), "frame without deltas rejected");
  }

  std::printf("Test J: reanchor clears dangerous residual\n");
  {
    DeltaTwistFollower follower(fastConfig());
    follower.submitFrame(makeFrame({Vec6{0.20, 0.0, 0.0, 0.0, 0.0, 0.0}}), identityPose());
    tickOpenLoop(follower, POLICY_DT * 0.5);
    Pose6D ref = identityPose();
    ref.x = 0.25;
    ref.y = -0.04;
    ref.z = 0.10;
    follower.reanchor(ref);
    const Pose6D out = follower.tick(TICK);
    check(rb_servo::math::positionDistance(out, ref) < 1e-5, "next output starts near reanchor reference");
    check(linearNorm(follower.diag().pending_delta) < 1e-9, "old pending delta cleared on reanchor");
    check(angularNorm(follower.diag().pending_delta) < 1e-9, "old pending rotation cleared on reanchor");
  }

  std::printf("Test K: telemetry step phase reaches ringdown\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.consume_steps = 1;
    cfg.reserve_steps = 1;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(makeFrame({
        Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    }), identityPose());
    check(follower.diag().step_phase == DeltaTwistStepPhase::Normal, "first step kind normal");
    tickPerfectFeedback(follower, POLICY_DT);
    check(follower.diag().step_phase == DeltaTwistStepPhase::Reserve, "second step kind reserve");
    tickPerfectFeedback(follower, POLICY_DT);
    check(follower.diag().step_phase == DeltaTwistStepPhase::Ringdown, "exhausted zero-residual window reports ringdown");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
