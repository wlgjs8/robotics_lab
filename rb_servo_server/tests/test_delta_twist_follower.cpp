// test_delta_twist_follower.cpp - local per-frame action delta follower.

#include "rb_servo/control/delta_twist_follower.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <vector>

using rb_servo::Pose6D;
using rb_servo::Vec6;
using rb_servo::control::AxisLimit;
using rb_servo::control::ChunkFrame;
using rb_servo::control::DeltaTwistFollower;
using rb_servo::control::DeltaTwistFollowerConfig;
using rb_servo::control::DeltaTwistStepPhase;
using rb_servo::control::GraspCommitArmCommand;
using rb_servo::control::GraspCloseSoonSource;
using rb_servo::control::GraspPhase;

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

static ChunkFrame makeOpenMotionFrameWithGripHorizon(
    int motion_rows,
    int horizon_rows,
    int close_step
) {
  ChunkFrame f;
  f.policy_dt = POLICY_DT;
  f.wire_seq = 64;
  f.recv_seq = 19;
  f.recv_time = 20.0;
  for (int i = 0; i < motion_rows; ++i) {
    f.delta.push_back(Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    f.grip.push_back(80.0);
  }
  f.has_grip_horizon = true;
  for (int i = 0; i < horizon_rows; ++i) {
    f.grip_horizon.push_back(i == close_step ? 0.0 : 80.0);
  }
  return f;
}

static ChunkFrame makeOpenMotionFrameWithGripHorizonValues(
    int motion_rows,
    std::initializer_list<double> horizon
) {
  ChunkFrame f;
  f.policy_dt = POLICY_DT;
  f.wire_seq = 65;
  f.recv_seq = 20;
  f.recv_time = 21.0;
  for (int i = 0; i < motion_rows; ++i) {
    f.delta.push_back(Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    f.grip.push_back(80.0);
  }
  f.has_grip_horizon = true;
  for (double value : horizon) {
    f.grip_horizon.push_back(value);
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

  std::printf("Test I2: surface projector discards delta instead of residualizing it\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.surface_action_projector.enable = true;
    cfg.surface_action_projector.floor_z_m = 0.0;
    cfg.surface_action_projector.floor_z_m_configured = true;
    cfg.surface_action_projector.soft_floor_margin_m = 0.012;
    cfg.surface_action_projector.stop_floor_margin_m = 0.002;
    cfg.surface_action_projector.max_tangent_delta_near_floor_m = 1.0;
    cfg.surface_action_projector.max_down_delta_near_floor_m = 0.0002;
    cfg.surface_action_projector.max_yaw_delta_near_floor_rad = 1.0;
    cfg.surface_action_projector.max_pitch_roll_delta_near_floor_rad = 1.0;
    DeltaTwistFollower follower(cfg);
    Pose6D near_floor = identityPose();
    near_floor.z = 0.004;
    follower.submitFrame(makeFrame({Vec6{0.0, 0.0, -0.004, 0.0, 0.0, 0.0}}), near_floor);
    check(follower.diag().surface_projection.active, "surface projector activates near floor");
    check(follower.diag().step_delta.z < -0.003, "raw step delta remains visible in telemetry");
    check(follower.diag().projected_step_delta.z > follower.diag().step_delta.z,
          "projected step delta is reduced before pending residual");
    check(std::fabs(follower.pendingDelta().z - follower.diag().projected_step_delta.z) < 1e-12,
          "pending residual receives projected delta only");
    check(follower.diag().surface_projection.discarded_delta_local.z < 0.0,
          "discarded downward delta is reported");
  }

  std::printf("Test I3: late grip horizon close is visible to close_soon\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.surface_action_projector.close_lookahead_steps = 24;
    cfg.surface_action_projector.close_threshold = 25.0;
    cfg.surface_action_projector.close_is_greater = false;
    cfg.grasp_commit.enable = true;
    cfg.grasp_commit.close_lookahead_steps = 24;
    cfg.grasp_commit.close_threshold = 25.0;
    cfg.grasp_commit.close_is_greater = false;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(
        makeOpenMotionFrameWithGripHorizon(10, 24, 16),
        identityPose());
    check(follower.diag().surface_projection.close_soon,
          "late close in full grip horizon is detected");
    check(follower.currentGrip() == 80.0,
          "current gripper target still comes from executable motion row");
    check(follower.windowConsumed() == 1,
          "full grip horizon does not execute extra motion rows");
  }

  std::printf("Test I4: short grip lookahead does not see late close\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.surface_action_projector.close_lookahead_steps = 4;
    cfg.surface_action_projector.close_threshold = 25.0;
    cfg.surface_action_projector.close_is_greater = false;
    cfg.grasp_commit.enable = true;
    cfg.grasp_commit.close_lookahead_steps = 4;
    cfg.grasp_commit.close_threshold = 25.0;
    cfg.grasp_commit.close_is_greater = false;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(
        makeOpenMotionFrameWithGripHorizon(10, 24, 16),
        identityPose());
    check(!follower.diag().surface_projection.close_soon,
          "late close outside short lookahead is ignored");
  }

  std::printf("Test I5: grip horizon summary finds first less-than-threshold close\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.grasp_commit.enable = true;
    cfg.grasp_commit.close_lookahead_steps = 4;
    cfg.grasp_commit.close_threshold = 25.0;
    cfg.grasp_commit.close_is_greater = false;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(
        makeOpenMotionFrameWithGripHorizonValues(4, {80.0, 70.0, 30.0, 20.0}),
        identityPose());
    const auto& summary = follower.diag().grip_horizon;
    check(summary.available, "full grip horizon availability is reported");
    check(summary.len == 4, "full grip horizon length is reported");
    check(summary.current_index == 0, "current horizon index matches consumed delta row");
    check(summary.argmin == 3 && summary.min == 20.0, "horizon min and argmin are reported");
    check(summary.argmax == 0 && summary.max == 80.0, "horizon max and argmax are reported");
    check(summary.first_close_index == 3, "less-than threshold first close index is reported");
    check(summary.first_close_steps_ahead == 3, "less-than threshold steps-ahead is reported");
    check(summary.close_soon_source == static_cast<int>(GraspCloseSoonSource::FullGripHorizon),
          "less-than close source is full grip horizon");
    check(summary.close_soon_steps_ahead == 3, "less-than close_soon steps-ahead is reported");
  }

  std::printf("Test I6: grip horizon summary finds first greater-than-threshold close\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.grasp_commit.enable = true;
    cfg.grasp_commit.close_lookahead_steps = 3;
    cfg.grasp_commit.close_threshold = 60.0;
    cfg.grasp_commit.close_is_greater = true;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(
        makeOpenMotionFrameWithGripHorizonValues(3, {10.0, 20.0, 70.0}),
        identityPose());
    const auto& summary = follower.diag().grip_horizon;
    check(summary.first_close_index == 2, "greater-than threshold first close index is reported");
    check(summary.first_close_steps_ahead == 2, "greater-than threshold steps-ahead is reported");
    check(summary.close_soon_source == static_cast<int>(GraspCloseSoonSource::FullGripHorizon),
          "greater-than close source is full grip horizon");
    check(summary.close_soon_steps_ahead == 2, "greater-than close_soon steps-ahead is reported");
  }

  std::printf("Test I7: grip horizon summary reports no-close sentinel\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.grasp_commit.enable = true;
    cfg.grasp_commit.close_lookahead_steps = 4;
    cfg.grasp_commit.close_threshold = 25.0;
    cfg.grasp_commit.close_is_greater = false;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(
        makeOpenMotionFrameWithGripHorizonValues(3, {80.0, 70.0, 30.0}),
        identityPose());
    const auto& summary = follower.diag().grip_horizon;
    check(summary.first_close_index == -1, "no-close first index sentinel is -1");
    check(summary.first_close_steps_ahead == -1, "no-close steps-ahead sentinel is -1");
    check(summary.close_soon_source == static_cast<int>(GraspCloseSoonSource::None),
          "no-close source is none");
    check(summary.close_soon_steps_ahead == -1, "no-close close_soon steps-ahead sentinel is -1");
  }

  std::printf("Test I8: grip horizon telemetry is safe when no full horizon is present\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.grasp_commit.enable = true;
    cfg.grasp_commit.close_lookahead_steps = 4;
    cfg.grasp_commit.close_threshold = 5.0;
    cfg.grasp_commit.close_is_greater = false;
    DeltaTwistFollower follower(cfg);
    follower.submitFrame(makeFrame({
        Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    }), identityPose());
    const auto& summary = follower.diag().grip_horizon;
    check(!summary.available, "missing full grip horizon is reported");
    check(summary.len == 0, "missing full grip horizon length is zero");
    check(summary.current_index == 0, "missing horizon still reports current policy index");
    check(std::isnan(summary.min) && std::isnan(summary.max), "missing horizon extrema are NaN");
    check(summary.argmin == -1 && summary.argmax == -1, "missing horizon extrema indices are sentinels");
    check(summary.first_close_index == -1, "missing horizon first close index is sentinel");
    check(summary.close_soon_source == static_cast<int>(GraspCloseSoonSource::None),
          "missing horizon without motion-overlay close reports no source");
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

  std::printf("Test K: grasp closing hold drops incoming policy deltas\n");
  {
    DeltaTwistFollower follower(fastConfig());
    const Pose6D start = identityPose();
    follower.submitFrame(makeFrame({
        Vec6{0.03, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.05, 0.0, 0.0, 0.0, 0.0, 0.0},
    }), start);
    follower.setFeedbackPose(start);
    GraspCommitArmCommand hold;
    hold.phase = GraspPhase::ClosingHold;
    hold.commit_active = true;
    hold.drop_policy_delta = true;
    hold.clear_residual = true;
    hold.hold_pose = true;
    hold.gripper_override_active = true;
    hold.gripper_target = 0.0;
    hold.policy_delta_dropped = true;
    follower.setGraspCommitCommand(hold);
    const Pose6D out = tickOpenLoop(follower, POLICY_DT * 1.1);
    check(linearNorm(follower.pendingDelta()) < 1e-12, "closing hold keeps pending residual clear");
    check(follower.windowConsumed() == 2, "closing hold consumes old rows only as dropped deltas");
    check(follower.diag().grasp_commit.policy_delta_dropped, "grasp telemetry reports dropped policy delta");
    check(follower.currentGrip() == 0.0, "closing hold keeps gripper close target");
    check(rb_servo::math::positionDistance(out, start) < 1e-9, "closing hold keeps pose at feedback reference");
  }

  std::printf("Test K2: blocked pause does not consume steps or grow residual\n");
  {
    DeltaTwistFollower follower(fastConfig());
    const Pose6D start = identityPose();
    follower.submitFrame(makeFrame({
        Vec6{0.010, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.020, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.030, 0.0, 0.0, 0.0, 0.0, 0.0},
    }), start);
    const int consumed_before = follower.windowConsumed();
    follower.setBlocked(true, 0x11);
    for (int i = 0; i < 4; ++i) {
      follower.pauseBlocked(start, POLICY_DT);
    }
    check(follower.windowConsumed() == consumed_before, "blocked pause does not consume policy rows");
    check(follower.diag().seg_step_index == 0, "blocked pause leaves current follower_step unchanged");
    check(linearNorm(follower.pendingDelta()) < 1e-12, "blocked pause clears pending residual");
    check(follower.diag().blocked, "blocked telemetry is set");
    check(follower.diag().block_reason == 0x11, "blocked reason bitmask is preserved");
  }

  std::printf("Test K3: block timeout requires fresh chunk before resume\n");
  {
    DeltaTwistFollowerConfig cfg = fastConfig();
    cfg.block_requires_fresh_chunk_sec = 0.010;
    DeltaTwistFollower follower(cfg);
    const Pose6D start = identityPose();
    follower.submitFrame(makeFrame({
        Vec6{0.010, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.020, 0.0, 0.0, 0.0, 0.0, 0.0},
    }), start);
    follower.setBlocked(true, 0x01);
    follower.pauseBlocked(start, 0.020);
    check(follower.diag().block_requires_fresh_chunk, "long block marks current window stale");
    const int consumed_before = follower.windowConsumed();
    follower.setBlocked(false);
    (void)follower.tick(POLICY_DT);
    check(follower.windowConsumed() == consumed_before, "old rows are not consumed after unblock without fresh chunk");
    check(follower.diag().block_requires_fresh_chunk, "fresh chunk is still required after unblock");
    ChunkFrame fresh = makeFrame({
        Vec6{0.003, 0.0, 0.0, 0.0, 0.0, 0.0},
    });
    fresh.recv_seq = 100;
    fresh.wire_seq = 101;
    follower.submitFrame(fresh, start);
    check(!follower.diag().block_requires_fresh_chunk, "fresh chunk clears stale-window requirement");
    check(follower.windowConsumed() == 1, "fresh chunk consumes its first row normally");
    check(follower.diag().seg_recv_seq == 100, "fresh chunk recv_seq is reported");
  }

  std::printf("Test K4: safety block during grasp commit clears residual and keeps gripper stable\n");
  {
    DeltaTwistFollower follower(fastConfig());
    const Pose6D start = identityPose();
    follower.submitFrame(makeFrame({
        Vec6{0.030, 0.0, 0.0, 0.0, 0.0, 0.0},
        Vec6{0.050, 0.0, 0.0, 0.0, 0.0, 0.0},
    }), start);
    GraspCommitArmCommand closing;
    closing.phase = GraspPhase::ClosingHold;
    closing.commit_active = true;
    closing.drop_policy_delta = true;
    closing.clear_residual = true;
    closing.hold_pose = true;
    closing.gripper_override_active = true;
    closing.gripper_target = 0.0;
    closing.policy_delta_dropped = true;
    follower.setGraspCommitCommand(closing);
    follower.setBlocked(true, 0x02);
    follower.pauseBlocked(start, POLICY_DT);
    check(linearNorm(follower.pendingDelta()) < 1e-12, "blocked grasp commit clears residual");
    check(follower.windowConsumed() == 1, "blocked grasp commit does not consume more rows");
    check(follower.currentGrip() == 0.0, "blocked closing keeps stable close gripper target");
    check(follower.diag().grasp_commit.gripper_override_active, "grasp gripper override remains visible");
  }

  std::printf("Test L: grasp lift out moves upward and holds gripper closed\n");
  {
    DeltaTwistFollower follower(fastConfig());
    const Pose6D start = identityPose();
    follower.submitFrame(makeFrame({
        Vec6{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    }), start);
    follower.setFeedbackPose(start);
    GraspCommitArmCommand lift;
    lift.phase = GraspPhase::LiftOut;
    lift.commit_active = true;
    lift.drop_policy_delta = true;
    lift.clear_residual = true;
    lift.lift_active = true;
    lift.gripper_override_active = true;
    lift.gripper_target = 0.0;
    lift.policy_delta_dropped = true;
    lift.lift_height_m = 0.030;
    lift.lift_progress = 0.5;
    follower.setGraspCommitCommand(lift);
    const Pose6D mid = follower.tick(TICK);
    check(mid.z > 0.014 && mid.z < 0.016, "lift progress moves pose in stand +z");
    check(follower.currentGrip() == 0.0, "lift keeps gripper close target");
    lift.lift_progress = 1.0;
    follower.setGraspCommitCommand(lift);
    const Pose6D top = follower.tick(TICK);
    check(top.z > 0.029 && top.z < 0.031, "lift reaches configured height at full progress");
    check(linearNorm(follower.pendingDelta()) < 1e-12, "lift does not accumulate pending residual");
  }

  std::printf("Test M: telemetry step phase reaches ringdown\n");
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
