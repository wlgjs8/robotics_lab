#include "rb_servo/control/grasp_commit_coordinator.hpp"

#include <cmath>
#include <cstdio>

using rb_servo::ArmId;
using rb_servo::GraspCommitConfig;
using rb_servo::control::GraspCommitArmInput;
using rb_servo::control::GraspCommitCoordinator;
using rb_servo::control::GraspPhase;
using rb_servo::control::kGraspCloseSoonSourceNearFloorDwellFallback;
using rb_servo::control::kGraspDwellReasonProjectedMotionHigh;

static int g_failures = 0;

static void check(bool ok, const char* name) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

static bool near(double a, double b, double eps = 1e-9) {
  return std::fabs(a - b) <= eps;
}

static GraspCommitConfig baseConfig() {
  GraspCommitConfig cfg;
  cfg.enable = true;
  cfg.close_threshold = 25.0;
  cfg.close_is_greater = false;
  cfg.commit_floor_band_m = 0.015;
  cfg.both_arm_sync_timeout_sec = 0.150;
  cfg.preclose_translation_scale = 0.0;
  cfg.preclose_angular_scale = 0.25;
  cfg.closing_hold_sec = 0.200;
  cfg.lift_height_m = 0.030;
  cfg.lift_duration_sec = 0.350;
  cfg.bimanual_sync = true;
  cfg.close_target = 0.0;
  return cfg;
}

static GraspCommitArmInput nearClose() {
  GraspCommitArmInput in;
  in.follower_active = true;
  in.surface_active = true;
  in.surface_close_soon = true;
  in.surface_min_tip_dist_m = 0.006;
  in.commanded_linear_speed_m_s = 0.0;
  in.commanded_angular_speed_rad_s = 0.0;
  in.projected_linear_norm_m = 0.0;
  in.gripper_cmd = 80.0;
  return in;
}

static GraspCommitArmInput nearNoClose() {
  GraspCommitArmInput in = nearClose();
  in.surface_close_soon = false;
  return in;
}

static GraspCommitArmInput farClose() {
  GraspCommitArmInput in;
  in.follower_active = true;
  in.surface_active = false;
  in.surface_close_soon = true;
  in.surface_min_tip_dist_m = 0.050;
  in.commanded_linear_speed_m_s = 0.0;
  in.commanded_angular_speed_rad_s = 0.0;
  in.projected_linear_norm_m = 0.0;
  in.gripper_cmd = 80.0;
  return in;
}

static GraspCommitArmInput farNoClose() {
  GraspCommitArmInput in = farClose();
  in.surface_close_soon = false;
  return in;
}

int main() {
  std::printf("Test A: enter pregrasp on near floor close soon\n");
  {
    GraspCommitCoordinator coord(baseConfig());
    coord.update(nearClose(), nearNoClose(), 0.002);
    check(coord.phase(ArmId::Left) == GraspPhase::PreGraspCommit,
          "near-floor close intent enters PreGraspCommit");
    const auto cmd = coord.command(ArmId::Left);
    check(cmd.commit_active, "pregrasp reports commit active");
    check(near(cmd.translation_scale, 0.0), "pregrasp translation scale freezes translation");
    check(near(cmd.angular_scale, 0.25), "pregrasp keeps configured angular alignment scale");
    check(cmd.clear_residual, "pregrasp requests residual clear");
  }

  std::printf("Test B: no commit when far from floor\n");
  {
    GraspCommitCoordinator coord(baseConfig());
    coord.update(farClose(), farClose(), 0.002);
    check(coord.phase(ArmId::Left) == GraspPhase::Normal,
          "close intent far from floor stays normal");
    check(!coord.command(ArmId::Left).commit_active, "far close has no commit command");
  }

  std::printf("Test C: close direction supports greater-is-closed configs\n");
  {
    GraspCommitConfig cfg = baseConfig();
    cfg.close_is_greater = true;
    cfg.close_threshold = 0.60;
    GraspCommitCoordinator coord(cfg);
    GraspCommitArmInput in;
    in.follower_active = true;
    in.surface_active = true;
    in.surface_min_tip_dist_m = 0.004;
    in.surface_close_soon = true;
    coord.update(in, nearNoClose(), 0.002);
    check(coord.phase(ArmId::Left) == GraspPhase::PreGraspCommit,
          "greater-is-closed close_soon input enters pregrasp");
    cfg.freeze_gripper_until_commit = true;
    GraspCommitCoordinator freeze_coord(cfg);
    freeze_coord.update(in, nearNoClose(), 0.002);
    const auto freeze_cmd = freeze_coord.command(ArmId::Left);
    check(freeze_cmd.freeze_gripper && !freeze_cmd.gripper_override_active,
          "pregrasp can freeze current gripper without closing before commit");
  }

  std::printf("Test D: bimanual sync waits and then times out\n");
  {
    GraspCommitCoordinator coord(baseConfig());
    coord.update(nearClose(), nearNoClose(), 0.002);
    check(coord.phase(ArmId::Left) == GraspPhase::PreGraspCommit,
          "first arm waits in pregrasp");
    check(coord.phase(ArmId::Right) != GraspPhase::ClosingHold,
          "other arm has not forced close immediately");
    for (int i = 0; i < 80; ++i) {
      coord.update(nearClose(), nearNoClose(), 0.002);
    }
    check(coord.phase(ArmId::Left) == GraspPhase::ClosingHold,
          "timeout starts closing hold for first arm");
    check(coord.phase(ArmId::Right) == GraspPhase::ClosingHold,
          "timeout starts closing hold for both arms together");
    check(coord.command(ArmId::Left).drop_policy_delta,
          "closing hold drops policy deltas");
  }

  std::printf("Test E: closing hold then lift then fresh chunk resume\n");
  {
    GraspCommitConfig cfg = baseConfig();
    cfg.bimanual_sync = false;
    GraspCommitCoordinator coord(cfg);
    coord.update(nearClose(), nearNoClose(), 0.002);
    coord.update(nearClose(), nearNoClose(), 0.002);
    check(coord.phase(ArmId::Left) == GraspPhase::ClosingHold,
          "independent arm enters closing hold after pregrasp tick");
    for (int i = 0; i < 110; ++i) {
      coord.update(nearClose(), nearNoClose(), 0.002);
    }
    check(coord.phase(ArmId::Left) == GraspPhase::LiftOut,
          "closing hold advances to lift out");
    const auto lift_cmd = coord.command(ArmId::Left);
    check(lift_cmd.lift_active, "lift command is active");
    check(lift_cmd.gripper_override_active && near(lift_cmd.gripper_target, cfg.close_target),
          "gripper close override remains active during lift");
    check(lift_cmd.lift_progress >= 0.0 && lift_cmd.lift_progress <= 1.0,
          "lift progress is bounded");
    for (int i = 0; i < 190; ++i) {
      coord.update(nearClose(), nearNoClose(), 0.002);
    }
    check(coord.phase(ArmId::Left) == GraspPhase::ResumeWaitFreshChunk,
          "lift completes into resume-wait");
    GraspCommitArmInput fresh = nearNoClose();
    fresh.fresh_chunk = true;
    fresh.recv_seq = 42;
    coord.update(fresh, nearNoClose(), 0.002);
    check(coord.phase(ArmId::Left) == GraspPhase::SurfaceApproach ||
              coord.phase(ArmId::Left) == GraspPhase::Normal,
          "fresh chunk releases resume wait without consuming stale tail");
  }

  std::printf("Test F: safety block clears into resume wait\n");
  {
    GraspCommitConfig cfg = baseConfig();
    cfg.bimanual_sync = false;
    GraspCommitCoordinator coord(cfg);
    coord.update(nearClose(), nearNoClose(), 0.002);
    GraspCommitArmInput blocked = nearClose();
    blocked.safety_blocked = true;
    coord.update(blocked, nearNoClose(), 0.002);
    const auto cmd = coord.command(ArmId::Left);
    check(coord.phase(ArmId::Left) == GraspPhase::ResumeWaitFreshChunk,
          "safety block moves commit to resume wait");
    check(cmd.clear_residual && cmd.drop_policy_delta,
          "safety block command clears residual and drops stale deltas");
    check(cmd.blocked, "safety block telemetry is reported");
    check(cmd.phase_before_block == GraspPhase::PreGraspCommit,
          "phase before block is preserved for telemetry");
    const double elapsed_before = cmd.sync_wait_sec;
    coord.update(blocked, nearNoClose(), 0.050);
    check(near(coord.command(ArmId::Left).sync_wait_sec, elapsed_before),
          "blocked update does not advance grasp phase timers");
    check(coord.command(ArmId::Left).phase_before_block == GraspPhase::PreGraspCommit,
          "repeated blocked updates keep original phase-before-block telemetry");
  }

  std::printf("Test G: dwell triggers commit when enabled\n");
  {
    GraspCommitConfig cfg = baseConfig();
    cfg.bimanual_sync = false;
    cfg.enable_near_floor_dwell_fallback = true;
    cfg.dwell_min_duration_sec = 0.010;
    GraspCommitCoordinator coord(cfg);
    coord.update(nearNoClose(), farNoClose(), 0.011);
    check(coord.phase(ArmId::Left) == GraspPhase::PreGraspCommit,
          "near-floor dwell enters PreGraspCommit");
    const auto cmd = coord.command(ArmId::Left);
    check(cmd.close_soon, "dwell fallback reports close soon");
    check(cmd.close_soon_source == kGraspCloseSoonSourceNearFloorDwellFallback,
          "dwell fallback uses close source 3");
    check(cmd.dwell_fallback_triggered, "dwell fallback trigger telemetry is set");
    check(cmd.dwell_active, "dwell active telemetry is set");
  }

  std::printf("Test H: dwell does not trigger when disabled\n");
  {
    GraspCommitConfig cfg = baseConfig();
    cfg.bimanual_sync = false;
    cfg.enable_near_floor_dwell_fallback = false;
    cfg.dwell_min_duration_sec = 0.010;
    GraspCommitCoordinator coord(cfg);
    coord.update(nearNoClose(), farNoClose(), 0.011);
    check(coord.phase(ArmId::Left) == GraspPhase::SurfaceApproach,
          "disabled dwell fallback stays in surface approach");
    check(coord.command(ArmId::Left).close_soon_source !=
              kGraspCloseSoonSourceNearFloorDwellFallback,
          "disabled dwell fallback does not emit source 3");
  }

  std::printf("Test I: dwell does not trigger far from floor\n");
  {
    GraspCommitConfig cfg = baseConfig();
    cfg.bimanual_sync = false;
    cfg.enable_near_floor_dwell_fallback = true;
    cfg.dwell_min_duration_sec = 0.010;
    GraspCommitCoordinator coord(cfg);
    coord.update(farNoClose(), farNoClose(), 0.011);
    check(coord.phase(ArmId::Left) == GraspPhase::Normal,
          "far dwell candidate stays normal");
    check(!coord.command(ArmId::Left).dwell_active,
          "far dwell candidate does not report dwell active");
  }

  std::printf("Test J: dwell requires small projected motion\n");
  {
    GraspCommitConfig cfg = baseConfig();
    cfg.bimanual_sync = false;
    cfg.enable_near_floor_dwell_fallback = true;
    cfg.dwell_min_duration_sec = 0.010;
    cfg.dwell_require_projected_motion_small = true;
    GraspCommitCoordinator coord(cfg);
    GraspCommitArmInput moving = nearNoClose();
    moving.projected_linear_norm_m = 0.010;
    coord.update(moving, farNoClose(), 0.011);
    check(coord.phase(ArmId::Left) == GraspPhase::SurfaceApproach,
          "high projected motion does not enter PreGraspCommit");
    check((coord.command(ArmId::Left).dwell_reason &
           kGraspDwellReasonProjectedMotionHigh) != 0,
          "dwell reason marks high projected motion");
  }

  std::printf("Test K: dwell bimanual sync waits and then times out\n");
  {
    GraspCommitConfig cfg = baseConfig();
    cfg.bimanual_sync = true;
    cfg.enable_near_floor_dwell_fallback = true;
    cfg.dwell_min_duration_sec = 0.010;
    cfg.both_arm_sync_timeout_sec = 0.020;
    GraspCommitCoordinator coord(cfg);
    coord.update(nearNoClose(), farNoClose(), 0.011);
    check(coord.phase(ArmId::Left) == GraspPhase::PreGraspCommit,
          "dwelling arm waits in pregrasp");
    check(coord.phase(ArmId::Right) != GraspPhase::ClosingHold,
          "other arm is not forced closed before sync timeout");
    check(coord.command(ArmId::Left).close_soon_source ==
              kGraspCloseSoonSourceNearFloorDwellFallback,
          "bimanual dwell fallback records source 3");
    for (int i = 0; i < 10; ++i) {
      coord.update(nearNoClose(), farNoClose(), 0.002);
    }
    check(coord.phase(ArmId::Left) == GraspPhase::ClosingHold,
          "dwell sync timeout starts closing hold for dwelling arm");
    check(coord.phase(ArmId::Right) == GraspPhase::ClosingHold,
          "dwell sync timeout starts closing hold for both arms together");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
