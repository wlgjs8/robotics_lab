#include "rb_servo/control/grasp_commit_coordinator.hpp"

#include <cmath>
#include <cstdio>

using rb_servo::ArmId;
using rb_servo::GraspCommitConfig;
using rb_servo::control::GraspCommitArmInput;
using rb_servo::control::GraspCommitCoordinator;
using rb_servo::control::GraspPhase;

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

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
