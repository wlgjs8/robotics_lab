#include "rb_servo/control/grasp_commit_coordinator.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo::control {
namespace {

double clamp01(double value) {
  if (!std::isfinite(value)) return 0.0;
  return std::clamp(value, 0.0, 1.0);
}

bool isCommitPhase(GraspPhase phase) {
  return phase == GraspPhase::PreGraspCommit ||
         phase == GraspPhase::ClosingHold ||
         phase == GraspPhase::LiftOut ||
         phase == GraspPhase::ResumeWaitFreshChunk;
}

}  // namespace

int graspPhaseKind(GraspPhase phase) {
  switch (phase) {
    case GraspPhase::SurfaceApproach:
      return 1;
    case GraspPhase::PreGraspCommit:
      return 2;
    case GraspPhase::ClosingHold:
      return 3;
    case GraspPhase::LiftOut:
      return 4;
    case GraspPhase::ResumeWaitFreshChunk:
      return 5;
    case GraspPhase::Normal:
    default:
      return 0;
  }
}

GraspCommitCoordinator::GraspCommitCoordinator(const GraspCommitConfig& cfg)
    : cfg_(cfg) {}

void GraspCommitCoordinator::reconfigure(const GraspCommitConfig& cfg) {
  const bool changed =
      cfg_.enable != cfg.enable ||
      cfg_.close_threshold != cfg.close_threshold ||
      cfg_.close_is_greater != cfg.close_is_greater ||
      cfg_.close_lookahead_steps != cfg.close_lookahead_steps ||
      cfg_.commit_floor_band_m != cfg.commit_floor_band_m ||
      cfg_.both_arm_sync_timeout_sec != cfg.both_arm_sync_timeout_sec ||
      cfg_.preclose_translation_scale != cfg.preclose_translation_scale ||
      cfg_.preclose_angular_scale != cfg.preclose_angular_scale ||
      cfg_.closing_hold_sec != cfg.closing_hold_sec ||
      cfg_.lift_height_m != cfg.lift_height_m ||
      cfg_.lift_duration_sec != cfg.lift_duration_sec ||
      cfg_.bimanual_sync != cfg.bimanual_sync ||
      cfg_.freeze_gripper_until_commit != cfg.freeze_gripper_until_commit ||
      cfg_.close_target != cfg.close_target;
  cfg_ = cfg;
  if (changed) {
    reset();
  }
}

void GraspCommitCoordinator::reset() {
  left_ = ArmState{};
  right_ = ArmState{};
}

bool GraspCommitCoordinator::nearFloor(const GraspCommitArmInput& in) const {
  if (!in.follower_active) return false;
  if (in.surface_active) return true;
  return std::isfinite(in.surface_min_tip_dist_m) &&
         in.surface_min_tip_dist_m < cfg_.commit_floor_band_m;
}

void GraspCommitCoordinator::enter(ArmState* state, GraspPhase phase) {
  if (!state || state->phase == phase) return;
  state->phase = phase;
  state->phase_elapsed_sec = 0.0;
  state->blocked = false;
  if (phase == GraspPhase::PreGraspCommit) {
    state->sync_wait_sec = 0.0;
    state->ready = true;
  }
  if (phase == GraspPhase::Normal) {
    state->sync_wait_sec = 0.0;
    state->ready = false;
    state->phase_before_block = GraspPhase::Normal;
  }
}

void GraspCommitCoordinator::updateArm(
    ArmState* state,
    const GraspCommitArmInput& input,
    double dt_sec
) {
  if (!state) return;
  state->near_floor = nearFloor(input);
  state->close_soon = input.surface_close_soon;

  if (!cfg_.enable || !input.follower_active) {
    enter(state, GraspPhase::Normal);
    state->phase_elapsed_sec = 0.0;
    state->ready = false;
    state->blocked = false;
    return;
  }

  if (input.safety_blocked) {
    state->blocked = true;
    if (isCommitPhase(state->phase)) {
      if (state->phase != GraspPhase::ResumeWaitFreshChunk) {
        state->phase_before_block = state->phase;
      }
      enter(state, GraspPhase::ResumeWaitFreshChunk);
      state->blocked = true;
    }
    return;
  }
  state->blocked = false;

  if (state->phase == GraspPhase::ResumeWaitFreshChunk && input.fresh_chunk) {
    enter(state, GraspPhase::Normal);
  }

  switch (state->phase) {
    case GraspPhase::Normal:
      if (state->near_floor && state->close_soon) {
        enter(state, GraspPhase::PreGraspCommit);
      } else if (state->near_floor) {
        enter(state, GraspPhase::SurfaceApproach);
      }
      break;
    case GraspPhase::SurfaceApproach:
      if (!state->near_floor) {
        enter(state, GraspPhase::Normal);
      } else if (state->close_soon) {
        enter(state, GraspPhase::PreGraspCommit);
      }
      break;
    case GraspPhase::PreGraspCommit:
      state->ready = true;
      if (!cfg_.bimanual_sync && state->phase_elapsed_sec > 0.0) {
        enter(state, GraspPhase::ClosingHold);
      }
      break;
    case GraspPhase::ClosingHold:
      if (state->phase_elapsed_sec >= cfg_.closing_hold_sec) {
        enter(state, GraspPhase::LiftOut);
      }
      break;
    case GraspPhase::LiftOut:
      if (state->phase_elapsed_sec >= cfg_.lift_duration_sec) {
        enter(state, GraspPhase::ResumeWaitFreshChunk);
      }
      break;
    case GraspPhase::ResumeWaitFreshChunk:
      break;
  }

  state->phase_elapsed_sec += std::max(0.0, dt_sec);
}

void GraspCommitCoordinator::applyBimanualSync(double dt_sec) {
  if (!cfg_.enable || !cfg_.bimanual_sync) return;
  const bool left_waiting = left_.phase == GraspPhase::PreGraspCommit;
  const bool right_waiting = right_.phase == GraspPhase::PreGraspCommit;
  if (!left_waiting && !right_waiting) return;

  if (left_waiting) left_.sync_wait_sec += std::max(0.0, dt_sec);
  if (right_waiting) right_.sync_wait_sec += std::max(0.0, dt_sec);

  const bool both_ready = left_waiting && right_waiting;
  const bool timeout =
      (left_waiting && left_.sync_wait_sec >= cfg_.both_arm_sync_timeout_sec) ||
      (right_waiting && right_.sync_wait_sec >= cfg_.both_arm_sync_timeout_sec);
  if (both_ready || timeout) {
    if (left_.phase != GraspPhase::ResumeWaitFreshChunk) {
      enter(&left_, GraspPhase::ClosingHold);
    }
    if (right_.phase != GraspPhase::ResumeWaitFreshChunk) {
      enter(&right_, GraspPhase::ClosingHold);
    }
  }
}

void GraspCommitCoordinator::update(
    const GraspCommitArmInput& left,
    const GraspCommitArmInput& right,
    double dt_sec
) {
  updateArm(&left_, left, dt_sec);
  updateArm(&right_, right, dt_sec);
  applyBimanualSync(dt_sec);
}

GraspCommitArmCommand GraspCommitCoordinator::makeCommand(const ArmState& state) const {
  GraspCommitArmCommand cmd;
  cmd.phase = cfg_.enable ? state.phase : GraspPhase::Normal;
  cmd.commit_active = isCommitPhase(cmd.phase);
  cmd.close_soon = state.close_soon;
  cmd.ready = state.ready;
  cmd.sync_wait_sec = state.sync_wait_sec;
  cmd.blocked = state.blocked;
  cmd.phase_before_block = state.phase_before_block;

  if (!cfg_.enable) return cmd;

  switch (cmd.phase) {
    case GraspPhase::PreGraspCommit:
      cmd.translation_scale = clamp01(cfg_.preclose_translation_scale);
      cmd.angular_scale = clamp01(cfg_.preclose_angular_scale);
      cmd.clear_residual = true;
      cmd.policy_delta_dropped = cmd.translation_scale <= 0.0;
      if (cfg_.freeze_gripper_until_commit) {
        cmd.freeze_gripper = true;
      }
      break;
    case GraspPhase::ClosingHold:
      cmd.drop_policy_delta = true;
      cmd.clear_residual = true;
      cmd.hold_pose = true;
      cmd.policy_delta_dropped = true;
      cmd.closing_hold_elapsed_sec = state.phase_elapsed_sec;
      cmd.gripper_override_active = true;
      cmd.gripper_target = cfg_.close_target;
      break;
    case GraspPhase::LiftOut:
      cmd.drop_policy_delta = true;
      cmd.clear_residual = true;
      cmd.lift_active = true;
      cmd.policy_delta_dropped = true;
      cmd.lift_elapsed_sec = state.phase_elapsed_sec;
      cmd.lift_progress =
          cfg_.lift_duration_sec > 0.0 ? clamp01(state.phase_elapsed_sec / cfg_.lift_duration_sec)
                                       : 1.0;
      cmd.lift_height_m = cfg_.lift_height_m;
      cmd.gripper_override_active = true;
      cmd.gripper_target = cfg_.close_target;
      break;
    case GraspPhase::ResumeWaitFreshChunk:
      cmd.drop_policy_delta = true;
      cmd.clear_residual = true;
      cmd.hold_pose = true;
      cmd.policy_delta_dropped = true;
      cmd.resume_wait_fresh_chunk = true;
      cmd.gripper_override_active = true;
      cmd.gripper_target = cfg_.close_target;
      break;
    case GraspPhase::SurfaceApproach:
    case GraspPhase::Normal:
    default:
      break;
  }
  return cmd;
}

GraspCommitArmCommand GraspCommitCoordinator::command(ArmId arm) const {
  return makeCommand(arm == ArmId::Left ? left_ : right_);
}

GraspPhase GraspCommitCoordinator::phase(ArmId arm) const {
  return arm == ArmId::Left ? left_.phase : right_.phase;
}

}  // namespace rb_servo::control
