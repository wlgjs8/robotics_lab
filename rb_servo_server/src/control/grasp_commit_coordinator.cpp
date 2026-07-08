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
      cfg_.close_target != cfg.close_target ||
      cfg_.enable_near_floor_dwell_fallback != cfg.enable_near_floor_dwell_fallback ||
      cfg_.dwell_floor_band_m != cfg.dwell_floor_band_m ||
      cfg_.dwell_min_duration_sec != cfg.dwell_min_duration_sec ||
      cfg_.dwell_max_linear_speed_m_s != cfg.dwell_max_linear_speed_m_s ||
      cfg_.dwell_max_angular_speed_rad_s != cfg.dwell_max_angular_speed_rad_s ||
      cfg_.dwell_require_both_arms_near_floor != cfg.dwell_require_both_arms_near_floor ||
      cfg_.dwell_require_projected_motion_small != cfg.dwell_require_projected_motion_small ||
      cfg_.dwell_projected_linear_norm_threshold_m !=
          cfg.dwell_projected_linear_norm_threshold_m ||
      cfg_.dwell_trigger_close_even_without_model_close !=
          cfg.dwell_trigger_close_even_without_model_close;
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

bool GraspCommitCoordinator::dwellFloor(const GraspCommitArmInput& in) const {
  return in.follower_active &&
         std::isfinite(in.surface_min_tip_dist_m) &&
         in.surface_min_tip_dist_m < cfg_.dwell_floor_band_m;
}

bool GraspCommitCoordinator::gripperAlreadyClosed(double gripper_cmd) const {
  if (!std::isfinite(gripper_cmd) || !std::isfinite(cfg_.close_threshold)) return true;
  return cfg_.close_is_greater ? gripper_cmd >= cfg_.close_threshold
                               : gripper_cmd <= cfg_.close_threshold;
}

void GraspCommitCoordinator::updateDwellState(
    ArmState* state,
    const GraspCommitArmInput& input,
    double dt_sec
) {
  if (!state) return;
  state->dwell_ready = false;
  state->dwell_reason = 0;
  if (!cfg_.enable_near_floor_dwell_fallback) {
    state->dwell_active = false;
    state->dwell_elapsed_sec = 0.0;
    return;
  }

  if (!input.follower_active) state->dwell_reason |= kGraspDwellReasonFollowerInactive;
  if (!dwellFloor(input)) state->dwell_reason |= kGraspDwellReasonFarFromFloor;
  if (!std::isfinite(input.commanded_linear_speed_m_s) ||
      input.commanded_linear_speed_m_s > cfg_.dwell_max_linear_speed_m_s) {
    state->dwell_reason |= kGraspDwellReasonLinearMotionHigh;
  }
  if (!std::isfinite(input.commanded_angular_speed_rad_s) ||
      input.commanded_angular_speed_rad_s > cfg_.dwell_max_angular_speed_rad_s) {
    state->dwell_reason |= kGraspDwellReasonAngularMotionHigh;
  }
  if (cfg_.dwell_require_projected_motion_small &&
      (!std::isfinite(input.projected_linear_norm_m) ||
       input.projected_linear_norm_m > cfg_.dwell_projected_linear_norm_threshold_m)) {
    state->dwell_reason |= kGraspDwellReasonProjectedMotionHigh;
  }
  if (gripperAlreadyClosed(input.gripper_cmd)) {
    state->dwell_reason |= kGraspDwellReasonGripperClosedOrUnknown;
  }

  const int blocking_reasons =
      kGraspDwellReasonFollowerInactive |
      kGraspDwellReasonFarFromFloor |
      kGraspDwellReasonLinearMotionHigh |
      kGraspDwellReasonAngularMotionHigh |
      kGraspDwellReasonProjectedMotionHigh |
      kGraspDwellReasonGripperClosedOrUnknown;
  state->dwell_active = (state->dwell_reason & blocking_reasons) == 0;
  if (state->dwell_active) {
    state->dwell_elapsed_sec += std::max(0.0, dt_sec);
  } else {
    state->dwell_elapsed_sec = 0.0;
  }
  if (state->dwell_elapsed_sec >= cfg_.dwell_min_duration_sec) {
    state->dwell_reason |= kGraspDwellReasonDurationMet;
  }
  state->dwell_ready =
      cfg_.dwell_trigger_close_even_without_model_close &&
      state->dwell_active &&
      state->dwell_elapsed_sec >= cfg_.dwell_min_duration_sec &&
      !state->dwell_fallback_triggered &&
      (state->phase == GraspPhase::Normal || state->phase == GraspPhase::SurfaceApproach);
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
    state->commit_close_soon_source = kGraspCloseSoonSourceNone;
    state->commit_close_soon_steps_ahead = -1;
    state->dwell_fallback_triggered = false;
  }
}

void GraspCommitCoordinator::enterPreGrasp(
    ArmState* state,
    int close_soon_source,
    int close_soon_steps_ahead
) {
  if (!state) return;
  enter(state, GraspPhase::PreGraspCommit);
  state->close_soon = true;
  state->close_soon_source = close_soon_source;
  state->close_soon_steps_ahead = close_soon_steps_ahead;
  state->commit_close_soon_source = close_soon_source;
  state->commit_close_soon_steps_ahead = close_soon_steps_ahead;
  if (close_soon_source == kGraspCloseSoonSourceNearFloorDwellFallback) {
    state->dwell_fallback_triggered = true;
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
  state->close_soon_source = input.surface_close_soon
      ? (input.close_soon_source != kGraspCloseSoonSourceNone
             ? input.close_soon_source
             : kGraspCloseSoonSourceMotionOverlayGrip)
      : kGraspCloseSoonSourceNone;
  state->close_soon_steps_ahead = input.surface_close_soon ? input.close_soon_steps_ahead : -1;
  updateDwellState(state, input, dt_sec);

  if (!cfg_.enable || !input.follower_active) {
    enter(state, GraspPhase::Normal);
    state->phase_elapsed_sec = 0.0;
    state->ready = false;
    state->blocked = false;
    state->dwell_active = false;
    state->dwell_elapsed_sec = 0.0;
    state->dwell_ready = false;
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
        enterPreGrasp(state, state->close_soon_source, state->close_soon_steps_ahead);
      } else if (!cfg_.bimanual_sync && state->dwell_ready) {
        enterPreGrasp(state, kGraspCloseSoonSourceNearFloorDwellFallback, -1);
      } else if (state->near_floor) {
        enter(state, GraspPhase::SurfaceApproach);
      }
      break;
    case GraspPhase::SurfaceApproach:
      if (!state->near_floor) {
        enter(state, GraspPhase::Normal);
      } else if (state->close_soon) {
        enterPreGrasp(state, state->close_soon_source, state->close_soon_steps_ahead);
      } else if (!cfg_.bimanual_sync && state->dwell_ready) {
        enterPreGrasp(state, kGraspCloseSoonSourceNearFloorDwellFallback, -1);
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

void GraspCommitCoordinator::applyDwellFallbackBimanual() {
  if (!cfg_.enable || !cfg_.bimanual_sync || !cfg_.enable_near_floor_dwell_fallback ||
      !cfg_.dwell_trigger_close_even_without_model_close) {
    return;
  }

  const auto can_join = [](const ArmState& state) {
    return state.phase == GraspPhase::Normal || state.phase == GraspPhase::SurfaceApproach;
  };
  const auto ready_or_near = [](const ArmState& state) {
    return state.near_floor || state.ready || state.phase == GraspPhase::PreGraspCommit;
  };
  const bool left_candidate = left_.dwell_ready;
  const bool right_candidate = right_.dwell_ready;
  if (!left_candidate && !right_candidate) return;

  if (cfg_.dwell_require_both_arms_near_floor && !(left_.near_floor && right_.near_floor)) {
    return;
  }

  const bool left_can_start =
      left_candidate &&
      (!cfg_.dwell_require_both_arms_near_floor || right_.near_floor);
  const bool right_can_start =
      right_candidate &&
      (!cfg_.dwell_require_both_arms_near_floor || left_.near_floor);

  if (left_can_start && can_join(left_)) {
    enterPreGrasp(&left_, kGraspCloseSoonSourceNearFloorDwellFallback, -1);
    if (right_.near_floor && can_join(right_)) {
      enterPreGrasp(&right_, kGraspCloseSoonSourceNearFloorDwellFallback, -1);
    }
  }
  if (right_can_start && can_join(right_)) {
    enterPreGrasp(&right_, kGraspCloseSoonSourceNearFloorDwellFallback, -1);
    if (left_.near_floor && can_join(left_)) {
      enterPreGrasp(&left_, kGraspCloseSoonSourceNearFloorDwellFallback, -1);
    }
  }

  if (!left_can_start && left_candidate && ready_or_near(right_) && can_join(left_)) {
    enterPreGrasp(&left_, kGraspCloseSoonSourceNearFloorDwellFallback, -1);
  }
  if (!right_can_start && right_candidate && ready_or_near(left_) && can_join(right_)) {
    enterPreGrasp(&right_, kGraspCloseSoonSourceNearFloorDwellFallback, -1);
  }
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
  applyDwellFallbackBimanual();
  applyBimanualSync(dt_sec);
}

GraspCommitArmCommand GraspCommitCoordinator::makeCommand(const ArmState& state) const {
  GraspCommitArmCommand cmd;
  cmd.phase = cfg_.enable ? state.phase : GraspPhase::Normal;
  cmd.commit_active = isCommitPhase(cmd.phase);
  cmd.close_soon = state.close_soon ||
      (isCommitPhase(cmd.phase) && state.commit_close_soon_source != kGraspCloseSoonSourceNone);
  cmd.close_soon_source = state.commit_close_soon_source != kGraspCloseSoonSourceNone
      ? state.commit_close_soon_source
      : state.close_soon_source;
  cmd.close_soon_steps_ahead = state.commit_close_soon_source != kGraspCloseSoonSourceNone
      ? state.commit_close_soon_steps_ahead
      : state.close_soon_steps_ahead;
  cmd.ready = state.ready;
  cmd.sync_wait_sec = state.sync_wait_sec;
  cmd.blocked = state.blocked;
  cmd.phase_before_block = state.phase_before_block;
  cmd.dwell_active = state.dwell_active;
  cmd.dwell_elapsed_sec = state.dwell_elapsed_sec;
  cmd.dwell_fallback_triggered =
      state.dwell_fallback_triggered ||
      state.commit_close_soon_source == kGraspCloseSoonSourceNearFloorDwellFallback;
  cmd.dwell_reason = state.dwell_reason;

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
