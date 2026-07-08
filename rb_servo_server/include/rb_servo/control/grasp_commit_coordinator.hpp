#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

#include <cstdint>

namespace rb_servo::control {

enum class GraspPhase {
  Normal = 0,
  SurfaceApproach = 1,
  PreGraspCommit = 2,
  ClosingHold = 3,
  LiftOut = 4,
  ResumeWaitFreshChunk = 5,
};

struct GraspCommitArmInput {
  bool follower_active{false};
  bool surface_active{false};
  bool surface_close_soon{false};
  double surface_min_tip_dist_m{0.0};
  bool fresh_chunk{false};
  bool safety_blocked{false};
  std::uint64_t recv_seq{0};
};

struct GraspCommitArmCommand {
  GraspPhase phase{GraspPhase::Normal};
  bool commit_active{false};
  bool close_soon{false};
  bool ready{false};
  double sync_wait_sec{0.0};
  double closing_hold_elapsed_sec{0.0};
  double lift_elapsed_sec{0.0};
  double lift_progress{0.0};

  bool gripper_override_active{false};
  double gripper_target{0.0};
  bool freeze_gripper{false};
  bool policy_delta_dropped{false};
  bool resume_wait_fresh_chunk{false};
  bool blocked{false};
  GraspPhase phase_before_block{GraspPhase::Normal};

  bool drop_policy_delta{false};
  bool clear_residual{false};
  bool hold_pose{false};
  bool lift_active{false};
  double lift_height_m{0.0};
  double translation_scale{1.0};
  double angular_scale{1.0};
};

class GraspCommitCoordinator {
 public:
  explicit GraspCommitCoordinator(const GraspCommitConfig& cfg);

  void reconfigure(const GraspCommitConfig& cfg);
  const GraspCommitConfig& config() const { return cfg_; }
  void reset();

  void update(
      const GraspCommitArmInput& left,
      const GraspCommitArmInput& right,
      double dt_sec
  );

  GraspCommitArmCommand command(ArmId arm) const;
  GraspPhase phase(ArmId arm) const;

 private:
  struct ArmState {
    GraspPhase phase{GraspPhase::Normal};
    double phase_elapsed_sec{0.0};
    double sync_wait_sec{0.0};
    bool ready{false};
    bool close_soon{false};
    bool near_floor{false};
    bool blocked{false};
    GraspPhase phase_before_block{GraspPhase::Normal};
  };

  bool nearFloor(const GraspCommitArmInput& in) const;
  void enter(ArmState* state, GraspPhase phase);
  void updateArm(
      ArmState* state,
      const GraspCommitArmInput& input,
      double dt_sec
  );
  void applyBimanualSync(double dt_sec);
  GraspCommitArmCommand makeCommand(const ArmState& state) const;

  GraspCommitConfig cfg_;
  ArmState left_;
  ArmState right_;
};

int graspPhaseKind(GraspPhase phase);

}  // namespace rb_servo::control
