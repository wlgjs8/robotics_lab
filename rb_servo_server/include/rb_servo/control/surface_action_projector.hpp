#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

#include <limits>
#include <vector>

namespace rb_servo::control {

enum class SurfaceMode {
  Normal = 0,
  SurfaceApproach = 1,
  PreClose = 2,
  HullScaled = 3,
};

enum class SurfacePhase {
  Normal = 0,
  Reserve = 1,
};

struct SurfaceProjectionResult {
  Vec6 raw_delta_local{};
  Vec6 projected_delta_local{};
  Vec6 discarded_delta_local{};

  SurfaceMode mode{SurfaceMode::Normal};
  bool active{false};
  bool close_soon{false};
  bool hull_scaled{false};

  double min_tip_dist_m{std::numeric_limits<double>::quiet_NaN()};
  double down_scale{1.0};
  double tangent_scale{1.0};
  double hull_alpha{1.0};
};

class SurfaceActionProjector {
 public:
  explicit SurfaceActionProjector(const SurfaceActionProjectorConfig& cfg);

  void reconfigure(const SurfaceActionProjectorConfig& cfg);
  const SurfaceActionProjectorConfig& config() const { return cfg_; }

  SurfaceProjectionResult project(
      ArmId side,
      const Pose6D& current_tcp_pose_stand,
      const Vec6& raw_delta_local,
      const std::vector<double>& future_grip_window,
      double current_grip,
      SurfacePhase phase
  ) const;

 private:
  SurfaceActionProjectorConfig cfg_;
};

}  // namespace rb_servo::control
