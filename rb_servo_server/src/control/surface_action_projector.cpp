#include "rb_servo/control/surface_action_projector.hpp"

#include "rb_servo/control/floor_constraint.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rb_servo::control {
namespace {

constexpr double kEps = 1e-12;

Vec6 sub(const Vec6& a, const Vec6& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z, a.rx - b.rx, a.ry - b.ry, a.rz - b.rz};
}

Vec6 mul(const Vec6& a, double s) {
  return {a.x * s, a.y * s, a.z * s, a.rx * s, a.ry * s, a.rz * s};
}

Pose6D deltaPose(const Vec6& delta) {
  Pose6D p;
  p.x = delta.x;
  p.y = delta.y;
  p.z = delta.z;
  p.rx = delta.rx;
  p.ry = delta.ry;
  p.rz = delta.rz;
  return p;
}

bool finiteVec6(const Vec6& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
         std::isfinite(v.rx) && std::isfinite(v.ry) && std::isfinite(v.rz);
}

bool finitePose(const Pose6D& pose) {
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z)) {
    return false;
  }
  try {
    return math::rotationFromPose(pose).allFinite();
  } catch (...) {
    return false;
  }
}

double smoothScale(double distance, double stop_margin, double soft_margin) {
  if (!std::isfinite(distance)) return 1.0;
  if (distance <= stop_margin) return 0.0;
  if (distance >= soft_margin) return 1.0;
  const double denom = std::max(soft_margin - stop_margin, kEps);
  const double t = std::clamp((distance - stop_margin) / denom, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

math::Vector3 floorNormal(const SurfaceActionProjectorConfig& cfg) {
  math::Vector3 n(
      cfg.floor_normal_stand[0],
      cfg.floor_normal_stand[1],
      cfg.floor_normal_stand[2]
  );
  const double norm = n.norm();
  if (!std::isfinite(norm) || norm <= kEps) {
    return math::Vector3(0.0, 0.0, 1.0);
  }
  return n / norm;
}

double signedFloorDistance(
    const Pose6D& pose,
    const SurfaceActionProjectorConfig& cfg,
    const std::vector<FloorCheckPointConfig>& points,
    double current_grip
) {
  if (!finitePose(pose)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const math::Vector3 n = floorNormal(cfg);
  const math::Vector3 floor_point(0.0, 0.0, cfg.floor_z_m);
  const math::Vector3 tcp(pose.x, pose.y, pose.z);
  double min_dist = n.dot(tcp - floor_point);
  if (!std::isfinite(min_dist)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (!points.empty()) {
    std::vector<FloorCheckPointConfig> interp;
    interpolateOffsetPoints(points, current_grip, interp);
    const math::Matrix3 R = math::rotationFromPose(pose);
    for (const FloorCheckPointConfig& point : interp) {
      const math::Vector3 offset(point.offset_m[0], point.offset_m[1], point.offset_m[2]);
      const math::Vector3 world = tcp + R * offset;
      const double dist = n.dot(world - floor_point);
      if (!std::isfinite(dist)) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      min_dist = std::min(min_dist, dist);
    }
  }
  return min_dist;
}

bool closeSoon(
    const SurfaceActionProjectorConfig& cfg,
    const std::vector<double>& future_grip_window
) {
  if (cfg.close_lookahead_steps <= 0 || future_grip_window.empty()) {
    return false;
  }
  const std::size_t n = std::min(
      static_cast<std::size_t>(cfg.close_lookahead_steps),
      future_grip_window.size()
  );
  bool found = false;
  double extreme = cfg.close_is_greater
      ? -std::numeric_limits<double>::infinity()
      : std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    const double value = future_grip_window[i];
    if (!std::isfinite(value)) continue;
    found = true;
    if (cfg.close_is_greater) {
      extreme = std::max(extreme, value);
    } else {
      extreme = std::min(extreme, value);
    }
  }
  if (!found) return false;
  return cfg.close_is_greater ? extreme >= cfg.close_threshold
                              : extreme <= cfg.close_threshold;
}

bool clampNorm(math::Vector3* v, double max_norm) {
  if (!v) return false;
  const double n = v->norm();
  if (!std::isfinite(n) || max_norm <= 0.0) {
    const bool changed = v->squaredNorm() > 0.0;
    *v = math::Vector3::Zero();
    return changed;
  }
  if (n <= max_norm) return false;
  *v *= max_norm / n;
  return true;
}

}  // namespace

SurfaceActionProjector::SurfaceActionProjector(const SurfaceActionProjectorConfig& cfg)
    : cfg_(cfg) {}

void SurfaceActionProjector::reconfigure(const SurfaceActionProjectorConfig& cfg) {
  cfg_ = cfg;
}

SurfaceProjectionResult SurfaceActionProjector::project(
    ArmId side,
    const Pose6D& current_tcp_pose_stand,
    const Vec6& raw_delta_local,
    const std::vector<double>& future_grip_window,
    double current_grip,
    SurfacePhase phase
) const {
  (void)side;
  (void)phase;

  SurfaceProjectionResult result;
  result.raw_delta_local = raw_delta_local;
  result.projected_delta_local = raw_delta_local;
  result.discarded_delta_local = Vec6{};
  result.close_soon = closeSoon(cfg_, future_grip_window);

  if (!cfg_.enable) {
    return result;
  }
  if (!finiteVec6(raw_delta_local) || !finitePose(current_tcp_pose_stand)) {
    result.projected_delta_local = Vec6{};
    result.discarded_delta_local = raw_delta_local;
    result.active = true;
    return result;
  }

  result.min_tip_dist_m = signedFloorDistance(
      current_tcp_pose_stand,
      cfg_,
      cfg_.gripper_floor_check_points_tcp,
      current_grip
  );
  if (!std::isfinite(result.min_tip_dist_m)) {
    return result;
  }

  const math::Vector3 n = floorNormal(cfg_);
  const math::Matrix3 R_tcp_stand = math::rotationFromPose(current_tcp_pose_stand);
  math::Vector3 d_world =
      R_tcp_stand * math::Vector3(raw_delta_local.x, raw_delta_local.y, raw_delta_local.z);
  const double dn = d_world.dot(n);
  math::Vector3 d_normal = dn * n;
  math::Vector3 d_tangent = d_world - d_normal;
  const bool near_floor = result.min_tip_dist_m < cfg_.soft_floor_margin_m;

  if (near_floor) {
    result.active = true;
    result.mode = SurfaceMode::SurfaceApproach;
    if (dn < 0.0) {
      result.down_scale = smoothScale(
          result.min_tip_dist_m,
          cfg_.stop_floor_margin_m,
          cfg_.soft_floor_margin_m
      );
      d_normal *= result.down_scale;
      const double blocked = 1.0 - result.down_scale;
      result.tangent_scale = std::clamp(1.0 - cfg_.tangent_coupling * blocked, 0.0, 1.0);
      d_tangent *= result.tangent_scale;
    }
    if (result.close_soon) {
      if (result.min_tip_dist_m < cfg_.close_floor_band_m) {
        result.mode = SurfaceMode::PreClose;
        result.down_scale = 0.0;
        d_normal.setZero();
        result.tangent_scale *= std::clamp(cfg_.preclose_tangent_scale, 0.0, 1.0);
        d_tangent *= std::clamp(cfg_.preclose_tangent_scale, 0.0, 1.0);
      } else {
        result.tangent_scale *= std::clamp(cfg_.close_tangent_scale, 0.0, 1.0);
        d_tangent *= std::clamp(cfg_.close_tangent_scale, 0.0, 1.0);
      }
    }

    clampNorm(&d_tangent, cfg_.max_tangent_delta_near_floor_m);
    const double dn_projected = d_normal.dot(n);
    if (dn_projected < -cfg_.max_down_delta_near_floor_m) {
      d_normal = -std::max(0.0, cfg_.max_down_delta_near_floor_m) * n;
    }
  }

  const math::Vector3 projected_world = d_tangent + d_normal;
  const math::Vector3 projected_local = R_tcp_stand.transpose() * projected_world;
  Vec6 projected{
      projected_local.x(),
      projected_local.y(),
      projected_local.z(),
      raw_delta_local.rx,
      raw_delta_local.ry,
      raw_delta_local.rz,
  };

  if (near_floor) {
    // OpenPI action rows carry local/body rotvec deltas. Near the floor we treat
    // local rz as the yaw-like component and keep rx/ry conservative so pitch or
    // roll cannot sweep a fingertip below the hard floor before the final guard.
    math::Vector3 roll_pitch(projected.rx, projected.ry, 0.0);
    clampNorm(&roll_pitch, cfg_.max_pitch_roll_delta_near_floor_rad);
    projected.rx = roll_pitch.x();
    projected.ry = roll_pitch.y();
    projected.rz = std::clamp(
        projected.rz,
        -std::max(0.0, cfg_.max_yaw_delta_near_floor_rad),
        std::max(0.0, cfg_.max_yaw_delta_near_floor_rad)
    );
  }

  if (cfg_.hull_line_search_iters > 0) {
    const auto safe_at_alpha = [&](double alpha) {
      const Pose6D candidate =
          math::composeDeltaLocal(current_tcp_pose_stand, deltaPose(mul(projected, alpha)));
      const double dist = signedFloorDistance(
          candidate,
          cfg_,
          cfg_.gripper_floor_check_points_tcp,
          current_grip
      );
      return std::isfinite(dist) && dist >= cfg_.min_tip_margin_m;
    };
    if (!safe_at_alpha(1.0)) {
      double alpha = 0.0;
      if (safe_at_alpha(0.0)) {
        double lo = 0.0;
        double hi = 1.0;
        for (int i = 0; i < cfg_.hull_line_search_iters; ++i) {
          const double mid = 0.5 * (lo + hi);
          if (safe_at_alpha(mid)) {
            lo = mid;
          } else {
            hi = mid;
          }
        }
        alpha = lo;
      }
      projected = mul(projected, alpha);
      result.hull_scaled = true;
      result.hull_alpha = alpha;
      result.active = true;
      if (result.mode == SurfaceMode::Normal) {
        result.mode = SurfaceMode::HullScaled;
      }
    }
  }

  result.projected_delta_local = projected;
  result.discarded_delta_local = sub(raw_delta_local, projected);
  return result;
}

}  // namespace rb_servo::control
