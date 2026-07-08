#include "rb_servo/control/surface_action_projector.hpp"
#include "rb_servo/math/se3.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

using rb_servo::ArmId;
using rb_servo::FloorCheckPointConfig;
using rb_servo::Pose6D;
using rb_servo::SurfaceActionProjectorConfig;
using rb_servo::Vec6;
using rb_servo::control::SurfaceActionProjector;
using rb_servo::control::SurfaceMode;
using rb_servo::control::SurfacePhase;

static int g_failures = 0;

static void check(bool ok, const char* name) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

static Pose6D poseAtZ(double z) {
  Pose6D p;
  p.z = z;
  p.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
  return p;
}

static double linearNorm(const Vec6& v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

static double angularNorm(const Vec6& v) {
  return std::sqrt(v.rx * v.rx + v.ry * v.ry + v.rz * v.rz);
}

static bool near(double a, double b, double eps = 1e-9) {
  return std::fabs(a - b) <= eps;
}

static SurfaceActionProjectorConfig baseConfig() {
  SurfaceActionProjectorConfig cfg;
  cfg.enable = true;
  cfg.floor_z_m = 0.0;
  cfg.floor_z_m_configured = true;
  cfg.soft_floor_margin_m = 0.012;
  cfg.stop_floor_margin_m = 0.002;
  cfg.close_floor_band_m = 0.015;
  cfg.min_tip_margin_m = 0.002;
  cfg.max_tangent_delta_near_floor_m = 1.0;
  cfg.max_down_delta_near_floor_m = 1.0;
  cfg.max_yaw_delta_near_floor_rad = 1.0;
  cfg.max_pitch_roll_delta_near_floor_rad = 1.0;
  cfg.close_threshold = 25.0;
  cfg.close_is_greater = false;
  cfg.close_lookahead_steps = 4;
  return cfg;
}

static FloorCheckPointConfig point(std::array<double, 3> offset) {
  FloorCheckPointConfig p;
  p.name = "tip";
  p.offset_m = offset;
  p.offset_closed_m = offset;
  p.has_closed = false;
  return p;
}

int main() {
  std::printf("Test A: far from floor no change\n");
  {
    SurfaceActionProjector projector(baseConfig());
    const Vec6 raw{0.003, -0.002, -0.001, 0.001, -0.002, 0.003};
    const auto out = projector.project(
        ArmId::Left, poseAtZ(0.20), raw, {100.0, 100.0}, 100.0, SurfacePhase::Normal);
    check(near(out.projected_delta_local.x, raw.x), "x unchanged far from floor");
    check(near(out.projected_delta_local.z, raw.z), "z unchanged far from floor");
    check(near(out.projected_delta_local.rz, raw.rz), "rotation unchanged far from floor");
    check(!out.active, "projector inactive far from floor");
  }

  std::printf("Test B: down motion near floor is clamped\n");
  {
    SurfaceActionProjectorConfig cfg = baseConfig();
    cfg.max_down_delta_near_floor_m = 0.0002;
    SurfaceActionProjector projector(cfg);
    const Vec6 raw{0.0, 0.0, -0.0020, 0.0, 0.0, 0.0};
    const auto out = projector.project(
        ArmId::Left, poseAtZ(0.004), raw, {}, 100.0, SurfacePhase::Normal);
    check(out.active, "projector active in soft floor band");
    check(out.projected_delta_local.z > raw.z, "downward local z is reduced");
    check(out.projected_delta_local.z >= -cfg.max_down_delta_near_floor_m - 1e-12,
          "downward z respects per-frame cap");
  }

  std::printf("Test C: tangent coupling when down is blocked\n");
  {
    SurfaceActionProjectorConfig cfg = baseConfig();
    cfg.tangent_coupling = 0.80;
    SurfaceActionProjector projector(cfg);
    const Vec6 raw{0.004, 0.0, -0.004, 0.0, 0.0, 0.0};
    const auto out = projector.project(
        ArmId::Right, poseAtZ(cfg.stop_floor_margin_m), raw, {}, 100.0, SurfacePhase::Normal);
    check(out.down_scale <= 1e-9, "downward scale is zero at stop margin");
    check(out.tangent_scale < 0.25, "tangent scale shrinks when down is blocked");
    check(out.projected_delta_local.x < raw.x * 0.25, "forward tangent delta is reduced");
  }

  std::printf("Test D: close soon freezes translation near floor\n");
  {
    SurfaceActionProjectorConfig cfg = baseConfig();
    cfg.preclose_tangent_scale = 0.0;
    SurfaceActionProjector projector(cfg);
    const Vec6 raw{0.003, 0.0, -0.001, 0.0, 0.0, 0.006};
    const auto out = projector.project(
        ArmId::Left, poseAtZ(0.006), raw, {100.0, 20.0, 20.0}, 100.0, SurfacePhase::Normal);
    check(out.close_soon, "future close detected from opening-percent grip window");
    check(out.mode == SurfaceMode::PreClose, "preclose mode selected near floor");
    check(linearNorm(out.projected_delta_local) < 1e-12, "translation is frozen before close");
    check(near(out.projected_delta_local.rz, raw.rz), "small yaw remains available");
  }

  std::printf("Test E: hull line search prevents floor violation\n");
  {
    SurfaceActionProjectorConfig cfg = baseConfig();
    cfg.soft_floor_margin_m = 0.004;
    cfg.gripper_floor_check_points_tcp.push_back(point({0.050, 0.0, 0.0}));
    cfg.hull_line_search_iters = 12;
    SurfaceActionProjector projector(cfg);
    const Pose6D start = poseAtZ(0.006);
    const Vec6 raw{0.0, 0.0, 0.0, 0.0, 0.20, 0.0};
    const auto out = projector.project(
        ArmId::Left, start, raw, {}, 100.0, SurfacePhase::Normal);
    const Pose6D candidate = rb_servo::math::composeDeltaLocal(start, Pose6D{
        out.projected_delta_local.x,
        out.projected_delta_local.y,
        out.projected_delta_local.z,
        out.projected_delta_local.rx,
        out.projected_delta_local.ry,
        out.projected_delta_local.rz
    });
    const auto R = rb_servo::math::rotationFromPose(candidate);
    const rb_servo::math::Vector3 tcp(candidate.x, candidate.y, candidate.z);
    const rb_servo::math::Vector3 tip = tcp + R * rb_servo::math::Vector3(0.050, 0.0, 0.0);
    check(out.hull_scaled, "hull line search scaled risky rotation");
    check(out.hull_alpha < 1.0, "hull alpha reports scale-down");
    check(tip.z() >= cfg.min_tip_margin_m - 1e-6, "tip remains above floor margin");
  }

  std::printf("Test F: disabled projector leaves delta unchanged\n");
  {
    SurfaceActionProjectorConfig cfg = baseConfig();
    cfg.enable = false;
    SurfaceActionProjector projector(cfg);
    const Vec6 raw{0.004, 0.0, -0.004, 0.1, 0.2, 0.3};
    const auto out = projector.project(
        ArmId::Left, poseAtZ(0.001), raw, {0.0}, 0.0, SurfacePhase::Normal);
    check(near(out.projected_delta_local.x, raw.x), "disabled x unchanged");
    check(near(out.projected_delta_local.z, raw.z), "disabled z unchanged");
    check(near(out.projected_delta_local.ry, raw.ry), "disabled rotation unchanged");
    check(!out.active, "disabled projector reports inactive");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
