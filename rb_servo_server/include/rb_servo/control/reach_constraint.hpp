#pragma once

// Stand-frame reachable-shell (workspace reach limit) decision logic. Pure
// functions: the servo loop supplies a candidate TCP pose (from FK), the arm's
// base origin in the stand frame, and the config; this module computes, for the
// inner (r_min) and outer (r_max) spherical shells, the radial margin of the
// most-exposed checked point (TCP + configured TCP-frame offset points) and a
// per-arm Allow / Hold / Latch decision plus the radial unit direction of each
// binding point (used to build its stand-frame velocity Jacobian for the
// projection). The shell is the radial generalization of safety.roi_box (an AABB)
// centered on the shoulder: it bounds how far the TCP can be from the arm base so
// a Cartesian command never asks for a pose past the arm's reach (where IK fails
// and the legacy behavior was to silently stop). Keeping the decision pure makes
// the policy unit-testable without a backend; the servo loop enforces it with the
// shared velocity-damper projection (mirror of decideFloorAction/decideRoiAction).

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/floor_constraint.hpp"  // FloorAction, floorActionForPolicy, FloorCheckPointConfig
#include "rb_servo/core/types.hpp"
#include "rb_servo/math/se3.hpp"

namespace rb_servo {

// Shell index: 0 = inner (r_min, lower), 1 = outer (r_max, upper). Names match the
// published telemetry / GUI labels.
inline const char* reachShellName(int shell) {
    return shell == 0 ? "r_min" : "r_max";
}

// Per-shell evaluation: signed radial margin (>= 0 inside the shell, < 0 outside)
// of the most-exposed checked point against that shell, plus that point's
// TCP-frame offset and its stand-frame radial unit direction (from the arm base
// to the point) used to build the stand-direction velocity Jacobian.
struct ReachShellEval {
    double margin_m = std::numeric_limits<double>::quiet_NaN();
    math::Vector3 offset_tcp = math::Vector3::Zero();      // {0,0,0} = the TCP point
    std::array<double, 3> dir_stand{0.0, 0.0, 0.0};        // radial unit, stand frame
};

// FK evaluation of one arm against the reach shell. checked=false means the TCP
// pose could not be evaluated (non-finite) — the caller fails closed when the
// constraint is enabled.
struct ReachArmEvaluation {
    bool checked = false;
    bool violated = false;  // any shell margin < 0 (some checked point is outside)
    double min_margin_m = std::numeric_limits<double>::quiet_NaN();  // closest shell
    std::string closest_shell = "r_max";
    ReachShellEval shells[2];     // [0]=inner (r_min), [1]=outer (r_max)
    double r_near_m = std::numeric_limits<double>::quiet_NaN();  // nearest point radius
    double r_far_m = std::numeric_limits<double>::quiet_NaN();   // farthest point radius
    // Sum over shells of max(0, -margin): total "how far outside" the shell. Used
    // as the escape metric (a candidate outside is allowed when it is not getting
    // deeper outside vs the previous sent pose).
    double outside_depth_m = 0.0;
};

// Escape epsilon (mirror of kFloorEscapeEpsilonM / kRoiEscapeEpsilonM): a
// candidate outside a shell is allowed when its outside-depth does not increase vs
// the previous sent pose, so an arm already outside can slide back in / tangentially.
inline constexpr double kReachEscapeEpsilonM = 1e-4;

// Evaluate the TCP (+ each TCP-frame offset point) against the stand-frame shell
// [r_min_m, r_max_m] centered at base_stand. Fills *out and returns true (checked).
// Returns false and leaves out->checked=false when any point coordinate is
// non-finite. r_min_m <= 0 disables the inner shell (its margin stays +inf).
inline bool reachEvaluateShell(
    const Pose6D& tcp_stand,
    const std::array<double, 3>& base_stand,
    const std::vector<FloorCheckPointConfig>& tcp_offset_points,
    double r_min_m,
    double r_max_m,
    ReachArmEvaluation* out
) {
    ReachArmEvaluation r;
    if (!std::isfinite(tcp_stand.x) || !std::isfinite(tcp_stand.y) ||
        !std::isfinite(tcp_stand.z)) {
        if (out) *out = r;  // checked=false
        return false;
    }
    // Gather every checked point's stand-frame coordinates and TCP-frame offset.
    struct Pt {
        std::array<double, 3> p{};
        math::Vector3 off = math::Vector3::Zero();
    };
    std::vector<Pt> pts;
    pts.push_back(Pt{{tcp_stand.x, tcp_stand.y, tcp_stand.z}, math::Vector3::Zero()});
    if (!tcp_offset_points.empty()) {
        const math::Matrix3 rotation = math::rotationFromPose(tcp_stand);
        for (const FloorCheckPointConfig& point : tcp_offset_points) {
            const math::Vector3 offset(point.offset_m[0], point.offset_m[1], point.offset_m[2]);
            const math::Vector3 w = rotation * offset;
            const std::array<double, 3> p{tcp_stand.x + w.x(), tcp_stand.y + w.y(),
                                          tcp_stand.z + w.z()};
            if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {
                if (out) *out = r;  // checked=false
                return false;
            }
            pts.push_back(Pt{p, offset});
        }
    }
    // Radial distance of each point from the arm base, and the near/far binders.
    const auto radius = [&](const std::array<double, 3>& p) {
        const double dx = p[0] - base_stand[0];
        const double dy = p[1] - base_stand[1];
        const double dz = p[2] - base_stand[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };
    std::size_t i_near = 0;
    std::size_t i_far = 0;
    double r_near = radius(pts[0].p);
    double r_far = r_near;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const double ri = radius(pts[i].p);
        if (ri < r_near) { r_near = ri; i_near = i; }
        if (ri > r_far) { r_far = ri; i_far = i; }
    }
    // Radial unit direction (stand frame) from the base to a point. Falls back to
    // +x at the center to keep a finite, normalized direction.
    const auto radial_unit = [&](const std::array<double, 3>& p, double rad) -> std::array<double, 3> {
        if (!(rad > 1e-9)) return {1.0, 0.0, 0.0};
        return {(p[0] - base_stand[0]) / rad, (p[1] - base_stand[1]) / rad,
                (p[2] - base_stand[2]) / rad};
    };

    const bool inner_active = r_min_m > 0.0;
    // Inner shell (lower): nearest point must stay >= r_min. margin = r_near - r_min.
    const double m_lo = inner_active ? (r_near - r_min_m)
                                     : std::numeric_limits<double>::infinity();
    r.shells[0] = ReachShellEval{m_lo, pts[i_near].off, radial_unit(pts[i_near].p, r_near)};
    // Outer shell (upper): farthest point must stay <= r_max. margin = r_max - r_far.
    const double m_hi = r_max_m - r_far;
    r.shells[1] = ReachShellEval{m_hi, pts[i_far].off, radial_unit(pts[i_far].p, r_far)};

    double min_margin = m_hi;
    int closest = 1;
    if (inner_active && m_lo < min_margin) { min_margin = m_lo; closest = 0; }
    r.checked = true;
    r.r_near_m = r_near;
    r.r_far_m = r_far;
    r.min_margin_m = min_margin;
    r.violated = min_margin < 0.0;
    r.closest_shell = reachShellName(closest);
    r.outside_depth_m = std::max(0.0, -m_hi) + (inner_active ? std::max(0.0, -m_lo) : 0.0);
    if (out) *out = r;
    return true;
}

// Decide what to do with one arm's candidate joint target against the shell.
// Mirrors decideRoiAction/decideFloorAction: monitor_only / disabled always Allow;
// FK failure fails closed per policy; inside the shell -> Allow; outside -> Allow
// only when not getting deeper outside vs the previous sent pose (escape). The
// servo loop enforces the shell with the shared velocity-damper projection; this
// pure decision is the policy reference and is unit-tested directly.
inline FloorAction decideReachAction(
    const ReachArmEvaluation& candidate,
    const ReachArmEvaluation& previous_sent,
    const ReachConstraintConfig& config
) {
    if (!config.enable) return FloorAction::Allow;
    if (config.monitor_only) return FloorAction::Allow;
    if (!candidate.checked || !std::isfinite(candidate.min_margin_m)) {
        return floorActionForPolicy(config.fail_policy);
    }
    if (candidate.min_margin_m >= 0.0) return FloorAction::Allow;  // inside the shell
    if (previous_sent.checked && std::isfinite(previous_sent.outside_depth_m) &&
        candidate.outside_depth_m < previous_sent.outside_depth_m + kReachEscapeEpsilonM) {
        return FloorAction::Allow;  // escape: slide back in / tangential while outside
    }
    return floorActionForPolicy(config.fail_policy);
}

}  // namespace rb_servo
