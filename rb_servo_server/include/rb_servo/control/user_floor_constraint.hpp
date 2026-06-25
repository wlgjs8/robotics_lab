#pragma once

// Stand-frame USER-defined tilted floor plane constraint decision logic. Unlike
// safety.floor_constraint (a HORIZONTAL plane z >= z_min_m), this is an ARBITRARY
// plane defined by a point p0 and unit normal n (pointing into the allowed/upper
// half-space): the TCP of either arm — and each configured TCP-frame offset point —
// must satisfy n . (p - p0) >= margin_m. It is fit in the GUI from >= 3 captured
// floor-contact points (both arms) and pushed at runtime via the leaseless
// SetUserSafetyFloorPlane command. Independent of and ADDITIVE to floor_constraint
// (both apply when enabled; the stricter wins). Pure functions: the servo loop
// supplies a candidate TCP pose (from FK) + config; this module computes the signed
// distance of the most-exposed point and an Allow/Hold/Latch decision. Enforced
// with the shared velocity-damper projection (mirror of decideFloorAction), using
// the constant plane normal n as the stand-frame direction for every point. Keeping
// the decision pure makes the policy unit-testable without a backend.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/floor_constraint.hpp"  // FloorAction, floorActionForPolicy, FloorCheckPointConfig, kFloorEscapeEpsilonM
#include "rb_servo/core/types.hpp"
#include "rb_servo/math/se3.hpp"

namespace rb_servo {

// FK evaluation of one arm against the user floor plane. checked=false means the
// TCP pose could not be evaluated (non-finite) — the caller fails closed when the
// constraint is enabled. signed_dist_m is the MINIMUM over {TCP, each offset point}
// of n . (p - p0) - margin_m (>= 0 on/above the plane, < 0 below); lowest_* describe
// the most-exposed point, used to build its normal-velocity Jacobian and for telemetry.
struct UserFloorArmEvaluation {
    bool checked = false;
    bool violated = false;
    double signed_dist_m = std::numeric_limits<double>::quiet_NaN();
    std::string lowest_point = "tcp";
    math::Vector3 lowest_offset_tcp = math::Vector3::Zero();   // {0,0,0} = the TCP point
    math::Vector3 lowest_point_stand = math::Vector3::Zero();  // stand-frame xyz of the lowest point
};

// Evaluate the TCP (+ each TCP-frame offset point p = tcp_position + R_tcp * offset)
// against the plane {p0, n} with the given margin. n is assumed already unit
// (validated upstream by validateUserFloorPlaneRequest). Fills *out and returns true
// (checked); returns false with out->checked=false on any non-finite coordinate.
inline bool userFloorEvaluatePlane(
    const Pose6D& tcp_stand,
    const std::vector<FloorCheckPointConfig>& tcp_offset_points,
    const math::Vector3& p0,
    const math::Vector3& n,
    double margin_m,
    UserFloorArmEvaluation* out
) {
    UserFloorArmEvaluation r;
    if (!std::isfinite(tcp_stand.x) || !std::isfinite(tcp_stand.y) ||
        !std::isfinite(tcp_stand.z)) {
        if (out) *out = r;  // checked=false
        return false;
    }
    const auto signed_dist = [&](const math::Vector3& p) { return n.dot(p - p0) - margin_m; };
    // The TCP point is always the first checked point.
    const math::Vector3 tcp_p(tcp_stand.x, tcp_stand.y, tcp_stand.z);
    double lowest = signed_dist(tcp_p);
    r.lowest_point = "tcp";
    r.lowest_offset_tcp = math::Vector3::Zero();
    r.lowest_point_stand = tcp_p;
    if (!tcp_offset_points.empty()) {
        const math::Matrix3 rotation = math::rotationFromPose(tcp_stand);
        for (const FloorCheckPointConfig& point : tcp_offset_points) {
            const math::Vector3 offset(point.offset_m[0], point.offset_m[1], point.offset_m[2]);
            const math::Vector3 p = tcp_p + rotation * offset;
            if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
                if (out) *out = UserFloorArmEvaluation{};  // checked=false
                return false;
            }
            const double s = signed_dist(p);
            if (s < lowest) {
                lowest = s;
                r.lowest_point = point.name;
                r.lowest_offset_tcp = offset;
                r.lowest_point_stand = p;
            }
        }
    }
    r.checked = true;
    r.signed_dist_m = lowest;
    r.violated = lowest < 0.0;
    if (out) *out = r;
    return true;
}

// Decide what to do with one arm's candidate joint target against the plane.
// Mirrors decideFloorAction: monitor_only / disabled always Allow; FK failure fails
// closed per policy; on/above the plane (signed_dist >= 0) -> Allow; below -> Allow
// only when NOT descending (signed_dist not decreasing) vs the previous sent pose
// (escape: slide/lift while below). The servo loop enforces it with the shared
// velocity-damper projection; this pure decision is the policy reference.
inline FloorAction decideUserFloorAction(
    const UserFloorArmEvaluation& candidate,
    const UserFloorArmEvaluation& previous_sent,
    const UserFloorConstraintConfig& config
) {
    if (!config.enable) return FloorAction::Allow;
    if (config.monitor_only) return FloorAction::Allow;
    if (!candidate.checked || !std::isfinite(candidate.signed_dist_m)) {
        return floorActionForPolicy(config.fail_policy);
    }
    if (candidate.signed_dist_m >= 0.0) return FloorAction::Allow;
    if (previous_sent.checked && std::isfinite(previous_sent.signed_dist_m) &&
        candidate.signed_dist_m > previous_sent.signed_dist_m - kFloorEscapeEpsilonM) {
        return FloorAction::Allow;  // escape: lateral/upward while below the plane
    }
    return floorActionForPolicy(config.fail_policy);
}

// Validate a runtime SetUserSafetyFloorPlane request against the configured envelope.
// Returns std::nullopt on success, otherwise a machine-readable reject reason. n_raw
// must be (near) unit; n.z must be > 0 and the tilt from vertical bounded by
// max_tilt_deg so the allowed half-space always opens upward (a tilted plane can
// never invert into "stay BELOW a near-horizontal plane"). This is the tilted-plane
// analog of the floor's [runtime_min_z_m, runtime_max_z_m].
inline std::optional<std::string> validateUserFloorPlaneRequest(
    const std::array<double, 3>& p0,
    const std::array<double, 3>& n_raw,
    double margin_m,
    const UserFloorConstraintConfig& config
) {
    if (!config.enable) return std::string("user_floor_disabled");
    for (double v : p0) if (!std::isfinite(v)) return std::string("user_floor_point_not_finite");
    for (double v : n_raw) if (!std::isfinite(v)) return std::string("user_floor_normal_not_finite");
    const math::Vector3 n(n_raw[0], n_raw[1], n_raw[2]);
    const double len = n.norm();
    if (len < 1e-6) return std::string("user_floor_normal_degenerate");
    if (std::abs(len - 1.0) > 1e-3) return std::string("user_floor_normal_not_unit");
    if (!std::isfinite(margin_m) || margin_m < 0.0 || margin_m > config.max_margin_m) {
        return std::string("user_floor_margin_out_of_range");
    }
    if (n.z() <= 0.0) return std::string("user_floor_normal_not_upward");
    const double tilt_deg = std::acos(std::clamp(n.z() / len, -1.0, 1.0)) * 180.0 / M_PI;
    if (tilt_deg > config.max_tilt_deg) return std::string("user_floor_tilt_excessive");
    if (p0[2] < config.runtime_min_point_z_m) return std::string("user_floor_point_below_runtime_min");
    if (p0[2] > config.runtime_max_point_z_m) return std::string("user_floor_point_above_runtime_max");
    return std::nullopt;
}

}  // namespace rb_servo
