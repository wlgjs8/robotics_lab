#pragma once

// Stand-frame axis-aligned ROI box (workspace limit) decision logic. Pure
// functions: the servo loop supplies a candidate TCP pose (from FK) and the
// config; this module computes, for each of the 6 box faces, the margin of the
// most-exposed checked point (TCP + configured TCP-frame offset points) and a
// per-arm Allow / Hold / Latch decision. The box is the 3D generalization of
// safety.floor_constraint (a single z-lower face): every face is enforced the
// same way (velocity-damper projection in the servo loop). Keeping the decision
// pure makes the policy unit-testable without a backend.

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/floor_constraint.hpp"  // FloorAction, floorActionForPolicy, FloorCheckPointConfig
#include "rb_servo/core/types.hpp"
#include "rb_servo/math/se3.hpp"

namespace rb_servo {

// Stand-frame face index: axis in {0=x, 1=y, 2=z}, side in {0=min (lower),
// 1=max (upper)}. Names match the published telemetry / GUI labels.
inline const char* roiFaceName(int axis, int side) {
    static const char* kNames[3][2] = {
        {"x_min", "x_max"}, {"y_min", "y_max"}, {"z_min", "z_max"}};
    if (axis < 0 || axis > 2 || side < 0 || side > 1) return "?";
    return kNames[axis][side];
}

// Per-face evaluation: signed margin (>= 0 inside the box, < 0 outside) of the
// most-exposed checked point against that face, plus that point's TCP-frame
// offset (used to build its stand-axis velocity Jacobian for the projection).
struct RoiFaceEval {
    double margin_m = std::numeric_limits<double>::quiet_NaN();
    math::Vector3 offset_tcp = math::Vector3::Zero();  // {0,0,0} = the TCP point
};

// FK evaluation of one arm against the ROI box. checked=false means the TCP
// pose could not be evaluated (non-finite) — the caller fails closed when the
// constraint is enabled.
struct RoiArmEvaluation {
    bool checked = false;
    bool violated = false;  // any face margin < 0 (some checked point is outside)
    double min_margin_m = std::numeric_limits<double>::quiet_NaN();  // closest face
    std::string closest_face = "x_min";
    RoiFaceEval faces[3][2];          // [axis][side]
    // Sum over faces of max(0, -margin): total "how far outside" the box. Used as
    // the escape metric (a candidate outside the box is allowed when it is not
    // getting deeper outside vs the previous sent pose).
    double outside_depth_m = 0.0;
};

// Escape epsilon (mirror of kFloorEscapeEpsilonM): a candidate outside the box
// is allowed when its outside-depth does not increase vs the previous sent pose,
// so an arm already outside can slide back in / move tangentially.
inline constexpr double kRoiEscapeEpsilonM = 1e-4;

// Evaluate the TCP (+ each TCP-frame offset point) against the stand-frame box
// [min_m, max_m]. Fills *out and returns true (checked). Returns false and
// leaves out->checked=false when any point coordinate is non-finite.
inline bool roiEvaluateBox(
    const Pose6D& tcp_stand,
    const std::vector<FloorCheckPointConfig>& tcp_offset_points,
    const std::array<double, 3>& min_m,
    const std::array<double, 3>& max_m,
    RoiArmEvaluation* out
) {
    RoiArmEvaluation r;
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
    double min_margin = std::numeric_limits<double>::infinity();
    int closest_axis = 0;
    int closest_side = 0;
    double outside = 0.0;
    for (int k = 0; k < 3; ++k) {
        // Lower face binds on the point with the smallest p_k; upper on the largest.
        std::size_t i_min = 0;
        std::size_t i_max = 0;
        for (std::size_t i = 1; i < pts.size(); ++i) {
            if (pts[i].p[k] < pts[i_min].p[k]) i_min = i;
            if (pts[i].p[k] > pts[i_max].p[k]) i_max = i;
        }
        const double m_lo = pts[i_min].p[k] - min_m[k];
        const double m_hi = max_m[k] - pts[i_max].p[k];
        r.faces[k][0] = RoiFaceEval{m_lo, pts[i_min].off};
        r.faces[k][1] = RoiFaceEval{m_hi, pts[i_max].off};
        if (m_lo < min_margin) { min_margin = m_lo; closest_axis = k; closest_side = 0; }
        if (m_hi < min_margin) { min_margin = m_hi; closest_axis = k; closest_side = 1; }
        outside += std::max(0.0, -m_lo) + std::max(0.0, -m_hi);
    }
    r.checked = true;
    r.min_margin_m = min_margin;
    r.violated = min_margin < 0.0;
    r.closest_face = roiFaceName(closest_axis, closest_side);
    r.outside_depth_m = outside;
    if (out) *out = r;
    return true;
}

// Decide what to do with one arm's candidate joint target against the box.
// Mirrors decideFloorAction: monitor_only / disabled always Allow; FK failure
// fails closed per policy; inside the box -> Allow; outside -> Allow only when
// not getting deeper outside vs the previous sent pose (escape). The servo loop
// enforces the box with the shared velocity-damper projection; this pure
// decision is the policy reference and is unit-tested directly.
inline FloorAction decideRoiAction(
    const RoiArmEvaluation& candidate,
    const RoiArmEvaluation& previous_sent,
    const RoiBoxConfig& config
) {
    if (!config.enable) return FloorAction::Allow;
    if (config.monitor_only) return FloorAction::Allow;
    if (!candidate.checked || !std::isfinite(candidate.min_margin_m)) {
        return floorActionForPolicy(config.fail_policy);
    }
    if (candidate.min_margin_m >= 0.0) return FloorAction::Allow;  // inside the box
    if (previous_sent.checked && std::isfinite(previous_sent.outside_depth_m) &&
        candidate.outside_depth_m < previous_sent.outside_depth_m + kRoiEscapeEpsilonM) {
        return FloorAction::Allow;  // escape: slide back in / tangential while outside
    }
    return floorActionForPolicy(config.fail_policy);
}

// Validate a runtime SetSafetyRoiBounds request against the configured envelope.
// Returns std::nullopt on success, otherwise a machine-readable reject reason.
inline std::optional<std::string> validateRoiBoundsRequest(
    const std::array<double, 3>& requested_min_m,
    const std::array<double, 3>& requested_max_m,
    const RoiBoxConfig& config
) {
    if (!config.enable) return std::string("roi_box_disabled");
    for (int k = 0; k < 3; ++k) {
        if (!std::isfinite(requested_min_m[k]) || !std::isfinite(requested_max_m[k])) {
            return std::string("roi_bounds_not_finite");
        }
        if (requested_min_m[k] > requested_max_m[k]) {
            return std::string("roi_min_above_max");
        }
        if (requested_min_m[k] < config.runtime_min_m[k]) {
            return std::string("roi_below_runtime_min");
        }
        if (requested_max_m[k] > config.runtime_max_m[k]) {
            return std::string("roi_above_runtime_max");
        }
    }
    return std::nullopt;
}

}  // namespace rb_servo
