#pragma once

// Stand-frame floor plane constraint decision logic. Pure functions: the servo
// loop supplies per-arm candidate/previous-sent TCP z evaluations (from FK) and
// the config; this module decides Allow / Hold / Latch per arm. Keeping the
// decision pure makes the policy unit-testable without a backend.

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"
#include "rb_servo/math/se3.hpp"

namespace rb_servo {

// FK evaluation of one arm's TCP against the floor plane. checked=false means
// the TCP z could not be computed (kinematics missing, non-finite joints, FK
// throw) — the caller treats that as fail-closed when the constraint is enabled.
// With safety.floor_constraint.tcp_offset_points configured, tcp_z_m is the
// LOWEST checked point's z (TCP + each TCP-frame offset point, e.g. gripper
// fingertips) and lowest_point names which one it was.
struct FloorArmEvaluation {
    bool checked = false;
    double tcp_z_m = std::numeric_limits<double>::quiet_NaN();
    bool violated = false;
    std::string lowest_point = "tcp";
};

// Lowest stand-frame z over the TCP and the configured TCP-frame offset
// points (p = tcp_position + R_tcp * offset). Returns NaN when the TCP pose
// is non-finite. lowest_name (optional) reports which point won.
inline double floorLowestZWithOffsets(
    const Pose6D& tcp_stand,
    const std::vector<FloorCheckPointConfig>& tcp_offset_points,
    std::string* lowest_name
) {
    if (lowest_name) *lowest_name = "tcp";
    if (!std::isfinite(tcp_stand.z)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double lowest = tcp_stand.z;
    if (!tcp_offset_points.empty()) {
        const math::Matrix3 rotation = math::rotationFromPose(tcp_stand);
        for (const FloorCheckPointConfig& point : tcp_offset_points) {
            const math::Vector3 offset(point.offset_m[0], point.offset_m[1], point.offset_m[2]);
            const double z = tcp_stand.z + (rotation * offset).z();
            if (!std::isfinite(z)) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            if (z < lowest) {
                lowest = z;
                if (lowest_name) *lowest_name = point.name;
            }
        }
    }
    return lowest;
}

enum class FloorAction {
    Allow,
    Hold,   // revert this arm to its previous sent target (non-latching)
    Latch,  // latch a fault (FloorConstraintFailPolicy::FaultLatch)
};

// Minimum upward progress (meters) for the escape exception: a candidate below
// the plane is allowed only if it strictly raises the TCP vs the previously
// sent configuration, so an arm that starts below the plane can still be jogged
// up and out without a fault reset.
inline constexpr double kFloorEscapeEpsilonM = 1e-4;

inline FloorAction floorActionForPolicy(FloorConstraintFailPolicy policy) {
    return policy == FloorConstraintFailPolicy::FaultLatch ? FloorAction::Latch
                                                           : FloorAction::Hold;
}

// Decide what to do with one arm's candidate joint target.
// - monitor_only: always Allow (telemetry still published by the caller).
// - candidate not checked (FK failure): fail closed per policy.
// - candidate above the plane: Allow.
// - candidate below the plane: Allow only when it strictly raises the TCP vs the
//   previous sent evaluation (escape); previous unchecked => fail closed.
inline FloorAction decideFloorAction(
    const FloorArmEvaluation& candidate,
    const FloorArmEvaluation& previous_sent,
    const FloorConstraintConfig& config,
    double effective_z_min_m
) {
    if (!config.enable) return FloorAction::Allow;
    if (config.monitor_only) return FloorAction::Allow;
    if (!candidate.checked || !std::isfinite(candidate.tcp_z_m)) {
        return floorActionForPolicy(config.fail_policy);
    }
    if (candidate.tcp_z_m >= effective_z_min_m) return FloorAction::Allow;
    if (previous_sent.checked && std::isfinite(previous_sent.tcp_z_m) &&
        candidate.tcp_z_m > previous_sent.tcp_z_m + kFloorEscapeEpsilonM) {
        return FloorAction::Allow;  // escape: strictly upward while below the plane
    }
    return floorActionForPolicy(config.fail_policy);
}

// Validate a runtime SetSafetyFloorZ request against the configured bounds.
// Returns std::nullopt on success, otherwise a machine-readable reject reason.
inline std::optional<std::string> validateFloorZRequest(
    double requested_z_m,
    const FloorConstraintConfig& config
) {
    if (!config.enable) return std::string("floor_constraint_disabled");
    if (!std::isfinite(requested_z_m)) return std::string("floor_z_not_finite");
    if (requested_z_m < config.runtime_min_z_m) return std::string("floor_z_below_runtime_min");
    if (requested_z_m > config.runtime_max_z_m) return std::string("floor_z_above_runtime_max");
    return std::nullopt;
}

}  // namespace rb_servo
