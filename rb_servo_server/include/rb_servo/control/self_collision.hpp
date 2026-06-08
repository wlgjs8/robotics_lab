#pragma once

// Dual-arm self-collision clearance: pairwise capsule distance between the two
// arms' link chains. Pure geometry (Eigen only); the servo loop supplies the
// per-arm kinematic chain points (in the stand frame) and the configured radii.

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <vector>

#include "rb_servo/math/capsule_distance.hpp"

namespace rb_servo {

struct SelfCollisionResult {
    bool checked = false;   // link geometry was available and evaluated
    bool violated = false;  // min surface clearance < margin
    double min_clearance_m = std::numeric_limits<double>::infinity();
    int left_bone = -1;
    int right_bone = -1;
};

// Minimum capsule-surface clearance between left and right arm link chains.
// *_points are ordered chain points [base, j1..j6, tcp]; consecutive points are
// capsule bone endpoints. bone_radius_m[i] is bone i's radius (the last radius is
// reused for any extra bones). Returns checked=false if either chain is too short
// (geometry unavailable) — the caller decides the fail-closed behavior.
inline SelfCollisionResult dualArmSelfCollisionClearance(
    const std::vector<std::array<double, 3>>& left_points,
    const std::vector<std::array<double, 3>>& right_points,
    const std::array<double, 7>& bone_radius_m,
    double margin_m
) {
    SelfCollisionResult result;
    if (left_points.size() < 2 || right_points.size() < 2) {
        return result;
    }
    result.checked = true;

    const auto as_vec = [](const std::array<double, 3>& p) {
        return math::Vector3(p[0], p[1], p[2]);
    };
    const auto radius_for = [&](std::size_t bone) {
        return bone_radius_m[std::min(bone, bone_radius_m.size() - 1)];
    };

    for (std::size_t li = 0; li + 1 < left_points.size(); ++li) {
        const math::Vector3 a0 = as_vec(left_points[li]);
        const math::Vector3 a1 = as_vec(left_points[li + 1]);
        const double ra = radius_for(li);
        for (std::size_t rj = 0; rj + 1 < right_points.size(); ++rj) {
            const math::Vector3 b0 = as_vec(right_points[rj]);
            const math::Vector3 b1 = as_vec(right_points[rj + 1]);
            const double rb = radius_for(rj);
            const double clearance = math::capsuleCapsuleDistance(a0, a1, ra, b0, b1, rb);
            if (clearance < result.min_clearance_m) {
                result.min_clearance_m = clearance;
                result.left_bone = static_cast<int>(li);
                result.right_bone = static_cast<int>(rj);
            }
        }
    }
    result.violated = result.min_clearance_m < margin_m;
    return result;
}

}  // namespace rb_servo
