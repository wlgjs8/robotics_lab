#pragma once

// Dual-arm self-collision clearance: pairwise capsule distance between the two
// arms' link chains. Pure geometry (Eigen only); the servo loop supplies the
// per-arm kinematic chain points (in the stand frame) and the configured radii.

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "rb_servo/config/config.hpp"
#include "rb_servo/math/capsule_distance.hpp"

namespace rb_servo {

struct SelfCollisionResult {
    bool checked = false;   // link geometry was available and evaluated
    bool violated = false;  // min surface clearance < margin
    double min_clearance_m = std::numeric_limits<double>::infinity();
    int left_bone = -1;
    int right_bone = -1;   // arm-arm: right arm bone; arm-stand: stand capsule index
    // Which checked pair produced min_clearance_m:
    // "left_right" | "left_stand" | "right_stand" | "" (not evaluated).
    std::string pair;
    std::string stand_capsule;  // nearest stand capsule name (arm-stand pairs only)
    // Closest bone-AXIS points of the min-clearance pair (stand frame): the
    // closest points on each member's capsule core segment, so they lie on the
    // physical link/stand member itself (not on the inflated capsule surface).
    // a = the pair's first member (left arm, or the arm for arm-stand pairs);
    // b = the second member (right arm or the stand capsule).
    // |b - a| == min_clearance_m + the two capsule radii.
    bool has_closest_points = false;
    std::array<double, 3> closest_point_a_m{};
    std::array<double, 3> closest_point_b_m{};
};

// Keep the result whose minimum clearance is smaller; an unchecked result never
// wins over a checked one (fail-closed combination happens at the caller).
inline SelfCollisionResult minSelfCollisionResult(
    const SelfCollisionResult& a,
    const SelfCollisionResult& b
) {
    if (!a.checked) return b;
    if (!b.checked) return a;
    SelfCollisionResult out = b.min_clearance_m < a.min_clearance_m ? b : a;
    out.violated = a.violated || b.violated;
    return out;
}

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
            math::Vector3 p_a;
            math::Vector3 p_b;
            const double clearance =
                math::capsuleCapsuleDistanceWithPoints(a0, a1, ra, b0, b1, rb, &p_a, &p_b);
            if (clearance < result.min_clearance_m) {
                result.min_clearance_m = clearance;
                result.left_bone = static_cast<int>(li);
                result.right_bone = static_cast<int>(rj);
                result.has_closest_points = true;
                result.closest_point_a_m = {p_a.x(), p_a.y(), p_a.z()};
                result.closest_point_b_m = {p_b.x(), p_b.y(), p_b.z()};
            }
        }
    }
    result.violated = result.min_clearance_m < margin_m;
    result.pair = "left_right";
    return result;
}

// Minimum capsule-surface clearance between one arm's link chain and the static
// stand capsules (all in the stand frame). Bones listed in ignore_bones are
// skipped (bone 0 sits on the stand mount plate by construction). Returns
// checked=false if the chain is too short or the stand list is empty — the
// caller decides the fail-closed behavior. The arm bone index is reported in
// left_bone and the stand capsule index in right_bone regardless of arm side.
inline SelfCollisionResult armStandCollisionClearance(
    const std::vector<std::array<double, 3>>& arm_points,
    const std::array<double, 7>& bone_radius_m,
    const std::vector<StandCapsuleConfig>& stand_capsules,
    double margin_m,
    const std::vector<int>& ignore_bones
) {
    SelfCollisionResult result;
    if (arm_points.size() < 2 || stand_capsules.empty()) {
        return result;
    }
    result.checked = true;

    const auto as_vec = [](const std::array<double, 3>& p) {
        return math::Vector3(p[0], p[1], p[2]);
    };
    const auto radius_for = [&](std::size_t bone) {
        return bone_radius_m[std::min(bone, bone_radius_m.size() - 1)];
    };
    const auto ignored = [&](std::size_t bone) {
        return std::find(ignore_bones.begin(), ignore_bones.end(), static_cast<int>(bone)) !=
            ignore_bones.end();
    };

    for (std::size_t bi = 0; bi + 1 < arm_points.size(); ++bi) {
        if (ignored(bi)) continue;
        const math::Vector3 a0 = as_vec(arm_points[bi]);
        const math::Vector3 a1 = as_vec(arm_points[bi + 1]);
        const double ra = radius_for(bi);
        for (std::size_t si = 0; si < stand_capsules.size(); ++si) {
            const StandCapsuleConfig& cap = stand_capsules[si];
            math::Vector3 p_a;
            math::Vector3 p_b;
            const double clearance = math::capsuleCapsuleDistanceWithPoints(
                a0, a1, ra,
                as_vec(cap.p0_m), as_vec(cap.p1_m), cap.radius_m,
                &p_a, &p_b
            );
            if (clearance < result.min_clearance_m) {
                result.min_clearance_m = clearance;
                result.left_bone = static_cast<int>(bi);
                result.right_bone = static_cast<int>(si);
                result.stand_capsule = cap.name;
                result.has_closest_points = true;
                result.closest_point_a_m = {p_a.x(), p_a.y(), p_a.z()};
                result.closest_point_b_m = {p_b.x(), p_b.y(), p_b.z()};
            }
        }
    }
    result.violated = result.min_clearance_m < margin_m;
    return result;
}

}  // namespace rb_servo
