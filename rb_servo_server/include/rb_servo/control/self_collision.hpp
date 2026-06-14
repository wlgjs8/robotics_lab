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
    // Full per-arm collision capsules (stand frame) evaluated this tick — the
    // EXACT geometry checked, FK'd per arm from the arm_capsules template. Lets a
    // viewer draw the real checked capsules over the arm mesh.
    bool has_capsules = false;
    std::vector<ArmCapsule> left_capsules;
    std::vector<ArmCapsule> right_capsules;
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

// Minimum capsule-surface clearance between the left and right arm capsule sets
// (stand frame). Each arm is a list of FK'd per-link capsules. Returns
// checked=false if either set is empty (geometry unavailable) — the caller
// decides the fail-closed behavior. left_bone/right_bone report the capsule
// indices of the closest pair.
inline SelfCollisionResult dualArmSelfCollisionClearance(
    const std::vector<ArmCapsule>& left_capsules,
    const std::vector<ArmCapsule>& right_capsules,
    double margin_m
) {
    SelfCollisionResult result;
    if (left_capsules.empty() || right_capsules.empty()) {
        return result;
    }
    result.checked = true;

    const auto as_vec = [](const std::array<double, 3>& p) {
        return math::Vector3(p[0], p[1], p[2]);
    };

    for (std::size_t li = 0; li < left_capsules.size(); ++li) {
        const ArmCapsule& la = left_capsules[li];
        for (std::size_t rj = 0; rj < right_capsules.size(); ++rj) {
            const ArmCapsule& rb = right_capsules[rj];
            math::Vector3 p_a;
            math::Vector3 p_b;
            const double clearance = math::capsuleCapsuleDistanceWithPoints(
                as_vec(la.p0_m), as_vec(la.p1_m), la.radius_m,
                as_vec(rb.p0_m), as_vec(rb.p1_m), rb.radius_m, &p_a, &p_b);
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

// Minimum capsule-surface clearance between one arm's capsules and the static
// stand capsules (all in the stand frame). Arm capsule indices listed in
// ignore_indices are skipped (link0/link1 sit on/near the stand mount plate by
// construction). Returns checked=false if either list is empty — the caller
// decides the fail-closed behavior. The arm capsule index is reported in
// left_bone and the stand capsule index in right_bone regardless of arm side.
inline SelfCollisionResult armStandCollisionClearance(
    const std::vector<ArmCapsule>& arm_capsules,
    const std::vector<StandCapsuleConfig>& stand_capsules,
    double margin_m,
    const std::vector<int>& ignore_indices
) {
    SelfCollisionResult result;
    if (arm_capsules.empty() || stand_capsules.empty()) {
        return result;
    }
    result.checked = true;

    const auto as_vec = [](const std::array<double, 3>& p) {
        return math::Vector3(p[0], p[1], p[2]);
    };
    const auto ignored = [&](std::size_t idx) {
        return std::find(ignore_indices.begin(), ignore_indices.end(), static_cast<int>(idx)) !=
            ignore_indices.end();
    };

    for (std::size_t bi = 0; bi < arm_capsules.size(); ++bi) {
        if (ignored(bi)) continue;
        const ArmCapsule& a = arm_capsules[bi];
        for (std::size_t si = 0; si < stand_capsules.size(); ++si) {
            const StandCapsuleConfig& cap = stand_capsules[si];
            math::Vector3 p_a;
            math::Vector3 p_b;
            const double clearance = math::capsuleCapsuleDistanceWithPoints(
                as_vec(a.p0_m), as_vec(a.p1_m), a.radius_m,
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
