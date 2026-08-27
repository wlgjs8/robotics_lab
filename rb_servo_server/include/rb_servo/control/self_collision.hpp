#pragma once

// Self-collision telemetry carrier: the result type the servo loop fills from the
// async URDF-mesh CollisionMonitor verdict and publishes for the GUI overlay
// (clearance, violation, witness points, nearest pair). The geometric check itself
// lives in the mesh CollisionMonitor (pinocchio + coal), not here.

#include <array>
#include <limits>
#include <string>

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
    // The two GEOM names behind min_clearance_m. `pair` above is only a side
    // category and collapses every same-side pair to "all" -- measured 2026-08-28
    // it read "all" on 115118 of 115439 rows, so it named nothing at all for the
    // case that matters (an arm folding onto itself). These carry the actual mesh
    // names, which is what a curation/tuning decision needs.
    std::string closest_geom_a;
    std::string closest_geom_b;
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

}  // namespace rb_servo
