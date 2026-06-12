#pragma once

// Analytic capsule/segment distance helpers (Eigen only, no FCL).
// Used by the server-side dual-arm self-collision guard: each robot link is
// approximated as a capsule (line segment + radius); the minimum distance
// between two capsules is the segment-segment distance minus the two radii
// (negative means interpenetration).

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace rb_servo::math {

using Vector3 = Eigen::Vector3d;

// Squared closest distance between segment [a0,a1] and segment [b0,b1].
// Closest points are returned in c_a / c_b. Robust to degenerate (point)
// segments. Ericson, Real-Time Collision Detection, ClosestPtSegmentSegment.
inline double closestPointSegmentSegmentSquared(
    const Vector3& a0,
    const Vector3& a1,
    const Vector3& b0,
    const Vector3& b1,
    Vector3* c_a,
    Vector3* c_b
) {
    constexpr double kEps = 1e-12;
    const Vector3 d1 = a1 - a0;  // direction of segment A
    const Vector3 d2 = b1 - b0;  // direction of segment B
    const Vector3 r = a0 - b0;
    const double a = d1.squaredNorm();
    const double e = d2.squaredNorm();
    const double f = d2.dot(r);

    double s = 0.0;
    double t = 0.0;
    if (a <= kEps && e <= kEps) {
        // Both segments are points.
        s = 0.0;
        t = 0.0;
    } else if (a <= kEps) {
        // Segment A is a point.
        s = 0.0;
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = d1.dot(r);
        if (e <= kEps) {
            // Segment B is a point.
            t = 0.0;
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = d1.dot(d2);
            const double denom = a * e - b * b;  // always >= 0
            if (denom > kEps) {
                s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
            } else {
                s = 0.0;  // parallel: pick an arbitrary point on A
            }
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }

    const Vector3 closest_a = a0 + d1 * s;
    const Vector3 closest_b = b0 + d2 * t;
    if (c_a) {
        *c_a = closest_a;
    }
    if (c_b) {
        *c_b = closest_b;
    }
    return (closest_a - closest_b).squaredNorm();
}

// Closest distance between segment [a0,a1] and segment [b0,b1].
inline double segmentSegmentDistance(
    const Vector3& a0,
    const Vector3& a1,
    const Vector3& b0,
    const Vector3& b1
) {
    return std::sqrt(closestPointSegmentSegmentSquared(a0, a1, b0, b1, nullptr, nullptr));
}

// Signed clearance between two capsules: the gap between their surfaces.
// Positive = separated by this distance; <= 0 = touching/interpenetrating.
inline double capsuleCapsuleDistance(
    const Vector3& a0,
    const Vector3& a1,
    double radius_a,
    const Vector3& b0,
    const Vector3& b1,
    double radius_b
) {
    return segmentSegmentDistance(a0, a1, b0, b1) - radius_a - radius_b;
}

// capsuleCapsuleDistance plus the closest CORE-segment points (witness
// points): p_a/p_b are the closest points on each capsule's core segment
// (the bone axis), so they lie ON the physical link rather than on the
// inflated capsule surface. |p_b - p_a| == clearance + radius_a + radius_b.
inline double capsuleCapsuleDistanceWithPoints(
    const Vector3& a0,
    const Vector3& a1,
    double radius_a,
    const Vector3& b0,
    const Vector3& b1,
    double radius_b,
    Vector3* p_a,
    Vector3* p_b
) {
    Vector3 c_a;
    Vector3 c_b;
    const double segment_distance =
        std::sqrt(closestPointSegmentSegmentSquared(a0, a1, b0, b1, &c_a, &c_b));
    if (p_a) *p_a = c_a;
    if (p_b) *p_b = c_b;
    return segment_distance - radius_a - radius_b;
}

}  // namespace rb_servo::math
