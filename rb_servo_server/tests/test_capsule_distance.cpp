#include <cmath>
#include <iostream>

#include <Eigen/Core>

#include "rb_servo/math/capsule_distance.hpp"

namespace {

using rb_servo::math::Vector3;
using rb_servo::math::capsuleCapsuleDistance;
using rb_servo::math::closestPointSegmentSegmentSquared;
using rb_servo::math::segmentSegmentDistance;

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

bool approx(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) <= tol;
}

bool testParallelSegments() {
    // Two unit segments along x, separated by 2.0 along z.
    const Vector3 a0(-1, 0, 0), a1(1, 0, 0);
    const Vector3 b0(-1, 0, 2), b1(1, 0, 2);
    RB_CHECK(approx(segmentSegmentDistance(a0, a1, b0, b1), 2.0));
    return true;
}

bool testCrossingSegmentsTouch() {
    // A along x through origin, B along y through origin: they intersect.
    const Vector3 a0(-1, 0, 0), a1(1, 0, 0);
    const Vector3 b0(0, -1, 0), b1(0, 1, 0);
    RB_CHECK(approx(segmentSegmentDistance(a0, a1, b0, b1), 0.0));
    return true;
}

bool testSkewSegments() {
    // A along x at z=0 (through origin); B along y at z=3 (through (0,0,3)).
    // Closest points are origin and (0,0,3) -> distance 3.
    const Vector3 a0(-2, 0, 0), a1(2, 0, 0);
    const Vector3 b0(0, -2, 3), b1(0, 2, 3);
    Vector3 ca, cb;
    const double d2 = closestPointSegmentSegmentSquared(a0, a1, b0, b1, &ca, &cb);
    RB_CHECK(approx(std::sqrt(d2), 3.0));
    RB_CHECK(approx((ca - Vector3(0, 0, 0)).norm(), 0.0, 1e-9));
    RB_CHECK(approx((cb - Vector3(0, 0, 3)).norm(), 0.0, 1e-9));
    return true;
}

bool testEndpointClamping() {
    // Collinear segments along x that do not overlap: [0,1] and [3,4].
    // Closest distance is between the near endpoints (1) and (3) -> 2.
    const Vector3 a0(0, 0, 0), a1(1, 0, 0);
    const Vector3 b0(3, 0, 0), b1(4, 0, 0);
    RB_CHECK(approx(segmentSegmentDistance(a0, a1, b0, b1), 2.0));
    return true;
}

bool testDegeneratePoints() {
    // Both segments are points.
    const Vector3 p(1, 2, 3), q(1, 2, 6);
    RB_CHECK(approx(segmentSegmentDistance(p, p, q, q), 3.0));
    // Point vs segment: point at (0,0,5) vs x-axis segment [-1,1] -> 5.
    const Vector3 pt(0, 0, 5), s0(-1, 0, 0), s1(1, 0, 0);
    RB_CHECK(approx(segmentSegmentDistance(pt, pt, s0, s1), 5.0));
    return true;
}

bool testCapsuleClearanceAndPenetration() {
    const Vector3 a0(-1, 0, 0), a1(1, 0, 0);
    const Vector3 b0(-1, 0, 2), b1(1, 0, 2);  // parallel, 2.0 apart
    // radii 0.3 + 0.5 -> clearance 2.0 - 0.8 = 1.2
    RB_CHECK(approx(capsuleCapsuleDistance(a0, a1, 0.3, b0, b1, 0.5), 1.2));
    // Overlapping radii (1.2 + 1.0 > 2.0) -> negative clearance (penetration).
    const double pen = capsuleCapsuleDistance(a0, a1, 1.2, b0, b1, 1.0);
    RB_CHECK(approx(pen, 2.0 - 2.2));
    RB_CHECK(pen < 0.0);
    return true;
}

}  // namespace

int main() {
    if (!testParallelSegments()) return 1;
    if (!testCrossingSegmentsTouch()) return 1;
    if (!testSkewSegments()) return 1;
    if (!testEndpointClamping()) return 1;
    if (!testDegeneratePoints()) return 1;
    if (!testCapsuleClearanceAndPenetration()) return 1;
    std::cout << "capsule_distance tests passed\n";
    return 0;
}
