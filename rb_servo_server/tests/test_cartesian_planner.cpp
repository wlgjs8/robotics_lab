#include <cmath>
#include <iostream>

#include "rb_servo/control/cartesian_trajectory_planner.hpp"

namespace {

constexpr double kEpsilon = 1e-9;

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

double quaternionAbsDot(
    const std::array<double, 4>& a,
    const std::array<double, 4>& b
) {
    return std::abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
}

rb_servo::Pose6D pose(
    double x,
    double y,
    double z,
    const std::array<double, 4>& quaternion_xyzw
) {
    rb_servo::Pose6D out;
    out.x = x;
    out.y = y;
    out.z = z;
    out.quaternion_xyzw = quaternion_xyzw;
    return out;
}

bool testConstantOrientationKeepsQuaternionFixed() {
    const std::array<double, 4> start_q{0.0, 0.0, std::sin(0.25), std::cos(0.25)};
    const std::array<double, 4> target_q{0.0, std::sin(0.5), 0.0, std::cos(0.5)};
    rb_servo::LinearCartesianPlanner planner;
    const rb_servo::Pose6D sample = planner.sample(
        rb_servo::CartesianTrajectoryRequest{
            pose(0.1, -0.2, 0.3, start_q),
            pose(0.4, 0.2, 0.9, target_q),
            rb_servo::CartesianOrientationInterpolation::Constant,
        },
        0.5
    );

    RB_CHECK(std::abs(sample.x - 0.25) < kEpsilon);
    RB_CHECK(std::abs(sample.y - 0.0) < kEpsilon);
    RB_CHECK(std::abs(sample.z - 0.6) < kEpsilon);
    RB_CHECK(sample.quaternion_xyzw.has_value());
    RB_CHECK(quaternionAbsDot(*sample.quaternion_xyzw, start_q) > 1.0 - 1e-12);
    return true;
}

bool testSlerpReachesTargetOrientation() {
    const std::array<double, 4> start_q{0.0, 0.0, 0.0, 1.0};
    const std::array<double, 4> target_q{0.0, 0.0, std::sin(0.75), std::cos(0.75)};
    rb_servo::LinearCartesianPlanner planner;
    const rb_servo::Pose6D sample = planner.sample(
        rb_servo::CartesianTrajectoryRequest{
            pose(0.1, -0.2, 0.3, start_q),
            pose(0.4, 0.2, 0.9, target_q),
            rb_servo::CartesianOrientationInterpolation::Slerp,
        },
        1.0
    );

    RB_CHECK(std::abs(sample.x - 0.4) < kEpsilon);
    RB_CHECK(std::abs(sample.y - 0.2) < kEpsilon);
    RB_CHECK(std::abs(sample.z - 0.9) < kEpsilon);
    RB_CHECK(sample.quaternion_xyzw.has_value());
    RB_CHECK(quaternionAbsDot(*sample.quaternion_xyzw, target_q) > 1.0 - 1e-12);
    return true;
}

bool testSlerpMidpointIsNormalized() {
    const std::array<double, 4> start_q{0.0, 0.0, 0.0, 1.0};
    const std::array<double, 4> target_q{0.0, 0.0, 1.0, 0.0};
    rb_servo::LinearCartesianPlanner planner;
    const rb_servo::Pose6D sample = planner.sample(
        rb_servo::CartesianTrajectoryRequest{
            pose(0.0, 0.0, 0.0, start_q),
            pose(1.0, 2.0, 3.0, target_q),
            rb_servo::CartesianOrientationInterpolation::Slerp,
        },
        0.5
    );

    RB_CHECK(sample.quaternion_xyzw.has_value());
    const auto& q = *sample.quaternion_xyzw;
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    RB_CHECK(std::abs(norm - 1.0) < 1e-12);
    RB_CHECK(std::abs(sample.x - 0.5) < kEpsilon);
    RB_CHECK(std::abs(sample.y - 1.0) < kEpsilon);
    RB_CHECK(std::abs(sample.z - 1.5) < kEpsilon);
    return true;
}

}  // namespace

int main() {
    if (!testConstantOrientationKeepsQuaternionFixed()) return 1;
    if (!testSlerpReachesTargetOrientation()) return 1;
    if (!testSlerpMidpointIsNormalized()) return 1;
    return 0;
}
