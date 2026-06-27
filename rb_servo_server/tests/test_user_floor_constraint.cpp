#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "rb_servo/control/user_floor_constraint.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

rb_servo::UserFloorConstraintConfig enabledConfig() {
    rb_servo::UserFloorConstraintConfig config;
    config.enable = true;
    config.has_initial_plane = true;
    return config;
}

rb_servo::Pose6D poseAt(double x, double y, double z) {
    rb_servo::Pose6D p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

// 30deg-tilted plane through the origin: normal tilted about the x-axis so its
// projection onto +z is cos30 and onto +y is sin30 (n.z > 0, opens upward).
rb_servo::math::Vector3 tiltedNormal30() {
    const double a = 30.0 * M_PI / 180.0;
    return rb_servo::math::Vector3(0.0, std::sin(a), std::cos(a)).normalized();
}

bool testHorizontalPlaneSignedDistance() {
    // Plane z=0, normal +z. Point at z=0.1 => signed dist 0.1, not violated.
    rb_servo::UserFloorArmEvaluation e;
    const rb_servo::math::Vector3 p0(0, 0, 0), n(0, 0, 1);
    RB_CHECK(rb_servo::userFloorEvaluatePlane(poseAt(0.3, -0.2, 0.1), {}, p0, n, 0.0, &e));
    RB_CHECK(e.checked);
    RB_CHECK(!e.violated);
    RB_CHECK(std::abs(e.signed_dist_m - 0.1) < 1e-12);
    RB_CHECK(e.lowest_point == "tcp");
    RB_CHECK(std::abs(e.lowest_point_stand.z() - 0.1) < 1e-12);
    // On the plane => ~0. Below => violated.
    rb_servo::UserFloorArmEvaluation on, below;
    rb_servo::userFloorEvaluatePlane(poseAt(0.0, 0.0, 0.0), {}, p0, n, 0.0, &on);
    RB_CHECK(std::abs(on.signed_dist_m) < 1e-12);
    rb_servo::userFloorEvaluatePlane(poseAt(0.0, 0.0, -0.05), {}, p0, n, 0.0, &below);
    RB_CHECK(below.violated);
    RB_CHECK(std::abs(below.signed_dist_m + 0.05) < 1e-12);
    return true;
}

bool testTiltedPlaneSignedDistance() {
    // Tilted plane through origin. signed dist of p is n.(p) (margin 0).
    const rb_servo::math::Vector3 p0(0, 0, 0);
    const rb_servo::math::Vector3 n = tiltedNormal30();
    // A point straight up at (0,0,1): signed dist = n.z = cos30 ~ 0.8660.
    rb_servo::UserFloorArmEvaluation e;
    rb_servo::userFloorEvaluatePlane(poseAt(0.0, 0.0, 1.0), {}, p0, n, 0.0, &e);
    RB_CHECK(e.checked && !e.violated);
    RB_CHECK(std::abs(e.signed_dist_m - std::cos(30.0 * M_PI / 180.0)) < 1e-9);
    // A point on the +y side, below the tilted plane: (0, 1, -1).
    // signed dist = sin30*1 + cos30*(-1) = 0.5 - 0.8660 < 0 => violated.
    rb_servo::UserFloorArmEvaluation v;
    rb_servo::userFloorEvaluatePlane(poseAt(0.0, 1.0, -1.0), {}, p0, n, 0.0, &v);
    RB_CHECK(v.violated);
    const double expect = std::sin(30.0 * M_PI / 180.0) - std::cos(30.0 * M_PI / 180.0);
    RB_CHECK(std::abs(v.signed_dist_m - expect) < 1e-9);
    return true;
}

bool testOffsetPointBindsLowest() {
    // TCP above the plane, but a downward offset point dips below it. The lowest
    // signed distance (most exposed point) must win, and lowest_point_stand must
    // equal tcp + R*offset (identity rotation here).
    const rb_servo::math::Vector3 p0(0, 0, 0), n(0, 0, 1);
    std::vector<rb_servo::FloorCheckPointConfig> offsets = {
        {"tip", {0.0, 0.0, -0.06}}};
    rb_servo::UserFloorArmEvaluation e;
    rb_servo::userFloorEvaluatePlane(poseAt(0.1, 0.2, 0.04), offsets, p0, n, 0.0, &e);
    RB_CHECK(e.checked);
    RB_CHECK(e.lowest_point == "tip");
    RB_CHECK(std::abs(e.signed_dist_m + 0.02) < 1e-12);  // 0.04 - 0.06 = -0.02
    RB_CHECK(e.violated);
    RB_CHECK(std::abs(e.lowest_point_stand.x() - 0.1) < 1e-12);
    RB_CHECK(std::abs(e.lowest_point_stand.z() - (-0.02)) < 1e-12);
    return true;
}

bool testMarginFoldsIn() {
    // margin lifts the plane up: a point 0.03 above the geometric plane with a
    // 0.05 margin reports signed_dist = 0.03 - 0.05 = -0.02 (violated).
    const rb_servo::math::Vector3 p0(0, 0, 0), n(0, 0, 1);
    rb_servo::UserFloorArmEvaluation e;
    rb_servo::userFloorEvaluatePlane(poseAt(0.0, 0.0, 0.03), {}, p0, n, 0.05, &e);
    RB_CHECK(std::abs(e.signed_dist_m + 0.02) < 1e-12);
    RB_CHECK(e.violated);
    return true;
}

bool testEscapeAllowsAscentWhileBelow() {
    auto cfg = enabledConfig();
    cfg.fail_policy = rb_servo::FloorConstraintFailPolicy::FaultLatch;
    rb_servo::UserFloorArmEvaluation deep, shallower;
    deep.checked = true;
    deep.signed_dist_m = -0.05;  // previous sent: 5cm below
    shallower.checked = true;
    shallower.signed_dist_m = -0.02;  // candidate: rising toward the plane
    // Rising (not descending) while below => Allow (escape).
    RB_CHECK(rb_servo::decideUserFloorAction(shallower, deep, cfg) == rb_servo::FloorAction::Allow);
    // Descending further => Latch (FaultLatch policy).
    RB_CHECK(rb_servo::decideUserFloorAction(deep, shallower, cfg) == rb_servo::FloorAction::Latch);
    return true;
}

bool testMonitorOnlyAndDisabledAllow() {
    rb_servo::UserFloorArmEvaluation below;
    below.checked = true;
    below.signed_dist_m = -0.2;
    auto monitor = enabledConfig();
    monitor.monitor_only = true;
    RB_CHECK(rb_servo::decideUserFloorAction(below, below, monitor) == rb_servo::FloorAction::Allow);
    rb_servo::UserFloorConstraintConfig disabled;  // enable=false
    RB_CHECK(rb_servo::decideUserFloorAction(below, below, disabled) == rb_servo::FloorAction::Allow);
    return true;
}

bool testFkFailureFailsClosed() {
    auto cfg = enabledConfig();
    rb_servo::UserFloorArmEvaluation unchecked;  // checked=false
    rb_servo::UserFloorArmEvaluation above;
    above.checked = true;
    above.signed_dist_m = 0.1;
    RB_CHECK(rb_servo::decideUserFloorAction(unchecked, above, cfg) == rb_servo::FloorAction::Hold);
    cfg.fail_policy = rb_servo::FloorConstraintFailPolicy::FaultLatch;
    RB_CHECK(rb_servo::decideUserFloorAction(unchecked, above, cfg) == rb_servo::FloorAction::Latch);
    // Non-finite TCP => checked=false (fail closed).
    const rb_servo::math::Vector3 p0(0, 0, 0), n(0, 0, 1);
    rb_servo::UserFloorArmEvaluation e;
    RB_CHECK(!rb_servo::userFloorEvaluatePlane(
        poseAt(0.0, 0.0, std::numeric_limits<double>::quiet_NaN()), {}, p0, n, 0.0, &e));
    RB_CHECK(!e.checked);
    return true;
}

bool testValidateRequest() {
    auto cfg = enabledConfig();
    cfg.max_tilt_deg = 35.0;
    cfg.max_margin_m = 0.2;
    cfg.runtime_min_point_z_m = -0.2;
    cfg.runtime_max_point_z_m = 0.5;
    // Valid 20deg plane.
    const double a = 20.0 * M_PI / 180.0;
    const std::array<double, 3> n20{0.0, std::sin(a), std::cos(a)};
    RB_CHECK(!rb_servo::validateUserFloorPlaneRequest({0, 0, 0.05}, n20, 0.0, cfg).has_value());
    // Disabled.
    rb_servo::UserFloorConstraintConfig disabled;
    RB_CHECK(rb_servo::validateUserFloorPlaneRequest({0, 0, 0}, {0, 0, 1}, 0.0, disabled) ==
             std::string("user_floor_disabled"));
    // Non-finite.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    RB_CHECK(rb_servo::validateUserFloorPlaneRequest({0, 0, nan}, {0, 0, 1}, 0.0, cfg) ==
             std::string("user_floor_point_not_finite"));
    // Degenerate / non-unit normal.
    RB_CHECK(rb_servo::validateUserFloorPlaneRequest({0, 0, 0}, {0, 0, 0}, 0.0, cfg) ==
             std::string("user_floor_normal_degenerate"));
    RB_CHECK(rb_servo::validateUserFloorPlaneRequest({0, 0, 0}, {0, 0, 2}, 0.0, cfg) ==
             std::string("user_floor_normal_not_unit"));
    // Downward normal.
    RB_CHECK(rb_servo::validateUserFloorPlaneRequest({0, 0, 0}, {0, 0, -1}, 0.0, cfg) ==
             std::string("user_floor_normal_not_upward"));
    // Excessive tilt (50deg > 35).
    const double b = 50.0 * M_PI / 180.0;
    RB_CHECK(rb_servo::validateUserFloorPlaneRequest(
        {0, 0, 0}, {0.0, std::sin(b), std::cos(b)}, 0.0, cfg) ==
             std::string("user_floor_tilt_excessive"));
    // Margin out of range.
    RB_CHECK(rb_servo::validateUserFloorPlaneRequest({0, 0, 0}, {0, 0, 1}, 0.5, cfg) ==
             std::string("user_floor_margin_out_of_range"));
    // Point z below / above runtime band.
    RB_CHECK(rb_servo::validateUserFloorPlaneRequest({0, 0, -0.5}, {0, 0, 1}, 0.0, cfg) ==
             std::string("user_floor_point_below_runtime_min"));
    RB_CHECK(rb_servo::validateUserFloorPlaneRequest({0, 0, 0.9}, {0, 0, 1}, 0.0, cfg) ==
             std::string("user_floor_point_above_runtime_max"));
    return true;
}

}  // namespace

int main() {
    if (!testHorizontalPlaneSignedDistance()) return 1;
    if (!testTiltedPlaneSignedDistance()) return 1;
    if (!testOffsetPointBindsLowest()) return 1;
    if (!testMarginFoldsIn()) return 1;
    if (!testEscapeAllowsAscentWhileBelow()) return 1;
    if (!testMonitorOnlyAndDisabledAllow()) return 1;
    if (!testFkFailureFailsClosed()) return 1;
    if (!testValidateRequest()) return 1;
    std::cout << "user_floor_constraint tests passed\n";
    return 0;
}
