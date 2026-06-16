#include <cmath>
#include <iostream>
#include <limits>

#include "rb_servo/control/floor_constraint.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

rb_servo::FloorConstraintConfig enabledConfig() {
    rb_servo::FloorConstraintConfig config;
    config.enable = true;
    config.z_min_m = 0.010;
    config.runtime_min_z_m = 0.0;
    config.runtime_max_z_m = 0.5;
    return config;
}

rb_servo::FloorArmEvaluation eval(double z) {
    rb_servo::FloorArmEvaluation result;
    result.checked = true;
    result.tcp_z_m = z;
    result.violated = false;
    return result;
}

bool testDisabledAlwaysAllows() {
    rb_servo::FloorConstraintConfig config;  // enable=false
    RB_CHECK(rb_servo::decideFloorAction(eval(-1.0), eval(-1.0), config, 0.01) ==
             rb_servo::FloorAction::Allow);
    return true;
}

bool testAbovePlaneAllows() {
    const auto config = enabledConfig();
    RB_CHECK(rb_servo::decideFloorAction(eval(0.5), eval(0.5), config, 0.01) ==
             rb_servo::FloorAction::Allow);
    // Exactly on the plane is legal (>=).
    RB_CHECK(rb_servo::decideFloorAction(eval(0.01), eval(0.5), config, 0.01) ==
             rb_servo::FloorAction::Allow);
    return true;
}

bool testBelowPlaneHoldsOrLatches() {
    auto config = enabledConfig();
    // Moving down below the plane: hold.
    RB_CHECK(rb_servo::decideFloorAction(eval(0.005), eval(0.02), config, 0.01) ==
             rb_servo::FloorAction::Hold);
    // FaultLatch policy: latch instead.
    config.fail_policy = rb_servo::FloorConstraintFailPolicy::FaultLatch;
    RB_CHECK(rb_servo::decideFloorAction(eval(0.005), eval(0.02), config, 0.01) ==
             rb_servo::FloorAction::Latch);
    return true;
}

bool testEscapeAllowsLateralOrUpwardWhileBelow() {
    const auto config = enabledConfig();
    // Already below the plane, candidate strictly higher than previous sent: allow.
    RB_CHECK(rb_servo::decideFloorAction(eval(0.005), eval(0.003), config, 0.01) ==
             rb_servo::FloorAction::Allow);
    // Same height while below: allow lateral sliding at the floor.
    RB_CHECK(rb_servo::decideFloorAction(eval(0.005), eval(0.005), config, 0.01) ==
             rb_servo::FloorAction::Allow);
    // Going further down while below: hold.
    RB_CHECK(rb_servo::decideFloorAction(eval(0.003), eval(0.005), config, 0.01) ==
             rb_servo::FloorAction::Hold);
    return true;
}

bool testFkFailureFailsClosed() {
    auto config = enabledConfig();
    rb_servo::FloorArmEvaluation unchecked;  // checked=false
    RB_CHECK(rb_servo::decideFloorAction(unchecked, eval(0.5), config, 0.01) ==
             rb_servo::FloorAction::Hold);
    config.fail_policy = rb_servo::FloorConstraintFailPolicy::FaultLatch;
    RB_CHECK(rb_servo::decideFloorAction(unchecked, eval(0.5), config, 0.01) ==
             rb_servo::FloorAction::Latch);
    // Previous-sent unchecked: no escape evidence, a violating candidate holds.
    config.fail_policy = rb_servo::FloorConstraintFailPolicy::ClampToHold;
    RB_CHECK(rb_servo::decideFloorAction(eval(0.005), unchecked, config, 0.01) ==
             rb_servo::FloorAction::Hold);
    return true;
}

bool testMonitorOnlyAllowsViolations() {
    auto config = enabledConfig();
    config.monitor_only = true;
    RB_CHECK(rb_servo::decideFloorAction(eval(-0.5), eval(-0.5), config, 0.01) ==
             rb_servo::FloorAction::Allow);
    rb_servo::FloorArmEvaluation unchecked;
    RB_CHECK(rb_servo::decideFloorAction(unchecked, unchecked, config, 0.01) ==
             rb_servo::FloorAction::Allow);
    return true;
}

bool testValidateFloorZRequest() {
    const auto config = enabledConfig();
    RB_CHECK(!rb_servo::validateFloorZRequest(0.010, config).has_value());
    // Bounds are inclusive.
    RB_CHECK(!rb_servo::validateFloorZRequest(0.0, config).has_value());
    RB_CHECK(!rb_servo::validateFloorZRequest(0.5, config).has_value());
    RB_CHECK(*rb_servo::validateFloorZRequest(-0.001, config) == "floor_z_below_runtime_min");
    RB_CHECK(*rb_servo::validateFloorZRequest(0.501, config) == "floor_z_above_runtime_max");
    RB_CHECK(*rb_servo::validateFloorZRequest(std::numeric_limits<double>::quiet_NaN(), config) ==
             "floor_z_not_finite");
    RB_CHECK(*rb_servo::validateFloorZRequest(std::numeric_limits<double>::infinity(), config) ==
             "floor_z_not_finite");
    rb_servo::FloorConstraintConfig disabled;
    RB_CHECK(*rb_servo::validateFloorZRequest(0.01, disabled) == "floor_constraint_disabled");
    return true;
}

bool testLowestZWithOffsets() {
    // Two fingertip points +-54 mm along the TCP x-axis (PIKA gripper layout).
    std::vector<rb_servo::FloorCheckPointConfig> points;
    points.push_back({"tip_a", {0.054, 0.0, 0.0}});
    points.push_back({"tip_b", {-0.054, 0.0, 0.0}});

    // Identity orientation: offsets are horizontal, the TCP is the lowest point.
    rb_servo::Pose6D level;
    level.z = 0.100;
    std::string lowest;
    rb_servo::math::Vector3 offset;
    double z = rb_servo::floorLowestZWithOffsets(level, points, &lowest, &offset);
    RB_CHECK(std::abs(z - 0.100) < 1e-12);
    RB_CHECK(lowest == "tcp");
    RB_CHECK(offset.norm() < 1e-12);

    // Pitch the tool 90 deg about y: the TCP x-axis maps to -z, so tip_a hangs
    // 54 mm BELOW the TCP point and must win the check.
    rb_servo::Pose6D rolled = level;
    rolled.ry = M_PI / 2.0;
    z = rb_servo::floorLowestZWithOffsets(rolled, points, &lowest, &offset);
    RB_CHECK(std::abs(z - (0.100 - 0.054)) < 1e-9);
    RB_CHECK(lowest == "tip_a");
    RB_CHECK(std::abs(offset.x() - 0.054) < 1e-12);

    // Opposite pitch: the other fingertip leads.
    rolled.ry = -M_PI / 2.0;
    z = rb_servo::floorLowestZWithOffsets(rolled, points, &lowest, &offset);
    RB_CHECK(std::abs(z - (0.100 - 0.054)) < 1e-9);
    RB_CHECK(lowest == "tip_b");
    RB_CHECK(std::abs(offset.x() + 0.054) < 1e-12);

    // No offset points: plain TCP z (legacy behavior).
    z = rb_servo::floorLowestZWithOffsets(rolled, {}, &lowest);
    RB_CHECK(std::abs(z - 0.100) < 1e-12);
    RB_CHECK(lowest == "tcp");

    // Non-finite TCP fails closed (NaN propagates to the caller).
    rb_servo::Pose6D invalid = level;
    invalid.z = std::numeric_limits<double>::quiet_NaN();
    RB_CHECK(std::isnan(rb_servo::floorLowestZWithOffsets(invalid, points, &lowest)));
    return true;
}

}  // namespace

int main() {
    if (!testDisabledAlwaysAllows()) return 1;
    if (!testAbovePlaneAllows()) return 1;
    if (!testBelowPlaneHoldsOrLatches()) return 1;
    if (!testEscapeAllowsLateralOrUpwardWhileBelow()) return 1;
    if (!testFkFailureFailsClosed()) return 1;
    if (!testMonitorOnlyAllowsViolations()) return 1;
    if (!testValidateFloorZRequest()) return 1;
    if (!testLowestZWithOffsets()) return 1;
    std::cout << "floor_constraint tests passed\n";
    return 0;
}
