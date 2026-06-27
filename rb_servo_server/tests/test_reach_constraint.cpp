#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "rb_servo/control/reach_constraint.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

rb_servo::ReachConstraintConfig enabledConfig() {
    rb_servo::ReachConstraintConfig config;
    config.enable = true;
    config.r_min_m = 0.2;
    config.r_max_m = 0.8;
    return config;
}

rb_servo::Pose6D poseAt(double x, double y, double z) {
    rb_servo::Pose6D p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

// Base at origin; evaluate a bare TCP point (no offsets) against the shell.
rb_servo::ReachArmEvaluation evalPoint(double x, double y, double z) {
    rb_servo::ReachArmEvaluation r;
    const auto cfg = enabledConfig();
    rb_servo::reachEvaluateShell(poseAt(x, y, z), {0.0, 0.0, 0.0}, {},
                                 cfg.r_min_m, cfg.r_max_m, &r);
    return r;
}

bool testInsidePointAllowsAndMargins() {
    // r = 0.6 from base: min margin is min(0.6 - 0.2, 0.8 - 0.6) = 0.2 (outer).
    const auto r = evalPoint(0.6, 0.0, 0.0);
    RB_CHECK(r.checked);
    RB_CHECK(!r.violated);
    RB_CHECK(std::abs(r.r_far_m - 0.6) < 1e-12);
    RB_CHECK(std::abs(r.r_near_m - 0.6) < 1e-12);
    RB_CHECK(std::abs(r.min_margin_m - 0.2) < 1e-12);  // closest is outer (r_max)
    RB_CHECK(r.closest_shell == "r_max");
    RB_CHECK(r.outside_depth_m == 0.0);
    // Radial unit direction at the binding point is +x (point on +x axis).
    RB_CHECK(std::abs(r.shells[1].dir_stand[0] - 1.0) < 1e-12);
    const auto cfg = enabledConfig();
    RB_CHECK(rb_servo::decideReachAction(r, r, cfg) == rb_servo::FloorAction::Allow);
    return true;
}

bool testNearInnerShellReported() {
    // r = 0.25: closest shell is r_min, margin 0.05.
    const auto r = evalPoint(0.0, 0.25, 0.0);
    RB_CHECK(!r.violated);
    RB_CHECK(std::abs(r.min_margin_m - 0.05) < 1e-12);
    RB_CHECK(r.closest_shell == "r_min");
    RB_CHECK(std::abs(r.shells[0].dir_stand[1] - 1.0) < 1e-12);  // +y radial
    return true;
}

bool testOutsideOuterViolatesAndHoldsOrLatches() {
    auto cfg = enabledConfig();
    // r = 0.9 is 0.1 beyond r_max.
    const auto out = evalPoint(0.9, 0.0, 0.0);
    RB_CHECK(out.checked);
    RB_CHECK(out.violated);
    RB_CHECK(out.min_margin_m < 0.0);
    RB_CHECK(std::abs(out.outside_depth_m - 0.1) < 1e-12);
    RB_CHECK(out.closest_shell == "r_max");
    const auto inside = evalPoint(0.5, 0.0, 0.0);
    RB_CHECK(rb_servo::decideReachAction(out, inside, cfg) == rb_servo::FloorAction::Hold);
    cfg.fail_policy = rb_servo::FloorConstraintFailPolicy::FaultLatch;
    RB_CHECK(rb_servo::decideReachAction(out, inside, cfg) == rb_servo::FloorAction::Latch);
    return true;
}

bool testInsideInnerViolates() {
    // r = 0.1 is 0.1 inside r_min (too close to the base).
    const auto out = evalPoint(0.1, 0.0, 0.0);
    RB_CHECK(out.violated);
    RB_CHECK(out.closest_shell == "r_min");
    RB_CHECK(std::abs(out.shells[0].margin_m - (0.1 - 0.2)) < 1e-12);
    RB_CHECK(std::abs(out.outside_depth_m - 0.1) < 1e-12);
    return true;
}

bool testEscapeAllowsReturnWhileOutside() {
    const auto cfg = enabledConfig();
    const auto deep = evalPoint(1.0, 0.0, 0.0);     // 0.2 outside r_max
    const auto shallow = evalPoint(0.9, 0.0, 0.0);  // 0.1 outside r_max
    // Coming back toward the shell (outside-depth shrinking): allow.
    RB_CHECK(rb_servo::decideReachAction(shallow, deep, cfg) == rb_servo::FloorAction::Allow);
    // Going further out: hold.
    RB_CHECK(rb_servo::decideReachAction(deep, shallow, cfg) == rb_servo::FloorAction::Hold);
    // Tangential while outside (same depth): allow.
    RB_CHECK(rb_servo::decideReachAction(shallow, shallow, cfg) == rb_servo::FloorAction::Allow);
    return true;
}

bool testInnerShellDisabledWhenRMinNonPositive() {
    rb_servo::ReachArmEvaluation r;
    rb_servo::ReachConstraintConfig cfg;
    cfg.enable = true;
    cfg.r_min_m = 0.0;  // disabled inner shell
    cfg.r_max_m = 0.8;
    // A point at the base (r=0) is NOT a violation when the inner shell is off.
    RB_CHECK(rb_servo::reachEvaluateShell(poseAt(0.0, 0.0, 0.0), {0.0, 0.0, 0.0}, {},
                                          cfg.r_min_m, cfg.r_max_m, &r));
    RB_CHECK(r.checked);
    RB_CHECK(!r.violated);
    RB_CHECK(!std::isfinite(r.shells[0].margin_m));  // inner margin = +inf
    RB_CHECK(r.closest_shell == "r_max");
    return true;
}

bool testBindingPointWithOffsets() {
    // Two fingertip points +-0.1 m along the TCP x-axis. Base at origin, TCP on
    // the +x axis at x=0.75 with identity orientation: tip_a -> 0.85 (0.05 beyond
    // r_max=0.8), tip_b -> 0.65. The outer shell binds on tip_a, inner on tip_b.
    std::vector<rb_servo::FloorCheckPointConfig> points;
    points.push_back({"tip_a", {0.1, 0.0, 0.0}});
    points.push_back({"tip_b", {-0.1, 0.0, 0.0}});
    const auto cfg = enabledConfig();
    rb_servo::ReachArmEvaluation r;
    RB_CHECK(rb_servo::reachEvaluateShell(poseAt(0.75, 0.0, 0.0), {0.0, 0.0, 0.0}, points,
                                          cfg.r_min_m, cfg.r_max_m, &r));
    RB_CHECK(r.violated);
    RB_CHECK(r.closest_shell == "r_max");
    RB_CHECK(std::abs(r.r_far_m - 0.85) < 1e-9);
    RB_CHECK(std::abs(r.r_near_m - 0.65) < 1e-9);
    RB_CHECK(std::abs(r.shells[1].offset_tcp.x() - 0.1) < 1e-12);   // outer binds tip_a
    RB_CHECK(std::abs(r.shells[0].offset_tcp.x() + 0.1) < 1e-12);   // inner binds tip_b
    RB_CHECK(std::abs(r.shells[1].margin_m - (0.8 - 0.85)) < 1e-9);
    return true;
}

bool testOffCenterBase() {
    // Base offset: r is measured from the base, not the stand origin.
    rb_servo::ReachArmEvaluation r;
    const auto cfg = enabledConfig();
    RB_CHECK(rb_servo::reachEvaluateShell(poseAt(1.0, 0.0, 0.0), {0.5, 0.0, 0.0}, {},
                                          cfg.r_min_m, cfg.r_max_m, &r));
    RB_CHECK(std::abs(r.r_far_m - 0.5) < 1e-12);  // |1.0 - 0.5|
    RB_CHECK(!r.violated);
    return true;
}

bool testMonitorOnlyAndDisabledAllow() {
    auto cfg = enabledConfig();
    cfg.monitor_only = true;
    const auto out = evalPoint(5.0, 0.0, 0.0);
    RB_CHECK(rb_servo::decideReachAction(out, out, cfg) == rb_servo::FloorAction::Allow);
    rb_servo::ReachConstraintConfig disabled;  // enable=false
    RB_CHECK(rb_servo::decideReachAction(out, out, disabled) == rb_servo::FloorAction::Allow);
    return true;
}

bool testFkFailureFailsClosed() {
    auto cfg = enabledConfig();
    rb_servo::ReachArmEvaluation unchecked;  // checked=false
    const auto inside = evalPoint(0.5, 0.0, 0.0);
    RB_CHECK(rb_servo::decideReachAction(unchecked, inside, cfg) == rb_servo::FloorAction::Hold);
    cfg.fail_policy = rb_servo::FloorConstraintFailPolicy::FaultLatch;
    RB_CHECK(rb_servo::decideReachAction(unchecked, inside, cfg) == rb_servo::FloorAction::Latch);
    // Non-finite TCP -> checked=false (fail closed).
    rb_servo::Pose6D bad = poseAt(0.5, 0.0, std::numeric_limits<double>::quiet_NaN());
    rb_servo::ReachArmEvaluation r;
    RB_CHECK(!rb_servo::reachEvaluateShell(bad, {0.0, 0.0, 0.0}, {}, cfg.r_min_m, cfg.r_max_m, &r));
    RB_CHECK(!r.checked);
    return true;
}

}  // namespace

int main() {
    if (!testInsidePointAllowsAndMargins()) return 1;
    if (!testNearInnerShellReported()) return 1;
    if (!testOutsideOuterViolatesAndHoldsOrLatches()) return 1;
    if (!testInsideInnerViolates()) return 1;
    if (!testEscapeAllowsReturnWhileOutside()) return 1;
    if (!testInnerShellDisabledWhenRMinNonPositive()) return 1;
    if (!testBindingPointWithOffsets()) return 1;
    if (!testOffCenterBase()) return 1;
    if (!testMonitorOnlyAndDisabledAllow()) return 1;
    if (!testFkFailureFailsClosed()) return 1;
    std::cout << "reach_constraint tests passed\n";
    return 0;
}
