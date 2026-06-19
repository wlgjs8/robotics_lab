#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "rb_servo/control/roi_box.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

rb_servo::RoiBoxConfig enabledConfig() {
    rb_servo::RoiBoxConfig config;
    config.enable = true;
    config.min_m = {-0.5, -1.0, 0.0};
    config.max_m = {0.5, 0.0, 1.0};
    config.runtime_min_m = {-1.0, -1.5, -0.2};
    config.runtime_max_m = {1.0, 0.5, 1.5};
    return config;
}

rb_servo::Pose6D poseAt(double x, double y, double z) {
    rb_servo::Pose6D p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

// Evaluate a bare TCP point (no offsets) against the standard box.
rb_servo::RoiArmEvaluation evalPoint(double x, double y, double z) {
    rb_servo::RoiArmEvaluation r;
    const auto cfg = enabledConfig();
    rb_servo::roiEvaluateBox(poseAt(x, y, z), {}, cfg.min_m, cfg.max_m, &r);
    return r;
}

bool testInsidePointAllowsAndMargins() {
    // Center of the box: min margin is the distance to the nearest face.
    const auto r = evalPoint(0.0, -0.5, 0.5);
    RB_CHECK(r.checked);
    RB_CHECK(!r.violated);
    RB_CHECK(std::abs(r.min_margin_m - 0.5) < 1e-12);  // 0.5 to every face
    RB_CHECK(r.outside_depth_m == 0.0);
    const auto cfg = enabledConfig();
    RB_CHECK(rb_servo::decideRoiAction(r, r, cfg) == rb_servo::FloorAction::Allow);
    return true;
}

bool testClosestFaceReported() {
    // Near the +x face (x=0.45, max_x=0.5): closest face is x_max, margin 0.05.
    const auto r = evalPoint(0.45, -0.5, 0.5);
    RB_CHECK(!r.violated);
    RB_CHECK(std::abs(r.min_margin_m - 0.05) < 1e-12);
    RB_CHECK(r.closest_face == "x_max");
    // Near the -y face... y in [-1, 0], at y=-0.97 closest to y_min (margin 0.03).
    const auto r2 = evalPoint(0.0, -0.97, 0.5);
    RB_CHECK(std::abs(r2.min_margin_m - 0.03) < 1e-12);
    RB_CHECK(r2.closest_face == "y_min");
    return true;
}

bool testOutsidePointViolatesAndHoldsOrLatches() {
    auto cfg = enabledConfig();
    // x = 0.6 is 0.1 beyond max_x.
    const auto out = evalPoint(0.6, -0.5, 0.5);
    RB_CHECK(out.checked);
    RB_CHECK(out.violated);
    RB_CHECK(out.min_margin_m < 0.0);
    RB_CHECK(std::abs(out.outside_depth_m - 0.1) < 1e-12);
    RB_CHECK(out.closest_face == "x_max");
    // Moving from inside to outside: hold (ClampToHold) / latch (FaultLatch).
    const auto inside = evalPoint(0.0, -0.5, 0.5);
    RB_CHECK(rb_servo::decideRoiAction(out, inside, cfg) == rb_servo::FloorAction::Hold);
    cfg.fail_policy = rb_servo::FloorConstraintFailPolicy::FaultLatch;
    RB_CHECK(rb_servo::decideRoiAction(out, inside, cfg) == rb_servo::FloorAction::Latch);
    return true;
}

bool testEscapeAllowsReturnWhileOutside() {
    const auto cfg = enabledConfig();
    const auto deep = evalPoint(0.7, -0.5, 0.5);     // 0.2 outside
    const auto shallow = evalPoint(0.6, -0.5, 0.5);  // 0.1 outside
    // Coming back toward the box (outside-depth shrinking): allow.
    RB_CHECK(rb_servo::decideRoiAction(shallow, deep, cfg) == rb_servo::FloorAction::Allow);
    // Going further out: hold.
    RB_CHECK(rb_servo::decideRoiAction(deep, shallow, cfg) == rb_servo::FloorAction::Hold);
    // Tangential while outside (same depth): allow.
    RB_CHECK(rb_servo::decideRoiAction(shallow, shallow, cfg) == rb_servo::FloorAction::Allow);
    return true;
}

bool testFaceBindingPointWithOffsets() {
    // Two fingertip points +-54 mm along the TCP x-axis (PIKA gripper layout).
    std::vector<rb_servo::FloorCheckPointConfig> points;
    points.push_back({"tip_a", {0.054, 0.0, 0.0}});
    points.push_back({"tip_b", {-0.054, 0.0, 0.0}});
    const auto cfg = enabledConfig();

    // Identity orientation, TCP at x=0.46: tip_a sits at x=0.514 (0.014 beyond
    // max_x=0.5) so the box is violated on the x_max face even though the TCP
    // point itself is inside.
    rb_servo::RoiArmEvaluation r;
    RB_CHECK(rb_servo::roiEvaluateBox(poseAt(0.46, -0.5, 0.5), points, cfg.min_m, cfg.max_m, &r));
    RB_CHECK(r.violated);
    RB_CHECK(r.closest_face == "x_max");
    // The x_max face must bind on tip_a (offset +0.054), x_min on tip_b (-0.054).
    RB_CHECK(std::abs(r.faces[0][1].offset_tcp.x() - 0.054) < 1e-12);   // x_max
    RB_CHECK(std::abs(r.faces[0][0].offset_tcp.x() + 0.054) < 1e-12);   // x_min
    RB_CHECK(std::abs(r.faces[0][1].margin_m - (0.5 - 0.514)) < 1e-9);
    return true;
}

bool testMonitorOnlyAndDisabledAllow() {
    auto cfg = enabledConfig();
    cfg.monitor_only = true;
    const auto out = evalPoint(2.0, 2.0, 2.0);
    RB_CHECK(rb_servo::decideRoiAction(out, out, cfg) == rb_servo::FloorAction::Allow);
    rb_servo::RoiBoxConfig disabled;  // enable=false
    RB_CHECK(rb_servo::decideRoiAction(out, out, disabled) == rb_servo::FloorAction::Allow);
    return true;
}

bool testFkFailureFailsClosed() {
    auto cfg = enabledConfig();
    rb_servo::RoiArmEvaluation unchecked;  // checked=false
    const auto inside = evalPoint(0.0, -0.5, 0.5);
    RB_CHECK(rb_servo::decideRoiAction(unchecked, inside, cfg) == rb_servo::FloorAction::Hold);
    cfg.fail_policy = rb_servo::FloorConstraintFailPolicy::FaultLatch;
    RB_CHECK(rb_servo::decideRoiAction(unchecked, inside, cfg) == rb_servo::FloorAction::Latch);
    // Non-finite TCP -> checked=false (fail closed).
    rb_servo::Pose6D bad = poseAt(0.0, -0.5, std::numeric_limits<double>::quiet_NaN());
    rb_servo::RoiArmEvaluation r;
    RB_CHECK(!rb_servo::roiEvaluateBox(bad, {}, cfg.min_m, cfg.max_m, &r));
    RB_CHECK(!r.checked);
    return true;
}

bool testValidateRoiBoundsRequest() {
    const auto cfg = enabledConfig();
    RB_CHECK(!rb_servo::validateRoiBoundsRequest({-0.5, -1.0, 0.0}, {0.5, 0.0, 1.0}, cfg).has_value());
    // Inclusive runtime envelope.
    RB_CHECK(!rb_servo::validateRoiBoundsRequest({-1.0, -1.5, -0.2}, {1.0, 0.5, 1.5}, cfg).has_value());
    // min > max on an axis.
    RB_CHECK(*rb_servo::validateRoiBoundsRequest({0.2, -1.0, 0.0}, {0.1, 0.0, 1.0}, cfg) ==
             "roi_min_above_max");
    // below runtime min.
    RB_CHECK(*rb_servo::validateRoiBoundsRequest({-1.1, -1.0, 0.0}, {0.5, 0.0, 1.0}, cfg) ==
             "roi_below_runtime_min");
    // above runtime max.
    RB_CHECK(*rb_servo::validateRoiBoundsRequest({-0.5, -1.0, 0.0}, {1.1, 0.0, 1.0}, cfg) ==
             "roi_above_runtime_max");
    // non-finite.
    RB_CHECK(*rb_servo::validateRoiBoundsRequest(
                 {-0.5, -1.0, std::numeric_limits<double>::quiet_NaN()}, {0.5, 0.0, 1.0}, cfg) ==
             "roi_bounds_not_finite");
    // disabled.
    rb_servo::RoiBoxConfig disabled;
    RB_CHECK(*rb_servo::validateRoiBoundsRequest({0, 0, 0}, {0, 0, 0}, disabled) == "roi_box_disabled");
    return true;
}

}  // namespace

int main() {
    if (!testInsidePointAllowsAndMargins()) return 1;
    if (!testClosestFaceReported()) return 1;
    if (!testOutsidePointViolatesAndHoldsOrLatches()) return 1;
    if (!testEscapeAllowsReturnWhileOutside()) return 1;
    if (!testFaceBindingPointWithOffsets()) return 1;
    if (!testMonitorOnlyAndDisabledAllow()) return 1;
    if (!testFkFailureFailsClosed()) return 1;
    if (!testValidateRoiBoundsRequest()) return 1;
    std::cout << "roi_box tests passed\n";
    return 0;
}
