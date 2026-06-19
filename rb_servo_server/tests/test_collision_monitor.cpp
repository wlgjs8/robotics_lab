// Unit test for the async mesh self-collision monitor.
// Skips gracefully if the unified URDF (mo_robot_descriptions sibling repo) is absent.

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <thread>

#include "rb_servo/control/collision_monitor.hpp"

#define RB_CHECK(expr)                                                          \
    do {                                                                       \
        if (!(expr)) {                                                         \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":"    \
                      << __LINE__ << "\n";                                     \
            return false;                                                     \
        }                                                                     \
    } while (0)

namespace fs = std::filesystem;
using namespace rb_servo;

static fs::path workspaceRoot() {
    // tests/ -> rb_servo_server -> robotics_lab -> workspace
    return fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
}

static CollisionMonitorConfig makeConfig(const fs::path& ws) {
    CollisionMonitorConfig c;
    c.enable = true;
    const fs::path urdf_dir =
        ws / "mo_robot_descriptions/mo_robot_descriptions/robots/urdf/dual_rb3_730e";
    c.unified_urdf = (urdf_dir / "dual_rb3_730e_ver3.urdf").string();
    c.package_dirs = {urdf_dir.string()};  // so "../../../meshes" resolves
    c.pika_gripper_mesh =
        (ws / "robotics_lab/rb_servo_server/descriptions/meshes/robots/rb3_730e/visual/tool/"
              "pika_gripper.STL")
            .string();
    const char* jn[kDof] = {"base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"};
    for (int i = 0; i < kDof; ++i) {
        c.left_joints[i] = std::string("dual_rb3_730e_left_") + jn[i] + "_joint";
        c.right_joints[i] = std::string("dual_rb3_730e_right_") + jn[i] + "_joint";
    }
    return c;
}

static bool run() {
    const fs::path ws = workspaceRoot();
    CollisionMonitorConfig cfg = makeConfig(ws);
    if (!fs::is_regular_file(cfg.unified_urdf)) {
        std::cout << "SKIP: unified URDF not found (" << cfg.unified_urdf << ")\n";
        return true;  // not a failure on machines without the sibling repo
    }

    CollisionMonitor mon(cfg);

    // geometry: link0..6 hulls (left 12 / right 12) + stand 7 + 2 gripper = 33
    std::cout << "geoms=" << mon.numGeometries() << " pairs=" << mon.numPairs() << "\n";
    RB_CHECK(mon.numGeometries() == 33);
    RB_CHECK(mon.numPairs() > 0);

    const JointArray init = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};

    // (1) init pose: no collision, clearance comfortably positive (PoC ~97 mm).
    CollisionVerdict v = mon.evalOnce(init, init);
    std::cout << "init min_clearance=" << v.min_clearance_m * 1000.0 << " mm  near="
              << (v.near.empty() ? "?" : v.near.front().name_a + "<->" + v.near.front().name_b)
              << "\n";
    RB_CHECK(v.valid);
    RB_CHECK(!v.hard_violation);
    RB_CHECK(std::isfinite(v.min_clearance_m));
    // Closest at init is the intra-arm link2<->link4 pair (~28 mm across the elbow):
    // well clear of the 5 mm hard floor; not a violation. (>15 mm regression guard.)
    RB_CHECK(v.min_clearance_m > 0.015);
    RB_CHECK(!v.near.empty());
    RB_CHECK(v.near.front().n.norm() > 0.5);  // unit approach dir populated

    // (2) velocity barrier: at large clearance, full speed regardless.
    RB_CHECK(collisionVelocityScale(v, cfg) == 1.0);

    // (3) barrier monotonicity + safety on a synthetic near/closing verdict.
    CollisionVerdict close = v;
    close.min_clearance_m = 0.010;   // 10 mm, inside d_slow
    close.closing_speed_m_s = 0.0;   // not closing -> free (anti-twitchy)
    RB_CHECK(collisionVelocityScale(close, cfg) == 1.0);
    close.closing_speed_m_s = 1.0;   // closing fast -> must slow (<1)
    const double s_fast = collisionVelocityScale(close, cfg);
    RB_CHECK(s_fast < 1.0 && s_fast >= 0.0);
    close.closing_speed_m_s = 0.05;  // closing slowly at 10 mm -> allowed full
    RB_CHECK(collisionVelocityScale(close, cfg) == 1.0);

    // (4) hard violation: HOLD unless clearly retreating; invalid -> stop.
    CollisionVerdict hv = v; hv.hard_violation = true;
    hv.clearance_rate_m_s = -0.5;  // approaching (clearance shrinking) -> stop
    RB_CHECK(collisionVelocityScale(hv, cfg) == 0.0);
    hv.clearance_rate_m_s = 0.0;   // stationary while breached -> hold (fail safe)
    RB_CHECK(collisionVelocityScale(hv, cfg) == 0.0);
    hv.clearance_rate_m_s = 0.5;   // clearly retreating -> allow escape
    RB_CHECK(collisionVelocityScale(hv, cfg) == 1.0);
    CollisionVerdict inv; // default invalid
    RB_CHECK(collisionVelocityScale(inv, cfg) == 0.0);

    // (5) staleness.
    RB_CHECK(collisionVerdictStale(inv, 100.0, cfg.max_staleness_s));
    RB_CHECK(!collisionVerdictStale(v, v.stamp_s + 0.001, cfg.max_staleness_s));
    RB_CHECK(collisionVerdictStale(v, v.stamp_s + 1.0, cfg.max_staleness_s));

    // (5b) age extrapolation (Issue 2a): a larger actual verdict age compensates a
    // larger closing distance, so the barrier is monotonically MORE restrictive as
    // the verdict ages. The default (age < 0) and age <= latency_s reproduce the
    // fixed-latency_s behavior exactly (backward compatible).
    CollisionVerdict aged = v;
    aged.min_clearance_m = 0.020;       // inside slow-zone (d_slow 25mm)
    aged.closing_speed_m_s = 0.5;       // fast head-on
    const double s_default = collisionVelocityScale(aged, cfg);            // fixed latency_s
    const double s_age_lat = collisionVelocityScale(aged, cfg, cfg.latency_s);
    const double s_age_small = collisionVelocityScale(aged, cfg, 0.0);     // age 0 -> max(lat,0)=lat
    RB_CHECK(s_default == s_age_lat);   // default arg == explicit latency
    RB_CHECK(s_age_small == s_default); // age below latency floor -> latency_s used
    RB_CHECK(s_default > 0.0 && s_default < 1.0);  // fixed latency: partial slow-down
    const double s_age_big = collisionVelocityScale(aged, cfg, 0.040);     // stale verdict
    RB_CHECK(s_age_big <= s_default);   // older verdict -> never less conservative
    RB_CHECK(s_age_big == 0.0);         // 0.020-0.005-0.5*0.040 = -0.005 <= 0 -> halt

    // (6) threaded publish/consume: start, submit, see a fresh verdict.
    mon.start();
    mon.submitTargets(init, init);
    CollisionVerdict t{};
    for (int i = 0; i < 200 && !t.valid; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        t = mon.latest();
    }
    mon.stop();
    RB_CHECK(t.valid);
    RB_CHECK(t.seq > 0);
    RB_CHECK(std::isfinite(t.min_clearance_m));
    std::cout << "threaded verdict seq=" << t.seq << " min=" << t.min_clearance_m * 1000.0
              << " mm\n";
    return true;
}

int main() {
    if (!run()) {
        std::cerr << "test_collision_monitor FAILED\n";
        return 1;
    }
    std::cout << "test_collision_monitor OK\n";
    return 0;
}
