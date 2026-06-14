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
    RB_CHECK(v.min_clearance_m > 0.03);   // >30 mm: link0 mount disabled, rest clear
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

    // (4) hard violation: block approach, allow retreat; invalid -> stop.
    CollisionVerdict hv = v; hv.hard_violation = true;
    hv.closing_speed_m_s = 0.5;  // approaching while breached -> stop
    RB_CHECK(collisionVelocityScale(hv, cfg) == 0.0);
    hv.closing_speed_m_s = 0.0;  // receding/holding while breached -> allow escape
    RB_CHECK(collisionVelocityScale(hv, cfg) == 1.0);
    CollisionVerdict inv; // default invalid
    RB_CHECK(collisionVelocityScale(inv, cfg) == 0.0);

    // (5) staleness.
    RB_CHECK(collisionVerdictStale(inv, 100.0, cfg.max_staleness_s));
    RB_CHECK(!collisionVerdictStale(v, v.stamp_s + 0.001, cfg.max_staleness_s));
    RB_CHECK(collisionVerdictStale(v, v.stamp_s + 1.0, cfg.max_staleness_s));

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
