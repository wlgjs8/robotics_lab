// Authoritative coal eval of the live pose using the REAL CollisionMonitor.
// Mirrors the stack_real.yaml self_collision.mesh geometry config (articulated
// gripper) and prints every near pair so we can see the gripper<->gripper number.
#include <cstdio>
#include <string>
#include "rb_servo/control/collision_monitor.hpp"
using namespace rb_servo;

static const std::string WS = "/home/plaif/workspace";
int main(int argc, char** argv) {
    const bool raw = argc > 1 && std::string(argv[1]) == "raw";  // raw=non-convex meshes
    CollisionMonitorConfig c;
    c.enable = true;
    const std::string urdf_dir =
        WS + "/mo_robot_descriptions/mo_robot_descriptions/robots/urdf/dual_rb3_730e";
    c.unified_urdf = urdf_dir + "/dual_rb3_730e_ver5.urdf";
    c.package_dirs = {urdf_dir};
    // The convex HULLS moved to the rb5_850e tree on 2026-09-02 (they are what the
    // server config depends on); the RAW meshes stayed with the rb3_730e assets that
    // still use them -- rb3_730e_pika_articulated.urdf, test_collision_monitor.cpp and
    // the capsule scripts. This tool needs both, because its raw mode exists precisely
    // to drive a non-convex mesh through the guard.
    const std::string mesh_root =
        WS + "/robotics_lab/rb_servo_server/descriptions/meshes/robots/";
    const std::string hull_dir = mesh_root + "rb5_850e/visual/tool/";
    const std::string raw_dir = mesh_root + "rb3_730e/visual/tool/";
    const std::string tool = raw ? raw_dir : hull_dir;
    const std::string sfx = raw ? ".STL" : "_hull.STL";  // _hull = the new fix
    printf("=== mesh variant: %s ===\n", raw ? "RAW (non-convex, tests step-2 guard)"
                                             : "HULL (production fix)");
    c.pika_gripper_mesh = hull_dir + "pika_gripper_hull.STL";
    c.pika_gripper_base_mesh = tool + "pika_gripper_base" + sfx;
    c.pika_finger_left_mesh = tool + "pika_finger_left" + sfx;
    c.pika_finger_right_mesh = tool + "pika_finger_right" + sfx;
    c.gripper_finger_travel_m = 0.047;
    c.stand_ignore_arm_substrings = {"link0"};
    c.check_intra_arm = true;
    c.intra_arm_min_chain_separation = 2;
    c.swept_samples = 1;          // endpoint (static pose); sweep irrelevant for one q
    c.max_near_pairs = 20;        // show plenty
    c.disabled_collision_pairs = {
        {"*left*link0*", "*left*link1*"}, {"*right*link0*", "*right*link1*"},
        {"*left*link0*", "*left*link2*"}, {"*right*link0*", "*right*link2*"},
        {"*left*link4*", "*left*link6*"}, {"*right*link4*", "*right*link6*"},
    };
    const char* jn[kDof] = {"base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"};
    for (int i = 0; i < kDof; ++i) {
        c.left_joints[i] = std::string("dual_rb3_730e_left_") + jn[i] + "_joint";
        c.right_joints[i] = std::string("dual_rb3_730e_right_") + jn[i] + "_joint";
    }

    CollisionMonitor mon(c);
    printf("geoms=%zu pairs=%zu\n", mon.numGeometries(), mon.numPairs());

    JointArray L{315.053, 77.7532, 97.5803, 37.4993, -122.826, -115.576};
    JointArray R{-253.709, -63.0735, -93.8389, 79.992, 90.8985, 123.119};
    // default jaw OPEN (100%) — the conservative envelope the servo uses on invalid fb
    CollisionVerdict v = mon.evalOnce(L, R);
    printf("valid=%d global_min=%.2f mm  self_min=%.2f  intra_min=%.2f  hard=%d\n",
           v.valid, v.min_clearance_m * 1000, v.self_min_clearance_m * 1000,
           v.intra_arm_min_clearance_m * 1000, v.hard_violation);
    printf("--- near pairs (sorted) ---\n");
    for (const auto& p : v.near) {
        printf("  %.2f mm  | %s <-> %s%s\n", p.d_m * 1000,
               p.name_a.c_str(), p.name_b.c_str(), p.intra_arm ? "  [intra]" : "");
    }
    return 0;
}
