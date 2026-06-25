// Unit test for the collision-free InitMotion planner + the whole-arm ground plane.
// Skips gracefully if the unified URDF (mo_robot_descriptions sibling repo) is absent.

#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <utility>
#include <vector>

#include "rb_servo/control/collision_monitor.hpp"
#include "rb_servo/control/init_motion_planner.hpp"

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
    return fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
}

static CollisionMonitorConfig makeConfig(const fs::path& ws) {
    CollisionMonitorConfig c;
    c.enable = true;
    const fs::path urdf_dir =
        ws / "mo_robot_descriptions/mo_robot_descriptions/robots/urdf/dual_rb3_730e";
    c.unified_urdf = (urdf_dir / "dual_rb3_730e_ver3.urdf").string();
    c.package_dirs = {urdf_dir.string()};
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

static InitMotionPlannerConfig makePlannerConfig() {
    InitMotionPlannerConfig p;
    p.enable = true;
    p.max_planning_time_sec = 3.0;
    p.max_iterations = 20000;
    p.step_size_rad = 0.20;
    p.edge_resolution_rad = 0.03;
    p.goal_bias = 0.1;
    p.shortcut_passes = 100;
    p.sample_margin_deg = 30.0;
    p.collision_margin_m = 0.005;
    p.seed = 7;
    p.waypoint_tol_deg = 1.5;
    p.max_segment_deg = 5.0;
    return p;
}

static bool run() {
    const fs::path ws = workspaceRoot();
    CollisionMonitorConfig cfg = makeConfig(ws);
    if (!fs::is_regular_file(cfg.unified_urdf)) {
        std::cout << "SKIP: unified URDF not found (" << cfg.unified_urdf << ")\n";
        return true;  // not a failure on machines without the sibling repo
    }

    const JointArray qmin{-360, -360, -360, -360, -360, -360};
    const JointArray qmax{360, 360, 360, 360, 360, 360};
    const JointArray init = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};

    // ---- (1) Whole-arm ground plane: a large static box (world frame) paired with
    // every arm link. An engulfing box must be detected as a hard floor violation; a
    // far-below box must leave the pose clear. This is the whole-arm floor mechanism. --
    {
        CollisionMonitorConfig engulf = cfg;
        ExtraCollisionShape box;
        box.name = "ground_plane";
        box.shape = "box";
        box.parent_frame = "stand";          // static frame -> classified as Stand
        box.size_m = {6.0, 6.0, 6.0};        // engulfs both arms
        box.xyz_m = {0.0, 0.0, 0.0};
        engulf.extra_collision.push_back(box);
        InitMotionPlanner planner(engulf, makePlannerConfig(), qmin, qmax);
        std::cout << "engulfing ground plane: minClearance(init)="
                  << planner.minClearance(init, init) * 1000.0 << " mm\n";
        RB_CHECK(!planner.configClear(init, init));  // every link penetrates the box
    }
    {
        CollisionMonitorConfig low = cfg;
        ExtraCollisionShape box;
        box.name = "ground_plane";
        box.shape = "box";
        box.parent_frame = "stand";
        box.size_m = {6.0, 6.0, 0.10};       // thin slab well below the arms
        box.xyz_m = {0.0, 0.0, -2.0};
        low.extra_collision.push_back(box);
        InitMotionPlanner planner(low, makePlannerConfig(), qmin, qmax);
        RB_CHECK(planner.configClear(init, init));   // arms comfortably above the slab
    }

    // ---- (2) Planner liveness + safety on a clear scene: a path is produced, its
    // endpoints match start/goal, every densified waypoint is collision+floor clear,
    // and no segment exceeds max_segment_deg per joint. ----
    InitMotionPlanner planner(cfg, makePlannerConfig(), qmin, qmax);
    RB_CHECK(planner.configClear(init, init));

    const JointArray goal_l = {15.0, -20.0, 70.0, 10.0, 50.0, 8.0};
    const JointArray goal_r = {-12.0, -35.0, 85.0, -5.0, 65.0, -6.0};
    RB_CHECK(planner.configClear(goal_l, goal_r));

    InitMotionPlanResult res = planner.plan(init, init, goal_l, goal_r);
    std::cout << "plan: success=" << res.success << " waypoints=" << res.waypoints.size()
              << " iters=" << res.iterations << " t=" << res.planning_time_s << "s ("
              << res.message << ")\n";
    RB_CHECK(res.success);
    RB_CHECK(res.waypoints.size() >= 2);

    // Endpoints.
    const auto& first = res.waypoints.front();
    const auto& last = res.waypoints.back();
    for (int i = 0; i < kDof; ++i) {
        RB_CHECK(std::abs(first.first[i] - init[i]) < 1e-6);
        RB_CHECK(std::abs(first.second[i] - init[i]) < 1e-6);
        RB_CHECK(std::abs(last.first[i] - goal_l[i]) < 1e-6);
        RB_CHECK(std::abs(last.second[i] - goal_r[i]) < 1e-6);
    }

    // Safety invariant: every emitted waypoint AND every dense midpoint is clear, and
    // the densification bound holds.
    for (std::size_t s = 0; s < res.waypoints.size(); ++s) {
        RB_CHECK(planner.configClear(res.waypoints[s].first, res.waypoints[s].second));
        if (s + 1 < res.waypoints.size()) {
            const auto& a = res.waypoints[s];
            const auto& b = res.waypoints[s + 1];
            for (int i = 0; i < kDof; ++i) {
                RB_CHECK(std::abs(b.first[i] - a.first[i]) <= 5.0 + 1e-6);
                RB_CHECK(std::abs(b.second[i] - a.second[i]) <= 5.0 + 1e-6);
            }
        }
    }

    // ---- (3) Fail-closed: a goal that is not collision/floor clear yields no plan
    // (no motion). Reuse the engulfing ground plane so the goal config is blocked. ----
    {
        CollisionMonitorConfig engulf = cfg;
        ExtraCollisionShape box;
        box.name = "ground_plane";
        box.shape = "box";
        box.parent_frame = "stand";
        box.size_m = {6.0, 6.0, 6.0};
        box.xyz_m = {0.0, 0.0, 0.0};
        engulf.extra_collision.push_back(box);
        InitMotionPlanner blocked(engulf, makePlannerConfig(), qmin, qmax);
        InitMotionPlanResult bad = blocked.plan(init, init, init, init);
        std::cout << "blocked goal: success=" << bad.success << " (" << bad.message << ")\n";
        RB_CHECK(!bad.success);
        RB_CHECK(bad.waypoints.empty());
    }

    std::cout << "test_init_motion_planner: OK\n";
    return true;
}

int main() {
    return run() ? 0 : 1;
}
