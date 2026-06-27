// Unit test for the collision-free InitMotion planner + the whole-arm ground plane.
// Skips gracefully if the unified URDF (mo_robot_descriptions sibling repo) is absent.

#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include "rb_servo/control/collision_monitor.hpp"
#include "rb_servo/control/init_motion_planner.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"

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

static KinematicsConfig testKinematicsConfig(const fs::path& ws) {
    KinematicsConfig cfg;
    cfg.enable = true;
    cfg.provider = "pinocchio";
    cfg.urdf = (ws / "robotics_lab/rb_servo_server/descriptions/urdf/rb3_730e.urdf").string();
    cfg.base_link = "world";
    cfg.tip_link = "tcp";
    cfg.joint_names = {"base_joint", "shoulder_joint", "elbow_joint",
                       "wrist1_joint", "wrist2_joint", "wrist3_joint"};
    cfg.q_units = "deg";
    cfg.ik.enable = true;
    cfg.ik.timeout_ms = 250.0;
    return cfg;
}

static ArmMountConfig mountFor(ArmId arm) {
    ArmMountConfig m;
    m.arm_id = arm;
    m.base_pose_in_stand = arm == ArmId::Left
        ? Pose6D{0.15707, -0.17036, 0.58036, 2.186649, 0.523831, 2.526296}
        : Pose6D{-0.15707, -0.17036, 0.58036, 2.186649, -0.523831, -2.526296};
    return m;
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

    // Lazy can be disabled to recover the eager edge-checking path. It should still
    // return a fully clear, densified path in the same simple scene.
    {
        InitMotionPlannerConfig eager_cfg = makePlannerConfig();
        eager_cfg.lazy_edges = false;
        eager_cfg.global_sample_fraction = 0.0;
        eager_cfg.sample_margin_deg_per_joint = JointArray{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        InitMotionPlanner eager(cfg, eager_cfg, qmin, qmax);
        InitMotionPlanResult eager_res = eager.plan(init, init, goal_l, goal_r);
        std::cout << "eager plan: success=" << eager_res.success
                  << " waypoints=" << eager_res.waypoints.size() << " ("
                  << eager_res.message << ")\n";
        RB_CHECK(eager_res.success);
        RB_CHECK(eager_res.fail_mode == InitMotionPlanResult::FailMode::None);
        RB_CHECK(eager_res.waypoints.size() >= 2);
        for (const auto& w : eager_res.waypoints) {
            RB_CHECK(eager.configClear(w.first, w.second));
        }
        for (int i = 0; i < kDof; ++i) {
            RB_CHECK(std::abs(eager_res.waypoints.front().first[i] - init[i]) < 1e-6);
            RB_CHECK(std::abs(eager_res.waypoints.back().first[i] - goal_l[i]) < 1e-6);
            RB_CHECK(std::abs(eager_res.waypoints.back().second[i] - goal_r[i]) < 1e-6);
        }
    }

    // Structured diagnostics: non-finite inputs fail closed before oracle use.
    {
        JointArray bad_goal = goal_l;
        bad_goal[0] = std::numeric_limits<double>::infinity();
        InitMotionPlanResult nf = planner.plan(init, init, bad_goal, goal_r);
        std::cout << "nonfinite goal: success=" << nf.success << " (" << nf.message << ")\n";
        RB_CHECK(!nf.success);
        RB_CHECK(nf.waypoints.empty());
        RB_CHECK(nf.fail_mode == InitMotionPlanResult::FailMode::NonFinite);
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
        RB_CHECK(bad.fail_mode == InitMotionPlanResult::FailMode::GoalNotClear);
        RB_CHECK(std::isfinite(bad.goal_clear_m));
    }

    // ---- (4) Collision-free TcpLinearMove (planLinearMove). ----
    // (4a) Without a private kinematics -> graceful Failed (no crash).
    {
        InitMotionPlanner no_kin(cfg, makePlannerConfig(), qmin, qmax);
        const InitMotionLinearResult r = no_kin.planLinearMove(
            init, init, true, Pose6D{}, true, Pose6D{}, false, 20);
        RB_CHECK(r.decision == InitMotionLinearResult::Decision::Failed);
    }

    const fs::path rb3_urdf =
        ws / "robotics_lab/rb_servo_server/descriptions/urdf/rb3_730e.urdf";
    if (fs::is_regular_file(rb3_urdf)) {
        auto kin = std::make_shared<PinocchioKinematics>(testKinematicsConfig(ws));
        const ArmMountConfig lm = mountFor(ArmId::Left);
        const ArmMountConfig rm = mountFor(ArmId::Right);

        // (4b) Zero-move (goal == current TCP): the straight path stays at the clear
        // init config -> Decision::Straight, no detour waypoints. Robust to mount
        // calibration (every sample is ~the known-clear init pose).
        {
            InitMotionPlanner planner_k(cfg, makePlannerConfig(), qmin, qmax, kin, lm, rm);
            const Pose6D pose_l = kin->computeTcpStand(ArmId::Left, init, lm);
            const Pose6D pose_r = kin->computeTcpStand(ArmId::Right, init, rm);
            const InitMotionLinearResult r = planner_k.planLinearMove(
                init, init, true, pose_l, true, pose_r, /*slerp=*/false, 24);
            std::cout << "linear zero-move: decision="
                      << static_cast<int>(r.decision) << " (" << r.message << ")\n";
            RB_CHECK(r.decision == InitMotionLinearResult::Decision::Straight);
            RB_CHECK(r.waypoints.empty());
        }

        // (4c) Blocked: with an engulfing ground plane every config collides, so the
        // straight path is not clear AND the detour goal is not clear -> fail-closed.
        {
            CollisionMonitorConfig engulf = cfg;
            ExtraCollisionShape box;
            box.name = "ground_plane";
            box.shape = "box";
            box.parent_frame = "stand";
            box.size_m = {6.0, 6.0, 6.0};
            box.xyz_m = {0.0, 0.0, 0.0};
            engulf.extra_collision.push_back(box);
            InitMotionPlanner planner_b(engulf, makePlannerConfig(), qmin, qmax, kin, lm, rm);
            const Pose6D pose_l = kin->computeTcpStand(ArmId::Left, init, lm);
            const Pose6D pose_r = kin->computeTcpStand(ArmId::Right, init, rm);
            const InitMotionLinearResult r = planner_b.planLinearMove(
                init, init, true, pose_l, true, pose_r, false, 24);
            std::cout << "linear blocked: decision="
                      << static_cast<int>(r.decision) << " (" << r.message << ")\n";
            RB_CHECK(r.decision != InitMotionLinearResult::Decision::Straight);
        }
    } else {
        std::cout << "SKIP (4b/4c): rb3_730e kinematics URDF not found\n";
    }

    // ---- (5) Gradient escape from a near-collision start. Raise a thin ground slab
    // toward the arms until `init` is JUST near-collision (not clear, but shallow) while
    // a clear goal still exists. Before the escape, RRT could not extend out of such a
    // start (tree_start stayed at the root) and planning failed; with the escape the
    // planner climbs out along increasing clearance and reaches the goal. ----
    {
        // A ground slab whose top sits at stand z=0.09 leaves `init` in shallow near-
        // collision (~4.5 mm, below the clearance gate) while the base-rotated goal
        // {30,-30,80,...} stays clear. Before the escape the RRT could not extend out of
        // `init` (tree_start stayed at the root, planning failed); the gradient escape
        // climbs `init` upward into the clear region, then plans on to the goal.
        const JointArray esc_goal_l = {30.0, -30.0, 80.0, 0.0, 60.0, 0.0};
        const JointArray esc_goal_r = {30.0, -30.0, 80.0, 0.0, 60.0, 0.0};
        CollisionMonitorConfig s = cfg;
        ExtraCollisionShape box;
        box.name = "ground_plane";
        box.shape = "box";
        box.parent_frame = "stand";
        box.size_m = {6.0, 6.0, 0.10};
        box.xyz_m = {0.0, 0.0, 0.09 - 0.05};  // slab top surface at z = 0.09
        s.extra_collision.push_back(box);
        InitMotionPlanner p(s, makePlannerConfig(), qmin, qmax);

        const bool init_near = !p.configClear(init, init) &&
                               p.minClearance(init, init) > 0.0005;  // shallow, positive
        const bool goal_ok = p.configClear(esc_goal_l, esc_goal_r);
        if (init_near && goal_ok) {
            std::cout << "escape scene: start_clear_mm=" << p.minClearance(init, init) * 1000.0
                      << " goal_clear_mm=" << p.minClearance(esc_goal_l, esc_goal_r) * 1000.0 << "\n";
            InitMotionPlanResult r = p.plan(init, init, esc_goal_l, esc_goal_r);
            std::cout << "escape plan: success=" << r.success
                      << " wps=" << r.waypoints.size() << " (" << r.message << ")\n";
            RB_CHECK(r.success);
            RB_CHECK(r.waypoints.size() >= 2);
            // First waypoint is the (near-collision) start; last reaches the goal.
            for (int i = 0; i < kDof; ++i) {
                RB_CHECK(std::abs(r.waypoints.front().first[i] - init[i]) < 1e-6);
                RB_CHECK(std::abs(r.waypoints.front().second[i] - init[i]) < 1e-6);
                RB_CHECK(std::abs(r.waypoints.back().first[i] - esc_goal_l[i]) < 1e-6);
                RB_CHECK(std::abs(r.waypoints.back().second[i] - esc_goal_r[i]) < 1e-6);
            }
            // The start really was near-collision (escape was needed); the goal is clear.
            RB_CHECK(!p.configClear(r.waypoints.front().first, r.waypoints.front().second));
            RB_CHECK(p.configClear(r.waypoints.back().first, r.waypoints.back().second));
            // The path crosses from the near-collision region into the clear region.
            bool became_clear = false;
            for (const auto& w : r.waypoints) {
                if (p.configClear(w.first, w.second)) { became_clear = true; break; }
            }
            RB_CHECK(became_clear);

            InitMotionPlannerConfig no_escape_cfg = makePlannerConfig();
            no_escape_cfg.escape_max_time_sec = 0.0;
            InitMotionPlanner no_escape(s, no_escape_cfg, qmin, qmax);
            InitMotionPlanResult failed_escape =
                no_escape.plan(init, init, esc_goal_l, esc_goal_r);
            std::cout << "escape timeout: success=" << failed_escape.success
                      << " (" << failed_escape.message << ")\n";
            RB_CHECK(!failed_escape.success);
            RB_CHECK(failed_escape.waypoints.empty());
            RB_CHECK(failed_escape.fail_mode == InitMotionPlanResult::FailMode::EscapeFailed);
            RB_CHECK(failed_escape.message.find("restarts_tried=") != std::string::npos);
            RB_CHECK(failed_escape.message.find("escape_time_s=") != std::string::npos);
        } else {
            std::cout << "SKIP (5): geometry differs; could not stage near-collision start\n";
        }
    }

    std::cout << "test_init_motion_planner: OK\n";
    return true;
}

int main() {
    return run() ? 0 : 1;
}
