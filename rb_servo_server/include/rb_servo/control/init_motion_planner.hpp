#pragma once

// Self-contained collision-free InitMotion planner.
//
// The legacy InitMotion is a single JointTarget PTP whose straight joint-space
// interpolation from the current pose to the init pose can pass through a
// self-colliding or floor-violating configuration; the reactive mesh barrier then
// brakes the arms at that boundary and they never REACH the init pose. This planner
// computes a collision-free + floor-safe joint path so InitMotion completes from any
// start pose, with the arms' paths free to overlap in space yet never collide in
// time (both arms are planned together in one combined configuration space).
//
// It is a minimal bidirectional RRT-Connect over the 12-DOF combined config
// (q_left[6] + q_right[6]) — referencing mo_motion_planner's RRTConnect approach but
// reimplemented here with no OMPL/coal dependency. Its state-validity oracle is a
// PRIVATE CollisionMonitor instance (its own pinocchio Data, never started) built
// from the same geometry as the servo guard, INCLUDING the injected ground-plane box
// — so it avoids self-collision (arm<->arm / arm<->stand / intra-arm) AND keeps every
// arm link above the floor. Using a private instance (not the running servo monitor)
// avoids racing the realtime monitor's shared eval buffers / published verdict.
//
// Output is a geometric waypoint list (start..goal); timing is delegated to the
// existing JointTarget SMD streaming + dq/ddq clamps at execution, so there is no
// time parameterization here.

#include <memory>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/collision_monitor.hpp"
#include "rb_servo/core/types.hpp"
#include "rb_servo/kinematics/i_kinematics.hpp"

namespace rb_servo {

struct InitMotionPlanResult {
    enum class FailMode { None, GoalNotClear, EscapeFailed, RrtBudget, NonFinite };

    bool success = false;
    // Collision-free, floor-safe waypoints from start to goal (inclusive), each a
    // (left_deg, right_deg) pair. Densified so no segment exceeds max_segment_deg per
    // joint. Empty on failure.
    std::vector<std::pair<JointArray, JointArray>> waypoints;
    std::string message;       // human-readable outcome / failure reason
    FailMode fail_mode = FailMode::None;
    double start_clear_m = std::numeric_limits<double>::quiet_NaN();
    double goal_clear_m = std::numeric_limits<double>::quiet_NaN();
    int tree_start_size = 0;
    int tree_goal_size = 0;
    int iterations = 0;        // RRT iterations consumed
    double planning_time_s = 0.0;
    // Number of LEADING waypoints that form the gradient-escape segment (when the start
    // was in near-collision). These are sub-threshold and must be followed precisely (no
    // pure-pursuit corner-cutting back into the obstacle). 0 when no escape was needed.
    int escape_waypoints = 0;
};

// Outcome of a collision-free TcpLinearMove decision.
struct InitMotionLinearResult {
    enum class Decision { Straight, Detour, Failed };
    Decision decision = Decision::Failed;
    JointArray goal_left{};   // IK'd target joint config (Straight & Detour)
    JointArray goal_right{};
    // Collision-free detour waypoints (Detour only), start..goal inclusive.
    std::vector<std::pair<JointArray, JointArray>> waypoints;
    std::string message;
    double planning_time_s = 0.0;
    // Leading gradient-escape waypoints in `waypoints` (Detour only); see
    // InitMotionPlanResult::escape_waypoints. 0 when no escape was needed.
    int escape_waypoints = 0;
    // Max per-joint angle (over both arms) between the start pose and the IK'd goal.
    // ~0 means the requested TCP target is essentially the current pose (e.g., the GUI
    // target marker is following current TCP) -> the move is a near no-op. Diagnostic.
    double goal_vs_start_max_deg = 0.0;
    // Max TCP position distance (m, over active arms) between the current pose and the
    // requested target. Pairs with goal_vs_start_max_deg to tell "small gizmo drag"
    // (small metres) apart from a frame/IK oddity (large metres, small degrees). Diagnostic.
    double goal_vs_start_cart_m = 0.0;
};

class InitMotionPlanner {
public:
    // Builds the private collision oracle from monitor_cfg (which should already
    // carry the ground plane). The instance is NOT started (no monitor thread);
    // swept_samples is forced to 1 — the planner does its own dense edge sampling.
    // Throws if the geometry model fails to load (same as the servo monitor).
    //
    // kinematics (optional): a PRIVATE IKinematics instance (own pinocchio Data, NOT
    // the servo loop's) used by planLinearMove for off-thread IK/FK; pass nullptr if
    // collision-free TcpLinearMove is not used. left_mount/right_mount are the arm
    // mount frames the IK/FK need.
    InitMotionPlanner(CollisionMonitorConfig monitor_cfg,
                      InitMotionPlannerConfig planner_cfg,
                      JointArray q_min_deg,
                      JointArray q_max_deg,
                      std::shared_ptr<IKinematics> kinematics = nullptr,
                      ArmMountConfig left_mount = {},
                      ArmMountConfig right_mount = {});
    ~InitMotionPlanner();  // defined in the .cpp (pimpl: Impl is incomplete here)
    InitMotionPlanner(const InitMotionPlanner&) = delete;
    InitMotionPlanner& operator=(const InitMotionPlanner&) = delete;

    // Plan a collision-free path from (start_left,start_right) to the wrapped nearest
    // branch of (goal_left,goal_right). Single-threaded; intended to run on a worker
    // thread off the 500 Hz servo loop. Not re-entrant (mutates the private oracle).
    InitMotionPlanResult plan(const JointArray& start_left, const JointArray& start_right,
                              const JointArray& goal_left, const JointArray& goal_right);

    // Collision-free TcpLinearMove decision (single-threaded; run on a worker). IK the
    // target pose(s), then densely sample the straight Cartesian path and oracle-check
    // each config: if all clear -> Decision::Straight (caller runs the exact MoveL); if
    // any sample collides / dips below the floor / IK-fails -> plan a joint-space detour
    // (RRT-Connect) to the IK'd goal -> Decision::Detour with waypoints. Decision::Failed
    // if the goal IK fails or no detour is found. left_active/right_active select which
    // arms move (an inactive arm holds at its start config across the whole path).
    InitMotionLinearResult planLinearMove(
        const JointArray& start_left, const JointArray& start_right,
        bool left_active, const Pose6D& goal_pose_left,
        bool right_active, const Pose6D& goal_pose_right,
        bool slerp, int check_samples);

    // State-validity oracle: true iff the combined config clears the hard floor by at
    // least collision_margin_m (no self-collision, no floor penetration). Exposed for
    // tests and used internally for every sampled config / edge sample.
    bool configClear(const JointArray& left, const JointArray& right);

    // Minimum mesh clearance (m) at a combined config; +inf if no pairs. For tests.
    double minClearance(const JointArray& left, const JointArray& right);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rb_servo
