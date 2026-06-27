// Unit test for the projection-based pure-pursuit follower (pursueWaypointsStep) used by
// the collision-free InitMotion sequencer and the linear-detour executor.
//
// Regression target: the previous "advance index only when the pose is within
// waypoint_tol_deg of waypoints[index]" logic stalled on any curved (corner) path,
// because the pure-pursuit lookahead cuts the corner and never passes within tol of the
// apex node. A stalled progress pointer froze the commanded target and the arm stopped
// part-way to the init pose (then failed-closed on execution_timeout). The projection
// progress pointer must never stall, so a single click drives smoothly to the goal.

#include <array>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "rb_servo/control/dual_arm_servo_loop.hpp"

#define RB_CHECK(expr)                                                          \
    do {                                                                       \
        if (!(expr)) {                                                         \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":"    \
                      << __LINE__ << "\n";                                     \
            return false;                                                     \
        }                                                                     \
    } while (0)

using namespace rb_servo;

using WP = std::pair<JointArray, JointArray>;

// Build a left-arm waypoint (right arm held at zero). Only joints 0/1 vary in these
// tests, which is enough to form a corner in joint space.
static WP lwp(double j0, double j1) {
    JointArray l{};
    JointArray r{};
    l[0] = j0;
    l[1] = j1;
    return {l, r};
}

static double maxJointErr(const JointArray& a, const JointArray& b) {
    double m = 0.0;
    for (int i = 0; i < kDof; ++i) m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

// A right-angle "L" path: leg 1 drives joint0 0->30 (joint1=0), leg 2 drives joint1
// 0->30 (joint0=30), densified to 5 deg/segment. Apex at (30, 0), goal at (30, 30).
static std::vector<WP> makeCornerPath() {
    std::vector<WP> w;
    for (double j0 = 0.0; j0 <= 30.0 + 1e-9; j0 += 5.0) w.push_back(lwp(j0, 0.0));
    for (double j1 = 5.0; j1 <= 30.0 + 1e-9; j1 += 5.0) w.push_back(lwp(30.0, j1));
    return w;
}

// Direct regression: a pose that has CUT the corner (well past the apex along leg-1's
// direction, but laterally off the apex by far more than tol) must still advance the
// progress pointer past the apex. The old proximity test left index frozen at 0 here.
static bool test_projection_advances_past_cut_corner() {
    const std::vector<WP> w = {lwp(0.0, 0.0), lwp(20.0, 0.0), lwp(20.0, 20.0)};
    // Past the apex on joint0 (21 > 20), but 8 deg off it on joint1 -> NOT within tol.
    JointArray cur_l{};
    cur_l[0] = 21.0;
    cur_l[1] = 8.0;
    JointArray cur_r{};
    std::size_t index = 0;
    const PursuitStep s = pursueWaypointsStep(w, cur_l, cur_r, index,
                                              /*waypoint_tol_deg=*/1.5,
                                              /*lookahead_deg=*/25.0);
    // Proximity (old) would keep index at 0 (cur is >1.5 deg from both node 0 and the
    // apex). Projection must advance past the apex segment.
    RB_CHECK(index == 1);
    RB_CHECK(!s.done);
    return true;
}

// End-to-end: simulate the servo loop streaming this corner path with a per-tick joint
// velocity clamp (which makes the pose cut the corner). The follower must drive to the
// goal and set done, never stalling. The old logic would freeze at the apex and time out.
static bool test_corner_path_reaches_goal_without_stall() {
    const std::vector<WP> w = makeCornerPath();
    const double tol = 1.5;
    const double lookahead = 25.0;
    const double max_step = 2.0;  // per-tick per-joint clamp (deg)

    JointArray cur_l = w.front().first;
    JointArray cur_r = w.front().second;
    std::size_t index = 0;
    std::size_t prev_index = 0;
    bool done = false;
    const int kMaxTicks = 1000;  // path is ~60 deg of motion; this is very generous
    int ticks = 0;
    for (; ticks < kMaxTicks; ++ticks) {
        const PursuitStep s = pursueWaypointsStep(w, cur_l, cur_r, index, tol, lookahead);
        // Monotonic, non-decreasing progress pointer (never walks backward).
        RB_CHECK(index >= prev_index);
        prev_index = index;
        if (s.done) {
            done = true;
            break;
        }
        // Move toward the commanded target with a per-joint velocity clamp; aiming at a
        // far lookahead target in a straight joint-space line is exactly what cuts corners.
        for (int i = 0; i < kDof; ++i) {
            const double dl = s.left[i] - cur_l[i];
            const double dr = s.right[i] - cur_r[i];
            cur_l[i] += std::max(-max_step, std::min(max_step, dl));
            cur_r[i] += std::max(-max_step, std::min(max_step, dr));
        }
    }

    RB_CHECK(done);
    RB_CHECK(ticks < kMaxTicks);
    RB_CHECK(index + 1 == w.size());
    // Pose actually settled at the goal config.
    RB_CHECK(maxJointErr(cur_l, w.back().first) <= tol);
    RB_CHECK(maxJointErr(cur_r, w.back().second) <= tol);
    std::cout << "corner-path reached goal in " << ticks << " ticks (no stall)\n";
    return true;
}

// A degenerate path with a zero-length (duplicate) segment must not divide-by-zero or
// stall: the projection treats it as already passed.
static bool test_degenerate_segment_is_passed() {
    const std::vector<WP> w = {lwp(0.0, 0.0), lwp(0.0, 0.0), lwp(10.0, 0.0)};
    JointArray cur_l = w.front().first;
    JointArray cur_r = w.front().second;
    std::size_t index = 0;
    const PursuitStep s = pursueWaypointsStep(w, cur_l, cur_r, index, 1.5, 25.0);
    // The duplicate node is skipped; the follower aims forward, not at a frozen node.
    RB_CHECK(index >= 1);
    RB_CHECK(!s.done);
    return true;
}

// The gradient-escape head (leading `escape_count` waypoints) must be followed PRECISELY:
// lookahead is suppressed so the command is the immediate next waypoint, never a far
// lookahead node that would cut the corner back toward the obstacle the escape climbs out
// of. Past the escape, the normal lookahead resumes.
static bool test_escape_head_followed_precisely() {
    // 7 nodes along joint0: 0,5,10,...,30.
    std::vector<WP> w;
    for (double j0 = 0.0; j0 <= 30.0 + 1e-9; j0 += 5.0) w.push_back(lwp(j0, 0.0));
    const JointArray cur_l = w.front().first;   // sitting at node 0
    const JointArray cur_r = w.front().second;

    // No escape: 25 deg lookahead aims far ahead (chord 25 <= 25 -> node 5, j0=25).
    {
        std::size_t idx = 0;
        const PursuitStep s = pursueWaypointsStep(w, cur_l, cur_r, idx, 1.5, 25.0, /*escape=*/0);
        RB_CHECK(s.left[0] >= 20.0);  // aimed far ahead
    }
    // Escape head covers nodes 0..3: the target must be the IMMEDIATE next node (j0=5),
    // not the far lookahead node.
    {
        std::size_t idx = 0;
        const PursuitStep s = pursueWaypointsStep(w, cur_l, cur_r, idx, 1.5, 25.0, /*escape=*/4);
        RB_CHECK(std::abs(s.left[0] - 5.0) < 1e-9);
    }
    // Once past the escape head (current pose at node 5, escape_count=4), the normal
    // lookahead resumes and aims far ahead again.
    {
        std::size_t idx = 0;
        const PursuitStep s = pursueWaypointsStep(w, w[5].first, w[5].second, idx, 1.5, 25.0,
                                                  /*escape=*/4);
        RB_CHECK(s.left[0] >= w[5].first[0] + 1e-9);  // advanced beyond node 5
    }
    return true;
}

int main() {
    bool ok = true;
    ok = test_projection_advances_past_cut_corner() && ok;
    ok = test_corner_path_reaches_goal_without_stall() && ok;
    ok = test_degenerate_segment_is_passed() && ok;
    ok = test_escape_head_followed_precisely() && ok;
    if (!ok) {
        std::cerr << "test_init_motion_pursuit: FAILED\n";
        return 1;
    }
    std::cout << "test_init_motion_pursuit: OK\n";
    return 0;
}
