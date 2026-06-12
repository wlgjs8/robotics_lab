#include <algorithm>
#include <cmath>
#include <iostream>

#include "rb_servo/control/joint_smd_tracker.hpp"
#include "rb_servo/control/trajectory_filter.hpp"

namespace {

using rb_servo::ArmCommand;
using rb_servo::ControlMode;
using rb_servo::JointArray;
using rb_servo::JointSmdTracker;
using rb_servo::JointTargetSmdConfig;
using rb_servo::RobotState;
using rb_servo::SafetyConfig;
using rb_servo::ServoConfig;
using rb_servo::TrajectoryFilter;
using rb_servo::kDof;

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

constexpr double kDt = 0.002;  // 500 Hz servo tick

JointTargetSmdConfig defaultSmdConfig() {
    JointTargetSmdConfig cfg;
    cfg.enable = true;
    return cfg;  // zeta=1, fn=0.4Hz, vel [30,30,30,45,45,60], accel [150,...,350]
}

// Critically damped + zero initial velocity: the response approaches the goal
// monotonically (no overshoot) and settles.
bool testStepResponseNoOvershootAndSettles() {
    JointSmdTracker smd(defaultSmdConfig());
    smd.reset(JointArray{}, JointArray{});
    JointArray goal{-124.66, 32.485, 119.074, -96.294, -81.798, -30.615};
    smd.setGoal(goal);
    JointArray q{};
    for (int tick = 0; tick < 5000; ++tick) {  // 10 s
        const JointArray next = smd.step(kDt);
        for (int i = 0; i < kDof; ++i) {
            // Monotone approach: never moves past the goal.
            const double before = goal[i] - q[i];
            const double after = goal[i] - next[i];
            RB_CHECK(before * after >= -1e-9);          // no sign flip (no overshoot)
            RB_CHECK(std::abs(after) <= std::abs(before) + 1e-9);  // error non-increasing
        }
        q = next;
    }
    for (int i = 0; i < kDof; ++i) {
        RB_CHECK(std::abs(q[i] - goal[i]) < 0.05);
    }
    return true;
}

// The profile must respect the configured per-joint velocity and accel limits.
bool testVelocityAndAccelClamps() {
    const JointTargetSmdConfig cfg = defaultSmdConfig();
    JointSmdTracker smd(cfg);
    smd.reset(JointArray{}, JointArray{});
    JointArray goal{180.0, -180.0, 180.0, -180.0, 180.0, -180.0};
    smd.setGoal(goal);
    JointArray prev_q{};
    JointArray prev_dq{};
    for (int tick = 0; tick < 4000; ++tick) {
        const JointArray q = smd.step(kDt);
        for (int i = 0; i < kDof; ++i) {
            const double dq = (q[i] - prev_q[i]) / kDt;
            RB_CHECK(std::abs(dq) <= cfg.max_velocity_deg_s[i] + 1e-6);
            const double ddq = (dq - prev_dq[i]) / kDt;
            RB_CHECK(std::abs(ddq) <= cfg.max_accel_deg_s2[i] + 1e-6);
            prev_dq[i] = dq;
        }
        prev_q = q;
    }
    return true;
}

// reset() must clamp the handed-over velocity into the profile limits.
bool testResetClampsInitialVelocity() {
    const JointTargetSmdConfig cfg = defaultSmdConfig();
    JointSmdTracker smd(cfg);
    JointArray q0{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    JointArray dq0{500.0, -500.0, 500.0, -500.0, 500.0, -500.0};
    smd.reset(q0, dq0);
    smd.setGoal(q0);  // spring force ~0 at the goal: first step shows the velocity
    const JointArray q1 = smd.step(kDt);
    for (int i = 0; i < kDof; ++i) {
        const double dq = (q1[i] - q0[i]) / kDt;
        RB_CHECK(std::abs(dq) <= cfg.max_velocity_deg_s[i] + 1e-6);
    }
    return true;
}

// Identical zeta/fn + zero initial velocity + no clamp engagement: every joint
// covers the same FRACTION of its delta each tick -> straight joint-space line.
bool testStraightLinePathWhenUnclamped() {
    JointTargetSmdConfig cfg = defaultSmdConfig();
    cfg.max_velocity_deg_s = {1e6, 1e6, 1e6, 1e6, 1e6, 1e6};
    cfg.max_accel_deg_s2 = {1e9, 1e9, 1e9, 1e9, 1e9, 1e9};
    JointSmdTracker smd(cfg);
    smd.reset(JointArray{}, JointArray{});
    JointArray goal{10.0, -20.0, 40.0, -80.0, 5.0, -2.5};
    smd.setGoal(goal);
    for (int tick = 0; tick < 1000; ++tick) {
        const JointArray q = smd.step(kDt);
        const double f0 = q[0] / goal[0];
        for (int i = 1; i < kDof; ++i) {
            RB_CHECK(std::abs(q[i] / goal[i] - f0) < 1e-9);
        }
    }
    return true;
}

ArmCommand jointTargetCommand(const JointArray& q) {
    ArmCommand cmd;
    cmd.mode = ControlMode::JointTarget;
    cmd.q_target_deg = q;
    return cmd;
}

SafetyConfig safetyConfigWithSmd(bool enable) {
    SafetyConfig safety;
    safety.dq_max_deg_s = {120.0, 120.0, 120.0, 120.0, 120.0, 120.0};
    safety.ddq_max_deg_s2 = {1e9, 1e9, 1e9, 1e9, 1e9, 1e9};
    safety.joint_target_smd.enable = enable;
    return safety;
}

// enable=false keeps the legacy behavior: full-speed rate-limited ramp.
bool testTrajectoryFilterDisabledKeepsLegacyRamp() {
    TrajectoryFilter filter(ServoConfig{}, safetyConfigWithSmd(false));
    const JointArray prev{};
    const JointArray goal{10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    const JointArray out =
        filter.computeJointTarget(jointTargetCommand(goal), RobotState{}, prev, kDt);
    for (int i = 0; i < kDof; ++i) {
        RB_CHECK(std::abs(out[i] - 120.0 * kDt) < 1e-12);  // dq_max * dt
    }
    return true;
}

// enable=true: the profile ramps smoothly from zero velocity (first steps are
// much slower than the legacy dq_max ramp), reaches the goal, and respects the
// SMD velocity limit end to end.
bool testTrajectoryFilterSmdProfile() {
    const SafetyConfig safety = safetyConfigWithSmd(true);
    TrajectoryFilter filter(ServoConfig{}, safety);
    const JointArray goal{-124.66, 32.485, 119.074, -96.294, -81.798, -30.615};
    JointArray prev{};
    const JointArray first =
        filter.computeJointTarget(jointTargetCommand(goal), RobotState{}, prev, kDt);
    // Starts from rest: first tick step bounded by accel*dt*dt (legacy would
    // jump dq_max*dt = 0.24 deg immediately).
    for (int i = 0; i < kDof; ++i) {
        const double accel_step = safety.joint_target_smd.max_accel_deg_s2[i] * kDt * kDt;
        RB_CHECK(std::abs(first[i] - prev[i]) <= accel_step + 1e-9);
    }
    prev = first;
    double max_dq = 0.0;
    for (int tick = 0; tick < 5000; ++tick) {  // 10 s
        const JointArray q =
            filter.computeJointTarget(jointTargetCommand(goal), RobotState{}, prev, kDt);
        for (int i = 0; i < kDof; ++i) {
            max_dq = std::max(max_dq, std::abs(q[i] - prev[i]) / kDt);
        }
        prev = q;
    }
    RB_CHECK(max_dq <= 60.0 + 1e-6);  // largest per-joint SMD velocity limit
    for (int i = 0; i < kDof; ++i) {
        RB_CHECK(std::abs(prev[i] - goal[i]) < 0.05);
    }
    return true;
}

// If another path (e.g. Cartesian) moved the sent target since the last SMD
// step, the profile must re-baseline from the caller's previous_sent_target
// instead of jumping back to its stale internal state.
bool testTrajectoryFilterRebaselinesAfterExternalMove() {
    TrajectoryFilter filter(ServoConfig{}, safetyConfigWithSmd(true));
    const JointArray goal{30.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    JointArray prev{};
    for (int tick = 0; tick < 100; ++tick) {
        prev = filter.computeJointTarget(jointTargetCommand(goal), RobotState{}, prev, kDt);
    }
    // Simulate an external move of the sent target (Cartesian path ran).
    JointArray moved = prev;
    moved[0] += 5.0;
    const JointArray out =
        filter.computeJointTarget(jointTargetCommand(goal), RobotState{}, moved, kDt);
    // Continues from `moved` (small profile step), not from the stale state.
    RB_CHECK(std::abs(out[0] - moved[0]) < 0.5);
    return true;
}

}  // namespace

int main() {
    if (!testStepResponseNoOvershootAndSettles()) return 1;
    if (!testVelocityAndAccelClamps()) return 1;
    if (!testResetClampsInitialVelocity()) return 1;
    if (!testStraightLinePathWhenUnclamped()) return 1;
    if (!testTrajectoryFilterDisabledKeepsLegacyRamp()) return 1;
    if (!testTrajectoryFilterSmdProfile()) return 1;
    if (!testTrajectoryFilterRebaselinesAfterExternalMove()) return 1;
    std::cout << "joint_smd_tracker tests passed\n";
    return 0;
}
