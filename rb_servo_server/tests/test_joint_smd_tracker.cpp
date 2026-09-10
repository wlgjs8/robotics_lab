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

JointTargetSmdConfig taperConfig(double arrival_decel) {
    JointTargetSmdConfig cfg = defaultSmdConfig();
    cfg.arrival_taper_enable = true;
    cfg.arrival_decel_deg_s2 = arrival_decel;
    cfg.arrival_min_speed_deg_s = 3.0;
    return cfg;
}

// The arrival taper must decouple the final deceleration from the cruise: the peak
// braking near the stop is far gentler (~arrival_decel) than the SMD's natural
// velocity-clamped braking, while the peak (cruise) velocity is untouched and the
// tracker still settles exactly on the stop.
bool testArrivalTaperGentlerButPreservesCruiseAndSettles() {
    JointArray stop{};
    stop[0] = 120.0;  // large single-joint move -> cruises at the velocity clamp
    const auto run = [&](const JointTargetSmdConfig& cfg, bool with_stop,
                         double& max_vel, double& max_decel, JointArray& last) {
        JointSmdTracker smd(cfg);
        smd.reset(JointArray{}, JointArray{});
        smd.setGoal(stop);
        if (with_stop) smd.setArrivalStop(stop);
        double prev_dq = 0.0;
        JointArray prev{};
        max_vel = 0.0;
        max_decel = 0.0;
        for (int t = 0; t < 10000; ++t) {
            const JointArray q = smd.step(kDt);
            const double dq = (q[0] - prev[0]) / kDt;
            const double ddq = (dq - prev_dq) / kDt;
            max_vel = std::max(max_vel, std::abs(dq));
            max_decel = std::max(max_decel, -ddq);  // most-negative accel = hardest braking
            prev_dq = dq;
            prev = q;
        }
        last = prev;
    };
    double vel_base = 0.0, decel_base = 0.0;
    JointArray last_base{};
    run(defaultSmdConfig(), false, vel_base, decel_base, last_base);
    // The SMD's natural braking peak is ~0.368*wn*v0 (here ~27.7 deg/s^2 at fn=0.4,
    // v0=30), NOT the accel clamp. Pick an arrival_decel well below it so the taper is
    // demonstrably gentler; the taper region then brakes at a constant ~arrival_decel.
    const double arrival_decel = 12.0;
    double vel_taper = 0.0, decel_taper = 0.0;
    JointArray last_taper{};
    run(taperConfig(arrival_decel), true, vel_taper, decel_taper, last_taper);

    RB_CHECK(std::abs(vel_taper - vel_base) < 1e-6);       // start/cruise unchanged
    RB_CHECK(decel_taper < 0.6 * decel_base);              // arrival meaningfully gentler
    RB_CHECK(decel_taper < arrival_decel + 8.0);           // braking bounded near arrival_decel
    RB_CHECK(std::abs(last_taper[0] - stop[0]) < 0.05);    // still settles on the stop
    return true;
}

// Uniform velocity scaling means a multi-joint tapered move keeps its straight
// joint-space line all the way into the stop (same invariant as the unclamped SMD).
bool testArrivalTaperPreservesStraightLine() {
    JointTargetSmdConfig cfg = taperConfig(80.0);
    cfg.max_velocity_deg_s = {1e6, 1e6, 1e6, 1e6, 1e6, 1e6};
    cfg.max_accel_deg_s2 = {1e9, 1e9, 1e9, 1e9, 1e9, 1e9};
    JointSmdTracker smd(cfg);
    smd.reset(JointArray{}, JointArray{});
    JointArray goal{10.0, -20.0, 40.0, -80.0, 5.0, -2.5};
    smd.setGoal(goal);
    smd.setArrivalStop(goal);
    for (int tick = 0; tick < 2000; ++tick) {
        const JointArray q = smd.step(kDt);
        const double f0 = q[0] / goal[0];
        for (int i = 1; i < kDof; ++i) {
            RB_CHECK(std::abs(q[i] / goal[i] - f0) < 1e-9);
        }
    }
    return true;
}

// The taper is inert unless BOTH the config enables it AND a stop is latched: taper
// enabled with no stop, and a stop with taper disabled, must both match the plain SMD.
bool testArrivalTaperInertWhenNoStopOrDisabled() {
    JointArray goal{};
    goal[0] = 120.0;
    JointArray ref{};
    {
        JointSmdTracker smd(defaultSmdConfig());
        smd.reset(JointArray{}, JointArray{});
        smd.setGoal(goal);
        for (int t = 0; t < 3000; ++t) ref = smd.step(kDt);
    }
    {  // taper enabled, but no arrival stop latched
        JointSmdTracker smd(taperConfig(50.0));
        smd.reset(JointArray{}, JointArray{});
        smd.setGoal(goal);
        JointArray q{};
        for (int t = 0; t < 3000; ++t) q = smd.step(kDt);
        for (int i = 0; i < kDof; ++i) RB_CHECK(std::abs(q[i] - ref[i]) < 1e-9);
    }
    {  // stop latched, but taper disabled in config
        JointSmdTracker smd(defaultSmdConfig());
        smd.reset(JointArray{}, JointArray{});
        smd.setGoal(goal);
        smd.setArrivalStop(goal);
        JointArray q{};
        for (int t = 0; t < 3000; ++t) q = smd.step(kDt);
        for (int i = 0; i < kDof; ++i) RB_CHECK(std::abs(q[i] - ref[i]) < 1e-9);
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

// As above, plus the supported rbpodo raw range [-360, 360] so the shortest-path
// JointTarget goal selection is active (it is a no-op when q_min/q_max are unset).
SafetyConfig safetyConfigWithRange() {
    SafetyConfig safety = safetyConfigWithSmd(false);  // legacy ramp path, run to settle
    safety.q_min_deg = {-360.0, -360.0, -360.0, -360.0, -360.0, -360.0};
    safety.q_max_deg = {360.0, 360.0, 360.0, 360.0, 360.0, 360.0};
    return safety;
}

// Ramp the legacy (smd-disabled) filter to convergence and return the resting target.
JointArray runJointTargetToSettle(const JointArray& prev, const JointArray& goal, const SafetyConfig& safety) {
    TrajectoryFilter filter(ServoConfig{}, safety);
    JointArray q = prev;
    for (int tick = 0; tick < 20000; ++tick) {  // 40 s @ 500 Hz: plenty for a <360 deg ramp
        q = filter.computeJointTarget(jointTargetCommand(goal), RobotState{}, q, kDt);
    }
    return q;
}

// J1 at 251.8 deg, target -131.66 deg: same physical pose as +228.34 deg (= -131.66 + 360).
// The shortest in-range path is the +228.34 representation (~23 deg), NOT -131.66 (~383 deg
// the long way). The robot must settle at 228.34 and never swing past it toward -131.
bool testJointTargetTakesShortestInRangePath() {
    const JointArray prev{251.8, 79.7, 128.5, -65.9, -129.9, -167.4};
    const JointArray goal{-131.663, 72.989, 113.400, -80.880, -107.064, -145.949};
    const JointArray out = runJointTargetToSettle(prev, goal, safetyConfigWithRange());
    RB_CHECK(std::abs(out[0] - (goal[0] + 360.0)) < 0.5);   // settled at +228.34, the short way
    RB_CHECK(out[0] > 200.0);                                // never went the long way toward -131
    for (int i = 1; i < kDof; ++i) {                         // other joints (|delta|<180) untouched
        RB_CHECK(std::abs(out[i] - goal[i]) < 0.5);
    }
    return true;
}

// Literal axes keep the commanded raw target instead of choosing the nearest
// +/-360 equivalent. This is used for wrist yaw cable safety: J6 must return to
// the configured raw InitMotion target even when an equivalent pose is closer.
bool testJointTargetLiteralAxisKeepsRawTarget() {
    SafetyConfig safety = safetyConfigWithRange();
    safety.joint_target_literal_axes = {false, false, false, false, false, true};
    JointArray prev{};
    prev[0] = 251.8;
    prev[5] = 251.8;
    JointArray goal{};
    goal[0] = -131.663;
    goal[5] = -131.663;

    const JointArray out = runJointTargetToSettle(prev, goal, safety);

    RB_CHECK(std::abs(out[0] - (goal[0] + 360.0)) < 0.5);  // J1 still shortest-path
    RB_CHECK(std::abs(out[5] - goal[5]) < 0.5);            // J6 stays literal
    return true;
}

// A near-limit target has no closer in-range equivalent (10 + 360 = 370 > q_max 360),
// so it must KEEP the literal target and take the long way — limits win over shortness.
bool testJointTargetNearLimitKeepsLiteralTarget() {
    JointArray prev{}; prev[0] = 350.0;
    JointArray goal{}; goal[0] = 10.0;  // 370 deg out of range -> stays 10
    const JointArray out = runJointTargetToSettle(prev, goal, safetyConfigWithRange());
    RB_CHECK(std::abs(out[0] - 10.0) < 0.5);
    return true;
}

// Within a revolution (|target - current| < 180): no wrap, settle exactly on the target.
bool testJointTargetNoSpuriousWrap() {
    JointArray prev{}; prev[0] = 10.0;
    JointArray goal{}; goal[0] = 20.0;
    const JointArray out = runJointTargetToSettle(prev, goal, safetyConfigWithRange());
    RB_CHECK(std::abs(out[0] - 20.0) < 0.5);
    return true;
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

// ---------------------------------------------------------------------------------
// DEPARTURE TAPER (max_jerk_deg_s3), 2026-09-10.
//
// Regression target: without it the tracker's ACCELERATION is a step. A resting arm
// handed a goal L degrees away commands ddq = wn^2*L on its very first tick — for the
// shipped real profile (fn 2.25 Hz, pursuit lookahead 6 deg) 1199 deg/s^2 reached in one
// 2 ms tick, a jerk of 600,000 deg/s^3. Measured on all 12 InitMotions of 2026-09-10
// (e.g. servo_log_20260910_134846.csv tick 53690: command velocity 0 -> 2.40 deg/s in one
// tick); larger than the peak command acceleration of a whole 50 s policy run.
//
// The taper shapes the INPUT (slews the goal), never the output. These tests pin both
// halves of that choice: the step is gone, AND the filter's own guarantees — no
// overshoot, straight-line joint path, unchanged cruise speed — all survive.
// ---------------------------------------------------------------------------------

// The shipped real joint_target_smd profile, so the numbers below are the ones the
// hardware actually sees.
JointTargetSmdConfig realProfile() {
    JointTargetSmdConfig cfg;
    cfg.enable = true;
    cfg.damping_ratio = 1.0;
    cfg.natural_frequency_hz = 2.25;
    cfg.max_velocity_deg_s = JointArray{60, 60, 60, 80, 80, 100};
    cfg.max_accel_deg_s2 = JointArray{1300, 1300, 1300, 1900, 1900, 2400};
    cfg.arrival_taper_enable = true;
    cfg.arrival_decel_deg_s2 = 40.0;
    cfg.arrival_min_speed_deg_s = 3.0;
    return cfg;
}
constexpr double kShippedJerk = 12000.0;   // stack_real.yaml
constexpr double kLookahead = 6.0;         // safety.init_motion_planner.execution_lookahead_deg

struct MoveProfile {
    double first_tick_accel = 0.0;
    double peak_accel = 0.0;
    double peak_jerk = 0.0;
    double cruise = 0.0;
    double overshoot = 0.0;
    int settle_tick = -1;
};

// Drives joint 0 the way applyInitMotionSequencer does: the pursuit carrot is kept
// `kLookahead` ahead of the filter output, the final stop is latched for the arrival
// taper, and the move runs to `distance`.
MoveProfile pursue(double max_jerk_deg_s3, double distance, int ticks = 3000) {
    JointTargetSmdConfig cfg = realProfile();
    cfg.max_jerk_deg_s3 = max_jerk_deg_s3;
    JointSmdTracker smd(cfg);
    smd.reset(JointArray{}, JointArray{});
    JointArray stop{};
    stop[0] = distance;
    smd.setArrivalStop(stop);
    MoveProfile out;
    double prev_q = 0.0, prev_v = 0.0, prev_a = 0.0;
    for (int t = 0; t < ticks; ++t) {
        JointArray carrot{};
        carrot[0] = std::min(prev_q + kLookahead, distance);
        smd.setGoal(carrot);
        const JointArray q = smd.step(kDt);
        const double v = (q[0] - prev_q) / kDt;
        const double a = (v - prev_v) / kDt;
        const double j = (a - prev_a) / kDt;
        if (t == 0) out.first_tick_accel = std::abs(a);
        out.peak_accel = std::max(out.peak_accel, std::abs(a));
        out.peak_jerk = std::max(out.peak_jerk, std::abs(j));
        out.cruise = std::max(out.cruise, v);
        out.overshoot = std::max(out.overshoot, q[0] - distance);
        if (out.settle_tick < 0 && std::abs(q[0] - distance) < 0.05) out.settle_tick = t;
        prev_q = q[0]; prev_v = v; prev_a = a;
    }
    return out;
}

bool testDepartureTaperRemovesTheStartStep() {
    const MoveProfile before = pursue(0.0, 40.0);
    const MoveProfile after = pursue(kShippedJerk, 40.0);

    // Baseline reproduces the measured hardware step: wn^2 * L = 199.85 * 6.0.
    RB_CHECK(std::abs(before.first_tick_accel - 1199.1) < 5.0);
    RB_CHECK(before.peak_jerk > 500000.0);

    // The step is gone. The goal slew is max_jerk/wn^2 per second, so the first tick's
    // acceleration demand is wn^2 * (slew * dt) = max_jerk * dt.
    RB_CHECK(std::abs(after.first_tick_accel - kShippedJerk * kDt) < 1.0);   // 24 deg/s^2
    RB_CHECK(after.first_tick_accel < before.first_tick_accel / 40.0);
    RB_CHECK(after.peak_jerk < before.peak_jerk / 40.0);

    // THE POINT: the move is not slowed down. Cruise is the profile's equilibrium speed
    // wn*L/(2*zeta) and must survive the taper; only the ramp-in costs time.
    RB_CHECK(after.cruise > before.cruise * 0.999);
    RB_CHECK(after.settle_tick > 0);
    RB_CHECK(after.settle_tick < before.settle_tick + 60);   // < +120 ms on a 1.56 s move
    return true;
}

// Why the taper shapes the input: a slew on the OUTPUT acceleration is phase lag inside
// the loop and destroys the critical damping this filter exists to provide. Measured
// while developing this, an output slew overshot by 1.53 deg on an 8.6 deg step.
bool testDepartureTaperNeverOvershoots() {
    // With an arrival stop (InitMotion) and without (plain PTP / waypoint replay, which
    // never latches one), across the range where an output slew was worst.
    for (double d = 0.5; d <= 60.0; d += 0.5) {
        RB_CHECK(pursue(kShippedJerk, d).overshoot <= 1e-9);

        JointTargetSmdConfig cfg = realProfile();
        cfg.max_jerk_deg_s3 = kShippedJerk;
        JointSmdTracker smd(cfg);
        smd.reset(JointArray{}, JointArray{});
        JointArray goal{};
        goal[0] = d;
        smd.setGoal(goal);   // fixed goal, NO arrival stop
        double worst = 0.0;
        for (int t = 0; t < 4000; ++t) worst = std::max(worst, smd.step(kDt)[0] - d);
        RB_CHECK(worst <= 1e-9);
    }
    return true;
}

// The slew is applied uniformly across joints, so a synchronized PTP still traces a
// straight line in joint space — the same rule the arrival taper follows. Travel is kept
// small enough that no per-joint velocity/accel clamp bites, since a saturating clamp
// bends the path on its own (that is what testStraightLinePathWhenUnclamped covers, and
// it is unchanged by the taper).
bool testDepartureTaperPreservesStraightLine() {
    JointTargetSmdConfig cfg = realProfile();
    cfg.max_jerk_deg_s3 = kShippedJerk;
    JointSmdTracker smd(cfg);
    smd.reset(JointArray{}, JointArray{});
    const JointArray goal{2.0, -5.0, 8.0, 1.0, -3.0, 4.0};
    smd.setGoal(goal);
    for (int t = 0; t < 3000; ++t) {
        const JointArray q = smd.step(kDt);
        if (std::abs(q[2]) < 1e-9) continue;   // before motion starts
        // Every joint keeps the same fraction of its own travel.
        const double reference = q[2] / goal[2];
        for (int i = 0; i < kDof; ++i) {
            RB_CHECK(std::abs(q[i] / goal[i] - reference) < 1e-9);
        }
    }
    return true;
}

// A handover seeds a VELOCITY (brake-before-plan, jog -> InitMotion). reset() must also
// restart the effective goal at the seed pose, or the first tick after a reseed steps.
bool testDepartureTaperResetsTheEffectiveGoal() {
    JointTargetSmdConfig cfg = realProfile();
    cfg.max_jerk_deg_s3 = kShippedJerk;
    JointSmdTracker smd(cfg);
    smd.reset(JointArray{}, JointArray{});
    JointArray far{};
    far[0] = 40.0;
    smd.setGoal(far);
    for (int t = 0; t < 200; ++t) smd.step(kDt);   // effective goal has marched out

    JointArray at_rest{};
    smd.reset(at_rest, JointArray{});
    JointArray carrot{};
    carrot[0] = kLookahead;
    smd.setGoal(carrot);
    const JointArray q1 = smd.step(kDt);
    const double first_accel = std::abs(q1[0] / (kDt * kDt));
    RB_CHECK(first_accel <= kShippedJerk * kDt + 1.0);
    return true;
}

// A continuity reseed must be invisible to the departure taper. TrajectoryFilter
// re-seeds whenever the SMD output and prev_sent differ by more than 0.02 deg — measured
// at 122 of 673 ticks during an InitMotion (servo_log 2026-09-10). If each reseed
// collapsed the effective goal onto the position (what reset() does, correctly, for a
// FRESH activation), the taper would never build its lead and the move would crawl:
// measured 42.4 -> 3.0 deg/s of cruise at that reset rate.
bool testDepartureTaperSurvivesContinuityReseeds() {
    const double unreseeded = pursue(kShippedJerk, 200.0, 3000).cruise;
    for (int period : {100, 20, 6, 3, 1}) {
        JointTargetSmdConfig cfg = realProfile();
        cfg.max_jerk_deg_s3 = kShippedJerk;
        JointSmdTracker smd(cfg);
        smd.reset(JointArray{}, JointArray{});
        JointArray stop{};
        stop[0] = 200.0;
        smd.setArrivalStop(stop);
        double prev_q = 0.0, prev_v = 0.0, cruise = 0.0;
        for (int t = 0; t < 3000; ++t) {
            if (t > 0 && t % period == 0) {
                JointArray at{}, dq{};
                at[0] = prev_q;
                dq[0] = prev_v;
                smd.reseed(at, dq);
            }
            JointArray carrot{};
            carrot[0] = std::min(prev_q + kLookahead, 200.0);
            smd.setGoal(carrot);
            const JointArray q = smd.step(kDt);
            const double v = (q[0] - prev_q) / kDt;
            cruise = std::max(cruise, v);
            prev_q = q[0]; prev_v = v;
        }
        RB_CHECK(std::abs(cruise - unreseeded) < 0.01);
    }

    // reset() keeps its own contract: a FRESH activation still latches the goal at the
    // seed pose, so a genuinely new move ramps from zero instead of inheriting a lead.
    JointTargetSmdConfig cfg = realProfile();
    cfg.max_jerk_deg_s3 = kShippedJerk;
    JointSmdTracker smd(cfg);
    smd.reset(JointArray{}, JointArray{});
    JointArray far{};
    far[0] = 60.0;
    smd.setGoal(far);
    for (int t = 0; t < 300; ++t) smd.step(kDt);
    JointArray elsewhere{};
    elsewhere[0] = 100.0;
    smd.reset(elsewhere, JointArray{});
    JointArray carrot{};
    carrot[0] = 100.0 + kLookahead;
    smd.setGoal(carrot);
    const JointArray q1 = smd.step(kDt);
    RB_CHECK(std::abs((q1[0] - 100.0) / (kDt * kDt)) <= kShippedJerk * kDt + 1.0);
    return true;
}

// Disabled (0) must be bit-for-bit the pre-2026-09-10 behavior.
bool testDepartureTaperDisabledIsUnchanged() {
    JointTargetSmdConfig legacy = realProfile();
    RB_CHECK(legacy.max_jerk_deg_s3 == 0.0);   // the field defaults to off
    JointTargetSmdConfig off = realProfile();
    off.max_jerk_deg_s3 = 0.0;
    JointSmdTracker a(off), b(legacy);
    a.reset(JointArray{}, JointArray{});
    b.reset(JointArray{}, JointArray{});
    JointArray goal{};
    goal[0] = 25.0;
    a.setGoal(goal);
    b.setGoal(goal);
    for (int t = 0; t < 600; ++t) {
        const JointArray qa = a.step(kDt);
        const JointArray qb = b.step(kDt);
        for (int i = 0; i < kDof; ++i) RB_CHECK(qa[i] == qb[i]);
    }
    return true;
}

}  // namespace

int main() {
    if (!testStepResponseNoOvershootAndSettles()) return 1;
    if (!testVelocityAndAccelClamps()) return 1;
    if (!testResetClampsInitialVelocity()) return 1;
    if (!testStraightLinePathWhenUnclamped()) return 1;
    if (!testArrivalTaperGentlerButPreservesCruiseAndSettles()) return 1;
    if (!testArrivalTaperPreservesStraightLine()) return 1;
    if (!testArrivalTaperInertWhenNoStopOrDisabled()) return 1;
    if (!testDepartureTaperRemovesTheStartStep()) return 1;
    if (!testDepartureTaperNeverOvershoots()) return 1;
    if (!testDepartureTaperPreservesStraightLine()) return 1;
    if (!testDepartureTaperResetsTheEffectiveGoal()) return 1;
    if (!testDepartureTaperSurvivesContinuityReseeds()) return 1;
    if (!testDepartureTaperDisabledIsUnchanged()) return 1;
    if (!testTrajectoryFilterDisabledKeepsLegacyRamp()) return 1;
    if (!testTrajectoryFilterSmdProfile()) return 1;
    if (!testTrajectoryFilterRebaselinesAfterExternalMove()) return 1;
    if (!testJointTargetTakesShortestInRangePath()) return 1;
    if (!testJointTargetLiteralAxisKeepsRawTarget()) return 1;
    if (!testJointTargetNearLimitKeepsLiteralTarget()) return 1;
    if (!testJointTargetNoSpuriousWrap()) return 1;
    std::cout << "joint_smd_tracker tests passed\n";
    return 0;
}
