#include "rb_servo/control/smd_pose_tracker.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

#define RB_CHECK(cond)                                                       \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::cerr << "CHECK failed at " << __FILE__ << ":" << __LINE__  \
                      << ": " #cond << "\n";                                 \
            return false;                                                    \
        }                                                                    \
    } while (false)

constexpr double kDt = 0.002;  // 500 Hz servo tick

rb_servo::PoseTrackSmdConfig defaultConfig() {
    rb_servo::PoseTrackSmdConfig cfg;
    cfg.enable = true;
    cfg.damping_ratio_linear = 1.0;
    cfg.natural_frequency_linear_hz = 0.5;
    cfg.damping_ratio_angular = 1.0;
    cfg.natural_frequency_angular_hz = 0.5;
    return cfg;
}

bool testCriticallyDampedStepHasNoOvershootAndConverges() {
    rb_servo::SmdPoseTracker tracker(defaultConfig());
    tracker.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    // First command latches the reference; the second carries the step delta.
    tracker.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.1, 0.0, 0.0, 0.0, 0.0, 0.0});

    double previous_x = 0.0;
    double x = 0.0;
    for (int i = 0; i < 5000; ++i) {  // 10 s >> settling time for fn=0.5 Hz
        const rb_servo::Pose6D pose = tracker.step(kDt);
        x = pose.x;
        RB_CHECK(x <= 0.1 + 1e-9);          // critical damping: no overshoot
        RB_CHECK(x >= previous_x - 1e-12);  // monotonic approach
        previous_x = x;
    }
    RB_CHECK(std::abs(x - 0.1) < 1e-4);  // converged to the integrated goal

    // Analytic critically damped step response: x(T)/A = 1-(1+wn T)e^(-wn T).
    rb_servo::SmdPoseTracker timing(defaultConfig());
    timing.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    timing.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    timing.updateGoalFromCommand({0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
    const double wn = 2.0 * M_PI * 0.5;
    const double T = 1.0;  // seconds
    double xt = 0.0;
    for (int i = 0; i < static_cast<int>(T / kDt); ++i) xt = timing.step(kDt).x;
    const double expected = 0.1 * (1.0 - (1.0 + wn * T) * std::exp(-wn * T));
    RB_CHECK(std::abs(xt - expected) < 0.002);
    return true;
}

bool testFirstCommandLatchesWithoutJump() {
    rb_servo::SmdPoseTracker tracker(defaultConfig());
    tracker.reset({0.5, 0.2, 0.3, 0.0, 0.0, 0.0});
    // An engagement offset between the commanded pose and the current state
    // must NOT pull the output: only deltas after the first command count.
    tracker.updateGoalFromCommand({0.9, 0.9, 0.9, 0.3, 0.3, 0.3});
    for (int i = 0; i < 1000; ++i) tracker.step(kDt);
    const rb_servo::Pose6D pose = tracker.step(kDt);
    RB_CHECK(std::abs(pose.x - 0.5) < 1e-9);
    RB_CHECK(std::abs(pose.y - 0.2) < 1e-9);
    RB_CHECK(std::abs(pose.z - 0.3) < 1e-9);
    // A subsequent delta moves the goal relative to the anchor.
    tracker.updateGoalFromCommand({0.92, 0.9, 0.9, 0.3, 0.3, 0.3});
    const rb_servo::Pose6D goal = tracker.goalPose();
    RB_CHECK(std::abs(goal.x - 0.52) < 1e-9);
    RB_CHECK(std::abs(goal.y - 0.2) < 1e-9);
    return true;
}

bool testDriftDetectionForExternalMove() {
    rb_servo::SmdPoseTracker tracker(defaultConfig());
    tracker.reset({0.5, 0.2, 0.3, 0.0, 0.0, 0.0});
    // Track a small streamed delta; the held pose stays near the anchor.
    tracker.updateGoalFromCommand({0.5, 0.2, 0.3, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.52, 0.2, 0.3, 0.0, 0.0, 0.0});
    for (int i = 0; i < 200; ++i) tracker.step(kDt);
    // The live reference still matches the held pose (no external move) -> no drift.
    RB_CHECK(!tracker.driftedFrom(tracker.currentPose(), 0.05, 0.10));
    RB_CHECK(!tracker.driftedFrom({0.52, 0.2, 0.3, 0.0, 0.0, 0.0}, 0.05, 0.10));
    // An external control path (e.g. a JointTarget init-return) moved the robot far away:
    // the live reference now diverges from the tracker's held pose -> drift is detected.
    RB_CHECK(tracker.driftedFrom({0.9, -0.3, 0.5, 0.0, 0.0, 0.0}, 0.05, 0.10));
    // A pure orientation move beyond the angular tolerance also trips.
    RB_CHECK(tracker.driftedFrom({0.52, 0.2, 0.3, 0.0, 0.0, 0.6}, 0.05, 0.10));
    // After re-anchoring at the live reference, drift clears and there is no jump.
    tracker.reset({0.9, -0.3, 0.5, 0.0, 0.0, 0.0});
    RB_CHECK(!tracker.driftedFrom({0.9, -0.3, 0.5, 0.0, 0.0, 0.0}, 0.05, 0.10));
    const rb_servo::Pose6D p = tracker.currentPose();
    RB_CHECK(std::abs(p.x - 0.9) < 1e-9 && std::abs(p.y + 0.3) < 1e-9 && std::abs(p.z - 0.5) < 1e-9);
    return true;
}

bool testRotationConvergesAndAxesAreIndependent() {
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.natural_frequency_linear_hz = 2.0;   // fast translation
    cfg.natural_frequency_angular_hz = 0.5;  // slow rotation
    rb_servo::SmdPoseTracker tracker(cfg);
    tracker.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.05, 0.0, 0.0, 0.0, 0.0, 0.8});

    // After 0.6 s the fast translation should be nearly settled while the
    // slow rotation is still clearly in transit (independent tuning).
    rb_servo::Pose6D pose{};
    for (int i = 0; i < 300; ++i) pose = tracker.step(kDt);
    RB_CHECK(std::abs(pose.x - 0.05) < 0.002);
    RB_CHECK(pose.rz > 0.05 && pose.rz < 0.75);

    for (int i = 0; i < 5000; ++i) pose = tracker.step(kDt);
    RB_CHECK(std::abs(pose.rz - 0.8) < 1e-3);
    RB_CHECK(std::abs(pose.rx) < 1e-6 && std::abs(pose.ry) < 1e-6);
    return true;
}

bool testForceAndVelocityClampsSaturateAndConverge() {
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.natural_frequency_linear_hz = 1.0;
    cfg.max_linear_velocity_m_s = 0.05;   // 50 mm/s
    cfg.max_linear_accel_m_s2 = 0.1;      // 100 mm/s^2 == max force with M=1
    cfg.max_angular_velocity_rad_s = 0.872664626;  // 50 deg/s
    cfg.max_angular_accel_rad_s2 = 1.745329252;    // 100 deg/s^2
    rb_servo::SmdPoseTracker tracker(cfg);
    tracker.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.5, 0.0, 0.0, 0.0, 0.0, 1.5});

    double prev_x = 0.0;
    double prev_v = 0.0;
    double prev_rz = 0.0;
    double max_x = 0.0;
    for (int i = 0; i < 15000; ++i) {  // 30 s
        const rb_servo::Pose6D pose = tracker.step(kDt);
        const double v = (pose.x - prev_x) / kDt;
        const double w = (pose.rz - prev_rz) / kDt;
        RB_CHECK(std::abs(v) <= 0.05 + 1e-9);            // velocity clamp
        RB_CHECK(std::abs(v - prev_v) / kDt <= 0.1 + 1e-6);  // force clamp
        RB_CHECK(std::abs(w) <= 0.872664626 + 1e-9);     // angular velocity clamp
        prev_x = pose.x;
        prev_v = v;
        prev_rz = pose.rz;
        max_x = std::max(max_x, pose.x);
    }
    RB_CHECK(std::abs(prev_x - 0.5) < 1e-3);   // converged
    // Force-limited braking from v_max glides at most v^2/(2a) = 12.5 mm.
    RB_CHECK(max_x <= 0.5 + 0.0130);
    RB_CHECK(std::abs(prev_rz - 1.5) < 1e-3);
    return true;
}

bool testClampsInactiveForSmallMotionsPreserveDynamics() {
    rb_servo::PoseTrackSmdConfig limited = defaultConfig();
    limited.natural_frequency_linear_hz = 1.0;
    limited.max_linear_velocity_m_s = 0.05;
    limited.max_linear_accel_m_s2 = 0.1;
    rb_servo::PoseTrackSmdConfig unlimited = limited;
    unlimited.max_linear_velocity_m_s = 0.0;
    unlimited.max_linear_accel_m_s2 = 0.0;

    rb_servo::SmdPoseTracker a(limited);
    rb_servo::SmdPoseTracker b(unlimited);
    for (rb_servo::SmdPoseTracker* t : {&a, &b}) {
        t->reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        t->updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        // 2 mm step at fn=1 Hz: peak accel wn^2*A ~ 0.079 < 0.1 and peak
        // velocity well under 0.05 -> clamps never engage.
        t->updateGoalFromCommand({0.002, 0.0, 0.0, 0.0, 0.0, 0.0});
    }
    for (int i = 0; i < 3000; ++i) {
        const rb_servo::Pose6D pa = a.step(kDt);
        const rb_servo::Pose6D pb = b.step(kDt);
        RB_CHECK(std::abs(pa.x - pb.x) < 1e-15);  // zeta/fn dynamics untouched
    }
    return true;
}

// Drives a constant-velocity ramp goal and returns the steady-state position
// lag (goal.x - x) after the transient has settled. With feedforward the lag
// must collapse to ~0; without it the lag is the classic 2*zeta/wn * v.
double rampSteadyStateLag(bool feedforward, double v_m_s) {
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.natural_frequency_linear_hz = 1.0;
    cfg.velocity_feedforward = feedforward;
    rb_servo::SmdPoseTracker tracker(cfg);
    tracker.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});  // latch reference
    double cmd_x = 0.0;
    double x = 0.0;
    double goal_x = 0.0;
    for (int i = 0; i < 8000; ++i) {  // 16 s >> settling time for fn=1 Hz
        cmd_x += v_m_s * kDt;  // command advances at constant velocity
        tracker.updateGoalFromCommand({cmd_x, 0.0, 0.0, 0.0, 0.0, 0.0});
        x = tracker.step(kDt).x;
        goal_x = tracker.goalPose().x;
    }
    return goal_x - x;  // steady-state position lag along the ramp
}

bool testVelocityFeedforwardZeroesRampLag() {
    const double v = 0.05;  // 50 mm/s constant-velocity goal
    const double wn = 2.0 * M_PI * 1.0;
    const double analytic_lag = 2.0 * 1.0 * v / wn;  // 2*zeta/wn * v, zeta=1

    const double lag_off = rampSteadyStateLag(/*feedforward=*/false, v);
    const double lag_on = rampSteadyStateLag(/*feedforward=*/true, v);

    // Legacy SMD lags a ramp by the analytic 2*zeta/wn*v (~15.9 mm here).
    RB_CHECK(std::abs(lag_off - analytic_lag) < 0.1 * analytic_lag);
    // Feedforward collapses it to ~0 (discretization floor only).
    RB_CHECK(std::abs(lag_on) < 1e-4);
    // And it is a real, large improvement, not a wash.
    RB_CHECK(std::abs(lag_on) < 0.02 * std::abs(lag_off));
    return true;
}

// Patch 5: command-twist feedforward source. A constant-velocity command twist
// should zero the ramp lag just like finite-difference feedforward.
double rampLagWithCommandTwist(const std::string& source, double v_m_s, bool supply_twist) {
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.natural_frequency_linear_hz = 1.0;
    cfg.velocity_feedforward = true;
    cfg.velocity_feedforward_source = source;
    rb_servo::SmdPoseTracker tracker(cfg);
    tracker.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    double cmd_x = 0.0;
    double x = 0.0;
    double goal_x = 0.0;
    for (int i = 0; i < 8000; ++i) {
        cmd_x += v_m_s * kDt;
        tracker.updateGoalFromCommand({cmd_x, 0.0, 0.0, 0.0, 0.0, 0.0});
        if (supply_twist) {
            tracker.setCommandTwist(Eigen::Vector3d(v_m_s, 0.0, 0.0), Eigen::Vector3d::Zero());
        }
        x = tracker.step(kDt).x;
        goal_x = tracker.goalPose().x;
    }
    return goal_x - x;
}

bool testCommandTwistFeedforwardZeroesRampLag() {
    const double v = 0.05;
    const double lag = rampLagWithCommandTwist("command_twist", v, /*supply_twist=*/true);
    RB_CHECK(std::abs(lag) < 1e-4);  // command twist feedforward zeroes the lag
    // The diagnostics report command_twist as the effective source, no fallback.
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.velocity_feedforward = true;
    cfg.velocity_feedforward_source = "command_twist";
    rb_servo::SmdPoseTracker tracker(cfg);
    tracker.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.01, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.setCommandTwist(Eigen::Vector3d(0.05, 0.0, 0.0), Eigen::Vector3d::Zero());
    tracker.step(kDt);
    RB_CHECK(tracker.lastStepInfo().velocity_feedforward_source == "command_twist");
    RB_CHECK(!tracker.lastStepInfo().command_twist_fallback);
    RB_CHECK(std::abs(tracker.lastStepInfo().goal_linear_velocity.x() - 0.05) < 1e-9);
    return true;
}

bool testCommandTwistMatchesFiniteDifferenceOnRamp() {
    // Fix 3: a correct body/stand-frame command twist must drive the SMD to the same
    // steady state as finite-difference feedforward on a constant-velocity ramp.
    const double v = 0.05;
    const double lag_fd = rampSteadyStateLag(/*feedforward=*/true, v);  // finite_difference
    const double lag_cmd = rampLagWithCommandTwist("command_twist", v, /*supply_twist=*/true);
    RB_CHECK(std::abs(lag_fd) < 1e-4);
    RB_CHECK(std::abs(lag_cmd) < 1e-4);
    RB_CHECK(std::abs(lag_cmd - lag_fd) < 1e-4);  // equivalent steady state
    return true;
}

bool testCommandTwistMissingFallsBackToFiniteDifference() {
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.velocity_feedforward = true;
    cfg.velocity_feedforward_source = "command_twist";
    rb_servo::SmdPoseTracker tracker(cfg);
    tracker.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.01, 0.0, 0.0, 0.0, 0.0, 0.0});
    // No setCommandTwist() -> must fall back, flagged, and not crash.
    tracker.step(kDt);
    RB_CHECK(tracker.lastStepInfo().velocity_feedforward_source == "finite_difference");
    RB_CHECK(tracker.lastStepInfo().command_twist_fallback);

    // Non-finite twist is ignored (also a safe fallback).
    rb_servo::SmdPoseTracker tracker2(cfg);
    tracker2.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker2.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker2.updateGoalFromCommand({0.01, 0.0, 0.0, 0.0, 0.0, 0.0});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    tracker2.setCommandTwist(Eigen::Vector3d(nan, 0.0, 0.0), Eigen::Vector3d::Zero());
    const rb_servo::Pose6D out = tracker2.step(kDt);
    RB_CHECK(std::isfinite(out.x));
    RB_CHECK(tracker2.lastStepInfo().command_twist_fallback);
    return true;
}

bool testStepInfoClipFlagsAndReanchorCount() {
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.natural_frequency_linear_hz = 1.0;
    cfg.max_linear_velocity_m_s = 0.05;  // tight velocity cap to force velocity clipping
    cfg.max_linear_accel_m_s2 = 1.0;     // accel cap also clips on the big initial step
    rb_servo::SmdPoseTracker tracker(cfg);
    tracker.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({1.0, 0.0, 0.0, 0.0, 0.0, 0.0});  // big step -> saturate
    bool saw_accel_clip = false;
    bool saw_vel_clip = false;
    for (int i = 0; i < 200; ++i) {
        tracker.step(kDt);
        saw_accel_clip = saw_accel_clip || tracker.lastStepInfo().linear_accel_clipped;
        saw_vel_clip = saw_vel_clip || tracker.lastStepInfo().linear_velocity_clipped;
    }
    RB_CHECK(saw_accel_clip);
    RB_CHECK(saw_vel_clip);

    // reanchorCount counts reset() only while already active (not the first reset).
    rb_servo::SmdPoseTracker t2(defaultConfig());
    RB_CHECK(t2.reanchorCount() == 0);
    t2.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});  // initial activation, not a re-anchor
    RB_CHECK(t2.reanchorCount() == 0);
    t2.reset({0.1, 0.0, 0.0, 0.0, 0.0, 0.0});  // re-anchor while active
    RB_CHECK(t2.reanchorCount() == 1);
    return true;
}

bool testDeactivateAndReanchor() {
    rb_servo::SmdPoseTracker tracker(defaultConfig());
    tracker.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
    for (int i = 0; i < 100; ++i) tracker.step(kDt);
    tracker.deactivate();
    RB_CHECK(!tracker.active());
    // Re-anchor at a new pose: state and goal restart there with zero velocity.
    tracker.reset({1.0, 1.0, 1.0, 0.0, 0.0, 0.0});
    RB_CHECK(tracker.active());
    const rb_servo::Pose6D pose = tracker.step(kDt);
    RB_CHECK(std::abs(pose.x - 1.0) < 1e-9);
    return true;
}

}  // namespace

int main() {
    if (!testCriticallyDampedStepHasNoOvershootAndConverges()) return 1;
    if (!testFirstCommandLatchesWithoutJump()) return 1;
    if (!testDriftDetectionForExternalMove()) return 1;
    if (!testRotationConvergesAndAxesAreIndependent()) return 1;
    if (!testForceAndVelocityClampsSaturateAndConverge()) return 1;
    if (!testClampsInactiveForSmallMotionsPreserveDynamics()) return 1;
    if (!testVelocityFeedforwardZeroesRampLag()) return 1;
    if (!testCommandTwistFeedforwardZeroesRampLag()) return 1;
    if (!testCommandTwistMatchesFiniteDifferenceOnRamp()) return 1;
    if (!testCommandTwistMissingFallsBackToFiniteDifference()) return 1;
    if (!testStepInfoClipFlagsAndReanchorCount()) return 1;
    if (!testDeactivateAndReanchor()) return 1;
    std::cout << "smd_pose_tracker tests passed\n";
    return 0;
}
