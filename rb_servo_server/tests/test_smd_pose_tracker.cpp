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

// RB_CHECK inside a lambda that returns a value: bail out with a sentinel the
// caller checks, since the macro's `return false` cannot be used there.
#define RB_CHECK_VALUE(cond, sentinel)                                       \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::cerr << "CHECK failed at " << __FILE__ << ":" << __LINE__  \
                      << ": " #cond << "\n";                                 \
            return (sentinel);                                               \
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

bool testReengageRelatchRejectsStaleJump() {
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.reengage_relatch_max_step_m = 0.012;  // 12 mm
    cfg.reengage_relatch_max_step_rad = 0.12;
    rb_servo::SmdPoseTracker tracker(cfg);
    tracker.reset({0.5, 0.2, 0.3, 0.0, 0.0, 0.0});
    tracker.updateGoalFromCommand({0.5, 0.2, 0.3, 0.0, 0.0, 0.0});    // latch reference
    tracker.updateGoalFromCommand({0.505, 0.2, 0.3, 0.0, 0.0, 0.0});  // 5 mm -> integrates
    RB_CHECK(std::abs(tracker.goalPose().x - 0.505) < 1e-9);

    // A 100 mm single delta is the re-engage stale jump: re-latch, goal must NOT move.
    const auto before = tracker.reanchorCount();
    tracker.updateGoalFromCommand({0.605, 0.2, 0.3, 0.0, 0.0, 0.0});
    RB_CHECK(std::abs(tracker.goalPose().x - 0.505) < 1e-9);  // did not jump to 0.605
    RB_CHECK(tracker.reanchorCount() == before + 1);
    // A subsequent small delta integrates from the re-latched reference (0.605).
    tracker.updateGoalFromCommand({0.610, 0.2, 0.3, 0.0, 0.0, 0.0});  // +5 mm
    RB_CHECK(std::abs(tracker.goalPose().x - 0.510) < 1e-9);

    // A large rotation delta also re-latches (orientation goal held).
    tracker.updateGoalFromCommand({0.610, 0.2, 0.3, 0.0, 0.0, 0.5});  // +0.5 rad > 0.12
    RB_CHECK(std::abs(tracker.goalPose().rz) < 1e-9);

    // Disabled by default (thresholds 0): the same big delta integrates (legacy path,
    // preserved for model-rollout / synthetic-step callers).
    rb_servo::SmdPoseTracker legacy(defaultConfig());
    legacy.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    legacy.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    legacy.updateGoalFromCommand({0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
    RB_CHECK(std::abs(legacy.goalPose().x - 0.1) < 1e-9);
    return true;
}

bool testSingularityVelocityScaling() {
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.natural_frequency_linear_hz = 1.0;
    cfg.max_linear_velocity_m_s = 0.10;
    cfg.singularity_scale_full_sigma = 0.10;
    cfg.singularity_scale_floor_sigma = 0.04;
    cfg.singularity_scale_min = 0.20;

    auto peak_velocity = [&](const rb_servo::PoseTrackSmdConfig& c, double sigma) {
        rb_servo::SmdPoseTracker t(c);
        t.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        t.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        t.updateGoalFromCommand({1.0, 0.0, 0.0, 0.0, 0.0, 0.0});  // big step -> saturate vmax
        t.setMinSingular(sigma);
        double vmax = 0.0, prev = 0.0;
        for (int i = 0; i < 800; ++i) {
            const double x = t.step(kDt).x;
            vmax = std::max(vmax, (x - prev) / kDt);
            prev = x;
        }
        return vmax;
    };

    const double v_full = peak_velocity(cfg, 0.20);  // sigma >= full -> scale 1 -> ~0.10
    const double v_sing = peak_velocity(cfg, 0.04);  // sigma <= floor -> scale 0.20 -> ~0.02
    const double v_mid = peak_velocity(cfg, 0.07);   // mid ramp -> scale 0.60 -> ~0.06
    RB_CHECK(v_full > 0.08);
    RB_CHECK(v_sing < 0.03);
    RB_CHECK(v_sing < 0.4 * v_full);          // clearly slowed near singular
    RB_CHECK(v_mid > v_sing && v_mid < v_full);  // monotone in sigma
    RB_CHECK(v_sing > 1e-4);                  // never frozen (scale_min > 0) -> can back out
    // sigma <= 0 means the IK did not compute the SVD (healthy pose) -> NO scaling, full
    // speed. This is the regression guard: feeding 0 must NOT throttle to scale_min.
    RB_CHECK(peak_velocity(cfg, 0.0) > 0.08);
    RB_CHECK(peak_velocity(cfg, -1.0) > 0.08);

    // Disabled by default (full_sigma 0): no scaling even at tiny sigma.
    rb_servo::PoseTrackSmdConfig off = defaultConfig();
    off.natural_frequency_linear_hz = 1.0;
    off.max_linear_velocity_m_s = 0.10;
    RB_CHECK(peak_velocity(off, 0.001) > 0.08);
    return true;
}

// Regression for the 2026-08-14 left-arm singularity lurch. An IK solve that fails
// (max_iterations) or converges before taking a DLS step reports sigma 0.0. That is the
// ABSENCE of a measurement, not a reading of "well conditioned", so it must not clear the
// last real sigma and hand the tracker its full velocity cap back. On hardware it did
// exactly that: the scale alternated 0.12 <-> 1.0 at 6-18 Hz across the singularity,
// on 38-43% of the ticks in the two event windows.
bool testUnmeasuredSigmaDoesNotReleaseTheSingularityGuard() {
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.natural_frequency_linear_hz = 1.0;
    cfg.max_linear_velocity_m_s = 0.10;
    cfg.singularity_scale_full_sigma = 0.10;
    cfg.singularity_scale_floor_sigma = 0.04;
    cfg.singularity_scale_min = 0.20;

    // Feed `first` once, then `rest` on every subsequent solve, and report peak speed.
    auto peak_velocity_after = [&](double first, double rest) {
        rb_servo::SmdPoseTracker t(cfg);
        t.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        t.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        t.updateGoalFromCommand({1.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        t.setMinSingular(first);
        double vmax = 0.0, prev = 0.0;
        for (int i = 0; i < 800; ++i) {
            t.setMinSingular(rest);
            const double x = t.step(kDt).x;
            vmax = std::max(vmax, (x - prev) / kDt);
            prev = x;
        }
        return vmax;
    };

    const double still_measured = peak_velocity_after(0.04, 0.04);
    const double unmeasured = peak_velocity_after(0.04, 0.0);   // IK failure ticks
    const double negative = peak_velocity_after(0.04, -1.0);    // explicit "unknown"
    RB_CHECK(unmeasured < 0.03);                                // stays throttled
    RB_CHECK(std::abs(unmeasured - still_measured) < 1e-9);     // identical to holding it
    RB_CHECK(std::abs(negative - still_measured) < 1e-9);
    // A real recovery still releases the guard — this must not become a one-way latch.
    RB_CHECK(peak_velocity_after(0.04, 0.20) > 0.08);

    // reset()/deactivate() drop the retained sample: a re-anchor is a new pose context.
    rb_servo::SmdPoseTracker t(cfg);
    t.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    t.setMinSingular(0.04);
    t.deactivate();
    t.reset({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    t.updateGoalFromCommand({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    t.updateGoalFromCommand({1.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    double vmax = 0.0, prev = 0.0;
    for (int i = 0; i < 800; ++i) {
        const double x = t.step(kDt).x;
        vmax = std::max(vmax, (x - prev) / kDt);
        prev = x;
    }
    RB_CHECK(vmax > 0.08);
    return true;
}


// QSYNC SETTLING HOLD (queue_sync.hold_motion_until_track), teleop side.
//
// While the hold pins the OUTPUT at prev_sent, a streamed TcpPoseTarget keeps
// arriving: its reference is the operator's hand, not a server clock, so it cannot
// be stopped the way computeServoTarget stops TcpLinearMove's quintic clock. Left
// running, the goal integrator accrues the whole hold and the SMD state chases it
// against a pinned output, and the lead discharges as a lunge on release.
// MEASURED 2026-08-28 (servo_log_20260828_100832.csv): 0.69 s of hold, 6.06 mm of
// right-arm lead, released at 99.7 deg/s / 6,068 deg/s^2 on J1 with two reversals.
//
// holdAt() every held tick is the fix: it pins state AND goal at the reference with
// zero velocity, so the release starts from where the arm actually is.
bool testSettlingHoldPinsTrackerAndKillsTheReleaseLunge() {
    // umi_large_smooth's tracking dynamics, so the numbers below are the ones the
    // teleop profile actually produces.
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.natural_frequency_linear_hz = 2.0;
    cfg.natural_frequency_angular_hz = 1.5;
    cfg.velocity_feedforward = true;
    cfg.max_linear_velocity_m_s = 0.30;
    cfg.max_linear_accel_m_s2 = 2.50;

    constexpr int kHoldTicks = 345;            // 0.69 s at 500 Hz: warmup 0.402 + drain 0.288
    constexpr double kHandSpeedMS = 0.05;      // operator keeps moving at 50 mm/s
    const rb_servo::Pose6D anchor{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // The commanded pose the source streams at tick i (absolute, as UMI teleop sends).
    const auto command_at = [&](int tick) {
        return rb_servo::Pose6D{kHandSpeedMS * kDt * tick, 0.0, 0.0, 0.0, 0.0, 0.0};
    };
    // Peak per-tick output step over `ticks` after the hold releases, plus the very
    // first post-release displacement away from the pinned pose.
    const auto release_profile = [&](bool pin_during_hold, double* first_jump_m) {
        rb_servo::SmdPoseTracker tracker(cfg);
        tracker.reset(anchor);
        const std::uint64_t reanchors_before = tracker.reanchorCount();
        for (int i = 0; i < kHoldTicks; ++i) {
            if (pin_during_hold) {
                // FIXED PATH: the stage returns the reference and never feeds the
                // goal integrator or steps the filter while the output is pinned.
                tracker.holdAt(anchor);
            } else {
                // BUG PATH: the stage kept smoothing against a pinned output.
                tracker.updateGoalFromCommand(command_at(i));
                tracker.step(kDt);
            }
        }
        if (pin_during_hold) {
            // A hold is not a re-anchor: smd_reanchor_count must not count its ticks.
            RB_CHECK_VALUE(tracker.reanchorCount() == reanchors_before, -1.0);
        }
        const double at_release = tracker.currentPose().x;
        *first_jump_m = std::abs(at_release - anchor.x);
        double peak_step = 0.0;
        double previous = at_release;
        for (int i = kHoldTicks; i < kHoldTicks + 250; ++i) {  // 0.5 s of release
            tracker.updateGoalFromCommand(command_at(i));
            const double x = tracker.step(kDt).x;
            peak_step = std::max(peak_step, std::abs(x - previous));
            previous = x;
        }
        return peak_step;
    };

    double bug_jump = 0.0;
    const double bug_peak_step = release_profile(false, &bug_jump);
    double fixed_jump = 0.0;
    const double fixed_peak_step = release_profile(true, &fixed_jump);
    RB_CHECK(fixed_peak_step >= 0.0 && bug_peak_step >= 0.0);  // -1 sentinel = reanchor leak

    // The bug path leaves the tracker tens of mm past the pinned arm (34.5 mm here:
    // with feedforward on it tracks the hand almost exactly). THAT OFFSET IS THE
    // LUNGE — the joint target steps from prev_sent to this pose on the release tick,
    // so the whole thing has to be closed at the clamp ceilings.
    RB_CHECK(bug_jump > 0.010);
    // The fix leaves it exactly at the reference, so the release injects nothing.
    RB_CHECK(fixed_jump < 1e-12);

    // Where the discharge is NOT: the SMD's own per-tick step is the same either way
    // (~0.1 mm, the commanded hand speed), because a stepped-but-pinned tracker is in
    // steady state, not sprinting. Asserting on that would test nothing. The defect
    // lives entirely in the accumulated OFFSET above, which the output layer sees as a
    // single-tick step: bug_jump / kDt = 17 m/s of implied catch-up against a 0.05 m/s
    // hand, versus exactly zero after the fix.
    RB_CHECK(fixed_peak_step < 1.5 * kHandSpeedMS * kDt);
    RB_CHECK(std::abs(bug_peak_step - fixed_peak_step) < 0.2 * kHandSpeedMS * kDt);
    RB_CHECK(bug_jump / kDt > 100.0 * kHandSpeedMS);

    // The tracker must resume normally afterwards: no dead filter, no latched hold.
    rb_servo::SmdPoseTracker resumed(cfg);
    resumed.reset(anchor);
    for (int i = 0; i < kHoldTicks; ++i) resumed.holdAt(anchor);
    RB_CHECK(resumed.active());
    for (int i = kHoldTicks; i < kHoldTicks + 1000; ++i) {
        resumed.updateGoalFromCommand(command_at(i));
        resumed.step(kDt);
    }
    // Steady-state tracking of a 50 mm/s ramp, resumed from the pinned pose. The goal
    // integrator restarts at the reference, so the output trails the ABSOLUTE command
    // by the whole hold offset by design — what matters is that it moves at the
    // commanded speed with the feedforward lag removed, not that it recovers the lead.
    const double travelled = resumed.currentPose().x;
    const double commanded_since_release = kHandSpeedMS * kDt * 1000;
    RB_CHECK(std::abs(travelled - commanded_since_release) < 0.002);
    return true;
}

bool testSeededResetCarriesVelocityIntoTheFirstSteps() {
    // A hand-off from a driver that was moving: the tracker must start MOVING at the
    // seeded velocity and shed it on its own critically damped dynamics, not stop in
    // one tick. With the command parked on the anchor the state coasts out to the
    // critically damped peak v0 / (wn e), then returns to the anchor without a
    // second overshoot.
    rb_servo::PoseTrackSmdConfig cfg = defaultConfig();
    cfg.natural_frequency_linear_hz = 3.5;
    cfg.natural_frequency_angular_hz = 2.5;
    cfg.max_linear_velocity_m_s = 0.5;
    cfg.max_angular_velocity_rad_s = 1.8;
    const rb_servo::Pose6D anchor{};  // identity: zero rotation vector
    const double v0 = 0.145;  // m/s along +x, the measured hand-off speed
    rb_servo::Vec6 twist{v0, 0.0, 0.0, 0.0, 0.0, 0.0};

    rb_servo::SmdPoseTracker seeded(cfg);
    seeded.reset(anchor, twist);
    rb_servo::SmdPoseTracker cold(cfg);
    cold.reset(anchor);
    RB_CHECK(seeded.active() && cold.active());

    const double wn = 2.0 * M_PI * cfg.natural_frequency_linear_hz;
    double prev_x = 0.0;
    double prev_step = v0 * kDt * 1.001;
    double first_step = -1.0;
    double peak_x = 0.0;
    int peak_tick = -1;
    double last_x = 0.0;
    for (int i = 0; i < 1000; ++i) {
        seeded.updateGoalFromCommand(anchor);  // stationary command
        cold.updateGoalFromCommand(anchor);
        const rb_servo::Pose6D p = seeded.step(kDt);
        const rb_servo::Pose6D c = cold.step(kDt);
        const double step = p.x - prev_x;
        if (i == 0) first_step = step;
        // Critically damped: the state never crosses back past the anchor.
        RB_CHECK(p.x >= -1e-9);
        // Before the peak the outbound step only ever shrinks (pure deceleration).
        if (peak_tick < 0) {
            RB_CHECK(step <= prev_step + 1e-12);
            if (step <= 0.0) {
                peak_tick = i;
                peak_x = prev_x;
            }
        }
        prev_step = step;
        prev_x = p.x;
        last_x = p.x;
        RB_CHECK(std::abs(c.x) < 1e-12);  // the cold tracker never moves
    }
    // First tick: the seeded velocity minus one tick of critical damping
    // (2 zeta wn v0 dt = 8.8 % at 3.5 Hz), never more than the seed itself.
    RB_CHECK(first_step > 0.85 * v0 * kDt);
    RB_CHECK(first_step <= v0 * kDt + 1e-12);
    // The excursion is the critically damped peak v0 / (wn e) at t = 1 / wn:
    // ~2.4 mm after ~45 ms, i.e. a coast, not a lunge and not a stop.
    RB_CHECK(peak_tick > 0);
    RB_CHECK(std::abs(peak_x - v0 / (wn * std::exp(1.0))) < 0.3e-3);
    RB_CHECK(std::abs(peak_tick * kDt - 1.0 / wn) < 0.01);
    // ...and it comes back to rest on the anchor.
    RB_CHECK(last_x < 1e-5);
    RB_CHECK(std::abs(prev_step) < 1e-7);

    // The seed is clamped to the profile caps.
    rb_servo::Vec6 fast{10.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    rb_servo::SmdPoseTracker capped(cfg);
    capped.reset(anchor, fast);
    capped.updateGoalFromCommand(anchor);
    const rb_servo::Pose6D q = capped.step(kDt);
    RB_CHECK(q.x <= cfg.max_linear_velocity_m_s * kDt + 1e-9);
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
    if (!testStepInfoClipFlagsAndReanchorCount()) return 1;
    if (!testDeactivateAndReanchor()) return 1;
    if (!testReengageRelatchRejectsStaleJump()) return 1;
    if (!testSingularityVelocityScaling()) return 1;
    if (!testUnmeasuredSigmaDoesNotReleaseTheSingularityGuard()) return 1;
    if (!testSettlingHoldPinsTrackerAndKillsTheReleaseLunge()) return 1;
    if (!testSeededResetCarriesVelocityIntoTheFirstSteps()) return 1;
    std::cout << "smd_pose_tracker tests passed\n";
    return 0;
}
