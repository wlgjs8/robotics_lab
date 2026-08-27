#include <cmath>
#include <iostream>

#include <nlohmann/json.hpp>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/safety_filter.hpp"
#include "rb_servo/network/state_publisher.hpp"

namespace {

constexpr double kEpsilon = 1e-9;

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

bool near(double a, double b) {
    return std::abs(a - b) < kEpsilon;
}

rb_servo::JointArray joints(double value) {
    rb_servo::JointArray out{};
    out.fill(value);
    return out;
}

rb_servo::SafetyConfig rbpodoRawControllerTestSafetyConfig() {
    rb_servo::SafetyConfig config;
    config.q_min_deg = rb_servo::rbpodoDefaultSafetyJointMinDeg();
    config.q_max_deg = rb_servo::rbpodoDefaultSafetyJointMaxDeg();
    config.dq_max_deg_s.fill(1000.0);
    config.ddq_max_deg_s2.fill(1000.0);
    config.max_tracking_error_deg = 1000.0;
    return config;
}

bool testNormalizeWrappedJointIntoRange() {
    const rb_servo::JointRangeNormalization normalized =
        rb_servo::normalizeJointForRange(-317.0, -190.0, 190.0, 360.0);
    RB_CHECK(normalized.was_wrapped);
    RB_CHECK(normalized.equivalent_in_range);
    RB_CHECK(near(normalized.normalized_value_deg, 43.0));
    return true;
}

bool testNoWrappingWhenPeriodZero() {
    const rb_servo::JointRangeNormalization normalized =
        rb_servo::normalizeJointForRange(-317.0, -190.0, 190.0, 0.0);
    RB_CHECK(!normalized.was_wrapped);
    RB_CHECK(!normalized.equivalent_in_range);
    RB_CHECK(near(normalized.normalized_value_deg, -317.0));
    return true;
}

bool testAmbiguousFullPeriodRangeDoesNotNormalize() {
    const rb_servo::JointRangeNormalization normalized =
        rb_servo::normalizeJointForRange(540.0, -180.0, 180.0, 360.0);
    RB_CHECK(normalized.was_wrapped);
    RB_CHECK(normalized.equivalent_in_range);
    RB_CHECK(near(normalized.normalized_value_deg, -180.0));
    return true;
}

bool testMotionSafetyDoesNotWrapTargetsByDefault() {
    rb_servo::SafetyConfig config;
    // Intentionally narrow diagnostic range to prove motion targets clamp instead of wrapping.
    config.q_min_deg.fill(-190.0);
    config.q_max_deg.fill(190.0);
    config.dq_max_deg_s.fill(1000.0);
    config.ddq_max_deg_s2.fill(1000.0);
    config.max_tracking_error_deg = 1000.0;
    config.joint_wrap_period_deg = {0.0, 0.0, 0.0, 360.0, 0.0, 360.0};
    config.joint_wrap_for_startup_validation = true;
    config.joint_wrap_for_motion_safety = false;

    rb_servo::SafetyFilter filter(config);
    rb_servo::RobotState state;
    state.q_actual_deg = joints(0.0);
    state.has_valid_joint_state = true;
    state.connection_state = rb_servo::RobotConnectionState::Connected;

    rb_servo::JointArray desired = joints(0.0);
    desired[3] = -317.0;
    const rb_servo::JointArray previous = joints(0.0);
    const rb_servo::SafetyCheckResult result =
        filter.filterJointTarget(desired, previous, previous, state, 10.0);

    RB_CHECK(result.ok);
    RB_CHECK(result.joint_limit_clamped);
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::JointLimitClamped);
    RB_CHECK(near(result.filtered_q_deg[3], -190.0));
    return true;
}

rb_servo::SafetyConfig trackingTestConfig() {
    rb_servo::SafetyConfig config = rbpodoRawControllerTestSafetyConfig();
    config.max_tracking_error_deg = 2.0;
    return config;
}

rb_servo::RobotState connectedState(rb_servo::JointArray q_actual) {
    rb_servo::RobotState state;
    state.q_actual_deg = q_actual;
    state.q_target_deg = q_actual;
    state.has_valid_joint_state = true;
    state.connection_state = rb_servo::RobotConnectionState::Connected;
    return state;
}

bool testTrackingErrorUsesActualByDefault() {
    rb_servo::SafetyFilter filter(trackingTestConfig());
    rb_servo::RobotState state = connectedState(joints(0.0));
    rb_servo::JointArray previous = joints(0.0);
    previous[0] = 5.0;
    RB_CHECK(filter.hasTrackingError(previous, state));
    const rb_servo::SafetyCheckResult result =
        filter.filterJointTarget(previous, previous, previous, state, 0.01);
    RB_CHECK(!result.ok);
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::TrackingError);
    RB_CHECK(result.tracking.tracking_error_source == "actual");
    return true;
}

bool testReferenceTrackingOverrideCanPassWithStaticActual() {
    rb_servo::SafetyFilter filter(trackingTestConfig());
    rb_servo::RobotState state = connectedState(joints(0.0));
    rb_servo::JointArray previous = joints(0.0);
    previous[0] = 5.0;
    state.q_target_deg = previous;

    rb_servo::SafetyTrackingState tracking;
    tracking.override_tracking_q = true;
    tracking.tracking_q_deg = state.q_target_deg;
    tracking.source = "reference";
    const rb_servo::SafetyCheckResult result =
        filter.filterJointTarget(previous, previous, previous, state, 0.01, tracking);
    RB_CHECK(result.ok);
    RB_CHECK(result.tracking.tracking_error_source == "reference");
    RB_CHECK(!result.tracking.controller_simulation_physical_motion_detected);
    RB_CHECK(result.tracking.command_reference_tracking_error_deg < kEpsilon);
    RB_CHECK(result.tracking.physical_command_actual_error_deg > 4.0);
    return true;
}

bool testReferenceTrackingInvalidFailsClosed() {
    rb_servo::SafetyFilter filter(trackingTestConfig());
    rb_servo::RobotState state = connectedState(joints(0.0));
    rb_servo::SafetyTrackingState tracking;
    tracking.override_tracking_q = true;
    tracking.source = "reference";
    tracking.source_valid = false;
    tracking.reason = "controller_simulation_reference_state_unavailable";
    const rb_servo::SafetyCheckResult result =
        filter.filterJointTarget(joints(0.0), joints(0.0), joints(0.0), state, 0.01, tracking);
    RB_CHECK(!result.ok);
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(result.reason == "controller_simulation_reference_state_unavailable");
    RB_CHECK(!result.tracking.tracking_error_source_valid);
    return true;
}

bool testControllerSimulationPhysicalMotionFaultsClosed() {
    rb_servo::SafetyFilter filter(trackingTestConfig());
    rb_servo::RobotState state = connectedState(joints(0.0));
    rb_servo::SafetyTrackingState tracking;
    tracking.override_tracking_q = true;
    tracking.tracking_q_deg = joints(0.0);
    tracking.source = "reference";
    tracking.controller_simulation_physical_motion_detected = true;
    tracking.controller_simulation_physical_motion_fault = true;
    tracking.reason = "controller_simulation_physical_motion_detected";
    const rb_servo::SafetyCheckResult result =
        filter.filterJointTarget(joints(0.0), joints(0.0), joints(0.0), state, 0.01, tracking);
    RB_CHECK(!result.ok);
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::TrackingError);
    RB_CHECK(result.reason == "controller_simulation_physical_motion_detected");
    RB_CHECK(result.tracking.controller_simulation_physical_motion_detected);
    return true;
}

bool testTrackingPreservesRawControllerValuesInsideConfiguredRange() {
    rb_servo::SafetyConfig config = rbpodoRawControllerTestSafetyConfig();
    rb_servo::SafetyFilter filter(config);

    rb_servo::JointArray raw = joints(0.0);
    raw[0] = 270.0;
    raw[2] = -317.0;
    rb_servo::RobotState state = connectedState(raw);

    rb_servo::JointArray desired = raw;
    desired[0] = 271.0;
    const rb_servo::SafetyCheckResult result =
        filter.filterJointTarget(desired, raw, raw, state, 10.0);

    RB_CHECK(result.ok);
    RB_CHECK(!result.joint_limit_clamped);
    RB_CHECK(near(result.filtered_q_deg[0], 271.0));
    RB_CHECK(near(result.filtered_q_deg[2], -317.0));
    RB_CHECK(result.tracking.physical_command_actual_error_deg < kEpsilon);
    return true;
}

bool testCommandTargetClampingUsesConfiguredRawRangeWithoutWrap() {
    rb_servo::SafetyConfig config = rbpodoRawControllerTestSafetyConfig();
    rb_servo::SafetyFilter filter(config);
    rb_servo::RobotState state = connectedState(joints(0.0));

    rb_servo::JointArray desired = joints(0.0);
    desired[0] = 400.0;
    desired[1] = -400.0;
    desired[2] = 270.0;
    const rb_servo::SafetyCheckResult result =
        filter.filterJointTarget(desired, joints(0.0), joints(0.0), state, 10.0);

    RB_CHECK(result.ok);
    RB_CHECK(result.joint_limit_clamped);
    RB_CHECK(near(result.filtered_q_deg[0], 360.0));
    RB_CHECK(near(result.filtered_q_deg[1], -360.0));
    RB_CHECK(near(result.filtered_q_deg[2], 270.0));
    return true;
}

bool testStatePublisherSerializesWrapDiagnostics() {
    rb_servo::ServoSnapshot snapshot;
    snapshot.left_state.q_actual_deg = joints(0.0);
    snapshot.left_state.q_actual_deg[2] = -317.0;
    snapshot.startup_validation.left.q_range_wrapped.push_back({3, -317.0, 43.0, 360.0});
    rb_servo::JointArray normalized = snapshot.left_state.q_actual_deg;
    normalized[2] = 43.0;
    snapshot.startup_validation.left.q_actual_normalized_for_safety_deg = normalized;

    rb_servo::StatePublisher publisher(rb_servo::NetworkConfig{});
    const nlohmann::json message = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    const nlohmann::json wrapped = message.at("left").at("q_range_wrapped").at(0);
    RB_CHECK(wrapped.at("joint").get<int>() == 3);
    RB_CHECK(near(wrapped.at("raw_deg").get<double>(), -317.0));
    RB_CHECK(near(wrapped.at("normalized_deg").get<double>(), 43.0));
    RB_CHECK(near(wrapped.at("period_deg").get<double>(), 360.0));
    RB_CHECK(near(message.at("left").at("q_actual_deg").at(2).get<double>(), -317.0));
    RB_CHECK(near(message.at("left").at("q_actual_normalized_for_safety_deg").at(2).get<double>(), 43.0));
    RB_CHECK(
        near(
            message.at("startup_validation")
                .at("left")
                .at("q_range_wrapped")
                .at(0)
                .at("normalized_deg")
                .get<double>(),
            43.0
        )
    );
    return true;
}


// ---- Deceleration limiting (2026-08-26 shake fix) ----------------------------

rb_servo::SafetyConfig decelTestSafetyConfig(double ratio, double budget_deg) {
    rb_servo::SafetyConfig config;
    config.q_min_deg = joints(-360.0);
    config.q_max_deg = joints(360.0);
    config.dq_max_deg_s.fill(100000.0);   // velocity clamp inert: isolate the accel stage
    config.ddq_max_deg_s2.fill(3000.0);
    config.max_tracking_error_deg = 1000.0;
    config.ddq_max_decel_ratio = ratio;
    config.decel_overshoot_budget_deg = budget_deg;
    return config;
}

// Drive one joint: q_prevprev -> q_prev at `prev_vel`, then ask for `desired_vel`.
// Returns the realized velocity of joint 0 after the clamp stack.
double realizedVelDegS(const rb_servo::SafetyConfig& config,
                       double prev_vel_deg_s,
                       double desired_vel_deg_s,
                       double dt_sec) {
    const rb_servo::SafetyFilter filter(config);
    rb_servo::JointArray prevprev = joints(0.0);
    rb_servo::JointArray prev = joints(0.0);
    prev[0] = prev_vel_deg_s * dt_sec;
    rb_servo::JointArray desired = prev;
    desired[0] = prev[0] + desired_vel_deg_s * dt_sec;
    const rb_servo::SafetyClampTelemetry clamp =
        filter.clampMotionDetailed(desired, prev, prevprev, dt_sec);
    return (clamp.q_after_accel_limit_deg[0] - prev[0]) / dt_sec;
}

// ratio 1.0 + budget 0.0 must reproduce the legacy filter exactly: acceleration is
// bounded by ddq_max, deceleration is not bounded at all.
bool testLegacyDecelBehaviorPreservedAtRatioOne() {
    const double dt = 0.002;
    const rb_servo::SafetyConfig config = decelTestSafetyConfig(1.0, 0.0);
    // Acceleration is clamped to ddq_max*dt = 6 deg/s per tick.
    RB_CHECK(near(realizedVelDegS(config, 0.0, 500.0, dt), 6.0));
    // Deceleration passes through untouched -- this is the legacy behavior the fix
    // makes configurable, kept as the default so nothing changes without opting in.
    RB_CHECK(near(realizedVelDegS(config, 196.0, 60.0, dt), 60.0));
    RB_CHECK(near(realizedVelDegS(config, 196.0, 0.0, dt), 0.0));
    return true;
}

// With a ratio > 1 the same hard stop is spread over the deceleration ceiling instead
// of landing in one tick. Reproduces the measured servo_log_20260826_042818.csv event:
// right J6 at 196 deg/s with the stream demanding 60 deg/s the next tick.
bool testDecelerationIsBoundedByRatio() {
    const double dt = 0.002;
    const rb_servo::SafetyConfig config = decelTestSafetyConfig(4.0, 1000.0);
    const double vel = realizedVelDegS(config, 196.0, 60.0, dt);
    // ddq_max*ratio*dt = 3000*4*0.002 = 24 deg/s of shed speed per tick.
    RB_CHECK(near(vel, 196.0 - 24.0));
    const double realized_decel = (vel - 196.0) / dt;
    RB_CHECK(realized_decel > -(3000.0 * 4.0) - kEpsilon);
    // The legacy filter would have realized -68,000 deg/s^2 here.
    RB_CHECK(realized_decel > -13000.0);
    // Acceleration is NOT widened by the ratio: still ddq_max.
    RB_CHECK(near(realizedVelDegS(config, 0.0, 500.0, dt), 6.0));
    return true;
}

// A sign reversal counts as shedding speed (it must decelerate through zero first),
// so it gets the deceleration budget rather than the acceleration one.
bool testSignReversalUsesDecelerationBudget() {
    const double dt = 0.002;
    const rb_servo::SafetyConfig config = decelTestSafetyConfig(4.0, 1000.0);
    RB_CHECK(near(realizedVelDegS(config, 100.0, -100.0, dt), 100.0 - 24.0));
    return true;
}

// Anti-overshoot with bounded jerk (2026-08-26 rev 2). The original budget clip
// was an ASSIGNMENT (`out = q +/- budget`): it capped the per-tick lead exactly
// but could move the command up to `budget` deg in one 2 ms tick (~125,000
// deg/s^2 at budget 0.5) and produced a +/-budget square wave on a hovering
// target. The clip is now rate-limited by the same decel dv budget as the ramp:
// per-tick velocity change NEVER exceeds ratio*ddq_max*dt, the excursion past a
// dead-stopped target is bounded by the physics coast v^2/(2*ratio*ddq), and
// the command then returns to the target. The budget knob shapes the transient
// but no longer authorizes a teleport.
bool testOvershootClipIsRateLimited() {
    const double dt = 0.002;
    const double budget = 0.05;
    const rb_servo::SafetyConfig config = decelTestSafetyConfig(4.0, budget);
    const rb_servo::SafetyFilter filter(config);
    const double dv_limit = 3000.0 * 4.0 * dt;  // 24 deg/s per tick
    const double v0 = 196.0;
    const double coast_bound = v0 * v0 / (2.0 * 3000.0 * 4.0);  // 1.6 deg

    rb_servo::JointArray prevprev = joints(0.0);
    rb_servo::JointArray prev = joints(0.0);
    prev[0] = v0 * dt;
    rb_servo::JointArray desired = prev;   // target stops dead at prev
    double max_excursion = 0.0;
    double out0 = prev[0];
    for (int tick = 0; tick < 400; ++tick) {
        const rb_servo::SafetyClampTelemetry clamp =
            filter.clampMotionDetailed(desired, prev, prevprev, dt);
        const double out = clamp.q_after_accel_limit_deg[0];
        const double vel = (out - prev[0]) / dt;
        const double prev_vel = (prev[0] - prevprev[0]) / dt;
        // The whole point: no tick may change velocity faster than the decel
        // ceiling -- the legacy clip teleported here.
        RB_CHECK(std::abs(vel - prev_vel) <= dv_limit + kEpsilon);
        max_excursion = std::max(max_excursion, out - desired[0]);
        prevprev = prev;
        prev[0] = out;
        out0 = out;
    }
    RB_CHECK(max_excursion > 0.0);                      // bounded decel must lead
    RB_CHECK(max_excursion <= coast_bound + 0.05);      // but only by the coast
    RB_CHECK(std::abs(out0 - desired[0]) <= budget + kEpsilon);  // and it returns
    return true;
}

// Steady streaming must be untouched by either knob -- no lag, no lead.
bool testSteadyStreamingIsUnaffected() {
    const double dt = 0.002;
    const rb_servo::SafetyConfig config = decelTestSafetyConfig(4.0, 0.5);
    RB_CHECK(near(realizedVelDegS(config, 120.0, 120.0, dt), 120.0));
    return true;
}


// ---- Joint-limit approach barrier (2026-08-26) -------------------------------

rb_servo::SafetyConfig barrierConfig(double band_deg, double a_brake, double limit_deg) {
    rb_servo::SafetyConfig config;
    config.q_min_deg = joints(-360.0);
    config.q_max_deg = joints(360.0);
    config.dq_max_deg_s.fill(170.0);
    config.ddq_max_deg_s2.fill(100000.0);   // accel stage inert: isolate the barrier
    config.max_tracking_error_deg = 1000.0;
    auto& jb = config.joint_limit_barrier;
    jb.enable = true;
    jb.inherit_bounds = false;
    jb.q_min_deg = joints(-limit_deg);
    jb.q_max_deg = joints(limit_deg);
    jb.d_slow_deg.fill(band_deg);
    jb.a_brake_deg_s2.fill(a_brake);
    return config;
}

// Returns joint 0's realized step (deg) for a commanded step from q_prev.
double barrierStepDeg(const rb_servo::SafetyConfig& config,
                      double q_prev_deg,
                      double commanded_step_deg,
                      double dt_sec) {
    const rb_servo::SafetyFilter filter(config);
    rb_servo::JointArray prevprev = joints(0.0);
    rb_servo::JointArray prev = joints(0.0);
    prev[0] = q_prev_deg;
    prevprev[0] = q_prev_deg;
    rb_servo::JointArray desired = prev;
    desired[0] = q_prev_deg + commanded_step_deg;
    const rb_servo::SafetyClampTelemetry clamp =
        filter.clampMotionDetailed(desired, prev, prevprev, dt_sec);
    return clamp.q_after_accel_limit_deg[0] - q_prev_deg;
}

// THE BARRIER COMES TO REST INSIDE THE BOUND, AND RETREAT STAYS FREE.
// A joint parked exactly on its bound holds its servo against the stop: measured
// 2026-08-27 the left elbow sat at 150.000 deg for 8 s while the encoder rang at
// 17 Hz (47% of its 5-50 Hz band energy vs 1.9% free), which is the noise the
// operator heard. 0.057 deg of clearance removed the peak entirely, so the
// barrier now brakes onto (bound - standoff). The bound itself is unchanged.
bool testJointLimitBarrierStandoffRestsShortOfTheBound() {
    const double dt = 0.002;
    const double limit = 150.0;
    const double band = 12.0;
    const double a = 1500.0;
    const double standoff = 0.10;
    rb_servo::SafetyConfig config = barrierConfig(band, a, limit);
    config.joint_limit_barrier.standoff_deg.fill(standoff);
    const double full_step = 170.0 * dt;

    // Integrating a full-speed closing command from the band edge must come to
    // rest at the standoff, NOT on the bound.
    double q = limit - band;
    for (int k = 0; k < 20000; ++k) {
        const double step = barrierStepDeg(config, q, full_step, dt);
        q += step;
        RB_CHECK(q <= limit - standoff + 1e-9);   // never reaches the bound itself
        if (step <= 1e-12) break;
    }
    RB_CHECK(q > limit - band);                    // it advanced
    RB_CHECK(std::abs(q - (limit - standoff)) < 0.02);   // and settled at the standoff

    // Sitting AT the standoff: no further closing, retreat unrestricted.
    RB_CHECK(near(barrierStepDeg(config, limit - standoff, full_step, dt), 0.0));
    RB_CHECK(near(barrierStepDeg(config, limit - standoff, -full_step, dt), -full_step));

    // Already INSIDE the standoff (e.g. parked there by PTP, or pushed there before
    // the standoff was configured): closing is refused, and -- the property that
    // matters for getting out -- retreat is still at full commanded speed.
    RB_CHECK(near(barrierStepDeg(config, limit - 0.5 * standoff, full_step, dt), 0.0));
    RB_CHECK(near(barrierStepDeg(config, limit - 0.5 * standoff, -full_step, dt), -full_step));
    RB_CHECK(near(barrierStepDeg(config, limit, -full_step, dt), -full_step));
    RB_CHECK(near(barrierStepDeg(config, limit + 0.2, -full_step, dt), -full_step));

    // Symmetric at the lower bound.
    RB_CHECK(near(barrierStepDeg(config, -(limit - standoff), -full_step, dt), 0.0));
    RB_CHECK(near(barrierStepDeg(config, -(limit - standoff), full_step, dt), full_step));

    // Far from the bound nothing changes.
    RB_CHECK(near(barrierStepDeg(config, 0.0, full_step, dt), full_step));

    // standoff 0 reproduces the previous behaviour exactly (rest ON the bound).
    rb_servo::SafetyConfig legacy = barrierConfig(band, a, limit);
    legacy.joint_limit_barrier.standoff_deg.fill(0.0);
    double ql = limit - band;
    for (int k = 0; k < 20000; ++k) {
        const double step = barrierStepDeg(legacy, ql, full_step, dt);
        ql += step;
        if (step <= 1e-12) break;
    }
    RB_CHECK(std::abs(ql - limit) < 0.02);
    return true;
}

bool testJointLimitBarrierBrakesOnlyTheClosingDirection() {
    const double dt = 0.002;
    const double limit = 150.0;      // the RB3-730E J3 URDF IK limit
    const double band = 12.0;
    const double a = 1500.0;
    const rb_servo::SafetyConfig config = barrierConfig(band, a, limit);
    const double full_step = 170.0 * dt;   // dq_max*dt = 0.34 deg

    // Far from the bound: untouched.
    RB_CHECK(near(barrierStepDeg(config, 0.0, full_step, dt), full_step));
    RB_CHECK(near(barrierStepDeg(config, limit - band - 1.0, full_step, dt), full_step));

    // Inside the band and CLOSING: capped at sqrt(2*a*margin)*dt.
    const double margin = 4.0;
    const double allowed = std::sqrt(2.0 * a * margin) * dt;
    RB_CHECK(allowed < full_step);
    RB_CHECK(near(barrierStepDeg(config, limit - margin, full_step, dt), allowed));

    // Same pose, RETREATING: never limited, so the arm can always be commanded out.
    RB_CHECK(near(barrierStepDeg(config, limit - margin, -full_step, dt), -full_step));

    // Symmetric on the lower bound.
    RB_CHECK(near(barrierStepDeg(config, -(limit - margin), -full_step, dt), -allowed));
    RB_CHECK(near(barrierStepDeg(config, -(limit - margin), full_step, dt), full_step));

    // AT the bound: no further closing at all, but retreat is still free -- this is what
    // replaces "pin at full speed, then chatter".
    RB_CHECK(near(barrierStepDeg(config, limit, full_step, dt), 0.0));
    RB_CHECK(near(barrierStepDeg(config, limit, -full_step, dt), -full_step));

    // The braking profile really does stop AT the bound: integrating the cap from the
    // band edge must never cross the limit.
    double q = limit - band;
    for (int k = 0; k < 20000; ++k) {
        const double step = barrierStepDeg(config, q, full_step, dt);
        q += step;
        RB_CHECK(q <= limit + 1e-9);
        if (step <= 1e-12) break;
    }
    RB_CHECK(q <= limit + 1e-9);
    RB_CHECK(q > limit - band);   // it did advance, it did not deadlock at the edge

    // Disabled by default: identical to the legacy filter.
    rb_servo::SafetyConfig off = config;
    off.joint_limit_barrier.enable = false;
    RB_CHECK(near(barrierStepDeg(off, limit - margin, full_step, dt), full_step));
    return true;
}

}  // namespace

int main() {
    if (!testNormalizeWrappedJointIntoRange()) return 1;
    if (!testNoWrappingWhenPeriodZero()) return 1;
    if (!testAmbiguousFullPeriodRangeDoesNotNormalize()) return 1;
    if (!testMotionSafetyDoesNotWrapTargetsByDefault()) return 1;
    if (!testTrackingErrorUsesActualByDefault()) return 1;
    if (!testReferenceTrackingOverrideCanPassWithStaticActual()) return 1;
    if (!testReferenceTrackingInvalidFailsClosed()) return 1;
    if (!testControllerSimulationPhysicalMotionFaultsClosed()) return 1;
    if (!testTrackingPreservesRawControllerValuesInsideConfiguredRange()) return 1;
    if (!testCommandTargetClampingUsesConfiguredRawRangeWithoutWrap()) return 1;
    if (!testStatePublisherSerializesWrapDiagnostics()) return 1;
    if (!testLegacyDecelBehaviorPreservedAtRatioOne()) return 1;
    if (!testDecelerationIsBoundedByRatio()) return 1;
    if (!testSignReversalUsesDecelerationBudget()) return 1;
    if (!testOvershootClipIsRateLimited()) return 1;
    if (!testSteadyStreamingIsUnaffected()) return 1;
    if (!testJointLimitBarrierBrakesOnlyTheClosingDirection()) return 1;
    if (!testJointLimitBarrierStandoffRestsShortOfTheBound()) return 1;
    return 0;
}
