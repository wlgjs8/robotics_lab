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

// The overshoot budget caps how far past the target the deceleration ramp may sit.
bool testOvershootBudgetCapsCommandLead() {
    const double dt = 0.002;
    const double budget = 0.05;
    const rb_servo::SafetyConfig config = decelTestSafetyConfig(4.0, budget);
    const rb_servo::SafetyFilter filter(config);
    rb_servo::JointArray prevprev = joints(0.0);
    rb_servo::JointArray prev = joints(0.0);
    prev[0] = 196.0 * dt;
    rb_servo::JointArray desired = prev;
    desired[0] = prev[0] + 0.0;   // target stops dead
    const rb_servo::SafetyClampTelemetry clamp =
        filter.clampMotionDetailed(desired, prev, prevprev, dt);
    const double lead = clamp.q_after_accel_limit_deg[0] - desired[0];
    RB_CHECK(lead > 0.0);                    // bounded decel means the command leads
    RB_CHECK(near(lead, budget));            // and the lead is capped at the budget
    // Zero budget collapses back to the legacy "never pass the target" clip.
    const rb_servo::SafetyFilter strict(decelTestSafetyConfig(4.0, 0.0));
    const rb_servo::SafetyClampTelemetry strict_clamp =
        strict.clampMotionDetailed(desired, prev, prevprev, dt);
    RB_CHECK(near(strict_clamp.q_after_accel_limit_deg[0], desired[0]));
    return true;
}

// Steady streaming must be untouched by either knob -- no lag, no lead.
bool testSteadyStreamingIsUnaffected() {
    const double dt = 0.002;
    const rb_servo::SafetyConfig config = decelTestSafetyConfig(4.0, 0.5);
    RB_CHECK(near(realizedVelDegS(config, 120.0, 120.0, dt), 120.0));
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
    if (!testOvershootBudgetCapsCommandLead()) return 1;
    if (!testSteadyStreamingIsUnaffected()) return 1;
    return 0;
}
