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

}  // namespace

int main() {
    if (!testNormalizeWrappedJointIntoRange()) return 1;
    if (!testNoWrappingWhenPeriodZero()) return 1;
    if (!testAmbiguousFullPeriodRangeDoesNotNormalize()) return 1;
    if (!testMotionSafetyDoesNotWrapTargetsByDefault()) return 1;
    if (!testStatePublisherSerializesWrapDiagnostics()) return 1;
    return 0;
}
