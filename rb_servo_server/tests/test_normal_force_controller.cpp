#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "rb_servo/control/normal_force_controller.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

bool near(double lhs, double rhs, double tolerance = 1e-12) {
    return std::abs(lhs - rhs) <= tolerance;
}

rb_servo::NormalForceControllerConfig controllerConfig() {
    rb_servo::NormalForceControllerConfig config;
    config.enable = true;
    config.virtual_mass_kg = 1.0;
    config.damping_n_s_per_m = 0.0;
    config.stiffness_n_per_m = 0.0;
    config.force_deadband_n = 0.5;
    config.max_dt_sec = 0.02;
    config.max_unload_offset_m = 1.0;
    config.max_unload_velocity_m_s = 10.0;
    config.max_unload_acceleration_m_s2 = 100.0;
    config.max_unload_jerk_m_s3 = 10'000.0;
    config.max_unload_step_m = 1.0;
    config.max_observed_energy_j = 10.0;
    return config;
}

rb_servo::NormalForceControllerCommand target(double force_n) {
    rb_servo::NormalForceControllerCommand command;
    command.target_contact_force_n = force_n;
    return command;
}

bool testExcessForceProducesOnlyOutwardCorrection() {
    rb_servo::NormalForceController controller(controllerConfig());
    controller.engage();

    const auto excess = controller.propose(10.0, target(3.0), 0.0, 0.01);
    RB_CHECK(excess.valid);
    RB_CHECK(near(excess.force_error_n, 7.0));
    RB_CHECK(near(excess.controlled_force_error_n, 6.5));
    RB_CHECK(near(excess.state.unload_acceleration_m_s2, 6.5));
    RB_CHECK(near(excess.state.unload_velocity_m_s, 0.065));
    RB_CHECK(near(excess.state.unload_offset_m, 0.00065));
    RB_CHECK(excess.state.unload_offset_m > 0.0);
    RB_CHECK(controller.commit(excess));

    // A subsequently below-target force may remove the prior unloading
    // correction, but the unilateral boundary prevents crossing past nominal.
    const auto return_to_nominal = controller.propose(0.0, target(100.0), 0.0, 0.01);
    RB_CHECK(return_to_nominal.valid);
    RB_CHECK(return_to_nominal.saturated);
    RB_CHECK(near(return_to_nominal.state.unload_offset_m, 0.0));
    RB_CHECK(return_to_nominal.state.unload_offset_m >= 0.0);

    // Below-target force at the nominal pose cannot create an inward correction
    // that would seek contact beyond the policy's nominal target.
    rb_servo::NormalForceController below_target(controllerConfig());
    below_target.engage();
    const auto no_penetration = below_target.propose(2.0, target(3.0), 0.0, 0.01);
    RB_CHECK(no_penetration.valid);
    RB_CHECK(no_penetration.saturated);
    RB_CHECK(near(no_penetration.force_error_n, -1.0));
    RB_CHECK(near(no_penetration.state.unload_offset_m, 0.0));
    RB_CHECK(near(no_penetration.state.unload_velocity_m_s, 0.0));
    RB_CHECK(near(no_penetration.state.unload_acceleration_m_s2, 0.0));
    return true;
}

bool testContinuousDeadband() {
    rb_servo::NormalForceController controller(controllerConfig());
    controller.engage();

    const auto inside = controller.propose(3.4, target(3.0), 0.0, 0.01);
    RB_CHECK(inside.valid);
    RB_CHECK(inside.in_deadband);
    RB_CHECK(near(inside.controlled_force_error_n, 0.0));
    RB_CHECK(near(inside.state.unload_offset_m, 0.0));

    const auto outside = controller.propose(3.7, target(3.0), 0.0, 0.01);
    RB_CHECK(outside.valid);
    RB_CHECK(!outside.in_deadband);
    RB_CHECK(near(outside.force_error_n, 0.7));
    RB_CHECK(near(outside.controlled_force_error_n, 0.2));
    RB_CHECK(near(outside.state.unload_offset_m, 0.00002));
    return true;
}

bool testDynamicAndUnilateralBoundsDoNotWindUp() {
    rb_servo::NormalForceControllerConfig config = controllerConfig();
    config.force_deadband_n = 0.0;
    config.max_unload_offset_m = 0.00002;
    config.max_unload_velocity_m_s = 0.001;
    config.max_unload_acceleration_m_s2 = 2.0;
    config.max_unload_jerk_m_s3 = 10.0;
    config.max_unload_step_m = 0.000005;
    rb_servo::NormalForceController controller(config);
    controller.engage();

    bool reached_boundary = false;
    for (int i = 0; i < 20; ++i) {
        const auto previous = controller.state();
        const double dt = 0.01;
        const auto proposal = controller.propose(100.0, target(0.0), 0.0, dt);
        RB_CHECK(proposal.valid);
        RB_CHECK(proposal.state.unload_offset_m >= 0.0);
        RB_CHECK(proposal.state.unload_offset_m <= config.max_unload_offset_m + 1e-12);
        RB_CHECK(std::abs(proposal.state.unload_velocity_m_s) <=
                 config.max_unload_velocity_m_s + 1e-12);
        const double realized_acceleration =
            (proposal.state.unload_velocity_m_s - previous.unload_velocity_m_s) / dt;
        const double realized_jerk =
            (proposal.state.unload_acceleration_m_s2 -
             previous.unload_acceleration_m_s2) / dt;
        RB_CHECK(std::abs(realized_acceleration) <=
                 config.max_unload_acceleration_m_s2 + 1e-12);
        RB_CHECK(std::abs(realized_jerk) <= config.max_unload_jerk_m_s3 + 1e-12);
        RB_CHECK(controller.commit(proposal));
        if (near(controller.state().unload_offset_m, config.max_unload_offset_m)) {
            reached_boundary = true;
        }
    }
    RB_CHECK(reached_boundary);
    RB_CHECK(near(controller.state().unload_offset_m, config.max_unload_offset_m));
    RB_CHECK(near(controller.state().unload_velocity_m_s, 0.0));
    return true;
}

bool testEnergyLimitRejectsWithoutMutation() {
    rb_servo::NormalForceControllerConfig config = controllerConfig();
    config.max_observed_energy_j = 0.05;
    rb_servo::NormalForceController controller(config);
    controller.engage();

    const auto rejected = controller.propose(10.0, target(3.0), 1.0, 0.01);
    RB_CHECK(!rejected.valid);
    RB_CHECK(!controller.commit(rejected));
    RB_CHECK(near(controller.state().observed_energy_j, 0.0));
    RB_CHECK(near(controller.state().unload_offset_m, 0.0));

    const auto dissipative = controller.propose(10.0, target(3.0), -1.0, 0.01);
    RB_CHECK(dissipative.valid);
    RB_CHECK(near(dissipative.state.observed_energy_j, 0.0));
    return true;
}

bool testTwoPhaseLifecycleAndProvenance() {
    rb_servo::NormalForceController first(controllerConfig());
    rb_servo::NormalForceController second(controllerConfig());
    first.engage();
    second.engage();

    const auto proposal = first.propose(10.0, target(3.0), 0.0, 0.01);
    RB_CHECK(proposal.valid);
    first.reject();
    RB_CHECK(near(first.state().unload_offset_m, 0.0));
    RB_CHECK(!second.commit(proposal));
    RB_CHECK(first.commit(proposal));
    RB_CHECK(!first.commit(proposal));

    const auto stale_lifecycle = first.propose(10.0, target(3.0), 0.0, 0.01);
    RB_CHECK(stale_lifecycle.valid);
    first.release();
    RB_CHECK(!first.engaged());
    RB_CHECK(near(first.state().unload_offset_m, 0.0));
    first.engage();
    RB_CHECK(!first.commit(stale_lifecycle));
    first.reset();
    RB_CHECK(!first.engaged());
    return true;
}

bool testInvalidInputsAndConfigFailClosed() {
    rb_servo::NormalForceControllerConfig disabled_config = controllerConfig();
    disabled_config.enable = false;
    rb_servo::NormalForceController disabled(disabled_config);
    disabled.engage();
    RB_CHECK(!disabled.propose(10.0, target(3.0), 0.0, 0.01).valid);

    rb_servo::NormalForceController controller(controllerConfig());
    controller.engage();
    RB_CHECK(!controller.propose(10.0, target(-1.0), 0.0, 0.01).valid);
    RB_CHECK(!controller.propose(
        std::numeric_limits<double>::quiet_NaN(), target(3.0), 0.0, 0.01).valid);
    RB_CHECK(!controller.propose(10.0, target(3.0), 0.0, 0.0).valid);
    RB_CHECK(!controller.propose(10.0, target(3.0), 0.0, 0.03).valid);

    bool rejected_bad_config = false;
    try {
        rb_servo::NormalForceControllerConfig bad = controllerConfig();
        bad.virtual_mass_kg = 0.0;
        rb_servo::NormalForceController invalid(bad);
    } catch (const std::invalid_argument&) {
        rejected_bad_config = true;
    }
    RB_CHECK(rejected_bad_config);
    return true;
}

}  // namespace

int main() {
    if (!testExcessForceProducesOnlyOutwardCorrection()) return 1;
    if (!testContinuousDeadband()) return 1;
    if (!testDynamicAndUnilateralBoundsDoNotWindUp()) return 1;
    if (!testEnergyLimitRejectsWithoutMutation()) return 1;
    if (!testTwoPhaseLifecycleAndProvenance()) return 1;
    if (!testInvalidInputsAndConfigFailClosed()) return 1;
    std::cout << "normal_force_controller tests passed\n";
    return 0;
}
