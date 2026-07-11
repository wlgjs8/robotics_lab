#include <cmath>
#include <iostream>

#include "rb_servo/control/force_controller.hpp"

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

rb_servo::ForceControlConfig controllerConfig() {
    rb_servo::ForceControlConfig config;
    config.enable = true;
    config.virtual_mass = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    config.damping = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    config.stiffness = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    config.max_dt_sec = 0.02;
    config.max_linear_velocity_m_s = 10.0;
    config.max_angular_velocity_rad_s = 10.0;
    config.max_linear_acceleration_m_s2 = 100.0;
    config.max_angular_acceleration_rad_s2 = 100.0;
    config.max_linear_jerk_m_s3 = 10000.0;
    config.max_angular_jerk_rad_s3 = 10000.0;
    config.max_pos_offset_m = 1.0;
    config.max_rot_offset_rad = 1.0;
    config.max_pos_step_m = 1.0;
    config.max_rot_step_rad = 1.0;
    config.max_energy_j = 10.0;
    return config;
}

rb_servo::ForceControlCommand xCommand() {
    rb_servo::ForceControlCommand command;
    command.mode = rb_servo::ForceControlMode::Admittance;
    command.enabled_axis.x = true;
    return command;
}

rb_servo::ForceControlConfig realProfileConfig() {
    rb_servo::ForceControlConfig config = controllerConfig();
    config.virtual_mass = {8.0, 8.0, 8.0, 0.8, 0.8, 0.8};
    config.damping = {100.0, 100.0, 100.0, 10.0, 10.0, 10.0};
    config.stiffness = {300.0, 300.0, 300.0, 30.0, 30.0, 30.0};
    config.wrench_deadband = {3.0, 3.0, 3.0, 0.6, 0.6, 0.6};
    config.max_pos_offset_m = 0.01;
    config.max_rot_offset_rad = 0.035;
    config.max_linear_velocity_m_s = 0.015;
    config.max_angular_velocity_rad_s = 0.05;
    config.max_linear_acceleration_m_s2 = 0.12;
    config.max_angular_acceleration_rad_s2 = 0.5;
    config.max_linear_jerk_m_s3 = 0.8;
    config.max_angular_jerk_rad_s3 = 3.0;
    config.max_pos_step_m = 0.001;
    config.max_rot_step_rad = 0.001;
    config.max_energy_j = 100.0;
    return config;
}

bool testProposeCommitAndReject() {
    rb_servo::ForceController controller(controllerConfig());
    controller.engage();
    rb_servo::Wrench6D wrench;
    wrench.fx = 10.0;
    const auto proposal = controller.propose(wrench, xCommand(), rb_servo::Vec6{}, 0.01);
    RB_CHECK(proposal.valid);
    RB_CHECK(near(proposal.state.acceleration_tcp.x, -10.0));
    RB_CHECK(near(proposal.state.velocity_tcp.x, -0.1));
    RB_CHECK(near(proposal.state.offset_tcp.x, -0.001));
    RB_CHECK(near(proposal.state.offset_tcp.y, 0.0));
    RB_CHECK(near(controller.state().offset_tcp.x, 0.0));

    controller.reject();
    RB_CHECK(near(controller.state().offset_tcp.x, 0.0));
    RB_CHECK(controller.commit(proposal));
    RB_CHECK(near(controller.state().offset_tcp.x, -0.001));
    RB_CHECK(!controller.commit(proposal));
    return true;
}

bool testCommandLimitsOnlyTightenServerLimits() {
    rb_servo::ForceControlConfig config = controllerConfig();
    config.max_pos_step_m = 0.001;
    rb_servo::ForceController controller(config);
    controller.engage();
    rb_servo::Wrench6D wrench;
    wrench.fx = 100.0;

    rb_servo::ForceControlCommand larger = xCommand();
    larger.max_pos_step_m = 0.1;
    const auto server_bounded = controller.propose(wrench, larger, rb_servo::Vec6{}, 0.01);
    RB_CHECK(server_bounded.valid);
    RB_CHECK(server_bounded.state.offset_tcp.x < -0.0009);
    RB_CHECK(std::abs(server_bounded.state.offset_tcp.x) <= 0.001 + 1e-12);

    rb_servo::ForceControlCommand tighter = xCommand();
    tighter.max_pos_step_m = 0.0002;
    const auto command_bounded = controller.propose(wrench, tighter, rb_servo::Vec6{}, 0.01);
    RB_CHECK(command_bounded.valid);
    RB_CHECK(command_bounded.state.offset_tcp.x < -0.00018);
    RB_CHECK(std::abs(command_bounded.state.offset_tcp.x) <= 0.0002 + 1e-12);
    return true;
}

bool testVariableDtAndDynamicBounds() {
    rb_servo::ForceControlConfig config = controllerConfig();
    config.max_linear_acceleration_m_s2 = 2.0;
    config.max_linear_jerk_m_s3 = 10.0;
    config.max_linear_velocity_m_s = 0.01;
    config.max_pos_step_m = 0.00005;
    rb_servo::ForceController controller(config);
    controller.engage();
    rb_servo::Wrench6D wrench;
    wrench.fx = 100.0;

    const auto bounded = controller.propose(wrench, xCommand(), rb_servo::Vec6{}, 0.01);
    RB_CHECK(bounded.valid);
    RB_CHECK(bounded.saturated);
    RB_CHECK(std::abs(bounded.state.acceleration_tcp.x) <= 0.1 + 1e-12);
    RB_CHECK(std::abs(bounded.state.velocity_tcp.x) <= 0.01 + 1e-12);
    RB_CHECK(std::abs(bounded.state.offset_tcp.x) <= 0.00005 + 1e-12);

    RB_CHECK(!controller.propose(wrench, xCommand(), rb_servo::Vec6{}, 0.0).valid);
    RB_CHECK(!controller.propose(wrench, xCommand(), rb_servo::Vec6{}, 0.03).valid);
    return true;
}

bool testPassivityObserverRejectsExcessEnergy() {
    rb_servo::ForceControlConfig config = controllerConfig();
    config.max_energy_j = 0.05;
    rb_servo::ForceController controller(config);
    controller.engage();
    rb_servo::Wrench6D wrench;
    wrench.fx = 10.0;
    rb_servo::Vec6 actual_twist;
    actual_twist.x = 1.0;
    const auto proposal = controller.propose(wrench, xCommand(), actual_twist, 0.01);
    RB_CHECK(!proposal.valid);
    RB_CHECK(!controller.commit(proposal));
    RB_CHECK(near(controller.state().observed_energy_j, 0.0));
    return true;
}

bool testResetAndInactiveModes() {
    rb_servo::ForceController controller(controllerConfig());
    RB_CHECK(!controller.engaged());
    controller.engage();
    RB_CHECK(controller.engaged());
    rb_servo::Wrench6D wrench;
    wrench.fx = 10.0;
    const auto active = controller.propose(wrench, xCommand(), rb_servo::Vec6{}, 0.01);
    RB_CHECK(controller.commit(active));
    controller.release();
    RB_CHECK(!controller.engaged());
    RB_CHECK(near(controller.state().offset_tcp.x, 0.0));

    rb_servo::ForceControlCommand off;
    RB_CHECK(!controller.propose(wrench, off, rb_servo::Vec6{}, 0.01).valid);
    controller.engage();
    controller.reset();
    RB_CHECK(!controller.engaged());
    return true;
}

bool testZeroErrorJitterAndSaturationDoNotWindUp() {
    rb_servo::ForceControlConfig config = controllerConfig();
    rb_servo::ForceController controller(config);
    controller.engage();

    const auto zero = controller.propose(
        rb_servo::Wrench6D{},
        xCommand(),
        rb_servo::Vec6{},
        0.002
    );
    RB_CHECK(zero.valid);
    RB_CHECK(near(zero.state.offset_tcp.x, 0.0));
    RB_CHECK(near(zero.state.velocity_tcp.x, 0.0));

    rb_servo::Wrench6D wrench;
    wrench.fx = 100.0;
    for (double dt : {0.0015, 0.002, 0.003, 0.001}) {
        const auto proposal = controller.propose(wrench, xCommand(), rb_servo::Vec6{}, dt);
        RB_CHECK(proposal.valid);
        RB_CHECK(std::isfinite(proposal.state.offset_tcp.x));
        RB_CHECK(std::abs(proposal.state.offset_tcp.x) <= config.max_pos_offset_m + 1e-12);
        RB_CHECK(controller.commit(proposal));
    }

    rb_servo::ForceControlConfig bounded_config = controllerConfig();
    bounded_config.max_pos_offset_m = 0.001;
    bounded_config.max_pos_step_m = 0.001;
    bounded_config.max_linear_velocity_m_s = 0.02;
    bounded_config.max_linear_acceleration_m_s2 = 0.2;
    bounded_config.max_linear_jerk_m_s3 = 2.0;
    rb_servo::ForceController bounded_controller(bounded_config);
    bounded_controller.engage();
    for (int i = 0; i < 1000; ++i) {
        const auto proposal = bounded_controller.propose(
            wrench,
            xCommand(),
            rb_servo::Vec6{},
            0.01
        );
        if (!proposal.valid) {
            const auto& state = bounded_controller.state();
            std::cerr << "bounded steady i=" << i
                      << " x=" << state.offset_tcp.x
                      << " v=" << state.velocity_tcp.x
                      << " a=" << state.acceleration_tcp.x
                      << ": " << proposal.reason << "\n";
        }
        RB_CHECK(proposal.valid);
        RB_CHECK(bounded_controller.commit(proposal));
    }
    RB_CHECK(bounded_controller.state().offset_tcp.x <
             -0.94 * bounded_config.max_pos_offset_m);
    RB_CHECK(std::abs(bounded_controller.state().offset_tcp.x) <=
             bounded_config.max_pos_offset_m + 1e-12);
    RB_CHECK(near(bounded_controller.state().velocity_tcp.x, 0.0));
    return true;
}

bool testEveryValidProposalHonorsFiniteDifferenceDynamics() {
    rb_servo::ForceControlConfig config = controllerConfig();
    config.max_pos_offset_m = 0.001;
    config.max_pos_step_m = 0.001;
    config.max_linear_velocity_m_s = 0.02;
    config.max_linear_acceleration_m_s2 = 0.2;
    config.max_linear_jerk_m_s3 = 2.0;
    rb_servo::ForceController controller(config);
    controller.engage();

    rb_servo::Wrench6D wrench;
    wrench.fx = 100.0;
    bool reached_hard_boundary = false;
    for (int i = 0; i < 1000; ++i) {
        const rb_servo::ForceControllerState previous = controller.state();
        const double dt = 0.01;
        const auto proposal = controller.propose(wrench, xCommand(), rb_servo::Vec6{}, dt);
        RB_CHECK(proposal.valid);
        const double realized_acceleration =
            (proposal.state.velocity_tcp.x - previous.velocity_tcp.x) / dt;
        const double realized_jerk =
            (proposal.state.acceleration_tcp.x - previous.acceleration_tcp.x) / dt;
        RB_CHECK(
            std::abs(realized_acceleration) <=
            config.max_linear_acceleration_m_s2 + 1e-12
        );
        RB_CHECK(std::abs(realized_jerk) <= config.max_linear_jerk_m_s3 + 1e-12);
        RB_CHECK(near(proposal.state.acceleration_tcp.x, realized_acceleration));
        RB_CHECK(controller.commit(proposal));
        if (proposal.saturated) reached_hard_boundary = true;
    }
    RB_CHECK(reached_hard_boundary);
    return true;
}

bool testRealProfileVelocityBoundaryBrakesWithoutFault() {
    rb_servo::ForceControlConfig config = realProfileConfig();

    rb_servo::ForceController controller(config);
    controller.engage();
    rb_servo::Wrench6D wrench;
    wrench.fx = 9.75;
    constexpr double dt = 0.002;
    double peak_speed = 0.0;
    bool bounded = false;
    for (int i = 0; i < 2000; ++i) {
        const rb_servo::ForceControllerState previous = controller.state();
        const auto proposal = controller.propose(wrench, xCommand(), rb_servo::Vec6{}, dt);
        if (!proposal.valid) {
            std::cerr << "real-profile load i=" << i
                      << " x=" << previous.offset_tcp.x
                      << " v=" << previous.velocity_tcp.x
                      << " a=" << previous.acceleration_tcp.x
                      << ": " << proposal.reason << "\n";
        }
        RB_CHECK(proposal.valid);
        RB_CHECK(std::abs(proposal.state.offset_tcp.x) <= config.max_pos_offset_m + 1e-9);
        RB_CHECK(std::abs(proposal.state.velocity_tcp.x) <=
                 config.max_linear_velocity_m_s + 1e-9);
        RB_CHECK(std::abs(proposal.state.acceleration_tcp.x) <=
                 config.max_linear_acceleration_m_s2 + 1e-9);
        RB_CHECK(std::abs(
            proposal.state.acceleration_tcp.x - previous.acceleration_tcp.x
        ) / dt <= config.max_linear_jerk_m_s3 + 1e-6);
        if (proposal.saturated) {
            bounded = true;
            RB_CHECK(proposal.limit_axes[0]);
            RB_CHECK(proposal.limit_reason == "jerk_limited_motion_envelope");
        }
        peak_speed = std::max(peak_speed, std::abs(proposal.state.velocity_tcp.x));
        RB_CHECK(controller.commit(proposal));
    }
    RB_CHECK(bounded);
    RB_CHECK(peak_speed > 0.014);

    for (int i = 0; i < 3000; ++i) {
        const rb_servo::ForceControllerState previous = controller.state();
        const auto proposal = controller.propose(
            rb_servo::Wrench6D{}, xCommand(), rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        RB_CHECK(std::abs(
            proposal.state.acceleration_tcp.x - previous.acceleration_tcp.x
        ) / dt <= config.max_linear_jerk_m_s3 + 1e-6);
        RB_CHECK(controller.commit(proposal));
    }
    RB_CHECK(std::abs(controller.state().offset_tcp.x) < 1e-4);
    RB_CHECK(std::abs(controller.state().velocity_tcp.x) < 1e-4);
    return true;
}

bool testRealProfileAllCartesianAxesStayBoundedWithoutFault() {
    const rb_servo::ForceControlConfig config = realProfileConfig();
    rb_servo::ForceController controller(config);
    controller.engage();

    rb_servo::ForceControlCommand command;
    command.mode = rb_servo::ForceControlMode::Admittance;
    command.enabled_axis = {true, true, true, true, true, true};
    rb_servo::Wrench6D wrench{10.0, -10.0, 10.0, 2.0, -2.0, 2.0};
    constexpr double dt = 0.002;
    std::array<bool, 6> observed_limit{};
    for (int i = 0; i < 2000; ++i) {
        const auto proposal = controller.propose(wrench, command, rb_servo::Vec6{}, dt);
        RB_CHECK(proposal.valid);
        RB_CHECK(std::abs(proposal.state.offset_tcp.x) <= config.max_pos_offset_m + 1e-9);
        RB_CHECK(std::abs(proposal.state.offset_tcp.y) <= config.max_pos_offset_m + 1e-9);
        RB_CHECK(std::abs(proposal.state.offset_tcp.z) <= config.max_pos_offset_m + 1e-9);
        RB_CHECK(std::abs(proposal.state.offset_tcp.rx) <= config.max_rot_offset_rad + 1e-9);
        RB_CHECK(std::abs(proposal.state.offset_tcp.ry) <= config.max_rot_offset_rad + 1e-9);
        RB_CHECK(std::abs(proposal.state.offset_tcp.rz) <= config.max_rot_offset_rad + 1e-9);
        for (std::size_t axis = 0; axis < observed_limit.size(); ++axis) {
            observed_limit[axis] = observed_limit[axis] || proposal.limit_axes[axis];
        }
        RB_CHECK(controller.commit(proposal));
    }
    for (bool limited : observed_limit) RB_CHECK(limited);
    return true;
}

bool testRealProfileReleaseSweepRemainsRecursivelyFeasible() {
    const rb_servo::ForceControlConfig config = realProfileConfig();
    constexpr double dt = 0.002;
    for (double force_n : {-30.0, -18.0, -10.0, 10.0, 18.0, 30.0}) {
        for (int load_ticks : {25, 75, 150, 300, 600, 1200, 2400}) {
            rb_servo::ForceController controller(config);
            controller.engage();
            rb_servo::Wrench6D wrench;
            wrench.fx = force_n;
            for (int i = 0; i < load_ticks; ++i) {
                const auto proposal = controller.propose(
                    wrench, xCommand(), rb_servo::Vec6{}, dt
                );
                if (!proposal.valid) {
                    std::cerr << "release sweep load force=" << force_n
                              << " ticks=" << load_ticks << " i=" << i
                              << ": " << proposal.reason << "\n";
                }
                RB_CHECK(proposal.valid);
                RB_CHECK(controller.commit(proposal));
            }
            for (int i = 0; i < 5000; ++i) {
                const auto proposal = controller.propose(
                    rb_servo::Wrench6D{}, xCommand(), rb_servo::Vec6{}, dt
                );
                if (!proposal.valid) {
                    const auto& state = controller.state();
                    std::cerr << "release sweep recenter force=" << force_n
                              << " ticks=" << load_ticks << " i=" << i
                              << " x=" << state.offset_tcp.x
                              << " v=" << state.velocity_tcp.x
                              << " a=" << state.acceleration_tcp.x
                              << ": " << proposal.reason << "\n";
                }
                RB_CHECK(proposal.valid);
                RB_CHECK(controller.commit(proposal));
            }
        }
    }
    return true;
}

bool testProposalProvenancePreventsStaleOrCrossControllerCommit() {
    rb_servo::ForceController first(controllerConfig());
    rb_servo::ForceController second(controllerConfig());
    first.engage();
    second.engage();
    rb_servo::Wrench6D wrench;
    wrench.fx = 1.0;

    const auto first_proposal = first.propose(wrench, xCommand(), rb_servo::Vec6{}, 0.01);
    RB_CHECK(first_proposal.valid);
    RB_CHECK(!second.commit(first_proposal));

    first.release();
    first.engage();
    RB_CHECK(!first.commit(first_proposal));
    RB_CHECK(near(first.state().offset_tcp.x, 0.0));
    return true;
}

bool testWrenchDeadbandSuppressesNoiseAndPreservesExcessSign() {
    rb_servo::ForceControlConfig config = controllerConfig();
    config.wrench_deadband = {2.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    rb_servo::ForceController controller(config);
    controller.engage();

    rb_servo::Wrench6D quiet;
    quiet.fx = 1.9;
    const auto quiet_proposal = controller.propose(
        quiet, xCommand(), rb_servo::Vec6{}, 0.01
    );
    RB_CHECK(quiet_proposal.valid);
    RB_CHECK(near(quiet_proposal.wrench_error_tcp.fx, 0.0));
    RB_CHECK(near(quiet_proposal.state.offset_tcp.x, 0.0));

    rb_servo::Wrench6D loaded;
    loaded.fx = 5.0;
    const auto loaded_proposal = controller.propose(
        loaded, xCommand(), rb_servo::Vec6{}, 0.01
    );
    RB_CHECK(loaded_proposal.valid);
    RB_CHECK(near(loaded_proposal.wrench_error_tcp.fx, -3.0));
    RB_CHECK(loaded_proposal.state.offset_tcp.x < 0.0);
    return true;
}

}  // namespace

int main() {
    if (!testProposeCommitAndReject()) return 1;
    if (!testCommandLimitsOnlyTightenServerLimits()) return 1;
    if (!testVariableDtAndDynamicBounds()) return 1;
    if (!testPassivityObserverRejectsExcessEnergy()) return 1;
    if (!testResetAndInactiveModes()) return 1;
    if (!testZeroErrorJitterAndSaturationDoNotWindUp()) return 1;
    if (!testEveryValidProposalHonorsFiniteDifferenceDynamics()) return 1;
    if (!testRealProfileVelocityBoundaryBrakesWithoutFault()) return 1;
    if (!testRealProfileAllCartesianAxesStayBoundedWithoutFault()) return 1;
    if (!testRealProfileReleaseSweepRemainsRecursivelyFeasible()) return 1;
    if (!testProposalProvenancePreventsStaleOrCrossControllerCommit()) return 1;
    if (!testWrenchDeadbandSuppressesNoiseAndPreservesExcessSign()) return 1;
    std::cout << "force_controller tests passed\n";
    return 0;
}
