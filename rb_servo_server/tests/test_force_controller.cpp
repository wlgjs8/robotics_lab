#include <cmath>
#include <iostream>

#include "rb_servo/control/force_controller.hpp"
#include "rb_servo/math/se3.hpp"

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
    config.virtual_mass = {2.0, 2.0, 2.0, 0.2, 0.2, 0.2};
    config.damping = {26.0, 26.0, 26.0, 1.55, 1.55, 1.55};
    config.stiffness = {80.0, 80.0, 80.0, 3.0, 3.0, 3.0};
    config.wrench_deadband = {1.5, 1.5, 1.5, 0.10, 0.10, 0.10};
    config.blockwise_release_recenter = true;
    config.max_pos_offset_m = 0.02;
    config.max_rot_offset_rad = 0.08;
    config.max_linear_velocity_m_s = 0.03;
    config.max_angular_velocity_rad_s = 0.15;
    config.max_linear_acceleration_m_s2 = 0.25;
    config.max_angular_acceleration_rad_s2 = 1.5;
    config.max_linear_jerk_m_s3 = 2.0;
    config.max_angular_jerk_rad_s3 = 10.0;
    config.max_pos_step_m = 0.001;
    config.max_rot_step_rad = 0.002;
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
        if (!proposal.valid) {
            std::cerr << "real-profile release i=" << i
                      << " x=" << previous.offset_tcp.x
                      << " v=" << previous.velocity_tcp.x
                      << " a=" << previous.acceleration_tcp.x
                      << ": " << proposal.reason << "\n";
        }
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

bool testRealProfileRotationsShareSensitivityBelowContactTelemetryThreshold() {
    const rb_servo::ForceControlConfig config = realProfileConfig();
    rb_servo::ForceController controller(config);
    controller.engage();

    rb_servo::ForceControlCommand command;
    command.mode = rb_servo::ForceControlMode::Admittance;
    command.enabled_axis = {false, false, false, true, true, true};
    rb_servo::Wrench6D wrench;
    wrench.tx = 0.15;
    wrench.ty = 0.15;
    wrench.tz = 0.15;

    const auto proposal = controller.propose(wrench, command, rb_servo::Vec6{}, 0.002);
    RB_CHECK(proposal.valid);
    RB_CHECK(near(proposal.wrench_error_tcp.tx, -0.05));
    RB_CHECK(near(proposal.wrench_error_tcp.ty, -0.05));
    RB_CHECK(near(proposal.wrench_error_tcp.tz, -0.05));
    RB_CHECK(proposal.state.offset_tcp.rx < 0.0);
    RB_CHECK(proposal.state.offset_tcp.ry < 0.0);
    RB_CHECK(proposal.state.offset_tcp.rz < 0.0);
    return true;
}

bool testBlockwiseReleaseDefersSiblingAndPreservesReturnDirection() {
    const rb_servo::ForceControlConfig config = realProfileConfig();
    rb_servo::ForceController controller(config);
    controller.engage();

    rb_servo::ForceControlCommand command;
    command.mode = rb_servo::ForceControlMode::Admittance;
    command.enabled_axis = {true, true, false, false, false, false};
    constexpr double dt = 0.002;

    rb_servo::Wrench6D loaded;
    loaded.fx = 3.0;
    loaded.fy = 4.0;
    for (int i = 0; i < 2500; ++i) {
        const auto proposal = controller.propose(
            loaded, command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        RB_CHECK(controller.commit(proposal));
    }

    rb_servo::Wrench6D y_only;
    y_only.fy = loaded.fy;
    const double x_at_partial_release = controller.state().offset_tcp.x;
    bool observed_deferred = false;
    for (int i = 0; i < 1000; ++i) {
        const auto proposal = controller.propose(
            y_only, command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        observed_deferred = observed_deferred ||
            proposal.translation_recenter_deferred;
        RB_CHECK(controller.commit(proposal));
    }
    RB_CHECK(observed_deferred);
    RB_CHECK(std::abs(controller.state().offset_tcp.x - x_at_partial_release) < 1e-5);

    const double release_x = controller.state().offset_tcp.x;
    const double release_y = controller.state().offset_tcp.y;
    const double release_norm = std::hypot(release_x, release_y);
    RB_CHECK(release_norm > 0.005);
    bool observed_coupled = false;
    double minimum_direction_cosine = 1.0;
    for (int i = 0; i < 5000; ++i) {
        const auto proposal = controller.propose(
            rb_servo::Wrench6D{}, command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        observed_coupled = observed_coupled ||
            proposal.translation_recenter_coupled;
        const double x = proposal.state.offset_tcp.x;
        const double y = proposal.state.offset_tcp.y;
        const double norm = std::hypot(x, y);
        if (norm > 0.001) {
            minimum_direction_cosine = std::min(
                minimum_direction_cosine,
                (x * release_x + y * release_y) / (norm * release_norm)
            );
        }
        RB_CHECK(controller.commit(proposal));
    }
    RB_CHECK(observed_coupled);
    // Hard-envelope recovery is allowed to deviate briefly from the common
    // direction; normal soft-envelope recentering remains coupled.
    RB_CHECK(minimum_direction_cosine > 0.995);
    RB_CHECK(std::hypot(
        controller.state().offset_tcp.x,
        controller.state().offset_tcp.y
    ) < 1e-4);

    rb_servo::ForceController rotation_controller(config);
    rotation_controller.engage();
    rb_servo::ForceControlCommand rotation_command;
    rotation_command.mode = rb_servo::ForceControlMode::Admittance;
    rotation_command.enabled_axis = {false, false, false, true, true, false};
    rb_servo::Wrench6D rotation_load;
    rotation_load.tx = 0.3;
    rotation_load.ty = 0.5;
    for (int i = 0; i < 2500; ++i) {
        const auto proposal = rotation_controller.propose(
            rotation_load, rotation_command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        RB_CHECK(rotation_controller.commit(proposal));
    }

    rb_servo::Wrench6D pitch_only;
    pitch_only.ty = rotation_load.ty;
    const double roll_at_partial_release =
        rotation_controller.state().offset_tcp.rx;
    bool observed_rotation_deferred = false;
    for (int i = 0; i < 1000; ++i) {
        const auto proposal = rotation_controller.propose(
            pitch_only, rotation_command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        observed_rotation_deferred = observed_rotation_deferred ||
            proposal.rotation_recenter_deferred;
        RB_CHECK(rotation_controller.commit(proposal));
    }
    RB_CHECK(observed_rotation_deferred);
    RB_CHECK(std::abs(
        rotation_controller.state().offset_tcp.rx - roll_at_partial_release
    ) < 1e-5);

    const double release_roll = rotation_controller.state().offset_tcp.rx;
    const double release_pitch = rotation_controller.state().offset_tcp.ry;
    const double rotation_release_norm = std::hypot(release_roll, release_pitch);
    RB_CHECK(rotation_release_norm > 0.01);
    bool observed_rotation_coupled = false;
    double minimum_rotation_direction_cosine = 1.0;
    for (int i = 0; i < 5000; ++i) {
        const auto proposal = rotation_controller.propose(
            rb_servo::Wrench6D{}, rotation_command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        observed_rotation_coupled = observed_rotation_coupled ||
            proposal.rotation_recenter_coupled;
        const double roll = proposal.state.offset_tcp.rx;
        const double pitch = proposal.state.offset_tcp.ry;
        const double norm = std::hypot(roll, pitch);
        if (norm > 0.002) {
            minimum_rotation_direction_cosine = std::min(
                minimum_rotation_direction_cosine,
                (roll * release_roll + pitch * release_pitch) /
                    (norm * rotation_release_norm)
            );
        }
        RB_CHECK(rotation_controller.commit(proposal));
    }
    RB_CHECK(observed_rotation_coupled);
    RB_CHECK(minimum_rotation_direction_cosine > 0.999);
    RB_CHECK(std::hypot(
        rotation_controller.state().offset_tcp.rx,
        rotation_controller.state().offset_tcp.ry
    ) < 1e-4);
    return true;
}

bool testBlockwiseReleaseKeepsSingleAxisGovernorWithAllAxesEnabled() {
    const rb_servo::ForceControlConfig config = realProfileConfig();
    rb_servo::ForceController controller(config);
    controller.engage();
    rb_servo::ForceControlCommand command;
    command.mode = rb_servo::ForceControlMode::Admittance;
    command.enabled_axis = {true, true, true, true, true, true};
    constexpr double dt = 0.002;

    rb_servo::Wrench6D wrench;
    wrench.fx = 9.75;
    for (int i = 0; i < 2000; ++i) {
        const auto proposal = controller.propose(
            wrench, command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        RB_CHECK(controller.commit(proposal));
    }
    for (int i = 0; i < 3000; ++i) {
        const auto proposal = controller.propose(
            rb_servo::Wrench6D{}, command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        RB_CHECK(!proposal.translation_recenter_coupled);
        RB_CHECK(controller.commit(proposal));
    }
    RB_CHECK(std::abs(controller.state().offset_tcp.x) < 1e-4);
    return true;
}

bool testBlockwiseReleasePreservesHardEnvelopeRecoveryJerk() {
    const rb_servo::ForceControlConfig config = realProfileConfig();
    rb_servo::ForceController controller(config);
    controller.engage();
    rb_servo::ForceControlCommand command;
    command.mode = rb_servo::ForceControlMode::Admittance;
    command.enabled_axis = {true, true, true, true, true, true};
    constexpr double dt = 0.002;

    struct TranslationPhase {
        int ticks;
        double fx;
        double fy;
        double fz;
    };
    // Deterministic reduction of the multi-axis load/release sequence that
    // reproduced the 2026-07-12 real-log latch. During the final release, one
    // axis leaves the soft recursively feasible set. Common recenter scaling
    // must not overwrite that axis' hard-envelope recovery jerk.
    constexpr TranslationPhase phases[] = {
        {423, -2.5, 7.5, 7.5},
        {457, -7.5, -2.5, 5.0},
        {153, 0.0, 0.0, 2.5},
        {195, 0.0, 0.0, 0.0},
    };

    bool observed_coupled_recenter = false;
    bool observed_recovery_fallback = false;
    for (std::size_t phase_index = 0;
         phase_index < std::size(phases);
         ++phase_index) {
        const TranslationPhase& phase = phases[phase_index];
        rb_servo::Wrench6D wrench;
        wrench.fx = phase.fx;
        wrench.fy = phase.fy;
        wrench.fz = phase.fz;
        for (int tick = 0; tick < phase.ticks; ++tick) {
            const auto proposal = controller.propose(
                wrench, command, rb_servo::Vec6{}, dt
            );
            RB_CHECK(proposal.valid);
            if (phase_index + 1 == std::size(phases)) {
                observed_coupled_recenter = observed_coupled_recenter ||
                    proposal.translation_recenter_coupled;
                observed_recovery_fallback = observed_recovery_fallback ||
                    (observed_coupled_recenter &&
                     !proposal.translation_recenter_coupled &&
                     proposal.saturated);
            }
            RB_CHECK(controller.commit(proposal));
        }
    }
    RB_CHECK(observed_coupled_recenter);
    RB_CHECK(observed_recovery_fallback);
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

bool testResponsiveRealProfileMovesOnWeakWrenchAndUsesExpandedTravel() {
    const rb_servo::ForceControlConfig config = realProfileConfig();
    rb_servo::ForceController controller(config);
    controller.engage();
    constexpr double dt = 0.002;

    rb_servo::Wrench6D weak_wrench;
    weak_wrench.fx = 2.0;
    for (int i = 0; i < 125; ++i) {
        const auto proposal = controller.propose(
            weak_wrench, xCommand(), rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        RB_CHECK(controller.commit(proposal));
    }
    RB_CHECK(controller.state().offset_tcp.x < -0.0005);

    rb_servo::Wrench6D guiding_wrench;
    guiding_wrench.fx = 3.0;
    for (int i = 0; i < 2000; ++i) {
        const auto proposal = controller.propose(
            guiding_wrench, xCommand(), rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        RB_CHECK(controller.commit(proposal));
    }
    RB_CHECK(controller.state().offset_tcp.x < -0.015);
    RB_CHECK(std::abs(controller.state().offset_tcp.x) <=
             config.max_pos_offset_m + 1e-9);
    return true;
}

bool testRealProfileSustainedWrenchNeverReturnsTowardZero() {
    const rb_servo::ForceControlConfig config = realProfileConfig();
    rb_servo::ForceController controller(config);
    controller.engage();
    constexpr double dt = 0.002;

    rb_servo::ForceControlCommand command;
    command.mode = rb_servo::ForceControlMode::Admittance;
    command.enabled_axis.y = true;
    command.enabled_axis.roll = true;

    // These loads exceed the configured deadbands and drive both axes into the
    // real profile's bounded travel envelope. While the same-direction wrench
    // remains active, neither axis may move back toward the zero-offset command
    // equilibrium. Recentring is reserved for the zero-wrench phase.
    rb_servo::Wrench6D wrench;
    wrench.fy = 6.0;
    wrench.tx = 1.2;
    double furthest_y = 0.0;
    double furthest_rx = 0.0;
    double max_y_return = 0.0;
    double max_rx_return = 0.0;
    for (int i = 0; i < 4000; ++i) {
        const auto proposal = controller.propose(
            wrench, command, rb_servo::Vec6{}, dt
        );
        if (!proposal.valid) {
            std::cerr << "sustained-wrench hold i=" << i
                      << ": " << proposal.reason << "\n";
        }
        RB_CHECK(proposal.valid);
        RB_CHECK(proposal.wrench_error_tcp.fy < 0.0);
        RB_CHECK(proposal.wrench_error_tcp.tx < 0.0);
        furthest_y = std::min(furthest_y, proposal.state.offset_tcp.y);
        furthest_rx = std::min(furthest_rx, proposal.state.offset_tcp.rx);
        max_y_return = std::max(
            max_y_return, proposal.state.offset_tcp.y - furthest_y
        );
        max_rx_return = std::max(
            max_rx_return, proposal.state.offset_tcp.rx - furthest_rx
        );
        RB_CHECK(controller.commit(proposal));
    }
    RB_CHECK(controller.state().offset_tcp.y < -0.015);
    RB_CHECK(controller.state().offset_tcp.rx < -0.06);
    if (max_y_return > 1e-12 || max_rx_return > 1e-12) {
        std::cerr << "sustained return y=" << max_y_return
                  << " rx=" << max_rx_return << "\n";
    }
    RB_CHECK(max_y_return <= 1e-12);
    RB_CHECK(max_rx_return <= 1e-12);

    // The weak loads leave only 0.5 N / 0.1 Nm after deadband removal. Their
    // corresponding spring terms are larger at the displaced pose, so the
    // unconstrained SMD equation would recenter despite continued contact.
    // Loaded hold must instead keep the furthest reached position.
    wrench.fy = 2.0;
    wrench.tx = 0.35;
    furthest_y = controller.state().offset_tcp.y;
    furthest_rx = controller.state().offset_tcp.rx;
    for (int i = 0; i < 1500; ++i) {
        const auto proposal = controller.propose(
            wrench, command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        RB_CHECK(proposal.wrench_error_tcp.fy < 0.0);
        RB_CHECK(proposal.wrench_error_tcp.tx < 0.0);
        furthest_y = std::min(furthest_y, proposal.state.offset_tcp.y);
        furthest_rx = std::min(furthest_rx, proposal.state.offset_tcp.rx);
        RB_CHECK(proposal.state.offset_tcp.y <= furthest_y + 1e-12);
        RB_CHECK(proposal.state.offset_tcp.rx <= furthest_rx + 1e-12);
        RB_CHECK(controller.commit(proposal));
    }
    RB_CHECK(std::abs(controller.state().velocity_tcp.y) < 1e-9);
    RB_CHECK(std::abs(controller.state().velocity_tcp.rx) < 1e-9);
    RB_CHECK(std::abs(controller.state().acceleration_tcp.y) < 1e-9);
    RB_CHECK(std::abs(controller.state().acceleration_tcp.rx) < 1e-9);

    for (int i = 0; i < 3000; ++i) {
        const auto proposal = controller.propose(
            rb_servo::Wrench6D{}, command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        RB_CHECK(controller.commit(proposal));
    }
    RB_CHECK(std::abs(controller.state().offset_tcp.y) < 1e-4);
    RB_CHECK(std::abs(controller.state().offset_tcp.rx) < 1e-4);

    // Exercise the mirrored positive-offset transform as well.
    rb_servo::ForceController mirrored(config);
    mirrored.engage();
    wrench.fy = -6.0;
    wrench.tx = -1.2;
    for (int i = 0; i < 4000; ++i) {
        const auto proposal = mirrored.propose(
            wrench, command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        RB_CHECK(mirrored.commit(proposal));
    }
    RB_CHECK(mirrored.state().offset_tcp.y > 0.015);
    RB_CHECK(mirrored.state().offset_tcp.rx > 0.06);
    wrench.fy = -2.0;
    wrench.tx = -0.35;
    double positive_y = mirrored.state().offset_tcp.y;
    double positive_rx = mirrored.state().offset_tcp.rx;
    for (int i = 0; i < 1500; ++i) {
        const auto proposal = mirrored.propose(
            wrench, command, rb_servo::Vec6{}, dt
        );
        RB_CHECK(proposal.valid);
        positive_y = std::max(positive_y, proposal.state.offset_tcp.y);
        positive_rx = std::max(positive_rx, proposal.state.offset_tcp.rx);
        RB_CHECK(proposal.state.offset_tcp.y >= positive_y - 1e-12);
        RB_CHECK(proposal.state.offset_tcp.rx >= positive_rx - 1e-12);
        RB_CHECK(mirrored.commit(proposal));
    }
    return true;
}

bool testRealProfileRecontactWhileRecenteringBrakesWithoutFault() {
    const rb_servo::ForceControlConfig config = realProfileConfig();
    constexpr double dt = 0.002;
    for (double measured_sign : {-1.0, 1.0}) {
        const double correction_direction = -measured_sign;
        for (int release_ticks : {10, 50, 100, 200}) {
            rb_servo::ForceController controller(config);
            controller.engage();

            rb_servo::Wrench6D strong_load;
            strong_load.fx = measured_sign * 6.0;
            for (int i = 0; i < 2500; ++i) {
                const auto proposal = controller.propose(
                    strong_load, xCommand(), rb_servo::Vec6{}, dt
                );
                RB_CHECK(proposal.valid);
                RB_CHECK(controller.commit(proposal));
            }
            RB_CHECK(
                correction_direction * controller.state().offset_tcp.x > 0.015
            );

            // Begin the stiffness-driven return, then restore a weaker contact
            // while velocity still points toward zero. The controller cannot
            // reverse that velocity in one jerk-limited tick; recontact must
            // remain proposal-valid while the return branch brakes it.
            for (int i = 0; i < release_ticks; ++i) {
                const auto proposal = controller.propose(
                    rb_servo::Wrench6D{}, xCommand(), rb_servo::Vec6{}, dt
                );
                RB_CHECK(proposal.valid);
                RB_CHECK(controller.commit(proposal));
            }
            RB_CHECK(
                correction_direction * controller.state().offset_tcp.x > 0.0
            );
            RB_CHECK(
                correction_direction * controller.state().velocity_tcp.x < 0.0
            );

            rb_servo::Wrench6D weak_recontact;
            weak_recontact.fx = measured_sign * 2.0;
            bool stopped_return = false;
            double aligned_offset_when_stopped = 0.0;
            for (int i = 0; i < 3000; ++i) {
                const auto proposal = controller.propose(
                    weak_recontact, xCommand(), rb_servo::Vec6{}, dt
                );
                if (!proposal.valid) {
                    std::cerr << "recontact braking sign=" << measured_sign
                              << " release_ticks=" << release_ticks
                              << " i=" << i << ": " << proposal.reason << "\n";
                }
                RB_CHECK(proposal.valid);
                RB_CHECK(
                    correction_direction * proposal.wrench_error_tcp.fx > 0.0
                );
                RB_CHECK(controller.commit(proposal));

                const double aligned_velocity = correction_direction *
                    controller.state().velocity_tcp.x;
                const double aligned_offset = correction_direction *
                    controller.state().offset_tcp.x;
                if (!stopped_return && aligned_velocity >= 0.0) {
                    stopped_return = true;
                    aligned_offset_when_stopped = aligned_offset;
                }
                if (stopped_return) {
                    RB_CHECK(aligned_offset >= aligned_offset_when_stopped - 1e-12);
                }
            }
            if (!stopped_return) {
                std::cerr << "recontact did not stop sign=" << measured_sign
                          << " release_ticks=" << release_ticks
                          << " x=" << controller.state().offset_tcp.x
                          << " v=" << controller.state().velocity_tcp.x
                          << " a=" << controller.state().acceleration_tcp.x << "\n";
            }
            RB_CHECK(stopped_return);
            RB_CHECK(
                correction_direction * controller.state().offset_tcp.x > 0.0
            );
            RB_CHECK(std::abs(controller.state().velocity_tcp.x) < 1e-9);
            RB_CHECK(std::abs(controller.state().acceleration_tcp.x) < 1e-9);
        }
    }
    return true;
}

bool testRealProfileLateRecontactDeadbandChatterRemainsValid() {
    const rb_servo::ForceControlConfig config = realProfileConfig();
    constexpr double dt = 0.002;
    for (double measured_sign : {-1.0, 1.0}) {
        const double correction_direction = -measured_sign;
        rb_servo::ForceController controller(config);
        controller.engage();

        rb_servo::Wrench6D strong_load;
        strong_load.fx = measured_sign * 6.0;
        for (int i = 0; i < 2500; ++i) {
            const auto proposal = controller.propose(
                strong_load, xCommand(), rb_servo::Vec6{}, dt
            );
            RB_CHECK(proposal.valid);
            RB_CHECK(controller.commit(proposal));
        }

        // Release almost all the way back to the command equilibrium while the
        // spring still has return velocity. Near-deadband contact chatter from
        // this state previously drove the loaded-hold oracle into a fault even
        // though the ordinary jerk-bounded motion envelope remained valid.
        bool reached_late_recontact_state = false;
        for (int i = 0; i < 10000; ++i) {
            const auto proposal = controller.propose(
                rb_servo::Wrench6D{}, xCommand(), rb_servo::Vec6{}, dt
            );
            RB_CHECK(proposal.valid);
            RB_CHECK(controller.commit(proposal));
            const double aligned_offset = correction_direction *
                controller.state().offset_tcp.x;
            const double aligned_velocity = correction_direction *
                controller.state().velocity_tcp.x;
            if (aligned_offset > 0.0 && aligned_offset < 1e-4 &&
                aligned_velocity < 0.0) {
                reached_late_recontact_state = true;
                break;
            }
        }
        RB_CHECK(reached_late_recontact_state);

        rb_servo::Wrench6D weak_recontact;
        for (int i = 0; i < 3000; ++i) {
            // Exercise the measured near-deadband chatter shape from the
            // physical capture: several loaded samples followed by several
            // released samples while the spring is still returning.
            weak_recontact.fx = (i % 12) < 8 ? measured_sign * 2.6 : 0.0;
            const auto proposal = controller.propose(
                weak_recontact, xCommand(), rb_servo::Vec6{}, dt
            );
            if (!proposal.valid) {
                const auto& state = controller.state();
                std::cerr << "late recontact sign=" << measured_sign
                          << " i=" << i
                          << " x=" << state.offset_tcp.x
                          << " v=" << state.velocity_tcp.x
                          << " a=" << state.acceleration_tcp.x
                          << ": " << proposal.reason << "\n";
            }
            RB_CHECK(proposal.valid);
            RB_CHECK(controller.commit(proposal));
        }

        // Once the contact remains continuously loaded, the controller must
        // settle on the load side of the equilibrium without a proposal fault.
        weak_recontact.fx = measured_sign * 2.6;
        for (int i = 0; i < 1000; ++i) {
            const auto proposal = controller.propose(
                weak_recontact, xCommand(), rb_servo::Vec6{}, dt
            );
            RB_CHECK(proposal.valid);
            RB_CHECK(controller.commit(proposal));
        }
        RB_CHECK(
            correction_direction * controller.state().offset_tcp.x > 0.0
        );
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

bool testForceLimiterOpensTranslationEnvelopeAboveLimit() {
    // Base compliance: velocity capped at 0.05 m/s. Limiter: above 10 N the
    // envelope opens 0.02 m/s per excess N up to 0.25 m/s.
    rb_servo::ForceControlConfig config = controllerConfig();
    config.damping = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    config.max_linear_velocity_m_s = 0.05;
    config.force_limit_n = 10.0;
    config.backoff_gain_m_s_per_n = 0.02;
    config.backoff_max_velocity_m_s = 0.25;

    const auto steady_velocity = [&](double force_n) -> double {
        rb_servo::ForceController controller(config);
        controller.engage();
        rb_servo::Wrench6D wrench;
        wrench.fx = force_n;
        double v = 0.0;
        for (int i = 0; i < 400; ++i) {
            const auto proposal = controller.propose(
                wrench, xCommand(), rb_servo::Vec6{}, 0.002);
            // A failed propose/commit returns 0: the outer checks then fail
            // loudly on the velocity expectations.
            if (!proposal.valid || !controller.commit(proposal)) return 0.0;
            v = controller.state().velocity_tcp.x;
        }
        return v;
    };

    // Below the limit: pinned at the base cap (offset moves along -x since the
    // wrench error is target(0) - measured(+F)).
    const double v_below = steady_velocity(8.0);
    RB_CHECK(std::abs(v_below) <= 0.05 + 1e-9);
    RB_CHECK(std::abs(std::abs(v_below) - 0.05) < 5e-3);
    // 25 N: excess after the 0-deadband error = 15 N -> +0.3 clamped by the
    // 0.25 backoff ceiling -> envelope 0.25.
    const double v_above = steady_velocity(25.0);
    RB_CHECK(std::abs(v_above) > 0.05 + 5e-3);
    RB_CHECK(std::abs(v_above) <= 0.25 + 1e-9);

    // Disabled limiter (force_limit_n = 0): high force stays at the base cap.
    config.force_limit_n = 0.0;
    config.backoff_gain_m_s_per_n = 0.0;
    config.backoff_max_velocity_m_s = 0.0;
    const double v_disabled = steady_velocity(25.0);
    RB_CHECK(std::abs(v_disabled) <= 0.05 + 1e-9);
    return true;
}

bool testRemoveComplianceOffsetInvertsAppliedCorrection() {
    // Base command pose with a non-trivial orientation.
    rb_servo::Pose6D base;
    base.x = 0.42; base.y = -0.11; base.z = 0.31;
    base.rx = 0.2; base.ry = -0.4; base.rz = 1.1;

    // tcp_origin-style compliance frame: +90 deg yaw, zero translation.
    rb_servo::Pose6D c_pose;
    c_pose.rz = 1.5707963267948966;

    // Committed translation-only offset (rotation axes disabled in the real profile).
    rb_servo::Pose6D offset;
    offset.x = 0.008; offset.y = -0.003; offset.z = 0.005;

    // measured = base * (C * offset * C^-1): the exact applyForceCorrection composition.
    const pinocchio::SE3 c = rb_servo::math::se3FromPose(c_pose);
    const pinocchio::SE3 d(
        rb_servo::math::exp3(rb_servo::math::Vector3(offset.rx, offset.ry, offset.rz)),
        rb_servo::math::Vector3(offset.x, offset.y, offset.z)
    );
    const rb_servo::Pose6D measured = rb_servo::math::poseFromSe3(
        rb_servo::math::se3FromPose(base) * (c * d * c.inverse())
    );

    const rb_servo::Pose6D recovered =
        rb_servo::removeComplianceOffsetFromMeasured(measured, c_pose, offset);
    RB_CHECK(rb_servo::math::positionDistance(recovered, base) < 1e-9);
    RB_CHECK(rb_servo::math::orientationDistanceRad(recovered, base) < 1e-9);

    // A zero offset is the identity: divergence checks see the raw measured pose.
    rb_servo::Pose6D zero_offset;
    const rb_servo::Pose6D untouched =
        rb_servo::removeComplianceOffsetFromMeasured(measured, c_pose, zero_offset);
    RB_CHECK(rb_servo::math::positionDistance(untouched, measured) < 1e-12);
    RB_CHECK(rb_servo::math::orientationDistanceRad(untouched, measured) < 1e-12);
    return true;
}

bool testContactForceNormalEstimatorFreezesForEpisode() {
    rb_servo::ContactForceNormalEstimator estimator;
    estimator.update(false, rb_servo::math::Vector3(1.0, 0.0, 0.0));
    RB_CHECK(!estimator.valid());

    estimator.update(true, rb_servo::math::Vector3(3.0, 4.0, 0.0));
    RB_CHECK(estimator.valid());
    RB_CHECK((estimator.normalStand() -
              rb_servo::math::Vector3(-0.6, -0.8, 0.0)).norm() < 1e-12);

    estimator.update(true, rb_servo::math::Vector3(0.0, 0.0, -10.0));
    RB_CHECK((estimator.normalStand() -
              rb_servo::math::Vector3(-0.6, -0.8, 0.0)).norm() < 1e-12);

    estimator.update(false, rb_servo::math::Vector3::Zero());
    RB_CHECK(!estimator.valid());
    RB_CHECK(estimator.normalStand().isZero(0.0));
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
    if (!testRealProfileRotationsShareSensitivityBelowContactTelemetryThreshold()) return 1;
    if (!testBlockwiseReleaseDefersSiblingAndPreservesReturnDirection()) return 1;
    if (!testBlockwiseReleaseKeepsSingleAxisGovernorWithAllAxesEnabled()) return 1;
    if (!testBlockwiseReleasePreservesHardEnvelopeRecoveryJerk()) return 1;
    if (!testRealProfileReleaseSweepRemainsRecursivelyFeasible()) return 1;
    if (!testResponsiveRealProfileMovesOnWeakWrenchAndUsesExpandedTravel()) return 1;
    if (!testRealProfileSustainedWrenchNeverReturnsTowardZero()) return 1;
    if (!testRealProfileRecontactWhileRecenteringBrakesWithoutFault()) return 1;
    if (!testRealProfileLateRecontactDeadbandChatterRemainsValid()) return 1;
    if (!testProposalProvenancePreventsStaleOrCrossControllerCommit()) return 1;
    if (!testWrenchDeadbandSuppressesNoiseAndPreservesExcessSign()) return 1;
    if (!testRemoveComplianceOffsetInvertsAppliedCorrection()) return 1;
    if (!testContactForceNormalEstimatorFreezesForEpisode()) return 1;
    if (!testForceLimiterOpensTranslationEnvelopeAboveLimit()) return 1;
    std::cout << "force_controller tests passed\n";
    return 0;
}
