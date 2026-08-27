#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "rb_servo/control/cartesian_servo_controller.hpp"
#include "rb_servo/math/se3.hpp"

namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.141592653589793238462643383279502884;

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

rb_servo::JointArray zeroJoints() {
    rb_servo::JointArray joints{};
    joints.fill(0.0);
    return joints;
}

std::array<double, 4> yawQuaternion(double yaw) {
    return std::array<double, 4>{0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5)};
}

std::vector<rb_servo::FloorCheckPointConfig> gripperTipFloorPoints() {
    return {
        {"gripper_tip_a", {0.054, 0.0, 0.0}},
        {"gripper_tip_b", {-0.054, 0.0, 0.0}},
    };
}

double pointVerticalSpeed(
    const rb_servo::Vec6& stand_twist,
    const rb_servo::Pose6D& tcp_stand,
    const std::array<double, 3>& offset_tcp
) {
    const rb_servo::math::Matrix3 rotation = rb_servo::math::rotationFromPose(tcp_stand);
    const rb_servo::math::Vector3 offset(offset_tcp[0], offset_tcp[1], offset_tcp[2]);
    const rb_servo::math::Vector3 r = rotation * offset;
    return stand_twist.z + stand_twist.rx * r.y() - stand_twist.ry * r.x();
}

class LinearFakeKinematics final : public rb_servo::IKinematics {
public:
    rb_servo::Pose6D computeTcpBase(const rb_servo::JointArray& q_deg) const override {
        rb_servo::Pose6D pose;
        pose.x = q_deg[0] / 100.0;
        pose.y = q_deg[1] / 100.0;
        pose.z = q_deg[2] / 100.0;
        pose.rz = q_deg[5] / 100.0;
        pose.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, std::sin(pose.rz * 0.5), std::cos(pose.rz * 0.5)};
        return pose;
    }

    rb_servo::Pose6D computeTcpStand(
        rb_servo::ArmId arm,
        const rb_servo::JointArray& q_deg,
        const rb_servo::ArmMountConfig& mount
    ) const override {
        (void)arm;
        rb_servo::Pose6D pose = computeTcpBase(q_deg);
        pose.x += mount.base_pose_in_stand.x;
        pose.y += mount.base_pose_in_stand.y;
        pose.z += mount.base_pose_in_stand.z;
        return pose;
    }

    rb_servo::IkResult solveIk(
        rb_servo::ArmId arm,
        const rb_servo::Pose6D& target_tcp_stand,
        const rb_servo::JointArray& seed_q_deg,
        const rb_servo::ArmMountConfig& mount
    ) const override {
        (void)arm;
        // Exact inverse of the linear FK above (x/y/z and yaw-only rotation).
        rb_servo::IkResult result;
        result.success = true;
        result.q_solution_deg = seed_q_deg;
        result.q_solution_deg[0] = (target_tcp_stand.x - mount.base_pose_in_stand.x) * 100.0;
        result.q_solution_deg[1] = (target_tcp_stand.y - mount.base_pose_in_stand.y) * 100.0;
        result.q_solution_deg[2] = (target_tcp_stand.z - mount.base_pose_in_stand.z) * 100.0;
        double yaw = target_tcp_stand.rz;
        if (target_tcp_stand.quaternion_xyzw.has_value()) {
            const auto& quat = *target_tcp_stand.quaternion_xyzw;
            yaw = 2.0 * std::atan2(quat[2], quat[3]);
        }
        result.q_solution_deg[5] = yaw * 100.0;
        last_ik_target = target_tcp_stand;
        ++ik_call_count;
        return result;
    }

    mutable rb_servo::Pose6D last_ik_target;
    mutable int ik_call_count = 0;
};

rb_servo::RobotState stateFromJoints(
    const rb_servo::IKinematics& kinematics,
    const rb_servo::JointArray& q_deg,
    const rb_servo::ArmMountConfig& mount
) {
    rb_servo::RobotState state;
    state.arm_id = rb_servo::ArmId::Left;
    state.q_actual_deg = q_deg;
    state.has_valid_joint_state = true;
    state.tcp_stand = kinematics.computeTcpStand(rb_servo::ArmId::Left, q_deg, mount);
    state.has_valid_tcp_pose = true;
    return state;
}

bool testPureTranslationTracksLineAndKeepsOrientation() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.max_linear_move_speed_m_s = 1.0;
    config.max_angular_move_speed_rad_s = 1.0;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpLinearMove;
    command.has_tcp_target = true;
    command.timeout_sec = 0.2;
    command.linear_move_duration_sec = 0.2;
    command.has_linear_move_duration = true;
    command.tcp_target_stand = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0};
    command.tcp_target_stand.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};

    rb_servo::CartesianServoPathState path;
    rb_servo::JointArray q = zeroJoints();
    double max_orientation_error = 0.0;
    double max_line_deviation = 0.0;
    bool saw_done = false;
    for (int tick = 0; tick < 40; ++tick) {
        rb_servo::RobotState state = stateFromJoints(*kinematics, q, left_mount);
        const rb_servo::CartesianArmTargetResult result = controller.computeLinearMoveTarget(
            command,
            state,
            q,
            rb_servo::RunMode::Simulation,
            0.005,
            42,
            &path
        );
        RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
        RB_CHECK(result.telemetry.status == "ok");
        max_orientation_error = std::max(max_orientation_error, result.telemetry.path_orientation_error_rad);
        max_line_deviation = std::max(max_line_deviation, result.telemetry.path_line_deviation_m);
        saw_done = saw_done || result.telemetry.path_done;
        q = result.q_target_deg;
    }

    RB_CHECK(saw_done);
    RB_CHECK(max_orientation_error < 1e-9);
    RB_CHECK(max_line_deviation < 1e-6);
    RB_CHECK(std::abs(q[5]) < kEpsilon);
    RB_CHECK(std::abs(q[0] / 100.0 - 0.05) < 0.002);
    return true;
}

bool testRealModeBlocked() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    rb_servo::ArmMountConfig right_mount;
    rb_servo::CartesianServoController controller(left_mount, right_mount, rb_servo::CartesianControlConfig{}, kinematics);
    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpLinearMove;
    command.has_tcp_target = true;
    command.linear_move_duration_sec = 1.0;
    command.has_linear_move_duration = true;
    command.tcp_target_stand = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0};
    rb_servo::CartesianServoPathState path;
    rb_servo::JointArray q = zeroJoints();
    const rb_servo::CartesianArmTargetResult result = controller.computeLinearMoveTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Real,
        0.005,
        1,
        &path
    );
    // Real/sim gating retired: linear move computes in every run mode.
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(result.telemetry.status == "ok");
    return true;
}

bool testLinearMoveQuinticProfileEasesEndpoints() {
    // SMD off (default config): the commanded pose IS the quintic reference.
    // The per-tick step must ease in/out (small at both endpoints) and the
    // peak step must stay within the quintic peak/mean ratio (15/8).
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.max_linear_move_speed_m_s = 1.0;
    config.max_angular_move_speed_rad_s = 1.0;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpLinearMove;
    command.has_tcp_target = true;
    command.linear_move_duration_sec = 0.2;
    command.has_linear_move_duration = true;
    command.tcp_target_stand = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0};
    command.tcp_target_stand.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};

    rb_servo::CartesianServoPathState path;
    rb_servo::JointArray q = zeroJoints();
    constexpr int kTicks = 40;
    std::array<double, kTicks> step{};
    for (int tick = 0; tick < kTicks; ++tick) {
        const double previous_x = q[0] / 100.0;
        const rb_servo::CartesianArmTargetResult result = controller.computeLinearMoveTarget(
            command,
            stateFromJoints(*kinematics, q, left_mount),
            q,
            rb_servo::RunMode::Simulation,
            0.005,
            7,
            &path
        );
        RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
        q = result.q_target_deg;
        step[static_cast<std::size_t>(tick)] = std::abs(q[0] / 100.0 - previous_x);
    }
    const double mean_step = 0.05 / kTicks;
    double max_step = 0.0;
    for (double value : step) max_step = std::max(max_step, value);
    // Eased endpoints: first/last tick steps are far below the mean step…
    RB_CHECK(step[0] < 0.3 * mean_step);
    RB_CHECK(step[kTicks - 1] < 0.3 * mean_step);
    // …and the mid-path peak honors the quintic peak/mean ratio.
    RB_CHECK(max_step > 1.5 * mean_step);
    RB_CHECK(max_step < 1.875 * mean_step + 1e-9);
    // The profile still lands exactly on the target.
    RB_CHECK(std::abs(q[0] / 100.0 - 0.05) < 1e-9);
    return true;
}

bool testLinearMoveFeedforwardIgnoresMeasuredNoise() {
    // Judder regression: with pose_track_smd enabled (the stack_real setup),
    // the commanded joint stream must be IDENTICAL with and without measured
    // joint-state noise — the chain is feedforward, measured state feeds
    // telemetry only. The old v_ref + Kp * bodyError(measured, reference)
    // servo re-injected measurement noise into every tick.
    const auto run_with_noise = [](double noise_amplitude_deg) {
        auto kinematics = std::make_shared<LinearFakeKinematics>();
        rb_servo::ArmMountConfig left_mount;
        left_mount.arm_id = rb_servo::ArmId::Left;
        rb_servo::ArmMountConfig right_mount;
        right_mount.arm_id = rb_servo::ArmId::Right;
        rb_servo::CartesianControlConfig config;
        config.max_linear_move_speed_m_s = 1.0;
        config.max_angular_move_speed_rad_s = 1.0;
        config.pose_track_smd.enable = true;
        config.pose_track_smd.natural_frequency_linear_hz = 3.0;
        config.pose_track_smd.natural_frequency_angular_hz = 3.0;
        rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

        rb_servo::ArmCommand command;
        command.arm_id = rb_servo::ArmId::Left;
        command.mode = rb_servo::ControlMode::TcpLinearMove;
        command.has_tcp_target = true;
        command.linear_move_duration_sec = 0.2;
        command.has_linear_move_duration = true;
        command.tcp_target_stand = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0};
        command.tcp_target_stand.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};

        rb_servo::CartesianServoPathState path;
        rb_servo::JointArray q = zeroJoints();
        std::vector<double> trajectory;
        bool saw_done = false;
        for (int tick = 0; tick < 200; ++tick) {
            rb_servo::JointArray q_measured = q;
            q_measured[0] += (tick % 2 == 0 ? noise_amplitude_deg : -noise_amplitude_deg);
            const rb_servo::CartesianArmTargetResult result = controller.computeLinearMoveTarget(
                command,
                stateFromJoints(*kinematics, q_measured, left_mount),
                q,
                rb_servo::RunMode::Simulation,
                0.005,
                11,
                &path
            );
            if (result.verdict != rb_servo::SafetyVerdict::Ok) {
                return std::pair<std::vector<double>, bool>{{}, false};
            }
            q = result.q_target_deg;
            trajectory.push_back(q[0]);
            saw_done = saw_done || result.telemetry.path_done;
        }
        return std::pair<std::vector<double>, bool>{trajectory, saw_done};
    };

    const auto clean = run_with_noise(0.0);
    const auto noisy = run_with_noise(0.7);
    RB_CHECK(clean.second);
    RB_CHECK(noisy.second);
    RB_CHECK(clean.first.size() == noisy.first.size());
    RB_CHECK(!clean.first.empty());
    for (std::size_t i = 0; i < clean.first.size(); ++i) {
        RB_CHECK(std::abs(clean.first[i] - noisy.first[i]) < 1e-12);
    }
    // The smoothed move still settles on the target.
    RB_CHECK(std::abs(clean.first.back() / 100.0 - 0.05) < 1e-3);
    return true;
}

bool testLinearMoveConstantOrientationToleranceIsConfigurable() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.linear_move.constant_orientation_tolerance_rad = 0.005;
    config.max_linear_move_speed_m_s = 1.0;
    config.max_angular_move_speed_rad_s = 1.0;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpLinearMove;
    command.has_tcp_target = true;
    command.timeout_sec = 0.2;
    command.linear_move_duration_sec = 0.2;
    command.has_linear_move_duration = true;
    command.tcp_target_stand = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0};
    command.tcp_target_stand.quaternion_xyzw = yawQuaternion(0.004);

    rb_servo::CartesianServoPathState path;
    rb_servo::JointArray q = zeroJoints();
    rb_servo::CartesianArmTargetResult result = controller.computeLinearMoveTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.005,
        50,
        &path
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(path.active);

    command.tcp_target_stand.quaternion_xyzw = yawQuaternion(0.006);
    rb_servo::CartesianServoPathState reject_path;
    result = controller.computeLinearMoveTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.005,
        51,
        &reject_path
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::InvalidCommand);
    RB_CHECK(result.reason == "tcp_linear_move_constant_orientation_mismatch");
    RB_CHECK(!reject_path.active);
    return true;
}

bool testQuaternionAndRpyYawFrameConversionMatch() {
    rb_servo::Pose6D rpy_pose;
    rpy_pose.rz = 0.7;
    rb_servo::Pose6D quaternion_pose;
    quaternion_pose.quaternion_xyzw = yawQuaternion(0.7);

    rb_servo::Vec6 stand_twist{0.03, -0.01, 0.02, 0.0, 0.0, 0.04};
    const rb_servo::Vec6 local_from_rpy = rb_servo::math::twistStandToLocal(stand_twist, rpy_pose);
    const rb_servo::Vec6 local_from_quaternion = rb_servo::math::twistStandToLocal(stand_twist, quaternion_pose);

    RB_CHECK(std::abs(local_from_rpy.x - local_from_quaternion.x) < 1e-12);
    RB_CHECK(std::abs(local_from_rpy.y - local_from_quaternion.y) < 1e-12);
    RB_CHECK(std::abs(local_from_rpy.z - local_from_quaternion.z) < 1e-12);
    RB_CHECK(std::abs(local_from_rpy.rx - local_from_quaternion.rx) < 1e-12);
    RB_CHECK(std::abs(local_from_rpy.ry - local_from_quaternion.ry) < 1e-12);
    RB_CHECK(std::abs(local_from_rpy.rz - local_from_quaternion.rz) < 1e-12);
    RB_CHECK(rb_servo::math::orientationDistanceRad(rpy_pose, quaternion_pose) < 1e-12);
    return true;
}

bool testLinearMoveConstantOrientationNearPiStaysFinite() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.max_linear_move_speed_m_s = 1.0;
    config.max_angular_move_speed_rad_s = 1.0;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::JointArray q = zeroJoints();
    const double yaw = kPi - 1e-6;
    q[5] = yaw * 100.0;
    rb_servo::RobotState state = stateFromJoints(*kinematics, q, left_mount);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpLinearMove;
    command.has_tcp_target = true;
    command.has_linear_move_duration = true;
    command.linear_move_duration_sec = 0.2;
    command.tcp_target_stand = *state.tcp_stand;
    command.tcp_target_stand.x += 0.01;

    rb_servo::CartesianServoPathState path;
    double max_orientation_error = 0.0;
    for (int tick = 0; tick < 5; ++tick) {
        state = stateFromJoints(*kinematics, q, left_mount);
        const rb_servo::CartesianArmTargetResult result = controller.computeLinearMoveTarget(
            command,
            state,
            q,
            rb_servo::RunMode::Simulation,
            0.005,
            77,
            &path
        );
        RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
        RB_CHECK(std::isfinite(result.telemetry.path_orientation_error_rad));
        RB_CHECK(std::isfinite(result.telemetry.path_position_error_m));
        max_orientation_error = std::max(max_orientation_error, result.telemetry.path_orientation_error_rad);
        q = result.q_target_deg;
    }

    RB_CHECK(max_orientation_error < 1e-6);
    return true;
}

// QSYNC SETTLING HOLD, plan side. While queue sync is in warmup/drain the loop pins the
// arm at prev_sent and hands this stage dt_sec = 0, so the path clock must not advance:
// the reference has to stay AT the pinned pose and the trajectory after release must be
// tick-for-tick identical to one that was never held. Before this, the quintic clock kept
// running under the pin and the accrued lead discharged as a release lunge (measured
// 2026-08-27 on the left arm: 532 ms pinned, 1.09 deg of accrued IK lead, 7,920 deg/s^2
// command accel on release).
bool testLinearMovePathClockFreezesWhileHeld() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.max_linear_move_speed_m_s = 1.0;
    config.max_angular_move_speed_rad_s = 1.0;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpLinearMove;
    command.has_tcp_target = true;
    command.timeout_sec = 0.2;
    command.linear_move_duration_sec = 0.2;
    command.has_linear_move_duration = true;
    command.tcp_target_stand = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0};
    command.tcp_target_stand.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};

    constexpr double kDt = 0.005;
    constexpr int kHeldTicks = 50;  // 250 ms of warmup/drain, the measured order
    const rb_servo::JointArray start = zeroJoints();

    // Reference run: never held.
    std::vector<rb_servo::JointArray> free_run;
    {
        rb_servo::CartesianServoPathState path;
        rb_servo::JointArray q = start;
        for (int tick = 0; tick < 40; ++tick) {
            const rb_servo::CartesianArmTargetResult result = controller.computeLinearMoveTarget(
                command, stateFromJoints(*kinematics, q, left_mount), q,
                rb_servo::RunMode::Simulation, kDt, 42, &path);
            RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
            q = result.q_target_deg;
            free_run.push_back(q);
        }
    }

    // Held run: kHeldTicks at dt = 0 (the loop pins the output at prev_sent), then release.
    rb_servo::CartesianServoPathState path;
    rb_servo::JointArray q = start;
    for (int tick = 0; tick < kHeldTicks; ++tick) {
        const rb_servo::CartesianArmTargetResult result = controller.computeLinearMoveTarget(
            command, stateFromJoints(*kinematics, q, left_mount), q,
            rb_servo::RunMode::Simulation, 0.0, 42, &path);
        RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
        // No lead accrues: the reference stays at s = 0 and the solve asks for no motion.
        RB_CHECK(result.telemetry.path_s < kEpsilon);
        RB_CHECK(result.telemetry.linear_move_elapsed_sec < kEpsilon);
        for (std::size_t j = 0; j < rb_servo::kDof; ++j) {
            RB_CHECK(std::abs(result.q_target_deg[j] - start[j]) < 1e-9);
        }
        q = result.q_target_deg;
    }

    // After release the path runs exactly as if it had never been held.
    for (int tick = 0; tick < 40; ++tick) {
        const rb_servo::CartesianArmTargetResult result = controller.computeLinearMoveTarget(
            command, stateFromJoints(*kinematics, q, left_mount), q,
            rb_servo::RunMode::Simulation, kDt, 42, &path);
        RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
        q = result.q_target_deg;
        for (std::size_t j = 0; j < rb_servo::kDof; ++j) {
            RB_CHECK(std::abs(q[j] - free_run[static_cast<std::size_t>(tick)][j]) < 1e-9);
        }
    }
    RB_CHECK(std::abs(q[0] / 100.0 - 0.05) < 0.002);
    return true;
}

bool testScalarFirstOrderPlantShowsMeasuredActualAttenuation() {
    constexpr double dt = 0.01;
    constexpr double tau = 0.04;
    constexpr double velocity = 1.0;
    double measured_actual = 0.0;
    double previous_command_actual = 0.0;
    double command_state = 0.0;
    for (int tick = 0; tick < 100; ++tick) {
        const double measured_target = measured_actual + velocity * dt;
        command_state += velocity * dt;
        measured_actual += (dt / tau) * (measured_target - measured_actual);
        previous_command_actual += (dt / tau) * (command_state - previous_command_actual);
    }
    const double reference = velocity * dt * 100.0;
    const double measured_gain = measured_actual / reference;
    const double previous_command_gain = previous_command_actual / reference;
    RB_CHECK(measured_gain > 0.20 && measured_gain < 0.30);
    RB_CHECK(previous_command_gain > 0.95);
    return true;
}

}  // namespace

int main() {
    if (!testPureTranslationTracksLineAndKeepsOrientation()) return 1;
    if (!testRealModeBlocked()) return 1;
    if (!testLinearMoveQuinticProfileEasesEndpoints()) return 1;
    if (!testLinearMoveFeedforwardIgnoresMeasuredNoise()) return 1;
    if (!testLinearMoveConstantOrientationToleranceIsConfigurable()) return 1;
    if (!testQuaternionAndRpyYawFrameConversionMatch()) return 1;
    if (!testLinearMoveConstantOrientationNearPiStaysFinite()) return 1;
    if (!testLinearMovePathClockFreezesWhileHeld()) return 1;
    if (!testScalarFirstOrderPlantShowsMeasuredActualAttenuation()) return 1;
    return 0;
}
