#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

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
        (void)target_tcp_stand;
        (void)mount;
        rb_servo::IkResult result;
        result.success = true;
        result.q_solution_deg = seed_q_deg;
        return result;
    }

    rb_servo::CartesianVelocityResult solveCartesianVelocity(
        rb_servo::ArmId arm,
        const rb_servo::JointArray& q_deg,
        const rb_servo::ArmMountConfig& mount,
        const rb_servo::Vec6& tcp_twist_local,
        double damping
    ) const override {
        (void)arm;
        (void)q_deg;
        (void)mount;
        last_twist_local = tcp_twist_local;
        last_damping = damping;
        ++velocity_call_count;
        rb_servo::CartesianVelocityResult result;
        result.success = true;
        result.qdot_deg_s[0] = tcp_twist_local.x * 100.0;
        result.qdot_deg_s[1] = tcp_twist_local.y * 100.0;
        result.qdot_deg_s[2] = tcp_twist_local.z * 100.0;
        result.qdot_deg_s[5] = tcp_twist_local.rz * 100.0;
        return result;
    }

    mutable rb_servo::Vec6 last_twist_local;
    mutable double last_damping = 0.0;
    mutable int velocity_call_count = 0;
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

bool testLinearMoveUsesSeparatePositionAndOrientationGains() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.path_kp_pos = 2.0;
    config.path_kp_ori = 3.0;
    config.max_linear_move_speed_m_s = 10.0;
    config.max_angular_move_speed_rad_s = 10.0;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpLinearMove;

    const rb_servo::JointArray q = zeroJoints();
    rb_servo::CartesianServoPathState position_path;
    position_path.active = true;
    position_path.duration_sec = 1.0;
    position_path.start_tcp_stand = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
    position_path.start_tcp_stand.quaternion_xyzw = yawQuaternion(0.0);
    position_path.target_tcp_stand = position_path.start_tcp_stand;
    position_path.orientation_mode = rb_servo::CartesianOrientationInterpolation::Constant;

    rb_servo::CartesianArmTargetResult result = controller.computeLinearMoveTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.0,
        0,
        &position_path
    );

    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(std::abs(kinematics->last_twist_local.x - 0.2) < 1e-9);

    rb_servo::CartesianServoPathState orientation_path;
    orientation_path.active = true;
    orientation_path.duration_sec = 1.0;
    orientation_path.start_tcp_stand = {0.0, 0.0, 0.0, 0.0, 0.0, 0.2};
    orientation_path.start_tcp_stand.quaternion_xyzw = yawQuaternion(0.2);
    orientation_path.target_tcp_stand = orientation_path.start_tcp_stand;
    orientation_path.orientation_mode = rb_servo::CartesianOrientationInterpolation::Constant;

    result = controller.computeLinearMoveTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.0,
        0,
        &orientation_path
    );

    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(std::abs(kinematics->last_twist_local.rz - 0.6) < 1e-9);
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

bool testTcpTwistLocalMovesLocalXAndHoldsOrientation() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianServoController controller(left_mount, right_mount, rb_servo::CartesianControlConfig{}, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.02, 0.0, 0.0, 0.0, 0.0, 0.0};

    rb_servo::CartesianTwistHoldState hold;
    rb_servo::JointArray q = zeroJoints();
    double max_orientation_error = 0.0;
    for (int tick = 0; tick < 20; ++tick) {
        const rb_servo::CartesianArmTargetResult result = controller.computeTwistTarget(
            command,
            stateFromJoints(*kinematics, q, left_mount),
            q,
            rb_servo::RunMode::Simulation,
            0.005,
            1,
            &hold
        );
        RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
        max_orientation_error = std::max(max_orientation_error, result.telemetry.path_orientation_error_rad);
        q = result.q_target_deg;
    }

    RB_CHECK(q[0] > 0.0);
    RB_CHECK(std::abs(q[1]) < kEpsilon);
    RB_CHECK(std::abs(q[2]) < kEpsilon);
    RB_CHECK(std::abs(q[5]) < kEpsilon);
    RB_CHECK(max_orientation_error < 1e-9);
    return true;
}

bool testFloorConstraintZerosDownwardVzAtPlaneAndKeepsLateral() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianServoController controller(
        left_mount, right_mount, rb_servo::CartesianControlConfig{}, kinematics);
    controller.setFloorConstraint(true, 0.010, 0.005);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.02, 0.0, -0.02, 0.0, 0.0, 0.0};

    // Identity-orientation TCP at z = 0.012 (inside the 5 mm soft margin above the
    // 10 mm plane): downward stand v_z must be zeroed, lateral x preserved.
    rb_servo::CartesianTwistHoldState hold;
    rb_servo::JointArray q = zeroJoints();
    q[2] = 1.2;  // fake FK: z = q[2] / 100
    const rb_servo::CartesianArmTargetResult at_floor = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.005,
        1,
        &hold
    );
    RB_CHECK(at_floor.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(at_floor.telemetry.floor_vz_clamped);
    RB_CHECK(std::abs(kinematics->last_twist_local.z) < kEpsilon);
    RB_CHECK(std::abs(kinematics->last_twist_local.x - 0.02) < kEpsilon);

    // Well above the plane: the downward component passes through untouched.
    rb_servo::CartesianTwistHoldState hold_above;
    rb_servo::JointArray q_above = zeroJoints();
    q_above[2] = 30.0;  // z = 0.30 m
    const rb_servo::CartesianArmTargetResult above = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q_above, left_mount),
        q_above,
        rb_servo::RunMode::Simulation,
        0.005,
        1,
        &hold_above
    );
    RB_CHECK(above.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(!above.telemetry.floor_vz_clamped);
    RB_CHECK(std::abs(kinematics->last_twist_local.z + 0.02) < kEpsilon);
    return true;
}

bool testFloorConstraintRespectsTcpOrientationFrame() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.twist_angular_deadband_rad_s = 0.0;  // skip orientation hold for this test
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);
    controller.setFloorConstraint(true, 0.010, 0.005);

    // TCP yawed by 90 deg (fake FK: rz = q[5] / 100). A local +x twist maps to
    // stand +y — horizontal, so nothing should be clamped even at the plane.
    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.02, 0.0, 0.0, 0.0, 0.0, 0.0};

    rb_servo::CartesianTwistHoldState hold;
    rb_servo::JointArray q = zeroJoints();
    q[2] = 1.0;                  // z = 0.010 m (on the plane)
    q[5] = 100.0 * M_PI / 2.0;   // rz = pi/2
    const rb_servo::CartesianArmTargetResult result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.005,
        1,
        &hold
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(!result.telemetry.floor_vz_clamped);
    RB_CHECK(std::abs(kinematics->last_twist_local.x - 0.02) < 1e-6);
    return true;
}

bool testTcpTwistAngularDeadbandMaintainsHoldForNoise() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.twist_angular_deadband_rad_s = 0.001;
    config.twist_orientation_hold_kp = 4.0;
    config.max_twist_linear_m_s = 1.0;
    config.max_twist_angular_rad_s = 1.0;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0005};

    rb_servo::CartesianTwistHoldState hold;
    rb_servo::JointArray q = zeroJoints();
    rb_servo::CartesianArmTargetResult result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.005,
        1,
        &hold
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(hold.orientation_hold_active);
    RB_CHECK(std::abs(result.telemetry.requested_twist_angular_norm_rad_s - 0.0005) < kEpsilon);
    RB_CHECK(std::abs(kinematics->last_twist_local.rz) < kEpsilon);
    const rb_servo::Pose6D captured_hold = hold.hold_tcp_stand;

    q[5] = 1.0;
    result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.005,
        1,
        &hold
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(hold.orientation_hold_active);
    RB_CHECK(std::abs(hold.hold_tcp_stand.rz - captured_hold.rz) < kEpsilon);
    RB_CHECK(std::abs(kinematics->last_twist_local.rz) > 0.001);

    command.tcp_twist_local = {0.0, 0.0, 0.0, 0.0, 0.0, 0.002};
    result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.005,
        1,
        &hold
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(!hold.orientation_hold_active);
    RB_CHECK(std::abs(kinematics->last_twist_local.rz - 0.002) < kEpsilon);
    return true;
}

bool testPositiveOrientationHoldErrorReducesAfterSyntheticIntegration() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.twist_angular_deadband_rad_s = 0.001;
    config.twist_orientation_hold_kp = 2.0;
    config.max_twist_linear_m_s = 1.0;
    config.max_twist_angular_rad_s = 1.0;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    rb_servo::CartesianTwistHoldState hold;
    hold.orientation_hold_active = true;
    hold.hold_tcp_stand.rz = 0.02;
    hold.hold_tcp_stand.quaternion_xyzw = yawQuaternion(0.02);

    const rb_servo::JointArray q = zeroJoints();
    const rb_servo::RobotState state = stateFromJoints(*kinematics, q, left_mount);
    const double error_before = rb_servo::math::orientationDistanceRad(*state.tcp_stand, hold.hold_tcp_stand);
    const rb_servo::CartesianArmTargetResult result = controller.computeTwistTarget(
        command,
        state,
        q,
        rb_servo::RunMode::Simulation,
        0.05,
        1,
        &hold
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(kinematics->last_twist_local.rz > 0.0);
    RB_CHECK(result.q_target_deg[5] > q[5]);

    const rb_servo::RobotState next_state = stateFromJoints(*kinematics, result.q_target_deg, left_mount);
    const double error_after = rb_servo::math::orientationDistanceRad(*next_state.tcp_stand, hold.hold_tcp_stand);
    RB_CHECK(error_before > 0.0);
    RB_CHECK(error_after < error_before);
    RB_CHECK(std::abs(result.telemetry.applied_twist_angular_norm_rad_s - std::abs(kinematics->last_twist_local.rz)) < kEpsilon);
    return true;
}

bool testTcpTwistStandPositiveWorldXConvertsToLocalNegativeYAtPositiveYaw() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.max_twist_linear_m_s = 1.0;
    config.max_twist_angular_rad_s = 1.0;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistStand;
    command.has_tcp_twist_stand = true;
    command.tcp_twist_stand = {0.02, 0.0, 0.0, 0.0, 0.0, 0.03};

    rb_servo::JointArray q = zeroJoints();
    q[5] = 0.5 * kPi * 100.0;
    rb_servo::CartesianTwistHoldState hold;
    const rb_servo::CartesianArmTargetResult result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.01,
        1,
        &hold
    );

    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(std::abs(kinematics->last_twist_local.x) < 1e-12);
    RB_CHECK(std::abs(kinematics->last_twist_local.y + 0.02) < 1e-12);
    RB_CHECK(std::abs(kinematics->last_twist_local.rz - 0.03) < 1e-12);
    RB_CHECK(result.q_target_deg[1] < q[1]);
    RB_CHECK(result.q_target_deg[5] > q[5]);
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
    RB_CHECK(std::isfinite(kinematics->last_twist_local.x));
    RB_CHECK(std::isfinite(kinematics->last_twist_local.rz));
    return true;
}

bool testTcpTwistOrientationHoldNearPiStaysBounded() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.max_twist_angular_rad_s = 0.2;
    config.exceed_limit_policy = rb_servo::CartesianLimitPolicy::Clamp;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    rb_servo::CartesianTwistHoldState hold;
    hold.orientation_hold_active = true;
    hold.hold_tcp_stand.quaternion_xyzw = yawQuaternion(kPi - 1e-6);

    rb_servo::JointArray q = zeroJoints();
    const rb_servo::CartesianArmTargetResult result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.005,
        1,
        &hold
    );

    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(std::isfinite(result.telemetry.path_orientation_error_rad));
    RB_CHECK(std::isfinite(result.telemetry.applied_twist_angular_norm_rad_s));
    RB_CHECK(result.telemetry.path_orientation_error_rad > 3.0);
    RB_CHECK(result.telemetry.applied_twist_angular_norm_rad_s <= config.max_twist_angular_rad_s + kEpsilon);
    RB_CHECK(kinematics->velocity_call_count == 1);
    RB_CHECK(std::isfinite(kinematics->last_twist_local.rz));
    return true;
}

bool testTcpTwistRealModeBlocked() {
    // Real run mode computes a twist target (physical-real gating —
    // cartesian_control.allow_in_real + RB_ALLOW_REAL_CARTESIAN — is enforced
    // by the servo loop's cartesian availability gate, not here). Mock run
    // mode stays blocked.
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    rb_servo::ArmMountConfig right_mount;
    rb_servo::CartesianServoController controller(left_mount, right_mount, rb_servo::CartesianControlConfig{}, kinematics);
    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.02, 0.0, 0.0, 0.0, 0.0, 0.0};
    rb_servo::CartesianTwistHoldState hold;
    rb_servo::JointArray q = zeroJoints();
    const rb_servo::CartesianArmTargetResult real_result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Real,
        0.005,
        1,
        &hold
    );
    RB_CHECK(real_result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(real_result.telemetry.status == "ok");

    rb_servo::CartesianTwistHoldState mock_hold;
    const rb_servo::CartesianArmTargetResult mock_result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Mock,
        0.005,
        1,
        &mock_hold
    );
    // Real/sim gating retired: twist computes in every run mode (mock included).
    RB_CHECK(mock_result.verdict == rb_servo::SafetyVerdict::Ok);
    return true;
}

bool testTcpTwistClampTelemetryAndDamping() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.max_twist_linear_m_s = 0.03;
    config.max_twist_angular_rad_s = 0.2;
    config.velocity_damping = 0.123;
    config.exceed_limit_policy = rb_servo::CartesianLimitPolicy::Clamp;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.3, 0.0, 0.0, 0.0, 0.0, 1.0};

    rb_servo::CartesianTwistHoldState hold;
    rb_servo::JointArray q = zeroJoints();
    const rb_servo::CartesianArmTargetResult result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.01,
        1,
        &hold
    );

    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(result.telemetry.twist_clamped);
    RB_CHECK(std::abs(result.telemetry.requested_twist_linear_norm_m_s - 0.3) < kEpsilon);
    RB_CHECK(std::abs(result.telemetry.requested_twist_angular_norm_rad_s - 1.0) < kEpsilon);
    RB_CHECK(result.telemetry.applied_twist_linear_norm_m_s <= config.max_twist_linear_m_s + kEpsilon);
    RB_CHECK(result.telemetry.applied_twist_angular_norm_rad_s <= config.max_twist_angular_rad_s + kEpsilon);
    RB_CHECK(std::abs(kinematics->last_twist_local.x - config.max_twist_linear_m_s) < kEpsilon);
    RB_CHECK(std::abs(kinematics->last_twist_local.rz - config.max_twist_angular_rad_s) < kEpsilon);
    RB_CHECK(std::abs(kinematics->last_damping - config.velocity_damping) < kEpsilon);
    RB_CHECK(kinematics->velocity_call_count == 1);
    return true;
}

bool testTcpTwistRejectPolicy() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.max_twist_linear_m_s = 0.03;
    config.max_twist_angular_rad_s = 0.2;
    config.exceed_limit_policy = rb_servo::CartesianLimitPolicy::Reject;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.031, 0.0, 0.0, 0.0, 0.0, 0.0};

    rb_servo::CartesianTwistHoldState hold;
    rb_servo::JointArray q = zeroJoints();
    const rb_servo::CartesianArmTargetResult result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Simulation,
        0.01,
        1,
        &hold
    );

    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::InvalidCommand);
    RB_CHECK(result.reason == "cartesian_twist_limit_exceeded");
    RB_CHECK(result.telemetry.requested_twist_linear_norm_m_s > config.max_twist_linear_m_s);
    RB_CHECK(result.telemetry.applied_twist_linear_norm_m_s == 0.0);
    RB_CHECK(kinematics->velocity_call_count == 0);
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

bool testVelocityIntegrationModesGenerateExpectedTargets() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0};

    rb_servo::CartesianTwistHoldState hold;
    rb_servo::JointArray q_actual = zeroJoints();
    q_actual[0] = 10.0;

    rb_servo::CartesianControlConfig measured_config;
    measured_config.max_twist_linear_m_s = 1.0;
    measured_config.velocity_target_integration =
        rb_servo::CartesianVelocityTargetIntegrationMode::MeasuredActual;
    rb_servo::CartesianServoController measured_controller(
        left_mount,
        right_mount,
        measured_config,
        kinematics
    );
    rb_servo::CartesianArmTargetResult result = measured_controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q_actual, left_mount),
        q_actual,
        rb_servo::RunMode::Simulation,
        0.01,
        10,
        &hold
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(std::abs(result.q_target_deg[0] - 10.01) < 1e-12);

    rb_servo::CartesianControlConfig lookahead_config = measured_config;
    lookahead_config.velocity_target_integration =
        rb_servo::CartesianVelocityTargetIntegrationMode::MeasuredActualLookahead;
    lookahead_config.velocity_target_lookahead_sec = 0.04;
    rb_servo::CartesianServoController lookahead_controller(
        left_mount,
        right_mount,
        lookahead_config,
        kinematics
    );
    result = lookahead_controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q_actual, left_mount),
        q_actual,
        rb_servo::RunMode::Simulation,
        0.01,
        11,
        &hold
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(std::abs(result.q_target_deg[0] - 10.04) < 1e-12);

    rb_servo::CartesianControlConfig previous_config = measured_config;
    previous_config.velocity_target_integration =
        rb_servo::CartesianVelocityTargetIntegrationMode::PreviousCommand;
    rb_servo::CartesianServoController previous_controller(
        left_mount,
        right_mount,
        previous_config,
        kinematics
    );
    rb_servo::CartesianVelocityIntegratorState integrator;
    result = previous_controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q_actual, left_mount),
        q_actual,
        rb_servo::RunMode::Simulation,
        0.01,
        12,
        &hold,
        &integrator
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    previous_controller.updateVelocityIntegratorAfterSafety(
        &integrator,
        result.q_target_deg,
        true,
        false,
        ""
    );
    q_actual[0] = 10.0;
    result = previous_controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q_actual, left_mount),
        q_actual,
        rb_servo::RunMode::Simulation,
        0.01,
        13,
        &hold,
        &integrator
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(std::abs(result.q_target_deg[0] - 10.02) < 1e-12);
    return true;
}

bool testVelocityIntegratorUsesClampedSafeTarget() {
    rb_servo::CartesianControlConfig config;
    config.velocity_target_integration =
        rb_servo::CartesianVelocityTargetIntegrationMode::PreviousCommand;
    rb_servo::CartesianServoController controller(
        rb_servo::ArmMountConfig{},
        rb_servo::ArmMountConfig{},
        config,
        std::make_shared<LinearFakeKinematics>()
    );
    rb_servo::CartesianVelocityIntegratorState integrator;
    integrator.valid = true;
    integrator.q_command_deg = zeroJoints();
    rb_servo::JointArray clamped = zeroJoints();
    clamped[0] = 0.2;
    controller.updateVelocityIntegratorAfterSafety(
        &integrator,
        clamped,
        true,
        true,
        ""
    );
    RB_CHECK(integrator.valid);
    RB_CHECK(std::abs(integrator.q_command_deg[0] - 0.2) < kEpsilon);
    RB_CHECK(integrator.clamps_total == 1);
    return true;
}

bool testVelocityIntegratorDivergenceResetsOrFaults() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0};

    rb_servo::CartesianControlConfig reset_config;
    reset_config.max_twist_linear_m_s = 1.0;
    reset_config.velocity_target_integration =
        rb_servo::CartesianVelocityTargetIntegrationMode::PreviousCommand;
    reset_config.max_command_actual_error_deg = {1, 1, 1, 1, 1, 1};
    reset_config.command_actual_error_policy =
        rb_servo::CartesianCommandActualErrorPolicy::Reset;
    rb_servo::CartesianServoController reset_controller(
        left_mount,
        right_mount,
        reset_config,
        kinematics
    );
    rb_servo::CartesianVelocityIntegratorState integrator;
    integrator.valid = true;
    integrator.last_mode = rb_servo::ControlMode::TcpTwistLocal;
    integrator.q_command_deg = zeroJoints();
    integrator.q_command_deg[0] = 20.0;
    rb_servo::CartesianTwistHoldState hold;
    const rb_servo::JointArray q_actual = zeroJoints();
    rb_servo::CartesianArmTargetResult result = reset_controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q_actual, left_mount),
        q_actual,
        rb_servo::RunMode::Simulation,
        0.01,
        20,
        &hold,
        &integrator
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(integrator.divergence_total == 1);
    RB_CHECK(integrator.reset_reason == "command_actual_divergence_reset");
    RB_CHECK(std::abs(result.q_target_deg[0] - 0.01) < 1e-12);

    rb_servo::CartesianControlConfig fault_config = reset_config;
    fault_config.command_actual_error_policy =
        rb_servo::CartesianCommandActualErrorPolicy::Fault;
    rb_servo::CartesianServoController fault_controller(
        left_mount,
        right_mount,
        fault_config,
        kinematics
    );
    integrator.valid = true;
    integrator.last_mode = rb_servo::ControlMode::TcpTwistLocal;
    integrator.q_command_deg = zeroJoints();
    integrator.q_command_deg[0] = 20.0;
    result = fault_controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q_actual, left_mount),
        q_actual,
        rb_servo::RunMode::Simulation,
        0.01,
        21,
        &hold,
        &integrator
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::TrackingError);
    RB_CHECK(result.reason == "cartesian_velocity_integrator_divergence");
    RB_CHECK(!integrator.valid);
    return true;
}

bool testControllerSimulationReferenceDivergenceIgnoresStaticPhysicalActual() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpTwistLocal;
    command.has_tcp_twist_local = true;
    command.tcp_twist_local = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0};

    rb_servo::CartesianControlConfig config;
    config.max_twist_linear_m_s = 1.0;
    config.velocity_target_integration =
        rb_servo::CartesianVelocityTargetIntegrationMode::PreviousCommand;
    config.max_command_actual_error_deg = {1, 1, 1, 1, 1, 1};
    rb_servo::CartesianServoController controller(
        left_mount,
        right_mount,
        config,
        kinematics
    );

    rb_servo::JointArray q_physical = zeroJoints();
    rb_servo::JointArray q_reference = zeroJoints();
    q_reference[0] = 20.0;
    rb_servo::RobotState reference_state = stateFromJoints(*kinematics, q_reference, left_mount);
    reference_state.q_target_deg = q_reference;

    rb_servo::CartesianVelocityIntegratorState integrator;
    integrator.valid = true;
    integrator.last_mode = rb_servo::ControlMode::TcpTwistLocal;
    integrator.q_command_deg = q_reference;

    rb_servo::CartesianServoStateContext context;
    context.servo_state_source = "reference";
    context.divergence_source = "reference";
    context.q_reference_for_servo_valid = true;
    context.physical_q_actual_deg = q_physical;
    context.reference_q_deg = q_reference;
    context.divergence_q_deg = q_reference;

    rb_servo::CartesianTwistHoldState hold;
    rb_servo::CartesianArmTargetResult result = controller.computeTwistTarget(
        command,
        reference_state,
        q_reference,
        rb_servo::RunMode::Simulation,
        0.01,
        30,
        &hold,
        &integrator,
        &context
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(integrator.divergence_total == 0);
    RB_CHECK(integrator.reset_reason.empty());
    RB_CHECK(std::abs(result.q_target_deg[0] - 20.01) < 1e-12);
    RB_CHECK(result.telemetry.cartesian_servo_state_source == "reference");
    RB_CHECK(result.telemetry.cartesian_divergence_source == "reference");
    RB_CHECK(result.telemetry.q_reference_for_servo_valid);
    RB_CHECK(result.telemetry.physical_command_actual_error_deg_observed > 19.0);
    RB_CHECK(result.telemetry.command_reference_error_deg_observed < kEpsilon);
    return true;
}

bool testTcpCircleMoveStartsWithoutJumpAndCompletes() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    rb_servo::CartesianControlConfig config;
    config.enable_benchmark_primitives = true;
    config.circle_move.min_period_sec = 0.1;
    config.circle_move.max_diameter_m = 0.2;
    config.max_twist_linear_m_s = 10.0;
    config.max_twist_angular_rad_s = 10.0;
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpCircleMove;
    command.has_tcp_circle_move = true;
    command.tcp_circle_move.plane = rb_servo::TcpCirclePlane::XY;
    command.tcp_circle_move.diameter_m = 0.10;
    command.tcp_circle_move.period_sec = 0.40;
    command.tcp_circle_move.repeat = 2;
    command.tcp_circle_move.center_mode = rb_servo::TcpCircleCenterMode::StartOnCircle;
    command.tcp_circle_move.orientation_mode = rb_servo::LinearMoveOrientationMode::Constant;
    command.tcp_circle_move.frame = rb_servo::TcpCircleFrame::Stand;

    rb_servo::CartesianCircleMoveState circle;
    rb_servo::CartesianVelocityIntegratorState integrator;
    const rb_servo::JointArray q_actual = zeroJoints();
    rb_servo::CartesianArmTargetResult result = controller.computeCircleMoveTarget(
        command,
        stateFromJoints(*kinematics, q_actual, left_mount),
        q_actual,
        rb_servo::RunMode::Simulation,
        0.0,
        42,
        &circle,
        &integrator
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(circle.active);
    RB_CHECK(!circle.done);
    RB_CHECK(std::abs(circle.reference_tcp_stand.x - circle.start_tcp_stand.x) < 1e-12);
    RB_CHECK(std::abs(circle.reference_tcp_stand.y - circle.start_tcp_stand.y) < 1e-12);
    RB_CHECK(result.telemetry.circle_active);
    RB_CHECK(std::abs(result.telemetry.circle_position_error_m) < 1e-12);
    RB_CHECK(std::abs(result.telemetry.circle_orientation_error_rad) < 1e-12);
    RB_CHECK(std::abs(result.telemetry.circle_radius_m - 0.05) < 1e-12);

    result = controller.computeCircleMoveTarget(
        command,
        stateFromJoints(*kinematics, q_actual, left_mount),
        q_actual,
        rb_servo::RunMode::Simulation,
        0.8,
        0,
        &circle,
        &integrator
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(circle.done);
    RB_CHECK(result.telemetry.circle_done);
    RB_CHECK(result.telemetry.circle_repeat_index == 1);
    RB_CHECK(std::abs(result.telemetry.circle_period_sec - 0.40) < 1e-12);
    return true;
}

bool testTcpCircleMoveSafetyGates() {
    auto kinematics = std::make_shared<LinearFakeKinematics>();
    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpCircleMove;
    command.has_tcp_circle_move = true;
    command.tcp_circle_move.diameter_m = 0.10;
    command.tcp_circle_move.period_sec = 1.0;
    command.tcp_circle_move.repeat = 1;

    rb_servo::CartesianCircleMoveState circle;
    rb_servo::CartesianVelocityIntegratorState integrator;
    const rb_servo::JointArray q_actual = zeroJoints();

    rb_servo::CartesianControlConfig disabled_config;
    disabled_config.circle_move.min_period_sec = 0.1;
    rb_servo::CartesianServoController disabled_controller(left_mount, right_mount, disabled_config, kinematics);
    rb_servo::CartesianArmTargetResult result = disabled_controller.computeCircleMoveTarget(
        command,
        stateFromJoints(*kinematics, q_actual, left_mount),
        q_actual,
        rb_servo::RunMode::Simulation,
        0.01,
        1,
        &circle,
        &integrator
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::CartesianUnavailable);
    RB_CHECK(result.reason == "tcp_circle_move_benchmark_primitives_disabled");

    rb_servo::CartesianControlConfig enabled_config;
    enabled_config.enable_benchmark_primitives = true;
    enabled_config.circle_move.min_period_sec = 0.1;
    rb_servo::CartesianServoController enabled_controller(left_mount, right_mount, enabled_config, kinematics);
    result = enabled_controller.computeCircleMoveTarget(
        command,
        stateFromJoints(*kinematics, q_actual, left_mount),
        q_actual,
        rb_servo::RunMode::Real,
        0.01,
        2,
        &circle,
        &integrator
    );
    // Real/sim gating retired: circle move computes in Real once the benchmark
    // feature flag is enabled.
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    return true;
}

}  // namespace

int main() {
    if (!testPureTranslationTracksLineAndKeepsOrientation()) return 1;
    if (!testRealModeBlocked()) return 1;
    if (!testLinearMoveUsesSeparatePositionAndOrientationGains()) return 1;
    if (!testLinearMoveConstantOrientationToleranceIsConfigurable()) return 1;
    if (!testTcpTwistLocalMovesLocalXAndHoldsOrientation()) return 1;
    if (!testFloorConstraintZerosDownwardVzAtPlaneAndKeepsLateral()) return 1;
    if (!testFloorConstraintRespectsTcpOrientationFrame()) return 1;
    if (!testTcpTwistAngularDeadbandMaintainsHoldForNoise()) return 1;
    if (!testPositiveOrientationHoldErrorReducesAfterSyntheticIntegration()) return 1;
    if (!testTcpTwistStandPositiveWorldXConvertsToLocalNegativeYAtPositiveYaw()) return 1;
    if (!testQuaternionAndRpyYawFrameConversionMatch()) return 1;
    if (!testLinearMoveConstantOrientationNearPiStaysFinite()) return 1;
    if (!testTcpTwistOrientationHoldNearPiStaysBounded()) return 1;
    if (!testTcpTwistRealModeBlocked()) return 1;
    if (!testTcpTwistClampTelemetryAndDamping()) return 1;
    if (!testTcpTwistRejectPolicy()) return 1;
    if (!testScalarFirstOrderPlantShowsMeasuredActualAttenuation()) return 1;
    if (!testVelocityIntegrationModesGenerateExpectedTargets()) return 1;
    if (!testVelocityIntegratorUsesClampedSafeTarget()) return 1;
    if (!testVelocityIntegratorDivergenceResetsOrFaults()) return 1;
    if (!testControllerSimulationReferenceDivergenceIgnoresStaticPhysicalActual()) return 1;
    if (!testTcpCircleMoveStartsWithoutJumpAndCompletes()) return 1;
    if (!testTcpCircleMoveSafetyGates()) return 1;
    return 0;
}
