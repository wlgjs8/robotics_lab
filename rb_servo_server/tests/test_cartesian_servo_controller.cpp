#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

#include "rb_servo/control/cartesian_servo_controller.hpp"

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
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::CartesianUnavailable);
    RB_CHECK(result.reason == "tcp_linear_move_simulation_only");
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
        &hold
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(!hold.orientation_hold_active);
    RB_CHECK(std::abs(kinematics->last_twist_local.rz - 0.002) < kEpsilon);
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
    const rb_servo::CartesianArmTargetResult result = controller.computeTwistTarget(
        command,
        stateFromJoints(*kinematics, q, left_mount),
        q,
        rb_servo::RunMode::Real,
        0.005,
        &hold
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::CartesianUnavailable);
    RB_CHECK(result.reason == "tcp_twist_simulation_only");
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
        &hold
    );

    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::InvalidCommand);
    RB_CHECK(result.reason == "cartesian_twist_limit_exceeded");
    RB_CHECK(result.telemetry.requested_twist_linear_norm_m_s > config.max_twist_linear_m_s);
    RB_CHECK(result.telemetry.applied_twist_linear_norm_m_s == 0.0);
    RB_CHECK(kinematics->velocity_call_count == 0);
    return true;
}

}  // namespace

int main() {
    if (!testPureTranslationTracksLineAndKeepsOrientation()) return 1;
    if (!testRealModeBlocked()) return 1;
    if (!testLinearMoveUsesSeparatePositionAndOrientationGains()) return 1;
    if (!testTcpTwistLocalMovesLocalXAndHoldsOrientation()) return 1;
    if (!testTcpTwistAngularDeadbandMaintainsHoldForNoise()) return 1;
    if (!testLinearMoveConstantOrientationNearPiStaysFinite()) return 1;
    if (!testTcpTwistOrientationHoldNearPiStaysBounded()) return 1;
    if (!testTcpTwistRealModeBlocked()) return 1;
    if (!testTcpTwistClampTelemetryAndDamping()) return 1;
    if (!testTcpTwistRejectPolicy()) return 1;
    return 0;
}
