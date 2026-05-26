#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

#include "rb_servo/control/cartesian_servo_controller.hpp"

namespace {

constexpr double kEpsilon = 1e-9;

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
        (void)damping;
        rb_servo::CartesianVelocityResult result;
        result.success = true;
        result.qdot_deg_s[0] = tcp_twist_local.x * 100.0;
        result.qdot_deg_s[1] = tcp_twist_local.y * 100.0;
        result.qdot_deg_s[2] = tcp_twist_local.z * 100.0;
        result.qdot_deg_s[5] = tcp_twist_local.rz * 100.0;
        return result;
    }
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
    rb_servo::CartesianServoController controller(left_mount, right_mount, config, kinematics);

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpLinearMove;
    command.has_tcp_target = true;
    command.timeout_sec = 0.2;
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

}  // namespace

int main() {
    if (!testPureTranslationTracksLineAndKeepsOrientation()) return 1;
    if (!testRealModeBlocked()) return 1;
    if (!testTcpTwistLocalMovesLocalXAndHoldsOrientation()) return 1;
    if (!testTcpTwistRealModeBlocked()) return 1;
    return 0;
}
