#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unistd.h>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/cartesian_controller.hpp"
#include "rb_servo/kinematics/ik_solver.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"
#include "rb_servo/math/se3.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

std::filesystem::path servoRoot() {
    return std::filesystem::path(__FILE__).parent_path().parent_path();
}

std::filesystem::path rb3Urdf() {
    return servoRoot() / "descriptions" / "urdf" / "rb3_730e.urdf";
}

std::string writeTempConfig(const std::string& name, const std::string& body) {
    const std::string path = "/tmp/rb-servo-ik-" + name + "-" + std::to_string(getpid()) + ".yaml";
    std::ofstream file(path);
    file << body;
    return path;
}

bool loadRejects(const std::string& path) {
    try {
        (void)rb_servo::loadConfigFromYaml(path);
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

rb_servo::KinematicsConfig testKinematicsConfig() {
    rb_servo::KinematicsConfig cfg;
    cfg.enable = true;
    cfg.provider = "pinocchio";
    cfg.urdf = rb3Urdf().string();
    cfg.base_link = "world";
    cfg.tip_link = "tcp";
    cfg.joint_names = {
        "base_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint",
    };
    cfg.q_units = "deg";
    cfg.publish_tcp = true;
    cfg.ik.enable = true;
    cfg.ik.timeout_ms = 250.0;
    return cfg;
}

rb_servo::ArmMountConfig leftMount() {
    rb_servo::ArmMountConfig mount;
    mount.arm_id = rb_servo::ArmId::Left;
    mount.base_pose_in_stand = {0.1601, -0.1725, 0.5825, 2.186649, 0.523831, 2.526296};
    return mount;
}

rb_servo::ArmMountConfig rightMount() {
    rb_servo::ArmMountConfig mount;
    mount.arm_id = rb_servo::ArmId::Right;
    mount.base_pose_in_stand = {-0.1601, -0.1725, 0.5825, 2.186649, -0.523831, -2.526296};
    return mount;
}

rb_servo::JointArray seedJoints() {
    return {10.0, -20.0, 35.0, 5.0, 25.0, -15.0};
}

bool finiteJoints(const rb_servo::JointArray& joints) {
    for (double joint : joints) {
        if (!std::isfinite(joint)) return false;
    }
    return true;
}

bool closeJoints(const rb_servo::JointArray& a, const rb_servo::JointArray& b, double tolerance_deg) {
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::fabs(a[i] - b[i]) > tolerance_deg) return false;
    }
    return true;
}

bool withinUrdfLimits(const rb_servo::JointArray& q_deg) {
    constexpr double kWideLimitDeg = 360.001;
    constexpr double kElbowLimitDeg = 150.001;
    return std::fabs(q_deg[0]) <= kWideLimitDeg &&
           std::fabs(q_deg[1]) <= kWideLimitDeg &&
           std::fabs(q_deg[2]) <= kElbowLimitDeg &&
           std::fabs(q_deg[3]) <= kWideLimitDeg &&
           std::fabs(q_deg[4]) <= kWideLimitDeg &&
           std::fabs(q_deg[5]) <= kWideLimitDeg;
}

double positionDistance(const rb_servo::Pose6D& a, const rb_servo::Pose6D& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string validIkYaml(const std::string& urdf_path) {
    return
        "schema: robotics_lab.rb_servo_server.v1\n"
        "kinematics:\n"
        "  enable: true\n"
        "  provider: pinocchio\n"
        "  urdf: \"" + urdf_path + "\"\n"
        "  base_link: world\n"
        "  tip_link: tcp\n"
        "  q_units: deg\n"
        "  publish_tcp: true\n"
        "  ik:\n"
        "    enable: true\n"
        "    max_iterations: 25\n"
        "    timeout_ms: 10.0\n"
        "    damping: 0.002\n"
        "    position_tolerance_m: 0.002\n"
        "    orientation_tolerance_rad: 0.03\n"
        "    max_step_deg: [1, 1, 1, 2, 2, 3]\n"
        "    singular_region_eps: 0.05\n"
        "    damping_max: 0.06\n"
        "    max_solution_jump_deg: 3.0\n"
        "cartesian_control:\n"
        "  warn_ik_duration_us: 2500\n"
        "  fail_ik_duration_us: 4500\n"
        "  path_kp: 7.0\n"
        "  twist_orientation_hold_kp: 8.0\n"
        "  velocity_damping: 0.02\n"
        "  max_twist_linear_m_s: 0.04\n"
        "  max_twist_angular_rad_s: 0.25\n"
        "  max_linear_move_speed_m_s: 0.06\n"
        "  max_angular_move_speed_rad_s: 0.35\n"
        "  linear_move:\n"
        "    constant_orientation_tolerance_rad: 0.004\n"
        "  max_cartesian_step_m: 0.003\n"
        "  max_cartesian_step_rad: 0.03\n"
        "  exceed_limit_policy: reject\n";
}

class LatencyKinematics final : public rb_servo::IKinematics {
public:
    explicit LatencyKinematics(double duration_us) : duration_us_(duration_us) {}

    rb_servo::Pose6D computeTcpBase(const rb_servo::JointArray& q_deg) const override {
        return {q_deg[0] * 0.001, q_deg[1] * 0.001, 0.4, 0.0, 0.0, 0.0};
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
        result.duration_us = duration_us_;
        result.iterations = 3;
        result.reason = "ok";
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
        (void)tcp_twist_local;
        (void)damping;
        rb_servo::CartesianVelocityResult result;
        result.success = true;
        return result;
    }

private:
    double duration_us_;
};

bool testIkConfigParsing() {
    const std::string path = writeTempConfig("valid", validIkYaml(rb3Urdf().string()));
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());

    RB_CHECK(cfg.kinematics.ik.enable);
    RB_CHECK(cfg.kinematics.ik.max_iterations == 25);
    RB_CHECK(std::fabs(cfg.kinematics.ik.timeout_ms - 10.0) < 1e-12);
    RB_CHECK(std::fabs(cfg.kinematics.ik.damping - 0.002) < 1e-12);
    RB_CHECK(std::fabs(cfg.kinematics.ik.position_tolerance_m - 0.002) < 1e-12);
    RB_CHECK(std::fabs(cfg.kinematics.ik.orientation_tolerance_rad - 0.03) < 1e-12);
    RB_CHECK(cfg.kinematics.ik.max_step_deg[0] == 1.0);
    RB_CHECK(cfg.kinematics.ik.max_step_deg[5] == 3.0);
    RB_CHECK(std::fabs(cfg.kinematics.ik.singular_region_eps - 0.05) < 1e-12);
    RB_CHECK(std::fabs(cfg.kinematics.ik.damping_max - 0.06) < 1e-12);
    RB_CHECK(std::fabs(cfg.kinematics.ik.max_solution_jump_deg - 3.0) < 1e-12);
    RB_CHECK(std::fabs(cfg.cartesian_control.warn_ik_duration_us - 2500.0) < 1e-12);
    RB_CHECK(std::fabs(cfg.cartesian_control.fail_ik_duration_us - 4500.0) < 1e-12);
    RB_CHECK(std::fabs(cfg.cartesian_control.path_kp - 7.0) < 1e-12);
    RB_CHECK(std::fabs(cfg.cartesian_control.twist_orientation_hold_kp - 8.0) < 1e-12);
    RB_CHECK(std::fabs(cfg.cartesian_control.velocity_damping - 0.02) < 1e-12);
    RB_CHECK(std::fabs(cfg.cartesian_control.max_twist_linear_m_s - 0.04) < 1e-12);
    RB_CHECK(std::fabs(cfg.cartesian_control.max_twist_angular_rad_s - 0.25) < 1e-12);
    RB_CHECK(std::fabs(cfg.cartesian_control.max_linear_move_speed_m_s - 0.06) < 1e-12);
    RB_CHECK(std::fabs(cfg.cartesian_control.max_angular_move_speed_rad_s - 0.35) < 1e-12);
    RB_CHECK(std::fabs(cfg.cartesian_control.linear_move.constant_orientation_tolerance_rad - 0.004) < 1e-12);
    RB_CHECK(cfg.cartesian_control.max_cartesian_step_m.has_value());
    RB_CHECK(std::fabs(*cfg.cartesian_control.max_cartesian_step_m - 0.003) < 1e-12);
    RB_CHECK(cfg.cartesian_control.max_cartesian_step_rad.has_value());
    RB_CHECK(std::fabs(*cfg.cartesian_control.max_cartesian_step_rad - 0.03) < 1e-12);
    RB_CHECK(cfg.cartesian_control.exceed_limit_policy == rb_servo::CartesianLimitPolicy::Reject);

    const std::string bad_iter_path = writeTempConfig(
        "bad-iter",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "kinematics:\n"
        "  enable: true\n"
        "  provider: pinocchio\n"
        "  urdf: \"" + rb3Urdf().string() + "\"\n"
        "  ik:\n"
        "    max_iterations: 0\n"
    );
    const bool bad_iter_rejected = loadRejects(bad_iter_path);
    ::unlink(bad_iter_path.c_str());
    RB_CHECK(bad_iter_rejected);

    const std::string bad_cartesian_key_path = writeTempConfig(
        "bad-cart-key",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  not_a_limit: 1.0\n"
    );
    const bool bad_cartesian_key_rejected = loadRejects(bad_cartesian_key_path);
    ::unlink(bad_cartesian_key_path.c_str());
    RB_CHECK(bad_cartesian_key_rejected);

    const std::string bad_cartesian_limit_path = writeTempConfig(
        "bad-cart-limit",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  max_twist_linear_m_s: -0.01\n"
    );
    const bool bad_cartesian_limit_rejected = loadRejects(bad_cartesian_limit_path);
    ::unlink(bad_cartesian_limit_path.c_str());
    RB_CHECK(bad_cartesian_limit_rejected);
    return true;
}

bool testCartesianLatencyBudgetTelemetry() {
    rb_servo::CartesianControlConfig cfg;
    cfg.enable = true;
    cfg.allow_in_simulation = true;
    cfg.allow_in_real = false;
    cfg.warn_ik_duration_us = 100.0;
    cfg.fail_ik_duration_us = 200.0;

    rb_servo::RobotState state;
    state.arm_id = rb_servo::ArmId::Left;
    state.has_valid_joint_state = true;
    state.q_actual_deg = seedJoints();
    state.fk_duration_us = 42.0;

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpPoseTarget;
    command.has_tcp_target = true;
    command.tcp_target_stand = {0.2, -0.1, 0.7, 0.01, 0.02, 0.03};

    rb_servo::CartesianController slow_sim_controller(
        leftMount(),
        rightMount(),
        cfg,
        std::make_shared<LatencyKinematics>(250.0)
    );
    const rb_servo::CartesianArmTargetResult slow_sim = slow_sim_controller.computeArmJointTarget(
        command,
        state,
        seedJoints(),
        rb_servo::RunMode::Simulation
    );
    RB_CHECK(slow_sim.verdict == rb_servo::SafetyVerdict::IkFailed);
    RB_CHECK(slow_sim.reason == "ik_duration_budget_exceeded");
    RB_CHECK(slow_sim.telemetry.attempted);
    RB_CHECK(!slow_sim.telemetry.success);
    RB_CHECK(slow_sim.telemetry.status == "failed");
    RB_CHECK(slow_sim.telemetry.fk_duration_us == 42.0);
    RB_CHECK(slow_sim.telemetry.ik_duration_us == 250.0);
    RB_CHECK(slow_sim.telemetry.ik_iterations == 3);
    RB_CHECK(slow_sim.telemetry.ik_warn_duration_exceeded);
    RB_CHECK(slow_sim.telemetry.ik_fail_duration_exceeded);
    RB_CHECK(slow_sim.telemetry.warn_ik_duration_us == 100.0);
    RB_CHECK(slow_sim.telemetry.fail_ik_duration_us == 200.0);
    RB_CHECK(closeJoints(slow_sim.q_target_deg, seedJoints(), 1e-12));

    rb_servo::CartesianController warned_controller(
        leftMount(),
        rightMount(),
        cfg,
        std::make_shared<LatencyKinematics>(150.0)
    );
    const rb_servo::CartesianArmTargetResult warned = warned_controller.computeArmJointTarget(
        command,
        state,
        seedJoints(),
        rb_servo::RunMode::Simulation
    );
    RB_CHECK(warned.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(warned.telemetry.success);
    RB_CHECK(warned.telemetry.ik_warn_duration_exceeded);
    RB_CHECK(!warned.telemetry.ik_fail_duration_exceeded);

    rb_servo::CartesianController real_mode_controller(
        leftMount(),
        rightMount(),
        cfg,
        std::make_shared<LatencyKinematics>(250.0)
    );
    const rb_servo::CartesianArmTargetResult real_mode = real_mode_controller.computeArmJointTarget(
        command,
        state,
        seedJoints(),
        rb_servo::RunMode::Real
    );
    RB_CHECK(real_mode.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(real_mode.telemetry.ik_warn_duration_exceeded);
    RB_CHECK(!real_mode.telemetry.ik_fail_duration_exceeded);
    return true;
}

bool testPinocchioIk() {
    rb_servo::PinocchioKinematics kin(testKinematicsConfig());
    const rb_servo::ArmMountConfig mount = leftMount();
    const rb_servo::JointArray seed = seedJoints();

    const rb_servo::Pose6D seed_pose = kin.computeTcpStand(rb_servo::ArmId::Left, seed, mount);
    const rb_servo::IkResult same_pose = kin.solveIk(
        rb_servo::ArmId::Left,
        seed_pose,
        seed,
        mount
    );
    RB_CHECK(same_pose.success);
    RB_CHECK(closeJoints(same_pose.q_solution_deg, seed, 1e-6));
    RB_CHECK(std::isfinite(same_pose.duration_us));
    RB_CHECK(same_pose.duration_us >= 0.0);
    RB_CHECK(same_pose.iterations >= 0);
    RB_CHECK(!same_pose.timed_out);
    RB_CHECK(same_pose.position_error_m <= testKinematicsConfig().ik.position_tolerance_m);
    RB_CHECK(same_pose.orientation_error_rad <= testKinematicsConfig().ik.orientation_tolerance_rad);

    rb_servo::JointArray roundtrip_target_q = seed;
    roundtrip_target_q[0] += 0.5;
    roundtrip_target_q[1] -= 0.5;
    roundtrip_target_q[2] += 0.5;
    const rb_servo::Pose6D roundtrip_target_pose = kin.computeTcpStand(
        rb_servo::ArmId::Left,
        roundtrip_target_q,
        mount
    );
    const rb_servo::IkResult roundtrip = kin.solveIk(
        rb_servo::ArmId::Left,
        roundtrip_target_pose,
        seed,
        mount
    );
    RB_CHECK(roundtrip.success);
    RB_CHECK(finiteJoints(roundtrip.q_solution_deg));
    RB_CHECK(roundtrip.position_error_m <= testKinematicsConfig().ik.position_tolerance_m);
    RB_CHECK(roundtrip.orientation_error_rad <= testKinematicsConfig().ik.orientation_tolerance_rad);
    const rb_servo::Pose6D roundtrip_solution_pose = kin.computeTcpStand(
        rb_servo::ArmId::Left,
        roundtrip.q_solution_deg,
        mount
    );
    RB_CHECK(positionDistance(roundtrip_solution_pose, roundtrip_target_pose) <= testKinematicsConfig().ik.position_tolerance_m);

    rb_servo::JointArray nearby_target_q = seed;
    nearby_target_q[0] += 1.0;
    nearby_target_q[2] += 1.0;
    const rb_servo::Pose6D nearby_pose = kin.computeTcpStand(rb_servo::ArmId::Left, nearby_target_q, mount);
    const rb_servo::IkResult nearby = kin.solveIk(
        rb_servo::ArmId::Left,
        nearby_pose,
        seed,
        mount
    );
    RB_CHECK(nearby.success);
    RB_CHECK(finiteJoints(nearby.q_solution_deg));
    RB_CHECK(withinUrdfLimits(nearby.q_solution_deg));
    RB_CHECK(nearby.orientation_error_rad <= testKinematicsConfig().ik.orientation_tolerance_rad);

    rb_servo::Pose6D translated_pose = seed_pose;
    translated_pose.x += 0.005;
    const rb_servo::IkResult pure_translation = kin.solveIk(
        rb_servo::ArmId::Left,
        translated_pose,
        seed,
        mount
    );
    RB_CHECK(pure_translation.success);
    RB_CHECK(finiteJoints(pure_translation.q_solution_deg));
    RB_CHECK(pure_translation.position_error_m <= testKinematicsConfig().ik.position_tolerance_m);
    RB_CHECK(pure_translation.orientation_error_rad <= testKinematicsConfig().ik.orientation_tolerance_rad);
    RB_CHECK(pure_translation.orientation_error_rad <= 0.005);

    rb_servo::Pose6D pure_orientation_pose = seed_pose;
    pure_orientation_pose.quaternion_xyzw.reset();
    pure_orientation_pose.rz += 0.005;
    const rb_servo::IkResult pure_orientation = kin.solveIk(
        rb_servo::ArmId::Left,
        pure_orientation_pose,
        seed,
        mount
    );
    RB_CHECK(pure_orientation.success);
    RB_CHECK(finiteJoints(pure_orientation.q_solution_deg));
    RB_CHECK(pure_orientation.position_error_m <= testKinematicsConfig().ik.position_tolerance_m);
    RB_CHECK(pure_orientation.orientation_error_rad <= testKinematicsConfig().ik.orientation_tolerance_rad);

    rb_servo::Pose6D unreachable = seed_pose;
    unreachable.x += 10.0;
    const rb_servo::IkResult unreachable_result = kin.solveIk(
        rb_servo::ArmId::Left,
        unreachable,
        seed,
        mount
    );
    RB_CHECK(!unreachable_result.success);
    RB_CHECK(!unreachable_result.reason.empty());
    RB_CHECK(std::isfinite(unreachable_result.duration_us));
    RB_CHECK(unreachable_result.duration_us >= 0.0);
    RB_CHECK(unreachable_result.iterations >= 0);
    RB_CHECK(finiteJoints(unreachable_result.q_solution_deg));
    RB_CHECK(std::isfinite(unreachable_result.position_error_m));
    RB_CHECK(std::isfinite(unreachable_result.orientation_error_rad));

    rb_servo::KinematicsConfig one_iter_cfg = testKinematicsConfig();
    one_iter_cfg.ik.max_iterations = 1;
    rb_servo::PinocchioKinematics one_iter_kin(one_iter_cfg);
    rb_servo::JointArray farther_target_q = seed;
    farther_target_q[1] += 8.0;
    farther_target_q[2] -= 8.0;
    const rb_servo::Pose6D farther_pose = one_iter_kin.computeTcpStand(
        rb_servo::ArmId::Left,
        farther_target_q,
        mount
    );
    const rb_servo::IkResult non_converged = one_iter_kin.solveIk(
        rb_servo::ArmId::Left,
        farther_pose,
        seed,
        mount
    );
    RB_CHECK(!non_converged.success);
    RB_CHECK(std::isfinite(non_converged.duration_us));
    RB_CHECK(non_converged.duration_us >= 0.0);
    RB_CHECK(non_converged.iterations >= 0);
    RB_CHECK(non_converged.reason == rb_servo::ik_solver::kReasonMaxIterations ||
             non_converged.reason == rb_servo::ik_solver::kReasonJointLimit);

    rb_servo::KinematicsConfig disabled_cfg = testKinematicsConfig();
    disabled_cfg.ik.enable = false;
    rb_servo::PinocchioKinematics disabled_kin(disabled_cfg);
    const rb_servo::IkResult disabled = disabled_kin.solveIk(
        rb_servo::ArmId::Left,
        seed_pose,
        seed,
        mount
    );
    RB_CHECK(!disabled.success);
    RB_CHECK(disabled.reason == rb_servo::ik_solver::kReasonKinematicsUnavailable);
    RB_CHECK(std::isfinite(disabled.duration_us));
    RB_CHECK(disabled.duration_us >= 0.0);

    std::cout << "[IK_LATENCY] same_pose_duration_us=" << same_pose.duration_us
              << " same_pose_iterations=" << same_pose.iterations
              << " roundtrip_duration_us=" << roundtrip.duration_us
              << " roundtrip_iterations=" << roundtrip.iterations
              << " pure_translation_duration_us=" << pure_translation.duration_us
              << " pure_translation_iterations=" << pure_translation.iterations
              << " pure_translation_orientation_error_rad=" << pure_translation.orientation_error_rad
              << " pure_orientation_duration_us=" << pure_orientation.duration_us
              << " pure_orientation_iterations=" << pure_orientation.iterations
              << " pure_orientation_position_error_m=" << pure_orientation.position_error_m
              << " nearby_duration_us=" << nearby.duration_us
              << " nearby_iterations=" << nearby.iterations
              << " unreachable_duration_us=" << unreachable_result.duration_us
              << " unreachable_iterations=" << unreachable_result.iterations
              << " non_converged_duration_us=" << non_converged.duration_us
              << " non_converged_iterations=" << non_converged.iterations
              << "\n";
    return true;
}

bool testInvalidTargetDoesNotThrow() {
    rb_servo::PinocchioKinematics kin(testKinematicsConfig());
    rb_servo::Pose6D target;
    target.x = std::nan("");
    const rb_servo::IkResult result = kin.solveIk(
        rb_servo::ArmId::Left,
        target,
        seedJoints(),
        leftMount()
    );
    RB_CHECK(!result.success);
    RB_CHECK(result.reason == rb_servo::ik_solver::kReasonInvalidTarget);
    RB_CHECK(std::isfinite(result.duration_us));
    RB_CHECK(result.duration_us >= 0.0);
    return true;
}

bool testIkSeedUsesPreviousSentTargetNotActualState() {
    // The streaming TcpPoseTarget path must seed IK from the previously SENT
    // target, not the measured joint state: an actual-state seed feeds the
    // robot's lagged physical response back into the next command (3-5 Hz
    // relay limit cycle with the IK tolerance dead zone at 500 Hz).
    rb_servo::CartesianControlConfig cfg;
    cfg.enable = true;
    cfg.allow_in_simulation = true;
    cfg.allow_in_real = false;

    rb_servo::RobotState state;
    state.arm_id = rb_servo::ArmId::Left;
    state.has_valid_joint_state = true;
    state.q_actual_deg = seedJoints();
    state.q_actual_deg[1] += 5.0;  // physical state lags the sent target

    rb_servo::ArmCommand command;
    command.arm_id = rb_servo::ArmId::Left;
    command.mode = rb_servo::ControlMode::TcpPoseTarget;
    command.has_tcp_target = true;
    command.tcp_target_stand = {0.2, -0.1, 0.7, 0.01, 0.02, 0.03};

    rb_servo::CartesianController controller(
        leftMount(),
        rightMount(),
        cfg,
        std::make_shared<LatencyKinematics>(1.0)
    );
    // LatencyKinematics echoes its seed as the IK solution, so the result
    // exposes which seed was used.
    const rb_servo::CartesianArmTargetResult result = controller.computeArmJointTarget(
        command,
        state,
        seedJoints(),
        rb_servo::RunMode::Simulation
    );
    RB_CHECK(result.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(closeJoints(result.q_target_deg, seedJoints(), 1e-12));

    // Fallback: a non-finite previous target falls back to the actual state.
    rb_servo::JointArray invalid_previous = seedJoints();
    invalid_previous[0] = std::numeric_limits<double>::quiet_NaN();
    const rb_servo::CartesianArmTargetResult fallback = controller.computeArmJointTarget(
        command,
        state,
        invalid_previous,
        rb_servo::RunMode::Simulation
    );
    RB_CHECK(fallback.verdict == rb_servo::SafetyVerdict::Ok);
    RB_CHECK(closeJoints(fallback.q_target_deg, state.q_actual_deg, 1e-12));
    return true;
}

bool testIkConditioningDiagnosticsPopulated() {
    // A normal reachable solve must report a positive smallest singular value,
    // the base damping (no singular-region ramp away from singularities), and a
    // finite seed jump; the branch-jump flag must not fire for a small move.
    rb_servo::KinematicsConfig cfg = testKinematicsConfig();
    cfg.ik.singular_region_eps = 0.04;
    cfg.ik.damping_max = 0.05;
    cfg.ik.max_solution_jump_deg = 5.0;
    rb_servo::PinocchioKinematics kin(cfg);
    const rb_servo::ArmMountConfig mount = leftMount();
    const rb_servo::JointArray seed = seedJoints();

    rb_servo::JointArray target_q = seed;
    target_q[0] += 1.0;
    target_q[2] += 1.0;
    const rb_servo::Pose6D target_pose = kin.computeTcpStand(rb_servo::ArmId::Left, target_q, mount);
    const rb_servo::IkResult result = kin.solveIk(rb_servo::ArmId::Left, target_pose, seed, mount);

    RB_CHECK(result.success);
    RB_CHECK(std::isfinite(result.min_singular_value));
    RB_CHECK(result.min_singular_value > 0.0);
    // applied_damping is the base damping outside the singular region, ramping
    // up to at most sqrt(damping^2 + damping_max^2) inside it.
    const double max_applied = std::sqrt(cfg.ik.damping * cfg.ik.damping +
                                         cfg.ik.damping_max * cfg.ik.damping_max);
    RB_CHECK(result.applied_damping >= cfg.ik.damping - 1e-9);
    RB_CHECK(result.applied_damping <= max_applied + 1e-9);
    RB_CHECK(std::isfinite(result.solution_jump_deg));
    RB_CHECK(result.solution_jump_deg >= 0.0);
    RB_CHECK(!result.branch_jump_suspected);
    return true;
}

bool testIkBranchJumpGuardFlagsLargeSeedDelta() {
    // The observability guard flags (but does not alter) a solution that lands
    // far from the seed in one solve. Seed far from the target so the converged
    // solution differs by more than the threshold.
    rb_servo::KinematicsConfig cfg = testKinematicsConfig();
    cfg.ik.max_solution_jump_deg = 2.0;
    rb_servo::PinocchioKinematics kin(cfg);
    const rb_servo::ArmMountConfig mount = leftMount();
    const rb_servo::JointArray seed = seedJoints();

    rb_servo::JointArray target_q = seed;
    target_q[0] += 20.0;  // >> 2 deg on joint 0
    const rb_servo::Pose6D target_pose = kin.computeTcpStand(rb_servo::ArmId::Left, target_q, mount);
    const rb_servo::IkResult result = kin.solveIk(rb_servo::ArmId::Left, target_pose, seed, mount);

    RB_CHECK(result.success);
    RB_CHECK(result.solution_jump_deg > 2.0);
    RB_CHECK(result.branch_jump_suspected);
    // The guard is observability-only: the solution still reaches the target.
    RB_CHECK(result.position_error_m <= cfg.ik.position_tolerance_m);
    return true;
}

bool testIkBranchJumpClampHoldsSeed() {
    // With the clamp enabled (and no re-solve configured), a solution that jumps
    // past the threshold is REPLACED by the seed (zero motion this tick) instead
    // of flipping to a distant branch -- the actual correction, not just a flag.
    rb_servo::KinematicsConfig cfg = testKinematicsConfig();
    cfg.ik.max_solution_jump_deg = 2.0;
    cfg.ik.branch_jump_clamp_to_seed = true;  // re-solve off (scale/retries default 0)
    rb_servo::PinocchioKinematics kin(cfg);
    const rb_servo::ArmMountConfig mount = leftMount();
    const rb_servo::JointArray seed = seedJoints();

    rb_servo::JointArray target_q = seed;
    target_q[0] += 20.0;  // same >2 deg seed delta as the guard test
    const rb_servo::Pose6D target_pose = kin.computeTcpStand(rb_servo::ArmId::Left, target_q, mount);
    const rb_servo::IkResult result = kin.solveIk(rb_servo::ArmId::Left, target_pose, seed, mount);

    RB_CHECK(result.success);
    RB_CHECK(result.branch_jump_clamped);
    RB_CHECK(result.branch_jump_suspected);
    RB_CHECK(closeJoints(result.q_solution_deg, seed, 1e-12));  // held the seed
    RB_CHECK(result.solution_jump_deg <= 1e-9);
    return true;
}

bool testIkBranchJumpClampDefaultOffLeavesSolutionUnchanged() {
    // Clamp fields default off => identical to the observability path: the
    // jumping solution is returned as-is (reaches target, not held).
    rb_servo::KinematicsConfig cfg = testKinematicsConfig();
    cfg.ik.max_solution_jump_deg = 2.0;  // clamp fields left at defaults
    rb_servo::PinocchioKinematics kin(cfg);
    const rb_servo::ArmMountConfig mount = leftMount();
    const rb_servo::JointArray seed = seedJoints();

    rb_servo::JointArray target_q = seed;
    target_q[0] += 20.0;
    const rb_servo::Pose6D target_pose = kin.computeTcpStand(rb_servo::ArmId::Left, target_q, mount);
    const rb_servo::IkResult result = kin.solveIk(rb_servo::ArmId::Left, target_pose, seed, mount);

    RB_CHECK(result.success);
    RB_CHECK(!result.branch_jump_clamped);
    RB_CHECK(result.solution_jump_deg > 2.0);
    RB_CHECK(result.position_error_m <= cfg.ik.position_tolerance_m);  // reached target
    return true;
}

bool testIkBranchJumpRateLimitBoundsStepTowardSolution() {
    // With rate-limit enabled, a solution that jumps past the threshold is NOT
    // held at the seed (no deadlock) and NOT accepted whole (no abrupt flip):
    // the seed->solution joint delta is scaled so the largest per-joint step
    // equals max_solution_jump_deg, advancing along the same direction.
    rb_servo::KinematicsConfig cfg = testKinematicsConfig();
    cfg.ik.max_solution_jump_deg = 2.0;
    cfg.ik.branch_jump_rate_limit = true;  // re-solve off (scale/retries default 0)
    rb_servo::PinocchioKinematics kin(cfg);
    const rb_servo::ArmMountConfig mount = leftMount();
    const rb_servo::JointArray seed = seedJoints();

    rb_servo::JointArray target_q = seed;
    target_q[0] += 20.0;  // same >2 deg seed delta as the clamp test
    const rb_servo::Pose6D target_pose = kin.computeTcpStand(rb_servo::ArmId::Left, target_q, mount);

    // Reference: the unaltered (jumping) solution, for direction comparison.
    rb_servo::KinematicsConfig allow_cfg = cfg;
    allow_cfg.ik.branch_jump_rate_limit = false;  // clamp fields all off => allow
    rb_servo::PinocchioKinematics allow_kin(allow_cfg);
    const rb_servo::IkResult full =
        allow_kin.solveIk(rb_servo::ArmId::Left, target_pose, seed, mount);
    RB_CHECK(full.success && full.solution_jump_deg > 2.0);

    const rb_servo::IkResult result = kin.solveIk(rb_servo::ArmId::Left, target_pose, seed, mount);
    RB_CHECK(result.success);
    RB_CHECK(result.branch_jump_suspected);
    RB_CHECK(!result.branch_jump_clamped);          // not held at the seed
    RB_CHECK(result.reason == "branch_jump_rate_limited");

    // Largest per-joint step is bounded to the threshold, and it actually moved.
    double max_abs = 0.0;
    for (std::size_t i = 0; i < rb_servo::kDof; ++i) {
        max_abs = std::max(max_abs, std::fabs(result.q_solution_deg[i] - seed[i]));
    }
    RB_CHECK(max_abs > 1e-6);                        // advanced (no deadlock)
    RB_CHECK(std::fabs(max_abs - 2.0) < 1e-6);       // capped at threshold
    RB_CHECK(std::fabs(result.solution_jump_deg - 2.0) < 1e-6);

    // Direction preserved: result == seed + s*(full - seed) for one scalar s.
    const double s = 2.0 / full.solution_jump_deg;
    for (std::size_t i = 0; i < rb_servo::kDof; ++i) {
        const double expected = seed[i] + (full.q_solution_deg[i] - seed[i]) * s;
        RB_CHECK(std::fabs(result.q_solution_deg[i] - expected) < 1e-9);
    }
    return true;
}

bool testIkSelectiveDampingDisabledByDefault() {
    // With singular_region_eps/damping_max defaulting to 0, applied_damping is
    // always the base damping (behavior-preserving).
    rb_servo::PinocchioKinematics kin(testKinematicsConfig());
    const rb_servo::ArmMountConfig mount = leftMount();
    const rb_servo::JointArray seed = seedJoints();
    rb_servo::JointArray target_q = seed;
    target_q[1] += 1.0;
    const rb_servo::Pose6D target_pose = kin.computeTcpStand(rb_servo::ArmId::Left, target_q, mount);
    const rb_servo::IkResult result = kin.solveIk(rb_servo::ArmId::Left, target_pose, seed, mount);
    RB_CHECK(result.success);
    RB_CHECK(std::fabs(result.applied_damping - testKinematicsConfig().ik.damping) < 1e-9);
    RB_CHECK(!result.branch_jump_suspected);  // guard off by default
    return true;
}

}  // namespace

bool testFloorPointZJacobianFiniteDifference() {
    // Stage 3: validate computeFloorPointZJacobian (stand-frame z-velocity Jacobian
    // of a TCP-frame offset point) against a central finite difference of the
    // offset point's stand-frame z through computeTcpStand.
    rb_servo::PinocchioKinematics kin(testKinematicsConfig());
    for (const rb_servo::ArmMountConfig mount : {leftMount(), rightMount()}) {
        const rb_servo::JointArray base = seedJoints();
        const std::array<double, 3> offset = {0.059, 0.0, 0.03};  // tip-like, off-axis
        rb_servo::JointArray Jz{};
        RB_CHECK(kin.computeFloorPointZJacobian(rb_servo::ArmId::Left, base, mount, offset, Jz));
        const auto pz = [&](const rb_servo::JointArray& q) {
            const rb_servo::Pose6D tcp = kin.computeTcpStand(rb_servo::ArmId::Left, q, mount);
            const rb_servo::math::Matrix3 R = rb_servo::math::rotationFromPose(tcp);
            const rb_servo::math::Vector3 off(offset[0], offset[1], offset[2]);
            return tcp.z + (R * off).z();
        };
        const double k = 3.14159265358979323846 / 180.0;
        const double h = 0.2;  // deg
        double max_err = 0.0;
        for (int j = 0; j < rb_servo::kDof; ++j) {
            rb_servo::JointArray qp = base, qm = base;
            qp[j] += h;
            qm[j] -= h;
            const double fd = (pz(qp) - pz(qm)) / (2.0 * h * k);  // d(p_z)/dq_j [m/rad]
            max_err = std::max(max_err, std::abs(fd - Jz[j]));
        }
        std::cout << "floor Jz FD max err = " << max_err << " m/rad\n";
        RB_CHECK(max_err < 1e-3);
    }
    return true;
}

bool testStandAxisJacobianFiniteDifference() {
    // ROI box (Stage 3): validate computeStandAxisJacobian for all three stand
    // axes against a central finite difference of the offset point's stand-frame
    // coordinate through computeTcpStand. axis=2 must match the floor z Jacobian.
    rb_servo::PinocchioKinematics kin(testKinematicsConfig());
    for (const rb_servo::ArmMountConfig mount : {leftMount(), rightMount()}) {
        const rb_servo::JointArray base = seedJoints();
        const std::array<double, 3> offset = {0.059, 0.0, 0.03};  // tip-like, off-axis
        for (int axis = 0; axis < 3; ++axis) {
            rb_servo::JointArray Jaxis{};
            RB_CHECK(kin.computeStandAxisJacobian(rb_servo::ArmId::Left, base, mount, offset,
                                                  axis, Jaxis));
            const auto pk = [&](const rb_servo::JointArray& q) {
                const rb_servo::Pose6D tcp = kin.computeTcpStand(rb_servo::ArmId::Left, q, mount);
                const rb_servo::math::Matrix3 R = rb_servo::math::rotationFromPose(tcp);
                const rb_servo::math::Vector3 off(offset[0], offset[1], offset[2]);
                const std::array<double, 3> p{tcp.x + (R * off).x(), tcp.y + (R * off).y(),
                                              tcp.z + (R * off).z()};
                return p[axis];
            };
            const double k = 3.14159265358979323846 / 180.0;
            const double h = 0.2;  // deg
            double max_err = 0.0;
            for (int j = 0; j < rb_servo::kDof; ++j) {
                rb_servo::JointArray qp = base, qm = base;
                qp[j] += h;
                qm[j] -= h;
                const double fd = (pk(qp) - pk(qm)) / (2.0 * h * k);  // d(p_axis)/dq_j [m/rad]
                max_err = std::max(max_err, std::abs(fd - Jaxis[j]));
            }
            std::cout << "stand axis " << axis << " J FD max err = " << max_err << " m/rad\n";
            RB_CHECK(max_err < 1e-3);
        }
        // axis=2 must equal computeFloorPointZJacobian exactly (shared impl).
        rb_servo::JointArray Jz{}, Jaxis2{};
        RB_CHECK(kin.computeFloorPointZJacobian(rb_servo::ArmId::Left, base, mount, offset, Jz));
        RB_CHECK(kin.computeStandAxisJacobian(rb_servo::ArmId::Left, base, mount, offset, 2, Jaxis2));
        for (int j = 0; j < rb_servo::kDof; ++j) RB_CHECK(std::abs(Jz[j] - Jaxis2[j]) < 1e-12);
    }
    return true;
}

int main() {
    if (!testFloorPointZJacobianFiniteDifference()) return 1;
    if (!testStandAxisJacobianFiniteDifference()) return 1;
    if (!testIkConfigParsing()) return 1;
    if (!testIkConditioningDiagnosticsPopulated()) return 1;
    if (!testIkBranchJumpGuardFlagsLargeSeedDelta()) return 1;
    if (!testIkBranchJumpClampHoldsSeed()) return 1;
    if (!testIkBranchJumpClampDefaultOffLeavesSolutionUnchanged()) return 1;
    if (!testIkBranchJumpRateLimitBoundsStepTowardSolution()) return 1;
    if (!testIkSelectiveDampingDisabledByDefault()) return 1;
    if (!testCartesianLatencyBudgetTelemetry()) return 1;
    if (!testIkSeedUsesPreviousSentTargetNotActualState()) return 1;
    if (!testPinocchioIk()) return 1;
    if (!testInvalidTargetDoesNotThrow()) return 1;
    return 0;
}
