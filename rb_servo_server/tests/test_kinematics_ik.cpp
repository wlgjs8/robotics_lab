#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include "rb_servo/config/config.hpp"
#include "rb_servo/kinematics/ik_solver.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"

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
    mount.base_pose_in_stand = {0.1601, -0.1725, 0.5825, 0.785, 2.35619, 0.0};
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
        "    max_step_deg: [1, 1, 1, 2, 2, 3]\n";
}

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
    return true;
}

bool testDisabledBuildReturnsUnavailable() {
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
    return true;
#else
    RB_CHECK(!rb_servo::PinocchioKinematics::isAvailable());
    rb_servo::PinocchioKinematics kin(testKinematicsConfig());
    const rb_servo::IkResult result = kin.solveIk(
        rb_servo::ArmId::Left,
        rb_servo::Pose6D{},
        seedJoints(),
        leftMount()
    );
    RB_CHECK(!result.success);
    RB_CHECK(result.reason == rb_servo::ik_solver::kReasonKinematicsUnavailable);
    return true;
#endif
}

bool testPinocchioIkIfEnabled() {
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
    RB_CHECK(rb_servo::PinocchioKinematics::isAvailable());
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
    RB_CHECK(same_pose.position_error_m <= testKinematicsConfig().ik.position_tolerance_m);
    RB_CHECK(same_pose.orientation_error_rad <= testKinematicsConfig().ik.orientation_tolerance_rad);

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
    RB_CHECK(finiteJoints(unreachable_result.q_solution_deg));

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
#endif
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
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
    RB_CHECK(result.reason == rb_servo::ik_solver::kReasonInvalidTarget);
#else
    RB_CHECK(result.reason == rb_servo::ik_solver::kReasonKinematicsUnavailable);
#endif
    return true;
}

}  // namespace

int main() {
    if (!testIkConfigParsing()) return 1;
    if (!testDisabledBuildReturnsUnavailable()) return 1;
    if (!testPinocchioIkIfEnabled()) return 1;
    if (!testInvalidTargetDoesNotThrow()) return 1;
    return 0;
}
