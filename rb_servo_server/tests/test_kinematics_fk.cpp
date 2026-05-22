#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <type_traits>
#include <unistd.h>

#include "rb_servo/config/config.hpp"
#include "rb_servo/kinematics/i_kinematics.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

template <typename T, typename = void>
struct HasComputeIk : std::false_type {};

template <typename T>
struct HasComputeIk<T, std::void_t<decltype(&T::computeIk)>> : std::true_type {};

std::filesystem::path servoRoot() {
    return std::filesystem::path(__FILE__).parent_path().parent_path();
}

std::filesystem::path rb3Urdf() {
    return servoRoot() / "descriptions" / "urdf" / "rb3_730e.urdf";
}

std::string writeTempConfig(const std::string& name, const std::string& body) {
    const std::string path = "/tmp/rb-servo-kinematics-" + name + "-" + std::to_string(getpid()) + ".yaml";
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

bool finitePose(const rb_servo::Pose6D& pose) {
    return std::isfinite(pose.x) &&
           std::isfinite(pose.y) &&
           std::isfinite(pose.z) &&
           std::isfinite(pose.rx) &&
           std::isfinite(pose.ry) &&
           std::isfinite(pose.rz);
}

bool differentPose(const rb_servo::Pose6D& a, const rb_servo::Pose6D& b) {
    return std::fabs(a.x - b.x) > 1e-9 ||
           std::fabs(a.y - b.y) > 1e-9 ||
           std::fabs(a.z - b.z) > 1e-9 ||
           std::fabs(a.rx - b.rx) > 1e-9 ||
           std::fabs(a.ry - b.ry) > 1e-9 ||
           std::fabs(a.rz - b.rz) > 1e-9;
}

std::string validKinematicsYaml(const std::string& urdf_path) {
    return
        "schema: robotics_lab.rb_servo_server.v1\n"
        "kinematics:\n"
        "  enable: true\n"
        "  provider: pinocchio\n"
        "  urdf: \"" + urdf_path + "\"\n"
        "  base_link: \"world\"\n"
        "  tip_link: \"tcp\"\n"
        "  joint_names:\n"
        "    - base_joint\n"
        "    - shoulder_joint\n"
        "    - elbow_joint\n"
        "    - wrist1_joint\n"
        "    - wrist2_joint\n"
        "    - wrist3_joint\n"
        "  q_units: deg\n"
        "  publish_tcp: true\n";
}

bool testKinematicsConfigValidation() {
    const std::string valid_path = writeTempConfig("valid", validKinematicsYaml(rb3Urdf().string()));
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(valid_path);
    ::unlink(valid_path.c_str());

    RB_CHECK(cfg.kinematics.enable);
    RB_CHECK(cfg.kinematics.provider == "pinocchio");
    RB_CHECK(cfg.kinematics.base_link == "world");
    RB_CHECK(cfg.kinematics.tip_link == "tcp");
    RB_CHECK(cfg.kinematics.joint_names.size() == rb_servo::kDof);
    RB_CHECK(cfg.kinematics.q_units == "deg");
    RB_CHECK(cfg.kinematics.publish_tcp);
    RB_CHECK(std::filesystem::is_regular_file(cfg.kinematics.urdf));

    const std::string bad_urdf_path = writeTempConfig(
        "bad-urdf",
        validKinematicsYaml("/tmp/does-not-exist-rb3-730e.urdf")
    );
    const bool bad_urdf_rejected = loadRejects(bad_urdf_path);
    ::unlink(bad_urdf_path.c_str());
    RB_CHECK(bad_urdf_rejected);

    const std::string bad_units_path = writeTempConfig(
        "bad-units",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "kinematics:\n"
        "  enable: true\n"
        "  provider: pinocchio\n"
        "  urdf: \"" + rb3Urdf().string() + "\"\n"
        "  q_units: rad\n"
    );
    const bool bad_units_rejected = loadRejects(bad_units_path);
    ::unlink(bad_units_path.c_str());
    RB_CHECK(bad_units_rejected);

    return true;
}

bool testIkRemainsUnavailable() {
    static_assert(!HasComputeIk<rb_servo::IKinematics>::value, "P2-A must not add IK to IKinematics");
    static_assert(!HasComputeIk<rb_servo::PinocchioKinematics>::value, "P2-A must not add IK to PinocchioKinematics");
    return true;
}

bool testDisabledBuildBehavior() {
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
    return true;
#else
    RB_CHECK(!rb_servo::PinocchioKinematics::isAvailable());
    rb_servo::KinematicsConfig cfg;
    cfg.enable = true;
    cfg.provider = "pinocchio";
    cfg.urdf = rb3Urdf().string();
    rb_servo::PinocchioKinematics kin(cfg);
    bool threw = false;
    try {
        (void)kin.computeTcpBase(rb_servo::JointArray{});
    } catch (const std::exception&) {
        threw = true;
    }
    RB_CHECK(threw);
    return true;
#endif
}

bool testPinocchioFkIfEnabled() {
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
    RB_CHECK(rb_servo::PinocchioKinematics::isAvailable());

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

    rb_servo::PinocchioKinematics kin(cfg);

    rb_servo::JointArray zero{};
    const rb_servo::Pose6D tcp_zero = kin.computeTcpBase(zero);
    RB_CHECK(finitePose(tcp_zero));

    rb_servo::JointArray base_90{};
    base_90[0] = 90.0;
    const rb_servo::Pose6D tcp_base_90 = kin.computeTcpBase(base_90);
    RB_CHECK(finitePose(tcp_base_90));
    RB_CHECK(differentPose(tcp_zero, tcp_base_90));

    rb_servo::ArmMountConfig mount;
    mount.arm_id = rb_servo::ArmId::Left;
    mount.base_pose_in_stand = {0.1601, -0.1725, 0.5825, 0.785, 2.35619, 0.0};
    const rb_servo::Pose6D tcp_stand = kin.computeTcpStand(rb_servo::ArmId::Left, zero, mount);
    RB_CHECK(finitePose(tcp_stand));

    cfg.tip_link = "missing_tip";
    bool bad_tip_threw = false;
    try {
        rb_servo::PinocchioKinematics bad_tip(cfg);
    } catch (const std::exception&) {
        bad_tip_threw = true;
    }
    RB_CHECK(bad_tip_threw);

    cfg.tip_link = "tcp";
    cfg.joint_names[5] = "missing_joint";
    bool bad_joint_threw = false;
    try {
        rb_servo::PinocchioKinematics bad_joint(cfg);
    } catch (const std::exception&) {
        bad_joint_threw = true;
    }
    RB_CHECK(bad_joint_threw);
#endif
    return true;
}

}  // namespace

int main() {
    if (!testKinematicsConfigValidation()) return 1;
    if (!testIkRemainsUnavailable()) return 1;
    if (!testDisabledBuildBehavior()) return 1;
    if (!testPinocchioFkIfEnabled()) return 1;
    return 0;
}
