#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <vector>

#include "rb_servo/control/self_collision.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

bool approx(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) <= tol;
}

std::string rb3Urdf() {
    return (std::filesystem::path(__FILE__).parent_path().parent_path() /
            "descriptions" / "urdf" / "rb3_730e.urdf")
        .string();
}

rb_servo::ArmCapsule cap(std::array<double, 3> p0, std::array<double, 3> p1, double r) {
    rb_servo::ArmCapsule c;
    c.p0_m = p0;
    c.p1_m = p1;
    c.radius_m = r;
    return c;
}

rb_servo::KinematicsConfig rb3KinematicsConfig() {
    rb_servo::KinematicsConfig cfg;
    cfg.enable = true;
    cfg.provider = "pinocchio";
    cfg.urdf = rb3Urdf();
    cfg.base_link = "world";
    cfg.tip_link = "tcp";
    cfg.joint_names = {
        "base_joint", "shoulder_joint", "elbow_joint",
        "wrist1_joint", "wrist2_joint", "wrist3_joint",
    };
    cfg.q_units = "deg";
    cfg.publish_tcp = true;
    return cfg;
}

bool testSyntheticApartAndOverlap() {
    // Two parallel capsules separated by 1.0 along z, each radius 0.05.
    const std::vector<rb_servo::ArmCapsule> left = {cap({0, 0, 0}, {1, 0, 0}, 0.05)};
    const std::vector<rb_servo::ArmCapsule> right = {cap({0, 0, 1.0}, {1, 0, 1.0}, 0.05)};
    const rb_servo::SelfCollisionResult apart =
        rb_servo::dualArmSelfCollisionClearance(left, right, 0.05);
    RB_CHECK(apart.checked);
    RB_CHECK(!apart.violated);
    RB_CHECK(approx(apart.min_clearance_m, 1.0 - 0.1));  // 1.0 - 2*0.05

    // Bring them to 0.08 apart: clearance = 0.08 - 0.10 = -0.02 < margin 0.05 -> violated.
    const std::vector<rb_servo::ArmCapsule> right_close = {cap({0, 0, 0.08}, {1, 0, 0.08}, 0.05)};
    const rb_servo::SelfCollisionResult over =
        rb_servo::dualArmSelfCollisionClearance(left, right_close, 0.05);
    RB_CHECK(over.checked);
    RB_CHECK(over.violated);
    RB_CHECK(over.min_clearance_m < 0.05);
    RB_CHECK(over.left_bone == 0 && over.right_bone == 0);
    return true;
}

bool testGeometryUnavailable() {
    const std::vector<rb_servo::ArmCapsule> empty;
    const std::vector<rb_servo::ArmCapsule> ok = {cap({0, 0, 1}, {1, 0, 1}, 0.05)};
    const rb_servo::SelfCollisionResult r =
        rb_servo::dualArmSelfCollisionClearance(empty, ok, 0.05);
    RB_CHECK(!r.checked);
    RB_CHECK(!r.violated);  // caller decides fail-closed; the function reports not-checked
    return true;
}

bool testRealKinematicsPath() {
    rb_servo::PinocchioKinematics kin(rb3KinematicsConfig());

    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    left_mount.base_pose_in_stand = {0.1601, -0.1725, 0.5825, 2.186649, 0.523831, 2.526296};
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    right_mount.base_pose_in_stand = {-0.1601, -0.1725, 0.5825, 2.186649, -0.523831, -2.526296};

    rb_servo::JointArray q{};
    q[0] = 0.0;
    q[1] = -90.0;
    q[2] = 0.0;
    q[3] = 0.0;
    q[4] = 90.0;
    q[5] = 0.0;

    const auto templates = rb_servo::defaultRb3ArmCapsules();
    const auto lc = kin.armCollisionCapsulesInStand(rb_servo::ArmId::Left, q, left_mount, templates);
    const auto rc = kin.armCollisionCapsulesInStand(rb_servo::ArmId::Right, q, right_mount, templates);
    RB_CHECK(lc.size() == templates.size() && rc.size() == templates.size());
    for (const auto& c : lc) {
        RB_CHECK(std::isfinite(c.p0_m[0]) && std::isfinite(c.p1_m[2]));
        RB_CHECK(c.radius_m > 0.0);
    }

    // Tiny margin: geometry is evaluated and yields a finite clearance.
    const rb_servo::SelfCollisionResult tight =
        rb_servo::dualArmSelfCollisionClearance(lc, rc, 0.001);
    RB_CHECK(tight.checked);
    RB_CHECK(std::isfinite(tight.min_clearance_m));
    // Huge margin: the two arms (within ~1 m of each other) must be flagged.
    const rb_servo::SelfCollisionResult wide =
        rb_servo::dualArmSelfCollisionClearance(lc, rc, 1.0);
    RB_CHECK(wide.violated);
    RB_CHECK(wide.left_bone >= 0 && wide.right_bone >= 0);
    RB_CHECK(wide.min_clearance_m < 1.0);
    return true;
}

// The shipped stack config (real mounts + initial pose + RB3 arm capsule template
// + stand capsules) must NOT false-positive — regression gate for the config.
bool testStandCapsulesAgainstStackInitialPose() {
    rb_servo::PinocchioKinematics kin(rb3KinematicsConfig());

    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    left_mount.base_pose_in_stand = {0.1601, -0.1725, 0.5825, 2.186649, 0.523831, 2.526296};
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    right_mount.base_pose_in_stand = {-0.1601, -0.1725, 0.5825, 2.186649, -0.523831, -2.526296};

    const auto cap_s = [](const char* name, std::array<double, 3> p0, std::array<double, 3> p1, double r) {
        rb_servo::StandCapsuleConfig c;
        c.name = name;
        c.p0_m = p0;
        c.p1_m = p1;
        c.radius_m = r;
        return c;
    };
    const std::vector<rb_servo::StandCapsuleConfig> stand = {
        cap_s("base_plate_1", {-0.1500, 0.0600, 0.0050}, {0.1500, 0.0600, 0.0050}, 0.0602),
        cap_s("base_plate_2", {-0.1500, -0.0600, 0.0050}, {0.1500, -0.0600, 0.0050}, 0.0602),
        cap_s("lower_column", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.4300}, 0.0943),
        cap_s("upper_column", {0.0, 0.0159, 0.3941}, {0.0, -0.1432, 0.5532}, 0.0943),
        cap_s("shoulder_block_1", {-0.1050, -0.1404, 0.6034}, {0.1050, -0.1404, 0.6034}, 0.0530),
        cap_s("shoulder_block_2", {-0.1050, -0.1934, 0.6564}, {0.1050, -0.1934, 0.6564}, 0.0530),
        cap_s("shoulder_block_3", {-0.1050, -0.1934, 0.5504}, {0.1050, -0.1934, 0.5504}, 0.0530),
        cap_s("shoulder_block_4", {-0.1050, -0.2464, 0.6034}, {0.1050, -0.2464, 0.6034}, 0.0530),
        cap_s("shoulder_plate_1", {-0.2500, -0.2136, 0.6766}, {0.2500, -0.2135, 0.6766}, 0.0388),
        cap_s("shoulder_plate_2", {-0.2500, -0.2666, 0.6235}, {0.2500, -0.2666, 0.6235}, 0.0388),
        cap_s("mount_plate_a_1", {-0.2562, -0.2172, 0.6838}, {-0.0992, -0.1062, 0.5728}, 0.0403),
        cap_s("mount_plate_a_2", {-0.2562, -0.2738, 0.6272}, {-0.0992, -0.1628, 0.5162}, 0.0403),
        cap_s("mount_plate_b_1", {0.0992, -0.1062, 0.5728}, {0.2562, -0.2172, 0.6838}, 0.0403),
        cap_s("mount_plate_b_2", {0.0992, -0.1628, 0.5162}, {0.2562, -0.2738, 0.6272}, 0.0403),
    };

    // Stack config initial pose.
    rb_servo::JointArray q{};
    q[0] = 0.0;
    q[1] = -30.0;
    q[2] = 80.0;
    q[3] = 0.0;
    q[4] = 60.0;
    q[5] = 0.0;

    const auto templates = rb_servo::defaultRb3ArmCapsules();
    const std::vector<int> ignore_indices{0, 1, 2};  // base/shoulder mount region
    constexpr double kMargin = 0.003;

    for (const auto& [arm, mount] : {
             std::pair<rb_servo::ArmId, rb_servo::ArmMountConfig*>{rb_servo::ArmId::Left, &left_mount},
             {rb_servo::ArmId::Right, &right_mount},
         }) {
        const auto caps = kin.armCollisionCapsulesInStand(arm, q, *mount, templates);
        const rb_servo::SelfCollisionResult result = rb_servo::armStandCollisionClearance(
            caps, stand, kMargin, ignore_indices);
        RB_CHECK(result.checked);
        if (result.violated) {
            std::cerr << "stand false positive: arm=" << (arm == rb_servo::ArmId::Left ? "left" : "right")
                      << " capsule=" << result.left_bone << " stand=" << result.stand_capsule
                      << " clearance=" << result.min_clearance_m << "\n";
        }
        RB_CHECK(!result.violated);
        std::cout << (arm == rb_servo::ArmId::Left ? "left" : "right")
                  << " arm min stand clearance at initial pose: " << result.min_clearance_m
                  << " m (capsule " << result.left_bone << ", " << result.stand_capsule << ")\n";
    }
    return true;
}

bool testArmStandClearanceAndIgnoreIndices() {
    rb_servo::StandCapsuleConfig column;
    column.name = "column";
    column.p0_m = {0.0, 0.0, 0.0};
    column.p1_m = {0.0, 0.0, 1.0};
    column.radius_m = 0.1;

    // Arm capsules: capsule 0 touches the column, capsule 1 is far away.
    const std::vector<rb_servo::ArmCapsule> arm = {
        cap({0.05, 0, 0.5}, {0.5, 0, 0.5}, 0.05),
        cap({0.5, 0, 0.5}, {1.5, 0, 0.5}, 0.05),
    };

    // No ignore list: capsule 0 starts inside the column -> violated.
    const rb_servo::SelfCollisionResult hit = rb_servo::armStandCollisionClearance(
        arm, {column}, 0.005, {});
    RB_CHECK(hit.checked);
    RB_CHECK(hit.violated);
    RB_CHECK(hit.left_bone == 0);
    RB_CHECK(hit.right_bone == 0);
    RB_CHECK(hit.stand_capsule == "column");

    // Ignoring capsule 0 (mount bone): only capsule 1 is checked -> clear.
    const rb_servo::SelfCollisionResult ignored = rb_servo::armStandCollisionClearance(
        arm, {column}, 0.005, {0});
    RB_CHECK(ignored.checked);
    RB_CHECK(!ignored.violated);
    RB_CHECK(ignored.left_bone == 1);
    // capsule 1 spans x in [0.5, 1.5]: clearance = 0.5 - 0.1 - 0.05 = 0.35.
    RB_CHECK(approx(ignored.min_clearance_m, 0.35));

    // Empty stand list -> unchecked (caller fails closed / skips per config).
    const rb_servo::SelfCollisionResult empty = rb_servo::armStandCollisionClearance(
        arm, {}, 0.005, {});
    RB_CHECK(!empty.checked);
    return true;
}

bool testMinResultCombination() {
    rb_servo::SelfCollisionResult a;
    a.checked = true;
    a.min_clearance_m = 0.2;
    a.pair = "left_right";
    rb_servo::SelfCollisionResult b;
    b.checked = true;
    b.min_clearance_m = 0.05;
    b.violated = true;
    b.pair = "right_stand";
    b.stand_capsule = "column";

    const rb_servo::SelfCollisionResult combined = rb_servo::minSelfCollisionResult(a, b);
    RB_CHECK(combined.checked);
    RB_CHECK(combined.violated);
    RB_CHECK(combined.pair == "right_stand");
    RB_CHECK(combined.stand_capsule == "column");
    RB_CHECK(approx(combined.min_clearance_m, 0.05));

    // Unchecked operand never wins.
    const rb_servo::SelfCollisionResult with_unchecked =
        rb_servo::minSelfCollisionResult(rb_servo::SelfCollisionResult{}, a);
    RB_CHECK(with_unchecked.checked);
    RB_CHECK(with_unchecked.pair == "left_right");
    return true;
}

}  // namespace

int main() {
    if (!testSyntheticApartAndOverlap()) return 1;
    if (!testGeometryUnavailable()) return 1;
    if (!testRealKinematicsPath()) return 1;
    if (!testArmStandClearanceAndIgnoreIndices()) return 1;
    if (!testStandCapsulesAgainstStackInitialPose()) return 1;
    if (!testMinResultCombination()) return 1;
    std::cout << "self_collision tests passed\n";
    return 0;
}
