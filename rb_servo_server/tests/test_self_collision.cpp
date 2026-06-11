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

using Points = std::vector<std::array<double, 3>>;

bool testSyntheticApartAndOverlap() {
    std::array<double, 7> radii;
    radii.fill(0.05);

    // Two parallel unit segments separated by 1.0 along z.
    const Points left = {{0, 0, 0}, {1, 0, 0}};
    const Points right = {{0, 0, 1.0}, {1, 0, 1.0}};
    const rb_servo::SelfCollisionResult apart =
        rb_servo::dualArmSelfCollisionClearance(left, right, radii, 0.05);
    RB_CHECK(apart.checked);
    RB_CHECK(!apart.violated);
    RB_CHECK(approx(apart.min_clearance_m, 1.0 - 0.1));  // 1.0 - 2*0.05

    // Bring them to 0.08 apart: clearance = 0.08 - 0.10 = -0.02 < margin 0.05 -> violated.
    const Points right_close = {{0, 0, 0.08}, {1, 0, 0.08}};
    const rb_servo::SelfCollisionResult over =
        rb_servo::dualArmSelfCollisionClearance(left, right_close, radii, 0.05);
    RB_CHECK(over.checked);
    RB_CHECK(over.violated);
    RB_CHECK(over.min_clearance_m < 0.05);
    RB_CHECK(over.left_bone == 0 && over.right_bone == 0);
    return true;
}

bool testGeometryUnavailable() {
    std::array<double, 7> radii;
    radii.fill(0.05);
    const Points too_short = {{0, 0, 0}};
    const Points ok = {{0, 0, 1}, {1, 0, 1}};
    const rb_servo::SelfCollisionResult r =
        rb_servo::dualArmSelfCollisionClearance(too_short, ok, radii, 0.05);
    RB_CHECK(!r.checked);
    RB_CHECK(!r.violated);  // caller decides fail-closed; the function reports not-checked
    return true;
}

bool testRealKinematicsPath() {
    rb_servo::KinematicsConfig cfg;
    cfg.enable = true;
    cfg.provider = "pinocchio";
    cfg.urdf = rb3Urdf();
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

    const Points lp = kin.linkCollisionPointsInStand(rb_servo::ArmId::Left, q, left_mount);
    const Points rp = kin.linkCollisionPointsInStand(rb_servo::ArmId::Right, q, right_mount);
    RB_CHECK(lp.size() == 8 && rp.size() == 8);

    std::array<double, 7> radii;
    radii.fill(0.02);
    // Tiny margin: geometry is evaluated and yields a finite clearance.
    const rb_servo::SelfCollisionResult tight =
        rb_servo::dualArmSelfCollisionClearance(lp, rp, radii, 0.001);
    RB_CHECK(tight.checked);
    RB_CHECK(std::isfinite(tight.min_clearance_m));
    // Huge margin: the two arms (within ~1 m of each other) must be flagged.
    const rb_servo::SelfCollisionResult wide =
        rb_servo::dualArmSelfCollisionClearance(lp, rp, radii, 1.0);
    RB_CHECK(wide.violated);
    RB_CHECK(wide.left_bone >= 0 && wide.right_bone >= 0);
    RB_CHECK(wide.min_clearance_m < 1.0);
    return true;
}

// The configured stand capsules (derived from the dual_rb3_730e_stand_ver3 URDF
// collision boxes) with the real mounts and the stack initial pose must NOT
// false-positive — this is the regression gate for the shipped stack config.
bool testStandCapsulesAgainstStackInitialPose() {
    rb_servo::KinematicsConfig cfg;
    cfg.enable = true;
    cfg.provider = "pinocchio";
    cfg.urdf = rb3Urdf();
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

    rb_servo::ArmMountConfig left_mount;
    left_mount.arm_id = rb_servo::ArmId::Left;
    left_mount.base_pose_in_stand = {0.1601, -0.1725, 0.5825, 2.186649, 0.523831, 2.526296};
    rb_servo::ArmMountConfig right_mount;
    right_mount.arm_id = rb_servo::ArmId::Right;
    right_mount.base_pose_in_stand = {-0.1601, -0.1725, 0.5825, 2.186649, -0.523831, -2.526296};

    // Stand capsules from the URDF collision boxes (stand frame, meters).
    const auto cap = [](const char* name, std::array<double, 3> p0, std::array<double, 3> p1, double r) {
        rb_servo::StandCapsuleConfig c;
        c.name = name;
        c.p0_m = p0;
        c.p1_m = p1;
        c.radius_m = r;
        return c;
    };
    const std::vector<rb_servo::StandCapsuleConfig> stand = {
        cap("base_plate_1", {-0.1500, 0.0600, 0.0050}, {0.1500, 0.0600, 0.0050}, 0.0602),
        cap("base_plate_2", {-0.1500, -0.0600, 0.0050}, {0.1500, -0.0600, 0.0050}, 0.0602),
        cap("lower_column", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.4300}, 0.0943),
        cap("upper_column", {0.0, 0.0159, 0.3941}, {0.0, -0.1432, 0.5532}, 0.0943),
        cap("shoulder_block_1", {-0.1050, -0.1404, 0.6034}, {0.1050, -0.1404, 0.6034}, 0.0530),
        cap("shoulder_block_2", {-0.1050, -0.1934, 0.6564}, {0.1050, -0.1934, 0.6564}, 0.0530),
        cap("shoulder_block_3", {-0.1050, -0.1934, 0.5504}, {0.1050, -0.1934, 0.5504}, 0.0530),
        cap("shoulder_block_4", {-0.1050, -0.2464, 0.6034}, {0.1050, -0.2464, 0.6034}, 0.0530),
        cap("shoulder_plate_1", {-0.2500, -0.2136, 0.6766}, {0.2500, -0.2135, 0.6766}, 0.0388),
        cap("shoulder_plate_2", {-0.2500, -0.2666, 0.6235}, {0.2500, -0.2666, 0.6235}, 0.0388),
        cap("mount_plate_a_1", {-0.2562, -0.2172, 0.6838}, {-0.0992, -0.1062, 0.5728}, 0.0403),
        cap("mount_plate_a_2", {-0.2562, -0.2738, 0.6272}, {-0.0992, -0.1628, 0.5162}, 0.0403),
        cap("mount_plate_b_1", {0.0992, -0.1062, 0.5728}, {0.2562, -0.2172, 0.6838}, 0.0403),
        cap("mount_plate_b_2", {0.0992, -0.1628, 0.5162}, {0.2562, -0.2738, 0.6272}, 0.0403),
    };

    // Stack config initial pose.
    rb_servo::JointArray q{};
    q[0] = 0.0;
    q[1] = -30.0;
    q[2] = 80.0;
    q[3] = 0.0;
    q[4] = 60.0;
    q[5] = 0.0;

    const std::array<double, 7> radii{0.10, 0.09, 0.08, 0.07, 0.06, 0.06, 0.06};
    const std::vector<int> ignore_bones{0};
    constexpr double kMargin = 0.003;

    for (const auto& [arm, mount] : {
             std::pair<rb_servo::ArmId, rb_servo::ArmMountConfig*>{rb_servo::ArmId::Left, &left_mount},
             {rb_servo::ArmId::Right, &right_mount},
         }) {
        const Points points = kin.linkCollisionPointsInStand(arm, q, *mount);
        const rb_servo::SelfCollisionResult result = rb_servo::armStandCollisionClearance(
            points, radii, stand, kMargin, ignore_bones);
        RB_CHECK(result.checked);
        if (result.violated) {
            std::cerr << "stand false positive: arm=" << (arm == rb_servo::ArmId::Left ? "left" : "right")
                      << " bone=" << result.left_bone << " capsule=" << result.stand_capsule
                      << " clearance=" << result.min_clearance_m << "\n";
        }
        RB_CHECK(!result.violated);
        std::cout << (arm == rb_servo::ArmId::Left ? "left" : "right")
                  << " arm min stand clearance at initial pose: " << result.min_clearance_m
                  << " m (bone " << result.left_bone << ", " << result.stand_capsule << ")\n";
    }
    return true;
}

bool testArmStandClearanceAndIgnoreBones() {
    std::array<double, 7> radii;
    radii.fill(0.05);

    rb_servo::StandCapsuleConfig column;
    column.name = "column";
    column.p0_m = {0.0, 0.0, 0.0};
    column.p1_m = {0.0, 0.0, 1.0};
    column.radius_m = 0.1;

    // Arm chain: bone 0 touches the column, bone 1 is far away.
    const Points arm = {{0.05, 0, 0.5}, {0.5, 0, 0.5}, {1.5, 0, 0.5}};

    // No ignore list: bone 0 starts inside the column -> violated.
    const rb_servo::SelfCollisionResult hit = rb_servo::armStandCollisionClearance(
        arm, radii, {column}, 0.005, {});
    RB_CHECK(hit.checked);
    RB_CHECK(hit.violated);
    RB_CHECK(hit.left_bone == 0);
    RB_CHECK(hit.right_bone == 0);
    RB_CHECK(hit.stand_capsule == "column");

    // Ignoring bone 0 (mount bone): only bone 1 is checked -> clear.
    const rb_servo::SelfCollisionResult ignored = rb_servo::armStandCollisionClearance(
        arm, radii, {column}, 0.005, {0});
    RB_CHECK(ignored.checked);
    RB_CHECK(!ignored.violated);
    RB_CHECK(ignored.left_bone == 1);
    // bone 1 spans x in [0.5, 1.5]: clearance = 0.5 - 0.1 - 0.05 = 0.35.
    RB_CHECK(approx(ignored.min_clearance_m, 0.35));

    // Empty stand list -> unchecked (caller fails closed / skips per config).
    const rb_servo::SelfCollisionResult empty = rb_servo::armStandCollisionClearance(
        arm, radii, {}, 0.005, {});
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
    if (!testArmStandClearanceAndIgnoreBones()) return 1;
    if (!testStandCapsulesAgainstStackInitialPose()) return 1;
    if (!testMinResultCombination()) return 1;
    std::cout << "self_collision tests passed\n";
    return 0;
}
