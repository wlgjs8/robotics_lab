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

}  // namespace

int main() {
    if (!testSyntheticApartAndOverlap()) return 1;
    if (!testGeometryUnavailable()) return 1;
    if (!testRealKinematicsPath()) return 1;
    std::cout << "self_collision tests passed\n";
    return 0;
}
