// OFFLINE ONLY: evaluate the tracked self-collision model at a recorded joint pose
// and print the nearest pairs with their witness points, so a clearance the guard
// reported can be checked against a tape measure on the cell. Builds the SAME
// CollisionMonitor the server builds from the stack config (unified URDF, mount
// calibration, hulls, disabled pairs, class floors), runs one synchronous
// evaluation, no thread, no backend, no socket.
//
//   collision_pose_probe --config rb_servo_server/config/stack_real.yaml \
//       --left -51.3,73.8,78.1,-9.5,-81.5,-90.7 --right ... [--near 20] \
//       [--gripper-left 100 --gripper-right 100] [--grep riser]
//
// Distances are the guard's own (convex hulls / boxes), in metres, stand frame.
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/collision_monitor.hpp"

using namespace rb_servo;

namespace {

JointArray parseJoints(const std::string& text) {
    JointArray q{};
    std::stringstream ss(text);
    std::string tok;
    int i = 0;
    while (std::getline(ss, tok, ',') && i < static_cast<int>(kDof)) {
        q[static_cast<std::size_t>(i++)] = std::stod(tok);
    }
    if (i != static_cast<int>(kDof)) {
        throw std::runtime_error("expected " + std::to_string(kDof) + " comma-separated joint values, got " +
                                 std::to_string(i));
    }
    return q;
}

CollisionMonitorConfig monitorConfigFrom(const DualArmConfig& config, int near) {
    const auto& m = config.safety.self_collision.mesh;
    CollisionMonitorConfig c;
    c.enable = true;
    c.unified_urdf = m.unified_urdf;
    c.package_dirs = m.package_dirs;
    c.pika_gripper_mesh = m.pika_gripper_mesh;
    c.pika_gripper_base_mesh = m.pika_gripper_base_mesh;
    c.pika_finger_left_mesh = m.pika_finger_left_mesh;
    c.pika_finger_right_mesh = m.pika_finger_right_mesh;
    c.gripper_finger_travel_m = m.gripper_finger_travel_m;
    c.stand_frame = m.stand_frame;
    c.left_prefix = m.left_prefix;
    c.right_prefix = m.right_prefix;
    c.stand_ignore_arm_substrings = m.stand_ignore_arm_substrings;
    c.left_arm_root_frame = m.left_arm_root_frame;
    c.right_arm_root_frame = m.right_arm_root_frame;
    c.check_intra_arm = m.check_intra_arm;
    c.intra_arm_min_chain_separation = m.intra_arm_min_chain_separation;
    c.disabled_collision_pairs = m.disabled_collision_pairs;
    c.debug_pair_curation = false;
    c.swept_samples = 1;
    c.d_hard_m = m.d_hard_m;
    c.d_slow_m = m.d_slow_m;
    c.a_brake_m_s2 = m.a_brake_m_s2;
    c.hyst_m = m.hyst_m;
    c.max_staleness_s = m.max_staleness_s;
    c.monitor_core = -1;
    c.monitor_realtime_priority = 0;
    c.max_near_pairs = near;
    const auto inherit = [](double value, double self_value) { return value > 0.0 ? value : self_value; };
    c.intra_arm_d_hard_m = inherit(m.intra_arm.d_hard_m, m.d_hard_m);
    c.intra_arm_d_slow_m = inherit(m.intra_arm.d_slow_m, m.d_slow_m);
    c.gripper_gripper_d_hard_m = inherit(m.gripper_gripper.d_hard_m, m.d_hard_m);
    c.gripper_gripper_d_slow_m = inherit(m.gripper_gripper.d_slow_m, m.d_slow_m);
    c.gripper_gripper_covered_d_hard_m = m.gripper_gripper.covered_d_hard_m;
    c.environment_d_hard_m = inherit(m.environment.d_hard_m, m.d_hard_m);
    c.environment_d_slow_m = inherit(m.environment.d_slow_m, m.d_slow_m);
    c.external_d_hard_m = m.external.d_hard_m;
    c.external_d_slow_m = m.external.d_slow_m;
    for (std::size_t i = 0; i < kDof; ++i) {
        c.left_joints[i] = m.left_prefix + config.kinematics.joint_names[i];
        c.right_joints[i] = m.right_prefix + config.kinematics.joint_names[i];
    }
    return c;
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path;
    std::string left_text, right_text, grep;
    int near = 24;
    double grip_left = 100.0, grip_right = 100.0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* flag) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string("missing value after ") + flag);
            return argv[++i];
        };
        if (a == "--config") config_path = next("--config");
        else if (a == "--left") left_text = next("--left");
        else if (a == "--right") right_text = next("--right");
        else if (a == "--near") near = std::stoi(next("--near"));
        else if (a == "--gripper-left") grip_left = std::stod(next("--gripper-left"));
        else if (a == "--gripper-right") grip_right = std::stod(next("--gripper-right"));
        else if (a == "--grep") grep = next("--grep");
        else {
            std::cerr << "usage: collision_pose_probe --config <yaml> --left q0,..,q5 --right q0,..,q5 "
                         "[--near N] [--gripper-left %] [--gripper-right %] [--grep substr]\n";
            return 2;
        }
    }
    if (config_path.empty() || left_text.empty() || right_text.empty()) {
        std::cerr << "usage: collision_pose_probe --config <yaml> --left q0,..,q5 --right q0,..,q5\n";
        return 2;
    }
    try {
        const DualArmConfig config = loadConfigFromYaml(config_path);
        if (!config.safety.self_collision.enable) {
            std::cerr << "safety.self_collision.enable is false in " << config_path << "\n";
            return 1;
        }
        CollisionMonitor monitor(monitorConfigFrom(config, near));
        if (monitor.hasArticulatedGripper()) {
            monitor.setGripperOpenPercent(ArmId::Left, grip_left);
            monitor.setGripperOpenPercent(ArmId::Right, grip_right);
        }
        const JointArray ql = parseJoints(left_text);
        const JointArray qr = parseJoints(right_text);
        const CollisionVerdict v = monitor.evalOnce(ql, qr);
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "geoms=" << monitor.numGeometries() << " pairs=" << monitor.numPairs()
                  << " min=" << v.min_clearance_m * 1e3 << " mm self=" << v.self_min_clearance_m * 1e3
                  << " intra=" << v.intra_arm_min_clearance_m * 1e3 << " env=" << v.environment_min_clearance_m * 1e3
                  << " gripper=" << v.gripper_gripper_min_clearance_m * 1e3 << " mm hard=" << v.hard_violation
                  << " near=" << v.near.size() << "\n";
        std::vector<CollisionNearPair> near_sorted = v.near;
        std::sort(near_sorted.begin(), near_sorted.end(),
                  [](const CollisionNearPair& a, const CollisionNearPair& b) { return a.d_m < b.d_m; });
        for (const auto& p : near_sorted) {
            if (!grep.empty() && p.name_a.find(grep) == std::string::npos &&
                p.name_b.find(grep) == std::string::npos) {
                continue;
            }
            const char* cls = p.external_box ? "external_box"
                            : p.external     ? "external"
                            : p.environment  ? "environment"
                            : p.gripper_gripper ? "gripper_gripper"
                            : p.intra_arm    ? "intra_arm"
                                             : "self";
            std::cout << std::setw(7) << p.d_m * 1e3 << " mm  " << std::setw(15) << cls << "  " << p.name_a
                      << " <-> " << p.name_b << "\n"
                      << "           p_a(stand)=(" << p.p_a.x() * 1e3 << ", " << p.p_a.y() * 1e3 << ", "
                      << p.p_a.z() * 1e3 << ") mm  p_b(stand)=(" << p.p_b.x() * 1e3 << ", " << p.p_b.y() * 1e3
                      << ", " << p.p_b.z() * 1e3 << ") mm  n(a->b)=(" << std::setprecision(3) << p.n.x() << ", "
                      << p.n.y() << ", " << p.n.z() << ")" << std::setprecision(1) << "\n";
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "collision_pose_probe: " << e.what() << "\n";
        return 1;
    }
}
