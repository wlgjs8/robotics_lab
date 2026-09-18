// OFFLINE ONLY: evaluate the tracked self-collision model at a recorded joint pose
// and print the nearest pairs with their witness points, so a clearance the guard
// reported can be checked against a tape measure on the cell. Builds the SAME
// CollisionMonitor the server builds from the stack config (unified URDF, mount
// calibration, hulls, disabled pairs, class floors), runs one synchronous
// evaluation, no thread, no backend, no socket.
//
//   collision_pose_probe --config rb_servo_server/config/stack_real.yaml \
//       --left -51.3,73.8,78.1,-9.5,-81.5,-90.7 --right ... [--near 20] \
//       [--gripper-left 100 --gripper-right 100] [--grep riser] \
//       [--unified-urdf PATH] [--swept N] [--repeat N] [--poses FILE]
//
// --unified-urdf swaps the collision URDF without touching the tracked config, and
// --repeat re-runs the evaluation and reports the monitor's OWN eval_ms (p50/p95/max)
// -- together they let an offline sweep compare collision-geometry variants on both
// accuracy and cost (rb_servo_server/tools/stand_hull_sweep.py).
//
// --poses FILE evaluates MANY poses in one process (one line = 12 comma-separated
// joint values, left 6 then right 6; blank lines and # comments skipped) and prints
// one TSV row per pose instead of the near-pair dump. Same model build for all of
// them, so the per-pose cost is the evaluation alone -- which is what makes a
// geometry variant's accuracy and cost measurable against a reference model without
// paying the model build 300 times.
//
// Distances are the guard's own (convex hulls / boxes), in metres, stand frame.
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <limits>
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

// Position of the n-th comma, i.e. where a 12-value pose line splits into its two
// 6-value halves. npos when the line has fewer than n commas.
std::size_t nthCommaPos(const std::string& line, std::size_t n) {
    std::size_t pos = std::string::npos;
    std::size_t from = 0;
    for (std::size_t k = 0; k < n; ++k) {
        pos = line.find(',', from);
        if (pos == std::string::npos) return std::string::npos;
        from = pos + 1;
    }
    return pos;
}

CollisionMonitorConfig monitorConfigFrom(const DualArmConfig& config, int near) {
    const auto& m = config.safety.self_collision.mesh;
    CollisionMonitorConfig c;
    c.enable = true;
    c.unified_urdf = m.unified_urdf;
    c.package_dirs = m.package_dirs;
    c.pika_gripper_mesh = m.pika_gripper_mesh;
    c.pika_gripper_base_meshes = m.pika_gripper_base_meshes;
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
    // The server's value, not a hardcoded 1: this tool's contract is "the SAME
    // CollisionMonitor the server builds", and swept_samples is what the per-eval cost
    // scales with, so a timing taken here has to be the server's timing.
    c.swept_samples = m.swept_samples;
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
    c.arm_stand_d_hard_m = inherit(m.arm_stand.d_hard_m, m.d_hard_m);
    c.arm_stand_d_slow_m = inherit(m.arm_stand.d_slow_m, m.d_slow_m);
    c.arm_stand_a_brake_m_s2 = inherit(m.arm_stand.a_brake_m_s2, m.a_brake_m_s2);
    c.arm_stand_hyst_m = inherit(m.arm_stand.hyst_m, m.hyst_m);
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
    std::string left_text, right_text, grep, unified_urdf_override, poses_path;
    int swept_override = 0;   // 0 = keep the config's value
    int repeat = 1;
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
        else if (a == "--unified-urdf") unified_urdf_override = next("--unified-urdf");
        else if (a == "--swept") swept_override = std::stoi(next("--swept"));
        else if (a == "--repeat") repeat = std::max(1, std::stoi(next("--repeat")));
        else if (a == "--poses") poses_path = next("--poses");
        else if (a == "--gripper-left") grip_left = std::stod(next("--gripper-left"));
        else if (a == "--gripper-right") grip_right = std::stod(next("--gripper-right"));
        else if (a == "--grep") grep = next("--grep");
        else {
            std::cerr << "usage: collision_pose_probe --config <yaml> --left q0,..,q5 --right q0,..,q5 "
                         "[--near N] [--gripper-left %] [--gripper-right %] [--grep substr] "
                         "[--unified-urdf PATH] [--swept N] [--repeat N] [--poses FILE]\n";
            return 2;
        }
    }
    // --poses carries its own joint values, so --left/--right are required only for
    // the single-pose mode.
    if (config_path.empty() || (poses_path.empty() && (left_text.empty() || right_text.empty()))) {
        std::cerr << "usage: collision_pose_probe --config <yaml> "
                     "(--left q0,..,q5 --right q0,..,q5 | --poses FILE)\n";
        return 2;
    }
    try {
        const DualArmConfig config = loadConfigFromYaml(config_path);
        if (!config.safety.self_collision.enable) {
            std::cerr << "safety.self_collision.enable is false in " << config_path << "\n";
            return 1;
        }
        CollisionMonitorConfig mc = monitorConfigFrom(config, near);
        if (!unified_urdf_override.empty()) mc.unified_urdf = unified_urdf_override;
        if (swept_override > 0) mc.swept_samples = swept_override;
        CollisionMonitor monitor(mc);
        if (monitor.hasArticulatedGripper()) {
            monitor.setGripperOpenPercent(ArmId::Left, grip_left);
            monitor.setGripperOpenPercent(ArmId::Right, grip_right);
        }
        if (!poses_path.empty()) {
            std::ifstream in(poses_path);
            if (!in) {
                std::cerr << "collision_pose_probe: cannot open --poses " << poses_path << "\n";
                return 1;
            }
            // stand_mm is the arm<->STAND minimum specifically. The verdict's `self`
            // figure lumps arm<->arm in with it, and a stand-geometry change moves only
            // the latter -- so a sweep comparing stand hulls needs the split, taken from
            // the near list by geometry name (use a large --near so the closest stand
            // pair cannot rank out of it).
            std::cout << "#i\tmin_mm\tself_mm\tstand_mm\tintra_mm\tenv_mm\tgrip_mm\teval_ms\n";
            std::cout << std::fixed;
            std::string line;
            int idx = 0;
            while (std::getline(in, line)) {
                if (line.empty() || line[0] == '#') continue;
                const std::size_t comma = nthCommaPos(line, kDof);
                if (comma == std::string::npos) {
                    std::cerr << "collision_pose_probe: --poses line " << idx
                              << " needs 12 comma-separated joint values\n";
                    return 1;
                }
                const JointArray l = parseJoints(line.substr(0, comma));
                const JointArray r = parseJoints(line.substr(comma + 1));
                const CollisionVerdict pv = monitor.evalOnce(l, r);
                // The monitor's own class flag since the 2026-09-10 split; before that
                // this had to be recovered from the geometry names.
                double stand_min = pv.arm_stand_min_clearance_m;
                for (const auto& np : pv.near) {
                    if (np.arm_stand) stand_min = std::min(stand_min, np.d_m);
                }
                std::cout << idx++ << '\t' << std::setprecision(3)
                          << pv.min_clearance_m * 1e3 << '\t'
                          << pv.self_min_clearance_m * 1e3 << '\t'
                          << stand_min * 1e3 << '\t'
                          << pv.intra_arm_min_clearance_m * 1e3 << '\t'
                          << pv.environment_min_clearance_m * 1e3 << '\t'
                          << pv.gripper_gripper_min_clearance_m * 1e3 << '\t'
                          << pv.eval_ms << '\n';
            }
            return 0;
        }
        const JointArray ql = parseJoints(left_text);
        const JointArray qr = parseJoints(right_text);
        CollisionVerdict v = monitor.evalOnce(ql, qr);
        // Cost, from the monitor's own instrumentation. The first evaluation warms
        // caches the server's steady state already has, so it is reported separately
        // rather than folded into the percentiles.
        const double eval_first_ms = v.eval_ms;
        std::vector<double> eval_ms;
        for (int r = 1; r < repeat; ++r) {
            v = monitor.evalOnce(ql, qr);
            eval_ms.push_back(v.eval_ms);
        }
        std::cout << std::fixed << std::setprecision(1);
        if (!eval_ms.empty()) {
            std::sort(eval_ms.begin(), eval_ms.end());
            const auto q = [&](double f) {
                return eval_ms[static_cast<std::size_t>(f * static_cast<double>(eval_ms.size() - 1))];
            };
            std::cout << std::setprecision(3) << "eval_ms first=" << eval_first_ms
                      << " p50=" << q(0.5) << " p95=" << q(0.95) << " max=" << eval_ms.back()
                      << " n=" << eval_ms.size() << std::setprecision(1) << "\n";
        } else {
            std::cout << std::setprecision(3) << "eval_ms=" << eval_first_ms
                      << std::setprecision(1) << "\n";
        }
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
            // Same order the monitor resolves thresholds in (nearPairHardFloorM).
            const char* cls = p.external_box ? "external_box"
                            : p.external     ? "external"
                            : p.environment  ? "environment"
                            : p.gripper_gripper ? "gripper_gripper"
                            : p.intra_arm    ? "intra_arm"
                            : p.arm_stand    ? "arm_stand"
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
