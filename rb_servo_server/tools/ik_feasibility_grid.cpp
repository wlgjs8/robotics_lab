// Offline IK-feasibility grid sampler for the viser "IK 불가 영역" overlay.
//
// Reuses the SAME Pinocchio IK solver the live server uses (rb_servo_core), so
// the feasibility map reflects the real solver's reachability rather than a
// re-implementation. For every cell of a regular grid in the ARM-BASE frame it
// asks: "can the TCP be placed here, for each of N uniformly-sampled approach
// directions?" The success fraction R in [0,1] (reachability index) is written
// out; the Python builder (tools/ik_infeasible_region.py) thresholds R and turns
// the low-R cells into the translucent infeasible-region mesh.
//
// Base-frame feasibility is mount-independent (the mount is a rigid SE(3)), so a
// single base-frame grid serves BOTH arms — mirrored at render time by each arm's
// /stand/<side>_base node, exactly like the reach envelope.
//
// Unlike the live 500 Hz solver (~2 ms budget), this offline pass uses a generous
// iteration/timeout budget and multiple seeds so R reflects whether a solution
// EXISTS, not whether a tight streaming budget happens to find it.
//
// Output JSON schema (frame = arm base, "world" URDF root):
//   { frame, spacing_m, origin_m:[x0,y0,z0], dims:[nx,ny,nz],
//     r_min_m, r_max_m, orientations, seeds, ik:{max_iterations,timeout_ms},
//     R:[nx*ny*nz floats, x-fastest then y then z; >r_max cells = 1.0] }

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"
#include "rb_servo/math/se3.hpp"

namespace {

namespace fs = std::filesystem;

fs::path servoRoot() {
    // <root>/rb_servo_server/tools/ik_feasibility_grid.cpp -> <root>/rb_servo_server
    return fs::path(__FILE__).parent_path().parent_path();
}

struct Args {
    std::string config;       // optional server yaml (kinematics block)
    std::string urdf;         // optional URDF override
    std::string reach_json;   // reach envelope sidecar (for r_min/r_max)
    std::string out;          // output grid JSON
    double spacing_m = 0.05;
    int orientations = 18;
    int seeds = 4;            // IK seeds tried per orientation before giving up
    int max_iterations = 200;
    double timeout_ms = 10.0;
    double r_max_override = 0.0;  // 0 => read from reach_json (fallback 1.06)
    int threads = 0;             // 0 => hardware_concurrency
};

[[noreturn]] void usage(const char* argv0, const std::string& msg = "") {
    if (!msg.empty()) std::cerr << "error: " << msg << "\n";
    std::cerr <<
        "usage: " << argv0 << " [options]\n"
        "  --config PATH        server yaml to source the kinematics block from\n"
        "  --urdf PATH          URDF override (default: descriptions/urdf/rb3_730e.urdf)\n"
        "  --reach-json PATH    reach envelope sidecar for r_min/r_max\n"
        "                       (default: descriptions/reach_envelope_rb3_730e.json)\n"
        "  --out PATH           output grid JSON (default: /tmp/ik_feasibility_grid.json)\n"
        "  --spacing-m F        grid spacing (default 0.05)\n"
        "  --orientations N     approach directions per cell (default 18)\n"
        "  --seeds N            IK seeds tried per orientation (default 4)\n"
        "  --max-iterations N   IK iterations budget (default 200)\n"
        "  --timeout-ms F       IK timeout per solve (default 10)\n"
        "  --r-max F            clip radius override (default: reach r_max, else 1.06)\n"
        "  --threads N          worker threads (default: hardware concurrency)\n";
    std::exit(2);
}

Args parseArgs(int argc, char** argv) {
    Args a;
    auto need = [&](int& i) -> std::string {
        if (i + 1 >= argc) usage(argv[0], std::string("missing value for ") + argv[i]);
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        if (k == "--config") a.config = need(i);
        else if (k == "--urdf") a.urdf = need(i);
        else if (k == "--reach-json") a.reach_json = need(i);
        else if (k == "--out") a.out = need(i);
        else if (k == "--spacing-m") a.spacing_m = std::stod(need(i));
        else if (k == "--orientations") a.orientations = std::stoi(need(i));
        else if (k == "--seeds") a.seeds = std::stoi(need(i));
        else if (k == "--max-iterations") a.max_iterations = std::stoi(need(i));
        else if (k == "--timeout-ms") a.timeout_ms = std::stod(need(i));
        else if (k == "--r-max") a.r_max_override = std::stod(need(i));
        else if (k == "--threads") a.threads = std::stoi(need(i));
        else if (k == "-h" || k == "--help") usage(argv[0]);
        else usage(argv[0], "unknown argument: " + k);
    }
    if (a.urdf.empty())
        a.urdf = (servoRoot() / "descriptions" / "urdf" / "rb3_730e.urdf").string();
    if (a.reach_json.empty())
        a.reach_json = (servoRoot() / "descriptions" / "reach_envelope_rb3_730e.json").string();
    if (a.out.empty()) a.out = "/tmp/ik_feasibility_grid.json";
    if (a.spacing_m <= 0.0) usage(argv[0], "--spacing-m must be > 0");
    if (a.orientations < 1) usage(argv[0], "--orientations must be >= 1");
    if (a.seeds < 1) usage(argv[0], "--seeds must be >= 1");
    return a;
}

rb_servo::KinematicsConfig kinematicsConfig(const Args& a) {
    rb_servo::KinematicsConfig cfg;
    if (!a.config.empty()) {
        // Source tolerances/frames from the same yaml the server uses, then force
        // the offline (generous) budget + ensure IK is enabled.
        cfg = rb_servo::loadConfigFromYaml(a.config).kinematics;
    }
    cfg.enable = true;
    cfg.provider = "pinocchio";
    cfg.urdf = a.urdf;
    if (cfg.base_link.empty()) cfg.base_link = "world";
    if (cfg.tip_link.empty()) cfg.tip_link = "tcp";
    if (cfg.joint_names.size() != static_cast<std::size_t>(rb_servo::kDof)) {
        cfg.joint_names = {"base_joint", "shoulder_joint", "elbow_joint",
                           "wrist1_joint", "wrist2_joint", "wrist3_joint"};
    }
    cfg.q_units = "deg";
    cfg.ik.enable = true;
    cfg.ik.max_iterations = a.max_iterations;
    cfg.ik.timeout_ms = a.timeout_ms;
    // The live server tunes IK for tiny streaming steps (max_step_deg ~2-4°/iter):
    // great for servoing, but it cannot converge from a neutral seed to an
    // arbitrary far target, which would FALSELY mark reachable cells infeasible.
    // This tool is a global existence test, so open the per-iteration step wide
    // (reachable targets then converge in a handful of iters; infeasible ones run
    // to max_iterations with cheap iters, no timeout needed).
    cfg.ik.max_step_deg = {90.0, 90.0, 90.0, 120.0, 120.0, 180.0};
    cfg.ik.damping = 0.001;
    cfg.ik.singular_region_eps = 0.02;
    cfg.ik.damping_max = 0.05;
    // Disable the streaming branch-jump guard: it is for continuous servoing, not
    // for an existence test of an isolated target.
    cfg.ik.max_solution_jump_deg = 0.0;
    cfg.ik.branch_jump_clamp_to_seed = false;
    cfg.ik.branch_jump_rate_limit = false;
    return cfg;
}

// N approach directions on a Fibonacci sphere; the tool z-axis (gripper approach)
// is aligned to each, wrist roll left free (not feasibility-critical here).
std::vector<Eigen::Quaterniond> approachOrientations(int n) {
    std::vector<Eigen::Quaterniond> out;
    out.reserve(n);
    const double golden = M_PI * (1.0 + std::sqrt(5.0));
    for (int i = 0; i < n; ++i) {
        const double z = 1.0 - 2.0 * (i + 0.5) / static_cast<double>(n);
        const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
        const double phi = golden * (i + 0.5);
        const Eigen::Vector3d dir(r * std::cos(phi), r * std::sin(phi), z);
        out.push_back(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), dir));
    }
    return out;
}

double readReachRMax(const std::string& path, double fallback) {
    std::ifstream f(path);
    if (!f) return fallback;
    try {
        nlohmann::json j;
        f >> j;
        if (j.contains("r_max_recommended_m")) return j["r_max_recommended_m"].get<double>();
        if (j.contains("r_max_raw_m")) return j["r_max_raw_m"].get<double>();
    } catch (...) {
    }
    return fallback;
}

double readReachRMin(const std::string& path, double fallback) {
    std::ifstream f(path);
    if (!f) return fallback;
    try {
        nlohmann::json j;
        f >> j;
        if (j.contains("r_min_recommended_m")) return j["r_min_recommended_m"].get<double>();
        if (j.contains("r_min_raw_m")) return j["r_min_raw_m"].get<double>();
    } catch (...) {
    }
    return fallback;
}

}  // namespace

// A cell is FEASIBLE if the TCP can be placed there for AT LEAST ONE sampled
// approach direction (any seed). We early-out on the first success, so reachable
// cells — the vast majority — cost ~1-2 solves; only the genuine "holes" pay the
// full orientations*seeds budget. Returns true if feasible.
bool cellFeasible(const rb_servo::IKinematics& kin, const rb_servo::ArmMountConfig& mount,
                  const std::vector<Eigen::Quaterniond>& orientations, int seeds,
                  double x, double y, double z, std::mt19937& rng) {
    static const rb_servo::JointArray neutral = {0.0, -30.0, 60.0, 0.0, 30.0, 0.0};
    std::uniform_real_distribution<double> wide(-180.0, 180.0);
    std::uniform_real_distribution<double> elbow(-150.0, 150.0);
    for (const auto& q : orientations) {
        rb_servo::Pose6D target;
        target.x = x;
        target.y = y;
        target.z = z;
        target.quaternion_xyzw = std::array<double, 4>{q.x(), q.y(), q.z(), q.w()};
        for (int s = 0; s < seeds; ++s) {
            rb_servo::JointArray seed = neutral;
            if (s > 0) {
                seed = {wide(rng), wide(rng), elbow(rng), wide(rng), wide(rng), wide(rng)};
            }
            if (kin.solveIk(rb_servo::ArmId::Left, target, seed, mount).success) {
                return true;  // reachable with some orientation -> not an IK hole
            }
        }
    }
    return false;
}

int main(int argc, char** argv) {
    const Args args = parseArgs(argc, argv);

    // Validate kinematics once up front (clear error if the URDF/Pinocchio fails).
    try {
        rb_servo::PinocchioKinematics probe(kinematicsConfig(args));
        (void)probe;
    } catch (const std::exception& exc) {
        std::cerr << "failed to init kinematics: " << exc.what() << "\n";
        return 1;
    }

    const double r_max = args.r_max_override > 0.0
                             ? args.r_max_override
                             : readReachRMax(args.reach_json, 1.06);
    const double r_min = readReachRMin(args.reach_json, 0.0);

    // Symmetric grid centred on the arm base, covering the reach sphere.
    const int half = static_cast<int>(std::ceil(r_max / args.spacing_m));
    const int dim = 2 * half + 1;
    const double origin = -half * args.spacing_m;
    const auto orientations = approachOrientations(args.orientations);

    unsigned int nthreads = args.threads > 0 ? static_cast<unsigned int>(args.threads)
                                             : std::thread::hardware_concurrency();
    if (nthreads == 0) nthreads = 1;

    std::cerr << "IK feasibility grid: dim=" << dim << "^3 spacing=" << args.spacing_m
              << "m r=[" << r_min << "," << r_max << "] N_orient=" << args.orientations
              << " seeds=" << args.seeds << " threads=" << nthreads
              << " (ik max_iter=" << args.max_iterations
              << " timeout=" << args.timeout_ms << "ms, early-out feasibility)\n";

    // occupied[idx] = 1 means "inside reach but IK-infeasible" (the drawn region).
    std::vector<std::uint8_t> occupied(static_cast<std::size_t>(dim) * dim * dim, 0);
    std::atomic<long> tested{0}, occupied_count{0};
    std::atomic<int> next_slice{0};

    auto worker = [&](unsigned int tid) {
        // Pinocchio Data is not thread-safe, so each worker owns its own solver.
        rb_servo::PinocchioKinematics kin(kinematicsConfig(args));
        rb_servo::ArmMountConfig mount;  // identity mount => base-frame targets
        mount.arm_id = rb_servo::ArmId::Left;
        mount.base_pose_in_stand = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::mt19937 rng(98765u + tid);
        for (int iz = next_slice.fetch_add(1); iz < dim; iz = next_slice.fetch_add(1)) {
            const double z = origin + iz * args.spacing_m;
            long slice_tested = 0, slice_occ = 0;
            for (int iy = 0; iy < dim; ++iy) {
                const double y = origin + iy * args.spacing_m;
                for (int ix = 0; ix < dim; ++ix) {
                    const double x = origin + ix * args.spacing_m;
                    if (x * x + y * y + z * z > r_max * r_max) continue;  // reach 부족
                    ++slice_tested;
                    if (!cellFeasible(kin, mount, orientations, args.seeds, x, y, z, rng)) {
                        const std::size_t idx =
                            (static_cast<std::size_t>(iz) * dim + iy) * dim + ix;
                        occupied[idx] = 1;
                        ++slice_occ;
                    }
                }
            }
            tested += slice_tested;
            occupied_count += slice_occ;
            std::cerr << "  z-slice " << (iz + 1) << "/" << dim
                      << " tested=" << tested.load() << " occ=" << occupied_count.load()
                      << "\r" << std::flush;
        }
    };

    std::vector<std::thread> pool;
    for (unsigned int t = 0; t < nthreads; ++t) pool.emplace_back(worker, t);
    for (auto& th : pool) th.join();
    std::cerr << "\n";

    nlohmann::json out;
    out["frame"] = "base";
    out["spacing_m"] = args.spacing_m;
    out["origin_m"] = {origin, origin, origin};
    out["dims"] = {dim, dim, dim};
    out["r_min_m"] = r_min;
    out["r_max_m"] = r_max;
    out["orientations"] = args.orientations;
    out["seeds"] = args.seeds;
    out["ik"] = {{"max_iterations", args.max_iterations}, {"timeout_ms", args.timeout_ms}};
    out["occupied"] = occupied;  // uint8 flat array, x-fastest then y then z

    std::ofstream of(args.out);
    if (!of) {
        std::cerr << "cannot write " << args.out << "\n";
        return 1;
    }
    of << out.dump();
    of.close();

    const long t = tested.load(), o = occupied_count.load();
    std::cerr << "done: tested=" << t << " cells, infeasible(occupied)=" << o
              << " (" << (t > 0 ? 100.0 * o / t : 0.0) << "%)\n  wrote " << args.out << "\n";
    return 0;
}
