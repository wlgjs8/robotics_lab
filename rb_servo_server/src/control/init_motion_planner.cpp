#include "rb_servo/control/init_motion_planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <sstream>

#include "rb_servo/math/se3.hpp"

namespace rb_servo {
namespace {

constexpr int kCombined = 2 * kDof;  // 12-DOF combined config (left6 + right6)
using Config = std::array<double, kCombined>;

constexpr double kRadToDeg = 57.29577951308232;

Config join(const JointArray& l, const JointArray& r) {
    Config c{};
    for (int i = 0; i < kDof; ++i) {
        c[i] = l[i];
        c[kDof + i] = r[i];
    }
    return c;
}

void split(const Config& c, JointArray* l, JointArray* r) {
    for (int i = 0; i < kDof; ++i) {
        (*l)[i] = c[i];
        (*r)[i] = c[kDof + i];
    }
}

double dist(const Config& a, const Config& b) {
    double s = 0.0;
    for (int i = 0; i < kCombined; ++i) {
        const double d = a[i] - b[i];
        s += d * d;
    }
    return std::sqrt(s);
}

Config lerp(const Config& a, const Config& b, double t) {
    Config c{};
    for (int i = 0; i < kCombined; ++i) c[i] = a[i] + t * (b[i] - a[i]);
    return c;
}

// Nearest-branch goal selection per joint (mirrors trajectory_filter's file-local
// shortestPathJointGoal): pick the in-range equivalent (target + 360*k) closest to
// the reference so InitMotion does not spin a full revolution to an equivalent pose.
double nearestBranch(double raw_target, double reference, double q_min, double q_max) {
    if (!(std::isfinite(raw_target) && std::isfinite(reference))) return raw_target;
    if (!(q_max > q_min)) return raw_target;
    double best = raw_target;
    double best_dist = std::abs(raw_target - reference);
    for (int k = -3; k <= 3; ++k) {
        if (k == 0) continue;
        const double cand = raw_target + 360.0 * static_cast<double>(k);
        if (cand < q_min || cand > q_max) continue;
        const double d = std::abs(cand - reference);
        if (d < best_dist) {
            best_dist = d;
            best = cand;
        }
    }
    return best;
}

}  // namespace

struct InitMotionPlanner::Impl {
    InitMotionPlannerConfig cfg;
    JointArray q_min_deg{};
    JointArray q_max_deg{};
    double clear_threshold_m = 0.0;  // d_hard + collision_margin
    std::unique_ptr<CollisionMonitor> oracle;
    std::mt19937 rng;
    // Private IK/FK for collision-free TcpLinearMove (own pinocchio Data; never the
    // servo loop's kinematics). Null when linear collision-free is unused.
    std::shared_ptr<IKinematics> kin;
    ArmMountConfig left_mount{};
    ArmMountConfig right_mount{};

    struct Node {
        Config q;
        int parent;
    };

    Impl(CollisionMonitorConfig monitor_cfg, InitMotionPlannerConfig planner_cfg,
         JointArray qmin, JointArray qmax,
         std::shared_ptr<IKinematics> kinematics, ArmMountConfig lmount, ArmMountConfig rmount)
        : cfg(planner_cfg), q_min_deg(qmin), q_max_deg(qmax), rng(planner_cfg.seed),
          kin(std::move(kinematics)), left_mount(lmount), right_mount(rmount) {
        clear_threshold_m = monitor_cfg.d_hard_m + cfg.collision_margin_m;
        // Endpoint-only eval: the planner does its own dense edge sampling, so the
        // private oracle must not sweep between arbitrary RRT node checks.
        monitor_cfg.swept_samples = 1;
        monitor_cfg.enable = true;
        oracle = std::make_unique<CollisionMonitor>(monitor_cfg);
        // NOTE: deliberately NOT started — used synchronously on the planning thread.
    }

    double minClearance(const Config& c) {
        JointArray l{}, r{};
        split(c, &l, &r);
        const CollisionVerdict v = oracle->evalOnce(l, r);
        return v.min_clearance_m;
    }

    bool clear(const Config& c) {
        JointArray l{}, r{};
        split(c, &l, &r);
        const CollisionVerdict v = oracle->evalOnce(l, r);
        return !v.hard_violation && v.min_clearance_m > clear_threshold_m;
    }

    // Validate the open segment (from, to]; the `from` endpoint is intentionally
    // excluded (it was validated when added to the tree, and excluding it lets the
    // arm escape a start pose that is already in/near collision).
    bool edgeClear(const Config& from, const Config& to) {
        const double edge_res_deg = std::max(1e-3, cfg.edge_resolution_rad * kRadToDeg);
        const int n = std::max(1, static_cast<int>(std::ceil(dist(from, to) / edge_res_deg)));
        for (int i = 1; i <= n; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(n);
            if (!clear(lerp(from, to, t))) return false;
        }
        return true;
    }

    Config sample(const Config& lo, const Config& hi) {
        Config c{};
        for (int i = 0; i < kCombined; ++i) {
            std::uniform_real_distribution<double> d(lo[i], hi[i]);
            c[i] = d(rng);
        }
        return c;
    }

    int nearest(const std::vector<Node>& tree, const Config& q) const {
        int best = 0;
        double best_d = std::numeric_limits<double>::infinity();
        for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
            const double d = dist(tree[i].q, q);
            if (d < best_d) {
                best_d = d;
                best = i;
            }
        }
        return best;
    }

    enum class Extend { Trapped, Advanced, Reached };

    // Steer from the nearest node toward q_target by at most step; add the new node
    // if the edge to it is clear. Reached when the step lands exactly on q_target.
    Extend extend(std::vector<Node>& tree, const Config& q_target, double step_deg) {
        const int ni = nearest(tree, q_target);
        const Config& q_near = tree[ni].q;
        const double d = dist(q_near, q_target);
        bool reached = false;
        Config q_new;
        if (d <= step_deg || d < 1e-9) {
            q_new = q_target;
            reached = true;
        } else {
            q_new = lerp(q_near, q_target, step_deg / d);
        }
        if (!edgeClear(q_near, q_new)) return Extend::Trapped;
        tree.push_back(Node{q_new, ni});
        return reached ? Extend::Reached : Extend::Advanced;
    }

    Extend connect(std::vector<Node>& tree, const Config& q_target, double step_deg) {
        Extend s = Extend::Advanced;
        while (s == Extend::Advanced) s = extend(tree, q_target, step_deg);
        return s;
    }

    static std::vector<Config> rootPath(const std::vector<Node>& tree, int leaf) {
        std::vector<Config> path;
        for (int i = leaf; i >= 0; i = tree[i].parent) path.push_back(tree[i].q);
        std::reverse(path.begin(), path.end());  // root -> leaf
        return path;
    }

    // Replace random index pairs with a straight, oracle-validated shortcut.
    void shortcut(std::vector<Config>& path) {
        if (path.size() < 3) return;
        std::uniform_real_distribution<double> u(0.0, 1.0);
        for (int pass = 0; pass < cfg.shortcut_passes && path.size() > 2; ++pass) {
            const int n = static_cast<int>(path.size());
            int i = static_cast<int>(u(rng) * (n - 1));
            int j = static_cast<int>(u(rng) * (n - 1));
            if (i > j) std::swap(i, j);
            if (j - i < 2) continue;
            if (edgeClear(path[i], path[j])) {
                path.erase(path.begin() + i + 1, path.begin() + j);
            }
        }
    }

    // Insert interior points so no segment exceeds max_segment_deg on any joint.
    std::vector<Config> densify(const std::vector<Config>& path) const {
        std::vector<Config> out;
        if (path.empty()) return out;
        out.push_back(path.front());
        for (std::size_t s = 1; s < path.size(); ++s) {
            const Config& a = path[s - 1];
            const Config& b = path[s];
            double max_delta = 0.0;
            for (int k = 0; k < kCombined; ++k) max_delta = std::max(max_delta, std::abs(b[k] - a[k]));
            const int n = std::max(1, static_cast<int>(std::ceil(max_delta /
                std::max(1e-6, cfg.max_segment_deg))));
            for (int i = 1; i <= n; ++i) {
                out.push_back(lerp(a, b, static_cast<double>(i) / static_cast<double>(n)));
            }
        }
        return out;
    }
};

InitMotionPlanner::InitMotionPlanner(CollisionMonitorConfig monitor_cfg,
                                     InitMotionPlannerConfig planner_cfg,
                                     JointArray q_min_deg, JointArray q_max_deg,
                                     std::shared_ptr<IKinematics> kinematics,
                                     ArmMountConfig left_mount, ArmMountConfig right_mount)
    : impl_(std::make_unique<Impl>(std::move(monitor_cfg), planner_cfg, q_min_deg, q_max_deg,
                                   std::move(kinematics), left_mount, right_mount)) {}

InitMotionPlanner::~InitMotionPlanner() = default;

bool InitMotionPlanner::configClear(const JointArray& left, const JointArray& right) {
    return impl_->clear(join(left, right));
}

double InitMotionPlanner::minClearance(const JointArray& left, const JointArray& right) {
    return impl_->minClearance(join(left, right));
}

InitMotionPlanResult InitMotionPlanner::plan(
    const JointArray& start_left, const JointArray& start_right,
    const JointArray& goal_left, const JointArray& goal_right) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    InitMotionPlanResult result;

    Impl& d = *impl_;

    // Wrap the goal to the nearest in-range joint branch (per arm) vs the start.
    JointArray gl = goal_left, gr = goal_right;
    for (int i = 0; i < kDof; ++i) {
        gl[i] = nearestBranch(goal_left[i], start_left[i], d.q_min_deg[i], d.q_max_deg[i]);
        gr[i] = nearestBranch(goal_right[i], start_right[i], d.q_min_deg[i], d.q_max_deg[i]);
    }
    const Config start = join(start_left, start_right);
    const Config goal = join(gl, gr);

    const auto finite = [](const Config& c) {
        return std::all_of(c.begin(), c.end(), [](double v) { return std::isfinite(v); });
    };
    if (!finite(start) || !finite(goal)) {
        result.message = "init motion plan: non-finite start/goal";
        return result;
    }
    if (!d.clear(goal)) {
        // The init pose itself collides / dips below the floor — refuse (fail-closed).
        std::ostringstream m;
        m << "init motion plan: goal config not collision/floor clear"
          << " (goal_clear_m=" << d.minClearance(goal)
          << ", thresh_m=" << d.clear_threshold_m << ")";
        result.message = m.str();
        return result;
    }

    const double step_deg = std::max(1e-3, d.cfg.step_size_rad * kRadToDeg);

    // Per-joint sampling band: [min(start,goal) - margin, max(start,goal) + margin]
    // clamped to the configured joint limits — keeps planning fast and avoids the
    // full +/-360 rbpodo range.
    Config lo{}, hi{};
    for (int k = 0; k < kCombined; ++k) {
        const int j = k % kDof;
        const double a = std::min(start[k], goal[k]) - d.cfg.sample_margin_deg;
        const double b = std::max(start[k], goal[k]) + d.cfg.sample_margin_deg;
        lo[k] = (d.q_max_deg[j] > d.q_min_deg[j]) ? std::max(a, d.q_min_deg[j]) : a;
        hi[k] = (d.q_max_deg[j] > d.q_min_deg[j]) ? std::min(b, d.q_max_deg[j]) : b;
        if (hi[k] < lo[k]) std::swap(hi[k], lo[k]);
    }

    std::vector<Config> raw;

    // Fast path: the straight joint-space edge is already collision-free.
    if (d.edgeClear(start, goal)) {
        raw = {start, goal};
    } else {
        // Bidirectional RRT-Connect. trees[0] rooted at start, trees[1] at goal; the
        // last-added node of each tree is its connection endpoint on success.
        std::vector<Impl::Node> tree_a{{start, -1}};
        std::vector<Impl::Node> tree_b{{goal, -1}};
        std::vector<Impl::Node>* trees[2] = {&tree_a, &tree_b};
        std::uniform_real_distribution<double> u(0.0, 1.0);
        bool connected = false;
        int iter = 0;
        for (; iter < d.cfg.max_iterations; ++iter) {
            if (std::chrono::duration<double>(Clock::now() - t0).count() >
                d.cfg.max_planning_time_sec) {
                break;
            }
            const int i = iter % 2;
            const int j = 1 - i;
            // Goal-biased growth toward the opposite tree's root accelerates linking.
            const Config q_rand = (u(d.rng) < d.cfg.goal_bias)
                ? trees[j]->front().q
                : d.sample(lo, hi);
            const Impl::Extend ext = d.extend(*trees[i], q_rand, step_deg);
            if (ext == Impl::Extend::Trapped) continue;
            const Config q_new = trees[i]->back().q;
            if (d.connect(*trees[j], q_new, step_deg) == Impl::Extend::Reached) {
                connected = true;
                break;
            }
        }
        result.iterations = iter;
        if (!connected) {
            result.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
            // Diagnostics: which budget ran out, how far each tree grew, and whether the
            // endpoints themselves are clear. tree_start staying ~1 means the START pose
            // could not escape (start in/near collision); both trees large but unconnected
            // means a narrow passage / the sample band (sample_margin_deg) is too tight or
            // the time/iteration budget is too small.
            std::ostringstream m;
            m << "init motion plan: RRT-Connect did not find a path within budget"
              << " (iters=" << iter << "/" << d.cfg.max_iterations
              << ", time=" << result.planning_time_s << "/" << d.cfg.max_planning_time_sec << "s"
              << ", tree_start=" << tree_a.size() << ", tree_goal=" << tree_b.size()
              << ", start_clear_m=" << d.minClearance(start)
              << ", goal_clear_m=" << d.minClearance(goal)
              << ", thresh_m=" << d.clear_threshold_m
              << ", sample_margin_deg=" << d.cfg.sample_margin_deg << ")";
            result.message = m.str();
            return result;
        }
        // start_path: start..q_new ; goal_path: goal..q_new -> reverse tail to append.
        std::vector<Config> start_path = Impl::rootPath(tree_a, static_cast<int>(tree_a.size()) - 1);
        std::vector<Config> goal_path = Impl::rootPath(tree_b, static_cast<int>(tree_b.size()) - 1);
        std::reverse(goal_path.begin(), goal_path.end());  // q_new..goal
        raw = start_path;
        raw.insert(raw.end(), goal_path.begin() + 1, goal_path.end());  // drop dup q_new
    }

    d.shortcut(raw);
    const std::vector<Config> dense = d.densify(raw);

    result.waypoints.reserve(dense.size());
    for (const Config& c : dense) {
        JointArray l{}, r{};
        split(c, &l, &r);
        result.waypoints.emplace_back(l, r);
    }
    result.success = true;
    result.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
    result.message = "ok";
    return result;
}

InitMotionLinearResult InitMotionPlanner::planLinearMove(
    const JointArray& start_left, const JointArray& start_right,
    bool left_active, const Pose6D& goal_pose_left,
    bool right_active, const Pose6D& goal_pose_right,
    bool slerp, int check_samples) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    InitMotionLinearResult res;
    Impl& d = *impl_;
    res.goal_left = start_left;
    res.goal_right = start_right;
    if (!d.kin) {
        res.message = "linear move: no private kinematics";
        return res;
    }
    if (!left_active && !right_active) {
        res.message = "linear move: no active arm";
        return res;
    }

    // IK the target pose(s) to a joint goal (seeded from start -> nearest branch).
    if (left_active) {
        const IkResult ik = d.kin->solveIk(ArmId::Left, goal_pose_left, start_left, d.left_mount);
        if (!ik.success) {
            res.message = "linear move: left goal IK failed";
            return res;
        }
        res.goal_left = ik.q_solution_deg;
    }
    if (right_active) {
        const IkResult ik = d.kin->solveIk(ArmId::Right, goal_pose_right, start_right, d.right_mount);
        if (!ik.success) {
            res.message = "linear move: right goal IK failed";
            return res;
        }
        res.goal_right = ik.q_solution_deg;
    }

    // Straight Cartesian-path feasibility: sample the exact MoveL path, per-sample IK
    // (seeded for joint continuity), oracle-check each combined config. An inactive arm
    // holds at its start config across the whole path.
    const Pose6D start_pose_left = left_active
        ? d.kin->computeTcpStand(ArmId::Left, start_left, d.left_mount) : Pose6D{};
    const Pose6D start_pose_right = right_active
        ? d.kin->computeTcpStand(ArmId::Right, start_right, d.right_mount) : Pose6D{};
    const int n = std::max(1, check_samples);
    JointArray prev_left = start_left;
    JointArray prev_right = start_right;
    bool straight_clear = true;
    for (int s = 1; s <= n; ++s) {
        const double frac = static_cast<double>(s) / static_cast<double>(n);
        JointArray q_left = start_left;
        JointArray q_right = start_right;
        if (left_active) {
            const Pose6D p = math::interpolateLinear(start_pose_left, goal_pose_left, slerp, frac);
            const IkResult ik = d.kin->solveIk(ArmId::Left, p, prev_left, d.left_mount);
            if (!ik.success) { straight_clear = false; break; }
            q_left = ik.q_solution_deg;
        }
        if (right_active) {
            const Pose6D p = math::interpolateLinear(start_pose_right, goal_pose_right, slerp, frac);
            const IkResult ik = d.kin->solveIk(ArmId::Right, p, prev_right, d.right_mount);
            if (!ik.success) { straight_clear = false; break; }
            q_right = ik.q_solution_deg;
        }
        if (!d.clear(join(q_left, q_right))) { straight_clear = false; break; }
        prev_left = q_left;
        prev_right = q_right;
    }

    if (straight_clear) {
        res.decision = InitMotionLinearResult::Decision::Straight;
        res.message = "straight path clear";
        res.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
        return res;
    }

    // Straight path blocked -> joint-space collision-free detour to the IK'd goal.
    InitMotionPlanResult plan_res = plan(start_left, start_right, res.goal_left, res.goal_right);
    res.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
    if (plan_res.success && !plan_res.waypoints.empty()) {
        res.decision = InitMotionLinearResult::Decision::Detour;
        res.waypoints = std::move(plan_res.waypoints);
        res.message = "detour: " + plan_res.message;
    } else {
        res.decision = InitMotionLinearResult::Decision::Failed;
        res.message = "detour planning failed: " + plan_res.message;
    }
    return res;
}

}  // namespace rb_servo
