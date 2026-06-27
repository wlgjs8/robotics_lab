#include "rb_servo/control/init_motion_planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <string>

#include "rb_servo/math/se3.hpp"

namespace rb_servo {
namespace {

constexpr int kCombined = 2 * kDof;  // 12-DOF combined config (left6 + right6)
using Config = std::array<double, kCombined>;
using ActiveMask = std::array<bool, kCombined>;

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

double dist2(const Config& a, const Config& b) {
    double s = 0.0;
    for (int i = 0; i < kCombined; ++i) {
        const double d = a[i] - b[i];
        s += d * d;
    }
    return s;
}

Config lerp(const Config& a, const Config& b, double t) {
    Config c{};
    for (int i = 0; i < kCombined; ++i) c[i] = a[i] + t * (b[i] - a[i]);
    return c;
}

ActiveMask makeActiveMask(bool left_active, bool right_active) {
    ActiveMask active{};
    for (int i = 0; i < kDof; ++i) {
        active[i] = left_active;
        active[kDof + i] = right_active;
    }
    return active;
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
    double clear_threshold_m = 0.0;           // self d_hard + collision_margin
    double external_clear_threshold_m = 0.0;  // external d_hard + collision_margin
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
        bool edge_validated = false;
        bool blocked = false;
    };

    struct KdTree {
        struct KdNode {
            Config point{};
            int index = -1;
            int axis = 0;
            int left = -1;
            int right = -1;
        };

        std::vector<KdNode> nodes;
        int root = -1;

        void insert(const Config& q, int index) {
            const int ni = static_cast<int>(nodes.size());
            nodes.push_back(KdNode{q, index, 0, -1, -1});
            if (root < 0) {
                root = ni;
                return;
            }
            int cur = root;
            int depth = 0;
            while (true) {
                const int axis = depth % kCombined;
                nodes[ni].axis = (depth + 1) % kCombined;
                if (q[axis] < nodes[cur].point[axis]) {
                    if (nodes[cur].left < 0) {
                        nodes[cur].left = ni;
                        return;
                    }
                    cur = nodes[cur].left;
                } else {
                    if (nodes[cur].right < 0) {
                        nodes[cur].right = ni;
                        return;
                    }
                    cur = nodes[cur].right;
                }
                ++depth;
            }
        }

        int nearest(const Config& q, const std::vector<Node>& tree) const {
            int best = -1;
            double best_d2 = std::numeric_limits<double>::infinity();
            const auto better = [&](int idx, double d2) {
                constexpr double eps = 1e-12;
                return d2 < best_d2 - eps ||
                       (std::abs(d2 - best_d2) <= eps && (best < 0 || idx < best));
            };
            std::function<void(int)> search = [&](int ni) {
                if (ni < 0) return;
                const KdNode& n = nodes[ni];
                if (n.index >= 0 && n.index < static_cast<int>(tree.size()) &&
                    !tree[n.index].blocked) {
                    const double d2 = dist2(tree[n.index].q, q);
                    if (better(n.index, d2)) {
                        best = n.index;
                        best_d2 = d2;
                    }
                }
                const int axis = n.axis;
                const double delta = q[axis] - n.point[axis];
                const int near = delta < 0.0 ? n.left : n.right;
                const int far = delta < 0.0 ? n.right : n.left;
                search(near);
                if (delta * delta <= best_d2 + 1e-12) search(far);
            };
            search(root);
            return best;
        }
    };

    struct RrtTree {
        std::vector<Node> nodes;
        KdTree kd;
        std::set<std::string> blacklisted_edges;

        void reset(const Config& root_q) {
            nodes.clear();
            kd = KdTree{};
            nodes.push_back(Node{root_q, -1, true, false});
            kd.insert(root_q, 0);
        }
    };

    Impl(CollisionMonitorConfig monitor_cfg, InitMotionPlannerConfig planner_cfg,
         JointArray qmin, JointArray qmax,
         std::shared_ptr<IKinematics> kinematics, ArmMountConfig lmount, ArmMountConfig rmount)
        : cfg(planner_cfg), q_min_deg(qmin), q_max_deg(qmax), rng(planner_cfg.seed),
          kin(std::move(kinematics)), left_mount(lmount), right_mount(rmount) {
        clear_threshold_m = monitor_cfg.d_hard_m + cfg.collision_margin_m;
        // External obstacles (the floor / ground_plane) plan to their own tighter
        // d_hard so InitMotion can approach the floor closer than it keeps the robot
        // from itself, matching the runtime external barrier.
        external_clear_threshold_m = monitor_cfg.external_d_hard_m + cfg.collision_margin_m;
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
        const CollisionDistanceSummary s = oracle->evalDistancesOnly(l, r);
        return s.min_clearance_m;
    }

    bool clear(const Config& c) {
        JointArray l{}, r{};
        split(c, &l, &r);
        return oracle->clearsThresholds(l, r, clear_threshold_m, external_clear_threshold_m);
    }

    // One collision evaluation of a combined config, plus the per-category clearance
    // gate applied to its verdict (mirrors clear() but lets the caller reuse the verdict
    // and its min-clearance without re-evaluating).
    CollisionDistanceSummary eval(const Config& c) {
        JointArray l{}, r{};
        split(c, &l, &r);
        return oracle->evalDistancesOnly(l, r);
    }
    bool isClear(const CollisionDistanceSummary& v) const {
        return !v.hard_violation &&
               v.self_min_clearance_m > clear_threshold_m &&
               v.external_min_clearance_m > external_clear_threshold_m;
    }

    // Gradient-ascent escape from a sub-threshold (near-collision) start. The bidirectional
    // RRT cannot extend out of such a start because every one-step edge is still below the
    // clearance gate (e.g., a gripper resting a few mm off the floor). Here we step along
    // the forward-difference gradient of min-clearance, line-searched so clearance STRICTLY
    // increases each step, until the per-category gate (clear) holds or progress stalls.
    // Returns start..escaped (>=1 node, last node clear) on success, or empty on failure.
    // Every emitted node has higher clearance than the previous, so executing the escape
    // moves the arm monotonically AWAY from the obstacle.
    std::vector<Config> escapeToClearAttempt(const Config& start, const ActiveMask& active,
                                             double step_deg,
                                             int max_steps,
                                             std::chrono::steady_clock::time_point deadline,
                                             double* last_clear_m) {
        std::vector<Config> path{start};
        Config cur = start;
        const double eps = std::max(1e-2, 0.1 * step_deg);  // perturbation, deg
        for (int it = 0; it < max_steps; ++it) {
            if (std::chrono::steady_clock::now() >= deadline) break;
            const CollisionDistanceSummary v0 = eval(cur);
            if (last_clear_m) *last_clear_m = v0.min_clearance_m;
            if (isClear(v0)) return path;
            const double c0 = v0.min_clearance_m;
            // Forward-difference gradient of min-clearance wrt each combined joint.
            Config grad{};
            double gnorm = 0.0;
            for (int k = 0; k < kCombined; ++k) {
                if (!active[k]) continue;
                const int j = k % kDof;
                Config cp = cur;
                cp[k] += eps;
                if (q_max_deg[j] > q_min_deg[j]) cp[k] = std::min(cp[k], q_max_deg[j]);
                grad[k] = (eval(cp).min_clearance_m - c0) / eps;
                gnorm += grad[k] * grad[k];
            }
            gnorm = std::sqrt(gnorm);
            if (gnorm < 1e-9) break;  // no ascent direction -> stuck
            // Line search: step step_deg along the unit gradient, halving until the
            // min-clearance strictly increases (or the step shrinks below a floor).
            bool improved = false;
            double s = step_deg;
            Config next = cur;
            for (int ls = 0; ls < 8; ++ls) {
                next = cur;
                for (int k = 0; k < kCombined; ++k) {
                    if (!active[k]) continue;
                    const int j = k % kDof;
                    next[k] += grad[k] / gnorm * s;
                    if (q_max_deg[j] > q_min_deg[j]) {
                        next[k] = std::clamp(next[k], q_min_deg[j], q_max_deg[j]);
                    }
                }
                const double next_clear = eval(next).min_clearance_m;
                if (last_clear_m) *last_clear_m = next_clear;
                if (next_clear > c0 + 1e-9) { improved = true; break; }
                s *= 0.5;
                if (s < 1e-2) break;
            }
            if (!improved) break;  // cannot climb further
            cur = next;
            path.push_back(cur);
        }
        return isClear(eval(cur)) ? path : std::vector<Config>{};
    }

    std::vector<Config> escapeToClear(const Config& start, const ActiveMask& active,
                                      double step_deg,
                                      std::chrono::steady_clock::time_point deadline,
                                      int* restarts_tried, double* last_clear_m) {
        if (restarts_tried) *restarts_tried = 0;
        std::vector<Config> path = escapeToClearAttempt(
            start, active, step_deg, cfg.escape_max_steps, deadline, last_clear_m);
        if (!path.empty() && clear(path.back())) return path;

        std::uniform_real_distribution<double> perturb(-cfg.escape_perturb_deg,
                                                       cfg.escape_perturb_deg);
        for (int attempt = 0; attempt < cfg.escape_restart_attempts; ++attempt) {
            if (std::chrono::steady_clock::now() >= deadline) break;
            if (restarts_tried) *restarts_tried = attempt + 1;
            Config seeded = start;
            for (int k = 0; k < kCombined; ++k) {
                if (!active[k]) continue;
                const int j = k % kDof;
                seeded[k] += perturb(rng);
                if (q_max_deg[j] > q_min_deg[j]) {
                    seeded[k] = std::clamp(seeded[k], q_min_deg[j], q_max_deg[j]);
                }
            }
            if (!edgeClear(start, seeded)) continue;
            std::vector<Config> restarted = escapeToClearAttempt(
                seeded, active, step_deg, cfg.escape_max_steps, deadline, last_clear_m);
            if (restarted.empty() || !clear(restarted.back())) continue;
            std::vector<Config> combined{start};
            combined.insert(combined.end(), restarted.begin(), restarted.end());
            return combined;
        }
        return {};
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

    static std::string edgeKey(const Config& a, const Config& b) {
        std::array<long long, kCombined> qa{};
        std::array<long long, kCombined> qb{};
        for (int i = 0; i < kCombined; ++i) {
            qa[i] = static_cast<long long>(std::llround(a[i] * 1000000.0));
            qb[i] = static_cast<long long>(std::llround(b[i] * 1000000.0));
        }
        if (qb < qa) std::swap(qa, qb);
        std::ostringstream oss;
        for (int i = 0; i < kCombined; ++i) oss << qa[i] << ',';
        oss << '|';
        for (int i = 0; i < kCombined; ++i) oss << qb[i] << ',';
        return oss.str();
    }

    int nearestLinear(const std::vector<Node>& tree, const Config& q) const {
        int best = 0;
        double best_d = std::numeric_limits<double>::infinity();
        for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
            if (tree[i].blocked) continue;
            const double d = dist(tree[i].q, q);
            if (d < best_d) {
                best_d = d;
                best = i;
            }
        }
        return best;
    }

    int nearest(const RrtTree& tree, const Config& q) const {
        const int kd_best = tree.kd.nearest(q, tree.nodes);
#ifndef NDEBUG
        const int lin_best = nearestLinear(tree.nodes, q);
        if (kd_best != lin_best) {
            std::cerr << "[WARN] InitMotion kd nearest mismatch kd=" << kd_best
                      << " linear=" << lin_best << "\n";
        }
#endif
        return kd_best;
    }

    enum class Extend { Trapped, Advanced, Reached };

    // Steer from the nearest node toward q_target by at most step; add the new node
    // if the edge to it is clear. Reached when the step lands exactly on q_target.
    Extend extend(RrtTree& tree, const Config& q_target, double step_deg) {
        const int ni = nearest(tree, q_target);
        if (ni < 0) return Extend::Trapped;
        const Config& q_near = tree.nodes[ni].q;
        const double d = dist(q_near, q_target);
        bool reached = false;
        Config q_new;
        if (d <= step_deg || d < 1e-9) {
            q_new = q_target;
            reached = true;
        } else {
            q_new = lerp(q_near, q_target, step_deg / d);
        }
        if (tree.blacklisted_edges.count(edgeKey(q_near, q_new)) != 0) {
            return Extend::Trapped;
        }
        const bool valid_edge = cfg.lazy_edges ? clear(q_new) : edgeClear(q_near, q_new);
        if (!valid_edge) return Extend::Trapped;
        const int new_index = static_cast<int>(tree.nodes.size());
        tree.nodes.push_back(Node{q_new, ni, !cfg.lazy_edges, false});
        tree.kd.insert(q_new, new_index);
        return reached ? Extend::Reached : Extend::Advanced;
    }

    Extend connect(RrtTree& tree, const Config& q_target, double step_deg) {
        Extend s = Extend::Advanced;
        while (s == Extend::Advanced) s = extend(tree, q_target, step_deg);
        return s;
    }

    static std::vector<Config> rootPath(const RrtTree& tree, int leaf) {
        std::vector<Config> path;
        for (int i = leaf; i >= 0; i = tree.nodes[i].parent) path.push_back(tree.nodes[i].q);
        std::reverse(path.begin(), path.end());  // root -> leaf
        return path;
    }

    void blockSubtree(RrtTree& tree, int root_idx) {
        if (root_idx <= 0 || root_idx >= static_cast<int>(tree.nodes.size())) return;
        std::vector<int> stack{root_idx};
        while (!stack.empty()) {
            const int cur = stack.back();
            stack.pop_back();
            if (cur < 0 || cur >= static_cast<int>(tree.nodes.size()) || tree.nodes[cur].blocked) {
                continue;
            }
            tree.nodes[cur].blocked = true;
            for (int i = 1; i < static_cast<int>(tree.nodes.size()); ++i) {
                if (tree.nodes[i].parent == cur) stack.push_back(i);
            }
        }
    }

    bool validatePathToRoot(RrtTree& tree, int leaf) {
        if (leaf < 0 || leaf >= static_cast<int>(tree.nodes.size())) return false;
        std::vector<int> ids;
        for (int i = leaf; i >= 0; i = tree.nodes[i].parent) ids.push_back(i);
        std::reverse(ids.begin(), ids.end());
        for (std::size_t p = 1; p < ids.size(); ++p) {
            const int child = ids[p];
            const int parent = tree.nodes[child].parent;
            if (tree.nodes[child].blocked) return false;
            if (tree.nodes[child].edge_validated) continue;
            const Config& a = tree.nodes[parent].q;
            const Config& b = tree.nodes[child].q;
            if (edgeClear(a, b)) {
                tree.nodes[child].edge_validated = true;
                continue;
            }
            tree.blacklisted_edges.insert(edgeKey(a, b));
            blockSubtree(tree, child);
            return false;
        }
        return true;
    }

    bool pathEdgesClear(const std::vector<Config>& path) {
        if (path.size() < 2) return true;
        for (std::size_t i = 1; i < path.size(); ++i) {
            if (!edgeClear(path[i - 1], path[i])) return false;
        }
        return true;
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
    return plan(start_left, start_right, true, goal_left, true, goal_right);
}

InitMotionPlanResult InitMotionPlanner::plan(
    const JointArray& start_left, const JointArray& start_right,
    bool left_active, const JointArray& goal_left,
    bool right_active, const JointArray& goal_right) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    InitMotionPlanResult result;

    Impl& d = *impl_;
    result.clear_threshold_m = d.clear_threshold_m;
    result.external_clear_threshold_m = d.external_clear_threshold_m;
    result.goal_clear_threshold_self_m = d.clear_threshold_m;
    result.goal_clear_threshold_external_m = d.external_clear_threshold_m;
    if (!left_active && !right_active) {
        result.message = "init motion plan: no active arm";
        result.fail_mode = InitMotionPlanResult::FailMode::NonFinite;
        result.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
        return result;
    }
    const ActiveMask active = makeActiveMask(left_active, right_active);
    const auto fill_nearest = [&](const CollisionDistanceSummary& s) {
        result.goal_self_min_clearance_m = s.self_min_clearance_m;
        result.goal_external_min_clearance_m = s.external_min_clearance_m;
        result.goal_nearest_pair_name_a = s.nearest_name_a;
        result.goal_nearest_pair_name_b = s.nearest_name_b;
        result.goal_nearest_pair_external = s.nearest_external;
        result.nearest_pair_distance_m = s.nearest_distance_m;
        result.nearest_pair_external = s.nearest_external;
        if (!s.nearest_name_a.empty() || !s.nearest_name_b.empty()) {
            result.nearest_pair = s.nearest_name_a + " <-> " + s.nearest_name_b;
        }
    };

    // Wrap the goal to the nearest in-range joint branch (per arm) vs the start.
    JointArray gl = left_active ? goal_left : start_left;
    JointArray gr = right_active ? goal_right : start_right;
    for (int i = 0; i < kDof; ++i) {
        if (left_active) {
            gl[i] = nearestBranch(goal_left[i], start_left[i], d.q_min_deg[i], d.q_max_deg[i]);
        }
        if (right_active) {
            gr[i] = nearestBranch(goal_right[i], start_right[i], d.q_min_deg[i], d.q_max_deg[i]);
        }
    }
    const Config start = join(start_left, start_right);
    const Config goal = join(gl, gr);

    const auto finite = [](const Config& c) {
        return std::all_of(c.begin(), c.end(), [](double v) { return std::isfinite(v); });
    };
    if (!finite(start) || !finite(goal)) {
        result.message = "init motion plan: non-finite start/goal";
        result.fail_mode = InitMotionPlanResult::FailMode::NonFinite;
        result.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
        return result;
    }
    const CollisionDistanceSummary start_summary = d.eval(start);
    const CollisionDistanceSummary goal_summary = d.eval(goal);
    result.start_clear_m = start_summary.min_clearance_m;
    result.goal_clear_m = goal_summary.min_clearance_m;
    fill_nearest(goal_summary);
    if (!d.isClear(goal_summary)) {
        // The init pose itself collides / dips below the floor — refuse (fail-closed).
        std::ostringstream m;
        m << "init motion plan: goal config not collision/floor clear"
          << " (goal_clear_m=" << result.goal_clear_m
          << ", thresh_m=" << d.clear_threshold_m
          << ", thresh_ext_m=" << d.external_clear_threshold_m
          << ", goal_self_min_clearance_m=" << result.goal_self_min_clearance_m
          << ", goal_external_min_clearance_m=" << result.goal_external_min_clearance_m
          << ", goal_nearest_pair_name_a="
          << (result.goal_nearest_pair_name_a.empty() ? std::string("unknown")
                                                       : result.goal_nearest_pair_name_a)
          << ", goal_nearest_pair_name_b="
          << (result.goal_nearest_pair_name_b.empty() ? std::string("unknown")
                                                       : result.goal_nearest_pair_name_b)
          << ", goal_nearest_pair_external=" << (result.goal_nearest_pair_external ? 1 : 0)
          << ", goal_clear_threshold_self_m=" << result.goal_clear_threshold_self_m
          << ", goal_clear_threshold_external_m=" << result.goal_clear_threshold_external_m
          << ", nearest_pair=" << (result.nearest_pair.empty() ? std::string("unknown") : result.nearest_pair)
          << ", nearest_pair_distance_m=" << result.nearest_pair_distance_m
          << ", nearest_pair_external=" << (result.nearest_pair_external ? 1 : 0) << ")";
        result.message = m.str();
        result.fail_mode = InitMotionPlanResult::FailMode::GoalNotClear;
        result.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
        return result;
    }

    const double step_deg = std::max(1e-3, d.cfg.step_size_rad * kRadToDeg);

    // Gradient escape: if the START pose is below the clearance gate (e.g., a gripper
    // resting a few mm off the floor after teleop), the bidirectional RRT cannot extend
    // out of it (every one-step edge is still sub-threshold). Climb along increasing
    // min-clearance until clear, then plan the rest from there; the escape segment (which
    // moves strictly AWAY from the obstacle) is prepended to the path below.
    std::vector<Config> escape_prefix;
    Config rrt_start = start;
    double escape_time_s = 0.0;
    int restarts_tried = 0;
    double last_escape_clear_m = result.start_clear_m;
    if (!d.clear(start)) {
        const auto escape_begin = Clock::now();
        const auto escape_deadline = escape_begin + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(d.cfg.escape_max_time_sec));
        escape_prefix = d.escapeToClear(start, active, step_deg, escape_deadline,
                                        &restarts_tried, &last_escape_clear_m);
        escape_time_s = std::chrono::duration<double>(Clock::now() - escape_begin).count();
        if (escape_prefix.size() < 2 || !d.clear(escape_prefix.back())) {
            std::ostringstream m;
            m << "init motion plan: start in near-collision and gradient escape failed"
              << " (start_clear_m=" << result.start_clear_m
              << ", escaped_clear_m=" << last_escape_clear_m
              << ", escape_steps=" << (escape_prefix.empty() ? 0 : escape_prefix.size() - 1)
              << ", restarts_tried=" << restarts_tried
              << ", escape_time_s=" << escape_time_s
              << ", thresh_self_m=" << d.clear_threshold_m
              << ", thresh_ext_m=" << d.external_clear_threshold_m << ")";
            result.message = m.str();
            result.fail_mode = InitMotionPlanResult::FailMode::EscapeFailed;
            result.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
            return result;
        }
        rrt_start = escape_prefix.back();
        std::cerr << "[INFO] InitMotion: start near-collision (clear_m="
                  << result.start_clear_m << "); gradient-escaped in "
                  << (escape_prefix.size() - 1) << " step(s) to clear_m="
                  << d.minClearance(rrt_start) << "\n";
    }
    const auto t_rrt_start = Clock::now();

    // Per-joint sampling band: [min(rrt_start,goal) - margin, max(rrt_start,goal) + margin]
    // clamped to the configured joint limits — keeps planning fast and avoids the
    // full +/-360 rbpodo range.
    Config lo{}, hi{}, glo{}, ghi{};
    const bool per_joint_margin_set = std::any_of(
        d.cfg.sample_margin_deg_per_joint.begin(), d.cfg.sample_margin_deg_per_joint.end(),
        [](double v) { return v > 0.0; });
    for (int k = 0; k < kCombined; ++k) {
        if (!active[k]) {
            lo[k] = rrt_start[k];
            hi[k] = rrt_start[k];
            glo[k] = rrt_start[k];
            ghi[k] = rrt_start[k];
            continue;
        }
        const int j = k % kDof;
        const double margin = (per_joint_margin_set && d.cfg.sample_margin_deg_per_joint[j] > 0.0)
            ? d.cfg.sample_margin_deg_per_joint[j]
            : d.cfg.sample_margin_deg;
        const double base_min = std::min(rrt_start[k], goal[k]);
        const double base_max = std::max(rrt_start[k], goal[k]);
        const double a = base_min - margin;
        const double b = base_max + margin;
        lo[k] = (d.q_max_deg[j] > d.q_min_deg[j]) ? std::max(a, d.q_min_deg[j]) : a;
        hi[k] = (d.q_max_deg[j] > d.q_min_deg[j]) ? std::min(b, d.q_max_deg[j]) : b;
        if (hi[k] < lo[k]) std::swap(hi[k], lo[k]);
        const double ga = base_min - d.cfg.global_sample_margin_deg;
        const double gb = base_max + d.cfg.global_sample_margin_deg;
        glo[k] = (d.q_max_deg[j] > d.q_min_deg[j]) ? std::max(ga, d.q_min_deg[j]) : ga;
        ghi[k] = (d.q_max_deg[j] > d.q_min_deg[j]) ? std::min(gb, d.q_max_deg[j]) : gb;
        if (ghi[k] < glo[k]) std::swap(ghi[k], glo[k]);
    }

    std::vector<Config> raw;

    // Fast path: the straight joint-space edge is already collision-free.
    if (d.edgeClear(rrt_start, goal)) {
        raw = {rrt_start, goal};
    } else {
        // Bidirectional RRT-Connect. trees[0] rooted at rrt_start, trees[1] at goal; the
        // last-added node of each tree is its connection endpoint on success.
        Impl::RrtTree tree_a;
        Impl::RrtTree tree_b;
        tree_a.reset(rrt_start);
        tree_b.reset(goal);
        Impl::RrtTree* trees[2] = {&tree_a, &tree_b};
        std::uniform_real_distribution<double> u(0.0, 1.0);
        bool connected = false;
        int iter = 0;
        for (; iter < d.cfg.max_iterations; ++iter) {
            if (std::chrono::duration<double>(Clock::now() - t_rrt_start).count() >
                d.cfg.max_planning_time_sec) {
                break;
            }
            const int i = iter % 2;
            const int j = 1 - i;
            // Goal-biased growth toward the opposite tree's root accelerates linking.
            Config q_rand{};
            const double r = u(d.rng);
            if (r < d.cfg.goal_bias) {
                q_rand = trees[j]->nodes.front().q;
            } else if (r < d.cfg.goal_bias + d.cfg.global_sample_fraction) {
                q_rand = d.sample(glo, ghi);
            } else {
                q_rand = d.sample(lo, hi);
            }
            const Impl::Extend ext = d.extend(*trees[i], q_rand, step_deg);
            if (ext == Impl::Extend::Trapped) continue;
            const Config q_new = trees[i]->nodes.back().q;
            if (d.connect(*trees[j], q_new, step_deg) == Impl::Extend::Reached) {
                if (d.validatePathToRoot(tree_a, static_cast<int>(tree_a.nodes.size()) - 1) &&
                    d.validatePathToRoot(tree_b, static_cast<int>(tree_b.nodes.size()) - 1)) {
                    connected = true;
                    break;
                }
            }
        }
        result.iterations = iter;
        result.tree_start_size = static_cast<int>(tree_a.nodes.size());
        result.tree_goal_size = static_cast<int>(tree_b.nodes.size());
        if (!connected) {
            result.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
            result.fail_mode = InitMotionPlanResult::FailMode::RrtBudget;
            const double rrt_time_s = std::chrono::duration<double>(Clock::now() - t_rrt_start).count();
            // Diagnostics: which budget ran out, how far each tree grew, and whether the
            // endpoints themselves are clear. tree_start staying ~1 means the START pose
            // could not escape (start in/near collision); both trees large but unconnected
            // means a narrow passage / the sample band (sample_margin_deg) is too tight or
            // the time/iteration budget is too small.
            std::ostringstream m;
            m << "init motion plan: RRT-Connect did not find a path within budget"
              << " (iters=" << iter << "/" << d.cfg.max_iterations
              << ", time=" << rrt_time_s << "/" << d.cfg.max_planning_time_sec << "s"
              << ", escape_time_s=" << escape_time_s
              << ", rrt_time_budget_s=" << d.cfg.max_planning_time_sec
              << ", tree_start=" << tree_a.nodes.size() << ", tree_goal=" << tree_b.nodes.size()
              << ", blacklisted_edges="
              << (tree_a.blacklisted_edges.size() + tree_b.blacklisted_edges.size())
              << ", start_clear_m=" << d.minClearance(rrt_start)
              << ", goal_clear_m=" << result.goal_clear_m
              << ", thresh_m=" << d.clear_threshold_m
              << ", sample_margin_deg=" << d.cfg.sample_margin_deg
              << ", global_sample_fraction=" << d.cfg.global_sample_fraction << ")";
            result.message = m.str();
            return result;
        }
        // start_path: start..q_new ; goal_path: goal..q_new -> reverse tail to append.
        std::vector<Config> start_path = Impl::rootPath(tree_a, static_cast<int>(tree_a.nodes.size()) - 1);
        std::vector<Config> goal_path = Impl::rootPath(tree_b, static_cast<int>(tree_b.nodes.size()) - 1);
        std::reverse(goal_path.begin(), goal_path.end());  // q_new..goal
        raw = start_path;
        raw.insert(raw.end(), goal_path.begin() + 1, goal_path.end());  // drop dup q_new
    }

    // Shortcut the RRT/straight portion FIRST (from rrt_start), then prepend the escape
    // segment untouched. Doing it in this order keeps the escape prefix exact so its
    // densified length (the leading waypoints to follow precisely) is known.
    d.shortcut(raw);
    if (!d.pathEdgesClear(raw)) {
        result.success = false;
        result.fail_mode = InitMotionPlanResult::FailMode::RrtBudget;
        result.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
        result.message = "init motion plan: final RRT path validation failed";
        return result;
    }
    int escape_waypoints = 0;
    if (!escape_prefix.empty()) {
        // Densified length of the escape segment = the leading waypoints of the final
        // path (densify is segment-local, so the escape head densifies identically).
        escape_waypoints = static_cast<int>(d.densify(escape_prefix).size());
        std::vector<Config> combined = std::move(escape_prefix);
        combined.insert(combined.end(), raw.begin() + 1, raw.end());  // drop dup rrt_start
        raw = std::move(combined);
    }

    const std::vector<Config> dense = d.densify(raw);
    result.escape_waypoints = escape_waypoints;

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

    // Diagnostic: how far the IK'd goal is from the current pose. ~0 means the requested
    // TCP target equals the current TCP (marker following current) -> a near no-op move.
    for (int i = 0; i < kDof; ++i) {
        res.goal_vs_start_max_deg = std::max(res.goal_vs_start_max_deg,
            std::abs(res.goal_left[i] - start_left[i]));
        res.goal_vs_start_max_deg = std::max(res.goal_vs_start_max_deg,
            std::abs(res.goal_right[i] - start_right[i]));
    }

    // Straight Cartesian-path feasibility: sample the exact MoveL path, per-sample IK
    // (seeded for joint continuity), oracle-check each combined config. An inactive arm
    // holds at its start config across the whole path.
    const Pose6D start_pose_left = left_active
        ? d.kin->computeTcpStand(ArmId::Left, start_left, d.left_mount) : Pose6D{};
    const Pose6D start_pose_right = right_active
        ? d.kin->computeTcpStand(ArmId::Right, start_right, d.right_mount) : Pose6D{};

    // Diagnostic: TCP position distance (m) from current to the requested target. Tells a
    // small gizmo drag (small metres) apart from a frame/IK oddity (large metres).
    const auto posDist = [](const Pose6D& a, const Pose6D& b) {
        const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };
    if (left_active) {
        res.goal_vs_start_cart_m = std::max(res.goal_vs_start_cart_m,
            posDist(start_pose_left, goal_pose_left));
    }
    if (right_active) {
        res.goal_vs_start_cart_m = std::max(res.goal_vs_start_cart_m,
            posDist(start_pose_right, goal_pose_right));
    }
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
    InitMotionPlanResult plan_res = plan(
        start_left, start_right, left_active, res.goal_left, right_active, res.goal_right);
    res.planning_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
    if (plan_res.success && !plan_res.waypoints.empty()) {
        res.decision = InitMotionLinearResult::Decision::Detour;
        res.waypoints = std::move(plan_res.waypoints);
        res.escape_waypoints = plan_res.escape_waypoints;
        res.message = "detour: " + plan_res.message;
    } else {
        res.decision = InitMotionLinearResult::Decision::Failed;
        res.message = "detour planning failed: " + plan_res.message;
    }
    return res;
}

}  // namespace rb_servo
