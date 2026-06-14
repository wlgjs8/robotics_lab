#include "rb_servo/control/collision_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <coal/mesh_loader/loader.h>
#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>

#include <Eigen/Geometry>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

namespace rb_servo {

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

double nowMonotonicS() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool nameContainsAny(const std::string& name, const std::vector<std::string>& subs) {
    for (const auto& s : subs) {
        if (name.find(s) != std::string::npos) return true;
    }
    return false;
}
}  // namespace

bool collisionVerdictStale(const CollisionVerdict& v, double now_s, double max_staleness_s) {
    if (!v.valid) return true;
    return (now_s - v.stamp_s) > max_staleness_s;
}

double collisionVelocityScale(const CollisionVerdict& v, const CollisionMonitorConfig& cfg) {
    if (!v.valid) return 0.0;            // no verdict yet -> fail closed
    constexpr double kRetreatEps = 1e-4;  // m/s; clearance must clearly grow to pass
    if (v.hard_violation) {
        // Inside the hard floor: HOLD (fail safe) unless the clearance is clearly
        // increasing (a genuine retreat), so the arm can still back out but cannot
        // sink deeper. Stationary/uncertain -> hold.
        return v.clearance_rate_m_s > kRetreatEps ? 1.0 : 0.0;
    }
    const double d = v.min_clearance_m;
    if (d > cfg.d_slow_m) return 1.0;    // far enough: barrier inactive
    const double vc = v.closing_speed_m_s;
    if (vc <= 1e-6) return 1.0;          // not closing (parallel/receding): free
    // Predicted clearance when the (latency-old) verdict is acted upon.
    const double d_eff = d - cfg.d_hard_m - vc * cfg.latency_s;
    if (d_eff <= 0.0) return 0.0;        // cannot guarantee a stop -> halt
    const double v_allow = std::sqrt(2.0 * cfg.a_brake_m_s2 * d_eff);
    if (vc <= v_allow) return 1.0;
    const double scale = v_allow / vc;
    return scale < 0.0 ? 0.0 : (scale > 1.0 ? 1.0 : scale);
}

struct CollisionMonitor::Impl {
    CollisionMonitorConfig cfg;

    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::GeometryModel geom;
    pinocchio::GeometryData gdata;
    Eigen::VectorXd q;
    std::array<int, kDof> left_qidx{};
    std::array<int, kDof> right_qidx{};
    // arm classification (frame ancestry) + chain depth (joint supports)
    pinocchio::FrameIndex left_root_fid = 0;
    pinocchio::FrameIndex right_root_fid = 0;
    bool have_arm_roots = false;
    std::array<pinocchio::JointIndex, kDof> left_jids{};
    std::array<pinocchio::JointIndex, kDof> right_jids{};
    // swept-volume: previous evaluated configuration
    Eigen::VectorXd prev_eval_q;
    bool have_prev_eval = false;

    // submitted targets (guarded; tiny critical section)
    mutable std::mutex in_mtx;
    JointArray left_deg{};
    JointArray right_deg{};
    bool has_targets = false;

    // published verdict (lock-free read via atomic shared_ptr)
    std::shared_ptr<const CollisionVerdict> published;

    // clearance-rate estimation state (monitor thread only): per-pair clearance
    // from the previous eval, indexed by collision-pair index, so the rate of the
    // currently-critical pair is tracked even as which pair is closest changes.
    std::vector<double> prev_pair_clear;
    double prev_stamp = 0.0;
    std::uint64_t seq = 0;

    std::thread thread;
    std::atomic<bool> running{false};

    explicit Impl(CollisionMonitorConfig c) : cfg(std::move(c)), data(), gdata() {
        buildModel();
        buildGeometry();
        gdata = pinocchio::GeometryData(geom);
        for (auto& req : gdata.distanceRequests) req.enable_nearest_points = true;
        // start with a neutral verdict (invalid -> fail closed until first eval)
        std::atomic_store(&published, std::make_shared<const CollisionVerdict>());
    }

    void buildModel() {
        pinocchio::urdf::buildModel(cfg.unified_urdf, model);
        data = pinocchio::Data(model);
        q = pinocchio::neutral(model);
        for (int i = 0; i < kDof; ++i) {
            const auto lj = model.getJointId(cfg.left_joints[i]);
            const auto rj = model.getJointId(cfg.right_joints[i]);
            left_jids[i] = lj;
            right_jids[i] = rj;
            left_qidx[i] = model.joints[lj].idx_q();
            right_qidx[i] = model.joints[rj].idx_q();
        }
        // Arm root frames for ancestry-based classification (default "<prefix>world").
        const std::string lr = cfg.left_arm_root_frame.empty() ? (cfg.left_prefix + "world")
                                                               : cfg.left_arm_root_frame;
        const std::string rr = cfg.right_arm_root_frame.empty() ? (cfg.right_prefix + "world")
                                                                : cfg.right_arm_root_frame;
        if (model.existFrame(lr) && model.existFrame(rr)) {
            left_root_fid = model.getFrameId(lr);
            right_root_fid = model.getFrameId(rr);
            have_arm_roots = true;
        }
    }

    // True if `start` frame has `root` on its parent-frame chain (kinematic ancestry).
    bool frameDescendsFrom(pinocchio::FrameIndex start, pinocchio::FrameIndex root) const {
        pinocchio::FrameIndex f = start;
        for (int guard = 0; guard < 1024; ++guard) {
            if (f == root) return true;
            const pinocchio::FrameIndex p = model.frames[f].parentFrame;
            if (p == f) break;  // reached the universe/root frame
            f = p;
        }
        return false;
    }

    enum class Side { Left, Right, Stand };
    Side classify(std::size_t geom_idx) const {
        const auto& go = geom.geometryObjects[geom_idx];
        if (have_arm_roots) {
            if (frameDescendsFrom(go.parentFrame, left_root_fid)) return Side::Left;
            if (frameDescendsFrom(go.parentFrame, right_root_fid)) return Side::Right;
            return Side::Stand;
        }
        // Fallback: name substring (only if the arm root frames were not found).
        if (go.name.find("left") != std::string::npos) return Side::Left;
        if (go.name.find("right") != std::string::npos) return Side::Right;
        return Side::Stand;
    }

    // Chain depth of an arm geometry: number of that arm's actuated joints on the
    // path universe->parentJoint (link0->0, link1->1, ..., link6/gripper->6).
    int chainDepth(std::size_t geom_idx, Side side) const {
        const pinocchio::JointIndex pj = geom.geometryObjects[geom_idx].parentJoint;
        const auto& sup = model.supports[pj];  // joints from universe to pj
        const auto& jids = (side == Side::Left) ? left_jids : right_jids;
        int depth = 0;
        for (pinocchio::JointIndex j : sup)
            for (int k = 0; k < kDof; ++k)
                if (j == jids[k]) { depth = std::max(depth, k + 1); }
        return depth;
    }

    void buildGeometry() {
        pinocchio::urdf::buildGeom(model, cfg.unified_urdf, pinocchio::COLLISION, geom,
                                   cfg.package_dirs);
        // convexify mesh (BVH) geometry for fast GJK; primitives untouched
        for (auto& go : geom.geometryObjects) {
            auto bvh = std::dynamic_pointer_cast<coal::BVHModelBase>(go.geometry);
            if (bvh) {
                bvh->buildConvexRepresentation(false);
                if (bvh->convex) go.geometry = bvh->convex;
            }
        }
        // attach Pika gripper convex hull to each arm's attachment_site frame
        if (!cfg.pika_gripper_mesh.empty()) {
            coal::MeshLoader loader;
            auto pbvh = loader.load(cfg.pika_gripper_mesh, coal::Vec3s(0.001, 0.001, 0.001));
            pbvh->buildConvexRepresentation(false);
            auto hull = pbvh->convex;
            const Eigen::Matrix3d rz =
                Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            for (const std::string prefix : {cfg.left_prefix, cfg.right_prefix}) {
                const auto fid = model.getFrameId(prefix + "attachment_site");
                const auto& fr = model.frames[fid];
                const pinocchio::SE3 place =
                    fr.placement * pinocchio::SE3(rz, Eigen::Vector3d::Zero());
                geom.addGeometryObject(pinocchio::GeometryObject(
                    prefix + "pika_gripper", fr.parentJoint, fr.parentFrame, place, hull));
            }
        }
        // Extra non-URDF collision primitives (wrist cameras, cables, table). An
        // arm-frame parent moves with the arm (classified left/right by ancestry);
        // a stand/world parent is a static obstacle paired against both arms.
        for (const auto& e : cfg.extra_collision) {
            if (!model.existFrame(e.parent_frame)) {
                throw std::runtime_error("collision_monitor: extra_collision '" + e.name +
                                         "' parent_frame not found: " + e.parent_frame);
            }
            std::shared_ptr<coal::CollisionGeometry> shape = makeExtraShape(e);
            const auto fid = model.getFrameId(e.parent_frame);
            const auto& fr = model.frames[fid];
            const Eigen::Matrix3d rot = pinocchio::rpy::rpyToMatrix(e.rpy[0], e.rpy[1], e.rpy[2]);
            const pinocchio::SE3 local(rot, Eigen::Vector3d(e.xyz_m[0], e.xyz_m[1], e.xyz_m[2]));
            geom.addGeometryObject(pinocchio::GeometryObject(
                e.name, fr.parentJoint, fid, fr.placement * local, shape));
        }
        curatePairs();
    }

    static std::shared_ptr<coal::CollisionGeometry> makeExtraShape(const ExtraCollisionShape& e) {
        if (e.shape == "box")
            return std::make_shared<coal::Box>(e.size_m[0], e.size_m[1], e.size_m[2]);
        if (e.shape == "sphere")
            return std::make_shared<coal::Sphere>(e.radius_m);
        if (e.shape == "capsule")
            return std::make_shared<coal::Capsule>(e.radius_m, e.length_m);
        if (e.shape == "cylinder")
            return std::make_shared<coal::Cylinder>(e.radius_m, e.length_m);
        throw std::runtime_error("collision_monitor: unknown extra_collision shape '" + e.shape +
                                 "' for '" + e.name + "'");
    }

    void curatePairs() {
        std::vector<std::size_t> li, ri, si;
        for (std::size_t i = 0; i < geom.geometryObjects.size(); ++i) {
            switch (classify(i)) {
                case Side::Left: li.push_back(i); break;
                case Side::Right: ri.push_back(i); break;
                case Side::Stand: si.push_back(i); break;
            }
        }
        geom.removeAllCollisionPairs();
        std::size_t n_lr = 0, n_arm_stand = 0, n_intra = 0;
        // left <-> right (whole arms)
        for (auto a : li)
            for (auto b : ri) { geom.addCollisionPair(pinocchio::CollisionPair(a, b)); ++n_lr; }
        // arm <-> stand (skip the bolted-on mount neighbors)
        for (const auto& arm : {li, ri}) {
            for (auto a : arm) {
                if (nameContainsAny(geom.geometryObjects[a].name, cfg.stand_ignore_arm_substrings))
                    continue;
                for (auto b : si) { geom.addCollisionPair(pinocchio::CollisionPair(a, b)); ++n_arm_stand; }
            }
        }
        // intra-arm (arm folding onto itself): only NON-adjacent links of the same arm
        if (cfg.check_intra_arm) {
            const int sep = cfg.intra_arm_min_chain_separation;
            auto intra = [&](const std::vector<std::size_t>& g, Side sd) {
                for (std::size_t x = 0; x < g.size(); ++x)
                    for (std::size_t y = x + 1; y < g.size(); ++y) {
                        if (std::abs(chainDepth(g[x], sd) - chainDepth(g[y], sd)) < sep) continue;
                        geom.addCollisionPair(pinocchio::CollisionPair(g[x], g[y]));
                        ++n_intra;
                    }
            };
            intra(li, Side::Left);
            intra(ri, Side::Right);
        }
        // STARTUP LOG: what the guard actually checks (operator confirmation).
        std::cerr << "[collision_monitor] geoms=" << geom.geometryObjects.size()
                  << " (left=" << li.size() << " right=" << ri.size() << " stand=" << si.size()
                  << (have_arm_roots ? ", classify=frame-ancestry" : ", classify=name-substring(FALLBACK)")
                  << ") pairs=" << geom.collisionPairs.size()
                  << " [left-right=" << n_lr << " arm-stand=" << n_arm_stand
                  << " intra-arm=" << n_intra << "]"
                  << " stand_ignore=[";
        for (std::size_t i = 0; i < cfg.stand_ignore_arm_substrings.size(); ++i)
            std::cerr << (i ? "," : "") << cfg.stand_ignore_arm_substrings[i];
        std::cerr << "] swept_samples=" << cfg.swept_samples
                  << (cfg.pika_gripper_mesh.empty() ? " gripper=NONE" : " gripper=attached")
                  << std::endl;
    }

    void setQ(const JointArray& l, const JointArray& r) {
        for (int i = 0; i < kDof; ++i) {
            q[left_qidx[i]] = l[i] * kDeg2Rad;
            q[right_qidx[i]] = r[i] * kDeg2Rad;
        }
    }

    CollisionVerdict reduce(double stamp) {
        CollisionVerdict v;
        v.stamp_s = stamp;
        v.valid = true;
        const std::size_t np = geom.collisionPairs.size();
        // collect (distance, pair index), partial-sort the closest K
        std::vector<std::pair<double, std::size_t>> ds;
        ds.reserve(np);
        std::vector<double> cur(np);
        double dmin = std::numeric_limits<double>::infinity();
        std::size_t kmin = 0;
        for (std::size_t k = 0; k < np; ++k) {
            const double d = gdata.distanceResults[k].min_distance;
            cur[k] = d;
            ds.emplace_back(d, k);
            if (d < dmin) { dmin = d; kmin = k; }
        }
        v.min_clearance_m = dmin;
        v.hard_violation = dmin < cfg.d_hard_m;
        const std::size_t K = std::min<std::size_t>(cfg.max_near_pairs, ds.size());
        std::partial_sort(ds.begin(), ds.begin() + K, ds.end(),
                          [](const auto& a, const auto& b) { return a.first < b.first; });
        v.near.reserve(K);
        for (std::size_t i = 0; i < K; ++i) {
            const std::size_t k = ds[i].second;
            const auto& cp = geom.collisionPairs[k];
            const auto& res = gdata.distanceResults[k];
            CollisionNearPair p;
            p.geom_a = static_cast<int>(cp.first);
            p.geom_b = static_cast<int>(cp.second);
            p.d_m = res.min_distance;
            p.p_a = res.nearest_points[0];
            p.p_b = res.nearest_points[1];
            const Eigen::Vector3d delta = p.p_b - p.p_a;
            p.n = delta.norm() > 1e-9 ? Eigen::Vector3d(delta.normalized()) : Eigen::Vector3d::UnitZ();
            p.name_a = geom.geometryObjects[cp.first].name;
            p.name_b = geom.geometryObjects[cp.second].name;
            v.near.push_back(std::move(p));
        }
        // Signed clearance rate of the currently-critical pair, tracked by pair
        // index so a switch of which pair is closest does not corrupt it.
        if (prev_pair_clear.size() == np && stamp > prev_stamp) {
            v.clearance_rate_m_s = (dmin - prev_pair_clear[kmin]) / (stamp - prev_stamp);
            v.closing_speed_m_s = v.clearance_rate_m_s < 0.0 ? -v.clearance_rate_m_s : 0.0;
        }
        prev_pair_clear = std::move(cur);
        prev_stamp = stamp;
        v.seq = ++seq;
        return v;
    }

    CollisionVerdict evalLocked(const JointArray& l, const JointArray& r) {
        setQ(l, r);  // writes the target config into q
        const int N = std::max(1, cfg.swept_samples);
        if (N <= 1 || !have_prev_eval) {
            pinocchio::computeDistances(model, data, geom, gdata, q);
        } else {
            // Swept-volume: sample prev_eval_q -> q (joint-space linear; the model
            // is revolute-only so this is exact), keep the worst (min) sample so a
            // fast step cannot tunnel a thin obstacle between evaluations.
            const std::size_t np = geom.collisionPairs.size();
            double worst = std::numeric_limits<double>::infinity();
            double worst_alpha = 1.0;
            Eigen::VectorXd qi(q.size());
            for (int s = 1; s <= N; ++s) {
                const double a = static_cast<double>(s) / static_cast<double>(N);
                qi = prev_eval_q + a * (q - prev_eval_q);
                pinocchio::computeDistances(model, data, geom, gdata, qi);
                double m = std::numeric_limits<double>::infinity();
                for (std::size_t k = 0; k < np; ++k)
                    m = std::min(m, gdata.distanceResults[k].min_distance);
                if (m < worst) { worst = m; worst_alpha = a; }
            }
            if (worst_alpha != 1.0) {  // leave gdata at the worst sample for reduce()
                qi = prev_eval_q + worst_alpha * (q - prev_eval_q);
                pinocchio::computeDistances(model, data, geom, gdata, qi);
            }
        }
        prev_eval_q = q;
        have_prev_eval = true;
        return reduce(nowMonotonicS());
    }

    void publish(CollisionVerdict v) {
        std::atomic_store(&published, std::make_shared<const CollisionVerdict>(std::move(v)));
    }

    CollisionVerdict load() const {
        auto p = std::atomic_load(&published);
        return p ? *p : CollisionVerdict{};
    }

    void run() {
        pinThread();
        while (running.load(std::memory_order_relaxed)) {
            JointArray l, r;
            bool ready;
            {
                std::lock_guard<std::mutex> lk(in_mtx);
                ready = has_targets;
                l = left_deg;
                r = right_deg;
            }
            if (!ready) {
                std::this_thread::sleep_for(std::chrono::microseconds(200));
                continue;
            }
            publish(evalLocked(l, r));
        }
    }

    void pinThread() {
#if defined(__linux__)
        if (cfg.monitor_core >= 0) {
            cpu_set_t set;
            CPU_ZERO(&set);
            CPU_SET(cfg.monitor_core, &set);
            pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
        }
#endif
    }
};

CollisionMonitor::CollisionMonitor(CollisionMonitorConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

CollisionMonitor::~CollisionMonitor() { stop(); }

void CollisionMonitor::submitTargets(const JointArray& left_deg, const JointArray& right_deg) {
    std::lock_guard<std::mutex> lk(impl_->in_mtx);
    impl_->left_deg = left_deg;
    impl_->right_deg = right_deg;
    impl_->has_targets = true;
}

CollisionVerdict CollisionMonitor::latest() const { return impl_->load(); }

CollisionVerdict CollisionMonitor::evalOnce(const JointArray& left_deg, const JointArray& right_deg) {
    CollisionVerdict v = impl_->evalLocked(left_deg, right_deg);
    impl_->publish(v);
    return v;
}

void CollisionMonitor::start() {
    if (impl_->running.exchange(true)) return;
    impl_->thread = std::thread([this] { impl_->run(); });
}

void CollisionMonitor::stop() {
    if (!impl_->running.exchange(false)) return;
    if (impl_->thread.joinable()) impl_->thread.join();
}

std::size_t CollisionMonitor::numGeometries() const { return impl_->geom.geometryObjects.size(); }
std::size_t CollisionMonitor::numPairs() const { return impl_->geom.collisionPairs.size(); }

}  // namespace rb_servo
