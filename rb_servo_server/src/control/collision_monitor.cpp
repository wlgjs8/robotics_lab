#include "rb_servo/control/collision_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <coal/mesh_loader/loader.h>
#include <coal/BVH/BVH_model.h>

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
    const double vc = v.closing_speed_m_s;
    if (v.hard_violation) {
        // inside the hard floor: block further approach, but allow a retreat
        // (closing speed <= 0) so the arm can back out of the keep-out zone.
        return vc > 1e-6 ? 0.0 : 1.0;
    }
    const double d = v.min_clearance_m;
    if (d > cfg.d_slow_m) return 1.0;    // far enough: barrier inactive
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

    // submitted targets (guarded; tiny critical section)
    mutable std::mutex in_mtx;
    JointArray left_deg{};
    JointArray right_deg{};
    bool has_targets = false;

    // published verdict (lock-free read via atomic shared_ptr)
    std::shared_ptr<const CollisionVerdict> published;

    // closing-speed estimation state (monitor thread only)
    double prev_min = std::numeric_limits<double>::quiet_NaN();
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
            left_qidx[i] = model.joints[model.getJointId(cfg.left_joints[i])].idx_q();
            right_qidx[i] = model.joints[model.getJointId(cfg.right_joints[i])].idx_q();
        }
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
        curatePairs();
    }

    enum class Side { Left, Right, Stand };
    Side classify(const std::string& nm) const {
        if (nm.find("left") != std::string::npos) return Side::Left;
        if (nm.find("right") != std::string::npos) return Side::Right;
        return Side::Stand;
    }

    void curatePairs() {
        std::vector<std::size_t> li, ri, si;
        for (std::size_t i = 0; i < geom.geometryObjects.size(); ++i) {
            switch (classify(geom.geometryObjects[i].name)) {
                case Side::Left: li.push_back(i); break;
                case Side::Right: ri.push_back(i); break;
                case Side::Stand: si.push_back(i); break;
            }
        }
        geom.removeAllCollisionPairs();
        for (auto a : li)
            for (auto b : ri) geom.addCollisionPair(pinocchio::CollisionPair(a, b));
        for (const auto& arm : {li, ri}) {
            for (auto a : arm) {
                if (nameContainsAny(geom.geometryObjects[a].name, cfg.stand_ignore_arm_substrings))
                    continue;  // mount neighbor: never paired vs stand
                for (auto b : si) geom.addCollisionPair(pinocchio::CollisionPair(a, b));
            }
        }
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
        double dmin = std::numeric_limits<double>::infinity();
        for (std::size_t k = 0; k < np; ++k) {
            const double d = gdata.distanceResults[k].min_distance;
            ds.emplace_back(d, k);
            if (d < dmin) dmin = d;
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
        // closing speed from consecutive global minima
        if (std::isfinite(prev_min) && stamp > prev_stamp) {
            const double dd = (prev_min - dmin) / (stamp - prev_stamp);
            v.closing_speed_m_s = dd > 0.0 ? dd : 0.0;
        }
        prev_min = dmin;
        prev_stamp = stamp;
        v.seq = ++seq;
        return v;
    }

    CollisionVerdict evalLocked(const JointArray& l, const JointArray& r) {
        setQ(l, r);
        pinocchio::computeDistances(model, data, geom, gdata, q);
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
