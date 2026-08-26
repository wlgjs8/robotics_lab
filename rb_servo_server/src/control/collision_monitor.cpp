#include "rb_servo/control/collision_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
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

bool nameStartsWith(const std::string& name, const std::string& prefix) {
    return name.rfind(prefix, 0) == 0;
}

bool globMatch(const std::string& pattern, const std::string& text) {
    std::size_t p = 0;
    std::size_t t = 0;
    std::size_t star = std::string::npos;
    std::size_t match = 0;
    while (t < text.size()) {
        if (p < pattern.size() && pattern[p] == text[t]) {
            ++p;
            ++t;
        } else if (p < pattern.size() && pattern[p] == '*') {
            star = p++;
            match = t;
        } else if (star != std::string::npos) {
            p = star + 1;
            t = ++match;
        } else {
            return false;
        }
    }
    while (p < pattern.size() && pattern[p] == '*') ++p;
    return p == pattern.size();
}

std::string ruleString(const CollisionPairPattern& rule) {
    return "[\"" + rule.pattern_a + "\",\"" + rule.pattern_b + "\"]";
}

// True iff the BVH triangle mesh is (numerically) convex: every vertex lies on the
// inner side of every face plane within `tol` meters. WHY THIS EXISTS: coal's
// BVHModelBase::buildConvexRepresentation() does NOT compute a convex hull — its own
// header says it "only takes the points of this model, does not check that the object
// is convex, does not compute a convex hull." This coal build also lacks qhull (so
// buildConvexHull() throws). Feeding a NON-convex mesh through buildConvexRepresentation
// yields an invalid Convex whose GJK distance is garbage (measured: two gripper-base
// hulls read 125 mm apart at a pose where the true clearance was 21 mm), which silently
// disabled the gripper<->gripper self-collision guard. Callers use this to refuse the
// broken convex and keep the (correct, if slower) BVH mesh instead. Convex meshes have
// their vertex-average strictly interior, so face normals orient outward reliably and
// the test returns true with ~0 defect; non-convex meshes have a poking vertex.
bool bvhMeshIsConvex(const coal::BVHModelBase& m, double tol) {
    if (!m.vertices || !m.tri_indices || m.num_vertices == 0 || m.num_tris == 0) {
        return false;  // cannot validate -> treat as non-convex (fail-closed to BVH)
    }
    const std::vector<coal::Vec3s>& V = *m.vertices;
    const std::vector<coal::Triangle>& T = *m.tri_indices;
    coal::Vec3s centroid = coal::Vec3s::Zero();
    for (unsigned i = 0; i < m.num_vertices; ++i) centroid += V[i];
    centroid /= static_cast<double>(m.num_vertices);
    for (unsigned t = 0; t < m.num_tris; ++t) {
        const coal::Vec3s& a = V[T[t][0]];
        const coal::Vec3s& b = V[T[t][1]];
        const coal::Vec3s& c = V[T[t][2]];
        coal::Vec3s n = (b - a).cross(c - a);
        const double nn = n.norm();
        if (nn < 1e-12) continue;            // degenerate triangle: skip
        n /= nn;
        if (n.dot(centroid - a) > 0.0) n = -n;  // orient outward (away from interior)
        for (unsigned i = 0; i < m.num_vertices; ++i) {
            if (n.dot(V[i] - a) > tol) return false;  // a vertex pokes past this face
        }
    }
    return true;
}
constexpr double kConvexTolM = 1e-3;  // 1 mm: real hull meshes defect ~0, gripper ~cm
}  // namespace

bool collisionPairPatternMatches(const CollisionPairPattern& rule,
                                 const std::string& name_a,
                                 const std::string& name_b) {
    return (globMatch(rule.pattern_a, name_a) && globMatch(rule.pattern_b, name_b)) ||
           (globMatch(rule.pattern_a, name_b) && globMatch(rule.pattern_b, name_a));
}

bool collisionVerdictStale(const CollisionVerdict& v, double now_s, double max_staleness_s) {
    if (!v.valid) return true;
    return (now_s - v.stamp_s) > max_staleness_s;
}

const char* externalBoxFeedAbortReason(bool feed_seen, double since_enforce_start_s,
                                       double since_last_feed_s, double startup_grace_s,
                                       double feed_timeout_s) {
    if (!feed_seen) {
        if (since_enforce_start_s > startup_grace_s) {
            return "no SetExternalBoxes feed received since startup (producer not running?)";
        }
        return nullptr;  // still inside startup grace
    }
    if (since_last_feed_s > feed_timeout_s) {
        return "SetExternalBoxes feed went stale (producer stopped sending)";
    }
    return nullptr;
}

double collisionVelocityScale(const CollisionVerdict& v, const CollisionMonitorConfig& cfg,
                              double verdict_age_s) {
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
    // Latency compensation, age-extrapolated (Issue 2a): use the larger of the
    // fixed floor and the actual verdict age so a lagging async verdict never
    // under-compensates the closing distance travelled before the command lands.
    const double comp_s =
        verdict_age_s >= 0.0 ? std::max(cfg.latency_s, verdict_age_s) : cfg.latency_s;
    // Predicted clearance when the (latency-old) verdict is acted upon.
    const double d_eff = d - cfg.d_hard_m - vc * comp_s;
    if (d_eff <= 0.0) return 0.0;        // cannot guarantee a stop -> halt
    const double v_allow = std::sqrt(2.0 * cfg.a_brake_m_s2 * d_eff);
    if (vc <= v_allow) return 1.0;
    const double scale = v_allow / vc;
    return scale < 0.0 ? 0.0 : (scale > 1.0 ? 1.0 : scale);
}

void buildCollisionConstraints(const CollisionVerdict& v, const CollisionMonitorConfig& cfg,
                               double verdict_age_s, std::vector<VelocityConstraint>& out,
                               std::unordered_set<std::uint64_t>* engaged_pairs) {
    if (!v.valid) return;
    const double age = std::max(0.0, verdict_age_s);
    for (const auto& p : v.near) {
        // Per-category barrier set. A keep-out BOX gets its own set (WIDE slow zone so a
        // fast teleop approach is braked before the hard floor — the floor's 5 mm slow
        // zone stops only ~0.12 m/s and let teleop overshoot ~40 mm into the box). The
        // floor (ground_plane, `external`) keeps the tight external_* set; intra-arm and
        // arm<->arm/stand keep their own. Monitor-only boxes are reported but not enforced.
        if (p.external_box && cfg.external_boxes.monitor_only) continue;
        const double d_hard = p.external_box ? cfg.external_box_d_hard_m
                            : p.external     ? cfg.external_d_hard_m
                            : p.intra_arm    ? cfg.intra_arm_d_hard_m
                                             : cfg.d_hard_m;
        const double d_slow = p.external_box ? cfg.external_box_d_slow_m
                            : p.external     ? cfg.external_d_slow_m
                            : p.intra_arm    ? cfg.intra_arm_d_slow_m
                                             : cfg.d_slow_m;
        const double a_brake = p.external_box ? cfg.external_box_a_brake_m_s2
                             : p.external     ? cfg.external_a_brake_m_s2
                             : p.intra_arm    ? cfg.intra_arm_a_brake_m_s2
                                              : cfg.a_brake_m_s2;
        const double recover = p.external_box ? cfg.external_box_recover_speed_m_s
                             : p.external     ? cfg.external_recover_speed_m_s
                             : p.intra_arm    ? cfg.intra_arm_recover_speed_m_s
                                              : cfg.recover_speed_m_s;
        const double hyst = p.external_box ? cfg.external_box_hyst_m
                          : p.external     ? cfg.external_hyst_m
                          : p.intra_arm    ? cfg.intra_arm_hyst_m
                                           : cfg.hyst_m;
        const double closing = p.rate_m_s < 0.0 ? -p.rate_m_s : 0.0;
        const double d_now = p.d_m - closing * age;  // age-extrapolated clearance
        // Engage/release hysteresis (caller-persisted): engage below d_slow,
        // release only above d_slow + hyst. In the hysteresis band the row
        // exists but xi is large, so it rarely binds — the point is that it
        // does not flap with margin noise.
        const std::uint64_t pair_key =
            (static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.geom_a)) << 32) |
            static_cast<std::uint32_t>(p.geom_b);
        const bool was_engaged = engaged_pairs && engaged_pairs->count(pair_key) > 0;
        const double engage_at = was_engaged ? d_slow + hyst : d_slow;
        if (d_now >= engage_at) {
            if (was_engaged) engaged_pairs->erase(pair_key);
            continue;
        }
        if (engaged_pairs) engaged_pairs->insert(pair_key);
        VelocityConstraint c;
        for (int i = 0; i < kDof; ++i) {
            c.J[i] = p.Jn_left[i];
            c.J[kDof + i] = p.Jn_right[i];
        }
        if (c.J.squaredNorm() < 1e-18) continue;  // pair not actuated by this command
        const double margin = d_now - d_hard;
        c.xi = margin > 0.0 ? std::sqrt(2.0 * a_brake * margin)
                            : -recover;  // below floor: block (or push out)
        c.d_now = d_now;
        out.push_back(std::move(c));
    }
}

CollisionProjectionResult solveVelocityProjectionForArm(
    std::vector<VelocityConstraint>& cons,
    ArmId arm,
    const JointArray& left_prev_deg, const JointArray& right_prev_deg,
    JointArray& left_target_deg, JointArray& right_target_deg,
    double dt_sec, int iterations, const JointArray& max_joint_vel_deg_s) {
    // Solve the full coupled system, then discard the peer's half. Both arms run
    // this and keep opposite halves, which reconstructs the coupled solution
    // without either arm double-correcting for the other.
    JointArray left_solved = left_target_deg;
    JointArray right_solved = right_target_deg;
    const CollisionProjectionResult out = solveVelocityProjection(
        cons, left_prev_deg, right_prev_deg, left_solved, right_solved,
        dt_sec, iterations, max_joint_vel_deg_s);
    if (arm == ArmId::Left) {
        left_target_deg = left_solved;
    } else {
        right_target_deg = right_solved;
    }
    return out;
}

CollisionProjectionResult solveVelocityProjection(
    std::vector<VelocityConstraint>& cons,
    const JointArray& left_prev_deg, const JointArray& right_prev_deg,
    JointArray& left_target_deg, JointArray& right_target_deg,
    double dt_sec, int iterations, const JointArray& max_joint_vel_deg_s) {
    CollisionProjectionResult out;
    if (dt_sec <= 1e-6 || cons.empty()) return out;

    constexpr int N = 2 * kDof;
    using Vec = Eigen::Matrix<double, N, 1>;
    Vec qprev, qtgt;
    for (int i = 0; i < kDof; ++i) {
        qprev[i] = left_prev_deg[i] * kDeg2Rad;
        qprev[kDof + i] = right_prev_deg[i] * kDeg2Rad;
        qtgt[i] = left_target_deg[i] * kDeg2Rad;
        qtgt[kDof + i] = right_target_deg[i] * kDeg2Rad;
    }
    const Vec qdot_des = (qtgt - qprev) / dt_sec;  // rad/s (J is per-rad)
    Vec qdot = qdot_des;

    // Gauss-Seidel, closest constraint first: enforce ddot = J.qdot >= -xi.
    std::sort(cons.begin(), cons.end(),
              [](const VelocityConstraint& x, const VelocityConstraint& y) {
                  return x.d_now < y.d_now;
              });
    const int M = std::max(1, iterations);
    for (int s = 0; s < M; ++s) {
        for (const auto& c : cons) {
            const double ddot = c.J.dot(qdot);
            if (ddot < -c.xi) {
                const double alpha = (-c.xi - ddot) / std::max(c.J.squaredNorm(), 1e-9);
                qdot += alpha * c.J;  // alpha > 0: cancels only the excess closing
            }
        }
    }

    // Per-joint velocity clamp (approximate; the exact joint-limit-aware solve is
    // Stage 4 QP). Same ceiling for both arms.
    for (int i = 0; i < kDof; ++i) {
        const double lim = max_joint_vel_deg_s[i] * kDeg2Rad;
        if (lim > 0.0) {
            const double before_left = qdot[i];
            const double before_right = qdot[kDof + i];
            qdot[i] = std::clamp(qdot[i], -lim, lim);
            qdot[kDof + i] = std::clamp(qdot[kDof + i], -lim, lim);
            if (qdot[i] != before_left || qdot[kDof + i] != before_right) {
                out.ceiling_clamped = true;
            }
        }
    }

    const Vec dq = qdot - qdot_des;
    const Vec qnew = qprev + qdot * dt_sec;
    for (int i = 0; i < kDof; ++i) {
        left_target_deg[i] = qnew[i] / kDeg2Rad;
        right_target_deg[i] = qnew[kDof + i] / kDeg2Rad;
    }
    const double rad2deg = 1.0 / kDeg2Rad;
    out.active = true;
    out.active_pairs = static_cast<int>(cons.size());
    out.left_correction_deg_s = dq.head(kDof).norm() * rad2deg;
    out.right_correction_deg_s = dq.tail(kDof).norm() * rad2deg;
    out.max_correction_deg_s = dq.norm() * rad2deg;
    return out;
}

CollisionProjectionResult applyCollisionVelocityProjection(
    const CollisionVerdict& v, const CollisionMonitorConfig& cfg,
    const JointArray& left_prev_deg, const JointArray& right_prev_deg,
    JointArray& left_target_deg, JointArray& right_target_deg,
    double dt_sec, double verdict_age_s, const JointArray& max_joint_vel_deg_s) {
    if (!v.valid || dt_sec <= 1e-6 || v.near.empty()) return CollisionProjectionResult{};
    std::vector<VelocityConstraint> cons;
    cons.reserve(v.near.size());
    buildCollisionConstraints(v, cfg, verdict_age_s, cons);
    return solveVelocityProjection(cons, left_prev_deg, right_prev_deg, left_target_deg,
                                   right_target_deg, dt_sec, cfg.projection_iterations,
                                   max_joint_vel_deg_s);
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
    // velocity-space (idx_v) columns for the actuated joints — the Jacobian column
    // index for each command DOF (== idx_q for a fixed-base revolute chain, but
    // idx_v is the correct mapping). Used to slice J_n into Jn_left/Jn_right.
    std::array<int, kDof> left_vidx{};
    std::array<int, kDof> right_vidx{};
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
    // Tracked separately so per-arm submission only arms the evaluator once BOTH
    // arms have reported; a default-constructed counterpart is not a real pose.
    bool has_left = false;
    bool has_right = false;

    // runtime-controlled "ground_plane" whole-arm floor (ground_plane.follow_safety_floors).
    // gp_index_ is the geometry index of the injected box (SIZE_MAX if absent);
    // gp_thickness_ is its box thickness (local z extent). The desired pose is set by
    // setGroundPlanePose() under in_mtx and applied on the monitor thread before each
    // eval. gp_controlled_ stays false (static config placement) until the first call.
    std::size_t gp_index_ = std::numeric_limits<std::size_t>::max();
    double gp_thickness_ = 0.0;
    // Runtime-controlled external boxes are preallocated once in the fixed
    // GeometryModel and parked when disabled/stale. The caller only updates poses;
    // all GeometryObject placement writes happen on the monitor thread.
    std::vector<std::size_t> external_box_indices_;
    // per-collision-pair external flag (1 = arm<->external obstacle, e.g. ground_plane).
    std::vector<char> pair_external_;
    // per-collision-pair runtime external-box flag, distinct from ground_plane external.
    std::vector<char> pair_external_box_;
    // per-collision-pair intra-arm flag (1 = same-arm non-adjacent link pair).
    std::vector<char> pair_intra_;
    std::vector<std::string> pair_category_;
    // per-collision-pair arm membership: does the pair touch the left / right arm
    // (parallel to pair_external_/pair_category_). A pair touches an arm if either of
    // its two bodies belongs to that arm. Used by the active-arm-masked queries so the
    // planner can ignore pairs that involve only the stationary non-active arm.
    std::vector<char> pair_left_;
    std::vector<char> pair_right_;

    // True if the pair at index k should be considered for the given active-arm set.
    // Both-included is the unmasked fast path; otherwise keep only pairs that touch an
    // included arm.
    bool pairActive(std::size_t k, bool include_left, bool include_right) const {
        if (include_left && include_right) return true;
        const bool touches_left = k < pair_left_.size() && pair_left_[k];
        const bool touches_right = k < pair_right_.size() && pair_right_[k];
        return (include_left && touches_left) || (include_right && touches_right);
    }
    // ARTICULATED gripper: per-arm movable finger hull geometry + the (open) base
    // placement, repositioned along local +X by the live jaw percent. gripper_[0]=Left,
    // [1]=Right. gripper_pct_ is guarded by in_mtx; defaults OPEN (largest envelope) so
    // an oracle that never calls setGripperOpenPercent stays conservative.
    struct GripperFingers {
        std::size_t left_idx = std::numeric_limits<std::size_t>::max();
        std::size_t right_idx = std::numeric_limits<std::size_t>::max();
        pinocchio::SE3 attach_placement = pinocchio::SE3::Identity();
        bool present = false;
    };
    GripperFingers gripper_[2];
    double gripper_pct_[2] = {100.0, 100.0};  // guarded by in_mtx
    bool articulated_ = false;
    // Collision meshes that were NOT convex and were kept as (correct but slow) BVH
    // instead of a convex hull. >0 means the geometry is not the fast convex path:
    // distances are still correct, but eval is much slower (BVH-vs-BVH), so the per-eval
    // performance contract (servo reaction budget) only holds when this is 0. Set in
    // buildGeometry; surfaced via CollisionMonitor::numNonConvexMeshes() for tests/ops.
    std::size_t non_convex_mesh_count_ = 0;
    // stand frame pose in the model's universe frame (constant; the stand is fixed to
    // the root). setGroundPlanePose() takes the floor plane in STAND frame (matching
    // the servo loop's floor_constraint / user_floor), so we map it through this.
    pinocchio::SE3 o_M_stand_ = pinocchio::SE3::Identity();
    bool gp_controlled_ = false;   // guarded by in_mtx
    bool gp_enabled_ = false;      // guarded by in_mtx
    Eigen::Vector3d gp_point_ = Eigen::Vector3d::Zero();    // guarded by in_mtx
    Eigen::Vector3d gp_normal_ = Eigen::Vector3d::UnitZ();  // guarded by in_mtx
    bool external_boxes_controlled_ = false;                // guarded by in_mtx
    std::vector<ExternalBoxPose> external_box_poses_;       // guarded by in_mtx
    double external_boxes_stamp_s_ = 0.0;                   // guarded by in_mtx

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
        // Locate the injected whole-arm floor box so its pose can be tracked at runtime
        // (ground_plane.follow_safety_floors). Thickness comes from its extra_collision
        // entry (size_m[2]); needed to place the TOP face at the requested plane.
        for (std::size_t i = 0; i < geom.geometryObjects.size(); ++i) {
            if (geom.geometryObjects[i].name == "ground_plane") { gp_index_ = i; break; }
        }
        for (const auto& e : cfg.extra_collision) {
            if (e.name == "ground_plane") { gp_thickness_ = e.size_m[2]; break; }
        }
        // curatePairs() tags external-obstacle pairs as it adds them. Keep this
        // fail-closed if a future curation path forgets to mirror the pair vector.
        if (pair_external_.size() != geom.collisionPairs.size()) {
            pair_external_.assign(geom.collisionPairs.size(), 0);
        }
        if (pair_external_box_.size() != geom.collisionPairs.size()) {
            pair_external_box_.assign(geom.collisionPairs.size(), 0);
        }
        if (pair_intra_.size() != geom.collisionPairs.size()) {
            pair_intra_.assign(geom.collisionPairs.size(), 0);
        }
        if (pair_category_.size() != geom.collisionPairs.size()) {
            pair_category_.assign(geom.collisionPairs.size(), "self");
        }
        // Constant stand-frame pose in the universe frame (FK once at neutral; the
        // stand is rigidly fixed to the root so this never changes).
        if (model.existFrame(cfg.stand_frame)) {
            pinocchio::forwardKinematics(model, data, pinocchio::neutral(model));
            pinocchio::updateFramePlacements(model, data);
            o_M_stand_ = data.oMf[model.getFrameId(cfg.stand_frame)];
        }
        // start with a neutral verdict (invalid -> fail closed until first eval)
        std::atomic_store(&published, std::make_shared<const CollisionVerdict>());
    }

    pinocchio::SE3 disabledObstaclePlacement() const {
        return pinocchio::SE3(Eigen::Matrix3d::Identity(),
                              Eigen::Vector3d(0.0, 0.0, -1000.0));
    }

    // Geometry placement (in the universe frame, == geometryObjects[].placement since
    // the box is universe-attached) that puts the box TOP face on the STAND-frame plane
    // {point, normal}, with `normal` as the box local +z (so a tilted plane tilts the
    // box). The box center is half a thickness below the top face along the normal.
    // `enabled=false` parks it far below so no collision pair is ever near it (inert).
    pinocchio::SE3 groundPlanePlacement(bool enabled, const Eigen::Vector3d& point,
                                        const Eigen::Vector3d& normal) const {
        if (!enabled) {
            return disabledObstaclePlacement();
        }
        Eigen::Vector3d n = normal;
        const double nn = n.norm();
        n = (nn > 1e-9) ? (n / nn).eval() : Eigen::Vector3d::UnitZ();
        // Rotation mapping local +z onto n (Eigen builds a valid basis for any target).
        const Eigen::Matrix3d rot =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), n).toRotationMatrix();
        const Eigen::Vector3d center = point - n * (gp_thickness_ * 0.5);
        // {point, normal} are in STAND frame -> map the whole placement to universe.
        return o_M_stand_ * pinocchio::SE3(rot, center);
    }

    // Runtime external boxes use center pose {R,t} in the same STAND frame accepted by
    // setGroundPlanePose(). R is guaranteed orthonormal by the caller and is not
    // reconditioned here.
    pinocchio::SE3 externalBoxPlacement(bool enabled, const ExternalBoxPose& pose) const {
        if (!enabled) return disabledObstaclePlacement();
        return o_M_stand_ * pinocchio::SE3(pose.R, pose.t);
    }

    // Apply the latest requested ground-plane pose to the geometry model (monitor
    // thread only, just before computeDistances). No-op until setGroundPlanePose ran.
    void applyGroundPlanePose() {
        if (gp_index_ == std::numeric_limits<std::size_t>::max()) return;
        bool controlled, enabled;
        Eigen::Vector3d point, normal;
        {
            std::lock_guard<std::mutex> lk(in_mtx);
            controlled = gp_controlled_;
            enabled = gp_enabled_;
            point = gp_point_;
            normal = gp_normal_;
        }
        if (!controlled) return;
        geom.geometryObjects[gp_index_].placement = groundPlanePlacement(enabled, point, normal);
    }

    // Apply the latest external-box poses (monitor thread only, just before distance
    // computation). Geometry is fixed; only placement changes. Stale "hold" keeps the
    // last applied placements, while stale "disable" parks every preallocated slot.
    void applyExternalBoxes() {
        if (external_box_indices_.empty()) return;
        bool controlled;
        double stamp;
        std::vector<ExternalBoxPose> poses;
        {
            std::lock_guard<std::mutex> lk(in_mtx);
            controlled = external_boxes_controlled_;
            stamp = external_boxes_stamp_s_;
            poses = external_box_poses_;
        }
        if (!controlled) return;
        const double now = nowMonotonicS();
        if (now - stamp > cfg.external_boxes.stale_timeout_s) {
            if (cfg.external_boxes.stale_policy == "disable") {
                for (const std::size_t idx : external_box_indices_) {
                    geom.geometryObjects[idx].placement = disabledObstaclePlacement();
                }
            }
            return;
        }
        for (std::size_t k = 0; k < external_box_indices_.size(); ++k) {
            const bool enabled = k < poses.size() && poses[k].enable;
            const ExternalBoxPose pose = enabled ? poses[k] : ExternalBoxPose{};
            geom.geometryObjects[external_box_indices_[k]].placement =
                externalBoxPlacement(enabled, pose);
        }
    }

    // Reposition each arm's two finger hulls along the local jaw axis (+X) for the live
    // open percent (monitor thread only, before computeDistances). finger_pos goes 0 at
    // OPEN (percent 100, the STL pose) to travel at CLOSED (percent 0); left finger +X,
    // right finger -X, mirroring rb3_730e_pika_articulated.urdf + the GUI articulation.
    void applyGripperFingers() {
        if (!articulated_) return;
        double pct[2];
        {
            std::lock_guard<std::mutex> lk(in_mtx);
            pct[0] = gripper_pct_[0];
            pct[1] = gripper_pct_[1];
        }
        for (int a = 0; a < 2; ++a) {
            const GripperFingers& g = gripper_[a];
            if (!g.present) continue;
            const double t = std::clamp(pct[a], 0.0, 100.0) / 100.0;
            const double finger_pos = (1.0 - t) * cfg.gripper_finger_travel_m;
            const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            geom.geometryObjects[g.left_idx].placement =
                g.attach_placement * pinocchio::SE3(I, Eigen::Vector3d(finger_pos, 0.0, 0.0));
            geom.geometryObjects[g.right_idx].placement =
                g.attach_placement * pinocchio::SE3(I, Eigen::Vector3d(-finger_pos, 0.0, 0.0));
        }
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
            left_vidx[i] = model.joints[lj].idx_v();
            right_vidx[i] = model.joints[rj].idx_v();
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
        // Convexify mesh (BVH) geometry for fast GJK; primitives untouched. ONLY when the
        // mesh is actually convex — buildConvexRepresentation does not hull a non-convex
        // mesh, it silently produces an invalid Convex with wrong GJK distances (see
        // bvhMeshIsConvex). A non-convex mesh keeps its BVH (correct distance, slower) and
        // is reported below so the broken-convex failure mode can never recur silently.
        std::vector<std::string> non_convex_kept_bvh;
        for (auto& go : geom.geometryObjects) {
            auto bvh = std::dynamic_pointer_cast<coal::BVHModelBase>(go.geometry);
            if (bvh) {
                if (bvhMeshIsConvex(*bvh, kConvexTolM)) {
                    bvh->buildConvexRepresentation(false);
                    if (bvh->convex) go.geometry = bvh->convex;
                } else {
                    non_convex_kept_bvh.push_back(go.name);
                }
            }
        }
        if (!non_convex_kept_bvh.empty()) {
            std::cerr << "[collision_monitor] WARNING: " << non_convex_kept_bvh.size()
                      << " URDF collision mesh(es) are NOT convex; kept as BVH (correct but"
                         " slower). Provide precomputed convex-hull meshes for: ";
            for (std::size_t i = 0; i < non_convex_kept_bvh.size(); ++i)
                std::cerr << (i ? ", " : "") << non_convex_kept_bvh[i];
            std::cerr << std::endl;
        }
        non_convex_mesh_count_ = non_convex_kept_bvh.size();
        // Attach the Pika gripper to each arm's attachment_site frame. Two modes:
        //  (1) ARTICULATED (base + both finger meshes set): a static base hull plus two
        //      movable finger hulls placed at identity in the attachment_site frame
        //      (mirrors rb3_730e_pika_articulated.urdf — no +90° Z). The finger hulls
        //      are repositioned along local +X by setGripperOpenPercent so the checked
        //      jaw tracks the live gripper. Default placement = OPEN (finger_pos 0).
        //  (2) SINGLE HULL (pika_gripper_mesh only): the legacy static convex hull at
        //      attachment_site with a +90° Z rotation (the combined-hull STL frame).
        // Load a gripper mesh as a convex shape for fast GJK — but ONLY if it is actually
        // convex. A non-convex STL would become an invalid Convex (wrong distances, which
        // silently disabled the gripper guard); fail-closed to the raw BVH (correct,
        // slower) and warn loudly so the operator supplies a *_hull.STL. Return the common
        // CollisionGeometry base so either shape works downstream (placement-only updates).
        auto loadConvex = [this](coal::MeshLoader& ld,
                                 const std::string& path) -> std::shared_ptr<coal::CollisionGeometry> {
            auto bvh = ld.load(path, coal::Vec3s(0.001, 0.001, 0.001));
            if (bvhMeshIsConvex(*bvh, kConvexTolM)) {
                bvh->buildConvexRepresentation(false);
                if (bvh->convex) return bvh->convex;
            }
            ++non_convex_mesh_count_;
            std::cerr << "[collision_monitor] WARNING: gripper mesh '" << path
                      << "' is NOT convex; coal cannot hull it here (no qhull) -> keeping the"
                         " raw BVH mesh (correct but slower). Provide a precomputed convex-hull"
                         " STL (*_hull.STL) for fast, reliable gripper self-collision.\n";
            return bvh;
        };
        const bool articulated = !cfg.pika_gripper_base_mesh.empty() &&
                                 !cfg.pika_finger_left_mesh.empty() &&
                                 !cfg.pika_finger_right_mesh.empty();
        if (articulated) {
            coal::MeshLoader loader;
            auto base_hull = loadConvex(loader, cfg.pika_gripper_base_mesh);
            auto fl_hull = loadConvex(loader, cfg.pika_finger_left_mesh);
            auto fr_hull = loadConvex(loader, cfg.pika_finger_right_mesh);
            const std::array<std::pair<std::string, ArmId>, 2> arms = {{
                {cfg.left_prefix, ArmId::Left}, {cfg.right_prefix, ArmId::Right}}};
            for (const auto& [prefix, aid] : arms) {
                const auto fid = model.getFrameId(prefix + "attachment_site");
                const auto& fr = model.frames[fid];
                const pinocchio::SE3 place = fr.placement;  // identity local (URDF mirror)
                geom.addGeometryObject(pinocchio::GeometryObject(
                    prefix + "pika_gripper_base", fr.parentJoint, fr.parentFrame, place, base_hull));
                geom.addGeometryObject(pinocchio::GeometryObject(
                    prefix + "pika_finger_left", fr.parentJoint, fr.parentFrame, place, fl_hull));
                geom.addGeometryObject(pinocchio::GeometryObject(
                    prefix + "pika_finger_right", fr.parentJoint, fr.parentFrame, place, fr_hull));
                GripperFingers& g = gripper_[aid == ArmId::Left ? 0 : 1];
                g.left_idx = geom.getGeometryId(prefix + "pika_finger_left");
                g.right_idx = geom.getGeometryId(prefix + "pika_finger_right");
                g.attach_placement = place;
                g.present = true;
            }
            articulated_ = true;
        } else if (!cfg.pika_gripper_mesh.empty()) {
            coal::MeshLoader loader;
            auto hull = loadConvex(loader, cfg.pika_gripper_mesh);
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
        if (cfg.external_boxes.enable) {
            if (cfg.external_boxes.max_count < 0) {
                throw std::runtime_error("collision_monitor: external_boxes.max_count must be >= 0");
            }
            if (!model.existFrame(cfg.stand_frame)) {
                throw std::runtime_error("collision_monitor: external_boxes stand_frame not found: " +
                                         cfg.stand_frame);
            }
            const auto fid = model.getFrameId(cfg.stand_frame);
            const auto& fr = model.frames[fid];
            std::array<double, 3> dims{};
            for (int i = 0; i < 3; ++i) {
                dims[i] = cfg.external_boxes.size_m[i] + 2.0 * cfg.external_boxes.margin_m[i];
                if (!std::isfinite(dims[i]) || dims[i] <= 0.0) {
                    throw std::runtime_error(
                        "collision_monitor: external_boxes inflated size must be finite and > 0");
                }
            }
            external_box_indices_.reserve(static_cast<std::size_t>(cfg.external_boxes.max_count));
            for (int k = 0; k < cfg.external_boxes.max_count; ++k) {
                const std::string name = "external_box_" + std::to_string(k);
                auto shape = std::make_shared<coal::Box>(dims[0], dims[1], dims[2]);
                geom.addGeometryObject(pinocchio::GeometryObject(
                    name, fr.parentJoint, fid, fr.placement * disabledObstaclePlacement(), shape));
                external_box_indices_.push_back(geom.getGeometryId(name));
            }
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
        enum class PairCategory { LeftRight, ArmStand, IntraArm, External, ExternalBox };
        const auto sideString = [](Side side) {
            switch (side) {
                case Side::Left: return "left";
                case Side::Right: return "right";
                case Side::Stand: return "stand";
            }
            return "unknown";
        };
        const auto categoryString = [](PairCategory c) {
            switch (c) {
                case PairCategory::LeftRight: return "left-right";
                case PairCategory::ArmStand: return "arm-stand";
                case PairCategory::IntraArm: return "intra-arm";
                case PairCategory::External: return "external";
                case PairCategory::ExternalBox: return "external-box";
            }
            return "unknown";
        };
        const auto isExternalGeometry = [&](std::size_t i) {
            // External barrier category is currently reserved for the injected
            // whole-arm floor. Other stand geometry remains arm-stand and keeps
            // the self-collision threshold set.
            return geom.geometryObjects[i].name == "ground_plane";
        };
        const auto isExternalBoxGeometry = [&](std::size_t i) {
            return nameStartsWith(geom.geometryObjects[i].name, "external_box_");
        };
        const auto parentFrameName = [&](std::size_t i) -> std::string {
            const auto fid = geom.geometryObjects[i].parentFrame;
            if (fid < model.frames.size()) return model.frames[fid].name;
            return "<invalid>";
        };
        const auto parentJointName = [&](std::size_t i) -> std::string {
            const auto jid = geom.geometryObjects[i].parentJoint;
            if (jid < model.names.size()) return model.names[jid];
            return "<invalid>";
        };
        const auto disabledRule = [&](std::size_t a, std::size_t b)
            -> const CollisionPairPattern* {
            const std::string& an = geom.geometryObjects[a].name;
            const std::string& bn = geom.geometryObjects[b].name;
            for (const auto& rule : cfg.disabled_collision_pairs) {
                if (collisionPairPatternMatches(rule, an, bn)) return &rule;
            }
            return nullptr;
        };

        std::vector<std::size_t> li, ri, si;
        for (std::size_t i = 0; i < geom.geometryObjects.size(); ++i) {
            switch (classify(i)) {
                case Side::Left: li.push_back(i); break;
                case Side::Right: ri.push_back(i); break;
                case Side::Stand: si.push_back(i); break;
            }
        }
        if (cfg.debug_pair_curation) {
            std::cerr << "[collision_monitor] pair_curation geometry_classification\n";
            for (std::size_t i = 0; i < geom.geometryObjects.size(); ++i) {
                const Side side = classify(i);
                std::cerr << "[collision_monitor] geom"
                          << " id=" << i
                          << " name=" << geom.geometryObjects[i].name
                          << " parentFrame=" << parentFrameName(i)
                          << " parentJoint=" << parentJointName(i)
                          << " side=" << sideString(side)
                          << " chain_depth=";
                if (side == Side::Left || side == Side::Right) {
                    std::cerr << chainDepth(i, side);
                } else {
                    std::cerr << "-";
                }
                std::cerr << "\n";
            }
        }
        geom.removeAllCollisionPairs();
        pair_external_.clear();
        pair_external_box_.clear();
        pair_intra_.clear();
        pair_category_.clear();
        pair_left_.clear();
        pair_right_.clear();
        std::size_t n_lr = 0, n_arm_stand = 0, n_intra = 0, n_external = 0;
        std::size_t n_external_box = 0, n_disabled = 0;
        const auto tryAddPair = [&](std::size_t a, std::size_t b, PairCategory category) {
            if (const CollisionPairPattern* rule = disabledRule(a, b)) {
                ++n_disabled;
                if (cfg.debug_pair_curation) {
                    std::cerr << "[collision_monitor] disabled_pair"
                              << " rule=" << ruleString(*rule)
                              << " geom_a=" << geom.geometryObjects[a].name
                              << " geom_b=" << geom.geometryObjects[b].name
                              << " category=" << categoryString(category)
                              << " reason=disabled_collision_pairs\n";
                }
                return;
            }
            geom.addCollisionPair(pinocchio::CollisionPair(a, b));
            pair_external_.push_back(category == PairCategory::External ? 1 : 0);
            pair_external_box_.push_back(category == PairCategory::ExternalBox ? 1 : 0);
            pair_intra_.push_back(category == PairCategory::IntraArm ? 1 : 0);
            pair_category_.push_back(categoryString(category));
            const Side sa = classify(a);
            const Side sb = classify(b);
            pair_left_.push_back((sa == Side::Left || sb == Side::Left) ? 1 : 0);
            pair_right_.push_back((sa == Side::Right || sb == Side::Right) ? 1 : 0);
            switch (category) {
                case PairCategory::LeftRight: ++n_lr; break;
                case PairCategory::ArmStand: ++n_arm_stand; break;
                case PairCategory::IntraArm: ++n_intra; break;
                case PairCategory::External: ++n_external; break;
                case PairCategory::ExternalBox: ++n_external_box; break;
            }
        };
        // left <-> right (whole arms)
        for (auto a : li)
            for (auto b : ri) tryAddPair(a, b, PairCategory::LeftRight);
        // arm <-> stand (skip the bolted-on mount neighbors)
        for (const auto& arm : {li, ri}) {
            for (auto a : arm) {
                for (auto b : si) {
                    const bool box = isExternalBoxGeometry(b);
                    if (!box &&
                        nameContainsAny(geom.geometryObjects[a].name,
                                        cfg.stand_ignore_arm_substrings)) {
                        continue;
                    }
                    const PairCategory category = isExternalGeometry(b)
                        ? PairCategory::External
                        : (box ? PairCategory::ExternalBox : PairCategory::ArmStand);
                    tryAddPair(a, b, category);
                }
            }
        }
        // intra-arm (arm folding onto itself): only NON-adjacent links of the same arm
        if (cfg.check_intra_arm) {
            const int sep = cfg.intra_arm_min_chain_separation;
            auto intra = [&](const std::vector<std::size_t>& g, Side sd) {
                for (std::size_t x = 0; x < g.size(); ++x)
                    for (std::size_t y = x + 1; y < g.size(); ++y) {
                        if (std::abs(chainDepth(g[x], sd) - chainDepth(g[y], sd)) < sep) continue;
                        tryAddPair(g[x], g[y], PairCategory::IntraArm);
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
                  << " intra-arm=" << n_intra << " external=" << n_external
                  << " external-box=" << n_external_box
                  << " disabled=" << n_disabled << "]"
                  << " disabled_collision_pairs=" << cfg.disabled_collision_pairs.size()
                  << " stand_ignore=[";
        for (std::size_t i = 0; i < cfg.stand_ignore_arm_substrings.size(); ++i)
            std::cerr << (i ? "," : "") << cfg.stand_ignore_arm_substrings[i];
        std::cerr << "] swept_samples=" << cfg.swept_samples
                  << (articulated_ ? " gripper=articulated(base+2fingers)"
                                   : (cfg.pika_gripper_mesh.empty() ? " gripper=NONE"
                                                                    : " gripper=hull"))
                  << std::endl;
    }

    void setQ(const JointArray& l, const JointArray& r) {
        for (int i = 0; i < kDof; ++i) {
            q[left_qidx[i]] = l[i] * kDeg2Rad;
            q[right_qidx[i]] = r[i] * kDeg2Rad;
        }
    }

    CollisionVerdict reduce(double stamp, const Eigen::VectorXd& q_eval) {
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
        // Per-category minima + per-pair hard_violation: ground_plane and enforced
        // external-box pairs use external_d_hard_m, intra-arm pairs use
        // intra_arm_d_hard_m, and arm<->arm / arm<->stand pairs keep d_hard_m.
        // Box clearance is reported separately so ground_plane telemetry remains
        // unchanged.
        double self_min = std::numeric_limits<double>::infinity();
        double intra_min = std::numeric_limits<double>::infinity();
        double ext_min = std::numeric_limits<double>::infinity();
        double ext_box_min = std::numeric_limits<double>::infinity();
        std::vector<double> ext_box_clearance(
            external_box_indices_.size(), std::numeric_limits<double>::infinity());
        bool hard = false;
        const bool enforce_external_boxes = !cfg.external_boxes.monitor_only;
        for (std::size_t k = 0; k < np; ++k) {
            const double d = gdata.distanceResults[k].min_distance;
            cur[k] = d;
            const bool ext = k < pair_external_.size() && pair_external_[k];
            const bool ext_box = k < pair_external_box_.size() && pair_external_box_[k];
            const bool intra = k < pair_intra_.size() && pair_intra_[k];
            if (ext_box) {
                if (d < ext_box_min) ext_box_min = d;
                const auto& cp = geom.collisionPairs[k];
                auto it = std::find(external_box_indices_.begin(),
                                    external_box_indices_.end(),
                                    static_cast<std::size_t>(cp.first));
                if (it == external_box_indices_.end()) {
                    it = std::find(external_box_indices_.begin(),
                                   external_box_indices_.end(),
                                   static_cast<std::size_t>(cp.second));
                }
                if (it != external_box_indices_.end()) {
                    const std::size_t slot = static_cast<std::size_t>(
                        std::distance(external_box_indices_.begin(), it));
                    if (d < ext_box_clearance[slot]) ext_box_clearance[slot] = d;
                }
            }
            if (ext_box && !enforce_external_boxes) {
                continue;
            }
            ds.emplace_back(d, k);
            if (d < dmin) { dmin = d; kmin = k; }
            if (ext) {
                if (d < ext_min) ext_min = d;
                if (d < cfg.external_d_hard_m) hard = true;
            } else if (ext_box) {
                if (d < cfg.external_box_d_hard_m) hard = true;
            } else if (intra) {
                if (d < intra_min) intra_min = d;
                if (d < cfg.intra_arm_d_hard_m) hard = true;
            } else {
                if (d < self_min) self_min = d;
                if (d < cfg.d_hard_m) hard = true;
            }
        }
        v.min_clearance_m = dmin;
        v.self_min_clearance_m = self_min;
        v.intra_arm_min_clearance_m = intra_min;
        v.external_min_clearance_m = ext_min;
        v.external_box_min_clearance_m = ext_box_min;
        v.external_box_clearance_m = std::move(ext_box_clearance);
        v.hard_violation = hard;
        const std::size_t K = std::min<std::size_t>(cfg.max_near_pairs, ds.size());
        std::partial_sort(ds.begin(), ds.begin() + K, ds.end(),
                          [](const auto& a, const auto& b) { return a.first < b.first; });

        // Per-pair clearance Jacobian (Stage 1). computeDistances leaves data.oMi at
        // q_eval but does NOT compute joint Jacobians; do it once here for the K near
        // pairs. The witness-point translational Jacobian is the joint LWA Jacobian
        // shifted to the witness point: J_p = J_lin - skew(p - o) * J_ang.
        pinocchio::computeJointJacobians(model, data, q_eval);
        const int nv = model.nv;
        Eigen::Matrix<double, 6, Eigen::Dynamic> J6(6, nv);
        const auto jacobianOfPoint =
            [&](pinocchio::JointIndex jid, const Eigen::Vector3d& p) -> Eigen::MatrixXd {
            Eigen::MatrixXd Jp = Eigen::MatrixXd::Zero(3, nv);
            if (jid == 0) return Jp;  // universe (static geometry): no motion
            J6.setZero();
            pinocchio::getJointJacobian(model, data, jid, pinocchio::LOCAL_WORLD_ALIGNED, J6);
            const Eigen::Vector3d r = p - data.oMi[jid].translation();
            Eigen::Matrix3d S;
            S << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
            Jp = J6.topRows<3>() - S * J6.bottomRows<3>();
            return Jp;
        };

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
            p.external = k < pair_external_.size() && pair_external_[k];
            p.external_box = k < pair_external_box_.size() && pair_external_box_[k];
            p.intra_arm = k < pair_intra_.size() && pair_intra_[k];
            // d_dot = n^T (v_b - v_a) = J_n * qdot. Slice into command left/right cols.
            const pinocchio::JointIndex ja = geom.geometryObjects[cp.first].parentJoint;
            const pinocchio::JointIndex jb = geom.geometryObjects[cp.second].parentJoint;
            const Eigen::MatrixXd Jrel = jacobianOfPoint(jb, p.p_b) - jacobianOfPoint(ja, p.p_a);
            const Eigen::RowVectorXd Jn = p.n.transpose() * Jrel;  // 1 x nv
            for (int c = 0; c < kDof; ++c) {
                p.Jn_left[c] = Jn(left_vidx[c]);
                p.Jn_right[c] = Jn(right_vidx[c]);
            }
            // Per-pair signed clearance rate (+ = separating), tracked by pair index.
            if (prev_pair_clear.size() == np && stamp > prev_stamp) {
                p.rate_m_s = (cur[k] - prev_pair_clear[k]) / (stamp - prev_stamp);
            }
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
        applyGroundPlanePose();  // track the operator's active floor (if controlled)
        applyExternalBoxes();    // track camera-provided keep-out boxes (if controlled)
        applyGripperFingers();   // track the live jaw open percent (articulated gripper)
        const int N = std::max(1, cfg.swept_samples);
        Eigen::VectorXd q_eval = q;  // configuration gdata ends up at (for J_n)
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
            q_eval = prev_eval_q + worst_alpha * (q - prev_eval_q);
        }
        prev_eval_q = q;
        have_prev_eval = true;
        return reduce(nowMonotonicS(), q_eval);
    }

    CollisionDistanceSummary summarizeCurrentDistances(
        bool include_left = true, bool include_right = true) const {
        CollisionDistanceSummary s;
        s.valid = true;
        const std::size_t np = geom.collisionPairs.size();
        for (std::size_t k = 0; k < np; ++k) {
            if (!pairActive(k, include_left, include_right)) continue;
            const double d = gdata.distanceResults[k].min_distance;
            const bool ext = k < pair_external_.size() && pair_external_[k];
            const bool ext_box = k < pair_external_box_.size() && pair_external_box_[k];
            const bool intra = k < pair_intra_.size() && pair_intra_[k];
            if (ext_box && cfg.external_boxes.monitor_only) continue;
            if (d < s.min_clearance_m) {
                s.min_clearance_m = d;
                s.nearest_distance_m = d;
                const auto& cp = geom.collisionPairs[k];
                s.nearest_name_a = geom.geometryObjects[cp.first].name;
                s.nearest_name_b = geom.geometryObjects[cp.second].name;
                s.nearest_external = k < pair_external_.size() && pair_external_[k];
                if (k < pair_category_.size()) {
                    s.nearest_category = pair_category_[k];
                } else {
                    s.nearest_category = s.nearest_external ? "external" : "self";
                }
                s.nearest_intra_arm = intra;
            }
            if (ext) {
                if (d < s.external_min_clearance_m) s.external_min_clearance_m = d;
                if (d < cfg.external_d_hard_m) s.hard_violation = true;
            } else if (ext_box) {
                if (d < cfg.external_box_d_hard_m) s.hard_violation = true;
            } else if (intra) {
                if (d < s.intra_arm_min_clearance_m) s.intra_arm_min_clearance_m = d;
                if (d < cfg.intra_arm_d_hard_m) s.hard_violation = true;
            } else {
                if (d < s.self_min_clearance_m) s.self_min_clearance_m = d;
                if (d < cfg.d_hard_m) s.hard_violation = true;
            }
            if (!std::isfinite(d)) s.hard_violation = true;
        }
        return s;
    }

    CollisionDistanceSummary evalDistancesOnly(const JointArray& l, const JointArray& r,
                                               bool include_left = true,
                                               bool include_right = true) {
        setQ(l, r);
        applyGroundPlanePose();
        applyExternalBoxes();
        applyGripperFingers();
        pinocchio::computeDistances(model, data, geom, gdata, q);
        return summarizeCurrentDistances(include_left, include_right);
    }

    bool clearsThresholds(const JointArray& l, const JointArray& r,
                          double self_thresh_m, double external_thresh_m,
                          double intra_arm_thresh_m,
                          bool include_left = true, bool include_right = true) {
        if (!(std::isfinite(self_thresh_m) && std::isfinite(external_thresh_m) &&
              std::isfinite(intra_arm_thresh_m))) {
            return false;
        }
        setQ(l, r);
        applyGroundPlanePose();
        applyExternalBoxes();
        applyGripperFingers();
        pinocchio::updateGeometryPlacements(model, data, geom, gdata, q);
        const std::size_t np = geom.collisionPairs.size();
        for (std::size_t k = 0; k < np; ++k) {
            const auto& cp = geom.collisionPairs[k];
            if (!gdata.activeCollisionPairs[k] ||
                geom.geometryObjects[cp.first].disableCollision ||
                geom.geometryObjects[cp.second].disableCollision) {
                continue;
            }
            if (!pairActive(k, include_left, include_right)) continue;
            const auto& res = pinocchio::computeDistance(
                geom, gdata, static_cast<pinocchio::PairIndex>(k));
            const double d = res.min_distance;
            if (!std::isfinite(d)) return false;
            const bool ext = k < pair_external_.size() && pair_external_[k];
            const bool ext_box = k < pair_external_box_.size() && pair_external_box_[k];
            const bool intra = k < pair_intra_.size() && pair_intra_[k];
            if (ext_box && cfg.external_boxes.monitor_only) continue;
            const double threshold =
                  ext_box ? std::max(external_thresh_m, cfg.external_box_d_hard_m)
                : ext     ? std::max(external_thresh_m, cfg.external_d_hard_m)
                : intra   ? std::max(intra_arm_thresh_m, cfg.intra_arm_d_hard_m)
                          : std::max(self_thresh_m, cfg.d_hard_m);
            if (d <= threshold) return false;
        }
        return true;
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
    impl_->has_left = true;
    impl_->has_right = true;
    impl_->has_targets = true;
}

void CollisionMonitor::submitArmTarget(ArmId arm, const JointArray& q_deg) {
    // Per-arm submission for per-arm control threads, which never hold both
    // candidates at one instant. The monitor pairs the latest of each -- they can
    // be up to one tick apart, on top of the async evaluation lag this guard
    // already tolerates through its staleness bound.
    //
    // A pair is only evaluated once BOTH arms have submitted: checking one arm
    // against a default-constructed other would test a pose the robot is not in.
    std::lock_guard<std::mutex> lk(impl_->in_mtx);
    if (arm == ArmId::Left) {
        impl_->left_deg = q_deg;
        impl_->has_left = true;
    } else {
        impl_->right_deg = q_deg;
        impl_->has_right = true;
    }
    impl_->has_targets = impl_->has_left && impl_->has_right;
}

CollisionVerdict CollisionMonitor::latest() const { return impl_->load(); }

void CollisionMonitor::setGroundPlanePose(bool enabled, const Eigen::Vector3d& point,
                                          const Eigen::Vector3d& normal) {
    std::lock_guard<std::mutex> lk(impl_->in_mtx);
    impl_->gp_controlled_ = true;
    impl_->gp_enabled_ = enabled;
    impl_->gp_point_ = point;
    impl_->gp_normal_ = normal;
}

void CollisionMonitor::setExternalBoxes(const std::vector<ExternalBoxPose>& boxes,
                                        double stamp_monotonic_s) {
    std::lock_guard<std::mutex> lk(impl_->in_mtx);
    impl_->external_boxes_controlled_ = true;
    impl_->external_box_poses_ = boxes;
    impl_->external_boxes_stamp_s_ = stamp_monotonic_s;
}

bool CollisionMonitor::hasGroundPlane() const {
    return impl_->gp_index_ != std::numeric_limits<std::size_t>::max();
}

void CollisionMonitor::setGripperOpenPercent(ArmId arm, double percent) {
    std::lock_guard<std::mutex> lk(impl_->in_mtx);
    impl_->gripper_pct_[arm == ArmId::Left ? 0 : 1] = percent;
}

bool CollisionMonitor::hasArticulatedGripper() const { return impl_->articulated_; }

CollisionVerdict CollisionMonitor::evalOnce(const JointArray& left_deg, const JointArray& right_deg) {
    CollisionVerdict v = impl_->evalLocked(left_deg, right_deg);
    impl_->publish(v);
    return v;
}

CollisionDistanceSummary CollisionMonitor::evalDistancesOnly(const JointArray& left_deg,
                                                             const JointArray& right_deg) {
    return impl_->evalDistancesOnly(left_deg, right_deg);
}

CollisionDistanceSummary CollisionMonitor::evalDistancesOnly(const JointArray& left_deg,
                                                             const JointArray& right_deg,
                                                             bool include_left, bool include_right) {
    return impl_->evalDistancesOnly(left_deg, right_deg, include_left, include_right);
}

bool CollisionMonitor::clearsThresholds(const JointArray& left_deg, const JointArray& right_deg,
                                        double self_thresh_m, double external_thresh_m) {
    return impl_->clearsThresholds(left_deg, right_deg, self_thresh_m, external_thresh_m,
                                   self_thresh_m);
}

bool CollisionMonitor::clearsThresholds(const JointArray& left_deg, const JointArray& right_deg,
                                        double self_thresh_m, double external_thresh_m,
                                        double intra_arm_thresh_m) {
    return impl_->clearsThresholds(left_deg, right_deg, self_thresh_m, external_thresh_m,
                                   intra_arm_thresh_m);
}

bool CollisionMonitor::clearsThresholds(const JointArray& left_deg, const JointArray& right_deg,
                                        double self_thresh_m, double external_thresh_m,
                                        bool include_left, bool include_right) {
    return impl_->clearsThresholds(left_deg, right_deg, self_thresh_m, external_thresh_m,
                                   self_thresh_m, include_left, include_right);
}

bool CollisionMonitor::clearsThresholds(const JointArray& left_deg, const JointArray& right_deg,
                                        double self_thresh_m, double external_thresh_m,
                                        double intra_arm_thresh_m,
                                        bool include_left, bool include_right) {
    return impl_->clearsThresholds(left_deg, right_deg, self_thresh_m, external_thresh_m,
                                   intra_arm_thresh_m, include_left, include_right);
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
std::size_t CollisionMonitor::numNonConvexMeshes() const { return impl_->non_convex_mesh_count_; }

}  // namespace rb_servo
