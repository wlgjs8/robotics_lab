#pragma once

// Async dual-arm + stand self-collision monitor (URDF mesh/box geometry via
// pinocchio + coal). Runs in its OWN thread so the 500 Hz / 2 ms servo_j loop
// carries zero collision compute: servo_j only reads the latest verdict (atomic,
// ns) and applies a cheap velocity barrier.
//
// Rationale and measured timing: see
//   llm-wiki/projects/robotics-lab-async-self-collision.md
//   robotics_lab/scripts/bench_self_collision.cpp  (mesh worst ~1.5 ms in-loop)
//
// Design note: ALL barrier parameters are a SINGLE shared set, common to every
// motion primitive. Speed adaptation comes from the MEASURED closing speed of the
// geometry, not from per-primitive config, so nothing needs to be retuned per
// primitive.

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "rb_servo/core/types.hpp"

namespace rb_servo {

// Extra collision primitive (geometry the URDF lacks: wrist camera, cable, table).
// Attached to a named frame — an arm frame moves with the arm (auto left/right by
// ancestry); "stand"/"world" is a static obstacle paired against both arms.
struct ExtraCollisionShape {
    std::string name;
    std::string shape = "box";   // box | sphere | capsule | cylinder
    std::string parent_frame;
    std::array<double, 3> size_m{0.0, 0.0, 0.0};  // box: full extents
    double radius_m = 0.0;
    double length_m = 0.0;
    std::array<double, 3> xyz_m{0.0, 0.0, 0.0};
    std::array<double, 3> rpy{0.0, 0.0, 0.0};
};

// One-time setup + the single shared barrier parameter set.
struct CollisionMonitorConfig {
    bool enable = false;

    // Geometry source (unified stand+both-arms URDF; collision = box + meshes).
    std::string unified_urdf;             // .../dual_rb3_730e_ver3.urdf
    std::vector<std::string> package_dirs;  // resolve mesh "../../../meshes" paths
    std::string pika_gripper_mesh;        // optional; attached as a convex hull
    // ARTICULATED gripper collision (optional). When base + both finger meshes are set,
    // the gripper is modeled as a STATIC base hull + two MOVABLE finger hulls (convex
    // hulls of the visual STLs) attached at <prefix>attachment_site, mirroring
    // rb3_730e_pika_articulated.urdf: identity placement (no +90° Z), fingers translate
    // along the local +X (jaw axis) by setGripperOpenPercent so the checked gripper jaw
    // tracks the live open percent. Takes precedence over pika_gripper_mesh (single
    // static hull) when all three are present; otherwise the single-hull path is used.
    std::string pika_gripper_base_mesh;
    std::string pika_finger_left_mesh;
    std::string pika_finger_right_mesh;
    double gripper_finger_travel_m = 0.047;  // per-finger jaw travel open(0)->closed
    std::string stand_frame = "stand";    // unified-URDF frame for stand geometry
    std::string left_prefix = "dual_rb3_730e_left_";   // attachment_site prefix
    std::string right_prefix = "dual_rb3_730e_right_";
    // Unified-model joint names for the 6+6 actuated joints, command order.
    std::array<std::string, kDof> left_joints{};
    std::array<std::string, kDof> right_joints{};

    // Pair curation: arm<->arm and arm<->stand only (never intra-arm). Arm
    // geometry whose name contains any of these is NOT paired against the stand
    // (mount neighbors that touch the stand by construction). Replaces the
    // capsule path's stand_ignore_bones. NOTE: mesh geometry is accurate, so
    // (unlike the inflated capsules that needed [0,1,2]) only link0 — the base
    // flange bolted to the stand — structurally overlaps; link1/link2 are checked.
    std::vector<std::string> stand_ignore_arm_substrings{"link0"};

    // Left/right arm classification is by KINEMATIC-TREE ancestry, not by name
    // substring: a geometry is "left" iff this frame is on its parent-frame chain.
    // Defaults are derived from the prefixes ("<prefix>world") if left empty.
    std::string left_arm_root_frame;
    std::string right_arm_root_frame;

    // Intra-arm self-collision: also check a link of an arm against NON-adjacent
    // links of the SAME arm (an arm folding onto itself). Adjacency is measured by
    // chain depth (number of the arm's actuated joints between the two geometries);
    // pairs with separation < intra_arm_min_chain_separation are skipped (adjacent
    // links touch by construction).
    bool check_intra_arm = true;
    int intra_arm_min_chain_separation = 2;
    std::vector<CollisionPairPattern> disabled_collision_pairs;
    bool debug_pair_curation = false;

    // Swept-volume guard: each evaluation also samples intermediate configurations
    // between the previous evaluated config and the current target and keeps the
    // worst (min) clearance, so fast motion cannot tunnel a thin obstacle between
    // ticks. 1 = endpoint only (no sweep). >=2 adds interior samples.
    int swept_samples = 2;  // 1=endpoint, >=2 sweeps (cost ~x per sample)

    // ---- SELF-collision velocity-barrier params (robot<->robot / robot<->stand) ----
    // Common to ALL primitives. EXTERNAL-obstacle pairs (the floor / ground_plane —
    // see external_* below) use their own set so the floor can be approached closer
    // than the robot approaches itself.
    double d_hard_m = 0.005;       // hard floor: never cross; clamp_hold below this
    double d_slow_m = 0.025;       // above this clearance the barrier is inactive
    double a_brake_m_s2 = 4.0;     // emergency decel the barrier assumes
    double hyst_m = 0.005;         // release hysteresis for the discrete fault flag
    // Velocity-damper projection (Stage 2).
    int projection_iterations = 3;     // Gauss-Seidel sweeps over active near pairs
    double recover_speed_m_s = 0.0;    // active push-out below d_hard (0 = block deeper only)
    double latency_s = 0.010;      // assumed worst monitor->servo reaction latency
    double max_staleness_s = 0.020;  // verdict older than this -> fail closed

    // Thread placement (-1 = no affinity). Pin to an isolated core for stable
    // worst-case eval time.
    int monitor_core = -1;

    int max_near_pairs = 8;        // how many closest pairs to report in a verdict
    std::vector<ExtraCollisionShape> extra_collision;  // non-URDF geometry (camera/table/...)

    // ---- EXTERNAL-collision velocity-barrier params ----
    // Applied to pairs flagged external in the verdict (currently arm<->ground_plane,
    // the whole-arm floor). Kept fully separate from the self-collision set above so
    // the floor — a known flat surface the operator approaches deliberately — can be
    // cleared by a smaller margin than the robot keeps from itself. Defaults mirror the
    // self set except for a tighter d_hard.
    double external_d_hard_m = 0.003;
    double external_d_slow_m = 0.025;
    double external_a_brake_m_s2 = 4.0;
    double external_hyst_m = 0.005;
    double external_recover_speed_m_s = 0.0;
    double external_latency_s = 0.010;
};

// One reported near pair (witness points + approach direction in stand frame).
struct CollisionNearPair {
    int geom_a = -1;
    int geom_b = -1;
    double d_m = 0.0;                       // signed clearance (<0 = penetration)
    Eigen::Vector3d p_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_b = Eigen::Vector3d::Zero();
    Eigen::Vector3d n = Eigen::Vector3d::Zero();  // unit a->b
    std::string name_a;
    std::string name_b;
    // True if this is an arm<->EXTERNAL-obstacle pair (the floor / ground_plane), which
    // uses the external_* barrier params (smaller d_hard) instead of the self set.
    bool external = false;
    // Per-pair clearance Jacobian rows (Stage 1): d(clearance)/dt = J_n * qdot,
    // split into this command's actuated left/right joint columns (command order,
    // idx_v mapping). For an arm<->stand pair only the arm's row is non-zero, so the
    // velocity projection that consumes this acts only on the offending DOFs.
    Eigen::Matrix<double, 1, kDof> Jn_left = Eigen::Matrix<double, 1, kDof>::Zero();
    Eigen::Matrix<double, 1, kDof> Jn_right = Eigen::Matrix<double, 1, kDof>::Zero();
    // Signed clearance rate of THIS pair (+ = separating), tracked by pair index.
    double rate_m_s = 0.0;
};

// A published snapshot. Consumed by servo_j read-only.
struct CollisionVerdict {
    std::uint64_t seq = 0;
    double stamp_s = 0.0;                   // monotonic time the snapshot was taken
    bool valid = false;                     // false until first eval
    double min_clearance_m = std::numeric_limits<double>::infinity();
    // Per-category minima (the smaller of the two drives min_clearance_m). Self =
    // robot<->robot/stand; external = arm<->floor/obstacle. Each is compared against its
    // own d_hard for hard_violation and used by the InitMotion planner's per-category
    // clearance gate.
    double self_min_clearance_m = std::numeric_limits<double>::infinity();
    double external_min_clearance_m = std::numeric_limits<double>::infinity();
    // Signed rate of the CURRENTLY-critical (global-min) pair's clearance, tracked
    // per-pair so a switch of which pair is closest does not corrupt it.
    // + = separating, - = approaching.
    double clearance_rate_m_s = 0.0;
    double closing_speed_m_s = 0.0;         // max(0, -clearance_rate_m_s)
    bool hard_violation = false;            // min_clearance_m < d_hard_m
    std::vector<CollisionNearPair> near;    // up to max_near_pairs, sorted ascending
};

// True if the verdict is missing or too old to trust (caller should fail closed).
bool collisionVerdictStale(const CollisionVerdict& v, double now_s, double max_staleness_s);

// Speed scale in [0,1] for the commanded motion this tick. 1.0 = full speed.
// Pure function of the verdict + shared params (no state). Properties:
//   - !valid                          -> 0.0  (no verdict: fail closed)
//   - hard_violation & approaching    -> 0.0  (inside the floor, still closing: stop)
//   - hard_violation & receding       -> 1.0  (allow escaping the keep-out zone)
//   - clearance > d_slow_m            -> 1.0  (far: no effect)
//   - nothing closing (v_c ~ 0)       -> 1.0  (parallel/receding motion is free:
//                                              this is the anti-twitchy property)
//   - closing too fast to brake       -> v_allow/v_c  (smooth slow-down)
// Staleness must be checked separately (needs current time): a stale verdict
// should be treated as scale 0.0 by the caller.
//
// verdict_age_s: actual age of the verdict (now - v.stamp_s). When >= 0 the
// latency compensation uses max(cfg.latency_s, verdict_age_s) so a stale verdict
// never under-compensates (Issue 2a: the async monitor can lag the fixed
// latency_s floor). When < 0 (default) the fixed cfg.latency_s is used.
double collisionVelocityScale(const CollisionVerdict& v, const CollisionMonitorConfig& cfg,
                              double verdict_age_s = -1.0);

// Outcome of the velocity-damper projection (Stage 2).
struct CollisionProjectionResult {
    bool active = false;               // >=1 constraint engaged (command modified)
    int active_pairs = 0;
    double max_correction_deg_s = 0.0; // ||qdot - qdot_desired|| (both arms), deg/s
    double left_correction_deg_s = 0.0;
    double right_correction_deg_s = 0.0;
};

// One linear inequality on the commanded joint velocity (Stage 3, unified solver):
// d(constraint)/dt = J . qdot >= -xi, with qdot the 12-vector [left6; right6] in
// rad/s. Self-collision near pairs and floor-plane points are both expressed this
// way and solved together so they cannot fight. d_now is only a sort key (closest
// constraint relaxed first in Gauss-Seidel).
struct VelocityConstraint {
    Eigen::Matrix<double, 2 * kDof, 1> J = Eigen::Matrix<double, 2 * kDof, 1>::Zero();
    double xi = 0.0;
    double d_now = 0.0;
};

// Build the self-collision velocity constraints from a verdict (per near pair within
// d_slow, age-extrapolated). Appends to `out` (so floor rows can be added too).
void buildCollisionConstraints(const CollisionVerdict& v, const CollisionMonitorConfig& cfg,
                               double verdict_age_s, std::vector<VelocityConstraint>& out);

// Solve a set of velocity constraints by Gauss-Seidel projection (closest first),
// modifying the joint targets (degrees) in place. Pure; shared by self-collision and
// floor. `cons` is sorted in place. Returns per-arm correction magnitudes.
CollisionProjectionResult solveVelocityProjection(
    std::vector<VelocityConstraint>& cons,
    const JointArray& left_prev_deg, const JointArray& right_prev_deg,
    JointArray& left_target_deg, JointArray& right_target_deg,
    double dt_sec, int iterations, const JointArray& max_joint_vel_deg_s);

// Directional velocity-damper projection (Stage 2) — the chatter-free, fast-safe
// replacement for the scalar collisionVelocityScale. For each near pair within
// d_slow it removes ONLY the closing component of the commanded joint velocity
// (per-pair clearance Jacobian J_n), bounding the allowed closing speed by the
// braking limit sqrt(2 a_brake (d_now - d_hard)); tangential and separating motion
// pass through untouched, so the boundary never toggles and escape is always free.
// d_now is age-extrapolated (verdict_age_s). Joint targets (degrees) are modified
// in place; q velocity is clamped per joint to max_joint_vel_deg_s. A no-op (returns
// active=false) when the verdict is invalid, dt<=0, or nothing is within d_slow.
CollisionProjectionResult applyCollisionVelocityProjection(
    const CollisionVerdict& v, const CollisionMonitorConfig& cfg,
    const JointArray& left_prev_deg, const JointArray& right_prev_deg,
    JointArray& left_target_deg, JointArray& right_target_deg,
    double dt_sec, double verdict_age_s, const JointArray& max_joint_vel_deg_s);

struct CollisionDistanceSummary {
    bool hard_violation = false;
    double min_clearance_m = std::numeric_limits<double>::infinity();
    double self_min_clearance_m = std::numeric_limits<double>::infinity();
    double external_min_clearance_m = std::numeric_limits<double>::infinity();
    std::string nearest_name_a;
    std::string nearest_name_b;
    double nearest_distance_m = std::numeric_limits<double>::infinity();
    bool nearest_external = false;
    bool valid = false;
};

bool collisionPairPatternMatches(const CollisionPairPattern& rule,
                                 const std::string& name_a,
                                 const std::string& name_b);

// Owns the geometry model + the monitor thread + the published verdict.
class CollisionMonitor {
public:
    explicit CollisionMonitor(CollisionMonitorConfig config);
    ~CollisionMonitor();
    CollisionMonitor(const CollisionMonitor&) = delete;
    CollisionMonitor& operator=(const CollisionMonitor&) = delete;

    // Feed the latest targets (command order). Cheap atomic store; the thread
    // picks them up on its next iteration. Safe to call from servo_j.
    void submitTargets(const JointArray& left_deg, const JointArray& right_deg);

    // Latest published verdict (atomic load, no blocking, no compute).
    CollisionVerdict latest() const;

    // Runtime reposition of the injected "ground_plane" whole-arm floor box so it
    // tracks the operator's active viser floor (see ground_plane.follow_safety_floors).
    // `enabled=false` moves the box far below (inert: no pair ever near it). When
    // enabled, the box top face is placed at `point` with `normal` as its up axis (a
    // tilted user floor tilts the box). Cheap (mutex-guarded store); the monitor thread
    // applies it before its next distance eval. No-op if the model has no ground_plane.
    // Safe to call from servo_j. `normal` must be (near) unit and is used as-is.
    void setGroundPlanePose(bool enabled, const Eigen::Vector3d& point,
                            const Eigen::Vector3d& normal);

    // True if the model contains a "ground_plane" geometry (i.e. it can be tracked).
    bool hasGroundPlane() const;

    // Runtime jaw open percent (0 closed .. 100 open) for the ARTICULATED gripper
    // model: repositions that arm's two finger hulls along the local jaw axis (+X) so
    // the checked gripper tracks the live gripper, mirroring the URDF/GUI articulation.
    // Cheap (mutex store); applied on the monitor thread before its next eval. No-op
    // unless the articulated gripper meshes were configured. Safe to call from servo_j.
    // An oracle that never calls this stays at the conservative OPEN jaw (e.g. the
    // InitMotion planner) — the largest envelope.
    void setGripperOpenPercent(ArmId arm, double percent);

    // True if the articulated gripper (movable finger hulls) is in the model.
    bool hasArticulatedGripper() const;

    // Run one evaluation synchronously on the calling thread (startup / tests).
    CollisionVerdict evalOnce(const JointArray& left_deg, const JointArray& right_deg);

    // Planner-only lightweight queries. They update geometry placements and distances
    // but do not compute near-pair Jacobians/rates or publish a runtime verdict.
    CollisionDistanceSummary evalDistancesOnly(const JointArray& left_deg,
                                               const JointArray& right_deg);
    bool clearsThresholds(const JointArray& left_deg, const JointArray& right_deg,
                          double self_thresh_m, double external_thresh_m);

    void start();   // spawn the monitor thread
    void stop();    // join the monitor thread

    std::size_t numGeometries() const;
    std::size_t numPairs() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rb_servo
