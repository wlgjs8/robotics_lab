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
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
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
    // SCHED_FIFO priority of the monitor thread (0 = leave it SCHED_OTHER). The
    // monitor was the only safety-critical thread with no RT priority and no
    // core: CFS-scheduled among the GUI, the runner and the policy servers, with
    // max_staleness_s the entire margin absorbing that (2026-09-04 review).
    int monitor_realtime_priority = 0;
    // Gauss-Seidel convergence bound (Stage 3.5, 2026-09-04): the solve sweeps
    // until no row moved more than projection_tol_rad_s, at most this many
    // times. projection_iterations below is the MINIMUM. Three fixed sweeps over
    // 6-10 non-orthogonal rows sorted by a noisy d_now did not converge, so the
    // solution depended on the row order and jumped between ticks.
    int projection_max_sweeps = 50;
    double projection_tol_rad_s = 1e-6;

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

    // ---- INTRA-ARM self-collision velocity-barrier params ----
    // Applied only to same-arm non-adjacent link pairs. Arm<->arm and arm<->stand
    // keep the self set above.
    double intra_arm_d_hard_m = 0.005;
    double intra_arm_d_slow_m = 0.025;
    double intra_arm_a_brake_m_s2 = 4.0;
    double intra_arm_hyst_m = 0.005;
    double intra_arm_recover_speed_m_s = 0.0;
    double intra_arm_latency_s = 0.010;

    // ---- CELL-STRUCTURE (env_*) velocity-barrier params (2026-09-06) ----
    // The env_* geometry in the unified URDF that opted into a <collision>: the
    // riser under the stand base plate today. Its own class because the self set
    // does not fit a measured furniture box in either direction -- see
    // SelfCollisionConfig::MeshConfig::EnvironmentConfig for the measurement.
    // Defaults are the 25/67 mm pair this stack shipped as its self set until
    // 2026-09-05, which clears the riser's measured 35.7 mm closest approach and
    // still satisfies d_slow >= d_hard + v_max^2/(2*a_brake) at the 0.50 m/s ceiling.
    double environment_d_hard_m = 0.025;
    double environment_d_slow_m = 0.067;
    double environment_a_brake_m_s2 = 3.0;
    double environment_hyst_m = 0.010;
    double environment_recover_speed_m_s = 0.0;
    double environment_latency_s = 0.010;

    // ---- GRIPPER<->GRIPPER velocity-barrier params (2026-09-04) ----
    // The nine cross-arm pairs among the two Pika hulls (base, finger_left,
    // finger_right x the same on the other arm). They used to sit inside the
    // left-right set with the 25 mm self floor, which forbids the tip-to-tip
    // handover the cell needs. Their own class keeps the arm<->gripper and
    // arm<->arm pairs untouched, and lets the servo loop EXCLUDE these rows when
    // force control covers both arms (the contact is then the F/T sensor's to
    // handle). Defaults mirror the self set; the loop inherits unset values.
    double gripper_gripper_d_hard_m = 0.005;
    double gripper_gripper_d_slow_m = 0.025;
    double gripper_gripper_a_brake_m_s2 = 4.0;
    double gripper_gripper_hyst_m = 0.005;
    double gripper_gripper_recover_speed_m_s = 0.0;

    // ---- EXTERNAL-BOX keep-out velocity-barrier params ----
    // Applied ONLY to arm<->runtime-external-box pairs (the detected NTC-321 keep-out
    // boxes). Kept SEPARATE from the floor's external_* set above: a box is a keep-out the
    // operator drives TOWARD at teleop speed, so the slow zone must be WIDE enough to brake
    // the fastest approach before the hard floor. The stoppable approach speed is
    // sqrt(2*a_brake*(d_slow - d_hard)); the floor's 5 mm slow zone stops only ~0.12 m/s,
    // but SpaceMouse teleop reaches ~0.8 m/s and overshot ~40 mm INTO the box. These
    // defaults stop ~0.9 m/s (sqrt(2*6*(0.080-0.010))) and actively eject on penetration.
    double external_box_d_hard_m = 0.010;
    double external_box_d_slow_m = 0.080;
    double external_box_a_brake_m_s2 = 6.0;
    double external_box_hyst_m = 0.010;
    double external_box_recover_speed_m_s = 0.030;  // >0: push back out if it does penetrate
    double external_box_latency_s = 0.010;

    struct ExternalBoxesConfig {
        bool enable = false;
        int max_count = 2;
        std::array<double, 3> size_m{0.380, 0.240, 0.105};  // NTC-321 outer extents
        std::array<double, 3> margin_m{0.025, 0.025, 0.025};  // per-axis [x,y,z] box-local inflation; index 2 = height
        bool monitor_only = true;
        double stale_timeout_s = 0.5;
        std::string stale_policy = "hold";  // "hold" | "disable"
    };
    ExternalBoxesConfig external_boxes;
};

struct ExternalBoxPose {
    bool enable = false;
    // Caller guarantees R is orthonormal; CollisionMonitor stores and applies it as-is.
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    // Box center in the same stand frame used by setGroundPlanePose's `point`.
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
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
    // True for arm<->runtime external box pairs. Kept distinct from `external`
    // so ground-plane semantics and telemetry remain unchanged.
    bool external_box = false;
    // True for same-arm non-adjacent link pairs. These use intra_arm_* barrier
    // params instead of the arm<->arm / arm<->stand self set.
    bool intra_arm = false;
    // True for an arm<->cell-structure pair: the other member is env_* geometry
    // (environment_* params). Distinct from `external` (the whole-arm floor plane)
    // and from a plain arm<->stand pair.
    bool environment = false;
    // True for a cross-arm pair of two gripper hulls (gripper_gripper_* params;
    // excludable by the servo loop when force control covers both arms).
    bool gripper_gripper = false;
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
    // Per-category minima. Self = arm<->arm / arm<->stand, intra_arm = same-arm
    // non-adjacent links, external = arm<->floor/obstacle. Each is compared
    // against its own d_hard for hard_violation and used by the InitMotion
    // planner's per-category clearance gate.
    double self_min_clearance_m = std::numeric_limits<double>::infinity();
    double intra_arm_min_clearance_m = std::numeric_limits<double>::infinity();
    double external_min_clearance_m = std::numeric_limits<double>::infinity();
    double external_box_min_clearance_m = std::numeric_limits<double>::infinity();
    double gripper_gripper_min_clearance_m = std::numeric_limits<double>::infinity();
    double environment_min_clearance_m = std::numeric_limits<double>::infinity();
    // Per preallocated external box slot (slot 0=green, slot 1=gray).
    // +inf means no finite/active pair for that slot.
    std::vector<double> external_box_clearance_m;
    // Signed rate of the CURRENTLY-critical (global-min) pair's clearance, tracked
    // per-pair so a switch of which pair is closest does not corrupt it.
    // + = separating, - = approaching.
    double clearance_rate_m_s = 0.0;
    double closing_speed_m_s = 0.0;         // max(0, -clearance_rate_m_s)
    bool hard_violation = false;            // any pair below its category d_hard
    // Split of hard_violation by whether the breaching pair is gripper<->gripper,
    // so the loop can leave a covered handover contact out of the breach verdict
    // without losing every other pair's floor.
    bool hard_violation_non_gripper = false;
    bool gripper_gripper_hard_violation = false;
    // The near list: every pair inside its class's engage band (d < d_slow + hyst,
    // so a row the loop has engaged can never fall out of the list -- the
    // single-tick correction dropouts of 2026-08-28 were rows vanishing when a
    // pair lost its slot among the K nearest) plus the K globally nearest pairs,
    // sorted ascending by clearance.
    std::vector<CollisionNearPair> near;
    int near_band_count = 0;                // pairs of `near` inside their class band
    // The configuration the near list, witness points and Jacobians were
    // evaluated at (command order, degrees). Lets the consumer extrapolate each
    // pair's clearance to the pose it is about to command from, first order and
    // free of timing noise: d_now = d + Jn . (q - q_eval).
    std::array<double, 2 * kDof> q_eval_deg{};
    bool q_eval_valid = false;
    double eval_ms = 0.0;                   // wall time of the evaluation
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
    // The trailing global per-joint velocity ceiling actually bound this tick.
    // That ceiling is a 1-tick step (it is NOT accel-limited downstream), so
    // when it engages it must be visible in the log, not only in the summed
    // correction magnitude.
    bool ceiling_clamped = false;
    int sweeps_used = 0;               // Gauss-Seidel sweeps actually run
    bool converged = false;            // no row moved more than tol in the last sweep
};

struct VelocityProjectionOptions {
    int min_sweeps = 3;
    int max_sweeps = 50;
    double tol_rad_s = 1e-6;
};

// One linear inequality on the commanded joint velocity (Stage 3, unified solver):
// d(constraint)/dt = J . qdot >= -xi, with qdot the 12-vector [left6; right6] in
// rad/s. Self-collision near pairs and floor-plane points are both expressed this
// way and solved together so they cannot fight. d_now is only a sort key (closest
// constraint relaxed first in Gauss-Seidel).
// Which barrier parameter set a row was built from. Each class has its OWN
// d_hard, so a raw clearance is not comparable across classes: 20 mm is a breach
// for an arm<->arm pair (floor 25 mm) and normal for an intra-arm pair (floor
// 5 mm). Rows that are not collision pairs (floor plane, ROI, reach) keep Other.
enum class ConstraintClass : std::uint8_t {
    Other = 0, Self, IntraArm, External, ExternalBox, GripperGripper
};

struct VelocityConstraint {
    Eigen::Matrix<double, 2 * kDof, 1> J = Eigen::Matrix<double, 2 * kDof, 1>::Zero();
    double xi = 0.0;
    double d_now = 0.0;
    // Diagnosis fields. `d_now - d_hard` is the headroom that actually decides
    // whether this row is near its floor; `pair_key` identifies the geom pair so
    // the caller can name it out of the verdict's near list. Reported, never used
    // by the solve.
    double d_hard = 0.0;
    std::uint64_t pair_key = 0;   // geom_a<<32 | geom_b, same key as engaged_pairs
    ConstraintClass klass = ConstraintClass::Other;
};

// The barrier band ONE near pair is enforced against, by its category. The
// selection order is significant and is the same in every consumer: external_box ->
// external -> intra_arm -> gripper_gripper -> self (a pair is at most one of the
// first four, so the order only fixes the fallthrough to the self set).
//
// These exist as functions, rather than as ternaries repeated per call site, because
// the categories carry very different floors (self 40 mm, gripper<->gripper 25 mm,
// intra-arm 5 mm on the RB5) and the near list is ordered by RAW clearance. Anything
// that asks "is this pair violating?" or "how close is it, relatively?" MUST use the
// pair's own band; using the self floor for all of them reports the permanently-near
// structural intra-arm pair as a violation and misses the pair that really breached.
// buildCollisionConstraints and the state publisher's near-pair telemetry both read
// them here so the two cannot drift apart.
double nearPairHardFloorM(const CollisionMonitorConfig& cfg, const CollisionNearPair& p);
double nearPairSlowBandM(const CollisionMonitorConfig& cfg, const CollisionNearPair& p);

// Build the self-collision velocity constraints from a verdict (per near pair within
// d_slow, age-extrapolated). Appends to `out` (so floor rows can be added too).
//
// `engaged_pairs` (optional, caller-persisted across ticks, keyed by
// geom_a<<32|geom_b) makes the per-category `hyst_m` release hysteresis live: a
// pair engages at d_now < d_slow and releases only at d_now >= d_slow + hyst.
// Without it (nullptr, and in older builds) the row set flapped on/off at the
// band edge with verdict-age noise — the on/off limit cycle the hyst_m config
// comment promised to break but never did.
void buildCollisionConstraints(const CollisionVerdict& v, const CollisionMonitorConfig& cfg,
                               double verdict_age_s, std::vector<VelocityConstraint>& out,
                               std::unordered_set<std::uint64_t>* engaged_pairs = nullptr);

struct ConstraintBuildOptions {
    // Leave the gripper<->gripper rows out (force control owns that contact).
    // Pairs are still reported in the verdict; only the barrier rows are skipped,
    // and an excluded pair is dropped from the engaged set.
    bool exclude_gripper_gripper = false;
    // When both are set and the verdict carries q_eval, d_now is extrapolated by
    // Jn . (q - q_eval) -- the clearance at the pose the command starts from --
    // instead of rate * age. The rate was a finite difference between
    // evaluations that were not synchronised with the tick (it alternated
    // between 0 and 1.33x the true closing speed), and near the floor that noise
    // multiplied the sqrt barrier's infinite slope.
    const JointArray* left_q_deg = nullptr;
    const JointArray* right_q_deg = nullptr;
};

void buildCollisionConstraints(const CollisionVerdict& v, const CollisionMonitorConfig& cfg,
                               double verdict_age_s, std::vector<VelocityConstraint>& out,
                               std::unordered_set<std::uint64_t>* engaged_pairs,
                               const ConstraintBuildOptions& opts);

// Solve a set of velocity constraints by Gauss-Seidel projection (closest first),
// modifying the joint targets (degrees) in place. Pure; shared by self-collision and
// floor. `cons` is sorted in place. Returns per-arm correction magnitudes.
CollisionProjectionResult solveVelocityProjection(
    std::vector<VelocityConstraint>& cons,
    const JointArray& left_prev_deg, const JointArray& right_prev_deg,
    JointArray& left_target_deg, JointArray& right_target_deg,
    double dt_sec, int iterations, const JointArray& max_joint_vel_deg_s);

// Same solve, sweeping until convergence: at least opts.min_sweeps, then until
// no row changed qdot by more than opts.tol_rad_s, at most opts.max_sweeps. The
// per-joint ceiling is applied after every sweep, inside the loop, so a row the
// ceiling re-violated is revisited instead of being left broken by a trailing
// clamp. The fixed-iteration entry point above is this with min == max.
CollisionProjectionResult solveVelocityProjection(
    std::vector<VelocityConstraint>& cons,
    const JointArray& left_prev_deg, const JointArray& right_prev_deg,
    JointArray& left_target_deg, JointArray& right_target_deg,
    double dt_sec, const VelocityProjectionOptions& opts,
    const JointArray& max_joint_vel_deg_s);

// Per-arm entry point for per-arm control threads.
//
// Runs the SAME coupled 12-DOF solve and writes back only `arm`'s half. That is
// deliberate and is not a decoupled approximation: both arms see one shared
// collision verdict, hence the same constraint set, and the same inputs apart
// from one tick of peer-candidate staleness -- so they agree on the solution and
// each simply keeps its own part of it.
//
// The naive alternative (each arm solving as if the peer will not yield) is safe
// but DOUBLE-corrects, because both yield for a correction only one of them
// needed: near-contact approach speed halves for no safety gain. Solving the
// coupled system on both sides avoids that.
//
// Peer staleness is handled where every other latency already is -- the caller
// folds it into the collision verdict age that buildCollisionConstraints()
// extrapolates the clearance with (`d_now = d - closing * age`). At the command
// ceiling one tick is ~0.6 mm of closing, against an existing 50 ms staleness
// budget, so this adds no new margin term.
CollisionProjectionResult solveVelocityProjectionForArm(
    std::vector<VelocityConstraint>& cons,
    ArmId arm,
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
    double intra_arm_min_clearance_m = std::numeric_limits<double>::infinity();
    double external_min_clearance_m = std::numeric_limits<double>::infinity();
    double environment_min_clearance_m = std::numeric_limits<double>::infinity();
    std::string nearest_name_a;
    std::string nearest_name_b;
    double nearest_distance_m = std::numeric_limits<double>::infinity();
    bool nearest_external = false;
    bool nearest_intra_arm = false;
    std::string nearest_category;
    bool valid = false;
};

bool collisionPairPatternMatches(const CollisionPairPattern& rule,
                                 const std::string& name_a,
                                 const std::string& name_b);

// Fail-closed liveness decision for an ENFORCED external-box keep-out feed. Pure (all
// times in seconds) so it is unit-testable. Returns a human-readable abort reason, or
// nullptr if the feed is acceptable. Semantics:
//   - feed never seen: acceptable only within the startup grace (producer may be coming
//     up); past the grace -> abort (producer not running).
//   - feed seen before: acceptable only if the last feed is within feed_timeout_s;
//     a larger gap -> abort (producer stopped). Generous vs a normal multi-Hz feed so a
//     transient blip never false-aborts.
// Caller applies this ONLY when the boxes are enforced (enable && !monitor_only).
const char* externalBoxFeedAbortReason(bool feed_seen, double since_enforce_start_s,
                                       double since_last_feed_s, double startup_grace_s,
                                       double feed_timeout_s);

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
    // Per-arm variant for per-arm control threads (they never hold both
    // candidates at one instant). Evaluation starts once both arms have
    // submitted at least once.
    void submitArmTarget(ArmId arm, const JointArray& q_deg);

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

    // Runtime update for preallocated external keep-out boxes. Cheap
    // mutex-guarded store; the monitor thread applies placements before its next
    // distance eval. No-op if external_boxes.enable=false at construction.
    void setExternalBoxes(const std::vector<ExternalBoxPose>& boxes,
                          double stamp_monotonic_s);

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
    bool clearsThresholds(const JointArray& left_deg, const JointArray& right_deg,
                          double self_thresh_m, double external_thresh_m,
                          double intra_arm_thresh_m);

    // Active-arm-masked variants (planner use): only pairs that involve an INCLUDED
    // arm are considered. With include_left=false, every pair whose two bodies are
    // both non-left (left-only intra/arm-stand/external pairs) is skipped, and
    // symmetrically for include_right. A left<->right pair counts if EITHER arm is
    // included. This lets single-arm InitMotion gate solely on the collisions its
    // moving arm can cause (active<->other-arm, active<->stand, active<->external,
    // active intra-arm) and never fail-closed on the stationary other arm's own
    // clearance, which the runtime monitor already owns. (true,true) is identical to
    // the unmasked overloads above.
    CollisionDistanceSummary evalDistancesOnly(const JointArray& left_deg,
                                               const JointArray& right_deg,
                                               bool include_left, bool include_right);
    bool clearsThresholds(const JointArray& left_deg, const JointArray& right_deg,
                          double self_thresh_m, double external_thresh_m,
                          bool include_left, bool include_right);
    bool clearsThresholds(const JointArray& left_deg, const JointArray& right_deg,
                          double self_thresh_m, double external_thresh_m,
                          double intra_arm_thresh_m,
                          bool include_left, bool include_right);
    // With a CELL-STRUCTURE threshold for the env_* geometry. The overloads above
    // gate those pairs at self_thresh_m, which is what they did before env_* was
    // checked at all; a planner that knows the environment band should pass it, or it
    // plans against a floor 15 mm wider than the barrier actually enforces.
    bool clearsThresholds(const JointArray& left_deg, const JointArray& right_deg,
                          double self_thresh_m, double external_thresh_m,
                          double intra_arm_thresh_m, double environment_thresh_m,
                          bool include_left, bool include_right);

    void start();   // spawn the monitor thread
    void stop();    // join the monitor thread

    std::size_t numGeometries() const;
    std::size_t numPairs() const;
    // Number of collision meshes that were NOT convex and were kept as a (correct but
    // slow) BVH rather than convexified. 0 = the fast convex path everywhere (the
    // per-eval performance contract holds); >0 means correct-but-slow geometry is in
    // play (a loud startup warning named them) and a precomputed convex hull is needed.
    std::size_t numNonConvexMeshes() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rb_servo
