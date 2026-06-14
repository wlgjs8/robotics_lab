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
// motion primitive (TcpPoseTarget, TcpTwistLocal, ...). Speed adaptation comes
// from the MEASURED closing speed of the geometry, not from per-primitive config,
// so nothing needs to be retuned per primitive.

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

// One-time setup + the single shared barrier parameter set.
struct CollisionMonitorConfig {
    bool enable = false;

    // Geometry source (unified stand+both-arms URDF; collision = box + meshes).
    std::string unified_urdf;             // .../dual_rb3_730e_ver3.urdf
    std::vector<std::string> package_dirs;  // resolve mesh "../../../meshes" paths
    std::string pika_gripper_mesh;        // optional; attached as a convex hull
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

    // ---- shared velocity-barrier params (common to ALL primitives) ----
    double d_hard_m = 0.005;       // hard floor: never cross; clamp_hold below this
    double d_slow_m = 0.025;       // above this clearance the barrier is inactive
    double a_brake_m_s2 = 4.0;     // emergency decel the barrier assumes
    double hyst_m = 0.005;         // release hysteresis for the discrete fault flag
    double latency_s = 0.005;      // assumed worst monitor->servo reaction latency
    double max_staleness_s = 0.020;  // verdict older than this -> fail closed

    // Thread placement (-1 = no affinity). Pin to an isolated core for stable
    // worst-case eval time.
    int monitor_core = -1;

    int max_near_pairs = 8;        // how many closest pairs to report in a verdict
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
};

// A published snapshot. Consumed by servo_j read-only.
struct CollisionVerdict {
    std::uint64_t seq = 0;
    double stamp_s = 0.0;                   // monotonic time the snapshot was taken
    bool valid = false;                     // false until first eval
    double min_clearance_m = std::numeric_limits<double>::infinity();
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
double collisionVelocityScale(const CollisionVerdict& v, const CollisionMonitorConfig& cfg);

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

    // Run one evaluation synchronously on the calling thread (startup / tests).
    CollisionVerdict evalOnce(const JointArray& left_deg, const JointArray& right_deg);

    void start();   // spawn the monitor thread
    void stop();    // join the monitor thread

    std::size_t numGeometries() const;
    std::size_t numPairs() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rb_servo
