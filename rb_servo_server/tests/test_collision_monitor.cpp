// Unit test for the async mesh self-collision monitor.
// Skips gracefully if the unified URDF (mo_robot_descriptions sibling repo) is absent.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <utility>

#include "rb_servo/control/collision_monitor.hpp"

#define RB_CHECK(expr)                                                          \
    do {                                                                       \
        if (!(expr)) {                                                         \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":"    \
                      << __LINE__ << "\n";                                     \
            return false;                                                     \
        }                                                                     \
    } while (0)

namespace fs = std::filesystem;
using namespace rb_servo;

static fs::path workspaceRoot() {
    // tests/ -> rb_servo_server -> robotics_lab -> workspace
    return fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
}

static CollisionMonitorConfig makeConfig(const fs::path& ws) {
    CollisionMonitorConfig c;
    c.enable = true;
    // The tracked RB5 model, the same file the server enforces against. Previously the
    // sibling checkout's dual_rb5_850e_ver3, which meant this suite kept validating the
    // retired robot after the 2026-09-02 swap -- and, because that model ships
    // non-convex collision meshes, kept SKIPPING its own per-eval performance gate.
    const fs::path urdf_dir = ws / "robotics_lab/rb_servo_server/descriptions/urdf";
    c.unified_urdf = (urdf_dir / "dual_rb5_850e_ver3.urdf").string();
    c.package_dirs = {urdf_dir.string()};
    // The precomputed HULL, matching what the tracked configs load. The fixture used
    // the raw pika_gripper.STL, which is not convex, so the monitor kept it as a BVH --
    // and the per-eval performance gate below, whose whole purpose is to hold the servo
    // reaction budget, skipped itself on every run because of that one mesh.
    c.pika_gripper_mesh =
        (ws / "robotics_lab/rb_servo_server/descriptions/meshes/robots/rb5_850e/visual/tool/"
              "pika_gripper_hull.STL")
            .string();
    c.left_prefix = "dual_rb5_850e_left_";
    c.right_prefix = "dual_rb5_850e_right_";
    const char* jn[kDof] = {"base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"};
    for (int i = 0; i < kDof; ++i) {
        c.left_joints[i] = c.left_prefix + jn[i] + "_joint";
        c.right_joints[i] = c.right_prefix + jn[i] + "_joint";
    }
    return c;
}

// Shared probe pose, CHOSEN BY SCANNING the monitor itself rather than by eye, because
// runExternalDHard needs two things at once from it: the whole arm clear of the z=0
// stand floor (it drops a ground plane 5 mm under the lowest arm point and needs that
// height positive), and EVERY other pair further than that 5 mm, so the floor is
// unambiguously the closest pair. The RB3 value {0,-30,80,0,60,0} fails the first on RB5
// -- 90.6 mm BELOW the floor -- because the mounts mirror in Y, sit 10 mm lower, and the
// links are ~30% longer. A first replacement fixed the floor but failed the second: at
// J3 = 160 the arm folds enough for the gripper hull to PENETRATE its own link2
// (-0.46 mm), which self_min_clearance_m does not report because intra-arm pairs are
// counted separately. Scanned against the monitor on both criteria at once; this pose
// clears the floor by 338.5 mm with the nearest non-floor pair at 21.3 mm.
static const JointArray kInitPose = {-90.0, -90.0, 110.0, 0.0, 0.0, 0.0};

static bool sameDistance(double a, double b) {
    if (std::isinf(a) && std::isinf(b) && std::signbit(a) == std::signbit(b)) return true;
    return std::abs(a - b) < 1e-6;
}

static bool nearPairMatches(const CollisionNearPair& p, const CollisionPairPattern& rule) {
    return collisionPairPatternMatches(rule, p.name_a, p.name_b);
}

static bool verdictHasPair(const CollisionVerdict& v, const CollisionPairPattern& rule) {
    for (const auto& p : v.near) {
        if (nearPairMatches(p, rule)) return true;
    }
    return false;
}

static bool runPairPatternMatching() {
    CollisionPairPattern exact{"right_link0", "right_link1"};
    RB_CHECK(collisionPairPatternMatches(exact, "right_link0", "right_link1"));
    RB_CHECK(collisionPairPatternMatches(exact, "right_link1", "right_link0"));
    RB_CHECK(!collisionPairPatternMatches(exact, "right_link0", "right_link2"));

    CollisionPairPattern glob{"*right*link0*", "*right*link1*"};
    RB_CHECK(collisionPairPatternMatches(
        glob, "dual_rb5_850e_right_link0", "dual_rb5_850e_right_link1"));
    RB_CHECK(collisionPairPatternMatches(
        glob, "dual_rb5_850e_right_link1", "dual_rb5_850e_right_link0"));
    RB_CHECK(!collisionPairPatternMatches(
        glob, "dual_rb5_850e_left_link0", "dual_rb5_850e_right_link1"));

    CollisionPairPattern link02{"*right*link0*", "*right*link2*"};
    RB_CHECK(collisionPairPatternMatches(
        link02, "dual_rb5_850e_right_link0_0", "dual_rb5_850e_right_link2_0"));
    RB_CHECK(collisionPairPatternMatches(
        link02, "dual_rb5_850e_right_link2_0", "dual_rb5_850e_right_link0_0"));
    RB_CHECK(!collisionPairPatternMatches(
        link02, "dual_rb5_850e_right_link0_0", "dual_rb5_850e_right_link3_0"));
    return true;
}

static bool run() {
    const fs::path ws = workspaceRoot();
    CollisionMonitorConfig cfg = makeConfig(ws);
    if (!fs::is_regular_file(cfg.unified_urdf)) {
        std::cout << "SKIP: unified URDF not found (" << cfg.unified_urdf << ")\n";
        return true;  // not a failure on machines without the sibling repo
    }

    CollisionMonitor mon(cfg);

    // geometry: link0..6 hulls (left 12 / right 12) + stand 7 + 2 gripper = 33
    std::cout << "geoms=" << mon.numGeometries() << " pairs=" << mon.numPairs() << "\n";
    RB_CHECK(mon.numGeometries() == 51);  // see the breakdown in runArticulatedGripper
    RB_CHECK(mon.numPairs() > 0);

    {
        CollisionMonitorConfig all_pairs_cfg = cfg;
        all_pairs_cfg.swept_samples = 1;
        all_pairs_cfg.max_near_pairs = 10000;
        CollisionPairPattern adjacent_right{"*right*link0*", "*right*link1*"};
        CollisionPairPattern link02_right{"*right*link0*", "*right*link2*"};
        CollisionPairPattern wrist_left{"*left*link4*", "*left*link6*"};
        CollisionPairPattern wrist_right{"*right*link4*", "*right*link6*"};
        CollisionMonitor all_pairs(all_pairs_cfg);
        const JointArray init = kInitPose;
        const CollisionVerdict before = all_pairs.evalOnce(init, init);
        RB_CHECK(before.valid);
        const bool adjacent_present = verdictHasPair(before, adjacent_right);
        const bool link02_present = verdictHasPair(before, link02_right);
        std::cout << "right link0-link1 baseline pair present=" << adjacent_present << "\n";
        std::cout << "right link0-link2 baseline pair present=" << link02_present << "\n";

        CollisionMonitorConfig disabled_cfg = all_pairs_cfg;
        disabled_cfg.disabled_collision_pairs.push_back(adjacent_right);
        disabled_cfg.disabled_collision_pairs.push_back(link02_right);
        CollisionMonitor disabled(disabled_cfg);
        const CollisionVerdict after = disabled.evalOnce(init, init);
        RB_CHECK(after.valid);
        RB_CHECK(!verdictHasPair(after, adjacent_right));
        RB_CHECK(!verdictHasPair(after, link02_right));
        if (adjacent_present || link02_present) {
            RB_CHECK(disabled.numPairs() < all_pairs.numPairs());
        } else {
            // On URDFs where these structural base pairs are classified as adjacent
            // same-arm links, intra-arm chain separation already excludes them
            // before SRDF-style rules.
            RB_CHECK(disabled.numPairs() == all_pairs.numPairs());
        }

        CollisionMonitorConfig reversed_cfg = all_pairs_cfg;
        reversed_cfg.disabled_collision_pairs.push_back(
            CollisionPairPattern{"*right*link1*", "*right*link0*"});
        reversed_cfg.disabled_collision_pairs.push_back(
            CollisionPairPattern{"*right*link2*", "*right*link0*"});
        CollisionMonitor reversed(reversed_cfg);
        RB_CHECK(reversed.numPairs() == disabled.numPairs());

        CollisionMonitorConfig lr_cfg = all_pairs_cfg;
        CollisionPairPattern left_right{"*left*link6*", "*right*link6*"};
        lr_cfg.disabled_collision_pairs.push_back(left_right);
        CollisionMonitor lr_disabled(lr_cfg);
        const CollisionVerdict lr_after = lr_disabled.evalOnce(init, init);
        RB_CHECK(lr_after.valid);
        RB_CHECK(!verdictHasPair(lr_after, left_right));
        RB_CHECK(lr_disabled.numPairs() < all_pairs.numPairs());

        CollisionMonitorConfig wrist_cfg = all_pairs_cfg;
        wrist_cfg.disabled_collision_pairs.push_back(wrist_left);
        wrist_cfg.disabled_collision_pairs.push_back(wrist_right);
        CollisionMonitor wrist_disabled(wrist_cfg);
        const CollisionVerdict wrist_after = wrist_disabled.evalOnce(init, init);
        RB_CHECK(wrist_after.valid);
        RB_CHECK(!verdictHasPair(wrist_after, wrist_left));
        RB_CHECK(!verdictHasPair(wrist_after, wrist_right));
        RB_CHECK(wrist_disabled.numPairs() < all_pairs.numPairs());
    }

    const JointArray init = kInitPose;

    // (1) init pose: no collision, clearance comfortably positive (PoC ~97 mm).
    CollisionVerdict v = mon.evalOnce(init, init);
    std::cout << "init min_clearance=" << v.min_clearance_m * 1000.0 << " mm  near="
              << (v.near.empty() ? "?" : v.near.front().name_a + "<->" + v.near.front().name_b)
              << "\n";
    RB_CHECK(v.valid);
    RB_CHECK(!v.hard_violation);
    RB_CHECK(std::isfinite(v.min_clearance_m));
    // Closest at init is the intra-arm link2<->link4 pair (~28 mm across the elbow):
    // well clear of the 5 mm hard floor; not a violation. (>15 mm regression guard.)
    RB_CHECK(v.min_clearance_m > 0.015);
    RB_CHECK(!v.near.empty());
    RB_CHECK(v.near.front().n.norm() > 0.5);  // unit approach dir populated
    {
        CollisionDistanceSummary s = mon.evalDistancesOnly(init, init);
        RB_CHECK(s.valid);
        RB_CHECK(s.hard_violation == v.hard_violation);
        RB_CHECK(sameDistance(s.min_clearance_m, v.min_clearance_m));
        RB_CHECK(sameDistance(s.self_min_clearance_m, v.self_min_clearance_m));
        RB_CHECK(sameDistance(s.intra_arm_min_clearance_m, v.intra_arm_min_clearance_m));
        RB_CHECK(sameDistance(s.external_min_clearance_m, v.external_min_clearance_m));
        const double self_thresh = cfg.d_hard_m + 0.005;
        const double ext_thresh = cfg.external_d_hard_m + 0.005;
        const double intra_thresh = cfg.intra_arm_d_hard_m + 0.005;
        const bool gate = !v.hard_violation &&
                          v.self_min_clearance_m > self_thresh &&
                          v.intra_arm_min_clearance_m > intra_thresh &&
                          v.external_min_clearance_m > ext_thresh;
        RB_CHECK(mon.clearsThresholds(init, init, self_thresh, ext_thresh, intra_thresh) == gate);
    }
    {
        CollisionMonitorConfig endpoint_cfg = cfg;
        endpoint_cfg.swept_samples = 1;
        CollisionMonitor endpoint(endpoint_cfg);
        const double self_thresh = endpoint_cfg.d_hard_m + 0.005;
        const double ext_thresh = endpoint_cfg.external_d_hard_m + 0.005;
        const double intra_thresh = endpoint_cfg.intra_arm_d_hard_m + 0.005;
        const std::array<std::pair<JointArray, JointArray>, 4> samples{{
            {init, init},
            {JointArray{8.0, -38.0, 72.0, 4.0, 55.0, 6.0},
             JointArray{-7.0, -25.0, 88.0, -6.0, 64.0, -5.0}},
            {JointArray{18.0, -45.0, 95.0, 16.0, 35.0, -14.0},
             JointArray{-16.0, -18.0, 62.0, -12.0, 78.0, 12.0}},
            {JointArray{-22.0, -20.0, 70.0, -25.0, 82.0, 18.0},
             JointArray{20.0, -44.0, 96.0, 24.0, 38.0, -16.0}},
        }};
        for (const auto& sample : samples) {
            const CollisionVerdict full = endpoint.evalOnce(sample.first, sample.second);
            const CollisionDistanceSummary light =
                endpoint.evalDistancesOnly(sample.first, sample.second);
            RB_CHECK(full.valid);
            RB_CHECK(light.valid);
            RB_CHECK(light.hard_violation == full.hard_violation);
            RB_CHECK(sameDistance(light.min_clearance_m, full.min_clearance_m));
            RB_CHECK(sameDistance(light.self_min_clearance_m, full.self_min_clearance_m));
            RB_CHECK(sameDistance(light.intra_arm_min_clearance_m,
                                  full.intra_arm_min_clearance_m));
            RB_CHECK(sameDistance(light.external_min_clearance_m, full.external_min_clearance_m));
            const bool full_gate = !full.hard_violation &&
                                   full.self_min_clearance_m > self_thresh &&
                                   full.intra_arm_min_clearance_m > intra_thresh &&
                                   full.external_min_clearance_m > ext_thresh;
            const bool light_gate = !light.hard_violation &&
                                    light.self_min_clearance_m > self_thresh &&
                                    light.intra_arm_min_clearance_m > intra_thresh &&
                                    light.external_min_clearance_m > ext_thresh;
            RB_CHECK(light_gate == full_gate);
            RB_CHECK(endpoint.clearsThresholds(sample.first, sample.second,
                                               self_thresh, ext_thresh,
                                               intra_thresh) == full_gate);
        }
    }

    // (2) velocity barrier: at large clearance, full speed regardless.
    RB_CHECK(collisionVelocityScale(v, cfg) == 1.0);

    // (3) barrier monotonicity + safety on a synthetic near/closing verdict.
    CollisionVerdict close = v;
    close.min_clearance_m = 0.010;   // 10 mm, inside d_slow
    close.closing_speed_m_s = 0.0;   // not closing -> free (anti-twitchy)
    RB_CHECK(collisionVelocityScale(close, cfg) == 1.0);
    close.closing_speed_m_s = 1.0;   // closing fast -> must slow (<1)
    const double s_fast = collisionVelocityScale(close, cfg);
    RB_CHECK(s_fast < 1.0 && s_fast >= 0.0);
    close.closing_speed_m_s = 0.05;  // closing slowly at 10 mm -> allowed full
    RB_CHECK(collisionVelocityScale(close, cfg) == 1.0);

    // (4) hard violation: HOLD unless clearly retreating; invalid -> stop.
    CollisionVerdict hv = v; hv.hard_violation = true;
    hv.clearance_rate_m_s = -0.5;  // approaching (clearance shrinking) -> stop
    RB_CHECK(collisionVelocityScale(hv, cfg) == 0.0);
    hv.clearance_rate_m_s = 0.0;   // stationary while breached -> hold (fail safe)
    RB_CHECK(collisionVelocityScale(hv, cfg) == 0.0);
    hv.clearance_rate_m_s = 0.5;   // clearly retreating -> allow escape
    RB_CHECK(collisionVelocityScale(hv, cfg) == 1.0);
    CollisionVerdict inv; // default invalid
    RB_CHECK(collisionVelocityScale(inv, cfg) == 0.0);

    // (5) staleness.
    RB_CHECK(collisionVerdictStale(inv, 100.0, cfg.max_staleness_s));
    RB_CHECK(!collisionVerdictStale(v, v.stamp_s + 0.001, cfg.max_staleness_s));
    RB_CHECK(collisionVerdictStale(v, v.stamp_s + 1.0, cfg.max_staleness_s));

    // (5b) age extrapolation (Issue 2a): a larger actual verdict age compensates a
    // larger closing distance, so the barrier is monotonically MORE restrictive as
    // the verdict ages. The default (age < 0) and age <= latency_s reproduce the
    // fixed-latency_s behavior exactly (backward compatible).
    CollisionVerdict aged = v;
    aged.min_clearance_m = 0.020;       // inside slow-zone (d_slow 25mm)
    aged.closing_speed_m_s = 0.5;       // fast head-on
    const double s_default = collisionVelocityScale(aged, cfg);            // fixed latency_s
    const double s_age_lat = collisionVelocityScale(aged, cfg, cfg.latency_s);
    const double s_age_small = collisionVelocityScale(aged, cfg, 0.0);     // age 0 -> max(lat,0)=lat
    RB_CHECK(s_default == s_age_lat);   // default arg == explicit latency
    RB_CHECK(s_age_small == s_default); // age below latency floor -> latency_s used
    RB_CHECK(s_default > 0.0 && s_default < 1.0);  // fixed latency: partial slow-down
    const double s_age_big = collisionVelocityScale(aged, cfg, 0.040);     // stale verdict
    RB_CHECK(s_age_big <= s_default);   // older verdict -> never less conservative
    RB_CHECK(s_age_big == 0.0);         // 0.020-0.005-0.5*0.040 = -0.005 <= 0 -> halt

    // (5c) per-pair clearance Jacobian J_n (Stage 1): central finite-difference.
    // Use a NO-SWEEP monitor so each eval is a pure endpoint distance (the swept
    // guard mixes configs and would break finite differencing).
    {
        CollisionMonitorConfig cfg1 = cfg;
        cfg1.swept_samples = 1;
        CollisionMonitor mon1(cfg1);
        const JointArray base = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};
        const CollisionVerdict v0 = mon1.evalOnce(base, base);
        RB_CHECK(!v0.near.empty());
        const CollisionNearPair pr = v0.near.front();  // closest pair to validate
        RB_CHECK(pr.Jn_left.allFinite() && pr.Jn_right.allFinite());
        RB_CHECK(pr.Jn_left.norm() + pr.Jn_right.norm() > 1e-6);  // not all-zero
        const JointArray dl = {0.4, -0.5, 0.6, 0.3, -0.2, 0.5};   // deg perturbation
        const JointArray dr = {-0.3, 0.4, -0.5, 0.2, 0.6, -0.4};
        JointArray lp, lm, rp, rm;
        for (int i = 0; i < kDof; ++i) {
            lp[i] = base[i] + dl[i]; lm[i] = base[i] - dl[i];
            rp[i] = base[i] + dr[i]; rm[i] = base[i] - dr[i];
        }
        const CollisionVerdict vp = mon1.evalOnce(lp, rp);
        const CollisionVerdict vm = mon1.evalOnce(lm, rm);
        const auto find = [&](const CollisionVerdict& vv) -> const CollisionNearPair* {
            for (const auto& np : vv.near)
                if (np.geom_a == pr.geom_a && np.geom_b == pr.geom_b) return &np;
            return nullptr;
        };
        const CollisionNearPair* pp = find(vp);
        const CollisionNearPair* pm = find(vm);
        RB_CHECK(pp != nullptr && pm != nullptr);  // same pair survives the step
        const double dd_fd = pp->d_m - pm->d_m;     // central diff over +-delta
        const double k = 3.14159265358979323846 / 180.0;
        double dd_pred = 0.0;
        for (int i = 0; i < kDof; ++i) {
            dd_pred += pr.Jn_left[i] * (2.0 * dl[i] * k) + pr.Jn_right[i] * (2.0 * dr[i] * k);
        }
        std::cout << "Jn FD: pred=" << dd_pred * 1000.0 << " mm  actual=" << dd_fd * 1000.0
                  << " mm\n";
        RB_CHECK(std::abs(dd_pred - dd_fd) < 5e-4);  // <0.5 mm agreement over the step
    }

    // (5b) Per-arm projection must reconstruct the coupled solution exactly.
    //
    // Each arm runs the same coupled solve and keeps only its own half; together
    // that must equal the one-shot coupled result. The naive alternative -- each
    // arm solving as if the peer stays put -- double-corrects, because both yield
    // for a correction only one of them needed.
    //
    // The constraint is built BY HAND rather than taken from the monitor: this
    // needs one whose Jacobian genuinely spans both arms (a left<->right pair).
    // Constraints picked out of a real verdict are usually arm<->stand, where the
    // peer half is ~0 and the comparison cannot tell the two implementations
    // apart -- an earlier version of this test passed for both because of exactly
    // that.
    {
        const JointArray base = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};
        VelocityConstraint c;
        for (int i = 0; i < kDof; ++i) {
            c.J[i] = (i % 2 == 0) ? 0.30 : -0.20;          // left half
            c.J[kDof + i] = (i % 2 == 0) ? -0.25 : 0.35;   // right half, comparable size
        }
        c.xi = 0.05;      // small allowance -> the barrier binds
        c.d_now = 0.010;
        const std::vector<VelocityConstraint> seed{c};

        const double dt = 0.002;
        const double rad2deg = 180.0 / 3.14159265358979323846;
        // qdot = -J * k gives ddot = -k|J|^2; k = 2*xi/|J|^2 makes ddot = -2*xi.
        const double k = 2.0 * c.xi / c.J.squaredNorm();
        JointArray lp = base, rp = base, lt = base, rt = base;
        for (int i = 0; i < kDof; ++i) {
            lt[i] = base[i] - c.J[i] * k * dt * rad2deg;
            rt[i] = base[i] - c.J[kDof + i] * k * dt * rad2deg;
        }
        const JointArray vmax = {1e6, 1e6, 1e6, 1e6, 1e6, 1e6};  // isolate the barrier

        std::vector<VelocityConstraint> cons_ref = seed;
        JointArray l_ref = lt, r_ref = rt;
        solveVelocityProjection(cons_ref, lp, rp, l_ref, r_ref, dt, 3, vmax);
        double deflect = 0.0;
        for (int i = 0; i < kDof; ++i) {
            deflect += std::abs(l_ref[i] - lt[i]) + std::abs(r_ref[i] - rt[i]);
        }
        RB_CHECK(deflect > 1e-6);   // the barrier must actually have bound

        std::vector<VelocityConstraint> cons_l = seed;
        JointArray l_arm = lt, r_peer = rt;
        solveVelocityProjectionForArm(cons_l, ArmId::Left, lp, rp, l_arm, r_peer, dt, 3, vmax);
        std::vector<VelocityConstraint> cons_r = seed;
        JointArray l_peer = lt, r_arm = rt;
        solveVelocityProjectionForArm(cons_r, ArmId::Right, lp, rp, l_peer, r_arm, dt, 3, vmax);

        for (int i = 0; i < kDof; ++i) {
            RB_CHECK(std::abs(l_arm[i] - l_ref[i]) < 1e-12);
            RB_CHECK(std::abs(r_arm[i] - r_ref[i]) < 1e-12);
            // The half an arm does not own must come back untouched.
            RB_CHECK(std::abs(r_peer[i] - rt[i]) < 1e-12);
            RB_CHECK(std::abs(l_peer[i] - lt[i]) < 1e-12);
        }
    }

    // (6) threaded publish/consume: start, submit, see a fresh verdict.
    //
    // Submitted PER ARM, the way per-arm control threads do it: neither thread
    // ever holds both candidates, so the monitor pairs the latest of each. One
    // arm alone must NOT arm the evaluator -- pairing a real pose with a
    // default-constructed counterpart would check a pose the robot is not in.
    mon.start();
    // latest() keeps the previous verdict, so the evidence that a lone arm does
    // not arm the evaluator is that the verdict SEQUENCE does not advance.
    const std::uint64_t seq_before = mon.latest().seq;
    mon.submitArmTarget(ArmId::Left, init);
    for (int i = 0; i < 25; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    RB_CHECK(mon.latest().seq == seq_before);
    mon.submitArmTarget(ArmId::Right, init);
    CollisionVerdict t{};
    for (int i = 0; i < 200 && !t.valid; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        t = mon.latest();
    }
    mon.stop();
    RB_CHECK(t.valid);
    RB_CHECK(t.seq > 0);
    RB_CHECK(std::isfinite(t.min_clearance_m));
    std::cout << "threaded verdict seq=" << t.seq << " min=" << t.min_clearance_m * 1000.0
              << " mm\n";

    // (7) eval-time guard (Stage 1 cost gate): the per-pair J_n computation runs in
    // the async monitor, not the 2 ms servo path, but it must still stay well within
    // the ~5 ms reaction budget. Time evalOnce over a few configs and assert a loose
    // ceiling (typical ~0.4-1.5 ms; bound generous to absorb CI noise).
    {
        const JointArray a = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};
        const JointArray b = {10.0, -20.0, 60.0, 15.0, 40.0, -10.0};
        for (int i = 0; i < 5; ++i) mon.evalOnce(a, b);  // warm up
        const int iters = 50;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < iters; ++i) mon.evalOnce((i % 2) ? a : b, (i % 2) ? b : a);
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count() / iters;
        std::cout << "evalOnce mean (with J_n) = " << ms << " ms  (non-convex meshes kept as BVH: "
                  << mon.numNonConvexMeshes() << ")\n";
        // The reaction-budget ceiling only applies to the fast convex path. Non-convex
        // meshes are (correctly) kept as BVH and are MUCH slower to evaluate; the
        // production model (ver5 + precomputed *_hull gripper meshes) has none, so the
        // strict gate holds there. This legacy fixture (ver3) still carries some non-convex
        // arm visual meshes, so only assert the ceiling when the geometry is all-convex.
        if (mon.numNonConvexMeshes() == 0) {
            RB_CHECK(ms < 5.0);  // must not blow the reaction-latency budget
        } else {
            std::cout << "  [perf gate skipped: " << mon.numNonConvexMeshes()
                      << " non-convex BVH mesh(es) in this fixture — supply convex hulls"
                         " for the fast path]\n";
        }
    }
    return true;
}

// Synthetic single-pair verdict for the pure projection tests (no URDF needed).
static CollisionVerdict makePairVerdict(double d_m, double rate,
                                        const std::array<double, kDof>& jl,
                                        const std::array<double, kDof>& jr) {
    CollisionVerdict v;
    v.valid = true;
    v.stamp_s = 0.0;
    v.min_clearance_m = d_m;
    v.hard_violation = d_m < 0.015;
    CollisionNearPair p;
    p.d_m = d_m;
    p.rate_m_s = rate;
    for (int i = 0; i < kDof; ++i) {
        p.Jn_left[i] = jl[i];
        p.Jn_right[i] = jr[i];
    }
    v.near.push_back(p);
    return v;
}

// Engage/release hysteresis on the constraint rows (buildCollisionConstraints with
// a caller-persisted engaged set): engage below d_slow, release only above
// d_slow + hyst_m, so margin noise at the band edge cannot flap the row on/off.
static bool runConstraintHysteresis() {
    CollisionMonitorConfig cfg;
    cfg.d_hard_m = 0.010;
    cfg.d_slow_m = 0.030;
    cfg.a_brake_m_s2 = 3.0;
    cfg.hyst_m = 0.010;
    std::array<double, kDof> jl{};
    jl[0] = 1.0;
    std::array<double, kDof> jr{};
    std::unordered_set<std::uint64_t> engaged;
    std::vector<VelocityConstraint> cons;

    // Fresh pair just OUTSIDE d_slow: no row, no engagement.
    buildCollisionConstraints(makePairVerdict(0.032, 0.0, jl, jr), cfg, 0.0, cons, &engaged);
    RB_CHECK(cons.empty() && engaged.empty());
    // Inside d_slow: row appears, pair engages.
    cons.clear();
    buildCollisionConstraints(makePairVerdict(0.028, 0.0, jl, jr), cfg, 0.0, cons, &engaged);
    RB_CHECK(cons.size() == 1 && engaged.size() == 1);
    // Back to 0.032 (inside the hysteresis band): the row PERSISTS -- this is
    // the flap the dead hyst_m config comment promised to break.
    cons.clear();
    buildCollisionConstraints(makePairVerdict(0.032, 0.0, jl, jr), cfg, 0.0, cons, &engaged);
    RB_CHECK(cons.size() == 1 && engaged.size() == 1);
    // Above d_slow + hyst_m: released.
    cons.clear();
    buildCollisionConstraints(makePairVerdict(0.041, 0.0, jl, jr), cfg, 0.0, cons, &engaged);
    RB_CHECK(cons.empty() && engaged.empty());
    // Stateless call (no engaged set) keeps the legacy edge behavior.
    cons.clear();
    buildCollisionConstraints(makePairVerdict(0.032, 0.0, jl, jr), cfg, 0.0, cons);
    RB_CHECK(cons.empty());
    std::cout << "constraint hysteresis: engage<d_slow, release>=d_slow+hyst OK\n";
    return true;
}

// Each row must carry the floor it was built against, so a raw clearance can be
// turned into a comparable "how close to breaching" number.
//
// This is what the servo log could not answer on 2026-08-28: the CSV had one
// min-clearance minimised across rows whose floors differ 5x (arm<->arm 25 mm vs
// intra-arm 5 mm), and a pair column that only carried a SIDE category and read
// "all" on 115118 of 115439 rows. A run showing "30 s below 25 mm" was therefore
// indistinguishable between an arm<->arm breach and ordinary intra-arm geometry,
// and tuning the barrier on it would have traded workspace for nothing.
static bool runConstraintClassAndFloor() {
    CollisionMonitorConfig cfg;
    cfg.d_hard_m = 0.025;          // arm<->arm / arm<->stand
    cfg.d_slow_m = 0.067;
    cfg.a_brake_m_s2 = 3.0;
    cfg.intra_arm_d_hard_m = 0.005;
    cfg.intra_arm_d_slow_m = 0.015;
    cfg.intra_arm_a_brake_m_s2 = 3.0;
    std::array<double, kDof> jl{};
    jl[0] = 1.0;
    std::array<double, kDof> jr{};

    // Same 12 mm clearance, two classes: a BREACH for arm<->arm, comfortably
    // inside its floor for intra-arm. The raw d_now cannot tell them apart.
    std::vector<VelocityConstraint> self_rows;
    buildCollisionConstraints(makePairVerdict(0.012, 0.0, jl, jr), cfg, 0.0, self_rows);
    RB_CHECK(self_rows.size() == 1);
    RB_CHECK(self_rows[0].klass == ConstraintClass::Self);
    RB_CHECK(std::abs(self_rows[0].d_hard - 0.025) < 1e-12);
    RB_CHECK(self_rows[0].d_now - self_rows[0].d_hard < 0.0);   // below ITS floor

    CollisionVerdict intra = makePairVerdict(0.012, 0.0, jl, jr);
    intra.near[0].intra_arm = true;
    std::vector<VelocityConstraint> intra_rows;
    buildCollisionConstraints(intra, cfg, 0.0, intra_rows);
    RB_CHECK(intra_rows.size() == 1);
    RB_CHECK(intra_rows[0].klass == ConstraintClass::IntraArm);
    RB_CHECK(std::abs(intra_rows[0].d_hard - 0.005) < 1e-12);
    RB_CHECK(intra_rows[0].d_now - intra_rows[0].d_hard > 0.0);  // above ITS floor

    // Identical raw clearance, opposite verdicts once the floor is carried.
    RB_CHECK(std::abs(self_rows[0].d_now - intra_rows[0].d_now) < 1e-12);

    // The key identifies the geom pair, so the caller can name it out of the verdict.
    RB_CHECK(self_rows[0].pair_key ==
             ((static_cast<std::uint64_t>(
                   static_cast<std::uint32_t>(intra.near[0].geom_a)) << 32) |
              static_cast<std::uint32_t>(intra.near[0].geom_b)));
    std::cout << "constraint class/floor: 12mm reads breach for self, clear for intra_arm OK\n";
    return true;
}

// Stage 2: directional velocity-damper projection (pure function; analytic checks).
static bool runProjection() {
    CollisionMonitorConfig pc;
    pc.d_hard_m = 0.015;
    pc.d_slow_m = 0.035;
    pc.a_brake_m_s2 = 4.0;
    pc.projection_iterations = 3;
    pc.recover_speed_m_s = 0.0;
    const JointArray zero{0, 0, 0, 0, 0, 0};
    // Non-binding joint-vel ceiling (test commands use large per-tick deltas to make
    // the projection math easy to read; we are not exercising the clamp here).
    const JointArray big{1e7, 1e7, 1e7, 1e7, 1e7, 1e7};
    const double dt = 0.002;
    const double deg2rad = 3.14159265358979323846 / 180.0;
    // Pair clearance Jacobian: +qdot on left joint0 SEPARATES (Jn_left[0] = -1 means
    // a +command closes -> ddot = -qdot). Right columns zero -> arm-stand-like.
    const std::array<double, kDof> jl = {-1.0, 0, 0, 0, 0, 0};
    const std::array<double, kDof> jr = {0, 0, 0, 0, 0, 0};

    // (1) closing too fast: reduced to the braking limit xi, not toggled/zeroed.
    // d=20mm, margin=5mm, xi=sqrt(2*4*0.005)=0.2 m/s -> qdot_left0 -> 0.2 rad/s.
    {
        CollisionVerdict v = makePairVerdict(0.020, 0.0, jl, jr);
        JointArray lt = zero, rt = zero;
        lt[0] = 1.0;  // +1 deg/tick command (closing)
        const auto r = applyCollisionVelocityProjection(v, pc, zero, zero, lt, rt, dt, 0.0, big);
        const double expect_deg = 0.2 * dt / deg2rad;  // ~0.0229 deg
        std::cout << "proj close: lt0=" << lt[0] << " expect=" << expect_deg << "\n";
        RB_CHECK(r.active && r.left_correction_deg_s > 0.0);
        RB_CHECK(std::abs(lt[0] - expect_deg) < 2e-3);
        for (int i = 1; i < kDof; ++i) RB_CHECK(lt[i] == 0.0 && rt[i] == 0.0);
        RB_CHECK(rt[0] == 0.0);  // independent DOF untouched
    }
    // (2) separating motion is free (no change), even inside d_slow.
    {
        CollisionVerdict v = makePairVerdict(0.020, 0.0, jl, jr);
        JointArray lt = zero, rt = zero;
        lt[0] = -1.0;  // command separates
        const auto r = applyCollisionVelocityProjection(v, pc, zero, zero, lt, rt, dt, 0.0, big);
        RB_CHECK(std::abs(lt[0] - (-1.0)) < 1e-9);  // untouched
        RB_CHECK(r.left_correction_deg_s < 1e-6);
    }
    // (3) tangential + independent arm free: command joint1 (Jn=0) and right joint0.
    {
        CollisionVerdict v = makePairVerdict(0.020, 0.0, jl, jr);
        JointArray lt = zero, rt = zero;
        lt[1] = 1.0;  // tangential to the constraint
        rt[0] = 5.0;  // other arm; Jn_right = 0
        applyCollisionVelocityProjection(v, pc, zero, zero, lt, rt, dt, 0.0, big);
        RB_CHECK(std::abs(lt[1] - 1.0) < 1e-9 && lt[0] == 0.0);
        RB_CHECK(std::abs(rt[0] - 5.0) < 1e-9);  // independent arm untouched
    }
    // (4) below d_hard: closing fully blocked (xi=0 -> qdot_left0 -> 0); escape free.
    {
        CollisionVerdict v = makePairVerdict(0.010, 0.0, jl, jr);  // < d_hard
        JointArray lt = zero, rt = zero;
        lt[0] = 1.0;  // closing
        applyCollisionVelocityProjection(v, pc, zero, zero, lt, rt, dt, 0.0, big);
        RB_CHECK(std::abs(lt[0]) < 2e-4);  // ~0: no deeper
        JointArray lt2 = zero, rt2 = zero;
        lt2[0] = -1.0;  // escaping
        applyCollisionVelocityProjection(v, pc, zero, zero, lt2, rt2, dt, 0.0, big);
        RB_CHECK(std::abs(lt2[0] - (-1.0)) < 1e-9);  // escape unrestricted
    }
    // (5) far (> d_slow): inactive, command unchanged.
    {
        CollisionVerdict v = makePairVerdict(0.050, 0.0, jl, jr);
        JointArray lt = zero, rt = zero;
        lt[0] = 1.0;
        const auto r = applyCollisionVelocityProjection(v, pc, zero, zero, lt, rt, dt, 0.0, big);
        RB_CHECK(!r.active && std::abs(lt[0] - 1.0) < 1e-9);
    }
    // (6b) combined solve (Stage 3): a collision-like row and a floor-like row that
    // share a joint are BOTH satisfied after the unified Gauss-Seidel (no fighting).
    {
        std::vector<VelocityConstraint> cons(2);
        cons[0].J[0] = -1.0;                 // collision: joint0 closing
        cons[0].xi = 0.20;
        cons[0].d_now = 0.020;
        cons[1].J[0] = -1.0; cons[1].J[1] = -1.0;  // floor-like: couples joint0 + joint1
        cons[1].xi = 0.10;
        cons[1].d_now = 0.010;               // closest -> relaxed first
        JointArray lt = zero, rt = zero;
        lt[0] = 1.0;
        lt[1] = 0.5;
        solveVelocityProjection(cons, zero, zero, lt, rt, dt, 10, big);
        const double qd0 = lt[0] * deg2rad / dt;
        const double qd1 = lt[1] * deg2rad / dt;
        const double ddot0 = -qd0;            // cons[0]
        const double ddot1 = -qd0 - qd1;      // cons[1]
        std::cout << "combined: ddot0=" << ddot0 << " (>= -0.20)  ddot1=" << ddot1
                  << " (>= -0.10)\n";
        RB_CHECK(ddot0 >= -0.20 - 1e-3);      // both constraints feasible
        RB_CHECK(ddot1 >= -0.10 - 1e-3);
    }

    // (6) age extrapolation pulls a closing pair from beyond d_slow into the active
    // set: d=40mm (>d_slow) but closing 1 m/s for 10 ms -> d_now=30mm (active).
    {
        CollisionVerdict v = makePairVerdict(0.040, -1.0, jl, jr);  // rate<0 = closing
        JointArray lt = zero, rt = zero;
        lt[0] = 1.0;
        const auto r0 = applyCollisionVelocityProjection(v, pc, zero, zero, lt, rt, dt, 0.0, big);
        RB_CHECK(!r0.active && std::abs(lt[0] - 1.0) < 1e-9);  // age 0: still far/inactive
        JointArray lt2 = zero, rt2 = zero;
        lt2[0] = 1.0;
        const auto r1 = applyCollisionVelocityProjection(v, pc, zero, zero, lt2, rt2, dt, 0.010, big);
        RB_CHECK(r1.active && lt2[0] < 1.0 && lt2[0] > 0.0);  // age 10ms: now braking
    }
    // (7) Intra-arm uses its own barrier params. A 10mm clearance is inside the
    // global self hard floor (18mm) but outside the intra-arm hard floor (5mm).
    {
        CollisionMonitorConfig cat;
        cat.d_hard_m = 0.018;
        cat.d_slow_m = 0.025;
        cat.a_brake_m_s2 = 3.0;
        cat.intra_arm_d_hard_m = 0.005;
        cat.intra_arm_d_slow_m = 0.015;
        cat.intra_arm_a_brake_m_s2 = 3.0;
        CollisionVerdict intra = makePairVerdict(0.010, 0.0, jl, jr);
        intra.hard_violation = false;
        intra.near[0].intra_arm = true;
        CollisionVerdict self = intra;
        self.hard_violation = true;
        self.near[0].intra_arm = false;

        std::vector<VelocityConstraint> intra_cons;
        std::vector<VelocityConstraint> self_cons;
        buildCollisionConstraints(intra, cat, 0.0, intra_cons);
        buildCollisionConstraints(self, cat, 0.0, self_cons);
        RB_CHECK(intra_cons.size() == 1);
        RB_CHECK(self_cons.size() == 1);
        RB_CHECK(intra_cons[0].xi > 0.15);  // sqrt(2*3*(10mm-5mm)) ~= 0.173m/s
        RB_CHECK(std::abs(self_cons[0].xi) < 1e-12);  // below self hard -> hold
    }
    return true;
}

// Runtime-tracked whole-arm floor (ground_plane.follow_safety_floors): the injected
// "ground_plane" box can be repositioned/disabled at runtime via setGroundPlanePose
// so it follows the operator's active viser floor. Verifies: parked (disabled) -> not
// a near pair; raised into the arm volume (enabled) -> becomes the binding pair;
// disabling again drops it back out. (Mirrors how dual_arm_servo_loop injects the box.)
static bool runGroundPlane() {
    const fs::path ws = workspaceRoot();
    CollisionMonitorConfig cfg = makeConfig(ws);
    if (!fs::is_regular_file(cfg.unified_urdf)) {
        std::cout << "SKIP: unified URDF not found (ground_plane test)\n";
        return true;
    }
    // Inject the whole-arm floor box exactly as the servo loop does (parent_frame is
    // the stand frame so its pose maps through o_M_stand_).
    ExtraCollisionShape gp;
    gp.name = "ground_plane";
    gp.shape = "box";
    gp.parent_frame = cfg.stand_frame;  // "stand"
    gp.size_m = {4.0, 4.0, 0.10};       // [Lx, Ly, thickness]
    gp.xyz_m = {0.0, 0.0, 0.001 - 0.05};  // top face at z=1mm (center half-thickness below)
    cfg.extra_collision.push_back(gp);

    CollisionMonitor mon(cfg);
    RB_CHECK(mon.hasGroundPlane());

    const JointArray init = kInitPose;

    auto groundPlaneNear = [](const CollisionVerdict& v) {
        for (const auto& p : v.near)
            if (p.name_a == "ground_plane" || p.name_b == "ground_plane") return true;
        return false;
    };

    // (1) Parked/disabled: the box sits far below, never a near pair.
    mon.setGroundPlanePose(false, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
    const CollisionVerdict off = mon.evalOnce(init, init);
    RB_CHECK(off.valid);
    RB_CHECK(!groundPlaneNear(off));
    const double baseline = off.min_clearance_m;
    std::cout << "ground_plane OFF: min_clearance=" << baseline * 1000.0
              << " mm gp_near=" << groundPlaneNear(off) << "\n";

    // (2) Enabled and raised to z=0.5 m (stand frame): the box now intrudes into the
    // lower arm volume, so it becomes the closest pair and clearance drops below the
    // parked baseline.
    mon.setGroundPlanePose(true, Eigen::Vector3d(0.0, 0.0, 0.5), Eigen::Vector3d::UnitZ());
    const CollisionVerdict on = mon.evalOnce(init, init);
    RB_CHECK(on.valid);
    RB_CHECK(groundPlaneNear(on));
    RB_CHECK(on.min_clearance_m < baseline);
    std::cout << "ground_plane ON(z=0.5): min_clearance=" << on.min_clearance_m * 1000.0
              << " mm gp_near=" << groundPlaneNear(on) << "\n";

    // (3) Disable again: the box returns far below and drops out of the near pairs,
    // restoring the parked baseline clearance.
    mon.setGroundPlanePose(false, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
    const CollisionVerdict off2 = mon.evalOnce(init, init);
    RB_CHECK(!groundPlaneNear(off2));
    RB_CHECK(std::abs(off2.min_clearance_m - baseline) < 1e-6);
    return true;
}

// External-collision category: the arm<->ground_plane pair is flagged `external` and
// gated by external_d_hard_m, NOT the (robot) self d_hard. The SAME floor clearance is a
// hard violation or not depending purely on the external d_hard.
static bool runExternalDHard() {
    const fs::path ws = workspaceRoot();
    CollisionMonitorConfig base = makeConfig(ws);
    if (!fs::is_regular_file(base.unified_urdf)) {
        std::cout << "SKIP: unified URDF not found (external d_hard test)\n";
        return true;
    }
    ExtraCollisionShape gp;
    gp.name = "ground_plane";
    gp.shape = "box";
    gp.parent_frame = base.stand_frame;
    gp.size_m = {4.0, 4.0, 0.10};
    gp.xyz_m = {0.0, 0.0, 0.001 - 0.05};
    base.extra_collision.push_back(gp);

    const JointArray init = kInitPose;

    // Measure the lowest arm/gripper point's height above a z=0 floor.
    CollisionMonitor m0(base);
    m0.setGroundPlanePose(true, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
    const CollisionVerdict v0 = m0.evalOnce(init, init);
    const double H = v0.external_min_clearance_m;  // lowest point height above z=0
    RB_CHECK(std::isfinite(H) && H > 0.010);
    const double z_top = H - 0.005;  // floor top 5 mm below that point -> ~5 mm clearance

    // Self d_hard large (20 mm), external d_hard tiny (2 mm): the floor pair at ~5 mm is
    // NOT a hard violation (5 mm > 2 mm) even though it is well inside the self d_hard.
    CollisionMonitorConfig cA = base;
    cA.d_hard_m = 0.020;
    cA.external_d_hard_m = 0.002;
    CollisionMonitor mA(cA);
    mA.setGroundPlanePose(true, Eigen::Vector3d(0.0, 0.0, z_top), Eigen::Vector3d::UnitZ());
    const CollisionVerdict vA = mA.evalOnce(init, init);
    RB_CHECK(!vA.near.empty());
    RB_CHECK(vA.near.front().external);  // closest pair is the floor, flagged external
    RB_CHECK(std::abs(vA.external_min_clearance_m - 0.005) < 0.0015);
    RB_CHECK(std::isfinite(vA.self_min_clearance_m) && vA.self_min_clearance_m > 0.020);
    RB_CHECK(!vA.hard_violation);  // governed by external d_hard (2 mm), not self (20 mm)

    // Same geometry/clearance, external d_hard raised to 10 mm (> 5 mm): now the floor
    // pair IS a hard violation — proving the external d_hard alone governs floor pairs.
    CollisionMonitorConfig cB = base;
    cB.external_d_hard_m = 0.010;
    CollisionMonitor mB(cB);
    mB.setGroundPlanePose(true, Eigen::Vector3d(0.0, 0.0, z_top), Eigen::Vector3d::UnitZ());
    const CollisionVerdict vB = mB.evalOnce(init, init);
    RB_CHECK(vB.hard_violation);
    std::cout << "external d_hard: floor clearance=" << vA.external_min_clearance_m * 1000.0
              << "mm  hard@2mm=" << vA.hard_violation << " hard@10mm=" << vB.hard_violation << "\n";
    return true;
}

// Intra-arm category: same-arm pairs use intra_arm_d_hard_m instead of the global
// arm<->arm / arm<->stand self d_hard. A folded single-arm pose in the 5-18mm
// band is therefore not a hard violation, while the same clearance to a stand
// obstacle remains a self hard violation.
static bool runIntraArmDHard() {
    const fs::path ws = workspaceRoot();
    CollisionMonitorConfig base = makeConfig(ws);
    if (!fs::is_regular_file(base.unified_urdf)) {
        std::cout << "SKIP: unified URDF not found (intra-arm d_hard test)\n";
        return true;
    }
    base.swept_samples = 1;
    base.max_near_pairs = 10000;
    base.d_hard_m = 0.018;
    base.d_slow_m = 0.025;
    base.intra_arm_d_hard_m = 0.005;
    base.intra_arm_d_slow_m = 0.015;
    base.intra_arm_a_brake_m_s2 = 3.0;
    const JointArray init = kInitPose;

    CollisionMonitor intra(base);
    bool found_intra_band = false;
    CollisionDistanceSummary intra_summary;
    JointArray folded = init;
    // RB5-850E folds to +/-165, so the intra-arm band sits further round than the
    // RB3 scan (135..150) could reach.
    for (double elbow_abs = 130.0; elbow_abs <= 165.0 && !found_intra_band; elbow_abs += 1.0) {
        for (double sign : std::array<double, 2>{1.0, -1.0}) {
            folded = init;
            folded[2] = sign * elbow_abs;
            const CollisionDistanceSummary s =
                intra.evalDistancesOnly(folded, init, true, false);
            if (std::isfinite(s.intra_arm_min_clearance_m) &&
                s.intra_arm_min_clearance_m > 0.0055 &&
                s.intra_arm_min_clearance_m < 0.0175 &&
                !s.hard_violation) {
                found_intra_band = true;
                intra_summary = s;
                break;
            }
        }
    }
    if (!found_intra_band) {
        std::cout << "SKIP: no 5-18mm intra-arm sample found in elbow scan\n";
    } else {
        RB_CHECK(intra_summary.nearest_intra_arm ||
                 intra_summary.intra_arm_min_clearance_m <= intra_summary.min_clearance_m + 1e-6);
        RB_CHECK(!intra_summary.hard_violation);
        RB_CHECK(intra_summary.intra_arm_min_clearance_m < base.d_hard_m);
        RB_CHECK(intra_summary.intra_arm_min_clearance_m > base.intra_arm_d_hard_m);
        std::cout << "intra-arm d_hard: clearance="
                  << intra_summary.intra_arm_min_clearance_m * 1000.0
                  << "mm hard=" << intra_summary.hard_violation << "\n";
    }

    // Build an arm<->stand clearance in the same 10mm band. This must still be a
    // hard violation because arm<->stand keeps the global self d_hard=18mm.
    CollisionMonitorConfig floor_ref = base;
    ExtraCollisionShape gp;
    gp.name = "ground_plane";
    gp.shape = "box";
    gp.parent_frame = base.stand_frame;
    gp.size_m = {4.0, 4.0, 0.10};
    gp.xyz_m = {0.0, 0.0, 0.001 - 0.05};
    floor_ref.extra_collision.push_back(gp);
    CollisionMonitor m0(floor_ref);
    m0.setGroundPlanePose(true, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
    const CollisionVerdict v0 = m0.evalOnce(init, init);
    const double H = v0.external_min_clearance_m;
    RB_CHECK(std::isfinite(H) && H > 0.020);

    CollisionMonitorConfig stand_probe = base;
    ExtraCollisionShape box;
    box.name = "stand_clearance_probe";
    box.shape = "box";
    box.parent_frame = base.stand_frame;
    box.size_m = {4.0, 4.0, 0.10};
    box.xyz_m = {0.0, 0.0, (H - 0.010) - 0.05};
    stand_probe.extra_collision.push_back(box);
    CollisionMonitor stand_monitor(stand_probe);
    const CollisionDistanceSummary stand_summary =
        stand_monitor.evalDistancesOnly(init, init);
    RB_CHECK(std::isfinite(stand_summary.self_min_clearance_m));
    RB_CHECK(stand_summary.self_min_clearance_m < base.d_hard_m);
    RB_CHECK(stand_summary.hard_violation);
    RB_CHECK(stand_summary.nearest_category == "arm-stand" ||
             stand_summary.self_min_clearance_m <= stand_summary.min_clearance_m + 1e-6);
    std::cout << "arm-stand self d_hard: clearance="
              << stand_summary.self_min_clearance_m * 1000.0
              << "mm hard=" << stand_summary.hard_violation << "\n";
    return true;
}

// A prefix that does not match the URDF must FAIL CLOSED. left_prefix/right_prefix
// default to one robot's link naming, so pointing unified_urdf at a different robot
// without setting them used to degrade silently in two ways -- the ancestry lookup
// just cleared have_arm_roots, and that flag is what separates arm<->arm from
// intra-arm pairs and therefore which d_hard_m floor applies, while the gripper attach
// indexed model.frames[getFrameId(...)] with no existence test at all, reading off the
// end of the vector because Pinocchio returns nframes for an unknown name. Caught
// during the RB3-730E -> RB5-850E swap, where every frame is renamed at once.
static bool runPrefixMismatchFailsClosed() {
    const fs::path ws = workspaceRoot();
    CollisionMonitorConfig cfg = makeConfig(ws);
    if (!fs::is_regular_file(cfg.unified_urdf)) {
        std::cout << "SKIP: unified URDF not found (prefix mismatch test)\n";
        return true;
    }
    CollisionMonitorConfig bad = cfg;
    bad.left_prefix = "no_such_robot_left_";
    for (int i = 0; i < kDof; ++i) bad.left_joints[i] = bad.left_prefix + "base_joint";
    bool threw = false;
    try {
        CollisionMonitor probe(bad);
    } catch (const std::exception& e) {
        threw = true;
        std::cout << "prefix mismatch refused: " << e.what() << "\n";
    }
    RB_CHECK(threw);
    return true;
}

// Articulated gripper: the two finger hulls reposition along the jaw axis with the live
// open percent, so finger<->other-geometry clearances change between OPEN and CLOSED.
static bool runArticulatedGripper() {
    const fs::path ws = workspaceRoot();
    CollisionMonitorConfig cfg = makeConfig(ws);
    if (!fs::is_regular_file(cfg.unified_urdf)) {
        std::cout << "SKIP: unified URDF not found (articulated gripper test)\n";
        return true;
    }
    const fs::path tool =
        ws / "robotics_lab/rb_servo_server/descriptions/meshes/robots/rb5_850e/visual/tool";
    cfg.pika_gripper_base_mesh = (tool / "pika_gripper_base.STL").string();
    cfg.pika_finger_left_mesh = (tool / "pika_finger_left.STL").string();
    cfg.pika_finger_right_mesh = (tool / "pika_finger_right.STL").string();
    cfg.gripper_finger_travel_m = 0.047;
    if (!fs::is_regular_file(cfg.pika_finger_left_mesh)) {
        std::cout << "SKIP: finger meshes not found (articulated gripper test)\n";
        return true;
    }
    cfg.max_near_pairs = 600;  // report every pair so finger pairs are always present

    CollisionMonitor mon(cfg);
    RB_CHECK(mon.hasArticulatedGripper());
    // RB5-850E, derived rather than guessed:
    //   per arm  11 link hulls (link0,1,4,5,6 single + link2,link3 CoACD x3)
    //          +  3 gripper (base + 2 fingers)          = 14
    //   stand     7 primitive boxes + 20 CoACD hulls    = 27
    //   total    14 x 2 + 27                            = 55
    // The single-hull baseline replaces the 3 gripper geoms with 1, so 51.
    // The gripper bolts straight to the flange -- the F/T sensor is inside
    // pika_gripper.STL, not a separate body (docs/reference/pika_tool_geometry.md).
    std::cout << "articulated geoms=" << mon.numGeometries() << "\n";
    RB_CHECK(mon.numGeometries() == 55);

    const JointArray init = kInitPose;
    auto fingerClears = [](const CollisionVerdict& v) {
        std::map<std::string, double> mp;
        for (const auto& p : v.near) {
            if (p.name_a.find("pika_finger") != std::string::npos ||
                p.name_b.find("pika_finger") != std::string::npos) {
                mp[p.name_a + "|" + p.name_b] = p.d_m;
            }
        }
        return mp;
    };

    mon.setGripperOpenPercent(ArmId::Left, 100.0);
    mon.setGripperOpenPercent(ArmId::Right, 100.0);
    const auto fo = fingerClears(mon.evalOnce(init, init));
    mon.setGripperOpenPercent(ArmId::Left, 0.0);
    mon.setGripperOpenPercent(ArmId::Right, 0.0);
    const auto fc = fingerClears(mon.evalOnce(init, init));
    RB_CHECK(!fo.empty());
    double max_delta = 0.0;
    for (const auto& [k, d] : fo) {
        auto it = fc.find(k);
        if (it != fc.end()) max_delta = std::max(max_delta, std::abs(d - it->second));
    }
    // The fingers translate up to 0.047 m; their clearances must move with the jaw.
    RB_CHECK(max_delta > 0.001);
    std::cout << "articulated gripper: finger clearance delta open->closed = "
              << max_delta * 1000.0 << "mm\n";
    return true;
}

// Fail-closed external-box feed liveness decision (pure helper used by the servo loop's
// checkExternalBoxFeedOrAbort): abort the process if an ENFORCED box keep-out has no live
// producer feed, instead of silently running without keep-out.
static bool runExternalBoxFeedLiveness() {
    constexpr double grace = 10.0, timeout = 3.0;
    // Never seen, still within startup grace -> OK (producer may be coming up).
    RB_CHECK(externalBoxFeedAbortReason(false, 2.0, 0.0, grace, timeout) == nullptr);
    RB_CHECK(externalBoxFeedAbortReason(false, grace, 0.0, grace, timeout) == nullptr);  // boundary
    // Never seen, past grace -> ABORT (producer not running).
    RB_CHECK(externalBoxFeedAbortReason(false, grace + 0.1, 0.0, grace, timeout) != nullptr);
    // Seen and fresh -> OK.
    RB_CHECK(externalBoxFeedAbortReason(true, 100.0, 1.0, grace, timeout) == nullptr);
    RB_CHECK(externalBoxFeedAbortReason(true, 100.0, timeout, grace, timeout) == nullptr);  // boundary
    // Seen but stale beyond timeout -> ABORT (producer stopped).
    RB_CHECK(externalBoxFeedAbortReason(true, 100.0, timeout + 0.1, grace, timeout) != nullptr);
    std::cout << "external box feed liveness: OK\n";
    return true;
}

// External keep-out BOX pairs must route to the box-only barrier set (wide slow zone),
// NOT the floor's `external` set — the fix for teleop overshooting ~40mm into a box
// because boxes reused the floor's 5 mm slow zone.
static bool runExternalBoxBarrierRouting() {
    const std::array<double, kDof> jl = {-1.0, 0, 0, 0, 0, 0};  // +cmd closes the pair
    const std::array<double, kDof> jr = {0, 0, 0, 0, 0, 0};
    CollisionMonitorConfig cfg;
    cfg.external_boxes.monitor_only = false;         // enforce boxes
    cfg.external_d_slow_m = 0.005;                   // floor: narrow (5 mm)
    cfg.external_box_d_hard_m = 0.010;               // box: wide keep-out set
    cfg.external_box_d_slow_m = 0.080;
    cfg.external_box_a_brake_m_s2 = 6.0;
    cfg.external_box_recover_speed_m_s = 0.030;

    // A pair at 40 mm: inside the BOX slow zone (80 mm) but outside the FLOOR slow zone (5 mm).
    CollisionVerdict vbox = makePairVerdict(0.040, 0.0, jl, jr);
    vbox.near[0].external_box = true;
    std::vector<VelocityConstraint> cbox;
    buildCollisionConstraints(vbox, cfg, 0.0, cbox);
    RB_CHECK(cbox.size() == 1);  // box's wide slow zone engaged at 40 mm
    // xi uses the BOX a_brake/d_hard: sqrt(2*6*(0.040-0.010)) = 0.6.
    RB_CHECK(std::abs(cbox[0].xi - std::sqrt(2.0 * 6.0 * 0.030)) < 1e-6);

    // The SAME 40 mm pair as a FLOOR (external) pair must NOT engage (5 mm slow zone) —
    // proves routing selects the box set for boxes, not the shared floor set.
    CollisionVerdict vflo = makePairVerdict(0.040, 0.0, jl, jr);
    vflo.near[0].external = true;
    std::vector<VelocityConstraint> cflo;
    buildCollisionConstraints(vflo, cfg, 0.0, cflo);
    RB_CHECK(cflo.empty());
    std::cout << "external box barrier routing: OK\n";
    return true;
}


// 2026-09-04: gripper<->gripper class routing + exclusion, and configuration-based
// clearance extrapolation (ConstraintBuildOptions).
static bool runGripperClassAndExtrapolation() {
    CollisionMonitorConfig cfg;
    cfg.d_hard_m = 0.010;
    cfg.d_slow_m = 0.030;
    cfg.a_brake_m_s2 = 3.0;
    cfg.hyst_m = 0.010;
    cfg.gripper_gripper_d_hard_m = 0.010;
    cfg.gripper_gripper_d_slow_m = 0.030;
    cfg.gripper_gripper_a_brake_m_s2 = 3.0;
    cfg.gripper_gripper_hyst_m = 0.010;
    std::array<double, kDof> jl{};
    jl[0] = -1.0;  // +q on left joint0 CLOSES the gap (1 m per rad)
    std::array<double, kDof> jr{};
    std::unordered_set<std::uint64_t> engaged;
    std::vector<VelocityConstraint> cons;

    // A gripper pair inside the band: a row of its own class, unless excluded.
    CollisionVerdict grip = makePairVerdict(0.020, 0.0, jl, jr);
    grip.near.front().gripper_gripper = true;
    buildCollisionConstraints(grip, cfg, 0.0, cons, &engaged);
    RB_CHECK(cons.size() == 1 && cons[0].klass == ConstraintClass::GripperGripper);
    RB_CHECK(engaged.size() == 1);
    cons.clear();
    ConstraintBuildOptions excl;
    excl.exclude_gripper_gripper = true;
    buildCollisionConstraints(grip, cfg, 0.0, cons, &engaged, excl);
    RB_CHECK(cons.empty() && engaged.empty());  // no row, and no longer "engaged"
    // A non-gripper pair is untouched by the exclusion.
    buildCollisionConstraints(makePairVerdict(0.020, 0.0, jl, jr), cfg, 0.0, cons, &engaged, excl);
    RB_CHECK(cons.size() == 1 && cons[0].klass == ConstraintClass::Self);
    cons.clear();
    engaged.clear();

    // Extrapolation by configuration: the verdict was evaluated at q_eval = 0; the
    // command starts from q = +0.1 rad on the closing joint, so the clearance the
    // row must use is d + Jn.dq = 0.050 - 0.100 = -0.050 (already through the
    // floor), even though rate*age would have said 0.050 (no row).
    CollisionVerdict far = makePairVerdict(0.050, 0.0, jl, jr);
    far.q_eval_valid = true;
    far.q_eval_deg.fill(0.0);
    buildCollisionConstraints(far, cfg, 0.0, cons, &engaged);
    RB_CHECK(cons.empty());
    JointArray ql{};
    ql[0] = 0.1 * 180.0 / 3.14159265358979323846;
    JointArray qr{};
    ConstraintBuildOptions by_q;
    by_q.left_q_deg = &ql;
    by_q.right_q_deg = &qr;
    buildCollisionConstraints(far, cfg, 0.0, cons, &engaged, by_q);
    RB_CHECK(cons.size() == 1);
    RB_CHECK(std::abs(cons[0].d_now - (-0.050)) < 1e-9);
    RB_CHECK(cons[0].xi == -cfg.recover_speed_m_s);  // below the floor: block deeper
    // Without q_eval in the verdict the option falls back to rate * age.
    cons.clear();
    engaged.clear();
    CollisionVerdict legacy = makePairVerdict(0.050, -1.0, jl, jr);  // closing 1 m/s
    buildCollisionConstraints(legacy, cfg, 0.025, cons, &engaged, by_q);
    RB_CHECK(cons.size() == 1 && std::abs(cons[0].d_now - 0.025) < 1e-9);
    std::cout << "gripper class + exclusion + q-extrapolation OK\n";
    return true;
}

// The solve sweeps to convergence. Two rows at 45 deg that each undo part of
// the other's fix converge geometrically (ratio 1/2 per sweep): three fixed
// sweeps leave a visible violation, the convergent solve does not, and it
// reports how many sweeps it took.
static bool runSolverConvergence() {
    const double dt = 0.002;
    const double rad2deg = 180.0 / 3.14159265358979323846;
    const JointArray zero{0, 0, 0, 0, 0, 0};
    const JointArray big{1e7, 1e7, 1e7, 1e7, 1e7, 1e7};
    const auto make_rows = []() {
        std::vector<VelocityConstraint> cons(2);
        cons[0].J[0] = 1.0;                       // qdot0 >= 0
        cons[0].xi = 0.0;
        cons[0].d_now = 0.001;
        const double c = std::cos(3.14159265358979323846 / 4.0);
        cons[1].J[0] = -c;                        // -c qdot0 + c qdot1 >= 0
        cons[1].J[1] = c;
        cons[1].xi = 0.0;
        cons[1].d_now = 0.002;
        return cons;
    };
    // Desired: both joints closing at -1 rad/s.
    const auto desired_target = [&]() {
        JointArray t{};
        t[0] = -1.0 * dt * rad2deg;
        t[1] = -1.0 * dt * rad2deg;
        return t;
    };
    const auto residual = [&](const std::vector<VelocityConstraint>& cons, const JointArray& lt) {
        double worst = 0.0;
        for (const auto& c : cons) {
            double ddot = 0.0;
            for (int i = 0; i < kDof; ++i) ddot += c.J[i] * (lt[i] / rad2deg / dt);
            worst = std::max(worst, -c.xi - ddot);
        }
        return worst;
    };
    {
        auto cons = make_rows();
        JointArray lt = desired_target();
        JointArray rt{};
        const auto r = solveVelocityProjection(cons, zero, zero, lt, rt, dt, 3, big);
        RB_CHECK(r.sweeps_used == 3);
        RB_CHECK(residual(cons, lt) > 1e-3);  // three sweeps: row 2 still violated
    }
    {
        auto cons = make_rows();
        JointArray lt = desired_target();
        JointArray rt{};
        VelocityProjectionOptions o;
        o.min_sweeps = 3;
        o.max_sweeps = 50;
        o.tol_rad_s = 1e-9;
        const auto r = solveVelocityProjection(cons, zero, zero, lt, rt, dt, o, big);
        RB_CHECK(r.converged);
        RB_CHECK(r.sweeps_used > 3 && r.sweeps_used <= 50);
        RB_CHECK(residual(cons, lt) < 1e-6);
        // qdot0 -> 0, qdot1 -> 0: both closing components removed, nothing else.
        RB_CHECK(std::abs(lt[0]) < 1e-6 && std::abs(lt[1]) < 1e-6);
    }
    {
        // The ceiling is applied inside the loop: a row that needs more speed than
        // the ceiling allows is left (honestly) unconverged, but never beyond it.
        auto cons = make_rows();
        cons[0].xi = 100.0;  // demands qdot0 >= -100 -> fine; make row 2 demand a lot
        cons[1].xi = -50.0;  // -c qdot0 + c qdot1 >= 50 -> qdot1 >= ~70.7 rad/s
        JointArray lt = desired_target();
        JointArray rt{};
        JointArray lim{};
        lim.fill(10.0 * rad2deg);  // 10 rad/s per joint
        VelocityProjectionOptions o;
        o.max_sweeps = 20;
        const auto r = solveVelocityProjection(cons, zero, zero, lt, rt, dt, o, lim);
        RB_CHECK(r.ceiling_clamped);
        RB_CHECK(!r.converged);
        RB_CHECK(lt[1] / rad2deg / dt <= 10.0 + 1e-9);
    }
    std::cout << "solver convergence (sweeps to tolerance, in-loop ceiling) OK\n";
    return true;
}


// The near list can no longer drop an engaged pair (2026-09-04): every pair inside
// its class band is reported regardless of max_near_pairs. With a 1 m self band
// and K = 1, the list must hold every arm<->arm / arm<->stand pair, not one.
static bool runNearBandInclusion() {
    const fs::path ws = workspaceRoot();
    CollisionMonitorConfig cfg = makeConfig(ws);
    if (!fs::is_regular_file(cfg.unified_urdf)) {
        std::cout << "SKIP (near band): unified URDF not found\n";
        return true;
    }
    cfg.swept_samples = 1;
    cfg.max_near_pairs = 1;
    cfg.d_slow_m = 10.0;  // every self pair is "in band"
    cfg.hyst_m = 0.0;
    cfg.intra_arm_d_slow_m = 0.001;  // intra-arm pairs stay out of band
    cfg.intra_arm_hyst_m = 0.0;
    cfg.gripper_gripper_d_slow_m = 10.0;
    cfg.gripper_gripper_hyst_m = 0.0;
    CollisionMonitor mon(cfg);
    const CollisionVerdict v = mon.evalOnce(kInitPose, kInitPose);
    RB_CHECK(v.valid);
    RB_CHECK(v.q_eval_valid);
    for (int i = 0; i < kDof; ++i) {
        RB_CHECK(std::abs(v.q_eval_deg[static_cast<std::size_t>(i)] - kInitPose[i]) < 1e-9);
        RB_CHECK(std::abs(v.q_eval_deg[static_cast<std::size_t>(kDof + i)] - kInitPose[i]) < 1e-9);
    }
    std::size_t self_pairs = 0, gripper_pairs = 0, intra_pairs = 0;
    for (const auto& p : v.near) {
        if (p.intra_arm) ++intra_pairs;
        else if (p.gripper_gripper) ++gripper_pairs;
        else if (!p.external && !p.external_box) ++self_pairs;
    }
    std::cout << "near band: near=" << v.near.size() << " band=" << v.near_band_count
              << " self=" << self_pairs << " gripper=" << gripper_pairs
              << " intra=" << intra_pairs << " eval_ms=" << v.eval_ms << "\n";
    RB_CHECK(v.near.size() > 100);                 // far more than K = 1
    RB_CHECK(v.near_band_count == static_cast<int>(self_pairs + gripper_pairs));
    RB_CHECK(intra_pairs <= 1);                     // at most the single nearest slot
    RB_CHECK(gripper_pairs == 1);                   // one hull per arm -> one cross pair
    // Ascending order is preserved.
    for (std::size_t i = 1; i < v.near.size(); ++i) RB_CHECK(v.near[i - 1].d_m <= v.near[i].d_m);
    // And the K-nearest semantics are intact when nothing is in band.
    cfg.d_slow_m = 0.001;
    cfg.gripper_gripper_d_slow_m = 0.001;
    cfg.max_near_pairs = 5;
    CollisionMonitor mon5(cfg);
    const CollisionVerdict v5 = mon5.evalOnce(kInitPose, kInitPose);
    RB_CHECK(v5.valid && v5.near.size() == 5 && v5.near_band_count == 0);
    std::cout << "near band inclusion OK\n";
    return true;
}

int main() {
    if (!runExternalBoxFeedLiveness()) {
        std::cerr << "test_collision_monitor (external box feed liveness) FAILED\n";
        return 1;
    }
    if (!runExternalBoxBarrierRouting()) {
        std::cerr << "test_collision_monitor (external box barrier routing) FAILED\n";
        return 1;
    }
    if (!runPairPatternMatching()) {
        std::cerr << "test_collision_monitor (pair pattern matching) FAILED\n";
        return 1;
    }
    if (!runConstraintHysteresis()) {
        std::cout << "FAIL: runConstraintHysteresis\n";
        return 1;
    }
    if (!runConstraintClassAndFloor()) {
        std::cout << "FAIL: runConstraintClassAndFloor\n";
        return 1;
    }
    if (!runProjection()) {
        std::cerr << "test_collision_monitor (projection) FAILED\n";
        return 1;
    }
    if (!runGripperClassAndExtrapolation()) {
        std::cerr << "test_collision_monitor (gripper class / extrapolation) FAILED\n";
        return 1;
    }
    if (!runSolverConvergence()) {
        std::cerr << "test_collision_monitor (solver convergence) FAILED\n";
        return 1;
    }
    if (!runNearBandInclusion()) {
        std::cerr << "test_collision_monitor (near band inclusion) FAILED\n";
        return 1;
    }
    if (!runPrefixMismatchFailsClosed()) {
        std::cerr << "prefix mismatch fail-closed test failed\n";
        return 1;
    }
    if (!runArticulatedGripper()) {
        std::cerr << "test_collision_monitor (articulated gripper) FAILED\n";
        return 1;
    }
    if (!runGroundPlane()) {
        std::cerr << "test_collision_monitor (ground_plane) FAILED\n";
        return 1;
    }
    if (!runExternalDHard()) {
        std::cerr << "test_collision_monitor (external d_hard) FAILED\n";
        return 1;
    }
    if (!runIntraArmDHard()) {
        std::cerr << "test_collision_monitor (intra-arm d_hard) FAILED\n";
        return 1;
    }
    if (!run()) {
        std::cerr << "test_collision_monitor FAILED\n";
        return 1;
    }
    std::cout << "test_collision_monitor OK\n";
    return 0;
}
