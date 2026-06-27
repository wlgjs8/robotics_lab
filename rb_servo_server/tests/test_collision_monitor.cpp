// Unit test for the async mesh self-collision monitor.
// Skips gracefully if the unified URDF (mo_robot_descriptions sibling repo) is absent.

#include <array>
#include <chrono>
#include <cmath>
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
    const fs::path urdf_dir =
        ws / "mo_robot_descriptions/mo_robot_descriptions/robots/urdf/dual_rb3_730e";
    c.unified_urdf = (urdf_dir / "dual_rb3_730e_ver3.urdf").string();
    c.package_dirs = {urdf_dir.string()};  // so "../../../meshes" resolves
    c.pika_gripper_mesh =
        (ws / "robotics_lab/rb_servo_server/descriptions/meshes/robots/rb3_730e/visual/tool/"
              "pika_gripper.STL")
            .string();
    const char* jn[kDof] = {"base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"};
    for (int i = 0; i < kDof; ++i) {
        c.left_joints[i] = std::string("dual_rb3_730e_left_") + jn[i] + "_joint";
        c.right_joints[i] = std::string("dual_rb3_730e_right_") + jn[i] + "_joint";
    }
    return c;
}

static bool sameDistance(double a, double b) {
    if (std::isinf(a) && std::isinf(b) && std::signbit(a) == std::signbit(b)) return true;
    return std::abs(a - b) < 1e-12;
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
        glob, "dual_rb3_730e_right_link0", "dual_rb3_730e_right_link1"));
    RB_CHECK(collisionPairPatternMatches(
        glob, "dual_rb3_730e_right_link1", "dual_rb3_730e_right_link0"));
    RB_CHECK(!collisionPairPatternMatches(
        glob, "dual_rb3_730e_left_link0", "dual_rb3_730e_right_link1"));
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
    RB_CHECK(mon.numGeometries() == 33);
    RB_CHECK(mon.numPairs() > 0);

    {
        CollisionMonitorConfig all_pairs_cfg = cfg;
        all_pairs_cfg.swept_samples = 1;
        all_pairs_cfg.max_near_pairs = 10000;
        CollisionPairPattern adjacent_right{"*right*link0*", "*right*link1*"};
        CollisionMonitor all_pairs(all_pairs_cfg);
        const JointArray init = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};
        const CollisionVerdict before = all_pairs.evalOnce(init, init);
        RB_CHECK(before.valid);
        const bool adjacent_present = verdictHasPair(before, adjacent_right);
        std::cout << "right link0-link1 baseline pair present=" << adjacent_present << "\n";

        CollisionMonitorConfig disabled_cfg = all_pairs_cfg;
        disabled_cfg.disabled_collision_pairs.push_back(adjacent_right);
        CollisionMonitor disabled(disabled_cfg);
        const CollisionVerdict after = disabled.evalOnce(init, init);
        RB_CHECK(after.valid);
        RB_CHECK(!verdictHasPair(after, adjacent_right));
        if (adjacent_present) {
            RB_CHECK(disabled.numPairs() < all_pairs.numPairs());
        } else {
            // On URDFs where link0/link1 are classified as adjacent same-arm links,
            // intra-arm chain separation already excludes the pair before SRDF rules.
            RB_CHECK(disabled.numPairs() == all_pairs.numPairs());
        }

        CollisionMonitorConfig reversed_cfg = all_pairs_cfg;
        reversed_cfg.disabled_collision_pairs.push_back(
            CollisionPairPattern{"*right*link1*", "*right*link0*"});
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
    }

    const JointArray init = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};

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
        RB_CHECK(sameDistance(s.external_min_clearance_m, v.external_min_clearance_m));
        const double self_thresh = cfg.d_hard_m + 0.005;
        const double ext_thresh = cfg.external_d_hard_m + 0.005;
        const bool gate = !v.hard_violation &&
                          v.self_min_clearance_m > self_thresh &&
                          v.external_min_clearance_m > ext_thresh;
        RB_CHECK(mon.clearsThresholds(init, init, self_thresh, ext_thresh) == gate);
    }
    {
        CollisionMonitorConfig endpoint_cfg = cfg;
        endpoint_cfg.swept_samples = 1;
        CollisionMonitor endpoint(endpoint_cfg);
        const double self_thresh = endpoint_cfg.d_hard_m + 0.005;
        const double ext_thresh = endpoint_cfg.external_d_hard_m + 0.005;
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
            RB_CHECK(sameDistance(light.external_min_clearance_m, full.external_min_clearance_m));
            const bool full_gate = !full.hard_violation &&
                                   full.self_min_clearance_m > self_thresh &&
                                   full.external_min_clearance_m > ext_thresh;
            const bool light_gate = !light.hard_violation &&
                                    light.self_min_clearance_m > self_thresh &&
                                    light.external_min_clearance_m > ext_thresh;
            RB_CHECK(light_gate == full_gate);
            RB_CHECK(endpoint.clearsThresholds(sample.first, sample.second,
                                               self_thresh, ext_thresh) == full_gate);
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

    // (6) threaded publish/consume: start, submit, see a fresh verdict.
    mon.start();
    mon.submitTargets(init, init);
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
        std::cout << "evalOnce mean (with J_n) = " << ms << " ms\n";
        RB_CHECK(ms < 5.0);  // must not blow the reaction-latency budget
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

    const JointArray init = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};

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

    const JointArray init = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};

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
        ws / "robotics_lab/rb_servo_server/descriptions/meshes/robots/rb3_730e/visual/tool";
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
    // arms(24) + stand(7) + base+2fingers per arm (6) = 37 (single-hull baseline was 33).
    std::cout << "articulated geoms=" << mon.numGeometries() << "\n";
    RB_CHECK(mon.numGeometries() == 37);

    const JointArray init = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};
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

int main() {
    if (!runPairPatternMatching()) {
        std::cerr << "test_collision_monitor (pair pattern matching) FAILED\n";
        return 1;
    }
    if (!runProjection()) {
        std::cerr << "test_collision_monitor (projection) FAILED\n";
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
    if (!run()) {
        std::cerr << "test_collision_monitor FAILED\n";
        return 1;
    }
    std::cout << "test_collision_monitor OK\n";
    return 0;
}
