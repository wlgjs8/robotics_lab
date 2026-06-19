// Unit test for the async mesh self-collision monitor.
// Skips gracefully if the unified URDF (mo_robot_descriptions sibling repo) is absent.

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <thread>

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

int main() {
    if (!runProjection()) {
        std::cerr << "test_collision_monitor (projection) FAILED\n";
        return 1;
    }
    if (!run()) {
        std::cerr << "test_collision_monitor FAILED\n";
        return 1;
    }
    std::cout << "test_collision_monitor OK\n";
    return 0;
}
