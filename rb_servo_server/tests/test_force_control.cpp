// test_force_control.cpp - the invariants of the ported F/T pipeline, THE ONE
// admittance law and the force gate. Every check here is a claim that would be
// expensive to discover on the robot, and several are claims controller-manager
// already paid for on hardware.
//
// 2026-09-15: the law was unified (admittance_overlay.hpp). ONE vector law
//
//     m * v' + b * v = (|F| - rest_force_n)+ * F_hat        k = 0, rotation RIGID
//
// on the translation VECTOR in the stand frame, judged and cut along the MEASURED
// force direction. The per-axis rows, the stream/hold pair, the declared press axis
// and the hold-engage latch are gone, and so are the tests that pinned them; the
// tests that replace them are dated below. Forces must now EXCEED rest_force_n
// (10 N) to move anything.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/hold_fold.hpp"
#include "rb_servo/control/admittance_overlay.hpp"
#include "rb_servo/control/preview_contact_authority.hpp"
#include "rb_servo/control/smd_pose_tracker.hpp"
#include "rb_servo/sensor/ft_pipeline.hpp"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("CHECK failed: %s at %s:%d\n", #cond, __FILE__, __LINE__); \
            ++g_failures;                                                        \
        }                                                                        \
    } while (0)

bool near(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

using rb_servo::math::Vector3;
constexpr double kDt = 0.002;

// The cell's real sensor + tool, so the numbers under test are the shipped ones.
rb_servo::FtArmConfig cellConfig() {
    rb_servo::FtArmConfig c;
    c.enable = true;
    c.sensor_offset_mm = {0.0, 0.0, 45.0};
    // THE MEASURED LEFT-HANDED TRIAD (det = -1). Every check below that depends on
    // the mapping depends on this staying exactly what controller-manager measured.
    c.axis_fx = {0.0, -1.0, 0.0};
    c.axis_fy = {1.0, 0.0, 0.0};
    c.axis_fz = {0.0, 0.0, -1.0};
    c.deadzone_force_n = {2.0, 2.0, 2.0};
    c.deadzone_torque_nm = {0.5, 0.5, 0.5};
    c.tool_load_tau_s = 1.0;
    c.tool_xyz_mm = {0.0, 0.0, 202.642};
    c.tool_rpy_deg = {0.0, 0.0, 0.0};
    c.tool_mass_kg = 0.7912;
    c.tool_com_mm = {-0.02, 3.24, 25.34};
    c.applied_force_mm = {0.0, 0.0, 202.642};
    return c;
}

// A pipeline already past the liveness check, so step() runs.
rb_servo::sensor::FtPipeline livePipeline(const rb_servo::FtArmConfig& cfg) {
    rb_servo::sensor::FtPipeline pipe;
    pipe.configure(cfg, kDt);
    rb_servo::Wrench6D a{};
    a.fx = 0.0;
    rb_servo::Wrench6D b{};
    b.fx = 1.0;   // varies above the noise floor -> connected
    pipe.livenessSample(a);
    pipe.livenessSample(b);
    pipe.livenessDecide();
    return pipe;
}

rb_servo::sensor::FtPipelineInput input(const rb_servo::Wrench6D& raw,
                                        const rb_servo::math::Matrix3& r_stand_flange) {
    rb_servo::sensor::FtPipelineInput in;
    in.raw_sensor_axes = raw;
    in.raw_valid = true;
    in.r_stand_flange = r_stand_flange;
    in.kinematics_valid = true;
    return in;
}

// ---------------------------------------------------------------------------
// THE F/T PIPELINE (unchanged by the 2026-09-15 law unification)
// ---------------------------------------------------------------------------

// THE AXIS MAP IS A BASIS, NOT A ROTATION. This pins the exact permutation and sign
// controller-manager converged on the cell: a sensor +X reading must come out along
// flange -Y, +Y along flange +X, and +Z along flange -Z. Get any of these wrong and
// the arm complies in a direction nobody pushed.
bool testAxisMapIsTheMeasuredLeftHandedBasis() {
    rb_servo::FtArmConfig cfg = cellConfig();
    cfg.tool_mass_kg = 0.0;              // isolate the mapping from gravity
    cfg.tool_com_mm = {0.0, 0.0, 0.0};
    cfg.deadzone_force_n = {0.0, 0.0, 0.0};
    cfg.deadzone_torque_nm = {0.0, 0.0, 0.0};
    auto pipe = livePipeline(cfg);

    CHECK(near(pipe.axesDeterminant(), -1.0, 1e-12));

    rb_servo::Wrench6D raw{};
    raw.fx = 10.0;
    CHECK(pipe.step(input(raw, rb_servo::math::Matrix3::Identity())));
    CHECK(near(pipe.rawSensor().fx, 0.0));
    CHECK(near(pipe.rawSensor().fy, -10.0));
    CHECK(near(pipe.rawSensor().fz, 0.0));

    raw = rb_servo::Wrench6D{};
    raw.fy = 10.0;
    CHECK(pipe.step(input(raw, rb_servo::math::Matrix3::Identity())));
    CHECK(near(pipe.rawSensor().fx, 10.0));
    CHECK(near(pipe.rawSensor().fy, 0.0));

    raw = rb_servo::Wrench6D{};
    raw.fz = 10.0;
    CHECK(pipe.step(input(raw, rb_servo::math::Matrix3::Identity())));
    CHECK(near(pipe.rawSensor().fz, -10.0));
    return true;
}

// GRAVITY IS SUBTRACTED IN FULL, because the box is told a zero payload. With the
// flange pointing up, a tool of mass m must show -m*g along sensor Z in the raw
// channel and EXACTLY ZERO in the compensated one.
bool testFullToolGravityIsSubtracted() {
    rb_servo::FtArmConfig cfg = cellConfig();
    cfg.axis_fx = {1.0, 0.0, 0.0};        // identity map: isolate gravity from the basis
    cfg.axis_fy = {0.0, 1.0, 0.0};
    cfg.axis_fz = {0.0, 0.0, 1.0};
    cfg.tool_com_mm = {0.0, 0.0, 0.0};    // no lever: isolate force from torque
    cfg.deadzone_force_n = {0.0, 0.0, 0.0};
    cfg.deadzone_torque_nm = {0.0, 0.0, 0.0};
    auto pipe = livePipeline(cfg);

    const double weight = cfg.tool_mass_kg * 9.80665;
    rb_servo::Wrench6D raw{};
    raw.fz = -weight;                     // what a sensor reads holding that tool
    CHECK(pipe.step(input(raw, rb_servo::math::Matrix3::Identity())));
    CHECK(near(pipe.gravitySensor().fz, -weight, 1e-9));
    // The whole reading was gravity, so nothing is left.
    CHECK(near(pipe.compSensorNoDeadzone().fz, 0.0, 1e-9));
    return true;
}

// TOOL INERTIA IS SUBTRACTED BESIDE GRAVITY (2026-09-15): a tool accelerating at a
// reads -m*a on the sensor exactly as it reads m*g, so with the COMMANDED COM
// acceleration supplied the compensated force of a free-flying tool is zero, and
// without it the whole inertial reaction stands - the 12 N the first isotropic-law
// policy run mistook for contact (servo_log_20260915_153420).
bool testToolInertiaIsSubtractedWithGravity() {
    rb_servo::FtArmConfig cfg = cellConfig();
    cfg.axis_fx = {1.0, 0.0, 0.0};
    cfg.axis_fy = {0.0, 1.0, 0.0};
    cfg.axis_fz = {0.0, 0.0, 1.0};
    cfg.tool_com_mm = {0.0, 0.0, 0.0};
    cfg.deadzone_force_n = {0.0, 0.0, 0.0};
    cfg.deadzone_torque_nm = {0.0, 0.0, 0.0};
    auto pipe = livePipeline(cfg);
    const double m = cfg.tool_mass_kg;
    const rb_servo::math::Vector3 a(3.0, 0.0, 5.0);          // stand frame, m/s^2
    rb_servo::Wrench6D raw{};
    raw.fx = -m * a.x();                                      // the tool resists +x with -m*a
    raw.fz = -m * 9.80665 - m * a.z();                        // weight plus the +z reaction
    // Without the acceleration the pipeline sees -m*a as an external push.
    CHECK(pipe.step(input(raw, rb_servo::math::Matrix3::Identity())));
    CHECK(near(pipe.compSensorNoDeadzone().fx, -m * a.x(), 1e-9));
    CHECK(near(pipe.compSensorNoDeadzone().fz, -m * a.z(), 1e-9));
    CHECK(near(pipe.inertialSensor().fx, 0.0, 1e-12));
    // With it the whole reading is gravity + inertia, so nothing is left.
    auto in = input(raw, rb_servo::math::Matrix3::Identity());
    in.com_accel_stand = a;
    in.inertia_valid = true;
    CHECK(pipe.step(in));
    CHECK(near(pipe.inertialSensor().fx, -m * a.x(), 1e-9));
    CHECK(near(pipe.inertialSensor().fz, -m * a.z(), 1e-9));
    CHECK(near(pipe.compSensorNoDeadzone().fx, 0.0, 1e-9));
    CHECK(near(pipe.compSensorNoDeadzone().fz, 0.0, 1e-9));
    // A non-finite acceleration is refused, never subtracted.
    in.com_accel_stand = rb_servo::math::Vector3(std::nan(""), 0.0, 0.0);
    CHECK(pipe.step(in));
    CHECK(near(pipe.inertialSensor().fx, 0.0, 1e-12));
    return true;
}

// A TARE AVERAGES `raw - gravity`, NEVER `raw`. Averaging raw would fold the tare
// pose's gravity into the bias and then step() would subtract gravity a SECOND time,
// leaving a standing force of one tool weight that looks exactly like contact.
bool testTareDoesNotDoubleSubtractGravity() {
    rb_servo::FtArmConfig cfg = cellConfig();
    cfg.axis_fx = {1.0, 0.0, 0.0};
    cfg.axis_fy = {0.0, 1.0, 0.0};
    cfg.axis_fz = {0.0, 0.0, 1.0};
    cfg.tool_com_mm = {0.0, 0.0, 0.0};
    cfg.deadzone_force_n = {0.0, 0.0, 0.0};
    cfg.deadzone_torque_nm = {0.0, 0.0, 0.0};
    auto pipe = livePipeline(cfg);

    const double weight = cfg.tool_mass_kg * 9.80665;
    const double offset = 3.0;            // a real sensor offset, on top of gravity
    rb_servo::Wrench6D raw{};
    raw.fz = -weight + offset;

    for (int i = 0; i < 250; ++i) {
        CHECK(pipe.step(input(raw, rb_servo::math::Matrix3::Identity())));
        pipe.tareSample();
    }
    std::string reason;
    CHECK(pipe.tareCommit(250, &reason));
    CHECK(pipe.biasValid());
    CHECK(pipe.biasSource() == "tare");
    // The bias must be the OFFSET alone. If tareSample had averaged raw, this would
    // be `offset - weight` and the next line would show a standing -7.76 N.
    CHECK(near(pipe.bias().fz, offset, 1e-9));

    CHECK(pipe.step(input(raw, rb_servo::math::Matrix3::Identity())));
    CHECK(near(pipe.compSensorNoDeadzone().fz, 0.0, 1e-9));
    return true;
}

// A SENSOR THAT IS NOT THERE MUST READ EXACTLY ZERO, not a bias- and
// gravity-derived number. Zero is the one value force logic treats as "nothing is
// being felt"; anything else is a force nobody measured.
bool testDisconnectedSensorPinsCompensatedChannelsToZero() {
    rb_servo::FtArmConfig cfg = cellConfig();
    rb_servo::sensor::FtPipeline pipe;
    pipe.configure(cfg, kDt);
    // A stream that arrives but never varies: unplugged or frozen.
    rb_servo::Wrench6D flat{};
    flat.fz = -7.76;
    for (int i = 0; i < 10; ++i) pipe.livenessSample(flat);
    CHECK(!pipe.livenessDecide());
    CHECK(!pipe.connected());

    CHECK(!pipe.step(input(flat, rb_servo::math::Matrix3::Identity())));
    CHECK(near(pipe.compSensor().fz, 0.0));
    CHECK(near(pipe.compTcp().fz, 0.0));
    CHECK(near(pipe.compStand().fz, 0.0));
    return true;
}

// THE DEADZONE IS SOFT AND CONTINUOUS. A hard band would step by the band width at
// the threshold, which a force law reads as an impulse.
bool testDeadzoneIsContinuous() {
    rb_servo::FtArmConfig cfg = cellConfig();
    cfg.axis_fx = {1.0, 0.0, 0.0};
    cfg.axis_fy = {0.0, 1.0, 0.0};
    cfg.axis_fz = {0.0, 0.0, 1.0};
    cfg.tool_mass_kg = 0.0;
    cfg.tool_com_mm = {0.0, 0.0, 0.0};
    auto pipe = livePipeline(cfg);

    rb_servo::Wrench6D raw{};
    raw.fx = 2.0;                          // exactly at the band edge
    CHECK(pipe.step(input(raw, rb_servo::math::Matrix3::Identity())));
    CHECK(near(pipe.compSensor().fx, 0.0, 1e-12));
    raw.fx = 2.001;
    CHECK(pipe.step(input(raw, rb_servo::math::Matrix3::Identity())));
    CHECK(near(pipe.compSensor().fx, 0.001, 1e-12));
    raw.fx = 5.0;
    CHECK(pipe.step(input(raw, rb_servo::math::Matrix3::Identity())));
    CHECK(near(pipe.compSensor().fx, 3.0, 1e-12));
    return true;
}

bool testExternallyVerifiedSensorDoesNotGrantTareOrAcceptInvalidWrench() {
    auto cfg = cellConfig();
    cfg.tool_mass_kg = 0.0;
    auto pipe = rb_servo::sensor::FtPipeline{};
    pipe.configure(cfg, kDt);
    pipe.setExternallyVerifiedConnection(true);
    CHECK(pipe.connected());
    CHECK(!pipe.biasValid());
    auto in = input(rb_servo::Wrench6D{0,10,0,0,0,0}, rb_servo::math::Matrix3::Identity());
    CHECK(pipe.step(in));
    CHECK(near(pipe.rawSensor().fx, 10.0));
    in.raw_valid = false;
    CHECK(!pipe.step(in));
    CHECK(near(pipe.compTcp().fx, 0.0));
    in.raw_valid = true;
    pipe.setExternallyVerifiedConnection(false);
    CHECK(!pipe.step(in));
    CHECK(!pipe.biasValid());
    return true;
}

// ---------------------------------------------------------------------------
// THE ONE LAW AND ITS GATE (2026-09-15)
// ---------------------------------------------------------------------------

// THE FOLLOWER'S CUT, as a test helper: remove (1-g) of the advance's projection onto
// the MEASURED F_hat (ForceGate::contactNormal), and only when that projection is INTO
// the contact (advance . F_hat < 0; +F_hat is the force ON the tool, i.e. the free-space
// direction). This is the one rule every live consumer applies
// (CartesianChunkFollower::setAdvanceGate, the pose-track hold, the preview QP); the
// gate itself carries no apply, so a test that wants the rule states it.
Vector3 cutAlong(const Vector3& advance, const Vector3& normal, double gate, double* removed) {
    if (removed != nullptr) *removed = 0.0;
    if (normal.isZero(0.0) || gate >= 1.0) return advance;
    const double proj = advance.dot(normal);
    if (proj >= 0.0) return advance;
    const Vector3 cut = (1.0 - gate) * proj * normal;
    if (removed != nullptr) *removed = cut.norm();
    return advance - cut;
}

// THE ONE LAW, as the loader would install it from stack_real.yaml: m 20 kg, the gate
// pair 12 / 10 N at 4 mm/s, and b DERIVED from that pair - (12 - 10) N / 4 mm/s =
// 500 N*s/m. A unit test builds the config by hand, so it must set law.b itself; the
// gate reads the same field (bEff() == law.b) so the crossing cannot drift from the
// law. Rotation is rigid by construction (no rotation config exists any more). The
// wrench filter is the shipped 25 Hz where a harness below models the loop's filter.
rb_servo::ForceControlConfig oneLaw() {
    rb_servo::ForceControlConfig c;
    c.enable = true;
    c.law.m = 20.0;
    c.law.b = 500.0;
    c.gate_enable = true;
    c.gate_peak_force_n = 12.0;
    c.gate_rest_force_n = 10.0;
    c.gate_peak_vel_mm_s = 4.0;
    c.gate_close_tau_s = 0.10;
    c.gate_open_tau_s = 1.0;     // 0.40 -> 1.0 with k = 0: a fast re-open feeds the ring
    c.max_deviation_m = 0.040;
    c.max_deviation_rad = 0.2617993878;
    c.wrench_filter_hz = 25.0;
    return c;
}

// The law's yield distance from rest under a constant excess for t seconds (the
// continuous closed form; the Euler integration below differs by < 0.05 mm at 20 mm/s):
// v_ss = excess/b, tau = m/b, d = v_ss * (t - tau * (1 - exp(-t/tau))).
double yieldDistance(const rb_servo::ForceControlConfig& c, double excess_n, double t_sec) {
    const double v_ss = excess_n / c.law.b;
    const double tau = c.law.m / c.law.b;
    return v_ss * (t_sec - tau * (1.0 - std::exp(-t_sec / tau)));
}

// A STEADY FORCE YIELDS THE EXCESS OVER rest_force_n AT (|F| - rest)/b, AND THAT IS THE
// NUMBER AN OPERATOR CAN CHECK WITH A RULER: 20 N for 1 s must walk 20 mm minus the
// m/b ramp (19.2 mm). Replaces the F/k spring check on 2026-09-15 - there is no spring.
bool testSteadyForceYieldsTheExcessOverRest() {
    const rb_servo::ForceControlConfig cfg = oneLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, kDt);
    const Vector3 f(0.0, 0.0, 20.0);
    const Vector3 m = Vector3::Zero();
    for (int i = 0; i < 500; ++i) overlay.step(f, m);   // 1 s
    const double want = yieldDistance(cfg, 20.0 - cfg.gate_rest_force_n, 1.0);
    std::printf("  20 N for 1 s: yielded %.3f mm (closed form %.3f mm, steady %.1f mm/s)\n",
                overlay.deviation().z() * 1e3, want * 1e3, overlay.velocity().z() * 1e3);
    CHECK(near(overlay.deviation().z(), want, 1e-4));
    CHECK(near(overlay.velocity().z(), (20.0 - cfg.gate_rest_force_n) / cfg.law.b, 1e-5));
    // Along the force only.
    CHECK(overlay.deviation().x() == 0.0 && overlay.deviation().y() == 0.0);
    CHECK(!overlay.bounded());
    return true;
}

// THE FENCE HOLDS AND SAYS SO. Past the bound the deviation's NORM is clamped, the
// velocity along the clamped direction is zeroed so nothing winds up against it, and
// `bounded()` latches for the caller to publish. Gate off: this is the dead backstop of
// the absolute-target path (the fold makes it unreachable everywhere else).
bool testFenceClampsAndReportsSaturation() {
    rb_servo::ForceControlConfig cfg = oneLaw();
    cfg.gate_enable = false;
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, kDt);
    const Vector3 f(0.0, 0.0, 200.0);
    const Vector3 m = Vector3::Zero();
    for (int i = 0; i < 20000; ++i) overlay.step(f, m);
    CHECK(overlay.bounded());
    CHECK(overlay.deviation().norm() <= cfg.max_deviation_m + 1e-9);
    CHECK(overlay.deviation().norm() > cfg.max_deviation_m - 1e-6);
    // No outward momentum survives the clamp.
    CHECK(overlay.velocity().dot(overlay.deviation().normalized()) <= 1e-12);
    return true;
}

// ROTATION IS RIGID (2026-09-15): whatever torque is applied, the rotational deviation
// and its rate are exactly zero while the translation yields - and a torque alone
// moves nothing at all. Replaces the per-axis Rigid-mode check.
bool testRotationIsRigid() {
    const rb_servo::ForceControlConfig cfg = oneLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, kDt);
    const Vector3 f(0.0, 0.0, 14.0);
    const Vector3 torque(5.0, 0.0, 0.0);
    for (int i = 0; i < 1000; ++i) overlay.step(f, torque);
    CHECK(overlay.deviationRot().norm() == 0.0);
    CHECK(overlay.velocityRot().norm() == 0.0);
    CHECK(overlay.deviation().z() > 1e-3);                 // the translation did yield
    // A torque with no force: nothing.
    rb_servo::control::AdmittanceOverlay twist;
    twist.configure(cfg, kDt);
    for (int i = 0; i < 1000; ++i) twist.step(Vector3::Zero(), torque);
    CHECK(twist.deviation().norm() == 0.0);
    CHECK(!twist.hasDeviation());
    // compose() leaves the orientation untouched.
    rb_servo::Pose6D nominal;
    nominal.x = 0.5; nominal.rx = 3.0; nominal.ry = -0.1; nominal.rz = 1.5;
    const rb_servo::Pose6D emitted = overlay.compose(nominal);
    CHECK(emitted.rx == nominal.rx && emitted.ry == nominal.ry && emitted.rz == nominal.rz);
    CHECK(near(emitted.z, nominal.z + overlay.deviation().z(), 1e-15));
    return true;
}

// LEAVING SERVICE FREEZES THE DISPLACEMENT AND DROPS THE MOMENTUM. It must NOT walk
// the deviation back: under contact the nominal is inside the workpiece, so retiring
// would command the tool the whole deviation deeper.
bool testFreezeKeepsTheDeviationAndDropsVelocity() {
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(oneLaw(), kDt);
    const Vector3 f(0.0, 0.0, 14.0);
    const Vector3 m = Vector3::Zero();
    for (int i = 0; i < 200; ++i) overlay.step(f, m);
    const double held = overlay.deviation().z();
    CHECK(held > 1e-5);
    CHECK(overlay.velocity().norm() > 1e-6);
    overlay.freeze();
    CHECK(near(overlay.deviation().z(), held, 1e-12));
    CHECK(near(overlay.velocity().norm(), 0.0, 1e-12));
    return true;
}

// *** THE OVERLAY MUST NOT WIND AGAINST A COMMAND THAT IS NOT REACHING THE ROBOT. ***
//
// It is an open-loop integrator on the measured wrench: if its output never lands,
// the wrench never answers, and it winds until something else stops it. Measured
// 2026-08-26 - the servo stream deadlocked in queue-sync warmup, the arm never moved,
// a hand stayed on the tool, and the deviation wound the command 54 deg out before
// the tracking latch fired on a fault that named the wrong subsystem.
//
// FREEZING is the right answer, not resetting: the deviation already on the wire is
// the pose the arm is holding, and walking it back would command the tool through
// whatever it is resting against.
bool testFreezeHoldsTheDeviationWhileTheCommandIsNotExecuted() {
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(oneLaw(), kDt);
    // A steady contact winds the deviation up (14 N: 8 mm/s, 4 s -> ~32 mm).
    const Vector3 f(0.0, 0.0, 14.0);
    const Vector3 m = Vector3::Zero();
    for (int i = 0; i < 2000; ++i) overlay.step(f, m);
    const double held = overlay.deviation().z();
    CHECK(held > 1e-4);
    CHECK(!overlay.bounded());
    // The stream stops reaching the robot. The caller stops stepping and freezes.
    overlay.freeze();
    CHECK(near(overlay.deviation().z(), held, 1e-12));       // the pose on the wire is kept
    CHECK(near(overlay.velocity().norm(), 0.0, 1e-12));      // the momentum is stale
    // The same wrench with nobody stepping it: the deviation must not have moved.
    CHECK(near(overlay.deviation().z(), held, 1e-12));
    return true;
}

// A frozen overlay that resumes must pick the deviation back up where it left it,
// not restart from zero - restarting would snap the emitted command off the pose the
// arm is holding.
bool testResumeContinuesFromTheFrozenDeviation() {
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(oneLaw(), kDt);
    const Vector3 f(0.0, 0.0, 14.0);
    const Vector3 m = Vector3::Zero();
    for (int i = 0; i < 500; ++i) overlay.step(f, m);
    const double held = overlay.deviation().z();
    CHECK(held > 1e-4);
    overlay.freeze();
    overlay.step(f, m);                            // one tick after the resume
    // It moved on from `held`, it did not restart at 0.
    CHECK(overlay.deviation().z() > held * 0.9);
    CHECK(overlay.deviation().z() >= held);
    return true;
}

// THE GATE IS PROJECTIVE, ALONG THE MEASURED F_hat (2026-09-15). Scaling the whole
// advance would kill sliding along a contact AND throttle backing out of it, which is
// the escape an operator needs. A tilted force (3, 0, 20) gives a tilted normal, and the
// tangent plane of THAT normal is what stays free.
bool testGateAttenuatesOnlyIntoTheContact() {
    rb_servo::control::ForceGate gate;
    gate.configure(oneLaw(), kDt);
    // Drive the gate shut against a tilted reaction with a stream far above the
    // crossing speed, where the curve's ratio is ~1e-5.
    const Vector3 f(3.0, 0.0, 20.0);
    const Vector3 m = Vector3::Zero();
    for (int i = 0; i < 5000; ++i) gate.update(f, m, 0.200);
    CHECK(gate.translation() < 0.02);
    const Vector3 normal = gate.contactNormal();
    CHECK((normal - f.normalized()).norm() < 1e-12);

    double removed = 0.0;
    // INTO the contact (against the force on the tool) -> cut to the law's share.
    const Vector3 into = cutAlong(-0.001 * normal, normal, gate.translation(), &removed);
    CHECK(into.norm() < 1e-4);
    CHECK(removed > 0.0009);
    // TANGENTIAL (perpendicular to F_hat, not to a stand axis) -> untouched.
    const Vector3 tangent = Vector3(20.0, 0.0, -3.0).normalized() * 0.001;   // . (3,0,20) = 0
    const Vector3 tang = cutAlong(tangent, normal, gate.translation(), &removed);
    CHECK((tang - tangent).norm() < 1e-12);
    CHECK(removed < 1e-15);
    const Vector3 side(0.0, 0.001, 0.0);
    CHECK((cutAlong(side, normal, gate.translation(), &removed) - side).norm() < 1e-12);
    CHECK(removed < 1e-15);
    // RETREATING (along the force on the tool) -> untouched, at full authority.
    const Vector3 out = cutAlong(0.001 * normal, normal, gate.translation(), &removed);
    CHECK((out - 0.001 * normal).norm() < 1e-12);
    CHECK(removed == 0.0);
    return true;
}

// FAST TO CLOSE, SLOW TO OPEN. A gate that re-opens as fast as it closes becomes a
// relay against the contact and sustains a limit cycle.
//
// COMPARED AS A PER-TICK RATE OVER THE SAME GAP, not as time-to-cross-a-threshold:
// the two directions start from different distances, so crossing times say nothing
// about the time constants. This is the comparison that is actually about tau.
bool testGateIsAsymmetric() {
    rb_servo::ForceControlConfig cfg = oneLaw();
    cfg.gate_close_tau_s = 0.10;
    cfg.gate_open_tau_s = 0.40;
    const Vector3 m = Vector3::Zero();
    const Vector3 hard(0.0, 0.0, 20.0);          // curve -> ~2e-5 at 200 mm/s
    const Vector3 none = Vector3::Zero();        // curve -> 1

    // ONE tick of closing from a fully open gate: gap ~1.0.
    rb_servo::control::ForceGate closing;
    closing.configure(cfg, kDt);
    closing.update(hard, m, 0.200);
    const double close_step = 1.0 - closing.translation();

    // ONE tick of opening from a fully closed gate: the same gap of ~1.0.
    rb_servo::control::ForceGate opening;
    opening.configure(cfg, kDt);
    for (int i = 0; i < 5000; ++i) opening.update(hard, m, 0.200);  // -> ~0
    const double closed = opening.translation();
    CHECK(closed < 1e-3);
    opening.update(none, m, 0.0);
    const double open_step = opening.translation() - closed;

    CHECK(close_step > 0.0);
    CHECK(open_step > 0.0);
    // tau 0.10 vs 0.40 -> closing moves 4x per tick over an equal gap.
    CHECK(close_step > open_step * 3.5);
    return true;
}

// g(0) = 1 EXACTLY, whatever the demand: free space costs the plan nothing, and with
// no force there is no normal to cut along.
bool testGateIsOpenInFreeSpace() {
    rb_servo::control::ForceGate gate;
    gate.configure(oneLaw(), kDt);
    const Vector3 zero = Vector3::Zero();
    for (int i = 0; i < 1000; ++i) gate.update(zero, zero, 0.200);
    CHECK(gate.translation() == 1.0);
    CHECK(gate.contactNormal().isZero(0.0));
    CHECK(gate.forceDirection().isZero(0.0));
    double removed = 1.0;
    const Vector3 adv(0.0, 0.0, -0.001);
    CHECK((cutAlong(adv, gate.contactNormal(), gate.translation(), &removed) - adv).norm() == 0.0);
    CHECK(removed == 0.0);
    return true;
}

// A RELEASED GATE RE-OPENS TO EXACTLY 1.0. A first-order slew only approaches 1;
// without the snap a gate that closed once stayed at 0.9999... and its nanometre
// "cuts" kept invoking the tracker's hold (2026-09-04 22:32).
bool testGateReopensToExactlyOneAfterRelease() {
    rb_servo::ForceControlConfig cfg = oneLaw();
    cfg.gate_open_tau_s = 1.0;
    rb_servo::control::ForceGate gate;
    gate.configure(cfg, kDt);
    const Vector3 zero = Vector3::Zero();
    const Vector3 push(0.0, 0.0, 20.0);
    for (int i = 0; i < 500; ++i) gate.update(push, zero, 0.200);
    CHECK(gate.translation() < 0.02);
    // Release and wait 15 tau: the exponential alone would sit at 1 - 3e-7.
    for (int i = 0; i < 7500; ++i) gate.update(zero, zero, 0.0);
    CHECK(gate.translation() == 1.0);
    double removed = 1.0;
    cutAlong(Vector3(0.0, 0.0, -0.001), Vector3(0.0, 0.0, 1.0), gate.translation(), &removed);
    CHECK(removed == 0.0);
    return true;
}

// THE HOLD FOLD DELTA: the whole plan-vs-sent shortfall, with a noise floor below
// which nothing is booked and a snap cap above which the fold is declined.
// (hold_fold.hpp is the PLAN fold on a safety hold - unrelated to the force fold.)
bool testHoldFoldDeltaFloorAndCap() {
    rb_servo::control::HoldFoldLimits lim;
    lim.min_step_m = 1e-5;
    lim.min_step_rad = 1e-5;
    lim.max_step_m = 0.03;
    lim.max_step_rad = 0.2;
    rb_servo::Pose6D emitted;
    emitted.x = 0.5; emitted.y = -0.2; emitted.z = 0.1; emitted.rz = 0.3;
    rb_servo::control::HoldFoldDelta d;
    bool capped = true;
    // Identical poses: nothing to book, not capped.
    CHECK(!rb_servo::control::computeHoldFold(emitted, emitted, lim, &d, &capped));
    CHECK(!capped);
    // An IK residual of 2 um: below the floor.
    rb_servo::Pose6D tiny = emitted;
    tiny.x += 2e-6;
    CHECK(!rb_servo::control::computeHoldFold(emitted, tiny, lim, &d, &capped));
    // A 12 mm hold: booked as achieved - emitted.
    rb_servo::Pose6D held = emitted;
    held.x -= 0.012;
    CHECK(rb_servo::control::computeHoldFold(emitted, held, lim, &d, &capped));
    CHECK(near(d.dp.x(), -0.012, 1e-12));
    CHECK(near(d.dist_m, 0.012, 1e-12));
    CHECK(d.angle_rad < 1e-9);
    // A rotation-only shortfall of 0.05 rad about z: dR left-composes emitted into achieved.
    rb_servo::Pose6D turned = emitted;
    turned.rz += 0.05;
    CHECK(rb_servo::control::computeHoldFold(emitted, turned, lim, &d, &capped));
    CHECK(near(d.angle_rad, 0.05, 1e-9));
    const rb_servo::math::Matrix3 back = d.dR.toRotationMatrix() * rb_servo::math::rotationFromPose(emitted);
    CHECK(near((back - rb_servo::math::rotationFromPose(turned)).norm(), 0.0, 1e-9));
    // A 50 mm "shortfall" is a snap somewhere else: declined and flagged.
    rb_servo::Pose6D snap = emitted;
    snap.y += 0.05;
    CHECK(!rb_servo::control::computeHoldFold(emitted, snap, lim, &d, &capped));
    CHECK(capped);
    return true;
}

// THE OSCILLATION GUARD TRIPS ON A LIMIT CYCLE AND NOT ON A PUSH (2026-08-27; driven
// through TRANSLATION since 2026-09-15 - rotation is rigid, so the only part that can
// reverse is the translation velocity). Amplitude caps bound the per-tick motion but
// cannot see a sustained oscillation; the guard counts velocity-direction reversals
// above 0.35 x the 0.5 m/s cap = 0.175 m/s. A steady push has zero reversals; an
// alternating drive must freeze compliance within the window, hold the deviation, and
// release only after the wrench has been quiet for the release window.
//
// +/-150 N at a 100-tick period. +/-100 N does NOT reach the floor on the reverse
// swing: the yield line gives 0.18 m/s, but the 5 m/s^2 acceleration cap and the 40 ms
// law lag cap the swing at ~0.15 m/s inside 50 ticks (checked against the discrete
// law), so the guard - correctly - never sees a reversal at amplitude.
bool testOscillationGuardTripsFreezesAndReleases() {
    rb_servo::ForceControlConfig cfg = oneLaw();
    cfg.gate_enable = false;
    cfg.max_deviation_m = 0.0;   // no fence: the GUARD is what stops this, not the clamp
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, kDt);
    const Vector3 zero = Vector3::Zero();

    // A steady 150 N push: large, ONE direction, well above the amplitude floor.
    // Never trips.
    for (int i = 0; i < 1500; ++i) overlay.step(Vector3(150.0, 0.0, 0.0), zero);
    CHECK(overlay.velocity().x() > cfg.oscillation_min_velocity_frac * cfg.max_velocity_m_s);
    CHECK(overlay.oscillationTrips() == 0);
    CHECK(!overlay.oscillationFrozen());
    overlay.reset();

    // The incident's shape, on the translation: +/-150 N along x, 50 ticks per sign.
    int tripped_at = -1;
    for (int i = 0; i < 2000; ++i) {
        const double sign = ((i / 50) % 2 == 0) ? 1.0 : -1.0;
        overlay.step(Vector3(sign * 150.0, 0.0, 0.0), zero);
        if (overlay.oscillationFrozen()) {
            tripped_at = i;
            break;
        }
    }
    std::printf("  oscillation guard: tripped at tick %d (window %.0f ticks)\n", tripped_at,
                cfg.oscillation_window_sec / kDt);
    CHECK(tripped_at >= 0);
    CHECK(overlay.oscillationTrips() == 1);
    CHECK(overlay.velocity().norm() == 0.0);
    const Vector3 frozen_dev = overlay.deviation();

    // While frozen and still under load, the deviation must NOT move (no
    // integration) and the guard must NOT release (the wrench is not quiet).
    for (int i = 0; i < 500; ++i) overlay.step(Vector3(150.0, 0.0, 0.0), zero);
    CHECK(overlay.oscillationFrozen());
    CHECK((overlay.deviation() - frozen_dev).norm() == 0.0);

    // Quiet wrench (under the 5 N release threshold) for the release window:
    // compliance rejoins.
    const int release_ticks = static_cast<int>(cfg.oscillation_release_quiet_sec / kDt) + 10;
    for (int i = 0; i < release_ticks; ++i) overlay.step(Vector3(4.0, 0.0, 0.0), zero);
    CHECK(!overlay.oscillationFrozen());
    // ... and a steady push integrates again along the law.
    overlay.reset();
    for (int i = 0; i < 2000; ++i) overlay.step(Vector3(0.0, 0.0, 14.0), zero);
    CHECK(overlay.deviation().z() > 0.02);     // 8 mm/s x 4 s minus the ramp
    CHECK(overlay.oscillationTrips() == 1);    // and not re-tripped
    return true;
}

// THE CONTACT-SHOCK LOW-PASS TAKES THE BURST, NOT THE CONTACT.
// A real contact arrives as a burst: measured 2026-08-27 the compensated |F|
// swung 0 -> 98.6 N and back every few ticks (53.8 N inside one 2 ms tick), and
// the overlay followed it into 1,501 deg/s^2 of commanded acceleration. The
// filter has to flatten that while leaving the STEADY force the law regulates
// against exactly where it was. Modelled here on the deviation the law produces,
// which is what actually reaches the robot.
bool testWrenchFilterFlattensShockAndKeepsSteadyForce() {
    const double dt = kDt;
    const double hz = 25.0;
    const double alpha = dt / (1.0 / (2.0 * M_PI * hz) + dt);
    const auto lowpass = [&](double state, double x) { return state + alpha * (x - state); };
    const rb_servo::ForceControlConfig law = oneLaw();

    // 1) A STEADY force is untouched: 20 N through the filter still yields at
    //    (20 - 10)/b = 20 mm/s; the filter only delays the onset by its 6.4 ms.
    {
        rb_servo::control::AdmittanceOverlay overlay;
        overlay.configure(law, dt);
        double s = 0.0;
        const Vector3 m = Vector3::Zero();
        for (int i = 0; i < 500; ++i) {
            s = lowpass(s, 20.0);
            overlay.step(Vector3(0.0, 0.0, s), m);
        }
        CHECK(near(overlay.velocity().z(), 10.0 / law.law.b, 1e-5));
        const double want = yieldDistance(law, 10.0, 1.0);
        CHECK(overlay.deviation().z() < want);                 // late by the filter...
        CHECK(overlay.deviation().z() > want - 3e-4);           // ... by well under 0.3 mm
    }

    // 2) THE SHOCK ITSELF. The incident's signature was the per-tick jump:
    //    53.8 N inside one 2 ms tick. That is what the filter has to take out,
    //    and it is measured on the wrench, not through the law.
    {
        double s = 0.0;
        double raw_jump = 0.0;
        double filt_jump = 0.0;
        double prev_raw = 0.0;
        double prev_s = 0.0;
        for (int i = 0; i < 4000; ++i) {
            const double raw = ((i / 3) % 2 == 0) ? 98.6 : 0.0;
            s = lowpass(s, raw);
            if (i > 100) {
                raw_jump = std::max(raw_jump, std::abs(raw - prev_raw));
                filt_jump = std::max(filt_jump, std::abs(s - prev_s));
            }
            prev_raw = raw;
            prev_s = s;
        }
        std::printf("  shock: max |dF| per tick raw=%.1f N filtered=%.1f N (%.1fx)\n",
                    raw_jump, filt_jump, raw_jump / std::max(filt_jump, 1e-9));
        CHECK(raw_jump > 50.0);              // the incident's 53.8 N/tick
        CHECK(filt_jump < 0.25 * raw_jump);  // and the law no longer sees it
    }

    // 3) Through the LAW, inside the fence, the burst reaches the deviation smaller.
    //    The burst alternates 12 <-> 24 N: BOTH above rest, so the one-sided law is
    //    in its linear regime on every tick and what is measured is the filter's own
    //    attenuation of the burst, not the rest threshold clipping half the raw drive
    //    (a 0 <-> 24 N burst straddling rest showed only 1.8x for that reason). The
    //    mean 16 mm/s over 2 s stays under the 40 mm fence, so this measures the
    //    filter and not the clamp.
    const auto peak_deviation_rate = [&](bool filtered) {
        rb_servo::ForceControlConfig cfg = oneLaw();
        cfg.gate_enable = false;          // isolate the law from the gate
        rb_servo::control::AdmittanceOverlay overlay;
        overlay.configure(cfg, dt);
        const Vector3 m = Vector3::Zero();
        double s = 0.0;
        double worst = 0.0;
        double prev_v = 0.0;
        for (int i = 0; i < 1000; ++i) {
            const double raw = ((i / 3) % 2 == 0) ? 24.0 : 12.0;
            s = lowpass(s, raw);
            overlay.step(Vector3(0.0, 0.0, filtered ? s : raw), m);
            CHECK(!overlay.bounded());    // never on the fence: this is the law, not the clamp
            const double v = overlay.velocity().z();
            if (i > 100) worst = std::max(worst, std::abs(v - prev_v) / dt);  // m/s^2
            prev_v = v;
        }
        return worst;
    };
    const double raw_peak = peak_deviation_rate(false);
    const double filt_peak = peak_deviation_rate(true);
    std::printf("  law: deviation accel peak raw=%.3f filtered=%.3f m/s^2 (%.1fx)\n",
                raw_peak, filt_peak, raw_peak / std::max(filt_peak, 1e-9));
    CHECK(filt_peak < raw_peak * 0.5);   // the burst is at least halved

    // 4) The filter is a LAW input, not a motion filter: at 0 Hz the caller feeds
    //    the raw wrench and the behaviour is bit-identical to before.
    {
        rb_servo::control::AdmittanceOverlay a, b;
        a.configure(law, dt);
        b.configure(law, dt);
        const Vector3 m = Vector3::Zero();
        for (int i = 0; i < 500; ++i) {
            const Vector3 f(0.0, 0.0, (i % 7) * 3.0);   // 0..18 N: crosses rest
            a.step(f, m);
            b.step(f, m);
        }
        CHECK(a.deviation().z() > 0.0);
        CHECK(a.deviation().z() == b.deviation().z());
    }
    return true;
}

// dropDeviation() DROPS THE DISPLACEMENT AND KEEPS THE MOMENTUM - the opposite half
// of freeze(). The next tick continues from the live velocity, so a k = 0 law that is
// folded every tick still yields at (|F| - rest)/b instead of restarting from rest
// each tick (CM's failed "stateless k = 0": 0.04 mm/tick at 10 N, i.e. no compliance).
bool testDropDeviationKeepsTheVelocity() {
    const rb_servo::ForceControlConfig cfg = oneLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, kDt);
    const Vector3 f(0.0, 0.0, 20.0);
    const Vector3 m = Vector3::Zero();
    for (int i = 0; i < 500; ++i) overlay.step(f, m);      // 1 s: v -> (20-10)/b = 20 mm/s
    const double v = overlay.velocity().z();
    CHECK(near(v, 10.0 / cfg.law.b, 2e-4));
    CHECK(overlay.deviation().z() > 0.01);
    overlay.dropDeviation();
    CHECK(overlay.deviation().norm() == 0.0);
    CHECK(overlay.velocity().z() == v);
    overlay.step(f, m);
    CHECK(near(overlay.deviation().z(), v * kDt, 1e-6));  // one tick at the live speed
    // Folding EVERY tick keeps yielding at (|F|-rest)/b: 500 ticks of (step, drop)
    // walk 20 mm.
    double walked = 0.0;
    for (int i = 0; i < 500; ++i) {
        overlay.step(f, m);
        walked += overlay.deviation().z();
        overlay.dropDeviation();
        CHECK(!overlay.bounded());
    }
    CHECK(near(walked, 0.020, 5e-4));
    return true;
}

// ---------------------------------------------------------------------------
// THE CLOSED-LOOP MODEL (2026-09-15: 3-vector, measured normal, unconditional fold)
// ---------------------------------------------------------------------------

// A plant: a plan streamed at v_cmd along +z into a wall at z = wall_z of stiffness
// k_env, the wrench it reports (the wall's push on the tool, -z, plus any lateral load)
// delayed by `delay_ticks` and low-passed like the servo loop does, the gate on the plan
// advance cut along gate.contactNormal() (the measured F_hat), the overlay on the
// emitted pose. `fold` books the deviation into the plan every tick (the servo loop's
// fold - unconditional since 2026-09-15); without it the overlay carries the walk.
//
// Two model options for the design fact this file has to state (see
// testLateralLoadNeverDeadlocksAndMatchesTheClosedForm):
//   lateral_n  a constant external lateral load on the tool along +x [N];
//   mu         Coulomb friction: a load of mu * F_n riding on the normal force along
//              the tangential slide direction (+x here - the direction the surface
//              drags the tool, so the relative slip has one sign and the closed form
//              has one root).
struct WallLoop {
    rb_servo::control::AdmittanceOverlay overlay;
    rb_servo::control::ForceGate gate;
    bool fold = false;
    double dt = kDt;
    double wall_z = 0.010;
    double k_env = 4400.0;
    double v_cmd = 0.050;
    double lpf_hz = 25.0;
    double lateral_n = 0.0;
    double mu = 0.0;
    std::vector<Vector3> delay;
    std::size_t head = 0;
    Vector3 f_filt = Vector3::Zero();   // the filtered PHYSICAL force vector the loop hands out
    bool primed = false;
    Vector3 plan = Vector3::Zero();
    Vector3 emitted = Vector3::Zero();
    Vector3 absorbed = Vector3::Zero();
    Vector3 last_step = Vector3::Zero();   // the emitted pose's displacement this tick
    double force_seen = 0.0;               // the TRUE normal contact force F_n [N]

    WallLoop(const rb_servo::ForceControlConfig& cfg, bool fold_on, int delay_ticks)
        : fold(fold_on),
          delay(static_cast<std::size_t>(std::max(1, delay_ticks)), Vector3::Zero()) {
        overlay.configure(cfg, dt);
        gate.configure(cfg, dt);
        lpf_hz = cfg.wrench_filter_hz;
    }
    void tick() {
        // The sensor reports the contact of `delay` ticks ago.
        const double pen = emitted.z() - wall_z;
        const double f_n = pen > 0.0 ? k_env * pen : 0.0;
        delay[head] = Vector3(lateral_n + mu * f_n, 0.0, -f_n);
        head = (head + 1) % delay.size();
        const Vector3 f_raw = delay[head];
        if (lpf_hz > 0.0) {
            if (!primed) {
                f_filt = f_raw;
                primed = true;
            } else {
                const double tau = 1.0 / (2.0 * M_PI * lpf_hz);
                const double a = std::min(1.0, dt / (tau + dt));
                f_filt += a * (f_raw - f_filt);
            }
        } else {
            f_filt = f_raw;
        }
        force_seen = f_n;
        const Vector3 zero = Vector3::Zero();
        // THE DEMAND, not the achieved advance: `v_cmd` is what the plan asks for and
        // the gate never touches it, which is the property the crossing rests on.
        gate.update(f_filt, zero, v_cmd);
        // The plan advance, cut along the ONE normal the gate publishes.
        const Vector3 adv = cutAlong(Vector3(0.0, 0.0, v_cmd * dt), gate.contactNormal(),
                                     gate.translation(), nullptr);
        plan += adv;
        overlay.step(f_filt, zero);
        const Vector3 now = plan + overlay.deviation();
        last_step = now - emitted;
        emitted = now;
        if (fold) {
            const Vector3 d = overlay.deviation();
            plan += d;
            absorbed += d;
            overlay.dropDeviation();
        }
    }
};

// THE STEADY STATE OF THAT LOOP, CLOSED FORM. With c = F_n/|F| and |F| = sqrt(F_n^2 +
// L^2), L = lateral_n + mu*F_n: the law yields along F_hat at (|F|-rest)/b, of which
// c*(...) is retreat from the wall; the surviving advance into the wall after the cut
// along the tilted F_hat is u_z' = v_s*(1 - c^2 + g*c^2), g = curve(|F|, v_s).
// Equilibrium: c*(|F|-rest)/b = v_s*(1 - c^2 + g*c^2). Solved by bisection on F_n.
double closedFormNormalForce(const rb_servo::ForceControlConfig& cfg, double lateral_n,
                             double mu, double v_s) {
    const double b = cfg.law.b;
    const double rest = cfg.gate_rest_force_n;
    const double peak = cfg.gate_peak_force_n;
    const double v_cross = (peak - rest) / b;
    const auto curve = [&](double f) {
        if (v_s <= v_cross) return 1.0;
        return std::pow(v_cross / v_s, std::pow(f / peak, rb_servo::ForceControlConfig::kGateCurveExponent));
    };
    const auto h = [&](double f_n) {
        const double l = lateral_n + mu * f_n;
        const double f = std::hypot(f_n, l);
        const double c = f > 0.0 ? f_n / f : 0.0;
        const double g = curve(f);
        return c * (f - rest) / b - v_s * (1.0 - c * c + g * c * c);
    };
    double lo = 1e-6, hi = 500.0;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (h(mid) < 0.0) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

// THE FOLD IS A GAUGE CHANGE, CLOSED THROUGH A WALL: with and without it the emitted
// pose is identical on every tick, while the deviation that walks without bound in the
// overlay is, with the fold, exactly the displacement booked into the plan.
bool testFoldIsInvisibleToTheContact() {
    rb_servo::ForceControlConfig cfg = oneLaw();
    cfg.max_deviation_m = 0.0;     // no fence: the unfolded walk must not be clipped
    WallLoop a(cfg, false, 9), b(cfg, true, 9);
    double max_err = 0.0;
    for (int i = 0; i < 2500; ++i) {
        a.tick();
        b.tick();
        max_err = std::max(max_err, (a.emitted - b.emitted).norm());
    }
    std::printf("  fold invariance: max emitted error %.3e m over 5 s; unfolded deviation "
                "%.1f mm, folded plan shift %.1f mm\n",
                max_err, a.overlay.deviation().z() * 1e3, b.absorbed.z() * 1e3);
    CHECK(max_err < 1e-9);
    CHECK(b.overlay.deviation().norm() == 0.0);                        // nothing standing
    CHECK((a.overlay.deviation() - b.absorbed).norm() < 1e-9);         // ... it moved here
    CHECK(a.overlay.deviation().z() < -0.010);                         // and it IS a walk
    return true;
}

// ============================================================================
// THE TWO THINGS THE FORCE DESIGN PROMISES (2026-09-11). Everything else in this
// file is a property of one of its parts; these two are the contract.
// ============================================================================

// (1) A STREAMED CONTACT CONVERGES AT peak_force_n, FOR EVERY STREAM SPEED. That is
// the whole point of the curve: the old smoothstep bounded the force and converged to
// b*v_s (CM measured 6.16 N at 50 mm/s, 4.01 at 25, 1.87 at 10 - the same wall giving a
// different force to a fast hand than to a slow one), and our own fade-to-zero version
// converged to 0 N because the law kept yielding where the gate had stopped the plan.
// Frictionless, head-on: the measured F_hat IS the wall normal, so the crossing is
// exact. (The lateral-load case moved to its own test on 2026-09-15: judged on the
// vector it converges ABOVE the declaration, by design - see below.)
bool testStreamedContactConvergesAtTheDeclaredForceAtEverySpeed() {
    const rb_servo::ForceControlConfig cfg = oneLaw();
    const auto converged = [&](double v_mm_s) {
        WallLoop w(cfg, true, 9);           // 18 ms of transport delay
        // THE MEASURED CONTACT STIFFNESS, not a rigid jig: 39.5 N at 8.9 mm of
        // penetration on the floor (servo_log_20260911_100038, right arm 197.8 s) is
        // 4.4 N/mm, arm compliance included. At a rigid 30.7 kN/m this law rings by its
        // own delay-margin table (tools/force_loop_margin.py: -1.3 dB at 18 ms) and no
        // gate curve changes that - the crossing is a steady-state property.
        w.k_env = 4400.0;
        w.v_cmd = v_mm_s * 1e-3;
        double fsum = 0.0, fmin = 1e9, fmax = -1e9;
        for (int i = 0; i < 6000; ++i) {    // 12 s
            w.tick();
            if (i >= 5500) {
                const double f = w.force_seen;
                fsum += f; fmin = std::min(fmin, f); fmax = std::max(fmax, f);
            }
        }
        std::printf("    v_s %6.1f mm/s -> %.2f N (p-p %.3f)\n", v_mm_s, fsum / 500.0,
                    fmax - fmin);
        return fsum / 500.0;
    };
    std::printf("  the crossing, swept 5x in stream speed (declared %.1f N):\n",
                cfg.gate_peak_force_n);
    const double f30 = converged(30.0), f60 = converged(60.0), f150 = converged(150.0);
    const double lo = std::min({f30, f60, f150}), hi = std::max({f30, f60, f150});
    // AT the declaration, not merely bounded by it.
    CHECK(std::abs(f30 - cfg.gate_peak_force_n) < 0.5);
    CHECK(std::abs(f60 - cfg.gate_peak_force_n) < 0.5);
    CHECK(std::abs(f150 - cfg.gate_peak_force_n) < 0.5);
    // And the SPREAD is what a declared force means: CM measured < 0.2 N over 5x.
    CHECK(hi - lo < 0.5);
    // The closed form agrees: head-on the root is the declaration itself.
    CHECK(near(closedFormNormalForce(cfg, 0.0, 0.0, 0.030), cfg.gate_peak_force_n, 1e-6));
    CHECK(near(closedFormNormalForce(cfg, 0.0, 0.0, 0.150), cfg.gate_peak_force_n, 1e-6));
    return true;
}

// (2) AN EXTERNAL CONTACT RESTS AT rest_force_n, AND FREE SPACE IS NEVER SOUGHT - IN
// EVERY DIRECTION (2026-09-15). The operator's acceptance test in software: *"양팔을
// 당겨서 바닥으로 밀거야. 그러면 10N 이상이 센싱 될것이고, 그렇다면 10 N 까지만 유지되도록
// 로봇이 동작하면 돼."* The plan is NOT advancing here (demand 0), which is exactly the
// case a gate cannot answer - it can only reduce a speed the plan asked for.
bool testExternalContactRestsAtTheRestForceAndFreeSpaceIsNeverSought() {
    const rb_servo::ForceControlConfig cfg = oneLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, kDt);
    const Vector3 zero = Vector3::Zero();

    // |F| <= rest IS AN EQUILIBRIUM IN EVERY DIRECTION: the 26 axis, edge and corner
    // directions of the cube, at 5 N and at exactly the rest force, for 30 s each.
    // Nothing moves and nothing is fenced. (At 10.0 N the vector's norm can sit one
    // ulp above 10 on an irrational direction - a drive of ~1e-15 N, which over 30 s
    // is ~1e-16 m; the 5 N set is exactly zero.)
    std::vector<Vector3> dirs;
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
            for (int k = -1; k <= 1; ++k)
                if (i != 0 || j != 0 || k != 0) dirs.push_back(Vector3(i, j, k).normalized());
    CHECK(dirs.size() == 26);
    for (const Vector3& d : dirs) {
        for (const double mag : {5.0, cfg.gate_rest_force_n}) {
            overlay.reset();
            const Vector3 f = d * mag;
            for (int i = 0; i < 15000; ++i) overlay.step(f, zero);   // 30 s
            CHECK(overlay.deviation().norm() <= 1e-12);
            CHECK(overlay.velocity().norm() <= 1e-12);
            CHECK(!overlay.bounded());
            if (mag < cfg.gate_rest_force_n) CHECK(overlay.deviation().norm() == 0.0);
        }
    }
    // And with no force at all, for 60 s.
    overlay.reset();
    for (int i = 0; i < 30000; ++i) overlay.step(zero, zero);
    CHECK(overlay.deviation().norm() == 0.0);
    CHECK(!overlay.bounded());

    // A HAND PRESS, held, along a TILTED direction: 40 N with no plan advance. The arm
    // yields the EXCESS only, along the force, so it stops when the contact reads
    // rest_force_n. Modelled as a wall the arm is pressed into: the force falls as the
    // arm retreats along the wall's push.
    overlay.reset();
    const Vector3 n_out = Vector3(0.3, 0.0, -1.0).normalized();   // the wall's push on the tool
    const double k_env = 4400.0;                                   // the measured 4.4 N/mm
    const double pen0 = 40.0 / k_env;
    const auto contact = [&]() {
        return k_env * std::max(0.0, pen0 - overlay.deviation().dot(n_out));
    };
    for (int i = 0; i < 30000; ++i) overlay.step(n_out * contact(), zero);
    const double settled = contact();
    std::printf("  hand press with no plan advance along (%.2f, %.2f, %.2f): 40.0 N -> %.2f N at "
                "%.2f mm of yield (declared rest %.1f N)\n", n_out.x(), n_out.y(), n_out.z(),
                settled, overlay.deviation().norm() * 1e3, cfg.gate_rest_force_n);
    CHECK(std::abs(settled - cfg.gate_rest_force_n) < 0.5);
    CHECK(overlay.deviation().cross(n_out).norm() < 1e-9);   // yielded ALONG the force
    CHECK(overlay.deviation().dot(n_out) > 0.0);
    // And it STAYS: no spring, so nothing pulls the arm back off the surface.
    const Vector3 held = overlay.deviation();
    for (int i = 0; i < 5000; ++i) overlay.step(n_out * contact(), zero);
    CHECK((overlay.deviation() - held).norm() < 1e-4);
    CHECK(std::abs(contact() - cfg.gate_rest_force_n) < 0.5);

    // A SUSTAINED DRAG MUST NOT REACH THE FENCE. A hand that follows the arm holds a
    // constant 20 N, so the arm yields (20-10)/b = 20 mm/s for as long as it is pushed.
    // With the fold that travel belongs to the PLAN and the overlay's own deviation
    // stays at zero; without it the overlay would pin the 40 mm fence
    // (servo_log_20260911_133829).
    overlay.reset();
    Vector3 plan = Vector3::Zero();
    for (int i = 0; i < 1000; ++i) {                       // 2 s of a 20 N drag
        overlay.step(Vector3(0.0, 0.0, -20.0), zero);
        plan += overlay.deviation();                       // the fold books it
        overlay.dropDeviation();
        CHECK(!overlay.bounded());
    }
    const double want = yieldDistance(cfg, 10.0, 2.0);
    std::printf("  2 s of a 20 N drag: plan moved %.1f mm (closed form %.1f mm), overlay "
                "deviation %.4f mm, fence never pinned\n", plan.z() * 1e3, -want * 1e3,
                overlay.deviation().norm() * 1e3);
    CHECK(near(plan.z(), -want, 5e-4));
    CHECK(overlay.deviation().norm() == 0.0);
    return true;
}

// THE CURVE'S FIXED POINTS, CLOSED FORM. g(0) = 1 exactly (free space and light contact
// cost the plan nothing) and g(peak) = v_cross/v_s exactly, whatever v_s is - which is
// the algebra behind (1). Below the crossing speed the gate is wide open by design:
// F = rest + b*v_s cannot reach the declaration, so there is nothing to give. And a
// source with NO demand (a Hold) reads 1.0 whatever |F| is: the gate can only reduce a
// speed the plan asked for (2026-09-15).
bool testCurveFixedPointsAndMonotonicity() {
    const rb_servo::ForceControlConfig cfg = oneLaw();
    rb_servo::control::ForceGate gate;
    gate.configure(cfg, kDt);
    const double b = cfg.law.b, v_cross = (cfg.gate_peak_force_n - cfg.gate_rest_force_n) / b;
    CHECK(gate.bEff() == 500.0);
    CHECK(gate.mEff() == cfg.law.m);
    CHECK(std::abs(gate.crossSpeedMs() - v_cross) < 1e-12);
    CHECK(near(v_cross, cfg.gate_peak_vel_mm_s * 1e-3, 1e-12));   // = the declared peak_vel
    const Vector3 zero = Vector3::Zero();
    const auto raw_gate = [&](double force_n, double v_s) {
        rb_servo::control::ForceGate g;
        g.configure(cfg, kDt);
        // Step the slew to convergence so the raw curve is what is read back.
        for (int i = 0; i < 20000; ++i) g.update(Vector3(0.0, 0.0, -force_n), zero, v_s);
        return g.translation();
    };
    for (const double v_s : {0.030, 0.060, 0.150, 0.500}) {
        const double g_peak = raw_gate(cfg.gate_peak_force_n, v_s);
        std::printf("  g(peak) at v_s %5.0f mm/s = %.6f, want v_cross/v_s = %.6f\n",
                    v_s * 1e3, g_peak, v_cross / v_s);
        CHECK(std::abs(g_peak - v_cross / v_s) < 1e-6);
    }
    CHECK(raw_gate(0.0, 0.150) == 1.0);
    // Below (and at) the crossing speed: open, by the same argument that pins the crossing.
    CHECK(raw_gate(cfg.gate_peak_force_n, v_cross * 0.5) == 1.0);
    CHECK(raw_gate(cfg.gate_peak_force_n, v_cross) == 1.0);
    // NO DEMAND -> 1.0 exactly, whatever the force.
    for (const double f : {5.0, 12.0, 50.0, 200.0}) {
        CHECK(raw_gate(f, 0.0) == 1.0);
        CHECK(raw_gate(f, -1.0) == 1.0);
    }
    // Monotone non-increasing in |F| at a fixed speed.
    double previous = 1.0;
    for (double force = 0.0; force < 3.0 * cfg.gate_peak_force_n; force += 0.5) {
        const double g = raw_gate(force, 0.150);
        CHECK(g <= previous + 1e-12);
        previous = g;
    }
    CHECK(previous < 1e-6);   // and it does go essentially shut far above the declaration
    return true;
}

// strip() IS THE INVERSE OF compose(), so a plan-side stage reading FK of the sent
// joints gets the nominal back exactly. Translation only since 2026-09-15: the
// rotation is rigid, so compose() and strip() leave the orientation untouched.
bool testStripInvertsCompose() {
    rb_servo::control::AdmittanceOverlay overlay;
    const rb_servo::ForceControlConfig cfg = oneLaw();
    overlay.configure(cfg, kDt);
    const Vector3 f(3.0, -7.0, 10.0);         // |F| = 12.6 N: above rest, off-axis
    const Vector3 m(0.4, 0.2, -0.3);          // a torque, ignored by a rigid rotation
    for (int i = 0; i < 500; ++i) overlay.step(f, m);
    CHECK(overlay.hasDeviation());
    CHECK(overlay.deviation().cross(f).norm() < 1e-9);   // along the force
    rb_servo::Pose6D nominal;
    nominal.x = 0.5; nominal.y = -0.2; nominal.z = 0.1;
    nominal.rx = 3.0; nominal.ry = -0.1; nominal.rz = 1.5;
    const rb_servo::Pose6D emitted = overlay.compose(nominal);
    CHECK(emitted.rx == nominal.rx && emitted.ry == nominal.ry && emitted.rz == nominal.rz);
    CHECK((Vector3(emitted.x, emitted.y, emitted.z) - Vector3(nominal.x, nominal.y, nominal.z)
           - overlay.deviation()).norm() < 1e-15);
    const rb_servo::Pose6D back = overlay.strip(emitted);
    CHECK(near(back.x, nominal.x, 1e-12) && near(back.y, nominal.y, 1e-12) && near(back.z, nominal.z, 1e-12));
    CHECK(back.rx == nominal.rx && back.ry == nominal.ry && back.rz == nominal.rz);
    const rb_servo::math::Matrix3 r_err =
        rb_servo::math::rotationFromPose(back).transpose() * rb_servo::math::rotationFromPose(nominal);
    CHECK(near(r_err.trace(), 3.0, 1e-9));
    rb_servo::control::AdmittanceOverlay quiet;
    quiet.configure(cfg, kDt);
    CHECK(!quiet.hasDeviation());
    CHECK(quiet.quiescent());
    return true;
}

// THE GATE IS JUDGED ON THE PHYSICAL VECTOR (2026-09-15): |F| of the stand-frame
// force, its direction the normal. Replaces the magnitude-override test - there is no
// override, the vector IS the judgement.
bool testGateIsJudgedOnThePhysicalVector() {
    rb_servo::ForceControlConfig cfg = oneLaw();
    cfg.gate_close_tau_s = kDt;   // one tick
    rb_servo::control::ForceGate gate;
    gate.configure(cfg, kDt);
    const Vector3 zero = Vector3::Zero();
    const Vector3 f(3.0, 0.0, 12.0);          // a diagonal 12.4 N press
    gate.update(f, zero, 0.200);
    CHECK(near(gate.forceN(), std::sqrt(153.0), 1e-12));
    CHECK((gate.forceDirection() - f / std::sqrt(153.0)).norm() < 1e-12);
    CHECK((gate.contactNormal() - gate.forceDirection()).norm() == 0.0);
    CHECK(near(gate.demandMs(), 0.200, 1e-15));
    // g = (v_cross/v_s)^((|F|/peak)^2) = 0.02^1.0625 = 0.0157: the curve does not reach
    // zero, and that is the point - the surviving advance IS the law's yield at the crossing.
    CHECK(gate.translation() < 0.03);
    CHECK(gate.translation() > 0.01);
    // And the cut is along the vector's own direction.
    double removed = 0.0;
    cutAlong(-0.001 * gate.contactNormal(), gate.contactNormal(), gate.translation(), &removed);
    CHECK(removed > 0.0009);
    // 1 N: above the noise band, so a normal is published; 0.4 N: none.
    gate.update(Vector3(0.0, 0.0, 1.0), zero, 0.200);
    CHECK(near(gate.forceN(), 1.0));
    CHECK((gate.contactNormal() - Vector3(0.0, 0.0, 1.0)).norm() < 1e-12);
    gate.update(Vector3(0.0, 0.0, 0.4), zero, 0.200);
    CHECK(near(gate.forceN(), 0.4));
    CHECK(gate.contactNormal().isZero(0.0));
    CHECK(!gate.forceDirection().isZero(0.0));   // the direction is still known ...
    CHECK(gate.forceDirection().z() == 1.0);      // ... just not published as a normal
    return true;
}

// THE GATE ON THE ABSOLUTE-TARGET PATH holds the tracker's STATE, not its goal: the
// advance into the contact is cut, the inward momentum dropped, sliding untouched, and
// the goal still where the source put it - so a released contact leaves no offset.
// The normal is the measured F_hat of the wall's push (2026-09-15).
bool testPoseTrackGateHoldsStateNotGoal() {
    rb_servo::PoseTrackSmdConfig cfg;
    cfg.enable = true;
    cfg.natural_frequency_linear_hz = 2.0;
    cfg.natural_frequency_angular_hz = 2.0;
    rb_servo::SmdPoseTracker tracker(cfg);
    rb_servo::Pose6D start;
    start.z = 0.100;
    tracker.reset(start);
    rb_servo::Pose6D goal = start;
    CHECK(tracker.updateGoalFromCommand(goal) == 0.0);   // latches the reference, no step
    goal.z = 0.050;                           // 50 mm DOWN, through a surface at z = 0.09
    goal.x = 0.020;                           // and 20 mm sideways (sliding)
    // The returned value is the integrated command step - the absolute source's DEMAND
    // for the gate.
    CHECK(near(tracker.updateGoalFromCommand(goal), std::hypot(0.050, 0.020), 1e-12));
    rb_servo::control::ForceGate gate;
    rb_servo::ForceControlConfig fc = oneLaw();
    fc.gate_close_tau_s = kDt;
    gate.configure(fc, kDt);
    const Vector3 zero = Vector3::Zero();
    const Vector3 wall(0.0, 0.0, 12.0);   // the wall pushes the tool +z at the declared force
    for (int i = 0; i < 200; ++i) gate.update(wall, zero, 0.200);
    CHECK(gate.translation() < 0.03);
    CHECK((gate.contactNormal() - Vector3(0.0, 0.0, 1.0)).norm() < 1e-12);
    double z_min = 1.0, x_last = 0.0;
    for (int i = 0; i < 500; ++i) {
        gate.update(wall, zero, 0.200);
        const Vector3 normal = gate.contactNormal();
        const rb_servo::Pose6D before = tracker.currentPose();
        rb_servo::Pose6D out = tracker.step(kDt);
        const Vector3 p0(before.x, before.y, before.z), p1(out.x, out.y, out.z);
        const double proj = (p1 - p0).dot(normal);
        if (proj < 0.0) {
            const Vector3 cut = (1.0 - gate.translation()) * proj * normal;
            tracker.constrainTranslation(p1 - cut, normal, 1.0 - gate.translation());
        }
        const rb_servo::Pose6D now = tracker.currentPose();
        z_min = std::min(z_min, now.z);
        x_last = now.x;
    }
    // NOT "never advanced": the curve leaves exactly the law's own yield share, which
    // is what makes the contact converge at peak_force_n instead of at 0 N. What must
    // hold is that the 50 mm the goal asked for does not happen - measured residual
    // here is under 1 mm against an ungated 50 mm.
    std::printf("  pose-track: %.3f mm of residual advance into the contact (goal asked "
                "50 mm), %.1f mm of sliding\n", (0.100 - z_min) * 1e3, x_last * 1e3);
    CHECK(z_min > 0.100 - 0.001);
    CHECK(x_last > 0.015);                    // but slid sideways toward the goal
    CHECK(near(tracker.goalPose().z, 0.050, 1e-9));   // the goal is untouched
    // Release: the gate opens, the tracker resumes toward the goal from rest, no jump.
    for (int i = 0; i < 21; ++i) gate.update(zero, zero, 0.0);
    const rb_servo::Pose6D a = tracker.step(kDt);
    const rb_servo::Pose6D b = tracker.step(kDt);
    CHECK(std::abs(b.z - a.z) < 1e-4);        // one tick of ordinary SMD motion, not a lunge
    return true;
}

// ============================================================================
// THE VECTOR LAW'S OWN INVARIANTS (new 2026-09-15)
// ============================================================================

// (a) A DIAGONAL PUSH YIELDS ONLY THE EXCESS, ALONG THE FORCE. It is the VECTOR that is
// one-sided, not each axis: a diagonal 10 N reads 10 N (rest - nothing moves), not
// 7.1 N per axis (which a per-axis rest of 10 N would also not move, but for the wrong
// reason: a diagonal 14 N would then read 9.9 N per axis and not move either). At 14 N
// the arm yields 4 N / b = 8 mm/s along (1,1,0)/sqrt2, folded into the plan, and stops
// where it was dragged when the hand lets go.
bool testDiagonalPushYieldsOnlyTheExcessAlongTheForce() {
    const rb_servo::ForceControlConfig cfg = oneLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, kDt);
    const Vector3 zero = Vector3::Zero();
    const Vector3 f_hat = Vector3(1.0, 1.0, 0.0).normalized();
    // AT rest, diagonally, for 30 s: nothing.
    for (int i = 0; i < 15000; ++i) overlay.step(f_hat * 10.0, zero);
    CHECK(overlay.deviation().norm() <= 1e-12);
    CHECK(overlay.velocity().norm() <= 1e-12);
    CHECK(!overlay.bounded());
    // 14 N diagonally, folded every tick, for 2 s.
    overlay.reset();
    Vector3 plan = Vector3::Zero();
    for (int i = 0; i < 1000; ++i) {
        overlay.step(f_hat * 14.0, zero);
        plan += overlay.deviation();
        overlay.dropDeviation();
        CHECK(!overlay.bounded());
    }
    const double want = yieldDistance(cfg, 4.0, 2.0);   // 16 mm minus the m/b ramp
    std::printf("  14 N diagonal, 2 s folded: plan travelled %.3f mm (closed form %.3f mm), "
                "x %.3f / y %.3f mm\n", plan.norm() * 1e3, want * 1e3, plan.x() * 1e3,
                plan.y() * 1e3);
    CHECK(std::abs(plan.norm() - want) < 5e-4);
    CHECK(std::abs(plan.x() - plan.y()) < 1e-12);
    CHECK(plan.z() == 0.0);
    CHECK(plan.cross(f_hat).norm() < 1e-9);
    // RELEASE: the momentum runs out in m/b and the plan stays where it was dragged.
    const Vector3 v_release = overlay.velocity();
    CHECK(near(v_release.norm(), 4.0 / cfg.law.b, 1e-6));
    Vector3 extra = Vector3::Zero();
    for (int i = 0; i < 250; ++i) {                    // 0.5 s
        overlay.step(zero, zero);
        extra += overlay.deviation();
        overlay.dropDeviation();
    }
    CHECK(extra.norm() <= cfg.law.m * v_release.norm() / cfg.law.b + 1e-5);
    CHECK(extra.dot(f_hat) > 0.0);                     // coasting, not springing back
    CHECK(overlay.velocity().norm() < 1e-6);
    return true;
}

// (b) A HAND GRABBING THE TOOL MID-STREAM: the plan streams +x at 100 mm/s while a 14 N
// side force along -y stands for 1 s. The gate closes (|F| > peak, demand > v_cross)
// but the advance is PERPENDICULAR to F_hat, so it cuts nothing: the stream is not
// slowed by a force it is not pushing into. The arm yields along the hand at 8 mm/s,
// folded into the plan, and when the hand lets go the plan is where the arm was dragged
// - no standing offset, no return.
bool testHandGrabMidStreamYieldsAlongTheForceAndLeavesNoOffset() {
    const rb_servo::ForceControlConfig cfg = oneLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    rb_servo::control::ForceGate gate;
    overlay.configure(cfg, kDt);
    gate.configure(cfg, kDt);
    const Vector3 zero = Vector3::Zero();
    const double v_s = 0.100;
    const Vector3 grab(0.0, -14.0, 0.0);
    const int grab_ticks = 500, after_ticks = 250;
    Vector3 plan = Vector3::Zero();
    double plan_y_at_release = 0.0;
    Vector3 v_at_release = Vector3::Zero();
    for (int i = 0; i < grab_ticks + after_ticks; ++i) {
        const Vector3 f = i < grab_ticks ? grab : zero;
        gate.update(f, zero, v_s);
        double removed = -1.0;
        const Vector3 adv = cutAlong(Vector3(v_s * kDt, 0.0, 0.0), gate.contactNormal(),
                                     gate.translation(), &removed);
        CHECK(removed == 0.0);                          // advance . F_hat == 0: nothing to cut
        plan += adv;
        overlay.step(f, zero);
        plan += overlay.deviation();                    // the fold
        overlay.dropDeviation();
        CHECK(overlay.deviation().norm() == 0.0);       // nothing stands in the overlay
        if (i == grab_ticks - 1) {
            plan_y_at_release = plan.y();
            v_at_release = overlay.velocity();
            CHECK(gate.translation() < 0.1);            // the gate DID close ...
            CHECK((gate.contactNormal() - Vector3(0.0, -1.0, 0.0)).norm() < 1e-12);
        }
    }
    const double want_y = -yieldDistance(cfg, 4.0, 1.0);   // -8 mm plus the ramp
    std::printf("  hand grab mid-stream: plan x %.3f mm (stream %.3f), y at release %.3f mm "
                "(closed form %.3f), y final %.3f mm\n", plan.x() * 1e3, v_s * 1.5 * 1e3,
                plan_y_at_release * 1e3, want_y * 1e3, plan.y() * 1e3);
    CHECK(near(plan.x(), v_s * (grab_ticks + after_ticks) * kDt, 1e-9));   // ... and cut nothing
    CHECK(std::abs(plan_y_at_release - want_y) < 3e-4);
    CHECK(plan.z() == 0.0);
    // After release: the coast is bounded by m*v/b, then everything is still.
    CHECK(std::abs(plan.y() - plan_y_at_release) <= cfg.law.m * v_at_release.norm() / cfg.law.b + 1e-5);
    CHECK(overlay.velocity().norm() < 1e-6);
    CHECK(overlay.deviation().norm() == 0.0);
    return true;
}

// (c) A LATERAL LOAD NEVER DEADLOCKS, AND THE NORMAL FORCE LANDS ABOVE THE DECLARATION -
// BY THE CLOSED FORM. This is the design fact the isotropic law carries (measured in the
// model, not a bug): the cut is projective along F_hat, so the part of the advance
// perpendicular to the TILTED F_hat survives, and under a lateral load or friction the
// normal force converges above peak_force_n. What can NOT happen any more is the 14:12
// deadlock ("gate shut, law at rest"): the law always yields along F_hat, so the plan
// keeps moving - tangentially - every tick. Cutting the WHOLE into-contact advance would
// fix the number and kill sliding along a surface; not taken.
bool testLateralLoadNeverDeadlocksAndMatchesTheClosedForm() {
    const rb_servo::ForceControlConfig cfg = oneLaw();
    struct Result { double f_n; double pp; double slowest_window_m_s; };
    const auto run = [&](double lateral, double mu, double v_s) {
        WallLoop w(cfg, true, 9);
        w.lateral_n = lateral;
        w.mu = mu;
        w.k_env = 4400.0;
        w.v_cmd = v_s;
        std::vector<double> speed;
        speed.reserve(6000);
        double fsum = 0.0, fmin = 1e9, fmax = -1e9;
        for (int i = 0; i < 6000; ++i) {
            w.tick();
            speed.push_back(w.last_step.norm() / w.dt);
            if (i >= 5500) {
                fsum += w.force_seen;
                fmin = std::min(fmin, w.force_seen);
                fmax = std::max(fmax, w.force_seen);
            }
        }
        // The slowest 0.2 s window of the emitted pose's speed, over the whole run.
        const int win = 100;
        double sum = 0.0, slowest = 1e9;
        for (int i = 0; i < static_cast<int>(speed.size()); ++i) {
            sum += speed[static_cast<std::size_t>(i)];
            if (i >= win) sum -= speed[static_cast<std::size_t>(i - win)];
            if (i >= win - 1) slowest = std::min(slowest, sum / win);
        }
        return Result{fsum / 500.0, fmax - fmin, slowest};
    };
    std::printf("  lateral load / friction, closed form vs loop (declared %.1f N):\n",
                cfg.gate_peak_force_n);
    // A constant 20 N side load at 100 mm/s.
    {
        const double closed = closedFormNormalForce(cfg, 20.0, 0.0, 0.100);
        const Result r = run(20.0, 0.0, 0.100);
        std::printf("    L = 20 N, v_s 100 mm/s: closed form F_n %.2f N, loop %.2f N (p-p %.3f), "
                    "slowest 0.2 s window %.1f mm/s\n", closed, r.f_n, r.pp,
                    r.slowest_window_m_s * 1e3);
        CHECK(closed > 24.0 && closed < 29.0);                 // ~26.3 N: ABOVE the declaration
        CHECK(std::abs(r.f_n - closed) < 1.5);
        CHECK(r.slowest_window_m_s >= 0.01 * 0.100);           // never stalls
    }
    // Coulomb friction mu 0.3 at 30 and 150 mm/s.
    for (const double v_s : {0.030, 0.150}) {
        const double closed = closedFormNormalForce(cfg, 0.0, 0.3, v_s);
        const Result r = run(0.0, 0.3, v_s);
        std::printf("    mu 0.3, v_s %3.0f mm/s: closed form F_n %.2f N, loop %.2f N (p-p %.3f), "
                    "slowest 0.2 s window %.1f mm/s\n", v_s * 1e3, closed, r.f_n, r.pp,
                    r.slowest_window_m_s * 1e3);
        CHECK(closed > cfg.gate_peak_force_n);                 // above, by design
        CHECK(std::abs(r.f_n - closed) < 1.5);
        CHECK(r.slowest_window_m_s >= 0.01 * v_s);
    }
    // The bracket the design note quotes: ~12.3 N at 30 mm/s, ~15.8 N at 150 mm/s.
    CHECK(std::abs(closedFormNormalForce(cfg, 0.0, 0.3, 0.030) - 12.3) < 0.3);
    CHECK(std::abs(closedFormNormalForce(cfg, 0.0, 0.3, 0.150) - 15.8) < 0.3);
    return true;
}

// (d) THE CONTACT NORMAL NEVER SWITCHES ACROSS THE OPERATING FORCE. Everything the gate
// does is continuous in |F|; the direction is the one place a discontinuity could hide
// (the 2026-09-11 version hid one at rest_force_n). The normal is published from the
// 0.5 N noise band up, so a force sweeping 4 -> 50 N through rest and peak, with the
// arms' measured 52 Hz ripple and noise on the side axes, never withdraws it, never
// flips it, and the cut it drives changes by well under 5 % of the advance per tick.
bool testContactNormalNeverSwitchesAcrossTheOperatingForce() {
    const rb_servo::ForceControlConfig cfg = oneLaw();
    rb_servo::control::ForceGate gate;
    gate.configure(cfg, kDt);
    const Vector3 zero = Vector3::Zero();
    const double alpha = kDt / (1.0 / (2.0 * M_PI * cfg.wrench_filter_hz) + kDt);
    uint64_t lcg = 0x9E3779B97F4A7C15ULL;
    const auto noise = [&]() {   // uniform on [-0.5, 0.5] N, deterministic
        lcg = lcg * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(lcg >> 11) / 9007199254740992.0 - 0.5;
    };
    const Vector3 adv(0.0, 0.0, -0.001);   // 1 mm into the contact
    Vector3 f_filt = Vector3::Zero();
    Vector3 prev_cut = Vector3::Zero();
    int zero_normals = 0, flips = 0;
    double max_cut_change = 0.0, min_force = 1e9;
    for (int i = 0; i < 2500; ++i) {           // 5 s
        const double t = i * kDt;
        const double amplitude = 4.0 + 46.0 * t / 5.0 + 1.0 * std::sin(2.0 * M_PI * 52.0 * t);
        const Vector3 raw(noise(), noise(), amplitude);
        f_filt = i == 0 ? raw : Vector3(f_filt + alpha * (raw - f_filt));
        gate.update(f_filt, zero, 0.100);
        min_force = std::min(min_force, gate.forceN());
        const Vector3 n = gate.contactNormal();
        if (gate.forceN() > 2.0 && n.isZero(0.0)) ++zero_normals;
        if (n.z() <= 0.0) ++flips;             // the wall pushes +z throughout
        const Vector3 cut = adv - cutAlong(adv, n, gate.translation(), nullptr);
        if (i > 0) max_cut_change = std::max(max_cut_change, (cut - prev_cut).norm());
        prev_cut = cut;
    }
    std::printf("  normal sweep 4 -> 50 N: min |F| %.2f N, %d zero normals, %d flips, max cut "
                "change/tick %.2f %% of the advance\n", min_force, zero_normals, flips,
                max_cut_change / adv.norm() * 100.0);
    CHECK(min_force > 2.0);
    CHECK(zero_normals == 0);
    CHECK(flips == 0);
    CHECK(max_cut_change < 0.05 * adv.norm());
    CHECK(gate.translation() < 0.01);          // it did close over the sweep

    // BELOW THE BAND: no normal, so a consumer removes nothing whatever the gate reads
    // - and the gate reads > 0.99 there anyway (g(0.4 N) = 0.9964 at 100 mm/s).
    rb_servo::control::ForceGate quiet;
    quiet.configure(cfg, kDt);
    for (int i = 0; i < 100; ++i) quiet.update(Vector3(0.0, 0.0, 0.4), zero, 0.100);
    CHECK(quiet.contactNormal().isZero(0.0));
    CHECK(quiet.translation() > 0.99);
    double removed = 1.0;
    CHECK((cutAlong(adv, quiet.contactNormal(), quiet.translation(), &removed) - adv).norm() == 0.0);
    CHECK(removed == 0.0);
    // With no demand (a Hold) or no force it is exactly 1.0.
    quiet.reset();
    for (int i = 0; i < 100; ++i) quiet.update(Vector3(0.0, 0.0, 0.4), zero, 0.0);
    CHECK(quiet.translation() == 1.0);
    CHECK(quiet.contactNormal().isZero(0.0));
    quiet.reset();
    for (int i = 0; i < 100; ++i) quiet.update(zero, zero, 0.100);
    CHECK(quiet.translation() == 1.0);
    return true;
}

}  // namespace

int main() {
    testExternallyVerifiedSensorDoesNotGrantTareOrAcceptInvalidWrench();
    testPoseTrackGateHoldsStateNotGoal();
    testStripInvertsCompose();
    testGateIsJudgedOnThePhysicalVector();
    testDropDeviationKeepsTheVelocity();
    testFoldIsInvisibleToTheContact();
    testStreamedContactConvergesAtTheDeclaredForceAtEverySpeed();
    testExternalContactRestsAtTheRestForceAndFreeSpaceIsNeverSought();
    testCurveFixedPointsAndMonotonicity();
    testDiagonalPushYieldsOnlyTheExcessAlongTheForce();
    testHandGrabMidStreamYieldsAlongTheForceAndLeavesNoOffset();
    testLateralLoadNeverDeadlocksAndMatchesTheClosedForm();
    testContactNormalNeverSwitchesAcrossTheOperatingForce();
    testWrenchFilterFlattensShockAndKeepsSteadyForce();
    testOscillationGuardTripsFreezesAndReleases();
    testAxisMapIsTheMeasuredLeftHandedBasis();
    testFullToolGravityIsSubtracted();
    testToolInertiaIsSubtractedWithGravity();
    testTareDoesNotDoubleSubtractGravity();
    testDisconnectedSensorPinsCompensatedChannelsToZero();
    testDeadzoneIsContinuous();
    testSteadyForceYieldsTheExcessOverRest();
    testFenceClampsAndReportsSaturation();
    testRotationIsRigid();
    testFreezeKeepsTheDeviationAndDropsVelocity();
    testGateAttenuatesOnlyIntoTheContact();
    testGateIsAsymmetric();
    testGateIsOpenInFreeSpace();
    testGateReopensToExactlyOneAfterRelease();
    testHoldFoldDeltaFloorAndCap();
    testFreezeHoldsTheDeviationWhileTheCommandIsNotExecuted();
    testResumeContinuesFromTheFrozenDeviation();
    if (g_failures == 0) std::printf("force control tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
