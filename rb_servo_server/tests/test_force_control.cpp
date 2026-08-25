// test_force_control.cpp - the invariants of the ported F/T pipeline and admittance
// overlay. Every check here is a claim that would be expensive to discover on the
// robot, and several are claims controller-manager already paid for on hardware.
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/admittance_overlay.hpp"
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
    pipe.configure(cfg, 0.002);
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
    pipe.configure(cfg, 0.002);
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

// ---------------------------------------------------------------------------

rb_servo::ForceControlConfig shippedLaw() {
    rb_servo::ForceControlConfig c;
    c.enable = true;
    for (int i = 0; i < 3; ++i) {
        c.translation[i] = {rb_servo::ForceAxisMode::Compliance, 15.0, 434.7, 400.0, 0.0};
        c.rotation[i] = {rb_servo::ForceAxisMode::Compliance, 1.0, 15.72, 8.0, 0.0};
    }
    c.gate_enable = true;
    c.gate_max_force_n = 10.0;
    c.gate_max_torque_nm = 1.4;
    c.max_deviation_m = 0.040;
    c.max_deviation_rad = 0.2617993878;
    return c;
}

// A STEADY FORCE DEFLECTS F/k, AND THAT IS THE NUMBER AN OPERATOR CAN CHECK WITH A
// RULER. At the shipped 400 N/m, 10 N must settle at 25 mm.
bool testSteadyForceConvergesToForceOverStiffness() {
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(shippedLaw(), 0.002);
    const rb_servo::math::Vector3 f(0.0, 0.0, 10.0);
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 20000; ++i) overlay.step(f, m);   // 40 s
    CHECK(near(overlay.deviation().z(), 10.0 / 400.0, 1e-4));
    return true;
}

// THE FENCE HOLDS AND SAYS SO. Past the bound the deviation is clamped, the velocity
// along the clamped direction is zeroed so nothing winds up against it, and
// `bounded()` latches for the caller to publish.
bool testFenceClampsAndReportsSaturation() {
    rb_servo::ForceControlConfig cfg = shippedLaw();
    for (int i = 0; i < 3; ++i) cfg.translation[i].k = 0.0;   // pure mass-damper: it walks
    cfg.gate_enable = false;
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, 0.002);
    const rb_servo::math::Vector3 f(0.0, 0.0, 200.0);
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 20000; ++i) overlay.step(f, m);
    CHECK(overlay.bounded());
    CHECK(overlay.deviation().norm() <= cfg.max_deviation_m + 1e-9);
    return true;
}

// A RIGID AXIS DOES NOT DEVIATE AT ALL, whatever is pushing on it — that is what a
// client asking for the nominal path on one axis means.
bool testRigidAxisDoesNotDeviate() {
    rb_servo::ForceControlConfig cfg = shippedLaw();
    cfg.translation[0].mode = rb_servo::ForceAxisMode::Rigid;
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, 0.002);
    const rb_servo::math::Vector3 f(50.0, 50.0, 0.0);
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 5000; ++i) overlay.step(f, m);
    CHECK(near(overlay.deviation().x(), 0.0, 1e-12));
    CHECK(overlay.deviation().y() > 1e-3);   // the compliant axis did move
    return true;
}

// LEAVING SERVICE FREEZES THE DISPLACEMENT AND DROPS THE MOMENTUM. It must NOT walk
// the deviation back: under contact the nominal is inside the workpiece, so retiring
// would command the tool the whole deviation deeper.
bool testFreezeKeepsTheDeviationAndDropsVelocity() {
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(shippedLaw(), 0.002);
    const rb_servo::math::Vector3 f(0.0, 0.0, 10.0);
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 200; ++i) overlay.step(f, m);
    const double held = overlay.deviation().z();
    CHECK(held > 1e-5);
    CHECK(overlay.velocity().norm() > 1e-6);
    overlay.freeze();
    CHECK(near(overlay.deviation().z(), held, 1e-12));
    CHECK(near(overlay.velocity().norm(), 0.0, 1e-12));
    return true;
}

// THE GATE IS PROJECTIVE. Scaling the whole advance would kill sliding along a
// contact AND throttle backing out of it, which is the escape an operator needs.
bool testGateAttenuatesOnlyIntoTheContact() {
    rb_servo::ForceControlConfig cfg = shippedLaw();
    rb_servo::control::ForceGate gate;
    gate.configure(cfg, 0.002);
    // Drive the gate closed against a +Z reaction (the environment pushes the tool up).
    const rb_servo::math::Vector3 f(0.0, 0.0, 20.0);
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 5000; ++i) gate.update(f, m);
    CHECK(gate.translation() < 0.02);

    double removed = 0.0;
    // INTO the contact (advance opposes the reaction) -> attenuated.
    const auto into = gate.applyTranslation(rb_servo::math::Vector3(0.0, 0.0, -0.001), &removed);
    CHECK(std::abs(into.z()) < 1e-4);
    CHECK(removed > 0.0);
    // TANGENTIAL -> untouched.
    const auto tang = gate.applyTranslation(rb_servo::math::Vector3(0.001, 0.0, 0.0), &removed);
    CHECK(near(tang.x(), 0.001, 1e-12));
    CHECK(near(removed, 0.0));
    // RETREATING -> untouched, at full authority.
    const auto out = gate.applyTranslation(rb_servo::math::Vector3(0.0, 0.0, 0.001), &removed);
    CHECK(near(out.z(), 0.001, 1e-12));
    CHECK(near(removed, 0.0));
    return true;
}

// FAST TO CLOSE, SLOW TO OPEN. A gate that re-opens as fast as it closes becomes a
// relay against the contact and sustains a limit cycle.
//
// COMPARED AS A PER-TICK RATE OVER THE SAME GAP, not as time-to-cross-a-threshold:
// the two directions start from different distances, so crossing times say nothing
// about the time constants. This is the comparison that is actually about tau.
bool testGateIsAsymmetric() {
    rb_servo::ForceControlConfig cfg = shippedLaw();
    cfg.gate_close_tau_s = 0.10;
    cfg.gate_open_tau_s = 0.40;
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    const rb_servo::math::Vector3 hard(0.0, 0.0, 20.0);   // fade -> 0
    const rb_servo::math::Vector3 none = rb_servo::math::Vector3::Zero();   // fade -> 1

    // ONE tick of closing from a fully open gate: gap 1.0.
    rb_servo::control::ForceGate closing;
    closing.configure(cfg, 0.002);
    closing.update(hard, m);
    const double close_step = 1.0 - closing.translation();

    // ONE tick of opening from a fully closed gate: the same gap of 1.0.
    rb_servo::control::ForceGate opening;
    opening.configure(cfg, 0.002);
    for (int i = 0; i < 5000; ++i) opening.update(hard, m);   // drive it to ~0
    const double closed = opening.translation();
    opening.update(none, m);
    const double open_step = opening.translation() - closed;

    CHECK(close_step > 0.0);
    CHECK(open_step > 0.0);
    // tau 0.10 vs 0.40 -> closing moves 4x per tick over an equal gap.
    CHECK(close_step > open_step * 3.5);
    return true;
}

// g(0) = 1 EXACTLY: free space and light contact must cost the plan nothing.
bool testGateIsOpenInFreeSpace() {
    rb_servo::control::ForceGate gate;
    gate.configure(shippedLaw(), 0.002);
    const rb_servo::math::Vector3 zero = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 1000; ++i) gate.update(zero, zero);
    CHECK(near(gate.translation(), 1.0, 1e-12));
    double removed = 1.0;
    const auto adv = gate.applyTranslation(rb_servo::math::Vector3(0.0, 0.0, -0.001), &removed);
    CHECK(near(adv.z(), -0.001, 1e-12));
    CHECK(near(removed, 0.0));
    return true;
}

}  // namespace

int main() {
    testAxisMapIsTheMeasuredLeftHandedBasis();
    testFullToolGravityIsSubtracted();
    testTareDoesNotDoubleSubtractGravity();
    testDisconnectedSensorPinsCompensatedChannelsToZero();
    testDeadzoneIsContinuous();
    testSteadyForceConvergesToForceOverStiffness();
    testFenceClampsAndReportsSaturation();
    testRigidAxisDoesNotDeviate();
    testFreezeKeepsTheDeviationAndDropsVelocity();
    testGateAttenuatesOnlyIntoTheContact();
    testGateIsAsymmetric();
    testGateIsOpenInFreeSpace();
    if (g_failures == 0) std::printf("force control tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
