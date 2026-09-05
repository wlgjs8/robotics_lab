// test_force_control.cpp - the invariants of the ported F/T pipeline and admittance
// overlay. Every check here is a claim that would be expensive to discover on the
// robot, and several are claims controller-manager already paid for on hardware.
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <algorithm>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/hold_fold.hpp"
#include "rb_servo/control/admittance_overlay.hpp"
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

// The shipped STREAM law (controller-manager's follow 10 N row).
rb_servo::ForceControlConfig shippedLaw() {
    rb_servo::ForceControlConfig c;
    c.enable = true;
    for (int i = 0; i < 3; ++i) {
        c.stream.translation[i] = {rb_servo::ForceAxisMode::Compliance, 15.0, 434.7, 400.0, 0.0};
        c.stream.rotation[i] = {rb_servo::ForceAxisMode::Compliance, 1.0, 15.72, 8.0, 0.0};
        // The shipped HOLD law (controller-manager's admittance.yaml, per-axis).
        c.hold.translation[i] = {rb_servo::ForceAxisMode::Compliance, 10.0, 379.4, 2499.1, 0.0};
        c.hold.rotation[i] = {rb_servo::ForceAxisMode::Compliance, 1.0, 37.94, 249.91, 0.0};
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
    for (int i = 0; i < 3; ++i) cfg.stream.translation[i].k = 0.0;   // pure mass-damper: it walks
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
    cfg.stream.translation[0].mode = rb_servo::ForceAxisMode::Rigid;
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

// THE STREAM CHANNEL IGNORES A VIBRATION AND HOLDS ON A SUSTAINED CONTACT
// (2026-09-04). A zero-mean 15 Hz force of 20 N amplitude (the tool's motion-
// excited vibration, measured 3-5 N RMS with 12-33 ms excursions over 10 N) closes
// the tick channel; the stream channel, judged on the low-passed VECTOR, must not
// arm. Pressed steadily it must arm after the dwell and cut only INTO the contact.
bool testGateStreamChannelIgnoresVibrationAndHoldsSustainedContact() {
    rb_servo::ForceControlConfig cfg = shippedLaw();
    cfg.gate_stream_judge_lpf_hz = 2.0;
    cfg.gate_stream_arm_force_n = 5.0;
    cfg.gate_stream_release_force_n = 2.0;
    cfg.gate_stream_arm_dwell_sec = 0.10;
    rb_servo::control::ForceGate gate;
    gate.configure(cfg, 0.002);
    const rb_servo::math::Vector3 zero = rb_servo::math::Vector3::Zero();
    const rb_servo::math::Vector3 into(0.0, 0.0, -0.001);
    double min_tick = 1.0;
    double max_stream_removed = 0.0;
    double max_slow = 0.0;
    for (int i = 0; i < 2000; ++i) {
        // Ring-up over 0.2 s like a real resonance: a full-amplitude vibration that
        // starts abruptly has a one-sided first half-cycle (a 5 N transient in the
        // slow filter) that no physical tool produces.
        const double ring_up = std::min(1.0, i / 100.0);
        const double f = ring_up * 20.0 * std::sin(2.0 * M_PI * 15.0 * i * 0.002);
        const rb_servo::math::Vector3 vib(0.0, 0.0, f);
        gate.update(vib, zero, std::abs(f));
        gate.updateStream(vib);
        min_tick = std::min(min_tick, gate.translation());
        max_slow = std::max(max_slow, gate.streamForceN());
        double removed = 0.0;
        gate.applyStreamTranslation(into, &removed);
        max_stream_removed = std::max(max_stream_removed, removed);
        CHECK(!gate.streamArmed());
    }
    CHECK(min_tick < 0.5);                       // the tick channel did react
    CHECK(max_slow < 4.0);                       // 20 N at 15 Hz is ~2.6 N after the 2 Hz vector LPF
    CHECK(near(gate.streamTranslation(), 1.0, 1e-9));
    CHECK(near(max_stream_removed, 0.0));        // the stream channel did nothing
    // A sustained 20 N push (+Z reaction): arms after the dwell, closes, cuts INTO only.
    const rb_servo::math::Vector3 push(0.0, 0.0, 20.0);
    int armed_at = -1;
    for (int i = 0; i < 1000; ++i) {
        gate.update(push, zero, 20.0);
        gate.updateStream(push);
        if (armed_at < 0 && gate.streamArmed()) armed_at = i;
    }
    CHECK(armed_at >= 50);                       // not before the 100 ms dwell
    CHECK(armed_at < 250);                       // and not long after the 2 Hz LPF crosses 5 N
    CHECK(gate.streamTranslation() < 0.02);
    double removed = 0.0;
    const auto held = gate.applyStreamTranslation(into, &removed);
    CHECK(std::abs(held.z()) < 1e-4);
    CHECK(removed > 0.0);
    const auto tang = gate.applyStreamTranslation(rb_servo::math::Vector3(0.001, 0.0, 0.0), &removed);
    CHECK(near(tang.x(), 0.001, 1e-12));
    CHECK(near(removed, 0.0));
    const auto out = gate.applyStreamTranslation(rb_servo::math::Vector3(0.0, 0.0, 0.001), &removed);
    CHECK(near(out.z(), 0.001, 1e-12));
    CHECK(near(removed, 0.0));
    // Release: the force goes away, the channel disarms below 2 N and re-opens slowly.
    int disarmed_at = -1;
    for (int i = 0; i < 2000; ++i) {
        gate.update(zero, zero, 0.0);
        gate.updateStream(zero);
        if (disarmed_at < 0 && !gate.streamArmed()) disarmed_at = i;
    }
    CHECK(disarmed_at > 0);
    CHECK(disarmed_at < 500);
    CHECK(gate.streamTranslation() > 0.9);
    return true;
}

// A RELEASED GATE RE-OPENS TO EXACTLY 1.0 - on both channels. A first-order slew
// only approaches 1; without the snap a gate that closed once stayed at 0.9999...
// and its nanometre "cuts" kept invoking the tracker's hold (2026-09-04 22:32).
bool testGateReopensToExactlyOneAfterRelease() {
    rb_servo::ForceControlConfig cfg = shippedLaw();
    cfg.gate_open_tau_s = 1.0;
    rb_servo::control::ForceGate gate;
    gate.configure(cfg, 0.002);
    const rb_servo::math::Vector3 zero = rb_servo::math::Vector3::Zero();
    const rb_servo::math::Vector3 push(0.0, 0.0, 20.0);
    for (int i = 0; i < 500; ++i) {
        gate.update(push, zero, 20.0);
        gate.updateStream(push);
    }
    CHECK(gate.translation() < 0.02);
    CHECK(gate.streamTranslation() < 0.02);
    // Release and wait 15 tau: the exponential alone would sit at 1 - 3e-7.
    for (int i = 0; i < 7500; ++i) {
        gate.update(zero, zero, 0.0);
        gate.updateStream(zero);
    }
    CHECK(gate.translation() == 1.0);
    CHECK(gate.streamTranslation() == 1.0);
    // The slow force direction has decayed too; even a stale one must cut nothing.
    double removed = 1.0;
    gate.applyStreamTranslation(rb_servo::math::Vector3(0.0, 0.0, -0.001), &removed);
    CHECK(removed == 0.0);
    gate.applyTranslation(rb_servo::math::Vector3(0.0, 0.0, -0.001), &removed);
    CHECK(removed == 0.0);
    return true;
}

// THE HOLD FOLD DELTA: the whole plan-vs-sent shortfall, with a noise floor below
// which nothing is booked and a snap cap above which the fold is declined.
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

// A SIGN-FLIPPING DIRECTION CUTS NOTHING. With the tick channel, a 20 N force that
// alternates +Z / -Z each tick cut the tracker on every other tick (the chopping
// measured on 2026-09-04); judged on the low-passed VECTOR it is no force at all.
bool testGateStreamChannelAveragesOutAFlippingDirection() {
    rb_servo::ForceControlConfig cfg = shippedLaw();
    cfg.gate_stream_judge_lpf_hz = 2.0;
    cfg.gate_stream_arm_force_n = 5.0;
    cfg.gate_stream_release_force_n = 2.0;
    cfg.gate_stream_arm_dwell_sec = 0.10;
    rb_servo::control::ForceGate gate;
    gate.configure(cfg, 0.002);
    const rb_servo::math::Vector3 zero = rb_servo::math::Vector3::Zero();
    const rb_servo::math::Vector3 up(0.0, 0.0, 20.0);
    const rb_servo::math::Vector3 down(0.0, 0.0, -20.0);
    // Prime in free space first: the slow filter seeds on its first sample (so a
    // contact already standing at enable is not ramped into from a stale zero), and
    // this test is about the steady state, not the seed.
    for (int i = 0; i < 200; ++i) {
        gate.update(zero, zero, 0.0);
        gate.updateStream(zero);
    }
    double tick_removed_total = 0.0;
    double stream_removed_total = 0.0;
    for (int i = 0; i < 1000; ++i) {
        const auto& f = (i % 2 == 0) ? up : down;
        gate.update(f, zero, 20.0);
        gate.updateStream(f);
        double r = 0.0;
        gate.applyTranslation(rb_servo::math::Vector3(0.0, 0.0, -0.001), &r);
        tick_removed_total += r;
        gate.applyStreamTranslation(rb_servo::math::Vector3(0.0, 0.0, -0.001), &r);
        stream_removed_total += r;
    }
    CHECK(!gate.streamArmed());
    CHECK(gate.streamForceN() < 1.0);
    CHECK(near(gate.streamTranslation(), 1.0, 1e-9));
    CHECK(tick_removed_total > 0.1);             // the tick channel chopped
    CHECK(near(stream_removed_total, 0.0));      // the stream channel did not
    return true;
}

// *** THE TWO LAWS ARE NOT INTERCHANGEABLE, AND THE RATIO IS WHY. ***
//
// This is the regression for a real hardware finding (2026-08-26): the STREAM law
// was applied to an operator hand-push and the arm turned ~9 deg where
// controller-manager turns ~0.3 deg for the same push. Both laws are "correct" —
// they are just answers to different problems. The stream law may be soft because
// the GATE holding the plan back bounds the force; a compliant Hold has no plan
// advance to gate, so its spring is the only bound there is.
//
// What separates them is not stiffness in the abstract but the RATIO k_r/k_t, which
// decides how much of a push off the control point becomes rotation rather than
// translation.
bool testHoldLawTurnsFarLessThanTheStreamLawForTheSamePush() {
    const rb_servo::ForceControlConfig cfg = shippedLaw();

    const double kt_stream = cfg.stream.translation[0].k;
    const double kr_stream = cfg.stream.rotation[0].k;
    const double kt_hold = cfg.hold.translation[0].k;
    const double kr_hold = cfg.hold.rotation[0].k;

    // The hold law is stiffer on both, and far more so in rotation.
    CHECK(kt_hold > kt_stream * 5.0);
    CHECK(kr_hold > kr_stream * 25.0);
    // THE RATIO: hold resists a torque ~5x more, relative to how it resists a force.
    const double ratio_stream = kr_stream / kt_stream;
    const double ratio_hold = kr_hold / kt_hold;
    CHECK(ratio_hold > ratio_stream * 4.0);

    // The measured push that started this: 13 N with 1.34 Nm about the TCP.
    // Under the stream law that is ~9.6 deg; under the hold law ~0.31 deg.
    const double deg_stream = 1.34 / kr_stream * 180.0 / M_PI;
    const double deg_hold = 1.34 / kr_hold * 180.0 / M_PI;
    CHECK(deg_stream > 9.0);
    CHECK(deg_hold < 0.5);
    return true;
}

// Selecting the law must NOT disturb the deviation. A Hold that becomes a stream (or
// the reverse) mid-contact would otherwise snap the emitted command off the pose it
// is holding — the same reason leaving service freezes rather than retires.
bool testSwappingTheLawKeepsTheDeviation() {
    const rb_servo::ForceControlConfig cfg = shippedLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, 0.002);
    overlay.setLaw(cfg.hold);
    const rb_servo::math::Vector3 f(0.0, 0.0, 25.0);
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 4000; ++i) overlay.step(f, m);
    const double held = overlay.deviation().z();
    CHECK(held > 1e-4);
    overlay.setLaw(cfg.stream);
    CHECK(near(overlay.deviation().z(), held, 1e-12));
    return true;
}

// The hold law's converged deflection is the number an operator checks with a ruler.
bool testHoldLawDeflectsForceOverStiffness() {
    const rb_servo::ForceControlConfig cfg = shippedLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, 0.002);
    overlay.setLaw(cfg.hold);
    const rb_servo::math::Vector3 f(0.0, 0.0, 10.0);
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 20000; ++i) overlay.step(f, m);
    // 10 N / 2499.1 N/m = 4.0 mm
    CHECK(near(overlay.deviation().z(), 10.0 / cfg.hold.translation[2].k, 1e-5));
    return true;
}

// *** THE OVERLAY MUST NOT WIND AGAINST A COMMAND THAT IS NOT REACHING THE ROBOT. ***
//
// It is an open-loop integrator on the measured wrench: if its output never lands,
// the wrench never answers, and it winds until something else stops it. Measured
// 2026-08-26 — the servo stream deadlocked in queue-sync warmup, the arm never moved,
// a hand stayed on the tool, and the deviation wound the command 54 deg out before
// the tracking latch fired on a fault that named the wrong subsystem.
//
// FREEZING is the right answer, not resetting: the deviation already on the wire is
// the pose the arm is holding, and walking it back would command the tool through
// whatever it is resting against.
bool testFreezeHoldsTheDeviationWhileTheCommandIsNotExecuted() {
    const rb_servo::ForceControlConfig cfg = shippedLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, 0.002);
    overlay.setLaw(cfg.hold);

    // A steady contact winds the deviation up.
    const rb_servo::math::Vector3 f(0.0, 0.0, 10.0);
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 2000; ++i) overlay.step(f, m);
    const double held = overlay.deviation().z();
    CHECK(held > 1e-4);

    // The stream stops reaching the robot. The caller stops stepping and freezes.
    overlay.freeze();
    const double after_freeze = overlay.deviation().z();
    CHECK(near(after_freeze, held, 1e-12));       // the pose on the wire is kept
    CHECK(near(overlay.velocity().norm(), 0.0, 1e-12));   // the momentum is stale

    // Ten thousand ticks of the same wrench with nobody stepping it: the deviation
    // must not have moved. Under the 2026-08-26 behaviour this is where 54 deg came
    // from.
    CHECK(near(overlay.deviation().z(), held, 1e-12));
    return true;
}

// A frozen overlay that resumes must pick the deviation back up where it left it,
// not restart from zero — restarting would snap the emitted command off the pose the
// arm is holding.
bool testResumeContinuesFromTheFrozenDeviation() {
    const rb_servo::ForceControlConfig cfg = shippedLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, 0.002);
    overlay.setLaw(cfg.hold);
    const rb_servo::math::Vector3 f(0.0, 0.0, 10.0);
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 500; ++i) overlay.step(f, m);
    const double held = overlay.deviation().z();
    overlay.freeze();
    overlay.step(f, m);                            // one tick after the resume
    // It moved on from `held`, it did not restart at 0.
    CHECK(overlay.deviation().z() > held * 0.9);
    return true;
}

}  // namespace

// THE OSCILLATION GUARD TRIPS ON A LIMIT CYCLE AND NOT ON A PUSH (2026-08-27).
// Amplitude caps bound the per-tick motion but cannot see a sustained oscillation;
// the guard counts velocity-direction reversals at amplitude. A steady push has
// zero reversals; an alternating drive at the incident's scale must freeze
// compliance within the window, hold the deviation, and release only after the
// wrench has been quiet for the release window.
bool testOscillationGuardTripsFreezesAndReleases() {
    rb_servo::ForceControlConfig cfg = shippedLaw();
    cfg.gate_enable = false;
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, 0.002);
    const rb_servo::math::Vector3 zero = rb_servo::math::Vector3::Zero();

    // A steady 25 Nm twist: large, ONE direction (walks into the rotation fence,
    // where the anti-windup zeroes the outward rate). Never trips.
    for (int i = 0; i < 1500; ++i) {
        overlay.step(zero, rb_servo::math::Vector3(0.0, 0.0, 25.0));
    }
    CHECK(overlay.oscillationTrips() == 0);
    CHECK(!overlay.oscillationFrozen());
    overlay.reset();

    // The incident's shape: an alternating torque (the 2026-08-27 blowup ran at
    // ~5.3 Hz on the wrist axes — high velocity, small deviation, far inside the
    // fence). +/-25 Nm at 100-tick period: the rotation rate swings past the
    // amplitude floor in BOTH directions (a shorter period never develops the
    // reverse swing above the floor) while the deviation stays ~0.15 rad, under
    // the 0.26 rad fence. Must trip inside the window and freeze.
    int tripped_at = -1;
    for (int i = 0; i < 2000; ++i) {
        const double sign = ((i / 50) % 2 == 0) ? 1.0 : -1.0;
        overlay.step(zero, rb_servo::math::Vector3(0.0, 0.0, sign * 25.0));
        if (overlay.oscillationFrozen()) {
            tripped_at = i;
            break;
        }
    }
    CHECK(tripped_at >= 0);
    CHECK(overlay.oscillationTrips() == 1);
    CHECK(near(overlay.velocityRot().norm(), 0.0));
    const double frozen_dev = overlay.deviationRot().z();

    // While frozen and still under load, the deviation must NOT move (no
    // integration) and the guard must NOT release (the wrench is not quiet).
    for (int i = 0; i < 500; ++i) {
        overlay.step(zero, rb_servo::math::Vector3(0.0, 0.0, 25.0));
    }
    CHECK(overlay.oscillationFrozen());
    CHECK(near(overlay.deviationRot().z(), frozen_dev));

    // Quiet wrench for the release window: compliance rejoins and a steady
    // torque integrates again toward tau/k.
    const int release_ticks =
        static_cast<int>(cfg.oscillation_release_quiet_sec / 0.002) + 10;
    for (int i = 0; i < release_ticks; ++i) {
        overlay.step(zero, zero);
    }
    CHECK(!overlay.oscillationFrozen());
    overlay.reset();
    for (int i = 0; i < 4000; ++i) {
        overlay.step(zero, rb_servo::math::Vector3(0.0, 0.0, 0.5));
    }
    CHECK(overlay.deviationRot().z() > 0.03);  // 0.5 Nm / 8 = 0.0625 rad target
    CHECK(overlay.oscillationTrips() == 1);    // and not re-tripped
    return true;
}

// THE CONTACT-SHOCK LOW-PASS TAKES THE BURST, NOT THE CONTACT.
// A real contact arrives as a burst: measured 2026-08-27 the compensated |F|
// swung 0 -> 98.6 N and back every few ticks (53.8 N inside one 2 ms tick), and
// the overlay followed it into 1,501 deg/s^2 of commanded acceleration. The
// filter has to flatten that while leaving the STEADY force the law regulates
// against exactly where it was -- otherwise it would change the contact the
// operator tuned k against. Modelled here on the deviation the law produces,
// which is what actually reaches the robot.
bool testWrenchFilterFlattensShockAndKeepsSteadyForce() {
    const double dt = 0.002;
    const double hz = 25.0;
    const double alpha = dt / (1.0 / (2.0 * M_PI * hz) + dt);
    const auto lowpass = [&](double state, double x) { return state + alpha * (x - state); };

    // 1) STEADY force is untouched: 10 N through the filter still settles at F/k.
    {
        rb_servo::control::AdmittanceOverlay overlay;
        overlay.configure(shippedLaw(), dt);
        double s = 0.0;
        const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
        for (int i = 0; i < 20000; ++i) {
            s = lowpass(s, 10.0);
            overlay.step(rb_servo::math::Vector3(0.0, 0.0, s), m);
        }
        CHECK(near(overlay.deviation().z(), 10.0 / 400.0, 1e-4));
    }

    // 2) THE SHOCK ITSELF. The incident's signature was the per-tick jump:
    //    53.8 N inside one 2 ms tick. That is what the filter has to take out,
    //    and it is measured on the wrench, not through the law -- at the
    //    incident's 98.6 N the deviation is pinned on the 40 mm fence (98.6/400
    //    = 246 mm of spring travel), so anything measured downstream of it is
    //    reading the fence's velocity zeroing, not the filter.
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

    // 3) Through the LAW, below the fence, the burst reaches the deviation
    //    smaller. 12 N peaks settle at 30 mm, inside the 40 mm fence, so this
    //    measures the filter and not the clamp.
    const auto peak_deviation_rate = [&](bool filtered) {
        rb_servo::ForceControlConfig cfg = shippedLaw();
        cfg.gate_enable = false;          // isolate the law from the gate
        rb_servo::control::AdmittanceOverlay overlay;
        overlay.configure(cfg, dt);
        const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
        double s = 0.0;
        double worst = 0.0;
        double prev_v = 0.0;
        for (int i = 0; i < 4000; ++i) {
            const double raw = ((i / 3) % 2 == 0) ? 12.0 : 0.0;
            s = lowpass(s, raw);
            overlay.step(rb_servo::math::Vector3(0.0, 0.0, filtered ? s : raw), m);
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

    // 3) The filter is a LAW input, not a motion filter: at 0 Hz the caller feeds
    //    the raw wrench and the behaviour is bit-identical to before.
    {
        rb_servo::control::AdmittanceOverlay a, b;
        a.configure(shippedLaw(), dt);
        b.configure(shippedLaw(), dt);
        const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
        for (int i = 0; i < 500; ++i) {
            const rb_servo::math::Vector3 f(0.0, 0.0, (i % 7) * 3.0);
            a.step(f, m);
            b.step(f, m);
        }
        CHECK(near(a.deviation().z(), b.deviation().z()));
    }
    return true;
}

// ---------------------------------------------------------------------------
// THE k = 0 LAW AND THE FOLD (2026-09-03, controller-manager's live configuration)
// ---------------------------------------------------------------------------

// CM's live follow row, as shipped in stack_real.yaml since 2026-09-03.
rb_servo::ForceControlConfig springlessLaw() {
    rb_servo::ForceControlConfig c = shippedLaw();
    for (int i = 0; i < 3; ++i) {
        c.stream.translation[i] = {rb_servo::ForceAxisMode::Compliance, 12.0, 1000.0, 0.0, 0.0};
        c.stream.rotation[i] = {rb_servo::ForceAxisMode::Compliance, 0.3, 30.0, 0.0, 0.0};
        c.hold.translation[i] = c.stream.translation[i];
        c.hold.rotation[i] = c.stream.rotation[i];
    }
    c.fold_deviation = true;
    c.wrench_filter_hz = 25.0;
    c.gate_close_tau_s = 0.10;
    c.gate_open_tau_s = 1.0;     // 0.40 -> 1.0 with k = 0: a fast re-open feeds the ring
    return c;
}

// THE PURE-DAMPER PREDICATE IS THE WHOLE GATE ON THE FOLD. k == 0 with no force
// target, per triad; a rigid axis neither helps nor hinders; any spring or any
// FORCE-mode axis refuses.
bool testPureDamperPredicate() {
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(shippedLaw(), 0.002);                 // k = 400: a spring
    CHECK(!overlay.pureDamperTranslation());
    CHECK(!overlay.pureDamperRotation());
    rb_servo::ForceControlConfig k0 = springlessLaw();
    overlay.setLaw(k0.stream);
    CHECK(overlay.pureDamperTranslation());
    CHECK(overlay.pureDamperRotation());
    // A rigid axis is admitted (it holds d = 0 by construction).
    rb_servo::ForceLawConfig law = k0.stream;
    law.translation[0].mode = rb_servo::ForceAxisMode::Rigid;
    overlay.setLaw(law);
    CHECK(overlay.pureDamperTranslation());
    // A FORCE-mode axis walks by design and must keep its designed stop (the fence).
    law = k0.stream;
    law.translation[2].mode = rb_servo::ForceAxisMode::Force;
    law.translation[2].ref_force = -10.0;
    overlay.setLaw(law);
    CHECK(!overlay.pureDamperTranslation());
    CHECK(overlay.pureDamperRotation());                    // triads answer independently
    // A spring on one rotation axis refuses the rotation triad only.
    law = k0.stream;
    law.rotation[1].k = 8.0;
    overlay.setLaw(law);
    CHECK(overlay.pureDamperTranslation());
    CHECK(!overlay.pureDamperRotation());
    return true;
}

// dropDeviation() DROPS THE DISPLACEMENT AND KEEPS THE MOMENTUM - the opposite half
// of freeze(). The next tick continues from the live velocity, so a k = 0 law that is
// folded every tick still yields at F/b instead of restarting from rest each tick
// (CM's failed "stateless k = 0": 0.04 mm/tick at 10 N, i.e. no compliance).
bool testDropDeviationKeepsTheVelocity() {
    rb_servo::ForceControlConfig cfg = springlessLaw();
    rb_servo::control::AdmittanceOverlay overlay;
    overlay.configure(cfg, 0.002);
    const rb_servo::math::Vector3 f(0.0, 0.0, 10.0);
    const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
    for (int i = 0; i < 2000; ++i) overlay.step(f, m);      // 4 s: v -> F/b = 10 mm/s
    const double v = overlay.velocity().z();
    CHECK(near(v, 10.0 / 1000.0, 2e-4));
    CHECK(overlay.deviation().z() > 0.01);
    overlay.dropDeviation();
    CHECK(near(overlay.deviation().norm(), 0.0, 1e-15));
    CHECK(near(overlay.velocity().z(), v, 1e-15));
    overlay.step(f, m);
    CHECK(near(overlay.deviation().z(), v * 0.002, 1e-6));  // one tick at the live speed
    // Folding EVERY tick keeps yielding at F/b: 500 ticks of (step, drop) walk 10 mm.
    double walked = 0.0;
    for (int i = 0; i < 500; ++i) {
        overlay.step(f, m);
        walked += overlay.deviation().z();
        overlay.dropDeviation();
    }
    CHECK(near(walked, 0.010, 5e-4));
    return true;
}

// A one-axis plant: a plan streamed at v_cmd into a wall of stiffness k_env, the
// wrench it reports delayed by `delay_ticks` and low-passed like the servo loop does,
// the gate on the plan advance, the overlay on the emitted pose. `fold` books the
// deviation into the plan every tick (the servo loop's foldForceDeviation); without it
// the plan walks into the wall forever and the overlay carries the difference.
struct WallLoop {
    rb_servo::control::AdmittanceOverlay overlay;
    rb_servo::control::ForceGate gate;
    bool fold = false;
    double dt = 0.002;
    double wall_z = 0.010;
    double k_env = 30663.0;
    double v_cmd = 0.050;
    double lpf_hz = 25.0;
    double deadzone_n = 0.0;        // soft per-axis deadzone on what the LAW sees
    bool gate_on_physical = false;  // judge the gate on |F| before the deadzone
    std::vector<double> delay;
    std::size_t head = 0;
    double f_filt = 0.0;
    double f_phys_filt = 0.0;
    bool primed = false;
    double plan_z = 0.0;
    double emitted_z = 0.0;
    double absorbed_z = 0.0;
    double force_seen = 0.0;

    WallLoop(const rb_servo::ForceControlConfig& cfg, bool fold_on, int delay_ticks)
        : fold(fold_on), delay(static_cast<std::size_t>(std::max(1, delay_ticks)), 0.0) {
        overlay.configure(cfg, dt);
        gate.configure(cfg, dt);
        lpf_hz = cfg.wrench_filter_hz;
    }
    void tick() {
        // The sensor reports the contact of `delay` ticks ago.
        const double pen = emitted_z - wall_z;
        delay[head] = pen > 0.0 ? -k_env * pen : 0.0;
        head = (head + 1) % delay.size();
        const double f_raw = delay[head];
        const double f_dz = std::abs(f_raw) <= deadzone_n
            ? 0.0 : (f_raw < 0.0 ? f_raw + deadzone_n : f_raw - deadzone_n);
        if (lpf_hz > 0.0) {
            if (!primed) { f_filt = f_dz; f_phys_filt = std::abs(f_raw); primed = true; }
            else {
                const double tau = 1.0 / (2.0 * M_PI * lpf_hz);
                const double a = std::min(1.0, dt / (tau + dt));
                f_filt += a * (f_dz - f_filt);
                f_phys_filt += a * (std::abs(f_raw) - f_phys_filt);
            }
        } else {
            f_filt = f_dz;
            f_phys_filt = std::abs(f_raw);
        }
        force_seen = f_raw;   // the TRUE contact force (what the wall feels)
        const rb_servo::math::Vector3 f(0.0, 0.0, f_filt);
        const rb_servo::math::Vector3 m = rb_servo::math::Vector3::Zero();
        gate.update(f, m, gate_on_physical ? f_phys_filt : -1.0, -1.0);
        // The plan advance into the wall, projectively gated (the chunk follower's
        // setAdvanceGate does this per segment; per tick is the same law).
        double removed = 0.0;
        const rb_servo::math::Vector3 adv =
            gate.applyTranslation(rb_servo::math::Vector3(0.0, 0.0, v_cmd * dt), &removed);
        plan_z += adv.z();
        overlay.step(f, m, gate.translation());
        emitted_z = plan_z + overlay.deviation().z();
        if (fold) {
            const double d = overlay.deviation().z();
            plan_z += d;
            absorbed_z += d;
            overlay.dropDeviation();
        }
    }
};

// THE FOLD IS A GAUGE CHANGE, CLOSED THROUGH A WALL: with and without it the emitted
// pose is identical on every tick, while the deviation that walks without bound in the
// overlay is, with the fold, exactly the displacement booked into the plan.
bool testFoldIsInvisibleToTheContact() {
    rb_servo::ForceControlConfig cfg = springlessLaw();
    cfg.max_deviation_m = 0.0;     // no fence: the unfolded walk must not be clipped
    WallLoop a(cfg, false, 9), b(cfg, true, 9);
    double max_err = 0.0;
    for (int i = 0; i < 2500; ++i) {
        a.tick();
        b.tick();
        max_err = std::max(max_err, std::abs(a.emitted_z - b.emitted_z));
    }
    std::printf("  fold invariance: max emitted error %.3e m over 5 s; unfolded deviation "
                "%.1f mm, folded plan shift %.1f mm\n",
                max_err, a.overlay.deviation().z() * 1e3, b.absorbed_z * 1e3);
    CHECK(max_err < 1e-9);
    CHECK(near(b.overlay.deviation().z(), 0.0, 1e-15));        // nothing standing
    CHECK(near(a.overlay.deviation().z(), b.absorbed_z, 1e-9)); // ... it moved here
    CHECK(a.overlay.deviation().z() < -0.010);                  // and it IS a walk
    return true;
}

// WITH k = 0 THE CONTACT FORCE IS A BY-PRODUCT, NOT A DESIGNED NUMBER (CM 0028 SS2):
// the plan creeps in at g(F)*v_cmd while the damper retreats at F/b, so a streamed
// contact rests where b*v_cmd*g(F/F_max) = F - here ~7.6 N for 50 mm/s at b = 1000,
// below the 10 N gate and well above zero. And the loop's stability against a rigid
// surface is a DELAY margin (docs/reference/force_control_stability_margin.md):
// b = 1000 at 18 ms of delay is +4.7 dB against 30.7 kN/m and settles; the old
// b = 434.7 / m = 15 row is -3.7 dB there and rings.
bool testSpringlessContactSettlesAtTheDamperFixedPointAndTheOldRowRings() {
    auto run = [](const rb_servo::ForceControlConfig& cfg, double* p2p_last_s, double* f_mean) {
        WallLoop w(cfg, true, 9);           // 18 ms of delay
        double fmin = 1e9, fmax = -1e9, fsum = 0.0;
        for (int i = 0; i < 3000; ++i) {    // 6 s
            w.tick();
            if (i >= 2500) {                // the last second
                fmin = std::min(fmin, w.force_seen);
                fmax = std::max(fmax, w.force_seen);
                fsum += w.force_seen;
            }
        }
        *p2p_last_s = fmax - fmin;
        *f_mean = -fsum / 500.0;
    };
    double p2p_live = 0.0, f_live = 0.0, p2p_old = 0.0, f_old = 0.0;
    run(springlessLaw(), &p2p_live, &f_live);
    rb_servo::ForceControlConfig old = springlessLaw();
    for (int i = 0; i < 3; ++i) {
        old.stream.translation[i].m = 15.0;
        old.stream.translation[i].b = 434.7;
    }
    run(old, &p2p_old, &f_old);
    std::printf("  b=1000/m=12: last-second force %.2f N mean, %.3f N p-p | b=434.7/m=15: "
                "%.2f N mean, %.3f N p-p\n", f_live, p2p_live, f_old, p2p_old);
    // Fixed point b*v_cmd*g(F/10) = F for b*v_cmd = 50 N is F = 7.6 N.
    CHECK(f_live > 6.5 && f_live < 8.7);
    CHECK(p2p_live < 0.5);                  // settled flat (0.00 N p-p in the model)
    CHECK(p2p_old > 4.0);                   // the old row rings (16 N p-p in the model)
    // The same live law with the OLD gate re-open (0.40 s) rings 9 N p-p: the gate is a
    // second loop, and its re-open speed is what feeds the damper ring at this delay.
    rb_servo::ForceControlConfig fast_gate = springlessLaw();
    fast_gate.gate_open_tau_s = 0.40;
    double p2p_fast = 0.0, f_fast = 0.0;
    run(fast_gate, &p2p_fast, &f_fast);
    std::printf("  b=1000 with gate open_tau 0.40: %.2f N mean, %.3f N p-p\n", f_fast, p2p_fast);
    CHECK(p2p_fast > 4.0);
    return true;
}

// THE HAND-GUIDE LATCH IS A HYSTERESIS, NOT A THRESHOLD: nothing moves below engage,
// once moving it keeps yielding down to release, and it never toggles inside the band.
// Disabled (engage 0) it is always engaged - the pre-2026-09-03 behaviour.
bool testHoldEngageLatchHysteresis() {
    rb_servo::control::HoldEngageLatch latch;
    latch.configure(5.0, 2.0);
    CHECK(latch.enabled());
    CHECK(!latch.engaged());                 // starts frozen
    CHECK(!latch.update(3.0));               // inside the band from below: still frozen
    CHECK(!latch.update(4.99));
    CHECK(latch.update(5.0));                // reaches engage
    CHECK(latch.update(3.0));                // inside the band from above: keeps yielding
    CHECK(latch.update(2.01));
    CHECK(!latch.update(2.0));               // reaches release
    CHECK(!latch.update(4.0));               // and stays frozen until engage again
    latch.update(6.0);
    CHECK(latch.engaged());
    latch.reset();
    CHECK(!latch.engaged());
    rb_servo::control::HoldEngageLatch off;
    off.configure(0.0, 0.0);
    CHECK(!off.enabled());
    CHECK(off.engaged());
    CHECK(off.update(0.0));
    return true;
}

// A SPRING UNDER THE GATE HOLDS THE CONFIGURED FORCE AT ANY SPEED (CM 0028, the
// 2026-09-04 stream law: k 400 / m 6 / b 500, gate 10 N judged on the PHYSICAL force
// through a 3 N deadzone). The pure-damper law settles wherever gate creep and damper
// retreat balance - a by-product of speed - and limit-cycles at the policy's 100+ mm/s.
bool testSpringUnderTheGateHoldsTheConfiguredForce() {
    auto sustained = [](double k, bool physical, double v_cmd, double* ripple) {
        rb_servo::ForceControlConfig cfg = springlessLaw();
        for (int i = 0; i < 3; ++i) cfg.stream.translation[i] = {rb_servo::ForceAxisMode::Compliance, 6.0, 500.0, k, 0.0};
        cfg.gate_max_force_n = 10.0;
        WallLoop w(cfg, k == 0.0, 13);      // 26 ms of delay; fold only on the pure damper
        w.k_env = 15000.0;
        w.v_cmd = v_cmd;
        w.deadzone_n = 3.0;
        w.gate_on_physical = physical;
        double fmin = 1e9, fmax = -1e9, fsum = 0.0;
        for (int i = 0; i < 6000; ++i) {
            w.tick();
            if (i >= 5000) { fmin = std::min(fmin, -w.force_seen); fmax = std::max(fmax, -w.force_seen); fsum += -w.force_seen; }
        }
        *ripple = fmax - fmin;
        return fsum / 1000.0;
    };
    double r = 0.0;
    std::printf("  sustained contact force, gate 10 N (true force):\n");
    for (double v : {0.02, 0.05, 0.135}) {
        const double f_spring = sustained(400.0, true, v, &r);
        double r0 = 0.0;
        const double f_damper = sustained(0.0, false, v, &r0);
        std::printf("    v=%3.0f mm/s: spring+gate %.1f N (p-p %.2f) | pure damper %.1f N (p-p %.2f)\n",
                    v * 1e3, f_spring, r, f_damper, r0);
        CHECK(f_spring > 9.0 && f_spring < 12.0);   // the configured 10 N, +/- the deadzone's bite
        CHECK(r < 1.0);                              // and it HOLDS there
    }
    // Judged on the deadzoned wrench the same law settles ~3 N high.
    const double f_dz = sustained(400.0, false, 0.05, &r);
    std::printf("    v= 50 mm/s judged on the deadzoned wrench: %.1f N\n", f_dz);
    CHECK(f_dz > 12.0);
    // The pure damper at the policy's speed limit-cycles.
    double r_lc = 0.0;
    sustained(0.0, false, 0.135, &r_lc);
    CHECK(r_lc > 4.0);
    return true;
}

// strip() IS THE INVERSE OF compose(), position and rotation, so a plan-side stage
// reading FK of the sent joints gets the nominal back exactly.
bool testStripInvertsCompose() {
    rb_servo::control::AdmittanceOverlay overlay;
    rb_servo::ForceControlConfig cfg = shippedLaw();
    for (int i = 0; i < 3; ++i) cfg.stream.rotation[i] = {rb_servo::ForceAxisMode::Compliance, 1.0, 15.72, 8.0, 0.0};
    overlay.configure(cfg, 0.002);
    const rb_servo::math::Vector3 f(3.0, -7.0, 10.0);
    const rb_servo::math::Vector3 m(0.4, 0.2, -0.3);
    for (int i = 0; i < 500; ++i) overlay.step(f, m);
    CHECK(overlay.hasDeviation());
    rb_servo::Pose6D nominal;
    nominal.x = 0.5; nominal.y = -0.2; nominal.z = 0.1;
    nominal.rx = 3.0; nominal.ry = -0.1; nominal.rz = 1.5;
    const rb_servo::Pose6D emitted = overlay.compose(nominal);
    const rb_servo::Pose6D back = overlay.strip(emitted);
    CHECK(near(back.x, nominal.x, 1e-12) && near(back.y, nominal.y, 1e-12) && near(back.z, nominal.z, 1e-12));
    const rb_servo::math::Matrix3 r_err =
        rb_servo::math::rotationFromPose(back).transpose() * rb_servo::math::rotationFromPose(nominal);
    CHECK(near(r_err.trace(), 3.0, 1e-9));
    rb_servo::control::AdmittanceOverlay quiet;
    quiet.configure(cfg, 0.002);
    CHECK(!quiet.hasDeviation());
    return true;
}

// THE GATE'S MAGNITUDE OVERRIDE: the fade is judged on the number handed in, the
// direction still on the vector.
bool testGateMagnitudeOverride() {
    rb_servo::ForceControlConfig cfg = shippedLaw();
    cfg.gate_close_tau_s = 0.002;   // one tick
    rb_servo::control::ForceGate gate;
    gate.configure(cfg, 0.002);
    const rb_servo::math::Vector3 small(0.0, 0.0, 1.0);   // the deadzoned wrench: 1 N
    gate.update(small, rb_servo::math::Vector3::Zero(), 12.0, 0.0);   // physical: 12 N
    CHECK(near(gate.forceN(), 12.0));
    CHECK(gate.translation() < 0.01);                    // closed on the physical magnitude
    const rb_servo::math::Vector3 adv(0.0, 0.0, -0.001);  // into the +z force
    double removed = 0.0;
    gate.applyTranslation(adv, &removed);
    CHECK(removed > 0.00099);                           // and it cuts along the vector's direction
    gate.update(small, rb_servo::math::Vector3::Zero());  // no override: the vector's own 1 N
    CHECK(near(gate.forceN(), 1.0));
    return true;
}

// THE GATE ON THE ABSOLUTE-TARGET PATH holds the tracker's STATE, not its goal: the
// advance into the contact is cut, the inward momentum dropped, sliding untouched, and
// the goal still where the source put it - so a released contact leaves no offset.
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
    tracker.updateGoalFromCommand(goal);      // latches the reference
    goal.z = 0.050;                           // 50 mm DOWN, through a surface at z = 0.09
    goal.x = 0.020;                           // and 20 mm sideways (sliding)
    tracker.updateGoalFromCommand(goal);
    rb_servo::control::ForceGate gate;
    rb_servo::ForceControlConfig fc = shippedLaw();
    fc.gate_close_tau_s = 0.002;
    gate.configure(fc, 0.002);
    // THE SHIPPED PATH USES THE STREAM CHANNEL (2026-09-04 pm): judged on the slow
    // force vector with a dwell, so a wall pushing +z at 12 N arms it after ~150 ms.
    const rb_servo::math::Vector3 wall(0.0, 0.0, 12.0);   // wall pushes +z
    for (int i = 0; i < 200; ++i) {
        gate.update(wall, rb_servo::math::Vector3::Zero());
        gate.updateStream(wall);
    }
    CHECK(gate.streamArmed());
    CHECK(gate.streamTranslation() < 0.01);
    double z_min = 1.0, x_last = 0.0;
    for (int i = 0; i < 500; ++i) {
        gate.update(wall, rb_servo::math::Vector3::Zero());
        gate.updateStream(wall);
        const rb_servo::Pose6D before = tracker.currentPose();
        rb_servo::Pose6D out = tracker.step(0.002);
        const rb_servo::math::Vector3 p0(before.x, before.y, before.z), p1(out.x, out.y, out.z);
        double removed = 0.0;
        const rb_servo::math::Vector3 kept = gate.applyStreamTranslation(p1 - p0, &removed);
        if (removed > 0.0) {
            tracker.constrainTranslation(p0 + kept, gate.streamForceDirection(),
                                         1.0 - gate.streamTranslation());
        }
        const rb_servo::Pose6D now = tracker.currentPose();
        z_min = std::min(z_min, now.z);
        x_last = now.x;
    }
    CHECK(z_min > 0.100 - 1e-6);              // never advanced into the +z force
    CHECK(x_last > 0.015);                    // but slid sideways toward the goal
    CHECK(near(tracker.goalPose().z, 0.050, 1e-9));   // the goal is untouched
    // Release: the gate opens, the tracker resumes toward the goal from rest, no jump.
    for (int i = 0; i < 21; ++i) {
        gate.update(rb_servo::math::Vector3::Zero(), rb_servo::math::Vector3::Zero());
        gate.updateStream(rb_servo::math::Vector3::Zero());
    }
    const rb_servo::Pose6D a = tracker.step(0.002);
    const rb_servo::Pose6D b = tracker.step(0.002);
    CHECK(std::abs(b.z - a.z) < 1e-4);        // one tick of ordinary SMD motion, not a lunge
    return true;
}

bool testExternallyVerifiedSensorDoesNotGrantTareOrAcceptInvalidWrench() {
    auto cfg = cellConfig();
    cfg.tool_mass_kg = 0.0;
    auto pipe = rb_servo::sensor::FtPipeline{};
    pipe.configure(cfg, 0.002);
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

int main() {
    testExternallyVerifiedSensorDoesNotGrantTareOrAcceptInvalidWrench();
    testPoseTrackGateHoldsStateNotGoal();
    testSpringUnderTheGateHoldsTheConfiguredForce();
    testStripInvertsCompose();
    testGateMagnitudeOverride();
    testHoldEngageLatchHysteresis();
    testPureDamperPredicate();
    testDropDeviationKeepsTheVelocity();
    testFoldIsInvisibleToTheContact();
    testSpringlessContactSettlesAtTheDamperFixedPointAndTheOldRowRings();
    testWrenchFilterFlattensShockAndKeepsSteadyForce();
    testOscillationGuardTripsFreezesAndReleases();
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
    testGateStreamChannelIgnoresVibrationAndHoldsSustainedContact();
    testGateStreamChannelAveragesOutAFlippingDirection();
    testGateReopensToExactlyOneAfterRelease();
    testHoldFoldDeltaFloorAndCap();
    testHoldLawTurnsFarLessThanTheStreamLawForTheSamePush();
    testSwappingTheLawKeepsTheDeviation();
    testHoldLawDeflectsForceOverStiffness();
    testFreezeHoldsTheDeviationWhileTheCommandIsNotExecuted();
    testResumeContinuesFromTheFrozenDeviation();
    if (g_failures == 0) std::printf("force control tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
