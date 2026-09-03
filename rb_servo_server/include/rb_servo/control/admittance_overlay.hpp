// admittance_overlay.hpp - the per-axis admittance law, ported from
// controller-manager's `adm::AdmittanceOverlay` (src/arm/motions/AdmittanceOverlay.h)
// and its FOLLOW path's tuning, which is the consumer whose problem matches ours: a
// streamed Cartesian plan that must yield to contact without abandoning the plan.
//
// THE MODEL, PER AXIS:   m*d'' + b*d' + k*d = wrench
//   d = the DEVIATION from the nominal (followed) pose OF THIS TICK — not from a
//       frozen entry pose. A steady force deflects F/k and b decides how it gets
//       there.
//   COMPLIANCE (k >= 0): springs back to the nominal. k = 0 is the legal pure
//                        mass-damper.
//   FORCE:               no stiffness — the deviation WALKS until the measured
//                        wrench along that axis equals ref_force. In free space that
//                        walk meets the FENCE, which is its designed stop.
//   RIGID (m <= 0 or b < 0): the axis does not deviate at all.
//
// THE STATE LIVES IN THE STAND FRAME; the axis dynamics are diagonal in the
// WORKSPACE frame (the tool's, re-aimed every tick). The state is rotated in,
// integrated per axis, and rotated back. Storing the displacement in a rotating
// frame without a Coriolis term would make the offset physically SWEEP as the tool
// turns — push, then twist the wrist, and the offset follows the wrist, as though
// the wall had moved.
//
// THE DEVIATION IS PER-EPISODE. It is an offset from the nominal of its own tick, so
// it means nothing across an episode boundary: the next episode's nominal restarts
// at the live pose, and re-composing an old offset onto it teleports the command.
// `reset()` is what every episode edge must call.
//
// LEAVING SERVICE FREEZES the displacement and DROPS the momentum — it does NOT walk
// the deviation back. Under contact the nominal chain is INSIDE the workpiece (the
// deviation is what was holding the command at the surface), so retiring would
// command the tool the whole deviation DEEPER. CM measured that as 25 mm of
// penetration at five times the streaming envelope before replacing the ramp.
//
// RT-safe: no allocation, no locks, no I/O.
#pragma once

#include <array>
#include <cstdint>

#include "rb_servo/config/config.hpp"
#include "rb_servo/math/se3.hpp"

namespace rb_servo {
namespace control {

class AdmittanceOverlay {
public:
    AdmittanceOverlay() = default;

    // COLD: adopt the law and zero the state.
    void configure(const ForceControlConfig& cfg, double control_period_sec);

    // Swap the LAW without touching the integration state — resetting the deviation
    // mid-motion would snap the emitted command off the pose it is holding. Called
    // PER TICK by whoever owns the tick, because the streaming law and the
    // hand-push law are different laws for different problems (see ForceLawConfig).
    void setLaw(const ForceLawConfig& law) { law_ = law; }
    const ForceLawConfig& law() const { return law_; }

    // Per-episode. Clears the deviation, the velocity and the fence latch.
    void reset();

    // Re-aim the per-axis triad. The axis law is only meaningful in a frame the task
    // reasons in, and for a streamed tool path that is the TOOL's: a per-axis
    // {m,b,k} diagonal in the stand frame means the axis meanings rotate as the arm
    // moves, which is not what "soft along the approach, stiff laterally" can mean.
    // Called PER TICK, not latched at start.
    void setWorkspaceFrame(const math::Matrix3& r_stand_ws) { r_ws_ = r_stand_ws; }
    const math::Matrix3& workspaceFrame() const { return r_ws_; }

    // RT: one tick of the virtual dynamics, driven by the STAND-frame wrench
    // referenced at the TCP. `gate` (0..1) throttles the FORCE-mode walk only — a
    // gated axis stops WALKING, it does not stop yielding.
    void step(const math::Vector3& force_stand, const math::Vector3& torque_stand,
              double gate = 1.0);

    // RT: the overlay is leaving service. FREEZE the displacement, DROP the momentum.
    // The stored momentum is stale after a pause; a resume re-derives it from the
    // live wrench rather than replaying it.
    void freeze();

    // Compose the deviation onto a nominal pose. The pivot is the TCP — the same
    // point the wrench is referenced at — so the tool turns about its own control
    // point and there is no lever term to get the sign of wrong.
    //
    // *** THE WRENCH REFERENCE POINT AND THE COMPOSE PIVOT MOVE TOGETHER, ALWAYS. ***
    // A torque referenced at one point driving a rotation about another is not a
    // tuning choice, it is a frame error: a pure lateral force at the TCP carries
    // zero torque about the TCP but L*F about the sensor origin, so mixing them
    // makes a straight push twist the tool.
    Pose6D compose(const Pose6D& nominal_stand) const;

    bool quiescent(double eps_m = 1e-6) const;
    bool bounded() const { return bounded_; }

    // THE PURE MASS-DAMPER PREDICATE - the exact condition under which the deviation
    // may be HANDED OFF to the plan that produced the nominal (the FOLD, ported from
    // controller-manager's `AdmittanceOverlay::pure_damper_t/r` + `Arm::absorb_overlay_offset`).
    //
    // *** WHY k == 0 IS AN ALGEBRAIC BOUNDARY AND NOT A TASTE JUDGEMENT. *** step()'s
    // acceleration is (f - b*v - k*d)/m. At k == 0 the d term VANISHES IDENTICALLY: d
    // stops being a state of the law and becomes a bare integrator of v that nothing
    // reads back. Moving a displacement out of d and into the plan's own reference is
    // then a GAUGE CHANGE - (nominal, d) -> (nominal + delta, d - delta) leaves the
    // ODE's future, the emitted pose and the plan's tracking error all invariant. At
    // ANY k != 0 the -k*d term revives, the spring's origin follows the transfer, and
    // the identical edit becomes CM's 2026-08-04 force-ceiling leash (the ORIGIN WALK
    // reverted on hardware the same day).
    //
    // A FORCE-mode axis (ref_force != 0) is EXCLUDED DELIBERATELY, not for want of an
    // argument about its k: such an axis is DESIGNED to walk until it meets the fence
    // when nothing presses back, and handing that walk to the plan would delete the
    // designed stop. RIGID axes are admitted: they hold d = 0 by construction and
    // contribute nothing to the transfer either way.
    //
    // Translation and rotation answer INDEPENDENTLY; the caller folds only when BOTH
    // say yes, because the plan is shifted as one SE(3) displacement.
    bool pureDamperTranslation() const { return pureDamperTriad(law_.translation); }
    bool pureDamperRotation() const { return pureDamperTriad(law_.rotation); }

    // RT: THE DISPLACEMENT HAS BEEN TAKEN OVER BY THE PLAN - drop our copy of it.
    //
    // *** ONLY A CALLER THAT HAS ALREADY APPLIED THE SAME DISPLACEMENT TO THE PLAN MAY
    // CALL THIS. *** Zeroing without transferring does not remove an offset, it STRIPS
    // one: the emitted command loses the whole deviation in a single tick.
    //
    // THE MOMENTUM IS KEPT, and that is the whole difference from freeze(). vp_/w_ are
    // the live dynamics: m*dd + b*d' = w is a first-order law in the VELOCITY, so
    // dropping it would restart the wrench response from rest every tick and flatten
    // the axis to a rigid one (CM's failed 2026-08-25 "stateless k=0" attempt: 0.04 mm
    // per tick at 10 N, i.e. no force control at all).
    void dropDeviation();
    const math::Vector3& deviation() const { return dp_; }        // [m], stand
    const math::Vector3& deviationRot() const { return er_; }     // [rad] rotvec, stand
    const math::Vector3& velocity() const { return vp_; }         // [m/s], stand
    const math::Vector3& velocityRot() const { return w_; }       // [rad/s], stand

    // Oscillation guard (cfg.oscillation_*): a sustained run of velocity-direction
    // reversals at meaningful amplitude is a limit cycle, never an operator's push.
    // While latched, step() holds the deviation frozen (momentum dropped) and only
    // releases after the wrench has stayed below the release thresholds for
    // release_quiet_sec. The per-part caps bound amplitude per tick; this bounds
    // the thing they cannot see (measured 2026-08-27: a ~5.3 Hz coupled oscillation
    // grew INSIDE the caps until wrist IK amplification saturated every joint).
    bool oscillationFrozen() const { return osc_frozen_; }
    uint64_t oscillationTrips() const { return osc_trips_; }

private:
    void applyFence();
    void stepOscillationGuard(const math::Vector3& force_stand,
                              const math::Vector3& torque_stand);
    static bool pureDamperTriad(const std::array<ForceAxisConfig, 3>& axes);

    ForceControlConfig cfg_{};
    ForceLawConfig law_{};
    double dt_ = 0.002;
    math::Matrix3 r_ws_ = math::Matrix3::Identity();

    math::Vector3 dp_ = math::Vector3::Zero();   // translation deviation [m], stand
    math::Vector3 vp_ = math::Vector3::Zero();   // its velocity [m/s]
    math::Vector3 er_ = math::Vector3::Zero();   // rotation deviation [rad] rotvec
    math::Vector3 w_ = math::Vector3::Zero();    // its rate [rad/s]
    bool bounded_ = false;

    // Oscillation guard state (see oscillationFrozen()).
    static constexpr int kOscRingSize = 16;
    uint64_t osc_tick_ = 0;
    bool osc_frozen_ = false;
    uint64_t osc_trips_ = 0;
    uint64_t osc_quiet_ticks_ = 0;
    std::array<math::Vector3, 2> osc_prev_v_{math::Vector3::Zero(),
                                             math::Vector3::Zero()};
    std::array<std::array<uint64_t, kOscRingSize>, 2> osc_reversal_ticks_{};
    std::array<int, 2> osc_reversal_head_{0, 0};
};

// The FORCE GATE. The plan advance's reflection ratio falls as the contact force
// rises — CM's own framing: *"힘이 큰 방향으로는 조심스럽게 움직인다"*.
//
// APPLIED PROJECTIVELY, and that is the whole design: scaling the WHOLE advance
// would kill sliding along a contact surface AND would throttle backing OUT of it,
// which is exactly the escape an operator needs. Only the component pushing INTO the
// measured wrench is attenuated.
//
// THE FORCE DIRECTION is used rather than the deviation direction because it is
// available AT FIRST CONTACT, when the deviation is still ~0 and a deviation-directed
// projection would do nothing at the moment it is most needed.
class ForceGate {
public:
    void configure(const ForceControlConfig& cfg, double control_period_sec);
    void reset();

    // RT: fold this tick's wrench into the gate. Both channels use a KNEE-LESS
    // smoothstep — g(0) = 1 and g(1) = 0 EXACTLY, both endpoints with zero slope, so
    // free space costs nothing and the advance can actually reach a full stop (which
    // is what creates the equilibrium that bounds the deviation at F/k). A knee
    // would double the gate's loop gain, which is why one is refused at load.
    void update(const math::Vector3& force_stand, const math::Vector3& torque_stand);

    // Attenuate one plan advance. Returns the surviving advance; `removed` reports
    // the magnitude taken out, so a log can say how much the gate actually did.
    math::Vector3 applyTranslation(const math::Vector3& advance_stand, double* removed) const;
    math::Vector3 applyRotation(const math::Vector3& advance_stand, double* removed) const;

    double translation() const { return gate_t_; }
    double rotation() const { return gate_r_; }
    double forceN() const { return force_n_; }
    double torqueNm() const { return torque_nm_; }
    bool closed() const { return gate_t_ < 0.02 || gate_r_ < 0.02; }

private:
    ForceControlConfig cfg_{};
    double dt_ = 0.002;
    double gate_t_ = 1.0;
    double gate_r_ = 1.0;
    math::Vector3 force_dir_ = math::Vector3::Zero();
    math::Vector3 torque_dir_ = math::Vector3::Zero();
    double force_n_ = 0.0;
    double torque_nm_ = 0.0;
};

// THE HAND-GUIDE ENGAGEMENT LATCH (2026-09-03). A Schmitt trigger on the physical
// (pre-deadzone) force magnitude that decides whether the HOLD law integrates at all.
//
// WHY: with k = 0 the hold law has no restoring force, so the only thing between a
// parasitic wrist force and motion is the F/T deadzone. Measured on the RB5 right arm
// (servo_log_20260903_205241, 228 s with the hand OFF): the compensated force sat
// above the 2 N deadzone on 7.6 % of ticks and the arm crawled 38 times on its own
// (0.2-2.1 mm each, up to 2.3 mm/s), once for 18 s along a steady 1-3 N pull at the
// F/T housing that also turned the tool 7.4 deg. A hand that MEANS to guide the arm
// pushes far harder than that. So: nothing moves until |F| reaches engage_n, and
// once moving it keeps yielding down to release_n, below which the overlay freezes
// (momentum dropped - the damper's own tau is m/b = 12 ms anyway).
//
// engage_n <= 0 disables the latch (always engaged: the pre-2026-09-03 behaviour).
// RT-safe, state = one bool.
class HoldEngageLatch {
public:
    void configure(double engage_n, double release_n);
    void reset() { engaged_ = false; }
    // Fold this tick's physical |F| in and return whether the hold law may integrate.
    bool update(double force_magnitude_n);
    bool engaged() const { return !enabled() || engaged_; }
    bool enabled() const { return engage_n_ > 0.0; }
    double engageN() const { return engage_n_; }
    double releaseN() const { return release_n_; }

private:
    double engage_n_ = 0.0;
    double release_n_ = 0.0;
    bool engaged_ = false;
};

}  // namespace control
}  // namespace rb_servo
