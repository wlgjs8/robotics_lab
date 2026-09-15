// admittance_overlay.hpp - THE force law on the emitted Cartesian target, and the
// gate on the source's advance. Ported from controller-manager's
// `adm::AdmittanceOverlay` (src/arm/motions/AdmittanceOverlay.h) and its FOLLOW
// path, then unified on 2026-09-15: one law for every source (a streamed plan, a
// Hold, an absolute teleop target), on the force VECTOR, isotropic, one-sided.
//
// THE MODEL (translation, stand frame):
//
//     m * v' + b * v = (|F| - rest_force_n)+ * F_hat          k = 0, always
//
//   d = the DEVIATION from the nominal pose OF THIS TICK, integrated from v.
//   F = the compensated, PRE-deadzone physical force at the TCP (the caller
//       low-passes it with force_control.wrench_filter_hz), F_hat its direction.
//   |F| <= rest_force_n is an equilibrium in EVERY direction: the drive is zero, the
//       velocity decays with tau = m/b, nothing moves. Free space is never sought,
//       so there is no walk for a fence to have to stop.
//   |F| >  rest_force_n: the arm yields ALONG THE MEASURED FORCE at (|F| - rest)/b,
//       whatever the direction. It is the vector that is one-sided, not each axis:
//       a diagonal 10 N push must read 10 N, not 5.8 N per axis.
//   Rotation is RIGID: er_ and w_ stay zero (compose/strip keep their rotation
//       algebra for the day a ref-torque design brings rotational compliance back).
//
// THE STATE LIVES IN THE STAND FRAME and the law is isotropic, so there is no
// workspace triad to re-aim and no Coriolis question: a push in any direction is the
// same law.
//
// THE DEVIATION IS PER-EPISODE and, on a fold path, per-TICK: the servo loop hands
// it to the source's plan every tick (dropDeviation) and this object keeps only the
// velocity. `reset()` is what every episode edge must call.
//
// LEAVING SERVICE FREEZES the displacement and DROPS the momentum - it does NOT walk
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

    // COLD: adopt the law (force_control.law, b already derived by the loader) and
    // zero the state.
    void configure(const ForceControlConfig& cfg, double control_period_sec);

    // Per-episode. Clears the deviation, the velocity and the fence latch.
    void reset();

    // RT: one tick of the law, driven by the PHYSICAL (compensated, pre-deadzone,
    // filtered) STAND-frame wrench referenced at the TCP. The torque is read only by
    // the oscillation guard's release test; rotation is rigid.
    //
    // THE GATE DOES NOT ENTER HERE. It used to multiply the drive, which cancels at
    // the operating point: v_cmd = v_s*g against a yield g*(F-ref)/b solves to
    // F = ref + b*v_s, i.e. the stream speed is back in the converged force - exactly
    // what the crossing exists to remove. The gate attenuates the SOURCE's advance
    // and nothing else.
    void step(const math::Vector3& force_phys_stand, const math::Vector3& torque_phys_stand);

    // RT: the overlay is leaving service. FREEZE the displacement, DROP the momentum.
    // The stored momentum is stale after a pause; a resume re-derives it from the
    // live wrench rather than replaying it.
    void freeze();

    // Compose the deviation onto a nominal pose. The pivot is the TCP - the same
    // point the wrench is referenced at - so the tool turns about its own control
    // point and there is no lever term to get the sign of wrong.
    //
    // *** THE WRENCH REFERENCE POINT AND THE COMPOSE PIVOT MOVE TOGETHER, ALWAYS. ***
    // A torque referenced at one point driving a rotation about another is not a
    // tuning choice, it is a frame error: a pure lateral force at the TCP carries
    // zero torque about the TCP but L*F about the sensor origin, so mixing them
    // makes a straight push twist the tool.
    Pose6D compose(const Pose6D& nominal_stand) const;

    // THE INVERSE OF compose(): the nominal an emitted pose was composed from, given
    // the deviation standing NOW. Every plan-side stage that anchors on "where the
    // robot was last commanded" (the chunk follower's cold seed / re-anchor / lead
    // guard, the pose-track SMD's reseed, the Hold source's latch) reads FK of the
    // sent joints - which is nominal + deviation. On a fold path the deviation is ~0
    // and it does not matter; on the absolute-target path (fold declined) it stands
    // in the overlay and an unstripped anchor would compose it twice.
    Pose6D strip(const Pose6D& emitted_stand) const;
    bool hasDeviation(double eps_m = 1e-9, double eps_rad = 1e-9) const {
        return dp_.norm() >= eps_m || er_.norm() >= eps_rad;
    }

    bool quiescent(double eps_m = 1e-6) const;
    bool bounded() const { return bounded_; }

    // RT: THE DISPLACEMENT HAS BEEN TAKEN OVER BY THE PLAN - drop our copy of it.
    //
    // *** ONLY A CALLER THAT HAS ALREADY APPLIED THE SAME DISPLACEMENT TO THE PLAN MAY
    // CALL THIS. *** Zeroing without transferring does not remove an offset, it STRIPS
    // one: the emitted command loses the whole deviation in a single tick.
    //
    // THE MOMENTUM IS KEPT, and that is the whole difference from freeze(). vp_ is
    // the live dynamics: m*v' + b*v = drive is a first-order law in the VELOCITY, so
    // dropping it would restart the wrench response from rest every tick and flatten
    // the law to a rigid one (CM's failed 2026-08-25 "stateless k=0" attempt: 0.04 mm
    // per tick at 10 N, i.e. no force control at all). The gauge argument that makes
    // the hand-off exact: with k = 0 the displacement is a bare integrator nothing
    // reads back, so (nominal, d) -> (nominal + d, 0) leaves the ODE's future, the
    // emitted pose and the plan's tracking error all invariant.
    void dropDeviation();
    const math::Vector3& deviation() const { return dp_; }        // [m], stand
    const math::Vector3& deviationRot() const { return er_; }     // [rad] rotvec, stand (0: rigid)
    const math::Vector3& velocity() const { return vp_; }         // [m/s], stand
    const math::Vector3& velocityRot() const { return w_; }       // [rad/s], stand (0: rigid)
    double mass() const { return cfg_.law.m; }
    double damping() const { return cfg_.law.b; }
    double restForceN() const { return cfg_.gate_rest_force_n; }

    // Oscillation guard (cfg.oscillation_*): a sustained run of velocity-direction
    // reversals at meaningful amplitude is a limit cycle, never an operator's push.
    // While latched, step() holds the deviation frozen (momentum dropped) and only
    // releases after the wrench has stayed below the release thresholds for
    // release_quiet_sec. The rate caps bound amplitude per tick; this bounds the
    // thing they cannot see (measured 2026-08-27: a ~5.3 Hz coupled oscillation grew
    // INSIDE the caps until wrist IK amplification saturated every joint).
    bool oscillationFrozen() const { return osc_frozen_; }
    uint64_t oscillationTrips() const { return osc_trips_; }

private:
    void applyFence();
    void stepOscillationGuard();

    ForceControlConfig cfg_{};
    double dt_ = 0.002;

    math::Vector3 dp_ = math::Vector3::Zero();   // translation deviation [m], stand
    math::Vector3 vp_ = math::Vector3::Zero();   // its velocity [m/s]
    math::Vector3 er_ = math::Vector3::Zero();   // rotation deviation [rad] rotvec (rigid: 0)
    math::Vector3 w_ = math::Vector3::Zero();    // its rate [rad/s] (rigid: 0)
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

// THE FORCE GATE. The source advance's reflection ratio falls as the contact force
// rises - CM's own framing: *"힘이 큰 방향으로는 조심스럽게 움직인다"*.
//
// ONE CURVE, AND ITS CROSSING IS THE DESIGN (CM 0049, adopted 2026-09-11). The ratio
// is not a fade to zero: it returns an ABSOLUTE speed at the declared force, so the
// gate curve and the law's yield line cross AT that force for every demand. See
// ForceControlConfig's gate block for the equations and the measurements.
//
// APPLIED PROJECTIVELY BY ITS CONSUMERS, and that is the whole design: scaling the
// WHOLE advance would kill sliding along a contact surface AND would throttle backing
// OUT of it, which is exactly the escape an operator needs. Only the component
// pushing INTO the contact is attenuated. The gate itself carries no apply: the chunk
// follower (setAdvanceGate), the pose-track stage and the preview QP each cut along
// the ONE normal this object publishes, so the three cannot drift apart.
//
// THE DIRECTION IS THE MEASURED FORCE (2026-09-15). It was declared (a tool-frame
// axis) from 2026-09-11 to 2026-09-15 because with a law that yielded on ONE axis
// only, judging the gate on |F| along the measured direction closed it for lateral
// forces nothing yielded against (the 14:12 deadlock). With the law isotropic along
// F_hat that argument is gone - law and gate share the direction by construction - and
// the measured direction is what "any direction" means. Below kContactNormalNoiseBandN
// the normal is zero: there the sign is genuinely undefined and the gate is ~1
// anyway (g(0) = 1, and (|F|/peak)^2 keeps g > 0.99 below ~1.5 N), so a consumer
// handed a zero normal removes nothing. There is no switch at the operating point.
class ForceGate {
public:
    // The sign band on |F| [N] below which no contact normal is published. A noise
    // floor, far below rest_force_n on purpose: putting a switch at the rest force
    // toggled the advance authority between 1 % and 100 % at the wrench's ripple rate
    // (52 Hz on both arms, servo_log_20260911_134703).
    static constexpr double kContactNormalNoiseBandN = 0.5;

    void configure(const ForceControlConfig& cfg, double control_period_sec);
    void reset();

    // RT: fold this tick's PHYSICAL (pre-deadzone, filtered) stand wrench into the
    // gate (the CM 0049 curve; see ForceControlConfig for the derivation).
    //   g = (v_cross/v_s)^((|F|/peak_force_n)^q),  v_cross = (peak-rest)/b = peak_vel
    // and g = 1 whenever the source demands no more than v_cross - below that speed
    // F = rest + b*v_s cannot reach the declaration, so the gate has nothing to give.
    // `source_demand_m_s` is the SOURCE's demanded advance speed, pre-gate. It must
    // be neither the achieved speed nor the law's own yield: at the operating point
    // the achieved speed IS v_cross, so a gate fed its own output reads g = 1,
    // re-opens, and the crossing is gone; and a Hold whose yield was read back as
    // demand closed to 0.095 during a hand push (servo_log_20260915_112545).
    void update(const math::Vector3& force_phys_stand, const math::Vector3& torque_phys_stand,
                double source_demand_m_s);

    double translation() const { return gate_t_; }
    double forceN() const { return force_n_; }
    double torqueNm() const { return torque_nm_; }
    // Unit direction of the measured force (the wall's push on the tool = the
    // free-space direction), stand frame; zero when no force stands.
    const math::Vector3& forceDirection() const { return force_dir_; }
    // THE ONE CONTACT NORMAL every consumer cuts along: forceDirection() when |F| is
    // above the noise band, zero otherwise. Unit-or-zero, which is what the preview
    // QP's input validation requires.
    const math::Vector3& contactNormal() const { return contact_normal_; }
    bool closed() const { return gate_t_ < 0.02; }

    // ---- WHAT THE GATE ACTUALLY RAN AT (CM 0049's columns) -------------------
    double bEff() const { return b_eff_; }
    double mEff() const { return m_eff_; }
    double crossSpeedMs() const { return v_cross_; }
    double demandMs() const { return demand_; }

private:
    static double snapOpen(double g);   // 1 - 1e-6 < g  ->  exactly 1.0
    ForceControlConfig cfg_{};
    double dt_ = 0.002;
    double gate_t_ = 1.0;
    math::Vector3 force_dir_ = math::Vector3::Zero();
    math::Vector3 contact_normal_ = math::Vector3::Zero();
    double force_n_ = 0.0;
    double torque_nm_ = 0.0;
    // The curve's derived numbers, fixed at configure() from the declared pair.
    double b_eff_ = 0.0;
    double m_eff_ = 0.0;
    double v_cross_ = 0.0;
    double demand_ = 0.0;
};

}  // namespace control
}  // namespace rb_servo
