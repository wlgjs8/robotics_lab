// chunk_follower_core.hpp — pure kinematic core for the Ruckig receding-horizon
// chunk-follower. No servo-loop / Pinocchio / Eigen dependencies: it is a thin,
// unit-testable layer over ruckig that implements the settled control math —
//   * the per-segment BVP guards (clamp, forward-safe, corner, co-clamp),
//   * Ruckig-owned v/a/j limiting with minimum_duration=dt; infeasible segments
//     degrade by duration growth (T-slip),
//   * alpha telemetry compatibility with the removed sacrifice ladder,
//   * per-segment state chaining from the ruckig OUTPUT (not measured).
//
// It operates on an abstract N-dim axis vector; the caller maps Cartesian
// position (3) and, later, the orientation rotation-vector (3) into the axes.
// Orientation log3/exp3 lives in the servo-loop layer (se3.cpp), not here.
//
// SPIKE FINDINGS baked in (see tools/ruckig_spike.cpp):
//   - ruckig 0.9.2 honors non-zero target_acceleration + minimum_duration.
//   - at dt=33ms the convergence regime is JERK-limited: |Δv|_cap = j·dt²/4.
//   - construct once + PRE-WARM (first calculate() is a ~3ms one-time hit).

#pragma once

#include <ruckig/ruckig.hpp>

#include <array>
#include <cmath>
#include <cstddef>

namespace rb_servo::control {

// Per-axis kinematic limits (Cartesian; fixed conservative values — the
// joint→cartesian derivation is deliberately out of scope for now).
struct AxisLimit {
  double v_max{1.0};
  double a_max{6.0};
  double j_max{30.0};
};

// Reachable |Δv| in a FIXED time T with boundary accel a0=af=0, jerk+accel
// limited. Piecewise: jerk-triangle below the accel-saturation knee, trapezoid
// above. At the follower's dt this returns the jerk branch (a_max is slack).
inline double dvCapacity(double a_max, double j_max, double T) {
  const double knee = 2.0 * a_max / j_max;
  return (T >= knee) ? a_max * (T - a_max / j_max) : 0.25 * j_max * T * T;
}

// A self-consistent Taylor sample of the reference at the target grid point:
// pf = p_k, vf = central 1st diff, af = central 2nd diff (all per axis).
template <std::size_t N>
struct BoundarySample {
  std::array<double, N> pf{};
  std::array<double, N> vf{};
  std::array<double, N> af{};
  // sign of the two flanking step displacements d_k, d_{k+1}, per axis — used
  // for corner (direction-reversal) detection. +1 / 0 / -1.
  std::array<int, N> sign_dk{};
  std::array<int, N> sign_dkp1{};
};

struct GuardConfig {
  // Feedforward accel damping, (0, 1], SPLIT BY AXIS CLASS (axes 0-2 linear, 3-5 angular).
  //
  // af is a second difference of the policy knots divided by dt^2 (see buildSample), so it
  // multiplies knot noise by 1/dt^2 -- ~900x at 30 Hz -- and grows QUADRATICALLY as dt shrinks:
  // the same commanded path demands 2x the af at SPEED_SCALE 1.0 that it does at 0.75.
  //
  // 2026-07-31 measurement (E1 rollouts, right arm, SPEED_SCALE 1.0) showed the two axis classes
  // are nowhere near each other, which is why a single scalar could not be tuned:
  //     translation af p90 = 1.53 m/s^2   =  31% of a_max   (headroom)
  //     rotation    af p90 = 8.56 rad/s^2 =  95% of a_max   (saturated)
  // With the rotation target alone consuming 95% of the acceleration limit, ruckig could not reach
  // the knot inside dt -- `converged` 1.5% and projection error p90 6.3 mm on the right arm, versus
  // 70.6% / 0.13 mm on the left. So beta=1.0 was not buying fidelity, it was losing it.
  //
  // The earlier single-scalar sweep (2026-07-24: 1.0/0.8/0.6) was abandoned because 0.6 cost
  // descent authority (right z_min 16-17 mm vs 9-15 mm). That regression is TRANSLATION -- the very
  // axis class with headroom. Damping the two classes together traded away real final-approach
  // deceleration to suppress rotational noise. Split so each can be set on its own evidence.
  //
  // DO NOT reach for these to fix trembling. Damping the rotation class was proposed from the 95%
  // reading above and REFUTED by the follower_replay sweep: relaxing beta made feasibility WORSE.
  // The cost of a segment is |af_target - a0| / dt, not |af| -- a0 already carries the honest
  // curvature, so shrinking af_target enlarges that mismatch and spends jerk arriving flatter than
  // the arm is actually travelling. af is a boundary condition, not a demand. The binding
  // constraint was the angular limits themselves (see stack_real.yaml ruckig_follower).
  //
  // The split is kept because it makes the knob measurable per axis class: one scalar could not
  // separate rotational noise from the translational final-approach deceleration, which is why the
  // 2026-07-24 sweep lost descent authority with no roughness win.
  double af_damping_beta_lin{0.85};
  double af_damping_beta_ang{0.85};
  double eps_clamp{1e-9};         // limit clamp tolerance
  // Corner (direction-reversal) detection deadbands, per axis class. A flanking
  // step displacement below the deadband contributes sign 0, which cannot form a
  // reversal pair -- so a LARGER deadband makes the guard ignore small wobble.
  // These must be sized against the actual per-step displacement: at 30 Hz the
  // policy commands ~2 mm and ~0.3 deg per step, and 0.3 deg spread over three
  // rotation axes leaves per-axis components only a few times the 5e-4 rad
  // (0.029 deg) default -- so rotational sign noise trips the guard on a large
  // fraction of segments even when the motion is visually smooth.
  double corner_deadband_lin_m{3e-4};
  double corner_deadband_ang_rad{5e-4};
  // Ring-down applied to the target velocity of an axis whose flanking steps
  // reverse (its target acceleration is always zeroed). 1.0 = no ring-down.
  // NOTE the cost is not confined to the reversing axis: ruckig runs its default
  // TIME synchronization here (buildInput sets minimum_duration but never
  // in.synchronization), so braking one axis stretches the whole 6-DoF segment
  // duration and can push it past dt -- i.e. rotational wobble slows translation.
  double corner_velocity_scale{0.25};
  int    ladder_rungs{3};         // retained for compatibility; predictive ladder is unused
};

// Result of building + solving one segment.
struct SegmentSolve {
  ruckig::Result result{ruckig::Result::Error};
  double duration{0.0};
  // Where the chained state was taken on the solved trajectory [s]. Legacy: dt. With
  // the core time-stretch it is clamp(duration, dt, chain_max), i.e. the knot itself
  // whenever the stretch covers the whole overrun (see solve()).
  double chain_at_sec{0.0};
  double alpha{1.0};              // telemetry-compatible; predictive alpha is disabled
  bool   converged{false};       // duration ≈ dt (within tol) → in converging regime
  bool   corner{false};          // any axis rang down af→0 this segment
  // Ruckig's independent axis times and guarded boundary conditions, copied
  // from this solve without additional OTG calls. Axes 0..2 stand translation;
  // 3..5 the current R0_ref-local rotation tangent, not Euler angles.
  std::array<double, 6> axis_duration_sec{};
  std::array<double, 6> target_velocity{};
  std::array<double, 6> target_acceleration{};
  // Optional deadline-aware jerk search. A common scale <= 1 is applied only
  // when the SAME guarded BVP can still finish within the policy period. This
  // is a trajectory choice below the configured caps, not a new motion limit.
  double jerk_scale{1.0};
  int jerk_search_calculations{0};
};

template <std::size_t N>
class ChunkFollowerSegment {
 public:
  ChunkFollowerSegment(const std::array<AxisLimit, N>& limits, double dt,
                       GuardConfig guard = {}, bool preserve_current_state = false)
      : limits_(limits), dt_(dt), guard_(guard),
        preserve_current_state_(preserve_current_state) {
    prewarm();
  }

  ChunkFollowerSegment(const ChunkFollowerSegment&) = default;
  // Ruckig has const DOF fields and therefore no assignment operator. Retain
  // this destination's prewarmed calculator; calculate(input, trajectory) takes
  // all boundary state explicitly. The trajectory, chained state and sample
  // gauge below are the complete persistent state of this wrapper.
  ChunkFollowerSegment& operator=(const ChunkFollowerSegment& source) {
    if (this == &source) return *this;
    limits_ = source.limits_;
    dt_ = source.dt_;
    guard_ = source.guard_;
    preserve_current_state_ = source.preserve_current_state_;
    last_traj_ = source.last_traj_;
    have_traj_ = source.have_traj_;
    p0_ = source.p0_;
    v0_ = source.v0_;
    a0_ = source.a0_;
    sample_offset_ = source.sample_offset_;
    have_state_ = source.have_state_;
    return *this;
  }

  // Swap limits/guards in place (profile change). The ruckig OTG object is
  // limit-agnostic (limits ride on each input), so no reconstruction and no
  // re-prewarm; chained state and the current trajectory are dropped.
  void reconfigure(const std::array<AxisLimit, N>& limits, double dt, GuardConfig guard,
                   bool preserve_current_state = false) {
    limits_ = limits;
    dt_ = dt;
    guard_ = guard;
    preserve_current_state_ = preserve_current_state;
    have_state_ = false;
    have_traj_ = false;
  }

  // Seed the chained start state (called on (re)activation). v0/a0 default to
  // zero for a clean latch, or to the previous controller's state for a
  // continuous handoff.
  void seed(const std::array<double, N>& p0,
            const std::array<double, N>& v0 = {},
            const std::array<double, N>& a0 = {}) {
    p0_ = p0; v0_ = v0; a0_ = a0; have_state_ = true;
  }

  bool hasState() const { return have_state_; }
  // A sampled-state preemption must never replay the displaced trajectory if
  // both the next solve and its ring-down fail. Keep p/v/a, drop samples only.
  void discardTrajectory() { have_traj_ = false; sample_offset_ = {}; }

  // TRANSLATE THE CHAINED STATE AND THE IN-FLIGHT SEGMENT BY `delta`, keeping the
  // velocity/acceleration state untouched. This is the force overlay's FOLD landing
  // in the plan: the displacement the admittance law grew this tick is moved out of
  // the overlay and into the reference chain, so the next solve starts from the pose
  // the arm was actually commanded to, and the samples still to be consumed from
  // the current segment are shifted with it (a Ruckig trajectory is translation-
  // invariant, so shifting both endpoints shifts every sample). `p0_` is the END
  // state of the last solve; the sample offset carries the same delta into
  // sample() until the next solve, which starts from the shifted p0_ and therefore
  // needs no offset of its own.
  void shiftPosition(const std::array<double, N>& delta) {
    if (!have_state_) return;
    for (std::size_t d = 0; d < N; ++d) {
      p0_[d] += delta[d];
      sample_offset_[d] += delta[d];
    }
  }
  // Update the segment period (policy_dt may vary per chunk frame). Cheap; does
  // not re-prewarm (allocation already done at construction).
  void setDt(double dt) { dt_ = dt; }
  double dt() const { return dt_; }
  const std::array<double, N>& p0() const { return p0_; }
  const std::array<double, N>& v0() const { return v0_; }
  const std::array<double, N>& a0() const { return a0_; }

  // Predictive alpha gate is intentionally disabled. The old finite rung ladder
  // diluted target vf/af for acceleration cases, but inverted the braking case:
  // when |vf| < |v0|, shrinking vf can increase |α·vf - v0| and force α down to
  // the floor, commanding an even harder brake. Keep the signature and
  // SegmentSolve.alpha for telemetry compatibility; Ruckig's v/a/j limits plus
  // minimum_duration=dt are the envelope, and infeasible targets degrade by
  // duration growth (T-slip).
  double predictAlpha(const BoundarySample<N>& s) const {
    (void)s;
    return 1.0;
  }

  // Forward-safe invariant (independent predictor, evaluated at the undilated
  // target): ‖v0‖ ≤ √(vf² + 2·a_max·d) per axis. If violated, the start velocity
  // is too high to reach (pf,vf) without overshoot → the segment will T-slip and
  // decelerate; α cannot help. Exposed for telemetry / the will-slip flag.
  bool forwardSafe(const BoundarySample<N>& s) const {
    for (std::size_t d = 0; d < N; ++d) {
      const double d_disp = std::fabs(s.pf[d] - p0_[d]);
      const double bound = std::sqrt(s.vf[d] * s.vf[d] + 2.0 * limits_[d].a_max * d_disp);
      if (std::fabs(v0_[d]) > bound + 1e-9) return false;
    }
    return true;
  }

  // Build the guarded BVP, solve, and — on success — CHAIN the start state from
  // the ruckig output at t = chain_at, where chain_at = clamp(duration, dt, chain_max).
  //
  // chain_max < 0 (the default) means chain_max = dt: the legacy contract, one policy
  // period per segment, the overrun of an infeasible knot truncated and re-targeted
  // from wherever the profile got. A chain_max above dt is the CORE TIME-STRETCH
  // (2026-09-06): a knot the limits cannot reach in one period is reached in the time
  // the time-optimal profile actually needs, the chained state is the knot itself
  // (with its target velocity), and the caller lets the segment last that long. The
  // executed path is then the policy's path at the follower's limits instead of a
  // truncated one; the plan clock is what slows down. Bounded by chain_max so a
  // pathological solve (a corner ring-down of several periods) cannot stall the clock.
  SegmentSolve solve(const BoundarySample<N>& sample, double chain_max_sec = -1.0,
                     bool minimize_deadline_jerk = false) {
    SegmentSolve out;
    if (!have_state_) return out;

    const double alpha = predictAlpha(sample);
    ruckig::InputParameter<N> in;
    bool corner = buildInput(sample, alpha, in);

    ruckig::Trajectory<N> traj;
    out.result = otg_.calculate(in, traj);
    if (out.result != ruckig::Result::Working) return out;

    // Time-optimal Ruckig profiles normally spend j_max even for a small
    // correction. If the original BVP already meets this period, spend that
    // existing time budget on a less abrupt profile. Keep the original solve
    // as the known-feasible fallback: targets, all guards, and the minimum
    // duration are built ONCE and never softened during this search.
    //
    // Six bisections give a bounded RT cost and a scale resolution of 1/64.
    // Synchronization can have blocked duration intervals, so this is a
    // bounded feasible search, not a claim of a globally minimum jerk. Every
    // accepted candidate is checked; an invalid or late candidate cannot
    // replace the original trajectory. Infeasible-deadline segments retain
    // their original time-stretch/braking behavior.
    const double deadline = dt_ + 1e-9;
    if (minimize_deadline_jerk && traj.get_duration() <= deadline &&
        currentStateWithinLimits(in)) {
      double lo = 0.0;
      double hi = 1.0;
      for (int iteration = 0; iteration < 6; ++iteration) {
        const double scale = 0.5 * (lo + hi);
        auto candidate = in;
        for (std::size_t d = 0; d < N; ++d) {
          candidate.max_jerk[d] = in.max_jerk[d] * scale;
        }
        // A lower jerk must not enlarge an initial-state brake beyond the
        // velocity envelope. Already out-of-envelope states use the original
        // full-authority recovery; their physical p/v/a is never clipped.
        if (!currentStateWithinLimits(candidate)) {
          lo = scale;
          continue;
        }
        ruckig::Trajectory<N> candidate_traj;
        ++out.jerk_search_calculations;
        const auto result = otg_.calculate(candidate, candidate_traj);
        if (result == ruckig::Result::Working &&
            candidate_traj.get_duration() <= deadline) {
          hi = scale;
          out.jerk_scale = scale;
          traj = candidate_traj;
        } else {
          lo = scale;
        }
      }
    }

    out.duration = traj.get_duration();
    const auto independent = traj.get_independent_min_durations();
    for (std::size_t d = 0; d < std::min<std::size_t>(N, 6); ++d) {
      out.axis_duration_sec[d] = independent[d];
      out.target_velocity[d] = in.target_velocity[d];
      out.target_acceleration[d] = in.target_acceleration[d];
    }
    out.alpha = alpha;
    out.corner = corner;
    out.converged = out.duration <= dt_ * 1.001 + 1e-9;

    // Chain (p0,v0,a0) ← ruckig OUTPUT at t=chain_at (never from measured encoder).
    const double chain_max = chain_max_sec > dt_ ? chain_max_sec : dt_;
    const double chain_at = std::min(std::max(out.duration, dt_), chain_max);
    out.chain_at_sec = chain_at;
    std::array<double, N> p, v, a;
    traj.at_time(chain_at, p, v, a);
    p0_ = p; v0_ = v; a0_ = a;
    last_traj_ = traj;
    have_traj_ = true;
    sample_offset_ = {};   // the new trajectory was solved FROM the shifted p0_
    return out;
  }

  // Sample the current segment's trajectory at time t∈[0,chain_at] (per-2ms tick).
  bool sample(double t, std::array<double, N>& p, std::array<double, N>& v,
              std::array<double, N>& a) const {
    if (!have_traj_) return false;
    last_traj_.at_time(t, p, v, a);
    for (std::size_t d = 0; d < N; ++d) p[d] += sample_offset_[d];
    return true;
  }

 private:
  static bool currentStateWithinLimits(const ruckig::InputParameter<N>& in) {
    for (std::size_t d = 0; d < N; ++d) {
      const double v = in.current_velocity[d];
      const double a = in.current_acceleration[d];
      if (std::abs(v) > in.max_velocity[d] ||
          std::abs(a) > in.max_acceleration[d]) return false;
      // Velocity reached while bringing the current acceleration to zero at
      // this jerk. Lowering jerk is allowed only if that brake stays within
      // the original velocity cap (same viability condition as target guards).
      const double brake_velocity = v + std::copysign(
          a * a / (2.0 * in.max_jerk[d]), a);
      if (std::abs(brake_velocity) > in.max_velocity[d]) return false;
    }
    return true;
  }

  void prewarm() {
    // Absorb the ~3ms first-calculate() allocation off the RT path.
    ruckig::InputParameter<N> in;
    for (std::size_t d = 0; d < N; ++d) {
      in.target_position[d] = 0.001;
      in.max_velocity[d] = limits_[d].v_max;
      in.max_acceleration[d] = limits_[d].a_max;
      in.max_jerk[d] = limits_[d].j_max;
    }
    in.minimum_duration = dt_;
    ruckig::Trajectory<N> traj;
    otg_.calculate(in, traj);
  }

  static double clampAbs(double x, double lim, double eps) {
    const double c = lim * (1.0 + eps);
    return x > c ? c : (x < -c ? -c : x);
  }
  // Clamp strictly INSIDE the limit. Ruckig validates the TARGET state against the
  // limits with a plain `vf > vMax` (ruckig.hpp validate_input, check_target_state_
  // within_limits=true), so a target clamped to lim*(1+eps) -- 5e-10 above -- is an
  // ErrorInvalidInput and the whole segment solve fails. MEASURED 2026-09-06,
  // servo_log_20260906_123643.csv: 557 failed solves on the left arm, ALL inside the
  // 118.68-120.0 s window where the policy asked the wrist for 53-84 deg/s against a
  // 0.9 rad/s cap -- every knot faster than the limit broke the solve instead of
  // saturating, the plan clock kept consuming knots against a frozen core (projection
  // error 18.5 deg), and the run ended on the divergence latch. Right arm: 33 at
  // 29.22 s. Targets are clamped inside the limit; the legacy current state keeps
  // clampAbs. Continuous mode instead lets Ruckig brake from the exact state.
  static double clampInside(double x, double lim, double eps) {
    const double c = lim * (1.0 - eps);
    return x > c ? c : (x < -c ? -c : x);
  }

  // Legacy Δv-capacity feasibility of a dilated sample on ALL axes (cheap; no
  // ruckig call). Retained alongside dvCapacity for documentation/debugging; the
  // production predictive alpha gate no longer calls it.
  bool dvFeasibleAtAlpha(const BoundarySample<N>& s, double alpha) const {
    for (std::size_t d = 0; d < N; ++d) {
      const double vf = alpha * s.vf[d];
      const double dv = std::fabs(vf - v0_[d]);
      const double cap = dvCapacity(limits_[d].a_max, limits_[d].j_max, dt_);
      if (dv > cap) return false;
    }
    return true;
  }

  // Fill a ruckig input with guarded, α-dilated boundary conditions.
  // Returns true if any axis rang down af→0 at a corner.
  bool buildInput(const BoundarySample<N>& s, double alpha,
                  ruckig::InputParameter<N>& in) const {
    bool corner = false;
    for (std::size_t d = 0; d < N; ++d) {
      const auto& L = limits_[d];
      // A re-linearized sampled state can already lie outside a component limit
      // in the new tangent. Clipping it invents an instantaneous v/a change.
      // In the continuous mode, give Ruckig the exact current state and let its
      // brake profile return within the SAME limits/jerk; target guards below
      // remain unchanged. The legacy mode retains its epsilon clamp.
      in.current_position[d] = p0_[d];
      in.current_velocity[d] = preserve_current_state_ ? v0_[d] :
          clampAbs(v0_[d], L.v_max, guard_.eps_clamp);
      in.current_acceleration[d] = preserve_current_state_ ? a0_[d] :
          clampAbs(a0_[d], L.a_max, guard_.eps_clamp);

      // target: pf unchanged; vf/af time-dilated by α (vf·α, af·α²) then damped.
      // Axes 0-2 are stand-frame translation, 3-5 the rotation vector about R0_ref_ (see
      // CartesianChunkFollower::buildSample), the same split the corner deadbands already use.
      // N<=3 instantiations (the pure-translation unit tests) take the linear beta on every axis,
      // so they are unchanged as long as the two betas are equal.
      const double af_beta =
          (d < 3) ? guard_.af_damping_beta_lin : guard_.af_damping_beta_ang;
      double vf = alpha * s.vf[d];
      double af = alpha * alpha * s.af[d] * af_beta;

      // corner: direction reversal between the flanking steps → ring down af,
      // and pull vf toward zero (do NOT carry acceleration through a reversal).
      if (s.sign_dk[d] != 0 && s.sign_dkp1[d] != 0 &&
          s.sign_dk[d] != s.sign_dkp1[d]) {
        af = 0.0;
        vf *= guard_.corner_velocity_scale;  // ring-down toward the corner
        corner = true;
      }

      in.current_position[d] = p0_[d];
      in.target_position[d] = s.pf[d];
      // Target state strictly inside the limits (see clampInside). Ruckig additionally
      // requires the velocity just BEFORE the target -- vf +/- af^2/(2 j_max), the
      // velocity from which jerking the acceleration to af arrives at vf -- to be
      // within [vMin, vMax], so a non-zero af eats into the reachable |vf|.
      af = clampInside(af, L.a_max, guard_.eps_clamp);
      double vf_hi = L.v_max * (1.0 - guard_.eps_clamp);
      double vf_lo = -vf_hi;
      const double af_margin = (af * af) / (2.0 * L.j_max);
      if (af < 0.0) vf_hi = std::max(0.0, vf_hi - af_margin);
      if (af > 0.0) vf_lo = std::min(0.0, vf_lo + af_margin);
      in.target_velocity[d] = std::min(std::max(vf, vf_lo), vf_hi);
      in.target_acceleration[d] = af;
      in.max_velocity[d] = L.v_max;
      in.max_acceleration[d] = L.a_max;
      in.max_jerk[d] = L.j_max;
    }
    in.minimum_duration = dt_;
    return corner;
  }

  std::array<AxisLimit, N> limits_{};
  double dt_{1.0 / 30.0};
  GuardConfig guard_{};
  bool preserve_current_state_{false};

  ruckig::Ruckig<N> otg_{};
  ruckig::Trajectory<N> last_traj_{};
  bool have_traj_{false};

  std::array<double, N> p0_{}, v0_{}, a0_{};
  // Accumulated shiftPosition() since the last solve (see shiftPosition).
  std::array<double, N> sample_offset_{};
  bool have_state_{false};
};

}  // namespace rb_servo::control
