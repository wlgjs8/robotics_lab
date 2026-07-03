// chunk_follower_core.hpp — pure kinematic core for the Ruckig receding-horizon
// chunk-follower. No servo-loop / Pinocchio / Eigen dependencies: it is a thin,
// unit-testable layer over ruckig that implements the settled control math —
//   * the a_max/jerk predictive convergence gate (validated in ruckig_spike),
//   * the per-segment BVP guards (clamp, forward-safe, corner, co-clamp),
//   * the bounded sacrifice ladder with true time-dilation (vf*α, af*α²),
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
  double af_damping_beta{0.85};   // (0.7, 1]; damps the feedforward af
  double eps_clamp{1e-9};         // limit clamp tolerance
  int    ladder_rungs{3};         // bounded sacrifice-ladder attempts (no unbounded bisection)
};

// Result of building + solving one segment.
struct SegmentSolve {
  ruckig::Result result{ruckig::Result::Error};
  double duration{0.0};
  double alpha{1.0};              // time-dilation factor actually applied (1.0 = none)
  bool   converged{false};       // duration ≈ dt (within tol) → in converging regime
  bool   corner{false};          // any axis rang down af→0 this segment
};

template <std::size_t N>
class ChunkFollowerSegment {
 public:
  ChunkFollowerSegment(const std::array<AxisLimit, N>& limits, double dt,
                       GuardConfig guard = {})
      : limits_(limits), dt_(dt), guard_(guard) {
    prewarm();
  }

  // Swap limits/guards in place (profile change). The ruckig OTG object is
  // limit-agnostic (limits ride on each input), so no reconstruction and no
  // re-prewarm; chained state and the current trajectory are dropped.
  void reconfigure(const std::array<AxisLimit, N>& limits, double dt, GuardConfig guard) {
    limits_ = limits;
    dt_ = dt;
    guard_ = guard;
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
  // Update the segment period (policy_dt may vary per chunk frame). Cheap; does
  // not re-prewarm (allocation already done at construction).
  void setDt(double dt) { dt_ = dt; }
  double dt() const { return dt_; }
  const std::array<double, N>& p0() const { return p0_; }
  const std::array<double, N>& v0() const { return v0_; }
  const std::array<double, N>& a0() const { return a0_; }

  // Predictive convergence gate (Δv capacity): the sacrifice ladder dilates the
  // TARGET (vf·α, af·α²) to bring |vf − v0| inside the jerk/accel Δv capacity.
  // Returns the largest α∈(0,1] predicted feasible on all axes, or the smallest
  // rung if none fit (caller then lets T slip). NOTE: α cannot fix an over-fast
  // START velocity — that is the forward-safe case below, which needs T-slip.
  double predictAlpha(const BoundarySample<N>& s) const {
    for (int r = 0; r < guard_.ladder_rungs; ++r) {
      const double alpha = 1.0 - static_cast<double>(r) / guard_.ladder_rungs;  // 1.0, .66, .33
      if (dvFeasibleAtAlpha(s, alpha)) return alpha;
    }
    return 1.0 / guard_.ladder_rungs;  // most-dilated rung; still may slip
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

  // Build the guarded BVP, run the (bounded) sacrifice ladder, solve, and — on
  // success — CHAIN the start state from the ruckig output at t=dt.
  SegmentSolve solve(const BoundarySample<N>& sample) {
    SegmentSolve out;
    if (!have_state_) return out;

    const double alpha = predictAlpha(sample);
    ruckig::InputParameter<N> in;
    bool corner = buildInput(sample, alpha, in);

    ruckig::Trajectory<N> traj;
    out.result = otg_.calculate(in, traj);
    if (out.result != ruckig::Result::Working) return out;

    out.duration = traj.get_duration();
    out.alpha = alpha;
    out.corner = corner;
    out.converged = out.duration <= dt_ * 1.001 + 1e-9;

    // Chain (p0,v0,a0) ← ruckig OUTPUT at t=dt (never from measured encoder).
    std::array<double, N> p, v, a;
    traj.at_time(dt_, p, v, a);
    p0_ = p; v0_ = v; a0_ = a;
    last_traj_ = traj;
    have_traj_ = true;
    return out;
  }

  // Sample the current segment's trajectory at time t∈[0,dt] (per-2ms tick).
  bool sample(double t, std::array<double, N>& p, std::array<double, N>& v,
              std::array<double, N>& a) const {
    if (!have_traj_) return false;
    last_traj_.at_time(t, p, v, a);
    return true;
  }

 private:
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

  // Δv-capacity feasibility of a dilated sample on ALL axes (cheap; no ruckig
  // call). This is what the ladder's α actually addresses.
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
      // start: chained state, ε-clamped to limits.
      in.current_position[d] = p0_[d];
      in.current_velocity[d] = clampAbs(v0_[d], L.v_max, guard_.eps_clamp);
      in.current_acceleration[d] = clampAbs(a0_[d], L.a_max, guard_.eps_clamp);

      // target: pf unchanged; vf/af time-dilated by α (vf·α, af·α²) then damped.
      double vf = alpha * s.vf[d];
      double af = alpha * alpha * s.af[d] * guard_.af_damping_beta;

      // corner: direction reversal between the flanking steps → ring down af,
      // and pull vf toward zero (do NOT carry acceleration through a reversal).
      if (s.sign_dk[d] != 0 && s.sign_dkp1[d] != 0 &&
          s.sign_dk[d] != s.sign_dkp1[d]) {
        af = 0.0;
        vf *= 0.25;  // ring-down; keeps vf & af co-scaled toward the corner
        corner = true;
      }

      in.current_position[d] = p0_[d];
      in.target_position[d] = s.pf[d];
      in.target_velocity[d] = clampAbs(vf, L.v_max, guard_.eps_clamp);
      in.target_acceleration[d] = clampAbs(af, L.a_max, guard_.eps_clamp);
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

  ruckig::Ruckig<N> otg_{};
  ruckig::Trajectory<N> last_traj_{};
  bool have_traj_{false};

  std::array<double, N> p0_{}, v0_{}, a0_{};
  bool have_state_{false};
};

}  // namespace rb_servo::control
