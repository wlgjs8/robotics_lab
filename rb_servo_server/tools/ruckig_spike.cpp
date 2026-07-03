// ruckig_spike.cpp — throwaway pre-flight spike for the Cartesian chunk-follower.
//
// Resolves the plan's BLOCKER (SPIKE-1) and validates the user's a_max/jerk
// "non-convergence" capacity condition (SPIKE-2) against the REAL solver, before
// any BVP code is written. Not wired into CMake; compile standalone:
//
//   g++ -std=c++17 -O2 -I/opt/ros/humble/include \
//       rb_servo_server/tools/ruckig_spike.cpp \
//       -L/opt/ros/humble/lib/x86_64-linux-gnu -lruckig -Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu \
//       -o /tmp/ruckig_spike && /tmp/ruckig_spike
//
// System ruckig is 0.9.2 (ROS Humble).

#include <ruckig/ruckig.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>

using ruckig::Ruckig;
using ruckig::InputParameter;
using ruckig::Trajectory;
using ruckig::Result;

static constexpr double DT = 1.0 / 30.0;  // 33.33 ms chunk step

// Reachable |Δv| in a FIXED time T, boundary accel a0=af=0, jerk+accel limited.
//   accel-saturated (T >= 2*a_max/j_max):  Δv = a_max*(T - a_max/j_max)   (trapezoid area)
//   jerk-limited     (T <  2*a_max/j_max):  Δv = j_max*T^2/4               (triangle area)
static double dvCapacity(double a_max, double j_max, double T) {
  const double t_ramp2 = 2.0 * a_max / j_max;
  if (T >= t_ramp2) return a_max * (T - a_max / j_max);
  return j_max * T * T / 4.0;
}

static const char* regime(double a_max, double j_max, double T) {
  return (T >= 2.0 * a_max / j_max) ? "accel-saturated" : "JERK-limited";
}

int main() {
  int failures = 0;
  auto check = [&](bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++failures;
  };

  // Representative CONSERVATIVE Cartesian linear limits (fixed; joint->cartesian
  // derivation deliberately excluded per the current scope).
  const double V = 1.0;    // m/s
  const double A = 6.0;    // m/s^2
  const double J = 30.0;   // m/s^3

  std::printf("=== ruckig %s spike (dt=%.4f s) ===\n", "0.9.2", DT);
  std::printf("limits: v=%.2f a=%.2f j=%.2f | 2*a/j=%.3fs -> at dt the regime is %s\n\n",
              V, A, J, 2 * A / J, regime(A, J, DT));

  // -------------------------------------------------------------------------
  // SPIKE-1: non-zero target acceleration + minimum_duration = dt.
  // Does 0.9.2 accept af!=0 and return a valid trajectory terminating at
  // (pf, vf, af)? This is the BVP's foundation.
  // -------------------------------------------------------------------------
  std::printf("SPIKE-1: af != 0 with minimum_duration = dt\n");
  {
    Ruckig<1> otg;
    InputParameter<1> in;
    in.current_position     = {0.0};
    in.current_velocity     = {0.10};
    in.current_acceleration = {0.0};
    in.target_position      = {0.10 * DT + 0.5 * 0.3 * DT * DT};  // a plausible pf
    in.target_velocity      = {0.13};
    in.target_acceleration  = {0.30};   // <-- non-zero af (feedforward curvature)
    in.max_velocity         = {V};
    in.max_acceleration     = {A};
    in.max_jerk             = {J};
    in.minimum_duration     = DT;

    Trajectory<1> traj;
    Result res = otg.calculate(in, traj);
    check(res == Result::Working, "calculate returns Working with af!=0 + min_duration");

    std::array<double, 1> p, v, a;
    traj.at_time(traj.get_duration(), p, v, a);
    const double ep = std::fabs(p[0] - in.target_position[0]);
    const double ev = std::fabs(v[0] - in.target_velocity[0]);
    const double ea = std::fabs(a[0] - in.target_acceleration[0]);
    std::printf("    duration=%.5f s (min=%.5f)  terminal err: p=%.2e v=%.2e a=%.2e\n",
                traj.get_duration(), DT, ep, ev, ea);
    check(ep < 1e-6 && ev < 1e-6 && ea < 1e-6, "trajectory terminates exactly at (pf,vf,af)");
    check(traj.get_duration() >= DT - 1e-9, "duration honors minimum_duration floor");

    // at_time(dt) must return acceleration (needed for state chaining a0/alpha0).
    std::array<double, 1> pc, vc, ac;
    traj.at_time(DT, pc, vc, ac);
    check(std::isfinite(ac[0]), "at_time(dt) returns finite acceleration (chaining source)");
    std::printf("    chained state @t=dt:  p=%.5f v=%.5f a=%.5f\n", pc[0], vc[0], ac[0]);
  }
  std::printf("\n");

  // -------------------------------------------------------------------------
  // SPIKE-2: the a_max/jerk NON-CONVERGENCE condition, validated against ruckig.
  // Sweep |Δv| = |vf - v0| with a0=af=0 and pf on the average-velocity line, find
  // the empirical |Δv| where duration first exceeds dt, and compare to the
  // predicted capacity dvCapacity(A,J,dt). This is the predictive feasibility
  // gate the follower will run BEFORE calling ruckig.
  // -------------------------------------------------------------------------
  std::printf("SPIKE-2: convergence capacity  |vf-v0| vs T_opt crossing dt\n");
  {
    const double v0 = 0.0, a0 = 0.0, af = 0.0;
    const double pred = dvCapacity(A, J, DT);
    double emp_cross = -1.0;
    double prev_dv = 0.0, prev_dur = DT;

    Ruckig<1> otg;
    for (int i = 1; i <= 4000; ++i) {
      const double dv = i * 1e-4;            // sweep Δv in 0.1 mm/s steps
      const double vf = v0 + dv;
      InputParameter<1> in;
      in.current_position     = {0.0};
      in.current_velocity     = {v0};
      in.current_acceleration = {a0};
      // pf on the mid-velocity line so position never dominates the timing:
      in.target_position      = {0.5 * (v0 + vf) * DT};
      in.target_velocity      = {vf};
      in.target_acceleration  = {af};
      in.max_velocity         = {V};
      in.max_acceleration     = {A};
      in.max_jerk             = {J};
      in.minimum_duration     = DT;

      Trajectory<1> traj;
      if (otg.calculate(in, traj) != Result::Working) continue;
      if (traj.get_duration() > DT + 1e-6 && emp_cross < 0.0) {
        // linear-interpolate the crossing between prev and current sample
        const double frac = (DT - prev_dur) / (traj.get_duration() - prev_dur + 1e-12);
        emp_cross = prev_dv + frac * (dv - prev_dv);
        break;
      }
      prev_dv = dv;
      prev_dur = traj.get_duration();
    }

    std::printf("    predicted capacity (%s): |Δv|_cap = %.5f m/s\n",
                regime(A, J, DT), pred);
    std::printf("    empirical ruckig crossing: |Δv|      = %.5f m/s\n", emp_cross);
    const double rel = emp_cross > 0 ? std::fabs(emp_cross - pred) / pred : 1.0;
    std::printf("    relative error = %.1f%%\n", rel * 100.0);
    check(emp_cross > 0, "found an empirical dt-crossing");
    check(rel < 0.15, "predicted capacity matches ruckig within 15%");

    // Also confirm: BELOW capacity -> duration == dt (converges); ABOVE -> slips.
    auto durAt = [&](double dv) {
      InputParameter<1> in;
      in.current_position = {0.0}; in.current_velocity = {v0}; in.current_acceleration = {a0};
      in.target_position = {0.5 * (v0 + (v0 + dv)) * DT};
      in.target_velocity = {v0 + dv}; in.target_acceleration = {af};
      in.max_velocity = {V}; in.max_acceleration = {A}; in.max_jerk = {J};
      in.minimum_duration = DT;
      Trajectory<1> traj; otg.calculate(in, traj); return traj.get_duration();
    };
    check(std::fabs(durAt(0.5 * pred) - DT) < 1e-6, "|Δv| = 0.5*cap  -> converges in dt");
    check(durAt(2.0 * pred) > DT + 1e-6,            "|Δv| = 2.0*cap  -> T slips (no converge)");
    std::printf("    dur@0.5cap=%.5f  dur@2cap=%.5f\n", durAt(0.5 * pred), durAt(2.0 * pred));
  }
  std::printf("\n");

  // -------------------------------------------------------------------------
  // SPIKE-2b: does a non-zero, jerk-limited af CHANGE the capacity? (the
  // follower feeds af = central 2nd difference, not 0). Sanity that feeding a
  // consistent af does not collapse feasibility.
  // -------------------------------------------------------------------------
  std::printf("SPIKE-2b: on-reference consistent sample converges in dt\n");
  {
    Ruckig<1> otg;
    // A smooth reference; move FROM the state at index k-1 (t=0) TO the state at
    // index k (t=dt). Both boundaries are SELF-CONSISTENT central-difference
    // samples of the SAME reference (the design's invariant). Use a cubic so af
    // is genuinely non-zero and non-trivial at both ends.
    const double dt = DT;
    auto p = [](double t){ return 0.20 * t + 0.5 * 0.8 * t * t + (1.0 / 6.0) * 5.0 * t * t * t; };
    auto v = [](double t){ return 0.20 + 0.8 * t + 0.5 * 5.0 * t * t; };
    auto a = [](double t){ return 0.8 + 5.0 * t; };  // jerk = 5 (within J=30)
    // central differences at each grid point (what the follower actually feeds):
    auto vc = [&](double t){ return (p(t + dt) - p(t - dt)) / (2 * dt); };
    auto ac = [&](double t){ return (p(t + dt) - 2 * p(t) + p(t - dt)) / (dt * dt); };
    InputParameter<1> in;
    in.current_position = {p(0.0)};  in.current_velocity = {vc(0.0)};  in.current_acceleration = {ac(0.0)};
    in.target_position  = {p(dt)};   in.target_velocity  = {vc(dt)};   in.target_acceleration  = {ac(dt)};
    in.max_velocity = {V}; in.max_acceleration = {A}; in.max_jerk = {J};
    in.minimum_duration = dt;
    Trajectory<1> traj;
    Result res = otg.calculate(in, traj);
    std::printf("    v0=%.4f a0=%.4f -> vf=%.4f af=%.4f  Result=%d duration=%.5f (exact v,a: v0=%.4f a0=%.4f)\n",
                vc(0.0), ac(0.0), vc(dt), ac(dt), static_cast<int>(res), traj.get_duration(), v(0.0), a(0.0));
    check(res == Result::Working, "consistent-sample BVP solves");
    check(traj.get_duration() <= dt + 1e-6, "on-reference consistent sample converges in dt (T_opt≈dt)");
  }
  std::printf("\n");

  // -------------------------------------------------------------------------
  // SPIKE-4: rough RT timing of calculate() for 6 coupled DOF (the in-loop cost).
  // -------------------------------------------------------------------------
  std::printf("SPIKE-4: calculate() timing, Ruckig<6> time-synced\n");
  {
    Ruckig<6> otg;
    std::vector<double> us;
    us.reserve(20000);
    for (int i = 0; i < 20000; ++i) {
      InputParameter<6> in;
      const double s = 1e-3 * (i % 97);  // vary the problem so it isn't cached-trivial
      for (int d = 0; d < 6; ++d) {
        in.current_position[d]     = 0.0;
        in.current_velocity[d]     = 0.05 + 0.001 * d;
        in.current_acceleration[d] = 0.0;
        in.target_position[d]      = 0.01 + s + 0.002 * d;
        in.target_velocity[d]      = 0.06 + 0.001 * d;
        in.target_acceleration[d]  = 0.10;
        in.max_velocity[d]         = V;
        in.max_acceleration[d]     = A;
        in.max_jerk[d]             = J;
      }
      in.minimum_duration = DT;
      Trajectory<6> traj;
      auto t0 = std::chrono::steady_clock::now();
      otg.calculate(in, traj);
      auto t1 = std::chrono::steady_clock::now();
      us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    std::sort(us.begin(), us.end());
    const double p50 = us[us.size() / 2];
    const double p99 = us[us.size() * 99 / 100];
    const double pmax = us.back();
    std::printf("    calculate<6>: p50=%.2f us  p99=%.2f us  max=%.2f us\n", p50, p99, pmax);
    check(p99 < 400.0, "p99 update < 400us (in-loop RT budget gate)");
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              failures == 0 ? "ALL SPIKES PASSED" : "SPIKE FAILURES",
              failures, failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}
