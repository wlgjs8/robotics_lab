#pragma once

// SAFETY PLAN GATE law (safety.plan_gate) — one pure tick of the per-arm
// plan-clock rate gate that applySafety() maintains and applyChunkFollowerStage()
// consumes on the following tick.
//
// WHAT IT IS FOR. The geometric safety layers slow the ARM but nothing slowed the
// PLAN: the chunk follower kept integrating at up to 0.45 m/s while a barrier held
// the arm, and the accumulated lead discharged either as a release lunge (measured
// 10,050 deg/s^2 command accel at a clamp exit) or as re-anchor stop-go. The gate
// paces the plan clock by how much of the requested step actually survived, so the
// reference cannot outrun the arm.
//
// WHAT IT IS NOT. It is not a safety mechanism. Whatever removed the motion — the
// obstruction projection, a clamp — already did so in the same tick, on the command
// that goes to the wire. The gate only stops the PLAN winding up behind that, which
// is why it may take a few ms to close (see attack_alpha) instead of stepping.
//
// TWO CORRECTIONS, measured on servo_log_20260827_213651.csv (169 s rollout):
//
//  1. INPUT. The gate first shipped reading `desired` vs final, i.e. everything the
//     safety filter did — and the CLAMPS dominated it. 77.6% of the left arm's gated
//     ticks co-occurred with the J3 approach barrier and only 5.5% with the
//     self-collision projection; the right arm split 30% barrier / 30% projection /
//     28.5% acceleration clamp. But the barrier parking J3 at its 149.90 deg standoff
//     is the barrier WORKING, and while it holds, realized < requested on that joint
//     is permanently true — so a layer designed to stop ONE joint smoothly was
//     stopping the plan clock of ALL SIX abruptly. The acceleration clamp is not an
//     obstruction either: it passes the motion through on the following ticks, so the
//     plan has nothing to wait for. Cost of the mistake: 11.07 s of plan time
//     discarded on the left arm (6.5% of the run, ~53 chunks of lag; 9.42 s of it on
//     the barrier, 0.72 s on the projection), surfacing as a ~4.8 Hz command ripple —
//     exactly the 208 ms chunk arrival rate, coherent across all six joints, at
//     2.1-2.8x the baseline 5-25 Hz tremble after matching for speed.
//     The caller now passes the POST-CLAMP, PRE-PROJECTION target as `requested_q`,
//     so only an obstruction (self-collision, floor, ROI, reach) can close the gate.
//     A genuine HOLD (stale verdict, IK refusal, fault) freezes the plan through
//     markCartesianSolveBlocked instead, not through this gate.
//
//  2. RATIO. It was max_j|requested-prev| over max_j|final-prev| — two INDEPENDENT
//     per-joint maxima, so the argmax could be different joints and the "fraction
//     that survived" was really J1's requested step over J4's realized one. The
//     ratio is now the realized step projected onto the requested direction,
//     (r . q)/|q|^2, which is the question the plan clock actually asks — how much
//     progress along the intended step survived — and which credits a preserved
//     tangential slide instead of reading it as a full block.
//
// Deterministic and side-effect free — unit-tested in tests/test_plan_gate.cpp.

#include <algorithm>
#include <cmath>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {
namespace control {

// Advance `gate` by one tick. `prev_q` is the last sent command, `requested_q` the
// target as it stands before the obstruction projection, `final_q` the target after
// it. Returns the new gate in [min_gate, 1].
inline double planGateStep(
    double gate,
    const JointArray& requested_q,
    const JointArray& final_q,
    const JointArray& prev_q,
    const SafetyPlanGateConfig& cfg
) {
    double requested_sq = 0.0;
    double along = 0.0;
    for (int j = 0; j < kDof; ++j) {
        const double req = requested_q[j] - prev_q[j];
        const double got = final_q[j] - prev_q[j];
        requested_sq += req * req;
        along += got * req;
    }
    // Deadband on the Euclidean step norm (the directional ratio's own scale):
    // below it the step is noise and only the release runs, so idle and hold ticks
    // cannot drive the gate.
    if (std::sqrt(requested_sq) > cfg.deadband_deg) {
        const double instant = std::clamp(along / requested_sq, cfg.min_gate, 1.0);
        if (instant < gate) {
            // Attack and release never run on the same tick.
            return gate + cfg.attack_alpha * (instant - gate);
        }
    }
    return gate + cfg.release_alpha * (1.0 - gate);
}


// IK-THROTTLE PLAN PACING. One first-order gate carrying how much of the requested
// joint step the IK branch-jump rate limiter actually let through, engaged only after
// the throttle has PERSISTED. `run` is the count of consecutive throttled ticks
// (caller-maintained, reset to 0 on any un-throttled tick).
//
// The persistence condition is the whole design. Feeding transient clamps (the
// joint-limit barrier, the acceleration clamp) into the plan gate was measured and
// REVERTED -- the barrier's job is to hold ONE joint at its standoff, so realized <
// requested is permanently true there while it works correctly, and letting that stop
// the plan clock of all six joints produced a 4.8 Hz ripple at 2.1-2.8x the baseline
// tremble. A sustained IK throttle is different in kind: measured 2026-08-28
// (servo_log_20260828_135443.csv) it held for 150 consecutive ticks with IK wanting
// 4.233 deg/tick against a 0.350 ceiling, and the plan advancing through it wound the
// follower divergence to 50.05 mm -- grazing the 50 mm re-anchor latch -- whose
// catch-up was that run's two worst seconds.
//
// Never returns below cfg.min_gate: the plan SLOWS to the rate the arm is achieving,
// it never stops, so this cannot produce the stop-go the freeze alternative would.
inline double ikThrottlePlanGateStep(
    double gate,
    int run,
    bool throttled,
    double achieved_ratio,
    const SafetyPlanGateConfig& cfg
) {
    const bool sustained =
        cfg.ik_throttle_min_ticks > 0 && throttled && run >= cfg.ik_throttle_min_ticks;
    const double target =
        sustained ? std::clamp(achieved_ratio, cfg.min_gate, 1.0) : 1.0;
    const double alpha = target < gate ? cfg.attack_alpha : cfg.release_alpha;
    return gate + alpha * (target - gate);
}

// RAMPED ENGAGEMENT of the pinned/throttled joint low-pass. Attacks in one tick and
// decays with `release_sec`, so no gate transition can step the transfer function. The
// instant release it replaces put a median 4,482 deg/s^2 into the command against 83
// elsewhere (2026-08-28, 54x, peak 184,224 = 122x ddq_max), reversing a joint between
// two 2 ms samples. release_sec <= 0 reproduces that instant release.
inline double lowpassEngagementStep(double engage, bool gate_on,
                                    double release_sec, double dt_sec) {
    if (gate_on) return 1.0;
    if (!(release_sec > 0.0) || !(dt_sec > 0.0)) return 0.0;
    return engage * std::exp(-dt_sec / release_sec);
}


// DIVERGENCE LEASH on the plan clock (2026-09-06, docs/plans/
// plan_clock_time_stretch_20260906.md). The plan-vs-sent divergence -- the quantity
// the soft chunk_follower_divergence bound reads -- drives a linear ramp: 1.0 up to
// `start`, falling to `min_gate` at `full` (the soft bound), per axis, the more
// restrictive of position and orientation wins. So the plan clock cannot wind the
// divergence past the soft bound unless the chain stops delivering entirely, and it
// never stops (min_gate > 0): the chain's lag is absorbed as time, not latched.
// Stateless: the divergence is an integrated quantity and already smooth.
struct PlanLeashParams {
    double start_m = 0.0;
    double start_rad = 0.0;
    double full_m = 0.0;
    double full_rad = 0.0;
    double min_gate = 0.0;
};

inline double planLeashGate(double pos_err_m, double ang_err_rad, const PlanLeashParams& p) {
    const auto ramp = [](double err, double start, double full) {
        if (!(full > start)) return err > start ? 0.0 : 1.0;   // degenerate: step
        return std::clamp(1.0 - (err - start) / (full - start), 0.0, 1.0);
    };
    const double g = std::min(ramp(pos_err_m, p.start_m, p.full_m),
                              ramp(ang_err_rad, p.start_rad, p.full_rad));
    return std::max(std::clamp(p.min_gate, 0.0, 1.0), g);
}

}  // namespace control
}  // namespace rb_servo
