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

}  // namespace control
}  // namespace rb_servo
