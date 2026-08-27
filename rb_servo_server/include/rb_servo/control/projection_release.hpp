#pragma once

// PROJECTION RELEASE SLEW (safety.projection_release_slew_deg_s2).
//
// The geometric projection re-solves from scratch every tick, so the correction it
// applies is free to jump. Engaging fast is the whole point of a safety damper;
// DIS-engaging fast is not, and the measurements say the disengage is where the
// vibration comes from.
//
// MEASURED, servo_log_20260828_004539.csv, the left-arm <-> stand event at
// t=223.37-224.01 s (clearance pinned at 24.86-25.01 mm against its own 25 mm
// floor, 6-10 constraint rows engaged, follower active throughout):
//
//   - The barrier did its job. It stopped the approach AT the floor and held it
//     there for 0.58 s while the arm kept moving tangentially (J1 held ~-40 deg/s,
//     J5 swept +1.7 -> +31 deg/s). Nothing about that needs changing.
//   - But the correction shrank in one-tick steps: |dcorr|/tick was p50 0.197 and
//     p95 8.42 deg/s, a completely split distribution. In the trace the correction
//     reads 12.28 -> 3.93 -> 12.36 and 12.36 -> 3.94 -> 12.09: single-tick dropouts
//     to a third and straight back, each injecting ~4,280 deg/s^2. 40 such dropouts
//     on the left arm, 25 on the right.
//   - And the release was a step: 12.09 -> 0.00 in one tick, immediately followed by
//     J1 -53.8 -> -83.6 deg/s and 5,946 deg/s^2. Seven more step-releases on the
//     right arm (correction before release p50 5.57, max 15.63 deg/s).
//   - Net: 8-60 Hz command vibration of 2.88 in that window against a run median of
//     0.248 -- 12x.
//
// A standoff (braking to zero further out) does NOT fix this: it moves WHERE the
// barrier holds, and the vibration is in HOW the correction changes. What fixes it
// is bounding the release, which is what this does.
//
// The law is asymmetric on purpose:
//   - growing the correction (removing MORE velocity) is instantaneous, so the
//     safety response is never delayed;
//   - shrinking it is rate-limited, so a dropout or a release becomes a ramp.
// Holding a slightly stale correction is always the conservative side: it keeps the
// arm not-closing marginally longer, it is bounded by the previous tick's value, and
// it decays. Retreat is unaffected in steady state -- the residual is gone within a
// few tens of ms.
//
// Deterministic and side-effect free apart from the caller-owned `prev` state --
// unit-tested in tests/test_projection_release.cpp.

#include <cmath>

#include "rb_servo/core/types.hpp"

namespace rb_servo {
namespace control {

// One tick. `requested_q` is the target BEFORE the projection, `final_q` the target
// after it (modified in place), `prev_correction_deg` the caller-persisted per-joint
// correction from last tick, in degrees of target displacement.
//
// MUST be called every tick, including ticks where no constraint row ran: the step
// release happens exactly when the rows disengage, and skipping those ticks would
// drop the correction to zero instantly -- the defect this exists to remove.
inline void projectionReleaseStep(
    const JointArray& requested_q,
    JointArray& final_q,
    JointArray& prev_correction_deg,
    double dt_sec,
    double slew_deg_s2
) {
    if (!(slew_deg_s2 > 0.0) || !(dt_sec > 0.0)) {
        // Disabled: record what the projection did so enabling it mid-run cannot
        // release a correction this law never saw.
        for (int j = 0; j < kDof; ++j) {
            prev_correction_deg[j] = requested_q[j] - final_q[j];
        }
        return;
    }
    // slew is the rate of change of the correction VELOCITY (deg/s^2); the
    // correction itself is a per-tick displacement, hence dt twice.
    const double max_release_deg = slew_deg_s2 * dt_sec * dt_sec;
    for (int j = 0; j < kDof; ++j) {
        double c = requested_q[j] - final_q[j];
        const double p = prev_correction_deg[j];
        if (p != 0.0) {
            const double sign = p > 0.0 ? 1.0 : -1.0;
            const double along = c * sign;              // < 0 when the sign flipped
            const double floor_mag = std::abs(p) - max_release_deg;
            if (floor_mag > 0.0 && along < floor_mag) {
                c = sign * floor_mag;
            }
        }
        final_q[j] = requested_q[j] - c;
        prev_correction_deg[j] = c;
    }
}

}  // namespace control
}  // namespace rb_servo
