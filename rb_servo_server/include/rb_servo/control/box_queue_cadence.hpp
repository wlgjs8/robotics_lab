#pragma once

#include <cstdint>
#include <optional>

#include "rb_servo/core/types.hpp"

namespace rb_servo {

// Holds the rbpodo control box's command queue at a bounded depth by trimming
// the servo loop's send period.
//
// THE DEFECT THIS FIXES
// ---------------------
// The servo loop streams servo_j at exactly servo.rate_hz off the host clock,
// while the box consumes at its own slightly slower rate. Nothing closes that
// loop, so the difference accumulates in the box's command queue and the
// plan-to-robot delay grows for as long as the stack stays up.
//
// Measured on hardware 2026-08-18 (logs/servo_log_20260818_113008.csv,
// tools/analyze_box_queue_lag.py), with the RBACK observer but no controller:
//
//     fill:  14.2 -> 45.5 ticks over 48 s   = +0.65 ticks/s
//     lag:     64 -> 114 ms                 = +1.30 ms/s
//                          0.65 x 2 ms/tick = 1.30 ms/s
//
// The fill growth rate IS the lag drift rate, to two significant figures, on
// both arms. Absolute values close too: lag ~= fill x 2 ms + ~25 ms, where the
// constant is servo_t2_sec (21 ms) plus the arm's mechanical response.
//
// It matters because preview_max_actual_lead_rad (4 deg) and
// preview_max_actual_lead_m (35 mm) in stack_real.yaml are speed x delay
// budgets. With the delay drifting, the same rollout trips or does not trip
// depending only on stack uptime -- which is what latched a ChunkFollowerFault
// on the right arm at t=69 s in the 10:45 run.
//
// SIGN CONVENTION (get this wrong and the loop runs away)
// -------------------------------------------------------
//     trim > 0  ->  LONGER period  ->  send slower  ->  queue DRAINS
//     trim < 0  ->  SHORTER period ->  send faster  ->  queue GROWS
//
// TWO LEVERS: A SHARED RATE TRIM, AND A PER-ARM LEVEL TRIM
// --------------------------------------------------------
// rb_servo_server runs ONE loop for both arms (controller-manager gets a per-arm
// RT thread with its own FLL), so a single period trim must serve two boxes. The
// trim is a RATE lever and it is correctly shared: the 2026-08-18 runs measured
// both boxes consuming at the SAME rate (+0.65 ticks/s each), so one integrator
// learning one clock offset is the right model.
//
// What a shared rate lever CANNOT fix is a LEVEL difference. It holds whichever
// arm it is pointed at; the other sits at whatever offset it warmed up with. We
// point it at the SHALLOWER queue because underrun is the dangerous failure -- an
// empty queue means the box has no next q_ref and holds the last one, which is a
// stall the arm feels as a jerk -- while overshooting the other arm only costs it
// latency.
//
// That residual is not small and it is not stable. Measured:
//
//     18:38 run   left 5.0  right 14.0-15.0   -> 10 ticks = 20 ms
//     18:52 run   right 5.0 left  8.0-9.0     ->  4 ticks =  8 ms
//
// and WHICH arm gets held flips between sessions (whichever warmed up shallower).
// So the deep arm carries up to 20 ms of pure latency, and every A/B comparison
// between arms is confounded by an offset that changes run to run.
//
// BoxQueueLevelConfig below closes that gap with a per-arm lever: skipping one
// arm's send for one tick drops that arm's queue by exactly one tick and touches
// nothing else. Read its comment before changing any of it -- the safety argument
// is not obvious.
//
// WHY ONE MODEST CLAMP AND NO DRAIN PHASE
// ----------------------------------------
// The trim widens the gap between consecutive q_ref writes. The box's servo_j
// interpolates against servo_t1_sec (2 ms), so feeding it much slower than that
// risks the box running dry BETWEEN commands -- micro-stalls that read as jerk.
// controller-manager can afford an aggressive drain (up to +2000 us) because its
// Drain phase runs at task entry, before the task commands real motion; we may be
// asked to converge while a policy is already driving. So we keep one modest clamp
// at all times. (An earlier version of this comment said controller-manager drains
// "during Warmup, before the task streams". That is wrong: QSYNC's Drain is a
// phase INSIDE OnTask, entered after Warmup. The conclusion stands, the stated
// reason did not.)
//
// At the default +-120 us on a 2000 us period the send rate moves by ~6%, which
// drains ~28 ticks/s -- the 40-tick startup backlog clears in about 1.4 s. That
// is fast enough that a separate drain phase buys nothing and costs a second
// code path.
//
// The clamp is also bounded by servo.filter_dt_max_ratio (1.5): the follower is
// advanced by the CLAMPED actual period, so a trim that pushes the period past
// 1.5x nominal would make the follower under-advance and distort trajectory
// timing. Anything at or below +1000 us stays inside that clamp.
// Per-arm level trim: hold BOTH arms at target_fill, not just the shallower one.
//
// THE LEVER
// ---------
// Skipping one arm's servo_j send for one tick drops that arm's queue by exactly
// one tick. The box keeps popping from its queue at its own rate; we simply do
// not append. The other arm is untouched. servo_dispatcher enqueues per arm
// already, so this is a guarded skip, not an architecture change.
//
// THE COST, WHICH DETERMINES THE WHOLE DESIGN
// -------------------------------------------
// Queue latency IS trajectory waypoints in flight. An arm at fill 14 is executing
// a trajectory 28 ms old. To make it execute a 10 ms old one, it has to skip 18 ms
// of trajectory forward. There is no way around this:
//
//   Holding the arm's trajectory phase back by one tick per skip -- so that no
//   waypoint is dropped -- cancels the gain exactly. Queue latency falls 2 ms and
//   trajectory phase falls 2 ms behind, and the pose the arm reaches at any wall
//   clock time is bit-identical. Net zero. Do not "fix" the discontinuity that way.
//
// So a skip MUST drop a waypoint, and the box then covers two trajectory steps in
// one of its ticks: a one-tick doubling of joint velocity. The cost is therefore
// proportional to how fast the arm is moving AT THAT INSTANT, which is why the
// speed gate below is the load-bearing part of this design and not a refinement.
//
// At skip_max_joint_speed_deg_s = 1.0 the dropped step is 0.002 deg, against a
// measured median per-tick motion of 0.0106 deg and a box encoder grid of 2e-4
// deg. This is the same trade controller-manager makes: its Drain phase (+2000 us
// for a second = 250 commands instead of 500) is 250 skips spread evenly, taken at
// task entry while the arm is still parked. Distributed skipping and quantized
// skipping are the same operation.
//
// WHEN THE WORK ACTUALLY HAPPENS
// ------------------------------
// Mostly at stream entry. The startup backlog measured 41-51 ticks with the arm
// stationary, so the bulk of the correction is free. In the 18:38 run the right
// arm sat at 14 ticks from t=6 s to the end -- it was never equalized at entry,
// and paid 20 ms for 88 s. After entry the residual differential drift measured
// +1 tick per 84-154 s, i.e. roughly one skip every two minutes. Opportunity is
// not scarce: 28-30% of ticks are below 1.0 deg/s and the longest measured gap
// between such ticks was 9.8 s (left) / 16.9 s (right).
//
// WHY THIS CANNOT CAUSE AN UNDERRUN
// ---------------------------------
// A skip is only ever issued to an arm whose queue is ABOVE target -- the arm with
// buffer to spare -- and never to the arm the shared trim is holding. The
// dangerous direction is unreachable by construction, not by tuning.
struct BoxQueueLevelConfig {
    // Fail-closed: min()-only behaviour until a config opts in.
    bool enable = false;

    // Skip only while the LOW-PASSED depth exceeds target by more than this.
    //
    // The decision runs on the filtered depth, not the raw one, because the raw
    // integer genuinely swings +-1 around its level (measured 18:52 right arm:
    // p50 5, p99 6, min 4) as sends and box consumption drift in and out of
    // alignment. Skipping on those excursions would walk the arm down toward the
    // protect floor for no reason. The 0.1 s filter averages them out, so an arm
    // whose true level is target reads ~target and never trips this.
    //
    // 0.5 leaves the arm at target..target+1 (10-11 ms). Driving it to exactly
    // target would mean margin 0, which trades that last millisecond for a skip
    // on every upward excursion.
    double skip_margin_ticks = 0.5;

    // Hard floor on the RAW depth, independent of the filter: never skip an arm
    // at or below target no matter what the low-pass believes.
    int skip_floor_offset_ticks = 1;

    // The speed gate. See the cost argument above -- this is what makes a dropped
    // waypoint free. Max |dq| across the arm's joints, in deg/s.
    double skip_max_joint_speed_deg_s = 1.0;

    // Minimum ticks between skips on the same arm. Must be >= 2: a skip produces
    // no RBACK for that arm, so the next decision has to wait for the observation
    // that reflects it, or the loop is running open.
    int skip_min_interval_ticks = 2;
};

struct BoxQueueCadenceConfig {
    // Fail-closed: the loop runs at the plain fixed tick until a config opts in.
    bool enable = false;

    // Queue depth setpoint, in ticks (1 tick = one control period = 2 ms at
    // 500 Hz). controller-manager holds 5 on this same hardware.
    int target_fill = 5;

    // At or below this depth the queue is about to run dry; the PI is overridden
    // and the loop wakes early hard until it recovers.
    int protect_fill = 1;
    double protect_trim_us = -80.0;

    // Fill low-pass, applied only on a fresh observation (~0.1 s at 500 Hz).
    double lpf_alpha = 0.02;

    // Asymmetric proportional gain [us per tick of error]: drain gently when the
    // queue is deep (latency is merely bad), refill briskly when it is shallow
    // (underrun is a stall).
    double kp_above_us = 6.0;
    double kp_below_us = 18.0;

    // Integral [us per tick of error per cycle]. This is the term that learns the
    // constant host-vs-box clock offset; controller-manager converged to
    // +2.562 us/cycle on these boxes, which is the +1.28 ms/s we measured.
    double ki_us = 0.006;
    double integral_clamp_us = 25.0;

    // Hard bound on the trim. See the rationale above before widening it.
    double trim_clamp_us = 120.0;

    // Observe before acting: no trim at all until the queue has been reported for
    // this long, so a cold or misreporting box never gets a guessed correction.
    double warmup_sec = 0.4;

    // Ticks without a fresh observation before the trim is released back to zero.
    // A box that stops answering must not leave a stale correction applied.
    int stale_observation_ticks = 25;

    // Per-arm level trim. Off by default; see BoxQueueLevelConfig above.
    BoxQueueLevelConfig level;
};

// One tick of input. Joint speeds are the PREVIOUS tick's commanded |dq|/dt per
// arm: the decision made here shapes the NEXT tick's send, and at 500 Hz the
// speed one tick either side of that is the same number for gating purposes.
struct BoxQueueCadenceInput {
    std::optional<int> left_fill;
    std::optional<int> right_fill;
    double left_joint_speed_deg_s = 0.0;
    double right_joint_speed_deg_s = 0.0;
    double dt_sec = 0.0;
};

class BoxQueueCadence {
public:
    enum class Phase { Disabled, Warmup, Tracking, Stale };

    explicit BoxQueueCadence(BoxQueueCadenceConfig config);

    // Feed this tick's observed queue depth per arm (nullopt = not observed this
    // tick) and get the period trim to apply to the NEXT tick's deadline.
    // Also latches the per-arm skip decision for the NEXT tick; read it with
    // skipNext(). `dt_sec` is the tick period actually elapsed.
    std::int64_t update(const BoxQueueCadenceInput& input);

    // Depth-only overload: no speed information, so the per-arm level trim never
    // fires. Kept for tests and for callers that do not track commanded speed.
    std::int64_t update(
        std::optional<int> left_fill,
        std::optional<int> right_fill,
        double dt_sec
    );

    // Drop all learned state. Call when the command stream stops, so a resumed
    // stream re-observes instead of acting on a stale queue picture.
    void reset();

    Phase phase() const { return phase_; }
    double trimUs() const { return trim_us_; }
    double fillLpf() const { return fill_lpf_; }
    double integralUs() const { return integral_us_; }
    // Depth the controller actually acted on this tick (the shallower arm), or
    // -1 when nothing was observed.
    int controlledFill() const { return controlled_fill_; }

    // Whether the NEXT tick should withhold this arm's servo_j send to drop its
    // queue by one. At most one arm is ever true on a given tick.
    bool skipNext(ArmId arm) const {
        return arm == ArmId::Left ? skip_next_left_ : skip_next_right_;
    }
    // Cumulative skips issued, per arm. Rising fast means the speed gate or the
    // margin is wrong -- expect ~40 at entry, then ~1 every two minutes.
    std::uint64_t skipCount(ArmId arm) const {
        return arm == ArmId::Left ? skip_count_left_ : skip_count_right_;
    }
    // Per-arm low-passed depth the level trim decides on (-1 before it is seeded).
    double armFillLpf(ArmId arm) const {
        const bool seeded = arm == ArmId::Left ? arm_lpf_seeded_left_ : arm_lpf_seeded_right_;
        if (!seeded) return -1.0;
        return arm == ArmId::Left ? arm_fill_lpf_left_ : arm_fill_lpf_right_;
    }

private:
    BoxQueueCadenceConfig config_;
    Phase phase_ = Phase::Disabled;
    double elapsed_sec_ = 0.0;
    double fill_lpf_ = 0.0;
    bool lpf_seeded_ = false;
    double integral_us_ = 0.0;
    double trim_us_ = 0.0;
    int controlled_fill_ = -1;
    int ticks_since_observation_ = 0;

    // Per-arm level trim state.
    void updateArmLevel(const BoxQueueCadenceInput& input);
    double arm_fill_lpf_left_ = 0.0;
    double arm_fill_lpf_right_ = 0.0;
    bool arm_lpf_seeded_left_ = false;
    bool arm_lpf_seeded_right_ = false;
    bool skip_next_left_ = false;
    bool skip_next_right_ = false;
    int ticks_since_skip_left_ = 0;
    int ticks_since_skip_right_ = 0;
    std::uint64_t skip_count_left_ = 0;
    std::uint64_t skip_count_right_ = 0;
};

const char* toString(BoxQueueCadence::Phase phase);

}  // namespace rb_servo
