#include "rb_servo/control/queue_sync_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

#define RB_CHECK(cond)                                                       \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::cerr << "CHECK failed at " << __FILE__ << ":" << __LINE__  \
                      << ": " #cond << "\n";                                 \
            return false;                                                    \
        }                                                                    \
    } while (false)

constexpr double kNominalPeriodUs = 2000.0;   // 500 Hz

rb_servo::QueueSyncConfig testConfig() {
    rb_servo::QueueSyncConfig cfg;
    cfg.enable = true;
    // Shorten the observation windows so a test does not have to simulate
    // seconds of warmup; the control law itself is unchanged.
    cfg.warmup_min_sec = 0.02;
    cfg.warmup_max_sec = 0.05;
    return cfg;
}

// Simulated control box: a FIFO fed by our sends and drained on the box's own
// clock. This is the measured plant -- a pure integrator with a clock mismatch.
class BoxQueue {
public:
    BoxQueue(double drain_hz, double initial_fill)
        : drain_hz_(drain_hz), fill_(initial_fill) {}

    // Advance by one send period of `period_us`, then return the occupancy the
    // box would report in its RBACK for that command.
    int step(double period_us) {
        fill_ += 1.0;                                   // our command enters the queue
        fill_ -= drain_hz_ * (period_us * 1e-6);        // the box consumes on its clock
        if (fill_ < 0.0) fill_ = 0.0;
        return static_cast<int>(fill_);
    }

    double fill() const { return fill_; }

private:
    double drain_hz_;
    double fill_;
};

struct RunResult {
    double final_fill = 0.0;
    double final_trim_us = 0.0;
    std::string phase;
    rb_servo::QueueSyncDecision last;
};

RunResult run(rb_servo::QueueSyncController& ctrl, BoxQueue& box, int ticks,
              bool streaming = true) {
    RunResult result;
    uint64_t now_ns = 0;
    uint64_t seq = 0;
    double period_us = kNominalPeriodUs;
    for (int i = 0; i < ticks; ++i) {
        const int fill = box.step(period_us);
        ++seq;
        rb_servo::QueueSyncController::Observation obs;
        obs.streaming = streaming;
        obs.fill_valid = true;
        obs.fill = fill;
        obs.rback_sequence = seq;
        obs.now_ns = now_ns;
        const rb_servo::QueueSyncDecision d = ctrl.step(obs);
        period_us = kNominalPeriodUs + d.period_trim_us;
        now_ns += static_cast<uint64_t>(period_us * 1000.0);
        result.last = d;
    }
    result.final_fill = box.fill();
    result.final_trim_us = result.last.period_trim_us;
    result.phase = result.last.phase;
    return result;
}

// REGRESSION (2026-08-26). The law is timed off `obs.now_ns`, and `now_ns` only
// advances when the caller steps. A caller that stepped only on SEND ticks and held
// otherwise stopped the clock the law reads: Warmup never reached its timeout, the
// hold never lifted, and the arm sat there. Measured on hardware as 13 RBACKs parsed
// against 26691 ticks, with the box's own joint reference spanning 0.00 deg.
//
// Pinned here in the form the plant actually showed: a box that reports fill 0 no
// matter what we send (fw v8.7.3 ignored the stream for ~254 ms after activation).
// That is precisely the input the `developed` test can never fire on, so ONLY the
// timeout can end Warmup -- and the timeout is only reachable if every tick is
// stepped. The stream must also never be throttled to a stop while this resolves.
bool testWarmupBackoffKeepsProbingAndNeverStopsTheStream() {
    rb_servo::QueueSyncController ctrl(testConfig());
    uint64_t now_ns = 0;
    uint64_t seq = 0;
    double period_us = kNominalPeriodUs;
    double worst_period_us = 0.0;
    std::string phase = "idle";

    // 0.2 s at 500 Hz -- four times warmup_max_sec (0.05 s), so a law that needs the
    // timeout has had every chance to reach it.
    for (int i = 0; i < 100; ++i) {
        rb_servo::QueueSyncController::Observation obs;
        obs.streaming = true;
        obs.fill_valid = true;
        obs.fill = 0;                    // the silent box: consumes nothing, reports 0
        obs.rback_sequence = ++seq;
        obs.now_ns = now_ns;
        const rb_servo::QueueSyncDecision d = ctrl.step(obs);
        period_us = kNominalPeriodUs + d.period_trim_us;
        worst_period_us = std::max(worst_period_us, period_us);
        now_ns += static_cast<uint64_t>(period_us * 1000.0);
        phase = d.phase;
    }

    // Warmup ENDED. With the fill pinned at 0 the `developed` test cannot fire, so
    // this passing means the timeout was reached, which means every tick was stepped.
    RB_CHECK(phase != "warmup");
    RB_CHECK(phase != "idle");
    // A fill of 0 is at/below target, so there is nothing to drain.
    RB_CHECK(phase == "track");
    // THE STREAM WAS NEVER STOPPED. The removed warmup back-pressure used to throttle
    // sends here; nothing may reintroduce a trim that stalls the cadence.
    RB_CHECK(worst_period_us < 2.0 * kNominalPeriodUs);
    return true;
}

// The same startup, carried through to the steady state: the box ignores the stream
// for its first ~254 ms and then begins consuming normally. The backlog built during
// the silence is real and must DRAIN, not persist -- the pre-fix failure mode was a
// queue that took 2.3 s to clear, which is 2.3 s of dead time on every command.
bool testHeldTicksDoNotCountAsSends() {
    rb_servo::QueueSyncController ctrl(testConfig());
    uint64_t now_ns = 0;
    uint64_t seq = 0;
    double period_us = kNominalPeriodUs;
    double buried = 0.0;                 // what the silent box accumulated
    rb_servo::QueueSyncDecision d;

    for (int i = 0; i < 127; ++i) {      // ~254 ms of a box consuming nothing
        buried += 1.0;
        rb_servo::QueueSyncController::Observation obs;
        obs.streaming = true;
        obs.fill_valid = true;
        obs.fill = 0;                    // ...while still reporting an empty queue
        obs.rback_sequence = ++seq;
        obs.now_ns = now_ns;
        d = ctrl.step(obs);
        period_us = kNominalPeriodUs + d.period_trim_us;
        now_ns += static_cast<uint64_t>(period_us * 1000.0);
    }
    RB_CHECK(d.phase != "warmup");       // the silence did not latch the law

    // The box wakes up: the buried commands become the visible backlog and it now
    // drains on its own clock.
    BoxQueue box(499.34, buried);
    for (int i = 0; i < 20000; ++i) {
        const int fill = box.step(period_us);
        rb_servo::QueueSyncController::Observation obs;
        obs.streaming = true;
        obs.fill_valid = true;
        obs.fill = fill;
        obs.rback_sequence = ++seq;
        obs.now_ns = now_ns;
        d = ctrl.step(obs);
        period_us = kNominalPeriodUs + d.period_trim_us;
        now_ns += static_cast<uint64_t>(period_us * 1000.0);
    }
    RB_CHECK(d.phase == "track");
    RB_CHECK(d.locked);
    RB_CHECK(std::abs(box.fill() - 5.0) <= 1.0);
    return true;
}

// The headline case: the measured operating point. Box drains at 499.34 Hz
// against a 500 Hz nominal send, starting from the measured backlog.
bool testLocksMeasuredDriftToTarget() {
    rb_servo::QueueSyncController ctrl(testConfig());
    BoxQueue box(499.34, 24.0);
    const RunResult r = run(ctrl, box, 20000);   // 40 s
    RB_CHECK(r.phase == "track");
    RB_CHECK(std::abs(r.final_fill - 5.0) <= 1.0);
    RB_CHECK(r.last.locked);
    // The integral must have learned the drift: holding 5 requires sending at
    // the box's rate, i.e. a period ~ 1e6/499.34 us. Trim = that minus nominal.
    const double required_trim_us = 1e6 / 499.34 - kNominalPeriodUs;
    RB_CHECK(std::abs(r.final_trim_us - required_trim_us) < 0.5);
    return true;
}

bool testUnregulatedQueueGrowsWithoutBound() {
    // Control off: the same plant must diverge, or the test above proves nothing.
    rb_servo::QueueSyncConfig cfg = testConfig();
    cfg.enable = false;
    rb_servo::QueueSyncController ctrl(cfg);
    BoxQueue box(499.34, 24.0);
    const RunResult r = run(ctrl, box, 20000);
    RB_CHECK(r.final_trim_us == 0.0);
    RB_CHECK(r.final_fill > 40.0);              // grew, did not settle
    return true;
}

bool testDrainsLargeBacklogQuickly() {
    rb_servo::QueueSyncController ctrl(testConfig());
    BoxQueue box(499.34, 400.0);                // deep backlog
    const RunResult r = run(ctrl, box, 15000);  // 30 s
    RB_CHECK(r.phase == "track");
    RB_CHECK(std::abs(r.final_fill - 5.0) <= 1.5);
    return true;
}

bool testRefillsFromEmpty() {
    // Below target the controller must shorten the period, not lengthen it.
    rb_servo::QueueSyncController ctrl(testConfig());
    BoxQueue box(499.34, 0.0);
    const RunResult r = run(ctrl, box, 20000);
    RB_CHECK(r.final_fill > 2.0);
    RB_CHECK(r.last.underrun_events > 0);       // and it must have said so
    return true;
}

bool testProtectOverridesPiWhenNearEmpty() {
    rb_servo::QueueSyncConfig cfg = testConfig();
    rb_servo::QueueSyncController ctrl(cfg);
    BoxQueue box(499.34, 30.0);
    run(ctrl, box, 8000);                       // reach Track
    // Force a near-empty observation and check the protective trim, not the PI.
    rb_servo::QueueSyncController::Observation obs;
    obs.streaming = true;
    obs.fill_valid = true;
    obs.fill = cfg.protect_fill;
    obs.rback_sequence = 999999;
    obs.now_ns = 60'000'000'000ull;
    const rb_servo::QueueSyncDecision d = ctrl.step(obs);
    RB_CHECK(d.period_trim_us == cfg.protect_adj_us);
    RB_CHECK(d.period_trim_us < 0.0);           // shorter period = send sooner
    return true;
}

bool testDisabledIsAlwaysNeutral() {
    rb_servo::QueueSyncConfig cfg = testConfig();
    cfg.enable = false;
    rb_servo::QueueSyncController ctrl(cfg);
    BoxQueue box(499.34, 500.0);
    const RunResult r = run(ctrl, box, 3000);
    RB_CHECK(r.final_trim_us == 0.0);
    RB_CHECK(r.phase == "idle");
    return true;
}

bool testSilentStreamIsNeutralAndReArms() {
    rb_servo::QueueSyncController ctrl(testConfig());
    BoxQueue box(499.34, 24.0);
    run(ctrl, box, 8000);                       // reach Track
    // Stream goes silent: no trim, and the phase must fall back so the next
    // entry re-runs warmup/drain instead of acting on a pre-gap observation.
    rb_servo::QueueSyncController::Observation obs;
    obs.streaming = false;
    obs.fill_valid = true;
    obs.fill = 24;
    obs.rback_sequence = 500000;
    obs.now_ns = 60'000'000'000ull;
    const rb_servo::QueueSyncDecision d = ctrl.step(obs);
    RB_CHECK(d.period_trim_us == 0.0);
    RB_CHECK(d.phase == "idle");
    return true;
}

// A DIRECT-TEACHING GAP RE-RUNS WARMUP+DRAIN (2026-09-06). Freedrive stops the wire
// for as long as the operator hand-guides the arm, and the box queue empties while it
// does. Resuming on the pre-gap phase would mean a wound-up integral and a Track-phase
// trim aimed at a queue that no longer exists. The law must fall back to Idle for the
// silent stretch and climb warmup -> drain -> track again against the real queue.
//
// This is the whole of the "re-drain on re-entry" requirement: no extra call is
// needed, only that the caller keeps stepping with streaming=false while the wire is
// suppressed (ArmWorker::setSendSuppressed does exactly that).
bool testStreamGapReRunsWarmupAndDrainOnReEntry() {
    rb_servo::QueueSyncController ctrl(testConfig());
    BoxQueue box(499.34, 24.0);
    const RunResult tracked = run(ctrl, box, 8000);
    RB_CHECK(tracked.phase == "track");
    RB_CHECK(tracked.last.locked);

    // The teaching session: the wire is quiet and the box drains itself empty.
    BoxQueue drained(499.34, 0.0);
    const RunResult quiet = run(ctrl, drained, 500, /*streaming=*/false);
    RB_CHECK(quiet.phase == "idle");
    RB_CHECK(quiet.final_trim_us == 0.0);   // nothing to regulate, no actuation
    RB_CHECK(!quiet.last.locked);

    // Re-entry: the law must NOT come back in Track on the pre-gap state.
    BoxQueue refilled(499.34, 0.0);
    std::string first_phase;
    uint64_t now_ns = 0;
    uint64_t seq = 100000;
    double period_us = kNominalPeriodUs;
    for (int i = 0; i < 40; ++i) {
        const int fill = refilled.step(period_us);
        rb_servo::QueueSyncController::Observation obs;
        obs.streaming = true;
        obs.fill_valid = true;
        obs.fill = fill;
        obs.rback_sequence = ++seq;
        obs.now_ns = now_ns;
        const rb_servo::QueueSyncDecision d = ctrl.step(obs);
        if (i == 0) first_phase = d.phase;
        period_us = kNominalPeriodUs + d.period_trim_us;
        now_ns += static_cast<uint64_t>(period_us * 1000.0);
    }
    RB_CHECK(first_phase == "warmup");   // re-armed, not resumed

    // ...and it converges again from there.
    const RunResult again = run(ctrl, refilled, 8000);
    RB_CHECK(again.phase == "track");
    RB_CHECK(again.last.locked);
    return true;
}

bool testRebaseTriggersRedrain() {
    rb_servo::QueueSyncConfig cfg = testConfig();
    rb_servo::QueueSyncController ctrl(cfg);
    BoxQueue box(499.34, 24.0);
    run(ctrl, box, 8000);                       // reach Track
    rb_servo::QueueSyncController::Observation obs;
    obs.streaming = true;
    obs.fill_valid = true;
    obs.fill = cfg.target_fill + cfg.redrain_fill_margin + 10;
    obs.rback_sequence = 777777;
    obs.now_ns = 60'000'000'000ull;
    const rb_servo::QueueSyncDecision d = ctrl.step(obs);
    RB_CHECK(d.phase == "drain");
    RB_CHECK(d.redrain_events > 0);
    RB_CHECK(d.period_trim_us >= cfg.drain_adj_us);
    return true;
}

bool testStoppedConsumptionIsReportedNotTrimmedAt() {
    // A box that stopped consuming makes the fill rise far faster than any
    // period trim can correct. That must surface as an event.
    rb_servo::QueueSyncController ctrl(testConfig());
    uint64_t now_ns = 0;
    uint64_t seq = 0;
    int fill = 10;
    rb_servo::QueueSyncDecision d;
    for (int i = 0; i < 2000; ++i) {
        rb_servo::QueueSyncController::Observation obs;
        obs.streaming = true;
        obs.fill_valid = true;
        obs.fill = fill;
        obs.rback_sequence = ++seq;
        obs.now_ns = now_ns;
        d = ctrl.step(obs);
        fill += 1;                              // nothing drains
        now_ns += 2'000'000ull;
    }
    RB_CHECK(d.no_consumption_events > 0);
    return true;
}

bool testIntegralSurvivesResetButLevelStateDoesNot() {
    rb_servo::QueueSyncController ctrl(testConfig());
    BoxQueue box(499.34, 24.0);
    const RunResult before = run(ctrl, box, 20000);
    RB_CHECK(before.last.integral_us != 0.0);
    ctrl.reset();
    // The learned clock drift is a property of the hardware pair, so it must
    // survive; the queue level state must not.
    rb_servo::QueueSyncController::Observation obs;
    obs.streaming = true;
    obs.fill_valid = false;
    obs.fill = -1;
    obs.rback_sequence = 0;
    obs.now_ns = 0;
    const rb_servo::QueueSyncDecision d = ctrl.step(obs);
    RB_CHECK(std::abs(d.integral_us - before.last.integral_us) < 1e-9);
    RB_CHECK(d.last_fill == -1);
    RB_CHECK(d.phase == "warmup");
    return true;
}

bool testTrimIsBounded() {
    // Whatever the observation, the Track trim must stay inside the clamp: an
    // unbounded trim would desynchronise the loop from its own control period.
    rb_servo::QueueSyncConfig cfg = testConfig();
    rb_servo::QueueSyncController ctrl(cfg);
    BoxQueue box(499.34, 24.0);
    run(ctrl, box, 8000);
    uint64_t seq = 900000;
    uint64_t now = 60'000'000'000ull;
    for (int fill : {2, 3, 4, 5, 6, 7, 8, 12, 15, 18}) {
        rb_servo::QueueSyncController::Observation obs;
        obs.streaming = true;
        obs.fill_valid = true;
        obs.fill = fill;
        obs.rback_sequence = ++seq;
        obs.now_ns = now;
        now += 2'000'000ull;
        const rb_servo::QueueSyncDecision d = ctrl.step(obs);
        RB_CHECK(d.period_trim_us <= std::max(cfg.adj_clamp_us, cfg.drain_max_us));
        RB_CHECK(d.period_trim_us >= std::min(cfg.protect_adj_us, -cfg.adj_clamp_us));
        RB_CHECK(kNominalPeriodUs + d.period_trim_us > 0.0);
    }
    return true;
}

bool testBothArmsLockIndependently() {
    // The point of per-arm actuation: two boxes with different clocks must both
    // reach the target. A single shared trim could only satisfy one.
    rb_servo::QueueSyncController left(testConfig());
    rb_servo::QueueSyncController right(testConfig());
    BoxQueue left_box(499.335, 19.0);           // measured left arm
    BoxQueue right_box(499.330, 14.0);          // measured right arm
    const RunResult l = run(left, left_box, 20000);
    const RunResult r = run(right, right_box, 20000);
    RB_CHECK(std::abs(l.final_fill - 5.0) <= 1.0);
    RB_CHECK(std::abs(r.final_fill - 5.0) <= 1.0);
    RB_CHECK(l.last.locked && r.last.locked);
    // Their learned trims must differ, which is exactly what one shared
    // actuator could not have produced.
    RB_CHECK(std::abs(l.final_trim_us - r.final_trim_us) > 1e-3);
    return true;
}

// Step the controller directly, so a test can control FRESHNESS -- which the
// BoxQueue harness cannot, because it advances the sequence every tick.
class DirectFeed {
public:
    explicit DirectFeed(rb_servo::QueueSyncController& ctrl) : ctrl_(ctrl) {}

    // A FRESH observation: the sequence advances, so the number describes "now".
    rb_servo::QueueSyncDecision fresh(int fill) {
        ++seq_;
        return step(fill);
    }
    // A STALE cycle: the controller runs, but no new RBACK was parsed. This is the
    // condition `last_fill` persists through and the one the field case hit.
    rb_servo::QueueSyncDecision stale(int fill) { return step(fill); }

private:
    rb_servo::QueueSyncDecision step(int fill) {
        rb_servo::QueueSyncController::Observation obs;
        obs.streaming = true;
        obs.fill_valid = true;
        obs.fill = fill;
        obs.rback_sequence = seq_;
        now_ns_ += 2'000'000;   // one 500 Hz period
        obs.now_ns = now_ns_;
        return ctrl_.step(obs);
    }
    rb_servo::QueueSyncController& ctrl_;
    // Start well past whatever `run()` consumed, so time never goes backwards
    // (the law subtracts unsigned nanosecond stamps).
    uint64_t now_ns_ = 60'000'000'000ull;
    uint64_t seq_ = 1'000'000;
};

// Bring a controller to Track with the measured plant, so the tests below start
// from a locked regulator rather than from Idle.
bool reachTrack(rb_servo::QueueSyncController& ctrl, const rb_servo::QueueSyncConfig& cfg,
                RunResult& out) {
    BoxQueue box(499.34, cfg.target_fill);
    out = run(ctrl, box, 4000);
    return out.phase == "track";
}

// THE 2026-09-02 FIELD CASE, AS A TEST (servo_log_20260902_230031). The left arm's
// RBACK flow stopped 2 % into the run and never resumed -- 299 frames parsed against
// the right arm's 77,321 -- and `locked` stayed 1 for all 77,130 remaining ticks
// (~154 s) because it was computed from a `last_fill_` that persists forever.
//
// THE FROZEN VALUE WAS EXACTLY THE SETPOINT, which is why it was harmless and why a
// test on |fill - target| alone could never have caught it: every term of the old
// condition was legitimately satisfied. Only freshness distinguishes the two states.
bool testLockedGoesFalseWhenFeedbackFreezes() {
    const rb_servo::QueueSyncConfig cfg = testConfig();
    rb_servo::QueueSyncController ctrl(cfg);
    RunResult warm;
    RB_CHECK(reachTrack(ctrl, cfg, warm));
    RB_CHECK(warm.last.locked);
    RB_CHECK(warm.last.stale_cycles == 0);

    DirectFeed feed(ctrl);
    rb_servo::QueueSyncDecision d = feed.fresh(cfg.target_fill);
    RB_CHECK(d.locked);
    RB_CHECK(d.stale_cycles == 0);

    // The ACK flow dies here. The reported fill never changes, because nothing is
    // reporting it any more.
    for (int i = 0; i < cfg.stall_cycles; ++i) d = feed.stale(cfg.target_fill);
    RB_CHECK(d.stale_cycles >= cfg.stall_cycles);
    RB_CHECK(d.stall_events >= 1);
    RB_CHECK(!d.locked);                       // <-- the bug this test exists for

    // And it comes back the moment a real observation does.
    d = feed.fresh(cfg.target_fill);
    RB_CHECK(d.stale_cycles == 0);
    RB_CHECK(d.locked);
    return true;
}

// One reading at the floor is not evidence. `last_fill` is an integer scraped out of
// a TCP byte stream across chunk boundaries with a carry, and CM had exactly one bad
// sample of that quantity de-energize a healthy cell whose queue read 5 in 13,000+
// samples. We only count, so the stake here is whether the number can be CITED.
bool testUnderrunNeedsConsecutiveConfirmations() {
    const rb_servo::QueueSyncConfig cfg = testConfig();
    rb_servo::QueueSyncController ctrl(cfg);
    RunResult warm;
    RB_CHECK(reachTrack(ctrl, cfg, warm));
    const uint64_t base = warm.last.underrun_events;

    DirectFeed feed(ctrl);
    rb_servo::QueueSyncDecision d = feed.fresh(0);
    RB_CHECK(d.underrun_events == base);       // a single sample counts for nothing

    // A recovery RESETS the run rather than letting isolated samples accumulate.
    feed.fresh(cfg.target_fill);
    for (int i = 0; i < cfg.underrun_confirm - 1; ++i) {
        d = feed.fresh(0);
        RB_CHECK(d.underrun_events == base);
    }
    d = feed.fresh(0);
    RB_CHECK(d.underrun_events == base + 1);   // the Nth consecutive witness

    // STALE cycles are not witnesses either: they are the same reading seen again.
    for (int i = 0; i < 50; ++i) d = feed.stale(0);
    RB_CHECK(d.underrun_events == base + 1);
    return true;
}

// A queue hovering at the band edge reports 3,4,3,4,... Re-arming off the EDGE makes
// every return a fresh event -- at 500 fresh RBACKs a second that is an event every
// other cycle, which is a per-SAMPLE test wearing per-EPISODE clothes. CM hit exactly
// this and re-armed on recovery to the setpoint instead.
bool testBandEventsAreOncePerEpisode() {
    rb_servo::QueueSyncConfig cfg = testConfig();
    cfg.warn_fill = 3;    // leaves fill 4 as "out of band but still below target"
    rb_servo::QueueSyncController ctrl(cfg);
    RunResult warm;
    RB_CHECK(reachTrack(ctrl, cfg, warm));
    const uint64_t base_warn = warm.last.warn_events;
    const uint64_t base_dip = warm.last.dip_events;

    DirectFeed feed(ctrl);
    rb_servo::QueueSyncDecision d = feed.fresh(cfg.warn_fill);
    RB_CHECK(d.warn_events == base_warn + 1);

    for (int i = 0; i < 20; ++i) {
        feed.fresh(cfg.warn_fill + 1);         // off the edge, still under target
        d = feed.fresh(cfg.warn_fill);         // back into the band
    }
    RB_CHECK(d.warn_events == base_warn + 1);  // ONE episode, not 21
    RB_CHECK(d.dip_events == base_dip);        // and it has not ended yet

    // Recovery to TARGET is the honest end of the episode: it closes and re-arms.
    d = feed.fresh(cfg.target_fill);
    RB_CHECK(d.dip_events == base_dip + 1);
    RB_CHECK(d.dip_last_min == cfg.warn_fill);
    RB_CHECK(d.dip_last_ms > 0.0);
    d = feed.fresh(cfg.warn_fill);
    RB_CHECK(d.warn_events == base_warn + 2);  // a NEW episode does count
    return true;
}

// The dip report is only as good as its approach. CM's first underrun guard printed
// the word "QueueUnderrun" and not one number; the trace is what distinguishes a
// ramp-down (a real drain) from a one-observation vanish-and-return (a reporting
// glitch -- a queue fed one command per tick cannot climb to the setpoint in one).
bool testFillTraceRecordsFreshObservationsOnly() {
    rb_servo::QueueSyncConfig cfg = testConfig();
    cfg.warn_fill = 3;
    rb_servo::QueueSyncController ctrl(cfg);
    RunResult warm;
    RB_CHECK(reachTrack(ctrl, cfg, warm));

    DirectFeed feed(ctrl);
    const int ramp[] = {6, 5, 4, 3, 2, 1};
    rb_servo::QueueSyncDecision d;
    for (int v : ramp) d = feed.fresh(v);
    RB_CHECK(d.fill_trace_n >= 6);
    for (int i = 0; i < 6; ++i) {
        RB_CHECK(d.fill_trace[d.fill_trace_n - 6 + i] == ramp[i]);  // newest last
    }
    const int n_before = d.fill_trace_n;
    for (int i = 0; i < 10; ++i) d = feed.stale(1);
    RB_CHECK(d.fill_trace_n == n_before);      // a stale repeat is not an observation
    RB_CHECK(d.fill_trace[d.fill_trace_n - 1] == 1);

    // And it is bounded: the trace is a window, not a growing log.
    for (int i = 0; i < 100; ++i) d = feed.fresh(cfg.target_fill);
    RB_CHECK(d.fill_trace_n == rb_servo::kQueueSyncFillTraceN);
    return true;
}

struct NamedTest {
    const char* name;
    bool (*fn)();
};

}  // namespace

int main() {
    const std::vector<NamedTest> tests = {
        {"locks measured drift to target", testLocksMeasuredDriftToTarget},
        {"unregulated queue grows without bound", testUnregulatedQueueGrowsWithoutBound},
        {"drains large backlog quickly", testDrainsLargeBacklogQuickly},
        {"refills from empty", testRefillsFromEmpty},
        {"protect overrides PI when near empty", testProtectOverridesPiWhenNearEmpty},
        {"disabled is always neutral", testDisabledIsAlwaysNeutral},
        {"silent stream is neutral and re-arms", testSilentStreamIsNeutralAndReArms},
        {"stream gap re-runs warmup+drain on re-entry", testStreamGapReRunsWarmupAndDrainOnReEntry},
        {"rebase triggers redrain", testRebaseTriggersRedrain},
        {"stopped consumption is reported", testStoppedConsumptionIsReportedNotTrimmedAt},
        {"integral survives reset, level state does not", testIntegralSurvivesResetButLevelStateDoesNot},
        {"trim is bounded", testTrimIsBounded},
        {"both arms lock independently", testBothArmsLockIndependently},
        {"locked goes false when feedback freezes", testLockedGoesFalseWhenFeedbackFreezes},
        {"underrun needs consecutive confirmations", testUnderrunNeedsConsecutiveConfirmations},
        {"band events are once per episode", testBandEventsAreOncePerEpisode},
        {"fill trace records fresh observations only", testFillTraceRecordsFreshObservationsOnly},
    };
    int failed = 0;
    for (const NamedTest& t : tests) {
        if (!t.fn()) {
            std::cerr << "FAILED: " << t.name << "\n";
            ++failed;
        }
    }
    if (failed != 0) {
        std::cerr << failed << " queue_sync_controller test(s) failed\n";
        return 1;
    }
    std::cout << "all " << tests.size() << " queue_sync_controller tests passed\n";
    return 0;
}
