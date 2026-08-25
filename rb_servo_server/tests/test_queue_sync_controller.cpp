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
        {"rebase triggers redrain", testRebaseTriggersRedrain},
        {"stopped consumption is reported", testStoppedConsumptionIsReportedNotTrimmedAt},
        {"integral survives reset, level state does not", testIntegralSurvivesResetButLevelStateDoesNot},
        {"trim is bounded", testTrimIsBounded},
        {"both arms lock independently", testBothArmsLockIndependently},
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
