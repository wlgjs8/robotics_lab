#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

#include "rb_servo/control/box_queue_cadence.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

constexpr double kNominalPeriodSec = 0.002;  // 500 Hz

rb_servo::BoxQueueCadenceConfig enabledConfig() {
    rb_servo::BoxQueueCadenceConfig config;
    config.enable = true;
    return config;
}

// The box queue is a pure integrator: we add one command per tick, the box
// removes them at its own steady rate. `consume_hz` below 1/period is exactly the
// host-vs-box clock offset that makes the real queue grow.
class QueuePlant {
public:
    QueuePlant(double fill, double consume_hz) : fill_(fill), consume_hz_(consume_hz) {}

    // Advance one tick of length `dt_sec` during which we sent one command.
    void step(double dt_sec) {
        fill_ += 1.0 - consume_hz_ * dt_sec;
        if (fill_ < 0.0) fill_ = 0.0;  // the box cannot execute what it does not have
    }

    int observedFill() const { return static_cast<int>(fill_); }
    double fill() const { return fill_; }

private:
    double fill_;
    double consume_hz_;
};

// Runs the controller against the plant and returns the fill history.
std::vector<double> simulate(
    rb_servo::BoxQueueCadence& cadence,
    QueuePlant& plant,
    int ticks
) {
    std::vector<double> history;
    history.reserve(static_cast<std::size_t>(ticks));
    std::int64_t trim_ns = 0;
    for (int i = 0; i < ticks; ++i) {
        const double dt = kNominalPeriodSec + static_cast<double>(trim_ns) * 1e-9;
        plant.step(dt);
        const int fill = plant.observedFill();
        trim_ns = cadence.update(fill, fill, dt);
        history.push_back(plant.fill());
    }
    return history;
}

// Opting out must leave the loop on its plain fixed tick.
bool testDisabledNeverTrims() {
    rb_servo::BoxQueueCadenceConfig config;  // enable defaults to false
    rb_servo::BoxQueueCadence cadence(config);
    for (int i = 0; i < 1000; ++i) {
        RB_CHECK(cadence.update(40, 40, kNominalPeriodSec) == 0);
    }
    RB_CHECK(cadence.phase() == rb_servo::BoxQueueCadence::Phase::Disabled);
    return true;
}

// No correction may be applied off a queue we have not watched for long enough.
bool testWarmupHoldsTrimAtZero() {
    auto config = enabledConfig();
    config.warmup_sec = 0.4;
    rb_servo::BoxQueueCadence cadence(config);

    const int warmup_ticks = static_cast<int>(config.warmup_sec / kNominalPeriodSec);
    for (int i = 0; i < warmup_ticks - 1; ++i) {
        RB_CHECK(cadence.update(40, 40, kNominalPeriodSec) == 0);
        RB_CHECK(cadence.phase() == rb_servo::BoxQueueCadence::Phase::Warmup);
    }
    // Past warmup it must start acting on the (deep) queue.
    for (int i = 0; i < 5; ++i) {
        cadence.update(40, 40, kNominalPeriodSec);
    }
    RB_CHECK(cadence.phase() == rb_servo::BoxQueueCadence::Phase::Tracking);
    RB_CHECK(cadence.trimUs() > 0.0);
    return true;
}

// Sign check, stated independently of the plant: a queue deeper than target must
// LENGTHEN the period (send slower), a shallower one must SHORTEN it. Getting
// this backwards would drive the queue away without bound.
bool testTrimSignFollowsQueueDepth() {
    auto config = enabledConfig();
    rb_servo::BoxQueueCadence deep(config);
    for (int i = 0; i < 500; ++i) deep.update(40, 40, kNominalPeriodSec);
    RB_CHECK(deep.trimUs() > 0.0);

    rb_servo::BoxQueueCadence shallow(config);
    // Stay above protect_fill so this exercises the PI, not the override.
    for (int i = 0; i < 500; ++i) shallow.update(3, 3, kNominalPeriodSec);
    RB_CHECK(shallow.trimUs() < 0.0);
    return true;
}

// The whole point: a queue that would otherwise grow without bound must settle
// at the setpoint and stay there.
bool testConvergesToTargetAndHolds() {
    auto config = enabledConfig();
    rb_servo::BoxQueueCadence cadence(config);
    // 499.4 Hz reproduces the +0.65 ticks/s measured on hardware 2026-08-18.
    QueuePlant plant(45.0, 499.4);

    const auto history = simulate(cadence, plant, 15000);  // 30 s

    RB_CHECK(cadence.phase() == rb_servo::BoxQueueCadence::Phase::Tracking);
    const double settled = history.back();
    if (std::abs(settled - config.target_fill) > 2.0) {
        std::cerr << "did not settle: final fill=" << settled
                  << " target=" << config.target_fill << "\n";
        return false;
    }
    // And it must have actually come DOWN from the backlog, not merely stopped
    // growing at some arbitrary depth.
    RB_CHECK(history.front() > 40.0);

    // Stationarity is the property the actual_lead budgets depend on: the second
    // half of the run must not drift.
    const double mid = history[history.size() / 2];
    if (std::abs(settled - mid) > 1.5) {
        std::cerr << "still drifting: mid=" << mid << " final=" << settled << "\n";
        return false;
    }
    return true;
}

// Convergence must not depend on starting deep; a fresh box starts near empty.
bool testConvergesFromEmpty() {
    auto config = enabledConfig();
    rb_servo::BoxQueueCadence cadence(config);
    QueuePlant plant(0.0, 499.4);
    const auto history = simulate(cadence, plant, 15000);
    RB_CHECK(std::abs(history.back() - config.target_fill) <= 2.0);
    return true;
}

// An empty queue means the box has no next q_ref and holds the last one -- a
// stall the arm feels as a jerk. The override must fire off the RAW depth and
// outrank whatever the PI wanted.
bool testUnderrunOverrideOutranksThePi() {
    auto config = enabledConfig();
    rb_servo::BoxQueueCadence cadence(config);

    // Sit deep long enough that the PI is asking to drain (positive trim).
    for (int i = 0; i < 1000; ++i) cadence.update(40, 40, kNominalPeriodSec);
    RB_CHECK(cadence.trimUs() > 0.0);

    // A single near-empty reading must immediately flip to wake-early, even
    // though the low-passed fill is still deep.
    const std::int64_t trim_ns = cadence.update(config.protect_fill, config.protect_fill,
                                                kNominalPeriodSec);
    RB_CHECK(cadence.trimUs() == config.protect_trim_us);
    RB_CHECK(trim_ns < 0);
    return true;
}

// One trim serves two boxes, so it must protect the shallower queue.
bool testControlsOnTheShallowerArm() {
    auto config = enabledConfig();
    rb_servo::BoxQueueCadence cadence(config);
    for (int i = 0; i < 1000; ++i) cadence.update(40, 3, kNominalPeriodSec);
    RB_CHECK(cadence.controlledFill() == 3);
    // Shallow arm below target -> speed up, despite the other arm being deep.
    RB_CHECK(cadence.trimUs() < 0.0);

    // An arm that is not being streamed reports nothing and must simply drop out
    // rather than be read as an empty queue.
    rb_servo::BoxQueueCadence single(config);
    for (int i = 0; i < 1000; ++i) single.update(std::nullopt, 40, kNominalPeriodSec);
    RB_CHECK(single.controlledFill() == 40);
    RB_CHECK(single.trimUs() > 0.0);
    return true;
}

// A box that stops answering must not leave a correction applied off a queue
// picture we can no longer see.
bool testStaleObservationsReleaseTheTrim() {
    auto config = enabledConfig();
    rb_servo::BoxQueueCadence cadence(config);
    for (int i = 0; i < 1000; ++i) cadence.update(40, 40, kNominalPeriodSec);
    RB_CHECK(cadence.trimUs() > 0.0);

    std::int64_t trim_ns = 1;
    for (int i = 0; i < config.stale_observation_ticks + 1; ++i) {
        trim_ns = cadence.update(std::nullopt, std::nullopt, kNominalPeriodSec);
    }
    RB_CHECK(cadence.phase() == rb_servo::BoxQueueCadence::Phase::Stale);
    RB_CHECK(trim_ns == 0);
    RB_CHECK(cadence.trimUs() == 0.0);

    // Recovery must go back through warmup, not resume mid-correction.
    cadence.update(40, 40, kNominalPeriodSec);
    RB_CHECK(cadence.phase() == rb_servo::BoxQueueCadence::Phase::Warmup);
    return true;
}

// The clamp is what keeps the emitted q_ref spacing close to servo_t1_sec; a
// wider trim risks the box running dry between commands. It must hold under any
// error, including the absurd.
bool testTrimStaysWithinTheClamp() {
    auto config = enabledConfig();
    rb_servo::BoxQueueCadence cadence(config);
    for (const int fill : {0, 1, 5, 50, 500, 100000}) {
        for (int i = 0; i < 600; ++i) {
            const std::int64_t trim_ns = cadence.update(fill, fill, kNominalPeriodSec);
            const double trim_us = static_cast<double>(trim_ns) / 1000.0;
            if (trim_us > config.trim_clamp_us + 1e-9 ||
                trim_us < -config.trim_clamp_us - 1e-9) {
                std::cerr << "trim escaped clamp: fill=" << fill
                          << " trim_us=" << trim_us << "\n";
                return false;
            }
        }
    }
    return true;
}

// reset() is called when the command stream stops; a resumed stream must
// re-observe rather than act on the queue picture it left behind.
bool testResetReturnsToWarmup() {
    auto config = enabledConfig();
    rb_servo::BoxQueueCadence cadence(config);
    for (int i = 0; i < 1000; ++i) cadence.update(40, 40, kNominalPeriodSec);
    RB_CHECK(cadence.trimUs() != 0.0);

    cadence.reset();
    RB_CHECK(cadence.phase() == rb_servo::BoxQueueCadence::Phase::Warmup);
    RB_CHECK(cadence.trimUs() == 0.0);
    RB_CHECK(cadence.integralUs() == 0.0);
    RB_CHECK(cadence.update(40, 40, kNominalPeriodSec) == 0);
    return true;
}


// ---------------------------------------------------------------------------
// Per-arm level trim (servo.box_queue_cadence.level)
// ---------------------------------------------------------------------------

rb_servo::BoxQueueCadenceConfig levelConfig() {
    auto config = enabledConfig();
    config.level.enable = true;
    return config;
}

// Drive both arms to Tracking with the given depths, then return the controller
// so a test can inspect the skip decision it latched for the next tick.
void settle(
    rb_servo::BoxQueueCadence& cadence,
    int left_fill,
    int right_fill,
    double left_speed,
    double right_speed,
    int ticks
) {
    for (int i = 0; i < ticks; ++i) {
        rb_servo::BoxQueueCadenceInput in;
        in.left_fill = left_fill;
        in.right_fill = right_fill;
        in.left_joint_speed_deg_s = left_speed;
        in.right_joint_speed_deg_s = right_speed;
        in.dt_sec = kNominalPeriodSec;
        cadence.update(in);
    }
}

// The whole point: the arm the shared rate trim is NOT holding gets levelled.
bool testLevelSkipsOnlyTheDeepArm() {
    rb_servo::BoxQueueCadence cadence(levelConfig());
    settle(cadence, 5, 20, 0.0, 0.0, 2000);
    RB_CHECK(cadence.skipCount(rb_servo::ArmId::Right) > 0);
    RB_CHECK(cadence.skipCount(rb_servo::ArmId::Left) == 0);
    return true;
}

// The safety argument in the header is "a skip is only ever issued to an arm
// above target". If that ever stops holding, this is the test that says so.
bool testLevelNeverSkipsAnArmAtOrBelowTarget() {
    auto config = levelConfig();
    rb_servo::BoxQueueCadence cadence(config);
    settle(cadence, config.target_fill, config.target_fill, 0.0, 0.0, 4000);
    RB_CHECK(cadence.skipCount(rb_servo::ArmId::Left) == 0);
    RB_CHECK(cadence.skipCount(rb_servo::ArmId::Right) == 0);

    rb_servo::BoxQueueCadence shallow(config);
    settle(shallow, 2, 2, 0.0, 0.0, 4000);
    RB_CHECK(shallow.skipCount(rb_servo::ArmId::Left) == 0);
    RB_CHECK(shallow.skipCount(rb_servo::ArmId::Right) == 0);
    return true;
}

// The speed gate is what makes a dropped waypoint cheap, so a moving arm must
// not be skipped no matter how deep its queue is.
bool testLevelSpeedGateBlocksSkipsWhileMoving() {
    auto config = levelConfig();
    rb_servo::BoxQueueCadence cadence(config);
    const double fast = config.level.skip_max_joint_speed_deg_s * 10.0;
    settle(cadence, 5, 20, fast, fast, 3000);
    RB_CHECK(cadence.skipCount(rb_servo::ArmId::Right) == 0);

    // Same depth, now slow: the skip that was withheld above must appear.
    settle(cadence, 5, 20, 0.0, 0.0, 100);
    RB_CHECK(cadence.skipCount(rb_servo::ArmId::Right) > 0);
    return true;
}

// Skipping both arms in one tick does nothing for the differential we are
// correcting and drops a waypoint on two arms for no gain.
bool testLevelNeverSkipsBothArmsOnTheSameTick() {
    rb_servo::BoxQueueCadence cadence(levelConfig());
    for (int i = 0; i < 3000; ++i) {
        rb_servo::BoxQueueCadenceInput in;
        in.left_fill = 20;
        in.right_fill = 20;
        in.dt_sec = kNominalPeriodSec;
        cadence.update(in);
        RB_CHECK(!(cadence.skipNext(rb_servo::ArmId::Left) &&
                   cadence.skipNext(rb_servo::ArmId::Right)));
    }
    return true;
}

// A skipped send returns no RBACK, so the next decision has to wait for the
// observation that reflects it. Two skips in a row would be running open.
bool testLevelNeverSkipsTwoTicksInARow() {
    rb_servo::BoxQueueCadence cadence(levelConfig());
    bool prev = false;
    for (int i = 0; i < 3000; ++i) {
        rb_servo::BoxQueueCadenceInput in;
        in.left_fill = 30;
        in.right_fill = 5;
        in.dt_sec = kNominalPeriodSec;
        cadence.update(in);
        const bool now = cadence.skipNext(rb_servo::ArmId::Left);
        RB_CHECK(!(now && prev));
        prev = now;
    }
    return true;
}

// Deciding without a fresh observation would act on a queue picture that may
// already be one skip out of date.
bool testLevelRequiresAFreshObservation() {
    rb_servo::BoxQueueCadence cadence(levelConfig());
    settle(cadence, 5, 20, 0.0, 0.0, 1000);
    const std::uint64_t before = cadence.skipCount(rb_servo::ArmId::Right);

    for (int i = 0; i < 10; ++i) {
        rb_servo::BoxQueueCadenceInput in;
        in.left_fill = 5;
        in.right_fill = std::nullopt;  // right said nothing this tick
        in.dt_sec = kNominalPeriodSec;
        cadence.update(in);
        RB_CHECK(!cadence.skipNext(rb_servo::ArmId::Right));
    }
    RB_CHECK(cadence.skipCount(rb_servo::ArmId::Right) == before);
    return true;
}

// Off by default, and the depth-only overload must never reach the level trim.
bool testLevelDisabledByDefault() {
    rb_servo::BoxQueueCadence cadence(enabledConfig());
    for (int i = 0; i < 3000; ++i) cadence.update(5, 40, kNominalPeriodSec);
    RB_CHECK(cadence.skipCount(rb_servo::ArmId::Left) == 0);
    RB_CHECK(cadence.skipCount(rb_servo::ArmId::Right) == 0);
    RB_CHECK(!cadence.skipNext(rb_servo::ArmId::Right));
    return true;
}

// Warmup and Stale are both "we do not trust the queue picture" states, and a
// waypoint drop is not something to do on an untrusted picture.
bool testLevelIsSilentOutsideTracking() {
    auto config = levelConfig();
    rb_servo::BoxQueueCadence cadence(config);
    // Warmup: deep queue, stationary arm, but not enough elapsed time yet.
    for (int i = 0; i < 10; ++i) {
        rb_servo::BoxQueueCadenceInput in;
        in.left_fill = 40;
        in.right_fill = 40;
        in.dt_sec = kNominalPeriodSec;
        cadence.update(in);
        RB_CHECK(cadence.phase() == rb_servo::BoxQueueCadence::Phase::Warmup);
        RB_CHECK(!cadence.skipNext(rb_servo::ArmId::Left));
        RB_CHECK(!cadence.skipNext(rb_servo::ArmId::Right));
    }
    // Stale: the box stopped answering.
    settle(cadence, 5, 40, 0.0, 0.0, 1000);
    for (int i = 0; i < config.stale_observation_ticks + 5; ++i) {
        rb_servo::BoxQueueCadenceInput in;
        in.dt_sec = kNominalPeriodSec;
        cadence.update(in);
    }
    RB_CHECK(cadence.phase() == rb_servo::BoxQueueCadence::Phase::Stale);
    RB_CHECK(!cadence.skipNext(rb_servo::ArmId::Left));
    RB_CHECK(!cadence.skipNext(rb_servo::ArmId::Right));
    return true;
}

// End to end against the plant: a deep arm converges to the target band and
// stops skipping, rather than skipping forever or walking past target.
bool testLevelConvergesAndThenGoesQuiet() {
    auto config = levelConfig();
    rb_servo::BoxQueueCadence cadence(config);
    double right_fill = 20.0;
    std::uint64_t skips_in_last_second = 0;
    for (int i = 0; i < 6000; ++i) {
        rb_servo::BoxQueueCadenceInput in;
        in.left_fill = config.target_fill;
        in.right_fill = static_cast<int>(right_fill);
        in.dt_sec = kNominalPeriodSec;
        const std::uint64_t before = cadence.skipCount(rb_servo::ArmId::Right);
        cadence.update(in);
        const bool skipped = cadence.skipCount(rb_servo::ArmId::Right) > before;
        if (skipped) right_fill -= 1.0;   // the skip the plant would have seen
        if (i >= 5500 && skipped) ++skips_in_last_second;
    }
    RB_CHECK(right_fill >= static_cast<double>(config.target_fill));
    RB_CHECK(right_fill <= static_cast<double>(config.target_fill) + 1.0);
    RB_CHECK(skips_in_last_second == 0);
    return true;
}

}  // namespace

int main() {
    if (!testDisabledNeverTrims()) return 1;
    if (!testWarmupHoldsTrimAtZero()) return 1;
    if (!testTrimSignFollowsQueueDepth()) return 1;
    if (!testConvergesToTargetAndHolds()) return 1;
    if (!testConvergesFromEmpty()) return 1;
    if (!testUnderrunOverrideOutranksThePi()) return 1;
    if (!testControlsOnTheShallowerArm()) return 1;
    if (!testStaleObservationsReleaseTheTrim()) return 1;
    if (!testTrimStaysWithinTheClamp()) return 1;
    if (!testResetReturnsToWarmup()) return 1;
    if (!testLevelSkipsOnlyTheDeepArm()) return 1;
    if (!testLevelNeverSkipsAnArmAtOrBelowTarget()) return 1;
    if (!testLevelSpeedGateBlocksSkipsWhileMoving()) return 1;
    if (!testLevelNeverSkipsBothArmsOnTheSameTick()) return 1;
    if (!testLevelNeverSkipsTwoTicksInARow()) return 1;
    if (!testLevelRequiresAFreshObservation()) return 1;
    if (!testLevelDisabledByDefault()) return 1;
    if (!testLevelIsSilentOutsideTracking()) return 1;
    if (!testLevelConvergesAndThenGoesQuiet()) return 1;
    return 0;
}
