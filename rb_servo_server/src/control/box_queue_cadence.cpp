#include "rb_servo/control/box_queue_cadence.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {
namespace {
// Saturation for the "ticks since last skip" counters. Any value at or above the
// configured interval means "eligible"; this just stops them counting forever.
constexpr int kSkipIntervalCeiling = 1'000'000;
}  // namespace

const char* toString(BoxQueueCadence::Phase phase) {
    switch (phase) {
        case BoxQueueCadence::Phase::Disabled: return "disabled";
        case BoxQueueCadence::Phase::Warmup: return "warmup";
        case BoxQueueCadence::Phase::Tracking: return "tracking";
        case BoxQueueCadence::Phase::Stale: return "stale";
    }
    return "unknown";
}

BoxQueueCadence::BoxQueueCadence(BoxQueueCadenceConfig config)
    : config_(config) {
    phase_ = config_.enable ? Phase::Warmup : Phase::Disabled;
    ticks_since_skip_left_ = kSkipIntervalCeiling;
    ticks_since_skip_right_ = kSkipIntervalCeiling;
}

void BoxQueueCadence::reset() {
    phase_ = config_.enable ? Phase::Warmup : Phase::Disabled;
    elapsed_sec_ = 0.0;
    fill_lpf_ = 0.0;
    lpf_seeded_ = false;
    integral_us_ = 0.0;
    trim_us_ = 0.0;
    controlled_fill_ = -1;
    ticks_since_observation_ = 0;
    arm_fill_lpf_left_ = 0.0;
    arm_fill_lpf_right_ = 0.0;
    arm_lpf_seeded_left_ = false;
    arm_lpf_seeded_right_ = false;
    skip_next_left_ = false;
    skip_next_right_ = false;
    ticks_since_skip_left_ = kSkipIntervalCeiling;
    ticks_since_skip_right_ = kSkipIntervalCeiling;
    // skip_count_* deliberately survive reset: they are a session-long audit of
    // how much waypoint dropping this run did, not controller state.
}

std::int64_t BoxQueueCadence::update(
    std::optional<int> left_fill,
    std::optional<int> right_fill,
    double dt_sec
) {
    BoxQueueCadenceInput input;
    input.left_fill = left_fill;
    input.right_fill = right_fill;
    input.dt_sec = dt_sec;
    // Speeds left at 0. That reads as "stationary", which would pass the speed
    // gate -- harmless because this overload is only used where the per-arm level
    // trim is disabled, and it stays correct if that ever changes because a
    // stationary arm is exactly when a skip is free.
    return update(input);
}

std::int64_t BoxQueueCadence::update(const BoxQueueCadenceInput& input) {
    const std::optional<int>& left_fill = input.left_fill;
    const std::optional<int>& right_fill = input.right_fill;
    const double dt_sec = input.dt_sec;

    // Skip decisions are single-tick. Clear them first so every early return
    // below leaves both arms sending, which is the safe default.
    skip_next_left_ = false;
    skip_next_right_ = false;
    if (ticks_since_skip_left_ < kSkipIntervalCeiling) ++ticks_since_skip_left_;
    if (ticks_since_skip_right_ < kSkipIntervalCeiling) ++ticks_since_skip_right_;

    if (!config_.enable) {
        phase_ = Phase::Disabled;
        trim_us_ = 0.0;
        controlled_fill_ = -1;
        return 0;
    }

    // Drive the SHALLOWER queue: see the header on why underrun outranks latency
    // when one trim has to serve two boxes. An arm that is not being streamed
    // produces no RBACK, so it drops out of this on its own.
    std::optional<int> observed;
    if (left_fill.has_value() && right_fill.has_value()) {
        observed = std::min(*left_fill, *right_fill);
    } else if (left_fill.has_value()) {
        observed = left_fill;
    } else if (right_fill.has_value()) {
        observed = right_fill;
    }

    if (observed.has_value()) {
        ticks_since_observation_ = 0;
    } else if (ticks_since_observation_ < config_.stale_observation_ticks) {
        ++ticks_since_observation_;
    }
    controlled_fill_ = observed.value_or(-1);

    const bool stale =
        config_.stale_observation_ticks > 0 &&
        ticks_since_observation_ >= config_.stale_observation_ticks;
    if (stale) {
        // The box stopped answering. Release the correction rather than hold a
        // trim justified by a queue picture we can no longer see, and rewind the
        // warmup clock: when it starts answering again the queue may be nothing
        // like what we last saw, so re-observe before acting on it.
        phase_ = Phase::Stale;
        integral_us_ = 0.0;
        lpf_seeded_ = false;
        arm_lpf_seeded_left_ = false;
        arm_lpf_seeded_right_ = false;
        trim_us_ = 0.0;
        elapsed_sec_ = 0.0;
        return 0;
    }

    if (std::isfinite(dt_sec) && dt_sec > 0.0) {
        elapsed_sec_ += dt_sec;
    }

    // Observe before acting: never trim off a queue we have not actually seen.
    if (phase_ == Phase::Warmup || phase_ == Phase::Stale) {
        if (!observed.has_value() || elapsed_sec_ < config_.warmup_sec) {
            if (phase_ != Phase::Warmup) phase_ = Phase::Warmup;
            trim_us_ = 0.0;
            return 0;
        }
        phase_ = Phase::Tracking;
        fill_lpf_ = static_cast<double>(*observed);
        lpf_seeded_ = true;
    }

    // The low-pass only advances on a fresh reading (a repeated stale value would
    // otherwise drag it), but the integral advances every cycle: it is tracking a
    // constant clock offset, which accrues whether or not the box just spoke.
    if (observed.has_value()) {
        if (!lpf_seeded_) {
            fill_lpf_ = static_cast<double>(*observed);
            lpf_seeded_ = true;
        } else {
            fill_lpf_ += config_.lpf_alpha * (static_cast<double>(*observed) - fill_lpf_);
        }
    }

    const double error = fill_lpf_ - static_cast<double>(config_.target_fill);
    integral_us_ += config_.ki_us * error;
    integral_us_ = std::clamp(integral_us_, -config_.integral_clamp_us, config_.integral_clamp_us);

    const double kp = (error >= 0.0) ? config_.kp_above_us : config_.kp_below_us;
    double trim_us = kp * error + integral_us_;
    trim_us = std::clamp(trim_us, -config_.trim_clamp_us, config_.trim_clamp_us);

    // Underrun override: act on the RAW depth, not the low-passed one. By the
    // time a 0.1 s filter admits that the queue is empty the arm has already
    // stalled. This deliberately outranks the PI.
    if (observed.has_value() && *observed <= config_.protect_fill) {
        trim_us = config_.protect_trim_us;
    }

    trim_us_ = trim_us;

    // The shared trim above is a rate lever and can only hold one arm. Level the
    // other one here.
    updateArmLevel(input);

    return static_cast<std::int64_t>(trim_us * 1000.0);
}

void BoxQueueCadence::updateArmLevel(const BoxQueueCadenceInput& input) {
    const BoxQueueLevelConfig& level = config_.level;

    // Advance the per-arm filters whenever an arm speaks, even with the feature
    // off, so enabling it mid-session does not act on an unseeded filter.
    const auto advance = [&](std::optional<int> fill, double& lpf, bool& seeded) {
        if (!fill.has_value()) return;
        if (!seeded) {
            lpf = static_cast<double>(*fill);
            seeded = true;
        } else {
            lpf += config_.lpf_alpha * (static_cast<double>(*fill) - lpf);
        }
    };
    advance(input.left_fill, arm_fill_lpf_left_, arm_lpf_seeded_left_);
    advance(input.right_fill, arm_fill_lpf_right_, arm_lpf_seeded_right_);

    if (!level.enable || phase_ != Phase::Tracking) return;

    const double target = static_cast<double>(config_.target_fill);
    const int raw_floor = config_.target_fill + level.skip_floor_offset_ticks;
    const int min_interval = std::max(2, level.skip_min_interval_ticks);

    // An arm may be skipped only if EVERY one of these holds. Ordered cheapest
    // first; each is load-bearing and none is a heuristic.
    const auto eligible = [&](std::optional<int> fill, double lpf, bool seeded,
                              double speed_deg_s, int ticks_since_skip) {
        // Fresh observation required: a skip produces no RBACK for that arm, so
        // deciding without one would run the level loop open.
        if (!fill.has_value() || !seeded) return false;
        // Raw floor. Independent of the filter, so no filter state can talk us
        // into skipping an arm that is actually at or below target.
        if (*fill < raw_floor) return false;
        // Filtered margin. See the header on why this is not the raw value.
        if (lpf - target <= level.skip_margin_ticks) return false;
        // The speed gate. A dropped waypoint costs a one-tick velocity doubling;
        // this is what makes that cost negligible.
        if (!(speed_deg_s <= level.skip_max_joint_speed_deg_s)) return false;
        // Rate limit, and never two ticks in a row on the same arm.
        if (ticks_since_skip < min_interval) return false;
        return true;
    };

    const bool left_ok = eligible(input.left_fill, arm_fill_lpf_left_, arm_lpf_seeded_left_,
                                  input.left_joint_speed_deg_s, ticks_since_skip_left_);
    const bool right_ok = eligible(input.right_fill, arm_fill_lpf_right_, arm_lpf_seeded_right_,
                                   input.right_joint_speed_deg_s, ticks_since_skip_right_);

    if (!left_ok && !right_ok) return;

    // Never both on the same tick: skipping both is a no-op for the differential
    // we are correcting, and it would put a waypoint drop on two arms at once for
    // no gain. Deeper arm first; the other one gets its turn next interval.
    const bool take_left = left_ok && (!right_ok || arm_fill_lpf_left_ >= arm_fill_lpf_right_);
    if (take_left) {
        skip_next_left_ = true;
        ticks_since_skip_left_ = 0;
        ++skip_count_left_;
    } else {
        skip_next_right_ = true;
        ticks_since_skip_right_ = 0;
        ++skip_count_right_;
    }
}

}  // namespace rb_servo
