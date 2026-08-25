#include "rb_servo/control/queue_sync_controller.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {
namespace {

uint64_t secToNs(double sec) {
    if (!std::isfinite(sec) || sec <= 0.0) return 0;
    return static_cast<uint64_t>(sec * 1e9);
}

const char* phaseName(int phase) {
    switch (phase) {
        case 0: return "idle";
        case 1: return "warmup";
        case 2: return "drain";
        default: return "track";
    }
}

}  // namespace

QueueSyncController::QueueSyncController(QueueSyncConfig config)
    : config_(std::move(config)) {}

void QueueSyncController::reset() {
    phase_ = Phase::Idle;
    phase_start_ns_ = 0;
    last_rback_sequence_ = 0;
    last_fill_ = -1;
    fill_lpf_ = 0.0;
    stale_cycles_ = 0;
    underrun_active_ = false;
    highwater_active_ = false;
    consumption_ref_ns_ = 0;
    consumption_ref_fill_ = -1;
    // The warmup evidence is about THIS stream entry: a box that was consuming
    // before the gap says nothing about the one after it.
    warmup_sends_ = 0;
    warmup_fill_evidenced_ = false;
    warmup_probe_countdown_ = 0;
    // integral_us_ deliberately SURVIVES a reset: it encodes the box-vs-host
    // clock drift, which is a property of the hardware pair and does not change
    // because the stream paused. Re-learning it from zero would re-run the whole
    // drift transient after every task boundary.
}

QueueSyncDecision QueueSyncController::step(const Observation& obs) {
    QueueSyncDecision out = counters_;
    out.period_trim_us = 0.0;
    out.hold_send = false;

    const bool fresh = obs.fill_valid && obs.rback_sequence != last_rback_sequence_;
    if (fresh) {
        last_fill_ = obs.fill;
        last_rback_sequence_ = obs.rback_sequence;
    }

    if (!config_.enable || !obs.streaming) {
        // Nothing to regulate when the stream is silent. Drop back to Idle so the
        // next stream entry re-runs warmup+drain against the real queue instead
        // of acting on a pre-gap observation.
        if (phase_ != Phase::Idle) {
            phase_ = Phase::Idle;
            last_fill_ = -1;
            stale_cycles_ = 0;
            consumption_ref_ns_ = 0;
            consumption_ref_fill_ = -1;
            warmup_sends_ = 0;
            warmup_fill_evidenced_ = false;
            warmup_probe_countdown_ = 0;
        }
        out.last_fill = last_fill_;
        out.fill_valid = last_fill_ >= 0;
        out.fill_lpf = fill_lpf_;
        out.integral_us = integral_us_;
        out.phase = phaseName(0);
        out.locked = false;
        counters_ = out;
        counters_.period_trim_us = 0.0;
        return out;
    }

    // ---- consumption / backlog watches (every streaming phase) ----
    if (last_fill_ >= 0) {
        if (consumption_ref_ns_ == 0 || consumption_ref_fill_ < 0) {
            consumption_ref_ns_ = obs.now_ns;
            consumption_ref_fill_ = last_fill_;
        } else if (obs.now_ns - consumption_ref_ns_ >= 1'000'000'000ull) {
            // A rise this fast is not a clock-drift problem and a period trim
            // cannot fix it: the box has (nearly) stopped consuming. Report.
            if (last_fill_ - consumption_ref_fill_ > config_.no_consumption_rise_per_sec) {
                out.no_consumption_events += 1;
            }
            consumption_ref_ns_ = obs.now_ns;
            consumption_ref_fill_ = last_fill_;
        }
        if (last_fill_ >= config_.highwater_fill && !highwater_active_) {
            highwater_active_ = true;
            out.highwater_events += 1;
        } else if (last_fill_ < config_.highwater_fill / 2) {
            highwater_active_ = false;  // re-arm well below the level
        }
    }

    double trim_us = 0.0;
    switch (phase_) {
        case Phase::Idle:
            phase_ = Phase::Warmup;
            phase_start_ns_ = obs.now_ns;
            last_fill_ = -1;  // a stale pre-entry observation is meaningless here
            break;

        case Phase::Warmup: {
            // The box MAY show a startup transient that ramps the fill up over
            // ~0.5 s, and acting on that before it develops just fights it -- so
            // the exit test below waits for the ramp. But a fill pinned at 0 never
            // trips that test, and that is the case measured on fw v8.7.3: an
            // already-activated box consumed nothing for ~254 ms while reporting
            // 0, so full-rate streaming through Warmup buried ~128 commands in a
            // queue that then took 2.3 s to drain.
            //
            // EVIDENCE, not a level: a single observation above 0 proves the queue
            // can hold, and that is all we need. Until then, back off after a
            // bounded number of sends. Deliberately NOT a hard stop -- fill 0 is
            // also what a box consuming at exactly our rate looks like, and
            // silencing that one with no way to earn evidence would be worse than
            // the backlog. One probe every warmup_probe_interval keeps the
            // evidence path alive and caps the growth at 1/N of the send rate.
            // SENDS, not ticks: a held tick did not put anything in the box's
            // queue, so it is no evidence either way about whether the box consumes.
            if (obs.sent) ++warmup_sends_;
            if (last_fill_ > 0) warmup_fill_evidenced_ = true;
            if (!warmup_fill_evidenced_ &&
                warmup_sends_ >= config_.warmup_unevidenced_sends) {
                if (warmup_probe_countdown_ > 0) {
                    --warmup_probe_countdown_;
                    out.hold_send = true;
                    out.warmup_holds_total += 1;
                } else {
                    warmup_probe_countdown_ = std::max(1, config_.warmup_probe_interval) - 1;
                }
            }
            const uint64_t dt = obs.now_ns - phase_start_ns_;
            const bool developed =
                last_fill_ >= config_.target_fill + 3 && dt >= secToNs(config_.warmup_min_sec);
            if (developed || dt >= secToNs(config_.warmup_max_sec)) {
                phase_ = (last_fill_ > config_.target_fill + 1) ? Phase::Drain : Phase::Track;
                phase_start_ns_ = obs.now_ns;
                fill_lpf_ = last_fill_ >= 0 ? last_fill_ : config_.target_fill;
            }
            break;
        }

        case Phase::Drain: {
            // Send slower until the queue reaches the target, proportional to the
            // excess so a small overshoot drains gently and a large backlog still
            // clears in seconds rather than minutes.
            const double excess =
                last_fill_ >= 0 ? static_cast<double>(last_fill_ - config_.target_fill) : 0.0;
            trim_us = std::clamp(config_.drain_per_fill_us * excess,
                                 config_.drain_adj_us, config_.drain_max_us);
            const bool reached = fresh && last_fill_ >= 0 && last_fill_ <= config_.target_fill;
            if (reached || obs.now_ns - phase_start_ns_ >= secToNs(config_.drain_timeout_sec)) {
                phase_ = Phase::Track;
                phase_start_ns_ = obs.now_ns;
                fill_lpf_ = last_fill_ >= 0 ? last_fill_ : config_.target_fill;
            }
            break;
        }

        case Phase::Track: {
            // A queue REBASE (a jump far above target, e.g. at a motion boundary)
            // cannot be trimmed away a few microseconds at a time.
            if (last_fill_ >= config_.target_fill + config_.redrain_fill_margin) {
                phase_ = Phase::Drain;
                phase_start_ns_ = obs.now_ns;
                out.redrain_events += 1;
                trim_us = config_.drain_adj_us;
                break;
            }
            const bool low = last_fill_ >= 0 && last_fill_ <= config_.protect_fill;
            if (low && !underrun_active_) {
                underrun_active_ = true;
                out.underrun_events += 1;
            } else if (!low) {
                underrun_active_ = false;
            }
            stale_cycles_ = fresh ? 0 : stale_cycles_ + 1;
            if (stale_cycles_ == config_.stall_cycles) {
                out.stall_events += 1;  // edge: one report per stall
            }

            // Asymmetric PI on the filtered fill. The integral converges to the
            // per-cycle period extension that matches the box's consumption rate
            // (the clock-drift correction, and with it the phase lock); the
            // proportional term regulates the level to the target.
            if (fresh) fill_lpf_ += config_.lpf_alpha * (last_fill_ - fill_lpf_);
            const double error = fill_lpf_ - config_.target_fill;
            integral_us_ = std::clamp(integral_us_ + config_.ki_us * error,
                                      -config_.integral_clamp_us, config_.integral_clamp_us);
            const double kp = error >= 0.0 ? config_.kp_above_us : config_.kp_below_us;
            trim_us = std::clamp(kp * error + integral_us_,
                                 -config_.adj_clamp_us, config_.adj_clamp_us);
            // Underrun protection overrides the PI: an empty queue starves the box
            // (motion hiccup), whereas extra latency merely costs latency.
            if (low) trim_us = config_.protect_adj_us;
            break;
        }
    }

    out.period_trim_us = trim_us;
    out.fill_lpf = fill_lpf_;
    out.integral_us = integral_us_;
    out.last_fill = last_fill_;
    out.fill_valid = last_fill_ >= 0;
    out.phase = phaseName(static_cast<int>(phase_));
    out.locked = phase_ == Phase::Track && last_fill_ >= 0 &&
                 std::abs(last_fill_ - config_.target_fill) <= 1;
    counters_ = out;
    return out;
}

}  // namespace rb_servo
