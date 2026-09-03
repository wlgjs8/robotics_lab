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
    warn_active_ = false;
    dip_active_ = false;
    dip_start_ns_ = 0;
    dip_min_ = 0;
    underrun_run_ = 0;
    fill_trace_n_ = 0;
    consumption_ref_ns_ = 0;
    consumption_ref_fill_ = -1;
    // integral_us_ deliberately SURVIVES a reset: it encodes the box-vs-host
    // clock drift, which is a property of the hardware pair and does not change
    // because the stream paused. Re-learning it from zero would re-run the whole
    // drift transient after every task boundary.
}

QueueSyncDecision QueueSyncController::step(const Observation& obs) {
    QueueSyncDecision out = counters_;
    out.period_trim_us = 0.0;

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
            // An episode cannot span a stream gap: the queue it was about is gone.
            // A dip still open when the stream ends is dropped rather than reported,
            // because its duration would be the gap and not the dip.
            warn_active_ = false;
            dip_active_ = false;
            underrun_active_ = false;
            underrun_run_ = 0;
            fill_trace_n_ = 0;
        }
        out.last_fill = last_fill_;
        out.fill_valid = last_fill_ >= 0;
        out.fill_lpf = fill_lpf_;
        out.integral_us = integral_us_;
        out.phase = phaseName(0);
        out.locked = false;
        out.stale_cycles = 0;
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
            // A warmup back-pressure that throttled sends here was REMOVED on
            // 2026-08-26. It existed to bound the backlog built while the box
            // ignores the stream for its first ~254 ms; the stream is now simply
            // not started until the first motion command
            // (DualArmServoLoop::servo_stream_armed_), which is what
            // controller-manager does, so that window is no longer entered and a
            // second mechanism here would only be another thing to keep in sync.
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
            // ---- THE BAND, ON FRESH OBSERVATIONS ONLY -----------------------------
            // `last_fill_` PERSISTS between RBACKs, so counting off it every cycle
            // counts one reading many times -- and the case where that matters is the
            // dangerous one: a stalled ACK flow freezes the value at whatever it last
            // was, which may be in the band while the queue itself has since refilled.
            // `fresh` is the only thing that says the number describes NOW.
            if (fresh && last_fill_ >= 0) {
                if (fill_trace_n_ < kQueueSyncFillTraceN) {
                    fill_trace_[fill_trace_n_++] = last_fill_;
                } else {
                    for (int i = 1; i < kQueueSyncFillTraceN; ++i) fill_trace_[i - 1] = fill_trace_[i];
                    fill_trace_[kQueueSyncFillTraceN - 1] = last_fill_;
                }
                const bool in_band = last_fill_ <= config_.warn_fill;
                // A BAND TEST, NOT AN EQUALITY TEST. `== warn_fill` asks the sample to
                // land exactly on the number, so a drop that steps 5 -> 2 between two
                // observations skips it and reports nothing at all.
                if (in_band && !warn_active_) {
                    warn_active_ = true;
                    out.warn_events += 1;
                }
                if (in_band) {
                    if (!dip_active_) {
                        dip_active_ = true;
                        dip_start_ns_ = obs.now_ns;
                        dip_min_ = last_fill_;
                    } else if (last_fill_ < dip_min_) {
                        dip_min_ = last_fill_;
                    }
                }
                // CONFIRMED underrun. Counted in CONSECUTIVE fresh observations so a
                // single mis-scraped integer cannot create the event on its own.
                if (last_fill_ <= config_.protect_fill) {
                    if (++underrun_run_ >= config_.underrun_confirm && !underrun_active_) {
                        underrun_active_ = true;
                        out.underrun_events += 1;
                    }
                } else {
                    underrun_run_ = 0;
                }
                // RE-ARM ON RECOVERY TO **TARGET**, NOT MERELY OFF THE BAND EDGE, and
                // that is what makes these per-EPISODE. Re-arming at the edge turns a
                // queue hovering there (3,4,3,4,...) into an event every other
                // observation. Recovery to the setpoint is the honest end of an episode.
                if (last_fill_ >= config_.target_fill) {
                    if (dip_active_) {
                        dip_active_ = false;
                        dip_last_min_ = dip_min_;
                        dip_last_ms_ = static_cast<double>(obs.now_ns - dip_start_ns_) / 1.0e6;
                        out.dip_events += 1;
                    }
                    warn_active_ = false;
                    underrun_active_ = false;
                    underrun_run_ = 0;
                }
            }
            const bool low = last_fill_ >= 0 && last_fill_ <= config_.protect_fill;
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
    out.stale_cycles = stale_cycles_;
    out.dip_last_min = dip_last_min_;
    out.dip_last_ms = dip_last_ms_;
    for (int i = 0; i < fill_trace_n_; ++i) out.fill_trace[i] = fill_trace_[i];
    out.fill_trace_n = fill_trace_n_;
    // *** LOCKED REQUIRES THE FEEDBACK TO BE RECENT. *** Without this term `locked`
    // is computed from `last_fill_`, which persists forever, so a frozen reading that
    // happens to sit near the setpoint reports a phase lock indefinitely. That is not
    // hypothetical: on 2026-09-02 the left arm's RBACK flow died 2 % into a run and
    // this flag stayed 1 for all 77,130 remaining ticks (~154 s) with the loop fully
    // open (servo_log_20260902_230031). A health indicator that cannot go false when
    // the measurement stops is not an indicator. `stall_cycles` is the same threshold
    // that reports the stall, so the two agree by construction rather than by tuning.
    const bool feedback_recent = stale_cycles_ < config_.stall_cycles;
    out.locked = phase_ == Phase::Track && last_fill_ >= 0 && feedback_recent &&
                 std::abs(last_fill_ - config_.target_fill) <= 1;
    counters_ = out;
    return out;
}

}  // namespace rb_servo
