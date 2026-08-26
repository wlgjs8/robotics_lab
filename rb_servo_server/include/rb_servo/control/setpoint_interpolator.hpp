#pragma once

// Worker-side setpoint RATE CONVERSION (servo.worker_setpoint_interpolation).
//
// The servo loop produces setpoints on the host clock (nominal 500.000 Hz,
// measured 500.006) while the cadence-owning worker sends on the box-locked
// period (nominal + queue-sync trim, ~499.35 Hz). The difference —
// ~0.65 setpoints/s per arm — previously vanished in the latest-wins mailbox:
// one setpoint per beat (~1.5 s) was silently overwritten, and the box FIFO
// played a DOUBLE step where it was skipped (measured 2026-08-26:
// review_request_2026-08-26/README.md §3, confirmed real by the
// position-conservation test — the [lag7,lag8] window carries 3v, not 2v).
//
// Discarding ~0.65/s is arithmetically unavoidable when the producer runs
// 0.13 % faster than the wire; the only choice is WHERE the discontinuity
// goes. A per-tick delta clamp would accumulate an unbounded deficit (the
// producer is permanently faster). Rate conversion puts it nowhere: the
// worker walks a continuous cursor through the setpoint sequence at
// exactly (send_period + trim) / loop_period setpoints per send — the same
// trim the queue-sync law derives from the box clock — and emits the linear
// interpolation between the two bracketing setpoints. The 0.13 % mismatch
// becomes a uniform time dilation (every step is 1.0013x) instead of a 2x
// step once per beat. Cost: one setpoint (~2 ms) of added latency, which is
// why this is config-gated. The per-arm ownership plan
// (omx_wiki/per-arm-control-ownership-plan.md) removes the cost structurally;
// this is the interim 80 %.
//
// Note on the resampling prohibition: controller-manager forbids fractional
// cursors on RECORDED TABLES ("no re-timing of a recorded table"). This is a
// live setpoint stream, where constant-rate consumption of a paced input is
// exactly what CM's own FollowUnit does. The interpolation is between two
// adjacent 2 ms samples of a jerk-limited trajectory — content the plan
// itself authored 2 ms apart.
//
// Pure, single-threaded state machine: the caller (ArmWorker) serializes
// push()/sample() under its own mutex. Deterministic — unit-tested in
// tests/test_setpoint_interpolator.cpp.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include "rb_servo/core/types.hpp"
#include "rb_servo/robot/backend_result.hpp"

namespace rb_servo {

class SetpointInterpolator {
public:
    struct Telemetry {
        bool active = false;             // cursor established (>= 2 setpoints seen)
        double delay_setpoints = 0.0;    // newest index - cursor (latency, in setpoints)
        uint64_t rebase_total = 0;       // cursor fell too far behind -> stepped forward
        uint64_t hold_total = 0;         // cursor pinned at newest (producer stalled)
    };

    // Producer stalled long enough that continuity is gone; start fresh.
    void reset() {
        head_ = 0;
        cursor_ = -1.0;
        telemetry_.active = false;
        telemetry_.delay_setpoints = 0.0;
    }

    // A new loop setpoint arrived (call under the caller's lock).
    void push(const SendServoJRequest& request) {
        ring_[head_ % kRing] = request;
        ++head_;
    }

    bool hasSetpoint() const { return head_ > 0; }

    // Sample the stream at this send instant. `period_ratio` is the actual
    // send period over the nominal loop period, e.g. (2000 + trim_us) / 2000.
    // Returns nullopt only before the first push.
    std::optional<SendServoJRequest> sample(double period_ratio) {
        if (head_ == 0) return std::nullopt;
        const double newest = static_cast<double>(head_ - 1);
        if (head_ == 1 || cursor_ < 0.0) {
            if (head_ >= 2) {
                // Establish the steady one-setpoint delay as soon as a bracket
                // exists; before that, bridge with the newest sample verbatim.
                cursor_ = newest - 1.0;
                telemetry_.active = true;
            } else {
                telemetry_.delay_setpoints = 0.0;
                return ring_[(head_ - 1) % kRing];
            }
        }
        if (!(std::isfinite(period_ratio)) || period_ratio <= 0.0) {
            period_ratio = 1.0;
        }
        cursor_ += period_ratio;
        if (cursor_ > newest) {
            // Producer stalled (no fresh bracket): hold at the newest setpoint,
            // which is exactly the legacy repeat-last-setpoint wire behaviour.
            cursor_ = newest;
            ++telemetry_.hold_total;
        }
        if (newest - cursor_ > kMaxDelay) {
            // Producer burst / worker stall: do not replay the backlog, step to
            // the steady delay in one move (a single logged discontinuity).
            cursor_ = newest - 1.0;
            ++telemetry_.rebase_total;
        }
        telemetry_.delay_setpoints = newest - cursor_;

        const double floor_idx = std::floor(cursor_);
        const uint64_t i0 = static_cast<uint64_t>(floor_idx);
        const uint64_t i1 = std::min<uint64_t>(i0 + 1, head_ - 1);
        const double frac = cursor_ - floor_idx;
        const SendServoJRequest& a = ring_[i0 % kRing];
        const SendServoJRequest& b = ring_[i1 % kRing];
        SendServoJRequest out = b;  // metadata (seq/host_time/deadline) from the
                                    // newer bracket: honest within one setpoint
        for (std::size_t j = 0; j < out.q_target_deg.size(); ++j) {
            out.q_target_deg[j] =
                a.q_target_deg[j] + frac * (b.q_target_deg[j] - a.q_target_deg[j]);
        }
        return out;
    }

    const Telemetry& telemetry() const { return telemetry_; }

private:
    // Ring depth 4 covers the cursor's reachable window: the low clamp keeps
    // newest - cursor <= kMaxDelay (2.5), so floor(cursor) >= newest - 3.
    static constexpr int kRing = 4;
    static constexpr double kMaxDelay = 2.5;
    std::array<SendServoJRequest, kRing> ring_{};
    uint64_t head_ = 0;     // total pushes; ring_[(head_-1) % kRing] = newest
    double cursor_ = -1.0;  // continuous index into the push sequence
    Telemetry telemetry_{};
};

}  // namespace rb_servo
