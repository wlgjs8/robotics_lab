#pragma once

// Ring-down detector for the post-Init-Motion F/T software zero (2026-08-19).
//
// The tare used to wait a FIXED auto_tare_settle_sec (3 s, sized for the worst
// dual-arm Init Motion ring-down) before collecting its window, so the arm sat
// unarmed for settle+collect (~4 s) after every Init Motion — long enough that
// operators started rollouts before arming, and the acceptance then landed
// mid-motion. This detector watches the rolling per-axis standard deviation of
// the pre-tare external wrench over the last N fresh samples: once every axis is
// inside the same stddev limits the tare window itself has to meet, the
// structure has stopped ringing and collection can start immediately. The fixed
// wait is kept only as the maximum. Header-only and stateless apart from the
// ring buffer so it is unit-testable in isolation (test_ft_wrench_pipeline).

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace rb_servo {

class TareSettleDetector {
public:
    static constexpr int kMaxWindow = 512;

    void configure(int window_samples) {
        window_ = std::clamp(window_samples, 2, kMaxWindow);
        reset();
    }

    void reset() {
        head_ = 0;
        count_ = 0;
        sum_.fill(0.0);
        sum_squares_.fill(0.0);
    }

    int window() const { return window_; }
    int count() const { return count_; }
    bool full() const { return count_ >= window_; }

    // Push one fresh 6-axis sample (fx, fy, fz, tx, ty, tz). Non-finite samples
    // reset the detector: a window that spans a sensor glitch must not vouch for
    // quiescence.
    void push(const std::array<double, 6>& sample) {
        for (double v : sample) {
            if (!std::isfinite(v)) {
                reset();
                return;
            }
        }
        if (count_ >= window_) {
            const std::array<double, 6>& old = ring_[static_cast<std::size_t>(head_)];
            for (std::size_t i = 0; i < 6; ++i) {
                sum_[i] -= old[i];
                sum_squares_[i] -= old[i] * old[i];
            }
        } else {
            ++count_;
        }
        ring_[static_cast<std::size_t>(head_)] = sample;
        for (std::size_t i = 0; i < 6; ++i) {
            sum_[i] += sample[i];
            sum_squares_[i] += sample[i] * sample[i];
        }
        head_ = (head_ + 1) % window_;
    }

    // Per-axis stddev over the current window (NaN until the window is full).
    std::array<double, 6> stddev() const {
        std::array<double, 6> out{};
        if (!full()) {
            out.fill(std::numeric_limits<double>::quiet_NaN());
            return out;
        }
        const double n = static_cast<double>(count_);
        for (std::size_t i = 0; i < 6; ++i) {
            const double mean = sum_[i] / n;
            const double var = std::max(0.0, sum_squares_[i] / n - mean * mean);
            out[i] = std::sqrt(var);
        }
        return out;
    }

    // Largest force-axis and torque-axis stddev (NaN until full) — for telemetry.
    double maxForceStddev() const {
        const auto s = stddev();
        return std::max({s[0], s[1], s[2]});
    }
    double maxTorqueStddev() const {
        const auto s = stddev();
        return std::max({s[3], s[4], s[5]});
    }

    // True once the window is full and every axis is inside its limit.
    bool quiet(double max_force_stddev, double max_torque_stddev) const {
        if (!full()) return false;
        const auto s = stddev();
        for (std::size_t i = 0; i < 3; ++i) {
            if (!(s[i] <= max_force_stddev)) return false;
        }
        for (std::size_t i = 3; i < 6; ++i) {
            if (!(s[i] <= max_torque_stddev)) return false;
        }
        return true;
    }

private:
    int window_ = 150;
    int head_ = 0;
    int count_ = 0;
    std::array<std::array<double, 6>, kMaxWindow> ring_{};
    std::array<double, 6> sum_{};
    std::array<double, 6> sum_squares_{};
};

}  // namespace rb_servo
