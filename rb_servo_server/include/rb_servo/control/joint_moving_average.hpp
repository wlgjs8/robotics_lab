#pragma once

#include "rb_servo/core/types.hpp"

#include <cstddef>
#include <deque>

namespace rb_servo {

// Moving average over the last N joint targets, applied as the FINAL output
// stage (after the safety filter, before send/prev-sent bookkeeping). Each
// output is a convex combination of safety-passed targets, so joint limits
// and per-tick velocity bounds are preserved by construction. At 500 Hz a
// window of 40 is an 80 ms boxcar (~40 ms group delay). window <= 1 is a
// pass-through. The first sample fills the whole window (no warm-up ramp);
// the buffer is refreshed every tick, including hold/fault ticks, so it never
// carries stale history across pauses.
class JointMovingAverage {
public:
    explicit JointMovingAverage(int window) : window_(window) {}

    JointArray apply(const JointArray& value) {
        if (window_ <= 1) return value;
        if (samples_.empty()) {
            for (int i = 0; i < window_; ++i) push(value);
        } else {
            push(value);
        }
        JointArray out{};
        const double n = static_cast<double>(samples_.size());
        for (int j = 0; j < kDof; ++j) out[j] = sum_[j] / n;
        return out;
    }

    void reset() {
        samples_.clear();
        sum_ = JointArray{};
    }

private:
    void push(const JointArray& value) {
        samples_.push_back(value);
        for (int j = 0; j < kDof; ++j) sum_[j] += value[j];
        while (samples_.size() > static_cast<std::size_t>(window_)) {
            for (int j = 0; j < kDof; ++j) sum_[j] -= samples_.front()[j];
            samples_.pop_front();
        }
    }

    int window_ = 0;
    std::deque<JointArray> samples_;
    JointArray sum_{};
};

}  // namespace rb_servo
