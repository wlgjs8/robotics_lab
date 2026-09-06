// command_tracking_window.hpp -- "does the robot track its command?" with the
// control box's dead time taken into account.
//
// The measured TCP always lags the command that was SENT this tick: sent -> box
// reference is a pure dead time of ~8 ticks and the whole sent -> actual chain
// measures ~22 ms on the RB5 boxes (see AGENTS.md / the 2026-08 delay study). A
// naive |sent_now - actual_now| therefore grows linearly with speed (1.4 rad/s x
// 22 ms = 1.8 deg) and would report a perfectly tracking arm as "not tracking"
// exactly when it moves fast -- which is when the question matters.
//
// So the test is: does the measured pose match ANY command sent within the last
// `capacity` ticks? A tracking arm always matches the one sent one dead time ago.
// A pinned arm matches none once the command has moved on by more than the
// tolerance, regardless of the window (every entry is further away than the
// newest one it stopped at... or older ones that are even further back along the
// path), so the window cannot excuse a genuine tracking failure for longer than
// the window itself (50 ms at 500 Hz).
//
// The match is scored jointly: score = max(pos / pos_tol, ang / ang_tol), the
// entry with the smallest score is reported, and tracking == (score <= 1).
#pragma once

#include "rb_servo/core/types.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>

namespace rb_servo::control {

struct CommandTrackingMatch {
    bool within = false;         // some recent command is inside both tolerances
    double pos_m = 0.0;          // distances to the best-scoring recent command
    double ang_rad = 0.0;
    int lag_ticks = -1;          // how many ticks ago that command was recorded (-1: empty)
    double score = std::numeric_limits<double>::infinity();
};

template <std::size_t Capacity>
class CommandTrackingWindow {
public:
    static_assert(Capacity > 0, "window needs at least one slot");
    static constexpr std::size_t kCapacity = Capacity;

    void clear() { size_ = 0; head_ = 0; }
    std::size_t size() const { return size_; }

    // Record the command that was sent this tick (newest entry, lag 0).
    void record(const Pose6D& sent) {
        ring_[head_] = sent;
        head_ = (head_ + 1) % Capacity;
        size_ = std::min(size_ + 1, Capacity);
    }

    CommandTrackingMatch match(const Pose6D& actual, double pos_tol_m, double ang_tol_rad) const {
        CommandTrackingMatch best;
        if (size_ == 0) return best;
        const double pos_scale = pos_tol_m > 0.0 ? 1.0 / pos_tol_m : std::numeric_limits<double>::infinity();
        const double ang_scale = ang_tol_rad > 0.0 ? 1.0 / ang_tol_rad : std::numeric_limits<double>::infinity();
        for (std::size_t lag = 0; lag < size_; ++lag) {
            const std::size_t idx = (head_ + Capacity - 1 - lag) % Capacity;
            const double pos = math::positionDistance(ring_[idx], actual);
            const double ang = math::orientationDistanceRad(ring_[idx], actual);
            // A zero tolerance can only be met exactly (fail-closed on 0 * inf).
            const double ps = pos > 0.0 ? pos * pos_scale : 0.0;
            const double as = ang > 0.0 ? ang * ang_scale : 0.0;
            const double score = std::max(ps, as);
            if (score < best.score) {
                best.score = score;
                best.pos_m = pos;
                best.ang_rad = ang;
                best.lag_ticks = static_cast<int>(lag);
            }
        }
        best.within = best.score <= 1.0;
        return best;
    }

private:
    std::array<Pose6D, Capacity> ring_{};
    std::size_t head_ = 0;
    std::size_t size_ = 0;
};

}  // namespace rb_servo::control
