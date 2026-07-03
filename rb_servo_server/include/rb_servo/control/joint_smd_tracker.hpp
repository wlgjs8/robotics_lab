#pragma once

// Joint-space Spring-Mass-Damper tracking filter for the JointTarget
// primitive — the joint-space mirror of SmdPoseTracker (TcpPoseTarget).
//
// The published joint target follows the commanded goal as a second-order
// system (mass fixed at 1.0) stepped at the servo rate, per joint:
//
//   ddq = wn^2 * (goal - q) - 2 * zeta * wn * dq
//
// Per-joint velocity/accel clamps act as the max-force saturation; inside the
// limits the zeta/fn dynamics are exact. With damping_ratio >= 1.0 and zero
// initial velocity the response has no overshoot, and identical zeta/fn across
// joints traces a straight line in joint space (synchronized-PTP path).

#include <algorithm>
#include <cmath>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

class JointSmdTracker {
public:
    explicit JointSmdTracker(const JointTargetSmdConfig& config) : config_(config) {}

    bool active() const { return active_; }
    const JointArray& position() const { return q_; }

    // Initialize the filter state at q with the given joint velocity (clamped
    // to the configured per-joint velocity limits — velocity continuity when a
    // streaming mode hands over mid-motion). The goal latches to q until
    // setGoal() moves it.
    void reset(const JointArray& q_deg, const JointArray& dq_deg_s) {
        for (int i = 0; i < kDof; ++i) {
            q_[i] = q_deg[i];
            const double dq = std::isfinite(dq_deg_s[i]) ? dq_deg_s[i] : 0.0;
            dq_[i] = std::clamp(dq, -config_.max_velocity_deg_s[i], config_.max_velocity_deg_s[i]);
        }
        goal_ = q_;
        active_ = true;
    }

    void deactivate() { active_ = false; clearArrivalStop(); }

    // Latest-wins goal: a new JointTarget packet just moves the goal, so both
    // one-shot PTP and streaming jog targets get the same smooth profile.
    void setGoal(const JointArray& goal_deg) { goal_ = goal_deg; }

    // Arrival-decel taper: latch the true final stop pose so step() eases the
    // per-step velocity into it as sqrt(2*arrival_decel*d). goal_ (the pursuit
    // carrot) leads the stop; the taper shapes the last stretch only. No-op when
    // config_.arrival_taper_enable is false. clearArrivalStop() reverts to the
    // pure SMD profile (streaming jog / plain PTP with no defined endpoint).
    void setArrivalStop(const JointArray& stop_deg) {
        arrival_stop_ = stop_deg;
        has_arrival_stop_ = true;
    }
    void clearArrivalStop() { has_arrival_stop_ = false; }

    // Advance the SMD state by dt toward the goal and return the smoothed
    // joint target. Requires active(); dt <= 0 returns the state unchanged.
    JointArray step(double dt_sec) {
        if (!(dt_sec > 0.0) || !std::isfinite(dt_sec)) {
            return q_;
        }
        const double wn = 2.0 * M_PI * config_.natural_frequency_hz;
        for (int i = 0; i < kDof; ++i) {
            double ddq = wn * wn * (goal_[i] - q_[i]) - 2.0 * config_.damping_ratio * wn * dq_[i];
            ddq = std::clamp(ddq, -config_.max_accel_deg_s2[i], config_.max_accel_deg_s2[i]);
            dq_[i] = std::clamp(
                dq_[i] + ddq * dt_sec,
                -config_.max_velocity_deg_s[i],
                config_.max_velocity_deg_s[i]
            );
        }
        // Arrival-decel taper: cap the dominant-joint speed at sqrt(2*a*d) and scale ALL
        // joints by the same factor so the straight-line joint path is preserved while the
        // last stretch eases in. d is the max per-joint distance to the final stop; far from
        // the stop the cap exceeds the SMD speed and the taper is inert (start/cruise
        // unchanged). The min-speed floor keeps the tail from crawling to waypoint_tol.
        if (has_arrival_stop_ && config_.arrival_taper_enable) {
            double d = 0.0;
            for (int i = 0; i < kDof; ++i) {
                d = std::max(d, std::abs(arrival_stop_[i] - q_[i]));
            }
            double v_cap = std::sqrt(2.0 * config_.arrival_decel_deg_s2 * std::max(d, 0.0));
            v_cap = std::max(v_cap, config_.arrival_min_speed_deg_s);
            double v_now = 0.0;
            for (int i = 0; i < kDof; ++i) {
                v_now = std::max(v_now, std::abs(dq_[i]));
            }
            if (v_now > v_cap && v_now > 0.0) {
                const double scale = v_cap / v_now;
                for (int i = 0; i < kDof; ++i) {
                    dq_[i] *= scale;
                }
            }
        }
        for (int i = 0; i < kDof; ++i) {
            q_[i] += dq_[i] * dt_sec;
        }
        return q_;
    }

private:
    JointTargetSmdConfig config_;
    bool active_ = false;
    JointArray q_{};
    JointArray dq_{};
    JointArray goal_{};
    JointArray arrival_stop_{};
    bool has_arrival_stop_ = false;
};

}  // namespace rb_servo
