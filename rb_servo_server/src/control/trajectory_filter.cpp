#include "rb_servo/control/trajectory_filter.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>

namespace rb_servo {
namespace {

bool nearlyEqualJointArray(const JointArray& a, const JointArray& b, double tol_deg) {
    for (int i = 0; i < kDof; ++i) {
        if (!(std::abs(a[i] - b[i]) <= tol_deg)) {
            return false;
        }
    }
    return true;
}

bool finiteJointArray(const JointArray& joints) {
    return std::all_of(joints.begin(), joints.end(), [](double value) {
        return std::isfinite(value);
    });
}

// Choose the shortest-path representation of an absolute JointTarget goal.
//
// The robot operates on RAW continuous joint angles in [q_min_deg, q_max_deg]
// (= [-360, 360] for the supported rbpodo scope, see docs/joint_range_policy.md),
// so a single physical orientation has up to three valid raw representations
// (theta, theta+/-360). A bare absolute target can therefore sit a full
// revolution away from where the joint currently is (e.g. q=251.8 deg, target=-131.66
// deg => -383 deg "the long way"), making an InitMotion/PTP spin ~360 deg to reach
// an equivalent pose. This is the "continuous motion-safe unwrapping" the joint
// range policy deferred: pick, per joint, the equivalent angle (target + 360*k)
// that is in-range AND closest to the reference (the current sent target). It is
// limit-bounded (never returns an out-of-range angle) and endpoint-equivalent (same
// physical pose mod 360), so when no closer in-range equivalent exists it returns the
// raw target unchanged (e.g. a near-limit target keeps the only legal long way around).
// This selects the GOAL only; the downstream ramp/SMD is still monotonic from the
// reference (no mid-trajectory period wrapping, which stays rejected).
// Axes marked in safety.joint_target_literal_axes deliberately opt out and keep
// the raw command target; this protects cable-sensitive wrist yaw returns.
JointArray shortestPathJointGoal(
    const JointArray& raw_target,
    const JointArray& reference,
    const JointArray& q_min_deg,
    const JointArray& q_max_deg,
    const JointBoolArray& literal_axes
) {
    JointArray out = raw_target;
    for (int i = 0; i < kDof; ++i) {
        if (literal_axes[static_cast<std::size_t>(i)]) {
            continue;
        }
        if (!(std::isfinite(raw_target[i]) && std::isfinite(reference[i]))) {
            continue;
        }
        // Only normalize when a valid in-range window exists for this joint; an
        // unset/degenerate [q_min, q_max] (default zero arrays) disables it safely.
        if (!(q_max_deg[i] > q_min_deg[i])) {
            continue;
        }
        double best = raw_target[i];
        double best_dist = std::abs(raw_target[i] - reference[i]);
        for (int k = -3; k <= 3; ++k) {
            if (k == 0) {
                continue;
            }
            const double cand = raw_target[i] + 360.0 * static_cast<double>(k);
            if (cand < q_min_deg[i] || cand > q_max_deg[i]) {
                continue;
            }
            const double dist = std::abs(cand - reference[i]);
            if (dist < best_dist) {
                best_dist = dist;
                best = cand;
            }
        }
        out[i] = best;
    }
    return out;
}

}  // namespace

TrajectoryFilter::TrajectoryFilter(const ServoConfig& servo_config, const SafetyConfig& safety_config)
    : servo_config_(servo_config),
      safety_config_(safety_config),
      joint_smd_(safety_config.joint_target_smd) {}

JointArray TrajectoryFilter::computeJointTarget(
    const ArmCommand& command,
    const RobotState& state,
    const JointArray& previous_sent_target,
    double dt_sec
) {
    JointArray out{};
    switch (command.mode) {
        case ControlMode::Hold:
            joint_smd_.deactivate();
            out = command.has_joint_target && finiteJointArray(command.q_target_deg)
                ? command.q_target_deg
                : holdTarget(previous_sent_target);
            break;
        case ControlMode::Idle:
        case ControlMode::ArmMotion:
        case ControlMode::DisarmMotion:
        case ControlMode::EmergencyStop:
        case ControlMode::ResetFault:
        case ControlMode::SetSafetyFloorZ:
        case ControlMode::SetSafetyRoiBounds:
        case ControlMode::SetExternalBoxes:
            joint_smd_.deactivate();
            out = holdTarget(previous_sent_target);
            break;
        case ControlMode::JointTarget: {
            // Reach the commanded joint pose by the shortest in-range path unless an
            // axis is configured literal for cable safety (docs/joint_range_policy.md).
            const JointArray goal = shortestPathJointGoal(
                command.q_target_deg, previous_sent_target,
                safety_config_.q_min_deg, safety_config_.q_max_deg,
                safety_config_.joint_target_literal_axes);
            out = safety_config_.joint_target_smd.enable
                ? smdJointTarget(goal, previous_sent_target, dt_sec)
                : filterJointTarget(goal, previous_sent_target, dt_sec);
            break;
        }
        case ControlMode::TcpPoseTarget:
        case ControlMode::TcpLinearMove:
            // Cartesian modes are intentionally deferred.
            joint_smd_.deactivate();
            out = holdTarget(previous_sent_target);
            break;
        default:
            joint_smd_.deactivate();
            out = holdTarget(previous_sent_target);
            break;
    }
    last_previous_sent_target_ = previous_sent_target;
    has_last_previous_sent_target_ = true;
    return out;
}

JointArray TrajectoryFilter::holdTarget(const JointArray& previous_sent_target) const {
    return previous_sent_target;
}

JointArray TrajectoryFilter::filterJointTarget(
    const JointArray& raw_target,
    const JointArray& previous_sent_target,
    double dt_sec
) const {
    JointArray out = previous_sent_target;
    for (int i = 0; i < kDof; ++i) {
        const double max_step = safety_config_.dq_max_deg_s[i] * dt_sec;
        const double delta = std::clamp(raw_target[i] - previous_sent_target[i], -max_step, max_step);
        out[i] = previous_sent_target[i] + delta;
    }
    return out;
}

JointArray TrajectoryFilter::smdJointTarget(
    const JointArray& raw_target,
    const JointArray& previous_sent_target,
    double dt_sec
) {
    // (Re)activate when the profile is fresh OR when somebody else moved the
    // sent target since our last step (e.g. the Cartesian path ran in between
    // — those modes bypass this filter entirely, so deactivate() may not have
    // been called). The internal state must always continue exactly from the
    // caller's previous_sent_target to stay jump-free.
    constexpr double kContinuityTolDeg = 1e-6;
    // How far the SMD's last output drifted from what was actually sent (safety/output-MA
    // can modify it). A non-trivial mismatch here forces a reset below, which defeats the
    // SMD's smoothing — instrumented to diagnose init-motion jerk.
    const bool was_active = joint_smd_.active();
    double continuity_mismatch_deg = 0.0;
    if (was_active) {
        const JointArray pos = joint_smd_.position();
        for (int i = 0; i < kDof; ++i) {
            continuity_mismatch_deg =
                std::max(continuity_mismatch_deg, std::abs(pos[i] - previous_sent_target[i]));
        }
    }
    const bool reset =
        !was_active ||
        !nearlyEqualJointArray(joint_smd_.position(), previous_sent_target, kContinuityTolDeg);
    if (reset) {
        JointArray dq0{};
        if (has_last_previous_sent_target_ && dt_sec > 0.0 && std::isfinite(dt_sec)) {
            for (int i = 0; i < kDof; ++i) {
                dq0[i] = (previous_sent_target[i] - last_previous_sent_target_[i]) / dt_sec;
            }
        }
        joint_smd_.reset(previous_sent_target, dq0);
    }
    joint_smd_.setGoal(raw_target);
    const JointArray out = joint_smd_.step(dt_sec);
    // DIAGNOSTIC (throttled ~1 Hz, shared across both arms): is the joint-target SMD
    // persisting tick-to-tick, or being reset every tick? A reset WHILE it was already
    // active means safety/output-MA changed the SMD output before it became prev_sent, so
    // the continuity check (==prev_sent) fails and the SMD re-seeds from scratch each tick
    // — that defeats its accel/jerk smoothing and lets per-tick goal jitter pass straight
    // through (the init-motion jerk hypothesis). High active-resets/sec == defeated.
    {
        static std::atomic<long long> calls{0}, active_resets{0}, last_ns{0}, max_mismatch_micro{0};
        calls.fetch_add(1, std::memory_order_relaxed);
        if (reset && was_active) active_resets.fetch_add(1, std::memory_order_relaxed);
        const long long mm = static_cast<long long>(continuity_mismatch_deg * 1e6);
        long long cur = max_mismatch_micro.load(std::memory_order_relaxed);
        while (mm > cur &&
               !max_mismatch_micro.compare_exchange_weak(cur, mm, std::memory_order_relaxed)) {}
        const long long now = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        long long prev = last_ns.load(std::memory_order_relaxed);
        if ((prev == 0 || now - prev > 1000000000LL) &&
            last_ns.compare_exchange_strong(prev, now, std::memory_order_relaxed)) {
            const long long c = calls.exchange(0, std::memory_order_relaxed);
            const long long r = active_resets.exchange(0, std::memory_order_relaxed);
            const long long mmx = max_mismatch_micro.exchange(0, std::memory_order_relaxed);
            std::cerr << "[INFO] joint_target SMD: " << r << "/" << c
                      << " active-continuity-resets/sec (high == smoothing defeated -> jerk), "
                      << "max_mismatch=" << (static_cast<double>(mmx) * 1e-6) << " deg\n";
        }
    }
    return out;
}

}  // namespace rb_servo
