#include "rb_servo/control/barrier_status.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace rb_servo::control {

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
// A row whose Jacobian has (numerically) nothing in this arm's six columns does not
// act on this arm, whatever the other arm is doing.
constexpr double kArmJacobianEps2 = 1e-18;

}  // namespace

std::array<ArmBarrierRows, 2> classifyBarrierRows(
    const std::vector<VelocityConstraint>& cons,
    const JointArray& left_prev_deg, const JointArray& right_prev_deg,
    const JointArray& left_requested_deg, const JointArray& right_requested_deg,
    double left_correction_deg_s, double right_correction_deg_s, double dt_sec,
    const BarrierRowClassifyOptions& opts) {
    std::array<ArmBarrierRows, 2> out{};
    if (!(dt_sec > 0.0) || cons.empty()) return out;

    Eigen::Matrix<double, 2 * kDof, 1> qdot_req;
    for (int i = 0; i < kDof; ++i) {
        qdot_req[i] = (left_requested_deg[i] - left_prev_deg[i]) * kDeg2Rad / dt_sec;
        qdot_req[kDof + i] = (right_requested_deg[i] - right_prev_deg[i]) * kDeg2Rad / dt_sec;
    }
    const double correction[2] = {left_correction_deg_s, right_correction_deg_s};

    for (int arm = 0; arm < 2; ++arm) {
        ArmBarrierRows& a = out[static_cast<std::size_t>(arm)];
        double tightest_engaged = std::numeric_limits<double>::infinity();
        double tightest_violated = std::numeric_limits<double>::infinity();
        const VelocityConstraint* engaged_row = nullptr;
        const VelocityConstraint* violated_row = nullptr;
        for (const VelocityConstraint& c : cons) {
            if (c.klass == ConstraintClass::Other) continue;
            const double j_arm2 = arm == 0 ? c.J.head(kDof).squaredNorm()
                                           : c.J.tail(kDof).squaredNorm();
            if (j_arm2 < kArmJacobianEps2) continue;
            const double headroom = c.d_now - c.d_hard;
            if (headroom < tightest_engaged) {
                tightest_engaged = headroom;
                engaged_row = &c;
            }
            // The request closed faster than this row allows.
            if (c.J.dot(qdot_req) < -c.xi) {
                ++a.violated_rows;
                if (headroom < tightest_violated) {
                    tightest_violated = headroom;
                    violated_row = &c;
                }
            }
        }
        a.braking = violated_row != nullptr && correction[arm] > opts.correction_eps_deg_s;
        const VelocityConstraint* shown = a.braking ? violated_row : engaged_row;
        if (shown != nullptr) {
            a.has_row = true;
            a.headroom_m = shown->d_now - shown->d_hard;
            a.pair_key = shown->pair_key;
            a.klass = shown->klass;
        }
        a.held = a.braking && a.headroom_m <= opts.held_headroom_m;
    }
    return out;
}

BarrierEpisodeTracker::BarrierEpisodeTracker(BarrierEpisodeOptions opts) : opts_(opts) {}

void BarrierEpisodeTracker::accumulate(Episode& e, const BarrierTickInput& in,
                                       double blocked_deg_s) {
    e.last_true_ns = in.now_ns;
    e.last_dt_s = in.dt_s;
    ++e.ticks;
    if (in.held) e.held_s += in.dt_s;
    if (in.correction_deg_s > blocked_deg_s) ++e.blocked_ticks;
    e.folded_m += in.folded_m;
    e.max_correction_deg_s = std::max(e.max_correction_deg_s, in.correction_deg_s);
    if (std::isfinite(in.headroom_m) &&
        (!std::isfinite(e.min_headroom_m) || in.headroom_m < e.min_headroom_m)) {
        e.min_headroom_m = in.headroom_m;
        if (!in.pair.empty()) e.pair = in.pair;
        if (!in.klass.empty()) e.klass = in.klass;
    }
    if (e.pair.empty() && !in.pair.empty()) {
        e.pair = in.pair;
        e.klass = in.klass;
    }
    if (e.reason.empty() || in.reason == "stale_verdict") e.reason = in.reason;
}

BarrierEpisodeReport BarrierEpisodeTracker::report(const Episode& e,
                                                   BarrierEpisodeReport::Kind kind) const {
    BarrierEpisodeReport r;
    r.kind = kind;
    r.duration_s = span(e);
    r.held_s = e.held_s;
    r.reason = e.reason;
    r.pair = e.pair;
    r.klass = e.klass;
    r.min_headroom_m = e.min_headroom_m;
    r.folded_m = e.folded_m;
    r.max_correction_deg_s = e.max_correction_deg_s;
    r.counted_blocked_fraction =
        e.ticks > 0 ? static_cast<double>(e.blocked_ticks) / static_cast<double>(e.ticks) : 0.0;
    r.start_tcp_valid = e.start_tcp_valid;
    r.start_tcp_m = e.start_tcp_m;
    return r;
}

std::vector<BarrierEpisodeReport> BarrierEpisodeTracker::update(const BarrierTickInput& in) {
    std::vector<BarrierEpisodeReport> reports;
    held_opened_this_tick_ = false;
    const auto gap_s = [&](const Episode& e) {
        return in.now_ns > e.last_true_ns
            ? static_cast<double>(in.now_ns - e.last_true_ns) * 1e-9 : 0.0;
    };

    // Cumulative totals count the flagged ticks themselves, never the bridged gaps.
    if (in.held) {
        held_total_s_ += in.dt_s;
        held_folded_total_m_ += in.folded_m;
    }
    if (in.braking || in.held) braking_total_s_ += in.dt_s;

    // ---- held episodes ----
    if (in.held) {
        if (!held_.open) {
            held_ = Episode{};
            held_.open = true;
            held_.start_ns = in.now_ns;
            held_.next_ongoing_s = opts_.ongoing_report_period_s;
            ++held_count_;
            held_opened_this_tick_ = true;
        }
        accumulate(held_, in, opts_.blocked_counter_deg_s);
        if (opts_.ongoing_report_period_s > 0.0 && span(held_) >= held_.next_ongoing_s) {
            reports.push_back(report(held_, BarrierEpisodeReport::Kind::HeldOngoing));
            held_.next_ongoing_s += opts_.ongoing_report_period_s;
        }
    } else if (held_.open && gap_s(held_) > opts_.bridge_s) {
        held_.open = false;
        if (span(held_) >= opts_.report_held_min_s) {
            reports.push_back(report(held_, BarrierEpisodeReport::Kind::HeldEnded));
        }
    }

    // ---- braking episodes (held ticks are braking ticks too) ----
    const bool braking = in.braking || in.held;
    if (braking) {
        if (!braking_.open) {
            braking_ = Episode{};
            braking_.open = true;
            braking_.start_ns = in.now_ns;
        }
        accumulate(braking_, in, opts_.blocked_counter_deg_s);
    } else if (braking_.open && gap_s(braking_) > opts_.bridge_s) {
        braking_.open = false;
        const double d = span(braking_);
        // A mostly-held episode is already reported as held; this one is for long
        // in-band braking that never reached the floor.
        if (d >= opts_.report_braking_min_s && braking_.held_s < 0.5 * d) {
            reports.push_back(report(braking_, BarrierEpisodeReport::Kind::BrakingEnded));
        }
    }
    return reports;
}

void BarrierEpisodeTracker::setHeldEpisodeStartTcp(const std::array<double, 3>& tcp_m) {
    if (!held_.open) return;
    held_.start_tcp_valid = true;
    held_.start_tcp_m = tcp_m;
}

double BarrierEpisodeTracker::heldEpisodeS() const {
    return held_.open ? span(held_) : 0.0;
}

std::string formatBarrierReport(const std::string& arm, const BarrierEpisodeReport& r) {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    const char* what = r.kind == BarrierEpisodeReport::Kind::BrakingEnded ? "BRAKING" : "HELD";
    os << "[WARN] barrier " << what << ' ' << arm << " arm ";
    os.precision(2);
    os << r.duration_s << " s ("
       << (r.kind == BarrierEpisodeReport::Kind::HeldOngoing ? "ongoing" : "ended") << "): ";
    if (r.reason == "stale_verdict") {
        os << "stale collision verdict (both arms held, no pair)";
    } else {
        os << (r.pair.empty() ? std::string("<unnamed pair>") : r.pair) << " ["
           << (r.klass.empty() ? std::string("?") : r.klass) << "]";
    }
    if (std::isfinite(r.min_headroom_m)) os << ", min headroom " << r.min_headroom_m * 1000.0 << " mm";
    if (r.kind == BarrierEpisodeReport::Kind::BrakingEnded) {
        os << ", held " << r.held_s << " s of it";
    }
    os.precision(1);
    os << ", plan folded " << r.folded_m * 1000.0 << " mm";
    os << ", max correction " << r.max_correction_deg_s << " deg/s";
    os.precision(0);
    os << ", seen by self_collision_clamp_count on " << r.counted_blocked_fraction * 100.0
       << "% of ticks";
    if (r.start_tcp_valid) {
        os.precision(3);
        os << ", TCP at start (" << r.start_tcp_m[0] << ", " << r.start_tcp_m[1] << ", "
           << r.start_tcp_m[2] << ") m";
    }
    return os.str();
}

}  // namespace rb_servo::control
