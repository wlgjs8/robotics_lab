#pragma once

// PER-ARM BARRIER STATUS (2026-09-15). Which arm the collision barrier is slowing or
// holding, against which pair, for how long -- as its own columns, state fields and
// console reports.
//
// WHY THIS EXISTS. A full scan of the 2026-09-10..15 servo logs (39 policy runs, 78 GB)
// found the barrier HOLDING arms at their floors near the stand -- arm<->arm pairs
// (left/right gripper at the 5 mm force-covered floor, left link5/6 against the right
// gripper / link3) and arm<->stand (link3 against stand_collision_*) -- on 24 % of the
// right arm's time above 150 mm and 18 % between 60 and 150 mm at stand-frame x < 0.45,
// against < 1 % farther out. None of it was answerable from the log as it stood:
//
//   - While a pair is held, the hold fold re-books the plan onto the held pose every
//     tick, so the per-tick correction stays at 0.7-1.9 deg/s. `self_collision_clamp_count`
//     and the SelfCollision verdict only count ticks above 2 deg/s ("blocked"), so 60-95 %
//     of held time was invisible to both. Measured: a 1.57 s hold in
//     servo_log_20260910_165651 (left link6 <-> right gripper) logged
//     safety_verdict=Ok, motion_state=Running and clamp_count +0 for its whole length,
//     while the policy's command over it was simply discarded (18-243 mm per episode).
//   - The projection columns name only the ONE tightest row of the whole solve, not the
//     row that is acting on a given arm.
//   - The policy step log carried nothing at all.
//
// Definitions (per arm, per tick, evaluated on the solver's own rows):
//   braking  a collision row that has this arm in its Jacobian was violated by the
//            REQUESTED motion (J.qdot_requested < -xi, i.e. the command closed faster
//            than the row allows) and this arm's command was actually corrected.
//   held     braking, and the tightest such row is at its floor: headroom
//            (d_now - d_hard) <= held_headroom_m. At or below the floor the row allows
//            no closing at all, so the arm makes no progress toward that pair.
// A stale collision verdict holds both arms outright (fail closed); that is reported
// as held with reason "stale_verdict" and no pair.
//
// These are DIAGNOSTICS. Nothing here feeds back into motion, so the thresholds below
// are reporting constants, not safety parameters.

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "rb_servo/control/collision_monitor.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo::control {

struct BarrierRowClassifyOptions {
    // "At its floor". The held episodes in the 09-10..15 logs sat at -0.1..0.0 mm; a row
    // 0.5 mm out still caps closing at sqrt(2*4.5*0.0005) = 67 mm/s, so anything looser
    // would start calling ordinary in-band braking a hold.
    double held_headroom_m = 0.0005;
    // Same epsilon applySafety uses for "this arm's command was modified".
    double correction_eps_deg_s = 1e-3;
};

struct ArmBarrierRows {
    bool braking = false;
    bool held = false;
    int violated_rows = 0;   // collision rows with this arm in J that the request violated
    // The row reported for this arm: the tightest VIOLATED row when braking, otherwise
    // the tightest engaged collision row that has this arm in its Jacobian. has_row
    // false = no collision row touched this arm this tick.
    bool has_row = false;
    double headroom_m = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t pair_key = 0;
    ConstraintClass klass = ConstraintClass::Other;
};

// Classify both arms from the rows handed to solveVelocityProjection. `*_requested_deg`
// is the target BEFORE the projection (plan_gate_requested in applySafety) and
// `*_correction_deg_s` the solver's per-arm correction. Rows of class Other (floor /
// ROI / reach points) are ignored: this is the collision barrier's status only.
std::array<ArmBarrierRows, 2> classifyBarrierRows(
    const std::vector<VelocityConstraint>& cons,
    const JointArray& left_prev_deg, const JointArray& right_prev_deg,
    const JointArray& left_requested_deg, const JointArray& right_requested_deg,
    double left_correction_deg_s, double right_correction_deg_s, double dt_sec,
    const BarrierRowClassifyOptions& opts = {});

struct BarrierEpisodeOptions {
    // The monitor is asynchronous and a held pair flickers across its floor for a tick
    // or two; gaps shorter than this do not split an episode.
    double bridge_s = 0.05;
    // Report a held episode when it ends if it lasted at least this long. 0.2 s is ten
    // policy steps at 30 Hz, well past anything a human watching would call a stutter.
    double report_held_min_s = 0.2;
    // Report a braking episode (in band, mostly not held) at least this long.
    double report_braking_min_s = 1.0;
    // While a held episode is still open, report it again every period.
    double ongoing_report_period_s = 2.0;
    // The threshold self_collision_clamp_count / the SelfCollision verdict count a tick
    // as "blocked" at (kReanchorDegPerSec in applySafety). Reported so the message says
    // how much of the hold those two signals saw.
    double blocked_counter_deg_s = 2.0;
};

struct BarrierTickInput {
    std::uint64_t now_ns = 0;
    double dt_s = 0.0;
    bool braking = false;
    bool held = false;
    std::string reason;     // "row" | "stale_verdict"
    std::string pair;       // "geom_a <-> geom_b", empty when unknown / no row
    std::string klass;      // arm_arm | arm_stand | environment | gripper_gripper | intra_arm | floor | external_box
    double headroom_m = std::numeric_limits<double>::quiet_NaN();
    double correction_deg_s = 0.0;  // this arm's applied correction this tick
    double folded_m = 0.0;          // plan shortfall booked by the hold fold this tick
};

struct BarrierEpisodeReport {
    enum class Kind { HeldEnded, HeldOngoing, BrakingEnded };
    Kind kind = Kind::HeldEnded;
    double duration_s = 0.0;
    double held_s = 0.0;                // held time inside the episode (== duration for held)
    std::string reason;
    std::string pair;                   // the pair at the episode's minimum headroom
    std::string klass;
    double min_headroom_m = std::numeric_limits<double>::quiet_NaN();
    double folded_m = 0.0;              // plan motion the hold fold discarded over the episode
    double max_correction_deg_s = 0.0;
    double counted_blocked_fraction = 0.0;  // ticks above blocked_counter_deg_s / episode ticks
    bool start_tcp_valid = false;
    std::array<double, 3> start_tcp_m{};
};

class BarrierEpisodeTracker {
public:
    BarrierEpisodeTracker() = default;
    explicit BarrierEpisodeTracker(BarrierEpisodeOptions opts);

    // Advance one tick. Returns the reports that became due this tick (usually none).
    std::vector<BarrierEpisodeReport> update(const BarrierTickInput& in);

    // True on the tick a held episode opened; the caller may then attach the TCP.
    bool heldEpisodeOpenedThisTick() const { return held_opened_this_tick_; }
    void setHeldEpisodeStartTcp(const std::array<double, 3>& tcp_m);

    // Length of the currently open held episode [s]; 0 when none is open.
    double heldEpisodeS() const;
    std::uint64_t heldCount() const { return held_count_; }
    double heldTotalS() const { return held_total_s_; }
    double brakingTotalS() const { return braking_total_s_; }
    double heldFoldedM() const { return held_folded_total_m_; }

    const BarrierEpisodeOptions& options() const { return opts_; }

private:
    struct Episode {
        bool open = false;
        std::uint64_t start_ns = 0;
        std::uint64_t last_true_ns = 0;
        double last_dt_s = 0.0;
        double held_s = 0.0;
        std::uint64_t ticks = 0;
        std::uint64_t blocked_ticks = 0;
        double min_headroom_m = std::numeric_limits<double>::quiet_NaN();
        std::string pair;
        std::string klass;
        std::string reason;
        double folded_m = 0.0;
        double max_correction_deg_s = 0.0;
        double next_ongoing_s = 0.0;
        bool start_tcp_valid = false;
        std::array<double, 3> start_tcp_m{};
    };
    static void accumulate(Episode& e, const BarrierTickInput& in, double blocked_deg_s);
    static double span(const Episode& e) {
        return static_cast<double>(e.last_true_ns - e.start_ns) * 1e-9 + e.last_dt_s;
    }
    BarrierEpisodeReport report(const Episode& e, BarrierEpisodeReport::Kind kind) const;

    BarrierEpisodeOptions opts_;
    Episode held_;
    Episode braking_;
    bool held_opened_this_tick_ = false;
    std::uint64_t held_count_ = 0;
    double held_total_s_ = 0.0;
    double braking_total_s_ = 0.0;
    double held_folded_total_m_ = 0.0;
};

// One console line for a report, e.g.
// "[WARN] barrier HELD left arm 1.57 s (ended): <pair> [arm_arm], min headroom -0.02 mm,
//  plan folded 21.3 mm, max correction 1.2 deg/s, seen by clamp_count on 0% of ticks,
//  TCP at start (0.388, -0.052, -0.187) m"
std::string formatBarrierReport(const std::string& arm, const BarrierEpisodeReport& r);

}  // namespace rb_servo::control
