// test_barrier_status.cpp — per-arm collision-barrier status and its episodes.
//
// The case this exists for (see include/rb_servo/control/barrier_status.hpp): a pair
// HELD at its floor with the hold fold re-booking the plan every tick keeps the
// per-tick correction around 1 deg/s, below the 2 deg/s bar that
// self_collision_clamp_count and the SelfCollision verdict use — so 60-95% of held
// time was invisible in the log. `held` must fire anyway, and the episode must report
// how little of itself those two signals saw.

#include "rb_servo/control/barrier_status.hpp"

#include <cmath>
#include <cstdio>
#include <string>

using rb_servo::ConstraintClass;
using rb_servo::JointArray;
using rb_servo::VelocityConstraint;
using rb_servo::kDof;
using rb_servo::control::BarrierEpisodeOptions;
using rb_servo::control::BarrierEpisodeReport;
using rb_servo::control::BarrierEpisodeTracker;
using rb_servo::control::BarrierTickInput;
using rb_servo::control::classifyBarrierRows;
using rb_servo::control::formatBarrierReport;

static int g_failures = 0;
static void check(bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

constexpr double kDt = 0.002;  // 500 Hz

static JointArray zeros() { return JointArray{0, 0, 0, 0, 0, 0}; }

// A request that drives left J1 (or right J1) by `deg` this tick.
static JointArray step(double deg) {
    JointArray q = zeros();
    q[0] = deg;
    return q;
}

// One collision row: J[-1] on the chosen arm's J1, so a POSITIVE joint velocity closes
// the pair. xi is the allowance the barrier grants (m/s).
static VelocityConstraint row(bool left_arm, double d_now, double d_hard, double xi,
                              ConstraintClass klass = ConstraintClass::Self) {
    VelocityConstraint c;
    c.J[left_arm ? 0 : kDof] = -1.0;
    c.xi = xi;
    c.d_now = d_now;
    c.d_hard = d_hard;
    c.klass = klass;
    c.pair_key = 0x1234;
    return c;
}

static bool inBandButRequestWithinAllowanceIsNotBraking() {
    // 40 mm out, allowance 0.5 m/s, request closes at ~0.087 m/s: the row exists and is
    // reported (so the operator can see what is near) but nothing is being removed.
    const std::vector<VelocityConstraint> cons{row(true, 0.060, 0.020, 0.5)};
    const auto s = classifyBarrierRows(cons, zeros(), zeros(), step(0.01), zeros(),
                                       0.0, 0.0, kDt);
    return !s[0].braking && !s[0].held && s[0].has_row &&
           std::abs(s[0].headroom_m - 0.040) < 1e-9;
}

static bool violatedRowWithCorrectionIsBraking() {
    const std::vector<VelocityConstraint> cons{row(true, 0.030, 0.020, 0.1)};
    const auto s = classifyBarrierRows(cons, zeros(), zeros(), step(1.0), zeros(),
                                       5.0, 0.0, kDt);
    return s[0].braking && !s[0].held && s[0].violated_rows == 1 &&
           std::abs(s[0].headroom_m - 0.010) < 1e-9;
}

static bool rowAtItsFloorIsHeldEvenAtOneDegPerSecond() {
    // THE CASE. Headroom 0.0 mm, allowance 0 (nothing may close), and the correction is
    // 1.0 deg/s — half the 2 deg/s bar clamp_count uses.
    const std::vector<VelocityConstraint> cons{row(true, 0.020, 0.020, 0.0)};
    const auto s = classifyBarrierRows(cons, zeros(), zeros(), step(1.0), zeros(),
                                       1.0, 0.0, kDt);
    return s[0].braking && s[0].held;
}

static bool correctionBelowEpsilonIsNotBraking() {
    const std::vector<VelocityConstraint> cons{row(true, 0.020, 0.020, 0.0)};
    const auto s = classifyBarrierRows(cons, zeros(), zeros(), step(1.0), zeros(),
                                       0.0, 0.0, kDt);
    return !s[0].braking && !s[0].held && s[0].has_row;
}

static bool theOtherArmsRowDoesNotBlameThisArm() {
    // A row that only carries the right arm's joints, with both corrections nonzero.
    const std::vector<VelocityConstraint> cons{row(false, 0.020, 0.020, 0.0)};
    const auto s = classifyBarrierRows(cons, zeros(), zeros(), zeros(), step(1.0),
                                       5.0, 5.0, kDt);
    return !s[0].braking && !s[0].has_row && s[1].braking && s[1].held;
}

static bool floorAndRoiRowsAreNotTheCollisionBarrier() {
    // ConstraintClass::Other = floor plane / ROI face / reach shell points. They share
    // the solve but are not what this status reports.
    const std::vector<VelocityConstraint> cons{
        row(true, 0.000, 0.000, 0.0, ConstraintClass::Other)};
    const auto s = classifyBarrierRows(cons, zeros(), zeros(), step(1.0), zeros(),
                                       5.0, 0.0, kDt);
    return !s[0].braking && !s[0].has_row;
}

static bool tightestViolatedRowIsTheOneReported() {
    std::vector<VelocityConstraint> cons{row(true, 0.050, 0.020, 0.0),
                                         row(true, 0.021, 0.020, 0.0)};
    cons[1].pair_key = 0x5678;
    const auto s = classifyBarrierRows(cons, zeros(), zeros(), step(1.0), zeros(),
                                       5.0, 0.0, kDt);
    return s[0].braking && s[0].violated_rows == 2 && s[0].pair_key == 0x5678 &&
           std::abs(s[0].headroom_m - 0.001) < 1e-9;
}

static BarrierTickInput heldTick(std::uint64_t now_ns, double correction_deg_s = 1.0,
                                 double folded_m = 0.0005) {
    BarrierTickInput in;
    in.now_ns = now_ns;
    in.dt_s = kDt;
    in.braking = true;
    in.held = true;
    in.reason = "row";
    in.pair = "left_link6_0 <-> right_pika_gripper_base";
    in.klass = "arm_arm";
    in.headroom_m = -0.00002;
    in.correction_deg_s = correction_deg_s;
    in.folded_m = folded_m;
    return in;
}

static bool heldEpisodeIsReportedWithTheBlindSpotQuantified() {
    BarrierEpisodeTracker tracker;
    std::uint64_t t = 0;
    std::vector<BarrierEpisodeReport> reports;
    for (int i = 0; i < 500; ++i) {  // 1.0 s held at 1 deg/s
        reports = tracker.update(heldTick(t));
        t += 2'000'000;
    }
    if (!reports.empty()) return false;  // nothing reported while it is still open
    if (tracker.heldCount() != 1) return false;
    if (std::abs(tracker.heldEpisodeS() - 1.0) > 0.01) return false;
    BarrierTickInput idle;
    idle.dt_s = kDt;
    // The episode closes on the first tick whose gap exceeds bridge_s, so collect.
    std::vector<BarrierEpisodeReport> ended;
    for (int i = 0; i < 40; ++i) {
        idle.now_ns = t;
        for (const BarrierEpisodeReport& r : tracker.update(idle)) ended.push_back(r);
        t += 2'000'000;
    }
    if (ended.size() != 1) return false;
    const BarrierEpisodeReport& r = ended.front();
    return r.kind == BarrierEpisodeReport::Kind::HeldEnded &&
           std::abs(r.duration_s - 1.0) < 0.02 &&
           r.counted_blocked_fraction == 0.0 &&   // the 2 deg/s counter saw NONE of it
           std::abs(r.folded_m - 0.25) < 1e-6 &&  // 500 ticks x 0.5 mm of discarded plan
           r.pair == "left_link6_0 <-> right_pika_gripper_base" && r.klass == "arm_arm" &&
           std::abs(tracker.heldTotalS() - 1.0) < 0.01;
}

static bool aOneTickFlickerDoesNotSplitAnEpisode() {
    BarrierEpisodeTracker tracker;
    std::uint64_t t = 0;
    for (int i = 0; i < 100; ++i) {
        tracker.update(heldTick(t));
        t += 2'000'000;
    }
    BarrierTickInput idle;
    idle.now_ns = t;
    idle.dt_s = kDt;
    tracker.update(idle);  // one tick of nothing, well inside bridge_s
    t += 2'000'000;
    for (int i = 0; i < 100; ++i) {
        tracker.update(heldTick(t));
        t += 2'000'000;
    }
    return tracker.heldCount() == 1 && tracker.heldEpisodeS() > 0.39;
}

static bool shortHoldsAreNotReportedButStillCounted() {
    BarrierEpisodeTracker tracker;
    std::uint64_t t = 0;
    for (int i = 0; i < 20; ++i) {  // 40 ms, under report_held_min_s
        tracker.update(heldTick(t));
        t += 2'000'000;
    }
    BarrierTickInput idle;
    idle.dt_s = kDt;
    std::vector<BarrierEpisodeReport> ended;
    for (int i = 0; i < 40; ++i) {
        idle.now_ns = t;
        for (const BarrierEpisodeReport& r : tracker.update(idle)) ended.push_back(r);
        t += 2'000'000;
    }
    return ended.empty() && tracker.heldCount() == 1 &&
           std::abs(tracker.heldTotalS() - 0.04) < 1e-6;
}

static bool anOpenHoldReportsItselfPeriodically() {
    BarrierEpisodeTracker tracker;
    std::uint64_t t = 0;
    int ongoing = 0;
    for (int i = 0; i < 2600; ++i) {  // 5.2 s held
        for (const BarrierEpisodeReport& r : tracker.update(heldTick(t))) {
            if (r.kind == BarrierEpisodeReport::Kind::HeldOngoing) ++ongoing;
        }
        t += 2'000'000;
    }
    return ongoing == 2;  // at 2 s and 4 s
}

static bool longInBandBrakingIsReportedSeparately() {
    BarrierEpisodeTracker tracker;
    std::uint64_t t = 0;
    BarrierTickInput in = heldTick(t);
    in.held = false;
    in.headroom_m = 0.012;
    std::vector<BarrierEpisodeReport> reports;
    for (int i = 0; i < 600; ++i) {  // 1.2 s braking in band, never at the floor
        in.now_ns = t;
        tracker.update(in);
        t += 2'000'000;
    }
    BarrierTickInput idle;
    idle.dt_s = kDt;
    std::vector<BarrierEpisodeReport> ended;
    for (int i = 0; i < 40; ++i) {
        idle.now_ns = t;
        for (const BarrierEpisodeReport& r : tracker.update(idle)) ended.push_back(r);
        t += 2'000'000;
    }
    return ended.size() == 1 &&
           ended.front().kind == BarrierEpisodeReport::Kind::BrakingEnded &&
           tracker.heldCount() == 0 && tracker.brakingTotalS() > 1.19;
}

static bool reportLineNamesArmPairAndBlindSpot() {
    BarrierEpisodeReport r;
    r.kind = BarrierEpisodeReport::Kind::HeldEnded;
    r.duration_s = 1.57;
    r.pair = "left_link6_0 <-> right_pika_gripper_base";
    r.klass = "arm_arm";
    r.min_headroom_m = -0.00002;
    r.folded_m = 0.0213;
    r.max_correction_deg_s = 1.2;
    r.counted_blocked_fraction = 0.0;
    const std::string line = formatBarrierReport("left", r);
    return line.find("HELD left arm 1.57 s") != std::string::npos &&
           line.find("right_pika_gripper_base") != std::string::npos &&
           line.find("[arm_arm]") != std::string::npos &&
           line.find("plan folded 21.3 mm") != std::string::npos &&
           line.find("clamp_count on 0%") != std::string::npos;
}

int main() {
    std::printf("test_barrier_status\n");
    check(inBandButRequestWithinAllowanceIsNotBraking(),
          "a row in band that the request does not violate is not braking");
    check(violatedRowWithCorrectionIsBraking(), "violated row + correction = braking");
    check(rowAtItsFloorIsHeldEvenAtOneDegPerSecond(),
          "a row AT its floor is held at 1 deg/s (under the clamp_count bar)");
    check(correctionBelowEpsilonIsNotBraking(), "no correction = not braking");
    check(theOtherArmsRowDoesNotBlameThisArm(), "a row without this arm in J is not its");
    check(floorAndRoiRowsAreNotTheCollisionBarrier(), "Other-class rows are ignored");
    check(tightestViolatedRowIsTheOneReported(), "the tightest violated row is reported");
    check(heldEpisodeIsReportedWithTheBlindSpotQuantified(),
          "held episode reports duration, folded plan and the clamp_count fraction");
    check(aOneTickFlickerDoesNotSplitAnEpisode(), "a one-tick flicker does not split it");
    check(shortHoldsAreNotReportedButStillCounted(), "short holds count but do not report");
    check(anOpenHoldReportsItselfPeriodically(), "an open hold reports every 2 s");
    check(longInBandBrakingIsReportedSeparately(), "long in-band braking reports too");
    check(reportLineNamesArmPairAndBlindSpot(), "the report line names arm, pair, blind spot");
    std::printf("%s\n", g_failures == 0 ? "ALL PASS" : "FAILURES");
    return g_failures == 0 ? 0 : 1;
}
