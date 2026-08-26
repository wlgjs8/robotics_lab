// SetpointInterpolator: worker-side rate conversion of the loop's setpoint
// stream. The property under test is the one that motivated it (2026-08-26):
// with the producer 0.13 % faster than the consumer, latest-wins dropped one
// setpoint per beat and the wire carried a DOUBLE step; the interpolator must
// carry every setpoint as a uniform time dilation with NO doubled step.

#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

#include "rb_servo/control/setpoint_interpolator.hpp"

namespace {

int g_failures = 0;
void check(bool ok, const char* name) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

rb_servo::SendServoJRequest setpoint(double q, uint64_t seq) {
    rb_servo::SendServoJRequest r;
    r.q_target_deg.fill(q);
    r.command_seq = seq;
    r.host_time_ns = seq * 2'000'000ull;
    r.deadline_ns = r.host_time_ns + 150'000'000ull;
    return r;
}

}  // namespace

int main() {
    using rb_servo::SetpointInterpolator;

    // --- The beat scenario: producer 500.006 Hz, consumer 499.35 Hz. --------
    // Constant velocity v = 0.02 deg per producer tick. Simulate by event time:
    // pushes at i*2.000 ms, samples at k*2.0026 ms, ratio = 2.0026/2.000.
    std::printf("Beat scenario: no doubled steps, uniform dilation\n");
    {
        SetpointInterpolator interp;
        const double v = 0.02;
        const double push_dt = 2.000, samp_dt = 2.0026;
        const double ratio = samp_dt / push_dt;
        std::vector<double> outputs;
        uint64_t pushes = 0;
        double next_push = 0.0, next_samp = 0.1;  // worker phase-offset 100 us
        for (int step = 0; step < 20000; ++step) {
            if (next_push <= next_samp) {
                interp.push(setpoint(v * static_cast<double>(pushes), pushes));
                ++pushes;
                next_push += push_dt;
            } else {
                auto out = interp.sample(ratio);
                if (out) outputs.push_back(out->q_target_deg[0]);
                next_samp += samp_dt;
            }
        }
        // Wire deltas: every step must be ~v*ratio -- no 2x anywhere. The old
        // latest-wins path produced one 2v step per ~1.5 s here.
        double dmin = 1e9, dmax = -1e9;
        for (std::size_t k = 200; k + 1 < outputs.size(); ++k) {
            const double d = outputs[k + 1] - outputs[k];
            dmin = std::min(dmin, d);
            dmax = std::max(dmax, d);
        }
        const double expect = v * ratio;
        std::printf("    wire delta min=%.6f max=%.6f expected=%.6f (2x would be %.6f)\n",
                    dmin, dmax, expect, 2.0 * v);
        check(dmax < 1.5 * v, "no doubled step on the wire");
        check(dmin > 0.5 * v, "no stalled/zero step on the wire");
        check(std::fabs(dmax - dmin) < 0.1 * v, "dilation is uniform");
        check(interp.telemetry().rebase_total == 0, "no rebase in steady state");
        // Latency: the cursor trails the newest setpoint by about one setpoint.
        check(interp.telemetry().delay_setpoints > 0.2 &&
                  interp.telemetry().delay_setpoints < 2.2,
              "steady delay is ~1 setpoint (~2 ms)");
    }

    // --- Producer stall: hold at newest (legacy repeat semantics). ----------
    std::printf("Producer stall: hold at newest\n");
    {
        SetpointInterpolator interp;
        for (uint64_t i = 0; i < 10; ++i) interp.push(setpoint(0.02 * i, i));
        std::optional<rb_servo::SendServoJRequest> last;
        for (int k = 0; k < 20; ++k) last = interp.sample(1.0013);
        check(last.has_value(), "sample keeps returning during a stall");
        check(std::fabs(last->q_target_deg[0] - 0.02 * 9) < 1e-12,
              "held exactly at the newest setpoint");
        check(interp.telemetry().hold_total > 0, "holds are counted");
    }

    // --- Producer burst / consumer stall: single rebase, bounded step. ------
    std::printf("Burst: one rebase, no backlog replay\n");
    {
        SetpointInterpolator interp;
        for (uint64_t i = 0; i < 4; ++i) interp.push(setpoint(0.02 * i, i));
        auto a = interp.sample(1.0013);
        for (uint64_t i = 4; i < 12; ++i) interp.push(setpoint(0.02 * i, i));
        auto b = interp.sample(1.0013);
        check(a.has_value() && b.has_value(), "samples across the burst");
        check(interp.telemetry().rebase_total == 1, "burst causes exactly one rebase");
        check(interp.telemetry().delay_setpoints <= 2.5, "delay re-bounded after rebase");
    }

    // --- Metadata comes from the newer bracket; deadline honesty. -----------
    std::printf("Metadata: newer bracket\n");
    {
        SetpointInterpolator interp;
        interp.push(setpoint(0.0, 100));
        interp.push(setpoint(0.02, 101));
        interp.push(setpoint(0.04, 102));
        auto out = interp.sample(1.0);
        check(out.has_value(), "sample with a bracket");
        check(out->command_seq >= 101, "seq from the newer bracket");
    }

    // --- Before any push: nullopt; single push: verbatim bridge. ------------
    std::printf("Startup: bridge semantics\n");
    {
        SetpointInterpolator interp;
        check(!interp.sample(1.0).has_value(), "no setpoint -> nullopt");
        interp.push(setpoint(1.23, 7));
        auto out = interp.sample(1.0);
        check(out.has_value() && std::fabs(out->q_target_deg[0] - 1.23) < 1e-12,
              "single setpoint bridges verbatim");
    }

    std::printf("\n=== %s (%d failure%s) ===\n",
                g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
