import csv
import math
import tempfile
import unittest
from pathlib import Path

import analyze_box_latency as abl


TICK_NS = 2_000_000  # 500 Hz


def write_log(
    path,
    *,
    ticks=4000,
    sent_to_ref=5,
    ref_to_actual=3,
    amplitude=20.0,
    hold_every=0,
    idle=False,
    include_q_ref=True,
    extras=True,
    rback=None,
    rback_warmup=0,
    malformed=0,
):
    """Synthesize a servo log with a known sent->ref / ref->actual delay.

    The command is a chirp so the delay objective has a sharp minimum -- a pure
    sine is nearly shift-invariant and would not discriminate.
    """

    def q(i, j):
        if idle or i < 0:
            return 10.0 + j
        phase = 2.0 * math.pi * (0.4 + 0.9 * i / ticks) * (i / 500.0)
        return 10.0 + j + amplitude * math.sin(phase + 0.3 * j)

    header = ["tick", "loop_start_time_ns"]
    for arm in ("left", "right"):
        header += [f"{arm}_q_actual_{j}" for j in range(6)]
    for arm in ("left", "right"):
        header += [f"{arm}_q_sent_{j}" for j in range(6)]
    if include_q_ref:
        for arm in ("left", "right"):
            header += [f"{arm}_q_ref_{j}" for j in range(6)]
        header += ["left_q_ref_valid", "right_q_ref_valid",
                   "left_q_actual_valid", "right_q_actual_valid"]
    if extras:
        for arm in ("left", "right"):
            header += [f"{arm}_state_age_us", f"{arm}_send_start_ns",
                       f"{arm}_send_end_ns", f"{arm}_reqdata_exchange_sequence"]
    if rback is not None:
        for arm in ("left", "right"):
            header += [f"{arm}_rback_observed", f"{arm}_rback_fill",
                       f"{arm}_rback_fill_min", f"{arm}_rback_fill_max",
                       f"{arm}_rback_seq", f"{arm}_rback_parsed_total",
                       f"{arm}_rback_drained_total", f"{arm}_rback_malformed_total"]

    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        last_ref = None
        for i in range(ticks):
            sent = [q(i, j) for j in range(6)]
            ref = [q(i - sent_to_ref, j) for j in range(6)]
            if hold_every and last_ref is not None and i % hold_every:
                ref = list(last_ref)
            last_ref = list(ref)
            actual = [q(i - sent_to_ref - ref_to_actual, j) for j in range(6)]
            row = [i, i * TICK_NS] + actual + actual + sent + sent
            if include_q_ref:
                row += ref + ref + [1, 1, 1, 1]
            if extras:
                for _ in ("left", "right"):
                    row += [50.0, i * TICK_NS, i * TICK_NS + 5000, i]
            if rback is not None:
                # -1 until the first ACK arrives, mirroring the C++ contract.
                fill = -1 if i < rback_warmup else rback
                for _ in ("left", "right"):
                    row += [int(fill >= 0), fill, fill, fill,
                            max(0, i - rback_warmup), max(0, i - rback_warmup),
                            i + 1, malformed]
            writer.writerow(row)
    return header


class AnalyzeBoxLatencyTest(unittest.TestCase):
    def analyze(self, path, **kwargs):
        opts = dict(arms=["left"], start_sec=0.0, duration_sec=0.0,
                    max_lag=40, n_segments=1)
        opts.update(kwargs)
        return abl.analyze(Path(path), **opts)

    def test_recovers_known_delays(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, sent_to_ref=5, ref_to_actual=3)
            report = self.analyze(path)
            self.assertAlmostEqual(report["tick_rate_hz"], 500.0, places=3)
            stages = report["arms"]["left"]["stages"]
            self.assertAlmostEqual(
                stages["sent_to_ref"]["summary"]["lag_ticks_weighted"], 5.0, places=2)
            self.assertAlmostEqual(
                stages["ref_to_actual"]["summary"]["lag_ticks_weighted"], 3.0, places=2)
            self.assertAlmostEqual(
                stages["sent_to_actual"]["summary"]["lag_ticks_weighted"], 8.0, places=2)
            self.assertAlmostEqual(
                stages["sent_to_ref"]["summary"]["lag_ms_weighted"], 10.0, places=2)

    def test_stages_are_additive(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, sent_to_ref=7, ref_to_actual=2)
            stages = self.analyze(path)["arms"]["left"]["stages"]
            total = (stages["sent_to_ref"]["summary"]["lag_ticks_weighted"]
                     + stages["ref_to_actual"]["summary"]["lag_ticks_weighted"])
            self.assertAlmostEqual(
                total, stages["sent_to_actual"]["summary"]["lag_ticks_weighted"], places=2)

    def test_gain_is_unity_for_pure_delay(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, sent_to_ref=4)
            fits = self.analyze(path)["arms"]["left"]["stages"]["sent_to_ref"]["per_joint"]
            for fit in fits:
                self.assertTrue(fit["usable"])
                self.assertAlmostEqual(fit["gain"], 1.0, places=3)
                # The objective must actually discriminate: the residual at the
                # fitted lag is orders of magnitude below the zero-lag residual.
                self.assertLess(fit["rmse_deg"], 1e-6)
                self.assertGreater(fit["rmse_at_zero_lag_deg"], 0.1)
                self.assertGreater(
                    fit["rmse_at_zero_lag_deg"], 1000.0 * fit["rmse_deg"])

    def test_missing_q_ref_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, include_q_ref=False)
            with self.assertRaises(abl.LogError) as ctx:
                self.analyze(path)
            self.assertIn("left_q_ref_0", str(ctx.exception))
            self.assertIn("predates", str(ctx.exception))

    def test_idle_window_reports_unobservable_instead_of_guessing(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, idle=True)
            arm = self.analyze(path)["arms"]["left"]
            self.assertIn("error", arm)
            self.assertIn("idle", arm["error"])
            self.assertNotIn("stages", arm)

    def test_held_readback_is_counted(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, hold_every=2)
            fresh = self.analyze(path)["arms"]["left"]["readback_freshness"]["q_ref"]
            self.assertTrue(fresh["usable"])
            self.assertAlmostEqual(fresh["fresh_fraction"], 0.5, delta=0.02)
            self.assertAlmostEqual(fresh["implied_update_hz"], 250.0, delta=10.0)

    def test_window_selection(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, ticks=5000)
            report = self.analyze(path, start_sec=2.0, duration_sec=3.0)
            self.assertAlmostEqual(report["window_sec"][0], 2.0, places=2)
            self.assertLess(report["window_sec"][1], 5.0)
            self.assertAlmostEqual(report["ticks"], 1500, delta=2)

    def test_window_too_short_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, ticks=5000)
            with self.assertRaises(abl.LogError):
                self.analyze(path, start_sec=0.0, duration_sec=0.1)

    def test_model_separates_dead_time_from_filter(self):
        # A pure shift fit reads a filtered stage as one big delay. The model
        # must recover the 3-tick dead time and the 0.1 coefficient separately.
        import numpy as np

        ticks, delay, a = 6000, 3, 0.1
        x = np.zeros(ticks)
        for i in range(ticks):
            phase = 2.0 * math.pi * (0.4 + 0.9 * i / ticks) * (i / 500.0)
            x[i] = 20.0 * math.sin(phase)
        y = np.zeros(ticks)
        for k in range(1, ticks):
            y[k] = (1 - a) * y[k - 1] + a * x[max(0, k - delay)]
        spans = [(500, ticks)]
        fit = abl._first_order_fit(x, y, spans, 30)
        self.assertTrue(fit["usable"])
        self.assertEqual(fit["delay_ticks"], delay)
        self.assertAlmostEqual(fit["a"], a, places=3)
        self.assertAlmostEqual(fit["tau_ticks"], -1 / math.log(1 - a), places=2)
        self.assertAlmostEqual(fit["ramp_lag_ticks"], delay + (1 - a) / a, places=2)

    def test_free_decay_pole_recovers_known_pole(self):
        import numpy as np

        pole = 0.9
        err = np.array([2.0 * pole ** k for k in range(80)])
        got = abl._free_decay_pole(err, [(0, 80)], floor_deg=0.02)
        self.assertTrue(got["usable"])
        self.assertAlmostEqual(got["pole"], pole, places=4)
        self.assertAlmostEqual(got["a"], 1 - pole, places=4)
        self.assertGreater(got["r2"], 0.999)
        # 90 % of a 0.9-pole decay takes ln(0.1)/ln(0.9) ticks.
        self.assertAlmostEqual(got["settling_ticks"]["90%"],
                               math.log(0.1) / math.log(pole), places=2)

    def test_free_decay_rejects_non_exponential(self):
        import numpy as np

        linear = np.linspace(2.0, 0.02, 80)   # ramps down, does not decay
        got = abl._free_decay_pole(linear, [(0, 80)], floor_deg=0.02)
        # It may still fit a pole, but the r2 must expose that it is not one.
        if got.get("usable"):
            self.assertLess(got["r2"], 0.99)

    def test_decay_spans_start_after_the_move(self):
        import numpy as np

        src = np.r_[np.arange(100.0), np.full(200, 99.0)]
        moving = np.r_[np.ones(100, dtype=bool), np.zeros(200, dtype=bool)]
        spans = abl._decay_spans(src, moving, max_len=50)
        self.assertEqual(len(spans), 1)
        self.assertEqual(spans[0][0], 100)
        self.assertEqual(spans[0][1], 150)

    def test_rback_queue_reported_as_constant(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, rback=5, rback_warmup=10)
            qa = self.analyze(path)["arms"]["left"]["rback_queue"]
            self.assertEqual(qa["fill_min"], 5)
            self.assertEqual(qa["fill_max"], 5)
            self.assertTrue(qa["fill_constant"])
            self.assertEqual(qa["fill_median"], 5.0)
            self.assertEqual(qa["never_reported_ticks"], 10)
            self.assertIn("RBACK queue", abl.format_report(self.analyze(path)))

    def test_rback_absent_column_is_not_invented(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path)                       # no rback columns at all
            report = self.analyze(path)
            self.assertNotIn("rback_queue", report["arms"]["left"])
            self.assertIn("left_rback_fill", report["columns_absent"])

    def test_rback_never_reported_is_not_counted_as_empty_queue(self):
        # fill == -1 for every tick must not average in as a zero-depth queue.
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, rback=-1)
            qa = self.analyze(path)["arms"]["left"]["rback_queue"]
            self.assertEqual(qa["reported_ticks"], 0)
            self.assertNotIn("fill_median", qa)
            self.assertIn("no occupancy", abl.format_report(self.analyze(path)))

    def test_rback_drift_detected_and_locked_distinguished(self):
        import csv as _csv

        def log_with_fill(path, fill_of_tick):
            write_log(path, ticks=3000, rback=0)          # header shape only
            rows = list(_csv.reader(open(path)))
            hdr, body = rows[0], rows[1:]
            ix = {n: i for i, n in enumerate(hdr)}
            for k, row in enumerate(body):
                for arm in ("left", "right"):
                    row[ix[f"{arm}_rback_fill"]] = str(fill_of_tick(k))
            with open(path, "w", newline="") as h:
                w = _csv.writer(h)
                w.writerow(hdr)
                w.writerows(body)

        with tempfile.TemporaryDirectory() as tmp:
            # 1 tick/s growth at 500 Hz == one extra queue entry every 500 ticks.
            drifting = Path(tmp) / "drift.csv"
            log_with_fill(drifting, lambda k: 5 + k // 500)
            d = self.analyze(drifting)["arms"]["left"]["rback_queue"]["fill_drift"]
            self.assertAlmostEqual(d["ticks_per_sec"], 1.0, delta=0.05)
            self.assertAlmostEqual(d["ms_per_sec"], 2.0, delta=0.1)
            self.assertAlmostEqual(d["implied_box_drain_hz"], 499.0, delta=0.5)
            self.assertIn("DRIFTING", abl.format_report(self.analyze(drifting)))

            locked = Path(tmp) / "locked.csv"
            log_with_fill(locked, lambda k: 5)
            d2 = self.analyze(locked)["arms"]["left"]["rback_queue"]["fill_drift"]
            self.assertAlmostEqual(d2["ticks_per_sec"], 0.0, delta=0.01)
            self.assertIn("LOCKED", abl.format_report(self.analyze(locked)))

    def test_rback_malformed_counter_surfaces(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path, rback=5, malformed=3)
            text = abl.format_report(self.analyze(path))
            self.assertIn("did not parse", text)

    def test_report_formats_without_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "servo_log.csv"
            write_log(path)
            text = abl.format_report(self.analyze(path, n_segments=3))
            self.assertIn("sent_to_ref", text)
            self.assertIn("stationarity", text)


if __name__ == "__main__":
    unittest.main()
