import argparse
import csv
import math
import os
import tempfile
import unittest

import analyze_ft_application_point as afap


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def default_args(**overrides):
    args = argparse.Namespace(
        enter_n=8.0,
        exit_n=4.0,
        min_len=20,
        min_delta_n=5.0,
        pre_baseline=200,
        baseline_gap=20,
        warn_baseline_n=2.0,
        expect_r=None,
        tol_mm=15.0,
    )
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def synthetic_rows(r_true, bias, quiet_before=1000, quiet_after=500):
    """Quiet -> two-phase push (+y then +x) applied at r_true -> quiet."""
    rows = []

    def noise(i, k):
        return 0.03 * math.sin(0.7 * i + k)

    def emit(i, force):
        tau = cross(r_true, force)
        rows.append(tuple(
            bias[k] + noise(i, k) + (force + tau)[k] for k in range(6)
        ))

    i = 0
    for _ in range(quiet_before):
        emit(i, (0.0, 0.0, 0.0))
        i += 1
    for _ in range(200):
        emit(i, (0.0, 12.0, 0.0))
        i += 1
    for _ in range(200):
        emit(i, (10.0, 0.0, 0.0))
        i += 1
    for _ in range(quiet_after):
        emit(i, (0.0, 0.0, 0.0))
        i += 1
    return rows


def write_csv(path, rows, arm="right"):
    header = ["tick", "loop_start_time_ns"]
    header += [f"{arm}_ft_fast_external_{c}" for c in afap.WRENCH_FIELDS]
    header += [f"{arm}_ft_healthy", f"{arm}_force_control_operating_mode"]
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i, w in enumerate(rows):
            writer.writerow([i, 1_000_000_000 + i * 2_000_000, *w, 1, "monitor"])


class PureMathTest(unittest.TestCase):
    def test_per_sample_point_recovers_perpendicular_lever(self):
        force = (0.0, 10.0, 0.0)
        r_true = (0.0, 0.0, -0.1)
        tau = cross(r_true, force)
        [point] = afap.per_sample_points([force + tau])
        for got, want in zip(point, r_true):
            self.assertAlmostEqual(got, want, places=9)

    def test_fit_lever_recovers_r_and_couple(self):
        r_true = (0.02, -0.01, -0.15)
        tau0 = (0.1, -0.05, 0.02)
        deltas = []
        for force in ((10.0, 0.0, 0.0), (0.0, 8.0, 0.0), (0.0, 0.0, 12.0),
                      (5.0, 5.0, 0.0), (0.0, 4.0, 7.0)):
            tau = tuple(t + c for t, c in zip(cross(r_true, force), tau0))
            deltas.append(force + tau)
        fit = afap.fit_lever(deltas)
        self.assertIsNotNone(fit)
        for got, want in zip(fit["r"], r_true):
            self.assertAlmostEqual(got, want, places=6)
        for got, want in zip(fit["tau0"], tau0):
            self.assertAlmostEqual(got, want, places=6)
        # exact data; only the tiny Tikhonov regularizer contributes residual
        self.assertLess(fit["rms_nm"], 1e-6)

    def test_fit_lever_without_couple_survives_steady_push(self):
        # A steady push (constant dF) cannot separate tau0 from r x dF; the
        # no-couple fit must still recover the perpendicular lever exactly.
        r_true = (0.01, 0.0, -0.05)
        force = (0.0, 12.0, 0.0)
        tau = cross(r_true, force)
        deltas = [force + tau] * 10
        fit = afap.fit_lever(deltas, fit_couple=False)
        self.assertIsNotNone(fit)
        self.assertAlmostEqual(fit["r"][0], r_true[0], places=6)
        self.assertAlmostEqual(fit["r"][2], r_true[2], places=6)

    def test_detect_events_hysteresis_and_min_len(self):
        dmags = [0.0] * 100 + [12.0] * 50 + [0.0] * 100 + [12.0] * 5 + [0.0] * 50
        events = afap.detect_events(dmags, 8.0, 4.0, 20)
        self.assertEqual(events, [(100, 150)])


class AnalyzeCsvTest(unittest.TestCase):
    def analyze(self, r_true, bias, args=None):
        rows = synthetic_rows(r_true, bias)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "servo_log.csv")
            write_csv(path, rows)
            return afap.analyze_arm(path, "right", args or default_args())

    def test_recovers_application_point_from_push_event(self):
        r_true = (0.010, 0.000, -0.050)
        result = self.analyze(r_true, bias=(0.0,) * 6)
        self.assertFalse(result["tare_suspect"])
        events = [e for e in result["events"] if "skipped" not in e]
        self.assertEqual(len(events), 1)
        err_m = math.sqrt(sum(
            (got - want) ** 2 for got, want in zip(events[0]["r_fit_m"], r_true)
        ))
        self.assertLess(err_m, 0.002)

    def test_untared_baseline_is_flagged_and_event_analysis_survives(self):
        r_true = (0.010, 0.000, -0.050)
        bias = (30.0, 5.0, -3.0, 0.5, -0.2, 0.1)
        result = self.analyze(r_true, bias)
        self.assertTrue(result["tare_suspect"])
        self.assertGreater(result["quiet_baseline_f_n"], 2.0)
        events = [e for e in result["events"] if "skipped" not in e]
        self.assertEqual(len(events), 1)
        err_m = math.sqrt(sum(
            (got - want) ** 2 for got, want in zip(events[0]["r_fit_m"], r_true)
        ))
        self.assertLess(err_m, 0.002)

    def test_expectation_verdicts(self):
        r_true = (0.010, 0.000, -0.050)
        passing = self.analyze(r_true, (0.0,) * 6,
                               default_args(expect_r=r_true))
        self.assertIn("PASS", [v["verdict"] for v in passing["verdicts"]])
        failing = self.analyze(r_true, (0.0,) * 6,
                               default_args(expect_r=(0.0, 0.0, 0.0)))
        self.assertIn("FAIL", [v["verdict"] for v in failing["verdicts"]])

    def test_event_without_pre_baseline_window_is_skipped(self):
        rows = synthetic_rows((0.0, 0.0, -0.05), (0.0,) * 6, quiet_before=50)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "servo_log.csv")
            write_csv(path, rows)
            result = afap.analyze_arm(path, "right", default_args())
        self.assertTrue(result["events"])
        self.assertIn("skipped", result["events"][0])

    def test_missing_columns_fail_loudly(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "servo_log.csv")
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["tick", "loop_start_time_ns"])
                writer.writerow([0, 0])
            with self.assertRaises(KeyError):
                afap.load_rows(path, "right")


if __name__ == "__main__":
    unittest.main()
