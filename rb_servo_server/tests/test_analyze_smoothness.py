import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import analyze_smoothness


def base_fields(with_wire=False, with_projection=False):
    fields = [
        "tick",
        "loop_start_time_ns",
        "safety_verdict",
        "fault_latched",
        "fault_reason",
        "command_buffer_returned_age_ms",
        "command_buffer_latest_timeout_ms",
        "left_send_start_ns",
        "right_send_start_ns",
        "left_follower_reanchor_count",
        "right_follower_reanchor_count",
    ]
    for arm in ("left", "right"):
        for j in range(6):
            fields.append(f"{arm}_q_sent_{j}")
        for j in range(6):
            fields.append(f"{arm}_q_ref_{j}")
    if with_wire:
        for arm in ("left", "right"):
            fields += [
                f"{arm}_worker_pending_overwrites_total",
                f"{arm}_worker_repeated_sends_total",
                f"{arm}_worker_wire_dispatches_total",
                f"{arm}_worker_wire_send_start_ns",
                f"{arm}_worker_wire_send_end_ns",
            ]
    if with_projection:
        fields += [
            "projection_active",
            "projection_constraint_count",
            "left_projection_correction_deg_s",
            "right_projection_correction_deg_s",
            "projection_ceiling_clamped",
            "projection_min_margin_m",
            "selfcol_verdict_age_ms",
        ]
    return fields


class AnalyzeSmoothnessTest(unittest.TestCase):
    def write_log(self, path, rows, with_wire=False, with_projection=False):
        fields = base_fields(with_wire, with_projection)
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)

    def synth_rows(self, n=600, with_wire=False, with_projection=False):
        """Constant-velocity motion with three injected events:

        - RoiViolation verdict for ticks [200, 210) whose EXIT (tick 210)
          carries a q_sent velocity step (accel spike inside the +/-4 window)
        - a re-anchor counter step at tick 300 with a spike at tick 302
        - a fault rising edge at tick 400 with cmd age 149.5 / timeout 150
          (deadline-race signature)
        """
        rows = []
        q = 0.0
        vel = 0.02  # deg/tick, constant -> zero accel baseline
        reanchor = 0
        for t in range(n):
            step = vel
            if t == 210 or t == 302:
                step = vel + 0.05  # one-tick velocity step -> 12,500 deg/s^2
            q += step
            row = {
                "tick": t,
                "loop_start_time_ns": 1_000_000_000 + t * 2_000_000,
                "safety_verdict": "RoiViolation" if 200 <= t < 210 else "Ok",
                "fault_latched": 1 if t >= 400 else 0,
                "fault_reason": "CommandTimeout test" if t >= 400 else "",
                "command_buffer_returned_age_ms": 149.5 if t == 400 else 10.0,
                "command_buffer_latest_timeout_ms": 150,
                "left_send_start_ns": 1_000_030_000 + t * 2_000_000,
                "right_send_start_ns": 1_000_030_000 + t * 2_000_000,
                "left_follower_reanchor_count": reanchor,
                "right_follower_reanchor_count": 0,
            }
            if t == 300:
                reanchor += 1
                row["left_follower_reanchor_count"] = reanchor
            for arm in ("left", "right"):
                for j in range(6):
                    row[f"{arm}_q_sent_{j}"] = q if (arm == "left" and j == 0) else 0.0
                    row[f"{arm}_q_ref_{j}"] = q if (arm == "left" and j == 0) else 0.0
            if with_wire:
                # wire period 2002.6us vs loop 2000us
                for arm in ("left", "right"):
                    row[f"{arm}_worker_pending_overwrites_total"] = t // 750
                    row[f"{arm}_worker_repeated_sends_total"] = 0
                    row[f"{arm}_worker_wire_dispatches_total"] = t
                    row[f"{arm}_worker_wire_send_start_ns"] = (
                        2_000_000_000 + t * 2_002_600
                    )
                    row[f"{arm}_worker_wire_send_end_ns"] = (
                        2_000_003_000 + t * 2_002_600
                    )
            if with_projection:
                active = 1 if 200 <= t < 210 else 0
                row["projection_active"] = active
                row["projection_constraint_count"] = 2 * active
                row["left_projection_correction_deg_s"] = 5.0 * active
                row["right_projection_correction_deg_s"] = 0.0
                row["projection_ceiling_clamped"] = 1 if t == 205 else 0
                row["projection_min_margin_m"] = 0.004 if active else -1
                row["selfcol_verdict_age_ms"] = 12.5 if active else -1
            rows.append(row)
        return rows

    def test_detects_boundary_transition_spike(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "log.csv"
            self.write_log(path, self.synth_rows())
            result = analyze_smoothness.analyze(str(path))
        bt = result["boundary_transitions"]["left"]
        self.assertEqual(bt["entries"], 1)
        self.assertEqual(bt["exits"], 1)
        # the exit at tick 210 carries the injected 12,500 deg/s^2 step
        self.assertGreaterEqual(bt["windows_over_2000"], 1)
        worst = max(w["accel_deg_s2"] for w in bt["worst"])
        self.assertGreater(worst, 10_000.0)

    def test_detects_reanchor_post_spike(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "log.csv"
            self.write_log(path, self.synth_rows())
            result = analyze_smoothness.analyze(str(path))
        ra = result["reanchor"]["left"]
        self.assertEqual(ra["events"], 1)
        self.assertEqual(ra["post_peaks_over_2000"], 1)

    def test_flags_deadline_race_on_fault_onset(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "log.csv"
            self.write_log(path, self.synth_rows())
            result = analyze_smoothness.analyze(str(path))
        self.assertEqual(len(result["fault_onsets"]), 1)
        onset = result["fault_onsets"][0]
        self.assertTrue(onset["deadline_race"])
        self.assertAlmostEqual(onset["cmd_age_ms"], 149.5)

    def test_quiet_baseline_has_no_tail_events(self):
        rows = self.synth_rows()
        # strip the injected spikes/events: constant velocity everywhere
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "log.csv"
            quiet = []
            q = 0.0
            for row in rows:
                q += 0.02
                row = dict(row)
                row["left_q_sent_0"] = q
                row["left_q_ref_0"] = q
                row["safety_verdict"] = "Ok"
                row["fault_latched"] = 0
                row["left_follower_reanchor_count"] = 0
                quiet.append(row)
            self.write_log(path, quiet)
            result = analyze_smoothness.analyze(str(path))
        tail = result["accel_tail"]["left"]["q_sent"]
        self.assertEqual(tail["exceed_per_sec"]["2000"], 0.0)
        self.assertEqual(result["boundary_transitions"]["left"]["entries"], 0)
        self.assertEqual(result["reanchor"]["left"]["events"], 0)
        self.assertEqual(result["fault_onsets"], [])

    def test_wire_and_projection_columns(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "log.csv"
            self.write_log(
                path,
                self.synth_rows(with_wire=True, with_projection=True),
                with_wire=True,
                with_projection=True,
            )
            result = analyze_smoothness.analyze(str(path))
            totals = analyze_smoothness.wire_counter_totals(str(path))
        wire = result["wire"]["left"]
        self.assertAlmostEqual(wire["wire_period_us"]["mean"], 2002.6, delta=0.5)
        self.assertEqual(wire["wire_period_us"]["over_2500us"], 0)
        proj = result["projection"]
        self.assertEqual(proj["ticks_active"], 10)
        self.assertEqual(proj["ceiling_clamped_ticks"], 1)
        self.assertAlmostEqual(proj["min_margin_m"], 0.004)
        self.assertAlmostEqual(proj["max_verdict_age_ms"], 12.5)
        self.assertEqual(totals["left_worker_wire_dispatches_total"], 599)

    def test_missing_optional_columns_degrade(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "log.csv"
            self.write_log(path, self.synth_rows())
            result = analyze_smoothness.analyze(str(path))
        self.assertEqual(result["projection"], "n/a")
        self.assertIn("n/a", result["wire"]["left"])


if __name__ == "__main__":
    unittest.main()
