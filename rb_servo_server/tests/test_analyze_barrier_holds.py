"""analyze_barrier_holds.py against synthetic servo logs (2026-09-15). No robot involved.

The case: a barrier hold whose per-tick correction stays UNDER the 2 deg/s bar that
self_collision_clamp_count uses. The reader must find it in both schemas -- the native
`<side>_barrier_*` columns, and the reconstruction from pre-2026-09-15 logs.
"""
import csv
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import analyze_barrier_holds as audit


DT_NS = 2_000_000
PAIR = "dual_rb5_850e_left_link6_0 <-> dual_rb5_850e_right_pika_gripper_base"


def write_log(path, *, native, held_ticks=500, correction=1.0, gap_ticks=40):
    """held_ticks of a held left arm, then gap_ticks of nothing. 500 ticks = 1.0 s."""
    base = ["loop_start_time_ns"]
    per_side = ["projection_applied_correction_deg_s", "hold_fold_m",
                "tcp_command_stand_x_m", "tcp_command_stand_y_m", "tcp_command_stand_z_m"]
    fields = base + [f"{s}_{c}" for s in ("left", "right") for c in per_side]
    fields += ["projection_min_headroom_m", "projection_min_headroom_pair",
               "projection_min_headroom_class"]
    if native:
        fields += [f"left_barrier_{c}" for c in ("held", "braking", "pair", "class", "headroom_m")]
        fields += [f"right_barrier_{c}" for c in ("held", "braking", "pair", "class", "headroom_m")]
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for i in range(held_ticks + gap_ticks):
            held = i < held_ticks
            row = {f: "" for f in fields}
            row["loop_start_time_ns"] = i * DT_NS
            row["left_projection_applied_correction_deg_s"] = correction if held else 0.0
            row["right_projection_applied_correction_deg_s"] = 0.0
            row["left_hold_fold_m"] = 0.0005 if held else 0.0
            row["right_hold_fold_m"] = 0.0
            row["left_tcp_command_stand_x_m"] = 0.40
            row["left_tcp_command_stand_y_m"] = -0.05
            row["left_tcp_command_stand_z_m"] = -0.19
            row["right_tcp_command_stand_x_m"] = 0.60
            row["right_tcp_command_stand_y_m"] = -0.20
            row["right_tcp_command_stand_z_m"] = -0.20
            row["projection_min_headroom_m"] = -0.00002 if held else 0.02
            row["projection_min_headroom_pair"] = PAIR if held else ""
            row["projection_min_headroom_class"] = "self" if held else ""
            if native:
                row["left_barrier_held"] = "1" if held else "0"
                row["left_barrier_braking"] = "1" if held else "0"
                row["left_barrier_pair"] = PAIR if held else ""
                row["left_barrier_class"] = "arm_arm" if held else ""
                row["left_barrier_headroom_m"] = -0.00002 if held else ""
                row["right_barrier_held"] = "0"
                row["right_barrier_braking"] = "0"
                row["right_barrier_pair"] = ""
                row["right_barrier_class"] = ""
                row["right_barrier_headroom_m"] = ""
            w.writerow(row)


class BarrierHoldsTest(unittest.TestCase):
    def _held(self, result):
        return [e for e in result["episodes"] if e["kind"] == "held"]

    def test_native_columns_find_the_hold_the_clamp_counter_missed(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log_native.csv"
            write_log(log, native=True)
            result = audit.analyze(log)
        self.assertTrue(result["native_columns"])
        held = self._held(result)
        self.assertEqual(len(held), 1)
        e = held[0]
        self.assertEqual(e["arm"], "left")
        self.assertAlmostEqual(e["duration_s"], 1.0, delta=0.02)
        self.assertEqual(e["pair"], PAIR)
        self.assertEqual(e["class"], "arm_arm")
        # 1 deg/s is under the 2 deg/s bar: the counter saw none of it.
        self.assertEqual(e["seen_by_clamp_count_pct"], 0.0)
        # 500 ticks x 0.5 mm of plan the fold discarded.
        self.assertAlmostEqual(e["folded_mm"], 250.0, delta=0.5)
        self.assertEqual(e["tcp_start_m"][0], 0.4)

    def test_old_logs_are_reconstructed(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log_legacy.csv"
            write_log(log, native=False)
            result = audit.analyze(log)
        self.assertFalse(result["native_columns"])
        held = self._held(result)
        self.assertEqual(len(held), 1)
        self.assertAlmostEqual(held[0]["duration_s"], 1.0, delta=0.02)
        self.assertEqual(held[0]["arm"], "left")

    def test_the_other_arm_is_not_blamed(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log_native.csv"
            write_log(log, native=True)
            result = audit.analyze(log)
        self.assertEqual([e["arm"] for e in self._held(result)], ["left"])

    def test_a_short_hold_is_still_an_episode(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log_native.csv"
            write_log(log, native=True, held_ticks=30)   # 60 ms
            result = audit.analyze(log)
        held = self._held(result)
        self.assertEqual(len(held), 1)
        self.assertAlmostEqual(held[0]["duration_s"], 0.06, delta=0.01)

    def test_x_bins_track_the_stand_frame(self):
        self.assertEqual(audit._xbin(0.40), "x<0.42")
        self.assertEqual(audit._xbin(0.60), "x>=0.56")


if __name__ == "__main__":
    unittest.main()
