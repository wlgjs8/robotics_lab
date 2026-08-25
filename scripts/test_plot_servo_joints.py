import csv
import math
import tempfile
import unittest
from pathlib import Path

import plot_servo_joints as psj


TICK_NS = 2_000_000


def write_log(path, *, ticks=800, move_from=200, move_to=600, include_q_ref=True,
              status=True):
    header = ["loop_start_time_ns"]
    for arm in ("left", "right"):
        header += [f"{arm}_q_{k}_{j}" for k in ("actual", "sent") for j in range(6)]
    if include_q_ref:
        for arm in ("left", "right"):
            header += [f"{arm}_q_ref_{j}" for j in range(6)]
    if status:
        header.append("init_motion_aggregate_status")

    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for i in range(ticks):
            if i < move_from:
                s = 0.0
            elif i > move_to:
                s = 10.0
            else:
                s = 10.0 * (i - move_from) / (move_to - move_from)
            sent = [s + j for j in range(6)]
            ref = [s * 0.98 + j for j in range(6)]
            actual = [s * 0.96 + j for j in range(6)]
            row = [i * TICK_NS] + actual + sent + actual + sent
            if include_q_ref:
                row += ref + ref
            if status:
                row.append("executing" if move_from <= i <= move_to else "idle")
            writer.writerow(row)


class PlotServoJointsTest(unittest.TestCase):
    def test_writes_png_for_default_window(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log.csv"
            out = Path(tmp) / "out.png"
            write_log(log)
            self.assertEqual(psj.main([str(log), "-o", str(out)]), 0)
            self.assertTrue(out.exists())
            self.assertGreater(out.stat().st_size, 10_000)

    def test_motion_window_brackets_the_move(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log.csv"
            write_log(log, ticks=800, move_from=200, move_to=600)
            data = psj.load(log, ["left", "right"])
            lo, hi = psj.motion_window(data, ["left", "right"], pad=50)
            self.assertLessEqual(lo, 201)
            self.assertGreaterEqual(hi, 600)
            self.assertGreater(lo, 100)   # idle head is excluded
            self.assertLess(hi, 800)      # idle tail is excluded

    def test_missing_q_ref_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log.csv"
            out = Path(tmp) / "out.png"
            write_log(log, include_q_ref=False)
            self.assertEqual(psj.main([str(log), "-o", str(out)]), 2)
            self.assertFalse(out.exists())

    def test_phase_spans_track_init_motion_status(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log.csv"
            write_log(log, move_from=200, move_to=600)
            data = psj.load(log, ["left"])
            spans = psj.phase_spans(data, 0, data["n"])
            self.assertEqual(len(spans), 1)
            lo, hi, label = spans[0]
            self.assertEqual(label, "executing")
            self.assertEqual((lo, hi), (200, 601))

    def test_explicit_tick_window_and_variants(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log.csv"
            write_log(log)
            for extra, name in (([], "abs"), (["--relative"], "rel"), (["--error"], "err")):
                out = Path(tmp) / f"{name}.png"
                argv = [str(log), "-o", str(out), "--arm", "left",
                        "--tick-start", "300", "--tick-stop", "500"] + extra
                self.assertEqual(psj.main(argv), 0, name)
                self.assertTrue(out.exists(), name)

    def test_html_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log.csv"
            out = Path(tmp) / "out.html"
            write_log(log)
            self.assertEqual(psj.main([str(log), "-o", str(out), "--html"]), 0)
            text = out.read_text()
            self.assertIn('id="servo-joints"', text)
            self.assertIn("plotly_relayout", text)      # zoom hook is wired
            self.assertIn("scrollZoom", text)

    def test_html_integer_tick_thresholds_are_ordered(self):
        # The clamp ladder in the page must stay monotone: a tighter zoom never
        # gets a coarser tick step, or the axis stops resolving single 2 ms ticks.
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log.csv"
            out = Path(tmp) / "out.html"
            write_log(log)
            psj.main([str(log), "-o", str(out), "--html"])
            text = out.read_text()
            self.assertIn("span <= 12 ? 1 : (span <= 30 ? 2 : (span <= 60 ? 5 : null))", text)
            self.assertIn("ax.dtick === undefined || ax.dtick === null", text)

    def test_html_default_extension(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log.csv"
            write_log(log)
            self.assertEqual(psj.main([str(log), "--html"]), 0)
            self.assertTrue((Path(tmp) / "servo_log_joints.html").exists())

    def test_html_missing_q_ref_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log.csv"
            out = Path(tmp) / "out.html"
            write_log(log, include_q_ref=False)
            self.assertEqual(psj.main([str(log), "-o", str(out), "--html"]), 2)
            self.assertFalse(out.exists())

    def test_empty_tick_window_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "servo_log.csv"
            write_log(log)
            self.assertEqual(
                psj.main([str(log), "-o", str(Path(tmp) / "x.png"),
                          "--tick-start", "300", "--tick-stop", "301"]), 2)


if __name__ == "__main__":
    unittest.main()
