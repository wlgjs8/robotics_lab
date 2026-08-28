"""The model's own predicted rows must survive the run that needs them.

A rollout on 2026-08-28 vibrated at 11-13 Hz. The control chain was cleared
stage by stage from the servo log -- chunk follower 0.80x on that band, output
SMD 0.73x, IK residual 0.8 um with branch_jump 0, force-control deviation
0.16 mm rms dominated by 1.1 Hz -- which left the policy's own predicted
trajectory as the only remaining source, and 180 um of 11-13 Hz was measured in
the command entering the follower.

That could not be CONFIRMED, because the chunk rows are published to the servo
and drawn in the GUI but written nowhere: the servo log keeps chunk metadata
(seq/age/horizon) and the rollout step log keeps ids and flags. So the last link
was closed by reading code, not by measurement.

Hence: always on, no toggle. A spectrum of the model's output is not something
anyone thinks to enable before the run that needs it.
"""
from __future__ import annotations

import json
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.rollout_step_log import CHUNK_ROW_LOG_SCHEMA, ChunkRowLogger


def _sample_rows(n=6):
    # x,y,z,qx,qy,qz,qw,grip
    return [[0.1 + 0.001 * i, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0, 40.0] for i in range(n)]


class ChunkRowLoggingTest(unittest.TestCase):
    def _write_one(self, tmp, **over):
        path = Path(tmp) / "rows.jsonl"
        log = ChunkRowLogger(path)
        self.assertTrue(log.enabled, log.disabled_reason)
        kw = dict(
            seq=7,
            t_mono=1.5,
            t_wall=time.time(),
            policy_dt_sec=1.0 / 30.0,
            execute_limit=6,
            runway_steps=4,
            anchor_mode="command",
            stitch_mode="boundary",
            speed_scale=1.0,
            projected={"left": _sample_rows(), "right": _sample_rows()},
            projected_delta={"left": _sample_rows(), "right": _sample_rows()},
        )
        kw.update(over)
        self.assertTrue(log.log_chunk(**kw))
        log.close()
        return path

    def test_rows_reach_the_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write_one(tmp)
            lines = path.read_text().strip().splitlines()
            self.assertEqual(len(lines), 1)
            rec = json.loads(lines[0])
            self.assertEqual(rec["schema"], CHUNK_ROW_LOG_SCHEMA)
            self.assertEqual(rec["seq"], 7)
            self.assertEqual(rec["execute_limit"], 6)
            self.assertEqual(rec["runway_steps"], 4)
            self.assertEqual(rec["anchor_mode"], "command")
            self.assertEqual(rec["stitch_mode"], "boundary")
            # The point of the whole file: the actual poses, per arm.
            self.assertEqual(len(rec["left"]), 6)
            self.assertEqual(len(rec["left"][0]), 8)   # x y z qx qy qz qw grip
            self.assertEqual(len(rec["right"]), 6)
            self.assertEqual(len(rec["left_delta"]), 6)

    def test_an_arm_with_no_rows_is_recorded_as_null(self):
        """A masked arm publishes nothing; that must be visible, not missing."""
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write_one(
                tmp,
                projected={"left": _sample_rows(), "right": None},
                projected_delta={"left": _sample_rows(), "right": None},
            )
            rec = json.loads(path.read_text().strip())
            self.assertIsNotNone(rec["left"])
            self.assertIsNone(rec["right"])

    def test_a_writer_that_cannot_start_disables_itself_quietly(self):
        """Telemetry must never take a rollout down with it."""
        def boom(_path):
            raise OSError("no space")

        log = ChunkRowLogger("/nonexistent/x.jsonl", writer_factory=boom)
        self.assertFalse(log.enabled)
        self.assertIn("writer_start_error", log.disabled_reason or "")
        self.assertFalse(
            log.log_chunk(
                seq=1, t_mono=0.0, t_wall=0.0, policy_dt_sec=None,
                execute_limit=0, runway_steps=0, anchor_mode="", stitch_mode="",
                speed_scale=None, projected={}, projected_delta={},
            )
        )
        log.close()

    def test_non_finite_scalars_do_not_produce_invalid_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write_one(tmp, speed_scale=float("nan"), policy_dt_sec=float("inf"))
            rec = json.loads(path.read_text().strip())   # would raise on NaN/Infinity
            self.assertIsNone(rec["speed_scale"])
            self.assertIsNone(rec["policy_dt_sec"])

    def test_close_is_idempotent(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = ChunkRowLogger(Path(tmp) / "rows.jsonl")
            log.close()
            log.close()
            self.assertFalse(log.enabled)


if __name__ == "__main__":
    unittest.main()
