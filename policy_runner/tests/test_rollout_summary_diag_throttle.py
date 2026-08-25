"""RolloutSummaryRecorder must not rebuild summary-only diagnostics every control tick.

record_source() runs once per control-loop tick (~110-290 Hz). Three of the fields it
maintained -- camera_runtime, inference_diagnostics -- are read exactly once,
by to_dict(), when the rollout summary is written. Rebuilding them per tick cost ~5.7 ms/tick
because OpenpiRemoteActionSource.inference_diagnostics_snapshot() deep-copies a rolling window
of 64 inference events and takes the lock shared with the inference thread. The window fills at
one event per chunk (~235 ms), so the cost ramped over ~15 s and then plateaued -- exactly the
measured tick profile (0.68 -> 5.75 ms) that pinned the achieved policy rate near 21 Hz.
"""
from __future__ import annotations

import unittest
from types import SimpleNamespace

from policy_runner.rollout_modes import RolloutModePolicy, RolloutSummaryRecorder


class _CountingSource:
    """Counts the expensive summary-only probes; the cheap attribute reads are free."""

    def __init__(self) -> None:
        self.diag_calls = 0
        self.camera_calls = 0
        self.image_decode_count = 0
        self.missing_camera_count = 0
        self.camera_names = ["left_realsense_color"]
        self.command_family = "tcp_target_pose"

    def inference_diagnostics_snapshot(self) -> dict[str, object]:
        self.diag_calls += 1
        return {"total_inferences": self.diag_calls}

    def camera_runtime_status(self) -> dict[str, object]:
        self.camera_calls += 1
        return {"state": "ready"}



def _recorder(**kw) -> RolloutSummaryRecorder:
    return RolloutSummaryRecorder(
        policy=RolloutModePolicy.from_value("sim_dryrun"),
        checkpoint_path="openpi://test",
        config_path="test.yaml",
        **kw,
    )


class RolloutSummaryDiagnosticsThrottleTest(unittest.TestCase):
    def test_tick_loop_does_not_rebuild_diagnostics_every_call(self) -> None:
        source = _CountingSource()
        rec = _recorder()
        for _ in range(500):
            rec.record_source(source)
        # First call primes the fields; the rest fall inside the refresh interval.
        self.assertEqual(source.diag_calls, 1, "diagnostics rebuilt per tick")
        self.assertEqual(source.camera_calls, 1)

    def test_cheap_fields_still_track_every_tick(self) -> None:
        # Throttling must not stop the counters/labels the summary also reports.
        source = _CountingSource()
        rec = _recorder()
        rec.record_source(source)
        source.image_decode_count = 42
        source.command_family = "spacemouse"
        rec.record_source(source)
        self.assertEqual(rec.image_decode_count, 42)
        self.assertEqual(rec.command_family, "spacemouse")

    def test_to_dict_forces_a_fresh_snapshot(self) -> None:
        source = _CountingSource()
        rec = _recorder()
        rec.record_source(source)
        self.assertEqual(source.diag_calls, 1)
        document = rec.to_dict(source)
        self.assertEqual(source.diag_calls, 2, "written summary must not be stale")
        self.assertEqual(document["inference_diagnostics"]["total_inferences"], 2)

    def test_zero_interval_restores_per_call_refresh(self) -> None:
        source = _CountingSource()
        rec = _recorder(diagnostics_refresh_sec=0.0)
        for _ in range(5):
            rec.record_source(source)
        self.assertEqual(source.diag_calls, 5)


if __name__ == "__main__":
    unittest.main()
