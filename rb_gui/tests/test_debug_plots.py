from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rb_servo_gui.app import (
    _debug_uplot_cursor,
    _debug_uplot_sync_supported,
    _set_debug_mode_active,
)
from rb_servo_gui.debug_plots import (
    ArmScopeSample,
    build_arm_series,
    build_debug_snapshot,
    finite_difference,
    moving_average,
)
from rb_servo_gui.scope_receiver import ScopeStore, parse_scope_payload


class FakeHandle:
    def __init__(self, visible: bool = True) -> None:
        self.visible = visible


class FakeUrdf:
    def __init__(self, show_visual: bool = True) -> None:
        self.show_visual = show_visual


def _joint6(value: float) -> list[float]:
    return [value + idx for idx in range(6)]


def _scope_payload(*, start_ns: int = 1_000_000_000, step_ns: int = 2_000_000) -> dict:
    times = [start_ns + index * step_ns for index in range(3)]

    def arm(base: float) -> dict:
        return {
            "t_robot_ns": [10, 12, 14],
            "q_sent": [_joint6(base + 0.0), _joint6(base + 1.0), _joint6(base + 3.0)],
            "q_ref": [_joint6(base + 0.0), _joint6(base + 0.5), _joint6(base + 2.0)],
            "q_actual": [_joint6(base + 0.0), _joint6(base + 0.25), _joint6(base + 1.0)],
        }

    return {
        "schema": "robotics_lab.scope.v1",
        "n": len(times),
        "t_host_ns": times,
        "left": arm(0.0),
        "right": arm(10.0),
    }


class DebugPlotMathTests(unittest.TestCase):
    def test_finite_difference_uses_actual_dt(self) -> None:
        diff = finite_difference((0.0, 1.0, 3.0, 6.0), (0.0, 1.0, 2.0, 3.0))
        self.assertTrue(math.isnan(diff[0]))
        self.assertEqual(diff[1:], (1.0, 2.0, 3.0))

        variable_dt = finite_difference((0.0, 1.0, 4.0), (0.0, 0.5, 2.0))
        self.assertTrue(math.isnan(variable_dt[0]))
        self.assertEqual(variable_dt[1:], (2.0, 2.0))

    def test_moving_average_ignores_nan_gaps(self) -> None:
        smoothed = moving_average((1.0, math.nan, 5.0, 7.0), 3)
        self.assertEqual(smoothed[0], 1.0)
        self.assertEqual(smoothed[1], 1.0)
        self.assertEqual(smoothed[2], 3.0)
        self.assertEqual(smoothed[3], 6.0)

    def test_build_arm_series_three_traces_and_derivatives(self) -> None:
        samples = (
            ArmScopeSample(0.0, _joint6(0.0), _joint6(0.0), _joint6(1.0)),
            ArmScopeSample(1.0, _joint6(1.0), _joint6(2.0), _joint6(2.0)),
            ArmScopeSample(2.0, _joint6(3.0), _joint6(6.0), _joint6(3.0)),
            ArmScopeSample(3.0, _joint6(6.0), _joint6(12.0), _joint6(4.0)),
        )
        series = build_arm_series(
            samples, joint_index=0, window_sec=3.0, smooth=False
        )
        self.assertEqual(series.position.time_s, (-3.0, -2.0, -1.0, 0.0))
        self.assertEqual(series.position.sent, (0.0, 1.0, 3.0, 6.0))
        self.assertEqual(series.position.ref, (0.0, 2.0, 6.0, 12.0))
        self.assertEqual(series.position.actual, (1.0, 2.0, 3.0, 4.0))
        self.assertEqual(series.velocity.sent[1:], (1.0, 2.0, 3.0))
        self.assertEqual(series.velocity.ref[1:], (2.0, 4.0, 6.0))
        self.assertEqual(series.velocity.actual[1:], (1.0, 1.0, 1.0))
        self.assertAlmostEqual(series.rms_sent_actual_deg or 0.0, math.sqrt(1.5))
        self.assertAlmostEqual(series.sample_rate_hz or 0.0, 1.0)

    def test_build_debug_snapshot_windows_by_arm(self) -> None:
        samples = {
            "left": (
                ArmScopeSample(0.0, _joint6(0.0), _joint6(0.0), _joint6(0.0)),
                ArmScopeSample(1.0, _joint6(1.0), _joint6(1.0), _joint6(1.0)),
                ArmScopeSample(3.0, _joint6(3.0), _joint6(3.0), _joint6(3.0)),
            )
        }
        snap = build_debug_snapshot(
            samples, joint_index=0, window_sec=2.0, smooth=False
        )
        self.assertEqual(snap.arms["left"].position.time_s, (-2.0, 0.0))
        self.assertEqual(snap.arms["right"].sample_count, 0)


class ScopeReceiverTests(unittest.TestCase):
    def test_parse_scope_payload_parallel_arrays(self) -> None:
        batch = parse_scope_payload(_scope_payload())
        self.assertEqual(batch.n, 3)
        self.assertEqual(batch.t_host_ns[1], 1_002_000_000)
        self.assertEqual(batch.arms["left"].q_sent[2][0], 3.0)
        self.assertEqual(batch.arms["right"].q_ref[1][0], 10.5)

    def test_scope_store_buffers_batches_and_reports_stats(self) -> None:
        store = ScopeStore(history_sec=10.0)
        payload = json.dumps(_scope_payload()).encode("utf-8")
        self.assertTrue(store.update_from_json_bytes(payload, received_monotonic=100.0))

        samples = store.snapshot_samples()
        self.assertEqual(len(samples["left"]), 3)
        self.assertEqual(samples["left"][2].q_actual_deg[0], 1.0)
        stats = store.stats(now=100.1)
        self.assertEqual(stats.received_batches, 1)
        self.assertEqual(stats.invalid_packets, 0)
        self.assertEqual(stats.received_samples, 6)
        self.assertEqual(stats.buffer_samples["right"], 3)

    def test_scope_store_rejects_bad_schema(self) -> None:
        store = ScopeStore()
        bad = _scope_payload()
        bad["schema"] = "wrong"
        self.assertFalse(
            store.update_from_json_bytes(json.dumps(bad).encode("utf-8"))
        )
        self.assertEqual(store.stats().invalid_packets, 1)

    def test_scope_store_trims_history(self) -> None:
        store = ScopeStore(history_sec=1.0)
        first = _scope_payload(start_ns=0, step_ns=200_000_000)
        second = _scope_payload(start_ns=2_000_000_000, step_ns=200_000_000)
        store.update_from_json_bytes(json.dumps(first).encode("utf-8"))
        store.update_from_json_bytes(json.dumps(second).encode("utf-8"))
        samples = store.snapshot_samples()["left"]
        self.assertEqual(samples[0].time_s, 2.0)
        self.assertAlmostEqual(samples[-1].time_s, 2.4)


class DebugUplotConfigTests(unittest.TestCase):
    def test_cursor_sync_is_x_only(self) -> None:
        self.assertTrue(_debug_uplot_sync_supported())
        cursor = _debug_uplot_cursor()
        self.assertEqual(cursor["sync"]["key"], "rb_scope")
        self.assertEqual(cursor["sync"]["scales"], ("x", None))
        self.assertTrue(cursor["drag"]["x"])
        self.assertFalse(cursor["drag"]["y"])


class DebugVisibilityTests(unittest.TestCase):
    def test_debug_mode_hides_and_restores_panels_and_scene(self) -> None:
        normal = FakeHandle(True)
        debug_folder = FakeHandle(False)
        stand = FakeHandle(True)
        hidden_marker = FakeHandle(False)
        cloud = FakeHandle(True)
        left_urdf = FakeUrdf(True)
        right_ref = FakeUrdf(False)
        handles = {
            "debug_folder": debug_folder,
            "debug_normal_panel_handles": [normal],
            "scene": {
                "stand": stand,
                "hidden_marker": hidden_marker,
                "left_urdf": left_urdf,
                "right_urdf_ref": right_ref,
            },
            "pc_handle": cloud,
        }

        _set_debug_mode_active(handles, True)
        self.assertTrue(debug_folder.visible)
        self.assertFalse(normal.visible)
        self.assertFalse(stand.visible)
        self.assertFalse(hidden_marker.visible)
        self.assertFalse(cloud.visible)
        self.assertFalse(left_urdf.show_visual)
        self.assertFalse(right_ref.show_visual)

        _set_debug_mode_active(handles, False)
        self.assertFalse(debug_folder.visible)
        self.assertTrue(normal.visible)
        self.assertTrue(stand.visible)
        self.assertFalse(hidden_marker.visible)
        self.assertTrue(cloud.visible)
        self.assertTrue(left_urdf.show_visual)
        self.assertFalse(right_ref.show_visual)


if __name__ == "__main__":
    unittest.main()
