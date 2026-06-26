from __future__ import annotations

import json
import sys
import unittest
import urllib.request
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rb_servo_gui.app import _set_debug_mode_active
from rb_servo_gui.scope_dashboard import ScopeDashboardServer, scope_delta_payload
from rb_servo_gui.scope_receiver import (
    ARMS,
    ScopeArmBatch,
    ScopeBatch,
    ScopeStore,
    parse_scope_payload,
)


class FakeHandle:
    def __init__(self, visible: bool = True) -> None:
        self.visible = visible


class FakeUrdf:
    def __init__(self, show_visual: bool = True) -> None:
        self.show_visual = show_visual


def _joint6(value: float) -> tuple[float, ...]:
    return tuple(value + idx for idx in range(6))


def _scope_payload(
    *, start_ns: int = 1_000_000_000, step_ns: int = 2_000_000
) -> dict:
    times = [start_ns + index * step_ns for index in range(3)]

    def arm(base: float) -> dict:
        return {
            "t_robot_ns": [10, 12, 14],
            "q_sent": [list(_joint6(base + 0.0)), list(_joint6(base + 1.0)), list(_joint6(base + 3.0))],
            "q_ref": [list(_joint6(base + 0.0)), list(_joint6(base + 0.5)), list(_joint6(base + 2.0))],
            "q_actual": [list(_joint6(base + 0.0)), list(_joint6(base + 0.25)), list(_joint6(base + 1.0))],
        }

    return {
        "schema": "robotics_lab.scope.v1",
        "n": len(times),
        "t_host_ns": times,
        "left": arm(0.0),
        "right": arm(10.0),
    }


def _batch(t_host_ns: tuple[int, ...]) -> ScopeBatch:
    def arm(base: float) -> ScopeArmBatch:
        return ScopeArmBatch(
            t_robot_ns=tuple(100 + index for index, _ in enumerate(t_host_ns)),
            q_sent=tuple(_joint6(base + index) for index, _ in enumerate(t_host_ns)),
            q_ref=tuple(_joint6(base + index + 0.25) for index, _ in enumerate(t_host_ns)),
            q_actual=tuple(_joint6(base + index + 0.5) for index, _ in enumerate(t_host_ns)),
        )

    return ScopeBatch(
        t_host_ns=t_host_ns,
        arms={
            "left": arm(0.0),
            "right": arm(10.0),
        },
    )


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


class ScopeDashboardTests(unittest.TestCase):
    def test_delta_payload_tracks_per_arm_cursors(self) -> None:
        store = ScopeStore(history_sec=10.0)
        store.append_batch(_batch((1_000_000_000, 1_002_000_000)))
        cursors = {arm: None for arm in ARMS}

        payload, cursors = scope_delta_payload(store, cursors)
        self.assertEqual(payload["schema"], "robotics_lab.scope.dashboard.v1")
        self.assertEqual(len(payload["arms"]["left"]["t"]), 2)
        self.assertEqual(len(payload["arms"]["right"]["q_sent"]), 2)

        payload, cursors = scope_delta_payload(store, cursors)
        self.assertEqual(payload["arms"]["left"]["t"], [])
        self.assertEqual(payload["arms"]["right"]["q_actual"], [])

        store.append_batch(_batch((1_004_000_000,)))
        payload, _ = scope_delta_payload(store, cursors)
        self.assertEqual(payload["arms"]["left"]["t"], [1.004])
        self.assertEqual(payload["arms"]["left"]["q_ref"][0][0], 0.25)

    def test_empty_store_payload_is_graceful(self) -> None:
        payload, cursors = scope_delta_payload(
            ScopeStore(), {arm: None for arm in ARMS}
        )
        self.assertEqual(payload["arms"]["left"]["t"], [])
        self.assertEqual(payload["arms"]["right"]["q_sent"], [])
        self.assertIn("stats", payload)
        self.assertEqual(set(cursors), set(ARMS))

    def test_delta_cursor_resets_after_timestamp_rollback(self) -> None:
        store = ScopeStore(history_sec=10.0)
        store.append_batch(_batch((2_000_000_000,)))
        payload, cursors = scope_delta_payload(store, {arm: None for arm in ARMS})
        self.assertEqual(payload["arms"]["left"]["t"], [2.0])

        store.append_batch(_batch((100_000_000,)))
        payload, _ = scope_delta_payload(store, cursors)
        self.assertEqual(payload["arms"]["left"]["t"], [0.1])
        self.assertEqual(payload["arms"]["right"]["t"], [0.1])

    def test_static_routes_set_expected_mime_types(self) -> None:
        server = ScopeDashboardServer(ScopeStore(), host="127.0.0.1", port=0).start()
        try:
            for route, content_type in (
                ("/", "text/html"),
                ("/uPlot.min.css", "text/css"),
                ("/uPlot.iife.min.js", "application/javascript"),
            ):
                with urllib.request.urlopen(server.url + route, timeout=2.0) as response:
                    self.assertEqual(response.status, 200)
                    self.assertEqual(response.headers.get_content_type(), content_type)
                    self.assertGreater(len(response.read(32)), 0)
        finally:
            server.stop()

    def test_sse_events_stream_empty_payload(self) -> None:
        server = ScopeDashboardServer(ScopeStore(), host="127.0.0.1", port=0).start()
        response = None
        try:
            response = urllib.request.urlopen(server.url + "/events", timeout=2.0)
            line = response.readline()
            self.assertTrue(line.startswith(b"data: "))
            payload = json.loads(line[len(b"data: ") :])
            self.assertEqual(payload["schema"], "robotics_lab.scope.dashboard.v1")
            self.assertEqual(payload["arms"]["left"]["t"], [])
            self.assertEqual(response.readline(), b"\n")
        finally:
            if response is not None:
                response.close()
            server.stop()


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
