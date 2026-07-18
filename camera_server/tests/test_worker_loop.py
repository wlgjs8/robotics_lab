#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import pathlib
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load_module(name: str, rel: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


worker = _load_module("stereo_worker_worker_loop", "stereo_worker/worker.py")


def _box(label="green"):
    T = np.eye(4)
    T[:3, 3] = (0.1, -0.7, 0.055)
    return {
        "T": T,
        "dims": (0.34, 0.20, 0.11),
        "footprint": (0.34, 0.20),
        "n": 900,
        "label": label,
        "fitness": 0.7,
        "rmse": 0.006,
    }


class LockStateTest(unittest.TestCase):
    def test_new_lock_state_initial_shape(self):
        self.assertEqual(
            worker._new_lock_state(),
            {
                "locked": False,
                "box": None,
                "lock_seq": 0,
                "lock_monotonic": None,
                "last_result": None,
            },
        )


class LockGateTest(unittest.TestCase):
    def test_evaluate_lock_gate_reasons(self):
        cases = [
            (None, (False, "reject_no_track")),
            ({"rmse": None, "fitness": 1.0}, (False, "reject_rmse")),
            ({"rmse": 0.013, "fitness": 1.0}, (False, "reject_rmse")),
            ({"rmse": 0.012, "fitness": None}, (False, "reject_fitness")),
            ({"rmse": 0.012, "fitness": 0.49}, (False, "reject_fitness")),
            ({"rmse": 0.012, "fitness": 0.5}, (True, "ok")),
        ]

        for candidate, expected in cases:
            with self.subTest(candidate=candidate):
                self.assertEqual(worker.evaluate_lock_gate(candidate, 0.012, 0.5), expected)

    def test_evaluate_lock_gate_checks_rmse_before_fitness(self):
        candidate = {"rmse": 0.013, "fitness": 0.49}

        self.assertEqual(worker.evaluate_lock_gate(candidate, 0.012, 0.5),
                         (False, "reject_rmse"))


class HeartbeatBoxesTest(unittest.TestCase):
    def test_build_heartbeat_boxes_outputs_only_locked_boxes_with_lock_fields(self):
        locks = {
            "green": {
                "locked": True,
                "box": _box("green"),
                "lock_seq": 3,
                "lock_monotonic": 8.25,
                "last_result": "ok",
            },
            "gray": worker._new_lock_state(),
        }

        boxes = worker.build_heartbeat_boxes(locks, now_monotonic=10.0)

        self.assertEqual(len(boxes), 1)
        self.assertEqual(boxes[0]["label"], "green")
        self.assertTrue(boxes[0]["locked"])
        self.assertEqual(boxes[0]["lock_seq"], 3)
        self.assertAlmostEqual(boxes[0]["lock_age_s"], 1.75)
        self.assertEqual(boxes[0]["n"], 900)

    def test_build_heartbeat_boxes_locked_without_monotonic_has_none_age(self):
        locks = {
            "green": {
                "locked": True,
                "box": _box("green"),
                "lock_seq": 1,
                "lock_monotonic": None,
                "last_result": "ok",
            },
        }

        boxes = worker.build_heartbeat_boxes(locks, now_monotonic=10.0)

        self.assertIsNone(boxes[0]["lock_age_s"])


class LockStatusTest(unittest.TestCase):
    def test_build_lock_status_includes_all_labels(self):
        locks = {
            "green": {
                "locked": True,
                "box": _box("green"),
                "lock_seq": 2,
                "lock_monotonic": 7.0,
                "last_result": "ok",
            },
            "gray": worker._new_lock_state(),
            "blue": {
                "locked": False,
                "box": None,
                "lock_seq": 0,
                "lock_monotonic": None,
                "last_result": "reject_rmse",
            },
        }

        status = worker.build_lock_status(locks, now_monotonic=10.5)

        self.assertEqual(set(status), {"green", "gray", "blue"})
        self.assertEqual(status["green"], {
            "locked": True,
            "lock_seq": 2,
            "lock_age_s": 3.5,
            "last_result": "ok",
        })
        self.assertEqual(status["gray"], {
            "locked": False,
            "lock_seq": 0,
            "lock_age_s": None,
            "last_result": None,
        })
        self.assertEqual(status["blue"], {
            "locked": False,
            "lock_seq": 0,
            "lock_age_s": None,
            "last_result": "reject_rmse",
        })


class PublishBoxesLockTelemetryTest(unittest.TestCase):
    class FakeSocket:
        def __init__(self):
            self.parts = None

        def send_multipart(self, parts):
            self.parts = parts

    def _publisher(self):
        pub = worker.CloudPublisher.__new__(worker.CloudPublisher)
        pub._json = json
        pub._sock = self.FakeSocket()
        return pub

    def _payload(self, pub):
        self.assertEqual(pub._sock.parts[0], b"stereo.boxes")
        return json.loads(pub._sock.parts[1].decode())

    def test_publish_boxes_without_lock_args_has_backward_compatible_payload(self):
        pub = self._publisher()

        pub.publish_boxes(7, [_box("gray")])

        payload = self._payload(pub)
        self.assertNotIn("phase", payload)
        self.assertNotIn("locks", payload)

    def test_publish_boxes_adds_phase_locks_and_box_lock_fields_when_present(self):
        pub = self._publisher()
        box = _box("gray")
        box.update({"locked": True, "lock_seq": 2, "lock_age_s": 1.5})
        locks = {
            "gray": {
                "locked": True,
                "lock_seq": 2,
                "lock_age_s": 1.5,
                "last_result": "ok",
            },
        }

        pub.publish_boxes(9, [box], phase="burst", locks=locks)

        payload = self._payload(pub)
        self.assertEqual(payload["phase"], "burst")
        self.assertEqual(payload["locks"], locks)
        encoded = payload["boxes"][0]
        self.assertIs(encoded["locked"], True)
        self.assertEqual(encoded["lock_seq"], 2)
        self.assertEqual(encoded["lock_age_s"], 1.5)


if __name__ == "__main__":
    unittest.main()
