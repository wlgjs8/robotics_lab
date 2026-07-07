from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import numpy as np

    from policy_runner.flow_dataset import pose_compose_local
    from policy_runner.flow_inference import FlowMatchingActionSource
except Exception:  # pragma: no cover - torch/numpy are optional in some envs
    np = None


class RecordingPublisher:
    def __init__(self) -> None:
        self.calls = []

    def publish(self, **kwargs) -> None:
        self.calls.append(kwargs)


@unittest.skipIf(np is None, "numpy/torch flow inference extras are not installed")
class ChunkOverlayProjectionTest(unittest.TestCase):
    def test_publish_chunk_overlay_integrates_from_measured_anchor(self) -> None:
        assert np is not None
        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        publisher = RecordingPublisher()
        source._chunk_overlay_publisher = publisher
        source._chunk_overlay_seq = 0
        source.policy_dt_sec = 0.05
        source.chunk_execute_steps = 2
        source.max_linear_velocity_m_s = 1.0
        source.max_angular_velocity_rad_s = 1.0
        source.arm_mask = np.asarray([1.0, 0.0], dtype=np.float32)
        source._chunk_crossfade_steps = 0
        source._steps_since_boundary = 0
        source._prev_emitted_twist_by_arm = {"left": None, "right": None}
        source._chunk = np.zeros((2, 14), dtype=np.float64)
        source._chunk[0, 0:7] = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0]
        source._chunk[1, 0:7] = [0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 60.0]
        source._last_overlay_payload = {
            "left": {
                "tcp_stand": {
                    "x": 0.40,
                    "y": -0.20,
                    "z": 0.30,
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                }
            },
            "right": {
                "tcp_stand": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                }
            },
        }

        source._publish_chunk_overlay(123.0)

        self.assertEqual(len(publisher.calls), 1)
        packet = publisher.calls[0]
        self.assertEqual(packet["seq"], 1)
        self.assertEqual(packet["policy_dt_sec"], 0.05)
        self.assertIsNone(packet["right"])
        self.assertIsNone(packet["right_delta"])
        anchor = np.asarray([0.40, -0.20, 0.30, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        expected0 = pose_compose_local(anchor, np.asarray([0.01, 0.0, 0.0, 0.0, 0.0, 0.0]))
        expected1 = pose_compose_local(expected0, np.asarray([0.0, 0.02, 0.0, 0.0, 0.0, 0.0]))
        self.assertEqual(len(packet["left"][0]), 8)
        self.assertEqual(len(packet["left"][1]), 8)
        np.testing.assert_allclose(packet["left"][0][:7], expected0[:7], atol=1e-7)
        np.testing.assert_allclose(packet["left"][1][:7], expected1[:7], atol=1e-7)
        self.assertEqual(packet["left"][0][7], 70.0)
        self.assertEqual(packet["left"][1][7], 60.0)
        self.assertEqual(len(packet["left_delta"]), 2)
        np.testing.assert_allclose(packet["left_delta"][0], [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0], atol=1e-7)
        np.testing.assert_allclose(packet["left_delta"][1], [0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 60.0], atol=1e-7)

    def test_publish_chunk_overlay_delta_rows_are_conditioned_deltas(self) -> None:
        assert np is not None
        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        publisher = RecordingPublisher()
        source._chunk_overlay_publisher = publisher
        source._chunk_overlay_seq = 0
        source.policy_dt_sec = 0.05
        source.chunk_execute_steps = 1
        source.max_linear_velocity_m_s = 0.2  # max per-frame translation = 0.01 m
        source.max_angular_velocity_rad_s = 1.0
        source.arm_mask = np.asarray([1.0, 0.0], dtype=np.float32)
        source._chunk_crossfade_steps = 0
        source._steps_since_boundary = 0
        source._prev_emitted_twist_by_arm = {"left": None, "right": None}
        source._chunk = np.zeros((1, 14), dtype=np.float64)
        source._chunk[0, 0:7] = [0.10, 0.0, 0.0, 0.0, 0.0, 0.0, 42.0]
        source._last_overlay_payload = {
            "left": {
                "tcp_stand": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                }
            },
            "right": {
                "tcp_stand": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                }
            },
        }

        source._publish_chunk_overlay(123.0)

        packet = publisher.calls[0]
        expected_delta = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
        np.testing.assert_allclose(packet["left_delta"][0][:6], expected_delta, atol=1e-7)
        expected_pose = pose_compose_local(
            np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            np.asarray(expected_delta),
        )
        np.testing.assert_allclose(packet["left"][0][:7], expected_pose[:7], atol=1e-7)
        self.assertEqual(packet["left_delta"][0][6], 42.0)

    def test_publish_chunk_overlay_chain_anchor_continues_from_prev_tail(self) -> None:
        assert np is not None
        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        publisher = RecordingPublisher()
        source._chunk_overlay_publisher = publisher
        source._chunk_overlay_seq = 0
        source.policy_dt_sec = 0.05
        source.chunk_execute_steps = 2
        source.max_linear_velocity_m_s = 1.0
        source.max_angular_velocity_rad_s = 1.0
        source.arm_mask = np.asarray([1.0, 0.0], dtype=np.float32)
        source._chunk_crossfade_steps = 0
        source._steps_since_boundary = 0
        source._prev_emitted_twist_by_arm = {"left": None, "right": None}
        source.chunk_anchor_source = "chain"
        payload = {
            "left": {"tcp_stand": {"x": 0.40, "y": -0.20, "z": 0.30, "quaternion_xyzw": [0, 0, 0, 1]}},
            "right": {"tcp_stand": {"x": 0.0, "y": 0.0, "z": 0.0, "quaternion_xyzw": [0, 0, 0, 1]}},
        }
        source._last_overlay_payload = payload

        chunk1 = np.zeros((2, 14), dtype=np.float64)
        chunk1[0, 0] = 0.01
        chunk1[1, 1] = 0.02
        source._chunk = chunk1
        source._publish_chunk_overlay(1.0)
        first_tail = np.asarray(publisher.calls[0]["left"][-1][:7], dtype=np.float64)
        np.testing.assert_allclose(publisher.calls[0]["left_delta"][0][:6], [0.01, 0, 0, 0, 0, 0], atol=1e-9)
        np.testing.assert_allclose(publisher.calls[0]["left_delta"][1][:6], [0, 0.02, 0, 0, 0, 0], atol=1e-9)

        # re-publishing the SAME chunk must not advance the chain (idempotent)
        source._publish_chunk_overlay(1.5)
        np.testing.assert_allclose(
            np.asarray(publisher.calls[1]["left"][0][:7]),
            np.asarray(publisher.calls[0]["left"][0][:7]),
            atol=1e-9,
        )

        # activating the NEXT chunk promotes the tail: chunk2 anchors on it, NOT
        # on the (unchanged) measured payload pose
        chunk2 = np.zeros((2, 14), dtype=np.float64)
        chunk2[0, 2] = 0.03
        source._overlay_chain_advance()
        source._chunk = chunk2
        source._publish_chunk_overlay(2.0)
        expected0 = pose_compose_local(first_tail, np.asarray([0.0, 0.0, 0.03, 0.0, 0.0, 0.0]))
        np.testing.assert_allclose(
            np.asarray(publisher.calls[2]["left"][0][:7]), expected0[:7], atol=1e-7
        )
        np.testing.assert_allclose(publisher.calls[2]["left_delta"][0][:6], [0, 0, 0.03, 0, 0, 0], atol=1e-9)

    def test_arm_init_reanchor_resets_chain_for_that_arm_only(self) -> None:
        assert np is not None
        from types import SimpleNamespace

        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        source._target_pose_by_arm = {"left": None, "right": None}
        source._gripper_targets_by_arm = {"left": 40.0, "right": 50.0}
        source._tcp_tp_conditioners = None
        left_tail = np.asarray([0.1, 0.2, 0.3, 0, 0, 0, 1.0])
        right_tail = np.asarray([0.4, 0.5, 0.6, 0, 0, 0, 1.0])
        source._overlay_chain_prev = {"left": left_tail, "right": right_tail}
        source._overlay_chain_pending = {"left": left_tail.copy(), "right": right_tail.copy()}
        rtc_reset = {"called": False}
        source.reset_rtc = lambda: rtc_reset.__setitem__("called", True)
        snapshot = SimpleNamespace(payload={
            "left": {"tcp_stand": {"x": 0.7, "y": 0.0, "z": 0.2, "quaternion_xyzw": [0, 0, 0, 1]}},
            "right": {"tcp_stand": {"x": -0.7, "y": 0.0, "z": 0.2, "quaternion_xyzw": [0, 0, 0, 1]}},
        })

        after = source._reanchor_arms_to_snapshot(("left",), snapshot)

        self.assertIsNotNone(after["left"])
        self.assertIsNone(source._overlay_chain_prev["left"])       # init arm: chain dropped
        self.assertIsNone(source._overlay_chain_pending["left"])
        self.assertIsNotNone(source._overlay_chain_prev["right"])   # other arm: chain kept
        np.testing.assert_allclose(source._overlay_chain_prev["right"], right_tail)
        self.assertTrue(rtc_reset["called"])                          # RTC prev cold-started


if __name__ == "__main__":
    unittest.main()
