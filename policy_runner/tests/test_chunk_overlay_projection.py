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
        anchor = np.asarray([0.40, -0.20, 0.30, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        expected0 = pose_compose_local(anchor, np.asarray([0.01, 0.0, 0.0, 0.0, 0.0, 0.0]))
        expected1 = pose_compose_local(expected0, np.asarray([0.0, 0.02, 0.0, 0.0, 0.0, 0.0]))
        np.testing.assert_allclose(packet["left"][0][:6], expected0[:6], atol=1e-7)
        np.testing.assert_allclose(packet["left"][1][:6], expected1[:6], atol=1e-7)
        self.assertEqual(packet["left"][0][6], 70.0)
        self.assertEqual(packet["left"][1][6], 60.0)


if __name__ == "__main__":
    unittest.main()
