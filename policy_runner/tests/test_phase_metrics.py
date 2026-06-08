from __future__ import annotations

import unittest
from dataclasses import dataclass
from pathlib import Path

try:
    import numpy as np
except ModuleNotFoundError as exc:  # pragma: no cover - exercised in minimal CI images.
    raise unittest.SkipTest("numpy is required for phase metric tests") from exc

from policy_runner.flow_dataset import FlowSampleRef
from policy_runner.phase_segmentation import extract_phase_boundaries
from policy_runner import imitation_experiments as ie


@dataclass
class _Episode:
    path: Path
    length: int
    left_gripper: np.ndarray
    right_gripper: np.ndarray
    timestamps: np.ndarray


class _Dataset:
    def __init__(self, episode: _Episode, starts: list[int]):
        self.episodes = [episode]
        self.sample_refs = [FlowSampleRef(0, start) for start in starts]
        self.action_horizon = 1
        self._cache = {}

    def phase_boundaries_for_episode(self, episode_index: int):
        episode = self.episodes[episode_index]
        key = str(episode.path)
        if key not in self._cache:
            self._cache[key] = extract_phase_boundaries(
                episode.left_gripper,
                episode.right_gripper,
                episode.length,
            )
        return self._cache[key]


def _signals(length: int = 100) -> tuple[np.ndarray, np.ndarray]:
    left = np.full(length, 90.0, dtype=np.float64)
    right = np.full(length, 90.0, dtype=np.float64)
    right[10:30] = 10.0
    left[60:80] = 10.0
    return left, right


class PhaseMetricTests(unittest.TestCase):
    def test_boundary_extraction_and_phase_assignment(self) -> None:
        left, right = _signals()
        boundaries = extract_phase_boundaries(left, right, len(left))
        self.assertTrue(boundaries.clean)
        self.assertEqual((boundaries.b1, boundaries.b2, boundaries.b3, boundaries.b4), (10, 30, 60, 80))
        self.assertEqual(boundaries.phase_for_frame(9), "right_pick")
        self.assertEqual(boundaries.phase_for_frame(10), "right_place")
        self.assertEqual(boundaries.phase_for_frame(30), "left_pick")
        self.assertEqual(boundaries.phase_for_frame(60), "left_place")

        episode = _Episode(Path("episode.hdf5"), len(left), left, right, np.arange(len(left)) / 30.0)
        dataset = _Dataset(episode, [0, 12, 35, 70])
        self.assertEqual([ie._sample_phase(dataset, i) for i in range(4)], [
            "right_pick",
            "right_place",
            "left_pick",
            "left_place",
        ])

    def test_active_inactive_arm_metric_routing(self) -> None:
        stats = {"action_std": [1.0] * 14}
        metrics = ie._empty_metric_accumulator()
        pred = np.zeros((2, 14), dtype=np.float64)
        target = np.zeros((2, 14), dtype=np.float64)
        mask = np.ones((2, 14), dtype=np.float64)
        pred[0, 0:3] = [3.0, 4.0, 0.0]
        pred[1, 0:3] = [0.0, 0.0, 12.0]
        pred[:, 7:14] = 1.0

        ie._accumulate_raw_metrics(metrics, pred, target, mask, phase="right_pick")
        finalized = ie._finalize_metrics(metrics, stats=stats)
        phase = finalized["by_phase"]["right_pick"]
        self.assertAlmostEqual(phase["active_arm_action_mse"]["value"], 1.0)
        self.assertAlmostEqual(phase["inactive_arm_pred_motion"]["value"], 8.5)
        self.assertAlmostEqual(phase["inactive_arm_action_mse"]["value"], 169.0 / 14.0)

    def test_ordered_timing_and_critical_instant_metrics(self) -> None:
        left, right = _signals()
        length = len(left)
        episode = _Episode(Path("episode.hdf5"), length, left, right, np.arange(length) / 30.0)
        dataset = _Dataset(episode, list(range(length)))
        ordered = ie._empty_ordered_eval_accumulator(dataset)
        # The action-chunk gripper dim is a DELTA (target minus the current
        # observation gripper). The metric reconstructs the absolute predicted
        # gripper as obs + delta, so build the deltas from a desired absolute
        # predicted gripper trajectory that closes/opens a few frames late.
        pred_abs_left = np.full(length, 90.0, dtype=np.float64)
        pred_abs_right = np.full(length, 90.0, dtype=np.float64)
        pred_abs_right[13:33] = 10.0
        pred_abs_left[64:82] = 10.0

        for frame in range(length):
            pred = np.zeros((1, 14), dtype=np.float64)
            target = np.zeros((1, 14), dtype=np.float64)
            pred[0, 6] = pred_abs_left[frame] - left[frame]
            pred[0, 13] = pred_abs_right[frame] - right[frame]
            if frame == 10:
                pred[0, 7:10] = [1.0, 2.0, 2.0]
            ie._accumulate_ordered_prediction(ordered, dataset, frame, pred, target)

        event_metrics = ie._finalize_ordered_event_metrics(ordered, dataset)
        timing = event_metrics["gripper_event_timing"]["events"]
        self.assertAlmostEqual(timing["right_close"]["mean_ms"], 100.0)
        self.assertAlmostEqual(timing["right_open"]["mean_ms"], 100.0)
        self.assertAlmostEqual(timing["left_close"]["mean_ms"], 4000.0 / 30.0)
        self.assertEqual(timing["left_open"]["missed_event_rate"], 0.0)
        critical = event_metrics["critical_instant_endpoint_error"]["events"]
        self.assertAlmostEqual(critical["right_close"]["translation_error_mean"], 3.0)


if __name__ == "__main__":
    unittest.main()
