from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import numpy as np

    from policy_runner.chunk_ensemble import (
        ChunkEnsembleScheduler,
        blend_weight,
        overlay_rows_with_runway,
    )
    from policy_runner.flow_dataset import pose_compose_local
except Exception:  # pragma: no cover - numpy/torch extras optional in some envs
    np = None


def _raw_chunk(horizon: int, left_step: np.ndarray, right_step: np.ndarray,
               grip_left: float = 30.0, grip_right: float = 40.0) -> np.ndarray:
    chunk = np.zeros((horizon, 14), dtype=np.float64)
    chunk[:, 0:6] = left_step
    chunk[:, 7:13] = right_step
    chunk[:, 6] = grip_left
    chunk[:, 13] = grip_right
    return chunk


def _integrate(anchor: np.ndarray, deltas: list[np.ndarray]) -> np.ndarray:
    cur = np.asarray(anchor, dtype=np.float64)
    for d in deltas:
        cur = np.asarray(pose_compose_local(cur, np.asarray(d, dtype=np.float64)), dtype=np.float64)
    return cur


@unittest.skipIf(np is None, "numpy extras not installed")
class ChunkEnsembleTest(unittest.TestCase):
    R = 6
    H = 24
    SEED = {
        "left": np.asarray([0.4, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
        "right": np.asarray([-0.4, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
    }

    def _anchor(self, arm: str) -> np.ndarray:
        return self.SEED[arm]

    def test_weights_are_seam_exact(self) -> None:
        self.assertEqual(blend_weight(0, self.R), 0.0)     # pure old at window start
        self.assertEqual(blend_weight(self.R - 1, self.R), 1.0)  # pure new at end

    def test_first_window_is_pure_2r(self) -> None:
        sched = ChunkEnsembleScheduler(self.R, self.H)
        step = np.asarray([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])
        raw = _raw_chunk(self.H, step, np.zeros(6))
        window = sched.begin(raw, self._anchor)
        self.assertEqual(window.shape, (2 * self.R, 14))
        self.assertEqual(sched.last_window_provenance, "pure-first")
        # integrating the synthetic deltas from the seed reproduces C1's plan
        end = _integrate(self.SEED["left"], [window[j][0:6] for j in range(2 * self.R)])
        expect = _integrate(self.SEED["left"], [step] * (2 * self.R))
        np.testing.assert_allclose(end, expect, atol=1e-9)
        # first (2R) window kicks C2 after consuming [0..R)
        self.assertEqual(sched.kick_index_for_active_window(), self.R)
        self.assertEqual(sched.window_start, 2 * self.R)
        # steady windows kick at their start
        sched.note_kick(self.R)
        c2 = _raw_chunk(self.H, step, np.zeros(6))
        self.assertIsNotNone(sched.advance(c2, self._anchor))
        self.assertEqual(sched.kick_index_for_active_window(), 0)

    def test_steady_blend_alignment_and_seams(self) -> None:
        R, H = self.R, self.H
        sched = ChunkEnsembleScheduler(R, H)
        step1 = np.asarray([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])   # C1: +x
        step2 = np.asarray([0.0, 0.01, 0.0, 0.0, 0.0, 0.0])   # C2: +y (divergent plan)
        c1 = _raw_chunk(H, step1, np.zeros(6), grip_left=20.0)
        c2 = _raw_chunk(H, step2, np.zeros(6), grip_left=80.0)

        sched.begin(c1, self._anchor)          # windows [0..12) pure C1
        sched.note_kick(R)                      # C2 kicked at wall 6 (first-window rule)
        window = sched.advance(c2, self._anchor)  # window [12..18)
        self.assertEqual(window.shape, (R, 14))
        self.assertEqual(sched.last_window_provenance, "blend")

        # C1's plan value at wall 11 (last of the pure window) from the seed:
        c1_abs = [self.SEED["left"]]
        for _ in range(H):
            c1_abs.append(_integrate(c1_abs[-1], [step1]))
        # C2 anchored at plan value of wall 5 (= kick 6 - 1): C1 abs index 5
        c2_anchor = c1_abs[6]  # c1_abs[0]=seed, so index i+1 = wall i target => wall5 -> [6]
        c2_abs = [c2_anchor]
        for _ in range(H):
            c2_abs.append(_integrate(c2_abs[-1], [step2]))

        # integrate executed stream: 12 pure steps + window deltas
        executed = [self.SEED["left"]]
        first_window_deltas = [step1] * (2 * R)
        for d in first_window_deltas:
            executed.append(_integrate(executed[-1], [d]))
        cur = executed[-1]
        targets = []
        for j in range(R):
            cur = _integrate(cur, [window[j][0:6]])
            targets.append(cur)
        # wall 12 target (j=0): pure C1 => c1_abs wall12 -> index 13
        np.testing.assert_allclose(targets[0], c1_abs[13], atol=1e-9)
        # wall 17 target (j=R-1): pure C2 => C2 index (17-6)=11 -> c2_abs[12]
        np.testing.assert_allclose(targets[-1], c2_abs[12], atol=1e-9)
        # interior is a genuine mix: x-progress between the two pure plans
        mid = targets[R // 2]
        self.assertGreater(mid[0], c2_abs[12][0] - 1e-12)   # more x than pure-C2 end
        self.assertLess(mid[0], c1_abs[13][0] + 1e-12)      # less x than pure-C1
        # grip lerp endpoints
        self.assertAlmostEqual(window[0][6], 20.0, places=9)
        self.assertAlmostEqual(window[R - 1][6], 80.0, places=9)

    def test_late_chunk_runs_on_old_runway_then_joins(self) -> None:
        R, H = self.R, self.H
        sched = ChunkEnsembleScheduler(R, H)
        c1 = _raw_chunk(H, np.asarray([0.01, 0, 0, 0, 0, 0]), np.zeros(6))
        sched.begin(c1, self._anchor)
        sched.note_kick(R)
        # boundary at wall 12 with NO new chunk -> pure old runway [12..18)
        window = sched.advance(None, self._anchor)
        self.assertIsNotNone(window)
        self.assertEqual(sched.last_window_provenance, "pure-new")
        # next boundary (wall 18): C1 covers [18..24) exactly (its last runway)
        window2 = sched.advance(None, self._anchor)
        self.assertIsNotNone(window2)
        # wall 24: C1 exhausted -> starved
        window3 = sched.advance(None, self._anchor)
        self.assertIsNone(window3)
        self.assertEqual(sched.last_window_provenance, "starved")
        # late C2 (kicked at wall 6) finally arrives: covers [24..30)? 24-6=18, 18+6=24<=24 OK
        c2 = _raw_chunk(H, np.asarray([0.0, 0.01, 0, 0, 0, 0]), np.zeros(6))
        window4 = sched.advance(c2, self._anchor)
        self.assertIsNotNone(window4)

    def test_runway_replays_next_no_new_window_with_grips(self) -> None:
        R, H = self.R, 30
        sched = ChunkEnsembleScheduler(R, H)
        step1 = np.asarray([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])
        step2 = np.asarray([0.0, 0.01, 0.0, 0.0, 0.0, 0.0])
        c1 = _raw_chunk(H, step1, np.zeros(6), grip_left=20.0, grip_right=30.0)
        c2 = _raw_chunk(H, step2, np.zeros(6), grip_left=80.0, grip_right=90.0)
        c2[:, 6] = np.arange(H, dtype=np.float64) + 100.0
        c2[:, 13] = np.arange(H, dtype=np.float64) + 200.0

        sched.begin(c1, self._anchor)
        sched.note_kick(R)
        window = sched.advance(c2, self._anchor)
        self.assertIsNotNone(window)
        self.assertEqual(sched.last_window_provenance, "blend")

        runway = sched.runway_segment()
        self.assertEqual(runway.shape, (R, 14))
        frame = overlay_rows_with_runway(window, scheduler=sched, stitch_mode="ensemble")
        self.assertEqual(frame.shape, (2 * R, 14))
        np.testing.assert_allclose(frame[R:], runway, atol=1e-7)

        next_window = sched.advance(None, self._anchor)
        self.assertIsNotNone(next_window)
        self.assertEqual(sched.last_window_provenance, "pure-new")
        np.testing.assert_allclose(runway, next_window[:R], atol=1e-7)
        np.testing.assert_allclose(runway[:, 6], c2[2 * R : 3 * R, 6], atol=1e-7)
        np.testing.assert_allclose(runway[:, 13], c2[2 * R : 3 * R, 13], atol=1e-7)

    def test_bootstrap_2r_overlay_frame_is_not_expanded(self) -> None:
        R, H = self.R, self.H
        sched = ChunkEnsembleScheduler(R, H)
        first = sched.begin(
            _raw_chunk(H, np.asarray([0.01, 0, 0, 0, 0, 0]), np.zeros(6)),
            self._anchor,
        )

        frame = overlay_rows_with_runway(first, scheduler=sched, stitch_mode="ensemble")

        self.assertEqual(frame.shape, (2 * R, 14))
        np.testing.assert_array_equal(frame, first)

    def test_runway_truncates_when_latest_plan_tail_is_short(self) -> None:
        R, H = self.R, 20
        sched = ChunkEnsembleScheduler(R, H)
        c1 = _raw_chunk(H, np.asarray([0.01, 0, 0, 0, 0, 0]), np.zeros(6))
        c1[:, 6] = np.arange(H, dtype=np.float64) + 10.0
        c1[:, 13] = np.arange(H, dtype=np.float64) + 20.0

        sched.begin(c1, self._anchor)
        window = sched.advance(None, self._anchor)
        self.assertIsNotNone(window)

        runway = sched.runway_segment()
        self.assertEqual(runway.shape, (H - 3 * R, 14))
        np.testing.assert_allclose(runway[:, 6], c1[3 * R :, 6], atol=1e-7)
        np.testing.assert_allclose(runway[:, 13], c1[3 * R :, 13], atol=1e-7)

    def test_boundary_overlay_rows_are_unchanged(self) -> None:
        R, H = self.R, self.H
        sched = ChunkEnsembleScheduler(R, H)
        sched.begin(_raw_chunk(H, np.asarray([0.01, 0, 0, 0, 0, 0]), np.zeros(6)), self._anchor)
        rows = np.arange(R * 14, dtype=np.float32).reshape(R, 14)

        frame = overlay_rows_with_runway(rows, scheduler=sched, stitch_mode="boundary")

        self.assertEqual(frame.shape, rows.shape)
        np.testing.assert_array_equal(frame, rows)

    def test_reset_reseeds_from_anchor_fn(self) -> None:
        R, H = self.R, self.H
        sched = ChunkEnsembleScheduler(R, H)
        c1 = _raw_chunk(H, np.asarray([0.01, 0, 0, 0, 0, 0]), np.zeros(6))
        sched.begin(c1, self._anchor)
        sched.reset()
        self.assertIsNone(sched.plan_tail_pose("left"))
        new_seed = {"left": np.asarray([0.9, 0.0, 0.5, 0, 0, 0, 1.0]),
                    "right": np.asarray([-0.9, 0.0, 0.5, 0, 0, 0, 1.0])}
        window = sched.begin(c1, lambda arm: new_seed[arm])
        end = _integrate(new_seed["left"], [window[j][0:6] for j in range(2 * R)])
        # synthetic chunks are float32 (pipeline contract) -> ~1e-7 rounding
        self.assertAlmostEqual(end[0], 0.9 + 0.01 * 2 * R, places=5)

    def test_blend_none_executes_pure_new_segment(self) -> None:
        R, H = self.R, self.H
        sched = ChunkEnsembleScheduler(R, H, blend_mode="none")
        step1 = np.asarray([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])
        step2 = np.asarray([0.0, 0.01, 0.0, 0.0, 0.0, 0.0])
        c1 = _raw_chunk(H, step1, np.zeros(6), grip_left=20.0)
        c2 = _raw_chunk(H, step2, np.zeros(6), grip_left=80.0)
        first = sched.begin(c1, self._anchor)
        sched.note_kick(R)
        window = sched.advance(c2, self._anchor)
        self.assertEqual(sched.last_window_provenance, "pure-new")
        # executed stream: 12 pure C1 steps, then the window
        cur = self.SEED["left"]
        for j in range(2 * R):
            cur = _integrate(cur, [first[j][0:6]])
        targets = []
        for j in range(R):
            cur = _integrate(cur, [window[j][0:6]])
            targets.append(cur)
        # C2 anchored at plan wall-5 value; its wall-12 target = c2 index 6
        c2_anchor = _integrate(self.SEED["left"], [step1] * R)
        c2_abs = [c2_anchor]
        for _ in range(H):
            c2_abs.append(_integrate(c2_abs[-1], [step2]))
        # j=0 is already PURE NEW (C2 wall-12 = index 6 -> c2_abs[7])
        np.testing.assert_allclose(targets[0], c2_abs[7], atol=1e-6)
        np.testing.assert_allclose(targets[-1], c2_abs[12], atol=1e-6)
        # grip switches to the new chunk immediately (no lerp)
        self.assertAlmostEqual(window[0][6], 80.0, places=5)
        with self.assertRaises(ValueError):
            ChunkEnsembleScheduler(R, H, blend_mode="cubic")

    def test_validation(self) -> None:
        with self.assertRaises(ValueError):
            ChunkEnsembleScheduler(6, 17)   # H < 3R
        with self.assertRaises(ValueError):
            ChunkEnsembleScheduler(1, 24)


if __name__ == "__main__":
    unittest.main()
