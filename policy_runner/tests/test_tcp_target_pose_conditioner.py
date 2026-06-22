from __future__ import annotations

import unittest

import numpy as np

from policy_runner.action_sources.tcp_delta import tcp_pose_target_stand_intent
from policy_runner.flow_dataset import pose_compose_local
from policy_runner.tcp_target_pose_conditioner import (
    OnlineTcpTargetPoseConditioner,
    interp_pose,
)

DT = 0.02  # 50 Hz policy step
RATE = 500.0
TICK = 1.0 / RATE


def _pose(x: float, qz: float = 0.0) -> np.ndarray:
    q = np.array([0.0, 0.0, qz, np.sqrt(max(0.0, 1.0 - qz * qz))])
    return np.array([x, 0.0, 0.2, *q], dtype=np.float64)


def _targets(anchor: np.ndarray, deltas: list[np.ndarray]) -> np.ndarray:
    out = []
    cur = anchor
    for d in deltas:
        cur = np.asarray(pose_compose_local(cur, d), dtype=np.float64)
        out.append(cur)
    return np.asarray(out, dtype=np.float64)


class InterpPoseTest(unittest.TestCase):
    def test_endpoints_and_midpoint(self) -> None:
        p0 = _pose(0.0)
        p1 = _pose(0.1)
        np.testing.assert_allclose(interp_pose(p0, p1, 0.0), p0, atol=1e-9)
        np.testing.assert_allclose(interp_pose(p0, p1, 1.0), p1, atol=1e-9)
        np.testing.assert_allclose(interp_pose(p0, p1, 0.5)[0], 0.05, atol=1e-9)

    def test_quaternion_sign_continuity(self) -> None:
        p0 = _pose(0.0, qz=0.0)
        p1 = _pose(0.0, qz=0.3)
        p1[3:7] = -p1[3:7]  # opposite hemisphere, same rotation
        mid = interp_pose(p0, p1, 0.5)
        # slerp must take the short way -> w stays positive, no 180 flip
        self.assertGreater(mid[6], 0.9)


class ConditionerFohTest(unittest.TestCase):
    def _make(self, reanchor="measured_blend", blend_steps=2):
        return OnlineTcpTargetPoseConditioner(
            mode="foh_se3", reanchor_mode=reanchor, policy_dt_sec=DT,
            servo_rate_hz=RATE, blend_steps=blend_steps,
        )

    def test_intermediate_ticks_differ_between_two_source_samples(self) -> None:
        cond = self._make()
        anchor = _pose(0.0)
        deltas = [np.array([0.01, 0, 0, 0, 0, 0]) for _ in range(4)]
        tgt = _targets(anchor, deltas)
        cond.begin_chunk(measured_anchor=anchor, measured_targets=tgt,
                         continuous_targets=None, t_wall_start=0.0, chunk_id=0)
        # ticks within the first policy interval [0, DT)
        xs = [cond.sample(k * TICK).pose[0] for k in range(0, 10)]
        # consecutive 500 Hz ticks emit DIFFERENT (interpolated) x, not a held value
        diffs = np.diff(xs)
        self.assertTrue(np.all(diffs > 1e-6), f"expected strictly increasing interp, got {xs}")

    def test_hits_exact_source_targets_at_sample_times(self) -> None:
        cond = self._make(reanchor="measured_legacy")
        anchor = _pose(0.0)
        deltas = [np.array([0.01, 0, 0, 0, 0, 0.05]) for _ in range(3)]
        tgt = _targets(anchor, deltas)
        cond.begin_chunk(measured_anchor=anchor, measured_targets=tgt,
                         continuous_targets=None, t_wall_start=0.0, chunk_id=0)
        # knot j is at t = j*DT: knot0=anchor, knot1=tgt[0], knot2=tgt[1], knot3=tgt[2]
        np.testing.assert_allclose(cond.sample(0.0).pose, anchor, atol=1e-6)
        np.testing.assert_allclose(cond.sample(1 * DT).pose, tgt[0], atol=1e-6)
        np.testing.assert_allclose(cond.sample(2 * DT).pose, tgt[1], atol=1e-6)
        np.testing.assert_allclose(cond.sample(3 * DT).pose, tgt[2], atol=1e-6)

    def test_reanchor_flag_only_on_first_sample(self) -> None:
        cond = self._make()
        anchor = _pose(0.0)
        tgt = _targets(anchor, [np.array([0.01, 0, 0, 0, 0, 0])])
        cond.begin_chunk(measured_anchor=anchor, measured_targets=tgt,
                         continuous_targets=None, t_wall_start=0.0, chunk_id=7)
        first = cond.sample(0.0)
        self.assertTrue(first.reanchor)
        self.assertEqual(first.chunk_id, 7)
        self.assertFalse(cond.sample(TICK).reanchor)

    def test_measured_blend_no_one_tick_jump_across_boundary(self) -> None:
        cond = self._make(reanchor="measured_blend", blend_steps=4)
        # chunk 1
        a0 = _pose(0.0)
        d = [np.array([0.01, 0, 0, 0, 0, 0]) for _ in range(4)]
        t1 = _targets(a0, d)
        cond.begin_chunk(measured_anchor=a0, measured_targets=t1,
                         continuous_targets=None, t_wall_start=0.0, chunk_id=0)
        last = None
        for k in range(0, int(round(4 * DT * RATE)) + 1):
            last = cond.sample(k * TICK)
        last_emitted = last.pose.copy()
        # chunk 2 boundary: measured anchor LAGS the last emitted target by 8 mm (drift gap)
        a1 = last_emitted.copy()
        a1[0] -= 0.008
        measured2 = _targets(a1, d)
        continuous2 = _targets(last_emitted, d)
        t_start = (4 * DT)
        cond.begin_chunk(measured_anchor=a1, measured_targets=measured2,
                         continuous_targets=continuous2, t_wall_start=t_start, chunk_id=1)
        # first tick of the new chunk must NOT jump back to the lagging measured anchor
        prev = last_emitted
        max_step = 0.0
        for k in range(0, int(round(4 * DT * RATE)) + 1):
            ct = cond.sample(t_start + k * TICK)
            step = float(np.linalg.norm(ct.pose[:3] - prev[:3]))
            max_step = max(max_step, step)
            prev = ct.pose
        # per-tick linear step must stay small (well under a 2 mm/tick cap)
        self.assertLess(max_step, 0.002, f"max per-tick step {max_step*1000:.3f} mm too large")

    def test_last_emitted_continuous_has_no_boundary_jump(self) -> None:
        cond = self._make(reanchor="last_emitted_continuous")
        a0 = _pose(0.0)
        d = [np.array([0.01, 0, 0, 0, 0, 0]) for _ in range(3)]
        cond.begin_chunk(measured_anchor=a0, measured_targets=_targets(a0, d),
                         continuous_targets=None, t_wall_start=0.0, chunk_id=0)
        last = None
        for k in range(0, int(round(3 * DT * RATE)) + 1):
            last = cond.sample(k * TICK)
        last_emitted = last.pose.copy()
        a1 = last_emitted.copy()
        a1[0] -= 0.02  # big measured drift; continuous mode must ignore it
        cont2 = _targets(last_emitted, d)
        cond.begin_chunk(measured_anchor=a1, measured_targets=_targets(a1, d),
                         continuous_targets=cont2, t_wall_start=3 * DT, chunk_id=1)
        first = cond.sample(3 * DT)
        np.testing.assert_allclose(first.pose, last_emitted, atol=1e-6)

    def test_stall_hold_reemits_last_target(self) -> None:
        cond = self._make()
        a0 = _pose(0.0)
        cond.begin_chunk(measured_anchor=a0, measured_targets=_targets(a0, [np.array([0.01, 0, 0, 0, 0, 0])]),
                         continuous_targets=None, t_wall_start=0.0, chunk_id=0)
        emitted = cond.sample(DT).pose.copy()
        held = cond.hold()
        self.assertTrue(held.hold)
        self.assertTrue(held.stall)
        np.testing.assert_allclose(held.pose, emitted, atol=1e-9)

    def test_twist_estimate_present_when_interpolating(self) -> None:
        cond = self._make(reanchor="measured_legacy")
        a0 = _pose(0.0)
        tgt = _targets(a0, [np.array([0.02, 0, 0, 0, 0, 0]), np.array([0.02, 0, 0, 0, 0, 0])])
        cond.begin_chunk(measured_anchor=a0, measured_targets=tgt,
                         continuous_targets=None, t_wall_start=0.0, chunk_id=0)
        ct = cond.sample(0.5 * DT)
        self.assertIsNotNone(ct.twist)
        # x velocity ~ 0.02 m / DT
        self.assertAlmostEqual(ct.twist[0], 0.02 / DT, places=3)


class TcpPoseTargetTwistPayloadTest(unittest.TestCase):
    def test_twist_attached_only_when_supplied(self) -> None:
        with_twist = tcp_pose_target_stand_intent(
            left=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
            left_twist=[0.05, 0.0, 0.0, 0.0, 0.0, 0.1],
        )
        self.assertEqual(with_twist.left["tcp_target_twist_stand"], [0.05, 0.0, 0.0, 0.0, 0.0, 0.1])
        # Backward compatible: no twist key when not supplied (existing clients).
        without = tcp_pose_target_stand_intent(left=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
        self.assertNotIn("tcp_target_twist_stand", without.left)

    def test_bad_twist_length_rejected(self) -> None:
        with self.assertRaises(ValueError):
            tcp_pose_target_stand_intent(
                left=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0], left_twist=[0.05, 0.0]
            )


if __name__ == "__main__":
    unittest.main()
