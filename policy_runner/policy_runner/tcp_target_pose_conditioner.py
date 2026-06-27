"""Online command-conditioning for the policy ``tcp_target_pose`` rollout path.

This is the online command-conditioning layer for the imitation-rollout consumer of
``TcpPoseTarget``. The policy emits one absolute stand-frame target per ~30 Hz
chunk step; without conditioning those step targets are held (ZOH)
between policy ticks, so the 500 Hz SMD reference generator (B-stage) sees a 30 Hz
staircase. This module turns the per-step targets into a smooth 500 Hz SE(3) stream by
first-order-hold interpolation (position lerp + quaternion slerp), with explicit
chunk-boundary reanchor / blend and stall handling.

It is deliberately **torch-free** (numpy + scipy only) and self-contained so the policy
runtime path can be unit-tested without the ML extras. The absolute target *composition*
(ee_local delta -> stand pose) stays in ``flow_inference``; this module only interpolates,
blends, and holds the already-composed per-step targets.

Pose convention everywhere: ``(7,)`` = ``[x, y, z, qx, qy, qz, qw]`` (scipy xyzw) in the
stand frame, matching ``flow_dataset.pose_compose_local`` / ``pose_from_state_payload``.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from policy_runner.flow_dataset import pose_delta_local

CONDITIONING_MODES = ("legacy_step_hold", "foh_se3")
REANCHOR_MODES = ("measured_legacy", "last_emitted_continuous", "measured_blend")

_ZERO6 = np.zeros(6, dtype=np.float64)


def _quat_canonical(q: np.ndarray, ref: np.ndarray | None = None) -> np.ndarray:
    """Unit xyzw quaternion in a consistent hemisphere (dot with ref >= 0)."""
    q = np.asarray(q, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    q = q / n if n > 0 else np.asarray([0.0, 0.0, 0.0, 1.0])
    if ref is not None and float(np.dot(q, np.asarray(ref, dtype=np.float64).reshape(4))) < 0.0:
        q = -q
    return q


def interp_pose(p0: np.ndarray, p1: np.ndarray, u: float) -> np.ndarray:
    """SE(3) first-order hold: linear position lerp + quaternion slerp at u in [0,1]."""
    p0 = np.asarray(p0, dtype=np.float64).reshape(7)
    p1 = np.asarray(p1, dtype=np.float64).reshape(7)
    u = float(min(1.0, max(0.0, u)))
    pos = (1.0 - u) * p0[:3] + u * p1[:3]
    q0 = _quat_canonical(p0[3:7])
    q1 = _quat_canonical(p1[3:7], ref=q0)
    dot = float(np.dot(q0, q1))
    if dot > 0.9995:  # nearly parallel -> nlerp (numerically safe)
        q = _quat_canonical((1.0 - u) * q0 + u * q1)
    else:
        theta = np.arccos(max(-1.0, min(1.0, dot)))
        s = np.sin(theta)
        q = (np.sin((1.0 - u) * theta) / s) * q0 + (np.sin(u * theta) / s) * q1
        q = _quat_canonical(q)
    out = np.empty(7, dtype=np.float64)
    out[:3] = pos
    out[3:7] = q
    return out


def _blend_pose(a: np.ndarray, b: np.ndarray, beta: float) -> np.ndarray:
    """Blend pose a -> b by beta (same SE(3) interpolation as interp_pose)."""
    return interp_pose(a, b, beta)


def _pose_twist(p_lo: np.ndarray, p_hi: np.ndarray, dt: float) -> np.ndarray:
    """Episode-time conditioned twist [v(3), w(3)] from the FOH knot pair."""
    if dt <= 0.0:
        return _ZERO6.copy()
    v = (np.asarray(p_hi[:3], dtype=np.float64) - np.asarray(p_lo[:3], dtype=np.float64)) / dt
    r_lo = Rotation.from_quat(_quat_canonical(p_lo[3:7]))
    r_hi = Rotation.from_quat(_quat_canonical(p_hi[3:7], ref=p_lo[3:7]))
    w = (r_lo.inv() * r_hi).as_rotvec() / dt
    return np.concatenate([v, w]).astype(np.float64)


@dataclass
class ConditionedTarget:
    pose: np.ndarray            # (7,) absolute stand-frame TcpPoseTarget
    hold: bool                  # re-emitting a held target (boundary stall / clamp end)
    stall: bool                 # next chunk was not ready when this was emitted
    dropout: bool               # source dropout (reserved; policy path has none today)
    reanchor: bool              # first emitted target of a freshly begun chunk
    chunk_id: int
    chunk_index_lo: int         # source step index below the current wall time (-1 = anchor)
    chunk_index_hi: int
    interpolation_alpha: float
    twist: np.ndarray | None    # (6,) estimated conditioned twist, episode-time
    emitted_delta_from_prev: np.ndarray  # (6,) pose_delta_local(prev_emitted, pose)


class OnlineTcpPoseTargetConditioner:
    """Per-arm online FOH conditioner for streamed ``tcp_target_pose`` rollout.

    Lifecycle per chunk: ``begin_chunk(...)`` with the per-step absolute targets, then
    ``sample(t_wall)`` once per 500 Hz tick. When the next chunk is not ready at the
    boundary, the caller invokes ``hold()`` to re-emit the last target.
    """

    def __init__(
        self,
        *,
        mode: str = "foh_se3",
        reanchor_mode: str = "measured_blend",
        policy_dt_sec: float,
        servo_rate_hz: float = 500.0,
        blend_steps: int = 2,
    ) -> None:
        if mode not in CONDITIONING_MODES:
            raise ValueError(f"unknown tcp_target_pose conditioning mode: {mode}")
        if reanchor_mode not in REANCHOR_MODES:
            raise ValueError(f"unknown reanchor_mode: {reanchor_mode}")
        if policy_dt_sec is None or float(policy_dt_sec) <= 0.0:
            raise ValueError("policy_dt_sec must be positive")
        self.mode = mode
        self.reanchor_mode = reanchor_mode
        self.policy_dt_sec = float(policy_dt_sec)
        self.servo_rate_hz = float(servo_rate_hz)
        self.blend_steps = max(1, int(blend_steps))
        self._prev_emitted: np.ndarray | None = None
        self._knots: np.ndarray | None = None
        self._times: np.ndarray | None = None
        self._chunk_id = -1
        self._first_sample = False

    @property
    def last_emitted(self) -> np.ndarray | None:
        """The last emitted absolute target pose (7,), or None before the first sample.

        Used by the runtime to seed continuity / measured_blend at chunk boundaries."""
        if self._prev_emitted is None:
            return None
        return self._prev_emitted.copy()

    def reset(self) -> None:
        self._prev_emitted = None
        self._knots = None
        self._times = None
        self._chunk_id = -1
        self._first_sample = False

    def begin_chunk(
        self,
        *,
        measured_anchor: np.ndarray,
        measured_targets: np.ndarray,
        continuous_targets: np.ndarray | None,
        t_wall_start: float,
        chunk_id: int,
    ) -> None:
        """Install a chunk's per-step absolute targets as FOH knots.

        ``measured_targets[k]`` = absolute target k anchored at the live measured pose
        (drift-correcting). ``continuous_targets[k]`` = the same deltas composed onto the
        last emitted target (drift-free continuity); pass ``None`` on the first chunk.
        Knots are ``[P, S_0 .. S_{K-1}]`` at times ``t_wall_start + j*policy_dt``.
        """

        measured_targets = np.asarray(measured_targets, dtype=np.float64).reshape(-1, 7)
        k = measured_targets.shape[0]
        if k == 0:
            raise ValueError("measured_targets must contain at least one step")
        first_chunk = self._prev_emitted is None
        if first_chunk or self.reanchor_mode == "measured_legacy":
            anchor = np.asarray(measured_anchor, dtype=np.float64).reshape(7)
            body = measured_targets
        elif self.reanchor_mode == "last_emitted_continuous":
            anchor = self._prev_emitted.copy()
            body = _require_continuous(continuous_targets, k)
        else:  # measured_blend
            anchor = self._prev_emitted.copy()
            cont = _require_continuous(continuous_targets, k)
            body = np.empty_like(measured_targets)
            for i in range(k):
                beta = min(1.0, float(i + 1) / float(self.blend_steps))
                body[i] = _blend_pose(cont[i], measured_targets[i], beta)
        knots = np.vstack([anchor.reshape(1, 7), body])
        self._knots = knots
        self._times = float(t_wall_start) + self.policy_dt_sec * np.arange(knots.shape[0], dtype=np.float64)
        self._chunk_id = int(chunk_id)
        self._first_sample = True

    def sample(self, t_wall: float) -> ConditionedTarget:
        if self._knots is None or self._times is None:
            pose = self._prev_emitted if self._prev_emitted is not None else np.asarray([0, 0, 0, 0, 0, 0, 1.0])
            return self._emit(pose, hold=True, stall=True, lo=-1, hi=-1, alpha=0.0, dt=0.0)
        t = float(min(self._times[-1], max(self._times[0], t_wall)))
        m = self._times.size
        lo = int(np.clip(np.searchsorted(self._times, t, side="right") - 1, 0, m - 1))
        hi = min(lo + 1, m - 1)
        denom = float(self._times[hi] - self._times[lo])
        alpha = (t - float(self._times[lo])) / denom if denom > 0 else 0.0
        pose = interp_pose(self._knots[lo], self._knots[hi], alpha)
        at_end = lo == hi
        return self._emit(pose, hold=at_end, stall=False, lo=lo, hi=hi, alpha=alpha, dt=denom)

    def hold(self) -> ConditionedTarget:
        """Re-emit the last target (next chunk not ready). Keeps the accumulator."""
        pose = self._prev_emitted if self._prev_emitted is not None else np.asarray([0, 0, 0, 0, 0, 0, 1.0])
        return self._emit(pose, hold=True, stall=True, lo=-1, hi=-1, alpha=0.0, dt=0.0)

    def _emit(self, pose, *, hold, stall, lo, hi, alpha, dt) -> ConditionedTarget:
        pose = np.asarray(pose, dtype=np.float64).reshape(7)
        if self._prev_emitted is not None:
            delta = np.asarray(pose_delta_local(self._prev_emitted, pose), dtype=np.float64).reshape(6)
        else:
            delta = _ZERO6.copy()
        twist = None
        if not hold and self._knots is not None and lo >= 0 and hi >= 0 and lo != hi:
            twist = _pose_twist(self._knots[lo], self._knots[hi], dt)
        reanchor = bool(self._first_sample and not stall)
        self._first_sample = False
        self._prev_emitted = pose.copy()
        return ConditionedTarget(
            pose=pose,
            hold=bool(hold),
            stall=bool(stall),
            dropout=False,
            reanchor=reanchor,
            chunk_id=int(self._chunk_id),
            chunk_index_lo=int(lo) - 1,
            chunk_index_hi=int(hi) - 1,
            interpolation_alpha=float(alpha),
            twist=twist,
            emitted_delta_from_prev=delta,
        )


def _require_continuous(continuous_targets: np.ndarray | None, k: int) -> np.ndarray:
    if continuous_targets is None:
        raise ValueError("continuous_targets is required for this reanchor_mode after the first chunk")
    arr = np.asarray(continuous_targets, dtype=np.float64).reshape(-1, 7)
    if arr.shape[0] != k:
        raise ValueError("continuous_targets must match measured_targets length")
    return arr
