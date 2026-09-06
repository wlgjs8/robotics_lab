"""Timestamped coordinator-command FK history, independent of policy commands.

``tcp_command_stand`` is FK of the coordinator's attempted q_sent target. It is
neither a controller ACK nor measured motion. Server loop-start timestamps and
Python monotonic time are comparable only on the same host. The UDP collector
requires a loopback sender; remote clocks are deliberately unsupported.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


SIDES = ("left", "right")


def _unsigned(value: Any, name: str, *, positive: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < int(positive):
        raise ValueError(f"missing_or_invalid_{name}")
    return value


def observation_stamp(payload: dict[str, Any]) -> tuple[int, int, int]:
    """Logical servo sample time, tick and motion epoch; never receipt-time fallback."""
    if not isinstance(payload, dict):
        raise ValueError("invalid_state_payload")
    return (
        _unsigned(payload.get("loop_start_time_ns"), "loop_start_time_ns", positive=True),
        _unsigned(payload.get("tick"), "tick"),
        _unsigned(payload.get("motion_epoch"), "motion_epoch"),
    )


def _arm_epoch(payload: dict[str, Any], side: str) -> int:
    arm = payload.get(side)
    fc = arm.get("force_control") if isinstance(arm, dict) else None
    return _unsigned(
        fc.get("reference_reset_count") if isinstance(fc, dict) else None,
        f"{side}_reference_reset_count",
    )


def _command_pose(payload: dict[str, Any], side: str) -> np.ndarray:
    arm = payload.get(side)
    raw = arm.get("tcp_command_stand") if isinstance(arm, dict) else None
    if not isinstance(raw, dict):
        raise ValueError(f"missing_{side}_tcp_command_stand")
    try:
        p = np.asarray([raw[axis] for axis in ("x", "y", "z")], dtype=np.float64)
        # The server publishes canonical xyzw. Requiring it here avoids mixing
        # legacy RPY and rotation-vector conventions in this new source.
        q = np.asarray(raw["quaternion_xyzw"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid_{side}_tcp_command_stand") from exc
    if p.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite(np.r_[p, q])):
        raise ValueError(f"invalid_{side}_tcp_command_stand")
    norm = float(np.linalg.norm(q))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError(f"invalid_{side}_command_quaternion")
    return np.r_[p, q / norm]


@dataclass(frozen=True)
class _Sample:
    time_ns: int
    tick: int
    motion_epoch: int
    reference_epoch: int
    pose: np.ndarray


class ServoCommandHistory:
    """Bounded per-packet history; sampling never reads past its frozen cutoff.

    The source timeout comes from the existing state client. Interpolation may
    span at most one requested policy period; larger gaps are not invented as a
    smooth trajectory. Epoch changes clear only the affected arm, except server
    restart/motion_epoch changes, which clear both.
    """

    def __init__(self, *, stale_timeout_sec: float, maxlen: int = 2048):
        if not np.isfinite(stale_timeout_sec) or stale_timeout_sec <= 0 or maxlen < 2:
            raise ValueError("history requires a positive stale timeout and maxlen >= 2")
        self.stale_timeout_sec = float(stale_timeout_sec)
        self._history: dict[str, deque[_Sample]] = {s: deque(maxlen=maxlen) for s in SIDES}
        self._lock = threading.Lock()
        self._latest_time_ns = 0
        self._latest_tick = -1
        self._motion_epoch: int | None = None
        self._reference_epoch: dict[str, int | None] = dict.fromkeys(SIDES)
        self._generation = 0
        self._last_rejection = "history_empty"

    @property
    def latest_time_ns(self) -> int:
        with self._lock:
            return self._latest_time_ns

    def _clear_locked(self, reason: str) -> None:
        for history in self._history.values():
            history.clear()
        self._last_rejection = reason

    def record(
        self, payload: dict[str, Any], received_monotonic: float, *, same_host_clock: bool,
    ) -> bool:
        """Record one actual server packet. Repeated/reordered samples add no time."""
        with self._lock:
            if not same_host_clock:
                self._clear_locked("unsupported_remote_clock_domain")
                return False
            try:
                stamp, tick, motion = observation_stamp(payload)
                if stamp <= self._latest_time_ns:
                    return False  # A delayed old UDP packet cannot clear newer live history.
                age = float(received_monotonic) - stamp * 1e-9
                if not np.isfinite(age) or age < -1e-6:
                    raise ValueError("server_clock_ahead_of_client")
                if age > self.stale_timeout_sec:
                    raise ValueError("stale_server_sample")
            except (TypeError, ValueError) as exc:
                self._clear_locked(str(exc))
                return False
            restarted = tick < self._latest_tick
            if restarted or motion != self._motion_epoch:
                self._clear_locked("server_restart" if restarted else "motion_epoch_changed")
                self._generation += 1
                self._reference_epoch = dict.fromkeys(SIDES)
            self._latest_time_ns, self._latest_tick, self._motion_epoch = stamp, tick, motion
            if payload.get("fault_latched") is True:
                self._clear_locked("server_fault_latched")
                return False
            added = False
            for side in SIDES:
                try:
                    epoch = _arm_epoch(payload, side)
                    pose = _command_pose(payload, side)
                except (TypeError, ValueError) as exc:
                    self._history[side].clear()
                    self._last_rejection = str(exc)
                    continue
                if epoch != self._reference_epoch[side]:
                    self._history[side].clear()
                    self._reference_epoch[side] = epoch
                self._history[side].append(_Sample(stamp, tick, motion, epoch, pose))
                added = True
            return added

    @staticmethod
    def _at(samples: list[_Sample], time_ns: int, max_gap_ns: int):
        for i, sample in enumerate(samples):
            if sample.time_ns == time_ns:
                return sample.pose.copy(), [sample.time_ns, sample.time_ns]
            if sample.time_ns > time_ns:
                if i == 0:
                    break
                previous = samples[i - 1]
                gap = sample.time_ns - previous.time_ns
                if gap > max_gap_ns:
                    raise ValueError("sparse_history_bracket")
                alpha = (time_ns - previous.time_ns) / gap
                position = previous.pose[:3] + alpha * (sample.pose[:3] - previous.pose[:3])
                rotations = Rotation.from_quat(np.stack([previous.pose[3:], sample.pose[3:]]))
                quaternion = Slerp([0.0, 1.0], rotations)([alpha]).as_quat()[0]
                return np.r_[position, quaternion], [previous.time_ns, sample.time_ns]
        raise ValueError("history_bracket_unavailable")

    def body_deltas(
        self, payload: dict[str, Any], *, policy_dt_sec: float,
        now_monotonic: float, not_before_ns: int = 0,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        """Incoming body deltas over [observation-policy_dt, observation], not SI velocity."""
        out = {side: np.zeros(6, dtype=np.float64) for side in SIDES}
        diag: dict[str, Any] = {
            "sample_mode": "fixed_step", "source": "servo_command",
            "pose_source": "tcp_command_stand",
            "command_semantics": "coordinator_attempted_q_sent_fk_not_controller_ack",
            "clock_domain": "same_host_monotonic",
            "time_source": "frozen_observation.loop_start_time_ns",
            "policy_dt_sec": float(policy_dt_sec), "scale": 1.0,
            "valid": False, "arms": {},
        }
        try:
            end, tick, motion = observation_stamp(payload)
            if not np.isfinite(policy_dt_sec) or policy_dt_sec <= 0:
                raise ValueError("invalid_policy_dt")
            window_ns = round(float(policy_dt_sec) * 1e9)
            start = end - window_ns
            age = float(now_monotonic) - end * 1e-9
            diag.update(window_start_time_ns=start, window_end_time_ns=end,
                        observation_tick=tick, motion_epoch=motion,
                        observation_age_ms=age * 1e3, not_before_time_ns=int(not_before_ns))
            if not np.isfinite(age) or age < -1e-6:
                raise ValueError("server_clock_ahead_of_client")
            if age > self.stale_timeout_sec:
                raise ValueError("stale_observation")
            if payload.get("fault_latched") is True:
                raise ValueError("server_fault_latched")
            if start < not_before_ns:
                raise ValueError("window_crosses_policy_reset")
            with self._lock:
                histories = {side: list(h) for side, h in self._history.items()}
                generation, latest_time, last_rejection = (
                    self._generation, self._latest_time_ns, self._last_rejection)
            diag.update(history_generation=generation, history_latest_time_ns=latest_time)
            for side in SIDES:
                try:
                    epoch = _arm_epoch(payload, side)
                    # Parsing the current payload too prevents a malformed observation
                    # from silently borrowing a valid pose from another packet.
                    _command_pose(payload, side)
                    samples = [s for s in histories[side] if s.time_ns <= end]
                    if not samples:
                        raise ValueError(last_rejection)
                    if any(s.motion_epoch != motion or s.reference_epoch != epoch for s in samples):
                        raise ValueError("observation_epoch_mismatch")
                    previous, previous_bracket = self._at(samples, start, window_ns)
                    current, current_bracket = self._at(samples, end, window_ns)
                    r_previous = Rotation.from_quat(previous[3:])
                    out[side] = np.r_[r_previous.inv().apply(current[:3] - previous[:3]),
                                     (r_previous.inv() * Rotation.from_quat(current[3:])).as_rotvec()]
                    diag["arms"][side] = {
                        "valid": True, "zero_reason": None, "reference_reset_count": epoch,
                        "start_bracket_time_ns": previous_bracket,
                        "end_bracket_time_ns": current_bracket,
                        "window_dt_sec": window_ns * 1e-9,
                        "start_pose": previous.tolist(), "end_pose": current.tolist(),
                        "delta": out[side].tolist(),
                    }
                except (KeyError, TypeError, ValueError) as exc:
                    diag["arms"][side] = {"valid": False, "zero_reason": str(exc)}
            diag["valid"] = all(diag["arms"][side]["valid"] for side in SIDES)
            diag["zero_reason"] = None if diag["valid"] else "one_or_more_servo_command_windows_unavailable"
        except (KeyError, TypeError, ValueError) as exc:
            diag["zero_reason"] = str(exc)
            diag["arms"] = {side: {"valid": False, "zero_reason": str(exc)} for side in SIDES}
        return out, diag
