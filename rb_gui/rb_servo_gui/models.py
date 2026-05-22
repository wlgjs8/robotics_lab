from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping
import math
import time

DOF = 6


def finite_joint_array(value: Any) -> tuple[float, ...] | None:
    if not isinstance(value, list | tuple) or len(value) != DOF:
        return None
    out: list[float] = []
    for item in value:
        if not isinstance(item, int | float) or not math.isfinite(float(item)):
            return None
        out.append(float(item))
    return tuple(out)


@dataclass(frozen=True)
class ArmSnapshot:
    mode: str
    q_actual_deg: tuple[float, ...]
    q_sent_deg: tuple[float, ...]
    q_previous_sent_deg: tuple[float, ...]
    has_valid_joint_state: bool
    connection_state: str
    send_ok: bool
    send_duration_us: float | None = None

    @classmethod
    def parse(cls, data: Mapping[str, Any]) -> "ArmSnapshot | None":
        q_actual = finite_joint_array(data.get("q_actual_deg"))
        q_sent = finite_joint_array(data.get("q_sent_deg"))
        q_prev = finite_joint_array(data.get("q_previous_sent_deg"))
        if q_actual is None or q_sent is None or q_prev is None:
            return None
        if not bool(data.get("has_valid_joint_state", False)):
            return None
        return cls(
            mode=str(data.get("mode", "Unknown")),
            q_actual_deg=q_actual,
            q_sent_deg=q_sent,
            q_previous_sent_deg=q_prev,
            has_valid_joint_state=True,
            connection_state=str(data.get("connection_state", "Disconnected")),
            send_ok=bool(data.get("send_ok", False)),
            send_duration_us=float(data["send_duration_us"]) if isinstance(data.get("send_duration_us"), int | float) else None,
        )


@dataclass(frozen=True)
class StateSnapshot:
    tick: int
    received_monotonic: float
    left: ArmSnapshot
    right: ArmSnapshot
    motion_state: str
    safety_verdict: str
    fault_latched: bool
    fault_reason: str
    logger_health: Mapping[str, Any]
    mounts: Mapping[str, Any]
    raw: Mapping[str, Any]

    @classmethod
    def parse(cls, data: Mapping[str, Any], *, received_monotonic: float | None = None) -> "StateSnapshot | None":
        if int(data.get("schema_version", -1)) != 1:
            return None
        left_raw = data.get("left")
        right_raw = data.get("right")
        if not isinstance(left_raw, Mapping) or not isinstance(right_raw, Mapping):
            return None
        left = ArmSnapshot.parse(left_raw)
        right = ArmSnapshot.parse(right_raw)
        if left is None or right is None:
            return None
        tick = data.get("tick")
        if not isinstance(tick, int) or tick < 0:
            return None
        return cls(
            tick=tick,
            received_monotonic=time.monotonic() if received_monotonic is None else received_monotonic,
            left=left,
            right=right,
            motion_state=str(data.get("motion_state", "Disconnected")),
            safety_verdict=str(data.get("safety_verdict", "Unknown")),
            fault_latched=bool(data.get("fault_latched", False)),
            fault_reason=str(data.get("fault_reason", "")),
            logger_health=data.get("logger_health", {}) if isinstance(data.get("logger_health", {}), Mapping) else {},
            mounts=data.get("mounts", {}) if isinstance(data.get("mounts", {}), Mapping) else {},
            raw=data,
        )

    def stale(self, *, now: float | None = None, threshold_sec: float = 0.5) -> bool:
        now_value = time.monotonic() if now is None else now
        return now_value - self.received_monotonic > threshold_sec
