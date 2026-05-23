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
class Pose6D:
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    quaternion_xyzw: tuple[float, float, float, float] | None = None

    @classmethod
    def parse(cls, value: Any) -> "Pose6D | None":
        try:
            quaternion_xyzw = None
            if isinstance(value, Mapping):
                values = (value["x"], value["y"], value["z"], value["rx"], value["ry"], value["rz"])
                quaternion_xyzw = _parse_quaternion_xyzw(value)
            elif isinstance(value, list | tuple) and len(value) == 6:
                values = tuple(value)
            else:
                return None
            parsed = tuple(float(item) for item in values)
        except Exception:
            return None
        if not all(math.isfinite(item) for item in parsed):
            return None
        return cls(*parsed, quaternion_xyzw=quaternion_xyzw)

    def as_tuple(self) -> tuple[float, float, float, float, float, float]:
        return (self.x, self.y, self.z, self.rx, self.ry, self.rz)


def _parse_quaternion_xyzw(value: Mapping[str, Any]) -> tuple[float, float, float, float] | None:
    raw = value.get("quaternion_xyzw")
    if isinstance(raw, list | tuple) and len(raw) == 4:
        values = raw
    elif all(key in value for key in ("qx", "qy", "qz", "qw")):
        values = (value["qx"], value["qy"], value["qz"], value["qw"])
    else:
        return None
    try:
        parsed = tuple(float(item) for item in values)
    except Exception:
        return None
    if len(parsed) != 4 or not all(math.isfinite(item) for item in parsed):
        return None
    norm = math.sqrt(sum(item * item for item in parsed))
    if not math.isfinite(norm) or norm <= 0.0:
        return None
    return (parsed[0] / norm, parsed[1] / norm, parsed[2] / norm, parsed[3] / norm)


@dataclass(frozen=True)
class ArmSnapshot:
    mode: str
    q_actual_deg: tuple[float, ...] | None
    q_sent_deg: tuple[float, ...] | None
    q_previous_sent_deg: tuple[float, ...] | None
    has_valid_joint_state: bool
    connection_state: str
    send_ok: bool
    error_code: int | None = None
    tcp_stand: Pose6D | None = None
    tcp_base: Pose6D | None = None
    has_valid_tcp_pose: bool = False
    tcp_deferred: bool = True
    send_duration_us: float | None = None

    @classmethod
    def parse(cls, data: Mapping[str, Any]) -> "ArmSnapshot | None":
        q_actual = finite_joint_array(data.get("q_actual_deg"))
        q_sent = finite_joint_array(data.get("q_sent_deg"))
        q_prev = finite_joint_array(data.get("q_previous_sent_deg"))
        has_valid_joint_state = bool(data.get("has_valid_joint_state", False)) and q_actual is not None and q_sent is not None and q_prev is not None
        error_code = data.get("error_code")
        tcp_stand = Pose6D.parse(data.get("tcp_stand"))
        tcp_base = Pose6D.parse(data.get("tcp_base"))
        has_valid_tcp_pose_raw = data.get("has_valid_tcp_pose")
        has_valid_tcp_pose = bool(has_valid_tcp_pose_raw) if isinstance(has_valid_tcp_pose_raw, bool) else tcp_stand is not None
        tcp_deferred = bool(data.get("tcp_deferred", tcp_stand is None and tcp_base is None and not has_valid_tcp_pose))
        return cls(
            mode=str(data.get("mode", "Unknown")),
            q_actual_deg=q_actual,
            q_sent_deg=q_sent,
            q_previous_sent_deg=q_prev,
            has_valid_joint_state=has_valid_joint_state,
            connection_state=str(data.get("connection_state", "Disconnected")),
            send_ok=bool(data.get("send_ok", False)),
            error_code=int(error_code) if isinstance(error_code, int) else None,
            tcp_stand=tcp_stand,
            tcp_base=tcp_base,
            has_valid_tcp_pose=has_valid_tcp_pose and tcp_stand is not None,
            tcp_deferred=tcp_deferred,
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
