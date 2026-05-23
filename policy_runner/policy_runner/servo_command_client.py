from __future__ import annotations

import json
import socket
import uuid
from dataclasses import dataclass
from typing import Any, Callable

from .robot_state_client import parse_udp_endpoint


MOTION_MODES = {"ArmMotion", "JointTarget", "JointVelocity"}


@dataclass(frozen=True)
class CommandIntent:
    mode: str
    timeout_sec: float = 0.2
    left: dict[str, Any] | None = None
    right: dict[str, Any] | None = None
    coupled_timeout: bool = True

    @property
    def is_motion(self) -> bool:
        if self.mode in MOTION_MODES:
            return True
        return _arm_mode(self.left) in MOTION_MODES or _arm_mode(self.right) in MOTION_MODES

    @classmethod
    def hold(cls, timeout_sec: float = 0.2) -> "CommandIntent":
        return cls("Hold", timeout_sec=timeout_sec, left={}, right={})

    @classmethod
    def arm_motion(cls, timeout_sec: float = 0.2) -> "CommandIntent":
        return cls("ArmMotion", timeout_sec=timeout_sec)

    @classmethod
    def disarm_motion(cls, timeout_sec: float = 0.2) -> "CommandIntent":
        return cls("DisarmMotion", timeout_sec=timeout_sec)

    @classmethod
    def emergency_stop(cls, timeout_sec: float = 0.2) -> "CommandIntent":
        return cls("EmergencyStop", timeout_sec=timeout_sec)

    @classmethod
    def reset_fault(cls, timeout_sec: float = 0.2) -> "CommandIntent":
        return cls("ResetFault", timeout_sec=timeout_sec)

    @classmethod
    def joint_target(
        cls,
        *,
        left: list[float] | tuple[float, ...] | None = None,
        right: list[float] | tuple[float, ...] | None = None,
        timeout_sec: float = 0.2,
    ) -> "CommandIntent":
        return cls(
            "Hold",
            timeout_sec=timeout_sec,
            left=_joint_arm("JointTarget", "q_target_deg", left),
            right=_joint_arm("JointTarget", "q_target_deg", right),
        )

    @classmethod
    def joint_velocity(
        cls,
        *,
        left: list[float] | tuple[float, ...] | None = None,
        right: list[float] | tuple[float, ...] | None = None,
        timeout_sec: float = 0.2,
    ) -> "CommandIntent":
        return cls(
            "Hold",
            timeout_sec=timeout_sec,
            left=_joint_arm("JointVelocity", "dq_target_deg_s", left),
            right=_joint_arm("JointVelocity", "dq_target_deg_s", right),
        )


class ServoCommandClient:
    """UDP command sender compatible with rb_servo_server CommandServer."""

    def __init__(
        self,
        endpoint: str,
        timeout_sec: float = 0.2,
        socket_factory: Callable[..., socket.socket] = socket.socket,
        source_id: str = "policy_runner",
        session_id: str | None = None,
    ):
        self.endpoint = endpoint
        self.timeout_sec = timeout_sec
        self.source_id = source_id
        self.session_id = session_id or uuid.uuid4().hex
        self._seq = 0
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        parsed = parse_udp_endpoint(endpoint)
        self._address = (parsed.host, parsed.port)

    @property
    def next_seq(self) -> int:
        return self._seq + 1

    def close(self) -> None:
        self._socket.close()

    def send(self, intent: CommandIntent) -> int:
        self._seq += 1
        payload = self.build_packet(intent, self._seq)
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self._socket.sendto(data, self._address)
        return self._seq

    def build_packet(self, intent: CommandIntent, seq: int) -> dict[str, Any]:
        packet: dict[str, Any] = {
            "seq": seq,
            "mode": intent.mode,
            "timeout_sec": intent.timeout_sec or self.timeout_sec,
            "coupled_timeout": intent.coupled_timeout,
            "source_id": self.source_id,
            "session_id": self.session_id,
        }
        if intent.left is not None:
            packet["left"] = intent.left
        if intent.right is not None:
            packet["right"] = intent.right
        return packet


def _arm_mode(value: dict[str, Any] | None) -> str | None:
    if not value:
        return None
    mode = value.get("mode")
    return str(mode) if mode is not None else None


def _joint_arm(mode: str, field: str, values: list[float] | tuple[float, ...] | None) -> dict[str, Any]:
    if values is None:
        return {"mode": "Hold"}
    if len(values) != 6:
        raise ValueError(f"{field} must contain 6 values")
    return {"mode": mode, field: [float(v) for v in values]}
