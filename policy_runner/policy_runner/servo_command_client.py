from __future__ import annotations

import json
import socket
import uuid
from dataclasses import dataclass
from typing import Any, Callable

from .robot_state_client import CommandSourceLeaseReadback, StateStreamLeaseReadback, parse_udp_endpoint


MOTION_MODES = {
    "ArmMotion",
    "JointTarget",
    "TcpPoseTarget",
    "TcpLinearMove",
}


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
    def gripper_target(
        cls,
        *,
        left: float | None = None,
        right: float | None = None,
        timeout_sec: float = 0.2,
    ) -> "CommandIntent":
        return cls(
            "Hold",
            timeout_sec=timeout_sec,
            left=_gripper_arm(left),
            right=_gripper_arm(right),
        )

    @classmethod
    def arm_motion(cls, timeout_sec: float = 0.2) -> "CommandIntent":
        return cls("ArmMotion", timeout_sec=timeout_sec)

    @classmethod
    def acquire_lease(cls, timeout_sec: float = 0.2) -> "CommandIntent":
        return cls("AcquireLease", timeout_sec=timeout_sec)

    @classmethod
    def release_lease(cls, timeout_sec: float = 0.2) -> "CommandIntent":
        return cls("ReleaseLease", timeout_sec=timeout_sec)

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
        joint_target_profile: str | None = None,
    ) -> "CommandIntent":
        return cls(
            "JointTarget",
            timeout_sec=timeout_sec,
            left=_joint_arm(
                "JointTarget",
                "q_target_deg",
                left,
                joint_target_profile=joint_target_profile,
            ),
            right=_joint_arm(
                "JointTarget",
                "q_target_deg",
                right,
                joint_target_profile=joint_target_profile,
            ),
        )

    @classmethod
    def init_motion(
        cls,
        *,
        left: list[float] | tuple[float, ...] | None = None,
        right: list[float] | tuple[float, ...] | None = None,
        timeout_sec: float = 0.2,
    ) -> "CommandIntent":
        return cls.joint_target(
            left=left,
            right=right,
            timeout_sec=timeout_sec,
            joint_target_profile="init_motion",
        )


@dataclass(frozen=True)
class LeaseAcquireResult:
    seq: int
    readback: CommandSourceLeaseReadback | None = None

    @property
    def lease_token(self) -> str | None:
        return None if self.readback is None else self.readback.active_lease_token


class ServoCommandClient:
    """UDP command sender compatible with rb_servo_server CommandServer."""

    def __init__(
        self,
        endpoint: str,
        timeout_sec: float = 0.2,
        socket_factory: Callable[..., socket.socket] = socket.socket,
        source_id: str = "policy_runner",
        session_id: str | None = None,
        packet_sink: Callable[[dict[str, Any]], None] | None = None,
    ):
        self.endpoint = endpoint
        self.timeout_sec = timeout_sec
        self.source_id = source_id
        self.session_id = session_id or uuid.uuid4().hex
        self.lease_token: str | None = None
        self._packet_sink = packet_sink
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
        if self._packet_sink is not None:
            self._packet_sink(payload)
        return self._seq

    def acquire_lease(
        self,
        readback: StateStreamLeaseReadback | None = None,
        *,
        timeout_sec: float = 1.0,
        poll_interval_sec: float = 0.01,
        monotonic_fn: Callable[[], float] | None = None,
        sleep_fn: Callable[[float], None] | None = None,
    ) -> LeaseAcquireResult:
        seq = self.send(CommandIntent.acquire_lease(timeout_sec=self.timeout_sec))
        if readback is None:
            return LeaseAcquireResult(seq=seq)

        kwargs: dict[str, Any] = {
            "source_id": self.source_id,
            "session_id": self.session_id,
            "timeout_sec": timeout_sec,
            "poll_interval_sec": poll_interval_sec,
        }
        if monotonic_fn is not None:
            kwargs["monotonic_fn"] = monotonic_fn
        if sleep_fn is not None:
            kwargs["sleep_fn"] = sleep_fn
        lease = readback.wait_for_active_lease(**kwargs)
        self.lease_token = lease.active_lease_token
        return LeaseAcquireResult(seq=seq, readback=lease)

    def release_lease(self) -> int:
        """Best-effort voluntary lease release (fire-and-forget UDP).

        Sent on shutdown so a restarted client does not collide with its own
        stale lease for the server's lease_timeout_sec window. The server only
        honors a release from the owning (source_id, session_id, token)."""
        try:
            return self.send(CommandIntent.release_lease(timeout_sec=self.timeout_sec))
        finally:
            self.lease_token = None

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
        if self.lease_token:
            packet["lease_token"] = self.lease_token
        return packet


def _arm_mode(value: dict[str, Any] | None) -> str | None:
    if not value:
        return None
    mode = value.get("mode")
    return str(mode) if mode is not None else None


def _joint_arm(
    mode: str,
    field: str,
    values: list[float] | tuple[float, ...] | None,
    *,
    joint_target_profile: str | None = None,
) -> dict[str, Any]:
    if values is None:
        return {"mode": "Hold"}
    if len(values) != 6:
        raise ValueError(f"{field} must contain 6 values")
    payload: dict[str, Any] = {"mode": mode, field: [float(v) for v in values]}
    if joint_target_profile is not None:
        payload["joint_target_profile"] = str(joint_target_profile)
    return payload


def _gripper_arm(value: float | None) -> dict[str, Any]:
    payload: dict[str, Any] = {"mode": "Hold"}
    if value is not None:
        payload["gripper_target"] = float(value)
    return payload
