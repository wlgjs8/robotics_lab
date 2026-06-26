from __future__ import annotations

import json
import math
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping


RECORD_COMMAND_SCHEMA = "robotics_lab.record_cmd.v1"
RECORD_STATUS_SCHEMA = "robotics_lab.recording_state.v1"
ARM_INIT_COMMAND_SCHEMA = "robotics_lab.arm_init_cmd.v1"
ARM_INIT_STATE_SCHEMA = "robotics_lab.arm_init_state.v1"


def parse_udp_endpoint(raw: str | None, *, default_host: str, default_port: int) -> tuple[str, int]:
    text = (raw or "").strip()
    if text.startswith("udp://"):
        text = text[len("udp://") :]
    host, sep, port_text = text.rpartition(":")
    if not sep:
        return default_host, default_port
    try:
        port = int(port_text)
    except ValueError:
        return default_host, default_port
    if port <= 0 or port > 65535:
        return default_host, default_port
    return host or default_host, port


@dataclass(frozen=True)
class RecordingCommandResult:
    ok: bool
    message: str
    payload: dict[str, Any] | None = None


@dataclass(frozen=True)
class ArmInitCommandResult:
    ok: bool
    message: str
    payload: dict[str, Any] | None = None


class RecordingCommandClient:
    """UDP sender for policy_runner control-plane commands."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 50441,
        *,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self.endpoint = (host, int(port))
        self.seq = 0
        self.sent_packets: list[dict[str, Any]] = []
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, command: str, *, task: str = "", operator: str | None = None) -> RecordingCommandResult:
        if command not in {"start", "stop"}:
            return RecordingCommandResult(False, f"invalid recording command: {command}")
        self.seq += 1
        payload: dict[str, Any] = {
            "schema": RECORD_COMMAND_SCHEMA,
            "seq": self.seq,
            "command": command,
        }
        task_text = str(task or "").strip()
        operator_text = str(operator or "").strip()
        if task_text:
            payload["task"] = task_text
        if operator_text:
            payload["operator"] = operator_text
        try:
            self._socket.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), self.endpoint)
        except OSError as exc:
            return RecordingCommandResult(False, f"recording {command} send failed: {exc}", payload)
        self.sent_packets.append(payload)
        return RecordingCommandResult(True, f"recording {command} sent", payload)

    def send_arm_init(
        self,
        arms: str,
        *,
        action: str = "toggle",
        left_q_deg: tuple[float, ...] | list[float] | None = None,
        right_q_deg: tuple[float, ...] | list[float] | None = None,
    ) -> ArmInitCommandResult:
        if arms not in {"both", "left", "right"}:
            return ArmInitCommandResult(False, f"invalid arm init arms: {arms}")
        if action != "toggle":
            return ArmInitCommandResult(False, f"invalid arm init action: {action}")
        try:
            left = _finite_joint6(left_q_deg, "left_q_deg")
            right = _finite_joint6(right_q_deg, "right_q_deg")
        except ValueError as exc:
            return ArmInitCommandResult(False, str(exc))
        self.seq += 1
        payload: dict[str, Any] = {
            "schema": ARM_INIT_COMMAND_SCHEMA,
            "seq": self.seq,
            "arms": arms,
            "action": action,
        }
        if left is not None:
            payload["left_q_deg"] = left
        if right is not None:
            payload["right_q_deg"] = right
        try:
            self._socket.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), self.endpoint)
        except OSError as exc:
            return ArmInitCommandResult(False, f"arm init {arms} send failed: {exc}", payload)
        self.sent_packets.append(payload)
        return ArmInitCommandResult(True, f"arm init {arms} toggle sent", payload)

    def close(self) -> None:
        self._socket.close()


def _finite_joint6(value: tuple[float, ...] | list[float] | None, label: str) -> list[float] | None:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"{label} must contain 6 values")
    parsed = [float(item) for item in value]
    if any(not math.isfinite(item) for item in parsed):
        raise ValueError(f"{label} values must be finite")
    return parsed


def normalize_recording_status(block: Mapping[str, Any] | None) -> dict[str, Any]:
    if not isinstance(block, Mapping):
        block = {}
    state = str(block.get("state", "") or "")
    recording = bool(block.get("recording", state == "recording"))
    state = "recording" if recording else "idle"
    episode_name = str(block.get("episode_name", "") or "")
    try:
        frame_count = int(block.get("frame_count", 0) or 0)
    except (TypeError, ValueError):
        frame_count = 0
    try:
        rate_hz = float(block.get("rate_hz", 30.0) or 30.0)
    except (TypeError, ValueError):
        rate_hz = 30.0
    error = str(block.get("error", "") or "")
    last_command = str(block.get("last_command", "") or "")
    return {
        "recording": recording,
        "state": state,
        "episode_name": episode_name,
        "frame_count": max(0, frame_count),
        "rate_hz": rate_hz,
        "error": error,
        "last_command": last_command,
    }


def normalize_arm_init_status(block: Mapping[str, Any] | None) -> dict[str, Any]:
    if not isinstance(block, Mapping):
        block = {}
    left_on = bool(block.get("init_override_left", False))
    right_on = bool(block.get("init_override_right", False))
    error = str(block.get("error", "") or "")
    last_command = str(block.get("last_command", "") or "")
    return {
        "schema": str(block.get("schema", ARM_INIT_STATE_SCHEMA) or ARM_INIT_STATE_SCHEMA),
        "init_override_left": left_on,
        "init_override_right": right_on,
        "left_state": "init_motion" if left_on else "policy",
        "right_state": "init_motion" if right_on else "policy",
        "last_command": last_command,
        "error": error,
    }


class RecordingStatusStore:
    def __init__(self) -> None:
        self.received_packets = 0
        self.invalid_packets = 0
        self._latest: dict[str, Any] | None = None
        self._latest_arm_init: dict[str, Any] | None = None
        self._received_monotonic = float("-inf")
        self._lock = threading.Lock()

    def update(
        self,
        block: Mapping[str, Any],
        *,
        arm_init: Mapping[str, Any] | None = None,
        received_monotonic: float | None = None,
    ) -> None:
        status = normalize_recording_status(block)
        arm_init_status = normalize_arm_init_status(arm_init) if arm_init is not None else None
        with self._lock:
            self._latest = status
            if arm_init_status is not None:
                self._latest_arm_init = arm_init_status
            self._received_monotonic = time.monotonic() if received_monotonic is None else received_monotonic
            self.received_packets += 1

    def update_from_packet(self, data: bytes, *, received_monotonic: float | None = None) -> bool:
        try:
            payload = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            with self._lock:
                self.invalid_packets += 1
            return False
        if not isinstance(payload, Mapping) or payload.get("schema") != RECORD_STATUS_SCHEMA:
            with self._lock:
                self.invalid_packets += 1
            return False
        recording = payload.get("recording")
        if not isinstance(recording, Mapping):
            with self._lock:
                self.invalid_packets += 1
            return False
        arm_init = payload.get("arm_init")
        self.update(
            recording,
            arm_init=arm_init if isinstance(arm_init, Mapping) else None,
            received_monotonic=received_monotonic,
        )
        return True

    def latest(self) -> dict[str, Any] | None:
        with self._lock:
            return None if self._latest is None else dict(self._latest)

    def latest_arm_init(self) -> dict[str, Any] | None:
        with self._lock:
            return None if self._latest_arm_init is None else dict(self._latest_arm_init)

    def is_stale(self, *, now: float | None = None, threshold_sec: float = 1.5) -> bool:
        with self._lock:
            if self._latest is None:
                return True
            received = self._received_monotonic
        now_value = time.monotonic() if now is None else now
        return now_value - received > threshold_sec


class RecordingStatusReceiver:
    def __init__(
        self,
        store: RecordingStatusStore,
        *,
        host: str = "0.0.0.0",
        port: int = 50442,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self.store = store
        self.host = host
        self.port = int(port)
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((self.host, self.port))
        self._socket.settimeout(0.1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="recording-status-receiver", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._socket.close()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data, _addr = self._socket.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            self.store.update_from_packet(data)
