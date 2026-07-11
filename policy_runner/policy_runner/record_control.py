from __future__ import annotations

import json
import os
import socket
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, TextIO

from .config import PolicyRunnerConfig
from .robot_state_client import StateSnapshot, parse_udp_endpoint

# Korea Standard Time (UTC+9, no DST): recorded-episode session folders are
# stamped in KST so the folder name matches the operator's `make run` wall clock.
_KST = timezone(timedelta(hours=9))


def _session_dir_name(now: datetime | None = None) -> str:
    """Per-run episode folder name: ``data_<KST YYYYMMDD_HHMMSS>``.

    Computed once at session start (~ ``make run`` time) so every episode
    recorded in one run is grouped under the same folder.
    """
    stamp = (now or datetime.now(_KST)).strftime("%Y%m%d_%H%M%S")
    return f"data_{stamp}"


RECORD_COMMAND_SCHEMA = "robotics_lab.record_cmd.v1"
RECORD_STATUS_SCHEMA = "robotics_lab.recording_state.v1"


@dataclass(frozen=True)
class RecordCommand:
    command: str
    task: str = ""
    operator: str | None = None


def parse_record_command(data: bytes | dict[str, Any]) -> RecordCommand:
    payload = parse_control_payload(data)
    if not isinstance(payload, dict):
        raise ValueError("record command must be a JSON object")
    if payload.get("schema") != RECORD_COMMAND_SCHEMA:
        raise ValueError("unsupported record command schema")
    command = payload.get("command")
    if command not in {"start", "stop"}:
        raise ValueError("record command must be start or stop")
    task = payload.get("task", "")
    operator = payload.get("operator")
    if task is None:
        task = ""
    if not isinstance(task, str):
        raise ValueError("record command task must be a string")
    if operator is not None and not isinstance(operator, str):
        raise ValueError("record command operator must be a string")
    return RecordCommand(command=command, task=task, operator=operator or None)


def parse_control_payload(data: bytes | dict[str, Any]) -> dict[str, Any]:
    if isinstance(data, bytes):
        try:
            payload = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("control command must be JSON") from exc
    else:
        payload = data
    if not isinstance(payload, dict):
        raise ValueError("control command must be a JSON object")
    return payload


class RecordControlServer:
    """Non-blocking UDP receiver for rb_gui episode start/stop commands."""

    def __init__(
        self,
        bind: str,
        *,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        endpoint = parse_udp_endpoint(bind)
        self.bind = bind
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((endpoint.host, endpoint.port))
        self._socket.setblocking(False)

    def drain_payloads(self, *, max_packets: int = 32) -> list[dict[str, Any]]:
        payloads: list[dict[str, Any]] = []
        for _ in range(max(1, int(max_packets))):
            try:
                data, _addr = self._socket.recvfrom(65536)
            except BlockingIOError:
                break
            except OSError:
                break
            try:
                payloads.append(parse_control_payload(data))
            except ValueError:
                continue
        return payloads

    def drain(self, *, max_packets: int = 32) -> list[RecordCommand]:
        commands: list[RecordCommand] = []
        for payload in self.drain_payloads(max_packets=max_packets):
            try:
                commands.append(parse_record_command(payload))
            except ValueError:
                continue
        return commands

    def close(self) -> None:
        self._socket.close()


class RecordStatusPublisher:
    """Best-effort UDP publisher for GUI recording status."""

    def __init__(
        self,
        endpoint: str,
        *,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        parsed = parse_udp_endpoint(endpoint)
        self.endpoint = endpoint
        self._address = (parsed.host, parsed.port)
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(
        self,
        recording: dict[str, Any],
        *,
        arm_init: Mapping[str, Any] | None = None,
        runner_role: str = "unknown",
        action_source: str = "",
        command_source_id: str = "",
        command_session_id: str = "",
        control_endpoint: str | None = None,
        status_endpoint: str | None = None,
        spacemouse: Mapping[str, Any] | None = None,
        force_recovery: Mapping[str, Any] | None = None,
        camera_runtime: Mapping[str, Any] | None = None,
    ) -> None:
        payload = {
            "schema": RECORD_STATUS_SCHEMA,
            "host_time_ns": time.time_ns(),
            "recording": recording,
            "runner_role": runner_role,
            "action_source": action_source,
            "command_source_id": command_source_id,
            "command_session_id": command_session_id,
            "control_endpoint": control_endpoint,
            "status_endpoint": status_endpoint or self.endpoint,
        }
        if arm_init is not None:
            payload["arm_init"] = dict(arm_init)
        if spacemouse is not None:
            payload["spacemouse"] = dict(spacemouse)
        if force_recovery is not None:
            payload["force_recovery"] = dict(force_recovery)
        if camera_runtime is not None:
            payload["camera_runtime"] = dict(camera_runtime)
        try:
            self._socket.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), self._address)
        except OSError:
            pass

    def close(self) -> None:
        self._socket.close()


class RecordingSupervisor:
    """Owns the live HDF5 episode recorder inside policy_runner.run()."""

    def __init__(
        self,
        config: PolicyRunnerConfig,
        *,
        control_server: RecordControlServer | None = None,
        status_publisher: RecordStatusPublisher | None = None,
        camera_client_factory: Callable[..., Any] | None = None,
        recorder_factory: Callable[..., Any] | None = None,
        session_dir_name: str | None = None,
        stderr: TextIO | None = None,
    ) -> None:
        self.config = config
        self.control_server = control_server
        self.status_publisher = status_publisher
        self.camera_client_factory = camera_client_factory
        self.recorder_factory = recorder_factory
        self.stderr = stderr
        self.camera_client: Any | None = None
        self.recorder: Any | None = None
        self.episode_name = ""
        self.frame_count = 0
        self.error = ""
        self.last_command = ""
        self._last_status_publish = float("-inf")
        self._last_status: dict[str, Any] = {"recording": self.status_block(), "arm_init": None}
        # One folder per run, fixed at session start (~ `make run` time, KST):
        # data_<YYYYMMDD_HHMMSS>. Every episode recorded this session is written
        # here as episode_000.hdf5, episode_001.hdf5, ... run_stack.sh exports the
        # exact make-run timestamp via RB_RECORD_SESSION_DIR; a direct invocation
        # falls back to KST now. The folder itself is created lazily by the
        # recorder on the first episode, so an idle run leaves no empty folder.
        session_name = (
            session_dir_name
            or os.environ.get("RB_RECORD_SESSION_DIR")
            or _session_dir_name()
        )
        self.session_output_dir = Path(self.config.recording.output_dir) / session_name

    @classmethod
    def from_config(cls, config: PolicyRunnerConfig, *, stderr: TextIO | None = None) -> "RecordingSupervisor":
        control: RecordControlServer | None = None
        publisher: RecordStatusPublisher | None = None
        error = ""
        if config.recording.control_enabled:
            try:
                control = RecordControlServer(config.recording.control_bind)
            except Exception as exc:  # noqa: BLE001 - recording must not block teleop startup
                error = f"control_bind_failed:{type(exc).__name__}"
                if stderr is not None:
                    print(
                        f"policy_runner recording control disabled: {exc}",
                        file=stderr,
                    )
        if config.recording.status_endpoint:
            try:
                publisher = RecordStatusPublisher(config.recording.status_endpoint)
            except Exception as exc:  # noqa: BLE001 - status is best effort
                if stderr is not None:
                    print(
                        f"policy_runner recording status disabled: {exc}",
                        file=stderr,
                    )
        supervisor = cls(config, control_server=control, status_publisher=publisher, stderr=stderr)
        supervisor.error = error
        return supervisor

    @property
    def recording(self) -> bool:
        active = self.recorder is not None and bool(getattr(self.recorder, "has_active_episode", False))
        return active

    def drain_commands(self, snapshot: StateSnapshot, *, action_source: str) -> None:
        self.dispatch_control_payloads(
            self.drain_control_payloads(),
            snapshot,
            action_source=action_source,
        )

    def drain_control_payloads(self, *, max_packets: int = 32) -> list[dict[str, Any]]:
        if self.control_server is None:
            return []
        return self.control_server.drain_payloads(max_packets=max_packets)

    def dispatch_control_payloads(
        self,
        payloads: Iterable[dict[str, Any]],
        snapshot: StateSnapshot,
        *,
        action_source: str,
    ) -> None:
        for payload in payloads:
            try:
                command = parse_record_command(payload)
            except ValueError:
                continue
            self.handle_command(command, snapshot, action_source=action_source)

    def handle_command(self, command: RecordCommand, snapshot: StateSnapshot, *, action_source: str) -> None:
        self.last_command = command.command
        if command.command == "start":
            self.start(snapshot, task=command.task, operator=command.operator, action_source=action_source)
        elif command.command == "stop":
            self.stop()

    def start(
        self,
        snapshot: StateSnapshot,
        *,
        task: str,
        operator: str | None,
        action_source: str,
    ) -> None:
        if self.recording:
            return
        self.error = ""
        try:
            self._ensure_recorder()
            assert self.recorder is not None
            self.recorder.start_episode(
                reset_snapshot=snapshot,
                task_description=task,
                action_source=action_source,
                operator_id=operator,
                dataset_metadata=self.config.recording.dataset_metadata,
            )
            self.episode_name = str(getattr(self.recorder, "episode_id", "") or "")
            self.frame_count = int(getattr(self.recorder, "frame_count", 0) or 0)
        except Exception as exc:  # noqa: BLE001 - recording failures must not affect command flow
            self.error = f"start_failed:{type(exc).__name__}"
            if self.stderr is not None:
                print(f"policy_runner recording start failed: {exc}", file=self.stderr)
            self._close_recorder_resources()

    def record_frame(
        self,
        snapshot: StateSnapshot,
        *,
        action_packet: dict[str, Any] | None,
        action_host_time_ns: int | None,
        action_seq: int | None,
    ) -> None:
        if not self.recording or self.recorder is None:
            return
        try:
            self.recorder.record_frame(
                state_snapshot=snapshot,
                action_packet=action_packet,
                action_host_time_ns=action_host_time_ns,
                action_seq=action_seq,
            )
            self.frame_count = int(getattr(self.recorder, "frame_count", self.frame_count) or 0)
            self.episode_name = str(getattr(self.recorder, "episode_id", self.episode_name) or "")
        except Exception as exc:  # noqa: BLE001 - keep teleop alive, surface status
            self.error = f"record_failed:{type(exc).__name__}"
            if self.stderr is not None:
                print(f"policy_runner recording frame failed: {exc}", file=self.stderr)

    def stop(self) -> None:
        if not self.recording:
            self._close_recorder_resources()
            return
        try:
            assert self.recorder is not None
            self.recorder.close()
        except Exception as exc:  # noqa: BLE001 - surface status and reset to idle
            self.error = f"stop_failed:{type(exc).__name__}"
            if self.stderr is not None:
                print(f"policy_runner recording stop failed: {exc}", file=self.stderr)
        finally:
            self._close_recorder_resources()

    def stamp_snapshot(self, snapshot: StateSnapshot) -> None:
        snapshot.payload["recording"] = self.status_block()

    def status_block(self) -> dict[str, Any]:
        return {
            "recording": self.recording,
            "state": "recording" if self.recording else "idle",
            "episode_name": self.episode_name,
            "frame_count": int(self.frame_count),
            "rate_hz": float(self.config.recording.rate_hz),
            "last_command": self.last_command,
            "error": self.error,
        }

    def publish_status(
        self,
        *,
        now_monotonic: float,
        force: bool = False,
        arm_init: Mapping[str, Any] | None = None,
        runner_role: str = "unknown",
        action_source: str = "",
        command_client: Any | None = None,
        spacemouse: Mapping[str, Any] | None = None,
        force_recovery: Mapping[str, Any] | None = None,
        camera_runtime: Mapping[str, Any] | None = None,
    ) -> None:
        status = self.status_block()
        combined_status: dict[str, Any] = {
            "recording": status,
            "arm_init": dict(arm_init) if arm_init is not None else None,
            "runner_role": runner_role,
            "action_source": action_source,
            "command_source_id": str(getattr(command_client, "source_id", "") or ""),
            "command_session_id": str(getattr(command_client, "session_id", "") or ""),
            "control_endpoint": self.control_server.bind if self.control_server is not None else None,
            "status_endpoint": self.config.recording.status_endpoint,
            "spacemouse": dict(spacemouse) if spacemouse is not None else None,
            "force_recovery": dict(force_recovery) if force_recovery is not None else None,
            "camera_runtime": dict(camera_runtime) if camera_runtime is not None else None,
        }
        changed = combined_status != self._last_status
        period = 1.0 / max(float(self.config.recording.status_rate_hz), 1.0)
        due = now_monotonic - self._last_status_publish >= period
        if self.status_publisher is not None and (force or changed or due):
            self.status_publisher.publish(
                status,
                arm_init=arm_init,
                runner_role=runner_role,
                action_source=action_source,
                command_source_id=combined_status["command_source_id"],
                command_session_id=combined_status["command_session_id"],
                control_endpoint=combined_status["control_endpoint"],
                status_endpoint=combined_status["status_endpoint"],
                spacemouse=spacemouse,
                force_recovery=force_recovery,
                camera_runtime=camera_runtime,
            )
            self._last_status_publish = now_monotonic
            self._last_status = combined_status

    def close(self) -> None:
        self.stop()
        if self.control_server is not None:
            self.control_server.close()
            self.control_server = None
        if self.status_publisher is not None:
            self.status_publisher.close()
            self.status_publisher = None

    def _ensure_recorder(self) -> None:
        if self.recorder is not None:
            return
        if self.camera_client_factory is None:
            from .camera_bundle_client import CameraBundleClient

            self.camera_client_factory = CameraBundleClient
        if self.recorder_factory is None:
            from .recording import Hdf5EpisodeRecorder

            self.recorder_factory = Hdf5EpisodeRecorder
        self.camera_client = self.camera_client_factory(
            zmq_endpoint=self.config.camera.zmq_endpoint,
            topic=self.config.camera.bundle_topic,
            max_age_ms=self.config.camera.max_age_ms,
            include_depth=True,
        )
        self.recorder = self.recorder_factory(
            self.session_output_dir,
            recording_rate_hz=self.config.recording.rate_hz,
            camera_client=self.camera_client,
            expected_cameras=self.config.camera.expected_cameras,
            record_zero_on_missing=True,
        )

    def _close_recorder_resources(self) -> None:
        if self.camera_client is not None:
            close = getattr(self.camera_client, "close", None)
            if callable(close):
                close()
        self.camera_client = None
        self.recorder = None
        self.episode_name = ""
        self.frame_count = 0
