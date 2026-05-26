from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .robot_state_client import StateSnapshot


def utc_episode_name(prefix: str = "episode") -> str:
    return f"{prefix}_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"


@dataclass(frozen=True)
class EpisodePaths:
    root: Path
    metadata: Path
    robot_state: Path
    actions: Path


class EpisodeRecorder:
    def __init__(
        self,
        output_dir: str | Path,
        *,
        episode_name: str | None = None,
        metadata: dict[str, Any] | None = None,
    ):
        root = Path(output_dir) / (episode_name or utc_episode_name())
        root.mkdir(parents=True, exist_ok=False)
        self.paths = EpisodePaths(
            root=root,
            metadata=root / "episode_metadata.json",
            robot_state=root / "robot_state.jsonl",
            actions=root / "actions.jsonl",
        )
        meta = {
            "schema": "robotics_lab.policy_runner.episode.v1",
            "created_wall_time_ns": time.time_ns(),
            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "modalities": ["robot_state", "actions"],
        }
        if metadata:
            meta.update(metadata)
        self.paths.metadata.write_text(json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self._state_handle = self.paths.robot_state.open("a", encoding="utf-8")
        self._action_handle = self.paths.actions.open("a", encoding="utf-8")
        self._last_state_key: tuple[Any, Any] | None = None
        self._last_state_payload: dict[str, Any] | None = None

    @property
    def latest_state_payload(self) -> dict[str, Any] | None:
        return self._last_state_payload

    def close(self) -> None:
        self._state_handle.close()
        self._action_handle.close()

    def record_state(self, snapshot: StateSnapshot) -> None:
        payload = snapshot.payload
        key = (payload.get("tick"), payload.get("host_time_ns"))
        if key == self._last_state_key:
            return
        self._last_state_key = key
        self._last_state_payload = payload
        self._write_jsonl(
            self._state_handle,
            {
                "schema": "robotics_lab.policy_runner.robot_state_sample.v1",
                "received_wall_time_ns": time.time_ns(),
                "received_monotonic_sec": snapshot.received_monotonic,
                "payload": payload,
            },
        )

    def record_action(self, packet: dict[str, Any]) -> None:
        state = self._last_state_payload or {}
        self._write_jsonl(
            self._action_handle,
            {
                "schema": "robotics_lab.policy_runner.action_sample.v1",
                "sent_wall_time_ns": time.time_ns(),
                "nearest_state_tick": state.get("tick"),
                "nearest_state_host_time_ns": state.get("host_time_ns"),
                "packet": packet,
            },
        )

    @staticmethod
    def _write_jsonl(handle: Any, value: dict[str, Any]) -> None:
        handle.write(json.dumps(value, separators=(",", ":"), sort_keys=True) + "\n")
        handle.flush()


def record_state_stream(
    *,
    bind: str,
    output_dir: str | Path,
    stale_timeout_sec: float = 0.5,
    duration_sec: float = 0.0,
    poll_timeout_sec: float = 0.2,
) -> Path:
    from .robot_state_client import RobotStateClient

    recorder = EpisodeRecorder(
        output_dir,
        metadata={
            "recording_mode": "state_only",
            "robot_state_bind": bind,
        },
    )
    client = RobotStateClient(bind, stale_timeout_sec)
    deadline = None if duration_sec <= 0.0 else time.monotonic() + duration_sec
    try:
        while deadline is None or time.monotonic() < deadline:
            snapshot = client.poll_once(timeout_sec=poll_timeout_sec)
            if snapshot is not None:
                recorder.record_state(snapshot)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        recorder.close()
    return recorder.paths.root
