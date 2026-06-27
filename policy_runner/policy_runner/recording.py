from __future__ import annotations

import hashlib
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .camera_bundle_client import _MISSING_BUNDLE_AGE_US, bundle_clock_ns, resolve_frame

from .robot_state_client import StateSnapshot


DATASET_METADATA_SCHEMA = "robotics_lab.policy_runner.dataset_metadata.v1"


def utc_episode_name(prefix: str = "episode") -> str:
    return f"{prefix}_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"


def build_dataset_metadata(
    *,
    git_commit: str | None = None,
    config_hash: str | None = None,
    backend_type: str | None = None,
    run_mode: str | None = None,
    operation_mode: str | None = None,
    physical_motion_expected: bool | None = None,
    controller_pgmode: str | None = None,
    calibration_status: str | None = None,
    camera_status: str | None = None,
    command_source_id: str | None = None,
    benchmark_linkage: dict[str, Any] | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build operator-supplied dataset provenance metadata.

    The helper is intentionally passive: it only formats metadata for recorders
    and does not inspect hardware, set safety gates, or send commands.
    """

    metadata: dict[str, Any] = {
        "schema": DATASET_METADATA_SCHEMA,
        "git_commit": git_commit,
        "config_hash": config_hash,
        "backend_type": backend_type,
        "run_mode": run_mode,
        "operation_mode": operation_mode,
        "physical_motion_expected": physical_motion_expected,
        "controller_pgmode": controller_pgmode,
        "calibration_status": calibration_status,
        "camera_status": camera_status,
        "command_source_id": command_source_id,
        "benchmark_linkage": dict(benchmark_linkage or {}),
    }
    if extra:
        metadata.update(extra)
    return metadata


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


@dataclass
class _EpisodeBuffer:
    episode_id: str
    task_description: str
    action_source: str
    operator_id: str | None
    dataset_metadata: dict[str, Any]
    cartesian_control_snapshot: dict[str, Any]
    kinematics_snapshot: dict[str, Any]
    cartesian_control_hash: str
    kinematics_hash: str
    reset_qpos_left: list[float]
    reset_qpos_right: list[float]
    reset_tcp_stand_left: list[float]
    reset_tcp_stand_right: list[float]
    start_wall_time_ns: int
    start_monotonic: float
    last_appended_monotonic: float
    qpos_left: list[list[float]]
    qpos_right: list[list[float]]
    qsent_left: list[list[float]]
    qsent_right: list[list[float]]
    tcp_stand_left: list[list[float]]
    tcp_stand_right: list[list[float]]
    q_ref_left: list[list[float]]
    q_ref_right: list[list[float]]
    tcp_actual_stand_left: list[list[float]]
    tcp_actual_stand_right: list[list[float]]
    tcp_ref_stand_left: list[list[float]]
    tcp_ref_stand_right: list[list[float]]
    tcp_actual_valid_left: list[bool]
    tcp_actual_valid_right: list[bool]
    tcp_ref_valid_left: list[bool]
    tcp_ref_valid_right: list[bool]
    tcp_tracking_source_left: list[str]
    tcp_tracking_source_right: list[str]
    diagnostics_suspect_left: list[bool]
    diagnostics_suspect_right: list[bool]
    send_duration_us_left: list[int]
    send_duration_us_right: list[int]
    ack_policy_left: list[str]
    ack_policy_right: list[str]
    controller_acceptance_observed_left: list[bool]
    controller_acceptance_observed_right: list[bool]
    fault_latched: list[bool]
    state_age_us: list[int]
    state_host_time_ns: list[int]
    command_seq: list[int]
    action_mode: list[str]
    action_source_id: list[str]
    action_tcp_target_stand_left: list[list[float]]
    action_tcp_target_stand_right: list[list[float]]
    action_joint_target_left: list[list[float]]
    action_joint_target_right: list[list[float]]
    action_spacemouse_axes_left: list[list[float]]
    action_spacemouse_axes_right: list[list[float]]
    action_spacemouse_buttons_left: list[list[bool]]
    action_spacemouse_buttons_right: list[list[bool]]
    action_deadman_left: list[bool]
    action_deadman_right: list[bool]
    action_host_time_ns: list[int]
    action_seq: list[int]
    images: dict[str, list[Any]]
    image_shapes: dict[str, tuple[int, ...]]
    image_dtypes: dict[str, Any]
    bundle_seq: list[int]
    bundle_time_ns: list[int]
    bundle_age_us: list[int]


class Hdf5EpisodeRecorder:
    """ACT-compatible per-episode HDF5 recorder.

    Buffers state/action frames, plus optional camera frames, in memory and
    writes one HDF5 file per episode when end_episode() is called.
    """

    SCHEMA = "robotics_lab.episode.v1"
    VALID_END_REASONS = {
        "operator_success",
        "operator_failed",
        "timeout",
        "fault_latched",
        "operator_abort",
        "other",
    }

    def __init__(
        self,
        output_dir: str | Path,
        *,
        recording_rate_hz: float = 30.0,
        camera_client: Any | None = None,
        expected_cameras: list[str] | None = None,
        record_zero_on_missing: bool = True,
    ):
        try:
            import h5py  # noqa: F401
            import numpy  # noqa: F401
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Hdf5EpisodeRecorder requires optional dependencies: "
                "install policy_runner with the recording extra"
            ) from exc
        if recording_rate_hz < 1.0 or recording_rate_hz > 100.0:
            raise ValueError("recording_rate_hz must be in [1.0, 100.0]")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.recording_rate_hz = float(recording_rate_hz)
        self._period_sec = 1.0 / self.recording_rate_hz
        self.camera_client = camera_client
        self.expected_cameras = list(expected_cameras or [])
        self.record_zero_on_missing = bool(record_zero_on_missing)
        self._current_episode: _EpisodeBuffer | None = None

    @property
    def has_active_episode(self) -> bool:
        return self._current_episode is not None

    @property
    def episode_id(self) -> str | None:
        return None if self._current_episode is None else self._current_episode.episode_id

    @property
    def frame_count(self) -> int:
        return 0 if self._current_episode is None else len(self._current_episode.qpos_left)

    def start_episode(
        self,
        *,
        reset_snapshot: StateSnapshot,
        task_description: str,
        action_source: str,
        operator_id: str | None = None,
        dataset_metadata: dict[str, Any] | None = None,
    ) -> None:
        """Begin an episode and anchor reset pose from the current state."""

        if self._current_episode is not None:
            raise RuntimeError(
                f"Episode '{self._current_episode.episode_id}' is still active; "
                "call end_episode() first"
            )
        payload = reset_snapshot.payload
        left = payload.get("left", {}) if isinstance(payload.get("left"), dict) else {}
        right = payload.get("right", {}) if isinstance(payload.get("right"), dict) else {}
        cartesian_control_snapshot = _dict_or_empty(payload.get("cartesian_control_snapshot"))
        kinematics_snapshot = _dict_or_empty(payload.get("kinematics_snapshot"))
        self._current_episode = _EpisodeBuffer(
            episode_id=self._next_episode_id(),
            task_description=task_description,
            action_source=action_source,
            operator_id=operator_id,
            dataset_metadata=dict(dataset_metadata or {}),
            cartesian_control_snapshot=cartesian_control_snapshot,
            kinematics_snapshot=kinematics_snapshot,
            cartesian_control_hash=_hash_canonical_json(cartesian_control_snapshot),
            kinematics_hash=_hash_canonical_json(kinematics_snapshot),
            reset_qpos_left=_float_list(left.get("q_actual_deg"), 6),
            reset_qpos_right=_float_list(right.get("q_actual_deg"), 6),
            reset_tcp_stand_left=_extract_tcp_stand_7(left),
            reset_tcp_stand_right=_extract_tcp_stand_7(right),
            start_wall_time_ns=time.time_ns(),
            start_monotonic=time.monotonic(),
            last_appended_monotonic=float("-inf"),
            qpos_left=[],
            qpos_right=[],
            qsent_left=[],
            qsent_right=[],
            tcp_stand_left=[],
            tcp_stand_right=[],
            q_ref_left=[],
            q_ref_right=[],
            tcp_actual_stand_left=[],
            tcp_actual_stand_right=[],
            tcp_ref_stand_left=[],
            tcp_ref_stand_right=[],
            tcp_actual_valid_left=[],
            tcp_actual_valid_right=[],
            tcp_ref_valid_left=[],
            tcp_ref_valid_right=[],
            tcp_tracking_source_left=[],
            tcp_tracking_source_right=[],
            diagnostics_suspect_left=[],
            diagnostics_suspect_right=[],
            send_duration_us_left=[],
            send_duration_us_right=[],
            ack_policy_left=[],
            ack_policy_right=[],
            controller_acceptance_observed_left=[],
            controller_acceptance_observed_right=[],
            fault_latched=[],
            state_age_us=[],
            state_host_time_ns=[],
            command_seq=[],
            action_mode=[],
            action_source_id=[],
            action_tcp_target_stand_left=[],
            action_tcp_target_stand_right=[],
            action_joint_target_left=[],
            action_joint_target_right=[],
            action_spacemouse_axes_left=[],
            action_spacemouse_axes_right=[],
            action_spacemouse_buttons_left=[],
            action_spacemouse_buttons_right=[],
            action_deadman_left=[],
            action_deadman_right=[],
            action_host_time_ns=[],
            action_seq=[],
            images={},
            image_shapes={},
            image_dtypes={},
            bundle_seq=[],
            bundle_time_ns=[],
            bundle_age_us=[],
        )

    def record_frame(
        self,
        *,
        state_snapshot: StateSnapshot,
        action_packet: dict[str, Any] | None,
        action_host_time_ns: int | None,
        action_seq: int | None,
    ) -> None:
        """Append one state/action frame if the recording period has elapsed."""

        ep = self._current_episode
        if ep is None:
            raise RuntimeError("no active episode; call start_episode() first")

        now = time.monotonic()
        if ep.last_appended_monotonic == float("-inf"):
            ep.last_appended_monotonic = now
        else:
            elapsed = now - ep.last_appended_monotonic
            if elapsed < self._period_sec - 1e-6:
                return
            periods_elapsed = max(1, int(elapsed / self._period_sec))
            ep.last_appended_monotonic += periods_elapsed * self._period_sec

        payload = state_snapshot.payload
        left = payload.get("left", {}) if isinstance(payload.get("left"), dict) else {}
        right = payload.get("right", {}) if isinstance(payload.get("right"), dict) else {}

        ep.qpos_left.append(_float_list(left.get("q_actual_deg"), 6))
        ep.qpos_right.append(_float_list(right.get("q_actual_deg"), 6))
        ep.qsent_left.append(_float_list(left.get("q_sent_deg"), 6))
        ep.qsent_right.append(_float_list(right.get("q_sent_deg"), 6))
        ep.tcp_stand_left.append(_extract_tcp_stand_7(left))
        ep.tcp_stand_right.append(_extract_tcp_stand_7(right))
        ep.q_ref_left.append(_float_list(_first_present(left, "q_ref_deg", "q_target_deg"), 6))
        ep.q_ref_right.append(_float_list(_first_present(right, "q_ref_deg", "q_target_deg"), 6))
        ep.tcp_actual_stand_left.append(_extract_tcp_pose_7(left, "tcp_actual_stand", fallback_key="tcp_stand"))
        ep.tcp_actual_stand_right.append(_extract_tcp_pose_7(right, "tcp_actual_stand", fallback_key="tcp_stand"))
        ep.tcp_ref_stand_left.append(_extract_tcp_pose_7(left, "tcp_ref_stand"))
        ep.tcp_ref_stand_right.append(_extract_tcp_pose_7(right, "tcp_ref_stand"))
        ep.tcp_actual_valid_left.append(_bool_with_fallback(left.get("tcp_actual_valid"), bool(left.get("tcp_stand"))))
        ep.tcp_actual_valid_right.append(_bool_with_fallback(right.get("tcp_actual_valid"), bool(right.get("tcp_stand"))))
        ep.tcp_ref_valid_left.append(bool(left.get("tcp_ref_valid", False)))
        ep.tcp_ref_valid_right.append(bool(right.get("tcp_ref_valid", False)))
        ep.tcp_tracking_source_left.append(_tracking_source(left))
        ep.tcp_tracking_source_right.append(_tracking_source(right))
        ep.diagnostics_suspect_left.append(_diagnostics_suspect(left))
        ep.diagnostics_suspect_right.append(_diagnostics_suspect(right))
        ep.send_duration_us_left.append(_duration_us(left))
        ep.send_duration_us_right.append(_duration_us(right))
        ep.ack_policy_left.append(str(_first_present(left, "ack_policy", "ack_semantics") or ""))
        ep.ack_policy_right.append(str(_first_present(right, "ack_policy", "ack_semantics") or ""))
        ep.controller_acceptance_observed_left.append(bool(left.get("controller_acceptance_observed", False)))
        ep.controller_acceptance_observed_right.append(bool(right.get("controller_acceptance_observed", False)))
        ep.fault_latched.append(bool(payload.get("fault_latched", False)))
        ep.state_age_us.append(int(payload.get("state_age_us", 0) or 0))
        ep.state_host_time_ns.append(int(payload.get("host_time_ns", 0) or 0))
        ep.command_seq.append(int(payload.get("command_seq", 0) or 0))

        if isinstance(action_packet, dict):
            left_action = action_packet.get("left", {}) if isinstance(action_packet.get("left"), dict) else {}
            right_action = action_packet.get("right", {}) if isinstance(action_packet.get("right"), dict) else {}
            left_mode = str(left_action.get("mode", "Hold"))
            right_mode = str(right_action.get("mode", "Hold"))
            mode = _dominant_action_mode(left_mode, right_mode)
            ep.action_mode.append(mode)
            ep.action_source_id.append(str(action_packet.get("source_id", "") or ""))
            ep.action_tcp_target_stand_left.append(_tcp_target_values(left_action.get("tcp_target_stand")))
            ep.action_tcp_target_stand_right.append(_tcp_target_values(right_action.get("tcp_target_stand")))
            ep.action_joint_target_left.append(_float_list(left_action.get("q_target_deg"), 6))
            ep.action_joint_target_right.append(_float_list(right_action.get("q_target_deg"), 6))
            ep.action_spacemouse_axes_left.append(_spacemouse_axes(left_action, action_packet, "left"))
            ep.action_spacemouse_axes_right.append(_spacemouse_axes(right_action, action_packet, "right"))
            ep.action_spacemouse_buttons_left.append(_spacemouse_buttons(left_action, action_packet, "left"))
            ep.action_spacemouse_buttons_right.append(_spacemouse_buttons(right_action, action_packet, "right"))
            ep.action_deadman_left.append(bool(left_action.get("deadman", False)))
            ep.action_deadman_right.append(bool(right_action.get("deadman", False)))
        else:
            ep.action_mode.append("Hold")
            ep.action_source_id.append("")
            ep.action_tcp_target_stand_left.append([0.0] * 6)
            ep.action_tcp_target_stand_right.append([0.0] * 6)
            ep.action_joint_target_left.append([0.0] * 6)
            ep.action_joint_target_right.append([0.0] * 6)
            ep.action_spacemouse_axes_left.append([0.0] * 6)
            ep.action_spacemouse_axes_right.append([0.0] * 6)
            ep.action_spacemouse_buttons_left.append([False] * 8)
            ep.action_spacemouse_buttons_right.append([False] * 8)
            ep.action_deadman_left.append(False)
            ep.action_deadman_right.append(False)

        ep.action_host_time_ns.append(int(action_host_time_ns or 0))
        ep.action_seq.append(int(action_seq or 0))
        self._record_camera_frame(ep)

    def end_episode(
        self,
        *,
        success: bool,
        end_reason: str,
        notes: str | None = None,
    ) -> Path:
        """Flush the active episode to HDF5 and return the written path."""

        ep = self._current_episode
        if ep is None:
            raise RuntimeError("no active episode")
        if end_reason not in self.VALID_END_REASONS:
            raise ValueError(f"unknown end_reason: {end_reason}")

        import h5py
        import numpy as np

        duration_sec = time.monotonic() - ep.start_monotonic
        out_path = self.output_dir / f"{ep.episode_id}.hdf5"

        with h5py.File(out_path, "w") as h5:
            h5.attrs["schema"] = self.SCHEMA
            h5.attrs["episode_id"] = ep.episode_id
            h5.attrs["created_wall_time_ns"] = ep.start_wall_time_ns
            h5.attrs["task_description"] = ep.task_description
            h5.attrs["action_source"] = ep.action_source
            if ep.operator_id is not None:
                h5.attrs["operator_id"] = ep.operator_id
            h5.attrs["reset_qpos_left"] = np.asarray(ep.reset_qpos_left, dtype=np.float32)
            h5.attrs["reset_qpos_right"] = np.asarray(ep.reset_qpos_right, dtype=np.float32)
            h5.attrs["reset_tcp_stand_left"] = np.asarray(ep.reset_tcp_stand_left, dtype=np.float32)
            h5.attrs["reset_tcp_stand_right"] = np.asarray(ep.reset_tcp_stand_right, dtype=np.float32)
            h5.attrs["success"] = bool(success)
            h5.attrs["end_reason"] = end_reason
            if notes is not None:
                h5.attrs["notes"] = notes
            h5.attrs["duration_sec"] = duration_sec
            h5.attrs["recording_rate_hz"] = self.recording_rate_hz
            h5.attrs["frame_count"] = len(ep.qpos_left)
            h5.attrs["cartesian_control_hash"] = ep.cartesian_control_hash
            h5.attrs["kinematics_hash"] = ep.kinematics_hash

            config = h5.create_group("config")
            _write_dict_as_attrs(config.create_group("cartesian_control"), ep.cartesian_control_snapshot)
            _write_dict_as_attrs(config.create_group("kinematics"), ep.kinematics_snapshot)
            if ep.dataset_metadata:
                metadata = h5.create_group("metadata")
                dataset = metadata.create_group("dataset")
                _write_dict_as_attrs(dataset, ep.dataset_metadata)

            obs = h5.create_group("observations")
            obs.create_dataset(
                "qpos_left",
                data=np.asarray(ep.qpos_left, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "qpos_right",
                data=np.asarray(ep.qpos_right, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "qsent_left",
                data=np.asarray(ep.qsent_left, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "qsent_right",
                data=np.asarray(ep.qsent_right, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "tcp_stand_left",
                data=np.asarray(ep.tcp_stand_left, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "tcp_stand_right",
                data=np.asarray(ep.tcp_stand_right, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "q_ref_left",
                data=np.asarray(ep.q_ref_left, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "q_ref_right",
                data=np.asarray(ep.q_ref_right, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "tcp_actual_stand_left",
                data=np.asarray(ep.tcp_actual_stand_left, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "tcp_actual_stand_right",
                data=np.asarray(ep.tcp_actual_stand_right, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "tcp_ref_stand_left",
                data=np.asarray(ep.tcp_ref_stand_left, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset(
                "tcp_ref_stand_right",
                data=np.asarray(ep.tcp_ref_stand_right, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            obs.create_dataset("tcp_actual_valid_left", data=np.asarray(ep.tcp_actual_valid_left, dtype=bool))
            obs.create_dataset("tcp_actual_valid_right", data=np.asarray(ep.tcp_actual_valid_right, dtype=bool))
            obs.create_dataset("tcp_ref_valid_left", data=np.asarray(ep.tcp_ref_valid_left, dtype=bool))
            obs.create_dataset("tcp_ref_valid_right", data=np.asarray(ep.tcp_ref_valid_right, dtype=bool))
            string_dtype = h5py.string_dtype(encoding="utf-8")
            obs.create_dataset(
                "tcp_tracking_source_left",
                data=np.asarray(ep.tcp_tracking_source_left, dtype=object),
                dtype=string_dtype,
            )
            obs.create_dataset(
                "tcp_tracking_source_right",
                data=np.asarray(ep.tcp_tracking_source_right, dtype=object),
                dtype=string_dtype,
            )
            obs.create_dataset("diagnostics_suspect_left", data=np.asarray(ep.diagnostics_suspect_left, dtype=bool))
            obs.create_dataset("diagnostics_suspect_right", data=np.asarray(ep.diagnostics_suspect_right, dtype=bool))
            obs.create_dataset("send_duration_us_left", data=np.asarray(ep.send_duration_us_left, dtype=np.int64))
            obs.create_dataset("send_duration_us_right", data=np.asarray(ep.send_duration_us_right, dtype=np.int64))
            obs.create_dataset(
                "ack_policy_left",
                data=np.asarray(ep.ack_policy_left, dtype=object),
                dtype=string_dtype,
            )
            obs.create_dataset(
                "ack_policy_right",
                data=np.asarray(ep.ack_policy_right, dtype=object),
                dtype=string_dtype,
            )
            obs.create_dataset(
                "controller_acceptance_observed_left",
                data=np.asarray(ep.controller_acceptance_observed_left, dtype=bool),
            )
            obs.create_dataset(
                "controller_acceptance_observed_right",
                data=np.asarray(ep.controller_acceptance_observed_right, dtype=bool),
            )
            obs.create_dataset("fault_latched", data=np.asarray(ep.fault_latched, dtype=bool))
            obs.create_dataset("state_age_us", data=np.asarray(ep.state_age_us, dtype=np.int64))
            obs.create_dataset("state_host_time_ns", data=np.asarray(ep.state_host_time_ns, dtype=np.int64))
            obs.create_dataset("command_seq", data=np.asarray(ep.command_seq, dtype=np.int64))
            if ep.images:
                images = obs.create_group("images")
                for cam_name, frames in ep.images.items():
                    if not frames:
                        continue
                    # Preserve native dtype: uint8 H,W,C color and uint16 H,W
                    # depth coexist; chunk one frame at a time regardless of rank.
                    stacked = np.ascontiguousarray(np.stack(frames, axis=0))
                    images.create_dataset(
                        cam_name,
                        data=stacked,
                        dtype=stacked.dtype,
                        chunks=(1, *stacked.shape[1:]),
                        compression="gzip",
                        compression_opts=1,
                    )
            if ep.bundle_seq:
                obs.create_dataset("bundle_seq", data=np.asarray(ep.bundle_seq, dtype=np.int64))
                obs.create_dataset("bundle_host_time_ns", data=np.asarray(ep.bundle_time_ns, dtype=np.int64))
                obs.create_dataset("bundle_age_us", data=np.asarray(ep.bundle_age_us, dtype=np.int64))

            action = h5.create_group("action")
            action.create_dataset("mode", data=np.asarray(ep.action_mode, dtype=object), dtype=string_dtype)
            action.create_dataset(
                "source_id",
                data=np.asarray(ep.action_source_id, dtype=object),
                dtype=string_dtype,
            )
            action.create_dataset(
                "tcp_target_stand_left",
                data=np.asarray(ep.action_tcp_target_stand_left, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            action.create_dataset(
                "tcp_target_stand_right",
                data=np.asarray(ep.action_tcp_target_stand_right, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            action.create_dataset(
                "joint_target_left",
                data=np.asarray(ep.action_joint_target_left, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            action.create_dataset(
                "joint_target_right",
                data=np.asarray(ep.action_joint_target_right, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            action.create_dataset(
                "spacemouse_axes_left",
                data=np.asarray(ep.action_spacemouse_axes_left, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            action.create_dataset(
                "spacemouse_axes_right",
                data=np.asarray(ep.action_spacemouse_axes_right, dtype=np.float32),
                compression="gzip",
                compression_opts=1,
            )
            action.create_dataset("spacemouse_buttons_left", data=np.asarray(ep.action_spacemouse_buttons_left, dtype=bool))
            action.create_dataset("spacemouse_buttons_right", data=np.asarray(ep.action_spacemouse_buttons_right, dtype=bool))
            action.create_dataset("deadman_left", data=np.asarray(ep.action_deadman_left, dtype=bool))
            action.create_dataset("deadman_right", data=np.asarray(ep.action_deadman_right, dtype=bool))
            action.create_dataset(
                "action_host_time_ns",
                data=np.asarray(ep.action_host_time_ns, dtype=np.int64),
            )
            action.create_dataset("seq", data=np.asarray(ep.action_seq, dtype=np.int64))

        self._current_episode = None
        return out_path

    def close(self) -> None:
        if self._current_episode is not None:
            self.end_episode(success=False, end_reason="operator_abort")

    def _next_episode_id(self) -> str:
        base = utc_episode_name("ep")
        if not (self.output_dir / f"{base}.hdf5").exists():
            return base
        suffix = 1
        while True:
            candidate = f"{base}_{suffix:03d}"
            if not (self.output_dir / f"{candidate}.hdf5").exists():
                return candidate
            suffix += 1

    def _record_camera_frame(self, ep: _EpisodeBuffer) -> None:
        if self.camera_client is None:
            return
        import numpy as np

        bundle = self.camera_client.poll(timeout_ms=0)
        if bundle is None:
            bundle = self.camera_client.latest()

        if bundle is not None:
            # bundle_time_ns is on the camera_server clock (monotonic_raw), not epoch.
            bundle_age_ns = bundle_clock_ns() - int(bundle.bundle_time_ns)
            ep.bundle_seq.append(int(bundle.bundle_seq))
            ep.bundle_time_ns.append(int(bundle.bundle_time_ns))
            ep.bundle_age_us.append(int(bundle_age_ns // 1000))
        else:
            ep.bundle_seq.append(0)
            ep.bundle_time_ns.append(0)
            ep.bundle_age_us.append(_MISSING_BUNDLE_AGE_US)

        cameras = list(self.expected_cameras)
        if not cameras and bundle is not None:
            cameras = sorted(bundle.frames.keys())

        for cam_name in cameras:
            # expected_cameras may use dataset names ('left_realsense_color')
            # while bundle keys are '<camera>.<stream>'; resolve_frame maps both.
            frame = resolve_frame(bundle.frames, cam_name) if bundle is not None else None
            if frame is not None:
                pixels = np.ascontiguousarray(frame.pixels)
                shape = tuple(int(dim) for dim in pixels.shape)
                # Color frames are H,W,C (uint8); depth/mono frames are H,W
                # (e.g. RealSense z16 -> uint16). Accept both ranks and keep the
                # native dtype — forcing uint8 would corrupt 16-bit depth.
                if len(shape) not in (2, 3):
                    raise RuntimeError(
                        f"camera '{cam_name}' pixels must have shape H,W (depth/mono) or H,W,C"
                    )
                previous_shape = ep.image_shapes.get(cam_name)
                if previous_shape is not None and previous_shape != shape:
                    raise RuntimeError(
                        f"camera '{cam_name}' shape changed from {previous_shape} to {shape}"
                    )
                ep.image_shapes.setdefault(cam_name, shape)
                ep.image_dtypes.setdefault(cam_name, pixels.dtype)
                frame_pixels = pixels
            else:
                if not self.record_zero_on_missing:
                    raise RuntimeError(
                        f"camera '{cam_name}' missing from bundle and record_zero_on_missing is False"
                    )
                if cam_name not in ep.image_shapes:
                    continue
                frame_pixels = np.zeros(
                    ep.image_shapes[cam_name],
                    dtype=ep.image_dtypes.get(cam_name, np.uint8),
                )
            ep.images.setdefault(cam_name, []).append(frame_pixels)


def _float_list(value: Any, length: int) -> list[float]:
    if not isinstance(value, (list, tuple)):
        return [0.0] * length
    out = [float(item) for item in value[:length]]
    if len(out) < length:
        out.extend([0.0] * (length - len(out)))
    return out


def _dict_or_empty(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, dict) else {}


def _first_present(mapping: dict[str, Any], *keys: str) -> Any:
    for key in keys:
        value = mapping.get(key)
        if value is not None:
            return value
    return None


def _hash_canonical_json(value: dict[str, Any]) -> str:
    if not value:
        return ""
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _write_dict_as_attrs(group: Any, value: dict[str, Any]) -> None:
    for key, item in sorted(value.items()):
        name = str(key)
        if isinstance(item, dict):
            _write_dict_as_attrs(group.create_group(name), item)
            continue
        if item is None:
            group.attrs[name] = ""
            continue
        try:
            group.attrs[name] = item
        except (TypeError, ValueError):
            group.attrs[name] = json.dumps(item, sort_keys=True, separators=(",", ":"))


def _extract_tcp_stand_7(arm_state: dict[str, Any]) -> list[float]:
    """Extract [x, y, z, qx, qy, qz, qw] from an arm state dict."""

    return _extract_tcp_pose_7(arm_state, "tcp_stand")


def _extract_tcp_pose_7(
    arm_state: dict[str, Any],
    key: str,
    *,
    fallback_key: str | None = None,
) -> list[float]:
    """Extract [x, y, z, qx, qy, qz, qw] from an arm TCP pose field."""

    tcp = arm_state.get(key)
    if not isinstance(tcp, dict) and fallback_key is not None:
        tcp = arm_state.get(fallback_key)
    if not isinstance(tcp, dict):
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    x = float(tcp.get("x", 0.0) or 0.0)
    y = float(tcp.get("y", 0.0) or 0.0)
    z = float(tcp.get("z", 0.0) or 0.0)
    quat = tcp.get("quaternion_xyzw")
    if isinstance(quat, list) and len(quat) == 4:
        return [x, y, z, float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]
    qx = float(tcp.get("qx", 0.0) or 0.0)
    qy = float(tcp.get("qy", 0.0) or 0.0)
    qz = float(tcp.get("qz", 0.0) or 0.0)
    qw = float(tcp.get("qw", 1.0) or 1.0)
    return [x, y, z, qx, qy, qz, qw]


def _bool_with_fallback(value: Any, fallback: bool) -> bool:
    return bool(value) if isinstance(value, bool) else bool(fallback)


def _tracking_source(arm_state: dict[str, Any]) -> str:
    source = _first_present(
        arm_state,
        "tcp_tracking_source",
        "tcp_tracking_source_recommendation",
    )
    return str(source or "")


def _diagnostics_suspect(arm_state: dict[str, Any]) -> bool:
    if isinstance(arm_state.get("diagnostics_suspect"), bool):
        return bool(arm_state["diagnostics_suspect"])
    return str(arm_state.get("lifecycle_state", "")) == "diagnostics_suspect"


def _duration_us(arm_state: dict[str, Any]) -> int:
    value = arm_state.get("send_duration_us")
    if value is None and isinstance(arm_state.get("backend_timing"), dict):
        value = arm_state["backend_timing"].get("send_duration_us")
    return int(value or 0)


def _dominant_action_mode(left_mode: str, right_mode: str) -> str:
    modes = (left_mode, right_mode)
    for candidate in ("TcpPoseTarget", "TcpLinearMove", "JointTarget"):
        if candidate in modes:
            return candidate
    return left_mode


def _tcp_target_values(value: Any) -> list[float]:
    if isinstance(value, list):
        return _float_list(value, 6)
    if not isinstance(value, dict):
        return [0.0] * 6
    return [
        float(value.get("x", 0.0) or 0.0),
        float(value.get("y", 0.0) or 0.0),
        float(value.get("z", 0.0) or 0.0),
        float(value.get("rx", 0.0) or 0.0),
        float(value.get("ry", 0.0) or 0.0),
        float(value.get("rz", 0.0) or 0.0),
    ]


def _spacemouse_axes(
    arm_action: dict[str, Any],
    packet: dict[str, Any],
    side: str,
) -> list[float]:
    value = _spacemouse_value(arm_action, packet, side, "axes")
    return _float_list(value, 6)


def _spacemouse_buttons(
    arm_action: dict[str, Any],
    packet: dict[str, Any],
    side: str,
) -> list[bool]:
    value = _spacemouse_value(arm_action, packet, side, "buttons")
    if not isinstance(value, (list, tuple)):
        return [False] * 8
    out = [bool(item) for item in value[:8]]
    if len(out) < 8:
        out.extend([False] * (8 - len(out)))
    return out


def _spacemouse_value(
    arm_action: dict[str, Any],
    packet: dict[str, Any],
    side: str,
    name: str,
) -> Any:
    for container in (arm_action, packet.get(side), packet):
        if not isinstance(container, dict):
            continue
        for key in (
            f"spacemouse_{name}",
            f"spacemouse_raw_{name}",
            f"raw_{name}",
        ):
            if key in container:
                return container[key]
        raw = container.get("spacemouse_raw")
        if isinstance(raw, dict) and name in raw:
            return raw[name]
    return None


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
