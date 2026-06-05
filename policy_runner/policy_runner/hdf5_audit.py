from __future__ import annotations

import json
import math
from collections import Counter
from pathlib import Path
from typing import Any

import h5py
import numpy as np

from .dataset_manifest import DatasetManifest
from .flow_dataset import decode_hdf5_image_value, discover_hdf5_episodes


HDF5_AUDIT_SCHEMA = "robotics_lab.policy_runner.hdf5_audit.v1"
SUPPORTED_POSE_FORMATS = {"x,y,z,qx,qy,qz,qw"}
KNOWN_POSE_FRAMES = {"stand", "steamvr_world", "left_base", "right_base"}
SUPPORTED_IMAGE_ENCODINGS = {
    "jpeg",
    "jpg",
    "png",
    "png16",
    "rgb8",
    "rgba8",
    "gray8",
    "uint8",
}


def audit_hdf5_episodes(
    episodes_dir: str | Path,
    *,
    dataset_manifest: str | Path | DatasetManifest | None = None,
    single_arm_side: str | None = None,
) -> dict[str, Any]:
    """Audit HDF5 episodes before flow training or real-rollout consideration."""

    manifest = _coerce_manifest(dataset_manifest)
    side = single_arm_side or (manifest.single_arm_side if manifest is not None else "left")
    if side not in {"left", "right"}:
        raise ValueError("single_arm_side must be left or right")

    root = Path(episodes_dir)
    paths = discover_hdf5_episodes(root)
    if not paths:
        raise ValueError(f"no HDF5 episodes found under {root}")

    episodes = [
        _audit_episode(path, root=root, single_arm_side=side, manifest=manifest)
        for path in paths
    ]
    if sum(int(episode["length"]) for episode in episodes) <= 0:
        raise ValueError(f"HDF5 audit found zero frames under {root}")

    aggregate_warnings = _apply_mixed_camera_warnings(episodes)
    formats = Counter(str(episode["format_name"]) for episode in episodes)
    aggregate_blockers = sorted(
        {
            blocker
            for episode in episodes
            for blocker in episode.get("deployment_blockers", [])
        }
    )
    aggregate_camera_names = sorted(
        {
            camera_name
            for episode in episodes
            for camera_name in episode.get("camera_names", [])
        }
    )
    return {
        "schema": HDF5_AUDIT_SCHEMA,
        "episodes": episodes,
        "aggregate": {
            "episode_count": len(episodes),
            "frame_count": int(sum(int(episode["length"]) for episode in episodes)),
            "camera_names": aggregate_camera_names,
            "formats": dict(sorted(formats.items())),
            "warnings": aggregate_warnings,
            "deployment_blockers": aggregate_blockers,
            "warning_count": int(
                sum(len(episode.get("warnings", [])) for episode in episodes)
            ),
            "deployment_blocker_count": int(
                sum(len(episode.get("deployment_blockers", [])) for episode in episodes)
            ),
        },
    }


def write_hdf5_audit_outputs(
    report: dict[str, Any],
    *,
    output_json: str | Path,
    output_md: str | Path,
) -> None:
    json_path = Path(output_json)
    md_path = Path(output_md)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    md_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    md_path.write_text(render_hdf5_audit_markdown(report), encoding="utf-8")


def render_hdf5_audit_markdown(report: dict[str, Any]) -> str:
    aggregate = report.get("aggregate", {})
    lines = [
        "# HDF5 Audit Report",
        "",
        f"Schema: `{report.get('schema', '')}`",
        "",
        "## Aggregate",
        "",
        f"- Episodes: {aggregate.get('episode_count', 0)}",
        f"- Frames: {aggregate.get('frame_count', 0)}",
        f"- Formats: `{json.dumps(aggregate.get('formats', {}), sort_keys=True)}`",
        f"- Cameras: {', '.join(aggregate.get('camera_names', [])) or '(none)'}",
        f"- Warning count: {aggregate.get('warning_count', 0)}",
        f"- Deployment blocker count: {aggregate.get('deployment_blocker_count', 0)}",
        "",
    ]
    blockers = aggregate.get("deployment_blockers", [])
    if blockers:
        lines.extend(["## Deployment Blockers", ""])
        lines.extend(f"- {blocker}" for blocker in blockers)
        lines.append("")

    lines.extend([
        "## Episodes",
        "",
        "| Path | Format | Frames | Hz | Cameras | Warnings | Deployment Blockers |",
        "| --- | --- | ---: | ---: | --- | ---: | ---: |",
    ])
    for episode in report.get("episodes", []):
        cameras = ", ".join(episode.get("camera_names", []))
        lines.append(
            "| "
            f"{episode.get('path', '')} | "
            f"{episode.get('format_name', '')} | "
            f"{episode.get('length', 0)} | "
            f"{episode.get('effective_hz', 0.0)} | "
            f"{cameras} | "
            f"{len(episode.get('warnings', []))} | "
            f"{len(episode.get('deployment_blockers', []))} |"
        )

    for episode in report.get("episodes", []):
        lines.extend(["", f"### {episode.get('path', '')}", ""])
        if episode.get("warnings"):
            lines.append("Warnings:")
            lines.extend(f"- {warning}" for warning in episode["warnings"])
            lines.append("")
        if episode.get("deployment_blockers"):
            lines.append("Deployment blockers:")
            lines.extend(f"- {blocker}" for blocker in episode["deployment_blockers"])
            lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def _audit_episode(
    path: Path,
    *,
    root: Path,
    single_arm_side: str,
    manifest: DatasetManifest | None,
) -> dict[str, Any]:
    warnings: list[str] = []
    blockers: list[str] = []
    with h5py.File(path, "r") as handle:
        format_name = _detect_format(handle)
        camera_paths = _camera_paths_for_format(handle, format_name)
        camera_names = sorted(camera_paths)
        timestamps = _timestamps_for_format(handle, format_name)
        base_length = _episode_length(handle, format_name, timestamps, {})
        length = _episode_length(handle, format_name, timestamps, camera_paths)
        timestamps = timestamps[:length]
        duration_sec, effective_hz = _timing_summary(timestamps, length)
        pose_frame = _pose_frame(handle, format_name)
        pose_format = _pose_format(handle, format_name)
        arm_mask = _arm_mask_for_format(handle, format_name, single_arm_side)
        action_kind = _action_kind_for_format(handle, format_name)

        if format_name == "unsupported":
            _append(warnings, "unsupported_format: HDF5 layout is not supported by flow_dataset")
            _append(blockers, "unsupported_format: episode cannot be used for training")

        _check_required_attrs(handle, manifest, warnings, blockers)
        _check_pose_metadata(pose_frame, pose_format, manifest, warnings, blockers)
        _check_timestamp_outliers(timestamps, warnings)
        _check_gripper_presence(handle, format_name, warnings)
        _check_quaternion_norms(handle, format_name, length, warnings)
        _check_action_matches_pose(handle, format_name, length, warnings)
        camera_encodings = _check_images(
            handle,
            camera_paths,
            length,
            max(base_length, length),
            warnings,
        )

        if format_name == "pika_umi_single_arm":
            _append(
                warnings,
                f"single_arm_default_{single_arm_side}: pika_umi_single_arm mapped to {single_arm_side} arm",
            )

    return {
        "path": _display_path(path, root),
        "format_name": format_name,
        "length": int(length),
        "duration_sec": _rounded(duration_sec, 4),
        "effective_hz": _rounded(effective_hz, 3),
        "pose_frame": pose_frame,
        "pose_format": pose_format,
        "camera_names": camera_names,
        "camera_encodings": camera_encodings,
        "arm_mask": arm_mask,
        "action_kind": action_kind,
        "warnings": warnings,
        "deployment_blockers": blockers,
    }


def _detect_format(handle: h5py.File) -> str:
    schema = _decode_attr(handle.attrs.get("schema"))
    if schema == "robotics_lab.episode.v1" and "observations" in handle:
        return "robotics_lab_dual_arm"
    if _looks_like_pika_bimanual(handle):
        return "pika_umi_bimanual"
    if _looks_like_pika_single_arm(handle):
        return "pika_umi_single_arm"
    return "unsupported"


def _looks_like_pika_single_arm(handle: h5py.File) -> bool:
    return (
        "action" in handle
        and "timestamp" in handle
        and "observations" in handle
        and isinstance(handle["observations"], h5py.Group)
        and "pose" in handle["observations"]
        and "images" in handle["observations"]
    )


def _looks_like_pika_bimanual(handle: h5py.File) -> bool:
    if "timestamp" not in handle or "observations" not in handle:
        return False
    return any(
        "pose" in handle[group_path] and "gripper" in handle[group_path]
        for group_path in _bimanual_arm_groups(handle).values()
        if group_path in handle
    )


def _camera_paths_for_format(handle: h5py.File, format_name: str) -> dict[str, str]:
    if format_name == "pika_umi_single_arm":
        return _camera_paths(handle, "observations/images")
    if format_name == "pika_umi_bimanual":
        out: dict[str, str] = {}
        for side, group_path in _bimanual_arm_groups(handle).items():
            image_group = f"{group_path}/images"
            for camera_name, camera_path in _camera_paths(handle, image_group).items():
                out[f"{side}_{camera_name}"] = camera_path
        return out
    if format_name == "robotics_lab_dual_arm":
        return _camera_paths(handle, "observations/images")
    return {}


def _camera_paths(handle: h5py.File, group_path: str) -> dict[str, str]:
    if group_path not in handle:
        return {}
    group = handle[group_path]
    if not isinstance(group, h5py.Group):
        return {}
    return {
        str(name): f"{group_path}/{name}"
        for name, value in group.items()
        if isinstance(value, h5py.Dataset)
    }


def _timestamps_for_format(handle: h5py.File, format_name: str) -> np.ndarray:
    if format_name in {"pika_umi_single_arm", "pika_umi_bimanual"} and "timestamp" in handle:
        return np.asarray(handle["timestamp"], dtype=np.float64)
    if format_name == "robotics_lab_dual_arm" and "observations" in handle:
        obs = handle["observations"]
        for name in ("timestamp", "timestamps", "state_host_time_ns", "bundle_host_time_ns"):
            if name not in obs:
                continue
            values = np.asarray(obs[name], dtype=np.float64)
            if name.endswith("_ns"):
                values = values / 1e9
            return values
    return np.asarray([], dtype=np.float64)


def _episode_length(
    handle: h5py.File,
    format_name: str,
    timestamps: np.ndarray,
    camera_paths: dict[str, str],
) -> int:
    candidates: list[int] = [int(len(timestamps))] if len(timestamps) else []
    if format_name == "pika_umi_single_arm":
        for name in ("action", "observations/pose", "observations/gripper"):
            if name in handle:
                candidates.append(int(handle[name].shape[0]))
    elif format_name == "pika_umi_bimanual":
        for group_path in _bimanual_arm_groups(handle).values():
            if group_path not in handle:
                continue
            group = handle[group_path]
            for name in ("pose", "gripper", "action"):
                if name in group:
                    candidates.append(int(group[name].shape[0]))
    elif format_name == "robotics_lab_dual_arm" and "observations" in handle:
        obs = handle["observations"]
        for name in (
            "tcp_stand_left",
            "tcp_actual_stand_left",
            "tcp_stand_right",
            "tcp_actual_stand_right",
            "qpos_left",
            "qpos_right",
        ):
            if name in obs:
                candidates.append(int(obs[name].shape[0]))
    for camera_path in camera_paths.values():
        if camera_path in handle:
            candidates.append(int(handle[camera_path].shape[0]))
    return min(candidates) if candidates else 0


def _timing_summary(timestamps: np.ndarray, length: int) -> tuple[float, float]:
    if length <= 1 or len(timestamps) <= 1:
        return 0.0, 0.0
    duration = float(timestamps[length - 1] - timestamps[0])
    if not math.isfinite(duration) or duration <= 0.0:
        return 0.0, 0.0
    return duration, float(length) / duration


def _pose_frame(handle: h5py.File, format_name: str) -> str:
    pose_frame = _decode_attr(handle.attrs.get("pose_frame"))
    if pose_frame:
        return pose_frame
    if format_name == "robotics_lab_dual_arm":
        return "stand"
    return "unknown"


def _pose_format(handle: h5py.File, format_name: str) -> str:
    pose_format = _decode_attr(handle.attrs.get("pose_format"))
    if pose_format:
        return pose_format
    if format_name == "robotics_lab_dual_arm":
        return "x,y,z,qx,qy,qz,qw"
    return "unknown"


def _arm_mask_for_format(
    handle: h5py.File,
    format_name: str,
    single_arm_side: str,
) -> list[float]:
    if format_name == "pika_umi_single_arm":
        return [1.0, 0.0] if single_arm_side == "left" else [0.0, 1.0]
    if format_name == "pika_umi_bimanual":
        groups = _bimanual_arm_groups(handle)
        return [
            1.0 if groups.get("left") is not None else 0.0,
            1.0 if groups.get("right") is not None else 0.0,
        ]
    if format_name == "robotics_lab_dual_arm":
        return [1.0, 1.0]
    return [0.0, 0.0]


def _action_kind_for_format(handle: h5py.File, format_name: str) -> str:
    if format_name in {"pika_umi_single_arm", "pika_umi_bimanual"}:
        return "target_pose"
    if format_name == "robotics_lab_dual_arm" and "action" in handle:
        action = handle["action"]
        for name in (
            "tcp_delta_stand_left",
            "tcp_delta_left",
            "tcp_twist_stand_left",
            "tcp_twist_local_left",
            "tcp_twist_left",
            "tcp_delta_stand_right",
            "tcp_delta_right",
            "tcp_twist_stand_right",
            "tcp_twist_local_right",
            "tcp_twist_right",
        ):
            if name in action:
                return "delta"
    return "target_pose"


def _check_required_attrs(
    handle: h5py.File,
    manifest: DatasetManifest | None,
    warnings: list[str],
    blockers: list[str],
) -> None:
    if manifest is None:
        return
    for key, expected in manifest.required_attrs.items():
        if key not in handle.attrs:
            message = f"required_attr_missing: root attr {key} is required by dataset manifest"
            _append(warnings, message)
            _append(blockers, message)
            continue
        actual = _decode_attr(handle.attrs.get(key))
        if actual != str(expected):
            message = f"required_attr_mismatch: root attr {key}={actual!r}, expected {expected!r}"
            _append(warnings, message)
            _append(blockers, message)


def _check_pose_metadata(
    pose_frame: str,
    pose_format: str,
    manifest: DatasetManifest | None,
    warnings: list[str],
    blockers: list[str],
) -> None:
    if pose_format not in SUPPORTED_POSE_FORMATS:
        _append(warnings, f"unsupported_pose_format: {pose_format}")
    if pose_frame not in KNOWN_POSE_FRAMES:
        _append(warnings, f"unknown_pose_frame: {pose_frame}")

    if pose_frame != "stand":
        if manifest is not None and manifest.retarget_is_measured_for(pose_frame, "stand"):
            return
        transform_status = "missing"
        if manifest is not None and manifest.retarget:
            transform_status = str(manifest.retarget.get("transform_status", "missing") or "missing")
        message = (
            "retarget_required: pose_frame "
            f"{pose_frame} must have a measured retarget transform to stand "
            f"before real policy rollout; transform_status={transform_status}"
        )
        _append(warnings, f"deployment_blocker: {message}")
        _append(blockers, message)


def _check_timestamp_outliers(timestamps: np.ndarray, warnings: list[str]) -> None:
    if len(timestamps) < 3:
        return
    dt = np.diff(timestamps.astype(np.float64))
    finite = dt[np.isfinite(dt)]
    if finite.size == 0:
        _append(warnings, "timestamp_dt_outliers: all timestamp deltas are non-finite")
        return
    positive = finite[finite > 0.0]
    if positive.size == 0:
        _append(warnings, "timestamp_dt_outliers: no positive timestamp deltas")
        return
    median = float(np.median(positive))
    if median <= 0.0:
        _append(warnings, "timestamp_dt_outliers: non-positive median timestamp delta")
        return
    bad = (~np.isfinite(dt)) | (dt <= 0.0) | (np.abs(dt - median) > 0.5 * median)
    if bool(np.any(bad)):
        _append(
            warnings,
            f"timestamp_dt_outliers: {int(np.count_nonzero(bad))} dt values differ from median {median:.6f}s",
        )


def _check_gripper_presence(
    handle: h5py.File,
    format_name: str,
    warnings: list[str],
) -> None:
    if format_name == "pika_umi_single_arm" and "observations/gripper" not in handle:
        _append(warnings, "missing_gripper_field: observations/gripper is absent")
    if format_name == "pika_umi_bimanual":
        for side, group_path in _bimanual_arm_groups(handle).items():
            if group_path in handle and "gripper" not in handle[group_path]:
                _append(warnings, f"missing_gripper_field: {side} gripper is absent")
    if format_name == "robotics_lab_dual_arm" and "observations" in handle:
        obs = handle["observations"]
        if "gripper_left" not in obs and "left_gripper" not in obs:
            _append(warnings, "missing_gripper_field: left gripper is absent")
        if "gripper_right" not in obs and "right_gripper" not in obs:
            _append(warnings, "missing_gripper_field: right gripper is absent")


def _check_quaternion_norms(
    handle: h5py.File,
    format_name: str,
    length: int,
    warnings: list[str],
) -> None:
    for label, poses in _pose_arrays(handle, format_name, length):
        if poses.ndim != 2 or poses.shape[1] < 7 or poses.shape[0] == 0:
            continue
        quat = poses[:length, 3:7].astype(np.float64)
        norms = np.linalg.norm(quat, axis=1)
        bad = (~np.isfinite(norms)) | (np.abs(norms - 1.0) > 1e-2)
        if bool(np.any(bad)):
            _append(
                warnings,
                f"non_normalized_quaternion: {label} has {int(np.count_nonzero(bad))} quaternion norm outliers",
            )


def _check_action_matches_pose(
    handle: h5py.File,
    format_name: str,
    length: int,
    warnings: list[str],
) -> None:
    if length <= 0:
        return
    if format_name == "pika_umi_single_arm":
        if "action" in handle and "observations/pose" in handle:
            _check_action_pose_pair(
                np.asarray(handle["action"], dtype=np.float32)[:length, :7],
                np.asarray(handle["observations/pose"], dtype=np.float32)[:length, :7],
                "single_arm",
                warnings,
            )
    elif format_name == "pika_umi_bimanual":
        for side, group_path in _bimanual_arm_groups(handle).items():
            if group_path in handle and "action" in handle[group_path] and "pose" in handle[group_path]:
                _check_action_pose_pair(
                    np.asarray(handle[f"{group_path}/action"], dtype=np.float32)[:length, :7],
                    np.asarray(handle[f"{group_path}/pose"], dtype=np.float32)[:length, :7],
                    side,
                    warnings,
                )
    elif format_name == "robotics_lab_dual_arm" and "action" in handle and "observations" in handle:
        action = handle["action"]
        obs = handle["observations"]
        for side in ("left", "right"):
            action_name = f"target_pose_{side}"
            pose_name = f"tcp_stand_{side}"
            if action_name in action and pose_name in obs:
                _check_action_pose_pair(
                    np.asarray(action[action_name], dtype=np.float32)[:length, :7],
                    np.asarray(obs[pose_name], dtype=np.float32)[:length, :7],
                    side,
                    warnings,
                )


def _check_action_pose_pair(
    action_pose: np.ndarray,
    current_pose: np.ndarray,
    label: str,
    warnings: list[str],
) -> None:
    if action_pose.ndim != 2 or current_pose.ndim != 2:
        return
    count = min(action_pose.shape[0], current_pose.shape[0])
    if count <= 0 or action_pose.shape[1] < 7 or current_pose.shape[1] < 7:
        return
    if bool(np.allclose(action_pose[:count, :7], current_pose[:count, :7], rtol=0.0, atol=1e-7)):
        _append(
            warnings,
            f"action_matches_observation_pose: {label} action pose columns are identical to current pose",
        )


def _check_images(
    handle: h5py.File,
    camera_paths: dict[str, str],
    length: int,
    expected_length: int,
    warnings: list[str],
) -> dict[str, str]:
    encodings: dict[str, str] = {}
    for camera_name, camera_path in sorted(camera_paths.items()):
        dataset = handle[camera_path]
        encoding = _image_encoding(dataset)
        encodings[camera_name] = encoding
        if encoding not in SUPPORTED_IMAGE_ENCODINGS:
            _append(
                warnings,
                f"unsupported_image_encoding: {camera_name} encoding={encoding}",
            )
        if int(dataset.shape[0]) < expected_length:
            _append(
                warnings,
                f"missing_images: {camera_name} has {int(dataset.shape[0])} frames for episode length {expected_length}",
            )
        for index in _sample_indices(length, int(dataset.shape[0])):
            try:
                decoded = decode_hdf5_image_value(dataset[index], image_size=8)
            except Exception as exc:
                _append(
                    warnings,
                    f"corrupt_images: {camera_name} frame {index} could not decode: {exc}",
                )
                continue
            if decoded.shape != (3, 8, 8) or not bool(np.isfinite(decoded).all()):
                _append(
                    warnings,
                    f"corrupt_images: {camera_name} frame {index} decoded to invalid image",
                )
    return encodings


def _image_encoding(dataset: h5py.Dataset) -> str:
    attr = _decode_attr(dataset.attrs.get("encoding")).lower()
    if attr:
        return attr
    inferred = _infer_image_encoding(dataset)
    return inferred or "unknown"


def _infer_image_encoding(dataset: h5py.Dataset) -> str | None:
    if dataset.shape and dataset.shape[0] > 0:
        try:
            value = dataset[0]
        except Exception:
            value = None
        data = _bytes_from_hdf5_value(value)
        if data.startswith(b"\xff\xd8\xff"):
            return "jpeg"
        if data.startswith(b"\x89PNG\r\n\x1a\n"):
            return "png"
    if dataset.dtype == np.uint8 and len(dataset.shape) >= 3:
        return "rgb8"
    return None


def _bytes_from_hdf5_value(value: Any) -> bytes:
    if isinstance(value, (bytes, bytearray)):
        return bytes(value)
    if isinstance(value, np.ndarray) and value.dtype == np.uint8:
        return value.tobytes()
    try:
        return bytes(value)
    except Exception:
        return b""


def _sample_indices(length: int, dataset_length: int) -> list[int]:
    upper = min(length, dataset_length)
    if upper <= 0:
        return []
    candidates = {0, upper // 2, upper - 1}
    return sorted(index for index in candidates if 0 <= index < upper)


def _pose_arrays(
    handle: h5py.File,
    format_name: str,
    length: int,
) -> list[tuple[str, np.ndarray]]:
    out: list[tuple[str, np.ndarray]] = []
    if length <= 0:
        return out
    if format_name == "pika_umi_single_arm":
        if "observations/pose" in handle:
            out.append(("observations/pose", np.asarray(handle["observations/pose"], dtype=np.float32)[:length]))
        if "action" in handle:
            action = np.asarray(handle["action"], dtype=np.float32)[:length]
            if action.ndim == 2 and action.shape[1] >= 7:
                out.append(("action", action[:, :7]))
    elif format_name == "pika_umi_bimanual":
        for side, group_path in _bimanual_arm_groups(handle).items():
            if group_path not in handle:
                continue
            group = handle[group_path]
            if "pose" in group:
                out.append((f"{side}/pose", np.asarray(group["pose"], dtype=np.float32)[:length]))
            if "action" in group:
                action = np.asarray(group["action"], dtype=np.float32)[:length]
                if action.ndim == 2 and action.shape[1] >= 7:
                    out.append((f"{side}/action", action[:, :7]))
    elif format_name == "robotics_lab_dual_arm" and "observations" in handle:
        obs = handle["observations"]
        for name in (
            "tcp_stand_left",
            "tcp_actual_stand_left",
            "tcp_ref_stand_left",
            "tcp_stand_right",
            "tcp_actual_stand_right",
            "tcp_ref_stand_right",
        ):
            if name in obs:
                out.append((f"observations/{name}", np.asarray(obs[name], dtype=np.float32)[:length]))
    return out


def _bimanual_arm_groups(handle: h5py.File) -> dict[str, str]:
    if "observations" not in handle or not isinstance(handle["observations"], h5py.Group):
        return {}
    observations = handle["observations"]
    group_names = [
        str(name)
        for name, value in observations.items()
        if isinstance(value, h5py.Group) and "pose" in value
    ]
    out: dict[str, str] = {}
    for name in group_names:
        canonical = name.strip().lower()
        if canonical in {"left", "right"}:
            out[canonical] = f"observations/{name}"
    arm_names = _decode_attr(handle.attrs.get("arm_names"))
    for name in [part.strip() for part in arm_names.split(",") if part.strip()]:
        canonical = name.lower()
        if canonical in {"left", "right"} and name in observations:
            out.setdefault(canonical, f"observations/{name}")
    remaining = [name for name in group_names if f"observations/{name}" not in out.values()]
    for side, name in zip(("left", "right"), remaining):
        out.setdefault(side, f"observations/{name}")
    return out


def _apply_mixed_camera_warnings(episodes: list[dict[str, Any]]) -> list[str]:
    camera_sets = {tuple(episode.get("camera_names", [])) for episode in episodes}
    if len(camera_sets) <= 1:
        return []
    message = "mixed_camera_names: camera names differ across episodes"
    for episode in episodes:
        _append(episode["warnings"], message)
    return [message]


def _display_path(path: Path, root: Path) -> str:
    if root.is_file():
        return path.name
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def _rounded(value: float, digits: int) -> float:
    if not math.isfinite(float(value)):
        return 0.0
    return round(float(value), digits)


def _decode_attr(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    if isinstance(value, np.bytes_):
        return bytes(value).decode("utf-8", errors="replace")
    return str(value)


def _append(values: list[str], message: str) -> None:
    if message not in values:
        values.append(message)


def _coerce_manifest(value: str | Path | DatasetManifest | None) -> DatasetManifest | None:
    if value is None:
        return None
    if isinstance(value, DatasetManifest):
        return value
    return DatasetManifest.load(value)
