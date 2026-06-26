from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


UMI_EPISODE_SCHEMA = "robotics_lab.umi_episode.v1"
UMI_IMPORT_MANIFEST_SCHEMA = "robotics_lab.policy_runner.umi_import_manifest.v1"
UMI_CONVERSION_REPORT_SCHEMA = "robotics_lab.policy_runner.umi_conversion_report.v1"
UMI_RETARGET_SCHEMA = "robotics_lab.umi_retarget.v1"
ROBOTICS_LAB_EPISODE_SCHEMA = "robotics_lab.episode.v1"
POSE_FORMAT_XYZW = "x,y,z,qx,qy,qz,qw"
RETARGET_STATUSES = {"missing", "configured_estimate", "measured", "accepted"}
PHYSICAL_ROLLOUT_RETARGET_STATUSES = {"measured", "accepted"}
GRIPPER_UNITS = {"percent", "mm", "raw"}


@dataclass(frozen=True)
class ArmRetarget:
    # Tracker -> robot-TCP-equivalent tool offset only. There is no world
    # (steamvr->stand) transform: it was never measured and the body-frame
    # (ee_local) action representation cancels it (wiki umi-tcp-delta-frame).
    T_tcp_umi_gripper: tuple[float, float, float, float, float, float, float]
    gripper_open_close_units: str


@dataclass(frozen=True)
class UmiRetargetConfig:
    path: Path
    sha256: str
    schema: str
    status: str
    source_pose_frame: str
    left: ArmRetarget
    right: ArmRetarget
    quality: dict[str, Any]

    @property
    def is_measured(self) -> bool:
        return self.status == "measured"

    @property
    def allows_physical_rollout(self) -> bool:
        return self.status in PHYSICAL_ROLLOUT_RETARGET_STATUSES


def import_umi_session(
    input_path: str | Path,
    output_dir: str | Path,
    *,
    task: str,
    left_device: str | None = None,
    right_device: str | None = None,
    retarget_config: str | Path | UmiRetargetConfig | None = None,
    require_measured_retarget: bool = False,
    overwrite: bool = False,
) -> dict[str, Any]:
    """Create a training-ready UMI episode directory without duplicating image payloads."""

    h5py, np = _require_hdf5()
    del h5py, np

    source_root = Path(input_path)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    retarget = _coerce_retarget_config(retarget_config)
    _check_retarget_requirement(retarget, require_measured_retarget)

    episode_paths = [
        path
        for path in discover_umi_hdf5_episodes(source_root)
        if not _path_is_relative_to(path.resolve(), destination.resolve())
    ]
    if not episode_paths:
        raise ValueError(f"no UMI HDF5 episodes found under {source_root}")

    episodes: list[dict[str, Any]] = []
    for source in episode_paths:
        imported_path = _link_episode_into_output(
            source,
            source_root=source_root,
            output_dir=destination,
            overwrite=overwrite,
        )
        summary = summarize_umi_episode(source, retarget_config=retarget)
        summary["source_path"] = str(source)
        summary["imported_path"] = str(imported_path)
        episodes.append(summary)

    report = _conversion_report(
        command="umi-import",
        input_path=source_root,
        output_path=destination,
        task=task,
        episodes=episodes,
        retarget=retarget,
        left_device=left_device,
        right_device=right_device,
    )
    manifest = _manifest_from_report(report, output_dir=destination)
    _write_json(destination / "manifest.json", manifest)
    (destination / "conversion_report.md").write_text(
        render_conversion_report_markdown(report),
        encoding="utf-8",
    )
    return manifest


def convert_umi_episode(
    input_path: str | Path,
    output_path: str | Path,
    *,
    output_format: str,
    retarget_config: str | Path | UmiRetargetConfig | None = None,
    require_measured_retarget: bool = False,
    task: str | None = None,
    poses_only: bool = False,
) -> dict[str, Any]:
    """Convert one UMI episode to a FlowHdf5Dataset-compatible HDF5 target.

    When ``poses_only`` is set, camera image datasets are skipped. The poses,
    grippers, timestamps, and attrs are written as usual; the output is a slim
    episode suitable for motion (TcpTargetPose) replay/profiling that never reads
    images. The training data flow keeps the default (images included).
    """

    if output_format not in {"robotics_lab_dual_arm", "pika_bimanual"}:
        raise ValueError("output_format must be robotics_lab_dual_arm or pika_bimanual")

    h5py, np = _require_hdf5()
    del h5py, np

    source = Path(input_path)
    destination = Path(output_path)
    if source.resolve() == destination.resolve():
        raise ValueError("umi-convert output must not overwrite the input HDF5 episode")
    destination.parent.mkdir(parents=True, exist_ok=True)
    retarget = _coerce_retarget_config(retarget_config)
    _check_retarget_requirement(retarget, require_measured_retarget)

    if output_format == "robotics_lab_dual_arm":
        _write_robotics_lab_dual_arm_episode(source, destination, retarget=retarget, task=task, poses_only=poses_only)
    else:
        if poses_only:
            raise ValueError("--poses-only is only supported for --format robotics_lab_dual_arm")
        _write_pika_bimanual_episode(source, destination, retarget=retarget)

    summary = summarize_umi_episode(destination, retarget_config=retarget)
    summary["source_path"] = str(source)
    summary["imported_path"] = str(destination)
    report = _conversion_report(
        command="umi-convert",
        input_path=source,
        output_path=destination,
        task=task,
        episodes=[summary],
        retarget=retarget,
        output_format=output_format,
    )
    manifest = _manifest_from_report(report, output_dir=destination.parent)
    _write_json(destination.parent / "manifest.json", manifest)
    (destination.parent / "conversion_report.md").write_text(
        render_conversion_report_markdown(report),
        encoding="utf-8",
    )
    return manifest


def discover_umi_hdf5_episodes(root: str | Path) -> list[Path]:
    path = Path(root)
    if path.is_file():
        return [path] if path.suffix.lower() in {".hdf5", ".h5"} else []
    return sorted(
        candidate
        for candidate in path.rglob("*")
        if candidate.is_file()
        and candidate.suffix.lower() in {".hdf5", ".h5"}
        and candidate.name not in {"manifest.hdf5", "manifest.h5"}
    )


def load_umi_retarget_config(path: str | Path) -> UmiRetargetConfig:
    config_path = Path(path)
    raw = config_path.read_bytes()
    data = _load_mapping(config_path, raw)
    schema = str(data.get("schema", UMI_RETARGET_SCHEMA) or "")
    if schema != UMI_RETARGET_SCHEMA:
        raise ValueError(f"{config_path}: unsupported UMI retarget schema: {schema}")
    status = str(data.get("status", "missing") or "missing")
    if status not in RETARGET_STATUSES:
        raise ValueError(f"{config_path}: status must be one of {sorted(RETARGET_STATUSES)}, got {status!r}")
    source_pose_frame = str(data.get("source_pose_frame", "") or "")
    if not source_pose_frame:
        raise ValueError(f"{config_path}: source_pose_frame is required")
    return UmiRetargetConfig(
        path=config_path,
        sha256=hashlib.sha256(raw).hexdigest(),
        schema=schema,
        status=status,
        source_pose_frame=source_pose_frame,
        left=_arm_retarget_from_mapping(config_path, data.get("left"), side="left"),
        right=_arm_retarget_from_mapping(config_path, data.get("right"), side="right"),
        quality=dict(data.get("quality") or {}),
    )


def summarize_umi_episode(
    path: str | Path,
    *,
    retarget_config: UmiRetargetConfig | None = None,
) -> dict[str, Any]:
    h5py, np = _require_hdf5()
    from .flow_dataset import decode_hdf5_image_value

    episode_path = Path(path)
    with h5py.File(episode_path, "r") as handle:
        format_name = _detect_episode_format(handle)
        timestamps = _read_timestamps(handle)
        arm_groups = _umi_arm_groups(handle) if format_name != "robotics_lab_dual_arm" else {}
        length = _episode_length(handle, format_name, timestamps, arm_groups)
        timestamps = timestamps[:length]
        timestamp_summary = _timestamp_summary(timestamps)
        pose_frame = _pose_frame(handle, format_name)
        pose_format = _pose_format(handle, format_name)
        retarget_status = _episode_retarget_status(handle, retarget_config)
        camera_paths = _camera_paths_for_summary(handle, format_name, arm_groups)
        camera_decode = _decode_camera_samples(handle, camera_paths, length, decode_hdf5_image_value)
        per_arm = {
            side: _arm_availability_summary(handle, format_name, side, arm_groups, length, timestamps)
            for side in ("left", "right")
        }
        warnings, blockers = _summary_warnings(
            format_name=format_name,
            pose_frame=pose_frame,
            pose_format=pose_format,
            retarget_status=retarget_status,
            retarget_config=retarget_config,
        )

    return {
        "schema": UMI_CONVERSION_REPORT_SCHEMA + ".episode",
        "path": str(episode_path),
        "format_name": format_name,
        "length": int(length),
        "duration_sec": _round(timestamp_summary["duration_sec"], 6),
        "effective_hz": _round(timestamp_summary["effective_hz"], 3),
        "capture_hz": _round(_capture_hz_from_attrs(episode_path), 3),
        "pose_frame": pose_frame,
        "pose_format": pose_format,
        "retarget_status": retarget_status,
        "retarget_config_hash": retarget_config.sha256 if retarget_config is not None else "",
        "arm_mask": [
            1.0 if per_arm["left"]["present"] else 0.0,
            1.0 if per_arm["right"]["present"] else 0.0,
        ],
        "camera_names": sorted(camera_paths),
        "quality_gates": {
            "episode_count": 1,
            "frame_count": int(length),
            "duration_sec": _round(timestamp_summary["duration_sec"], 6),
            "per_arm_frame_availability": per_arm,
            "camera_decode_success": camera_decode,
            "timestamp_jitter": timestamp_summary["jitter"],
            "action_velocity_step_distribution": {
                side: per_arm[side]["action_step_distribution"] for side in ("left", "right")
            },
            "gripper_distribution": {
                side: per_arm[side]["gripper_distribution"] for side in ("left", "right")
            },
            "ik_feasibility": {
                "status": "not_run",
                "reason": "robot kinematics are not available in the UMI file/import path",
            },
            "workspace_envelope_violations": {
                "status": "not_run",
                "reason": "no workspace envelope was configured for offline UMI import",
            },
            "retarget_status": retarget_status,
        },
        "warnings": warnings,
        "deployment_blockers": blockers,
    }


def render_conversion_report_markdown(report: dict[str, Any]) -> str:
    aggregate = report.get("aggregate", {})
    retarget = report.get("retarget", {})
    lines = [
        "# UMI Import Conversion Report",
        "",
        f"Schema: `{report.get('schema', '')}`",
        f"Command: `{report.get('command', '')}`",
        f"Input: `{report.get('input_path', '')}`",
        f"Output: `{report.get('output_path', '')}`",
        "",
        "## Retarget Metadata",
        "",
        f"- Status: `{retarget.get('status', 'missing')}`",
        f"- Source pose frame: `{retarget.get('source_pose_frame', '')}`",
        f"- Config hash: `{retarget.get('sha256', '')}`",
        "",
        "## Data Quality Gates",
        "",
        f"- Episode count: {aggregate.get('episode_count', 0)}",
        f"- Frame count: {aggregate.get('frame_count', 0)}",
        f"- Duration sec: {_round(float(aggregate.get('duration_sec', 0.0) or 0.0), 6)}",
        f"- Retarget status: `{aggregate.get('retarget_status', 'missing')}`",
        f"- Camera decode failures: {aggregate.get('camera_decode_failed_count', 0)}",
        f"- Timestamp jitter outliers: {aggregate.get('timestamp_jitter_outlier_count', 0)}",
        f"- Deployment blocker count: {aggregate.get('deployment_blocker_count', 0)}",
        "",
    ]
    blockers = aggregate.get("deployment_blockers", [])
    if blockers:
        lines.extend(["## Real Rollout Blockers", ""])
        lines.extend(f"- {blocker}" for blocker in blockers)
        lines.append("")

    lines.extend(
        [
            "## Episodes",
            "",
            "| Path | Format | Frames | Hz | Arm Mask | Cameras | Warnings | Blockers |",
            "| --- | --- | ---: | ---: | --- | --- | ---: | ---: |",
        ]
    )
    for episode in report.get("episodes", []):
        lines.append(
            "| "
            f"{episode.get('path', '')} | "
            f"{episode.get('format_name', '')} | "
            f"{episode.get('length', 0)} | "
            f"{episode.get('effective_hz', 0.0)} | "
            f"{episode.get('arm_mask', [])} | "
            f"{', '.join(episode.get('camera_names', []))} | "
            f"{len(episode.get('warnings', []))} | "
            f"{len(episode.get('deployment_blockers', []))} |"
        )

    for episode in report.get("episodes", []):
        gates = episode.get("quality_gates", {})
        lines.extend(["", f"### {episode.get('path', '')}", ""])
        lines.append(f"- Duration: {gates.get('duration_sec', 0.0)} sec")
        lines.append(f"- Retarget status: `{gates.get('retarget_status', 'missing')}`")
        lines.append(
            "- Camera decode: "
            f"{json.dumps(gates.get('camera_decode_success', {}), sort_keys=True)}"
        )
        lines.append(
            "- Timestamp jitter: "
            f"{json.dumps(gates.get('timestamp_jitter', {}), sort_keys=True)}"
        )
        lines.append(
            "- IK feasibility: "
            f"{json.dumps(gates.get('ik_feasibility', {}), sort_keys=True)}"
        )
        lines.append(
            "- Workspace envelope violations: "
            f"{json.dumps(gates.get('workspace_envelope_violations', {}), sort_keys=True)}"
        )
        if episode.get("warnings"):
            lines.append("")
            lines.append("Warnings:")
            lines.extend(f"- {warning}" for warning in episode["warnings"])
        if episode.get("deployment_blockers"):
            lines.append("")
            lines.append("Deployment blockers:")
            lines.extend(f"- {blocker}" for blocker in episode["deployment_blockers"])

    return "\n".join(lines).rstrip() + "\n"


def run_umi_import_cli(args: argparse.Namespace) -> int:
    try:
        import_umi_session(
            args.input,
            args.output_dir,
            task=args.task,
            left_device=args.left_device,
            right_device=args.right_device,
            retarget_config=args.retarget_config,
            require_measured_retarget=args.require_measured_retarget,
            overwrite=args.overwrite,
        )
    except Exception as exc:
        print(f"policy_runner umi-import failed: {exc}", flush=True)
        return 1
    print(f"wrote UMI manifest: {Path(args.output_dir) / 'manifest.json'}", flush=True)
    print(f"wrote UMI conversion report: {Path(args.output_dir) / 'conversion_report.md'}", flush=True)
    return 0


def run_umi_convert_cli(args: argparse.Namespace) -> int:
    try:
        convert_umi_episode(
            args.input,
            args.output,
            output_format=args.format,
            retarget_config=args.retarget_config,
            require_measured_retarget=args.require_measured_retarget,
            poses_only=getattr(args, "poses_only", False),
        )
    except Exception as exc:
        print(f"policy_runner umi-convert failed: {exc}", flush=True)
        return 1
    output_dir = Path(args.output).parent
    print(f"wrote converted UMI episode: {args.output}", flush=True)
    print(f"wrote UMI manifest: {output_dir / 'manifest.json'}", flush=True)
    print(f"wrote UMI conversion report: {output_dir / 'conversion_report.md'}", flush=True)
    return 0


def _require_hdf5():
    try:
        import h5py  # type: ignore
        import numpy as np  # type: ignore
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "UMI import/convert requires h5py and numpy; install policy_runner[recording] or policy_runner[ml]"
        ) from exc
    return h5py, np


def _coerce_retarget_config(
    value: str | Path | UmiRetargetConfig | None,
) -> UmiRetargetConfig | None:
    if value is None:
        return None
    if isinstance(value, UmiRetargetConfig):
        return value
    return load_umi_retarget_config(value)


def _check_retarget_requirement(
    retarget: UmiRetargetConfig | None,
    require_measured_retarget: bool,
) -> None:
    if not require_measured_retarget:
        return
    status = retarget.status if retarget is not None else "missing"
    if status not in PHYSICAL_ROLLOUT_RETARGET_STATUSES:
        raise ValueError(
            "--require-measured-retarget requires retarget config status=measured or accepted; "
            f"got {status}"
        )


def _arm_retarget_from_mapping(path: Path, value: Any, *, side: str) -> ArmRetarget:
    if not isinstance(value, dict):
        raise ValueError(f"{path}: {side} retarget mapping is required")
    units = str(value.get("gripper_open_close_units", "raw") or "raw")
    if units not in GRIPPER_UNITS:
        raise ValueError(f"{path}: {side}.gripper_open_close_units must be one of {sorted(GRIPPER_UNITS)}")
    return ArmRetarget(
        T_tcp_umi_gripper=_pose_tuple(
            value.get("T_tcp_umi_gripper"),
            path=path,
            field=f"{side}.T_tcp_umi_gripper",
        ),
        gripper_open_close_units=units,
    )


def _pose_tuple(value: Any, *, path: Path, field: str) -> tuple[float, float, float, float, float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        raise ValueError(f"{path}: {field} must be [x,y,z,qx,qy,qz,qw]")
    out = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in out):
        raise ValueError(f"{path}: {field} must contain finite numbers")
    norm = math.sqrt(sum(item * item for item in out[3:7]))
    if norm <= 1e-12:
        raise ValueError(f"{path}: {field} quaternion must be nonzero")
    return out  # type: ignore[return-value]


def _link_episode_into_output(
    source: Path,
    *,
    source_root: Path,
    output_dir: Path,
    overwrite: bool,
) -> Path:
    if source_root.is_dir():
        try:
            relative = source.relative_to(source_root)
        except ValueError:
            relative = Path(source.name)
    else:
        relative = Path(source.name)
    destination = output_dir / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        if _same_file_or_link(destination, source):
            return destination
        if not overwrite:
            raise FileExistsError(f"{destination} already exists; pass --overwrite to replace it")
        destination.unlink()
    relative_target = os.path.relpath(source, destination.parent)
    try:
        destination.symlink_to(relative_target)
    except OSError:
        try:
            os.link(source, destination)
        except OSError:
            shutil.copy2(source, destination)
    return destination


def _same_file_or_link(destination: Path, source: Path) -> bool:
    try:
        return destination.samefile(source)
    except OSError:
        pass
    if destination.is_symlink():
        try:
            return (destination.parent / os.readlink(destination)).resolve() == source.resolve()
        except OSError:
            return False
    return False


def _path_is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def _detect_episode_format(handle: Any) -> str:
    schema = _decode_attr(handle.attrs.get("schema"))
    if schema == ROBOTICS_LAB_EPISODE_SCHEMA and "observations" in handle:
        return "robotics_lab_dual_arm"
    if "timestamp" in handle and "observations" in handle and _umi_arm_groups(handle):
        return "pika_umi_bimanual"
    if "timestamp" in handle and "observations/pose" in handle and "observations/gripper" in handle:
        return "pika_umi_single_arm"
    return "unsupported"


def _umi_arm_groups(handle: Any) -> dict[str, str]:
    if "observations" not in handle:
        return {}
    observations = handle["observations"]
    h5py, _ = _require_hdf5()
    if not isinstance(observations, h5py.Group):
        return {}
    out: dict[str, str] = {}
    for name, value in observations.items():
        if not isinstance(value, h5py.Group) or "pose" not in value:
            continue
        canonical = str(name).strip().lower()
        if canonical in {"left", "right"}:
            out[canonical] = f"observations/{name}"
    arm_names = _decode_attr(handle.attrs.get("arm_names"))
    for name in [part.strip() for part in arm_names.split(",") if part.strip()]:
        canonical = name.lower()
        if canonical in {"left", "right"} and name in observations:
            group = observations[name]
            if isinstance(group, h5py.Group) and "pose" in group:
                out.setdefault(canonical, f"observations/{name}")
    return out


def _episode_length(
    handle: Any,
    format_name: str,
    timestamps: Any,
    arm_groups: dict[str, str],
) -> int:
    candidates: list[int] = [int(len(timestamps))] if len(timestamps) else []
    if format_name == "pika_umi_bimanual":
        for group_path in arm_groups.values():
            group = handle[group_path]
            for name in ("pose", "gripper", "action"):
                if name in group:
                    candidates.append(int(group[name].shape[0]))
            if "images" in group:
                for dataset in group["images"].values():
                    candidates.append(int(dataset.shape[0]))
    elif format_name == "pika_umi_single_arm":
        for path in ("observations/pose", "observations/gripper", "action"):
            if path in handle:
                candidates.append(int(handle[path].shape[0]))
        if "observations/images" in handle:
            for dataset in handle["observations/images"].values():
                candidates.append(int(dataset.shape[0]))
    elif format_name == "robotics_lab_dual_arm" and "observations" in handle:
        obs = handle["observations"]
        for name in ("tcp_stand_left", "tcp_stand_right", "timestamp"):
            if name in obs:
                candidates.append(int(obs[name].shape[0]))
        if "images" in obs:
            for dataset in obs["images"].values():
                candidates.append(int(dataset.shape[0]))
    return min(candidates) if candidates else 0


def _read_timestamps(handle: Any):
    _, np = _require_hdf5()
    if "timestamp" in handle:
        return np.asarray(handle["timestamp"])
    if "observations" in handle:
        obs = handle["observations"]
        for name in ("timestamp", "timestamps", "state_host_time_ns", "bundle_host_time_ns"):
            if name in obs:
                return np.asarray(obs[name])
    return np.asarray([], dtype=np.float64)


def _timestamp_summary(timestamps: Any) -> dict[str, Any]:
    _, np = _require_hdf5()
    normalized = _timestamps_to_seconds(timestamps)
    length = int(len(normalized))
    if length <= 1:
        return {
            "duration_sec": 0.0,
            "effective_hz": 0.0,
            "jitter": {
                "median_dt_sec": 0.0,
                "max_abs_jitter_sec": 0.0,
                "outlier_count": 0,
                "non_monotonic_count": 0,
            },
        }
    duration = float(normalized[-1] - normalized[0])
    if not math.isfinite(duration) or duration <= 0.0:
        duration = 0.0
    dt = np.diff(normalized.astype(np.float64))
    finite = dt[np.isfinite(dt)]
    positive = finite[finite > 0.0]
    median = float(np.median(positive)) if positive.size else 0.0
    jitter = np.abs(dt - median) if median > 0.0 else np.asarray([], dtype=np.float64)
    outliers = int(np.count_nonzero(jitter > 0.5 * median)) if median > 0.0 else 0
    non_monotonic = int(np.count_nonzero((~np.isfinite(dt)) | (dt <= 0.0)))
    return {
        "duration_sec": duration,
        "effective_hz": float(length / duration) if duration > 0.0 else 0.0,
        "jitter": {
            "median_dt_sec": _round(median, 9),
            "max_abs_jitter_sec": _round(float(jitter.max()) if jitter.size else 0.0, 9),
            "outlier_count": outliers,
            "non_monotonic_count": non_monotonic,
        },
    }


def _timestamps_to_seconds(timestamps: Any):
    _, np = _require_hdf5()
    values = np.asarray(timestamps, dtype=np.float64)
    if values.size <= 1:
        return values
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return values
    span = float(np.nanmax(finite) - np.nanmin(finite))
    positive_dt = np.diff(values)
    positive_dt = positive_dt[np.isfinite(positive_dt) & (positive_dt > 0.0)]
    median_dt = float(np.median(positive_dt)) if positive_dt.size else 0.0
    if span > 1e6 or median_dt > 1000.0:
        return values / 1e9
    return values


def _pose_frame(handle: Any, format_name: str) -> str:
    pose_frame = _decode_attr(handle.attrs.get("pose_frame"))
    if pose_frame:
        return pose_frame
    if format_name == "robotics_lab_dual_arm":
        return "stand"
    return "unknown"


def _pose_format(handle: Any, format_name: str) -> str:
    pose_format = _decode_attr(handle.attrs.get("pose_format"))
    if pose_format:
        return pose_format
    if format_name in {"robotics_lab_dual_arm", "pika_umi_bimanual", "pika_umi_single_arm"}:
        return POSE_FORMAT_XYZW
    return "unknown"


def _episode_retarget_status(handle: Any, retarget_config: UmiRetargetConfig | None) -> str:
    if retarget_config is not None:
        return retarget_config.status
    status = _decode_attr(handle.attrs.get("retarget_status"))
    return status or "missing"


def _camera_paths_for_summary(
    handle: Any,
    format_name: str,
    arm_groups: dict[str, str],
) -> dict[str, str]:
    h5py, _ = _require_hdf5()
    out: dict[str, str] = {}
    if format_name == "pika_umi_bimanual":
        for side, group_path in arm_groups.items():
            image_path = f"{group_path}/images"
            if image_path in handle:
                for name, value in handle[image_path].items():
                    if isinstance(value, h5py.Dataset):
                        out[f"{side}_{name}"] = f"{image_path}/{name}"
    elif format_name == "pika_umi_single_arm" and "observations/images" in handle:
        for name, value in handle["observations/images"].items():
            if isinstance(value, h5py.Dataset):
                out[str(name)] = f"observations/images/{name}"
    elif format_name == "robotics_lab_dual_arm" and "observations/images" in handle:
        for name, value in handle["observations/images"].items():
            if isinstance(value, h5py.Dataset):
                out[str(name)] = f"observations/images/{name}"
    return out


def _decode_camera_samples(
    handle: Any,
    camera_paths: dict[str, str],
    length: int,
    decode_fn: Any,
) -> dict[str, Any]:
    decoded = 0
    failed = 0
    per_camera: dict[str, dict[str, int]] = {}
    for camera_name, camera_path in sorted(camera_paths.items()):
        dataset = handle[camera_path]
        indices = _sample_indices(length, int(dataset.shape[0]))
        camera_decoded = 0
        camera_failed = 0
        for index in indices:
            try:
                image = decode_fn(dataset[index], image_size=8)
                valid = image.shape == (3, 8, 8)
                if valid:
                    camera_decoded += 1
                else:
                    camera_failed += 1
            except Exception:
                camera_failed += 1
        decoded += camera_decoded
        failed += camera_failed
        per_camera[camera_name] = {
            "sampled_frames": len(indices),
            "decoded_frames": camera_decoded,
            "failed_frames": camera_failed,
            "dataset_frames": int(dataset.shape[0]),
        }
    return {
        "sampled_frame_count": int(decoded + failed),
        "decoded_frame_count": int(decoded),
        "failed_frame_count": int(failed),
        "per_camera": per_camera,
    }


def _arm_availability_summary(
    handle: Any,
    format_name: str,
    side: str,
    arm_groups: dict[str, str],
    length: int,
    timestamps: Any,
) -> dict[str, Any]:
    _, np = _require_hdf5()
    pose = _arm_pose(handle, format_name, side, arm_groups, length)
    gripper = _arm_gripper(handle, format_name, side, arm_groups, length)
    action = _arm_action(handle, format_name, side, arm_groups, length)
    action_delta = _arm_action_delta(handle, format_name, side, length)
    present = pose is not None
    pose_count = int(pose.shape[0]) if pose is not None else 0
    gripper_count = int(gripper.shape[0]) if gripper is not None else 0
    action_count = int(action.shape[0]) if action is not None else 0
    return {
        "present": bool(present),
        "pose_frame_count": pose_count,
        "gripper_frame_count": gripper_count,
        "action_frame_count": action_count,
        "missing_frame_count": max(0, int(length) - min([count for count in (pose_count, gripper_count) if count > 0], default=0)),
        "pose_finite": bool(pose is not None and np.isfinite(pose).all()),
        "gripper_distribution": _gripper_distribution(gripper),
        "action_encoding": "per_step_delta_stand" if action_delta is not None else "target_pose",
        "action_step_distribution": _action_step_distribution(
            action_delta if action_delta is not None else action if action is not None else pose,
            timestamps,
            per_step_delta=action_delta is not None,
        ),
    }


def _arm_pose(
    handle: Any,
    format_name: str,
    side: str,
    arm_groups: dict[str, str],
    length: int,
):
    _, np = _require_hdf5()
    if format_name == "pika_umi_bimanual":
        group_path = arm_groups.get(side)
        if group_path and f"{group_path}/pose" in handle:
            return np.asarray(handle[f"{group_path}/pose"], dtype=np.float32)[:length, :7]
    if format_name == "pika_umi_single_arm" and side == "left" and "observations/pose" in handle:
        return np.asarray(handle["observations/pose"], dtype=np.float32)[:length, :7]
    if format_name == "robotics_lab_dual_arm" and "observations" in handle:
        obs = handle["observations"]
        for name in (f"tcp_stand_{side}", f"tcp_actual_stand_{side}"):
            if name in obs:
                return np.asarray(obs[name], dtype=np.float32)[:length, :7]
    return None


def _arm_gripper(
    handle: Any,
    format_name: str,
    side: str,
    arm_groups: dict[str, str],
    length: int,
):
    _, np = _require_hdf5()
    if format_name == "pika_umi_bimanual":
        group_path = arm_groups.get(side)
        if group_path and f"{group_path}/gripper" in handle:
            return np.asarray(handle[f"{group_path}/gripper"], dtype=np.float32)[:length]
    if format_name == "pika_umi_single_arm" and side == "left" and "observations/gripper" in handle:
        return np.asarray(handle["observations/gripper"], dtype=np.float32)[:length]
    if format_name == "robotics_lab_dual_arm" and "observations" in handle:
        obs = handle["observations"]
        for name in (f"gripper_{side}", f"{side}_gripper"):
            if name in obs:
                return np.asarray(obs[name], dtype=np.float32)[:length]
    return None


def _arm_action(
    handle: Any,
    format_name: str,
    side: str,
    arm_groups: dict[str, str],
    length: int,
):
    _, np = _require_hdf5()
    if format_name == "pika_umi_bimanual":
        group_path = arm_groups.get(side)
        if group_path and f"{group_path}/action" in handle:
            return np.asarray(handle[f"{group_path}/action"], dtype=np.float32)[:length]
    if format_name == "pika_umi_single_arm" and side == "left" and "action" in handle:
        return np.asarray(handle["action"], dtype=np.float32)[:length]
    if format_name == "robotics_lab_dual_arm" and "action" in handle:
        action = handle["action"]
        for name in (f"tcp_target_stand_{side}", f"target_pose_{side}", f"tcp_pose_target_{side}"):
            if name in action:
                return np.asarray(action[name], dtype=np.float32)[:length]
    return None


def _arm_action_delta(
    handle: Any,
    format_name: str,
    side: str,
    length: int,
):
    _ = handle, format_name, side, length
    return None


def _gripper_distribution(values: Any | None) -> dict[str, Any]:
    _, np = _require_hdf5()
    if values is None:
        return {
            "present": False,
            "min": 0.0,
            "max": 0.0,
            "open_events": 0,
            "close_events": 0,
            "hold_events": 0,
        }
    array = np.asarray(values, dtype=np.float32)
    if array.ndim == 2 and array.shape[1] > 0:
        vector = array[:, 0]
    else:
        vector = array.reshape(-1)
    finite = vector[np.isfinite(vector)]
    if finite.size == 0:
        return {
            "present": True,
            "min": 0.0,
            "max": 0.0,
            "open_events": 0,
            "close_events": 0,
            "hold_events": 0,
        }
    delta = np.diff(finite)
    return {
        "present": True,
        "min": _round(float(finite.min()), 6),
        "max": _round(float(finite.max()), 6),
        "open_events": int(np.count_nonzero(delta > 1e-6)),
        "close_events": int(np.count_nonzero(delta < -1e-6)),
        "hold_events": int(np.count_nonzero(np.abs(delta) <= 1e-6)),
    }


def _action_step_distribution(
    values: Any | None,
    timestamps: Any,
    *,
    per_step_delta: bool = False,
) -> dict[str, Any]:
    _, np = _require_hdf5()
    if values is None:
        return {"present": False, "translation_step_m": _empty_stats(), "translation_velocity_m_s": _empty_stats()}
    array = np.asarray(values, dtype=np.float32)
    if array.ndim != 2 or array.shape[0] < 2 or array.shape[1] < 3:
        return {"present": True, "translation_step_m": _empty_stats(), "translation_velocity_m_s": _empty_stats()}
    if per_step_delta:
        step = np.linalg.norm(array[:-1, :3].astype(np.float64), axis=1)
    else:
        step = np.linalg.norm(np.diff(array[:, :3].astype(np.float64), axis=0), axis=1)
    seconds = _timestamps_to_seconds(timestamps)
    velocity = np.asarray([], dtype=np.float64)
    if len(seconds) >= len(array):
        dt = np.diff(seconds[: len(array)].astype(np.float64))
        velocity = np.divide(step, dt, out=np.zeros_like(step), where=dt > 0.0)
        velocity = velocity[np.isfinite(velocity)]
    return {
        "present": True,
        "translation_step_m": _numeric_stats(step),
        "translation_velocity_m_s": _numeric_stats(velocity),
    }


def _numeric_stats(values: Any) -> dict[str, float]:
    _, np = _require_hdf5()
    array = np.asarray(values, dtype=np.float64)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return _empty_stats()
    return {
        "min": _round(float(array.min()), 9),
        "p50": _round(float(np.percentile(array, 50)), 9),
        "p95": _round(float(np.percentile(array, 95)), 9),
        "max": _round(float(array.max()), 9),
    }


def _empty_stats() -> dict[str, float]:
    return {"min": 0.0, "p50": 0.0, "p95": 0.0, "max": 0.0}


def _summary_warnings(
    *,
    format_name: str,
    pose_frame: str,
    pose_format: str,
    retarget_status: str,
    retarget_config: UmiRetargetConfig | None,
) -> tuple[list[str], list[str]]:
    warnings: list[str] = []
    blockers: list[str] = []
    if format_name == "unsupported":
        warnings.append("unsupported_format: UMI HDF5 layout is not recognized")
        blockers.append("unsupported_format: episode cannot be used for training")
    if pose_format != POSE_FORMAT_XYZW:
        warnings.append(f"unsupported_pose_format: {pose_format}")
    if retarget_status not in RETARGET_STATUSES:
        warnings.append(f"unknown_retarget_status: {retarget_status}")
    if retarget_status not in PHYSICAL_ROLLOUT_RETARGET_STATUSES:
        blockers.append(
            "retarget_status_not_physical_rollout_ready: physical real policy rollout requires "
            f"measured or accepted UMI retarget metadata; retarget_status={retarget_status}"
        )
    return warnings, blockers


def _capture_hz_from_attrs(path: Path) -> float:
    h5py, _ = _require_hdf5()
    with h5py.File(path, "r") as handle:
        try:
            return float(handle.attrs.get("capture_hz", 0.0) or 0.0)
        except (TypeError, ValueError):
            return 0.0


def _sample_indices(length: int, dataset_length: int) -> list[int]:
    upper = min(int(length), int(dataset_length))
    if upper <= 0:
        return []
    return sorted({0, upper // 2, upper - 1})


def _write_robotics_lab_dual_arm_episode(
    source: Path,
    destination: Path,
    *,
    retarget: UmiRetargetConfig | None,
    task: str | None,
    poses_only: bool = False,
) -> None:
    h5py, np = _require_hdf5()

    with h5py.File(source, "r") as src, h5py.File(destination, "w") as dst:
        format_name = _detect_episode_format(src)
        if format_name not in {"pika_umi_bimanual", "pika_umi_single_arm"}:
            raise ValueError(f"{source}: unsupported UMI source format for robotics_lab conversion: {format_name}")
        timestamps = _read_timestamps(src)
        arm_groups = _umi_arm_groups(src)
        length = _episode_length(src, format_name, timestamps, arm_groups)
        if length <= 0:
            raise ValueError(f"{source}: cannot convert zero-frame UMI episode")
        source_pose_frame = _pose_frame(src, format_name)
        if retarget is not None and retarget.source_pose_frame != source_pose_frame:
            raise ValueError(
                f"{source}: retarget source_pose_frame={retarget.source_pose_frame} "
                f"does not match episode pose_frame={source_pose_frame}"
            )

        left_pose_source = _pose_or_identity(src, format_name, "left", arm_groups, length)
        right_pose_source = _pose_or_identity(src, format_name, "right", arm_groups, length)
        left_action_source = _action_pose_or_pose(src, format_name, "left", arm_groups, length, left_pose_source)
        right_action_source = _action_pose_or_pose(src, format_name, "right", arm_groups, length, right_pose_source)
        left_pose = _retarget_poses(left_pose_source, retarget.left if retarget is not None else None)
        right_pose = _retarget_poses(right_pose_source, retarget.right if retarget is not None else None)
        left_action_pose = _retarget_poses(left_action_source, retarget.left if retarget is not None else None)
        right_action_pose = _retarget_poses(right_action_source, retarget.right if retarget is not None else None)
        left_gripper = _gripper_or_zero(src, format_name, "left", arm_groups, length)
        right_gripper = _gripper_or_zero(src, format_name, "right", arm_groups, length)

        dst.attrs["schema"] = ROBOTICS_LAB_EPISODE_SCHEMA
        dst.attrs["source_schema"] = _decode_attr(src.attrs.get("schema")) or UMI_EPISODE_SCHEMA
        dst.attrs["source_path"] = str(source)
        dst.attrs["source_pose_frame"] = source_pose_frame
        dst.attrs["pose_format"] = POSE_FORMAT_XYZW
        dst.attrs["retarget_status"] = retarget.status if retarget is not None else "missing"
        dst.attrs["retarget_config_hash"] = retarget.sha256 if retarget is not None else ""
        dst.attrs["retarget_config_path"] = str(retarget.path) if retarget is not None else ""
        dst.attrs["retarget_source_pose_frame"] = retarget.source_pose_frame if retarget is not None else source_pose_frame
        dst.attrs["frame_count"] = int(length)
        dst.attrs["reset_tcp_stand_left"] = left_pose[0].astype(np.float32)
        dst.attrs["reset_tcp_stand_right"] = right_pose[0].astype(np.float32)
        dst.attrs["task_description"] = task or _decode_attr(src.attrs.get("task_description"))
        dst.attrs["success"] = bool(src.attrs.get("success", True))
        dst.attrs["end_reason"] = _decode_attr(src.attrs.get("end_reason")) or "offline_import"

        obs = dst.create_group("observations")
        obs.create_dataset("timestamp", data=timestamps[:length])
        obs.create_dataset("tcp_stand_left", data=left_pose.astype(np.float32), compression="gzip", compression_opts=1)
        obs.create_dataset("tcp_stand_right", data=right_pose.astype(np.float32), compression="gzip", compression_opts=1)
        obs.create_dataset("gripper_left", data=left_gripper.astype(np.float32), compression="gzip", compression_opts=1)
        obs.create_dataset("gripper_right", data=right_gripper.astype(np.float32), compression="gzip", compression_opts=1)
        if poses_only:
            dst.attrs["poses_only"] = True
        else:
            _copy_images_to_robotics_observations(src, obs, format_name, arm_groups)

        # Action is the absolute (tool-offset) target pose only. Per-step deltas are
        # derived at training time in the end-effector body frame (ee_local).
        action = dst.create_group("action")
        action.create_dataset("target_pose_left", data=left_action_pose.astype(np.float32), compression="gzip", compression_opts=1)
        action.create_dataset("target_pose_right", data=right_action_pose.astype(np.float32), compression="gzip", compression_opts=1)
        action.create_dataset("gripper_left", data=_action_gripper_or_current(src, format_name, "left", arm_groups, length, left_gripper))
        action.create_dataset("gripper_right", data=_action_gripper_or_current(src, format_name, "right", arm_groups, length, right_gripper))


def _write_pika_bimanual_episode(
    source: Path,
    destination: Path,
    *,
    retarget: UmiRetargetConfig | None,
) -> None:
    h5py, _ = _require_hdf5()
    with h5py.File(source, "r") as src, h5py.File(destination, "w") as dst:
        for key, value in src.attrs.items():
            dst.attrs[key] = value
        dst.attrs["schema"] = _decode_attr(src.attrs.get("schema")) or UMI_EPISODE_SCHEMA
        dst.attrs["retarget_status"] = retarget.status if retarget is not None else _decode_attr(src.attrs.get("retarget_status")) or "missing"
        dst.attrs["retarget_config_hash"] = retarget.sha256 if retarget is not None else ""
        dst.attrs["retarget_config_path"] = str(retarget.path) if retarget is not None else ""
        for name, item in src.items():
            src.copy(item, dst, name=name)


def _pose_or_identity(
    handle: Any,
    format_name: str,
    side: str,
    arm_groups: dict[str, str],
    length: int,
):
    _, np = _require_hdf5()
    pose = _arm_pose(handle, format_name, side, arm_groups, length)
    if pose is not None:
        return pose
    out = np.zeros((length, 7), dtype=np.float32)
    out[:, 6] = 1.0
    return out


def _action_pose_or_pose(
    handle: Any,
    format_name: str,
    side: str,
    arm_groups: dict[str, str],
    length: int,
    pose: Any,
):
    action = _arm_action(handle, format_name, side, arm_groups, length)
    if action is not None and action.ndim == 2 and action.shape[1] >= 7:
        return action[:, :7]
    return pose


def _gripper_or_zero(
    handle: Any,
    format_name: str,
    side: str,
    arm_groups: dict[str, str],
    length: int,
):
    _, np = _require_hdf5()
    gripper = _arm_gripper(handle, format_name, side, arm_groups, length)
    if gripper is None:
        return np.zeros(length, dtype=np.float32)
    array = np.asarray(gripper, dtype=np.float32)
    if array.ndim == 2 and array.shape[1] > 0:
        return array[:length, 0].reshape(-1)
    return array[:length].reshape(-1)


def _action_gripper_or_current(
    handle: Any,
    format_name: str,
    side: str,
    arm_groups: dict[str, str],
    length: int,
    current: Any,
):
    _, np = _require_hdf5()
    action = _arm_action(handle, format_name, side, arm_groups, length)
    if action is not None and action.ndim == 2 and action.shape[1] > 7:
        return np.asarray(action[:length, 7], dtype=np.float32).reshape(-1)
    return np.asarray(current, dtype=np.float32).reshape(-1)


def _retarget_poses(poses: Any, retarget: ArmRetarget | None):
    _, np = _require_hdf5()
    array = np.asarray(poses, dtype=np.float32)
    if retarget is None:
        return array.copy()
    T_tcp_umi = retarget.T_tcp_umi_gripper
    out = np.zeros_like(array, dtype=np.float32)
    inv_tcp_umi = _pose_inverse(T_tcp_umi)
    # Only the tracker->TCP-equivalent tool offset is applied; there is no
    # world (steamvr->stand) transform — that mapping was never measured and the
    # body-frame (ee_local) action representation cancels it (wiki umi-tcp-delta-frame).
    for index, pose in enumerate(array):
        converted = _pose_multiply(tuple(float(item) for item in pose[:7]), inv_tcp_umi)
        out[index, :] = np.asarray(converted, dtype=np.float32)
    return out


def _copy_images_to_robotics_observations(
    src: Any,
    obs: Any,
    format_name: str,
    arm_groups: dict[str, str],
) -> None:
    h5py, _ = _require_hdf5()
    image_paths = _camera_paths_for_summary(src, format_name, arm_groups)
    if not image_paths:
        return
    images = obs.create_group("images")
    for camera_name, camera_path in sorted(image_paths.items()):
        dataset = src[camera_path]
        src.copy(dataset, images, name=camera_name)
        if isinstance(images[camera_name], h5py.Dataset):
            images[camera_name].attrs["source_path"] = camera_path


def _pose_inverse(pose: tuple[float, float, float, float, float, float, float]):
    translation = pose[:3]
    quat = _quat_normalize(pose[3:7])
    inv_q = (-quat[0], -quat[1], -quat[2], quat[3])
    inv_t = _quat_rotate(inv_q, (-translation[0], -translation[1], -translation[2]))
    return (inv_t[0], inv_t[1], inv_t[2], inv_q[0], inv_q[1], inv_q[2], inv_q[3])


def _pose_multiply(
    a: tuple[float, float, float, float, float, float, float],
    b: tuple[float, float, float, float, float, float, float],
):
    qa = _quat_normalize(a[3:7])
    qb = _quat_normalize(b[3:7])
    rotated = _quat_rotate(qa, b[:3])
    q = _quat_multiply(qa, qb)
    return (
        a[0] + rotated[0],
        a[1] + rotated[1],
        a[2] + rotated[2],
        q[0],
        q[1],
        q[2],
        q[3],
    )


def _quat_normalize(value: tuple[float, float, float, float] | list[float] | Any):
    q = tuple(float(item) for item in value)
    norm = math.sqrt(sum(item * item for item in q))
    if not math.isfinite(norm) or norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def _quat_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _quat_normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def _quat_rotate(q: tuple[float, float, float, float], vector: tuple[float, float, float]):
    qx, qy, qz, qw = q
    vx, vy, vz = vector
    # Offline retarget conversion only: rotate a vector by a unit xyzw quaternion.
    uv = (
        qy * vz - qz * vy,
        qz * vx - qx * vz,
        qx * vy - qy * vx,
    )
    uuv = (
        qy * uv[2] - qz * uv[1],
        qz * uv[0] - qx * uv[2],
        qx * uv[1] - qy * uv[0],
    )
    return (
        vx + 2.0 * (qw * uv[0] + uuv[0]),
        vy + 2.0 * (qw * uv[1] + uuv[1]),
        vz + 2.0 * (qw * uv[2] + uuv[2]),
    )


def _conversion_report(
    *,
    command: str,
    input_path: Path,
    output_path: Path,
    task: str | None,
    episodes: list[dict[str, Any]],
    retarget: UmiRetargetConfig | None,
    left_device: str | None = None,
    right_device: str | None = None,
    output_format: str | None = None,
) -> dict[str, Any]:
    blockers = sorted(
        {
            blocker
            for episode in episodes
            for blocker in episode.get("deployment_blockers", [])
        }
    )
    warnings = sorted(
        {
            warning
            for episode in episodes
            for warning in episode.get("warnings", [])
        }
    )
    aggregate = {
        "episode_count": len(episodes),
        "frame_count": int(sum(int(episode.get("length", 0) or 0) for episode in episodes)),
        "duration_sec": _round(sum(float(episode.get("duration_sec", 0.0) or 0.0) for episode in episodes), 6),
        "camera_names": sorted({name for episode in episodes for name in episode.get("camera_names", [])}),
        "retarget_status": retarget.status if retarget is not None else "missing",
        "camera_decode_failed_count": int(
            sum(
                int(episode.get("quality_gates", {}).get("camera_decode_success", {}).get("failed_frame_count", 0))
                for episode in episodes
            )
        ),
        "timestamp_jitter_outlier_count": int(
            sum(
                int(episode.get("quality_gates", {}).get("timestamp_jitter", {}).get("outlier_count", 0))
                for episode in episodes
            )
        ),
        "warnings": warnings,
        "deployment_blockers": blockers,
        "warning_count": int(sum(len(episode.get("warnings", [])) for episode in episodes)),
        "deployment_blocker_count": int(sum(len(episode.get("deployment_blockers", [])) for episode in episodes)),
    }
    return {
        "schema": UMI_CONVERSION_REPORT_SCHEMA,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": command,
        "input_path": str(input_path),
        "output_path": str(output_path),
        "output_format": output_format or "",
        "task": task or "",
        "requested_devices": {
            "left": left_device or "",
            "right": right_device or "",
        },
        "retarget": _retarget_manifest(retarget),
        "episodes": episodes,
        "aggregate": aggregate,
    }


def _manifest_from_report(report: dict[str, Any], *, output_dir: Path) -> dict[str, Any]:
    return {
        "schema": UMI_IMPORT_MANIFEST_SCHEMA,
        "created_utc": report.get("created_utc", ""),
        "task": report.get("task", ""),
        "episodes_dir": str(output_dir),
        "command": report.get("command", ""),
        "input_path": report.get("input_path", ""),
        "output_path": report.get("output_path", ""),
        "output_format": report.get("output_format", ""),
        "retarget": report.get("retarget", {}),
        "aggregate": report.get("aggregate", {}),
        "episodes": [
            {
                "source_path": episode.get("source_path", episode.get("path", "")),
                "imported_path": episode.get("imported_path", episode.get("path", "")),
                "format_name": episode.get("format_name", ""),
                "frame_count": episode.get("length", 0),
                "duration_sec": episode.get("duration_sec", 0.0),
                "pose_frame": episode.get("pose_frame", ""),
                "pose_format": episode.get("pose_format", ""),
                "retarget_status": episode.get("retarget_status", "missing"),
                "retarget_config_hash": episode.get("retarget_config_hash", ""),
                "arm_mask": episode.get("arm_mask", []),
                "camera_names": episode.get("camera_names", []),
                "quality_gates": episode.get("quality_gates", {}),
                "warnings": episode.get("warnings", []),
                "deployment_blockers": episode.get("deployment_blockers", []),
            }
            for episode in report.get("episodes", [])
        ],
    }


def _retarget_manifest(retarget: UmiRetargetConfig | None) -> dict[str, Any]:
    if retarget is None:
        return {
            "schema": UMI_RETARGET_SCHEMA,
            "path": "",
            "sha256": "",
            "status": "missing",
            "source_pose_frame": "",
        }
    return {
        "schema": retarget.schema,
        "path": str(retarget.path),
        "sha256": retarget.sha256,
        "status": retarget.status,
        "source_pose_frame": retarget.source_pose_frame,
        "quality": retarget.quality,
    }


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _load_mapping(path: Path, raw: bytes) -> dict[str, Any]:
    text = raw.decode("utf-8")
    if path.suffix.lower() == ".json":
        loaded = json.loads(text)
    else:
        try:
            import yaml  # type: ignore
        except ModuleNotFoundError:
            loaded = _parse_simple_yaml(text)
        else:
            loaded = yaml.safe_load(text)
    if not isinstance(loaded, dict):
        raise ValueError(f"{path}: expected a mapping")
    return loaded


def _parse_simple_yaml(text: str) -> dict[str, Any]:
    root: dict[str, Any] = {}
    stack: list[tuple[int, dict[str, Any] | list[Any]]] = [(-1, root)]
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.split("#", 1)[0].rstrip()
        if not line:
            continue
        indent = len(line) - len(line.lstrip(" "))
        stripped = line.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if stripped.startswith("- "):
            if not isinstance(parent, list):
                raise ValueError(f"line {line_number}: list item without list parent")
            parent.append(_parse_scalar_or_inline_list(stripped[2:].strip()))
            continue
        key, separator, value = stripped.partition(":")
        if separator != ":":
            raise ValueError(f"line {line_number}: expected key: value")
        key = key.strip()
        value = value.strip()
        if not isinstance(parent, dict):
            raise ValueError(f"line {line_number}: mapping item without mapping parent")
        if value:
            parent[key] = _parse_scalar_or_inline_list(value)
        else:
            container: dict[str, Any] = {}
            parent[key] = container
            stack.append((indent, container))
    return root


def _parse_scalar_or_inline_list(value: str) -> Any:
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar_or_inline_list(part.strip()) for part in inner.split(",")]
    if (
        (value.startswith('"') and value.endswith('"'))
        or (value.startswith("'") and value.endswith("'"))
    ):
        return value[1:-1]
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered in {"null", "none"}:
        return None
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _decode_attr(value: Any) -> str:
    if value is None:
        return ""
    try:
        import numpy as np  # type: ignore
    except ModuleNotFoundError:
        np = None
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    if np is not None and isinstance(value, np.bytes_):
        return bytes(value).decode("utf-8", errors="replace")
    return str(value)


def _round(value: float, digits: int) -> float:
    if not math.isfinite(float(value)):
        return 0.0
    return round(float(value), digits)
