from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import h5py
import numpy as np

from .se3 import quat_canonical

try:
    from policy_runner import hdf5_audit as _policy_audit
except Exception:  # pragma: no cover - import failure is reported in detected notes.
    _policy_audit = None


POSITION_MM_INFERENCE_THRESHOLD = 10.0
TIMESTAMP_NS_THRESHOLD = 1.0e12
TIMESTAMP_US_THRESHOLD = 1.0e9
TIMESTAMP_MS_THRESHOLD = 1.0e6
IDENTITY_COMPONENT_DOMINANCE_MARGIN = 0.05


@dataclass
class EpisodeData:
    path: str
    t_source: np.ndarray | None
    left_pose: np.ndarray | None
    right_pose: np.ndarray | None
    left_gripper: np.ndarray | None
    right_gripper: np.ndarray | None
    detected: dict


def load_episode(path: str, nominal_rate_hz: float = 30.0) -> EpisodeData:
    """Load an HDF5 episode read-only and return canonical xyzw-meter arrays.

    Missing arrays are represented as None. The source HDF5 is opened with
    mode "r" only and is never modified.
    """

    episode_path = Path(path)
    detected: dict[str, Any] = {
        "path": str(episode_path),
        "nominal_rate_hz": float(nominal_rate_hz),
        "policy_runner_hdf5_audit_reused": _policy_audit is not None,
        "notes": [],
    }
    with h5py.File(episode_path, "r") as handle:
        datasets = _list_datasets(handle)
        detected["datasets"] = datasets
        detected["root_attrs"] = {key: _decode_attr(value) for key, value in handle.attrs.items()}
        format_name = _detect_format(handle)
        detected["format_name"] = format_name
        detected["pose_frame"] = _pose_frame(handle, format_name)
        detected["pose_format"] = _pose_format(handle, format_name)
        detected["timestamp_candidates"] = _timestamp_candidates(handle)
        t_source, timestamp_meta = _select_timestamps(handle, format_name)
        detected["timestamp"] = timestamp_meta
        pose_candidates = _pose_candidates(handle, format_name)
        detected["pose_candidates"] = [_candidate_public_dict(item) for item in pose_candidates]
        gripper_candidates = _gripper_candidates(handle)
        detected["gripper_candidates"] = [_candidate_public_dict(item) for item in gripper_candidates]

        selected: dict[str, str | None] = {"left_pose": None, "right_pose": None, "left_gripper": None, "right_gripper": None}
        left_pose = _select_pose(handle, pose_candidates, "left", detected, selected)
        right_pose = _select_pose(handle, pose_candidates, "right", detected, selected)
        left_gripper = _select_gripper(handle, gripper_candidates, "left", selected)
        right_gripper = _select_gripper(handle, gripper_candidates, "right", selected)
        detected["selected"] = selected

        if t_source is None:
            detected["nominal_rate_used"] = True
            detected["notes"].append("no timestamp array detected; downstream tools must use nominal_rate_hz")
        else:
            detected["nominal_rate_used"] = False

    return EpisodeData(
        path=str(episode_path),
        t_source=t_source,
        left_pose=left_pose,
        right_pose=right_pose,
        left_gripper=left_gripper,
        right_gripper=right_gripper,
        detected=detected,
    )


def _list_datasets(handle: h5py.File) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []

    def visit(name: str, obj: h5py.Dataset | h5py.Group) -> None:
        if isinstance(obj, h5py.Dataset):
            out.append(
                {
                    "path": name,
                    "shape": tuple(int(dim) for dim in obj.shape),
                    "dtype": str(obj.dtype),
                    "attrs": {key: _decode_attr(value) for key, value in obj.attrs.items()},
                }
            )

    handle.visititems(visit)
    return out


def _detect_format(handle: h5py.File) -> str:
    if _policy_audit is not None and hasattr(_policy_audit, "_detect_format"):
        try:
            return str(_policy_audit._detect_format(handle))
        except Exception:
            pass
    if "timestamp" in handle and "observations" in handle:
        return "generic_observations"
    return "unknown"


def _pose_frame(handle: h5py.File, format_name: str) -> str:
    if _policy_audit is not None and hasattr(_policy_audit, "_pose_frame"):
        try:
            return str(_policy_audit._pose_frame(handle, format_name))
        except Exception:
            pass
    return _decode_attr(handle.attrs.get("pose_frame")) or "unknown"


def _pose_format(handle: h5py.File, format_name: str) -> str:
    if _policy_audit is not None and hasattr(_policy_audit, "_pose_format"):
        try:
            return str(_policy_audit._pose_format(handle, format_name))
        except Exception:
            pass
    return _decode_attr(handle.attrs.get("pose_format")) or "unknown"


def _select_timestamps(handle: h5py.File, format_name: str) -> tuple[np.ndarray | None, dict[str, Any]]:
    if _policy_audit is not None and hasattr(_policy_audit, "_timestamps_for_format"):
        try:
            values = np.asarray(_policy_audit._timestamps_for_format(handle, format_name), dtype=np.float64)
            if values.size:
                return values, {"key": "policy_runner_format_timestamp", "source": "policy_runner.hdf5_audit"}
        except Exception:
            pass
    candidates = _timestamp_candidates(handle)
    if not candidates:
        return None, {"key": None, "source": "not_found"}
    best = candidates[0]
    values = np.asarray(handle[best["path"]], dtype=np.float64).reshape(-1)
    values, units = _convert_timestamp_units(values, best["path"], best.get("attrs", {}))
    return values, {"key": best["path"], "source": "sniffed", "units": units}


def _timestamp_candidates(handle: h5py.File) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for name, dataset in _iter_datasets(handle):
        if dataset.ndim != 1 or not np.issubdtype(dataset.dtype, np.number):
            continue
        lowered = name.lower()
        score = 0
        if "timestamp" in lowered or "timestamps" in lowered:
            score += 4
        if lowered.endswith("/time") or lowered == "time":
            score += 3
        if "host_time" in lowered or lowered.endswith("_ns"):
            score += 2
        if score <= 0:
            continue
        out.append(
            {
                "path": name,
                "shape": tuple(int(dim) for dim in dataset.shape),
                "dtype": str(dataset.dtype),
                "attrs": {key: _decode_attr(value) for key, value in dataset.attrs.items()},
                "score": score,
            }
        )
    return sorted(out, key=lambda item: (-int(item["score"]), str(item["path"])))


@dataclass(frozen=True)
class _Candidate:
    path: str
    arm: str | None
    kind: str
    score: int
    shape: tuple[int, ...]
    dtype: str
    attrs: dict[str, str]


def _pose_candidates(handle: h5py.File, format_name: str) -> list[_Candidate]:
    known: list[_Candidate] = []
    if format_name == "pika_umi_bimanual":
        groups = _bimanual_arm_groups(handle)
        for arm, group_path in groups.items():
            for name, kind, score in (("action", "action", 90), ("target_pose", "target", 85), ("pose", "pose", 70)):
                path = f"{group_path}/{name}"
                if path in handle and _is_pose_like(handle[path]):
                    known.append(_candidate_from_dataset(path, handle[path], arm, kind, score))
    elif format_name == "pika_umi_single_arm":
        for path, kind, score in (("action", "action", 90), ("observations/pose", "pose", 70)):
            if path in handle and _is_pose_like(handle[path]):
                known.append(_candidate_from_dataset(path, handle[path], "left", kind, score))

    sniffed: list[_Candidate] = []
    for path, dataset in _iter_datasets(handle):
        if not _is_pose_like(dataset):
            continue
        lowered = path.lower()
        arm = _arm_from_path(lowered)
        kind = "pose"
        score = 20
        if "action" in lowered:
            kind = "action"
            score += 35
        if "target" in lowered:
            kind = "target"
            score += 30
        if "pose" in lowered or "tcp" in lowered:
            score += 20
        if arm is not None:
            score += 15
        sniffed.append(_candidate_from_dataset(path, dataset, arm, kind, score))

    by_path: dict[str, _Candidate] = {item.path: item for item in sniffed}
    for item in known:
        by_path[item.path] = item
    return sorted(by_path.values(), key=lambda item: (-item.score, item.path))


def _gripper_candidates(handle: h5py.File) -> list[_Candidate]:
    out: list[_Candidate] = []
    for path, dataset in _iter_datasets(handle):
        if dataset.ndim not in {1, 2} or not np.issubdtype(dataset.dtype, np.number):
            continue
        lowered = path.lower()
        if "gripper" not in lowered:
            continue
        arm = _arm_from_path(lowered)
        score = 40 + (15 if arm is not None else 0)
        out.append(_candidate_from_dataset(path, dataset, arm, "gripper", score))
    return sorted(out, key=lambda item: (-item.score, item.path))


def _select_pose(
    handle: h5py.File,
    candidates: list[_Candidate],
    arm: str,
    detected: dict[str, Any],
    selected: dict[str, str | None],
) -> np.ndarray | None:
    for candidate in candidates:
        if candidate.arm not in {arm, None}:
            continue
        raw = np.asarray(handle[candidate.path], dtype=np.float64)
        if raw.ndim != 2 or raw.shape[1] < 7:
            continue
        pose, meta = _canonical_pose(raw[:, :7], candidate, detected)
        selected[f"{arm}_pose"] = candidate.path
        detected[f"{arm}_pose_conversion"] = meta
        return pose
    return None


def _canonical_pose(raw: np.ndarray, candidate: _Candidate, detected: dict[str, Any]) -> tuple[np.ndarray, dict[str, Any]]:
    values = np.asarray(raw, dtype=np.float64)
    position = values[:, :3].copy()
    position_units = _detect_position_units(position, candidate.attrs)
    if position_units == "millimeters":
        position /= 1000.0
    quat_order = _detect_quat_order(values[:, 3:7], candidate.attrs, detected.get("pose_format", ""))
    quat_raw = values[:, 3:7] if quat_order == "xyzw" else values[:, [4, 5, 6, 3]]
    quat = np.zeros_like(quat_raw, dtype=np.float64)
    ref = None
    bad_rows = 0
    for index, q in enumerate(quat_raw):
        try:
            quat[index] = quat_canonical(q, ref=ref)
            ref = quat[index]
        except ValueError:
            quat[index] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64) if ref is None else ref
            bad_rows += 1
    pose = np.concatenate([position, quat], axis=1)
    return pose.astype(np.float64), {
        "source_key": candidate.path,
        "position_units": position_units,
        "quat_order": quat_order,
        "bad_quaternion_rows": int(bad_rows),
        "canonical_quat_order": "xyzw",
        "canonical_position_units": "meters",
    }


def _select_gripper(
    handle: h5py.File,
    candidates: list[_Candidate],
    arm: str,
    selected: dict[str, str | None],
) -> np.ndarray | None:
    for candidate in candidates:
        if candidate.arm not in {arm, None}:
            continue
        raw = np.asarray(handle[candidate.path], dtype=np.float64)
        if raw.ndim == 1:
            values = raw
        elif raw.ndim == 2 and raw.shape[1] >= 1:
            values = raw[:, 0]
        else:
            continue
        selected[f"{arm}_gripper"] = candidate.path
        return values.astype(np.float64)
    return None


def _detect_position_units(position: np.ndarray, attrs: dict[str, str]) -> str:
    unit_text = " ".join(str(value).lower() for value in attrs.values())
    if "millimeter" in unit_text or unit_text == "mm":
        return "millimeters"
    if "meter" in unit_text or unit_text == "m":
        return "meters"
    finite = np.abs(position[np.isfinite(position)])
    if finite.size and float(np.nanpercentile(finite, 95)) > POSITION_MM_INFERENCE_THRESHOLD:
        return "millimeters"
    return "meters"


def _detect_quat_order(quat: np.ndarray, attrs: dict[str, str], pose_format: str) -> str:
    text = " ".join([pose_format.lower(), *(str(value).lower() for value in attrs.values())])
    compact = text.replace(" ", "")
    if "qx,qy,qz,qw" in compact or "xyzw" in compact:
        return "xyzw"
    if "qw,qx,qy,qz" in compact or "wxyz" in compact:
        return "wxyz"
    finite = np.asarray(quat, dtype=np.float64)
    finite = finite[np.isfinite(finite).all(axis=1)]
    if finite.size == 0:
        return "xyzw"
    first_abs = float(np.nanmedian(np.abs(finite[:, 0])))
    last_abs = float(np.nanmedian(np.abs(finite[:, 3])))
    if first_abs > last_abs + IDENTITY_COMPONENT_DOMINANCE_MARGIN:
        return "wxyz"
    return "xyzw"


def _convert_timestamp_units(values: np.ndarray, path: str, attrs: dict[str, str]) -> tuple[np.ndarray, str]:
    unit_text = " ".join([path.lower(), *(str(value).lower() for value in attrs.values())])
    finite = np.abs(values[np.isfinite(values)])
    scale = 1.0
    units = "seconds"
    if "nanosecond" in unit_text or unit_text.endswith("_ns") or (finite.size and float(np.nanmedian(finite)) > TIMESTAMP_NS_THRESHOLD):
        scale = 1.0e-9
        units = "nanoseconds"
    elif "microsecond" in unit_text or unit_text.endswith("_us") or (finite.size and float(np.nanmedian(finite)) > TIMESTAMP_US_THRESHOLD):
        scale = 1.0e-6
        units = "microseconds"
    elif "millisecond" in unit_text or unit_text.endswith("_ms") or (finite.size and float(np.nanmedian(finite)) > TIMESTAMP_MS_THRESHOLD):
        scale = 1.0e-3
        units = "milliseconds"
    out = values.astype(np.float64) * scale
    if out.size and np.nanmin(out) > 1.0e6:
        out = out - out[0]
    return out, units


def _bimanual_arm_groups(handle: h5py.File) -> dict[str, str]:
    if _policy_audit is not None and hasattr(_policy_audit, "_bimanual_arm_groups"):
        try:
            groups = _policy_audit._bimanual_arm_groups(handle)
            if groups:
                return dict(groups)
        except Exception:
            pass
    if "observations" not in handle or not isinstance(handle["observations"], h5py.Group):
        return {}
    out: dict[str, str] = {}
    for side in ("left", "right"):
        if side in handle["observations"]:
            out[side] = f"observations/{side}"
    return out


def _iter_datasets(handle: h5py.File):
    out: list[tuple[str, h5py.Dataset]] = []

    def visit(name: str, obj: h5py.Dataset | h5py.Group) -> None:
        if isinstance(obj, h5py.Dataset):
            out.append((name, obj))

    handle.visititems(visit)
    return out


def _is_pose_like(dataset: h5py.Dataset) -> bool:
    return (
        dataset.ndim == 2
        and dataset.shape[1] >= 7
        and np.issubdtype(dataset.dtype, np.number)
    )


def _candidate_from_dataset(path: str, dataset: h5py.Dataset, arm: str | None, kind: str, score: int) -> _Candidate:
    return _Candidate(
        path=path,
        arm=arm,
        kind=kind,
        score=score,
        shape=tuple(int(dim) for dim in dataset.shape),
        dtype=str(dataset.dtype),
        attrs={key: _decode_attr(value) for key, value in dataset.attrs.items()},
    )


def _candidate_public_dict(candidate: _Candidate) -> dict[str, Any]:
    return {
        "path": candidate.path,
        "arm": candidate.arm,
        "kind": candidate.kind,
        "score": candidate.score,
        "shape": candidate.shape,
        "dtype": candidate.dtype,
        "attrs": candidate.attrs,
    }


def _arm_from_path(path: str) -> str | None:
    parts = {part for part in path.replace("\\", "/").split("/") if part}
    if "left" in parts or path.endswith("_left") or "_left_" in path:
        return "left"
    if "right" in parts or path.endswith("_right") or "_right_" in path:
        return "right"
    return None


def _decode_attr(value: Any) -> str:
    if _policy_audit is not None and hasattr(_policy_audit, "_decode_attr"):
        try:
            return str(_policy_audit._decode_attr(value))
        except Exception:
            pass
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    if isinstance(value, np.bytes_):
        return bytes(value).decode("utf-8", errors="replace")
    return str(value)
