from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import numpy as np


VECTOR_COLUMNS: dict[str, int] = {
    "source_raw_target": 7,
    "conditioned_goal_after_A": 7,
    "conditioned_twist_after_A": 6,
    "reference_after_B": 7,
    "q_target": 6,
    "q_actual": 6,
    "actual_tcp": 7,
    "smd_goal_vel": 6,
    "ma_in": 6,
    "ma_out": 6,
}

SCALAR_COLUMNS = [
    "t",
    "t_source",
    "src_idx",
    "chunk_id",
    "step_id",
    "arm",
    "smd_state",
    "vel_clip_flag",
    "acc_clip_flag",
    "ik_solve_us",
    "ik_pos_err",
    "ik_ori_err",
    "ik_solution_jump_deg",
    "branch_jump_flag",
    "singular_damping_flag",
    "safety_proj_flag",
    "self_collision_flag",
    "floor_flag",
    "roi_flag",
]


class TrajectoryLogWriter:
    def __init__(self, path: str | Path, *, metadata: dict[str, Any] | None = None):
        self.path = Path(path)
        self.metadata = dict(metadata or {})
        self.rows: list[dict[str, Any]] = []

    def append(self, row: dict[str, Any]) -> None:
        self.rows.append(dict(row))

    def extend(self, rows) -> None:
        for row in rows:
            self.append(row)

    def write(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        suffix = self.path.suffix.lower()
        if suffix == ".csv":
            self._write_csv()
        elif suffix == ".jsonl":
            self._write_jsonl()
        elif suffix == ".npz":
            self._write_npz()
        else:
            raise ValueError(f"unsupported trajectory log suffix: {self.path.suffix}")
        self.write_metadata(self.path.parent / "run_meta.json")

    def write_metadata(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(_jsonable(self.metadata), indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _write_csv(self) -> None:
        flattened = [_flatten_row(row) for row in self.rows]
        fieldnames = _fieldnames(flattened)
        with self.path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(flattened)

    def _write_jsonl(self) -> None:
        with self.path.open("w", encoding="utf-8") as handle:
            for row in self.rows:
                handle.write(json.dumps(_jsonable(row), sort_keys=True) + "\n")

    def _write_npz(self) -> None:
        columns: dict[str, list[Any]] = {}
        for row in self.rows:
            for key, value in row.items():
                columns.setdefault(key, []).append(value)
        arrays = {key: np.asarray(value) for key, value in columns.items()}
        np.savez(self.path, **arrays)


class TrajectoryLogReader:
    def __init__(self, path: str | Path):
        self.path = Path(path)

    def read(self) -> list[dict[str, Any]]:
        suffix = self.path.suffix.lower()
        if suffix == ".csv":
            return self._read_csv()
        if suffix == ".jsonl":
            return self._read_jsonl()
        if suffix == ".npz":
            return self._read_npz()
        raise ValueError(f"unsupported trajectory log suffix: {self.path.suffix}")

    def read_metadata(self) -> dict[str, Any]:
        path = self.path.parent / "run_meta.json"
        if not path.exists():
            return {}
        return json.loads(path.read_text(encoding="utf-8"))

    def _read_csv(self) -> list[dict[str, Any]]:
        with self.path.open("r", newline="", encoding="utf-8") as handle:
            rows = [dict(row) for row in csv.DictReader(handle)]
        return [_unflatten_row(row) for row in rows]

    def _read_jsonl(self) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        with self.path.open("r", encoding="utf-8") as handle:
            for line in handle:
                if line.strip():
                    rows.append(json.loads(line))
        return rows

    def _read_npz(self) -> list[dict[str, Any]]:
        with np.load(self.path, allow_pickle=True) as data:
            if not data.files:
                return []
            count = int(np.asarray(data[data.files[0]]).shape[0])
            rows: list[dict[str, Any]] = []
            for index in range(count):
                row = {}
                for key in data.files:
                    value = np.asarray(data[key])
                    row[key] = value[index].item() if value[index].shape == () else value[index]
                rows.append(row)
            return rows


def _flatten_row(row: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in row.items():
        if key in VECTOR_COLUMNS:
            arr = np.asarray(value, dtype=np.float64).reshape(-1)
            for index in range(VECTOR_COLUMNS[key]):
                out[f"{key}_{index}"] = float(arr[index]) if index < arr.size else np.nan
        else:
            out[key] = value
    return out


def _unflatten_row(row: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = dict(row)
    for key, size in VECTOR_COLUMNS.items():
        values = []
        present = False
        for index in range(size):
            flat_key = f"{key}_{index}"
            if flat_key in out:
                present = True
                values.append(_parse_scalar(out.pop(flat_key)))
        if present:
            while len(values) < size:
                values.append(np.nan)
            out[key] = np.asarray(values, dtype=np.float64)
    for key, value in list(out.items()):
        out[key] = _parse_scalar(value)
    return out


def _fieldnames(rows: list[dict[str, Any]]) -> list[str]:
    ordered: list[str] = []
    for key in SCALAR_COLUMNS:
        if any(key in row for row in rows):
            ordered.append(key)
    for key, size in VECTOR_COLUMNS.items():
        flat = [f"{key}_{index}" for index in range(size)]
        if any(any(item in row for item in flat) for row in rows):
            ordered.extend(flat)
    extras = sorted({key for row in rows for key in row} - set(ordered))
    return ordered + extras


def _parse_scalar(value: Any) -> Any:
    if value == "":
        return np.nan
    if isinstance(value, str):
        for parser in (int, float):
            try:
                return parser(value)
            except ValueError:
                pass
    return value


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value

