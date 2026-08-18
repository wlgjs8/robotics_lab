"""chimpbin — readers for a controller-manager `func write` capture and the cm_bridge sidecar.

CHIMPBIN (DataRecorder.h): one text header line, then packed little-endian rows.
    "CHIMPBIN\\t<version>\\t<row_bytes>\\t<numpy_dtype_json>[\\tsils=<0|1>]\\n"  + N * row_bytes
The dtype is rebuilt from EACH FILE's own header (never a hard-coded schema), so old captures
still read - columns a file does not have simply are not in the array. `Capture.has(...)`
tells a consumer which schema generation it is looking at.

Sidecar (cm_bridge_node.py `Sidecar`): JSONL, one record per line, every record carrying
`mono_ns` on CLOCK_MONOTONIC - the same clock as the capture's `mono_ns` column (schema 4+).
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class Capture:
    path: str
    version: int
    sils: bool
    rows: np.ndarray            # structured array, one row per 2 ms tick
    side: str                   # "left" | "right" (from the first row)

    def has(self, *cols: str) -> bool:
        names = self.rows.dtype.names or ()
        return all(c in names for c in cols)

    def col(self, name: str, default: float = 0.0) -> np.ndarray:
        if name in (self.rows.dtype.names or ()):
            return self.rows[name]
        return np.full(len(self.rows), default)

    def vec3(self, prefix: str, suffixes=("x", "y", "z")) -> np.ndarray:
        return np.stack([self.col(f"{prefix}{s}") for s in suffixes], axis=1)

    def joints(self, prefix: str) -> np.ndarray:
        return np.stack([self.col(f"{prefix}{j}") for j in range(6)], axis=1)


def read_capture(path: str) -> Capture:
    with open(path, "rb") as f:
        header = f.readline().decode("utf-8", "replace").rstrip("\n")
        parts = header.split("\t")
        if len(parts) < 4 or parts[0] != "CHIMPBIN":
            raise ValueError(f"not a CHIMPBIN capture: {path}")
        version = int(parts[1])
        row_bytes = int(parts[2])
        dtype = np.dtype([tuple(x) for x in json.loads(parts[3])])
        if dtype.itemsize != row_bytes:
            raise ValueError(f"{path}: header row_bytes {row_bytes} != dtype itemsize {dtype.itemsize}")
        sils = any(p.strip() == "sils=1" for p in parts[4:])
        rows = np.fromfile(f, dtype=dtype)
    side = "left"
    if len(rows) and "side" in dtype.names:
        side = "right" if int(rows["side"][0]) == 1 else "left"
    return Capture(path=path, version=version, sils=sils, rows=rows, side=side)


@dataclass
class Sidecar:
    path: str
    session: dict[str, Any] = field(default_factory=dict)
    chunks: list[dict[str, Any]] = field(default_factory=list)      # type == "chunk"
    grip_cmd: list[dict[str, Any]] = field(default_factory=list)    # type == "grip_cmd"
    grip_fb: list[dict[str, Any]] = field(default_factory=list)     # type == "grip_fb"
    cmds: list[dict[str, Any]] = field(default_factory=list)        # JointTarget / lease / collision
    follow_pubs: list[dict[str, Any]] = field(default_factory=list) # type == "follow_pub" (commit mode)
    follow_steps: list[dict[str, Any]] = field(default_factory=list)# type == "follow_step"
    # index: side -> {pub_mono_ns: chunk record}
    by_stamp: dict[str, dict[int, dict[str, Any]]] = field(default_factory=dict)
    # index: side -> {pub_mono_ns: follow_pub record} (commit mode: which slice went out)
    pub_by_stamp: dict[str, dict[int, dict[str, Any]]] = field(default_factory=dict)
    by_seq: dict[int, dict[str, Any]] = field(default_factory=dict)

    def chunk_for_stamp(self, side: str, stamp_ns: int):
        return self.by_stamp.get(side, {}).get(int(stamp_ns))

    def pub_for_stamp(self, side: str, stamp_ns: int):
        """The follow_pub record (skip / abs0 / n) behind a controller stamp; None in replace mode."""
        return self.pub_by_stamp.get(side, {}).get(int(stamp_ns))

    def latest_chunk_at(self, side: str, mono_ns: int):
        """The last chunk RECEIVED for `side` at or before mono_ns (None if none)."""
        best = None
        for c in self.chunks:
            if side not in (c.get("rows") or {}) or c["mono_ns"] > mono_ns:
                continue
            if best is None or c["mono_ns"] > best["mono_ns"]:
                best = c
        return best


def read_sidecar(path: str | None) -> Sidecar | None:
    if not path or not os.path.exists(path):
        return None
    sc = Sidecar(path=path)
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            t = r.get("type")
            if t == "session":
                sc.session = r
            elif t == "chunk":
                sc.chunks.append(r)
                sc.by_seq[int(r["seq"])] = r
                for side, p in (r.get("pub_mono_ns") or {}).items():
                    sc.by_stamp.setdefault(side, {})[int(p)] = r
            elif t == "follow_pub":
                sc.follow_pubs.append(r)
            elif t == "follow_step":
                sc.follow_steps.append(r)
            elif t == "grip_cmd":
                sc.grip_cmd.append(r)
            elif t == "grip_fb":
                sc.grip_fb.append(r)
            else:
                sc.cmds.append(r)
    sc.chunks.sort(key=lambda c: c["mono_ns"])
    for r in sc.follow_pubs:   # commit mode: stamp -> the chunk it sliced (+ the slice itself)
        side, st = r.get("side"), r.get("pub_mono_ns")
        if side and st is not None and int(r["seq"]) in sc.by_seq:
            sc.by_stamp.setdefault(side, {})[int(st)] = sc.by_seq[int(r["seq"])]
            sc.pub_by_stamp.setdefault(side, {})[int(st)] = r
    return sc


def sidecar_mono_range(path: str) -> tuple[int, int] | None:
    """(first, last) mono_ns of a sidecar file, or None if unreadable/empty. Reads the first and
    the last few KB only, so scanning a directory of sidecars is cheap."""
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as f:
            head = f.read(65536).decode("utf-8", "replace").splitlines()
            f.seek(max(0, size - 65536))
            tail = f.read().decode("utf-8", "replace").splitlines()
        first = last = None
        for line in head:
            try:
                first = int(json.loads(line)["mono_ns"]); break
            except Exception:
                continue
        for line in reversed(tail):
            try:
                last = int(json.loads(line)["mono_ns"]); break
            except Exception:
                continue
        if first is None or last is None:
            return None
        return first, last
    except OSError:
        return None


def select_sidecar(captures: list["Capture"], candidates: list[str]) -> str | None:
    """The sidecar whose CLOCK_MONOTONIC span overlaps the captures' mono_ns span the most.
    Needs schema-4 captures (mono_ns); returns None otherwise or if nothing overlaps."""
    lo = hi = None
    for c in captures:
        if not c.has("mono_ns") or len(c.rows) == 0:
            continue
        m = c.rows["mono_ns"]
        lo = int(m.min()) if lo is None else min(lo, int(m.min()))
        hi = int(m.max()) if hi is None else max(hi, int(m.max()))
    if lo is None:
        return None
    best, best_ov = None, 0
    for path in candidates:
        r = sidecar_mono_range(path)
        if r is None:
            continue
        ov = min(hi, r[1]) - max(lo, r[0])
        if ov > best_ov:
            best, best_ov = path, ov
    return best


def find_capture_set(directory: str) -> tuple[list[str], str | None]:
    """The newest data_left/right pair under `directory` (recursively) and the sidecar that
    overlaps it in time (the newest cm_bridge_sidecar_*.jsonl whose mtime is >= the capture's)."""
    bins: list[tuple[float, str]] = []
    sidecars: list[tuple[float, str]] = []
    for root, _dirs, files in os.walk(directory):
        for fn in files:
            p = os.path.join(root, fn)
            if fn.startswith("data_") and fn.endswith(".bin"):
                bins.append((os.path.getmtime(p), p))
            elif fn.startswith("cm_bridge_sidecar_") and fn.endswith(".jsonl") or fn == "sidecar.jsonl":
                sidecars.append((os.path.getmtime(p), p))
    if not bins:
        return [], None
    bins.sort()
    newest_mtime = bins[-1][0]
    # take every capture written within 5 s of the newest one (the per-arm pair)
    chosen = [p for m, p in bins if newest_mtime - m < 5.0]
    sidecar = None
    if sidecars:
        # a sidecar is opened when the bridge starts and appended until it stops -> its mtime is
        # >= the capture's; pick the one with the smallest mtime that is still >= newest_mtime,
        # else the newest overall.
        later = sorted([(m, p) for m, p in sidecars if m >= newest_mtime - 5.0])
        sidecar = later[0][1] if later else sorted(sidecars)[-1][1]
    return chosen, sidecar


# ------------------------------------------------------------------------------------------------
# flow-infer observation dump (policy_runner/observation_dump.py) + reinfer variants
@dataclass
class ObsDump:
    path: str
    by_seq: dict[int, dict[str, Any]] = field(default_factory=dict)
    ready_ns: np.ndarray | None = None      # sorted ready_mono_ns
    seq_by_ready: np.ndarray | None = None  # seq aligned with ready_ns
    variants: dict[int, list[dict[str, Any]]] = field(default_factory=dict)  # seq -> reinfer variants

    def record_for_chunk(self, chunk: dict[str, Any]):
        """The inference record that produced a sidecar chunk: by chunk_metadata.inference_seq
        when the packet carried it, else the last inference READY before the chunk was received."""
        meta = chunk.get("chunk_metadata") or {}
        seq = meta.get("inference_seq")
        if seq is not None and int(seq) in self.by_seq:
            return self.by_seq[int(seq)]
        if self.ready_ns is None or len(self.ready_ns) == 0:
            return None
        i = int(np.searchsorted(self.ready_ns, int(chunk["mono_ns"]), side="right")) - 1
        if i < 0:
            return None
        return self.by_seq.get(int(self.seq_by_ready[i]))

    def image(self, rec: dict[str, Any], side: str):
        key = f"observation/{side}_wrist_0_rgb"
        fn = (rec.get("files") or {}).get(key)
        if not fn:
            return None
        try:
            import sys as _sys, os as _os
            _sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "..", "policy_runner"))
            from policy_runner.observation_dump import load_png_rgb
            return load_png_rgb(os.path.join(self.path, fn))
        except Exception:
            return None

    def actions(self, rec: dict[str, Any]):
        fn = (rec.get("files") or {}).get("arrays")
        if not fn:
            return None
        try:
            z = np.load(os.path.join(self.path, fn))
            return np.asarray(z["actions"], dtype=np.float32) if "actions" in z else None
        except Exception:
            return None


def read_obs_dump(path: str | None, variants_json: str | None = None) -> ObsDump | None:
    if not path or not os.path.exists(os.path.join(path, "manifest.jsonl")):
        return None
    d = ObsDump(path=path)
    with open(os.path.join(path, "manifest.jsonl"), encoding="utf-8") as f:
        for line in f:
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            if r.get("type") == "inference":
                d.by_seq[int(r["seq"])] = r
    if d.by_seq:
        pairs = sorted((int(r["ready_mono_ns"]), s) for s, r in d.by_seq.items())
        d.ready_ns = np.array([p[0] for p in pairs], dtype=np.int64)
        d.seq_by_ready = np.array([p[1] for p in pairs], dtype=np.int64)
    if variants_json and os.path.exists(variants_json):
        with open(variants_json, encoding="utf-8") as f:
            for v in json.load(f):
                d.variants.setdefault(int(v["seq"]), []).append(v)
    return d
