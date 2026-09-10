from __future__ import annotations

import math
import os
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np


DIAGNOSTIC_IMAGES_ENV = "FLOW_INFER_DIAGNOSTIC_IMAGES"
DIAGNOSTIC_IMAGE_MAX_BUNDLES_ENV = "FLOW_INFER_DIAGNOSTIC_IMAGE_MAX_BUNDLES"
DIAGNOSTIC_IMAGE_MAX_SECONDS_ENV = "FLOW_INFER_DIAGNOSTIC_IMAGE_MAX_SECONDS"
DEFAULT_DIAGNOSTIC_IMAGE_SECONDS = 60.0


def _positive_int(raw: object) -> int | None:
    """Env/arg -> a usable count, or None when unset or unusable."""
    if raw is None or (isinstance(raw, str) and not raw.strip()):
        return None
    try:
        return max(0, int(raw))
    except (TypeError, ValueError):
        return None


def _positive_float(raw: object) -> float | None:
    if raw is None or (isinstance(raw, str) and not raw.strip()):
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return max(0.0, value) if math.isfinite(value) else None


def rgb_image_metrics(image: np.ndarray) -> dict[str, float | int]:
    """Cheap focus and luminance indicators on a bounded-size RGB sample."""
    rgb = np.asarray(image)
    if rgb.ndim != 3 or rgb.shape[2] < 3 or rgb.size == 0:
        raise ValueError("RGB diagnostics require a non-empty HWC image")
    stride = max(1, int(np.ceil(max(rgb.shape[0], rgb.shape[1]) / 96.0)))
    sample = rgb[::stride, ::stride, :3].astype(np.float32, copy=False)
    luminance = 0.2126 * sample[..., 0] + 0.7152 * sample[..., 1] + 0.0722 * sample[..., 2]
    dx = np.diff(luminance, axis=1)
    dy = np.diff(luminance, axis=0)
    focus = 0.0
    if dx.size:
        focus += float(np.mean(dx * dx))
    if dy.size:
        focus += float(np.mean(dy * dy))
    return {
        "width": int(rgb.shape[1]),
        "height": int(rgb.shape[0]),
        "sample_stride": stride,
        "luminance_mean": float(np.mean(luminance)),
        "luminance_p05": float(np.percentile(luminance, 5.0)),
        "luminance_p95": float(np.percentile(luminance, 95.0)),
        "focus_gradient_energy": focus,
    }


def run_directory(configured: str, *, now: datetime | None = None) -> Path:
    """Per-run output directory for one inference process.

    Every run gets its own directory, whatever the operator asked for: `auto`
    names it under `logs/`, and an explicit path is treated as the PARENT with
    the run landing in a `run_<timestamp>` child.  Two runs must never share a
    directory, because snapshot filenames carry only `bundle_seq` — and
    `bundle_seq` restarts from zero whenever camera_server restarts, so a shared
    directory silently overwrites the earlier run's frames (measured 2026-09-09:
    two consecutive rollouts interleaved 1014 pairs into one fixed directory
    with no run boundary in the names).
    """
    stamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    if configured.lower() == "auto":
        return Path("logs") / f"flow_obs_{stamp}"
    return Path(configured).expanduser() / f"run_{stamp}"


def _create_run_directory(base: Path, attempts: int = 100) -> Path:
    """Create `base`, disambiguating runs that start within the same second."""
    for attempt in range(1, attempts + 1):
        candidate = base if attempt == 1 else base.with_name(f"{base.name}_{attempt}")
        try:
            candidate.mkdir(parents=True, exist_ok=False)
            return candidate
        except FileExistsError:
            continue
    raise OSError(f"no unique run directory available under {base.parent}")


class BackgroundRgbSnapshotWriter:
    """Opt-in, bounded, best-effort JPEG writer that never waits in inference."""

    def __init__(
        self,
        mode: str | None = None,
        max_bundles: int | None = None,
        max_seconds: float | None = None,
    ) -> None:
        configured = str(mode if mode is not None else os.environ.get(DIAGNOSTIC_IMAGES_ENV, "off")).strip()
        self.directory: Path | None = None
        if configured.lower() not in {"", "off"}:
            self.directory = run_directory(configured)
        self.max_bundles = _positive_int(
            max_bundles if max_bundles is not None else os.environ.get(DIAGNOSTIC_IMAGE_MAX_BUNDLES_ENV)
        )
        self.max_seconds = _positive_float(
            max_seconds if max_seconds is not None else os.environ.get(DIAGNOSTIC_IMAGE_MAX_SECONDS_ENV)
        )
        if self.max_bundles is None and self.max_seconds is None:
            # No budget asked for: capture the first minute of the run. A bundle
            # count was the old default (120 -> ~12 s at the measured ~10 Hz
            # inference rate), which was too short to contain the phase being
            # diagnosed; a duration says what the operator actually means and
            # does not drift with the inference rate.
            self.max_seconds = DEFAULT_DIAGNOSTIC_IMAGE_SECONDS
        self._queue: queue.Queue[tuple[int, dict[str, np.ndarray]] | None] = queue.Queue(maxsize=4)
        self._submitted = 0
        self._written = 0
        self._queue_drops = 0
        self._cap_drops = 0
        self._write_errors = 0
        self._submission_errors = 0
        self._first_submit_mono: float | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        if self.directory is not None and self._budget_allows_any():
            try:
                self.directory = _create_run_directory(self.directory)
                self._thread = threading.Thread(target=self._run, name="flow-rgb-snapshots", daemon=True)
                self._thread.start()
            except OSError:
                self.directory = None
                self._write_errors += 1

    def _budget_allows_any(self) -> bool:
        return (self.max_bundles is None or self.max_bundles > 0) and (
            self.max_seconds is None or self.max_seconds > 0.0
        )

    def budget_description(self) -> str:
        parts = []
        if self.max_bundles is not None:
            parts.append(f"max_bundles={self.max_bundles}")
        if self.max_seconds is not None:
            parts.append(f"max_seconds={self.max_seconds:g}")
        return " ".join(parts) if parts else "unbounded"

    def submit(self, bundle_seq: int, images: dict[str, np.ndarray]) -> None:
        if self._thread is None:
            return
        with self._lock:
            now = time.monotonic()
            if self._first_submit_mono is None:
                # The window starts at the first frame, so model load and camera
                # preflight do not eat the operator's budget.
                self._first_submit_mono = now
            if self.max_bundles is not None and self._submitted >= self.max_bundles:
                self._cap_drops += 1
                return
            if self.max_seconds is not None and now - self._first_submit_mono > self.max_seconds:
                self._cap_drops += 1
                return
            self._submitted += 1
        try:
            item = (
                int(bundle_seq),
                {side: np.asarray(images[side]).copy() for side in ("left", "right")},
            )
        except Exception:  # noqa: BLE001 - diagnostics must never affect inference.
            with self._lock:
                self._submission_errors += 1
            return
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            with self._lock:
                self._queue_drops += 1

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "enabled": self._thread is not None,
                "directory": None if self.directory is None else str(self.directory),
                "max_bundles": self.max_bundles,
                "max_seconds": self.max_seconds,
                "elapsed_sec": (
                    None if self._first_submit_mono is None
                    else round(time.monotonic() - self._first_submit_mono, 3)
                ),
                "submitted_bundles": self._submitted,
                "written_bundles": self._written,
                "queue_drops": self._queue_drops,
                "cap_drops": self._cap_drops,
                "write_errors": self._write_errors,
                "submission_errors": self._submission_errors,
                "queue_depth": self._queue.qsize(),
            }

    def close(self) -> None:
        thread = self._thread
        if thread is None:
            return
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(None)
            except (queue.Empty, queue.Full):
                pass
        thread.join(timeout=2.0)
        self._thread = None

    def _run(self) -> None:
        import cv2

        assert self.directory is not None
        while True:
            item = self._queue.get()
            if item is None:
                return
            bundle_seq, images = item
            ok = True
            try:
                for side, rgb in images.items():
                    path = self.directory / f"bundle_{bundle_seq:010d}_{side}.jpg"
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    if not cv2.imwrite(str(path), bgr, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                        ok = False
            except Exception:  # noqa: BLE001 - diagnostics must never affect motion.
                ok = False
            with self._lock:
                if ok:
                    self._written += 1
                else:
                    self._write_errors += 1
