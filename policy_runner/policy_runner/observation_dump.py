"""observation_dump — record EXACTLY what each live inference saw and answered.

One record per policy inference (the OpenPI remote path):
  <dir>/manifest.jsonl          one line per inference (see ObservationDumper.record)
  <dir>/000123_left.png         observation/left_wrist_0_rgb  (LOSSLESS PNG, RGB order preserved)
  <dir>/000123_right.png        observation/right_wrist_0_rgb
  <dir>/000123_left_depth.npy   (only when depth is sent)
  <dir>/000123.npz              state, prev_action_chunk (RTC), actions (server, model space),
                                rtc_raw_actions, actions_scaled (what the runner used)

Why lossless: the point of the dump is to RE-RUN the same inference offline with one input
changed (e.g. the velocity proprio) and attribute the chunk difference to that change alone. A
lossy image would put a second, uncontrolled difference into every comparison.

Join keys: `inference_seq` (== chunk_metadata.inference_seq that rides the chunk overlay packet
-> the bridge sidecar -> the controller capture via fol_chunk_stamp_ns), and `request_mono_ns` /
`ready_mono_ns` (CLOCK_MONOTONIC, the same clock as the controller's `mono_ns` column).

Encoding happens on a background thread (PNG ~10-20 ms per image); the inference thread only
copies the arrays and enqueues. Best-effort: any failure disables the dumper and prints once.
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time
from typing import Any

import numpy as np


class ObservationDumper:
    IMAGE_KEYS = ("observation/left_wrist_0_rgb", "observation/right_wrist_0_rgb")
    DEPTH_KEYS = ("observation/left_wrist_0_depth", "observation/right_wrist_0_depth")

    def __init__(self, directory: str, stderr=None) -> None:
        self.dir = str(directory)
        os.makedirs(self.dir, exist_ok=True)
        self._stderr = stderr
        self._q: queue.Queue = queue.Queue(maxsize=64)
        self._manifest = open(os.path.join(self.dir, "manifest.jsonl"), "a", encoding="utf-8")
        self._dead = False
        self._dropped = 0
        self._written = 0
        self._thread = threading.Thread(target=self._run, daemon=True, name="obs-dump")
        self._thread.start()
        self._write_line({"type": "session", "schema": "robotics_lab.observation_dump.v1",
                          "wall_ns": time.time_ns(), "mono_ns": time.monotonic_ns(),
                          "clock": "CLOCK_MONOTONIC (== controller mono_ns / bridge sidecar)"})

    # ------------------------------------------------------------------ producer (inference thread)
    def record(self, *, inference_seq: int, obs: dict[str, Any], result: dict[str, Any] | None,
               request_mono_ns: int, ready_mono_ns: int, extra: dict[str, Any] | None = None) -> None:
        if self._dead:
            return
        try:
            item = {
                "seq": int(inference_seq),
                "request_mono_ns": int(request_mono_ns),
                "ready_mono_ns": int(ready_mono_ns),
                "wall_ns": time.time_ns(),
                "images": {},
                "depth": {},
                "arrays": {},
                "meta": {},
            }
            for k in self.IMAGE_KEYS:
                v = obs.get(k)
                if isinstance(v, np.ndarray):
                    item["images"][k] = np.array(v, dtype=np.uint8, copy=True)
            for k in self.DEPTH_KEYS:
                v = obs.get(k)
                if isinstance(v, np.ndarray):
                    item["depth"][k] = np.array(v, copy=True)
            st = obs.get("observation/state")
            if st is not None:
                item["arrays"]["state"] = np.array(st, dtype=np.float32, copy=True)
            pac = obs.get("prev_action_chunk")
            if pac is not None:
                item["arrays"]["prev_action_chunk"] = np.array(pac, dtype=np.float32, copy=True)
            item["meta"]["prompt"] = obs.get("prompt")
            for k in ("inference_delay", "execute_horizon", "prefix_attention_schedule", "max_guidance_weight"):
                if k in obs:
                    item["meta"][k] = obs[k]
            if result is not None:
                a = result.get("actions")
                if a is not None:
                    item["arrays"]["actions"] = np.array(a, dtype=np.float32, copy=True)
                r = result.get("rtc_raw_actions")
                if r is not None:
                    item["arrays"]["rtc_raw_actions"] = np.array(r, dtype=np.float32, copy=True)
                item["meta"]["result_keys"] = sorted(str(k) for k in result.keys())
            if extra:
                item["meta"].update(extra)
            self._q.put_nowait(item)
        except queue.Full:
            self._dropped += 1
        except Exception as exc:  # noqa: BLE001 - never let the dump touch the rollout
            self._disable(f"record failed: {type(exc).__name__}: {exc}")

    def note_scaled_actions(self, inference_seq: int, actions_scaled: np.ndarray) -> None:
        """The chunk as the runner actually used it (post gripper scaling); appended as its own
        small record so it never blocks on the image encode."""
        if self._dead:
            return
        try:
            self._q.put_nowait({"seq": int(inference_seq), "scaled_only": True,
                                "arrays": {"actions_scaled": np.array(actions_scaled, dtype=np.float32, copy=True)}})
        except queue.Full:
            self._dropped += 1

    def close(self) -> None:
        if self._dead:
            return
        try:
            self._q.put(None, timeout=2.0)
            self._thread.join(timeout=15.0)
        except Exception:
            pass
        try:
            self._write_line({"type": "end", "written": self._written, "dropped": self._dropped,
                              "wall_ns": time.time_ns()})
            self._manifest.close()
        except Exception:
            pass

    # ------------------------------------------------------------------ consumer (background)
    def _run(self) -> None:
        while True:
            item = self._q.get()
            if item is None:
                return
            try:
                self._write_item(item)
            except Exception as exc:  # noqa: BLE001
                self._disable(f"write failed: {type(exc).__name__}: {exc}")
                return

    def _write_item(self, item: dict[str, Any]) -> None:
        seq = int(item["seq"])
        stem = f"{seq:06d}"
        if item.get("scaled_only"):
            path = os.path.join(self.dir, f"{stem}_scaled.npz")
            np.savez_compressed(path, **item["arrays"])
            self._write_line({"type": "scaled", "seq": seq, "file": os.path.basename(path)})
            return
        files: dict[str, str] = {}
        for k, img in item["images"].items():
            side = "left" if "left" in k else "right"
            path = os.path.join(self.dir, f"{stem}_{side}.png")
            self._save_png(path, img)
            files[k] = os.path.basename(path)
        for k, dep in item["depth"].items():
            side = "left" if "left" in k else "right"
            path = os.path.join(self.dir, f"{stem}_{side}_depth.npy")
            np.save(path, dep)
            files[k] = os.path.basename(path)
        if item["arrays"]:
            path = os.path.join(self.dir, f"{stem}.npz")
            np.savez_compressed(path, **item["arrays"])
            files["arrays"] = os.path.basename(path)
        rec = {
            "type": "inference",
            "seq": seq,
            "request_mono_ns": item["request_mono_ns"],
            "ready_mono_ns": item["ready_mono_ns"],
            "wall_ns": item["wall_ns"],
            "files": files,
            "image_shapes": {k: list(v.shape) for k, v in item["images"].items()},
            "state": item["arrays"]["state"].tolist() if "state" in item["arrays"] else None,
            "actions_shape": list(item["arrays"]["actions"].shape) if "actions" in item["arrays"] else None,
            **{k: v for k, v in item["meta"].items()},
        }
        self._write_line(rec)
        self._written += 1

    @staticmethod
    def _save_png(path: str, rgb: np.ndarray) -> None:
        try:
            import cv2
            bgr = rgb[..., ::-1] if rgb.ndim == 3 and rgb.shape[-1] == 3 else rgb
            if not cv2.imwrite(path, np.ascontiguousarray(bgr), [int(cv2.IMWRITE_PNG_COMPRESSION), 3]):
                raise RuntimeError("cv2.imwrite returned False")
        except ImportError:
            from PIL import Image
            Image.fromarray(rgb).save(path, format="PNG", compress_level=3)

    def _write_line(self, rec: dict[str, Any]) -> None:
        self._manifest.write(json.dumps(rec, separators=(",", ":"), default=_json_default) + "\n")
        self._manifest.flush()

    def _disable(self, why: str) -> None:
        self._dead = True
        try:
            print(f"[obs-dump] DISABLED: {why}", file=self._stderr, flush=True)
        except Exception:
            pass


def _json_default(o: Any):
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def load_png_rgb(path: str) -> np.ndarray:
    """Inverse of _save_png: the exact uint8 RGB array that was sent."""
    try:
        import cv2
        bgr = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if bgr is None:
            raise RuntimeError(f"cannot read {path}")
        return np.ascontiguousarray(bgr[..., ::-1]) if bgr.ndim == 3 and bgr.shape[-1] == 3 else bgr
    except ImportError:
        from PIL import Image
        return np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)
