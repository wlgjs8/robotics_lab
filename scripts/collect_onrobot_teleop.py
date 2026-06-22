#!/usr/bin/env python3
"""On-robot teleop episode collector with a viser Start button (1s delay).

PASSIVE recorder: you drive the robot with the UMI device (the existing teleop
path commands the arms); this tool only RECORDS, commanding nothing and holding no
lease. It captures camera_server frames (ZMQ bundle + /dev/shm) + the robot's own
proprio (tcp_stand) + gripper at `--record-hz`, and writes an episode in the SAME
HDF5 schema as the pika collector (pika/pika_win/episode_writer.write_episode_payload),
so it flows through the existing convert -> train pipeline.

Why on-robot (vs .40 human-UMI): the wrist images then carry the ROBOT camera's
viewpoint + robot-speed motion blur + robot gripper occlusion = exactly the live
deploy distribution (closes the appearance/viewpoint gap UMI data can't).

pose: the robot tcp_stand (stand frame), NOT steamvr -> pose_frame is rewritten to
"stand". ee_local + reset-relative conversion is frame-invariant, so this drops into
the same training format.

Run (rtx3060, robot + camera_server up, drive with UMI):
  PYTHONPATH=policy_runner ~/openpi/.venv/bin/python scripts/collect_onrobot_teleop.py \
      --out-dir data_onrobot --state-bind udp://0.0.0.0:50356
Open the printed viser URL, press "Start (1s delay)", teleop a pick&place, press
"Stop & Save".
"""
from __future__ import annotations
import argparse, sys, threading, time
from pathlib import Path

import numpy as np
import cv2
import viser

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "policy_runner"))
sys.path.insert(0, "/home/plaif/workspace/pika")  # pika_win.episode_writer (h5py+numpy only)

from policy_runner.camera_bundle_client import CameraBundleClient, resolve_frame  # noqa: E402
from policy_runner.robot_state_client import RobotStateClient  # noqa: E402
from policy_runner.flow_dataset import pose_from_state_payload  # noqa: E402
from pika_win.episode_writer import write_episode_payload  # noqa: E402
import h5py  # noqa: E402


# Inlined from flow_inference (avoid pulling torch into this base-python tool).
def _pos_or_zero(v):
    if v is None:
        return None
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if x >= 0 else None


def gripper_from_payload(payload, arm):
    ap = payload.get(arm, {})
    if not isinstance(ap, dict):
        return None
    for cn in ("gripper", "gripper_state"):
        c = ap.get(cn)
        if isinstance(c, dict):
            for k in ("gripper_position", "position", "target", "value"):
                v = _pos_or_zero(c.get(k))
                if v is not None:
                    return v
    for k in ("gripper_position", "gripper_target", "gripper", "gripper_value"):
        v = _pos_or_zero(ap.get(k))
        if v is not None:
            return v
    return None


def encode_png(frame) -> np.ndarray:
    """CameraFrame.pixels (decoded RGB HWC, per openpi_remote) -> PNG as a 1D uint8
    array (what the pika vlen-uint8 dataset expects: ds[i] = arr). Stored in BGR so
    the converter's cv2.imdecode + BGR2RGB yields correct RGB."""
    arr = np.asarray(frame.pixels)
    if arr.ndim == 3 and arr.shape[-1] == 3:
        bgr = cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".png", bgr)
        if not ok:
            raise RuntimeError("png encode failed")
        return np.asarray(buf, np.uint8).reshape(-1)
    return np.asarray(arr, np.uint8).reshape(-1)  # already encoded -> pass through


class Collector:
    def __init__(self, args):
        self.args = args
        self.arms = [a.strip() for a in args.arms.split(",") if a.strip()]
        self.cam = CameraBundleClient(zmq_endpoint=args.camera_zmq, max_age_ms=args.max_age_ms)
        self.state = RobotStateClient(args.state_bind, stale_timeout_sec=1.0)
        self.state.start()
        self.cam_keys = {a: f"{a}_realsense_color" for a in self.arms}
        self._frames: list[dict] = []
        self._recording = False
        self._lock = threading.Lock()
        self._stop = False
        self.status = "idle"
        self.dropped = 0
        self._cap = threading.Thread(target=self._capture_loop, daemon=True)
        self._cap.start()

    # ---- capture (passive) ----
    def _capture_loop(self):
        dt = 1.0 / float(self.args.record_hz)
        nxt = time.perf_counter()
        while not self._stop:
            now = time.perf_counter()
            if now < nxt:
                time.sleep(min(dt * 0.25, nxt - now)); continue
            nxt += dt
            if not self._recording:
                continue
            bundle = self.cam.poll(timeout_ms=0)
            snap = self.state.latest
            if bundle is None or snap is None:
                self.dropped += 1
                continue
            payload = snap.payload
            arms_frame = []
            ok = True
            for a in self.arms:
                fr = resolve_frame(bundle.frames, self.cam_keys[a])
                if fr is None or getattr(fr, "pixels", None) is None:
                    ok = False; break
                pose = np.asarray(pose_from_state_payload(payload, a), np.float32)  # [7] tcp_stand
                g = gripper_from_payload(payload, a)
                g = float(g) if g is not None else 0.0
                arms_frame.append({
                    "pose": pose.tolist(),
                    "gripper": [g, g],            # [measured, commanded]
                    "command": 1,                  # recording-active marker
                    "png": {"realsense_color": encode_png(fr)},
                })
            if not ok:
                self.dropped += 1; continue
            with self._lock:
                self._frames.append({"ts": time.time(), "arms": arms_frame})

    # ---- controls ----
    def arm_start(self):
        if self._recording:
            return
        with self._lock:
            self._frames = []
        self.dropped = 0
        self.status = "starting in 1.0s..."
        def go():
            time.sleep(1.0)
            self._recording = True
            self.status = "RECORDING"
        threading.Thread(target=go, daemon=True).start()

    def stop_and_save(self) -> str:
        if not self._recording and not self._frames:
            self.status = "nothing recorded"; return ""
        self._recording = False
        with self._lock:
            frames = list(self._frames)
        if len(frames) < 2:
            self.status = "too few frames"; return ""
        # build pika payload (arms_data images = {key: [png bytes,...]})
        names = self.arms
        arms_data, arms_meta = [], []
        for ai, a in enumerate(names):
            arms_data.append({
                "pose": [f["arms"][ai]["pose"] for f in frames],
                "gripper": [f["arms"][ai]["gripper"] for f in frames],
                "command": [f["arms"][ai]["command"] for f in frames],
                "images": {"realsense_color": [f["arms"][ai]["png"]["realsense_color"] for f in frames]},
            })
            arms_meta.append({"realsense_sn": "", "tracker_sn": "", "fisheye_dev": "", "calib": None})
        payload = {
            "record_hz": self.args.record_hz, "pose_tip_frame": False,
            "names": names, "ts": [f["ts"] for f in frames],
            "arms_meta": arms_meta, "arms_data": arms_data,
        }
        out_dir = Path(self.args.out_dir); out_dir.mkdir(parents=True, exist_ok=True)
        idx = len(list(out_dir.glob("episode_*.hdf5")))
        path = out_dir / f"episode_{idx:04d}.hdf5"
        write_episode_payload(str(path), payload)
        # honest frame label: this is robot stand, not steamvr
        with h5py.File(path, "a") as h:
            h.attrs["pose_frame"] = "stand"
            h.attrs["source"] = "onrobot_teleop_passive"
        self.status = f"saved {path.name} ({len(frames)} frames, {self.dropped} dropped)"
        return str(path)

    def close(self):
        self._stop = True
        try: self.cam.close()
        except Exception: pass
        try: self.state.close()
        except Exception: pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default="data_onrobot")
    ap.add_argument("--state-bind", default="udp://0.0.0.0:50356",
                    help="server state fanout slot (50356=free debug slot during make run)")
    ap.add_argument("--camera-zmq", default="tcp://127.0.0.1:5600")
    ap.add_argument("--max-age-ms", type=float, default=200.0)
    ap.add_argument("--record-hz", type=float, default=30.0)
    ap.add_argument("--arms", default="left,right")
    ap.add_argument("--viser-port", type=int, default=8081)
    args = ap.parse_args()

    c = Collector(args)
    server = viser.ViserServer(port=args.viser_port)
    server.gui.add_markdown("## On-robot teleop collector\nDrive with UMI; this only records.")
    b_start = server.gui.add_button("▶ Start (1s delay)", color="green")
    b_stop = server.gui.add_button("■ Stop & Save", color="red")
    status = server.gui.add_text("Status", initial_value="idle", disabled=True)
    frames_t = server.gui.add_text("Frames", initial_value="0", disabled=True)

    @b_start.on_click
    def _(_):
        c.arm_start()

    @b_stop.on_click
    def _(_):
        c.stop_and_save()

    print(f"[collect] viser on http://127.0.0.1:{args.viser_port}  state={args.state_bind} cam={args.camera_zmq}")
    print("[collect] press Start (1s delay), teleop with UMI, then Stop & Save.")
    try:
        while True:
            status.value = c.status
            with c._lock:
                frames_t.value = str(len(c._frames))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        c.close()


if __name__ == "__main__":
    main()
