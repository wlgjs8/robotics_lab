#!/usr/bin/env python3
"""cm_replay — 2 ms 3D replay of a controller-manager capture + the cm_bridge sidecar.

    .venv/bin/python cm_bridge/tools/cm_replay.py --capture-dir logs/record_gate_<stamp>
    .venv/bin/python cm_bridge/tools/cm_replay.py --bin data_left_*.bin data_right_*.bin \\
                                                   --sidecar logs/cm_bridge_sidecar_*.jsonl
    -> http://127.0.0.1:8082

WHAT IT SHOWS, per arm, on ONE 2 ms timeline (the capture's mono_ns column):
  solid robot        ACTUAL   = jnt_ang (measured joints) + gripper FEEDBACK % (ext1 / sidecar)
  blue ghost         COMMAND  = cmd (the controller's emitted joint command) + gripper COMMAND %
  grey ghost (opt.)  BOX REF  = jnt_ref (the box's echoed reference)
  traces             tcp_act (green) / tcp_cmd (blue) / follow ref = the IK TARGET (orange) /
                     follow cmd = the delta-integral CHAIN, what the policy ASKED for (magenta)
  yellow polyline    the CHUNK being played (from the sidecar, joined by fol_chunk_stamp_ns) with
                     the row the follower is heading to marked; a faint copy = the newest chunk
                     RECEIVED but not yet adopted (the receding-horizon replace latency)
  side panel         the numbers behind the picture at the cursor: lag/gap/tracking error, gate,
                     deviation, |F|, gripper cmd/fb + their ages, chunk seq/idx/n
  plots              gripper cmd vs fb, and tracking error / lag / |F|, with the cursor
Controls: play/pause, speed, 2 ms step, jump to next/previous chunk adoption or gripper command.

FRAMES. Everything is drawn in the controller's base_frame (the device file's `base_frame`,
= "root" in follow.yaml's telemetry_frame): tcp_* and fol_* columns are root-frame mm; the
sidecar's chunk rows are the absolute rows flow-infer published (metres, integrated from the
bridge's state fanout, which republishes the controller's cmd/pose in that same frame). The
arm URDFs are mounted with the device file's `transform` matrices, so FK of the logged joints
lands on the logged TCPs. Older captures (< schema 4) still replay cmd/act/ref (no follow /
chunk / gripper overlays: those columns do not exist in them).
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "rb_gui"))

from chimpbin import (Capture, Sidecar, find_capture_set, read_capture, read_sidecar,  # noqa: E402
                      select_sidecar, read_obs_dump)

DT_TICK = 0.002


# ------------------------------------------------------------------------------------------------
# geometry helpers
def _wxyz_from_matrix(R: np.ndarray) -> tuple[float, float, float, float]:
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s; x = (m[2, 1] - m[1, 2]) / s; y = (m[0, 2] - m[2, 0]) / s; z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s; x = 0.25 * s; y = (m[0, 1] + m[1, 0]) / s; z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s; x = (m[0, 1] + m[1, 0]) / s; y = 0.25 * s; z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s; x = (m[0, 2] + m[2, 0]) / s; y = (m[1, 2] + m[2, 1]) / s; z = 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return (w / n, x / n, y / n, z / n)


def _rot_zyx_deg(rx: float, ry: float, rz: float) -> np.ndarray:
    """ZYX euler (Rz*Ry*Rx), degrees - the recorder's rpy convention."""
    a, b, c = math.radians(rx), math.radians(ry), math.radians(rz)
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    Rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _load_device(path: str):
    """Mount transforms (m) per side + the stand descriptor from a controller-manager device file."""
    import yaml
    with open(path, encoding="utf-8") as f:
        dev = yaml.safe_load(f)
    mounts = {}
    for side in ("left", "right"):
        a = (dev.get("arms") or {}).get(side)
        if not a or "transform" not in a:
            continue
        T = np.array(a["transform"], dtype=float)
        mounts[side] = (T[:3, :3], T[:3, 3] / 1000.0)
    stand = None
    rel = dev.get("stand")
    if isinstance(rel, str):
        ddir = os.path.dirname(os.path.abspath(path))
        for cand in (os.path.join(ddir, rel), os.path.join(ddir, "params-presets", rel)):
            if os.path.isfile(cand):
                with open(cand, encoding="utf-8") as f:
                    sd = yaml.safe_load(f) or {}
                stand = {"name": sd.get("name"), "descriptor": cand,
                         "xyz_mm": sd.get("transform", {}).get("xyz", [0, 0, 0]),
                         "rpy_deg": sd.get("transform", {}).get("rpy", [0, 0, 0])}
                break
    return mounts, stand


# ------------------------------------------------------------------------------------------------
class ArmData:
    """One arm's capture, columns pulled into plain arrays (SI for the scene, mm/deg for text)."""

    def __init__(self, cap: Capture):
        self.cap = cap
        r = cap.rows
        self.n = len(r)
        self.schema4 = cap.has("mono_ns", "fol_engaged", "ext0")
        if self.schema4:
            self.mono = r["mono_ns"].astype(np.int64)
        else:
            # pre-schema-4: synthesize a monotonic axis from pc_time_s (no cross-process join)
            self.mono = (r["pc_time_s"] * 1e9).astype(np.int64)
        self.q_act = np.deg2rad(cap.joints("jnt_ang"))
        self.q_cmd = np.deg2rad(cap.joints("cmd"))
        self.q_ref = np.deg2rad(cap.joints("jnt_ref"))
        self.tcp_act = cap.vec3("tcp_act_", ("x", "y", "z")) / 1000.0
        self.tcp_cmd = cap.vec3("tcp_cmd_", ("x", "y", "z")) / 1000.0
        self.tcp_ref = cap.vec3("tcp_ref_", ("x", "y", "z")) / 1000.0
        self.state = r["state"] if "state" in (r.dtype.names or ()) else None
        self.task = r["task"] if "task" in (r.dtype.names or ()) else None
        self.f_comp = np.linalg.norm(cap.vec3("comp_dz_f", ("x", "y", "z")), axis=1)
        if self.schema4:
            self.fol_on = r["fol_engaged"].astype(bool)
            self.fol_cmd = cap.vec3("fol_cmd_", ("x", "y", "z")) / 1000.0
            self.fol_ref = cap.vec3("fol_ref_", ("x", "y", "z")) / 1000.0
            self.fol_cmd_rpy = cap.vec3("fol_cmd_", ("Rx", "Ry", "Rz"))
            self.fol_ref_rpy = cap.vec3("fol_ref_", ("Rx", "Ry", "Rz"))
            self.fol_lag = cap.col("fol_lag_mm"); self.fol_gap = cap.col("fol_gap_mm")
            self.fol_gate_t = cap.col("fol_gate_t", 1.0); self.fol_gate_r = cap.col("fol_gate_r", 1.0)
            self.chunk_stamp = r["fol_chunk_stamp_ns"].astype(np.int64)
            self.chunk_idx = r["fol_chunk_idx"].astype(int); self.chunk_n = r["fol_chunk_n"].astype(int)
            self.fol_playing = r["fol_playing"].astype(bool)
            self.dev_t = np.linalg.norm(cap.vec3("dev_t", ("x", "y", "z")), axis=1)
            self.ext = np.stack([r[f"ext{i}"] for i in range(8)], axis=1)
            self.ext_seq = r["ext_seq"].astype(int)
            self.ext_stamp = r["ext_stamp_ns"].astype(np.int64)
        else:
            z = np.zeros(self.n)
            self.fol_on = np.zeros(self.n, dtype=bool)
            self.fol_cmd = self.fol_ref = np.zeros((self.n, 3))
            self.fol_cmd_rpy = self.fol_ref_rpy = np.zeros((self.n, 3))
            self.fol_lag = self.fol_gap = z; self.fol_gate_t = self.fol_gate_r = np.ones(self.n)
            self.chunk_stamp = np.zeros(self.n, dtype=np.int64)
            self.chunk_idx = self.chunk_n = np.zeros(self.n, dtype=int)
            self.fol_playing = np.zeros(self.n, dtype=bool)
            self.dev_t = z; self.ext = np.zeros((self.n, 8)); self.ext_seq = np.zeros(self.n, dtype=int)
            self.ext_stamp = np.zeros(self.n, dtype=np.int64)
        self.track_err_mm = np.linalg.norm(self.tcp_cmd - self.tcp_act, axis=1) * 1000.0

    def index_at(self, mono_ns: int) -> int:
        i = int(np.searchsorted(self.mono, mono_ns, side="right")) - 1
        return max(0, min(self.n - 1, i))

    # gripper levels (percent) at a row: cmd from ext0, fb from ext1 (None if never published)
    def grip_cmd_at(self, i: int):
        return float(self.ext[i, 0]) if self.ext_seq[i] > 0 else None

    def grip_fb_at(self, i: int):
        return float(self.ext[i, 1]) if self.ext_seq[i] > 0 and self.ext[i, 3] > 0 else None


# ------------------------------------------------------------------------------------------------
class Replay:
    COLORS = {"act": (40, 200, 90), "cmd": (60, 120, 255), "ref": (255, 150, 30),
              "fcmd": (230, 60, 230), "chunk": (250, 220, 40), "chunk_next": (160, 150, 90),
              "boxref": (150, 150, 150)}

    def __init__(self, arms: dict[str, ArmData], sidecar: Sidecar | None, mounts, stand,
                 urdf_path: str, stand_mesh: str | None, host: str, port: int, title: str,
                 obs_dump=None):
        import viser
        from viser.extras import ViserUrdf
        from rb_servo_gui import scene as gscene

        self.arms = arms
        self.sidecar = sidecar
        self.obs = obs_dump                      # flow-infer observation dump (chimpbin.ObsDump) or None
        self._img_cache = {}                     # (seq, side) -> np.ndarray, small LRU-ish
        self._last_img_seq = None
        self.g = gscene
        self.server = viser.ViserServer(host=host, port=port, label=title)
        # master timeline = union of the arms' ticks
        allm = np.unique(np.concatenate([a.mono for a in arms.values()]))
        self.mono = allm
        self.t0 = int(allm[0]); self.t1 = int(allm[-1])
        self.dur = (self.t1 - self.t0) / 1e9
        self.cursor = 0                       # index into self.mono
        self.playing = False
        self.speed = 1.0
        self.trail_s = 2.0
        self._lock = threading.Lock()
        self._last_plot = 0.0
        self._slider_programmatic = False

        sc = self.server.scene
        sc.set_up_direction("+z")
        # a first camera that frames the cell (the user can orbit from there)
        @self.server.on_client_connect
        def _cam(client):
            try:
                client.camera.position = (1.9, -1.4, 1.35)
                client.camera.look_at = (0.35, 0.0, 0.55)
            except Exception:
                pass
        sc.add_grid("/grid", width=3.0, height=3.0, position=(0, 0, -0.75))
        sc.add_frame("/root", show_axes=True, axes_length=0.15, axes_radius=0.004)
        # stand
        if stand_mesh and os.path.exists(stand_mesh) and stand:
            try:
                import trimesh
                mesh = trimesh.load(stand_mesh, force="mesh")
                mesh.apply_scale(0.001)
                # descriptor transform (base -> stand) then the urdf-internal origin the cockpit
                # composer uses for this stand (0,0,0.01, yaw -90 deg) - see cockpit/assets/urdf/stands.
                R1 = _rot_zyx_deg(*stand["rpy_deg"]); p1 = np.array(stand["xyz_mm"]) / 1000.0
                R2 = _rot_zyx_deg(0, 0, -90.0); p2 = np.array([0, 0, 0.01])
                R = R1 @ R2; p = R1 @ p2 + p1
                sc.add_mesh_trimesh("/root/stand", mesh, position=tuple(p), wxyz=_wxyz_from_matrix(R))
            except Exception as exc:  # the stand is decoration; never block the replay on it
                print(f"[cm_replay] stand mesh skipped: {exc}")
        # arms
        from pathlib import Path
        urdf_path = Path(urdf_path)   # ViserUrdf loads a Path (a str is taken as a parsed URDF)
        self.urdf = {}
        self.base_frames = {}
        for side, a in arms.items():
            R, p = mounts.get(side, (np.eye(3), np.zeros(3)))
            base = f"/root/{side}_base"
            self.base_frames[(side, "act")] = sc.add_frame(base, wxyz=_wxyz_from_matrix(R), position=tuple(p), show_axes=False)
            self.base_frames[(side, "cmd")] = sc.add_frame(base + "_cmd", wxyz=_wxyz_from_matrix(R), position=tuple(p), show_axes=False)
            self.base_frames[(side, "ref")] = sc.add_frame(base + "_ref", wxyz=_wxyz_from_matrix(R), position=tuple(p), show_axes=False, visible=False)
            self.urdf[(side, "act")] = ViserUrdf(self.server, urdf_path, root_node_name=base)
            self.urdf[(side, "cmd")] = ViserUrdf(self.server, urdf_path, root_node_name=base + "_cmd",
                                                 mesh_color_override=(0.25, 0.5, 1.0, 0.35))
            self.urdf[(side, "ref")] = ViserUrdf(self.server, urdf_path, root_node_name=base + "_ref",
                                                 mesh_color_override=(0.6, 0.6, 0.6, 0.30))
        # per-arm dynamic scene nodes
        self.nodes: dict[tuple[str, str], object] = {}
        for side in arms:
            for key, col in (("act", "act"), ("cmd", "cmd"), ("ref", "ref"), ("fcmd", "fcmd")):
                self.nodes[(side, "pt_" + key)] = sc.add_icosphere(
                    f"/root/{side}/pt_{key}", radius=0.006, color=self.COLORS[col], position=(0, 0, 0))
            self.nodes[(side, "chunk_pt")] = sc.add_icosphere(
                f"/root/{side}/chunk_target", radius=0.008, color=self.COLORS["chunk"], position=(0, 0, 0))
            self.nodes[(side, "fcmd_frame")] = sc.add_frame(f"/root/{side}/fcmd_frame", show_axes=True,
                                                            axes_length=0.05, axes_radius=0.002, visible=False)
            self.nodes[(side, "ref_frame")] = sc.add_frame(f"/root/{side}/ref_frame", show_axes=True,
                                                           axes_length=0.05, axes_radius=0.002, visible=False)
            self.nodes[(side, "label")] = sc.add_label(f"/root/{side}/label", text=side, position=(0, 0, 0))
        self._build_gui()
        self._render(force=True)
        threading.Thread(target=self._play_loop, daemon=True).start()

    # ---------------------------------------------------------------- GUI
    def _build_gui(self):
        gui = self.server.gui
        sides = list(self.arms)
        with gui.add_folder("Timeline"):
            self.md_time = gui.add_markdown("t = 0.000 s")
            self.slider = gui.add_slider("t [s]", min=0.0, max=max(self.dur, DT_TICK), step=DT_TICK,
                                         initial_value=0.0)
            self.slider.on_update(lambda _: None if self._slider_programmatic else self._seek_seconds(self.slider.value))
            row = gui.add_button_group("step", ["|<", "-10", "-1", "+1", "+10", ">|"])
            row.on_click(lambda ev: self._step_click(ev.target.value))
            self.btn_play = gui.add_button("▶ play")
            self.btn_play.on_click(lambda _: self._toggle_play())
            self.dd_speed = gui.add_dropdown("speed", ["0.05x", "0.1x", "0.25x", "0.5x", "1x", "2x", "4x"],
                                             initial_value="0.25x")
            self.dd_speed.on_update(lambda _: setattr(self, "speed", float(self.dd_speed.value[:-1])))
            self.speed = 0.25
            nav = gui.add_button_group("jump", ["◀ chunk", "chunk ▶", "◀ grip", "grip ▶"])
            nav.on_click(lambda ev: self._jump(ev.target.value))
            self.dd_side = gui.add_dropdown("jump arm", sides, initial_value=sides[0])
        with gui.add_folder("Show"):
            self.cb_cmd_ghost = gui.add_checkbox("command ghost (blue)", True)
            self.cb_ref_ghost = gui.add_checkbox("box-ref ghost (grey)", False)
            self.cb_traces = gui.add_checkbox("traces", True)
            self.cb_full = gui.add_checkbox("full-length traces", False)
            self.sl_trail = gui.add_slider("trail [s]", min=0.2, max=10.0, step=0.1, initial_value=2.0)
            self.cb_chunk = gui.add_checkbox("chunk (adopted, yellow)", True)
            self.cb_chunk_next = gui.add_checkbox("chunk (newest received)", True)
            self.cb_frames = gui.add_checkbox("cmd/ref orientation triads", False)
            for cb in (self.cb_cmd_ghost, self.cb_ref_ghost, self.cb_traces, self.cb_full, self.cb_chunk,
                       self.cb_chunk_next, self.cb_frames):
                cb.on_update(lambda _: self._render(force=True))
            self.sl_trail.on_update(lambda _: self._render(force=True))
        with gui.add_folder("At cursor"):
            self.md_info = gui.add_markdown("")
        if self.obs is not None:
            with gui.add_folder("Policy input (the frame that made this chunk)"):
                self.md_obs = gui.add_markdown("")
                blank = np.zeros((48, 64, 3), dtype=np.uint8)
                self.img_left = gui.add_image(blank, label="left wrist (as sent)", format="jpeg", jpeg_quality=80)
                self.img_right = gui.add_image(blank, label="right wrist (as sent)", format="jpeg", jpeg_quality=80)
                self.cb_variants = gui.add_checkbox("reinfer variants (composed from the same anchor)", True)
                self.cb_variants.on_update(lambda _: self._render(force=True))
                self.md_chain = gui.add_markdown("")
        with gui.add_folder("Plots"):
            self.plots = {}
            for side in sides:
                self.plots[side] = gui.add_plotly(self._make_fig(side, 0.0), aspect=0.5)
        with gui.add_folder("Capture"):
            lines = []
            for side, a in self.arms.items():
                lines.append(f"- **{side}**: `{os.path.basename(a.cap.path)}` v{a.cap.version} "
                             f"{'SILS' if a.cap.sils else 'REAL'} rows={a.n}"
                             + ("" if a.schema4 else " (pre-schema-4: no follow/chunk/gripper)"))
            if self.sidecar:
                lines.append(f"- sidecar: `{os.path.basename(self.sidecar.path)}` chunks={len(self.sidecar.chunks)} "
                             f"grip_cmd={len(self.sidecar.grip_cmd)} grip_fb={len(self.sidecar.grip_fb)}")
            else:
                lines.append("- sidecar: none (chunk previews unavailable)")
            gui.add_markdown("\n".join(lines))

    # ---------------------------------------------------------------- transport
    def _toggle_play(self):
        self.playing = not self.playing
        self.btn_play.label = "❚❚ pause" if self.playing else "▶ play"

    def _seek_seconds(self, t: float):
        i = int(np.searchsorted(self.mono, self.t0 + int(t * 1e9), side="left"))
        self._set_cursor(i)

    def _set_cursor(self, i: int, from_slider: bool = False):
        i = max(0, min(len(self.mono) - 1, i))
        with self._lock:
            self.cursor = i
        self._render()

    def _step_click(self, which: str):
        self.playing = False; self.btn_play.label = "▶ play"
        n = len(self.mono)
        i = {"|<": 0, "-10": self.cursor - 10, "-1": self.cursor - 1, "+1": self.cursor + 1,
             "+10": self.cursor + 10, ">|": n - 1}[which]
        self._set_cursor(i)

    def _jump(self, which: str):
        self.playing = False; self.btn_play.label = "▶ play"
        a = self.arms[self.dd_side.value]
        i = a.index_at(int(self.mono[self.cursor]))
        if which.endswith("chunk ▶") or which.startswith("chunk"):
            forward = "▶" in which
            key = a.chunk_stamp
        else:
            forward = "▶" in which
            key = a.ext[:, 0]
        # runs of equal key: forward = first tick of the NEXT run; backward = first tick of the
        # current run, or of the previous run if we already stand on a run start.
        j = i
        if forward:
            while j < a.n - 1 and key[j] == key[i]:
                j += 1
        else:
            while j > 0 and key[j - 1] == key[i]:
                j -= 1
            if j == i and j > 0:
                j -= 1
                while j > 0 and key[j - 1] == key[j]:
                    j -= 1
        m = int(a.mono[j])
        self._set_cursor(int(np.searchsorted(self.mono, m, side="left")))

    def _play_loop(self):
        last = time.monotonic()
        while True:
            time.sleep(0.01)
            now = time.monotonic()
            if not self.playing:
                last = now
                continue
            adv = (now - last) * self.speed
            last = now
            with self._lock:
                i = self.cursor
            target = int(self.mono[i]) + int(adv * 1e9)
            j = int(np.searchsorted(self.mono, target, side="right")) - 1
            if j >= len(self.mono) - 1:
                self.playing = False; self.btn_play.label = "▶ play"
                j = len(self.mono) - 1
            self._set_cursor(max(i, j))

    # ---------------------------------------------------------------- render
    def _render(self, force: bool = False):
        with self._lock:
            i_master = self.cursor
        m = int(self.mono[i_master])
        t = (m - self.t0) / 1e9
        try:
            if abs(self.slider.value - t) > DT_TICK / 2:
                self._slider_programmatic = True
                self.slider.value = t
        except Exception:
            pass
        finally:
            self._slider_programmatic = False
        self.md_time.content = f"**t = {t:8.3f} s**  (tick {i_master} / {len(self.mono) - 1}, mono {m})"
        info = []
        for side, a in self.arms.items():
            i = a.index_at(m)
            self._render_arm(side, a, i, m)
            info.append(self._info_line(side, a, i, m))
        self.md_info.content = "\n\n".join(info)
        if self.obs is not None:
            self._render_obs(m)
        now = time.monotonic()
        if force or not self.playing or now - self._last_plot > 0.25:
            self._last_plot = now
            for side in self.arms:
                try:
                    self.plots[side].figure = self._make_fig(side, t)
                except Exception:
                    pass

    def _render_arm(self, side: str, a: ArmData, i: int, m: int):
        g = self.g
        grip_fb = a.grip_fb_at(i)
        grip_cmd = a.grip_cmd_at(i)
        if grip_fb is None and self.sidecar:
            grip_fb = self._sidecar_grip(self.sidecar.grip_fb, side, m)
        if grip_cmd is None and self.sidecar:
            grip_cmd = self._sidecar_grip(self.sidecar.grip_cmd, side, m)
        g._update_urdf_config(self.urdf[(side, "act")], tuple(a.q_act[i]), grip_fb)
        if self.cb_cmd_ghost.value:
            g._update_urdf_config(self.urdf[(side, "cmd")], tuple(a.q_cmd[i]), grip_cmd)
        self.base_frames[(side, "cmd")].visible = self.cb_cmd_ghost.value
        if self.cb_ref_ghost.value:
            g._update_urdf_config(self.urdf[(side, "ref")], tuple(a.q_ref[i]), grip_fb)
        self.base_frames[(side, "ref")].visible = self.cb_ref_ghost.value
        # points
        self.nodes[(side, "pt_act")].position = tuple(a.tcp_act[i])
        self.nodes[(side, "pt_cmd")].position = tuple(a.tcp_cmd[i])
        fol = bool(a.fol_on[i])
        self.nodes[(side, "pt_ref")].visible = fol
        self.nodes[(side, "pt_fcmd")].visible = fol
        if fol:
            self.nodes[(side, "pt_ref")].position = tuple(a.fol_ref[i])
            self.nodes[(side, "pt_fcmd")].position = tuple(a.fol_cmd[i])
        self.nodes[(side, "label")].position = tuple(a.tcp_act[i] + np.array([0, 0, 0.06]))
        # triads
        show_fr = self.cb_frames.value and fol
        for key, p, rpy in (("fcmd_frame", a.fol_cmd[i], a.fol_cmd_rpy[i]), ("ref_frame", a.fol_ref[i], a.fol_ref_rpy[i])):
            h = self.nodes[(side, key)]
            h.visible = show_fr
            if show_fr:
                h.position = tuple(p); h.wxyz = _wxyz_from_matrix(_rot_zyx_deg(*rpy))
        # traces
        sc = self.server.scene
        if self.cb_traces.value:
            if self.cb_full.value:
                lo = 0
            else:
                lo = a.index_at(m - int(self.sl_trail.value * 1e9))
            for key, arr, col in (("act", a.tcp_act, "act"), ("cmd", a.tcp_cmd, "cmd"),
                                  ("ref", a.fol_ref, "ref"), ("fcmd", a.fol_cmd, "fcmd")):
                seg = arr[lo:i + 1]
                if key in ("ref", "fcmd"):
                    on = a.fol_on[lo:i + 1]
                    seg = np.where(on[:, None], seg, np.nan)
                self._polyline(f"/root/{side}/trace_{key}", seg, self.COLORS[col], 2.0)
        else:
            for key in ("act", "cmd", "ref", "fcmd"):
                self._polyline(f"/root/{side}/trace_{key}", None, (0, 0, 0), 1.0)
        # chunk previews
        adopted = None
        if self.sidecar and self.cb_chunk.value and fol and a.chunk_stamp[i] != 0:
            adopted = self.sidecar.chunk_for_stamp(side, int(a.chunk_stamp[i]))
        if adopted is not None:
            rows = np.array(adopted["rows"].get(side, []), dtype=float)
            # commit mode: the controller holds a SLICE of this chunk (follow_pub: orig_skip = first
            # policy row; step_of_sub maps the controller's sub-delta idx to a policy row when the
            # bridge time-stretched over-envelope steps)
            pubrec = self.sidecar.pub_for_stamp(side, int(a.chunk_stamp[i]))
            skip = int(pubrec.get("orig_skip", pubrec.get("skip")) or 0) if pubrec else 0
            sos = (pubrec or {}).get("step_of_sub")
            if len(rows):
                self._polyline(f"/root/{side}/chunk", rows[skip:, :3] if skip < len(rows) else rows[:, :3],
                               self.COLORS["chunk"], 4.0)
                ci = int(a.chunk_idx[i])
                if isinstance(sos, list) and sos:
                    k = int(sos[min(max(ci - 1, 0), len(sos) - 1)]) if ci > 0 else int(sos[0])
                else:
                    k = skip + ci
                k = min(max(k, 0), len(rows) - 1)
                self.nodes[(side, "chunk_pt")].visible = True
                self.nodes[(side, "chunk_pt")].position = tuple(rows[k, :3])
        else:
            self._polyline(f"/root/{side}/chunk", None, (0, 0, 0), 1.0)
            self.nodes[(side, "chunk_pt")].visible = False
        latest = None
        if self.sidecar and self.cb_chunk_next.value:
            latest = self.sidecar.latest_chunk_at(side, m)
            if latest is not None and adopted is not None and latest is adopted:
                latest = None
        if latest is not None:
            rows = np.array(latest["rows"].get(side, []), dtype=float)
            self._polyline(f"/root/{side}/chunk_next", rows[:, :3] if len(rows) else None,
                           self.COLORS["chunk_next"], 2.0)
        else:
            self._polyline(f"/root/{side}/chunk_next", None, (0, 0, 0), 1.0)

    def _polyline(self, name: str, pts, color, width):
        sc = self.server.scene
        if pts is None or len(pts) < 2:
            if name in getattr(self, "_lines", {}):
                self._lines[name].visible = False
            return
        pts = np.asarray(pts, dtype=float)
        # split on NaN gaps
        segs = np.stack([pts[:-1], pts[1:]], axis=1)          # (N-1, 2, 3)
        ok = ~np.isnan(segs).any(axis=(1, 2))
        segs = segs[ok]
        if len(segs) == 0:
            if name in getattr(self, "_lines", {}):
                self._lines[name].visible = False
            return
        cols = np.tile(np.array(color, dtype=np.uint8), (len(segs), 2, 1))
        if not hasattr(self, "_lines"):
            self._lines = {}
        h = self._lines.get(name)
        if h is None:
            self._lines[name] = sc.add_line_segments(name, points=segs, colors=cols, line_width=width)
        else:
            h.points = segs; h.colors = cols; h.visible = True

    # ---------------------------------------------------------------- policy input / variants
    def _adopted_chunk(self, side: str, m: int):
        a = self.arms.get(side)
        if a is None or self.sidecar is None:
            return None
        i = a.index_at(m)
        if not a.fol_on[i] or a.chunk_stamp[i] == 0:
            return None
        return self.sidecar.chunk_for_stamp(side, int(a.chunk_stamp[i]))

    def _render_obs(self, m: int):
        # the frame that produced the ADOPTED chunk of the "jump arm" (both arms share one
        # inference; use whichever arm has an adopted chunk right now)
        chunk = None
        for side in (self.dd_side.value, *[s for s in self.arms if s != self.dd_side.value]):
            chunk = self._adopted_chunk(side, m)
            if chunk is not None:
                break
        rec = self.obs.record_for_chunk(chunk) if chunk is not None else None
        if rec is None:
            self.md_obs.content = "no adopted chunk / no matching inference record"
            for side in self.arms:
                self._polyline(f"/root/{side}/raw_chunk", None, (0, 0, 0), 1.0)
                for k in range(8):
                    self._polyline(f"/root/{side}/variant{k}", None, (0, 0, 0), 1.0)
            return
        seq = int(rec["seq"])
        meta = chunk.get("chunk_metadata") or {}
        joined_by = "inference_seq" if meta.get("inference_seq") is not None else "time (last inference ready before the chunk)"
        lat_ms = (int(rec["ready_mono_ns"]) - int(rec["request_mono_ns"])) / 1e6
        pub = int(chunk["mono_ns"])
        age_ms = (m - int(rec["request_mono_ns"])) / 1e6
        st = rec.get("state")
        vp = rec.get("velproprio") or {}
        lines = [f"**inference {seq}** (joined by {joined_by}) · request→ready **{lat_ms:.0f} ms** · "
                 f"image age at cursor **{age_ms:.0f} ms** · chunk received {(pub - int(rec['ready_mono_ns']))/1e6:.0f} ms after ready",
                 f"proprio_mode `{rec.get('proprio_mode')}` state = `{np.round(np.array(st, dtype=float), 4).tolist() if st is not None else None}`"]
        arms_vp = vp.get("arms") if isinstance(vp, dict) else None
        if isinstance(arms_vp, dict):
            for side in ("left", "right"):
                d = (arms_vp.get(side) or {}).get("delta")
                if d is not None:
                    d = np.array(d, dtype=float)
                    lines.append(f"velproprio {side}: |v| {np.linalg.norm(d[:3])*1e3:.2f} mm/step, |w| {np.rad2deg(np.linalg.norm(d[3:6])):.2f} °/step "
                                 f"({'valid' if (arms_vp.get(side) or {}).get('valid') else 'INVALID: ' + str((arms_vp.get(side) or {}).get('zero_reason'))})")
        self.md_obs.content = "  \n".join(lines)
        if seq != self._last_img_seq:
            self._last_img_seq = seq
            for side, handle in (("left", self.img_left), ("right", self.img_right)):
                img = self._img_cache.get((seq, side))
                if img is None:
                    img = self.obs.image(rec, side)
                    if img is not None:
                        if len(self._img_cache) > 12:
                            self._img_cache.clear()
                        self._img_cache[(seq, side)] = img
                if img is not None:
                    try:
                        handle.image = img
                    except Exception:
                        pass
        # the 4-stage chain + variant overlays per arm
        chain = []
        raw = self.obs.actions(rec)
        for side in self.arms:
            a = self.arms[side]
            i = a.index_at(m)
            rows = np.array(chunk["rows"].get(side, []), dtype=float)
            off = 0 if side == "left" else 7
            parts = [f"**{side}**"]
            if raw is not None and len(raw):
                parts.append(f"1 model raw: |δ| mean {np.linalg.norm(raw[:, off:off+3], axis=1).mean()*1e3:.2f} mm/step, "
                             f"grip {raw[0, off+6]*100:.0f}→{raw[-1, off+6]*100:.0f} %")
            if len(rows) >= 2:
                dr = np.linalg.norm(np.diff(rows[:, :3], axis=0), axis=1) * 1e3
                parts.append(f"2 runner rows = 3 bridge deltas: {len(rows)} rows, |Δ| mean {dr.mean():.2f} mm/row "
                             f"(execute {chunk.get('execute_steps')} + runway {chunk.get('runway_steps')})")
            if a.fol_on[i]:
                v_cmd = float(a.cap.col("fol_v_cmd")[i])
                parts.append(f"4 controller: v_cmd {v_cmd:.1f} mm/s = {v_cmd*0.0334:.2f} mm/period · idx {int(a.chunk_idx[i])}/{int(a.chunk_n[i])} "
                             f"· gate {a.fol_gate_t[i]:.2f}")
            chain.append(" · ".join(parts))
            # overlays: the raw model chunk and reinfer variants, composed from the recorded row 0
            # with the runner's own local composition (pose_compose_local) - identical treatment
            # for recorded-raw and every variant, so their difference is the model's alone.
            anchor = rows[0, :7] if len(rows) else None
            if anchor is not None and raw is not None:
                self._polyline(f"/root/{side}/raw_chunk", self._compose(anchor, raw[:, off:off+6]), (255, 255, 255), 2.0)
            else:
                self._polyline(f"/root/{side}/raw_chunk", None, (0, 0, 0), 1.0)
            variants = self.obs.variants.get(seq, []) if self.cb_variants.value else []
            palette = [(255, 80, 80), (80, 200, 255), (120, 255, 120), (255, 160, 60), (200, 120, 255), (255, 220, 120), (100, 255, 220), (255, 120, 200)]
            for k in range(8):
                if k < len(variants) and anchor is not None:
                    va = np.array(variants[k].get("actions_mean") or variants[k].get("actions"), dtype=float)
                    self._polyline(f"/root/{side}/variant{k}", self._compose(anchor, va[:, off:off+6]), palette[k], 2.0)
                else:
                    self._polyline(f"/root/{side}/variant{k}", None, (0, 0, 0), 1.0)
            if variants:
                chain.append("variants: " + ", ".join(f"[{k}] {v['variant']}" for k, v in enumerate(variants[:8])))
        self.md_chain.content = "  \n".join(chain)

    @staticmethod
    def _compose(anchor7: np.ndarray, deltas6: np.ndarray) -> np.ndarray:
        try:
            sys.path.insert(0, os.path.join(ROOT, "policy_runner"))
            from policy_runner.flow_dataset import pose_compose_local
        except Exception:
            return np.zeros((0, 3))
        cur = np.asarray(anchor7, dtype=np.float64)
        pts = [cur[:3].copy()]
        for d in np.asarray(deltas6, dtype=np.float64):
            cur = np.asarray(pose_compose_local(cur, d), dtype=np.float64)
            pts.append(cur[:3].copy())
        return np.array(pts)

    @staticmethod
    def _sidecar_grip(records, side: str, mono_ns: int):
        best = None
        for r in records:
            if r["mono_ns"] > mono_ns:
                break
            v = (r.get("pct") or {}).get(side)
            if v is not None:
                best = float(v)
        return best

    # ---------------------------------------------------------------- text + plots
    def _info_line(self, side: str, a: ArmData, i: int, m: int) -> str:
        st = a.state[i].decode(errors="replace").strip("\x00") if a.state is not None else "?"
        tk = a.task[i].decode(errors="replace").strip("\x00") if a.task is not None else "?"
        parts = [f"**{side}** `{st}/{tk}` q_act={np.round(np.rad2deg(a.q_act[i]), 2).tolist()}"]
        parts.append(f"tracking |cmd−act| = **{a.track_err_mm[i]:.2f} mm** · |F| = {a.f_comp[i]:.1f} N")
        if a.schema4:
            if a.fol_on[i]:
                ch = f"stamp {int(a.chunk_stamp[i])} idx {int(a.chunk_idx[i])}/{int(a.chunk_n[i])} {'playing' if a.fol_playing[i] else 'idle'}"
                if self.sidecar:
                    c = self.sidecar.chunk_for_stamp(side, int(a.chunk_stamp[i]))
                    if c is not None:
                        pr = self.sidecar.pub_for_stamp(side, int(a.chunk_stamp[i]))
                        pub_ns = int(pr["pub_mono_ns"]) if pr else int((c.get("pub_mono_ns") or {}).get(side, m))
                        age = (m - pub_ns) / 1e6
                        sl = (f" slice orig_skip {pr.get('orig_skip', pr.get('skip'))} steps {pr.get('n_steps')} subs {pr.get('n_sub')} stretch {pr.get('max_stretch')}"
                              if pr else "")
                        ch = f"seq {c.get('seq')} (pub {age:.0f} ms ago{sl}) idx {int(a.chunk_idx[i])}/{int(a.chunk_n[i])} {'playing' if a.fol_playing[i] else 'idle'}"
                parts.append(f"follow: lag(cmd→ref) {a.fol_lag[i]:.2f} mm · gap(cmd→emitted) {a.fol_gap[i]:.2f} mm · "
                             f"gate t/r {a.fol_gate_t[i]:.2f}/{a.fol_gate_r[i]:.2f} · dev {a.dev_t[i]:.1f} mm · chunk {ch}")
            else:
                parts.append("follow: not engaged")
            gc, gf = a.grip_cmd_at(i), a.grip_fb_at(i)
            age_c = (m - a.ext[i, 2]) / 1e6 if a.ext[i, 2] > 0 else None
            age_f = (m - a.ext[i, 3]) / 1e6 if a.ext[i, 3] > 0 else None
            parts.append("gripper: cmd " + (f"**{gc:.1f} %** ({age_c:.0f} ms ago)" if gc is not None else "—")
                         + " · fb " + (f"**{gf:.1f} %** ({age_f:.0f} ms ago)" if gf is not None else "—")
                         + f" · ext_seq {int(a.ext_seq[i])}")
        return "  \n".join(parts)

    def _make_fig(self, side: str, t: float):
        import plotly.graph_objects as go
        a = self.arms[side]
        n = a.n
        step = max(1, n // 2500)
        ts = (a.mono[::step] - self.t0) / 1e9
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=ts, y=a.track_err_mm[::step], name="|cmd−act| mm", line=dict(color="rgb(60,120,255)", width=1)))
        if a.schema4:
            lag = np.where(a.fol_on, a.fol_lag, np.nan)[::step]
            fig.add_trace(go.Scatter(x=ts, y=lag, name="follow lag mm", line=dict(color="rgb(255,150,30)", width=1)))
            gc = np.where(a.ext_seq > 0, a.ext[:, 0], np.nan)[::step]
            gf = np.where((a.ext_seq > 0) & (a.ext[:, 3] > 0), a.ext[:, 1], np.nan)[::step]
            fig.add_trace(go.Scatter(x=ts, y=gc, name="grip cmd %", yaxis="y2", line=dict(color="rgb(230,60,230)", width=1, shape="hv")))
            fig.add_trace(go.Scatter(x=ts, y=gf, name="grip fb %", yaxis="y2", line=dict(color="rgb(40,200,90)", width=1)))
        fig.add_trace(go.Scatter(x=ts, y=a.f_comp[::step], name="|F| N", line=dict(color="rgb(200,60,60)", width=1, dash="dot")))
        fig.add_vline(x=t, line=dict(color="black", width=1))
        fig.update_layout(title=f"{side}", margin=dict(l=40, r=40, t=30, b=30), height=260,
                          legend=dict(orientation="h", y=-0.25, font=dict(size=9)),
                          xaxis=dict(title="t [s]"), yaxis=dict(title="mm / N"),
                          yaxis2=dict(title="%", overlaying="y", side="right", range=[0, 100]))
        return fig


# ------------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture-dir", help="directory holding data_*.bin (+ a sidecar); newest pair is used")
    ap.add_argument("--bin", nargs="*", help="explicit CHIMPBIN capture files (left/right)")
    ap.add_argument("--sidecar", help="cm_bridge sidecar JSONL")
    ap.add_argument("--device", default=os.path.join(ROOT, "cm_bridge/config/monkey/active.yaml"),
                    help="controller-manager device file (mount transforms + stand descriptor)")
    ap.add_argument("--urdf", default=None, help="arm URDF (default: rb_gui's articulated pika URDF)")
    ap.add_argument("--stand-mesh", default=os.path.join(
        ROOT, "submodules/controller-manager/cockpit/assets/meshes/stands/dual_rb3_730e_stand_ver3.stl"))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8082)
    ap.add_argument("--obs-dump", help="flow-infer observation dump dir (--observation-dump-dir): shows the wrist "
                                       "frames + state that produced the adopted chunk, and the raw model chunk")
    ap.add_argument("--variants", help="reinfer.py --json output: overlays the re-inferred chunks per inference")
    ap.add_argument("--start-t", type=float, default=0.0, help="initial cursor [s from capture start]")
    ap.add_argument("--headless-check", action="store_true",
                    help="load everything, render a few ticks, print a summary and exit (no server kept)")
    args = ap.parse_args()

    bins = list(args.bin or [])
    sidecar_path = args.sidecar
    if args.capture_dir:
        found, sc = find_capture_set(args.capture_dir)
        bins = bins or found
        sidecar_path = sidecar_path or sc
    if not bins:
        ap.error("no capture: give --capture-dir or --bin")
    arms: dict[str, ArmData] = {}
    for b in bins:
        cap = read_capture(b)
        if len(cap.rows) == 0:
            print(f"[cm_replay] {b}: empty capture, skipped"); continue
        arms[cap.side] = ArmData(cap)
        print(f"[cm_replay] {cap.side}: {b} v{cap.version} rows={len(cap.rows)} "
              f"{'SILS' if cap.sils else 'REAL'} schema4={arms[cap.side].schema4}")
    if not arms:
        ap.error("no usable capture rows")
    if not sidecar_path:
        # no sidecar named/found beside the capture: pick, among the bridge sidecars under
        # <repo>/logs, the one whose CLOCK_MONOTONIC span overlaps the capture (schema 4 only)
        import glob
        cands = sorted(set(glob.glob(os.path.join(ROOT, "logs", "cm_bridge_sidecar_*.jsonl")))
                       | set(glob.glob(os.path.join(ROOT, "logs", "**", "*sidecar*.jsonl"), recursive=True)))
        sidecar_path = select_sidecar([a.cap for a in arms.values()], cands)
        if sidecar_path:
            print(f"[cm_replay] sidecar auto-selected by time overlap: {sidecar_path}")
    sidecar = read_sidecar(sidecar_path)
    if sidecar:
        print(f"[cm_replay] sidecar: {sidecar_path} chunks={len(sidecar.chunks)} grip_cmd={len(sidecar.grip_cmd)} grip_fb={len(sidecar.grip_fb)}")
    else:
        print("[cm_replay] no sidecar: chunk previews off")
    if not os.path.exists(args.device):
        ap.error(f"device file not found: {args.device} (mounts are safety-relevant geometry; no default)")
    mounts, stand = _load_device(args.device)
    if len(mounts) < len(arms):
        ap.error(f"device file {args.device} lacks a mount transform for every captured arm")
    from rb_servo_gui import scene as gscene
    urdf_path = args.urdf or str(gscene._robot_urdf_path())
    if not os.path.exists(urdf_path):
        ap.error(f"URDF not found: {urdf_path}")

    obs = read_obs_dump(args.obs_dump, args.variants)
    if args.obs_dump and obs is None:
        print(f"[cm_replay] WARNING: no manifest.jsonl under {args.obs_dump} - policy-input panel off")
    elif obs is not None:
        print(f"[cm_replay] obs dump: {args.obs_dump} inferences={len(obs.by_seq)} variants={sum(len(v) for v in obs.variants.values())}")
    rep = Replay(arms, sidecar, mounts, stand, urdf_path, args.stand_mesh, args.host, args.port,
                 title="cm_replay", obs_dump=obs)
    if args.start_t > 0:
        rep._seek_seconds(args.start_t)
    if args.headless_check:
        n = len(rep.mono)
        for i in (0, n // 4, n // 2, (3 * n) // 4, n - 1):
            rep._set_cursor(i)
        # exercise every GUI path once (the callbacks are what a browser would drive)
        rep._set_cursor(n // 2)
        for which in ("chunk ▶", "◀ chunk", "grip ▶", "◀ grip"):
            rep._jump(which)
        for cb in (rep.cb_ref_ghost, rep.cb_frames, rep.cb_full, rep.cb_chunk_next):
            cb.value = not cb.value; rep._render(force=True)
        for which in ("+1", "-1", "+10", "-10", "|<", ">|"):
            rep._step_click(which)
        rep._set_cursor(0); rep.speed = 4.0; rep._toggle_play(); time.sleep(1.0); rep.playing = False
        print(f"[cm_replay] headless check OK: {n} ticks, {rep.dur:.2f} s, "
              f"arms={list(arms)}, sidecar={'yes' if sidecar else 'no'}, "
              f"played to tick {rep.cursor}", flush=True)
        # viser's server threads can abort at interpreter finalization ("terminate called without
        # an active exception") AFTER a clean check - not our code; skip the finalizer walk.
        os._exit(0)
    print(f"[cm_replay] http://{args.host}:{args.port}  ({len(rep.mono)} ticks, {rep.dur:.2f} s)")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
