#!/usr/bin/env python3
"""F/T tool {mass, CoM} identification for the RB5 cell, CM-procedure-equivalent.

Model (FtIdentify.h, sensor frame = flange-aligned axes, origin at the SRO):
    F = bias_F + m * u          u = R_world_flange^T * g_world
    M = bias_M + (m*com) x u
Linear in x = [m, m*com(3), bias_F(3), bias_M(3)]; bias is CO-estimated, so no
tare is needed and the estimate also REVEALS whether the box was gravity-
compensating the eft stream (a compensated stream fits m ~ 0).

Three phases:
  plan   offline: generate wrist-only pose candidates around the init pose,
         validate them against the collision model (arms, stand, riser box,
         work-table boxes - the tables are NOT in the server's model, so this
         is the only check they get), pick a spread-maximising set.
  run    on the live stack: move ONE arm at a time through its poses with the
         server's own collision-free InitMotion planner (single-arm profile,
         other arm holds), dwell still, average raw F/T from the state stream.
  solve  least squares per arm; compare against the config values; refuse the
         degenerate cases CM refuses (tiny mass, no orientation spread).
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "rb_servo_server/descriptions/urdf/dual_rb5_850e_ver3.urdf"
MESHES = ROOT / "rb_servo_server/descriptions/meshes/robots/rb5_850e/visual/tool"
INIT_JSON = Path.home() / ".rb_servo_gui/init_motion.json"
PLAN_JSON = ROOT / "logs/ft_cog_plan.json"
SAMPLES_JSON = ROOT / "logs/ft_cog_samples.json"

ARMS = ("left", "right")
PREFIX = {"left": "dual_rb5_850e_left_", "right": "dual_rb5_850e_right_"}
JOINTS = ("base_joint", "shoulder_joint", "elbow_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint")
G = 9.80665
# Riser + tables in the STAND frame (make_rb5_850e_urdfs.py ENVIRONMENT).
BOXES_STAND = {
    "riser":  (np.array([0.0, 0.0049, -0.1625]), np.array([0.307, 0.324, 0.295]) / 2),
    "table1": (np.array([0.25, 0.0, -0.710]),    np.array([0.80, 0.80, 0.80]) / 2),
    "table2": (np.array([1.05, 0.0, -0.710]),    np.array([0.80, 0.80, 0.80]) / 2),
}
# Offline gates (mm). Deliberately wider than the server's floors: the planner
# and the runtime monitor re-verify with the real geometry; these only keep us
# from ASKING for a tight pose. Table gate covers geometry the server does NOT.
GATE_MM = {"other_arm": 80.0, "stand": 70.0, "riser": 60.0, "table": 60.0, "intra_tool": 30.0}
WRIST_ABS_LIMIT_DEG = 170.0
N_POSES = 8


def load_init() -> dict[str, list[float]]:
    d = json.loads(INIT_JSON.read_text())
    return {"left": [float(v) for v in d["left"]], "right": [float(v) for v in d["right"]]}


# ---------------------------------------------------------------- model/geometry
def load_model():
    import yourdfpy
    from functools import partial
    u = yourdfpy.URDF.load(
        URDF, build_scene_graph=False, load_meshes=False,
        build_collision_scene_graph=True, load_collision_meshes=True,
        filename_handler=partial(yourdfpy.filename_handler_magic, dir=URDF.parent))
    import trimesh
    grip = {}
    for name in ("pika_gripper_base_hull.STL", "pika_finger_left_hull.STL", "pika_finger_right_hull.STL"):
        m = trimesh.load_mesh(str(MESHES / name))
        m.apply_scale(0.001)  # STL in mm, attached at attachment_site identity (open)
        grip[name] = np.asarray(m.vertices)
    return u, grip


def set_cfg(u, q_left_deg, q_right_deg):
    cfg = {}
    for side, q in (("left", q_left_deg), ("right", q_right_deg)):
        for i, j in enumerate(JOINTS):
            cfg[PREFIX[side] + j] = math.radians(float(q[i]))
    u.update_cfg(cfg)


def flange_R_world(u, side):
    T = u.get_transform(PREFIX[side] + "attachment_site", "world", collision_geometry=True)
    return np.asarray(T)[:3, :3]


def gravity_dir_sensor(u, side):
    return flange_R_world(u, side).T @ np.array([0.0, 0.0, -1.0])


def arm_point_sets(u, grip, side, sub=220):
    """{label: Nx3 world points} for one arm at the CURRENT cfg."""
    out = {}
    par = u.collision_scene.graph.transforms.parents
    for g, mesh in u.collision_scene.geometry.items():
        link = par[g]
        if not link.startswith(PREFIX[side]):
            continue
        T = np.asarray(u.get_transform(link, "world", collision_geometry=True))
        v = np.asarray(mesh.vertices)
        v = v[:: max(1, len(v) // sub)]
        out[link.rsplit("_", 1)[-1]] = (v @ T[:3, :3].T) + T[:3, 3]
    Ta = np.asarray(u.get_transform(PREFIX[side] + "attachment_site", "world", collision_geometry=True))
    gv = np.vstack(list(grip.values()))
    gv = gv[:: max(1, len(gv) // (3 * sub))]
    out["tool"] = (gv @ Ta[:3, :3].T) + Ta[:3, 3]
    return out


def stand_point_sets(u, sub=250):
    out = {}
    par = u.collision_scene.graph.transforms.parents
    for g, mesh in u.collision_scene.geometry.items():
        link = par[g]
        if link.startswith("dual_rb5_850e_"):
            continue
        T = np.asarray(u.get_transform(link, "world", collision_geometry=True))
        v = np.asarray(mesh.vertices)
        v = v[:: max(1, len(v) // sub)]
        out[link] = (v @ T[:3, :3].T) + T[:3, 3]
    return out


def min_dist(a: np.ndarray, b: np.ndarray) -> float:
    d2 = ((a[:, None, :] - b[None, :, :]) ** 2).sum(axis=2)
    return float(np.sqrt(d2.min()))


def pose_clearances_mm(u, grip, side, other_pts, stand_pts, T_world_stand_inv):
    """Worst clearance of the MOVING arm per gate category, mm."""
    mine = arm_point_sets(u, grip, side)
    moving = np.vstack([mine[k] for k in mine if k in ("link4", "link5", "link6", "tool")])
    all_mine = np.vstack(list(mine.values()))
    other = np.vstack(list(other_pts.values()))
    res = {"other_arm": min_dist(all_mine, other) * 1000}
    res["stand"] = min(min_dist(moving, pts) for pts in stand_pts.values()) * 1000
    ps = (moving @ T_world_stand_inv[:3, :3].T) + T_world_stand_inv[:3, 3]
    for name, (c, h) in BOXES_STAND.items():
        d = np.linalg.norm(np.maximum(np.abs(ps - c) - h, 0.0), axis=1).min() * 1000
        key = "riser" if name == "riser" else "table"
        res[key] = min(res.get(key, 1e9), d)
    own_upper = np.vstack([mine[k] for k in ("link1", "link2", "link3") if k in mine])
    res["intra_tool"] = min_dist(mine["tool"], own_upper) * 1000
    return res


# ROI box (stand frame) the server ENFORCES on the TCP + 4 fingertip points
# (safety_verdict RoiViolation froze the first capture attempt at wp 22/48).
# We do not touch the envelope; we only ask for poses whose whole PATH stays in.
ROI_MIN = np.array([0.3, -0.4, -0.4])
ROI_MAX = np.array([1.1, 0.4, 0.59])
ROI_MARGIN_M = 0.035
TCP_FROM_ATT = np.array([0.0, 0.0, 0.247642])
TIP_OFFSETS_TCP = np.array([  # OPEN fingertips in the TCP frame (scene.py)
    [0.057, 0.012, 0.0], [0.057, -0.012, 0.0], [-0.057, 0.012, 0.0], [-0.057, -0.012, 0.0]])


def roi_path_ok(u, side, q_from, q_to, other_q, T_world_stand_inv, steps=12):
    """TCP + fingertips stay inside the ROI (with margin) along a joint-space
    interpolation of the move. The planner's path may differ, but wrist-dominant
    free-space moves come back near-direct; the runtime clamp is still the guard."""
    for t in np.linspace(0.0, 1.0, steps):
        q = [a + t * (b - a) for a, b in zip(q_from, q_to)]
        ql = q if side == "left" else other_q
        qr = q if side == "right" else other_q
        set_cfg(u, ql, qr)
        T = np.asarray(u.get_transform(PREFIX[side] + "attachment_site", "world", collision_geometry=True))
        pts_att = np.vstack([TCP_FROM_ATT, TCP_FROM_ATT + TIP_OFFSETS_TCP])
        pw = (pts_att @ T[:3, :3].T) + T[:3, 3]
        ps = (pw @ T_world_stand_inv[:3, :3].T) + T_world_stand_inv[:3, 3]
        if (ps < ROI_MIN + ROI_MARGIN_M).any() or (ps > ROI_MAX - ROI_MARGIN_M).any():
            return False
    return True


# ------------------------------------------------------------------------- plan
def phase_plan() -> dict:
    init = load_init()
    u, grip = load_model()
    T_ws = np.asarray(u.get_transform("stand", "world", collision_geometry=True))
    T_inv = np.linalg.inv(T_ws)
    stand_pts = stand_point_sets(u)
    plan = {"init": init, "poses": {}}
    for side in ARMS:
        other = "right" if side == "left" else "left"
        set_cfg(u, init["left"], init["right"])
        other_pts = arm_point_sets(u, grip, other)
        base = init[side]
        cands = []
        for d2 in (-20, 0, 20):
            for d3 in (-20, 0, 20):
                for d4 in (-90, -45, 0, 45, 90):
                    for d5 in (-140, -105, -70, -35, 0, 35, 70, 105, 140):
                        for d6 in (-90, 0, 90):
                            q = list(base)
                            q[1] += d2
                            q[2] += d3
                            q[3] += d4
                            q[4] += d5
                            q[5] += d6
                            if any(abs(q[i]) > WRIST_ABS_LIMIT_DEG for i in (3, 4, 5)):
                                continue
                            if abs(q[2]) > 160.0:
                                continue
                            cands.append(((d2, d3, d4, d5, d6), q))
        # Cheap ROI-path prefilter first (the expensive collision check runs on
        # the survivors only). Path = init -> pose, which is also how the driver
        # routes every move (always via init).
        roi_ok = [(d, q) for d, q in cands
                  if roi_path_ok(u, side, base, q, init[other], T_inv)]
        print(f"[{side}] {len(cands)} candidates, {len(roi_ok)} pass the ROI path check")
        if len(roi_ok) > 140:
            rng = np.random.default_rng(7)
            keep = {0}  # always keep the first (smallest delta ordering not guaranteed; init added below)
            idx = rng.choice(len(roi_ok), size=140, replace=False)
            roi_ok = [roi_ok[i] for i in sorted(set(idx) | keep)]
        # Make sure the init pose itself is in the pool.
        if not any(d == (0, 0, 0, 0, 0) for d, _ in roi_ok):
            roi_ok.append(((0, 0, 0, 0, 0), list(base)))
        rows = []
        for delta, q in roi_ok:
            ql = q if side == "left" else init["left"]
            qr = q if side == "right" else init["right"]
            set_cfg(u, ql, qr)
            uvec = gravity_dir_sensor(u, side)
            cl = pose_clearances_mm(u, grip, side, other_pts, stand_pts, T_inv)
            ok = all(cl[k] >= GATE_MM[k] for k in GATE_MM)
            rows.append({"delta": list(delta), "q": q, "u": uvec.tolist(),
                         "clear_mm": {k: round(v, 1) for k, v in cl.items()}, "ok": ok})
        good = [r for r in rows if r["ok"]]
        # Greedy max-min angular spread, seeded with the init pose.
        seed = next((r for r in good if r["delta"] == [0, 0, 0, 0, 0]), good[0])
        sel = [seed]
        while len(sel) < N_POSES and len(sel) < len(good):
            best, best_score = None, -1.0
            for r in good:
                if r in sel:
                    continue
                score = min(math.degrees(math.acos(np.clip(np.dot(r["u"], s["u"]), -1, 1))) for s in sel)
                if score > best_score:
                    best, best_score = r, score
            if best is None or best_score < 5.0:
                break
            sel.append(best)
        spread = max(
            math.degrees(math.acos(np.clip(np.dot(a["u"], b["u"]), -1, 1)))
            for i, a in enumerate(sel) for b in sel[i + 1:])
        plan["poses"][side] = sel
        print(f"[{side}] candidates {len(cands)}, collision-clear {len(good)}, "
              f"selected {len(sel)}, gravity-direction spread {spread:.1f} deg")
        for r in sel:
            print(f"   d(J4,J5,J6)={r['delta']}  clear={r['clear_mm']}")
        if spread < 45.0:
            raise SystemExit(f"{side}: spread {spread:.1f} deg is too small for a trustworthy fit")
    PLAN_JSON.write_text(json.dumps(plan, indent=1))
    print(f"plan -> {PLAN_JSON}")
    return plan


# -------------------------------------------------------------------------- run
class StateListener(threading.Thread):
    def __init__(self, port=50378):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.latest = None
        self.stamp = 0.0

    def run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(70000)
            except socket.timeout:
                continue
            try:
                self.latest = json.loads(data.decode())
                self.stamp = time.monotonic()
            except ValueError:
                pass

    def fresh(self, max_age=1.0):
        return self.latest is not None and (time.monotonic() - self.stamp) < max_age


def q_actual(st, side):
    try:
        return [float(v) for v in st[side]["q_actual_deg"]]
    except (KeyError, TypeError, ValueError):
        return None


def raw_wrench(st, side):
    try:
        w = st[side]["force_torque"]["raw_sensor_axes_at_sro"]
        return [float(v) for v in w] if len(w) == 6 else None
    except (KeyError, TypeError):
        return None


def phase_run() -> None:
    plan = json.loads(PLAN_JSON.read_text())
    init = plan["init"]
    sys.path.insert(0, str(ROOT / "rb_gui"))
    from rb_servo_gui.command_client import CommandClient
    client = CommandClient("127.0.0.1", 50256, source_id="ft_cog_capture")
    listener = StateListener()
    listener.start()
    t0 = time.monotonic()
    while not listener.fresh():
        if time.monotonic() - t0 > 5.0:
            raise SystemExit("no state on UDP 50378 - is the stack (make run) up?")
        time.sleep(0.1)

    def guard(st):
        if st.get("fault_latched"):
            raise SystemExit(f"ABORT: fault latched: {st.get('fault_reason')}")

    def wait_arrival(side, target, timeout=90.0):
        """InitMotion done AND q_actual at the target. Returns False on plan failure.

        The sequencer's status is LATCHED from the previous request until the new
        one takes over, so a terminal value only counts after we have seen the new
        request start (status leaves the terminal set, or the arm starts moving),
        or after a grace period long enough for a no-op accept."""
        t_start = time.monotonic()
        started = False
        while time.monotonic() - t_start < timeout:
            st = listener.latest
            if not listener.fresh():
                raise SystemExit("ABORT: state stream went stale")
            guard(st)
            im = st.get("init_motion") or {}
            status = im.get("status")
            if not started:
                if status in ("planning", "executing"):
                    started = True
                elif time.monotonic() - t_start > 3.0:
                    started = True  # no-op accept (already at target) never shows executing
                else:
                    time.sleep(0.05)
                    continue
            if status == "failed":
                print(f"   plan REFUSED: {im.get('message')}")
                return False
            q = q_actual(st, side)
            if q is not None and all(abs(q[i] - target[i]) < 1.5 for i in range(6)):
                if status in ("done", "idle", "", None):
                    return True
            time.sleep(0.1)
        print("   TIMEOUT waiting for arrival")
        return False

    def move_arm(side, q_target):
        pkt = client.build_init_motion_arm(
            side, q_target if side == "left" else init["left"],
            q_target if side == "right" else init["right"], timeout_sec=60.0)
        client.send(pkt)

    def move_both_init():
        client.send(client.build_init_motion(init["left"], init["right"], timeout_sec=60.0))
        okl = wait_arrival("left", init["left"])
        okr = wait_arrival("right", init["right"])
        if not (okl and okr):
            raise SystemExit("ABORT: could not reach the init pose")

    def dwell(side, seconds=6.0, skip=1.5):
        fs, qs = [], []
        t_start = time.monotonic()
        while time.monotonic() - t_start < seconds:
            st = listener.latest
            guard(st)
            if time.monotonic() - t_start >= skip:
                w = raw_wrench(st, side)
                q = q_actual(st, side)
                if w is not None and q is not None:
                    fs.append(w)
                    qs.append(q)
            time.sleep(0.02)
        if len(fs) < 100:
            return None
        F = np.array(fs)
        Q = np.array(qs)
        if Q.std(axis=0).max() > 0.05:
            print(f"   arm not still (q std {Q.std(axis=0).max():.3f} deg) - dropping this dwell")
            return None
        return {"q_mean": Q.mean(axis=0).tolist(),
                "wrench_mean": F.mean(axis=0).tolist(),
                "wrench_std": F.std(axis=0).tolist(),
                "n": len(fs)}

    print("== homing both arms (collision-free InitMotion) ==")
    move_both_init()
    samples = {"left": [], "right": []}
    for side in ARMS:
        print(f"== {side} arm: {len(plan['poses'][side])} poses ==")
        for k, pose in enumerate(plan["poses"][side]):
            print(f"[{side} {k+1}/{len(plan['poses'][side])}] d={pose['delta']} -> move")
            move_arm(side, pose["q"])
            if not wait_arrival(side, pose["q"]):
                print("   skipped")
                move_arm(side, init[side])
                if not wait_arrival(side, init[side]):
                    raise SystemExit("ABORT: could not return to init after a skip")
                continue
            s = dwell(side)
            if s is not None:
                s["delta"] = pose["delta"]
                samples[side].append(s)
                w = s["wrench_mean"]
                print(f"   captured n={s['n']}  F=[{w[0]:7.2f},{w[1]:7.2f},{w[2]:7.2f}] N  "
                      f"std_max={max(s['wrench_std'][:3]):.2f}")
            # Route the next move via init: that is the leg the offline ROI/collision
            # validation actually checked (init <-> pose), so pose-to-pose legs never
            # run unvalidated.
            move_arm(side, init[side])
            if not wait_arrival(side, init[side]):
                raise SystemExit("ABORT: could not return to init between poses")
        print(f"== {side} arm back to init ==")
        move_arm(side, init[side])
        if not wait_arrival(side, init[side]):
            raise SystemExit("ABORT: could not return to init")
    SAMPLES_JSON.write_text(json.dumps(samples, indent=1))
    print(f"samples -> {SAMPLES_JSON}  (left {len(samples['left'])}, right {len(samples['right'])})")


# ------------------------------------------------------------------------ solve
def phase_solve() -> None:
    samples = json.loads(SAMPLES_JSON.read_text())
    init = load_init()
    u, _grip = load_model()
    for side in ARMS:
        rows = samples.get(side, [])
        if len(rows) < 6:
            print(f"[{side}] only {len(rows)} poses - CM refuses < 6; not solving")
            continue
        A, b, us = [], [], []
        for s in rows:
            ql = s["q_mean"] if side == "left" else init["left"]
            qr = s["q_mean"] if side == "right" else init["right"]
            set_cfg(u, ql, qr)
            uv = gravity_dir_sensor(u, side) * G      # m/s^2, sensor frame
            us.append(uv / G)
            F = np.array(s["wrench_mean"][:3])
            M = np.array(s["wrench_mean"][3:])
            skew = np.array([[0, -uv[2], uv[1]], [uv[2], 0, -uv[0]], [-uv[1], uv[0], 0]])
            rF = np.zeros((3, 10)); rF[:, 0] = uv; rF[:, 4:7] = np.eye(3)
            rM = np.zeros((3, 10)); rM[:, 1:4] = -skew; rM[:, 7:10] = np.eye(3)
            A.extend([rF, rM]); b.extend([F, M])
        A = np.vstack(A); b = np.hstack(b)
        x, *_ = np.linalg.lstsq(A, b, rcond=None)
        resid = A @ x - b
        rms = float(np.sqrt(np.mean(resid[: len(rows) * 3] ** 2)))
        cond = float(np.linalg.cond(A))
        spread = max(math.degrees(math.acos(np.clip(np.dot(a, c), -1, 1)))
                     for i, a in enumerate(us) for c in us[i + 1:])
        m = float(x[0]); com_mm = (x[1:4] / m * 1000.0) if abs(m) > 1e-6 else np.zeros(3)
        print(f"\n[{side}] poses={len(rows)} spread={spread:.1f} deg cond={cond:.1f} force-rms={rms:.3f} N")
        print(f"  mass      = {m:.4f} kg")
        print(f"  com (SRO) = [{com_mm[0]:.2f}, {com_mm[1]:.2f}, {com_mm[2]:.2f}] mm")
        print(f"  bias F    = [{x[4]:.2f}, {x[5]:.2f}, {x[6]:.2f}] N   bias M = [{x[7]:.3f}, {x[8]:.3f}, {x[9]:.3f}] Nm")
        cfg_mass = {"left": 0.7912, "right": 0.7822}[side]
        cfg_com = {"left": [-0.02, 3.24, 25.34], "right": [-1.24, 4.03, 26.56]}[side]
        print(f"  config    = {cfg_mass} kg @ {cfg_com} mm  ->  d_mass={1000*(m-cfg_mass):+.0f} g")
        if abs(m) < 0.2:
            print("  !! mass ~ 0: the BOX is gravity-compensating the eft stream (payload nonzero)"
                  " - estimate rejected, zero the box payload and re-capture")
        if spread < 30.0:
            print("  !! spread < 30 deg - the CM refusal threshold; do not trust this fit")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("phase", choices=["plan", "run", "solve"])
    args = ap.parse_args()
    {"plan": phase_plan, "run": phase_run, "solve": phase_solve}[args.phase]()
