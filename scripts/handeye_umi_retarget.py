#!/usr/bin/env python3
"""Measure the UMI->stand retarget (T_stand_source) for robotics_lab.

Solves the robot-world / hand-eye problem

    A_i . Y = X . B_i

with, for each captured sample i:
    A_i = robot TCP pose in the STAND frame
          (rb_servo_server servo_state.v1, payload[side]["tcp_stand"])
    B_i = Vive tracker pose in the STEAMVR_WORLD frame
          (OpenVR, TrackingUniverseStanding -- same frame the PIKA episodes used)
    X   = T_stand_steamvr  (== umi_retarget `T_stand_source`)   <- the unknown we want
    Y   = T_tcp_tracker    (the calibration-rig mount offset; a nuisance term, not exported)

Stages
------
capture : with a Vive tracker rigidly bolted to the robot TCP, jog the arm to many
          diverse poses and record synchronized (A_i, B_i) pairs.
solve   : solve X per arm, report residuals, write calibration/umi_retarget.yaml
          (schema robotics_lab.umi_retarget.v1).
selftest: numerically verify the solver with synthetic data (no hardware needed).

T_tcp_umi_gripper note
----------------------
The retarget's `T_tcp_umi_gripper` is the UMI *device's* tracker->gripper offset during
data collection -- NOT the mount (Y) used here. It is known from the PIKA SDK:
GRIPPER_OFFSET = (0.172, 0.0, -0.076) m. CORRECTION (2026-06-11): this translation is
defined in the rotation-corrected gripper frame, not tracker-local — the full official
transform is T_tip = T_raw . R_corr . Trans(0.172,0,-0.076) with R_corr =
Rx(-20deg).[Ry(-90deg).Rx(-90deg)] (raw-frame lever-arm (0,-0.0126,+0.1876) m).
Downstream applies (umi_pipeline._retarget_poses):

    converted = T_stand_source . umi_pose . inverse(T_tcp_umi_gripper)

For inverse(T_tcp_umi_gripper) to map tracker->gripper we need
    inverse(T_tcp_umi_gripper) = ^tracker T_gripper = (translation = gripper_in_tracker, rot = I)
hence T_tcp_umi_gripper = inverse(that). Override with --gripper-in-tracker / --gripper-rot-*.

Only the ROTATION of X strictly matters for the delta/twist flow policy (translation cancels
in pose deltas), but the full SE(3) X is solved and exported for completeness/visualization.

Pose I/O format everywhere is [x, y, z, qx, qy, qz, qw] (meters, xyzw quaternion).
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# SE(3) / quaternion helpers.  External pose format: [x,y,z, qx,qy,qz,qw] (xyzw).
# Internal quaternion math uses wxyz for the multiply matrices.
# ---------------------------------------------------------------------------


def quat_xyzw_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("zero-norm quaternion")
    return q / n


def quat_xyzw_to_rot(q):
    x, y, z, w = quat_xyzw_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat_xyzw(R):
    """Shepperd's method -> [qx,qy,qz,qw]."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return quat_xyzw_normalize([x, y, z, w])


def pose7_to_T(p):
    p = np.asarray(p, dtype=float)
    T = np.eye(4)
    T[:3, :3] = quat_xyzw_to_rot(p[3:7])
    T[:3, 3] = p[:3]
    return T


def T_to_pose7(T):
    q = rot_to_quat_xyzw(T[:3, :3])
    return np.concatenate([T[:3, 3], q])


def T_inv(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rot_angle(R):
    """Geodesic rotation angle (radians)."""
    c = (np.trace(R) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


# ---------------------------------------------------------------------------
# Hand-eye solver:  A_i . Y = X . B_i   ->   X (and Y), with residuals.
# ---------------------------------------------------------------------------


def solve_handeye(A_list, B_list, min_rel_angle_deg=2.0):
    """Solve A_i Y = X B_i for X (and Y) from full-pose pairs.

    Reduction: for sample pairs (i,j),
        A_j A_i^-1 . X = X . B_j B_i^-1     (classic AX=XB)
    rotation via quaternion null-space SVD; translation via linear least squares.
    """
    n = len(A_list)
    if n < 3:
        raise ValueError("need >= 3 pose pairs (>= 8 recommended)")
    A = [np.asarray(T, float) for T in A_list]
    B = [np.asarray(T, float) for T in B_list]
    thr = math.radians(min_rel_angle_deg)

    # --- rotation of X ---
    # Relative motions satisfy  R_Aij R_X = R_X R_Bij.  Solve sign-robustly via the
    # Kronecker (vec) form:  (I (x) R_Aij - R_Bij^T (x) I) vec(R_X) = 0, then project
    # the null vector to SO(3).  (Quaternion linear forms suffer +-sign ambiguity.)
    I3 = np.eye(3)
    rows = []
    for i, j in itertools.combinations(range(n), 2):
        Aij = A[j] @ T_inv(A[i])
        Bij = B[j] @ T_inv(B[i])
        if rot_angle(Aij[:3, :3]) < thr or rot_angle(Bij[:3, :3]) < thr:
            continue  # near-pure-translation pair carries no rotation info
        rows.append(np.kron(I3, Aij[:3, :3]) - np.kron(Bij[:3, :3].T, I3))
    if not rows:
        raise ValueError("no pose pair has enough relative rotation; vary orientation more")
    M = np.vstack(rows)
    _, _, Vt = np.linalg.svd(M)
    Rraw = Vt[-1].reshape(3, 3, order="F")          # vec is column-major
    if np.linalg.det(Rraw) < 0:                      # null vector sign is arbitrary;
        Rraw = -Rraw                                 # fix BEFORE projection (det~s^3)
    U, _, Vt2 = np.linalg.svd(Rraw)                 # nearest orthonormal
    RX = U @ Vt2
    if np.linalg.det(RX) < 0:                        # numerical guard
        U[:, -1] *= -1
        RX = U @ Vt2

    # --- translations t_Y, t_X via least squares ---
    #   R_Ai t_Y - t_X = R_X t_Bi - t_Ai
    G = np.zeros((3 * n, 6))
    h = np.zeros(3 * n)
    for i in range(n):
        RAi = A[i][:3, :3]
        tAi = A[i][:3, 3]
        tBi = B[i][:3, 3]
        G[3 * i:3 * i + 3, 0:3] = RAi
        G[3 * i:3 * i + 3, 3:6] = -np.eye(3)
        h[3 * i:3 * i + 3] = RX @ tBi - tAi
    sol, *_ = np.linalg.lstsq(G, h, rcond=None)
    tY = sol[0:3]
    tX = sol[3:6]

    # --- rotation of Y (for residuals only): R_Y = R_Ai^T R_X R_Bi, averaged ---
    quats = np.array([rot_to_quat_xyzw(A[i][:3, :3].T @ RX @ B[i][:3, :3])
                      for i in range(n)])  # xyzw
    quats[quats[:, 3] < 0] *= -1.0  # hemisphere align by w
    _, V = np.linalg.eigh(quats.T @ quats)
    RY = quat_xyzw_to_rot(V[:, -1])  # eigenvector for largest eigenvalue (quaternion mean)

    X = np.eye(4); X[:3, :3] = RX; X[:3, 3] = tX
    Y = np.eye(4); Y[:3, :3] = RY; Y[:3, 3] = tY

    # --- residuals: compare A_i Y vs X B_i ---
    t_errs, r_errs = [], []
    for i in range(n):
        E = T_inv(X @ B[i]) @ (A[i] @ Y)
        t_errs.append(np.linalg.norm(E[:3, 3]))
        r_errs.append(rot_angle(E[:3, :3]))
    res = {
        "n_pairs": n,
        "trans_rms_m": float(np.sqrt(np.mean(np.square(t_errs)))),
        "trans_max_m": float(np.max(t_errs)),
        "rot_rms_deg": float(math.degrees(np.sqrt(np.mean(np.square(r_errs))))),
        "rot_max_deg": float(math.degrees(np.max(r_errs))),
    }
    return X, Y, res


# ---------------------------------------------------------------------------
# Live capture: robot state (UDP) + Vive tracker (OpenVR).
# ---------------------------------------------------------------------------


def _parse_udp(endpoint):
    e = endpoint.replace("udp://", "")
    host, port = e.rsplit(":", 1)
    return host, int(port)


def _tcp_stand_from_payload(payload, side):
    """Mirror flow_dataset._pose_from_state_arm: payload[side]['tcp_stand'] -> pose7."""
    arm = payload.get(side)
    if not isinstance(arm, dict):
        return None
    if arm.get("tcp_actual_valid") is False:
        return None  # don't capture a stale/invalid pose
    pose = arm.get("tcp_stand") or arm.get("tcp_actual_stand")
    if isinstance(pose, (list, tuple)) and len(pose) == 7:
        return [float(v) for v in pose]  # flat [x,y,z,qx,qy,qz,qw]
    if not isinstance(pose, dict):
        return None
    pos = pose.get("position", pose)
    x = float(pos.get("x")); y = float(pos.get("y")); z = float(pos.get("z"))
    q = pose.get("quaternion_xyzw")
    if q is not None and len(q) == 4:
        qx, qy, qz, qw = (float(v) for v in q)
    else:
        qx = float(pose.get("qx")); qy = float(pose.get("qy"))
        qz = float(pose.get("qz")); qw = float(pose.get("qw"))
    return [x, y, z, qx, qy, qz, qw]


class RobotStateUDP:
    def __init__(self, bind):
        import socket
        import threading
        host, port = _parse_udp(bind)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.settimeout(0.2)
        self._latest = None
        self._lock = threading.Lock()
        self._running = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(65536)
            except OSError:
                continue
            try:
                payload = json.loads(data.decode("utf-8"))
            except Exception:
                continue
            with self._lock:
                self._latest = payload

    def tcp_stand(self, side):
        with self._lock:
            p = self._latest
        return _tcp_stand_from_payload(p, side) if p else None

    def close(self):
        self._running = False


class TrackerOpenVR:
    """Minimal OpenVR generic-tracker reader (TrackingUniverseStanding, xyzw)."""

    def __init__(self):
        import openvr  # local import: only needed for capture
        self.openvr = openvr
        self.vr = openvr.init(openvr.VRApplication_Background)
        self.origin = openvr.TrackingUniverseStanding
        self.klass = openvr.TrackedDeviceClass_GenericTracker

    def poses(self):
        openvr = self.openvr
        n = openvr.k_unMaxTrackedDeviceCount
        raw = self.vr.getDeviceToAbsoluteTrackingPose(self.origin, 0, n)
        out = {}
        for i in range(n):
            if self.vr.getTrackedDeviceClass(i) != self.klass:
                continue
            p = raw[i]
            if not (p.bDeviceIsConnected and p.bPoseIsValid):
                continue
            pos, quat = _mat34_to_pos_quat(p.mDeviceToAbsoluteTracking)
            try:
                sn = self.vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
            except Exception:
                sn = "dev%d" % i
            out[sn] = [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]
        return out

    def close(self):
        self.openvr.shutdown()


def _mat34_to_pos_quat(m):
    x, y, z = m[0][3], m[1][3], m[2][3]
    R = np.array([[m[0][0], m[0][1], m[0][2]],
                  [m[1][0], m[1][1], m[1][2]],
                  [m[2][0], m[2][1], m[2][2]]])
    return (x, y, z), rot_to_quat_xyzw(R)


def cmd_capture(args):
    robot = RobotStateUDP(args.robot_state)
    tracker = TrackerOpenVR()
    print(f"[capture] side={args.side}  robot-state={args.robot_state}")
    print("Waiting for first robot-state + tracker frames ...")
    pairs = []
    try:
        while True:
            line = input(
                f"\n[{len(pairs)} captured] Move robot to a new diverse pose, then "
                "ENTER to capture  (q ENTER to finish): "
            ).strip().lower()
            if line == "q":
                break
            A = robot.tcp_stand(args.side)
            tp = tracker.poses()
            if A is None:
                print("  !! no robot tcp_stand yet (is rb_servo_server publishing?) -- skipped")
                continue
            if not tp:
                print("  !! no valid tracker pose (occluded / SteamVR down?) -- skipped")
                continue
            if args.tracker_serial:
                if args.tracker_serial not in tp:
                    print(f"  !! tracker {args.tracker_serial} not seen (seen: {list(tp)}) -- skipped")
                    continue
                B = tp[args.tracker_serial]
            elif len(tp) == 1:
                B = next(iter(tp.values()))
            else:
                print(f"  !! multiple trackers {list(tp)}; pass --tracker-serial -- skipped")
                continue
            pairs.append({"A_tcp_stand": A, "B_tracker_steamvr": B, "t": time.time()})
            print(f"  ok  A={np.round(A,4).tolist()}\n      B={np.round(B,4).tolist()}")
    finally:
        robot.close()
        tracker.close()
    out = {
        "schema": "robotics_lab.handeye_pairs.v1",
        "side": args.side,
        "source_pose_frame": "steamvr_world",
        "target_pose_frame": "stand",
        "pairs": pairs,
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n[capture] wrote {len(pairs)} pairs -> {args.out}")
    if len(pairs) < 8:
        print("  NOTE: >= 8 diverse poses (varied orientation) strongly recommended.")


# ---------------------------------------------------------------------------
# Dual-arm teleop-friendly capture (one process, both arms, live feedback).
# ---------------------------------------------------------------------------


def _pose_delta(p, q):
    """(trans m, rot deg) between two pose7."""
    D = T_inv(pose7_to_T(p)) @ pose7_to_T(q)
    return float(np.linalg.norm(D[:3, 3])), float(math.degrees(rot_angle(D[:3, :3])))


def _max_rot_spread(poses):
    """Max pairwise relative rotation (deg) among pose7 list -> orientation diversity."""
    if len(poses) < 2:
        return 0.0
    Ts = [pose7_to_T(p) for p in poses]
    best = 0.0
    for i, j in itertools.combinations(range(len(Ts)), 2):
        a = math.degrees(rot_angle((T_inv(Ts[i]) @ Ts[j])[:3, :3]))
        best = max(best, a)
    return best


def _min_delta_to(poses, p):
    """Smallest (trans, rot) distance from p to any pose in the list."""
    if not poses:
        return (1e9, 1e9)
    ds = [_pose_delta(q, p) for q in poses]
    return (min(d[0] for d in ds), min(d[1] for d in ds))


def _running_solve(pairs, min_rel_angle_deg):
    """Return (X, res) once solvable, ('err', msg) if not enough info, or None (<4)."""
    if len(pairs) < 4:
        return None
    A = [pose7_to_T(p["A_tcp_stand"]) for p in pairs]
    B = [pose7_to_T(p["B_tracker_steamvr"]) for p in pairs]
    try:
        X, _Y, res = solve_handeye(A, B, min_rel_angle_deg=min_rel_angle_deg)
        return X, res
    except Exception as exc:  # not enough rotation spread yet, etc.
        return ("err", str(exc))


def _write_pairs(path, side, pairs):
    out = {
        "schema": "robotics_lab.handeye_pairs.v1",
        "side": side,
        "source_pose_frame": "steamvr_world",
        "target_pose_frame": "stand",
        "pairs": pairs,
    }
    with open(path, "w") as f:
        json.dump(out, f, indent=2)


def _window_stable(samples, key, hold_sec, still_trans, still_rot):
    """True if all `key` poses within the last hold_sec are within thresholds of the latest."""
    if not samples:
        return False, None
    now = samples[-1]["t"]
    win = [s[key] for s in samples if s.get(key) is not None and now - s["t"] <= hold_sec]
    if len(win) < 4:
        return False, None
    latest = win[-1]
    for p in win[:-1]:
        dt, dr = _pose_delta(latest, p)
        if dt > still_trans or dr > still_rot:
            return False, latest
    return True, latest


def cmd_capture_dual(args):
    import select
    from collections import deque

    sides = [s for s in ("left", "right") if s in args.arms.split(",")]
    serial = {"left": args.tracker_left, "right": args.tracker_right}
    out = {"left": args.out_left, "right": args.out_right}

    robot = RobotStateUDP(args.robot_state)
    tracker = TrackerOpenVR()
    print(f"[capture-dual] arms={sides}  robot-state={args.robot_state}")
    print(f"  trackers: left={serial['left']}  right={serial['right']}")

    # --- identify mode: just stream what's visible, to map serials to arms ---
    if args.identify:
        print("\n[identify] streaming tracker serials + arm tcp (Ctrl-C to stop)."
              "\n  Wiggle ONE arm; the serial whose position changes is that arm's tracker.\n")
        try:
            while True:
                tp = tracker.poses()
                la = robot.tcp_stand("left"); ra = robot.tcp_stand("right")
                msg = "  trackers: " + ", ".join(
                    f"{sn}=({p[0]:+.2f},{p[1]:+.2f},{p[2]:+.2f})" for sn, p in tp.items()
                ) if tp else "  trackers: (none)"
                print(msg + f"   | L_tcp={'ok' if la else '--'} R_tcp={'ok' if ra else '--'}")
                time.sleep(0.3)
        except KeyboardInterrupt:
            pass
        finally:
            robot.close(); tracker.close()
        return 0

    if any(serial[s] is None for s in sides):
        print("ERROR: pass --tracker-left/--tracker-right serials (run with --identify "
              "first to find them).", file=sys.stderr)
        robot.close(); tracker.close()
        return 2

    pairs = {s: [] for s in sides}
    samples = deque(maxlen=200)
    auto = args.auto

    def snapshot(side):
        A = robot.tcp_stand(side)
        tp = tracker.poses()
        B = tp.get(serial[side])
        return A, B

    def do_capture(side, force=False):
        A, B = snapshot(side)
        if A is None:
            print(f"  [{side}] !! no robot tcp_stand (server publishing to {args.robot_state}?)")
            return False
        if B is None:
            print(f"  [{side}] !! tracker {serial[side]} not visible (occluded / SteamVR?)")
            return False
        dt, dr = _min_delta_to([p["A_tcp_stand"] for p in pairs[side]], A)
        if not force and (dt < args.min_move_trans and dr < args.min_move_deg):
            return False  # too close to an existing pose; skip silently in auto
        pairs[side].append({"A_tcp_stand": A, "B_tracker_steamvr": B, "t": time.time()})
        _write_pairs(out[side], side, pairs[side])
        spread = _max_rot_spread([p["A_tcp_stand"] for p in pairs[side]])
        line = f"  [{side}] #{len(pairs[side])}  orient-spread={spread:5.1f}deg"
        rs = _running_solve(pairs[side], args.min_rel_angle_deg)
        if rs and rs[0] != "err":
            X, res = rs
            line += (f" | resid {res['trans_rms_m']*1000:5.1f}mm/{res['rot_rms_deg']:.2f}deg")
        elif rs and rs[0] == "err":
            line += " | (need more orientation variety to solve)"
        print(line)
        _print_crosscheck()
        return True

    def _print_crosscheck():
        if not all(len(pairs[s]) >= 4 for s in ("left", "right") if s in sides) or len(sides) < 2:
            return
        rl = _running_solve(pairs["left"], args.min_rel_angle_deg)
        rr = _running_solve(pairs["right"], args.min_rel_angle_deg)
        if not (rl and rr and rl[0] != "err" and rr[0] != "err"):
            return
        Xl, Xr = rl[0], rr[0]
        dr = math.degrees(rot_angle(Xl[:3, :3].T @ Xr[:3, :3]))
        dt = float(np.linalg.norm(Xl[:3, 3] - Xr[:3, 3]))
        flag = "  <-- should be ~0; >2deg means a grasp slipped / FK issue" if dr > 2.0 else "  (good)"
        print(f"    L/R T_stand_source agreement: rot {dr:5.2f}deg, trans {dt*1000:5.1f}mm{flag}")

    print("\nDrive the arms with SpaceMouse teleop. Vary ORIENTATION a lot (the solver needs")
    print("rotational diversity), pause to let a pose settle. Commands:")
    print("  [ENTER] capture all arms now   a) toggle auto-capture   u) undo last   "
          "s) show solve   q) finish\n")
    if auto:
        print("  AUTO-CAPTURE ON: hold an arm still (and moved enough from prior poses) -> auto-snap\n")

    try:
        while True:
            now = time.time()
            s = {"t": now}
            for side in sides:
                s[side + "_A"] = robot.tcp_stand(side)
                tp = tracker.poses()
                s[side + "_B"] = tp.get(serial[side])
            samples.append(s)

            if auto:
                for side in sides:
                    ok_a, _ = _window_stable(samples, side + "_A", args.hold_sec,
                                             args.still_trans, args.still_rot)
                    ok_b, _ = _window_stable(samples, side + "_B", args.hold_sec,
                                             args.still_trans_track, args.still_rot)
                    if ok_a and ok_b:
                        do_capture(side, force=False)

            if select.select([sys.stdin], [], [], args.poll_sec)[0]:
                line = sys.stdin.readline().strip().lower()
                if line == "q":
                    break
                elif line == "a":
                    auto = not auto
                    print(f"  >> auto-capture {'ON' if auto else 'OFF'}")
                elif line == "u":
                    for side in sides:
                        if pairs[side]:
                            pairs[side].pop()
                            _write_pairs(out[side], side, pairs[side])
                    print("  >> undid last capture on all arms")
                elif line == "s":
                    for side in sides:
                        rs = _running_solve(pairs[side], args.min_rel_angle_deg)
                        if rs is None:
                            print(f"  [{side}] {len(pairs[side])} pairs (need >=4 to solve)")
                        elif rs[0] == "err":
                            print(f"  [{side}] {len(pairs[side])} pairs - {rs[1]}")
                        else:
                            X, res = rs
                            print(f"  [{side}] {res['n_pairs']} pairs  "
                                  f"T_stand_source={np.round(T_to_pose7(X),4).tolist()}  "
                                  f"resid {res['trans_rms_m']*1000:.1f}mm/{res['rot_rms_deg']:.2f}deg")
                    _print_crosscheck()
                else:  # ENTER -> manual capture all arms (force past the min-move gate)
                    for side in sides:
                        do_capture(side, force=True)
            else:
                time.sleep(0.0)  # select already waited poll_sec
    finally:
        robot.close()
        tracker.close()

    for side in sides:
        _write_pairs(out[side], side, pairs[side])
        print(f"[capture-dual] wrote {len(pairs[side])} pairs -> {out[side]}")
    print("\nNext (tomorrow): solve + write the retarget yaml, e.g.")
    pl = out.get("left"); pr = out.get("right")
    print(f"  python3 scripts/handeye_umi_retarget.py solve "
          f"{'--pairs-left ' + pl if 'left' in sides else ''} "
          f"{'--pairs-right ' + pr if 'right' in sides else ''} "
          f"--status measured --measured-date <YYYY-MM-DD> "
          f"--out calibration/umi_retarget.yaml")
    return 0


# ---------------------------------------------------------------------------
# Solve + write umi_retarget.yaml
# ---------------------------------------------------------------------------


def _load_pairs(path):
    with open(path) as f:
        d = json.load(f)
    A = [pose7_to_T(p["A_tcp_stand"]) for p in d["pairs"]]
    B = [pose7_to_T(p["B_tracker_steamvr"]) for p in d["pairs"]]
    return A, B, d.get("side")


def _gripper_T_tcp_umi(gripper_in_tracker, rot_xyzw):
    """Build T_tcp_umi_gripper s.t. inverse(it) = (trans=gripper_in_tracker, rot=rot)."""
    inv = np.eye(4)
    inv[:3, :3] = quat_xyzw_to_rot(rot_xyzw)
    inv[:3, 3] = np.asarray(gripper_in_tracker, float)
    return T_to_pose7(T_inv(inv))


def _arm_block(X, gripper_pose7, units):
    return {
        "T_stand_source": [float(v) for v in np.round(T_to_pose7(X), 9)],
        "T_tcp_umi_gripper": [float(v) for v in np.round(gripper_pose7, 9)],
        "gripper_open_close_units": units,
    }


def cmd_solve(args):
    git = [float(v) for v in args.gripper_in_tracker.split(",")]
    grot = [float(v) for v in args.gripper_rot_xyzw.split(",")]
    gripper_pose7 = _gripper_T_tcp_umi(git, grot)

    sides = {}
    for side, path in (("left", args.pairs_left), ("right", args.pairs_right)):
        if not path:
            continue
        A, B, tagged = _load_pairs(path)
        if tagged and tagged != side:
            print(f"  WARNING: {path} tagged side={tagged} but used as {side}")
        X, Y, res = solve_handeye(A, B, min_rel_angle_deg=args.min_rel_angle_deg)
        sides[side] = X
        print(f"\n[solve:{side}] {res['n_pairs']} pairs")
        print(f"  T_stand_source = {np.round(T_to_pose7(X),5).tolist()}")
        print(f"  residual  trans rms={res['trans_rms_m']*1000:.2f}mm "
              f"max={res['trans_max_m']*1000:.2f}mm | "
              f"rot rms={res['rot_rms_deg']:.3f}deg max={res['rot_max_deg']:.3f}deg")
        if res["trans_rms_m"] > 0.01 or res["rot_rms_deg"] > 1.0:
            print("  !! residual above target (<10mm / <1deg): add poses / check rigidity")

    if not sides:
        print("ERROR: provide --pairs-left and/or --pairs-right", file=sys.stderr)
        return 2

    identity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    doc = {
        "schema": "robotics_lab.umi_retarget.v1",
        "status": args.status,
        "source_pose_frame": "steamvr_world",
        "target_pose_frame": "stand",
        "left": _arm_block(sides["left"], gripper_pose7, args.gripper_units)
        if "left" in sides else {
            "T_stand_source": identity, "T_tcp_umi_gripper": gripper_pose7,
            "gripper_open_close_units": args.gripper_units},
        "right": _arm_block(sides["right"], gripper_pose7, args.gripper_units)
        if "right" in sides else {
            "T_stand_source": identity, "T_tcp_umi_gripper": gripper_pose7,
            "gripper_open_close_units": args.gripper_units},
        "quality": {
            "measured_date": args.measured_date,
            "max_translation_error_m": None,
            "max_rotation_error_rad": None,
        },
    }
    try:
        import yaml
        with open(args.out, "w") as f:
            yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False)
    except ImportError:
        with open(args.out, "w") as f:
            json.dump(doc, f, indent=2)
    missing = {"left", "right"} - set(sides)
    note = f"  (sides solved: {sorted(sides)}; left as identity: {sorted(missing)})" if missing else ""
    print(f"\n[solve] wrote {args.out}{note}")
    if missing:
        print("  NOTE: unsolved side left at identity -- not valid for that arm.")
    return 0


def cmd_selftest(args):
    rng = np.random.default_rng(0)

    def rand_T(tscale=0.5):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        T = np.eye(4)
        T[:3, :3] = quat_xyzw_to_rot([q[1], q[2], q[3], q[0]])
        T[:3, 3] = rng.normal(size=3) * tscale
        return T

    X_true = rand_T(); Y_true = rand_T(0.2)
    A_list, B_list = [], []
    for _ in range(args.n):
        A = rand_T()
        B = T_inv(X_true) @ A @ Y_true            # since A Y = X B  => B = X^-1 A Y
        if args.noise > 0:
            B[:3, 3] += rng.normal(size=3) * args.noise
        A_list.append(A); B_list.append(B)
    X, Y, res = solve_handeye(A_list, B_list)
    dR = math.degrees(rot_angle(T_inv(X_true)[:3, :3] @ X[:3, :3]))
    dt = float(np.linalg.norm(X_true[:3, 3] - X[:3, 3]))
    print(f"[selftest] n={args.n} noise={args.noise}m")
    print(f"  recovered X rot err = {dR:.4f} deg, trans err = {dt*1000:.3f} mm")
    print(f"  residual: {res}")
    ok = dR < (0.5 if args.noise else 1e-3) and dt < (0.02 if args.noise else 1e-4)
    print("  RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capture", help="record (robot tcp_stand, vive tracker) pairs")
    c.add_argument("--side", required=True, choices=["left", "right"])
    c.add_argument("--robot-state", default="udp://0.0.0.0:50120",
                   help="UDP bind for servo_state.v1 (default policy_runner port 50120)")
    c.add_argument("--tracker-serial", default=None,
                   help="Vive tracker serial (required if >1 tracker visible)")
    c.add_argument("--out", required=True)
    c.set_defaults(func=cmd_capture)

    cd = sub.add_parser(
        "capture-dual",
        help="teleop-friendly capture: both arms, one process, auto/manual + live residual")
    cd.add_argument("--robot-state", default="udp://127.0.0.1:50376",
                    help="UDP bind for servo_state.v1. During SpaceMouse teleop, policy_runner "
                         "holds 50120 -- bind a free fanout endpoint instead (stack_real.yaml "
                         "publishes 50376). Confirm it's in network.state_pub_endpoints.")
    cd.add_argument("--tracker-left", default=None, help="left Sense Vive tracker serial")
    cd.add_argument("--tracker-right", default=None, help="right Sense Vive tracker serial")
    cd.add_argument("--arms", default="left,right", help="comma list: left,right | left | right")
    cd.add_argument("--out-left", default="calibration/handeye_pairs_left.json")
    cd.add_argument("--out-right", default="calibration/handeye_pairs_right.json")
    cd.add_argument("--identify", action="store_true",
                    help="stream visible tracker serials + arm tcp to map serials->arms, then exit")
    cd.add_argument("--auto", action="store_true",
                    help="auto-capture when an arm holds still and is far enough from prior poses")
    cd.add_argument("--hold-sec", type=float, default=0.4,
                    help="stationarity window for auto-capture")
    cd.add_argument("--still-trans", type=float, default=0.002,
                    help="max robot tcp drift over the window to count as still (m)")
    cd.add_argument("--still-trans-track", type=float, default=0.004,
                    help="max tracker drift over the window to count as still (m)")
    cd.add_argument("--still-rot", type=float, default=0.5,
                    help="max rotation drift over the window to count as still (deg)")
    cd.add_argument("--min-move-deg", type=float, default=8.0,
                    help="auto-capture only if >= this rotation from every prior pose (deg)")
    cd.add_argument("--min-move-trans", type=float, default=0.03,
                    help="...or >= this translation from every prior pose (m)")
    cd.add_argument("--min-rel-angle-deg", type=float, default=2.0,
                    help="min relative rotation for a pose pair to inform the running solve")
    cd.add_argument("--poll-sec", type=float, default=0.05)
    cd.set_defaults(func=cmd_capture_dual)

    s = sub.add_parser("solve", help="solve T_stand_source and write umi_retarget.yaml")
    s.add_argument("--pairs-left", default=None)
    s.add_argument("--pairs-right", default=None)
    s.add_argument("--out", default="calibration/umi_retarget.yaml")
    s.add_argument("--status", default="measured",
                   choices=["measured", "accepted", "configured_estimate"])
    s.add_argument("--measured-date", default=None, help="e.g. 2026-06-08")
    s.add_argument("--gripper-in-tracker", default="0.172,0.0,-0.076",
                   help="gripper origin in tracker-local frame (m), from PIKA SDK")
    s.add_argument("--gripper-rot-xyzw", default="0,0,0,1",
                   help="tracker->gripper rotation (xyzw); identity unless CAD says otherwise")
    s.add_argument("--gripper-units", default="percent", choices=["percent", "mm", "raw"])
    s.add_argument("--min-rel-angle-deg", type=float, default=2.0)
    s.set_defaults(func=cmd_solve)

    t = sub.add_parser("selftest", help="numeric solver check with synthetic data")
    t.add_argument("--n", type=int, default=20)
    t.add_argument("--noise", type=float, default=0.0, help="tracker translation noise (m)")
    t.set_defaults(func=cmd_selftest)

    args = ap.parse_args()
    sys.exit(args.func(args) or 0)


if __name__ == "__main__":
    main()
