#!/usr/bin/env python3
"""cm_bridge — policy chunk stream -> controller-manager FollowUnit.

Runs INSIDE the CM docker image (rclpy available, host network). P1 scope:

  UDP 50264 chunk frames (robotics_lab.chunk_overlay.v2/v3, absolute
  stand-frame TCP rows [x,y,z,qx,qy,qz,qw] per arm, policy_dt_sec)
    -> per-period LOCAL deltas (T_k^-1 o T_{k+1}; frame-independent, see
       cm_bridge/docs/design.md D2 — flange-vs-tool audit tracked as R1)
    -> geometry_msgs/PoseArray on /monkey/<side>/cmd/follow
       (REPLACE-at-boundary receding horizon, FollowUnit contract)

  CM state -> robotics_lab.servo_state.v1 UDP fanout re-publish (minimal
  field set; topic/type configurable until the R2/R3 audits settle).

NOT yet wired (P1 remainder): command JSON 50256 reset/init -> MOVJ action,
collision gate (P2). Fail-closed posture: on chunk-stream anomalies the
bridge simply stops publishing follow chunks — FollowUnit brakes to rest and
exits to Idle on its silence gate by design.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

ACCEPTED_SCHEMAS = (
    "robotics_lab.chunk_overlay.v2",
    "robotics_lab.chunk_overlay.v3",
    "robotics_lab.chunk_frame.v1",
)


def q_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_rotate_inv(q, v):
    """R(q)^T v — rotate v into the local frame of q."""
    vx = (v[0], v[1], v[2], 0.0)
    r = q_mul(q_mul(q_conj(q), vx), q)
    return (r[0], r[1], r[2])


def rows_to_local_deltas(rows):
    """Consecutive absolute rows -> per-period deltas in the row-k local frame.

    delta translation = R_k^T (p_{k+1} - p_k); delta rotation = q_k^-1 * q_{k+1}.
    Frame-independent of the stand<->CM extrinsic (design.md D2).
    """
    deltas = []
    for k in range(len(rows) - 1):
        p0, q0 = rows[k][0:3], rows[k][3:7]
        p1, q1 = rows[k + 1][0:3], rows[k + 1][3:7]
        dp = q_rotate_inv(q0, (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]))
        dq = q_mul(q_conj(tuple(q0)), tuple(q1))
        n = math.sqrt(sum(c * c for c in dq)) or 1.0
        dq = tuple(c / n for c in dq)
        if dq[3] < 0.0:  # keep the short arc
            dq = tuple(-c for c in dq)
        deltas.append((dp, dq))
    return deltas


def rotvec_to_quat_xyzw(r):
    """Rotation vector [rad] -> quaternion (x, y, z, w), short arc."""
    ang = math.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
    if ang < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(ang / 2.0) / ang
    return (r[0] * s, r[1] * s, r[2] * s, math.cos(ang / 2.0))


class FollowPacer:
    """Controller-paced N-step commit of the policy chunk stream, WITHOUT cutting a step short
    (2026-08-19, operator decision).

    THE CONTRACT. Every policy step is executed to the letter: its full delta, at no more than
    the follow envelope, taking as many controller periods as that needs (TIME-STRETCH); a
    fresh chunk takes over only when the current one has finished N whole steps; and the
    per-step gripper target fires exactly when THAT step's motion has arrived. Nothing about a
    step is dropped because time ran out - the runner's clock and the controller's clock are
    deliberately decoupled here.

    HOW. The controller (FollowUnit) plays one delta per input period T at constant rate
    ||delta||/T and CUTS a delta that exceeds max_vel*T / max_rot*T. So the pacer SUBDIVIDES:
    each policy step delta d becomes n = max(ceil(|dt|/(v_max T)), ceil(|dr|/(w_max T)), 1)
    equal sub-deltas d/n, each inside the envelope, so the controller plays them back-to-back
    at exactly the cap and the step takes n periods. The step's gripper target rides ONLY the
    LAST sub-delta (aux0; aux1 = the policy step index), so the controller's "that delta just
    finished" event fires the gripper at the step's arrival point. Envelope values are read
    from the SAME follow.yaml the controller loads (fail-closed if unreadable).

    HAND-OVER. The pacer keeps the newest runner chunk per arm as a candidate and counts the
    controller's progress in POLICY STEPS from its act/follow_step events (aux1 tells which
    policy step each sub-delta belongs to). One period before the boundary that completes N
    policy steps of the current message it publishes the next message: the newest candidate,
    sliced by how many policy steps the controller has STARTED since that candidate arrived
    (the runner anchored the candidate at the arm's command pose when it activated it, so its
    row 0 is "from where the arm was then"; steps started after that are behind us). No use of
    the runner's own step counter - stretch, starvation and drift all fall out of the count.
    A candidate exhausted at that point means "continue what you hold" (every message carries
    all remaining sub-deltas, capped at max_chunk). The controller runs `commit_steps: 1` in
    this mode: adoption timing is the pacer's, and it publishes only at hand-over points.

    `mode == "replace"` keeps the pre-2026-08-19 behaviour (publish every frame at once) for A/B.
    """

    def __init__(self, node, platform, commit_steps, mode, gripper_source, envelope, max_chunk):
        from sensor_msgs.msg import JointState
        self._node = node
        self.N = max(1, int(commit_steps))
        self.mode = mode
        self.gripper_source = gripper_source
        self.env = envelope                    # {"T": s, "vmax_mm": mm/s, "wmax_dps": deg/s}
        self.max_chunk = int(max_chunk)
        self._JointState = JointState
        self._lock = threading.Lock()
        self.cand = {s: [] for s in ("left", "right")}       # newest-last candidates
        self.pubs = {s: {} for s in ("left", "right")}       # stamp -> {cand, sub_skip, orig_skip, subs}
        self.ctrl = {s: {"stamp": 0, "idx": -1, "n": 0, "playing": False, "kind": None,
                         "event_mono": 0, "events": 0, "orig_starts": 0, "cur_orig": None,
                         "commit_start_orig": None} for s in ("left", "right")}
        self.last_pub = {s: None for s in ("left", "right")}  # (cand seq, orig_skip)
        self.window = {s: None for s in ("left", "right")}
        self.stats = {"published": 0, "held": 0, "starved": 0, "grip_events": 0,
                      "subs": 0, "steps": 0, "max_stretch": 0}
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self._aux_pub = {s: node.create_publisher(JointState, f"/{platform}/{s}/cmd/follow_aux", qos)
                         for s in ("left", "right")}
        step_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=64)
        for s in ("left", "right"):
            node.create_subscription(JointState, f"/{platform}/{s}/act/follow_step",
                                     lambda m, side=s: self._on_step(side, m), step_qos)
        self._warned = set()
        self._period_ns = int(self.env["T"] * 1e9)
        self._last_frame_ns = 0
        self._last_midstep_warn_ns = 0
        # per-sub-delta caps with a hair of margin so a boundary case never trips the controller cut
        self.cap_t_m = self.env["vmax_mm"] * 1e-3 * self.env["T"] * 0.995
        self.cap_r_rad = math.radians(self.env["wmax_dps"]) * self.env["T"] * 0.995

    # ------------------------------------------------------------ subdivision
    def _subdivide(self, deltas, grips):
        """[(dp, dq)] policy steps -> [(dp_sub, dq_sub, aux0, aux1)] sub-deltas. Every sub-delta
        is inside the envelope; the step's gripper target rides ONLY its last sub-delta."""
        subs = []
        max_n = 0
        for k, (dp, dq) in enumerate(deltas):
            rv = _quat_to_rotvec(dq)
            nt = math.sqrt(dp[0] ** 2 + dp[1] ** 2 + dp[2] ** 2)
            nr = math.sqrt(rv[0] ** 2 + rv[1] ** 2 + rv[2] ** 2)
            n = max(1, math.ceil(nt / self.cap_t_m - 1e-9) if self.cap_t_m > 0 else 1,
                    math.ceil(nr / self.cap_r_rad - 1e-9) if self.cap_r_rad > 0 else 1)
            max_n = max(max_n, n)
            g = grips[k] if k < len(grips) else None
            g = float(g) if isinstance(g, (int, float)) and math.isfinite(g) else float("nan")
            for j in range(n):
                sub_dp = (dp[0] / n, dp[1] / n, dp[2] / n)
                sub_dq = rotvec_to_quat_xyzw((rv[0] / n, rv[1] / n, rv[2] / n))
                last = (j == n - 1)
                subs.append((sub_dp, sub_dq, g if last else float("nan"), float(k)))
        return subs, max_n

    # ------------------------------------------------------------ runner side
    def on_frame(self, frame, recv_ns):
        self._last_frame_ns = recv_ns
        meta = frame.get("chunk_metadata") or {}
        act = meta.get("activation_step_seq")
        act = int(act) if isinstance(act, (int, float)) else None
        for side, rows in frame["arms"].items():
            deltas, grips = self._deltas_for(frame, side, rows)
            if not deltas:
                continue
            subs, max_n = self._subdivide(deltas, grips)
            with self._lock:
                c = self.ctrl[side]
                cand = {"seq": int(frame["seq"]), "recv_ns": recv_ns, "act": act,
                        "deltas": deltas, "grips": grips, "subs": subs, "max_stretch": max_n,
                        "orig_starts_at_recv": c["orig_starts"],
                        "host_time_ns": frame.get("host_time_ns")}
                hist = self.cand[side]
                hist.append(cand)
                del hist[:-4]
                self.stats["max_stretch"] = max(self.stats["max_stretch"], max_n)
                if self.mode == "replace":
                    self._publish(side, cand, 0, "replace")
                    continue
                if not c["playing"]:
                    self._publish(side, cand, 0, "start")   # arm idle: the runner anchored row 0 at its pose
                    continue
                w = self.window[side]
                if w is not None and recv_ns <= w["until_ns"]:
                    self._publish_next(side, "window-refresh")

    def _deltas_for(self, frame, side, rows):
        d = (frame.get("deltas") or {}).get(side)
        grips = (frame.get("grip") or {}).get(side) or []
        if (frame.get("grip_source") or {}).get(side) == "raw" and "rawgrip" not in self._warned:
            self._warned.add("rawgrip")
            self._node.get_logger().warning(
                "chunk packet carries no per-row gripper COMMAND (left_grip_cmd/right_grip_cmd): the "
                "RAW model opening will be actuated (no close-bias/snap/hold-open) - update flow-infer")
        if isinstance(d, list) and len(d) == len(rows) and all(isinstance(x, list) and len(x) >= 6 for x in d):
            out = []
            for x in d:
                v = [float(u) for u in x[:6]]
                if not all(math.isfinite(u) for u in v):
                    return [], []
                out.append((tuple(v[:3]), rotvec_to_quat_xyzw(v[3:6])))
            return out, [grips[k] if k < len(grips) else None for k in range(len(out))]
        if "rowdiff" not in self._warned:
            self._warned.add("rowdiff")
            self._node.get_logger().warning(
                "chunk packet carries no per-step deltas (left_delta/right_delta): falling back to "
                "row differences - the first step of every chunk is lost and the phase is one step late")
        return rows_to_local_deltas(rows), [grips[k + 1] if k + 1 < len(grips) else None for k in range(len(rows) - 1)]

    # ------------------------------------------------------------ controller side
    def _on_step(self, side, msg):
        f = dict(zip(msg.name, msg.position))
        try:
            kind = int(f["kind"]); stamp = int(f["stamp_ns"]); idx = int(f["idx"]); n = int(f["n"])
            prev_idx = int(f["prev_idx"]); prev_stamp = int(f["prev_stamp_ns"])
            prev_aux0 = float(f["prev_aux0"]); aux1 = float(f["aux1"]); prev_aux1 = float(f["prev_aux1"])
        except (KeyError, ValueError, TypeError):
            return
        mono = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        # 1) the gripper: a finite prev_aux0 means the delta that just FINISHED was the last
        #    sub-delta of a policy step -> that step's target, at its arrival point
        if self.gripper_source == "follow_step" and prev_idx >= 0 and math.isfinite(prev_aux0):
            try:
                self._node.gripper.command(**{side: prev_aux0})
                self.stats["grip_events"] += 1
            except Exception:
                pass
        rec = {"type": "follow_step", "mono_ns": mono, "side": side, "kind": kind, "stamp": stamp,
               "idx": idx, "n": n, "prev_stamp": prev_stamp, "prev_idx": prev_idx,
               "prev_aux0": prev_aux0 if math.isfinite(prev_aux0) else None,
               "orig": int(aux1) if math.isfinite(aux1) else None,
               "prev_orig": int(prev_aux1) if math.isfinite(prev_aux1) else None}
        with self._lock:
            c = self.ctrl[side]
            c.update(kind=kind, stamp=stamp, idx=idx, n=n, event_mono=mono)
            c["events"] += 1
            if kind in (1, 2):
                c["playing"] = True
                pub = self.pubs[side].get(stamp)
                orig = int(aux1) if math.isfinite(aux1) else None
                if kind == 2:                       # a message was adopted: a new commit window
                    # the delta that FINISHED here (prev_*) must have been the last sub-delta of a
                    # policy step - a mid-step adoption means the controller is not adopting at OUR
                    # hand-over points (follow.yaml commit_steps != 1 loaded? not reset yet?)
                    prev_pub = self.pubs[side].get(prev_stamp)
                    if prev_pub is not None and prev_idx >= 0 and not self._is_last_sub(prev_pub, prev_idx):
                        if mono - self._last_midstep_warn_ns > 10_000_000_000:
                            self._last_midstep_warn_ns = mono
                            self._node.get_logger().error(
                                f"{side}: the controller adopted a chunk MID-STEP (prev sub-delta {prev_idx} was not "
                                f"a step end). Is follow.yaml commit_steps: 1 LOADED (reset/restart the controller)?")
                    c["commit_start_orig"] = orig
                    c["cur_orig"] = None
                if orig is not None and orig != c["cur_orig"]:
                    c["cur_orig"] = orig
                    c["orig_starts"] += 1           # a policy step started (any message)
                rec["orig_starts"] = c["orig_starts"]
                # is this sub-delta the LAST of a policy step that completes the commit window?
                last_of_step = pub is not None and self._is_last_sub(pub, idx)
                steps_in_window = (orig - c["commit_start_orig"] + 1) if (orig is not None and c["commit_start_orig"] is not None) else None
                exhausted_next = pub is not None and (idx == n - 1)
                if last_of_step and steps_in_window is not None and (steps_in_window % self.N == 0 or exhausted_next):
                    self.window[side] = {"until_ns": mono + int(self._period_ns * 0.7)}
                    self._publish_next(side, "boundary")
            elif kind == 0:
                c["playing"] = False; c["cur_orig"] = None
                self.stats["starved"] += 1
            elif kind == 3:
                c["playing"] = False; c["cur_orig"] = None
                self.window[side] = None
        self._node.sidecar.write(rec)

    def request_hold(self, reason="runner-hold"):
        """A runner Hold counts only when the runner has actually STOPPED streaming (no chunk for
        two commit windows) and an arm is playing - a runner emits Hold ticks in between chunks too
        (start-up, stalls), and acting on those would clear a chunk it just sent."""
        now = time.monotonic_ns()
        with self._lock:
            playing = any(c["playing"] for c in self.ctrl.values())
        if not playing or now - self._last_frame_ns < int(2 * self.N * self._period_ns):
            return False
        self.hold(reason)
        return True

    def hold(self, reason="hold"):
        """Runner-initiated stop (a Hold command, an episode end): publish ONE zero delta per arm so
        the unit adopts it at its next boundary, plays nothing, and falls back to Idle on the silence
        rule - instead of continuing the tail of the last chunk (up to ~24 stretched steps)."""
        with self._lock:
            for side in ("left", "right"):
                self.cand[side].clear()
                self.window[side] = None
                cand = {"seq": -1, "recv_ns": time.monotonic_ns(), "act": None,
                        "deltas": [((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))], "grips": [None],
                        "subs": [((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), float("nan"), 0.0)],
                        "max_stretch": 1, "orig_starts_at_recv": self.ctrl[side]["orig_starts"],
                        "host_time_ns": None}
                self.last_pub[side] = None
                self._publish(side, cand, 0, reason)

    @staticmethod
    def _is_last_sub(pub, idx):
        subs = pub["subs"]
        return 0 <= idx < len(subs) and (idx == len(subs) - 1 or subs[idx][3] != subs[idx + 1][3])

    def _publish_next(self, side, reason):
        """Publish the newest candidate sliced by the policy steps the controller started since it
        arrived (lock held). Nothing to publish = the unit continues its current message."""
        hist = self.cand[side]
        if not hist:
            return
        c = self.ctrl[side]
        cand = hist[-1]
        orig_skip = max(0, c["orig_starts"] - cand["orig_starts_at_recv"])
        if orig_skip >= len(cand["deltas"]):
            self.stats["held"] += 1
            return
        if self.last_pub[side] == (cand["seq"], orig_skip):
            return
        self._publish(side, cand, orig_skip, reason)

    def _publish(self, side, cand, orig_skip, reason):
        subs = cand["subs"]
        first = next((i for i, s in enumerate(subs) if int(s[3]) >= orig_skip), len(subs))
        subs = subs[first:first + self.max_chunk]
        if not subs:
            return
        now = time.monotonic_ns()
        stamp = _stamp_from_mono(now)
        aux = self._JointState()
        aux.header.stamp = stamp
        aux.header.frame_id = "follow_aux:grip_pct,policy_step"
        vals = []
        for _dp, _dq, a0, a1 in subs:
            vals.extend([a0, a1])
        aux.position = vals
        self._aux_pub[side].publish(aux)          # aux FIRST (same stamp), then the pose chunk
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = f"{side}_local_delta"
        for dp, dq, _a0, _a1 in subs:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = dp
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) = dq
            msg.poses.append(pose)
        self._node._follow_pub[side].publish(msg)
        self.pubs[side][now] = {"cand": cand, "sub_skip": first, "orig_skip": orig_skip, "subs": subs}
        if len(self.pubs[side]) > 64:
            for k in sorted(self.pubs[side])[:-64]:
                del self.pubs[side][k]
        self.last_pub[side] = (cand["seq"], orig_skip)
        self.stats["published"] += 1
        self.stats["subs"] += len(subs)
        self.stats["steps"] += len(set(int(s[3]) for s in subs))
        self._node._last_pub_ns[side] = now
        try:
            self._node.ext.update(side, chunk_seq=cand["seq"], chunk_pub_mono_ns=now)
        except Exception:
            pass
        self._node.sidecar.write({"type": "follow_pub", "mono_ns": now, "side": side, "seq": cand["seq"],
                                  "act": cand["act"], "orig_skip": orig_skip, "sub_skip": first,
                                  "n_sub": len(subs), "n_steps": len(set(int(s[3]) for s in subs)),
                                  "max_stretch": cand["max_stretch"], "reason": reason, "pub_mono_ns": now,
                                  "step_of_sub": [int(s[3]) for s in subs],
                                  "grips": [s[2] if math.isfinite(s[2]) else None for s in subs]})


def _quat_to_rotvec(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    vn = math.sqrt(x * x + y * y + z * z)
    if vn < 1e-12:
        return (0.0, 0.0, 0.0)
    ang = 2.0 * math.atan2(vn, w)
    return (x / vn * ang, y / vn * ang, z / vn * ang)


def load_follow_envelope(path):
    """The controller's follow envelope from the SAME follow.yaml it loads. Fail-closed: no value
    here may be guessed - a wrong cap makes the controller cut steps (the exact thing this pacer
    exists to prevent) or, if too small, only slows the arm."""
    import yaml
    with open(path, encoding="utf-8") as f:
        d = yaml.safe_load(f) or {}
    try:
        env = {"T": float(d["input_period_ms"]) * 1e-3,
               "vmax_mm": float(d["max_vel_mms"]), "wmax_dps": float(d["max_rot_dps"]),
               "max_chunk": int(d.get("max_chunk", 50)), "commit_steps": int(d.get("commit_steps", 1))}
    except (KeyError, TypeError, ValueError) as exc:
        raise SystemExit(f"follow.yaml {path}: missing/invalid envelope key ({exc}) - refusing to pace")
    if not (env["T"] > 0 and env["vmax_mm"] > 0 and env["wmax_dps"] > 0):
        raise SystemExit(f"follow.yaml {path}: non-positive envelope {env}")
    return env


class ChunkIngress(threading.Thread):
    """UDP 50264 listener; keeps only the newest valid frame per arm."""

    def __init__(self, bind, on_frame):
        super().__init__(daemon=True, name="chunk-ingress")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host, port = bind.split(":")
        self._sock.bind((host, int(port)))
        self._sock.settimeout(0.5)
        self._on_frame = on_frame
        self._stop = threading.Event()
        self.stats = {"frames": 0, "rejects": 0, "last_seq": None}

    def run(self):
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            frame = self._parse(data)
            if frame is None:
                self.stats["rejects"] += 1
                continue
            self.stats["frames"] += 1
            self.stats["last_seq"] = frame["seq"]
            self._on_frame(frame)

    def stop(self):
        self._stop.set()

    @staticmethod
    def _parse(data):
        try:
            pkt = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return None
        if pkt.get("schema_version") not in ACCEPTED_SCHEMAS:
            return None
        seq = pkt.get("seq")
        dt = pkt.get("policy_dt_sec")
        if not isinstance(seq, (int, float)) or not isinstance(dt, (int, float)):
            return None
        if not (isinstance(dt, (int, float)) and math.isfinite(dt) and dt > 1e-4):
            return None
        out = {"seq": int(seq), "policy_dt_sec": float(dt), "arms": {}, "grip": {},
               "deltas": {},
               # carried for the sidecar/replay only (never used for control):
               "host_time_ns": pkt.get("host_time_ns"),
               "execute_steps": pkt.get("execute_steps"),
               "runway_steps": pkt.get("runway_steps"),
               # producer-side identity of the inference that made this chunk (inference_seq,
               # observation_bundle_seq, proprio diagnostics, alignment) + its timing - the join
               # key into flow-infer's observation dump. Small dicts; "rolling" stats dropped.
               "chunk_metadata": pkt.get("chunk_metadata") if isinstance(pkt.get("chunk_metadata"), dict) else None,
               "inference_timing": {k: v for k, v in pkt["inference_timing"].items() if k != "rolling"}
                                   if isinstance(pkt.get("inference_timing"), dict) else None}
        for side in ("left", "right"):
            rows = pkt.get(side)
            if rows is None:
                continue
            if not isinstance(rows, list) or len(rows) < 2:
                continue
            ok_rows = []
            grips = []
            for row in rows:
                if not isinstance(row, list) or len(row) < 7:
                    ok_rows = None
                    break
                vals = row[:7]
                if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
                    ok_rows = None
                    break
                ok_rows.append([float(v) for v in vals])
                # column 7 = the policy's per-row gripper target (producer units, %) - replay data
                g = row[7] if len(row) > 7 else None
                grips.append(float(g) if isinstance(g, (int, float)) and math.isfinite(g) else None)
            if ok_rows:
                out["arms"][side] = ok_rows
                # the per-row gripper COMMAND (runner-mapped: close-bias/snap/hold-open) when the
                # packet carries it, else the raw model opening from column 7
                gc = pkt.get(f"{side}_grip_cmd")
                if isinstance(gc, list) and len(gc) == len(ok_rows) and all(isinstance(v, (int, float)) for v in gc):
                    out["grip"][side] = [float(v) for v in gc]
                    out.setdefault("grip_source", {})[side] = "runner_cmd"
                else:
                    out["grip"][side] = grips
                    out.setdefault("grip_source", {})[side] = "raw"
                d = pkt.get(f"{side}_delta")
                if isinstance(d, list):
                    out["deltas"][side] = d
        if not out["arms"]:
            return None
        return out


class Sidecar:
    """The bridge's own append-only JSONL log, on the SAME clock as the controller's recorder.

    controller-manager's `func write` capture (DataRecorder, schema 4) logs per 2 ms tick the
    stamp of the follow chunk it is PLAYING and the level of <side>/cmd/ext_scalars; what it
    cannot hold is the chunk CONTENT (up to 50 rows x 2 arms per message - not a fixed-size
    row) and the wire-level events around it. Those go here, every record stamped with
    time.monotonic_ns() == CLOCK_MONOTONIC == rt::now_ns()'s clock == the recorder's `mono_ns`
    column, so a replay joins the two by plain time comparison:
      chunk    one per accepted 50264 frame: seq/host_time_ns, policy_dt, execute/runway steps,
               absolute rows + per-row grip per arm, per-step deltas, and pub_mono_ns per side =
               the header.stamp we put on the PoseArray = the recorder's fol_chunk_stamp_ns.
      grip_cmd one per gripper_target forwarded (the pi0.5 command as flow-infer dispatched it)
      grip_fb  one per gripper_state.v1 feedback packet
      cmd      JointTarget resets, lease events
    Best-effort: a write failure disables the sidecar and logs once, never touches control.
    """

    SCHEMA = "robotics_lab.cm_bridge_sidecar.v1"

    def __init__(self, path, logger):
        self._f = None
        self._lock = threading.Lock()
        self._logger = logger
        self.path = path
        if not path:
            return
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            self._f = open(path, "a", buffering=1, encoding="utf-8")
            self.write({"type": "session", "schema": self.SCHEMA,
                        "wall_ns": time.time_ns(), "clock": "CLOCK_MONOTONIC (== controller mono_ns)"})
        except OSError as exc:
            self._f = None
            logger.warning(f"sidecar disabled ({exc})")

    def close(self):
        with self._lock:
            if self._f is not None:
                try:
                    self._f.flush(); self._f.close()
                except OSError:
                    pass
                self._f = None

    def write(self, rec):
        if self._f is None:
            return
        rec.setdefault("mono_ns", time.monotonic_ns())
        try:
            line = json.dumps(rec, separators=(",", ":"))
            with self._lock:
                self._f.write(line + "\n")
        except (OSError, TypeError, ValueError) as exc:
            self._logger.warning(f"sidecar write failed, disabling ({exc})")
            self._f = None


def _stamp_from_mono(mono_ns):
    """builtin_interfaces/Time from a CLOCK_MONOTONIC ns value (sec fits int32 for ~68 years)."""
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec = int(mono_ns // 1_000_000_000)
    t.nanosec = int(mono_ns % 1_000_000_000)
    return t


class ExtScalarsPublisher:
    """<side>/cmd/ext_scalars (sensor_msgs/JointState) -> the controller's recorder row.

    THE INDEX CONTRACT (the controller records position[0..7] as ext0..ext7 and header.stamp as
    ext_stamp_ns; it never reads name[]). Values are LEVELS - each publish repeats every slot:
      0 grip_cmd_pct        last gripper command forwarded to gripper_server (pi0.5's dispatch)
      1 grip_fb_pct         last gripper feedback (gripper_state.v1 position_percent)
      2 grip_cmd_mono_ns    CLOCK_MONOTONIC when [0] was forwarded   (double: exact to ~104 days)
      3 grip_fb_mono_ns     CLOCK_MONOTONIC when [1] was received
      4 chunk_seq           wire seq of the last chunk frame published to cmd/follow
      5 chunk_pub_mono_ns   CLOCK_MONOTONIC of that publish (== the PoseArray header.stamp)
      6 chunk_seq_ext_pub   (reserved 0)
      7 (reserved 0)
    Published on every gripper event (cmd forward / feedback), ~30-50 Hz per side. Nothing in the
    controller acts on these; they exist so the 2 ms log carries the gripper on the same timeline.
    """

    NAMES = ["grip_cmd_pct", "grip_fb_pct", "grip_cmd_mono_ns", "grip_fb_mono_ns",
             "chunk_seq", "chunk_pub_mono_ns", "reserved6", "reserved7"]

    def __init__(self, node, platform):
        from sensor_msgs.msg import JointState
        self._JointState = JointState
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._pub = {s: node.create_publisher(JointState, f"/{platform}/{s}/cmd/ext_scalars", qos)
                     for s in ("left", "right")}
        self._lock = threading.Lock()
        self._level = {s: [0.0] * 8 for s in ("left", "right")}

    def close(self):
        """Stop publishing (called on shutdown; the gripper/ingress threads may still be live)."""
        with self._lock:
            self._closed = True

    def update(self, side, **kv):
        """Set named slots for one side and publish the whole level with the current stamp."""
        if getattr(self, "_closed", False):
            return
        with self._lock:
            lv = self._level[side]
            for k, v in kv.items():
                lv[self.NAMES.index(k)] = float(v)
            vals = list(lv)
        msg = self._JointState()
        msg.header.stamp = _stamp_from_mono(time.monotonic_ns())
        msg.name = list(self.NAMES)
        msg.position = vals
        self._pub[side].publish(msg)


class StateRepublisher:
    """CM ROS state -> robotics_lab.servo_state.v1 UDP fanout.

    Field set = the flow-infer real_policy hard requirements (2026-08-16 audit):
    top-level fault_latched/motion_state + permissive command_source readback;
    per arm has_valid_joint_state, q_actual_deg (rad->deg from act/joint),
    has_valid_tcp_pose, tcp_stand and tcp_command_stand (act/pose, cmd/pose —
    CM base_frame; the stand<->CM-root extrinsic is identity until P3 fills the
    calibrated mapping).
    """

    def __init__(self, node, platform, endpoints, rate_hz=100.0):
        from sensor_msgs.msg import JointState
        from geometry_msgs.msg import PoseStamped

        self._node = node
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._endpoints = endpoints
        self._q = {"left": None, "right": None}
        self._tcp = {"left": None, "right": None}
        self._tcp_cmd = {"left": None, "right": None}
        self._seq = 0
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        from geometry_msgs.msg import WrenchStamped
        self._wrench = {"left": None, "right": None}
        for side in ("left", "right"):
            node.create_subscription(
                JointState, f"/{platform}/{side}/act/joint",
                lambda m, s=side: self._q.__setitem__(
                    s, [math.degrees(v) for v in m.position[:6]]), qos)
            node.create_subscription(
                WrenchStamped, f"/{platform}/{side}/act/wrench_af",
                lambda m, s=side: self._wrench.__setitem__(
                    s, [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
                        m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z]), qos)
            node.create_subscription(
                PoseStamped, f"/{platform}/{side}/act/pose",
                lambda m, s=side: self._tcp.__setitem__(s, m.pose), qos)
            node.create_subscription(
                PoseStamped, f"/{platform}/{side}/cmd/pose",
                lambda m, s=side: self._tcp_cmd.__setitem__(s, m.pose), qos)
        node.create_timer(1.0 / rate_hz, self._publish)

    @staticmethod
    def _pose7(p):
        return {
            "x": p.position.x, "y": p.position.y, "z": p.position.z,
            "quaternion_xyzw": [p.orientation.x, p.orientation.y,
                                p.orientation.z, p.orientation.w],
        }

    def _arm(self, side):
        q = self._q[side]
        tcp = self._tcp[side]
        out = {
            "has_valid_joint_state": q is not None,
            "has_valid_tcp_pose": tcp is not None,
        }
        if q is not None:
            out["q_actual_deg"] = q
        if tcp is not None:
            out["tcp_stand"] = self._pose7(tcp)
            out["tcp_actual_stand"] = out["tcp_stand"]
        if self._tcp_cmd[side] is not None:
            out["tcp_command_stand"] = self._pose7(self._tcp_cmd[side])
        g = getattr(self, "gripper", None)
        if g is not None and g.position[side] is not None:
            out["gripper"] = {"gripper_position": g.position[side]}
        if self._wrench[side] is not None:
            out["wrench_af"] = self._wrench[side]
        return out

    def _publish(self):
        self._seq += 1
        fault = getattr(self, "fault_provider", lambda: None)()
        state = {
            "schema": "robotics_lab.servo_state.v1",
            "seq": self._seq,
            "source": "cm_bridge",
            "fault_latched": fault is not None,
            "latched_fault_reason": fault or "",
            "motion_state": "FaultLatched" if fault is not None else "Running",
            # Lease ECHO: CM owns the single command path, so there is no real
            # arbitration here — but legacy clients (flow-infer real_policy)
            # send AcquireLease and BLOCK until the readback names THEM. Echo
            # the last claimant so that handshake completes; unclaimed keeps
            # the permissive cm_bridge stamp.
            "command_source": (lambda lease: {
                "active": True, "expired": False, "enforce_lease": False,
                "active_source_id": lease[0] if lease else "cm_bridge",
                "active_session_id": lease[1] if lease else "cm",
                "active_lease_token": "cm-echo" if lease else "",
                "verdict": "allowed", "reason": "",
                "command_requires_lease": False, "command_has_lease": True,
            })(getattr(self, "lease_provider", lambda: None)()),
            "left": self._arm("left"),
            "right": self._arm("right"),
        }
        payload = json.dumps(state).encode("utf-8")
        for host, port in self._endpoints:
            try:
                self._sock.sendto(payload, (host, port))
            except OSError:
                pass


class GripperForwarder:
    """Command-packet gripper_target -> gripper_server (robotics_lab.gripper_cmd.v1
    on 50410), gripper_state.v1 feedback (50420) -> state fanout stamp.
    Mirrors rb_servo_server's gripper_bridge forwarding role."""

    def __init__(self, cmd_endpoint=("127.0.0.1", 50410), fb_bind=("127.0.0.1", 50420)):
        # gripper_server receives gripper_cmd.v1 on cmd_endpoint (50410) and pushes
        # gripper_state.v1 to fb_bind (50420); in the CM stack this bridge is the sole
        # consumer of that feedback (rb_servo_server is not running). Both are CLI-
        # overridable so an isolated test instance can run beside the live stack
        # without commanding the real grippers.
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_endpoint = cmd_endpoint
        self._seq = 0
        self.position = {"left": None, "right": None}
        self._last = {"left": None, "right": None}
        # Observability taps (set by CmBridge after construction): the sidecar log and the
        # ext_scalars publisher. Both are best-effort and never gate the forwarding.
        self.sidecar = None
        self.ext = None
        try:
            self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._rx.bind(fb_bind)
            self._rx.settimeout(0.5)
            threading.Thread(target=self._fb_loop, daemon=True, name="gripper-fb").start()
        except OSError:
            self._rx = None

    def _fb_loop(self):
        while True:
            try:
                data, _ = self._rx.recvfrom(8192)
                st = json.loads(data.decode("utf-8"))
            except (socket.timeout, OSError, json.JSONDecodeError, UnicodeDecodeError):
                continue
            now = time.monotonic_ns()
            fb = {}
            for side in ("left", "right"):
                arm = st.get(side) or {}
                pos = arm.get("position_percent", arm.get("percent"))
                if isinstance(pos, (int, float)):
                    self.position[side] = float(pos)
                    fb[side] = float(pos)
                    if self.ext is not None:
                        try:
                            self.ext.update(side, grip_fb_pct=float(pos), grip_fb_mono_ns=now)
                        except Exception:
                            pass
            if fb and self.sidecar is not None:
                self.sidecar.write({"type": "grip_fb", "mono_ns": now, "pct": fb,
                                    "seq": st.get("seq")})

    def command(self, left=None, right=None):
        if left is None and right is None:
            return
        if left is not None:
            self._last["left"] = float(left)
        if right is not None:
            self._last["right"] = float(right)
        self._seq += 1
        msg = {"schema": "robotics_lab.gripper_cmd.v1", "seq": self._seq,
               "deadman": True, "host_time_ns": time.time_ns()}
        for side in ("left", "right"):
            if self._last[side] is not None:
                msg[side] = {"percent": self._last[side], "valid": True}
        self._tx.sendto(json.dumps(msg).encode("utf-8"), self._cmd_endpoint)
        now = time.monotonic_ns()
        for side, val in (("left", left), ("right", right)):
            if val is not None and self.ext is not None:
                try:
                    self.ext.update(side, grip_cmd_pct=float(val), grip_cmd_mono_ns=now)
                except Exception:
                    pass
        if self.sidecar is not None:
            self.sidecar.write({"type": "grip_cmd", "mono_ns": now, "seq": self._seq,
                                "pct": {s: v for s, v in (("left", left), ("right", right))
                                        if v is not None}})


class ResetIngress:
    """Legacy command JSON (UDP 50256) JointTarget/init_motion -> CM MOVJ.

    P1 scope: only `mode == "JointTarget"` is honored (episode reset). The goal
    is REGISTERED on /<platform>/<side>/cmd/move (cell_msgs/action/Move,
    kind=MOVJ, one Waypoint) and released with srv/Sync target LEFT/RIGHT.
    Units (SILS-verified 2026-08-16): Move waypoints are DEGREES (matches the
    legacy wire q_target_deg directly; note the STATE topics are radians).
    """

    SYNC_TARGET = {"left": 0, "right": 1}

    def __init__(self, node, args):
        from rclpy.action import ActionClient
        from cell_msgs.action import Move
        from cell_msgs.msg import Waypoint
        from cell_msgs.srv import Sync

        self._node = node
        self._Move, self._Waypoint, self._Sync = Move, Waypoint, Sync
        self._move = {
            s: ActionClient(node, Move, f"/{args.platform}/{s}/cmd/move")
            for s in ("left", "right")
        }
        self._sync = node.create_client(Sync, f"/{args.platform}/cell/cmd/sync")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host, port = args.command_bind.split(":")
        self._sock.bind((host, int(port)))
        self._sock.settimeout(0.5)
        self._stop = threading.Event()
        self._pending = []
        self._plock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True, name="reset-ingress").start()
        # Action/service calls must run on the rclpy executor thread, not the
        # ingress thread (wait-set/context races otherwise) — drain via timer.
        node.create_timer(0.1, self._drain)

    def _loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            try:
                cmd = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            mode = cmd.get("mode")
            if mode in ("AcquireLease", "ReleaseLease"):
                sid, ses = cmd.get("source_id"), cmd.get("session_id")
                claim = (sid, ses) if (mode == "AcquireLease" and sid and ses) else None
                self._node.set_lease_echo(claim)
                self._node.get_logger().info(f"lease echo: {mode} -> {claim}")
                self._node.sidecar.write({"type": "cmd", "mode": mode, "seq": cmd.get("seq"),
                                          "source_id": sid, "session_id": ses})
                continue
            gl = (cmd.get("left") or {}).get("gripper_target")
            gr = (cmd.get("right") or {}).get("gripper_target")
            if gl is not None or gr is not None:
                if self._node.args.gripper_source == "command":
                    self._node.gripper.command(left=gl, right=gr)
                else:
                    # follow_step mode: the gripper rides the chunk and fires at the step the
                    # controller reports finished; the runner's own dispatch is only recorded
                    self._node.sidecar.write({"type": "grip_cmd_runner_ignored", "seq": cmd.get("seq"),
                                              "pct": {k: v for k, v in (("left", gl), ("right", gr)) if v is not None}})
            if cmd.get("mode") == "Hold":
                # the runner asked for a stop (episode end / its own hold): stop the follow at the
                # next boundary instead of playing out the tail of the last chunk
                if getattr(self._node, "pacer", None) is not None and self._node.args.follow_mode == "commit":
                    if self._node.pacer.request_hold("runner-hold"):
                        self._node.get_logger().info("runner Hold with the stream stopped -> follow stopped at the next boundary")
                        self._node.sidecar.write({"type": "cmd", "mode": "Hold", "seq": cmd.get("seq"), "acted": True})
                continue
            if cmd.get("mode") != "JointTarget":
                continue  # P1: everything else is FollowUnit's or out of scope
            for side in ("left", "right"):
                arm = cmd.get(side) or {}
                q_deg = arm.get("q_target_deg")
                if isinstance(q_deg, list) and len(q_deg) >= 6:
                    self._node.get_logger().info(f"reset MOVJ {side}: {q_deg[:6]}")
                    self._node.sidecar.write({"type": "cmd", "mode": "JointTarget", "side": side,
                                              "seq": cmd.get("seq"), "q_target_deg": q_deg[:6]})
                    with self._plock:
                        self._pending.append((side, [float(v) for v in q_deg[:6]]))

    def _drain(self):
        with self._plock:
            items, self._pending = self._pending, []
        for side, q_deg in items:
            self._send_movj(side, q_deg)

    def _send_movj(self, side, q_deg, velocity_pct=20.0):
        goal = self._Move.Goal()
        goal.kind = 1  # MOVJ
        wp = self._Waypoint()
        wp.v = [float(v) for v in q_deg]
        goal.waypoints = [wp]
        goal.velocity_pct = [float(velocity_pct)]
        goal.smoothing_pct = 0.0
        client = self._move[side]
        if not client.server_is_ready():
            self._node.get_logger().error(f"move server absent for {side}")
            return
        fut = client.send_goal_async(goal)
        fut.add_done_callback(lambda f, s=side: self._on_goal_response(f, s))

    def _on_goal_response(self, fut, side):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._node.get_logger().error(f"MOVJ goal REJECTED for {side}")
            return
        self._release(side)

    def _release(self, side):
        req = self._Sync.Request()
        req.target = self.SYNC_TARGET[side]
        sfut = self._sync.call_async(req)
        sfut.add_done_callback(
            lambda f, s=side: self._node.get_logger().info(
                f"sync {s}: accepted={getattr(f.result(), 'accepted', None)} "
                f"msg={getattr(f.result(), 'message', '')!r}"))

    def stop(self):
        self._stop.set()


class CmBridge(Node):
    def __init__(self, args):
        super().__init__("cm_bridge")
        self.args = args
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=4,
        )
        self._follow_pub = {
            side: self.create_publisher(
                PoseArray, f"/{args.platform}/{side}/cmd/follow", qos
            )
            for side in ("left", "right")
        }
        endpoints = []
        for ep in args.state_endpoints.split(","):
            host, port = ep.strip().rsplit(":", 1)
            endpoints.append((host, int(port)))
        self._state = StateRepublisher(self, args.platform, endpoints)
        # Observability first (threads below write into them from their first packet).
        self.sidecar = Sidecar(args.sidecar, self.get_logger())
        self.ext = ExtScalarsPublisher(self, args.platform)
        if self.sidecar.path:
            self.get_logger().info(f"sidecar -> {self.sidecar.path}")
        gh, gp = args.gripper_cmd_endpoint.rsplit(":", 1)
        fh, fp = args.gripper_fb_bind.rsplit(":", 1)
        self.gripper = GripperForwarder(cmd_endpoint=(gh, int(gp)), fb_bind=(fh, int(fp)))
        self.gripper.sidecar = self.sidecar
        self.gripper.ext = self.ext
        self._state.gripper = self.gripper
        # Lease echo state must exist BEFORE ResetIngress spawns its thread
        # (an AcquireLease can arrive immediately).
        self._lease_echo = None            # (source_id, session_id) of last AcquireLease
        self._lease_lock = threading.Lock()
        self._state.lease_provider = lambda: self._lease_echo
        self._reset = ResetIngress(self, args)
        # P2 fail-closed latch: collision monitor trips via the control port;
        # a latched bridge drops all follow chunks and reports fault_latched
        # until an explicit collision_clear (operator decision).
        self.latched_reason = None
        self._state.fault_provider = lambda: self.latched_reason
        threading.Thread(target=self._control_loop, daemon=True,
                         name="control-ingress").start()
        self._last_pub_ns = {"left": 0, "right": 0}
        # The follow envelope the pacer subdivides against = the controller's own follow.yaml.
        env = load_follow_envelope(args.follow_yaml)
        if args.follow_mode == "commit" and env["commit_steps"] != 1:
            # The pacer subdivides steps into sub-deltas and hands over only at policy-step ends;
            # a controller commit counted in sub-deltas would defer those hand-overs unpredictably.
            raise SystemExit(f"{args.follow_yaml}: commit_steps must be 1 when the bridge paces "
                             f"(sub-delta commit would fight the policy-step commit); got {env['commit_steps']}")
        self.pacer = FollowPacer(self, args.platform, args.commit_steps, args.follow_mode,
                                 args.gripper_source, env, env["max_chunk"])
        self.get_logger().info(
            f"follow mode={args.follow_mode} commit_steps={args.commit_steps} (policy steps) "
            f"gripper_source={args.gripper_source} envelope T={env['T']*1e3:.1f}ms vmax={env['vmax_mm']:.0f}mm/s "
            f"wmax={env['wmax_dps']:.0f}dps -> sub-delta caps {self.pacer.cap_t_m*1e3:.2f}mm / "
            f"{math.degrees(self.pacer.cap_r_rad):.2f}deg (from {args.follow_yaml})")
        self._ingress = ChunkIngress(args.chunk_bind, self._on_frame)
        self._ingress.start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"chunk ingress {args.chunk_bind} -> /{args.platform}/<side>/cmd/follow "
            f"(period audit: FollowUnit consumes one delta per input_period_ms; "
            f"chunk policy_dt must match the mounted follow profile)"
        )

    def set_lease_echo(self, claim):
        """(source_id, session_id) of the last AcquireLease, None on release."""
        with self._lease_lock:
            self._lease_echo = claim

    def _control_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host, port = self.args.control_bind.split(":")
        sock.bind((host, int(port)))
        while True:
            try:
                data, _ = sock.recvfrom(4096)
                msg = json.loads(data.decode("utf-8"))
            except (OSError, json.JSONDecodeError, UnicodeDecodeError):
                continue
            if msg.get("cmd") == "collision_trip":
                self.latched_reason = str(msg.get("reason", "collision"))
                self.get_logger().error(f"COLLISION TRIP latched: {self.latched_reason}")
                self.sidecar.write({"type": "collision_trip", "reason": self.latched_reason})
            elif msg.get("cmd") == "collision_clear":
                self.get_logger().warning("collision latch cleared by operator")
                self.latched_reason = None
                self.sidecar.write({"type": "collision_clear"})

    def _on_frame(self, frame):
        # Called from the ingress thread; rclpy publishers are thread-safe.
        recv_ns = time.monotonic_ns()
        if self.latched_reason is not None:
            self.sidecar.write({"type": "chunk_dropped", "mono_ns": recv_ns, "seq": frame["seq"],
                                "reason": self.latched_reason})
            return  # fail-closed: no follow chunks while latched
        # The sidecar record FIRST (the pacer's follow_pub records refer to this seq); publishing
        # is the pacer's decision (immediate in replace mode / at stream start, else at the
        # controller's commit boundary). header.stamp = OUR CLOCK_MONOTONIC at publish: the
        # controller carries it through FollowChunk::stamp_ns into the recorder row while the
        # chunk plays; the follow_pub record holds the same number - the offline join.
        self.sidecar.write({
            "type": "chunk", "mono_ns": recv_ns, "seq": frame["seq"],
            "host_time_ns": frame.get("host_time_ns"),
            "policy_dt_sec": frame["policy_dt_sec"],
            "execute_steps": frame.get("execute_steps"), "runway_steps": frame.get("runway_steps"),
            "pub_mono_ns": {},   # see follow_pub records (a chunk may be published later / sliced)
            "chunk_metadata": frame.get("chunk_metadata"),
            "inference_timing": frame.get("inference_timing"),
            "rows": frame["arms"],          # absolute [x,y,z,qx,qy,qz,qw] per arm (controller base)
            "grip": frame.get("grip", {}),  # per-row gripper COMMAND per arm (runner-mapped, or raw)
            "grip_source": frame.get("grip_source", {}),
            "deltas": frame.get("deltas", {}),
        })
        self.pacer.on_frame(frame, recv_ns)

    def _report(self):
        st = self._ingress.stats
        ps = self.pacer.stats
        self.get_logger().info(
            f"frames={st['frames']} rejects={st['rejects']} last_seq={st['last_seq']} | "
            f"pacer published={ps['published']} held={ps['held']} starved={ps['starved']} "
            f"grip_events={ps['grip_events']} stretch: subs/step={ps['subs']/max(1,ps['steps']):.2f} "
            f"max={ps['max_stretch']}"
        )
        if (self.args.follow_mode == "commit" and ps["published"] > 3
                and sum(c["events"] for c in self.pacer.ctrl.values()) == 0):
            self.get_logger().error(
                "commit mode but NO act/follow_step events from the controller: the stream is running "
                "degraded (every chunk published at once) and the GRIPPER IS NOT BEING COMMANDED. The "
                "controller binary needs cm_bridge/upstream/0002 (follow_step + commit_steps) - restart it.")

    def destroy_node(self):
        # Order matters at teardown: silence the producers that publish from OTHER threads
        # (chunk ingress -> follow, gripper -> ext_scalars) BEFORE the node's publishers go away,
        # or a publish racing rcl_shutdown segfaults the interpreter on exit.
        self._ingress.stop()
        self.ext.close()
        self.gripper.ext = None
        self._reset.stop()
        time.sleep(0.6)   # > the ingress/reset socket timeouts (0.5 s): let their loops exit
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", default="monkey")
    ap.add_argument("--chunk-bind", default="0.0.0.0:50264")
    ap.add_argument("--command-bind", default="0.0.0.0:50256")
    ap.add_argument("--control-bind", default="127.0.0.1:50259")
    ap.add_argument("--state-endpoints",
                    default="127.0.0.1:50356,127.0.0.1:50366,127.0.0.1:50376,127.0.0.1:50378,127.0.0.1:50388",
                    help="servo_state.v1 UDP fanout targets (legacy port map)")
    ap.add_argument("--follow-mode", choices=("commit", "replace"), default="commit",
                    help="commit: controller-paced N-step commit (needs follow.yaml commit_steps + act/follow_step); "
                         "replace: pre-2026-08-19 publish-every-frame behaviour")
    ap.add_argument("--commit-steps", type=int, default=4,
                    help="N POLICY steps per commit (= the runner's execute steps); the controller's follow.yaml "
                         "commit_steps must be 1 (the pacer subdivides steps and hands over at step ends)")
    ap.add_argument("--follow-yaml",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "monkey",
                                         "params-tasks", "follow.yaml"),
                    help="the controller's follow.yaml (envelope for the sub-delta caps; fail-closed if unreadable)")
    ap.add_argument("--gripper-source", choices=("follow_step", "command"), default="follow_step",
                    help="follow_step: per-step gripper target rides the chunk and fires when the controller "
                         "reports that step finished; command: the runner's own gripper_target dispatch (legacy)")
    ap.add_argument("--gripper-cmd-endpoint", default="127.0.0.1:50410",
                    help="gripper_server command endpoint (gripper_cmd.v1)")
    ap.add_argument("--gripper-fb-bind", default="127.0.0.1:50420",
                    help="where gripper_server pushes gripper_state.v1 feedback")
    ap.add_argument("--sidecar",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                         "logs", "cm_bridge_sidecar_" + time.strftime("%Y%m%d_%H%M%S") + ".jsonl"),
                    help="JSONL sidecar log path (chunk content + gripper events on CLOCK_MONOTONIC); "
                         "'' disables")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = CmBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:   # rclpy raises ExternalShutdownException on SIGINT/SIGTERM
        if type(exc).__name__ != "ExternalShutdownException":
            raise
    finally:
        node.destroy_node()
        node.sidecar.close()
        if rclpy.ok():
            rclpy.shutdown()
        # rclpy/rmw_fastrtps tears down C++ statics at interpreter exit and, intermittently,
        # segfaults AFTER everything of ours is closed (observed on Humble; nothing of ours is on
        # the stack - faulthandler prints nothing). Everything is flushed by here, so skip the
        # finalizer walk. The exit code is what a supervisor sees; a spurious 139 is a false alarm.
        sys.stdout.flush(); sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
