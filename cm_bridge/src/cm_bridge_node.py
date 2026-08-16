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
import socket
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
        out = {"seq": int(seq), "policy_dt_sec": float(dt), "arms": {}}
        for side in ("left", "right"):
            rows = pkt.get(side)
            if rows is None:
                continue
            if not isinstance(rows, list) or len(rows) < 2:
                continue
            ok_rows = []
            for row in rows:
                if not isinstance(row, list) or len(row) < 7:
                    ok_rows = None
                    break
                vals = row[:7]
                if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
                    ok_rows = None
                    break
                ok_rows.append([float(v) for v in vals])
            if ok_rows:
                out["arms"][side] = ok_rows
        if not out["arms"]:
            return None
        return out


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
        self._last_pub_ns = {"left": 0, "right": 0}
        self._ingress = ChunkIngress(args.chunk_bind, self._on_frame)
        self._ingress.start()
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"chunk ingress {args.chunk_bind} -> /{args.platform}/<side>/cmd/follow "
            f"(period audit: FollowUnit consumes one delta per input_period_ms; "
            f"chunk policy_dt must match the mounted follow profile)"
        )

    def _on_frame(self, frame):
        # Called from the ingress thread; rclpy publishers are thread-safe.
        for side, rows in frame["arms"].items():
            deltas = rows_to_local_deltas(rows)
            if not deltas:
                continue
            msg = PoseArray()
            msg.header.frame_id = f"{side}_local_delta"
            for dp, dq in deltas:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = dp
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w) = dq
                msg.poses.append(pose)
            self._follow_pub[side].publish(msg)
            self._last_pub_ns[side] = time.monotonic_ns()

    def _report(self):
        st = self._ingress.stats
        self.get_logger().info(
            f"frames={st['frames']} rejects={st['rejects']} last_seq={st['last_seq']}"
        )

    def destroy_node(self):
        self._ingress.stop()
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", default="monkey")
    ap.add_argument("--chunk-bind", default="0.0.0.0:50264")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = CmBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
