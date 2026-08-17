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

    def __init__(self, cmd_endpoint=("127.0.0.1", 50410), fb_bind=("127.0.0.1", 50421)):
        # NOTE: gripper_server pushes state to 50420; the legacy consumer there is
        # rb_servo_server. We bind 50421 unless free — actually the server sends
        # to a configured endpoint (50420); we bind it directly since rb_servo_server
        # is not running in the CM stack.
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_endpoint = cmd_endpoint
        self._seq = 0
        self.position = {"left": None, "right": None}
        self._last = {"left": None, "right": None}
        try:
            self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._rx.bind(("127.0.0.1", 50420))
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
            for side in ("left", "right"):
                arm = st.get(side) or {}
                pos = arm.get("position_percent", arm.get("percent"))
                if isinstance(pos, (int, float)):
                    self.position[side] = float(pos)

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
                continue
            gl = (cmd.get("left") or {}).get("gripper_target")
            gr = (cmd.get("right") or {}).get("gripper_target")
            if gl is not None or gr is not None:
                self._node.gripper.command(left=gl, right=gr)
            if cmd.get("mode") != "JointTarget":
                continue  # P1: everything else is FollowUnit's or out of scope
            for side in ("left", "right"):
                arm = cmd.get(side) or {}
                q_deg = arm.get("q_target_deg")
                if isinstance(q_deg, list) and len(q_deg) >= 6:
                    self._node.get_logger().info(f"reset MOVJ {side}: {q_deg[:6]}")
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
        self.gripper = GripperForwarder()
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
            elif msg.get("cmd") == "collision_clear":
                self.get_logger().warning("collision latch cleared by operator")
                self.latched_reason = None

    def _on_frame(self, frame):
        # Called from the ingress thread; rclpy publishers are thread-safe.
        if self.latched_reason is not None:
            return  # fail-closed: no follow chunks while latched
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
    ap.add_argument("--command-bind", default="0.0.0.0:50256")
    ap.add_argument("--control-bind", default="127.0.0.1:50259")
    ap.add_argument("--state-endpoints",
                    default="127.0.0.1:50356,127.0.0.1:50366,127.0.0.1:50376,127.0.0.1:50378,127.0.0.1:50388",
                    help="servo_state.v1 UDP fanout targets (legacy port map)")
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
