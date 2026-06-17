#!/usr/bin/env python3
"""Simulator evaluation of streaming TcpPoseTarget tracking + smoothness.

Brings up the hardware-free Cartesian sim stack (rb_servo_server pinocchio_gate +
two rb_simulator endpoints) exactly like scripts/cartesian_acceptance.py, then
streams absolute TcpPoseTarget setpoints to the LEFT arm under three regimes and
measures how well the arm tracks them and how smooth (tremble-free) the motion is:

  far     - random large step targets (settling, overshoot, IK branch jumps)
  complex - continuous smooth Lissajous pose trajectory (tracking lag, jerk)
  fine    - tiny sub-mm increments / slow ramp (quantization, chatter)

This evaluates the command path the new flow `tcp_target_pose` family emits
(absolute tcp_target_stand setpoints, quaternion authoritative). The sim is
kinematic-ideal, so it isolates command-stream smoothness + server IK behaviour
(ik_solution_jump_deg / branch jumps), not physical actuator dynamics.

Run with the repo's normal python3 (needs numpy + rbsim deps):
  PYTHONPATH=rb_simulator/src python3 scripts/eval_tcp_target_pose_tracking.py
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
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "rb_simulator" / "src"))

import cartesian_acceptance as ca  # noqa: E402


# --------------------------------------------------------------------------- IO
class StampedStateRecorder:
    """UDP state receiver that stamps every packet with a monotonic arrival time.

    Exposes `.snapshots` (payload list) so cartesian_acceptance helpers
    (wait_for / latest_valid_after / arm_motion) work unchanged, plus `.stamped`
    = list of (recv_monotonic_ns, payload) for smoothness analysis.
    """

    def __init__(self, host: str, port: int, output_path: Path) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.settimeout(0.2)
        self._out = output_path.open("w", encoding="utf-8")
        self.snapshots: list[dict[str, Any]] = []
        self.stamped: list[tuple[int, dict[str, Any]]] = []
        self._run = False
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> None:
        self._run = True
        self._thread.start()

    def _loop(self) -> None:
        while self._run:
            try:
                payload, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            recv_ns = time.monotonic_ns()
            try:
                obj = json.loads(payload.decode("utf-8"))
            except Exception:
                continue
            self.snapshots.append(obj)
            self.stamped.append((recv_ns, obj))
            self._out.write(json.dumps(obj, separators=(",", ":")) + "\n")

    def stop(self) -> None:
        self._run = False
        try:
            self._thread.join(timeout=1.0)
        finally:
            try:
                self._sock.close()
            finally:
                self._out.close()


class CommandLog:
    def __init__(self, path: Path) -> None:
        self._out = path.open("w", encoding="utf-8")
        self.sent: list[dict[str, Any]] = []

    def record(self, packet: dict[str, Any]) -> None:
        self.sent.append(packet)
        self._out.write(json.dumps(packet, separators=(",", ":")) + "\n")

    def close(self) -> None:
        self._out.close()


# ---------------------------------------------------------------------- helpers
def rotvec_to_quat(rotvec: np.ndarray) -> list[float]:
    v = np.asarray(rotvec, dtype=float).reshape(3)
    ang = float(np.linalg.norm(v))
    if ang < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    axis = v / ang
    h = 0.5 * ang
    return [*(axis * math.sin(h)).tolist(), math.cos(h)]


def make_target(p0: list[float], q0: list[float], dpos: np.ndarray, drot: np.ndarray) -> dict[str, Any]:
    q = ca.quat_multiply(q0, rotvec_to_quat(drot))
    # Mirror the production _pose_target_arm_payload 7-value object form: server
    # readOptionalPose6D REQUIRES x,y,z,rx,ry,rz; quaternion_xyzw is authoritative.
    return {
        "x": float(p0[0] + dpos[0]),
        "y": float(p0[1] + dpos[1]),
        "z": float(p0[2] + dpos[2]),
        "rx": 0.0,
        "ry": 0.0,
        "rz": 0.0,
        "quaternion_xyzw": q,
    }


def stream_targets(
    ctx: ca.Context,
    cmdlog: CommandLog,
    targets: list[dict[str, Any]],
    hold_per_target_sec: float,
    rate_hz: float,
) -> None:
    """Send each target repeatedly for hold_per_target_sec at rate_hz (deadman-safe)."""
    period = 1.0 / rate_hz
    for target in targets:
        deadline = time.monotonic() + hold_per_target_sec
        while time.monotonic() < deadline:
            packet = ca.pose_target_command(target)
            ca.send_udp(ctx.args.command_host, ctx.args.command_port, packet, cmdlog)
            time.sleep(period)


def quat_angle(a: list[float], b: list[float]) -> float:
    d = abs(sum(x * y for x, y in zip(a, b)))
    d = min(1.0, max(-1.0, d))
    return 2.0 * math.acos(d)


def left_pose_series(stamped: list[tuple[int, dict[str, Any]]], t0_ns: int, t1_ns: int):
    ts, pos, quat = [], [], []
    for recv_ns, snap in stamped:
        if recv_ns < t0_ns or recv_ns > t1_ns:
            continue
        arm = snap.get("left")
        if not isinstance(arm, dict):
            continue
        tcp = arm.get("tcp_stand")
        if not isinstance(tcp, dict):
            continue
        q = tcp.get("quaternion_xyzw")
        if not (isinstance(q, list) and len(q) == 4):
            continue
        ts.append(recv_ns)
        pos.append([float(tcp.get("x", 0.0)), float(tcp.get("y", 0.0)), float(tcp.get("z", 0.0))])
        quat.append([float(v) for v in q])
    return np.asarray(ts, dtype=np.int64), np.asarray(pos, dtype=float), np.asarray(quat, dtype=float)


def commanded_at(cmd_times: np.ndarray, cmd_pos: np.ndarray, cmd_quat: np.ndarray, t_ns: int):
    """Most recent commanded target at arrival time t_ns (zero-order hold)."""
    idx = int(np.searchsorted(cmd_times, t_ns, side="right") - 1)
    if idx < 0:
        idx = 0
    return cmd_pos[idx], cmd_quat[idx]


def smoothness_metrics(ts_ns: np.ndarray, pos: np.ndarray) -> dict[str, float]:
    if len(ts_ns) < 8:
        return {"samples": int(len(ts_ns))}
    t = (ts_ns - ts_ns[0]) / 1e9
    # resample to uniform grid for clean derivatives / FFT
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return {"samples": int(len(ts_ns))}
    grid = np.arange(t[0], t[-1], dt)
    rp = np.stack([np.interp(grid, t, pos[:, i]) for i in range(3)], axis=1)
    vel = np.gradient(rp, dt, axis=0)
    acc = np.gradient(vel, dt, axis=0)
    jerk = np.gradient(acc, dt, axis=0)
    speed = np.linalg.norm(vel, axis=1)
    jerk_mag = np.linalg.norm(jerk, axis=1)
    # chatter: velocity sign reversals on the dominant axis above a noise floor
    dom = int(np.argmax(np.ptp(rp, axis=0)))
    v = vel[:, dom]
    floor = 0.02 * float(np.max(np.abs(v)) + 1e-9)
    sign = np.sign(np.where(np.abs(v) < floor, 0.0, v))
    nz = sign[sign != 0]
    reversals = int(np.sum(np.abs(np.diff(nz)) > 1)) if len(nz) > 1 else 0
    duration = float(grid[-1] - grid[0]) or 1.0
    # high-frequency energy fraction of the speed signal (>5 Hz)
    sp = speed - float(np.mean(speed))
    freqs = np.fft.rfftfreq(len(sp), dt)
    power = np.abs(np.fft.rfft(sp)) ** 2
    total = float(np.sum(power)) + 1e-18
    hf = float(np.sum(power[freqs > 5.0])) / total
    return {
        "samples": int(len(ts_ns)),
        "rate_hz": round(1.0 / dt, 1),
        "rms_jerk_m_s3": round(float(np.sqrt(np.mean(jerk_mag**2))), 4),
        "max_jerk_m_s3": round(float(np.max(jerk_mag)), 4),
        "vel_reversals_per_s": round(reversals / duration, 2),
        "hf_power_frac_gt5hz": round(hf, 4),
        "peak_speed_m_s": round(float(np.max(speed)), 4),
    }


def tracking_metrics(
    stamped, cmdlog: CommandLog, t0_ns: int, t1_ns: int
) -> dict[str, Any]:
    ts, pos, quat = left_pose_series(stamped, t0_ns, t1_ns)
    if len(ts) < 5:
        return {"error": "insufficient state samples", "samples": int(len(ts))}
    cmds = [c for c in cmdlog.sent if t0_ns <= int(c["host_time_ns"]) <= t1_ns and "tcp_target_stand" in c.get("left", {})]
    if not cmds:
        return {"error": "no commands in window"}
    cmd_times = np.asarray([int(c["host_time_ns"]) for c in cmds], dtype=np.int64)
    cmd_pos = np.asarray([[t["x"], t["y"], t["z"]] for t in (c["left"]["tcp_target_stand"] for c in cmds)], dtype=float)
    cmd_quat = np.asarray([c["left"]["tcp_target_stand"]["quaternion_xyzw"] for c in cmds], dtype=float)
    perr, oerr = [], []
    for i in range(len(ts)):
        cp, cq = commanded_at(cmd_times, cmd_pos, cmd_quat, int(ts[i]))
        perr.append(float(np.linalg.norm(pos[i] - cp)))
        oerr.append(quat_angle(quat[i].tolist(), cq.tolist()))
    perr = np.asarray(perr)
    oerr = np.asarray(oerr)
    # steady-state: last 20% of samples (after motion settles)
    tail = max(3, len(perr) // 5)
    out = {
        "samples": int(len(ts)),
        "pos_err_mm": {
            "mean": round(float(np.mean(perr)) * 1e3, 3),
            "p95": round(float(np.percentile(perr, 95)) * 1e3, 3),
            "max": round(float(np.max(perr)) * 1e3, 3),
            "steady_mean": round(float(np.mean(perr[-tail:])) * 1e3, 3),
        },
        "ori_err_deg": {
            "mean": round(math.degrees(float(np.mean(oerr))), 3),
            "p95": round(math.degrees(float(np.percentile(oerr, 95))), 3),
            "max": round(math.degrees(float(np.max(oerr))), 3),
            "steady_mean": round(math.degrees(float(np.mean(oerr[-tail:]))), 3),
        },
    }
    out["smoothness"] = smoothness_metrics(ts, pos)
    return out


def server_ik_metrics(stamped, t0_ns: int, t1_ns: int) -> dict[str, Any]:
    jump, poserr, orierr, dur = [], [], [], []
    branch_suspect = 0
    fault = False
    for recv_ns, snap in stamped:
        if recv_ns < t0_ns or recv_ns > t1_ns:
            continue
        if snap.get("fault_latched") is True:
            fault = True
        tel = snap.get("left", {}).get("cartesian_solve")
        if not isinstance(tel, dict):
            continue
        for key, bucket in (("ik_solution_jump_deg", jump), ("position_error_m", poserr),
                            ("orientation_error_rad", orierr), ("ik_duration_us", dur)):
            v = tel.get(key)
            if isinstance(v, (int, float)) and math.isfinite(v):
                bucket.append(float(v))
        if tel.get("ik_branch_jump_suspected") is True:
            branch_suspect += 1
    return {
        "max_ik_solution_jump_deg": round(max(jump), 3) if jump else None,
        "ik_branch_jump_suspected_count": branch_suspect,
        "max_ik_position_error_m": round(max(poserr), 5) if poserr else None,
        "max_ik_orientation_error_rad": round(max(orierr), 5) if orierr else None,
        "max_ik_duration_us": round(max(dur), 1) if dur else None,
        "fault_latched": fault,
    }


# ---------------------------------------------------------------------- regimes
def build_targets(regime: str, p0: list[float], q0: list[float], rng: np.random.Generator):
    if regime == "far":
        targets = []
        for _ in range(8):
            d = rng.uniform(-1, 1, size=3)
            d = d / (np.linalg.norm(d) + 1e-9) * rng.uniform(0.05, 0.11)
            dr = rng.uniform(-1, 1, size=3)
            dr = dr / (np.linalg.norm(dr) + 1e-9) * rng.uniform(0.08, 0.30)
            targets.append(make_target(p0, q0, d, dr))
        return targets, dict(hold=2.0, rate=100.0)
    if regime == "complex":
        targets = []
        T, rate = 9.0, 100.0
        n = int(T * rate)
        A = np.array([0.045, 0.05, 0.035])
        f = np.array([0.18, 0.27, 0.13])
        for k in range(n):
            t = k / rate
            d = A * np.array([math.cos(2 * math.pi * f[0] * t) - 1.0,
                              math.sin(2 * math.pi * f[1] * t),
                              math.sin(2 * math.pi * f[2] * t)])
            dr = 0.18 * np.array([math.sin(2 * math.pi * 0.15 * t),
                                  math.sin(2 * math.pi * 0.11 * t + 1.0),
                                  math.sin(2 * math.pi * 0.09 * t + 2.0)])
            targets.append(make_target(p0, q0, d, dr))
        return targets, dict(hold=1.0 / rate, rate=rate, streamed_traj=True)
    if regime == "fine":
        targets = []
        T, rate = 6.0, 100.0
        n = int(T * rate)
        for k in range(n):
            s = k / n
            d = np.array([0.02 * s, 0.0, 0.0])              # slow 2cm ramp (~0.03mm/step)
            d[1] = 0.003 * math.floor(s * 10) / 10.0        # 0.3mm staircase steps
            dr = np.array([0.0, 0.0, math.radians(2.0) * s])  # up to 2deg slow yaw
            targets.append(make_target(p0, q0, d, dr))
        return targets, dict(hold=1.0 / rate, rate=rate, streamed_traj=True)
    raise ValueError(regime)


def stream_trajectory(ctx, cmdlog, targets, rate_hz):
    period = 1.0 / rate_hz
    for target in targets:
        packet = ca.pose_target_command(target)
        ca.send_udp(ctx.args.command_host, ctx.args.command_port, packet, cmdlog)
        time.sleep(period)


# ------------------------------------------------------------------------- main
def build_args(artifact_dir: Path) -> argparse.Namespace:
    return argparse.Namespace(
        root=ROOT,
        mode="start-local",
        server=ROOT / "rb_servo_server/build/pinocchio_gate/rb_servo_server",
        server_config=ROOT / "rb_servo_server/config/dual_simulator_tcp_acceptance.yaml",
        left_config=ROOT / "rb_simulator/config/left_rb3_730e.yaml",
        right_config=ROOT / "rb_simulator/config/right_rb3_730e.yaml",
        rbsim_command="python3 -m rbsim",
        command_host="127.0.0.1",
        command_port=50010,
        state_host="127.0.0.1",
        state_port=50110,
        startup_timeout_sec=10.0,
        capture_sec=1.0,
        position_tolerance_m=0.004,
        orientation_tolerance_rad=0.01,
        skip_estop_reset=True,
        record_run_label="",
        near_pi_math_tests_run=False,
    )


def main() -> int:
    ts = time.strftime("%Y%m%dT%H%M%S")
    artifact_dir = ROOT / "artifacts" / "tcp_target_pose_eval" / ts
    artifact_dir.mkdir(parents=True, exist_ok=True)
    args = build_args(artifact_dir)
    ca.prepare_start_local_server_config(args, artifact_dir)
    # Raise state publish rate (default 20 Hz) so trembling up to ~50 Hz is resolvable.
    cfg_text = args.server_config.read_text(encoding="utf-8")
    cfg_text = cfg_text.replace("state_pub_rate_hz: 20", "state_pub_rate_hz: 200")
    # 500 Hz servo loop (moving rbsim plant; pgmode VM freezes q_actual so cannot
    # show motion — rbsim DOES advance q_actual, giving a true 500 Hz loop + motion).
    cfg_text = cfg_text.replace("rate_hz: 100", "rate_hz: 500")
    args.server_config.write_text(cfg_text, encoding="utf-8")

    recorder = StampedStateRecorder(args.state_host, args.state_port, artifact_dir / "state_stream.jsonl")
    cmdlog = CommandLog(artifact_dir / "command_packets.jsonl")
    recorder.start()
    left_proc = right_proc = server_proc = None
    results: dict[str, Any] = {"artifact_dir": str(artifact_dir), "regimes": {}}
    try:
        left_cfg = ca.load_simulator_config(args.left_config)
        right_cfg = ca.load_simulator_config(args.right_config)
        lh, lp = ca.parse_tcp_endpoint(left_cfg.control_bind)
        rh, rp = ca.parse_tcp_endpoint(right_cfg.control_bind)
        env = os.environ.copy()
        env["PYTHONPATH"] = str(ROOT / "rb_simulator/src") + (os.pathsep + env.get("PYTHONPATH", ""))
        left_proc = ca.start_process(["python3", "-m", "rbsim", "--config", str(args.left_config.resolve())], ROOT, artifact_dir / "left.log", env)
        ca.wait_tcp(lh, lp, args.startup_timeout_sec, "left simulator")
        right_proc = ca.start_process(["python3", "-m", "rbsim", "--config", str(args.right_config.resolve())], ROOT, artifact_dir / "right.log", env)
        ca.wait_tcp(rh, rp, args.startup_timeout_sec, "right simulator")
        server_proc = ca.start_process([str(args.server.resolve()), "--config", str(args.server_config.resolve())], artifact_dir, artifact_dir / "server.log")

        ca.wait_for(recorder, ca.valid_state, args.startup_timeout_sec, "valid FK TCP state")
        ctx = ca.Context(args, recorder, cmdlog)
        rng = np.random.default_rng(0)

        for regime in ("far", "complex", "fine"):
            start = ca.arm_motion(ctx)
            sp = ca.arm_pose(start, "left", f"{regime}.start")
            p0 = ca.vec(sp)
            q0 = sp["quaternion_xyzw"]
            targets, cfg = build_targets(regime, p0, q0, rng)
            time.sleep(0.2)
            t0 = time.monotonic_ns()
            if cfg.get("streamed_traj"):
                stream_trajectory(ctx, cmdlog, targets, cfg["rate"])
            else:
                stream_targets(ctx, cmdlog, targets, cfg["hold"], cfg["rate"])
            time.sleep(0.8)  # let final setpoint settle
            t1 = time.monotonic_ns()
            ca.send_udp(args.command_host, args.command_port, ca.hold_command(), cmdlog)
            results["regimes"][regime] = {
                "n_targets": len(targets),
                "tracking": tracking_metrics(recorder.stamped, cmdlog, t0, t1),
                "server_ik": server_ik_metrics(recorder.stamped, t0, t1),
            }
            time.sleep(0.3)
    finally:
        cmdlog.close()
        ca.terminate_process(server_proc)
        ca.terminate_process(right_proc)
        ca.terminate_process(left_proc)
        recorder.stop()

    (artifact_dir / "eval_summary.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
