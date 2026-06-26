#!/usr/bin/env python3
"""TRUE 500 Hz evaluation of streaming TcpPoseTarget against the VM rbpodo stack.

Assumes the 500 Hz pgmode controller-sim server is ALREADY running
(rb_servo_server/build/rbpodo_real_gate/rb_servo_server --config
rb_servo_server/config/local/stack_sim.yaml, command 50256, state 50356/66/76)
with the virtual Rainbow control boxes up (make vm-up).

Uses the PRODUCTION command path: ServoCommandClient (lease) +
tcp_pose_target_stand_intent (the new 7-value object payload). Streams far /
complex / fine TcpPoseTarget profiles and measures, at the real 500 Hz servo
loop, the server cartesian_solve telemetry: IK solve duration vs the 2 ms tick
budget, IK solution jumps / branch jumps, reference-vs-target tracking, and
reference-trajectory smoothness.

NOTE pgmode controller-simulation does NOT move q_actual (tcp_actual_stand is
frozen by design). So this measures the 500 Hz CONTROL/IK path and the commanded
REFERENCE trajectory (tcp_ref_stand), not physical achieved-pose dynamics (the
100 Hz rbsim run covers achieved-pose).
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "policy_runner"))

import eval_tcp_target_pose_tracking as base  # StampedStateRecorder, build_targets, smoothness_metrics
from policy_runner.servo_command_client import ServoCommandClient, CommandIntent
from policy_runner.robot_state_client import RobotStateClient, StateStreamLeaseReadback
from policy_runner.action_sources.tcp_pose_target import tcp_pose_target_stand_intent

CMD = "udp://127.0.0.1:50256"
STATE_REC = "udp://127.0.0.1:50356"
STATE_LEASE = "udp://127.0.0.1:50376"
WARN_US = 1000.0
FAIL_US = 5000.0
BUDGET_US = 2000.0  # 500 Hz tick budget

# rbpodo rest pose (folded, arms up, clear of floor) — GUI default InitMotion
# target. pgmode q_actual follows JOINT commands, so we drive here first to get
# a well-conditioned IK seed before streaming absolute TcpPoseTarget (otherwise
# the IK branch-jump guard clamps every solve to the frozen startup seed).
REST_LEFT_DEG = [-131.663, 72.989, 113.400, -80.880, -107.064, -145.949]
REST_RIGHT_DEG = [135.099, -64.017, -114.457, 84.379, 112.485, 129.893]


def q_actual_left(rec):
    for snap in reversed(rec.snapshots):
        q = snap.get("left", {}).get("q_actual_deg")
        if isinstance(q, list) and len(q) == 6:
            return q
    return None


def init_posture(client, rec, timeout_sec=20.0):
    """Drive both arms to the rest posture via JointTarget; wait for q_actual."""
    deadline = time.monotonic() + timeout_sec
    period = 0.05
    reached = False
    while time.monotonic() < deadline:
        client.send(CommandIntent.joint_target(left=REST_LEFT_DEG, right=REST_RIGHT_DEG, timeout_sec=0.3))
        time.sleep(period)
        q = q_actual_left(rec)
        if q is not None:
            err = max(abs(a - b) for a, b in zip(q, REST_LEFT_DEG))
            if err < 2.0:
                reached = True
                break
    q = q_actual_left(rec)
    err = max(abs(a - b) for a, b in zip(q, REST_LEFT_DEG)) if q else None
    return reached, q, err


def latest_left(rec):
    for snap in reversed(rec.snapshots):
        L = snap.get("left")
        if isinstance(L, dict) and isinstance(L.get("tcp_ref_stand"), dict):
            return snap, L
    return None, None


def left_pose7(L, key="tcp_ref_stand"):
    p = L.get(key) or L.get("tcp_actual_stand") or L.get("tcp_stand")
    q = p["quaternion_xyzw"]
    return [float(p["x"]), float(p["y"]), float(p["z"]), float(q[0]), float(q[1]), float(q[2]), float(q[3])]


def wait_motion_state(rec, wanted, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if rec.snapshots:
            ms = rec.snapshots[-1].get("motion_state")
            if ms in wanted:
                return ms
        time.sleep(0.05)
    return rec.snapshots[-1].get("motion_state") if rec.snapshots else None


def target7(p0, q0, dpos, drot):
    q = _quat_mul(q0, base.rotvec_to_quat(drot))
    return [p0[0] + float(dpos[0]), p0[1] + float(dpos[1]), p0[2] + float(dpos[2]), *q]


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    q = [aw * bx + ax * bw + ay * bz - az * by,
         aw * by - ax * bz + ay * bw + az * bx,
         aw * bz + ax * by - ay * bx + az * bw,
         aw * bw - ax * bx - ay * by - az * bz]
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return [v / n for v in q]


def gen_targets(regime, p0, q0, rng):
    if regime == "large":
        # axis-aligned >=100mm steps (orientation fixed) to test reach + IK timing
        dirs = [(0.12, 0, 0), (-0.12, 0, 0), (0, 0.12, 0), (0, -0.12, 0),
                (0, 0, 0.12), (0, 0, -0.12), (0.10, 0.10, 0.0), (-0.10, 0.0, 0.10)]
        out = [target7(p0, q0, np.array(d), np.zeros(3)) for d in dirs]
        return out, dict(hold=2.0, rate=100.0)
    if regime == "complex":
        out = []; T, rate = 8.0, 200.0
        A = np.array([0.045, 0.05, 0.035]); f = np.array([0.18, 0.27, 0.13])
        for k in range(int(T * rate)):
            t = k / rate
            d = A * np.array([math.cos(2*math.pi*f[0]*t) - 1.0, math.sin(2*math.pi*f[1]*t), math.sin(2*math.pi*f[2]*t)])
            r = 0.12 * np.array([math.sin(2*math.pi*0.15*t), math.sin(2*math.pi*0.11*t+1), math.sin(2*math.pi*0.09*t+2)])
            out.append(target7(p0, q0, d, r))
        return out, dict(rate=rate, stream=True)
    if regime == "policy":
        # policy scale: continuous 1-5 mm motion + occasional small steps + tiny rot
        out = []; T, rate = 8.0, 200.0; n = int(T * rate)
        for k in range(n):
            t = k / rate
            d = np.array([0.003 * math.sin(2*math.pi*0.4*t),                 # ~3mm osc
                          0.002 * math.sin(2*math.pi*0.7*t + 1.0),           # ~2mm osc
                          0.005 * (k // (rate)) / max(1, T)])                # ~5mm/s ramp staircase
            r = np.array([0.0, 0.0, math.radians(0.5) * math.sin(2*math.pi*0.3*t)])
            out.append(target7(p0, q0, d, r))
        return out, dict(rate=rate, stream=True)
    raise ValueError(regime)


def stream(client, targets, cfg):
    rate = cfg["rate"]; period = 1.0 / rate
    if cfg.get("stream"):
        for tg in targets:
            client.send(tcp_pose_target_stand_intent(left=tg, timeout_sec=0.3))
            time.sleep(period)
    else:
        for tg in targets:
            end = time.monotonic() + cfg["hold"]
            while time.monotonic() < end:
                client.send(tcp_pose_target_stand_intent(left=tg, timeout_sec=0.3))
                time.sleep(period)


def metrics(rec, t0_ns, t1_ns):
    ikd, jump, iters, perr, oerr, gmp = [], [], [], [], [], []
    warn = fail = timed = branch_susp = branch_clamp = 0
    overrun = degraded = 0
    ref_ts, ref_pos = [], []
    act_ts, act_pos = [], []
    fault = False
    n = 0
    for recv_ns, snap in rec.stamped:
        if recv_ns < t0_ns or recv_ns > t1_ns:
            continue
        n += 1
        if snap.get("fault_latched") is True:
            fault = True
        if snap.get("send_period_overrun") is True:
            overrun += 1
        if snap.get("tracking_error_degraded") is True:
            degraded += 1
        L = snap.get("left", {})
        ref = L.get("tcp_ref_stand")
        if isinstance(ref, dict):
            ref_ts.append(recv_ns); ref_pos.append([ref["x"], ref["y"], ref["z"]])
        act = L.get("tcp_actual_stand")
        if isinstance(act, dict):
            act_ts.append(recv_ns); act_pos.append([act["x"], act["y"], act["z"]])
        cs = L.get("cartesian_solve")
        if not isinstance(cs, dict):
            continue
        for key, b in (("ik_duration_us", ikd), ("ik_solution_jump_deg", jump),
                       ("ik_iterations", iters), ("position_error_m", perr),
                       ("orientation_error_rad", oerr), ("goal_minus_measured_pos_m", gmp)):
            v = cs.get(key)
            if isinstance(v, (int, float)) and math.isfinite(v):
                b.append(float(v))
        warn += 1 if cs.get("ik_warn_duration_exceeded") is True else 0
        fail += 1 if cs.get("ik_fail_duration_exceeded") is True else 0
        timed += 1 if cs.get("ik_timed_out") is True else 0
        branch_susp += 1 if cs.get("ik_branch_jump_suspected") is True else 0
        branch_clamp += 1 if cs.get("ik_branch_jump_clamped") is True else 0

    def stat(a, sc=1.0, nd=3):
        if not a:
            return None
        return {"mean": round(float(np.mean(a)) * sc, nd), "p95": round(float(np.percentile(a, 95)) * sc, nd),
                "max": round(float(np.max(a)) * sc, nd)}
    sm = base.smoothness_metrics(np.asarray(ref_ts, dtype=np.int64), np.asarray(ref_pos, dtype=float)) if len(ref_ts) > 8 else {"samples": len(ref_ts)}
    sm_act = base.smoothness_metrics(np.asarray(act_ts, dtype=np.int64), np.asarray(act_pos, dtype=float)) if len(act_ts) > 8 else {"samples": len(act_ts)}
    ref_span_mm = round(float(np.max(np.ptp(np.asarray(ref_pos), axis=0))) * 1e3, 2) if len(ref_pos) > 1 else 0.0
    act_span_mm = round(float(np.max(np.ptp(np.asarray(act_pos), axis=0))) * 1e3, 2) if len(act_pos) > 1 else 0.0
    return {
        "ref_pos_span_mm": ref_span_mm,
        "actual_pos_span_mm": act_span_mm,
        "actual_trajectory_smoothness": sm_act,
        "state_samples": n,
        "ik_duration_us": stat(ikd, nd=1),
        "ik_over_2ms_budget_count": int(np.sum(np.asarray(ikd) > BUDGET_US)) if ikd else 0,
        "ik_warn_gt1ms_count": warn,
        "ik_fail_gt5ms_count": fail,
        "ik_timed_out_count": timed,
        "ik_iterations_max": int(max(iters)) if iters else None,
        "ik_solution_jump_deg_max": round(max(jump), 2) if jump else None,
        "ik_branch_jump_suspected_count": branch_susp,
        "ik_branch_jump_clamped_count": branch_clamp,
        "ref_vs_target_pos_err_mm": stat(perr, sc=1e3),
        "ref_vs_target_ori_err_deg": stat([math.degrees(v) for v in oerr]),
        "goal_minus_measured_pos_m_max": round(max(gmp), 4) if gmp else None,
        "send_period_overrun_count": overrun,
        "tracking_error_degraded_count": degraded,
        "fault_latched": fault,
        "ref_trajectory_smoothness": sm,
    }


def main() -> int:
    ts = time.strftime("%Y%m%dT%H%M%S")
    art = ROOT / "artifacts" / "tcp_target_pose_eval" / f"500hz_{ts}"
    art.mkdir(parents=True, exist_ok=True)
    rec = base.StampedStateRecorder("127.0.0.1", 50356, art / "state.jsonl")
    rec.start()
    sclient = RobotStateClient(bind=STATE_LEASE)
    sclient.start()
    client = ServoCommandClient(CMD, timeout_sec=0.3, source_id="tcp_target_eval")
    results = {"artifact_dir": str(art), "note": "REAL Rainbow controllers in pgmode simulation (no physical motion); init via JointTarget then stream TcpPoseTarget; profiles policy(1-5mm)/large(>=100mm)/complex", "regimes": {}}
    try:
        time.sleep(0.5)
        lease = client.acquire_lease(StateStreamLeaseReadback(sclient), timeout_sec=4.0)
        print(f"lease_token={client.lease_token}")
        if not client.lease_token:
            print("WARN: no lease granted (viser may hold it). Aborting.")
            return 2
        # Arm
        for _ in range(30):
            client.send(CommandIntent.arm_motion(timeout_sec=0.3))
            time.sleep(0.1)
            if rec.snapshots and rec.snapshots[-1].get("motion_state") in {"ArmedHold", "Running"}:
                break
        ms = wait_motion_state(rec, {"ArmedHold", "Running"}, 5.0)
        print(f"motion_state={ms}")
        # Drive to a well-conditioned init posture via JointTarget FIRST.
        reached, q_now, q_err = init_posture(client, rec, timeout_sec=20.0)
        print(f"init_posture reached={reached} q_err_deg={None if q_err is None else round(q_err,2)} q_actual={[round(x,1) for x in q_now] if q_now else None}")
        results["init_posture"] = {"reached": reached, "q_err_deg": q_err}
        time.sleep(0.5)
        rng = np.random.default_rng(0)
        for regime in ("large", "complex", "policy"):
            # keep armed
            client.send(CommandIntent.arm_motion(timeout_sec=0.3)); time.sleep(0.2)
            snap, L = latest_left(rec)
            p7 = left_pose7(L); p0 = p7[:3]; q0 = p7[3:7]
            targets, cfg = gen_targets(regime, p0, q0, rng)
            t0 = time.monotonic_ns()
            stream(client, targets, cfg)
            time.sleep(0.5)
            t1 = time.monotonic_ns()
            results["regimes"][regime] = {"n_targets": len(targets), **metrics(rec, t0, t1)}
            print(f"[{regime}] done")
    finally:
        try:
            client.release_lease()
        except Exception:
            pass
        client.close()
        sclient.stop()
        rec.stop()
    (art / "eval_500hz_summary.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
