#!/usr/bin/env python3
"""How long does the jaw take to close, from a CLEAN step command?

The rollout logs answer this only on the 34 ms policy grid, and only for the ramped
commands the policy actually emits (measured 2026-09-17: the model's own close ramp is
1.5-2x slower than the jaw, so the logs measure the ramp, not the actuator). This drives a
step directly on the serial port and samples the jaw off the SDK's own reader-thread
callback, so the numbers are the actuator's.

Reports per step, per arm:
    dead_ms    write returned -> first jaw sample that moved >= --move-eps mm
    t90_ms     write -> 90% of the travel actually achieved
    settle_ms  write -> last sample outside +-0.5 mm of the final value
    v_max      fastest mm/s between consecutive DISTINCT telemetry samples

Telemetry rate is measured, not assumed (`probe`), because every time here is quantised by
it. Zeroing is NOT touched: `set_zero` would redefine the closed stop and shift the whole
session's mm scale (see tools/measure_gripper_units.py). Only the gripper moves.

    tools/measure_gripper_step_response.py probe
    tools/measure_gripper_step_response.py step --move
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import sys
import threading
import time

SDK_PATH = "/home/plaif/workspace/pika_sdk"
PORTS = {"left": "/dev/pika-left", "right": "/dev/pika-right"}
RL_PATH = "/home/plaif/workspace/robotics_lab/policy_runner"

if RL_PATH not in sys.path:
    sys.path.insert(0, RL_PATH)
from policy_runner.gripper import sdk_jaw_mm, sdk_jaw_mm_to_rad  # noqa: E402

MAX_RAD = 1.66  # measured open stop (left 1.6741 / right 1.6687); stay below it


def connect(arms):
    if SDK_PATH not in sys.path:
        sys.path.insert(0, SDK_PATH)
    from pika.gripper import Gripper

    out = {}
    for arm in arms:
        g = Gripper(port=PORTS[arm])
        if not g.connect():
            raise SystemExit(f"{arm}: connect failed on {PORTS[arm]}")
        if not g.enable():
            raise SystemExit(f"{arm}: enable failed")
        out[arm] = g
    time.sleep(0.6)
    return out


class Sampler:
    """Stamp every 'motor' telemetry frame as it ARRIVES on the SDK reader thread.

    Polling get_motor_position() cannot do this: it is a dict read, so a poll faster than
    the device's push rate just re-reads the same sample and reports a fake early time.
    """

    def __init__(self, gripper):
        self._g = gripper
        self.samples: list[tuple[float, float]] = []  # (t_mono, rad)
        self._on = False
        self._lock = threading.Lock()
        comm = getattr(gripper, "serial_comm", None)
        inner = getattr(comm, "callback", None)
        if comm is None or not callable(inner):
            raise SystemExit("SDK shape unrecognised: cannot stamp telemetry arrival")

        def _stamped(data, _inner=inner):
            result = _inner(data)
            try:
                if isinstance(data, dict) and "motor" in data:
                    with self._lock:
                        if self._on:
                            self.samples.append((time.monotonic(), float(self._g.get_motor_position())))
            except Exception:
                pass
            return result

        comm.callback = _stamped

    def start(self):
        with self._lock:
            self.samples = []
            self._on = True

    def stop(self):
        with self._lock:
            self._on = False
            return list(self.samples)


def distinct(samples, eps=1e-6):
    """Drop repeated positions: the device pushes frames faster than the encoder updates."""
    out = []
    for t, rad in samples:
        if not out or abs(rad - out[-1][1]) > eps:
            out.append((t, rad))
    return out


def settle(gripper, target_rad, timeout=2.0, eps=0.004, quiet=0.25):
    """Wait until the jaw stops moving (reached or blocked). Returns the resting rad."""
    gripper.set_motor_angle(target_rad)
    t0 = time.monotonic()
    last, last_change = None, t0
    while time.monotonic() - t0 < timeout:
        time.sleep(0.01)
        pos = float(gripper.get_motor_position())
        if last is None or abs(pos - last) > eps:
            last, last_change = pos, time.monotonic()
        elif time.monotonic() - last_change > quiet:
            return pos
    return float(gripper.get_motor_position())


def run_step(gripper, sampler, from_mm, to_mm, record_sec, settle_sec):
    rest = settle(gripper, sdk_jaw_mm_to_rad(min(from_mm, sdk_jaw_mm(MAX_RAD))))
    time.sleep(settle_sec)
    sampler.start()
    time.sleep(0.08)  # a few pre-step samples, so "first movement" has a baseline
    target_rad = sdk_jaw_mm_to_rad(to_mm)
    t_write = time.monotonic()
    ok = bool(gripper.set_motor_angle(target_rad))
    t_written = time.monotonic()
    time.sleep(record_sec)
    raw = sampler.stop()
    return {
        "ok": ok,
        "from_mm": sdk_jaw_mm(rest),
        "to_mm": to_mm,
        "t_write": t_write,
        "t_written": t_written,
        "samples": [(t, sdk_jaw_mm(r)) for t, r in raw],
    }


def analyse(ev, move_eps):
    s = ev["samples"]
    if len(s) < 5:
        return None
    t0 = ev["t_write"]
    pre = [mm for t, mm in s if t < t0]
    start = statistics.median(pre) if pre else s[0][1]
    post = [(t - t0, mm) for t, mm in s if t >= t0]
    if not post:
        return None
    final = statistics.median([mm for t, mm in post[-5:]])
    travel = start - final
    out = {
        "start_mm": start,
        "final_mm": final,
        "travel_mm": travel,
        "cmd_mm": ev["to_mm"],
        "write_ms": (ev["t_written"] - t0) * 1000.0,
        "n_samples": len(post),
    }
    if abs(travel) < move_eps:
        out["dead_ms"] = out["t90_ms"] = out["settle_ms"] = out["v_max_mm_s"] = None
        return out
    sgn = math.copysign(1.0, travel)
    dead = t90 = None
    for dt, mm in post:
        if dead is None and sgn * (start - mm) >= move_eps:
            dead = dt * 1000.0
        if t90 is None and sgn * (start - mm) >= 0.9 * abs(travel):
            t90 = dt * 1000.0
    last_out = 0.0
    for dt, mm in post:
        if abs(mm - final) > 0.5:
            last_out = dt
    out["dead_ms"], out["t90_ms"] = dead, t90
    out["settle_ms"] = last_out * 1000.0
    d = distinct([(t, mm) for t, mm in s])
    v = [abs(d[i + 1][1] - d[i][1]) / (d[i + 1][0] - d[i][0])
         for i in range(len(d) - 1) if d[i + 1][0] > d[i][0]]
    out["v_max_mm_s"] = max(v) if v else None
    return out


def cmd_probe(args):
    gs = connect(args.arms)
    report = {}
    try:
        samplers = {a: Sampler(g) for a, g in gs.items()}
        for s in samplers.values():
            s.start()
        time.sleep(args.probe_sec)
        for arm, g in gs.items():
            raw = samplers[arm].stop()
            d = distinct(raw)
            span = (raw[-1][0] - raw[0][0]) if len(raw) > 1 else 0.0
            report[arm] = {
                "motor_position_rad": float(g.get_motor_position()),
                "jaw_mm_sdk": float(g.get_gripper_distance()),
                "jaw_mm_local": sdk_jaw_mm(float(g.get_motor_position())),
                "motor_current_ma": float(g.get_motor_current()),
                "frame_hz": (len(raw) - 1) / span if span > 0 else None,
                "distinct_sample_hz": (len(d) - 1) / span if span > 0 else None,
            }
    finally:
        for g in gs.values():
            try:
                g.disable(); g.disconnect()
            except Exception:
                pass
    print(json.dumps(report, indent=2))
    for arm, r in report.items():
        if abs(r["motor_current_ma"]) > args.max_current_ma:
            print(f"!! {arm}: |current| {r['motor_current_ma']:.0f} mA > {args.max_current_ma:.0f} "
                  f"-- the jaw is pressing on something. Clear it before `step`.", file=sys.stderr)
    return report


def cmd_step(args):
    if not args.move:
        raise SystemExit("`step` physically moves the jaw: pass --move to confirm")
    pairs = [tuple(float(x) for x in p.split(":")) for p in args.steps.split(",")]
    gs = connect(args.arms)
    rows = []
    try:
        samplers = {a: Sampler(g) for a, g in gs.items()}
        for rep in range(args.repeats):
            for (f_mm, t_mm) in pairs:
                for arm, g in gs.items():
                    ev = run_step(g, samplers[arm], f_mm, t_mm, args.record_sec, args.settle_sec)
                    a = analyse(ev, args.move_eps)
                    if a is None:
                        continue
                    a.update(arm=arm, rep=rep, req_from_mm=f_mm, req_to_mm=t_mm)
                    rows.append(a)
                    print(f"[{arm}] {f_mm:5.1f}->{t_mm:5.1f} mm  "
                          f"start {a['start_mm']:5.1f} final {a['final_mm']:5.1f} "
                          f"(travel {a['travel_mm']:5.1f})  "
                          f"dead {a['dead_ms']!s:>6.6} t90 {a['t90_ms']!s:>6.6} "
                          f"settle {a['settle_ms']!s:>6.6} ms  vmax {a['v_max_mm_s']!s:>6.6} mm/s",
                          flush=True)
        if args.restore:
            for arm, g in gs.items():
                settle(g, sdk_jaw_mm_to_rad(args.restore_mm))
    finally:
        for g in gs.values():
            try:
                g.disable(); g.disconnect()
            except Exception:
                pass
    out = pathlib.Path(args.out)
    out.write_text(json.dumps(rows, indent=2))
    print(f"\nwrote {out} ({len(rows)} steps)")
    for arm in args.arms:
        sel = [r for r in rows if r["arm"] == arm and r["dead_ms"] is not None]
        if not sel:
            continue
        print(f"\n=== {arm} (n={len(sel)}) ===")
        for key, lab in (("dead_ms", "dead time"), ("t90_ms", "t90"),
                         ("settle_ms", "settle +-0.5mm"), ("v_max_mm_s", "v_max mm/s")):
            v = sorted(r[key] for r in sel if r[key] is not None)
            if v:
                print(f"   {lab:16s} min {v[0]:7.1f}  p50 {v[len(v)//2]:7.1f}  max {v[-1]:7.1f}")
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=("probe", "step"))
    p.add_argument("--arms", default="left,right")
    p.add_argument("--move", action="store_true", help="required by `step`: confirms physical motion")
    p.add_argument("--steps", default="74:0,40:0,30:7,25:12,0:74",
                   help="comma-separated from_mm:to_mm")
    p.add_argument("--repeats", type=int, default=3)
    p.add_argument("--record-sec", type=float, default=1.5)
    p.add_argument("--settle-sec", type=float, default=0.4)
    p.add_argument("--move-eps", type=float, default=0.5, help="mm that counts as 'moved'")
    p.add_argument("--probe-sec", type=float, default=3.0)
    p.add_argument("--max-current-ma", type=float, default=300.0)
    p.add_argument("--restore", action="store_true", help="reopen to --restore-mm when done")
    p.add_argument("--restore-mm", type=float, default=74.0)
    p.add_argument("--out", default="/tmp/gripstep/step_response.json")
    args = p.parse_args()
    args.arms = [a.strip() for a in args.arms.split(",") if a.strip()]
    (cmd_probe if args.command == "probe" else cmd_step)(args)


if __name__ == "__main__":
    main()
