#!/usr/bin/env python3
"""Capture TCP pose from the rb_servo_server state fanout and plot x,y,z vs time.

Subscribes to a spare state-fanout UDP port (default 50356; server publishes to
50356/50366/50376) and records left/right tcp_actual_stand. Run it DURING a
rollout (robot moving) for `--duration` seconds, then it writes a PNG + CSV.

  PYTHONPATH=policy_runner ~/openpi/.venv/bin/python scripts/capture_tcp_trajectory.py \
      --port 50356 --duration 20 --arm right --out /tmp/tcp_traj.png
"""
import argparse, json, socket, time
import numpy as np


def arm_xyz(payload, side):
    arm = payload.get(side, {})
    for key in ("tcp_actual_stand", "tcp_stand"):
        v = arm.get(key)
        if v is None:
            continue
        if isinstance(v, dict):  # {position:{x,y,z},...}
            p = v.get("position", v)
            return [float(p["x"]), float(p["y"]), float(p["z"])]
        if isinstance(v, (list, tuple)) and len(v) >= 3:
            return [float(v[0]), float(v[1]), float(v[2])]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=50356)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--arm", choices=["right", "left", "both"], default="right")
    ap.add_argument("--out", default="/tmp/tcp_traj.png")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(1.0)
    print(f"[cap] listening udp {args.bind}:{args.port} for {args.duration}s "
          f"(run a rollout now)...", flush=True)

    sides = ["right", "left"] if args.arm == "both" else [args.arm]
    rows = {s: [] for s in sides}   # (t, x, y, z)
    t0 = None
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        try:
            payload = json.loads(data.decode("utf-8", "ignore"))
        except Exception:
            continue
        now = time.monotonic()
        if t0 is None:
            t0 = now
        for s in sides:
            xyz = arm_xyz(payload, s)
            if xyz is not None:
                rows[s].append([now - t0, *xyz])
    sock.close()

    total = sum(len(v) for v in rows.values())
    if total == 0:
        print("[cap] NO state packets with tcp pose received — wrong port, or robot "
              "not publishing / not moving. Server fans out to 50356/50366/50376; "
              "pick a FREE one (GUI uses 50366, flow-infer often 50376).")
        return 1

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(sides), 1, figsize=(11, 3.2 * len(sides)), squeeze=False)
    for ax, s in zip(axes[:, 0], sides):
        a = np.asarray(rows[s])
        if a.size == 0:
            ax.set_title(f"{s}: no data"); continue
        t = a[:, 0]
        for j, lab in enumerate("xyz"):
            col = a[:, 1 + j]
            ax.plot(t, (col - col[0]) * 1000.0, label=f"{lab}  (mm, rel start)")
        hz = len(t) / max(1e-6, t[-1] - t[0])
        # high-freq jitter metric: std of 1st-difference per axis (mm)
        jit = np.std(np.diff(a[:, 1:4], axis=0) * 1000.0, axis=0)
        ax.set_title(f"{s} TCP  |  {len(t)} samples @ {hz:.0f} Hz  |  "
                     f"step-jitter mm x/y/z = {jit[0]:.2f}/{jit[1]:.2f}/{jit[2]:.2f}")
        ax.set_xlabel("time (s)"); ax.set_ylabel("Δ pos (mm)")
        ax.grid(True, alpha=0.3); ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"[cap] wrote {args.out}  ({total} samples)")
    csv = args.out.rsplit(".", 1)[0] + ".csv"
    with open(csv, "w") as f:
        f.write("arm,t,x,y,z\n")
        for s in sides:
            for r in rows[s]:
                f.write(f"{s},{r[0]:.4f},{r[1]:.6f},{r[2]:.6f},{r[3]:.6f}\n")
    print(f"[cap] wrote {csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
