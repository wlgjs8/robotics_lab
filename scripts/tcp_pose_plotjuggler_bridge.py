#!/usr/bin/env python3
"""Bridge rb_servo_server state fanout -> flat TCP pose JSON for PlotJuggler.

Subscribes to a state-fanout UDP port (default 50356; the server publishes to
50356/50366/50376), converts each arm's tcp_actual_stand
(x,y,z,qx,qy,qz,qw) into x,y,z + rx,ry,rz (rotation VECTOR, matching the repo's
6-DOF pose convention, e.g. waypoints.json *_pose), and re-streams a flat JSON
object to PlotJuggler's UDP Server (default 127.0.0.1:9870).

Stdlib only -> runs under plain `python3`. Start it AFTER the server is up.
"""
import argparse, json, math, socket, time


def quat_to_rotvec(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    if qw < 0.0:                      # shortest-arc
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    s = math.sqrt(qx * qx + qy * qy + qz * qz)
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    ang = 2.0 * math.atan2(s, qw)
    f = ang / s
    return (qx * f, qy * f, qz * f)


def pose_field(arm, field):
    """Extract {x,y,z,rx,ry,rz} from arm[field] (server publishes a dict with
    x,y,z, rx,ry,rz rotvec, and qx,qy,qz,qw). field e.g. 'tcp_command_stand'
    (the 500 Hz servo command) or 'tcp_actual_stand' (measured)."""
    v = arm.get(field)
    if isinstance(v, dict) and all(c in v for c in ("x", "y", "z")):
        if all(c in v for c in ("rx", "ry", "rz")):
            return {c: float(v[c]) for c in ("x", "y", "z", "rx", "ry", "rz")}
        rx, ry, rz = quat_to_rotvec(float(v["qx"]), float(v["qy"]), float(v["qz"]), float(v["qw"]))
        return {"x": float(v["x"]), "y": float(v["y"]), "z": float(v["z"]),
                "rx": rx, "ry": ry, "rz": rz}
    if isinstance(v, (list, tuple)) and len(v) >= 7:  # fallback: list
        rx, ry, rz = quat_to_rotvec(float(v[3]), float(v[4]), float(v[5]), float(v[6]))
        return {"x": float(v[0]), "y": float(v[1]), "z": float(v[2]),
                "rx": rx, "ry": ry, "rz": rz}
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-port", type=int, default=50356)
    ap.add_argument("--out-host", default="127.0.0.1")
    ap.add_argument("--out-port", type=int, default=9870)
    a = ap.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", a.in_port))
    rx.settimeout(1.0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[bridge] state udp:{a.in_port} -> PlotJuggler udp:{a.out_host}:{a.out_port} "
          f"(fields: right/x,y,z,rx,ry,rz and left/...)", flush=True)

    n = 0
    t0 = time.monotonic()
    while True:
        try:
            data, _ = rx.recvfrom(65535)
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            break
        try:
            p = json.loads(data.decode("utf-8", "ignore"))
        except Exception:
            continue
        out = {"t": time.monotonic() - t0}
        for side in ("right", "left"):
            arm = p.get(side, {})
            if not isinstance(arm, dict):
                continue
            cmd = pose_field(arm, "tcp_command_stand")        # 500 Hz servo COMMAND
            act = pose_field(arm, "tcp_actual_stand") or pose_field(arm, "tcp_stand")  # measured
            if cmd is not None:
                out[f"{side}_cmd"] = cmd
            if act is not None:
                out[f"{side}_act"] = act
        if len(out) > 1:
            tx.sendto(json.dumps(out).encode(), (a.out_host, a.out_port))
            n += 1
            if n == 1 or n % 500 == 0:
                print(f"[bridge] forwarded {n} packets ({[k for k in out if k!='t']})", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
