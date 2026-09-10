#!/usr/bin/env python3
"""Live view of the self-collision guard's near pairs while `make run` is up.

Park the arm by hand (compliant Hold / hand-guide, or just push it while the stack
is holding), and read WHICH pair the guard is looking at, how far apart the model
says the two witness points are, and WHERE those points are in the stand frame --
so a tape measure can be put on the same two spots. Nothing is commanded; this only
listens to the state the server already publishes.

The server must fan its state out to this listener's port: stack_real.yaml
`network.state_pub_endpoints` carries `udp://127.0.0.1:50390` for it (added
2026-09-07). Run:

    python3 rb_servo_server/tools/near_pairs_watch.py            # 2 Hz, top 8 pairs
    python3 rb_servo_server/tools/near_pairs_watch.py --grep riser --top 3
    python3 rb_servo_server/tools/near_pairs_watch.py --once     # one snapshot, then exit

Every snapshot ends with a ready-to-paste line for the offline probe
(build/rbpodo_real_gate/collision_pose_probe --config ... --left ... --right ...),
which re-evaluates the same model at that pose with the full pair list.
"""
from __future__ import annotations

import argparse
import json
import re
import socket
import sys
import time
from typing import Any


def _short(name: Any) -> str:
    if not isinstance(name, str):
        return "?"
    return re.sub(r"^dual_[a-z0-9_]+?_(?=(?:left|right)_)", "", name)


def _mm(v: Any) -> str:
    try:
        return f"{float(v) * 1e3:7.1f}"
    except (TypeError, ValueError):
        return "      ?"


def _xyz_mm(v: Any) -> str:
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        return "(?, ?, ?)"
    return "(" + ", ".join(f"{float(x) * 1e3:.1f}" for x in v) + ")"


def _cls(p: dict[str, Any]) -> str:
    for key in ("external_box", "external", "environment", "gripper_gripper", "intra_arm",
                "arm_stand"):
        if p.get(key):
            return key
    return "self"


def _find_key(node: Any, key: str) -> Any:
    """Depth-first search for `key` anywhere in the message (the verdict has moved
    between top level and sub-objects before; do not couple to one layout)."""
    if isinstance(node, dict):
        if key in node:
            return node[key]
        for v in node.values():
            found = _find_key(v, key)
            if found is not None:
                return found
    elif isinstance(node, list):
        for v in node:
            found = _find_key(v, key)
            if found is not None:
                return found
    return None


def render(msg: dict[str, Any], top: int, grep: str) -> str:
    sc = msg.get("self_collision")
    lines: list[str] = []
    verdict = _find_key(msg, "safety_verdict")
    if not isinstance(sc, dict):
        return "state has no self_collision block (guard disabled?)"
    lines.append(
        f"verdict={verdict}  guard enabled={sc.get('enabled')} checked={sc.get('checked')} "
        f"violated={sc.get('violated')}  min={_mm(sc.get('min_clearance_m')).strip()} mm"
    )
    pairs = sc.get("near_pairs")
    if not isinstance(pairs, list) or not pairs:
        lines.append("near_pairs: none published (server too old, or no pair inside the near set)")
    else:
        rows = [p for p in pairs if isinstance(p, dict) and isinstance(p.get("clearance_m"), (int, float))]
        rows.sort(key=lambda p: float(p["clearance_m"]))
        shown = 0
        for p in rows:
            if grep and grep not in str(p.get("name_a", "")) and grep not in str(p.get("name_b", "")):
                continue
            d = float(p["clearance_m"])
            hard = p.get("d_hard_m")
            slow = p.get("d_slow_m")
            rate = p.get("rate_m_s")
            band = "HARD" if isinstance(hard, (int, float)) and d < hard else (
                "slow" if isinstance(slow, (int, float)) and d < slow else "ok")
            closing = ""
            if isinstance(rate, (int, float)):
                closing = f" rate={float(rate) * 1e3:+.1f} mm/s" + (" CLOSING" if float(rate) < -0.001 else "")
            lines.append(
                f"{_mm(d)} mm {band:4s} {_cls(p):15s} {_short(p.get('name_a'))} <-> {_short(p.get('name_b'))}"
                f"  floor={_mm(hard).strip()} band={_mm(slow).strip()}{closing}"
            )
            lines.append(f"          a={_xyz_mm(p.get('p_a_m'))} mm  b={_xyz_mm(p.get('p_b_m'))} mm  (stand frame)")
            shown += 1
            if shown >= top:
                break
        if shown == 0:
            lines.append(f"near_pairs: {len(rows)} published, none match --grep {grep!r}")
    q_l = (msg.get("left") or {}).get("q_actual_deg") if isinstance(msg.get("left"), dict) else None
    q_r = (msg.get("right") or {}).get("q_actual_deg") if isinstance(msg.get("right"), dict) else None
    if isinstance(q_l, list) and isinstance(q_r, list) and len(q_l) == 6 and len(q_r) == 6:
        fmt = lambda q: ",".join(f"{float(v):.3f}" for v in q)  # noqa: E731
        lines.append(
            "probe: rb_servo_server/build/rbpodo_real_gate/collision_pose_probe "
            f"--config rb_servo_server/config/stack_real.yaml --left {fmt(q_l)} --right {fmt(q_r)}"
        )
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="0.0.0.0:50390", help="host:port to listen on (default 0.0.0.0:50390)")
    ap.add_argument("--top", type=int, default=8, help="pairs to show per snapshot")
    ap.add_argument("--grep", default="", help="only pairs whose geometry names contain this substring")
    ap.add_argument("--period", type=float, default=0.5, help="seconds between printed snapshots")
    ap.add_argument("--once", action="store_true", help="print one snapshot and exit")
    ap.add_argument("--json", action="store_true", help="print the raw self_collision block instead")
    args = ap.parse_args()
    host, _, port = args.bind.rpartition(":")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host or "0.0.0.0", int(port)))
    sock.settimeout(1.0)
    last_print = 0.0
    waited = 0
    while True:
        try:
            payload, _ = sock.recvfrom(1 << 20)
        except socket.timeout:
            waited += 1
            if waited % 5 == 0:
                print(f"no state on {args.bind} for {waited} s -- is `make run` up and is this port in "
                      f"network.state_pub_endpoints?", file=sys.stderr)
            continue
        try:
            msg = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if not isinstance(msg, dict):
            continue
        now = time.monotonic()
        if now - last_print < args.period and not args.once:
            continue
        last_print = now
        if args.json:
            print(json.dumps(msg.get("self_collision"), indent=1))
        else:
            print(time.strftime("%H:%M:%S") + "  " + render(msg, args.top, args.grep))
            print("-" * 100)
        sys.stdout.flush()
        if args.once:
            return 0


if __name__ == "__main__":
    raise SystemExit(main())
