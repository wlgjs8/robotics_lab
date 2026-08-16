#!/usr/bin/env python3
"""Extract the absolute stand-frame command knots from a rollout-step JSONL.

One `x y z qx qy qz qw` line per committed policy step, for rb_servo_server's
tools/follower_replay so an offline sweep sees the exact reference the follower
saw on hardware.
"""
import json, sys
arm = sys.argv[2] if len(sys.argv) > 2 else "right"
n = 0
for line in open(sys.argv[1]):
    line = line.strip()
    if not line:
        continue
    rec = json.loads(line)
    p = (rec.get("arms", {}).get(arm) or {}).get("cmd_pose")
    if not p or len(p) < 7:
        continue
    print(" ".join(f"{v:.9f}" for v in p[:7]))
    n += 1
print(f"# {n} knots from {sys.argv[1]} ({arm})", file=sys.stderr)
