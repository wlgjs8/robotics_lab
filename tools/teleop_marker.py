#!/usr/bin/env python3
"""Operator event marker for teleop debugging.

Run this in a SEPARATE terminal on the SAME PC as rb_servo_server / policy_runner
(so its CLOCK_MONOTONIC matches the servo loop's loop_start_time_ns and the
policy logs' mono_ns). The instant the robot misbehaves, type a one-letter tag
and press Enter — the marker is timestamped (monotonic + wallclock) and the
offline analyzer (logs/analyze_capture.py) will dump the full servo/teleop
context around each marker.

Tags (free text; these are just conventions):
  u  = moved while I was NOT operating (untouched)
  d  = went in a direction I did NOT command
  s  = speed was wrong (too fast / too slow / lurch / stall)
  <anything else> = free-form note

Alignment: marker mono_ns == int(time.monotonic()*1e9) == servo_log
loop_start_time_ns (both are Linux CLOCK_MONOTONIC, same boot epoch), as long as
this runs natively on the same host.
"""
import sys
import time
from datetime import datetime
from pathlib import Path

LOGS = Path(__file__).resolve().parents[1] / "logs"
LOGS.mkdir(parents=True, exist_ok=True)
path = LOGS / f"teleop_marker_{datetime.now().strftime('%Y%m%d_%H%M%S')}_KST.log"

TAGS = {
    "u": "MOVED_WHILE_IDLE",
    "d": "WRONG_DIRECTION",
    "s": "WRONG_SPEED",
}

def main() -> None:
    with open(path, "w", buffering=1, encoding="utf-8") as fh:
        fh.write(f"# teleop operator markers  started={datetime.now().isoformat()}\n")
        fh.write("# fields: mono_ns=<CLOCK_MONOTONIC ns, aligns w/ servo loop_start_time_ns>  wall=<iso>  tag  note\n")
        print(f"[marker] logging to {path}")
        print("[marker] type a tag (u=idle-move, d=wrong-dir, s=wrong-speed) + Enter the moment it happens. Ctrl-D to stop.")
        n = 0
        try:
            for line in sys.stdin:
                mono_ns = int(time.monotonic() * 1e9)
                wall = datetime.now().isoformat()
                text = line.strip()
                key = text.split()[0].lower() if text else ""
                tag = TAGS.get(key, "NOTE")
                fh.write(f"mono_ns={mono_ns}  wall={wall}  tag={tag}  note={text!r}\n")
                n += 1
                print(f"[marker] #{n} {tag} @ mono_ns={mono_ns} ({wall})")
        except KeyboardInterrupt:
            pass
        print(f"[marker] {n} markers written to {path}")

if __name__ == "__main__":
    main()
