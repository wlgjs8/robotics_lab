#!/usr/bin/env python3
"""One-shot InitMotion test: move both arms to the demo init pose via the
wrap-to-nearest short path (no replay). Verifies the 360deg-unwind fix on the
real arms. Reuses batch_replay_episodes.return_to_init_pose (which now wraps the
target to the live joint branch)."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for p in (str(REPO_ROOT), str(SCRIPT_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

import batch_replay_episodes as batch  # noqa: E402
import replay_episode_tcp_pose_target as driver  # noqa: E402

INIT_LEFT = "-131.7,73,113.4,-80.9,-107.1,-145.9"
INIT_RIGHT = "135.1,-64,-114.5,84.4,112.5,129.9"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server-config", default="rb_servo_server/config/local/stack_real_replaybind.yaml")
    ap.add_argument("--init-left-joints", default=INIT_LEFT)
    ap.add_argument("--init-right-joints", default=INIT_RIGHT)
    ap.add_argument("--init-timeout-sec", type=float, default=25.0)
    ap.add_argument("--init-tol-deg", type=float, default=1.0)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--i-am-at-the-estop", action="store_true")
    cli = ap.parse_args()

    if not (cli.execute and cli.i_am_at_the_estop):
        print("DRY (no motion): pass --execute --i-am-at-the-estop to move the real arms")
        return 0

    server = driver.load_server_config(Path(cli.server_config))
    target = batch.JointTargets(batch.parse_joint_list(cli.init_left_joints),
                                batch.parse_joint_list(cli.init_right_joints))
    args = SimpleNamespace(
        state_timeout_sec=2.0, source_id="init_move_test",
        init_timeout_sec=cli.init_timeout_sec, init_tol_deg=cli.init_tol_deg,
        init_lease_grace_sec=0.4, allow_controller_sim_arm_error=True, dwell_sec=0.5,
    )
    print(f"InitMotion test -> demo init pose (server={cli.server_config})")
    result = batch.return_to_init_pose(args, server, target)
    print(f"arrived={result.arrived} start_delta_deg={result.start_delta_deg} "
          f"fault={result.fault} timeout={result.timeout}")
    return 0 if result.arrived else 1


if __name__ == "__main__":
    raise SystemExit(main())
