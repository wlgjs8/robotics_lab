#!/usr/bin/env python3
"""Replay a dumped per-step policy action sequence as TcpTwist commands.

Counterpart of scripts/dump_policy_actions.py: takes the action-dump JSON
(produced on the GPU host, teacher-forced validation-episode predictions) and
streams it to rb_servo_server through the standard policy_runner runtime
(SafetyGate + lease + arm-motion handshake). No torch required.

The per-step 6D deltas are converted to twists (delta * speed / dt) and sent as
TcpTwistLocal (action_frame=ee_local) or TcpTwistStand (stand). Gripper dims
are ignored by default (arm-motion visualization only). Intended for the rbpodo
controller-simulation (pgmode) carve-out; the injected action source carries the
same requirements as the rbpodo controller-sim Cartesian sources, so the
existing SafetyGate semantics are unchanged.

Example:
  python3 scripts/replay_policy_actions.py \
    --dump artifacts/replay_rollout/_action_dump_eelocal.json \
    --episode 0 --speed 0.5 \
    --config policy_runner/config/rbpodo_pgmode_umi_500hz_ack.yaml
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

REPO_PR = Path(__file__).resolve().parents[1] / "policy_runner"
if REPO_PR.exists() and str(REPO_PR) not in sys.path:
    sys.path.insert(0, str(REPO_PR))

from policy_runner.action_sources.tcp_delta import (  # noqa: E402
    cartesian_action_requirements,
    clamp_tcp_twist,
    tcp_twist_local_intent,
    tcp_twist_stand_intent,
)
from policy_runner.config import load_config  # noqa: E402
from policy_runner.main import LEASE_READBACK_TIMEOUT_EXIT_CODE, run  # noqa: E402


class ReplayTwistActionSource:
    """Streams a fixed twist schedule; time-indexed so the command_rate_hz loop
    re-sends the current step's twist until its dt window elapses (constant
    velocity over the window reproduces the per-step delta)."""

    def __init__(
        self,
        steps: list[tuple[tuple[float, ...] | None, tuple[float, ...] | None]],
        step_dt_sec: float,
        intent_builder,
        timeout_sec: float,
        requirements,
        done_grace_sec: float = 1.0,
    ):
        self.requirements = requirements
        self._steps = steps
        self._dt = step_dt_sec
        self._build = intent_builder
        self._timeout = timeout_sec
        self._grace = done_grace_sec
        self._t0: float | None = None
        self._last_logged = -1

    def next_intent(self, snapshot, now_monotonic):
        _ = snapshot
        if self._t0 is None:
            self._t0 = now_monotonic
        idx = int((now_monotonic - self._t0) / self._dt)
        if idx >= len(self._steps):
            if now_monotonic - self._t0 > len(self._steps) * self._dt + self._grace:
                print("replay complete", flush=True)
                raise KeyboardInterrupt  # run() treats this as a clean exit
            return None
        if idx // 50 != self._last_logged:
            self._last_logged = idx // 50
            print(f"replay step {idx}/{len(self._steps)}", flush=True)
        left, right = self._steps[idx]
        return self._build(left=left, right=right, timeout_sec=self._timeout)


def wait_for_lease_free(bind_endpoint: str, timeout_sec: float) -> None:
    """Block until the server reports no active command-source lease.

    A previous client's lease is held for lease_timeout_sec after it exits;
    AcquireLease sent while it is still active is rejected and run() does not
    retry, so starting early just burns the readback timeout."""
    host, port = bind_endpoint.removeprefix("udp://").rsplit(":", 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, int(port)))
    sock.settimeout(1.0)
    deadline = time.monotonic() + timeout_sec
    notified = False
    try:
        while time.monotonic() < deadline:
            try:
                data, _ = sock.recvfrom(65536)
            except socket.timeout:
                continue
            lease = json.loads(data).get("command_source", {})
            if lease.get("active_source_id") is None or lease.get("expired"):
                return
            if not notified:
                notified = True
                print(
                    f"waiting for stale lease to expire "
                    f"(holder={lease.get('active_source_id')}, "
                    f"lease_timeout={lease.get('lease_timeout_sec')}s)",
                    flush=True,
                )
            time.sleep(0.5)
        raise SystemExit("timed out waiting for the command-source lease to free up")
    finally:
        sock.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", required=True, help="action dump JSON from dump_policy_actions.py")
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--config", required=True, help="policy_runner YAML config (pgmode controller-sim)")
    ap.add_argument("--speed", type=float, default=0.5,
                    help="time-scale factor (<1 = slower, same path geometry)")
    ap.add_argument("--max-steps", type=int, default=0, help="0 = all steps")
    ap.add_argument("--max-linear-m-s", type=float, default=0.2)
    ap.add_argument("--max-angular-rad-s", type=float, default=0.4)
    ap.add_argument("--use-gt-actions", action="store_true",
                    help="replay the recorded ground-truth actions instead of the model predictions")
    args = ap.parse_args()

    if not 0.0 < args.speed <= 1.0:
        raise SystemExit("--speed must be in (0, 1]")

    dump = json.loads(Path(args.dump).read_text())
    if dump.get("schema") != "robotics_lab.policy_runner.action_dump.v1":
        raise SystemExit(f"unsupported dump schema: {dump.get('schema')}")
    episode = dump["episodes"][args.episode]
    frame = str(dump["action_frame"])
    dt = float(dump["dt_mean_sec"])
    builder = {"ee_local": tcp_twist_local_intent, "stand": tcp_twist_stand_intent}.get(frame)
    if builder is None:
        raise SystemExit(f"unsupported action_frame: {frame}")

    arm_mask = episode["arm_mask"]
    actions = episode["gt_actions"] if args.use_gt_actions else episode["actions"]
    if args.max_steps > 0:
        actions = actions[: args.max_steps]
    step_dt = dt / args.speed
    steps = []
    clipped = 0
    for raw in actions:
        def to_twist(d6):
            twist = tuple(v * args.speed / dt for v in d6)
            clamped = clamp_tcp_twist(twist, args.max_linear_m_s, args.max_angular_rad_s)
            return clamped, clamped != twist
        left = right = None
        if arm_mask[0] > 0:
            left, c = to_twist(raw[0:6])
            clipped += c
        if arm_mask[1] > 0:
            right, c = to_twist(raw[7:13])
            clipped += c
        steps.append((left, right))

    config = load_config(args.config)
    print(
        f"replaying {episode['name']} ({len(steps)} steps, "
        f"{'GT' if args.use_gt_actions else 'model'} actions, frame={frame}, "
        f"dt={dt:.4f}s, speed={args.speed}x -> step_dt={step_dt:.4f}s, "
        f"duration~{len(steps) * step_dt:.1f}s, client-clipped arm-steps={clipped}); "
        f"gripper dims ignored",
        flush=True,
    )
    requirements = cartesian_action_requirements(
        allow_rbpodo_controller_simulation=bool(
            getattr(config.safety, "allow_rbpodo_controller_simulation_cartesian", False)
        ),
    )
    source = ReplayTwistActionSource(
        steps, step_dt, builder, timeout_sec=config.servo_command.timeout_sec,
        requirements=requirements,
    )
    # AcquireLease is a single fire-and-forget UDP packet inside run(); a stale
    # lease or a lost packet makes the readback time out without retry, so wait
    # for the lease to free up and retry the whole run (no motion command is
    # sent before the lease is granted, so retrying is safe).
    for attempt in range(3):
        if config.servo_command.acquire_lease:
            wait_for_lease_free(config.robot_state.bind, timeout_sec=90.0)
        rc = run(config, source=source)
        if rc != LEASE_READBACK_TIMEOUT_EXIT_CODE:
            return rc
        print(f"lease acquisition failed (attempt {attempt + 1}/3); retrying", flush=True)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
