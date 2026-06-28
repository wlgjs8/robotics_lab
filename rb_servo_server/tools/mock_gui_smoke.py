#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GUI_SRC = ROOT.parent / "rb_gui"
sys.path.insert(0, str(GUI_SRC))

from rb_servo_gui.command_client import CommandClient  # noqa: E402
from rb_servo_gui.safety import OperatorSafety, Readiness  # noqa: E402
from rb_servo_gui.state_receiver import StateReceiver, StateStore  # noqa: E402


def wait_for_snapshot(store: StateStore, timeout_sec: float = 2.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        latest = store.latest()
        if latest is not None and not store.is_stale():
            return latest
        time.sleep(0.05)
    raise RuntimeError("state snapshot was not received within timeout")


def main() -> int:
    parser = argparse.ArgumentParser(description="GUI contract smoke for an explicit rb_servo_server config")
    parser.add_argument("--server", default=str(ROOT / "build" / "rb_servo_server"))
    parser.add_argument("--config", required=True)
    parser.add_argument("--keep-server", action="store_true", help="Do not launch/stop a server process")
    args = parser.parse_args()

    proc: subprocess.Popen[str] | None = None
    store = StateStore(stale_after_sec=0.5)
    receiver = StateReceiver(store, host="127.0.0.1", port=50110)
    receiver.start()
    try:
        if not args.keep_server:
            proc = subprocess.Popen(
                [args.server, "--config", args.config],
                cwd=str(ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            time.sleep(0.3)
            if proc.poll() is not None:
                output = proc.stdout.read() if proc.stdout else ""
                raise RuntimeError(f"server exited early with {proc.returncode}:\n{output}")

        first = wait_for_snapshot(store, timeout_sec=2.0)
        safety = OperatorSafety(
            store,
            CommandClient("127.0.0.1", 50010),
            desired_mode="mock",
            observed_server_mode="mock",
            sim_readiness=Readiness(no_go_reason="sim readiness not proven"),
        )
        ok, message = safety.send_lifecycle("ArmMotion")
        if not ok:
            raise RuntimeError(f"ArmMotion blocked unexpectedly: {message}")
        time.sleep(0.25)
        armed = wait_for_snapshot(store, timeout_sec=2.0)
        ok, message = safety.jog_joint("left", 0, 1.0)
        if not ok:
            raise RuntimeError(f"joint jog blocked unexpectedly: {message}")
        time.sleep(0.25)
        jogged = wait_for_snapshot(store, timeout_sec=2.0)
        ok, tcp_message = safety.tcp_jog_unavailable()
        if ok:
            raise RuntimeError("TCP jog unexpectedly succeeded")
        if len(safety.command_client.sent_packets) != 2:
            raise RuntimeError(f"expected exactly ArmMotion + JointTarget sends, got {len(safety.command_client.sent_packets)}")

        result = {
            "first_tick": first.tick,
            "armed_tick": armed.tick,
            "jogged_tick": jogged.tick,
            "packets_received": store.received_packets,
            "left_q_sent_after_jog": jogged.left.q_sent_deg,
            "tcp_jog": "blocked",
            "tcp_reason": tcp_message,
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    finally:
        receiver.stop()
        if proc is not None:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
