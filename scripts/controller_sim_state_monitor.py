#!/usr/bin/env python3
"""Fail-closed gate and bounded telemetry monitor for rbpodo pgmode simulation."""
from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import time
from collections import Counter
from pathlib import Path
from typing import Any

ARMS = ("left", "right")


def _socket(port: int) -> socket.socket:
    value = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    value.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    value.bind(("0.0.0.0", port))
    value.settimeout(1.0)
    return value


def _message(value: socket.socket) -> dict[str, Any] | None:
    try:
        payload, _ = value.recvfrom(1 << 20)
        decoded = json.loads(payload.decode("utf-8", errors="replace"))
        return decoded if isinstance(decoded, dict) else None
    except (socket.timeout, json.JSONDecodeError):
        return None


def _controller_sim_errors(message: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    for arm in ARMS:
        value = message.get(arm) if isinstance(message.get(arm), dict) else {}
        gate = value.get("cartesian_gate") if isinstance(value.get("cartesian_gate"), dict) else {}
        operation_mode = str(gate.get("operation_mode", "")).strip().lower()
        physical_expected = value.get(
            "physical_motion_expected", gate.get("physical_motion_expected")
        )
        physical_detected = value.get("controller_simulation_physical_motion_detected")
        if operation_mode not in {"simulation", "sim"}:
            errors.append(f"{arm}.operation_mode={operation_mode!r}")
        if physical_expected is not False:
            errors.append(f"{arm}.physical_motion_expected={physical_expected!r}")
        if physical_detected is True:
            errors.append(f"{arm}.controller_simulation_physical_motion_detected=true")
    return errors


def gate(port: int, count: int, timeout_sec: float) -> int:
    receiver = _socket(port)
    deadline = time.monotonic() + timeout_sec
    clean = 0
    errors: list[str] = []
    try:
        while clean < count and time.monotonic() < deadline:
            message = _message(receiver)
            if message is None or "left" not in message:
                continue
            current = _controller_sim_errors(message)
            if current:
                errors.extend(current)
                break
            clean += 1
    finally:
        receiver.close()
    if errors:
        print("[controller-sim-gate] FAIL: " + "; ".join(sorted(set(errors))), flush=True)
        return 2
    if clean < count:
        print(
            f"[controller-sim-gate] FAIL: received {clean}/{count} state packets in {timeout_sec:g}s",
            flush=True,
        )
        return 3
    print(
        f"[controller-sim-gate] PASS: {clean} packets, both arms simulation, "
        "physical_motion_expected=false, physical motion not detected",
        flush=True,
    )
    return 0


def _pose(arm: dict[str, Any], key: str) -> list[float | None]:
    pose = arm.get(key)
    if not isinstance(pose, dict):
        return [None] * 7
    quat = pose.get("quaternion_xyzw")
    if isinstance(quat, list) and len(quat) >= 4:
        qx, qy, qz, qw = quat[:4]
    else:
        qx, qy, qz, qw = (pose.get(name) for name in ("qx", "qy", "qz", "qw"))
    return [pose.get("x"), pose.get("y"), pose.get("z"), qx, qy, qz, qw]


def _finite_xyz(pose: list[float | None]) -> tuple[float, float, float] | None:
    try:
        xyz = tuple(float(value) for value in pose[:3])
    except (TypeError, ValueError):
        return None
    return xyz if all(math.isfinite(value) for value in xyz) else None


def monitor(port: int, duration_sec: float, csv_path: Path, summary_path: Path) -> int:
    receiver = _socket(port)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        "received_monotonic_sec",
        "host_time_ns",
        "tick",
        "command_seq",
        "fault_latched",
        "latched_fault_reason",
        "safety_verdict",
    ]
    for arm in ARMS:
        columns.extend(
            f"{arm}_{name}"
            for name in (
                "physical_motion_expected",
                "physical_motion_detected",
                "servo_state_source",
                "follower_controller",
                "follower_active",
                "follower_actual_lead_m",
                "follower_actual_lead_rad",
                "actual_x",
                "actual_y",
                "actual_z",
                "ref_x",
                "ref_y",
                "ref_z",
                "command_x",
                "command_y",
                "command_z",
            )
        )
    started = time.monotonic()
    packet_count = 0
    invalid_mode_packets = 0
    physical_detected_packets = 0
    unsafe_errors: list[str] = []
    fault_packets = 0
    verdicts: Counter[str] = Counter()
    follower_sources: dict[str, Counter[str]] = {arm: Counter() for arm in ARMS}
    follower_active_packets = {arm: 0 for arm in ARMS}
    max_actual_lead_m = {arm: 0.0 for arm in ARMS}
    max_follower_divergence_m = {arm: 0.0 for arm in ARMS}
    max_follower_divergence_rad = {arm: 0.0 for arm in ARMS}
    max_projection_error_m = {arm: 0.0 for arm in ARMS}
    max_projection_error_rad = {arm: 0.0 for arm in ARMS}
    first_tick: int | None = None
    last_tick: int | None = None
    initial_actual: dict[str, tuple[float, float, float] | None] = {arm: None for arm in ARMS}
    max_actual_displacement_m = {arm: 0.0 for arm in ARMS}
    initial_ref: dict[str, tuple[float, float, float] | None] = {arm: None for arm in ARMS}
    max_ref_displacement_m = {arm: 0.0 for arm in ARMS}
    with csv_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        try:
            while time.monotonic() - started < duration_sec:
                message = _message(receiver)
                if message is None or "left" not in message:
                    continue
                packet_count += 1
                try:
                    tick = int(message.get("tick"))
                    first_tick = tick if first_tick is None else first_tick
                    last_tick = tick
                except (TypeError, ValueError):
                    pass
                errors = _controller_sim_errors(message)
                if errors:
                    invalid_mode_packets += 1
                    unsafe_errors.extend(errors)
                    if any("physical_motion_detected=true" in item for item in errors):
                        physical_detected_packets += 1
                fault = bool(message.get("fault_latched"))
                fault_packets += int(fault)
                verdict = str(message.get("safety_verdict", ""))
                verdicts[verdict] += 1
                row: list[Any] = [
                    time.monotonic(),
                    message.get("host_time_ns"),
                    message.get("tick"),
                    message.get("command_seq"),
                    fault,
                    message.get("latched_fault_reason"),
                    verdict,
                ]
                for arm in ARMS:
                    value = message.get(arm) if isinstance(message.get(arm), dict) else {}
                    solve = value.get("cartesian_solve") if isinstance(value.get("cartesian_solve"), dict) else {}
                    source = str(solve.get("cartesian_servo_state_source", ""))
                    follower_sources[arm][source] += 1
                    follower_active_packets[arm] += int(bool(solve.get("follower_active")))
                    lead = solve.get("follower_actual_lead_m")
                    for source_value, maximum in (
                        (lead, max_actual_lead_m),
                        (solve.get("follower_divergence_pos_m"), max_follower_divergence_m),
                        (solve.get("follower_divergence_ang_rad"), max_follower_divergence_rad),
                        (solve.get("follower_projection_error_m"), max_projection_error_m),
                        (solve.get("follower_projection_error_rad"), max_projection_error_rad),
                    ):
                        try:
                            maximum[arm] = max(maximum[arm], abs(float(source_value)))
                        except (TypeError, ValueError):
                            pass
                    actual = _pose(value, "tcp_actual_stand")
                    reference = _pose(value, "tcp_ref_stand")
                    command = _pose(value, "tcp_command_stand")
                    for pose, initial, maximum in (
                        (actual, initial_actual, max_actual_displacement_m),
                        (reference, initial_ref, max_ref_displacement_m),
                    ):
                        xyz = _finite_xyz(pose)
                        if xyz is not None and initial[arm] is None:
                            initial[arm] = xyz
                        if xyz is not None and initial[arm] is not None:
                            maximum[arm] = max(
                                maximum[arm],
                                math.dist(xyz, initial[arm]),
                            )
                    row.extend(
                        [
                            value.get("physical_motion_expected"),
                            value.get("controller_simulation_physical_motion_detected"),
                            source,
                            solve.get("follower_controller"),
                            solve.get("follower_active"),
                            lead,
                            solve.get("follower_actual_lead_rad"),
                            *actual[:3],
                            *reference[:3],
                            *command[:3],
                        ]
                    )
                writer.writerow(row)
                # This monitor is also the experiment watchdog. A controller
                # leaving pgmode simulation, advertising physical motion, or
                # detecting encoder motion is a stop condition, not merely a
                # statistic to report after the requested duration.
                if errors:
                    handle.flush()
                    break
                if fault:
                    unsafe_errors.append(
                        "fault_latched:" + str(message.get("fault_reason", "unknown"))
                    )
                    handle.flush()
                    break
                if packet_count % 500 == 0:
                    handle.flush()
        finally:
            receiver.close()
    summary = {
        "schema": "robotics_lab.controller_sim_monitor.v1",
        "duration_sec": time.monotonic() - started,
        "packet_count": packet_count,
        "first_tick": first_tick,
        "last_tick": last_tick,
        "invalid_mode_or_motion_packets": invalid_mode_packets,
        "physical_motion_detected_packets": physical_detected_packets,
        "unsafe_errors": sorted(set(unsafe_errors)),
        "fault_latched_packets": fault_packets,
        "safety_verdict_counts": dict(verdicts),
        "servo_state_source_counts": {
            arm: dict(values) for arm, values in follower_sources.items()
        },
        "follower_active_packets": follower_active_packets,
        "max_follower_actual_lead_m": max_actual_lead_m,
        "max_follower_divergence_m": max_follower_divergence_m,
        "max_follower_divergence_rad": max_follower_divergence_rad,
        "max_follower_projection_error_m": max_projection_error_m,
        "max_follower_projection_error_rad": max_projection_error_rad,
        "max_tcp_actual_displacement_m": max_actual_displacement_m,
        "max_tcp_ref_displacement_m": max_ref_displacement_m,
        "csv": str(csv_path),
    }
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if packet_count > 0 and invalid_mode_packets == 0 and fault_packets == 0 else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=50356)
    parser.add_argument("--gate", type=int, default=0)
    parser.add_argument("--gate-timeout-sec", type=float, default=30.0)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--summary", type=Path)
    args = parser.parse_args()
    if args.gate > 0:
        return gate(args.port, args.gate, args.gate_timeout_sec)
    if args.duration_sec <= 0 or args.csv is None or args.summary is None:
        parser.error("monitor mode requires --duration-sec, --csv, and --summary")
    return monitor(args.port, args.duration_sec, args.csv, args.summary)


if __name__ == "__main__":
    raise SystemExit(main())
