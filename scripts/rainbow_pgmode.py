#!/usr/bin/env python3
"""Set or verify Rainbow controller pgmode simulation.

This tool sends only the no-motion controller mode command
``pgmode simulation``. It never sends ``pgmode real``, reset, collision,
servo-power, or motion commands.
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import socket
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Any


REAL_ROBOT_IPS = {"172.28.60.200", "172.28.60.201"}
PGMODE_SIMULATION_COMMAND = "pgmode simulation\n"
SCHEMA = "robotics_lab.rainbow_pgmode.v1"


class RainbowPgmodeError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Set or verify Rainbow controller pgmode simulation. This is a "
            "no-motion setup tool for rbpodo controller-simulation benchmarks."
        )
    )
    parser.add_argument("--ips", nargs="+", help="Controller IPs to contact.")
    parser.add_argument("--port", type=int, default=5000, help="Rainbow command TCP port.")
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument(
        "--set-simulation",
        action="store_true",
        help="Send 'pgmode simulation' to each controller.",
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="Only read rbpodo CobotData.real_vs_simulation_mode; do not send pgmode.",
    )
    parser.add_argument("--summary-json", type=Path, help="Optional JSON summary artifact path.")
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required before connecting to known real controller IPs.",
    )
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def mode_name(value: Any) -> str:
    try:
        mode = int(value)
    except (TypeError, ValueError):
        return "unknown"
    if mode == 1:
        return "simulation"
    if mode == 0:
        return "real"
    return "unknown"


def mode_is_simulation(value: Any) -> bool | None:
    name = mode_name(value)
    if name == "simulation":
        return True
    if name == "real":
        return False
    return None


def classify_response(raw_response: str, response_timed_out: bool) -> tuple[bool, str]:
    if response_timed_out and not raw_response:
        return True, "no_response_after_send"
    text = raw_response.strip()
    if not text:
        return True, "empty_response"
    lowered = text.lower()
    if any(token in lowered for token in ("error", "fail", "invalid", "denied", "unknown")):
        return False, "controller_error_response"
    if "pgmode real" in lowered:
        return False, "unexpected_real_mode_response"
    if any(token in lowered for token in ("ok", "success", "simulation", "pgmode")):
        return True, "controller_success_response"
    return True, "unrecognized_non_error_response"


def send_pgmode_simulation(ip: str, port: int, timeout_sec: float) -> dict[str, Any]:
    started = time.monotonic()
    raw_response = ""
    response_timed_out = False
    try:
        with socket.create_connection((ip, port), timeout=timeout_sec) as sock:
            sock.settimeout(timeout_sec)
            sock.sendall(PGMODE_SIMULATION_COMMAND.encode("ascii"))
            chunks: list[bytes] = []
            while True:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    response_timed_out = True
                    break
                if not chunk:
                    break
                chunks.append(chunk)
                if b"\n" in chunk:
                    break
            raw_response = b"".join(chunks).decode("utf-8", errors="replace")
    except OSError as exc:
        return {
            "pgmode_command_sent": False,
            "command_ok": False,
            "response_raw": raw_response,
            "response_classification": "transport_error",
            "response_timed_out": response_timed_out,
            "duration_ms": (time.monotonic() - started) * 1000.0,
            "error_name": type(exc).__name__,
            "error_message": str(exc),
        }
    command_ok, classification = classify_response(raw_response, response_timed_out)
    return {
        "pgmode_command_sent": True,
        "command_ok": command_ok,
        "response_raw": raw_response,
        "response_classification": classification,
        "response_timed_out": response_timed_out,
        "duration_ms": (time.monotonic() - started) * 1000.0,
    }


def read_controller_mode(ip: str, timeout_sec: float) -> dict[str, Any]:
    try:
        rbpodo = importlib.import_module("rbpodo")
    except Exception as exc:
        return {
            "verification_available": False,
            "verified_by": "rbpodo.CobotData",
            "verification_error_name": type(exc).__name__,
            "verification_error_message": "Python rbpodo module is unavailable",
        }
    if not hasattr(rbpodo, "CobotData"):
        return {
            "verification_available": False,
            "verified_by": "rbpodo.CobotData",
            "verification_error_name": "AttributeError",
            "verification_error_message": "Python rbpodo module does not expose CobotData",
        }
    try:
        data = rbpodo.CobotData(ip)
        state = data.request_data(timeout_sec)
        if state is None:
            raise RainbowPgmodeError("CobotData.request_data returned no state")
        sdata = getattr(state, "sdata", state)
        raw_mode = getattr(sdata, "real_vs_simulation_mode", None)
        is_sim = mode_is_simulation(raw_mode)
        return {
            "verification_available": True,
            "verified_by": "rbpodo.CobotData",
            "real_vs_simulation_mode": raw_mode,
            "controller_mode": mode_name(raw_mode),
            "confirmed_simulation": is_sim is True,
            "controller_not_simulation_warning": (
                None if is_sim is True else "controller not confirmed in pgmode simulation"
            ),
        }
    except Exception as exc:
        return {
            "verification_available": False,
            "verified_by": "rbpodo.CobotData",
            "verification_error_name": type(exc).__name__,
            "verification_error_message": str(exc),
        }


def validate_common(
    ips: list[str],
    timeout_sec: float,
    port: int,
    confirmation: bool,
    set_simulation: bool,
    verify_only: bool,
) -> None:
    if not ips:
        raise RainbowPgmodeError("--ips is required")
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise RainbowPgmodeError("--timeout-sec must be finite and positive")
    if port < 1 or port > 65535:
        raise RainbowPgmodeError("--port must be in [1, 65535]")
    if set_simulation and verify_only:
        raise RainbowPgmodeError("--set-simulation and --verify-only are mutually exclusive")
    if not set_simulation and not verify_only:
        raise RainbowPgmodeError("choose --set-simulation or --verify-only")
    known = set(ips) & REAL_ROBOT_IPS
    if known and not confirmation:
        raise RainbowPgmodeError(
            "refusing known real controller IP without --i-understand-this-connects-to-real-controller"
        )
    if known and os.environ.get("RB_ALLOW_REAL_ROBOT") != "1":
        raise RainbowPgmodeError("known real controller IP requires RB_ALLOW_REAL_ROBOT=1")


def run_pgmode(
    ips: list[str],
    timeout_sec: float,
    *,
    port: int = 5000,
    confirmation: bool = False,
    set_simulation: bool = False,
    verify_only: bool = False,
) -> dict[str, Any]:
    validate_common(ips, timeout_sec, port, confirmation, set_simulation, verify_only)
    results: list[dict[str, Any]] = []
    for ip in ips:
        item: dict[str, Any] = {
            "ip": ip,
            "port": port,
            "ok": False,
            "action": "verify_only" if verify_only else "set_simulation",
            "pgmode_real_sent": False,
            "motion_command_sent": False,
            "reset_command_sent": False,
            "collision_threshold_changed": False,
            "servo_power_changed": False,
        }
        command_result: dict[str, Any] = {}
        if set_simulation:
            command_result = send_pgmode_simulation(ip, port, timeout_sec)
            item.update(command_result)
        else:
            item.update({
                "pgmode_command_sent": False,
                "command_ok": None,
                "response_raw": "",
                "response_classification": "not_sent_verify_only",
            })

        verification = read_controller_mode(ip, timeout_sec)
        item.update(verification)
        if verification.get("verification_available"):
            item["ok"] = bool(verification.get("confirmed_simulation"))
            if not item["ok"]:
                item["error_name"] = "controller_not_confirmed_in_pgmode_simulation"
                item["error_message"] = "controller not confirmed in pgmode simulation"
        elif verify_only:
            item["ok"] = False
            item["error_name"] = "pgmode_verification_unavailable"
            item["error_message"] = "controller not confirmed in pgmode simulation"
        else:
            item["ok"] = bool(command_result.get("command_ok"))
            if item["ok"]:
                item["confirmed_simulation"] = False
                item["controller_not_simulation_warning"] = (
                    "pgmode simulation command was sent but controller mode verification was unavailable"
                )
            else:
                item.setdefault("error_name", "pgmode_command_failed")
                item.setdefault("error_message", "pgmode simulation command was not accepted")
        results.append(item)

    overall_ok = all(bool(item.get("ok")) for item in results)
    return {
        "schema": SCHEMA,
        "timestamp_unix_sec": time.time(),
        "action": "verify_only" if verify_only else "set_simulation",
        "ips": list(ips),
        "known_real_controller_ips": sorted(REAL_ROBOT_IPS),
        "safety_note": (
            "This tool only sends pgmode simulation or reads controller mode. "
            "It never sends pgmode real, reset, collision-threshold, servo-power, or motion commands."
        ),
        "overall_result": "ok" if overall_ok else "error",
        "results": results,
    }


def ensure_controller_simulation_mode(
    ips: list[str],
    timeout_sec: float,
    *,
    port: int = 5000,
    confirmation: bool = False,
    set_simulation: bool = False,
    verify_only: bool = True,
) -> dict[str, Any]:
    summary = run_pgmode(
        ips,
        timeout_sec,
        port=port,
        confirmation=confirmation,
        set_simulation=set_simulation,
        verify_only=verify_only,
    )
    if summary.get("overall_result") != "ok":
        raise RainbowPgmodeError("controller not confirmed in pgmode simulation")
    return summary


def write_summary(path: Path, summary: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


class _FakePgmodeServer:
    def __init__(self, response: str):
        self.response = response
        self.received = ""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        try:
            conn, _ = self.sock.accept()
        except OSError:
            return
        with conn:
            while True:
                data = conn.recv(1)
                if not data:
                    break
                self.received += data.decode("utf-8", errors="replace")
                if data == b"\n":
                    break
            conn.sendall((self.response + "\n").encode("utf-8"))
        self.sock.close()

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass
        self.thread.join(timeout=1.0)


def run_self_test() -> int:
    success_server = _FakePgmodeServer("OK pgmode simulation")
    try:
        result = send_pgmode_simulation("127.0.0.1", success_server.port, 0.5)
    finally:
        success_server.close()
    assert success_server.received == PGMODE_SIMULATION_COMMAND
    assert result["command_ok"] is True

    error_server = _FakePgmodeServer("ERROR cannot change mode")
    try:
        result = send_pgmode_simulation("127.0.0.1", error_server.port, 0.5)
    finally:
        error_server.close()
    assert result["command_ok"] is False

    try:
        run_pgmode(["172.28.60.200"], 0.1, set_simulation=True, confirmation=False)
    except RainbowPgmodeError as exc:
        assert "--i-understand-this-connects-to-real-controller" in str(exc)
    else:
        raise AssertionError("expected confirmation failure")

    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "summary.json"
        summary = {
            "schema": SCHEMA,
            "overall_result": "ok",
            "results": [{"ip": "127.0.0.1", "ok": True}],
        }
        write_summary(path, summary)
        assert json.loads(path.read_text(encoding="utf-8"))["schema"] == SCHEMA

    print("rainbow_pgmode self-test passed")
    return 0


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    try:
        summary = run_pgmode(
            list(args.ips or []),
            args.timeout_sec,
            port=args.port,
            confirmation=args.i_understand_this_connects_to_real_controller,
            set_simulation=args.set_simulation,
            verify_only=args.verify_only,
        )
    except RainbowPgmodeError as exc:
        print(f"rainbow_pgmode: FAIL: {exc}", file=sys.stderr)
        return 2
    if args.summary_json:
        write_summary(args.summary_json, summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("overall_result") == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
