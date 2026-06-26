"""Standalone gripper server (Phase 1 of docs/plans/gripper_server_design.md).

Single owner of the robot-mounted Pika grippers. Wraps the existing
``PikaSerialGripperBackend`` (so it inherits homing / percent<->rad / serial
safety) behind a tiny UDP service:

  * IN  ``robotics_lab.gripper_cmd.v1``   {seq,left{percent,valid},right{...},deadman}
  * OUT ``robotics_lab.gripper_state.v1`` {host_time_ns,left{percent,target_percent,
                                            moving,ok,fault},right{...}}

This removes the current serial contention (policy backend + umi_gripper_follow
both opening the ports) and produces a single gripper-state source. Runs
hardware-free with ``--backend sim`` (a ``SimPikaGripper`` whose feedback eases
toward the commanded position), so it works under ``make run MODE=sim`` and in
tests with no serial hardware.

Later phases route commands through rb_servo_server and stamp the feedback into
the servo_state JSON; the wire schemas here are forward-compatible with that.

Run:    python3 -m policy_runner.gripper_server --backend sim
Monitor: python3 -m policy_runner.gripper_server --monitor
Send:    python3 -m policy_runner.gripper_server --send left=50,right=80
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

from .gripper import GripperCommand, PikaSerialGripperBackend

COMMAND_SCHEMA = "robotics_lab.gripper_cmd.v1"
STATE_SCHEMA = "robotics_lab.gripper_state.v1"
ARMS = ("left", "right")
_MOVING_EPS_PERCENT = 1.0


# --------------------------------------------------------------------------- #
# Hardware-free gripper (sim / dev). Implements only the subset of the Pika SDK
# that PikaSerialGripperBackend uses; the motor position eases toward the last
# commanded angle (time-based, rate independent) so current_percent() reflects
# commands and the GUI viz moves.
# --------------------------------------------------------------------------- #
class SimPikaGripper:
    def __init__(
        self,
        port: str = "",
        *,
        tau_sec: float = 0.12,
        clock: Callable[[], float] = time.monotonic,
        start_rad: float = 0.0,
    ) -> None:
        self.port = port
        self._tau = float(tau_sec)
        self._clock = clock
        self._pos = float(start_rad)
        self._target = float(start_rad)
        self._last = clock()

    def connect(self) -> bool:
        return True

    def enable(self) -> bool:
        return True

    def _advance(self) -> float:
        now = self._clock()
        dt = max(0.0, now - self._last)
        self._last = now
        alpha = 1.0 - math.exp(-dt / self._tau) if self._tau > 0.0 else 1.0
        self._pos += (self._target - self._pos) * alpha
        return self._pos

    def get_motor_position(self) -> float:
        return self._advance()

    def set_motor_angle(self, rad: float) -> bool:
        self._advance()
        self._target = max(0.0, float(rad))
        return True

    def set_zero(self) -> bool:
        self._pos = 0.0
        self._target = 0.0
        self._last = self._clock()
        return True

    def disable(self) -> None:  # pragma: no cover - trivial
        pass

    def disconnect(self) -> None:  # pragma: no cover - trivial
        pass


@dataclass
class GripperServerConfig:
    command_bind: tuple[str, int] = ("0.0.0.0", 50410)
    state_endpoints: tuple[tuple[str, int], ...] = (("127.0.0.1", 50420),)
    backend: str = "sim"  # "sim" | "pika"
    ports: Mapping[str, str] = field(
        default_factory=lambda: {"left": "/dev/pika-left", "right": "/dev/pika-right"}
    )
    sdk_path: str | None = None
    min_rad: float = 0.0
    max_rad: float = 1.75
    deadband_rad: float = 0.005
    backend_max_hz: float = 60.0
    home_on_connect: bool = True
    rate_hz: float = 50.0
    stale_timeout_sec: float = 0.5
    on_stale: str = "hold"  # hold | open | close
    debug_stats: bool = False
    debug_stats_period_sec: float = 1.0


@dataclass
class GripperServerStats:
    command_packets: int = 0
    command_arm_setpoints: int = 0
    backend_send_calls: int = 0
    physical_sends: int = 0
    rate_limited: int = 0
    deadband_holds: int = 0
    backend_drops: int = 0
    last_reason: dict[str, str | None] = field(
        default_factory=lambda: {arm: None for arm in ARMS}
    )
    last_target: dict[str, float | None] = field(
        default_factory=lambda: {arm: None for arm in ARMS}
    )
    last_actual: dict[str, float | None] = field(
        default_factory=lambda: {arm: None for arm in ARMS}
    )


def parse_command(data: bytes) -> dict[str, Any] | None:
    """Decode a gripper_cmd.v1 packet; None if malformed / wrong schema."""
    try:
        msg = json.loads(data.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(msg, dict) or msg.get("schema") != COMMAND_SCHEMA:
        return None
    out: dict[str, Any] = {"deadman": bool(msg.get("deadman", True)), "arms": {}}
    for arm in ARMS:
        block = msg.get(arm)
        if not isinstance(block, Mapping):
            continue
        if not bool(block.get("valid", True)):
            continue
        pct = block.get("percent")
        try:
            pct = float(pct)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(pct):
            continue
        out["arms"][arm] = max(0.0, min(100.0, pct))
    return out


class GripperServer:
    def __init__(
        self,
        config: GripperServerConfig,
        *,
        backend: Any | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self._clock = clock
        self._backend = backend if backend is not None else self._build_backend()
        self._cmd_target: dict[str, float | None] = {arm: None for arm in ARMS}
        self._cmd_time: dict[str, float | None] = {arm: None for arm in ARMS}
        self._deadman = True
        self._cmd_sock: socket.socket | None = None
        self._state_sock: socket.socket | None = None
        self._running = False
        self.stats = GripperServerStats()
        self._last_stats_log = 0.0

    # -- construction ------------------------------------------------------- #
    def _build_backend(self) -> PikaSerialGripperBackend:
        cfg = self.config
        if cfg.backend == "sim":
            clock = self._clock

            def _sim_factory(port: str = "") -> SimPikaGripper:
                return SimPikaGripper(port, clock=clock)

            gripper_cls: Any = _sim_factory
            home = False  # no physical stop to reference in sim
        elif cfg.backend == "pika":
            gripper_cls = None  # real SDK
            home = cfg.home_on_connect
        else:
            raise ValueError(f"unknown gripper backend: {cfg.backend!r}")
        return PikaSerialGripperBackend(
            ports=cfg.ports,
            sdk_path=cfg.sdk_path,
            min_rad=cfg.min_rad,
            max_rad=cfg.max_rad,
            deadband_rad=cfg.deadband_rad,
            max_hz=cfg.backend_max_hz,
            gripper_cls=gripper_cls,
            clock=self._clock,
            home_on_connect=home,
        )

    # -- lifecycle ---------------------------------------------------------- #
    def start(self) -> "GripperServer":
        self._backend.connect()
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._cmd_sock.bind(self.config.command_bind)
        self._cmd_sock.setblocking(False)
        self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return self

    def close(self) -> None:
        self._running = False
        for sock in (self._cmd_sock, self._state_sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._cmd_sock = self._state_sock = None
        try:
            self._backend.close()
        except Exception:  # noqa: BLE001 - shutdown best effort
            pass

    # -- per-iteration logic ------------------------------------------------ #
    def apply_command(self, msg: dict[str, Any], now: float) -> None:
        """Fold one decoded command packet into the latest per-arm setpoints."""
        self._deadman = bool(msg.get("deadman", True))
        arms = msg.get("arms", {})
        self.stats.command_packets += 1
        self.stats.command_arm_setpoints += len(arms) if isinstance(arms, Mapping) else 0
        if not isinstance(arms, Mapping):
            arms = {}
        for arm, pct in arms.items():
            if arm in ARMS:
                self._cmd_target[arm] = float(pct)
                self._cmd_time[arm] = now

    def effective_targets(self, now: float) -> dict[str, float | None]:
        """Per-arm percent to drive, after stale / deadman handling.

        Fresh + deadman engaged -> the commanded percent. Otherwise the on_stale
        policy: hold (keep last commanded position), open (100), or close (0).
        None means "never commanded, nothing to send"."""
        out: dict[str, float | None] = {}
        for arm in ARMS:
            last = self._cmd_target[arm]
            t = self._cmd_time[arm]
            fresh = (
                self._deadman
                and t is not None
                and (now - t) <= self.config.stale_timeout_sec
            )
            if fresh:
                out[arm] = last
            elif self.config.on_stale == "open":
                out[arm] = 100.0
            elif self.config.on_stale == "close":
                out[arm] = 0.0
            else:  # hold
                out[arm] = last
        return out

    def build_state(self, targets: dict[str, float | None], host_time_ns: int) -> dict[str, Any]:
        msg: dict[str, Any] = {"schema": STATE_SCHEMA, "host_time_ns": host_time_ns}
        for arm in ARMS:
            actual = self._backend.current_percent(arm)
            self.stats.last_actual[arm] = None if actual is None else float(actual)
            target = targets.get(arm)
            moving = (
                actual is not None
                and target is not None
                and abs(actual - target) > _MOVING_EPS_PERCENT
            )
            msg[arm] = {
                "percent": None if actual is None else round(float(actual), 2),
                "target_percent": None if target is None else round(float(target), 2),
                "moving": bool(moving),
                "ok": actual is not None,
                "fault": None,
            }
        return msg

    def _drain_commands(self, now: float) -> int:
        n = 0
        if self._cmd_sock is None:
            return 0
        while True:
            try:
                data, _ = self._cmd_sock.recvfrom(4096)
            except (BlockingIOError, OSError):
                break
            msg = parse_command(data)
            if msg is not None:
                self.apply_command(msg, now)
                n += 1
        return n

    def step(self, host_time_ns: int | None = None) -> dict[str, Any]:
        """One control iteration: drain commands, drive the backend, publish state.
        Returns the state packet that was published."""
        now = self._clock()
        self._drain_commands(now)
        targets = self.effective_targets(now)
        for arm, pct in targets.items():
            if pct is None:
                continue
            try:
                result = self._backend.send(
                    GripperCommand(
                        arm=arm,
                        value=float(pct),
                        command_type="target",
                        source="gripper_server",
                    )
                )
                self._record_backend_result(arm, float(pct), result)
            except Exception as exc:  # noqa: BLE001 - backend.send already swallows serial errors
                self._record_backend_exception(arm, float(pct), exc)
        stamp = host_time_ns if host_time_ns is not None else time.time_ns()
        state = self.build_state(targets, stamp)
        self._publish(state)
        self._maybe_log_debug_stats(now)
        return state

    def _record_backend_result(self, arm: str, pct: float, result: Any) -> None:
        self.stats.backend_send_calls += 1
        self.stats.last_target[arm] = pct
        reason = str(getattr(result, "reason", "") or "")
        self.stats.last_reason[arm] = reason
        if bool(getattr(result, "sent_to_physical", False)):
            self.stats.physical_sends += 1
        elif reason == "gripper_rate_limited":
            self.stats.rate_limited += 1
        elif reason == "gripper_deadband_hold":
            self.stats.deadband_holds += 1
        if bool(getattr(result, "dropped", False)):
            self.stats.backend_drops += 1

    def _record_backend_exception(self, arm: str, pct: float, exc: Exception) -> None:
        self.stats.backend_send_calls += 1
        self.stats.backend_drops += 1
        self.stats.last_target[arm] = pct
        self.stats.last_reason[arm] = f"backend_exception:{type(exc).__name__}"

    def _maybe_log_debug_stats(self, now: float) -> None:
        if not self.config.debug_stats:
            return
        period = max(0.1, float(self.config.debug_stats_period_sec))
        if now - self._last_stats_log < period:
            return
        self._last_stats_log = now

        def arm_summary(arm: str) -> str:
            target = self.stats.last_target.get(arm)
            actual = self.stats.last_actual.get(arm)
            reason = self.stats.last_reason.get(arm) or "-"
            target_text = "-" if target is None else f"{target:.2f}"
            actual_text = "-" if actual is None else f"{actual:.2f}"
            return f"{arm}:tgt={target_text},actual={actual_text},reason={reason}"

        print(
            "[gripper_server] stats "
            f"cmd_packets={self.stats.command_packets} "
            f"cmd_arms={self.stats.command_arm_setpoints} "
            f"backend_calls={self.stats.backend_send_calls} "
            f"physical_sends={self.stats.physical_sends} "
            f"rate_limited={self.stats.rate_limited} "
            f"deadband_holds={self.stats.deadband_holds} "
            f"drops={self.stats.backend_drops} "
            f"{arm_summary('left')} {arm_summary('right')}",
            file=sys.stderr,
            flush=True,
        )

    def _publish(self, state: dict[str, Any]) -> None:
        if self._state_sock is None:
            return
        payload = json.dumps(state, separators=(",", ":")).encode("utf-8")
        for endpoint in self.config.state_endpoints:
            try:
                self._state_sock.sendto(payload, endpoint)
            except OSError:
                pass

    def run(self) -> None:
        self._running = True
        period = 1.0 / self.config.rate_hz if self.config.rate_hz > 0 else 0.0
        try:
            while self._running:
                start = self._clock()
                self.step()
                if period > 0.0:
                    sleep = period - (self._clock() - start)
                    if sleep > 0.0:
                        time.sleep(sleep)
        except KeyboardInterrupt:  # pragma: no cover - operator Ctrl-C
            pass
        finally:
            self.close()


# --------------------------------------------------------------------------- #
# CLI: run server / monitor / one-shot send
# --------------------------------------------------------------------------- #
def _parse_endpoint(text: str) -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    return (host or "127.0.0.1", int(port))


def _build_config_from_args(args: argparse.Namespace) -> GripperServerConfig:
    return GripperServerConfig(
        command_bind=_parse_endpoint(args.bind),
        state_endpoints=tuple(_parse_endpoint(e) for e in (args.state_endpoint or ["127.0.0.1:50420"])),
        backend=args.backend,
        ports={"left": args.left_port, "right": args.right_port},
        sdk_path=args.pika_sdk_path,
        rate_hz=args.rate,
        stale_timeout_sec=args.stale_timeout,
        on_stale=args.on_stale,
        home_on_connect=args.home_on_connect,
        debug_stats=args.debug_stats,
        debug_stats_period_sec=args.debug_stats_period,
    )


def _send_once(spec: str, bind: str) -> None:
    host, port = _parse_endpoint(bind)
    msg: dict[str, Any] = {"schema": COMMAND_SCHEMA, "seq": 0, "deadman": True}
    for token in spec.split(","):
        arm, _, val = token.partition("=")
        arm = arm.strip()
        if arm in ARMS:
            msg[arm] = {"percent": float(val), "valid": True}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(json.dumps(msg).encode("utf-8"), (host if host != "0.0.0.0" else "127.0.0.1", port))
    sock.close()
    print(f"sent {msg}", file=sys.stderr)


def _monitor(endpoint: str) -> None:
    host, port = _parse_endpoint(endpoint)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host if host != "127.0.0.1" else "0.0.0.0", port))
    print(f"listening for {STATE_SCHEMA} on {host}:{port}", file=sys.stderr)
    while True:
        data, _ = sock.recvfrom(4096)
        try:
            msg = json.loads(data.decode("utf-8"))
        except ValueError:
            continue
        l, r = msg.get("left", {}), msg.get("right", {})
        print(f"L pct={l.get('percent')} tgt={l.get('target_percent')} mov={l.get('moving')} | "
              f"R pct={r.get('percent')} tgt={r.get('target_percent')} mov={r.get('moving')}",
              flush=True)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="robotics_lab gripper server (Phase 1)")
    p.add_argument("--backend", choices=("sim", "pika"), default="sim")
    p.add_argument("--bind", default="0.0.0.0:50410", help="command listen endpoint")
    p.add_argument("--state-endpoint", action="append", help="state publish endpoint (repeatable)")
    p.add_argument("--left-port", default="/dev/pika-left")
    p.add_argument("--right-port", default="/dev/pika-right")
    p.add_argument("--pika-sdk-path", default=None)
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--stale-timeout", type=float, default=0.5)
    p.add_argument("--on-stale", choices=("hold", "open", "close"), default="hold")
    p.add_argument(
        "--home-on-connect",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "home grippers on connect (drive to closed stop + re-zero so absolute-angle "
            "commands map identically per arm). --no-home-on-connect holds each gripper "
            "at its current power-on position (no startup motion)."
        ),
    )
    p.add_argument("--debug-stats", action="store_true", help="print 1 Hz command/send diagnostics")
    p.add_argument("--debug-stats-period", type=float, default=1.0)
    p.add_argument("--send", default=None, help="one-shot send, e.g. 'left=50,right=80'")
    p.add_argument("--monitor", action="store_true", help="print published gripper state")
    args = p.parse_args(argv)

    if args.send is not None:
        _send_once(args.send, args.bind)
        return 0
    if args.monitor:
        endpoint = (args.state_endpoint or ["127.0.0.1:50420"])[0]
        try:
            _monitor(endpoint)
        except KeyboardInterrupt:
            pass
        return 0
    config = _build_config_from_args(args)
    server = GripperServer(config).start()
    print(
        f"gripper_server up: backend={config.backend} cmd<-{config.command_bind} "
        f"state->{list(config.state_endpoints)} rate={config.rate_hz}Hz on_stale={config.on_stale}",
        file=sys.stderr,
    )
    server.run()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
