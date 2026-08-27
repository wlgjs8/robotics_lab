from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class UdpEndpoint:
    host: str
    port: int


@dataclass(frozen=True)
class StateSnapshot:
    payload: dict[str, Any]
    received_monotonic: float

    def is_stale(self, now_monotonic: float, stale_timeout_sec: float) -> bool:
        return now_monotonic - self.received_monotonic > stale_timeout_sec


@dataclass(frozen=True)
class CommandSourceLeaseReadback:
    active: bool
    expired: bool
    enforce_lease: bool
    active_source_id: str | None
    active_session_id: str | None
    active_lease_token: str | None
    verdict: str | None
    reason: str | None
    command_requires_lease: bool | None
    command_has_lease: bool | None

    def matches(self, source_id: str, session_id: str, lease_token: str | None = None) -> bool:
        if not self.active or self.expired:
            return False
        if self.active_source_id != source_id or self.active_session_id != session_id:
            return False
        return lease_token is None or self.active_lease_token == lease_token


@dataclass(frozen=True)
class FaultLatchReadback:
    latched: bool
    motion_state: str | None
    latched_fault_reason: str | None
    reason: str | None


@dataclass(frozen=True)
class JointLimitStallReadback:
    """One tick's answer to "is an arm wedged against a joint bound".

    A policy can ask for a pose the arm cannot reach, and when the blocking
    joint is at its bound the result is a standoff nobody breaks: the arm
    cannot get closer, so the scene barely changes, so the policy asks for the
    same pose again. Measured 2026-08-27 (servo_log_20260827_141433.csv): both
    elbows sat at their bound for the last 6.4 s of a rollout with the
    Cartesian error stuck at 32 mm / 0.081 rad, while the IK asked to close on
    the bound 2912 times and to retreat ZERO times. The controller is behaving
    correctly there -- the barrier only refuses the closing direction, retreat
    stays free -- so nothing downstream can end it.

    `blocked` means the clamp is active on this arm right now. It says nothing
    about duration; the caller decides how long is too long.
    """

    blocked: bool
    arm: str | None
    joint_index: int | None
    q_actual_deg: float | None
    pos_err_m: float | None
    ori_err_rad: float | None


def parse_udp_endpoint(endpoint: str) -> UdpEndpoint:
    prefix = "udp://"
    if not endpoint.startswith(prefix):
        raise ValueError(f"only udp:// endpoints are supported: {endpoint}")
    rest = endpoint[len(prefix):]
    host, sep, port_text = rest.rpartition(":")
    if not sep or not host:
        raise ValueError(f"invalid UDP endpoint: {endpoint}")
    port = int(port_text)
    if port < 0 or port > 65535:
        raise ValueError(f"UDP port out of range: {endpoint}")
    return UdpEndpoint("127.0.0.1" if host == "localhost" else host, port)


def command_source_lease_from_snapshot(snapshot: StateSnapshot) -> CommandSourceLeaseReadback:
    raw = snapshot.payload.get("command_source", {})
    if not isinstance(raw, dict):
        raw = {}
    return CommandSourceLeaseReadback(
        active=bool(raw.get("active", False)),
        expired=bool(raw.get("expired", False)),
        enforce_lease=bool(raw.get("enforce_lease", False)),
        active_source_id=_optional_str(raw.get("active_source_id")),
        active_session_id=_optional_str(raw.get("active_session_id")),
        active_lease_token=_optional_str(raw.get("active_lease_token")),
        verdict=_optional_str(raw.get("verdict")),
        reason=_optional_str(raw.get("reason")),
        command_requires_lease=_optional_bool(raw.get("command_requires_lease")),
        command_has_lease=_optional_bool(raw.get("command_has_lease")),
    )


def fault_latch_from_snapshot(snapshot: StateSnapshot) -> FaultLatchReadback:
    payload = snapshot.payload
    raw_context = payload.get("fault_context")
    top_level_motion_state = _optional_str(payload.get("motion_state"))
    top_level_latched = _optional_bool(payload.get("fault_latched")) is True
    context_latched = False
    motion_state = top_level_motion_state
    latched_fault_reason = None
    reason = None

    if isinstance(raw_context, dict):
        context_latched = _optional_bool(raw_context.get("latched")) is True
        motion_state = _optional_str(raw_context.get("motion_state")) or top_level_motion_state
        latched_fault_reason = _optional_str(raw_context.get("latched_fault_reason"))
        reason = _optional_str(raw_context.get("reason"))

    latched = (
        context_latched
        or top_level_latched
        or motion_state in {"FaultLatched", "EmergencyLatched"}
        or top_level_motion_state in {"FaultLatched", "EmergencyLatched"}
    )
    return FaultLatchReadback(
        latched=latched,
        motion_state=motion_state,
        latched_fault_reason=latched_fault_reason,
        reason=reason,
    )


def joint_limit_stall_from_snapshot(snapshot: StateSnapshot) -> JointLimitStallReadback:
    """Read the per-arm joint-limit clamp out of one state message.

    The server already publishes everything needed, per arm, under
    <arm>.cartesian_solve: safety_joint_limit_clamped (did the clamp bind this
    tick), safety_joint_limit_limited_joint (which axis), and the Cartesian
    residual the solve could not remove. Reports the FIRST blocked arm; when
    both are blocked, which one is named does not change the decision.
    """

    payload = snapshot.payload
    for arm in ("left", "right"):
        arm_payload = payload.get(arm)
        if not isinstance(arm_payload, dict):
            continue
        solve = arm_payload.get("cartesian_solve")
        if not isinstance(solve, dict):
            continue
        if _optional_bool(solve.get("safety_joint_limit_clamped")) is not True:
            continue
        joint_index = solve.get("safety_joint_limit_limited_joint")
        q_actual = None
        joints = arm_payload.get("q_actual_deg")
        if (
            isinstance(joint_index, int)
            and isinstance(joints, list)
            and 0 <= joint_index < len(joints)
            and isinstance(joints[joint_index], (int, float))
        ):
            q_actual = float(joints[joint_index])
        return JointLimitStallReadback(
            blocked=True,
            arm=arm,
            joint_index=joint_index if isinstance(joint_index, int) else None,
            q_actual_deg=q_actual,
            pos_err_m=_optional_float(solve.get("pos_err_m")),
            ori_err_rad=_optional_float(solve.get("ori_err_rad")),
        )
    return JointLimitStallReadback(
        blocked=False,
        arm=None,
        joint_index=None,
        q_actual_deg=None,
        pos_err_m=None,
        ori_err_rad=None,
    )


class StateStreamLeaseReadback:
    """Waits for command-source lease confirmation on the UDP state stream."""

    def __init__(self, state_client: Any):
        self.state_client = state_client

    def wait_for_active_lease(
        self,
        *,
        source_id: str,
        session_id: str,
        lease_token: str | None = None,
        timeout_sec: float = 1.0,
        poll_interval_sec: float = 0.01,
        monotonic_fn: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> CommandSourceLeaseReadback:
        deadline = monotonic_fn() + max(timeout_sec, 0.0)
        last_readback: CommandSourceLeaseReadback | None = None
        while True:
            snapshot = getattr(self.state_client, "latest", None)
            if snapshot is not None:
                last_readback = command_source_lease_from_snapshot(snapshot)
                if last_readback.matches(source_id, session_id, lease_token):
                    return last_readback
            now = monotonic_fn()
            if now >= deadline:
                raise TimeoutError(_lease_timeout_message(source_id, session_id, last_readback))
            sleep_fn(min(max(poll_interval_sec, 0.0), deadline - now))


class RobotStateClient:
    """UDP JSON state subscriber with latest-snapshot cache."""

    def __init__(
        self,
        bind: str,
        stale_timeout_sec: float = 0.5,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ):
        self.bind = bind
        self.stale_timeout_sec = stale_timeout_sec
        self._socket_factory = socket_factory
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: StateSnapshot | None = None

    @property
    def latest(self) -> StateSnapshot | None:
        with self._lock:
            return self._latest

    @property
    def local_port(self) -> int:
        if self._socket is None:
            raise RuntimeError("state client is not open")
        return int(self._socket.getsockname()[1])

    def open(self) -> None:
        if self._socket is not None:
            return
        endpoint = parse_udp_endpoint(self.bind)
        sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((endpoint.host, endpoint.port))
        self._socket = sock

    def close(self) -> None:
        self.stop()
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def start(self) -> None:
        self.open()
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._thread_main, name="policy-runner-state", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def poll_once(self, timeout_sec: float = 0.0) -> StateSnapshot | None:
        self.open()
        assert self._socket is not None
        self._socket.settimeout(timeout_sec)
        try:
            data, _addr = self._socket.recvfrom(65536)
        except socket.timeout:
            return None
        snapshot = StateSnapshot(json.loads(data.decode("utf-8")), time.monotonic())
        with self._lock:
            self._latest = snapshot
        return snapshot

    def is_latest_stale(self, now_monotonic: float | None = None) -> bool:
        snapshot = self.latest
        if snapshot is None:
            return True
        now = time.monotonic() if now_monotonic is None else now_monotonic
        return snapshot.is_stale(now, self.stale_timeout_sec)

    def _thread_main(self) -> None:
        while self._running:
            self.poll_once(timeout_sec=0.1)


def _optional_str(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


def _optional_bool(value: Any) -> bool | None:
    return value if isinstance(value, bool) else None


class JointLimitStallTracker:
    """Turns per-tick "blocked" readbacks into "blocked for too long".

    The stall that matters is a STANDOFF, not a clamp: brushing a bound while
    passing through it is ordinary, and the barrier exists to make that smooth.
    What nothing downstream can break is the arm sitting on the bound while the
    policy keeps asking for the pose behind it. So this only counts time while
    the clamp stays continuously active on the SAME joint, and any tick that
    clears the clamp (or moves to a different joint) restarts the clock --
    which is exactly what a policy successfully steering away looks like.
    """

    def __init__(self, hold_sec: float) -> None:
        self._hold_sec = float(hold_sec)
        self._since: float | None = None
        self._key: tuple[str | None, int | None] | None = None

    @property
    def enabled(self) -> bool:
        return self._hold_sec > 0.0

    def held_for(self, now_monotonic: float) -> float:
        if self._since is None:
            return 0.0
        return now_monotonic - self._since

    def update(
        self, readback: JointLimitStallReadback, now_monotonic: float
    ) -> float | None:
        """Returns the stall duration once it exceeds hold_sec, else None.

        Reports once per episode: the caller acts, and a repeat only comes
        after the clamp clears and the arm wedges again.
        """
        if not self.enabled or not readback.blocked:
            self._since = None
            self._key = None
            return None
        key = (readback.arm, readback.joint_index)
        if self._key != key:
            self._key = key
            self._since = now_monotonic
            return None
        if self._since is None:
            self._since = now_monotonic
            return None
        held = now_monotonic - self._since
        if held < self._hold_sec:
            return None
        # Latch this episode so the caller is told once, not every tick.
        self._since = None
        self._key = None
        return held

    def reset(self) -> None:
        self._since = None
        self._key = None


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _lease_timeout_message(
    source_id: str,
    session_id: str,
    last_readback: CommandSourceLeaseReadback | None,
) -> str:
    if last_readback is None:
        return (
            "command source lease readback timed out: "
            f"no state command_source observed for source_id={source_id} session_id={session_id}"
        )
    return (
        "command source lease readback timed out: "
        f"wanted source_id={source_id} session_id={session_id}, "
        f"last_active={last_readback.active} expired={last_readback.expired} "
        f"active_source_id={last_readback.active_source_id} "
        f"active_session_id={last_readback.active_session_id} "
        f"verdict={last_readback.verdict} reason={last_readback.reason}"
    )
