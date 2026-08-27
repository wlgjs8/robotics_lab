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
            # The server publishes the residual as position_error_m /
            # orientation_error_rad (CartesianSolveTelemetry, the same fields the
            # servo log writes as <arm>_cart_pos_err_m / _cart_ori_err_rad).
            # These were read under the wrong names until 2026-08-27, so both
            # were silently always None: the abort line printed no residual and
            # JointLimitStallTracker could not tell a wedged arm from one that
            # was riding a bound and tracking. Keep the old names as a fallback
            # so a server that ever published them still reports.
            pos_err_m=_optional_float(
                solve.get("position_error_m", solve.get("pos_err_m"))
            ),
            ori_err_rad=_optional_float(
                solve.get("orientation_error_rad", solve.get("ori_err_rad"))
            ),
        )
    return JointLimitStallReadback(
        blocked=False,
        arm=None,
        joint_index=None,
        q_actual_deg=None,
        pos_err_m=None,
        ori_err_rad=None,
    )


@dataclass(frozen=True)
class RoiExitReadback:
    """One tick's answer to "has this arm actually left the safety ROI box".

    Reads the server's own MEASURED-pose evaluation (roi_box.<arm>.measured), not a
    reimplementation here. The box is checked against the TCP *and* the four gripper
    tip points, interpolated to the live jaw opening, against the effective runtime
    bounds -- reproducing that in Python would be a second copy of the geometry free
    to disagree with the layer that is actually stopping the arm.

    It disagreed, measured 2026-08-27 (servo_log_20260827_232728.csv, right arm,
    t=72.6-77.1 s): the server raised RoiViolation 13 times and the damper fought the
    boundary for 4.5 s, with the tip envelope up to 6.2 mm outside the operator's box
    on 1949 of 2701 ticks -- while the TCP point alone never came closer than 8.7 mm
    INSIDE it. A TCP-only test would have reported that arm as inside for the whole
    event.

    Sibling key roi_box.<arm>.violated is the COMMANDED-pose verdict, which is what
    the damper acts on; this is the measured twin, i.e. where the arm went, not what
    was asked of it.

    `outside` is None when the answer is unknown (ROI disabled, the server too old to
    publish the measured block, FK unavailable). Unknown is not "outside": nothing
    moves an arm on the strength of a state message it could not read.
    """

    outside: bool | None
    min_margin_m: float | None
    closest_face: str | None


def roi_exit_from_snapshot(snapshot: StateSnapshot) -> dict[str, RoiExitReadback]:
    """Per-arm measured-pose ROI readback for one state message.

    Per ARM and independent: both arms can be outside at once and each is recovered
    on its own, unlike joint_limit_stall_from_snapshot which reports only the first
    blocked arm because the caller there aborts the whole rollout either way.
    """

    roi = snapshot.payload.get("roi_box")
    enabled = isinstance(roi, dict) and roi.get("enabled") is True
    result: dict[str, RoiExitReadback] = {}
    for arm in ("left", "right"):
        arm_block = roi.get(arm) if isinstance(roi, dict) else None
        measured = arm_block.get("measured") if isinstance(arm_block, dict) else None
        if (
            not enabled
            or not isinstance(measured, dict)
            or measured.get("checked") is not True
            or not isinstance(measured.get("violated"), bool)
        ):
            result[arm] = RoiExitReadback(None, None, None)
            continue
        face = measured.get("closest_face")
        result[arm] = RoiExitReadback(
            outside=bool(measured["violated"]),
            min_margin_m=_optional_float(measured.get("min_margin_m")),
            closest_face=face if isinstance(face, str) else None,
        )
    return result


class RoiExitTracker:
    """Edge-detects "this arm's measured TCP left the ROI box" for one arm.

    Fires ONCE on the inside->outside edge and stays quiet until the arm is read
    back inside, so a level condition ("still outside") becomes the single event
    the recovery needs. Re-arming on the way back in is what makes a second trip
    possible without any timer: the recovery parks the arm at the init pose, which
    is inside, so the tracker re-arms as a side effect of the recovery working.

    The only debounce is _CONFIRM_SAMPLES consecutive outside readings. This is not
    a tuning knob and is deliberately not configurable -- it exists so that ONE
    malformed or reordered UDP state datagram cannot send an arm home, and at the
    command-loop rate it costs single-digit milliseconds. An unknown reading
    (outside is None) neither confirms nor clears: the tracker holds its state.
    """

    _CONFIRM_SAMPLES = 2

    def __init__(self) -> None:
        self._outside = False
        self._streak = 0

    @property
    def outside(self) -> bool:
        return self._outside

    def update(self, readback: RoiExitReadback) -> bool:
        """Returns True on the tick the arm is confirmed to have left the box."""
        if readback.outside is None:
            return False
        if not readback.outside:
            self._outside = False
            self._streak = 0
            return False
        if self._outside:
            return False
        self._streak += 1
        if self._streak < self._CONFIRM_SAMPLES:
            return False
        self._outside = True
        self._streak = 0
        return True

    def reset(self) -> None:
        self._outside = False
        self._streak = 0


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

    A joint RIDING its bound is not a standoff. A posture can be saturated and
    still perfectly feasible: the solve puts the elbow on the limit and reaches
    the commanded pose anyway, with the other five joints doing the work. The
    clamp is then continuously active and the arm is tracking, which is the one
    case the rule above gets wrong -- it aborted a healthy rollout on
    2026-08-27 (servo_log_20260827_230413.csv, left arm): J3 sat inside a
    0.167 deg band at the 149.90 deg standoff for 7.8 s while J1/J4/J6 swept
    10.5/13.0/12.1 deg, the TCP travelled 28/36/38 mm in x/y/z, IK refused zero
    ticks, and the Cartesian residual FELL 16.25 -> 7.88 -> 0.01 mm. The abort
    fired at the 4 s mark -- just as the residual reached 0.01 mm.

    So convergence of the solve residual also restarts the clock. The separator
    is the CONTINUOUS time the residual stays converged, and it is not close:

        real standoff  (servo_log_20260827_213651.csv, 3 episodes)
            residual p50 8.4 / 18.9 / 29.5 mm, flat or growing across the
            episode, longest continuous sub-1 mm run 0.02-0.03 s
        riding the bound (servo_log_20260827_230413.csv)
            residual p50 6.1 mm falling to 0.01 mm, longest continuous
            sub-1 mm run 3.60 s

    A 0.5 s converged hold sits two orders of magnitude away from either side.
    Orientation is gated too; it costs nothing (in both logs the position and
    orientation residuals converge together, giving identical durations) and it
    keeps the test honest about reaching the commanded POSE, not just a point.

    If the server does not publish the residual the readback carries None, and
    convergence cannot be judged -- the clock then behaves exactly as before.
    """

    def __init__(
        self,
        hold_sec: float,
        converged_pos_err_m: float = 0.001,
        converged_ori_err_rad: float = 0.008727,  # 0.5 deg
        converged_hold_sec: float = 0.5,
    ) -> None:
        self._hold_sec = float(hold_sec)
        self._converged_pos_err_m = float(converged_pos_err_m)
        self._converged_ori_err_rad = float(converged_ori_err_rad)
        self._converged_hold_sec = float(converged_hold_sec)
        self._since: float | None = None
        self._key: tuple[str | None, int | None] | None = None
        self._converged_since: float | None = None

    def _converged(self, readback: JointLimitStallReadback) -> bool:
        """True when the solve reached the commanded pose this tick."""
        if readback.pos_err_m is None or readback.ori_err_rad is None:
            return False
        return (
            readback.pos_err_m < self._converged_pos_err_m
            and readback.ori_err_rad < self._converged_ori_err_rad
        )

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
            self._converged_since = None
            return None
        key = (readback.arm, readback.joint_index)
        if self._key != key:
            self._key = key
            self._since = now_monotonic
            self._converged_since = None
            return None
        if self._since is None:
            self._since = now_monotonic
            self._converged_since = None
            return None
        # The arm is clamped but tracking: once it has held the commanded pose
        # for converged_hold_sec, the bound is being ridden, not fought.
        if self._converged(readback):
            if self._converged_since is None:
                self._converged_since = now_monotonic
            elif now_monotonic - self._converged_since >= self._converged_hold_sec:
                self._since = now_monotonic
                self._converged_since = None
                return None
        else:
            self._converged_since = None
        held = now_monotonic - self._since
        if held < self._hold_sec:
            return None
        # Latch this episode so the caller is told once, not every tick.
        self._since = None
        self._key = None
        self._converged_since = None
        return held

    def reset(self) -> None:
        self._since = None
        self._key = None
        self._converged_since = None


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
