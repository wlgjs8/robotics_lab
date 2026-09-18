from __future__ import annotations

import json
import logging
import math
import os
import re
import sys
import threading
import time
import types
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Protocol


REAL_GRIPPER_ENV = "RB_ALLOW_REAL_GRIPPER"
GRIPPER_EPSILON = 1e-12
_PIKA_SDK_LOGGER_NAMES = ("pika", "pika.serial_comm", "pika.gripper", "pika.sense")


def suppress_pika_sdk_logging() -> None:
    """Silence noisy logs emitted by the vendor Pika SDK.

    The SDK calls logging.basicConfig() at import time and its serial reader
    reports parse noise at ERROR level. policy_runner reports gripper command
    outcomes through GripperDispatchResult instead.
    """
    names = set(_PIKA_SDK_LOGGER_NAMES)
    names.update(
        name
        for name in logging.root.manager.loggerDict
        if name == "pika" or name.startswith("pika.")
    )
    for name in names:
        logger = logging.getLogger(name)
        logger.handlers.clear()
        logger.propagate = False
        logger.disabled = True
        logger.setLevel(logging.CRITICAL + 1)


@dataclass(frozen=True)
class GripperCommand:
    arm: str
    value: float
    command_type: str = "delta"
    source: str = "flow_policy"

    def __post_init__(self) -> None:
        if self.arm not in {"left", "right"}:
            raise ValueError("gripper command arm must be left or right")
        if self.command_type not in {"delta", "target"}:
            raise ValueError("gripper command_type must be delta or target")

    @property
    def is_nonzero(self) -> bool:
        return abs(float(self.value)) > GRIPPER_EPSILON

    def to_dict(self) -> dict[str, Any]:
        return {
            "arm": self.arm,
            "value": float(self.value),
            "command_type": self.command_type,
            "source": self.source,
        }


@dataclass(frozen=True)
class GripperDispatchResult:
    command: GripperCommand
    accepted: bool
    sent_to_physical: bool
    dropped: bool
    reason: str


class GripperBackend(Protocol):
    supports_controller_simulation: bool

    def send(self, command: GripperCommand) -> GripperDispatchResult:
        ...


@dataclass
class NoopGripperBackend:
    """Dry-run gripper backend that records commands and never touches hardware."""

    reason: str = "noop_gripper_backend"
    supports_controller_simulation: bool = False
    commands: list[GripperCommand] = field(default_factory=list)

    def send(self, command: GripperCommand) -> GripperDispatchResult:
        self.commands.append(command)
        return GripperDispatchResult(
            command=command,
            accepted=False,
            sent_to_physical=False,
            dropped=True,
            reason=self.reason,
        )


# --- pika SDK telemetry framing ---------------------------------------------------------------
# Every POSITION_CTRL write truncates the telemetry frame in flight, which the vendor parser then
# turns into a 78 ms outage. Measured 2026-09-17 on this cell:
#
#   the write leaves a half frame in the stream -> `{\r\n"motor":{\r` + the next frame's `{`, i.e.
#   TWO unmatched '{'. serial_comm._find_json brace-matches from the FIRST '{' in the buffer, so it
#   can never balance again; it returns None for every subsequent read while perfectly good frames
#   pile up behind the poison, until `len(buffer) > 2000` makes the SDK discard the WHOLE buffer.
#   Net: ~3 ms of real corruption on the wire costs ~12 frames / 78 ms of jaw feedback.
#
#   stock parser, 20 Hz writes: 77.0 Hz frames, write->frame p95 83.4 ms
#   stock parser, 60 Hz writes: 13.7 Hz frames, write->frame p95 117.1 ms   <- gripper.max_hz
#   with this resync,   20 Hz:  155.3 Hz frames, write->frame p95 11.7 ms
#   with this resync,   60 Hz:  126.3 Hz frames, write->frame p95 11.6 ms
#   no writes (either):        ~167 Hz
#
# It matters because the jaw opening is the policy's only non-visual input on the griponly
# checkpoints, and the outage lands exactly on the grasp: measured over 31 rollout logs, the
# published sample is 3.5 ms old while the jaw is parked and 21 ms (p90 80, p99 141) while it moves.
#
# Fixed here rather than in the vendor tree: policy_runner already wraps the SDK from this side
# (suppress_pika_sdk_logging, _install_sample_clock), and the COLLECTION rig never writes to its
# Sense, so it never hits this and must not be perturbed.
_PIKA_FRAME_MARK = '{\r\n"motor":'
_PIKA_TRAILING_COMMA = re.compile(r",\s*([}\]])")


def _find_json_resync(self: Any) -> Any:
    """serial_comm._find_json, but anchored on a FRAME START and able to resync.

    Differences from the vendor version, both required:
      * scans from `_PIKA_FRAME_MARK`, not from any '{', so a half frame cannot capture the match;
      * when a candidate does not close (or does not parse), drops up to the NEXT frame start
        instead of keeping it -- one truncated frame costs one frame, not the whole buffer.
    """
    while True:
        start = self.buffer.find(_PIKA_FRAME_MARK)
        if start == -1:
            # No frame start at all: keep a tail in case one is straddling the read boundary.
            if len(self.buffer) > 4096:
                self.buffer = self.buffer[-512:]
            return None
        depth = 0
        end = -1
        for i in range(start, len(self.buffer)):
            char = self.buffer[i]
            if char == "{":
                depth += 1
            elif char == "}":
                depth -= 1
                if depth == 0:
                    end = i
                    break
        if end == -1:
            nxt = self.buffer.find(_PIKA_FRAME_MARK, start + 1)
            if nxt == -1:
                # Genuinely incomplete (still arriving) -> wait for more bytes.
                return None
            self.buffer = self.buffer[nxt:]  # truncated frame -> discard just it
            continue
        raw = self.buffer[start:end + 1]
        self.buffer = self.buffer[end + 1:]
        try:
            return json.loads(_PIKA_TRAILING_COMMA.sub(r"\1", raw))
        except Exception:  # noqa: BLE001 - malformed frame is data, not a control-flow error
            continue


def install_pika_frame_resync(gripper: Any) -> bool:
    """Swap the resync parser onto ONE gripper's serial reader. Returns False if the SDK
    shape does not match (age/feedback stays exactly as before; nothing is fabricated)."""
    comm = getattr(gripper, "serial_comm", None)
    if comm is None or not hasattr(comm, "buffer") or not hasattr(comm, "_find_json"):
        return False
    comm._find_json = types.MethodType(_find_json_resync, comm)
    return True


def _import_pika_gripper_class(sdk_path: str | None) -> type:
    """Import pika.gripper.Gripper, optionally from a configured SDK copy.

    The AgileX Pika SDK is not packaged on this PC's default sys.path; the
    copy from the SteamVR PC's conda env lives at gripper.pika_sdk_path
    (same convention as scripts/umi_gripper_follow.py).
    """
    if sdk_path and sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    try:
        from pika.gripper import Gripper  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError(
            f"pika.gripper import failed ({exc}); set gripper.pika_sdk_path to the "
            "directory containing the 'pika' package"
        ) from exc
    return Gripper


# --- pika SDK jaw geometry -------------------------------------------------------------------
# The COLLECTION rig writes `sense.get_gripper_distance()` into every dataset -- millimetres of jaw
# opening (pika_sdk/pika/sense.py:162), NOT a fraction of anything. The deploy runtime used to
# report `(rad - min_rad)/(max_rad - min_rad) * 100` instead, and the SDK linkage between motor
# angle and opening is a four-bar, so the two numbers are not proportional. Measured on the real
# grippers 2026-09-16 (tools/measure_gripper_units.py): the old percent OVER-reported the opening by
# up to +4.85 mm, peaking at a physical 23 mm jaw -- i.e. squarely inside the band the jaw crosses
# while grasping, and squarely inside the band where the policy's commanded z has its step. Both
# arms agreed to 0.05 mm, so this is geometry, not per-arm calibration drift.
#
# These reimplement pika_sdk/pika/gripper.py:220-237 rather than calling the SDK's own
# get_gripper_distance()/set_gripper_distance(), so the target integration, deadband, rate gate and
# rad-space clamp below stay in one place and keep working on a mock gripper in tests.
_SDK_MAX_ANGLE_RAD = (180.0 - 43.99) / 180.0 * math.pi


def _sdk_half_span_mm(angle_rad: float) -> float:
    """pika_sdk get_distance(angle): half of the jaw span, in mm."""
    a = _SDK_MAX_ANGLE_RAD - float(angle_rad)
    height = 0.0325 * math.sin(a)
    width_d = 0.0325 * math.cos(a)
    return (math.sqrt(0.058**2 - (height - 0.01456) ** 2) + width_d) * 1000.0


def sdk_jaw_mm(angle_rad: float) -> float:
    """pika_sdk get_gripper_distance(): jaw opening in mm, zero at motor angle 0.

    Valid because `_home_one` re-zeroes the motor on the CLOSED mechanical stop, so angle 0 is a
    physically closed jaw on both arms."""
    return (_sdk_half_span_mm(angle_rad) - _sdk_half_span_mm(0.0)) * 2.0


def sdk_jaw_mm_to_rad(jaw_mm: float, lo: float = 0.0, hi: float = _SDK_MAX_ANGLE_RAD) -> float:
    """Inverse of sdk_jaw_mm by bisection (the SDK's set_gripper_distance does the same search).
    Monotonic over [0, _SDK_MAX_ANGLE_RAD], so bisection is exact to the tolerance."""
    target = float(jaw_mm)
    if target <= sdk_jaw_mm(lo):
        return lo
    if target >= sdk_jaw_mm(hi):
        return hi
    for _ in range(64):
        mid = 0.5 * (lo + hi)
        if sdk_jaw_mm(mid) < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


GRIPPER_UNITS = ("sdk_mm", "motor_fraction")


class PikaSerialGripperBackend:
    """Drives the robot-mounted Pika grippers over local serial POSITION_CTRL.

    Policy gripper actions are per-step deltas in the DATASET's gripper units.
    Those units are `units="sdk_mm"` (default): millimetres of jaw opening as
    the pika SDK's get_gripper_distance() defines them, which is exactly what
    the collection rig recorded. `units="motor_fraction"` restores the previous
    behaviour (percent of [min_rad, max_rad]) for reproducing pre-2026-09-16
    runs; it is WRONG against every pika dataset -- see sdk_jaw_mm above.

    Deltas integrate onto a per-arm target seeded from the live motor position
    at connect(), clamped to [min_rad, max_rad]; 'target' commands set the
    opening absolutely. current_percent() reports the live opening in the same
    units for proprio feedback (the name is kept for its callers; under
    units="sdk_mm" it is millimetres, not percent).

    send() never raises into the control loop: serial errors are reported as
    dropped dispatch results.
    """

    def __init__(
        self,
        ports: Mapping[str, str],
        *,
        sdk_path: str | None = None,
        min_rad: float = 0.0,
        # Measured open mechanical stop 2026-09-16: left 1.6741 rad (96.90 mm), right 1.6687 rad
        # (96.65 mm). The previous 1.75 sat PAST the stop, so a full-open command just pressed on
        # it. 1.66 keeps a small margin below the tighter arm; the collection rig, bolted at
        # 74-76 mm, never demonstrates anything near it.
        max_rad: float = 1.66,
        units: str = "sdk_mm",
        deadband_rad: float = 0.005,
        max_hz: float = 60.0,
        supports_controller_simulation: bool = False,
        suppress_sdk_logs: bool = True,
        gripper_cls: type | None = None,
        clock: Callable[[], float] = time.monotonic,
        home_on_connect: bool = True,
        # Seconds the motor needs after enable() before it will ACT on a position command.
        # Measured 2026-09-17: with home_on_connect=False, a set_motor_angle issued straight
        # after connect() is accepted (returns True, the bytes go out) and silently ignored --
        # reproduced 2/2 on the right arm, while the left moved every time. The left only worked
        # because it is enabled FIRST and the right arm's own connect+enable (~0.5 s) served as
        # its settle. Homing used to hide this for both arms; the tracked stack runs
        # --no-home-on-connect, so the last-enabled arm lost its first command on every start.
        enable_settle_sec: float = 0.5,
        home_timeout_sec: float = 3.0,
        home_settle_eps_rad: float = 0.01,
        home_poll_sec: float = 0.05,
        # Measured 2026-09-16 on this cell: pressing the EMPTY jaw onto its own stop peaks at
        # ~203 mA (right arm; left stayed lower), while homing onto a 12 mm shank sat at 580-720 mA.
        # 400 sits between them with ~2x margin either way. 200 was tried first and false-positived
        # on the empty right arm by 3 mA.
        home_max_current_ma: float = 400.0,
    ) -> None:
        if max_rad <= min_rad:
            raise ValueError("gripper max_rad must be greater than min_rad")
        if units not in GRIPPER_UNITS:
            raise ValueError(f"gripper units must be one of {GRIPPER_UNITS}, got {units!r}")
        if deadband_rad < 0.0:
            raise ValueError("gripper deadband_rad must be non-negative")
        self.ports = {str(arm): str(port) for arm, port in ports.items()}
        for arm in self.ports:
            if arm not in {"left", "right"}:
                raise ValueError("gripper port arms must be left or right")
        self.sdk_path = sdk_path
        self.min_rad = float(min_rad)
        self.max_rad = float(max_rad)
        self.units = str(units)
        self.deadband_rad = float(deadband_rad)
        self.min_period_sec = 1.0 / float(max_hz) if max_hz > 0 else 0.0
        self.supports_controller_simulation = bool(supports_controller_simulation)
        self.suppress_sdk_logs = bool(suppress_sdk_logs)
        self.home_on_connect = bool(home_on_connect)
        self.enable_settle_sec = max(0.0, float(enable_settle_sec))
        self.home_timeout_sec = float(home_timeout_sec)
        self.home_settle_eps_rad = float(home_settle_eps_rad)
        self.home_poll_sec = float(home_poll_sec)
        self.home_max_current_ma = float(home_max_current_ma)
        self._gripper_cls = gripper_cls
        self._clock = clock
        self._grippers: dict[str, Any] = {}
        self._targets: dict[str, float] = {}
        self._last_sent: dict[str, tuple[float, float]] = {}
        # Arrival time of the most recent pika telemetry frame per arm, stamped
        # by _install_sample_clock so consumers can report a real sensor age.
        self._sample_time: dict[str, float] = {}
        # Arms whose homing ended pressing on something; the reported scale is not trustworthy.
        self._home_suspect: dict[str, float] = {}

    def connect(self) -> "PikaSerialGripperBackend":
        if self.suppress_sdk_logs:
            suppress_pika_sdk_logging()
        if self._gripper_cls is None:
            for arm, port in self.ports.items():
                if not os.path.exists(port):
                    self.close()
                    raise RuntimeError(f"pika gripper {arm} serial port not found: {port}")
        last_enable = self._clock()
        gripper_cls = self._gripper_cls or _import_pika_gripper_class(self.sdk_path)
        if self.suppress_sdk_logs:
            suppress_pika_sdk_logging()
        for arm, port in self.ports.items():
            gripper = gripper_cls(port=port)
            if not gripper.connect():
                self.close()
                detail = port
                if os.path.islink(port):
                    detail = f"{port} -> {os.path.realpath(port)}"
                raise RuntimeError(f"pika gripper {arm} connect failed on {detail}")
            if not gripper.enable():
                self.close()
                raise RuntimeError(f"pika gripper {arm} enable failed on {port}")
            last_enable = self._clock()
            install_pika_frame_resync(gripper)
            self._grippers[arm] = gripper
            self._targets[arm] = self._seed_target(gripper)
            self._install_sample_clock(arm, gripper)
        if self.home_on_connect and self._grippers:
            self._home_all_concurrent()   # its own motion already covers the settle
        elif self._grippers:
            self._await_enable_settle(last_enable)
        return self

    def _await_enable_settle(self, last_enable: float) -> None:
        """Hold off the first command until every motor will actually act on it.

        Only the time since the LAST enable matters: the arms are enabled in sequence, so the
        earlier ones have already settled while the final one has not."""
        remaining = self.enable_settle_sec - (self._clock() - last_enable)
        if remaining > 0.0:
            time.sleep(remaining)

    def _install_sample_clock(self, arm: str, gripper: Any) -> bool:
        """Stamp the arrival time of each pika telemetry frame for `arm`.

        get_motor_position() is a lock+dict read off the SDK's reader thread, so
        the caller cannot tell a fresh sample from one the device sent 50 ms ago.
        Measured 2026-08-19 from rollout logs: while the jaw is actually moving,
        35% of consecutive 30 Hz reads repeat the previous value -- the telemetry
        lands at ~18.5 Hz, so the "measured" percent handed to the policy (and
        logged as gripper_meas_pct) is 27 ms old on average and ~54 ms at worst.
        That age was previously invisible: the only age in the log is the
        message's publish->receive time, which is ~0.05 ms and reads as instant.

        Wraps the SDK's own serial callback rather than polling, so the stamp is
        the frame's arrival, not our observation of it. Only 'motor' frames carry
        Position, so only those advance the clock. Returns False (age stays
        unavailable, never fabricated) when the SDK shape does not match.
        """
        comm = getattr(gripper, "serial_comm", None)
        inner = getattr(comm, "callback", None)
        if comm is None or not callable(inner):
            return False
        clock = self._clock
        stamps = self._sample_time

        def _stamped(data: Any, _inner: Any = inner, _arm: str = arm) -> Any:
            result = _inner(data)
            try:
                if isinstance(data, Mapping) and "motor" in data:
                    stamps[_arm] = clock()
            except Exception:  # noqa: BLE001 - never break the SDK reader thread
                pass
            return result

        comm.callback = _stamped
        return True

    def sample_age_sec(self, arm: str) -> float | None:
        """Seconds since the pika telemetry frame backing current_percent(arm).

        None when no frame has been stamped (SDK shape unrecognised, sim backend,
        or nothing received yet) -- the consumers log null rather than 0 so a
        missing measurement never masquerades as a fresh one."""
        stamp = self._sample_time.get(arm)
        if stamp is None:
            return None
        age = float(self._clock()) - float(stamp)
        return age if age >= 0.0 else 0.0

    def _home_all_concurrent(self) -> None:
        """Reference every gripper to its CLOSED mechanical stop and re-zero there,
        so absolute-angle commands (set_motor_angle) map to the SAME physical
        opening on both (identical) grippers. Without this the motor zero is the
        arbitrary power-on position, so a 100%-open command lands at a different
        physical opening per arm (observed: right ~39% vs left ~70% open).

        Both arms home in parallel threads (independent serial ports) so the whole
        step takes one gripper's homing time, not the sum. Failures are logged and
        non-fatal: a connected+enabled gripper still works, just uncalibrated."""
        threads = []
        for arm, gripper in self._grippers.items():
            t = threading.Thread(
                target=self._home_one, args=(arm, gripper), name=f"gripper-home-{arm}", daemon=True
            )
            t.start()
            threads.append(t)
        # Join with margin over the per-arm settle timeout so a stuck arm can't
        # hang startup forever.
        for t in threads:
            t.join(timeout=self.home_timeout_sec + 1.0)

    def _home_one(self, arm: str, gripper: Any) -> None:
        try:
            # 1. Drive toward the closed stop (min_rad). set_motor_angle clamps
            #    rad<0 to 0, so commanding min_rad bottoms the jaw on the stop.
            gripper.set_motor_angle(self.min_rad)
            # 2. Wait until the jaw stops moving (settled against the stop) so the
            #    re-zero references the true mechanical closed position.
            self._wait_until_settled(gripper)
            # 3. Define the closed stop as zero. Subsequent set_motor_angle(rad) is
            #    now consistent across both grippers; max_rad == true full open.
            # A homing that ended against an OBJECT rather than the jaw's own stop silently
            # poisons the whole session: set_zero() then defines "closed" at the object's width,
            # every reported opening is shifted by it, and -- because set_motor_angle clamps rad<0
            # to 0 -- the gripper can never squeeze past that point no matter what the policy
            # commands. Measured 2026-09-16: with a 12 mm shank in the jaws both arms homed to a
            # reported 1.3-1.6 mm at -580/-720 mA and stalled at the command floor. The motor
            # current is the only way to tell the two cases apart, so check it rather than trusting
            # that the jaws were empty.
            current = self.motor_current_ma(arm)
            if current is not None and abs(current) > self.home_max_current_ma:
                print(
                    f"[gripper] WARN home {arm}: settled at {abs(current):.0f} mA "
                    f"(> {self.home_max_current_ma:.0f}), i.e. pressing on SOMETHING, not on its own "
                    "stop. The zero is being set at that object's width: every opening this session "
                    "is offset and the jaw cannot squeeze past it. Clear the jaws and re-home.",
                    file=sys.stderr,
                    flush=True,
                )
                self._home_suspect[arm] = float(current)
            else:
                self._home_suspect.pop(arm, None)
            if hasattr(gripper, "set_zero"):
                gripper.set_zero()
            self._targets[arm] = self.min_rad
            self._last_sent.pop(arm, None)
            print(f"[gripper] homed {arm}: closed-stop zeroed", file=sys.stderr, flush=True)
        except Exception as exc:  # noqa: BLE001 - homing must not crash startup
            print(
                f"[gripper] WARN home {arm} failed ({type(exc).__name__}: {exc}); "
                "gripper left uncalibrated (open may be partial)",
                file=sys.stderr,
                flush=True,
            )

    def _wait_until_settled(self, gripper: Any) -> None:
        """Poll the motor position until two consecutive reads agree within
        home_settle_eps_rad (jaw stopped) or home_timeout_sec elapses."""
        deadline = self._clock() + self.home_timeout_sec
        last: float | None = None
        while self._clock() < deadline:
            time.sleep(self.home_poll_sec)
            try:
                pos = float(gripper.get_motor_position())
            except Exception:
                return  # no feedback -> fall back to the time already spent moving
            if last is not None and abs(pos - last) < self.home_settle_eps_rad:
                return
            last = pos

    def _seed_target(self, gripper: Any) -> float:
        try:
            position = float(gripper.get_motor_position())
        except Exception:
            position = self.min_rad
        return self._clamp(position)

    def _clamp(self, value: float) -> float:
        return max(self.min_rad, min(self.max_rad, float(value)))

    def _units_to_rad(self, value: float) -> float:
        if self.units == "sdk_mm":
            return sdk_jaw_mm_to_rad(value)
        return self.min_rad + (self.max_rad - self.min_rad) * float(value) / 100.0

    def _rad_to_units(self, rad: float) -> float:
        if self.units == "sdk_mm":
            return sdk_jaw_mm(rad)
        return (float(rad) - self.min_rad) / (self.max_rad - self.min_rad) * 100.0

    def current_percent(self, arm: str) -> float | None:
        """Live jaw opening in DATASET units (proprio feedback).

        Millimetres under the default units="sdk_mm"; percent of [min_rad, max_rad] under
        units="motor_fraction". The method name predates the unit fix and is kept for its callers
        (gripper_server, the rollout step log's gripper_meas_pct / gripper_proprio_pct)."""
        gripper = self._grippers.get(arm)
        if gripper is None:
            return None
        try:
            return self._rad_to_units(float(gripper.get_motor_position()))
        except Exception:
            return None

    def motor_current_ma(self, arm: str) -> float | None:
        """Live motor phase current in mA (pika SDK `get_motor_current`), negative while squeezing.

        This is the ONLY grip-effort signal in the cell: the collection rig cannot record force at
        all (the Pika Sense is a passive handheld -- no motor, so no current; confirmed against its
        whole SDK surface, the vendor API_Doc and the manual's Output Data row), so any calibration
        of "how hard is this grip" has to come from the robot side."""
        gripper = self._grippers.get(arm)
        if gripper is None:
            return None
        try:
            return float(gripper.get_motor_current())
        except Exception:  # noqa: BLE001 - telemetry must never raise into the loop
            return None

    def target_units(self, arm: str) -> float | None:
        """The integrated setpoint this backend is actually HOLDING, in dataset units.

        Not the same as the number the caller sent: a command past the open mechanical stop is
        clamped to the stop. Consumers that compare a target against the measured opening
        (gripper_server's `moving` flag) must use this one, or a command that cannot be reached
        latches "moving" forever."""
        rad = self._targets.get(arm)
        return None if rad is None else self._rad_to_units(rad)

    def send(self, command: GripperCommand) -> GripperDispatchResult:
        gripper = self._grippers.get(command.arm)
        if gripper is None:
            return self._result(command, accepted=False, sent=False, dropped=True, reason="gripper_arm_not_connected")
        # Command values are in dataset units; motors take rad. The delta is applied in UNITS
        # space and converted once, because under units="sdk_mm" the unit->rad map is a four-bar
        # and a delta does NOT convert to a fixed number of radians. (Under "motor_fraction" the
        # map is linear, so this is algebraically the same as the old fixed delta_rad.)
        if command.command_type == "target":
            target_units = float(command.value)
        else:
            current_units = self._rad_to_units(self._targets.get(command.arm, self.min_rad))
            target_units = current_units + float(command.value)
        target = self._clamp(self._units_to_rad(target_units))
        # The integrated target always advances; deadband/rate gates only skip
        # the serial write so small deltas accumulate instead of being lost.
        self._targets[command.arm] = target
        now = self._clock()
        last = self._last_sent.get(command.arm)
        if last is not None:
            last_time, last_rad = last
            if self.min_period_sec > 0.0 and now - last_time < self.min_period_sec:
                # Held, not lost: the integrated target carries to the next send.
                return self._result(command, accepted=True, sent=False, dropped=False, reason="gripper_rate_limited")
            if abs(target - last_rad) < self.deadband_rad:
                return self._result(command, accepted=True, sent=False, dropped=False, reason="gripper_deadband_hold")
        try:
            ok = bool(gripper.set_motor_angle(target))
        except Exception as exc:
            return self._result(command, accepted=False, sent=False, dropped=True, reason=f"gripper_serial_error:{exc}")
        if not ok:
            return self._result(command, accepted=False, sent=False, dropped=True, reason="gripper_command_rejected")
        self._last_sent[command.arm] = (now, target)
        return self._result(command, accepted=True, sent=True, dropped=False, reason="gripper_position_sent")

    def close(self) -> None:
        for gripper in self._grippers.values():
            for method_name in ("disable", "disconnect"):
                method = getattr(gripper, method_name, None)
                if method is None:
                    continue
                try:
                    method()
                except Exception:
                    pass
        self._grippers = {}

    @staticmethod
    def _result(
        command: GripperCommand, *, accepted: bool, sent: bool, dropped: bool, reason: str
    ) -> GripperDispatchResult:
        return GripperDispatchResult(
            command=command,
            accepted=accepted,
            sent_to_physical=sent,
            dropped=dropped,
            reason=reason,
        )


@dataclass
class GripperRuntime:
    rollout_mode: str
    allow_real_gripper_motion: bool = False
    backend: GripperBackend = field(default_factory=NoopGripperBackend)
    env: Mapping[str, str] | None = None
    command_count: int = 0
    dropped_count: int = 0
    results: list[GripperDispatchResult] = field(default_factory=list)

    def dispatch(
        self,
        commands: list[GripperCommand] | tuple[GripperCommand, ...],
        *,
        concurrent: bool = False,
    ) -> list[GripperDispatchResult]:
        # Gate first (cheap, in order). The slow part is backend.send -> the
        # blocking per-arm serial write; with concurrent=True those run in
        # parallel threads so both grippers move at once instead of
        # left-then-right (each arm is an independent serial port, so concurrent
        # writes are safe and touch disjoint backend state). Default stays
        # sequential for the per-tick policy path.
        plan: list[tuple[str, Any]] = []  # ("result", res) | ("send", command)
        for command in commands:
            # Idle-suppression is DELTA-only: a ~zero delta means "no change", skip
            # it. For a "target" command, value 0 is a legitimate goal (fully
            # closed), so never skip it here -- the backend deadband/rate gate
            # decides whether the integrated target needs an actual serial write.
            if command.command_type == "delta" and not command.is_nonzero:
                continue
            self.command_count += 1
            decision = self._gate(command)
            if decision is not None:
                self.dropped_count += 1
                plan.append(("result", decision))
                continue
            plan.append(("send", command))

        send_cmds = [item for kind, item in plan if kind == "send"]
        sent: dict[int, GripperDispatchResult] = {}
        if concurrent and len(send_cmds) > 1:
            threads = []
            for cmd in send_cmds:
                t = threading.Thread(
                    target=lambda c=cmd: sent.__setitem__(id(c), self.backend.send(c))
                )
                t.start()
                threads.append(t)
            for t in threads:
                t.join()
        else:
            for cmd in send_cmds:
                sent[id(cmd)] = self.backend.send(cmd)

        results: list[GripperDispatchResult] = []
        for kind, item in plan:
            result = item if kind == "result" else sent[id(item)]
            if kind == "send" and result.dropped:
                self.dropped_count += 1
            self.results.append(result)
            results.append(result)
        return results

    @property
    def latest_reason(self) -> str | None:
        if not self.results:
            return None
        return self.results[-1].reason

    def _gate(self, command: GripperCommand) -> GripperDispatchResult | None:
        mode = str(self.rollout_mode or "").strip().lower()
        if mode == "real_policy":
            if not self.allow_real_gripper_motion:
                return self._drop(command, "real_gripper_config_not_allowed")
            if self._env().get(REAL_GRIPPER_ENV) != "1":
                return self._drop(command, "real_gripper_env_missing")
            return None
        if mode == "controller_sim":
            if bool(getattr(self.backend, "supports_controller_simulation", False)):
                return None
            return self._drop(command, "controller_sim_gripper_logged_noop")
        if mode in {"offline_eval", "sim_dryrun", "real_readonly"}:
            return self._drop(command, f"{mode}_gripper_logged_noop")
        return self._drop(command, "gripper_backend_not_configured")

    def _drop(self, command: GripperCommand, reason: str) -> GripperDispatchResult:
        return GripperDispatchResult(
            command=command,
            accepted=False,
            sent_to_physical=False,
            dropped=True,
            reason=reason,
        )

    def _env(self) -> Mapping[str, str]:
        return os.environ if self.env is None else self.env


def gripper_commands_from_flow_step(
    step: Any,
    *,
    arm_mask: Any,
    command_type: str = "delta",
    source: str = "flow_policy",
) -> list[GripperCommand]:
    values = list(step)
    mask = list(arm_mask)
    commands: list[GripperCommand] = []
    if len(values) >= 7 and len(mask) >= 1 and float(mask[0]) > 0.0:
        commands.append(
            GripperCommand(
                arm="left",
                value=float(values[6]),
                command_type=command_type,
                source=source,
            )
        )
    if len(values) >= 14 and len(mask) >= 2 and float(mask[1]) > 0.0:
        commands.append(
            GripperCommand(
                arm="right",
                value=float(values[13]),
                command_type=command_type,
                source=source,
            )
        )
    return commands
