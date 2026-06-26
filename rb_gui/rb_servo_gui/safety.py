from __future__ import annotations

from dataclasses import dataclass
import math
import os
import re
from pathlib import Path
from typing import Literal

from .command_client import CommandClient
from .models import StateSnapshot
from .state_receiver import StateStore


_FLOOR_Z_LINE_RE = re.compile(r"^(\s*z_min_m\s*:\s*)([0-9.eE+-]+)(.*)$")


def persist_floor_z_to_config(config_path: str | Path, floor_z_m: float) -> tuple[bool, str]:
    """Rewrite floor_constraint.z_min_m in the server config yaml (text-level,
    comments preserved) so a viser "Send floor z" survives a stack restart.

    Only the FIRST z_min_m line inside the floor_constraint block is touched.
    Returns (ok, short message for the GUI status field)."""
    path = Path(config_path)
    try:
        lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    except OSError as exc:
        return False, f"yaml unchanged: {type(exc).__name__}: {exc}"

    in_floor_block = False
    block_indent = -1
    for index, line in enumerate(lines):
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if stripped.startswith("floor_constraint:"):
            in_floor_block = True
            block_indent = indent
            continue
        if not in_floor_block:
            continue
        # Leaving the block: a non-empty, non-comment line at or above the
        # block's own indentation level.
        if stripped and not stripped.startswith("#") and indent <= block_indent:
            break
        match = _FLOOR_Z_LINE_RE.match(line.rstrip("\n"))
        if match:
            newline = "\n" if line.endswith("\n") else ""
            lines[index] = f"{match.group(1)}{floor_z_m:.4f}{match.group(3)}{newline}"
            try:
                path.write_text("".join(lines), encoding="utf-8")
            except OSError as exc:
                return False, f"yaml unchanged: {type(exc).__name__}: {exc}"
            return True, f"saved z_min_m={floor_z_m:.4f} to {path.name}"
    return False, f"yaml unchanged: floor_constraint.z_min_m not found in {path.name}"


_ROI_MIN_LINE_RE = re.compile(r"^(\s*min_m\s*:\s*)\[[^\]]*\](.*)$")
_ROI_MAX_LINE_RE = re.compile(r"^(\s*max_m\s*:\s*)\[[^\]]*\](.*)$")


def _fmt_vec3(values: tuple[float, float, float]) -> str:
    return "[" + ", ".join(f"{float(v):.3f}" for v in values) + "]"


def persist_roi_bounds_to_config(
    config_path: str | Path,
    roi_min_m: tuple[float, float, float],
    roi_max_m: tuple[float, float, float],
) -> tuple[bool, str]:
    """Rewrite roi_box.min_m / roi_box.max_m in the server config yaml (text-level,
    comments preserved) so a viser "Send ROI box" survives a stack restart.

    Only the FIRST min_m / max_m lines inside the roi_box block are touched.
    Returns (ok, short message for the GUI status field)."""
    path = Path(config_path)
    try:
        lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    except OSError as exc:
        return False, f"yaml unchanged: {type(exc).__name__}: {exc}"

    in_roi_block = False
    block_indent = -1
    wrote_min = False
    wrote_max = False
    for index, line in enumerate(lines):
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if stripped.startswith("roi_box:"):
            in_roi_block = True
            block_indent = indent
            continue
        if not in_roi_block:
            continue
        if stripped and not stripped.startswith("#") and indent <= block_indent:
            break  # left the roi_box block
        newline = "\n" if line.endswith("\n") else ""
        if not wrote_min:
            m = _ROI_MIN_LINE_RE.match(line.rstrip("\n"))
            if m:
                lines[index] = f"{m.group(1)}{_fmt_vec3(roi_min_m)}{m.group(2)}{newline}"
                wrote_min = True
                continue
        if not wrote_max:
            m = _ROI_MAX_LINE_RE.match(line.rstrip("\n"))
            if m:
                lines[index] = f"{m.group(1)}{_fmt_vec3(roi_max_m)}{m.group(2)}{newline}"
                wrote_max = True
                continue
    if not (wrote_min and wrote_max):
        return False, f"yaml unchanged: roi_box.min_m/max_m not found in {path.name}"
    try:
        path.write_text("".join(lines), encoding="utf-8")
    except OSError as exc:
        return False, f"yaml unchanged: {type(exc).__name__}: {exc}"
    return True, f"saved roi_box bounds to {path.name}"

Mode = Literal["mock", "simulation", "real"]
Backend = Literal["mock", "simulator", "rbpodo", "unknown"]

_VALID_MODES: set[str] = {"mock", "simulation", "real"}
_VALID_BACKENDS: set[str] = {"mock", "simulator", "rbpodo", "unknown"}
_SIMULATOR_ALIASES: set[str] = {"rbsim_local", "rbsim", "rb_simulator"}


@dataclass(frozen=True)
class ObservedModeBackend:
    mode: Mode
    backend: Backend
    warnings: tuple[str, ...] = ()


def normalize_observed_mode_backend(mode: str | None, backend: str | None = None) -> ObservedModeBackend:
    raw_mode = (mode or "mock").strip()
    raw_backend = (backend or "").strip()
    warnings: list[str] = []

    if raw_mode in _SIMULATOR_ALIASES:
        normalized_mode: Mode = "simulation"
        warnings.append(f"deprecated observed mode {raw_mode!r} normalized to 'simulation'")
    elif raw_mode in _VALID_MODES:
        normalized_mode = raw_mode  # type: ignore[assignment]
    else:
        normalized_mode = "mock"
        warnings.append(f"invalid observed mode {raw_mode!r} normalized to 'mock'")

    if raw_backend in _SIMULATOR_ALIASES:
        normalized_backend: Backend = "simulator"
        warnings.append(f"deprecated observed backend {raw_backend!r} normalized to 'simulator'")
    elif raw_backend in _VALID_BACKENDS:
        normalized_backend = raw_backend  # type: ignore[assignment]
    elif raw_backend:
        normalized_backend = "unknown"
        warnings.append(f"unknown observed backend {raw_backend!r} displayed as 'unknown'")
    elif normalized_mode == "simulation":
        normalized_backend = "simulator"
    elif normalized_mode == "real":
        normalized_backend = "rbpodo"
    else:
        normalized_backend = "mock"

    return ObservedModeBackend(normalized_mode, normalized_backend, tuple(warnings))


@dataclass(frozen=True)
class Readiness:
    configured: bool = True
    running: bool = False
    connected: bool = False
    ready: bool = False
    fault: bool = False
    no_go_reason: str = ""
    cartesian_available: bool | None = None
    cartesian_no_go_reason: str = ""


class OperatorSafety:
    lifecycle_modes = {"ArmMotion", "DisarmMotion", "Hold", "EmergencyStop", "ResetFault"}
    motion_modes = {
        "JointTarget",
        "TcpPoseTarget",
        "TcpLinearMove",
    }
    # Actions that are always allowed (no motion gate): emergency/stop + fault reset.
    _non_motion_actions = {"EmergencyStop", "Hold", "ResetFault"}
    _latched_motion_states = {"FaultLatched", "EmergencyLatched"}

    def __init__(
        self,
        store: StateStore,
        command_client: CommandClient,
        *,
        desired_mode: Mode | str = "mock",
        observed_server_mode: Mode | str = "mock",
        observed_backend: Backend | str | None = None,
        ops_available: bool = False,
        max_jog_step_deg: float = 2.0,
        max_tcp_linear_step_m: float = 0.005,
        max_tcp_angular_step_rad: float = 0.02,
        command_timeout_sec: float = 0.2,
        init_left_joint_deg: tuple[float, ...] | None = None,
        init_right_joint_deg: tuple[float, ...] | None = None,
        init_motion_timeout_sec: float = 10.0,
    ) -> None:
        desired = normalize_observed_mode_backend(str(desired_mode), None)
        observed = normalize_observed_mode_backend(str(observed_server_mode), None if observed_backend is None else str(observed_backend))
        self.store = store
        self.command_client = command_client
        self.desired_mode = desired.mode
        self.observed_server_mode = observed.mode
        self.observed_backend = observed.backend
        self.config_warnings = desired.warnings + observed.warnings
        self.ops_available = ops_available
        self.max_jog_step_deg = float(max_jog_step_deg)
        self.max_tcp_linear_step_m = float(max_tcp_linear_step_m)
        self.max_tcp_angular_step_rad = float(max_tcp_angular_step_rad)
        self.command_timeout_sec = float(command_timeout_sec)
        self.init_left_joint_deg = self._validated_joint6(init_left_joint_deg)
        self.init_right_joint_deg = self._validated_joint6(init_right_joint_deg)
        self.init_motion_timeout_sec = float(init_motion_timeout_sec)
        self.status_message = "; ".join(self.config_warnings) if self.config_warnings else "starting"
        self.recording_intent = False
        self.last_tcp_command = "none"

    def set_desired_mode(self, mode: Mode | str) -> None:
        normalized = normalize_observed_mode_backend(str(mode), None)
        self.desired_mode = normalized.mode
        if not self.ops_available:
            self.status_message = f"desired {self.desired_mode}; running process is not reconfigured without ops surface"

    def latest_valid(self) -> StateSnapshot | None:
        latest = self.store.latest()
        if latest is None or self.store.is_stale():
            return None
        return latest

    @staticmethod
    def _validated_joint6(values: tuple[float, ...] | None) -> tuple[float, ...] | None:
        if values is None or len(values) != 6:
            return None
        try:
            parsed = tuple(float(value) for value in values)
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in parsed):
            return None
        return parsed

    def readiness(self) -> Readiness:
        # Derived purely from the live server state stream: the server is the
        # sole motion authority, so the GUI mirrors what it reports instead of
        # consulting launch-time env flags. cartesian_available reflects the same
        # state-derived TCP gate the buttons use.
        latest = self.latest_valid()
        if latest is None:
            return Readiness(configured=True, no_go_reason="state stream missing or stale")
        connected = latest.left.connection_state == "Connected" and latest.right.connection_state == "Connected"
        joint_valid = latest.left.has_valid_joint_state and latest.right.has_valid_joint_state
        backend_fault = bool(latest.left.error_code or latest.right.error_code or latest.safety_verdict in {"Fault", "BackendFault", "RobotFault"})
        server_fault = latest.fault_latched or latest.motion_state in self._latched_motion_states
        cartesian_reason = self.tcp_command_disabled_reason()
        cartesian_kwargs = {
            "cartesian_available": cartesian_reason is None,
            "cartesian_no_go_reason": cartesian_reason or "",
        }
        if server_fault:
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=True, no_go_reason="server fault latched", **cartesian_kwargs)
        if backend_fault:
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=True, no_go_reason="robot/backend fault", **cartesian_kwargs)
        if not joint_valid:
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=False, no_go_reason="joint state invalid", **cartesian_kwargs)
        return Readiness(configured=True, running=True, connected=connected, ready=connected, fault=False, **cartesian_kwargs)

    def blocked_reason(self, action: str) -> str | None:
        # State-derived, server-as-authority gate. Real/sim execution gating and
        # env readiness retired: mode mismatch never blocks; only a missing/stale
        # state stream, invalid joints, or a latched fault stop a motion command.
        latest = self.latest_valid()
        if latest is None:
            return "state stream missing or stale"
        motion = action not in self._non_motion_actions
        if motion and (not latest.left.has_valid_joint_state or not latest.right.has_valid_joint_state):
            return "joint state invalid"
        if motion and latest.motion_state in self._latched_motion_states:
            return f"motion latched ({latest.motion_state}); reset fault before motion"
        if motion and latest.fault_latched:
            return f"fault latched: {latest.fault_reason or 'unknown'}; reset and arm before motion"
        return None

    @staticmethod
    def _arm_has_tcp_pose(latest: StateSnapshot, arm: Literal["left", "right"]) -> bool:
        arm_state = latest.left if arm == "left" else latest.right
        return bool(arm_state.has_valid_tcp_pose and arm_state.tcp_stand is not None and not arm_state.tcp_deferred)

    @staticmethod
    def _arm_cartesian_reason(latest: StateSnapshot, arm: Literal["left", "right"]) -> str | None:
        """None when this arm can take a Cartesian command, else the reason.

        Availability comes straight from the server. The plain Cartesian gate
        (cartesian_available) is authoritative for real and simulation. The
        controller-simulation streaming flag is meaningful ONLY in the rbpodo
        controller-simulation carve-out — it is always False off that path, so
        consulting it everywhere would wrongly block real motion (where the
        server reports cartesian_available=True with the controller-sim flag
        False). An unknown (None) gate is treated as available — the server
        rejects the command itself if Cartesian is truly closed."""
        arm_state = latest.left if arm == "left" else latest.right
        if not (arm_state.has_valid_tcp_pose and arm_state.tcp_stand is not None and not arm_state.tcp_deferred):
            return f"{arm} FK/TCP pose unavailable"
        if arm_state.is_controller_simulation:
            available = arm_state.controller_simulation_cartesian_available
            if available is None:
                available = arm_state.cartesian_available
        else:
            available = arm_state.cartesian_available
        if available is False:
            return arm_state.cartesian_unavailable_reason or f"{arm} Cartesian unavailable (server gate)"
        return None

    def tcp_command_disabled_reason(self, arm: Literal["left", "right"] | None = None) -> str | None:
        # RB_GUI_ENABLE_TCP_POSE_COMMANDS / mode / backend / env-readiness locks
        # retired: TCP pose commands are available in every run mode. Availability
        # is derived from the live per-arm server Cartesian gate; the server stays
        # the authority (CartesianUnavailable, safety clamps, fault latch).
        reason = self.blocked_reason("TcpPoseTarget")
        if reason:
            return reason
        latest = self.latest_valid()
        if latest is None:
            return "state stream missing or stale"
        if arm is not None:
            return self._arm_cartesian_reason(latest, arm)
        left_reason = self._arm_cartesian_reason(latest, "left")
        right_reason = self._arm_cartesian_reason(latest, "right")
        if left_reason is None or right_reason is None:
            return None
        return left_reason

    def control_disabled_states(self) -> dict[str, bool]:
        """Return visual disabled-state for controls.

        Callback-level blocking remains the authority; this method keeps the GUI
        honest by disabling controls whenever an action would be rejected. Keys
        cover every motion tab so a control is never live-but-dead.
        """
        tcp_reason = self.tcp_command_disabled_reason()
        states: dict[str, bool] = {
            "jog": self.blocked_reason("JointTarget") is not None,
            "init_motion": self.init_motion_disabled_reason() is not None,
            "tcp_pose": tcp_reason is not None,
            "tcp_linear": tcp_reason is not None,
        }
        for mode in self.lifecycle_modes:
            states[f"lifecycle:{mode}"] = self.blocked_reason(mode) is not None
        return states

    def control_disabled_reasons(self) -> dict[str, str | None]:
        """Block reason per control key (None when allowed).

        Mirrors control_disabled_states but carries the human-readable reason so
        button-group tabs (which viser cannot grey) can surface it proactively in
        their status line instead of only after a rejected click."""
        tcp_reason = self.tcp_command_disabled_reason()
        return {
            "jog": self.blocked_reason("JointTarget"),
            "init_motion": self.init_motion_disabled_reason(),
            "tcp_pose": tcp_reason,
            "tcp_linear": tcp_reason,
        }

    def send_lifecycle(self, mode: str) -> tuple[bool, str]:
        if mode not in self.lifecycle_modes:
            return False, f"unsupported lifecycle mode {mode}"
        reason = self.blocked_reason(mode)
        if reason:
            return False, reason
        self.command_client.send_lifecycle(mode, timeout_sec=self.command_timeout_sec)
        return True, f"sent {mode}"

    def send_set_floor_z(self, floor_z_m: float) -> tuple[bool, str]:
        # Non-motion, leaseless safety adjustment: no motion-block check, but
        # require a live state stream reporting the constraint enabled so the
        # operator sees the applied value (the server bounds-checks the request).
        latest = self.latest_valid()
        if latest is None:
            return False, "state stream missing or stale"
        floor = latest.floor_constraint
        if floor is None or not bool(floor.get("enabled", False)):
            return False, "floor constraint disabled on server"
        try:
            self.command_client.send_set_safety_floor_z(
                float(floor_z_m), timeout_sec=self.command_timeout_sec
            )
        except ValueError as exc:
            return False, str(exc)
        sent = f"sent SetSafetyFloorZ {float(floor_z_m) * 1000:.1f}mm"
        # Persist to the server config yaml (explicit user request: a viser
        # "Send floor z" is also the new startup default). Requires the stack
        # launcher to expose the config path; a send without it stays
        # runtime-only and says so.
        config_path = os.environ.get("RB_GUI_SERVER_CONFIG_PATH", "").strip()
        if not config_path:
            return True, f"{sent} (runtime only: RB_GUI_SERVER_CONFIG_PATH not set)"
        _, save_message = persist_floor_z_to_config(config_path, float(floor_z_m))
        return True, f"{sent} ({save_message})"

    def send_set_floor_enabled(self, enabled: bool) -> tuple[bool, str]:
        # Non-motion, leaseless: runtime enforce on/off for the stand floor. Requires
        # a live state stream; the server honours enable=true only when the floor is
        # opted in at config (floor_constraint.enable=true).
        latest = self.latest_valid()
        if latest is None:
            return False, "state stream missing or stale"
        try:
            self.command_client.send_set_safety_floor_enabled(
                bool(enabled), timeout_sec=self.command_timeout_sec
            )
        except ValueError as exc:
            return False, str(exc)
        return True, f"sent SetSafetyFloorEnabled {'ON' if enabled else 'OFF'}"

    def send_set_roi_bounds(
        self,
        roi_min_m: tuple[float, float, float],
        roi_max_m: tuple[float, float, float],
    ) -> tuple[bool, str]:
        # Non-motion, leaseless safety adjustment (mirror of send_set_floor_z):
        # require a live state stream reporting roi_box enabled; the server
        # bounds-checks the request against its per-axis runtime envelope.
        latest = self.latest_valid()
        if latest is None:
            return False, "state stream missing or stale"
        roi = latest.roi_box
        if roi is None or not bool(roi.get("enabled", False)):
            return False, "roi box disabled on server"
        try:
            self.command_client.send_set_safety_roi_bounds(
                roi_min_m, roi_max_m, timeout_sec=self.command_timeout_sec
            )
        except ValueError as exc:
            return False, str(exc)
        sent = (
            "sent SetSafetyRoiBounds "
            + " ".join(
                f"{a}[{roi_min_m[k] * 1000:.0f},{roi_max_m[k] * 1000:.0f}]"
                for k, a in enumerate(("x", "y", "z"))
            )
            + "mm"
        )
        config_path = os.environ.get("RB_GUI_SERVER_CONFIG_PATH", "").strip()
        if not config_path:
            return True, f"{sent} (runtime only: RB_GUI_SERVER_CONFIG_PATH not set)"
        _, save_message = persist_roi_bounds_to_config(config_path, roi_min_m, roi_max_m)
        return True, f"{sent} ({save_message})"

    def send_set_user_floor_plane(
        self,
        point_m: tuple[float, float, float],
        normal: tuple[float, float, float],
        *,
        margin_m: float = 0.0,
        enable: bool = True,
    ) -> tuple[bool, str]:
        # Non-motion, leaseless safety adjustment (mirror of send_set_floor_z): a live
        # state stream is required so the operator sees the applied plane. The server
        # bounds-checks the request (unit normal, tilt vs max_tilt_deg, point z,
        # margin) and rejects an enable when safety.user_floor_constraint.enable=false.
        # A disable (enable=False) is accepted unconditionally. Plane persistence
        # across restarts is handled GUI-side (user_floor.json re-sent on startup),
        # so there is no server-config YAML rewrite here.
        latest = self.latest_valid()
        if latest is None:
            return False, "state stream missing or stale"
        try:
            self.command_client.send_set_user_safety_floor_plane(
                point_m, normal, margin_m=margin_m, enable=enable,
                timeout_sec=self.command_timeout_sec,
            )
        except ValueError as exc:
            return False, str(exc)
        if not enable:
            return True, "sent SetUserSafetyFloorPlane disable"
        return True, (
            "sent SetUserSafetyFloorPlane point["
            + ",".join(f"{v * 1000:.0f}" for v in point_m)
            + "]mm n["
            + ",".join(f"{v:.3f}" for v in normal)
            + f"] margin {margin_m * 1000:.1f}mm"
        )

    def send_freedrive(
        self, *, left: bool | None = None, right: bool | None = None
    ) -> tuple[bool, str]:
        """Per-arm direct-teaching (free-drive) toggle.

        left/right: True enters free-drive (hand-guidable), False exits + resyncs,
        None leaves that arm untouched. Requires a live state stream; the server
        is the authority (servo.allow_freedrive + lease + supervision)."""
        if left is None and right is None:
            return False, "specify at least one arm"
        latest = self.latest_valid()
        if latest is None:
            return False, "state stream missing or stale"
        try:
            self.command_client.send_freedrive(
                left=left, right=right, timeout_sec=self.command_timeout_sec
            )
        except ValueError as exc:
            return False, str(exc)
        parts: list[str] = []
        if left is not None:
            parts.append(f"left {'ON' if left else 'OFF'}")
        if right is not None:
            parts.append(f"right {'ON' if right else 'OFF'}")
        return True, "sent Freedrive " + ", ".join(parts)

    def init_motion_disabled_reason(self) -> str | None:
        reason = self.blocked_reason("JointTarget")
        if reason:
            return reason
        if self.init_left_joint_deg is None or self.init_right_joint_deg is None:
            return "init motion target not configured"
        if self.init_motion_timeout_sec <= 0.0 or not math.isfinite(self.init_motion_timeout_sec):
            return "init motion timeout must be positive and finite"
        latest = self.latest_valid()
        if latest is None:
            return "state stream missing or stale"
        if latest.motion_state not in {"ArmedHold", "Running"}:
            return "ArmMotion first; init motion requires ArmedHold or Running"
        return None

    def send_init_motion(self) -> tuple[bool, str]:
        reason = self.init_motion_disabled_reason()
        if reason:
            return False, reason
        assert self.init_left_joint_deg is not None
        assert self.init_right_joint_deg is not None
        self.command_client.send(
            self.command_client.build_init_motion(
                self.init_left_joint_deg,
                self.init_right_joint_deg,
                timeout_sec=self.init_motion_timeout_sec,
            )
        )
        return True, "sent JointTarget init_motion profile"

    def set_init_joints(
        self,
        left_q_deg: tuple[float, ...] | None,
        right_q_deg: tuple[float, ...] | None,
    ) -> tuple[bool, str]:
        # Update the init-motion target pose at runtime (used by the WayPoint
        # "set as init" button). Persistence to JSON is handled by the caller.
        left = self._validated_joint6(left_q_deg)
        right = self._validated_joint6(right_q_deg)
        if left is None or right is None:
            return False, "init joints require finite 6-DOF values for both arms"
        self.init_left_joint_deg = left
        self.init_right_joint_deg = right
        return True, "init motion pose updated"

    def jog_joint(self, arm: Literal["left", "right"], joint_index: int, delta_deg: float) -> tuple[bool, str]:
        reason = self.blocked_reason("JointTarget")
        if reason:
            return False, reason
        latest = self.latest_valid()
        if latest is None:
            return False, "state stream missing or stale"
        if joint_index < 0 or joint_index >= 6:
            return False, "joint index out of range"
        if not math.isfinite(delta_deg):
            return False, "non-finite jog delta rejected"
        if latest.left.q_sent_deg is None or latest.left.q_actual_deg is None or latest.right.q_sent_deg is None or latest.right.q_actual_deg is None:
            return False, "joint state invalid"
        clamped_delta = max(-self.max_jog_step_deg, min(self.max_jog_step_deg, float(delta_deg)))
        left = list(latest.left.q_sent_deg if latest.left.has_valid_joint_state else latest.left.q_actual_deg)
        right = list(latest.right.q_sent_deg if latest.right.has_valid_joint_state else latest.right.q_actual_deg)
        target = left if arm == "left" else right
        target[joint_index] += clamped_delta
        if not all(math.isfinite(v) for v in left + right):
            return False, "non-finite target rejected"
        self.command_client.send(self.command_client.build_joint_target(tuple(left), tuple(right), timeout_sec=self.command_timeout_sec))
        return True, f"sent {arm} J{joint_index + 1} jog {clamped_delta:+.3f} deg"

    def send_joint_target(
        self,
        *,
        left_q_deg: tuple[float, ...] | None,
        right_q_deg: tuple[float, ...] | None,
    ) -> tuple[bool, str]:
        # Absolute dual-arm JointTarget (used by WayPoint moveJ). Designed to be
        # re-sent at a hold cadence: the standard command_timeout_sec freshness
        # acts as the deadman, so releasing the hold lets the server hold in
        # place once commands go stale.
        reason = self.blocked_reason("JointTarget")
        if reason:
            return False, reason
        if self.latest_valid() is None:
            return False, "state stream missing or stale"
        left = self._validated_joint6(left_q_deg)
        right = self._validated_joint6(right_q_deg)
        if left is None or right is None:
            return False, "joint target requires finite 6-DOF values for both arms"
        self.command_client.send(
            self.command_client.build_joint_target(left, right, timeout_sec=self.command_timeout_sec)
        )
        return True, "sent JointTarget"

    def tcp_jog_unavailable(self) -> tuple[bool, str]:
        return False, "TCP jog unavailable: FK/IK is deferred; no Cartesian motion command sent"

    def send_tcp_pose_target(
        self,
        *,
        left_pose: tuple[float, ...] | None = None,
        right_pose: tuple[float, ...] | None = None,
        left_quaternion_xyzw: tuple[float, ...] | None = None,
        right_quaternion_xyzw: tuple[float, ...] | None = None,
    ) -> tuple[bool, str]:
        reason = self.tcp_command_disabled_reason()
        if reason:
            return False, reason
        if left_pose is None and right_pose is None:
            return False, "no TCP target selected"
        values = (
            list(left_pose or ())
            + list(right_pose or ())
            + list(left_quaternion_xyzw or ())
            + list(right_quaternion_xyzw or ())
        )
        try:
            finite_values = [float(value) for value in values]
        except (TypeError, ValueError):
            return False, "non-finite TCP target rejected"
        if any(not math.isfinite(value) for value in finite_values):
            return False, "non-finite TCP target rejected"
        for quaternion in (left_quaternion_xyzw, right_quaternion_xyzw):
            if quaternion is None:
                continue
            if len(quaternion) != 4:
                return False, "TCP target quaternion must have 4 values"
            norm = math.sqrt(sum(float(value) * float(value) for value in quaternion))
            if not math.isfinite(norm) or norm <= 0.0:
                return False, "TCP target quaternion must be non-zero"
        try:
            packet = self.command_client.build_tcp_pose_target(
                left_pose=left_pose,
                right_pose=right_pose,
                left_quaternion_xyzw=left_quaternion_xyzw,
                right_quaternion_xyzw=right_quaternion_xyzw,
                timeout_sec=self.command_timeout_sec,
            )
        except ValueError as exc:
            return False, str(exc)
        self.command_client.send(packet)
        arms = []
        if left_pose is not None:
            arms.append("left")
        if right_pose is not None:
            arms.append("right")
        return True, f"sent {'+'.join(arms)} TCP pose target (server may report CartesianUnavailable until IK is implemented)"

    def send_tcp_linear_move(
        self,
        *,
        left_pose: tuple[float, ...] | None = None,
        right_pose: tuple[float, ...] | None = None,
        left_quaternion_xyzw: tuple[float, ...] | None = None,
        right_quaternion_xyzw: tuple[float, ...] | None = None,
        duration_sec: float | None = None,
        linear_speed_m_s: float | None = None,
        angular_speed_rad_s: float | None = None,
        orientation_mode: str = "constant",
    ) -> tuple[bool, str]:
        reason = self.tcp_command_disabled_reason()
        if reason:
            return False, reason
        if left_pose is None and right_pose is None:
            return False, "no TCP linear target selected"
        for arm, pose in (("left", left_pose), ("right", right_pose)):
            if pose is None:
                continue
            arm_reason = self.tcp_command_disabled_reason(arm)  # type: ignore[arg-type]
            if arm_reason:
                return False, arm_reason
        values = (
            list(left_pose or ())
            + list(right_pose or ())
            + list(left_quaternion_xyzw or ())
            + list(right_quaternion_xyzw or ())
            + [value for value in (duration_sec, linear_speed_m_s, angular_speed_rad_s) if value is not None]
        )
        try:
            finite_values = [float(value) for value in values]
        except (TypeError, ValueError):
            return False, "non-finite TCP linear target rejected"
        if any(not math.isfinite(value) for value in finite_values):
            return False, "non-finite TCP linear target rejected"
        if duration_sec is None and linear_speed_m_s is None:
            return False, "duration_sec or linear_speed_m_s is required"
        for label, value in (
            ("duration_sec", duration_sec),
            ("linear_speed_m_s", linear_speed_m_s),
            ("angular_speed_rad_s", angular_speed_rad_s),
        ):
            if value is not None and float(value) <= 0.0:
                return False, f"{label} must be positive"
        if str(orientation_mode).strip().lower() not in {"constant", "slerp"}:
            return False, "orientation_mode must be constant or slerp"
        for quaternion in (left_quaternion_xyzw, right_quaternion_xyzw):
            if quaternion is None:
                continue
            if len(quaternion) != 4:
                return False, "TCP linear target quaternion must have 4 values"
            norm = math.sqrt(sum(float(value) * float(value) for value in quaternion))
            if not math.isfinite(norm) or norm <= 0.0:
                return False, "TCP linear target quaternion must be non-zero"
        try:
            packet = self.command_client.build_tcp_linear_move(
                left_pose=left_pose,
                right_pose=right_pose,
                left_quaternion_xyzw=left_quaternion_xyzw,
                right_quaternion_xyzw=right_quaternion_xyzw,
                duration_sec=duration_sec,
                linear_speed_m_s=linear_speed_m_s,
                angular_speed_rad_s=angular_speed_rad_s,
                orientation_mode=orientation_mode,
                timeout_sec=self.command_timeout_sec,
            )
        except ValueError as exc:
            return False, str(exc)
        self.command_client.send(packet)
        arms = []
        if left_pose is not None:
            arms.append("left")
        if right_pose is not None:
            arms.append("right")
        self.last_tcp_command = f"TcpLinearMove {'+'.join(arms)} {str(orientation_mode).strip().lower()}"
        return True, f"sent {'+'.join(arms)} TCP linear move"

    def set_recording_intent(self, active: bool) -> str:
        self.recording_intent = bool(active)
        return "recording intent marked; server logger health is read-only in this milestone"
