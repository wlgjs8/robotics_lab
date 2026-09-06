from __future__ import annotations

import copy
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Iterable

from .servo_command_client import CommandIntent


ARM_INIT_COMMAND_SCHEMA = "robotics_lab.arm_init_cmd.v1"

# Debug instrumentation for the flow-infer vs InitMotion cancel race. When enabled
# (RB_ARM_INIT_DEBUG=1) every latch transition is logged WITH ITS CAUSE so a
# committed init move that gets cancelled by a leaked flow TcpPoseTarget can be
# traced to either (a) a cancel/toggle edge or (b) a server-status-driven clear.
# Default off -> zero behavior/output change for normal runs and tests.
_ARM_INIT_DEBUG = os.environ.get("RB_ARM_INIT_DEBUG", "").strip().lower() not in {
    "",
    "0",
    "false",
    "no",
    "off",
}


def _dbg(msg: str) -> None:
    if not _ARM_INIT_DEBUG:
        return
    try:
        now = time.time()
        wall = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now % 1) * 1000):03d}"
        print(f"[arm_init {wall} mono_ns={time.monotonic_ns()}] {msg}", file=sys.stderr, flush=True)
    except Exception:
        pass
ARM_INIT_STATE_SCHEMA = "robotics_lab.arm_init_state.v1"


@dataclass(frozen=True)
class ArmInitCommand:
    arms: str
    action: str = "toggle"
    left_q_deg: tuple[float, ...] | None = None
    right_q_deg: tuple[float, ...] | None = None
    # action == "config" only: flip the ROI auto-recover toggle. None leaves it
    # alone, so a config packet that only refreshes the init pose is expressible.
    auto_roi_recover: bool | None = None


@dataclass(frozen=True)
class ArmInitTransitions:
    started: tuple[str, ...] = ()
    done: tuple[str, ...] = ()
    cancelled: tuple[str, ...] = ()
    failed: tuple[str, ...] = ()

    @property
    def any(self) -> bool:
        return bool(self.started or self.done or self.cancelled or self.failed)


@dataclass
class _ArmRuntime:
    on: bool = False
    q_deg: tuple[float, ...] | None = None
    state: str = "policy"
    fail: bool = False
    fail_mode: str = ""
    message: str = ""
    server_status: str = "idle"
    goal_nearest_pair_name_a: str = ""
    goal_nearest_pair_name_b: str = ""
    goal_nearest_pair_category: str = ""
    goal_clearance_m: float | None = None
    goal_threshold_m: float | None = None
    goal_margin_deficit_m: float | None = None
    # Monotonic time at which the arm reached "done" but its F/T auto-tare was still
    # pending; None when not waiting.
    tare_wait_started: float | None = None
    request_id: int = 0
    motion_done: bool = False
    ft_required: bool = False
    tare_timed_out: bool = False
    tare_wait_elapsed: float = 0.0


def _ft_tare_pending(payload: Any, arm: str, *, required: bool = False) -> tuple[bool, str]:
    """Is the server's post-InitMotion F/T auto-tare still in flight for `arm`?

    Reads the state fanout's ``<arm>.force_torque`` block. Pending while the
    auto-tare is armed or settling, or while the bias is invalid with a live
    sensor. Disabled F/T needs no tare. Missing telemetry stays pending once
    enabled F/T was observed; legacy/mock states may omit it.
    An enabled but disconnected sensor must not release the latch."""
    if not isinstance(payload, dict):
        return required, "no payload"
    arm_block = payload.get(arm)
    if not isinstance(arm_block, dict):
        return required, "no arm block"
    ft = arm_block.get("force_torque")
    if not isinstance(ft, dict):
        return required, "no force_torque block"
    if ft.get("enabled") is False:
        return False, "ft disabled"
    required = required or ft.get("enabled") is True
    if required and ft.get("connected") is not True:
        return True, "ft disconnected or unavailable"
    stage = str(ft.get("auto_tare_stage", "") or "").strip().lower()
    if stage in {"awaiting_init", "settling"}:
        return True, f"auto_tare_stage={stage}"
    if str(ft.get("tare_state", "")).lower() == "settling":
        return True, "tare samples accumulating"
    if ft.get("bias_valid") is False or (required and ft.get("bias_valid") is not True):
        return True, "bias invalid or unavailable"
    return False, f"auto_tare_stage={stage or 'idle'} bias_valid={ft.get('bias_valid')}"


def _snapshot_monotonic(snapshot: Any) -> float:
    value = getattr(snapshot, "received_monotonic", None)
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def parse_arm_init_command(data: bytes | dict[str, Any]) -> ArmInitCommand:
    if isinstance(data, bytes):
        try:
            payload = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("arm init command must be JSON") from exc
    else:
        payload = data
    if not isinstance(payload, dict):
        raise ValueError("arm init command must be a JSON object")
    if payload.get("schema") != ARM_INIT_COMMAND_SCHEMA:
        raise ValueError("unsupported arm init command schema")
    arms = payload.get("arms")
    if arms not in {"both", "left", "right"}:
        raise ValueError("arm init command arms must be both, left, or right")
    action = payload.get("action", "toggle")
    if action not in {"start", "cancel", "toggle", "config"}:
        raise ValueError("arm init command action must be start, cancel, toggle, or config")
    auto_roi_recover = payload.get("auto_roi_recover")
    if auto_roi_recover is not None and not isinstance(auto_roi_recover, bool):
        raise ValueError("arm init command auto_roi_recover must be a bool")
    return ArmInitCommand(
        arms=str(arms),
        action=str(action),
        left_q_deg=_optional_joint6(payload.get("left_q_deg"), "left_q_deg"),
        right_q_deg=_optional_joint6(payload.get("right_q_deg"), "right_q_deg"),
        auto_roi_recover=auto_roi_recover,
    )


class ArmInitOverrideController:
    """Runtime per-arm InitMotion latch owned by policy_runner.run()."""

    def __init__(
        self,
        *,
        timeout_sec: float = 0.2,
        auto_clear_on_done: bool = True,
        auto_clear_on_failed: bool = False,
        resume_flow_on_done: bool = True,
        resume_flow_on_failed: bool = False,
        allow_manual_cancel_after_failed: bool = True,
        reset_flow_source_on_start: bool = True,
        reset_flow_source_on_resume: bool = True,
        ft_tare_wait_sec: float = 2.0,
    ) -> None:
        self.timeout_sec = float(timeout_sec)
        # See ArmInitOverrideConfig.ft_tare_wait_sec. MEASURED 2026-09-04
        # (servo_log_20260904_005550.csv 47.04-47.12 s): the runner resumed the
        # policy 80 ms after "done", the tare's 0.5 s settle never completed, and
        # force control stayed uncovered ("F/T has no bias yet") for the rest of the
        # episode (47-59 s); same in the 00:36 session (139-165 s).
        self.ft_tare_wait_sec = max(0.0, float(ft_tare_wait_sec))
        self.auto_clear_on_done = bool(auto_clear_on_done)
        self.auto_clear_on_failed = bool(auto_clear_on_failed)
        self.resume_flow_on_done = bool(resume_flow_on_done)
        self.resume_flow_on_failed = bool(resume_flow_on_failed)
        self.allow_manual_cancel_after_failed = bool(allow_manual_cancel_after_failed)
        self.reset_flow_source_on_start = bool(reset_flow_source_on_start)
        self.reset_flow_source_on_resume = bool(reset_flow_source_on_resume)
        self._left = _ArmRuntime()
        self._right = _ArmRuntime()
        # ROI AUTO-RECOVER toggle, owned here rather than in run() so the GUI
        # checkbox reads its own truth back out of arm_init_state: the operator
        # must see the state of the policy_runner that is actually driving, not
        # the state of whichever GUI session last clicked. Off until the operator
        # turns it on -- an autonomous move nobody asked for is not a default.
        self.auto_roi_recover = False
        self.last_command = ""
        self.error = ""
        self.changed = False
        self._pending: dict[str, set[str]] = {
            "started": set(),
            "done": set(),
            "cancelled": set(),
            "failed": set(),
        }
        # Edge state for the compose_intent leak detector (debug-only).
        self._dbg_prev_any_on = False

    @property
    def left_on(self) -> bool:
        return self._left.on

    @property
    def right_on(self) -> bool:
        return self._right.on

    @property
    def left_q_deg(self) -> tuple[float, ...] | None:
        return self._left.q_deg

    @property
    def right_q_deg(self) -> tuple[float, ...] | None:
        return self._right.q_deg

    def handle_command(self, command: ArmInitCommand) -> bool:
        self.changed = False
        self.error = ""
        self.last_command = command.arms
        if command.left_q_deg is not None:
            self._left.q_deg = command.left_q_deg
        if command.right_q_deg is not None:
            self._right.q_deg = command.right_q_deg

        arms = _arms_for_selector(command.arms)
        action = command.action
        if action == "config":
            # Settings-only packet: it must never start or cancel a move. The init
            # pose it carries was already stored above, which is the point -- the
            # GUI re-sends it whenever the operator edits the pose, so the auto
            # recovery cannot park an arm at a stale target.
            if command.auto_roi_recover is None:
                return False
            changed = bool(self.auto_roi_recover) != bool(command.auto_roi_recover)
            self.auto_roi_recover = bool(command.auto_roi_recover)
            self.changed = changed
            _dbg(f"config auto_roi_recover={self.auto_roi_recover} changed={changed}")
            return changed
        if action == "toggle":
            failed = any(self._runtime(arm).fail for arm in arms)
            action = "start" if failed and not self.allow_manual_cancel_after_failed else (
                "cancel" if any(self._runtime(arm).on for arm in arms) else "start"
            )
        _dbg(
            f"cmd recv arms={command.arms} action_req={command.action} -> resolved={action} "
            f"(pre left_on={self._left.on} right_on={self._right.on} "
            f"left_fail={self._left.fail} right_fail={self._right.fail} "
            f"left_status={self._left.server_status} right_status={self._right.server_status})"
        )
        if action == "cancel":
            return self._cancel_arms(arms)
        return self._start_arms(arms)

    def handle_payloads(self, payloads: Iterable[dict[str, Any]]) -> bool:
        changed = False
        for payload in payloads:
            if not isinstance(payload, dict) or payload.get("schema") != ARM_INIT_COMMAND_SCHEMA:
                continue
            try:
                changed = self.handle_command(parse_arm_init_command(payload)) or changed
            except ValueError as exc:
                self.error = str(exc)
        return changed

    def consume_transitions(self) -> ArmInitTransitions:
        transitions = ArmInitTransitions(
            started=tuple(sorted(self._pending["started"])),
            done=tuple(sorted(self._pending["done"])),
            cancelled=tuple(sorted(self._pending["cancelled"])),
            failed=tuple(sorted(self._pending["failed"])),
        )
        for values in self._pending.values():
            values.clear()
        return transitions

    def update_from_snapshot(self, snapshot: Any) -> bool:
        payload = getattr(snapshot, "payload", None)
        if not isinstance(payload, dict):
            return False
        block = payload.get("init_motion")
        if not isinstance(block, dict):
            return False
        changed = False
        for arm in ("left", "right"):
            runtime = self._runtime(arm)
            arm_block = block.get(arm)
            if not isinstance(arm_block, dict):
                arm_block = block
            status = _norm_status(arm_block.get("status"))
            ft_arm = payload.get(arm)
            ft = ft_arm.get("force_torque") if isinstance(ft_arm, dict) else None
            if isinstance(ft, dict) and ft.get("enabled") is True:
                runtime.ft_required = True
            elif isinstance(ft, dict) and ft.get("enabled") is False:
                runtime.ft_required = False
            # Do not consume a previous request's Done before this request lands.
            # Legacy/mock snapshots without an acknowledgement remain compatible.
            ack = arm_block.get("request_id")
            if runtime.on and not runtime.motion_done and ack is not None and ack != runtime.request_id:
                continue
            fail_mode = str(arm_block.get("fail_mode", "") or "")
            nearest_a = str(arm_block.get("goal_nearest_pair_name_a", "") or "")
            nearest_b = str(arm_block.get("goal_nearest_pair_name_b", "") or "")
            nearest_category = str(arm_block.get("goal_nearest_pair_category", "") or "")
            clearance_m = _optional_float(
                arm_block.get("goal_nearest_pair_distance_m", arm_block.get("nearest_pair_distance_m"))
            )
            threshold_m = _goal_threshold_m(arm_block)
            margin_deficit_m = _optional_float(arm_block.get("goal_clear_margin_deficit_m"))
            runtime.fail_mode = fail_mode or runtime.fail_mode
            runtime.goal_nearest_pair_name_a = nearest_a or runtime.goal_nearest_pair_name_a
            runtime.goal_nearest_pair_name_b = nearest_b or runtime.goal_nearest_pair_name_b
            runtime.goal_nearest_pair_category = nearest_category or runtime.goal_nearest_pair_category
            runtime.goal_clearance_m = clearance_m if clearance_m is not None else runtime.goal_clearance_m
            runtime.goal_threshold_m = threshold_m if threshold_m is not None else runtime.goal_threshold_m
            runtime.goal_margin_deficit_m = (
                margin_deficit_m if margin_deficit_m is not None else runtime.goal_margin_deficit_m
            )
            message = _format_failed_message(arm_block)
            if not message:
                message = str(arm_block.get("message", "") or "")
            prev_status = runtime.server_status
            if status and status != runtime.server_status:
                if runtime.on:
                    self._event(arm, "server_status", status=status, snapshot_mono=_snapshot_monotonic(snapshot))
                _dbg(
                    f"{arm} server_status {runtime.server_status or 'none'} -> {status} "
                    f"(latch on={runtime.on} msg={message!r})"
                )
            if status:
                runtime.server_status = status
            if message:
                runtime.message = message
            if not runtime.on:
                # EXTERNAL InitMotion (e.g. the rb_gui button that commands the
                # server directly, bypassing the arm_init_cmd channel): the arm
                # was moved outside the policy stream. Surface the completion as
                # a DONE transition so the source re-anchors at the new pose
                # (plan chain / conditioners / RTC prev) exactly like a
                # channel-initiated init.
                if status == "done" and prev_status in {"planning", "executing"}:
                    self._pending["done"].add(arm)
                    changed = True
                continue
            # A delayed Done after Failed cannot revive a failed request. Only
            # an explicit start/retry clears this latch and assigns a new ID.
            if runtime.fail and status != "failed":
                continue
            if status in {"planning", "executing"} and not runtime.motion_done:
                runtime.tare_wait_started = None
                next_state = f"init {status}"
                if runtime.state != next_state:
                    runtime.state = next_state
                    changed = True
                continue
            if status == "failed":
                self._mark_failed(arm, message or fail_mode or "init_failed")
                changed = True
                continue
            if status == "done" or runtime.motion_done:
                # Done is latched locally: emit Hold while the server settles/tare
                # completes, even after its sequencer returns Idle. Re-emitting
                # InitMotion here used to invalidate the bias every packet.
                runtime.motion_done = True
                pending, why = _ft_tare_pending(payload, arm, required=runtime.ft_required)
                if pending and self.ft_tare_wait_sec > 0.0:
                    now = _snapshot_monotonic(snapshot)
                    if runtime.tare_wait_started is None:
                        runtime.tare_wait_started = now
                        self._event(arm, "tare_wait", reason=why, snapshot_mono=now)
                        _dbg(f"{arm} done; holding for the F/T auto-tare ({why})")
                    waited = now - runtime.tare_wait_started
                    runtime.tare_wait_elapsed = max(0.0, waited)
                    if waited < self.ft_tare_wait_sec:
                        if runtime.state != "init tare":
                            runtime.state = "init tare"
                            changed = True
                        runtime.message = f"waiting for F/T tare ({why})"
                        continue
                    if not runtime.tare_timed_out:
                        runtime.tare_timed_out = True
                        self._event(arm, "tare_timeout_hold", reason=why, waited_sec=waited)
                        changed = True
                    self.error = f"{arm} F/T tare did not land within {self.ft_tare_wait_sec:.2f} s ({why}); holding"
                    runtime.state = "init tare blocked"
                    runtime.message = self.error
                    continue
                if pending:
                    self._event(arm, "tare_wait_disabled", reason=why)
                elif runtime.tare_wait_started is not None:
                    runtime.tare_wait_elapsed = max(0., _snapshot_monotonic(snapshot)-runtime.tare_wait_started)
                    self._event(arm, "tare_ready", waited_sec=runtime.tare_wait_elapsed)
                if self.error.startswith(f"{arm} F/T tare"):
                    self.error = ""
                runtime.tare_wait_started = None
                self._mark_done(arm)
                changed = True
                continue
        self.changed = self.changed or changed
        return changed

    def compose_intent(self, intent: CommandIntent | None) -> CommandIntent | None:
        if _ARM_INIT_DEBUG:
            any_on = self.left_on or self.right_on
            if any_on != self._dbg_prev_any_on:
                if any_on:
                    _dbg(f"OVERRIDE ENGAGED left_on={self.left_on} right_on={self.right_on}")
                else:
                    left_mode = (
                        intent.left.get("mode")
                        if intent is not None and isinstance(intent.left, dict)
                        else None
                    )
                    right_mode = (
                        intent.right.get("mode")
                        if intent is not None and isinstance(intent.right, dict)
                        else None
                    )
                    _dbg(
                        "OVERRIDE DROPPED both off -> flow passes through to server "
                        f"(flow left_mode={left_mode} right_mode={right_mode}) "
                        "<- this tick's flow command can CANCEL a committed init move"
                    )
                self._dbg_prev_any_on = any_on
        if not self.left_on and not self.right_on:
            return intent
        left = _arm_payload(intent.left if intent is not None else None)
        right = _arm_payload(intent.right if intent is not None else None)
        timeout_sec = intent.timeout_sec if intent is not None else self.timeout_sec
        coupled_timeout = intent.coupled_timeout if intent is not None else True
        if self.left_on:
            if self._left.fail or self._left.motion_done:
                left = {"mode": "Hold"}
            elif self.left_q_deg is None:
                self.error = "missing_left_init_q_deg"
                return intent
            else:
                left = _init_arm_payload(self.left_q_deg, self._left.request_id)
        if self.right_on:
            if self._right.fail or self._right.motion_done:
                right = {"mode": "Hold"}
            elif self.right_q_deg is None:
                self.error = "missing_right_init_q_deg"
                return intent
            else:
                right = _init_arm_payload(self.right_q_deg, self._right.request_id)
        # Carry the top-level fields through. `tcp_target_profile` selects the
        # server profile for BOTH arms' TcpPoseTarget; dropping it made the packet
        # fall back to the server default (`umi_large_smooth`, no chunk follower),
        # so a single-arm InitMotion pulled the PEER arm off its chunk follower on
        # the override's first tick ("disengaged (mode/enable)") and left it on the
        # pose-track SMD for the whole override. MEASURED 2026-09-04
        # servo_log_20260904_003600.csv 148.554 s: left `tcp_target_profile`
        # flow_infer_smooth -> umi_large_smooth on the tick the right init started,
        # left J5 22.3 -> 0 deg/s in one tick (-11,171 deg/s^2), then a 250 mm/s SMD
        # chase of a 22 mm lead; follower back only at 151.88 s with the profile.
        return CommandIntent(
            _mixed_top_mode(left, right),
            timeout_sec=timeout_sec,
            left=left,
            right=right,
            coupled_timeout=coupled_timeout,
            tcp_target_profile=intent.tcp_target_profile if intent is not None else None,
            metadata=copy.deepcopy(intent.metadata) if intent is not None and intent.metadata is not None else None,
        )

    def stamp_snapshot(self, snapshot: Any) -> None:
        payload = getattr(snapshot, "payload", None)
        if isinstance(payload, dict):
            payload["arm_init"] = self.status_block()

    def status_block(self) -> dict[str, Any]:
        return {
            "schema": ARM_INIT_STATE_SCHEMA,
            "left_request_id": self._left.request_id,
            "right_request_id": self._right.request_id,
            "left_tare_wait_elapsed_sec": self._left.tare_wait_elapsed,
            "right_tare_wait_elapsed_sec": self._right.tare_wait_elapsed,
            "left_tare_timed_out": self._left.tare_timed_out,
            "right_tare_timed_out": self._right.tare_timed_out,
            "init_override_left": bool(self.left_on),
            "init_override_right": bool(self.right_on),
            "left_state": self._state_for("left"),
            "right_state": self._state_for("right"),
            "left_server_status": self._left.server_status,
            "right_server_status": self._right.server_status,
            "left_message": self._left.message,
            "right_message": self._right.message,
            "left_fail_mode": self._left.fail_mode,
            "right_fail_mode": self._right.fail_mode,
            "left_goal_nearest_pair_name_a": self._left.goal_nearest_pair_name_a,
            "left_goal_nearest_pair_name_b": self._left.goal_nearest_pair_name_b,
            "left_goal_nearest_pair_category": self._left.goal_nearest_pair_category,
            "left_goal_clearance_m": self._left.goal_clearance_m,
            "left_goal_threshold_m": self._left.goal_threshold_m,
            "left_goal_margin_deficit_m": self._left.goal_margin_deficit_m,
            "right_goal_nearest_pair_name_a": self._right.goal_nearest_pair_name_a,
            "right_goal_nearest_pair_name_b": self._right.goal_nearest_pair_name_b,
            "right_goal_nearest_pair_category": self._right.goal_nearest_pair_category,
            "right_goal_clearance_m": self._right.goal_clearance_m,
            "right_goal_threshold_m": self._right.goal_threshold_m,
            "right_goal_margin_deficit_m": self._right.goal_margin_deficit_m,
            "last_command": self.last_command,
            "error": self.error,
            "auto_clear_on_done": self.auto_clear_on_done,
            "auto_clear_on_failed": self.auto_clear_on_failed,
            "resume_flow_on_done": self.resume_flow_on_done,
            "resume_flow_on_failed": self.resume_flow_on_failed,
            "allow_manual_cancel_after_failed": self.allow_manual_cancel_after_failed,
            "auto_roi_recover": bool(self.auto_roi_recover),
        }

    def source_mask_for(self, base_mask: Any) -> Any:
        if base_mask is None:
            return None
        try:
            import numpy as np

            mask = np.asarray(base_mask, dtype=np.float32).copy()
        except Exception:
            return None
        if mask.shape[0] < 2:
            return None
        if self.left_on:
            mask[0] = 0.0
        if self.right_on:
            mask[1] = 0.0
        return mask

    def _runtime(self, arm: str) -> _ArmRuntime:
        return self._left if arm == "left" else self._right

    def _event(self, arm: str, event: str, **fields: Any) -> None:
        # Edge-only events land in the existing captured console, never per tick.
        try:
            print("[arm_init_event] " + json.dumps({
                "schema": "robotics_lab.arm_init_event.v1", "t_mono": time.monotonic(),
                "t_wall": time.time(), "arm": arm, "request_id": self._runtime(arm).request_id,
                "event": event, **fields,
            }, sort_keys=True), flush=True)
        except (OSError, ValueError):
            pass  # Telemetry failure must not change the latch or command path.

    def _start_arms(self, arms: tuple[str, ...]) -> bool:
        missing = [
            arm
            for arm in arms
            if self._runtime(arm).q_deg is None
        ]
        if missing:
            self.error = "missing_init_q_deg" if len(missing) > 1 else f"missing_{missing[0]}_init_q_deg"
            return False
        changed = False
        for arm in arms:
            runtime = self._runtime(arm)
            was_on = runtime.on
            self._pending["started"].add(arm)
            changed = True
            runtime.request_id = max(time.monotonic_ns(), runtime.request_id + 1)
            runtime.motion_done = False
            # Keep the known F/T prerequisite across requests. A missing block
            # on the new request must not turn a previously enabled sensor into
            # the legacy/no-F/T path; explicit disabled telemetry clears it.
            runtime.tare_wait_started = None
            runtime.tare_timed_out = False
            runtime.tare_wait_elapsed = 0.0
            self._event(arm, "start")
            if not was_on:
                _dbg(f"{arm} latch SET cause=start")
            runtime.on = True
            runtime.fail = False
            runtime.fail_mode = ""
            runtime.state = "init requested"
            runtime.message = ""
            runtime.server_status = "requested"
            runtime.goal_nearest_pair_name_a = ""
            runtime.goal_nearest_pair_name_b = ""
            runtime.goal_nearest_pair_category = ""
            runtime.goal_clearance_m = None
            runtime.goal_threshold_m = None
            runtime.goal_margin_deficit_m = None
        self.changed = changed
        return changed

    def _cancel_arms(self, arms: tuple[str, ...]) -> bool:
        changed = False
        for arm in arms:
            runtime = self._runtime(arm)
            if runtime.on or runtime.state.startswith("init"):
                self._pending["cancelled"].add(arm)
                changed = True
                self._event(arm, "cancel", state=runtime.state, tare_timed_out=runtime.tare_timed_out)
                _dbg(
                    f"{arm} latch CLEAR cause=cancel "
                    f"(prev on={runtime.on} server_status={runtime.server_status} state={runtime.state})"
                )
            runtime.on = False
            runtime.fail = False
            runtime.fail_mode = ""
            runtime.state = "cancelled"
            runtime.message = ""
            runtime.server_status = "cancelled"
            runtime.goal_nearest_pair_name_a = ""
            runtime.goal_nearest_pair_name_b = ""
            runtime.goal_nearest_pair_category = ""
            runtime.goal_clearance_m = None
            runtime.goal_threshold_m = None
            runtime.goal_margin_deficit_m = None
        self.changed = changed
        return changed

    def _mark_done(self, arm: str) -> None:
        runtime = self._runtime(arm)
        if runtime.state != "init done":
            self._event(arm, "resume" if self.auto_clear_on_done and self.resume_flow_on_done else "done_hold")
        _cleared = self.auto_clear_on_done and self.resume_flow_on_done
        _dbg(
            f"{arm} server reported DONE while latch on={runtime.on} "
            f"(server_status={runtime.server_status}); "
            f"latch {'CLEAR->flow' if _cleared else 'HOLD'} cause=done"
        )
        if runtime.state != "init done":
            self._pending["done"].add(arm)
        runtime.fail = False
        runtime.fail_mode = ""
        runtime.message = ""
        runtime.server_status = "done"
        runtime.goal_nearest_pair_name_a = ""
        runtime.goal_nearest_pair_name_b = ""
        runtime.goal_nearest_pair_category = ""
        runtime.goal_clearance_m = None
        runtime.goal_threshold_m = None
        runtime.goal_margin_deficit_m = None
        if self.auto_clear_on_done and self.resume_flow_on_done:
            runtime.on = False
            runtime.state = "policy"
        else:
            runtime.state = "init done"

    def _mark_failed(self, arm: str, message: str) -> None:
        runtime = self._runtime(arm)
        runtime.motion_done = False
        _cleared = self.auto_clear_on_failed or self.resume_flow_on_failed
        _dbg(
            f"{arm} server reported FAILED while latch on={runtime.on} "
            f"(server_status={runtime.server_status} msg={message!r}); "
            f"latch {'CLEAR->flow' if _cleared else 'HOLD(fail-closed)'} cause=failed"
        )
        if not runtime.fail:
            self._event(arm, "failed", reason=message, resume_enabled=_cleared)
            self._pending["failed"].add(arm)
        runtime.fail = True
        runtime.message = message
        runtime.server_status = "failed"
        if self.auto_clear_on_failed or self.resume_flow_on_failed:
            runtime.on = False
            runtime.state = "policy"
        else:
            runtime.on = True
            runtime.state = "init failed"

    def _state_for(self, arm: str) -> str:
        runtime = self._runtime(arm)
        if runtime.on:
            return runtime.state if runtime.state.startswith("init") else "init requested"
        if runtime.state in {"cancelled", "init done"}:
            return runtime.state
        return "policy"


def source_arm_mask_copy(source: object) -> Any:
    if not hasattr(source, "arm_mask"):
        return None
    try:
        import numpy as np

        return np.asarray(getattr(source, "arm_mask"), dtype=np.float32).copy()
    except Exception:
        return None


def apply_source_arm_mask(source: object, base_mask: Any, controller: ArmInitOverrideController) -> None:
    mask = controller.source_mask_for(base_mask)
    if mask is None:
        return
    try:
        setattr(source, "arm_mask", mask)
    except Exception:
        return


def apply_source_override_transitions(
    source: object,
    transitions: ArmInitTransitions,
    snapshot: Any,
    *,
    reset_on_start: bool = True,
    reset_on_resume: bool = True,
    reset_on_failed: bool = False,
) -> None:
    if not transitions.any:
        return
    for event, arms, reset_enabled in (
        ("start", transitions.started, reset_on_start),
        ("done", transitions.done, reset_on_resume),
        ("cancel", transitions.cancelled, reset_on_resume),
        ("failed", transitions.failed, reset_on_failed),
    ):
        if not arms:
            continue
        hook = getattr(source, f"on_arm_init_override_{event}", None)
        if callable(hook):
            try:
                hook(tuple(arms), snapshot)
                if event == "failed" and reset_enabled:
                    reset_source_after_override_change(source, arms=arms)
                continue
            except Exception:
                pass
        if reset_enabled:
            reset_source_after_override_change(source, arms=arms)


def should_invalidate_chunks_for_override(
    arm_mask: Any,
    overridden_arms: Iterable[str],
    arm_indices: Iterable[tuple[str, int]] = (("left", 0), ("right", 1)),
) -> bool:
    """Should an arm-init override throw the SHARED policy chunk away?

    Only when it leaves no arm behind. The overridden arm is masked out the
    moment the override starts and every consumer of the chunk skips a masked
    arm, so its stale slice is never read; a peer arm's slice is still valid
    because that arm did not move.

    Discarding regardless is what broke: the chunk-overlay publisher returns
    early on a null chunk, for BOTH arms, so one arm's recovery blanked the
    whole stream. MEASURED on servo_log_20260828_004539.csv -- 7 right-arm
    roi_auto_recover events (all y_max) each cost the LEFT arm its chunk
    follower for the entire override (0.9-2.1 s, 5-11 chunk frames skipped),
    ending in a 12,113 deg/s^2 re-engage burst, the largest command
    acceleration in that run. The left arm never moved.

    `arm_mask` is the PRE-override mask: the caller runs before
    apply_source_arm_mask() installs the new one, so the arms about to be
    masked are subtracted here instead.
    """
    overridden = set(overridden_arms)
    for arm, idx in arm_indices:
        if arm in overridden:
            continue
        try:
            if idx < len(arm_mask) and float(arm_mask[idx]) > 0.0:
                return False  # a peer is still driving: keep the chunk
        except (TypeError, ValueError, IndexError):
            continue
    return True


def reset_source_after_override_change(source: object, *, arms: Iterable[str] | None = None) -> None:
    for name in ("_clear_target_pose_state", "reset_engagement"):
        hook = getattr(source, name, None)
        if callable(hook):
            try:
                if arms is None:
                    hook()
                else:
                    try:
                        hook(tuple(arms))
                    except TypeError:
                        hook()
            except Exception:
                pass


def intent_uses_source_requirements(intent: CommandIntent | None, controller: ArmInitOverrideController) -> bool:
    if intent is None:
        return False
    for arm_name, payload in (("left", intent.left), ("right", intent.right)):
        if arm_name == "left" and controller.left_on:
            continue
        if arm_name == "right" and controller.right_on:
            continue
        mode = payload.get("mode") if isinstance(payload, dict) else None
        if isinstance(mode, str) and mode not in {"", "Hold"}:
            return True
        if isinstance(payload, dict) and payload.get("gripper_target") is not None:
            return True
    return False


def _optional_joint6(value: Any, label: str) -> tuple[float, ...] | None:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"{label} must contain 6 values")
    parsed = tuple(float(item) for item in value)
    if any(not math.isfinite(item) for item in parsed):
        raise ValueError(f"{label} values must be finite")
    return parsed


def _arms_for_selector(arms: str) -> tuple[str, ...]:
    if arms == "both":
        return ("left", "right")
    return (arms,)


def _norm_status(value: Any) -> str:
    text = str(value or "").strip().lower()
    if text in {"", "not_attempted"}:
        return ""
    if text in {"idle", "planning", "executing", "done", "failed"}:
        return text
    return text.replace("_", " ")


def _optional_float(value: Any) -> float | None:
    if not isinstance(value, (int, float)):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _goal_threshold_m(block: dict[str, Any]) -> float | None:
    explicit = _optional_float(block.get("goal_threshold_m"))
    if explicit is not None:
        return explicit
    external = bool(block.get("goal_nearest_pair_external", False))
    key = "goal_clear_threshold_external_m" if external else "goal_clear_threshold_self_m"
    return _optional_float(block.get(key))


def _format_failed_message(block: dict[str, Any]) -> str:
    fail_mode = str(block.get("fail_mode", "") or "")
    if fail_mode != "goal_not_clear":
        return str(block.get("message", "") or "")
    name_a = str(block.get("goal_nearest_pair_name_a", "") or "")
    name_b = str(block.get("goal_nearest_pair_name_b", "") or "")
    clearance_m = _optional_float(
        block.get("goal_nearest_pair_distance_m", block.get("nearest_pair_distance_m"))
    )
    threshold_m = _goal_threshold_m(block)
    if name_a or name_b:
        pair = f"{name_a or 'unknown'} <-> {name_b or 'unknown'}"
        if clearance_m is not None and threshold_m is not None:
            return (
                f"goal_not_clear: pair {pair}, "
                f"clearance {clearance_m * 1000.0:.1f} mm < required {threshold_m * 1000.0:.1f} mm"
            )
        return f"goal_not_clear: pair {pair}"
    return str(block.get("message", "") or "goal_not_clear")


def _arm_payload(value: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(value, dict):
        return {"mode": "Hold"}
    payload = copy.deepcopy(value)
    if not payload:
        payload["mode"] = "Hold"
    elif payload.get("mode") is None:
        payload["mode"] = "Hold"
    return payload


def _init_arm_payload(q_deg: tuple[float, ...], request_id: int) -> dict[str, Any]:
    return {
        "mode": "JointTarget",
        "q_target_deg": [float(value) for value in q_deg],
        "joint_target_profile": "init_motion",
        "init_motion_request_id": request_id,
    }


def _mixed_top_mode(left: dict[str, Any], right: dict[str, Any]) -> str:
    left_mode = str(left.get("mode", "Hold"))
    right_mode = str(right.get("mode", "Hold"))
    if left_mode == right_mode:
        return left_mode
    return "Hold"
