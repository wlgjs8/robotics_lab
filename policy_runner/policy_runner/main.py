from __future__ import annotations

import argparse
import dataclasses
import os
import sys
import time
from typing import Callable, TextIO

from .action_sources import (
    DualSpaceMousePoseTargetActionSource,
    HoldActionSource,
    JointSineActionSource,
    TeleopMuxActionSource,
    UmiDualCartesianActionSource,
)
from .config import PolicyRunnerConfig, load_config
from .teleop_capture import TeleopCaptureLogger
from .geometry import GeometryStatus, load_geometry_status
from .robot_state_client import (
    RobotStateClient,
    StateSnapshot,
    StateStreamLeaseReadback,
    command_source_lease_from_snapshot,
    fault_latch_from_snapshot,
)
from .rollout_modes import (
    ReadOnlyActionSource,
    RolloutMode,
    RolloutModePolicy,
    RolloutModeValidationError,
    RolloutSummaryRecorder,
    write_rollout_summary,
)
from .safety import SafetyGate
from .servo_command_client import CommandIntent, ServoCommandClient
from .arm_init_control import (
    ArmInitOverrideController,
    apply_source_override_transitions,
    apply_source_arm_mask,
    intent_uses_source_requirements,
    source_arm_mask_copy,
)
from .record_control import RecordingSupervisor
from .spacemouse import HidSpaceMouseReader, ScriptedSpaceMouseReader, SpaceMouseReader, SpaceMouseSample
from .spacemouse_registry import RegistrySpaceMouseReader, SpaceMouseDeviceRegistry
from .action_sources.umi_dual_cartesian import MockUmiPoseReader, UdpUmiPoseReader, UmiPoseReader


STARTUP_TIMEOUT_EXIT_CODE = 2
LEASE_READBACK_TIMEOUT_EXIT_CODE = 3
FAULT_LATCH_EXIT_CODE = 4
FORCE_RECOVERY_TIMEOUT_EXIT_CODE = 5

# Quiet period (no motion intents) after which the teleop loop voluntarily
# releases the command-source lease so one-shot GUI commands can run between
# teleop bursts; the next motion intent re-acquires lazily.
IDLE_LEASE_RELEASE_SEC = 1.0
LEASE_RETRY_BACKOFF_SEC = 2.0

# Per-arm ABSOLUTE gripper close-bias fallbacks (opening percent subtracted from
# the model's opening target so a marginal grasp clamps). The two pika grippers
# clamp differently, so left/right default to independent values; these apply when
# neither the per-arm flag (--gripper-close-bias-left/right) nor the shared
# --gripper-close-bias is given. Mirrored in the flow-infer arg help text below.
DEFAULT_GRIPPER_CLOSE_BIAS_LEFT = 2.0
DEFAULT_GRIPPER_CLOSE_BIAS_RIGHT = 6.0


def _intent_record_packet(
    command_client: ServoCommandClient | object | None,
    intent: CommandIntent,
    seq: int,
) -> dict[str, object]:
    build_packet = getattr(command_client, "build_packet", None)
    if callable(build_packet):
        try:
            return build_packet(intent, seq)
        except Exception:
            pass
    packet: dict[str, object] = {
        "seq": int(seq),
        "mode": intent.mode,
        "timeout_sec": intent.timeout_sec,
        "coupled_timeout": intent.coupled_timeout,
    }
    if intent.left is not None:
        packet["left"] = intent.left
    if intent.right is not None:
        packet["right"] = intent.right
    return packet


def _active_lease_loss_reason(
    snapshot: StateSnapshot,
    command_client: object | None,
) -> tuple[str, bool] | None:
    """Return (reason, foreign_owner_active) when our cached lease is no longer valid."""
    if command_client is None:
        return None
    source_id = getattr(command_client, "source_id", None)
    session_id = getattr(command_client, "session_id", None)
    if not isinstance(source_id, str) or not source_id:
        return None
    if not isinstance(session_id, str) or not session_id:
        return None
    lease = command_source_lease_from_snapshot(snapshot)
    if not lease.enforce_lease:
        return None
    lease_token = getattr(command_client, "lease_token", None)
    if lease.matches(source_id, session_id, lease_token):
        return None
    if not lease.active:
        if lease.expired:
            return ("server reports our command-source lease expired", False)
        return ("server reports no active command-source lease", False)
    owner = f"{lease.active_source_id or '<unknown>'}/{lease.active_session_id or '<unknown>'}"
    if lease.active_source_id == source_id and lease.active_session_id == session_id:
        return ("server reports our session with a different lease token", False)
    return (f"server lease is held by {owner}", True)


def _log_final_intent(
    source: object,
    intent: CommandIntent | None,
    snapshot: StateSnapshot,
    arm_init_override: ArmInitOverrideController,
    decision: object,
    *,
    sent: bool,
    command_seq: int = 0,
    drop_reason: str | None = None,
) -> None:
    hook = getattr(source, "log_final_intent", None)
    if not callable(hook):
        return
    try:
        hook(
            intent,
            snapshot,
            arm_init_override=arm_init_override,
            decision_allowed=bool(getattr(decision, "allowed", False)),
            sent=sent,
            command_seq=int(command_seq),
            drop_reason=drop_reason,
        )
    except Exception as exc:  # noqa: BLE001 - debug logging must never break control
        print(f"policy_runner final_intent_log_failed: {exc}", file=sys.stderr)


def _runner_role(config: PolicyRunnerConfig, source: object, action_source_name: str) -> str:
    explicit = str(getattr(source, "runner_role", "") or "")
    if explicit in {"stack", "flow_infer", "unknown"}:
        return explicit
    haystack = " ".join(
        str(value)
        for value in (
            action_source_name,
            config.action_source,
            getattr(source, "command_family", ""),
            getattr(source, "policy_label", ""),
            type(source).__name__,
        )
    ).lower()
    if (
        "flow" in haystack
        or "openpi" in haystack
        or "directbc" in haystack
        or "pointcloud" in haystack
    ):
        return "flow_infer"
    if action_source_name and action_source_name != "hold":
        return "stack"
    return "unknown"


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0].startswith("-"):
        parser = argparse.ArgumentParser(description="robotics_lab policy_runner")
        parser.add_argument("--config", required=True, help="policy_runner YAML config")
        parser.add_argument(
            "--action-source",
            default=None,
            help=(
                "override config.action_source (e.g. teleop_mux, "
                "dual_spacemouse_pose_target, umi_dual_cartesian) — debug aid to "
                "isolate one teleop source; the stack default is teleop_mux"
            ),
        )
        parser.add_argument(
            "--verbose",
            action="store_true",
            help="print live teleop input (SpaceMouse/UMI) and loop send/drop stats",
        )
        args = parser.parse_args(argv)
        if args.verbose:
            # The action sources and the run loop read this env at construction.
            os.environ["POLICY_RUNNER_TELEOP_DEBUG"] = "1"
        config = load_config(args.config)
        if args.action_source:
            config = dataclasses.replace(config, action_source=args.action_source)
        return run(config)
    return _main_with_subcommands(argv)


def run(
    config: PolicyRunnerConfig,
    *,
    state_client: RobotStateClient | None = None,
    command_client: ServoCommandClient | None = None,
    source: object | None = None,
    sleep_fn=time.sleep,
    monotonic_fn=time.monotonic,
    stderr: TextIO = sys.stderr,
    state_sink: Callable[[StateSnapshot], None] | None = None,
    send_commands: bool = True,
    rollout_recorder: RolloutSummaryRecorder | None = None,
    geometry_status: GeometryStatus | None = None,
    recording_supervisor: RecordingSupervisor | None = None,
) -> int:
    state_client = state_client or RobotStateClient(config.robot_state.bind, config.robot_state.stale_timeout_sec)
    if send_commands:
        command_client = command_client or ServoCommandClient(
            config.servo_command.endpoint,
            config.servo_command.timeout_sec,
        )
    source = source or make_action_source(config)
    abort_on_fault_latch = (
        _runner_role(config, source, str(getattr(source, "name", config.action_source)))
        == "flow_infer"
    )
    safety_gate = SafetyGate(
        config.mode,
        config.safety,
        config.robot_state.stale_timeout_sec,
        geometry_status=geometry_status or _load_runtime_geometry_status(config),
    )
    state_client.start()
    armed_for_motion = False
    lease_acquired = False
    lease_retry_after = float("-inf")
    last_motion_intent_time: float | None = None
    period = 1.0 / max(config.command_rate_hz, 1.0)
    startup_deadline = monotonic_fn() + max(config.runtime.startup_timeout_sec, 0.0)
    # POLICY_RUNNER_TELEOP_DEBUG=1: 1 Hz loop stats (sent/dropped/no-intent + last drop reason).
    teleop_debug = os.environ.get("POLICY_RUNNER_TELEOP_DEBUG", "") == "1"
    debug_sent = 0
    debug_dropped = 0
    debug_no_intent = 0
    debug_last_drop_reason = ""
    # Lazily initialized from the loop's `now`: tests inject finite scripted
    # monotonic_fn sequences, so no extra monotonic_fn() calls here.
    debug_last_print: float | None = None
    recording_supervisor = recording_supervisor or RecordingSupervisor.from_config(config, stderr=stderr)
    arm_init_override = ArmInitOverrideController(
        timeout_sec=config.servo_command.timeout_sec,
        auto_clear_on_done=config.arm_init_override.auto_clear_on_done,
        auto_clear_on_failed=config.arm_init_override.auto_clear_on_failed,
        resume_flow_on_done=config.arm_init_override.resume_flow_on_done,
        resume_flow_on_failed=config.arm_init_override.resume_flow_on_failed,
        allow_manual_cancel_after_failed=config.arm_init_override.allow_manual_cancel_after_failed,
        reset_flow_source_on_start=config.arm_init_override.reset_flow_source_on_start,
        reset_flow_source_on_resume=config.arm_init_override.reset_flow_source_on_resume,
    )
    source_base_arm_mask = source_arm_mask_copy(source)
    last_record_packet: dict[str, object] | None = None
    last_record_packet_time_ns = 0
    last_record_packet_seq = 0

    def _remember_sent_intent(intent: CommandIntent, seq: int) -> None:
        nonlocal last_record_packet, last_record_packet_time_ns, last_record_packet_seq
        last_record_packet = _intent_record_packet(command_client, intent, seq)
        last_record_packet_time_ns = time.time_ns()
        last_record_packet_seq = int(seq)

    def _remember_unsent_intent(intent: CommandIntent) -> None:
        nonlocal last_record_packet, last_record_packet_time_ns, last_record_packet_seq
        last_record_packet = _intent_record_packet(command_client, intent, 0)
        last_record_packet_time_ns = time.time_ns()
        last_record_packet_seq = 0

    def _clear_tick_record_packet() -> None:
        nonlocal last_record_packet, last_record_packet_time_ns, last_record_packet_seq
        last_record_packet = None
        last_record_packet_time_ns = 0
        last_record_packet_seq = 0

    def _finish_tick(snapshot: StateSnapshot, now_value: float) -> int | None:
        action_source_name = str(getattr(source, "name", config.action_source))
        _t = monotonic_fn() if _prof.on else 0.0
        recording_supervisor.record_frame(
            snapshot,
            action_packet=last_record_packet,
            action_host_time_ns=last_record_packet_time_ns,
            action_seq=last_record_packet_seq,
        )
        if _prof.on:
            _prof.add("record_frame", monotonic_fn() - _t)
            _t = monotonic_fn()
        recording_supervisor.stamp_snapshot(snapshot)
        arm_init_override.stamp_snapshot(snapshot)
        if _prof.on:
            _prof.add("stamp2", monotonic_fn() - _t)
            _t = monotonic_fn()
        recording_supervisor.publish_status(
            now_monotonic=now_value,
            arm_init=arm_init_override.status_block(),
            runner_role=_runner_role(config, source, action_source_name),
            action_source=action_source_name,
            command_client=command_client,
            spacemouse=_spacemouse_status(source),
            force_recovery=_force_recovery_status(source),
            camera_runtime=_camera_runtime_status(source),
        )
        if _prof.on:
            _prof.add("publish_status", monotonic_fn() - _t)
        abort_reason = _terminal_abort_reason(source)
        if abort_reason is not None:
            label = "camera_abort" if abort_reason == "camera_stale_timeout" else "force_recovery_abort"
            print(f"policy_runner {label}: {abort_reason}", file=stderr)
            return FORCE_RECOVERY_TIMEOUT_EXIT_CODE
        completion_reason = _completion_reason(source)
        if completion_reason is not None:
            print(f"policy_runner completed: {completion_reason}", file=stderr)
            return 0
        _ts = monotonic_fn() if _prof.on else 0.0
        sleep_fn(period)
        if _prof.on:
            _prof.add("sleep", monotonic_fn() - _ts)
        return None
    _capture = TeleopCaptureLogger()

    def _log_final_and_capture(
        intent: CommandIntent | None,
        snapshot: StateSnapshot,
        decision: object,
        now_value: float,
        *,
        sent: bool,
        command_seq: int = 0,
        arm_seq: int = 0,
        drop_reason: str | None = None,
    ) -> None:
        _log_final_intent(
            source,
            intent,
            snapshot,
            arm_init_override,
            decision,
            sent=sent,
            command_seq=command_seq,
            drop_reason=drop_reason,
        )
        _capture.log(
            now_value,
            source,
            intent,
            getattr(decision, "allowed", False),
            phase="final",
            sent=sent,
            command_seq=command_seq,
            arm_seq=arm_seq,
            drop_reason=drop_reason,
        )

    # Tick profiler (FLOW_INFER_TICK_PROFILE=1; telemetry only, off by default).
    #
    # Why: with policy_dt = 33.4 ms (SPEED_SCALE=1.0) the rollout only achieved a ~45 ms
    # step period, while the MINIMUM observed step was exactly 33.4 ms -- so the loop can
    # reach 30 Hz but usually misses the deadline. Cameras (29.7 fps, bundle age 21 ms) and
    # inference (94 ms of a 235 ms budget) were both ruled out, which leaves this Python
    # tick. _step_deadline is reset from the CURRENT time, so lateness is absorbed rather
    # than caught up: the achieved rate silently settles at whatever the tick can sustain.
    # This names the phase responsible instead of guessing.
    class _TickProfile:
        __slots__ = ("on", "t_prev", "acc", "n", "last_report", "worst", "cur", "spike_sec")

        def __init__(self) -> None:
            self.on = os.environ.get("FLOW_INFER_TICK_PROFILE", "0") == "1"
            self.t_prev = None
            self.acc: dict[str, float] = {}
            self.n = 0
            self.last_report = None
            self.worst: dict[str, float] = {}
            # Per-tick phase durations, dumped immediately when one tick blows
            # past the spike threshold. The 5 s aggregate above can only say
            # WHICH phase held the worst single tick; this says which phases the
            # offending tick itself spent its time in (the 60-150 ms loop
            # freezes observed 2026-08-14 skip a chunk and stall the arm, and
            # arrive rarely enough that the aggregate hides them).
            self.cur: dict[str, float] = {}
            # 50 ms: the 20260814_145123 run had two 67-76 ms loop stalls that each
            # slipped a policy step and cost a hold tick, sitting just under the
            # original 80 ms threshold.
            self.spike_sec = float(os.environ.get("FLOW_INFER_TICK_SPIKE_MS", "50")) / 1000.0

        def tick(self, t: float) -> None:
            if not self.on:
                return
            if self.t_prev is not None:
                total = t - self.t_prev
                self.add("TOTAL", total)
                self.n += 1
                if total >= self.spike_sec:
                    named = sum(v for k, v in self.cur.items() if k != "TOTAL")
                    parts = " ".join(
                        f"{k}={v * 1000:.1f}"
                        for k, v in sorted(self.cur.items(), key=lambda kv: -kv[1])
                        if k != "TOTAL"
                    )
                    print(
                        f"[tick-spike] dt={total * 1000:.0f}ms {parts} "
                        f"OTHER={(total - named) * 1000:.1f}",
                        file=stderr,
                        flush=True,
                    )
            self.cur = {}
            self.t_prev = t
            if self.last_report is None:
                self.last_report = t
            elif t - self.last_report >= 5.0 and self.n:
                # "other" = the part of the tick no timer covers. Chasing it is the point:
                # the first profiling pass had TOTAL=9.0 ms with every named phase <=0.7 ms.
                total = self.acc.get("TOTAL", 0.0)
                named = sum(v for k, v in self.acc.items() if k != "TOTAL")
                parts = " ".join(
                    f"{k}={self.acc[k] / self.n * 1000:.2f}/{self.worst[k] * 1000:.0f}"
                    for k in sorted(self.acc, key=lambda k: -self.acc[k])
                )
                other = (total - named) / self.n * 1000
                print(
                    f"[tick-profile] n={self.n} mean/max ms  {parts} OTHER={other:.2f}",
                    file=stderr,
                    flush=True,
                )
                self.acc.clear()
                self.worst.clear()
                self.n = 0
                self.last_report = t

        def add(self, key: str, dt: float) -> None:
            if not self.on:
                return
            self.acc[key] = self.acc.get(key, 0.0) + dt
            if key != "TOTAL":
                self.cur[key] = self.cur.get(key, 0.0) + dt
            if dt > self.worst.get(key, 0.0):
                self.worst[key] = dt

    _prof = _TickProfile()
    if _prof.on:
        print("[tick-profile] enabled (mean/max ms per phase, every 5 s)", file=stderr, flush=True)

    try:
        while True:
            _tick_t0 = monotonic_fn() if _prof.on else 0.0
            snapshot = state_client.latest
            now = monotonic_fn()
            _prof.tick(_tick_t0 if _prof.on else now)
            if _prof.on:
                _prof.add("snapshot", now - _tick_t0)
            if snapshot is None:
                if now >= startup_deadline:
                    print(
                        "policy_runner startup_timeout_no_state: "
                        f"no robot state received within {config.runtime.startup_timeout_sec:.3f}s "
                        f"on {config.robot_state.bind}",
                        file=stderr,
                    )
                    return STARTUP_TIMEOUT_EXIT_CODE
                sleep_fn(period)
                continue
            if abort_on_fault_latch:
                fault_latch = fault_latch_from_snapshot(snapshot)
                if fault_latch.latched:
                    fields = []
                    if fault_latch.motion_state is not None:
                        fields.append(f"motion_state={fault_latch.motion_state}")
                    if fault_latch.latched_fault_reason is not None:
                        fields.append(f"verdict={fault_latch.latched_fault_reason}")
                    if fault_latch.reason is not None:
                        fields.append(f"reason={fault_latch.reason}")
                    suffix = f" {' '.join(fields)}" if fields else ""
                    print(f"policy_runner fault_latch_abort:{suffix}", file=stderr)
                    return FAULT_LATCH_EXIT_CODE
            _t0 = monotonic_fn() if _prof.on else 0.0
            action_source_name = str(getattr(source, "name", config.action_source))
            drain_payloads = getattr(recording_supervisor, "drain_control_payloads", None)
            dispatch_payloads = getattr(recording_supervisor, "dispatch_control_payloads", None)
            if callable(drain_payloads) and callable(dispatch_payloads):
                control_payloads = drain_payloads()
                dispatch_payloads(control_payloads, snapshot, action_source=action_source_name)
                arm_init_override.handle_payloads(control_payloads)
                _handle_spacemouse_control_payloads(source, control_payloads)
            else:
                recording_supervisor.drain_commands(snapshot, action_source=action_source_name)
            arm_init_override.update_from_snapshot(snapshot)
            transitions = arm_init_override.consume_transitions()
            apply_source_override_transitions(
                source,
                transitions,
                snapshot,
                reset_on_start=config.arm_init_override.reset_flow_source_on_start,
                reset_on_resume=config.arm_init_override.reset_flow_source_on_resume,
                reset_on_failed=config.arm_init_override.resume_flow_on_failed,
            )
            apply_source_arm_mask(source, source_base_arm_mask, arm_init_override)
            recording_supervisor.stamp_snapshot(snapshot)
            arm_init_override.stamp_snapshot(snapshot)
            _clear_tick_record_packet()
            if _prof.on:
                _prof.add("pre_intent", monotonic_fn() - _t0)
            _t0 = monotonic_fn() if _prof.on else 0.0
            if state_sink is not None:
                state_sink(snapshot)
            if _prof.on:
                _prof.add("state_sink", monotonic_fn() - _t0)
            _t0 = monotonic_fn() if _prof.on else 0.0
            if rollout_recorder is not None:
                rollout_recorder.record_state(snapshot)
            if _prof.on:
                _prof.add("record_state", monotonic_fn() - _t0)
            _t0 = monotonic_fn() if _prof.on else 0.0
            intent = source.next_intent(snapshot, now)
            if _prof.on:
                _prof.add("next_intent", monotonic_fn() - _t0)
            intent = arm_init_override.compose_intent(intent)
            source_requirements = getattr(source, "requirements", None)
            requirements = (
                source_requirements
                if intent_uses_source_requirements(intent, arm_init_override)
                else None
            )
            _t0 = monotonic_fn() if _prof.on else 0.0
            decision = safety_gate.evaluate(snapshot, intent, requirements, now)
            if _prof.on:
                _prof.add("safety_gate", monotonic_fn() - _t0)
            _capture.log(now, source, intent, decision.allowed, phase="pre")
            if rollout_recorder is not None:
                rollout_recorder.record_decision(decision)
                rollout_recorder.record_source(source)
            if teleop_debug:
                if intent is None:
                    debug_no_intent += 1
                elif not decision.allowed:
                    debug_dropped += 1
                    debug_last_drop_reason = decision.reason or ""
                if debug_last_print is None:
                    debug_last_print = now
                if now - debug_last_print >= 1.0:
                    print(
                        f"[teleop] sent={debug_sent} dropped={debug_dropped} "
                        f"no_intent={debug_no_intent} last_drop={debug_last_drop_reason or '-'}",
                        flush=True,
                    )
                    debug_last_print = now
            # Idle lease handoff: release the lease after a short quiet period
            # (no motion intents) so one-shot GUI commands work BETWEEN teleop
            # bursts instead of being rejected with
            # lease_conflict for up to lease_timeout_sec (60s). The lazy-acquire
            # block below re-acquires on the next motion intent. This also fixes
            # resume-after-expiry: once the server lease expired, the stale
            # lease_acquired flag made resumed teleop commands be dropped
            # (lease_required) with no re-acquire.
            if intent is not None and intent.is_motion:
                last_motion_intent_time = now
            if send_commands and lease_acquired and config.servo_command.acquire_lease:
                assert command_client is not None
                loss = _active_lease_loss_reason(snapshot, command_client)
                if loss is not None:
                    loss_reason, foreign_owner_active = loss
                    print(
                        f"policy_runner lease lost (will reacquire): {loss_reason}",
                        file=stderr,
                    )
                    lease_acquired = False
                    armed_for_motion = False
                    if hasattr(command_client, "lease_token"):
                        command_client.lease_token = None
                    lease_retry_after = (
                        now + LEASE_RETRY_BACKOFF_SEC
                        if foreign_owner_active
                        else float("-inf")
                    )
            if (
                (intent is None or not intent.is_motion)
                and send_commands
                and lease_acquired
                and config.servo_command.acquire_lease
                and last_motion_intent_time is not None
                and now - last_motion_intent_time >= IDLE_LEASE_RELEASE_SEC
            ):
                assert command_client is not None
                try:
                    command_client.release_lease()
                except Exception as exc:  # noqa: BLE001 - best-effort handoff
                    print(f"policy_runner idle lease_release_failed: {exc}", file=stderr)
                lease_acquired = False
                lease_retry_after = float("-inf")
            if decision.allowed and intent is not None:
                if not send_commands:
                    if rollout_recorder is not None:
                        rollout_recorder.record_dropped("rollout_mode_command_send_disabled", intent)
                    _remember_unsent_intent(intent)
                    _log_final_and_capture(
                        intent,
                        snapshot,
                        decision,
                        now,
                        sent=False,
                        command_seq=0,
                        drop_reason="rollout_mode_command_send_disabled",
                    )
                    abort_code = _finish_tick(snapshot, now)
                    if abort_code is not None:
                        return abort_code
                    continue
                assert command_client is not None
                # Lazy lease: acquire on the FIRST motion intent, not at startup.
                # An idle policy_runner must not camp on the lease. On conflict /
                # readback timeout, drop this tick and retry with backoff so a
                # temporary GUI lease holder does not kill the teleop process.
                if config.servo_command.acquire_lease and not lease_acquired and intent.is_motion:
                    if now < lease_retry_after:
                        _log_final_and_capture(
                            intent,
                            snapshot,
                            decision,
                            now,
                            sent=False,
                            command_seq=0,
                            drop_reason="lease_retry_backoff",
                        )
                        abort_code = _finish_tick(snapshot, now)
                        if abort_code is not None:
                            return abort_code
                        continue
                    try:
                        command_client.acquire_lease(
                            StateStreamLeaseReadback(state_client),
                            timeout_sec=config.servo_command.lease_readback_timeout_sec,
                            monotonic_fn=monotonic_fn,
                            sleep_fn=sleep_fn,
                        )
                        lease_acquired = True
                    except TimeoutError as exc:
                        print(
                            f"policy_runner lease busy (will retry): {exc}",
                            file=stderr,
                        )
                        lease_retry_after = now + LEASE_RETRY_BACKOFF_SEC
                        _log_final_and_capture(
                            intent,
                            snapshot,
                            decision,
                            now,
                            sent=False,
                            command_seq=0,
                            drop_reason="lease_busy",
                        )
                        abort_code = _finish_tick(snapshot, now)
                        if abort_code is not None:
                            return abort_code
                        continue
                arm_seq = 0
                if intent.is_motion and not armed_for_motion:
                    arm = CommandIntent.arm_motion(timeout_sec=config.servo_command.timeout_sec)
                    arm_decision = safety_gate.evaluate(snapshot, arm, getattr(source, "requirements", None), now)
                    if rollout_recorder is not None:
                        rollout_recorder.record_decision(arm_decision)
                    if arm_decision.allowed:
                        arm_seq = command_client.send(arm)
                        _remember_sent_intent(arm, arm_seq)
                        if rollout_recorder is not None:
                            rollout_recorder.record_sent(arm)
                        armed_for_motion = True
                    else:
                        if rollout_recorder is not None:
                            rollout_recorder.record_dropped(arm_decision.reason, arm)
                        if teleop_debug:
                            debug_dropped += 1
                            debug_last_drop_reason = f"arm:{arm_decision.reason or ''}"
                        _log_final_and_capture(
                            intent,
                            snapshot,
                            decision,
                            now,
                            sent=False,
                            command_seq=0,
                            drop_reason=f"arm:{arm_decision.reason or ''}",
                        )
                        abort_code = _finish_tick(snapshot, now)
                        if abort_code is not None:
                            return abort_code
                        continue
                _t0 = monotonic_fn() if _prof.on else 0.0
                seq = command_client.send(intent)
                if _prof.on:
                    _prof.add("cmd_send", monotonic_fn() - _t0)
                _remember_sent_intent(intent, seq)
                _log_final_and_capture(
                    intent,
                    snapshot,
                    decision,
                    now,
                    sent=True,
                    command_seq=seq,
                    arm_seq=arm_seq,
                    drop_reason=None,
                )
                if teleop_debug:
                    debug_sent += 1
                if rollout_recorder is not None:
                    rollout_recorder.record_sent(intent)
            elif intent is not None:
                if rollout_recorder is not None:
                    rollout_recorder.record_dropped(decision.reason, intent)
                _log_final_and_capture(
                    intent,
                    snapshot,
                    decision,
                    now,
                    sent=False,
                    command_seq=0,
                    drop_reason=decision.reason,
                )
            abort_code = _finish_tick(snapshot, now)
            if abort_code is not None:
                return abort_code
    except KeyboardInterrupt:
        return 0
    finally:
        _capture.close()
        recording_supervisor.close()
        _close_if_supported(source)
        state_client.close()
        if command_client is not None:
            if lease_acquired:
                # Voluntary handoff so an immediate restart does not collide
                # with this session's stale lease until lease_timeout_sec.
                try:
                    command_client.release_lease()
                except Exception as exc:  # noqa: BLE001 - best-effort on shutdown
                    print(f"policy_runner lease_release_failed: {exc}", file=stderr)
            command_client.close()


def make_action_source(config: PolicyRunnerConfig):
    if config.action_source == "hold":
        return HoldActionSource(timeout_sec=config.servo_command.timeout_sec)
    if config.action_source == "joint_sine":
        return JointSineActionSource(
            amplitude_deg=config.joint_sine.amplitude_deg,
            frequency_hz=config.joint_sine.frequency_hz,
            selected_arm=config.joint_sine.selected_arm,
            timeout_sec=config.servo_command.timeout_sec,
            simulation_only=config.joint_sine.simulation_only,
        )
    if config.action_source == "dual_spacemouse_pose_target":
        return _make_dual_spacemouse_pose_target_source(config)
    if config.action_source == "umi_dual_cartesian":
        return _make_umi_dual_cartesian_source(config)
    if config.action_source == "teleop_mux":
        return TeleopMuxActionSource(
            _make_dual_spacemouse_pose_target_source(config),
            _make_umi_dual_cartesian_source(config),
            tie_break=config.teleop_mux.tie_break,
        )
    raise ValueError(f"unknown action_source: {config.action_source}")


def _make_dual_spacemouse_pose_target_source(
    config: PolicyRunnerConfig,
) -> DualSpaceMousePoseTargetActionSource:
    sm = config.spacemouse_pose_target_dual
    left = sm.left
    right = sm.right
    registry = None
    if sm.discovery.enable and left.mock_script is None and right.mock_script is None:
        registry = SpaceMouseDeviceRegistry(
            vendor_id=sm.discovery.vendor_id,
            product_id=sm.discovery.product_id,
            interface_number=sm.discovery.interface_number,
            scan_period_sec=sm.discovery.scan_period_sec,
            poll_period_sec=sm.discovery.poll_period_sec,
            neutral_threshold=sm.deadband,
        )
        left_reader: SpaceMouseReader = RegistrySpaceMouseReader(registry, "left")
        right_reader: SpaceMouseReader = RegistrySpaceMouseReader(registry, "right")
    else:
        left_reader = _spacemouse_reader_from_device_config(left)
        right_reader = _spacemouse_reader_from_device_config(right)
    return DualSpaceMousePoseTargetActionSource(
        left_reader=left_reader,
        right_reader=right_reader,
        max_linear_step_m=sm.max_linear_step_m,
        max_angular_step_rad=sm.max_angular_step_rad,
        max_target_lead_m=sm.max_target_lead_m,
        max_target_lead_rad=sm.max_target_lead_rad,
        deadband=sm.deadband,
        activation_deadband=sm.activation_deadband,
        response_curve_gamma=sm.response_curve_gamma,
        linear_axis_signs=sm.linear_axis_signs,
        angular_axis_signs=sm.angular_axis_signs,
        angular_axis_order=sm.angular_axis_order,
        sample_stale_timeout_sec=sm.sample_stale_timeout_sec,
        require_deadman=sm.require_deadman,
        startup_requires_neutral=sm.startup_requires_neutral,
        startup_neutral_hold_sec=sm.startup_neutral_hold_sec,
        left_deadman_button=left.deadman_button,
        right_deadman_button=right.deadman_button,
        gripper_buttons_enable=sm.gripper_buttons.enable,
        gripper_open_button=sm.gripper_buttons.open_button,
        gripper_close_button=sm.gripper_buttons.close_button,
        gripper_open_percent=sm.gripper_buttons.open_percent,
        gripper_close_percent=sm.gripper_buttons.close_percent,
        timeout_sec=config.servo_command.timeout_sec,
        allow_rbpodo_controller_simulation=(
            config.safety.allow_rbpodo_controller_simulation_cartesian
        ),
        device_registry=registry,
    )


def _spacemouse_status(source: object):
    status = getattr(source, "spacemouse_status", None)
    return status() if callable(status) else None


def _force_recovery_status(source: object) -> dict[str, object] | None:
    status = getattr(source, "force_recovery_status", None)
    if not callable(status):
        return None
    try:
        return dict(status())
    except Exception:
        return None


def _camera_runtime_status(source: object) -> dict[str, object] | None:
    status = getattr(source, "camera_runtime_status", None)
    if not callable(status):
        return None
    try:
        return dict(status())
    except Exception:
        return None


def _terminal_abort_reason(source: object) -> str | None:
    value = getattr(source, "terminal_abort_reason", None)
    if value is None:
        value = getattr(source, "force_recovery_terminal_abort_reason", None)
    if value is None:
        return None
    resolved = str(value).strip()
    return resolved or None


def _completion_reason(source: object) -> str | None:
    hook = getattr(source, "completion_reason", None)
    if not callable(hook):
        return None
    value = hook()
    if value is None:
        return None
    resolved = str(value).strip()
    return resolved or None


def _handle_spacemouse_control_payloads(source: object, payloads) -> None:
    handler = getattr(source, "handle_spacemouse_control", None)
    if not callable(handler):
        return
    for payload in payloads:
        handler(payload)


def _make_umi_dual_cartesian_source(config: PolicyRunnerConfig) -> UmiDualCartesianActionSource:
    umi = config.umi_dual_cartesian
    return UmiDualCartesianActionSource(
        left_reader=_umi_reader_from_config(umi.left, "left"),
        right_reader=_umi_reader_from_config(umi.right, "right"),
        max_linear_step_m=umi.max_linear_step_m,
        max_angular_step_rad=umi.max_angular_step_rad,
        input_moving_average_window=umi.input_moving_average_window,
        deadband_linear_m=umi.deadband_linear_m,
        deadband_angular_rad=umi.deadband_angular_rad,
        linear_axis_signs=umi.linear_axis_signs,
        angular_axis_signs=umi.angular_axis_signs,
        gripper_offset=umi.gripper_offset,
        r_align=umi.r_align,
        workspace_bounds=umi.workspace_bounds,
        sample_hold_timeout_sec=umi.sample_hold_timeout_sec,
        timeout_sec=config.servo_command.timeout_sec,
        deadman_release_grace_sec=umi.deadman_release_grace_sec,
        tcp_pose_target_conditioning=umi.tcp_pose_target_conditioning,
        target_lead_clamp=umi.target_lead_clamp,
    )


def _spacemouse_reader_from_device_config(device_config) -> SpaceMouseReader:
    if device_config.mock_script is not None:
        return ScriptedSpaceMouseReader(device_config.mock_script)
    return _LazyHidSpaceMouseReader(
        device=device_config.device,
        path=device_config.path,
        device_number=device_config.device_number,
    )


def _umi_reader_from_config(reader_config, side: str) -> UmiPoseReader:
    if reader_config.mock_script is not None:
        return MockUmiPoseReader(reader_config.mock_script)
    if reader_config.endpoint:
        return UdpUmiPoseReader(reader_config.endpoint, side)
    return MockUmiPoseReader("pgmode_umi_smoke")


def _parse_rotation_axes(args) -> tuple[bool, bool, bool]:
    """Resolve which per-arm rotation axes (rx, ry, rz) the policy may command
    from --rotation-axes / --translation-only into a 3-tuple of keep flags.
    --translation-only forces all-off; otherwise --rotation-axes is a subset of
    x/y/z to keep ('none'/'' = keep none, 'all'/'xyz' = keep all)."""
    if bool(getattr(args, "translation_only", False)):
        return (False, False, False)
    spec = str(getattr(args, "rotation_axes", "xyz") or "").strip().lower()
    if spec in ("none", "off"):
        return (False, False, False)
    if spec == "all":
        return (True, True, True)
    invalid = sorted(set(spec) - set("xyz"))
    if invalid:
        raise SystemExit(
            f"--rotation-axes: invalid axis {invalid}; use any subset of x,y,z or 'none'"
        )
    return ("x" in spec, "y" in spec, "z" in spec)


def _load_runtime_geometry_status(config: PolicyRunnerConfig) -> GeometryStatus:
    if not config.geometry.path:
        return GeometryStatus.unavailable("geometry_path_missing")
    return load_geometry_status(config.geometry.path)


def _close_if_supported(value: object) -> None:
    close = getattr(value, "close", None)
    if callable(close):
        close()


class _LazyHidSpaceMouseReader(SpaceMouseReader):
    def __init__(
        self,
        *,
        device: str | None = None,
        path: str | None = None,
        device_number: int = 0,
    ):
        self._device = device
        self._path = path
        self._device_number = device_number
        self._reader: HidSpaceMouseReader | None = None

    def read(self, timeout_sec: float | None = None) -> SpaceMouseSample | None:
        if self._reader is None:
            self._reader = HidSpaceMouseReader(
                device=self._device,
                path=self._path,
                device_number=self._device_number,
            )
        return self._reader.read(timeout_sec=timeout_sec)

    def close(self) -> None:
        if self._reader is not None:
            self._reader.close()
            self._reader = None


def _main_with_subcommands(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="robotics_lab policy_runner")
    sub = parser.add_subparsers(dest="command", required=True)

    hdf5_record = sub.add_parser(
        "hdf5-record",
        help="Teleop and record episodes to ACT-compatible HDF5 files.",
    )
    hdf5_record.add_argument("--config", required=True, help="policy_runner YAML config")
    hdf5_record.add_argument("--output-dir", default=None, help="Override recording.output_dir from config")
    hdf5_record.add_argument("--task", required=True, help="Task description for this batch")
    hdf5_record.add_argument("--operator", default=None, help="Operator ID")
    hdf5_record.add_argument("--rate", type=float, default=None, help="Override recording.rate_hz from config")
    hdf5_record.add_argument("--with-camera", action="store_true", help="Record camera.bundle images")
    hdf5_record.add_argument("--zmq-endpoint", default=None, help="Override camera.zmq_endpoint")

    flow_infer = sub.add_parser(
        "flow-infer",
        help="Run an external OpenPI action-chunk policy server as a live action source.",
    )
    flow_infer.add_argument(
        "--checkpoint",
        default="openpi://127.0.0.1:8000",
        help=(
            "openpi://HOST:PORT 로 서빙되는 외부 OpenPI 정책 서버. 사내 학습 체크포인트"
            " 스택은 제거되었으므로 로컬 .pt 경로는 받지 않는다."
        ),
    )
    flow_infer.add_argument("--config", required=True, help="policy_runner YAML config")
    flow_infer.add_argument(
        "--rollout-mode",
        required=True,
        choices=RolloutMode.choices(),
        help=(
            "Where inferred flow-policy actions may go: sim_dryrun, "
            "controller_sim, real_readonly, or real_policy."
        ),
    )
    flow_infer.add_argument(
        "--send-dryrun-commands",
        action="store_true",
        help="Test-only override allowing sim_dryrun to send UDP commands.",
    )
    flow_infer.add_argument(
        "--rollout-summary",
        default="outputs/rollout_summary.json",
        help="Path for rollout_summary JSON output.",
    )
    flow_infer.add_argument(
        "--rollout-step-log",
        default=None,
        help=(
            "Optional per-policy-step JSONL telemetry path. File I/O runs on a "
            "best-effort background writer and is disabled on logging failure."
        ),
    )
    flow_infer.add_argument(
        "--training-episode-hdf5",
        default=None,
        help=(
            "Controller-simulation only: teacher-force an OpenPI remote policy with the saved "
            "RGB-D and velocity proprio from this raw training episode, then execute its predictions."
        ),
    )
    flow_infer.add_argument(
        "--training-episode-retarget-config",
        default="calibration/umi_retarget_eelocal.yaml",
        help="Authoritative tracker-to-TCP retarget YAML used when the training dataset was converted.",
    )
    flow_infer.add_argument(
        "--training-episode-output-dir",
        default="outputs/training_episode_replay",
        help="Directory for teacher-forced model predictions and comparison metrics.",
    )
    flow_infer.add_argument(
        "--training-episode-video-dir",
        default=None,
        help=(
            "Optional directory containing the four final LeRobot training MP4 stream directories. "
            "When omitted, decoded images come from the raw HDF5."
        ),
    )
    flow_infer.add_argument(
        "--training-episode-parquet",
        default=None,
        help="Optional matching LeRobot parquet; state/action equality is validated before motion.",
    )
    flow_infer.add_argument(
        "--training-episode-start-frame",
        type=int,
        default=0,
        help="First training frame used as a teacher-forced chunk anchor.",
    )
    flow_infer.add_argument("--device", default="auto")
    flow_infer.add_argument(
        "--execute-arms",
        choices=("both", "left", "right"),
        default="both",
        help=(
            "Runtime execution mask: suppress the non-selected arm's commands "
            "(twist + gripper) so it physically holds. The checkpoint stays "
            "dual-arm (gate/selected_arms unchanged); only what is SENT to the "
            "servo is masked. Use 'right' to run the right-arm-first phase with "
            "the idle left arm held (avoids idle-arm noise creep)."
        ),
    )
    flow_infer.add_argument(
        "--ee-local-r-align",
        default="pika_rz180",
        help=(
            "Fixed rotation between the training EE body frame and the RB TCP frame for "
            "ee_local checkpoints. Preset name or 9 row-major floats. Presets: "
            "'pika_rz180' (measured pika-UMI correction, 180deg about approach(z) on BOTH "
            "translation and rotation; pika UMI always uses this), 'pika_rz180_trans_only' "
            "(ablation: flip x/y translation only, leave rotation unchanged). "
            "Default: pika_rz180 (frames assumed identical)."
        ),
    )
    flow_infer.add_argument(
        "--proprio-mode",
        default="velocity",
        choices=("pose", "velocity", "velocity_grip", "velocity_grav"),
        help=(
            "observation/state representation sent to an openpi server; MUST match the served "
            "checkpoint's training distribution (openpi convert --state-mode). 'pose' (default) = "
            "14-D reset-relative pose; 'velocity' = 12-D ee_local velocity (init-pose-independent, "
            "no gripper); 'velocity_grip' = 14-D ee_local velocity + absolute gripper; 'velocity_grav' = "
            "20-D ee_local velocity + gravity-tilt anchor + absolute gripper [pos_vel3, rot_vel3, "
            "gravity3, grip] x L,R (gravity = world-down in the tool frame, yaw-invariant). velocity* are "
            "finite-differenced from the robot TCP pose (use a small --chunk-execute-steps for the "
            "cleanest per-step velocity). nostate (zero_state) checkpoints ignore state, so any value "
            "works there. Only the openpi-remote source uses this."
        ),
    )
    flow_infer.add_argument(
        "--velproprio-sample-mode",
        default="replan",
        choices=("replan", "fixed_step", "camera_frame"),
        help=(
            "How velocity* proprio picks its finite-difference window (openpi-remote only). 'replan' "
            "(default, legacy): difference the TCP pose between successive proprio samples (chunk-replan "
            "boundaries) and rescale by policy_dt/wall_dt — the window length AND wall-clock both depend "
            "on controller/inference timing, so a slower/burstier controller (or SEQUENTIAL holds) reports "
            "a SMALLER velocity than training saw. 'fixed_step': difference the MEASURED pose over a fixed "
            "~policy_dt window from a per-tick pose history, decoupled from replan cadence and inference "
            "latency — reproduces the training converter's single 30 Hz frame delta regardless of the "
            "controller. 'camera_frame': measured TCP local delta over [camera_time - policy_dt, "
            "camera_time], no dt normalization, closest to OpenPI/UMI converter semantics."
        ),
    )
    flow_infer.add_argument(
        "--velproprio-source",
        default="measured",
        choices=("measured", "command"),
        help=(
            "Source stream for velocity* proprio finite differences (openpi-remote only). "
            "'measured' (default) uses live robot TCP pose. 'command' uses the runner's emitted "
            "absolute TcpPoseTarget stream, matching UMI training semantics where velocity is "
            "finite-differenced from the same trajectory the actions describe. Requires "
            "--velproprio-sample-mode fixed_step and --proprio-mode velocity*."
        ),
    )
    flow_infer.add_argument(
        "--include-depth",
        action="store_true",
        help=(
            "For an RGB-D (include_depth) openpi checkpoint: also send the live D405 depth as "
            "observation/*_wrist_0_depth (z16 -> _depth_to_image, BIT-IDENTICAL to the converter). "
            "Enables z16 decode on the camera bundle client. MUST match the checkpoint's training "
            "(--include-depth converter) AND the same --depth-z-near/far-mm + --depth-units-m."
        ),
    )
    flow_infer.add_argument("--depth-z-near-mm", type=float, default=50.0, help="depth clip near (mm); match training")
    flow_infer.add_argument("--depth-z-far-mm", type=float, default=700.0, help="depth clip far (mm); match training")
    flow_infer.add_argument(
        "--depth-units-m", type=float, default=1e-4,
        help="metres per stored-depth count (Pika D405: 1e-4 = 100um; check collect.log + match training)",
    )
    flow_infer.add_argument(
        "--blank-depth",
        action="store_true",
        help=(
            "DEPTH ABLATION for an RGB-D openpi checkpoint: still send the "
            "*_wrist_0_depth keys (server input transform satisfied, 5-camera token "
            "structure unchanged) but fill them with a constant all-far frame "
            "instead of live depth. If the policy behaves the same as with live "
            "depth, it did not learn to use depth content. Use INSTEAD of "
            "--include-depth; live depth is not read."
        ),
    )
    flow_infer.add_argument(
        "--camera-preview",
        action="store_true",
        default=False,
        help=(
            "Open a live OpenCV window showing the camera frames the policy consumes "
            "(spawns policy_runner.camera_preview alongside the rollout)."
        ),
    )
    flow_infer.add_argument(
        "--policy-dt-sec",
        type=float,
        default=0.0334,
        help=(
            "Seconds represented by one flow action step before --speed-scale. Controller/real "
            "flow rollout requires this value or checkpoint dataset_stats.dt_mean_sec; sim_dryrun "
            "can fall back to 1/command_rate_hz."
        ),
    )
    flow_infer.add_argument(
        "--speed-scale",
        type=float,
        default=1.0,
        help=(
            "Replay policy action steps faster/slower without changing the model: effective "
            "policy_dt = policy_dt_sec / speed_scale, while source-side velocity clamps are "
            "multiplied by speed_scale so the same per-step deltas are not clipped. Use 2.0 "
            "for 2x flow-infer motion after the server flow_infer_smooth profile is tuned."
        ),
    )
    flow_infer.add_argument(
        "--max-linear-velocity-m-s",
        type=float,
        default=0.45,
        help=(
            "Flow action linear per-step clamp in m/s, applied client-side to the model's "
            "output deltas as |Δpos| <= v * policy_dt before integration. Default 0.45 covers "
            "the pika ee_local delta distribution (p99.9 ~= 0.36 m/s); 0.15 clipped ~5%% of "
            "model outputs. openpi-remote carries no checkpoint action stats, so this value is used directly."
        ),
    )
    flow_infer.add_argument(
        "--max-angular-velocity-rad-s",
        type=float,
        default=2.0,
        help="Override flow action angular clamp; omitted uses checkpoint action statistics.",
    )
    flow_infer.add_argument(
        "--chunk-execute-steps",
        type=int,
        default=24,
        help="Number of sampled action steps to execute before resampling; default is action_horizon//2.",
    )
    flow_infer.add_argument(
        "--chunk-overlay-runway-steps",
        type=int,
        default=4,
        help=(
            "Extra future action rows to publish in the chunk overlay beyond --chunk-execute-steps. "
            "This feeds servo-side follower reserve_steps only; Python execution, gripper dispatch, "
            "and chunk anchoring still use --chunk-execute-steps."
        ),
    )
    flow_infer.add_argument(
        "--chunk-anchor-source",
        choices=["actual", "command", "chain"],
        default="actual",
        help=(
            "Pose the chunk deltas integrate from at each (re)anchor: 'actual' = measured FK(q_actual) "
            "(legacy), 'command' = FK(q_sent), the pose the server actually commanded — removes servo "
            "tracking-lag re-compensation from the integrated targets. 'chain' = pure plan-chain: each "
            "chunk integrates from the previous chunk's integrated tail (no robot-state re-anchor; "
            "boundary shortfall carries over instead of being discarded — pair with "
            "--tcp-target-pose-reanchor-mode last_emitted_continuous). Proprio/reset/tracking logs stay measured."
        ),
    )
    flow_infer.add_argument(
        "--chunk-stitch-mode",
        choices=["boundary", "ensemble"],
        default="boundary",
        help=(
            "How consecutive chunks are stitched. 'boundary' (default): swap whole chunks at the "
            "execute boundary (optionally RTC-inpainted). 'ensemble': observation-aligned recursive "
            "2-chunk blend — kick every --ensemble-period R steps; each executed R-window linearly "
            "blends the old chunk's [2R..3R) with the new chunk's [R..2R) (time-aligned); requires "
            "action_horizon >= 3R. Forces reanchor last_emitted_continuous + chain anchoring."
        ),
    )
    flow_infer.add_argument(
        "--ensemble-period",
        type=int,
        default=6,
        help="R: replan/blend window length in policy steps for --chunk-stitch-mode ensemble.",
    )
    flow_infer.add_argument(
        "--ensemble-blend",
        choices=["linear", "none"],
        default="linear",
        help=(
            "Window mixing for ensemble mode: 'linear' = lerp(old[2R..3R), new[R..2R)); "
            "'none' = execute the newest chunk's [R..2R) PURE (old plan is only the late runway; "
            "window seams may step by the plans' divergence, absorbed by FOH + follower jerk limits)."
        ),
    )
    flow_infer.add_argument(
        "--stream-prefetch-at",
        type=int,
        default=None,
        help=(
            "Kick the next chunk inference after this many consumed steps of the current window "
            "(default: legacy early kick at ~2). For RTC pair with --rtc-inference-delay == "
            "chunk_execute_steps - stream_prefetch_at (the frozen prefix that runs on the old plan "
            "while the new chunk is inpainted). Ignored in --sequential-chunk-inference."
        ),
    )
    flow_infer.add_argument(
        "--sequential-chunk-inference",
        action="store_true",
        help=(
            "Blocking sequential chunk mode (verification): consume the chunk to the END, only then "
            "run inference with the freshest observation (the robot holds still for the inference "
            "latency), and re-anchor the new chunk's deltas to the measured pose at activation. "
            "Disables the mid-chunk prefetch kick; pair with --chunk-execute-steps == action horizon."
        ),
    )
    flow_infer.add_argument(
        "--action-horizon",
        type=int,
        default=50,
        help=(
            "OpenPI remote model action horizon. New servers report this in websocket metadata; "
            "set explicitly for old servers or to validate h8/h24/h50 deploys."
        ),
    )
    flow_infer.add_argument(
        "--gripper-open-hold-steps",
        type=int,
        default=0,
        help="Hold the gripper fully OPEN for the first N executed policy steps so the arm can "
             "reach before the policy is allowed to close it (avoids premature grasp at start). "
             "0 = disabled (policy controls the gripper from step 0).",
    )
    flow_infer.add_argument(
        "--gripper-action-mode",
        choices=["absolute", "delta", "binary"],
        default="binary",
        help="How to interpret the checkpoint's action gripper dim. 'absolute' (DEFAULT, matches "
             "the latest openpi `--gripper-mode absolute` checkpoints): the action IS the next-step "
             "opening percent -> command it directly (no integration). 'binary' (for checkpoints "
             "trained with a binarized open/close gripper, e.g. openpi `binary 25`): the action is "
             "bimodal -> threshold it (--gripper-binary-threshold) and snap to the physical open/close "
             "presets (--gripper-open-percent / --gripper-close-percent). 'delta' (legacy): the action "
             "is a per-step opening change `(target-current)/100` -> integrate it onto the current "
             "opening. Use 'delta' only for older checkpoints trained with the relative gripper action.",
    )
    flow_infer.add_argument(
        "--gripper-open-percent",
        type=float,
        default=100.0,
        help="Opening percent commanded for the OPEN level in --gripper-action-mode binary "
             "(DEFAULT 75). Clamped to [0,100]. Also used as the hold value for "
             "--gripper-open-hold-steps in binary mode.",
    )
    flow_infer.add_argument(
        "--gripper-close-percent",
        type=float,
        default=3.0,
        help="Opening percent commanded for the CLOSE level in --gripper-action-mode binary "
             "(DEFAULT 3). Clamped to [0,100].",
    )
    flow_infer.add_argument(
        "--gripper-binary-threshold",
        type=float,
        default=40.0,
        help="Decision threshold (opening percent) for --gripper-action-mode binary: the model's "
             "gripper output >= threshold -> OPEN, else CLOSE (DEFAULT 40, the midpoint of the "
             "model's bimodal 0/100 output). Only used in binary mode.",
    )
    flow_infer.add_argument(
        "--gripper-close-bias",
        type=float,
        default=None,
        help="SHARED ABSOLUTE close-bias (opening percent subtracted from the target so grasps "
             "close more firmly; lower opening = more closed) applied to BOTH arms, unless a "
             "per-arm flag (--gripper-close-bias-left / --gripper-close-bias-right) overrides it. "
             "E.g. 1.0 turns an 18%% command into 17%%. Clamped to [0,100]; no effect in "
             "--gripper-action-mode delta/binary. UNSET by default -> the per-arm defaults apply "
             "(left 2.0, right 6.0).",
    )
    flow_infer.add_argument(
        "--gripper-close-bias-left",
        type=float,
        default=4.0,
        help="LEFT-arm ABSOLUTE close-bias override (opening percent). Wins over --gripper-close-bias. "
             "When unset, falls back to --gripper-close-bias, then to the left default (4.0). "
             "Clamped to [0,100]; no effect in delta/binary mode.",
    )
    flow_infer.add_argument(
        "--gripper-close-bias-right",
        type=float,
        default=4.0,
        help="RIGHT-arm ABSOLUTE close-bias override (opening percent). Wins over --gripper-close-bias. "
             "When unset, falls back to --gripper-close-bias, then to the right default (4.0). "
             "Clamped to [0,100]; no effect in delta/binary mode.",
    )
    flow_infer.add_argument(
        "--prompt",
        type=str,
        default=None,
        help="Override the language prompt sent to the openpi server (default OPENPI_DEFAULT_PROMPT). "
        "For the colorprompt model pass the phase_color prompt, e.g. 'pick up the black bolt with the "
        "right arm and put it in the green box' (right phase) or 'pick up the gray bolt with the left "
        "arm and put it in the gray box' (left phase).",
    )
    flow_infer.add_argument(
        "--gripper-close-snap-percent",
        type=float,
        default=15.0,
        help="ABSOLUTE close-snap deadzone (opening percent): after mapping, any gripper opening "
             "STRICTLY BELOW this snaps to 0 (fully closed), so small policy jitter near the closed "
             "end does not leave the jaw cracked open. E.g. 10 -> any commanded opening <10%% closes "
             "fully. Clamped to [0,100]; DEFAULT 15.0, and 0 turns the snap OFF. Turning it off is "
             "load-bearing, not cosmetic: the deployed pi05 checkpoints floor their gripper channel "
             "around 2-12%% opening and never command a full close on their own, so --gripper-close-"
             "snap-percent 0 leaves the jaw cracked open at every grasp and NO pick can succeed "
             "(measured 2026-07-30: 0/8 snap-off runs closed past 2.2%%, vs 5 successful picks with "
             "the default). Absolute mode only (no effect in delta; binary already snaps to "
             "--gripper-close-percent).",
    )
    flow_infer.add_argument(
        "--gripper-command-deadband-percent",
        type=float,
        default=0.0,
        help="Re-hold the last SENT gripper command until the target moves more than this "
             "(opening percent). The gripper channel is otherwise dispatched every policy "
             "step with no rate limit or hysteresis, so model jitter re-targets the jaw at "
             "30 Hz -- measured 2026-07-31 on the right arm at 4.8-9.1 command changes per "
             "SECOND, mean dwell 2.3-3.8 steps, 28-50%% of them reversing direction, which "
             "is audible as buzz. Offline on four recorded runs, 5%% cut the churn ~3x "
             "(9.1 -> 3.0 changes/s) with ZERO added lag, 1.4-1.7%% mean tracking error, and "
             "no change to any full-close event. The fully-closed (0%%) state is exempt in "
             "both directions so a grasp commitment / release is never suppressed. "
             "0 = off (DEFAULT). Absolute and binary modes both honour it.",
    )
    flow_infer.add_argument(
        "--chunk-knot-filter-hz",
        type=float,
        default=0.0,
        help="Zero-phase FIR low-pass cutoff (Hz) applied to the chunk's per-step POSE "
             "deltas before execution and before the overlay/follower feed. The gripper "
             "channel is NOT filtered. 0 = off (DEFAULT). "
             "Measured 2026-07-31 on the vibrating left arm: the policy's own 30 Hz action "
             "stream carries 26.5%% of its energy at 5-10 Hz, against 2.3%% for a human UMI "
             "teleop run doing the same pick-and-place on the same stand at higher speed; "
             "the stand's ~13/17 Hz modes turn that shoulder into the vibration. Offline "
             "(tools/follower_replay), 7 taps @5 Hz gave 5-10 Hz 27.4->18.5%%, 13-19 Hz "
             "4.3->1.4%%, path deviation p95 2.8 mm, descent travel preserved to 1%%. "
             "Zero phase because the chunk carries H=24 rows while ~5 execute, so the "
             "forward half of the kernel reads real future knots.",
    )
    flow_infer.add_argument(
        "--chunk-knot-filter-taps",
        type=int,
        default=7,
        help="FIR length for --chunk-knot-filter-hz (odd, >=3). Longer = sharper stopband "
             "but more of the chunk consumed as filter context. 7 is the measured "
             "cost/benefit knee; 11 @3 Hz reached 5-10 Hz 10.0%% at p95 4.8 mm deviation.",
    )
    flow_infer.add_argument(
        "--rtc",
        action="store_true",
        help="Enable Real-Time Chunking (RTC) for the openpi remote source: the server freezes the "
             "first --rtc-inference-delay actions of the next chunk and inpaints the rest toward the "
             "previous chunk, so async replan is smooth without the boundary crossfade (which is then "
             "disabled). Keeps long-horizon commitment AND reactivity. DEFAULT OFF (vanilla sampling). "
             "Requires an openpi server that returns 'rtc_raw_actions' (else it stays vanilla).",
    )
    flow_infer.add_argument(
        "--rtc-inference-delay",
        type=int,
        default=8,
        help="RTC delay d (policy steps guaranteed to execute during inference latency); the first d "
             "actions are hard-frozen to the previous chunk. ~ceil(inference_latency / policy_dt); "
             "clamped to [0, chunk_execute_steps]. Only used with --rtc.",
    )
    flow_infer.add_argument(
        "--rtc-schedule",
        choices=["exp", "linear", "zeros"],
        default="exp",
        help="RTC soft-mask schedule over the guided region: 'exp' (DEFAULT, the paper's convex ramp), "
             "'linear' (plain ramp), or 'zeros' (hard freeze only, no soft guidance). Only used with --rtc.",
    )
    flow_infer.add_argument(
        "--rtc-max-guidance-weight",
        type=float,
        default=5.0,
        help="RTC guidance-weight clip (beta); 5.0 per the paper. Only used with --rtc.",
    )
    flow_infer.add_argument(
        "--chunk-crossfade-steps",
        type=int,
        default=2,
        help=(
            "Blend the first N actions after each chunk-resample boundary from the "
            "previously emitted action (alpha 0->1) to remove the boundary jerk "
            "without steady-state lag. Default 2; set 0 to disable."
        ),
    )
    flow_infer.add_argument("--max-linear-step-m", type=float, default=0.020)
    flow_infer.add_argument("--max-angular-step-rad", type=float, default=0.03)
    flow_infer.add_argument(
        "--rotation-axes",
        default="xyz",
        help=(
            "Which per-arm rotation axes the policy may command: any subset of x,y,z "
            "(e.g. 'z' = yaw only, 'xy', 'xyz' = all, the default). Use 'none' or '' to "
            "drop all rotation (translation only). Disabled axes are zeroed so the arm "
            "holds that orientation component; translation (dxyz) and gripper actions "
            "are unchanged. Useful when a checkpoint/task has unreliable predicted "
            "rotation on some axes."
        ),
    )
    flow_infer.add_argument(
        "--translation-only",
        action="store_true",
        help="Shortcut for --rotation-axes none: zero all per-arm rotation action.",
    )
    flow_infer.add_argument(
        "--tcp-target-pose-conditioning",
        choices=["legacy_step_hold", "foh_se3"],
        default="foh_se3",
        help=(
            "A-stage conditioning for the streamed tcp_target_pose path. legacy_step_hold "
            "(default) holds each ~30 Hz step target between policy ticks (ZOH into the SMD). "
            "foh_se3 emits an SE(3)-interpolated absolute target every servo tick (smooth 500 Hz "
            "command). Only affects async-streamed rollout (real/sim)."
        ),
    )
    flow_infer.add_argument(
        "--tcp-target-pose-reanchor-mode",
        choices=["measured_legacy", "last_emitted_continuous", "measured_blend"],
        default="measured_blend",
        help=(
            "Chunk-boundary handling for foh_se3. measured_blend (default): anchor the new chunk to the measured "
            "pose for drift correction but blend in from the last emitted target (no one-tick jump). "
            "measured_legacy reanchors straight to measured. last_emitted_continuous gives perfect continuity "
            "without drift correction (tests/sim)."
        ),
    )
    flow_infer.add_argument("--tcp-target-pose-blend-steps", type=int, default=8)
    hdf5_audit = sub.add_parser(
        "hdf5-audit",
        help="Inspect UMI/Pika and robotics_lab HDF5 episodes before training.",
    )
    hdf5_audit.add_argument("--episodes-dir", required=True)
    hdf5_audit.add_argument("--dataset-manifest", default=None)
    hdf5_audit.add_argument("--single-arm-side", choices=("left", "right"), default=None)
    hdf5_audit.add_argument("--output-json", required=True)
    hdf5_audit.add_argument("--output-md", required=True)

    hdf5_view = sub.add_parser(
        "hdf5-view",
        help="View HDF5 episode images, poses, deltas, and actions in an OpenCV window.",
    )
    hdf5_view.add_argument("episode", help="HDF5 episode file")
    hdf5_view.add_argument("--single-arm-side", choices=("left", "right"), default="left")
    hdf5_view.add_argument("--camera-names", default=None, help="Comma-separated camera allow-list")
    hdf5_view.add_argument("--action-frame", choices=("ee_local",), default="ee_local")
    hdf5_view.add_argument("--start-frame", type=int, default=0)
    hdf5_view.add_argument("--fps", type=float, default=None)
    hdf5_view.add_argument("--image-size", type=int, default=320)
    hdf5_view.add_argument("--trail-length", type=int, default=120)

    umi_import = sub.add_parser(
        "umi-import",
        help="Import canonical UMI HDF5 episodes by linking raw files and writing manifest/report metadata.",
    )
    umi_import.add_argument("--input", required=True, help="Raw UMI session directory or HDF5 file")
    umi_import.add_argument("--output-dir", required=True, help="Output dataset directory")
    umi_import.add_argument("--task", required=True, help="Task name or description")
    umi_import.add_argument("--left-device", default=None, help="Expected left UMI device serial or name")
    umi_import.add_argument("--right-device", default=None, help="Expected right UMI device serial or name")
    umi_import.add_argument("--retarget-config", default=None, help="UMI retarget YAML/JSON config")
    umi_import.add_argument(
        "--require-measured-retarget",
        action="store_true",
        help="Fail unless the retarget config has status=measured or status=accepted",
    )
    umi_import.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace existing imported links/files under the output directory",
    )

    umi_convert = sub.add_parser(
        "umi-convert",
        help="Convert one UMI HDF5 episode to a FlowHdf5Dataset-compatible layout.",
    )
    umi_convert.add_argument("--input", required=True, help="Input UMI HDF5 episode")
    umi_convert.add_argument("--output", required=True, help="Output HDF5 episode")
    umi_convert.add_argument(
        "--format",
        choices=("robotics_lab_dual_arm", "pika_bimanual"),
        required=True,
        help="Conversion target layout",
    )
    umi_convert.add_argument("--retarget-config", default=None, help="UMI retarget YAML/JSON config")
    umi_convert.add_argument(
        "--require-measured-retarget",
        action="store_true",
        help="Fail unless the retarget config has status=measured or status=accepted",
    )
    umi_convert.add_argument(
        "--poses-only",
        action="store_true",
        help="Skip camera image datasets (slim output for TcpPoseTarget motion replay/profiling)",
    )

    args = parser.parse_args(argv)
    if args.command == "hdf5-record":
        from .recording import Hdf5EpisodeRecorder

        config = load_config(args.config)
        output_dir = args.output_dir if args.output_dir is not None else config.recording.output_dir
        rate_hz = args.rate if args.rate is not None else config.recording.rate_hz
        camera_client = None
        if args.with_camera or config.camera.enable:
            from .camera_bundle_client import CameraBundleClient

            camera_client = CameraBundleClient(
                zmq_endpoint=args.zmq_endpoint or config.camera.zmq_endpoint,
                topic=config.camera.bundle_topic,
                max_age_ms=config.camera.max_age_ms,
                include_depth=True,
            )
        recorder = Hdf5EpisodeRecorder(
            output_dir,
            recording_rate_hz=rate_hz,
            camera_client=camera_client,
            expected_cameras=config.camera.expected_cameras,
            record_zero_on_missing=config.camera.record_zero_on_missing,
        )
        state_client = RobotStateClient(config.robot_state.bind, config.robot_state.stale_timeout_sec)

        print("Waiting for robot state to anchor reset_pose...", flush=True)
        reset_snapshot = None
        deadline = time.monotonic() + max(config.runtime.startup_timeout_sec, 0.0)
        while time.monotonic() < deadline:
            reset_snapshot = state_client.poll_once(timeout_sec=0.2)
            if reset_snapshot is not None:
                break
        if reset_snapshot is None:
            print("ERROR: did not receive robot state within startup timeout", flush=True)
            state_client.close()
            if camera_client is not None:
                camera_client.close()
            return STARTUP_TIMEOUT_EXIT_CODE

        recorder.start_episode(
            reset_snapshot=reset_snapshot,
            task_description=args.task,
            action_source=config.action_source,
            operator_id=args.operator,
            dataset_metadata=config.recording.dataset_metadata,
        )
        print("Started episode; reset anchored. Press Ctrl-C to end.", flush=True)

        last_packet: dict[str, object] | None = None
        last_packet_time_ns = 0
        last_packet_seq = 0

        def packet_sink(packet: dict[str, object]) -> None:
            nonlocal last_packet, last_packet_time_ns, last_packet_seq
            last_packet = packet
            last_packet_time_ns = time.time_ns()
            last_packet_seq = int(packet.get("seq", 0) or 0)

        def state_sink(snapshot: StateSnapshot) -> None:
            recorder.record_frame(
                state_snapshot=snapshot,
                action_packet=last_packet,
                action_host_time_ns=last_packet_time_ns,
                action_seq=last_packet_seq,
            )

        command_client = ServoCommandClient(
            config.servo_command.endpoint,
            config.servo_command.timeout_sec,
            packet_sink=packet_sink,
        )
        try:
            rc = run(
                config,
                state_client=state_client,
                command_client=command_client,
                state_sink=state_sink,
                recording_supervisor=RecordingSupervisor(config),
            )
        finally:
            if recorder.has_active_episode:
                if "rc" not in locals():
                    end_reason = "operator_abort"
                    success = False
                elif rc == 0:
                    end_reason = "operator_abort"
                    success = False
                elif rc == STARTUP_TIMEOUT_EXIT_CODE:
                    end_reason = "timeout"
                    success = False
                elif rc == FAULT_LATCH_EXIT_CODE:
                    end_reason = "fault_latch"
                    success = False
                else:
                    end_reason = "other"
                    success = False
                path = recorder.end_episode(success=success, end_reason=end_reason)
                print(f"Episode written: {path}", flush=True)
            if camera_client is not None:
                camera_client.close()
        return rc
    if args.command == "hdf5-audit":
        from .hdf5_audit import audit_hdf5_episodes, write_hdf5_audit_outputs

        try:
            report = audit_hdf5_episodes(
                args.episodes_dir,
                dataset_manifest=args.dataset_manifest,
                single_arm_side=args.single_arm_side,
            )
            write_hdf5_audit_outputs(
                report,
                output_json=args.output_json,
                output_md=args.output_md,
            )
        except Exception as exc:
            print(f"policy_runner hdf5-audit failed: {exc}", file=sys.stderr)
            return 1
        print(f"wrote HDF5 audit JSON: {args.output_json}", flush=True)
        print(f"wrote HDF5 audit report: {args.output_md}", flush=True)
        return 0
    if args.command == "hdf5-view":
        from .hdf5_viewer import run_hdf5_viewer_cli

        return run_hdf5_viewer_cli(args)
    if args.command == "umi-import":
        from .umi_pipeline import run_umi_import_cli

        return run_umi_import_cli(args)
    if args.command == "umi-convert":
        from .umi_pipeline import run_umi_convert_cli

        return run_umi_convert_cli(args)
    if args.command == "flow-infer":
        from .flow_inference import resolve_flow_policy_dt_sec
        from .gripper import GripperCommand, GripperRuntime
        from .openpi_remote import OPENPI_CHECKPOINT_PREFIX, OpenpiRemoteActionSource

        config = load_config(args.config)
        rollout_policy = RolloutModePolicy.from_value(
            args.rollout_mode,
            send_dryrun_commands=args.send_dryrun_commands,
        )
        # 사내 학습 체크포인트(flow / direct_bc / pc_v1) 런타임은 제거되었다.
        # 유일한 정책 소스는 openpi:// 원격 서버이므로 로컬 경로는 fail-closed.
        if not str(args.checkpoint).startswith(OPENPI_CHECKPOINT_PREFIX):
            print(
                "policy_runner flow-infer requires an "
                f"{OPENPI_CHECKPOINT_PREFIX}HOST:PORT checkpoint; "
                f"got {args.checkpoint!r}",
                file=sys.stderr,
            )
            return 2
        # 원격 서버는 정규화 통계를 노출하지 않는다 -> policy dt 는 --policy-dt-sec
        # (또는 sim_dryrun 의 command_rate_hz) 에서만 나온다.
        dataset_stats = None

        training_episode_provider = None
        if args.training_episode_hdf5 is not None:
            if rollout_policy.mode != RolloutMode.CONTROLLER_SIM:
                print(
                    "policy_runner flow-infer training-episode replay is controller_sim only",
                    file=sys.stderr,
                )
                return 2
            replay_errors = []
            if bool(config.safety.allow_real_motion):
                replay_errors.append("safety.allow_real_motion must be false")
            if str(config.gripper.backend) != "none":
                replay_errors.append("gripper.backend must be none")
            if bool(config.safety.allow_real_gripper_motion):
                replay_errors.append("safety.allow_real_gripper_motion must be false")
            if not bool(args.include_depth) or bool(args.blank_depth):
                replay_errors.append("use --include-depth without --blank-depth")
            if str(args.proprio_mode) != "velocity":
                replay_errors.append("--proprio-mode must be velocity")
            if bool(getattr(args, "rtc", False)):
                replay_errors.append("--rtc must be disabled")
            if not bool(getattr(args, "sequential_chunk_inference", False)):
                replay_errors.append("--sequential-chunk-inference is required")
            if str(getattr(args, "chunk_stitch_mode", "boundary")) != "boundary":
                replay_errors.append("--chunk-stitch-mode must be boundary")
            if int(getattr(args, "chunk_overlay_runway_steps", -1)) != 0:
                replay_errors.append("--chunk-overlay-runway-steps must be 0")
            if int(getattr(args, "chunk_crossfade_steps", -1)) != 0:
                replay_errors.append("--chunk-crossfade-steps must be 0")
            if str(getattr(args, "chunk_anchor_source", "actual")) != "chain":
                replay_errors.append("--chunk-anchor-source must be chain")
            if str(args.tcp_target_pose_reanchor_mode) != "last_emitted_continuous":
                replay_errors.append(
                    "--tcp-target-pose-reanchor-mode must be last_emitted_continuous"
                )
            if bool(args.camera_preview):
                replay_errors.append("--camera-preview must be disabled")
            if _parse_rotation_axes(args) != (True, True, True):
                replay_errors.append("all rotation axes must remain enabled")
            if str(args.execute_arms) != "both":
                replay_errors.append("--execute-arms must be both")
            if replay_errors:
                print(
                    "policy_runner flow-infer training-episode replay rejected: "
                    + "; ".join(replay_errors),
                    file=sys.stderr,
                )
                return 2
            from .training_episode_replay import TrainingEpisodeReplay

            try:
                training_episode_provider = TrainingEpisodeReplay(
                    args.training_episode_hdf5,
                    retarget_config=args.training_episode_retarget_config,
                    output_dir=args.training_episode_output_dir,
                    training_video_dir=args.training_episode_video_dir,
                    training_parquet=args.training_episode_parquet,
                    depth_z_near_mm=float(args.depth_z_near_mm),
                    depth_z_far_mm=float(args.depth_z_far_mm),
                    depth_units_m=float(args.depth_units_m),
                    start_frame=int(args.training_episode_start_frame),
                )
            except ValueError as exc:
                print(f"policy_runner flow-infer training-episode replay rejected: {exc}", file=sys.stderr)
                return 2
            print(
                "[flow-infer] training-episode teacher forcing ON: saved RGB-D + recorded velocity "
                "proprio; live cameras disabled; controller output remains pgmode simulation only",
                flush=True,
            )

        camera_client = None
        if config.camera.enable and training_episode_provider is None:
            from .camera_bundle_client import CameraBundleClient

            camera_client = CameraBundleClient(
                zmq_endpoint=config.camera.zmq_endpoint,
                topic=config.camera.bundle_topic,
                max_age_ms=config.camera.max_age_ms,
                # z16 decode for RGB-D checkpoints; --blank-depth synthesizes the
                # depth channel, so it neither needs nor reads the live z16 stream.
                include_depth=bool(
                    getattr(args, "include_depth", False)
                    and not getattr(args, "blank_depth", False)
                ),
            )
        preview_process = None
        if args.camera_preview and config.camera.enable and training_episode_provider is None:
            import subprocess

            # Same ZMQ bundle/shm + resolve_frame mapping as the runtime; PUB
            # fans out so the preview never interferes with inference.
            preview_cmd = [
                sys.executable,
                "-m",
                "policy_runner.camera_preview",
                "--zmq-endpoint",
                config.camera.zmq_endpoint,
                "--topic",
                config.camera.bundle_topic,
                "--cameras",
                ",".join(config.camera.expected_cameras)
                or "left_realsense_color,right_realsense_color",
            ]
            # For an RGB-D rollout, decode + show the depth panels too, with the
            # SAME z_near/z_far/units the policy is fed — otherwise the preview's
            # bundle client drops depth and its panels read [MISSING].
            if getattr(args, "include_depth", False):
                preview_cmd += [
                    "--include-depth",
                    "--depth-z-near-mm",
                    str(args.depth_z_near_mm),
                    "--depth-z-far-mm",
                    str(args.depth_z_far_mm),
                    "--depth-units-m",
                    str(args.depth_units_m),
                ]
            preview_process = subprocess.Popen(preview_cmd)
        # Physical gripper hardware connects (and energizes motors) only when
        # the rollout mode could actually dispatch to it; sim_dryrun and
        # real_readonly always stay on the fail-closed Noop backend.
        gripper_backend = None
        if config.gripper.backend == "pika_serial":
            mode_value = rollout_policy.mode.value
            wants_gripper_hardware = (
                mode_value == "controller_sim"
                and config.gripper.actuate_in_controller_simulation
            ) or (mode_value == "real_policy" and config.safety.allow_real_gripper_motion)
            if wants_gripper_hardware:
                from .gripper import PikaSerialGripperBackend

                gripper_backend = PikaSerialGripperBackend(
                    ports={
                        "left": config.gripper.left_port,
                        "right": config.gripper.right_port,
                    },
                    sdk_path=config.gripper.pika_sdk_path or None,
                    min_rad=config.gripper.min_rad,
                    max_rad=config.gripper.max_rad,
                    deadband_rad=config.gripper.deadband_rad,
                    max_hz=config.gripper.max_hz,
                    suppress_sdk_logs=config.gripper.suppress_sdk_logs,
                    supports_controller_simulation=(
                        config.gripper.actuate_in_controller_simulation
                    ),
                    # Homing closes both grippers to their stop and re-zeros there.
                    # DEFAULT OFF: homing left the LEFT gripper stuck closed (it
                    # bottoms on its stop and fails to re-open; verified 2026-06-16
                    # left seed 0.35 vs open 70 with homing off). Set
                    # RB_GRIPPER_HOME_ON_CONNECT=1 to re-enable (e.g. once homing
                    # is fixed to release/verify the jaw after re-zeroing).
                    home_on_connect=(
                        os.environ.get("RB_GRIPPER_HOME_ON_CONNECT", "0")
                        not in ("0", "false", "False", "no", "")
                    ),
                ).connect()
        gripper_runtime = (
            GripperRuntime(
                rollout_mode=rollout_policy.mode.value,
                allow_real_gripper_motion=config.safety.allow_real_gripper_motion,
                backend=gripper_backend,
            )
            if gripper_backend is not None
            else GripperRuntime(
                rollout_mode=rollout_policy.mode.value,
                allow_real_gripper_motion=config.safety.allow_real_gripper_motion,
            )
        )
        # Optionally open both grippers fully at startup so every rollout begins
        # from a known open pose. DEFAULT OFF (gripper.startup_open): hold each
        # gripper at its current power-on position so the server start does not
        # force the asymmetric right-opens/left-closes move. Routed through the
        # same GripperRuntime gate as policy commands, so it honors real_policy +
        # allow_real_gripper_motion + RB_ALLOW_REAL_GRIPPER and is a logged noop
        # when the hardware lane is closed (percent units: 100 = open = max_rad).
        if gripper_backend is not None and config.gripper.startup_open:
            open_results = gripper_runtime.dispatch(
                [
                    GripperCommand(
                        arm=arm,
                        value=100.0,
                        command_type="target",
                        source="startup_open",
                    )
                    for arm in ("left", "right")
                    if arm in gripper_backend.ports
                ],
                # Open both grippers at once (parallel serial writes) so they
                # actuate simultaneously, not left-then-right.
                concurrent=True,
            )
            for result in open_results:
                print(
                    f"[flow-infer] startup gripper open {result.command.arm}: "
                    f"sent_to_physical={result.sent_to_physical} reason={result.reason}",
                    flush=True,
                )
            # Physical grippers actuate with a delay; let them finish opening
            # before the first policy step so inference does not begin while the
            # jaws are still moving. Only wait when a command actually reached
            # hardware (logged noop in sim / closed gripper lane -> no wait).
            if any(result.sent_to_physical for result in open_results):
                time.sleep(1.5)
                print("[flow-infer] startup gripper open settle: waited 1.5s", flush=True)
        elif gripper_backend is not None:
            # startup_open disabled: leave both grippers where they powered on (the
            # backend already seeded its targets from the live motor positions on
            # connect), so the server start does not move the jaws.
            print(
                "[flow-infer] startup gripper open disabled "
                "(gripper.startup_open=false): holding current gripper position",
                flush=True,
            )
        elif (
            config.gripper.startup_open
            and rollout_policy.mode.value == "real_policy"
            and bool(config.safety.allow_real_gripper_motion)
            and os.environ.get("RB_ALLOW_REAL_GRIPPER") == "1"
        ):
            # Gripper rides the COMMAND STREAM (gripper.backend != pika_serial -> no direct
            # backend here; a separate gripper_server owns the pika serial). The direct-backend
            # startup-open above is skipped, so open both grippers via a Hold + gripper_target=100
            # command instead: rb_servo_server forwards the gripper setpoint to the gripper_server
            # regardless of arm mode, so inference still begins from a known OPEN pose.
            from .robot_state_client import RobotStateClient, StateStreamLeaseReadback

            _osc = RobotStateClient(config.robot_state.bind, config.robot_state.stale_timeout_sec)
            _osc.start()
            _occ = ServoCommandClient(config.servo_command.endpoint, config.servo_command.timeout_sec)
            try:
                _ot0 = time.monotonic()
                while _osc.latest is None and time.monotonic() - _ot0 < 5.0:
                    time.sleep(0.05)
                if _osc.latest is None:
                    print("[flow-infer] startup gripper open (command-stream) SKIPPED: no robot state", flush=True)
                else:
                    # Open to the configured OPEN level (--gripper-open-percent, the
                    # same value the policy commands for "open" in binary mode) so the
                    # rollout begins from the policy's own open pose, not a hardcoded
                    # 100%.
                    _open_pct = max(0.0, min(100.0, float(getattr(args, "gripper_open_percent", 50.0))))
                    _occ.acquire_lease(StateStreamLeaseReadback(_osc), timeout_sec=4.0)
                    _occ.send(CommandIntent.arm_motion(timeout_sec=0.5))
                    time.sleep(0.2)
                    _open_intent = CommandIntent(
                        "Hold",
                        timeout_sec=1.0,
                        left={"mode": "Hold", "gripper_target": _open_pct},
                        right={"mode": "Hold", "gripper_target": _open_pct},
                    )
                    _odl = time.monotonic() + 1.5
                    while time.monotonic() < _odl:
                        _occ.send(_open_intent)
                        time.sleep(0.05)
                    _occ.release_lease()
                    print(
                        f"[flow-infer] startup gripper open via command stream (both -> {_open_pct:g}%)",
                        flush=True,
                    )
                    # Physical jaws keep moving after the last setpoint; settle before
                    # signalling the policy so the first inference sees a stable open
                    # pose, not jaws still in motion.
                    time.sleep(1.5)
                    print("[flow-infer] startup gripper open settle: waited 1.5s", flush=True)
            finally:
                _occ.close()
                _osc.close()
        try:
            base_policy_dt_sec = resolve_flow_policy_dt_sec(
                rollout_policy.mode,
                policy_dt_sec=args.policy_dt_sec,
                command_rate_hz=config.command_rate_hz,
                dataset_stats=dataset_stats,
            )
            speed_scale = float(getattr(args, "speed_scale", 1.0) or 1.0)
            if not (speed_scale > 0.0):
                raise ValueError("--speed-scale must be positive")
            policy_dt_sec = float(base_policy_dt_sec) / speed_scale
            max_linear_velocity_m_s = (
                None if args.max_linear_velocity_m_s is None else float(args.max_linear_velocity_m_s) * speed_scale
            )
            max_angular_velocity_rad_s = (
                None if args.max_angular_velocity_rad_s is None else float(args.max_angular_velocity_rad_s) * speed_scale
            )
            if abs(speed_scale - 1.0) > 1e-6:
                print(
                    f"[flow-infer] speed_scale={speed_scale:g}: policy_dt "
                    f"{float(base_policy_dt_sec):.4f}s -> {policy_dt_sec:.4f}s, "
                    f"linear clamp={max_linear_velocity_m_s}, angular clamp={max_angular_velocity_rad_s}",
                    flush=True,
                )
            source_kwargs = {
                "timeout_sec": config.servo_command.timeout_sec,
                "camera_client": camera_client,
                "policy_dt_sec": policy_dt_sec,
                "max_linear_velocity_m_s": max_linear_velocity_m_s,
                "max_angular_velocity_rad_s": max_angular_velocity_rad_s,
                "max_linear_step_m": args.max_linear_step_m,
                "max_angular_step_rad": args.max_angular_step_rad,
                "chunk_execute_steps": args.chunk_execute_steps,
                "chunk_overlay_runway_steps": args.chunk_overlay_runway_steps,
                "chunk_crossfade_steps": args.chunk_crossfade_steps,
                "allow_rbpodo_controller_simulation_cartesian": (
                    rollout_policy.allows_controller_simulation_cartesian
                ),
                "ee_local_r_align": args.ee_local_r_align,
                "gripper_runtime": gripper_runtime,
                "device": args.device,
            }
            # tcp_target_pose A-stage conditioning (Patch 3).
            tcp_tp_kwargs = {
                "tcp_target_pose_conditioning": args.tcp_target_pose_conditioning,
                "tcp_target_pose_reanchor_mode": args.tcp_target_pose_reanchor_mode,
                "tcp_target_pose_blend_steps": args.tcp_target_pose_blend_steps,
            }
            # Which physical camera feeds the checkpoint's left/right_wrist_0_rgb.
            # 'fisheye' (the fe65 deploy) reads the camera_server fisheye streams and
            # applies the training-time 0.65 center-crop; 'realsense' is the default.
            _wrist_camera_names = (
                ("left_fisheye_color", "right_fisheye_color")
                if config.camera.wrist_source == "fisheye"
                else ("left_realsense_color", "right_realsense_color")
            )
            source = OpenpiRemoteActionSource(
                args.checkpoint,
                **source_kwargs,
                **tcp_tp_kwargs,
                episode_observation_provider=training_episode_provider,
                camera_names=_wrist_camera_names,
                wrist_crop_frac=float(config.camera.wrist_crop_frac),
                action_horizon=args.action_horizon,
                proprio_mode=args.proprio_mode,
                velproprio_sample_mode=getattr(args, "velproprio_sample_mode", "replan"),
                velproprio_source=getattr(args, "velproprio_source", "measured"),
                **({"prompt": args.prompt} if getattr(args, "prompt", None) else {}),
                include_depth=bool(args.include_depth or args.blank_depth),
                blank_depth=bool(args.blank_depth),
                depth_z_near_mm=float(args.depth_z_near_mm),
                depth_z_far_mm=float(args.depth_z_far_mm),
                depth_units_m=float(args.depth_units_m),
                rtc_enabled=bool(getattr(args, "rtc", False)),
                rtc_inference_delay=int(getattr(args, "rtc_inference_delay", 2)),
                rtc_prefix_attention_schedule=str(getattr(args, "rtc_schedule", "exp")),
                rtc_max_guidance_weight=float(getattr(args, "rtc_max_guidance_weight", 5.0)),
            )
            source.runner_role = "flow_infer"
            source.name = "flow_infer"
            source.configure_rollout_step_log(args.rollout_step_log)
            if args.rollout_step_log:
                print(
                    f"[flow-infer] rollout step telemetry={args.rollout_step_log}",
                    flush=True,
                )
            source.configure_force_recovery(config.force_recovery)
            configure_camera_runtime = getattr(source, "configure_camera_runtime", None)
            if callable(configure_camera_runtime):
                configure_camera_runtime(config.camera)
            # Runtime execution mask: suppress the non-selected arm's per-step
            # commands so it holds in place. Only source.arm_mask (the emission
            # gate) is changed; source.checkpoint_arm_mask / selected_arms (the
            # gate identity) stay dual-arm, so _validate_real_policy is unaffected
            # and the suppression is strictly safer.
            if args.execute_arms != "both":
                import numpy as _np

                _mask = _np.asarray(
                    [1.0, 0.0] if args.execute_arms == "left" else [0.0, 1.0],
                    dtype=_np.float32,
                )
                source.arm_mask = _mask
                print(
                    f"[flow-infer] --execute-arms={args.execute_arms}: runtime "
                    f"arm_mask={_mask.tolist()} (other arm held; checkpoint stays "
                    f"dual-arm {list(source.checkpoint_arm_mask)})",
                    flush=True,
                )
            # Decouple chunk inference from the 500 Hz servo loop for live
            # streaming rollouts: background prefetch + per-step hold so the loop
            # never blocks on inference (removes the pulsed start/stop vibration).
            if rollout_policy.mode.value in {"controller_sim", "real_policy", "real_readonly"}:
                source.enable_async_chunking = True
                source.nonblocking_stream_inference = True
            # Sequential verification mode: no mid-chunk prefetch — infer only at
            # the boundary stall (fresh observation), robot pauses during latency.
            source.sequential_stream_inference = bool(
                getattr(args, "sequential_chunk_inference", False)
            )
            source.chunk_anchor_source = str(getattr(args, "chunk_anchor_source", "actual"))
            stitch_mode = str(getattr(args, "chunk_stitch_mode", "boundary"))
            if stitch_mode == "ensemble":
                if bool(getattr(args, "sequential_chunk_inference", False)):
                    raise RolloutModeValidationError(
                        "--chunk-stitch-mode ensemble and --sequential-chunk-inference are mutually exclusive"
                    )
                period = int(getattr(args, "ensemble_period", 6))
                # fail fast on H < 3R (the scheduler enforces the same rule)
                from .chunk_ensemble import ChunkEnsembleScheduler

                blend_mode = str(getattr(args, "ensemble_blend", "linear"))
                ChunkEnsembleScheduler(period, int(source.action_horizon), blend_mode=blend_mode)
                source.chunk_stitch_mode = "ensemble"
                source.ensemble_period = period
                source.ensemble_blend_mode = blend_mode
                # the mode presumes pure plan-chain integration on both paths
                source.chunk_anchor_source = "chain"
                if getattr(source, "_tcp_tp_reanchor_mode", None) != "last_emitted_continuous":
                    source._tcp_tp_reanchor_mode = "last_emitted_continuous"
                # RTC (if enabled) replans every R now, not every execute window
                source.rtc_replan_period = period
                _window_desc = (
                    "lerp(old[2R..3R), new[R..2R))" if blend_mode == "linear" else "new[R..2R) PURE (no blend)"
                )
                print(
                    f"[flow-infer] chunk-stitch-mode=ensemble R={period}: kick every {period} steps, "
                    f"window = {_window_desc}; H={int(source.action_horizon)} "
                    f"(runway {int(source.action_horizon) - 3 * period} steps)",
                    flush=True,
                )
            if getattr(args, "stream_prefetch_at", None) is not None:
                source.stream_prefetch_at = int(args.stream_prefetch_at)
                print(
                    f"[flow-infer] stream_prefetch_at={source.stream_prefetch_at}: next-chunk inference "
                    "kicks at this consumed-step index (RTC frozen prefix = execute_steps - this)",
                    flush=True,
                )
            if source.chunk_anchor_source != "actual":
                _anchor_desc = {
                    "command": "FK(q_sent) (command pose)",
                    "chain": "the previous chunk's integrated tail (pure plan-chain)",
                }.get(source.chunk_anchor_source, source.chunk_anchor_source)
                print(
                    f"[flow-infer] chunk-anchor-source={source.chunk_anchor_source}: chunk deltas "
                    f"integrate from {_anchor_desc}; proprio/reset stay measured",
                    flush=True,
                )
            if source.sequential_stream_inference:
                print(
                    "[flow-infer] sequential-chunk-inference ON: consume full chunk -> hold during "
                    "inference -> re-anchor next chunk to measured pose",
                    flush=True,
                )
            # Hold the gripper open for the first N policy steps (reach-before-grasp).
            source.gripper_open_hold_steps = int(getattr(args, "gripper_open_hold_steps", 0) or 0)
            # Interpret the action gripper dim as an absolute opening (default,
            # latest openpi), a binarized open/close target (binary checkpoints),
            # or a per-step delta to integrate (legacy checkpoints). 'binary' is a
            # flavour of the absolute (target-command) path.
            gripper_mode = str(getattr(args, "gripper_action_mode", "absolute"))
            source.gripper_action_absolute = gripper_mode in ("absolute", "binary")
            source.gripper_binary = gripper_mode == "binary"
            source.gripper_open_percent = float(getattr(args, "gripper_open_percent", 50.0))
            source.gripper_close_percent = float(getattr(args, "gripper_close_percent", 7.0))
            source.gripper_binary_threshold = float(
                getattr(args, "gripper_binary_threshold", 50.0)
            )
            # Close-bias: subtract a few percent from the absolute opening target so
            # a marginal grasp clamps (no effect in delta or binary mode). Resolved
            # PER ARM (the two pika grippers clamp differently): an explicit
            # --gripper-close-bias-<arm> wins; else the shared --gripper-close-bias;
            # else the per-arm default (left 2.0 / right 6.0). The shared base is
            # kept for the startup log + tests that set `gripper_close_bias` directly.
            _shared_bias = getattr(args, "gripper_close_bias", None)
            _left_bias = getattr(args, "gripper_close_bias_left", None)
            _right_bias = getattr(args, "gripper_close_bias_right", None)
            if _left_bias is None:
                _left_bias = (
                    _shared_bias
                    if _shared_bias is not None
                    else DEFAULT_GRIPPER_CLOSE_BIAS_LEFT
                )
            if _right_bias is None:
                _right_bias = (
                    _shared_bias
                    if _shared_bias is not None
                    else DEFAULT_GRIPPER_CLOSE_BIAS_RIGHT
                )
            source.gripper_close_bias = (
                float(_shared_bias) if _shared_bias is not None else 0.0
            )
            source.gripper_close_bias_left = float(min(100.0, max(0.0, _left_bias)))
            source.gripper_close_bias_right = float(min(100.0, max(0.0, _right_bias)))
            # Close-snap deadzone: collapse a near-closed absolute opening to fully
            # closed (0%) so small jitter doesn't leave the jaw cracked open.
            source.gripper_close_snap_percent = float(
                getattr(args, "gripper_close_snap_percent", 0.0) or 0.0
            )
            source.gripper_command_deadband_percent = float(
                getattr(args, "gripper_command_deadband_percent", 0.0) or 0.0
            )
            _knot_hz = float(getattr(args, "chunk_knot_filter_hz", 0.0) or 0.0)
            if _knot_hz > 0.0:
                from scipy.signal import firwin
                _taps = int(getattr(args, "chunk_knot_filter_taps", 7) or 7)
                if _taps < 3 or _taps % 2 == 0:
                    raise ValueError("--chunk-knot-filter-taps must be odd and >= 3")
                _knot_fs = 1.0 / float(policy_dt_sec)
                if not (_knot_hz < 0.5 * _knot_fs):
                    raise ValueError(
                        f"--chunk-knot-filter-hz {_knot_hz:g} must be below the policy "
                        f"Nyquist {0.5 * _knot_fs:.2f} Hz (policy_dt {policy_dt_sec:.4f}s)"
                    )
                source._chunk_knot_filter_taps = firwin(
                    _taps, _knot_hz, fs=_knot_fs, window="hamming"
                )
                print(
                    f"[flow-infer] chunk knot filter: {_taps}-tap zero-phase FIR @"
                    f"{_knot_hz:g}Hz (policy {_knot_fs:.1f}Hz) on pose deltas; gripper unfiltered",
                    flush=True,
                )
            # Per-axis rotation gate: keep only the selected rx/ry/rz axes of the
            # per-arm rotation action; disabled axes are zeroed so the arm holds
            # that orientation component (translation + gripper unchanged). Applies
            # to every source kind (flow / openpi / direct_bc).
            rotation_axes_enabled = _parse_rotation_axes(args)
            source.rotation_axes_enabled = rotation_axes_enabled
            if not all(rotation_axes_enabled):
                _kept = [
                    n for n, e in zip(("rx", "ry", "rz"), rotation_axes_enabled) if e
                ] or ["none"]
                print(
                    f"[flow-infer] rotation axes kept={','.join(_kept)} "
                    "(disabled axes zeroed; dxyz translation + gripper unchanged)",
                    flush=True,
                )
            if source.gripper_binary:
                detail = (
                    f", open={source.gripper_open_percent:g}% close={source.gripper_close_percent:g}%"
                    f" threshold={source.gripper_binary_threshold:g}%"
                )
            elif source.gripper_action_absolute:
                _lb = float(getattr(source, "gripper_close_bias_left", 0.0) or 0.0)
                _rb = float(getattr(source, "gripper_close_bias_right", 0.0) or 0.0)
                detail = f", close-bias L={_lb:g}%/R={_rb:g}%" if (_lb or _rb) else ""
                # Announce close-snap in BOTH states. It used to print only when
                # non-zero, so `--gripper-close-snap-percent 0` looked identical to
                # the default in the banner while silently making every grasp
                # impossible (the checkpoints never command a full close on their own).
                if source.gripper_close_snap_percent:
                    detail += f", close-snap<{source.gripper_close_snap_percent:g}%"
                else:
                    detail += ", close-snap OFF (policy must command full close itself)"
            else:
                detail = ""
            print(
                f"[flow-infer] gripper action mode = {gripper_mode}{detail}",
                flush=True,
            )
            geometry_status = _load_runtime_geometry_status(config)
            rollout_policy.validate_config(
                config,
                checkpoint_camera_names=source.camera_names,
                geometry_status=geometry_status,
                checkpoint_arm_mask=source.checkpoint_arm_mask,
                checkpoint_has_nonzero_gripper_commands=(
                    source.checkpoint_has_nonzero_gripper_commands
                ),
            )
            recorder = RolloutSummaryRecorder(
                rollout_policy,
                checkpoint_path=str(args.checkpoint),
                config_path=str(args.config),
                command_family=source.command_family,
                camera_names=list(source.camera_names),
                allow_real_gripper_motion=config.safety.allow_real_gripper_motion,
                selected_arms=list(source.checkpoint_selected_arms),
                left_arm_mask=source.checkpoint_arm_mask[0],
                right_arm_mask=source.checkpoint_arm_mask[1],
                collision_model_status=config.safety.collision_model_status,
            )
            run_source = source
            if not rollout_policy.may_send_commands:
                run_source = ReadOnlyActionSource(source, recorder)
                run_source.runner_role = getattr(source, "runner_role", "flow_infer")
                run_source.name = getattr(source, "name", "flow_infer")
            rc = run(
                config,
                source=run_source,
                send_commands=rollout_policy.may_send_commands,
                rollout_recorder=recorder,
                geometry_status=geometry_status,
            )
            write_rollout_summary(recorder, args.rollout_summary, source=run_source)
            print(f"wrote rollout_summary: {args.rollout_summary}", flush=True)
            return rc
        except (RolloutModeValidationError, ValueError) as exc:
            print(f"policy_runner flow-infer rollout-mode rejected: {exc}", file=sys.stderr)
            if camera_client is not None:
                camera_client.close()
            if training_episode_provider is not None:
                training_episode_provider.close()
            return 2
        except Exception:
            if camera_client is not None:
                camera_client.close()
            if training_episode_provider is not None:
                training_episode_provider.close()
            raise
        finally:
            # Disable+disconnect the serial grippers on every exit path.
            if gripper_backend is not None:
                gripper_backend.close()
            if preview_process is not None:
                preview_process.terminate()
                try:
                    preview_process.wait(timeout=2.0)
                except Exception:
                    preview_process.kill()
    raise ValueError(f"unknown policy_runner command: {args.command}")
