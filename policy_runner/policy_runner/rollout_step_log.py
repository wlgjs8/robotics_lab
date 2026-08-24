"""Best-effort per-policy-step JSONL telemetry for flow-infer rollouts.

The command loop only builds a small record and performs a non-blocking queue
put. Directory creation, JSON encoding, and file writes run on a daemon writer
thread. Any logging failure disables this telemetry surface without affecting
the rollout.
"""
from __future__ import annotations

import json
import math
import queue
import threading
import time
from pathlib import Path
from typing import Any, Callable, Mapping


ROLLOUT_STEP_LOG_SCHEMA = "robotics_lab.policy_runner.rollout_step.v1"
_ARMS = ("left", "right")
_STOP = object()


class _AsyncJsonlWriter:
    def __init__(self, path: str | Path, *, queue_capacity: int = 2048) -> None:
        if queue_capacity <= 0:
            raise ValueError("queue_capacity must be positive")
        self.path = Path(path).expanduser()
        self._queue: queue.Queue[object] = queue.Queue(maxsize=int(queue_capacity))
        self._accepting = True
        self._failed = threading.Event()
        self._closed = threading.Event()
        self._disabled_reason: str | None = None
        self._thread = threading.Thread(
            target=self._run,
            name="rollout-step-jsonl",
            daemon=True,
        )
        self._thread.start()

    @property
    def enabled(self) -> bool:
        return self._accepting and not self._failed.is_set()

    @property
    def disabled_reason(self) -> str | None:
        return self._disabled_reason

    def disable(self, reason: str) -> None:
        self._accepting = False
        self._disabled_reason = str(reason)
        self._failed.set()
        try:
            self._queue.put_nowait(_STOP)
        except queue.Full:
            pass

    def submit(self, record: Mapping[str, Any]) -> bool:
        if not self.enabled:
            return False
        try:
            self._queue.put_nowait(dict(record))
            return True
        except queue.Full:
            self.disable("queue_full")
        except Exception as exc:  # noqa: BLE001 - telemetry must never affect control.
            self.disable(f"queue_error:{type(exc).__name__}")
        return False

    def close(self) -> None:
        if self._closed.is_set():
            return
        self._accepting = False
        if self._thread.is_alive() and not self._failed.is_set():
            try:
                self._queue.put(_STOP, timeout=0.5)
            except Exception as exc:  # noqa: BLE001 - best-effort shutdown only.
                self.disable(f"close_queue_error:{type(exc).__name__}")
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8", buffering=64 * 1024) as handle:
                while not self._failed.is_set():
                    item = self._queue.get()
                    if item is _STOP:
                        break
                    handle.write(
                        json.dumps(
                            item,
                            allow_nan=False,
                            separators=(",", ":"),
                        )
                        + "\n"
                    )
                handle.flush()
        except Exception as exc:  # noqa: BLE001 - telemetry must never affect control.
            self.disable(f"writer_error:{type(exc).__name__}")
        finally:
            self._closed.set()


class RolloutStepLogger:
    """Non-blocking JSONL logger for one record per committed policy step."""

    def __init__(
        self,
        path: str | Path,
        *,
        writer_factory: Callable[[str | Path], Any] = _AsyncJsonlWriter,
    ) -> None:
        self.path = str(path)
        self._enabled = True
        self._disabled_reason: str | None = None
        self._writer: Any | None = None
        try:
            self._writer = writer_factory(path)
        except Exception as exc:  # noqa: BLE001 - startup logging failure is non-fatal.
            self._disable(f"writer_start_error:{type(exc).__name__}")

    @property
    def enabled(self) -> bool:
        if not self._enabled or self._writer is None:
            return False
        return bool(getattr(self._writer, "enabled", True))

    @property
    def disabled_reason(self) -> str | None:
        if self._disabled_reason is not None:
            return self._disabled_reason
        return getattr(self._writer, "disabled_reason", None)

    def log_step(
        self,
        *,
        state_payload: Mapping[str, Any],
        command_intent: Any | None,
        conditioned_targets: Mapping[str, Any] | None,
        raw_delta_ee_local: Mapping[str, Any] | None,
        gripper_cmd_pct: Mapping[str, Any] | None,
        chunk_id: int | None,
        chunk_step_index: int | None,
        stall: bool = False,
        hold: bool = False,
        inference_latency_ms: float | None = None,
        rtc: Mapping[str, Any] | None = None,
        gripper_proprio: Mapping[str, Any] | None = None,
        t_mono: float | None = None,
        t_wall: float | None = None,
    ) -> bool:
        if not self.enabled:
            return False
        try:
            record = build_rollout_step_record(
                state_payload=state_payload,
                command_intent=command_intent,
                conditioned_targets=conditioned_targets,
                raw_delta_ee_local=raw_delta_ee_local,
                gripper_cmd_pct=gripper_cmd_pct,
                chunk_id=chunk_id,
                chunk_step_index=chunk_step_index,
                stall=stall,
                hold=hold,
                inference_latency_ms=inference_latency_ms,
                rtc=rtc,
                gripper_proprio=gripper_proprio,
                t_mono=time.monotonic() if t_mono is None else t_mono,
                t_wall=time.time() if t_wall is None else t_wall,
            )
            if not bool(self._writer.submit(record)):
                self._disable("writer_disabled")
                return False
            return True
        except Exception as exc:  # noqa: BLE001 - hot-path telemetry is fail-safe.
            self._disable(f"hot_path_error:{type(exc).__name__}")
            return False

    def close(self) -> None:
        writer = self._writer
        self._enabled = False
        if writer is None:
            return
        try:
            writer.close()
        except Exception:  # noqa: BLE001 - shutdown telemetry is best effort.
            pass

    def _disable(self, reason: str) -> None:
        self._enabled = False
        self._disabled_reason = str(reason)
        disable = getattr(self._writer, "disable", None)
        if callable(disable):
            try:
                disable(reason)
            except Exception:  # noqa: BLE001 - already handling telemetry failure.
                pass


def build_rollout_step_record(
    *,
    state_payload: Mapping[str, Any],
    command_intent: Any | None,
    conditioned_targets: Mapping[str, Any] | None,
    raw_delta_ee_local: Mapping[str, Any] | None,
    gripper_cmd_pct: Mapping[str, Any] | None,
    chunk_id: int | None,
    chunk_step_index: int | None,
    stall: bool,
    hold: bool,
    inference_latency_ms: float | None,
    t_mono: float,
    t_wall: float,
    rtc: Mapping[str, Any] | None = None,
    gripper_proprio: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    mono = _required_finite_float(t_mono, "t_mono")
    wall = _required_finite_float(t_wall, "t_wall")
    targets = conditioned_targets if isinstance(conditioned_targets, Mapping) else {}
    inferred_chunk_ids = {
        int(value.chunk_id)
        for value in targets.values()
        if value is not None and getattr(value, "chunk_id", None) is not None
    }
    resolved_chunk_id = (
        int(chunk_id)
        if chunk_id is not None
        else (next(iter(inferred_chunk_ids)) if len(inferred_chunk_ids) == 1 else None)
    )
    target_values = [value for value in targets.values() if value is not None]
    resolved_stall = bool(stall) or any(
        bool(getattr(value, "stall", False))
        for value in target_values
    )
    resolved_hold = (
        bool(hold)
        or any(bool(getattr(value, "hold", False)) for value in target_values)
        or _intent_is_hold(command_intent)
    )

    record: dict[str, Any] = {
        "schema": ROLLOUT_STEP_LOG_SCHEMA,
        "t_mono": mono,
        "t_wall": wall,
        "chunk_id": resolved_chunk_id,
        "chunk_step_index": None if chunk_step_index is None else int(chunk_step_index),
        "stall": resolved_stall,
        "hold": resolved_hold,
        "arms": {},
    }
    latency = _finite_float(inference_latency_ms)
    if latency is not None:
        record["inference_latency_ms"] = latency
    rtc_record = _rtc_record(rtc)
    if rtc_record is not None:
        record["rtc"] = rtc_record

    payload = state_payload if isinstance(state_payload, Mapping) else {}
    for arm in _ARMS:
        conditioned = targets.get(arm)
        cmd_pose = _finite_vector(getattr(conditioned, "pose", None), 7)
        if cmd_pose is None:
            cmd_pose = _command_pose_from_intent(command_intent, arm)
        meas_pose = _measured_pose(payload, arm)
        force_control = _arm_mapping(payload, arm).get("force_control")
        force_torque = _arm_mapping(payload, arm).get("force_torque")
        force_control = force_control if isinstance(force_control, Mapping) else {}
        force_torque = force_torque if isinstance(force_torque, Mapping) else {}
        raw_delta = (
            raw_delta_ee_local.get(arm)
            if isinstance(raw_delta_ee_local, Mapping)
            else None
        )
        gripper_cmd = (
            gripper_cmd_pct.get(arm)
            if isinstance(gripper_cmd_pct, Mapping)
            else None
        )
        if gripper_cmd is None:
            gripper_cmd = _gripper_command_from_intent(command_intent, arm)
        arm_record = {
            "cmd_pose": cmd_pose,
            "meas_pose": meas_pose,
            "cmd_minus_meas_z_mm": (
                None
                if cmd_pose is None or meas_pose is None
                else (cmd_pose[2] - meas_pose[2]) * 1000.0
            ),
            "raw_delta_ee_local": _finite_vector(raw_delta, 6),
            "gripper_cmd_pct": _finite_float(gripper_cmd),
            "gripper_meas_pct": _measured_gripper_pct(payload, arm),
            # TRANSPORT age only: publish->receive of the gripper_state.v1 message
            # `gripper_meas_pct` came from. It reads ~0.05 ms in every run, which
            # invites the conclusion that the gripper feedback is instant -- it is
            # not. This number does NOT include the age of the pika telemetry
            # SAMPLE inside that message, which is the dominant term
            # (`gripper_sample_age_ms` below).
            "gripper_feedback_age_ms": _gripper_feedback_age_ms(payload, arm),
            # SENSOR age: pika serial sample -> gripper_server publish, stamped in
            # the gripper server from the SDK's own reader-thread callback. The jaw
            # telemetry arrives at ~18.5 Hz against a 30 Hz policy grid, so the
            # measured percent the policy sees is typically 27 ms and at worst
            # ~54 ms old. null when the publisher is too old to stamp it -- never a
            # fabricated 0, which would repeat the mistake above.
            "gripper_sample_age_ms": _gripper_sample_age_ms(payload, arm),
            # What the model ACTUALLY received on its proprio gripper channel, and
            # which signal produced it (--gripper-proprio-source: actual / command /
            # hybrid_free / hybrid_jam / actual_no_command). Without this the A/B is
            # unreadable after the fact: `gripper_meas_pct` and `gripper_cmd_pct`
            # are both logged, but which one reached the policy was not.
            "gripper_proprio_pct": _finite_float(
                (gripper_proprio or {}).get(arm, {}).get("pct")
                if isinstance((gripper_proprio or {}).get(arm), Mapping)
                else None
            ),
            "gripper_proprio_source": (
                (gripper_proprio or {}).get(arm, {}).get("source")
                if isinstance((gripper_proprio or {}).get(arm), Mapping)
                else None
            ),
            # Force-control arming state. The FT software zero only runs after an
            # Init Motion (auto_tare_after_init_motion), so a rollout started
            # without one sits in awaiting_init_tare for its whole duration with
            # the contact reflex NEVER armed -- and with safety.floor_constraint
            # disabled that leaves no floor backstop at all. Recording it per step
            # makes an unprotected run visible in the log instead of inferable.
            "force_control_state": (
                str(force_control.get("state"))
                if force_control.get("state") is not None
                else None
            ),
            "compliance_offset_surface": _finite_vector(
                force_control.get("compliance_offset_surface"),
                6,
            ),
            "correction_m": _finite_float(force_control.get("correction_m")),
            "wrench_tcp_fz": _vector_component(force_torque.get("wrench_tcp"), 2),
            "control_external_wrench_fz": _vector_component(
                force_torque.get("control_external_wrench"),
                2,
            ),
        }
        record["arms"][arm] = arm_record
    return record


def _rtc_record(rtc: Any) -> dict[str, Any] | None:
    """Per-chunk RTC delay accounting: what we told the server vs what happened.

    ``configured_delay`` is the static ``--rtc-inference-delay`` the client sends as
    ``inference_delay``; the server hard-freezes exactly that many leading rows of the
    new chunk. ``realized_delay`` is ``source_start_index`` -- the policy steps actually
    emitted between the observation this chunk was inferred from and its activation,
    i.e. the rows the warm-row alignment drops. They must match: the frozen prefix is
    exactly the dropped prefix only when ``realized == configured``. ``delay_error``
    < 0 means we over-froze (frozen rows bleed into the executed window -> the robot
    replays stale plan); > 0 means we under-froze (executed rows had no continuity
    guarantee -> chunk-boundary jump).
    """
    if not isinstance(rtc, Mapping):
        return None
    configured = _finite_int(rtc.get("configured_delay"))
    realized = _finite_int(rtc.get("realized_delay"))
    if configured is None and realized is None:
        return None
    out: dict[str, Any] = {
        "configured_delay": configured,
        "realized_delay": realized,
        "delay_error": (
            None if configured is None or realized is None else realized - configured
        ),
    }
    for key in ("execute_horizon", "schedule", "alignment_outcome"):
        value = rtc.get(key)
        if value is None:
            continue
        out[key] = value if isinstance(value, str) else _finite_int(value)
    return out


def _finite_int(value: Any) -> int | None:
    resolved = _finite_float(value)
    return None if resolved is None else int(resolved)


def _arm_mapping(payload: Mapping[str, Any], arm: str) -> Mapping[str, Any]:
    value = payload.get(arm)
    return value if isinstance(value, Mapping) else {}


def _measured_pose(payload: Mapping[str, Any], arm: str) -> list[float] | None:
    arm_payload = _arm_mapping(payload, arm)
    gate = arm_payload.get("cartesian_gate")
    gate = gate if isinstance(gate, Mapping) else {}
    controller_sim = arm_payload.get("controller_simulation_mode")
    controller_sim = controller_sim if isinstance(controller_sim, Mapping) else {}
    operation_mode = str(
        arm_payload.get("operation_mode", gate.get("operation_mode", ""))
    ).strip().lower()
    physical_motion_expected = arm_payload.get(
        "physical_motion_expected",
        gate.get("physical_motion_expected"),
    )
    configured_source = str(
        gate.get("controller_simulation_servo_state_source", "")
    ).strip().lower()
    recommended = str(controller_sim.get("recommended_tracking_pose", "")).strip()
    reference_required = (
        operation_mode in {"simulation", "sim"}
        and physical_motion_expected is False
        and configured_source == "reference"
    ) or recommended == "tcp_ref_stand"
    if reference_required:
        if arm_payload.get("tcp_ref_valid") is False:
            return None
        pose = arm_payload.get("tcp_ref_stand")
    else:
        pose = arm_payload.get("tcp_stand") or arm_payload.get("tcp_actual_stand")
    if not isinstance(pose, Mapping):
        return None
    quat = pose.get("quaternion_xyzw")
    if not isinstance(quat, (list, tuple)) or len(quat) < 4:
        quat = [pose.get(name) for name in ("qx", "qy", "qz", "qw")]
    return _finite_vector(
        [pose.get("x"), pose.get("y"), pose.get("z"), *quat[:4]],
        7,
    )


def _command_pose_from_intent(intent: Any | None, arm: str) -> list[float] | None:
    arm_payload = getattr(intent, arm, None)
    if not isinstance(arm_payload, Mapping):
        return None
    if str(arm_payload.get("mode", "")) != "TcpPoseTarget":
        return None
    raw = arm_payload.get("tcp_target_stand")
    if isinstance(raw, Mapping):
        quat = raw.get("quaternion_xyzw")
        if not isinstance(quat, (list, tuple)) or len(quat) < 4:
            return None
        raw = [raw.get("x"), raw.get("y"), raw.get("z"), *quat[:4]]
    return _finite_vector(raw, 7)


def _intent_is_hold(intent: Any | None) -> bool:
    if intent is None:
        return False
    if str(getattr(intent, "mode", "")) == "Hold":
        return True
    modes = []
    for arm in _ARMS:
        arm_payload = getattr(intent, arm, None)
        if isinstance(arm_payload, Mapping):
            modes.append(str(arm_payload.get("mode", "")))
    return bool(modes) and all(mode == "Hold" for mode in modes)


def _gripper_command_from_intent(intent: Any | None, arm: str) -> float | None:
    arm_payload = getattr(intent, arm, None)
    if not isinstance(arm_payload, Mapping):
        return None
    return _finite_float(arm_payload.get("gripper_target"))


def _measured_gripper_pct(payload: Mapping[str, Any], arm: str) -> float | None:
    arm_payload = _arm_mapping(payload, arm)
    for container_name in ("gripper", "gripper_state"):
        container = arm_payload.get(container_name)
        if not isinstance(container, Mapping):
            continue
        if container.get("valid") is False or container.get("stale") is True:
            continue
        for key in ("percent", "gripper_position", "position"):
            value = _finite_float(container.get(key))
            if value is not None:
                return value
    return None


def _gripper_feedback_age_ms(payload: Mapping[str, Any], arm: str) -> float | None:
    """Age of the gripper feedback message backing this step's measured percent.

    Mirrors _measured_gripper_pct's container search so the age always describes
    the same block the percent was read from. None when the bridge could not stamp
    it (no usable host_time_ns) -- never a fabricated 0, which would read as
    "feedback is instant" and send the latency hunt in the wrong direction.
    """
    arm_payload = _arm_mapping(payload, arm)
    for container_name in ("gripper", "gripper_state"):
        container = arm_payload.get(container_name)
        if not isinstance(container, Mapping):
            continue
        if container.get("valid") is False or container.get("stale") is True:
            continue
        value = _finite_float(container.get("feedback_age_ms"))
        if value is not None:
            return value
    return None


def _gripper_sample_age_ms(payload: Mapping[str, Any], arm: str) -> float | None:
    """Age of the pika telemetry SAMPLE backing this step's measured percent.

    Distinct from _gripper_feedback_age_ms, which only covers publish->receive of
    the message. This is sample->publish: how stale the jaw reading already was
    when the gripper server serialised it. The two add up to the real age of
    `gripper_meas_pct`; logging only the transport half is what made the feedback
    look instant (~0.05 ms) while the sample itself was 27-54 ms old.

    None when the publisher did not stamp it (older gripper_server / bridge).
    """
    arm_payload = _arm_mapping(payload, arm)
    for container_name in ("gripper", "gripper_state"):
        container = arm_payload.get(container_name)
        if not isinstance(container, Mapping):
            continue
        if container.get("valid") is False or container.get("stale") is True:
            continue
        value = _finite_float(container.get("sample_age_ms"))
        if value is not None:
            return value
    return None


def _vector_component(value: Any, index: int) -> float | None:
    vector = _finite_vector(value, index + 1, allow_longer=True)
    return None if vector is None else vector[index]


def _finite_vector(
    value: Any,
    length: int,
    *,
    allow_longer: bool = False,
) -> list[float] | None:
    if value is None or isinstance(value, (str, bytes, Mapping)):
        return None
    try:
        items = list(value)
    except (TypeError, ValueError):
        return None
    if len(items) < length or (not allow_longer and len(items) != length):
        return None
    out = [_finite_float(item) for item in items[:length]]
    if any(item is None for item in out):
        return None
    return [float(item) for item in out]


def _required_finite_float(value: Any, name: str) -> float:
    resolved = _finite_float(value)
    if resolved is None:
        raise ValueError(f"{name} must be finite")
    return resolved


def _finite_float(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        resolved = float(value)
    except (TypeError, ValueError):
        return None
    return resolved if math.isfinite(resolved) else None
