from __future__ import annotations

import argparse
import dataclasses
import os
import sys
import time
from typing import Callable, TextIO

from .action_sources import (
    DualSpaceMouseCartesianActionSource,
    HoldActionSource,
    JointSineActionSource,
    JointVelocityActionSource,
    MasterArmJointActionSource,
    SpaceMouseCartesianActionSource,
    SpaceMouseJointVelocityActionSource,
    TcpDeltaActionSource,
    TeleopMuxActionSource,
    UmiDualCartesianActionSource,
)
from .config import PolicyRunnerConfig, load_config
from .dataset_manifest import parse_camera_names
from .geometry import GeometryStatus, load_geometry_status
from .robot_state_client import RobotStateClient, StateSnapshot, StateStreamLeaseReadback
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
from .spacemouse import HidSpaceMouseReader, ScriptedSpaceMouseReader, SpaceMouseReader, SpaceMouseSample
from .action_sources.umi_dual_cartesian import MockUmiPoseReader, UdpUmiPoseReader, UmiPoseReader


STARTUP_TIMEOUT_EXIT_CODE = 2
LEASE_READBACK_TIMEOUT_EXIT_CODE = 3

# Quiet period (no motion intents) after which the teleop loop voluntarily
# releases the command-source lease so one-shot GUI commands (InitMotion / jog)
# can run between teleop bursts; the next motion intent re-acquires lazily.
IDLE_LEASE_RELEASE_SEC = 1.0


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
                "dual_spacemouse_cartesian, umi_dual_cartesian) — debug aid to "
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
) -> int:
    state_client = state_client or RobotStateClient(config.robot_state.bind, config.robot_state.stale_timeout_sec)
    if send_commands:
        command_client = command_client or ServoCommandClient(
            config.servo_command.endpoint,
            config.servo_command.timeout_sec,
        )
    source = source or make_action_source(config)
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
    try:
        while True:
            snapshot = state_client.latest
            now = monotonic_fn()
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
            if state_sink is not None:
                state_sink(snapshot)
            if rollout_recorder is not None:
                rollout_recorder.record_state(snapshot)
            intent = source.next_intent(snapshot, now)
            decision = safety_gate.evaluate(snapshot, intent, getattr(source, "requirements", None), now)
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
            # (no motion intents) so one-shot GUI commands (InitMotion / jog)
            # work BETWEEN teleop bursts instead of being rejected with
            # lease_conflict for up to lease_timeout_sec (60s). The lazy-acquire
            # block below re-acquires on the next motion intent. This also fixes
            # resume-after-expiry: once the server lease expired, the stale
            # lease_acquired flag made resumed teleop commands be dropped
            # (lease_required) with no re-acquire.
            if intent is not None and intent.is_motion:
                last_motion_intent_time = now
            elif (
                send_commands
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
                    sleep_fn(period)
                    continue
                assert command_client is not None
                # Lazy lease: acquire on the FIRST motion intent, not at startup.
                # An idle policy_runner must not camp on the lease (it would
                # block one-shot GUI commands like InitMotion). On conflict /
                # readback timeout, drop this tick and retry with backoff so a
                # temporary GUI lease holder does not kill the teleop process.
                if config.servo_command.acquire_lease and not lease_acquired and intent.is_motion:
                    if now < lease_retry_after:
                        sleep_fn(period)
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
                        lease_retry_after = now + 2.0
                        sleep_fn(period)
                        continue
                if intent.is_motion and not armed_for_motion:
                    arm = CommandIntent.arm_motion(timeout_sec=config.servo_command.timeout_sec)
                    arm_decision = safety_gate.evaluate(snapshot, arm, getattr(source, "requirements", None), now)
                    if rollout_recorder is not None:
                        rollout_recorder.record_decision(arm_decision)
                    if arm_decision.allowed:
                        command_client.send(arm)
                        if rollout_recorder is not None:
                            rollout_recorder.record_sent(arm)
                        armed_for_motion = True
                    else:
                        if rollout_recorder is not None:
                            rollout_recorder.record_dropped(arm_decision.reason, arm)
                        if teleop_debug:
                            debug_dropped += 1
                            debug_last_drop_reason = f"arm:{arm_decision.reason or ''}"
                        sleep_fn(period)
                        continue
                command_client.send(intent)
                if teleop_debug:
                    debug_sent += 1
                if rollout_recorder is not None:
                    rollout_recorder.record_sent(intent)
            elif intent is not None and rollout_recorder is not None:
                rollout_recorder.record_dropped(decision.reason, intent)
            sleep_fn(period)
    except KeyboardInterrupt:
        return 0
    finally:
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
    if config.action_source == "joint_velocity":
        return JointVelocityActionSource(
            velocity_deg_s=config.joint_velocity.velocity_deg_s,
            selected_arm=config.joint_velocity.selected_arm,
            timeout_sec=config.servo_command.timeout_sec,
            simulation_only=config.joint_velocity.simulation_only,
        )
    if config.action_source == "master_arm_joint":
        ma = config.master_arm_joint
        return MasterArmJointActionSource(
            config_path=ma.config_path,
            python_module_dir=ma.python_module_dir,
            module_name=ma.module_name,
            selected_arm=ma.selected_arm,
            mapping_mode=ma.mapping_mode,
            robot_init_strategy=ma.robot_init_strategy,
            deadman_side=ma.deadman_side,
            deadman_switch=ma.deadman_switch,
            require_deadman=ma.require_deadman,
            use_gravity_compensation=ma.use_gravity_compensation,
            hold_master_on_release=ma.hold_master_on_release,
            enable_gripper_readers=ma.enable_gripper_readers,
            left_scale=ma.left_scale,
            right_scale=ma.right_scale,
            left_sign=ma.left_sign,
            right_sign=ma.right_sign,
            left_offset_deg=ma.left_offset_deg,
            right_offset_deg=ma.right_offset_deg,
            left_robot_init_deg=ma.left_robot_init_deg,
            right_robot_init_deg=ma.right_robot_init_deg,
            left_joint_map=ma.left_joint_map,
            right_joint_map=ma.right_joint_map,
            max_joint_velocity_deg_s=ma.max_joint_velocity_deg_s,
            smoothing_alpha=ma.smoothing_alpha,
            wrap_delta=ma.wrap_delta,
            timeout_sec=config.servo_command.timeout_sec,
        )
    if config.action_source == "spacemouse_joint_velocity":
        return SpaceMouseJointVelocityActionSource(
            reader=_LazyHidSpaceMouseReader(),
            selected_arm=config.spacemouse.selected_arm,
            max_joint_velocity_deg_s=config.spacemouse.max_joint_velocity_deg_s,
            deadband=config.spacemouse.deadband,
            smoothing_alpha=config.spacemouse.smoothing_alpha,
            require_deadman=config.spacemouse.require_deadman,
            deadman_button=config.spacemouse.deadman_button,
            timeout_sec=config.servo_command.timeout_sec,
        )
    if config.action_source == "tcp_delta":
        return TcpDeltaActionSource(
            delta=config.tcp_delta.delta,
            selected_arm=config.tcp_delta.selected_arm,
            frame=config.tcp_delta.frame,
            max_linear_step_m=config.tcp_delta.max_linear_step_m,
            max_angular_step_rad=config.tcp_delta.max_angular_step_rad,
            timeout_sec=config.servo_command.timeout_sec,
            simulation_only=config.tcp_delta.simulation_only,
        )
    if config.action_source == "spacemouse_cartesian":
        return SpaceMouseCartesianActionSource(
            reader=_LazyHidSpaceMouseReader(),
            selected_arm=config.spacemouse_cartesian.selected_arm,
            frame=config.spacemouse_cartesian.frame,
            max_linear_velocity_m_s=config.spacemouse_cartesian.max_linear_velocity_m_s,
            max_angular_velocity_rad_s=config.spacemouse_cartesian.max_angular_velocity_rad_s,
            deadband=config.spacemouse_cartesian.deadband,
            response_curve_gamma=config.spacemouse_cartesian.response_curve_gamma,
            sample_hold_timeout_sec=config.spacemouse_cartesian.sample_hold_timeout_sec,
            require_deadman=config.spacemouse_cartesian.require_deadman,
            deadman_button=config.spacemouse_cartesian.deadman_button,
            timeout_sec=config.servo_command.timeout_sec,
            allow_rbpodo_controller_simulation=(
                config.safety.allow_rbpodo_controller_simulation_cartesian
            ),
        )
    if config.action_source == "dual_spacemouse_cartesian":
        return _make_dual_spacemouse_cartesian_source(config)
    if config.action_source == "umi_dual_cartesian":
        return _make_umi_dual_cartesian_source(config)
    if config.action_source == "teleop_mux":
        return TeleopMuxActionSource(
            _make_dual_spacemouse_cartesian_source(config),
            _make_umi_dual_cartesian_source(config),
            tie_break=config.teleop_mux.tie_break,
        )
    raise ValueError(f"unknown action_source: {config.action_source}")


def _make_dual_spacemouse_cartesian_source(
    config: PolicyRunnerConfig,
) -> DualSpaceMouseCartesianActionSource:
    left = config.spacemouse_cartesian_dual.left
    right = config.spacemouse_cartesian_dual.right
    return DualSpaceMouseCartesianActionSource(
        left_reader=_spacemouse_reader_from_device_config(left),
        right_reader=_spacemouse_reader_from_device_config(right),
        frame=config.spacemouse_cartesian_dual.frame,
        max_linear_velocity_m_s=config.spacemouse_cartesian_dual.max_linear_velocity_m_s,
        max_angular_velocity_rad_s=config.spacemouse_cartesian_dual.max_angular_velocity_rad_s,
        deadband=config.spacemouse_cartesian_dual.deadband,
        response_curve_gamma=config.spacemouse_cartesian_dual.response_curve_gamma,
        linear_axis_signs=config.spacemouse_cartesian_dual.linear_axis_signs,
        angular_axis_signs=config.spacemouse_cartesian_dual.angular_axis_signs,
        angular_axis_order=config.spacemouse_cartesian_dual.angular_axis_order,
        sample_hold_timeout_sec=config.spacemouse_cartesian_dual.sample_hold_timeout_sec,
        require_deadman=config.spacemouse_cartesian_dual.require_deadman,
        activation_deadband=config.spacemouse_cartesian_dual.activation_deadband,
        startup_requires_neutral=config.spacemouse_cartesian_dual.startup_requires_neutral,
        startup_neutral_hold_sec=config.spacemouse_cartesian_dual.startup_neutral_hold_sec,
        left_deadman_button=left.deadman_button,
        right_deadman_button=right.deadman_button,
        timeout_sec=config.servo_command.timeout_sec,
        allow_rbpodo_controller_simulation=(
            config.safety.allow_rbpodo_controller_simulation_cartesian
        ),
    )


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
        delta_frame=umi.delta_frame,
        gripper_offset=umi.gripper_offset,
        r_align=umi.r_align,
        workspace_bounds=umi.workspace_bounds,
        sample_hold_timeout_sec=umi.sample_hold_timeout_sec,
        timeout_sec=config.servo_command.timeout_sec,
        deadman_release_grace_sec=umi.deadman_release_grace_sec,
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

    record = sub.add_parser("record", help="Record rb_servo_server state stream without sending commands.")
    record.add_argument("--state-bind", default="udp://0.0.0.0:50120")
    record.add_argument("--output-dir", default="/data/policy_episodes")
    record.add_argument("--duration-sec", type=float, default=0.0)
    record.add_argument("--stale-timeout-sec", type=float, default=0.5)

    teleop = sub.add_parser("teleop-record", help="Run a policy action source and record state/action JSONL.")
    teleop.add_argument("--config", required=True, help="policy_runner YAML config")
    teleop.add_argument("--output-dir", default="/data/policy_episodes")

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

    train = sub.add_parser("train", help="Train a small V1 behavior-cloning baseline from JSONL episodes.")
    train.add_argument("--episodes-dir", default="/data/policy_episodes")
    train.add_argument("--checkpoint", default="/data/checkpoints/bc_state_to_twist.pt")
    train.add_argument("--epochs", type=int, default=50)
    train.add_argument("--batch-size", type=int, default=64)
    train.add_argument("--lr", type=float, default=1e-3)
    train.add_argument(
        "--strict-config-check",
        action="store_true",
        help="Abort training if HDF5 episodes carry different config hashes.",
    )

    ml_preflight = sub.add_parser(
        "ml-preflight",
        help="Report ML dependency status and verify the requested vision backbone.",
    )
    ml_preflight.add_argument(
        "--vision-backbone",
        choices=(
            "tiny_cnn",
            "resnet18",
            "resnet50",
            "dinov3",
            "dinov3_convnext_tiny",
            "dinov3_convnext_small",
            "dinov3_convnext_base",
            "dinov3_convnext_large",
        ),
        default="tiny_cnn",
    )
    ml_preflight.add_argument(
        "--require-cuda",
        action="store_true",
        help="Fail if PyTorch cannot see CUDA.",
    )
    ml_preflight.add_argument(
        "--expect-cuda-device-count",
        type=int,
        default=None,
        help="Fail unless PyTorch sees exactly this many CUDA devices.",
    )

    flow_train = sub.add_parser(
        "flow-train",
        help="Train an offline image-conditioned flow-matching action-chunk baseline from HDF5 episodes.",
    )
    flow_train.add_argument("--episodes-dir", default=None)
    flow_train.add_argument("--dataset-manifest", default=None)
    flow_train.add_argument("--camera-names", default=None, help="Comma-separated camera allow-list")
    flow_train.add_argument("--exclude-camera-names", default=None, help="Comma-separated camera deny-list")
    flow_train.add_argument("--single-arm-side", choices=("left", "right"), default=None)
    flow_train.add_argument("--action-frame", choices=("ee_local",), default="ee_local")
    flow_train.add_argument("--max-episodes", type=int, default=None)
    flow_train.add_argument("--checkpoint", default="outputs/flow_policy.pt")
    flow_train.add_argument("--vision-backbone", default="resnet50")
    flow_train.add_argument("--action-horizon", type=int, default=16)
    flow_train.add_argument("--batch-size", type=int, default=32)
    flow_train.add_argument("--epochs", type=int, default=100)
    flow_train.add_argument("--lr", type=float, default=1e-4)
    flow_train.add_argument("--image-size", type=int, default=224)
    flow_train.add_argument("--hidden-dim", type=int, default=128)
    flow_train.add_argument("--condition-encoder", choices=("transformer", "mlp"), default="transformer")
    flow_train.add_argument(
        "--train-vision",
        action="store_true",
        help="Unfreeze the vision encoder; the default keeps it frozen.",
    )
    flow_train.add_argument("--val-split", type=float, default=0.1)
    flow_train.add_argument("--sample-steps", type=int, default=16)
    flow_train.add_argument("--device", default="auto")
    flow_train.add_argument("--max-stats-samples", type=int, default=None)
    flow_train.add_argument("--write-eval-report", default=None)

    imitation_snapshot = sub.add_parser(
        "imitation-snapshot",
        help="Materialize an immutable HDF5 dataset snapshot for official imitation experiments.",
    )
    imitation_snapshot.add_argument("--data-dir", required=True)
    imitation_snapshot.add_argument("--output-dir", required=True)
    imitation_snapshot.add_argument("--action-horizon", type=int, default=16)

    imitation_split = sub.add_parser(
        "imitation-split",
        help="Create a deterministic episode-level train/validation split from an imitation snapshot.",
    )
    imitation_split.add_argument("--snapshot", required=True)
    imitation_split.add_argument("--output-dir", required=True)
    imitation_split.add_argument("--val-ratio", type=float, default=0.2)
    imitation_split.add_argument("--seed", type=int, default=0)

    imitation_experiment = sub.add_parser(
        "imitation-experiment",
        help="Run leakage-safe imitation-learning baselines and action-chunk models from HDF5 episodes.",
    )
    imitation_experiment.add_argument("--data-dir", required=True)
    imitation_experiment.add_argument("--output-dir", required=True)
    imitation_experiment.add_argument("--snapshot", default=None)
    imitation_experiment.add_argument("--split", default=None)
    imitation_experiment.add_argument(
        "--models",
        default="zero,train_mean,state_mlp,direct_bc_tiny,direct_bc_resnet18,flow_tiny,flow_resnet18,act_tiny",
        help="Comma-separated model list.",
    )
    imitation_experiment.add_argument("--camera-names", default=None, help="Comma-separated camera allow-list")
    imitation_experiment.add_argument("--exclude-camera-names", default=None, help="Comma-separated camera deny-list")
    imitation_experiment.add_argument("--action-frame", choices=("ee_local",), default="ee_local")
    imitation_experiment.add_argument("--action-horizon", type=int, default=16)
    imitation_experiment.add_argument("--image-size", type=int, default=128)
    imitation_experiment.add_argument(
        "--image-crop",
        choices=("none", "center_square"),
        default="none",
        help="Deterministic image crop before resize; center_square is fixed and not validation-tuned.",
    )
    imitation_experiment.add_argument("--batch-size", type=int, default=64)
    imitation_experiment.add_argument("--epochs", type=int, default=20)
    imitation_experiment.add_argument("--lr", type=float, default=1e-4)
    imitation_experiment.add_argument("--hidden-dim", type=int, default=256)
    imitation_experiment.add_argument("--device", default="auto")
    imitation_experiment.add_argument("--seed", type=int, default=0)
    imitation_experiment.add_argument("--max-train-samples", type=int, default=None)
    imitation_experiment.add_argument("--max-val-samples", type=int, default=None)
    imitation_experiment.add_argument("--flow-sample-steps", type=int, default=8)
    imitation_experiment.add_argument("--diffusion-sample-steps", type=int, default=16)
    imitation_experiment.add_argument(
        "--eval-samples",
        type=int,
        default=8,
        help="best-of-N samples per eval item (minADE-style, multimodal-robust); 1 disables best-of-N",
    )
    imitation_experiment.add_argument(
        "--split-mode",
        choices=("primary", "session_holdout"),
        default="primary",
        help=(
            "primary uses the fixed episode train/validation split; session_holdout "
            "trains on non-held sessions and validates on session_holdout_val."
        ),
    )

    imitation_aggregate = sub.add_parser(
        "imitation-aggregate",
        help="Aggregate one or more imitation leaderboard summaries into a combined report.",
    )
    imitation_aggregate.add_argument(
        "--input",
        action="append",
        required=True,
        help="Input directory or leaderboard_summary.json. May be supplied multiple times.",
    )
    imitation_aggregate.add_argument("--output-dir", required=True)
    imitation_aggregate.add_argument("--label", default="combined")

    infer = sub.add_parser("infer", help="Run a trained behavior-cloning checkpoint in simulation.")
    infer.add_argument("--config", required=True, help="policy_runner YAML config")
    infer.add_argument("--checkpoint", default="/data/checkpoints/bc_state_to_twist.pt")
    infer.add_argument(
        "--ignore-config-drift",
        action="store_true",
        help="Suppress one-shot checkpoint/runtime config hash drift warnings.",
    )

    flow_infer = sub.add_parser(
        "flow-infer",
        help="Run a trained flow-matching action-chunk checkpoint in simulation.",
    )
    flow_infer.add_argument("--checkpoint", default="outputs/flow_policy.pt")
    flow_infer.add_argument("--config", required=True, help="policy_runner YAML config")
    flow_infer.add_argument(
        "--rollout-mode",
        required=True,
        choices=RolloutMode.choices(),
        help=(
            "Where inferred flow-policy actions may go: offline_eval, sim_dryrun, "
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
        "--episodes-dir",
        default=None,
        help="HDF5 episode file or directory for rollout-mode offline_eval.",
    )
    flow_infer.add_argument(
        "--max-offline-samples",
        type=int,
        default=1,
        help="Maximum HDF5 samples to evaluate in rollout-mode offline_eval.",
    )
    flow_infer.add_argument("--sample-steps", type=int, default=16)
    flow_infer.add_argument(
        "--deterministic-sampling",
        action="store_true",
        help=(
            "Integrate the flow ODE from zero init (deterministic mean path) instead of "
            "random noise. Default is stochastic sampling (x_T ~ N(0,I)), correct for "
            "multimodal action distributions."
        ),
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
        "--image-size",
        type=int,
        default=None,
        help=(
            "Image size for direct-BC imitation checkpoints or ensembles that do not store image_size; "
            "must match training."
        ),
    )
    flow_infer.add_argument(
        "--ensemble-name",
        default="top5",
        help="Named ensemble from an imitation ensemble report JSON; default is top5.",
    )
    flow_infer.add_argument(
        "--command-family",
        choices=("tcp_twist_local", "tcp_target_pose"),
        default=None,
        help=(
            "Flow action command family for ee_local body-frame deltas. Defaults to "
            "tcp_twist_local; tcp_target_pose composes each delta into an absolute "
            "TcpPoseTarget."
        ),
    )
    flow_infer.add_argument(
        "--allow-tcp-twist-local",
        action="store_true",
        help=(
            "Allow the TcpTwistLocal flow command family for controller_sim/real_policy "
            "(ee_local checkpoints; default command family)."
        ),
    )
    flow_infer.add_argument(
        "--allow-tcp-target-pose",
        action="store_true",
        help=(
            "Allow the TcpPoseTarget flow command family for controller_sim/real_policy "
            "(ee_local deltas composed into absolute tcp_target_stand setpoints)."
        ),
    )
    flow_infer.add_argument(
        "--ee-local-r-align",
        default=None,
        help=(
            "Fixed rotation between the training EE body frame and the RB TCP frame for "
            "ee_local checkpoints. Preset name or 9 row-major floats. Presets: "
            "'pika_rz180' (measured pika-UMI correction, 180deg about approach(z) on BOTH "
            "translation and rotation; pika UMI always uses this), 'pika_rz180_trans_only' "
            "(ablation: flip x/y translation only, leave rotation unchanged). "
            "Default: none (frames assumed identical)."
        ),
    )
    flow_infer.add_argument(
        "--camera-preview",
        action="store_true",
        help=(
            "Open a live OpenCV window showing the camera frames the policy consumes "
            "(spawns policy_runner.camera_preview alongside the rollout)."
        ),
    )
    flow_infer.add_argument(
        "--policy-dt-sec",
        type=float,
        default=None,
        help=(
            "Seconds represented by one flow action step. Controller/real flow rollout "
            "requires this value or checkpoint dataset_stats.dt_mean_sec; sim_dryrun "
            "can fall back to 1/command_rate_hz."
        ),
    )
    flow_infer.add_argument(
        "--max-linear-velocity-m-s",
        type=float,
        default=None,
        help="Override flow twist linear clamp; omitted uses checkpoint action statistics.",
    )
    flow_infer.add_argument(
        "--max-angular-velocity-rad-s",
        type=float,
        default=None,
        help="Override flow twist angular clamp; omitted uses checkpoint action statistics.",
    )
    flow_infer.add_argument(
        "--chunk-execute-steps",
        type=int,
        default=None,
        help="Number of sampled action steps to execute before resampling; default is action_horizon//2.",
    )
    flow_infer.add_argument(
        "--chunk-crossfade-steps",
        type=int,
        default=2,
        help=(
            "Blend the first N twists after each chunk-resample boundary from the "
            "previously emitted twist (alpha 0->1) to remove the boundary jerk "
            "without steady-state lag. 0 (default) disables crossfade. Try 2-3."
        ),
    )
    flow_infer.add_argument("--max-linear-step-m", type=float, default=0.002)
    flow_infer.add_argument("--max-angular-step-rad", type=float, default=0.01)

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
        help="Skip camera image datasets (slim output for TcpTargetPose motion replay/profiling)",
    )

    args = parser.parse_args(argv)
    if args.command == "record":
        from .recording import record_state_stream

        path = record_state_stream(
            bind=args.state_bind,
            output_dir=args.output_dir,
            duration_sec=args.duration_sec,
            stale_timeout_sec=args.stale_timeout_sec,
        )
        print(f"policy_runner recorded state episode: {path}", flush=True)
        return 0
    if args.command == "teleop-record":
        from .recording import EpisodeRecorder

        config = load_config(args.config)
        recorder = EpisodeRecorder(
            args.output_dir,
            metadata={
                "recording_mode": "teleop_record",
                "policy_config": args.config,
                "action_source": config.action_source,
                "dataset_metadata": config.recording.dataset_metadata,
            },
        )
        state_client = RobotStateClient(config.robot_state.bind, config.robot_state.stale_timeout_sec)
        command_client = ServoCommandClient(
            config.servo_command.endpoint,
            config.servo_command.timeout_sec,
            packet_sink=recorder.record_action,
        )
        try:
            return run(
                config,
                state_client=state_client,
                command_client=command_client,
                state_sink=recorder.record_state,
            )
        finally:
            recorder.close()
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
                else:
                    end_reason = "other"
                    success = False
                path = recorder.end_episode(success=success, end_reason=end_reason)
                print(f"Episode written: {path}", flush=True)
            if camera_client is not None:
                camera_client.close()
        return rc
    if args.command == "train":
        from .training import train_behavior_cloning

        train_behavior_cloning(
            episodes_dir=args.episodes_dir,
            checkpoint_path=args.checkpoint,
            epochs=args.epochs,
            batch_size=args.batch_size,
            lr=args.lr,
            strict_config_check=args.strict_config_check,
        )
        return 0
    if args.command == "ml-preflight":
        from .ml_preflight import run_ml_preflight

        return run_ml_preflight(
            vision_backbone=args.vision_backbone,
            require_cuda=args.require_cuda,
            expect_cuda_device_count=args.expect_cuda_device_count,
        )
    if args.command == "flow-train":
        from .ml_preflight import check_ml_preflight, render_ml_preflight

        preflight = check_ml_preflight(vision_backbone=args.vision_backbone)
        if not bool(preflight["requested_backbone"]["ok"]):
            sys.stderr.write(render_ml_preflight(preflight))
            sys.stderr.flush()
            return 1

        from .flow_training import train_flow_matching

        train_flow_matching(
            episodes_dir=args.episodes_dir,
            checkpoint_path=args.checkpoint,
            vision_backbone=args.vision_backbone,
            action_horizon=args.action_horizon,
            batch_size=args.batch_size,
            epochs=args.epochs,
            lr=args.lr,
            image_size=args.image_size,
            hidden_dim=args.hidden_dim,
            condition_encoder=args.condition_encoder,
            frozen_vision=not args.train_vision,
            val_split=args.val_split,
            sample_steps=args.sample_steps,
            device=args.device,
            max_stats_samples=args.max_stats_samples,
            dataset_manifest=args.dataset_manifest,
            camera_names=parse_camera_names(args.camera_names),
            exclude_camera_names=parse_camera_names(args.exclude_camera_names),
            single_arm_side=args.single_arm_side,
            max_episodes=args.max_episodes,
            write_eval_report=args.write_eval_report,
            action_frame=args.action_frame,
        )
        return 0
    if args.command == "imitation-snapshot":
        from .imitation_experiments import create_dataset_snapshot

        result = create_dataset_snapshot(
            data_dir=args.data_dir,
            output_dir=args.output_dir,
            action_horizon=args.action_horizon,
        )
        print(
            "imitation snapshot written: "
            f"{result.snapshot_path} hash={result.snapshot_hash} "
            f"valid={result.valid_episode_count} rejected={result.rejected_episode_count}",
            flush=True,
        )
        return 0
    if args.command == "imitation-split":
        from .imitation_experiments import create_split_manifest

        result = create_split_manifest(
            snapshot_path=args.snapshot,
            output_dir=args.output_dir,
            val_ratio=args.val_ratio,
            seed=args.seed,
        )
        print(
            "imitation split written: "
            f"{result.split_path} hash={result.split_hash} "
            f"train={result.train_count} val={result.val_count} "
            f"session_holdout_val={result.session_holdout_val_count}",
            flush=True,
        )
        return 0
    if args.command == "imitation-experiment":
        from .imitation_experiments import run_imitation_experiment

        report = run_imitation_experiment(
            data_dir=args.data_dir,
            output_dir=args.output_dir,
            snapshot_path=args.snapshot,
            split_path=args.split,
            models=[name.strip() for name in args.models.split(",") if name.strip()],
            action_horizon=args.action_horizon,
            image_size=args.image_size,
            image_crop=args.image_crop,
            camera_names=parse_camera_names(args.camera_names),
            exclude_camera_names=parse_camera_names(args.exclude_camera_names),
            action_frame=args.action_frame,
            batch_size=args.batch_size,
            epochs=args.epochs,
            lr=args.lr,
            hidden_dim=args.hidden_dim,
            device=args.device,
            seed=args.seed,
            max_train_samples=args.max_train_samples,
            max_val_samples=args.max_val_samples,
            flow_sample_steps=args.flow_sample_steps,
            diffusion_sample_steps=args.diffusion_sample_steps,
            eval_samples=args.eval_samples,
            split_mode=args.split_mode,
        )
        print(f"imitation leaderboard written: {report['output_dir']}/leaderboard_report.md", flush=True)
        return 0
    if args.command == "imitation-aggregate":
        from .imitation_experiments import aggregate_imitation_reports

        result = aggregate_imitation_reports(
            input_dirs=args.input,
            output_dir=args.output_dir,
            label=args.label,
        )
        print(
            "imitation aggregate written: "
            f"{result.report_path} rows={result.row_count} failed={result.failed_count}",
            flush=True,
        )
        return 0
    if args.command == "infer":
        from .training import BehaviorCloningActionSource

        config = load_config(args.config)
        return run(
            config,
            source=BehaviorCloningActionSource(
                args.checkpoint,
                config.servo_command.timeout_sec,
                ignore_config_drift=args.ignore_config_drift,
            ),
        )
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
        from .flow_inference import (
            DirectBcCheckpointEnsembleActionSource,
            DirectBcImageActionSource,
            FlowMatchingActionSource,
            action_chunk_checkpoint_kind,
            canonical_flow_command_family,
            load_action_chunk_checkpoint_dataset_stats,
            resolve_flow_command_family,
            resolve_flow_policy_dt_sec,
            run_direct_bc_ensemble_offline_eval,
            run_direct_bc_offline_eval,
            run_flow_offline_eval,
            validate_flow_command_family,
        )
        from .gripper import GripperCommand, GripperRuntime
        from .openpi_remote import OPENPI_CHECKPOINT_PREFIX, OpenpiRemoteActionSource

        config = load_config(args.config)
        rollout_policy = RolloutModePolicy.from_value(
            args.rollout_mode,
            send_dryrun_commands=args.send_dryrun_commands,
        )
        try:
            if str(args.checkpoint).startswith(OPENPI_CHECKPOINT_PREFIX):
                checkpoint_kind = "openpi_remote"
                dataset_stats = None
            else:
                checkpoint_kind = action_chunk_checkpoint_kind(args.checkpoint, device="cpu")
                dataset_stats = load_action_chunk_checkpoint_dataset_stats(
                    args.checkpoint,
                    device="cpu",
                    ensemble_name=args.ensemble_name,
                )
            command_family = resolve_flow_command_family(
                rollout_policy.mode,
                args.command_family,
                dataset_stats=dataset_stats,
            )
            validate_flow_command_family(
                rollout_policy.mode,
                command_family,
                allow_tcp_twist_local=args.allow_tcp_twist_local,
                allow_tcp_target_pose=args.allow_tcp_target_pose,
                dataset_stats=dataset_stats,
            )
        except (RolloutModeValidationError, ValueError) as exc:
            print(f"policy_runner flow-infer rollout-mode rejected: {exc}", file=sys.stderr)
            return 2
        if rollout_policy.mode == RolloutMode.OFFLINE_EVAL:
            if checkpoint_kind == "openpi_remote":
                print(
                    "policy_runner flow-infer offline_eval is not supported for openpi:// remote policies; "
                    "use sim_dryrun (optionally OPENPI_REMOTE_FAKE_IMAGES=1) instead",
                    file=sys.stderr,
                )
                return 2
            if args.episodes_dir is None:
                print(
                    "policy_runner flow-infer offline_eval requires --episodes-dir with one or more HDF5 samples",
                    file=sys.stderr,
                )
                return 2
            try:
                rollout_policy.validate_config(config)
                if checkpoint_kind == "direct_bc":
                    offline_result = run_direct_bc_offline_eval(
                        checkpoint_path=args.checkpoint,
                        episodes_dir=args.episodes_dir,
                        device=args.device,
                        max_samples=args.max_offline_samples,
                        command_family=canonical_flow_command_family(command_family),
                        image_size=args.image_size,
                    )
                elif checkpoint_kind == "direct_bc_ensemble":
                    offline_result = run_direct_bc_ensemble_offline_eval(
                        report_path=args.checkpoint,
                        episodes_dir=args.episodes_dir,
                        device=args.device,
                        max_samples=args.max_offline_samples,
                        command_family=canonical_flow_command_family(command_family),
                        image_size=args.image_size,
                        ensemble_name=args.ensemble_name,
                    )
                else:
                    offline_result = run_flow_offline_eval(
                        checkpoint_path=args.checkpoint,
                        episodes_dir=args.episodes_dir,
                        sample_steps=args.sample_steps,
                        device=args.device,
                        max_samples=args.max_offline_samples,
                        command_family=canonical_flow_command_family(command_family),
                    )
            except (RolloutModeValidationError, ValueError) as exc:
                print(f"policy_runner flow-infer rollout-mode rejected: {exc}", file=sys.stderr)
                return 2
            recorder = RolloutSummaryRecorder(
                rollout_policy,
                checkpoint_path=str(args.checkpoint),
                config_path=str(args.config),
                command_family=offline_result.command_family,
                camera_names=offline_result.camera_names,
                selected_arms=list(offline_result.selected_arms),
                left_arm_mask=offline_result.checkpoint_arm_mask[0]
                if offline_result.checkpoint_arm_mask is not None
                else None,
                right_arm_mask=offline_result.checkpoint_arm_mask[1]
                if offline_result.checkpoint_arm_mask is not None
                and len(offline_result.checkpoint_arm_mask) > 1
                else None,
                allow_real_gripper_motion=config.safety.allow_real_gripper_motion,
                collision_model_status=config.safety.collision_model_status,
            )
            recorder.image_decode_count = offline_result.image_decode_count
            recorder.missing_camera_count = offline_result.missing_camera_count
            recorder.proposed_intent_count = offline_result.action_chunk_count
            write_rollout_summary(recorder, args.rollout_summary)
            print(f"wrote rollout_summary: {args.rollout_summary}", flush=True)
            return 0

        camera_client = None
        if config.camera.enable:
            from .camera_bundle_client import CameraBundleClient

            camera_client = CameraBundleClient(
                zmq_endpoint=config.camera.zmq_endpoint,
                topic=config.camera.bundle_topic,
                max_age_ms=config.camera.max_age_ms,
            )
        preview_process = None
        if args.camera_preview and config.camera.enable:
            import subprocess

            # Same ZMQ bundle/shm + resolve_frame mapping as the runtime; PUB
            # fans out so the preview never interferes with inference.
            preview_process = subprocess.Popen(
                [
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
                ],
            )
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
        # Open both grippers fully at startup so every rollout begins from a
        # known open pose. Routed through the same GripperRuntime gate as policy
        # commands, so it honors real_policy + allow_real_gripper_motion +
        # RB_ALLOW_REAL_GRIPPER and is a logged noop when the hardware lane is
        # closed (percent units: 100 = open = max_rad).
        if gripper_backend is not None:
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
        try:
            policy_dt_sec = resolve_flow_policy_dt_sec(
                rollout_policy.mode,
                command_family,
                policy_dt_sec=args.policy_dt_sec,
                command_rate_hz=config.command_rate_hz,
                dataset_stats=dataset_stats,
            )
            source_kwargs = {
                "timeout_sec": config.servo_command.timeout_sec,
                "camera_client": camera_client,
                "command_family": command_family,
                "policy_dt_sec": policy_dt_sec,
                "max_linear_velocity_m_s": args.max_linear_velocity_m_s,
                "max_angular_velocity_rad_s": args.max_angular_velocity_rad_s,
                "max_linear_step_m": args.max_linear_step_m,
                "max_angular_step_rad": args.max_angular_step_rad,
                "chunk_execute_steps": args.chunk_execute_steps,
                "chunk_crossfade_steps": args.chunk_crossfade_steps,
                "allow_rbpodo_controller_simulation_cartesian": (
                    rollout_policy.allows_controller_simulation_cartesian
                ),
                "ee_local_r_align": args.ee_local_r_align,
                "gripper_runtime": gripper_runtime,
                "device": args.device,
            }
            if checkpoint_kind == "openpi_remote":
                source = OpenpiRemoteActionSource(
                    args.checkpoint,
                    **source_kwargs,
                )
            elif checkpoint_kind == "direct_bc":
                source = DirectBcImageActionSource(
                    args.checkpoint,
                    image_size=args.image_size,
                    **source_kwargs,
                )
            elif checkpoint_kind == "direct_bc_ensemble":
                source = DirectBcCheckpointEnsembleActionSource(
                    args.checkpoint,
                    ensemble_name=args.ensemble_name,
                    image_size=args.image_size,
                    **source_kwargs,
                )
            else:
                source = FlowMatchingActionSource(
                    args.checkpoint,
                    sample_steps=args.sample_steps,
                    stochastic_sampling=not args.deterministic_sampling,
                    **source_kwargs,
                )
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
            return 2
        except Exception:
            if camera_client is not None:
                camera_client.close()
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
