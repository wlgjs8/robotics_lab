from __future__ import annotations

import argparse
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
)
from .config import PolicyRunnerConfig, load_config
from .dataset_manifest import parse_camera_names
from .geometry import GeometryStatus, load_geometry_status
from .robot_state_client import RobotStateClient, StateSnapshot, StateStreamLeaseReadback
from .safety import SafetyGate
from .servo_command_client import CommandIntent, ServoCommandClient
from .spacemouse import HidSpaceMouseReader, SpaceMouseReader, SpaceMouseSample


STARTUP_TIMEOUT_EXIT_CODE = 2
LEASE_READBACK_TIMEOUT_EXIT_CODE = 3


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0].startswith("-"):
        parser = argparse.ArgumentParser(description="robotics_lab policy_runner")
        parser.add_argument("--config", required=True, help="policy_runner YAML config")
        args = parser.parse_args(argv)
        config = load_config(args.config)
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
) -> int:
    state_client = state_client or RobotStateClient(config.robot_state.bind, config.robot_state.stale_timeout_sec)
    command_client = command_client or ServoCommandClient(
        config.servo_command.endpoint,
        config.servo_command.timeout_sec,
    )
    source = source or make_action_source(config)
    safety_gate = SafetyGate(
        config.mode,
        config.safety,
        config.robot_state.stale_timeout_sec,
        geometry_status=_load_runtime_geometry_status(config),
    )
    state_client.start()
    armed_for_motion = False
    lease_acquired = False
    period = 1.0 / max(config.command_rate_hz, 1.0)
    startup_deadline = monotonic_fn() + max(config.runtime.startup_timeout_sec, 0.0)
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
            if config.servo_command.acquire_lease and not lease_acquired:
                try:
                    command_client.acquire_lease(
                        StateStreamLeaseReadback(state_client),
                        timeout_sec=config.servo_command.lease_readback_timeout_sec,
                        monotonic_fn=monotonic_fn,
                        sleep_fn=sleep_fn,
                    )
                except TimeoutError as exc:
                    print(f"policy_runner lease_readback_timeout: {exc}", file=stderr)
                    return LEASE_READBACK_TIMEOUT_EXIT_CODE
                lease_acquired = True
            intent = source.next_intent(snapshot, now)
            decision = safety_gate.evaluate(snapshot, intent, getattr(source, "requirements", None), now)
            if decision.allowed and intent is not None:
                if intent.is_motion and not armed_for_motion:
                    arm = CommandIntent.arm_motion(timeout_sec=config.servo_command.timeout_sec)
                    arm_decision = safety_gate.evaluate(snapshot, arm, getattr(source, "requirements", None), now)
                    if arm_decision.allowed:
                        command_client.send(arm)
                        armed_for_motion = True
                command_client.send(intent)
            sleep_fn(period)
    except KeyboardInterrupt:
        return 0
    finally:
        _close_if_supported(source)
        state_client.close()
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
        left = config.spacemouse_cartesian_dual.left
        right = config.spacemouse_cartesian_dual.right
        return DualSpaceMouseCartesianActionSource(
            left_reader=_LazyHidSpaceMouseReader(
                device=left.device,
                path=left.path,
                device_number=left.device_number,
            ),
            right_reader=_LazyHidSpaceMouseReader(
                device=right.device,
                path=right.path,
                device_number=right.device_number,
            ),
            frame=config.spacemouse_cartesian_dual.frame,
            max_linear_velocity_m_s=config.spacemouse_cartesian_dual.max_linear_velocity_m_s,
            max_angular_velocity_rad_s=config.spacemouse_cartesian_dual.max_angular_velocity_rad_s,
            deadband=config.spacemouse_cartesian_dual.deadband,
            response_curve_gamma=config.spacemouse_cartesian_dual.response_curve_gamma,
            sample_hold_timeout_sec=config.spacemouse_cartesian_dual.sample_hold_timeout_sec,
            left_deadman_button=left.deadman_button,
            right_deadman_button=right.deadman_button,
            timeout_sec=config.servo_command.timeout_sec,
            allow_rbpodo_controller_simulation=(
                config.safety.allow_rbpodo_controller_simulation_cartesian
            ),
        )
    raise ValueError(f"unknown action_source: {config.action_source}")


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
        choices=("tiny_cnn", "resnet18", "resnet50", "dinov3"),
        default="tiny_cnn",
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
    flow_infer.add_argument("--sample-steps", type=int, default=16)
    flow_infer.add_argument("--device", default="auto")
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
        help="Fail unless the retarget config has status=measured",
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
        help="Fail unless the retarget config has status=measured",
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

        return run_ml_preflight(vision_backbone=args.vision_backbone)
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
        from .flow_inference import FlowMatchingActionSource

        config = load_config(args.config)
        camera_client = None
        if config.camera.enable:
            from .camera_bundle_client import CameraBundleClient

            camera_client = CameraBundleClient(
                zmq_endpoint=config.camera.zmq_endpoint,
                topic=config.camera.bundle_topic,
                max_age_ms=config.camera.max_age_ms,
            )
        try:
            source = FlowMatchingActionSource(
                args.checkpoint,
                timeout_sec=config.servo_command.timeout_sec,
                camera_client=camera_client,
                sample_steps=args.sample_steps,
                max_linear_step_m=args.max_linear_step_m,
                max_angular_step_rad=args.max_angular_step_rad,
                device=args.device,
            )
        except Exception:
            if camera_client is not None:
                camera_client.close()
            raise
        return run(config, source=source)
    raise ValueError(f"unknown policy_runner command: {args.command}")
