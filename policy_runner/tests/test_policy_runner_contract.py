from __future__ import annotations

import contextlib
import json
import sys
import tempfile
import time
import unittest
from io import StringIO
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.action_sources.hold import HoldActionSource
from policy_runner.action_sources.joint_sine import JointSineActionSource
from policy_runner.action_sources.tcp_pose_target import tcp_pose_target_stand_intent
from policy_runner.arm_init_control import (
    ARM_INIT_COMMAND_SCHEMA,
    ArmInitOverrideController,
    parse_arm_init_command,
)
from policy_runner.config import config_from_mapping, load_config
from policy_runner.main import LEASE_READBACK_TIMEOUT_EXIT_CODE, STARTUP_TIMEOUT_EXIT_CODE, make_action_source, run
from policy_runner.record_control import RecordCommand, RecordingSupervisor, parse_record_command
from policy_runner.recording import _hash_canonical_json
from policy_runner.robot_state_client import (
    RobotStateClient,
    StateSnapshot,
    StateStreamLeaseReadback,
    command_source_lease_from_snapshot,
)
from policy_runner.safety import ActionRequirements, SafetyConfig, SafetyGate
from policy_runner.servo_command_client import CommandIntent, ServoCommandClient
from policy_runner.training import (
    ACTION_DIM,
    STATE_DIM,
    BehaviorCloningActionSource,
    _PolicyNet,
)

try:
    import torch
except ModuleNotFoundError:
    torch = None


def sample_state(**overrides):
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "left": {"has_valid_joint_state": True, "q_actual_deg": [0, -30, 80, 0, 60, 0]},
        "right": {"has_valid_joint_state": True, "q_actual_deg": [0, -30, 80, 0, 60, 0]},
    }
    payload.update(overrides)
    return StateSnapshot(payload=payload, received_monotonic=time.monotonic())


def sample_config_snapshots(path_kp_pos=6.0):
    return {
        "cartesian_control_snapshot": {
            "schema": "robotics_lab.cartesian_control_snapshot.v1",
            "enable": True,
            "path_kp_pos": path_kp_pos,
        },
        "kinematics_snapshot": {
            "schema": "robotics_lab.kinematics_snapshot.v1",
            "provider": "pinocchio",
            "ik": {"damping": 0.001},
        },
    }


class FakeRecvSocket:
    def __init__(self, packets):
        self.packets = list(packets)
        self.bound = None
        self.timeout = None
        self.closed = False

    def bind(self, address):
        self.bound = address

    def getsockname(self):
        host, port = self.bound
        return (host, 50120 if port == 0 else port)

    def settimeout(self, timeout):
        self.timeout = timeout

    def recvfrom(self, _size):
        if not self.packets:
            raise TimeoutError("no fake packet queued")
        return self.packets.pop(0), ("127.0.0.1", 50000)

    def close(self):
        self.closed = True


class FakeSendSocket:
    def __init__(self):
        self.sent = []
        self.closed = False

    def sendto(self, data, address):
        self.sent.append((data, address))
        return len(data)

    def close(self):
        self.closed = True


class FakeStateClient:
    def __init__(self, snapshots):
        self._snapshots = list(snapshots)
        self.started = False
        self.closed = False

    @property
    def latest(self):
        if not self._snapshots:
            return None
        return self._snapshots[0]

    def start(self):
        self.started = True

    def close(self):
        self.closed = True


class FakeLatestSequenceStateClient:
    def __init__(self, snapshots):
        self._snapshots = list(snapshots)
        self.latest_reads = 0

    @property
    def latest(self):
        self.latest_reads += 1
        if not self._snapshots:
            return None
        if len(self._snapshots) == 1:
            return self._snapshots[0]
        return self._snapshots.pop(0)


class FakeCommandClient:
    def __init__(self):
        self.sent = []
        self.acquire_calls = []
        self.closed = False

    def send(self, intent):
        self.sent.append(intent)
        return len(self.sent)

    def acquire_lease(self, readback, **kwargs):
        self.acquire_calls.append((readback, kwargs))
        self.sent.append(CommandIntent.acquire_lease())
        return None

    def release_lease(self):
        self.sent.append(CommandIntent.release_lease())
        return len(self.sent)

    def close(self):
        self.closed = True


class FakeRecordingCameraClient:
    instances = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.closed = False
        FakeRecordingCameraClient.instances.append(self)

    def close(self):
        self.closed = True


class FakeHdf5Recorder:
    instances = []

    def __init__(self, output_dir, **kwargs):
        self.output_dir = output_dir
        self.kwargs = kwargs
        self.has_active_episode = False
        self.episode_id = None
        self.frame_count = 0
        self.start_calls = []
        self.frames = []
        self.closed = False
        FakeHdf5Recorder.instances.append(self)

    def start_episode(self, **kwargs):
        self.has_active_episode = True
        self.episode_id = "episode-test"
        self.start_calls.append(kwargs)

    def record_frame(self, **kwargs):
        self.frames.append(kwargs)
        self.frame_count += 1

    def close(self):
        self.closed = True
        self.has_active_episode = False


class FakeRunRecordingSupervisor:
    def __init__(self):
        self.drain_calls = []
        self.control_payloads = []
        self.frames = []
        self.stamped = []
        self.published = []
        self.closed = False

    def drain_commands(self, snapshot, *, action_source):
        self.drain_calls.append((snapshot, action_source))

    def drain_control_payloads(self):
        return []

    def dispatch_control_payloads(self, payloads, snapshot, *, action_source):
        self.control_payloads.append((list(payloads), snapshot, action_source))
        self.drain_calls.append((snapshot, action_source))

    def stamp_snapshot(self, snapshot):
        snapshot.payload["recording"] = {
            "recording": True,
            "state": "recording",
            "episode_name": "episode-run",
            "frame_count": len(self.frames),
            "rate_hz": 30.0,
        }
        self.stamped.append(snapshot)

    def record_frame(self, snapshot, *, action_packet, action_host_time_ns, action_seq):
        self.frames.append((snapshot, action_packet, action_host_time_ns, action_seq))

    def publish_status(self, *, now_monotonic, force=False, arm_init=None):
        self.published.append((now_monotonic, force, arm_init))

    def close(self):
        self.closed = True


class FakeArmInitControlSupervisor(FakeRunRecordingSupervisor):
    def __init__(self, payloads):
        super().__init__()
        self._payloads = list(payloads)

    def drain_control_payloads(self):
        payloads = self._payloads
        self._payloads = []
        return payloads


class CloseableSource:
    requirements = ActionRequirements(requires_valid_joint_state=False)

    def __init__(self):
        self.closed = False

    def next_intent(self, snapshot, now_monotonic):
        _ = snapshot, now_monotonic
        return None

    def close(self):
        self.closed = True


class CartesianOnceSource:
    requirements = ActionRequirements(
        requires_geometry=True,
        requires_valid_tcp_pose=True,
        simulation_only=True,
        requires_observed_simulation=True,
        cartesian_motion=True,
    )

    def next_intent(self, snapshot, now_monotonic):
        _ = snapshot, now_monotonic
        return tcp_pose_target_stand_intent(left=(0.3, 0.1, 0.5, 0.0, 0.0, 0.0))


def geometry_file_text():
    return """\
calibration_id: CONFIG_ESTIMATE_TEST
status: configured_estimate
geometry_valid_for_real_policy: false
robot:
  T_stand_left_base:
    parent: stand
    child: left_base
    xyz_rpy: [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
    status: configured_estimate
  T_stand_right_base:
    parent: stand
    child: right_base
    xyz_rpy: [-0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
    status: configured_estimate
"""


class PolicyRunnerContractTest(unittest.TestCase):
    def test_config_example_loads_without_yaml_dependency(self):
        cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "replay_sim.yaml")
        self.assertEqual(cfg.action_source, "hold")

    def test_rbpodo_pgmode_spacemouse_config_loads_with_explicit_safety_opt_in(self):
        config_dir = Path(__file__).resolve().parents[1] / "config"
        cfg = load_config(config_dir / "rbpodo_pgmode_spacemouse_500hz_ack.yaml")

        self.assertEqual(cfg.mode, "real")
        self.assertEqual(cfg.action_source, "dual_spacemouse_pose_target")
        self.assertFalse(cfg.safety.allow_real_motion)
        self.assertTrue(cfg.safety.allow_rbpodo_controller_simulation_cartesian)
        self.assertFalse(cfg.safety.allow_configured_estimate_geometry_in_real)
        self.assertEqual(cfg.command_rate_hz, 500.0)
        self.assertEqual(cfg.servo_command.endpoint, "udp://127.0.0.1:50256")
        self.assertEqual(cfg.robot_state.bind, "udp://0.0.0.0:50376")
        self.assertTrue(cfg.servo_command.acquire_lease)
        self.assertEqual(cfg.spacemouse_pose_target_dual.max_linear_step_m, 0.001)
        self.assertEqual(cfg.spacemouse_pose_target_dual.max_angular_step_rad, 0.01)
        self.assertEqual(cfg.spacemouse_pose_target_dual.sample_stale_timeout_sec, 0.05)
        self.assertFalse(cfg.spacemouse_pose_target_dual.gripper_buttons.enable)
        self.assertEqual(cfg.recording.dataset_metadata["backend_type"], "rbpodo")
        self.assertEqual(cfg.recording.dataset_metadata["operation_mode"], "simulation")
        self.assertFalse(cfg.recording.dataset_metadata["physical_motion_expected"])

        source = make_action_source(cfg)
        try:
            self.assertTrue(source.requirements.allow_rbpodo_controller_simulation_cartesian)
        finally:
            close = getattr(source, "close", None)
            if callable(close):
                close()

    def test_stack_configs_record_six_rgbd_streams_at_30hz(self):
        config_dir = Path(__file__).resolve().parents[1] / "config"
        expected = [
            "left_realsense_color",
            "left_realsense_depth",
            "right_realsense_color",
            "right_realsense_depth",
            "head_color",
            "head_depth",
        ]
        for name in ("stack_sim.yaml", "stack_real.yaml", "flow_real_realsense.yaml"):
            with self.subTest(name=name):
                cfg = load_config(config_dir / name)
                self.assertEqual(cfg.recording.rate_hz, 30.0)
                self.assertEqual(cfg.camera.expected_cameras, expected)
                self.assertTrue(cfg.camera.record_zero_on_missing)

    def test_record_command_parser_accepts_start_stop_schema(self):
        start = parse_record_command(
            json.dumps(
                {
                    "schema": "robotics_lab.record_cmd.v1",
                    "command": "start",
                    "task": "fold towel",
                    "operator": "operator-a",
                }
            ).encode("utf-8")
        )
        self.assertEqual(start.command, "start")
        self.assertEqual(start.task, "fold towel")
        self.assertEqual(start.operator, "operator-a")
        stop = parse_record_command({"schema": "robotics_lab.record_cmd.v1", "command": "stop"})
        self.assertEqual(stop.command, "stop")
        with self.assertRaises(ValueError):
            parse_record_command({"schema": "robotics_lab.record_cmd.v1", "command": "pause"})

    def test_arm_init_command_parser_and_toggle_state(self):
        left_q = [1, 2, 3, 4, 5, 6]
        right_q = [-1, -2, -3, -4, -5, -6]
        command = parse_arm_init_command(
            {
                "schema": ARM_INIT_COMMAND_SCHEMA,
                "arms": "left",
                "action": "toggle",
                "left_q_deg": left_q,
                "right_q_deg": right_q,
            }
        )
        self.assertEqual(command.arms, "left")
        self.assertEqual(command.left_q_deg, tuple(float(v) for v in left_q))

        controller = ArmInitOverrideController()
        self.assertTrue(controller.handle_command(command))
        self.assertTrue(controller.status_block()["init_override_left"])
        self.assertFalse(controller.status_block()["init_override_right"])

        both = parse_arm_init_command(
            {
                "schema": ARM_INIT_COMMAND_SCHEMA,
                "arms": "both",
                "action": "toggle",
                "left_q_deg": left_q,
                "right_q_deg": right_q,
            }
        )
        self.assertTrue(controller.handle_command(both))
        self.assertFalse(controller.status_block()["init_override_left"])
        self.assertFalse(controller.status_block()["init_override_right"])

        self.assertTrue(controller.handle_command(both))
        self.assertTrue(controller.status_block()["init_override_left"])
        self.assertTrue(controller.status_block()["init_override_right"])

        with self.assertRaises(ValueError):
            parse_arm_init_command({"schema": ARM_INIT_COMMAND_SCHEMA, "arms": "middle"})

    def test_arm_init_override_composes_mixed_policy_command(self):
        controller = ArmInitOverrideController()
        controller.handle_command(
            parse_arm_init_command(
                {
                    "schema": ARM_INIT_COMMAND_SCHEMA,
                    "arms": "left",
                    "left_q_deg": [1, 0, 0, 0, 0, 0],
                    "right_q_deg": [0, 1, 0, 0, 0, 0],
                }
            )
        )
        source_intent = tcp_pose_target_stand_intent(
            left=(0.1, 0.2, 0.3, 0, 0, 0),
            right=(0.4, 0.5, 0.6, 0, 0, 0),
        )

        mixed = controller.compose_intent(source_intent)

        self.assertIsNotNone(mixed)
        self.assertEqual(mixed.mode, "Hold")
        self.assertTrue(mixed.is_motion)
        self.assertEqual(mixed.left["mode"], "JointTarget")
        self.assertEqual(mixed.left["joint_target_profile"], "init_motion")
        self.assertEqual(mixed.left["q_target_deg"], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(mixed.right["mode"], "TcpPoseTarget")
        self.assertEqual(mixed.right["tcp_target_stand"], [0.4, 0.5, 0.6, 0.0, 0.0, 0.0])

    def test_recording_supervisor_starts_records_stamps_and_stops(self):
        FakeRecordingCameraClient.instances.clear()
        FakeHdf5Recorder.instances.clear()
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "geometry": {"path": ""},
                "recording": {"output_dir": "episodes", "rate_hz": 30.0},
                "camera": {
                    "expected_cameras": [
                        "left_realsense_color",
                        "left_realsense_depth",
                        "right_realsense_color",
                        "right_realsense_depth",
                        "head_color",
                        "head_depth",
                    ],
                    "record_zero_on_missing": True,
                },
            }
        )
        supervisor = RecordingSupervisor(
            cfg,
            camera_client_factory=FakeRecordingCameraClient,
            recorder_factory=FakeHdf5Recorder,
        )
        state = sample_state()

        supervisor.handle_command(
            RecordCommand("start", task="collect test", operator="op"),
            state,
            action_source="teleop_mux",
        )
        self.assertTrue(supervisor.recording)
        self.assertEqual(FakeRecordingCameraClient.instances[0].kwargs["include_depth"], True)
        recorder = FakeHdf5Recorder.instances[0]
        self.assertEqual(recorder.kwargs["recording_rate_hz"], 30.0)
        self.assertEqual(recorder.kwargs["expected_cameras"], cfg.camera.expected_cameras)
        self.assertTrue(recorder.kwargs["record_zero_on_missing"])
        self.assertEqual(recorder.start_calls[0]["task_description"], "collect test")
        self.assertEqual(recorder.start_calls[0]["action_source"], "teleop_mux")

        packet = {"seq": 9, "mode": "Hold", "left": {"mode": "Hold"}, "right": {"mode": "Hold"}}
        supervisor.record_frame(state, action_packet=packet, action_host_time_ns=123, action_seq=9)
        self.assertEqual(supervisor.frame_count, 1)
        supervisor.stamp_snapshot(state)
        self.assertEqual(state.payload["recording"]["state"], "recording")
        self.assertEqual(state.payload["recording"]["episode_name"], "episode-test")
        self.assertEqual(state.payload["recording"]["frame_count"], 1)

        supervisor.handle_command(RecordCommand("stop"), state, action_source="teleop_mux")
        self.assertFalse(supervisor.recording)
        self.assertTrue(recorder.closed)
        self.assertTrue(FakeRecordingCameraClient.instances[0].closed)
        supervisor.stamp_snapshot(state)
        self.assertEqual(state.payload["recording"]["state"], "idle")

    def test_rbpodo_pgmode_spacemouse_server_template_is_controller_sim_only(self):
        root = Path(__file__).resolve().parents[2]
        template = root / "rb_servo_server" / "config" / "dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml"
        text = template.read_text()

        self.assertIn("rate_hz: 500", text)
        self.assertIn("mode: sdk_ack_worker", text)
        self.assertIn('command_bind: "udp://127.0.0.1:50256"', text)
        self.assertIn('"udp://127.0.0.1:50366"', text)
        self.assertIn('"udp://127.0.0.1:50376"', text)
        self.assertIn("operation_mode: simulation", text)
        self.assertIn("allow_in_controller_simulation: true", text)
        self.assertIn("allow_in_real: false", text)
        self.assertIn("enforce_lease: true", text)
        self.assertIn("provider: null", text)
        self.assertIn("enable: false", text)

    def test_rbpodo_pgmode_spacemouse_tool_exists(self):
        root = Path(__file__).resolve().parents[2]
        tool = root / "tools" / "rbpodo_pgmode_spacemouse.sh"
        text = tool.read_text()
        generator = root / "tools" / "create_rbpodo_pgmode_spacemouse_local_config.sh"
        generator_text = generator.read_text()

        self.assertTrue(tool.exists())
        self.assertTrue(generator.exists())
        self.assertIn("RB_ALLOW_RBPODO_ASYNC_STREAMING", text)
        self.assertIn("server-dry-run", text)
        self.assertIn("policy-dry-run", text)
        self.assertIn("check", text)
        self.assertIn("127.0.0.1:50256", text)
        self.assertIn("0.0.0.0:50376", text)
        self.assertIn("127.0.0.1:50366", text)
        self.assertIn("mock_script: pgmode_spacemouse_smoke", text)
        self.assertIn("--left-ip", generator_text)
        self.assertIn("RB_PGMODE_SPACEMOUSE_LEFT_IP", generator_text)
        self.assertIn("ignored by git", generator_text)

    def test_runtime_config_defaults_and_parses_startup_timeout(self):
        default_cfg = config_from_mapping({"schema": "robotics_lab.policy_runner.v1"})
        self.assertEqual(default_cfg.runtime.startup_timeout_sec, 5.0)
        self.assertFalse(default_cfg.servo_command.acquire_lease)
        self.assertEqual(default_cfg.servo_command.lease_readback_timeout_sec, 1.0)
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "runtime": {"startup_timeout_sec": 0.25},
                "servo_command": {"acquire_lease": True, "lease_readback_timeout_sec": 0.75},
            }
        )
        self.assertEqual(cfg.runtime.startup_timeout_sec, 0.25)
        self.assertTrue(cfg.servo_command.acquire_lease)
        self.assertEqual(cfg.servo_command.lease_readback_timeout_sec, 0.75)

    def test_umi_sample_stale_timeout_alias_maps_to_hold_timeout(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "umi_dual_cartesian": {"sample_stale_timeout_sec": 0.25},
            }
        )

        self.assertEqual(cfg.umi_dual_cartesian.sample_hold_timeout_sec, 0.25)

        with self.assertRaisesRegex(ValueError, "must not set both"):
            config_from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.v1",
                    "umi_dual_cartesian": {
                        "sample_hold_timeout_sec": 0.2,
                        "sample_stale_timeout_sec": 0.25,
                    },
                }
            )

    def test_stack_policy_configs_load(self):
        config_dir = Path(__file__).resolve().parents[1] / "config"

        real_cfg = load_config(config_dir / "stack_real.yaml")
        sim_cfg = load_config(config_dir / "stack_sim.yaml")

        self.assertEqual(real_cfg.umi_dual_cartesian.sample_hold_timeout_sec, 0.5)
        self.assertEqual(sim_cfg.umi_dual_cartesian.sample_hold_timeout_sec, 0.05)
        self.assertEqual(real_cfg.spacemouse_pose_target_dual.sample_stale_timeout_sec, 0.05)
        self.assertEqual(sim_cfg.spacemouse_pose_target_dual.sample_stale_timeout_sec, 0.05)
        self.assertTrue(real_cfg.spacemouse_pose_target_dual.gripper_buttons.enable)
        self.assertTrue(sim_cfg.spacemouse_pose_target_dual.gripper_buttons.enable)
        self.assertEqual(real_cfg.spacemouse_pose_target_dual.gripper_buttons.open_percent, 100.0)
        self.assertEqual(real_cfg.spacemouse_pose_target_dual.gripper_buttons.close_percent, 10.0)

    def test_spacemouse_gripper_buttons_config_validation(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "spacemouse_pose_target_dual": {
                    "require_deadman": False,
                    "gripper_buttons": {
                        "enable": True,
                        "open_button": 2,
                        "close_button": 3,
                        "open_percent": 90,
                        "close_percent": 12,
                    },
                },
            }
        )

        buttons = cfg.spacemouse_pose_target_dual.gripper_buttons
        self.assertTrue(buttons.enable)
        self.assertEqual(buttons.open_button, 2)
        self.assertEqual(buttons.close_button, 3)
        self.assertEqual(buttons.open_percent, 90.0)
        self.assertEqual(buttons.close_percent, 12.0)

        with self.assertRaisesRegex(ValueError, "must differ"):
            config_from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.v1",
                    "spacemouse_pose_target_dual": {
                        "gripper_buttons": {
                            "enable": True,
                            "open_button": 1,
                            "close_button": 1,
                        },
                    },
                }
            )

        with self.assertRaisesRegex(ValueError, "open_percent"):
            config_from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.v1",
                    "spacemouse_pose_target_dual": {
                        "gripper_buttons": {"enable": True, "open_percent": 120},
                    },
                }
            )

        with self.assertRaisesRegex(ValueError, "conflicts"):
            config_from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.v1",
                    "spacemouse_pose_target_dual": {
                        "require_deadman": True,
                        "left": {"deadman_button": 0},
                        "right": {"deadman_button": 2},
                        "gripper_buttons": {
                            "enable": True,
                            "open_button": 0,
                            "close_button": 1,
                        },
                    },
                }
            )

    def test_recording_metadata_must_be_mapping(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "recording": {
                    "dataset_metadata": {
                        "backend_type": "rbpodo",
                        "operation_mode": "simulation",
                    }
                },
            }
        )
        self.assertEqual(cfg.recording.dataset_metadata["backend_type"], "rbpodo")

        with self.assertRaisesRegex(ValueError, "recording.dataset_metadata must be a mapping"):
            config_from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.v1",
                    "recording": {"dataset_metadata": "not-a-map"},
                }
            )

    def test_make_action_source_accepts_all_configured_action_sources_without_hardware(self):
        for action_source in (
            "hold",
            "joint_sine",
            "master_arm_joint",
            "dual_spacemouse_pose_target",
        ):
            with self.subTest(action_source=action_source):
                cfg = config_from_mapping(
                    {
                        "schema": "robotics_lab.policy_runner.v1",
                        "action_source": action_source,
                    }
                )
                source = make_action_source(cfg)
                self.assertIsNotNone(source)
                close = getattr(source, "close", None)
                if callable(close):
                    close()

    def test_udp_state_subscriber_receives_latest_snapshot(self):
        packet = json.dumps(sample_state().payload).encode("utf-8")
        fake_socket = FakeRecvSocket([packet])
        client = RobotStateClient(
            "udp://127.0.0.1:0",
            stale_timeout_sec=0.5,
            socket_factory=lambda *_args: fake_socket,
        )
        client.open()
        try:
            snapshot = client.poll_once(timeout_sec=0.5)
            self.assertIsNotNone(snapshot)
            self.assertEqual(snapshot.payload["motion_state"], "ConnectedHold")
            self.assertFalse(client.is_latest_stale())
        finally:
            client.close()

    def test_command_sender_emits_joint_target_profile_packet(self):
        fake_socket = FakeSendSocket()
        client = ServoCommandClient(
            "udp://127.0.0.1:50010",
            socket_factory=lambda *_args: fake_socket,
        )
        try:
            seq = client.send(CommandIntent.init_motion(left=[1, 0, 0, 0, 0, 0], right=[-1, 0, 0, 0, 0, 0]))
            data, address = fake_socket.sent[0]
            packet = json.loads(data.decode("utf-8"))
            self.assertEqual(seq, 1)
            self.assertEqual(address, ("127.0.0.1", 50010))
            self.assertEqual(packet["seq"], 1)
            self.assertEqual(packet["mode"], "JointTarget")
            self.assertEqual(packet["source_id"], "policy_runner")
            self.assertTrue(packet["session_id"])
            self.assertEqual(packet["timeout_sec"], 0.2)
            self.assertEqual(packet["left"]["mode"], "JointTarget")
            self.assertEqual(packet["left"]["q_target_deg"], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.assertEqual(packet["left"]["joint_target_profile"], "init_motion")
            self.assertEqual(packet["right"]["mode"], "JointTarget")
            self.assertEqual(packet["right"]["joint_target_profile"], "init_motion")
        finally:
            client.close()

    def test_command_packet_schemas_match_rb_servo_parser_contract(self):
        client = ServoCommandClient("udp://127.0.0.1:50010", socket_factory=lambda *_args: FakeSendSocket())
        cases = [
            CommandIntent.hold(),
            CommandIntent.arm_motion(),
            CommandIntent.disarm_motion(),
            CommandIntent.emergency_stop(),
            CommandIntent.reset_fault(),
            CommandIntent.joint_target(left=[0, 1, 2, 3, 4, 5], right=[5, 4, 3, 2, 1, 0]),
            CommandIntent.init_motion(left=[1, 0, 0, 0, 0, 0], right=[0, 1, 0, 0, 0, 0]),
        ]
        for seq, intent in enumerate(cases, start=1):
            with self.subTest(mode=intent.mode):
                packet = client.build_packet(intent, seq)
                self.assertEqual(packet["seq"], seq)
                self.assertIn("mode", packet)
                self.assertEqual(packet["source_id"], "policy_runner")
                self.assertTrue(packet["session_id"])
                self.assertIn("timeout_sec", packet)
                self.assertGreater(packet["timeout_sec"], 0.0)
        target = client.build_packet(cases[-2], 6)
        self.assertEqual(target["left"]["mode"], "JointTarget")
        self.assertEqual(target["left"]["q_target_deg"], [0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
        profiled = client.build_packet(cases[-1], 7)
        self.assertEqual(profiled["right"]["mode"], "JointTarget")
        self.assertEqual(profiled["right"]["q_target_deg"], [0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(profiled["right"]["joint_target_profile"], "init_motion")

    def test_command_sender_emits_acquire_lease_and_reuses_readback_token(self):
        fake_socket = FakeSendSocket()
        client = ServoCommandClient(
            "udp://127.0.0.1:50010",
            socket_factory=lambda *_args: fake_socket,
            source_id="policy_runner",
            session_id="session-1",
        )
        state_client = FakeLatestSequenceStateClient(
            [
                sample_state(command_source={"active": False}),
                sample_state(
                    command_source={
                        "enforce_lease": True,
                        "active": True,
                        "expired": False,
                        "active_source_id": "policy_runner",
                        "active_session_id": "session-1",
                        "active_lease_token": "lease-token",
                        "verdict": "Ok",
                    }
                ),
            ]
        )
        clock = iter([0.0, 0.01, 0.02]).__next__
        try:
            result = client.acquire_lease(
                StateStreamLeaseReadback(state_client),
                timeout_sec=0.1,
                monotonic_fn=clock,
                sleep_fn=lambda _seconds: None,
            )
            self.assertEqual(result.seq, 1)
            self.assertEqual(result.lease_token, "lease-token")
            self.assertEqual(client.lease_token, "lease-token")

            acquire_packet = json.loads(fake_socket.sent[0][0].decode("utf-8"))
            self.assertEqual(acquire_packet["mode"], "AcquireLease")
            self.assertEqual(acquire_packet["source_id"], "policy_runner")
            self.assertEqual(acquire_packet["session_id"], "session-1")
            self.assertNotIn("lease_token", acquire_packet)

            client.send(CommandIntent.hold())
            hold_packet = json.loads(fake_socket.sent[1][0].decode("utf-8"))
            self.assertEqual(hold_packet["lease_token"], "lease-token")
        finally:
            client.close()

    def test_state_stream_lease_readback_times_out_on_wrong_owner(self):
        snapshot = sample_state(
            command_source={
                "enforce_lease": True,
                "active": True,
                "expired": False,
                "active_source_id": "rb_gui",
                "active_session_id": "gui-session",
                "active_lease_token": "gui-token",
                "verdict": "Ok",
            }
        )
        readback = command_source_lease_from_snapshot(snapshot)
        self.assertFalse(readback.matches("policy_runner", "policy-session"))

        state_client = FakeLatestSequenceStateClient([snapshot])
        with self.assertRaisesRegex(TimeoutError, "active_source_id=rb_gui"):
            StateStreamLeaseReadback(state_client).wait_for_active_lease(
                source_id="policy_runner",
                session_id="policy-session",
                timeout_sec=0.0,
                monotonic_fn=lambda: 1.0,
                sleep_fn=lambda _seconds: None,
            )

    def test_hold_receives_state_and_sends_no_motion_by_default(self):
        source = HoldActionSource()
        self.assertIsNone(source.next_intent(sample_state(), time.monotonic()))

    def test_joint_sources_are_simulation_only_by_default(self):
        sine = JointSineActionSource((1, 1, 1, 1, 1, 1))
        self.assertTrue(sine.requirements.simulation_only)

    def test_real_mode_blocks_motion_without_explicit_allow(self):
        gate = SafetyGate("real", SafetyConfig(allow_real_motion=False), stale_timeout_sec=0.5)
        intent = CommandIntent.joint_target(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        decision = gate.evaluate(sample_state(), intent, now_monotonic=time.monotonic())
        # Real/sim gating retired: allowed.
        self.assertTrue(decision.allowed)

    def test_observed_real_mode_blocks_motion_without_explicit_allow(self):
        gate = SafetyGate("simulation", SafetyConfig(allow_real_motion=False), stale_timeout_sec=0.5)
        intent = CommandIntent.joint_target(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        decision = gate.evaluate(sample_state(run_mode="real"), intent, now_monotonic=time.monotonic())
        # Real/sim gating retired: allowed.
        self.assertTrue(decision.allowed)

    def test_safety_blocks_stale_fault_and_invalid_state(self):
        gate = SafetyGate("simulation", SafetyConfig(), stale_timeout_sec=0.01)
        intent = CommandIntent.joint_target(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        stale = StateSnapshot(sample_state().payload, time.monotonic() - 1.0)
        self.assertEqual(gate.evaluate(stale, intent).reason, "state_stream_stale")
        self.assertEqual(gate.evaluate(sample_state(fault_latched=True), intent).reason, "fault_latched")
        invalid = sample_state(left={"has_valid_joint_state": False, "q_actual_deg": [0, 0, 0, 0, 0, 0]})
        self.assertEqual(gate.evaluate(invalid, intent).reason, "invalid_joint_state")

    def test_safety_blocks_missing_camera_or_kinematics_requirements(self):
        intent = CommandIntent.joint_target(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        camera_gate = SafetyGate("simulation", SafetyConfig(camera_available=False), stale_timeout_sec=0.5)
        camera_decision = camera_gate.evaluate(
            sample_state(),
            intent,
            ActionRequirements(requires_camera=True),
        )
        self.assertEqual(camera_decision.reason, "camera_unavailable")
        kinematics_gate = SafetyGate("simulation", SafetyConfig(kinematics_available=False), stale_timeout_sec=0.5)
        kinematics_decision = kinematics_gate.evaluate(
            sample_state(),
            intent,
            ActionRequirements(requires_kinematics=True),
        )
        self.assertEqual(kinematics_decision.reason, "kinematics_unavailable")

    def test_joint_sine_uses_current_joint_state(self):
        source = JointSineActionSource((1, 1, 1, 1, 1, 1), frequency_hz=0.25, selected_arm="left")
        state = sample_state()
        source.next_intent(state, 10.0)
        intent = source.next_intent(state, 11.0)
        self.assertEqual(intent.left["mode"], "JointTarget")
        self.assertEqual(intent.right["mode"], "Hold")
        self.assertEqual(len(intent.left["q_target_deg"]), 6)

    def test_startup_timeout_returns_clean_error_when_no_state_arrives(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "runtime": {"startup_timeout_sec": 0.01},
                "geometry": {"path": ""},
            }
        )
        state_client = FakeStateClient([])
        command_client = FakeCommandClient()
        source = CloseableSource()
        stderr = StringIO()

        result = run(
            cfg,
            state_client=state_client,
            command_client=command_client,
            source=source,
            monotonic_fn=iter([0.0, 0.02]).__next__,
            sleep_fn=lambda _period: None,
            stderr=stderr,
        )

        self.assertEqual(result, STARTUP_TIMEOUT_EXIT_CODE)
        self.assertTrue(state_client.started)
        self.assertTrue(state_client.closed)
        self.assertTrue(command_client.closed)
        self.assertTrue(source.closed)
        self.assertIn("startup_timeout_no_state", stderr.getvalue())
        self.assertEqual(command_client.sent, [])

    def test_run_loads_geometry_from_config_for_geometry_dependent_actions(self):
        geometry_path = Path(self.id().replace(".", "_") + ".yaml")
        geometry_path.write_text(geometry_file_text())
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "geometry": {"path": str(geometry_path)},
                "runtime": {"startup_timeout_sec": 0.1},
            }
        )
        state = sample_state(
            observed_mode="simulation",
            observed_backend="simulator",
            left={
                "has_valid_joint_state": True,
                "has_valid_tcp_pose": True,
                "q_actual_deg": [0, -30, 80, 0, 60, 0],
            },
            right={
                "has_valid_joint_state": True,
                "has_valid_tcp_pose": True,
                "q_actual_deg": [0, -30, 80, 0, 60, 0],
            },
        )
        command_client = FakeCommandClient()
        try:
            result = run(
                cfg,
                state_client=FakeStateClient([state]),
                command_client=command_client,
                source=CartesianOnceSource(),
                monotonic_fn=lambda: time.monotonic(),
                sleep_fn=lambda _period: (_ for _ in ()).throw(KeyboardInterrupt()),
            )
        finally:
            geometry_path.unlink(missing_ok=True)

        self.assertEqual(result, 0)
        self.assertEqual([intent.mode for intent in command_client.sent], ["ArmMotion", "TcpPoseTarget"])

    def test_run_records_this_tick_action_packet_and_stamps_recording_state(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "action_source": "joint_sine",
                "geometry": {"path": ""},
                "runtime": {"startup_timeout_sec": 0.1},
                "recording": {"rate_hz": 30.0},
            }
        )
        state = sample_state()
        command_client = FakeCommandClient()
        recorder = FakeRunRecordingSupervisor()
        seen_states = []

        result = run(
            cfg,
            state_client=FakeStateClient([state]),
            command_client=command_client,
            source=JointSineActionSource((1, 0, 0, 0, 0, 0)),
            sleep_fn=lambda _period: (_ for _ in ()).throw(KeyboardInterrupt()),
            state_sink=seen_states.append,
            recording_supervisor=recorder,
        )

        self.assertEqual(result, 0)
        self.assertTrue(recorder.closed)
        self.assertEqual([intent.mode for intent in command_client.sent], ["ArmMotion", "JointTarget"])
        self.assertEqual(len(recorder.frames), 1)
        _snapshot, action_packet, action_host_time_ns, action_seq = recorder.frames[0]
        self.assertEqual(action_packet["mode"], "JointTarget")
        self.assertEqual(action_packet["seq"], 2)
        self.assertEqual(action_packet["left"]["mode"], "JointTarget")
        self.assertGreater(action_host_time_ns, 0)
        self.assertEqual(action_seq, 2)
        self.assertEqual(seen_states[0].payload["recording"]["state"], "recording")
        self.assertEqual(recorder.drain_calls[0][1], "joint_sine")

    def test_run_applies_arm_init_override_before_safety_and_send(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "geometry": {"path": ""},
                "runtime": {"startup_timeout_sec": 0.1},
                "recording": {"rate_hz": 30.0},
            }
        )

        class DualTcpSource:
            requirements = ActionRequirements(requires_valid_joint_state=True)
            name = "dual_tcp"

            def next_intent(self, snapshot, now_monotonic):
                _ = snapshot, now_monotonic
                return tcp_pose_target_stand_intent(
                    left=(0.1, 0.2, 0.3, 0, 0, 0),
                    right=(0.4, 0.5, 0.6, 0, 0, 0),
                )

            def close(self):
                pass

        command_client = FakeCommandClient()
        recorder = FakeArmInitControlSupervisor(
            [
                {
                    "schema": ARM_INIT_COMMAND_SCHEMA,
                    "arms": "left",
                    "action": "toggle",
                    "left_q_deg": [1, 2, 3, 4, 5, 6],
                    "right_q_deg": [-1, -2, -3, -4, -5, -6],
                }
            ]
        )
        seen_states = []

        result = run(
            cfg,
            state_client=FakeStateClient([sample_state()]),
            command_client=command_client,
            source=DualTcpSource(),
            sleep_fn=lambda _period: (_ for _ in ()).throw(KeyboardInterrupt()),
            state_sink=seen_states.append,
            recording_supervisor=recorder,
        )

        self.assertEqual(result, 0)
        self.assertEqual([intent.mode for intent in command_client.sent], ["ArmMotion", "Hold"])
        mixed = command_client.sent[-1]
        self.assertEqual(mixed.left["mode"], "JointTarget")
        self.assertEqual(mixed.left["joint_target_profile"], "init_motion")
        self.assertEqual(mixed.left["q_target_deg"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(mixed.right["mode"], "TcpPoseTarget")
        self.assertTrue(seen_states[0].payload["arm_init"]["init_override_left"])
        self.assertFalse(seen_states[0].payload["arm_init"]["init_override_right"])
        _snapshot, action_packet, _host_time, action_seq = recorder.frames[0]
        self.assertEqual(action_packet["mode"], "Hold")
        self.assertEqual(action_packet["left"]["joint_target_profile"], "init_motion")
        self.assertEqual(action_packet["right"]["mode"], "TcpPoseTarget")
        self.assertEqual(action_seq, 2)
        self.assertTrue(recorder.published[0][2]["init_override_left"])

    def test_run_acquires_configured_lease_before_motion(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "servo_command": {"acquire_lease": True, "lease_readback_timeout_sec": 0.1},
                "geometry": {"path": ""},
                "runtime": {"startup_timeout_sec": 0.1},
            }
        )
        command_client = FakeCommandClient()
        source = JointSineActionSource((1, 0, 0, 0, 0, 0))

        result = run(
            cfg,
            state_client=FakeStateClient([sample_state()]),
            command_client=command_client,
            source=source,
            monotonic_fn=lambda: time.monotonic(),
            sleep_fn=lambda _period: (_ for _ in ()).throw(KeyboardInterrupt()),
        )

        self.assertEqual(result, 0)
        # Trailing ReleaseLease is the voluntary shutdown handoff (so a restart
        # does not collide with this session's stale lease).
        self.assertEqual(
            [intent.mode for intent in command_client.sent],
            ["AcquireLease", "ArmMotion", "JointTarget", "ReleaseLease"],
        )
        self.assertEqual(len(command_client.acquire_calls), 1)
        self.assertEqual(command_client.acquire_calls[0][1]["timeout_sec"], 0.1)

    def test_run_retries_when_lease_readback_missing(self):
        # Lazy-lease contract: the lease is only acquired on the FIRST motion
        # intent, and a missing/busy lease readback does not kill the process —
        # the tick is dropped with a warning and acquisition is retried.
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "servo_command": {"acquire_lease": True, "lease_readback_timeout_sec": 0.05},
                "geometry": {"path": ""},
                "runtime": {"startup_timeout_sec": 0.5},
            }
        )
        state_client = FakeStateClient([sample_state()])
        send_socket = FakeSendSocket()
        command_client = ServoCommandClient(
            "udp://127.0.0.1:50010",
            socket_factory=lambda *_args: send_socket,
            source_id="policy_runner",
            session_id="session-1",
        )

        class MotionEverySource:
            requirements = None

            def next_intent(self, snapshot, now):
                return CommandIntent.joint_target(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])

            def close(self):
                pass

        stderr = StringIO()
        ticks = {"n": 0}

        def stop_after(_period):
            # The readback poll inside acquire_lease also calls sleep_fn, so
            # stop only after the retry warning is visible (KI inside the
            # poll would short-circuit before the TimeoutError fires).
            ticks["n"] += 1
            if "lease busy" in stderr.getvalue() or ticks["n"] >= 500:
                raise KeyboardInterrupt
            time.sleep(0.001)  # real delay so the 0.05 s readback deadline can pass

        result = run(
            cfg,
            state_client=state_client,
            command_client=command_client,
            source=MotionEverySource(),
            sleep_fn=stop_after,
            stderr=stderr,
        )

        self.assertEqual(result, 0)
        self.assertIn("lease busy", stderr.getvalue())
        modes = [json.loads(data.decode())["mode"] for data, _addr in send_socket.sent]
        # The acquire attempt went out, but no motion command did.
        self.assertIn("AcquireLease", modes)
        self.assertNotIn("ArmMotion", modes)
        self.assertNotIn("ArmMotion", modes)
        for data, _addr in send_socket.sent:
            packet = json.loads(data.decode())
            for arm in ("left", "right"):
                if isinstance(packet.get(arm), dict):
                    self.assertNotEqual(packet[arm].get("mode"), "JointTarget")

    def test_run_releases_lease_after_idle_and_reacquires_on_resume(self):
        # Idle lease handoff contract: after IDLE_LEASE_RELEASE_SEC without a
        # motion intent the loop voluntarily releases the lease (so one-shot
        # GUI commands like InitMotion work between teleop bursts) and lazily
        # re-acquires on the next motion intent.
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "servo_command": {"acquire_lease": True, "lease_readback_timeout_sec": 0.1},
                "geometry": {"path": ""},
                "runtime": {"startup_timeout_sec": 0.1},
            }
        )
        command_client = FakeCommandClient()

        class BurstIdleBurstSource:
            requirements = ActionRequirements(requires_valid_joint_state=False)

            def __init__(self):
                self.calls = 0

            def next_intent(self, snapshot, now_monotonic):
                _ = snapshot, now_monotonic
                self.calls += 1
                if self.calls in (1, 4):
                    return CommandIntent.joint_target(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
                return None

            def close(self):
                pass

        # monotonic_fn calls: startup deadline, then one per tick.
        times = iter([0.0, 0.0, 2.0, 2.05, 2.1])
        sleeps = {"n": 0}

        def sleep_fn(_period):
            sleeps["n"] += 1
            if sleeps["n"] >= 4:
                raise KeyboardInterrupt

        result = run(
            cfg,
            state_client=FakeStateClient([sample_state()]),
            command_client=command_client,
            source=BurstIdleBurstSource(),
            monotonic_fn=lambda: next(times),
            sleep_fn=sleep_fn,
        )

        self.assertEqual(result, 0)
        modes = [intent.mode for intent in command_client.sent]
        # tick1 (t=0.0): lazy acquire + arm + motion; tick2 (t=2.0): idle past
        # the quiet period -> voluntary release; tick3 (t=2.05): still idle, no
        # double release; tick4 (t=2.1): motion resumes -> re-acquire + motion.
        self.assertEqual(
            modes[:6],
            ["AcquireLease", "ArmMotion", "JointTarget", "ReleaseLease", "AcquireLease", "JointTarget"],
        )
        self.assertEqual(len(command_client.acquire_calls), 2)

    def test_missing_runtime_geometry_blocks_geometry_dependent_actions(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "geometry": {"path": ""},
                "runtime": {"startup_timeout_sec": 0.1},
            }
        )
        state = sample_state(
            observed_mode="simulation",
            observed_backend="simulator",
            left={
                "has_valid_joint_state": True,
                "has_valid_tcp_pose": True,
                "q_actual_deg": [0, -30, 80, 0, 60, 0],
            },
            right={
                "has_valid_joint_state": True,
                "has_valid_tcp_pose": True,
                "q_actual_deg": [0, -30, 80, 0, 60, 0],
            },
        )
        command_client = FakeCommandClient()

        result = run(
            cfg,
            state_client=FakeStateClient([state]),
            command_client=command_client,
            source=CartesianOnceSource(),
            monotonic_fn=lambda: time.monotonic(),
            sleep_fn=lambda _period: (_ for _ in ()).throw(KeyboardInterrupt()),
        )

        self.assertEqual(result, 0)
        # Geometry availability gating retired with the real/sim policy rules:
        # the Cartesian command is sent even without runtime geometry.
        self.assertTrue(command_client.sent)

    @unittest.skipIf(torch is None, "torch not installed")
    def test_behavior_cloning_warns_once_on_runtime_config_drift(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "bc.pt"
            self._write_bc_checkpoint(
                checkpoint,
                cartesian_control_hash="a" * 64,
                kinematics_hash="b" * 64,
            )
            source = BehaviorCloningActionSource(checkpoint)
            snapshot = sample_state(**sample_config_snapshots(path_kp_pos=7.0))

            stderr = StringIO()
            with contextlib.redirect_stderr(stderr):
                source.next_intent(snapshot, time.monotonic())
                source.next_intent(snapshot, time.monotonic())

            self.assertEqual(stderr.getvalue().count("config drift detected"), 1)
            self.assertIn("cartesian_control", stderr.getvalue())
            self.assertIn("kinematics", stderr.getvalue())

    @unittest.skipIf(torch is None, "torch not installed")
    def test_behavior_cloning_silent_when_runtime_config_hashes_match(self):
        with tempfile.TemporaryDirectory() as tmp:
            snapshots = sample_config_snapshots(path_kp_pos=7.0)
            checkpoint = Path(tmp) / "bc.pt"
            self._write_bc_checkpoint(
                checkpoint,
                cartesian_control_hash=_hash_canonical_json(snapshots["cartesian_control_snapshot"]),
                kinematics_hash=_hash_canonical_json(snapshots["kinematics_snapshot"]),
            )
            source = BehaviorCloningActionSource(checkpoint)

            stderr = StringIO()
            with contextlib.redirect_stderr(stderr):
                source.next_intent(sample_state(**snapshots), time.monotonic())

            self.assertEqual(stderr.getvalue(), "")

    @unittest.skipIf(torch is None, "torch not installed")
    def test_behavior_cloning_v1_checkpoint_without_config_hashes_is_silent(self):
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "bc.pt"
            self._write_bc_checkpoint(checkpoint)
            source = BehaviorCloningActionSource(checkpoint)

            stderr = StringIO()
            with contextlib.redirect_stderr(stderr):
                source.next_intent(sample_state(**sample_config_snapshots(path_kp_pos=9.0)), time.monotonic())

            self.assertEqual(stderr.getvalue(), "")

    def _write_bc_checkpoint(
        self,
        path: Path,
        *,
        cartesian_control_hash: str | None = None,
        kinematics_hash: str | None = None,
    ) -> None:
        assert torch is not None
        checkpoint = {
            "schema": "robotics_lab.policy_runner.bc_checkpoint.v1",
            "input_dim": STATE_DIM,
            "output_dim": ACTION_DIM,
            "obs_mean": [0.0] * STATE_DIM,
            "obs_std": [1.0] * STATE_DIM,
            "model_state": _PolicyNet(STATE_DIM, ACTION_DIM).state_dict(),
        }
        if cartesian_control_hash is not None:
            checkpoint["cartesian_control_hash"] = cartesian_control_hash
        if kinematics_hash is not None:
            checkpoint["kinematics_hash"] = kinematics_hash
        torch.save(checkpoint, path)


if __name__ == "__main__":
    unittest.main()
