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

from policy_runner.action_sources.tcp_delta import tcp_delta_stand_intent
from policy_runner.action_sources.hold import HoldActionSource
from policy_runner.action_sources.joint_sine import JointSineActionSource
from policy_runner.action_sources.joint_velocity import JointVelocityActionSource
from policy_runner.config import config_from_mapping, load_config
from policy_runner.main import LEASE_READBACK_TIMEOUT_EXIT_CODE, STARTUP_TIMEOUT_EXIT_CODE, make_action_source, run
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

    def close(self):
        self.closed = True


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
        requires_simulator_backend_if_available=True,
        cartesian_motion=True,
    )

    def next_intent(self, snapshot, now_monotonic):
        _ = snapshot, now_monotonic
        return tcp_delta_stand_intent(left=(0.001, 0.0, 0.0, 0.0, 0.0, 0.0))


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
        cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "simulator_hold.yaml")
        self.assertEqual(cfg.action_source, "hold")
        self.assertFalse(cfg.safety.allow_real_motion)

    def test_simulator_action_examples_load_and_remain_simulation_only(self):
        config_dir = Path(__file__).resolve().parents[1] / "config"
        examples = {
            "simulator_spacemouse_joint_velocity.yaml": "spacemouse_joint_velocity",
            "simulator_tcp_delta.yaml": "tcp_delta",
            "simulator_spacemouse_cartesian.yaml": "spacemouse_cartesian",
            "simulator_tcp_twist_local.yaml": "spacemouse_cartesian",
            "simulator_dual_spacemouse_cartesian.yaml": "dual_spacemouse_cartesian",
        }
        for filename, action_source in examples.items():
            with self.subTest(filename=filename):
                path = config_dir / filename
                text = path.read_text()
                self.assertNotIn("172.28.60.200", text)
                self.assertNotIn("172.28.60.201", text)
                cfg = load_config(path)
                self.assertEqual(cfg.mode, "simulation")
                self.assertEqual(cfg.action_source, action_source)
                self.assertFalse(cfg.safety.allow_real_motion)
                self.assertGreater(cfg.runtime.startup_timeout_sec, 0.0)
                source = make_action_source(cfg)
                self.assertIsNotNone(source)
                close = getattr(source, "close", None)
                if callable(close):
                    close()
        self.assertTrue(load_config(config_dir / "simulator_tcp_delta.yaml").tcp_delta.simulation_only)

    def test_rbpodo_pgmode_spacemouse_config_loads_with_explicit_safety_opt_in(self):
        config_dir = Path(__file__).resolve().parents[1] / "config"
        cfg = load_config(config_dir / "rbpodo_pgmode_spacemouse_500hz_ack.yaml")

        self.assertEqual(cfg.mode, "real")
        self.assertEqual(cfg.action_source, "dual_spacemouse_cartesian")
        self.assertFalse(cfg.safety.allow_real_motion)
        self.assertTrue(cfg.safety.allow_rbpodo_controller_simulation_cartesian)
        self.assertFalse(cfg.safety.allow_configured_estimate_geometry_in_real)
        self.assertEqual(cfg.command_rate_hz, 500.0)
        self.assertEqual(cfg.servo_command.endpoint, "udp://127.0.0.1:50256")
        self.assertEqual(cfg.robot_state.bind, "udp://0.0.0.0:50376")
        self.assertTrue(cfg.servo_command.acquire_lease)
        self.assertEqual(cfg.spacemouse_cartesian_dual.max_linear_velocity_m_s, 0.2)
        self.assertEqual(cfg.spacemouse_cartesian_dual.max_angular_velocity_rad_s, 0.4)
        self.assertEqual(cfg.spacemouse_cartesian_dual.sample_hold_timeout_sec, 0.05)
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
        self.assertIn("enable_benchmark_primitives: false", text)
        self.assertIn("enforce_lease: true", text)
        self.assertIn("provider: null", text)
        self.assertIn("enable: false", text)

    def test_rbpodo_pgmode_spacemouse_tool_exists(self):
        root = Path(__file__).resolve().parents[2]
        tool = root / "tools" / "rbpodo_pgmode_spacemouse.sh"
        text = tool.read_text()

        self.assertTrue(tool.exists())
        self.assertIn("RB_ALLOW_RBPODO_ASYNC_STREAMING", text)
        self.assertIn("server-dry-run", text)
        self.assertIn("policy-dry-run", text)
        self.assertIn("check", text)
        self.assertIn("127.0.0.1:50256", text)
        self.assertIn("0.0.0.0:50376", text)
        self.assertIn("127.0.0.1:50366", text)
        self.assertIn("mock_script: pgmode_spacemouse_smoke", text)

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
            "joint_velocity",
            "master_arm_joint",
            "spacemouse_joint_velocity",
            "tcp_delta",
            "spacemouse_cartesian",
            "dual_spacemouse_cartesian",
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

    def test_command_sender_emits_rb_servo_compatible_joint_velocity_packet(self):
        fake_socket = FakeSendSocket()
        client = ServoCommandClient(
            "udp://127.0.0.1:50010",
            socket_factory=lambda *_args: fake_socket,
        )
        try:
            seq = client.send(CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[-1, 0, 0, 0, 0, 0]))
            data, address = fake_socket.sent[0]
            packet = json.loads(data.decode("utf-8"))
            self.assertEqual(seq, 1)
            self.assertEqual(address, ("127.0.0.1", 50010))
            self.assertEqual(packet["seq"], 1)
            self.assertEqual(packet["mode"], "Hold")
            self.assertEqual(packet["source_id"], "policy_runner")
            self.assertTrue(packet["session_id"])
            self.assertEqual(packet["timeout_sec"], 0.2)
            self.assertEqual(packet["left"]["mode"], "JointVelocity")
            self.assertEqual(packet["left"]["dq_target_deg_s"], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.assertEqual(packet["right"]["mode"], "JointVelocity")
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
            CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[0, 1, 0, 0, 0, 0]),
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
        velocity = client.build_packet(cases[-1], 7)
        self.assertEqual(velocity["right"]["mode"], "JointVelocity")
        self.assertEqual(velocity["right"]["dq_target_deg_s"], [0.0, 1.0, 0.0, 0.0, 0.0, 0.0])

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
        velocity = JointVelocityActionSource((1, 0, 0, 0, 0, 0))
        self.assertTrue(sine.requirements.simulation_only)
        self.assertTrue(velocity.requirements.simulation_only)

    def test_real_mode_blocks_motion_without_explicit_allow(self):
        gate = SafetyGate("real", SafetyConfig(allow_real_motion=False), stale_timeout_sec=0.5)
        intent = CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        decision = gate.evaluate(sample_state(), intent, now_monotonic=time.monotonic())
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "real_motion_not_allowed")

    def test_observed_real_mode_blocks_motion_without_explicit_allow(self):
        gate = SafetyGate("simulation", SafetyConfig(allow_real_motion=False), stale_timeout_sec=0.5)
        intent = CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        decision = gate.evaluate(sample_state(run_mode="real"), intent, now_monotonic=time.monotonic())
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "real_motion_not_allowed")

    def test_safety_blocks_stale_fault_and_invalid_state(self):
        gate = SafetyGate("simulation", SafetyConfig(), stale_timeout_sec=0.01)
        intent = CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        stale = StateSnapshot(sample_state().payload, time.monotonic() - 1.0)
        self.assertEqual(gate.evaluate(stale, intent).reason, "state_stream_stale")
        self.assertEqual(gate.evaluate(sample_state(fault_latched=True), intent).reason, "fault_latched")
        invalid = sample_state(left={"has_valid_joint_state": False, "q_actual_deg": [0, 0, 0, 0, 0, 0]})
        self.assertEqual(gate.evaluate(invalid, intent).reason, "invalid_joint_state")

    def test_safety_blocks_missing_camera_or_kinematics_requirements(self):
        intent = CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
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
        self.assertEqual([intent.mode for intent in command_client.sent], ["ArmMotion", "TcpDeltaStand"])

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
        source = JointVelocityActionSource((1, 0, 0, 0, 0, 0))

        result = run(
            cfg,
            state_client=FakeStateClient([sample_state()]),
            command_client=command_client,
            source=source,
            monotonic_fn=lambda: time.monotonic(),
            sleep_fn=lambda _period: (_ for _ in ()).throw(KeyboardInterrupt()),
        )

        self.assertEqual(result, 0)
        self.assertEqual([intent.mode for intent in command_client.sent], ["AcquireLease", "ArmMotion", "Hold"])
        self.assertEqual(len(command_client.acquire_calls), 1)
        self.assertEqual(command_client.acquire_calls[0][1]["timeout_sec"], 0.1)

    def test_run_returns_lease_timeout_when_readback_missing(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "servo_command": {"acquire_lease": True, "lease_readback_timeout_sec": 0.1},
                "geometry": {"path": ""},
                "runtime": {"startup_timeout_sec": 0.1},
            }
        )
        state_client = FakeStateClient([sample_state()])
        command_client = ServoCommandClient(
            "udp://127.0.0.1:50010",
            socket_factory=lambda *_args: FakeSendSocket(),
            source_id="policy_runner",
            session_id="session-1",
        )
        source = CloseableSource()
        stderr = StringIO()

        result = run(
            cfg,
            state_client=state_client,
            command_client=command_client,
            source=source,
            monotonic_fn=iter([0.0, 0.0, 0.0, 0.2]).__next__,
            sleep_fn=lambda _period: None,
            stderr=stderr,
        )

        self.assertEqual(result, LEASE_READBACK_TIMEOUT_EXIT_CODE)
        self.assertIn("lease_readback_timeout", stderr.getvalue())

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
        self.assertEqual(command_client.sent, [])

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
