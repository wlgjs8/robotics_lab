from __future__ import annotations

import json
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

from rb_servo_gui.cog_calibration import CogPoseMeasurement, CogSample
from rb_servo_gui.cog_gui import (
    CogGuiSession,
    CogIdentificationConfig,
    resolve_cog_waypoints,
)
from rb_servo_gui.app import _set_cog_conflict_controls
from rb_servo_gui.command_client import CommandClient
from rb_servo_gui.models import StateSnapshot
from rb_servo_gui.safety import OperatorSafety
from rb_servo_gui.state_receiver import StateStore


def _config_mapping() -> dict[str, object]:
    return {
        "enable": True,
        "observation_model": "rigid_payload",
        "wrench_convention": "sensor_reaction",
        "min_poses": 5,
        "arrival_tolerance_deg": 0.5,
        "settle_sec": 0.0,
        "samples_per_pose": 3,
        "max_force_stddev_n": 0.75,
        "max_torque_stddev_nm": 0.15,
        "max_force_fit_rms_n": 1.5,
        "max_torque_fit_rms_nm": 0.3,
        "max_design_condition_number": 1000.0,
    }


def _state(
    *,
    source_id: str | None = None,
    session_id: str | None = None,
    freshness: int = 1,
    profile: str = "direct",
    inhibit: bool = False,
    safety_verdict: str = "Ok",
    q_actual: tuple[float, ...] = (0.0, -30.0, 80.0, 0.0, 60.0, 0.0),
) -> dict[str, object]:
    ft = {
        "enabled": True,
        "auto_tare_enabled": True,
        "healthy": True,
        "stale": False,
        "freshness_value": freshness,
        "freshness_advanced": True,
        "raw_sensor_wrench": [1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
        "t_tcp_sensor": [0.0, 0.0, -0.202642, 0.0, 0.0, 1.5707963267948966],
        "wrench_tcp": [1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
        "gravity_tcp": [0.0, 0.0, -9.81],
        "gravity_compensation_model": "rigid_payload",
        "gravity_compensation_calibration_id": "",
        "modeled_gravity_wrench": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "payload_identification_inhibit": inhibit,
        "joint_target_profile": profile,
    }
    arm = {
        "mode": "Hold",
        "q_actual_deg": list(q_actual),
        "q_sent_deg": list(q_actual),
        "q_previous_sent_deg": list(q_actual),
        "has_valid_joint_state": True,
        "connection_state": "Connected",
        "send_ok": True,
        "error_code": 0,
        "tcp_actual_stand": [0.3, -0.1, 0.45, 0.0, 0.0, 0.0],
        "tcp_actual_valid": True,
        "force_torque": ft,
    }
    command_source: dict[str, object] = {"active": source_id is not None}
    if source_id is not None and session_id is not None:
        command_source.update(
            {
                "source_id": source_id,
                "session_id": session_id,
                "active_source_id": source_id,
                "active_session_id": session_id,
            }
        )
    return {
        "schema_version": 1,
        "tick": freshness,
        "left": dict(arm),
        "right": dict(arm),
        "motion_state": "ConnectedHold",
        "safety_verdict": safety_verdict,
        "fault_latched": False,
        "fault_reason": "",
        "logger_health": {},
        "mounts": {},
        "command_source": command_source,
        "force_torque": {"payload_identification": _config_mapping()},
    }


class _RecordingClient(CommandClient):
    def __init__(self) -> None:
        super().__init__("127.0.0.1", 9, source_id="rb_gui", session_id="cog-test")
        self.lease_packets: list[dict[str, object]] = []

    def send(self, packet):
        self.sent_packets.append(dict(packet))

    def acquire_lease(self):
        packet = self._lease_packet("AcquireLease")
        self.hold_lease = True
        self.lease_packets.append(packet)
        return packet

    def release_lease(self):
        packet = self._lease_packet("ReleaseLease")
        self.hold_lease = False
        self.lease_packets.append(packet)
        return packet


class CogGuiContractTest(unittest.TestCase):
    def test_config_requires_explicit_observation_model(self) -> None:
        config = _config_mapping()
        del config["observation_model"]
        with self.assertRaisesRegex(ValueError, "observation_model"):
            CogIdentificationConfig.parse(config)

    def test_config_requires_explicit_wrench_convention(self) -> None:
        config = _config_mapping()
        del config["wrench_convention"]
        with self.assertRaisesRegex(ValueError, "wrench_convention"):
            CogIdentificationConfig.parse(config)

    def test_controller_compensated_model_rejects_wrench_convention(self) -> None:
        config = _config_mapping()
        config["observation_model"] = "controller_compensated_linear"
        with self.assertRaisesRegex(ValueError, "must be empty"):
            CogIdentificationConfig.parse(config)
        del config["wrench_convention"]
        parsed = CogIdentificationConfig.parse(config)
        self.assertEqual(parsed.observation_model, "controller_compensated_linear")
        self.assertIsNone(parsed.wrench_convention)

    def test_conflict_control_restore_preserves_preexisting_disabled_state(self) -> None:
        class Control:
            def __init__(self, disabled: bool) -> None:
                self.disabled = disabled

        already_blocked = Control(True)
        normally_enabled = Control(False)
        emergency_stop = Control(False)
        hold = Control(False)
        handles = {
            "waypoint_edit_controls": [already_blocked, normally_enabled],
            "lifecycle_buttons": {"EmergencyStop": emergency_stop, "Hold": hold},
        }
        _set_cog_conflict_controls(handles, True)
        self.assertTrue(already_blocked.disabled)
        self.assertTrue(normally_enabled.disabled)
        self.assertFalse(emergency_stop.disabled)
        self.assertTrue(hold.disabled)
        _set_cog_conflict_controls(handles, False)
        self.assertTrue(already_blocked.disabled)
        self.assertFalse(normally_enabled.disabled)
        # Lifecycle buttons are deliberately not force-enabled; their normal
        # safety refresh remains authoritative on the next GUI tick.
        self.assertTrue(hold.disabled)

    def test_state_parser_retains_server_cog_contract(self) -> None:
        latest = StateSnapshot.parse(_state(inhibit=True, profile="payload_identification"))
        self.assertIsNotNone(latest)
        assert latest is not None
        self.assertEqual(latest.payload_identification, _config_mapping())
        ft = latest.right.force_torque
        self.assertIsNotNone(ft)
        assert ft is not None
        self.assertEqual(ft.gravity_tcp, (0.0, 0.0, -9.81))
        self.assertEqual(ft.gravity_compensation_model, "rigid_payload")
        self.assertEqual(ft.modeled_gravity_wrench, (0.0,) * 6)
        self.assertTrue(ft.payload_identification_inhibit)
        self.assertEqual(ft.joint_target_profile, "payload_identification")

    def test_one_arm_packet_has_profile_and_other_arm_hold(self) -> None:
        client = _RecordingClient()
        packet = client.build_joint_target_arm(
            "right",
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
            joint_target_profile="payload_identification",
        )
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"], {"mode": "Hold"})
        self.assertEqual(packet["right"]["mode"], "JointTarget")
        self.assertEqual(
            packet["right"]["joint_target_profile"], "payload_identification"
        )

    def test_resolve_uses_natural_order_and_rejects_duplicates(self) -> None:
        waypoints = {
            f"joint{index}": {
                "right_q": (float(index), 0.0, 0.0, 0.0, 0.0, 0.0)
            }
            for index in (10, 2, 1, 5, 3)
        }
        resolved = resolve_cog_waypoints(
            waypoints, prefix="joint", arm="right", min_poses=5
        )
        self.assertEqual(
            [waypoint.name for waypoint in resolved],
            ["joint1", "joint2", "joint3", "joint5", "joint10"],
        )
        waypoints["joint10"]["right_q"] = waypoints["joint1"]["right_q"]
        with self.assertRaisesRegex(ValueError, "unique"):
            resolve_cog_waypoints(
                waypoints, prefix="joint", arm="right", min_poses=5
            )

    def test_start_acquires_only_lease_and_first_run_sends_motion(self) -> None:
        store = StateStore(stale_after_sec=5.0)
        self.assertTrue(store.update_from_json_bytes(json.dumps(_state()).encode()))
        client = _RecordingClient()
        safety = OperatorSafety(store, client, command_timeout_sec=0.2)
        session = CogGuiSession(safety)
        store.add_update_callback(session.on_snapshot)
        config = CogIdentificationConfig.parse(_config_mapping())
        waypoints = resolve_cog_waypoints(
            {
                f"joint{index}": {
                    "right_q": (float(index), -30.0, 80.0, 0.0, 60.0, 0.0)
                }
                for index in range(1, 6)
            },
            prefix="joint",
            arm="right",
            min_poses=5,
        )

        ok, message = session.start(arm="right", waypoints=waypoints, config=config)
        self.assertTrue(ok, message)
        self.assertEqual(client.sent_packets, [], "Start must send no motion/Hold packet")
        self.assertEqual(client.lease_packets[-1]["mode"], "AcquireLease")
        self.assertEqual(session.status().state, "waiting_lease")

        ok, message = safety.send_joint_target(
            left_q_deg=(0.0,) * 6,
            right_q_deg=(0.0,) * 6,
        )
        self.assertFalse(ok)
        self.assertIn("payload-identification session", message)
        self.assertEqual(client.sent_packets, [])

        owner_state = _state(source_id=client.source_id, session_id=client.session_id)
        self.assertTrue(store.update_from_json_bytes(json.dumps(owner_state).encode()))
        self.assertEqual(session.status().state, "armed")
        ok, message = session.run_pulse(store.latest(), stale=False)
        self.assertTrue(ok, message)
        self.assertEqual(len(client.sent_packets), 1)
        self.assertEqual(
            client.sent_packets[0]["right"]["joint_target_profile"],
            "payload_identification",
        )
        self.assertEqual(client.sent_packets[0]["left"], {"mode": "Hold"})

        self.assertTrue(session.stop()[0])
        self.assertFalse(safety.payload_identification_active)

    def test_sample_callback_requires_profile_and_deduplicates_freshness(self) -> None:
        store = StateStore(stale_after_sec=5.0)
        self.assertTrue(store.update_from_json_bytes(json.dumps(_state()).encode()))
        client = _RecordingClient()
        safety = OperatorSafety(store, client, command_timeout_sec=1.0)
        session = CogGuiSession(safety)
        store.add_update_callback(session.on_snapshot)
        config_data = _config_mapping()
        config_data["samples_per_pose"] = 3
        config = CogIdentificationConfig.parse(config_data)
        waypoints = resolve_cog_waypoints(
            {
                f"joint{index}": {
                    "right_q": (float(index), -30.0, 80.0, 0.0, 60.0, 0.0)
                }
                for index in range(1, 6)
            },
            prefix="joint",
            arm="right",
            min_poses=5,
        )
        self.assertTrue(session.start(arm="right", waypoints=waypoints, config=config)[0])
        owner = dict(source_id=client.source_id, session_id=client.session_id)
        target = waypoints[0].q_target_deg
        store.update_from_json_bytes(json.dumps(_state(**owner, q_actual=target)).encode())
        self.assertTrue(session.run_pulse(store.latest(), stale=False)[0])

        # Arrival starts settling. Without the server profile/inhibit confirmation,
        # the next packet cannot enter sampling.
        store.update_from_json_bytes(json.dumps(_state(**owner, q_actual=target, freshness=2)).encode())
        store.update_from_json_bytes(json.dumps(_state(**owner, q_actual=target, freshness=3)).encode())
        self.assertEqual(session.status().state, "settling")
        self.assertEqual(session.status().sample_count, 0)

        confirmed = dict(profile="payload_identification", inhibit=True)
        store.update_from_json_bytes(
            json.dumps(_state(**owner, **confirmed, q_actual=target, freshness=4)).encode()
        )
        self.assertEqual(session.status().state, "sampling")
        self.assertEqual(session.status().sample_count, 1)
        store.update_from_json_bytes(
            json.dumps(_state(**owner, **confirmed, q_actual=target, freshness=4)).encode()
        )
        self.assertEqual(session.status().sample_count, 1)

    def test_deadman_expiry_pauses_and_stops_target_renewal(self) -> None:
        store = StateStore(stale_after_sec=5.0)
        self.assertTrue(store.update_from_json_bytes(json.dumps(_state()).encode()))
        client = _RecordingClient()
        safety = OperatorSafety(store, client, command_timeout_sec=0.001)
        session = CogGuiSession(safety)
        store.add_update_callback(session.on_snapshot)
        config = CogIdentificationConfig.parse(_config_mapping())
        waypoints = resolve_cog_waypoints(
            {
                f"joint{index}": {
                    "right_q": (float(index), -30.0, 80.0, 0.0, 60.0, 0.0)
                }
                for index in range(1, 6)
            },
            prefix="joint",
            arm="right",
            min_poses=5,
        )
        self.assertTrue(session.start(arm="right", waypoints=waypoints, config=config)[0])
        owner_state = _state(source_id=client.source_id, session_id=client.session_id)
        store.update_from_json_bytes(json.dumps(owner_state).encode())
        self.assertTrue(session.run_pulse(store.latest(), stale=False)[0])
        sent_count = len(client.sent_packets)
        time.sleep(0.003)
        session.watchdog(store.latest(), stale=False)
        self.assertEqual(session.status().state, "armed")
        self.assertIn("released", session.status().message)
        self.assertEqual(len(client.sent_packets), sent_count)

    def test_non_ok_server_safety_verdict_blocks_and_releases_lease(self) -> None:
        store = StateStore(stale_after_sec=5.0)
        self.assertTrue(store.update_from_json_bytes(json.dumps(_state()).encode()))
        client = _RecordingClient()
        safety = OperatorSafety(store, client, command_timeout_sec=1.0)
        session = CogGuiSession(safety)
        config = CogIdentificationConfig.parse(_config_mapping())
        waypoints = resolve_cog_waypoints(
            {
                f"joint{index}": {
                    "right_q": (float(index), -30.0, 80.0, 0.0, 60.0, 0.0)
                }
                for index in range(1, 6)
            },
            prefix="joint",
            arm="right",
            min_poses=5,
        )
        self.assertTrue(session.start(arm="right", waypoints=waypoints, config=config)[0])
        owner_state = _state(
            source_id=client.source_id,
            session_id=client.session_id,
            safety_verdict="InvalidCommand",
        )
        self.assertTrue(store.update_from_json_bytes(json.dumps(owner_state).encode()))

        ok, message = session.run_pulse(store.latest(), stale=False)
        self.assertFalse(ok)
        self.assertIn("InvalidCommand", message)
        self.assertEqual(session.status().state, "blocked")
        self.assertEqual(client.sent_packets, [])

        session.watchdog(store.latest(), stale=False)
        self.assertFalse(client.hold_lease)
        self.assertFalse(safety.payload_identification_active)

    def test_calculation_rejection_automatically_saves_diagnostic_evidence(self) -> None:
        store = StateStore(stale_after_sec=5.0)
        client = _RecordingClient()
        session = CogGuiSession(OperatorSafety(store, client, command_timeout_sec=1.0))
        config_data = _config_mapping()
        config_data["max_force_fit_rms_n"] = 0.001
        config = CogIdentificationConfig.parse(config_data)
        directions = np.asarray(
            [
                [0.0, 0.0, -1.0],
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
            ]
        )
        mass = 1.7
        cog = np.asarray([0.018, -0.012, 0.145])
        poses: list[CogPoseMeasurement] = []
        freshness = 1
        for index, direction in enumerate(directions):
            gravity = 9.80665 * direction
            payload_force = mass * gravity
            reaction_force = -payload_force + np.asarray([0.4, -0.25, 0.1])
            if index == 0:
                reaction_force = reaction_force + np.asarray([0.1, 0.0, 0.0])
            reaction_torque = -np.cross(cog, payload_force) + np.asarray(
                [0.025, -0.018, 0.011]
            )
            samples = []
            for sample_index in range(3):
                samples.append(
                    CogSample(
                        freshness_value=freshness,
                        wrench_tcp=tuple(reaction_force) + tuple(reaction_torque),
                        gravity_tcp=tuple(gravity),
                        received_monotonic=float(freshness),
                    )
                )
                freshness += 1
            poses.append(
                CogPoseMeasurement(
                    f"joint{index + 1}",
                    tuple(float(index + joint) for joint in range(6)),
                    tuple(samples),
                )
            )
        session._config = config
        session._arm = "right"
        session._accepted = poses
        session._run_id = "right-gui-blocked"
        session._provenance = {"server_config": dict(config.raw)}
        session._state = "complete"

        with tempfile.TemporaryDirectory() as directory:
            ok, message = session.calculate(directory)

            self.assertFalse(ok)
            self.assertIn("force_fit_rms_exceeded", message)
            status = session.status()
            self.assertEqual(status.state, "calculation_blocked")
            self.assertIsNone(status.result)
            self.assertIsNotNone(status.saved_report)
            assert status.saved_report is not None
            report = json.loads(Path(status.saved_report).read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "BLOCKED / NOT APPLIED")
            self.assertEqual(report["failure"]["code"], "force_fit_rms_exceeded")
            self.assertEqual(report["candidate_estimate"]["wrench_convention"], "sensor_reaction")

            ok, repeated_message = session.calculate(directory)
            self.assertFalse(ok)
            self.assertIn("diagnostic evidence already saved", repeated_message)
            self.assertNotIn("save failed", repeated_message)

    def test_controller_compensated_calculation_returns_residual_not_cog(self) -> None:
        mapping = _config_mapping()
        mapping["observation_model"] = "controller_compensated_linear"
        del mapping["wrench_convention"]
        config = CogIdentificationConfig.parse(mapping)
        gravity_vectors = np.asarray(
            [
                [0.0, 0.0, -9.80665],
                [0.0, -9.80665, 0.0],
                [-9.80665, 0.0, 0.0],
                [0.0, 9.80665, 0.0],
                [9.80665, 0.0, 0.0],
                [5.662, 5.662, -5.662],
                [-5.662, 5.662, -5.662],
            ]
        )
        force_matrix = np.asarray(
            [[-0.7, 1.0, 0.0], [-0.9, -0.8, -0.04], [-0.2, 0.0, 0.25]]
        )
        torque_matrix = np.asarray(
            [[-0.22, -0.14, -0.01], [0.13, -0.23, 0.0], [0.0, 0.0, 0.001]]
        )
        poses = []
        freshness = 1
        for index, gravity in enumerate(gravity_vectors):
            wrench = tuple(force_matrix @ gravity + np.asarray([-1.0, 9.7, -24.9]))
            wrench += tuple(torque_matrix @ gravity + np.asarray([2.3, 0.1, 0.03]))
            samples = []
            for _sample_index in range(3):
                samples.append(
                    CogSample(
                        freshness_value=freshness,
                        wrench_tcp=wrench,
                        gravity_tcp=tuple(gravity),
                        received_monotonic=float(freshness),
                    )
                )
                freshness += 1
            poses.append(
                CogPoseMeasurement(
                    f"joint{index + 1}",
                    tuple(float(index + joint) for joint in range(6)),
                    tuple(samples),
                )
            )

        client = _RecordingClient()
        store = StateStore(stale_after_sec=5.0)
        session = CogGuiSession(OperatorSafety(store, client))
        session._config = config
        session._arm = "right"
        session._accepted = poses
        session._run_id = "right-linear"
        session._state = "complete"

        ok, message = session.calculate()

        self.assertTrue(ok, message)
        result = session.status().result
        self.assertIsNotNone(result)
        assert result is not None
        self.assertEqual(result.observation_model, "controller_compensated_linear")
        self.assertFalse(result.physical_mass_cog_identifiable)
        self.assertLess(result.force_fit_rms_n, 1e-10)
        self.assertLess(result.torque_fit_rms_nm, 1e-10)


if __name__ == "__main__":
    unittest.main()
