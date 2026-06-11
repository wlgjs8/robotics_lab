from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.action_sources.dual_spacemouse_cartesian import (  # noqa: E402
    DualSpaceMouseCartesianActionSource,
)
from policy_runner.action_sources.teleop_mux import (  # noqa: E402
    OWNER_IDLE,
    OWNER_SPACEMOUSE,
    OWNER_UMI,
    TeleopMuxActionSource,
)
from policy_runner.action_sources.umi_dual_cartesian import (  # noqa: E402
    UmiDualCartesianActionSource,
    UmiSample,
)
from policy_runner.config import config_from_mapping  # noqa: E402
from policy_runner.main import make_action_source  # noqa: E402
from policy_runner.robot_state_client import StateSnapshot  # noqa: E402
from policy_runner.spacemouse import SpaceMouseSample  # noqa: E402


def sample_state(left_pose=None, right_pose=None) -> StateSnapshot:
    left_pose = left_pose or {"x": 1.0, "y": 2.0, "z": 3.0, "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
    right_pose = right_pose or {"x": -1.0, "y": 2.0, "z": 3.0, "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
    arm = {
        "has_valid_joint_state": True,
        "has_valid_tcp_pose": True,
        "q_actual_deg": [0, -30, 80, 0, 60, 0],
    }
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
        "observed_mode": "real",
        "observed_backend": "rbpodo",
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "left": {**arm, "tcp_stand": left_pose},
        "right": {**arm, "tcp_stand": right_pose},
    }
    return StateSnapshot(payload=payload, received_monotonic=0.0)


class QueueSpaceMouseReader:
    """Per-tick scripted reader: each read() pops one entry. An entry may be a
    SpaceMouseSample, None (no sample), or an Exception instance (raised)."""

    def __init__(self):
        self.entries: list = []
        self.reads = 0
        self.closed = False

    def push(self, entry) -> None:
        self.entries.append(entry)

    def read(self, timeout_sec: float | None = None):
        _ = timeout_sec
        self.reads += 1
        entry = self.entries.pop(0) if self.entries else None
        if isinstance(entry, Exception):
            raise entry
        return entry

    def close(self) -> None:
        self.closed = True


class QueueUmiReader:
    def __init__(self):
        self.entries: list = []
        self.closed = False

    def push(self, entry) -> None:
        self.entries.append(entry)

    def read(self):
        return self.entries.pop(0) if self.entries else None

    def close(self) -> None:
        self.closed = True


def sm_sample(magnitude=0.0, monotonic=0.0) -> SpaceMouseSample:
    return SpaceMouseSample(
        tx=magnitude, ty=0.0, tz=0.0, rx=0.0, ry=0.0, rz=0.0,
        buttons=(False, False),
        timestamp_monotonic=monotonic,
    )


def umi_sample(pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), *, deadman=True, monotonic=0.0) -> UmiSample:
    return UmiSample(tuple(float(v) for v in pose), 0.0, bool(deadman), float(monotonic))


IDENTITY_R_ALIGN = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


def make_mux(*, tie_break="umi", allow_controller_sim=True):
    sm_left, sm_right = QueueSpaceMouseReader(), QueueSpaceMouseReader()
    umi_left, umi_right = QueueUmiReader(), QueueUmiReader()
    spacemouse = DualSpaceMouseCartesianActionSource(
        left_reader=sm_left,
        right_reader=sm_right,
        require_deadman=False,
        startup_requires_neutral=False,
        deadband=0.08,
        activation_deadband=0.12,
        allow_rbpodo_controller_simulation=allow_controller_sim,
    )
    umi = UmiDualCartesianActionSource(
        left_reader=umi_left,
        right_reader=umi_right,
        gripper_offset=(0.0, 0.0, 0.0),
        r_align=IDENTITY_R_ALIGN,
    )
    mux = TeleopMuxActionSource(spacemouse, umi, tie_break=tie_break)
    return mux, (sm_left, sm_right), (umi_left, umi_right)


class TeleopMuxOwnershipTest(unittest.TestCase):
    def test_spacemouse_engages_first_and_suppresses_umi(self):
        mux, (sm_left, _), (umi_left, _) = make_mux()
        snapshot = sample_state()

        sm_left.push(sm_sample(0.5, monotonic=0.0))
        intent = mux.next_intent(snapshot, 0.0)
        self.assertIsNotNone(intent)
        self.assertEqual(intent.mode, "TcpTwistLocal")
        self.assertEqual(mux.owner, OWNER_SPACEMOUSE)

        # UMI deadman held while SpaceMouse owns: intent stays SpaceMouse and
        # the UMI latches are reset every tick (no stale clutch state).
        sm_left.push(sm_sample(0.5, monotonic=0.002))
        umi_left.push(umi_sample(monotonic=0.002))
        intent = mux.next_intent(snapshot, 0.002)
        self.assertEqual(intent.mode, "TcpTwistLocal")
        self.assertEqual(mux.owner, OWNER_SPACEMOUSE)
        self.assertFalse(mux.umi_source.engaged)

    def test_idle_handoff_spacemouse_to_umi_relatches_fresh(self):
        mux, (sm_left, _), (umi_left, _) = make_mux()
        snapshot = sample_state()

        sm_left.push(sm_sample(0.5, monotonic=0.0))
        mux.next_intent(snapshot, 0.0)
        self.assertEqual(mux.owner, OWNER_SPACEMOUSE)

        # Cap back to neutral: the final zero twist passes through, then idle.
        sm_left.push(sm_sample(0.0, monotonic=0.002))
        intent = mux.next_intent(snapshot, 0.002)
        self.assertEqual(intent.mode, "TcpTwistLocal")
        self.assertEqual(intent.left["tcp_twist_local"], [0.0] * 6)
        sm_left.push(sm_sample(0.0, monotonic=0.004))
        self.assertIsNone(mux.next_intent(snapshot, 0.004))
        self.assertEqual(mux.owner, OWNER_IDLE)

        # UMI engages after handoff: relative-init re-latches at the CURRENT
        # robot TCP, so the very first target equals the snapshot pose.
        umi_left.push(umi_sample(monotonic=0.006))
        intent = mux.next_intent(snapshot, 0.006)
        self.assertIsNotNone(intent)
        self.assertEqual(intent.mode, "TcpPoseTarget")
        self.assertEqual(mux.owner, OWNER_UMI)
        pose = intent.left["tcp_target_stand"]
        self.assertAlmostEqual(pose[0], 1.0)
        self.assertAlmostEqual(pose[1], 2.0)
        self.assertAlmostEqual(pose[2], 3.0)

    def test_umi_owner_suppresses_spacemouse(self):
        mux, (sm_left, _), (umi_left, _) = make_mux()
        snapshot = sample_state()

        umi_left.push(umi_sample(monotonic=0.0))
        intent = mux.next_intent(snapshot, 0.0)
        self.assertEqual(intent.mode, "TcpPoseTarget")
        self.assertEqual(mux.owner, OWNER_UMI)

        umi_left.push(umi_sample(monotonic=0.002))
        sm_left.push(sm_sample(0.5, monotonic=0.002))
        intent = mux.next_intent(snapshot, 0.002)
        self.assertEqual(intent.mode, "TcpPoseTarget")
        self.assertEqual(mux.owner, OWNER_UMI)
        self.assertFalse(mux.spacemouse_source.engaged)

    def test_umi_release_hands_back_to_idle_then_spacemouse(self):
        mux, (sm_left, _), (umi_left, _) = make_mux()
        snapshot = sample_state()

        umi_left.push(umi_sample(monotonic=0.0))
        mux.next_intent(snapshot, 0.0)
        self.assertEqual(mux.owner, OWNER_UMI)

        # Deadman released: latch-clear tick passes through, then idle.
        umi_left.push(umi_sample(monotonic=0.002, deadman=False))
        intent = mux.next_intent(snapshot, 0.002)
        self.assertIsNotNone(intent)
        umi_left.push(umi_sample(monotonic=0.004, deadman=False))
        self.assertIsNone(mux.next_intent(snapshot, 0.004))
        self.assertEqual(mux.owner, OWNER_IDLE)

        sm_left.push(sm_sample(0.5, monotonic=0.006))
        intent = mux.next_intent(snapshot, 0.006)
        self.assertEqual(intent.mode, "TcpTwistLocal")
        self.assertEqual(mux.owner, OWNER_SPACEMOUSE)

    def test_same_tick_engage_falls_to_tie_break(self):
        for tie_break, expected_owner, expected_mode in (
            ("umi", OWNER_UMI, "TcpPoseTarget"),
            ("spacemouse", OWNER_SPACEMOUSE, "TcpTwistLocal"),
        ):
            with self.subTest(tie_break=tie_break):
                mux, (sm_left, _), (umi_left, _) = make_mux(tie_break=tie_break)
                snapshot = sample_state()
                sm_left.push(sm_sample(0.5, monotonic=0.0))
                umi_left.push(umi_sample(monotonic=0.0))
                intent = mux.next_intent(snapshot, 0.0)
                self.assertEqual(intent.mode, expected_mode)
                self.assertEqual(mux.owner, expected_owner)

    def test_invalid_tie_break_rejected(self):
        with self.assertRaises(ValueError):
            make_mux(tie_break="left_arm")


class TeleopMuxSpaceMouseFailureTest(unittest.TestCase):
    def test_hid_failure_while_idle_degrades_to_umi_only(self):
        mux, (sm_left, _), (umi_left, _) = make_mux()
        snapshot = sample_state()

        sm_left.push(RuntimeError("failed to open SpaceMouse HID device"))
        self.assertIsNone(mux.next_intent(snapshot, 0.0))
        reads_after_failure = sm_left.reads

        # SpaceMouse is never polled again; UMI keeps working.
        umi_left.push(umi_sample(monotonic=0.002))
        intent = mux.next_intent(snapshot, 0.002)
        self.assertEqual(intent.mode, "TcpPoseTarget")
        self.assertEqual(mux.owner, OWNER_UMI)
        self.assertEqual(sm_left.reads, reads_after_failure)

    def test_hid_failure_while_owner_emits_one_safety_zero_twist(self):
        mux, (sm_left, _), (umi_left, _) = make_mux()
        snapshot = sample_state()

        sm_left.push(sm_sample(0.5, monotonic=0.0))
        mux.next_intent(snapshot, 0.0)
        self.assertEqual(mux.owner, OWNER_SPACEMOUSE)

        sm_left.push(OSError("device unplugged"))
        intent = mux.next_intent(snapshot, 0.002)
        self.assertIsNotNone(intent)
        self.assertEqual(intent.mode, "TcpTwistLocal")
        self.assertEqual(intent.left["tcp_twist_local"], [0.0] * 6)
        self.assertEqual(intent.right["tcp_twist_local"], [0.0] * 6)
        self.assertEqual(mux.owner, OWNER_IDLE)

        umi_left.push(umi_sample(monotonic=0.004))
        intent = mux.next_intent(snapshot, 0.004)
        self.assertEqual(intent.mode, "TcpPoseTarget")
        self.assertEqual(mux.owner, OWNER_UMI)


class TeleopMuxContractTest(unittest.TestCase):
    def test_requirements_fail_closed_on_controller_simulation(self):
        mux, _, _ = make_mux(allow_controller_sim=True)
        self.assertTrue(mux.requirements.allow_rbpodo_controller_simulation_cartesian)
        mux, _, _ = make_mux(allow_controller_sim=False)
        self.assertFalse(mux.requirements.allow_rbpodo_controller_simulation_cartesian)
        self.assertTrue(mux.requirements.cartesian_motion)
        self.assertTrue(mux.requirements.requires_valid_tcp_pose)

    def test_close_closes_both_sources(self):
        mux, (sm_left, sm_right), (umi_left, umi_right) = make_mux()
        mux.close()
        for reader in (sm_left, sm_right, umi_left, umi_right):
            self.assertTrue(reader.closed)

    def test_make_action_source_builds_mux_from_config(self):
        config = config_from_mapping(
            {
                "action_source": "teleop_mux",
                "teleop_mux": {"tie_break": "spacemouse"},
                "spacemouse_cartesian_dual": {
                    "left": {"mock_script": "pgmode_spacemouse_smoke"},
                    "right": {"mock_script": "pgmode_spacemouse_smoke"},
                },
                "umi_dual_cartesian": {
                    "left": {"mock_script": "pgmode_umi_smoke"},
                    "right": {"mock_script": "pgmode_umi_smoke"},
                },
            }
        )
        source = make_action_source(config)
        self.assertIsInstance(source, TeleopMuxActionSource)
        self.assertEqual(source.tie_break, "spacemouse")
        source.close()

    def test_config_rejects_unknown_tie_break(self):
        with self.assertRaises(ValueError):
            config_from_mapping(
                {
                    "action_source": "teleop_mux",
                    "teleop_mux": {"tie_break": "both"},
                }
            )


if __name__ == "__main__":
    unittest.main()
