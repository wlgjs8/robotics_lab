from __future__ import annotations

import io
import json
import subprocess
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

import h5py
import numpy as np

from scripts import batch_replay_episodes as batch
from scripts import replay_episode_tcp_pose_target as replay


Q = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def pose(x: float, y: float = 0.0, z: float = 0.2) -> np.ndarray:
    return np.asarray([x, y, z, *Q], dtype=np.float64)


class BatchReplayEpisodesTest(unittest.TestCase):
    def test_batch_dry_run_two_episode_subset_sends_nothing_and_writes_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            episodes = root / "episodes"
            write_data_tcp(episodes / "episode_000.hdf5", offset=0.0)
            write_data_tcp(episodes / "episode_001.hdf5", offset=0.02)
            config = write_server_config(root / "server.yaml")
            out_dir = root / "out"

            stdout = io.StringIO()
            with redirect_stdout(stdout):
                rc = batch.main(
                    [
                        "--episodes-dir",
                        str(episodes),
                        "--episodes",
                        "000,001",
                        "--init-mode",
                        "rest_stow",
                        "--mock-q-actual",
                        "rest_stow",
                        "--server-config",
                        str(config),
                        "--out-dir",
                        str(out_dir),
                        "--max-linear-speed-m-s",
                        "10.0",
                        "--max-angular-speed-rad-s",
                        "10.0",
                    ]
                )

            self.assertEqual(rc, 0)
            text = stdout.getvalue()
            self.assertIn("BATCH DRY RUN — no motion sent", text)
            self.assertIn("| episode | status | segment |", text)
            self.assertIn("episode_000.hdf5", text)
            self.assertIn("episode_001.hdf5", text)
            summaries = list(out_dir.glob("batch_*/batch_summary.json"))
            self.assertEqual(len(summaries), 1)
            payload = json.loads(summaries[0].read_text(encoding="utf-8"))
            self.assertTrue(payload["dry_run"])
            self.assertFalse(payload["halted"])
            self.assertEqual(payload["episode_count_attempted"], 2)
            self.assertEqual([item["status"] for item in payload["results"]], ["ok", "ok"])

    def test_driver_failure_halts_and_does_not_attempt_later_episodes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            episodes = root / "episodes"
            for index in range(3):
                write_empty_hdf5(episodes / f"episode_{index:03d}.hdf5")
            args = batch.parse_args(
                [
                    "--episodes-dir",
                    str(episodes),
                    "--init-mode",
                    "rest_stow",
                    "--out-dir",
                    str(root / "out"),
                ]
            )
            attempted: list[str] = []

            def fake_runner(cmd: list[str]) -> subprocess.CompletedProcess[str]:
                episode = Path(cmd[cmd.index("--data-tcp") + 1]).name
                attempted.append(episode)
                code = 3 if episode == "episode_001.hdf5" else 0
                return subprocess.CompletedProcess(cmd, code, stdout="", stderr="")

            rc = batch.run_batch(args, driver_runner=fake_runner)

            self.assertEqual(rc, 1)
            self.assertEqual(attempted, ["episode_000.hdf5", "episode_001.hdf5"])

    def test_controller_sim_arm_error_flag_is_passed_to_driver_command(self) -> None:
        args = batch.parse_args(
            [
                "--episodes-dir",
                "episodes",
                "--init-mode",
                "rest_stow",
                "--allow-controller-sim-arm-error",
            ]
        )

        cmd = batch.build_driver_command(args, Path("episodes/episode_000.hdf5"), "batch_test", dry_run=True)

        self.assertIn("--allow-controller-sim-arm-error", cmd)

    def test_init_return_arrival_and_timeout_logic(self) -> None:
        server = replay.ServerRuntimeConfig(
            path=Path("server.yaml"),
            command_endpoint="udp://127.0.0.1:1",
            state_bind=None,
            servo_rate_hz=10.0,
            command_timeout_sec=0.2,
            smd_max_linear_velocity_m_s=1.0,
            smd_max_angular_velocity_rad_s=1.0,
            floor_z_min_m=None,
            roi_min_m=None,
            roi_max_m=None,
            raw={},
        )
        target = batch.JointTargets((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), (-1.0, -2.0, -3.0, -4.0, -5.0, -6.0))
        arrived_state = FakeState(target)
        arrived_client = FakeCommandClient()
        arrived = batch.drive_joint_target_until_arrived(
            arrived_state,
            arrived_client,
            server,
            target,
            tol_deg=1.0,
            timeout_sec=0.05,
            period_sec=0.001,
        )
        self.assertTrue(arrived.arrived)
        self.assertFalse(any(item == "hold" for item in arrived_client.sent))

        timeout_state = FakeState(batch.JointTargets((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)))
        timeout_client = FakeCommandClient()
        timed_out = batch.drive_joint_target_until_arrived(
            timeout_state,
            timeout_client,
            server,
            target,
            tol_deg=0.1,
            timeout_sec=0.003,
            period_sec=0.001,
        )
        self.assertFalse(timed_out.arrived)
        self.assertTrue(timed_out.timeout)
        self.assertIn("hold", timeout_client.sent)

    def test_init_return_graces_startup_lease_until_source_is_active(self) -> None:
        server = make_server_config()
        target = batch.JointTargets((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), (-1.0, -2.0, -3.0, -4.0, -5.0, -6.0))
        state = SequencedLeaseState(
            target,
            active_sources=[None, None, None, "fake"],
            active_sessions=[None, None, None, "fake-session"],
        )
        client = FakeCommandClient()

        result = batch.drive_joint_target_until_arrived(
            state,
            client,
            server,
            target,
            tol_deg=0.1,
            timeout_sec=0.1,
            period_sec=0.001,
            init_lease_grace_sec=0.05,
        )

        self.assertTrue(result.arrived)
        self.assertIsNone(result.fault)
        self.assertGreaterEqual(client.sent.count("command"), 4)
        self.assertNotIn("hold", client.sent)

    def test_init_return_faults_with_lease_lost_after_grace(self) -> None:
        server = make_server_config()
        target = batch.JointTargets((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), (-1.0, -2.0, -3.0, -4.0, -5.0, -6.0))
        state = SequencedLeaseState(
            target,
            active_sources=[None, None, None, None, None, None],
            active_sessions=[None, None, None, None, None, None],
        )
        client = FakeCommandClient()

        result = batch.drive_joint_target_until_arrived(
            state,
            client,
            server,
            target,
            tol_deg=0.1,
            timeout_sec=0.1,
            period_sec=0.001,
            init_lease_grace_sec=0.003,
        )

        self.assertFalse(result.arrived)
        self.assertEqual(result.fault, "command_source_lease_lost")
        self.assertGreater(client.sent.count("command"), 1)
        self.assertEqual(client.sent[-1], "hold")

    def test_init_return_does_not_grace_non_lease_faults(self) -> None:
        server = make_server_config()
        target = batch.JointTargets((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), (-1.0, -2.0, -3.0, -4.0, -5.0, -6.0))
        state = SequencedLeaseState(
            target,
            active_sources=[None],
            active_sessions=[None],
            extra={"fault_latched": True},
        )
        client = FakeCommandClient()

        result = batch.drive_joint_target_until_arrived(
            state,
            client,
            server,
            target,
            tol_deg=0.1,
            timeout_sec=0.1,
            period_sec=0.001,
            init_lease_grace_sec=0.05,
        )

        self.assertFalse(result.arrived)
        self.assertEqual(result.fault, "fault_latched")
        self.assertEqual(client.sent, ["command", "hold"])

    def test_init_return_arrives_when_lease_is_already_held(self) -> None:
        server = make_server_config()
        target = batch.JointTargets((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), (-1.0, -2.0, -3.0, -4.0, -5.0, -6.0))
        state = SequencedLeaseState(target, active_sources=["fake"], active_sessions=["fake-session"])
        client = FakeCommandClient()

        result = batch.drive_joint_target_until_arrived(
            state,
            client,
            server,
            target,
            tol_deg=0.1,
            timeout_sec=0.1,
            period_sec=0.001,
            init_lease_grace_sec=0.05,
        )

        self.assertTrue(result.arrived)
        self.assertIsNone(result.fault)
        self.assertEqual(client.sent, ["command"])

    def test_driver_non_interactive_still_dry_runs_without_execute(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            data_tcp = write_data_tcp(root / "episode_000.hdf5")
            config = write_server_config(root / "server.yaml")

            class NoSocketClient:
                def __init__(self, *args, **kwargs):
                    raise AssertionError("non-interactive without --execute must stay dry-run")

            stdout = io.StringIO()
            with patch.object(replay, "ServoCommandClient", NoSocketClient), redirect_stdout(stdout):
                rc = replay.main(
                    [
                        "--source",
                        "ee_local",
                        "--data-tcp",
                        str(data_tcp),
                        "--server-config",
                        str(config),
                        "--anchor",
                        "mock",
                        "--mock-current-pose",
                        "default",
                        "--non-interactive",
                        "--max-linear-speed-m-s",
                        "10.0",
                        "--max-angular-speed-rad-s",
                        "10.0",
                    ]
                )

            self.assertEqual(rc, 0)
            self.assertIn("DRY RUN — no motion sent", stdout.getvalue())


class FakeLatest:
    def __init__(self, payload):
        self.payload = payload


class FakeState:
    def __init__(self, joints: batch.JointTargets):
        self.latest = FakeLatest(
            {
                "left": {"q_actual_deg": list(joints.left)},
                "right": {"q_actual_deg": list(joints.right)},
            }
        )

    def is_latest_stale(self) -> bool:
        return False


class SequencedLeaseState:
    def __init__(
        self,
        joints: batch.JointTargets,
        *,
        active_sources: list[str | None],
        active_sessions: list[str | None],
        extra: dict | None = None,
    ):
        self._joints = joints
        self._active_sources = active_sources
        self._active_sessions = active_sessions
        self._extra = extra or {}
        self._index = 0

    @property
    def latest(self) -> FakeLatest:
        index = min(self._index, len(self._active_sources) - 1)
        session_index = min(self._index, len(self._active_sessions) - 1)
        self._index += 1
        payload = {
            "left": {"q_actual_deg": list(self._joints.left)},
            "right": {"q_actual_deg": list(self._joints.right)},
            "command_source": {
                "enforce_lease": True,
                "active_source_id": self._active_sources[index],
                "active_session_id": self._active_sessions[session_index],
            },
            **self._extra,
        }
        return FakeLatest(payload)

    def is_latest_stale(self) -> bool:
        return False


class FakeCommandClient:
    source_id = "fake"
    session_id = "fake-session"

    def __init__(self):
        self.sent: list[str] = []

    def send(self, intent):
        is_plain_hold = getattr(intent, "mode", None) == "Hold" and not getattr(intent, "left", None) and not getattr(intent, "right", None)
        self.sent.append("hold" if is_plain_hold else "command")


def make_server_config() -> replay.ServerRuntimeConfig:
    return replay.ServerRuntimeConfig(
        path=Path("server.yaml"),
        command_endpoint="udp://127.0.0.1:1",
        state_bind=None,
        servo_rate_hz=10.0,
        command_timeout_sec=0.2,
        smd_max_linear_velocity_m_s=1.0,
        smd_max_angular_velocity_rad_s=1.0,
        floor_z_min_m=None,
        roi_min_m=None,
        roi_max_m=None,
        raw={},
    )


def write_data_tcp(path: Path, *, offset: float = 0.0) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    left = np.stack([pose(offset), pose(offset + 0.01), pose(offset + 0.02)], axis=0).astype(np.float32)
    right = np.stack([pose(-offset), pose(-offset - 0.01), pose(-offset - 0.02)], axis=0).astype(np.float32)
    with h5py.File(path, "w") as handle:
        action = handle.create_group("action")
        action.create_dataset("tcp_target_stand_left", data=left)
        action.create_dataset("tcp_target_stand_right", data=right)
        action.create_dataset("gripper_left", data=np.zeros(3, dtype=np.float32))
        action.create_dataset("gripper_right", data=np.zeros(3, dtype=np.float32))
        observations = handle.create_group("observations")
        observations.create_dataset("timestamp", data=np.asarray([0.0, 0.1, 0.2], dtype=np.float64))
    return path


def write_empty_hdf5(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w"):
        pass
    return path


def write_server_config(path: Path) -> Path:
    path.write_text(
        """
servo:
  rate_hz: 10
  command_timeout_sec: 0.2
cartesian_control:
  pose_track_smd:
    max_linear_velocity_m_s: 10.0
    max_angular_velocity_rad_s: 10.0
safety:
  floor_constraint:
    enable: true
    z_min_m: 0.0
    monitor_only: false
  roi_box:
    enable: true
    min_m: [-2.0, -2.0, 0.0]
    max_m: [2.0, 2.0, 2.0]
    monitor_only: false
network:
  command_bind: "udp://127.0.0.1:50010"
  state_pub_endpoint: "udp://127.0.0.1:50110"
""",
        encoding="utf-8",
    )
    return path


if __name__ == "__main__":
    unittest.main()
