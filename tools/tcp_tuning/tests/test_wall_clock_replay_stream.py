from __future__ import annotations

import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

import numpy as np

from scripts import replay_episode_tcp_pose_target as replay
from tcp_tuning.trajectory_log import TrajectoryLogReader

from tools.tcp_tuning.tests.test_real_replay_driver import write_server_config


Q = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def linear_plan(n: int = 11, rate_hz: float = 500.0, hold_mask: np.ndarray | None = None) -> replay.ReplayPlan:
    """A single-arm plan with a straight-line x ramp on the episode-time grid."""

    t_servo = np.arange(n, dtype=np.float64) / rate_hz
    x = np.linspace(0.0, 0.1, n)
    if hold_mask is not None:
        # Freeze x where held so the goal genuinely does not move.
        for i in range(1, n):
            if hold_mask[i]:
                x[i] = x[i - 1]
    goal = np.zeros((n, 7), dtype=np.float64)
    goal[:, 0] = x
    goal[:, 3:7] = Q
    twist = np.zeros((n, 6), dtype=np.float64)
    src = np.arange(n, dtype=np.int64)
    hold = np.zeros(n, dtype=bool) if hold_mask is None else np.asarray(hold_mask, dtype=bool)
    return replay.ReplayPlan(
        paths=replay.ReplayPaths(
            episode_id="ep", run_name="run", run_dir=Path("/tmp/run"),
            npz_path=Path("/tmp/x.npz"), log_path=Path("/tmp/run/log.csv"),
        ),
        selected_arms=("left",),
        t_servo=t_servo,
        goals={"left": goal},
        raw_targets={"left": goal.copy()},
        twists={"left": twist},
        grippers={"left": np.full(n, np.nan)},
        src_lo=src,
        src_hi=src,
        hold=hold,
        gap=np.zeros(n, dtype=bool),
        dropout=np.zeros(n, dtype=bool),
        current_poses={"left": goal[0].copy()},
        init_deltas={},
        init_poses={},
        stream_speed_stats={},
        bounds_notes=[],
        segment_selection={},
        would_abort_large_init=False,
        dry_run=True,
    )


class WallClockResampleTest(unittest.TestCase):
    def test_time_scale_1_produces_about_n_ticks(self) -> None:
        plan = linear_plan(n=11)
        stream = replay.build_wall_clock_replay_stream(plan, time_scale=1.0, rate_hz=500.0)
        self.assertEqual(stream.size, 11)

    def test_time_scale_2_produces_about_2n_ticks_not_n(self) -> None:
        plan = linear_plan(n=11)
        stream = replay.build_wall_clock_replay_stream(plan, time_scale=2.0, rate_hz=500.0)
        # ts=2 advances episode by half a stored step per wall tick -> ~2x ticks.
        self.assertGreaterEqual(stream.size, 20)
        self.assertLessEqual(stream.size, 22)

    def test_max_per_wall_tick_delta_halves_at_time_scale_2(self) -> None:
        plan = linear_plan(n=11)
        s1 = replay.build_wall_clock_replay_stream(plan, time_scale=1.0, rate_hz=500.0)
        s2 = replay.build_wall_clock_replay_stream(plan, time_scale=2.0, rate_hz=500.0)

        def max_delta(stream: replay.WallClockStream) -> float:
            g = stream.goals["left"]
            return float(np.max(np.linalg.norm(np.diff(g[:, :3], axis=0), axis=1)))

        self.assertAlmostEqual(max_delta(s2), 0.5 * max_delta(s1), places=6)

    def test_dispatch_is_500hz_in_both_time_scales(self) -> None:
        plan = linear_plan(n=11)
        for ts in (1.0, 2.0):
            stream = replay.build_wall_clock_replay_stream(plan, time_scale=ts, rate_hz=500.0)
            dt_wall = np.diff(stream.t_wall)
            np.testing.assert_allclose(dt_wall, 1.0 / 500.0, atol=1e-9)

    def test_no_unexpected_zoh_ticks_for_moving_trajectory(self) -> None:
        plan = linear_plan(n=11)
        stream = replay.build_wall_clock_replay_stream(plan, time_scale=2.0, rate_hz=500.0)
        # Interior ticks are interpolated -> the only ZOH/hold tick is the final endpoint.
        self.assertEqual(int(np.count_nonzero(stream.hold[:-1])), 0)
        self.assertEqual(replay._stale_repeated_count(stream, plan.selected_arms), 0)

    def test_genuine_hold_is_flagged_but_not_counted_as_stale(self) -> None:
        hold_mask = np.zeros(11, dtype=bool)
        hold_mask[4:7] = True
        plan = linear_plan(n=11, hold_mask=hold_mask)
        stream = replay.build_wall_clock_replay_stream(plan, time_scale=1.0, rate_hz=500.0)
        self.assertGreater(int(np.count_nonzero(stream.hold)), 0)
        # held ticks are excluded from the unexpected-repeat count
        self.assertEqual(replay._stale_repeated_count(stream, plan.selected_arms), 0)

    def test_resample_hits_exact_stored_samples_at_time_scale_1(self) -> None:
        plan = linear_plan(n=11)
        stream = replay.build_wall_clock_replay_stream(plan, time_scale=1.0, rate_hz=500.0)
        np.testing.assert_allclose(stream.goals["left"], plan.goals["left"], atol=1e-9)
        np.testing.assert_allclose(stream.interp_alpha, 0.0, atol=1e-9)


class WallClockDryRunLogTest(unittest.TestCase):
    def _run_dry(self, tmp: Path, extra: list[str]) -> Path:
        npz = _write_linear_npz(tmp / "episode_lin" / "clean_foh_se3_500hz.npz", n=21)
        config = write_server_config(tmp / "server.yaml", max_linear=5.0, max_angular=5.0)
        out = tmp / "out"

        class NoSocketClient:
            def __init__(self, *args, **kwargs):
                raise AssertionError("dry-run must not construct ServoCommandClient")

        with patch.object(replay, "ServoCommandClient", NoSocketClient), redirect_stdout(io.StringIO()):
            rc = replay.main([
                "--npz", str(npz), "--server-config", str(config), "--rate-hz", "500",
                "--mock-current-pose", "first", "--out-dir", str(out),
                "--max-linear-speed-m-s", "5.0", "--max-angular-speed-rad-s", "5.0", *extra,
            ])
        self.assertEqual(rc, 0)
        logs = list((out / "episode_lin" / "runs").glob("*/log.csv"))
        self.assertEqual(len(logs), 1)
        return logs[0]

    def test_log_and_meta_have_wall_clock_fields_and_2x_ticks(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            log1 = self._run_dry(root, ["--time-scale", "1.0"])
            rows1 = TrajectoryLogReader(log1).read()
            n1 = sum(1 for r in rows1 if str(r.get("arm")) == "left")

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            log2 = self._run_dry(root, ["--time-scale", "2.0"])
            rows2 = TrajectoryLogReader(log2).read()
            n2 = sum(1 for r in rows2 if str(r.get("arm")) == "left")
            for col in ("t_episode", "src_idx_lo", "src_idx_hi", "interpolation_alpha",
                        "time_scale", "time_scale_mode", "effective_command_rate_hz", "hold"):
                self.assertIn(col, rows2[0], f"missing log column {col}")
            meta = json.loads((log2.parent / "run_meta.json").read_text())
            self.assertEqual(meta["time_scale_mode"], "wall_clock_resample")
            self.assertEqual(meta["wall_clock_dispatch_rate_hz"], 500.0)
            for key in ("effective_command_rate_hz", "held_tick_count", "stale_or_repeated_target_count"):
                self.assertIn(key, meta)
            self.assertEqual(meta["stale_or_repeated_target_count"], 0)

        self.assertGreaterEqual(n2, 2 * n1 - 2)

    def test_legacy_sleep_reproduces_stored_tick_count(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            log = self._run_dry(root, ["--time-scale", "2.0", "--time-scale-mode", "legacy_sleep"])
            rows = TrajectoryLogReader(log).read()
            n = sum(1 for r in rows if str(r.get("arm")) == "left")
            self.assertEqual(n, 21)
            meta = json.loads((log.parent / "run_meta.json").read_text())
            self.assertEqual(meta["time_scale_mode"], "legacy_sleep")
            self.assertEqual(meta["tick_count"], 21)


def _write_linear_npz(path: Path, n: int = 21, rate_hz: float = 500.0) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    t = np.arange(n, dtype=np.float64) / rate_hz
    goal = np.zeros((n, 7), dtype=np.float64)
    goal[:, 0] = np.linspace(0.0, 0.05, n)
    goal[:, 3:7] = Q
    arrays = {
        "t_servo": t,
        "servo_rate_hz": np.asarray(rate_hz),
        "mode": np.asarray("clean_foh_se3"),
        "episode": np.asarray("episode_lin.hdf5"),
        "seed": np.asarray(-1),
        "segments": np.asarray([[0, n]], dtype=np.int64),
        "gaps": np.empty((0, 2), dtype=np.float64),
        "meta_json": np.asarray(json.dumps({"episode_id": "episode_lin"})),
    }
    for arm in ("left", "right"):
        arrays[f"{arm}_source_raw_target"] = goal
        arrays[f"{arm}_conditioned_goal"] = goal
        arrays[f"{arm}_conditioned_twist"] = np.zeros((n, 6), dtype=np.float64)
        arrays[f"{arm}_gripper"] = np.full(n, np.nan)
        arrays[f"{arm}_valid"] = np.ones(n, dtype=bool)
        arrays[f"{arm}_hold"] = np.zeros(n, dtype=bool)
        arrays[f"{arm}_dropout"] = np.zeros(n, dtype=bool)
        arrays[f"{arm}_gap"] = np.zeros(n, dtype=bool)
        arrays[f"{arm}_reanchor"] = np.zeros(n, dtype=bool)
        arrays[f"{arm}_src_id_lo"] = np.arange(n, dtype=np.int64)
        arrays[f"{arm}_src_id_hi"] = np.arange(n, dtype=np.int64)
    np.savez(path, **arrays)
    return path


if __name__ == "__main__":
    unittest.main()
