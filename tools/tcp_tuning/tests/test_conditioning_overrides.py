from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts import replay_episode_tcp_pose_target as replay
from tcp_tuning.config import Config, apply_cli_overrides, load_config


class ApplyCliOverridesTest(unittest.TestCase):
    def test_overrides_change_smoothing_and_gap_fields(self) -> None:
        cfg = apply_cli_overrides(
            Config(),
            {"smoothing_method": "lowpass", "lowpass_cutoff_hz": 4.0, "gap_median_multiplier": 5.0},
        )
        self.assertEqual(cfg.smoothing.method, "lowpass")
        self.assertEqual(cfg.smoothing.lowpass_cutoff_hz, 4.0)
        self.assertEqual(cfg.conditioning.gap_median_multiplier, 5.0)
        # untouched fields keep defaults
        self.assertEqual(cfg.smoothing.window_samples, Config().smoothing.window_samples)

    def test_none_values_are_skipped(self) -> None:
        cfg = apply_cli_overrides(Config(), {"smoothing_method": None, "smoothing_window_samples": None})
        self.assertEqual(cfg.smoothing.method, Config().smoothing.method)

    def test_unknown_override_name_fails_loudly(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown conditioning CLI override"):
            apply_cli_overrides(Config(), {"not_a_field": 3})


class LoadConfigTest(unittest.TestCase):
    def test_yaml_changes_effective_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "cond.yaml"
            path.write_text(
                "smoothing:\n  method: savgol\n  window_samples: 15\n"
                "conditioning:\n  gap_absolute_threshold_sec: 0.2\n",
                encoding="utf-8",
            )
            cfg = load_config(path)
        self.assertEqual(cfg.smoothing.window_samples, 15)
        self.assertEqual(cfg.conditioning.gap_absolute_threshold_sec, 0.2)

    def test_unknown_yaml_key_fails_loudly(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.yaml"
            path.write_text("smoothing:\n  bogus: 1\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown tcp_tuning config key"):
                load_config(path)


class PhaseConfigPrecedenceTest(unittest.TestCase):
    def _cfg(self, argv: list[str]):
        args = replay.parse_args(["--npz", "x.npz", *argv])
        return replay._phase1_config(args, rate_hz=500.0)

    def test_defaults_identical_without_config(self) -> None:
        cfg = self._cfg([])
        default = Config()
        self.assertEqual(cfg.smoothing.__dict__, default.smoothing.__dict__)
        # only servo_rate_hz is forced
        self.assertEqual(cfg.conditioning.servo_rate_hz, 500.0)
        self.assertEqual(cfg.conditioning.gap_median_multiplier, default.conditioning.gap_median_multiplier)

    def test_cli_override_beats_yaml(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "cond.yaml"
            path.write_text("smoothing:\n  method: savgol\n  window_samples: 7\n", encoding="utf-8")
            cfg = self._cfg(["--conditioning-config", str(path), "--smoothing-window-samples", "21"])
        self.assertEqual(cfg.smoothing.method, "savgol")
        self.assertEqual(cfg.smoothing.window_samples, 21)  # CLI wins over YAML's 7

    def test_smoothing_tag_reflects_method(self) -> None:
        args = replay.parse_args(["--npz", "x.npz", "--smoothing-method", "savgol",
                                  "--smoothing-window-samples", "15", "--smoothing-polyorder", "3"])
        self.assertEqual(replay._smoothing_tag(args, 500.0), "_savgol_w15p3")
        args2 = replay.parse_args(["--npz", "x.npz", "--smoothing-method", "none"])
        self.assertEqual(replay._smoothing_tag(args2, 500.0), "_smnone")


if __name__ == "__main__":
    unittest.main()
