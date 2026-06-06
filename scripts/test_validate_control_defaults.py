#!/usr/bin/env python3
"""Unit tests for control-default registry validation."""

from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml  # type: ignore[import-not-found]

import validate_control_defaults as validator


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULTS_REL = Path("configs/control_defaults/gene_26_5_ackon500_controller_sim.yaml")
MATRIX_REL = Path("configs/rbpodo_circle_ablation/ackon500_gene_goal_best.yaml")
SERVER_CONFIG_REL = Path("rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz_goal.example.yaml")


def load_yaml(path: Path) -> dict[str, object]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise AssertionError(f"expected mapping in {path}")
    return value


def write_yaml(path: Path, value: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


class DefaultsFixture:
    def __enter__(self) -> "DefaultsFixture":
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        for rel in (DEFAULTS_REL, MATRIX_REL, SERVER_CONFIG_REL):
            target = self.root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(REPO_ROOT / rel, target)
        return self

    def __exit__(self, *_exc: object) -> None:
        self.tmp.cleanup()

    @property
    def defaults_path(self) -> Path:
        return self.root / DEFAULTS_REL

    @property
    def matrix_path(self) -> Path:
        return self.root / MATRIX_REL

    @property
    def server_config_path(self) -> Path:
        return self.root / SERVER_CONFIG_REL


class ControlDefaultsValidationTest(unittest.TestCase):
    def test_validate_control_defaults_accepts_tracked_best(self) -> None:
        report = validator.validate_defaults(REPO_ROOT / DEFAULTS_REL, root=REPO_ROOT)
        self.assertEqual(report.profile_name, validator.CONTROLLER_SIM_PROFILE)
        self.assertEqual(report.parameters["servo_rate_hz"], 500)
        self.assertEqual(report.parameters["async_mode"], "sdk_ack_worker")
        self.assertEqual(report.physical_real_status, "not_promoted")

    def test_validate_control_defaults_rejects_allow_in_real_true(self) -> None:
        with DefaultsFixture() as fixture:
            config = load_yaml(fixture.server_config_path)
            cartesian = config["cartesian_control"]
            self.assertIsInstance(cartesian, dict)
            cartesian["allow_in_real"] = True
            write_yaml(fixture.server_config_path, config)
            with self.assertRaisesRegex(validator.ValidationError, "allow_in_real"):
                validator.validate_defaults(fixture.defaults_path, root=fixture.root)

    def test_validate_control_defaults_rejects_operation_mode_real_for_controller_sim_profile(self) -> None:
        with DefaultsFixture() as fixture:
            config = load_yaml(fixture.server_config_path)
            left_robot = config["left_robot"]
            self.assertIsInstance(left_robot, dict)
            left_robot["operation_mode"] = "real"
            write_yaml(fixture.server_config_path, config)
            with self.assertRaisesRegex(validator.ValidationError, "operation_mode"):
                validator.validate_defaults(fixture.defaults_path, root=fixture.root)

    def test_validate_control_defaults_rejects_matrix_operation_mode_real_for_controller_sim_profile(self) -> None:
        with DefaultsFixture() as fixture:
            matrix = load_yaml(fixture.matrix_path)
            experiments = matrix["experiments"]
            self.assertIsInstance(experiments, list)
            first = experiments[0]
            self.assertIsInstance(first, dict)
            overrides = first["config_overrides"]
            self.assertIsInstance(overrides, dict)
            overrides["left_robot.operation_mode"] = "real"
            write_yaml(fixture.matrix_path, matrix)
            with self.assertRaisesRegex(validator.ValidationError, "operation_mode"):
                validator.validate_defaults(fixture.defaults_path, root=fixture.root)

    def test_validate_control_defaults_rejects_ack_off_for_ackon500_default(self) -> None:
        with DefaultsFixture() as fixture:
            config = load_yaml(fixture.server_config_path)
            right_robot = config["right_robot"]
            self.assertIsInstance(right_robot, dict)
            right_robot["disable_waiting_ack"] = True
            write_yaml(fixture.server_config_path, config)
            with self.assertRaisesRegex(validator.ValidationError, "disable_waiting_ack"):
                validator.validate_defaults(fixture.defaults_path, root=fixture.root)

    def test_validate_control_defaults_reports_physical_real_blocked(self) -> None:
        with DefaultsFixture() as fixture:
            report_path = fixture.root / "artifacts/control_defaults/gene_26_5_defaults_report.md"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "scripts/validate_control_defaults.py"),
                    "--defaults",
                    str(fixture.defaults_path),
                    "--write-report",
                    str(report_path),
                ],
                cwd=fixture.root,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.assertIn("physical real status=not_promoted", completed.stdout)
            report = report_path.read_text(encoding="utf-8")
            self.assertIn(validator.PHYSICAL_WARNING, report)
            self.assertIn("Physical real remains blocked", report)
            self.assertIn("physical_real_conservative_seed", report)


if __name__ == "__main__":
    unittest.main()
