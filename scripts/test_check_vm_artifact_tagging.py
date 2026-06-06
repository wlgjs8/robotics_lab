#!/usr/bin/env python3
"""Tests for VM parity artifact tagging guardrails."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import check_vm_artifact_tagging as tagging


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/check_vm_artifact_tagging.py"


def test_args(root: Path) -> argparse.Namespace:
    return argparse.Namespace(
        root=root,
        vm_root=Path("artifacts/vm_parity"),
        physical_root=None,
        strict_all_vm_json=False,
        json=False,
    )


class VmArtifactTaggingTest(unittest.TestCase):
    def test_help_works(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(SCRIPT), "--help"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self.assertIn("--vm-root", completed.stdout)
        self.assertIn("--physical-root", completed.stdout)

    def test_valid_vm_manifest_passes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = root / "artifacts/vm_parity/WU-01/ova_verify.json"
            manifest.parent.mkdir(parents=True)
            manifest.write_text(
                json.dumps({"source": tagging.VM_SOURCE, "physical_motion": False}),
                encoding="utf-8",
            )
            result = tagging.check(test_args(root))
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["vm_json_checked"], 1)

    def test_missing_vm_tags_fail(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manifest = root / "artifacts/vm_parity/WU-02/parity_report.json"
            manifest.parent.mkdir(parents=True)
            manifest.write_text(json.dumps({"source": "unknown"}), encoding="utf-8")
            result = tagging.check(test_args(root))
        self.assertEqual(result["status"], "FAIL")
        self.assertGreaterEqual(len(result["errors"]), 1)

    def test_non_manifest_vm_json_is_skipped_by_default(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            dump = root / "artifacts/vm_parity/WU-02/state_dump_left.json"
            dump.parent.mkdir(parents=True)
            dump.write_text(json.dumps({"raw": True}), encoding="utf-8")
            result = tagging.check(test_args(root))
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["vm_json_checked"], 0)
        self.assertEqual(result["vm_json_skipped"], 1)

    def test_physical_artifact_vm_source_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            physical = root / "artifacts/circle_tracking/run/summary.json"
            physical.parent.mkdir(parents=True)
            physical.write_text(
                json.dumps({"source": tagging.VM_SOURCE, "physical_motion": False}),
                encoding="utf-8",
            )
            result = tagging.check(test_args(root))
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("physical artifact contains source", result["errors"][0])

    def test_physical_artifact_vm_reference_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            physical = root / "artifacts/physical_acceptance/run/summary.json"
            physical.parent.mkdir(parents=True)
            physical.write_text(
                json.dumps({"input": "artifacts/vm_parity/WU-05/circle_run_left.json"}),
                encoding="utf-8",
            )
            result = tagging.check(test_args(root))
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("references artifacts/vm_parity", result["errors"][0])


if __name__ == "__main__":
    unittest.main()
