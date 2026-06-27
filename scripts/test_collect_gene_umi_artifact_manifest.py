#!/usr/bin/env python3
"""Unit tests for the GENE/UMI artifact manifest collector."""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import collect_gene_umi_artifact_manifest as manifest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/collect_gene_umi_artifact_manifest.py"


class ArtifactManifestTest(unittest.TestCase):
    def test_help_works(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(SCRIPT), "--help"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self.assertIn("--output-json", completed.stdout)
        self.assertIn("--include-missing", completed.stdout)

    def test_manifest_includes_existing_files(self) -> None:
        body = b"schema: robotics_lab.pgmode.v1\n"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            path = root / "artifacts/rbpodo_pgmode/simulation_mode_summary.json"
            path.parent.mkdir(parents=True)
            path.write_bytes(body)

            result = manifest.build_manifest(root)
            items = result["items"]

        self.assertEqual(result["schema"], manifest.SCHEMA)
        self.assertIn(
            {
                "kind": "pgmode_transition",
                "path": "artifacts/rbpodo_pgmode/simulation_mode_summary.json",
                "exists": True,
                "sha256": hashlib.sha256(body).hexdigest(),
                "size_bytes": len(body),
                "modified_at": items[0]["modified_at"],
            },
            items,
        )

    def test_missing_files_are_represented_when_requested(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            result = manifest.build_manifest(root, include_missing=True)

        missing = [
            item
            for item in result["items"]
            if item["path"] == "outputs/flow_policy.pt"
        ]
        self.assertEqual(len(missing), 1)
        self.assertFalse(missing[0]["exists"])
        self.assertIsNone(missing[0]["sha256"])

    def test_sha256_is_deterministic(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            path = root / "outputs/rollout_summary.json"
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps({"rollout_summary": "ok"}, sort_keys=True), encoding="utf-8")

            first = manifest.build_manifest(root)
            second = manifest.build_manifest(root)

        first_digest = next(
            item["sha256"] for item in first["items"] if item["path"] == "outputs/rollout_summary.json"
        )
        second_digest = next(
            item["sha256"] for item in second["items"] if item["path"] == "outputs/rollout_summary.json"
        )
        self.assertEqual(first_digest, second_digest)
        self.assertEqual(
            first_digest,
            hashlib.sha256(b'{"rollout_summary": "ok"}').hexdigest(),
        )

    def test_cli_writes_json_and_markdown(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            artifact = root / "outputs/rollout_summary.json"
            artifact.parent.mkdir(parents=True)
            artifact.write_text("{}", encoding="utf-8")
            output_json = root / "manifest.json"
            output_md = root / "manifest.md"

            subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--root",
                    str(root),
                    "--output-json",
                    str(output_json),
                    "--output-md",
                    str(output_md),
                    "--include-missing",
                ],
                check=True,
            )

            loaded = json.loads(output_json.read_text(encoding="utf-8"))
            rendered = output_md.read_text(encoding="utf-8")

        self.assertEqual(loaded["schema"], manifest.SCHEMA)
        self.assertIn("rollout_summary", rendered)
        self.assertIn("MISSING", rendered)


if __name__ == "__main__":
    unittest.main()
