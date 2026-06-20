#!/usr/bin/env python3
"""Collect GENE/UMI policy-transition artifact metadata."""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCHEMA = "robotics_lab.gene_umi.artifact_manifest.v1"


@dataclass(frozen=True)
class ArtifactSpec:
    kind: str
    fixed_paths: tuple[str, ...]
    glob_patterns: tuple[str, ...] = ()


ARTIFACT_SPECS: tuple[ArtifactSpec, ...] = (
    ArtifactSpec(
        kind="hdf5_audit",
        fixed_paths=(
            "data/umi_episodes/audit.json",
            "data/umi_episodes/audit.md",
            "outputs/hdf5_audit.json",
            "outputs/hdf5_audit.md",
        ),
        glob_patterns=(
            "artifacts/**/*hdf5*audit*.json",
            "artifacts/**/*hdf5*audit*.md",
            "data/**/*audit.json",
            "data/**/*audit.md",
            "outputs/**/*audit.json",
            "outputs/**/*audit.md",
        ),
    ),
    ArtifactSpec(
        kind="flow_training",
        fixed_paths=(
            "outputs/flow_policy.pt",
            "outputs/flow_eval_summary.json",
            "outputs/flow_eval_report.md",
        ),
        glob_patterns=(
            "outputs/**/*flow*policy*.pt",
            "outputs/**/*flow_eval_summary.json",
            "outputs/**/*flow_eval_report.md",
            "artifacts/**/*flow_eval_summary.json",
            "artifacts/**/*flow_eval_report.md",
        ),
    ),
    ArtifactSpec(
        kind="rollout_summary",
        fixed_paths=(
            "outputs/rollout_summary.json",
        ),
        glob_patterns=(
            "outputs/**/rollout_summary.json",
            "artifacts/**/rollout_summary.json",
        ),
    ),
    ArtifactSpec(
        kind="pgmode_transition",
        fixed_paths=(
            "artifacts/rbpodo_physical_transition/transition_report.json",
            "artifacts/rbpodo_physical_transition/transition_report.md",
            "artifacts/rbpodo_pgmode/simulation_mode_summary.json",
        ),
        glob_patterns=(
            "artifacts/rbpodo_physical_transition/**/*.json",
            "artifacts/rbpodo_physical_transition/**/*.md",
            "artifacts/rbpodo_pgmode/**/*.json",
            "artifacts/rbpodo_pgmode/**/*.md",
        ),
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Collect a JSON/Markdown artifact_manifest for UMI HDF5 audit, "
            "flow training, rollout, and pgmode transition evidence."
        )
    )
    parser.add_argument("--output-json", type=Path, help="Path to write manifest JSON.")
    parser.add_argument("--output-md", type=Path, help="Path to write grouped Markdown.")
    parser.add_argument("--root", type=Path, default=Path("."), help="Repository root to scan.")
    parser.add_argument(
        "--include-missing",
        action="store_true",
        help="Include expected fixed artifact paths even when they are absent.",
    )
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def relative_posix(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def existing_glob_matches(root: Path, patterns: tuple[str, ...]) -> list[Path]:
    matches: list[Path] = []
    for pattern in patterns:
        for path in root.glob(pattern):
            if path.is_file():
                matches.append(path)
    return matches


def build_item(kind: str, rel_path: str, root: Path) -> dict[str, Any]:
    path = root / rel_path
    exists = path.is_file()
    item: dict[str, Any] = {
        "kind": kind,
        "path": rel_path,
        "exists": exists,
        "sha256": sha256_file(path) if exists else None,
    }
    if exists:
        stat = path.stat()
        item["size_bytes"] = stat.st_size
        item["modified_at"] = (
            datetime.fromtimestamp(stat.st_mtime, timezone.utc)
            .replace(microsecond=0)
            .isoformat()
            .replace("+00:00", "Z")
        )
    return item


def collect_items(root: Path, *, include_missing: bool = False) -> list[dict[str, Any]]:
    root = root.resolve()
    items: list[dict[str, Any]] = []
    seen: set[tuple[str, str]] = set()

    for spec in ARTIFACT_SPECS:
        candidates: list[str] = []
        for fixed in spec.fixed_paths:
            if include_missing or (root / fixed).is_file():
                candidates.append(fixed)
        for path in existing_glob_matches(root, spec.glob_patterns):
            candidates.append(relative_posix(path, root))

        for rel_path in sorted(set(candidates)):
            key = (spec.kind, rel_path)
            if key in seen:
                continue
            seen.add(key)
            items.append(build_item(spec.kind, rel_path, root))

    return sorted(items, key=lambda item: (str(item["kind"]), str(item["path"])))


def build_manifest(root: Path, *, include_missing: bool = False) -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "generated_at": utc_now_iso(),
        "items": collect_items(root, include_missing=include_missing),
    }


def render_markdown(manifest: dict[str, Any]) -> str:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for item in manifest["items"]:
        grouped.setdefault(str(item["kind"]), []).append(item)

    lines = [
        "# GENE UMI Artifact Manifest",
        "",
        f"Schema: `{manifest['schema']}`",
        f"Generated at: `{manifest['generated_at']}`",
        "",
    ]
    if not grouped:
        lines.extend(["No artifacts found.", ""])
        return "\n".join(lines)

    for kind in sorted(grouped):
        lines.extend([f"## {kind}", ""])
        for item in grouped[kind]:
            status = "present" if item["exists"] else "MISSING"
            sha = item["sha256"] if item["sha256"] is not None else "none"
            size = f", {item['size_bytes']} bytes" if item["exists"] else ""
            lines.append(f"- **{status}** `{item['path']}` - sha256 `{sha}`{size}")
        lines.append("")
    return "\n".join(lines)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def main() -> int:
    args = parse_args()
    manifest = build_manifest(args.root, include_missing=args.include_missing)

    if args.output_json:
        write_text(args.output_json, json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    if args.output_md:
        write_text(args.output_md, render_markdown(manifest))
    if not args.output_json and not args.output_md:
        print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
