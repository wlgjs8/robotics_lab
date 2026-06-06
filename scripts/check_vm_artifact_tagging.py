#!/usr/bin/env python3
"""Guard VM parity artifacts from being mistaken for physical evidence."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Iterable


VM_SOURCE = "controller_simulation_vm"
MANIFEST_NAME_TOKENS = (
    "manifest",
    "summary",
    "report",
    "ova_verify",
    "reachability",
    "timing",
    "parity",
    "availability",
)
DEFAULT_PHYSICAL_ROOTS = (
    Path("artifacts/circle_tracking"),
    Path("artifacts/rbpodo_physical_transition"),
    Path("artifacts/physical_acceptance"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check that artifacts/vm_parity manifests are tagged as controller "
            "simulation, and that physical acceptance artifacts do not reference "
            "VM-controller evidence."
        )
    )
    parser.add_argument("--root", type=Path, default=Path("."), help="Repository root.")
    parser.add_argument(
        "--vm-root",
        type=Path,
        default=Path("artifacts/vm_parity"),
        help="VM artifact root relative to --root unless absolute.",
    )
    parser.add_argument(
        "--physical-root",
        type=Path,
        action="append",
        default=None,
        help="Physical artifact root to scan. May be repeated.",
    )
    parser.add_argument(
        "--strict-all-vm-json",
        action="store_true",
        help="Require tags on every VM JSON file, not only manifest-like JSON files.",
    )
    parser.add_argument("--json", action="store_true", help="Print machine-readable result.")
    return parser.parse_args()


def resolve(root: Path, path: Path) -> Path:
    return path if path.is_absolute() else root / path


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def is_manifest_like(path: Path) -> bool:
    name = path.name.lower()
    return any(token in name for token in MANIFEST_NAME_TOKENS)


def walk_json_files(root: Path) -> list[Path]:
    if not root.exists():
        return []
    return sorted(path for path in root.rglob("*.json") if path.is_file())


def contains_value(value: Any, needle: str) -> bool:
    if isinstance(value, dict):
        return any(contains_value(item, needle) for item in value.values())
    if isinstance(value, list):
        return any(contains_value(item, needle) for item in value)
    if isinstance(value, str):
        return needle in value
    return False


def vm_manifest_errors(vm_root: Path, strict_all_json: bool) -> tuple[list[str], int, int]:
    errors: list[str] = []
    checked = 0
    skipped = 0
    for path in walk_json_files(vm_root):
        if not strict_all_json and not is_manifest_like(path):
            skipped += 1
            continue
        checked += 1
        try:
            data = load_json(path)
        except (OSError, json.JSONDecodeError) as exc:
            errors.append(f"{path}: invalid JSON: {exc}")
            continue
        if not isinstance(data, dict):
            errors.append(f"{path}: VM manifest must be a JSON object")
            continue
        if data.get("source") != VM_SOURCE:
            errors.append(f"{path}: expected source={VM_SOURCE!r}")
        if data.get("physical_motion") is not False:
            errors.append(f"{path}: expected physical_motion=false")
    return errors, checked, skipped


def physical_artifact_errors(roots: Iterable[Path]) -> tuple[list[str], int]:
    errors: list[str] = []
    checked = 0
    for root in roots:
        for path in walk_json_files(root):
            checked += 1
            try:
                data = load_json(path)
            except (OSError, json.JSONDecodeError):
                continue
            if contains_value(data, VM_SOURCE):
                errors.append(f"{path}: physical artifact contains source={VM_SOURCE!r}")
            if contains_value(data, "artifacts/vm_parity"):
                errors.append(f"{path}: physical artifact references artifacts/vm_parity")
    return errors, checked


def check(args: argparse.Namespace) -> dict[str, Any]:
    root = args.root.resolve()
    vm_root = resolve(root, args.vm_root)
    physical_roots = [
        resolve(root, item)
        for item in (args.physical_root if args.physical_root is not None else DEFAULT_PHYSICAL_ROOTS)
    ]

    vm_errors, vm_checked, vm_skipped = vm_manifest_errors(vm_root, args.strict_all_vm_json)
    physical_errors, physical_checked = physical_artifact_errors(physical_roots)
    errors = vm_errors + physical_errors
    return {
        "schema": "robotics_lab.vm_parity.artifact_tagging_check.v1",
        "status": "PASS" if not errors else "FAIL",
        "vm_root": str(vm_root),
        "physical_roots": [str(path) for path in physical_roots],
        "vm_json_checked": vm_checked,
        "vm_json_skipped": vm_skipped,
        "physical_json_checked": physical_checked,
        "errors": errors,
    }


def main() -> int:
    args = parse_args()
    result = check(args)
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(
            "check_vm_artifact_tagging: "
            f"{result['status']} "
            f"(vm_checked={result['vm_json_checked']}, physical_checked={result['physical_json_checked']})"
        )
        for error in result["errors"]:
            print(f"ERROR: {error}", file=sys.stderr)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
