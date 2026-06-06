#!/usr/bin/env python3
"""Check tracked rbpodo real configs for the supported raw joint range policy."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import re
import tempfile
import textwrap


EXPECTED_MIN = [-360.0, -360.0, -360.0, -360.0, -360.0, -360.0]
EXPECTED_MAX = [360.0, 360.0, 360.0, 360.0, 360.0, 360.0]
GLOBAL_180_MIN = [-180.0, -180.0, -180.0, -180.0, -180.0, -180.0]
GLOBAL_180_MAX = [180.0, 180.0, 180.0, 180.0, 180.0, 180.0]


def _parse_array(text: str, key: str) -> list[float] | None:
    match = re.search(rf"(?m)^\s*{re.escape(key)}:\s*(\[[^\n]+\])\s*$", text)
    if not match:
        return None
    value = ast.literal_eval(match.group(1))
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{key} must be a six-element array")
    return [float(item) for item in value]


def _matches(actual: list[float] | None, expected: list[float]) -> bool:
    return actual is not None and all(abs(a - b) < 1e-9 for a, b in zip(actual, expected))


def check_file(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    if "backend_type: rbpodo" not in text:
        return []
    q_min = _parse_array(text, "q_min_deg")
    q_max = _parse_array(text, "q_max_deg")
    errors: list[str] = []
    if q_min is None or q_max is None:
        errors.append(f"{path}: rbpodo real config must explicitly set q_min_deg and q_max_deg")
        return errors
    if _matches(q_min, GLOBAL_180_MIN) and _matches(q_max, GLOBAL_180_MAX):
        errors.append(f"{path}: global [-180, 180] joint safety range is not a supported rbpodo default")
    if not _matches(q_min, EXPECTED_MIN) or not _matches(q_max, EXPECTED_MAX):
        errors.append(
            f"{path}: expected explicit rbpodo raw controller joint range "
            "q_min_deg [-360]*6 and q_max_deg [360]*6"
        )
    return errors


def tracked_real_configs(repo_root: Path) -> list[Path]:
    config_dir = repo_root / "rb_servo_server" / "config"
    return sorted(
        path
        for path in config_dir.glob("dual_real*.yaml")
        if path.is_file() and "local" not in path.parts
    )


def run(repo_root: Path) -> int:
    errors: list[str] = []
    for path in tracked_real_configs(repo_root):
        errors.extend(check_file(path))
    if errors:
        for error in errors:
            print(error)
        return 1
    return 0


def self_test() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        config_dir = root / "rb_servo_server" / "config"
        config_dir.mkdir(parents=True)
        good = config_dir / "dual_real_good.example.yaml"
        good.write_text(
            textwrap.dedent(
                """
                left_robot:
                  backend_type: rbpodo
                safety:
                  q_min_deg: [-360, -360, -360, -360, -360, -360]
                  q_max_deg: [360, 360, 360, 360, 360, 360]
                """
            ).strip()
            + "\n",
            encoding="utf-8",
        )
        if check_file(good):
            return 1
        bad = config_dir / "dual_real_bad.example.yaml"
        bad.write_text(
            textwrap.dedent(
                """
                left_robot:
                  backend_type: rbpodo
                safety:
                  q_min_deg: [-180, -180, -180, -180, -180, -180]
                  q_max_deg: [180, 180, 180, 180, 180, 180]
                """
            ).strip()
            + "\n",
            encoding="utf-8",
        )
        if not check_file(bad):
            return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    return run(args.repo_root.resolve())


if __name__ == "__main__":
    raise SystemExit(main())
