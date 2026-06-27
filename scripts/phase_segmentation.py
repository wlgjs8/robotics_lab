#!/usr/bin/env python3
"""Report gripper-event phase segmentation quality for bolt pick-place HDF5 data."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
POLICY_RUNNER_ROOT = REPO_ROOT / "policy_runner"
if str(POLICY_RUNNER_ROOT) not in sys.path:
    sys.path.insert(0, str(POLICY_RUNNER_ROOT))

from policy_runner.phase_segmentation import PHASE_NAMES, analyze_dataset  # noqa: E402


def _render_report_md(report: dict[str, Any]) -> str:
    summary = report["summary"]
    lines = [
        "# Gripper-event phase segmentation report",
        "",
        f"- Episodes: {summary['episodes']}",
        f"- Clean gripper-event extraction: {summary['clean']} ({summary['clean_rate'] * 100:.1f}%)",
        f"- Quarter-split fallback: {summary['fallback']}",
        "",
        "## Phase length (frames, clean episodes only)",
        "",
        "| Phase | mean | std | min | max |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for name in PHASE_NAMES:
        stats = summary["phase_frame_stats"].get(name)
        if stats:
            lines.append(
                f"| {name} | {stats['mean']} | {stats['std']} | {stats['min']} | {stats['max']} |"
            )
    if summary["fallback_paths"]:
        lines += ["", "## Fallback episodes", ""]
        lines += [f"- {path}" for path in summary["fallback_paths"]]
    lines.append("")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze gripper-event phase segmentation.")
    parser.add_argument("--data-dir", required=True, help="Episodes root (recursively scanned).")
    parser.add_argument("--episode-paths", help="Optional split_manifest.json to restrict episodes.")
    parser.add_argument("--manifest-key", default="session_holdout_val", help="Key in split manifest.")
    parser.add_argument("--out-json", required=True)
    parser.add_argument("--out-md", required=True)
    args = parser.parse_args()

    from policy_runner.flow_dataset import FlowHdf5Dataset

    episode_paths = None
    if args.episode_paths:
        manifest = json.loads(Path(args.episode_paths).read_text(encoding="utf-8"))
        episode_paths = [entry["path"] for entry in manifest[args.manifest_key]]

    dataset = FlowHdf5Dataset(
        args.data_dir,
        action_horizon=1,
        image_size=64,
        camera_names=[],
        episode_paths=episode_paths,
    )
    report = analyze_dataset(dataset)
    Path(args.out_json).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    Path(args.out_md).write_text(_render_report_md(report), encoding="utf-8")
    print(_render_report_md(report))


if __name__ == "__main__":
    main()
