#!/usr/bin/env python3
"""Re-evaluate existing imitation checkpoints with gripper-event phase metrics.

No retraining. For each checkpoint we rebuild the validation dataset and model
from the checkpoint's own self-describing payload, then run the existing
`imitation_experiments._evaluate_*` routines twice on the identical samples:

  * baseline: the original equal-time `_sample_phase` (4 quarters)
  * gripper:  `phase_segmentation` boundaries (right/left close+open)

so per-phase MSE differences are a controlled apples-to-apples comparison.

Run inside the cuda-ml image, e.g.:

  docker run --rm --gpus '"device=0"' \
    --entrypoint python3 \
    -v "$PWD/data:/data/policy_episodes:ro" \
    -v "$PWD/outputs/flow_runs:/outputs/flow_runs" \
    -v "$PWD/policy_runner/dinov3:/app/policy_runner/dinov3:ro" \
    -v "$PWD/scripts:/scripts:ro" \
    robotics_lab/policy_runner:cuda-ml \
    /scripts/reeval_phase_metrics.py --auto \
      --split-manifest /outputs/flow_runs/imitation_official/split_manifest.json \
      --data-dir /data/policy_episodes \
      --out-dir /outputs/flow_runs/imitation_phase_reeval
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
POLICY_RUNNER_ROOT = REPO_ROOT / "policy_runner"
if POLICY_RUNNER_ROOT.exists() and str(POLICY_RUNNER_ROOT) not in sys.path:
    sys.path.insert(0, str(POLICY_RUNNER_ROOT))

from policy_runner import imitation_experiments as ie  # noqa: E402
from policy_runner.flow_dataset import FlowHdf5Dataset, FLOW_ACTION_DIM, FLOW_PROPRIO_DIM  # noqa: E402
from policy_runner import phase_segmentation as ps  # noqa: E402

# Curated best single-checkpoint runs per family (relative to flow_runs root).
DEFAULT_RUNS = [
    "imitation_dinov3_tiny_holdout_color384_h1_seed5_lr3e4",   # direct_bc best single
    "imitation_dinov3_tiny_holdout_color384_h1_seed9",         # direct_bc 2nd
    "imitation_dinov3_tiny_holdout_color384_h1_seed12_lr3e4",  # direct_bc 3rd
    "imitation_dinov3_small_holdout_color384_h1_seed5_lr3e4",  # direct_bc small backbone
    "imitation_flow_dinov3_tiny_holdout_color384_h1_seed9_lr3e4_steps4",  # flow best
    "imitation_act_dinov3_tiny_holdout_color384_h1_seed9_lr3e4",          # act best
    "imitation_structured_dinov3_tiny_holdout_color384_h1_seed1",         # structured best
    "imitation_diffusion_dinov3_tiny_holdout_color384_h1",                # diffusion best
    "imitation_baselines_holdout_color384_h1",                            # state_mlp / constants
]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def _run_meta(run_dir: Path) -> dict[str, Any]:
    """image_size / image_crop / per-model args from a run's saved artifacts."""
    meta: dict[str, Any] = {"image_size": 224, "image_crop": "none", "flow_sample_steps": 4, "diffusion_sample_steps": 16}
    for name in ("train_dataset_stats.json", "leaderboard_summary.json"):
        p = run_dir / name
        if not p.exists():
            continue
        blob = _load_json(p)
        args = blob.get("args", blob)
        for key in ("image_size", "image_crop", "flow_sample_steps", "diffusion_sample_steps"):
            if isinstance(args, dict) and key in args and args[key] is not None:
                meta[key] = args[key]
    return meta


def _build_model(payload: dict[str, Any], hidden_dim: int, camera_count: int, device, torch):
    family = payload["model_family"]
    backbone = payload.get("backbone")
    horizon = int(payload["action_horizon"]) if "action_horizon" in payload else None
    if family == "state_only_mlp":
        model = ie._build_state_mlp(action_horizon=horizon, hidden_dim=hidden_dim)
    elif family == "direct_bc_chunk":
        model = ie._build_direct_bc_policy(backbone=backbone, action_horizon=horizon, camera_count=camera_count, hidden_dim=hidden_dim)
    elif family == "arm_structured_direct":
        model = ie._build_structured_direct_policy(backbone=backbone, action_horizon=horizon, camera_count=camera_count, hidden_dim=hidden_dim)
    elif family == "act_style_transformer_chunk":
        model = ie._build_act_chunk_policy(backbone=backbone, action_horizon=horizon, camera_count=camera_count, hidden_dim=hidden_dim)
    elif family == "diffusion_policy_x0":
        model = ie._build_diffusion_policy(backbone=backbone, action_horizon=horizon, camera_count=camera_count, hidden_dim=hidden_dim, torch=torch)
    elif family == "flow_matching":
        from policy_runner.flow_model import FlowMatchingPolicy, FlowModelConfig
        cfg = dict(payload["model_config"])
        cfg["camera_names"] = tuple(cfg.get("camera_names", []))
        model = FlowMatchingPolicy(FlowModelConfig(**cfg))
    else:
        raise ValueError(f"unsupported family for re-eval: {family}")
    model.load_state_dict(payload["model_state"])
    return model.to(device)


def _evaluate(family, model, dataset, indices, stats, device, torch, meta):
    if family == "flow_matching":
        return ie._evaluate_flow_model(model=model, dataset=dataset, indices=indices, stats=stats, device=device, sample_steps=int(meta["flow_sample_steps"]), torch=torch)
    if family == "diffusion_policy_x0":
        return ie._evaluate_diffusion_model(model=model, dataset=dataset, indices=indices, stats=stats, device=device, sample_steps=int(meta["diffusion_sample_steps"]), torch=torch)
    return ie._evaluate_supervised_model(model=model, dataset=dataset, indices=indices, stats=stats, device=device, torch=torch)


def reeval_checkpoint(ckpt_path: Path, run_dir: Path, val_paths: list[str], data_dir: str, device, torch) -> dict[str, Any] | None:
    payload = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    if "model_state" not in payload or "model_family" not in payload:
        return None
    family = payload["model_family"]
    stats = payload["dataset_stats"]
    meta = _run_meta(run_dir)
    if family == "flow_matching":
        # flow_matching.v1 schema stores everything under model_config (FlowModelConfig).
        cfg = payload["model_config"]
        camera_names = list(cfg.get("camera_names", []))
        horizon = int(cfg["action_horizon"])
        hidden_dim = int(cfg.get("hidden_dim", 256))
    else:
        # imitation_checkpoint.v1 schema: flat keys + hidden_dim from result.json.
        camera_names = list(payload.get("camera_names", []))
        horizon = int(payload["action_horizon"])
        hidden_dim = 256
        rj = run_dir / ckpt_path.parent.name / "result.json"
        if rj.exists():
            hidden_dim = int(_load_json(rj).get("model_config", {}).get("hidden_dim", 256))

    dataset = FlowHdf5Dataset(
        data_dir,
        action_horizon=horizon,
        image_size=int(meta["image_size"]),
        image_crop=str(meta["image_crop"]),
        camera_names=camera_names,
        episode_paths=val_paths,
        stats=stats,
        normalize=True,
    )
    indices = list(range(len(dataset)))
    model = _build_model(payload, hidden_dim, len(camera_names), device, torch)

    # Pass 1: original equal-time phase (restore afterwards).
    original_phase = ie._sample_phase
    ie._sample_phase = ie._quarter_sample_phase
    try:
        baseline = _evaluate(family, model, dataset, indices, stats, device, torch, meta)
    finally:
        ie._sample_phase = original_phase

    # Pass 2: gripper-event phase.
    cache = ps.build_boundary_cache(dataset)
    ie._sample_phase = ps.make_sample_phase(cache)
    try:
        gripper = _evaluate(family, model, dataset, indices, stats, device, torch, meta)
    finally:
        ie._sample_phase = original_phase

    clean = sum(1 for b in cache.values() if b.clean)
    return {
        "run": run_dir.name,
        "model": payload.get("model_family"),
        "model_name": ckpt_path.parent.name,
        "family": family,
        "backbone": payload.get("backbone"),
        "image_size": meta["image_size"],
        "samples": len(dataset),
        "episodes": len(dataset.episodes),
        "clean_segmentation": clean,
        "overall_normalized_action_mse": gripper["normalized_action_mse"],
        "overall_action_mse": gripper["action_mse"],
        "by_phase_quarter": {k: v["normalized_action_mse"] for k, v in baseline["by_phase"].items()},
        "by_phase_gripper": {k: v["normalized_action_mse"] for k, v in gripper["by_phase"].items()},
        "by_phase_gripper_action_mse": {k: v["action_mse"] for k, v in gripper["by_phase"].items()},
        "by_phase_gripper_expanded": gripper["by_phase"],
        "gripper_event_timing": gripper.get("gripper_event_timing", {}),
        "critical_instant_endpoint_error": gripper.get("critical_instant_endpoint_error", {}),
    }


def _render_md(rows: list[dict[str, Any]], split_key: str) -> str:
    lines = [
        "# Phase-wise re-evaluation (gripper-event boundaries)",
        "",
        f"- Validation split: `{split_key}`",
        "- Per-phase numbers are **normalized action MSE** (lower is better).",
        "- `quarter` = original equal-time split; `gripper` = right/left close+open events.",
        "",
        "## Gripper-event phase MSE (the meaningful breakdown)",
        "",
        "| Model | Backbone | Overall | right_pick | right_place | left_pick | left_place | clean segs |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for r in sorted(rows, key=lambda x: x["overall_normalized_action_mse"]):
        g = r["by_phase_gripper"]
        lines.append(
            f"| {r['model_name']} ({r['run']}) | {r['backbone'] or '-'} | "
            f"{r['overall_normalized_action_mse']:.4f} | {g.get('right_pick', float('nan')):.4f} | "
            f"{g.get('right_place', float('nan')):.4f} | {g.get('left_pick', float('nan')):.4f} | "
            f"{g.get('left_place', float('nan')):.4f} | {r['clean_segmentation']}/{r['episodes']} |"
        )
    lines += [
        "",
        "## Gripper timing error by event",
        "",
        "Offline gripper crossing timing error in milliseconds; lower is better and this remains a proxy, not rollout success.",
        "",
        "| Model | right_close mean/median | right_open mean/median | left_close mean/median | left_open mean/median |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for r in sorted(rows, key=lambda x: x["overall_normalized_action_mse"]):
        timing = r.get("gripper_event_timing", {}).get("events", {})
        lines.append(
            f"| {r['model_name']} ({r['run']}) | "
            f"{_timing_cell(timing.get('right_close', {}))} | "
            f"{_timing_cell(timing.get('right_open', {}))} | "
            f"{_timing_cell(timing.get('left_close', {}))} | "
            f"{_timing_cell(timing.get('left_open', {}))} |"
        )
    lines += [
        "",
        "## Per-phase active/inactive arm split",
        "",
        "| Model | Phase | Active arm action MSE | Inactive pred translation L2 | Inactive arm action MSE |",
        "| --- | --- | ---: | ---: | ---: |",
    ]
    for r in sorted(rows, key=lambda x: x["overall_normalized_action_mse"]):
        expanded = r.get("by_phase_gripper_expanded", {})
        for phase in ps.PHASE_NAMES:
            phase_metrics = expanded.get(phase, {})
            lines.append(
                f"| {r['model_name']} ({r['run']}) | {phase} | "
                f"{_metric_value(phase_metrics.get('active_arm_action_mse'))} | "
                f"{_metric_value(phase_metrics.get('inactive_arm_pred_motion'))} | "
                f"{_metric_value(phase_metrics.get('inactive_arm_action_mse'))} |"
            )
    lines += ["", "## Quarter-split vs gripper-event (best model, shows the bias)", ""]
    best = sorted(rows, key=lambda x: x["overall_normalized_action_mse"])[0]
    lines += [
        f"Model: `{best['model_name']}` ({best['run']})",
        "",
        "| Phase | quarter (old) | gripper (new) |",
        "| --- | ---: | ---: |",
    ]
    for name in ps.PHASE_NAMES:
        q = best["by_phase_quarter"].get(name, float("nan"))
        g = best["by_phase_gripper"].get(name, float("nan"))
        lines.append(f"| {name} | {q:.4f} | {g:.4f} |")
    lines.append("")
    return "\n".join(lines)


def _timing_cell(values: dict[str, Any]) -> str:
    mean = values.get("mean_ms")
    median = values.get("median_ms")
    if mean is None or median is None:
        return ""
    return f"{float(mean):.2f}/{float(median):.2f}"


def _metric_value(values: Any) -> str:
    if not isinstance(values, dict) or values.get("value") is None:
        return ""
    return f"{float(values['value']):.6g}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flow-runs-root", default="/outputs/flow_runs")
    parser.add_argument("--split-manifest", required=True)
    parser.add_argument("--split-key", default="session_holdout_val", choices=["session_holdout_val", "validation"])
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--auto", action="store_true", help="Use curated DEFAULT_RUNS list.")
    parser.add_argument("--runs", nargs="*", default=[], help="Explicit run dir names under flow-runs-root.")
    args = parser.parse_args()

    import torch

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    root = Path(args.flow_runs_root)
    manifest = _load_json(Path(args.split_manifest))
    val_paths = [e["path"] for e in manifest[args.split_key]]

    run_names = list(args.runs)
    if args.auto:
        run_names += DEFAULT_RUNS
    seen: set[str] = set()
    run_names = [r for r in run_names if not (r in seen or seen.add(r))]

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    for run_name in run_names:
        run_dir = root / run_name
        ckpts = sorted(run_dir.glob("*/checkpoint.pt"))
        if not ckpts:
            print(f"[skip] no checkpoint.pt under {run_dir}", flush=True)
            continue
        for ckpt in ckpts:
            try:
                row = reeval_checkpoint(ckpt, run_dir, val_paths, args.data_dir, device, torch)
            except Exception as exc:  # keep going across models
                print(f"[error] {ckpt}: {type(exc).__name__}: {exc}", flush=True)
                continue
            if row is None:
                continue
            rows.append(row)
            g = row["by_phase_gripper"]
            print(
                f"[ok] {row['model_name']:>34s} ({row['run'][:40]:40s}) overall={row['overall_normalized_action_mse']:.4f} "
                f"R_pick={g.get('right_pick', float('nan')):.3f} R_place={g.get('right_place', float('nan')):.3f} "
                f"L_pick={g.get('left_pick', float('nan')):.3f} L_place={g.get('left_place', float('nan')):.3f} "
                f"clean={row['clean_segmentation']}/{row['episodes']}",
                flush=True,
            )

    (out_dir / "phase_reeval.json").write_text(json.dumps({"split_key": args.split_key, "rows": rows}, indent=2, sort_keys=True))
    if rows:
        md = _render_md(rows, args.split_key)
        (out_dir / "phase_reeval.md").write_text(md)
        print("\n" + md)
    print(f"\nWrote {out_dir/'phase_reeval.json'} and phase_reeval.md", flush=True)


if __name__ == "__main__":
    main()
