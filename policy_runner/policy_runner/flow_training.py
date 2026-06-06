from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader, Subset

from .dataset_manifest import DatasetManifest
from .flow_dataset import (
    FLOW_ACTION_DIM,
    FLOW_CHECKPOINT_SCHEMA,
    FlowHdf5Dataset,
    compute_dataset_statistics,
    denormalize_actions,
    write_dataset_statistics,
)
from .flow_model import (
    FlowMatchingPolicy,
    FlowModelConfig,
    flow_matching_loss,
    sample_action_chunks,
)


@dataclass(frozen=True)
class FlowTrainingResult:
    checkpoint_path: Path
    dataset_stats_path: Path
    curves_path: Path
    eval_report_path: Path
    eval_summary_path: Path
    validation_metrics: dict[str, float]


def train_flow_matching(
    *,
    episodes_dir: str | Path | None,
    checkpoint_path: str | Path,
    vision_backbone: str = "resnet50",
    action_horizon: int = 16,
    batch_size: int = 32,
    epochs: int = 100,
    lr: float = 1e-4,
    image_size: int = 224,
    hidden_dim: int = 128,
    condition_encoder: str = "transformer",
    frozen_vision: bool = True,
    val_split: float = 0.1,
    sample_steps: int = 16,
    device: str = "auto",
    max_stats_samples: int | None = None,
    dataset_manifest: str | Path | DatasetManifest | None = None,
    camera_names: list[str] | None = None,
    exclude_camera_names: list[str] | None = None,
    single_arm_side: str | None = None,
    max_episodes: int | None = None,
    write_eval_report: str | Path | None = None,
) -> FlowTrainingResult:
    if epochs <= 0:
        raise ValueError("epochs must be positive")
    if batch_size <= 0:
        raise ValueError("batch_size must be positive")
    if not 0.0 <= val_split < 1.0:
        raise ValueError("val_split must be in [0, 1)")

    checkpoint = Path(checkpoint_path)
    checkpoint.parent.mkdir(parents=True, exist_ok=True)
    stats_path = checkpoint.parent / "dataset_stats.json"
    curves_path = checkpoint.parent / "training_curves.jsonl"
    eval_report_path = Path(write_eval_report) if write_eval_report is not None else checkpoint.parent / "flow_eval_report.md"
    eval_summary_path = eval_report_path.parent / "flow_eval_summary.json"
    manifest = _coerce_dataset_manifest(dataset_manifest)
    if manifest is None:
        resolved_episodes_dir = str(episodes_dir or "data/episodes")
        dataset_kwargs = {
            "camera_names": camera_names,
            "exclude_camera_names": exclude_camera_names or [],
            "single_arm_side": single_arm_side or "left",
            "include_formats": None,
            "required_attrs": {},
        }
    else:
        resolved_episodes_dir = manifest.resolved_episodes_dir(episodes_dir)
        dataset_kwargs = manifest.training_dataset_kwargs(
            camera_names_override=camera_names,
            exclude_camera_names_override=exclude_camera_names,
            single_arm_side_override=single_arm_side,
        )

    stats_source = FlowHdf5Dataset(
        resolved_episodes_dir,
        action_horizon=action_horizon,
        image_size=image_size,
        normalize=False,
        max_episodes=max_episodes,
        **dataset_kwargs,
    )
    stats = compute_dataset_statistics(stats_source, max_samples=max_stats_samples)
    write_dataset_statistics(stats_path, stats)

    dataset = FlowHdf5Dataset(
        resolved_episodes_dir,
        action_horizon=action_horizon,
        image_size=image_size,
        camera_names=list(stats["camera_names"]),
        single_arm_side=dataset_kwargs["single_arm_side"],
        include_formats=dataset_kwargs.get("include_formats"),
        exclude_camera_names=dataset_kwargs.get("exclude_camera_names"),
        required_attrs=dataset_kwargs.get("required_attrs"),
        episode_paths=dataset_kwargs.get("episode_paths"),
        include_patterns=dataset_kwargs.get("include_patterns"),
        max_episodes=max_episodes,
        stats=stats,
        normalize=True,
    )
    train_indices, val_indices = _split_indices(len(dataset), val_split)
    train_loader = DataLoader(
        Subset(dataset, train_indices),
        batch_size=batch_size,
        shuffle=True,
    )
    val_loader = DataLoader(
        Subset(dataset, val_indices),
        batch_size=batch_size,
        shuffle=False,
    )

    torch_device = _resolve_device(device)
    model_config = FlowModelConfig(
        action_horizon=action_horizon,
        action_dim=FLOW_ACTION_DIM,
        proprio_dim=dataset.proprio_dim,
        camera_names=tuple(dataset.camera_names),
        vision_backbone=vision_backbone,
        hidden_dim=hidden_dim,
        condition_encoder=condition_encoder,
        frozen_vision=frozen_vision,
    )
    model = FlowMatchingPolicy(model_config).to(torch_device)
    optimizer = torch.optim.AdamW(
        (parameter for parameter in model.parameters() if parameter.requires_grad),
        lr=float(lr),
    )

    last_metrics: dict[str, float] = {}
    with curves_path.open("w", encoding="utf-8") as curves:
        for epoch in range(1, epochs + 1):
            model.train()
            total_loss = 0.0
            total_count = 0
            for batch in train_loader:
                batch = _to_device(batch, torch_device)
                loss = flow_matching_loss(model, batch)
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=10.0)
                optimizer.step()
                count = int(batch["action_chunk"].shape[0])
                total_loss += float(loss.item()) * count
                total_count += count
            train_loss = total_loss / max(total_count, 1)
            last_metrics = validate_flow_policy(
                model,
                val_loader,
                stats=stats,
                device=torch_device,
                sample_steps=sample_steps,
            )
            row = {"epoch": epoch, "train_loss": train_loss, **last_metrics}
            curves.write(json.dumps(row, sort_keys=True) + "\n")
            curves.flush()
            print(
                "epoch="
                f"{epoch} train_loss={train_loss:.8f} "
                f"action_mse={last_metrics['action_mse']:.8f} "
                f"gripper_mse={last_metrics['gripper_mse']:.8f} "
                f"chunk_endpoint_error={last_metrics['chunk_endpoint_error']:.8f}",
                flush=True,
            )

    checkpoint_payload = {
        "schema": FLOW_CHECKPOINT_SCHEMA,
        "model_config": model_config.to_dict(),
        "action_horizon": int(action_horizon),
        "action_dim": FLOW_ACTION_DIM,
        "proprio_dim": dataset.proprio_dim,
        "camera_names": list(dataset.camera_names),
        "image_size": int(image_size),
        "dataset_stats": stats,
        "validation_metrics": last_metrics,
        "model_state": model.cpu().state_dict(),
        "training_args": {
            "episodes_dir": str(resolved_episodes_dir),
            "dataset_manifest": str(dataset_manifest) if dataset_manifest is not None else None,
            "camera_names": list(dataset.camera_names),
            "exclude_camera_names": list(dataset_kwargs.get("exclude_camera_names") or []),
            "single_arm_side": dataset_kwargs["single_arm_side"],
            "max_episodes": max_episodes,
            "vision_backbone": vision_backbone,
            "batch_size": int(batch_size),
            "epochs": int(epochs),
            "lr": float(lr),
            "hidden_dim": int(hidden_dim),
            "condition_encoder": condition_encoder,
            "frozen_vision": bool(frozen_vision),
            "val_split": float(val_split),
            "sample_steps": int(sample_steps),
        },
    }
    torch.save(checkpoint_payload, checkpoint)
    checkpoint_info = {
        "schema": FLOW_CHECKPOINT_SCHEMA,
        "sha256": _sha256_file(checkpoint),
    }
    audit_report = _best_effort_audit(
        resolved_episodes_dir,
        dataset_manifest=manifest,
        single_arm_side=dataset_kwargs["single_arm_side"],
    )
    eval_summary = build_flow_eval_summary(
        stats=stats,
        validation_metrics=last_metrics,
        checkpoint=checkpoint_info,
        checkpoint_path=checkpoint,
        curves_path=curves_path,
        training_args=checkpoint_payload["training_args"],
        audit_report=audit_report,
    )
    write_flow_eval_artifacts(
        summary=eval_summary,
        report_path=eval_report_path,
        summary_path=eval_summary_path,
    )
    print(f"saved flow checkpoint: {checkpoint}", flush=True)
    print(f"saved dataset stats: {stats_path}", flush=True)
    print(f"saved training curves: {curves_path}", flush=True)
    print(f"saved flow eval report: {eval_report_path}", flush=True)
    print(f"saved flow eval summary: {eval_summary_path}", flush=True)
    return FlowTrainingResult(
        checkpoint_path=checkpoint,
        dataset_stats_path=stats_path,
        curves_path=curves_path,
        eval_report_path=eval_report_path,
        eval_summary_path=eval_summary_path,
        validation_metrics=last_metrics,
    )


def _coerce_dataset_manifest(value: str | Path | DatasetManifest | None) -> DatasetManifest | None:
    if value is None:
        return None
    if isinstance(value, DatasetManifest):
        return value
    return DatasetManifest.load(value)


def build_flow_eval_summary(
    *,
    stats: dict[str, Any],
    validation_metrics: dict[str, float],
    checkpoint: dict[str, str],
    checkpoint_path: Path,
    curves_path: Path,
    training_args: dict[str, Any],
    audit_report: dict[str, Any] | None,
) -> dict[str, Any]:
    audit_aggregate = (audit_report or {}).get("aggregate", {})
    warnings = _collect_audit_values(audit_report, "warnings")
    deployment_blockers = _collect_audit_values(audit_report, "deployment_blockers")
    return {
        "schema": "robotics_lab.policy_runner.flow_eval_summary.v1",
        "dataset": {
            "formats": list(stats.get("formats", [])),
            "episode_count": int(stats.get("episode_count", 0)),
            "frame_count": int(stats.get("frame_count", audit_aggregate.get("frame_count", 0) or 0)),
            "sample_count": int(stats.get("total_sample_count", stats.get("sample_count", 0))),
            "stats_sample_count": int(stats.get("sample_count", 0)),
            "camera_names": list(stats.get("camera_names", [])),
            "image_decode_count": int(stats.get("image_decode_count", 0)),
            "missing_camera_count": int(stats.get("missing_camera_count", 0)),
            "action_horizon": int(stats.get("action_horizon", 0)),
            "arm_mask_counts": dict(stats.get("arm_mask_counts", {})),
        },
        "validation": {
            "action_mse": float(validation_metrics.get("action_mse", 0.0)),
            "gripper_mse": float(validation_metrics.get("gripper_mse", 0.0)),
            "chunk_endpoint_error": float(validation_metrics.get("chunk_endpoint_error", 0.0)),
            "image_decode_count": float(validation_metrics.get("image_decode_count", 0.0)),
            "missing_camera_count": float(validation_metrics.get("missing_camera_count", 0.0)),
        },
        "action_distribution_percentiles": dict(
            stats.get("action_distribution_percentiles", {})
        ),
        "checkpoint": {
            "path": str(checkpoint_path),
            "schema": str(checkpoint.get("schema", "")),
            "sha256": str(checkpoint.get("sha256", "")),
        },
        "artifacts": {
            "dataset_stats": "dataset_stats.json",
            "training_curves": str(curves_path),
        },
        "training_args": dict(training_args),
        "warnings": {
            "audit": warnings,
            "deployment_blockers": deployment_blockers,
            "audit_warning_count": int(audit_aggregate.get("warning_count", len(warnings)) or 0),
            "audit_deployment_blocker_count": int(
                audit_aggregate.get("deployment_blocker_count", len(deployment_blockers)) or 0
            ),
        },
    }


def write_flow_eval_artifacts(
    *,
    summary: dict[str, Any],
    report_path: str | Path,
    summary_path: str | Path,
) -> None:
    report = Path(report_path)
    summary_output = Path(summary_path)
    report.parent.mkdir(parents=True, exist_ok=True)
    summary_output.parent.mkdir(parents=True, exist_ok=True)
    summary_output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    report.write_text(render_flow_eval_report(summary), encoding="utf-8")


def render_flow_eval_report(summary: dict[str, Any]) -> str:
    dataset = summary.get("dataset", {})
    validation = summary.get("validation", {})
    checkpoint = summary.get("checkpoint", {})
    warnings = summary.get("warnings", {})
    lines = [
        "# Flow Evaluation Report",
        "",
        "## Dataset",
        "",
        f"- Formats: {', '.join(dataset.get('formats', [])) or '(none)'}",
        f"- Episodes: {dataset.get('episode_count', 0)}",
        f"- Frames: {dataset.get('frame_count', 0)}",
        f"- Samples: {dataset.get('sample_count', 0)}",
        f"- Cameras: {', '.join(dataset.get('camera_names', [])) or '(none)'}",
        f"- Image decode count: {dataset.get('image_decode_count', 0)}",
        f"- Missing camera count: {dataset.get('missing_camera_count', 0)}",
        f"- Action horizon: {dataset.get('action_horizon', 0)}",
        f"- Arm mask counts: `{json.dumps(dataset.get('arm_mask_counts', {}), sort_keys=True)}`",
        "",
        "## Validation",
        "",
        f"- action_mse: {float(validation.get('action_mse', 0.0)):.8g}",
        f"- gripper_mse: {float(validation.get('gripper_mse', 0.0)):.8g}",
        f"- chunk_endpoint_error: {float(validation.get('chunk_endpoint_error', 0.0)):.8g}",
        "",
        "## Checkpoint",
        "",
        f"- Schema: `{checkpoint.get('schema', '')}`",
        f"- SHA-256: `{checkpoint.get('sha256', '')}`",
        f"- Path: `{checkpoint.get('path', '')}`",
        "",
        "## Action Distribution Percentiles",
        "",
        "| Dimension | p01 | p05 | p50 | p95 | p99 |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for name, values in summary.get("action_distribution_percentiles", {}).items():
        lines.append(
            "| "
            f"{name} | "
            f"{float(values.get('p01', 0.0)):.8g} | "
            f"{float(values.get('p05', 0.0)):.8g} | "
            f"{float(values.get('p50', 0.0)):.8g} | "
            f"{float(values.get('p95', 0.0)):.8g} | "
            f"{float(values.get('p99', 0.0)):.8g} |"
        )
    lines.extend(["", "## Warnings", ""])
    audit_warnings = list(warnings.get("audit", []))
    deployment_blockers = list(warnings.get("deployment_blockers", []))
    if not audit_warnings and not deployment_blockers:
        lines.append("- None")
    else:
        lines.extend(f"- {warning}" for warning in audit_warnings)
        lines.extend(f"- deployment_blocker: {blocker}" for blocker in deployment_blockers)
    return "\n".join(lines).rstrip() + "\n"


def _best_effort_audit(
    episodes_dir: str | Path,
    *,
    dataset_manifest: DatasetManifest | None,
    single_arm_side: str,
) -> dict[str, Any] | None:
    try:
        from .hdf5_audit import audit_hdf5_episodes

        return audit_hdf5_episodes(
            episodes_dir,
            dataset_manifest=dataset_manifest,
            single_arm_side=single_arm_side,
        )
    except Exception as exc:  # noqa: BLE001 - report generation should not hide a trained checkpoint.
        return {
            "aggregate": {
                "warnings": [f"audit_unavailable: {type(exc).__name__}: {exc}"],
                "deployment_blockers": [],
                "warning_count": 1,
                "deployment_blocker_count": 0,
            },
            "episodes": [],
        }


def _collect_audit_values(report: dict[str, Any] | None, key: str) -> list[str]:
    if not report:
        return []
    values: list[str] = []
    aggregate = report.get("aggregate", {})
    for value in aggregate.get(key, []):
        if str(value) not in values:
            values.append(str(value))
    for episode in report.get("episodes", []):
        for value in episode.get(key, []):
            if str(value) not in values:
                values.append(str(value))
    return values


def _sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@torch.no_grad()
def validate_flow_policy(
    model: FlowMatchingPolicy,
    loader: DataLoader,
    *,
    stats: dict[str, Any],
    device: torch.device,
    sample_steps: int = 16,
) -> dict[str, float]:
    model.eval()
    action_sq_sum = 0.0
    action_count = 0.0
    gripper_sq_sum = 0.0
    gripper_count = 0.0
    endpoint_sum = 0.0
    endpoint_count = 0.0
    image_decode_count = 0
    missing_camera_count = 0
    for batch in loader:
        batch = _to_device(batch, device)
        pred = sample_action_chunks(
            model,
            batch["images"].float(),
            batch["proprio"].float(),
            steps=sample_steps,
        )
        target = batch["action_chunk"].float()
        mask = batch["action_mask"].float()
        pred_raw = denormalize_actions(pred, stats)
        target_raw = denormalize_actions(target, stats)
        error_sq = (pred_raw - target_raw) ** 2
        action_sq_sum += float((error_sq * mask).sum().item())
        action_count += float(mask.sum().item())

        grip_mask = torch.zeros_like(mask)
        grip_mask[:, :, 6] = mask[:, :, 6]
        grip_mask[:, :, 13] = mask[:, :, 13]
        gripper_sq_sum += float((error_sq * grip_mask).sum().item())
        gripper_count += float(grip_mask.sum().item())

        for arm_start in (0, 7):
            active = mask[:, -1, arm_start : arm_start + 6].sum(dim=-1) > 0.0
            if not bool(active.any()):
                continue
            endpoint_error = torch.linalg.vector_norm(
                pred_raw[:, -1, arm_start : arm_start + 6]
                - target_raw[:, -1, arm_start : arm_start + 6],
                dim=-1,
            )
            endpoint_sum += float(endpoint_error[active].sum().item())
            endpoint_count += float(active.sum().item())

        image_decode_count += int(batch["image_decode_count"].sum().item())
        missing_camera_count += int(batch["missing_camera_count"].sum().item())

    return {
        "action_mse": action_sq_sum / max(action_count, 1.0),
        "gripper_mse": gripper_sq_sum / max(gripper_count, 1.0),
        "chunk_endpoint_error": endpoint_sum / max(endpoint_count, 1.0),
        "image_decode_count": float(image_decode_count),
        "missing_camera_count": float(missing_camera_count),
    }


def _split_indices(dataset_len: int, val_split: float) -> tuple[list[int], list[int]]:
    indices = list(range(dataset_len))
    rng = np.random.default_rng(0)
    rng.shuffle(indices)
    if dataset_len <= 1 or val_split == 0.0:
        return indices, indices
    val_count = max(1, int(round(dataset_len * val_split)))
    val_count = min(val_count, dataset_len - 1)
    return indices[val_count:], indices[:val_count]


def _to_device(batch: dict[str, Any], device: torch.device) -> dict[str, torch.Tensor]:
    out: dict[str, torch.Tensor] = {}
    for key, value in batch.items():
        tensor = value if isinstance(value, torch.Tensor) else torch.as_tensor(value)
        out[key] = tensor.to(device)
    return out


def _resolve_device(device: str) -> torch.device:
    if device == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(device)
