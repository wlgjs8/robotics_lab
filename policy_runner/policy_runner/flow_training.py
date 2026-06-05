from __future__ import annotations

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
    single_arm_side: str | None = None,
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
    manifest = _coerce_dataset_manifest(dataset_manifest)
    if manifest is None:
        resolved_episodes_dir = str(episodes_dir or "data/episodes")
        dataset_kwargs = {
            "camera_names": camera_names,
            "single_arm_side": single_arm_side or "left",
        }
    else:
        resolved_episodes_dir = manifest.resolved_episodes_dir(episodes_dir)
        dataset_kwargs = manifest.training_dataset_kwargs(
            camera_names_override=camera_names,
            single_arm_side_override=single_arm_side,
        )

    stats_source = FlowHdf5Dataset(
        resolved_episodes_dir,
        action_horizon=action_horizon,
        image_size=image_size,
        normalize=False,
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
    print(f"saved flow checkpoint: {checkpoint}", flush=True)
    print(f"saved dataset stats: {stats_path}", flush=True)
    print(f"saved training curves: {curves_path}", flush=True)
    return FlowTrainingResult(
        checkpoint_path=checkpoint,
        dataset_stats_path=stats_path,
        curves_path=curves_path,
        validation_metrics=last_metrics,
    )


def _coerce_dataset_manifest(value: str | Path | DatasetManifest | None) -> DatasetManifest | None:
    if value is None:
        return None
    if isinstance(value, DatasetManifest):
        return value
    return DatasetManifest.load(value)


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
