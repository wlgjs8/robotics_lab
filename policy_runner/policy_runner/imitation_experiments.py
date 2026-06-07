from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .flow_dataset import (
    FLOW_ACTION_DIM,
    FLOW_PROPRIO_DIM,
    FLOW_CHECKPOINT_SCHEMA,
    FLOW_ACTION_DIM_NAMES,
    FlowHdf5Dataset,
    compute_dataset_statistics,
    denormalize_actions,
    discover_hdf5_episodes,
    load_flow_episode_index,
    write_dataset_statistics,
)


IMITATION_SNAPSHOT_SCHEMA = "robotics_lab.policy_runner.imitation_snapshot.v1"
IMITATION_SPLIT_SCHEMA = "robotics_lab.policy_runner.imitation_split.v1"
IMITATION_REPORT_SCHEMA = "robotics_lab.policy_runner.imitation_report.v1"
PHASE_NAMES = ("right_pick", "right_place", "left_pick", "left_place")
DEFAULT_MODEL_FAMILY = (
    "zero",
    "train_mean",
    "state_mlp",
    "direct_bc_tiny",
    "direct_bc_resnet18",
    "flow_tiny",
    "flow_resnet18",
    "diffusion_resnet18",
    "act_tiny",
    "structured_resnet18",
)


@dataclass(frozen=True)
class SnapshotResult:
    snapshot_path: Path
    snapshot_hash: str
    valid_episode_count: int
    rejected_episode_count: int


@dataclass(frozen=True)
class SplitResult:
    split_path: Path
    split_hash: str
    train_count: int
    val_count: int
    session_holdout_val_count: int


@dataclass(frozen=True)
class AggregateResult:
    summary_path: Path
    report_path: Path
    row_count: int
    failed_count: int


def create_dataset_snapshot(
    *,
    data_dir: str | Path,
    output_dir: str | Path,
    action_horizon: int = 16,
    single_arm_side: str = "left",
) -> SnapshotResult:
    root = Path(data_dir)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    created_at = _utc_now()
    valid: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []

    for path in discover_hdf5_episodes(root):
        try:
            index = load_flow_episode_index(path, single_arm_side=single_arm_side)
            frame_count = int(index.length)
            sample_count = max(0, frame_count - int(action_horizon))
            warnings: list[str] = []
            if frame_count <= action_horizon:
                warnings.append("too_short_for_action_horizon")
            if not index.camera_paths:
                warnings.append("no_cameras_detected")
            if not np.isfinite(index.timestamps).all():
                warnings.append("nonfinite_timestamps")
            timestamp_range = _timestamp_range(index.timestamps)
            valid.append(
                {
                    "path": str(Path(path).resolve()),
                    "relative_path": str(Path(path).resolve().relative_to(root.resolve()))
                    if _path_is_relative_to(Path(path).resolve(), root.resolve())
                    else str(path),
                    "session": _session_name(root, path),
                    "sha256": _sha256_file(path),
                    "frame_count": frame_count,
                    "sample_count_horizon": sample_count,
                    "detected_cameras": sorted(index.camera_paths),
                    "detected_format": index.format_name,
                    "timestamp_range": timestamp_range,
                    "action_dim": FLOW_ACTION_DIM,
                    "proprio_dim": FLOW_PROPRIO_DIM,
                    "arm_mask": index.arm_mask.astype(float).tolist(),
                    "audit_warnings": warnings,
                }
            )
        except Exception as exc:  # noqa: BLE001 - snapshot must preserve rejection reason.
            rejected.append(
                {
                    "path": str(Path(path).resolve()),
                    "session": _session_name(root, path),
                    "reason": f"{type(exc).__name__}: {exc}",
                }
            )

    if not valid:
        raise ValueError(f"no valid HDF5 episodes found under {root}")

    payload = {
        "schema": IMITATION_SNAPSHOT_SCHEMA,
        "created_at": created_at,
        "data_dir": str(root.resolve()),
        "action_horizon": int(action_horizon),
        "single_arm_side": single_arm_side,
        "episodes": sorted(valid, key=lambda item: item["path"]),
        "rejected": sorted(rejected, key=lambda item: item["path"]),
        "aggregate": {
            "valid_episode_count": len(valid),
            "rejected_episode_count": len(rejected),
            "frame_count": int(sum(item["frame_count"] for item in valid)),
            "sample_count_horizon": int(sum(item["sample_count_horizon"] for item in valid)),
            "sessions": sorted({item["session"] for item in valid}),
            "detected_cameras": sorted(
                {camera for item in valid for camera in item["detected_cameras"]}
            ),
            "detected_formats": sorted({item["detected_format"] for item in valid}),
            "audit_warning_count": int(sum(len(item["audit_warnings"]) for item in valid)),
        },
    }
    snapshot_hash = _payload_hash(payload)
    payload["snapshot_hash"] = snapshot_hash
    snapshot_path = output / "dataset_snapshot.json"
    snapshot_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return SnapshotResult(
        snapshot_path=snapshot_path,
        snapshot_hash=snapshot_hash,
        valid_episode_count=len(valid),
        rejected_episode_count=len(rejected),
    )


def create_split_manifest(
    *,
    snapshot_path: str | Path,
    output_dir: str | Path,
    val_ratio: float = 0.2,
    seed: int = 0,
    create_session_holdout: bool = True,
) -> SplitResult:
    if not 0.0 < val_ratio < 1.0:
        raise ValueError("val_ratio must be in (0, 1)")
    snapshot = _load_json(snapshot_path)
    if snapshot.get("schema") != IMITATION_SNAPSHOT_SCHEMA:
        raise ValueError(f"unsupported snapshot schema: {snapshot.get('schema')}")
    episodes = list(snapshot.get("episodes", []))
    if len(episodes) < 2:
        raise ValueError("at least two valid episodes are required for train/validation split")

    train, val, algorithm = _session_stratified_split(episodes, val_ratio=val_ratio, seed=seed)
    holdout_train: list[dict[str, Any]] = []
    holdout_val: list[dict[str, Any]] = []
    holdout_sessions: list[str] = []
    if create_session_holdout:
        holdout_train, holdout_val, holdout_sessions = _session_holdout_split(
            episodes,
            val_ratio=val_ratio,
            seed=seed,
        )
    payload = {
        "schema": IMITATION_SPLIT_SCHEMA,
        "created_at": _utc_now(),
        "snapshot_path": str(Path(snapshot_path).resolve()),
        "snapshot_hash": str(snapshot["snapshot_hash"]),
        "val_ratio": float(val_ratio),
        "split_seed": int(seed),
        "split_algorithm": algorithm,
        "train": train,
        "validation": val,
        "session_holdout_train": holdout_train,
        "session_holdout_val": holdout_val,
        "session_holdout_sessions": holdout_sessions,
    }
    split_hash = _payload_hash(payload)
    payload["split_hash"] = split_hash
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    split_path = output / "split_manifest.json"
    split_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return SplitResult(
        split_path=split_path,
        split_hash=split_hash,
        train_count=len(train),
        val_count=len(val),
        session_holdout_val_count=len(holdout_val),
    )


def run_imitation_experiment(
    *,
    data_dir: str | Path,
    output_dir: str | Path,
    snapshot_path: str | Path | None = None,
    split_path: str | Path | None = None,
    models: Iterable[str] = DEFAULT_MODEL_FAMILY,
    action_horizon: int = 16,
    image_size: int = 128,
    image_crop: str = "none",
    camera_names: list[str] | None = None,
    exclude_camera_names: list[str] | None = None,
    batch_size: int = 64,
    epochs: int = 20,
    lr: float = 1e-4,
    hidden_dim: int = 256,
    device: str = "auto",
    seed: int = 0,
    max_train_samples: int | None = None,
    max_val_samples: int | None = None,
    flow_sample_steps: int = 8,
    diffusion_sample_steps: int = 16,
    split_mode: str = "primary",
) -> dict[str, Any]:
    torch = _require_torch()
    _seed_everything(seed, torch=torch)

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    if snapshot_path is None:
        snapshot = create_dataset_snapshot(
            data_dir=data_dir,
            output_dir=output,
            action_horizon=action_horizon,
        )
        snapshot_path = snapshot.snapshot_path
    if split_path is None:
        split = create_split_manifest(snapshot_path=snapshot_path, output_dir=output, seed=seed)
        split_path = split.split_path

    snapshot_payload = _load_json(snapshot_path)
    split_payload = _load_json(split_path)
    snapshot_payload["_source_path"] = str(Path(snapshot_path).resolve())
    split_payload["_source_path"] = str(Path(split_path).resolve())
    _validate_snapshot_split(snapshot_payload, split_payload)
    effective_split = _effective_split_for_mode(
        snapshot_payload,
        split_payload,
        split_mode=split_mode,
    )
    train_paths = [entry["path"] for entry in effective_split["train"]]
    val_paths = [entry["path"] for entry in effective_split["validation"]]
    if not train_paths or not val_paths:
        raise ValueError("split manifest must contain non-empty train and validation episode lists")

    selected_cameras = camera_names
    excluded = exclude_camera_names or []
    stats_source = FlowHdf5Dataset(
        data_dir,
        action_horizon=action_horizon,
        image_size=image_size,
        image_crop=image_crop,
        episode_paths=train_paths,
        camera_names=selected_cameras,
        exclude_camera_names=excluded,
        normalize=False,
    )
    stats = compute_dataset_statistics(stats_source, max_samples=max_train_samples)
    stats["snapshot_hash"] = snapshot_payload["snapshot_hash"]
    stats["split_hash"] = split_payload["split_hash"]
    stats["split_mode"] = split_mode
    stats["stats_source"] = "train_only"
    stats_path = output / "train_dataset_stats.json"
    write_dataset_statistics(stats_path, stats)

    train_dataset = FlowHdf5Dataset(
        data_dir,
        action_horizon=action_horizon,
        image_size=image_size,
        image_crop=image_crop,
        episode_paths=train_paths,
        camera_names=list(stats["camera_names"]),
        exclude_camera_names=excluded,
        stats=stats,
        normalize=True,
    )
    val_dataset = FlowHdf5Dataset(
        data_dir,
        action_horizon=action_horizon,
        image_size=image_size,
        image_crop=image_crop,
        episode_paths=val_paths,
        camera_names=list(stats["camera_names"]),
        exclude_camera_names=excluded,
        stats=stats,
        normalize=True,
    )
    train_indices = _limited_indices(len(train_dataset), max_train_samples)
    val_indices = _limited_indices(len(val_dataset), max_val_samples)

    torch_device = _resolve_device(device, torch=torch)
    gpu_info = _gpu_info(torch=torch)
    model_names = list(models)
    results: list[dict[str, Any]] = []
    for model_name in model_names:
        model_dir = output / model_name
        model_dir.mkdir(parents=True, exist_ok=True)
        start = time.perf_counter()
        try:
            if model_name == "zero":
                result = _evaluate_constant_baseline(
                    name=model_name,
                    prediction=np.zeros(FLOW_ACTION_DIM, dtype=np.float32),
                    train_dataset=train_dataset,
                    val_dataset=val_dataset,
                    train_indices=train_indices,
                    val_indices=val_indices,
                    stats=stats,
                    output_dir=model_dir,
                )
            elif model_name == "train_mean":
                result = _evaluate_constant_baseline(
                    name=model_name,
                    prediction=np.asarray(stats["action_mean"], dtype=np.float32),
                    train_dataset=train_dataset,
                    val_dataset=val_dataset,
                    train_indices=train_indices,
                    val_indices=val_indices,
                    stats=stats,
                    output_dir=model_dir,
                )
            elif model_name == "state_mlp":
                result = _train_state_mlp(
                    train_dataset=train_dataset,
                    val_dataset=val_dataset,
                    train_indices=train_indices,
                    val_indices=val_indices,
                    stats=stats,
                    output_dir=model_dir,
                    batch_size=batch_size,
                    epochs=epochs,
                    lr=lr,
                    hidden_dim=hidden_dim,
                    device=torch_device,
                    torch=torch,
                )
            elif model_name.startswith("direct_bc_"):
                backbone, loss_name = _backbone_and_loss(model_name.removeprefix("direct_bc_"))
                result = _train_direct_bc(
                    name=model_name,
                    backbone=backbone,
                    loss_name=loss_name,
                    train_dataset=train_dataset,
                    val_dataset=val_dataset,
                    train_indices=train_indices,
                    val_indices=val_indices,
                    stats=stats,
                    output_dir=model_dir,
                    batch_size=batch_size,
                    epochs=epochs,
                    lr=lr,
                    hidden_dim=hidden_dim,
                    device=torch_device,
                    torch=torch,
                )
            elif model_name.startswith("structured_"):
                backbone, loss_name = _backbone_and_loss(model_name.removeprefix("structured_"))
                result = _train_structured_direct(
                    name=model_name,
                    backbone=backbone,
                    loss_name=loss_name,
                    train_dataset=train_dataset,
                    val_dataset=val_dataset,
                    train_indices=train_indices,
                    val_indices=val_indices,
                    stats=stats,
                    output_dir=model_dir,
                    batch_size=batch_size,
                    epochs=epochs,
                    lr=lr,
                    hidden_dim=hidden_dim,
                    device=torch_device,
                    torch=torch,
                )
            elif model_name.startswith("flow_"):
                backbone = model_name.removeprefix("flow_")
                result = _train_flow(
                    name=model_name,
                    backbone=_backbone_name(backbone),
                    train_dataset=train_dataset,
                    val_dataset=val_dataset,
                    train_indices=train_indices,
                    val_indices=val_indices,
                    stats=stats,
                    output_dir=model_dir,
                    batch_size=batch_size,
                    epochs=epochs,
                    lr=lr,
                    hidden_dim=hidden_dim,
                    sample_steps=flow_sample_steps,
                    device=torch_device,
                    torch=torch,
                )
            elif model_name.startswith("diffusion_"):
                backbone = model_name.removeprefix("diffusion_")
                result = _train_diffusion(
                    name=model_name,
                    backbone=_backbone_name(backbone),
                    train_dataset=train_dataset,
                    val_dataset=val_dataset,
                    train_indices=train_indices,
                    val_indices=val_indices,
                    stats=stats,
                    output_dir=model_dir,
                    batch_size=batch_size,
                    epochs=epochs,
                    lr=lr,
                    hidden_dim=hidden_dim,
                    sample_steps=diffusion_sample_steps,
                    device=torch_device,
                    torch=torch,
                )
            elif model_name.startswith("act_"):
                backbone = model_name.removeprefix("act_")
                result = _train_act(
                    name=model_name,
                    backbone=_backbone_name(backbone),
                    train_dataset=train_dataset,
                    val_dataset=val_dataset,
                    train_indices=train_indices,
                    val_indices=val_indices,
                    stats=stats,
                    output_dir=model_dir,
                    batch_size=batch_size,
                    epochs=epochs,
                    lr=lr,
                    hidden_dim=hidden_dim,
                    device=torch_device,
                    torch=torch,
                )
            else:
                raise ValueError(f"unknown imitation model family: {model_name}")
        except Exception as exc:  # noqa: BLE001 - failed experiments are reportable evidence.
            result = {
                "model": model_name,
                "family": "failed",
                "status": "failed",
                "error_type": type(exc).__name__,
                "error": str(exc),
                "checkpoint_path": None,
                "checkpoint_sha256": None,
                "metrics": {},
                "latency": {},
            }
        result["wall_time_sec"] = float(time.perf_counter() - start)
        result["gpu_info"] = gpu_info
        results.append(result)
        (model_dir / "result.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    report = _build_report(
        output_dir=output,
        snapshot=snapshot_payload,
        split=split_payload,
        effective_split=effective_split,
        stats=stats,
        stats_path=stats_path,
        results=results,
        args={
            "data_dir": str(Path(data_dir).resolve()),
            "action_horizon": action_horizon,
            "image_size": image_size,
            "image_crop": image_crop,
            "camera_names": list(stats["camera_names"]),
            "exclude_camera_names": excluded,
            "batch_size": batch_size,
            "epochs": epochs,
            "lr": lr,
            "hidden_dim": hidden_dim,
            "device": str(torch_device),
            "seed": seed,
            "max_train_samples": max_train_samples,
            "max_val_samples": max_val_samples,
            "flow_sample_steps": flow_sample_steps,
            "diffusion_sample_steps": diffusion_sample_steps,
            "split_mode": split_mode,
            "models": model_names,
        },
    )
    report_path = output / "leaderboard_report.md"
    summary_path = output / "leaderboard_summary.json"
    report_path.write_text(_render_report_markdown(report), encoding="utf-8")
    summary_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def _train_state_mlp(**kwargs: Any) -> dict[str, Any]:
    model = _build_state_mlp(
        action_horizon=kwargs["train_dataset"].action_horizon,
        hidden_dim=kwargs["hidden_dim"],
    ).to(kwargs["device"])
    return _train_supervised_model(
        model=model,
        family="state_only_mlp",
        uses_images=False,
        model_name="state_mlp",
        **kwargs,
    )


def _train_direct_bc(*, name: str, backbone: str, loss_name: str = "mse", **kwargs: Any) -> dict[str, Any]:
    model = _build_direct_bc_policy(
        backbone=backbone,
        action_horizon=kwargs["train_dataset"].action_horizon,
        camera_count=len(kwargs["train_dataset"].camera_names),
        hidden_dim=kwargs["hidden_dim"],
    ).to(kwargs["device"])
    return _train_supervised_model(
        model=model,
        family="direct_bc_chunk",
        uses_images=True,
        model_name=name,
        backbone=backbone,
        loss_name=loss_name,
        **kwargs,
    )


def _train_structured_direct(*, name: str, backbone: str, loss_name: str = "mse", **kwargs: Any) -> dict[str, Any]:
    model = _build_structured_direct_policy(
        backbone=backbone,
        action_horizon=kwargs["train_dataset"].action_horizon,
        camera_count=len(kwargs["train_dataset"].camera_names),
        hidden_dim=kwargs["hidden_dim"],
    ).to(kwargs["device"])
    return _train_supervised_model(
        model=model,
        family="arm_structured_direct",
        uses_images=True,
        model_name=name,
        backbone=backbone,
        loss_name=loss_name,
        **kwargs,
    )


def _train_act(*, name: str, backbone: str, **kwargs: Any) -> dict[str, Any]:
    model = _build_act_chunk_policy(
        backbone=backbone,
        action_horizon=kwargs["train_dataset"].action_horizon,
        camera_count=len(kwargs["train_dataset"].camera_names),
        hidden_dim=kwargs["hidden_dim"],
    ).to(kwargs["device"])
    return _train_supervised_model(
        model=model,
        family="act_style_transformer_chunk",
        uses_images=True,
        model_name=name,
        backbone=backbone,
        **kwargs,
    )


def _train_flow(*, name: str, backbone: str, **kwargs: Any) -> dict[str, Any]:
    torch = kwargs["torch"]
    from .flow_model import FlowMatchingPolicy, FlowModelConfig, flow_matching_loss

    config = FlowModelConfig(
        action_horizon=kwargs["train_dataset"].action_horizon,
        action_dim=FLOW_ACTION_DIM,
        proprio_dim=FLOW_PROPRIO_DIM,
        camera_names=tuple(kwargs["train_dataset"].camera_names),
        vision_backbone=backbone,
        hidden_dim=kwargs["hidden_dim"],
        condition_encoder="transformer",
        frozen_vision=True,
    )
    model = FlowMatchingPolicy(config).to(kwargs["device"])
    optimizer = torch.optim.AdamW(
        (parameter for parameter in model.parameters() if parameter.requires_grad),
        lr=float(kwargs["lr"]),
    )
    train_loader = _loader(
        kwargs["train_dataset"],
        kwargs["train_indices"],
        batch_size=kwargs["batch_size"],
        shuffle=True,
        torch=torch,
    )
    curves_path = kwargs["output_dir"] / "training_curves.jsonl"
    with curves_path.open("w", encoding="utf-8") as curves:
        for epoch in range(1, kwargs["epochs"] + 1):
            model.train()
            total = 0.0
            count = 0
            for batch in train_loader:
                batch = _to_device(batch, kwargs["device"], torch=torch)
                loss = flow_matching_loss(model, batch)
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
                optimizer.step()
                n = int(batch["action_chunk"].shape[0])
                total += float(loss.item()) * n
                count += n
            row = {"epoch": epoch, "train_loss": total / max(count, 1)}
            curves.write(json.dumps(row, sort_keys=True) + "\n")
            curves.flush()
    metrics = _evaluate_flow_model(
        model=model,
        dataset=kwargs["val_dataset"],
        indices=kwargs["val_indices"],
        stats=kwargs["stats"],
        device=kwargs["device"],
        sample_steps=kwargs["sample_steps"],
        torch=torch,
    )
    train_metrics = _evaluate_flow_model(
        model=model,
        dataset=kwargs["train_dataset"],
        indices=kwargs["train_indices"],
        stats=kwargs["stats"],
        device=kwargs["device"],
        sample_steps=kwargs["sample_steps"],
        torch=torch,
    )
    latency = _measure_flow_latency(
        model=model,
        dataset=kwargs["val_dataset"],
        stats=kwargs["stats"],
        device=kwargs["device"],
        sample_steps=kwargs["sample_steps"],
        torch=torch,
    )
    checkpoint = kwargs["output_dir"] / "checkpoint.pt"
    payload = {
        "schema": FLOW_CHECKPOINT_SCHEMA,
        "model_family": "flow_matching",
        "model_config": config.to_dict(),
        "dataset_stats": kwargs["stats"],
        "train_metrics": train_metrics,
        "validation_metrics": metrics,
        "model_state": model.cpu().state_dict(),
    }
    torch.save(payload, checkpoint)
    return {
        "model": name,
        "family": "flow_matching",
        "backbone": backbone,
        "checkpoint_path": str(checkpoint),
        "checkpoint_sha256": _sha256_file(checkpoint),
        "training_curves": str(curves_path),
        "train_metrics": train_metrics,
        "metrics": metrics,
        "latency": latency,
        "model_config": config.to_dict(),
        "shortcut_ablations": _flow_shortcut_ablations(
            model=model.to(kwargs["device"]),
            dataset=kwargs["val_dataset"],
            indices=kwargs["val_indices"],
            stats=kwargs["stats"],
            device=kwargs["device"],
            sample_steps=kwargs["sample_steps"],
            torch=torch,
        ),
    }


def _train_diffusion(*, name: str, backbone: str, **kwargs: Any) -> dict[str, Any]:
    torch = kwargs["torch"]
    model = _build_diffusion_policy(
        backbone=backbone,
        action_horizon=kwargs["train_dataset"].action_horizon,
        camera_count=len(kwargs["train_dataset"].camera_names),
        hidden_dim=kwargs["hidden_dim"],
        torch=torch,
    ).to(kwargs["device"])
    optimizer = torch.optim.AdamW(
        (parameter for parameter in model.parameters() if parameter.requires_grad),
        lr=float(kwargs["lr"]),
    )
    train_loader = _loader(
        kwargs["train_dataset"],
        kwargs["train_indices"],
        batch_size=kwargs["batch_size"],
        shuffle=True,
        torch=torch,
    )
    curves_path = kwargs["output_dir"] / "training_curves.jsonl"
    with curves_path.open("w", encoding="utf-8") as curves:
        for epoch in range(1, kwargs["epochs"] + 1):
            model.train()
            total = 0.0
            denom = 0.0
            for batch in train_loader:
                batch = _to_device(batch, kwargs["device"], torch=torch)
                loss = _diffusion_x0_loss(model=model, batch=batch, torch=torch)
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
                optimizer.step()
                n = float(batch["action_mask"].float().sum().item())
                total += float(loss.item()) * n
                denom += n
            curves.write(json.dumps({"epoch": epoch, "train_loss": total / max(denom, 1.0)}, sort_keys=True) + "\n")
            curves.flush()
    metrics = _evaluate_diffusion_model(
        model=model,
        dataset=kwargs["val_dataset"],
        indices=kwargs["val_indices"],
        stats=kwargs["stats"],
        device=kwargs["device"],
        sample_steps=kwargs["sample_steps"],
        torch=torch,
    )
    train_metrics = _evaluate_diffusion_model(
        model=model,
        dataset=kwargs["train_dataset"],
        indices=kwargs["train_indices"],
        stats=kwargs["stats"],
        device=kwargs["device"],
        sample_steps=kwargs["sample_steps"],
        torch=torch,
    )
    latency = _measure_diffusion_latency(
        model=model,
        dataset=kwargs["val_dataset"],
        device=kwargs["device"],
        sample_steps=kwargs["sample_steps"],
        torch=torch,
    )
    checkpoint = kwargs["output_dir"] / "checkpoint.pt"
    torch.save(
        {
            "schema": "robotics_lab.policy_runner.imitation_checkpoint.v1",
            "model_family": "diffusion_policy_x0",
            "backbone": backbone,
            "loss_name": "x0",
            "action_horizon": kwargs["train_dataset"].action_horizon,
            "action_dim": FLOW_ACTION_DIM,
            "proprio_dim": FLOW_PROPRIO_DIM,
            "camera_names": kwargs["train_dataset"].camera_names,
            "dataset_stats": kwargs["stats"],
            "train_metrics": train_metrics,
            "validation_metrics": metrics,
            "sample_steps": kwargs["sample_steps"],
            "model_state": model.cpu().state_dict(),
        },
        checkpoint,
    )
    return {
        "model": name,
        "family": "diffusion_policy_x0",
        "backbone": backbone,
        "loss_name": "x0",
        "checkpoint_path": str(checkpoint),
        "checkpoint_sha256": _sha256_file(checkpoint),
        "training_curves": str(curves_path),
        "train_metrics": train_metrics,
        "metrics": metrics,
        "latency": latency,
        "model_config": {
            "action_horizon": kwargs["train_dataset"].action_horizon,
            "action_dim": FLOW_ACTION_DIM,
            "proprio_dim": FLOW_PROPRIO_DIM,
            "camera_names": list(kwargs["train_dataset"].camera_names),
            "backbone": backbone,
            "hidden_dim": kwargs["hidden_dim"],
            "sample_steps": kwargs["sample_steps"],
        },
        "shortcut_ablations": _diffusion_shortcut_ablations(
            model=model.to(kwargs["device"]),
            dataset=kwargs["val_dataset"],
            indices=kwargs["val_indices"],
            stats=kwargs["stats"],
            device=kwargs["device"],
            sample_steps=kwargs["sample_steps"],
            torch=torch,
        ),
    }


def _train_supervised_model(
    *,
    model: Any,
    family: str,
    uses_images: bool,
    train_dataset: FlowHdf5Dataset,
    val_dataset: FlowHdf5Dataset,
    train_indices: list[int],
    val_indices: list[int],
    stats: dict[str, Any],
    output_dir: Path,
    batch_size: int,
    epochs: int,
    lr: float,
    hidden_dim: int,
    device: Any,
    torch: Any,
    model_name: str | None = None,
    backbone: str | None = None,
    loss_name: str = "mse",
    **_: Any,
) -> dict[str, Any]:
    optimizer = torch.optim.AdamW(
        (parameter for parameter in model.parameters() if parameter.requires_grad),
        lr=float(lr),
    )
    train_loader = _loader(train_dataset, train_indices, batch_size=batch_size, shuffle=True, torch=torch)
    curves_path = output_dir / "training_curves.jsonl"
    with curves_path.open("w", encoding="utf-8") as curves:
        for epoch in range(1, epochs + 1):
            model.train()
            total = 0.0
            denom = 0.0
            for batch in train_loader:
                batch = _to_device(batch, device, torch=torch)
                pred = model(batch["images"].float(), batch["proprio"].float())
                target = batch["action_chunk"].float()
                mask = batch["action_mask"].float()
                loss = _masked_supervised_loss(pred, target, mask, loss_name=loss_name)
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
                optimizer.step()
                n = float(mask.sum().item())
                total += float(loss.item()) * n
                denom += n
            curves.write(json.dumps({"epoch": epoch, "train_loss": total / max(denom, 1.0)}, sort_keys=True) + "\n")
            curves.flush()
    metrics = _evaluate_supervised_model(
        model=model,
        dataset=val_dataset,
        indices=val_indices,
        stats=stats,
        device=device,
        torch=torch,
    )
    train_metrics = _evaluate_supervised_model(
        model=model,
        dataset=train_dataset,
        indices=train_indices,
        stats=stats,
        device=device,
        torch=torch,
    )
    latency = _measure_supervised_latency(
        model=model,
        dataset=val_dataset,
        device=device,
        torch=torch,
    )
    checkpoint = output_dir / "checkpoint.pt"
    torch.save(
        {
            "schema": "robotics_lab.policy_runner.imitation_checkpoint.v1",
            "model_family": family,
            "backbone": backbone,
            "loss_name": loss_name,
            "action_horizon": train_dataset.action_horizon,
            "action_dim": FLOW_ACTION_DIM,
            "proprio_dim": FLOW_PROPRIO_DIM,
            "camera_names": train_dataset.camera_names,
            "dataset_stats": stats,
            "train_metrics": train_metrics,
            "validation_metrics": metrics,
            "model_state": model.cpu().state_dict(),
        },
        checkpoint,
    )
    result = {
        "model": model_name or family,
        "family": family,
        "backbone": backbone,
        "loss_name": loss_name,
        "checkpoint_path": str(checkpoint),
        "checkpoint_sha256": _sha256_file(checkpoint),
        "training_curves": str(curves_path),
        "train_metrics": train_metrics,
        "metrics": metrics,
        "latency": latency,
        "model_config": {
            "action_horizon": train_dataset.action_horizon,
            "action_dim": FLOW_ACTION_DIM,
            "proprio_dim": FLOW_PROPRIO_DIM,
            "camera_names": list(train_dataset.camera_names),
            "backbone": backbone,
            "hidden_dim": hidden_dim,
            "loss_name": loss_name,
            "uses_images": bool(uses_images),
        },
    }
    if uses_images:
        result["shortcut_ablations"] = _supervised_shortcut_ablations(
            model=model.to(device),
            dataset=val_dataset,
            indices=val_indices,
            stats=stats,
            device=device,
            torch=torch,
        )
    return result


def _masked_supervised_loss(pred: Any, target: Any, mask: Any, *, loss_name: str) -> Any:
    if loss_name == "mse":
        loss = (pred - target) ** 2
    elif loss_name == "l1":
        loss = (pred - target).abs()
    elif loss_name in {"huber", "smoothl1"}:
        import torch

        loss = torch.nn.functional.smooth_l1_loss(pred, target, reduction="none", beta=0.5)
    else:
        raise ValueError(f"unknown supervised loss: {loss_name}")
    return (loss * mask).sum() / mask.sum().clamp_min(1.0)


def _evaluate_constant_baseline(
    *,
    name: str,
    prediction: np.ndarray,
    train_dataset: FlowHdf5Dataset,
    val_dataset: FlowHdf5Dataset,
    train_indices: list[int],
    val_indices: list[int],
    stats: dict[str, Any],
    output_dir: Path,
) -> dict[str, Any]:
    pred_chunk = np.tile(prediction.reshape(1, -1), (val_dataset.action_horizon, 1)).astype(np.float32)
    train_summary = _evaluate_constant_prediction(
        pred_chunk=pred_chunk,
        dataset=train_dataset,
        indices=train_indices,
        stats=stats,
    )
    summary = _evaluate_constant_prediction(
        pred_chunk=pred_chunk,
        dataset=val_dataset,
        indices=val_indices,
        stats=stats,
    )
    output_path = output_dir / "constant_baseline.json"
    output_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {
        "model": name,
        "family": "constant_baseline",
        "checkpoint_path": None,
        "checkpoint_sha256": None,
        "train_metrics": train_summary,
        "metrics": summary,
        "latency": {"median_ms": 0.0, "p95_ms": 0.0},
        "artifact": str(output_path),
        "model_config": {
            "action_horizon": val_dataset.action_horizon,
            "action_dim": FLOW_ACTION_DIM,
            "prediction": "zero" if name == "zero" else "train_mean",
        },
    }


def _evaluate_constant_prediction(
    *,
    pred_chunk: np.ndarray,
    dataset: FlowHdf5Dataset,
    indices: list[int],
    stats: dict[str, Any],
) -> dict[str, Any]:
    metrics = _empty_metric_accumulator()
    for index in indices:
        sample = dataset.raw_sample(index)
        _accumulate_raw_metrics(
            metrics,
            pred_chunk,
            sample["action_chunk"],
            sample["action_mask"],
            phase=_sample_phase(dataset, index),
        )
    return _finalize_metrics(metrics, stats=stats)


def _evaluate_supervised_model(*, model: Any, dataset: FlowHdf5Dataset, indices: list[int], stats: dict[str, Any], device: Any, torch: Any) -> dict[str, Any]:
    model.eval()
    metrics = _empty_metric_accumulator()
    loader = _loader(dataset, indices, batch_size=64, shuffle=False, torch=torch, include_index=True)
    with torch.no_grad():
        for batch in loader:
            sample_indices = batch.pop("_sample_index").cpu().numpy().astype(int).tolist()
            batch = _to_device(batch, device, torch=torch)
            pred = model(batch["images"].float(), batch["proprio"].float())
            pred_raw = denormalize_actions(pred, stats).cpu().numpy()
            target_raw = denormalize_actions(batch["action_chunk"].float(), stats).cpu().numpy()
            mask = batch["action_mask"].float().cpu().numpy()
            for row, sample_index in enumerate(sample_indices):
                _accumulate_raw_metrics(
                    metrics,
                    pred_raw[row],
                    target_raw[row],
                    mask[row],
                    phase=_sample_phase(dataset, sample_index),
                )
    return _finalize_metrics(metrics, stats=stats)


def _evaluate_flow_model(*, model: Any, dataset: FlowHdf5Dataset, indices: list[int], stats: dict[str, Any], device: Any, sample_steps: int, torch: Any) -> dict[str, Any]:
    from .flow_model import sample_action_chunks

    model.eval()
    metrics = _empty_metric_accumulator()
    loader = _loader(dataset, indices, batch_size=32, shuffle=False, torch=torch, include_index=True)
    with torch.no_grad():
        for batch in loader:
            sample_indices = batch.pop("_sample_index").cpu().numpy().astype(int).tolist()
            batch = _to_device(batch, device, torch=torch)
            pred = sample_action_chunks(
                model,
                batch["images"].float(),
                batch["proprio"].float(),
                steps=sample_steps,
            )
            pred_raw = denormalize_actions(pred, stats).cpu().numpy()
            target_raw = denormalize_actions(batch["action_chunk"].float(), stats).cpu().numpy()
            mask = batch["action_mask"].float().cpu().numpy()
            for row, sample_index in enumerate(sample_indices):
                _accumulate_raw_metrics(
                    metrics,
                    pred_raw[row],
                    target_raw[row],
                    mask[row],
                    phase=_sample_phase(dataset, sample_index),
                )
    return _finalize_metrics(metrics, stats=stats)


def _evaluate_diffusion_model(*, model: Any, dataset: FlowHdf5Dataset, indices: list[int], stats: dict[str, Any], device: Any, sample_steps: int, torch: Any) -> dict[str, Any]:
    model.eval()
    metrics = _empty_metric_accumulator()
    loader = _loader(dataset, indices, batch_size=32, shuffle=False, torch=torch, include_index=True)
    with torch.no_grad():
        for batch in loader:
            sample_indices = batch.pop("_sample_index").cpu().numpy().astype(int).tolist()
            batch = _to_device(batch, device, torch=torch)
            pred = _sample_diffusion_x0(
                model=model,
                images=batch["images"].float(),
                proprio=batch["proprio"].float(),
                sample_steps=sample_steps,
                torch=torch,
            )
            pred_raw = denormalize_actions(pred, stats).cpu().numpy()
            target_raw = denormalize_actions(batch["action_chunk"].float(), stats).cpu().numpy()
            mask = batch["action_mask"].float().cpu().numpy()
            for row, sample_index in enumerate(sample_indices):
                _accumulate_raw_metrics(
                    metrics,
                    pred_raw[row],
                    target_raw[row],
                    mask[row],
                    phase=_sample_phase(dataset, sample_index),
                )
    return _finalize_metrics(metrics, stats=stats)


def _supervised_shortcut_ablations(*, model: Any, dataset: FlowHdf5Dataset, indices: list[int], stats: dict[str, Any], device: Any, torch: Any) -> dict[str, Any]:
    return {
        "zero_image": _evaluate_supervised_model_with_image_mode(
            model=model,
            dataset=dataset,
            indices=indices,
            stats=stats,
            device=device,
            torch=torch,
            image_mode="zero",
        ),
        "image_shuffle": _evaluate_supervised_model_with_image_mode(
            model=model,
            dataset=dataset,
            indices=indices,
            stats=stats,
            device=device,
            torch=torch,
            image_mode="shuffle",
        ),
    }


def _flow_shortcut_ablations(*, model: Any, dataset: FlowHdf5Dataset, indices: list[int], stats: dict[str, Any], device: Any, sample_steps: int, torch: Any) -> dict[str, Any]:
    return {
        "zero_image": _evaluate_flow_model_with_image_mode(
            model=model,
            dataset=dataset,
            indices=indices,
            stats=stats,
            device=device,
            sample_steps=sample_steps,
            torch=torch,
            image_mode="zero",
        ),
        "image_shuffle": _evaluate_flow_model_with_image_mode(
            model=model,
            dataset=dataset,
            indices=indices,
            stats=stats,
            device=device,
            sample_steps=sample_steps,
            torch=torch,
            image_mode="shuffle",
        ),
    }


def _diffusion_shortcut_ablations(*, model: Any, dataset: FlowHdf5Dataset, indices: list[int], stats: dict[str, Any], device: Any, sample_steps: int, torch: Any) -> dict[str, Any]:
    return {
        "zero_image": _evaluate_diffusion_model_with_image_mode(
            model=model,
            dataset=dataset,
            indices=indices,
            stats=stats,
            device=device,
            sample_steps=sample_steps,
            torch=torch,
            image_mode="zero",
        ),
        "image_shuffle": _evaluate_diffusion_model_with_image_mode(
            model=model,
            dataset=dataset,
            indices=indices,
            stats=stats,
            device=device,
            sample_steps=sample_steps,
            torch=torch,
            image_mode="shuffle",
        ),
    }


def _evaluate_supervised_model_with_image_mode(*, model: Any, dataset: FlowHdf5Dataset, indices: list[int], stats: dict[str, Any], device: Any, torch: Any, image_mode: str) -> dict[str, Any]:
    model.eval()
    metrics = _empty_metric_accumulator()
    loader = _loader(dataset, indices, batch_size=64, shuffle=False, torch=torch, include_index=True)
    with torch.no_grad():
        for batch in loader:
            sample_indices = batch.pop("_sample_index").cpu().numpy().astype(int).tolist()
            batch = _to_device(batch, device, torch=torch)
            batch["images"] = _alter_images(batch["images"], mode=image_mode, torch=torch)
            pred = model(batch["images"].float(), batch["proprio"].float())
            pred_raw = denormalize_actions(pred, stats).cpu().numpy()
            target_raw = denormalize_actions(batch["action_chunk"].float(), stats).cpu().numpy()
            mask = batch["action_mask"].float().cpu().numpy()
            for row, sample_index in enumerate(sample_indices):
                _accumulate_raw_metrics(metrics, pred_raw[row], target_raw[row], mask[row], phase=_sample_phase(dataset, sample_index))
    return _finalize_metrics(metrics, stats=stats)


def _evaluate_flow_model_with_image_mode(*, model: Any, dataset: FlowHdf5Dataset, indices: list[int], stats: dict[str, Any], device: Any, sample_steps: int, torch: Any, image_mode: str) -> dict[str, Any]:
    from .flow_model import sample_action_chunks

    model.eval()
    metrics = _empty_metric_accumulator()
    loader = _loader(dataset, indices, batch_size=32, shuffle=False, torch=torch, include_index=True)
    with torch.no_grad():
        for batch in loader:
            sample_indices = batch.pop("_sample_index").cpu().numpy().astype(int).tolist()
            batch = _to_device(batch, device, torch=torch)
            batch["images"] = _alter_images(batch["images"], mode=image_mode, torch=torch)
            pred = sample_action_chunks(model, batch["images"].float(), batch["proprio"].float(), steps=sample_steps)
            pred_raw = denormalize_actions(pred, stats).cpu().numpy()
            target_raw = denormalize_actions(batch["action_chunk"].float(), stats).cpu().numpy()
            mask = batch["action_mask"].float().cpu().numpy()
            for row, sample_index in enumerate(sample_indices):
                _accumulate_raw_metrics(metrics, pred_raw[row], target_raw[row], mask[row], phase=_sample_phase(dataset, sample_index))
    return _finalize_metrics(metrics, stats=stats)


def _evaluate_diffusion_model_with_image_mode(*, model: Any, dataset: FlowHdf5Dataset, indices: list[int], stats: dict[str, Any], device: Any, sample_steps: int, torch: Any, image_mode: str) -> dict[str, Any]:
    model.eval()
    metrics = _empty_metric_accumulator()
    loader = _loader(dataset, indices, batch_size=32, shuffle=False, torch=torch, include_index=True)
    with torch.no_grad():
        for batch in loader:
            sample_indices = batch.pop("_sample_index").cpu().numpy().astype(int).tolist()
            batch = _to_device(batch, device, torch=torch)
            batch["images"] = _alter_images(batch["images"], mode=image_mode, torch=torch)
            pred = _sample_diffusion_x0(
                model=model,
                images=batch["images"].float(),
                proprio=batch["proprio"].float(),
                sample_steps=sample_steps,
                torch=torch,
            )
            pred_raw = denormalize_actions(pred, stats).cpu().numpy()
            target_raw = denormalize_actions(batch["action_chunk"].float(), stats).cpu().numpy()
            mask = batch["action_mask"].float().cpu().numpy()
            for row, sample_index in enumerate(sample_indices):
                _accumulate_raw_metrics(metrics, pred_raw[row], target_raw[row], mask[row], phase=_sample_phase(dataset, sample_index))
    return _finalize_metrics(metrics, stats=stats)


def _alter_images(images: Any, *, mode: str, torch: Any) -> Any:
    if mode == "zero":
        return torch.zeros_like(images)
    if mode == "shuffle" and images.shape[0] > 1:
        return images[torch.randperm(images.shape[0], device=images.device)]
    return images


def _empty_metric_accumulator() -> dict[str, Any]:
    return {
        "sq_sum": 0.0,
        "count": 0.0,
        "translation_endpoint_sum": 0.0,
        "translation_endpoint_count": 0.0,
        "rotation_endpoint_sum": 0.0,
        "rotation_endpoint_count": 0.0,
        "gripper_sq_sum": 0.0,
        "gripper_count": 0.0,
        "inactive_leakage_sum": 0.0,
        "inactive_leakage_count": 0.0,
        "smoothness_sum": 0.0,
        "smoothness_count": 0.0,
        "phase": {
            phase: {"sq_sum": 0.0, "count": 0.0}
            for phase in PHASE_NAMES
        },
    }


def _accumulate_raw_metrics(metrics: dict[str, Any], pred: np.ndarray, target: np.ndarray, mask: np.ndarray, *, phase: str) -> None:
    error = (pred - target).astype(np.float64)
    active = mask.astype(np.float64)
    metrics["sq_sum"] += float(((error * error) * active).sum())
    metrics["count"] += float(active.sum())
    normalized_phase = metrics["phase"][phase]
    normalized_phase["sq_sum"] += float(((error * error) * active).sum())
    normalized_phase["count"] += float(active.sum())
    for arm_start in (0, 7):
        if active[-1, arm_start : arm_start + 6].sum() <= 0.0:
            continue
        metrics["translation_endpoint_sum"] += float(np.linalg.norm(error[-1, arm_start : arm_start + 3]))
        metrics["translation_endpoint_count"] += 1.0
        metrics["rotation_endpoint_sum"] += float(np.linalg.norm(error[-1, arm_start + 3 : arm_start + 6]))
        metrics["rotation_endpoint_count"] += 1.0
    grip_mask = np.zeros_like(active)
    grip_mask[:, 6] = active[:, 6]
    grip_mask[:, 13] = active[:, 13]
    metrics["gripper_sq_sum"] += float(((error * error) * grip_mask).sum())
    metrics["gripper_count"] += float(grip_mask.sum())
    if phase in {"right_pick", "right_place"}:
        leakage = np.linalg.norm(pred[:, 0:6], axis=-1)
        metrics["inactive_leakage_sum"] += float(leakage.sum())
        metrics["inactive_leakage_count"] += float(leakage.size)
    elif phase in {"left_pick", "left_place"}:
        leakage = np.linalg.norm(pred[:, 7:13], axis=-1)
        metrics["inactive_leakage_sum"] += float(leakage.sum())
        metrics["inactive_leakage_count"] += float(leakage.size)
    if pred.shape[0] > 1:
        diffs = np.diff(pred[:, :14], axis=0)
        metrics["smoothness_sum"] += float(np.linalg.norm(diffs, axis=-1).sum())
        metrics["smoothness_count"] += float(diffs.shape[0])


def _finalize_metrics(metrics: dict[str, Any], *, stats: dict[str, Any]) -> dict[str, Any]:
    raw_action_mse = metrics["sq_sum"] / max(metrics["count"], 1.0)
    action_std = np.asarray(stats["action_std"], dtype=np.float64)
    scale = float(np.mean(np.square(np.maximum(action_std, 1e-12))))
    by_phase = {}
    for phase, values in metrics["phase"].items():
        mse = values["sq_sum"] / max(values["count"], 1.0)
        by_phase[phase] = {
            "action_mse": mse,
            "normalized_action_mse": mse / max(scale, 1e-12),
        }
    return {
        "action_mse": raw_action_mse,
        "normalized_action_mse": raw_action_mse / max(scale, 1e-12),
        "translation_endpoint_error": metrics["translation_endpoint_sum"] / max(metrics["translation_endpoint_count"], 1.0),
        "rotation_endpoint_error": metrics["rotation_endpoint_sum"] / max(metrics["rotation_endpoint_count"], 1.0),
        "gripper_mse": metrics["gripper_sq_sum"] / max(metrics["gripper_count"], 1.0),
        "inactive_arm_leakage": metrics["inactive_leakage_sum"] / max(metrics["inactive_leakage_count"], 1.0),
        "smoothness": metrics["smoothness_sum"] / max(metrics["smoothness_count"], 1.0),
        "by_phase": by_phase,
    }


class _IndexedDataset:
    def __init__(self, dataset: FlowHdf5Dataset, indices: list[int], *, include_index: bool):
        self.dataset = dataset
        self.indices = indices
        self.include_index = include_index

    def __len__(self) -> int:
        return len(self.indices)

    def __getitem__(self, index: int) -> dict[str, Any]:
        sample_index = self.indices[index]
        sample = dict(self.dataset[sample_index])
        if self.include_index:
            sample["_sample_index"] = np.asarray(sample_index, dtype=np.int64)
        return sample


def _loader(dataset: FlowHdf5Dataset, indices: list[int], *, batch_size: int, shuffle: bool, torch: Any, include_index: bool = False) -> Any:
    from torch.utils.data import DataLoader

    return DataLoader(
        _IndexedDataset(dataset, indices, include_index=include_index),
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=2,
        pin_memory=bool(torch.cuda.is_available()),
    )


def _build_state_mlp(*, action_horizon: int, hidden_dim: int) -> Any:
    from torch import nn

    class StateMlp(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.action_horizon = int(action_horizon)
            self.net = nn.Sequential(
                nn.Linear(FLOW_PROPRIO_DIM, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, self.action_horizon * FLOW_ACTION_DIM),
            )

        def forward(self, images: Any, proprio: Any) -> Any:
            del images
            out = self.net(proprio)
            return out.reshape(out.shape[0], self.action_horizon, FLOW_ACTION_DIM)

    return StateMlp()


def _build_direct_bc_policy(*, backbone: str, action_horizon: int, camera_count: int, hidden_dim: int) -> Any:
    import torch
    from torch import nn
    from .flow_model import VisionBackbone

    class DirectBcPolicy(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.action_horizon = int(action_horizon)
            self.vision = VisionBackbone(backbone, hidden_dim, frozen=True)
            self.camera_embedding = nn.Embedding(max(camera_count, 1), hidden_dim)
            self.proprio = nn.Sequential(
                nn.Linear(FLOW_PROPRIO_DIM, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
            self.head = nn.Sequential(
                nn.Linear(hidden_dim * 2, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, self.action_horizon * FLOW_ACTION_DIM),
            )

        def forward(self, images: Any, proprio: Any) -> Any:
            batch = proprio.shape[0]
            prop = self.proprio(proprio)
            if images.shape[1] > 0:
                flat = images.reshape(batch * images.shape[1], *images.shape[2:])
                vision = self.vision(flat).reshape(batch, images.shape[1], -1)
                cam_ids = torch.arange(images.shape[1], device=images.device).clamp_max(
                    self.camera_embedding.num_embeddings - 1
                )
                vision = vision + self.camera_embedding(cam_ids)[None, :, :]
                pooled = vision.mean(dim=1)
            else:
                pooled = prop.new_zeros((batch, prop.shape[-1]))
            out = self.head(torch.cat([prop, pooled], dim=-1))
            return out.reshape(batch, self.action_horizon, FLOW_ACTION_DIM)

    return DirectBcPolicy()


def _build_structured_direct_policy(*, backbone: str, action_horizon: int, camera_count: int, hidden_dim: int) -> Any:
    import torch
    from torch import nn
    from .flow_model import VisionBackbone

    class StructuredDirectPolicy(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.action_horizon = int(action_horizon)
            self.vision = VisionBackbone(backbone, hidden_dim, frozen=True)
            self.camera_embedding = nn.Embedding(max(camera_count, 1), hidden_dim)
            self.proprio = nn.Sequential(
                nn.Linear(FLOW_PROPRIO_DIM, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
            self.arm_token = nn.Embedding(2, hidden_dim)
            self.shared = nn.Sequential(
                nn.Linear(hidden_dim * 2, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.GELU(),
            )
            self.left_head = nn.Linear(hidden_dim, self.action_horizon * 7)
            self.right_head = nn.Linear(hidden_dim, self.action_horizon * 7)
            self.gripper_head = nn.Sequential(
                nn.Linear(hidden_dim * 2, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, self.action_horizon * 2),
            )

        def forward(self, images: Any, proprio: Any) -> Any:
            batch = proprio.shape[0]
            prop = self.proprio(proprio)
            if images.shape[1] > 0:
                flat = images.reshape(batch * images.shape[1], *images.shape[2:])
                vision = self.vision(flat).reshape(batch, images.shape[1], -1)
                cam_ids = torch.arange(images.shape[1], device=images.device).clamp_max(
                    self.camera_embedding.num_embeddings - 1
                )
                pooled = (vision + self.camera_embedding(cam_ids)[None, :, :]).mean(dim=1)
            else:
                pooled = prop.new_zeros((batch, prop.shape[-1]))
            trunk = self.shared(torch.cat([prop, pooled], dim=-1))
            left_token = trunk + self.arm_token.weight[0][None, :]
            right_token = trunk + self.arm_token.weight[1][None, :]
            left = self.left_head(left_token).reshape(batch, self.action_horizon, 7)
            right = self.right_head(right_token).reshape(batch, self.action_horizon, 7)
            grippers = self.gripper_head(torch.cat([left_token, right_token], dim=-1)).reshape(
                batch,
                self.action_horizon,
                2,
            )
            left = left.clone()
            right = right.clone()
            left[:, :, 6] = grippers[:, :, 0]
            right[:, :, 6] = grippers[:, :, 1]
            return torch.cat([left, right], dim=-1)

    return StructuredDirectPolicy()


def _build_act_chunk_policy(*, backbone: str, action_horizon: int, camera_count: int, hidden_dim: int) -> Any:
    import torch
    from torch import nn
    from .flow_model import VisionBackbone

    class ActChunkPolicy(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.action_horizon = int(action_horizon)
            self.vision = VisionBackbone(backbone, hidden_dim, frozen=True)
            self.camera_embedding = nn.Embedding(max(camera_count, 1), hidden_dim)
            self.proprio = nn.Linear(FLOW_PROPRIO_DIM, hidden_dim)
            self.query = nn.Embedding(self.action_horizon, hidden_dim)
            heads = max(1, min(8, hidden_dim // 32))
            layer = nn.TransformerEncoderLayer(
                d_model=hidden_dim,
                nhead=heads,
                dim_feedforward=hidden_dim * 4,
                dropout=0.0,
                activation="gelu",
                batch_first=True,
                norm_first=True,
            )
            self.transformer = nn.TransformerEncoder(layer, num_layers=3)
            self.head = nn.Linear(hidden_dim, FLOW_ACTION_DIM)

        def forward(self, images: Any, proprio: Any) -> Any:
            batch = proprio.shape[0]
            tokens = [self.proprio(proprio).unsqueeze(1)]
            if images.shape[1] > 0:
                flat = images.reshape(batch * images.shape[1], *images.shape[2:])
                vision = self.vision(flat).reshape(batch, images.shape[1], -1)
                cam_ids = torch.arange(images.shape[1], device=images.device).clamp_max(
                    self.camera_embedding.num_embeddings - 1
                )
                tokens.append(vision + self.camera_embedding(cam_ids)[None, :, :])
            query_ids = torch.arange(self.action_horizon, device=images.device)
            tokens.append(self.query(query_ids)[None, :, :].expand(batch, -1, -1))
            encoded = self.transformer(torch.cat(tokens, dim=1))
            return self.head(encoded[:, -self.action_horizon :, :])

    return ActChunkPolicy()


def _build_diffusion_policy(*, backbone: str, action_horizon: int, camera_count: int, hidden_dim: int, torch: Any) -> Any:
    from torch import nn
    from .flow_model import SinusoidalTimeEmbedding, VisionBackbone

    class DiffusionChunkPolicy(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.action_horizon = int(action_horizon)
            self.action_dim = FLOW_ACTION_DIM
            self.diffusion_train_steps = 100
            self.vision = VisionBackbone(backbone, hidden_dim, frozen=True)
            self.camera_embedding = nn.Embedding(max(camera_count, 1), hidden_dim)
            self.proprio = nn.Sequential(
                nn.Linear(FLOW_PROPRIO_DIM, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
            self.condition = nn.Sequential(
                nn.Linear(hidden_dim * 2, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
            self.time_embedding = nn.Sequential(
                SinusoidalTimeEmbedding(hidden_dim),
                nn.Linear(hidden_dim, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
            self.step_embedding = nn.Embedding(self.action_horizon, hidden_dim)
            self.denoiser = nn.Sequential(
                nn.Linear(FLOW_ACTION_DIM + hidden_dim, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.GELU(),
                nn.Linear(hidden_dim, FLOW_ACTION_DIM),
            )

        def encode_condition(self, images: Any, proprio: Any) -> Any:
            batch = proprio.shape[0]
            prop = self.proprio(proprio)
            if images.shape[1] > 0:
                flat = images.reshape(batch * images.shape[1], *images.shape[2:])
                vision = self.vision(flat).reshape(batch, images.shape[1], -1)
                cam_ids = torch.arange(images.shape[1], device=images.device).clamp_max(
                    self.camera_embedding.num_embeddings - 1
                )
                pooled = (vision + self.camera_embedding(cam_ids)[None, :, :]).mean(dim=1)
            else:
                pooled = prop.new_zeros((batch, prop.shape[-1]))
            return self.condition(torch.cat([prop, pooled], dim=-1))

        def forward(self, images: Any, proprio: Any, x_t: Any, t: Any) -> Any:
            if x_t.ndim != 3:
                raise ValueError("x_t must be B,H,A")
            if x_t.shape[1] != self.action_horizon:
                raise ValueError("x_t horizon does not match model config")
            cond = self.encode_condition(images, proprio) + self.time_embedding(t)
            step_ids = torch.arange(self.action_horizon, device=x_t.device)
            step_tokens = self.step_embedding(step_ids)[None, :, :] + cond[:, None, :]
            return self.denoiser(torch.cat([x_t, step_tokens], dim=-1))

    return DiffusionChunkPolicy()


def _diffusion_alpha_bar(train_steps: int, *, device: Any, dtype: Any, torch: Any) -> Any:
    betas = torch.linspace(1e-4, 0.02, int(train_steps), device=device, dtype=dtype)
    return torch.cumprod(1.0 - betas, dim=0)


def _diffusion_x0_loss(*, model: Any, batch: dict[str, Any], torch: Any) -> Any:
    actions = batch["action_chunk"].float()
    mask = batch.get("action_mask")
    if mask is None:
        mask = torch.ones_like(actions)
    else:
        mask = mask.float()
    batch_size = int(actions.shape[0])
    t_idx = torch.randint(
        0,
        int(model.diffusion_train_steps),
        (batch_size,),
        device=actions.device,
    )
    alpha_bar = _diffusion_alpha_bar(
        int(model.diffusion_train_steps),
        device=actions.device,
        dtype=actions.dtype,
        torch=torch,
    )[t_idx]
    noise = torch.randn_like(actions)
    x_t = alpha_bar.sqrt()[:, None, None] * actions + (1.0 - alpha_bar).sqrt()[:, None, None] * noise
    t = t_idx.to(dtype=actions.dtype) / float(max(1, int(model.diffusion_train_steps) - 1))
    pred_x0 = model(batch["images"].float(), batch["proprio"].float(), x_t, t)
    loss = ((pred_x0 - actions) ** 2) * mask
    return loss.sum() / mask.sum().clamp_min(1.0)


def _sample_diffusion_x0(*, model: Any, images: Any, proprio: Any, sample_steps: int, torch: Any) -> Any:
    if sample_steps <= 0:
        raise ValueError("sample_steps must be positive")
    model.eval()
    batch_size = int(proprio.shape[0])
    train_steps = int(model.diffusion_train_steps)
    alpha_bars = _diffusion_alpha_bar(
        train_steps,
        device=proprio.device,
        dtype=proprio.dtype,
        torch=torch,
    )
    x = torch.randn(
        (batch_size, int(model.action_horizon), FLOW_ACTION_DIM),
        dtype=proprio.dtype,
        device=proprio.device,
    )
    indices = torch.linspace(
        train_steps - 1,
        0,
        steps=min(int(sample_steps), train_steps),
        device=proprio.device,
    ).round().long()
    indices = torch.unique_consecutive(indices)
    for i, t_idx in enumerate(indices):
        t_int = int(t_idx.item())
        alpha_t = alpha_bars[t_int]
        t = torch.full(
            (batch_size,),
            float(t_int) / float(max(1, train_steps - 1)),
            dtype=proprio.dtype,
            device=proprio.device,
        )
        pred_x0 = model(images, proprio, x, t)
        if i == len(indices) - 1:
            x = pred_x0
            continue
        next_t = int(indices[i + 1].item())
        alpha_next = alpha_bars[next_t]
        eps = (x - alpha_t.sqrt() * pred_x0) / (1.0 - alpha_t).sqrt().clamp_min(1e-6)
        x = alpha_next.sqrt() * pred_x0 + (1.0 - alpha_next).sqrt() * eps
    return x


def _measure_supervised_latency(*, model: Any, dataset: FlowHdf5Dataset, device: Any, torch: Any) -> dict[str, float]:
    model.eval()
    if len(dataset) == 0:
        return {"median_ms": 0.0, "p95_ms": 0.0}
    sample = dataset[0]
    images = torch.as_tensor(sample["images"])[None].to(device)
    proprio = torch.as_tensor(sample["proprio"])[None].to(device)
    return _latency_loop(lambda: model(images.float(), proprio.float()), torch=torch, device=device)


def _measure_flow_latency(*, model: Any, dataset: FlowHdf5Dataset, stats: dict[str, Any], device: Any, sample_steps: int, torch: Any) -> dict[str, float]:
    from .flow_model import sample_action_chunks

    del stats
    model.eval()
    if len(dataset) == 0:
        return {"median_ms": 0.0, "p95_ms": 0.0}
    sample = dataset[0]
    images = torch.as_tensor(sample["images"])[None].to(device)
    proprio = torch.as_tensor(sample["proprio"])[None].to(device)
    return _latency_loop(
        lambda: sample_action_chunks(model, images.float(), proprio.float(), steps=sample_steps),
        torch=torch,
        device=device,
    )


def _measure_diffusion_latency(*, model: Any, dataset: FlowHdf5Dataset, device: Any, sample_steps: int, torch: Any) -> dict[str, float]:
    model.eval()
    if len(dataset) == 0:
        return {"median_ms": 0.0, "p95_ms": 0.0}
    sample = dataset[0]
    images = torch.as_tensor(sample["images"])[None].to(device)
    proprio = torch.as_tensor(sample["proprio"])[None].to(device)
    return _latency_loop(
        lambda: _sample_diffusion_x0(
            model=model,
            images=images.float(),
            proprio=proprio.float(),
            sample_steps=sample_steps,
            torch=torch,
        ),
        torch=torch,
        device=device,
    )


def _latency_loop(fn: Any, *, torch: Any, device: Any) -> dict[str, float]:
    elapsed: list[float] = []
    with torch.no_grad():
        for _ in range(5):
            fn()
        if str(device).startswith("cuda"):
            torch.cuda.synchronize(device)
        for _ in range(30):
            start = time.perf_counter()
            fn()
            if str(device).startswith("cuda"):
                torch.cuda.synchronize(device)
            elapsed.append((time.perf_counter() - start) * 1000.0)
    return {
        "median_ms": float(np.percentile(elapsed, 50)),
        "p95_ms": float(np.percentile(elapsed, 95)),
    }


def _to_device(batch: dict[str, Any], device: Any, *, torch: Any) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in batch.items():
        tensor = value if hasattr(value, "to") else torch.as_tensor(value)
        out[key] = tensor.to(device)
    return out


def _sample_phase(dataset: FlowHdf5Dataset, sample_index: int) -> str:
    ref = dataset.sample_refs[sample_index]
    episode = dataset.episodes[ref.episode_index]
    denom = max(1, episode.length - dataset.action_horizon)
    fraction = min(0.999999, max(0.0, float(ref.start) / float(denom)))
    return PHASE_NAMES[min(3, int(fraction * 4.0))]


def _limited_indices(length: int, limit: int | None) -> list[int]:
    indices = list(range(length))
    if limit is not None:
        return indices[: max(1, min(length, int(limit)))]
    return indices


def _session_stratified_split(episodes: list[dict[str, Any]], *, val_ratio: float, seed: int) -> tuple[list[dict[str, Any]], list[dict[str, Any]], str]:
    rng = np.random.default_rng(seed)
    by_session: dict[str, list[dict[str, Any]]] = {}
    for episode in episodes:
        by_session.setdefault(str(episode["session"]), []).append(episode)
    if all(len(items) >= 2 for items in by_session.values()):
        train: list[dict[str, Any]] = []
        val: list[dict[str, Any]] = []
        for session, items in sorted(by_session.items()):
            order = list(sorted(items, key=lambda item: item["path"]))
            rng.shuffle(order)
            val_count = max(1, int(round(len(order) * val_ratio)))
            val_count = min(val_count, len(order) - 1)
            val.extend(order[:val_count])
            train.extend(order[val_count:])
        return _split_entries(train), _split_entries(val), "session_stratified_episode"
    order = list(sorted(episodes, key=lambda item: item["path"]))
    rng.shuffle(order)
    val_count = max(1, int(round(len(order) * val_ratio)))
    val_count = min(val_count, len(order) - 1)
    return _split_entries(order[val_count:]), _split_entries(order[:val_count]), "global_episode_fallback_sessions_too_small"


def _session_holdout_split(
    episodes: list[dict[str, Any]],
    *,
    val_ratio: float,
    seed: int,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[str]]:
    sessions = sorted({str(episode["session"]) for episode in episodes})
    if len(sessions) < 3:
        return [], [], []
    rng = np.random.default_rng(seed)
    shuffled = list(sessions)
    rng.shuffle(shuffled)
    holdout_count = max(1, int(round(len(shuffled) * val_ratio)))
    holdout_count = min(holdout_count, len(shuffled) - 1)
    held = set(shuffled[:holdout_count])
    train = [episode for episode in episodes if str(episode["session"]) not in held]
    val = [episode for episode in episodes if str(episode["session"]) in held]
    return _split_entries(train), _split_entries(val), sorted(held)


def _split_entries(episodes: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "path": episode["path"],
            "relative_path": episode.get("relative_path", ""),
            "session": episode["session"],
            "sha256": episode["sha256"],
            "frame_count": episode["frame_count"],
            "detected_format": episode["detected_format"],
            "detected_cameras": list(episode.get("detected_cameras", [])),
        }
        for episode in sorted(episodes, key=lambda item: item["path"])
    ]


def _build_report(
    *,
    output_dir: Path,
    snapshot: dict[str, Any],
    split: dict[str, Any],
    effective_split: dict[str, Any],
    stats: dict[str, Any],
    stats_path: Path,
    results: list[dict[str, Any]],
    args: dict[str, Any],
) -> dict[str, Any]:
    successful_results = [
        item
        for item in results
        if item.get("status", "completed") != "failed"
        and "normalized_action_mse" in item.get("metrics", {})
    ]
    failed_results = [item for item in results if item.get("status") == "failed"]
    leaderboard = sorted(
        successful_results,
        key=lambda item: float(item["metrics"].get("normalized_action_mse", math.inf)),
    )
    best = leaderboard[0] if leaderboard else None
    zero = _find_result(results, "zero")
    mean = _find_result(results, "train_mean")
    state = _find_result(results, "state_mlp")
    visual_warning = None
    if best and state and best.get("family") != "state_only_mlp":
        best_mse = float(best["metrics"].get("normalized_action_mse", math.inf))
        state_mse = float(state["metrics"].get("normalized_action_mse", math.inf))
        if best_mse >= state_mse:
            visual_warning = "best image+state model does not outperform state-only; do not claim visual grounding"
    return {
        "schema": IMITATION_REPORT_SCHEMA,
        "created_at": _utc_now(),
        "output_dir": str(output_dir),
        "dataset": {
            "snapshot_path": str(snapshot.get("_source_path", output_dir / "dataset_snapshot.json")),
            "snapshot_hash": snapshot["snapshot_hash"],
            "valid_episode_count": snapshot["aggregate"]["valid_episode_count"],
            "frame_count": snapshot["aggregate"]["frame_count"],
            "sessions": snapshot["aggregate"]["sessions"],
            "detected_cameras": snapshot["aggregate"]["detected_cameras"],
            "detected_formats": snapshot["aggregate"]["detected_formats"],
        },
        "split": {
            "split_path": str(split.get("_source_path", output_dir / "split_manifest.json")),
            "split_hash": split["split_hash"],
            "split_mode": effective_split["split_mode"],
            "train_episode_count": len(effective_split["train"]),
            "validation_episode_count": len(effective_split["validation"]),
            "primary_train_episode_count": len(split["train"]),
            "primary_validation_episode_count": len(split["validation"]),
            "session_holdout_val_episode_count": len(split.get("session_holdout_val", [])),
            "session_holdout_train_episode_count": len(
                effective_split.get("session_holdout_train", split.get("session_holdout_train", []))
            ),
            "session_holdout_sessions": list(
                effective_split.get("session_holdout_sessions", split.get("session_holdout_sessions", []))
            ),
            "split_algorithm": split["split_algorithm"],
            "effective_split_algorithm": effective_split["split_algorithm"],
            "val_ratio": split["val_ratio"],
        },
        "normalization": {
            "path": str(stats_path),
            "source": "train_only",
            "train_episode_count": stats["episode_count"],
            "sample_count": stats["sample_count"],
            "action_dim_names": list(FLOW_ACTION_DIM_NAMES),
        },
        "args": args,
        "leaderboard": [
            {
                "rank": rank,
                "model": item["model"],
                "family": item["family"],
                "backbone": item.get("backbone"),
                "train_normalized_action_mse": item.get("train_metrics", {}).get("normalized_action_mse"),
                "normalized_action_mse": item["metrics"]["normalized_action_mse"],
                "action_mse": item["metrics"]["action_mse"],
                "translation_endpoint_error": item["metrics"]["translation_endpoint_error"],
                "rotation_endpoint_error": item["metrics"]["rotation_endpoint_error"],
                "gripper_mse": item["metrics"]["gripper_mse"],
                "inactive_arm_leakage": item["metrics"]["inactive_arm_leakage"],
                "latency": item.get("latency", {}),
                "checkpoint_sha256": item.get("checkpoint_sha256"),
                "wall_time_sec": item.get("wall_time_sec"),
                "loss_name": item.get("loss_name"),
                "split_mode": effective_split["split_mode"],
            }
            for rank, item in enumerate(leaderboard, start=1)
        ],
        "failed_runs": failed_results,
        "results": results,
        "baseline_comparison": {
            "best_minus_zero_normalized_mse": _metric_delta(best, zero, "normalized_action_mse"),
            "best_minus_train_mean_normalized_mse": _metric_delta(best, mean, "normalized_action_mse"),
            "best_minus_state_only_normalized_mse": _metric_delta(best, state, "normalized_action_mse"),
        },
        "warnings": [
            warning
            for warning in [
                visual_warning,
                _primary_holdout_overlap_warning(split, effective_split["split_mode"]),
            ]
            if warning
        ],
        "recommendation": _recommendation(best, state),
        "safety": {
            "real_mode_behavior_touched": False,
            "robot_motion_commands_sent": False,
            "scope": "offline HDF5 imitation-learning experiment only",
        },
    }


def _render_report_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Imitation Learning Leaderboard",
        "",
        "## Dataset",
        "",
        f"- Snapshot hash: `{report['dataset']['snapshot_hash']}`",
        f"- Episodes: {report['dataset']['valid_episode_count']}",
        f"- Frames: {report['dataset']['frame_count']}",
        f"- Sessions: {', '.join(report['dataset']['sessions'])}",
        f"- Cameras: {', '.join(report['dataset']['detected_cameras'])}",
        f"- Split mode: `{report['split']['split_mode']}`",
        f"- Split: {report['split']['effective_split_algorithm']} train={report['split']['train_episode_count']} val={report['split']['validation_episode_count']}",
        f"- Normalization: train-only, samples={report['normalization']['sample_count']}",
        "",
        "## Leaderboard",
        "",
        "| Rank | Model | Split | Family | Backbone | Train Norm MSE | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |",
        "| ---: | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in report["leaderboard"]:
        latency = row.get("latency", {})
        lines.append(
            f"| {row['rank']} | {row['model']} | {row.get('split_mode', '')} | "
            f"{row['family']} | {row.get('backbone') or ''} | "
            f"{_format_optional_float(row.get('train_normalized_action_mse'))} | "
            f"{row['normalized_action_mse']:.6g} | {row['action_mse']:.6g} | "
            f"{row['translation_endpoint_error']:.6g} | {row['rotation_endpoint_error']:.6g} | "
            f"{row['gripper_mse']:.6g} | {float(latency.get('median_ms', 0.0)):.4g} | "
            f"`{row.get('checkpoint_sha256') or ''}` |"
        )
    lines.extend(["", "## Recommendation", "", report["recommendation"], ""])
    if report["warnings"]:
        lines.extend(["## Warnings", ""])
        lines.extend(f"- {warning}" for warning in report["warnings"])
        lines.append("")
    if report.get("failed_runs"):
        lines.extend(["## Failed Runs", ""])
        for failed in report["failed_runs"]:
            lines.append(
                f"- `{failed.get('model', '')}`: {failed.get('error_type', 'Error')}: {failed.get('error', '')}"
            )
        lines.append("")
    lines.extend(
        [
            "## Safety",
            "",
            "- Offline HDF5 training/evaluation only.",
            "- No real robot mode, robot connection, or robot motion behavior was touched.",
        ]
    )
    return "\n".join(lines).rstrip() + "\n"


def _effective_split_for_mode(
    snapshot: dict[str, Any],
    split: dict[str, Any],
    *,
    split_mode: str,
) -> dict[str, Any]:
    if split_mode == "primary":
        effective = dict(split)
        effective["split_mode"] = "primary"
        effective["split_algorithm"] = split.get("split_algorithm", "primary_episode_split")
        return effective
    if split_mode != "session_holdout":
        raise ValueError("split_mode must be 'primary' or 'session_holdout'")

    holdout_val = list(split.get("session_holdout_val", []))
    if not holdout_val:
        raise ValueError("split manifest has no session_holdout_val entries")
    holdout_sessions = sorted(
        set(split.get("session_holdout_sessions", []))
        or {str(entry["session"]) for entry in holdout_val}
    )
    if split.get("session_holdout_train"):
        holdout_train = list(split["session_holdout_train"])
    else:
        held = set(holdout_sessions)
        holdout_train = [
            episode
            for episode in snapshot.get("episodes", [])
            if str(episode.get("session", "")) not in held
        ]
        holdout_train = _split_entries(holdout_train)
    train_paths = {entry["path"] for entry in holdout_train}
    val_paths = {entry["path"] for entry in holdout_val}
    overlap = sorted(train_paths & val_paths)
    if overlap:
        raise ValueError("session-holdout train/validation overlap detected: " + ", ".join(overlap[:5]))
    train_sessions = {str(entry["session"]) for entry in holdout_train}
    val_sessions = {str(entry["session"]) for entry in holdout_val}
    session_overlap = sorted(train_sessions & val_sessions)
    if session_overlap:
        raise ValueError("session-holdout session overlap detected: " + ", ".join(session_overlap))
    effective = dict(split)
    effective["train"] = holdout_train
    effective["validation"] = holdout_val
    effective["session_holdout_train"] = holdout_train
    effective["session_holdout_sessions"] = holdout_sessions
    effective["split_mode"] = "session_holdout"
    effective["split_algorithm"] = "session_holdout_by_folder"
    return effective


def _primary_holdout_overlap_warning(split: dict[str, Any], split_mode: str) -> str | None:
    if split_mode != "primary":
        return None
    train = {entry["path"] for entry in split.get("train", [])}
    holdout = {entry["path"] for entry in split.get("session_holdout_val", [])}
    overlap = train & holdout
    if not overlap:
        return None
    return (
        "primary split training includes episodes from session_holdout_val; "
        "use --split-mode session_holdout for strict held-session evaluation"
    )


def aggregate_imitation_reports(
    *,
    input_dirs: Iterable[str | Path],
    output_dir: str | Path,
    label: str = "combined",
) -> AggregateResult:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    report_paths: list[Path] = []
    for entry in input_dirs:
        path = Path(entry)
        if path.is_file():
            report_paths.append(path)
        else:
            report_paths.extend(sorted(path.rglob("leaderboard_summary.json")))
    unique_paths = sorted(dict.fromkeys(report_paths))
    rows: list[dict[str, Any]] = []
    failed: list[dict[str, Any]] = []
    metadata: dict[str, Any] = {}
    for path in unique_paths:
        try:
            report = _load_json(path)
        except Exception as exc:  # noqa: BLE001
            failed.append(
                {
                    "model": str(path),
                    "family": "report_load_failed",
                    "status": "failed",
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                    "source_report": str(path),
                }
            )
            continue
        if not metadata:
            metadata = {
                "snapshot_hash": report.get("dataset", {}).get("snapshot_hash", ""),
                "split_hash": report.get("split", {}).get("split_hash", ""),
                "split_mode": report.get("split", {}).get("split_mode", "primary"),
                "train_episode_count": report.get("split", {}).get("train_episode_count", 0),
                "validation_episode_count": report.get("split", {}).get("validation_episode_count", 0),
            }
        for row in report.get("leaderboard", []):
            item = dict(row)
            item["source_report"] = str(path)
            item.setdefault("split_mode", report.get("split", {}).get("split_mode", "primary"))
            rows.append(item)
        for item in report.get("failed_runs", []):
            failed_item = dict(item)
            failed_item["source_report"] = str(path)
            failed.append(failed_item)
    rows.sort(key=lambda item: float(item.get("normalized_action_mse", math.inf)))
    for rank, row in enumerate(rows, start=1):
        row["rank"] = rank
    payload = {
        "schema": "robotics_lab.policy_runner.imitation_combined_leaderboard.v1",
        "created_at": _utc_now(),
        "label": label,
        "source_reports": [str(path) for path in unique_paths],
        "rows": rows,
        "failed_runs": failed,
        **metadata,
    }
    summary_path = output / "combined_leaderboard.json"
    report_path = output / "combined_leaderboard.md"
    summary_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    report_path.write_text(_render_combined_report(payload), encoding="utf-8")
    return AggregateResult(
        summary_path=summary_path,
        report_path=report_path,
        row_count=len(rows),
        failed_count=len(failed),
    )


def _render_combined_report(payload: dict[str, Any]) -> str:
    lines = [
        "# Combined Imitation Leaderboard",
        "",
        f"- Label: `{payload.get('label', '')}`",
        f"- Snapshot hash: `{payload.get('snapshot_hash', '')}`",
        f"- Split hash: `{payload.get('split_hash', '')}`",
        f"- Source reports: {len(payload.get('source_reports', []))}",
        "",
        "| Rank | Model | Split | Family | Backbone | Loss | Train Norm MSE | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Source |",
        "| ---: | --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in payload.get("rows", []):
        latency = row.get("latency", {})
        lines.append(
            f"| {row.get('rank', '')} | {row.get('model', '')} | "
            f"{row.get('split_mode', '')} | {row.get('family', '')} | "
            f"{row.get('backbone') or ''} | {row.get('loss_name') or ''} | "
            f"{_format_optional_float(row.get('train_normalized_action_mse'))} | "
            f"{float(row.get('normalized_action_mse', 0.0)):.6g} | "
            f"{float(row.get('action_mse', 0.0)):.6g} | "
            f"{float(row.get('translation_endpoint_error', 0.0)):.6g} | "
            f"{float(row.get('rotation_endpoint_error', 0.0)):.6g} | "
            f"{float(row.get('gripper_mse', 0.0)):.6g} | "
            f"{float(latency.get('median_ms', 0.0)):.4g} | {row.get('source_report', '')} |"
        )
    if payload.get("failed_runs"):
        lines.extend(["", "## Failed Runs", ""])
        for item in payload["failed_runs"]:
            lines.append(
                f"- `{item.get('model', '')}` from {item.get('source_report', '')}: "
                f"{item.get('error_type', '')}: {item.get('error', '')}"
            )
    lines.extend(
        [
            "",
            "## Safety",
            "",
            "- Offline HDF5 training/evaluation only.",
            "- No robot commands or real-mode behavior are part of these reports.",
        ]
    )
    return "\n".join(lines).rstrip() + "\n"


def _format_optional_float(value: Any) -> str:
    if value is None:
        return ""
    try:
        return f"{float(value):.6g}"
    except (TypeError, ValueError):
        return ""


def _recommendation(best: dict[str, Any] | None, state: dict[str, Any] | None) -> str:
    if best is None:
        return "No completed model is available."
    if best.get("family") == "constant_baseline":
        return (
            "No learned rollout candidate is recommended yet because the best validation score is a "
            "constant baseline. Train longer or revise the model/data setup before simulator rollout."
        )
    if state is not None and best.get("model") != "state_mlp":
        best_mse = float(best["metrics"].get("normalized_action_mse", math.inf))
        state_mse = float(state["metrics"].get("normalized_action_mse", math.inf))
        if best_mse >= state_mse:
            return (
                "Use the state-only model only as a diagnostic baseline; collect or crop more visually informative data "
                "before claiming image grounding."
            )
    return f"Next rollout candidate for simulator dry-run is `{best['model']}` from family `{best['family']}`."


def _find_result(results: list[dict[str, Any]], model_name: str) -> dict[str, Any] | None:
    for result in results:
        if result.get("model") == model_name:
            return result
    return None


def _metric_delta(best: dict[str, Any] | None, baseline: dict[str, Any] | None, metric: str) -> float | None:
    if best is None or baseline is None:
        return None
    return float(best["metrics"].get(metric, math.nan)) - float(baseline["metrics"].get(metric, math.nan))


def _gpu_info(*, torch: Any) -> dict[str, Any]:
    info = {
        "cuda_available": bool(torch.cuda.is_available()),
        "cuda_device_count": int(torch.cuda.device_count()) if torch.cuda.is_available() else 0,
        "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES", ""),
        "devices": [],
        "nvidia_smi": "",
    }
    if torch.cuda.is_available():
        info["devices"] = [
            {"index": index, "name": torch.cuda.get_device_name(index)}
            for index in range(torch.cuda.device_count())
        ]
    try:
        completed = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=index,name,memory.used,utilization.gpu",
                "--format=csv,noheader,nounits",
            ],
            check=False,
            text=True,
            capture_output=True,
            timeout=5,
        )
        info["nvidia_smi"] = completed.stdout.strip()
    except Exception as exc:  # noqa: BLE001
        info["nvidia_smi"] = f"unavailable: {type(exc).__name__}: {exc}"
    return info


def _require_torch() -> Any:
    try:
        import torch
    except ModuleNotFoundError as exc:
        raise RuntimeError("imitation experiments require torch; use policy_runner[ml] or the cuda-ml Docker image") from exc
    return torch


def _resolve_device(device: str, *, torch: Any) -> Any:
    if device == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(device)


def _seed_everything(seed: int, *, torch: Any) -> None:
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _backbone_name(name: str) -> str:
    if name == "tiny":
        return "tiny_cnn"
    return name


def _backbone_and_loss(name: str) -> tuple[str, str]:
    for suffix, loss_name in (
        ("_smoothl1", "smoothl1"),
        ("_huber", "huber"),
        ("_l1", "l1"),
        ("_mse", "mse"),
    ):
        if name.endswith(suffix):
            return _backbone_name(name[: -len(suffix)]), loss_name
    return _backbone_name(name), "mse"


def _timestamp_range(timestamps: np.ndarray) -> dict[str, float | None]:
    if timestamps.size == 0:
        return {"start": None, "end": None}
    return {"start": float(timestamps[0]), "end": float(timestamps[-1])}


def _session_name(root: Path, path: Path) -> str:
    try:
        relative = Path(path).resolve().relative_to(root.resolve())
    except ValueError:
        return Path(path).parent.name
    return relative.parts[0] if len(relative.parts) > 1 else "."


def _path_is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _validate_snapshot_split(snapshot: dict[str, Any], split: dict[str, Any]) -> None:
    if snapshot.get("schema") != IMITATION_SNAPSHOT_SCHEMA:
        raise ValueError(f"unsupported snapshot schema: {snapshot.get('schema')}")
    if split.get("schema") != IMITATION_SPLIT_SCHEMA:
        raise ValueError(f"unsupported split schema: {split.get('schema')}")
    if split.get("snapshot_hash") != snapshot.get("snapshot_hash"):
        raise ValueError("split manifest snapshot_hash does not match dataset snapshot")
    train = {entry["path"] for entry in split.get("train", [])}
    val = {entry["path"] for entry in split.get("validation", [])}
    overlap = sorted(train & val)
    if overlap:
        raise ValueError("train/validation split overlap detected: " + ", ".join(overlap[:5]))


def _payload_hash(payload: dict[str, Any]) -> str:
    stable_payload = dict(payload)
    stable_payload.pop("created_at", None)
    text = json.dumps(stable_payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()
