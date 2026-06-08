import argparse
import json
import time
from pathlib import Path

import torch

from policy_runner import imitation_experiments as ie
from policy_runner.flow_dataset import FlowHdf5Dataset


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-name", required=True)
    parser.add_argument("--hidden-dim", type=int, required=True)
    parser.add_argument("--hard-weight", type=float, required=True)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    root = Path("/outputs/flow_runs")
    out_root = root / "imitation_distill_cached_direct_tiny_holdout_color384_h1_latest"
    out_dir = out_root / args.out_name / "direct_bc_dinov3_convnext_tiny_distill"
    out_dir.mkdir(parents=True, exist_ok=True)
    ref = json.loads(
        (root / "imitation_dinov3_tiny_holdout_color384_h1_seed5_lr3e4/leaderboard_summary.json").read_text()
    )
    run_args = ref["args"]
    stats = ie._load_json(ref["normalization"]["path"])
    snapshot = ie._load_json(ref["dataset"]["snapshot_path"])
    split = ie._load_json(ref["split"]["split_path"])
    snapshot["_source_path"] = ref["dataset"]["snapshot_path"]
    split["_source_path"] = ref["split"]["split_path"]
    ie._validate_snapshot_split(snapshot, split)
    effective = ie._effective_split_for_mode(snapshot, split, split_mode="session_holdout")
    train_paths = [entry["path"] for entry in effective["train"]]
    val_paths = [entry["path"] for entry in effective["validation"]]
    train_dataset = FlowHdf5Dataset(
        run_args["data_dir"],
        action_horizon=1,
        image_size=384,
        image_crop="none",
        episode_paths=train_paths,
        camera_names=list(stats["camera_names"]),
        exclude_camera_names=run_args["exclude_camera_names"],
        stats=stats,
        normalize=True,
    )
    val_dataset = FlowHdf5Dataset(
        run_args["data_dir"],
        action_horizon=1,
        image_size=384,
        image_crop="none",
        episode_paths=val_paths,
        camera_names=list(stats["camera_names"]),
        exclude_camera_names=run_args["exclude_camera_names"],
        stats=stats,
        normalize=True,
    )

    cache = torch.load(out_root / "train_teacher_cache.pt", map_location="cpu", weights_only=False)
    teacher_predictions = cache["teacher_predictions"]
    if int(teacher_predictions.shape[0]) != len(train_dataset):
        raise ValueError("teacher cache sample count does not match train dataset")

    torch.manual_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    student = ie._build_direct_bc_policy(
        backbone="dinov3_convnext_tiny",
        action_horizon=1,
        camera_count=len(stats["camera_names"]),
        hidden_dim=args.hidden_dim,
    ).to(device)
    optimizer = torch.optim.AdamW(
        (parameter for parameter in student.parameters() if parameter.requires_grad),
        lr=args.lr,
    )
    train_indices = list(range(len(train_dataset)))
    val_indices = list(range(len(val_dataset)))
    loader = ie._loader(
        train_dataset,
        train_indices,
        batch_size=args.batch_size,
        shuffle=True,
        torch=torch,
        include_index=True,
    )
    curves_path = out_dir / "training_curves.jsonl"
    start = time.perf_counter()
    with curves_path.open("w", encoding="utf-8") as curves:
        for epoch in range(1, args.epochs + 1):
            student.train()
            total = 0.0
            hard_total = 0.0
            teacher_total = 0.0
            denom = 0.0
            for batch in loader:
                sample_indices = batch.pop("_sample_index").long()
                batch = ie._to_device(batch, device, torch=torch)
                target_teacher = teacher_predictions[sample_indices].to(device)
                target_hard = batch["action_chunk"].float()
                mask = batch["action_mask"].float()
                pred = student(batch["images"].float(), batch["proprio"].float())
                hard_loss = ((pred - target_hard) ** 2 * mask).sum() / mask.sum().clamp_min(1.0)
                teacher_loss = ((pred - target_teacher) ** 2 * mask).sum() / mask.sum().clamp_min(1.0)
                loss = args.hard_weight * hard_loss + (1.0 - args.hard_weight) * teacher_loss
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(student.parameters(), 10.0)
                optimizer.step()
                n = float(mask.sum().item())
                total += float(loss.item()) * n
                hard_total += float(hard_loss.item()) * n
                teacher_total += float(teacher_loss.item()) * n
                denom += n
            row = {
                "epoch": epoch,
                "train_loss": total / max(denom, 1.0),
                "hard_loss": hard_total / max(denom, 1.0),
                "teacher_loss": teacher_total / max(denom, 1.0),
            }
            curves.write(json.dumps(row, sort_keys=True) + "\n")
            curves.flush()
            print(row, flush=True)

    metrics = ie._evaluate_supervised_model(
        model=student,
        dataset=val_dataset,
        indices=val_indices,
        stats=stats,
        device=device,
        torch=torch,
    )
    train_metrics = ie._evaluate_supervised_model(
        model=student,
        dataset=train_dataset,
        indices=train_indices,
        stats=stats,
        device=device,
        torch=torch,
    )
    latency = ie._measure_supervised_latency(
        model=student,
        dataset=val_dataset,
        device=device,
        torch=torch,
    )
    shortcut_ablations = ie._supervised_shortcut_ablations(
        model=student,
        dataset=val_dataset,
        indices=val_indices,
        stats=stats,
        device=device,
        torch=torch,
    )
    checkpoint_path = out_dir / "checkpoint.pt"
    checkpoint_payload = {
        "schema": "robotics_lab.policy_runner.imitation_checkpoint.v1",
        "model_family": "direct_bc_distilled_cached_ensemble",
        "backbone": "dinov3_convnext_tiny",
        "loss_name": "cached_distill_mse",
        "action_horizon": 1,
        "action_dim": 14,
        "proprio_dim": 16,
        "camera_names": list(stats["camera_names"]),
        "dataset_stats": stats,
        "train_metrics": train_metrics,
        "validation_metrics": metrics,
        "teacher_cache_path": str(out_root / "train_teacher_cache.pt"),
        "teacher_checkpoint_sha256": cache["teacher_checkpoint_sha256"],
        "hard_weight": args.hard_weight,
        "model_state": student.cpu().state_dict(),
    }
    torch.save(checkpoint_payload, checkpoint_path)
    checkpoint_sha = ie._sha256_file(checkpoint_path)
    result = {
        "model": f"distill_cached_direct_bc_dinov3_convnext_tiny_{args.out_name}",
        "family": "direct_bc_distilled_cached_ensemble",
        "backbone": "dinov3_convnext_tiny",
        "loss_name": "cached_distill_mse",
        "checkpoint_path": str(checkpoint_path),
        "checkpoint_sha256": checkpoint_sha,
        "training_curves": str(curves_path),
        "train_metrics": train_metrics,
        "metrics": metrics,
        "latency": latency,
        "shortcut_ablations": shortcut_ablations,
        "wall_time_sec": time.perf_counter() - start,
        "model_config": {
            "action_horizon": 1,
            "action_dim": 14,
            "proprio_dim": 16,
            "camera_names": list(stats["camera_names"]),
            "backbone": "dinov3_convnext_tiny",
            "hidden_dim": args.hidden_dim,
            "loss_name": "cached_distill_mse",
            "hard_weight": args.hard_weight,
            "teacher": "top5_prediction_ensemble_train_cache",
            "epochs": args.epochs,
            "lr": args.lr,
        },
        "safety": {
            "scope": "offline HDF5 cached distillation only",
            "real_mode_behavior_touched": False,
            "robot_motion_commands_sent": False,
        },
    }
    (out_dir / "result.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    leaderboard = [
        {
            "rank": 1,
            "model": result["model"],
            "family": result["family"],
            "backbone": result["backbone"],
            "train_normalized_action_mse": train_metrics["normalized_action_mse"],
            "normalized_action_mse": metrics["normalized_action_mse"],
            "action_mse": metrics["action_mse"],
            "translation_endpoint_error": metrics["translation_endpoint_error"],
            "rotation_endpoint_error": metrics["rotation_endpoint_error"],
            "gripper_mse": metrics["gripper_mse"],
            "inactive_arm_leakage": metrics["inactive_arm_leakage"],
            "latency": latency,
            "checkpoint_sha256": checkpoint_sha,
            "wall_time_sec": result["wall_time_sec"],
            "loss_name": "cached_distill_mse",
            "split_mode": "session_holdout",
        }
    ]
    summary = {
        "schema": ie.IMITATION_REPORT_SCHEMA,
        "created_at": ie._utc_now(),
        "output_dir": str(out_dir.parent),
        "dataset": ref["dataset"],
        "split": ref["split"],
        "normalization": ref["normalization"],
        "args": {
            **run_args,
            "models": [result["model"]],
            "epochs": args.epochs,
            "lr": args.lr,
            "hidden_dim": args.hidden_dim,
            "hard_weight": args.hard_weight,
            "teacher_cache_path": str(out_root / "train_teacher_cache.pt"),
        },
        "leaderboard": leaderboard,
        "failed_runs": [],
        "results": [result],
        "warnings": [
            "Teacher top5 ensemble was selected from validation leaderboard; treat as post-hoc compression evidence, not leakage-free model-family selection.",
            "Teacher cache contains train-split predictions only.",
        ],
        "recommendation": "Use only if it beats the best single checkpoint and runtime checkpoint loading is accepted.",
        "safety": result["safety"],
    }
    (out_dir.parent / "leaderboard_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        "RESULT",
        result["model"],
        metrics["normalized_action_mse"],
        train_metrics["normalized_action_mse"],
        latency,
        checkpoint_sha[:12],
        flush=True,
    )


if __name__ == "__main__":
    main()
