#!/usr/bin/env python3
"""Dump per-step policy actions for validation episodes (run in cuda-ml image).

Teacher-forced: for every timestep t of each selected validation episode the
model sees the recorded (normalized) image+proprio sample and we keep the first
action of the predicted chunk. The resulting raw 14-dim action sequence can be
replayed against rb_servo_server (e.g. as TcpTwistLocal) without torch.

Output JSON schema (robotics_lab.policy_runner.action_dump.v1):
  {checkpoint, action_frame, dt_mean_sec, action_dim_names, episodes: [
     {name, length, arm_mask, actions: [[14] raw], gt_actions: [[14] raw]}]}

Example (on the GPU host, repo root):
  docker run --rm --gpus all -u $(id -u):$(id -g) \
    -e HOME=/tmp -e USER=plaif -e TORCHINDUCTOR_CACHE_DIR=/tmp/torchinductor \
    -e POLICY_RUNNER_DINOV3_DIR=/app/policy_runner/dinov3 \
    -v "$PWD/data_tcp:/data/policy_episodes:ro" \
    -v "$PWD/policy_runner/dinov3:/app/policy_runner/dinov3:ro" \
    -v "$PWD/outputs/flow_runs:/outputs/flow_runs" \
    -v "$PWD/scripts:/scripts:ro" \
    --entrypoint python3 robotics_lab/policy_runner:cuda-ml \
    /scripts/dump_policy_actions.py \
      --run-dir /outputs/flow_runs/ee_local_seed5_lr3e4 \
      --checkpoint /outputs/flow_runs/ee_local_seed5_lr3e4/direct_bc_dinov3_convnext_tiny/checkpoint.pt \
      --split-manifest /outputs/flow_runs/ee_local_seed5_lr3e4/split_manifest.json \
      --split-key session_holdout_val \
      --data-dir /data/policy_episodes \
      --episodes 3 \
      --out /outputs/flow_runs/_action_dump_eelocal.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# Prefer repo copy if mounted; otherwise the installed package in the image is used.
REPO_PR = Path(__file__).resolve().parents[1] / "policy_runner"
if REPO_PR.exists() and str(REPO_PR) not in sys.path:
    sys.path.insert(0, str(REPO_PR))

from policy_runner import imitation_experiments as ie  # noqa: E402
from policy_runner.flow_dataset import FlowHdf5Dataset, denormalize_actions  # noqa: E402

ACTION_DIM_NAMES = [
    "left_dx", "left_dy", "left_dz", "left_drx", "left_dry", "left_drz", "left_grip",
    "right_dx", "right_dy", "right_dz", "right_drx", "right_dry", "right_drz", "right_grip",
]


def _run_meta(run_dir: Path) -> dict:
    meta = {"image_size": 224, "image_crop": "none"}
    for name in ("leaderboard_summary.json", "train_dataset_stats.json"):
        p = run_dir / name
        if not p.exists():
            continue
        blob = json.loads(p.read_text())
        args = blob.get("args", blob)
        for k in ("image_size", "image_crop"):
            if isinstance(args, dict) and args.get(k) is not None:
                meta[k] = args[k]
    return meta


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--split-manifest", required=True)
    ap.add_argument("--split-key", default="session_holdout_val",
                    choices=["validation", "session_holdout_val", "train", "session_holdout_train"])
    ap.add_argument("--data-dir", required=True)
    ap.add_argument("--episodes", type=int, default=3, help="number of validation episodes to dump")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    import torch

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    run_dir = Path(args.run_dir)
    ckpt_path = Path(args.checkpoint)
    payload = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    stats = payload["dataset_stats"]
    action_frame = str(stats.get("proprio_action_frame", "stand"))
    meta = _run_meta(run_dir)
    camera_names = list(payload.get("camera_names", []))
    horizon = int(payload["action_horizon"])

    hidden_dim = 256
    rj = ckpt_path.parent / "result.json"
    if rj.exists():
        hidden_dim = int(json.loads(rj.read_text()).get("model_config", {}).get("hidden_dim", 256))

    manifest = json.loads(Path(args.split_manifest).read_text())
    val_paths = [e["path"] for e in manifest[args.split_key]]

    ds = FlowHdf5Dataset(
        args.data_dir,
        action_horizon=horizon,
        image_size=int(meta["image_size"]),
        image_crop=str(meta["image_crop"]),
        camera_names=camera_names,
        episode_paths=val_paths,
        stats=stats,
        normalize=True,
        action_frame=action_frame,
    )
    print(f"val: {len(ds.episodes)} episodes, {len(ds)} samples; "
          f"image_size={meta['image_size']}; action_frame={action_frame}", flush=True)

    model = ie._build_direct_bc_policy(
        backbone=payload["backbone"], action_horizon=horizon,
        camera_count=len(camera_names), hidden_dim=hidden_dim,
    ).to(device)
    model.load_state_dict(payload["model_state"])
    model.eval()

    refmap = {}
    for i, ref in enumerate(ds.sample_refs):
        refmap[(ref.episode_index, ref.start)] = i

    episodes_out = []
    for ei in range(min(args.episodes, len(ds.episodes))):
        ep = ds.episodes[ei]
        L = int(ep.length)
        actions, gt_actions = [], []
        for t in range(L - 1):
            idx = refmap.get((ei, t))
            if idx is None:
                break
            sample = ds[idx]
            img = torch.as_tensor(sample["images"]).unsqueeze(0).float().to(device)
            prop = torch.as_tensor(sample["proprio"]).unsqueeze(0).float().to(device)
            with torch.no_grad():
                pred = model(img, prop)
            pred_raw = denormalize_actions(pred, stats)[0].cpu().numpy()  # [H,14]
            gt_raw = ds.raw_sample(idx)["action_chunk"]                   # [H,14]
            actions.append(np.round(pred_raw[0], 7).tolist())
            gt_actions.append(np.round(gt_raw[0], 7).tolist())
        episodes_out.append({
            "name": Path(ep.path).parent.name + "/" + Path(ep.path).name,
            "length": L,
            "arm_mask": [float(ep.arm_mask[0]), float(ep.arm_mask[1])],
            "actions": actions,
            "gt_actions": gt_actions,
        })
        print(f"episode {ei}: {episodes_out[-1]['name']} steps={len(actions)} "
              f"arm_mask={episodes_out[-1]['arm_mask']}", flush=True)

    out = {
        "schema": "robotics_lab.policy_runner.action_dump.v1",
        "checkpoint": str(ckpt_path),
        "action_frame": action_frame,
        "dt_mean_sec": stats.get("dt_mean_sec"),
        "action_horizon": horizon,
        "action_dim_names": ACTION_DIM_NAMES,
        "note": "actions are teacher-forced per-step first-chunk predictions, raw units "
                "(m / rad / gripper units), frame per action_frame",
        "episodes": episodes_out,
    }
    Path(args.out).write_text(json.dumps(out))
    print(f"wrote {args.out}", flush=True)


if __name__ == "__main__":
    main()
