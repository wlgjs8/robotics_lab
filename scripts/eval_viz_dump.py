#!/usr/bin/env python3
"""Offline GT-vs-prediction dump for an imitation checkpoint (run in cuda-ml image).

Reuses the repo's own evaluation routine for apples-to-apples quantitative metrics,
and additionally reconstructs 3D action-chunk trajectories (GT vs predicted) for a
few validation episodes so they can be plotted locally (the cuda-ml image has no
matplotlib). Read-only with respect to the model/checkpoint.

Outputs:
  <out_prefix>_metrics.json : full eval metrics + per-dim raw RMSE
  <out_prefix>_viz.json     : episode paths + per-anchor GT/pred short segments

Example (on the GPU host):
  docker run --rm --gpus all -u $(id -u):$(id -g) \
    -e HOME=/tmp -e USER=plaif -e TORCHINDUCTOR_CACHE_DIR=/tmp/torchinductor \
    -e POLICY_RUNNER_DINOV3_DIR=/app/policy_runner/dinov3 \
    -v "$PWD/data:/data/policy_episodes:ro" \
    -v "$PWD/policy_runner/dinov3:/app/policy_runner/dinov3:ro" \
    -v "$PWD/outputs/flow_runs:/outputs/flow_runs" \
    -v "$PWD/scripts:/scripts:ro" \
    --entrypoint python3 robotics_lab/policy_runner:cuda-ml \
    /scripts/eval_viz_dump.py \
      --run-dir /outputs/flow_runs/imitation_train_metrics_best_small_primary \
      --checkpoint /outputs/flow_runs/imitation_train_metrics_best_small_primary/direct_bc_dinov3_convnext_small/checkpoint.pt \
      --split-manifest /outputs/flow_runs/imitation_official/split_manifest.json \
      --data-dir /data/policy_episodes \
      --out-prefix /outputs/flow_runs/_eval_viz_best
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
from policy_runner.flow_dataset import (  # noqa: E402
    FlowHdf5Dataset,
    denormalize_actions,
    normalize_runtime_proprio,
    runtime_proprio_from_state,
)

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


def _predict_chunk(model, ds, idx, stats, device, torch):
    sample = ds[idx]  # normalized
    img = torch.as_tensor(sample["images"]).unsqueeze(0).float().to(device)
    prop = torch.as_tensor(sample["proprio"]).unsqueeze(0).float().to(device)
    with torch.no_grad():
        pred = model(img, prop)
    pred_raw = denormalize_actions(pred, stats)[0].cpu().numpy()  # [H,14]
    gt_raw = ds.raw_sample(idx)["action_chunk"]                   # [H,14]
    return pred_raw, gt_raw


def _quat_to_R(q):
    q = np.asarray(q, float)
    n = np.linalg.norm(q) or 1.0
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _rotvec_to_R(rv):
    rv = np.asarray(rv, float)
    th = float(np.linalg.norm(rv))
    if th < 1e-9:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def _integrate_positions(start_pose7, deltas6, frame):
    """start pose [x,y,z,qx,qy,qz,qw] + per-step 6D deltas -> (H+1,3) positions.

    'ee_local' composes each delta in the moving body frame (SE(3)); 'stand' adds
    the translation in the fixed world frame (legacy cumulative sum).
    """
    start_pose7 = np.asarray(start_pose7, float)
    t = start_pose7[:3].copy()
    out = [t.copy()]
    if frame == "ee_local":
        R = _quat_to_R(start_pose7[3:7])
        for d in deltas6:
            t = t + R @ np.asarray(d[:3], float)
            R = R @ _rotvec_to_R(d[3:6])
            out.append(t.copy())
    else:
        for d in deltas6:
            t = t + np.asarray(d[:3], float)
            out.append(t.copy())
    return np.asarray(out)


def _R_to_quat(R):
    R = np.asarray(R, float)
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2; w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def _apply_delta(pose7, delta6, frame):
    """Compose a per-step 6D delta onto a pose [x,y,z,qx,qy,qz,qw] -> new pose7."""
    pose7 = np.asarray(pose7, float)
    R = _quat_to_R(pose7[3:7]); t = pose7[:3].copy()
    if frame == "ee_local":
        t = t + R @ np.asarray(delta6[:3], float)
        R = R @ _rotvec_to_R(delta6[3:6])
    else:
        t = t + np.asarray(delta6[:3], float)
        R = _rotvec_to_R(delta6[3:6]) @ R
    return np.concatenate([t, _R_to_quat(R)])


def _pose_payload(pose7, grip):
    p = np.asarray(pose7, float)
    return {"tcp_stand": {"position": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
                          "quaternion_xyzw": [float(p[3]), float(p[4]), float(p[5]), float(p[6])]},
            "gripper": float(grip)}


def _rollout(model, ds, ep, ei, refmap, stats, frame, device, torch):
    """Open-loop rollout: chain 1-step predictions (teacher-forced GT images) from the
    episode reset pose; returns predicted left/right position paths [N,3]."""
    N = int(ep.length)
    rolled_l = np.asarray(ep.reset_left_pose, float).copy()
    rolled_r = np.asarray(ep.reset_right_pose, float).copy()
    arm_mask = np.asarray(ep.arm_mask, float)
    posL = [rolled_l[:3].copy()]
    posR = [rolled_r[:3].copy()]
    for t in range(N - 1):
        idx = refmap.get((ei, t))
        if idx is None:
            break
        payload = {"left": _pose_payload(rolled_l, ep.left_gripper[t]),
                   "right": _pose_payload(rolled_r, ep.right_gripper[t])}
        proprio = runtime_proprio_from_state(
            payload, reset_left_pose=ep.reset_left_pose, reset_right_pose=ep.reset_right_pose,
            arm_mask=arm_mask, action_frame=frame)
        proprio = normalize_runtime_proprio(proprio, stats)
        img = torch.as_tensor(ds[idx]["images"]).unsqueeze(0).float().to(device)
        prop = torch.as_tensor(proprio).unsqueeze(0).float().to(device)
        with torch.no_grad():
            pred = denormalize_actions(model(img, prop), stats)[0].cpu().numpy()
        step = pred[0]
        if arm_mask[0] > 0:
            rolled_l = _apply_delta(rolled_l, step[0:6], frame)
        if arm_mask[1] > 0:
            rolled_r = _apply_delta(rolled_r, step[7:13], frame)
        posL.append(rolled_l[:3].copy())
        posR.append(rolled_r[:3].copy())
    return np.asarray(posL), np.asarray(posR)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--split-manifest", required=True)
    ap.add_argument("--split-key", default="validation",
                    choices=["validation", "session_holdout_val"])
    ap.add_argument("--data-dir", required=True)
    ap.add_argument("--out-prefix", required=True)
    ap.add_argument("--viz-episodes", type=int, default=3)
    ap.add_argument("--anchors-per-episode", type=int, default=6)
    args = ap.parse_args()

    import torch

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    run_dir = Path(args.run_dir)
    ckpt_path = Path(args.checkpoint)
    payload = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    if payload.get("model_family") != "direct_bc_chunk":
        print(f"WARNING: model_family={payload.get('model_family')} (script tuned for direct_bc_chunk)")
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

    # --- quantitative: repo's own routine (apples-to-apples with result.json) ---
    indices = list(range(len(ds)))
    metrics = ie._evaluate_supervised_model(
        model=model, dataset=ds, indices=indices, stats=stats, device=device, torch=torch)

    # --- per-dim raw RMSE (masked by active arm) ---
    sq = np.zeros(14)
    cnt = np.zeros(14)
    for idx in indices:
        pred_raw, gt_raw = _predict_chunk(model, ds, idx, stats, device, torch)
        mask = ds.raw_sample(idx)["action_mask"]  # [H,14]
        d2 = (pred_raw - gt_raw) ** 2 * mask
        sq += d2.sum(axis=0)
        cnt += mask.sum(axis=0)
    per_dim_rmse = {ACTION_DIM_NAMES[i]: float(np.sqrt(sq[i] / cnt[i])) if cnt[i] > 0 else None
                    for i in range(14)}

    metrics_out = {
        "checkpoint": str(ckpt_path),
        "image_size": int(meta["image_size"]),
        "n_val_episodes": len(ds.episodes),
        "n_val_samples": len(ds),
        "eval_metrics": metrics,
        "per_dim_raw_rmse": per_dim_rmse,
        "action_dim_names": ACTION_DIM_NAMES,
        "note": "delta dims in meters/radian (stand-frame), grip in recorded gripper units",
    }
    Path(args.out_prefix + "_metrics.json").write_text(json.dumps(metrics_out, indent=2))

    # --- viz: per-episode paths + per-anchor GT/pred short segments ---
    refmap = {}
    for i, ref in enumerate(ds.sample_refs):
        refmap[(ref.episode_index, ref.start)] = i

    viz_eps = []
    n_eps = min(args.viz_episodes, len(ds.episodes))
    for ei in range(n_eps):
        ep = ds.episodes[ei]
        L = int(ep.length)
        amask = [float(ep.arm_mask[0]), float(ep.arm_mask[1])]
        max_start = L - horizon - 1
        if max_start < 1:
            continue
        anchors_t = sorted(set(int(t) for t in np.linspace(0, max_start, args.anchors_per_episode)))
        anchors = []
        for t in anchors_t:
            if (ei, t) not in refmap:
                continue
            pred_raw, gt_raw = _predict_chunk(model, ds, refmap[(ei, t)], stats, device, torch)
            entry = {"t": t}
            if amask[0] > 0:
                entry["pred_left"] = np.round(
                    _integrate_positions(ep.left_pose[t], pred_raw[:, 0:6], action_frame), 5).tolist()
                entry["gt_left"] = np.round(ep.left_pose[t:t + horizon + 1, :3], 5).tolist()
            if amask[1] > 0:
                entry["pred_right"] = np.round(
                    _integrate_positions(ep.right_pose[t], pred_raw[:, 7:13], action_frame), 5).tolist()
                entry["gt_right"] = np.round(ep.right_pose[t:t + horizon + 1, :3], 5).tolist()
            anchors.append(entry)
        roll_l, roll_r = _rollout(model, ds, ep, ei, refmap, stats, action_frame, device, torch)
        viz_eps.append({
            "name": Path(ep.path).parent.name + "/" + Path(ep.path).name,
            "arm_mask": amask,
            "length": L,
            "left_path": np.round(ep.left_pose[:, :3], 5).tolist() if amask[0] > 0 else [],
            "right_path": np.round(ep.right_pose[:, :3], 5).tolist() if amask[1] > 0 else [],
            "rollout_left": np.round(roll_l, 5).tolist() if amask[0] > 0 else [],
            "rollout_right": np.round(roll_r, 5).tolist() if amask[1] > 0 else [],
            "anchors": anchors,
        })

    viz_out = {
        "checkpoint": str(ckpt_path),
        "action_horizon": horizon,
        "action_frame": action_frame,
        "per_dim_raw_rmse": per_dim_rmse,
        "action_dim_names": ACTION_DIM_NAMES,
        "frame": "gripper-TCP positions in the capture (steamvr) frame; rollout is open-loop teacher-forced-image",
        "episodes": viz_eps,
    }
    Path(args.out_prefix + "_viz.json").write_text(json.dumps(viz_out))

    print("\n=== SUMMARY ===", flush=True)
    print(f"action_mse={metrics.get('action_mse'):.4f}  "
          f"normalized_action_mse={metrics.get('normalized_action_mse'):.4f}", flush=True)
    print(f"translation_endpoint_error={metrics.get('translation_endpoint_error')}  "
          f"rotation_endpoint_error={metrics.get('rotation_endpoint_error')}", flush=True)
    print("per-dim raw RMSE:", flush=True)
    for n in ACTION_DIM_NAMES:
        v = per_dim_rmse[n]
        print(f"  {n:10s} {('%.5f' % v) if v is not None else 'n/a'}", flush=True)
    print(f"\nwrote {args.out_prefix}_metrics.json and {args.out_prefix}_viz.json", flush=True)


if __name__ == "__main__":
    main()
