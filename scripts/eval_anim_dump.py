#!/usr/bin/env python3
"""Dense per-timestep GT-vs-prediction dump for trajectory ANIMATION.

Runs in the cuda-ml image (read-only w.r.t. the checkpoint). For the first N
validation episodes it runs the model at every timestep (stride configurable) and
stores, per anchor t: the predicted action-chunk positions (integrated) and the GT
chunk positions, for both arms, plus the full demo path and gripper signals.

Output: an .npz (arrays) + a sidecar .meta.json (names, per-episode fps, horizon).
Plot/encode locally with scripts/eval_animate.py (the image has no matplotlib).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_PR = Path(__file__).resolve().parents[1] / "policy_runner"
if REPO_PR.exists() and str(REPO_PR) not in sys.path:
    sys.path.insert(0, str(REPO_PR))

from policy_runner import imitation_experiments as ie  # noqa: E402
from policy_runner.flow_dataset import FlowHdf5Dataset, denormalize_actions  # noqa: E402


def _run_meta(run_dir: Path) -> dict:
    meta = {"image_size": 224, "image_crop": "none"}
    for name in ("leaderboard_summary.json", "train_dataset_stats.json"):
        p = run_dir / name
        if not p.exists():
            continue
        blob = json.loads(p.read_text())
        a = blob.get("args", blob)
        for k in ("image_size", "image_crop"):
            if isinstance(a, dict) and a.get(k) is not None:
                meta[k] = a[k]
    return meta


def _integrate(start_xyz, deltas_xyz):
    out = [np.asarray(start_xyz, dtype=float)]
    for d in deltas_xyz:
        out.append(out[-1] + np.asarray(d, dtype=float))
    return np.asarray(out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--split-manifest", required=True)
    ap.add_argument("--split-key", default="validation",
                    choices=["validation", "session_holdout_val"])
    ap.add_argument("--data-dir", required=True)
    ap.add_argument("--out", required=True, help="output .npz path")
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--stride", type=int, default=1)
    args = ap.parse_args()

    import torch

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    run_dir = Path(args.run_dir)
    ckpt_path = Path(args.checkpoint)
    payload = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    stats = payload["dataset_stats"]
    meta = _run_meta(run_dir)
    camera_names = list(payload.get("camera_names", []))
    horizon = int(payload["action_horizon"])
    hidden = 256
    rj = ckpt_path.parent / "result.json"
    if rj.exists():
        hidden = int(json.loads(rj.read_text()).get("model_config", {}).get("hidden_dim", 256))

    manifest = json.loads(Path(args.split_manifest).read_text())
    val_paths = [e["path"] for e in manifest[args.split_key]]
    ds = FlowHdf5Dataset(
        args.data_dir, action_horizon=horizon, image_size=int(meta["image_size"]),
        image_crop=str(meta["image_crop"]), camera_names=camera_names,
        episode_paths=val_paths, stats=stats, normalize=True)
    model = ie._build_direct_bc_policy(
        backbone=payload["backbone"], action_horizon=horizon,
        camera_count=len(camera_names), hidden_dim=hidden).to(device)
    model.load_state_dict(payload["model_state"])
    model.eval()

    refmap = {(r.episode_index, r.start): i for i, r in enumerate(ds.sample_refs)}
    out: dict[str, np.ndarray] = {}
    meta_eps = []
    n = min(args.episodes, len(ds.episodes))
    for ei in range(n):
        ep = ds.episodes[ei]
        L = int(ep.length)
        amask = [float(ep.arm_mask[0]), float(ep.arm_mask[1])]
        ts = [t for t in range(0, max(1, L - horizon - 1), args.stride) if (ei, t) in refmap]
        idxs = [refmap[(ei, t)] for t in ts]
        preds = []
        B = 64
        for s in range(0, len(idxs), B):
            cidx = idxs[s:s + B]
            imgs = np.stack([ds[j]["images"] for j in cidx], 0)
            props = np.stack([ds[j]["proprio"] for j in cidx], 0)
            with torch.no_grad():
                pr = model(torch.as_tensor(imgs).float().to(device),
                           torch.as_tensor(props).float().to(device))
            preds.append(denormalize_actions(pr, stats).cpu().numpy())
        preds = np.concatenate(preds, 0) if preds else np.zeros((0, horizon, 14))

        predL, gtL, predR, gtR = [], [], [], []
        for k, t in enumerate(ts):
            predL.append(_integrate(ep.left_pose[t][:3], preds[k][:, 0:3]))
            gtL.append(ep.left_pose[t:t + horizon + 1, :3])
            predR.append(_integrate(ep.right_pose[t][:3], preds[k][:, 7:10]))
            gtR.append(ep.right_pose[t:t + horizon + 1, :3])

        out[f"ep{ei}_left_path"] = ep.left_pose[:, :3].astype(np.float32)
        out[f"ep{ei}_right_path"] = ep.right_pose[:, :3].astype(np.float32)
        out[f"ep{ei}_gripL"] = np.asarray(ep.left_gripper, dtype=np.float32)
        out[f"ep{ei}_gripR"] = np.asarray(ep.right_gripper, dtype=np.float32)
        out[f"ep{ei}_t"] = np.asarray(ts, dtype=np.int32)
        out[f"ep{ei}_predL"] = np.asarray(predL, dtype=np.float32)
        out[f"ep{ei}_gtL"] = np.asarray(gtL, dtype=np.float32)
        out[f"ep{ei}_predR"] = np.asarray(predR, dtype=np.float32)
        out[f"ep{ei}_gtR"] = np.asarray(gtR, dtype=np.float32)

        tarr = np.asarray(ep.timestamps, dtype=np.float64).reshape(-1)
        dt = float(np.median(np.diff(tarr))) if tarr.size > 1 else 1.0 / 30.0
        if dt > 1000:
            dt /= 1e9
        fps = (1.0 / dt / args.stride) if dt > 0 else 30.0
        meta_eps.append({"name": Path(ep.path).parent.name + "/" + Path(ep.path).name,
                         "arm_mask": amask, "length": L, "fps": round(fps, 3),
                         "n_frames": len(ts)})
        print(f"ep{ei} {meta_eps[-1]['name']}: {len(ts)} frames @ {fps:.1f}fps", flush=True)

    np.savez_compressed(args.out, **out)
    Path(args.out + ".meta.json").write_text(
        json.dumps({"horizon": horizon, "n_episodes": n, "episodes": meta_eps}, indent=2))
    print(f"wrote {args.out} and {args.out}.meta.json", flush=True)


if __name__ == "__main__":
    main()
