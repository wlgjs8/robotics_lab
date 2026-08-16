"""Paired 30-vs-90 fps policy A/B on captured fisheye frames.

Feeds the deployed VA checkpoint (l2 branch, serve_va.py preprocessing replicated)
the same static scene captured through camera_server at 30 fps and 90 fps, with a
fixed state, and asks: does the 90 fps appearance shift move the actions beyond
the within-condition noise floor?

Run with the openpi venv:
  /home/plaif/workspace/openpi/.venv/bin/python tools/ab_policy_30v90.py
"""
import pathlib
import sys

import numpy as np
import torch

CKPT = "/home/plaif/va_runs/ema999/step_80000.pt"
WEIGHTS_DIR = "/home/plaif/dinov3_weights"
DINOV3_SRC = "/home/plaif/dinov3_src"
VA_SRC = "/home/plaif/workspace/openpi/examples/pika_umi"
FRAMES_30 = sys.argv[2] if len(sys.argv) > 2 else "/tmp/ab_frames_30fps.npz"
FRAMES_90 = sys.argv[3] if len(sys.argv) > 3 else "/tmp/ab_frames_90fps.npz"
REAL_DIM = 14
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)

sys.path.insert(0, DINOV3_SRC)
sys.path.insert(0, VA_SRC)


def _normalize(x, q01, q99):
    return (x - q01) / (q99 - q01 + 1e-6) * 2.0 - 1.0


def _unnormalize(x, q01, q99):
    return (x + 1.0) / 2.0 * (q99 - q01 + 1e-6) + q01


def _prep(img, res, device):
    from openpi_client import image_tools

    x = image_tools.resize_with_pad(np.asarray(img), res, res)
    x = torch.from_numpy(np.asarray(x).copy()).permute(2, 0, 1).float() / 255.0
    m = torch.tensor(IMAGENET_MEAN)[:, None, None]
    s = torch.tensor(IMAGENET_STD)[:, None, None]
    return ((x - m) / s)[None].to(device).to(torch.bfloat16)


def main():
    from train_va_dinov3 import VAPolicy
    import openpi.training.config as _config

    device = "cuda"
    ck = torch.load(CKPT, map_location="cpu", weights_only=False)
    res, horizon = int(ck.get("resolution", 224)), ck["horizon"]
    model = VAPolicy(ck["size"], pathlib.Path(WEIGHTS_DIR), horizon,
                     backbone_type=ck.get("backbone", "dinov3"), wm_head=ck.get("wm", False),
                     head_mode=ck.get("head", "l2"), layers=ck.get("layers", 4),
                     split_head=ck.get("split_head", False))
    sd = ck.get("ema") or ck["model"]
    model.load_state_dict({k: v.float() for k, v in sd.items()})
    model = model.to(device).eval().to(torch.bfloat16)

    cfg = _config.get_config(ck["config"])
    norm_stats = cfg.data.create(cfg.assets_dirs, cfg.model).norm_stats
    a_q01 = np.asarray(norm_stats["actions"].q01)[:REAL_DIM]
    a_q99 = np.asarray(norm_stats["actions"].q99)[:REAL_DIM]
    s_q01 = np.asarray(norm_stats["state"].q01)[:REAL_DIM]
    s_q99 = np.asarray(norm_stats["state"].q99)[:REAL_DIM]

    # Fixed mid-range state (normalises to ~0); identical across conditions so any
    # action difference is attributable to the images alone.
    state = (s_q01 + s_q99) / 2.0
    sn = _normalize(state, s_q01, s_q99)
    st = torch.from_numpy(sn).float()[None].to(device).to(torch.bfloat16)

    source = sys.argv[1] if len(sys.argv) > 1 else "fisheye"  # fisheye | realsense
    print(f"image source: {source}")
    results = {}
    for tag, path in (("30fps", FRAMES_30), ("90fps", FRAMES_90)):
        data = np.load(path)
        lf, rf = data[f"left_{source}_color"], data[f"right_{source}_color"]
        chunks = []
        with torch.no_grad():
            for i in range(lf.shape[0]):
                il = _prep(lf[i], res, device)
                ir = _prep(rf[i], res, device)
                c = model.sample_actions(il, ir, st, num_steps=10, noise_scale=1.0,
                                         branch="l2").float().cpu().numpy()[0]
                chunks.append(_unnormalize(c.astype(np.float64), a_q01, a_q99))
        results[tag] = np.stack(chunks)  # (N, H, 14)
        print(f"{tag}: {results[tag].shape[0]} chunks, brightness "
              f"L={lf.mean():.1f} R={rf.mean():.1f}")

    a30, a90 = results["30fps"], results["90fps"]
    mean_diff = np.abs(a30.mean(0) - a90.mean(0))          # (H, 14)
    pooled_std = np.sqrt((a30.std(0) ** 2 + a90.std(0) ** 2) / 2) + 1e-12
    effect = mean_diff / pooled_std

    mm = 1000.0
    exec_rows = slice(0, 5)  # FLOW_INFER_CHUNK_EXECUTE_STEPS=5: only these rows run
    print("\n=== executed rows 0-4 (the rows the robot actually runs) ===")
    for arm, base in (("left", 0), ("right", 7)):
        t = slice(base, base + 3)
        print(f"  {arm} xyz |mean diff| {mean_diff[exec_rows, t].mean() * mm:.4f} mm/step "
              f"(within-cond std {pooled_std[exec_rows, t].mean() * mm:.4f} mm) "
              f"effect={effect[exec_rows, t].mean():.2f}")
        g = base + 6
        print(f"  {arm} grip |mean diff| {mean_diff[exec_rows, g].mean():.5f} "
              f"(std {pooled_std[exec_rows, g].mean():.5f}) effect={effect[exec_rows, g].mean():.2f}")
    print(f"\n  z@row4 left diff {mean_diff[4, 2] * mm:.4f} mm (std {pooled_std[4, 2] * mm:.4f}) | "
          f"right diff {mean_diff[4, 9] * mm:.4f} mm (std {pooled_std[4, 9] * mm:.4f})")
    print(f"  worst effect size over all rows/dims: {effect.max():.2f} "
          f"at row/dim {np.unravel_index(effect.argmax(), effect.shape)}")

    # Cumulative displacement over the execute window: does the 5-step motion differ?
    cum30 = a30[:, exec_rows].sum(1)   # (N, 14)
    cum90 = a90[:, exec_rows].sum(1)
    for arm, base in (("left", 0), ("right", 7)):
        t = slice(base, base + 3)
        d = np.abs(cum30.mean(0) - cum90.mean(0))[t] * mm
        s = np.sqrt((cum30.std(0)[t] ** 2 + cum90.std(0)[t] ** 2) / 2).mean() * mm
        print(f"  {arm} 5-step cumulative xyz diff {d.mean():.4f} mm (within-cond std {s:.4f} mm)")

    np.savez("/tmp/ab_policy_results.npz", a30=a30, a90=a90)
    print("\nsaved /tmp/ab_policy_results.npz")


if __name__ == "__main__":
    main()
