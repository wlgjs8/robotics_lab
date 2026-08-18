#!/usr/bin/env python3
"""reinfer — re-run recorded live inferences offline with ONE input changed, and diff the chunks.

Run it with the SAME interpreter flow-infer uses (it needs the openpi websocket client):
    /home/plaif/workspace/openpi/.venv/bin/python cm_bridge/tools/reinfer.py \\
        --dump outputs/obs_dump/<stamp>_<tag> --server openpi://127.0.0.1:8001 \\
        --seq 120 --repeat 3 --vel-scale 0.5,0.75,1.0,1.25,1.5

What it does, per recorded inference (`--seq N`, `--seqs A:B`, or `--all`):
  1. rebuilds the EXACT observation that was sent (lossless PNG wrist images, state vector,
     prompt, and — if RTC was on — the same prev_action_chunk / delay / horizon / schedule);
  2. `--repeat R` sends it unchanged R times: the spread between those answers is the sampling
     NOISE FLOOR of the served policy, printed first so every other difference is read against it;
  3. `--vel-scale k1,k2,...` multiplies the VELOCITY dims of observation/state (per --arm/--part)
     and re-infers each variant; `--set i=v` overrides single state entries instead;
  4. prints, per variant, the chunk difference against the RECORDED answer: per-arm mean/max
     translation delta difference [mm], rotation [deg], gripper [%], and the mean cosine between
     the recorded and new per-step translation deltas (direction agreement). --csv/--json save it.

The recorded answer is the model-space `actions` the server returned then (gripper as fraction);
the comparison uses the same space, so units are: translation m/step (printed as mm), rotation
rad/step rotvec (printed as deg), gripper fraction (printed as %). The state layout comes from the
manifest's proprio_mode:
    velocity       (12) [vL(6) | vR(6)]
    velocity_grip  (14) [vL(6) gL | vR(6) gR]
    velocity_grav  (20) [vL(6) gravL(3) gL | vR(6) gravR(3) gR]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "policy_runner"))

_LAYOUT = {  # proprio_mode -> (state_dim, {arm: (vel_slice, grip_index)})
    "velocity": (12, {"left": (slice(0, 6), None), "right": (slice(6, 12), None)}),
    "velocity_grip": (14, {"left": (slice(0, 6), 6), "right": (slice(7, 13), 13)}),
    "velocity_grav": (20, {"left": (slice(0, 6), 9), "right": (slice(10, 16), 19)}),
}


def load_manifest(dump_dir: str):
    recs, scaled = {}, {}
    with open(os.path.join(dump_dir, "manifest.jsonl"), encoding="utf-8") as f:
        for line in f:
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            if r.get("type") == "inference":
                recs[int(r["seq"])] = r
            elif r.get("type") == "scaled":
                scaled[int(r["seq"])] = r["file"]
    return recs, scaled


def build_obs(dump_dir: str, rec: dict):
    from policy_runner.observation_dump import load_png_rgb
    obs = {}
    for key, fn in rec["files"].items():
        path = os.path.join(dump_dir, fn)
        if key.endswith("_rgb"):
            obs[key] = load_png_rgb(path)
        elif key.endswith("_depth"):
            obs[key] = np.load(path)
    arrays = np.load(os.path.join(dump_dir, rec["files"]["arrays"])) if "arrays" in rec["files"] else {}
    obs["observation/state"] = np.asarray(arrays["state"], dtype=np.float32) if "state" in arrays else np.asarray(rec["state"], dtype=np.float32)
    obs["prompt"] = rec.get("prompt")
    if "prev_action_chunk" in arrays:
        obs["prev_action_chunk"] = np.asarray(arrays["prev_action_chunk"], dtype=np.float32)
        for k in ("inference_delay", "execute_horizon", "prefix_attention_schedule", "max_guidance_weight"):
            if k in rec:
                obs[k] = rec[k]
    recorded = np.asarray(arrays["actions"], dtype=np.float32) if "actions" in arrays else None
    return obs, recorded


def diff_chunks(a: np.ndarray, b: np.ndarray) -> dict:
    """a = recorded, b = new; both (H, 14) model space."""
    out = {}
    h = min(len(a), len(b))
    a, b = a[:h], b[:h]
    for arm, off in (("left", 0), ("right", 7)):
        ta, tb = a[:, off:off + 3], b[:, off:off + 3]
        ra, rb = a[:, off + 3:off + 6], b[:, off + 3:off + 6]
        dt = np.linalg.norm(ta - tb, axis=1) * 1e3
        dr = np.rad2deg(np.linalg.norm(ra - rb, axis=1))
        dg = np.abs(a[:, off + 6] - b[:, off + 6]) * 100.0
        na, nb = np.linalg.norm(ta, axis=1), np.linalg.norm(tb, axis=1)
        ok = (na > 1e-6) & (nb > 1e-6)
        cos = float(np.mean(np.sum(ta[ok] * tb[ok], axis=1) / (na[ok] * nb[ok]))) if ok.any() else float("nan")
        out[arm] = {
            "trans_mm_mean": float(dt.mean()), "trans_mm_max": float(dt.max()),
            "rot_deg_mean": float(dr.mean()), "rot_deg_max": float(dr.max()),
            "grip_pct_mean": float(dg.mean()), "grip_pct_max": float(dg.max()),
            "trans_cos": cos,
            "rec_speed_mm": float(na.mean() * 1e3), "new_speed_mm": float(nb.mean() * 1e3),
        }
    return out


def fmt(d: dict) -> str:
    return " | ".join(
        f"{arm[0].upper()}: dT {v['trans_mm_mean']:.2f}/{v['trans_mm_max']:.2f}mm dR {v['rot_deg_mean']:.2f}/{v['rot_deg_max']:.2f}° "
        f"dG {v['grip_pct_mean']:.1f}/{v['grip_pct_max']:.1f}% cos {v['trans_cos']:.3f} |δ| {v['rec_speed_mm']:.2f}->{v['new_speed_mm']:.2f}mm"
        for arm, v in d.items())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", required=True, help="observation dump directory (manifest.jsonl inside)")
    ap.add_argument("--server", default="openpi://127.0.0.1:8001")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--seq", type=int, help="one inference_seq")
    g.add_argument("--seqs", help="range A:B (inclusive)")
    g.add_argument("--all", action="store_true")
    ap.add_argument("--repeat", type=int, default=2, help="unchanged re-inferences for the noise floor")
    ap.add_argument("--samples", type=int, default=1,
                    help="inferences per VARIANT, averaged (the served policy is stochastic: with 1 sample a "
                         "variant's difference is mostly the noise floor; average N and compare the variant MEAN "
                         "chunk against the unchanged-input MEAN chunk of the same N)")
    ap.add_argument("--vel-scale", default="", help="comma list of multipliers on the velocity dims")
    ap.add_argument("--arm", choices=("both", "left", "right"), default="both")
    ap.add_argument("--part", choices=("all", "trans", "rot"), default="all")
    ap.add_argument("--set", action="append", default=[], metavar="IDX=VAL",
                    help="explicit state override(s), applied to every variant")
    ap.add_argument("--csv", help="write per-variant rows here")
    ap.add_argument("--json", help="write everything here")
    args = ap.parse_args()

    recs, _scaled = load_manifest(args.dump)
    if not recs:
        ap.error(f"no inference records in {args.dump}/manifest.jsonl")
    if args.seq is not None:
        seqs = [args.seq]
    elif args.seqs:
        a, b = (int(x) for x in args.seqs.split(":"))
        seqs = [s for s in sorted(recs) if a <= s <= b]
    elif args.all:
        seqs = sorted(recs)
    else:
        seqs = [sorted(recs)[len(recs) // 2]]
        print(f"[reinfer] no --seq given: using the middle one, seq {seqs[0]}")
    scales = [float(x) for x in args.vel_scale.split(",") if x.strip()]
    overrides = []
    for s in args.set:
        i, v = s.split("=")
        overrides.append((int(i), float(v)))

    from policy_runner.openpi_remote import _OpenpiWebsocketClient
    uri = args.server
    if uri.startswith("openpi://"):          # the flow-infer spelling -> the websocket URI
        uri = "ws://" + uri[len("openpi://"):]
    client = _OpenpiWebsocketClient(uri)
    rows, everything = [], []
    for seq in seqs:
        rec = recs[seq]
        obs, recorded = build_obs(args.dump, rec)
        mode = str(rec.get("proprio_mode", ""))
        layout = _LAYOUT.get(mode)
        state0 = np.array(obs["observation/state"], dtype=np.float32)
        print(f"\n=== seq {seq}  proprio_mode={mode} state_dim={len(state0)} rtc={'prev_action_chunk' in obs} "
              f"images={[k.split('/')[-1] for k in obs if k.startswith('observation/') and 'rgb' in k]}")
        print(f"    state = {np.round(state0, 4).tolist()}")
        if recorded is None:
            print("    (no recorded actions in the dump - skipping)"); continue

        def run(state: np.ndarray):
            o = dict(obs); o["observation/state"] = np.asarray(state, dtype=np.float32)
            t0 = time.monotonic()
            res = client.infer(o)
            return np.asarray(res.get("actions"), dtype=np.float32)[:, :14], time.monotonic() - t0

        # 1. noise floor (single re-inferences of the unchanged input vs the recorded answer)
        floor = []
        for r in range(max(0, args.repeat)):
            act, dt = run(state0)
            d = diff_chunks(recorded, act); floor.append(d)
            print(f"  [repeat {r+1}] {dt*1e3:5.0f} ms  {fmt(d)}")
            rows.append({"seq": seq, "variant": f"repeat{r+1}", "ref": "recorded", **_flat(d)})
        if floor:
            fl_t = max(v["trans_mm_max"] for d in floor for v in d.values())
            fl_g = max(v["grip_pct_max"] for d in floor for v in d.values())
            print(f"  noise floor: max dT {fl_t:.2f} mm, max dG {fl_g:.1f} %  <- read the variants against this")
        # baseline MEAN chunk for the averaged comparison (only when --samples > 1)
        n_s = max(1, int(args.samples))
        base_mean = None
        if n_s > 1:
            acts = [run(state0)[0] for _ in range(n_s)]
            base_mean = np.mean(acts, axis=0)
            spread = diff_chunks(base_mean, acts[0])
            print(f"  [baseline mean of {n_s}] one sample vs the mean: {fmt(spread)}")

        # 2. variants
        variants = []
        for k in scales:
            st = state0.copy()
            if layout is None:
                print(f"  --vel-scale needs a velocity proprio_mode (got {mode!r}); skipping scale {k}"); break
            for arm, (sl, _g) in layout[1].items():
                if args.arm != "both" and arm != args.arm:
                    continue
                idx = list(range(sl.start, sl.stop))
                if args.part == "trans": idx = idx[:3]
                elif args.part == "rot": idx = idx[3:]
                st[idx] *= k
            variants.append((f"vel x{k:g} ({args.arm}/{args.part})", st))
        if overrides:
            st = state0.copy()
            for i, v in overrides: st[i] = v
            variants.append(("set " + ",".join(f"{i}={v:g}" for i, v in overrides), st))
        for name, st in variants:
            acts, dts = [], []
            for _ in range(n_s):
                a_, dt_ = run(st); acts.append(a_); dts.append(dt_)
            act = np.mean(acts, axis=0) if n_s > 1 else acts[0]
            d = diff_chunks(recorded, act)
            print(f"  [{name:24s}] {np.mean(dts)*1e3:5.0f} ms  vs recorded: {fmt(d)}")
            rows.append({"seq": seq, "variant": name, "ref": "recorded", **_flat(d)})
            if base_mean is not None:
                d2 = diff_chunks(base_mean, act)
                print(f"  {'':26s}         vs baseline mean({n_s}): {fmt(d2)}")
                rows.append({"seq": seq, "variant": name, "ref": f"baseline_mean{n_s}", **_flat(d2)})
            everything.append({"seq": seq, "variant": name, "samples": n_s, "state": st.tolist(),
                               "actions_mean": act.tolist(), "diff_vs_recorded": d,
                               "diff_vs_baseline_mean": (diff_chunks(base_mean, act) if base_mean is not None else None)})
    if args.csv and rows:
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
        print(f"[reinfer] csv -> {args.csv}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(everything, f)
        print(f"[reinfer] json -> {args.json}")
    return 0


def _flat(d: dict) -> dict:
    out = {}
    for arm, v in d.items():
        for k, x in v.items():
            out[f"{arm}_{k}"] = x
    return out


if __name__ == "__main__":
    sys.exit(main())
