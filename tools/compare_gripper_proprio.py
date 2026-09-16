#!/usr/bin/env python3
"""Does the gripper number the policy sees at DEPLOY match the one it was trained on?

For the `griponly` / `mask_velocity_sentinel` pika checkpoints the gripper opening is the ONLY
non-visual input that reaches the model: the 14-D `velocity_grip` state has all 12 velocity dims
replaced by a sentinel (`transforms.MaskStateDims`, `VELOCITY_STATE_DIMS`), leaving dims 6 and 13.
Measured against the served checkpoint on 2026-09-16, that one number moves the commanded approach
z by ~0.31 mm per 1% of jaw opening, with a STEP between roughly 20% and 40% -- and the real jaw
sits inside that step during the final approach. So a distribution mismatch here is worth ~10 mm.

Both sides are directly comparable because both are the raw percent divided by 100:
  training  `observations/<arm>/gripper_synced[:, 0] / 100`
            (openpi `examples/pika_umi/convert_pika_umi_storage_video.py:716,329`)
  deploy    `arms.<arm>.gripper_proprio_pct / 100`
            (policy_runner rollout step log; source selected by --gripper-proprio-source)

WARNING: rollout logs from BEFORE the 2026-09-16 unit fix carry percent of the motor range in those
fields, not millimetres, and are NOT comparable to the collection side. They over-report the opening
by up to +4.85 mm around a 23 mm jaw. See docs/reports/rgb_depth_cue_and_vision_attribution_20260916.md.

Two views, because the marginal histogram is the misleading one -- deploy and demo spend different
amounts of time idle, which shifts the marginal without any calibration error at all:

  marginal   whole-run percentiles and band occupancy. Good for spotting a hard range/offset gap
             (e.g. demos reach 74% open, the robot never exceeds 55%).
  aligned    the jaw trajectory in the 2 s BEFORE each close event, aligned on the crossing. This
             is the window where the model is deciding whether it has grasped, so this is the
             comparison that carries the 10 mm.

Usage
    tools/compare_gripper_proprio.py \
        --episodes ~/Downloads/data_20260902_221844/episode_*.hdf5 \
        --rollout outputs/sweep/20260916_172527_boltv2_griponly_devjit_40k.jsonl
"""

from __future__ import annotations

import argparse
import glob
import json
import pathlib
import sys

import numpy as np

ARMS = ("left", "right")
# Commanded-z sensitivity to the state's jaw value, measured on the served checkpoint 2026-09-16:
# mm of commanded approach z per 1 UNIT of the jaw number. Unit-agnostic (it was measured by varying
# the number handed to the model), so it reads as "per mm of jaw" now that gripper.units is sdk_mm.
MM_PER_PCT = 0.31
# The band where that sensitivity lives (below it the model commands a lift, above it ~nothing).
STEP_BAND = (20.0, 40.0)
BANDS = [(-100, 0), (0, 10), (10, 20), (20, 30), (30, 40), (40, 55), (55, 70), (70, 200)]


def collection_series(paths: list[str], decimate: int) -> dict[str, list[np.ndarray]]:
    """Per-arm list of per-episode jaw traces, decimated the way the converter decimates."""
    import h5py

    out: dict[str, list[np.ndarray]] = {a: [] for a in ARMS}
    for p in paths:
        try:
            f = h5py.File(p, "r")
        except OSError as exc:
            print(f"# skip {p}: {exc}", file=sys.stderr)
            continue
        with f:
            for arm in ARMS:
                g = f[f"observations/{arm}"]
                key = "gripper_synced" if "gripper_synced" in g else "gripper"
                out[arm].append(np.nan_to_num(np.asarray(g[key], dtype=np.float64)[:, 0])[::decimate])
    return out


def rollout_series(path: str) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], str]:
    """Per-arm jaw trace actually fed to the model, plus the measured trace and the source label."""
    prop: dict[str, list[float]] = {a: [] for a in ARMS}
    meas: dict[str, list[float]] = {a: [] for a in ARMS}
    source = "?"
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            arms = rec.get("arms")
            if not isinstance(arms, dict):
                continue
            for arm in ARMS:
                a = arms.get(arm)
                if not isinstance(a, dict):
                    continue
                v = a.get("gripper_proprio_pct")
                if isinstance(v, (int, float)):
                    prop[arm].append(float(v))
                    source = str(a.get("gripper_proprio_source", source))
                m = a.get("gripper_meas_pct")
                if isinstance(m, (int, float)):
                    meas[arm].append(float(m))
    return ({a: np.asarray(v) for a, v in prop.items()},
            {a: np.asarray(v) for a, v in meas.items()}, source)


SANE = 1e3  # a jaw percent outside this is corruption, not a reading


def split_sane(v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(clean, corrupt). The 2026-09-16 rollout fed the model 1.5e156 for 3 consecutive ticks, which
    silently destroyed every percentile until this split existed."""
    bad = ~np.isfinite(v) | (np.abs(v) > SANE)
    return v[~bad], v[bad]


def band_table(label: str, series: dict[str, np.ndarray]) -> None:
    hdr = "".join(f"{lo:>4.0f}-{hi:<4.0f}" for lo, hi in BANDS)
    print(f"{label:<26}{'n':>8}{'p1':>7}{'p50':>7}{'p99':>7}{'max':>7}   {hdr}")
    for arm in ARMS:
        v, bad = split_sane(series[arm])
        if v.size == 0:
            print(f"  {arm:<24}{'-':>8}")
            continue
        if bad.size:
            print(f"  !! {arm}: {bad.size} CORRUPT sample(s) fed to the model, e.g. {bad[0]:.3g} "
                  f"-- excluded from the stats below")
        occ = "".join(f"{100 * np.mean((v >= lo) & (v < hi)):>8.1f}" for lo, hi in BANDS)
        print(f"  {arm:<24}{v.size:>8d}{np.percentile(v,1):>7.1f}{np.percentile(v,50):>7.1f}"
              f"{np.percentile(v,99):>7.1f}{v.max():>7.1f}   {occ}")


def proprio_vs_measured(prop: dict[str, np.ndarray], meas: dict[str, np.ndarray]) -> None:
    """With --gripper-proprio-source actual these should be the same number. Where they are not is
    worth reporting, because the divergence concentrates in the close (the steep band)."""
    print("\n=== proprio vs measured (same tick; they should agree when source=actual)")
    print(f"  {'arm':<8}{'exact':>9}{'|d|>2%':>9}{'p99 |d|':>10}{'jaw at those ticks':>22}")
    for arm in ARMS:
        a, b = prop[arm], meas[arm]
        n = min(a.size, b.size)
        if n == 0:
            continue
        a, b = a[:n], b[:n]
        ok = np.isfinite(a) & np.isfinite(b) & (np.abs(a) < SANE) & (np.abs(b) < SANE)
        d = np.abs(a[ok] - b[ok])
        big = d > 2.0
        jaw = float(np.median(b[ok][big])) if big.any() else float("nan")
        print(f"  {arm:<8}{100 * np.mean(d < 1e-9):>8.1f}%{100 * np.mean(big):>8.1f}%"
              f"{np.percentile(d, 99):>10.2f}{jaw:>21.1f}%")


def close_windows(v: np.ndarray, hz: float, pre_s: float) -> list[np.ndarray]:
    """Traces of length pre_s ending at each 40% -> 15% crossing."""
    n = int(round(pre_s * hz))
    out = []
    i = 1
    while i < len(v):
        if v[i] <= 15.0 < v[i - 1] or (v[i] <= 15.0 and v[i - 1] > 40.0):
            if i - n >= 0 and v[max(0, i - n) : i].max() > 40.0:
                out.append(v[i - n : i])
            i += n
        i += 1
    return out


def aligned_report(label: str, wins: dict[str, list[np.ndarray]], hz: float, pre_s: float) -> dict:
    stats = {}
    print(f"\n{label}  (jaw % at T-x seconds before the 15% crossing)")
    marks = [1.5, 1.0, 0.7, 0.5, 0.3, 0.15, 0.0]
    print(f"  {'arm':<8}{'events':>8}" + "".join(f"{f'T-{m:.2f}':>9}" for m in marks))
    for arm in ARMS:
        w = wins[arm]
        if not w:
            print(f"  {arm:<8}{0:>8}")
            stats[arm] = None
            continue
        W = np.stack(w)
        n = W.shape[1]
        vals = []
        for m in marks:
            idx = int(round(n - 1 - m * hz))
            idx = max(0, min(n - 1, idx))
            vals.append(float(np.median(W[:, idx])))
        stats[arm] = dict(zip([f"T-{m:.2f}" for m in marks], vals))
        print(f"  {arm:<8}{len(w):>8}" + "".join(f"{v:>9.1f}" for v in vals))
    return stats


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--episodes", nargs="+", required=True, help="collection HDF5 (globs ok)")
    ap.add_argument("--rollout", nargs="+", required=True, help="policy_runner rollout step log(s)")
    ap.add_argument("--decimate", type=int, default=3, help="90 Hz capture -> 30 Hz, as the converter does")
    ap.add_argument("--capture-hz", type=float, default=30.0, help="rate AFTER decimation")
    ap.add_argument("--rollout-hz", type=float, default=None, help="default: derived from the log's t_mono")
    ap.add_argument("--pre-s", type=float, default=1.5)
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    eps = [p for pat in args.episodes for p in sorted(glob.glob(pat))] or args.episodes
    rolls = [p for pat in args.rollout for p in sorted(glob.glob(pat))] or args.rollout

    coll = collection_series(eps, args.decimate)
    coll_flat = {a: np.concatenate(v) if v else np.array([]) for a, v in coll.items()}

    prop_all: dict[str, list[np.ndarray]] = {a: [] for a in ARMS}
    meas_all: dict[str, list[np.ndarray]] = {a: [] for a in ARMS}
    hz_seen, source = [], "?"
    for r in rolls:
        p, m, src = rollout_series(r)
        source = src
        for a in ARMS:
            if p[a].size:
                prop_all[a].append(p[a])
            if m[a].size:
                meas_all[a].append(m[a])
        if args.rollout_hz is None:
            with open(r) as fh:
                ts = [json.loads(ln).get("t_mono") for ln in fh if ln.strip()][:5000]
            ts = [t for t in ts if isinstance(t, (int, float))]
            if len(ts) > 100:
                hz_seen.append(1.0 / max(np.median(np.diff(ts)), 1e-6))
    roll_hz = args.rollout_hz or (float(np.median(hz_seen)) if hz_seen else 30.0)
    prop = {a: np.concatenate(v) if v else np.array([]) for a, v in prop_all.items()}
    meas = {a: np.concatenate(v) if v else np.array([]) for a, v in meas_all.items()}

    print(f"# collection: {len(eps)} episodes, decimated 1/{args.decimate} -> {args.capture_hz:.0f} Hz")
    print(f"# deploy:     {len(rolls)} rollout log(s), {roll_hz:.0f} Hz, "
          f"gripper_proprio_source = {source}\n")
    print("=== marginal distribution (% of samples per jaw band)")
    band_table("collection (training)", coll_flat)
    band_table("deploy (proprio fed)", prop)
    if any(v.size for v in meas.values()):
        band_table("deploy (measured)", meas)

    print(f"\n# the {STEP_BAND[0]:.0f}-{STEP_BAND[1]:.0f}% band is where commanded z has its step "
          f"(~{MM_PER_PCT:.2f} mm per 1%)")
    if any(v.size for v in meas.values()):
        proprio_vs_measured(prop, meas)

    hz_c, hz_r = args.capture_hz, roll_hz
    cw = {a: [w for tr in coll[a] for w in close_windows(tr, hz_c, args.pre_s)] for a in ARMS}
    rw = {a: close_windows(split_sane(prop[a])[0], hz_r, args.pre_s) for a in ARMS}
    sc = aligned_report("=== aligned on the close: COLLECTION", cw, hz_c, args.pre_s)
    sr = aligned_report("=== aligned on the close: DEPLOY", rw, hz_r, args.pre_s)

    print("\n=== gap (deploy - collection), and what it is worth in commanded z")
    for arm in ARMS:
        if not sc.get(arm) or not sr.get(arm):
            print(f"  {arm:<8} (not enough close events on one side)")
            continue
        keys = list(sc[arm])
        d = [sr[arm][k] - sc[arm][k] for k in keys]
        print(f"  {arm:<8}" + "".join(f"{v:>9.1f}" for v in d) + "   pct")
        print(f"  {'':<8}" + "".join(f"{v * MM_PER_PCT:>9.1f}" for v in d) + "   mm of commanded z")

    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(
            {"collection_aligned": sc, "deploy_aligned": sr, "source": source,
             "rollout_hz": roll_hz, "mm_per_pct": MM_PER_PCT}, indent=1))
        print(f"\n# wrote {args.json}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
