"""Per-run readout for the 8001 prefetch/RTC A/B campaign.

Usage: python3 tools/analyze_rtc_ab.py outputs/sweep/<stamp>_<tag>.jsonl [more.jsonl ...]

Prints, per run: latency percentiles (+ prefetch_at=1 budget check), RTC telemetry
(realized_delay histogram, delay_error, alignment outcomes), stall/hold, chunk
boundary kick (cmd position step at boundary vs within-chunk, and direction
reversal rates), and gripper command swings. Pair with operator-scored success.
"""
import json
import sys

import numpy as np


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            try:
                d = json.loads(line)
            except Exception:
                continue
            a = d.get("arms") or {}
            if (isinstance(a.get("left"), dict) and isinstance(a.get("right"), dict)
                    and a["left"].get("cmd_pose") and a["right"].get("cmd_pose")):
                rows.append(d)
    return rows


def analyze(path):
    rows = load(path)
    if len(rows) < 20:
        print(f"{path}: only {len(rows)} usable rows, skipping")
        return
    print(f"\n=== {path} ({len(rows)} steps, {rows[-1]['t_mono']-rows[0]['t_mono']:.0f}s) ===")

    inf = np.array([r.get("inference_latency_ms") or np.nan for r in rows], dtype=float)
    inf = inf[~np.isnan(inf)]
    p50, p95, p99 = (np.percentile(inf, q) for q in (50, 95, 99))
    print(f"latency ms: p50={p50:.1f} p95={p95:.1f} p99={p99:.1f} max={inf.max():.1f}")
    print(f"  prefetch_at=1 budget (100.2ms): p95 {'OK' if p95 < 100.2 else 'EXCEEDED'} "
          f"({(inf > 100.2).mean()*100:.1f}% of steps over)")

    rtcs = [r.get("rtc") for r in rows if r.get("rtc")]
    if rtcs:
        rd = {}
        for x in rtcs:
            rd[x.get("realized_delay")] = rd.get(x.get("realized_delay"), 0) + 1
        de = np.array([x.get("delay_error") for x in rtcs if x.get("delay_error") is not None], dtype=float)
        ao = {}
        for x in rtcs:
            ao[x.get("alignment_outcome")] = ao.get(x.get("alignment_outcome"), 0) + 1
        print(f"rtc: realized_delay={dict(sorted(rd.items(), key=lambda kv: -kv[1]))} "
              f"delay_error mean={de.mean():.2f} | alignment={ao}")

    stall = sum(1 for r in rows if r.get("stall"))
    hold = sum(1 for r in rows if r.get("hold"))
    print(f"stall={stall} hold={hold} ({stall/len(rows)*100:.2f}% / {hold/len(rows)*100:.2f}%)")

    chunk_ids = np.array([r.get("chunk_id", -1) for r in rows])
    boundary = np.zeros(len(rows), dtype=bool)
    boundary[1:] = chunk_ids[1:] != chunk_ids[:-1]

    for side in ("left", "right"):
        pos = np.array([r["arms"][side]["cmd_pose"][:3] for r in rows], dtype=float)
        step = np.diff(pos, axis=0)                      # step i = pos[i+1]-pos[i]
        mag = np.linalg.norm(step, axis=1) * 1000        # mm
        # boundary flag for step i corresponds to rows[i+1] starting a new chunk
        bmask = boundary[1:]
        moving = mag > 0.05                              # ignore parked phases
        b_mag = mag[bmask & moving]
        w_mag = mag[~bmask & moving]
        # direction reversal: consecutive steps with negative dot product
        dots = (step[1:] * step[:-1]).sum(axis=1)
        rev = dots < 0
        rev_b = rev[bmask[1:] & moving[1:] & moving[:-1]]
        rev_w = rev[~bmask[1:] & moving[1:] & moving[:-1]]
        if len(b_mag) and len(w_mag):
            print(f"{side}: boundary |dcmd| p50={np.percentile(b_mag,50):.3f} p95={np.percentile(b_mag,95):.3f}mm "
                  f"vs within p50={np.percentile(w_mag,50):.3f}mm "
                  f"(ratio p50 {np.percentile(b_mag,50)/max(np.percentile(w_mag,50),1e-9):.2f}x) | "
                  f"reversal at boundary {rev_b.mean()*100 if len(rev_b) else float('nan'):.1f}% "
                  f"vs within {rev_w.mean()*100 if len(rev_w) else float('nan'):.1f}%")
        g = np.array([r["arms"][side].get("gripper_cmd_pct", np.nan) for r in rows], dtype=float)
        gd = np.abs(np.diff(g))
        print(f"{side}: grip swings >20%={int(np.nansum(gd>20))} >40%={int(np.nansum(gd>40))} "
              f"per-1000-steps={np.nansum(gd>20)/len(rows)*1000:.1f}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyze(p)
