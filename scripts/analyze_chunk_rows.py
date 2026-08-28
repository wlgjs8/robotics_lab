#!/usr/bin/env python3
"""Spectrum of the POLICY's own predicted trajectory, from a chunk-row log.

Answers the one question the servo log cannot: is a command-band oscillation
the model's, or something the controller added?

Context (2026-08-28). A rollout vibrated at 11-13 Hz and every control stage was
cleared from the servo log -- chunk follower 0.80x on that band, output SMD
0.73x, IK residual 0.8 um with branch_jump 0, force-control deviation 0.16 mm
rms dominated by 1.1 Hz, robot structure AMPLIFYING 1.07-1.71x (J3 worst). What
was left was 180 um of 11-13 Hz already present in the command entering the
follower. That pointed at the policy, but could not be confirmed: the chunk rows
were published and drawn and never written down. policy_runner now always writes
them; this reads them.

The executed trajectory is the first `execute_limit` rows of each chunk,
concatenated -- the rows the arm actually ran, sampled at 1/policy_dt (~30 Hz,
Nyquist ~15 Hz, so a 10-14 Hz band is measurable but close to the edge; treat
anything above ~13 Hz as indicative rather than exact).

Usage:
  scripts/analyze_chunk_rows.py outputs/sweep/<stamp>.chunks.jsonl [--band 10 14]
"""
from __future__ import annotations

import argparse
import json
import math
import sys


def _load(path):
    chunks = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                chunks.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return chunks


def _executed_track(chunks, arm):
    """Concatenate the executed window of every chunk into one xyz track."""
    xs, ys, zs = [], [], []
    for c in chunks:
        rows = c.get(arm)
        if not rows:
            continue
        limit = int(c.get("execute_limit") or 0)
        if limit <= 0:
            continue
        for row in rows[:limit]:
            if not row or len(row) < 3:
                continue
            xs.append(float(row[0]))
            ys.append(float(row[1]))
            zs.append(float(row[2]))
    return xs, ys, zs


def _detrend(v, order=3):
    n = len(v)
    if n < order + 2:
        return [x - (sum(v) / n) for x in v]
    # Least squares on a Vandermonde basis, without numpy.
    import numpy as np

    t = np.arange(n, dtype=float)
    a = np.asarray(v, dtype=float)
    return list(a - np.polyval(np.polyfit(t, a, order), t))


def _band_rms(v, fs, lo, hi):
    import numpy as np

    n = len(v)
    if n < 32:
        return float("nan")
    x = np.asarray(_detrend(v), dtype=float)
    spec = np.fft.rfft(x)
    freq = np.fft.rfftfreq(n, 1.0 / fs)
    keep = np.where((freq >= lo) & (freq < hi), spec, 0)
    return float(np.std(np.fft.irfft(keep, n=n)))


def _peak(v, fs, lo=1.0):
    import numpy as np

    n = len(v)
    x = np.asarray(_detrend(v), dtype=float)
    mag = np.abs(np.fft.rfft(x * np.hanning(n)))
    freq = np.fft.rfftfreq(n, 1.0 / fs)
    sel = freq >= lo
    if not sel.any():
        return float("nan")
    return float(freq[sel][int(np.argmax(mag[sel]))])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--band", nargs=2, type=float, default=[10.0, 14.0])
    args = ap.parse_args(argv)

    chunks = _load(args.log)
    if not chunks:
        print(f"no chunk records in {args.log}", file=sys.stderr)
        return 1
    dts = [c.get("policy_dt_sec") for c in chunks if c.get("policy_dt_sec")]
    dt = dts[0] if dts else 1.0 / 30.0
    fs = 1.0 / dt
    lo, hi = args.band
    print(f"{args.log}: {len(chunks)} chunks, policy_dt={dt*1000:.2f} ms "
          f"(fs={fs:.1f} Hz, Nyquist {fs/2:.1f} Hz)")
    print(f"  anchor={chunks[0].get('anchor_mode')} stitch={chunks[0].get('stitch_mode')} "
          f"execute_limit={chunks[0].get('execute_limit')} runway={chunks[0].get('runway_steps')}")
    if hi > fs / 2:
        print(f"  NOTE: band top {hi:.1f} Hz is above Nyquist; results are aliased")
    for arm in ("left", "right"):
        xs, ys, zs = _executed_track(chunks, arm)
        if len(xs) < 32:
            print(f"\n  {arm}: {len(xs)} executed rows (too few)")
            continue
        print(f"\n  {arm}: {len(xs)} executed rows ({len(xs)*dt:.2f} s of plan)")
        tot = 0.0
        for name, v in (("x", xs), ("y", ys), ("z", zs)):
            rms = _band_rms(v, fs, lo, hi) * 1e6
            tot += rms * rms
            print(f"    {name}: {lo:.0f}-{hi:.0f} Hz rms = {rms:8.1f} um   peak = {_peak(v, fs):5.2f} Hz")
        print(f"    |xyz| {lo:.0f}-{hi:.0f} Hz rms = {math.sqrt(tot):8.1f} um")
        # Step-to-step geometry: a smooth plan turns little between steps.
        import numpy as np

        p = np.stack([np.asarray(xs), np.asarray(ys), np.asarray(zs)], axis=1)
        d = np.diff(p, axis=0)
        step = np.linalg.norm(d, axis=1)
        d2 = np.linalg.norm(np.diff(d, axis=0), axis=1)
        cos = []
        for i in range(len(d) - 1):
            na, nb = np.linalg.norm(d[i]), np.linalg.norm(d[i + 1])
            if na > 1e-9 and nb > 1e-9:
                cos.append(float(np.dot(d[i], d[i + 1]) / (na * nb)))
        med_step = float(np.median(step)) * 1e6
        med_d2 = float(np.median(d2)) * 1e6
        print(f"    step p50 = {med_step:8.1f} um   2nd-diff p50 = {med_d2:8.1f} um   "
              f"ratio = {med_d2/max(med_step,1e-9):.2f}")
        if cos:
            cos = np.asarray(cos)
            print(f"    direction cos p50 = {float(np.median(cos)):+.3f}   "
                  f"reversals = {int((cos < 0).sum())}/{len(cos)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
