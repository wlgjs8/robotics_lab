#!/usr/bin/env python3
"""Queue-sync health + command-resampling probe for rb_servo_server logs."""
import csv, sys, math
from collections import Counter
import numpy as np

path = sys.argv[1]
f = open(path, newline='')
r = csv.reader(f)
hdr = next(r)
idx = {n: i for i, n in enumerate(hdr)}

want = ['tick','loop_start_time_ns','period_ms','motion_state',
        'left_send_start_ns','right_send_start_ns',
        'left_rback_observed','left_rback_fill','left_rback_seq',
        'right_rback_observed','right_rback_fill','right_rback_seq',
        'left_qsync_phase','left_qsync_trim_us','left_qsync_locked','left_qsync_fill_lpf','left_qsync_integral_us',
        'right_qsync_phase','right_qsync_trim_us','right_qsync_locked','right_qsync_fill_lpf','right_qsync_integral_us',
        'left_qsync_underrun_events','left_qsync_stall_events','left_qsync_highwater_events',
        'left_qsync_redrain_events','left_qsync_no_consumption_events',
        'right_qsync_underrun_events','right_qsync_stall_events','right_qsync_highwater_events',
        'right_qsync_redrain_events','right_qsync_no_consumption_events']
for s in ('left','right'):
    for j in range(6):
        want += [f'{s}_q_sent_{j}', f'{s}_q_ref_{j}', f'{s}_q_actual_{j}']
miss = [w for w in want if w not in idx]
if miss: print("MISSING:", miss[:8])
cols = {w: idx[w] for w in want if w in idx}

rows = []
for row in r:
    if len(row) < len(hdr):  # torn last line
        continue
    rows.append(row)
f.close()
N = len(rows)

def col(name, dtype=float):
    i = cols[name]
    out = np.empty(N, dtype=dtype)
    if dtype is float:
        for k, row in enumerate(rows):
            v = row[i]
            try: out[k] = float(v)
            except Exception: out[k] = np.nan
    return out

def scol(name):
    i = cols[name]
    return [row[i] for row in rows]

t_ns = col('loop_start_time_ns')
dur = (t_ns[-1]-t_ns[0])/1e9
print(f"=== {path}")
print(f"ticks={N}  duration={dur:.1f}s  loop_rate={(N-1)/dur:.3f} Hz")
ms = Counter(scol('motion_state'))
print("motion_state:", dict(ms.most_common(6)))

for arm in ('left','right'):
    print(f"\n--- {arm} ---")
    ph = Counter(scol(f'{arm}_qsync_phase'))
    tot = sum(ph.values())
    print("  phase:", {k: f"{100*v/tot:.2f}%" for k,v in ph.most_common()})
    # actual send cadence: distinct send_start_ns
    ss = col(f'{arm}_send_start_ns')
    ss = ss[np.isfinite(ss)]
    ss = ss[ss > 0]
    uniq = np.unique(ss)
    if uniq.size > 1:
        span = (uniq[-1]-uniq[0])/1e9
        print(f"  [TRAP §2-1] send_start_ns is the LOOP-side ENQUEUE stamp, NOT the wire send.")
        print(f"  enqueue stamps={uniq.size}  span={span:.1f}s  enqueue_rate={(uniq.size-1)/span:.3f} Hz")
        d = np.diff(uniq)/1e3  # us
        print(f"  send period us: p1={np.percentile(d,1):.1f} med={np.median(d):.1f} "
              f"p99={np.percentile(d,99):.1f} max={d.max():.1f}  >4000us={(d>4000).sum()}")
        gen_rate = (N-1)/dur
        print(f"  GEN {gen_rate:.3f} Hz vs ENQUEUE {(uniq.size-1)/span:.3f} Hz -> reads ~0 drops, "
              f"which is the ARTEFACT. Use drop_rate.py (Delta rback_seq) for the real send rate.")
    # fill
    obs = np.array([x.strip().lower() in ('1','true') for x in scol(f'{arm}_rback_observed')])
    fill = col(f'{arm}_rback_fill')
    seq = col(f'{arm}_rback_seq')
    fresh = np.zeros(N, bool)
    fresh[1:] = (seq[1:] != seq[:-1]) & obs[1:]
    ff = fill[fresh]
    if ff.size:
        h = Counter(ff.astype(int).tolist())
        s = sum(h.values())
        print(f"  RBACK fresh={ff.size} ({100*ff.size/N:.1f}% of ticks)  fill median={np.median(ff):.1f}")
        print("  fill hist:", {k: f"{100*v/s:.2f}%" for k,v in sorted(h.items())[:14]})
        dff = np.diff(ff)
        print(f"  |dfill|>=2 events={int((np.abs(dff)>=2).sum())} "
              f"({(np.abs(dff)>=2).sum()/max(dur,1e-9):.2f}/s)  max|dfill|={int(np.abs(dff).max()) if dff.size else 0}")
    trim = col(f'{arm}_qsync_trim_us')
    tr = trim[np.isfinite(trim)]
    print(f"  trim us: p1={np.percentile(tr,1):.1f} med={np.median(tr):.2f} p99={np.percentile(tr,99):.1f} "
          f"min={tr.min():.1f} max={tr.max():.1f}  |trim|>50us={100*(np.abs(tr)>50).mean():.2f}%")
    integ = col(f'{arm}_qsync_integral_us')
    print(f"  integral us: first={integ[0]:.2f} last={integ[-1]:.2f}")
    for c in ('underrun','stall','highwater','redrain','no_consumption'):
        v = col(f'{arm}_qsync_{c}_events')
        v = v[np.isfinite(v)]
        if v.size and v[-1] > 0:
            print(f"  ** {c}_events: {int(v[0])} -> {int(v[-1])}  (+{int(v[-1]-v[0])} this run)")
