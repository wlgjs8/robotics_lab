#!/usr/bin/env python3
"""Model-side chunk seam metrics from policy_runner chunk_rows.v1 logs.
For consecutive chunk records k, k+1 (n = rows elapsed between them), compare the executable
absolute rows: position jump at activation |p_{k+1}[0] - p_k[n]|, velocity change at the seam
|v_{k+1}[0] - v_k[n]| (v = per-row difference / policy_dt), against interior row-to-row velocity
changes inside a chunk. Reported per arm in mm and mm/s; rotation via quaternion angle."""
import json, sys, math, numpy as np
def qang(q1, q2):
    d = abs(float(np.dot(q1, q2))); d = min(1.0, d); return 2*math.acos(d)
def analyze(path):
    recs = [json.loads(l) for l in open(path)]
    recs = [r for r in recs if r.get('left') and r.get('right')]
    out = {}
    dt = float(np.median([r['policy_dt_sec'] for r in recs]))
    rtc = set(bool(r.get('rtc_enabled')) for r in recs)
    act = set(r.get('chunk_metadata', {}).get('activation_mode') for r in recs)
    for arm in ('left', 'right'):
        pj, vseam, vint, aseam, aint, gseam, shifts, speeds = [], [], [], [], [], [], [], []
        for a, b in zip(recs[:-1], recs[1:]):
            A = np.array(a[arm], float); B = np.array(b[arm], float)
            n = int(round((b['t_mono'] - a['t_mono']) / dt))
            if n < 1 or n + 1 >= len(A) or len(B) < 2: continue
            shifts.append(n)
            pA, pB = A[:, :3], B[:, :3]
            pj.append(np.linalg.norm(pB[0] - pA[n]) * 1000)
            vA = (pA[n+1] - pA[n]) / dt; vA0 = (pA[n] - pA[n-1]) / dt
            vB = (pB[1] - pB[0]) / dt
            vseam.append(np.linalg.norm(vB - vA0) * 1000)   # velocity entering vs leaving the seam
            speeds.append(np.linalg.norm(vA0) * 1000)
            # interior: consecutive velocity changes inside chunk A
            vs = np.diff(pA, axis=0) / dt
            for i in range(1, len(vs)): vint.append(np.linalg.norm(vs[i] - vs[i-1]) * 1000)
            qA, qB = A[:, 3:7], B[:, 3:7]
            aseam.append(math.degrees(qang(qB[0], qA[n])))
            for i in range(1, len(qA)): aint.append(math.degrees(qang(qA[i], qA[i-1])))
            gseam.append(abs(B[0, 7] - A[n, 7]))
        def q(x, p): return float(np.quantile(x, p)) if len(x) else float('nan')
        out[arm] = dict(n=len(pj), shift_p50=q(shifts, .5),
            pos_jump_mm=(q(pj,.5), q(pj,.9), max(pj) if pj else float('nan')),
            dv_seam_mms=(q(vseam,.5), q(vseam,.9), q(vseam,.99)),
            dv_interior_mms=(q(vint,.5), q(vint,.9), q(vint,.99)),
            speed_mms_p50=q(speeds,.5),
            rot_jump_deg=(q(aseam,.5), q(aseam,.9)), rot_interior_step_deg=(q(aint,.5), q(aint,.9)),
            grip_jump=(q(gseam,.5), q(gseam,.9)))
    return dict(path=path, chunks=len(recs), dt=dt, rtc=rtc, activation=act, arms=out)
if __name__ == '__main__':
    for p in sys.argv[1:]:
        r = analyze(p)
        print(f"\n=== {p.split('/')[-1]}  chunks={r['chunks']} dt={r['dt']:.4f} rtc={r['rtc']} act={r['activation']}")
        for arm, o in r['arms'].items():
            print(f"  {arm:5} n={o['n']:4d} shift={o['shift_p50']:.0f} | pos jump mm p50/p90/max {o['pos_jump_mm'][0]:.2f}/{o['pos_jump_mm'][1]:.2f}/{o['pos_jump_mm'][2]:.1f}"
                  f" | dv seam mm/s p50/p90/p99 {o['dv_seam_mms'][0]:.0f}/{o['dv_seam_mms'][1]:.0f}/{o['dv_seam_mms'][2]:.0f}"
                  f" | dv interior {o['dv_interior_mms'][0]:.0f}/{o['dv_interior_mms'][1]:.0f}/{o['dv_interior_mms'][2]:.0f}"
                  f" | speed p50 {o['speed_mms_p50']:.0f} | rot jump deg p50/p90 {o['rot_jump_deg'][0]:.2f}/{o['rot_jump_deg'][1]:.2f} (interior step {o['rot_interior_step_deg'][0]:.2f}/{o['rot_interior_step_deg'][1]:.2f})"
                  f" | grip jump p50/p90 {o['grip_jump'][0]:.2f}/{o['grip_jump'][1]:.2f}")
