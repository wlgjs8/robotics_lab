#!/usr/bin/env python3
"""Grasp descent depth from rollout step logs. Per arm, each jaw close (cmd < CLOSED_PCT held > MIN_HOLD_S):
z bottom of the measured and commanded TCP in [t_close-1.5 s, t_close+0.5 s], the gap cmd-meas at the bottom,
vertical speed just before the close, and the outcome proxy CARRY (closed segment travels > 0.15 m in xy).
Reference REST_Z = -0.2658 m (stand frame: table -0.295 + foam 0.020 + bolt head radius 0.0092, the sim's calibrated cell)."""
import json, sys, glob, re, numpy as np
REST_Z = -0.2658
CLOSED_PCT, MIN_HOLD_S = 15.0, 0.4
def read(path):
    rows=[]
    for line in open(path):
        try: d=json.loads(line)
        except Exception: continue
        a=d.get('arms')
        if not a: continue
        if all(a.get(s,{}).get('gripper_cmd_pct') is not None and a[s].get('meas_pose') for s in ('left','right')): rows.append(d)
    return rows
def segments(closed,t):
    out=[];i=0
    while i<len(closed):
        if closed[i]:
            j=i
            while j+1<len(closed) and closed[j+1]: j+=1
            if t[j]-t[i]>MIN_HOLD_S: out.append((i,j))
            i=j+1
        else: i+=1
    return out
groups={}
for path in sys.argv[1:]:
    rows=read(path)
    if len(rows)<200: continue
    name=path.split('/')[-1]; m=re.match(r'(\d{8})_\d{6}_(.*)\.jsonl',name); day,model=m.group(1),m.group(2)
    t=np.array([r['t_mono'] for r in rows]); t=t-t[0]
    for side in ('left','right'):
        g=np.array([float(r['arms'][side]['gripper_cmd_pct']) for r in rows])
        gm=np.array([float(r['arms'][side].get('gripper_meas_pct') or np.nan) for r in rows])
        Pm=np.array([r['arms'][side]['meas_pose'][:3] for r in rows]); Pc=np.array([r['arms'][side]['cmd_pose'][:3] for r in rows])
        for i,j in segments(g<CLOSED_PCT,t):
            w=(t>=t[i]-1.5)&(t<=t[i]+0.5)
            if w.sum()<5: continue
            zm=Pm[w,2].min(); zc=Pc[w,2].min(); k=np.argmin(Pm[w,2]); kk=np.where(w)[0][k]
            pre=(t>=t[i]-0.5)&(t<t[i]); vz=(Pm[i,2]-Pm[pre][0,2])/max(t[i]-t[pre][0],1e-3) if pre.sum()>1 else np.nan
            travel=np.linalg.norm(Pm[j,:2]-Pm[i,:2]); carry=travel>0.15
            groups.setdefault((model,day,side),[]).append(dict(z_meas_min=zm,z_cmd_min=zc,gap=(Pc[kk,2]-Pm[kk,2]),z_close=Pm[i,2],vz=vz,carry=carry,travel=travel,min_meas=np.nanmin(gm[i:j+1]),t=t[i],run=name))
def q(x,p): x=np.asarray(x,float); return np.quantile(x,p) if len(x) else np.nan
print(f"{'model':32}{'day':10}{'arm':6}{'n':>4}{'carry':>6} | meas bottom-REST mm p50/p90 (carry | no-carry) | cmd bottom-REST p50 | cmd-meas gap@bottom p50 | vz pre-close mm/s p50")
for (model,day,side),ev in sorted(groups.items()):
    c=[e for e in ev if e['carry']]; nc=[e for e in ev if not e['carry']]
    f=lambda L,k: (q([e[k] for e in L],.5)-REST_Z)*1000 if L else np.nan
    f9=lambda L,k: (q([e[k] for e in L],.9)-REST_Z)*1000 if L else np.nan
    print(f"{model:32}{day:10}{side:6}{len(ev):4d}{len(c):6d} | {f(c,'z_meas_min'):6.1f}/{f9(c,'z_meas_min'):6.1f} | {f(nc,'z_meas_min'):6.1f}/{f9(nc,'z_meas_min'):6.1f} | {f(ev,'z_cmd_min'):6.1f} | {q([e['gap'] for e in ev],.5)*1000:5.1f} | {q([e['vz'] for e in ev],.5)*1000:6.0f}")
# pooled by model/arm
print("\nPOOLED by model/arm (meas bottom - REST_Z, mm):")
pool={}
for (model,day,side),ev in groups.items(): pool.setdefault((model.replace('_30k','_40k'),side),[]).extend(ev)
for (model,side),ev in sorted(pool.items()):
    c=[e['z_meas_min'] for e in ev if e['carry']]; nc=[e['z_meas_min'] for e in ev if not e['carry']]
    allz=[e['z_meas_min'] for e in ev]
    print(f"  {model:32}{side:6} n={len(ev):4d} carry={len(c):3d} | all p10/p50/p90 {(q(allz,.1)-REST_Z)*1000:6.1f}/{(q(allz,.5)-REST_Z)*1000:6.1f}/{(q(allz,.9)-REST_Z)*1000:6.1f} | carry p50 {(q(c,.5)-REST_Z)*1000:6.1f} | no-carry p50 {(q(nc,.5)-REST_Z)*1000:6.1f} | share bottom > +10 mm: {100*np.mean([(z-REST_Z)>0.010 for z in allz]):4.0f}% | cmd-meas gap p50 {q([e['gap'] for e in ev],.5)*1000:4.1f} mm")

print("\nDECOMPOSITION (devjit + griponly, per arm, carry vs no-carry): meas bottom = cmd bottom (model intent) + tracking gap")
print(f"{'model':30}{'arm':6}{'outcome':9}{'n':>4} | cmd bottom-REST p50/p90 | meas-cmd bottom p50 | jaw closed above bottom (z_close - z_min) p50 | vz pre-close p50 | min meas jaw p50")
for (model,side),ev in sorted(pool.items()):
    for label,L in (('carry',[e for e in ev if e['carry']]),('no-carry',[e for e in ev if not e['carry']])):
        if not L: continue
        cb=[(e['z_cmd_min']-REST_Z)*1000 for e in L]; gap=[(e['z_meas_min']-e['z_cmd_min'])*1000 for e in L]; early=[(e['z_close']-e['z_meas_min'])*1000 for e in L]
        vz=[e['vz']*1000 for e in L if not np.isnan(e['vz'])]; mm=[e['min_meas'] for e in L if not np.isnan(e['min_meas'])]
        print(f"{model:30}{side:6}{label:9}{len(L):4d} | {q(cb,.5):6.1f}/{q(cb,.9):6.1f} | {q(gap,.5):6.1f} | {q(early,.5):6.1f} | {q(vz,.5) if vz else float('nan'):6.0f} | {q(mm,.5) if mm else float('nan'):5.1f}")
print("\nRIGHT arm devjit, by day: no-carry closes -- how many are shallow because of INTENT (cmd bottom > -12 mm) vs LAG (cmd deep, gap > 6 mm)")
for (model,day,side),ev in sorted(groups.items()):
    if side!='right' or 'devjit' not in model: continue
    nc=[e for e in ev if not e['carry']]
    if not nc: continue
    intent=sum(1 for e in nc if (e['z_cmd_min']-REST_Z)>-0.012); lag=sum(1 for e in nc if (e['z_cmd_min']-REST_Z)<=-0.012 and (e['z_meas_min']-e['z_cmd_min'])>0.006); deep=sum(1 for e in nc if (e['z_meas_min']-REST_Z)<=-0.015)
    print(f"  {day} {model}: no-carry n={len(nc)} | shallow intent {intent} | deep intent but lag {lag} | actually deep (<= -15 mm) yet no carry {deep}")

print("\nRETRY STRUCTURE (right arm): a close is a RETRY if the previous close on that arm ended < 3 s earlier")
for (model,side),ev in sorted(pool.items()):
    if side!='right': continue
    byrun={}
    for e in ev: byrun.setdefault(e['run'],[]).append(e)
    first=[]; retry=[]
    for run,L in byrun.items():
        L=sorted(L,key=lambda e:e['t']); prev_end=None
        for e in L:
            is_retry = prev_end is not None and (e['t']-prev_end)<3.0
            (retry if is_retry else first).append(e)
            prev_end=e['t']+0.0  # approximate segment end by close time (hold length not stored)
    for label,L in (('first attempt',first),('retry',retry)):
        if not L: continue
        c=[e for e in L if e['carry']]; nc=[e for e in L if not e['carry']]
        f=lambda X,k:(q([x[k] for x in X],.5)-REST_Z)*1000 if X else float('nan')
        print(f"  {model:30} {label:14} n={len(L):4d} carry={len(c):3d} ({100*len(c)/len(L):3.0f}%) | cmd bottom-REST p50: carry {f(c,'z_cmd_min'):6.1f} / no-carry {f(nc,'z_cmd_min'):6.1f} | meas bottom: carry {f(c,'z_meas_min'):6.1f} / no-carry {f(nc,'z_meas_min'):6.1f}")
print("\nPER-DAY carry rate (closes -> carries), both arms:")
for (model,day,side),ev in sorted(groups.items()):
    c=sum(e['carry'] for e in ev); print(f"  {day} {model:30} {side:6} {len(ev):4d} closes -> {c:3d} carries ({100*c/len(ev):3.0f}%) | meas bottom p50 {(q([e['z_meas_min'] for e in ev],.5)-REST_Z)*1000:6.1f} mm")
