import csv, sys
import numpy as np
path=sys.argv[1]
f=open(path,newline=''); r=csv.reader(f); hdr=next(r); idx={n:i for i,n in enumerate(hdr)}
rows=[row for row in r if len(row)>=len(hdr)]
N=len(rows)
def num(n):
    i=idx[n]; o=np.empty(N)
    for k,row in enumerate(rows):
        try:o[k]=float(row[i])
        except:o[k]=np.nan
    return o
def txt(n):
    i=idx[n]; return np.array([row[i] for row in rows])
t=(num('loop_start_time_ns')-num('loop_start_time_ns')[0])/1e9
run=txt('motion_state')=='Running'
print(f"=== {path}")
for arm in ('left','right'):
    ph=txt(f'{arm}_qsync_phase'); seq=num(f'{arm}_rback_seq'); trim=num(f'{arm}_qsync_trim_us')
    for phname in ('track','drain'):
        m=run&(ph==phname)
        if m.sum()<60: continue
        i=np.where(m)[0]
        # contiguous largest block
        br=np.where(np.diff(i)>1)[0]
        seg=np.split(i,br+1); seg=max(seg,key=len)
        dt=t[seg[-1]]-t[seg[0]]
        loop_ticks=len(seg)-1
        sends=seq[seg[-1]]-seq[seg[0]]
        tr=np.median(trim[seg])
        print(f"  {arm:5s} {phname:6s}: {dt:7.3f}s  loop_setpoints={loop_ticks:6d} ({loop_ticks/dt:8.3f} Hz)"
              f"  box_sends(rback_seq)={sends:6.0f} ({sends/dt:8.3f} Hz)")
        print(f"        trim_med={tr:7.2f}us -> predicted send rate {1e6/(2000+tr):8.3f} Hz"
              f"   |  DROPPED setpoints = {loop_ticks-sends:6.0f}  ({(loop_ticks-sends)/dt:7.2f}/s, "
              f"{100*(loop_ticks-sends)/max(loop_ticks,1):.2f}%)")
