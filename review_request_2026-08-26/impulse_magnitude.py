import csv, sys
import numpy as np
path=sys.argv[1]; arm=sys.argv[2] if len(sys.argv)>2 else 'left'
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
run=txt('motion_state')=='Running'; ph=txt(f'{arm}_qsync_phase'); seq=num(f'{arm}_rback_seq')
QR=np.stack([num(f'{arm}_q_ref_{j}') for j in range(6)],1)
track=run&(ph=='track'); i=np.where(track)[0]
noadv=[k for k in i[1:] if seq[k]==seq[k-1]]
dQR=np.vstack([np.zeros(6), np.diff(QR,axis=0)])
recs=[]
for k in noadv:
    a=k+8
    if a<4 or a>=N-4: continue
    j=int(np.argmax(np.abs(dQR[a])))
    nb=np.median(np.abs(np.concatenate([dQR[a-3:a,j],dQR[a+1:a+4,j]])))
    if nb<=1e-4: continue
    if np.abs(dQR[a,j])/nb>1.6:
        extra=np.abs(dQR[a,j])-nb
        recs.append((t[a], j, nb*500, np.abs(dQR[a,j])*500, extra, extra*500/0.002))
recs=np.array([(r[0],r[1],r[2],r[3],r[4],r[5]) for r in recs])
print(f"{arm}: confirmed doubled-step events at lag8 = {len(recs)} over {t[i[-1]]-t[i[0]]:.1f}s "
      f"({len(recs)/(t[i[-1]]-t[i[0]]):.2f}/s, one every {(t[i[-1]]-t[i[0]])/max(len(recs),1):.2f}s)")
print(f"  normal 1-tick velocity  : med={np.median(recs[:,2]):7.2f} deg/s  p90={np.percentile(recs[:,2],90):7.2f}")
print(f"  spike  1-tick velocity  : med={np.median(recs[:,3]):7.2f} deg/s  p90={np.percentile(recs[:,3],90):7.2f}  max={recs[:,3].max():7.2f}")
print(f"  implied 1-tick accel    : med={np.median(recs[:,5]):9.0f} deg/s^2  p90={np.percentile(recs[:,5],90):9.0f}  max={recs[:,5].max():9.0f}")
print(f"  (config safety ddq_max is 1500-3000 deg/s^2)")
