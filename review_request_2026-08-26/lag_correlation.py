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
run=txt('motion_state')=='Running'; ph=txt(f'{arm}_qsync_phase')
seq=num(f'{arm}_rback_seq')
QR=np.stack([num(f'{arm}_q_ref_{j}') for j in range(6)],1)
QS=np.stack([num(f'{arm}_q_sent_{j}') for j in range(6)],1)
track=run&(ph=='track')
i=np.where(track)[0]
# a loop tick where rback_seq did not advance == the worker sent nothing == that setpoint was overwritten
noadv=[k for k in i[1:] if seq[k]==seq[k-1]]
print(f"{arm}: track ticks={i.size}  ticks with NO new send (dropped setpoint) = {len(noadv)}")
print(f"   at t = {[f'{t[k]:.2f}' for k in noadv][:25]}")
dQR=np.vstack([np.zeros(6), np.diff(QR,axis=0)])
def ratio_at(k, lag):
    # doubled-step score at tick k+lag: |dq| vs mean of the 4 surrounding steps
    a=k+lag
    if a<3 or a>=N-3: return np.nan
    j=int(np.argmax(np.abs(dQR[a])))
    nb=np.abs(np.concatenate([dQR[a-3:a,j],dQR[a+1:a+4,j]]))
    m=np.median(nb)
    return np.abs(dQR[a,j])/m if m>1e-4 else np.nan
print("\n  lag  mean doubled-step ratio at drop ticks   |  control (random track ticks)")
rng=np.random.default_rng(0)
ctrl=rng.choice(i[10:-10], size=min(400,i.size-20), replace=False)
for lag in range(0,15):
    a=np.array([ratio_at(k,lag) for k in noadv]); a=a[np.isfinite(a)]
    b=np.array([ratio_at(k,lag) for k in ctrl]); b=b[np.isfinite(b)]
    if a.size:
        hits=int((a>1.6).sum())
        print(f"  {lag:3d}   n={a.size:3d} mean={a.mean():6.3f} max={a.max():6.3f}  ratio>1.6: {hits}/{a.size}"
              f"   |  ctrl mean={b.mean():6.3f}  >1.6: {100*(b>1.6).mean():5.2f}%")
