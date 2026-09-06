#!/usr/bin/env python3
"""Compare offline delta replay variants with the recorded follower output.

Reports post-SMD reference smoothness, not counterfactual motor/encoder motion.
The robot's physical response and contact forces are not simulated here.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import butter,sosfiltfilt


def quant(x):
    x=np.asarray(x);x=x[np.isfinite(x)]
    return dict(zip(("p50","p90","p99","max"),np.quantile(x,[.5,.9,.99,1]).tolist())) if len(x) else {}


def analyze(root: Path, windows: list[tuple[str,float,float]], plots: bool = True):
    obs=pd.read_csv(root/"observed.csv")
    result={"schema":"robotics_lab.delta_replay_comparison.v1","fidelity":{},"windows":{}}
    records=[];cache={};sos=butter(3,[3,15],btype="bandpass",fs=500,output="sos")
    for arm in sorted({w[0] for w in windows}):
        baseline=pd.read_csv(root/f"{arm}_baseline.csv")
        actual=obs.loc[(obs[arm+"_mode"]=="TcpPoseTarget") & (obs[arm+"_follower_active"]==1)].reset_index(drop=True)
        if len(baseline)!=len(actual) or np.max(abs(baseline.t-actual.t))>1e-6:
            raise ValueError("replay/recorded time grids differ")
        bpos=baseline[["smd_x","smd_y","smd_z"]].to_numpy()
        raw_obs=actual[[arm+"_follower_prefilter_stand_"+x+"_m" for x in "xyz"]].to_numpy()
        raw_sim=baseline[["raw_x","raw_y","raw_z"]].to_numpy()
        result["fidelity"][arm]={"duration_error_ms":quant(abs(baseline.duration-actual[arm+"_follower_duration_sec"])*1000),
                                "raw_position_error_mm":quant(np.linalg.norm(raw_obs-raw_sim,axis=1)*1000)}
        for path in sorted(root.glob(f"{arm}_*.csv")):
            variant=path.stem[len(arm)+1:];df=pd.read_csv(path)
            if len(df)!=len(baseline) or not np.allclose(df.t,baseline.t,rtol=0,atol=1e-7):
                raise ValueError(f"variant time grid mismatch: {path}")
            pos=df[["smd_x","smd_y","smd_z"]].to_numpy();band=sosfiltfilt(sos,pos,axis=0)*1000
            # Finite differences on the recorded time grid. These are Cartesian
            # reference derivatives, not sent-joint or measured acceleration.
            velocity=np.gradient(pos,df.t.to_numpy(),axis=0)
            acceleration=np.gradient(velocity,df.t.to_numpy(),axis=0)
            target_v=df[[f"vf_{i}" for i in range(3)]].to_numpy()
            target_a=df[[f"af_{i}" for i in range(3)]].to_numpy()
            target_dv=np.linalg.norm(np.diff(target_v,axis=0,prepend=target_v[:1]),axis=1)
            target_da=np.linalg.norm(np.diff(target_a,axis=0,prepend=target_a[:1]),axis=1)
            # The wire sequence here is the segment being consumed, not merely
            # the most recently received frame.
            seam=df.wire_seq.ne(df.wire_seq.shift())
            segment=df.segment.ne(df.segment.shift())
            cache[(arm,variant)]=(df,band)
            for a,lo,hi in windows:
                if a!=arm:continue
                mask=(df.t>=lo)&(df.t<hi);seg=df.loc[mask&segment]
                if seg.empty:raise ValueError(f"no segments in {a}:{lo}:{hi}")
                axis=seg[[f"axis_duration_{i}" for i in range(6)]].to_numpy()
                over=axis.max(axis=1)>seg.policy_dt.to_numpy()*1.001
                durations=seg.duration/seg.policy_dt
                # Re-align the initial translation to show LOCAL path changes
                # separately from drift accumulated before the review window.
                path_delta=(pos[mask]-pos[mask][0])-(bpos[mask]-bpos[mask][0])
                row={"arm":arm,"start":lo,"end":hi,"variant":variant,"segments":len(seg),
                     "converged_pct":100*float(seg.converged.mean()),"duration_ratio_p50":float(durations.median()),
                     "duration_ratio_p90":float(durations.quantile(.9)),"duration_ratio_max":float(durations.max()),
                     "smd_3_15hz_rms_mm":float(np.sqrt(np.mean(np.sum(band[mask]**2,axis=1)))),
                     "smd_speed_p99_mm_s":float(np.quantile(np.linalg.norm(velocity[mask],axis=1)*1000,.99)),
                     "smd_accel_p99_m_s2":float(np.quantile(np.linalg.norm(acceleration[mask],axis=1),.99)),
                     "seam_target_dv_max_mm_s":float(np.max(target_dv[mask&seam],initial=0)*1000),
                     "seam_target_da_max_m_s2":float(np.max(target_da[mask&seam],initial=0)),
                     "local_path_delta_p95_mm":float(np.quantile(np.linalg.norm(path_delta,axis=1)*1000,.95)),
                     "local_end_delta_mm":float(np.linalg.norm(path_delta[-1])*1000),
                     "projection_p95_mm":float(seg.projection_m.quantile(.95)*1000),
                     "stall_ticks":int(df.loc[mask,"stall"].sum())}
                records.append(row)
                key=f"{arm}:{lo:g}:{hi:g}"
                result["windows"].setdefault(key,{})[variant]={**row,"longest_axis_when_over_budget":
                    {str(k):int(v) for k,v in pd.Series(np.argmax(axis,axis=1)[over]).value_counts().items()}}
                if variant=="baseline":
                    result["windows"][key][variant]["raw_position_error_mm"]=quant(np.linalg.norm((raw_obs-raw_sim)[mask],axis=1)*1000)
                    result["windows"][key][variant]["duration_error_ms"]=quant(abs((baseline.duration-actual[arm+"_follower_duration_sec"])[mask])*1000)
    table=pd.DataFrame(records);table.to_csv(root/"comparison.csv",index=False)
    (root/"comparison.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    if plots:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        selection=["baseline","no_force_gate","linear_relaxed","angular_relaxed","overlap_linear4","overlap_quintic4"]
        fig,axs=plt.subplots(2,len(windows),figsize=(5.4*len(windows),7),squeeze=False)
        for col,(a,lo,hi) in enumerate(windows):
            for name in selection:
                if (a,name) not in cache:continue
                df,band=cache[(a,name)];m=(df.t>=lo)&(df.t<hi)
                axs[0,col].plot(df.t[m],band[m,2],label=name,lw=.8)
            rows=table[(table.arm==a)&(table.start==lo)&table.variant.isin(selection)]
            axs[1,col].barh(rows.variant,rows.smd_3_15hz_rms_mm,color="#4c78a8")
            axs[0,col].set_title(f"{a}: +{lo:g} to +{hi:g} s")
            axs[0,col].set_ylabel("Post-SMD TCP z, 3-15 Hz [mm]");axs[0,col].set_xlabel("Seconds from first chunk")
            axs[1,col].set_xlabel("Post-SMD vector RMS, 3-15 Hz [mm]")
            axs[0,col].grid(alpha=.2)
        axs[0,0].legend(fontsize=7,ncol=2)
        fig.suptitle("Offline delta_preview replay\nRecorded force fixed; reference trajectories only\nNo IK, motor dynamics or new contact simulation",fontsize=11)
        fig.tight_layout();fig.savefig(root/"comparison.png",dpi=145);plt.close(fig)
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("root",type=Path)
    p.add_argument("--window",action="append",required=True,help="arm:start:end, relative seconds")
    p.add_argument("--no-plots",action="store_true");args=p.parse_args()
    windows=[]
    for value in args.window:
        arm,start,end=value.split(":")
        if arm not in ("left","right") or float(end)<=float(start):raise ValueError("invalid window")
        windows.append((arm,float(start),float(end)))
    r=analyze(args.root,windows,not args.no_plots)
    print(json.dumps(r["fidelity"],indent=2))
    print(pd.read_csv(args.root/"comparison.csv").to_string(index=False))
