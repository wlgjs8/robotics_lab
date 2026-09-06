#!/usr/bin/env python3
"""Join a servo CSV and chunk-row JSONL into hardware-free replay events.

Timestamps are relative to the first published chunk. This exporter uses the
server's received-frame sequence, not Python publish time, to deliver frames.
Force inputs are recorded exogenous signals: counterfactuals do not simulate a
new physical contact. Default export covers the complete log intersection.
"""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import pandas as pd


def prepare(servo: Path, chunks_path: Path, out: Path, end: float | None = None) -> dict:
    chunks = [json.loads(line) for line in chunks_path.open() if line.strip()]
    if not chunks:
        raise ValueError("empty chunk log")
    origin = float(chunks[0]["t_mono"])
    by_seq = {int(c["seq"]): c for c in chunks}
    if len(by_seq) != len(chunks):
        raise ValueError("chunk seq must be unique within one producer run")
    with servo.open() as handle:
        header = next(csv.reader(handle))
    base = ["loop_start_time_ns", "filter_dt_ms", "chunk_frame_wire_seq", "chunk_frame_recv_seq"]
    cols = base.copy()
    for a in ("left", "right"):
        cols += [a + "_" + c for c in ["mode", "follower_active", "follower_duration_sec", "follower_converged",
                  "follower_step", "follower_wire_seq", "follower_t_in_seg_sec", "fc_covered", "fc_gate_translation", "plan_gate"]]
        for p, axes in [("tcp_command_stand", ["x_m","y_m","z_m","qx","qy","qz","qw"]),
                        ("fc_dev",["x_m","y_m","z_m","rx_rad","ry_rad","rz_rad"]),
                        ("ft_comp_stand",["fx_n","fy_n","fz_n"]),
                        ("follower_prefilter_stand",["x_m","y_m","z_m"]),
                        ("stage_tcp_target_stand",["x_m","y_m","z_m"]),
                        ("tcp_actual_stand",["x_m","y_m","z_m"])]:
            cols += [f"{a}_{p}_{x}" for x in axes]
    missing = set(cols)-set(header)
    if missing:
        raise ValueError(f"required servo fields absent: {sorted(missing)}")
    exact_gate={}
    for a in ("left","right"):
        names=[a+"_follower_advance_gate",a+"_follower_plan_rate_gate"]+[a+"_follower_advance_dir_"+x for x in "xyz"]
        exact_gate[a]=all(c in header for c in names)
        if exact_gate[a]:cols+=names
    df = pd.read_csv(servo, usecols=cols, low_memory=False)
    df["t"] = df.loop_start_time_ns*1e-9-origin
    df = df.loc[(df.t >= 0) & (df.t <= (end if end is not None else chunks[-1]["t_mono"]-origin+.15))].reset_index(drop=True)
    out.mkdir(parents=True,exist_ok=True)
    meta = {"schema":"robotics_lab.delta_replay_input.v1", "servo":str(servo.resolve()),
            "chunks":str(chunks_path.resolve()), "origin_mono":origin,"end_sec":end,
            "tick_dt_sec":"recorded filter_dt_ms",
            "gate_timing":"logged follower inputs when present; otherwise previous tick gate and compensated unfiltered stand wrench",
            "limits":"read from explicitly supplied stack config; no robot started", "arms":{}}
    for a in ("left","right"):
        previous_seq = None; records=0; frames=0
        active = ((df[a+"_mode"]=="TcpPoseTarget") & (df[a+"_follower_active"]==1)).to_numpy()
        rows=df.to_dict("records")
        with (out/f"{a}.events.jsonl").open("w") as handle:
            for i,row in enumerate(rows):
                event={"t":row["t"],"active":bool(active[i]),"dt":row["filter_dt_ms"]*.001}
                if active[i]:
                    prev=rows[max(0,i-1)]
                    ref=[prev[f"{a}_tcp_command_stand_{x}"] for x in ["x_m","y_m","z_m","qx","qy","qz","qw"]]
                    if prev[a+"_fc_covered"]:
                        rot=[prev[f"{a}_fc_dev_{x}_rad"] for x in ["rx","ry","rz"]]
                        if np.linalg.norm(rot)>1e-8:
                            raise ValueError("non-rigid force rotation requires full nominal SE(3) reconstruction")
                        ref[:3]=[ref[k]-prev[f"{a}_fc_dev_{x}_m"] for k,x in enumerate("xyz")]
                    event.update(reference=ref,gate=float(prev[a+"_fc_gate_translation"]) if prev[a+"_fc_covered"] else 1.,
                                 force=[float(prev[f"{a}_ft_comp_stand_f{x}_n"]) for x in "xyz"] if prev[a+"_fc_covered"] else [0.,0.,0.],
                                 plan_gate=float(prev[a+"_plan_gate"]))
                    if exact_gate[a]:
                        event.update(gate=float(row[a+"_follower_advance_gate"]),
                                     plan_gate=float(row[a+"_follower_plan_rate_gate"]),
                                     force=[float(row[a+"_follower_advance_dir_"+x]) for x in "xyz"])
                    seq=int(row["chunk_frame_wire_seq"])
                    if seq != previous_seq:
                        if seq not in by_seq or by_seq[seq].get(a+"_delta") is None:
                            raise ValueError(f"active {a} tick has no recorded delta frame {seq} at {row['t']}")
                        c=by_seq[seq]
                        event["frame"]={"seq":seq,"policy_dt":c["policy_dt_sec"],"delta":c[a+"_delta"]}
                        previous_seq=seq;frames+=1
                else:
                    previous_seq=None
                handle.write(json.dumps(event,allow_nan=False,separators=(",",":"))+"\n");records+=1
        meta["arms"][a]={"ticks":records,"active_ticks":int(active.sum()),"frames":frames,"exact_plan_gate_input":exact_gate[a]}
    df.to_csv(out/"observed.csv",index=False)
    (out/"manifest.json").write_text(json.dumps(meta,indent=2)+"\n")
    return meta


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("servo",type=Path);p.add_argument("chunks",type=Path);p.add_argument("out",type=Path)
    p.add_argument("--end-sec",type=float)
    args=p.parse_args();print(json.dumps(prepare(args.servo,args.chunks,args.out,args.end_sec),indent=2))
