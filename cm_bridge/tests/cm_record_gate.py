#!/usr/bin/env python3
"""cm_record_gate — the RECORDER gate: controller-manager `func write` capture (schema 4) +
the bridge sidecar carry a synchronized picture of one follow episode.

Assumes (see cm_bridge/tests/record_gate.sh, which sets all of this up):
  * a controller in SILS with both arms OnTask(Idle) and `func write start` already issued,
  * a cm_bridge_node with --chunk-bind CHUNK, --command-bind CMD, --state-endpoints STATE,
    a --sidecar path, and a gripper endpoint that is NOT the real gripper_server.

`stream` mode: pushes synthetic chunk frames (chunk_overlay.v3, both arms moving in +y,
24 rows, replan every 4; two speed phases, the second above the 50 mm/s envelope) to CHUNK, plus gripper_target commands to CMD every
0.5 s (a square wave 20 <-> 80 %) so the ext_scalars path is exercised.

`check` mode: given the two data_*.bin files and the sidecar, verifies
  [S4]   header says CHIMPBIN 4 and the schema-4 columns exist and are populated
  [MONO] mono_ns is monotone, 2 ms cadence, and overlaps the sidecar's time span
  [FOL]  fol_engaged rose during the stream and fol_chunk_stamp_ns takes values that are
         EXACTLY the sidecar's pub_mono_ns of published chunks (the join key holds)
  [EXT]  ext0 (grip_cmd_pct) tracks the gripper commands the streamer sent, and ext_stamp_ns
         is within one gripper period of the sidecar grip_cmd record it came from
Prints PASS/FAIL per check; exit 1 on any FAIL.
"""
import argparse
import json
import os
import socket
import sys
import time

DT = 1.0 / 29.94
ROWS = 24         # like the runner: execute 4 + runway 20 (all remaining rows of a 24-step chunk)
REPLAN = 4
VY = 0.030


def _rot_inv(q, v):
    """R(q)^T v for q = (x, y, z, w)."""
    x, y, z, w = q
    # q^-1 * (v,0) * q
    def mul(a, b):
        ax, ay, az, aw = a; bx, by, bz, bw = b
        return (aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx,
                aw*bz + ax*by - ay*bx + az*bw, aw*bw - ax*bx - ay*by - az*bz)
    r = mul(mul((-x, -y, -z, w), (v[0], v[1], v[2], 0.0)), (x, y, z, w))
    return (r[0], r[1], r[2])


def _state_pose(state_port, timeout=6.0):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", state_port))
    rx.settimeout(0.5)
    deadline = time.monotonic() + timeout
    out = {}
    while time.monotonic() < deadline and len(out) < 2:
        try:
            data, _ = rx.recvfrom(65535)
            st = json.loads(data)
        except (OSError, json.JSONDecodeError):
            continue
        for side in ("left", "right"):
            a = st.get(side, {})
            cmd = a.get("tcp_command_stand") or a.get("tcp_stand")
            act = a.get("tcp_stand")
            if side not in out and cmd and act:
                out[side] = [cmd["x"], cmd["y"], cmd["z"]] + act["quaternion_xyzw"]
    rx.close()
    return out


def stream(args):
    start = _state_pose(args.state_port)
    if len(start) < 2:
        print("STREAM FAIL: no servo_state.v1 for both arms on", args.state_port)
        return 1
    # optional: a synthetic flow-infer observation dump beside the stream, one record per chunk,
    # so the replay's policy-input panel and the inference_seq join get exercised without a policy
    dumper = None
    if args.obs_dump:
        import sys as _sys, os as _os
        _sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "..", "policy_runner"))
        from policy_runner.observation_dump import ObservationDumper
        dumper = ObservationDumper(args.obs_dump)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    chunk_addr = ("127.0.0.1", args.chunk_port)
    cmd_addr = ("127.0.0.1", args.cmd_port)
    seq = 0
    y = {s: start[s][1] for s in start}
    t0 = time.monotonic()
    replans = int(args.seconds / (REPLAN * DT))
    vys = [float(v) for v in args.vy.split(",")]      # speed phases, equal duration each
    grip_seq = 0
    next_grip = t0
    grip_val = 20.0
    plan_steps = []   # (chunk seq -> planned per-step speed) for the checker
    for i in range(replans):
        vy = vys[min(len(vys) - 1, (i * len(vys)) // max(1, replans))]
        req_ns = time.monotonic_ns()
        # the runner's contract, emulated: row 0 = its activation step (REPLAN per chunk), ALL
        # remaining rows, per-step local deltas INCLUDING delta 0 (left_delta/right_delta), and
        # a per-row gripper target that changes on the square wave
        pkt = {"schema_version": "robotics_lab.chunk_overlay.v3", "seq": seq,
               "policy_dt_sec": DT, "host_time_ns": time.time_ns(),
               "execute_steps": REPLAN, "runway_steps": ROWS - REPLAN,
               "chunk_metadata": {"inference_seq": seq + 1, "observation_step_seq": seq * REPLAN,
                                  "activation_step_seq": seq * REPLAN, "source_start_index": 0}}
        raw = []
        for side in start:
            rows, dl = [], []
            for k in range(ROWS):
                r = list(start[side])
                r[1] = y[side] + vy * DT * k
                r.append(grip_val)   # per-row grip target column
                rows.append(r)
                # ee_local delta for this step: the row frame is the start orientation (identity
                # local basis == world here since the synthetic pose does not rotate)
                dq = start[side][3:7]
                dloc = _rot_inv(dq, (0.0, vy * DT, 0.0))
                dl.append([dloc[0], dloc[1], dloc[2], 0.0, 0.0, 0.0, grip_val])
            pkt[side] = rows
            pkt[f"{side}_delta"] = dl
            pkt[f"{side}_grip_cmd"] = [grip_val] * ROWS
            y[side] += vy * DT * REPLAN
        if dumper is not None:
            import numpy as _np
            img = _np.full((480, 640, 3), 96 + (seq % 8) * 16, dtype=_np.uint8)
            state = _np.zeros(14, dtype=_np.float32); state[1] = vy * DT; state[8] = vy * DT
            state[6] = grip_val / 100.0; state[13] = grip_val / 100.0
            # a "model chunk" consistent with the rows: constant +y local delta per step
            acts = _np.zeros((ROWS, 14), dtype=_np.float32); acts[:, 1] = vy * DT; acts[:, 8] = vy * DT
            acts[:, 6] = grip_val / 100.0; acts[:, 13] = grip_val / 100.0
            dumper.record(inference_seq=seq + 1,
                          obs={"observation/left_wrist_0_rgb": img, "observation/right_wrist_0_rgb": img,
                               "observation/state": state, "prompt": "record gate synthetic"},
                          result={"actions": acts}, request_mono_ns=req_ns, ready_mono_ns=time.monotonic_ns(),
                          extra={"proprio_mode": "velocity_grip", "synthetic": True})
        tx.sendto(json.dumps(pkt).encode(), chunk_addr)
        plan_steps.append({"seq": seq, "vy": vy})
        seq += 1
        # gripper command square wave through the legacy command JSON
        now = time.monotonic()
        if now >= next_grip:
            grip_val = 80.0 if grip_val < 50 else 20.0
            grip_seq += 1
            cmd = {"seq": grip_seq, "mode": "Hold",
                   "left": {"gripper_target": grip_val}, "right": {"gripper_target": grip_val}}
            tx.sendto(json.dumps(cmd).encode(), cmd_addr)
            next_grip = now + 0.5
        # pace the replan cadence; --late-every K delays every K-th chunk by 1.5 periods (a slow
        # inference) so the pacer must continue the OLDER candidate (skip 4) and then re-slice
        target = t0 + (i + 1) * REPLAN * DT
        if args.late_every and (i + 2) % args.late_every == 0:
            target += 1.5 * DT
        while time.monotonic() < target:
            time.sleep(0.001)
    if dumper is not None:
        dumper.close()
    if args.plan_out:
        with open(args.plan_out, "w") as f:
            json.dump({"dt": DT, "rows": ROWS, "replan": REPLAN, "chunks": plan_steps}, f)
    print(f"streamed {seq} chunks over {time.monotonic() - t0:.1f}s, {grip_seq} gripper commands, "
          f"speed phases {vys} m/s"
          + (f", obs dump -> {args.obs_dump}" if dumper is not None else ""))
    return 0


# ---------------------------------------------------------------------------------------------
def read_bin(path):
    import numpy as np
    with open(path, "rb") as f:
        header = f.readline().decode("utf-8", "replace").rstrip("\n")
        parts = header.split("\t")
        if len(parts) < 4 or parts[0] != "CHIMPBIN":
            raise ValueError(f"not a CHIMPBIN file: {path}")
        version = int(parts[1]); row_bytes = int(parts[2])
        dtype = np.dtype([tuple(x) for x in json.loads(parts[3])])
        if dtype.itemsize != row_bytes:
            raise ValueError(f"row_bytes {row_bytes} != dtype itemsize {dtype.itemsize}")
        data = np.fromfile(f, dtype=dtype)
    return version, data


def _all_records(path):
    out = []
    with open(path) as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def check(args):
    import numpy as np
    ok = True

    def verdict(tag, cond, msg):
        nonlocal ok
        print(f"  {'PASS' if cond else 'FAIL'}  {tag:5s} {msg}")
        ok = ok and cond

    side = {}
    for path in args.bin:
        version, d = read_bin(path)
        s = "left" if len(d) and int(d["side"][0]) == 0 else "right"
        side[s] = (version, d, path)
    if not side:
        print("  FAIL  no .bin files"); return 1

    # sidecar
    chunks, grips = [], []
    with open(args.sidecar) as f:
        for line in f:
            r = json.loads(line)
            if r.get("type") == "chunk":
                chunks.append(r)
            elif r.get("type") == "grip_cmd":
                grips.append(r)
    print(f"sidecar: {len(chunks)} chunks, {len(grips)} grip_cmd records")

    for s, (version, d, path) in side.items():
        print(f"[{s}] {path}: {len(d)} rows, schema v{version}")
        cols = set(d.dtype.names)
        need = {"mono_ns", "fol_engaged", "fol_cmd_x", "fol_ref_x", "fol_chunk_stamp_ns",
                "fol_chunk_idx", "fol_chunk_n", "fol_gate_t", "dev_tx", "ext0", "ext1",
                "ext2", "ext_stamp_ns", "ext_seq", "follow_slot_seq"}
        verdict("S4", version >= 4 and need <= cols,
                f"version={version} schema-4 columns present={need <= cols}")
        if not (need <= cols):
            continue
        mono = d["mono_ns"].astype(np.int64)
        dm = np.diff(mono) / 1e6
        verdict("MONO", len(d) > 100 and np.all(dm > 0) and 1.0 < np.median(dm) < 3.0,
                f"rows={len(d)} median dt={np.median(dm):.3f} ms max={dm.max() if len(dm) else 0:.2f} ms")
        eng = d["fol_engaged"].astype(int)
        n_eng = int(eng.sum())
        verdict("FOL", n_eng > 200, f"fol_engaged ticks={n_eng} ({n_eng*0.002:.2f}s)")
        if n_eng:
            stamps = set(int(v) for v in d["fol_chunk_stamp_ns"][eng == 1] if int(v) != 0)
            pub = set(int(c["pub_mono_ns"].get(s, 0)) for c in chunks if c.get("pub_mono_ns", {}).get(s))
            # commit mode: the publish stamps live in follow_pub records (a chunk may be sliced /
            # published later than received)
            pub |= set(int(r["pub_mono_ns"]) for r in _all_records(args.sidecar)
                       if r.get("type") == "follow_pub" and r.get("side") == s)
            hit = len(stamps & pub)
            verdict("FOL", len(stamps) > 0 and hit == len(stamps),
                    f"adopted chunk stamps={len(stamps)} joined to sidecar pub_mono_ns={hit}")
            e = d[eng == 1]
            lag = np.sqrt((e["fol_cmd_x"] - e["fol_ref_x"])**2 + (e["fol_cmd_y"] - e["fol_ref_y"])**2
                          + (e["fol_cmd_z"] - e["fol_ref_z"])**2)
            print(f"        fol lag_mm median={np.median(e['fol_lag_mm']):.2f} (recomputed {np.median(lag):.2f}) "
                  f"gap_mm median={np.median(e['fol_gap_mm']):.2f} gate_t min={e['fol_gate_t'].min():.2f} "
                  f"chunk_n={int(e['fol_chunk_n'].max())} idx max={int(e['fol_chunk_idx'].max())} "
                  f"playing frac={e['fol_playing'].mean():.2f}")
            # the cmd chain must have moved along +y by ~VY * engaged time (mm)
            moved = float(e["fol_cmd_y"][-1] - e["fol_cmd_y"][0])
            print(f"        fol_cmd_y moved {moved:+.1f} mm over engaged span (expect ~{VY*1e3*n_eng*0.002:.0f} mm, "
                  f"cap-limited)")
        # ext: gripper command level
        ext_seq = d["ext_seq"].astype(int)
        verdict("EXT", ext_seq.max() >= max(1, len(grips) // 2),
                f"ext deposits seen by the recorder={ext_seq.max()} (sidecar grip_cmd={len(grips)})")
        if ext_seq.max() > 0 and grips:
            vals = sorted(set(round(float(v), 1) for v in d["ext0"][ext_seq > 0]))
            verdict("EXT", set(vals) <= {0.0, 20.0, 80.0} and (20.0 in vals or 80.0 in vals),
                    f"ext0 (grip_cmd_pct) levels={vals}")
            # each grip_cmd record's mono_ns should appear as ext2 (grip_cmd_mono_ns) within 1 ms
            g_mono = np.array([g["mono_ns"] for g in grips], dtype=np.float64)
            e2 = np.unique(d["ext2"][ext_seq > 0])
            e2 = e2[e2 > 0]
            near = sum(1 for gm in g_mono if len(e2) and np.min(np.abs(e2 - gm)) < 1e6)
            verdict("EXT", near >= max(1, len(g_mono) // 2),
                    f"grip_cmd_mono_ns joins: {near}/{len(g_mono)} sidecar records within 1 ms "
                    f"(the recorder samples the level; commands faster than the tick share a row)")
    # ---- the commit / stretch structure (2026-08-19) ----
    pubs = [r for r in _all_records(args.sidecar) if r.get("type") == "follow_pub"]
    steps = [r for r in _all_records(args.sidecar) if r.get("type") == "follow_step"]
    gcmds = [r for r in _all_records(args.sidecar) if r.get("type") == "grip_cmd"]
    ignored = [r for r in _all_records(args.sidecar) if r.get("type") == "grip_cmd_runner_ignored"]
    plan = json.load(open(args.plan)) if args.plan and os.path.exists(args.plan) else None
    print(f"sidecar: follow_pub={len(pubs)} follow_step={len(steps)} grip_cmd={len(gcmds)} runner-grip-ignored={len(ignored)}")
    for s, (version, d, path) in side.items():
        if not {"fol_chunk_stamp_ns", "fol_chunk_idx", "fol_engaged", "fol_cmd_y"} <= set(d.dtype.names):
            continue
        ev = [e for e in steps if e["side"] == s]
        windows, cur = [], None
        for e in ev:
            if e["kind"] == 2:
                if cur is not None: windows.append(len(cur))
                cur = set()
            if e["kind"] in (1, 2) and cur is not None and e.get("orig") is not None:
                cur.add(e["orig"])
            if e["kind"] in (0, 3) and cur is not None:
                windows.append(len(cur)); cur = None
        inner = windows[1:-1] if len(windows) > 2 else windows
        verdict("CMMT", len(inner) >= 3 and all(w == 4 for w in inner),
                f"[{s}] policy steps per adopted message = {inner[:12]}{'...' if len(inner) > 12 else ''} (expect all 4)")
        by_step = {}
        for e in ev:
            if e["kind"] in (1, 2) and e.get("orig") is not None:
                by_step[(e["stamp"], e["orig"])] = by_step.get((e["stamp"], e["orig"]), 0) + 1
        counts = sorted(by_step.values())
        sp = [r for r in pubs if r["side"] == s]
        mx = max((r.get("max_stretch") or 1) for r in sp) if sp else 1
        verdict("STRCH", mx >= 2 and max(counts, default=1) >= 2,
                f"[{s}] max stretch published={mx} sub-deltas; controller sub-deltas per policy step: "
                f"{dict((k, counts.count(k)) for k in sorted(set(counts)))} (fast phase must show 3)")
        gs = [r for r in gcmds if s in (r.get("pct") or {})]
        ends = [e for e in ev if e.get("prev_aux0") is not None]
        verdict("GRIP", len(gs) >= 3 and abs(len(gs) - len(ends)) <= 2 and len(ignored) >= 1,
                f"[{s}] gripper commands {len(gs)} vs step-end events {len(ends)} (one per policy step arrival); "
                f"runner's own dispatch ignored: {len(ignored)}")
        eng = d["fol_engaged"].astype(int) == 1
        moved = float(d["fol_cmd_y"][eng][-1] - d["fol_cmd_y"][eng][0]) if eng.any() else 0.0
        if plan is not None:
            vy_of_seq = {c["seq"]: c["vy"] for c in plan["chunks"]}
            pub_by_stamp = {int(r["pub_mono_ns"]): r for r in sp}
            planned = 0.0; nsteps = 0
            for (stamp, orig) in by_step:
                pr = pub_by_stamp.get(int(stamp))
                if pr is None: continue
                planned += vy_of_seq.get(pr["seq"], 0.0) * plan["dt"] * 1e3; nsteps += 1
            tol = 0.12 * plan["dt"] * 1e3 * 1.5
            verdict("NOCUT", nsteps > 10 and abs(moved - planned) <= tol,
                    f"[{s}] cmd chain moved {moved:.1f} mm vs planned {planned:.1f} mm over {nsteps} started steps "
                    f"(tol {tol:.1f} mm); a cut would lose ~{(1-50/120)*100:.0f}% of the fast phase")
    if args.obs_dump and os.path.exists(os.path.join(args.obs_dump, "manifest.jsonl")):
        seqs = set()
        with open(os.path.join(args.obs_dump, "manifest.jsonl")) as f:
            for line in f:
                r = json.loads(line)
                if r.get("type") == "inference":
                    seqs.add(int(r["seq"]))
        keyed = [c for c in chunks if (c.get("chunk_metadata") or {}).get("inference_seq") is not None]
        hit = sum(1 for c in keyed if int(c["chunk_metadata"]["inference_seq"]) in seqs)
        verdict("OBS", len(keyed) == len(chunks) and hit == len(keyed),
                f"sidecar chunks carrying inference_seq={len(keyed)}/{len(chunks)}, joined to obs-dump records={hit}")
    print("RECORD GATE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)
    s = sub.add_parser("stream")
    s.add_argument("--chunk-port", type=int, default=50264)
    s.add_argument("--cmd-port", type=int, default=50256)
    s.add_argument("--state-port", type=int, default=50378)
    s.add_argument("--seconds", type=float, default=4.0)
    s.add_argument("--obs-dump", default="", help="write a synthetic observation dump here (one record per chunk)")
    s.add_argument("--late-every", type=int, default=0, help="delay every K-th chunk by 1.5 periods (late-inference drill)")
    s.add_argument("--vy", default="0.03,0.12", help="comma list of +y speeds [m/s], one phase each (0.12 > the 50 mm/s envelope -> stretch x3)")
    s.add_argument("--plan-out", default="", help="write the streamed plan (chunk seq -> vy) here for the checker")
    c = sub.add_parser("check")
    c.add_argument("--bin", nargs="+", required=True)
    c.add_argument("--sidecar", required=True)
    c.add_argument("--obs-dump", default="", help="observation dump dir to verify the inference_seq join")
    c.add_argument("--plan", default="", help="the streamer's --plan-out file (for the NOCUT check)")
    args = ap.parse_args()
    return stream(args) if args.mode == "stream" else check(args)


if __name__ == "__main__":
    sys.exit(main())
