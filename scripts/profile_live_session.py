#!/usr/bin/env python3
"""Record a live `make run` session (UMI teleop OR model rollout) into the SAME
per-tick log schema the replay profiler uses, so the existing A/B/C/D analyzer
(`analyze_pgprofile_run.py`) can score teleop / rollout exactly like replay.

It subscribes (read-only) to a server state-fanout endpoint and, per published
tick, emits one row per arm via the replay driver's `log_row`. It commands nothing
and holds no lease — purely passive. Run it alongside `make run MODE=real` (or sim):

  # the server already fanouts to udp://127.0.0.1:50356 (recorder/debug slot,
  # free during make run: 50366=viser, 50376=policy_runner). No config change.
  python3 scripts/profile_live_session.py --label teleop_run1 --analyze
  # ... drive the teleop / start flow-infer ... Ctrl-C to stop & analyze.

Field mapping (server state -> log columns):
  conditioned_goal_after_A <- cartesian_solve.smd_goal_stand  (SMD integrated goal;
      best server-side proxy for the commanded/conditioned goal — the raw policy/
      teleop source target is only in the COMMANDER's log, not the state stream)
  reference_after_B        <- tcp_ref_stand   (post-IK controller reference)
  smd_ref_stand            <- cartesian_solve.smd_ref_stand (pure SMD, pre-IK)
  actual_tcp               <- tcp_actual_stand (REAL physical pose -> D-tier
      physical tracking + tremor ARE measurable here, unlike pgmode q_actual-frozen)
  q_target/q_actual, q_target_before/after_output_ma_deg, smd clips, ik telemetry
      <- as published.

So this gives live **B (smd_goal->smd_ref), C (output MA), D (actual vs ref)**.
A-tier (raw source -> conditioned) needs the commander log (rollout actions_*.jsonl
/ teleop recv log) and is recorded there, not here.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (str(REPO_ROOT), str(REPO_ROOT / "tools")):
    if path not in sys.path:
        sys.path.insert(0, path)

import analyze_pgprofile_run as ana  # noqa: E402
from tcp_tuning.config import load_config  # noqa: E402
from tcp_tuning.trajectory_log import TrajectoryLogWriter  # noqa: E402
from scripts.replay_episode_tcp_pose_target import (  # noqa: E402
    log_row,
    _state_pose_or_nan,
)

ARMS = ("left", "right")
NAN7 = np.full(7, np.nan, dtype=np.float64)
NAN6 = np.full(6, np.nan, dtype=np.float64)


def _endpoint_port(endpoint: str) -> int:
    return int(endpoint.rsplit(":", 1)[-1])


def record(args: argparse.Namespace) -> int:
    port = _endpoint_port(args.bind)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(args.idle_timeout_sec)

    base_name = args.label or time.strftime("live_%Y%m%dT%H%M%S")
    # Never overwrite an existing run: auto-suffix _02, _03, ... so repeating the
    # SAME --label across rollouts ACCUMULATES separate runs (each independently
    # analyzed; aggregate later with --aggregate). Appending into one log would
    # corrupt cross-correlation lag / span / HF at the run seams.
    run_name = base_name
    n = 1
    while (Path(args.out_dir) / run_name).exists():
        n += 1
        run_name = f"{base_name}_{n:02d}"
    out_dir = Path(args.out_dir) / run_name
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "log.csv"

    rows: list[dict[str, Any]] = []
    t0 = None
    last_seq = None
    samples = 0
    dropped_stale = 0
    print(f"[profile] listening on udp://0.0.0.0:{port}  -> {log_path}\n"
          f"[profile] drive teleop / start rollout now; Ctrl-C to stop"
          + (f"  (auto-stop {args.duration_sec}s)" if args.duration_sec else ""), flush=True)
    start_wall = time.perf_counter()
    try:
        while True:
            if args.duration_sec and (time.perf_counter() - start_wall) >= args.duration_sec:
                break
            if args.max_samples and samples >= args.max_samples:
                break
            try:
                data, _ = sock.recvfrom(1 << 16)
            except socket.timeout:
                print("[profile] idle (no state packet within idle-timeout); waiting...", flush=True)
                continue
            try:
                state = json.loads(data)
            except json.JSONDecodeError:
                continue
            now = time.perf_counter()
            if t0 is None:
                t0 = now
            t = now - t0
            # de-dup identical re-published frames by seq when present
            seq = state.get("seq")
            if seq is not None and seq == last_seq:
                dropped_stale += 1
                continue
            last_seq = seq
            for arm in ARMS:
                arm_state = state.get(arm) if isinstance(state.get(arm), dict) else {}
                solve = arm_state.get("cartesian_solve") if isinstance(arm_state.get("cartesian_solve"), dict) else {}
                cond_goal = _state_pose_or_nan(solve, "smd_goal_stand")
                rows.append(log_row(
                    t=t, t_source=t, src_idx=-1, arm=arm,
                    source_raw_target=NAN7,        # raw source target lives in the commander log
                    conditioned_goal=cond_goal,    # = SMD integrated goal (server-side proxy)
                    conditioned_twist=NAN6,
                    snapshot=state,
                    t_episode=t, time_scale=1.0, time_scale_mode="live",
                ))
            samples += 1
            if samples % 500 == 0:
                print(f"[profile] {samples} ticks ({t:.1f}s)", flush=True)
    except KeyboardInterrupt:
        print("\n[profile] stopped by user", flush=True)
    finally:
        sock.close()

    if not rows:
        print("[profile] no state received — is the server up and fanning out to this port?", file=sys.stderr)
        return 2
    rate = samples / max(t, 1e-9)
    writer = TrajectoryLogWriter(log_path, metadata={
        "session_label": run_name,
        "source": "profile_live_session",
        "bind": args.bind,
        "samples": samples,
        "duration_sec": round(t, 3),
        "effective_state_rate_hz": round(rate, 1),
        "dropped_stale_repeats": dropped_stale,
        "note": "live B/C/D profile; conditioned_goal_after_A=smd_goal_stand proxy; "
                "A-tier raw source is in the commander log, not here",
    })
    writer.extend(rows)
    writer.write()
    print(f"[profile] wrote {log_path}  ({samples} ticks, ~{rate:.0f} Hz)", flush=True)

    if args.analyze:
        cfg = load_config(None).metrics
        result = ana.analyze_run(log_path, cfg, episode_id=run_name, time_scale=1.0)
        (out_dir / "pgprofile_result.json").write_text(
            json.dumps(result, indent=2, allow_nan=False, default=lambda o: None) + "\n")
        (out_dir / "pgprofile_summary.md").write_text(ana.render_summary(result))
        print("\n" + ana.render_summary(result))
        if not args.keep_log:
            # The per-tick log.csv is huge (~1MB/s @500Hz). After analysis the
            # result.json/summary.md hold everything we need; drop the raw to save disk.
            try:
                log_path.unlink()
                print(f"[profile] dropped raw log.csv (--keep-log to retain); result.json kept", flush=True)
            except OSError:
                pass
    return 0


def aggregate(args: argparse.Namespace) -> int:
    """Combine all analyzed runs under --out-dir into one comparison table."""
    import csv
    base = Path(args.out_dir)
    results = sorted(base.glob("*/pgprofile_result.json"))
    if not results:
        print(f"[aggregate] no pgprofile_result.json under {base}", file=sys.stderr)
        return 2
    rows = []
    for rf in results:
        r = json.loads(rf.read_text())
        label = rf.parent.name
        c = r["classification"]; td = c["aggregate"]["tracking_detail"]
        B = r["B_reference_generation"]; D = r["physical_goal_tracking_C"]; isf = r["ik_safety_feasibility"]
        def wmax(d, *path, scale=1.0):
            vals = []
            for a in ("left", "right"):
                cur = d.get(a, {})
                for k in path:
                    cur = cur.get(k, {}) if isinstance(cur, dict) else {}
                if isinstance(cur, (int, float)):
                    vals.append(cur * scale)
            return round(max(vals), 3) if vals else None
        rows.append(dict(
            run=label, class_=c["primary_class"],
            B_pos_p95_mm=round(max((td.get(a, {}).get("pos_p95_mm") or 0) for a in ("left", "right")), 2),
            D_actual_vs_ref_p95_mm=wmax(D, "actual_tcp_vs_reference_after_B", "position_m", "p95", scale=1000),
            D_actual_vs_ref_max_mm=wmax(D, "actual_tcp_vs_reference_after_B", "position_m", "max", scale=1000),
            D_status=D.get("left", {}).get("status"),
            smd_linclip=sum(B[a]["smd_clip"]["linear_velocity_clipped_count"] for a in ("left", "right")),
            branch=sum(isf[a].get("ik_branch_jump_count") or 0 for a in ("left", "right")),
            selfcol=sum(isf[a].get("self_collision_count") or 0 for a in ("left", "right")),
        ))
    cols = ["run", "class_", "B_pos_p95_mm", "D_actual_vs_ref_p95_mm", "D_actual_vs_ref_max_mm",
            "D_status", "smd_linclip", "branch", "selfcol"]
    out = base / "aggregate_table.csv"
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols); w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in cols})
    print(f"[aggregate] {len(rows)} runs -> {out}\n")
    print(" | ".join(cols))
    for r in rows:
        print(" | ".join(str(r.get(k, "")) for k in cols))
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bind", default="udp://127.0.0.1:50356",
                   help="server state-fanout endpoint to subscribe to (default 50356, the recorder slot)")
    p.add_argument("--out-dir", default="outputs/tcp_live_profile")
    p.add_argument("--label", default=None, help="run subdir name (e.g. teleop_run1 / rollout_h8)")
    p.add_argument("--duration-sec", type=float, default=0.0, help="auto-stop after N sec (0 = until Ctrl-C)")
    p.add_argument("--max-samples", type=int, default=0, help="auto-stop after N ticks (0 = unlimited)")
    p.add_argument("--idle-timeout-sec", type=float, default=2.0)
    p.add_argument("--analyze", action="store_true", default=True, help="run the A/B/C/D analyzer after recording")
    p.add_argument("--keep-log", action="store_true", default=True,
                   help="keep the raw per-tick log.csv (~1MB/s). Default: drop it after --analyze to save disk.")
    p.add_argument("--aggregate", action="store_true",
                   help="don't record; combine all analyzed runs under --out-dir into aggregate_table.csv")
    args = p.parse_args(argv)
    if args.aggregate:
        return aggregate(args)
    return record(args)


if __name__ == "__main__":
    raise SystemExit(main())
