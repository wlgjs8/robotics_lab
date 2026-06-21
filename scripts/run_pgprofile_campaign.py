#!/usr/bin/env python3
"""Drive the full TcpTargetPose pgmode profiling campaign, episode by episode.

For every episode (ordered) this:
  1. ensures the pgmode stack is healthy (state flowing, no latched fault);
  2. replays it once via ``batch_replay_episodes.py`` at action_scale / time_scale
     fixed by the campaign (default 1.0 / 1.0), with the driver client speed guard
     lifted so the *server* SMD/IK profile is what shapes/limits the motion
     (the server config — the profile under evaluation — is never modified);
  3. runs the 3-tier analyzer (``analyze_pgprofile_run.py``) on the run log;
  4. records one ``primary_class`` + risk flags per episode, merging the offline
     manifest's speed precheck / validity;
  5. on controller fault, recovers via stop -> real/sim mode toggle -> restart,
     then continues (the only way to clear an rbpodo controller RobotFault latch);
  6. writes incremental aggregates so a partial / interrupted run is still useful,
     and is resumable (episodes with a result are skipped).

This does not tune any SMD / servo_j / safety parameter. Failed episodes are
classified, not fixed.
"""
from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (str(REPO_ROOT), str(REPO_ROOT / "tools")):
    if path not in sys.path:
        sys.path.insert(0, path)

import analyze_pgprofile_run as ana  # noqa: E402
from tcp_tuning.config import load_config  # noqa: E402

STATE_PORT = 50356  # the free fanout endpoint the driver/replaybind use
INIT_LEFT = "-131.7,73,113.4,-80.9,-107.1,-145.9"
INIT_RIGHT = "135.1,-64,-114.5,84.4,112.5,129.9"


# ----------------------------------------------------------------- state ------
def read_state(timeout: float = 2.0) -> dict[str, Any] | None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("0.0.0.0", STATE_PORT))
    except OSError:
        s.close()
        return None
    s.settimeout(timeout)
    try:
        data, _ = s.recvfrom(65535)
        return json.loads(data)
    except (socket.timeout, json.JSONDecodeError, OSError):
        return None
    finally:
        s.close()


def is_faulted(state: dict[str, Any] | None) -> bool:
    if not state:
        return False
    if bool(state.get("fault_latched")):
        return True
    verdict = str(state.get("safety_verdict", "")).strip().lower()
    return bool(verdict) and verdict not in ("ok", "safe")


# --------------------------------------------------------- stack lifecycle ----
def _run(cmd: list[str], timeout: float = 120.0, env: dict | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, cwd=str(REPO_ROOT), capture_output=True, text=True,
                          timeout=timeout, env=env, check=False)


def _server_running() -> bool:
    r = _run(["pgrep", "-f", "rb_servo_server --config"], timeout=10)
    return bool(r.stdout.strip())


def stop_stack() -> None:
    for pat in ("run_stack.sh", "rb_servo_server --config", "rb_servo_gui.app",
                "gripper_server", "policy_runner"):
        _run(["pkill", "-TERM", "-f", pat], timeout=20)
    time.sleep(3)
    for pat in ("rb_servo_server --config", "rb_servo_gui.app"):
        _run(["pkill", "-KILL", "-f", pat], timeout=20)
    time.sleep(1)


def start_stack(boot_timeout: float = 60.0) -> bool:
    log = REPO_ROOT / "logs/stack/make_run_campaign.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    import os
    env = dict(os.environ, ACTION_SOURCE="none")
    fh = open(log, "ab")
    subprocess.Popen(["make", "run", "MODE=sim"], cwd=str(REPO_ROOT),
                     stdout=fh, stderr=subprocess.STDOUT, env=env,
                     start_new_session=True)
    server_log = REPO_ROOT / "logs/stack/server.log"
    deadline = time.time() + boot_timeout
    while time.time() < deadline:
        try:
            if server_log.exists() and "CommandServer listening" in server_log.read_text(errors="ignore")[-4000:]:
                time.sleep(2)
                return True
        except OSError:
            pass
        time.sleep(1)
    return False


def toggle_mode() -> bool:
    _run(["bash", "tools/real_mode.sh"], timeout=60)
    time.sleep(3)
    res = _run(["bash", "tools/simulation_mode.sh"], timeout=60)
    return "real_vs_simulation_mode\": 1" in (res.stdout + res.stderr) or res.returncode == 0


def recover(log) -> bool:
    log("  RECOVER: stop stack -> real/sim toggle -> restart")
    stop_stack()
    toggle_mode()
    ok = start_stack()
    log(f"  RECOVER: restart {'ok' if ok else 'FAILED'}")
    time.sleep(2)
    return ok


# --------------------------------------------------------------- episode ------
def run_episode(stem: str, args, log) -> dict[str, Any]:
    # Leading-dash joint values must use the --opt=value form or argparse treats
    # them as flags.
    cmd = [
        sys.executable, "scripts/batch_replay_episodes.py",
        "--episodes-dir", args.episodes_dir, "--episodes", stem,
        "--init-mode", "joints",
        f"--init-left-joints={INIT_LEFT}", f"--init-right-joints={INIT_RIGHT}",
        "--source", "ee_local", "--mode", "clean_foh_se3", "--segment", "auto-largest",
        f"--action-scale={args.action_scale}", f"--time-scale={args.time_scale}",
        "--skip-failed-episodes",
        f"--max-linear-speed-m-s={args.client_max_linear}",
        f"--max-angular-speed-rad-s={args.client_max_angular}",
        "--server-config", args.server_config,
        "--out-dir", args.out_dir, "--execute", "--i-am-at-the-estop",
        "--allow-controller-sim-arm-error",
    ]
    before = {p.parent.name for p in Path(args.out_dir).glob("batch_*/batch_summary.json")}
    before |= {p.name for p in Path(args.out_dir).glob("batch_*")}
    try:
        proc = subprocess.run(cmd, cwd=str(REPO_ROOT), input="RUN 1 joints\n",
                              capture_output=True, text=True, timeout=args.episode_timeout, check=False)
    except subprocess.TimeoutExpired:
        return {"status": "timeout", "run_dir": None, "log_path": None, "refusal": "episode_timeout"}
    verdict = _extract_verdict((proc.stdout or "") + (proc.stderr or ""))
    # only consider batch dirs created by THIS invocation
    summ = _new_batch_result(Path(args.out_dir), stem, before)
    if summ is None:
        tail = ((proc.stdout or "")[-400:] + " | " + (proc.stderr or "")[-400:]).strip()
        return {"status": "no_result", "run_dir": None, "log_path": None,
                "refusal": tail[:400], "safety_verdict": verdict}
    summ.setdefault("safety_verdict", verdict)
    return summ


def _extract_verdict(text: str) -> str | None:
    """Pull 'safety_verdict=<X>' from a WATCHDOG STOP / driver line, if present."""
    import re
    m = re.findall(r"safety_verdict=([A-Za-z_]+)", text or "")
    return m[-1] if m else None


def _new_batch_result(out_dir: Path, stem: str, before: set[str]) -> dict[str, Any] | None:
    new_batches = sorted(p for p in out_dir.glob("batch_*/batch_summary.json")
                         if p.parent.name not in before)
    for bs in reversed(new_batches):
        try:
            d = json.loads(bs.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        for r in d.get("results", []):
            if r.get("episode", "").startswith(stem) or r.get("episode_id", "").endswith(stem):
                return r
    return None


def episode_stems(episodes_dir: Path, only: list[str] | None) -> list[str]:
    stems = sorted(p.stem for p in episodes_dir.glob("episode_*.hdf5"))
    if only:
        wanted = set(only)
        stems = [s for s in stems if s in wanted]
    return stems


def manifest_lookup(out_dir: Path) -> dict[str, dict[str, Any]]:
    mf = out_dir / "episode_manifest.json"
    if not mf.exists():
        return {}
    data = json.loads(mf.read_text())
    out = {}
    for e in data.get("episodes", []):
        eid = e.get("episode_id", "")
        stem = eid.split("__")[-1]
        out[stem] = e
    return out


# ------------------------------------------------------------------- main -----
def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--episodes-dir", default="data_tcp/replay_profiling_20260620")
    p.add_argument("--episodes", default=None, help="comma-separated stems; default all")
    p.add_argument("--out-dir", default="outputs/tcp_pgprofile")
    p.add_argument("--server-config", default="rb_servo_server/config/local/stack_sim_replaybind.yaml")
    p.add_argument("--action-scale", type=float, default=1.0)
    p.add_argument("--time-scale", type=float, default=1.0)
    p.add_argument("--client-max-linear", type=float, default=5.0,
                   help="driver pre-flight speed guard (lifted high so the SERVER SMD does the real clamping)")
    p.add_argument("--client-max-angular", type=float, default=10.0)
    p.add_argument("--episode-timeout", type=float, default=300.0)
    p.add_argument("--max-episodes", type=int, default=0, help="0 = all")
    p.add_argument("--no-recover", action="store_true", help="do not toggle/restart on fault")
    p.add_argument("--resume", action="store_true", help="skip episodes that already have a result")
    args = p.parse_args(argv)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = load_config(None)
    only = [s.strip() for s in args.episodes.split(",")] if args.episodes else None
    stems = episode_stems(Path(args.episodes_dir), only)
    if args.max_episodes:
        stems = stems[: args.max_episodes]
    manifest = manifest_lookup(out_dir)

    results_path = out_dir / "episode_results.json"
    results: dict[str, Any] = {}
    if args.resume and results_path.exists():
        try:
            results = {r["stem"]: r for r in json.loads(results_path.read_text()).get("episodes", [])}
        except (OSError, json.JSONDecodeError, KeyError):
            results = {}

    def log(msg: str) -> None:
        print(msg, flush=True)

    log(f"campaign: {len(stems)} episodes  scale={args.action_scale} ts={args.time_scale} "
        f"client_clamp lin={args.client_max_linear} ang={args.client_max_angular}")

    for i, stem in enumerate(stems):
        if args.resume and stem in results and results[stem].get("primary_class"):
            continue
        eid = f"{Path(args.episodes_dir).name}__{stem}"
        mrow = manifest.get(stem, {})
        spp = mrow.get("speed_precheck_pass")
        vclass = mrow.get("validity_class")

        # health check — retry state read before concluding the stack is gone, and
        # NEVER start a second stack if a server process is already running (that
        # spawned duplicate stacks fighting over the command port / control boxes).
        st = None
        for _ in range(3):
            st = read_state(2.0)
            if st is not None:
                break
        if is_faulted(st) and not args.no_recover:
            recover(log)
        elif st is None and not _server_running():
            log(f"  [{i+1}/{len(stems)}] {stem}: no state AND no server process; starting stack")
            if not args.no_recover:
                start_stack()
        elif st is None:
            log(f"  [{i+1}/{len(stems)}] {stem}: state read timed out but server is up; proceeding")

        log(f"[{i+1}/{len(stems)}] {stem}  (speed_ok={spp})")
        summ = run_episode(stem, args, log)
        status = summ.get("status")
        run_dir = summ.get("run_dir")
        log_path = summ.get("log_path") or (str(Path(run_dir) / "log.csv") if run_dir else None)

        rec: dict[str, Any] = {
            "stem": stem, "episode_id": eid, "batch_status": status,
            "speed_precheck_pass": spp, "validity_class": vclass,
            "run_dir": run_dir, "refusal": summ.get("refusal"),
        }

        if status == "ok" and log_path and Path(log_path).exists():
            try:
                result = ana.analyze_run(Path(log_path), cfg.metrics,
                                         speed_precheck_pass=spp, validity_class=vclass,
                                         episode_id=eid)
                rd = Path(run_dir)
                (rd / "pgprofile_result.json").write_text(
                    json.dumps(result, indent=2, allow_nan=False, default=lambda o: None) + "\n")
                (rd / "pgprofile_summary.md").write_text(ana.render_summary(result))
                cls = result["classification"]
                rec["primary_class"] = cls["primary_class"]
                rec["risk_flags"] = cls["risk_flags"]
                rec["hard_counts_zero"] = cls["aggregate"]["hard_counts_zero"]
                rec["aggregate"] = cls["aggregate"]
                rec["B_ref_following"] = _b_digest(result)
                rec["ik_safety"] = _ik_digest(result)
            except Exception as exc:  # noqa: BLE001
                rec["primary_class"] = ana.NEEDS_MANUAL_REVIEW
                rec["refusal"] = f"analyze_error: {type(exc).__name__}: {exc}"
        else:
            # pre-flight refusal / mid-stream watchdog / fault -> classify from
            # the controller safety_verdict when available, else status + manifest.
            rec["safety_verdict"] = summ.get("safety_verdict")
            rec["primary_class"] = _classify_nonok(status, summ.get("refusal"), spp,
                                                   verdict=summ.get("safety_verdict"))

        results[stem] = rec
        log(f"    -> {rec['primary_class']}  risks={rec.get('risk_flags', [])}")
        _write_aggregate(out_dir, results)

        # post-fault recovery
        st2 = read_state(1.5)
        if is_faulted(st2) and not args.no_recover:
            recover(log)

    log("campaign complete")
    _write_aggregate(out_dir, results, final=True)
    return 0


def _classify_nonok(status: str | None, refusal: str | None, spp: bool | None,
                    verdict: str | None = None) -> str:
    # A mid-stream WATCHDOG STOP carries the controller safety_verdict — the most
    # accurate signal for a halted episode.
    v = (verdict or "").lower()
    if v:
        if "ikfailed" in v or v == "ikfail":
            return ana.IK_FAILURE
        if "singular" in v:
            return ana.SINGULARITY_RISK
        if "branch" in v:
            return ana.IK_BRANCH_RISK
        if "selfcollision" in v or "self_collision" in v:
            return ana.SELF_COLLISION_RISK
        if "floor" in v:
            return ana.FLOOR_RISK
        if "roi" in v:
            return ana.WORKSPACE_ROI_RISK
        if "tracking" in v:
            return ana.TRACKING_LAG_HIGH
        if "fault" in v or "robotfault" in v or "ems" in v or "estop" in v:
            return ana.WATCHDOG_OR_LEASE_FAILURE
    text = (refusal or "").lower()
    if "exceeds client speed clamp" in text or "speed" in text:
        return ana.SPEED_LIMITED
    if status in ("fault", "faulted") or "fault" in text:
        return ana.WATCHDOG_OR_LEASE_FAILURE
    if status in ("timeout", "skipped", "halted"):
        return ana.WATCHDOG_OR_LEASE_FAILURE
    if spp is False:
        return ana.SPEED_LIMITED
    return ana.NEEDS_MANUAL_REVIEW


def _b_digest(result: dict[str, Any]) -> dict[str, Any]:
    out = {}
    for arm, b in result["controller_reference_goal_following_B"].items():
        err = b.get("reference_after_B_vs_conditioned_goal", {})
        lag = b.get("tcp_ref_lag_vs_conditioned_goal", {})
        out[arm] = {
            "pos_p95_mm": _mm((err.get("position_m") or {}).get("p95")),
            "ori_p95_deg": _deg((err.get("orientation_rad") or {}).get("p95")),
            "lag_ms": (abs(lag.get("lag_sec")) * 1000.0 if isinstance(lag, dict) and lag.get("lag_sec") is not None else None),
            "span_ratio": b.get("reference_span_ratio"),
            "endpoint_err_mm": _mm(b.get("reference_endpoint_error_m")),
            "smd_clamp": b.get("smd_goal_clamped_count"),
        }
    return out


def _ik_digest(result: dict[str, Any]) -> dict[str, Any]:
    out = {}
    for arm, f in result["ik_safety_feasibility"].items():
        out[arm] = {k: f.get(k) for k in (
            "ik_success_ratio", "ik_branch_jump_count", "ik_min_singular_value_p05",
            "ik_solve_us_p95", "ik_solve_us_max", "ik_over_budget_2ms_count",
            "roi_clamp_count", "floor_clamp_count", "self_collision_count")}
    return out


def _mm(v):
    return (v * 1000.0) if v is not None else None


def _deg(v):
    import math
    return (math.degrees(v)) if v is not None else None


def _write_aggregate(out_dir: Path, results: dict[str, Any], final: bool = False) -> None:
    rows = list(results.values())
    hist: dict[str, int] = {}
    for r in rows:
        c = r.get("primary_class", "?")
        hist[c] = hist.get(c, 0) + 1
    payload = {
        "schema": "robotics_lab.tcp_pgprofile.campaign.v1",
        "n_done": len(rows),
        "class_histogram": dict(sorted(hist.items())),
        "final": final,
        "episodes": rows,
    }
    (out_dir / "episode_results.json").write_text(
        json.dumps(payload, indent=2, allow_nan=False, default=lambda o: None) + "\n")
    # csv
    import csv
    cols = ["stem", "episode_id", "primary_class", "batch_status", "speed_precheck_pass",
            "validity_class", "hard_counts_zero", "risk_flags", "refusal"]
    with (out_dir / "episode_results.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            rr = dict(r)
            rr["risk_flags"] = "|".join(r.get("risk_flags", []) or [])
            w.writerow({k: rr.get(k, "") for k in cols})


if __name__ == "__main__":
    raise SystemExit(main())
