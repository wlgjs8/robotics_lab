#!/usr/bin/env python3
"""Package a pgprofile campaign directory into the consolidated deliverable set.

Given a campaign dir (e.g. outputs/tcp_pgprofile_ts1_vffx) this emits:
  - profile_summary.json / profile_summary.csv  (aggregate + per-class stats)
  - stage_summary.json                          (single consolidated stage record:
        config, exact command, git commit, velocity_feedforward parser default,
        class histogram, threshold pass/fail, paths)
  - representative_logs/INDEX.csv + symlinks     (per-class representative log.csv)
  - PROVENANCE.md                                (commit / command / vff-default / config)
Items already present in the dir (episode_manifest.*, per-episode run_meta.json /
metrics.json / pgprofile_result.json) are referenced, not duplicated.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

# representative-log selection: (class, how many, sort key, descending)
REPRESENTATIVE = [
    ("SPEED_LIMITED", 5, "speed_margin", True),
    ("SELF_COLLISION_RISK", 10, "self_collision", True),
    ("IK_FAILURE", 9999, None, False),
    ("IK_BRANCH_RISK", 9999, "branch", True),
    ("TRACKING_LAG_HIGH", 10, "pos_p95", True),
    ("SINGULARITY_RISK", 9999, None, False),
]


def git_commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], cwd=str(REPO_ROOT),
                              capture_output=True, text=True, check=False).stdout.strip()
    except Exception:  # noqa: BLE001
        return "unknown"


def vff_parser_default() -> dict[str, Any]:
    """Grep the C++ parser for the velocity_feedforward default + parse site."""
    hpp = REPO_ROOT / "rb_servo_server/include/rb_servo/config/config.hpp"
    cpp = REPO_ROOT / "rb_servo_server/src/config/config.cpp"
    default_line = ""
    for line in hpp.read_text(errors="ignore").splitlines():
        if "velocity_feedforward" in line and "=" in line:
            default_line = line.strip()
            break
    has_guard = "if (has(smd, \"velocity_feedforward\"))" in cpp.read_text(errors="ignore")
    return {
        "default_value": False if "= false" in default_line else None,
        "evidence_hpp": f"{hpp.relative_to(REPO_ROOT)}: {default_line}",
        "set_only_when_key_present": has_guard,
        "interpretation": ("velocity_feedforward defaults to false; config.cpp sets it "
                           "only when the YAML key is present, so an absent key == false."),
    }


def _remap_run_dir(run_dir: str | None, base: Path) -> Path | None:
    """Stored run_dir may predate a campaign-dir rename; remap to the actual base."""
    if not run_dir:
        return None
    p = Path(run_dir)
    if p.exists():
        return p
    # find the '<episode_id>/runs/<batch>' tail and reattach it under base
    parts = p.parts
    for i, part in enumerate(parts):
        if part.startswith("replay_profiling_") or "__episode_" in part:
            cand = base.joinpath(*parts[i:])
            if cand.exists():
                return cand
    return p if p.exists() else None


def _num(v: Any) -> float | None:
    try:
        f = float(v)
        return f
    except (TypeError, ValueError):
        return None


def _pctl(xs: list[float], p: float) -> float | None:
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    i = min(len(xs) - 1, int(round(p / 100 * (len(xs) - 1))))
    return xs[i]


def _arm_vals(rec: dict, group: str, key: str) -> list[float]:
    out = []
    for arm in ("left", "right"):
        v = ((rec.get(group) or {}).get(arm) or {}).get(key)
        n = _num(v)
        if n is not None:
            out.append(n)
    return out


def build_summary(results: dict, manifest: dict) -> dict[str, Any]:
    eps = results.get("episodes", [])
    hist = results.get("class_histogram", {})
    man_by = {e["episode_id"].split("__")[-1]: e for e in manifest.get("episodes", [])}

    def gather(key_group, key, pred=None):
        vals = []
        for r in eps:
            if pred and not pred(r):
                continue
            vals += _arm_vals(r, key_group, key)
        return vals

    def stat_block(pred=None):
        return {
            "B_ref_vs_cond_pos_p95_mm": {p: _pctl(gather("B_ref_following", "pos_p95_mm", pred), q)
                                         for p, q in (("p50", 50), ("p95", 95), ("max", 100))},
            "B_ref_vs_cond_ori_p95_deg": {p: _pctl(gather("B_ref_following", "ori_p95_deg", pred), q)
                                          for p, q in (("p50", 50), ("p95", 95), ("max", 100))},
            "B_lag_ms": {p: _pctl(gather("B_ref_following", "lag_ms", pred), q)
                         for p, q in (("p50", 50), ("p95", 95), ("max", 100))},
            "B_span_ratio": {p: _pctl(gather("B_ref_following", "span_ratio", pred), q)
                             for p, q in (("p50", 50), ("p95", 95))},
            "B_endpoint_err_mm": {p: _pctl(gather("B_ref_following", "endpoint_err_mm", pred), q)
                                  for p, q in (("p50", 50), ("p95", 95), ("max", 100))},
            "ik_solve_us_p95": {p: _pctl(gather("ik_safety", "ik_solve_us_p95", pred), q)
                                for p, q in (("p50", 50), ("p95", 95), ("max", 100))},
            "ik_solve_us_max": {p: _pctl(gather("ik_safety", "ik_solve_us_max", pred), q)
                                for p, q in (("p50", 50), ("p95", 95), ("max", 100))},
            "ik_min_singular_value_p05": {p: _pctl(gather("ik_safety", "ik_min_singular_value_p05", pred), q)
                                          for p, q in (("p50", 50), ("p10", 10), ("min", 0))},
        }

    per_class = {cls: stat_block(lambda r, c=cls: r.get("primary_class") == c) for cls in hist}
    return {
        "n_episodes": results.get("n_done", len(eps)),
        "class_histogram": dict(sorted(hist.items())),
        "real_ready_count": hist.get("REAL_READY_TS_1P0", 0),
        "overall": stat_block(),
        "per_class": per_class,
        "speed_precheck_pass_count": sum(1 for e in manifest.get("episodes", []) if e.get("speed_precheck_pass")),
        "thresholds": {
            "B_pos_p95_mm": 10.0, "B_ori_p95_deg": 2.0, "B_lag_ms": 150.0,
            "ik_solve_p95_us": 1000.0, "ik_solve_max_us": 3000.0,
            "singular_region_eps": 0.06, "hard_counts": "zero",
        },
    }


def write_csv_summary(summary: dict, path: Path) -> None:
    rows = []
    overall = summary["overall"]
    for metric, block in overall.items():
        row = {"scope": "overall", "metric": metric}
        row.update({k: block.get(k) for k in block})
        rows.append(row)
    for cls, blocks in summary["per_class"].items():
        for metric, block in blocks.items():
            row = {"scope": cls, "metric": metric}
            row.update({k: block.get(k) for k in block})
            rows.append(row)
    cols = ["scope", "metric", "p50", "p95", "max", "p10", "min"]
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in cols})


def select_representative(results: dict, man_by: dict) -> dict[str, list[dict]]:
    eps = results.get("episodes", [])
    out: dict[str, list[dict]] = {}
    for cls, n, sort_key, desc in REPRESENTATIVE:
        cand = [r for r in eps if r.get("primary_class") == cls and r.get("run_dir")]

        def keyf(r):
            if sort_key == "self_collision":
                return sum(_num((r.get("ik_safety", {}).get(a) or {}).get("self_collision_count")) or 0
                           for a in ("left", "right"))
            if sort_key == "branch":
                return sum(_num((r.get("ik_safety", {}).get(a) or {}).get("ik_branch_jump_count")) or 0
                           for a in ("left", "right"))
            if sort_key == "pos_p95":
                return max([_num((r.get("B_ref_following", {}).get(a) or {}).get("pos_p95_mm")) or 0
                            for a in ("left", "right")] + [0])
            if sort_key == "speed_margin":
                return _num(man_by.get(r["stem"], {}).get("linear_speed_margin")) or 0
            return 0

        cand.sort(key=keyf, reverse=desc)
        out[cls] = cand[:n]
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dir", required=True, help="campaign dir, e.g. outputs/tcp_pgprofile_ts1_vffx")
    p.add_argument("--server-config", default="rb_servo_server/config/local/stack_sim.yaml")
    p.add_argument("--time-scale", type=float, default=1.0)
    p.add_argument("--action-scale", type=float, default=1.0)
    p.add_argument("--velocity-feedforward", default="false (parser default; key absent in as-tested yaml)")
    args = p.parse_args(argv)

    base = Path(args.dir)
    results = json.loads((base / "episode_results.json").read_text())
    manifest = json.loads((base / "episode_manifest.json").read_text()) if (base / "episode_manifest.json").exists() else {"episodes": []}

    # speed margins for SPEED_LIMITED selection
    man_by = {e["episode_id"].split("__")[-1]: e for e in manifest.get("episodes", [])}

    summary = build_summary(results, manifest)
    (base / "profile_summary.json").write_text(json.dumps(summary, indent=2, default=lambda o: None) + "\n")
    write_csv_summary(summary, base / "profile_summary.csv")

    commit = git_commit()
    vff = vff_parser_default()
    exact_cmd = (
        "ACTION_SOURCE=none make run MODE=sim   # server: rb_servo_server/config/local/stack_sim.yaml\n"
        "python3 scripts/run_pgprofile_campaign.py --resume \\\n"
        f"  --episodes-dir data_tcp/replay_profiling_20260620 --out-dir {base} \\\n"
        f"  --server-config {args.server_config} \\\n"
        f"  --action-scale {args.action_scale} --time-scale {args.time_scale} \\\n"
        "  --client-max-linear 5.0 --client-max-angular 10.0\n"
        "# per episode the campaign calls batch_replay_episodes.py with:\n"
        "#   --init-mode joints --init-left-joints=-131.7,73,113.4,-80.9,-107.1,-145.9 \\\n"
        "#   --init-right-joints=135.1,-64,-114.5,84.4,112.5,129.9 \\\n"
        "#   --source ee_local --mode clean_foh_se3 --segment auto-largest \\\n"
        "#   --action-scale=1.0 --time-scale=1.0 --skip-failed-episodes \\\n"
        "#   --max-linear-speed-m-s=5.0 --max-angular-speed-rad-s=10.0 \\\n"
        "#   --server-config stack_sim_replaybind.yaml --execute --i-am-at-the-estop \\\n"
        "#   --allow-controller-sim-arm-error"
    )

    stage = {
        "schema": "robotics_lab.tcp_pgprofile.stage_summary.v1",
        "stage_dir": str(base),
        "git_commit": commit,
        "git_branch": "dev",
        "server_config_path": args.server_config,
        "driver_server_config_path": "rb_servo_server/config/local/stack_sim_replaybind.yaml",
        "as_tested_config_snapshot": str(base / "current_smd" / "as_tested_stack_sim.yaml"),
        "action_scale": args.action_scale,
        "time_scale": args.time_scale,
        "velocity_feedforward": args.velocity_feedforward,
        "velocity_feedforward_parser_default": vff,
        "client_speed_guard": {"linear_m_s": 5.0, "angular_rad_s": 10.0,
                               "note": "driver pre-flight guard lifted; SERVER SMD does the real clamp"},
        "exact_command": exact_cmd,
        "class_histogram": summary["class_histogram"],
        "real_ready_count": summary["real_ready_count"],
        "artifacts": {
            "episode_manifest": ["episode_manifest.json", "episode_manifest.csv"],
            "episode_results": ["episode_results.json", "episode_results.csv"],
            "profile_summary": ["profile_summary.json", "profile_summary.csv"],
            "report": "current_smd/REPORT.md",
            "plots": "current_smd/plots/",
            "per_episode": "<episode_id>/runs/<batch>/{run_meta.json,metrics.json,log.csv,pgprofile_result.json}",
            "representative_logs": "representative_logs/INDEX.csv",
        },
    }
    (base / "stage_summary.json").write_text(json.dumps(stage, indent=2, default=lambda o: None) + "\n")

    # representative logs (symlinks + index)
    rep = select_representative(results, man_by)
    rep_dir = base / "representative_logs"
    rep_dir.mkdir(exist_ok=True)
    index_rows = []
    for cls, recs in rep.items():
        cls_dir = rep_dir / cls
        cls_dir.mkdir(exist_ok=True)
        for r in recs:
            rd = _remap_run_dir(r.get("run_dir"), base)
            if rd is None:
                continue
            log = rd / "log.csv"
            if not log.exists():
                continue
            link = cls_dir / f"{r['stem']}.csv"
            try:
                if link.is_symlink() or link.exists():
                    link.unlink()
                link.symlink_to(Path(os.path.relpath(log, cls_dir)))
            except OSError:
                pass
            mrow = man_by.get(r["stem"], {})
            index_rows.append({
                "primary_class": cls, "stem": r["stem"],
                "log_csv": str(log),
                "symlink": str(link),
                "run_meta": str(rd / "run_meta.json"),
                "metrics": str(rd / "metrics.json"),
                "pgprofile_result": str(rd / "pgprofile_result.json"),
                "safety_verdict": r.get("safety_verdict"),
                "linear_speed_margin": mrow.get("linear_speed_margin"),
                "required_time_scale_estimate": mrow.get("required_time_scale_estimate"),
            })
    with (rep_dir / "INDEX.csv").open("w", newline="") as fh:
        cols = ["primary_class", "stem", "log_csv", "symlink", "run_meta", "metrics",
                "pgprofile_result", "safety_verdict", "linear_speed_margin", "required_time_scale_estimate"]
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(index_rows)
    (rep_dir / "INDEX.json").write_text(json.dumps(index_rows, indent=2, default=lambda o: None) + "\n")

    # PROVENANCE.md
    prov = [
        f"# PROVENANCE — {base.name}",
        "",
        f"- git commit: `{commit}` (branch dev)",
        f"- server control config (the evaluated profile): `{args.server_config}`",
        f"  snapshot: `{base/'current_smd'/'as_tested_stack_sim.yaml'}`",
        f"- driver config: `rb_servo_server/config/local/stack_sim_replaybind.yaml` "
        "(identical control params; differs only in state ports + gripper block)",
        f"- action_scale = {args.action_scale}, time_scale = {args.time_scale}, "
        f"velocity_feedforward = {args.velocity_feedforward}",
        "",
        "## velocity_feedforward parser default (item 10)",
        f"- default value: **{vff['default_value']}**",
        f"- evidence: `{vff['evidence_hpp']}`",
        f"- set only when YAML key present: {vff['set_only_when_key_present']}",
        f"- {vff['interpretation']}",
        "",
        "## exact command line (item 7)",
        "```bash",
        exact_cmd,
        "```",
        "",
        "## real-motion target config (item 9)",
        "- `rb_servo_server/config/local/stack_real_replaybind.yaml` carries the same "
        "TcpTargetPose replay wiring for real arms. Control params (SMD/IK/servo_j) match "
        "this sim profile; the real config additionally ENFORCES floor + self-collision "
        "(monitor_only:false) and ROI, and removes the controller-sim carve-outs. Real motion "
        "stays gated (site-local config + operator + E-stop). See repo file for the exact block.",
        "",
        f"## representative logs (item 6): see `representative_logs/INDEX.csv` ({len(index_rows)} entries)",
    ]
    (base / "PROVENANCE.md").write_text("\n".join(prov) + "\n")

    print(f"packaged {base}: profile_summary.{{json,csv}}, stage_summary.json, "
          f"representative_logs/ ({len(index_rows)} logs), PROVENANCE.md")
    print(f"  vff parser default = {vff['default_value']}; commit {commit[:10]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
