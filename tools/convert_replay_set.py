#!/usr/bin/env python3
"""Batch-convert the selected raw UMI episodes into a single unified data_tcp set.

Reads the INCLUDE list produced by ``tools/select_replay_episodes.py`` and converts
each episode with ``umi-convert`` (retarget + tool offset) into one flat output
folder, renumbered ``episode_0000.hdf5`` ... The conversion is ``--poses-only`` so
the slim outputs (pose/gripper/timestamp, no camera images) suit TcpTargetPose
motion replay/profiling, which never reads images.

Traceability (new index -> original session/episode) is preserved in
``manifest.json`` and ``index_map.csv`` even though the flat layout drops the
per-session directory structure.

Fail-soft: a failed episode is logged and skipped; the batch continues.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
POLICY_RUNNER = REPO_ROOT / "policy_runner"
for _p in (str(REPO_ROOT), str(POLICY_RUNNER)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from policy_runner.umi_pipeline import convert_umi_episode

SCHEMA = "robotics_lab.convert_replay_set.v1"
DEFAULT_INCLUDE = REPO_ROOT / "outputs" / "episode_selection" / "include_list.txt"
DEFAULT_OUT = REPO_ROOT / "data_tcp" / "replay_profiling_20260620"
DEFAULT_RETARGET = REPO_ROOT / "calibration" / "umi_retarget_eelocal.yaml"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--include-list", type=Path, default=DEFAULT_INCLUDE)
    p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    p.add_argument("--retarget-config", type=Path, default=DEFAULT_RETARGET)
    p.add_argument("--format", default="robotics_lab_dual_arm")
    p.add_argument("--with-images", action="store_true", help="Include camera images (default: poses-only)")
    p.add_argument("--force", action="store_true", help="Reconvert even if the output episode already exists")
    p.add_argument("--limit", type=int, default=0, help="Convert at most N episodes (0 = all; for smoke tests)")
    return p.parse_args(argv)


def session_episode(src: Path) -> tuple[str, str]:
    return (src.parent.name, src.stem)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    sources = [Path(l.strip()) for l in args.include_list.read_text().splitlines() if l.strip()]
    if args.limit > 0:
        sources = sources[: args.limit]
    if not sources:
        print(f"no sources in {args.include_list}", file=sys.stderr)
        return 1

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    poses_only = not args.with_images
    width = max(4, len(str(len(sources) - 1)))

    records: list[dict] = []
    failures: list[dict] = []
    log_path = out_dir / "convert.log"
    log = log_path.open("w", encoding="utf-8")

    def emit(msg: str) -> None:
        print(msg, flush=True)
        log.write(msg + "\n")
        log.flush()

    emit(f"convert_replay_set: {len(sources)} episodes -> {out_dir} (poses_only={poses_only})")
    for idx, src in enumerate(sources):
        sess, ep = session_episode(src)
        out_name = f"episode_{idx:0{width}d}.hdf5"
        out_path = out_dir / out_name
        rec = {"index": idx, "output": out_name, "source_session": sess, "source_episode": ep, "source_path": str(src)}
        if out_path.exists() and not args.force:
            rec["status"] = "skipped_exists"
            records.append(rec)
            emit(f"[{idx + 1}/{len(sources)}] skip exists {out_name}  <- {sess}/{ep}")
            continue
        if not src.exists():
            rec["status"] = "missing_source"
            failures.append(rec)
            records.append(rec)
            emit(f"[{idx + 1}/{len(sources)}] MISSING SOURCE {sess}/{ep}")
            continue
        try:
            manifest = convert_umi_episode(
                src,
                out_path,
                output_format=args.format,
                retarget_config=args.retarget_config,
                poses_only=poses_only,
            )
            epi = (manifest.get("episodes") or [{}])[0]
            rec["status"] = "ok"
            rec["frame_count"] = epi.get("frame_count")
            rec["duration_sec"] = epi.get("duration_sec")
            rec["size_bytes"] = out_path.stat().st_size
            records.append(rec)
            emit(f"[{idx + 1}/{len(sources)}] ok {out_name}  <- {sess}/{ep}  frames={rec.get('frame_count')}")
        except Exception as exc:  # fail-soft per episode
            rec["status"] = "error"
            rec["error"] = str(exc)
            failures.append(rec)
            records.append(rec)
            emit(f"[{idx + 1}/{len(sources)}] ERROR {sess}/{ep}: {exc}")
            if out_path.exists():
                out_path.unlink()  # drop partial output

    ok = [r for r in records if r["status"] == "ok"]
    skipped = [r for r in records if r["status"] == "skipped_exists"]
    total_bytes = sum(r.get("size_bytes", 0) for r in ok)
    summary = {
        "schema": SCHEMA,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "out_dir": str(out_dir),
        "retarget_config": str(args.retarget_config),
        "format": args.format,
        "poses_only": poses_only,
        "requested": len(sources),
        "converted": len(ok),
        "skipped_existing": len(skipped),
        "failed": len(failures),
        "total_output_bytes": total_bytes,
    }

    # Combined manifest (overwrites the per-episode manifest.json that umi-convert
    # wrote on each call) + a flat index map for traceability.
    (out_dir / "manifest.json").write_text(
        json.dumps({"summary": summary, "episodes": records}, indent=2), encoding="utf-8"
    )
    with (out_dir / "index_map.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["index", "output", "source_session", "source_episode", "status", "frame_count", "size_bytes"])
        for r in records:
            w.writerow([r["index"], r["output"], r["source_session"], r["source_episode"], r["status"], r.get("frame_count", ""), r.get("size_bytes", "")])

    emit(
        f"DONE converted={len(ok)} skipped={len(skipped)} failed={len(failures)} "
        f"size={total_bytes / 1e6:.1f}MB -> {out_dir}"
    )
    if failures:
        emit("FAILURES: " + ", ".join(f"{r['source_session']}/{r['source_episode']}" for r in failures))
    log.close()
    print(f"wrote {out_dir / 'manifest.json'}")
    print(f"wrote {out_dir / 'index_map.csv'}")
    print(f"wrote {log_path}")
    return 0 if not failures else 2


if __name__ == "__main__":
    raise SystemExit(main())
