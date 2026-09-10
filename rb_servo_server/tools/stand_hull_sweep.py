#!/usr/bin/env python3
"""OFFLINE ONLY: how much clearance does the stand's CoACD decomposition give away,
and what would a finer one cost?

The monitor does not check the stand's 38 k-triangle mesh -- coal cannot hull a
non-convex mesh, so it would stay a BVH (correct distances, far outside the per-eval
budget). make_rb5_850e_urdfs.py replaces it with 20 CoACD convex hulls, and those
OVER-APPROXIMATE the surface: the model reports the arm closer to the stand than it
is. That is the safe direction, but it is also pure padding stacked on top of
d_hard_m, and nobody had measured how much of it there is per parameter choice.

WHAT IS MEASURED, and why this way. Not a geometric surface metric: the number that
matters is the clearance THE GUARD REPORTS, so every variant is run through the real
CollisionMonitor (collision_pose_probe --poses) at real recorded arm poses, and
compared against a REFERENCE model built from the source STL as a BVH -- the same
"measure against the source STL, never the hulls" rule the 2026-09-06 mount
calibration used. The error of a variant at a pose is therefore

    err = stand_clearance(reference BVH) - stand_clearance(variant hulls)   >= 0

in millimetres, which is exactly what a tighter d_hard_m would have to cover. Cost is
the monitor's own eval_ms from the same runs, against safety.self_collision.mesh's
max_staleness_s budget.

    python3 rb_servo_server/tools/stand_hull_sweep.py --poses-from logs/servo_log_*.csv
    python3 rb_servo_server/tools/stand_hull_sweep.py --grid 20,40,60 --thresholds 0.08,0.05

Writes nothing into descriptions/: variants land in --work-dir (default a temp dir).
Promoting one is a separate, deliberate edit of make_rb5_850e_urdfs.py (STAND_HULLS /
coacd_params.json) plus a re-run of the mount-calibration audit, because the 27
contact poses were fitted against the source STL and their residuals have to be
re-confirmed on whatever geometry actually gets enforced.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SOURCE_STL = REPO / "rb_servo_server/descriptions/meshes/stands/dual_rb5_850e/dual_rb5_850e_stand_ver1.stl"
TRACKED_URDF = REPO / "rb_servo_server/descriptions/urdf/dual_rb5_850e_ver3.urdf"
TRACKED_HULL_DIR = REPO / "rb_servo_server/descriptions/meshes/stands/dual_rb5_850e/collision_ver1"
CONFIG = REPO / "rb_servo_server/config/stack_real.yaml"
PROBE_CANDIDATES = (
    REPO / "rb_servo_server/build/collision_pose_probe",
    REPO / "rb_servo_server/build/rbpodo_real_gate/collision_pose_probe",
)


def find_probe() -> Path:
    for p in PROBE_CANDIDATES:
        if p.exists():
            return p
    raise SystemExit(
        "collision_pose_probe not built. cmake --build rb_servo_server/build -j --target collision_pose_probe")


# ---------------------------------------------------------------------------
# poses


def poses_from_log(csv_path: Path, count: int, seed: int) -> list[tuple[float, ...]]:
    """Recorded arm poses, biased toward the ones that were CLOSE to something.

    Half the set is the globally nearest poses by selfcol_self_min_clearance_m (that
    column lumps arm<->arm in with arm<->stand, but a stand-hull change can only show
    up where an arm was near the stand, and those poses are in this half), and half is
    a uniform random sample so the comparison is not evaluated only at one posture."""
    rows: list[tuple[float, tuple[float, ...]]] = []
    with csv_path.open() as fh:
        reader = csv.DictReader(fh)
        cols = [f"left_q_actual_{i}" for i in range(6)] + [f"right_q_actual_{i}" for i in range(6)]
        missing = [c for c in cols if c not in (reader.fieldnames or [])]
        if missing:
            raise SystemExit(f"{csv_path}: missing columns {missing[:3]}")
        has_clear = "selfcol_self_min_clearance_m" in (reader.fieldnames or [])
        for row in reader:
            try:
                q = tuple(float(row[c]) for c in cols)
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(v) for v in q):
                continue
            d = math.inf
            if has_clear:
                try:
                    d = float(row["selfcol_self_min_clearance_m"])
                except (TypeError, ValueError):
                    d = math.inf
            rows.append((d if math.isfinite(d) else math.inf, q))
    if not rows:
        raise SystemExit(f"{csv_path}: no usable pose rows")
    half = max(1, count // 2)
    nearest = [q for _, q in sorted(rows, key=lambda r: r[0])[:half]]
    rng = random.Random(seed)
    spread = [q for _, q in rng.sample(rows, min(count - len(nearest), len(rows)))]
    # de-duplicate identical parked poses, which dominate a log by count
    seen: set[tuple[float, ...]] = set()
    out: list[tuple[float, ...]] = []
    for q in nearest + spread:
        key = tuple(round(v, 3) for v in q)
        if key in seen:
            continue
        seen.add(key)
        out.append(q)
    return out


def urdf_joint_limits(prefix: str, joint_names: list[str]) -> list[tuple[float, float]]:
    """(lower, upper) radians for one arm's actuated joints, from the tracked URDF."""
    root = ET.parse(TRACKED_URDF).getroot()
    out = []
    for base in joint_names:
        j = root.find(f"joint[@name='{prefix}{base}']")
        if j is None:
            raise SystemExit(f"unified URDF has no joint {prefix}{base}")
        lim = j.find("limit")
        if lim is None:
            raise SystemExit(f"{prefix}{base} has no <limit>")
        out.append((float(lim.get("lower", "0")), float(lim.get("upper", "0"))))
    return out


def random_poses(count: int, seed: int) -> list[tuple[float, ...]]:
    """Uniform random joint configurations inside the tracked URDF's limits.

    A recorded log cannot answer this question: one task visits one corner of the
    stand, and the hull padding is a property of the WHOLE surface. Random poses are
    not motions anyone would command -- they are probe points, and the only thing read
    off them is the arm<->stand distance, so reachability and self-collision do not
    matter.

    Joint ORDER comes from the config's kinematics.joint_names, not from the URDF's
    document order and never from sorting: the probe takes values in COMMAND order
    (base, shoulder, elbow, wrist1..3), and an alphabetical sort silently swaps elbow
    with shoulder -- which would pair each sampled angle with the wrong joint's limit
    and quietly probe a different envelope than the one reported."""
    import yaml  # noqa: PLC0415

    cfg = yaml.safe_load(CONFIG.read_text())
    kin = cfg.get("kinematics") or {}
    joint_names = list(kin.get("joint_names") or [])
    mesh = ((cfg.get("safety") or {}).get("self_collision") or {}).get("mesh") or {}
    left_prefix = str(mesh.get("left_prefix") or "")
    right_prefix = str(mesh.get("right_prefix") or "")
    if len(joint_names) != 6 or not left_prefix or not right_prefix:
        raise SystemExit("stack config: need kinematics.joint_names (6) and mesh left/right_prefix")
    lims = (urdf_joint_limits(left_prefix, joint_names) +
            urdf_joint_limits(right_prefix, joint_names))
    rng = random.Random(seed)
    deg = 180.0 / math.pi
    return [tuple(rng.uniform(lo, hi) * deg for lo, hi in lims) for _ in range(count)]


def write_poses(poses: list[tuple[float, ...]], path: Path) -> None:
    path.write_text("".join(",".join(f"{v:.4f}" for v in q) + "\n" for q in poses))


# ---------------------------------------------------------------------------
# geometry variants


def urdf_with_stand_collisions(meshes: list[str], out_path: Path) -> None:
    """The tracked unified URDF with stand_collision's <collision> list replaced.

    Everything else -- arm hulls, mounts, env_* boxes, joint limits -- is byte-identical
    to what the server enforces, so a difference between two runs is the stand geometry
    and nothing else."""
    tree = ET.parse(TRACKED_URDF)
    root = tree.getroot()
    link = root.find("link[@name='stand_collision']")
    if link is None:
        raise SystemExit("tracked unified URDF has no stand_collision link")
    for col in link.findall("collision"):
        link.remove(col)
    for filename in meshes:
        col = ET.SubElement(link, "collision")
        ET.SubElement(col, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geo = ET.SubElement(col, "geometry")
        ET.SubElement(geo, "mesh", {"filename": filename, "scale": "0.001 0.001 0.001"})
    tree.write(out_path, encoding="utf-8", xml_declaration=True)


def decompose(threshold: float, max_hulls: int, out_dir: Path) -> tuple[list[str], float]:
    """Run CoACD on the source stand mesh; returns (mesh paths, wall seconds)."""
    import trimesh  # noqa: PLC0415

    # Reuse a decomposition already sitting in --work-dir. CoACD is the slow half of a
    # sweep (10-60 s per variant) and it is deterministic for a given parameter set, so
    # re-running the comparison with a different POSE set should not pay for it again.
    existing = sorted(out_dir.glob("stand_hull_*.stl"))
    if existing:
        return [str(p) for p in existing], 0.0
    import coacd  # noqa: PLC0415 - optional heavy dep, only needed for new variants

    mesh = trimesh.load_mesh(str(SOURCE_STL))
    t0 = time.monotonic()
    parts = coacd.run_coacd(
        coacd.Mesh(mesh.vertices, mesh.faces),
        threshold=threshold,
        max_convex_hull=max_hulls,
        preprocess_mode="auto",
        preprocess_resolution=50,
        resolution=1000,
        mcts_nodes=10,
        mcts_iterations=60,
        mcts_max_depth=3,
        pca=False,
        merge=True,
    )
    secs = time.monotonic() - t0
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for i, (verts, faces) in enumerate(parts):
        hull = trimesh.Trimesh(vertices=verts, faces=faces).convex_hull
        p = out_dir / f"stand_hull_{i:03d}.stl"
        hull.export(p)
        paths.append(p)
    return [str(p) for p in paths], secs


# ---------------------------------------------------------------------------
# measurement


def run_probe(probe: Path, urdf: Path, poses_file: Path, near: int) -> list[dict[str, float]]:
    """One probe run over the whole pose file.

    --swept 1 IS LOAD-BEARING. safety.self_collision.mesh.swept_samples is 2, and the
    monitor's swept path interpolates from the PREVIOUS evaluated configuration to the
    current one (collision_monitor.cpp: prev_eval_q -> q) so a fast step cannot tunnel
    an obstacle. That is right for a servo loop, where consecutive evaluations are 2 ms
    apart, and meaningless here, where consecutive poses are unrelated random samples:
    each "sweep" teleports an arm across the cell and reports the worst point on the
    way. Measured 2026-09-10 before this was pinned: a pose whose true arm<->stand
    clearance is 82.3 mm reported -103.8 mm in a batch, and every variant inherited the
    same fiction, which made a 186 mm err_max look like hull padding.

    Cost therefore comes out at endpoint-only. The server pays ~2x that (N samples per
    eval), and all variants scale by the same factor, so the comparison is unaffected --
    the absolute per-eval figure to check a budget against is the single-pose
    --repeat run at the config's swept_samples."""
    cmd = [str(probe), "--config", str(CONFIG), "--poses", str(poses_file),
           "--unified-urdf", str(urdf), "--near", str(near), "--swept", "1"]
    res = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO)
    if res.returncode != 0:
        raise SystemExit(f"probe failed ({res.returncode}):\n{res.stderr[-2000:]}")
    rows = []
    header: list[str] | None = None
    for line in res.stdout.splitlines():
        if line.startswith("#i\t"):
            header = line.lstrip("#").split("\t")
            continue
        if header is None or not line or line.startswith("["):
            continue
        parts = line.split("\t")
        if len(parts) != len(header):
            continue
        rows.append({k: float(v) for k, v in zip(header, parts)})
    if not rows:
        raise SystemExit("probe produced no pose rows")
    return rows


def pct(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    return s[int(p * (len(s) - 1))]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--poses-from", type=Path, help="servo log CSV to draw arm poses from")
    ap.add_argument("--poses", type=Path, help="pose file (12 comma-separated joints per line)")
    ap.add_argument("--count", type=int, default=200, help="poses to draw from the log (default 200)")
    ap.add_argument("--random", type=int, default=0, metavar="N",
                    help="probe N random joint configurations instead of a log, then keep the "
                         "ones whose arm<->stand clearance is inside --stand-max")
    ap.add_argument("--keep", type=int, default=300, help="poses kept from --random (default 300)")
    ap.add_argument("--stand-max", type=float, default=150.0,
                    help="mm; upper end of the arm<->stand clearance band a random pose is kept "
                         "from -- farther away the padding cannot matter (default 150)")
    ap.add_argument("--stand-min", type=float, default=10.0,
                    help="mm; lower end of that band (default 10). NOT 0: the hulls CONTAIN the "
                         "source, so a hull clearance above 0 guarantees the pose is outside the "
                         "true stand too, which keeps the comparison out of the penetration "
                         "regime where coal's BVH distance is not a signed depth")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--grid", default="30,40,60", help="max_convex_hull values (default 30,40,60)")
    ap.add_argument("--thresholds", default="0.08,0.05", help="CoACD thresholds (default 0.08,0.05)")
    ap.add_argument("--near", type=int, default=200, help="--near passed to the probe (default 200)")
    ap.add_argument("--work-dir", type=Path, help="keep the generated variants here (default: temp)")
    ap.add_argument("--json-out", type=Path, help="also write the table as JSON")
    args = ap.parse_args(argv)

    probe = find_probe()
    tmp = Path(tempfile.mkdtemp(prefix="stand_hull_sweep_")) if args.work_dir is None else args.work_dir
    tmp.mkdir(parents=True, exist_ok=True)

    if args.random:
        cand = random_poses(args.random, args.seed)
        cand_file = tmp / "poses_candidates.txt"
        write_poses(cand, cand_file)
        tracked0 = sorted(TRACKED_HULL_DIR.glob("stand_hull_*.stl"))
        screen_urdf = tmp / "unified_screen.urdf"
        urdf_with_stand_collisions(
            [f"../meshes/stands/dual_rb5_850e/collision_ver1/{p.name}" for p in tracked0], screen_urdf)
        print(f"screening {len(cand)} random poses for arm<->stand proximity ...", flush=True)
        screened = run_probe(probe, screen_urdf, cand_file, args.near)
        band = [(r["stand_mm"], cand[int(r["i"])]) for r in screened
                if math.isfinite(r["stand_mm"]) and args.stand_min <= r["stand_mm"] <= args.stand_max]
        if not band:
            raise SystemExit(
                f"no random pose landed in {args.stand_min}..{args.stand_max} mm of the stand")
        # A SPREAD over the band, not the N closest. The closest poses cluster on one
        # hot spot of the stand, and the question is how much padding the decomposition
        # carries over the whole surface the arms can approach.
        rng = random.Random(args.seed + 1)
        picked = rng.sample(band, min(args.keep, len(band)))
        picked.sort(key=lambda t: t[0])
        poses = [q for _, q in picked]
        poses_file = tmp / "poses.txt"
        write_poses(poses, poses_file)
        n_poses = len(poses)
        print(f"  {len(band)} of {len(cand)} random poses in band; kept {n_poses}, "
              f"stand clearance {picked[0][0]:.1f}..{picked[-1][0]:.1f} mm")
    elif args.poses:
        poses_file = args.poses
        n_poses = sum(1 for line in poses_file.read_text().splitlines() if line and not line.startswith("#"))
    elif args.poses_from:
        poses = poses_from_log(args.poses_from, args.count, args.seed)
        poses_file = tmp / "poses.txt"
        write_poses(poses, poses_file)
        n_poses = len(poses)
    else:
        raise SystemExit("need --poses-from <servo log csv> or --poses <file>")
    print(f"poses: {n_poses} (from {args.poses_from or args.poses})")

    # REFERENCE: the source STL itself. coal keeps a non-convex mesh as a BVH, which
    # is the exact-distance (slow) path -- that is the point, it is the truth the
    # hulls are approximating.
    ref_urdf = tmp / "unified_reference_bvh.urdf"
    urdf_with_stand_collisions(
        [f"../meshes/stands/dual_rb5_850e/{SOURCE_STL.name}"], ref_urdf)
    print("reference: source STL as BVH (exact distance) ...", flush=True)
    ref = run_probe(probe, ref_urdf, poses_file, args.near)
    ref_stand = [r["stand_mm"] for r in ref]
    ref_eval = [r["eval_ms"] for r in ref]
    print(f"  stand clearance p50 {pct(ref_stand,0.5):.1f} / min {min(ref_stand):.1f} mm"
          f"   eval p50 {pct(ref_eval,0.5):.2f} ms")

    variants: list[tuple[str, Path, int, float]] = []
    tracked = sorted(TRACKED_HULL_DIR.glob("stand_hull_*.stl"))
    tracked_urdf = tmp / "unified_tracked.urdf"
    urdf_with_stand_collisions(
        [f"../meshes/stands/dual_rb5_850e/collision_ver1/{p.name}" for p in tracked], tracked_urdf)
    variants.append((f"tracked (20 hulls, thr 0.08)", tracked_urdf, len(tracked), 0.0))

    for thr in [float(t) for t in args.thresholds.split(",") if t.strip()]:
        for mh in [int(v) for v in args.grid.split(",") if v.strip()]:
            name = f"h{mh}_thr{thr:g}"
            out_dir = tmp / name
            print(f"decomposing {name} ...", flush=True)
            meshes, secs = decompose(thr, mh, out_dir)
            urdf = tmp / f"unified_{name}.urdf"
            urdf_with_stand_collisions([str(Path(m).resolve()) for m in meshes], urdf)
            variants.append((name, urdf, len(meshes), secs))

    print()
    hdr = (f"{'variant':<26}{'hulls':>6}{'err_mean':>10}{'err_p95':>9}{'err_max':>9}"
           f"{'eval_p50':>10}{'eval_p95':>10}{'coacd_s':>9}")
    print(hdr)
    print("-" * len(hdr))
    table = []
    for name, urdf, n_hulls, secs in variants:
        rows = run_probe(probe, urdf, poses_file, args.near)
        errs, evals = [], [r["eval_ms"] for r in rows]
        for r, rr in zip(rows, ref):
            if math.isfinite(r["stand_mm"]) and math.isfinite(rr["stand_mm"]):
                errs.append(rr["stand_mm"] - r["stand_mm"])
        entry = {
            "variant": name, "hulls": n_hulls,
            "err_min_mm": min(errs) if errs else float("nan"),
            "err_mean_mm": sum(errs) / len(errs) if errs else float("nan"),
            "err_p95_mm": pct(errs, 0.95), "err_max_mm": max(errs) if errs else float("nan"),
            "eval_p50_ms": pct(evals, 0.5), "eval_p95_ms": pct(evals, 0.95),
            "coacd_s": secs, "poses_compared": len(errs),
        }
        table.append(entry)
        print(f"{name:<26}{n_hulls:>6}{entry['err_mean_mm']:>10.2f}{entry['err_p95_mm']:>9.2f}"
              f"{entry['err_max_mm']:>9.2f}{entry['eval_p50_ms']:>10.3f}{entry['eval_p95_ms']:>10.3f}"
              f"{secs:>9.1f}")
    budget = 50.0
    worst = min((e["err_min_mm"] for e in table if e["err_min_mm"] == e["err_min_mm"]), default=0.0)
    if worst < -0.5:
        print(f"\nWARNING: a variant read up to {-worst:.2f} mm MORE clearance than the source mesh "
              f"(err_min). The hulls are supposed to contain the source, so that is either a "
              f"decomposition that does not cover it or a coal BVH artefact -- do not promote a "
              f"variant on this table until it is explained.")
    print(f"\nerr = reference(source STL BVH) - variant, mm; >0 means the model reports the arm CLOSER")
    print(f"eval budget: safety.self_collision.mesh.max_staleness_s = {budget:.0f} ms")
    print(f"reference (BVH) eval p50 {pct(ref_eval,0.5):.2f} ms -- the cost of not decomposing at all")
    print(f"variants kept in {tmp}")
    if args.json_out:
        args.json_out.write_text(json.dumps(
            {"poses": n_poses, "reference_eval_p50_ms": pct(ref_eval, 0.5), "table": table}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
