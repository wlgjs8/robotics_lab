#!/usr/bin/env python3
"""Build the viser "IK 불가 영역" overlay mesh from an IK-feasibility grid.

Companion to tools/reach_envelope.py. Where the reach envelope is the FK OUTER
shell (positions too FAR to reach), this marks positions that are INSIDE the
reach radius yet have NO inverse-kinematics solution for any sampled approach
direction — the genuine "IK holes" (inner dead zone, lower/back pockets, and the
near-full-extension shell where orientation freedom collapses).

The feasibility itself is computed by the C++ tool `ik_feasibility_grid`, which
reuses the SERVER's real Pinocchio IK solver (rb_servo_core) so the map matches
the deployed solver rather than a re-implementation. This script only turns the
resulting occupancy grid into a translucent surface mesh:

  occupied voxels --(exposed-face extraction, 6-neighbour)--> blocky boundary
                  --(trimesh Humphrey smoothing)--> rounded isosurface

Output mirrors the reach-envelope npz schema (shell_vertices_base_m / shell_faces
in the ARM-BASE frame) so rb_servo_gui/scene.py renders it under each arm's
/stand/<side>_base node — one base-frame mesh, mirrored for both arms.

Usage:
    # generate the grid (slow) + build the mesh in one go:
    python3 tools/ik_infeasible_region.py
    # reuse an already-computed grid:
    python3 tools/ik_infeasible_region.py --grid /tmp/ik_feasibility_grid.json
"""
from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
from pathlib import Path

import numpy as np


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _default_grid_exe() -> Path:
    return _repo_root() / "rb_servo_server" / "build" / "ik_feasibility_grid"


def _default_out() -> Path:
    return _repo_root() / "rb_servo_server" / "descriptions" / "ik_infeasible_rb3_730e.npz"


def run_grid_tool(exe: Path, out_json: Path, spacing: float, orientations: int,
                  seeds: int, threads: int) -> None:
    """Invoke the C++ feasibility sampler to produce the occupancy grid JSON."""
    if not exe.exists():
        raise SystemExit(
            f"feasibility tool not built: {exe}\n"
            f"  build it first: cmake --build rb_servo_server/build --target ik_feasibility_grid"
        )
    cmd = [str(exe), "--spacing-m", str(spacing), "--orientations", str(orientations),
           "--seeds", str(seeds), "--out", str(out_json)]
    if threads > 0:
        cmd += ["--threads", str(threads)]
    print("running:", " ".join(cmd))
    subprocess.run(cmd, check=True)


def exposed_surface(occ: np.ndarray, spacing: float, origin: float) -> tuple:
    """Boundary surface of the occupied voxel set (no internal faces).

    occ is a boolean array indexed [ix, iy, iz]. For each occupied voxel we emit a
    quad only on the faces whose neighbour is empty (or outside) — the exposed
    surface — so the translucent mesh has a single clean skin instead of the
    double-counted internal walls `as_boxes()` would produce. Returns (vertices
    Nx3 float, faces Mx3 int) with vertices at voxel-corner coordinates in the
    arm-base frame."""
    nx, ny, nz = occ.shape
    h = spacing / 2.0

    def centers(mask: np.ndarray) -> np.ndarray:
        ix, iy, iz = np.nonzero(mask)
        return np.stack([origin + ix * spacing, origin + iy * spacing,
                         origin + iz * spacing], axis=1)

    # 6 face directions: (axis, sign, the 4 corner offsets in ± half-steps)
    faces_def = [
        ((+1, 0, 0), [(+1, -1, -1), (+1, +1, -1), (+1, +1, +1), (+1, -1, +1)]),
        ((-1, 0, 0), [(-1, -1, -1), (-1, -1, +1), (-1, +1, +1), (-1, +1, -1)]),
        ((0, +1, 0), [(-1, +1, -1), (-1, +1, +1), (+1, +1, +1), (+1, +1, -1)]),
        ((0, -1, 0), [(-1, -1, -1), (+1, -1, -1), (+1, -1, +1), (-1, -1, +1)]),
        ((0, 0, +1), [(-1, -1, +1), (+1, -1, +1), (+1, +1, +1), (-1, +1, +1)]),
        ((0, 0, -1), [(-1, -1, -1), (-1, +1, -1), (+1, +1, -1), (+1, -1, -1)]),
    ]
    dims = (nx, ny, nz)
    verts: list = []
    faces: list = []
    for (dx, dy, dz), corners in faces_def:
        # neighbour[p] = occ[p + (dx,dy,dz)]; a face is exposed where the cell is
        # occupied but the neighbour in that direction is empty (or off-grid).
        d = (dx, dy, dz)
        dst = tuple(slice(max(0, -d[a]), dims[a] - max(0, d[a])) for a in range(3))
        src = tuple(slice(max(0, d[a]), dims[a] - max(0, -d[a])) for a in range(3))
        neighbour = np.zeros_like(occ)
        neighbour[dst] = occ[src]
        exposed = occ & ~neighbour
        c = centers(exposed)
        if len(c) == 0:
            continue
        base = sum(len(v) for v in verts)
        quad = np.stack([c + np.array([ox * h, oy * h, oz * h])
                         for (ox, oy, oz) in corners], axis=1)  # (k,4,3)
        k = quad.shape[0]
        verts.append(quad.reshape(-1, 3))
        vi = base + np.arange(k * 4).reshape(k, 4)
        faces.append(np.stack([vi[:, 0], vi[:, 1], vi[:, 2]], axis=1))
        faces.append(np.stack([vi[:, 0], vi[:, 2], vi[:, 3]], axis=1))
    if not verts:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int32)
    V = np.concatenate(verts, axis=0).astype(np.float64)
    F = np.concatenate(faces, axis=0).astype(np.int64)
    return V, F


def build_mesh(grid: dict, smooth_iters: int) -> tuple:
    dims = grid["dims"]
    occ_flat = np.asarray(grid["occupied"], dtype=np.uint8)
    # flat layout is x-fastest then y then z -> reshape [z,y,x] then to [x,y,z]
    occ = occ_flat.reshape(dims[2], dims[1], dims[0]).transpose(2, 1, 0).astype(bool)
    spacing = float(grid["spacing_m"])
    origin = float(grid["origin_m"][0])

    V, F = exposed_surface(occ, spacing, origin)
    if len(V) == 0:
        return V.astype(np.float32), F.astype(np.int32), int(occ.sum())

    import trimesh

    mesh = trimesh.Trimesh(vertices=V, faces=F, process=True)
    mesh.merge_vertices()
    try:
        mesh.fix_normals()  # consistent winding (the region is not star-shaped)
    except Exception:
        pass
    if smooth_iters > 0 and len(mesh.faces) > 0:
        # Humphrey smoothing rounds the blocky boundary into an isosurface while
        # resisting the shrinkage plain Laplacian causes.
        try:
            trimesh.smoothing.filter_humphrey(mesh, iterations=smooth_iters)
        except Exception:
            trimesh.smoothing.filter_laplacian(mesh, iterations=smooth_iters)
    return (mesh.vertices.astype(np.float32), mesh.faces.astype(np.int32),
            int(occ.sum()))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid", type=Path, default=None,
                    help="existing ik_feasibility_grid JSON to reuse (skip the C++ run)")
    ap.add_argument("--exe", type=Path, default=_default_grid_exe(),
                    help="path to the built ik_feasibility_grid tool")
    ap.add_argument("--out", type=Path, default=_default_out())
    ap.add_argument("--spacing-m", type=float, default=0.05)
    ap.add_argument("--orientations", type=int, default=18)
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--smooth-iters", type=int, default=8,
                    help="Humphrey smoothing passes (0 = blocky/no smoothing)")
    args = ap.parse_args()

    if args.grid is not None:
        grid_path = args.grid
    else:
        tmp = Path(tempfile.gettempdir()) / "ik_feasibility_grid.json"
        run_grid_tool(args.exe, tmp, args.spacing_m, args.orientations,
                      args.seeds, args.threads)
        grid_path = tmp

    grid = json.loads(Path(grid_path).read_text())
    verts, faces, occ_count = build_mesh(grid, args.smooth_iters)

    if len(verts) == 0:
        print("WARNING: no IK-infeasible cells found — nothing to render. "
              "Lower spacing/orientations or check the grid.")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.out,
        shell_vertices_base_m=verts,
        shell_faces=faces,
        r_min_m=float(grid["r_min_m"]),
        r_max_m=float(grid["r_max_m"]),
        spacing_m=float(grid["spacing_m"]),
        orientations=int(grid["orientations"]),
        seeds=int(grid["seeds"]),
        occupied_cells=occ_count,
    )
    sidecar = args.out.with_suffix(".json")
    sidecar.write_text(json.dumps({
        "r_min_m": float(grid["r_min_m"]),
        "r_max_m": float(grid["r_max_m"]),
        "spacing_m": float(grid["spacing_m"]),
        "orientations": int(grid["orientations"]),
        "seeds": int(grid["seeds"]),
        "occupied_cells": occ_count,
        "vertices": int(len(verts)),
        "faces": int(len(faces)),
        "ik": grid.get("ik", {}),
    }, indent=2))

    print(f"RB3-730E IK-infeasible region (base frame)")
    print(f"  grid            : spacing={grid['spacing_m']} m, r_max={grid['r_max_m']} m, "
          f"N_orient={grid['orientations']}, seeds={grid['seeds']}")
    print(f"  occupied cells  : {occ_count}")
    print(f"  mesh            : {len(verts)} verts, {len(faces)} faces "
          f"(smooth {args.smooth_iters})")
    print(f"  saved           : {args.out}")
    print(f"                    {sidecar}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
