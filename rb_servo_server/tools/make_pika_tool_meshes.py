#!/usr/bin/env python3
"""Build the pika tool display/collision meshes from the v15 PLA+TPU tip CAD.

WHY A GENERATOR AND NOT HAND-PLACED STLs
========================================
The eight meshes under descriptions/meshes/robots/rb5_850e/visual/tool/ used to be an
undated one-shot export: every file carried the same 2026-08-19 04:29 mtime, copied in
from the RB3 tree, with nothing on disk saying which CAD they came from or what
transform put them in the URDF's tool frame. That is exactly the asset that goes
silently stale when the hardware changes -- and it did, when the printed PLA+TPU tip
replaced the factory finger. This script makes the placement auditable: the transforms
are named constants with their provenance, and --check proves the committed meshes are
what it produces.

WHAT CHANGED IN THE HARDWARE (2026-09-04)
=========================================
The factory pika finger (one solid part, `gripper_finger_L/R` in the vendor Gripper
STEP) was replaced by a printed two-material tip:

  PLA_spine_v15.STL   the structural spine + mounting foot   (black PLA)
  TPU_blade_v15.stl   the compliant contact blade            (95A TPU, #F3E600)

`TPU_blade_v15.stl` is byte-identical to v14; v15 changed only the spine, thickening
the foot (Z -42..-34) and the lower spine (Z -26..-14) by ~20% of material. The tip
region (Z > +10) and every mounting datum are unchanged from v14.

THE 1.99 mm SEATING SHIFT -- READ THIS BEFORE TOUCHING THE TRANSFORMS
=====================================================================
The printed spine's foot was derived from the pika SENSE arm, not from the Gripper
finger it replaces (analysis/redesign v10 rev4 restored the LM guide seat off
`sense_arm_L`). All three of its mounting datums therefore sit ~1.99 mm further out
than the factory finger's, measured in each part's own contact-face-anchored frame.
The M2 hole PATTERN is identical (13.00 x 12.00 at Y +/-6.00), so it bolts on with no
interference at all -- the part simply seats 1.99 mm further inboard.

The carriage is the datum, and it is IN THE CAD: pika_gripper_base.STL carries the
carriage top face at URDF Z 145.30 (= the finger seat plane), four M2 tapped holes
(d 1.97) at URDF |X| 82.02 / 95.03, Y +/-6.00, and the rail screw (d 2.04) at |X|
80.01, Y -/+14.55. Converted to the factory finger's tip frame those are X -35.004 /
-48.024 and rail -32.990 -- and the factory finger's own holes are at -35.005 /
-48.003, rail -33.000. Residual <= 0.021 mm. That agreement is what validates the
whole coordinate chain below.

The v15 spine's holes are at -37.001 / -49.989, rail -34.950. Three independent
datums, all needing the SAME shift:

    M2 column A   -35.005 - (-37.001)  =  +1.996 mm
    M2 column B   -48.003 - (-49.989)  =  +1.986 mm
    rail screw    -33.000 - (-34.950)  =  +1.950 mm

Spread 0.046 mm, i.e. below FDM tolerance. BOLT_DATUM_SHIFT_MM below is the mean of
the two M2 columns (the clearance-fit primary locators; the rail screw taps its own
thread in PLA and has less positional authority) and the rail agrees with it to
0.04 mm.

CONSEQUENCE, AND WHAT THE HARDWARE ACTUALLY SAID
------------------------------------------------
2026-09-04, vernier caliper across the two TPU tip faces: FULL-OPEN GAP ~98 mm, and
the jaws close to contact. Neither CAD-pose prediction (90.35 mm with the shift above,
94.33 without, 94.04 for the factory finger) is that number, and the reason is a premise
nobody had checked: THE VENDOR'S CAD DOES NOT DRAW THE FINGERS AT THE OPEN STOP. It
draws them 3.8 mm/side inboard of it. The old FINGER_TRAVEL 0.047 was never measured
either -- it was picked so that the CAD pose would close to zero gap, which quietly
assumed the same thing.

So the placement below is anchored on the MEASUREMENT, not on the CAD pose:
MEASURED_OPEN_GAP_MM sets where the meshes sit, and the carriage stroke follows from it
(gap 0 at the closed stop => stroke = gap/2 = 49.0 mm per side, not 47). rb_gui, the
collision monitor's finger travel and the URDF joint limit all use that 49.

BOLT_DATUM_SHIFT_MM is kept because the datum check still uses it and because it is real
CAD evidence, but note what one gap measurement CANNOT do: seating shift s and extra
opening e only ever appear as (e - s) = 1.83 mm, so s = 1.991 / e = 3.83 and s = 0 /
e = 1.83 both reproduce 98.0 mm exactly. Separating them needs a carriage-referenced
measurement, and NOTHING WE COMPUTE DEPENDS ON THE SPLIT -- the render, the monitor and
the grasp width all consume the measured mapping. Do not spend hardware time on it.

What "the gripper closes to 0" turned out to mean: with stroke 49 and open gap 98 the
tips meet exactly at 0 %, no preload. The earlier prediction of a 3.7 mm overlap was an
artefact of the 47 mm travel, i.e. of the same unchecked premise.

WHAT IS DELIBERATELY UNCHANGED
==============================
* PIKA_TIP_OFFSET_M / tcp_joint / force_control tool_xyz_mm. Carriage face -> tip is
  102.342 mm on the factory finger and 102.278 on v15: 0.064 mm in a 247.642 mm chain.
  Moving it would be noise dressed as a measurement.
* The finger travel (0.047 m, rb_gui _GRIPPER_FINGER_TRAVEL_M and the config's
  gripper_finger_travel_m). That number is the CARRIAGE's stroke, which the tip swap
  does not touch. Below ~3.7% the rendered rigid tips now interpenetrate -- that is an
  honest depiction of the preload, and it keeps the collision monitor's finger hulls
  where the carriage actually is.
* pika_gripper_base{,_hull}.STL. The tip swap is entirely below the carriage face.

Usage:
  rb_servo_server/tools/make_pika_tool_meshes.py [--check]
  --check rebuilds into memory and byte-compares, so CI/reviewers can prove the
  committed meshes are what this script produces.
"""
from __future__ import annotations

import argparse
import hashlib
import math
import sys
from pathlib import Path

import numpy as np
import trimesh

REPO = Path(__file__).resolve().parents[2]
OUT_DIR = REPO / "rb_servo_server/descriptions/meshes/robots/rb5_850e/visual/tool"

# Source CAD, sibling checkout -- same arrangement as make_rb5_850e_urdfs.py reading
# mo_robot_descriptions. The digests are recorded so a silently swapped input is a
# loud failure rather than a redrawn gripper.
SRC_DIR = REPO.parent / "pika_gripper_tip/analysis/redesign/v15"
SRC_PLA = SRC_DIR / "PLA_spine_v15.STL"
SRC_TPU = SRC_DIR / "TPU_blade_v15.stl"
SRC_MD5 = {
    "PLA_spine_v15.STL": "20c921f4c73af0e1075bd9ff250e55ba",
    "TPU_blade_v15.stl": "4c6c88933955e463827e0e10bc9e61b9",  # byte-identical to v14
}

# PLA_spine_v15.STL was exported with its bbox minimum at the origin, unlike v14 which
# was already in the tip frame. This puts it back: the offset IS v14's bbox minimum,
# and the two parts' extents agree to 0.004 mm, so it is a translation and nothing else.
V15_PLA_TO_TIPFRAME = (-58.990, -19.055, -44.190)

# Tip frame (X = depth, 0 at the contact face; Y = width; Z = length, 0 at the pad root)
# -> URDF tool frame, fitted against the committed factory-finger meshes at 0.096 mm
# p99.9 surface deviation. The right finger is the left ROTATED 180 deg about Z, not
# mirrored (analysis/stl/README.md, self-mirror IoU 0.932).
TIPFRAME_TO_URDF_LEFT_XYZ = (-47.0162, 0.0006, 179.7142)
TIPFRAME_TO_URDF_RIGHT_XYZ = (47.0138, 0.0006, 179.7142)

# CAD-derived seating (see "THE 1.99 mm SEATING SHIFT"). Used ONLY by the datum check
# below -- the output placement is anchored on the measurement instead.
BOLT_DATUM_SHIFT_MM = 1.991

# MEASURED 2026-09-04: vernier caliper across the two TPU tip faces at the full-open
# stop. This is what places the meshes, because the vendor CAD's finger pose is not the
# open stop (3.8 mm/side inboard of it). The jaws close to contact, so the carriage
# stroke is half of this and FINGER_TRAVEL/gripper_finger_travel_m follow from it.
# Re-measure and change this ONE number if the tip or the gripper changes; do not
# re-derive it from the CAD pose.
MEASURED_OPEN_GAP_MM = 98.00

# ---- Colouring the rest of the tool ---------------------------------------------------
# pika_gripper_base.STL arrives as ONE mesh, so rb_gui rendered the whole tool in a
# single flat grey. It is really a stack of three things along Z, and the operator wants
# them apart: the flange-side mount adapters, the F/T sensor, and the gripper itself.
#
# The split is BY CONNECTED COMPONENT, not by a cutting plane -- a plane through Z 45
# would slice the sensor's cable connector (it spans Z 18..27 but belongs to the sensor)
# and leave half-parts on either side. Each component is assigned by its area-weighted
# Z centroid, so every part stays whole and the three outputs are an exact partition of
# the original faces (asserted).
#
# The boundaries come off the CAD's own radial profile (docs/reference/
# pika_gripper_sections.png and the components' Z spans):
#   Z  0..15   flange-side adapter plates: two components, r <= 35
#   Z 15..45   the RFT64-6A01-A: a Ø64 body in a Ø70 base on a bolt circle, with the
#              cable connector bulging to r=60 at Z 20..26. Ends at 45 = sensor_offset_mm.
#   Z 45..63   the tool-side adapter: its lower component is the SAME PART as the one at
#              Z 5..15 (identical face count and volume), i.e. the pair that sandwiches
#              the sensor -- which is what makes "mount" two pieces rather than one.
#   Z 63..145  the gripper: housing, then the rail/carriage plates from Z 123.8 up.
# The carriage plates at the top are grouped with the gripper, so they render black
# along with the housing they are continuous with. Move BASE_GROUP_BOUNDS_MM if they
# should stay grey instead.
BASE_GROUP_BOUNDS_MM = ((0.0, "mount"), (15.0, "ft_sensor"), (45.0, "mount"), (63.0, "gripper"))
BASE_GROUP_OUTPUT = {
    "mount": "pika_gripper_mount.STL",
    "ft_sensor": "pika_gripper_ft_sensor.STL",
    "gripper": "pika_gripper_body.STL",
}

# Invariants asserted after building, all from pika_gripper_base.STL (the vendor CAD's
# own carriage), in the URDF tool frame. Tolerance covers section/tessellation noise.
CARRIAGE_SEAT_Z_MM = 145.30
CARRIAGE_M2_ABS_X_MM = (82.02, 95.03)
CARRIAGE_M2_ABS_Y_MM = 6.00
DATUM_TOL_MM = 0.06


def _md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def _load(path: Path) -> trimesh.Trimesh:
    mesh = trimesh.load(path, force="mesh")
    mesh.merge_vertices()
    return mesh


def _rz(deg: float) -> np.ndarray:
    return trimesh.transformations.rotation_matrix(math.radians(deg), [0, 0, 1])


def _open_pose_shift(tip_face_tipx: float) -> float:
    """Tip-frame X shift that puts the tip face at the MEASURED full-open position."""
    face_target = -MEASURED_OPEN_GAP_MM / 2.0          # URDF X of the LEFT contact face
    return face_target - TIPFRAME_TO_URDF_LEFT_XYZ[0] - tip_face_tipx


def _placed(mesh: trimesh.Trimesh, side: str, shift_mm: float) -> trimesh.Trimesh:
    """Move a tip-frame part into the URDF tool frame for `side`."""
    out = mesh.copy()
    out.apply_translation((shift_mm, 0.0, 0.0))
    if side == "right":
        out.apply_transform(_rz(180.0))
        out.apply_translation(TIPFRAME_TO_URDF_RIGHT_XYZ)
    else:
        out.apply_translation(TIPFRAME_TO_URDF_LEFT_XYZ)
    return out


def _hull(mesh: trimesh.Trimesh, name: str) -> trimesh.Trimesh:
    hull = mesh.convex_hull
    # coal's buildConvexRepresentation does NOT compute a hull and this coal build has
    # no qhull, so a non-convex mesh handed to the monitor becomes an invalid Convex
    # whose GJK distance is garbage. The *_hull.STL files must be convex on disk.
    if not hull.is_convex:
        raise SystemExit(f"{name}: convex_hull() did not come back convex")
    return hull


def _m2_hole_centres(mesh: trimesh.Trimesh, z_mm: float) -> list[tuple[float, float]]:
    """Centres of the M2 clearance holes on a Z section, in the mesh's own frame."""
    section = mesh.section(plane_origin=[0.0, 0.0, z_mm], plane_normal=[0.0, 0.0, 1.0])
    if section is None:
        raise SystemExit(f"datum check: no section at Z={z_mm}")
    planar, to_3d = section.to_2D()
    centres = []
    for polygon in planar.polygons_full:
        for ring in polygon.interiors:
            pts = np.asarray(ring.coords)
            centre = pts.mean(axis=0)
            radii = np.linalg.norm(pts - centre, axis=1)
            if 1.0 < radii.mean() < 1.4 and radii.std() < 0.3:
                p3 = to_3d @ np.array([centre[0], centre[1], 0.0, 1.0])
                centres.append((float(p3[0]), float(p3[1])))
    return sorted(centres)


def _seat_plane_z(mesh: trimesh.Trimesh) -> float:
    """Z of the finger's seat face: the largest horizontal face near the carriage top."""
    normals = mesh.face_normals
    centres = mesh.triangles[:, :, 2].mean(axis=1)
    flat = (np.abs(normals[:, 2]) > 0.99) & (np.abs(centres - CARRIAGE_SEAT_Z_MM) < 3.0)
    if not flat.any():
        raise SystemExit("datum check: no horizontal face near the carriage top face")
    areas, zs = mesh.area_faces[flat], centres[flat]
    buckets: dict[float, float] = {}
    for z, area in zip(zs, areas):
        key = round(float(z), 2)
        buckets[key] = buckets.get(key, 0.0) + float(area)
    return max(buckets.items(), key=lambda kv: kv[1])[0]


def _assert_mounts_on_carriage(finger: trimesh.Trimesh, side: str) -> None:
    """The placed finger's seat plane and M2 holes must land on the carriage's."""
    seat_z = _seat_plane_z(finger)
    if abs(seat_z - CARRIAGE_SEAT_Z_MM) > 0.1:
        raise SystemExit(f"{side}: seat plane at Z={seat_z:.3f}, carriage is at "
                         f"{CARRIAGE_SEAT_Z_MM}")
    centres = _m2_hole_centres(finger, CARRIAGE_SEAT_Z_MM + 4.6)
    if len(centres) != 4:
        raise SystemExit(f"{side}: found {len(centres)} M2 holes at the seat, expected 4")
    for x, y in centres:
        dx = min(abs(abs(x) - target) for target in CARRIAGE_M2_ABS_X_MM)
        dy = abs(abs(y) - CARRIAGE_M2_ABS_Y_MM)
        if dx > DATUM_TOL_MM or dy > DATUM_TOL_MM:
            raise SystemExit(
                f"{side}: M2 hole at ({x:.3f}, {y:.3f}) is {dx:.3f}/{dy:.3f} mm off the "
                f"carriage pattern |X| {CARRIAGE_M2_ABS_X_MM} Y +/-{CARRIAGE_M2_ABS_Y_MM}")


def _split_base_by_material(base: trimesh.Trimesh) -> dict[str, trimesh.Trimesh]:
    """Partition pika_gripper_base.STL into mount / ft_sensor / gripper submeshes."""
    components = trimesh.graph.connected_components(
        base.face_adjacency, nodes=np.arange(len(base.faces)))
    masks: dict[str, list[int]] = {name: [] for name in BASE_GROUP_OUTPUT}
    for faces in components:
        faces = np.asarray(faces)
        areas = base.area_faces[faces]
        z = base.triangles[faces][:, :, 2].mean(axis=1)
        centroid = float(np.average(z, weights=areas)) if areas.sum() > 0 else float(z.mean())
        group = BASE_GROUP_BOUNDS_MM[0][1]
        for lower, name in BASE_GROUP_BOUNDS_MM:
            if centroid >= lower:
                group = name
        masks[group].extend(faces.tolist())
    total = sum(len(v) for v in masks.values())
    if total != len(base.faces) or len(set(f for v in masks.values() for f in v)) != total:
        raise SystemExit(f"base split is not a partition: {total} faces vs {len(base.faces)}")
    out = {}
    for name, faces in masks.items():
        if not faces:
            raise SystemExit(f"base split: group {name} came out empty")
        out[name] = base.submesh([sorted(faces)], append=True)
    return out


def build() -> dict[str, bytes]:
    for path, name in ((SRC_PLA, SRC_PLA.name), (SRC_TPU, SRC_TPU.name)):
        if not path.exists():
            raise SystemExit(f"source CAD missing: {path}\n"
                             "Expected the pika_gripper_tip checkout beside robotics_lab.")
        digest = _md5(path)
        if digest != SRC_MD5[name]:
            raise SystemExit(f"{name}: md5 {digest}, expected {SRC_MD5[name]}. The tip CAD "
                             "changed -- re-measure the datums before updating the digest.")

    pla_tip = _load(SRC_PLA)
    pla_tip.apply_translation(V15_PLA_TO_TIPFRAME)
    tpu_tip = _load(SRC_TPU)

    out: dict[str, bytes] = {}
    fingers: dict[str, trimesh.Trimesh] = {}
    open_shift = _open_pose_shift(float(tpu_tip.bounds[1][0]))
    for side in ("left", "right"):
        pla = _placed(pla_tip, side, open_shift)
        tpu = _placed(tpu_tip, side, open_shift)
        # Split meshes exist so the display URDF can put a <material> on each: black PLA
        # spine, #F3E600 95A TPU blade. yourdfpy gives multiple visuals per link their own
        # scene node and colour (verified on yourdfpy 0.0.60 / viser 1.0.30).
        out[f"pika_finger_{side}_pla.STL"] = pla.export(file_type="stl")
        out[f"pika_finger_{side}_tpu.STL"] = tpu.export(file_type="stl")
        # The combined finger keeps the ORIGINAL filename: the collision monitor tests and
        # the capsule-fitting scripts load it by that name, and it is what the hull and the
        # assembled pika_gripper.STL are built from.
        finger = trimesh.util.concatenate([pla, tpu])
        fingers[side] = finger
        # The datum check belongs at the CAD BOLT POSE, not at the open pose: it asks
        # whether the part mates with the carriage, which is a question about the part,
        # not about where in its stroke the jaw happens to be parked.
        _assert_mounts_on_carriage(
            _placed(pla_tip, side, BOLT_DATUM_SHIFT_MM), side)
        out[f"pika_finger_{side}.STL"] = finger.export(file_type="stl")
        out[f"pika_finger_{side}_hull.STL"] = _hull(
            finger, f"pika_finger_{side}_hull").export(file_type="stl")

    # pika_gripper.STL is the static whole-tool mesh, and it is NOT in the same frame as
    # the base/finger meshes: it is Rz(-90 deg) of their assembly, which is why the
    # legacy single-hull attach in stack_real.yaml carries rpy [0, 0, +pi/2] to undo it.
    # Verified against the previous committed file at 0.0000 mm max deviation.
    base = _load(OUT_DIR / "pika_gripper_base.STL")
    for group, mesh in _split_base_by_material(base).items():
        lo, hi = mesh.bounds[0][2], mesh.bounds[1][2]
        print(f"  base/{group:<9} {len(mesh.faces):>6} faces  Z {lo:7.2f}..{hi:7.2f}")
        out[BASE_GROUP_OUTPUT[group]] = mesh.export(file_type="stl")

    whole = trimesh.util.concatenate([base, fingers["left"], fingers["right"]])
    whole.apply_transform(_rz(-90.0))
    out["pika_gripper.STL"] = whole.export(file_type="stl")
    out["pika_gripper_hull.STL"] = _hull(whole, "pika_gripper_hull").export(file_type="stl")

    left_face = fingers["left"].bounds[1][0]
    right_face = fingers["right"].bounds[0][0]
    print(f"  seat -> tip (Z)   : {fingers['left'].bounds[1][2] - CARRIAGE_SEAT_Z_MM:.3f} mm "
          f"(factory 102.342)")
    print(f"  flange -> tip (Z) : {fingers['left'].bounds[1][2]:.3f} mm "
          f"(tcp_joint stays 247.642)")
    gap = right_face - left_face
    if abs(gap - MEASURED_OPEN_GAP_MM) > 0.02:
        raise SystemExit(f"placement gives a {gap:.3f} mm open gap, "
                         f"measured is {MEASURED_OPEN_GAP_MM}")
    print(f"  full-open jaw gap : {gap:.2f} mm (MEASURED); stroke {gap / 2:.2f} mm/side, "
          f"tips meet at 0 %")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="verify the committed meshes match what this script produces")
    args = ap.parse_args(argv)

    built = build()
    failures = 0
    for name, data in sorted(built.items()):
        dst = OUT_DIR / name
        if args.check:
            current = dst.read_bytes() if dst.exists() else b""
            if current == data:
                print(f"MATCH  {dst.relative_to(REPO)}")
            else:
                print(f"DIFFER {dst.relative_to(REPO)} "
                      f"({len(current)} bytes on disk, {len(data)} rebuilt)")
                failures += 1
        else:
            dst.write_bytes(data)
            print(f"wrote  {dst.relative_to(REPO)}  ({len(data)} bytes)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
