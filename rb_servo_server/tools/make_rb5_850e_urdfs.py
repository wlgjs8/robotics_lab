#!/usr/bin/env python3
"""Derive dual_rb5_850e_ver3.urdf (our self-collision model) from upstream ver2.

WHY A GENERATOR AND NOT A HAND-EDITED FILE
==========================================
The unified URDF is the geometry the async CollisionMonitor enforces, so the
property that must never break silently is that ver3's KINEMATICS are bit-identical
to ver2's: the joints, the links and their placements are upstream's, and nothing is
moved. A 44 KB hand edit cannot demonstrate that; a transform plus an FK regression
can, and this script is re-runnable when upstream ships a new ver. The regression is:
over 500 random configurations every shared frame deviates by 0.

WHAT IT CHANGES, AND WHY
========================
1. Convex collision shells. Upstream ver2 points link0/1/4/5/6 at raw *.stl that
   are NOT convex (measured mesh/hull volume ratio: link0 0.634, link6 0.917,
   link1/4/5 ~0.961). collision_monitor.cpp:672 tests convexity and keeps a
   non-convex mesh as a BVH -- distances stay CORRECT, but the per-eval servo
   reaction budget documented there only holds when non_convex_mesh_count_ == 0,
   and RB5 would have shipped 10 such geoms (5 links x 2 arms). We swap in
   precomputed hulls (link2/link3 keep upstream's CoACD sets).

2. Elbow bound +/-165 deg, not upstream's +/-179.9. The catalog value for RB5-850
   is +/-165 (Rainbow RB Series catalog p7). Shipping a wider URDF bound recreates
   exactly the trap docs/joint_range_policy.md records for RB3: JointTarget and
   InitMotion bypass IK and clear only the safety clamp, so they can park the elbow
   in the band the URDF allows but the controller refuses, and every subsequent
   Cartesian tick is then rejected. safety.q_min_deg/q_max_deg must agree.

3. A stand_collision link carrying CoACD hulls of the real stand, added ALONGSIDE
   upstream's primitive boxes (dual_rb3_730e_ver5 does the same). The boxes alone
   cover only 77.0% of the true stand surface with a 36.9 mm max gap (60k surface
   samples) -- i.e. a fifth of the stand is invisible to the guard. The 20 hulls
   cover 100.0%.

4. Visual elements are dropped. The sole consumer is
   buildGeom(model, urdf, pinocchio::COLLISION, ...), so visuals are never read,
   and dropping them makes this file self-contained inside robotics_lab instead of
   depending on mesh paths in the sibling mo_robot_descriptions checkout.

Usage:
  rb_servo_server/tools/make_rb5_850e_urdfs.py [--check]
  --check re-derives into a temp file and diffs, so CI/reviewers can prove the
  committed URDF is what this script produces.
"""
from __future__ import annotations

import argparse
import math
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
UPSTREAM = (REPO.parent / "mo_robot_descriptions/mo_robot_descriptions/robots/urdf"
            "/dual_rb5_850e/dual_rb5_850e_ver2.urdf")
OUT = REPO / "rb_servo_server/descriptions/urdf/dual_rb5_850e_ver3.urdf"
UPSTREAM_SINGLE = (REPO.parent / "mo_robot_descriptions/mo_robot_descriptions/robots/urdf"
                   "/rb5_850e/rb5_850e.urdf")
OUT_SINGLE = REPO / "rb_servo_server/descriptions/urdf/rb5_850e.urdf"
OUT_DISPLAY = REPO / "rb_servo_server/descriptions/urdf/rb5_850e_pika_articulated.urdf"

# Rainbow RB Series catalog p7: RB5-850 J3 working range +/-165 deg.
ELBOW_LIMIT_DEG = 165.0

# Tool chain below attachment_site, mirroring rb3_730e.urdf. The pika adapter is
# unchanged across the RB3->RB5 swap (operator, 2026-09-02: same adapter, bolted
# straight to the RB5 flange), so the offsets carry over.
#
# MEASURED ON THE ROBOT, 2026-09-02, not inherited. RB3's value was 247.642 mm -- the
# pika_gripper.STL tip plane measured from RB3's attachment_site -- and the hardware
# disagrees with it by about 15 mm. Where that 15 mm actually lives is worked out at
# the end of this comment; the measurement itself is below.
#
# Two hand-parked contact poses, read back with tools/rbpodo_read_state:
#   A  both TCPs touching the stand   tool axes [-0.77,-0.58,-0.27] / [-0.87,0.37,-0.34]
#   B  the two TCPs touching EACH OTHER (stand geometry not involved at all)
#                                     tool axes [ 0.14,-0.98,-0.13] / [-0.06,0.99, 0.10]
# At 247.642 mm, pose A left both tips 14.97 / 14.00 mm clear of the surface and pose
# B left the two tips 29.88 mm apart -- 14.94 mm per arm. The agreement is what
# identifies the cause: the two poses' tool axes are nearly ORTHOGONAL, and a mount
# position error cannot show up as the same along-the-tool-axis shortfall in both,
# whereas a tool-length error does so by construction. A combined fit over all three
# contact residuals gives 262.87 mm, and at that value pose A's clearances fall to
# 3.27 / 0.38 mm. The stand model, which the first pose alone had implicated, is fine.
#
# Residuals of a few mm are the precision of parking an arm against a surface by
# hand, so treat the 262.87 mm as +/- ~3 mm. Re-derive it with the two poses above
# if the gripper or its adapter is ever changed.
#
# WHERE THE 15 mm IS -- corrected twice. The first attempt blamed attachment_site and
# pushed it 15.23 mm out; rb_gui showed the gripper floating off the flange, and the
# meshes agree it was wrong: link6's visual ends 96.57 mm from its origin and upstream
# puts attachment_site at 96.70 mm, so upstream's attachment_site IS the flange face --
# and RB3 is the same convention, its link6 ending at exactly 100.00 mm. The second
# attempt guessed the physical fingers were longer than pika_gripper.STL; the operator
# says they are the CAD fingers. The actual answer is below, and it needed no guess.
FT_BASE_OFFSET_M = 0.015          # attachment_site -> ft_sensor_base
FT_MEASUREMENT_OFFSET_M = 0.030   # ft_sensor_base  -> ft_sensor_measurement
# THE 15 mm, RESOLVED. It is the F/T sensor's flange adapter, and both terms below
# have provenance -- this is no longer a fitted number:
#
#   flange -> gripper CAD origin = 45.0 - 30.0 = 15.000 mm
#     45.0 mm is offset.xyz in the controller-manager RFT64-6A01-A preset (flange ->
#     sensing reference origin); 30.0 mm is the sensor body. NOTE that preset's own
#     comment says "adapter 13mm + sensor 30mm = 43mm", disagreeing with its VALUE by
#     2 mm. The hardware settles it: 13 mm would put the tip at 260.642 mm, which the
#     contact measurement misses by 2.23 mm, outside its ~2 mm spread. 15 mm is right,
#     and it is what robotics_lab's own FT chain has always carried (FT_BASE_OFFSET_M).
#   gripper CAD origin -> tip    = 247.642 mm (pika_gripper.STL z-max)
#
# Sum 262.642 mm against the measured 262.870 mm: they agree to 0.228 mm.
TOOL_STANDOFF_M = 0.015           # flange -> gripper mounting face (the F/T adapter)
# The adapter is REAL HARDWARE that nothing modelled: link6's hull stops at the flange
# and the gripper hulls start at the gripper origin, so [0, 15] mm was a hole in the
# collision model. Radius 35 mm = the gripper base's own mounting-face radius, which
# bounds it from above (an adapter wider than the part it carries would be unusual) and
# sits entirely inside link6's 82 x 87 mm silhouette -- so it cannot raise a false
# positive that link6 does not already raise.
TOOL_ADAPTER_RADIUS_M = 0.035
PIKA_TIP_OFFSET_M = TOOL_STANDOFF_M + 0.247642   # attachment_site (flange) -> tcp

ARM_COLLISION = {                      # link -> our collision mesh(es), URDF-relative
    "link0": ["../meshes/robots/rb5_850e/collision/link0_hull.stl"],
    "link1": ["../meshes/robots/rb5_850e/collision/link1_hull.stl"],
    "link2": [f"../meshes/robots/rb5_850e/collision/link2/link2_hull_{i:03d}.stl" for i in range(3)],
    "link3": [f"../meshes/robots/rb5_850e/collision/link3/link3_hull_{i:03d}.stl" for i in range(3)],
    "link4": ["../meshes/robots/rb5_850e/collision/link4_hull.stl"],
    "link5": ["../meshes/robots/rb5_850e/collision/link5_hull.stl"],
    "link6": ["../meshes/robots/rb5_850e/collision/link6_hull.stl"],
}
# Stand display mesh, vendored here so rb_gui needs nothing outside this repo. Origin
# is upstream ver2's stand visual origin verbatim -- identity, unlike RB3 ver5 whose
# stand visual carried rpy [0,0,-1.5708] to undo its own base->stand +90 deg joint.
STAND_VISUAL_MESH = "../meshes/stands/dual_rb5_850e/dual_rb5_850e_stand_ver2.stl"

# Display meshes for rb5_850e_pika_articulated.urdf (rb_gui only; the C++ FK/IK never
# reads it). Vendored .dae rather than upstream's per-material .obj: the two are the
# same object -- extents agree to 0.0002 mm -- and the .dae deviates from the finer
# .obj surface by p50 0.0000 mm, p99 <= 0.063 mm, max 0.271 mm, an order below the
# 1-2 mm the operator needs to judge on screen, for 26 MB instead of 69 MB.
ARM_VISUAL = {f"link{i}": f"../meshes/robots/rb5_850e/visual/link{i}.dae" for i in range(7)}
TOOL_VISUAL = {
    "tool": "../meshes/robots/rb5_850e/visual/tool/pika_gripper_base.STL",
    "finger_left": "../meshes/robots/rb5_850e/visual/tool/pika_finger_left.STL",
    "finger_right": "../meshes/robots/rb5_850e/visual/tool/pika_finger_right.STL",
}
FINGER_TRAVEL_M = 0.05
STAND_VISUAL_XYZ = "0.0 0.0 0.0"
STAND_VISUAL_RPY = "0.0 0.0 0.0"
STAND_HULLS = [f"../meshes/stands/dual_rb5_850e/collision_ver2/stand_hull_{i:03d}.stl"
               for i in range(20)]



def add_tool_standoff(root, prefix: str = "") -> None:
    """Insert the F/T adapter between attachment_site and where the gripper mounts.

    Adds `<prefix>tool_adapter` (a collision cylinder spanning the standoff) and
    `<prefix>gripper_mount`, the frame the gripper's own geometry hangs off. The
    collision monitor attaches the pika hulls by frame NAME, so pointing it at
    gripper_mount instead of attachment_site is what moves them the 15 mm out --
    see safety.self_collision.mesh.gripper_attach_frame.
    """
    adapter = ET.SubElement(root, "link", {"name": f"{prefix}tool_adapter"})
    col = ET.SubElement(adapter, "collision")
    # URDF cylinders are drawn along local Z, and attachment_site's +Z IS the tool
    # axis, so the cylinder needs no rotation -- only centring on the standoff.
    ET.SubElement(col, "origin",
                  {"xyz": f"0.0 0.0 {TOOL_STANDOFF_M / 2.0}", "rpy": "0.0 0.0 0.0"})
    geo = ET.SubElement(col, "geometry")
    ET.SubElement(geo, "cylinder",
                  {"radius": f"{TOOL_ADAPTER_RADIUS_M}", "length": f"{TOOL_STANDOFF_M}"})
    joint = ET.SubElement(root, "joint",
                          {"name": f"{prefix}tool_adapter_joint", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": f"{prefix}attachment_site"})
    ET.SubElement(joint, "child", {"link": f"{prefix}tool_adapter"})
    ET.SubElement(joint, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})

    ET.SubElement(root, "link", {"name": f"{prefix}gripper_mount"})
    joint = ET.SubElement(root, "joint",
                          {"name": f"{prefix}gripper_mount_joint", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": f"{prefix}attachment_site"})
    ET.SubElement(joint, "child", {"link": f"{prefix}gripper_mount"})
    ET.SubElement(joint, "origin",
                  {"xyz": f"0.0 0.0 {TOOL_STANDOFF_M}", "rpy": "0.0 0.0 0.0"})


def _mesh_collision(path: str, scale: str) -> ET.Element:
    col = ET.Element("collision")
    ET.SubElement(col, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})
    geo = ET.SubElement(col, "geometry")
    ET.SubElement(geo, "mesh", {"filename": path, "scale": scale})
    return col


def build(src: Path) -> ET.ElementTree:
    tree = ET.parse(src)
    root = tree.getroot()

    # (4) ARM visuals are never read by buildGeom(..., COLLISION, ...); dropping them
    # keeps this file resolvable inside robotics_lab (upstream's RB5 visual meshes are
    # 67 MB of .obj). The STAND visual is the exception and is re-added below: rb_gui
    # renders the stand from it, and hardcoding that pose in the GUI instead is exactly
    # how the 90 deg stand misalignment happened on the RB3 -> RB5 swap (the GUI
    # carried RB3's stand visual origin, rpy [0,0,-1.5708], as a constant).
    for link in root.findall("link"):
        for vis in link.findall("visual"):
            link.remove(vis)
    stand_link = root.find("link[@name='stand']")
    if stand_link is None:
        raise SystemExit("dual: no stand link to attach the display mesh to")
    vis = ET.SubElement(stand_link, "visual")
    ET.SubElement(vis, "origin", {"xyz": STAND_VISUAL_XYZ, "rpy": STAND_VISUAL_RPY})
    geo = ET.SubElement(vis, "geometry")
    ET.SubElement(geo, "mesh", {"filename": STAND_VISUAL_MESH, "scale": "0.001 0.001 0.001"})

    # (1) arm collision shells -> precomputed convex hulls
    replaced = 0
    for link in root.findall("link"):
        name = link.get("name", "")
        suffix = name.rsplit("_", 1)[-1]
        if not (("_left_" in name or "_right_" in name) and suffix in ARM_COLLISION):
            continue
        for col in link.findall("collision"):
            link.remove(col)
        for path in ARM_COLLISION[suffix]:
            link.append(_mesh_collision(path, "1.0 1.0 1.0"))
            replaced += 1

    # (2) elbow bound
    lo, hi = -math.radians(ELBOW_LIMIT_DEG), math.radians(ELBOW_LIMIT_DEG)
    elbows = 0
    for joint in root.findall("joint"):
        if not joint.get("name", "").endswith("elbow_joint"):
            continue
        limit = joint.find("limit")
        if limit is None:
            raise SystemExit(f"{joint.get('name')} has no <limit>")
        limit.set("lower", f"{lo:.6f}")
        limit.set("upper", f"{hi:.6f}")
        elbows += 1

    # (3) stand hulls alongside the upstream boxes
    for prefix in ("dual_rb5_850e_left_", "dual_rb5_850e_right_"):
        add_tool_standoff(root, prefix)

    if root.find("link[@name='stand_collision']") is not None:
        raise SystemExit("upstream already defines stand_collision; rework this step")
    stand = ET.SubElement(root, "link", {"name": "stand_collision"})
    for path in STAND_HULLS:
        stand.append(_mesh_collision(path, "0.001 0.001 0.001"))
    joint = ET.SubElement(root, "joint",
                          {"name": "stand_collision_fixed", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": "stand"})
    ET.SubElement(joint, "child", {"link": "stand_collision"})
    ET.SubElement(joint, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})

    if replaced != 2 * sum(len(v) for v in ARM_COLLISION.values()):
        raise SystemExit(f"expected both arms' collision shells, replaced {replaced}")
    if elbows != 2:
        raise SystemExit(f"expected 2 elbow joints, found {elbows}")

    ET.indent(tree, space="  ")
    return tree


def _fixed_joint(name: str, parent: str, child: str, xyz: str, rpy: str) -> ET.Element:
    j = ET.Element("joint", {"name": name, "type": "fixed"})
    ET.SubElement(j, "parent", {"link": parent})
    ET.SubElement(j, "child", {"link": child})
    ET.SubElement(j, "origin", {"xyz": xyz, "rpy": rpy})
    return j


def build_single(src: Path) -> ET.ElementTree:
    """Single-arm model for kinematics.urdf (C++ Pinocchio FK/IK).

    That loader calls buildModel() only -- no geometry -- and needs exactly
    base_link "world", flange_link "attachment_site", tip_link "tcp" and the six
    named joints. Upstream supplies all of those, but puts tcp AT attachment_site
    (no tool), so tcp is re-parented onto the pika tip the way rb3_730e.urdf does,
    and the F/T chain rb3_730e.urdf carries is reproduced so the URDF still
    documents the physical stack the force-control config describes.

    Visuals are dropped: unused by the C++ loader, and vendoring them would mean
    92 MB of RB5 meshes in git. rb_gui therefore has no RB5 display model yet.
    """
    tree = ET.parse(src)
    root = tree.getroot()

    for link in root.findall("link"):
        for vis in link.findall("visual"):
            link.remove(vis)

    replaced = 0
    for link in root.findall("link"):
        name = link.get("name", "")
        if name not in ARM_COLLISION:
            continue
        for col in link.findall("collision"):
            link.remove(col)
        for path in ARM_COLLISION[name]:
            link.append(_mesh_collision(path, "1.0 1.0 1.0"))
            replaced += 1
    if replaced != sum(len(v) for v in ARM_COLLISION.values()):
        raise SystemExit(f"single-arm: replaced {replaced} collision shells")

    lo, hi = -math.radians(ELBOW_LIMIT_DEG), math.radians(ELBOW_LIMIT_DEG)
    elbow = root.find("joint[@name='elbow_joint']/limit")
    if elbow is None:
        raise SystemExit("single-arm: elbow_joint has no <limit>")
    elbow.set("lower", f"{lo:.6f}")
    elbow.set("upper", f"{hi:.6f}")

    # Upstream's tcp sits on top of attachment_site; ours is the pika tip.
    tcp_joint = root.find("joint[@name='tcp_joint']")
    if tcp_joint is None:
        raise SystemExit("single-arm: no tcp_joint")
    tcp_joint.find("parent").set("link", "attachment_site")
    tcp_joint.find("origin").set("xyz", f"0.0 0.0 {PIKA_TIP_OFFSET_M}")
    tcp_joint.find("origin").set("rpy", "0.0 0.0 0.0")

    add_tool_standoff(root)

    for name in ("ft_sensor_base", "ft_sensor_measurement", "tool"):
        if root.find(f"link[@name='{name}']") is not None:
            raise SystemExit(f"single-arm: upstream already defines {name}")
        ET.SubElement(root, "link", {"name": name})
    root.append(_fixed_joint("ft_sensor_base_joint", "attachment_site", "ft_sensor_base",
                             f"0.0 0.0 {FT_BASE_OFFSET_M}", f"0.0 0.0 {math.pi / 2:.16f}"))
    root.append(_fixed_joint("ft_sensor_measurement_joint", "ft_sensor_base",
                             "ft_sensor_measurement", f"0.0 0.0 {FT_MEASUREMENT_OFFSET_M}",
                             "0.0 0.0 0.0"))
    # `tool` carries the gripper body in the display model, so it hangs off the mount.
    root.append(_fixed_joint("tool_joint", "gripper_mount", "tool", "0.0 0.0 0.0",
                             "0.0 0.0 0.0"))

    ET.indent(tree, space="  ")
    return tree


def build_display(src: Path) -> ET.ElementTree:
    """rb_gui's display model: the IK arm plus visuals and an articulated gripper.

    Mirrors rb3_730e_pika_articulated.urdf. The C++ FK/IK must NEVER load this -- the
    two prismatic finger joints would change its DOF -- which is why it is a separate
    file from rb5_850e.urdf and why kinematics.urdf points at that one.

    Accuracy matters here: the operator judges 1-2 mm clearances off this render, so
    the arm carries the vendored .dae surfaces (max 0.271 mm from upstream's finer
    .obj) rather than the collision hulls, which are inflated 3.9-57.8% by convexity
    and would misstate exactly the gap being judged.
    """
    tree = build_single(src)
    root = tree.getroot()

    for link in root.findall("link"):
        mesh = ARM_VISUAL.get(link.get("name", ""))
        if mesh is None:
            continue
        vis = ET.SubElement(link, "visual")
        ET.SubElement(vis, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})
        geo = ET.SubElement(vis, "geometry")
        ET.SubElement(geo, "mesh", {"filename": mesh, "scale": "1.0 1.0 1.0"})

    # build_single() already created an empty `tool` link on attachment_site; give it
    # the gripper body, and add the two fingers as prismatic siblings so the viewer can
    # animate the live jaw percent.
    for name, mesh in TOOL_VISUAL.items():
        link = root.find(f"link[@name='{name}']")
        if link is None:
            link = ET.SubElement(root, "link", {"name": name})
        vis = ET.SubElement(link, "visual")
        # Identity: these links hang off gripper_mount, which already carries the
        # standoff. Putting it here as well would apply it twice.
        ET.SubElement(vis, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})
        geo = ET.SubElement(vis, "geometry")
        ET.SubElement(geo, "mesh", {"filename": mesh, "scale": "0.001 0.001 0.001"})
    for name, lower, upper in (("finger_left", 0.0, FINGER_TRAVEL_M),
                               ("finger_right", -FINGER_TRAVEL_M, 0.0)):
        joint = ET.SubElement(root, "joint",
                              {"name": f"{name}_joint", "type": "prismatic"})
        ET.SubElement(joint, "parent", {"link": "gripper_mount"})
        ET.SubElement(joint, "child", {"link": name})
        ET.SubElement(joint, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})
        ET.SubElement(joint, "axis", {"xyz": "1.0 0.0 0.0"})
        ET.SubElement(joint, "limit", {"lower": f"{lower}", "upper": f"{upper}",
                                       "effort": "20.0", "velocity": "0.5"})
    ET.indent(tree, space="  ")
    return tree


def write(tree: ET.ElementTree, dst: Path, source: str) -> None:
    header = (
        "<?xml version=\"1.0\"?>\n"
        "<!-- GENERATED by rb_servo_server/tools/make_rb5_850e_urdfs.py from\n"
        f"     mo_robot_descriptions {source}. DO NOT HAND-EDIT: rerun the\n"
        "     generator, which carries the rationale and the invariants it enforces.\n"
        f"     Elbow bound is the RB5-850 catalog value +/-{ELBOW_LIMIT_DEG:.0f} deg and must stay\n"
        "     equal to safety.q_min_deg/q_max_deg[2] in the stack config. -->\n"
    )
    body = ET.tostring(tree.getroot(), encoding="unicode")
    dst.write_text(header + body + "\n")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="verify the committed URDFs match what this script produces")
    args = ap.parse_args(argv)

    targets = [(UPSTREAM, OUT, build, "dual_rb5_850e_ver2.urdf"),
               (UPSTREAM_SINGLE, OUT_SINGLE, build_single, "rb5_850e.urdf"),
               (UPSTREAM_SINGLE, OUT_DISPLAY, build_display, "rb5_850e.urdf")]
    for src, _dst, _fn, _label in targets:
        if not src.exists():
            print(f"upstream URDF not found: {src}", file=sys.stderr)
            return 1

    rc = 0
    for src, dst, fn, label in targets:
        tree = fn(src)
        if args.check:
            with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as fh:
                tmp = Path(fh.name)
            write(tree, tmp, label)
            same = dst.exists() and dst.read_text() == tmp.read_text()
            tmp.unlink()
            print(f"{'MATCH' if same else 'DRIFT'}  {dst.relative_to(REPO)}")
            rc |= 0 if same else 1
        else:
            write(tree, dst, label)
            print(f"wrote {dst.relative_to(REPO)}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
