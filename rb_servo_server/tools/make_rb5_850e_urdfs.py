#!/usr/bin/env python3
"""Derive dual_rb5_850e_ver3.urdf (our self-collision model) from upstream ver1.

VER1, NOT VER2 -- WHICH STAND IS ACTUALLY BOLTED TO THE FLOOR (2026-09-06)
=========================================================================
Upstream ships two RB5 stands and they are NOT interchangeable: ver2 mounts each arm
15.00 mm further out along the 45 deg plate than ver1
(0.17036, +/-0.19707, 0.57036) vs (0.16285534, +/-0.18646447, 0.56285534), same rpy,
which puts the two arms 21.2 mm further apart in Y. This repo derived from ver2 from
2026-09-02 (commit de96559) until this change, and that was wrong for our cell.

Measured, three ways:
  * TAPE. The stand's full Y width is a direct check that needs no robot: ver1
    predicts 507.3 mm, ver2 521.4 mm. The operator measured ver1. (Height above the
    base-plate top is the same kind of check: 669.4 vs 681.5 mm.)
  * CONTACT, far from the mount. Both arms were hand-parked touching the stand and
    the joints read off the boxes. The RIGHT arm touched the base column at z~0.11-
    0.21 m, far enough from its own mount for the 15 mm to show: the model's contact
    point sits 7.00 mm off the true stand surface under ver2 and 0.50 mm under ver1.
    (The LEFT arm touched just under its own mount, where arm and stand move
    together, so it cannot tell the two apart -- 0.27 vs 0.29 mm. That degeneracy is
    why the error survived this long.)
  * ARM-TO-ARM. A hand-built 15 mm tip-to-wrist fixture reads 43.9 mm under ver2 and
    22.9 mm under ver1.
The two stands' SURFACES are identical wherever those contacts landed, so this is a
mount-origin difference, not a shape one -- but they do differ elsewhere (up to
68.7 mm, mostly the |y| > 0.10 m wings), which is why the whole model is regenerated
from ver1 rather than the mount origins being patched.

WHY A GENERATOR AND NOT A HAND-EDITED FILE
==========================================
The unified URDF is the geometry the async CollisionMonitor enforces, so the
property that must never break silently is that ver3's KINEMATICS are bit-identical
to upstream's: the joints, the links and their placements are upstream's, and nothing
is moved. A 44 KB hand edit cannot demonstrate that; a transform plus an FK regression
can, and this script is re-runnable when upstream ships a new ver. The regression is:
over 500 random configurations every shared frame deviates by 0.

WHAT IT CHANGES, AND WHY
========================
1. Convex collision shells. Upstream points link0/1/4/5/6 at raw *.stl that
   are NOT convex (measured mesh/hull volume ratio: link0 0.634, link6 0.917,
   link1/4/5 ~0.961). collision_monitor.cpp:672 tests convexity and keeps a
   non-convex mesh as a BVH -- distances stay CORRECT, but the per-eval servo
   reaction budget documented there only holds when non_convex_mesh_count_ == 0,
   and RB5 would have shipped 10 such geoms (5 links x 2 arms). We swap in
   precomputed hulls (link2/link3 keep upstream's CoACD sets).

2. Elbow bound +/-165 deg, not upstream's +/-179.9 (ver1 and ver2 both ship
   +/-3.14 rad). The catalog value for RB5-850
   is +/-165 (Rainbow RB Series catalog p7). Shipping a wider URDF bound recreates
   exactly the trap docs/joint_range_policy.md records for RB3: JointTarget and
   InitMotion bypass IK and clear only the safety clamp, so they can park the elbow
   in the band the URDF allows but the controller refuses, and every subsequent
   Cartesian tick is then rejected. safety.q_min_deg/q_max_deg must agree.

3. A stand_collision link carrying CoACD hulls of the real stand, REPLACING the raw
   stand mesh ver1 ships as its only <collision>. That mesh is 38 k non-convex
   triangles: coal keeps a non-convex mesh as a BVH, so it would be correct but far
   outside the per-eval budget, exactly the problem (1) exists to avoid. (ver2 also
   carried primitive boxes -- stand_base_col / stand_body_upper / stand_body_shoulder
   -- which ver1 does not have; the hulls were added alongside them then and stand
   alone now.) Measured on 6 k surface samples with a proper half-space containment
   test: the 20 hulls cover 100.0% of the true surface with no gap, and over-
   approximate it by mean 1.41 mm / p95 8.76 / max 29.87. The ver2 set they replace
   was mean 4.59 / p95 14.76 / max 37.80, so the guard also got ~3x tighter.

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
            "/dual_rb5_850e/dual_rb5_850e_ver1.urdf")
OUT = REPO / "rb_servo_server/descriptions/urdf/dual_rb5_850e_ver3.urdf"
UPSTREAM_SINGLE = (REPO.parent / "mo_robot_descriptions/mo_robot_descriptions/robots/urdf"
                   "/rb5_850e/rb5_850e.urdf")
OUT_SINGLE = REPO / "rb_servo_server/descriptions/urdf/rb5_850e.urdf"
OUT_DISPLAY = REPO / "rb_servo_server/descriptions/urdf/rb5_850e_pika_articulated.urdf"

# Rainbow RB Series catalog p7: RB5-850 J3 working range +/-165 deg.
ELBOW_LIMIT_DEG = 165.0

# Tool chain below attachment_site, mirroring rb3_730e.urdf.
#
# attachment_site IS THE FLANGE FACE on both robots -- link6's visual mesh ends at
# 96.57 mm and upstream puts the frame at 96.70 mm (RB3: mesh ends at 100.00, frame at
# 100.00). And pika_gripper.STL is measured FROM THAT FACE: sectioning it (see
# docs/reference/pika_gripper_sections.png) shows its first ~45 mm is the RFT64 F/T
# SENSOR -- a Ø64 body inside a Ø70 base, on a bolt circle, with the cable connector
# bulging to r=60 at z=20..25, exactly as the controller-manager RFT64-6A01-A part
# describes it. The gripper body starts around z=50 and the fingertip plane is at
# 247.642 mm.
#
# So flange -> tip = 247.642 mm, and the force-control config agrees with it by a
# different route: sensor_offset_mm 45 (flange -> sensing origin) + tool_xyz_mm 202.642
# (sensing origin -> tip) = 247.642.
#
# A CONTACT MEASUREMENT ON 2026-09-02 SAID 262.87 mm AND WAS WRONG, by 15 mm. Two
# hand-parked poses -- both TCPs on the stand, then the two TCPs touching each other --
# each came out ~15 mm long, and the agreement between them was read as confirmation.
# It was not: the two poses' tool axes are 60-67 deg apart, not the "nearly orthogonal"
# claimed at the time, and nothing verified that the FINGERTIP PLANE was the part
# making contact. Anything on the gripper touching first biases the fit long, the same
# way in both. Sectioning the STL is what settled it, and it needed no robot at all.
# Re-derive from the CAD, not from contact, if the tool changes.
FT_BASE_OFFSET_M = 0.015          # attachment_site -> ft_sensor_base
FT_MEASUREMENT_OFFSET_M = 0.030   # ft_sensor_base  -> ft_sensor_measurement
PIKA_TIP_OFFSET_M = 0.247642      # attachment_site (flange) -> tcp = pika_gripper.STL z-max

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
# is upstream ver1's stand visual origin verbatim -- identity, unlike RB3 ver5 whose
# stand visual carried rpy [0,0,-1.5708] to undo its own base->stand +90 deg joint.
STAND_VISUAL_MESH = "../meshes/stands/dual_rb5_850e/dual_rb5_850e_stand_ver1.stl"

# Display meshes for rb5_850e_pika_articulated.urdf (rb_gui only; the C++ FK/IK never
# reads it). Vendored .dae rather than upstream's per-material .obj: the two are the
# same object -- extents agree to 0.0002 mm -- and the .dae deviates from the finer
# .obj surface by p50 0.0000 mm, p99 <= 0.063 mm, max 0.271 mm, an order below the
# 1-2 mm the operator needs to judge on screen, for 26 MB instead of 69 MB.
ARM_VISUAL = {f"link{i}": f"../meshes/robots/rb5_850e/visual/link{i}.dae" for i in range(7)}
# The fingers carry TWO visuals each, one per printed material, so the operator can
# see which part of the tip is the rigid spine and which is the compliant blade. The
# factory one-piece finger was replaced on 2026-09-04 by the printed v15 tip: a black
# PLA spine plus a #F3E600 95A TPU blade. Both meshes and their placement come from
# rb_servo_server/tools/make_pika_tool_meshes.py, which carries the mounting-datum
# derivation and the 1.99 mm seating shift that goes with it.
#
# Multiple visuals per link is fine here and was verified, not assumed: yourdfpy 0.0.60
# gives each one its own scene node (geometry_0, geometry_1) parented to the link with
# the right transform, and viser 1.0.30's ViserUrdf renders them through
# add_mesh_trimesh, so the per-visual <material> colour survives.
# The `tool` link's three visuals are pika_gripper_base.STL partitioned by part (see
# make_pika_tool_meshes.py): the flange-side mount adapters keep the flat grey the whole
# tool used to render in, the RFT64 F/T sensor goes brighter, and the gripper itself goes
# black. pika_gripper_base.STL is unchanged and is still the collision/hull source.
TOOL_VISUAL = {
    "tool": [
        ("../meshes/robots/rb5_850e/visual/tool/pika_gripper_mount.STL", "mount_grey"),
        ("../meshes/robots/rb5_850e/visual/tool/pika_gripper_ft_sensor.STL", "ft_sensor_grey"),
        ("../meshes/robots/rb5_850e/visual/tool/pika_gripper_body.STL", "gripper_black"),
    ],
    "finger_left": [
        ("../meshes/robots/rb5_850e/visual/tool/pika_finger_left_pla.STL", "pla_black"),
        ("../meshes/robots/rb5_850e/visual/tool/pika_finger_left_tpu.STL", "tpu_95a_yellow"),
    ],
    "finger_right": [
        ("../meshes/robots/rb5_850e/visual/tool/pika_finger_right_pla.STL", "pla_black"),
        ("../meshes/robots/rb5_850e/visual/tool/pika_finger_right_tpu.STL", "tpu_95a_yellow"),
    ],
}
# The print materials, chosen so the RENDER lands on the target, not so the fraction
# looks tidy: trimesh's float32 round-trip floors, so the filament's #F3E600 =
# (243, 230, 0) needs G 0.902 -- the exact fraction 0.90196078 renders as 229.
# Verified by loading the generated URDF through yourdfpy.
#
# pla_black is lifted off 0.0 on purpose: the spine IS black filament, but at 0.0 it
# renders as an unshaded silhouette against the dark scene and the operator cannot read
# its form next to the yellow blade. Lower it if you want it truer to the part than to
# the screen.
# A single value ramp down the tool, so the four parts separate at a glance without any
# of them fighting the yellow blade for attention:
#   ft_sensor_grey  183  the RFT64's machined-aluminium look, the brightest thing here
#   mount_grey       76  the flange-side adapters, clearly darker than the sensor
#   gripper_black    43  the housing: black, but off 0 so its curvature still shades
#   pla_black        33  the printed spine, a shade under the housing it sits on
TOOL_MATERIALS = {
    "mount_grey": (0.30, 0.30, 0.30, 1.0),
    "ft_sensor_grey": (0.72, 0.72, 0.72, 1.0),
    "gripper_black": (0.17, 0.17, 0.17, 1.0),
    "pla_black": (0.13, 0.13, 0.13, 1.0),
    "tpu_95a_yellow": (0.95294118, 0.902, 0.0, 1.0),
}
# The CARRIAGE's stroke, MEASURED: the full-open gap is 98.0 mm and the jaws close to
# contact, so each finger travels half of it. The old 0.05/0.047 pair was never measured
# -- it was picked so the vendor CAD's finger pose would close to zero gap, which
# assumed that pose was the open stop. It is 3.8 mm/side short of it
# (make_pika_tool_meshes.py). Keep this equal to rb_gui's _GRIPPER_FINGER_TRAVEL_M and
# the config's gripper_finger_travel_m.
FINGER_TRAVEL_M = 0.049
# ---------------------------------------------------------------------------
# SITE MOUNT CALIBRATION (2026-09-06). Everything else in this file is upstream's
# geometry plus safety corrections; this is the one entry that is a property of THIS
# CELL, measured on it, and it must be re-measured if an arm is ever unbolted.
#
# HOW IT WAS MEASURED. 27 hand-parked CONTACT poses with both jaws at the mechanical
# open stop: the tip of one arm touching the other arm (6), or touching the stand
# (21, both arms). A contact means the true clearance is 0, so the model's residual
# distance at that pose is pure model error along that contact's normal -- one scalar
# equation each, which is why the campaign chased normal DIVERSITY rather than pose
# count. Solved for both mounts by least squares (tools: the offline audit built on
# CollisionMonitor itself, so the model being fitted is the one that gets enforced).
#
# Three things had to be handled or the answer came out wrong:
#   * The stand side is measured against the SOURCE STL, never the CoACD hulls --
#     those over-approximate by mean 1.4 / max 29.9 mm.
#   * The finger side uses the REAL printed mesh, not its convex hull: the hull fills
#     the TPU blade's honeycomb (p50 0.04 mm, but p90 5.2 / max 15.2 mm).
#   * A 7th unknown, a uniform stand SURFACE offset, is fitted alongside the mounts and
#     then DISCARDED. It comes out +1.94 +/- 0.32 mm (the real stand sits that far
#     outside its own STL -- paint, finish, or a nominal-dimension CAD). Leaving it out
#     of the fit biased both mounts, because every stand contact carries it; leaving it
#     out of the MODEL is correct, since it makes the guard conservative, not optimistic.
# Fit quality with that 7th term: residual RMS 1.06 mm, leave-one-out 1.51 mm, against
# a 0.2-0.6 mm repeatability floor measured by touching one spot from different arm
# postures. Condition number 3.2.
#
# WHAT IT IS WORTH, measured by re-evaluating all 27 contacts through the corrected
# model (excluding 4 where the closest pair switched, so before/after is not the same
# quantity): arm-to-arm error 2.54 -> 1.14 mm mean, and centred on zero (+1.74 ->
# +0.06). Arm-to-stand is unchanged (spread 1.54 -> 1.64 mm) -- this correction buys
# the CROSS-ARM pairs, which is where it matters: 187 arm<->arm plus the 9
# gripper<->gripper pairs a handover runs through.
#
# HONEST ABOUT THE Z. Per-component 1-sigma is 0.5-0.7 mm, so x and y are 2-3 sigma
# but BOTH z terms are inside their own error bar (-0.15 +/- 0.7, -0.71 +/- 0.7).
# They are applied because the least-squares estimate is unbiased, not because z is
# resolved. Do not read them as measured.
MOUNT_CALIBRATION_M = {           # added to upstream's stand_<side>_arm_base origin
    "left": (-0.00109, -0.00189, -0.00015),
    "right": (-0.00187, +0.00130, -0.00071),
}

STAND_VISUAL_XYZ = "0.0 0.0 0.0"
STAND_VISUAL_RPY = "0.0 0.0 0.0"
STAND_HULLS = [f"../meshes/stands/dual_rb5_850e/collision_ver1/stand_hull_{i:03d}.stl"
               for i in range(20)]

# ---------------------------------------------------------------------------
# CELL FURNITURE (env_* links): the work table, and the riser that carries the
# stand base plate above it. rb_gui draws anything named env_* that is fixed to
# `stand` (scene.py _environment_visuals_from_urdf); nothing else reads these.
#
# VISUAL ONLY, ON PURPOSE. buildGeom(..., pinocchio::COLLISION, ...) is what the
# CollisionMonitor loads, so a link with no <collision> adds zero geoms and the
# checked pair set is untouched. Giving the table a collision shell would start
# braking the arms against a surface whose pose has never been checked against
# contact -- that is a separate decision with its own evidence, not a side effect
# of wanting to see the table.
#
# THE FRAME, because every number below depends on it. Upstream ver2 models the
# stand base plate as stand_base_col, a 0.24 x 0.30 x 0.015 box centred at
# z = -0.0075, and the stand STL spans z -0.015 .. +0.6815. So:
#     z = 0      the plate's TOP face -- where the stand bolts down
#     z = -0.015 the plate's underside -- what the riser actually carries
#     +X         out over the work area (the enforced ROI is x 0.300 .. 1.100)
#     +Y         toward the LEFT arm (left mount y +0.19707)
# Each entry's `xyz` is the BOX CENTRE in the stand frame, in metres.
#
# NOT MEASURED YET. There is no CAD for either part, so these stay empty until
# the operator's tape measurements arrive. An empty list emits nothing and the
# committed URDF is unchanged -- which is the right failure: a guessed table is
# worse than no table, because the operator reads the gap between the drawn
# surface and the arm as a real clearance.
# RGBA NOTE. The tables are painted the SAME grey the viewer paints the stand,
# because they are the same kind of thing to the operator: fixed structure that is
# simply there. rb_gui paints the stand STL with _RB3_DARK_GRAY_RGB = (90, 90, 90)
# (the STL carries no colour of its own), so 90/255 = 0.352941 here. Change one and
# change the other. The riser is a lifted black -- darker than the stand so the
# 300 mm block reads as a separate part, but off 0 so its faces still shade
# (the same reason make_pika_tool_meshes.py lifts pla_black off 0).
ENVIRONMENT: list[dict] = [
    # 295, NOT the 300 first modelled, and it is a ROBOT MEASUREMENT rather than a tape
    # one. 2026-09-06 13:33 the operator hand-parked both gripper tips down onto the
    # table and held them there (servo_log_20260906_133253, last 10 s, both arms still
    # to 0.1-0.2 mm). The TCP IS the fingertip plane, so a flat tip reads the surface
    # directly. The LEFT arm settles it: its tool axis is 0.49 deg off vertical and all
    # four tip corners fall within 1.0 mm, i.e. genuinely flat, at z -310.5 mm. Riser =
    # 310 - 15 (the plate is 15 mm thick and z = 0 is its top face) = 295 mm.
    # The right arm agrees to 1.1 mm on the quantity that is actually comparable -- its
    # LOWEST tip corner, -309.4 -- but its tool axis reads 3.37 deg off vertical, so the
    # model has only its -x edge touching and its TCP centre 4.5 mm higher. See the note
    # in ENVIRONMENT's table entry; do not average the two TCP z values, they are not
    # measuring the same thing.
    # The F/T sensor could not arbitrate: ft_tare_state was empty in that run, so the
    # compensated wrench is untared bias (AGENTS.md auto-tare gap), not contact.
    # rb_gui can still retune this live (env_riser_height_m in ~/.rb_servo_gui/
    # settings.json); bring any settled value back here with its evidence.
    # FOOTPRINT 307 x 324, MEASURED. Two hand-parked poses (2026-09-06 13:43 and 13:52),
    # both arms, all four blade tips against the riser, joints read off the boxes with
    # rbpodo_read_state (read-only: it does not link rb::podo::Cobot, so it cannot command
    # motion while somebody is standing in the cell). The second pose was taken with both
    # grippers commanded FULLY OPEN, which is what makes the first one usable too: the
    # meshes are baked at the measured open stop, so travel = 0, and the two poses then
    # read the same faces to 1.7-4.1 mm. Had the jaws differed the readings would have
    # moved by tens of mm -- an earlier attempt to solve for the opening as a free unknown
    # returned -23 % (more closed than the mechanism closes), which was that free
    # parameter absorbing arm-to-arm error, not evidence about the jaws.
    # The fit is over all 8 finger meshes at once, three faces free (+x, +y, -y), each
    # finger contributing its minimum signed distance to the box. Fitting the MESH rather
    # than a single "outermost vertex" matters here: the blade tips straddle the block's
    # top edge, so the outermost vertex sits beyond the +x face while other vertices of
    # the same tip sit under the top face, and a per-face max reads ~20 mm too wide.
    #     +x face +153.3    +y face +166.9    -y face -157.1    rms residual 2.50 mm
    # -> Y span -157.1..+166.9 (324.0 wide, centre +4.9). X assumed concentric with the
    #    stand plate because NOTHING EVER TOUCHED THE -x FACE -- only +153.3 is measured.
    # Residual arm-to-arm bias: left -1.86 mm, right +1.41 mm. That 3.3 mm is the floor on
    # this measurement, and it is the same dual-arm relative error that the box DH sync
    # could not explain.
    # HEIGHT 295 is from a separate measurement (servo_log_20260906_133253): both gripper
    # tips parked flat DOWN on the table. That one does not depend on the jaw opening at
    # all -- the TCP IS the fingertip plane, wherever the jaws are. The left tool axis was
    # 0.49 deg off vertical with all four tip corners inside 1.0 mm at z -310.5 mm, and
    # 310 - 15 (plate thickness, z = 0 is its top face) = 295. The right arm agreed to
    # 1.1 mm on its lowest tip corner while sitting 3.37 deg tilted, which the operator
    # confirmed was a real tilt. F/T could not arbitrate anything here: ft_tare_state was
    # empty, so the compensated wrench is untared bias.
    # CHECKED, not just drawn (2026-09-06). The riser is the ONE env_* box the arms
    # actually approach: over 1,075 poses sampled from servo_log_20260906_131740 the
    # closest arm link came within 35.7 mm of it (left elbow, link3), while both work
    # tables stayed beyond 204 mm and never entered any barrier band. And nothing else
    # guards it -- safety.floor_constraint and self_collision.mesh.ground_plane are both
    # disabled, and the ground_plane comment says so explicitly: elbow/wrist descent is
    # left to arm<->arm / arm<->stand self-collision, which the riser was not part of.
    # Its band is safety.self_collision.mesh.environment (25/67 mm), NOT the 40/90 mm
    # self set: at 40 mm those same recorded poses violate on 1.1% of ticks, which would
    # clamp_hold the arm in a posture the cell already uses. See that config block.
    {"name": "env_stand_riser", "size": (0.307, 0.324, 0.295), "xyz": (0.0, 0.0049, -0.1625),
     "rgba": (0.20, 0.20, 0.21, 1.0), "collision": True,
     "measured": "2026-09-06 two four-tip contact poses with the grippers fully open "
                 "(rbpodo_read_state), 8-contact mesh fit, rms 2.50 mm, arm-to-arm bias "
                 "3.3 mm; height 295 from tips-down on the table."},
    # Two identical 800 mm cube tables butted along +X. The robot's riser stands on
    # table 1; the place boxes sit on table 2. The X placement is NOT a tape
    # measurement -- it is fitted to where the arms actually work in
    # servo_log_20260906_122520: the pick tray bottoms out at TCP z -292 mm over
    # x 356..518, and both place motions bottom out ~200 mm higher (box interiors) at
    # x 644..792. Putting the riser flush with table 1's back edge is the only
    # placement that puts the tray in the middle of table 1 AND the place boxes on
    # table 2. Re-measure the riser's setback from table 1's back edge to confirm.
    {"name": "env_work_table", "size": (0.80, 0.80, 0.80), "xyz": (0.25, 0.0, -0.710),
     "rgba": (0.352941, 0.352941, 0.352941, 1.0),
     "measured": "2026-09-06 operator: 800 x 800 x 800. Top z -310 mm follows the riser "
                 "measurement above (table top = riser underside). X/Y placement fitted "
                 "to the measured pick/place work regions, not taped."},
    {"name": "env_work_table_2", "size": (0.80, 0.80, 0.80), "xyz": (1.05, 0.0, -0.710),
     "rgba": (0.352941, 0.352941, 0.352941, 1.0),
     "measured": "2026-09-06 operator: second 800 cube butted to table 1 at x +650, "
                 "carrying the place boxes."},
]


def _environment_elements(spec: dict) -> tuple[ET.Element, ET.Element]:
    """One env_* link (a box) plus its fixed joint to `stand`.

    Visual only unless the entry sets `collision: True`. That flag is what puts the
    box into the CHECKED set: buildGeom(..., pinocchio::COLLISION, ...) is what the
    CollisionMonitor loads, so a <collision> here is the whole mechanism -- no
    extra_collision entry, no second copy of the dimensions. A <box> becomes a coal
    primitive, so it needs no convex hull STL.

    Turning it on for a piece of furniture is a SAFETY decision, not a drawing one:
    the arms will be braked and held against that surface, so its pose has to be
    measured (the `measured` provenance below is mandatory either way) and it needs a
    barrier band that its measured clearance actually clears --
    safety.self_collision.mesh.environment in the stack config, which is the band
    every env_* geometry is enforced against.
    """
    name = str(spec.get("name", ""))
    if not name.startswith("env_"):
        raise SystemExit(f"environment entry {name!r}: name must start with env_ "
                         "(rb_gui picks furniture up by that prefix)")
    size = tuple(float(v) for v in spec["size"])
    xyz = tuple(float(v) for v in spec["xyz"])
    if len(size) != 3 or min(size) <= 0.0:
        raise SystemExit(f"{name}: size must be three positive metres, got {size}")
    if len(xyz) != 3:
        raise SystemExit(f"{name}: xyz must be three metres, got {xyz}")
    if not str(spec.get("measured", "")).strip():
        raise SystemExit(f"{name}: no `measured` provenance. Every furniture dimension "
                         "is a tape measurement with no CAD behind it; record how and "
                         "when it was taken or it cannot be re-checked.")
    size_attr = " ".join(f"{v:.6g}" for v in size)
    link = ET.Element("link", {"name": name})
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": size_attr})
    material = ET.SubElement(visual, "material", {"name": name + "_material"})
    rgba = tuple(float(v) for v in spec.get("rgba", (0.55, 0.52, 0.48, 1.0)))
    ET.SubElement(material, "color", {"rgba": " ".join(f"{v:.4g}" for v in rgba)})
    if spec.get("collision", False):
        # The SAME box as the visual, from the same numbers, so what is drawn and what
        # is checked cannot disagree. (Two sources is how an operator ends up reading
        # the gap to a drawn surface as a real clearance.)
        collision = ET.SubElement(link, "collision")
        ET.SubElement(collision, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})
        col_geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(col_geometry, "box", {"size": size_attr})
    joint = _fixed_joint(name + "_fixed", "stand", name,
                         " ".join(f"{v:.6g}" for v in xyz), "0.0 0.0 0.0")
    return link, joint




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
    # ver1 ships the raw 38 k-triangle stand mesh as the stand's ONLY <collision>.
    # coal cannot hull a non-convex mesh, so it would be kept as a BVH: correct
    # distances, but far outside the per-eval budget the monitor's contract assumes.
    # Drop it here; step (3) below re-adds the same surface as 20 CoACD hulls.
    dropped_stand_cols = 0
    for col in stand_link.findall("collision"):
        stand_link.remove(col)
        dropped_stand_cols += 1
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

    # (3) stand hulls, replacing the raw stand mesh dropped above
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

    # (6) site mount calibration (see MOUNT_CALIBRATION_M). Applied here so the URDF
    # the CollisionMonitor loads and `left_mount`/`right_mount` in the stack configs
    # cannot drift apart -- rb_gui asserts they are equal
    # (test_default_mounts_match_the_tracked_unified_urdf), and the servo loop's FK/IK
    # reads the config while the guard reads this file.
    calibrated = 0
    for side, delta in MOUNT_CALIBRATION_M.items():
        joint = root.find(f"joint[@name='stand_{side}_arm_base_fixed']")
        if joint is None:
            raise SystemExit(f"upstream has no stand_{side}_arm_base_fixed to calibrate")
        origin = joint.find("origin")
        xyz = [float(v) for v in origin.get("xyz").split()]
        origin.set("xyz", " ".join(f"{v + d:.8f}" for v, d in zip(xyz, delta)))
        calibrated += 1
    if calibrated != 2:
        raise SystemExit(f"expected 2 arm mounts to calibrate, did {calibrated}")

    # (5) cell furniture: env_* boxes fixed to the stand (see ENVIRONMENT). Visual
    # only unless the entry opts into `collision`, which is what adds it to the
    # checked pair set (safety.self_collision.mesh.environment carries its band).
    for spec in ENVIRONMENT:
        name = str(spec.get("name", ""))
        if root.find(f"link[@name='{name}']") is not None:
            raise SystemExit(f"environment link {name} already exists in this URDF")
        link, joint = _environment_elements(spec)
        root.append(link)
        root.append(joint)

    if replaced != 2 * sum(len(v) for v in ARM_COLLISION.values()):
        raise SystemExit(f"expected both arms' collision shells, replaced {replaced}")
    if elbows != 2:
        raise SystemExit(f"expected 2 elbow joints, found {elbows}")
    # Fail closed if upstream ever stops shipping the raw stand collision: silently
    # emitting only the hulls would be fine, but silently emitting the hulls PLUS a
    # 38 k-triangle BVH (because this stopped matching) would not.
    if dropped_stand_cols != 1:
        raise SystemExit(
            f"expected exactly 1 raw <collision> on the stand link, dropped {dropped_stand_cols}")

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

    for name in ("ft_sensor_base", "ft_sensor_measurement", "tool"):
        if root.find(f"link[@name='{name}']") is not None:
            raise SystemExit(f"single-arm: upstream already defines {name}")
        ET.SubElement(root, "link", {"name": name})
    root.append(_fixed_joint("ft_sensor_base_joint", "attachment_site", "ft_sensor_base",
                             f"0.0 0.0 {FT_BASE_OFFSET_M}", f"0.0 0.0 {math.pi / 2:.16f}"))
    root.append(_fixed_joint("ft_sensor_measurement_joint", "ft_sensor_base",
                             "ft_sensor_measurement", f"0.0 0.0 {FT_MEASUREMENT_OFFSET_M}",
                             "0.0 0.0 0.0"))
    root.append(_fixed_joint("tool_joint", "attachment_site", "tool", "0.0 0.0 0.0",
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
    for mat_name, rgba in TOOL_MATERIALS.items():
        if root.find(f"material[@name='{mat_name}']") is not None:
            raise SystemExit(f"display: upstream already defines material {mat_name}")
        mat = ET.SubElement(root, "material", {"name": mat_name})
        ET.SubElement(mat, "color", {"rgba": " ".join(f"{c}" for c in rgba)})
    for name, visuals in TOOL_VISUAL.items():
        link = root.find(f"link[@name='{name}']")
        if link is None:
            link = ET.SubElement(root, "link", {"name": name})
        for mesh, mat_name in visuals:
            vis = ET.SubElement(link, "visual")
            ET.SubElement(vis, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})
            geo = ET.SubElement(vis, "geometry")
            ET.SubElement(geo, "mesh", {"filename": mesh, "scale": "0.001 0.001 0.001"})
            if mat_name is not None:
                ET.SubElement(vis, "material", {"name": mat_name})
    for name, lower, upper in (("finger_left", 0.0, FINGER_TRAVEL_M),
                               ("finger_right", -FINGER_TRAVEL_M, 0.0)):
        joint = ET.SubElement(root, "joint",
                              {"name": f"{name}_joint", "type": "prismatic"})
        ET.SubElement(joint, "parent", {"link": "attachment_site"})
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

    targets = [(UPSTREAM, OUT, build, "dual_rb5_850e_ver1.urdf"),
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
