#!/usr/bin/env python3.10
"""PoC: URDF/coal mesh-based self-collision vs current hand-fit capsule path.

Validates the two A-plan blockers with measurements:
  (A) Frame/mount agreement: unified-URDF FK == single-arm-URDF FK + mount transform.
  (B) Real-time performance: time computeDistances over curated collision pairs.

Run:
  PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages \
  LD_LIBRARY_PATH=/opt/openrobots/lib \
  python3.10 scripts/poc_urdf_self_collision.py
"""
import os
import time
import numpy as np
import pinocchio as pin

WS = "/home/plaif/workspace"
UNIFIED_URDF = f"{WS}/mo_robot_descriptions/mo_robot_descriptions/robots/urdf/dual_rb3_730e/dual_rb3_730e_ver3.urdf"
MESH_ROOT = f"{WS}/mo_robot_descriptions/mo_robot_descriptions/meshes"
MESH_PKG_DIR = f"{WS}/mo_robot_descriptions/mo_robot_descriptions"  # so '../../../meshes' resolves
SINGLE_URDF = f"{WS}/robotics_lab/rb_servo_server/descriptions/urdf/rb3_730e.urdf"
SINGLE_MESH_DIR = f"{WS}/robotics_lab/rb_servo_server/descriptions"

DEG = np.pi / 180.0
INIT_POSE_DEG = np.array([0.0, -30.0, 80.0, 0.0, 60.0, 0.0])

LEFT_JOINTS = [f"dual_rb3_730e_left_{n}_joint" for n in
               ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]]
RIGHT_JOINTS = [f"dual_rb3_730e_right_{n}_joint" for n in
                ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]]

LEFT_MOUNT = [0.1601, -0.1725, 0.5825, 2.186649, 0.523831, 2.526296]
RIGHT_MOUNT = [-0.1601, -0.1725, 0.5825, 2.186649, -0.523831, -2.526296]


def banner(s):
    print("\n" + "=" * 70 + f"\n{s}\n" + "=" * 70)


def set_q(model, q, joint_names, vals_rad):
    for jn, v in zip(joint_names, vals_rad):
        jid = model.getJointId(jn)
        q[model.joints[jid].idx_q] = v


def se3_from_xyzrpy(v):
    R = pin.rpy.rpyToMatrix(v[3], v[4], v[5])
    return pin.SE3(R, np.array(v[:3]))


# ----------------------------------------------------------------------------
banner("LOAD unified model + collision geometry")
model = pin.buildModelFromUrdf(UNIFIED_URDF)
data = model.createData()
print(f"nq={model.nq} nv={model.nv} njoints={model.njoints}")

URDF_DIR = os.path.dirname(UNIFIED_URDF)  # so '../../../meshes' resolves relative to urdf
geom = pin.buildGeomFromUrdf(model, UNIFIED_URDF, pin.GeometryType.COLLISION,
                             package_dirs=[URDF_DIR, MESH_PKG_DIR, MESH_ROOT, WS])
print(f"geometry objects: {len(geom.geometryObjects)}")

# convexify meshes for fast GJK; primitives (box/sphere) untouched
n_convex = 0
for go in geom.geometryObjects:
    try:
        go.geometry.buildConvexRepresentation(False)
        cvx = go.geometry.convex
        if cvx is not None:
            go.geometry = cvx
            n_convex += 1
    except Exception:
        pass
print(f"convexified: {n_convex}")

# classify geometry objects
names = [go.name for go in geom.geometryObjects]
def classify(nm):
    if "left" in nm:
        return "left"
    if "right" in nm:
        return "right"
    if "stand" in nm or "base_col" in nm or "body" in nm:
        return "stand"
    return "other"
groups = {"left": [], "right": [], "stand": [], "other": []}
for i, nm in enumerate(names):
    groups[classify(nm)].append(i)
for k, v in groups.items():
    print(f"  {k:6s}: {len(v):3d}  e.g. {[names[i] for i in v[:4]]}")

# ----------------------------------------------------------------------------
banner("BLOCKER A: FK / mount agreement (unified vs single-arm + mount)")
smodel = pin.buildModelFromUrdf(SINGLE_URDF)
sdata = smodel.createData()
s_joints = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
# discover single-arm revolute joint names
s_rev = [smodel.names[j] for j in range(1, smodel.njoints)
         if smodel.joints[j].nq == 1]
print(f"single-arm revolute joints: {s_rev}")
s_world = smodel.getFrameId("world")
s_tcp = smodel.getFrameId("tcp")

# Compare link6 (flange) — the kinematic chain that drives collision geometry.
# NB: 'tcp' differs between URDFs (single-arm tcp includes Pika tip +0.347642 m;
# unified tcp is link6-relative), so tcp is NOT a valid agreement check.
def single_flange_in_stand(qdeg, mount):
    sq = pin.neutral(smodel)
    for jn, v in zip(s_rev, qdeg * DEG):
        jid = smodel.getJointId(jn)
        sq[smodel.joints[jid].idx_q] = v
    pin.forwardKinematics(smodel, sdata, sq)
    pin.updateFramePlacements(smodel, sdata)
    base_M = sdata.oMf[s_world]
    flange_M = sdata.oMf[smodel.getFrameId("link6")]
    return se3_from_xyzrpy(mount) * (base_M.inverse() * flange_M)

def unified_flange_in_stand(qdeg_left, qdeg_right, side):
    q = pin.neutral(model)
    set_q(model, q, LEFT_JOINTS, qdeg_left * DEG)
    set_q(model, q, RIGHT_JOINTS, qdeg_right * DEG)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    stand_M = data.oMf[model.getFrameId("stand")]
    flange_M = data.oMf[model.getFrameId(f"dual_rb3_730e_{side}_link6")]
    return stand_M.inverse() * flange_M
single_tcp_in_stand = single_flange_in_stand
unified_tcp_in_stand = unified_flange_in_stand

rng = np.random.default_rng(0)
test_poses = [INIT_POSE_DEG] + [rng.uniform(-90, 90, 6) for _ in range(5)]
max_dt, max_dr = 0.0, 0.0
for qd in test_poses:
    for side, mount in (("left", LEFT_MOUNT), ("right", RIGHT_MOUNT)):
        ql = qd if side == "left" else np.zeros(6)
        qr = qd if side == "right" else np.zeros(6)
        u = unified_tcp_in_stand(ql, qr, side)
        s = single_tcp_in_stand(qd, mount)
        diff = u.inverse() * s
        dt = np.linalg.norm(diff.translation)
        dr = np.linalg.norm(pin.log3(diff.rotation))
        max_dt = max(max_dt, dt); max_dr = max(max_dr, dr)
print(f"max translation diff: {max_dt*1000:.4f} mm")
print(f"max rotation diff   : {max_dr*1000:.4f} mrad")
print("VERDICT:", "MATCH (mount consistent)" if max_dt < 1e-3 and max_dr < 1e-3
      else "MISMATCH -> investigate")

# ----------------------------------------------------------------------------
banner("Build curated collision pairs (left-right, left-stand, right-stand)")
def add_cross(gm, a, b):
    for i in a:
        for j in b:
            gm.addCollisionPair(pin.CollisionPair(i, j))
geom.removeAllCollisionPairs()
add_cross(geom, groups["left"], groups["right"])
add_cross(geom, groups["left"], groups["stand"])
add_cross(geom, groups["right"], groups["stand"])
n_cross = len(geom.collisionPairs)
print(f"cross pairs (capsule-equivalent scope): {n_cross}")
gdata = geom.createData()

# ----------------------------------------------------------------------------
banner("Clearance at INIT pose [0,-30,80,0,60,0] both arms")
q = pin.neutral(model)
set_q(model, q, LEFT_JOINTS, INIT_POSE_DEG * DEG)
set_q(model, q, RIGHT_JOINTS, INIT_POSE_DEG * DEG)
pin.computeDistances(model, data, geom, gdata, q)
dmin, dmin_idx = 1e9, -1
for k in range(n_cross):
    d = gdata.distanceResults[k].min_distance
    if d < dmin:
        dmin, dmin_idx = d, k
cp = geom.collisionPairs[dmin_idx]
print(f"min clearance: {dmin*1000:.2f} mm  between "
      f"[{names[cp.first]}] <-> [{names[cp.second]}]")
print("(memory note: hand-fit capsule path reported ~25.3mm full / +134mm after ignore_bones)")

# ----------------------------------------------------------------------------
banner("BLOCKER B: performance (computeDistances over cross pairs)")
N = 2000
# warm
pin.computeDistances(model, data, geom, gdata, q)
t0 = time.perf_counter()
for _ in range(N):
    pin.computeDistances(model, data, geom, gdata, q)
t1 = time.perf_counter()
per = (t1 - t0) / N
print(f"{n_cross} pairs: {per*1e6:.1f} us/cycle  (over {N} iters)")
print(f"  budget @500Hz = 2000 us, @250Hz = 4000 us")

# collide (boolean stop-at-first) is cheaper; measure for monitor fallback
req = pin.computeCollisions  # full
t0 = time.perf_counter()
for _ in range(N):
    pin.computeCollisions(model, data, geom, gdata, q, False)
t1 = time.perf_counter()
print(f"computeCollisions(stopAtFirst=False): {(t1-t0)/N*1e6:.1f} us/cycle")
print("\nNOTE: python binding adds per-call overhead; C++ in-loop will be faster.")
print("DONE.")
