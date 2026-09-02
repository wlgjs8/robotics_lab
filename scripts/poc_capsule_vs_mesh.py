#!/usr/bin/env python3.10
"""Compare current hand-fit CAPSULE self-collision vs URDF MESH/box self-collision.

Both paths use the SAME unified-model FK and the SAME pinocchio/coal dispatch, so
the only variable is geometry representation. Produces accuracy + performance tables.

Includes step 1: attach the Pika gripper (pika_gripper.STL) as a convex hull to each
arm's attachment_site frame (the unified URDF has no gripper collision otherwise).

Run:
  PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages \
  LD_LIBRARY_PATH=/opt/openrobots/lib \
  python3.10 scripts/poc_capsule_vs_mesh.py
"""
import os, time
import numpy as np
import pinocchio as pin
import coal

WS = "/home/plaif/workspace"
UNIFIED = f"{WS}/robotics_lab/rb_servo_server/descriptions/urdf/dual_rb5_850e_ver3.urdf"
PIKA_STL = f"{WS}/robotics_lab/rb_servo_server/descriptions/meshes/robots/rb5_850e/visual/tool/pika_gripper.STL"
DEG = np.pi / 180.0

LJ = [f"dual_rb5_850e_left_{n}_joint" for n in ["base","shoulder","elbow","wrist1","wrist2","wrist3"]]
RJ = [f"dual_rb5_850e_right_{n}_joint" for n in ["base","shoulder","elbow","wrist1","wrist2","wrist3"]]

# --- current hand-fit capsule definitions (config.hpp defaultRb3ArmCapsules) ---
ARM_CAPS = [
    ("link0",(+0.0696,-0.0007,+0.0256),(-0.0623,+0.0023,+0.0359),0.0566),
    ("link1",(-0.0052,-0.0212,-0.0681),(-0.0010,+0.0093,+0.0579),0.0647),
    ("link2",(-0.0003,-0.1552,-0.0497),(+0.0011,-0.0772,+0.0790),0.0804),
    ("link2",(+0.0003,-0.1174,+0.0812),(-0.0015,-0.1194,+0.2131),0.0376),
    ("link2",(-0.0028,-0.0744,+0.2178),(+0.0002,-0.1501,+0.3285),0.0706),
    ("link3",(-0.0001,+0.0175,-0.0464),(+0.0009,-0.0355,+0.0669),0.0624),
    ("link4",(+0.0014,-0.1107,+0.2140),(-0.0026,-0.0893,+0.3132),0.0405),
    ("link4",(+0.0407,+0.0077,+0.0764),(-0.0407,+0.0076,+0.0765),0.0490),
    ("link4",(+0.0003,-0.1457,+0.3534),(+0.0005,-0.0296,+0.3459),0.0525),
    ("link4",(+0.0003,-0.1362,+0.2054),(-0.0001,+0.0451,+0.1032),0.0513),
    ("link5",(+0.0034,-0.0138,+0.0650),(-0.0017,+0.0081,-0.0486),0.0399),
    ("link6",(+0.0018,+0.0347,+0.0784),(-0.0175,-0.0385,+0.0756),0.0307),
    ("attachment_site",(0,0,0),(0,0,0.1000),0.0350),
    ("attachment_site",(0,+0.0607,+0.0810),(0,+0.0940,+0.0810),0.0280),
    ("attachment_site",(-0.1039,0,+0.1305),(+0.1039,0,+0.1305),0.0317),
    ("attachment_site",(-0.0594,0,+0.2286),(+0.0594,0,+0.2286),0.0220),
]  # index 0 = link0 (stand_ignore_bones target)
STAND_CAPS = [
    ("base_plate_1",(-0.15,0.06,0.005),(0.15,0.06,0.005),0.0602),
    ("base_plate_2",(-0.15,-0.06,0.005),(0.15,-0.06,0.005),0.0602),
    ("lower_column",(0,0,0),(0,0,0.43),0.0943),
    ("upper_column",(0,0.0159,0.3941),(0,-0.1432,0.5532),0.0943),
    ("shoulder_block_1",(-0.105,-0.1404,0.6034),(0.105,-0.1404,0.6034),0.053),
    ("shoulder_block_2",(-0.105,-0.1934,0.6564),(0.105,-0.1934,0.6564),0.053),
    ("shoulder_block_3",(-0.105,-0.1934,0.5504),(0.105,-0.1934,0.5504),0.053),
    ("shoulder_block_4",(-0.105,-0.2464,0.6034),(0.105,-0.2464,0.6034),0.053),
    ("shoulder_plate_1",(-0.25,-0.2136,0.6766),(0.25,-0.2135,0.6766),0.0388),
    ("shoulder_plate_2",(-0.25,-0.2666,0.6235),(0.25,-0.2666,0.6235),0.0388),
    ("mount_plate_a_1",(-0.2562,-0.2172,0.6838),(-0.0992,-0.1062,0.5728),0.0403),
    ("mount_plate_a_2",(-0.2562,-0.2738,0.6272),(-0.0992,-0.1628,0.5162),0.0403),
    ("mount_plate_b_1",(0.0992,-0.1062,0.5728),(0.2562,-0.2172,0.6838),0.0403),
    ("mount_plate_b_2",(0.0992,-0.1628,0.5162),(0.2562,-0.2738,0.6272),0.0403),
]
STAND_IGNORE_BONES = [0]   # index into ARM_CAPS, from stack_sim.yaml
MARGIN_M = 0.003

def banner(s): print("\n"+"="*74+f"\n{s}\n"+"="*74)
def set_q(model,q,jn,vals):
    for n,v in zip(jn,vals): q[model.joints[model.getJointId(n)].idx_q]=v
def z_to(d):
    d=np.asarray(d,float); d=d/np.linalg.norm(d); z=np.array([0,0,1.])
    v=np.cross(z,d); c=float(z@d)
    if c>1-1e-9: return np.eye(3)
    if c<-1+1e-9: return np.diag([1.,-1.,-1.])
    vx=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+vx+vx@vx*(1/(1+c))
def cap_local(p0,p1):
    p0=np.array(p0,float); p1=np.array(p1,float)
    L=float(np.linalg.norm(p1-p0)); center=(p0+p1)/2
    R=z_to(p1-p0) if L>1e-9 else np.eye(3)
    return pin.SE3(R,center), L

# ---------------------------------------------------------------------------
banner("LOAD unified model")
model=pin.buildModelFromUrdf(UNIFIED); data=model.createData()
URDF_DIR=os.path.dirname(UNIFIED)

# ---- MESH geometry model (collision) + gripper hull attach (STEP 1) --------
gmesh=pin.buildGeomFromUrdf(model,UNIFIED,pin.GeometryType.COLLISION,package_dirs=[URDF_DIR])
for go in gmesh.geometryObjects:
    try:
        go.geometry.buildConvexRepresentation(False)
        if go.geometry.convex is not None: go.geometry=go.geometry.convex
    except Exception: pass

# attach Pika gripper convex hull to each arm's attachment_site
loader=coal.MeshLoader()
pika_bvh=loader.load(PIKA_STL, np.array([0.001,0.001,0.001]))
pika_bvh.buildConvexRepresentation(False)
pika_hull=pika_bvh.convex
Rz90=pin.SE3(pin.utils.rotate('z',np.pi/2),np.zeros(3))
for side in ("left","right"):
    fid=model.getFrameId(f"dual_rb5_850e_{side}_attachment_site")
    fr=model.frames[fid]
    placement=fr.placement*Rz90
    go=pin.GeometryObject(f"{side}_pika_gripper",fr.parentJoint,fr.parentFrame,placement,pika_hull)
    gmesh.addGeometryObject(go)
print(f"mesh geoms (incl 2 gripper hulls): {len(gmesh.geometryObjects)}")

mnames=[g.name for g in gmesh.geometryObjects]
def mside(nm):
    if "left" in nm: return "left"
    if "right" in nm: return "right"
    return "stand"
mg={"left":[],"right":[],"stand":[]}
for i,nm in enumerate(mnames): mg[mside(nm)].append(i)
# mesh: emulate ignore_bones[0] -> drop link0 geom vs stand
def is_link0(nm): return "link0" in nm
gmesh.removeAllCollisionPairs()
for i in mg["left"]:
    for j in mg["right"]: gmesh.addCollisionPair(pin.CollisionPair(i,j))
for s in ("left","right"):
    for i in mg[s]:
        if is_link0(mnames[i]): continue
        for j in mg["stand"]: gmesh.addCollisionPair(pin.CollisionPair(i,j))
dmesh=gmesh.createData()
print(f"mesh pairs: {len(gmesh.collisionPairs)}")

# ---- CAPSULE geometry model (built per ignore-set) -------------------------
def build_capsule_geom(ignore_set):
    g=pin.GeometryModel(); idxmap={"left":[],"right":[],"stand":[]}
    def add_cap(name,frame_name,p0,p1,r):
        fid=model.getFrameId(frame_name); fr=model.frames[fid]
        locT,L=cap_local(p0,p1)
        go=pin.GeometryObject(name,fr.parentJoint,fr.parentFrame,fr.placement*locT,coal.Capsule(r,L))
        return g.addGeometryObject(go)
    for side,prefix in (("left","dual_rb5_850e_left_"),("right","dual_rb5_850e_right_")):
        for k,(frame,p0,p1,r) in enumerate(ARM_CAPS):
            idxmap[side].append((k,add_cap(f"{side}_arm{k}_{frame}",prefix+frame,p0,p1,r)))
    for k,(name,p0,p1,r) in enumerate(STAND_CAPS):
        idxmap["stand"].append((k,add_cap(f"stand_{name}","stand",p0,p1,r)))
    g.removeAllCollisionPairs()
    for _,i in idxmap["left"]:
        for _,j in idxmap["right"]: g.addCollisionPair(pin.CollisionPair(i,j))
    for s in ("left","right"):
        for k,i in idxmap[s]:
            if k in ignore_set: continue
            for _,j in idxmap["stand"]: g.addCollisionPair(pin.CollisionPair(i,j))
    return g, g.createData(), [go.name for go in g.geometryObjects]

# deployed ignore [0] vs memory-recommended [0,1,2]
gcap, dcap, cnames = build_capsule_geom([0])           # stack_sim.yaml current
gcap3, dcap3, _    = build_capsule_geom([0,1,2])       # memory-recommended fix
print(f"capsule geoms: {len(gcap.geometryObjects)}  pairs[ignore=0]: {len(gcap.collisionPairs)}  pairs[ignore=0,1,2]: {len(gcap3.collisionPairs)}")

# ---------------------------------------------------------------------------
def min_clearance(geomm,geomd,q):
    pin.computeDistances(model,data,geomm,geomd,q)
    dmin,idx=1e9,-1
    for k in range(len(geomm.collisionPairs)):
        d=geomd.distanceResults[k].min_distance
        if d<dmin: dmin,idx=d,k
    cp=geomm.collisionPairs[idx]
    return dmin,cp.first,cp.second

POSES={
 "init [0,-30,80,0,60,0]": (np.array([0,-30,80,0,60,0.]),np.array([0,-30,80,0,60,0.])),
 "zeros":                  (np.zeros(6),np.zeros(6)),
 "elbows out (j2=-60)":    (np.array([0,-60,90,0,40,0.]),np.array([0,-60,90,0,40,0.])),
 "reach inward (j1=±40)":  (np.array([40,-20,70,0,50,0.]),np.array([-40,-20,70,0,50,0.])),
 "arms up (j2=-90)":       (np.array([0,-90,30,0,60,0.]),np.array([0,-90,30,0,60,0.])),
}
rng=np.random.default_rng(7)
for n in range(4): POSES[f"random#{n}"]=(rng.uniform(-80,80,6),rng.uniform(-80,80,6))

banner("ACCURACY: min self-collision clearance (mm)  CAPSULE vs MESH(+gripper)")
def short(nm):
    return nm.replace("dual_rb5_850e_","").replace("stand_","s.").replace("_arm","")
print("caps[0]=deployed ignore_bones, caps[012]=memory-recommended, mesh=URDF+gripper")
print(f"{'pose':24s} {'caps[0]':>8s} {'caps[012]':>9s} {'mesh':>8s}   mesh min-pair")
for label,(ql,qr) in POSES.items():
    q=pin.neutral(model); set_q(model,q,LJ,ql*DEG); set_q(model,q,RJ,qr*DEG)
    c0,_,_=min_clearance(gcap,dcap,q)
    c3,_,_=min_clearance(gcap3,dcap3,q)
    mmin,mi,mj=min_clearance(gmesh,dmesh,q)
    print(f"{label:24s} {c0*1000:8.1f} {c3*1000:9.1f} {mmin*1000:8.1f}   "
          f"{short(mnames[mi])}<->{short(mnames[mj])}")

banner("PERFORMANCE (unified-model FK + computeDistances, 2000 iters)")
q=pin.neutral(model); set_q(model,q,LJ,POSES['init [0,-30,80,0,60,0]'][0]*DEG)
set_q(model,q,RJ,POSES['init [0,-30,80,0,60,0]'][1]*DEG)
def bench(fn,N=2000):
    fn(); t0=time.perf_counter()
    for _ in range(N): fn()
    return (time.perf_counter()-t0)/N*1e6
tcap=bench(lambda: pin.computeDistances(model,data,gcap,dcap,q))
tmesh=bench(lambda: pin.computeDistances(model,data,gmesh,dmesh,q))
tcapcol=bench(lambda: pin.computeCollisions(model,data,gcap,dcap,q,False))
tmeshcol=bench(lambda: pin.computeCollisions(model,data,gmesh,dmesh,q,False))
print(f"{'aspect':26s} {'CAPSULE':>14s} {'MESH(+gripper)':>16s}")
print(f"{'geometry objects':26s} {len(gcap.geometryObjects):>14d} {len(gmesh.geometryObjects):>16d}")
print(f"{'collision pairs':26s} {len(gcap.collisionPairs):>14d} {len(gmesh.collisionPairs):>16d}")
print(f"{'computeDistances us/cyc':26s} {tcap:>14.1f} {tmesh:>16.1f}")
print(f"{'computeCollisions us/cyc':26s} {tcapcol:>14.1f} {tmeshcol:>16.1f}")
print(f"{'% of 500Hz (2000us) budget':26s} {tcap/2000*100:>13.1f}% {tmesh/2000*100:>15.1f}%")
print("\nNOTE: python binding adds fixed per-call overhead; C++ in-loop is faster for both.")
print("NOTE: gripper = single convex hull (fills finger gap -> conservative).")
print("      production should coacd-decompose pika_gripper.STL like link2/link4.")
print("DONE.")
