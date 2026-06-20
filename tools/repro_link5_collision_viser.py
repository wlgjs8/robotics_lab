#!/usr/bin/env python3
"""Standalone viser reproduction: dual arm at the user's exact q_sent pose, showing
the link5<->stand collision the CollisionMonitor detects (-9.65mm) highlighted in red.
Open http://localhost:8077 to view. Pure visualization (no robot/hardware)."""
import time
from pathlib import Path
import numpy as np
import trimesh
import yourdfpy
import viser
from viser.extras import ViserUrdf

ROOT = Path("/home/plaif/workspace/mo_robot_descriptions/mo_robot_descriptions")
URDF = ROOT / "robots/urdf/dual_rb3_730e/dual_rb3_730e_ver5.urdf"

# user's exact q_sent (deg) — sign-verified from the fixed Joint Monitor
LEFT  = dict(base=303.38, shoulder=45.36, elbow=144.39, wrist1=11.28, wrist2=-124.86, wrist3=-135.65)
RIGHT = dict(base=-228.49, shoulder=-60.56, elbow=-99.92, wrist1=63.20, wrist2=109.39, wrist3=85.18)
# This scene renders the MONITOR's exact geometry (dual ver5 URDF) at the pose above.
# coal reports link5 <-> stand = +24.6mm (hulls) / >30mm (exact mesh) -> NOT a collision.
# Open alongside the live viser to compare whether link5 actually penetrates the stand.
CLEARANCE_MM = 24.6

def cfg_array(model):
    out = []
    for jn in model.actuated_joint_names:
        val = 0.0
        for k, v in LEFT.items():
            if jn == f"dual_rb3_730e_left_{k}_joint":
                val = np.deg2rad(v)
        for k, v in RIGHT.items():
            if jn == f"dual_rb3_730e_right_{k}_joint":
                val = np.deg2rad(v)
        out.append(val)
    return np.array(out)

def _mat_to_wxyz_pos(T):
    import numpy as np
    R = T[:3, :3]
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = (R[2, 1] - R[1, 2]) / (4 * w); y = (R[0, 2] - R[2, 0]) / (4 * w); z = (R[1, 0] - R[0, 1]) / (4 * w)
    return (w, x, y, z), tuple(T[:3, 3])

def main():
    server = viser.ViserServer(host="0.0.0.0", port=8077)
    model = yourdfpy.URDF.load(str(URDF), load_meshes=True, load_collision_meshes=True,
                              build_scene_graph=True, build_collision_scene_graph=True)
    cfg = cfg_array(model)
    model.update_cfg(cfg)
    # Root everything under a frame = inv(stand) so the STAND sits upright at the
    # scene origin (same orientation the live GUI shows), and children given in
    # URDF-world coords render stand-relative. This removes the world->stand +90deg
    # that made the standalone scene look rotated vs the live viewer.
    import numpy as np
    Tinv = np.linalg.inv(model.get_transform("stand"))
    wxyz, pos = _mat_to_wxyz_pos(Tinv)
    server.scene.add_frame("/scene", wxyz=wxyz, position=pos, show_axes=False)
    # full collision geometry, translucent blue
    overlay = ViserUrdf(server, URDF, root_node_name="/scene/checked",
                        load_meshes=False, load_collision_meshes=True,
                        collision_mesh_color_override=(0.45, 0.65, 1.0, 0.22))
    overlay.update_cfg(cfg)

    # stand visual mesh for context (gray)
    T_stand = model.get_transform("stand", collision_geometry=False)
    o = model.link_map["stand"].visuals[0].origin
    sm = trimesh.load(str(ROOT / "meshes/stands/dual_rb3_730e/dual_rb3_730e_stand_ver2_clean.stl"), force="mesh")
    sm.apply_scale(0.001)
    sm.apply_transform(T_stand @ o)
    server.scene.add_mesh_trimesh("/scene/stand_visual", mesh=sm)

    # highlight link5 (the one in question) in solid RED at its world pose
    T5 = model.get_transform("dual_rb3_730e_left_link5", collision_geometry=True)
    l5 = trimesh.load(str(ROOT / "meshes/robots/rb3_730e/collision/link5.stl"), force="mesh")
    l5v = trimesh.transform_points(l5.vertices, T5)
    server.scene.add_mesh_simple("/scene/link5_RED", vertices=l5v, faces=l5.faces,
                                 color=(230, 30, 30), opacity=0.95)

    # label at link5 (monitor's reported clearance to the stand at this pose)
    server.scene.add_label("/scene/clearance",
                           f"monitor: link5 <-> stand = +{CLEARANCE_MM:.1f} mm (NOT flagged)",
                           position=tuple(T5[:3, 3] + np.array([0, 0, 0.08])))

    print("=" * 60)
    print("  viser ready -> open  http://localhost:8077")
    print(f"  link5<->stand clearance = {CLEARANCE_MM:.2f} mm (red = link5, gray = stand)")
    print("=" * 60, flush=True)
    while True:
        time.sleep(2.0)

if __name__ == "__main__":
    main()
