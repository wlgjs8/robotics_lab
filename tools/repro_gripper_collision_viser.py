#!/usr/bin/env python3
"""Standalone viser: dual arm at a q_sent pose, showing the MONITOR's gripper collision
hulls (red, exactly as the CollisionMonitor attaches them: pika_gripper.STL convex hull
at each attachment_site, rotated +90deg about Z) plus the arms and stand. Open
http://localhost:8078 and compare to the live viser — do the red gripper hulls overlap,
or is there a gap where your live grippers appear to collide?"""
import time
from pathlib import Path
import numpy as np
import trimesh
import yourdfpy
import viser
from viser.extras import ViserUrdf

ROOT = Path("/home/plaif/workspace/mo_robot_descriptions/mo_robot_descriptions")
URDF = ROOT / "robots/urdf/dual_rb3_730e/dual_rb3_730e_ver5.urdf"
GRIPPER_STL = Path("/home/plaif/workspace/robotics_lab/rb_servo_server/descriptions/"
                   "meshes/robots/rb3_730e/visual/tool/pika_gripper.STL")

LEFT  = dict(base=281.28, shoulder=63.57, elbow=110.14, wrist1=-21.33, wrist2=-96.49, wrist3=-133.82)
RIGHT = dict(base=-293.04, shoulder=-62.93, elbow=-113.52, wrist1=-0.77, wrist2=97.96, wrist3=135.29)
GRIPPER_CLEARANCE_MM = 12.1


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


def _wxyz_pos(T):
    R = T[:3, :3]
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = (R[2, 1] - R[1, 2]) / (4 * w); y = (R[0, 2] - R[2, 0]) / (4 * w); z = (R[1, 0] - R[0, 1]) / (4 * w)
    return (w, x, y, z), tuple(T[:3, 3])


def main():
    server = viser.ViserServer(host="0.0.0.0", port=8078)
    model = yourdfpy.URDF.load(str(URDF), load_meshes=True, load_collision_meshes=True,
                              build_scene_graph=True, build_collision_scene_graph=True)
    cfg = cfg_array(model)
    model.update_cfg(cfg)
    Tinv = np.linalg.inv(model.get_transform("stand"))
    wxyz, pos = _wxyz_pos(Tinv)
    server.scene.add_frame("/scene", wxyz=wxyz, position=pos, show_axes=False)
    overlay = ViserUrdf(server, URDF, root_node_name="/scene/arms",
                        load_meshes=False, load_collision_meshes=True,
                        collision_mesh_color_override=(0.45, 0.65, 1.0, 0.20))
    overlay.update_cfg(cfg)

    # stand visual (gray)
    o = model.link_map["stand"].visuals[0].origin
    sm = trimesh.load(str(ROOT / "meshes/stands/dual_rb3_730e/dual_rb3_730e_stand_ver2_clean.stl"), force="mesh")
    sm.apply_scale(0.001); sm.apply_transform(model.get_transform("stand") @ o)
    server.scene.add_mesh_trimesh("/scene/stand", mesh=sm)

    # gripper collision hull, attached EXACTLY as the monitor does: attachment_site * Rz(+90)
    g = trimesh.load(str(GRIPPER_STL), force="mesh"); g.apply_scale(0.001)
    ghull = g.convex_hull
    rz90 = trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 0, 1])
    for side, col in (("left", (230, 30, 30)), ("right", (230, 90, 30))):
        T = model.get_transform(f"dual_rb3_730e_{side}_attachment_site") @ rz90
        V = trimesh.transform_points(ghull.vertices, T)
        server.scene.add_mesh_simple(f"/scene/{side}_gripper_hull", vertices=V, faces=ghull.faces,
                                     color=col, opacity=0.85)
        tcp = model.get_transform(f"dual_rb3_730e_{side}_tcp")[:3, 3]
        server.scene.add_icosphere(f"/scene/{side}_tcp", radius=0.008, color=(0, 200, 0), position=tuple(tcp))
    server.scene.add_label("/scene/info",
                           f"monitor: gripper<->gripper hull = +{GRIPPER_CLEARANCE_MM:.1f} mm  (green = tcp/grasp point)",
                           position=tuple(model.get_transform("dual_rb3_730e_left_attachment_site")[:3, 3] + np.array([0, 0, 0.1])))
    print("=" * 60)
    print("  viser ready -> http://localhost:8078")
    print(f"  red = monitor gripper hulls (gap {GRIPPER_CLEARANCE_MM}mm); green = tcp/grasp points")
    print("=" * 60, flush=True)
    while True:
        time.sleep(2.0)


if __name__ == "__main__":
    main()
