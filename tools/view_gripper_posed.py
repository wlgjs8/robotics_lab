"""Standalone viser viewer that poses both arms at the InitMotion joint angles.

Mock GUI has no servo state, so the arms otherwise render folded at q=0 and the
tool/gripper on link6 is hard to inspect. This builds the same scene
(stand + both rb3_730e URDFs + gripper) and pushes the init joint config so the
gripper mounted on joint 6 is clearly visible.

Run:
    PYTHONPATH=rb_gui RB_GUI_PORT=8083 python3 tools/view_gripper_posed.py
"""
from __future__ import annotations

import os
import time

import viser

from rb_servo_gui.scene import (
    _add_scene_fallback,
    _joint_cfg_radians,
    _update_urdf_config,
)
from rb_servo_gui.app import (
    _DEFAULT_INIT_LEFT_JOINTS_DEG,
    _DEFAULT_INIT_RIGHT_JOINTS_DEG,
)


def main() -> None:
    host = os.environ.get("RB_GUI_HOST", "0.0.0.0")
    port = int(os.environ.get("RB_GUI_PORT", "8083"))
    server = viser.ViserServer(host=host, port=port)
    handles = _add_scene_fallback(server)

    poses = {
        "left_urdf": _DEFAULT_INIT_LEFT_JOINTS_DEG,
        "right_urdf": _DEFAULT_INIT_RIGHT_JOINTS_DEG,
    }
    for key, q_deg in poses.items():
        if key in handles:
            _update_urdf_config(handles[key], _joint_cfg_radians(q_deg))
            print(f"{key}: posed at {q_deg} deg")
        else:
            print(f"WARNING: {key} not in scene handles (asset load failed?)")

    if "urdf_error" in handles:
        print(f"urdf_error: {handles['urdf_error']}")
    if "stand_mesh_error" in handles:
        print(f"stand_mesh_error: {handles['stand_mesh_error']}")

    print(f"posed gripper viewer listening on http://{host}:{port}", flush=True)
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
