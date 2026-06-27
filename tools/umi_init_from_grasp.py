#!/usr/bin/env python3
"""Propose an InitMotion TCP pose that matches the UMI training init distribution.

Problem: UMI demos were collected with a handheld rig, so the *absolute* init
pose lives in `steamvr_world` (retarget=configured_estimate -> not physically
aligned to the robot stand; see calibration/umi_retarget_eelocal.yaml). What IS
transferable is the hand-camera-relative scene geometry: across 146 episodes the
gripper consistently starts a fixed body-frame offset *behind/above the bolt it
grasps*. This tool reproduces that offset on the robot.

Workflow:
  1. Place the box+bolts on the robot table where you want them.
  2. Jog each gripper to a representative *grasp* pose (closed on a bolt) and read
     observations/tcp_stand_{left,right} from the GUI / state stream.
  3. Run this tool with those grasp poses -> it prints the InitMotion TCP target.
  4. In the GUI, TcpPoseTarget-move each arm to that pose, then
     "Init Motion 으로 설정하기" to persist it (or export RB_GUI_INIT_*_JOINTS).

Pose convention (matches episode pose_format): x y z qx qy qz qw, stand frame, metres.

Measured mean offset (grasp expressed in the INIT hand frame), from data_tcp/*:
  left : t_body=[+0.006,-0.091,+0.205] m   (|d|=0.243),  ~14 deg wrist pitch+yaw
  right: t_body=[+0.021,-0.074,+0.206] m   (|d|=0.234),  ~17 deg wrist pitch+yaw
i.e. the bolt sits ~0.20 m ahead along the approach (camera) axis and ~0.08 m to
one side; the gripper is tilted ~20 deg forward from straight-down at init.
"""
from __future__ import annotations

import argparse
import numpy as np

# init-in-grasp-frame transform per arm (inverse of the measured grasp-in-init mean).
# t = init position in the grasp TCP frame [m]; q = init orientation rel. grasp (xyzw).
_INIT_IN_GRASP = {
    "left":  (np.array([0.013, 0.139, -0.176]), np.array([0.1253, 0.0181, -0.1089, 0.9859])),
    "right": (np.array([-0.010, 0.138, -0.171]), np.array([0.1719, -0.0688, 0.0750, 0.9798])),
}


def quat_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R: np.ndarray) -> np.ndarray:
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1 - R[0, 0] + R[1, 1] - R[2, 2]) * 2
            w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1 - R[0, 0] - R[1, 1] + R[2, 2]) * 2
            w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def init_from_grasp(arm: str, grasp: np.ndarray) -> np.ndarray:
    """grasp = [x,y,z,qx,qy,qz,qw] in stand frame -> init TCP in stand frame."""
    t_ig, q_ig = _INIT_IN_GRASP[arm]
    Rg = quat_to_R(grasp[3:7])
    pg = grasp[:3]
    p_init = pg + Rg @ t_ig
    R_init = Rg @ quat_to_R(q_ig)
    return np.concatenate([p_init, R_to_quat(R_init)])


def _parse_pose(s: str) -> np.ndarray:
    v = np.array([float(x) for x in s.replace(",", " ").split()])
    if v.size != 7:
        raise argparse.ArgumentTypeError("pose must be 7 numbers: x y z qx qy qz qw")
    return v


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--left-grasp", type=_parse_pose, required=True, help="left grasp TCP (stand): x y z qx qy qz qw")
    ap.add_argument("--right-grasp", type=_parse_pose, required=True, help="right grasp TCP (stand): x y z qx qy qz qw")
    args = ap.parse_args()
    for arm, grasp in (("left", args.left_grasp), ("right", args.right_grasp)):
        init = init_from_grasp(arm, grasp)
        p, q = init[:3], init[3:7]
        print(f"{arm:>5} InitMotion TCP (stand): "
              f"x={p[0]:+.4f} y={p[1]:+.4f} z={p[2]:+.4f} | "
              f"qx={q[0]:+.4f} qy={q[1]:+.4f} qz={q[2]:+.4f} qw={q[3]:+.4f}")


if __name__ == "__main__":
    main()
