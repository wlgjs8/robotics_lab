"""Velocity-proprio (--proprio-mode velocity / velocity_grip) for OpenpiRemoteActionSource.

Locks the deploy-side contract for the openpi velproprio checkpoints (openpi convert
--state-mode velocity / velocity_grip): the client must emit ee_local VELOCITY proprio
(finite-differenced from the robot TCP pose) instead of the default 14-D reset-relative
pose, matching the converter's `_arm_velocity` (cur^-1 . (p_next - p_cur), cur^-1 . R_next).
Guarded by torch availability (OpenpiRemoteActionSource imports flow_inference -> torch).
"""

from __future__ import annotations

import unittest

try:
    import numpy as np
    from scipy.spatial.transform import Rotation

    from policy_runner.flow_inference import resolve_ee_local_r_align
    from policy_runner.openpi_remote import OpenpiRemoteActionSource
except Exception:  # torch (a transitive import) may be absent
    np = None
    OpenpiRemoteActionSource = None  # type: ignore[assignment]
    resolve_ee_local_r_align = None  # type: ignore[assignment]


def _arm(pose):
    x, y, z, qx, qy, qz, qw = pose
    return {"tcp_stand": {"x": x, "y": y, "z": z, "quaternion_xyzw": [qx, qy, qz, qw]}}


def _payload(left_pose, right_pose):
    return {"left": _arm(left_pose), "right": _arm(right_pose)}


_IDENT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


@unittest.skipIf(OpenpiRemoteActionSource is None, "torch is not installed")
class VelocityProprioTest(unittest.TestCase):
    def _make_source(self, *, proprio_mode: str, r_align=None):
        src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        src.proprio_mode = proprio_mode
        src._state_dim = {"velocity": 12, "velocity_grav": 20}.get(proprio_mode, 14)
        src.ee_local_r_align = resolve_ee_local_r_align(r_align)
        # Huge policy_dt -> scale clamps to 1.0, so the raw one-step delta is returned
        # unscaled and we can assert the exact ee_local formula.
        src.policy_dt_sec = 1.0e9
        src._vel_prev_pose_by_arm = {"left": None, "right": None}
        src._vel_prev_sample_t = None
        src._live_gripper_percent = lambda side: None  # force the gripper fallback (-> 0.0)
        return src

    def test_state_dims(self) -> None:
        for mode, dim in (("velocity", 12), ("velocity_grip", 14), ("velocity_grav", 20)):
            src = self._make_source(proprio_mode=mode)
            out = src._proprio_state(_payload(_IDENT, _IDENT))
            self.assertEqual(out.shape, (dim,), mode)
            self.assertEqual(out.dtype, np.float32)

    def test_first_sample_is_zero_velocity(self) -> None:
        # No previous pose -> velocity 0 (matches the converter's vel[0]=0 / segment-start zeroing).
        src = self._make_source(proprio_mode="velocity")
        out = src._proprio_state(_payload([0.1, 0.2, 0.3] + [0, 0, 0, 1], _IDENT))
        np.testing.assert_allclose(out, np.zeros(12), atol=1e-7)

    def test_translation_velocity(self) -> None:
        src = self._make_source(proprio_mode="velocity")
        src._proprio_state(_payload(_IDENT, _IDENT))  # latch prev
        out = src._proprio_state(_payload([0.01, 0.02, 0.03] + [0, 0, 0, 1], _IDENT))
        # Identity orientation -> ee_local pos_vel == world delta; rot_vel 0; right arm static.
        np.testing.assert_allclose(out[:6], [0.01, 0.02, 0.03, 0, 0, 0], atol=1e-6)
        np.testing.assert_allclose(out[6:], np.zeros(6), atol=1e-6)

    def test_r_align_flips_xy_translation(self) -> None:
        src = self._make_source(proprio_mode="velocity", r_align="pika_rz180")
        src._proprio_state(_payload(_IDENT, _IDENT))
        out = src._proprio_state(_payload([0.01, 0.02, 0.03] + [0, 0, 0, 1], _IDENT))
        # pika_rz180 = diag(-1,-1,1) -> x,y flipped, z kept (on both linear and angular).
        np.testing.assert_allclose(out[:6], [-0.01, -0.02, 0.03, 0, 0, 0], atol=1e-6)

    def test_rotation_velocity_about_z(self) -> None:
        src = self._make_source(proprio_mode="velocity")
        src._proprio_state(_payload(_IDENT, _IDENT))
        theta = 0.2
        cur = [0, 0, 0, 0.0, 0.0, np.sin(theta / 2), np.cos(theta / 2)]
        out = src._proprio_state(_payload(cur, _IDENT))
        # ee_local rot_vel = rotvec(R_prev^-1 . R_cur) = [0,0,theta]; no translation.
        np.testing.assert_allclose(out[:3], [0, 0, 0], atol=1e-6)
        np.testing.assert_allclose(out[3:6], [0, 0, theta], atol=1e-6)

    def test_velocity_grip_keeps_gripper_dim(self) -> None:
        src = self._make_source(proprio_mode="velocity_grip")
        out = src._proprio_state(_payload(_IDENT, _IDENT))
        self.assertEqual(out.shape, (14,))
        # Gripper dims (6, 13) present; no gripper in payload + no live percent -> 0.0.
        self.assertEqual(out[6], 0.0)
        self.assertEqual(out[13], 0.0)


    def test_velocity_grav_layout_and_gravity(self) -> None:
        # velocity_grav per arm = [pos_vel(3), rot_vel(3), gravity(3), grip(1)] = 10; dual = 20.
        # gravity = world-down [0,0,-1] expressed in the tool frame (R_tcp^-1 . down); for an
        # IDENTITY tool orientation that is exactly [0,0,-1], a UNIT vector.
        src = self._make_source(proprio_mode="velocity_grav")
        out = src._proprio_state(_payload(_IDENT, _IDENT))
        self.assertEqual(out.shape, (20,))
        np.testing.assert_allclose(out[6:9], [0.0, 0.0, -1.0], atol=1e-6)    # left gravity
        np.testing.assert_allclose(out[16:19], [0.0, 0.0, -1.0], atol=1e-6)  # right gravity
        np.testing.assert_allclose(np.linalg.norm(out[6:9]), 1.0, atol=1e-6)
        self.assertEqual(out[9], 0.0)   # left grip (no payload/live -> 0)
        self.assertEqual(out[19], 0.0)  # right grip

    def test_velocity_grav_tilt_changes_gravity(self) -> None:
        # A roll/pitch tilts the in-frame gravity away from [0,0,-1] (the anchor SEES tilt ->
        # targets the #1 over-tilt). g = R^-1 . [0,0,-1]; assert against the direct scipy value.
        src = self._make_source(proprio_mode="velocity_grav")
        pitch = [0, 0, 0, float(np.sin(0.5)), 0.0, 0.0, float(np.cos(0.5))]  # 1.0 rad about tool X
        out = src._proprio_state(_payload(pitch, _IDENT))
        expected = Rotation.from_quat(pitch[3:7]).inv().apply([0.0, 0.0, -1.0])
        np.testing.assert_allclose(out[6:9], expected, atol=1e-6)

    def test_velocity_grav_yaw_invariant(self) -> None:
        # Rotating the tool about world-Z (stand up = heading) leaves the in-frame gravity
        # UNCHANGED -> the anchor never encodes the unmeasured steamvr->stand heading.
        src = self._make_source(proprio_mode="velocity_grav")
        yaw = [0, 0, 0, 0.0, 0.0, float(np.sin(0.5)), float(np.cos(0.5))]  # 1.0 rad about Z
        out = src._proprio_state(_payload(yaw, _IDENT))
        np.testing.assert_allclose(out[6:9], [0.0, 0.0, -1.0], atol=1e-6)


@unittest.skipIf(OpenpiRemoteActionSource is None, "torch is not installed")
class VelocityProprioFixedStepTest(unittest.TestCase):
    """velproprio_sample_mode='fixed_step': velocity from a fixed ~policy_dt window taken from
    the per-tick pose history, decoupled from replan cadence / inference latency."""

    def _make_source(self, *, proprio_mode: str = "velocity", policy_dt: float = 1.0 / 30.0):
        from collections import deque

        src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        src.proprio_mode = proprio_mode
        src._state_dim = {"velocity": 12, "velocity_grav": 20}.get(proprio_mode, 14)
        src.ee_local_r_align = resolve_ee_local_r_align(None)
        src.policy_dt_sec = policy_dt
        src.velproprio_sample_mode = "fixed_step"
        src._pose_history = {"left": deque(maxlen=512), "right": deque(maxlen=512)}
        src._last_now_monotonic = None
        src._vel_prev_pose_by_arm = {"left": None, "right": None}
        src._vel_prev_sample_t = None
        src._live_gripper_percent = lambda side: None
        return src

    def _push(self, src, side, t, pose):
        src._pose_history[side].append((float(t), np.asarray(pose, dtype=np.float64)))

    def test_translation_from_fixed_window(self) -> None:
        src = self._make_source()
        dt = src.policy_dt_sec
        cur = [0.01, 0.02, 0.03, 0, 0, 0, 1]
        # prev sample exactly one policy step old; current tick recorded at t=dt.
        self._push(src, "left", 0.0, _IDENT)
        self._push(src, "right", 0.0, _IDENT)
        self._push(src, "left", dt, cur)
        self._push(src, "right", dt, _IDENT)
        src._last_now_monotonic = dt
        out = src._proprio_state(_payload(cur, _IDENT))
        # window == policy_dt -> scale 1 -> raw ee_local delta; right arm static.
        np.testing.assert_allclose(out[:6], [0.01, 0.02, 0.03, 0, 0, 0], atol=1e-6)
        np.testing.assert_allclose(out[6:], np.zeros(6), atol=1e-6)

    def test_cold_start_is_zero(self) -> None:
        # Only the current tick in history -> buffer does not span a full policy step -> vel 0.
        src = self._make_source()
        cur = [0.05, 0.0, 0.0, 0, 0, 0, 1]
        self._push(src, "left", 0.0, cur)
        self._push(src, "right", 0.0, _IDENT)
        src._last_now_monotonic = 0.0
        out = src._proprio_state(_payload(cur, _IDENT))
        np.testing.assert_allclose(out, np.zeros(12), atol=1e-7)

    def test_window_normalized_to_one_policy_step(self) -> None:
        # prev sample 2*policy_dt old -> actual window 2*dt -> scale 0.5 halves the raw delta,
        # so the reported magnitude is per-policy-step regardless of the actual sample spacing.
        src = self._make_source()
        dt = src.policy_dt_sec
        self._push(src, "left", 0.0, _IDENT)
        self._push(src, "right", 0.0, _IDENT)
        src._last_now_monotonic = 2.0 * dt
        cur = [0.02, 0.0, 0.0, 0, 0, 0, 1]
        out = src._proprio_state(_payload(cur, _IDENT))
        np.testing.assert_allclose(out[:3], [0.01, 0.0, 0.0], atol=1e-6)

    def test_independent_of_inference_latency(self) -> None:
        # The SAME measured window must yield the SAME velocity no matter how much wall-clock
        # elapsed at the replan (the replan path would shrink it via policy_dt/wall_dt). Here the
        # current tick lands far in wall-time but the fixed window is anchored to `now`.
        src = self._make_source()
        dt = src.policy_dt_sec
        cur = [0.0, 0.008, 0.0, 0, 0, 0, 1]
        # A long idle gap then two samples one step apart around `now` = 5.0.
        self._push(src, "left", 5.0 - dt, _IDENT)
        self._push(src, "right", 5.0 - dt, _IDENT)
        self._push(src, "left", 5.0, cur)
        self._push(src, "right", 5.0, _IDENT)
        src._last_now_monotonic = 5.0
        out = src._proprio_state(_payload(cur, _IDENT))
        np.testing.assert_allclose(out[:6], [0.0, 0.008, 0.0, 0, 0, 0], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
