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
        src._state_dim = 12 if proprio_mode == "velocity" else 14
        src.ee_local_r_align = resolve_ee_local_r_align(r_align)
        # Huge policy_dt -> scale clamps to 1.0, so the raw one-step delta is returned
        # unscaled and we can assert the exact ee_local formula.
        src.policy_dt_sec = 1.0e9
        src._vel_prev_pose_by_arm = {"left": None, "right": None}
        src._vel_prev_sample_t = None
        src._live_gripper_percent = lambda side: None  # force the gripper fallback (-> 0.0)
        return src

    def test_state_dims(self) -> None:
        for mode, dim in (("velocity", 12), ("velocity_grip", 14)):
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


if __name__ == "__main__":
    unittest.main()
