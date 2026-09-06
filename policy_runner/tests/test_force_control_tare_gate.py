"""A rollout must not run compliance that has no F/T bias behind it.

servo_log_20260906_195814 (517 s pi0.5 rollout) is the case this closes. force_control
was enabled on both arms and never tared, so `covered` flickered on
`coverage_reason = "F/T has no bias yet - run a tare before enabling compliance"`. It
read covered on 93.6 % of ticks, and `gripper_gripper.exclude_when_force_covered` hands
the two grippers to the F/T sensor whenever coverage is on -- so the barrier was off
while a sensor that could not feel anything was nominally holding the contact. The
grippers closed to -6 mm with it off; when coverage dropped, the restored 25 mm floor
could only hold them there (clamp_hold stops further closing and never pushes out) and
the model ran to -48.7 mm.
"""
from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest import mock

from policy_runner.flow_inference import FlowMatchingActionSource


def _arm(*, fc_enabled, tare_state="accepted", bias_valid=True):
    return {
        "force_control": {"enabled": fc_enabled},
        "force_torque": {"tare_state": tare_state, "bias_valid": bias_valid},
    }


def _payload(left, right):
    return {"left": left, "right": right}


class ForceControlTareGateTest(unittest.TestCase):
    def setUp(self):
        self.source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)

    def check(self, payload):
        self.source._require_force_control_tare(payload)

    def test_accepted_tare_on_both_arms_passes(self):
        self.check(_payload(_arm(fc_enabled=True), _arm(fc_enabled=True)))

    def test_compliance_off_needs_no_tare(self):
        """Nothing is handed to the sensor, so an untared sensor is not a hazard."""
        self.check(_payload(
            _arm(fc_enabled=False, tare_state="none", bias_valid=False),
            _arm(fc_enabled=False, tare_state="none", bias_valid=False),
        ))

    def test_enabled_without_a_tare_is_refused(self):
        for state, bias in (("none", False), ("settling", False), ("rejected", False),
                            ("accepted", False), ("none", True)):
            with self.subTest(tare_state=state, bias_valid=bias):
                with self.assertRaisesRegex(ValueError, "accepted F/T tare"):
                    self.check(_payload(
                        _arm(fc_enabled=True, tare_state=state, bias_valid=bias),
                        _arm(fc_enabled=True),
                    ))

    def test_one_untared_arm_is_enough_to_refuse(self):
        with self.assertRaisesRegex(ValueError, r"right: tare_state=none"):
            self.check(_payload(
                _arm(fc_enabled=True),
                _arm(fc_enabled=True, tare_state="none", bias_valid=False),
            ))

    def test_missing_force_torque_block_is_refused_not_ignored(self):
        """An old server that publishes no F/T telemetry must not read as tared."""
        with self.assertRaisesRegex(ValueError, "accepted F/T tare"):
            self.check(_payload({"force_control": {"enabled": True}}, _arm(fc_enabled=True)))

    def test_non_dict_payload_is_a_no_op(self):
        self.check(None)
        self.check({})
        self.check({"left": None, "right": 3})

    def test_next_intent_refuses_before_any_policy_work(self):
        """The gate runs on every snapshot, ahead of inference and of any command."""
        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        source.tcp_target_profile = "flow_infer_smooth"
        payload = _payload(
            _arm(fc_enabled=True, tare_state="none", bias_valid=False),
            _arm(fc_enabled=True),
        )
        with mock.patch.object(FlowMatchingActionSource, "_before_policy_intent") as before:
            with self.assertRaisesRegex(ValueError, "Refusing policy inference and motion"):
                source.next_intent(SimpleNamespace(payload=payload), 1.0)
            before.assert_not_called()


if __name__ == "__main__":
    unittest.main()
