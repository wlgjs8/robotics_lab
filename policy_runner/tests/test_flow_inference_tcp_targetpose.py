from __future__ import annotations

import io
import json
import time
import unittest

from policy_runner.flow_inference import resolve_flow_policy_dt_sec
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.rollout_modes import RolloutMode
from policy_runner.servo_command_client import CommandIntent

# 로컬 학습 체크포인트(.pt)로 FlowMatchingActionSource 를 직접 만들어 델타 -> 절대
# TcpPoseTarget 합성 / 청크 경계 리앵커 / foh_se3 조건화를 돌리던 테스트들은
# 사내 학습 스택(flow_model, flow_training)과 함께 제거되었다. 배포 경로는
# openpi:// 원격 정책뿐이라 만들 수 있는 로컬 체크포인트가 없다. SE(3) 조건화
# 자체는 test_umi_dual_cartesian.py 의 OnlineTcpPoseTargetConditioner 커버리지가
# 남아 있다.


class FlowInferenceTcpPoseTargetTest(unittest.TestCase):
    def test_tcp_target_pose_policy_dt_resolves_without_family_optin(self) -> None:
        self.assertEqual(
            resolve_flow_policy_dt_sec(
                RolloutMode.SIM_DRYRUN,
                policy_dt_sec=None,
                command_rate_hz=100.0,
                dataset_stats={"dt_mean_sec": 0.02},
            ),
            0.02,
        )

    def test_final_intent_action_log_records_composed_command(self) -> None:
        from policy_runner.flow_inference import FlowMatchingActionSource

        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        source._action_log = io.StringIO()
        source._action_log_seq = 7
        source.command_family = "TcpPoseTarget"
        override = type("Override", (), {"left_on": True, "right_on": False})()
        snapshot = StateSnapshot(
            payload={"command_source": {"active_source_id": "flow", "active_session_id": "sess"}},
            received_monotonic=time.monotonic(),
        )
        intent = CommandIntent(
            "TcpPoseTarget",
            left={
                "mode": "JointTarget",
                "q_target_deg": [1, 2, 3, 4, 5, 6],
                "joint_target_profile": "init_motion",
            },
            right={
                "mode": "TcpPoseTarget",
                "tcp_target_stand": [0.4, 0.5, 0.6, 0, 0, 0],
            },
        )

        source.log_final_intent(
            intent,
            snapshot,
            arm_init_override=override,
            decision_allowed=True,
            sent=True,
            command_seq=10023,
        )

        record = json.loads(source._action_log.getvalue())
        self.assertEqual(record["event"], "final_intent")
        self.assertEqual(record["seq"], 7)
        self.assertEqual(record["command_seq"], 10023)
        self.assertEqual(record["left_mode"], "JointTarget")
        self.assertEqual(record["left_joint_target_profile"], "init_motion")
        self.assertEqual(record["right_mode"], "TcpPoseTarget")
        self.assertTrue(record["arm_init_override_left"])
        self.assertFalse(record["arm_init_override_right"])
        self.assertTrue(record["sent"])
        self.assertTrue(record["decision_allowed"])
        self.assertEqual(record["command_source_id"], "flow")
        self.assertEqual(record["command_session_id"], "sess")


if __name__ == "__main__":
    unittest.main()
