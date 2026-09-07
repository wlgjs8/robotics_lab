"""Init/tare uses the actual OpenPI/flow dispatch and runner composer; no I/O."""
from __future__ import annotations

import unittest
from unittest import mock

import numpy as np

from policy_runner.arm_init_control import (
    ARM_INIT_COMMAND_SCHEMA, ArmInitCommand, ArmInitOverrideController,
    apply_source_arm_mask, apply_source_override_transitions,
)
from policy_runner.config import config_from_mapping
from policy_runner.main import run
from policy_runner.openpi_remote import OpenpiRemoteActionSource
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.safety import ActionRequirements
from policy_runner.servo_command_client import CommandIntent
from test_policy_runner_contract import FakeArmInitControlSupervisor, FakeCommandClient, FakeTickSequenceStateClient
from test_preview_recovery_policy import RecoverySource, state


class InitSource(OpenpiRemoteActionSource, RecoverySource):
    requirements = ActionRequirements(requires_valid_joint_state=True)
    name = "flow_infer"

    def __init__(self):
        RecoverySource.__init__(self)
        self.proprio_mode = "pose"
        self.camera_checks = 0
        self._camera_runtime_terminal_abort_reason = None
        self.closed = False

    def _camera_runtime_gate(self, now):
        self.camera_checks += 1
        return False, None

    def _training_episode_completion_reason(self):
        return None

    def _before_policy_intent(self, snapshot, now):
        pass

    def _emit_step_intent(self, step, payload, gripper_targets):
        self.emitted.append(step.copy())
        arms = {arm: ({"mode": "TcpPoseTarget", "tcp_target_stand": [0,0,.3,0,0,0,1],
                       "gripper_target": 25.} if self.arm_mask[i] > 0 else {"mode": "Hold"})
                for i, arm in enumerate(("left", "right"))}
        return CommandIntent("TcpPoseTarget", left=arms['left'], right=arms['right'],
                             tcp_target_profile=self.tcp_target_profile)

    def _dispatch_gripper_step(self, step):
        self.gripper_rows.append(tuple(arm for arm in ("left", "right")
                                       if not self._arm_suspended_for_init(arm)))

    def close(self):
        self.closed = True


def snapshot(now, *, selected=(), status="idle", tare="accepted", bias=None):
    snap = state(now)
    snap.payload.update(schema_version=1, motion_state="Running", fault_latched=False,
                        init_motion={})
    for arm in ("left", "right"):
        active = arm in selected
        ts = tare if active else "accepted"
        snap.payload['init_motion'][arm] = {'status':status if active else 'idle'}
        pose = {"x":.1 if arm=='left' else .2,"y":0.,"z":.3,
                "quaternion_xyzw":[0.,0.,0.,1.]}
        snap.payload[arm] = {
            "has_valid_joint_state":True,"q_actual_deg":[0.,-30.,80.,0.,60.,0.],
            "tcp_actual_stand":pose,"tcp_command_stand":pose,
            "force_control":{"enabled":True},
            "force_torque":{"enabled":True,"connected":True,"tare_state":ts,
                "bias_valid":(ts=='accepted') if bias is None or not active else bias,
                "auto_tare_stage":'awaiting_init' if ts=='none' else 'idle'},
        }
    return snap


def step(controller, source, snap):
    controller.update_from_snapshot(snap)
    apply_source_override_transitions(source, controller.consume_transitions(), snap)
    apply_source_arm_mask(source, np.ones(2), controller)
    controller.stamp_snapshot(snap)
    source.now=snap.received_monotonic
    return controller.compose_intent(source.next_intent(snap, source.now))


class InitTareRunnerIntegrationTest(unittest.TestCase):
    def start(self, selector):
        c=ArmInitOverrideController()
        c.handle_command(ArmInitCommand(selector,'start', (0.,)*6,(0.,)*6))
        return c, InitSource()

    def test_single_and_both_init_survive_none_settling_and_resume_fresh(self):
        for selector in ('left','right','both'):
            with self.subTest(selector=selector):
                selected=('left','right') if selector=='both' else (selector,)
                c,s=self.start(selector)
                for now,status,tare in [(1.,'planning','none'),(1.1,'executing','none'),
                                        (1.2,'done','none'),(1.8,'idle','settling')]:
                    intent=step(c,s,snapshot(now,selected=selected,status=status,tare=tare))
                    for arm in selected:
                        command=getattr(intent,arm)
                        self.assertEqual(command['mode'],'JointTarget' if status in ('planning','executing') else 'Hold')
                        self.assertNotIn('gripper_target',command)
                    if selector=='both':
                        self.assertEqual(s.camera_checks,0)
                        self.assertIsNone(s._stream_request)
                generation=s._stream_generation
                step(c,s,snapshot(1.9,selected=selected,tare='accepted'))
                self.assertGreater(s._stream_generation,generation)
                self.assertFalse(c.left_on or c.right_on)
                self.assertIsNone(s._chunk)
                self.assertIsNotNone(s._stream_request)
                s.complete_request(1.91,rows=s.rows(50))
                intent=step(c,s,snapshot(1.92))
                self.assertEqual(intent.left['mode'],'TcpPoseTarget')
                self.assertEqual(intent.right['mode'],'TcpPoseTarget')
                np.testing.assert_array_equal(s.emitted[-1],np.full(14,50.))

    def test_plain_mask_or_injected_status_cannot_bypass_tare(self):
        s=InitSource();s.arm_mask=np.zeros(2)
        snap=snapshot(1.,selected=('left',),tare='none')
        snap.payload['arm_init']={'init_override_left':True,'init_override_right':True}
        with self.assertRaisesRegex(ValueError,'accepted F/T tare'):
            s.next_intent(snap,1.)
        s.set_arm_init_suspension(('left',))
        s.arm_mask=np.ones(2)
        with self.assertRaisesRegex(ValueError,'accepted F/T tare'):
            s.next_intent(snap,1.)

    def test_peer_tare_loss_is_still_rejected(self):
        c,s=self.start('left')
        snap=snapshot(1.,selected=('left','right'),status='idle',tare='none')
        with self.assertRaisesRegex(ValueError,'right: tare_state=none'):
            step(c,s,snap)

    def test_single_init_preserves_peer_stream_and_strips_cached_gripper(self):
        c,s=ArmInitOverrideController(),InitSource()
        step(c,s,snapshot(1.))
        s.complete_request(1.01,rows=s.rows(20))
        first=step(c,s,snapshot(1.02))
        self.assertIn('gripper_target',first.left)
        generation=s._stream_generation
        c.handle_command(ArmInitCommand('left','start',(0.,)*6,(0.,)*6))
        mixed=step(c,s,snapshot(1.025,selected=('left',),status='planning',tare='none'))
        self.assertEqual(mixed.left['mode'],'JointTarget')
        self.assertNotIn('gripper_target',mixed.left)
        self.assertEqual(mixed.right['mode'],'TcpPoseTarget')
        self.assertEqual(mixed.tcp_target_profile,'flow_infer_preview')
        self.assertEqual(s._stream_generation,generation)
        guarded=s._guard_preview_gripper_intent(first)
        self.assertNotIn('gripper_target',guarded.left)
        self.assertIn('gripper_target',guarded.right)

    def test_timeout_and_failed_init_stay_suspended(self):
        c,s=self.start('both')
        selected=('left','right')
        step(c,s,snapshot(1.,selected=selected,status='done',tare='none'))
        intent=step(c,s,snapshot(4.,selected=selected,tare='settling'))
        self.assertEqual(c.status_block()['left_state'],'init tare blocked')
        self.assertEqual(intent.left['mode'],'Hold')
        self.assertIsNone(s._stream_request)
        step(c,s,snapshot(4.1,selected=selected,status='failed',tare='none'))
        intent=step(c,s,snapshot(4.2,selected=selected,status='done',tare='accepted'))
        self.assertEqual(intent.left['mode'],'Hold')
        self.assertTrue(c.left_on and c.right_on)
        self.assertIsNone(s._stream_request)

    def test_external_init_never_sends_a_cancelling_policy_or_hold_packet(self):
        for selected in [('left',),('right',),('left','right')]:
            with self.subTest(selected=selected):
                c,s=ArmInitOverrideController(),InitSource()
                for now,status,tare in [(1.,'executing','none'),(1.2,'done','none'),(1.8,'idle','settling')]:
                    self.assertIsNone(step(c,s,snapshot(now,selected=selected,status=status,tare=tare)))
                    self.assertTrue(c.external_init_active)
                    self.assertIsNone(s._stream_request)
                    self.assertEqual(s.camera_checks,0)
                step(c,s,snapshot(1.9,selected=selected,tare='accepted'))
                self.assertFalse(c.external_init_active)
                self.assertIsNotNone(s._stream_request)

    def test_actual_runner_reaches_composer_through_openpi_tare_gate(self):
        for selector in ('left','right','both'):
            with self.subTest(selector=selector):
                selected=('left','right') if selector=='both' else (selector,)
                snapshots=[snapshot(t,selected=selected,status=status,tare=tare) for t,status,tare in
                           [(1.,'idle','accepted'),(1.1,'planning','none'),(1.2,'executing','none'),
                            (1.3,'done','none'),(1.8,'idle','settling'),(1.9,'idle','accepted')]]
                client=FakeTickSequenceStateClient(snapshots)
                command=FakeCommandClient();source=InitSource();captured=[]
                cfg=config_from_mapping({'schema':'robotics_lab.policy_runner.v1','geometry':{'path':''},
                    'runtime':{'startup_timeout_sec':.1},'recording':{'rate_hz':30.}})
                supervisor=FakeArmInitControlSupervisor([{'schema':ARM_INIT_COMMAND_SCHEMA,
                    'arms':selector,'action':'start','left_q_deg':[0.]*6,'right_q_deg':[0.]*6}])
                def sleep(_):
                    captured.append(list(command.sent))
                    client.advance()
                    if client.index>=len(snapshots): raise KeyboardInterrupt()
                with mock.patch.object(source,'log_final_intent',create=True):
                    result=run(cfg,state_client=client,command_client=command,source=source,
                        recording_supervisor=supervisor,monotonic_fn=lambda:client.latest.received_monotonic,
                        pacing_monotonic_fn=lambda:client.latest.received_monotonic,sleep_fn=sleep)
                self.assertEqual(result,0)
                self.assertEqual(len(captured),len(snapshots))
                self.assertTrue(source.closed)
                for idx in (1,2,3,4):
                    intent=captured[idx][-1]
                    for arm in selected:
                        self.assertEqual(getattr(intent,arm)['mode'],'JointTarget' if idx<3 else 'Hold')
                        self.assertNotIn('gripper_target',getattr(intent,arm))


if __name__=='__main__': unittest.main()
