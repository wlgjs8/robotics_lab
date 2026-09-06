"""Strict recorded-consumer export fixtures; requires offline pandas/pyarrow."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
try:
    import pandas as pd
    import pyarrow  # Required by Feather read/write, not by the servo runtime.
except ImportError:
    pd = None

ROOT = Path(__file__).resolve().parents[2]
if pd is not None:
    spec = importlib.util.spec_from_file_location('exporter', ROOT/'tools/prepare_recorded_follower_replay.py')
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)


@unittest.skipUnless(pd is not None, 'recorded Feather analysis requires optional pandas and pyarrow; run in the analysis/OpenPI Python environment')
class RecordedFollowerExportTest(unittest.TestCase):
    def fixture(self):
        rows = []
        for i in range(6):
            r = dict.fromkeys(exporter.required_columns(), 0.)
            r.update(loop_start_time_ns=998000000+2000000*i, filter_dt_ms=2.,
                     chunk_frame_wire_seq=37, chunk_frame_recv_seq=901,
                     chunk_frame_policy_dt_sec=.0334, chunk_frame_age_ms=1.)
            for arm in exporter.SIDES:
                r.update({arm+'_mode': 'TcpPoseTarget', arm+'_tcp_target_profile':'flow_infer_fresh',
                          arm+'_follower_controller':'delta_preview', arm+'_follower_active':1,
                          arm+'_follower_wire_seq':37, arm+'_follower_recv_seq':901,
                          arm+'_follower_plan_rate_gate':1.,arm+'_follower_advance_gate':1.})
                for stem in ['tcp_command_stand','tcp_actual_stand','follower_prefilter_stand','stage_tcp_target_stand','follower_pf_stand']:
                    r[arm+'_'+stem+'_qw'] = 1.
                r[arm+'_tcp_command_stand_x_m'] = i*.01
            rows.append(r)
        c = {'seq':37, 'chunk_id':99999, 't_mono':1., 'policy_dt_sec':.0334,
             'chunk_metadata':{'tcp_target_profile':'flow_infer_fresh','activation_mode':'ready_event',
                               'proprio':{'valid':True},'generation':2,'activation_step_seq':3,
                               'observation_step_seq':0,'source_start_index':3}}
        for arm in exporter.SIDES:
            c[arm+'_delta'] = [[.001,0,0,0,0,0,20] for _ in range(8)]
            c[arm] = [[0,0,0,0,0,0,1,20] for _ in range(8)]
        return pd.DataFrame(rows), [c]

    def run_export(self, d, chunks, path):
        return exporter.export_frame(d, chunks, 1.008, path)

    def test_receiver_wire_join_ignores_unrelated_policy_chunk_id(self):
        d,c = self.fixture()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp); m = self.run_export(d,c,path)
            e = [json.loads(x) for x in (path/'left.events.jsonl').read_text().splitlines()]
            self.assertEqual(e[0]['frame']['wire_seq'],37)
            self.assertEqual(e[0]['frame']['recv_seq'],901)
            self.assertEqual(m['arms']['left']['consumed_frames'],1)
            self.assertEqual(e[0]['previous_emitted'][0],0.)

    def test_live_spectral_comparison_includes_terminal_tail(self):
        spec=importlib.util.spec_from_file_location('replay_analyzer',ROOT/'tools/analyze_recorded_follower_replay.py')
        analyzer=importlib.util.module_from_spec(spec);spec.loader.exec_module(analyzer)
        np=analyzer.np;t=np.arange(0.,11.36,.002)
        d=pd.DataFrame({'t':t})
        for axis in ['x','y','z','qx','qy','qz']:
            d['stage_'+axis]=0.
        d['stage_qw']=1.
        quiet=analyzer.live_motion_bands(d)
        # A normal 4096-sample Welch call on this trace drops everything after
        # 8.19s. Only the terminal second changes, at a known in-band frequency.
        d['stage_x']=np.where(t>=10.,.001*np.sin(2*np.pi*5*(t-10.)),0.)
        tail=analyzer.live_motion_bands(d)
        self.assertTrue(tail['tail_window_added'])
        self.assertEqual(tail['window_count'],2)
        self.assertGreater(tail['covered_end_t_sec'],11.35)
        self.assertEqual(quiet['bands']['3-8']['linear_position_rms_mm'],0.)
        self.assertGreater(tail['bands']['3-8']['linear_position_rms_mm'],.001)

    def test_current_strip_and_previous_command_rows_are_not_shifted(self):
        d,c = self.fixture()
        d.loc[2,'left_fc_reference_strip_enabled']=1
        d.loc[2,'left_fc_reference_dev_x_m']=.003
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp);self.run_export(d,c,path)
            e = [json.loads(x) for x in (path/'left.events.jsonl').read_text().splitlines()]
            self.assertEqual(e[1]['previous_emitted'][0],.01)
            self.assertEqual(e[1]['reference_deviation'][0],.003)
            self.assertTrue(e[1]['reference_strip_enabled'])
            self.assertFalse(e[0]['reference_strip_enabled'])

    def test_init_mode_overrides_stale_active_and_resends_on_resume(self):
        d,c = self.fixture()
        d.loc[2:3,'left_mode']='JointTarget'
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp);m=self.run_export(d,c,path)
            e = [json.loads(x) for x in (path/'left.events.jsonl').read_text().splitlines()]
            self.assertFalse(e[1]['active']);self.assertFalse(e[2]['active'])
            self.assertNotIn('frame',e[1])
            self.assertIn('frame',e[3])
            self.assertEqual(m['arms']['left']['cold_starts'],2)

    def test_missing_or_misidentified_consumption_is_rejected(self):
        def future_chunk(d,c):
            second = copy.deepcopy(c[0]);second.update(seq=38,t_mono=1.006);c.append(second)
            d.loc[2:,'chunk_frame_wire_seq']=38
            d.loc[2:,'left_follower_wire_seq']=38
            d.loc[2:,'chunk_frame_recv_seq']=902
            d.loc[2:,'left_follower_recv_seq']=902
        for mutation, message in [
            (lambda d,c: d.drop(columns=['left_fc_reference_dev_x_m'],inplace=True),'missing recorded'),
            (lambda d,c: d.__setitem__('chunk_frame_recv_seq',902),'consumed/cache'),
            (lambda d,c: c[0].update(seq=38),'missing from producer'),
            (future_chunk,'publication'),
            (lambda d,c: c[0]['left_delta'][0].__setitem__(6,21),'gripper'),
            (lambda d,c: c[0]['left_delta'][0].__setitem__(0,float('nan')),'nonfinite'),
            (lambda d,c: d.__setitem__('left_fc_folded',1),'unsupported'),
            (lambda d,c: d.loc.__setitem__((2,'left_follower_warm_resume_count'),1),'unsupported')]:
            with self.subTest(message=message), tempfile.TemporaryDirectory() as tmp:
                d,c=self.fixture();mutation(d,c)
                with self.assertRaisesRegex(ValueError,message):
                    self.run_export(d,c,Path(tmp))

    def diagnostic_fixture(self):
        d,c=self.fixture()
        for col in exporter.diagnostic_columns():
            if col not in d:
                d[col]=0.
        for arm in exporter.SIDES:
            d[arm+'_tcp_target_profile']='flow_infer_preview'
            d[arm+'_fc_gate_translation']=1.
            d[arm+'_preview_execution_status']='active'
            c[0]['chunk_metadata']['tcp_target_profile']='flow_infer_preview'
        d['safety_verdict']='Ok'
        d['projection_min_headroom_class']='self'
        d['projection_min_headroom_pair']='left_link3 <-> stand'
        return d,c

    def test_reconstructed_fold_uses_vector_pose_and_post_overlay_same_row(self):
        d,c=self.diagnostic_fixture()
        # Achieved x=.02, overlay=.003, emitted=.0169 -> dp=.0001.
        d.loc[2,'left_fc_dev_x_m']=.003
        d.loc[2,'left_fc_reference_dev_x_m']=.004  # Must NOT strip the old gauge.
        d.loc[2,'left_stage_tcp_target_stand_x_m']=.0169
        d.loc[2,'left_hold_fold_m']=.0001
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            m=exporter.export_frame(d,c,1.008,p,'flow_infer_preview',reconstruct_geometry=True)
            e=[json.loads(x) for x in (p/'left.events.jsonl').read_text().splitlines()]
            self.assertEqual(e[1]['schema'],exporter.DIAGNOSTIC_SCHEMA)
            fold=e[1]['booked_hold_fold']
            self.assertAlmostEqual(fold['translation_m'][0],.0001,places=12)
            self.assertEqual(fold['booked_tick'],1002000000)
            self.assertEqual(fold['apply_order'],'next_tick_after_raw_before_preview')
            self.assertFalse(fold['rotation_independently_verified'])
            self.assertEqual(m['arms']['left']['reconstructed_hold_folds'],1)
            self.assertNotIn('booked_hold_fold',e[0])
            self.assertNotIn('booked_hold_fold',e[2])

    def test_reconstruction_rejects_scalar_only_or_wrong_row_fold(self):
        d,c=self.diagnostic_fixture()
        d.loc[2,'left_hold_fold_m']=.0001
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError,'magnitude reconstruction residual'):
                exporter.export_frame(d,c,1.008,Path(tmp),'flow_infer_preview',reconstruct_geometry=True)

    def test_reconstructed_rotation_uses_quaternion_and_current_overlay(self):
        from scipy.spatial.transform import Rotation
        import numpy as np
        d,c=self.diagnostic_fixture()
        emitted=Rotation.from_rotvec([.2,-.1,.3])
        delta=Rotation.from_rotvec([.01,.02,-.03])
        overlay=Rotation.from_rotvec([-.03,.04,.02])
        composed=overlay*delta*emitted
        d.loc[2,'left_stage_tcp_target_stand_x_m']=.0199
        d.loc[2,'left_hold_fold_m']=.0001
        for axis,value in zip(['qx','qy','qz','qw'],emitted.as_quat()):
            d.loc[2,'left_stage_tcp_target_stand_'+axis]=value
        for axis,value in zip(['qx','qy','qz','qw'],composed.as_quat()):
            d.loc[2,'left_tcp_command_stand_'+axis]=value
        for axis,value in zip(['rx_rad','ry_rad','rz_rad'],overlay.as_rotvec()):
            d.loc[2,'left_fc_dev_'+axis]=value
        fold=exporter.reconstruct_hold_fold(d.iloc[2],'left')
        residual=(Rotation.from_quat(fold['rotation_xyzw']).inv()*delta).magnitude()
        self.assertLess(residual,1e-12)
        self.assertFalse(fold['rotation_independently_verified'])
        np.testing.assert_allclose(fold['translation_m'],[.0001,0,0],atol=1e-12)

    def test_contact_export_preserves_unarmed_tick_gate_and_exact_zero_threshold(self):
        d,c=self.diagnostic_fixture()
        d.loc[2,'left_fc_covered']=1
        d.loc[2,'left_fc_gate_translation']=.5
        d.loc[2,'left_fc_wrench_filt_fx_n']=1e-10
        first=exporter.contact_event(d.iloc[2], 'left')
        self.assertEqual(first['gate'],.5)
        self.assertFalse(first['stream_armed'])
        self.assertEqual(first['normal_into_stand'],[0.,0.,0.])
        d.loc[2,'left_fc_wrench_filt_fx_n']=2e-9
        self.assertEqual(exporter.contact_event(d.iloc[2],'left')['normal_into_stand'],[-1.,0.,0.])

    def test_strict_v2_can_export_aligned_contact_without_accepting_geometry(self):
        d,c=self.diagnostic_fixture()
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            exporter.export_frame(d,c,1.008,p,'flow_infer_preview',include_contact=True)
            events=[json.loads(x) for x in (p/'left.events.jsonl').read_text().splitlines()]
            contact=[json.loads(x) for x in (p/'left.contact.jsonl').read_text().splitlines()]
            self.assertEqual([x['tick'] for x in events],[x['tick'] for x in contact])
            self.assertTrue(all(x['schema']==exporter.SCHEMA for x in events))
            d.loc[2,'left_hold_fold_m']=.0001
            with self.assertRaisesRegex(ValueError,'unsupported active hold_fold_m'):
                exporter.export_frame(d,c,1.008,p,'flow_infer_preview',include_contact=True)

    def test_missing_preview_stage_is_labeled_and_never_hides_active_missing_pose(self):
        d,c=self.diagnostic_fixture()
        d.loc[1,'left_stage_tcp_target_stand_x_m']=float('nan')
        d.loc[1,'left_preview_execution_status']='waiting'
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            exporter.export_frame(d,c,1.008,p,'flow_infer_preview',preview_lifecycle_readback=True)
            e=json.loads((p/'left.events.jsonl').read_text().splitlines()[0])
            self.assertFalse(e['observed_stage_valid'])
            self.assertEqual(e['observed_stage_source'],'display_only_current_nominal_command_no_preview_stage')
            self.assertEqual(e['previous_emitted'][0],0.)
            self.assertEqual(e['observed_stage'][0],.01)
            d.loc[1,'left_preview_execution_status']='active'
            with self.assertRaisesRegex(ValueError,'outside explicit preview'):
                exporter.export_frame(d,c,1.008,p,'flow_infer_preview',preview_lifecycle_readback=True)

if __name__=='__main__':
    unittest.main()
