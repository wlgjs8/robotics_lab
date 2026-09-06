"""Offline CLI/clock regressions; explicitly registered only with experiments ON."""
import copy
import csv
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import test_delta_follower_replay as recorded

ROOT = Path(__file__).resolve().parents[2]
BIN = Path(os.environ['RB_PREVIEW_REPLAY_BINARY'])
STACK = ROOT / 'rb_servo_server/config/stack_real.yaml'


def parameters():
    return {
        'schema': 'robotics_lab.preview_follower_replay.v1',
        'replan_period_sec': .01, 'preview_servo_period_sec': .002,
        'initial_state_policy': 'recorded_previous_emitted_nominal_at_rest',
        'tracker': {
            'planning_dt_sec': .01, 'horizon_steps': 24,
            'max_linear_velocity_m_s': .6, 'max_linear_acceleration_m_s2': 12.,
            'max_linear_jerk_m_s3': 2000., 'max_angular_velocity_rad_s': 1.4,
            'max_angular_acceleration_rad_s2': 40., 'max_angular_jerk_rad_s3': 4000.,
            'linear_tracking_scale_m': .01, 'angular_tracking_scale_rad': .03,
            'jerk_weight': 20., 'jerk_difference_weight': .01,
            'linear_tracking_tolerance_m': .02, 'angular_tracking_tolerance_rad': .08,
            'max_linear_tracking_slack_m': .08, 'max_angular_tracking_slack_rad': .25,
            'max_reference_chart_angle_rad': 1., 'feasibility_tolerance': 1e-7,
            'max_working_set_recalculations': 300, 'max_solve_time_sec': .5,
        },
    }


class PreviewReplayChecks(unittest.TestCase):
    def replay(self, events, params=None, options=(), phase=None):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            source, config, output = (directory / name for name in ('events.jsonl', 'params.json', 'out.csv'))
            source.write_text(''.join(json.dumps(e) + '\n' for e in events))
            config.write_text(json.dumps(parameters() if params is None else params))
            if phase is not None:
                phase_path=directory/'phase.json'
                phase_path.write_text(json.dumps(phase))
                options=(*options,'--execution-cursor',str(phase_path))
            result = subprocess.run([str(BIN), str(STACK), 'flow_infer_fresh', str(config),
                                     str(source), str(output), *options],
                                    cwd=ROOT, capture_output=True, text=True, timeout=15)
            with output.open() if output.exists() else open(os.devnull) as stream:
                rows = list(csv.DictReader(stream))
            summary_path = Path(str(output) + '.summary.json')
            summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
            return result, rows, summary

    def test_fresh_splice_and_cold_resume_keep_output_state(self):
        events = recorded.RecordedConsumerReplayTest.events()
        result, rows, summary = self.replay(events)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertEqual(len(rows), 38)
        self.assertEqual(summary['consumed_frames'], 4)
        seeds = [row for row in rows if row['reseeded'] == '1']
        self.assertEqual([int(row['tick']) for row in seeds], [1000000000, 1044000000])
        for row in seeds:
            self.assertEqual(float(row['stage_x']), .2)
            self.assertTrue(all(float(row[f'stage_v_{i}']) == 0 for i in range(6)))
            self.assertTrue(all(float(row[f'stage_a_{i}']) == 0 for i in range(6)))
        for row in rows:
            self.assertEqual(row['clone_live_mutated'], '0')
            for name in ('clone_zero_pos_error', 'clone_zero_rot_error', 'splice_position_error',
                         'splice_rotation_error', 'splice_velocity_error', 'splice_acceleration_error'):
                self.assertLess(abs(float(row[name])), 1e-9)
        self.assertGreater(float(rows[10]['stage_x']), float(rows[0]['stage_x']))

    def test_expired_trajectory_is_an_error_without_raw_fallback(self):
        events = recorded.RecordedConsumerReplayTest.events()[:2]
        events[1].update(t=.3, mono=1.3, dt=.3, tick=1300000000)
        result, rows, summary = self.replay(events)
        self.assertEqual(result.returncode, 2, result.stderr + result.stdout)
        self.assertEqual(summary['termination_reason'], 'accepted_plan_expired')
        self.assertEqual(len(rows), 1)

    def test_missing_parameters_empty_stream_and_no_active_rows_are_rejected(self):
        params = parameters()
        del params['tracker']['max_linear_jerk_m_s3']
        result, _, _ = self.replay(recorded.RecordedConsumerReplayTest.events()[:1], params)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('missing/extra fields', result.stderr)
        for events in ([], [dict(copy.deepcopy(recorded.RecordedConsumerReplayTest.events()[0]), active=False)]):
            result, rows, _ = self.replay(events)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(rows, [])

    @staticmethod
    def phase_parameters():
        return {'schema':'robotics_lab.preview_execution_cursor.v1',
                'max_backlog_sec':.1,'catchup_time_sec':.2,'max_rate':1.1,
                'translation_velocity_floor':1e-6,'angular_velocity_floor':1e-6}

    def test_execution_cursor_preserves_canonical_reference_across_fresh_and_reset(self):
        events=recorded.RecordedConsumerReplayTest.events()
        base, baseline, _=self.replay(events, options=('--reference-only',))
        result, rows, summary=self.replay(events, phase=self.phase_parameters())
        self.assertEqual(base.returncode,0,base.stderr)
        self.assertEqual(result.returncode,0,result.stderr+result.stdout)
        self.assertTrue(summary['execution_cursor'])
        self.assertEqual(len(rows),len(baseline))
        for row, ref in zip(rows,baseline,strict=True):
            for axis in ('x','y','z','qx','qy','qz','qw'):
                self.assertEqual(row['raw_'+axis],ref['raw_'+axis])
            self.assertGreaterEqual(float(row['cursor_backlog_sec']),0.)
            self.assertLessEqual(float(row['cursor_backlog_sec']),.1)
            self.assertLessEqual(float(row['cursor_time']),float(row['mono']))
            self.assertEqual(row['clone_live_mutated'],'0')
            if row['reseeded']=='1':
                self.assertEqual(float(row['cursor_backlog_sec']),0.)
        self.assertGreater(max(int(row['preview_count']) for row in rows),32)

    def test_cursor_options_and_explicit_phase_bounds(self):
        events=recorded.RecordedConsumerReplayTest.events()[:1]
        result, _, _=self.replay(events,options=('--own-stage-leash',),phase=self.phase_parameters())
        self.assertNotEqual(result.returncode,0)
        self.assertIn('mutually exclusive',result.stderr)
        phase=self.phase_parameters();phase['max_backlog_sec']=0
        result, _, _=self.replay(events,phase=phase)
        self.assertNotEqual(result.returncode,0)


if __name__ == '__main__':
    unittest.main()
