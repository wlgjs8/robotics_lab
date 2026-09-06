"""Exercise the offline binary without any backend/device startup."""
import csv
import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
BIN = Path(os.environ.get("RB_DELTA_REPLAY_BINARY", ROOT / "rb_servo_server/build/delta_follower_replay"))
CONFIG = ROOT / "rb_servo_server/config/stack_real.yaml"


class DeltaReplayTest(unittest.TestCase):
    def run_replay(self, directory, events, variant="baseline", start=None):
        source=directory / "events.jsonl";out=directory / (variant+".csv")
        source.write_text("".join(json.dumps(e)+"\n" for e in events))
        args=[str(BIN),str(CONFIG),str(source),str(out),variant]
        if start is not None:
            args.append(str(start))
        result=subprocess.run(args,cwd=ROOT,
                              text=True,capture_output=True,timeout=15)
        rows = []
        if out.exists():
            with out.open() as stream:
                rows = list(csv.DictReader(stream))
        return result, rows

    @staticmethod
    def constant_stream():
        events=[]
        for i in range(400):
            event={"t":i*.002,"dt":.002,"active":True,"reference":[0,0,0,0,0,0,1],
                   "gate":1.,"force":[0,0,0],"plan_gate":1.}
            if i%67==0:
                event["frame"]={"seq":i//67+1,"policy_dt":.0334,"delta":[[.001,0,0,0,0,0,20] for _ in range(8)]}
            events.append(event)
        return events

    def test_overlap_does_not_distort_identical_constant_predictions(self):
        with tempfile.TemporaryDirectory() as tmp:
            result,baseline=self.run_replay(Path(tmp),self.constant_stream())
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(len(baseline),400)
            self.assertTrue(all(row["stall"]=="0" for row in baseline))
            self.assertGreater(float(baseline[-1]["smd_x"]),.01)
            for variant in ("overlap_linear2","overlap_linear4","overlap_quintic4"):
                result,rows=self.run_replay(Path(tmp),self.constant_stream(),variant)
                self.assertEqual(result.returncode,0,result.stderr)
                for ref,row in zip(baseline,rows,strict=True):
                    for key in ("raw_x","smd_x","duration","axis_duration_0"):
                        self.assertAlmostEqual(float(ref[key]),float(row[key]),places=10)

    def test_missing_frame_and_invalid_time_fail_explicitly(self):
        with tempfile.TemporaryDirectory() as tmp:
            no_frame=self.constant_stream()[:1];no_frame[0].pop("frame")
            result,_=self.run_replay(Path(tmp),no_frame)
            self.assertNotEqual(result.returncode,0)
            self.assertIn("seed frame",result.stderr)
            bad_dt=self.constant_stream()[:1];bad_dt[0]["dt"]=0
            result,_=self.run_replay(Path(tmp),bad_dt)
            self.assertNotEqual(result.returncode,0)
            self.assertIn("servo dt",result.stderr)
            backward=self.constant_stream()[:2];backward[1]["t"]=-.1
            result,_=self.run_replay(Path(tmp),backward)
            self.assertNotEqual(result.returncode,0)
            self.assertIn("timestamps",result.stderr)

    def test_delayed_overlap_preserves_the_pre_switch_state(self):
        events=self.constant_stream()
        for i,event in enumerate(events):
            if "frame" in event:
                step=.001 if (i//67)%2==0 else .003
                for row in event["frame"]["delta"]:
                    row[0]=step
        with tempfile.TemporaryDirectory() as tmp:
            result,baseline=self.run_replay(Path(tmp),events)
            self.assertEqual(result.returncode,0,result.stderr)
            result,changed=self.run_replay(Path(tmp),events,"overlap_linear4",start=.4)
            self.assertEqual(result.returncode,0,result.stderr)
            for ref,row in zip(baseline,changed,strict=True):
                if float(row["t"])<.4:
                    self.assertEqual(ref,row)
            self.assertTrue(any(abs(float(ref["smd_x"])-float(row["smd_x"]))>1e-5
                                for ref,row in zip(baseline,changed,strict=True)))


class RecordedConsumerReplayTest(unittest.TestCase):
    @staticmethod
    def events():
        events = []
        for i in range(40):
            e = {"schema": "robotics_lab.recorded_follower_input.v2",
                 "tick": 1000000000+i*2000000, "t": i*.002, "mono": 1+i*.002,
                 "dt": .002, "active": i not in (20,21),
                 "previous_emitted": [.2,.1,.3,0,0,0,1], "actual": [.2,.1,.3,0,0,0,1],
                 "reference_strip_enabled": False, "reference_deviation": [0]*6,
                 "advance_gate": 1., "advance_direction": [0,0,0], "plan_rate_gate": 1.,
                 "observed_prefilter": [.2,.1,.3,0,0,0,1], "observed_stage": [.2,.1,.3,0,0,0,1]}
            if i in (0,10,22,32):
                e['frame'] = {'wire_seq': 700+i, 'recv_seq': 500+i, 'recv_time': e['mono']-.001,
                              'policy_dt': .0334, 'delta': [[.0005,0,0,0,0,0,20] for _ in range(8)]}
            events.append(e)
        return events

    def run_recorded(self, directory, events, options=()):
        source = directory/'recorded.jsonl'; output = directory/'recorded.csv'
        source.write_text(''.join(json.dumps(e)+'\n' for e in events))
        result = subprocess.run([str(BIN), '--recorded', str(CONFIG), 'flow_infer_fresh', str(source), str(output), *options],
                                cwd=ROOT, text=True, capture_output=True, timeout=15)
        rows = []
        if output.exists():
            with output.open() as stream:
                rows = list(csv.DictReader(stream))
        return result, rows

    def test_consumed_wire_identity_fresh_replacement_and_cold_reset(self):
        with tempfile.TemporaryDirectory() as tmp:
            result, rows = self.run_recorded(Path(tmp), self.events())
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(len(rows), 38)
            self.assertEqual(rows[10]['wire_seq'], '710')
            self.assertEqual(rows[20]['wire_seq'], '722')
            self.assertEqual([int(r['tick']) for r in rows if r['reseeded']=='1'], [1000000000,1044000000])
            self.assertIn('consumed_frames=4', result.stdout)

    def test_reference_strip_uses_current_deviation_and_exact_rotation(self):
        e = self.events()[:1]
        e[0]['reference_strip_enabled'] = True
        e[0]['reference_deviation'] = [.01,.02,.03,0,0,1.5707963267948966]
        with tempfile.TemporaryDirectory() as tmp:
            result, rows = self.run_recorded(Path(tmp), e)
            self.assertEqual(result.returncode, 0, result.stderr)
            for axis, expected in [('x',.19),('y',.08),('z',.27)]:
                self.assertAlmostEqual(float(rows[0]['reference_'+axis]), expected, places=12)
            self.assertAlmostEqual(abs(float(rows[0]['reference_qz'])), 2**-.5, places=12)
            self.assertLess(float(rows[0]['reference_qz'])*float(rows[0]['reference_qw']), 0)
            e[0]['reference_strip_enabled'] = False
            result, rows = self.run_recorded(Path(tmp), e)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertAlmostEqual(float(rows[0]['reference_x']), .2)
            self.assertAlmostEqual(float(rows[0]['reference_qz']), 0)

    def test_invalid_source_contract_fails_explicitly(self):
        mutations = [
            (lambda e: e[0].pop('frame'), 'seed frame'),
            (lambda e: e[0].pop('reference_deviation'), 'reference_deviation'),
            (lambda e: e[0].update(schema='legacy'), 'schema'),
            (lambda e: e[0].update(dt=0), 'clock'),
            (lambda e: e[0].update(advance_gate=1.1), 'gate'),
            (lambda e: e[0]['frame'].update(recv_time=2), 'identity/time'),
            (lambda e: e[0]['frame'].update(delta=[[0]*6]*3), 'dimension')]
        with tempfile.TemporaryDirectory() as tmp:
            for mutate, expected in mutations:
                with self.subTest(expected=expected):
                    events = copy.deepcopy(self.events()[:1]); mutate(events)
                    result, _ = self.run_recorded(Path(tmp), events)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(expected, result.stderr)

    def test_resume_requires_fresh_frame_instead_of_reusing_old_chunk(self):
        events = self.events()[:23]; events[22].pop('frame')
        with tempfile.TemporaryDirectory() as tmp:
            result, _ = self.run_recorded(Path(tmp), events)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('seed frame', result.stderr)

    def test_optional_leash_uses_own_previous_stage_and_preserves_recorded_gate(self):
        events=[]
        seed=self.events()[0]
        for i in range(300):
            e=copy.deepcopy(seed);e.update(t=i*.002,mono=1+i*.002,tick=1000000000+i*2000000)
            e.pop('frame')
            if i%40==0:
                e['frame']={'wire_seq':i+1,'recv_seq':i+1,'recv_time':e['mono']-.001,
                            'policy_dt':.0334,'delta':[[.02,0,0,0,0,0,20] for _ in range(8)]}
            if i>220:e['plan_rate_gate']=.5
            events.append(e)
        with tempfile.TemporaryDirectory() as tmp:
            result,rows=self.run_recorded(Path(tmp),events,('--conservative-stage-leash',))
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertTrue(any(float(r['recomputed_leash_gate'])<.99 for r in rows))
            for r in rows:
                self.assertAlmostEqual(float(r['plan_gate']),min(float(r['recorded_plan_gate']),float(r['recomputed_leash_gate'])),places=12)
            self.assertEqual(float(rows[0]['recomputed_leash_gate']),1.)


if __name__=="__main__":
    unittest.main()
