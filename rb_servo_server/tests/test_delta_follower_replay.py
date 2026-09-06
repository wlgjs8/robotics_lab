"""Exercise the offline binary without any backend/device startup."""
import csv
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
        return result,list(csv.DictReader(out.open())) if out.exists() else []

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


if __name__=="__main__":
    unittest.main()
