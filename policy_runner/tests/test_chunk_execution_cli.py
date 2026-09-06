from __future__ import annotations

import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

from policy_runner.main import main


ROOT = Path(__file__).resolve().parents[2]


class FreshExecutionCliTest(unittest.TestCase):
    def test_incompatible_modes_fail_before_devices_or_model_are_created(self):
        base = [
            "flow-infer", "--config", str(ROOT / "policy_runner/config/flow_real_realsense.yaml"),
            "--rollout-mode", "real_policy", "--checkpoint", "openpi://invalid.invalid:8003",
        ]
        cases = [
            (["--chunk-activation-mode", "ready_event", "--rtc"], "RTC"),
            (["--chunk-activation-mode", "ready_event", "--chunk-anchor-source", "chain"], "chain anchor"),
            (["--chunk-activation-mode", "ready_event", "--stream-prefetch-at", "2"], "stream_prefetch_at"),
            (["--chunk-activation-mode", "ready_event", "--sequential-chunk-inference"], "sequential"),
            (["--velproprio-source", "servo_command"], "fixed_step"),
        ]
        for options, reason in cases:
            with self.subTest(options=options), \
                    mock.patch("policy_runner.openpi_remote.OpenpiRemoteActionSource") as model, \
                    mock.patch("policy_runner.camera_bundle_client.CameraBundleClient") as camera, \
                    mock.patch("policy_runner.main.ServoCommandClient") as command, \
                    mock.patch("policy_runner.gripper.PikaSerialGripperBackend") as gripper, \
                    contextlib.redirect_stderr(io.StringIO()) as stderr:
                self.assertEqual(main(base + options), 2)
                self.assertIn(reason, stderr.getvalue())
                model.assert_not_called()
                camera.assert_not_called()
                command.assert_not_called()
                gripper.assert_not_called()

    def test_launcher_preserves_explicit_execution_selection(self):
        # The stand-in records argv and exits. No policy process or device opens.
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "argv.json"
            recorder = Path(directory) / "record-argv"
            recorder.write_text(
                "#!/usr/bin/env python3\nimport json,os,sys\n"
                "with open(os.environ['ARG_CAPTURE_PATH'],'w') as f: json.dump(sys.argv[1:],f)\n"
            )
            recorder.chmod(0o755)
            env = dict(os.environ)
            for name in list(env):
                if name.startswith("FLOW_INFER_"):
                    del env[name]
            env.update(
                ARG_CAPTURE_PATH=str(capture), FLOW_INFER_PYTHON=str(recorder),
                FLOW_INFER_INCLUDE_DEPTH="0", FLOW_INFER_RTC="0",
                FLOW_INFER_CHUNK_ANCHOR="command", FLOW_INFER_CHUNK_ACTIVATION_MODE="ready_event",
                FLOW_INFER_TCP_TARGET_PROFILE="flow_infer_fresh", FLOW_INFER_VELPROPRIO_SOURCE="servo_command",
                FLOW_INFER_VELPROPRIO_SAMPLE="fixed_step",
            )
            result = subprocess.run(
                ["bash", "tools/flow_infer_real_policy.sh", "--proprio-mode", "velocity_grip"],
                cwd=ROOT, env=env, capture_output=True, text=True, timeout=10,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            args = json.loads(capture.read_text())
            for name, value in [
                ("--chunk-activation-mode", "ready_event"),
                ("--tcp-target-profile", "flow_infer_fresh"),
                ("--velproprio-source", "servo_command"),
                ("--velproprio-sample-mode", "fixed_step"),
            ]:
                self.assertEqual(args[args.index(name) + 1], value)

    def test_experimental_profiles_preserve_each_stacks_existing_limits(self):
        import yaml

        for name in ("stack_real.yaml", "stack_sim.yaml"):
            with self.subTest(config=name):
                config = yaml.safe_load((ROOT / "rb_servo_server/config" / name).read_text())
                profiles = config["cartesian_control"]["tcp_pose_target_profiles"]
                fresh = profiles["flow_infer_fresh"]
                self.assertTrue(fresh["ruckig_follower"].pop("fresh_chunk_replan"))
                self.assertTrue(fresh["ruckig_follower"].pop("continuous_hold_resume"))
                smooth = profiles["flow_infer_smooth"]
                if name == "stack_real.yaml":
                    # The new translation-only conditioner is the sole additional
                    # difference. All angular, force, motion and guard settings
                    # remain covered by the complete equality check below.
                    self.assertEqual(fresh["ruckig_follower"]["output_smd"].pop("velocity_ff_linear_gain"), 0.8)
                    self.assertEqual(smooth["ruckig_follower"]["output_smd"].pop("velocity_ff_linear_gain"), 1.0)
                self.assertEqual(fresh, smooth)


if __name__ == "__main__":
    unittest.main()
