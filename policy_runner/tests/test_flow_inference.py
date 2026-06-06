from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path

from policy_runner.action_sources.tcp_delta import cartesian_action_requirements


class FlowInferenceCliTest(unittest.TestCase):
    def test_flow_infer_help_lists_rollout_mode_and_controller_sim(self) -> None:
        env = dict(os.environ)
        env["PYTHONPATH"] = "policy_runner"

        result = subprocess.run(
            [sys.executable, "-m", "policy_runner", "flow-infer", "--help"],
            cwd=Path(__file__).resolve().parents[2],
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--rollout-mode", result.stdout)
        self.assertIn("controller_sim", result.stdout)
        self.assertIn("--command-family", result.stdout)
        self.assertIn("tcp_twist_local", result.stdout)
        self.assertIn("--policy-dt-sec", result.stdout)

    def test_controller_sim_requirement_helper_allows_rbpodo_carveout(self) -> None:
        requirements = cartesian_action_requirements(allow_rbpodo_controller_simulation=True)

        self.assertTrue(requirements.allow_rbpodo_controller_simulation_cartesian)
        self.assertTrue(requirements.cartesian_motion)


if __name__ == "__main__":
    unittest.main()
