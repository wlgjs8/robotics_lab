#!/usr/bin/env python3
"""Validate controller-pgmode flow-infer experiment artifacts fail-closed."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import yaml


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return value


def _record(checks: list[dict[str, Any]], name: str, passed: bool, evidence: Any) -> None:
    checks.append({"name": name, "passed": bool(passed), "evidence": evidence})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment-dir", type=Path, required=True)
    parser.add_argument("--w6-monitor", type=Path, required=True)
    parser.add_argument("--w12-monitor", type=Path, required=True)
    parser.add_argument("--minimum-duration-sec", type=float, required=True)
    parser.add_argument("--expected-trials", type=int, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args()

    checks: list[dict[str, Any]] = []
    monitor_path = args.experiment_dir / "controller_monitor.json"
    trials_path = args.experiment_dir / "trials.tsv"
    server_log_path = args.experiment_dir / "server.log"
    monitor = _load_json(monitor_path)
    w6 = _load_json(args.w6_monitor)
    w12 = _load_json(args.w12_monitor)

    _record(
        checks,
        "long_duration",
        float(monitor.get("duration_sec", 0.0)) >= args.minimum_duration_sec,
        monitor.get("duration_sec"),
    )
    _record(checks, "long_packets_present", int(monitor.get("packet_count", 0)) > 0,
            monitor.get("packet_count"))
    _record(checks, "operation_mode_stayed_simulation",
            int(monitor.get("invalid_mode_or_motion_packets", -1)) == 0,
            monitor.get("unsafe_errors"))
    _record(checks, "no_physical_motion_detected",
            int(monitor.get("physical_motion_detected_packets", -1)) == 0,
            monitor.get("physical_motion_detected_packets"))
    _record(checks, "no_fault_latch",
            int(monitor.get("fault_latched_packets", -1)) == 0,
            monitor.get("fault_latched_packets"))
    displacement = monitor.get("max_tcp_actual_displacement_m", {})
    _record(
        checks,
        "physical_tcp_drift_diagnostic_recorded",
        isinstance(displacement, dict) and
        all(
            math.isfinite(float(displacement.get(arm, float("nan")))) and
            float(displacement.get(arm, -1.0)) >= 0.0
            for arm in ("left", "right")
        ),
        displacement,
    )
    server_log = server_log_path.read_text(encoding="utf-8", errors="replace") \
        if server_log_path.is_file() else ""
    raw_mode_markers = (
        "for left_stack_sim initial_state_valid=true controller_mode=simulation",
        "for right_stack_sim initial_state_valid=true controller_mode=simulation",
        "startup_previous_target_source left=reference right=reference",
    )
    missing_raw_mode_markers = [
        marker for marker in raw_mode_markers if marker not in server_log
    ]
    _record(
        checks,
        "controller_raw_mode_and_reference_startup_confirmed",
        not missing_raw_mode_markers,
        {"server_log": str(server_log_path), "missing_markers": missing_raw_mode_markers},
    )

    with trials_path.open(newline="", encoding="utf-8") as handle:
        trials = list(csv.DictReader(handle, delimiter="\t"))
    _record(checks, "expected_trial_count", len(trials) == args.expected_trials, len(trials))
    _record(checks, "all_trials_pass",
            len(trials) == args.expected_trials and
            all(row.get("validation") == "pass" for row in trials),
            {"pass": sum(row.get("validation") == "pass" for row in trials), "total": len(trials)})

    rollout_paths = sorted(args.experiment_dir.glob("trial_*_rollout.json"))
    rollout_failures: list[dict[str, Any]] = []
    parameter_failures: list[dict[str, Any]] = []
    total_commands = 0
    total_inferences = 0
    total_gripper_proposals = 0
    total_gripper_drops = 0
    for path in rollout_paths:
        rollout = _load_json(path).get("rollout_summary", {})
        if not isinstance(rollout, dict):
            rollout_failures.append({"path": str(path), "reason": "missing rollout_summary"})
            continue
        total_commands += int(rollout.get("sent_command_count", 0))
        diagnostics = rollout.get("inference_diagnostics", {})
        if isinstance(diagnostics, dict):
            total_inferences += int(diagnostics.get("total_inferences", 0))
        total_gripper_proposals += int(rollout.get("gripper_command_count", 0))
        total_gripper_drops += int(rollout.get("gripper_dropped_count", 0))
        valid = (
            rollout.get("rollout_mode") == "controller_sim" and
            rollout.get("operation_mode_seen") == "simulation" and
            rollout.get("physical_motion_expected") is False and
            rollout.get("allows_physical_real_motion") is False and
            rollout.get("allow_real_gripper_motion") is False and
            rollout.get("config_path") == "policy_runner/config/flow_sim_offline.yaml" and
            int(rollout.get("dropped_command_count", -1)) == 0 and
            int(rollout.get("sent_command_count", 0)) > 0
        )
        if not valid:
            rollout_failures.append({"path": str(path), "reason": "safety/runtime contract mismatch"})
        log_path = path.with_name(path.name.replace("_rollout.json", ".log"))
        log_text = log_path.read_text(encoding="utf-8", errors="replace") \
            if log_path.is_file() else ""
        expected_log_markers = (
            "--chunk-execute-steps 6",
            "--max-linear-velocity-m-s 0.005",
            "openpi action_horizon=24 chunk_execute_steps=6",
            "rotation axes kept=none",
        )
        missing_markers = [marker for marker in expected_log_markers if marker not in log_text]
        if missing_markers:
            parameter_failures.append(
                {"path": str(log_path), "missing_markers": missing_markers}
            )
    _record(checks, "rollout_artifact_count", len(rollout_paths) == args.expected_trials,
            len(rollout_paths))
    _record(checks, "all_rollout_contracts", not rollout_failures, rollout_failures)
    _record(checks, "all_trials_used_bounded_w6_parameters",
            not parameter_failures, parameter_failures)
    _record(checks, "commands_exercised", total_commands > 0, total_commands)
    _record(checks, "inferences_exercised", total_inferences > 0, total_inferences)
    _record(checks, "all_gripper_proposals_suppressed",
            total_gripper_proposals > 0 and total_gripper_drops == total_gripper_proposals,
            {"proposed": total_gripper_proposals, "dropped": total_gripper_drops})

    _record(checks, "w6_controller_trial_passed",
            int(w6.get("fault_latched_packets", -1)) == 0 and
            int(w6.get("physical_motion_detected_packets", -1)) == 0,
            {"faults": w6.get("fault_latched_packets"),
             "physical": w6.get("physical_motion_detected_packets")})
    _record(checks, "w12_strict_lead_gate_reproduced",
            int(w12.get("fault_latched_packets", 0)) > 0 and
            any("delta_preview_actual_lead_fault" in str(item)
                for item in w12.get("unsafe_errors", [])),
            w12.get("unsafe_errors"))

    sim_config = yaml.safe_load(Path("rb_servo_server/config/stack_sim.yaml").read_text())
    real_config = yaml.safe_load(Path("rb_servo_server/config/stack_real.yaml").read_text())
    policy_config = yaml.safe_load(Path("policy_runner/config/flow_sim_offline.yaml").read_text())
    cartesian = sim_config.get("cartesian_control", {})
    _record(checks, "sim_cartesian_authority_is_narrow",
            cartesian.get("allow_in_controller_simulation") is True and
            cartesian.get("allow_in_real") is False,
            {"allow_in_controller_simulation": cartesian.get("allow_in_controller_simulation"),
             "allow_in_real": cartesian.get("allow_in_real")})
    _record(
        checks,
        "physical_box_motion_guard_is_server_latched",
        sim_config.get("safety", {}).get(
            "controller_simulation_physical_motion_policy"
        ) == "fault_latch",
        sim_config.get("safety", {}).get(
            "controller_simulation_physical_motion_policy"
        ),
    )
    sim_profiles = cartesian.get("tcp_pose_target_profiles", {})
    real_profiles = real_config.get("cartesian_control", {}).get(
        "tcp_pose_target_profiles", {})
    _record(
        checks,
        "tcp_pose_target_profiles_match_real",
        isinstance(sim_profiles, dict) and bool(sim_profiles) and
        sim_profiles == real_profiles,
        {"profiles": sorted(sim_profiles) if isinstance(sim_profiles, dict) else []},
    )
    policy_safety = policy_config.get("safety", {})
    _record(
        checks,
        "offline_camera_and_no_real_gripper_authority",
        policy_config.get("camera", {}).get("zmq_endpoint") == "tcp://127.0.0.1:5700" and
        policy_config.get("gripper", {}).get("backend") == "none" and
        policy_safety.get("allow_real_motion") is False and
        policy_safety.get("allow_real_gripper_motion") is False,
        {
            "camera_endpoint": policy_config.get("camera", {}).get("zmq_endpoint"),
            "gripper_backend": policy_config.get("gripper", {}).get("backend"),
            "allow_real_motion": policy_safety.get("allow_real_motion"),
            "allow_real_gripper_motion": policy_safety.get("allow_real_gripper_motion"),
        },
    )

    passed = all(check["passed"] for check in checks)
    result = {
        "schema": "robotics_lab.flow_sim_experiment_validation.v1",
        "status": "pass" if passed else "fail",
        "checks": checks,
        "totals": {
            "checks": len(checks),
            "passed": sum(check["passed"] for check in checks),
            "trials": len(trials),
            "commands": total_commands,
            "inferences": total_inferences,
        },
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
