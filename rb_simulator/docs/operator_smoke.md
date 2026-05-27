# rb_simulator Operator Smoke

This smoke is the hardware-free operator check for the `rb_simulator` plus
`rb_servo_server` simulator backend path. It starts both local
processes, sends `ArmMotion`, sends one small dual-arm `JointTarget`, captures
the UDP state stream, and verifies that the servo CSV log reflects the same
commanded target.

Current P0-B topology: one simulator process owns the left arm and a second
simulator process owns the right arm. Host defaults are
`tcp://127.0.0.1:50200` for left control and `tcp://127.0.0.1:50210` for right
control. Wrong-arm requests are rejected fail-closed.

## Preconditions

Run this only after these software-only prerequisites are complete:

- left and right simulator processes can start from
  `rb_simulator/config/left_rb3_730e.yaml` and
  `rb_simulator/config/right_rb3_730e.yaml`.
- `rb_servo_server/build/rb_servo_server` exists.
- `rb_servo_server` has a per-arm simulator config that uses the loopback
  simulator backend, not `rbpodo`, not the tracked real template or local real
  configs, and not any exposed network bind.

The smoke runner fails closed if any prerequisite is missing. That failure means
the requested local smoke evidence is unavailable, not that hardware validation
should be attempted.

## Command

From the workspace root:

```bash
PYTHONPATH=rb_simulator/src \
python3 rb_simulator/tools/rbsim_servo_smoke.py \
  --left-simulator-command "python3 -m rbsim" \
  --right-simulator-command "python3 -m rbsim" \
  --left-simulator-config rb_simulator/config/left_rb3_730e.yaml \
  --right-simulator-config rb_simulator/config/right_rb3_730e.yaml \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator.yaml \
  --artifacts-dir /tmp/rbsim_servo_smoke
```

It starts one simulator process per arm, then starts
`rb_servo_server/build/hardware_free_gate/rb_servo_server --config
rb_servo_server/config/dual_simulator.yaml`. It listens on `127.0.0.1:50110`,
sends commands to `127.0.0.1:50010`, and records artifacts under the selected
artifact directory.

## Artifacts

The artifact directory is bounded and reviewable:

- `left_simulator.log` captures left simulator stdout/stderr.
- `right_simulator.log` captures right simulator stdout/stderr.
- `rb_servo_server.log` captures servo-server stdout/stderr.
- `state_stream.jsonl` captures at most 300 state packets and at most
  1 MB by default.
- `logs/*.csv` is the servo-server CSV log because the runner starts the
  server with the artifact directory as its working directory.
- `summary.json` records the validation result, state packet counts, target
  tick, servo log path, and dropped-sample/send-failure checks.

Use `--max-state-packets` and `--max-state-bytes` to make state capture more
or less strict without changing the smoke logic.

## Pass Criteria

The runner exits zero only when all of these are true:

- Both local processes stay alive long enough for startup validation.
- The state stream reports both arms `Connected` with
  `has_valid_joint_state=true`.
- All observed joint arrays used by the check are finite six-element arrays.
- The stream observes the `ArmMotion` command sequence.
- The stream observes the `JointTarget` command sequence.
- `left.q_sent_deg` and `right.q_sent_deg` reflect the commanded small target.
- The servo CSV log has required send, dropped-sample, command sequence, and
  sent-joint columns.
- The servo CSV log records no send failures, no dropped samples, and at least
  one row matching the commanded target.

Any non-zero exit means the smoke evidence is not usable as a pass artifact.

## What This Does Not Prove

This is simulator-only evidence. It does not prove:

- Rainbow Robotics simulator / OVA readiness.
- `rbpodo` readiness.
- Real robot readiness.
- Realtime scheduling readiness.
- Stop/reset behavior against real controller firmware.
- Production deployment safety, privileged Docker safety, or exposed network
  safety.

Those remain separate human-gated or hardware-gated validations. Do not use a
green result from this smoke to relax `RB_ALLOW_REAL_ROBOT`, network exposure,
physical E-stop, operator, or workspace-clearance gates.

## Stop, Reset, and Fault Evidence

Use simulator-local admin hooks and smoke artifacts for stop/reset/fault claims.
Evidence is usable only when it is captured in the bounded artifact directory.

- Stop evidence must show that state and CSV rows hold the last safe target
  after stop.
- Reset evidence must show `ConnectedHold` after `ResetFault`; a later
  `ArmMotion` is required before any motion target is accepted.
- Fault evidence must show explicit invalid-state, disconnect, send-failure,
  stop-failure, or reset-failure behavior and the corresponding hold or latch in
  the server state/log.

Do not treat these software hooks as physical controller stop/reset validation.

## Local Tooling Check

Before the simulator/backend prerequisites exist, validate only the smoke
runner's parser and artifact validators:

```bash
python3 rb_simulator/tools/rbsim_servo_smoke.py --self-test
```

The command should pass without launching simulator or servo-server processes.
