# rb_simulator Operator Smoke

This smoke is the hardware-free operator check for the `rb_simulator` plus
`rb_servo_server` rbsim backend path. It starts both local
processes, sends `ArmMotion`, sends one small dual-arm `JointTarget`, captures
the UDP state stream, and verifies that the servo CSV log reflects the same
commanded target.

Topology under test: one `rb_simulator` process owns both left and right arm
state. Both servo-server `RbsimBackend` instances connect to that same process
through `tcp://127.0.0.1:50200`; requests are routed by their `arm` field.

## Preconditions

Run this only after these software-only prerequisites are complete:

- `rb_simulator/build/rb_simulator` exists and starts from
  `rb_simulator/config/dual_rb3_730e.yaml`.
- `rb_servo_server/build/rb_servo_server` exists.
- `rb_servo_server/config/dual_rb_simulator.yaml` uses the
  loopback `rbsim` backend, not `rbpodo`, not `dual_real.yaml`, and not any
  exposed network bind.

The smoke runner fails closed if any prerequisite is missing. That failure means
the requested local smoke evidence is unavailable, not that hardware validation
should be attempted.

## Command

From the workspace root:

```bash
python3 rb_simulator/tools/rbsim_servo_smoke.py \
  --artifacts-dir /tmp/rbsim_servo_smoke
```

The default command starts:

- `rb_simulator/build/rb_simulator --config rb_simulator/config/dual_rb3_730e.yaml`
- `rb_servo_server/build/rb_servo_server --config rb_servo_server/config/dual_rb_simulator.yaml`

It then listens on `127.0.0.1:50110`, sends commands to `127.0.0.1:50010`,
and records artifacts under the selected artifact directory. It does not launch
one simulator per arm.

## Artifacts

The artifact directory is bounded and reviewable:

- `simulator.log` captures simulator stdout/stderr.
- `rb_servo_server.log` captures servo-server stdout/stderr.
- `state_snapshots.jsonl` captures at most 200 state packets and at most
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

- Rainbow Robotics rbsim / OVA readiness.
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
