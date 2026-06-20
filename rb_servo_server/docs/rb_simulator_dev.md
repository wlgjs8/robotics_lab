# Hardware-Free rb_simulator Development

This path runs `rb_servo_server` against the repo-local `rb_simulator` JSONL
backend. It is for software development and regression evidence only: no real
robot, no Rainbow Robotics OVA, no privileged Docker, no host networking, no
credentials, and no production network exposure.

## Config Pair

Use these files together:

- `../rb_simulator/config/left_rb3_730e.yaml`: starts one left-arm simulator
  process with control/admin endpoints on loopback TCP ports `50200` and
  `50201`.
- `../rb_simulator/config/right_rb3_730e.yaml`: starts one right-arm simulator
  process with control/admin endpoints on loopback TCP ports `50210` and
  `50211`.
- `config/dual_simulator.yaml`: starts `rb_servo_server` with
  `backend_type: simulator`, points the left arm at
  `tcp://127.0.0.1:50200`, points the right arm at
  `tcp://127.0.0.1:50210`, binds UDP command/state ports to loopback, and
  keeps realtime priority off.

The historical `dual_rbsim.yaml`, `dual_rb_simulator.yaml`, and
`dual_rb_simulator_compose.yaml` names have been moved to
`docs/archive/configs/`. They are archived references only, not runnable
source-of-truth profiles, and must not be used for new evidence. New docs and
configs should use `run_mode: simulation`, `backend_type: simulator`, and
`simulator_control_endpoint`.

The per-arm endpoint split is the supported topology. Each simulator process
owns exactly one arm and rejects wrong-arm requests fail-closed.

## Local Commands

Run these from the workspace root, `/home/plaif/workspace/robotics_lab`.

Unit and contract checks:

```bash
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure
```

Local simulator smoke, after the local `rb_servo_server` binary exists:

```bash
PYTHONPATH=rb_simulator/src \
python3 rb_simulator/tools/rbsim_servo_smoke.py \
  --left-simulator-command "python3 -m rbsim" \
  --right-simulator-command "python3 -m rbsim" \
  --left-simulator-config rb_simulator/config/left_rb3_730e.yaml \
  --right-simulator-config rb_simulator/config/right_rb3_730e.yaml \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator.yaml \
  --artifacts-dir rb_simulator/artifacts/rbsim_servo_smoke
```

Parser and artifact validator only:

```bash
python3 rb_simulator/tools/rbsim_servo_smoke.py --self-test
```

## Operator Stack (native)

The full operator stack — `rb_servo_server` + the viser GUI
(`http://127.0.0.1:8080`) + `policy_runner` — runs natively (no Docker) via the
repository-root `Makefile`:

```bash
make run MODE=sim
```

`MODE=sim` uses the rbpodo controller-simulation path. Build/install the stack
first with `make build` after editing source. For a pure hardware-free
`rb_simulator` operator run, start `rb_servo_server` directly with
`config/dual_simulator.yaml` against the two per-arm simulator processes shown in
the Config Pair above.

For a split-PC simulator stack, the server profile
`config/dual_simulator_remote_172_28_60_36.yaml` points left simulator traffic at
`tcp://172.28.60.36:50200` and right simulator traffic at
`tcp://172.28.60.36:50210`. Run the simulator processes on `172.28.60.36` and set
`RB_SIMULATOR_ALLOW_NON_LOOPBACK=1` so they may bind the LAN NIC; the
control/GUI PC then runs the server against that remote profile.

Notes:

- The simulator is the repo-local Python JSONL backend, not `mo_rbsim_docker` and
  not a Rainbow Robotics OVA.
- It does not require `privileged`, host networking, USB devices, or real robot
  environment variables.
- The server is built with mandatory Pinocchio/Eigen support, publishes FK TCP
  poses, and enables simulator-only Cartesian IK for GUI TCP target tests; real
  Cartesian motion remains disabled.

The hardware-free validation gate and the local smoke runner above remain the
primary regression evidence paths.

## Required Evidence

A passing local simulator smoke artifact must show:

- UDP state stream packets for both arms with valid six-joint arrays.
- `ArmMotion` observed before the small `JointTarget`.
- Servo CSV rows where sent joints match the bounded target.
- No send failures and no logger dropped samples.

Stop/reset/fault evidence must stay simulator-local and machine-testable:

- `stop` evidence: the simulator reports stopped/held state and the servo log
  does not continue moving away from the last safe target.
- `reset_fault` evidence: reset remains in `ConnectedHold`; a new `ArmMotion`
  is required before motion targets.
- Fault evidence: injected invalid state, send failure, stop failure, reset
  failure, or disconnect is visible in the state/log artifacts and causes the
  expected hold or latch behavior.

These artifacts do not prove Rainbow Robotics external simulator behavior,
`rbpodo`, realtime scheduling, physical stop/reset behavior, or real robot
readiness.
