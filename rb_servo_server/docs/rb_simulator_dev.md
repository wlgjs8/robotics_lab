# Hardware-Free rb_simulator Development

This path runs `rb_servo_server` against the repo-local `rb_simulator` JSONL
backend. It is for software development and regression evidence only: no real
robot, no Rainbow Robotics OVA, no privileged Docker, no host networking, no
credentials, and no production network exposure.

## Config Pair

Use these files together:

- `../rb_simulator/config/dual_rb3_730e.yaml`: starts the simulator control and
  admin JSONL endpoints on loopback TCP ports `50200` and `50201`. This is one
  dual-arm simulator process that owns both left and right arm state.
- `config/dual_rb_simulator.yaml`: starts `rb_servo_server` with
  `backend_type: rbsim_local`, points both arms at `tcp://127.0.0.1:50200`,
  binds UDP command/state ports to loopback, and keeps realtime priority off.

`config/dual_rbsim.yaml` is kept as a compatibility alias for the same local
software-simulator shape. Do not use it as evidence for Rainbow Robotics rbsim
or real robot readiness.

The shared control endpoint is intentional. `RbsimBackend` sends `arm: "left"`
or `arm: "right"` in every request, and the simulator demultiplexes that into
independent in-process arm state. A future per-arm endpoint split is possible,
but it is not the current supported topology.

## Local Commands

Run these from the workspace root, `/home/plaif/workspace/robotics_lab`.

Unit and contract checks:

```bash
python3 -m unittest discover rb_simulator/tests
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate -DRB_SERVO_ENABLE_RBPODO=OFF -DRB_SERVO_ALLOW_FETCHCONTENT=OFF -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure
```

Local simulator smoke, after both local binaries exist:

```bash
python3 rb_simulator/tools/rbsim_servo_smoke.py \
  --simulator rb_simulator/build/rb_simulator \
  --simulator-config rb_simulator/config/dual_rb3_730e.yaml \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_rb_simulator.yaml \
  --artifacts-dir rb_simulator/artifacts/rbsim_servo_smoke
```

If the simulator executable is not available yet, run only the parser and
artifact validator:

```bash
python3 rb_simulator/tools/rbsim_servo_smoke.py --self-test
```

## Compose Profile

`docker-compose.yml` includes a profile-gated software simulator pair:

- `rb_simulator`: Python JSONL simulator from `../rb_simulator`.
- `rb_servo_rbsim`: C++ server using `config/dual_rb_simulator.yaml`.

The profile is intentionally bounded:

- It uses the repo-local simulator image, not `mo_rbsim_docker` and not a
  Rainbow Robotics OVA.
- It does not request `privileged`, host networking, USB devices, or real robot
  environment variables.
- `rb_servo_rbsim` shares the simulator service network namespace so
  both backends reach the single dual-arm simulator at
  `tcp://127.0.0.1:50200` while the endpoint remains loopback-only.
- The default GUI/mock port mappings are pinned to `127.0.0.1`.

Treat the compose profile as wiring documentation until a human explicitly
chooses a container smoke. The hardware-free validation gate and the local smoke
runner above are the supported evidence paths for this phase.

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

These artifacts do not prove Rainbow Robotics rbsim, `rbpodo`, realtime
scheduling, physical stop/reset behavior, or real robot readiness.
