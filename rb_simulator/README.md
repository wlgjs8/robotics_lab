# rb_simulator

`rb_simulator` is the hardware-free simulator workspace for
`rb_servo_server`. It provides a deterministic local controller endpoint for
one RB3-730 arm per process, matching the real system topology of one
controller per arm.

It does not use `rbpodo`, robot hardware, privileged Docker, real robot
networks, or Rainbow Robotics simulator images.

## Scope

- Run one simulator process for the left arm and one simulator process for the
  right arm.
- Exercise `connect`, `initialize`, `readState`, `sendServoJ`, `stop`, and
  `resetFault` paths through the simulator backend contract.
- Reject wrong-arm requests fail-closed. A left process accepts `arm: "left"`
  and rejects `arm: "right"`; a right process does the inverse.
- Publish truthful joint state, connection state, servo-enabled state, and
  fault state for hardware-free servo-server tests.
- Support deterministic admin hooks for disconnects, invalid state, read/send
  failures, tracking bias, frozen motion, stop failure, reset failure, and
  manual ticks.

## Non-goals

- No real robot motion.
- No force, admittance, or impedance control.
- No Cartesian/TCP motion acceptance.
- No claim that green simulator tests replace Rainbow simulator or real robot
  acceptance.

## Layout

```text
rb_simulator/
  config/
    left_rb3_730e.yaml
    right_rb3_730e.yaml
    dual_rb3_730e.yaml    # historical only, not runnable for current topology
  docs/
    architecture.md
    protocol_v1.md
  src/
    rbsim/
  tests/
```

## Topology

Host-run default:

```text
rb_servo_server
  left backend_type=simulator  -> tcp://127.0.0.1:50200
  right backend_type=simulator -> tcp://127.0.0.1:50210

left simulator process
  arm: left
  control: tcp://127.0.0.1:50200
  admin:   tcp://127.0.0.1:50201

right simulator process
  arm: right
  control: tcp://127.0.0.1:50210
  admin:   tcp://127.0.0.1:50211
```

Containerized runs may bind inside each container to `0.0.0.0:50200` and
`0.0.0.0:50201`, but non-loopback binds are rejected unless
`RB_SIMULATOR_ALLOW_NON_LOOPBACK=1` is set.

## Running

From the repository root:

```bash
PYTHONPATH=rb_simulator/src python3 -m rbsim --config rb_simulator/config/left_rb3_730e.yaml
PYTHONPATH=rb_simulator/src python3 -m rbsim --config rb_simulator/config/right_rb3_730e.yaml
```

The Python module name and protocol schema retain the existing `rbsim` names
for compatibility. Public configuration should use `backend_type: simulator`
and `run_mode: simulation`.

## Tests

```bash
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
```

The deprecated `config/dual_rb3_730e.yaml` is kept only as a historical
dual-arm fixture for the old single-process schema. It is not a supported
runtime profile and must not be used for operator runs or smoke evidence.
