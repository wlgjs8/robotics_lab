# rb_simulator

`rb_simulator` is the hardware-free simulator workspace for
`rb_servo_server`. It provides a deterministic local controller endpoint for
one RB3-730 arm per process, matching the real system topology of one
controller per arm.

It does not use `rbpodo`, robot hardware, real robot networks, or Rainbow
Robotics simulator images. It runs natively (no Docker).

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
  docs/
    architecture.md
    protocol_v1.md
  src/
    rbsim/
  tests/
```

## Topology

Host-run default (one process per arm, loopback only):

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

Non-loopback binds are rejected unless `RB_SIMULATOR_ALLOW_NON_LOOPBACK=1` is set
explicitly (used for split-PC simulator runs that bind a specific NIC).

## Running

From the repository root:

```bash
PYTHONPATH=rb_simulator/src python3 -m rbsim --config rb_simulator/config/left_rb3_730e.yaml
PYTHONPATH=rb_simulator/src python3 -m rbsim --config rb_simulator/config/right_rb3_730e.yaml
```

These host-run profiles are loopback-only and are the supported way to start the
simulator processes.

The Python module name and protocol schema retain the existing `rbsim` names
for compatibility. Public configuration should use `backend_type: simulator`
and `run_mode: simulation`.

## Tests

```bash
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
```

The deprecated `dual_rb3_730e.yaml` fixture for the old single-process schema
has been moved to `docs/archive/configs/`. It is not a supported runtime
profile and must not be used for operator runs or smoke evidence.
