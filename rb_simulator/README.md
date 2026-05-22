# rb_simulator

`rb_simulator` is the hardware-free development and verification workspace for
`rb_servo_server`.

It is intentionally separate from `mo_rbsim_docker`. The existing
`mo_rbsim_docker` assets are useful reference material for robot identity,
network shape, and operator expectations, but this workspace owns a smaller,
deterministic simulator contract that can run in local tests and CI without an
OVA image, privileged Docker, robot hardware, or `rbpodo`.

## Scope

- Provide a deterministic dual-arm simulator that mimics the `IRobotBackend`
  behavior needed by `rb_servo_server`.
- Standardize the local simulator topology as one `rb_simulator` process that
  owns both left and right arm state.
- Exercise `connect`, `initialize`, `readState`, `sendServoJ`, `stop`, and
  `resetFault` paths through a new `rbsim` backend type.
- Publish truthful state, connection, servo-enabled, and fault fields so
  `rb_servo_server` can validate safety behavior before hardware gates.
- Support scripted fault injection for disconnects, invalid state, read/send
  failures, tracking error, stop failure, and reset failure.
- Provide a bounded operator smoke runner for the simulator-backed
  `rb_servo_server` path.

## Non-goals

- No Rainbow Robotics OVA conversion.
- No privileged Docker requirement.
- No real robot or `rbpodo` command execution.
- No claim that green simulator tests replace rbsim/real hardware acceptance.

## Layout

```text
rb_simulator/
  config/
    dual_rb3_730e.yaml
  docs/
    architecture.md
    operator_smoke.md
    task_breakdown.md
  src/
    README.md
  tests/
    README.md
  tools/
    rbsim_servo_smoke.py
```

The servo-side integration lives in
`../rb_servo_server/config/dual_rb_simulator.yaml`,
`RbsimBackend`, and the smoke tooling under `tools/`.

## Topology

The standard hardware-free topology is a single dual-arm simulator process:

```text
rb_servo_server
  left RbsimBackend  -> tcp://127.0.0.1:50200
  right RbsimBackend -> tcp://127.0.0.1:50200

rb_simulator process
  control endpoint: tcp://127.0.0.1:50200
  admin endpoint:   tcp://127.0.0.1:50201
  arm state: left + right
```

Each backend request carries `arm: "left"` or `arm: "right"`. The simulator
uses that field to route the operation to the matching in-process arm state.
This keeps dual-arm fault injection, deterministic ticks, and smoke-runner
process management in one local service.

A future two-endpoint or two-process layout may be added if per-arm simulator
isolation is needed. That is not the current standard; configs and smoke checks
should keep both arms pointed at the same control endpoint unless a future
migration explicitly updates the contract.

## Operator Smoke

After the simulator executable and `rb_servo_server` binary are available, run:

```bash
python3 rb_simulator/tools/rbsim_servo_smoke.py \
  --artifacts-dir /tmp/rbsim_servo_smoke
```

The smoke starts both local processes, sends `ArmMotion` and a small joint
target, verifies the UDP state stream and servo CSV log, and writes bounded
artifacts. See `docs/operator_smoke.md` and
`../rb_servo_server/docs/rb_simulator_dev.md` for pass criteria and
the explicit caveat that simulator-only evidence does not prove Rainbow rbsim,
`rbpodo`, or real robot readiness.

## Current Core

The first hardware-free core lives in `src/rbsim`. It loads
`config/dual_rb3_730e.yaml`, maintains independent left/right six-joint state
inside one simulator process, supports explicit connected, initialized,
servo-enabled, stopped, and faulted transitions, and advances actual joints
toward targets with deterministic fixed-step timing.

Run the unit coverage from the workspace root:

```bash
python3 -m unittest discover rb_simulator/tests
```

The loopback JSONL service and protocol contract now live beside the core. Run
the service from source with:

```bash
PYTHONPATH=rb_simulator/src python3 -m rbsim --config rb_simulator/config/dual_rb3_730e.yaml
```

The protocol spec is `docs/protocol_v1.md`. The simulator service binds only to
loopback by default, uses the repo-local config for control/admin endpoints,
and has deterministic admin hooks for invalid state, injected read/send/stop/reset
failures, latency, tracking bias, frozen motion, and manual ticks.

This core and service do not start Docker, import `rbpodo`, contact a real
robot, expose production network endpoints, or use `mo_rbsim_docker` assets.

## Compose Wiring

`../rb_servo_server/docker-compose.yml` contains a `sim` profile
with two hardware-free services:

- `rb_simulator`: this Python JSONL simulator.
- `rb_servo_rbsim`: `rb_servo_server` with
  `config/dual_rb_simulator.yaml`.

The compose profile is non-OVA and non-hardware. It does not use privileged
Docker, host networking, USB devices, real robot credentials, or
`mo_rbsim_docker`; `rb_servo_rbsim` shares the simulator network namespace so
both servo backends reach the single dual-arm simulator at
`tcp://127.0.0.1:50200`.
