# rb_simulator Architecture

## Target Result

`rb_simulator` provides a deterministic, hardware-free controller endpoint for
one RB3-730 arm per process. Two simulator processes represent the dual-arm
system, which mirrors the real topology of one physical controller per arm.

The simulator is not a camera simulator, policy runner, force controller, or
real robot acceptance substitute. It is a backend contract exerciser for
`rb_servo_server`.

## Process Topology

```text
rb_servo_server
  left backend_type=simulator  -> left simulator process
  right backend_type=simulator -> right simulator process

left simulator
  configured arm: left
  host control: tcp://127.0.0.1:50200
  host admin:   tcp://127.0.0.1:50201

right simulator
  configured arm: right
  host control: tcp://127.0.0.1:50210
  host admin:   tcp://127.0.0.1:50211
```

Each request may still carry an `arm` field for protocol compatibility.
Requests with a matching arm are accepted. Requests with a mismatched arm are
rejected before state or fault hooks are mutated.

## State Machine

Each process owns exactly one simulated arm state:

- disconnected
- connected
- initialized
- servo enabled
- stopped
- faulted

The simulator tracks six-joint actual position, target position, velocity,
joint-state validity, connection state, servo state, fault state, recoverable
fault state, error code, and monotonic robot time.

`send_servo_j` updates the target only after connected, initialized,
servo-enabled, non-faulted, valid state is confirmed. `stop` holds the current
actual position when valid, otherwise the last accepted safe target.
`reset_fault` clears only recoverable faults and does not re-enable motion.

## Admin Endpoint

The admin endpoint is for tests and smoke checks. It supports deterministic:

- manual ticks
- hook reset
- read/send/stop/reset failure injection
- latency injection
- disconnect/reconnect
- joint validity toggles
- stale state
- fault latching
- tracking bias
- frozen motion

When an admin command omits `arm`, the configured process arm is used. A
specified mismatched arm is rejected.

## Bind Policy

Loopback is the default safety boundary. Binding to `0.0.0.0` or another
non-loopback address requires:

```bash
RB_SIMULATOR_ALLOW_NON_LOOPBACK=1
```

This allows container-internal binds without making exposed host binds the
default.

## Configuration

Supported runtime profiles:

- `rb_simulator/config/left_rb3_730e.yaml`
- `rb_simulator/config/right_rb3_730e.yaml`

Deprecated historical fixture:

- `rb_simulator/config/dual_rb3_730e.yaml`

Public servo-server configuration should use:

```yaml
run_mode: simulation
backend_type: simulator
```

## Verification

Required P0-B unit gate:

```bash
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
```

This verifies per-arm config loading, single-arm state ownership, wrong-arm
rejection, protocol responses, bind policy, and concurrent left/right service
startup on loopback.
