# rb_simulator Task Breakdown

> Historical planning document. The current P0-B source of truth is
> `README.md`, `docs/architecture.md`, and `docs/protocol_v1.md`.

## Implementation Tasks

1. Scaffold `rb_simulator` executable and deterministic per-arm state machine.
   - Owns: `rb_simulator/src`, `rb_simulator/config`, simulator unit tests.
   - Acceptance: each simulator can load config, start on loopback, maintain
     one arm state, advance actual joints toward targets deterministically, and
     expose stop/reset/fault state transitions without touching hardware or
     Docker.

2. Add simulator protocol server and fault-injection admin API.
   - Owns: `rb_simulator/src`, `rb_simulator/tests`.
   - Acceptance: JSON Lines control operations support connect, initialize,
     read_state, send_servo_j, stop, reset_fault; admin operations can inject
     disconnect, invalid state, send failure, stop failure, reset failure,
     tracking bias, and latency; protocol tests are deterministic.

3. Integrate `rb_servo_server` with the simulator backend.
   - Owns: `rb_servo_server/include/rb_servo/robot`,
     `rb_servo_server/src/robot`,
     `rb_servo_server/src/config`,
     `rb_servo_server/config`.
   - Acceptance: `backend_type: simulator` parses, backend maps simulator
     responses into `RobotState`, and simulator config runs without
     `RB_SERVO_ENABLE_RBPODO`, `RB_ALLOW_REAL_ROBOT`, hardware, or Docker.

4. Add rb_simulator smoke tooling and operator docs.
   - Owns: `rb_simulator/tools` and docs under `rb_simulator/docs`.
   - Acceptance: one command starts simulator plus servo-server simulation profile,
     sends a small joint target, verifies state-stream truthfulness, and writes
     a bounded log artifact; docs state what this does and does not prove.
   - Current artifact: `tools/rbsim_servo_smoke.py` plus
     `docs/operator_smoke.md`. The runner intentionally fails closed until the
     simulator executable, protocol server, and C++ simulator backend/config
     tasks are complete.

## Test Tasks

1. Add hardware-free simulator CI gate.
   - Extend the existing hardware-free validation path to build/test
     `rb_simulator` and the simulator backend with downloads disabled where
     practical.
   - Acceptance: the gate exercises only loopback/local processes and skips
     OVA, privileged Docker, Rainbow simulator, RealSense, and real robot
     operations.

2. Add fault-regression coverage.
   - Test invalid state, stale/disconnected state, send failure, stop failure,
     reset failure, and tracking error through `rb_servo_server`.
   - Acceptance: each injected fault produces a truthful state snapshot and the
     expected hold/fault latch behavior.

3. Add timing and log analyzer coverage for simulator mode.
   - Define a short local profile and a longer optional profile.
   - Acceptance: analyzer fails closed on missing send columns, dropped samples,
     bad rate, high jitter, high skew, send failures, and excessive tracking
     error.

## Review Task

Review the complete simulator/backend/test change set before any human gate.

Required review focus:

- simulator protocol cannot accidentally target real hardware
- real-mode guard remains intact
- simulator config binds only to loopback by default
- fault injection is deterministic and test-only
- stop/reset/state truthfulness matches existing safety policy
- docs do not claim hardware readiness from simulator-only evidence

## Blocked Hardware-Gated Tasks

1. Validate against the real Rainbow Robotics simulator stack.
   - Blocked on explicit human approval, OVA/image availability, network
     topology, and confirmation that running the external simulator is allowed.

2. Validate real robot stop/reset/motion gates.
   - Blocked on named operator, physical E-stop, clear robot workspace,
     `RB_ALLOW_REAL_ROBOT=1`, realtime setup, and human-supervised runbook.

3. Approve any privileged Docker or exposed network deployment.
   - Blocked on a deployment review because the reference simulator stack uses
     privileged containers and fixed bridge networking.
