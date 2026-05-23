# Codex Migration Prompts: MIG-13+

These prompts continue after `MIG-00` through `MIG-12`.

## Recommended execution

Because the current `scripts/codex_gate.sh` may not yet know `MIG-13`, run the first prompt with the gate skipped:

```bash
CODEX_SKIP_GATE=1 ./scripts/codex_run_sequence.sh MIG-13
```

Then run the rest normally:

```bash
./scripts/codex_run_sequence.sh   MIG-14 MIG-15 MIG-16 MIG-17 MIG-18 MIG-19   MIG-20 MIG-21 MIG-22 MIG-23 MIG-24 MIG-25 MIG-26
```

For overnight exploration only:

```bash
CODEX_CONTINUE_ON_GATE_FAIL=1 ./scripts/codex_run_sequence.sh   MIG-14 MIG-15 MIG-16 MIG-17 MIG-18 MIG-19   MIG-20 MIG-21 MIG-22 MIG-23 MIG-24 MIG-25 MIG-26
```

## Sequence summary

- `MIG-13`: Persistent RbsimBackend JSON-line transport, buffered recv, MIG-13+ gate bootstrap.
- `MIG-14`: ArmWorker latest-wins command drop counters and worker telemetry.
- `MIG-15`: RbpodoBackend read-state semantics and real read-only acceptance hardening.
- `MIG-16`: Preserve latched FaultContext with backend details in state/log output.
- `MIG-17`: policy_runner runtime wiring, geometry loading, source shutdown, startup timeout.
- `MIG-18`: Stale config/docs cleanup and canonical naming rebaseline.
- `MIG-19`: Simulator Docker exposure strategy, either compose 0.0.0.0 bind or documented socat.
- `MIG-20`: Runnable TCP pose simulator acceptance config and stronger Pinocchio gate.
- `MIG-21`: TCP pose quaternion publishing and RPY-as-display convention.
- `MIG-22`: IK/FK latency telemetry and budget enforcement for simulator acceptance.
- `MIG-23`: Simulator worker-mode resetFault lifecycle path and lifecycle/servo queue separation.
- `MIG-24`: Command source lease/arbitration design and initial enforcement.
- `MIG-25`: Camera acceptance/runbook rebaseline and policy_runner camera readiness hooks.
- `MIG-26`: Final rebaseline for MIG-13+ and developer environment reproducibility.

## Notes

These prompts intentionally avoid quick test-specific patches. The focus is structural: persistent per-arm simulator transport, explicit worker telemetry, real read-only semantics, latched fault truth, action source runtime wiring, and acceptance-grade documentation.

---

# MIG-13

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-13: persistent RbsimBackend JSON-line transport, buffered receive, and gate bootstrap for MIG-13+.

Context:
MIG-00 through MIG-12 moved the control layer from bool backend calls to structured BackendResult/SendServoJResult and introduced ArmWorker. However, RbsimBackend::controlRequest still opens a new TCP connection for every operation. At worker read rates around 1 ms and servo rates around 100-200 Hz, that can mean thousands of TCP connect/send/recv/close cycles per second. ArmWorker moves blocking I/O off the ServoLoop thread, but it does not reduce syscall count unless the simulator backend keeps a persistent socket.

Allowed scope:
- rb_servo_server/include/rb_servo/robot/rbsim_backend.hpp
- rb_servo_server/src/robot/rbsim_backend.cpp
- rb_servo_server/include/rb_servo/robot/**, only for a small JsonLineTcpClient helper if you split files
- rb_servo_server/src/robot/**, only for that helper
- rb_servo_server/tests/*rbsim* or rb_servo_server/tests/test_rbsim_backend.cpp
- rb_servo_server/tests/test_backend_result.cpp, only if telemetry/result helpers need tests
- scripts/codex_gate.sh, to add MIG-13 through MIG-26 cases
- docs/servo_backend_contract.md and docs/architecture.md, only to document persistent simulator transport

Do not modify:
- rbpodo backend behavior
- ServoLoop fault policies
- ArmWorker queue policy
- GUI or policy_runner
- real robot configs

Goals:
1. Replace per-operation RbsimBackend TCP connection open/close with a persistent JSON-lines client.
2. Keep one persistent socket per RbsimBackend instance, which corresponds to one arm endpoint.
3. Support multiple request/response lines on the same socket. The Python simulator server already handles multiple lines per accepted connection.
4. Add buffered recvLine() that reads chunks, not one byte per syscall.
5. Close and reconnect the socket on transport-level failures:
   - TransportConnectFailed
   - TransportWriteFailed
   - TransportReadFailed
   - TransportTimeout
   - ProtocolError, conservatively
6. Do NOT close the socket for controller-level/protocol-successful robot errors:
   - RobotFault
   - ServoDisabled
   - WrongMode
   - WrongArm
   - InvalidTarget
   because those are meaningful backend responses, not TCP transport corruption.
7. Preserve existing BackendResult / SendServoJResult semantics, including state_after="response" when simulator error responses include state.
8. Add transport counters, at least internally and preferably state/log-visible later:
   - connections_opened_total
   - reconnects_total
   - requests_total
   - read_syscalls_total
   - write_syscalls_total
   - last_transport_error_kind
9. Update scripts/codex_gate.sh so it recognizes MIG-13 through MIG-26. MIG-13 may be run initially with CODEX_SKIP_GATE=1 because the old gate does not know it yet.

Implementation guidance:
- Add a small class such as JsonLineTcpClient inside rbsim_backend.cpp or a separate file:
  - connectIfNeeded(timeout_sec)
  - request(line, timeout_sec, BackendOp)
  - close()
  - recvLineBuffered()
- Avoid getaddrinfo on every request. Resolve lazily on connect or cache the parsed endpoint. Re-resolve on reconnect is acceptable.
- Use SO_RCVTIMEO and SO_SNDTIMEO on the persistent fd. If different ops require different timeouts, update socket timeouts before the op.
- Buffered recv should accumulate into read_buffer_ and extract one line at a time.
- request_id must continue to increase monotonically per RbsimBackend instance.
- Ensure destructor or backend shutdown path closes fd.
- If simulator restarts and the socket breaks, the next request should return a transport error and later requests should reconnect.

Suggested codex_gate.sh mapping for new tasks:
- MIG-13, MIG-14, MIG-15, MIG-16, MIG-20, MIG-21, MIG-22, MIG-23, MIG-24: run_servo_gate, plus optional Pinocchio where relevant.
- MIG-17: run_policy_runner_tests.
- MIG-18, MIG-19, MIG-25, MIG-26: shell/doc checks plus relevant Python tests.
- Never require real robot hardware.

Acceptance criteria:
- RbsimBackend can perform connect, initialize, repeated readState, repeated sendServoJ, stop, and resetFault over a reused socket.
- A test verifies that many sequential requests do not open one socket per request. Use an injectable test hook or a simulator-side connection counter if practical.
- A test verifies that RobotFault/fault_latched response does not force transport close.
- A test verifies that transport failure closes the socket and the next request reconnects.
- Existing direct and worker rbsim hardware-free gates remain green.
- No real robot behavior changes.

Required tests:
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure

Final response format:
1. Summary
2. Files changed
3. Persistent transport design
4. Transport close/reconnect policy
5. Counters added
6. Tests run
7. Test results
8. Remaining TODOs

```

---

# MIG-14

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-14: ArmWorker latest-wins drop counters and worker telemetry.

Context:
ArmWorker intentionally uses a latest-wins servo target policy. If a new send request arrives while another pending request has not yet been dispatched, pending_servo_j_ is overwritten. That can be acceptable for streaming servo targets, but it must not be silent. Operators and acceptance tests need to know when commands were dropped/superseded.

Allowed scope:
- rb_servo_server/include/rb_servo/control/arm_worker.hpp
- rb_servo_server/src/control/arm_worker.cpp
- rb_servo_server/include/rb_servo/core/types.hpp, only if state structs need telemetry fields
- rb_servo_server/src/control/dual_arm_servo_loop.cpp, only to propagate telemetry
- rb_servo_server/src/network/state_publisher.cpp
- rb_servo_server/tests/test_arm_worker.cpp
- rb_servo_server/tests/test_state_publisher.cpp or equivalent
- docs/servo_backend_contract.md

Do not modify:
- RbsimBackend persistent transport
- RbpodoBackend behavior
- policy_runner
- GUI except if state schema tests require parser compatibility; prefer not to touch GUI in this task

Goals:
1. Make latest-wins behavior explicit and measurable.
2. Add ArmWorker telemetry counters:
   - worker_command_drops_total
   - worker_pending_overwrites_total
   - worker_last_dropped_seq
   - worker_last_enqueued_seq
   - worker_last_dispatched_seq
   - worker_last_completed_seq
   - worker_queue_policy: "latest_wins"
3. Publish these fields in rb_servo_server state JSON per arm when worker mode is enabled. It is acceptable to publish zero/default values in direct mode for schema stability.
4. Ensure overwriting a pending command increments the counters exactly once per dropped command.
5. Keep latest-wins behavior; do not convert to an unbounded queue.
6. Do not treat command drops as immediate faults by default, but make them visible. A later policy may fault on excessive drops.

Implementation guidance:
- Add an ArmWorkerTelemetry struct if useful.
- ArmWorker::enqueueServoJ should check whether a pending request already exists before overwrite.
- If pending exists and is not the same seq, increment drops and store dropped seq.
- Protect counters under the same mutex or atomic fields.
- Add latest telemetry to ArmWorker::snapshot or an equivalent access method.
- StatePublisher should include telemetry under each arm, for example:
  "worker": {
    "enabled": true,
    "queue_policy": "latest_wins",
    "command_drops_total": 3,
    "pending_overwrites_total": 3,
    "last_dropped_seq": 1201,
    "last_enqueued_seq": 1204,
    "last_dispatched_seq": 1204,
    "last_completed_seq": 1204
  }

Acceptance criteria:
- Unit test: enqueue two requests while one is pending; first is counted as dropped.
- Unit test: no drop is counted when worker immediately dispatches the first request before the second arrives.
- Unit test: completed seq and dispatched seq update correctly.
- State JSON includes worker telemetry and remains parseable.
- Existing worker tests remain green.

Required tests:
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure

Final response format:
1. Summary
2. Files changed
3. Queue/drop semantics
4. State JSON fields added
5. Tests run
6. Test results
7. Remaining TODOs

```

---

# MIG-15

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-15: RbpodoBackend read-state semantics and read-only real acceptance hardening.

Context:
A real robot controller can be connected and return valid joint feedback while servo/motion is not enabled. That should be a successful readState with motion readiness false, not a transport/read failure. Motion-readiness failures should reject sendServoJ, not prevent state acquisition.

Allowed scope:
- rb_servo_server/src/robot/rbpodo_backend.cpp
- rb_servo_server/include/rb_servo/robot/rbpodo_backend.hpp, only if needed
- rb_servo_server/tests/test_rbpodo_backend_contract.cpp or existing tests with fake/stub backend helpers
- rb_servo_server/docs/first_real_robot_motion.md
- rb_servo_server/docs/servo_backend_contract.md
- README.md and docs/architecture.md, only to document real read-only semantics

Do not modify:
- RbsimBackend
- ArmWorker
- ServoDispatcher
- GUI/policy_runner
- Any real-motion enablement gates

Goals:
1. Distinguish state acquisition success from motion readiness.
2. readState().ok should mean: the backend successfully communicated with the controller and returned a finite, internally consistent RobotState.
3. servo_enabled=false should not by itself make readState().ok=false.
4. RobotState should expose servo_enabled=false and lifecycle_state / motion readiness truth.
5. sendServoJ should reject with ServoDisabled or WrongMode if motion is not ready, using latest/cached state if available.
6. Real read-only mode must be able to connect/read q_actual and publish state without attempting motion mode or servo_j.
7. stop() and resetFault() should remain DependencyUnavailable unless verified rbpodo APIs are actually wired. Do not fake them.

Implementation guidance:
- Review RbpodoBackend::errorFromSystemState or equivalent. Do not map servo_enabled=false to readState failure.
- Keep true robot error/fault states explicit:
  - If the controller reports emergency/soft-estop/collision/self-collision, return ok=true with has_error=true if q state is valid, or ok=false only if state itself is invalid/unreadable.
  - If q values are non-finite or missing, readState().ok=false with InvalidJointState.
  - If data_channel->request_data fails, readState().ok=false with TransportReadFailed or TransportTimeout.
- sendServoJ should check RB_ALLOW_REAL_MOTION=1 first. If not allowed, return SuppressedByPolicy.
- If latest state says servo_enabled=false or not motion-ready, sendServoJ returns accepted=false with BackendErrorKind::ServoDisabled or WrongMode.
- Document that real read-only acceptance expects servo_enabled may be false while q_actual is valid.

Acceptance criteria:
- Fake/stub rbpodo read with valid q and servo_enabled=false returns BackendResult.ok=true and state.servo_enabled=false.
- Fake/stub send in servo_disabled state returns SendServoJResult.accepted=false with ServoDisabled/WrongMode, not TransportWriteFailed.
- real + send_servo_commands=false still requires RB_ALLOW_REAL_ROBOT=1.
- real motion still requires RB_ALLOW_REAL_MOTION=1.
- stop/resetFault remain honest DependencyUnavailable unless actually verified.

Required tests:
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure

If rbpodo headers are installed, also run the RB_SERVO_ENABLE_RBPODO=ON build. If unavailable, report that hardware SDK build was not verified.

Final response format:
1. Summary
2. Files changed
3. readState semantics before/after
4. sendServoJ motion-readiness behavior
5. Tests run
6. Test results
7. Remaining real-hardware acceptance TODOs

```

---

# MIG-16

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-16: preserve latched FaultContext with backend details in state/log output.

Context:
MIG-06 added last_read/last_send telemetry. However, after a fault is latched, regular servo_j may be suppressed and last_send can later show SuppressedByPolicy. That can overwrite the most important diagnostic: the first fault that caused the latch. Preserve the original fault context separately.

Allowed scope:
- rb_servo_server/include/rb_servo/control/fault_classifier.hpp
- rb_servo_server/src/control/fault_classifier.cpp
- rb_servo_server/include/rb_servo/control/dual_arm_servo_loop.hpp
- rb_servo_server/src/control/dual_arm_servo_loop.cpp
- rb_servo_server/include/rb_servo/core/types.hpp, only for state/log fields
- rb_servo_server/src/network/state_publisher.cpp
- rb_servo_server/src/logging/** if servo logs include state/fault columns
- rb_servo_server/tests/test_fault_classifier.cpp
- rb_servo_server/tests/test_state_publisher.cpp or equivalent
- rb_servo_server/tests/test_rbsim_hardware_free_gate.py if it validates JSON schema
- docs/servo_backend_contract.md

Do not modify:
- backend transport behavior
- RbpodoBackend API calls
- GUI/policy_runner unless tests require defensive parser updates; prefer not to touch them

Goals:
1. Store the fault context that caused the fault latch:
   - verdict
   - domain
   - arm
   - backend_op
   - backend_error_kind
   - backend_error_name
   - backend_error_code
   - retryable
   - recoverable
   - robot_fault
   - transport_fault
   - state_after_source
   - reason
2. Do not let later SuppressedByPolicy send results overwrite the latched fault context.
3. Publish top-level JSON:
   "fault_context": {
     "latched": true,
     "verdict": "RobotStateError",
     "domain": "RobotState",
     "arm": "left",
     "backend_op": "SendServoJ",
     "backend_error_kind": "RobotFault",
     "backend_error_name": "fault_latched",
     "backend_error_code": "2222",
     "recoverable": true,
     "retryable": false,
     "state_after_source": "response",
     "reason": "..."
   }
4. Keep arm-level last_read/last_send as live telemetry.
5. Reset latched context only on explicit successful resetFault flow or server restart.

Implementation guidance:
- Add std::optional<FaultContext> latched_fault_context_ or equivalent to DualArmServoLoop.
- On latch transition from non-fault to fault, store the context returned by FaultClassifier.
- If already latched, do not replace it unless the new context is EmergencyStop/EmergencyLatched and policy says emergency overrides previous fault. If you implement override, document and test it.
- StatePublisher should serialize unavailable fields as null or omit them consistently.

Acceptance criteria:
- Test: robot fault causes latched fault_context.backend_error_kind=RobotFault.
- Test: after fault-latched send suppression, top-level fault_context still shows original RobotFault, while arm.last_send may show SuppressedByPolicy.
- Test: transport failure context shows TransportWriteFailed or TransportTimeout.
- Test: reset clears context if reset path succeeds in simulator/direct mode.
- Existing GUI parser should not break if fault_context gains fields.

Required tests:
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure
python3 -m unittest discover rb_gui/tests

Final response format:
1. Summary
2. Files changed
3. FaultContext schema
4. Latch/override/reset policy
5. Tests run
6. Test results
7. Remaining TODOs

```

---

# MIG-17

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-17: policy_runner runtime wiring, geometry loading, source shutdown, and startup timeout.

Context:
policy_runner contains action source modules beyond hold/joint_sine/joint_velocity, but runtime factory wiring may still only instantiate a subset. Geometry-aware SafetyGate also needs actual runtime geometry loading. A runner that waits forever for the first state packet is hard to diagnose.

Allowed scope:
- policy_runner/policy_runner/main.py
- policy_runner/policy_runner/config.py
- policy_runner/policy_runner/geometry.py
- policy_runner/policy_runner/safety.py, only if needed
- policy_runner/policy_runner/action_sources/__init__.py
- policy_runner/tests/**
- policy_runner/README.md

Do not modify:
- rb_servo_server
- rb_simulator
- rb_gui
- camera_server

Goals:
1. Wire all existing action sources into policy_runner runtime factory:
   - hold
   - joint_sine
   - joint_velocity
   - spacemouse_joint_velocity
   - tcp_delta
   - spacemouse_cartesian
2. Load geometry/calibration status from config and pass it into SafetyGate.
3. Add startup_timeout_sec so policy_runner fails clearly if no robot state arrives within the configured time.
4. Close action sources/readers on shutdown if they expose close().
5. Keep real Cartesian blocked by default.
6. Keep real motion blocked unless explicit allow_real_motion is true.
7. Keep real SpaceMouse HID dependency optional; tests must not require hardware.

Implementation guidance:
- Config example:
  runtime:
    startup_timeout_sec: 5.0
- If existing config has no runtime section, add a backward-compatible field under policy_runner or robot_state.
- If geometry.path is empty or missing, joint-only actions should still run.
- Geometry-dependent actions should fail closed with a clear SafetyDecision reason.
- Add tests for each action_source string to ensure make_action_source does not raise unexpectedly.
- Add tests that fake no state until startup timeout and assert a clean error/exit path.
- Add tests that a fake closeable action source has close() called.

Acceptance criteria:
- policy_runner can instantiate all six action sources through config.
- tcp_delta and spacemouse_cartesian are blocked in real mode by default.
- geometry is loaded and affects geometry-dependent action decisions.
- startup timeout prevents infinite wait for first state.
- Unit tests pass without SpaceMouse hardware.

Required tests:
python3 -m unittest discover policy_runner/tests

Final response format:
1. Summary
2. Files changed
3. Action sources wired
4. Geometry/startup timeout behavior
5. Tests run
6. Test results
7. Remaining TODOs

```

---

# MIG-18

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-18: stale config/docs cleanup and canonical naming rebaseline.

Context:
Latest code may already have removed the unsafe dual_real.yaml file, but stale docs and gate references can still mention it. Old simulator configs such as dual_rbsim.yaml, dual_rb_simulator.yaml, and rb_simulator/config/dual_rb3_730e.yaml can look runnable even if they are historical or compatibility-only. Clean this up so operators and future agents do not pick the wrong file.

Allowed scope:
- README.md
- AGENTS.md
- docs/**
- rb_servo_server/README.md
- rb_servo_server/docs/**
- rb_servo_server/config/*.yaml
- rb_simulator/README.md
- rb_simulator/docs/**
- rb_simulator/config/*.yaml
- scripts/codex_gate.sh
- scripts/hardware_free_validation.sh, only if config names change

Do not modify:
- C++ or Python runtime source code
- tests except config-contract tests if necessary
- real robot SDK integration

Goals:
1. Remove stale references to runnable `dual_real.yaml`.
2. Standardize real config guidance:
   - tracked template: rb_servo_server/config/dual_real.example.yaml
   - user local read-only config: rb_servo_server/config/local/dual_real_readonly.yaml
   - user local motion config: rb_servo_server/config/local/dual_real_motion.yaml
3. Ensure any mention of real robot IPs is limited to templates/docs that explicitly state safety gates.
4. Mark old simulator configs as deprecated compatibility-only or move them to docs/archive/configs/.
5. Make canonical simulator configs clear:
   - dual_simulator.yaml
   - dual_simulator_compose.yaml
   - dual_simulator_worker.yaml
   - rb_simulator/config/left_rb3_730e.yaml
   - rb_simulator/config/right_rb3_730e.yaml
6. Decide how to handle rb_simulator/config/dual_rb3_730e.yaml:
   - remove if non-runnable with single-arm schema, or
   - rename to dual_rb3_730e.legacy.NOT_RUNNABLE.yaml, or
   - add a loud header saying it is historical and not used by current per-arm topology.
7. Update scripts/codex_gate.sh so it checks current canonical files and does not require deleted files.

Acceptance criteria:
- `grep -R "dual_real.yaml" README.md docs rb_servo_server rb_simulator scripts` either returns nothing or only clearly historical/deprecated notes.
- No tracked config with old placeholder IPs 192.168.0.10/11 remains.
- Canonical simulator configs are documented in one place.
- Deprecated rbsim/rbsim_local names are documented with a removal plan.
- The repo does not present any legacy dual-arm simulator config as the recommended path.

Suggested checks:
grep -R "192\.168\.0\.1[01]" README.md docs rb_servo_server rb_simulator scripts || true
grep -R "dual_real.yaml" README.md docs rb_servo_server rb_simulator scripts || true
grep -R "rbsim_local" README.md docs rb_servo_server/config rb_simulator/config || true
bash -n scripts/codex_gate.sh

Final response format:
1. Summary
2. Files changed
3. Configs deprecated/moved/removed
4. Checks run
5. Remaining TODOs

```

---

# MIG-19

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-19: simplify or explicitly document simulator Docker exposure strategy.

Context:
The current Docker entrypoint may use a socat bridge from internal loopback ports to external container ports. This preserves loopback-only binding inside the simulator, but adds fragility and an extra dependency. The code already supports RB_SIMULATOR_ALLOW_NON_LOOPBACK=1. For a controller-like simulator topology, compose-specific 0.0.0.0 bind configs may be simpler and more recognizable.

Allowed scope:
- docker-compose.yml
- rb_simulator/docker/**
- rb_simulator/config/*compose*.yaml
- rb_simulator/config/left_rb3_730e_compose.yaml
- rb_simulator/config/right_rb3_730e_compose.yaml
- rb_simulator/README.md
- docs/architecture.md
- docs/hardware_free_validation.md
- scripts/hardware_free_validation.sh, only if startup commands change
- rb_simulator/tests/**, only if config contract tests are added

Do not modify:
- rb_servo_server core logic
- rbpodo backend
- GUI/policy_runner

Decision required:
Choose exactly one strategy and implement it consistently.

Preferred strategy A: compose-specific non-loopback bind
- Add left/right compose simulator configs with:
  control_bind: tcp://0.0.0.0:50200
  admin_bind: tcp://0.0.0.0:50201
- Compose services set RB_SIMULATOR_ALLOW_NON_LOOPBACK=1.
- Remove socat bridge complexity if no longer needed.
- Dockerfile no longer needs socat.

Alternative strategy B: keep socat bridge but document it
- Keep simulator process loopback-only.
- Document internal/external ports and why socat exists.
- Add config tests that prevent accidental port rewrite breakage.
- Keep socat dependency explicit.

Goals:
1. Make simulator exposure understandable to operators.
2. Keep default host-run simulator loopback-only.
3. Ensure compose simulator services expose controller-like endpoints to rb_servo_server.
4. Do not use real robot IPs for simulator.
5. Keep wrong-arm rejection intact.

Acceptance criteria:
- `docker compose up rb_simulator_left rb_simulator_right rb_servo_server` uses a clear, documented endpoint strategy.
- No hidden YAML string-rewrite behavior exists unless explicitly documented and tested.
- Host-run configs remain loopback-only by default.
- Compose-run configs are clear about 0.0.0.0 exposure or socat bridge.
- rb_servo_server compose config points to rb_simulator_left/right services, not localhost.

Required tests:
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
bash -n scripts/hardware_free_validation.sh
./scripts/check_deps.sh --profile hardware-free || true

If Docker is available, run the relevant compose config validation. If not, report that compose runtime was not verified.

Final response format:
1. Summary
2. Strategy chosen
3. Files changed
4. Security/exposure behavior
5. Tests run
6. Test results
7. Remaining TODOs

```

---

# MIG-20

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-20: runnable TCP pose simulator acceptance config and stronger Pinocchio gate.

Context:
The TCP pose acceptance script/runbook exists, but it can fail early if the selected config lacks kinematics, IK, and cartesian_control sections. Add a canonical simulator-only TCP acceptance config and gate support so P3 functionality can be validated intentionally.

Allowed scope:
- rb_servo_server/config/dual_simulator_tcp_acceptance.yaml
- rb_servo_server/config/dual_simulator_kinematics.yaml, if you prefer this name
- scripts/tcp_pose_simulator_acceptance.sh
- scripts/codex_gate.sh
- docs/runbooks/tcp_pose_simulator_acceptance.md
- docs/hardware_free_validation.md
- rb_servo_server/tests/*kinematics* or acceptance config tests

Do not modify:
- Pinocchio algorithms except minimal config hooks
- CartesianController logic unless tests expose a real bug
- real configs
- GUI/policy_runner

Goals:
1. Add a canonical simulator-only config with:
   - backend_type: simulator
   - run_mode: simulation
   - per-arm simulator endpoints
   - kinematics.enable: true
   - kinematics.publish_tcp: true
   - kinematics.ik.enable: true
   - cartesian_control.enable: true
   - cartesian_control.allow_in_simulation: true
   - cartesian_control.allow_in_real: false
   - servo.send_servo_commands: true
2. Update tcp_pose_simulator_acceptance.sh to default to this config.
3. Update the script to fail early if Pinocchio is unavailable, with a clear message, unless a flag such as --allow-missing-pinocchio is given for syntax-only environments.
4. Add codex_gate support for optional Pinocchio ON gate:
   - If Pinocchio is installed, build with RB_SERVO_ENABLE_PINOCCHIO=ON and run relevant tests.
   - If not installed, skip with a clear message unless the task explicitly requires it.
5. Acceptance script should exercise at least:
   - FK TCP state becomes non-null
   - small TcpDeltaStand command
   - unreachable target returns IkFailed/Cartesian failure without crash

Acceptance criteria:
- Config contract test verifies the acceptance config contains required sections and never uses real IPs.
- Script defaults to the new TCP acceptance config.
- P3-F gate becomes more meaningful than bash -n + grep when Pinocchio is available.
- Real Cartesian remains disabled by default.

Required tests:
bash -n scripts/tcp_pose_simulator_acceptance.sh
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure

If Pinocchio is installed, also run the ON build and tcp_pose_simulator_acceptance.sh.

Final response format:
1. Summary
2. Files changed
3. Acceptance config added
4. Pinocchio gate behavior
5. Tests run
6. Test results
7. Remaining TODOs

```

---

# MIG-21

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-21: TCP pose quaternion publishing and RPY-as-display convention.

Context:
RPY/Euler angles are convenient for display but are not a robust canonical orientation representation near singularities. FK/IK internals should use SO(3)/SE(3), and state JSON should expose quaternions so GUI/policy/datasets do not depend on Euler ambiguity.

Allowed scope:
- rb_servo_server/include/rb_servo/core/types.hpp
- rb_servo_server/src/kinematics/**
- rb_servo_server/src/network/state_publisher.cpp
- rb_gui/rb_servo_gui/models.py
- rb_gui/tests/**
- policy_runner/policy_runner/robot_state_client.py, only if it parses TCP pose
- policy_runner/tests/**, only if parser tests are needed
- docs/frame_contract.md
- docs/servo_backend_contract.md

Do not modify:
- IK solver behavior except adding orientation output fields
- Command protocol input units unless explicitly documenting compatibility
- real robot backend

Goals:
1. Keep existing Pose6D xyz/rpy fields for backward compatibility and display.
2. Add quaternion orientation fields for tcp_base and tcp_stand in state JSON:
   - qx, qy, qz, qw or quaternion_xyzw
3. Document convention:
   - positions in meters
   - quaternion normalized, xyzw or wxyz explicitly chosen
   - RPY is display/legacy, not canonical control representation
4. GUI should parse quaternion if present and continue working if absent.
5. policy_runner should not require quaternion unless a future action source uses it.
6. Add tests for normalized quaternion presence when FK is enabled.

Implementation guidance:
- If internal types only store Pose6D, add an optional Quaternion field or a PoseStamped/TcpPose struct without breaking existing code.
- Do not remove RPY fields from JSON.
- Avoid recomputing quaternion from RPY if FK already has rotation matrix/quaternion available.
- If Pinocchio is disabled, quaternion fields should be null/absent consistently with tcp pose being deferred.

Acceptance criteria:
- FK-enabled state includes quaternion for valid TCP pose.
- GUI parser handles both old and new schema.
- Docs define quaternion order and normalization.
- RPY is clearly marked display/legacy.

Required tests:
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests

If Pinocchio is installed, also run FK tests with RB_SERVO_ENABLE_PINOCCHIO=ON.

Final response format:
1. Summary
2. Files changed
3. State schema changes
4. Compatibility behavior
5. Tests run
6. Test results
7. Remaining TODOs

```

---

# MIG-22

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-22: IK/FK latency telemetry and budget enforcement for simulator acceptance.

Context:
Pinocchio FK/IK is now integrated, but real-time suitability depends on measured latency. At 200 Hz the loop period is 5 ms. IK iterations must be measured and surfaced so acceptance can distinguish a correct but too-slow path from a production-ready path.

Allowed scope:
- rb_servo_server/include/rb_servo/kinematics/**
- rb_servo_server/src/kinematics/**
- rb_servo_server/include/rb_servo/control/cartesian_controller.hpp
- rb_servo_server/src/control/cartesian_controller.cpp
- rb_servo_server/include/rb_servo/core/types.hpp
- rb_servo_server/src/network/state_publisher.cpp
- rb_servo_server/tests/test_kinematics_ik.cpp
- rb_servo_server/tests/test_cartesian_controller.cpp
- docs/runbooks/tcp_pose_simulator_acceptance.md

Do not modify:
- backend transport
- ArmWorker
- RbpodoBackend

Goals:
1. Add timing metrics for FK and IK:
   - fk_duration_us
   - ik_duration_us
   - ik_iterations
   - ik_status/reason
   - ik_timed_out
2. Add config budget fields:
   - kinematics.ik.max_duration_us or reuse timeout_ms with state telemetry
   - cartesian_control.warn_ik_duration_us
   - cartesian_control.fail_ik_duration_us, optional and simulator-only initially
3. State JSON should expose last Cartesian solve telemetry at least top-level or per-arm.
4. TCP acceptance script/runbook should record IK latency summary.
5. Do not turn latency warning into real fault unless explicitly configured; start with telemetry + acceptance threshold.

Implementation guidance:
- Extend IkResult with duration_us if not already present.
- CartesianController should propagate IK telemetry into control result.
- DualArmServoLoop should include latest cartesian telemetry in state snapshot.
- Use monotonic/steady clock.
- Keep timing overhead small.

Acceptance criteria:
- IK tests verify duration_us and iterations are populated on success/failure.
- State JSON includes latency telemetry after TCP command attempts.
- Acceptance runbook includes thresholds and artifact fields.
- No behavior change for joint-only commands.

Required tests:
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure

If Pinocchio is installed, run ON gate and capture IK latency test output.

Final response format:
1. Summary
2. Files changed
3. Telemetry fields added
4. Budget behavior
5. Tests run
6. Test results
7. Remaining TODOs

```

---

# MIG-23

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-23: simulator worker-mode resetFault lifecycle path and lifecycle/servo queue separation.

Context:
Worker mode currently handles streaming servo_j but may not expose resetFault in a way that lets simulator fault injection -> reset scenarios run through the worker path. Streaming servo targets and lifecycle commands have different semantics and should not share a silent latest-wins queue.

Allowed scope:
- rb_servo_server/include/rb_servo/control/arm_worker.hpp
- rb_servo_server/src/control/arm_worker.cpp
- rb_servo_server/include/rb_servo/control/servo_dispatcher.hpp
- rb_servo_server/src/control/servo_dispatcher.cpp
- rb_servo_server/src/control/dual_arm_servo_loop.cpp
- rb_servo_server/tests/test_arm_worker.cpp
- rb_servo_server/tests/test_rbsim_hardware_free_gate.py
- docs/servo_backend_contract.md

Do not modify:
- real worker enablement; worker mode must remain blocked in real mode unless accepted separately
- RbpodoBackend resetFault implementation
- GUI/policy_runner

Goals:
1. Add a worker lifecycle command path for simulator/mock worker mode:
   - resetFault
   - stop, if already meaningful for simulator/mock
2. Do not send lifecycle commands through the latest-wins servo target slot.
3. Keep servo_j stream latest-wins.
4. Lifecycle commands should be ordered and not silently overwritten. A small bounded queue is acceptable.
5. resetFault through worker mode should allow rbsim fault injection -> reset -> valid state again in simulator tests.
6. Real worker mode remains refused by config parser.

Implementation guidance:
- Define ArmWorkerCommand with kind ServoJ | ResetFault | Stop if helpful.
- Use separate queues:
  - servo target: single latest-wins slot
  - lifecycle queue: bounded FIFO, small size, no silent drop; if full, return structured error
- Add result structs for lifecycle command results or reuse BackendResult<RobotState>.
- ServoLoop resetFault path should use worker lifecycle command when io_model=worker.
- Keep emergency stop semantics conservative.

Acceptance criteria:
- Worker-mode rbsim fault injection can reset successfully through server command path.
- ResetFault is not silently dropped by servo target overwrite.
- Lifecycle queue full behavior is explicit and tested.
- Real mode still rejects io_model=worker.
- Existing direct-mode reset behavior remains unchanged.

Required tests:
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure
./scripts/hardware_free_validation.sh

Final response format:
1. Summary
2. Files changed
3. Worker lifecycle queue design
4. Tests run
5. Test results
6. Remaining TODOs

```

---

# MIG-24

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-24: command source lease/arbitration design and initial enforcement.

Context:
GUI, policy_runner, and SpaceMouse teleop can all become command sources. Sequence numbers alone are not enough because a newly-started source with seq=1 can be rejected as stale after another source sent seq=1000, or worse, multiple sources can fight for control. Introduce command source identity and a lease concept.

Allowed scope:
- rb_servo_server/src/network/command_server.cpp
- rb_servo_server/include/rb_servo/network/**
- rb_servo_server/include/rb_servo/core/types.hpp
- rb_servo_server/src/control/dual_arm_servo_loop.cpp, only command metadata propagation
- rb_servo_server/src/network/state_publisher.cpp
- rb_servo_server/tests/test_command_parser.cpp or equivalent
- rb_servo_server/docs/network_protocol.md
- policy_runner/policy_runner/servo_command_client.py
- rb_gui/rb_servo_gui/command_client.py, only to include source_id/session_id fields defensively

Do not modify:
- motion generation behavior
- backend transport
- real robot gates

Goals:
1. Extend command schema with optional fields:
   - source_id
   - session_id
   - lease_token
   - source_priority, optional future field
2. Implement initial single active normal-motion source lease in rb_servo_server:
   - A source may acquire lease via ArmMotion or a dedicated AcquireLease command.
   - Normal motion commands require the active lease when lease enforcement is enabled.
   - EmergencyStop should bypass lease.
   - ResetFault may require lease or operator source; document the policy.
3. Add config:
   command_source:
     enforce_lease: false by default initially, true in simulator acceptance config if tests pass
     lease_timeout_sec: 1.0
4. Publish active command source in state JSON.
5. Keep backward compatibility when enforce_lease=false.
6. Update policy_runner and GUI command clients to include source_id/session_id so migration can later enforce lease by default.

Implementation guidance:
- This task may start with parser + state + optional enforcement. Do not break existing tests by requiring lease everywhere by default.
- Use stable defaults:
  - GUI source_id: "rb_gui"
  - policy_runner source_id: "policy_runner"
- session_id can be generated at process start.
- Lease token should be opaque string or UUID.
- Stale lease should expire based on monotonic server time.

Acceptance criteria:
- Parser accepts commands with source metadata.
- State JSON publishes active command source/lease status.
- With enforce_lease=false, existing command behavior remains compatible.
- With enforce_lease=true, normal motion from non-owner is rejected with a clear verdict/reason.
- EmergencyStop bypasses lease.
- policy_runner and GUI include source metadata in outgoing commands.

Required tests:
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure
python3 -m unittest discover policy_runner/tests
python3 -m unittest discover rb_gui/tests

Final response format:
1. Summary
2. Files changed
3. Command schema additions
4. Lease behavior
5. Backward compatibility
6. Tests run
7. Test results
8. Remaining TODOs

```

---

# MIG-25

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-25: camera acceptance/runbook rebaseline and policy_runner camera readiness hooks.

Context:
Camera server config validation is safer now, but real 3-camera operation still needs explicit acceptance criteria. policy_runner should be able to know when camera observations are required and unavailable, even if the first policy modes are joint-only.

Allowed scope:
- camera_server/docs/**
- docs/runbooks/camera_acceptance.md
- docs/architecture.md
- camera_server/config/**, only adding example/local templates or comments
- scripts/check_deps.sh
- policy_runner/policy_runner/config.py
- policy_runner/policy_runner/safety.py
- policy_runner/tests/test_geometry_safety.py or new camera readiness tests
- README.md, only link to runbook

Do not modify:
- RealSense runtime code unless a small config validation bug is found
- rb_servo_server
- rb_simulator

Goals:
1. Add a real camera acceptance runbook:
   - D435f head 1280x720@30
   - D405 wrists 640x360@30 canonical profile, with optional 640x480 variant
   - serial identification
   - 10/30/60-minute drop/skew/bundle-rate test
   - USB disconnect/restart behavior
   - artifact list
2. Make dependency profiles clear:
   - hardware-free
   - real-camera
   - real-robot
   - kinematics
3. Add policy_runner safety metadata for action sources that require camera observations:
   - requires_camera
   - requires_camera_geometry
   - camera_stale_timeout_sec
4. Joint-only action sources must not require camera.
5. Future camera-dependent action source should be blocked if camera readiness is absent/stale.

Acceptance criteria:
- Runbook exists and is linked from README/docs.
- check_deps.sh documents/checks real-camera deps separately.
- policy_runner tests verify camera-dependent dummy action is blocked when camera unavailable.
- Joint-only actions remain allowed without camera.
- No real camera device is required for tests.

Required tests:
python3 -m unittest discover policy_runner/tests
bash -n scripts/check_deps.sh

If camera_server CMake deps are available, optionally run camera mock gate and report results.

Final response format:
1. Summary
2. Files changed
3. Camera acceptance criteria
4. policy_runner safety fields
5. Tests run
6. Test results
7. Remaining TODOs

```

---

# MIG-26

```text
Read AGENTS.md, README.md, docs/architecture.md, docs/servo_backend_contract.md, and TODO.md first.

This task continues the robotics_lab backend-contract / non-blocking servo-loop migration after MIG-00 through MIG-12.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the feature gated and report the limitation.
- Prefer explicit structured results, telemetry, and tests over log-string parsing or silent behavior.
- Keep all default runnable paths hardware-free unless the prompt explicitly says otherwise.

Implement ONLY MIG-26: final rebaseline for MIG-13+ and developer environment reproducibility.

Context:
After persistent simulator transport, ArmWorker telemetry, Rbpodo read semantics, FaultContext, policy_runner wiring, config cleanup, TCP acceptance, quaternion pose, IK latency, lifecycle queues, command lease, and camera acceptance are added, the repo needs a clear rebaseline. This task should not add new features; it should make validation reproducible and docs truthful.

Allowed scope:
- README.md
- AGENTS.md
- docs/**
- scripts/check_deps.sh
- scripts/install_deps_ubuntu.sh, if absent or incomplete
- scripts/codex_gate.sh
- scripts/hardware_free_validation.sh
- scripts/tcp_pose_simulator_acceptance.sh
- .devcontainer/** or scripts/docker/**, only if adding a dev environment helper
- Makefile

Do not modify:
- core runtime source code unless fixing a typo in docs-generated constants; prefer not
- real robot behavior

Goals:
1. Update docs to reflect the current architecture:
   CommandBuffer -> ServoCoordinator/ServoLoop -> Left/Right ArmWorker -> simulator/rbpodo endpoint.
2. Document that RbsimBackend now uses persistent JSON-line transport if MIG-13 succeeded.
3. Document worker queue policy and command drop counters if MIG-14 succeeded.
4. Document Rbpodo read-only semantics and stop/resetFault operator-intervention policy.
5. Document latched FaultContext schema.
6. Document command source lease and default enforcement state.
7. Add or update dependency installation docs/scripts:
   - yaml-cpp
   - nlohmann_json
   - python packages
   - optional Pinocchio
   - optional rbpodo
   - optional RealSense/ZMQ
8. Make codex_gate.sh understand all MIG-00 through MIG-26 tasks.
9. Add a summary validation command set:
   - Python simulator tests
   - GUI tests
   - policy_runner tests
   - rb_servo_server hardware-free CTest
   - optional Pinocchio gate
   - hardware_free_validation.sh
   - tcp_pose_simulator_acceptance.sh when Pinocchio is available
10. Make sure stale statements such as "dual_real.yaml runnable config" do not remain.

Acceptance criteria:
- A new contributor can run one documented hardware-free validation command and understand missing dependencies.
- `scripts/codex_gate.sh MIG-26` runs docs/shell checks and all available hardware-free gates.
- Docs clearly separate current supported modes from future/real-hardware acceptance modes.
- No docs imply force control, real Cartesian, or real worker mode is enabled.
- No docs imply measured calibration is complete.

Suggested checks:
bash -n scripts/codex_gate.sh
bash -n scripts/check_deps.sh
bash -n scripts/hardware_free_validation.sh
bash -n scripts/tcp_pose_simulator_acceptance.sh
python3 -m unittest discover rb_simulator/tests
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
./scripts/check_deps.sh --profile hardware-free || true

Final response format:
1. Summary
2. Files changed
3. Validation commands documented
4. Dependency/env changes
5. Checks/tests run
6. Test results
7. Remaining real-hardware acceptance TODOs

```