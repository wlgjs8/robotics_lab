# Codex Migration Prompts: BackendResult + Non-blocking Servo Loop

These prompts are intended to be copied into `.codex/prompts/*.md` and executed by `scripts/codex_run_sequence.sh`.

## Recommended execution

First bootstrap gate support:

```bash
CODEX_SKIP_GATE=1 ./scripts/codex_run_sequence.sh MIG-00
```

Then run the sequence:

```bash
./scripts/codex_run_sequence.sh \
  MIG-01 MIG-02 MIG-03 MIG-04 \
  MIG-05 MIG-06 MIG-07 \
  MIG-08 MIG-09 MIG-10 MIG-11 MIG-12
```

If you want logs but do not want the sequence to stop at the first failing gate:

```bash
CODEX_CONTINUE_ON_GATE_FAIL=1 ./scripts/codex_run_sequence.sh \
  MIG-01 MIG-02 MIG-03 MIG-04 \
  MIG-05 MIG-06 MIG-07 \
  MIG-08 MIG-09 MIG-10 MIG-11 MIG-12
```

## Sequence summary

- `MIG-00`: Migration infrastructure, safety cleanup, gate setup
- `MIG-01`: Structured backend result types
- `MIG-02`: IRobotBackend structured API migration
- `MIG-03`: Structured rb_simulator error protocol + RbsimBackend mapping
- `MIG-04`: RbpodoBackend structured result hardening
- `MIG-05`: FaultClassifier and FaultContext
- `MIG-06`: Fault-latched send suppression + backend truth in state JSON
- `MIG-07`: DualSendResult dispatch boundary
- `MIG-08`: ArmWorker scaffold
- `MIG-09`: ArmWorker IO model integration behind config
- `MIG-10`: Simulator worker-mode smoke + latency metrics
- `MIG-11`: Parallel/worker dispatch semantics + deadlines
- `MIG-12`: Migration rebaseline, acceptance docs, cleanup

## Notes

- This is intentionally more structural than a quick fix for `rbsim_hardware_free_gate`.
- The migration makes backend operation results explicit before moving network I/O out of the servo loop.
- Real robot motion remains gated throughout the sequence.

---

# MIG-00: Migration infrastructure, safety cleanup, gate setup

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-00: migration infrastructure, safety cleanup, and gate setup.

Purpose:
This is the bootstrap task for the migration sequence. It must make scripts/codex_gate.sh understand the MIG-* task IDs before later migration prompts are run through scripts/codex_run_sequence.sh.

Allowed scope:
- scripts/codex_gate.sh
- scripts/codex_run_sequence.sh, only if needed for robust gate behavior
- README.md
- AGENTS.md
- docs/**
- rb_servo_server/config/dual_real.yaml
- rb_servo_server/config/dual_real.example.yaml
- rb_servo_server/config/local/.gitkeep
- .gitignore

Do not modify:
- rb_servo_server source code
- rb_simulator runtime code
- rb_gui source code
- policy_runner source code
- camera_server source code

Goals:
1. Add codex_gate.sh cases for:
   - MIG-00 through MIG-12
   - MIG-* tasks should run the safest relevant existing gates.
2. Fix the P2-B gate bug where grep includes a non-existent geometry/ directory when calibration/ was chosen.
3. Remove or hard-disable rb_servo_server/config/dual_real.yaml if it still contains old placeholder IPs such as 192.168.0.10/11.
4. Keep dual_real.example.yaml as the only tracked real-robot template and ensure it is read-only by default:
   - left IP: 172.28.60.200
   - right IP: 172.28.60.201
   - servo.send_servo_commands: false
5. Document the migration target:
   - IRobotBackend must migrate from bool operations to structured BackendResult / SendServoJResult.
   - ServoLoop must gradually stop being the owner of blocking network I/O.
   - Future target: CommandBuffer -> ServoCoordinator -> Left/Right ArmWorker.
6. Add docs/architecture/servo_backend_contract.md or a similarly named source-of-truth doc.

Suggested codex_gate.sh mapping:
- MIG-00: shell syntax checks + grep safety docs
- MIG-01, MIG-02, MIG-04, MIG-05, MIG-06, MIG-07, MIG-09, MIG-10, MIG-11, MIG-12: run_servo_gate
- MIG-03: run_simulator_tests + run_servo_gate
- MIG-08: run_servo_gate
- Optionally include run_gui_tests for MIG-06 if state schema changes touch GUI parsing.
- Never require real robot hardware in these gates.

Important:
Since codex_gate.sh may not know MIG-00 before this task, run MIG-00 initially with:
CODEX_SKIP_GATE=1 ./scripts/codex_run_sequence.sh MIG-00

Acceptance criteria:
- scripts/codex_gate.sh recognizes all MIG-* task IDs.
- P2-B gate no longer fails if geometry/ does not exist.
- Tracked real robot config cannot be mistaken for a runnable motion config.
- README or docs clearly state this migration is not a real-motion enablement.
- No real motion gate is weakened.

Required checks:
bash -n scripts/codex_gate.sh
bash -n scripts/codex_run_sequence.sh
grep -R "RB_ALLOW_REAL_MOTION" README.md docs AGENTS.md >/dev/null
grep -R "BackendResult\|SendServoJResult\|ArmWorker" README.md docs AGENTS.md >/dev/null

Final response format:
1. Summary
2. Files changed
3. Gate cases added
4. Safety cleanup performed
5. Checks run
6. Test results
7. Remaining TODOs for MIG-01

```

---

# MIG-01: Structured backend result types

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-01: add structured backend result types without changing IRobotBackend API yet.

Allowed scope:
- rb_servo_server/include/rb_servo/robot/backend_result.hpp
- rb_servo_server/src/robot/backend_result.cpp
- rb_servo_server/include/rb_servo/core/types.hpp, only if a small enum/toString hook is unavoidable
- rb_servo_server/CMakeLists.txt
- rb_servo_server/tests/test_backend_result.cpp
- rb_servo_server/docs/backend_contract.md or docs/architecture/servo_backend_contract.md

Do not modify:
- IRobotBackend method signatures
- MockBackend/RbsimBackend/RbpodoBackend behavior
- DualArmServoLoop
- StatePublisher runtime schema
- simulator protocol

Goals:
Add the result vocabulary before migrating behavior. This should be a low-risk compile-safe task.

Required types:
- enum class BackendOp:
  Connect, Initialize, ReadState, SendServoJ, Stop, ResetFault
- enum class BackendErrorKind:
  None,
  TransportConnectFailed,
  TransportWriteFailed,
  TransportReadFailed,
  TransportTimeout,
  ProtocolError,
  UnsupportedSchema,
  WrongArm,
  WrongEndpoint,
  UnknownArm,
  RobotDisconnected,
  RobotNotInitialized,
  ServoDisabled,
  WrongMode,
  RobotFault,
  InvalidJointState,
  InvalidTarget,
  ControllerRejected,
  CommandTimeout,
  DependencyUnavailable,
  SuppressedByPolicy,
  Unknown
- struct BackendError:
  kind, code, name, message, retryable, recoverable, robot_fault, transport_fault
- struct BackendTiming:
  start_ns, end_ns, duration_us
- template <typename T> struct BackendResult:
  ok, op, value, error, timing
- struct SendServoJRequest:
  q_target_deg, command_seq, host_time_ns, deadline_ns
- struct SendServoJResult:
  accepted, error, timing, state_after, state_after_source, requested_q_deg
- struct ArmSendResult
- struct DualSendResult
- toString helpers for BackendOp and BackendErrorKind
- helpers:
  BackendError noBackendError()
  BackendError backendError(...)
  BackendTiming makeBackendTiming(start_ns, end_ns)
  BackendResult<RobotState> okReadState(...)
  BackendResult<RobotState> failedReadState(...)
  SendServoJResult acceptedSend(...)
  SendServoJResult rejectedSend(...)

Design requirements:
- Do not use exceptions for normal backend operation results.
- The error taxonomy must distinguish robot/controller fault from transport failure.
- SuppressedByPolicy must not be treated as a backend failure by default.
- State-after support must be optional and explicit via state_after_source:
  "response", "cache", "none".

Acceptance criteria:
- New tests verify toString mappings.
- New tests verify timing duration.
- New tests verify RobotFault and TransportTimeout have different flags.
- Existing servo gate still passes.

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
3. New result types
4. Tests run
5. Test results
6. Remaining TODOs for MIG-02

```

---

# MIG-02: IRobotBackend structured API migration

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-02: migrate IRobotBackend from bool API to structured result API.

Dependencies:
- MIG-01 must be complete.

Allowed scope:
- rb_servo_server/include/rb_servo/robot/i_robot_backend.hpp
- rb_servo_server/include/rb_servo/robot/mock_backend.hpp
- rb_servo_server/src/robot/mock_backend.cpp
- rb_servo_server/include/rb_servo/robot/rbsim_backend.hpp
- rb_servo_server/src/robot/rbsim_backend.cpp
- rb_servo_server/include/rb_servo/robot/rbpodo_backend.hpp
- rb_servo_server/src/robot/rbpodo_backend.cpp
- rb_servo_server/src/robot/backend_factory.cpp
- rb_servo_server/src/control/dual_arm_servo_loop.cpp
- rb_servo_server/include/rb_servo/core/types.hpp, only for ServoSample/ServoSnapshot compatibility if needed
- rb_servo_server/src/network/state_publisher.cpp, only minimal compile compatibility
- rb_servo_server/tests/*backend*
- rb_servo_server/tests/test_safety_policy.cpp
- rb_servo_server/tests/test_rbsim_hardware_free_gate.py, only if test helpers need schema-compatible field names

Do not modify:
- simulator Python protocol in this task
- fault classification policy beyond minimal compatibility
- ArmWorker/non-blocking design
- GUI/policy_runner

New IRobotBackend API:
- virtual BackendResult<RobotState> connect() = 0;
- virtual BackendResult<RobotState> initialize() = 0;
- virtual BackendResult<RobotState> readState() = 0;
- virtual SendServoJResult sendServoJ(const SendServoJRequest& request) = 0;
- virtual BackendResult<RobotState> stop() = 0;
- virtual BackendResult<RobotState> resetFault() = 0;

Migration rules:
- MockBackend should return precise structured results.
- RbsimBackend may initially map protocol failure to ProtocolError / Transport* / Unknown, but must not lose error.name/code if available.
- RbpodoBackend must keep RB_SERVO_ENABLE_RBPODO=OFF builds passing.
- RbpodoBackend must not guess unavailable APIs.
- DualArmServoLoop should consume structured results but preserve current safety behavior as much as possible; more nuanced classification is MIG-05.

Important:
Do not add the short-term workaround "send failed -> readRobotStates again" here. The point of this task is to carry failure cause in SendServoJResult.

Acceptance criteria:
- No remaining calls expect bool readState(...) or bool sendServoJ(...).
- A failed backend operation retains BackendErrorKind/name/code/message.
- ServoLoop still builds and publishes state.
- Existing behavior may still classify failure coarsely until MIG-05, but the data must be available.

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
3. Backend API migration details
4. Tests run
5. Test results
6. Known behavior intentionally deferred to MIG-03/MIG-05

```

---

# MIG-03: Structured rb_simulator error protocol + RbsimBackend mapping

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-03: structured rb_simulator error response and RbsimBackend mapping.

Dependencies:
- MIG-02 must be complete.

Allowed scope:
- rb_simulator/src/rbsim/protocol.py
- rb_simulator/src/rbsim/state_machine.py, only if ArmSnapshot needs one extra field
- rb_simulator/tests/**
- rb_servo_server/src/robot/rbsim_backend.cpp
- rb_servo_server/include/rb_servo/robot/rbsim_backend.hpp
- rb_servo_server/tests/test_rbsim_hardware_free_gate.py
- rb_servo_server/tests/*backend*, only RbsimBackend cases
- docs/architecture/servo_backend_contract.md
- docs/hardware_free_validation.md

Do not modify:
- RbpodoBackend
- DualArmServoLoop fault policy except where compile requires result fields
- ArmWorker/non-blocking design
- GUI/policy_runner

Goals:
Simulator already knows whether a failure is wrong-arm, fault-latched, servo-disabled, injected transport-like failure, etc. Do not flatten this into bool false.

Protocol changes:
- Error response must include:
  error.kind
  error.name
  error.message
  error.code
  error.retryable
  error.recoverable
- Stateful robot/controller errors should include "state" in the error response:
  fault_latched
  servo_disabled
  disconnected
  not_initialized
  invalid_joint_state
- Injected transport-like failures should not pretend to be robot faults:
  send_failure_injected -> kind TransportWriteFailed
  read_failure_injected -> kind TransportReadFailed
  stop_failure_injected -> kind TransportWriteFailed or ControllerRejected only if stateful
  reset_failure_injected -> kind TransportWriteFailed or ControllerRejected only if stateful
- wrong_arm -> kind WrongArm
- wrong_endpoint -> kind WrongEndpoint
- unsupported_schema_version -> kind UnsupportedSchema
- bad JSON/protocol -> kind ProtocolError

RbsimBackend mapping:
- Parse error.kind when present.
- Preserve error.name/code/message.
- If error response includes state, set SendServoJResult.state_after and state_after_source="response".
- If state is missing, state_after_source="none".
- Map older responses without kind to reasonable fallback for backward compatibility.

Acceptance criteria:
- Python simulator tests verify stateful error responses include state.
- Wrong-arm request remains explicit.
- RbsimBackend tests verify fault_latched maps to BackendErrorKind::RobotFault with code 2222 and state_after.
- Injected send failure maps to TransportWriteFailed, not RobotFault.
- Existing rbsim hardware-free smoke should either pass or fail only because MIG-05 classification is not yet complete.

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
3. Simulator protocol changes
4. Backend error mapping
5. Tests run
6. Test results
7. Remaining TODOs for MIG-05

```

---

# MIG-04: RbpodoBackend structured result hardening

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-04: harden RbpodoBackend structured result mapping.

Dependencies:
- MIG-02 must be complete.

Allowed scope:
- rb_servo_server/src/robot/rbpodo_backend.cpp
- rb_servo_server/include/rb_servo/robot/rbpodo_backend.hpp
- rb_servo_server/tests/*rbpodo* if tests exist or can be added without hardware
- rb_servo_server/docs/rbpodo_backend_plan.md
- docs/architecture/servo_backend_contract.md
- docs/runbooks/first_real_robot_motion.md if present

Do not modify:
- RbsimBackend
- simulator Python code
- DualArmServoLoop fault policy except compile compatibility
- ArmWorker/non-blocking design
- Force control

Goals:
Make RbpodoBackend return truthful structured results. This task is not about enabling real motion. It is about preserving cause and safety information.

Rules:
- Do not guess rbpodo API names.
- If rbpodo headers are unavailable, keep RB_SERVO_ENABLE_RBPODO=OFF build green and document unverified ON behavior.
- If stop/resetFault APIs are not verified, return BackendResult with:
  ok=false
  error.kind=DependencyUnavailable or ControllerRejected
  error.name="rbpodo_stop_unverified" / "rbpodo_reset_fault_unverified"
  message explaining operator intervention is required
- initialize() in read-only mode must not enter motion mode.
- sendServoJ() must require:
  RB_ALLOW_REAL_MOTION=1
  config servo.send_servo_commands=true at the caller side or explicit request policy
- Motion gate failure should map to SuppressedByPolicy, not TransportFailure.
- Real robot fault state from SystemState should map to RobotFault with error_code.
- Disconnected/request_data failure should map to TransportReadFailed or RobotDisconnected based on what is known.

Add/verify:
- last known RobotState cache inside RbpodoBackend only if it is timestamped and clearly marked.
- If send fails and a recent cache is included, state_after_source must be "cache".
- dq_actual_deg_s remains 0 only if rbpodo has no verified velocity field; document this.

Acceptance criteria:
- RB_SERVO_ENABLE_RBPODO=OFF build/tests pass.
- RbpodoBackend no longer returns anonymous bool failures.
- Real env gate failures are SuppressedByPolicy.
- Verified robot/controller errors are RobotFault/WrongMode/ServoDisabled where distinguishable.
- stop/resetFault unverified status is visible in docs and result errors.

Required tests:
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure

If rbpodo is installed, also run a compile-only ON gate:
cmake -S rb_servo_server -B rb_servo_server/build/rbpodo_check \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=ON \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/rbpodo_check -j

Final response format:
1. Summary
2. Files changed
3. rbpodo result mapping
4. Verified vs unverified rbpodo APIs
5. Tests run
6. Test results
7. Remaining TODOs

```

---

# MIG-05: FaultClassifier and FaultContext

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-05: add FaultClassifier and FaultContext.

Dependencies:
- MIG-02 must be complete.
- MIG-03 is strongly recommended.

Allowed scope:
- rb_servo_server/include/rb_servo/control/fault_classifier.hpp
- rb_servo_server/src/control/fault_classifier.cpp
- rb_servo_server/include/rb_servo/core/types.hpp
- rb_servo_server/src/core/types.cpp if present, or equivalent toString implementation file
- rb_servo_server/src/control/dual_arm_servo_loop.cpp
- rb_servo_server/tests/test_fault_classifier.cpp
- rb_servo_server/tests/test_safety_policy.cpp
- rb_servo_server/tests/test_rbsim_hardware_free_gate.py, only to align with improved contract

Do not modify:
- simulator protocol, except if a minor test fixture update is required
- rbpodo API calls
- ArmWorker/non-blocking design
- GUI/policy_runner

Goals:
Separate backend operation classification from the main servo loop. The servo loop should not contain ad-hoc if/else chains for every backend error.

Add:
- enum class FaultDomain:
  None, SafetyPolicy, Backend, RobotState, Command, Kinematics, Emergency
- struct FaultContext:
  SafetyVerdict verdict
  FaultDomain domain
  ArmId arm
  BackendOp backend_op
  BackendError backend_error
  int robot_error_code
  std::string reason
  bool recoverable
  bool retryable
  bool suppress_regular_servo
  std::optional<RobotState> state_after
- class FaultClassifier or free functions:
  classifyReadStateResult(...)
  classifySendServoJResult(...)
  classifyDualSendResult(...)
  classifyCommandValidation(...)
  classifyIkFailure(...)

Classification policy:
- BackendErrorKind::RobotFault -> RobotStateError or a dedicated Backend/Robot fault verdict if existing enum allows; reason must mention robot/controller fault.
- ServoDisabled, WrongMode, RobotNotInitialized, InvalidJointState -> RobotStateError.
- TransportTimeout, TransportReadFailed, TransportWriteFailed, TransportConnectFailed -> SendFailure or BackendError depending on available enum; reason must say transport.
- WrongArm/WrongEndpoint -> RobotStateError or BackendError with explicit reason.
- SuppressedByPolicy -> not a failure.
- ControllerRejected/InvalidTarget -> SendFailure or InvalidCommand depending on context.
- IkFailed remains IkFailed.
- EmergencyStop remains EmergencyStop.

Important:
Do not solve classification by doing an extra read after send failure. Use SendServoJResult.error and state_after.

Acceptance criteria:
- Unit tests verify RobotFault is not classified as generic transport SendFailure.
- Unit tests verify injected transport send failure remains transport/send failure.
- Unit tests verify SuppressedByPolicy does not latch a fault.
- rbsim fault smoke should now expect and receive RobotStateError or the chosen robot-fault verdict, not generic transport failure.
- The main loop is simpler because classification is delegated.

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
3. Fault taxonomy implemented
4. Tests run
5. Test results
6. rbsim fault classification behavior
7. Remaining TODOs for MIG-06

```

---

# MIG-06: Fault-latched send suppression + backend truth in state JSON

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-06: servo send policy, fault-latched suppression, and backend truth in state JSON.

Dependencies:
- MIG-05 must be complete.

Allowed scope:
- rb_servo_server/src/control/dual_arm_servo_loop.cpp
- rb_servo_server/include/rb_servo/control/dual_arm_servo_loop.hpp
- rb_servo_server/include/rb_servo/core/types.hpp
- rb_servo_server/src/network/state_publisher.cpp
- rb_servo_server/include/rb_servo/network/state_publisher.hpp, only if needed
- rb_gui/rb_servo_gui/models.py, only backwards-compatible parser changes
- rb_gui/tests/test_gui_contracts.py, only parser compatibility
- policy_runner/policy_runner/robot_state_client.py, only tolerant parsing if needed
- policy_runner/tests/test_policy_runner_contract.py, only tolerant parsing if needed
- rb_servo_server/tests/test_safety_policy.cpp
- rb_servo_server/tests/test_rbsim_hardware_free_gate.py

Do not modify:
- backend transport implementation except compile compatibility
- ArmWorker/non-blocking design
- Cartesian/FK/IK behavior
- force control

Goals:
Fault-latched state must not keep sending regular servo_j commands. Read-only or policy-suppressed sends must be visible but not treated as backend failures.

Send policy:
- Normal running:
  send_policy = "send_servo_j"
- Read-only:
  send_policy = "read_only"
  send result should be SuppressedByPolicy and accepted=false or sent=false, but not a fault.
- Fault latched:
  send_policy = "fault_latched"
  regular servo_j suppressed.
- Emergency latched:
  send_policy = "emergency_latched"
  regular servo_j suppressed.
- Cartesian unavailable / IK failed:
  no unsafe q should be sent for that command.
- One-shot stop/reset may remain unimplemented unless verified. Do not invent stop APIs.

State JSON additions:
Top-level:
- observed_mode
- observed_backend
- send_policy
- send_suppressed
- fault_context object

Per arm:
- has_error
- servo_enabled
- fault_recoverable if available
- lifecycle_state if available
- last_read:
    ok, backend_error_kind, error_name, error_code, duration_us
- last_send:
    accepted, backend_error_kind, error_name, error_code, duration_us, state_after_source

Backward compatibility:
- Existing fields such as send_ok, error_code, has_valid_joint_state must remain.
- GUI/policy parsers must tolerate new fields.
- Do not break existing state consumers.

Acceptance criteria:
- When a simulator arm is faulted, servo_server does not keep issuing regular send_servo_j to that arm after fault latch.
- Robot/controller fault is visible as backend_error_kind=RobotFault or equivalent.
- Read-only mode publishes send_suppressed=true and does not look like send failure.
- GUI tests pass if GUI parser touched.
- policy_runner tests pass if policy parser touched.
- rbsim_hardware_free_gate passes.

Required tests:
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure

If GUI touched:
python3 -m unittest discover rb_gui/tests

If policy_runner touched:
python3 -m unittest discover policy_runner/tests

Final response format:
1. Summary
2. Files changed
3. Send policy behavior
4. State JSON fields added
5. Tests run
6. Test results
7. Remaining TODOs for MIG-07

```

---

# MIG-07: DualSendResult dispatch boundary

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-07: introduce DualSendResult dispatch boundary while keeping direct sequential implementation.

Dependencies:
- MIG-02 must be complete.
- MIG-06 should be complete.

Allowed scope:
- rb_servo_server/include/rb_servo/control/servo_dispatcher.hpp
- rb_servo_server/src/control/servo_dispatcher.cpp
- rb_servo_server/src/control/dual_arm_servo_loop.cpp
- rb_servo_server/include/rb_servo/control/dual_arm_servo_loop.hpp
- rb_servo_server/include/rb_servo/core/types.hpp
- rb_servo_server/tests/test_servo_dispatcher.cpp
- rb_servo_server/CMakeLists.txt

Do not modify:
- ArmWorker/non-blocking implementation
- backend concrete implementations except compile compatibility
- simulator protocol
- GUI/policy_runner

Goals:
Create an explicit dispatch boundary so direct sequential send can later be replaced by parallel/worker dispatch without rewriting ServoLoop.

Add:
- ServoDispatchRequest:
  left SendServoJRequest
  right SendServoJRequest
  seq
  dispatch_start_ns
  deadline_ns
- ServoDispatcher:
  dispatchDirectSequential(IRobotBackend& left, IRobotBackend& right, const ServoDispatchRequest&)
  returns DualSendResult
- DualSendResult should include:
  left ArmSendResult
  right ArmSendResult
  dispatch_start_ns
  dispatch_end_ns
  left_right_start_skew_us
  left_right_end_skew_us
  any_transport_failure()
  any_robot_fault()
- ServoLoop should call dispatcher rather than calling left_robot_->sendServoJ and right_robot_->sendServoJ inline.

Important:
- This task does not make the loop non-blocking yet.
- It creates the seam for MIG-08/MIG-09.
- Preserve existing safety behavior from MIG-06.

Acceptance criteria:
- Unit tests verify skew/duration fields are populated.
- Unit tests verify left/right result preservation.
- ServoLoop no longer directly contains left send followed by right send logic except inside dispatcher.
- Existing hardware-free servo tests pass.

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
3. Dispatcher API
4. Tests run
5. Test results
6. Remaining TODOs for ArmWorker migration

```

---

# MIG-08: ArmWorker scaffold

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-08: add ArmWorker scaffold with tests, not integrated into ServoLoop yet.

Dependencies:
- MIG-02 must be complete.
- MIG-07 should be complete.

Allowed scope:
- rb_servo_server/include/rb_servo/control/arm_worker.hpp
- rb_servo_server/src/control/arm_worker.cpp
- rb_servo_server/tests/test_arm_worker.cpp
- rb_servo_server/CMakeLists.txt
- docs/architecture/servo_backend_contract.md

Do not modify:
- DualArmServoLoop runtime behavior
- existing backend concrete behavior except compile compatibility
- simulator protocol
- GUI/policy_runner

Goal:
Introduce the long-term IO ownership abstraction while keeping the active ServoLoop unchanged.

ArmWorker responsibilities:
- Own exactly one IRobotBackend instance.
- Run a dedicated thread.
- Maintain latest timestamped read result.
- Accept latest servo_j request with seq/deadline.
- Produce latest SendServoJResult by seq.
- Expose:
  start()
  stop()
  latestState(max_age_ns)
  enqueueServoJ(SendServoJRequest)
  lastSendResult()
  armId()
  name()
- Never block ServoLoop callers indefinitely.
- Use mutex/condition_variable or a clearly bounded queue.
- Prefer "latest command wins" for servo commands unless a bounded queue is explicitly easier and tested.
- Drop expired commands based on deadline_ns and return CommandTimeout/SuppressedByPolicy as appropriate.
- No real robot motion should become easier to enable.

Thread behavior:
- connect/initialize happens in worker start or explicit start lifecycle, based on current architecture.
- readState loop should be bounded by configured read timeout.
- sendServoJ should occur only when a queued request is present and not expired.
- If backend operation blocks internally, it blocks only the worker thread, not ServoLoop.

Tests:
Use MockBackend or a test backend to simulate:
- successful read loop
- send request accepted
- expired command dropped
- backend send failure preserved
- stop joins the thread
- no deadlock on destruction

Acceptance criteria:
- ArmWorker unit tests pass.
- No active ServoLoop behavior changes.
- ArmWorker does not require real hardware.
- Thread shutdown is deterministic.

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
3. ArmWorker API
4. Thread/shutdown behavior
5. Tests run
6. Test results
7. Remaining TODOs for MIG-09

```

---

# MIG-09: ArmWorker IO model integration behind config

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-09: integrate ArmWorker IO model behind a config flag.

Dependencies:
- MIG-08 must be complete.
- MIG-07 should be complete.

Allowed scope:
- rb_servo_server/include/rb_servo/config/config.hpp
- rb_servo_server/src/config/config.cpp
- rb_servo_server/src/control/dual_arm_servo_loop.cpp
- rb_servo_server/include/rb_servo/control/dual_arm_servo_loop.hpp
- rb_servo_server/src/control/servo_dispatcher.cpp
- rb_servo_server/include/rb_servo/control/servo_dispatcher.hpp
- rb_servo_server/config/*.yaml, only to add examples
- rb_servo_server/tests/test_config_loader.cpp
- rb_servo_server/tests/test_arm_worker.cpp
- rb_servo_server/tests/test_safety_policy.cpp

Do not modify:
- RbpodoBackend APIs beyond compile compatibility
- simulator protocol
- GUI/policy_runner

Goals:
Add an IO model switch:
servo:
  io_model: direct | worker

Default:
- direct

Worker mode:
- ServoLoop reads latest state snapshots from ArmWorker instead of directly blocking on backend readState.
- ServoLoop dispatches send requests to ArmWorker instead of directly blocking on backend sendServoJ.
- ServoLoop uses latest send result by seq/deadline.
- If worker state is stale, classify as RobotStateError or Backend/State stale according to safety policy.
- If worker send result for the seq is not available by deadline, classify as TransportTimeout/CommandTimeout, not silent success.

Important:
- Do not make worker mode the default for real robot yet.
- Do not remove direct mode.
- Direct mode must keep passing existing tests.
- Worker mode should be testable with MockBackend and simulator.

Config validation:
- io_model default direct.
- accepted values: direct, worker.
- real + worker allowed only if docs/safety gates make clear it is experimental, or disallow it initially. Prefer disallow real+worker until tested.

Acceptance criteria:
- Config parser accepts servo.io_model.
- direct mode behavior remains green.
- worker mode unit/integration test passes with mock backend.
- Worker mode does not block ServoLoop on backend read/send.
- State stale handling is explicit.

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
3. io_model behavior
4. Direct vs worker mode test coverage
5. Tests run
6. Test results
7. Remaining TODOs for MIG-10

```

---

# MIG-10: Simulator worker-mode smoke + latency metrics

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-10: simulator worker-mode smoke and latency metrics.

Dependencies:
- MIG-09 must be complete.

Allowed scope:
- rb_servo_server/config/dual_simulator_worker.yaml
- rb_servo_server/tests/test_rbsim_hardware_free_gate.py
- scripts/hardware_free_validation.sh
- scripts/codex_gate.sh
- docs/hardware_free_validation.md
- docs/architecture/servo_backend_contract.md
- rb_servo_server/src/network/state_publisher.cpp, only if metrics serialization is needed
- rb_servo_server/include/rb_servo/core/types.hpp, only metric fields if needed

Do not modify:
- ArmWorker core behavior except bug fixes discovered by smoke tests
- RbpodoBackend
- GUI/policy_runner

Goals:
Prove worker IO mode against the per-arm simulator stack without real hardware.

Add:
- rb_servo_server/config/dual_simulator_worker.yaml:
  backend_type: simulator
  run_mode: simulation
  servo.io_model: worker
  send_servo_commands: true
- A test mode in rbsim_hardware_free_gate that runs direct and worker configs, or a new dedicated test.
- State/log metrics:
  - left/right state age
  - left/right send result age
  - command seq
  - send deadline hit/miss
  - dispatch skew
  - worker loop read duration if available

Acceptance criteria:
- Worker-mode simulator smoke starts left and right per-arm simulators.
- rb_servo_server in worker mode receives state and sends joint commands.
- Wrong-arm, injected transport send failure, and simulator robot fault remain distinguishable.
- Fault-latched regular servo_j remains suppressed.
- Direct-mode smoke still passes.
- hardware_free_validation.sh documents and optionally runs worker mode.

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
./scripts/hardware_free_validation.sh

Final response format:
1. Summary
2. Files changed
3. Worker-mode smoke behavior
4. Metrics added
5. Tests run
6. Test results
7. Remaining TODOs for MIG-11

```

---

# MIG-11: Parallel/worker dispatch semantics + deadlines

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-11: parallel/worker dispatch semantics and deadline correctness.

Dependencies:
- MIG-10 must be complete.

Allowed scope:
- rb_servo_server/src/control/arm_worker.cpp
- rb_servo_server/include/rb_servo/control/arm_worker.hpp
- rb_servo_server/src/control/servo_dispatcher.cpp
- rb_servo_server/include/rb_servo/control/servo_dispatcher.hpp
- rb_servo_server/src/control/dual_arm_servo_loop.cpp
- rb_servo_server/tests/test_arm_worker.cpp
- rb_servo_server/tests/test_servo_dispatcher.cpp
- rb_servo_server/tests/test_safety_policy.cpp
- docs/architecture/servo_backend_contract.md

Do not modify:
- RbpodoBackend real motion behavior
- simulator protocol unless a test fixture requires it
- GUI/policy_runner

Goals:
Make dual-arm dispatch semantics explicit for independent controller endpoints.

Required behavior:
- In worker mode, left and right send requests for the same command seq are enqueued in the same ServoLoop tick.
- Neither arm's backend timeout may block enqueueing the other arm.
- If one worker times out or rejects, the other result is still reported.
- Policy for single-arm failure remains fail-closed:
  - stop/suppress further regular sends according to safety policy.
  - do not hide the successful arm result.
- Deadline is based on command host time and servo timeout, not arbitrary sleep.
- The result must expose:
  - both arms' accepted/rejected status
  - both error kinds
  - send start/end/duration
  - deadline miss flag
  - send skew metrics

Tests:
- test one arm slow, one arm fast.
- test one arm transport failure, one arm accepted.
- test both accepted.
- test deadline miss.
- test no deadlock on shutdown after pending command.

Acceptance criteria:
- Worker dispatch is not serially blocked by one arm.
- DualSendResult always contains left and right result records.
- Safety classifier handles mixed results deterministically.
- Existing hardware-free gates pass.

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
3. Dispatch/deadline semantics
4. Tests run
5. Test results
6. Remaining TODOs for MIG-12

```

---

# MIG-12: Migration rebaseline, acceptance docs, cleanup

```text
Read AGENTS.md and TODO.md first.

This task is part of the backend-contract and non-blocking servo-loop migration. The goal is not to patch tests superficially. The goal is to make rb_servo_server safer, more diagnosable, and more recognizable to engineers who have built robot arm control systems.

Hard safety rules:
- Never enable real robot motion implicitly.
- Real robot connection still requires RB_ALLOW_REAL_ROBOT=1.
- Real servo_j motion still requires RB_ALLOW_REAL_MOTION=1.
- Real Cartesian/TCP motion still requires RB_ALLOW_REAL_CARTESIAN=1.
- Do not activate force/admittance/impedance control.
- Do not fake rbpodo APIs. If headers/docs are not available, keep the build gated and report the limitation.
- Prefer explicit structured results over bool + log-string behavior.
- Keep the repo buildable at the end of this task.

Implement ONLY MIG-12: migration rebaseline, acceptance docs, and cleanup.

Dependencies:
- MIG-01 through MIG-11 should be complete.

Allowed scope:
- README.md
- AGENTS.md
- docs/**
- scripts/codex_gate.sh
- scripts/hardware_free_validation.sh
- scripts/tcp_pose_simulator_acceptance.sh
- rb_servo_server/config/*.yaml
- rb_simulator/config/*.yaml
- rb_gui tests/parser only if state schema compatibility requires
- policy_runner tests/parser only if state schema compatibility requires

Do not modify:
- Backend core code unless fixing small issues found by gates
- Rbpodo real motion behavior
- Force control

Goals:
Make the migration easy to review and hard to misuse.

Tasks:
1. Update docs to show the new architecture:
   CommandBuffer -> ServoCoordinator -> Left/Right ArmWorker.
2. Document backend result taxonomy and mapping:
   RobotFault vs TransportWriteFailed vs SuppressedByPolicy vs WrongMode.
3. Document real robot policy:
   - real read-only connect is allowed only with RB_ALLOW_REAL_ROBOT=1
   - real servo_j motion requires RB_ALLOW_REAL_MOTION=1
   - unverified stop/resetFault means operator intervention on real fault.
4. Ensure no dangerous tracked real config remains.
5. Ensure deprecated simulator configs are marked deprecated or removed if unsafe.
6. Add Pinocchio ON gate if dependency is available, but do not make hardware-free gate require Pinocchio.
7. Make tcp_pose_simulator_acceptance.sh fail early with useful messages if FK/IK config or Pinocchio is unavailable.
8. Ensure state_pub_rate_hz is actually wired or document it if already done.
9. Summarize direct vs worker io_model:
   - direct is current stable default unless worker smoke has been promoted.
   - worker is simulator-accepted if MIG-10/11 passed.
   - real+worker remains disabled or experimental until real read-only acceptance.

Acceptance criteria:
- docs explain the migration outcome clearly.
- codex_gate.sh has robust cases for MIG-* tasks.
- hardware_free_validation.sh does not claim real robot readiness.
- tcp pose acceptance does not silently pass without actually checking required readiness.
- All hardware-free Python/C++ tests pass where dependencies are available.

Required tests:
bash -n scripts/codex_gate.sh
bash -n scripts/hardware_free_validation.sh
bash -n scripts/tcp_pose_simulator_acceptance.sh
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
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
3. Migration outcome
4. Gates run
5. Test results
6. Remaining real-hardware acceptance tasks

```

---
