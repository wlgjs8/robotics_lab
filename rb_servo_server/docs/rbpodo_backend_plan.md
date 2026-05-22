# RbpodoBackend Implementation Plan and Hardware Acceptance Runbook

This document defines the implementation plan and acceptance gates for the real
Rainbow RB3-730 backend. It is a planning artifact only. Do not use it as
evidence that real-robot operation is ready.

Current state:

- `RbpodoBackend` is intentionally incomplete.
- With `RB_SERVO_ENABLE_RBPODO=OFF`, it fails immediately.
- With `RB_SERVO_ENABLE_RBPODO=ON`, it still refuses to connect because state
  read and servo send paths are not implemented.
- `config/dual_real.yaml` is a real-mode target config, but real mode must not
  be launched until the implementation and acceptance steps below are complete.

## Scope

Target robot path:

```text
rb_servo_server
  DualArmServoLoop
    left RbpodoBackend  -> left RB3-730 controller
    right RbpodoBackend -> right RB3-730 controller
```

The first real backend milestone is joint-servo only:

- connect
- initialize
- readState
- sendServoJ
- stop
- resetFault
- real-mode safety guards
- first-motion acceptance

Out of scope for the first milestone:

- Cartesian FK/IK motion
- force/admittance motion
- gripper integration
- camera/policy closed-loop autonomy
- exposed production network operation
- unsupervised runs

## Non-Negotiable Gates

Do not run real mode unless all are true:

- Named human operator is physically present.
- Physical E-stop is identified, reachable, and tested out of band.
- Workspace is clear and robot stands are mechanically secured.
- Correct left/right robot IPs are confirmed.
- `RB_ALLOW_REAL_ROBOT=1` is set for the process.
- `servo.enable_realtime_priority=true` is enabled and realtime setup succeeds.
- `safety.tracking_error_policy=fault_latch`.
- `safety.stop_both_arms_on_single_arm_error=true`.
- `safety.latch_fault_on_robot_state_error=true`.
- `network.command_bind` and `network.state_pub_bind` remain loopback unless a
  separate deployment review explicitly sets `RB_ALLOW_NETWORK_EXPOSURE=1`.
- Hardware-free gate and rb_simulator gate pass immediately before real work.

## Implementation Plan

### Phase 0: SDK Discovery and Build Contract

Goal: make the SDK dependency explicit without weakening the default
hardware-free build.

Tasks:

- Locate the locally installed rbpodo SDK headers, libraries, and examples.
- Verify the exact C++ API names for controller construction, connection,
  operation mode, state read, servo joint command, stop, and fault reset.
- Update CMake only behind `RB_SERVO_ENABLE_RBPODO=ON`.
- Keep default builds independent from rbpodo.
- Add a CMake failure that explains the required `CMAKE_PREFIX_PATH`,
  `RBPODO_ROOT`, or package path when `RB_SERVO_ENABLE_RBPODO=ON` but the SDK is
  missing.
- Link only the target that needs rbpodo symbols; do not make mock/rbsim tests
  require the SDK.

Acceptance:

- `-DRB_SERVO_ENABLE_RBPODO=OFF` still builds and runs hardware-free tests.
- `-DRB_SERVO_ENABLE_RBPODO=ON` fails clearly when the SDK is absent.
- With the SDK present, the project links without starting real robot code.

### Phase 1: Connection Object and Ownership

Goal: create a per-arm SDK object that can connect and disconnect safely.

Tasks:

- Store the rbpodo controller/client object in `RbpodoBackend::Impl`.
- Validate `BackendConfig::ip` is non-empty for `backend_type=rbpodo`.
- Keep `RB_ALLOW_REAL_ROBOT=1` enforcement in `connect()`.
- Add connection timeout/error handling if the SDK supports it.
- Set `impl_->connected=true` only after the SDK confirms connection.
- Ensure destructor/`stop()` does not throw.
- Never open a socket or construct a hardware client before the real-mode guard
  has passed.

Acceptance:

- Without `RB_ALLOW_REAL_ROBOT=1`, real config fails before hardware contact.
- With an unreachable IP in a hardware-approved dry run, `connect()` returns
  false or throws a controlled startup failure; the process does not continue
  into command serving.

### Phase 2: Initialize

Goal: put each arm into the intended controller mode without starting motion.

Tasks:

- Map config fields to SDK calls:
  - `operation_mode`
  - `speed_bar`
  - `servo_time_sec`
  - `servo_lookahead_sec`
  - `servo_gain`
  - `servo_acc`
  - `disable_waiting_ack`
- Confirm whether SDK units are seconds/degrees/radians and document the
  conversion.
- Enable servo mode only when required for subsequent `sendServoJ`.
- Do not send a motion target during `initialize()`.
- Return false if controller state, servo mode, or safety state is not accepted.

Acceptance:

- After `initialize()`, `readState()` can return valid joint data.
- `DualArmServoLoop` still starts in `ConnectedHold`.
- No motion target is sent before an explicit `ArmMotion` and `JointTarget`.

### Phase 3: readState

Goal: make `RobotState` truthful enough for startup, safety filtering, logging,
and state publication.

Tasks:

- Read actual joint positions from the controller.
- Populate `RobotState::q_actual_deg` in degrees.
- Populate `RobotState::q_target_deg` if the SDK exposes target or command
  state; otherwise use the last accepted backend target only as a clearly
  documented fallback.
- Populate `dq_actual_deg_s` when available; otherwise use finite zeros and
  document that velocity is not controller-derived yet.
- Set `host_time_ns=nowSteadyNs()` after receiving the state sample.
- Populate `robot_time_ns` if the controller exposes a monotonic controller
  timestamp; otherwise leave it zero and document the limitation.
- Set `connection_state=Connected` only for a confirmed live controller.
- Set `has_valid_joint_state=true` only after finite six-joint actual data has
  been received from the trusted controller path.
- Set `servo_enabled`, `has_error`, and `error_code` from controller state when
  available.
- Return false on timeout, malformed data, SDK exception, disconnected state, or
  controller fault that prevents a truthful state read.

Acceptance:

- Startup fails if either arm cannot provide a finite, in-limit, connected,
  error-free joint state.
- A controller fault is visible as `has_error=true` or failed read, causing the
  servo loop to latch according to real-mode policy.
- No failure path reports default zero joints as valid state.

### Phase 4: sendServoJ

Goal: send one bounded joint servo target per loop tick only after the servo
loop has filtered it.

Tasks:

- Confirm SDK joint unit expectations. Convert degrees to the SDK unit at the
  backend boundary if required.
- Use `servo_time_sec`, `servo_lookahead_sec`, `servo_gain`, and `servo_acc`
  consistently with SDK examples.
- Return true only after the SDK accepts the command.
- Return false on timeout, SDK exception, controller refusal, disconnected
  state, or fault.
- Do not mutate backend "last accepted" target on failed send.
- Avoid hidden retries inside the backend; one servo loop tick should produce at
  most one backend send attempt per arm.

Acceptance:

- `testSendFailureDoesNotAdvancePreviousTarget` remains valid for real backend
  semantics.
- Any send failure latches both arms in real mode.
- State/log artifacts show `left_send_ok` or `right_send_ok=false` when a send
  fails.

### Phase 5: stop

Goal: make stop fail-closed and truthful.

Tasks:

- Map `stop()` to the safest SDK stop/hold command available.
- Prefer a controller-level servo stop or hold-current-position command over
  synthesizing a new motion target.
- Return false if the SDK reports stop failure.
- Preserve state truthfulness after stop: the next `readState()` must show
  controller state as connected/error/stopped according to SDK data.

Acceptance:

- On stop success, no new motion target is sent by the backend.
- On stop failure, `DualArmServoLoop` and state/log artifacts do not claim a
  stopped controller unless the controller reported it.

### Phase 6: resetFault

Goal: clear recoverable controller faults only when explicit reset was requested.

Tasks:

- Map `resetFault()` to the SDK's recover/reset sequence.
- Return false if reset is unsupported, rejected, times out, or leaves the
  controller in fault.
- Do not re-enable motion implicitly.
- After reset, require a fresh valid `readState()` before clearing the servo
  loop latch.
- Keep `DualArmServoLoop` in `ConnectedHold`; require a new `ArmMotion`.

Acceptance:

- Reset cannot resume motion by itself.
- If either arm fails reset or fresh state validation, the latch remains active.
- Logs/state stream show `ConnectedHold` only after successful fresh state.

## Safety Guard TODOs

Before first hardware motion:

- Add focused tests for real-mode config refusal:
  - missing `RB_ALLOW_REAL_ROBOT`
  - realtime disabled
  - tracking policy not `fault_latch`
  - stop-both disabled
  - latch-on-robot-state-error disabled
  - exposed command/state bind without `RB_ALLOW_NETWORK_EXPOSURE=1`
- Add an SDK-absent CMake test or documented build check for
  `RB_SERVO_ENABLE_RBPODO=ON`.
- Add a fake or adapter-level rbpodo test seam if the SDK can be abstracted
  without linking hardware code into default tests.
- Confirm `dual_real.yaml` IPs, joint limits, servo rate, and speed parameters
  are site-specific placeholders until operator signoff.
- Keep GUI real-mode motion disabled until a separate operator-console review
  approves it.

## First-Motion Checklist

This checklist must be completed in order. Stop immediately on any failed item.

### 1. Desk Review

- Review this document with the operator.
- Review `config/dual_real.yaml` line by line.
- Confirm left/right robot identity and IP mapping.
- Confirm joint limits are conservative for the physical setup.
- Confirm the first target is within 1 degree of current actual joints.
- Confirm command source remains loopback.

### 2. Build and Software Evidence

Run these without real hardware contact:

```bash
RBSIM_SMOKE_MODE=skip bash scripts/hardware_free_validation.sh
python3 -m unittest discover rb_simulator/tests
python3 rb_simulator/tools/rbsim_servo_smoke.py --self-test
```

If SDK is installed, build the rbpodo-enabled binary without launching it:

```bash
cmake -S rb_servo_server -B rb_servo_server/build/rbpodo \
  -DRB_SERVO_ENABLE_RBPODO=ON
cmake --build rb_servo_server/build/rbpodo -j
```

### 3. Guard Dry Run

With real config but without `RB_ALLOW_REAL_ROBOT=1`, confirm startup refuses:

```bash
./rb_servo_server/build/rbpodo/rb_servo_server \
  --config rb_servo_server/config/dual_real.yaml
```

Expected result: non-zero exit before hardware contact.

### 4. Hardware Preflight

- Operator present.
- Physical E-stop reachable.
- Robot workspace clear.
- Robot controllers powered and in the expected idle state.
- Network route to each controller verified by approved site procedure.
- No policy runner or GUI motion source connected.
- Terminal ready to send `EmergencyStop`.
- Logging directory empty or uniquely named for this run.

### 5. Connect and Hold Only

Run real mode only after preflight:

```bash
RB_ALLOW_REAL_ROBOT=1 ./rb_servo_server/build/rbpodo/rb_servo_server \
  --config rb_servo_server/config/dual_real.yaml
```

Expected:

- both backends connect
- both backends initialize
- first valid state is read
- server enters `ConnectedHold`
- command server starts only after safe hold
- no motion command is sent

Stop condition:

- Any fault, invalid state, unexpected motion, failed realtime setup, or missing
  state field ends the run.

### 6. Arm Without Motion

Send only:

```bash
python3 rb_servo_server/tools/send_arm_motion.py
```

Expected:

- state transitions to `ArmedHold`
- previous sent target remains near current actual joints
- no commanded displacement beyond noise/hold behavior

### 7. First Joint Target

Use a hand-authored target no more than 1 degree from current actual joints on a
single low-risk joint, with both arms explicitly bounded. Keep command timeout
short.

Expected:

- small, smooth movement
- no tracking fault
- no send failure
- state/log `q_sent_deg` matches the bounded target
- operator confirms physical motion direction is correct

Stop condition:

- Any unexpected direction, arm mismatch, jerk, tracking error, controller
  warning, stale state, dropped command, or send failure.

### 8. Stop and Reset Evidence

- Send `EmergencyStop` and verify latch.
- Send `ResetFault` only after operator approval.
- Confirm reset returns to `ConnectedHold`, not `Running`.
- Require `ArmMotion` again before any new motion.

## Acceptance Evidence Package

Store the following artifacts for each hardware run:

- exact git revision or patch bundle
- `dual_real.yaml` copy used for the run
- build command and `RB_SERVO_ENABLE_RBPODO` state
- operator name and date
- startup stdout/stderr
- servo CSV log
- state snapshots around connect, hold, first arm, first target, stop, reset
- first target command payload
- notes on any controller warnings or operator observations

Do not mark real backend acceptance complete unless the evidence package shows:

- guarded startup behavior
- valid state before command serving
- no default-zero state substitution
- first motion bounded and correct direction
- send failure and reset semantics remain fail-closed
- no relaxation of real-mode guards

