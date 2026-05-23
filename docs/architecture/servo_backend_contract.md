# Servo Backend Contract Architecture Notes

This note records MIG-08 implementation boundaries. The root migration source
of truth remains `docs/servo_backend_contract.md`.

## MIG-08 ArmWorker Scaffold

`rb_servo::ArmWorker` introduces the long-term one-arm I/O ownership boundary
without changing active `DualArmServoLoop` behavior.

- One `ArmWorker` owns exactly one `IRobotBackend`.
- `start()` launches a dedicated worker thread and returns without performing
  backend I/O on the caller thread.
- The worker thread runs backend `connect()`, `initialize()`, `sendServoJ()`,
  and `readState()`.
- `latestState(max_age_ns)` returns the latest cached structured read result,
  or a structured stale/no-sample failure. It does not call the backend.
- `enqueueServoJ(request)` is latest-command-wins for commands not yet taken by
  the worker thread. It does not call the backend.
- Expired commands are recorded as `CommandTimeout` send results and are not
  dispatched to the backend.
- Commands submitted while the worker is stopped are recorded as
  `SuppressedByPolicy`.
- `lastSendResult()` returns the latest cached `ArmSendResult`, including the
  original request sequence and structured `SendServoJResult`.
- `stop()` requests shutdown, wakes the worker, and joins the thread.

MIG-08 does not integrate `ArmWorker` into `DualArmServoLoop`, does not change
backend concrete behavior, and does not weaken the existing real-hardware
environment gates.

## MIG-10 Worker Simulator Smoke And Metrics

`rb_servo_server/config/dual_simulator_worker.yaml` is the hardware-free
worker-I/O profile. It keeps the canonical simulator contract:

```yaml
backend_type: simulator
run_mode: simulation
servo:
  io_model: worker
  send_servo_commands: true
```

The profile targets one loopback simulator endpoint per arm and is not real
robot evidence. Real mode still rejects `servo.io_model: worker` until a
separate real-hardware acceptance task exists.

The hardware-free gate runs the direct per-arm simulator smoke and can also run
the worker-mode smoke. Worker smoke must prove that the server receives cached
worker state, dispatches joint `servo_j` requests through each `ArmWorker`, and
continues to distinguish wrong-arm simulator requests, injected transport send
failures, and simulator robot faults.

State snapshots and servo CSV logs expose MIG-10 diagnostics:

- per-arm `state_age_us`
- per-arm `send_result_age_us`
- top-level `command_seq`
- per-arm and aggregate `send_deadline_hit`
- `send_skew_us` / `dispatch_skew_us`
- per-arm `worker_loop_read_duration_us` when `servo.io_model: worker`

## MIG-11 Parallel Dispatch And Deadlines

Worker dispatch treats a dual-arm command as one coordination event for two
independent controller endpoints:

- The left and right `SendServoJRequest` records preserve the same command
  sequence, command host timestamp, and command-derived deadline.
- Both arm requests are enqueued before the dispatcher waits for either worker
  result, so a slow or timed-out backend on one arm cannot prevent enqueueing
  the other arm in the same servo-loop tick.
- `DualSendResult` always contains left and right `ArmSendResult` records.
  Missing worker responses are represented as structured `CommandTimeout`
  send results instead of absent data.
- A mixed result remains visible: if one arm times out or rejects and the other
  accepts, the accepted arm result is still reported while the safety policy
  classifies the aggregate as fail-closed according to the configured
  stop-both-arms behavior.
- Worker waits are bounded by `command.host_time_ns + min(left.timeout_sec,
  right.timeout_sec)`. They no longer depend on an arbitrary polling sleep or
  on a single backend call returning.
- Dispatch diagnostics preserve per-arm send start/end/duration timing and
  start/end skew. Dispatcher deadline helpers report whether either arm
  completed after its command deadline.
