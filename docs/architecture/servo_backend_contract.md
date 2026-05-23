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
