# Servo Backend Contract Architecture Notes

The canonical backend contract is now:

```text
docs/servo_backend_contract.md
```

This file is retained only so older links under `docs/architecture/` keep working.

Current backend architecture summary:

- backends return structured `BackendResult<RobotState>` and `SendServoJResult`
- `FaultClassifier` maps backend truth to safety verdicts
- simulator uses one endpoint/backend per arm
- `RbsimBackend` uses persistent JSON-line transport during healthy operation
- `ArmWorker` owns per-arm blocking I/O in worker mode
- worker mode remains simulator-only until real-hardware acceptance exists
- real rbpodo state acquisition is separate from motion readiness

For details, read `../servo_backend_contract.md`.
