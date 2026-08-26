# Servo Backend Contract Architecture Notes

The canonical backend contract is now:

```text
docs/servo_backend_contract.md
```

This file is retained only so older links under `docs/architecture/` keep working.

Current backend architecture summary:

- backends return structured `BackendResult<RobotState>` and `SendServoJResult`
- `FaultClassifier` maps backend truth to safety verdicts
- `ArmWorker` owns per-arm blocking I/O in worker mode
- worker mode is a supported real path and owns per-arm cadence for queue sync
- force-control v2 and its F/T/tare telemetry are live server contracts
- the supported J3 range is exactly `[-150 deg, +150 deg]`
- real rbpodo state acquisition is separate from motion readiness

For details, read `../servo_backend_contract.md`.
