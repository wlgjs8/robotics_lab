# rb_servo_server Architecture

The normative system architecture is [../../docs/architecture.md](../../docs/architecture.md),
and the detailed control/backend contract is
[../../docs/servo_backend_contract.md](../../docs/servo_backend_contract.md).
This page keeps only the server-local ownership summary.

## Runtime ownership

```text
CommandServer / CommandBuffer
  -> DualArmServoLoop
       -> left ArmWorker  -> left IRobotBackend
       -> right ArmWorker -> right IRobotBackend
```

`DualArmServoLoop` owns command freshness and lease interpretation, lifecycle,
followers, FK/IK, safety planning/filtering, force overlay, fault aggregation,
logging samples, and state snapshots. `ArmWorker` owns blocking per-arm backend
I/O; in the tracked real profile it also owns that arm's queue-synchronized send
cadence on an isolated RT CPU.

The supported high-rate profile is 500 Hz. `stack_real.yaml` uses worker I/O,
per-arm `SCHED_FIFO` scheduling, and firmware-v8.7.3 queue regulation.
`stack_sim.yaml` uses direct I/O and no queue regulation. Optional worker
setpoint interpolation is implemented but remains disabled in the tracked real
profile pending a supervised hardware A/B.

## Motion and force paths

```text
JointTarget / TcpPoseTarget / TcpLinearMove
  -> nominal joint/TCP target and follower plan
  -> server-owned force gate + admittance deviation, when the arm is covered
  -> Pinocchio IK, when Cartesian
  -> mode-independent joint/workspace/collision safety
  -> per-arm backend send
```

Force-control v2 is live. It reads raw controller F/T channels, applies the
controller-manager-derived sensor/tool calibration and tare/gravity
compensation, and publishes per-arm `force_torque`/`force_control` telemetry.
An arm without a valid tare bias is not covered.

J3 is fixed to the Rainbow/URDF range `[-150 deg, +150 deg]` in safety, the
joint-limit barrier, and IK.

## Process boundaries

Camera capture, image transport, policy inference, and the browser UI remain in
their own processes. They consume server state or send public commands; they do
not read backends directly or own the real-motion safety decision.
