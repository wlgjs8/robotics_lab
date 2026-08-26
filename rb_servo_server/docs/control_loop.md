# Control Loop

The normative behavior is in
[../../docs/servo_backend_contract.md](../../docs/servo_backend_contract.md).
The supported servo period is 2 ms (500 Hz).

## Ownership and tick flow

The supervisory `DualArmServoLoop` owns command/lifecycle state, target
generation, Cartesian followers, force overlay, safety/fault aggregation, and
logging/state snapshots. In direct I/O it performs backend reads/sends itself.
In worker I/O each `ArmWorker` owns blocking backend I/O and, with queue sync,
that arm's send cadence.

The control path is conceptually:

```text
fresh leased command or lifecycle event
  -> Hold / JointTarget / Cartesian follower
  -> force-control gate and deviation when covered
  -> Pinocchio IK when Cartesian
  -> safety plan gate and joint/workspace/collision projection
  -> final joint position/velocity/acceleration limits
  -> direct or worker backend send
  -> fault aggregation, telemetry, CSV sample
```

Invalid, stale, expired, or refused commands never synthesize a zero target.
They hold or deceleration-ramp from the last safe/sent target according to the
specific fault contract. A stale streaming Servo J request is a hold condition,
not a backend fault; lifecycle expiry and real backend failures still latch.

## Real worker cadence

The tracked real profile uses two RT workers on cpu1/cpu2 and the supervisory
loop on cpu3. Queue sync regulates each firmware-v8.7.3 FIFO independently.
Motion and follower-plan advance remain held through qsync warmup/drain and
release at track.

The optional worker setpoint interpolator converts the host-loop/box-clock rate
mismatch into uniform time dilation. It is implemented and unit-tested but is
disabled in the tracked real config pending a supervised hardware A/B.

The authoritative wire timestamps/counters are the `worker_wire_*` CSV fields;
legacy `left/right_send_start_ns` are loop enqueue timestamps in worker mode.
`left/right_state_host_time_ns` must be used to identify repeated readback
samples before interpreting a 0-then-2x `q_ref` step as physical motion.

## Force and joint limits

Force-control coverage requires a live, valid, tared F/T pipeline. Manual and
automatic tare share the 250-tick `raw - gravity` average. Gate/spring and the
TCP wrench-reference/compose-pivot pairing are loader-enforced invariants.

J3 is bounded to `[-150 deg, +150 deg]` at the supported safety, barrier, and IK
layers. A Cartesian request that needs more elbow range is unreachable and must
hold/refuse rather than widen the model.
