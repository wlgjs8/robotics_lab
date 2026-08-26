# Dual RB3-730E Physical Cartesian Circle

This page preserves the accepted slow dual-arm physical-circle evidence and
states the requirements for a current rerun. It is not a standalone motion
authorization or a copy-and-paste launch recipe.

> **Physical motion:** a rerun requires an operator holding the E-stop, a clear
> workspace, both controllers deliberately placed in `pgmode real`, and the
> tracked real stack reviewed immediately before launch. Simulator acceptance
> is never permission to move hardware.

## Status

The slow circle has run successfully on both physical RB3-730E arms under
operator supervision. That result established the real Cartesian bring-up
milestone; it does not transfer automatically to a changed firmware, Servo J
profile, collision model, force profile, or command source.

The accepted data path was:

```text
UMI-style Cartesian source
  -> policy_runner (`TcpPoseTarget`, ee-local delta composed to absolute pose)
  -> rb_servo_server
  -> rbpodo Servo J at 500 Hz
  -> physical left/right RB3-730E
```

The point-in-time tuning result was approximately 1.42 deg median joint
tracking error with a slow 5 cm-radius, 25 s/revolution target. That run used an
older TUNED-1 profile (`servo_alpha: 0.8`, direct synchronous sending) before
the current firmware-v8.7.3 worker/queue synchronization profile. Treat the
number only as historical evidence; do not copy the old parameters into the
current stack.

## Current Source of Truth

Use only:

- `rb_servo_server/config/stack_real.yaml`
- `policy_runner/config/stack_real.yaml`
- [pgmode-real transition ladder](pgmode_real_transition.md)
- [Servo/backend contract](../servo_backend_contract.md)
- [Joint-range policy](../joint_range_policy.md)

Do not create `config/local` variants. A current physical-circle rerun is a new
acceptance run against the reviewed tracked files, with its own artifacts.

## Current Safety Boundary

`rb_servo_server` owns the final allow/deny decision. The policy-side blanket
real-Cartesian block was retired; readiness checks still reject stale or
faulted state, and the server continues to enforce command-source lease,
deadman/freshness where applicable, finite/range checks, velocity and
acceleration filtering, tracking-error fault latching, stand-floor safety, and
the asynchronous URDF-mesh `CollisionMonitor`.

The controller `-2001` suspect fields remain visible and are accepted only by
the explicit tracked real-config policy. They are not silently reclassified as
healthy. EMS, SOS, soft E-stop, collision, unknown controller mode, invalid
state, and initialization errors remain stop conditions.

J3 is exactly `[-150, +150]` degrees for both arms. The value matches the
Rainbow/URDF contract and applies to all future safety, planning, and
acceptance use; never widen it to recover a trajectory.

The tracked real stack also includes the calibrated force-control v2 sections.
Their calibration is owned by controller-manager. A Cartesian-circle run is
not force acceptance: do not retune the sensor basis, tool mass/COM, TCP pivot,
gate/spring pair, or tare policy, and do not enter a force-motion mode as part
of this runbook.

## Current Servo Profile

Review the effective tracked values rather than transcribing them here. At the
time of this update the important boundaries are:

- 500 Hz with `servo_t1_sec: 0.002`
- per-arm worker I/O and real-time scheduling
- firmware-v8.7.3 `queue_sync` enabled with motion held until tracking
- controller LPF off under the tracked `servo_alpha` profile
- worker-side setpoint interpolation disabled pending a separate on-robot A/B

The queue synchronization and mailbox/interpolation changes have unit-test and
offline evidence, but the pending interpolation A/B must not be described as
physical acceptance.

## Rerun Sequence

1. Review the clean diff and both tracked stack configs. Validate the real
   config with the rbpodo-enabled server binary's `--check-config` mode.
2. Complete the read-only and Hold stages in
   [pgmode-real transition](pgmode_real_transition.md). Confirm both controller
   identities, real/simulation mode, finite joint state, J3 range, force/tare
   telemetry, collision-monitor health, and no latched fault.
3. Confirm the intended command source exclusively owns the lease. Verify the
   policy configuration and any camera/gripper gates independently.
4. Run a no-op or tiny supervised Cartesian target before the circle. Stop on
   any unexpected motion, tracking growth, queue-sync loss, stale state,
   collision-monitor fault, or force-control coverage change.
5. Run the slow circle only after the smaller stages pass in the same session.
   Preserve the exact config diff, logs, state stream, controller/firmware
   identity, and operator notes.
6. Stop the command source first, then the policy and server according to the
   supervised stack procedure. Verify the arms hold and no new fault is hidden.

## Evidence to Record

- commit and full tracked-config diff
- controller IP, identity, firmware, and `pgmode`
- `q_sent`, `q_ref`, `q_actual`, tracking-error maxima, and fault timeline
- RBACK fill and per-arm queue-sync phase/lock evidence
- worker mailbox/interpolation counters, including duplicated-readback evidence
- Cartesian target/actual traces and collision-monitor status
- force-control coverage, tare stage/validity, and confirmation that no
  force-motion mode was entered
- stop reason and operator/E-stop notes

The detailed offline queue analysis is in
[box latency](box_latency_offline.md). A result is current acceptance only when
the artifact records the current tracked profile; the historical TUNED-1 result
above remains audit context.
