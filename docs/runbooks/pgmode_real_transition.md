# rbpodo pgmode Simulation To Physical Real Transition

This runbook defines the evidence ladder from rbpodo controller `pgmode`
simulation to physical `operation_mode: real`. It is not permission to move
hardware. Every physical stage requires a clear workspace, an operator at the
cell, and an immediately available hardware E-stop.

Physical pass evidence uses `tcp_actual_stand`. `tcp_ref_stand` is controller
reference evidence only and must not be reported as physical tracking.

## Ladder and current state

| Stage | Purpose | Current status |
| --- | --- | --- |
| P0 | Reviewed controller-`pgmode` repeatability | regression baseline |
| P1 | Physical read-only diagnostics parity | exercised |
| P2 | Stop/reset/operator-stop procedure | operator procedure required |
| P3 | Physical hold/no-motion | exercised during bring-up |
| P4 | Tiny supervised joint motion | passed in the realized bring-up lane |
| P5 | Tiny physical Cartesian delta | passed |
| P6 | Slow 5 cm / 10 s physical circle | physical circle evidence exists |
| P7 | Stable 15 cm / 16 s circle | not production-ready |
| P8 | 15 cm / 8 s circle | pending P7 |
| P9 | 15 cm / 4 s fast circle | explicit future approval only |

The recorded TUNED-1 circle predates the current LPF-off, firmware-v8.7.3
queue-sync, and later P0/P1/P3 safety changes. It proves that the physical lane
was opened; it is not acceptance of the current high-speed profile.

## Config policy

Only these launch configs exist:

```text
rb_servo_server/config/stack_sim.yaml
rb_servo_server/config/stack_real.yaml
```

Do not create a stage-specific or `config/local` copy. For an acceptance stage:

1. Stop the stack.
2. Change exactly one reviewed setting in the relevant tracked config.
3. Record `git diff -- <config>` with the artifact.
4. Run `rb_servo_server --check-config` against that file.
5. Have the operator review the complete effective diff and physical stage.
6. Run only the approved stage under supervision.
7. Restore the reviewed tracked value explicitly and verify the final diff.

Never use a simulator test or a hidden local YAML as authority for real motion.

## Fixed safety/profile contracts

- `backend_type: rbpodo`; physical operation uses `run_mode: real` and
  `operation_mode: real`.
- J3 is `[-150 deg, +150 deg]` in safety, the joint-limit barrier, and IK.
- The Servo J executor is 500 Hz with `t1=0.002`, `t2=0.021`, `gain=1.0`, and
  script-level `alpha=10.0` (controller LPF off).
- Physical real uses worker I/O, per-arm RT scheduling, and queue sync at fill
  5. `hold_motion_until_track` prevents warmup/drain from overlapping motion.
- Worker setpoint interpolation is not part of the accepted real profile while
  the tracked value is `false`; enabling it requires its own hardware A/B.
- Async URDF-mesh `CollisionMonitor`, ROI, reach, tracking-error latch,
  lease/deadman, and E-stop remain active. The controller `-2001` carve-out does
  not replace server collision monitoring.
- Force-control v2 is live in the tracked real config. Do not remove or retune
  its measured sensor/tool calibration, gate/spring, TCP pivot/reference, or
  fences as part of a circle/latency acceptance stage.
- An untared arm is not force-control-covered. Automatic tare after InitMotion
  does not remove the operator's empty/unloaded check.

## Stage gates

### Read-only

Temporarily set `servo.send_servo_commands: false` in `stack_real.yaml`. Do not
change the tracked Cartesian or ACK policies for this stage: the server-wide
send gate is closed, so no Servo J send may occur and neither policy is being
accepted by the read-only result.
Verify valid raw joints, J3 inside ±150°, state age, controller mode,
diagnostics interpretation, and zero Servo J sends.

### Joint motion

Requires successful read-only evidence, verified stop procedure, valid state,
no latch, `servo.send_servo_commands: true`, and an operator-approved bounded
delta. The target must remain within every joint limit; J3 is never granted an
acceptance margin beyond ±150°.

### Cartesian motion

Additionally requires kinematics, current actual TCP, the real Cartesian config
gate, valid mesh collision verdict, workspace constraints, and a bounded target.
Every report must record desired versus `tcp_actual_stand`; reference tracking
may be recorded separately as informational.

### Fast circle

P7–P9 require reviewed artifacts from every earlier stage. P9 is never an
automatic promotion target and requires separate explicit approval.

## Required artifact fields

- config path and exact diff;
- source commit and server build identity;
- controller firmware/mode and arm endpoint;
- J3/safety range used;
- state age, loop/worker/wire timing, ACK semantics, qsync phase/fill;
- `q_sent`, `q_ref`, and `q_actual` validity;
- `tcp_actual_stand` tracking metrics for physical Cartesian stages;
- fault, tracking, workspace, collision, force coverage/tare, and stop results;
- whether physical motion was expected and observed; and
- explicit operator/supervision/E-stop confirmation.

Any missing safety-relevant value is a blocked stage, never a guessed default.
