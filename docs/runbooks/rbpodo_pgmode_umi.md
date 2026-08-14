# UMI Cartesian Teleop In The Current Stack

This note is retained as a historical entry point for UMI pgmode work. The active
launch surface is now the unified stack:

```bash
make run MODE=sim
```

That command starts `rb_servo_server`, `rb_gui`, camera services, and
`policy_runner` together. The policy runner uses:

```text
policy_runner/config/stack_sim.yaml
```

`stack_sim.yaml` runs `action_source: teleop_mux`, so UMI and SpaceMouse are
available side by side under one lease. To isolate UMI while debugging, pass
`POLICY_ACTION_SOURCE=umi_dual_cartesian` through the stack launcher environment.

The UMI readers bind the current stack ports:

```yaml
umi_dual_cartesian:
  left:
    udp_endpoint: "udp://0.0.0.0:50380"
  right:
    udp_endpoint: "udp://0.0.0.0:50381"
```

Each side binds its own port; the publisher sends the same combined packet to
both ports and each reader extracts its side. The JSON packet carries `t`,
`left`, and `right` blocks with `pose`, `gripper`, and `deadman` fields.

Controller-simulation safety remains config-driven. The server-side local config
must keep the controller in `operation_mode: simulation`, expose
`physical_motion_expected=false`, and keep physical real Cartesian disabled with
`cartesian_control.allow_in_real: false`. This runbook is not physical real
motion acceptance.

## Near a singularity the arm goes heavy on purpose

Driving an arm toward full extension (elbow J3 through 0) or a wrist alignment
(J5 through 0) collapses the IK's smallest singular value, and the joint speed a
small Cartesian motion needs there goes to infinity. The `umi_large_smooth`
profile's `singularity_scale_*` guard scales the SMD tracking velocity down as
that sigma drops — full speed at `sigma >= 0.12`, down to 2% of the cap at
`sigma <= 0.05`. **The arm feeling sluggish and then nearly stopping while your
hand keeps moving is the guard working; back the hand out and it recovers on its
own.** The guard never reaches zero, so it cannot deadlock.

If you drive in anyway, the IK runs out of iterations (`max_iterations`), that
arm goes `ArmedHold` and freezes at its last commanded joints until the target
comes back out. `fault_latched` stays 0 — this is a per-arm hold, not a fault,
and the other arm keeps tracking.

What this looked like before the guard was fixed (2026-08-14,
`logs/servo_log_20260814_100223.csv`): the left arm hit both singularity types in
one 90 s session, the commanded pose swung 18-30 deg out and back at 6-18 Hz,
and `q_actual` peaked at 415 deg/s. Telemetry to check after any such event:
`<side>_cart_min_singular`, `_cart_ik_iters` (100 == the cap), `_cart_reason`
(`max_iterations` / `branch_jump_rate_limited`), and `_cart_branch_jump_raw_deg`
(how far the IK's raw solution jumped before limiting).

UMI live teleop uses relative-from-init clutching. SteamVR world alignment and
measured hand-eye calibration are not used for that relative teleop path. The
current stack expects the pika publisher to send the official gripper-tip pose,
so receiver-side `gripper_offset` is zero and `r_align` carries the fixed
tip-frame-to-RB-TCP alignment from the stack YAML.
