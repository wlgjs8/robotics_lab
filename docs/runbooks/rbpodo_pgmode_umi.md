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

UMI live teleop uses relative-from-init clutching. SteamVR world alignment and
measured hand-eye calibration are not used for that relative teleop path. The
current stack expects the pika publisher to send the official gripper-tip pose,
so receiver-side `gripper_offset` is zero and `r_align` carries the fixed
tip-frame-to-RB-TCP alignment from the stack YAML.
