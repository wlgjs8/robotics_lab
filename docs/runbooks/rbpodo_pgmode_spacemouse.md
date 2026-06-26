# SpaceMouse Teleop In The Current Stack

This note is retained as a historical entry point for SpaceMouse pgmode work.
The active launch surface is now the unified stack:

```bash
make run MODE=sim
```

That command starts `rb_servo_server`, `rb_gui`, camera services, and
`policy_runner` together. The policy runner uses:

```text
policy_runner/config/stack_sim.yaml
```

`stack_sim.yaml` runs `action_source: teleop_mux`, so SpaceMouse and UMI are
available side by side under one lease. To isolate SpaceMouse while debugging,
pass `POLICY_ACTION_SOURCE=dual_spacemouse_pose_target` through the stack
launcher environment.

Controller-simulation safety remains config-driven. The server-side local config
must keep the controller in `operation_mode: simulation`, expose
`physical_motion_expected=false`, and keep physical real Cartesian disabled with
`cartesian_control.allow_in_real: false`. This runbook is not physical real
motion acceptance.

Expected ports in the stack profile:

```text
server command bind: udp://127.0.0.1:50256
rb_gui / viser:      udp://127.0.0.1:50366
policy_runner:       udp://0.0.0.0:50376
```

The viewer should use the controller reference TCP source in controller
simulation (`tcp_ref_stand` when the state stream recommends it). If the state
line does not report `backend=rbpodo`, `operation_mode=simulation`, and
`physical_motion_expected=false`, stop and inspect the local stack config before
sending teleop commands.
