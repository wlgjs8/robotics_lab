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

The stack no longer relies on fixed `/dev/hidrawN` numbers. `policy_runner`
discovers `256f:c652` interface `0` receivers wherever they are plugged in and
publishes their activity to `rb_gui`. Because the Universal Receivers report no
unique serial, open `조작 > SpaceMouse`, move each physical cap to identify it,
then assign left/right and press `좌/우 배정 적용`. A replugged receiver appears
as a new unassigned connection; motion stays blocked for it until reassigned and
held neutral. `좌우 교환` is available only while teleop ownership is idle.

The same panel reports `deadman`, startup-neutral progress, raw axes, activity,
sample age, and the active per-arm input gate. The tracked profile already uses
`deadman=off`. If an assigned device remains at `gate=startup-neutral` while a
centered raw axis exceeds `deadband`, the input is being stopped before a
Cartesian intent is generated; this is not a servo-server rejection. Keep the
server lease, stale-state, and motion safety gates enabled during this check.

Read-only discovery diagnostics:

```bash
PYTHONPATH=policy_runner python3 policy_runner/scripts/probe_spacemouse.py --list
```

Standalone dual-input test (stop `make run` first so no process owns either HID
handle):

```bash
PYTHONPATH=policy_runner python3 policy_runner/scripts/probe_spacemouse.py \
  --duration-sec 60 --rate-hz 100 --log --changes-only \
  | tee logs/spacemouse_dual_probe.log
```

Move receiver A only, then receiver B only, then both together. The final
summary reports each path's independent value-change count, peak axis, and last
released axes. This script opens HID readers only; it does not create a robot
command client or send servo commands.

Controller-simulation safety remains config-driven. The tracked server config
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
`physical_motion_expected=false`, stop and inspect the tracked stack config before
sending teleop commands.
