# rbpodo pgmode SpaceMouse Teleop

This runbook launches:

```text
SpaceMouse -> policy_runner -> rb_servo_server/rbpodo pgmode simulation -> rb_gui/viser
```

It connects to real Rainbow controller boxes, but the required robot
`operation_mode` is `simulation` and physical motion must remain unexpected.
This workflow is not physical real Cartesian acceptance.

## Profiles

- Server template:
  `rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml`
- Local server config:
  `rb_servo_server/config/local/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.yaml`
- policy_runner config:
  `policy_runner/config/rbpodo_pgmode_spacemouse_500hz_ack.yaml`

Ports:

```text
server command bind: udp://127.0.0.1:50256
state recorder/debug: udp://127.0.0.1:50356
rb_gui / viser:      udp://127.0.0.1:50366
policy_runner:       udp://127.0.0.1:50376
```

## Prepare Local Config

Create the ignored local server config from the tracked example:

```bash
tools/create_rbpodo_pgmode_spacemouse_local_config.sh \
  --left-ip 192.0.2.10 \
  --right-ip 192.0.2.11
tools/rbpodo_pgmode_spacemouse.sh check
```

The addresses above are documentation placeholders. Use site-local controller
addresses for an operator run, either through the flags above or with
`RB_PGMODE_SPACEMOUSE_LEFT_IP` and
`RB_PGMODE_SPACEMOUSE_RIGHT_IP`. To set non-default controller ports, pass
`--command-port`, `--data-port`, or the per-arm port flags.

The existing wrapper action delegates to the same generator:

```bash
tools/rbpodo_pgmode_spacemouse.sh prepare
```

You may also copy
`rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml`
to the local path manually and edit it there. The local copy is ignored by git.

Review the local copy before running. Keep physical real Cartesian disabled:

```bash
grep -H "operation_mode: simulation" rb_servo_server/config/local/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.yaml
grep -H "allow_in_real: false" rb_servo_server/config/local/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.yaml
grep -H "allow_in_controller_simulation: true" rb_servo_server/config/local/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.yaml
grep -H "allow_real_motion: false" policy_runner/config/rbpodo_pgmode_spacemouse_500hz_ack.yaml
```

`policy_runner.safety.allow_real_motion=false` is expected. The policy runner
uses only `allow_rbpodo_controller_simulation_cartesian=true` plus server
telemetry to admit pgmode simulation commands; it must not look like physical
real-motion approval.

For a hardware-free command preview without SpaceMouse HID devices:

```bash
tools/rbpodo_pgmode_spacemouse.sh server-dry-run
tools/rbpodo_pgmode_spacemouse.sh policy-dry-run
```

`policy-dry-run` writes a temporary `/tmp` policy config that uses
`mock_script: pgmode_spacemouse_smoke` for both SpaceMouse readers, then prints
the command it would run. It does not execute `policy_runner`, open HID
devices, or set rbpodo env gates.

## Operator Sequence

1. Start the server in rbpodo `pgmode` simulation:

```bash
tools/rbpodo_pgmode_spacemouse.sh server \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation \
  --with-required-env
```

`--with-required-env` exports the still-live rbpodo wrapper env vars
`RB_ALLOW_RBPODO_ASYNC_STREAMING=1` and
`RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1`. The controller-simulation
carve-out itself is config-driven, not env-gated (the legacy `RB_ALLOW_REAL_*` /
`RB_ALLOW_RBPODO_CONTROLLER_SIM_*` / `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED` env
gates were removed from the server runtime): the site-local config enables it via
`servo.allow_controller_simulation_motion: true` +
`cartesian_control.allow_in_controller_simulation: true`. These are separate from
`policy_runner.safety.allow_real_motion`, which stays `false`.

2. Start the viewer:

```bash
tools/rbpodo_pgmode_spacemouse.sh gui
```

The launcher prints the viewer URL, defaulting to
`http://127.0.0.1:8080`, and listens to server state on
`udp://0.0.0.0:50366`. It is a state-only viewer for this workflow: it does
not send robot commands, does not read SpaceMouse HID devices, and does not
route SpaceMouse packets. SpaceMouse commands route through `policy_runner`
only.

Leave TCP display on `Auto`; for controller simulation it should select
`tcp_ref_stand` when the state stream recommends
`reference_for_controller_simulation`.

In the Status tab, confirm the compact pgmode line contains the expected
controller-simulation boundary:

```text
pgmode_sim: backend=rbpodo run_mode=real operation_mode=simulation physical_motion_expected=false cartesian_available=true policy_runner_lease=active source=policy_runner command=TcpPoseTarget selected_tcp=tcp_ref_stand
```

If any required field is missing, the line reports
`degraded missing=...`. If `physical_motion_expected` is absent or not
`false`, it reports `warning=physical_motion_expected_not_false`.
Use TCP display `both` when you need to inspect physical-state
`tcp_actual_stand` next to controller-simulation reference `tcp_ref_stand`;
tracking should still use the reference when the server recommends it.

3. Start `policy_runner` with the manual SpaceMouse path:

```bash
tools/rbpodo_pgmode_spacemouse.sh teleop-record \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

For HDF5 recording use the same ordering:

```bash
tools/rbpodo_pgmode_spacemouse.sh hdf5-record \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation \
  --task "dual arm SpaceMouse pgmode simulation"
```

4. Verify no physical motion is expected. In the GUI or recorded state, confirm
   every arm reports `physical_motion_expected=false`,
   `cartesian_gate.operation_mode=simulation`, and
   `tcp_tracking_source_recommendation=reference_for_controller_simulation` or
   an equivalent `tcp_ref_stand` recommendation.

5. Verify deadman release stops motion intent. While both SpaceMouse deadmen are
   pressed, the policy path may send per-arm `TcpPoseTarget`. Release either
   deadman and confirm the source emits `Hold` or no motion intent according to
   the current lease/hold policy. If a SpaceMouse sample goes stale past
   `sample_stale_timeout_sec`, the source also stops target integration and
   emits `Hold`. When both devices are armed but centered, the dual source may
   emit per-arm `Hold`; this is the documented no-motion centered behavior.

## Required Evidence

Before trusting a run, inspect live state or recorder output for:

- `observed_mode=real`
- `observed_backend=rbpodo`
- per-arm `cartesian_gate.operation_mode=simulation`
- per-arm `controller_simulation_cartesian_enabled=true`
- per-arm `controller_simulation_streaming_cartesian_available=true`
- per-arm `controller_simulation_cartesian_enabled_for_current_command=true`
  while a `TcpPoseTarget` command is active; it may be `false` during startup
  `Hold`
- per-arm `physical_motion_expected=false`
- command-source lease active for `policy_runner`
- GUI pgmode line showing `operation_mode=simulation`,
  `physical_motion_expected=false`, `policy_runner_lease=active`, and
  `selected_tcp=tcp_ref_stand`
- `async_streaming_enabled=true`
- `async_streaming_mode=sdk_ack_worker`

Do not enable `cartesian_control.allow_in_real` or
`policy_runner.safety.allow_real_motion=true` for this workflow.

## Abort And Stop

- To stop manual commands, release both SpaceMouse deadmen. The policy runner
  should send zero twist on release or stale samples.
- To stop `policy_runner`, press `Ctrl-C` in its terminal.
- To stop the viewer, press `Ctrl-C` in the GUI terminal.
- To stop the server, press `Ctrl-C` in the server terminal. If a server fault
  or unexpected telemetry appears, stop `policy_runner` first, then the server,
  and do not restart until the local config and controller `pgmode` state have
  been rechecked.

This path is not physical real robot performance validation. It is controller
simulation readiness evidence for the manual SpaceMouse path only.

For the matching two-UMI / Vive-tracker relative Cartesian teleop path, see
`docs/runbooks/rbpodo_pgmode_umi.md`. It reuses this server profile but sends
`TcpPoseTarget` commands from `policy_runner` through `umi_dual_cartesian`.
