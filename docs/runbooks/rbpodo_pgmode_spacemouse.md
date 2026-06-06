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

```bash
tools/rbpodo_pgmode_spacemouse.sh prepare
```

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

## Launch

Start the server:

```bash
tools/rbpodo_pgmode_spacemouse.sh server \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation \
  --with-required-env
```

`--with-required-env` sets server-side rbpodo controller-simulation gates such
as `RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`,
`RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION`,
`RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN`, and
`RB_RBPODO_PGMODE_SIMULATION_CONFIRMED`. These env flags are required because
the server connects to real controller boxes in `pgmode` simulation. They are
separate from `policy_runner.safety.allow_real_motion`, which stays `false`.

Start the GUI:

```bash
tools/rbpodo_pgmode_spacemouse.sh gui
```

Run JSONL teleop recording:

```bash
tools/rbpodo_pgmode_spacemouse.sh teleop-record \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Run HDF5 teleop recording:

```bash
tools/rbpodo_pgmode_spacemouse.sh hdf5-record \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation \
  --task "dual arm SpaceMouse pgmode simulation"
```

## Required Evidence

Before trusting a run, inspect live state or recorder output for:

- `observed_mode=real`
- `observed_backend=rbpodo`
- per-arm `cartesian_gate.operation_mode=simulation`
- per-arm `controller_simulation_cartesian_enabled=true`
- per-arm `controller_simulation_streaming_cartesian_available=true`
- per-arm `controller_simulation_cartesian_enabled_for_current_command=true`
  while a `TcpTwistLocal` command is active; it may be `false` during startup
  `Hold`
- per-arm `physical_motion_expected=false`
- command-source lease active for `policy_runner`
- `async_streaming_enabled=true`
- `async_streaming_mode=sdk_ack_worker`

Do not set `RB_ALLOW_REAL_CARTESIAN` or
`policy_runner.safety.allow_real_motion=true` for this workflow.
