# rbpodo pgmode UMI Cartesian Teleop

This runbook launches:

```text
UMI / Vive trackers -> UDP or mock reader -> policy_runner -> rb_servo_server/rbpodo pgmode simulation -> rb_gui/viser
```

It connects to real Rainbow controller boxes only when the shared server
profile is launched, but the required robot `operation_mode` is `simulation`
and physical motion must remain unexpected. This is not physical real
Cartesian acceptance.

## Profiles

- Shared server template:
  `rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml`
- Shared local server config:
  `rb_servo_server/config/local/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.yaml`
- policy_runner config:
  `policy_runner/config/rbpodo_pgmode_umi_500hz_ack.yaml`
- wrapper:
  `tools/rbpodo_pgmode_umi.sh`

Ports match the SpaceMouse pgmode profile:

```text
server command bind: udp://127.0.0.1:50256
state recorder/debug: udp://127.0.0.1:50356
rb_gui / viser:      udp://127.0.0.1:50366
policy_runner:       udp://127.0.0.1:50376
```

## Safety Boundary

Keep these policy values:

```yaml
mode: real
action_source: umi_dual_cartesian
safety:
  allow_real_motion: false
  allow_rbpodo_controller_simulation_cartesian: true
```

The server must report per-arm `cartesian_gate.operation_mode=simulation` and
`physical_motion_expected=false`. Do not set `RB_ALLOW_REAL_CARTESIAN` for this
workflow. Do not modify measured calibration or `calibration/umi_retarget*.yaml`;
UMI live teleop uses relative-from-init clutching, not measured hand-eye
retarget.

The shared server profile opts into the rbpodo controller-simulation unreliable
status-field decode policy:

```yaml
servo:
  controller_simulation_treat_unreliable_status_fields_as_unavailable: true
  controller_simulation_async_supervision_nonlatching: true
safety:
  tracking_error_policy: fault_latch              # stays enforced for real mode
  controller_simulation_tracking_error_nonlatching: true
```

This policy is inert unless the controller-simulation motion gate is open
(`run_mode: real`, `backend_type: rbpodo`, `operation_mode: simulation`,
`servo.allow_controller_simulation_motion: true`, `RB_ALLOW_REAL_ROBOT=1`, and
`RB_ALLOW_REAL_MOTION=1`). When active, only `op_stat_self_collision` shape
validation and controller time plausibility are treated as unavailable. State
telemetry must still publish `rbpodo_diagnostics.raw`,
`rbpodo_diagnostics.unavailable_fields`, and
`rbpodo_state_decode_policy=controller_sim_unreliable_fields_unavailable`.
EMS, soft-estop, collision, SOS, and `op_stat_self_collision == 1` fault paths
remain enforced.

The async supervision non-latching option is also controller-simulation only.
When pgmode async ACK/q_ref supervision degrades, the server continues command
handling, sets top-level state `async_supervision_degraded=true`, keeps per-arm
async telemetry visible, and emits throttled WARN logs. Physical real mode and
non-async fault paths still latch.

The tracking-error non-latching option closes the same 1hr-stability gap on the
synchronous safety filter. The diagnostics_suspect controller's reference readback
lags the commanded joints, so the command-tracking divergence would otherwise
latch `TrackingError` mid-teleop. With
`safety.controller_simulation_tracking_error_nonlatching: true` (gated identically,
inert in real mode), that divergence is advisory: the server keeps following the
rate-limited desired target, sets top-level state `tracking_error_degraded=true`,
and emits throttled WARN logs. `tracking_error_policy` stays `fault_latch`. The
`controller_simulation_physical_motion` guard is excluded and still latches.

## Mock Preview

The tracked policy config defaults both readers to:

```yaml
mock_script: pgmode_umi_smoke
```

Preview without opening tracker sockets, HID devices, or setting env gates:

```bash
tools/rbpodo_pgmode_umi.sh server-dry-run
tools/rbpodo_pgmode_umi.sh policy-dry-run
```

`policy-dry-run` prints the command only. The mock script covers deadman
released, clutch engage, a small relative tracker move, gripper values, and
release.

## Operator Sequence

1. Prepare the shared ignored local server config:

```bash
tools/rbpodo_pgmode_umi.sh prepare
```

2. Start the server in rbpodo controller `pgmode` simulation:

```bash
tools/rbpodo_pgmode_umi.sh server \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation \
  --with-required-env
```

The delegated server wrapper may set controller-simulation env gates when
`--with-required-env` is present. It must not set `RB_ALLOW_REAL_CARTESIAN`.

3. Start the viewer:

```bash
tools/rbpodo_pgmode_umi.sh gui
```

4. Start UMI teleop:

```bash
tools/rbpodo_pgmode_umi.sh policy \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

For JSONL or HDF5 recording:

```bash
tools/rbpodo_pgmode_umi.sh teleop-record \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation

tools/rbpodo_pgmode_umi.sh hdf5-record \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation \
  --task "dual arm UMI pgmode simulation"
```

## Relative-Init Clutch Behavior

For each side, while that side's deadman is held:

```text
target_stand = T_arm_init * (inv(T_pika_init) * T_pika_now)
```

`T_arm_init` is latched from `snapshot.payload[side]["tcp_stand"]` on the
deadman rising edge. `T_pika_init` is latched from the same tracker sample.
Release clears both latches so the next press re-centers the hand and removes
accumulated drift. This is relative teleop only; SteamVR world alignment and
measured hand-eye calibration are not used.

The reader applies the fixed Pika/UMI gripper-tip offset:

```text
GRIPPER_OFFSET = [0.172, 0.0, -0.076] meters
```

`umi_dual_cartesian.r_align` can carry a fixed tracker-frame alignment as
either RPY `[rx, ry, rz]` radians or a 3x3 row-major rotation matrix. The
default is identity. Per-tick targets are clamped by
`max_linear_step_m` and `max_angular_step_rad`; optional workspace bounds clamp
target `x/y/z`.

## UDP Wire Schema

The Windows SteamVR/OpenVR publisher lives in the pika repo
(`pika/scripts/umi_teleop_publish.py`). The Linux reader expects one UDP JSON
object per tick:

```json
{
  "t": 123.456,
  "left": {
    "pose": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
    "gripper": 0.5,
    "deadman": true
  },
  "right": {
    "pose": [-0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
    "gripper": 50.0,
    "deadman": true
  }
}
```

`pose` is `[x,y,z,qx,qy,qz,qw]` in the SteamVR
`TrackingUniverseStanding` world. Only relative motion since clutch engage is
used. `gripper` may be normalized `0..1` or percent `0..100`; policy commands
carry percent units.

Example live reader config (tracked template:
`config/rbpodo_pgmode_umi_live.example.yaml`):

```yaml
umi_dual_cartesian:
  left:
    udp_endpoint: "udp://0.0.0.0:50380"
  right:
    udp_endpoint: "udp://0.0.0.0:50381"
```

Each side binds its own port; the publisher sends the same combined
`{"t","left","right"}` packet to BOTH ports and each reader extracts its side.

## Live Publisher (pika repo, Windows + SteamVR)

On the pika host (SteamVR running, Vive trackers + PIKA Sense connected), with
`pika/config/arms.json` mapping left/right `tracker_sn`/`com_port`:

```bash
conda run -n pika python scripts/umi_teleop_publish.py \
  --target-host <linux-robotics_lab-ip> \
  --left-port 50380 --right-port 50381 --rate 120
```

- Keys: `space` toggles both arms' clutch, `a` left, `l` right, `q` quit.
- Clutch toggle = PikaAnyArm trigger semantics: engaging re-snapshots init pose
  on the robotics_lab side (relative-init); releasing holds.
- `--no-sense` runs pose-only (no gripper). Gripper auto-ranges to `0..1` unless
  `--grip-open/--grip-closed` are given.
- Reader-side wire-schema parsing is covered by
  `policy_runner/tests/test_umi_dual_cartesian.py`; publisher-side packet
  building by `python scripts/umi_teleop_publish.py --selftest` in the pika repo.

## Required Evidence

Before trusting a run, inspect live state or recorder output for:

- `observed_mode=real`
- `observed_backend=rbpodo`
- per-arm `cartesian_gate.operation_mode=simulation`
- per-arm `controller_simulation_cartesian_enabled=true`
- per-arm `physical_motion_expected=false`
- per-arm `rbpodo_state_decode_policy=controller_sim_unreliable_fields_unavailable`
- per-arm `rbpodo_diagnostics.unavailable_fields` lists the suppressed
  controller-simulation fields when the decode policy is active
- `async_supervision_degraded=false` during healthy streaming; if it becomes
  true, inspect per-arm `async_streaming` telemetry and WARN logs
- `tracking_error_degraded=false` during healthy tracking; if it becomes true,
  the controller reference is lagging the command (advisory, not latched) — watch
  per-arm `command_reference_tracking_error_deg` and WARN logs
- `fault_latched=false`
- command-source lease active for `policy_runner`
- `TcpPoseTarget` commands while UMI deadmen are held
- `tcp_ref_stand` movement in controller simulation

If physical motion is expected or detected, stop policy_runner first, stop the
server, and do not restart until the local config and controller `pgmode` state
have been rechecked.

## Out Of Scope

- Measured hand-eye retarget or absolute-frame UMI policy consistency
  (teleop is relative-init / calibration-free; measured retarget only matters
  for absolute-frame policy data).
- Real non-pgmode Cartesian motion promotion.
