# policy_runner

`policy_runner` owns Python action sources, safety gating, recording, HDF5
audit, and flow/openpi inference for the dual-arm RB servo stack.

## Public Motion Surface

The runner emits only these public motion primitives:

- `JointTarget`
- `TcpPoseTarget`
- `TcpLinearMove`

Lifecycle and admin packets such as `Hold`, `ArmMotion`, `DisarmMotion`,
`EmergencyStop`, `ResetFault`, lease acquire/release, safety-plane updates, and
`Freedrive` remain lifecycle/safety commands rather than motion primitives.

The ergonomic `CommandIntent.init_motion()` helper emits `JointTarget` with
per-arm `joint_target_profile: init_motion`; it never emits a separate packet
mode.

## Action Sources

Supported action sources:

- `hold`
- `joint_sine`
- `dual_spacemouse_pose_target`
- `umi_dual_cartesian`
- `teleop_mux`

`dual_spacemouse_pose_target` treats each SpaceMouse as a virtual target cursor.
On engagement it latches each arm's virtual target from `tcp_ref_stand` when
available, otherwise `tcp_actual_stand`. Each fresh non-neutral sample composes a
small local-frame pose step onto that cursor and emits an absolute stand-frame
`TcpPoseTarget`. Neutral, missing, or stale samples do not integrate. Release or
HID failure emits `Hold` according to the current lease/hold policy.

The tracked stack discovers the two `256f:c652` Universal Receivers by USB
vendor/product and HID interface `0`; it does not pin `/dev/hidrawN`. The
receivers expose no unique USB serial, so every newly discovered connection is
unassigned until the operator maps it in `rb_gui` under `조작 > SpaceMouse`.
Move each cap to identify its live activity row, select the left/right mapping,
and apply it. Assignment changes emit a Hold, clear accumulated targets, and
apply the configured startup-neutral policy before motion resumes. The tracked
real profile temporarily disables startup-neutral for dual-device diagnosis;
restore it after the receiver issue is resolved. Unplugged devices are removed
and rescanned; reconnecting on any USB port creates a new unassigned connection
without restarting the stack.

The GUI SpaceMouse panel exposes these input gates without bypassing robot
safety: deadman state, startup-neutral enable/progress, deadband, activation
threshold, raw six-axis values, and sample age are shown per connection. In the
tracked real profile the SpaceMouse button deadman is already off; cap
deflection is the engagement signal. A device stuck at
`gate=startup-neutral 0.00/0.30s` with a raw axis outside the displayed deadband
has a center-offset/input issue. Do not disable lease or stale-sample handling
to diagnose it.

With the stack stopped, test both HID readers without any robot command path:

```bash
PYTHONPATH=policy_runner python3 policy_runner/scripts/probe_spacemouse.py \
  --duration-sec 60 --rate-hz 100 --log --changes-only \
  | tee logs/spacemouse_dual_probe.log
```

Move the first device, the second device, and then both simultaneously. The
summary must show independent `changes` and a nonzero `peak_axis` for both
paths.

The `make run` SpaceMouse profile can also send gripper presets. Each
SpaceMouse controls the gripper on the same arm: physical left button closes
and physical right button opens. On the current HID mapping this is button `0`
for close and button `14` for open. Tune the target percent values in YAML:

```yaml
spacemouse_pose_target_dual:
  gripper_buttons:
    enable: true
    close_button: 0
    open_button: 14
    open_percent: 100.0
    close_percent: 10.0
```

Button-only gripper commands are per-arm `Hold` payloads with
`gripper_target`, so they do not request arm motion. Under `teleop_mux`, they
are honored when the mux is idle or already owned by SpaceMouse; UMI ownership
continues to suppress SpaceMouse intents.

## Flow/OpenPI Rollout

`flow-infer` and OpenPI remote rollout always compose ee_local policy deltas into
absolute `TcpPoseTarget` setpoints. There is no public command-family selection
or opt-in flag for target-pose rollout.

Live rollout still requires an explicit `--rollout-mode`. `controller_sim` and
`real_policy` require a policy dt from CLI or checkpoint stats; dry-run and
read-only modes may use the command-rate fallback.

For live OpenPI `real_policy`, keep `make run` running and start
`tools/flow_infer_real_policy.sh` or `make flow-infer-real` from another
terminal. The stack teleop_mux uses state port `50376`; the flow configs use
`50378`, so `ACTION_SOURCE=none` is not part of the normal flow-infer path. The
`make run` joint scope dashboard listens on the separate fanout port `50356` by
default and must not consume the external flow-infer port.

The tracked real flow profile enables `force_recovery`. If either arm reports
`force_control.contact_active=true`, the policy source invalidates its cached and
in-flight chunk once, blocks new inference, and emits bimanual `Hold` commands
with the last committed absolute gripper targets. After both contacts clear it
discards RTC, target, velocity-history, and camera-cache state. Recovery resumes
only after both measured TCPs remain below 0.002 m/s linear and 0.05 rad/s angular
velocity for `max(0.12 s, policy_dt)` and a camera frame newer than the reset is
available. The next request is a single cold inference re-anchored to the measured
poses. Recontact restarts the gate and keeps the command at Hold. Contact
ownership has its own 5-second deadline (`force_contact_timeout`); the 2-second
settling deadline starts only after both contacts clear and reports
`force_settling_timeout`, or
`camera_stale_timeout` when the newer-camera predicate is the remaining blocker.
Live status and `rollout_summary.json` expose `blocked_on`, phase elapsed time,
per-arm contact/normal force, measured TCP velocities, camera barrier/latest
sequence and age, and the in-flight worker generation. OpenPI velocity proprio
remains measured `camera_frame` data; recovery never substitutes command
velocity.

Motion-shaping and camera-supply diagnosis is additive and does not change the
command or safety path. Each inference records a bounded 64-event history with
queue/inference/ready timing plus the camera bundle sequence, age, sync skew,
frame numbers, camera-server health/drop counters, and post-crop left/right RGB
focus and luminance indicators. The same latest timing/camera snapshot is added
to the optional chunk-overlay packet and the rollout summary.

RGB evidence images are disabled by default. Enable bounded, best-effort JPEG95
snapshots of the exact post-crop RGB inputs with either an automatic directory
or an explicit path:

```bash
FLOW_INFER_DIAGNOSTIC_IMAGES=auto \
FLOW_INFER_DIAGNOSTIC_IMAGE_MAX_BUNDLES=120 \
./tools/flow_infer_real_policy.sh ...
```

`auto` writes under `logs/flow_obs_<timestamp>`; an explicit directory may be
used instead. Writing runs on a bounded background queue and reports queue,
capacity, and write-error counters, so storage latency never blocks inference.
Use `FLOW_INFER_DIAGNOSTIC_IMAGES=off` (the default) for normal runs.

## HDF5 Schema

Current recordings use schema `robotics_lab.episode.v1`. The action group stores
absolute target-pose datasets:

- `/action/tcp_target_stand_left`
- `/action/tcp_target_stand_right`
- `/action/mode`
- `/action/seq`
- `/action/action_host_time_ns`
- optional raw teleop fields such as SpaceMouse axes/buttons and deadman state

Legacy dataset layouts are not supported.
