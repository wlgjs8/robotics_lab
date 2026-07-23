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

For the physical control boxes held in pgmode simulation, use the tracked
`policy_runner/config/flow_sim_offline.yaml` through
`make flow-infer-sim-offline`. It pairs with the normal `make run MODE=sim`
server path and recorded RGB-D replay on `tcp://127.0.0.1:5700`; it does not use
the physical camera service on `:5600`. The config disables physical-real arm
and gripper authority, while an `actual` tracking request automatically resolves
to the server-declared `tcp_ref_stand` in controller simulation and fails closed
if that reference is missing. The sim-only launcher defaults to W6; the
physical-real launcher remains W12.

To replay the policy from a *specific training observation* instead of looping
an offline camera publisher against live controller proprio, use the dedicated
teacher-forced path. It feeds the saved RGB-D plus the converter-equivalent 12-D
velocity state to the OpenPI server, executes the first W3 rows of each H24
prediction through the normal `TcpPoseTarget`/`delta_preview` controller path,
and exits after all full chunks are consumed:

```bash
# Terminal 1: controller boxes must already be held in pgmode simulation.
ACTION_SOURCE=none GRIPPER_SERVER=0 SCOPE_DASHBOARD=0 make run MODE=sim

# Terminal 2: no camera process is started or read.
FLOW_TRAINING_EPISODE_HDF5=/path/to/train_episode.hdf5 \
FLOW_TRAINING_EPISODE_VIDEO_DIR=/path/to/lerobot_episode_bundle \
FLOW_TRAINING_EPISODE_PARQUET=/path/to/episode.parquet \
make flow-infer-training-replay
```

`FLOW_INFER_CHUNK_EXECUTE_STEPS` defaults to 3, with overlay runway and
crossfade both fixed to zero. This executes 32 non-overlapping H24 predictions
for 96 policy steps. The tracked controller-simulation stack uses
`smoothing_window: 1` for this literal delta replay: applying its former
centered width-3 pose filter to a three-row integrated window shortened every
window endpoint by roughly one third of the last delta. Velocity, acceleration,
jerk, projection-error, actual-lead, IK, collision, floor, and lease gates
remain active. The runner defers its terminal `Hold` until the final three-row
controller window has elapsed.

The final LeRobot H264 stream directories and parquet are optional. When given,
the runner verifies that converted state/actions match the parquet and consumes
the exact stored training MP4 frames; otherwise it decodes the raw HDF5 RGB-D.
The replay is fail-closed to `controller_sim`, `gripper.backend: none`, full
rotation, sequential boundary chunks, chain anchoring, and
`last_emitted_continuous` re-anchoring. Artifacts under
`outputs/training_episode_replay` contain the raw model chunks, matching ground
truth chunks, source hashes, prediction error, and predicted/ground-truth path
metrics. Controller tracking remains visible in the normal action log, rollout
summary, and pgmode state monitor. This path never authorizes physical-real
motion and does not contact the live camera service.

The tracked real flow profile enables `force_recovery` with
`contact_behavior: continue` for the server-owned `cartesian_admittance` path.
Soft contact remains visible in status, but it does not invalidate the active
chunk, block inference, freeze the gripper, or emit a synthetic `Hold`; the
server projects loading policy increments and applies bounded compliance. A
hard force fault still increments the server `motion_epoch`, which invalidates
cached/in-flight policy work through the normal epoch path.

`contact_behavior: recover` remains available for the legacy guarded path. In
that mode either arm's `contact_active=true` invalidates cached/in-flight work,
emits bimanual `Hold` with frozen gripper targets, and waits for contact clear,
measured TCP settling, and a post-reset camera frame before one cold inference.
Its contact and settling deadlines report `force_contact_timeout`,
`force_settling_timeout`, or `camera_stale_timeout`. Live status and
`rollout_summary.json` expose the selected behavior, blocker, phase timing,
per-arm contact/force, TCP velocity, camera barrier, and worker generation.
OpenPI velocity proprio remains measured `camera_frame` data in either mode.
For velocity checkpoints, each inference now records whether both arms had a
complete measured-pose bracket at `[camera_time - policy_dt, camera_time]`.
The exact per-arm body deltas and any zero-substitution reason are included in
the diagnostic snapshot. The `delta_preview` server path requires this validity
bit; a missing bracket is therefore visible and fail-closed instead of silently
presented to the controller as a trustworthy zero velocity.

Chunk overlay schema `robotics_lab.chunk_overlay.v3` also carries the policy
observation step and activation step. Warm inference drops
`activation_step - observation_step` rows before activation, while cold start
keeps row 0 because motion was held during inference. One global emitted-step
sequence advances only when a policy row is committed, so the left arm, right
arm, and grippers remain on the same source row. Schema v2 remains readable by
diagnostic clients, but it is not sufficient to arm `delta_preview`.

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

Per-policy-step z/force/gripper diagnosis is also opt-in. The logger uses a
bounded background writer; a file or queue failure disables only this telemetry
and does not alter command generation:

```bash
FLOW_INFER_STEP_LOG=logs/bolt_pick_steps.jsonl \
./tools/flow_infer_real_policy.sh ...

python3 scripts/analyze_rollout_step_log.py \
  logs/bolt_pick_steps.jsonl \
  --png logs/bolt_pick_steps.png
```

Each `robotics_lab.policy_runner.rollout_step.v1` line carries the conditioned
absolute stand-frame command pose, the state-selected measured/reference pose,
the pre-FOH ee-local model delta, measured and commanded gripper openings, and
available force-control/F/T telemetry for both arms. Missing telemetry is
recorded as `null`, never substituted with a guessed value. Without `--png`, or
when matplotlib is unavailable, the analyzer still prints the text summary.

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
