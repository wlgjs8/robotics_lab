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

### Pika USB pairing

The physical wrist module carries its D405 and CH340-based Pika gripper through
one host cable. The CH340 adapters have no unique serial, so the real
`gripper_server` does not trust `ttyUSB` numbering or the legacy fixed
`/dev/pika-left` / `/dev/pika-right` udev links. At startup it:

1. reads the accepted left/right D405 librealsense serials from
   `camera_server/config/dual_realsense_d405.yaml`;
2. requires a live `camera.health` message from `tcp://127.0.0.1:5600`;
3. joins each camera's xHCI controller/root port to exactly one `1a86:7522`
   CH340 and opens that device through `/dev/serial/by-path`.

Every missing, duplicate, or conflicting identity fails closed before the Pika
backend opens. There is no fixed-port fallback. Start the physical camera
service before the real stack:

```bash
make cam-up-wrists  # or make cam-up with another accepted config containing both wrists
make run
```

If camera health or pairing is unavailable within five seconds,
`gripper_server` exits and `make run` tears down the stack with a nonzero
status. Replugging while the stack is running does not trigger automatic
reassignment; restart the stack so both pairs are proven again. A motionless
diagnostic prints the resolved mapping without importing/opening the Pika SDK:

```bash
PYTHONPATH=policy_runner .venv/bin/python -m policy_runner.gripper_server \
  --backend pika \
  --auto-pair-camera-config camera_server/config/dual_realsense_d405.yaml \
  --resolve-pairing-only
```

If `physical_port is not present in sysfs` reports a surviving USB device with a
missing `videoN` path, camera health may still contain the path from before a
USB reconnect. Restart the camera service with its existing rig/preset and run
the diagnostic again. The camera server refreshes identity from the active
pipeline on each start/reconnect. Pairing still requires the complete published
path to exist: accepting an old USB ancestor alone cannot prove the camera's
identity if devices have moved between ports.

Explicit `--left-port` / `--right-port` remain available for isolated
diagnostics but cannot be combined with auto-pairing. `MODE=sim` and an
explicit `GRIPPER_SERVER=0` remain camera-independent.

## Live Wrist-Camera Preview

With `camera_server` running, open the exact two RGB frames consumed by the
policy in a side-by-side OpenCV window:

```bash
./tools/subscribe_camera.sh
```

The preview subscribes read-only to `camera.bundle.policy` and its shared-memory
ring; it does not publish robot commands or interfere with policy inference.
Press `q`, Escape, or close the window to exit. Check the selected Python and
HighGUI backend without opening a window with:

```bash
PYTHONPATH=policy_runner .venv/bin/python -m policy_runner.camera_preview --check-gui
```

The repository `.venv` must contain exactly one OpenCV wheel. The native
operator stack uses GUI-enabled `opencv-python`; do not install
`opencv-python-headless` alongside it because both distributions own the same
`cv2` files. To repair an environment containing both variants:

```bash
.venv/bin/python -m pip uninstall -y \
  opencv-python opencv-python-headless \
  opencv-contrib-python opencv-contrib-python-headless
.venv/bin/python -m pip install -e rb_gui
.venv/bin/python -m pip check
```

## Flow/OpenPI Rollout

### InitMotion and tare completion

The arm-init override assigns a per-arm `init_motion_request_id` for every new
start/retry and reuses it while streaming that request. A matching Done latches
motion completion and changes the override to Hold until the enabled F/T
sensor is connected, its bias is valid, and auto-tare/sample settling is over.
Missing telemetry remains pending once enabled F/T was observed. A positive
`arm_init_override.ft_tare_wait_sec` is now a warning deadline: expiry displays
`init tare blocked` and continues Hold; a subsequently accepted tare releases
the latch. It does **not** time out into uncovered policy motion.

Explicit `ft_tare_wait_sec: 0` preserves the wait opt-out; disabled F/T and legacy
states that never report F/T have no tare prerequisite in this client. Manual
cancel, configured resume-on-failure, and InitMotion sent directly to the server
outside this override retain their existing semantics. These paths do not
create a bias; the server still refuses force coverage without a valid bias.

Separately, the current Flow action source checks every intent for enabled
force control. Each enabled arm must report `force_torque.bias_valid: true` and
`tare_state: accepted`; otherwise it exits before inference/publication. The
override's wait opt-out does not bypass this requirement. Complete both-arm
InitMotion/tare before a rollout. If a running policy observes tare invalidation
during InitMotion, restart the policy after accepted tare returns; the server's
native reset/resume behavior does not imply automatic Python-policy resumption.
`[arm_init_event]` console JSON records request IDs, start/status changes,
tare wait/timeout/ready, resume, cancel and failure. The `arm_init` state block
also exposes each request ID, elapsed tare wait, and timeout flag.

### Offline delta diagnostics

When chunk-row JSONL logging is enabled, each record now includes
`active_model_horizon` (the entire activated model chunk before per-arm
conditioning), `chunk_metadata`, and `rtc_enabled`. The activated chunk may
already have expired observation rows removed; it is not a fresh raw model
response. `left_delta`/`right_delta` remain the actual published, conditioned
inputs to `delta_preview`. These are additive fields under the existing schema.

The server CSV adds per-axis independent Ruckig minimum durations and guarded
target velocity/acceleration, segment count, the force advance gate/direction
and plan-rate gate actually used, and the output-SMD reseed flag. Axes 0–2 are
stand-frame translation; axes 3–5 are rotation tangent coordinates relative to
the segment reference. They are not joint indices or Euler-angle limits.

`tools/prepare_delta_replay.py`, the C++ `delta_follower_replay` executable, and
`tools/analyze_delta_replay.py` reproduce the follower/output-SMD reference path
without creating a backend or connecting to a robot. See
[the September 6 analysis](../docs/reports/griponly_replay_20260906.md) for
commands, reproduction accuracy, counterfactual limits, and results.

### Live rollout

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

The policy-side `force_recovery` action-source option remains removed. This is
separate from server force-control v2, which is live and publishes per-arm
`force_torque`/`force_control` state blocks. Force laws and recovery behavior
are server-config-owned; `policy_runner` does not reintroduce a client-side
force override or parse the removed v1 `force_control.contact_active` field. A
runner profile that still declares `force_recovery` fails to load.

The server still increments `motion_epoch` on a free-drive exit resync, and that
still invalidates cached/in-flight policy work through the normal epoch path.

The camera-readiness guard survives on its own: it still reports
`camera_stale_timeout` through `terminal_abort_reason`, and `rollout_summary.json`
still exposes its blocker and phase timing. OpenPI velocity proprio can use
measured, Python-command, or explicit `servo_command` history. The measured
`camera_frame` path records whether both arms had a complete measured-pose
bracket at `[camera_time - policy_dt, camera_time]`.
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

### Chunk activation scheduling

`--chunk-activation-mode fixed_steps` is the unchanged scheduling default:
execute the configured window (for example W4), prefetch at the configured or
existing automatic kick index, and activate at its boundary. The explicit
`ready_event` mode submits the next inference at chunk row 0 and activates a
ready result at the **next policy-row deadline**, including before W4 ends.
It does not replace half a row on a 500 Hz tick. Both arms and grippers still
commit the same selected source row, and the policy deadline grid survives
early replacements without accumulating command-tick lateness.

The request freezes the state payload, generation and `observation_step_seq`
together. Warm activation removes exactly the policy rows committed after
that snapshot; worker startup delay cannot silently move this reference.
There is at most one pending request or ready result. With no ready result,
the old chunk runs only up to the configured execute limit, followed by the
existing hold/feed-loss-watchdog behavior. No extra horizon or runway rows are
silently committed. InitMotion completion/cancel, a global init override and
server motion-epoch changes discard queued results and invalidate in-flight
generations. An expired early-replacement candidate leaves the still-valid
old rows available; inference failure remains idle/retry behavior.

For `servo_command` velocity/grip inputs, the request also freezes the selected
gripper values and sources before worker dispatch; inline inference uses the
same hook. A later command or jam-detector change cannot rewrite that request.
Gripper selection time is logged separately from the servo-state timestamp.
Images are still selected by the worker from the latest available camera
bundle, and `camera_minus_state_ms` records their difference from the frozen
state. This preserves the state/grip request snapshot; it does not establish
complete image-time synchronization.

`ready_event` rejects RTC, ensemble stitching, chain anchoring, sequential or
teacher-forced replay, and an explicit prefetch index before model initialization.
Those paths have fixed-prefix/window contracts and continue to use
`fixed_steps`. Chunk metadata adds `generation`, `activation_mode`,
`activation_reason`, `replaced_chunk_steps` and `tcp_target_profile`;
`execute_steps` in the overlay remains the **maximum executable window**, not
a promise that every row will be committed before a new chunk replaces it.
Timing telemetry exposes the activation mode and rejected-ready count.

The opt-in `flow_infer_fresh` TCP target profile is independent of scheduler
selection; both held-step and FOH command paths carry the selected profile.
The existing profile remains `flow_infer_smooth`. Profile selection does not
authorize physical execution or alter the lease, stale-state or fault gates.
Every fresh-profile state must advertise exactly one matching entry in
`chunk_execution_profiles`, with `enabled: true`, `controller: delta_preview`,
`fresh_chunk_replan: true` and `continuous_hold_resume: true`. Missing,
disabled, malformed or ambiguous support raises an error before camera-gate
fallback, inference, overlay publication or arm/gripper emission. This prevents
an older server from silently executing an unknown profile via its default
controller. The baseline profile does not require this new handshake.

The explicit `flow_infer_preview` profile additionally requires the same entry
to advertise `preview_execution: true` and a finite positive
`gripper_state_max_age_sec`. This selects the server preview executor only when
that executor is built and enabled; it never silently falls back to the fresh
profile. The launcher forwards `FLOW_INFER_TCP_TARGET_PROFILE=flow_infer_preview`.
Server integration and its validation status are documented in
[the preview execution contract](../docs/reference/preview_trajectory_execution.md).

For preview execution, new per-arm gripper targets require fresh
`preview_execution.left/right` telemetry with `enabled: true`, `active: true`,
`status: active` and `sample_time_ns` in the shared host monotonic clock. Both
sample and state-receive ages must meet the advertised bound. Waiting, faulted,
suppressed, missing or stale execution inhibits serial gripper dispatch and
removes that arm's gripper target from repeated motion packets. TCP/chunk
publication continues while the first plan is pending. An already accepted
gripper move may still finish; this guard prevents new commands and does not
retime model gripper rows to the server's independent execution cursor.

Hardware-free scheduling tests and a measured-latency replay use the real
dispatcher with in-memory inference completion and I/O. Run with an interpreter
that has the policy dependencies, including Torch:

```bash
PYTHONPATH=policy_runner python -m unittest discover -s policy_runner/tests -p test_chunk_activation_scheduler.py
python tools/replay_chunk_activation.py outputs/sweep/<run>.jsonl \
  --output outputs/scheduler_replay.json --policy-dt-sec 0.0334 \
  --tick-sec 0.002 --execute-steps 4
```

Use repeated `--exclude start:end` arguments to remove InitMotion/startup
windows relative to the first policy-step timestamp. This replay evaluates
waiting and scheduling only: higher inference request rate can change real
latency, observations and model output, and robot continuity must be tested in
the C++ consumer separately.

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
and gripper feedback age for both arms. Missing telemetry is
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
