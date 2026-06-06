# policy_runner

`policy_runner` is the Python action-source layer for `robotics_lab`. It sends
UDP JSON commands to `rb_servo_server` and consumes the UDP JSON state stream.

Supported action sources:

- `hold`: receive state and keep policy output as no-op by default.
- `joint_sine`: small simulation-only joint target motion.
- `joint_velocity`: fixed simulation-only joint velocity command.
- `master_arm_joint`: joint-space teleop from `mo_master_arm` to
  `JointTarget`.
- `spacemouse_joint_velocity`: SpaceMouse six-axis input mapped directly to
  joint velocity commands.
- `tcp_delta`: small simulation-only `TcpDeltaStand` command.
- `spacemouse_cartesian`: SpaceMouse input mapped to `TcpTwistLocal`;
  simulation by default, with an explicit rbpodo controller pgmode simulation
  profile.
- `dual_spacemouse_cartesian`: two-SpaceMouse input mapped to per-arm
  `TcpTwistLocal` commands; simulation by default, with an explicit rbpodo
  controller pgmode simulation profile.

Joint actions remain joint-only. Geometry-aware safety gates protect Cartesian
and camera-related action sources. Cartesian action sources remain
simulation-only unless a config explicitly opts into the rbpodo controller
pgmode simulation carve-out and the server state proves
`physical_motion_expected=false`.

## Safety

Motion commands are blocked when:

- the state stream is stale or absent
- `fault_latched` is true
- `motion_state` is `FaultLatched` or `EmergencyLatched`
- joint state is invalid and `require_valid_joint_state` is true
- an action source declares `requires_geometry` and the calibration registry is
  missing, invalid, lacks robot mount geometry, or is not policy-ready for the
  configured mode
- an action source declares `requires_camera_geometry` and measured camera
  intrinsics/extrinsics are unavailable
- an action source declares `requires_valid_tcp_pose` and the state stream does
  not report valid TCP pose for both arms
- a Cartesian source does not observe `observed_mode: simulation`, unless the
  explicit rbpodo pgmode simulation SpaceMouse profile is active
- a Cartesian source sees a non-simulator `observed_backend`/`backend_type`,
  unless per-arm `cartesian_gate` telemetry proves rbpodo controller pgmode
  simulation with physical motion not expected
- configured mode is `real` and `allow_real_motion` is false, except after
  the rbpodo controller-simulation Cartesian carve-out has passed
- an action source requires camera or kinematics inputs that are unavailable

`joint_sine` and `joint_velocity` are simulation-only by default. In real mode
they do not send motion unless the config explicitly sets
`allow_real_motion: true`.

`spacemouse_joint_velocity` also remains behind the same stale-state,
fault-state, and real-motion gates. Button 0 is the default deadman switch:
when it is released, the action source emits no motion command.

`master_arm_joint` is joint-only and intended for supervised master-arm
teleoperation. The tracked example config keeps `safety.allow_real_motion:
false`; real robot commands are still blocked until an operator explicitly
changes that local run configuration and the server-side real-motion gates are
also satisfied. The default mapping is delta mode: the runner latches master
and robot joint anchors on deadman press, then sends `JointTarget` commands
from the relative master-arm motion.

Joint-only sources do not require the geometry registry. Future Cartesian or
camera policy sources must declare geometry requirements before emitting
commands. The default geometry config is:

```yaml
geometry:
  path: "calibration/active_calibration.yaml"
safety:
  allow_configured_estimate_geometry_in_simulation: true
  allow_configured_estimate_geometry_in_real: false
```

The current active calibration is a configured estimate. It is acceptable for
simulation geometry-aware policy tests when the simulation toggle remains true,
but it is blocked for real geometry-dependent policy by default because
`geometry_valid_for_real_policy` is false.

Cartesian sources are stricter than joint sources. They require fresh state, no
fault latch, valid joint state, and valid TCP pose for both arms. The default
path still requires simulation as the observed mode and simulator backend when
the backend is reported. The only non-simulator SpaceMouse carve-out is rbpodo
controller `pgmode` simulation: `mode: real`, `backend_type: rbpodo`,
`operation_mode: simulation`, `physical_motion_expected=false`, explicit
policy-runner opt-in, command-source lease, and server-published
`cartesian_gate` evidence. Physical real Cartesian commands remain blocked;
do not set `RB_ALLOW_REAL_CARTESIAN` for this workflow.

Only one command source should be active at a time. Do not run GUI teleop and
`policy_runner` teleop against the same `rb_servo_server` command port
simultaneously.

## Example

```bash
python3 -m policy_runner --config policy_runner/config/simulator_hold.yaml
```

The default command endpoint is `udp://127.0.0.1:50010`. The state subscriber
bind must match the `rb_servo_server` state publisher destination.

Simulation-only example configs:

- `policy_runner/config/simulator_hold.yaml`: no-op runner for state/command
  wiring checks.
- `policy_runner/config/simulator_spacemouse_joint_velocity.yaml`:
  SpaceMouse joint velocity teleop.
- `policy_runner/config/simulator_tcp_delta.yaml`: scripted stand-frame
  `TcpDeltaStand`.
- `policy_runner/config/simulator_spacemouse_cartesian.yaml`: SpaceMouse
  local-frame `TcpTwistLocal`.
- `policy_runner/config/simulator_tcp_twist_local.yaml`: same SpaceMouse
  `TcpTwistLocal` path with velocity-unit field names called out.
- `policy_runner/config/simulator_dual_spacemouse_cartesian.yaml`: two
  SpaceMouse devices mapped to left/right simulator TCP twists.

Real joint-only example configs:

- `policy_runner/config/real_master_arm_joint.yaml`: `mo_master_arm`
  joint-space teleop wiring. It is motion-blocked by default with
  `allow_real_motion: false` and uses loopback UDP endpoints.

The simulator examples use loopback simulator endpoints and do not enable real
motion or real Cartesian motion. The real master-arm example also keeps motion
blocked by default; it only defines policy-runner command/state wiring.

Rbpodo controller pgmode simulation example:

- `policy_runner/config/rbpodo_pgmode_spacemouse_500hz_ack.yaml`: dual
  SpaceMouse `TcpTwistLocal` teleop for the ACKON500 500 Hz controller
  simulation profile. It sends commands to `udp://127.0.0.1:50256`, listens to
  server state on `50376`, acquires the command-source lease, and requires
  server telemetry to report controller-simulation Cartesian enabled with
  `physical_motion_expected=false`. It intentionally keeps
  `safety.allow_real_motion: false`; controller-simulation motion is admitted
  only through `safety.allow_rbpodo_controller_simulation_cartesian: true` and
  server gate evidence.

Use `tools/rbpodo_pgmode_spacemouse.sh` for the prepared server config, GUI,
and recorder commands.

## Imitation Data Collection

The Docker simulator stack can run a passive policy recorder with `make sim-up`.
The recorder receives the `rb_servo_server` state stream and writes JSONL
episodes under `policy_runner/episodes` without sending commands.

SpaceMouse teleop collection is an explicit mode because it sends simulator
motion commands:

```bash
make sim-teleop-up
```

This uses `policy_runner/config/simulator_dual_spacemouse_cartesian_compose.yaml`
and records both `robot_state.jsonl` and `actions.jsonl` in the same episode.
The default HID paths are `/dev/hidraw1` for the left SpaceMouse and
`/dev/hidraw6` for the right SpaceMouse.

Train and run the V1 behavior-cloning baseline with:

```bash
make policy-train
make sim-infer-up
```

The V1 dataset is robot-state plus policy-command JSONL only. Camera/image
episodes remain a later merge step using camera timestamps and robot state
timestamps.

Dataset provenance and future demonstration labels are defined in
`docs/runbooks/policy_data_collection.md`. New policy datasets must distinguish
hardware-free `rb_simulator`, rbpodo controller `pgmode` simulation, and future
physical real demonstrations. Controller-simulation episodes should carry
`backend_type: rbpodo`, `run_mode: real`, `operation_mode: simulation`, and
`physical_motion_expected: false`; they must not be mixed with physical real
episodes without explicit filtering.

## HDF5 Episode Recording

Record teleop episodes to ACT-compatible HDF5 files, one file per episode:

```bash
python3 -m policy_runner hdf5-record \
  --config policy_runner/config/simulator_dual_spacemouse_cartesian.yaml \
  --task "pick up cup with left arm" \
  --operator user_a
```

The episode reset pose is anchored to the robot joint/TCP state from the first
state packet received by this command. Move the robot to the desired reset
configuration before launching the recorder. Press `Ctrl-C` to end the current
episode and flush it to disk.

Schema: `robotics_lab.episode.v1`. Actions are recorded as `TcpTwistLocal`
twists. States are absolute `q_actual`, `q_sent`, and `tcp_stand` with
quaternion. `reset_qpos_left/right` and `reset_tcp_stand_left/right` are stored
as HDF5 root attributes so training code can compute reset-anchored deltas.
The server `cartesian_control_snapshot` and `kinematics_snapshot` are stored
under `/config` with SHA-256 root attributes. Training copies those hashes into
behavior-cloning checkpoints and warns, or aborts with
`--strict-config-check`, if HDF5 episodes disagree. Inference compares the
checkpoint hashes with the live state stream once at startup and warns on
drift unless `--ignore-config-drift` is supplied.

The recorder also accepts optional dataset metadata with schema
`robotics_lab.policy_runner.dataset_metadata.v1`. HDF5 episodes keep the same
root schema and add optional fields when present: `q_ref_left/right`,
`tcp_actual_stand_left/right`, `tcp_ref_stand_left/right`,
`tcp_tracking_source_left/right`, diagnostics-suspect flags, backend send
timing, ACK policy, controller acceptance flags, action `source_id`,
`TcpTwistStand`, joint target/velocity commands, and raw SpaceMouse
axes/buttons. These fields are additive and do not change command emission.

Install the optional recorder dependencies first:

```bash
python3 -m pip install -e "policy_runner[recording]"
```

To include camera images from `camera_server` bundles, install the camera extra
and pass `--with-camera`:

```bash
python3 -m pip install -e "policy_runner[camera]"
python3 -m policy_runner hdf5-record \
  --config policy_runner/config/simulator_dual_spacemouse_cartesian.yaml \
  --task "pick up cup with left arm" \
  --with-camera
```

Camera frames are read from the latest complete `camera.bundle` metadata and
POSIX shared memory slots, then stored under
`/observations/images/<camera_name>` as `uint8` RGB datasets with one-frame
chunks and gzip compression. Configure `camera.expected_cameras` to record a
fixed subset; missing frames after a shape has been observed are zero-filled and
`bundle_age_us` marks missing bundles.

## Image-Conditioned Flow Baseline

`flow-train` is the default image-conditioned imitation baseline for HDF5
episodes. It is custom flow matching rather than OpenPI so the first baseline
keeps the action representation, masking, normalization, and SafetyGate runtime
path local to `policy_runner`. OpenPI 0.5 remains a later comparison path after
the robotics_lab dataset schema and simulator execution loop are stable.

Train from pika UMI or robotics_lab HDF5 episodes:

```bash
python3 -m policy_runner flow-train \
  --episodes-dir data/episodes \
  --dataset-manifest data/episodes/manifest.yaml \
  --camera-names fisheye,realsense_color \
  --exclude-camera-names realsense_depth \
  --single-arm-side left \
  --max-episodes 100 \
  --checkpoint outputs/flow_policy.pt \
  --vision-backbone resnet50 \
  --action-horizon 16 \
  --batch-size 32 \
  --epochs 100 \
  --write-eval-report outputs/flow_eval_report.md
```

The checkpoint schema is `robotics_lab.policy_runner.flow_matching.v1`.
Training also writes `dataset_stats.json` and `training_curves.jsonl` beside the
checkpoint, plus `flow_eval_report.md` and `flow_eval_summary.json` evaluation
artifacts. The validation metrics are rollout-free: `action_mse`,
`gripper_mse`, `chunk_endpoint_error`, `image_decode_count`, and
`missing_camera_count`. The evaluation summary also records dataset formats,
episode/frame/sample counts, camera decode and missing counts, action
percentiles per dimension, arm mask counts, checkpoint schema/SHA-256, and
manifest/audit warnings.

Check local ML dependency readiness before training:

```bash
python3 -m policy_runner ml-preflight --vision-backbone tiny_cnn
python3 -m policy_runner ml-preflight --vision-backbone resnet18
```

`tiny_cnn` is a small PyTorch-only CNN intended for CPU smoke tests and CI when
`torchvision` is absent or broken. Use `resnet18` or `resnet50` for production
visual training only after the preflight reports a healthy `torchvision` import.

The policy output is a high-level action chunk, not a 500 Hz low-level servo
target. Each step is a 14D vector:

```text
left dx,dy,dz,drx,dry,drz,grip,
right dx,dy,dz,drx,dry,drz,grip
```

Pika UMI single-arm episodes are mapped to the left arm by default and carry a
zero right-arm `arm_mask`. Pika UMI bimanual episodes written as
`observations/<left|right>/...` are loaded with both arms active and camera
names prefixed by side, such as `left_realsense_color`. Future robotics_lab
dual-arm episodes can provide left/right cameras, TCP proprioception, and
gripper values. Proprio features are reset-relative; action chunks are
current-relative. Dataset statistics normalize proprio, action chunks, and
camera images before training.

Audit HDF5 episodes before training:

```bash
python3 -m policy_runner hdf5-audit \
  --episodes-dir data/umi_episodes \
  --output-json data/umi_episodes/audit.json \
  --output-md data/umi_episodes/audit.md
```

The audit JSON schema is `robotics_lab.policy_runner.hdf5_audit.v1`. It reports
the detected format, frame count, timing, pose frame/format, camera names and
encodings, arm mask, action kind, warnings, and deployment blockers for each
episode. Supported layouts are `pika_umi_single_arm`, `pika_umi_bimanual`, and
`robotics_lab_dual_arm`. A single-arm Pika/UMI file such as
`episode_002.hdf5` is interpreted as `pika_umi_single_arm` and maps to the left
arm unless `--single-arm-side right` or a dataset manifest says otherwise.

Audit warnings include timestamp dt outliers, missing/corrupt images,
unsupported image encodings, mixed camera names across episodes, missing
gripper fields, unsupported pose formats, non-normalized quaternions, unknown
pose frames, single-arm default mapping, and action pose columns that exactly
match current pose columns. Warnings are training-readiness evidence. Deployment
blockers are stricter: any non-`stand` `pose_frame`, including
`steamvr_world`, remains blocked for real policy rollout unless a retarget
manifest supplies a measured transform to `stand`.

## UMI Import And Retarget Metadata

`umi-import` and `umi-convert` are repository-side file pipelines. They do not
import or require a live UMI hardware SDK. Install the optional HDF5/image
dependencies first:

```bash
python3 -m pip install -e "policy_runner[umi]"
```

Canonical bimanual UMI input is:

```text
/timestamp
/observations/left/pose
/observations/left/gripper
/observations/left/action                 # optional
/observations/left/images/<camera_name>
/observations/right/pose
/observations/right/gripper
/observations/right/action                # optional
/observations/right/images/<camera_name>
attrs: schema=robotics_lab.umi_episode.v1 or absent Pika-compatible layout,
       arm_names=left,right,
       pose_format=x,y,z,qx,qy,qz,qw,
       pose_frame=steamvr_world|slam_world|robot_stand|stand|...,
       umi_device_serials=<JSON>,
       capture_hz=<Hz>,
       retarget_status=missing|configured_estimate|measured
```

Import links raw HDF5 episodes into the output directory instead of duplicating
image payloads, then writes `manifest.json` and `conversion_report.md`:

```bash
python3 -m policy_runner umi-import \
  --input raw_umi_session_dir \
  --output-dir data/umi_episodes \
  --task "task name" \
  --left-device SERIAL_OR_NAME \
  --right-device SERIAL_OR_NAME \
  --retarget-config calibration/umi_retarget.yaml
```

Conversion writes a FlowHdf5Dataset-compatible HDF5 file. The
`robotics_lab_dual_arm` target stores `/observations/tcp_stand_left/right` and
`/action/tcp_delta_stand_left/right`; camera names are flattened with arm
prefixes such as `left_wrist_rgb`.

```bash
python3 -m policy_runner umi-convert \
  --input episode_raw.hdf5 \
  --output episode_robotics_lab.hdf5 \
  --format robotics_lab_dual_arm \
  --retarget-config calibration/umi_retarget.yaml
```

Retarget config template: `calibration/umi_retarget.example.yaml`. Required
fields are `schema`, `status`, `source_pose_frame`, `target_pose_frame`, per-arm
`T_stand_source`, per-arm `T_tcp_umi_gripper`,
`gripper_open_close_units`, and `quality`. The config hash is stored in the
manifest and converted HDF5 attrs. `status: measured` is required only when
`--require-measured-retarget` is passed; `missing`, `configured_estimate`, and
unknown status values remain real-rollout blockers while still allowing offline
training/import review.

The conversion report includes episode/frame/duration counts, per-arm frame
availability, camera decode samples, timestamp jitter, action step/velocity
distribution, gripper min/max and event counts, IK feasibility status,
workspace-envelope status, and retarget status.

Dataset manifests use schema `robotics_lab.policy_runner.dataset_manifest.v1`:

```yaml
schema: robotics_lab.policy_runner.dataset_manifest.v1
episodes_dir: data/umi_episodes
include_formats:
  - pika_umi_single_arm
  - pika_umi_bimanual
  - robotics_lab_dual_arm
single_arm_side: left
camera_names:
  - fisheye
  - realsense_color
exclude_camera_names:
  - realsense_depth
required_attrs:
  pose_format: x,y,z,qx,qy,qz,qw
retarget:
  pose_frame: steamvr_world
  target_frame: stand
  transform_status: missing
```

For training-only offline smoke, the flow loader may still use reset-relative
deltas. For real policy rollout, `retarget.transform_status` must be
`measured`; `configured_estimate`, `accepted`, and `missing` remain deployment
blockers.

The model uses a frozen vision encoder by default (`resnet18` or `resnet50` via
`torchvision`) with a placeholder `dinov3` plugin hook for later optional
integration. Multi-view camera embeddings are fused with a proprio MLP through
an MLP or Transformer condition encoder, then a rectified-flow decoder learns
`v_theta(x_t, t, cond)` over action chunks.

Run a trained checkpoint as a simulator Cartesian action source:

```bash
python3 -m policy_runner flow-infer \
  --checkpoint outputs/flow_policy.pt \
  --config policy_runner/config/simulator_dual_spacemouse_cartesian.yaml
```

`flow-infer` emits bounded `TcpDeltaStand` steps from the current chunk. Gripper
values stay in the action vector for training but are not sent to hardware in
this baseline. Required camera frames must be available when sampling a new
chunk; otherwise the source emits no motion intent. All inference intents remain
behind the existing `SafetyGate`, and real Cartesian motion remains blocked.

## SpaceMouse Joint Velocity

SpaceMouse support belongs in `policy_runner`, not `rb_gui`. The joint-velocity
source maps the device axes to joint velocities:

```text
tx -> J1 velocity
ty -> J2 velocity
tz -> J3 velocity
rx -> J4 velocity
ry -> J5 velocity
rz -> J6 velocity
```

This is not Cartesian control. FK/IK is not required for this action source.

Example config fields:

```yaml
action_source: spacemouse_joint_velocity
command_rate_hz: 100
spacemouse:
  selected_arm: left
  max_joint_velocity_deg_s: [5, 5, 5, 8, 8, 10]
  deadband: 0.08
  smoothing_alpha: 0.2
  require_deadman: true
  deadman_button: 0
```

The default `command_rate_hz` is 100 Hz. Valid values are 1-500 Hz; the policy
runner should not attempt to publish above the servo-loop frequency.

Hardware-free tests use `FakeSpaceMouseReader`. Real HID support is optional:

```bash
python3 -m pip install -e policy_runner[spacemouse]
```

Probe attached devices without starting `rb_servo_server`:

```bash
python3 policy_runner/scripts/probe_spacemouse.py --list
python3 policy_runner/scripts/probe_spacemouse.py
python3 policy_runner/scripts/probe_spacemouse.py \
  --left-path /dev/hidraw1 \
  --right-path /dev/hidraw6
```

The probe defaults to `/dev/hidraw1` for the left SpaceMouse,
`/dev/hidraw6` for the right SpaceMouse, and a 30 second duration.

## Cartesian TCP Commands

`tcp_delta` emits a small one-shot scripted stand-frame TCP delta using the
server `TcpDeltaStand` mode. `TcpDelta*` commands use meters/radians and are
debug jog primitives, not velocity streams. `spacemouse_cartesian` maps
SpaceMouse axes to a simulation-only local TCP velocity command using
`TcpTwistLocal`:

```text
tx, ty, tz -> TCP local vx, vy, vz in m/s
rx, ry, rz -> TCP local wx, wy, wz in rad/s
```

Example config fields:

```yaml
action_source: spacemouse_cartesian
runtime:
  startup_timeout_sec: 5.0
command_rate_hz: 100
spacemouse_cartesian:
  selected_arm: left
  frame: local
  max_linear_velocity_m_s: 0.03
  max_angular_velocity_rad_s: 0.2
  deadband: 0.08
  response_curve_gamma: 3.0
  require_deadman: true
  deadman_button: 0
```

Button 0 is the default deadman switch. When an armed source is released, it
emits one explicit zero `TcpTwistLocal` before returning to no-command idle.
While armed, a centered puck still emits explicit zero twist instead of going
silent, so the robot does not coast until command timeout.
The Cartesian SpaceMouse source always requires a deadman switch.
`response_curve_gamma` controls the soft deadband response; the default 3.0 is
cubic, lower values approach linear, and higher values give more precision near
center. Linear and angular twist components are clamped as velocities.
Deprecated `max_linear_step_m` and `max_angular_step_rad` aliases still parse
with a warning for migration only.

`dual_spacemouse_cartesian` keeps one `policy_runner` command source and opens
two SpaceMouse devices. The left and right samples are aggregated into one UDP
command packet so the two arms are updated together:

```yaml
action_source: dual_spacemouse_cartesian
command_rate_hz: 100
spacemouse_cartesian_dual:
  frame: local
  max_linear_velocity_m_s: 0.03
  max_angular_velocity_rad_s: 0.2
  deadband: 0.08
  response_curve_gamma: 3.0
  left:
    device_number: 0
    deadman_button: 0
  right:
    device_number: 1
    deadman_button: 0
```

Each arm has its own deadman button. When an armed arm is released, the source
emits one explicit zero `TcpTwistLocal` for that arm so the server stops
without waiting for command timeout. Already released arms emit no repeated
commands. Use `path` or `device` under each side when stable HID selection is
needed.
The HID reader drains a bounded batch of queued events on each read and returns
only the latest sample, avoiding stale FIFO backlog during high-rate SpaceMouse
input.

`policy_runner` does not generate `TcpLinearMove` trajectories. Use
`rb_servo_server/tools/send_tcp_linear_move.py` for simulator-only MoveL-style
test packets.

## Runtime Startup

`policy_runner` fails closed if no first robot state packet arrives before
`runtime.startup_timeout_sec`. The default is 5 seconds:

```yaml
runtime:
  startup_timeout_sec: 5.0
```

This prevents a runner from waiting forever on a misconfigured or absent state
publisher. A missing or empty `geometry.path` does not block joint-only action
sources, but geometry-dependent Cartesian sources are blocked with an explicit
safety reason.

## Command Packets

All packets include `seq`, `mode`, and `timeout_sec`. Lifecycle packets use the
top-level mode:

```json
{"seq": 1, "mode": "ArmMotion", "timeout_sec": 0.2}
```

Joint actions use per-arm modes so either arm can hold independently:

```json
{
  "seq": 2,
  "mode": "Hold",
  "timeout_sec": 0.2,
  "left": {"mode": "JointTarget", "q_target_deg": [0, -30, 80, 0, 60, 0]},
  "right": {"mode": "Hold"}
}
```

Supported command modes are `Hold`, `ArmMotion`, `DisarmMotion`,
`EmergencyStop`, `ResetFault`, `JointTarget`, and `JointVelocity`.
Cartesian packets additionally use `TcpDeltaStand` with per-arm
`tcp_delta_stand` one-shot jog payloads and `TcpTwistLocal` with per-arm
`tcp_twist_local` velocity payloads in simulation only. `TcpTwistLocal` units
are `vx,vy,vz` in m/s and `wx,wy,wz` in rad/s.
