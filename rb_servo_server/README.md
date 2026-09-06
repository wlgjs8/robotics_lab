# rb_servo_server

The [constrained preview executor](../docs/reference/preview_trajectory_execution.md)
is selectable as `flow_infer_preview` when built with
`RB_SERVO_ENABLE_PREVIEW_EXECUTION=ON` (the build default). Recorded replay,
clean native integration and the real-backend config preflight pass; consult
the linked results and operator command for a supervised physical trial.
It uses an off-servo optimizer with no pose-output low-pass filter. Existing
`flow_infer_fresh` retains its conditioner. The optional
`RB_SERVO_BUILD_PREVIEW_EXPERIMENTS=ON` adds offline recorded replay tools.

C++ control server for synchronizing two Rainbow RB5-850 arms through a shared `servo_j`-style control loop.

The server is designed for:

1. fast mock-mode development without robots,
2. Rainbow rbpodo real/controller-simulation backends through `IRobotBackend`,
3. Python VLA / imitation policy integration through UDP commands,
4. Cartesian TCP control.

## Current status

Implemented in this server:

- dual-arm same-tick servo loop
- mock backend
- guarded `RbpodoBackend` integration path, enabled only by explicit stack config
- actual UDP JSON command receiver
- minimal YAML config parser for the provided config files
- velocity/acceleration safety clamps
- tracking-error guard with configurable policy
- latched fault state for EmergencyStop / real-mode tracking errors / robot state errors
- fail-safe command validation so missing payloads do not become zero joint targets
- Hold mode preserving the last accepted/sent joint reference as a recoverable
  pause; measured-pose re-anchoring is reserved for explicit lifecycle
  transitions such as Init Motion start/no-op, freedrive exit, and fault reset
- capped filter dt so one late tick does not create a large motion step
- servo period/jitter/filter-dt/safety logging
- structured backend result taxonomy for mock and rbpodo paths
- direct I/O plus worker I/O as the supported real queue-sync path, with
  per-arm RT scheduling
- mandatory Pinocchio/Eigen FK, IK, and Cartesian math support
- Cartesian command routing when kinematics and Cartesian config gates are
  enabled
- gripper command forwarding to the out-of-process `gripper_server`
- controller-manager-referenced F/T pipeline, manual/automatic tare,
  stream/hold force laws, force gate, and bounded deviation overlay

Still pending:

- measured camera/robot calibration
- supervised hardware A/B for optional worker setpoint interpolation
- fast physical circle promotion and policy task success

The active real-mode safety source of truth is the root `README.md`,
`AGENTS.md`, and `docs/servo_backend_contract.md`. Historical rbpodo planning
notes are archived under `docs/archive/planning/`; they are not runnable
operator instructions.

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

The repository-level `make build` keeps this stack incremental. Layout-sensitive
`config.hpp` changes explicitly rebuild every shipped server object while
preserving the CMake build tree. Use `make rebuild` only for an intentional hard
reset of `build/rbpodo_real_gate`, such as cache or toolchain recovery.

The build requires Eigen3 and Pinocchio. Cartesian FK, IK, orientation
interpolation, frame conversion, and SE(3) delta math delegate to
Eigen/Pinocchio rather than local fallback math. Install Pinocchio through an
Ubuntu robotpkg package under `/opt/openrobots` using
`../scripts/install_deps_ubuntu.sh --profile hardware-free`, conda/mamba, or a
source install exposed through `CMAKE_PREFIX_PATH`.

## Run controller-simulation mode

```bash
cd /home/plaif/workspace/robotics_lab
make run MODE=sim
```

The tracked sim profile currently targets the physical controller boxes held in
pgmode simulation. It tracks `q_target`/`tcp_ref_stand`, keeps physical-real
Cartesian authority off, and fault-latches any encoder-motion indication. A
Virtual ControlBox, whose simulated `q_actual` moves, is not accepted by this
profile without a future explicit endpoint-topology contract.

When startup changes pgmode or activates a controller, the backend reopens and
re-primes the dedicated pipelined state socket after the transition is confirmed.
This prevents a response requested before the transition from becoming the final
startup verdict; failure to re-prime still fails startup closed.

For a hardware-free mock smoke, use a temporary YAML outside the repository and
pass it explicitly:

```bash
./build/rb_servo_server --config /tmp/<mock-config>.yaml
```

Stop the server with `Ctrl+C`.

Inspect timing:

```bash
python3 tools/plot_servo_log.py logs/servo_log.csv
python3 tools/analyze_servo_log.py logs/servo_log.csv
```

For direct rbpodo `request_data()` latency, record a bounded passive capture
alongside the already-supervised server run (the capture command does not start
the server or authorize motion):

```bash
cd /home/plaif/workspace/robotics_lab
scripts/capture_rbpodo_reqdata_timing.sh \
  --interface enp6s0 \
  --left-ip 172.28.60.200 \
  --right-ip 172.28.60.201 \
  --duration-sec 120 \
  --output /tmp/rbpodo_reqdata.pcapng

python3 rb_servo_server/tools/analyze_reqdata_timing.py \
  --servo-log rb_servo_server/logs/servo_log.csv \
  --pcap /tmp/rbpodo_reqdata.pcapng \
  --left-ip 172.28.60.200 \
  --right-ip 172.28.60.201 \
  --output-csv /tmp/rbpodo_reqdata_timing.csv
```

Use the interface carrying controller traffic; `dumpcap -D` lists available
interfaces. The primary servo CSV records both steady-clock and system-clock
boundaries around the SDK call. The analyzer uses `tshark` to add the host
capture timestamp of the outbound `reqdata` payload and the first inbound
CobotData SystemState packet, then reports these phases independently:

- `call_start_to_reqdata_tx_us`: SDK/host work before the packet reaches the
  host capture point.
- `reqdata_tx_to_response_first_rx_us`: controller response plus network time.
- `response_first_rx_to_request_data_return_us`: remaining TCP-frame delivery,
  SDK polling/parsing, and host scheduling before the SDK returns.
- `backend_read_outside_request_data_us`: `readState()` work outside the SDK
  call, including state mapping and fault classification.

The capture is duration-bounded and stores only the first 128 bytes of each
packet. The inbound timestamp is the first response-frame packet at the host
capture point, not a controller-internal timestamp or necessarily the final TCP
segment.
The direct-call fields are unavailable when `state_read_pipelined: true`, because
that path does not invoke the vendor SDK's `request_data()` method. Packet
captures contain controller state traffic and should be handled as diagnostic
artifacts.

The servo CSV also carries the latest chunk-frame receive age/interarrival,
policy inference timing, camera bundle/frame age and focus indicators, plus
DeltaTwist per-arm execution budgets, acceleration commands, and a stable
14-bit clamp mask. For `delta_preview`, it additionally records the Ruckig
projection error and the commanded-pose lead over measured TCP, including each
persistent-error count. `analyze_servo_log.py` summarizes these optional columns and
continues to accept older logs that do not contain them. These fields are CSV
telemetry only; they are not added to state JSON and do not affect control.

## Flow chunk preview controller

`cartesian_control`'s active `ruckig_follower.controller: delta_preview` profile is the
timestamp-aligned flow-infer path. It accepts only chunk overlay schema v3 with
valid camera-frame velocity proprio metadata, drops policy rows already elapsed
between observation and activation on the publisher, integrates the remaining
ee-local deltas with the canonical Eigen/Pinocchio SE(3) path, and previews the
result through the existing Ruckig position/velocity/acceleration chain. Both
arms consume the same aligned policy row; per-arm motion is preserved, including
the near-zero inactive-arm intervals present in sequential PIKA UMI episodes.

The projection-error and actual-lead thresholds and their consecutive-error
budgets are mandatory positive config values. `fallback_policy: fault` is also
mandatory for this controller. Missing v3 metadata, invalid velocity proprio,
or persistent infeasibility therefore holds and ultimately faults; the server
does not substitute guessed bounds. The legacy `delta_twist` controller remains
parseable for regression profiles but is no longer selected by the tracked real
flow profile.

## Hardware-free validation

Hardware-free validation runs C++/Python tests and, when a temporary mock config is
available, mock-mode smoke against that explicit YAML. Cartesian
behavior is covered by Pinocchio-backed C++ tests and active-stack smoke. For
controller-level simulation, use the rbpodo controller `pgmode` simulation
(`make run MODE=sim`) or the Rainbow virtual control-box VMs. The old
software-simulator-oriented Cartesian acceptance runner is no longer part of
this validation surface.

This lane is not Rainbow Robotics external simulator/OVA, real robot, privileged
Docker, or production network validation.

## Fault behavior

The server never falls back to `[0, 0, 0, 0, 0, 0]` for invalid commands, IK-unavailable commands, stale commands, or safety failures.

Fail-safe rule:

```text
valid command   → filtered/clamped target
invalid command → previous safe sent target
stale command   → Hold at the last accepted/sent reference
Cartesian/IK not available → previous safe sent target
EmergencyStop   → latch current/last-safe pose and ignore motion commands
real tracking error → fault latch by default
mock tracking error → snap target to actual by default
```

Reset a latched fault:

```bash
python3 tools/send_reset_fault.py
```

## Direct teaching (free-drive)

`Freedrive` hands one arm to a human guide. The controller only accepts
`freedrive_teach_on()` from an idle state (otherwise M151, "Cannot run this
function"), so the server arms it in stages — `arming_quiesce` → `arming_confirm`
→ `active` — waiting for `sdata.robot_state == 1` (Idle) before issuing teach_on
and for `sdata.is_freedrive_mode` before calling it engaged. `servo.allow_freedrive`
gates the whole path, fail-closed.

**Quiescing means the WIRE goes quiet, not just the servo loop.** From the moment an
arm leaves `off`, `currentSendPolicy()` returns `freedrive` and no target is staged —
but that alone only empties each `ArmWorker`'s mailbox. Once the worker owns the
cadence (`servo.io_model: worker` + `queue_sync.enable`, which is the RB5 control-box
sync configuration) it keeps the stream alive on its own: `SetpointInterpolator::sample`
holds at the newest setpoint indefinitely, by design, because a skipped send is a FIFO
entry the box never receives. The controller therefore stayed in `robot_state == 3` and
every freedrive request aborted with `quiesce timeout: controller never reported idle`
(measured 2026-09-06).

`DualArmServoLoop::setWorkerSendSuppressed()` closes that gap by gating the worker's
wire send directly, driven by `anyFreedriveActive()` every tick. Three properties
matter:

- **Only the send is gated.** The cadence keeps ticking and the queue-sync law keeps
  being stepped, with `streaming=false`. A caller that stops stepping does not slow the
  law down, it stops it (`Warmup` latched forever, measured 2026-08-26).
- **Re-entry re-drains by itself.** `streaming=false` drops `QueueSyncController` to
  `idle`, so resuming re-runs `warmup` → `drain` → `track` against the real queue
  rather than a pre-gap fill and a wound-up integral. No separate drain call exists or
  is needed.
- **Entering suppression drops the cached setpoints** (mailbox, interpolator ring,
  last-sent point). They describe the pose the arm is about to be hand-moved away from;
  resuming from them would interpolate the arm back to it, downstream of every
  loop-side safety clamp. `resyncArmAfterFreedrive()` handles the same hazard on the
  loop side.

Lifecycle commands are unaffected — that is the path `freedrive_teach_on/off` itself
takes, so it still reaches the controller while the wire is quiet.

Freedrive is the ONLY suppressed policy that stops the wire. `fault_latched`,
`emergency_latched`, `read_only` and pre-arming keep streaming the held reference,
which is the behaviour those paths were validated with; changing them is a separate
decision with its own queue re-entry evidence to gather.

## Real robot config boundary

Real motion is config-driven and operator-supervised. Do not run real robot
configs during hardware-free validation; real validation is a separate
human-gated task. Use the tracked `config/stack_real.yaml` directly and change
one reviewed acceptance-stage setting at a time. It must keep unaccepted motion
paths off until the relevant acceptance task explicitly enables
`servo.send_servo_commands: true` and, for Cartesian motion,
`cartesian_control.allow_in_real: true`.

Rbpodo joint states and commands preserve raw controller degrees. The tracked
stack configs use explicit raw-degree ranges; see
`../docs/joint_range_policy.md`.

J3/elbow is fixed to `[-150 deg, +150 deg]` in the tracked safety limits,
joint-limit barrier, and URDF/Pinocchio IK model. Do not widen it to make an
unreachable Cartesian pose appear solvable.

## Fresh chunk execution profile

The optional `flow_infer_fresh` profile preserves each stack's motion limits
and enables `fresh_chunk_replan` and `continuous_hold_resume`. Both flags require
enabled `delta_preview` and default to false. The real profile additionally
selects `output_smd.mode: position_lowpass2` at 4.5/3.5 Hz with damping
sqrt(0.5) and velocity/profile feedforward off. Both translation and rotation
are conditioned, with finite tracking lag. The optional
`deadline_jerk_minimization` search is implemented but remains false: recorded
IK command spectra favored the filter alone. The controller-simulation profile
keeps its existing disabled output conditioner. Fresh frames replan from the current sampled p/v/a instead
of waiting for the previous segment endpoint; repeated IK refusal holds the
nominal sent reference without repeatedly cold-starting the output filter.
State JSON advertises these capabilities in `chunk_execution_profiles` so
the policy can reject an unsupported profile before emitting commands.

See [design, selection and limits](../docs/plans/plan_fresh_chunk_execution_20260906.md).
Plan splice continuity does not imply continuity across hard safety holds or
prove physical vibration reduction. Force/tare and motion limits are unchanged.
See [conditioner tradeoff and replay limits](../docs/reference/follower_output_conditioning.md)
and [bounded state publication](../docs/reference/state_udp_payload_budget.md).

## Command channel

Current stack command endpoint:

```text
udp://127.0.0.1:50256
```

The stack state fanout uses `50356` (joint scope dashboard), `50366` (viser
GUI), `50376` (stack policy_runner/teleop_mux), `50378` (external flow-infer
readback). Gripper
command/feedback uses `50410`/`50420`.

Minimal command:

```json
{
  "seq": 1,
  "mode": "JointTarget",
  "timeout_sec": 0.2,
  "left": {"q_target_deg": [0, -30, 80, 0, 60, 0]},
  "right": {"q_target_deg": [0, -30, 80, 0, 60, 0]}
}
```

The C++ receive timestamp is used for timeout checks.

## Force-control status

The UMI stream gate now retains the last armed contact normal during its existing
scalar release. CSV traces distinguish the consumed gate from the later force
update and the geometric projection from its release slew. See
[behavior, replay tradeoff and next-run fields](../docs/reference/umi_stream_gate_release.md).
The change has offline validation only; the latest-left replay reduces ripple
while increasing transient goal error, so both need checking on the next run.

Force-control v2 is live. V1 was removed on 2026-08-26 and archived, then v2
was rebuilt from controller-manager's operator-calibrated sensor/tool presets.
The tracked real stack enables both `force_torque:` and `force_control:` and
the overlay has been exercised on hardware.

The measured sensor basis on this cell is left-handed (`det=-1`). Gate and
spring are an indivisible loader contract, and the wrench reference point and
compose pivot are both the TCP. Per-arm state JSON publishes
`force_torque`/`force_control` blocks with raw/compensated wrench, bias/tare,
coverage, selected law, gate, deviation, fence, and refusal telemetry.

An untared arm is never covered. The GUI's leaseless `TareForceSensor` and
`force_torque.auto_tare_after_init_motion` use the same 250-tick
`raw - gravity` average. Automatic tare invalidates the previous bias at the
InitMotion request, then waits for arrival, settle time, and a low sent-speed
condition before sampling.

Fresh planner-backed InitMotion requests also discard the selected arm's previous force and
Cartesian reference, including a no-op request, while retaining sent-joint
history for the joint brake. Repeated packets for the same logical request do
not repeat that reset. First-chunk waiting targets use the same force-compose
eligibility as their reference conversion, so a frozen deviation cannot be
subtracted repeatedly while coverage is recovering. The internal reference
deviation, strip eligibility and reset count are available in state/CSV logs;
see the root `docs/servo_backend_contract.md`. From the repository root, run
`ctest --test-dir rb_servo_server/build -R '^force_overlay_resume$' --output-on-failure`
for the hardware-free regression.

A command packet carrying a `force_control` object is still rejected because
the law is owned by tracked server config, not the client payload. Archived v1
design and evidence remain under `docs/archive/force_control_v1/`.

## Viser operator GUI

The native operator stack runs `rb_servo_server` together with the viser GUI and
`policy_runner` (no Docker):

```bash
cd /home/plaif/workspace/robotics_lab
make run MODE=sim
```

`make run` launches the servo server, the viser GUI, and `policy_runner`
side by side; `MODE=sim` uses the rbpodo controller-simulation path, and the
plain `make run` targets the real controllers. Build/install the stack first
with `make build` after editing source. The GUI receives UDP state
snapshots and sends only validated UDP JSON commands; the server is built with
Pinocchio enabled so FK/IK powers the GUI TCP target tests. See
`docs/gui_operator_console.md`.

For hardware-free mock runs, start `rb_servo_server` directly with an explicit
temporary mock config outside the repository. Docker remains in use only for
`camera_server` (managed by `make cam-up` / `cam-down` / `cam-status`).
