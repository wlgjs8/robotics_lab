# TASK: UMI dual-arm Cartesian teleop action source (rbpodo pgmode simulation)

## Goal

Add a new policy_runner action source `umi_dual_cartesian` that lets an operator
teleoperate **two** Rainbow RB3-730E arms with **two UMI / Vive-tracker handheld
devices**, driving `rb_servo_server` (rbpodo backend) in **controller `pgmode`
simulation only**. This is the live counterpart of the existing dual SpaceMouse
teleop, but the input is an absolute 6-DoF tracker pose mapped with the
**relative-from-init (clutch) scheme** — NOT a measured hand-eye calibration.

This must reach a working `pgmode` simulation demo with a **mock/replay tracker
reader** (hardware-free), plus a defined UDP wire schema for the real Windows
SteamVR publisher.

## Non-negotiable constraints (read AGENTS.md, REVIEW.md first)

- rbpodo backend only; `operation_mode: simulation`; `physical_motion_expected: false`.
- Do NOT enable physical real motion. No `RB_ALLOW_REAL_CARTESIAN`, no
  `operation_mode: real`. `policy_runner.safety.allow_real_motion` stays `false`;
  use only `allow_rbpodo_controller_simulation_cartesian: true` carve-out.
- Do NOT touch / weaken the measured-calibration gate. Teleop does NOT require a
  measured `calibration/umi_retarget*.yaml`; leave those files and their
  `status` untouched. (Measured retarget only matters for the absolute offline
  policy frame, not for relative teleop.)
- Match existing code style/idioms. Add tests. Keep changes scoped to the
  files listed below.

## The teleop math (verified from agilexrobotics/PikaAnyArm)

On every arm `side ∈ {left,right}`, while that side's clutch/deadman is held:

    target_stand = T_arm_init · ( inv(T_pika_init) · T_pika_now )

- `T_arm_init`  = robot TCP pose in stand frame, **latched once** on clutch
  rising edge, read from the state snapshot: `snapshot.payload[side]["tcp_stand"]`.
- `T_pika_init` = tracker pose, **latched once** on the same rising edge.
- `T_pika_now`  = current tracker pose (live).
- Middle term = body-frame delta of the tracker since clutch engage.
- On clutch release: stop emitting motion for that side (Hold) and clear both
  latches so the next engage re-snapshots (drift reset / hand reposition).
- Apply a fixed tracker→gripper-tip tool offset constant
  `GRIPPER_OFFSET = (0.172, 0.0, -0.076) m` (pika SDK) and an optional fixed
  frame-alignment rotation `R_align` (config, default identity) — these are
  known device constants, never per-session measured.

## Deliverables

### 1. Intent builder — `policy_runner/policy_runner/action_sources/tcp_delta.py`
Add `tcp_pose_target_stand_intent(*, left, right, left_gripper, right_gripper,
timeout_sec)` mirroring `tcp_twist_local_intent`, using
`_arm_payload("TcpPoseTarget", "tcp_target_stand", pose6, gripper_target=...)`.
`pose6 = [x,y,z,rx,ry,rz]` in stand frame (meters + rotation). **Confirm the
rx,ry,rz rotation convention** against the rb_servo_server `TcpPoseTarget`
parser (see `rb_servo_server/docs/network_protocol.md` §Units/Pose and the
cartesian controller source) and convert accordingly — do not assume.
`build_packet` is already generic, so no servo_command_client packet changes
should be needed; verify.

### 2. Action source — `policy_runner/policy_runner/action_sources/umi_dual_cartesian.py`
- `class UmiDualCartesianActionSource` with
  `next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None`.
- `requirements = cartesian_action_requirements(allow_rbpodo_controller_simulation=True)`.
- Constructor takes a left and right `UmiPoseReader`, step clamps
  (`max_linear_step_m`, `max_angular_step_rad`), `gripper_offset`, `R_align`,
  optional workspace bounds, `sample_hold_timeout_sec`, `timeout_sec`.
- Implement the relative-init/clutch logic above with rising/falling-edge
  latching per side. Clamp each tick's target against the previous target by
  `max_linear_step_m` / `max_angular_step_rad` (reuse/extend `clamp_tcp_delta`
  semantics) so a tracker jump can't command a large jump. Apply workspace
  bounds clamp if configured.
- Gripper: map reader gripper value → per-arm `gripper_target` (percent units),
  passed through the intent (separate from pose), like the twist intents.
- Define a small reader interface in the same module:
  `UmiSample(pose_xyzw: tuple[7], gripper: float, deadman: bool, monotonic: float)`
  and `class UmiPoseReader(Protocol): def read(self) -> UmiSample | None`.
- Provide `MockUmiPoseReader` (replay a scripted list of samples, like the
  SpaceMouse `mock_script`) AND `UdpUmiPoseReader` that ingests the live wire
  schema (below). Stale-sample handling like the SpaceMouse `sample_hold_timeout_sec`.
- Reuse existing SE(3)/quat helpers if the repo has them; otherwise add minimal,
  tested ones (xyzw quat ↔ matrix, matrix compose/inverse, matrix→pose6 in the
  server's rotation convention).

### 3. Live wire schema — UDP JSON tracker publisher (Linux ingestion side only)
The Windows SteamVR/OpenVR side (out of scope here, but document it) publishes
per-tick UDP JSON to a configured endpoint, e.g.
`{"t": <monotonic>, "left": {"pose": [x,y,z,qx,qy,qz,qw], "gripper": <0..1 or percent>, "deadman": true},
  "right": {...}}`. `UdpUmiPoseReader` parses this. Frame of `pose` = steamvr
world (TrackingUniverseStanding) — same frame the pika episodes used; only its
relative motion is used, so no world alignment needed. Document the schema in
the runbook.

### 4. Config — `policy_runner/policy_runner/config.py`
Add `UmiDualCartesianConfig` dataclass + `_umi_dual_cartesian_config(...)` parser
(mirror `DualSpaceMouseCartesianConfig`): per-side reader config (udp endpoint or
`mock_script`/inline samples, deadman field), step clamps, gripper offset,
R_align, workspace bounds. Wire it into the config struct and `from_dict`.

### 5. Registration — `policy_runner/policy_runner/action_sources/__init__.py` + `main.py`
Export `UmiDualCartesianActionSource`; register `action_source: "umi_dual_cartesian"`
in the builder in `main.py` so the policy config selects it.

### 6. Policy config — `policy_runner/config/rbpodo_pgmode_umi_500hz_ack.yaml`
Copy `config/rbpodo_pgmode_spacemouse_500hz_ack.yaml`; set
`action_source: umi_dual_cartesian`; keep `operation_mode: simulation`,
`physical_motion_expected: false`, `allow_real_motion: false`,
`allow_rbpodo_controller_simulation_cartesian: true`. Add a `umi_dual_cartesian`
section with left/right reader config defaulting to `mock_script` so it runs
hardware-free.

### 7. Server config / launch
Reuse the existing rbpodo pgmode spacemouse SERVER template
(`rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml`)
unchanged — the server side is input-agnostic. If a launch wrapper is the
cleanest path, add `tools/rbpodo_pgmode_umi.sh` mirroring
`tools/rbpodo_pgmode_spacemouse.sh` with `server`, `gui`, `policy`,
`server-dry-run`, `policy-dry-run` actions.

### 8. Tests — `policy_runner/tests/`
Mirror the dual SpaceMouse tests:
- relative-init math: known `T_pika_init`, `T_pika_now`, `T_arm_init` → exact
  expected `target_stand` (selftest numeric).
- clutch rising/falling edge: latch on engage, Hold + relatch on release.
- step clamp limits a tracker jump.
- gripper passthrough; stale-sample hold.
- `MockUmiPoseReader` end-to-end `next_intent` produces a `TcpPoseTarget` intent
  with correct left/right payloads.
Run the existing policy_runner test suite; keep it green.

### 9. Runbook — `docs/runbooks/rbpodo_pgmode_umi.md`
Mirror `docs/runbooks/rbpodo_pgmode_spacemouse.md`: pgmode-simulation-only
operator sequence, the UDP wire schema, mock vs live modes, the relative-init
clutch behavior, and the safety preamble (no physical real motion). Add a short
pointer from the spacemouse runbook or README to this one.

## Acceptance (must demonstrate)
1. `policy-dry-run` equivalent prints the command and a `mock_script`-driven
   `umi_dual_cartesian` config without opening hardware.
2. New unit tests pass; existing policy_runner suite stays green.
3. A `pgmode` simulation run with `MockUmiPoseReader` makes both arms follow the
   scripted relative tracker motion (state stream shows `tcp_ref_stand` moving),
   `fault_latched=false`, `physical_motion_detected=false`.
4. No physical-real gates touched; measured-calibration files untouched.

## Out of scope (note in PR, do not implement)
- Windows SteamVR publisher process (only its UDP schema is defined here).
- Measured hand-eye retarget / absolute-frame data consistency.
- Real (non-pgmode) Cartesian motion promotion.
