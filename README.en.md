# robotics_lab

`robotics_lab` is the integration workspace for a dual-arm RB3-730 system with servo control, the rbpodo backend (real robot + controller `pgmode` simulation), camera capture, policy_runner, and an operator GUI.

## Current Phase

The project is currently in **rbpodo pgmode-real physical robot bring-up**.
Simulator-first Cartesian acceptance hardening is largely complete; validation
now proceeds on the physical RB3-730E hardware.

Mock / rbpodo controller-simulation (pgmode) behavior that is repeatedly validated and stabilized:

- structured backend result and fault telemetry
- `JointTarget`
- `TcpPoseTarget`
- `TcpLinearMove`
- GUI operator controls
- policy_runner SpaceMouse path
- command-source lease/arbitration
- camera readiness contracts

What has additionally been validated on the physical robot is listed under
"Current Maturity" below. Real motion is still a fail-closed operation requiring
operator supervision, an E-stop in hand, and explicit gates; passing simulator
acceptance is not permission to move hardware.

## Current Maturity

Supported for mock / controller-simulation:

- mock dual-arm servo control
- direct and worker I/O modes (mock / hardware-free)
- FK/TCP state publication with quaternion fields
- TCP PTP and Linear commands
- mock camera server
- GUI viewer/operator console for mock/simulation
- policy_runner joint and Cartesian action sources
- mandatory Eigen3/Pinocchio C++ Cartesian math path for `rb_servo_server`

Run / validated on pgmode-real (physical RB3-730E hardware):

- read-only physical diagnostics parity (controllers `.200`/`.201`, `tcp_actual_stand`)
- dual-arm physical Cartesian circle tracking — slow, TUNED-1 profile, median
  tracking ~1.42° (`docs/runbooks/rbpodo_real_physical_circle.md`)
- UMI dual-arm Cartesian teleop (relative-init) driving `TcpPoseTarget` on the real
  robot; UMI `data_tcp` replay verified on hardware (ee_local + r_align)
- **pi0.5 (openpi) `flow-infer` `real_policy` full closed-loop rollout on the real
  robot** — `TcpPoseTarget` + gripper commands. Runtime/engineering
  validated: motion is smooth and in-distribution (async chunking removes the 500 Hz
  loop vibration; the absolute-proprio frame gap is fixed by reset-relative retrain).
  **Task success is still model-limited** (see below)
- real gripper motion — Pika Gripper Backend, `RB_ALLOW_REAL_GRIPPER` +
  `measured_gripper_available` gate
- server-side self-collision guard — async URDF-mesh `CollisionMonitor` (33 geoms /
  337 pairs), enforced in real (velocity barrier), stale/hard-breach fail-closed
- policy-side real-Cartesian safety gate relaxation (PR #13) → `rb_servo_server`
  is the sole real-motion safety layer
- controller `-2001` (suspect diagnostics) accepted in real mode (PR #12);
  EMS/SOS/soft-estop/`collision_occur`/unknown-mode/init-error still latch

Not yet production-ready:

- **policy task success** — rollout motion is smooth but inaccurate (e.g. the left
  arm reaches into a collision instead of grasping). This is a model-quality /
  data-coverage / appearance-domain-gap problem, not a runtime one; init-pose
  distribution matching is in progress (`umi_init_from_grasp.py`)
- force control (`provider: null`, `enable: false`)
- fast physical circle stages (15 cm / 16 s and above, transition ladder P7–P9)
- measured hand-eye / camera calibration is still pending for general
  geometry-dependent policy, but is **not needed** for the currently deployed pika
  Sense≡Gripper + ee_local + image-conditioned policy (reset-relative cancels the
  steamvr→stand R; the tool offset is a known constant) — so it is not a blocker for
  the current policy

## Source Of Truth

Start here:

- `AGENTS.md`: instructions for Codex/Claude/other agents
- `REVIEW.md`: current review baseline and open items
- `docs/current_review.md`: short redirect to `REVIEW.md`; do not duplicate review content there
- `docs/architecture.md`: system topology, terminology, motion primitive contract, safety boundaries
- `docs/code_architecture_map.md`: code-verified component map, ports/wire-formats, and a doc-vs-code drift list
- `docs/servo_backend_contract.md`: backend result, fault, worker I/O, and state telemetry contract
- `docs/frame_contract.md`: shared frames and calibration status
- `docs/hardware_free_validation.md`: hardware-free validation boundary
- `docs/runbooks/camera_acceptance.md`: real three-camera acceptance
- `calibration/active_calibration.yaml`: configured-estimate robot/camera/stand setup registry

Historical prompt/planning files are audit context. When they conflict with the files above, the files above win.

## Canonical Terms

```yaml
run_mode: mock | simulation | real
backend_type: mock | rbpodo
```

`run_mode: simulation` now refers only to the rbpodo controller `pgmode`
simulation flavor; the `rb_simulator` software-simulator backend was removed.

## Real And Controller-Simulation Topology

Physical system:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

The rbpodo controller `pgmode` simulation reuses this per-arm rbpodo endpoint
shape, targeting either a Virtual ControlBox VM or a physical box held in
`pgmode`. Site/VM configs live under gitignored
`rb_servo_server/config/local/`.

## Safety

Real robot connection and motion are **no longer gated on env vars.** The legacy
`RB_ALLOW_REAL_ROBOT` / `RB_ALLOW_REAL_MOTION` / `RB_ALLOW_REAL_CARTESIAN` /
`RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION` (and the other `RB_ALLOW_*`)
execution gates were removed from the server runtime. `run_mode`/`operation_mode`
are telemetry labels only and do not decide whether motion is allowed.

Real motion is owned solely by **site-local config
(`rb_servo_server/config/local/`) + the mode-independent safety layer**, and
config is the single decider: real motion requires the site config to enable it
explicitly (`cartesian_control.allow_in_real: true`) plus operator supervision.
Through this config-driven path a dual-arm physical Cartesian circle has already
run under supervision (`docs/runbooks/rbpodo_real_physical_circle.md`).
The policy-side `SafetyGate` real-Cartesian block was relaxed in PR #13, so for
real motion `rb_servo_server` is the sole safety layer (safety filter,
tracking-error latch, async URDF-mesh self-collision guard (`CollisionMonitor`), lease, deadman);
controller-simulation safety is unchanged. Accepting the controller `-2001`
suspect diagnostics in real mode is a per-arm config opt-in
(`allow_real_motion_with_suspect_diagnostics: true`, no env).
EMS/SOS/soft-estop/`collision_occur`/unknown-mode/init-error still latch
regardless of config.

Force control remains inactive:

```yaml
force_control:
  provider: null
  enable: false
```

## Motion Primitive Summary

- `JointTarget`: absolute joint-space point-to-point target.
- `TcpPoseTarget`: PTP / MoveJ-like Cartesian final-pose target; path not guaranteed. Real mode opens via site-local config and has been validated on hardware.
- `TcpLinearMove`: MoveL-like Cartesian path primitive with `constant` / `slerp` orientation modes.

## Common Commands

Python checks:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
```

Hardware-free C++ checks:

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

`rb_servo_server` C++ builds require Eigen3 and Pinocchio. Cartesian FK/IK,
orientation interpolation, frame conversion, and SE(3) delta math delegate to
Eigen/Pinocchio. Missing Pinocchio is a missing C++ dependency, not a fallback
runtime mode.

Cartesian math rebaseline is part of the Pinocchio-backed C++ test suite:

```bash
ctest --test-dir rb_servo_server/build --output-on-failure
```

Cartesian acceptance now runs against an already-running rbpodo/mock server:

```bash
python3 scripts/cartesian_acceptance.py --mode assume-running
```

Behavior is additionally validated on rbpodo pgmode-simulation / VM / real. The
prior simulator-first hardware-free Cartesian acceptance lane was retired with
`rb_simulator`.

Start the integrated operator stack (native, not Docker). `make run` brings up
`rb_servo_server` + the viser GUI + `policy_runner` (SpaceMouse + UMI teleop):

```bash
make run            # pgmode real (+ gripper follower)
make run MODE=sim   # pgmode controller-simulation
```

After editing source, build/install the stack first with `make build`.
For hardware-free controller-simulation, boot the two Rainbow virtual
control-box VMs with `make vm-up` and then run `make run MODE=sim`.

Open:

```text
http://127.0.0.1:8080
```

## Canonical Configs

Tracked stack configs:

- `rb_servo_server/config/stack_sim.yaml` — rbpodo controller-simulation (`make run MODE=sim`)
- `rb_servo_server/config/stack_real.yaml` — physical real stack (`make run`, operator-supervised)

Site-local mock / real / controller-simulation (VM·onbox) configs (gitignored):

- `rb_servo_server/config/local/*.yaml`
- `rb_servo_server/config/local/dual_real_readonly.yaml`
- `rb_servo_server/config/local/dual_real_motion.yaml`

No tracked runnable real robot config should be added. `README.md` is the
canonical root README; this English README is a best-effort translation.
