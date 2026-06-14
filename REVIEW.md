# REVIEW.md

## Review Baseline

This review reflects the repository in **rbpodo pgmode-real physical bring-up**.
Simulator-first Cartesian hardening is largely complete and is now the regression
baseline; the active milestone is gated, operator-supervised validation on the
physical RB3-730E hardware (read-only diagnostics parity, then tiny motion, then a
slow physical circle, before any speed ladder). Real motion stays fail-closed and
passing simulator acceptance is never permission to move hardware.

This root file is the current review source of truth. `docs/current_review.md`
is only a short redirect here so review status does not drift across copies.
The numbered task log below (1–93) is an append-only historical audit of completed
tasks and is preserved verbatim, even where later real-motion work has moved past
its point-in-time caveats.

## Current Maturity

### Supported For Hardware-Free / Simulator Validation

- one simulated controller endpoint per arm
- structured backend result contract
- direct and worker servo I/O modes for simulator/mock use
- persistent simulator JSON-line transport
- command-source lease/arbitration
- FK/TCP state publication with quaternion fields
- `TcpPoseTarget` point-to-point Cartesian target
- `TcpLinearMove` simulator-only Cartesian path primitive
- `TcpTwistLocal` and `TcpTwistStand` simulator-only Cartesian velocity primitives
- GUI TCP PTP target controls
- GUI TCP Linear controls
- GUI Cartesian solve/path telemetry display
- policy_runner SpaceMouse Cartesian through `TcpTwistLocal`
- stand-frame floor plane constraint (`safety.floor_constraint`): joint-level FK
  backstop for all primitives + Cartesian z-clamp/twist v_z sliding assist,
  runtime-adjustable via leaseless `SetSafetyFloorZ` (config-bounded), GUI
  slider/plane visual; unit + config + GUI contract tests (note: pre-existing
  `tests/test_safety_filter.cpp` and `tests/test_state_publisher.cpp` are still
  not registered in CMakeLists — discovered during this work, left as-is)
- simulator-only Cartesian acceptance scripts
- mock camera and camera acceptance runbooks
- mandatory Eigen3/Pinocchio-backed Cartesian math in `rb_servo_server`

### Run / Validated On pgmode-real (Physical RB3-730E)

- read-only physical diagnostics parity (controllers `.200`/`.201`, `tcp_actual_stand`)
- dual-arm physical Cartesian circle tracking — slow, TUNED-1 profile, median ~1.42°
  (`docs/runbooks/rbpodo_real_physical_circle.md`)
- UMI dual-arm Cartesian teleop (relative-init) driving `TcpPoseTarget`; UMI `data_tcp`
  replay verified on hardware (ee_local + r_align)
- `flow-infer` `real_policy` full closed-loop rollout (pi0.5/openpi) on the physical
  robot — `TcpTwistLocal` streaming + gripper; the `_validate_real_policy` gate stays
  fully enforced and was satisfied via accepted/validated config. Runtime validated
  (smooth, in-distribution); task success is the remaining model-side gap
- real gripper motion via the Pika Gripper Backend (`RB_ALLOW_REAL_GRIPPER` +
  `measured_gripper_available` + `allow_real_gripper_motion`)
- server-side async URDF-mesh self-collision guard (`CollisionMonitor`, 33 geoms /
  337 pairs) enforced in real via a velocity barrier; stale/hard-breach fail closed
- policy-side real-Cartesian safety gate relaxation (PR #13); `rb_servo_server` is the
  sole real-motion safety layer
- controller `-2001` suspect-diagnostics acceptance in real (PR #12); EMS/SOS/soft-estop/
  `collision_occur`/unknown-mode/init-error still latch

### Not Yet Production-Ready

- policy task success — rollout motion is smooth but inaccurate (model quality / data
  coverage / appearance-domain gap, not runtime); init-pose matching in progress
- force/admittance/impedance control
- real `servo.io_model: worker` acceptance
- fast physical circle stages (15 cm / 16 s and above, ladder P7–P9)
- measured camera/robot calibration remains `configured_estimate` and is still required
  for general geometry-dependent policy, but is not needed for the deployed pika ee_local
  image-conditioned policy (reset-relative cancels the steamvr→stand transform)

## Safety Gates

Real robot connection:

```bash
RB_ALLOW_REAL_ROBOT=1
```

Real joint servo motion:

```bash
RB_ALLOW_REAL_MOTION=1
```

Real Cartesian/TCP motion:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

Accepting the controller `-2001` suspect diagnostics in real mode:

```bash
RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION=1
```

These gates are necessary but not sufficient. Config
(`cartesian_control.allow_in_real: true`) and operator supervision must also allow
the operation; they have already carried a supervised dual-arm physical Cartesian
circle. The policy-side `SafetyGate` real-Cartesian block was relaxed (PR #13), so
for real motion `rb_servo_server` is the sole safety layer; controller-simulation
safety is unchanged.

## Motion Primitive Review

### `TcpPoseTarget`

Status: simulator-supported; real mode validated on the dual-arm physical
Cartesian circle (gates + `cartesian_control.allow_in_real: true`).

Meaning: point-to-point Cartesian final-pose target. This is MoveJ-like at the TCP level: final pose is targeted, but the intermediate TCP path is not guaranteed to be linear.

Review requirements:

- quaternion target should be preserved end-to-end
- final position/orientation error must be visible in state telemetry
- GUI should describe it as PTP, not MoveL

### `TcpLinearMove`

Status: simulator acceptance candidate, plus a narrow rbpodo controller
`pgmode` simulation carve-out when `operation_mode=simulation` and
`physical_motion_expected=false` are reported by server telemetry.

Meaning: MoveL-like Cartesian path primitive. The TCP position reference should follow a straight line. Orientation mode must be explicit:

- `constant`: hold start orientation through the path
- `slerp`: interpolate start orientation to target orientation

Review requirements before real work:

- full mandatory-Pinocchio simulator acceptance must pass
- `path_done` telemetry must remain visible long enough for state subscribers and acceptance capture
- path line deviation must be checked over sampled state, not only final pose
- orientation deviation must be checked over sampled state
- real mode must remain blocked

### `TcpTwistLocal` / `TcpTwistStand`

Status: simulator acceptance candidate.

Meaning: streaming Cartesian velocity primitives.

Review requirements:

- server-side Cartesian velocity limits must remain active
- SpaceMouse must require deadman
- local-frame twist must preserve orientation when angular input is zero, within tolerance
- real mode must remain blocked
- physical real Cartesian mode must remain blocked; controller pgmode
  simulation is not physical real motion evidence

### `TcpDeltaLocal` / `TcpDeltaStand`

Status: low-level debug only.

Meaning: one-shot jog/debug command. These are not the default GUI target-move primitive.

## Current Validation Commands

Expected Python checks:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
```

Expected C++ hardware-free gate after dependencies are installed:

```bash
./scripts/codex_gate.sh HARDEN-10
```

Expected Cartesian math rebaseline after Pinocchio is installed:

```bash
./scripts/codex_gate.sh CART-MATH-03
```

Expected Cartesian simulator acceptance after Pinocchio is installed:

```bash
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05
```

## Current Circle Tracking Simulator Baseline

Circle tracking evidence is simulator-only and is not permission for real robot
motion. Use `scripts/generate_circle_benchmark_report.py` to produce a markdown
report and CSV table before changing this section.

Current recorded baseline candidate:

- Artifact: `artifacts/circle_tracking/left_twist_stand_15cm_16s_after/summary.json`
- Profile/controller: `circle_15cm_16s`, `twist_stand`, left arm
- Evidence: `repeat=3`, `radius_gain=0.994`, `rms_error_m=0.00222`,
  `p95_error_m=0.00337`, `fault_latched=false`, worker drops `0`
- Result semantics: artifact result is `completed`, not `pass`, because no
  explicit benchmark thresholds were supplied. It meets the current simulator
  baseline promotion criteria but should not be described as a threshold pass.

Stress evidence is not real-ready. The latest local GENE-style stress artifact
`artifacts/circle_tracking/gene_15cm_4s_after/summary.json` is useful for
tuning comparison only and must not be copied directly to hardware speed or
gain settings.

When replacing this baseline, record:

```text
Current circle tracking simulator baseline: <artifact path>, profile
circle_15cm_16s, controller <controller>, repeat <N>, radius_gain <value>,
rms_error_m <value>, p95_error_m <value>, result <completed/pass>.
Simulator-only; not real-ready.
```

## Current rbpodo Controller-Simulation Circle Status

rbpodo controller-simulation circle reporting is available through
`scripts/generate_rbpodo_circle_report.py`. This evidence category is distinct
from `rb_simulator`: it connects to real Rainbow controller boxes in `pgmode`
simulation and should normally score `tcp_ref_stand`, not physical
`tcp_actual_stand`.

Current recorded rbpodo controller-simulation baseline:

- Best stable baseline: pending; no reviewed `circle_15cm_16s`
  `tcp_ref_stand` artifact is recorded here.
- Best GENE-style ACKON500 stress result:
  `ackon500_best_server_circle_phase005_t2_080_alpha08_path06_ori06`,
  tracked as
  `rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz_goal.example.yaml`
  and `configs/rbpodo_circle_ablation/ackon500_gene_goal_best.yaml`.
  Evidence category is controller-reference lower-bound only:
  500 Hz, `sdk_ack_worker`, ACK-observed semantics, 15 cm / 4 s,
  `repeat>=5`, `tracking_source=tcp_ref_stand`, about 1.5 mm RMS, and about
  3.3 ms effective phase latency.
- Official repeatability validation matrix: pending operator run via
  `configs/rbpodo_circle_ablation/ackon500_gene_repeatability.yaml` and
  `tools/rbpodo_ackon500_gene_goal.sh --profile repeatability`. Required
  evidence is three left-arm and three right-arm best-profile rows with
  deterministic `repeatability_summary.csv/json` and
  `repeatability_report.md`; this does not retire the diagnostics-suspect
  caveat or approve physical real motion.
- Real physical circle benchmark: a slow dual-arm physical Cartesian circle has
  now been run under operator supervision (TUNED-1, median tracking ~1.42°;
  `docs/runbooks/rbpodo_real_physical_circle.md`). The faster/higher-speed circle
  ladder stages (15 cm / 16 s and above, P7–P9) remain not run and not approved,
  and `diagnostics_suspect_unresolved` (vendor `-2001` field semantics) is still
  open.

**ACKON500 PASS is controller-reference lower-bound evidence, not physical TCP tracking.**

The GENE 26.5 / ACKON500 default is a controller-simulation high-performance default only. It is not the physical-real default until the physical promotion ladder produces actual TCP tracking evidence.

The explicit default-profile registry is
`configs/control_defaults/gene_26_5_ackon500_controller_sim.yaml`. Validate it
with `python3 scripts/validate_control_defaults.py --defaults
configs/control_defaults/gene_26_5_ackon500_controller_sim.yaml`; the optional
report path is
`artifacts/control_defaults/gene_26_5_defaults_report.md`.

Required report fields keep this boundary machine-readable:

```yaml
physical_readiness:
  status: blocked
  blockers:
    - diagnostics_suspect_unresolved
    - physical_reference_to_actual_error_unmeasured
    - stop_resetFault_unverified
    - camera_tcp_calibration_unresolved
    - no_tiny_physical_acceptance
  next_required_acceptance:
    - read-only diagnostics parity
    - tiny joint no-op physical or approved safe mode
    - tiny physical joint move
    - tiny physical Cartesian move
    - low-speed circle
    - then speed ladder
controller_reference_result:
  status: pass|fail
  explanation: "tcp_ref_stand lower-bound evidence"
physical_tracking_result:
  status: not_measured
```

Transition ladder before fast physical circles:

1. Controller pgmode simulation repeatability
2. Right arm
3. Dual arm
4. P0 diagnostics root cause
5. Real controller read-only
6. Tiny physical acceptance
7. Slow physical circle
8. Fast physical circle only after approval

Before any physical circle discussion, record a rbpodo controller-simulation
report with `physical_motion_expected=false`, `physical_motion_detected=false`,
`tracking_source=tcp_ref_stand`, pgmode simulation confirmation, ACK semantics,
and q_ref/q_actual update rates. Controller-simulation results may guide future
low-speed parameter selection, but cannot be copied directly to real motion.

ACKON500-BEST-PROFILE-PROMOTION-01 promotes the current best 500 Hz
controller-simulation profile for reproducibility. It keeps
`operation_mode: simulation`, `cartesian_control.allow_in_real: false`,
`physical_motion_expected=false`, and `disable_waiting_ack: false`. It does not
claim physical real readiness.

The GENE/UMI policy-transition documentation path keeps HDF5 `hdf5-audit`
outputs, `flow-infer` `rollout_summary` files, controller-simulation
repeatability reports, pgmode transition reports, and the GENE 26.5 /
ACKON500 control-default report in an Artifact manifest / `artifact_manifest`.
This is evidence inventory only. (Status note: `real_policy` is no longer
ladder-blocked — its gate was satisfied via accepted/validated config and a full
`real_policy` rollout has since run on the physical robot; see "Run / Validated On
pgmode-real" above. `real_readonly`/`real_supervised` remain the no-motion lanes.)

RBPODO-CIRCLE-STATE-SOURCE-01 adds a controller-simulation-only Cartesian
state-source policy so rbpodo pgmode simulation can integrate and guard against
controller reference `q_ref` / `tcp_ref_stand` while still publishing and
monitoring physical `q_actual` separately. Physical real and rb_simulator paths
remain actual-state based.

## Open Review Items Before Real Robot

1. Full C++ hardware-free gate must pass on the development machine.
2. Mandatory Eigen3/Pinocchio Cartesian math gate must pass.
3. Full Cartesian simulator acceptance must pass repeatedly and preserve `acceptance_results.json` / `.csv` artifacts.
4. `TcpLinearMove path_done` telemetry persistence must remain covered by C++ tests.
5. Constant-orientation mismatch semantics must be explicit and tested.
6. Real rbpodo read-only acceptance must be run separately.
7. Real rbpodo stop/reset behavior remains operator-intervention-only until verified API wiring exists.
8. Real motion remains blocked.
9. Real camera acceptance remains separate.
10. Measured calibration is still absent.
11. ACKON500 controller-reference PASS remains physical-readiness blocked until diagnostics parity, tiny physical acceptance, and slow physical circle acceptance are complete.
11. Run BENCH-CIRCLE-01 simulator benchmark and record artifacts before real robot Cartesian testing. Latest CART-SERVO-01 conservative gate evidence: `artifacts/circle_tracking/bench_circle_01` safe 5 cm / 10 s threshold pass; `artifacts/circle_tracking/left_twist_stand_15cm_16s_after/summary.json` is the current 15 cm / 16 s simulator baseline candidate but is `completed`, not threshold `pass`. CART-SERVO-02 changes benchmark runs without thresholds from `pass` to `completed` so poor tracking cannot be mislabeled as a performance pass. CART-SERVO-03 adds the four-profile regression sequence and `scripts/compare_circle_benchmarks.py` for before/after summaries. GATE-BENCH-00 registers follow-on gate names for circle ablation, tuning, feedback comparison, server-side circle experiments, and reporting. BENCH-ABLATION-01 adds the matrix runner for factor-separated simulator-only circle tracking experiments; run and archive ablation artifacts before promoting any stress settings. CART-TUNE-02 adds named simulator circle profiles for baseline, stress, and conservative real-candidate separation; none is real-ready. BENCH-CIRCLE-FEEDBACK-01 adds simulator-only closed-loop benchmark modes to compare open-loop twist drift against command-source feedback compensation. BENCH-REPORT-01 defines baseline promotion rules, stress interpretation, and the real-candidate parameter policy.
12. SUPPORTED-SCOPE-RBPODO-500HZ supersedes the removed raw script TCP comparison track. Supported real-controller scope is rbpodo only; mock and simulator remain hardware-free validation surfaces.
13. Unsupported raw script TCP backend code, configs, scripts, tests, runbooks, and gates are removed from the active surface. Reintroducing direct raw script command paths requires a new accepted safety plan.
14. Real-controller command/control defaults are standardized at 500 Hz with `servo_t1_sec: 0.002`. Manual non-500 YAML overrides may remain parseable for compatibility, but they are not supported profiles.
15. Supported-scope gates now scan source, docs, configs, and scripts for removed comparison surfaces and unsupported robot-control defaults.
16. Rbpodo state/diagnostic tooling remains separate from motion approval; raw data-port capture is read-only diagnostic evidence only.
17. Real motion remains blocked unless the normal real robot gates, config gates, and a future explicit real-motion acceptance task all allow it.
18. GATE-RBPODO-00 registers rbpodo servo parameter, ACK semantics, acceptance, and docs task names. This is gate registration only; it does not change rbpodo behavior or enable real robot motion.
19. RBPODO-SERVO-PARAM-01 makes rbpodo `move_servo_j` parameters explicit as `servo_t1_sec`, `servo_t2_sec`, `servo_gain`, and `servo_alpha`; deprecated aliases warn, duplicate old/new keys fail, and real motion configs must align `servo_t1_sec` with `servo.rate_hz` unless explicitly accepted.
20. RBPODO-ACK-01 makes rbpodo ACK-on/off semantics observable. ACK-off remains non-default, requires `RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1` for real motion, and must be treated as socket-send evidence rather than controller-acceptance evidence.
21. RBPODO-ACCEPT-01 now documents the supported 500 Hz ACK-on rbpodo acceptance profile. Real execution remains pending; default mode is read-only, ACK-off is diagnostic-only, and tiny real motion is reserved for a future approval task.
22. RBPODO-DOC-01 documents rbpodo Servo J parameter mapping, 500 Hz rate matching, ACK-on/off interpretation, and the staged real acceptance sequence. Pending real items remain explicit read-only/no-op evidence and a separately approved tiny real joint motion task.
23. RBPODO-READONLY-DIAG-01 allows explicitly configured read-only rbpodo diagnostic startup to publish unsafe/faulted controller state after valid state acquisition. This is not motion readiness; `send_servo_commands=true` startup remains strict and real motion gates remain unchanged.
24. RBPODO-STATUS-FIELDS-01 preserves raw rbpodo status fields in state JSON and treats suspicious status layouts as motion-blocking diagnostics. Read-only bring-up may publish the raw fields after valid joint acquisition, but suspicious diagnostics remain unsafe and real motion gates are unchanged.
25. RBPODO-JOINT-WRAP-01 adds explicit startup-only joint wrapping diagnostics for rbpodo bring-up. Raw controller joint values remain published, motion target wrapping is refused, and real motion gates remain unchanged.
26. RBPODO-BRINGUP-TOOLS-01 adds a read-only rbpodo state dump tool and improves acceptance startup timeout diagnostics with server return code, log tail, and likely causes. It does not send motion or modify controller state.
27. Backend comparison gates and templates are superseded by the supported-scope rbpodo-only gates. Rbpodo is the only supported real backend.
28. Removed comparison configs under the backend comparison matrix are no longer active templates. Use rbpodo read-only or 500 Hz controller-simulation configs instead.
29. Persistent raw script socket probes are removed from the supported workflow. Rbpodo diagnostic probes remain read-only and separately gated.
30. Controller-simulation Servo J no-op acceptance is covered by the rbpodo 500 Hz acceptance runner, with pgmode simulation and real-controller confirmation required.
31. Raw data-port capture remains read-only diagnostic evidence for rbpodo state investigation and does not introduce a separate backend.
32. Backend comparison matrices are replaced by rbpodo 500 Hz acceptance, circle, and repeatability matrices.
33. Backend comparison reports are replaced by rbpodo-supported-scope reporting and ACKON500 evidence summaries.
34. GATE-RBPODO-CIRCLE-00 registers the rbpodo controller-simulation circle benchmark task names. This is gate registration only; controller pgmode simulation runs remain opt-in through explicit `CODEX_RUN_RBPODO_*` variables and tool-level confirmation flags, future 15cm/4s reports must keep `q_ref` and `tcp_reference` evidence explicit, and real physical motion gates remain unchanged.
35. RBPODO-CIRCLE-CONFIG-01 adds rbpodo controller-simulation templates for read-only diagnostics, Servo J no-op, stable 15 cm / 16 s circle tracking, and GENE-style 15 cm / 4 s stress. The templates use `operation_mode: simulation` for Rainbow pgmode simulation, keep `allow_in_real: false` for Cartesian control, leave local YAML out of git, and do not approve physical robot motion.
36. RBPODO-REFERENCE-TCP-01 publishes explicit actual/reference TCP telemetry: legacy `tcp_base`/`tcp_stand` remain actual aliases, `tcp_ref_*` is computed from controller reference joints when valid, and controller pgmode simulation reports should use `tcp_ref_stand` only as measurement telemetry, not as a physical safety signal.
37. RBPODO-CONTROLLER-SIM-GATE-01 adds explicit rbpodo controller-simulation motion gates. Servo J commands in `operation_mode: simulation` now require YAML opt-in, normal real-controller/motion env gates, `RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1`, and same-run pgmode confirmation; diagnostics-suspect startup override is narrower and separately gated by `RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1`.
38. RBPODO-CIRCLE-BENCH-01 adds a gated rbpodo controller-simulation circle benchmark runner. It requires real-controller/motion env gates, `RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1`, explicit controller confirmation, pgmode simulation confirmation, and `operation_mode: simulation`; default scoring uses `tcp_ref_stand` and physical `q_actual` drift is reported as a pgmode warning rather than motion success. No physical real circle acceptance is claimed.
39. RBPODO-CIRCLE-ABLATION-01 adds a rbpodo-only controller-simulation circle ablation matrix runner. It aggregates stable 15 cm / 16 s and GENE-style 15 cm / 4 s factors across controller, gain, ACK, and command-rate settings, but still requires pgmode simulation confirmation plus real-controller/motion env gates and stops on safety preflight or child benchmark errors.
40. RBPODO-CIRCLE-REPORT-01 adds reporting and decision policy for rbpodo controller-simulation circle evidence. Reports separate `rb_simulator`, `rbpodo_controller_simulation`, and future real physical categories, require explicit tracking-source and physical-motion fields, and keep GENE-style 15 cm / 4 s evidence marked stress/not-real-ready.
41. RBPODO-CONTROLLER-SIM-CARTESIAN-00 registers the controller-simulation Cartesian follow-up gates for the narrow rbpodo pgmode-simulation carve-out. This is gate registration only; controller-simulation benchmarks remain opt-in, `RB_ALLOW_REAL_CARTESIAN` is not weakened, and physical real Cartesian motion remains blocked.
42. RBPODO-CONTROLLER-SIM-CARTESIAN-01 adds the narrow server-side streaming Cartesian carve-out for rbpodo controller pgmode simulation. It requires `cartesian_control.allow_in_controller_simulation: true`, `operation_mode: simulation`, the existing controller-simulation motion gates, `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1`, and `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1`; physical real streaming Cartesian remains blocked and telemetry now publishes the gate decision.
43. RBPODO-CIRCLE-CONFIG-FIX-01 updates the circle controller-simulation templates to opt into `cartesian_control.allow_in_controller_simulation: true` while keeping `cartesian_control.allow_in_real: false`. Use `tools/create_rbpodo_circle_local_configs.sh` to create or refresh operator-local circle configs before running; stale local copies should be regenerated with `--force` only after reviewing local edits.
44. RBPODO-CIRCLE-BENCH-FIX-01 classifies pgmode circle runs with `cartesian_solve.status=unavailable` and singular/static reference paths as blocked server-side Cartesian rejection, not tracking evidence. The benchmark now reports Cartesian unavailable reason counts, `ArmedHold` counts, q_ref/tcp_ref movement flags, and hints for checking `allow_in_controller_simulation`, `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN`, `operation_mode: simulation`, and pgmode confirmation.
45. RBPODO-CIRCLE-DOC-01 documents the operator flow for rbpodo controller-simulation circle evidence: build the rbpodo server, apply realtime capabilities when needed, create local configs from templates, set or verify Rainbow `pgmode` simulation, include `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1`, score `tcp_ref_stand`, and interpret Servo J ACKs plus singular fits as blocked Cartesian evidence when `cartesian_solve.status=unavailable`.
46. RBPODO-LIVE-VIZ-00 registers live visualization and benchmark overlay follow-up gates. This is gate registration only: GUI visualization work must remain command-free, state fanout should be server-side, benchmark desired-circle overlays should use a separate UDP overlay, and controller-simulation benchmark runs remain opt-in behind existing safety gates.
47. STATE-FANOUT-01 standardizes server-side UDP state fanout through `network.state_pub_endpoints`. Circle controller-simulation templates now publish identical state JSON to separate benchmark-recorder and `rb_gui` ports; the single `state_pub_endpoint` remains a legacy one-consumer field, and benchmark tee/rebroadcast is not the primary live state path.
48. BENCH-OVERLAY-UDP-01 adds a telemetry-only UDP overlay from the rbpodo circle benchmark for desired circle geometry and live metrics. The overlay writes `overlay_stream.jsonl`, publishes at a bounded rate, and never replaces the server state stream or carries robot commands.
49. RBPODO-CIRCLE-LIVE-RUNBOOK-01 documents the live `rb_gui` workflow for rbpodo controller-simulation circle benchmarks: server state fanout feeds benchmark and GUI ports, benchmark overlay feeds the GUI overlay port, `tcp_ref_stand` is the pgmode tracking source, `physical_motion_expected=false` remains visible, and `policy_runner` is separate from this visualization path.
50. POLICY-DATASET-SCHEMA-01 defines additive policy/teleop dataset metadata for simulator, rbpodo controller `pgmode` simulation, and future physical real demonstrations. The policy recorder preserves optional actual/reference TCP, q_ref/q_target, ACK, diagnostics, command source, and SpaceMouse fields without changing command paths or weakening deadman, lease, or real-motion gates.
51. RBPODO-CIRCLE-UX-01 adds convenience wrappers for rbpodo controller-simulation circle bring-up, GUI launch, and benchmark execution. The wrappers validate local configs, require explicit confirmation for controller access, refuse physical-real Cartesian settings, keep `RB_ALLOW_*` env gates opt-in, and do not send GUI commands.
52. RBPODO-CIRCLE-METRICS-FIX-01 makes rbpodo circle summaries distinguish physical `q_actual`/`tcp_actual_stand`, commanded `q_sent`, optional published `q_ref_deg`, and controller-reference `tcp_ref_stand`. Missing `q_ref_deg` is now reported as `q_ref_moved: null` with a reason instead of a false static-reference claim; `tcp_ref_moved` is the primary controller-simulation motion evidence, and high integrator divergence with stationary `q_actual` is warned as a reference-state-source diagnostic.
53. RBPODO-CONTROLLER-SIM-TRACKING-01 switches the tracking-error guard to controller-reference joints only for rbpodo `operation_mode: simulation` when the controller-simulation motion gate is open and the config requests `controller_simulation_tracking_error_source: reference`. Physical-real tracking remains `q_actual`-based, controller-simulation physical `q_actual` drift is separately latched by policy, and rbpodo circle summaries now classify latched server faults as `faulted` instead of completed performance evidence.
54. RBPODO-CONTROLLER-SIM-STARTUP-REF-01 seeds previous sent targets and initial fault-hold targets from controller reference joints at rbpodo pgmode-simulation startup when reference tracking is explicitly active. Physical-real and rb_simulator startup remain `q_actual`-based, physical-motion baselines still use `q_actual`, and invalid startup references fail closed with `controller_simulation_startup_reference_unavailable`.
55. RBPODO-CIRCLE-FAULT-STARTUP-DIAG-01 makes the rbpodo circle benchmark distinguish no state packets from fault-latched startup packets. Startup fault summaries now report `result: startup_fault`, the latest state excerpt, latched fault reason, safety tracking fields, q_actual/q_target error summaries, and reference-tracking hints instead of labeling the condition as a state stream timeout.
56. RBPODO-CIRCLE-ABLATION-OVERRIDES-01 lets the rbpodo circle ablation runner generate per-experiment `resolved_server_config.yaml` files for allowlisted temporary factors such as bounded state publish rate, speed bar, servo rate/t1, command timeout, twist/linear limits, Cartesian path gains, and twist deadband. Source/local configs are not edited, physical-real Cartesian fields remain rejected, generated configs are audited in each artifact directory, and ablation summaries expose feedback limits, saturation, orientation drift, fit-center error, physical-motion detection, and fault-latch state.
57. RBPODO-CIRCLE-STAGE2-MATRICES-01 adds controller-simulation-only stage-2 rbpodo circle matrices for GENE 15 cm / 4 s gain splitting, publish-rate/speed-bar sweeps, and a 15 cm / 8 s middle-speed bridge. The 4 s rows remain stress evidence, zero orientation feedback gain is accepted for gain separation, and no matrix result is promoted to physical real Cartesian readiness.
58. RBPODO-CIRCLE-TUNING-REPORT-01 makes the rbpodo circle report structure-aware for tuning: open-loop baselines, closed-loop candidates, saturation-limited rows, orientation-unstable rows, center-drift-limited rows, state-publish/speed-bar mismatches, and stress-only rows are classified separately with scores and stage-summary picks. The report explicitly warns that open-loop radius can hide center drift, Kp=2 was aggressive in the previous stage, Kp_pos/Kp_ori must be tuned separately, and controller-simulation evidence remains not physical-real readiness.
59. MEASURE-P0-GATE-00 registers P0 measurement reliability gates for rbpodo state parity, raw 5001 capture, timestamp alignment, circle error decomposition, and reporting. These gates prioritize `diagnostics_suspect`, `tcp_ref_stand`, `q_ref`, lower bound interpretation, and measurement-artifact separation; controller access remains opt-in and no physical motion is authorized.
60. RBPODO-MEASURE-STATE-PARITY-01 publishes direct rbpodo reference-state metadata (`q_ref_deg`, `q_ref_source`, validity flags, SDK/decode policy) and adds read-only Python-vs-C++ state parity artifacts. Passing parity means both decoders agree on sampled fields; suspicious diagnostics remain motion-blocking and physical real motion is still unauthorized.
61. RBPODO-500HZ-CIRCLE-MATRIX-01 adds staged rbpodo controller-simulation circle matrices for the supported 500 Hz path: safe 5 cm / 10 s, 15 cm / 16 s baseline, 15 cm / 8 s bridge, and 15 cm / 4 s stress. The matrices keep `operation_mode: simulation`, `cartesian_control.allow_in_real: false`, telemetry publication decoupled from command rate, and `servo_t1_sec: 0.002`; 4 s remains stress evidence and no row is physical-real ready.
61. RBPODO-MEASURE-RAW-DATA-01 adds read-only Rainbow data-port 5001 raw capture and fixture summary tooling for investigating `diagnostics_suspect` field-layout questions. It requires real-controller confirmation and `RB_ALLOW_REAL_ROBOT=1` for known controller IPs, avoids command-port traffic and binary layout guessing, and keeps raw payload artifacts under `artifacts/` unless sanitized.
62. RBPODO-MEASURE-TIMESTAMP-01 adds offline timestamp alignment and jitter decomposition for rbpodo controller-simulation circle artifacts. Benchmarks now emit `alignment_summary.json` / `alignment_report.md`, classify timing as clean, command-generation-limited, ACK-spike-limited, state-publish-limited, or jitter-limited, and keep non-clean timing evidence from being treated as reliable tracking-error tails. The audit is read-only and does not send commands or change controller behavior.
63. RBPODO-CIRCLE-ERROR-DECOMP-01 adds offline circle tracking error decomposition for rbpodo controller-simulation artifacts. Benchmarks now emit `error_decomposition.json` / `cycle_error_decomposition.csv`, expose median/tail, center-removed, phase-aligned, orientation-equivalent, saturation, and timing-aware classifications in summaries/reports, and keep the work diagnostic-only with no control behavior or real-motion gate changes.
64. RBPODO-MEASURE-RELIABILITY-REPORT-01 adds measurement reliability grading for rbpodo pgmode circle artifacts. Reports now label unreliable, suspect, and controller-reference-valid lower-bound evidence; write `measurement_reliability_report.md` / `.csv` / `.json` for ablations; and keep diagnostics_suspect, q_ref validation, timing, saturation, orientation, and physical-real blockers visible before tuning recommendations. It does not enable physical real motion.
65. GATE-RBPODO-500-P0-P1-00 registers the next rbpodo controller-simulation gate tracks: P0 measurement reliability repair, 500 Hz pgmode-simulation acceptance, and P1 tuning/error-factor separation. The gates run compile/help/unit/schema checks by default, keep controller probes behind explicit `CODEX_RUN_RBPODO_*` opt-ins plus tool confirmation flags, keep 500 Hz evidence scoped to rbpodo pgmode simulation, and do not enable physical real motion.
66. P0-PARITY-REPAIR-01 adds a dedicated read-only rbpodo measurement config and hardens Python-vs-C++ state parity classification. The parity checker refuses supplied configs unless `servo.send_servo_commands: false`, waits for any state packet including diagnostics-suspect or fault-latched packets, separates `failed_server_exit`, `failed_transport`, and `failed_parity_mismatch`, and records `diagnostics_suspect_unresolved` rather than treating consistent suspect diagnostics as transport failure. It remains read-only and does not set pgmode, reset faults, or send Servo J.
67. P0-DIAGNOSTICS-ROOTCAUSE-01 adds an offline rbpodo diagnostics root-cause report that combines state dump, Python/C++ parity, and raw data-port capture evidence. It classifies likely causes as SDK/firmware layout mismatch, unknown field semantics, controller-reported real fault, Python/C++ decode mismatch, payload instability, or insufficient evidence; records SDK/module and C++ state-stream hints when available; and keeps `diagnostics_suspect_unresolved`, `stop_resetFault_unverified`, and `physical_reference_to_actual_error_unmeasured` as physical-real blockers. It does not suppress `diagnostics_suspect`, does not send commands, and does not enable physical real motion.
68. P0-RAW-PAYLOAD-FIXTURE-02 turns Rainbow data-port raw capture into repeatable fixture evidence. Captures now store compact per-sample length/hash/prefix/suffix metadata with first/last payload binaries by default, require `--save-each-sample` for per-sample raw binaries, compute changed-byte offset histograms and q_ref/payload transition counts, and add an offline fixture report that recommends collecting motion/no-op fixtures and comparing firmware SDK docs before inferring any binary layout. It remains read-only and does not touch command-port, pgmode, control, backend, motion, or safety-gate behavior.
69. P0-MEASUREMENT-GATING-01 makes measurement reliability a required report field for rbpodo controller-simulation summaries and ablations. Diagnostics-suspect runs are graded `suspect`, faulted/physical-motion/Cartesian-unavailable runs are `unreliable`, failed state parity caps reliability at `suspect`, `tcp_ref_stand` is labeled as controller-reference lower-bound evidence, and tuning results are separated from measurement reliability and physical-readiness blockers. This is reporting-only and does not change control behavior, rbpodo backend behavior, or real-motion gates.
70. RBPODO-500HZ-CONFIG-01 adds rbpodo controller-simulation 500 Hz circle templates for 5 cm / 10 s, 15 cm / 16 s, 15 cm / 8 s, and 15 cm / 4 s plus a local-config helper flag. Stage 0 evidence is limited to a single-arm pgmode-simulation no-op Servo J acceptance run with 5000/5000 sends over 10 s, loop p99 about 2.006 ms, and max send about 501 us. The templates stay ACK-on, `operation_mode: simulation`, `allow_in_real: false`, controller-reference based, and do not enable physical real motion.
71. RBPODO-500HZ-ACCEPT-01 adds a rb_servo_server-level 500 Hz Servo J no-op acceptance runner for rbpodo controller `pgmode` simulation. It requires real-controller/motion env gates, `RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1`, same-run pgmode confirmation, `operation_mode: simulation`, ACK-on Servo J, and a constant target captured from `q_ref_deg` / `q_target_deg`; it rejects physical real operation mode and does not approve physical robot motion.
72. RBPODO-500HZ-REPORT-01 is superseded by supported-scope 500 Hz rbpodo evidence reporting. Reports expose send success, controller-acceptance observation, send duration, jitter, deadline, command interval, state publication, q_ref, tracking, saturation, and measurement-reliability fields while keeping no-op, `tcp_ref_stand`, diagnostics-suspect, dual-arm, and physical-readiness caveats explicit. It does not change control behavior, benchmark execution, or real-motion gates.
73. P1-CIRCLE-FACTOR-MATRIX-01 adds rbpodo controller-simulation P1 factor matrices for gain splitting, state publish rate plus speed_bar, twist caps, Servo J `servo_t2_sec` plus `servo_alpha`, and future phase advance. The ablation runner now allowlists bounded `servo_t2_sec`/`servo_alpha` temporary overrides and carries `phase_advance_sec` as a dry-run matrix field for P1-DEADTIME-PHASE-ADVANCE-01. All rows remain GENE-style 15 cm / 4 s stress evidence, controller-reference lower bounds, and not IL-ready or physical-real Cartesian readiness.
74. P1-ORIENTATION-DIAG-01 adds orientation feedback diagnostics for rbpodo controller-simulation circle evidence. Benchmark summaries now expose desired/measured orientation source, sampled orientation error vectors, separate angular feedback/applied norms, angular saturation count/ratio, and orientation-equivalent tool-offset error in millimeters. The C++ Cartesian servo tests lock the current orientation sign and stand/local frame convention; no controller sign change or real-mode gate change is implied.
75. P1-DEADTIME-PHASE-ADVANCE-01 adds explicit benchmark-side circle command phase advance for rbpodo controller-simulation tuning. `--phase-advance-sec` defaults to `0.0`, is rejected above 25% of the circle period, advances only benchmark-generated command samples, and keeps tracking metrics tied to measurement-time desired poses. Summaries and ablation reports separate `commanded_phase_advance_ms` from measured `estimated_latency_ms` and classify matched phase-advance rows by phase-aligned RMS and saturation changes. Physical real commands, rbpodo backend logic, and real-motion gates are untouched.
76. P1-SERVO-PARAM-SWEEP-01 hardens rbpodo controller-simulation Servo J `servo_t2_sec`/`servo_alpha` ablation. The runner validates both override values and resolved per-arm config values with `0.02 < servo_t2_sec < 0.2` and `0 < servo_alpha < 1`, writes only per-experiment `resolved_server_config.yaml` files, exposes left/right parameter columns in summaries, and expands the P1 matrix across `pub50/speed0.1` plus `pub100/speed0.2`. Alpha semantics remain experimental; no rbpodo backend send semantics, physical real gates, or controller logic changed.
77. P1-SERVER-SIDE-CIRCLE-TRACK-SKELETON-01 adds the `TcpCircleTrack` command schema and fail-closed server skeleton for future tick-local circle feedback. The parser accepts bounded trajectory/feedback parameters, state telemetry exposes the server-side circle gate, and the servo loop rejects disabled commands as `tcp_circle_track_disabled`, enabled-but-incomplete commands as `tcp_circle_track_not_implemented`, and physical real operation as `tcp_circle_track_physical_real_blocked`. The config defaults to disabled, rbpodo backend behavior is unchanged, and no physical real Cartesian motion is enabled.
78. RBPODO-500HZ-ACK-SWEEP-01 separates 500 Hz no-op acceptance failures into preflight, startup, warmup, and measurement phases. The runner writes artifact-local resolved configs for command-timeout overrides and no-op Cartesian disablement, supports ACK timeout sweeps plus optional warmup, and reports first send failure, ACK timeout counts by arm, p99 send duration by arm, and deadline misses by arm. ACK timeout remains a failure, ACK-off is not promoted, circle benchmark behavior is unchanged, and physical real motion gates remain untouched.
79. RBPODO-ASYNC-GATE-00 registers future async ACK-supervised rbpodo 500 Hz controller-simulation gate names. The gates run compile/help/unit/schema checks by default, keep controller runs behind `CODEX_RUN_RBPODO_ASYNC_500HZ=1` plus explicit tool confirmations and async ACK-supervision env gates, and do not implement async mode, enable physical real motion, or weaken existing real/cartesian safety gates.
80. RBPODO-ASYNC-CONTRACT-01 defines the async ACK-supervised rbpodo controller-simulation config and state schema without changing transport or servo-loop behavior. `sdk_ack_worker` preserves `controller_ack_observed` semantics in a worker lane, `socket_send_supervised` is explicitly `socket_send_only` with q_ref/tcp_ref watchdog supervision, `operation_mode: real` is refused, and physical real motion remains blocked.
81. RBPODO-ASYNC-SDK-PROBE-01 adds a controller `pgmode` simulation-only Python rbpodo SDK capability probe for async ACK assumptions. It records ACK-on duration, ACK-off socket-send plus q_ref supervision evidence, separate-object concurrent read/send behavior, and SDK method exposure; it requires real-controller/motion env gates, pgmode confirmation, explicit user confirmation, no-op target checks against current `q_ref`/`q_actual`, and ACK-off env approval. It does not touch `rb_servo_server`, change rbpodo backend behavior, prove dual-arm 500 Hz acceptance, or authorize physical real motion.
82. RBPODO-ASYNC-WORKER-01 implements the controller-simulation-only rbpodo async worker path. The servo loop now enqueues per-arm latest-wins Servo J requests without waiting for ACK, worker threads record ACK/socket-send supervision telemetry, async faults latch the loop, and config gates still require rbpodo `operation_mode: simulation` plus explicit real/controller-sim/async env approval. Physical real operation remains refused.
83. RBPODO-ASYNC-REFERENCE-SUPERVISOR-01 adds controller `pgmode` simulation-only q_ref/tcp_ref reference supervision for async rbpodo streaming. In `socket_send_supervised`, socket send evidence is not enough: invalid q_ref faults, stale q_ref/tcp_ref update age faults, and target divergence follows the configured warning/fault policy. In `sdk_ack_worker`, ACK remains primary while reference supervision contributes warning telemetry except invalid q_ref. Physical q_actual motion detection remains a separate safety path.
84. RBPODO-ASYNC-500HZ-ACCEPT-01 extends the 500 Hz acceptance runner to validate async ACK-supervised controller-simulation modes. `sdk_ack_worker` now requires worker ACK telemetry, bounded overwrite/drop ratios, and nonblocking servo-loop evidence; `socket_send_supervised` requires explicit socket-send-only acknowledgement plus q_ref/tcp_ref watchdog health, q_ref update rate, target-error checks, and no supervision faults. The new safe 5 cm / 10 s async circle matrix references operator-local async configs, keeps physical motion expected false, and does not change C++ control behavior or real-mode gates.
85. RBPODO-ASYNC-CIRCLE-MATRIX-01 adds staged async 500 Hz rbpodo controller-simulation circle matrices for safe 5 cm / 10 s, 15 cm / 16 s, and 15 cm / 8 s plus 15 cm / 4 s stress. The matrices compare 500 Hz synchronous ACK-on where applicable, 500 Hz `socket_send_supervised` with reference supervision, and disabled `sdk_ack_worker` candidates until no-op evidence supports them. Socket-send rows remain `socket_send_only` evidence with reliability caveats, keep `operation_mode: simulation` and `allow_in_real: false`, and do not enable physical real motion or change control behavior.
86. RBPODO-ASYNC-REPORT-01 updates rbpodo 500 Hz reporting so synchronous ACK-on, async `sdk_ack_worker`, `socket_send_supervised`, and q_ref-supervised evidence are separated. Reports now carry async command counters, reference-supervision state, q_ref update/error fields, a four-lane comparative table, and conservative classifications for supervised pass, socket-send-only promising evidence, ACK-on blocking limits, reference-watchdog failure, unstable rows, and insufficient evidence. Socket-send-only rows are never reported as per-command controller ACK; this is reporting-only and does not alter control behavior, benchmark execution, physical real gates, or real-motion readiness.
87. RBPODO-ASYNC-RUNBOOK-01 documents the async ACK-supervised 500 Hz rbpodo controller-simulation workflow. The runbooks now separate synchronous ACK-on, `sdk_ack_worker`, and `socket_send_supervised`; explain the 2 ms ACK-wait fragility; require q_ref/tcp_ref watchdog supervision for socket-send-only rows; keep `operation_mode: simulation`, `physical_motion_expected=false`, and no physical real approval explicit; and preserve the SDK probe -> no-op -> safe 5 cm / 10 s -> 15 cm / 16 s -> 15 cm / 8 s -> 15 cm / 4 s stress acceptance order. This is documentation-only and does not change source, configs, gates, benchmark behavior, or real-motion readiness.
88. ACKON500-RESULT-CONTRACT-01 separates circle execution, safety, generic benchmark thresholds, official ACKON500 goal status, and diagnostic warnings in summaries and reports. New circle artifacts mirror `run_result.status` in legacy `result`, keep max-orientation spikes visible under generic thresholds/diagnostics, preserve the official p95-orientation ACKON500 criterion from `GOAL.md`, and continue rejecting socket-send-only rows for official ACK-ON goal passes. This is reporting-only and does not change servo control, rbpodo backend behavior, safety gates, or physical real gates.
89. ACKON500-RATE-ACCOUNTING-01 separates high-level UDP command counts from low-level async ServoJ worker totals and official tracking-window rate evidence. ACKON500 reports now expose `udp_command_count`, `server_servo_tick_count`, `async_commands_*_total`, `official_tracking_window_sec`, `measured_worker_window_sec`, `official_servo_rate_hz`, `goal_window_commands_sent`, `goal_window_commands_acked`, `ack_coverage_ratio`, `effective_goal_command_rate_hz`, and worker lifetime diagnostics; official pass/fail no longer uses the ambiguous total-counter `effective_command_rate_hz`. C++ changes are telemetry-only worker timestamps/phase publication and do not change control behavior, ACK semantics, physical real gates, or real-motion readiness.
90. BENCHMARK-LANE-CANONICALIZE-01 adds canonical benchmark lane metadata to circle summaries, ablation rows, comparison reports, 500 Hz reports, and ACKON500 goal reports. Reports now group by `benchmark_lane` and expose `control_loop_location`, `trajectory_generation_location`, `feedback_loop_location`, `low_level_send_mode`, `acceptance_semantics`, `tracking_source`, and `physical_motion_expected`. The official ACKON500 pass lane is `rbpodo_server_side_circle_ackon500_sdk_worker`; `socket_send_supervised` maps to `rbpodo_server_side_circle_500hz_socket_send_supervised` and cannot pass the official goal. `TcpCircleMove` remains the implemented server-side circle benchmark command, `TcpCircleTrack` remains the reserved skeleton, and both are grouped as `command_family: server_side_circle`. This is metadata/reporting/state-overlay only and does not change control behavior, physical real gates, or real-motion readiness.
91. ACKON500-BEST-PROFILE-PROMOTION-01 promotes the achieved 500 Hz ACKON500 controller-simulation result to a named best profile and runner default. The profile keeps `operation_mode: simulation`, `sdk_ack_worker`, ACK-observed semantics, `disable_waiting_ack: false`, `physical_motion_expected=false`, and `cartesian_control.allow_in_real: false`; it does not change control code, physical real gates, diagnostics-suspect policy, or physical-readiness status.
92. ACKON500-REPEATABILITY-VALIDATION-01 adds the official repeatability matrix and deterministic reporting for the named ACKON500 best profile. The matrix runs `best_left_run01..03` and `best_right_run01..03` with the same server-side circle, 500 Hz `sdk_ack_worker`, ACK-observed, `t2=0.08`, `alpha=0.8`, `speed_bar=0.2`, path-gain, phase-advance, and `tcp_ref_stand` semantics. Reports write `repeatability_summary.csv`, `repeatability_summary.json`, and `repeatability_report.md`; group rows by arm, benchmark lane, acceptance semantics, and tracking source; require each required row to pass the official ACKON500 contract; require left and right per-arm aggregates to pass before global `repeatable_pass`; and keep controller-reference, diagnostics-suspect, physical-readiness-blocked, and no-physical-real caveats explicit. This is config/reporting/wrapper-only and does not change control behavior, physical real gates, or real-motion readiness.
93. RBPODO-JOINT-RANGE-POLICY-01 makes raw rbpodo controller joint degrees the supported state/command/log source of truth and updates tracked real rbpodo templates to explicit `[-360, 360]` per-joint safety arrays. `[-180, 180]` remains only for intentional violation tests or site-owned conservative overrides, motion target wrapping stays refused, and kinematics-enabled rbpodo configs warn when the safety range differs from the `rb3_730e.urdf` IK model limits. This does not enable physical real motion.

## Reviewer Checklist

When reviewing a change, check:

- Did the change alter real-mode gates?
- Did it enable real robot or real Cartesian motion?
- Did it reintroduce bool-only backend errors?
- Did it weaken command-source lease, deadman, stale-state, or fault behavior?
- Did it confuse PTP, Linear, Twist, and Delta semantics?
- Did it update state telemetry when changing controller behavior?
- Did it update tests and acceptance scripts when changing behavior?
- Did it update docs when changing operator-visible behavior?

## Current Recommendation

The simulator acceptance baseline holds and the project has proceeded onto the
physical robot through the conservative ladder: read-only diagnostics parity →
tiny motion → slow physical Cartesian circle, all under operator supervision with
an E-stop. Continue up the ladder one stage at a time (slow → 15 cm / 16 s →
faster) only with explicit approval per stage, keep `rb_servo_server` as the sole
real-motion safety layer, and do not promote force control, grippers, measured
calibration, or full `real_policy` rollout until each is separately validated.
Any regression in simulator acceptance still blocks physical work.
