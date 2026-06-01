# REVIEW.md

## Review Baseline

This review reflects the repository after the simulator-first Cartesian hardening work. The current milestone is not real robot motion. The current milestone is repeated simulator validation of all motion primitives and operator interfaces before any real RB3-730 bring-up.

This root file is the current review source of truth. `docs/current_review.md`
is only a short redirect here so review status does not drift across copies.

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
- simulator-only Cartesian acceptance scripts
- mock camera and camera acceptance runbooks
- mandatory Eigen3/Pinocchio-backed Cartesian math in `rb_servo_server`

### Not Production-Ready

- real RB3-730 motion
- real Cartesian/TCP motion
- force/admittance/impedance control
- gripper integration
- measured camera/robot calibration
- real three-camera plus policy plus robot closed-loop behavior
- real `servo.io_model: worker` acceptance

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

These gates are necessary but not sufficient. Config and real-hardware acceptance must also explicitly allow the operation.

## Motion Primitive Review

### `TcpPoseTarget`

Status: simulator-supported.

Meaning: point-to-point Cartesian final-pose target. This is MoveJ-like at the TCP level: final pose is targeted, but the intermediate TCP path is not guaranteed to be linear.

Review requirements:

- quaternion target should be preserved end-to-end
- final position/orientation error must be visible in state telemetry
- GUI should describe it as PTP, not MoveL

### `TcpLinearMove`

Status: simulator acceptance candidate.

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
- Best GENE-style stress result: pending; no reviewed `gene_15cm_4s`
  rbpodo controller-simulation artifact is recorded here.
- Real physical circle benchmark: not run and not approved.

Before any physical circle discussion, record a rbpodo controller-simulation
report with `physical_motion_expected=false`, `physical_motion_detected=false`,
`tracking_source=tcp_ref_stand`, pgmode simulation confirmation, ACK semantics,
and q_ref/q_actual update rates. Controller-simulation results may guide future
low-speed parameter selection, but cannot be copied directly to real motion.

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
11. Run BENCH-CIRCLE-01 simulator benchmark and record artifacts before real robot Cartesian testing. Latest CART-SERVO-01 conservative gate evidence: `artifacts/circle_tracking/bench_circle_01` safe 5 cm / 10 s threshold pass; `artifacts/circle_tracking/left_twist_stand_15cm_16s_after/summary.json` is the current 15 cm / 16 s simulator baseline candidate but is `completed`, not threshold `pass`. CART-SERVO-02 changes benchmark runs without thresholds from `pass` to `completed` so poor tracking cannot be mislabeled as a performance pass. CART-SERVO-03 adds the four-profile regression sequence and `scripts/compare_circle_benchmarks.py` for before/after summaries. GATE-BENCH-00 registers follow-on gate names for circle ablation, tuning, feedback comparison, server-side circle experiments, and reporting. BENCH-ABLATION-01 adds the matrix runner for factor-separated simulator-only circle tracking experiments; run and archive ablation artifacts before promoting any stress settings. CART-TUNE-02 adds named simulator circle profiles for baseline, stress, and conservative real-candidate separation; none is real-ready. BENCH-CIRCLE-FEEDBACK-01 adds simulator-only closed-loop benchmark modes to compare open-loop twist drift against command-source feedback compensation. BENCH-REPORT-01 defines baseline promotion rules, stress interpretation, and the real-candidate parameter policy.
12. GATE-RBSCRIPT-00 registers the experimental `RBSCRIPT-*` task names for future rbscript_tcp backend comparison work. This is gate registration only; it does not enable rbscript_tcp real motion or weaken the existing real robot gates.
13. RBSCRIPT-TCP-01 adds an experimental raw Rainbow script TCP backend skeleton for comparison only. It requires `RB_ALLOW_RBSCRIPT_TCP=1` for real connection and `RB_ALLOW_RBSCRIPT_TCP_MOTION=1` for servo sends, keeps `send_servo_commands: false` in the tracked example, and is not accepted for production motion.
14. RBSCRIPT-TCP-02 adds a bounded `reqdata` data-port read path and fixture-backed state parser for `rbscript_tcp`. Real Rainbow binary payload parsing remains an open validation item; unknown data-port responses fail closed and must not be treated as production-ready state acquisition.
15. RBSCRIPT-ABLATION-01 adds no-motion backend ablation tooling for rbpodo vs experimental rbscript_tcp connection/read/ACK timing. It requires explicit real-controller confirmation and env gates for read-only real access; motion probes remain out of scope.
16. RBSCRIPT-RATE-PROBE-01 adds an explicit Rainbow rate sweep for ACK/read-state behavior at 50-200 Hz style rates. It remains no-motion by default, requires explicit real-controller confirmation and env gates, refuses motion-looking no-motion commands, and does not prove high-rate motion readiness.
17. RBSCRIPT-DOC-01 documents the rbpodo vs experimental rbscript_tcp comparison plan. `rbscript_tcp` must pass no-motion connect, read-only state acquisition, and no-motion ACK/rate evidence before any future real motion task; raw TCP is not a replacement for rbpodo yet, and `rt_script` remains future work.
18. GATE-RBPODO-00 registers rbpodo servo parameter, ACK semantics, acceptance, and docs task names. This is gate registration only; it does not change rbpodo behavior or enable real robot motion.
19. RBPODO-SERVO-PARAM-01 makes rbpodo `move_servo_j` parameters explicit as `servo_t1_sec`, `servo_t2_sec`, `servo_gain`, and `servo_alpha`; deprecated aliases warn, duplicate old/new keys fail, and real motion configs must align `servo_t1_sec` with `servo.rate_hz` unless explicitly accepted.
20. RBPODO-ACK-01 makes rbpodo ACK-on/off semantics observable. ACK-off remains non-default, requires `RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1` for real motion, and must be treated as socket-send evidence rather than controller-acceptance evidence.
21. RBPODO-ACCEPT-01 adds supervised rbpodo 100 Hz / 200 Hz ACK-on/off acceptance tooling and a runbook. Real execution remains pending; default mode is read-only, ACK-off requires explicit acknowledgement, and tiny real motion is reserved for a future approval task.
22. RBPODO-DOC-01 documents rbpodo Servo J parameter mapping, 100/200 Hz rate matching, ACK-on/off interpretation, and the staged real acceptance sequence. The pending real items are: rbpodo 100/200 Hz ACK-on acceptance, 200 Hz ACK-off acceptance, and a separately approved tiny real joint motion task.
23. RBPODO-READONLY-DIAG-01 allows explicitly configured read-only rbpodo diagnostic startup to publish unsafe/faulted controller state after valid state acquisition. This is not motion readiness; `send_servo_commands=true` startup remains strict and real motion gates remain unchanged.
24. RBPODO-STATUS-FIELDS-01 preserves raw rbpodo status fields in state JSON and treats suspicious status layouts as motion-blocking diagnostics. Read-only bring-up may publish the raw fields after valid joint acquisition, but suspicious diagnostics remain unsafe and real motion gates are unchanged.
25. RBPODO-JOINT-WRAP-01 adds explicit startup-only joint wrapping diagnostics for rbpodo bring-up. Raw controller joint values remain published, motion target wrapping is refused, and real motion gates remain unchanged.
26. RBPODO-BRINGUP-TOOLS-01 adds a read-only rbpodo state dump tool and improves acceptance startup timeout diagnostics with server return code, log tail, and likely causes. It does not send motion or modify controller state.
27. GATE-BACKEND-COMPARE-00 registers backend comparison gates for an apples-to-apples `rbpodo` vs `rbscript_tcp` track covering `read_state`, `servo_j_noop`, ACK-on, ACK-off, data-port, matrix, and report follow-ups. This is gate registration only; real-controller probes remain opt-in and real motion gates remain unchanged.
28. BACKEND-COMPARE-CONFIG-01 standardizes read-only comparison templates for `rbpodo` and experimental `rbscript_tcp`, removes stale tracked local YAML samples, and keeps controller-simulation no-op templates separate under `configs/backend_compare/*sim_noop.yaml`. The comparison templates remain non-motion by default.
29. RBSCRIPT-PERSISTENT-PROBE-01 adds opt-in persistent-socket mode and response-framing metrics to the Python rbscript no-motion probes. It keeps no-motion command validation and explicit real-controller gates intact, and does not change C++ backend or real motion behavior.
30. RBSCRIPT-SERVO-NOOP-01 adds a supervised `rbscript_tcp` controller-simulation Servo J no-op harness. It requires pgmode simulation, explicit q-current source, real-controller confirmation, `RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`, `RB_ALLOW_RBSCRIPT_TCP`, and `RB_ALLOW_RBSCRIPT_TCP_MOTION`; ACK-off remains socket-send-only evidence and tiny real motion remains unimplemented.
31. RBSCRIPT-DATA-PORT-01 keeps rbscript TCP data-port state fail-closed: the JSON fixture parser remains available for tests, real Rainbow 5001 payloads are classified as `rbscript_tcp_real_data_port_unsupported`, comparison output marks unsupported rbscript `read_state` as not comparable to `rbpodo`, and raw 5001 capture is read-only evidence for a future verified parser.
32. BACKEND-COMPARE-MATRIX-01 adds a serial matrix runner that invokes existing gated rbpodo/rbscript comparison scripts, writes per-experiment status plus aggregate CSV/JSON/Markdown reports, treats capability mismatches as unsupported instead of failed performance, and keeps ServoJ no-op controller simulation behind an explicit matrix flag and child-script safety gates.
33. BACKEND-COMPARE-REPORT-01 adds decision-oriented backend comparison reporting that classifies evidence as measured/comparable, measured/not-comparable, unsupported, or not-yet-run. Current interpretation remains: `rbpodo` is the primary real-backend candidate when read-state/read-only diagnostic evidence passes the documented rules; `rbscript_tcp` remains experimental while real data-port read_state and ServoJ no-op apples-to-apples evidence are incomplete.
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
61. RBPODO-500HZ-CIRCLE-MATRIX-01 adds staged rbpodo controller-simulation circle matrices for 100 Hz vs 500 Hz comparison: safe 5 cm / 10 s, 15 cm / 16 s baseline, 15 cm / 8 s bridge, and 15 cm / 4 s stress. The matrices keep `operation_mode: simulation`, `cartesian_control.allow_in_real: false`, `network.state_pub_rate_hz: 100`, and 500 Hz `servo_t1_sec: 0.002`; 4 s remains stress evidence and no 500 Hz row is real-ready.
61. RBPODO-MEASURE-RAW-DATA-01 adds read-only Rainbow data-port 5001 raw capture and fixture summary tooling for investigating `diagnostics_suspect` field-layout questions. It requires real-controller confirmation and `RB_ALLOW_REAL_ROBOT=1` for known controller IPs, avoids command-port traffic and binary layout guessing, and keeps raw payload artifacts under `artifacts/` unless sanitized.
62. RBPODO-MEASURE-TIMESTAMP-01 adds offline timestamp alignment and jitter decomposition for rbpodo controller-simulation circle artifacts. Benchmarks now emit `alignment_summary.json` / `alignment_report.md`, classify timing as clean, command-generation-limited, ACK-spike-limited, state-publish-limited, or jitter-limited, and keep non-clean timing evidence from being treated as reliable tracking-error tails. The audit is read-only and does not send commands or change controller behavior.
63. RBPODO-CIRCLE-ERROR-DECOMP-01 adds offline circle tracking error decomposition for rbpodo controller-simulation artifacts. Benchmarks now emit `error_decomposition.json` / `cycle_error_decomposition.csv`, expose median/tail, center-removed, phase-aligned, orientation-equivalent, saturation, and timing-aware classifications in summaries/reports, and keep the work diagnostic-only with no control behavior or real-motion gate changes.
64. RBPODO-MEASURE-RELIABILITY-REPORT-01 adds measurement reliability grading for rbpodo pgmode circle artifacts. Reports now label unreliable, suspect, and controller-reference-valid lower-bound evidence; write `measurement_reliability_report.md` / `.csv` / `.json` for ablations; and keep diagnostics_suspect, q_ref validation, timing, saturation, orientation, and physical-real blockers visible before tuning recommendations. It does not enable physical real motion.
65. GATE-RBPODO-500-P0-P1-00 registers the next rbpodo controller-simulation gate tracks: P0 measurement reliability repair, 500 Hz pgmode-simulation acceptance, and P1 tuning/error-factor separation. The gates run compile/help/unit/schema checks by default, keep controller probes behind explicit `CODEX_RUN_RBPODO_*` opt-ins plus tool confirmation flags, keep 500 Hz evidence scoped to rbpodo pgmode simulation, and do not enable physical real motion or rbscript_tcp.
66. P0-PARITY-REPAIR-01 adds a dedicated read-only rbpodo measurement config and hardens Python-vs-C++ state parity classification. The parity checker refuses supplied configs unless `servo.send_servo_commands: false`, waits for any state packet including diagnostics-suspect or fault-latched packets, separates `failed_server_exit`, `failed_transport`, and `failed_parity_mismatch`, and records `diagnostics_suspect_unresolved` rather than treating consistent suspect diagnostics as transport failure. It remains read-only and does not set pgmode, reset faults, or send Servo J.
67. P0-DIAGNOSTICS-ROOTCAUSE-01 adds an offline rbpodo diagnostics root-cause report that combines state dump, Python/C++ parity, and raw data-port capture evidence. It classifies likely causes as SDK/firmware layout mismatch, unknown field semantics, controller-reported real fault, Python/C++ decode mismatch, payload instability, or insufficient evidence; records SDK/module and C++ state-stream hints when available; and keeps `diagnostics_suspect_unresolved`, `stop_resetFault_unverified`, and `physical_reference_to_actual_error_unmeasured` as physical-real blockers. It does not suppress `diagnostics_suspect`, does not send commands, and does not enable physical real motion.
68. P0-RAW-PAYLOAD-FIXTURE-02 turns Rainbow data-port raw capture into repeatable fixture evidence. Captures now store compact per-sample length/hash/prefix/suffix metadata with first/last payload binaries by default, require `--save-each-sample` for per-sample raw binaries, compute changed-byte offset histograms and q_ref/payload transition counts, and add an offline fixture report that recommends collecting motion/no-op fixtures and comparing firmware SDK docs before inferring any binary layout. It remains read-only and does not touch command-port, pgmode, control, backend, motion, or safety-gate behavior.
69. P0-MEASUREMENT-GATING-01 makes measurement reliability a required report field for rbpodo controller-simulation summaries and ablations. Diagnostics-suspect runs are graded `suspect`, faulted/physical-motion/Cartesian-unavailable runs are `unreliable`, failed state parity caps reliability at `suspect`, `tcp_ref_stand` is labeled as controller-reference lower-bound evidence, and tuning results are separated from measurement reliability and physical-readiness blockers. This is reporting-only and does not change control behavior, rbpodo backend behavior, or real-motion gates.
70. RBPODO-500HZ-CONFIG-01 adds opt-in rbpodo controller-simulation 500 Hz circle templates for 5 cm / 10 s, 15 cm / 16 s, 15 cm / 8 s, and 15 cm / 4 s plus a local-config helper flag. Stage 0 evidence is limited to a single-arm pgmode-simulation no-op Servo J rate probe with 5000/5000 sends over 10 s, loop p99 about 2.006 ms, and max send about 501 us. The 500 Hz templates stay ACK-on, `operation_mode: simulation`, `allow_in_real: false`, controller-reference based, and state publication remains 100 Hz; they are not defaults and do not enable physical real motion.
71. RBPODO-500HZ-ACCEPT-01 adds a rb_servo_server-level 500 Hz Servo J no-op acceptance runner for rbpodo controller `pgmode` simulation. It requires real-controller/motion env gates, `RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1`, same-run pgmode confirmation, `operation_mode: simulation`, ACK-on Servo J, and a constant target captured from `q_ref_deg` / `q_target_deg`; it rejects physical real operation mode and does not approve physical robot motion.
72. RBPODO-500HZ-REPORT-01 adds a reporting-only 100 Hz vs 500 Hz rbpodo controller-simulation comparison for no-op acceptance, safe 5 cm / 10 s, 15 cm / 16 s, 15 cm / 8 s, and 15 cm / 4 s evidence. The report exposes send success, controller-acceptance observation, send duration, jitter, deadline, command interval, state publication, q_ref, tracking, saturation, and measurement-reliability fields; classifies 500 Hz evidence as no-op pass, circle improved, circle no-improvement, unstable, or insufficient; and keeps no-op, `tcp_ref_stand`, diagnostics-suspect, dual-arm, and default-rate caveats explicit. It does not change control behavior, benchmark execution, or real-motion gates.
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
85. RBPODO-ASYNC-CIRCLE-MATRIX-01 adds staged async 500 Hz rbpodo controller-simulation circle matrices for safe 5 cm / 10 s, 15 cm / 16 s, and 15 cm / 8 s plus 15 cm / 4 s stress. The matrices compare the tuned 100 Hz ACK-on baseline, 500 Hz synchronous ACK-on where applicable, 500 Hz `socket_send_supervised` with reference supervision, and disabled `sdk_ack_worker` candidates until no-op evidence supports them. Socket-send rows remain `socket_send_only` evidence with reliability caveats, keep `operation_mode: simulation` and `allow_in_real: false`, and do not enable physical real motion or change control behavior.

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

Stay in simulator acceptance hardening until all motion primitives pass repeated acceptance runs. Only after that should the project proceed to real robot read-only acceptance, then joint-only motion acceptance, and only later tiny Cartesian motion acceptance.
