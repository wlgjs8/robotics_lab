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
