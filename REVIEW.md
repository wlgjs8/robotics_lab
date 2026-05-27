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
