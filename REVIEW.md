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

## Current Physical Circle Status

The current Cartesian circle milestone is physical, not a benchmark. A slow
dual-arm physical Cartesian circle has been run under operator supervision
(TUNED-1, median tracking ~1.42°; `docs/runbooks/rbpodo_real_physical_circle.md`).
The faster/higher-speed circle ladder stages (15 cm / 16 s and above, P7–P9)
remain not run and not approved, and `diagnostics_suspect_unresolved` (vendor
`-2001` field semantics) is still open.

> The earlier rbpodo controller-simulation circle-tracking benchmark / ACKON500
> 500 Hz subsystem (configs, scripts, ablation tooling, and its `GOAL.md`
> snapshot) was removed 2026-06-20. Physical-real (`tcp_actual_stand`) evidence
> is the only Cartesian-circle evidence tracked here.

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
11. The rbpodo controller-simulation circle-tracking benchmark / ACKON500 500 Hz subsystem (configs, scripts, ablation/report tooling, `GOAL.md`) was removed 2026-06-20; its `tcp_ref_stand` lower-bound evidence is no longer tracked here. The Cartesian-circle milestone is now the physical slow circle (`docs/runbooks/rbpodo_real_physical_circle.md`).
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
32. Backend comparison matrices are replaced by rbpodo 500 Hz acceptance matrices.
33. Backend comparison reports are replaced by rbpodo-supported-scope reporting.
34–49. *(removed 2026-06-20)* The rbpodo controller-simulation circle-tracking
    benchmark subsystem — gate registration, circle/ablation config templates and
    runners, controller-reference TCP scoring tooling, the controller-simulation
    Cartesian carve-out env gates, the benchmark UDP overlay, and the live circle
    runbook — has been deleted. `STATE-FANOUT-01` (server-side
    `network.state_pub_endpoints` fanout) and the
    `cartesian_control.allow_in_controller_simulation` carve-out survive as
    general server features, but the circle-benchmark-specific configs, scripts,
    tools, and runbooks are gone. Controller-reference (`tcp_ref_stand`) circle
    evidence is no longer tracked; the Cartesian-circle milestone is the physical
    slow circle (`docs/runbooks/rbpodo_real_physical_circle.md`).
50. POLICY-DATASET-SCHEMA-01 defines additive policy/teleop dataset metadata for simulator, rbpodo controller `pgmode` simulation, and future physical real demonstrations. The policy recorder preserves optional actual/reference TCP, q_ref/q_target, ACK, diagnostics, command source, and SpaceMouse fields without changing command paths or weakening deadman, lease, or real-motion gates.
51–58. *(removed 2026-06-20)* The rbpodo controller-simulation circle convenience
    wrappers, summary/metric tooling, controller-reference tracking-error and
    startup-reference seeding tied to the circle benchmark, the ablation override
    generator, the stage-2 circle matrices, and the tuning report have been
    deleted along with the circle-benchmark subsystem. (The general
    `controller_simulation_tracking_error_source` / startup-reference server
    behavior remains in the backend; only the circle-benchmark scripts/configs
    are gone.)
59. *(removed 2026-06-20)* MEASURE-P0-GATE-00 registered P0 measurement-reliability
    gates for the controller-simulation circle benchmark (state parity, raw 5001
    capture, timestamp alignment, circle error decomposition). The circle error
    decomposition / report tooling was deleted; the still-open `diagnostics_suspect`
    state-parity work is captured by items 60 and 66–68 below.
60. RBPODO-MEASURE-STATE-PARITY-01 publishes direct rbpodo reference-state metadata (`q_ref_deg`, `q_ref_source`, validity flags, SDK/decode policy) and adds read-only Python-vs-C++ state parity artifacts. Passing parity means both decoders agree on sampled fields; suspicious diagnostics remain motion-blocking and physical real motion is still unauthorized.
61. *(removed 2026-06-20)* RBPODO-500HZ-CIRCLE-MATRIX-01, RBPODO-MEASURE-TIMESTAMP-01,
    and RBPODO-CIRCLE-ERROR-DECOMP-01 (items 61–63) were the staged 500 Hz
    controller-simulation circle matrices and the offline timestamp-alignment /
    error-decomposition tooling for circle artifacts; deleted with the
    circle-benchmark subsystem.
61. RBPODO-MEASURE-RAW-DATA-01 adds read-only Rainbow data-port 5001 raw capture and fixture summary tooling for investigating `diagnostics_suspect` field-layout questions. It requires real-controller confirmation and `RB_ALLOW_REAL_ROBOT=1` for known controller IPs, avoids command-port traffic and binary layout guessing, and keeps raw payload artifacts under `artifacts/` unless sanitized.
64. *(removed 2026-06-20)* RBPODO-MEASURE-RELIABILITY-REPORT-01 and
    GATE-RBPODO-500-P0-P1-00 (items 64–65) graded circle-artifact measurement
    reliability and registered the 500 Hz / P1 circle gate tracks; deleted with the
    circle-benchmark subsystem. The read-only `diagnostics_suspect` state-parity
    work survives in items 60 and 66–68.
66. P0-PARITY-REPAIR-01 adds a dedicated read-only rbpodo measurement config and hardens Python-vs-C++ state parity classification. The parity checker refuses supplied configs unless `servo.send_servo_commands: false`, waits for any state packet including diagnostics-suspect or fault-latched packets, separates `failed_server_exit`, `failed_transport`, and `failed_parity_mismatch`, and records `diagnostics_suspect_unresolved` rather than treating consistent suspect diagnostics as transport failure. It remains read-only and does not set pgmode, reset faults, or send Servo J.
67. P0-DIAGNOSTICS-ROOTCAUSE-01 adds an offline rbpodo diagnostics root-cause report that combines state dump, Python/C++ parity, and raw data-port capture evidence. It classifies likely causes as SDK/firmware layout mismatch, unknown field semantics, controller-reported real fault, Python/C++ decode mismatch, payload instability, or insufficient evidence; records SDK/module and C++ state-stream hints when available; and keeps `diagnostics_suspect_unresolved`, `stop_resetFault_unverified`, and `physical_reference_to_actual_error_unmeasured` as physical-real blockers. It does not suppress `diagnostics_suspect`, does not send commands, and does not enable physical real motion.
68. P0-RAW-PAYLOAD-FIXTURE-02 turns Rainbow data-port raw capture into repeatable fixture evidence. Captures now store compact per-sample length/hash/prefix/suffix metadata with first/last payload binaries by default, require `--save-each-sample` for per-sample raw binaries, compute changed-byte offset histograms and q_ref/payload transition counts, and add an offline fixture report that recommends collecting motion/no-op fixtures and comparing firmware SDK docs before inferring any binary layout. It remains read-only and does not touch command-port, pgmode, control, backend, motion, or safety-gate behavior.
69–92. *(removed 2026-06-20)* The entire rbpodo controller-simulation 500 Hz
    circle / ACKON500 benchmark program — measurement-gating report fields,
    500 Hz circle config templates and the no-op/ACK-sweep acceptance runners,
    the P1 factor/orientation/dead-time/servo-param circle matrices, the async
    ACK-supervised circle matrices and reports/runbooks, and the ACKON500
    result-contract / rate-accounting / benchmark-lane / best-profile /
    repeatability tooling — has been deleted. The associated gate tokens
    (`RBPODO-500HZ-*`, `P1-CIRCLE-*` / `P1-ORIENTATION-DIAG` / `P1-DEADTIME-*` /
    `P1-SERVER-SIDE-CIRCLE-TRACK-SKELETON`, `RBPODO-ASYNC-CIRCLE-MATRIX-01`,
    `ACKON500-*`) are retired. The `TcpCircleTrack` command schema and its
    fail-closed server skeleton (`tcp_circle_track_not_implemented`) remain in the
    C++ servo loop as a code primitive; only the circle-benchmark tooling was
    removed. The async ACK worker / reference-supervisor server behavior
    likewise remains in the backend; only the circle-benchmark scripts, configs,
    and runbooks are gone.
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
