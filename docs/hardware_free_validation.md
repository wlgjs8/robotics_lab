# Hardware-Free Validation

This document defines the default validation boundary for development without physical robots or cameras.

## Scope

Hardware-free validation may exercise:

- mock servo backend
- rb_gui parser/safety/model tests
- policy_runner action-source tests
- camera mock/stub paths
- C++ unit tests that do not require hardware
- mock-mode smoke tests

> The old per-arm software-simulator backend and its hardware-free simulator
> lane were retired. Hardware-free validation now uses the mock backend;
> controller behavior beyond mock is validated on rbpodo controller `pgmode`
> simulation (VM or onbox) and real.

It does not prove:

- real RB5-850 readiness
- real Cartesian/TCP readiness
- RealSense readiness
- gripper readiness
- physical force-control acceptance, sensor axes/signs, tare quality, contact
  response, or force/deviation fences
- measured calibration validity

## Base Commands

Python checks:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner scripts
```

Shell syntax checks:

```bash
bash -n scripts/check_deps.sh
if [ -f scripts/install_deps_ubuntu.sh ]; then bash -n scripts/install_deps_ubuntu.sh; fi
```

C++ hardware-free checks:

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

Cartesian behavior is covered by the Pinocchio-backed C++ tests above plus
active-stack smoke when a local mock config is available. The old
software-simulator-oriented Cartesian acceptance runner is no longer part of
the hardware-free validation surface.

## Dependency Preflight

```bash
./scripts/check_deps.sh --profile hardware-free
```

Install missing base packages on Ubuntu:

```bash
./scripts/install_deps_ubuntu.sh --profile hardware-free
```

`rb_servo_server` hardware-free C++ checks require valid Eigen3 and Pinocchio
CMake packages. The Ubuntu helper installs Pinocchio under `/opt/openrobots`:
robotpkg remains the jammy default, while non-jammy hosts such as Ubuntu 24.04
noble use a pinned Pinocchio `v3.9.0` source build when `ROBOTPKG_DIST` is
unset. Source builds use an automatic memory-capped job limit, overridable with
`PINOCCHIO_BUILD_JOBS`, to avoid OOM on low-RAM hosts. Missing Pinocchio is
reported as `Missing CMake package: pinocchio`. A skipped C++ build is not
acceptance evidence.

## Mock Smoke

Run `rb_servo_server` with an explicit temporary mock config outside the
repository and drive it with the bundled sender tools; the
state stream and servo log are the smoke evidence. Controller behavior beyond
mock is validated on rbpodo controller `pgmode` simulation (VM or onbox).

## Direct And Worker I/O

Direct mode and worker mode are both hardware-free (mock) validation targets.

Direct mode validates the straightforward servo loop/backends path.

Worker mode validates the architecture where each arm worker owns blocking backend I/O and its own send cadence. Worker mode is no longer mock-only — it is a supported real-mode path — so hardware-free coverage here is a regression baseline, not the boundary of where worker mode may run.

## Expected State Telemetry

Hardware-free state snapshots should expose enough diagnostics to debug motion and I/O behavior:

- `observed_mode`
- `observed_backend`
- `fault_context`
- arm-level `last_read`
- arm-level `last_send`
- `send_policy`
- `send_suppressed`
- dispatch/send skew
- worker drop counters, when worker mode is active
- TCP pose fields, when FK is enabled
- Cartesian solve/path telemetry, when Cartesian modes are active

## Pass Criteria

A hardware-free validation run is useful only when:

- tests are not skipped silently
- skipped checks are clearly reported with the missing dependency
- no real robot IPs are used in mock configs
- no real robot env gates are required
- no physical F/T device or force-motion path is enabled by a mock smoke
- mock-mode motion primitives stay hardware-free

## Not A Hardware Acceptance

The optional `RB_SERVO_BUILD_PREVIEW_EXPERIMENTS=ON` build adds a constrained
trajectory solver and copied-follower reference tests. Its recorded replay has
no backend/device path and no output LPF. The separate runtime flag
`RB_SERVO_ENABLE_PREVIEW_EXECUTION` connects a worker and finite executor to the
server under the explicit `flow_infer_preview` profile. Its native coordinator,
contact, stop and production-loop tests are separate from instantaneous offline
replay. Clean native integration passes 58 tests; both full recorded streams and
the real-backend config-only preflight also pass. Physical acceptance remains
separate; see the
[execution contract and current status](reference/preview_trajectory_execution.md).
The late-evening geometry/admission/rotation repair has separate evidence in
`outputs/preview_fold_repair_20260906/`: 56 native targets plus the GUI/tools
regression targets pass; all49 analysis-tool tests additionally execute in the
OpenPI analysis environment, because the repository GUI environment lacks the
optional pandas/pyarrow dependencies. New coverage includes exact geometry
booking/application timing through the production loop, EmergencyStop before
application and new-source resumption, strict gauge transport of pending/staged
plans, full angular norm certificates and nonzero-acceleration C2 rebases. The
three recorded default candidate replays complete without expiry faults; they
retain recorded force/geometry inputs and ideal nominal dispatch. Their success
is not physical controller/model acceptance. All objective attempts, including
a timing-sensitive failed1000 run and a backlog-limited4000 run, are retained.


The fresh chunk path also has dedicated hardware-free coverage:
`chunk_fresh_continuity` exercises C++ sampled-p/v/a splices, SO(3) derivatives,
force/plan gates and held-reference resume; `force_overlay_resume` repeats the
actual servo-loop InitMotion/tare/first-chunk regression with both new flags
enabled. Policy scheduler tests use the production dispatcher, and
`test_servo_command_proprio.py` checks frozen windows, epochs, gripper capture,
stale/missing data and model-request refusal. Recorded-latency and recorded-delta
replays are conditional comparisons, not hardware/model acceptance. See
[the design and evidence map](plans/plan_fresh_chunk_execution_20260906.md).

A passing hardware-free gate is not permission to run a physical robot. Real robot work must start with a separate read-only acceptance plan.


The output-conditioner regression also checks the selected tracked real gain,
frequency response against the historical amplification, and a moving-reference
stop. `test_follower_output_smd --replay-reference INPUT OUTPUT STACK_YAML PROFILE`
replays sampled references without connecting devices. Its JSONL requires explicit
initial pose/velocity and documents any reconstructed derivatives; it does not
recreate the follower or robot feedback. `recorded_pose_ik_audit` then audits actual
Pinocchio IK and joint clamps with a hashed per-arm calibrated URDF. It excludes
plant/contact feedback and the geometric/collision safety projection. The actual
mock servo-loop InitMotion/tare/IK-refusal regression is run separately. See
[conditioner evidence and limitations](reference/follower_output_conditioning.md).

The position-only conditioner additionally checks fixed-period non-amplification,
SO(3) changing-axis transport, antipodal quaternion representation, variable
periods, reset/fold and the selected profile through actual mock-loop
InitMotion/tare/IK-refusal fixtures. The optional deadline jerk search checks
the original p/v/a endpoint, deadline and caps, with exact fallback on braking
or infeasible cases.

`delta_follower_replay --recorded CONFIG PROFILE EVENTS OUTPUT` consumes explicit
recorded follower inputs without devices. `--conservative-stage-leash` recomputes
the leash from its own prior output, min-combined with the recorded gate; it is
still a fixed-model/force/actual-feedback counterfactual. Export with
`tools/prepare_recorded_follower_replay.py`, analyze with
`tools/analyze_recorded_follower_replay.py`. These analytics require pandas,
pyarrow and SciPy in the analysis interpreter. Exporter tests explicitly skip
when pandas/pyarrow are absent from the minimal test environment; run them in
the complete analysis environment as well, and report those skips separately.


### Post-tare state UDP regression (2026-09-06 23:02)

This communication-only repair removes the obsolete duplicate top-level
`last_cartesian_solve` alias and explicitly summarizes the new preview diagnostics
on UDP, preserving complete canonical per-arm state and full forensic CSV.
The expanded finite serializer fixture includes pose/IK/force, accepted tare,
transport/async/worker/timing/Init data and exact 64-bit IDs. Previous packets
75,968/75,960 B become57,014/57,006 B; valid-gripper localhost fanout is57,310 B.
These are synthetic byte-coverage fixtures, not captured dropped datagrams.

Three final C++ contracts pass (`state_publisher`, `safety_policy`,
`servo_logger_columns`,16.10 s);170 GUI/policy consumer tests and generated-packet
parsing/gating checks pass. Two ephemeral local UDP receivers obtain identical
packets with no overflow/send errors. Full CSV tests remain intact. The initial
policy test environment lacked torch; the configured OpenPI environment executes
the policy tests successfully. No physical backend is started. Motion equations,
limits and stack configuration are unchanged; physical rollout and full dynamics
replay are not repeated for this publication-only fix. Evidence and exact real
binary qualification: `outputs/state_udp_repair_20260906/report.md`.
