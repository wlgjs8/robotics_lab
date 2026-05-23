# Hardware-Free Validation Gate

This gate is the local and CI-equivalent regression command set for
hardware-free development. It exercises only mock/stub and loopback simulator
code paths and does not start real robot motion, RealSense capture, Docker,
privileged deployment, external network, credentialed, or production network
checks.

The architecture source of truth is [architecture.md](architecture.md). Public
terminology is:

```yaml
run_mode: mock | simulation | real
backend_type: mock | simulator | rbpodo
```

Run from the workspace root:

```bash
bash scripts/check_deps.sh --profile hardware-free
bash scripts/hardware_free_validation.sh
```

The script runs:

- `scripts/check_deps.sh --profile hardware-free`: fails before CMake when
  CMake, a C++17 compiler, `yaml-cpp`, `nlohmann_json`, or `python3` is
  missing.
- `camera_server`: CMake configure/build/CTest with `CAMERA_SERVER_FORCE_MOCK_CAMERA=ON` and `CAMERA_SERVER_FORCE_ZMQ_STUB=ON`.
- `rb_servo_server`: CMake configure/build/CTest with `RB_SERVO_ENABLE_RBPODO=OFF`,
  including the per-arm `rbsim_hardware_free_gate` integration test.
- `rb_servo_gui`: stdlib `unittest` discovery registered as the `rb_servo_gui_unittest` CTest test with `PYTHONPATH` pointed at top-level `rb_gui`.
- `rb_servo_server/tools/analyze_servo_log.py --self-test`: local
  mock/simulator analyzer profiles and fail-closed parser checks for generated
  sample logs.
- `rb_simulator`: `compileall` over the simulator package/tools, then stdlib
  `unittest` discovery for the deterministic simulator state-machine core and
  loopback JSONL protocol, including the structured MIG-03 error envelope
  (`error.kind/name/message/code/retryable/recoverable`) and state snapshots
  on stateful robot/controller rejections.
- `rb_simulator/tools/rbsim_servo_smoke.py --self-test`: parser, state-stream,
  and servo-log validator coverage without launching processes.
- Full local loopback smoke when prerequisites are present: starts separate
  left and right simulator Python module processes plus the freshly built
  `rb_servo_server`, checks wrong-arm requests fail closed, sends a small
  command through the direct simulator profile, and validates UDP state plus CSV
  log evidence.
- Worker-mode local loopback smoke when prerequisites are present: repeats the
  same per-arm simulator smoke with
  `rb_servo_server/config/dual_simulator_worker.yaml`, proving
  `servo.io_model: worker` can receive state and send joint commands without
  real hardware.

The rb_servo configure sets `RB_SERVO_ALLOW_FETCHCONTENT=OFF` so the local gate
does not silently download missing dependencies. If `nlohmann_json` is installed
outside the default CMake search path, set `CMAKE_PREFIX_PATH` before running the
script.

Additional dependency preflight profiles are available:

```bash
bash scripts/check_deps.sh --profile real-camera
bash scripts/check_deps.sh --profile real-robot
bash scripts/check_deps.sh --profile kinematics
```

These profiles report dependency readiness only. They do not open real robot
motion gates, start RealSense capture, or validate hardware.

Some sandboxed CI runners block `AF_INET` socket creation. In that case the
rb_servo tests keep the parser, sequence, source-allowlist, and config
validation assertions active, and self-skip only the live loopback UDP ingress
checks. The rb_simulator protocol tests likewise keep non-socket assertions
active and self-skip the live JSONL server cases when loopback socket creation
is denied by the sandbox.

The full simulator smoke is controlled by `RBSIM_SMOKE_MODE`. The environment
variable names are compatibility names in the current script, not public
architecture terminology:

- `auto` (default): run the full smoke when the freshly built servo-server
  binary, both per-arm simulator configs, and loopback socket support exist.
- `required`: fail closed if any full-smoke prerequisite is missing.
- `skip`: skip the full smoke intentionally while still running build, unit, and
  smoke-validator checks.

The worker-mode simulator smoke is controlled separately by
`RBSIM_WORKER_SMOKE_MODE` with the same `auto|required|skip` values. The worker
profile defaults to `rb_servo_server/config/dual_simulator_worker.yaml` and can
be overridden with `RBSIM_WORKER_SERVO_CONFIG`.

Dependency prerequisites for `RBSIM_SMOKE_MODE=required`:

- `RBSIM_COMMAND` defaults to `python3 -m rbsim`; the validation script passes
  it as both `--left-simulator-command` and `--right-simulator-command`, with
  `PYTHONPATH` set to `rb_simulator/src` by the smoke runner.
- `RBSIM_LEFT_CONFIG` and `RBSIM_RIGHT_CONFIG` point to per-arm simulator YAML
  profiles. Defaults are `rb_simulator/config/left_rb3_730e.yaml` and
  `rb_simulator/config/right_rb3_730e.yaml`.
- `RBSIM_SERVO_CONFIG` may point to a loopback-only direct simulator backend
  profile. If unset, the script uses
  `rb_servo_server/config/dual_simulator.yaml`.
- `RBSIM_WORKER_SERVO_CONFIG` may point to a loopback-only worker simulator
  backend profile. If unset, the script uses
  `rb_servo_server/config/dual_simulator_worker.yaml`.
- CMake can build `rb_servo_server` with `RB_SERVO_ENABLE_RBPODO=OFF` and
  `RB_SERVO_ALLOW_FETCHCONTENT=OFF`.

Smoke artifacts are written under
`rb_simulator/artifacts/hardware_free_gate` by default:

```text
direct/state_stream.jsonl
direct/servo_log.csv
direct/left_simulator.log
direct/right_simulator.log
direct/rb_servo_server.log
direct/summary.json
worker/state_stream.jsonl
worker/servo_log.csv
worker/left_simulator.log
worker/right_simulator.log
worker/rb_servo_server.log
worker/summary.json
```

The state stream includes MIG-10 diagnostics for each arm:
`state_age_us`, `send_result_age_us`, `send_deadline_hit`, and
`worker_loop_read_duration_us`. Top-level fields include `command_seq`,
`send_deadline_hit`, `send_skew_us`, and `dispatch_skew_us`. The CSV servo log
records the same age, deadline, skew, and worker-read-duration metrics.

Current compatibility assumption: the simulator process still binds loopback
only. The hardware-free smoke therefore uses left `127.0.0.1:50200` and right
`127.0.0.1:50210`. Docker Compose keeps the canonical
`rb_simulator_left:50200` and `rb_simulator_right:50200` topology by using the
simulator image entrypoint to proxy container port `50200` to the loopback-only
Python module inside each container.

Skipped by design:

- real robot and Rainbow rbsim motion
- external simulator or real robot timing acceptance; analyzer profiles here
  cover only hardware-free `rb_simulator` loopback logs
- RealSense capture and hardware-sync acceptance
- privileged containers, host IPC/network, and USB device access
- command-sender tools against non-mock endpoints
- production network exposure, external network, or credentialed operations
- force/admittance/impedance control
- real Cartesian/TCP motion

When new first-wave C++ regressions are added to the existing CTest targets, this
gate picks them up through `ctest`. Hardware acceptance remains a separate
human-gated task.
