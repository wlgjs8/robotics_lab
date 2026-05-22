# Hardware-Free Validation Gate

This gate is the local and CI-equivalent regression command set for first-wave
safety fixes. It exercises only mock/stub and loopback simulator code paths and
does not start real robot, Rainbow rbsim, RealSense, Docker, privileged
deployment, external network, credentialed, or production network checks.

Run from the workspace root:

```bash
bash scripts/hardware_free_validation.sh
```

The script runs:

- `camera_server`: CMake configure/build/CTest with `CAMERA_SERVER_FORCE_MOCK_CAMERA=ON` and `CAMERA_SERVER_FORCE_ZMQ_STUB=ON`.
- `rb_servo_server`: CMake configure/build/CTest with `RB_SERVO_ENABLE_RBPODO=OFF`.
- `rb_servo_gui`: stdlib `unittest` discovery registered as the `rb_servo_gui_unittest` CTest test with `PYTHONPATH` pointed at top-level `rb_gui`.
- `rb_servo_server/tools/analyze_servo_log.py --self-test`: local
  mock/rbsim analyzer profiles and fail-closed parser checks for generated
  sample logs.
- `rb_simulator`: `compileall` over the simulator package/tools, then stdlib
  `unittest` discovery for the deterministic dual-arm state-machine core and
  loopback JSONL protocol.
- `rb_simulator/tools/rbsim_servo_smoke.py --self-test`: parser, state-stream,
  and servo-log validator coverage without launching processes.
- Full local loopback smoke when prerequisites are present: starts the
  `rb_simulator` executable plus the freshly built `rb_servo_server`, sends a
  small command through the rbsim profile, and validates UDP state plus CSV log
  evidence.

The rb_servo configure sets `RB_SERVO_ALLOW_FETCHCONTENT=OFF` so the local gate
does not silently download missing dependencies. If `nlohmann_json` is installed
outside the default CMake search path, set `CMAKE_PREFIX_PATH` before running the
script.

Some sandboxed CI runners block `AF_INET` socket creation. In that case the
rb_servo tests keep the parser, sequence, source-allowlist, and config
validation assertions active, and self-skip only the live loopback UDP ingress
checks. The rb_simulator protocol tests likewise keep non-socket assertions
active and self-skip the live JSONL server cases when loopback socket creation
is denied by the sandbox.

The full rbsim smoke is controlled by `RBSIM_SMOKE_MODE`:

- `auto` (default): run the full smoke only when the simulator executable, the
  freshly built servo-server binary, and `config/dual_rb_simulator.yaml` exist.
- `required`: fail closed if any full-smoke prerequisite is missing.
- `skip`: skip the full smoke intentionally while still running build, unit, and
  smoke-validator checks.

Dependency prerequisites for `RBSIM_SMOKE_MODE=required`:

- `rb_simulator/build/rb_simulator` or `RBSIM_EXECUTABLE` points to an
  executable hardware-free simulator.
- `rb_servo_server/config/dual_rb_simulator.yaml` or
  `RBSIM_SERVO_CONFIG` points to a loopback-only `backend_type: rbsim` profile.
- CMake can build `rb_servo_server` with `RB_SERVO_ENABLE_RBPODO=OFF` and
  `RB_SERVO_ALLOW_FETCHCONTENT=OFF`.

Skipped by design:

- real robot and Rainbow rbsim motion
- Rainbow rbsim or real robot timing acceptance; analyzer profiles here cover
  only hardware-free rb_simulator loopback logs
- RealSense capture and hardware-sync acceptance
- Docker compose, privileged containers, host IPC/network, and USB device access
- command-sender tools against non-mock endpoints
- production network exposure, external network, or credentialed operations

When new first-wave C++ regressions are added to the existing CTest targets, this
gate picks them up through `ctest`. Hardware acceptance remains a separate
human-gated task.
