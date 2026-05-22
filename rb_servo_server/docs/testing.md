# Testing

Build and run the safety policy tests:

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Mock smoke test:

```bash
./build/rb_servo_server --config config/dual_mock.yaml
python3 tools/send_dual_joint_sine.py --rate 30 --amp-deg 2 --freq 0.2
```

The sine tool sends `ArmMotion` before its first `JointTarget` and waits briefly by default so a one-slot command receiver cannot lose the arm transition. The C++ `CommandBuffer` also preserves lifecycle commands separately from latest motion targets.

The smoke is meaningful only if `logs/servo_log.csv` shows `JointTarget`, `Running`, and non-trivial sent joint motion. After `EmergencyStop` and `ResetFault`, send `ArmMotion` again before motion targets.

The CSV should also contain send timing columns:

- `left_send_start_ns`, `left_send_end_ns`
- `right_send_start_ns`, `right_send_end_ns`
- `send_skew_us`
- `left_send_duration_us`, `right_send_duration_us`

Use these columns as measurement evidence before changing the sender architecture or attempting rbsim/real bring-up.

Full milestone budget checks use the stdlib analyzer:

```bash
python3 tools/analyze_servo_log.py --profile mock200 logs/servo_log.csv
python3 tools/analyze_servo_log.py --profile rbsim-local100 logs/servo_log.csv
python3 tools/analyze_servo_log.py --profile rbsim100 logs/servo_log.csv
```

`mock200` expects a 60 s, 200 Hz mock run. `rbsim-local100` is the short
local simulator-backed profile for 2 s, 100 Hz smoke logs. `rbsim100` is the
longer optional simulator-backed profile for 30 s, 100 Hz logs. The analyzer
fails closed on missing send/timing/joint columns, malformed send timestamps,
dropped samples, send failures, bad duration/rate/jitter/skew/send-duration
budgets, and tracking error above 2 deg.

The rbsim analyzer profiles validate only the hardware-free rb_simulator +
rb_servo_server loopback logs. They do not prove Rainbow rbsim timing,
network/host scheduling readiness, or real robot timing acceptance; those remain
separate human-gated hardware tasks.

## Hardware-free rb_simulator path

The local simulator path is documented in `docs/rb_simulator_dev.md`. Use
`config/dual_rb_simulator.yaml` with `../rb_simulator/config/dual_rb3_730e.yaml`;
both bind only loopback endpoints in this phase.

Simulator unit and contract checks:

```bash
python3 -m unittest discover ../rb_simulator/tests
python3 ../rb_simulator/tools/rbsim_servo_smoke.py --self-test
```

When the local simulator executable and `rb_servo_server` binary both exist,
the bounded smoke command is:

```bash
python3 ../rb_simulator/tools/rbsim_servo_smoke.py \
  --simulator ../rb_simulator/build/rb_simulator \
  --simulator-config ../rb_simulator/config/dual_rb3_730e.yaml \
  --server build/rb_servo_server \
  --server-config config/dual_rb_simulator.yaml \
  --artifacts-dir ../rb_simulator/artifacts/rbsim_servo_smoke
```

Pass evidence must include state packets, a servo CSV log, zero send failures,
zero dropped logger samples, and matching sent-joint rows for the small target.
Stop/reset/fault evidence must come from local simulator hooks and artifacts:
`stop` holds the last safe target, `reset_fault` returns only to
`ConnectedHold` and requires a new `ArmMotion`, and injected invalid state or
send/stop/reset/disconnect failures produce explicit hold or latch behavior.

This path does not validate Rainbow Robotics rbsim/OVA, `rbpodo`, real robot
motion, realtime scheduling acceptance, privileged Docker, broad network
exposure, or credentialed operations.

## Out-of-scope hardware gates

Real-mode startup, Rainbow Robotics rbsim/OVA validation, `rbpodo` validation,
privileged Docker, host networking, broad network exposure, and hardware-facing
sender tools are intentionally outside this hardware-free test phase. Keep those
under separate human-gated runbooks.
