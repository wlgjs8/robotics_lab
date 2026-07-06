# Testing

Build and run the safety policy tests:

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Mock smoke test, when a site-local mock config exists:

```bash
./build/rb_servo_server --config config/local/<mock-config>.yaml
python3 tools/send_dual_joint_sine.py --rate 30 --amp-deg 2 --freq 0.2
```

The sine tool sends `ArmMotion` before its first `JointTarget` and waits briefly by default so a one-slot command receiver cannot lose the arm transition. The C++ `CommandBuffer` also preserves lifecycle commands separately from latest motion targets.

The smoke is meaningful only if `logs/servo_log.csv` shows `JointTarget`, `Running`, and non-trivial sent joint motion. After `EmergencyStop` and `ResetFault`, send `ArmMotion` again before motion targets.

The CSV should also contain send timing columns:

- `left_send_start_ns`, `left_send_end_ns`
- `right_send_start_ns`, `right_send_end_ns`
- `send_skew_us`
- `left_send_duration_us`, `right_send_duration_us`

Use these columns as measurement evidence before changing the sender
architecture or attempting simulator/real bring-up.

Full milestone budget checks use the stdlib analyzer:

```bash
python3 tools/analyze_servo_log.py --profile mock200 logs/servo_log.csv
```

`mock200` expects a 60 s, 200 Hz mock run. The analyzer
fails closed on missing send/timing/joint columns, malformed send timestamps,
dropped samples, send failures, bad duration/rate/jitter/skew/send-duration
budgets, and tracking error above 2 deg.

The mock analyzer profile validates only the hardware-free mock +
rb_servo_server loopback logs. It does not prove Rainbow external simulator
timing, network/host scheduling readiness, or real robot timing acceptance;
those remain separate human-gated hardware tasks.

## Hardware-free path

Hardware-free testing uses C++/Python unit tests plus an explicit local mock
config when mock-mode smoke is needed. Cartesian behavior is validated against
Pinocchio-backed C++ tests and active-stack smoke. Controller-level simulation
uses the rbpodo controller `pgmode` simulation (`make run MODE=sim`) or the
Rainbow virtual control-box VMs. The old software-simulator-oriented Cartesian
acceptance runner is no longer part of this validation surface.

This path does not validate Rainbow Robotics external simulator/OVA, real robot
motion, realtime scheduling acceptance, privileged Docker, broad network
exposure, or credentialed operations.

## Out-of-scope hardware gates

Real-mode startup, Rainbow Robotics external simulator/OVA validation,
`rbpodo` validation, privileged Docker, host networking, broad network
exposure, and hardware-facing sender tools are intentionally outside this
hardware-free test phase. Keep those under separate human-gated runbooks.
