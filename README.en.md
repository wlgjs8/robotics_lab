# robotics_lab

English operational summary. The Korean [README.md](README.md) is the primary
public overview; normative details live in the contracts linked below.

`robotics_lab` integrates two Rainbow RB3-730E arms, the C++ servo server,
rbpodo real/controller-`pgmode` backends, cameras, the operator GUI, grippers,
and `policy_runner`.

## Current phase and maturity

The milestone is **rbpodo pgmode-real physical robot bring-up**. Mock and
rbpodo controller `pgmode` simulation remain the regression baseline. Physical
hardware has run, under operator supervision:

- read-only diagnostics parity;
- a slow dual-arm Cartesian circle;
- UMI Cartesian teleop and replay;
- a full pi0.5/openpi `flow-infer` `real_policy` rollout using absolute
  `TcpPoseTarget` setpoints and real grippers;
- the async URDF-mesh `CollisionMonitor` safety path; and
- the controller-manager-referenced force-control v2 overlay.

Runtime integration is validated; policy task success remains model/data
limited. Fast circle stages and general measured hand-eye calibration remain
open. The deployed pika ee-local image-conditioned policy has an accepted
policy-specific calibration carve-out; that does not make the configured
estimate valid for arbitrary geometry-dependent policy.

Passing simulation is never permission to move physical hardware.

## Sources of truth

- `AGENTS.md`: repository rules and safety invariants
- `docs/architecture.md`: topology, public terminology, safety ownership
- `docs/servo_backend_contract.md`: backend, worker I/O, queue, force, and
  telemetry contracts
- `docs/frame_contract.md`: frame/calibration contract
- `docs/joint_range_policy.md`: raw joint representation and limits
- `docs/hardware_free_validation.md`: hardware-free boundary
- `rb_servo_server/config/stack_real.yaml` and `stack_sim.yaml`: the only
  runnable stack configurations

Historical plans, archived force-control v1 material, and point-in-time reports
are evidence only. They do not override the sources above.

## Canonical terminology and topology

```yaml
run_mode: mock | simulation | real
backend_type: mock | rbpodo
```

Rbpodo is the only supported real-controller backend. `MockBackend` is the
hardware-free surface. The retired software-simulator and raw-script comparison
backends are not supported.

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

Controller `pgmode` simulation uses the same rbpodo endpoint shape with
`operation_mode: simulation` and `physical_motion_expected: false`. The tracked
`stack_sim.yaml` retains `run_mode: real` because it connects to an actual
controller endpoint; `run_mode: simulation` must not be reused for a retired
software-simulator backend.

## Safety and execution authority

The legacy `RB_ALLOW_REAL_*` server execution gates were removed. Real motion
is authorized by the reviewed tracked stack config plus mode-independent server
safety layers:

- joint/rate/acceleration and workspace safety filtering;
- tracking-error latching;
- async URDF-mesh self-collision monitoring;
- reach, ROI, and configured floor constraints;
- command-source lease/arbitration and client deadman; and
- operator supervision with a hardware E-stop.

`run_mode` and `operation_mode` are telemetry labels, not motion gates. The
policy-side real-Cartesian block is retired; `rb_servo_server` makes the final
allow/deny decision. Real gripper motion remains separately gated by
`allow_real_gripper_motion`, measured availability, and
`RB_ALLOW_REAL_GRIPPER=1`.

Do not create `config/local` launch variants. Change one reviewed setting at a
time in the appropriate tracked stack config so the effective runtime profile
remains visible and auditable.

## Joint range contract

Rbpodo state, commands, safety, tracking, and logs preserve raw controller
degrees. They are not normalized to `[-180, 180]`.

```yaml
safety:
  q_min_deg: [-360, -360, -150, -360, -360, -360]
  q_max_deg: [360, 360, 150, 360, 360, 360]
```

J3 is exactly `[-150 deg, +150 deg]`, matching Rainbow's RB3-730E range and the
URDF/Pinocchio model. The retired `+/-160 deg` margin and a widened J3 must not
be used to mask an unreachable Cartesian target.

## Servo J and control-box queue

Both tracked profiles pin the 500 Hz transparent-executor parameters:

```yaml
servo_t1_sec: 0.002
servo_t2_sec: 0.021
servo_gain: 1.0
servo_alpha: 10.0
```

Rainbow scales gain/alpha by `0.1`, so script-level `alpha=10.0` is effective
`1.0`: controller LPF off. Smoothing and bounds belong to the server loop.

Firmware v8.7.3 consumes Servo J through a FIFO. `stack_real.yaml` therefore
uses worker I/O, per-arm RT scheduling, and `queue_sync` at target fill 5.
Motion and the follower plan are held during qsync warmup/drain and released in
track. Worker-side setpoint interpolation exists and is unit-tested, but the
tracked real profile keeps it `false` pending a separate supervised hardware
A/B. `stack_sim.yaml` uses direct I/O without queue regulation, so its latency
is not comparable to physical real.

## Force-control v2

Force-control v1 was removed on 2026-08-26 and archived. V2 was rebuilt from
controller-manager's calibrated sensor/tool presets and is live:

- `force_torque:` and `force_control:` are server config sections;
- both are present and enabled in `stack_real.yaml`;
- the measured sensor basis on this cell is left-handed (`det=-1`);
- gate and spring ship together; and
- wrench reference point and compose pivot are both the TCP.

An arm without a valid bias is never covered. The GUI's leaseless
`TareForceSensor` command and `force_torque.auto_tare_after_init_motion` share
the same RT tare path: 250 samples of `raw - gravity`. Automatic tare arms when
InitMotion is requested but samples only after arrival, settling, and a low
sent-speed check. A client-supplied `force_control` command object is still
rejected: force-law authority is server config, not an action payload.

## Public motion surface

1. `JointTarget`: absolute joint-space PTP.
2. `TcpPoseTarget`: Cartesian final-pose PTP; intermediate TCP path is not
   guaranteed linear.
3. `TcpLinearMove`: finite MoveL-like path with `constant` or `slerp`
   orientation behavior.

`SetSafetyFloorZ` and `TareForceSensor` are leaseless non-motion commands, not
motion primitives.

## Build, test, and launch

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner scripts

cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

Eigen3 and Pinocchio are mandatory for the C++ Cartesian path. Missing
dependencies are not a fallback mode.

```bash
make build
make run            # physical real stack; operator supervision required
make run MODE=sim   # rbpodo controller pgmode simulation
```

The GUI is served at `http://127.0.0.1:8080`. Camera services are managed
separately with the `make cam-*` targets. Follow the supervised runbooks for
any physical operation.
