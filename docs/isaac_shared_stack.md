# Isaac shared-stack bridge

`rb_servo_server/build/rb_isaac_bridge <stack_real.yaml>` embeds the existing
`DualArmServoLoop` with an explicitly injected PhysX plant. It only exchanges
JSONL over stdin/stdout; library diagnostics go to stderr. It never constructs
rbpodo backends, probes boxes, starts network command/state services, or opens
physical grippers. The regular `rb_servo_server` entry point is unchanged.

The simulation application supplies actual joint position/velocity, in degrees,
and measured gripper opening. With `force_sensor_enabled: true`, it also supplies
a physical six-axis tool load at the flange. The bridge returns the shared controller's final
joint targets. Only backends with `supportsExternalStepping()` may opt into the
manual tick API. Mock and rbpodo do not opt in. Real execution keeps wall-clock
pacing and the asynchronous collision monitor.

The bridge loads the current tracked stack and explicitly projects plant-specific
runtime settings: direct I/O, manual 2 ms clock, synchronous collision evaluation,
nominal kinematics matching USD, optional PhysX F/T, physical gripper
bridge disabled, RT scheduling disabled. No source YAML is rewritten. It uses the
existing mock telemetry enum for compatibility and adds `plant: isaac_physx`.
This is separate from rbpodo's `run_mode: simulation` / controller pgmode.

The command lease parser, Cartesian profiles, delta_preview, output SMD,
Pinocchio IK, joint limits, collision/workspace projection and fault handling
are the shared implementations. `ChunkFrameReceiver.acceptPacket()` is the same
validated ingress used by UDP and the embedded transport.

JSONL protocol (maximum request 1 MiB):

- `init`: `{op, left:{q_deg[6],dq_deg_s[6],gripper_percent}, right:{...}}`.
  Initial clock is 1,000,000,000 ns and sequence 1.
- `init.force_sensor_enabled` selects the sensor contract for the entire session
  (omitted/false is the explicit older force-off test mode). When true, BOTH arms
  must supply `force_sensor: {valid:true, seq, time_ns, frame:"flange_at_flange",
  wrench:[Fx,Fy,Fz,Mx,My,Mz]}` on init and every step. SI units; sequence/time must
  equal the plant sample. Missing, nonfinite, stale or wrong-frame samples terminate
  the bridge. The source stack must enable both F/T arms and force control.
- `step`: same measurements plus `seq` (+1) and `time_ns` (+2,000,000).
- `command`: `{op:"command",packet:<existing command JSON>}`.
- `chunk`: `{op:"chunk",packet:<existing chunk_overlay.v3 JSON>}`.
- `close` or EOF stops the loop. Malformed plant state/time fails closed and exits 1.
- Replies contain `ok`, and step replies also contain the existing `state`, final
  per-arm targets, clock/sequence, and the explicit projection list. Rejected
  command/chunk packets have `ok:false`. The Python transport raises on rejection.

F/T uses the existing `FtPipeline`, gravity compensation, deadzone, tare,
coverage, admittance, force gate and deviation limits. The bridge shifts torque
from flange to the configured SRO (`M_sro = M_flange - r_sro × F`) and encodes the
configured electrical axis map before the usual pipeline decodes it. Its measured
left-handed channel mapping is retained. The only common pipeline addition is an
externally verified connection verdict for the externally stepped backend: a
deterministic sensor has no RFT electrical noise. It never grants a bias or covers
an invalid wrench. Hardware retains its original liveness procedure.

The Python plant reads negative incoming reaction at `attachment_site_joint`
(the whole tool's load path, unlike the `ft_sensor_measurement` leaf), validates
gravity residual before tare, then issues the existing `TareForceSensor` command
and allows the normal 250-sample averaging to complete. Source auto-tare is off
for this reset-only bootstrap; no synthetic bias/noise is injected. The JSON
contact profile and aggregate payload adjustment are documented in simulation.

The Python runtime keeps `policy_runner.main.run`, `OpenpiRemoteActionSource`,
`ServoCommandClient`, and `ChunkOverlayPublisher`. `isaac_transport.py` supplies
the pipe adapters, state client, simulation clock and model-response latency
scheduling. Defaults in the existing runner/source retain real behavior.

Run instructions, evidence and current contact limitations:
[simulation/docs/shared_stack.md](../../simulation/docs/shared_stack.md).
The simulation JSON schema is `robotics_lab.isaac_shared_stack.v1`, at
`simulation/config/shared_stack.json`; the real stack schema is unchanged.

Validation:

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j6
ctest --test-dir rb_servo_server/build --output-on-failure
PYTHONPATH=policy_runner python -m unittest discover policy_runner/tests -p test_isaac_transport.py -v
```

Hardware-free tests and sim rollout results do not establish physical acceptance.
No real config or real robot run was changed/executed for this implementation.

2026-09-05 F/T validation: CTest 44/44; Python bridge/clock/F/T 11/11;
simulation replay tests 4/4; actual PhysX force/torque injection 48 cases, maximum
error 0.038503 N / 0.028067 Nm. Correcting the tray wall geometry changed an
identical-input replay from a 13.04 s fault to a completed 15.002 s run, but the
final :8001 live run still faulted at policy time 22.496 s with overloaded contact.
This implementation is an instrumented provisional contact model, not physical
task acceptance. See the simulation report for the evidence and remaining limits.

2026-09-06 current simulation state: calibrated foam appearance and gripper close
bias 0/0, with each of the three local policies completing 30.002 s without a
controller fault. Correct final tray counts (gray/black) were 1/0, 0/0 and 1/1
for ports 8001, 8002 and 8003 on one shared scene. This is not a general success
rate. Camera intrinsics and contact identification remain open.

Pre-commit validation reran CTest 44/44, Isaac transport tests 11/11 and replay
tests 4/4. The full policy runner suite ran 570 tests with 2 failures and 3 skips;
the two `ChunkKnotFilterTest` failures also reproduce on unchanged commit
`62ee102eedeaa2faac787e39c42fe1e053da0066` with identical assertions. They were
not modified by this integration. See
[simulation validation](../../simulation/docs/validation_20260906.md) and
[recorded results](../../simulation/docs/results/20260906/README.md).
