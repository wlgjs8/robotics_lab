# Rainbow VM And Real Controller Parity

This runbook keeps home Rainbow VM development and office real-controller
bring-up on the same rbpodo code path. VM evidence is controller-simulation
evidence only, not physical acceptance.

## Mode Matrix

| Field | Home VM | Office Real |
| --- | --- | --- |
| `backend_type` | `rbpodo` | `rbpodo` |
| `run_mode` | `real` | `real` |
| `operation_mode` | `simulation` | `real` |
| IPs | two VM IPs | site controller IPs |
| command/data ports | rbpodo fixed `5000/5001` | rbpodo fixed `5000/5001` |
| Cartesian real gate | never `RB_ALLOW_REAL_CARTESIAN` | future physical acceptance only |
| evidence tag | `source=controller_simulation_vm` | physical run-specific source |
| `physical_motion_expected` | `false` | true only in approved physical acceptance |

`rb_simulator` remains a separate hardware-free contract mock using JSON/TCP
endpoints. It is useful for deterministic backend tests, but it does not
exercise rbpodo SDK, Rainbow controller protocol, firmware state decoding, or
Servo J ACK semantics.

## Local Config Pattern

Keep tracked templates generic and put runnable site configs under
`rb_servo_server/config/local/`.

For VM parity configs, prefer env-indirected IPs:

```yaml
left_robot:
  backend_type: rbpodo
  run_mode: real
  operation_mode: simulation
  ip: "${ROBOT_LEFT_IP}"

right_robot:
  backend_type: rbpodo
  run_mode: real
  operation_mode: simulation
  ip: "${ROBOT_RIGHT_IP}"
```

The config loader expands `${ROBOT_LEFT_IP}` and `${ROBOT_RIGHT_IP}` only for
robot `ip` fields. Missing or empty variables fail during config load.

## Environment Helpers

Print fail-closed VM exports:

```bash
tools/vm/home_vm_env.sh
```

Read-only state bring-up:

```bash
eval "$(tools/vm/home_vm_env.sh --left-ip <left-vm-ip> --right-ip <right-vm-ip> --readonly)"
```

Controller-simulation Servo J streaming:

```bash
eval "$(tools/vm/home_vm_env.sh --left-ip <left-vm-ip> --right-ip <right-vm-ip> --motion)"
```

Controller-simulation Cartesian:

```bash
eval "$(tools/vm/home_vm_env.sh --left-ip <left-vm-ip> --right-ip <right-vm-ip> --cartesian)"
```

The helper never sets `RB_ALLOW_REAL_CARTESIAN`.

## Recommended Sequence

1. WU-01: verify the OVA and confirm two distinct VM IPs reach `5000/5001`.
2. WU-02: run read-only rbpodo state dumps with `servo.send_servo_commands=false`.
3. WU-03: switch local VM configs to `${ROBOT_LEFT_IP}` and
   `${ROBOT_RIGHT_IP}`.
4. WU-04: run no-op Servo J streaming, then tiny controller-simulation joint
   motion with explicit motion gates.
5. WU-05: prove Cartesian commands are rejected with closed gates, then accepted
   only with controller-simulation Cartesian gates and
   `physical_motion_expected=false`.
6. Office physical work starts from a separate real-hardware acceptance plan.

## Fidelity Boundary

VM validates:

- rbpodo connection lifecycle
- fixed-port controller topology
- `CobotData` state decoding and diagnostic quirks
- Servo J dispatch, ACK policy, timing telemetry, and drops
- controller-simulation Cartesian gate behavior
- GUI, policy-runner, and recording wiring against rbpodo-shaped state

VM does not validate:

- physical TCP accuracy
- real servo dynamics
- measured latency
- GENE 26.5 physical readiness
- gripper, force control, RealSense, or measured calibration readiness

Run the guardrail check before promoting or sharing VM artifacts:

```bash
python3 scripts/check_vm_artifact_tagging.py
```
