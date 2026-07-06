# Runbook: dual RB3-730E physical real Cartesian circle (pgmode-real)

> ⚠️ **PHYSICAL MOTION.** `operation_mode: real` means the arms physically move. A person
> must hold the E-stop, the workspace must be clear, and both controllers must be set to
> **real** on the teach pendant. This runbook deliberately runs with the **policy-side
> real-motion safety gates removed** (PR #13) and the **controller `-2001` diagnostics
> accepted** (PR #12) — `rb_servo_server` is the **sole** real-motion safety layer.

## What this runs

A slow Cartesian circle traced by both arms:

```
synthetic UMI sender ─UDP→ policy_runner (umi_dual_cartesian, relative-init)
  ─TcpPoseTarget→ rb_servo_server (real rbpodo) ─servo_j 500Hz→ physical RB3-730E arms
                                  └─state→ viser (live viewer)
```

## Safety architecture (which real-motion gates were removed, and what remains)

- **Server (PR #12)** — `servo.allow_real_motion_with_suspect_diagnostics: true` accepts the
  vendor `-2001` (`op_stat_self_collision` / `robot_time` field-layout garbage) in real mode.
  EMS / SOS / soft-estop / `collision_occur` / unknown-mode / init-error still latch.
- **Policy (PR #13)** — the `SafetyGate` no longer blocks real Cartesian motion.
  Readiness checks still reject stale state, faults, missing camera/kinematics,
  and invalid TCP state where required. Controller-simulation safety is unchanged.
- **Server decision layer**: `cartesian_control.allow_in_real: true` (site-local config),
  `speed`/`step` clamps, `max_tracking_error_deg=10` **fault-latch**, `dq`/`ddq` limits, and the
  **URDF-capsule self-collision guard** (`clamp_to_hold`). The controller's own self-collision
  status is NOT trusted (see Known issues).

## Prerequisites

1. Controllers `.200` (left) / `.201` (right) set to **real** on the teach pendant; EMS/SOS/
   soft-estop clear; servo on; brakes released.
2. Workspace clear; **E-stop in hand**.
3. Build present: `rb_servo_server/build/rbpodo_real_gate/rb_servo_server` (configured with
   `RB_SERVO_ENABLE_RBPODO:BOOL=ON`).
4. Local config: `rb_servo_server/config/local/dual_real_rbpodo_PHYSICAL_circle_lowspeed.yaml`
   (gitignored, site-local; key values in **Config** below).

## Required config (server is run as root for RT priority)

Real motion is **config-driven, not env-gated** — the legacy `RB_ALLOW_REAL_*` /
`RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION` env gates were removed from the server runtime.
The site-local config below enables it explicitly:

```yaml
cartesian_control: { allow_in_real: true }
servo: { allow_real_motion_with_suspect_diagnostics: true }   # accepts the -2001 suspect diagnostics in real
```

(The controller-simulation carve-out `cartesian_control.allow_in_controller_simulation` /
`servo.allow_controller_simulation_motion` is **not** used here.)

## Config (key values — TUNED-1, anti-tremble)

```yaml
left_robot / right_robot:
  operation_mode: real
  servo_gain: 1.0          # do NOT lower — 0.5 caused ~5deg following lag + a right-servo deactivation latch
  servo_alpha: 0.8
  servo_t1_sec: 0.002
  speed_bar: 0.05          # NOTE: inert for servo_j (see Known issues); not a real speed cap
servo:
  allow_real_motion_with_suspect_diagnostics: true
  rbpodo_async_streaming.enable: false   # async streaming is controller-sim only; real uses synchronous direct send
safety:
  tracking_error_policy: fault_latch
  max_tracking_error_deg: 10.0
  dq_max_deg_s: [30,30,30,45,45,60]      # halved vs sim profile
  self_collision: { enable: true, margin_m: 0.05, fail_policy: clamp_to_hold }
cartesian_control:
  allow_in_real: true
  allow_in_controller_simulation: false
  path_kp_pos: 3.0                       # anti-tremble (was 6.0)
  path_kp_ori: 3.0                       # anti-tremble
  velocity_damping: 0.04                 # anti-tremble (was 0.01) — biggest lever
  max_cartesian_step_m: 0.001
```

## Launch (staged — verify each step before the next)

### 1. viser (state viewer; loads the rb3_730e URDF incl. the Pika gripper)

```bash
cd robotics_lab
PYTHONPATH=rb_gui RB_GUI_DESCRIPTIONS_DIR="$PWD/rb_servo_server/descriptions" \
  RB_GUI_STATE_BIND=0.0.0.0 RB_GUI_STATE_PORT=50366 RB_GUI_CIRCLE_OVERLAY_BIND=none \
  python3 -m rb_servo_gui.app
# → open http://<host>:8080
```

### 2. server (root, real env) — Hold first, no motion yet

```bash
cd robotics_lab
SUDO_ASKPASS=/path/to/askpass sudo -A env \
  rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --config rb_servo_server/config/local/dual_real_rbpodo_PHYSICAL_circle_lowspeed.yaml
```

Wait for `CommandServer listening on udp://127.0.0.1:50256`, then **verify (still no motion):**
`controller_mode=real`, `motion_state=ConnectedHold`, `fault_latched=false`, per-arm
`rbpodo_state_decode_policy=real_motion_suspect_diagnostics_accepted`, and
`cartesian_gate.operation_mode=real`. The arm should hold rock-stable (q_actual unchanged).

### 3. policy (existing pgmode config works in real after PR #13)

```bash
cd robotics_lab
PYTHONPATH=policy_runner \
  python3 -u -m policy_runner --config policy_runner/config/stack_real.yaml \
    --action-source umi_dual_cartesian
```

### 4. motion source — the slow circle (or the real UMI publisher)

```bash
python3 /tmp/umi_synth_sender_slow.py   # 5cm radius, 25s/rev, 120 Hz, deadman held, 600s
```

(For real UMI hardware, run the `.40` SteamVR publisher → `:50380/:50381` instead of the synthetic sender.)

## Monitor (in viser + state on udp://127.0.0.1:50356)

- `fault_latched` stays **false**; `motion_state=Running`; `command_source.source_id=policy_runner`.
- `q_actual` follows `q_sent` within ~1.5–2° (well under the 10° tracking-error latch).
- A latch is a **safe stop** (arms hold). Read the reason; to resume, restart the server (its
  initialize re-activates the servo). If the right servo stays off, re-enable it on the pendant.

## Stop

Stop the synth → policy → server (SIGTERM, graceful → arms hold under the controller brake).
Keep viser running.

## Tuning notes (trembling)

| Profile | path_kp / damping / synth | wrist q_actual HF | tracking (median) | result |
|---|---|---|---|---|
| Baseline | 6.0 / 0.01 / 50 Hz | ~0.02° | 1.69° | trembles |
| **TUNED-1** | **3.0 / 0.04 / 120 Hz** | **~0.01°** | **1.42°** | **keeper** |
| servo_gain 0.5 | (servo_gain only) | not improved | **5.19°** | **lag + servo-off latch — reverted** |

Lowering the cartesian path gain + raising `velocity_damping` + a smoother (120 Hz) reference
stream halved the wrist trembling. Lowering the controller `servo_gain` is a dead end: it adds
following lag and tripped a right-servo deactivation. Keep `servo_gain: 1.0`.

## Known issues

- **`speed_bar` is INERT for `servo_j`** — it is parsed and validated but never sent to the
  controller. The teach-pendant speed bar (≈80%) is the controller's own setting, independent of
  the config. Real-speed limiting comes from the Cartesian step/velocity limits + `dq_max` + the
  slow target, **not** `speed_bar`.
- **`-2001` diagnostics_suspect** — `op_stat_self_collision` / `robot_time` decode as garbage
  (vendor SDK↔firmware field-layout mismatch). Accepted in real via PR #12; the controller's
  self-collision status is therefore untrusted — rely on the server's URDF-capsule guard.
- **`ServoDisabled` (activation stage 0)** — the controller deactivated its servo (e.g. excessive
  following error, or an external deactivation). The server latches `RobotStateError` (correct).
  Restart to re-activate; check the pendant if it persists.
