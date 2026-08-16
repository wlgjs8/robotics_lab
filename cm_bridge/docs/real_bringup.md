# cm_bridge real bring-up ladder (P3)

Operator-supervised. Every step fail-closed; rollback at any point =
`./cm_bridge/run_cm_stack.sh down && make run CONTROLLER=legacy MODE=real`.

## 0. Preflight (once)

- [ ] Fill both `serial_number` fields in `cm_bridge/config/active.monkey.real.yaml`
      from the arm nameplates (`run real` refuses while blank).
- [ ] Boxes on firmware 26071103 (updated 2026-08-16; CM re-verifies at init and
      refuses otherwise).
- [ ] rb_servo_server stopped (script refuses otherwise — verified live).
- [ ] **Box bring-up done: `./tools/robot_on.sh`** (`pgmode real` + `mc jall init`)
      with CM DOWN (`run_cm_stack.sh down`) so the 5000 channel is exclusive.
      Verify via CM diagnostics after restart: `sim_mode: '0'`, `init_state`
      progressed, `estopped: '0'`. Skipping this yields error 202
      ServoOnTimeout at enable (observed live 2026-08-16).

## 1. Controller up, arms cold

`make run MODE=real` (CONTROLLER=cm default) — installs the device file,
starts CM (monkey-real), the bridge, and the collision monitor. Arms are NOT
energized. Check:

- [ ] `docker logs monkey-real`: firmware/config gate passed, both arms
      `Init -> Disabled`, `operation mode = REAL`.
- [ ] servo_state.v1 flowing (rb_gui or port 50378), q matches the pendant.

## 2. FK cross-check (before any motion)

- [ ] Read `left/right q` from the fanout; compute rb_servo_server FK TCP for
      the same q (offline) and compare with CM `cmd/pose` (`tcp_command_stand`).
      Must agree < 1 mm / < 0.1 deg. This validates the pika tool preset
      (`tools.pika.yaml` z 202.642 from SRO) and the mount matrices
      (base_frame == stand). Mismatch -> STOP, reconcile before energizing.

## 3. Energize + idle (operator console)

- [ ] `enable` -> both arms Enabled (right first by design), `task on`,
      per-arm `task idle`. Watch: no faults, qsync fill converges ~5.
- [ ] Deadman/E-stop within reach from here on.

## 4. First motion — single arm, MOVJ

- [ ] Send a small reset MOVJ (±5 deg) via the bridge (UDP 50256 JointTarget).
      Confirm motion, direction, speed; collision monitor CLEAR throughout.

## 5. Single-arm low-envelope follow

- [ ] Stream a short slow synthetic chunk profile (gate-style, vy<=10 mm/s)
      to ONE arm. Watch tracking, no OVERSIZED warnings, clean silence exit.
- [ ] Repeat with the other arm, then both.

## 6. Force path validation (required before force-armed contact work)

- [ ] `ft identify` (CM tool calibration) per arm with the pika tool; enter
      results in the device file `tool_calibrations`. Until then the
      compensated wrench carries the tool weight as bias — admittance is armed
      (follow overlay) but MUST NOT be trusted for contact tasks.
- [ ] After identify: gentle hand-push test on the tool — deviation within the
      fence (40 mm / 15 deg), clean return.

## 7. Collision drill (SILS-proven chain, live confirmation)

- [ ] With arms in free space, manually trip the monitor (lower a margin via a
      test flag or drive a slow approach toward the stand) — confirm: follow
      stream drops, fault_latched on the fanout, arms brake to rest via
      FollowUnit silence. `collision_clear` + re-idle recovers.

## 8. Policy rollout A/B

- [ ] flow-infer real_policy against the bridge state/chunk path; same task as
      the legacy baseline; compare with `tools/analyze_rtc_ab.py` + success.

Known deltas vs legacy to watch: no floor-plane constraint yet (bridge gate
only guards self/stand collision), gripper distal contact rides force control
(step 6 gate), servo params live in CM (t1 2ms/t2 21ms/gain 1/alpha 10
LPF-off on 26071103).
