# cm_bridge real bring-up ladder (P3)

Operator-supervised. Every step fail-closed; rollback at any point =
`./cm_bridge/run_cm_stack.sh down && make run CONTROLLER=legacy MODE=real`.

## 0. Preflight (once)

- [ ] `serial_number` in `cm_bridge/config/active.monkey.real.yaml` is COSMETIC and
      gates nothing (corrected 2026-08-18): controller-manager reads the field and
      never validates it (`Config.cpp:369`), and a boot with an empty serial passes
      its own config gate. The launcher's serial refusal was removed. What DOES stop
      a launch is an unfilled `ip` (fatal in CM) and a missing/empty tool preset —
      the 0-byte `tools/pika.yaml` that earlier looked like a serial problem.
- [ ] Boxes on firmware 26071103 (updated 2026-08-16; CM re-verifies at init and
      refuses otherwise).
- [ ] rb_servo_server stopped (script refuses otherwise — verified live).
- [ ] **Box bring-up done: `./tools/robot_on.sh`** (`pgmode real` + `mc jall init`)
      with CM DOWN (`run_cm_stack.sh down`) so the 5000 channel is exclusive.
      Verify via CM diagnostics after restart: `sim_mode: '0'`, `init_state`
      progressed, `estopped: '0'`. Skipping this yields error 202
      ServoOnTimeout at enable (observed live 2026-08-16).

## 1. Controller up, arms cold

`./cm_bridge/run_cm_stack.sh real` (or `make run CONTROLLER=cm MODE=real`) —
installs the device file into `cm_bridge/config/monkey/active.yaml`, verifies the
params directory resolves, then starts CM **natively** (no docker since
2026-08-18), the bridge, and the collision monitor. Arms are NOT energized.
The launcher blocks until the `[TASKCFG]` banner appears and prints it. Check:

- [ ] `logs/cm_controller_<stamp>.log`: firmware/config gate passed, both arms
      `Init -> Disabled`, `operation mode = REAL`.
- [ ] the printed `[TASKCFG] … follow{…}` line reads `vmax=100mm/s wmax=20dps
      a=707.1 T=33.4ms` — the banner is the ONLY ground truth for which params
      file is live.
- [ ] servo_state.v1 flowing (rb_gui or port 50378), q matches the pendant.

## 2. FK cross-check (before any motion)

- [ ] Read `left/right q` from the fanout; compute rb_servo_server FK TCP for
      the same q (offline) and compare with CM `cmd/pose` (`tcp_command_stand`).
      Must agree < 1 mm / < 0.1 deg. This validates the pika tool preset
      (`cm_bridge/config/monkey/params-presets/tools/pika.yaml`, z 202.642 from
      the SRO; the boot banner prints the composed `TCP synced -> xyz=[0,0,247.6]mm`)
      and the mount matrices
      (base_frame == stand). Mismatch -> STOP, reconcile before energizing.

## 3. Energize + idle (operator console)

- [ ] Console = `source submodules/controller-manager/platforms/monkey/scripts/env.sh`
      then `ros2 service call /monkey/cell/cmd/console cell_msgs/srv/Command
      "{command: <cmd>}"` (was `docker exec monkey-real …` before the native move).
- [ ] Canonical console sequence (LIVE-VERIFIED 2026-08-16; `disable` drops
      the box back to SIMULATION, and `reset` — param reload — only works in
      Disabled):
        `enable` (13s) -> **`mode real`** -> `task on` -> per-arm `task idle`.
      Watch: no faults, qsync fill converges ~5. Param retune cycle =
      `disable` -> `reset` -> `enable` -> `mode real` -> `task on` -> idle.
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
LPF-off on 26071103). And, from the 2026-08-18 envelope decision:

- **rotation is capped at 20 deg/s** (`max_rot_dps`, the 100 mm/s preset's
  trans<->rot time-scale rule). The task's own yaw maneuvers were measured at
  ~64 deg/s, so oversize rotation deltas WILL be cut — and FollowUnit's
  `OVERSIZED` WARN fires only ONCE per episode (`cap_warned_`), so count it on
  the bridge side rather than trusting the log.
- **the force gate is armed at compiled defaults** (25 N / 3.5 Nm, close 0.1 s
  / open 0.4 s — confirmed on the boot banner). Under contact the stream is
  attenuated INTO the wrench direction only. legacy had no equivalent.
