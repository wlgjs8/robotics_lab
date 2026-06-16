# Runbook: Per-arm Direct Teaching (Free-drive) without tearing down `make run`

## What this is

Releases `servo_j` control authority on one (or both) arm's Rainbow controller via
the rbpodo SDK `set_freedrive_mode` (`freedrive_teach_on()` / `freedrive_teach_off()`)
so an operator can hand-guide the arm, then re-acquires control with a **target
resync** — all from the viser GUI, **without killing `make run`**.

## Why a state machine (the M151 fix)

Real-time servo control (`move_servo_j` streaming) and direct teaching are mutually
exclusive controller regimes. Issuing `freedrive_teach_on()` while the controller is
still executing servo motion is rejected on the pendant with **M151 — "Direct
Teaching: Cannot run this function"** (the controller's "not the right
environment/conditions" error; not M234 "motion executing" or M206 "not activated").
Direct teaching is also a **physical** gravity-compensation function: it only runs in
`operation_mode: real`. In `operation_mode: simulation` the controller refuses with
the same M151 regardless of arm state.

The fix is a per-arm **arming state machine** that quiesces the servo stream and waits
for the controller to report idle *before* engaging free-drive:

```
Off ──ON──▶ Quiesce ──idle──▶ Confirm ──is_freedrive_mode==1──▶ Active
             │ servo_j         │ teach_on()                       │ hand-guiding
             │ suppressed      │                                  │
             └── timeout / teach_on M151 ──▶ abort ──▶ Off (resync, note)

Active ──OFF──▶ Exiting ──teach_off + is_freedrive_mode==0──▶ Off (resync)
```

- **Quiesce**: the moment ON is requested, `send_policy=="freedrive"` suppresses
  `servo_j` to both controllers. The server then waits for the controller's
  `robot_state` (rbpodo `sdata.robot_state`) to reach `1` (Idle) — the prior servo
  window (`t1+t2`) lapses and the arm settles. Only then is `freedrive_teach_on()`
  issued, so M151 cannot occur. (If the controller never reports a usable motion
  state, a 150 ms settle fallback applies; a 1 s hard deadline aborts.)
- **Confirm**: engagement is verified via the controller's `is_freedrive_mode` flag
  (`set_freedrive_mode` only ACKs *receipt*, so the flag is the only ground truth). In
  `operation_mode: simulation` the flag may never flip (teach is a no-op there), so the
  ACK is trusted after a short settle; in `operation_mode: real` it MUST confirm or the
  transition aborts.
- **Abort** (timeout, M151, or cancel) returns the arm to Off, resyncs the held target
  to the current actual joints (no jump on servo resume), and surfaces a note in
  telemetry.

Primary use case (inline recovery): a policy misbehaves → release the policy lease →
direct-teach the arm to a safe pose by hand → exit direct teaching (resync) →
`InitMotion` → re-arm the policy. No process restart, no viser/camera-pipeline teardown,
no physical teach-pendant button.

## Safety posture (fail-closed)

- **Config opt-in.** `servo.allow_freedrive: true` is required. Default is `false`;
  the server rejects every `Freedrive` command otherwise. It is enabled in the
  VM-sim local config (`rb_servo_server/config/local/stack_sim.yaml`).
- **Leased.** `Freedrive` requires the command-source lease (like `ArmMotion`/`ResetFault`),
  so a stray client cannot toggle it. The GUI brackets it with Acquire/Release.
- **Global send suppression.** While *any* arm is in free-drive the server sends **no
  `servo_j` to either controller** (`send_policy == "freedrive"`). The hand-guided arm
  is gravity-compensated; the other arm holds at its last controller reference (stiff).
- **Backend backstop.** `RbpodoBackend::sendServoJ()` independently refuses to emit
  `move_servo_j` while its `freedrive_active` flag is set (returns `SuppressedByPolicy`,
  which the fault classifier treats as non-latching — not a fault).
- **Motion-pipeline bypass.** During free-drive the servo loop bypasses
  `computeServoTarget`/`applySafety` for the held target, so the hand-driven actual
  divergence cannot latch a tracking error or trip the velocity clamp.
- **Resync on exit (the key safety point).** When an arm exits free-drive the server
  snaps its held target (`*_prev_sent_q_deg_`, `prevprev`, fault-hold, output MA) to the
  current **actual** joints and clears the per-arm Cartesian latch/integrator, so
  re-acquiring `servo_j` does **not** snap the arm back to the pre-teaching pose.

E-stop, fault latch, soft-estop, collision, and self-collision paths all still latch
exactly as before; free-drive is checked *after* fault/emergency in `currentSendPolicy()`.

## Wire protocol

Per-arm, sticky server state. One-shot lifecycle command (not streamed):

```json
{ "mode": "Freedrive",
  "left":  {"mode": "Freedrive", "freedrive_on": true},
  "right": {"mode": "Hold"} }
```

- `freedrive_on: true` enters free-drive on that arm; `false` exits (+ resync).
- An arm object of `{"mode": "Hold"}` is left untouched.
- A `Freedrive` arm object missing `freedrive_on` is ignored with a warning
  (ambiguous; never silently treated as off).

State JSON exposes per-arm lifecycle stage + last abort note:

```json
"freedrive": {
  "left_active": true, "right_active": false, "any_active": true,
  "left_stage": "active", "right_stage": "off", "note": ""
}
```

`*_stage` ∈ `off | arming_quiesce | arming_confirm | active | exiting`. `*_active`
is true only at `active`. `note` carries the last abort/failure reason (e.g.
`right: freedrive aborted (teach_on failed: ...)`).

## GUI usage (viser, http://127.0.0.1:8080)

Operator tab → **직접교시 (Direct Teaching)** folder:
- `왼팔 교시 ON` / `오른팔 교시 ON` / `양팔 교시 ON` — enter free-drive on that arm
  (or both arms in one packet).
- `왼팔 교시 OFF (재동기화)` / `오른팔 교시 OFF (재동기화)` / `양팔 교시 OFF (재동기화)` — exit + resync.
- `Freedrive` status line shows the live `freedrive` state from the stream.

## VM-sim verification procedure

The VM-sim stack runs the **rbpodo backend against the Rainbow virtual control boxes**
(`run_mode: real`, `operation_mode: simulation`, `io_model: direct`), so it exercises the
real `set_freedrive_mode` command path and the server state machine end-to-end with no
physical robot. (A virtual control box may no-op `freedrive_teach_on`; the value here is
verifying command plumbing, the sticky state machine, send suppression, the resync, and
the GUI — not gravity compensation itself.)

1. Boot the virtual control boxes:
   ```bash
   make vm-up && make vm-status
   ```
2. Confirm the gate is set (already added):
   ```bash
   grep allow_freedrive rb_servo_server/config/local/stack_sim.yaml   # -> allow_freedrive: true
   ```
3. Launch the full stack:
   ```bash
   make run MODE=sim
   ```
   Open the GUI at http://127.0.0.1:8080 (keep the viser viewer up — RB motion test rule).
4. `ArmMotion` → `InitMotion` so both arms are armed and servoing.
5. In **직접교시**: click `왼팔 교시 ON`.
   - Expect: `Last action: OK: sent Freedrive left ON`; `Freedrive` line shows
     `DIRECT TEACHING — 왼팔 ON (servo_j 억제됨)`; server log `entered freedrive`.
   - Verify in the state stream: `freedrive.left_active = true`, `send_policy == "freedrive"`,
     `motion_state == ConnectedHold`, no fault latch.
6. (Optional, hardware later) move the left arm; the right arm stays put.
7. Click `왼팔 교시 OFF (재동기화)`.
   - Expect: server log `exited freedrive` + `freedrive exit resync: left ...`;
     `freedrive.left_active = false`; `send_policy` returns to `send_servo_j`; **no jump**.
8. Repeat 5–7 for the right arm, and test `양팔 교시 OFF`.
9. Re-arm: `ArmMotion` → `InitMotion`, then resume the policy.

### Negative check (fail-closed)

Set `allow_freedrive: false` (or remove it) and repeat step 5 — the command must be
rejected with server log `Freedrive command rejected: servo.allow_freedrive=false` and
the arm must not enter free-drive.

## Automated coverage

- C++ `test_arm_worker` → `testSetFreedriveUsesLifecycleQueue` (worker routes
  `SetFreedrive` to the backend, on/off payload).
- C++ `test_safety_policy` → `testFreedriveArmingQuiescesUntilIdleThenEngages`
  (no teach_on while controller reports moving + servo_j suppressed; teach_on only
  after idle; confirm via `is_freedrive_mode`; exit resync) and
  `testFreedriveTeachOnFailureAbortsAndReleases` (M151/teach_on failure aborts to off,
  arm not left engaged, note surfaced).
- C++ `test_rbpodo_backend` → `testFreedriveControllerSignalsMapped`
  (`robot_state`/`is_freedrive_mode` mapped to `controller_motion_state`/
  `controller_freedrive_on`).
- rb_gui `test_gui_contracts` → `test_build_freedrive_per_arm_packet` (per-arm packet
  shape, both-arms ON/OFF + `Freedrive` ∈ `_LEASED_MODES`).

## On real hardware

Real physical direct teaching requires:
- `operation_mode: real` (physical gravity-compensation; `simulation` always M151s), and
- the site-local config opt-in `servo.allow_freedrive: true` (set in
  `config/local/stack_real.yaml`), plus operator supervision and E-stop.

Run with `make run` (defaults to `MODE=real` → `stack_real.yaml`). The arming state
machine handles the M151 root cause (quiesce-before-teach); on the pendant you should
see no M151, and the arm goes gravity-compensated only after the GUI shows
`arming_confirm → active`. If it returns to `off` with a `note`, read the note (e.g. a
teach_on rejection or a quiesce timeout).

Both-arm ON releases both arms simultaneously — the operator must hold both (the
opposite arm does not stiff-hold when it too is in free-drive).
