# AGENTS.md

## Current Project Phase

`robotics_lab` is a dual-arm RB5-850 integration workspace (RB3-730 until 2026-09-02). The current milestone is **rbpodo pgmode-real physical robot bring-up**. Simulator-first Cartesian acceptance hardening is largely complete and is now the regression baseline.

The mock / rbpodo controller-simulation (pgmode) stack remains the regression baseline (it must keep passing before any physical work):

- structured backend result and fault telemetry
- joint commands: `JointTarget`
- Cartesian point-to-point: `TcpPoseTarget`
- Cartesian path tracking: `TcpLinearMove`
- GUI and policy-runner safety gates
- command-source lease/arbitration

Real motion is now an active bring-up lane: read-only diagnostics parity, a slow dual-arm physical Cartesian circle, UMI teleop/replay, and a full `flow-infer` `real_policy` closed-loop rollout (pi0.5/openpi, `TcpPoseTarget` + real gripper) have all run on hardware under operator supervision (`docs/runbooks/rbpodo_real_physical_circle.md`, ladder `docs/runbooks/pgmode_real_transition.md`). `flow-infer` composes ee_local deltas into absolute `TcpPoseTarget` setpoints. The `real_policy` rollout-mode validation was satisfied via accepted/validated config — the lane is open, not blocked; runtime is validated and task success is the remaining model-side gap. Real-motion execution authority is config-driven and server-owned: tracked stack config plus the mode-independent safety layers decide whether motion is sent. Operator supervision and an E-stop remain physical operation procedure, and passing simulator tests is never permission to move hardware. For real Cartesian motion the policy-side block was retired (PR #13), so `rb_servo_server` makes the final allow/deny decision (plus the async URDF-mesh `CollisionMonitor`). Project-native force control v2 is live and hardware-validated against `controller-manager`; its measured configuration, coverage/tare preconditions, gate, spring, and deviation fences remain load-bearing safety contracts. Measured hand-eye calibration is unneeded for the deployed pika ee_local image-conditioned policy but still required for general geometry-dependent policy.

## Required Reading

Always read the current source-of-truth docs before editing code:

- `README.md`
- `REVIEW.md`, if present
- `docs/architecture.md`
- `docs/current_review.md`, if present
- `docs/servo_backend_contract.md`
- `docs/frame_contract.md`
- `docs/joint_range_policy.md`
- `docs/hardware_free_validation.md`
- the component README/docs for the module being changed

Historical prompt/planning files such as `TODO.md`, `CODEX_*PROMPTS*.md`, `MIG-*`, `HARDEN-*`, and `CART-HARDEN-*` notes are audit context only unless a task explicitly names them. When those files conflict with the current source-of-truth docs, the current source-of-truth docs win.

## Canonical Public Terminology

Use these values in config, docs, GUI labels, logs, and tests:

```yaml
run_mode: mock | simulation | real
backend_type: mock | rbpodo
```

Do not introduce removed simulator backend aliases or mixed simulator terms.
`run_mode: simulation` now refers only to the rbpodo controller `pgmode`
simulation flavor.

Supported real-controller scope is rbpodo only. The `MockBackend` remains for
hardware-free validation; the retired software-simulator backend and raw script
TCP comparison backends are no longer part of the active code, config, gate, or
runbook surface.

The supported J3/elbow range is the fitted arm's catalog range: exactly
`[-165 deg, +165 deg]` on the RB5-850E in service since 2026-09-02 (`[-150, +150]`
on the RB3-730E it replaced), matching the Rainbow documentation and the Pinocchio
URDF. Tracked safety limits, joint-limit barriers, IK, examples, and runbooks must
use that same range. Do not restore the retired `+/-160 deg` margin or widen J3 to
hide an unreachable Cartesian target.

## Target Topology

Physical robot topology:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

The rbpodo controller `pgmode` simulation topology mirrors this controller shape
(one rbpodo endpoint per arm), targeting either a Virtual ControlBox VM or a
physical box held in `pgmode`. The tracked `stack_real.yaml` and
`stack_sim.yaml` files are the only launch configs.

## Safety Boundary

Never enable real robot behavior implicitly. Real behavior is fail-closed, but
it is **no longer gated on env vars**: the legacy execution gates
(`RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`, `RB_ALLOW_REAL_CARTESIAN`,
`RB_ALLOW_RBPODO_ACK_DISABLED_MOTION`, and the other `RB_ALLOW_*`) were removed
from the server runtime. `run_mode`/`operation_mode` are telemetry labels only
and do not decide whether motion is allowed.

Real-motion execution authority is owned by **the tracked stack config + the
mode-independent safety layers**. Real motion requires `stack_real.yaml` to
enable it explicitly (e.g.
`cartesian_control.allow_in_real: true`). Real-hardware acceptance and operator
supervision are physical operation process, not extra software gates. The
controller `-2001` suspect-diagnostics acceptance and the rbpodo `pgmode`
controller-simulation carve-out are likewise config opt-ins (no env). Simulator
acceptance is not real-hardware acceptance.

The stand-frame floor plane constraint (`safety.floor_constraint`) is
mode-independent by design: when enabled it applies in mock,
controller-simulation, and real, to every motion primitive, at the final
joint-level safety gate. Enabling it requires `kinematics.enable=true`.
Runtime adjustment uses the leaseless `SetSafetyFloorZ` command and is
bounded server-side to the config `[runtime_min_z_m, runtime_max_z_m]`
envelope; `monitor_only: true` is a tuning aid only and never a real-motion
safety posture. Do not add env/mode gates that would disable it in real mode.

For new rbpodo configs, use canonical Rainbow Servo J fields only:
`servo_t1_sec`, `servo_t2_sec`, `servo_gain`, and `servo_alpha`.
Do not add new uses of deprecated aliases `servo_time_sec`,
`servo_lookahead_sec`, or `servo_acc`. `servo_t1_sec` must match the streaming
command period for supported real/controller-simulation configs: `0.002` at
500 Hz. Manual non-500 YAML overrides may remain parseable for compatibility,
but they are not supported profiles. ACK-off rbpodo settings are diagnostic
evidence only until a future real-motion task promotes them.

Tracked stack configs are the launch source of truth:

```text
rb_servo_server/config/stack_real.yaml
rb_servo_server/config/stack_sim.yaml
```

Do not create `config/local` launch variants. Change one reviewed setting at a
time in the appropriate tracked stack config so the effective runtime profile
remains visible and auditable.

## Force Control

ONE LAW, ONE STAGE, SINCE 2026-09-15 (operator: "hold / stream 구분 없이, 더 범용적인
force control"). There is no stream law, no hold law, no declared press axis and no
hand-guide latch. The law acts on the TRANSLATION VECTOR in the stand frame:

```
m * v' + b * v = (|F| - rest_force_n)+ * F_hat        k = 0, rotation RIGID
```

`F` is the compensated, PRE-deadzone physical force at the TCP (low-passed by
`force_control.wrench_filter_hz`; real 0.0 = raw, sim 25.0). At or below
`rest_force_n` NOTHING moves, in any direction, so free space is never sought and a
hand or a stopped press rests there. Above it the arm yields ALONG the measured
force at `(|F| - rest)/b` (24 N -> 8 mm/s, 30 N -> 20 mm/s at the 20/24 N pair shipped since 2026-09-15 pm; 10/12 before) and STAYS where it was
dragged: the fold books the deviation into the source's plan every tick, so plan ==
arm (yield-and-stay; a spring that RETURNS the arm was tried 2026-09-04/10 and
rejected). Per-axis one-sidedness was rejected too: a diagonal 10 N push would read
5.8 N per axis and never cross a per-axis rest force.

**Config (`stack_real.yaml` / `stack_sim.yaml`):** `force_control.law: { translation:
{m: 20.0}, rotation: rigid }` and `force_gate: {enable, peak_force_n 12, rest_force_n
10, peak_vel_mm_s 4, close_tau_s 0.10, open_tau_s 0.40}`. `b` is DERIVED =
`(peak_force_n - rest_force_n)/peak_vel_mm_s` = 2 N / 4 mm/s = 500 N*s/m and REFUSED
if typed. Unchanged keys: `max_deviation_m` 0.040 (a dead backstop that only the
absolute source can reach), the rate caps, `wrench_filter_hz`, the oscillation guard,
`coverage_recover_sec`, `hold_latch_max_command_gap_m`, `command_execution_*`,
`max_state_age_sec`. DELETED KEYS ARE REFUSED, NOT IGNORED: `stream`, `hold`,
`hold_compliance`, `hold_engage_force_n`, `hold_release_force_n`,
`hold_relatch_max_force_n`, `fold_deviation` fail the load with "was DELETED on
2026-09-15 ..."; `law.translation.{b,k,mode,ref_force}` fail with "may not be typed
(2026-09-15)"; `law.rotation` must be the literal `rigid`; the 2026-09-11 deletions
(`force_gate.max_force_n`, `max_torque_nm`, the `stream_*` channel) stay refused.

**The gate (CM 0049's curve, unchanged):** `g = (v_cross/v_s)^((|F|/peak_force_n)^2)`
with `v_cross = peak_vel_mm_s`, judged on |F| of the PHYSICAL vector, `v_s` = the
SOURCE's demanded advance only (never the achieved speed, never the law's own yield),
direction = `F_hat`. At `peak_force_n` the gate's speed IS the law's yield, so a
streamed contact converges at `peak_force_n` (24 N since 2026-09-15 pm) for every demand (closed-loop model at the 12 N pair: 12.00 N at
30/60/150 mm/s, 0.00 N p-p). Every consumer — the chunk follower
(`CartesianChunkFollower::setAdvanceGate`), the pose-track SMD
(`SmdPoseTracker::constrainTranslation`) and the preview QP
(`followerPreviewContactAuthority`) — cuts ONLY the into-contact component
(`advance . F_hat < 0`) along the ONE `contact_normal` the servo loop publishes
(`ForceGate::contactNormal()`: `F_hat` when |F| > 0.5 N, else zero; `+F_hat` is the
free-space direction). Force-reducing motion is always free.

**Sources (`ForceSourceKind {None, ChunkFollower, HoldPose, AbsoluteTarget}`), one per
arm per tick:** `chunk_follower` — demand = plan advance, fold books into the pending
plan fold; `hold` — a Hold under force control is ALWAYS a zero-demand source: latched
at the last COMMANDED TCP once the motion generator is at rest, promoted to a
`compliant_hold` TcpPoseTarget, routed to a hold-source stage that BYPASSES the
pose-track SMD, the fold moves the hold pose, walled at ROI/floor; `absolute` — UMI
teleop's absolute TcpPoseTarget: demand = raw target step/dt, fold DECLINED, fence
live. `DualArmServoLoop::publishAdvanceGate` is the single writer of the follower's
(gate, direction) slot; the ROI/floor fold owns it at gate 0 while it stands.

**Telemetry.** JSON `force_control`: `source` ("chunk_follower" | "hold" | "absolute" |
"none"), `source_demand_m_s`, `contact_normal_stand[3]`; REMOVED `law`,
`gate_rotation`, `gate_stream_speed_m_s`, `gate_wrench_norm_n`, `hold_engaged`,
`hold_force_n`. CSV: ADDED `<side>_fc_source`, `_fc_source_demand_m_s`,
`_fc_contact_normal_{x,y,z}`; REMOVED `_fc_law`, `_fc_gate_rotation`,
`_fc_gate_stream_speed_m_s`, `_fc_gate_wrench_norm_n`, all eleven `_smd_gate_*`,
`_fc_hold_engaged`, `_fc_hold_force_n`. `_fc_fold_sink` is "chunk_follower", "hold" or
"declined: ...". Kept: `_fc_gate_translation`, `_fc_gate_force_n` (|F| physical,
filtered), `_fc_gate_closed`, `_fc_gate_{b_eff,m_eff,cross_speed_m_s,rest_force_n,
peak_force_n}`, `_fc_dev_*`, `_fc_vel_*`, `_fc_folded`, `_fc_absorbed_*`,
`_fc_bounded`, `_fc_osc_*`, `_fc_wrench_filt_*`, `_fc_covered`,
`_fc_coverage_reason`, `_fc_reference_*`.

**Known limit (closed-loop model, kept as a test):** the projective cut leaves the
advance component perpendicular to `F_hat` alone, so under FRICTION the normal force
settles above the declaration: mu 0.3 -> ~12.3 N at 30 mm/s, ~15.8 N at 150 mm/s; a
constant 20 N lateral load at 100 mm/s -> ~26.5 N normal (and the arm yields sideways
to the 20 N). Deadlock ("gate shut, law at rest") is structurally impossible: the law
always has a yield direction. The impact PEAK `v*sqrt(k_env*m)` is bounded by nothing
here (39.5 N measured at a 115 mm/s approach); 10 N is the steady state.

CM is the reference. Sensor axes, tool mass/COM and the TCP offset come from
`submodules/controller-manager/platforms/monkey/params-presets/` and were calibrated
by the operator; do not re-derive them from the URDF. The sensor basis on this cell is
LEFT-HANDED (det = -1) — measured, not a bug. The runtime F/T adapter uses valid,
finite MEASURED joints from the wrench's RobotState for flange rotation and gravity
(an in-flight sent target is not the sensor pose). `RB_SERVO_BUILD_FORCE_EXPERIMENTS`
builds an OFFLINE-only reference candidate that is not linked to the runtime and
failed robustness qualification (`docs/reference/force_reference_development.md`).
The requirement of record (2026-09-11): transients above 10 N may occur, sustained
contact may not stay at or above it; a 20 N setting is not promoted.

Two loader invariants remain, both taught by hardware, plus one design invariant:
- THE GATE PAIR IS REQUIRED AND DERIVES b. `peak_force_n`, `rest_force_n` and
  `peak_vel_mm_s` must all be declared (else "law.b was not derived"); a damper with
  no gate ramps force with the plan, a gate with no rest point converges to 0 N.
- `law.translation.m >= 2*b*dt` (dt = 0.002) else REFUSED — the semi-implicit Euler
  step diverges above it. It used to be a silent raise of m; a number the operator
  predicts the robot from may not move on its own.
- (Still true) THE WRENCH REFERENCE POINT AND THE COMPOSE PIVOT MOVE TOGETHER: both
  are the TCP, or a straight push twists the tool.

**History (measured numbers kept; full text in git before 2026-09-15):**
- 08-26 v1 wiped; v2 rebuilt on CM: deviation tracked F/k to 0.97-0.99, 1.85 deg
  at 55 N, zero fence hits. 08-27 wrench low-pass + oscillation guard after a hand
  push pinned the 40 mm fence and rang ~5.3 Hz to E-stop (98.6 N swings).
- 09-03 k = 0 + fold like CM (k = 0 without the fold walked 9.5 m; k > 0 without
  the gate hit 961 N); rotation rigid (every unwanted degree came from 1-5 N at the
  F/T housing); hand-guide latch 5/2 N (38 crawls in 228 s at the 2 N deadzone).
- 09-04 stream spring k 400 tried; gate on physical |F|; Hold latched at the
  COMMAND, not the measured TCP (5-11k deg/s2 pedal kicks).
- 09-10 spring reverted (it RETURNS the arm: 37 mm out, back at 238 mm/s; the
  requirement is yield-and-stay); complete hold-back + plan-clock leash; two
  position clamps built and removed (-17.7/+9.6 m/s2 steps, 380 ms hold-off).
- 09-11 CM 0049 pair adopted (the old gate faded to ZERO at `max_force_n`, a pure
  damper's only equilibrium: 28 N -> 0.1 N measured); press axis declared (tool z,
  `mode: force`); 52 Hz direction switch at the rest threshold fixed (276
  transitions in 5.3 s, 3,216 deg/s2); gate judged on the press component (on |F|
  a 20 N lateral load converged at 10 N with gate 0.0000); one-sided row made
  foldable (a push pinned the fence 3,606 ticks); contact clamp DELETED (91-100 %
  of |command accel| > 10 m/s2 within 16 ms of a clamp; after: 13.3 -> 3.6 mm/s
  RMS, max 67 -> 12.6 m/s2); `open_tau_s` 0.15 rang (+50 % ripple) -> 0.40.
- 09-15 the latch was fed the press component instead of |F| (ba89b399): a
  20-40 N lateral push staircased, 31 toggles, release kicks 1,000-3,280 deg/s2 ->
  latch off; the compliant Hold's own yield was read back as stream demand (SMD
  goal-rate, corr 0.997 with the overlay velocity) and closed the gate to 0.095 in
  a hand push (`servo_log_20260915_112545`); tool x/y ungated, so a hand on the
  gripper mid-chunk saw F = b*v_plan (50 N at 100 mm/s); the fold declined on the
  press row and pinned the fence. -> THE UNIFICATION ABOVE.

**Hardware acceptance (in order; read the columns with
`rb_servo_server/tools/analyze_force_stage.py LOG --arm both`):**
1. InitMotion + auto-tare on both arms -> `_fc_covered` 1, `_fc_source` hold,
   `_fc_gate_translation` 1.00, `_fc_source_demand_m_s` 0, `_fc_vel_*` 0.
2. Hand-push each arm to the floor and hold: `_fc_gate_force_n` settles 9-11.5 N,
   `_fc_vel_*` -> 0, `_fc_fold_sink` hold; release -> the arm STAYS (command drift
   < 1 mm over 5 s), no re-approach.
3. Lateral and diagonal pushes > 10 N: the arm yields at `(|F| - 10)/500` along
   `_fc_contact_normal_*` and stays; 5-8 N moves nothing.
4. Policy run: grab the tool for 1-2 s, twice. The plan must NOT stop
   (`_fc_source` stays chunk_follower, `_fc_fold_sink` chunk_follower, no
   `braking_expired` / recovery), the arm yields and stays, `fault_latched` 0,
   max |`_q_sent_accel_deg_s2_*`| < 1,500, `_fc_bounded` 0.

**2026-09-15 (evening) — TOOL INERTIA IS COMPENSATED, FROM THE COMMANDED TRAJECTORY.**
The first policy run under the one law (`servo_log_20260915_153420`, hardware steps
1-3 of the acceptance passed, step 4 faulted in 0.25 s) showed what the press-axis
design had been hiding: the compensated |F| tracked the tool's OWN acceleration with
|F| / (m·|a_TCP|) = 1.0 and cos 0.85 on both arms (m 0.81 kg), so the policy's
12 m/s² start read as 12 N of contact, the law yielded and the gate closed against it
(0.27-0.36 at 80-218 mm/s demand), the executor answered with more acceleration, and
the loop ran to 30-100 N, 15,000-18,500 deg/s² and `accepted_deviation`. No hand was
on either tool (|F| < 1.5 N until the plan moved); the preview QP rejected nothing. The
box compensates neither weight nor inertia, so gravity now generalises to `m·(g − a)`
in `FtPipeline` (`force_torque.inertia_compensation`, `*_ft_inertial_sensor_*`
logged beside gravity). `a` is the COMMANDED tool-COM acceleration — one FK per tick
into a ring, central second difference read `command_delay_ticks` (14 ≈ 28 ms; the
lag scan on the same run peaks at 14-16 ticks) back; the measured joints are too noisy
to differentiate (27 N p99 from a 9-sample fit, 2026-09-11). Above `max_accel_m_s2`
(30) the term is refused and flagged, and the ring is cleared on every command-chain
break (init reset, freedrive resync). A model term like gravity, not a rule. Residual
in the violent regime is still ~half (the box's servo overshoots the commanded
acceleration ~2× there), but the loop cannot START from a compensated 12 → ~4 N.
Unverified on hardware at the time of writing.

**2026-09-15 (night) — THE FORCE THE SENSOR READS WHILE THE ARM MOVES IS THE ARM.**
Second policy run under the one law, inertia compensation live
(`servo_log_20260915_155708`): the compensation held (valid on 97-100 % of ticks,
4-24 N removed) and the start was gentle (0.8 m/s²), yet 150 ms in both executors ran
their commands to 3-5× the follower reference (250-540 mm/s against 30-160, lead 32 mm,
initial acceleration pinned at the 12 m/s² tracker limit) and the loop closed again
without a fault. The force-VECTOR spectrum in the vibration was 68-78 % in 15-60 Hz
(peaks 16-24 Hz), 0 % below 4 Hz: the 0.8 kg pika on its 200 mm lever ringing to the
executor's re-plans, |F| p50 20-27 N with nothing touched, the tare valid (0.6 N before
the plan moved). Command acceleration led force by ~48 ms (r 0.34) and force led command
acceleration by ~35 ms (r 0.35): an ~80 ms round trip, the 6-12 Hz in every spectrum. At
v_cross/v_s ≈ 0.03 the curve read g(5 N) = 0.55, g(8 N) = 0.22 — half the plan cut for a
force that was the robot shaking. Two config levers shipped together (operator): the
wrench VECTOR the law and gate read is low-passed at `wrench_filter_hz` 3 Hz (replayed,
the same vector reads p50 4 N / p90 7-11 N; a low-pass on |F| would have rectified the
ringing into DC instead), and the pair moved to `rest_force_n` 20 / `peak_force_n` 24 /
`peak_vel_mm_s` 8 (b stays 500, v_cross doubles to 8 mm/s, g(5 N) → 0.87). Not yet run on
hardware. The executor's overshoot of its own reference is the loop's energy source and
is the 2026-09-10 leash pathology, still open.

**2026-09-15 (night, second run) — TWO INPUTS, TWO BANDWIDTHS.** With the 3 Hz vector
and the 20/24 N pair the free-space loop was gone (`servo_log_20260915_161142` /
`_161234`), and the contact "oscillation" that remained was the force stage doing
NOTHING: filtered |F| <= 14.5 N, yield 0 on every tick, gate >= 0.87, while the preview
executor ran 33-48 mm ahead of the follower reference (`plan_lead_m` 0.6 -> 47.7 mm in
0.25 s, initial acceleration pinned at 12 m/s^2) into the objects and the raw force
spiked 40-84 N in few-ms bursts (22 % of ticks < 2 N, 14 % > 40 N) - a 3 Hz pole
averages that to 2-14 N. The right arm meanwhile bobbed at 2.2 Hz on a geometry wall
(`RoiViolation` 74 ticks, 31 geometry-hold folds, |F| < 6 N; the tracked ROI floor is
-0.330 m and the violation was at z -0.296, so a runtime ROI/floor or a reach/floor row).
The law and the gate therefore read DIFFERENT poles of the same physical vector: the
GATE keeps `wrench_filter_hz` 3 Hz (a plan cut is a sustained decision and must not
answer ringing), the LAW gets `law_filter_hz` 25 Hz (the 2026-08-27 shock filter: a
real impact yields within ~6 ms; `rest_force_n` 20 N, not the filter, keeps the
5-15 N ringing out of the yield). The loader refuses a law pole below the gate's.
Logged as `*_fc_wrench_filt_*` (law) beside `*_fc_gate_wrench_filt_*` (gate). The
hand-press "0 N": the law is one-sided, so a resting contact sits anywhere in 0-20 N
and only a STREAMED press converges at 24 N; and a hand on the gripper is distal to
the sensor, so the floor's reaction cancels it there (measured 9.5-13 N net while
pressed to the floor). Push the arm above the sensor to see the floor. The executor
lead runaway and the geometry-wall bounce are not force control and stay open.

**2026-09-15 (night) — THE PLAN-LEAD LEASH MOVED ONTO THE EXECUTOR'S CLOCK.** The
preview executor's target is never the follower's current pose: each replan clones the
live follower and rolls it 250 ms forward (`follower_preview_reference.hpp`), and the
`reference_trust` continuation is anchored on that rollout at +130 ms, so with a chunk
whose travel sits in its later rows the command legitimately runs ahead of the source.
The 2026-09-10 leash bounded that lead by slowing the FOLLOWER's knot clock — which is
the pose the lead is measured FROM, while the command clock stayed wall time: the
lead grew by exactly the plan time the gate removed (0.6 → 47.7 mm in 0.25 s with the
reference standing still, `servo_log_20260915_161234`), and `executor_waits` froze the
source to 0.0 outright during a brake while the lead measurement went blind. Now
`planLeashGate(plan_lead_m)` drives `LivePreviewExecution::setPlanClockGate` and the
follower's `setPlanRateGate` carries only the geometry gate: the active plan is sampled
at `gate × wall time` (`plan_time_`), the source keeps its clock, so the reference
catches the command up and the lead closes. The gate is slewed ≤ 0.05/tick (a 1.0 ↔
0.25 swing in 30 ms = 7.5 m/s² pseudo-acceleration, under the tracker's 12), the worker
splices the predecessor at the plan time the executor predicts for the splice instant
through that same slew (`PreviewExecutionRequest::predecessor_sample_time_sec`,
echoed as `spliced_predecessor_time_sec`), admission waits a tick or two if the
predecessor has not reached that point (never a forward step), and the lead is measured
through brakes too. Published as `*_preview_execution_plan_clock_gate`. Regression:
`planClockGateDilatesTheCommandAndKeepsSplicesC2` (test_live_preview_execution).
Still open, next: re-anchor the rollout/trust continuation on the follower's CURRENT
pose (`preview_execution_worker.cpp` knot fill, `preview_trajectory_tracker.cpp`
trust anchor). Unverified on hardware at the time of writing.

### The zero (tare)

Force control REFUSES to cover an arm with no bias (`forceControlCovered`), so the
tare is a precondition, not a nicety. Two ways in, one mechanism: the leaseless
`TareForceSensor` command (the GUI button), and
`force_torque.auto_tare_after_init_motion`. Both end in the same RT path — 250
consecutive ticks of `raw - gravity` averaged by `FtPipeline::tareSample/tareCommit`.
It averages `raw - gravity`, NEVER `raw`: the box is told a zero payload, so raw
still carries the tool's weight and averaging it would subtract gravity twice.

**Automatic tare on InitMotion does not sample when the move starts — it ARMS when
the move starts.** A tare averaged while the arm is accelerating records the arm's
own acceleration and the tool's swing as force, and nothing downstream can tell that
apart from a real bias. So the request tick arms it (`armAutoTareAfterInit`, called
from `applyInitMotionSequencer`'s fresh-request branch) and the samples are collected
only after that arm's sequencer reaches `Done`, has stood `settle_sec` at the init
pose, and its last SENT joint velocity is under `max_sent_speed_deg_s`
(`stepAutoTareAfterInit`, whose per-tick decision is the stateless
`stepAutoTareDecision` — unit-tested in `test_init_motion_pursuit`). It runs BEFORE
`applyInitMotionSequencer` in the tick so the `Done` set on the previous tick is still
readable; the next non-init command resets the exec to `Idle`.

Fail-closed, all of it: `invalidate_on_request` drops the existing zero the instant
InitMotion is requested, so an InitMotion that fails, stalls, is cancelled, or is
overtaken by a latched fault leaves the arm with NO zero and force control refuses it,
rather than leaving a stale zero the operator believes was just refreshed. The loader
refuses `enable: true` without a positive `settle_sec` and `max_sent_speed_deg_s`, and
refuses it with `safety.init_motion_planner.enable: false` (InitMotion then degrades
to a plain JointTarget with no completion event, so the tare would silently never
fire). Stage is published as `force_torque.<arm>.auto_tare_stage` and logged as
`<side>_ft_auto_tare_stage`.

THE OPERATOR'S CHECK IS UNCHANGED AND IS NOT WEAKER BECAUSE IT IS AUTOMATIC: whatever
load stands at the init pose becomes the new zero. Neither the GUI nor the server can
see a part in the gripper or a hand on the wrist.

Do not integrate an external force-control library into an active motion path
unless a task explicitly says to. Archived v1 design and evidence:
`docs/archive/force_control_v1/`.

## Motion Primitive Contract

Do not blur these modes.

### JointTarget

Absolute joint-space target. This is a joint-space point-to-point primitive.

### TcpPoseTarget

Cartesian point-to-point final-pose target. It is MoveJ-like in the sense that the final TCP pose is targeted, but the intermediate TCP path is not guaranteed linear and TCP orientation may vary along the joint-space path.

### TcpLinearMove

MoveL-like Cartesian path primitive. It has explicit timing/speed semantics and orientation interpolation semantics (`constant`/`slerp`). Real-motion-ready and used on the physical arms (the run-mode execution gate was retired; it computes in every run mode). It is a finite, bounded path (`linear_move.max_duration_sec`): once started it drives to completion from a single command even if the command's freshness/lease lapses, so one click always reaches the target; an explicit command-mode change, fault, or E-stop aborts it, and the per-tick safety gate still applies.

## Backend Contract

Do not reintroduce bool-only backend operations.

Backend APIs must preserve structured results:

- `BackendResult<RobotState>`
- `SendServoJResult`
- `BackendErrorKind`
- `BackendTiming`
- `FaultContext`

Do not parse backend error strings to infer safety behavior if structured fields are available. Mock and rbpodo backends should both map failures to the shared backend taxonomy.

Unsupported raw script TCP comparison paths must not be reintroduced. Rbpodo is
the only supported real backend; the mock path remains the hardware-free test
surface.

## Servo Loop And I/O Architecture

Target architecture:

```text
CommandBuffer
  -> ServoCoordinator / DualArmServoLoop
       -> Left ArmWorker  -> one left backend/controller endpoint
       -> Right ArmWorker -> one right backend/controller endpoint
```

The servo loop owns command freshness, lifecycle, target generation, FK/IK, Cartesian planning, safety filtering, fault classification, and dual-arm aggregation. Blocking backend I/O should live behind backend/worker boundaries and must produce structured result telemetry.

`servo.io_model: worker` is a supported real-mode path. The mock-only refusal was
retired: control-box queue sync needs each arm to own its own send cadence, and a
single loop has one period for two boxes running two different clocks.

## Calibration And Frames

Use `docs/frame_contract.md` and `calibration/active_calibration.yaml` as the frame/geometry source of truth. Current calibration is `configured_estimate`, not measured calibration. Joint-only control can run without measured calibration. Real geometry-dependent policy and real Cartesian camera-driven behavior require measured and accepted calibration.

## Development Rules

- Work only on the assigned task.
- Prefer small, reviewable changes.
- Keep configs explicit about what they enable and what safety layer owns it.
- Update tests, docs, and acceptance scripts together with behavior changes.
- Do not fake external APIs for rbpodo, Pinocchio, RealSense, SpaceMouse, or camera devices.
- If a dependency is missing, report it clearly and do not claim the gate passed.
- Do not claim C++ or Pinocchio runtime acceptance passed unless the command was actually run.
- Do not introduce custom SO(3), SE(3), quaternion interpolation, or frame-conversion math in production Cartesian control when Eigen/Pinocchio can provide it.
- Do not create production fallback math paths that bypass mandatory Eigen/Pinocchio Cartesian math.
- No silent fallback defaults for safety-relevant parameters. A value that bounds motion, contact, a tolerance, a geometry/frame, or any other safety-affecting decision MUST come from its authoritative source (server config / contract / measured evidence). If that source is missing or unreadable, FAIL CLOSED — do not fire, do not move, surface the reason — instead of substituting a guessed/hard-coded default. A guessed value can be wrong in the unsafe direction (e.g. a tolerance that lets the server plan a move when the caller assumed a no-op). Prefer `None`/error + a logged reason over a plausible constant. This applies to every component (C++, GUI, policy_runner), not just Cartesian math.
- Do not weaken command-source lease, deadman, stale-state, fault, or real-mode checks. Real Cartesian motion now relies on `rb_servo_server` for the final allow/deny decision (safety filter, tracking-error latch, self-collision guard, lease, deadman) — treat these as load-bearing, not optional.
- Real motion is explicit and operator-supervised — never enable or retune it incidentally as part of simulator/benchmark work. Force control is live only through its tracked, hardware-validated v2 configuration and must not be disabled, enabled, or retuned as an unrelated change. Gripper motion remains separately gated by `allow_real_gripper_motion`, measured gripper availability, and `RB_ALLOW_REAL_GRIPPER=1`.

## Expected Validation

For Python changes, run as applicable:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner scripts
```

For C++ servo changes, build and run the hardware-free C++ tests when dependencies are installed:

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

For Cartesian behavior, use the Pinocchio-backed C++ tests plus active-stack
smoke/acceptance on mock when a local mock config is available, rbpodo
controller `pgmode` simulation / VM, and physical real only through the separate
supervised runbooks. The old software-simulator-oriented Cartesian acceptance
runner has been removed.

## Required Final Report

Every agent must report:

1. Summary
2. Files changed
3. Config/schema changes
4. Tests run
5. Test results
6. Skipped checks and why
7. Remaining TODOs
8. Safety implications
9. Whether real-mode behavior was touched
