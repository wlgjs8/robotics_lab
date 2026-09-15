# Historical force/preview contract before the single-target revision

Source: AGENTS.md at `25dc45e42d82594976a4a470c40636b2aabab056`.
Historical measurements and previous design decisions; superseded as an operating contract by
[the current force/preview contract](../reference/force_preview_single_target.md).

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

