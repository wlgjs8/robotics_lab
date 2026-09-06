# Plan clock time-stretch: chain lag becomes time, not a fault (2026-09-06)

Status: implemented on top of the 2026-09-06 "fault criterion moved to the robot"
change (chunk follower divergence/lead excused while the robot tracks its command,
`CommandTrackingWindow`) and the flow_infer angular cap raise 0.90 -> 1.40 rad/s.
Hardware validation pending.

## Problem

`servo_log_20260906_123643.csv` (pi0.5 rollout, 126 s) ended at 120.0 s on
`chunk_follower_divergence` while the left arm was tracking its command to 0.7-1.1 deg.
The policy asked the wrist for 53-84 deg/s; the follower core capped rotation at
0.90 rad/s, the output SMD lagged a further 5.5 deg, the IK branch-jump throttle held
50-73 % of the ticks, and the plan sat 6 deg ahead of the sent command. Three
watchdogs (projection error, actual lead, divergence) all measured that same chain
lag. Two structural facts follow:

1. Whenever the policy asks for more than the chain can deliver, lag is inevitable.
   Its size is not a safety quantity while the robot is on its command.
2. Truncating the excess (the core chaining at `t = dt` and re-targeting the next
   knot from wherever it got) DISTORTS the path: a wrist turn requested during an
   approach is partly dropped, not delayed. The policy re-plans from proprio, so the
   loop still closes, but the executed path is no longer the policy's path.

## Mechanism (two gates, min-combined with the existing safety plan gate)

The chunk follower already has a plan-clock rate gate (`setPlanRateGate`), driven by
the obstruction projection and the sustained IK throttle (`safety.plan_gate`). Step 2
adds two more sources of "how fast may the plan advance", and the follower runs at
the slowest of them:

### A. Core time-stretch (`core_time_stretch_enable`, `core_time_stretch_max_ratio`)

Inside `CartesianChunkFollower`. When the Ruckig segment solve reports a duration
longer than the policy period (`duration > seg_dt`), the segment now LASTS that
duration: samples are taken at wall time over `[0, duration]`, the chained state is
taken at `duration` (the knot itself, with its target velocity), and the next knot is
consumed only then. Bounded by `max_ratio` (segment length <= `seg_dt * max_ratio`;
beyond it the legacy truncation resumes). Effects:

- the executed path is the policy's path, traversed at the follower's own v/a/j
  limits; the projection error (knot vs plan) collapses to the truncation residual;
- per wall-clock chunk period fewer knots are consumed; the remainder is dropped at
  the next preemption exactly as before (delta frames anchor at the chained state).
- telemetry: `follower_core_gate = seg_dt / segment_length` (1.0 = not stretched).

### B. Divergence leash (`plan_leash_*`)

In `applyChunkFollowerStage`. The plan-vs-sent divergence of the current tick
(`follower_divergence_pos_m/ang_rad`, the same quantity the soft latch reads) drives a
linear ramp: gate 1.0 up to `plan_leash_start_{m,rad}`, falling to
`plan_leash_min_gate` at `plan_leash_full_{m,rad}` (the soft bound, 50 mm / 0.10 rad).
The plan clock therefore cannot wind the divergence past the soft bound unless the
chain stops delivering entirely, and it never stops (min_gate > 0). It covers what A
does not: output SMD lag, hold-paused ticks, anything downstream of the core.

Combined: `setPlanRateGate(min(safety_plan_gate, leash_gate))`, then the core stretch
applies inside `tick()` on top of that.

## Config (both tracked stacks, flow_infer / delta_preview profile)

```yaml
core_time_stretch_enable: true
core_time_stretch_max_ratio: 4.0        # a segment may last up to 4 policy periods
plan_leash_enable: true
plan_leash_start_m: 0.010
plan_leash_start_rad: 0.0349            # 2 deg
plan_leash_full_m: 0.050                # == kPoseTrackReanchorPosTolM
plan_leash_full_rad: 0.10               # == kPoseTrackReanchorAngTolRad
plan_leash_min_gate: 0.25
```

Validation (loader, fail-closed): ratio >= 1 when stretch is enabled; all leash
distances positive finite with start < full and 0 <= min_gate < 1 when the leash is
enabled.

## Telemetry / acceptance on the next rollout

- `follower_core_gate` < 1 only during fast turns/translations; median 1.0.
- `follower_leash_gate` rarely below 0.5; `follower_divergence_ang_rad` never above
  0.10 rad; `follower_divergence_excused_count` grows only in fast-turn windows.
- `follower_projection_error_rad` p99 drops from 0.32 rad (18.5 deg, 12:36 run) to
  the truncation residual (< 0.05 rad expected).
- No `chunk_follower_divergence` / `delta_preview_actual_lead_fault` while
  `follower_cmd_tracks` is 1. The joint tracking latch (`max_tracking_error_deg`) and
  the hard divergence bound (0.10 m / 0.30 rad) remain the fault owners.
- Vibration: compare 5-30 Hz command band RMS per moving window with the 12:09 /
  12:36 baselines (`/tmp/vib/tremor_scan.py`); the stretch must not add content
  (it only slows the knot clock; the Ruckig profile itself is unchanged).

## Finding during implementation: Ruckig-refused solves

The unit test for the stretch exposed a defect that the 12:36 run then confirmed:
`ChunkFollowerSegment::buildInput` clamped the target velocity to `v_max * (1 + 1e-9)`
and Ruckig's `validate_input` rejects any target above `v_max` outright
(`ErrorInvalidInput`). So every knot faster than the follower's cap did not saturate,
it BROKE the segment solve. In `servo_log_20260906_123643.csv` the left arm shows 557
ticks with `follower_duration_sec == 0` while active, every one of them inside the
118.68-120.0 s fault window (right arm: 33 at 29.22 s). On a refused solve the old
trajectory stayed in place and the segment clock restarted, so the emitted pose
jumped back one segment of travel and replayed it at 30 Hz while the knot clock kept
consuming (projection error 18.5 deg).

Fixed: targets are clamped strictly inside the limits (`clampInside`, including
Ruckig's `vf +/- af^2 / (2 j_max)` pre-target velocity rule), a refused solve is
served by a ring-down from the chained state instead of a replay, and the count is
logged as `follower_solve_failures`. Expect this column to stay at 0 on the next run.

## Follow-up after the 13:17 rollout (servo_log_20260906_131740.csv)

The stretch and the leash ran as designed (core gate < 1 on 5-10 % of ticks, leash
floor reached in one 0.3 s window, no divergence/lead fault, projection error 0).
Two things they exposed were fixed the same evening:

1. **Output SMD acceleration lag + reseed snap.** The output SMD was fed the chained
   end-state velocity low-passed at nf, so under acceleration it lagged ~3a/wn^2:
   29 mm / 5.7 deg at 56.0-56.2 s, which crossed its 0.10 rad reseed bound and
   snapped the target 28 mm in one tick (7,500 deg/s^2). Fix: `output_smd.
   profile_feedforward` (sampled v AND a of the plan fed forward, unfiltered) and
   reseed bounds moved to the hard divergence latch (0.10 m / 0.30 rad). The reseed
   is now reported in `follower_output_smd_reseeded`.
2. **False corners.** `corner_velocity_scale` 0.25 -> 1.0: the corner guard fired on
   32-39 % of segments, driven by knot-noise sign flips (median 2 mm/s), each one a 4x
   boundary-velocity cut the next segment had to undo.

Also raised: flow_infer `max_linear_velocity_m_s` 0.45 -> 0.60 (knot demand max
570 mm/s). The right-arm lunge at 104.4 s of that run was a force-overlay
strip-without-compose runaway (the frozen 2.24 mm deviation from 97.3 s subtracted
from the hold target every tick while coverage was "recovering"); it was handled
separately by the operator.

## Rollback

The profile-feedforward setting above records the pre-14:44 experiment. After
the operator reported both-arm vibration, same-delta offline replay isolated
that setting transmitting row-transition acceleration pulses to the output.
`flow_infer_smooth` and `flow_infer_fresh` in `stack_real.yaml` now set it false;
the other limits, corner scale and reseed bounds remain unchanged. Output
smoothing improves with increased lag in that replay. See
`outputs/chunk_review_20260906_144454/replay/ablation/report.md` for conditions
and the physical-validation limitations.

Setting both pacing `*_enable` keys to false disables time stretch and the
leash. It does not undo subsequent output conditioning, fresh-replan or
hold/resume changes, and does not recreate an earlier binary.
