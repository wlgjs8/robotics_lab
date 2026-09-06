# Follower output conditioning — 2026-09-06

## Current selection: position-only conditioning

The real `flow_infer_fresh` profile now selects `output_smd.mode:
position_lowpass2`, linear/angular natural frequencies **4.5/3.5 Hz**, damping
`0.7071067811865476`, and both velocity/profile feedforward off. Legacy-only
LPF/gain fields remain neutral at 0/1. Fresh frame activation, held resume and
all existing follower v/a/j ceilings remain unchanged.

The new scalar response is `wn²/(s²+2ζwn s+wn²)`, integrated with the
trapezoidal rule. For fixed period and ζ≥sqrt(0.5), its bilinear discrete
steady-state sinusoidal gain cannot exceed one. Both translation and SO(3)
rotation use this position-only structure. Rotational errors share one output
body frame, and angular velocity is transported after each integration step.
Eigen/Pinocchio own log/exp and quaternion math. This scalar guarantee is not a
global nonlinear SO(3), variable-period, transient-energy or robot guarantee.

Reset and common force shifts update the previous input history as well as the
output state. Existing safety/IK holds bypass normal filter evolution and keep
the accepted nominal pose. Existing hard-divergence reseed thresholds are retained.
The loader rejects unknown modes, damping below sqrt(0.5), legacy FF options
mixed with the new mode, and selection without enabled fresh `delta_preview`.
`legacy_smd` remains the default; smooth and controller-simulation profiles keep
their previous output conditioning.

The initial selection recording (`servo_log_20260906_161839.csv`, 16:21:55 griponly) was
replayed through the actual follower and filter. All 945 consumed frames per arm
match the frozen baseline at CSV precision, with no wire/step mismatch. The
selected filter's offline Pinocchio joint-command RMS ratios are:

| Band | Left | Right |
|---|---:|---:|
| 0.5–3 Hz | 0.895 | 0.902 |
| 3–8 Hz | 0.820 | 0.825 |
| 8–20 Hz | 0.707 | 0.701 |

These are commands, not predicted measured robot motion. Joint spectra cover
post-resume 23.2–90.7 s with the recorded calibrated arm model/mounts and force
deviation. Each arm's 34,345 samples has zero IK failures or joint clamps.
Conservative leash replay additionally uses its own previous output, min-combined
with the recorded gate. Position lag p95 rises from **3.63/3.42 to 8.50/8.47 mm**;
its minimum leash gate is 0.865/0.955, with no normal-run stall, solve failure,
lead fault or reseed. This cannot release an already recorded closed gate and
does not simulate new force/contact, geometry projection or model responses.

Finite ramp delays are about 50 ms linear and 64 ms angular. Thus smoother
commands cost following error. The model's roughly 0.32 Hz large excursions
remain substantially intact; neither zero lag nor zero vibration is promised.
A faster 5.1/4.1 Hz candidate reduced joint 3–8 Hz RMS by only about 5%.

`deadline_jerk_minimization` was also implemented and tested, but is explicitly
**false** in the real profile. With at most six verified Ruckig trials it selects
less jerk only within the same guarded target p/v/a and deadline; braking and
infeasible cases retain the original solution and limits. It reduced typical
fresh-swap per-axis jerk from 2000 to about 1063–1094 m/s³, but the combined
8–20 Hz joint RMS ratios were 0.725/0.737, worse than the filter alone. This
bounded search is therefore available for experiments and is not enabled by
default. It is not a globally optimal minimum-jerk solver.

New CSV fields `*_follower_jerk_scale` and
`*_follower_jerk_search_calculations` accompany existing full prefilter pose,
sampled v/a, gates, lag and reseed diagnostics. State capabilities advertise
both the option and filter mode. The policy command retains `ready_event`,
`flow_infer_fresh`, `servo_command` and `last_emitted_continuous`; changes are
server-owned and require loading the rebuilt server and tracked config.

No physical component was started. No force-law, Servo J, IK/joint safety or
follower motion ceiling was retuned by this change. See
`outputs/fresh_motion_redesign_20260906/report.md` for the integrated tests,
reproduction commands, frozen geometry provenance and limitations;
`outputs/nonpeaking_conditioner_design_20260906/report.md` for the 16-candidate
filter comparison; and `outputs/deadline_jerk_replan_20260906/report.md` for the
optional trajectory search. The earlier iteration below is superseded.

## Follow-up real rollout: 17:18:47

The subsequent [constrained preview experiment](preview_trajectory_execution.md)
implements an offline candidate without an output low-pass filter. It is a
separate build target and is not a selectable replacement for this live profile.

The subsequent 92.72 s griponly rollout confirms that this filter is active:
all 980 frames per arm reproduce through native follower/filter replay, with
raw/stage pose error below 0.0000015 mm and no wire/step mismatch. Normal-run
InitMotion, repeated SMD reset, follower reanchor and IK refusal are absent.
Remaining 3–8 Hz motion is already in the sent joint commands; windowwise
measured/sent RMS ratios are 1.030/1.024. This is not zero physical vibration.

Additional lower-cutoff candidates reduce fixed-input command spectra, but
their added lag activates the plan leash and changes accumulated progress.
An anchored three-knot preview average avoids the legacy head-clamp bias,
yet changes the executed prefix when a fresh frame preempts after only two
or three rows; preserving the full chunk endpoint does not preserve the
executed displacement. Its right-arm path drifts substantially in replay,
and the subsequent native IK audit rejects some of the changed targets.
A scalar notch has a better frequency/lag tradeoff but more stop ringing;
native SO(3)/lifecycle implementation has not been validated.

These candidates are **not enabled**. The production selection above remains
4.5/3.5 Hz, `smoothing_window: 1`, with deadline jerk search off. Counterfactual
replays keep model observations, contact and actual feedback recorded; later
lead flags/IK results are diagnostic continuations, not predictions of a
physical rollout. A future preview redesign must retain the original executed-
prefix reference across arbitrary fresh preemption and bound accumulated
position/orientation offset. It must also check conditioned joint commands,
not only TCP spectra.
See `outputs/chunk_review_20260906_171847/report.md` for provenance, same-input
comparisons, native IK results and the limits of this analysis.

## Historical 0.8 feedforward iteration

The real `flow_infer_fresh` profile sets
`ruckig_follower.output_smd.velocity_ff_linear_gain: 0.8`. Linear and angular
natural frequencies remain 3.5 and 2.5 Hz, damping remains 1, velocity FF remains
enabled, and acceleration/profile FF remains disabled. Only the translation
velocity contribution to the output conditioner's damping term changes.
The legacy/default gain is 1; `flow_infer_smooth` explicitly retains 1.
`stack_sim.yaml` retains its existing disabled output conditioner.

This is a compromise selected against the 15:21:25 griponly rollout, not a claim
of zero vibration or optimal physical tracking. For critical damping and an FF
LPF cutoff equal to the natural frequency, the continuous translation response is

`H(s) = wn² ((1 + 2k)s + wn) / (s + wn)³`.

The historical k=1 response peaks at about 1.287 in the actual 500 Hz discrete
integrator. k=0.8 reduces that peak to 1.163. Some amplification remains. Eliminating
all peaking with this structure requires k≤(sqrt(3)−1)/2 in continuous time and costs
more tracking lag. The selected k=0.8 keeps the same cutoff and changes one active
parameter; angular behavior remains unchanged. Gain is finite in [0,1], and
non-default gain is refused unless velocity FF is enabled and profile FF disabled.

### Comparison on the same recorded reference

Source: `outputs/sweep/20260906_152125_boltv2_griponly_40k` and
`logs/servo_log_20260906_152057.csv`. Both arms have 55,224 active ticks. Metrics use
0.5<t<110 s, excluding the state gap at 82.6–83.6 s and an unmodeled geometric hold/fold
plus washout at 86.7–87.9 s. Filters evolve through the complete input; spectral
windows do not concatenate across excluded gaps.

| Metric | Left | Right |
|---|---:|---:|
| Prefilter→conditioned position error p95 | 3.38→4.60 mm | 1.84→3.25 mm |
| Same error, maximum in analysis intervals | 13.00→10.25 mm | 5.98→6.21 mm |
| Cartesian velocity RMS, 0.5–3 Hz, candidate/baseline | 0.953 | 0.947 |
| Cartesian velocity RMS, 3–8 Hz | 0.876 | 0.875 |
| Cartesian velocity RMS, 8–20 Hz | 0.867 | 0.867 |
| IK joint-command RMS, 3–8 Hz | 0.954 | 0.945 |
| IK joint-command RMS, 8–20 Hz | 0.926 | 0.905 |
| IK joint-command RMS, 0.5–3 Hz | 0.994 | 0.997 |

These are reference/command metrics, not counterfactual measured robot motion.
The model's roughly 1 Hz back-and-forth plan and recorded angular stage are retained.
The added steady-ramp lag is about 18.19 ms relative to the historical filter;
the selected discrete output lags the current sampled ramp by about 16.19 ms
(the old integrator leads that sample by roughly 2 ms).

Alternative k=0.7 at 3.5 Hz reduces Cartesian 8–20 Hz RMS by about 20%, but increases
position error p95 to 5.71/4.52 mm. A nonpeaking k≈0.366 at 4.5 Hz costs about
8.14/7.25 mm p95. Those alternatives were compared but are not enabled.

### Validation and limits

- Production C++ reference replay matches the independent numerical integrator
  exactly for both candidate and reconstructed baseline, with no internal reseeds.
- The original CSV lacks sampled velocity and prefilter rotation. Translation
  velocity is reconstructed from the recorded prefilter; baseline output matches
  within 0.000895 mm p95 after the stated exclusions. Angular stage is held at its
  recorded value, not replaced with endpoint rotation. New CSV fields record full
  prefilter pose and sampled physical derivatives for subsequent audits.
- Hashed per-arm calibrated URDFs and actual Pinocchio IK plus the existing joint
  clamps accept every candidate sample, with no IK failure or joint/branch clamp.
  This does not replay geometric/collision projection, new force feedback or plant
  response. Original-stage IK reproduces recorded sent joints at CSV precision
  outside the geometric intervention.
- Actual mock servo-loop tests separately cover InitMotion/tare, first chunk,
  repeated IK refusal and resume. The selected candidate holds the nominal stage
  fixed and resumes with zero first-stage step; it does not weaken a safety hold.
- A one-pass leash comparison creates no new gate<0.99 ticks. It is a diagnostic
  screen, not a closed-loop prediction: changed commands can alter force, future
  observations, proprio and model output.
- Normal ramp→stop tests improve overshoot. Hard safety holds bypass normal filter
  evolution; this is not a globally jerk-bounded or C2 output guarantee.

No Servo J gains, worker cadence, force/tare law, IK bounds, geometric constraints,
or v/a/j ceilings changed. A 10 Hz measured/reference amplification in the previous
run is not enough to retune hardware gains; lowering its command excitation is the
bounded change here. See the separate [UDP state reliability fix](state_udp_payload_budget.md).

### Selection and evidence

The supervised policy command keeps the three existing selections:

```bash
FLOW_INFER_CHUNK_ACTIVATION_MODE=ready_event
FLOW_INFER_TCP_TARGET_PROFILE=flow_infer_fresh
FLOW_INFER_VELPROPRIO_SOURCE=servo_command
```

The gain is server-owned in tracked `stack_real.yaml`, not a policy env setting.
The updated server binary/config must be loaded by restarting the stack through
the normal operator procedure. No robot/server was started by this offline work.
`last_emitted_continuous` remains selected on the policy side.

Evidence index:

- `outputs/lowfreq_conditioner_redesign_20260906/report.md` — final validation and provenance.
- `outputs/chunk_conditioner_design_20260906/balanced_report.md` — frequency/lag sweep and native match.
- `outputs/lowfreq_conditioner_redesign_20260906/ik_audit/report.md` — actual IK and lifecycle audit.
- `outputs/chunk_review_20260906_152125/udp/report.md` — state-packet failure reproduction and fix.
