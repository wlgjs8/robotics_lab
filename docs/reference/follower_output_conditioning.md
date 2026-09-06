# Follower output conditioning — 2026-09-06

## Selected change and reason

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

## Comparison on the same recorded reference

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

## Validation and limits

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

## Selection and evidence

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
