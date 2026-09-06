# UMI stream gate release and diagnostic logging

The pose-track SMD consumes the stream channel of `ForceGate`. After its
sustained-contact classifier disarms, the gate retains the last armed stand-frame
normal while its existing scalar slew reopens. It clears that normal when fully
open or reset. The force vector filter and contact classifier continue running;
a newly armed contact selects its current normal, including a different surface.

This is a scoped change to `ForceGate::updateStream`. It does not change the
force magnitude filter, arm/release thresholds, dwell, open/close time constants,
the tick gate, admittance law, safety projection, source pose filtering, or the
SMD goal. No new config flag or command option is required.

The reason is the 2026-09-06 UMI recordings: after disarming, the old reopening
gate continued to follow a small rotating force vector. Projecting every SMD step
and its velocity along that vector reproduced the recorded 8–9 Hz command ripple.
Retaining the last normal removes that direction modulation without instantly
opening the scalar gate. Contact/retreat along a changing surface during release
still needs supervised physical validation; preserving classifier parameters is
not proof of unchanged physical contact response.

## Offline evidence and limits

`umi_stream_gate_replay CONFIG INPUT.jsonl OUTPUT.csv` uses the production
`ForceGate` and `SmdPoseTracker` with fixed recorded goals/forces. The legacy
direction-following branch is an offline comparison oracle, not a runtime
fallback. The executable parses config but creates no robot backend, servo loop,
worker, or socket. The input starts with `umi_stream_gate_replay.v1` metadata
(profile, recorded frequency/caps, initial position and velocity), then increasing
time, force-vector warmup rows and sampled goal/force/period rows. Profiles must
be explicit and use SMD without velocity feedforward. This translation-only
comparison excludes lifecycle resets, SMD walls and singularity scaling; the
exporter checks that selected windows stay in that regime.

The local evidence is in `outputs/umi_gate_release_fix_20260906/`; export with
`prepare_replays.py`, run the C++ executable for each `replay_cases.json` entry,
then run `analyze_replays.py` using the OpenPI analysis environment. The legacy
SMD trajectory matches the recorded trajectory within 0.000014 mm. Metrics below
are XYZ 8–30 Hz position RMS in mm, not physical robot improvement estimates.

| Recording / arm / relative seconds | Legacy RMS | Revised RMS | Legacy goal error RMS | Revised goal error RMS |
|---|---:|---:|---:|---:|
| 19:53:29 / right / 154–158 | 0.3986 | 0.0124 | 30.28 | 21.53 |
| 19:53:29 / left / 171–175 | 0.2373 | 0.00412 | 18.96 | 12.66 |
| 20:34:54 / left / 70–76 | 0.1776 | 0.0143 | 22.22 | **30.00** |
| 20:34:54 / right / 114–120, never armed | 0.01124 | 0.01124 | 10.87 | 10.87 |

The command ripple falls by 92–98% in these contact-release windows. The newest
left-arm window pays an additional **7.78 mm transient goal-error RMS** because
the retained direction keeps limiting motion while the scalar gate opens. Its
maximum goal error also grows from **32.77 to 63.31 mm**. This is a material
tracking tradeoff, not an accepted physical-performance result. At the
end of each selected replay, revised-versus-legacy position differences are below
0.008 mm; this is local endpoint evidence, not a general bound on accumulated
path error. The no-contact right-arm trajectory is identical. These are fixed-input
comparisons; neither robot dynamics nor changed contact force/operator feedback
are simulated. The change does not address the separate self-collision projection
ripple or all measured right-arm vibration in the latest recording.

## Next-run CSV fields

For each arm, `*_smd_gate_sample_valid` says whether pose-track SMD actually
consumed a gate snapshot. Read the other `*_smd_gate_*` fields only when valid:

- `armed`, `releasing`, `translation`: classifier and scalar gate at consumption;
- `normal_x/y/z`: the applied unit normal in stand coordinates;
- `measured_fx/fy/fz_n`: slow pre-deadzone force vector, also in stand coordinates;
- `removed_velocity_m_s`: norm of the gate's removed displacement divided by that
  SMD step's `filter_dt`. It is the gate contribution, not total acceleration or
  downstream safety correction.

These are taken **before** the force update later in that servo tick. Existing
`*_fc_gate_stream_*` fields are taken **after** that update. Keeping both makes
the one-tick consumer timing explicit. Invalid snapshots reset each tick.

`projection_joint_stage_trace_valid` gates three raw-degree arrays per arm:
`*_projection_requested_q_deg_0..5`, `*_projection_solved_q_deg_0..5`, and
`*_projection_released_q_deg_0..5`. They are respectively the joint target after
upstream clamps, after the combined geometric solve, and after projection-release
slew. They are empty when invalid. They record the stages without changing their
behavior; the released stage is not necessarily the ultimate wire command.
Compare it with existing `*_q_sent_*`, worker send timestamps/overwrite counters,
`*_q_ref_*`, `*_q_actual_*`, and `*_state_host_time_ns` to locate further changes.
Use repeated state timestamps to identify repeated readback samples before
interpreting velocity or acceleration.

All new diagnostics are **CSV-only** to preserve the current UDP payload budget.
They are emitted automatically by the standard logger. No telemetry setting,
publisher flag, force retuning, or hardware motion is needed to enable them after
rebuilding/restarting the stack.

## Validation

`force_control` tests cover retained direction during rotating weak residual
forces, unchanged scalar reopening, tangential/retreat motion, newly sustained
contact in another direction, reset and fully-open cleanup. Existing tests cover
sustained contact holding, spring/fence, tare, and the independent tick gate.
`servo_logger_columns` checks exact values and header/row correspondence for the
new traces. Run the hardware-free C++ suite before a physical comparison.

Next supervised recordings should compare both vibration and transient tracking
error in contact/release/recontact, while using projection and wire/readback
traces to analyze the remaining independent sources. Passing these offline
checks does not establish physical acceptance.
