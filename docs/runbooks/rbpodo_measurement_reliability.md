# rbpodo Measurement Reliability

This runbook is for read-only measurement validation before interpreting
controller-simulation tracking reports.

It does not authorize physical motion.

## Reference-State Fields

For the rbpodo backend, `q_ref_deg` is the Rainbow controller reference joint
vector read directly from:

```text
rbpodo SystemState.sdata.jnt_ref
```

`q_target_deg` and `q_ref_deg` are published as the same vector. The state JSON
also publishes:

- `q_ref_source: rbpodo.sdata.jnt_ref`
- `q_ref_valid`
- `q_actual_valid`
- `rbpodo_sdk_state_source: CobotData.request_data`
- `rbpodo_state_decode_policy: strict_boolean_flags_with_suspect_large_values`

`tcp_ref_stand` is FK from `q_ref_deg`, not an independent controller field.
`tcp_actual_stand` remains FK from measured `q_actual_deg`.

## What Parity Proves

`scripts/rbpodo_state_parity_check.py` compares Python rbpodo `CobotData`
samples against C++ `rb_servo_server` state JSON by nearest host time. It
compares:

- `q_actual_deg`
- `q_ref_deg` / `q_target_deg`
- raw diagnostic fields such as `time`, `real_vs_simulation_mode`,
  `init_state_info`, and rbpodo operation status flags
- `diagnostics_suspect` agreement

A passing parity check means Python and C++ decoded the same values for the
sampled fields. It does not prove the raw diagnostic fields are semantically
valid. If both sides decode a huge invalid status value the result is
`suspect_but_consistent`, and diagnostics still require investigation.

Physical real motion must not proceed while `diagnostics_suspect` remains
unresolved.

## Safety Contract

The parity tools are read-only:

- no `move_servo_j`
- no `pgmode`
- no fault reset
- no controller-state mutation
- no benchmark run by default

Known real controller IPs require:

```bash
--i-understand-this-connects-to-real-controller
```

When the parity checker starts `rb_servo_server`, it refuses configs that do not
explicitly set:

```yaml
servo:
  send_servo_commands: false
```

Use `--use-running-server` only when an operator has already started the server
for the intended read-only or controller-simulation diagnostic session.

## Read-Only State Dump

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rbpodo_state_dump.py \
  --ips 172.28.60.200 172.28.60.201 \
  --output artifacts/rbpodo_measurement/state_dump.json \
  --pretty \
  --i-understand-this-connects-to-real-controller
```

The dump includes Python-side `q_ref_deg`, `q_ref_source`,
`q_ref_actual_delta_deg`, `q_actual_vs_q_ref_max_abs_error_deg`, and raw
diagnostic suspect reasons.

## Raw Data-Port Capture

Use raw Rainbow data-port capture when `diagnostics_suspect` persists and the
question is whether the rbpodo SDK, Python binding, firmware field layout,
C++ mapping, or raw 5001 payload interpretation is responsible.

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rainbow_data_port_capture.py \
  --ip 172.28.60.200 \
  --port 5001 \
  --duration-sec 5 \
  --rate-hz 10 \
  --artifact-dir artifacts/rbpodo_measurement/raw_data_left \
  --also-rbpodo-python \
  --i-understand-this-connects-to-real-controller
```

The tool only connects to TCP data port 5001 and sends the read-only
`reqdata` request by default. It does not connect to command port 5000, send
motion, set `pgmode`, reset faults, or parse binary payloads.

Artifacts include:

- `samples.jsonl`
- `raw_response.bin`
- per-sample `samples_<ip>_<index>.bin`
- optional `python_decoded_samples.jsonl`
- `summary.json`

The summary records `success_count`, `timeout_count`,
`unique_payload_lengths`, `unique_hash_count`, `stable_prefix_hex`, and the
optional `rbpodo_python_diagnostics_suspect_rate`. The fixture comparison is
limited to length/hash/prefix patterns and whether payload hashes changed while
Python `q_ref_deg` changed. It is not a binary parser.

Raw payloads may contain hardware-specific information. Keep real captures
under `artifacts/` and do not commit them unless they are reviewed and
sanitized. This capture is field-layout evidence only; it does not validate
motion safety. Map raw payload bytes to rbpodo SDK fields only after there is
enough fixture evidence to justify the layout.

## Timestamp Alignment Audit

Use the timestamp audit after a controller-simulation circle benchmark finishes
and before interpreting RMS, p95, or max tracking error as controller behavior:

```bash
python3 scripts/timestamp_alignment_audit.py \
  --artifact-dir artifacts/rbpodo_circle_ablation/gene4s_stage1/.../02_fb_kp05_ori05_pub50_speed01 \
  --output-md alignment_report.md \
  --output-json alignment_summary.json
```

The tool is offline/read-only. It reads `summary.json`,
`state_stream.jsonl`, `command_packets.jsonl`, optional
`overlay_stream.jsonl`, optional `feedback_terms.jsonl`, optional
`samples.csv`, and optional `rb_servo_server.log`. It never sends robot
commands or changes controller state.

The audit separates:

- command generation interval jitter and gaps
- state publish interval jitter and state age
- servo send duration
- ACK wait spikes at 5 ms, 10 ms, 20 ms, and 40 ms thresholds
- stale feedback state skips
- overlay interval and overlay/state skew
- tail error near versus away from ACK spikes, command gaps, and stale skips

Benchmark summaries include nested `timestamp_alignment` and
`tail_error_correlation` blocks, plus flat CSV fields such as
`timing_classification`, `ack_spike_count_10ms`,
`ack_spike_count_20ms`, `state_gap_count`, and `command_gap_count`.

`timing_classification` is conservative:

- `clean_timing`: no timing spikes above the audit thresholds were observed.
- `command_generation_limited`: host-side command generation has severe gaps.
- `ack_spike_limited`: ACK waits over 20 ms are present.
- `state_publish_limited`: state stream gaps over 2x expected are present.
- `jitter_limited`: smaller command or ACK timing spikes are present.

Non-clean timing does not prove the controller tracking is bad; it means the
artifact is measurement-limited and tail errors should not be treated as
reliable gain-quality evidence. Fix timing or run a controlled follow-up such
as an ACK-off controller-simulation experiment before using those tails for
tuning decisions.

## Circle Error Decomposition

Completed controller-simulation circle benchmarks write offline error
decomposition artifacts:

- `error_decomposition.json`
- `cycle_error_decomposition.csv`
- flat `summary.json` fields such as `tail_ratio`,
  `center_removed_rms_error_m`, `phase_aligned_rms_error_m`,
  `orientation_position_equiv_50mm_m`, and `error_classification`

The decomposition is offline/read-only. It consumes `samples.csv`, benchmark
summary fields, feedback artifacts, and timestamp-audit output; it does not
connect to controllers, send commands, or change state.

Use it to separate:

- phase lag, reported as `phase_lag_rad`, `estimated_latency_ms`, and
  `phase_aligned_rms_error_m`
- center drift, reported as `center_error_m` and
  `center_removed_rms_error_m`
- radius error, reported as `radius_error_m` and `radius_gain`
- tail spikes, reported as `median_error_m`, `mad_error_m`,
  `iqr_error_m`, `tail_ratio`, and `max_over_p95`
- orientation drift, reported as p50/p95/max angle plus position-equivalent
  error for 0.03 m, 0.05 m, and 0.10 m tool-offset guesses
- saturation and timing limits, based on feedback saturation and
  `timing_classification`

`error_classification` is a diagnosis hint, not a pass/fail gate. It can be
`phase_lag_limited`, `center_drift_limited`, `tail_spike_limited`,
`orientation_limited`, `saturation_limited`, `timing_jitter_limited`, or
`balanced_or_unclassified`. A tail-spike or timing-jitter classification means
RMS/p95 should not be used as clean controller tuning evidence until the tail
source is understood.

The benchmark option:

```bash
--tool-offset-m 0.03,0.05,0.10
```

controls the orientation-induced position-equivalent estimates. The default is
the three listed offsets.

## Measurement Reliability Report

Use the measurement reliability report before using rbpodo pgmode circle
metrics for tuning decisions:

```bash
python3 scripts/generate_rbpodo_measurement_reliability_report.py \
  artifacts/rbpodo_circle/gene_15cm4s_feedback_left/summary.json \
  --artifact-dir artifacts/rbpodo_measurement/reliability_report
```

For ablation summaries:

```bash
python3 scripts/generate_rbpodo_measurement_reliability_report.py \
  --ablation-summary-csv artifacts/rbpodo_circle_ablation/gene4s_stage1/ablation_summary.csv \
  --artifact-dir artifacts/rbpodo_measurement/reliability_report
```

Outputs:

- `measurement_reliability_report.md`
- `measurement_reliability_summary.csv`
- `measurement_reliability_summary.json`

Reliability levels:

- `unreliable`: no valid state, startup fault, blocked Cartesian path,
  fault-latched run, or physical motion detected during pgmode simulation.
- `suspect`: q_ref is missing or low-validity, timestamp/state/command timing
  is not clean, or evidence is otherwise incomplete.
- `controller_reference_valid`: the run completed with valid `tcp_ref_stand`
  and visible `q_ref`/`q_target`, no fault, and no physical motion detected.
  This is still controller-reference lower-bound evidence only.
- `physical_ready_candidate`: reserved for future physical real acceptance and
  not assigned while `diagnostics_suspect` remains unresolved.

Important caveats:

- `tcp_ref_stand` is FK from controller reference joints and is a lower bound
  on controller-reference tracking. It is not physical TCP tracking.
- `q_ref_deg` / `q_target_deg` visibility does not prove independent semantic
  validation of the raw `jnt_ref` field unless state parity and raw-data
  investigations have passed.
- A run with `diagnostics_suspect_count > 0` or an active diagnostics-suspect
  override cannot be physical-ready.
- `gene_15cm_4s` stress rows are labeled `IL_data_not_recommended` by default.
  Use stable, clean-timing profiles for imitation-learning data candidates.

Physical real blockers remain until explicitly closed by future acceptance
work:

- `diagnostics_suspect_unresolved`
- `stop_resetFault_unverified`
- `physical_reference_to_actual_error_unmeasured`
- `camera_tcp_calibration_unresolved`
- `no_tiny_physical_acceptance`

## Parity Check

Read-only server-start mode:

```bash
python3 scripts/rbpodo_state_parity_check.py \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --server-config rb_servo_server/config/local/dual_real_rbpodo_readonly.yaml \
  --ips 172.28.60.200 172.28.60.201 \
  --duration-sec 5 \
  --state-endpoint udp://127.0.0.1:50151 \
  --artifact-dir artifacts/rbpodo_measurement/state_parity \
  --i-understand-this-connects-to-real-controller
```

Already-running server mode:

```bash
python3 scripts/rbpodo_state_parity_check.py \
  --use-running-server \
  --ips 172.28.60.200 172.28.60.201 \
  --duration-sec 5 \
  --state-endpoint udp://127.0.0.1:50151 \
  --artifact-dir artifacts/rbpodo_measurement/state_parity \
  --i-understand-this-connects-to-real-controller
```

Artifacts:

- `summary.json`
- `samples_python.jsonl`
- `samples_cpp_state.jsonl`
- `parity.csv`
- `parity_report.md`

Key metrics:

- `max_q_actual_diff_deg`
- `max_q_ref_diff_deg`
- `raw_field_match_rate`
- `diagnostics_suspect_agreement_rate`
- `python_time_plausible`
- `cpp_time_plausible`
- `q_ref_source_available`

Interpretation:

- `passed`: Python and C++ samples agree and diagnostics are not suspect.
- `failed`: field mismatch, missing samples, or missing source metadata.
- `suspect_but_consistent`: Python and C++ agree, but the decoded values remain
  suspect and block motion interpretation.
