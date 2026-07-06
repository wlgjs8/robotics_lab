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
- `rbpodo_state_decode_policy: bounded_status_codes_with_boolean_safety_flags`
  or `controller_sim_unreliable_fields_unavailable` for the narrow
  controller-simulation opt-in described below

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

## Controller-Simulation Unavailable-Field Policy

For rbpodo controller `pgmode` simulation only, the server can opt into:

```yaml
servo:
  controller_simulation_treat_unreliable_status_fields_as_unavailable: true
```

The option defaults to `false` and is active only when the rbpodo
controller-simulation motion carve-out is open in the site-local config:
`run_mode: real`, `backend_type: rbpodo`, `operation_mode: simulation`,
`servo.allow_controller_simulation_motion: true` (config-driven, no env gate).
It is not a physical-real decode policy.

When active, the decoder treats only `op_stat_self_collision` shape validation
and controller time plausibility as unavailable. The state stream must expose
the raw values under `rbpodo_diagnostics.raw`, list the suppressed fields in
`rbpodo_diagnostics.unavailable_fields`, and publish
`rbpodo_state_decode_policy=controller_sim_unreliable_fields_unavailable`.
The policy does not suppress SOS, EMS, soft-estop, collision, unknown
`real_vs_simulation_mode`, or the explicit `op_stat_self_collision == 1` fault
path.

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
python3 scripts/rbpodo_state_dump.py \
  --ips 172.28.60.200 172.28.60.201 \
  --output artifacts/rbpodo_measurement/state_dump.json \
  --pretty \
  --i-understand-this-connects-to-real-controller
```

The dump includes Python-side `q_ref_deg`, `q_ref_source`,
`q_ref_actual_delta_deg`, `q_actual_vs_q_ref_max_abs_error_deg`, and raw
diagnostic suspect reasons. It also includes Python rbpodo SDK/module metadata
when available. To print that metadata without connecting to a controller:

```bash
python3 scripts/rbpodo_state_dump.py --print-sdk-info
```

## Raw Data-Port Capture

Use raw Rainbow data-port capture when `diagnostics_suspect` persists and the
question is whether the rbpodo SDK, Python binding, firmware field layout,
C++ mapping, or raw 5001 payload interpretation is responsible.

```bash
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

By default the capture is compact: every sample gets metadata in
`samples.jsonl`, while raw payload bytes are stored only as `first_payload.bin`
and `last_payload.bin`. Use `--save-each-sample` only when a supervised
investigation needs one binary fixture per sample:

```bash
python3 scripts/rainbow_data_port_capture.py \
  --ip 172.28.60.200 \
  --port 5001 \
  --duration-sec 5 \
  --rate-hz 10 \
  --artifact-dir artifacts/rbpodo_measurement/raw_data_left \
  --also-rbpodo-python \
  --save-each-sample \
  --i-understand-this-connects-to-real-controller
```

Artifacts include:

- `samples.jsonl`
- `first_payload.bin` and `last_payload.bin` by default
- per-sample `samples_<ip>_<index>.bin` only with `--save-each-sample`
- optional `python_decoded_samples.jsonl`
- `summary.json`

The summary records `success_count`, `timeout_count`,
`unique_payload_lengths`, `unique_hash_count`, `stable_prefix_hex`,
`stable_suffix_hex`, per-sample payload SHA256/length/prefix/suffix,
inter-sample `changed_byte_count`, `first_changed_offset`,
`changed_offsets_histogram`, and the optional
`rbpodo_python_diagnostics_suspect_rate`. With `--also-rbpodo-python`, the
capture also records `q_actual_deg`, `q_ref_deg`, per-controller
`q_ref_delta_norm_deg`, and q_ref/payload transition counts:

- `q_ref_changed_payload_changed_count`
- `q_ref_changed_payload_static_count`
- `q_ref_static_payload_changed_count`

Generate an offline fixture report from a capture directory:

```bash
python3 scripts/rainbow_data_fixture_report.py \
  --capture-dir artifacts/rbpodo_measurement/raw_data_left \
  --output-md fixture_report.md \
  --output-json fixture_report.json
```

Relative report paths are written inside `--capture-dir`. The report repeats
the unique payload lengths, unique hash count, stable prefix/suffix, offsets
that change most often, q_ref/payload transition counts, and the required next
step: collect motion/no-op fixture, compare firmware SDK docs, and do not infer
layout yet.

Raw payloads may contain hardware-specific information. Keep real captures
under `artifacts/` and do not commit them unless they are reviewed and
sanitized. This capture is field-layout evidence only; it does not validate
motion safety. Map raw payload bytes to rbpodo SDK fields only after there is
enough fixture evidence to justify the layout.

## Diagnostics Root-Cause Report

Use the diagnostics report when `diagnostics_suspect` is present in rbpodo
controller-simulation artifacts or when a controller-simulation result used
the diagnostics-suspect config opt-in:

```yaml
servo:
  allow_controller_simulation_motion: true
  allow_controller_simulation_diagnostics_suspect: true
```

The opt-in is controller-simulation-only. It does not make the diagnostics
healthy and it does not authorize physical real motion. The legacy
`RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM` env gate is no longer read
by the server.

Generate the report from the read-only state dump, Python/C++ parity summary,
and raw 5001 capture summary:

```bash
python3 scripts/generate_rbpodo_diagnostics_report.py \
  --state-dump artifacts/rbpodo_measurement/state_dump.json \
  --parity-summary artifacts/rbpodo_measurement/state_parity/summary.json \
  --raw-capture artifacts/rbpodo_measurement/raw_data/summary.json \
  --output-md artifacts/rbpodo_measurement/diagnostics_report.md \
  --output-json artifacts/rbpodo_measurement/diagnostics_report.json
```

Report sections include:

- Python rbpodo SDK/module file and version when available.
- C++ state-stream hints such as `rbpodo_sdk_state_source`,
  `rbpodo_state_decode_policy`, and `q_ref_source` when parity artifacts expose
  them.
- controller IPs and `real_vs_simulation_mode` / pgmode evidence.
- raw diagnostic fields: `time`, `real_vs_simulation_mode`,
  `init_state_info`, `init_error`, `op_stat_sos_flag`,
  `op_stat_ems_flag`, `op_stat_soft_estop_occur`,
  `op_stat_collision_occur`, and `op_stat_self_collision`.
- suspect reasons and Python/C++ parity result.
- raw payload length/hash stability and q_ref-change versus payload-change
  evidence.
- an explicit root-cause checklist and physical-real blockers.

The root-cause classifier is intentionally conservative:

- `controller_reports_real_fault`: a raw controller fault flag is a clear
  fault value such as `op_stat_collision_occur=1`.
- `python_cpp_decode_mismatch`: Python rbpodo and C++ state JSON disagree.
- `payload_unstable`: raw 5001 payload length or q_ref/payload relation is
  unstable enough to block layout inference.
- `sdk_firmware_layout_mismatch`: Python and C++ agree on suspicious raw values
  that look like a firmware/SDK layout issue, for example huge boolean-status
  values.
- `field_semantics_unknown`: decoders agree, but the raw field meaning is not
  independently verified.
- `insufficient_evidence`: state dump, parity, or raw capture evidence is
  missing or inconclusive.

A diagnostics-suspect controller-simulation artifact must not be promoted to
physical evidence unless the diagnostics root cause is resolved or explicitly
accepted by a separate safety review. The diagnostics report keeps these
blockers visible:

- `diagnostics_suspect_unresolved`
- `stop_resetFault_unverified`
- `physical_reference_to_actual_error_unmeasured`

## Timestamp Alignment Audit

Use the timestamp audit only for historical or explicitly collected
controller-simulation circle artifacts, before interpreting RMS, p95, or max
tracking error as controller behavior:

```bash
python3 scripts/timestamp_alignment_audit.py \
  --artifact-dir artifacts/rbpodo_measurement/<run_id> \
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

Historical controller-simulation circle artifacts may include offline error
decomposition files:

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

Use the measurement reliability report only before using archived or explicitly
collected rbpodo pgmode circle metrics for tuning decisions:

```bash
python3 scripts/generate_rbpodo_measurement_reliability_report.py \
  artifacts/rbpodo_circle/gene_15cm4s_feedback_left/summary.json \
  --artifact-dir artifacts/rbpodo_measurement/reliability_report
```

For ablation summaries:

```bash
python3 scripts/generate_rbpodo_measurement_reliability_report.py \
  --ablation-summary-csv artifacts/rbpodo_measurement/<run_id>/ablation_summary.csv \
  --artifact-dir artifacts/rbpodo_measurement/reliability_report
```

Outputs:

- `measurement_reliability_report.md`
- `measurement_reliability_summary.csv`
- `measurement_reliability_summary.json`

Reliability levels:

- `unreliable`: no valid state, startup fault, blocked Cartesian path,
  fault-latched run, or physical motion detected during pgmode simulation.
- `suspect`: diagnostics are suspect, q_ref is missing or low-validity,
  Python/C++ state parity failed, timestamp/state/command timing is not clean,
  or evidence is otherwise incomplete.
- `controller_reference_valid`: the run completed with valid `tcp_ref_stand`
  and visible `q_ref`/`q_target`, no fault, and no physical motion detected.
  This is still controller-reference lower-bound evidence only.
- `physical_ready_candidate`: not assigned by rbpodo pgmode simulation reports.
  Physical readiness comes from the physical acceptance runbooks, not from a
  controller-simulation reliability report.

Important caveats:

**Controller `pgmode` simulation PASS is controller-reference lower-bound evidence, not physical TCP tracking.**

- `tcp_ref_stand` is FK from controller reference joints and is a lower bound
  on controller-reference tracking. It is not physical TCP tracking.
- `q_ref_deg` / `q_target_deg` visibility does not prove independent semantic
  validation of the raw `jnt_ref` field unless state parity and raw-data
  investigations have passed.
- A run with `diagnostics_suspect_count > 0` or an active diagnostics-suspect
  override is graded `suspect` and cannot be physical-ready.
- A run with failed Python/C++ state parity is graded no higher than `suspect`.
- A run with `tracking_source=tcp_ref_stand` must be interpreted as
  `controller_reference_lower_bound`.
- A missing or low `q_ref_valid_ratio` records `q_ref_not_directly_validated`.
- `gene_15cm_4s` stress rows are labeled `IL_data_not_recommended` by default.
  Use stable, clean-timing profiles for imitation-learning data candidates.

Physical real blockers remain until explicitly closed by future acceptance
work:

- `diagnostics_suspect_unresolved`
- `stop_resetFault_unverified`
- `physical_reference_to_actual_error_unmeasured`
- `camera_tcp_calibration_unresolved`
- `no_tiny_physical_acceptance`

Required report boundary fields:

```yaml
physical_readiness:
  status: blocked
  blockers:
    - diagnostics_suspect_unresolved
    - physical_reference_to_actual_error_unmeasured
    - stop_resetFault_unverified
    - camera_tcp_calibration_unresolved
    - no_tiny_physical_acceptance
  next_required_acceptance:
    - read-only diagnostics parity
    - tiny joint no-op physical or approved safe mode
    - tiny physical joint move
    - tiny physical Cartesian move
    - low-speed circle
    - then speed ladder
controller_reference_result:
  status: pass|fail
  explanation: "tcp_ref_stand lower-bound evidence"
physical_tracking_result:
  status: not_measured
```

Transition ladder:

1. Controller pgmode simulation repeatability
2. Right arm
3. Dual arm
4. P0 diagnostics root cause
5. Real controller read-only
6. Tiny physical acceptance
7. Slow physical circle
8. Fast physical circle only after approval

## Parity Check

Read-only server-start mode:

```bash
python3 scripts/rbpodo_state_parity_check.py \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --server-config rb_servo_server/config/local/stack_sim_readonly_measurement.yaml \
  --ips 172.28.60.200 172.28.60.201 \
  --duration-sec 5 \
  --state-endpoint udp://127.0.0.1:50171 \
  --artifact-dir artifacts/rbpodo_measurement/state_parity \
  --i-understand-this-connects-to-real-controller
```

Create the local config from `rb_servo_server/config/stack_sim.yaml` and keep
the copy under `rb_servo_server/config/local/`. The local copy uses
`operation_mode: simulation`, `servo.send_servo_commands: false`, read-only
unsafe-startup allowances, and measurement-only state fanout on
`udp://127.0.0.1:50171`. The parity checker refuses any supplied server config
that does not explicitly set
`servo.send_servo_commands: false`.

Already-running server mode:

```bash
python3 scripts/rbpodo_state_parity_check.py \
  --use-running-server \
  --ips 172.28.60.200 172.28.60.201 \
  --duration-sec 5 \
  --state-endpoint udp://127.0.0.1:50171 \
  --artifact-dir artifacts/rbpodo_measurement/state_parity \
  --i-understand-this-connects-to-real-controller
```

Artifacts:

- `summary.json`
- `python_samples.jsonl`
- `cpp_state_samples.jsonl`
- `parity.csv`
- `parity_report.md`

Key metrics:

- `max_q_actual_diff_deg`
- `max_q_ref_diff_deg`
- `max_q_target_diff_deg`
- `raw_field_match_rate`
- `diagnostics_suspect_agreement_rate`
- `python_time_plausible`
- `cpp_time_plausible`
- `q_ref_source_available`

Interpretation:

- `passed`: Python and C++ samples agree and diagnostics are not suspect.
- `suspect_but_consistent`: Python and C++ agree, but the decoded values remain
  suspect and block motion interpretation. The summary caveats include
  `diagnostics_suspect_unresolved`.
- `failed_transport`: no state packets or Python rbpodo samples could be read.
- `failed_server_exit`: `rb_servo_server` exited before publishing state; the
  summary includes `server_log_tail`.
- `failed_parity_mismatch`: field mismatch, missing samples, missing source
  metadata, or missing C++ `q_ref_deg`. Missing C++ q-ref publication is
  reported with caveat `q_ref_not_published`.

The checker waits for any state packet before sampling. A fault-latched state
packet is still parity evidence and is reported as `parity_suspect`, not as a
state-stream transport failure.
