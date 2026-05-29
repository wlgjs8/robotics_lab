# rbscript TCP Backend Ablation Runbook

`RBSCRIPT-ABLATION-01` compares transport timing for the existing `rbpodo`
backend path and the experimental raw Rainbow `rbscript_tcp` path. It is a
no-motion benchmark by default. It measures connection, state acquisition, and
safe command ACK timing; it does not measure manipulation performance.

## Backend Roles

| Backend | Role | Real-motion status |
| --- | --- | --- |
| `rbpodo` | Primary vendor-library backend for real RB controller integration. | Still gated; read-only acceptance comes before motion. |
| `rbscript_tcp` | Experimental raw Rainbow script TCP backend for overhead comparison. | Not a replacement for `rbpodo`; not real-motion-ready. |
| `simulator` / `rbsim` | Software simulator backend used for hardware-free acceptance. | Simulator-only; passing it does not approve hardware. |

`rbscript_tcp` uses Rainbow script text commands on TCP command port 5000 and
state/data requests on TCP data port 5001. It is not UDP, and this project must
not add a UDP direct-to-controller command path.

## Comparison Config Templates

The backend comparison templates are tracked examples. Copy a template to
`rb_servo_server/config/local/`, edit it for the site, and keep the local copy
out of git. Do not treat tracked `config/local/*.yaml` files as production
configuration; that directory is operator-owned.

| Backend | Purpose | Template | Command/state endpoints | Controller ports | Default send |
| --- | --- | --- | --- | --- | --- |
| `rbpodo` | 100 Hz ACK-on read-only diagnostic | `rb_servo_server/config/dual_real_100hz_ack.example.yaml` | `50031` / `50131` | `5000` / `5001` | `false` |
| `rbpodo` | 200 Hz ACK-on read-only diagnostic | `rb_servo_server/config/dual_real_200hz_ack.example.yaml` | `50032` / `50132` | `5000` / `5001` | `false` |
| `rbpodo` | 200 Hz ACK-off read-only diagnostic | `rb_servo_server/config/dual_real_200hz_no_ack.example.yaml` | `50033` / `50133` | `5000` / `5001` | `false` |
| `rbscript_tcp` | 100 Hz ACK-on read-only diagnostic | `rb_servo_server/config/dual_real_rbscript_100hz_ack.example.yaml` | `50041` / `50141` | `5000` / `5001` | `false` |
| `rbscript_tcp` | 200 Hz ACK-on read-only diagnostic | `rb_servo_server/config/dual_real_rbscript_200hz_ack.example.yaml` | `50042` / `50142` | `5000` / `5001` | `false` |
| `rbscript_tcp` | 200 Hz ACK-off read-only diagnostic | `rb_servo_server/config/dual_real_rbscript_200hz_no_ack.example.yaml` | `50043` / `50143` | `5000` / `5001` | `false` |

The rbscript templates use `script_t1_sec`, `script_t2_sec`, `script_gain`,
and `script_alpha`; these are the raw-script equivalents of the rbpodo
`servo_t1_sec`, `servo_t2_sec`, `servo_gain`, and `servo_alpha` fields.

Controller-simulation no-op templates are separate and not defaults:

| Backend | Template | Command/state endpoints | Notes |
| --- | --- | --- | --- |
| `rbpodo` | `configs/backend_compare/rbpodo_200hz_ack_sim_noop.yaml` | `50034` / `50134` | Requires verified controller `pgmode` simulation, normal real-controller/motion env gates, and tool safety preflight. |
| `rbscript_tcp` | `configs/backend_compare/rbscript_tcp_200hz_ack_sim_noop.yaml` | `50044` / `50144` | Requires verified controller `pgmode` simulation, normal rbscript real-controller/motion env gates, and tool safety preflight. |

## Safety

This runbook is for read-only or no-motion probes first. The named
`dual_real_*` comparison templates keep `send_servo_commands: false`. Do not set
`pgmode real`, and do not run high-rate real motion as part of this task.

Only files whose names contain `sim_noop` intentionally set
`send_servo_commands: true`; they are for controller `pgmode` simulation no-op
checks only and still require explicit env gates plus tool-level safety
preflight. They are not real-motion acceptance configs.

Do not use this runbook on a real robot without an operator present and an
accessible E-stop. Do not run motion probes from this runbook; create and pass a
future motion-specific runbook first. Do not use undocumented controller
settings to unlock or bypass limits.

Known real controller IPs require an explicit confirmation flag:

```bash
--i-understand-this-connects-to-real-controller
```

Real `rbpodo` connect/read probes require:

```bash
RB_ALLOW_REAL_ROBOT=1
```

Real `rbscript_tcp` connect/read probes require:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_RBSCRIPT_TCP=1
```

Motion-capable modes are intentionally not implemented in
`RBSCRIPT-ABLATION-01`. If a future task adds one, it must also require
`--allow-motion`, explicit `--arm`, explicit `--max-delta-deg`, and the normal
real motion gates. For rbscript motion that also means
`RB_ALLOW_RBSCRIPT_TCP_MOTION=1`.

`RBSCRIPT-SERVO-NOOP-01` adds one narrow motion-capable controller-simulation
exception through `scripts/rbscript_servo_acceptance.py`. It is only for
`servo_j_noop` where the target equals the current joint vector and the selected
arm config says `operation_mode: simulation`. It still requires:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_RBSCRIPT_TCP=1
RB_ALLOW_RBSCRIPT_TCP_MOTION=1
```

The script also requires `--allow-motion` and
`--i-understand-this-connects-to-real-controller`. ACK-off profiles additionally
require `--allow-ack-disabled`, and ACK-off success is socket-send evidence
only, not controller acceptance evidence.

Real Cartesian motion remains separately blocked by `RB_ALLOW_REAL_CARTESIAN=1`
and is outside this rbscript comparison runbook.

## Staged Acceptance

Run stages in order and stop at the first unexpected controller error, timeout
burst, parse failure, or reconnect instability:

1. `connect_only`: establish no-motion connectivity and reconnect behavior.
2. `read_state`: validate state/data acquisition without motion.
3. `ack_no_motion`: measure script command ACK latency using a verified
   no-motion command.
4. simulation-mode `servo_j` no-op probe: only through
   `rbscript_servo_acceptance.py`, and only when controller pgmode simulation
   is set and verified without physical robot motion.
5. tiny real joint motion: future work only after separate approval, operator
   setup, and a motion-specific runbook.

For the simulation-mode `servo_j` stage, use `rbscript_servo_acceptance.py`.
Do not run it from a read-only tracked template directly. Copy a profile to a
site-owned local file, verify the controller is in pgmode simulation, set the
selected arm `operation_mode: simulation`, and use a filename containing
`sim_noop` if `servo.send_servo_commands: true` is enabled.

## What Is Compared

`rb_backend_ablation.py` supports:

- `connect_only`: repeatedly opens the selected backend connection and records
  connection latency.
- `read_state`: requests state at the requested rate and records read duration,
  parse failures, timeout counts, and valid-state counts.
- `command_ack_no_motion`: sends an explicitly supplied no-motion script command
  on the rbscript TCP command port and records ACK latency.

`rbscript_tcp` uses TCP script command port `5000` and data port `5001`. It is
not UDP. The state probe sends `reqdata` to the data port. Current parser
support is deliberately conservative: it recognizes the fixture schema used by
tests and refuses unknown/binary payloads instead of guessing Rainbow offsets.
Real Rainbow 5001 payload parsing is not accepted yet. Unknown data-port
responses are reported as `rbscript_tcp_real_data_port_unsupported` with:

- `rbscript_tcp_data_port_mode: real_controller_unsupported`
- `read_state_capability: unsupported`
- `comparable: false`

The only accepted parser mode today is the test/local
`rbscript_tcp_state_v1` JSON fixture, which is marked
`rbscript_tcp_data_port_mode: json_fixture` and
`read_state_capability: experimental`. Do not treat that fixture path as an
apples-to-apples real-controller state comparison with `rbpodo`.

The comparison is transport/client overhead, not the controller internal servo
loop limit. A 100 Hz or 200 Hz ACK result must be interpreted with controller
ACK/error behavior and timeout counts.

Primary metrics:

- ACK latency p50/p95/p99/max
- persistent socket mode and reconnect count
- command write duration p50/p95/max
- ACK/read response duration p50/p95/max
- response lines per command p50/p95/max
- stale, extra, and unrecognized response counts
- command success, error, and timeout counts
- M561/M568/M569/M570 counts when observed in controller responses
- state age and valid-state count
- requested rate vs achieved rate
- reconnect count and parse failure count

## rbpodo Read-State Probe

The Python ablation tool uses an optional Python `rbpodo` binding for direct
`rbpodo` read-state probes. If that module is not installed, the tool fails
clearly instead of pretending to measure rbpodo.

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rb_backend_ablation.py \
  --left-ip 172.28.60.200 \
  --right-ip 172.28.60.201 \
  --arm left \
  --backend rbpodo \
  --mode read_state \
  --duration-sec 10 \
  --rate-hz 100 \
  --artifact-dir artifacts/rb_backend_ablation/rbpodo_read_100hz \
  --i-understand-this-connects-to-real-controller
```

## rbscript TCP Connect-Only Probe

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_RBSCRIPT_TCP=1 \
python3 scripts/rb_backend_ablation.py \
  --left-ip 172.28.60.200 \
  --right-ip 172.28.60.201 \
  --arm left \
  --backend rbscript_tcp \
  --mode connect_only \
  --duration-sec 10 \
  --rate-hz 10 \
  --artifact-dir artifacts/rb_backend_ablation/rbscript_connect_only \
  --i-understand-this-connects-to-real-controller
```

## rbscript TCP Read-State Probe

Use this command to confirm gating and artifact behavior, but do not interpret a
real-controller unsupported result as worse read-state performance. Until a
verified Rainbow 5001 parser is added, `rbpodo` remains the state-read reference
backend.

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_RBSCRIPT_TCP=1 \
python3 scripts/rb_backend_ablation.py \
  --left-ip 172.28.60.200 \
  --right-ip 172.28.60.201 \
  --arm left \
  --backend rbscript_tcp \
  --mode read_state \
  --duration-sec 10 \
  --rate-hz 100 \
  --artifact-dir artifacts/rb_backend_ablation/rbscript_read_100hz \
  --i-understand-this-connects-to-real-controller
```

Expected unsupported real-controller data-port summaries include
`rbscript_tcp_data_port_mode: real_controller_unsupported`,
`read_state_capability: unsupported`, `comparable: false`, and
`not_comparable_reason`. No valid joint state is published from unsupported
payloads.

## Raw 5001 Data-Port Capture

Use raw capture when collecting evidence for a future parser. This is read-only:
it connects to TCP data port `5001`, sends `reqdata`, writes raw bytes/text, and
does not parse or mark state valid.

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_RBSCRIPT_TCP=1 \
python3 scripts/rainbow_rate_probe.py \
  --ip 172.28.60.200 \
  --backend rbscript_tcp \
  --mode read_state \
  --rates 1 \
  --duration-sec 1 \
  --artifact-dir artifacts/rate_probe/rbscript_raw_data_port_left \
  --capture-raw-data-port \
  --i-understand-this-connects-to-real-controller
```

Capture artifacts:

- `raw_data_port_capture.bin`
- `raw_data_port_capture.txt`
- `raw_data_port_capture.json`

Do not infer joint offsets or units from these files until the layout is
verified by controller documentation, captured fixtures with known joint values,
or accepted source evidence.

## rbscript ACK Probe

Only use this mode when you have a verified no-motion Rainbow script command.
The tool will not invent one, and it rejects obvious motion-capable tokens such
as `move_`, `servo`, `pgmode`, `joint`, and `jnt[`.

Use a 1 Hz sanity step before any sweep. The command placeholder must be
replaced with a site-verified no-motion command such as a controller status,
echo, or other documented no-op command that has already been proven not to
change controller mode, speed, joint targets, task state, or robot state.

Default `command_ack_no_motion` opens a fresh socket per sample for backward
compatibility. Use `--persistent-socket` for apples-to-apples comparison with
the C++ `RbscriptTcpBackend`, which keeps its command socket open during healthy
operation and reconnects only after transport errors.

1 Hz sanity example:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_RBSCRIPT_TCP=1 \
python3 scripts/rb_backend_ablation.py \
  --left-ip 172.28.60.200 \
  --right-ip 172.28.60.201 \
  --arm left \
  --backend rbscript_tcp \
  --mode command_ack_no_motion \
  --rbscript-no-motion-command '<verified no-motion command>' \
  --duration-sec 5 \
  --rate-hz 1 \
  --persistent-socket \
  --artifact-dir artifacts/rb_backend_ablation/rbscript_ack_1hz_sanity \
  --i-understand-this-connects-to-real-controller
```

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_RBSCRIPT_TCP=1 \
python3 scripts/rb_backend_ablation.py \
  --left-ip 172.28.60.200 \
  --right-ip 172.28.60.201 \
  --arm left \
  --backend rbscript_tcp \
  --mode command_ack_no_motion \
  --rbscript-no-motion-command '<verified no-motion script command>' \
  --duration-sec 10 \
  --rate-hz 100 \
  --persistent-socket \
  --artifact-dir artifacts/rb_backend_ablation/rbscript_ack_100hz \
  --i-understand-this-connects-to-real-controller
```

## Artifacts

Each run writes:

- `summary.json`
- `samples.csv`
- `responses.jsonl`
- `errors.jsonl`
- `timing_histogram.png`
- `loop_interval.png`
- `state_age.png`, when state age data exists
- `README.txt`

## rbscript TCP Servo J No-Op Acceptance

This is the apples-to-apples controller-simulation counterpart to the rbpodo
`servo_j_noop` acceptance path. It sends:

```text
move_servo_j(jnt[current_q], t1, t2, gain, alpha)
```

The script does not infer a physical-motion target. The target is exactly the
current joint vector from one explicit source:

- `--q-current-deg q0,q1,q2,q3,q4,q5`
- `--q-current-from-rbpodo`, which uses rbpodo only for state acquisition while
  the command path remains rbscript TCP
- rbscript data port, only when it returns the supported
  `rbscript_tcp_state_v1` JSON fixture schema

Because real controller data-port parsing is incomplete, prefer
`--q-current-deg` or `--q-current-from-rbpodo` until data-port parsing has its
own acceptance evidence. The script fails instead of faking state when the data
port is unsupported.

100 Hz ACK-on example:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBSCRIPT_TCP=1 \
RB_ALLOW_RBSCRIPT_TCP_MOTION=1 \
python3 scripts/rbscript_servo_acceptance.py \
  --config rb_servo_server/config/local/dual_real_rbscript_100hz_ack_sim_noop.yaml \
  --arm left \
  --mode servo_j_noop \
  --profile 100hz_ack \
  --duration-sec 10 \
  --q-current-deg '<q0,q1,q2,q3,q4,q5>' \
  --artifact-dir artifacts/rbscript_acceptance/servo_j_noop_100hz_ack_left \
  --allow-motion \
  --i-understand-this-connects-to-real-controller
```

200 Hz ACK-on example:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBSCRIPT_TCP=1 \
RB_ALLOW_RBSCRIPT_TCP_MOTION=1 \
python3 scripts/rbscript_servo_acceptance.py \
  --config rb_servo_server/config/local/dual_real_rbscript_200hz_ack_sim_noop.yaml \
  --arm left \
  --mode servo_j_noop \
  --profile 200hz_ack \
  --duration-sec 10 \
  --q-current-deg '<q0,q1,q2,q3,q4,q5>' \
  --artifact-dir artifacts/rbscript_acceptance/servo_j_noop_200hz_ack_left \
  --allow-motion \
  --i-understand-this-connects-to-real-controller
```

200 Hz ACK-off example:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBSCRIPT_TCP=1 \
RB_ALLOW_RBSCRIPT_TCP_MOTION=1 \
python3 scripts/rbscript_servo_acceptance.py \
  --config rb_servo_server/config/local/dual_real_rbscript_200hz_no_ack_sim_noop.yaml \
  --arm left \
  --mode servo_j_noop \
  --profile 200hz_no_ack \
  --duration-sec 10 \
  --q-current-deg '<q0,q1,q2,q3,q4,q5>' \
  --artifact-dir artifacts/rbscript_acceptance/servo_j_noop_200hz_no_ack_left \
  --allow-motion \
  --allow-ack-disabled \
  --i-understand-this-connects-to-real-controller
```

The acceptance artifacts are:

- `summary.json`
- `summary.csv`
- `samples.csv`
- `command_packets.jsonl`
- `responses.jsonl`
- `raw_config.yaml`
- `safety_preflight.json`
- `timing_ack_duration.png`, when ACK timings and matplotlib are available
- `loop_interval.png`, when multiple samples are available

For ACK-on profiles, review `controller_acceptance_observed_count` and
`ack_wait_duration_us`. For ACK-off profiles, review
`send_acceptance_semantics_distribution.socket_send_only`; it must not be
reported as controller acceptance.

## Compare Runs

```bash
python3 scripts/compare_backend_ablation.py \
  artifacts/rb_backend_ablation/rbpodo_read_100hz/summary.json \
  artifacts/rb_backend_ablation/rbscript_read_100hz/summary.json \
  --csv artifacts/rb_backend_ablation/comparison.csv
```

The comparison table reports backend, mode, requested rate, achieved rate,
persistent socket mode, reconnect count, read-state capability, comparability,
success rate, latency percentiles, timeout count, error count, and a
not-comparable reason when present. `rbscript_tcp` read-state rows with
`read_state_capability: unsupported` are shown as not comparable instead of as
slower performance results.

## Decision Report

Use `scripts/generate_backend_comparison_report.py` when combining partial
evidence from several comparison runs. The report classifies each evidence row
as one of:

- `measured_and_comparable`
- `measured_not_comparable`
- `unsupported`
- `not_yet_run`

Example:

```bash
python3 scripts/generate_backend_comparison_report.py \
  artifacts/rb_backend_ablation/20260529_101652_left/rbpodo_read_state_sweep/summary.json \
  artifacts/rb_backend_ablation/20260529_101652_left/rbscript_read_state_sweep/summary.json \
  artifacts/rb_backend_ablation/20260529_101652_left/rbscript_ack_no_motion_sweep/summary.json \
  --require-default-missing-rows \
  --output-md artifacts/rb_backend_ablation/20260529_101652_left/backend_comparison_decision_report.md \
  --output-csv artifacts/rb_backend_ablation/20260529_101652_left/backend_comparison_decision_evidence.csv \
  --output-json artifacts/rb_backend_ablation/20260529_101652_left/backend_comparison_decision.json
```

The decision report intentionally keeps capability groups separate:

- `connect_only`
- `read_state`
- `command_ack_no_motion`
- `servo_j_noop`

`rbscript_tcp command ACK test is not ServoJ test` is emitted whenever
rbscript no-motion ACK evidence is present. Do not compare that row against
rbpodo ServoJ acceptance. `rbscript_tcp read_state unsupported` is emitted when
the real Rainbow data-port parser is unavailable. `rbpodo ServoJ ACK-on/off not
yet run` is emitted until actual rbpodo `servo_j_noop` summaries are supplied.

Decision rules:

- `rbpodo` is the primary backend candidate only when read-state success is at
  least `0.99` through `100 Hz`, achieved rate is stable, read-only diagnostic
  state streaming works, and no command-path evidence contradicts it.
- `rbscript_tcp` remains experimental when real data-port read-state is
  unsupported, command ACK has timeout/error/unrecognized responses, or ServoJ
  no-op apples-to-apples evidence is missing.

The JSON report includes:

- `primary_backend_recommendation`
- `experimental_backend_status`
- `next_required_experiments`

The CSV is the evidence table with artifact paths for audit traceability.

## Backend Comparison Matrix

`scripts/run_backend_comparison_matrix.py` runs a YAML-defined comparison
matrix and collects the child script outputs into one report. The default matrix
behavior is read-only/no-motion only. It does not set any `RB_ALLOW_*`
environment variables; export those yourself only for the real-controller stage
you intend to run.

Example:

```bash
python3 scripts/run_backend_comparison_matrix.py \
  --matrix configs/backend_compare/rbpodo_vs_rbscript_left.yaml \
  --artifact-root artifacts/backend_compare/20260529_left \
  --max-workers 1 \
  --i-understand-this-connects-to-real-controller
```

Dry-run first:

```bash
python3 scripts/run_backend_comparison_matrix.py \
  --matrix configs/backend_compare/rbpodo_vs_rbscript_left.yaml \
  --artifact-root artifacts/backend_compare/dry_run_left \
  --max-workers 1 \
  --dry-run
```

Matrix entries may use:

- `script: rb_backend_ablation` for `connect_only`, `read_state`, and rbscript
  `command_ack_no_motion`
- `script: rainbow_rate_probe` for rate sweeps of `read_state` and rbscript
  `ack_no_motion`
- `script: rbpodo_servo_acceptance` for rbpodo controller-simulation
  `servo_j_noop`
- `script: rbscript_servo_acceptance` for rbscript controller-simulation
  `servo_j_noop`

Disabled experiments are skipped unless `--include-disabled` is passed.
Capability mismatches, such as `rbpodo` with `ack_no_motion`, are summarized as
`status: unsupported` rather than failed runs. Safety preflight failures,
missing tools for enabled experiments, or child script crashes stop the matrix.

Controller-simulation ServoJ no-op entries are never part of the default
read-only path. To include them, the matrix entry must be enabled and set
`allow_motion: true`, and the runner must be called with:

```bash
--allow-servo-j-noop-simulation
```

The child acceptance scripts still enforce their own config, confirmation, and
environment gates.

Matrix output under `--artifact-root`:

- `matrix_resolved.yaml`
- one subdirectory per experiment, with `experiment_command.txt` and
  `experiment_status.json`
- child-script artifacts such as `summary.json`, when executed
- `backend_comparison_summary.csv`
- `backend_comparison_summary.json`
- `backend_comparison_report.md`

## Interpreting Results

A lower ACK or read latency is not permission to move hardware. Before any
motion-oriented follow-up, first establish reliable read-only state acquisition,
zero unexpected controller errors, zero timeout bursts, and stable loop
intervals at the intended rate.

`rbscript_tcp` may reduce client/library overhead relative to `rbpodo`, but the
controller parser, ACK path, data path, and motion scheduler can still dominate.
Measure 50/75/100/125/150/200 Hz instead of assuming that a nominal servo period
is stable. `disable_waiting_ack` can increase apparent throughput, but it hides
immediate ACK errors and should not be used for acceptance evidence.

`rt_script` is future work and experimental. It is not part of this comparison
runbook, and it must not be used as an undocumented controller setting.

## Rainbow Rate Probe

`scripts/rainbow_rate_probe.py` runs an explicit rate sweep over the external
command/read path. It is still no-motion by default and does not set `pgmode
real`, does not send `move_servo_j` by default, does not disable waiting ACK,
and does not bypass controller limits.

Example rbscript TCP ACK/read sweep shape:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_RBSCRIPT_TCP=1 \
python3 scripts/rainbow_rate_probe.py \
  --ip 172.28.60.200 \
  --backend rbscript_tcp \
  --mode ack_no_motion \
  --rates 10,20,50,75,100,125,150,200 \
  --duration-sec 10 \
  --rbscript-no-motion-command '<verified no-motion script command>' \
  --persistent-socket \
  --artifact-dir artifacts/rate_probe/left_rbscript_ack \
  --i-understand-this-connects-to-real-controller
```

The probe refuses `ack_no_motion` unless the no-motion command is explicit. It
also rejects obvious motion-capable tokens such as `move_`, `servo`, `pgmode`,
`joint`, and `jnt[`. Use `read_state` when you only want data-port timing. For
rbscript ACK comparison, run both reconnect-per-sample and `--persistent-socket`
artifacts so response framing, reconnect overhead, and controller parser
behavior can be separated.

Artifacts:

- `summary.json`
- `summary.csv`
- `samples_<rate>.csv`
- `responses_<rate>.jsonl`
- `ack_latency_by_rate.png`
- `success_rate_by_rate.png`
- `loop_interval_by_rate.png`

The rate probe reports M-code counts for `M561`, `M568`, `M569`, and `M570`
when those strings appear in controller responses. Treat these as controller
diagnostics, not as reasons to unlock or bypass limits. For example, errors
such as previous motion not finished or Servo J parameter errors mean the
requested external path is not stable at that setting.

Persistent-socket response framing reads CRLF/LF-terminated lines, records the
primary response plus extra drained lines in `responses*.jsonl`, counts stale
lines found before the next command, and records write/read timing separately.
Stale or unrecognized responses are analysis evidence, not a reason to suppress
safety gates.

Interpretation caveats:

- ACK success at 200 Hz does not mean safe 200 Hz motion.
- Servo J `t1` lower bounds do not prove external TCP command-path stability.
- `rbpodo` and `rbscript_tcp` may differ in client overhead, but controller
  parser and ACK behavior may dominate at high rates.
- `disable_waiting_ack` hides immediate ACK/error behavior and is not supported
  by this rate probe task.
