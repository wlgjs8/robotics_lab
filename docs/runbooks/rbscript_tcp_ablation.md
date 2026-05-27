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

## Safety

This runbook is for read-only or no-motion probes first. Do not set
`send_servo_commands: true`, do not set `pgmode real`, and do not run high-rate
real motion as part of this task.

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

Real Cartesian motion remains separately blocked by `RB_ALLOW_REAL_CARTESIAN=1`
and is outside this rbscript comparison runbook.

## Staged Acceptance

Run stages in order and stop at the first unexpected controller error, timeout
burst, parse failure, or reconnect instability:

1. `connect_only`: establish no-motion connectivity and reconnect behavior.
2. `read_state`: validate state/data acquisition without motion.
3. `ack_no_motion`: measure script command ACK latency using a verified
   no-motion command.
4. simulation-mode `servo_j` probe: future work only, and only when simulation
   mode can be set and verified without real robot motion.
5. tiny real joint motion: future work only after separate approval, operator
   setup, and a motion-specific runbook.

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

The comparison is transport/client overhead, not the controller internal servo
loop limit. A 100 Hz or 200 Hz ACK result must be interpreted with controller
ACK/error behavior and timeout counts.

Primary metrics:

- ACK latency p50/p95/p99/max
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

## rbscript ACK Probe

Only use this mode when you have a verified no-motion Rainbow script command.
The tool will not invent one, and it rejects obvious motion-capable tokens such
as `move_`, `servo`, `pgmode`, `joint`, and `jnt[`.

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

## Compare Runs

```bash
python3 scripts/compare_backend_ablation.py \
  artifacts/rb_backend_ablation/rbpodo_read_100hz/summary.json \
  artifacts/rb_backend_ablation/rbscript_read_100hz/summary.json \
  --csv artifacts/rb_backend_ablation/comparison.csv
```

The comparison table reports backend, mode, requested rate, achieved rate,
success rate, latency percentiles, timeout count, and error count.

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
  --rates 50,75,100,125,150,200 \
  --duration-sec 10 \
  --rbscript-no-motion-command '<verified no-motion script command>' \
  --artifact-dir artifacts/rate_probe/left_rbscript_ack \
  --i-understand-this-connects-to-real-controller
```

The probe refuses `ack_no_motion` unless the no-motion command is explicit. It
also rejects obvious motion-capable tokens such as `move_`, `servo`, `pgmode`,
`joint`, and `jnt[`. Use `read_state` when you only want data-port timing.

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

Interpretation caveats:

- ACK success at 200 Hz does not mean safe 200 Hz motion.
- Servo J `t1` lower bounds do not prove external TCP command-path stability.
- `rbpodo` and `rbscript_tcp` may differ in client overhead, but controller
  parser and ACK behavior may dominate at high rates.
- `disable_waiting_ack` hides immediate ACK/error behavior and is not supported
  by this rate probe task.
