# cm_bridge/upstream — changes to controller-manager that are NOT yet upstream

The submodule is consumed read-only and never committed to from this repo (`AGENTS.md`,
`submodules/README.md`). When the bridge needs something INSIDE controller-manager, the change is
made in the submodule working tree, exported here as a patch, and sent upstream as a PR to
`PLAIF-dev/controller-manager` (owner: 박천만님). Until it lands, this directory is the durable
record: a fresh `git submodule update` discards the working tree, and the patch is how it comes
back.

| patch | what | status |
|---|---|---|
| `0002-follow-commit-steps-aux-step-events.patch` (**CUMULATIVE**: contains 0001; apply this one alone on a clean tree) | FollowUnit **commit_steps** (`follow.yaml`; adopt a fresh chunk only at a multiple of N played deltas — the controller becomes the pacer of an N-step commit), per-delta **aux** riding the chunk (`<side>/cmd/follow_aux`, JointState, paired by stamp at adoption; the policy's per-step gripper target), and **step events** `<side>/act/follow_step` (JointState per period boundary: which delta of which chunk started/finished, adoption, aux) drained from an RT ring by the IPC executor. | applied in the working tree 2026-08-19; **PR pending** |
| `0001-recorder-schema4-follow-observability-ext-scalars.patch` | DataRecorder **schema 4** (append-only columns): `mono_ns` (CLOCK_MONOTONIC per tick), the stream follower's telemetry per tick (`fol_*`: cmd/ref pose + rpy, in_v/in_w, lag/gap, gate, adopted-chunk stamp/idx/n, playing), overlay deviation (`dev_*`), and a new **external-scalars ingress** `<side>/cmd/ext_scalars` (sensor_msgs/JointState → `ExtScalarSlot`, recorded as `ext0..7` + stamp/seq). `FollowChunk` carries the PoseArray `header.stamp` (`stamp_ns`) so a capture joins to the publisher's own log. `FollowUnit::Telemetry` gains the fields the recorder needs. | applied in the working tree 2026-08-18; **PR pending** |

## Apply / re-apply

```bash
cd submodules/controller-manager
git apply --check ../../cm_bridge/upstream/0002-*.patch   # cumulative (includes 0001); must be silent
git apply         ../../cm_bridge/upstream/0002-*.patch
cd ../.. && tools/cm_local_setup.sh                       # rebuild (or colcon build --packages-select controller_manager)
./cm_bridge/tests/record_gate.sh                          # the recorder gate: PASS required
```

`run_cm_stack.sh` checks that the BUILT binary carries schema 4 (`fol_chunk_stamp_ns`) AND the
follow step events (`act/follow_step`) and refuses `real`/`sim` if not — a capture from an
unpatched binary would silently lack every column the replay tool is for, and the bridge's commit
pacer would run degraded with the gripper never commanded.

## Design notes for the PR (why the change looks the way it does)

- **Append-only.** Every new column is at the row end; the header version goes 3 → 4; the two
  upstream readers (`tools/_logread.py`, `FtIdentify`) keep working exactly as their comments say
  (dtype-from-header / size-gated).
- **Nothing on the RT path acts on the new inputs.** `ExtScalarSlot` is read only by
  `Arm::record_row`; the ext_scalars callback is deposit-only like jog/ft_config/dio. The chunk
  stamp is carried, never interpreted.
- **One extra FK per tick while Follow is active** (`kine_.forward(cmd_.q)` for `gap_mm`), microseconds.
- **The join contract**: a publisher stamps `cmd/follow` and `cmd/ext_scalars` with THIS host's
  `CLOCK_MONOTONIC` (== `rt::now_ns`), so `fol_chunk_stamp_ns` / `ext_stamp_ns` and `mono_ns` are
  directly comparable to the publisher's own records. cm_bridge does exactly that (its sidecar).
- Verified: `cm_bridge/tests/record_gate.sh` (isolated SILS instance) — schema-4 columns
  populated, 2.000 ms cadence, 29/29 adopted-chunk stamps join the sidecar, gripper command levels
  and their stamps ride the row within 1 ms.

## 0002 design notes for the PR

- NOTE (later 2026-08-19): robotics_lab now runs `commit_steps: 1` again — the bridge subdivides
  over-envelope steps into sub-deltas and paces the commit in POLICY steps itself, so a
  controller commit counted in sub-deltas would fight it. `commit_steps` stays a valid feature
  for publishers that do not subdivide.
- `commit_steps` defaults to 1 = today's REPLACE-at-every-boundary; nothing changes for teleop
  (chunks of 1) or for any deployment that does not set it. With N > 1 the unit refuses a fresh
  chunk except at `chunk_idx % N == 0` (or none held / exhausted); the latest-value slot still
  keeps only the newest deposit, so nothing queues.
- aux is a SEPARATE topic + slot because PoseArray has no room and its type is on the wire; the
  pairing rule (publisher sends aux first, same stamp; unit pairs by (stamp, n) at adoption, else
  NaN) makes a wrong pairing impossible. The unit never interprets aux.
- Step events go through a 32-deep RT ring (`FollowStepRing`) drained by `publish_motion` (250 Hz)
  — no ROS on the RT path, ≤ 4 ms latency, torn reads refused by seq check.
- Verified: `make cm-record-gate` (isolated SILS): 4 deltas played per adopted chunk, adopted
  slices 4 steps apart, 120/120 gripper commands issued from step events; the late-inference
  drill (`RECORD_GATE_LATE_EVERY=5`) shows the older candidate continued (skip 4) and the
  adopted sequence still contiguous.
