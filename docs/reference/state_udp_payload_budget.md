# Bounded state UDP publication

The state publisher sends one identical JSON snapshot to every configured UDP
consumer. Control and safety state, including the command TCP pose and force
reference epoch used by `servo_command` proprioception, must not disappear just
because optional collision witness geometry becomes large.

## Wire contract

The publisher uses a **64,000-byte soft budget** for the complete serialized
snapshot. This leaves 1,507 bytes below the IPv4 UDP payload maximum of 65,507
bytes. These are transport limits, not motion or collision configuration limits.
The current implementation remains IPv4 and does not introduce fragmentation,
compression, a second topic, or client-side snapshot merging.

When a complete snapshot exceeds the soft budget, only
`self_collision.near_pairs` is reduced. The monitor's full collision list,
Jacobians, projection, safety verdicts, and CSV telemetry are unchanged. Every
other existing JSON field is retained, including the geometry manifest and the
legacy duplicate `last_cartesian_solve` block. Pose/quaternion values retain their
existing numeric precision.

Selection gives priority to pairs whose clearance is below **their own**
`d_hard_m`; among the same priority it favors lower clearance. Selected pairs
retain their original relative order. A fitting snapshot preserves its entire
original witness list and ordering. The budget uses serialized UTF-8 JSON bytes,
including escaped strings, rather than a fixed pair or character count.

`self_collision` adds:

| Field | Meaning |
| --- | --- |
| `near_pairs_total` | Number of visualization pairs available before the wire budget. |
| `near_pairs_published` | Number present in this datagram. |
| `near_pairs_truncated` | Some visualization pairs were omitted for the wire budget. |
| `near_pairs_hard_total` | Available visualization pairs below their own hard floor. |
| `near_pairs_hard_published` | Such pairs present in this datagram. |
| `near_pairs_hard_truncated` | At least one such pair was omitted. |

The existing `near_count` remains the monitor list count. It is **not** necessarily
`near_pairs_total`: the servo loop first applies its visualization distance filter.
Neither `near_pairs_total` nor the hard-pair totals claim to count pairs outside
that pre-existing visualization filter.

If the aggregate verdict is violated and `near_pairs_hard_truncated` is true,
the GUI highlights all known collision groups conservatively. A surviving hard
pair must not make an omitted arm or structure look clear. Truncating only
non-violating witnesses preserves the existing per-group highlight behavior.
The witness rendering itself remains a partial view of the published pairs.

## Core overflow and diagnostics

If the core snapshot alone exceeds 64,000 bytes but still fits 65,507 bytes, it is
sent intact without witnesses. If it exceeds **65,507 bytes**, no UDP send is
attempted for that snapshot. `serializeSnapshot()` still returns the complete
core, so callers can inspect the actual oversize; it does not conceal the problem
by deleting core fields or returning an apparently valid partial state. Consumers
retain their existing stale-state handling. No stale-state threshold is widened.

Each snapshot includes `state_publication`:

- `payload_bytes`: exact size of this serialized UTF-8 JSON, including this field.
- `payload_budget_bytes` / `udp_max_payload_bytes`: the soft and hard limits.
- `oversize_dropped_total`: publisher snapshots rejected because the core exceeded
  the hard limit; increments once per snapshot, not once per destination.
- `send_errors_total`: failed `sendto` attempts across all destinations.
- `last_error_time_ns` / `last_error_code`: monotonic publisher time and numeric
  errno of the latest core overflow or send error; zero before an error.

Counts are copied into later snapshots. A failure cannot report itself in the
packet that was not sent. Logs report the first error, continued failures at
most once per five seconds, and recovery, including monotonic time, servo tick,
byte size/error details and consecutive counts. Core overflow logs also include
the source `loop_start_time_ns`; successful size recovery gives the number of
skipped snapshots and elapsed duration. Per-destination send recovery is logged
separately. The old one-warning-per-endpoint-for-process-lifetime behavior is
removed.

`chunk_execution_profiles[].output_smd` additionally publishes the selected
output filter settings, including `velocity_ff_linear_gain`; this is descriptive
metadata and does not select or tune the filter.

## Evidence and limits

The 2026-09-06 15:21:25 rollout repeated a frozen state endpoint around +82.8 to
+83.13 seconds while server motion continued. The server log also contained
`Message too long` for all four destinations, but those older warnings had no
timestamps and occurred only once per endpoint. Their exact temporal relation
cannot be reconstructed.

A passive, hardware-free C++ probe of the actual serializer and tracked config
reproduced the size failure with an explicitly populated, full-precision fixture:
16 witnesses used 62,262 bytes, 24 used 65,254, and 32 used 68,246 before this fix.
This fixture is **not** a captured live packet. The CSV monitor count reached 28
throughout +81.857–83.729 seconds, including the frozen-state interval, but the
actual pre-send byte size and visualization count were not logged. The available
records establish a concrete transport defect and a plausible stale-state path,
not the exact byte composition of the lost packets or the cause of the rollout's
persistent low-frequency motion.

Regression coverage in `test_state_publisher` checks serialized byte limits,
unchanged core JSON, urgency and order, escaped/UTF-8 strings, explicit incomplete
hard-pair metadata, actual loopback UDP fanout, and core-overflow drop/recovery
counts. `SelfCollisionRedGroupsTest` checks the conservative GUI fallback. These
are hardware-free communication tests, not a new robot rollout or model test.

The post-fix probe retained16 active-fixture witnesses at63,390B and bounded
24/32/256 requested witnesses to63,763/63,763/63,764B respectively. Root ran the
state publisher C++ regression target successfully (0.11s), including real local
UDP fanout, and the GUI suite passed369 tests (2.98s).

The reproducible probe and incident artifacts are under
`outputs/chunk_review_20260906_152125/udp/`. Large unbounded core configuration or
error strings can still make a snapshot impossible to send; that condition is
now explicit. A future split core/diagnostic protocol would require versioned
client dispatch/merge and late-join manifest handling, rather than quietly
omitting fields from today's complete-snapshot contract.
