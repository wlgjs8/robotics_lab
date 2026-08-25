# Wiki Log

## [2026-07-12T08:27:46.227Z] query
- **Pages:** none
- **Summary:** Query "flow infer chunk velocity proprio delta twist Ruckig PIKA UMI" → 0 results (of 0 total)

## [2026-07-12T08:28:05.396Z] ingest
- **Pages:** flow-infer-delta-preview-controller-contract.md
- **Summary:** Created new page "Flow Infer Delta Preview Controller Contract"

## [2026-07-12T08:28:05.398Z] add
- **Pages:** flow-infer-delta-preview-controller-contract.md
- **Summary:** Created wiki page flow-infer-delta-preview-controller-contract.md

## [2026-07-12T08:28:09.600Z] lint
- **Pages:** flow-infer-delta-preview-controller-contract.md
- **Summary:** Lint: 1 issues (1 orphan, 0 stale, 0 broken, 0 contradictions)

## [2026-07-12T08:29:21.194Z] ingest
- **Pages:** pika-umi-motion-distribution-baseline.md
- **Summary:** Created new page "PIKA UMI Motion Distribution Baseline"

## [2026-07-12T08:29:21.196Z] add
- **Pages:** pika-umi-motion-distribution-baseline.md
- **Summary:** Created wiki page pika-umi-motion-distribution-baseline.md

## [2026-07-12T08:29:21.421Z] lint
- **Pages:** pika-umi-motion-distribution-baseline.md
- **Summary:** Lint: 1 issues (1 orphan, 0 stale, 0 broken, 0 contradictions)

## [2026-08-25T10:03:33.438Z] ingest
- **Pages:** rainbow-control-box-servo-j-latency-fw-v8-6-1.md
- **Summary:** Created new page "Rainbow Control Box Servo J Latency (firmware v8.6.1)"

## [2026-08-25T10:03:33.438Z] add
- **Pages:** rainbow-control-box-servo-j-latency-fw-v8-6-1.md
- **Summary:** Created wiki page rainbow-control-box-servo-j-latency-fw-v8-6-1.md

## [2026-08-25T10:03:33.438Z] lint
- **Pages:** rainbow-control-box-servo-j-latency-fw-v8-6-1.md
- **Summary:** Lint: 0 issues (0 orphan, 0 stale, 0 broken, 0 contradictions)

## [2026-08-25T12:41:26.355Z] ingest
- **Pages:** rainbow-control-box-servo-j-latency-fw-v8-7-3.md
- **Summary:** Created new page "Rainbow Control Box Servo J Latency (firmware v8.7.3)"

## [2026-08-25T12:41:26.355Z] add
- **Pages:** rainbow-control-box-servo-j-latency-fw-v8-7-3.md
- **Summary:** Created wiki page rainbow-control-box-servo-j-latency-fw-v8-7-3.md

## [2026-08-25T12:41:26.355Z] lint
- **Pages:** rainbow-control-box-servo-j-latency-fw-v8-7-3.md
- **Summary:** Lint: 0 issues (0 orphan, 0 stale, 0 broken, 0 contradictions)

## [2026-08-25T21:36:21.000Z] update
- **Pages:** rainbow-control-box-servo-j-latency-fw-v8-7-3.md
- **Summary:** Re-measured on the current stack (334 s / 167,155 ticks). New section: the box ignores the servo_j stream for ~254 ms after connect while reporting RBACK fill 0, even at activation stage 6 -- fixed structurally by not streaming until the first motion command (controller-manager's rule), so the startup backlog goes 131 ticks -> none, fill at exactly 5 94.1 -> 99.4 %, drain phase 2.6 -> 0.1 %. Results table gains a CURRENT row: end to end 22.3 ms, 2 ms slower than 2026-08-25 and deliberately so -- the pipelined non-blocking state read costs 1 tick, now broken out beside transport and the mailbox hop. Fifth host-side bug recorded (suppressing an action without stepping its state machine -> 42 s servo silence). Worker state-age creep RESOLVED: the cadence owner was SCHED_OTHER and unpinned; now FIFO 80 + isolated cores, zero ticks over the 8 ms budget. The LPF-off jitter item is corrected -- the quoted per-LSB figure was 1/dt^3, and jerk fails for a different, measured reason (the box holds q_actual, so the quantisation is upstream of the logger).
