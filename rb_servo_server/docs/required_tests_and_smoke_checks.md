# Required Tests and Smoke Checks

Architect-level checklist for the unattended real-readiness safety loop. Every
implementation iteration (any Ralph commit, any critic re-review) must pass
every gate below before the iteration is allowed to advance. Gates are
grounded in the test/smoke surface that already exists in this repo; if a
gate cannot be satisfied with current infrastructure, the gap is called out
explicitly so it can be filled rather than papered over.

See also: [`docs/testing.md`](testing.md) (current minimum recipe),
[`docs/fail_safe_policy.md`](fail_safe_policy.md) (invariants under test),
[`docs/commit_slicing_advice.md`](commit_slicing_advice.md) (which layer a
change belongs to), and [`.omx/context/autonomous-real-readiness-loop-20260514T110745Z.md`](../.omx/context/autonomous-real-readiness-loop-20260514T110745Z.md)
(loop ground truth).

## 1. Gate ordering

Run gates in this order. Earlier gates are cheaper and isolate failures
upstream; never skip ahead.

| # | Gate | What it proves | Blocking? |
|---|---|---|---|
| G1 | Clean configure + build | Source compiles; CMake graph is consistent | Yes |
| G2 | `ctest` (unit) | `tests/test_safety_policy.cpp` invariants still hold | Yes |
| G3 | Mock smoke (server only) | Server starts under `config/dual_mock.yaml` and exits cleanly | Yes |
| G4 | Mock smoke (server + driver) | Sine driver completes a short loop without faulting the server | Yes |
| G5 | Real-mode env-var guard | Server refuses to launch under `config/dual_real.yaml` without `RB_ALLOW_REAL_ROBOT=1` | Yes |
| G6 | Touched-area regression | A behavior changed in this iteration is covered by a new or strengthened assertion in `tests/` | Yes for any behavioral change |
| G7 | Doc parity | Any policy or behavior change is reflected in `docs/fail_safe_policy.md`, `docs/network_protocol.md`, or `docs/testing.md` | Yes for any policy change |

Gates G1–G5 are exactly the set the loop context document already lists as
"fresh verification" (lines 27–31). They are the floor, not the ceiling.

## 2. Exact commands (run from repo root)

### G1 — configure + build

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j
```

Failure modes that block the iteration:

- non-zero exit from either command
- new warnings introduced on files this iteration touched (treat as failure
  until a critic explicitly waives them)
- `FetchContent` re-downloading `nlohmann_json` on a previously-built tree
  (signals an unintended dependency churn)

### G2 — unit tests

```bash
ctest --test-dir build --output-on-failure
```

Today the suite is one binary (`test_safety_policy`) with three test
functions:

- `testCommandValidation`
- `testEmergencyWinsAndResetDoesNotRun`
- `testSendFailureDoesNotAdvancePreviousTarget`

A green `ctest` is necessary but **not** sufficient. See §4.

### G3 — mock server smoke

```bash
timeout 1s ./build/rb_servo_server --config config/dual_mock.yaml
```

Pass criteria:

- exit code 0 (clean shutdown on timeout) or 124 (timeout fired without
  fault). Any other exit code, any unhandled exception, any
  `[0,0,0,0,0,0]` joint target in stderr/stdout is a fail.

### G4 — mock server + sine driver

In one shell:

```bash
./build/rb_servo_server --config config/dual_mock.yaml
```

In another:

```bash
python3 tools/send_dual_joint_sine.py --rate 30 --amp-deg 2 --freq 0.2
```

Pass criteria:

- sine driver sends `ArmMotion` before its first `JointTarget`
- server stays in `ConnectedHold` until `ArmMotion`, then accepts targets
- after a manually injected `EmergencyStop` (`python3 tools/send_emergency_stop.py`)
  the server latches and does **not** resume on `ResetFault` alone — a fresh
  `ArmMotion` is required (this is the invariant `testEmergencyWinsAndResetDoesNotRun`
  locks at the unit level; the smoke is the integration-level check)
- log output contains no `[0,0,0,0,0,0]` synthesized targets

### G5 — real-mode env-var guard

```bash
./build/rb_servo_server --config config/dual_real.yaml
```

This **must** fail-fast before opening a socket to hardware. Specifically,
without `RB_ALLOW_REAL_ROBOT=1` and the full real-mode guard set documented
in `docs/fail_safe_policy.md` §Real mode guard, the process exits non-zero.
A green build that does not fail this gate is **not** Go.

### G6 — touched-area regression

For any iteration that changes runtime behavior:

1. Identify the smallest behavior the change establishes or modifies (the
   `Directive:` line of the commit is a good anchor).
2. Confirm that `tests/test_safety_policy.cpp` (or a new file under
   `tests/`) asserts that behavior, either by reading existing assertions
   or by adding a new one.
3. Run the new/strengthened assertion via `ctest --output-on-failure
   --test-dir build -R <name>` and capture the output in the commit's
   `Tested:` trailer.

If a behavioral change ships without a corresponding test delta, the
critic must reject it. This rule is non-negotiable per the loop context
constraint "Add or strengthen tests for every policy or behavioral change."

### G7 — doc parity

If the iteration touches:

- a failure-handling row in `docs/fail_safe_policy.md` → update the table in
  the same commit (or the immediately preceding doc-only commit).
- the wire protocol in `docs/network_protocol.md` → update it.
- the test recipe in `docs/testing.md` → update it.

Doc drift is treated as a real bug, not a stylistic issue.

## 3. What "smoke" means here

In this repo, "smoke" is intentionally narrow:

- It runs the server binary against a **mock** backend (`config/dual_mock.yaml`).
- It never touches `config/dual_real.yaml` against real hardware. The real
  config is only exercised via the env-var guard check (G5) to prove the
  guard latches.
- It is short (≤ 1 s server, ≤ a few seconds of driver traffic). The intent
  is to catch startup, parser, and state-machine regressions — not soak
  testing or performance.
- It is not a substitute for unit assertions. Anything that can be locked
  at the unit level (`tests/test_safety_policy.cpp`) must be — smoke is
  the second net, not the primary one.

A change that "passes smoke" but has no covering unit assertion is at G6
status `unverified`, not `pass`.

## 4. Known coverage gaps

These are areas where the current suite does **not** catch regressions. An
iteration that touches one of these areas must lift the relevant gap into
a unit test before the critic can say Go.

- **Parser drop semantics for malformed payloads.** `command_server.cpp`
  drops malformed packets per `docs/fail_safe_policy.md` §Failure handling,
  but there is no unit assertion that the command buffer is unchanged after
  a malformed frame. `testCommandValidation` checks valid+invalid command
  classification, not buffer state.
- **`timeout_sec <= 0` rejection.** Listed in the failure table but not
  asserted in `tests/`.
- **Tracking-error policies (`snap_to_actual` in mock, `fault_latch` in real).**
  No unit-level coverage; relies entirely on integration smoke today.
- **`safety.stop_both_arms_on_single_arm_error`** path: never exercised in
  unit tests; only the single-arm send-failure case is covered
  (`testSendFailureDoesNotAdvancePreviousTarget`).
- **Realtime priority setup.** No CI-friendly check; verifying it requires
  privileged execution which the loop context (§Unknowns) explicitly rules
  out. Critic must continue to treat this as "operator-verified, not
  CI-verified" and not block on it.
- **`tools/send_*.py` operator scripts.** No automated coverage; relied on
  manually for G4.

## 5. Lint / format / static analysis

There is **no** `.clang-format`, `.clang-tidy`, or static-analysis target in
the current build system. Do not invent one inside this loop — a new
formatting rule is its own change, with its own commit, its own review, and
its own iteration. Mixing format churn into a safety-policy iteration
violates `docs/commit_slicing_advice.md` §2.

## 6. Evidence the critic should see in each iteration

Every Ralph commit message should end with a Lore-style trailer that lets a
critic verify gates without re-running them:

```
Tested:
  - G1 cmake configure + build: pass
  - G2 ctest: 3 passed, 0 failed
  - G3 mock-server smoke: clean exit
  - G4 mock + sine driver: ArmMotion gate honored, no zero synth
  - G5 real-mode guard: fails without RB_ALLOW_REAL_ROBOT (expected)
  - G6 touched-area regression: <test name> added/strengthened
Not-tested:
  - real hardware (out of scope per loop context)
```

If a gate is skipped, the trailer must say which one and why; an unexplained
skip is a critic blocker.
