# Stop Conditions for the Unattended Safety Loop

Architect-level definition of when the
`4:architect -> ralph -> 4:critic -> ralph -> 4:critic ...` loop described in
[`.omx/context/autonomous-real-readiness-loop-20260514T110745Z.md`](../.omx/context/autonomous-real-readiness-loop-20260514T110745Z.md)
is allowed to terminate, and what each terminal state must record. The goal
is to make termination decisions reproducible: a human auditor reading the
loop's artifacts after the fact must be able to point at exactly one of the
states below and say "this is why it stopped."

See also: [`docs/required_tests_and_smoke_checks.md`](required_tests_and_smoke_checks.md)
(gates Ralph must pass) and [`docs/commit_slicing_advice.md`](commit_slicing_advice.md)
(commit shape).

## 1. Vocabulary

| Term | Meaning |
|---|---|
| **Iteration** | One full `architect -> ralph -> critic` round. The architect team produces planning artifacts; Ralph makes implementation commits; the critic team renders one verdict at the end. |
| **Loop** | The repeating sequence of iterations until one of the stop conditions in §2 fires. |
| **Critic verdict** | Exactly one of `Go`, `No-Go`, `Needs More Investigation`, written into a `.omx/debates/*.md` artifact by the critic leader. Matches the vocabulary established in [`.omx/debates/critic-plan-review.md`](../.omx/debates/critic-plan-review.md). |
| **Hard blocker** | A condition that no further Ralph iteration can resolve from inside the loop. Enumerated in §4. |

The "iteration" in this document's title (`stop condition for this
iteration`) is the *current loop instance*, identified by the context file
under `.omx/context/autonomous-real-readiness-loop-<timestamp>.md`. A new
loop instance starts whenever a new context file is created.

## 2. Terminal states

The loop must terminate when, and only when, exactly one of these fires.
The driver MUST evaluate them in this order at the end of every critic
phase; the first one that matches wins.

### S1. Critic Go

**Trigger.** The most recent critic synthesis artifact under
`.omx/debates/` ends with the literal line `Go` in its `## Conclusion`
section (or equivalent), with no follow-on `Go-blocking: Yes` items in any
policy table.

**Action.**

- Write a final critic-go record to
  `.omx/context/autonomous-real-readiness-loop-<timestamp>-go.md` (or
  append a `## Final Verdict: Go` section to the existing context file).
- Tag the loop's tip commit as `loop-go-<iso-date>` so the human auditor
  has a single git anchor.
- Exit code 0.

**Note.** A scoped `Go` (e.g. "`Go` only for Milestone 1 mock-only") is
**not** S1 unless the scope of the loop context already matches that scope.
If the critic's Go is narrower than the loop's stated objective, the
verdict counts as `Needs More Investigation` for stop-condition purposes
and the loop continues.

### S2. Hard blocker proven

**Trigger.** Any of the conditions in §4 is demonstrated with concrete
evidence (commit hash, command output, file path).

**Action.**

- Append to the loop context file under `## Hard Blocker`, citing:
  - which §4 row was tripped,
  - the evidence (file/path, command, exit code, or commit hash),
  - why the blocker cannot be moved with one more Ralph iteration.
- Mark every still-open task in the current team `failed` with `error:
  "hard_blocker: <id>"` via `omx team api transition-task-status`.
- Exit code 1.

### S3. Max iteration cap reached

**Trigger.** Iteration count `N` ≥ `MAX_ITERATIONS`.

**Default.** `MAX_ITERATIONS = 5`. Rationale: the most recent critic
synthesis identified roughly five P0 items
(`.omx/debates/critic-plan-review.md` §"Final Plan" step 3); five
iterations gives Ralph one round per item plus headroom for re-review
churn. The driver may override via env var `SAFETY_LOOP_MAX_ITERATIONS`,
but any override > 8 requires an explicit escalation note in the context
file (operator cost grows roughly linearly with iteration count and
re-reviews thrash the same surface).

**Action.**

- Append to the loop context file under `## Cap Reached`, listing the
  unresolved critic findings from the most recent verdict and the
  recommended next manual review focus.
- Tag the tip commit as `loop-capped-<iso-date>`.
- Exit code 2.

## 3. Per-iteration end checklist

At the close of every iteration, the driver must produce the following
before evaluating §2. Missing any item means the iteration is incomplete
and the loop must re-run the critic phase (not advance), because §2
evaluation requires these inputs.

1. **Build/test evidence trailer** in the most recent Ralph commit,
   covering gates G1–G7 from
   [`docs/required_tests_and_smoke_checks.md`](required_tests_and_smoke_checks.md)
   §2. Missing trailer ⇒ iteration is not closed.
2. **Critic synthesis artifact** under `.omx/debates/` with one of
   `Go`/`No-Go`/`Needs More Investigation` in `## Conclusion`. Multiple
   verdicts or none ⇒ iteration is not closed.
3. **Touched-file diff** (`git diff <prev-tip>..HEAD --stat`) saved to the
   loop context file so the next critic team can scope review without
   re-deriving it.
4. **No-zero-pose audit.** Grep the iteration's commits' diffs for any new
   path that emits `[0,0,0,0,0,0]` outside of a validated user command;
   any hit ⇒ S2 (hard blocker, see §4 row B).

## 4. Hard blockers (S2 inventory)

These conditions terminate the loop at S2 because no further unattended
iteration can resolve them. Each row names the evidence the driver must
record.

| ID | Blocker | Evidence required |
|----|---------|---|
| A | Real-hardware-required regression: a critic objection can only be falsified by running against a real robot or rbsim build. | Critic objection text + the loop context's own note (§Unknowns) confirming "Real hardware… cannot be proven on this host." |
| B | Fail-safe invariant breach lands on disk: any committed code path synthesizes `[0,0,0,0,0,0]` outside a validated user command. | Commit hash + grep hit + reference to [`docs/fail_safe_policy.md`](fail_safe_policy.md) §Invariant. |
| C | Real-mode env-var guard weakened: a commit makes G5 (real-mode guard) pass even with `RB_ALLOW_REAL_ROBOT` unset. | `./build/rb_servo_server --config config/dual_real.yaml` exit code 0 + commit hash. Per loop context constraint, this must never happen; if it does, the loop is in a state the architect/critic process cannot self-correct. |
| D | A test was deleted to make the build green. | `git log -- tests/` showing a deletion + the commit's `Tested:` trailer claiming pass. Loop context constraint. |
| E | Tooling availability: `omx team api`, tmux session, or critic-team launch fails three iterations in a row with the same infra error. | `.omx/logs/` entries showing the same error class three times. Beyond the loop's control to fix; escalate. |
| F | Policy doc and code disagree after a Ralph commit and Ralph's iteration budget for §3 doc parity is exhausted. | `docs/fail_safe_policy.md` row vs. behavior in `src/control/` differ + the critic's verdict cites doc drift. |
| G | Operator interrupt: a human writes `STOP` into `.omx/context/<loop-id>-stop` or similar agreed dead-drop. | Presence of the file. |

Anything that does **not** match a row in §4 is **not** a hard blocker.
Transient build flakes, single-iteration test failures, intermittent
network/tooling errors, and "I can't tell yet, give me another iteration"
critic verdicts are all `Needs More Investigation` (continue), not S2.

## 5. What "this iteration" must NOT do

To keep stop-condition evaluation cheap and unambiguous, the loop must not:

- Mutate `.omx/debates/critic-plan-review.md` after a verdict is written.
  The verdict is the artifact; rewriting it loses the audit trail.
- Average or vote across multiple critic verdicts within a single critic
  phase. Per [`.omx/context/critic-plan-review-20260514T091652Z.md`](../.omx/context/critic-plan-review-20260514T091652Z.md)
  the *leader* synthesizes lanes into exactly one conclusion; the loop
  driver reads that conclusion, not the lane verdicts.
- Treat a green `ctest` alone as `Go`. Tests passing satisfies §3 item 1;
  it does not produce a critic verdict.
- Roll back Ralph commits without a critic finding pinning the rollback.
  Silent reverts hide loop state from the auditor.

## 6. Artifacts the loop must leave behind

Regardless of which terminal state fires, after termination these files
must exist and be readable without re-running the loop:

- `.omx/context/autonomous-real-readiness-loop-<timestamp>.md` — with a
  final section naming the terminal state (S1/S2/S3) and the evidence.
- `.omx/debates/*.md` — every critic phase's synthesis, one file per
  phase.
- `git log main..HEAD --oneline` — the Ralph commit list, all carrying
  Lore trailers per `docs/commit_slicing_advice.md`.
- A single tag on the tip commit (`loop-go-…`, `loop-blocked-…`, or
  `loop-capped-…`) so `git show <tag>` lands the auditor at the
  termination state.

If any of those is missing, the loop did not stop cleanly and a human
must reconcile state before another loop instance is started.
