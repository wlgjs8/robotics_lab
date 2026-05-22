> Historical review/planning document. Some findings may be obsolete.
> The current source of truth is root README.md and docs/architecture.md.

# Commit Slicing Advice

Architect-level guidance for how to slice future commits on
`rb_servo_server`, grounded in the seven commits between `main`
and the current branch tip (the real-readiness / safety-policy series).

## 1. What the current branch does well

- **One behavior per commit.** `d8d07f0` (arming gate), `bd18dcb` (send-failure
  latch), and `f406f93` (real-mode network + realtime guards) each carry a
  single, reviewable invariant. A reviewer can read just the diff and the
  `Directive:` line and decide whether to land it.
- **Constraint-driven messages.** Every commit message answers "what
  invariant is now enforced and what was rejected." That format is worth
  keeping: it makes future bisects, reverts, and incident reviews cheap.
- **Build/test evidence in the trailer.** `Tested:` / `Not-tested:` lines
  let the next worker know exactly what was exercised. Keep this even when
  the change feels trivial.

## 2. Slicing issues observed (and how to avoid them)

### 2.1 `fb6c662` mixes parser hardening with new operator tooling

That commit lands the command-server parser overhaul (`src/network/command_server.cpp`,
+349/-152) **and** four new `tools/send_*.py` operator scripts in the same
commit. The two concerns have different review audiences, different blast
radius, and different reversibility:

- Parser change: production behavior, regression risk, requires C++ review.
- Send tools: developer ergonomics, no runtime coupling, Python-only.

**Recipe for next time:**

1. Commit the parser change first, with the unit/integration evidence that
   proves drop-vs-buffer semantics.
2. Commit `tools/send_*.py` afterwards in a separate "operator helpers"
   commit. If a tool is *used* in the parser commit's test evidence, inline
   the one-liner needed for the test there and keep the rich CLI for the
   tooling commit.

Rule of thumb: **if a reviewer would skim half of a diff because it is in
a different language or directory tree, it is two commits.**

### 2.2 `7e0b951` bundles regression tests with policy docs

`Lock real-readiness safety policy with tests` carries
`tests/test_safety_policy.cpp` (+228), `docs/fail_safe_policy.md`,
`docs/network_protocol.md`, `docs/testing.md`, `rb-servo-server.md`, and a
`CMakeLists.txt` entry for the new test binary. Three problems:

- The docs lock in **policy**; the tests lock in **behavior**. If the policy
  is later softened, the docs commit is what you want to revert — but you
  can't revert it without dropping the test binary too.
- The `CMakeLists.txt` change couples the build graph to test-only files,
  so any later split forces a rebase-edit of CMake.
- Docs that describe behavior added in `bd18dcb` / `fb6c662` / `f406f93`
  arrive *after* the behavior, which is fine, but bundling them with new
  test code hides the temporal ordering reviewers care about.

**Recipe for next time:**

1. **Docs commit first** ("Document real-readiness safety policy") that
   updates `docs/fail_safe_policy.md`, `docs/network_protocol.md`,
   `rb-servo-server.md`. Reviewable as prose, revertable independently.
2. **Tests commit second** ("Lock safety policy with regression tests")
   that adds `tests/test_safety_policy.cpp`, the CMake entry, and the
   `docs/testing.md` paragraph that explains how to run it. The CMake +
   testing.md change belongs with the test code because both are about
   "how to run this," not "what the policy is."

### 2.3 Message-body line endings

Every commit body on this branch contains literal `\n` escape sequences
instead of real newlines, e.g.

```
Constraint: ...\nRejected: ...\nConfidence: ...
```

That means `git log`, GitHub's PR view, and `git blame -L` all render the
body as one wrapped paragraph and the `Directive:` line is no longer
greppable on its own. This is almost certainly the message being passed
through a `-m "..."` with embedded `\n` instead of a heredoc.

**Recipe:** use a heredoc so newlines survive.

```sh
git commit -F - <<'EOF'
Subject line in imperative mood

Constraint: ...
Rejected: ... | reason
Confidence: high|medium|low
Scope-risk: narrow|moderate|broad
Directive: ...
Tested: <commands run>
Not-tested: <what is deliberately deferred>
EOF
```

This is a small change, but it pays off the next time someone runs
`git log --grep '^Directive:'` to audit invariants across the tree.

## 3. Slicing principles for this codebase

The servo server has three concern layers that benefit from being kept in
separate commits whenever a change touches more than one:

| Layer            | Lives in                                                   | Typical reviewer concern         |
| ---------------- | ---------------------------------------------------------- | -------------------------------- |
| Wire / parser    | `src/network/`, `include/rb_servo/network/`                | Drop vs. accept, schema drift    |
| Control / motion | `src/control/`, `include/rb_servo/control/`, `src/core/`   | Realtime, safety, motion gates   |
| Config / startup | `src/config/`, `config/*.yaml`, `CMakeLists.txt`           | Operational defaults, build deps |
| Operator tools   | `tools/*.py`                                               | Ergonomics, no runtime impact    |
| Policy docs      | `docs/*.md`, `rb-servo-server.md`                          | Prose review only                |
| Regression tests | `tests/*.cpp`                                              | Lock behavior post-implementation |
| Review evidence  | `.omx/autoloop/*`, `.omx/context/*`                        | Audit trail only, no runtime impact |

Mixing two layers in one commit is fine **when the change is causally
indivisible** (e.g. `f406f93` had to land config defaults + startup guard
together — disarming either half would have left real mode insecure).
Mixing two layers when they can be ordered is a slicing miss.

## 4. Recommended commit order for safety-policy-shaped work

When the next worker lands a similarly-shaped patch series, prefer this
order — it minimises "scary" diffs and makes each commit revertable:

1. **Types / data structures** (`include/.../types.hpp`, `core/types.cpp`).
   Pure additive, no behavior change.
2. **Behavior change behind the new type** (control loop, parser).
   This is the commit a future reverter will reach for.
3. **Build / config / defaults** (CMake, YAML defaults).
   Includes any env-var gates so production stays fail-closed.
4. **Regression tests** (`tests/test_*.cpp` + CMake test target).
5. **Operator tooling** (`tools/*.py`) if any.
6. **Docs** that describe the now-locked policy.

Steps 4–6 can swap, but never put 5 before 2.

## 5. Size targets (soft)

These are not rules, just smoke-test thresholds — if a commit exceeds them
without a one-line justification in the body, it probably wants slicing:

- ≤ ~200 lines of non-test code changed.
- ≤ ~6 files touched (excluding generated or test fixtures).
- ≤ 2 layers from the table in §3.

By those thresholds `fb6c662` (476 LOC across wire+tools) and `7e0b951`
(325 LOC across docs+tests+build) were the two on this branch worth
re-slicing; everything else was within budget.

## 6. When to *not* slice

Resist the urge to slice when:

- Both halves of a change must land together for the system to stay
  fail-closed (e.g. tightening a default *and* the guard that enforces it).
- The second commit would not build or pass tests on its own. A bisect
  that lands on a broken intermediate commit is worse than a slightly
  larger atomic commit.
- The change is a pure rename or mechanical refactor — slicing those
  inflates review burden without adding revertability.

## 7. Review-evidence and autoloop artifact commits

Two commits since the original advisory exemplify a sixth concern layer
the table in §3 missed: **review evidence**.

- `e774bd0` (this advisory itself) — single file `docs/commit_slicing_advice.md`,
  +163/-0. Docs-only, advisory scope. Reverts cleanly without touching
  runtime, tests, build, or operator tools.
- `adc81d9` (`Record critic Go evidence for unattended safety loop`) — two
  files `.omx/autoloop/critic-iteration-1.md` and
  `.omx/autoloop/verdict-iteration-1.txt`, +239/-0. Records the
  architect/critic handoff verdict for an unattended safety loop. No
  runtime, build, or test surface touched.

Both are model-citizens for the slicing rules above: one concern layer,
small footprint, body answers Constraint/Rejected/Confidence/Scope-risk/
Directive, `Tested:` line ties back to the production verification this
artifact summarises rather than to a build of the artifact itself.

**Recipe for autoloop / critic-evidence commits:**

1. Commit `.omx/autoloop/*` (and `.omx/context/*` when those are the
   review record) on their own, separate from any docs commit that
   *describes policy*. Mixing prose policy (`docs/*.md`) with
   audit-trail evidence (`.omx/autoloop/*`) couples reverts: a future
   policy softening would otherwise have to keep stale critic evidence
   alive.
2. Keep the `Tested:` trailer pointing at the *underlying behavior's*
   verification commands (cmake/ctest/smoke), even though the artifact
   commit changed no buildable code. That trailer is what a future
   bisect uses to recover the verification path; "this commit is
   docs-only" is true but not load-bearing on its own.
3. Treat these commits as `Scope-risk: narrow` by default. A
   `Scope-risk: broad` review-evidence commit usually means the artifact
   has grown beyond a single iteration's verdict and should itself be
   sliced (one file per iteration, or split critic-from-architect
   evidence).

## 8. OMX auto-checkpoint commits must not survive into final history

`adc81d9`'s `Rejected:` line names a hazard worth promoting to a
first-class rule: **`omx(team)` auto-checkpoint commits and worker-pane
auto-merge commits must be rewritten before the branch is reviewed.**

Symptoms to look for in `git log main..HEAD`:

- Subjects like `omx(team) auto-checkpoint`, `omx: checkpoint`,
  `worker-N: wip`, or default `Merge branch ...` lines from worker
  worktrees.
- Empty or boilerplate bodies missing the
  Constraint/Rejected/Confidence/Scope-risk/Directive structure.
- Author identity `OmX <omx@oh-my-codex.dev>` as the sole author
  (co-authoring is fine; sole authorship by the harness is not).

**Recipe:**

1. Before opening review, rebase the auto-checkpoint range into Lore
   commits authored by the human/agent that produced the change. Use
   `git rebase -i main` interactively and squash adjacent
   auto-checkpoints into the behavior commit they belong to.
2. Preserve the OmX co-authorship trailer (`Co-authored-by: OmX
   <omx@oh-my-codex.dev>`) on the rewritten commit — that records the
   harness's involvement without ceding authorship.
3. If an auto-checkpoint cannot be cleanly attributed (it spans
   multiple behavior changes), that itself is a slicing signal: split
   the diff along behavior boundaries first, then reattribute each
   slice.

Rule of thumb: **if a reviewer cannot read the commit subject and infer
the invariant, the commit should not reach `main`.**

## 9. Quick checklist for the next safety-shaped patch series

Use this as the pre-PR gate; each line should be answerable "yes" or
have a one-line justification recorded in the relevant commit body.

- [ ] Every commit on the branch has a non-empty
      Constraint/Rejected/Confidence/Scope-risk/Directive body.
- [ ] No commit body contains literal `\n` escapes (see §2.3).
- [ ] No `omx(team) auto-checkpoint` or unattributed merge commits
      remain (see §8).
- [ ] No commit mixes a wire/parser change with operator tooling
      (see §2.1).
- [ ] No commit mixes policy prose with regression tests or build
      graph changes (see §2.2).
- [ ] No commit mixes policy prose with `.omx/autoloop/*` evidence
      (see §7).
- [ ] Commits land in the order in §4, or the body explains why an
      order swap was causally indivisible (see §3 last paragraph).
- [ ] No commit exceeds the §5 size thresholds without a justifying
      one-line note.
