#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
PROMPT_DIR="$REPO_ROOT/.codex/prompts"
ARTIFACT_ROOT="$REPO_ROOT/artifacts/codex_runs"
CODEX_BIN="${CODEX_BIN:-codex}"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 P1-A P1-B P1-C ..." >&2
  exit 2
fi

require_clean_tree() {
  if [[ -n "$(git status --porcelain)" ]]; then
    echo "ERROR: git working tree is not clean." >&2
    echo "Commit, stash, or discard changes before running the sequence." >&2
    git status --short >&2
    exit 1
  fi
}

run_one_task() {
  local task="$1"
  local prompt="$PROMPT_DIR/${task}.md"
  local timestamp
  timestamp="$(date +%Y%m%d_%H%M%S)"
  local artifact_dir="$ARTIFACT_ROOT/${timestamp}_${task}"

  if [[ ! -f "$prompt" ]]; then
    echo "ERROR: prompt not found: $prompt" >&2
    exit 1
  fi

  mkdir -p "$artifact_dir"

  echo "============================================================"
  echo "Running Codex task: $task"
  echo "Prompt: $prompt"
  echo "Artifacts: $artifact_dir"
  echo "============================================================"

  # "$CODEX_BIN" exec \
  #   --cd "$REPO_ROOT" \
  #   --sandbox workspace-write \
  #   --ask-for-approval never \
  #   --json \
  #   --output-last-message "$artifact_dir/final.md" \
  #   - < "$prompt" \
  #   > "$artifact_dir/events.jsonl" \
  #   2> "$artifact_dir/stderr.log"
  local codex_args
  codex_args=(
    exec
    --cd "$REPO_ROOT"
    --sandbox workspace-write
    -c approval_policy=never
    --json
    --output-last-message "$artifact_dir/final.md"
  )

  "$CODEX_BIN" "${codex_args[@]}" \
    - < "$prompt" \
    > "$artifact_dir/events.jsonl" \
    2> "$artifact_dir/stderr.log"

  # echo "Codex finished: $task"
  # echo "Running deterministic gate: $task"

  # "$REPO_ROOT/scripts/codex_gate.sh" "$task" \
  #   > "$artifact_dir/gate.stdout.log" \
  #   2> "$artifact_dir/gate.stderr.log"

  # echo "Gate passed: $task"
  echo "Codex finished: $task"

  local gate_status="not_run"
  local gate_rc=0

  if [[ "${CODEX_SKIP_GATE:-0}" == "1" ]]; then
    echo "Skipping deterministic gate for $task because CODEX_SKIP_GATE=1"
    gate_status="skipped"
    echo "Gate skipped by CODEX_SKIP_GATE=1" > "$artifact_dir/gate.stdout.log"
    : > "$artifact_dir/gate.stderr.log"
  else
    echo "Running deterministic gate: $task"

    set +e
    "$REPO_ROOT/scripts/codex_gate.sh" "$task" \
      > "$artifact_dir/gate.stdout.log" \
      2> "$artifact_dir/gate.stderr.log"
    gate_rc=$?
    set -e

    if [[ "$gate_rc" -eq 0 ]]; then
      echo "Gate passed: $task"
      gate_status="passed"
    else
      echo "Gate failed: $task with exit code $gate_rc"
      gate_status="failed"

      if [[ "${CODEX_CONTINUE_ON_GATE_FAIL:-0}" == "1" ]]; then
        echo "Continuing despite gate failure because CODEX_CONTINUE_ON_GATE_FAIL=1"
      else
        echo "Stopping because gate failed. To continue automatically, run with CODEX_CONTINUE_ON_GATE_FAIL=1"
        exit "$gate_rc"
      fi
    fi
  fi

  echo "$task,$gate_status,$gate_rc,$artifact_dir" >> "$ARTIFACT_ROOT/task_results.csv"

  if [[ -z "$(git status --porcelain)" ]]; then
    echo "No changes produced by $task. Skipping commit."
    return 0
  fi

  git diff --stat | tee "$artifact_dir/diff.stat.txt"

  git add -A
  # git commit -m "codex: implement ${task}"
  git commit -m "codex: implement ${task} [gate:${gate_status}]"

  echo "Committed: codex: implement ${task} [gate:${gate_status}]"
}

main() {
  cd "$REPO_ROOT"

  require_clean_tree

  mkdir -p "$ARTIFACT_ROOT"

  for task in "$@"; do
    run_one_task "$task"
  done

  echo "============================================================"
  echo "All requested tasks completed successfully."
  echo "Tasks: $*"
  echo "============================================================"
}

main "$@"