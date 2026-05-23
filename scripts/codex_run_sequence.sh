#!/usr/bin/env bash
set -euo pipefail

# Sequential Codex task runner.
# - Runs one prompt file per task from .codex/prompts/<TASK>.md
# - Runs scripts/codex_gate.sh <TASK> unless CODEX_SKIP_GATE=1
# - Commits each task separately, excluding artifacts/codex_runs from commits
# - Records final agent message, JSON events, stderr, gate stdout/stderr, and diff stat

if git rev-parse --show-toplevel >/dev/null 2>&1; then
  REPO_ROOT="$(git rev-parse --show-toplevel)"
else
  REPO_ROOT="$(pwd)"
fi

PROMPT_DIR="${PROMPT_DIR:-$REPO_ROOT/.codex/prompts}"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-$REPO_ROOT/artifacts/codex_runs}"
CODEX_BIN="${CODEX_BIN:-codex}"
CODEX_SANDBOX="${CODEX_SANDBOX:-workspace-write}"
CODEX_APPROVAL_POLICY="${CODEX_APPROVAL_POLICY:-never}"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 HARDEN-01 HARDEN-02 ..." >&2
  echo "Environment knobs:" >&2
  echo "  CODEX_SKIP_GATE=1" >&2
  echo "  CODEX_CONTINUE_ON_GATE_FAIL=1" >&2
  echo "  CODEX_CONTINUE_ON_CODEX_FAIL=1" >&2
  echo "  CODEX_ALLOW_DIRTY=1" >&2
  echo "  CODEX_TEE_LOGS=1" >&2
  echo "  CODEX_SANDBOX=workspace-write|danger-full-access" >&2
  exit 2
fi

require_clean_tree() {
  if [[ "${CODEX_ALLOW_DIRTY:-0}" == "1" ]]; then
    echo "CODEX_ALLOW_DIRTY=1: skipping clean tree check"
    return 0
  fi

  if [[ -n "$(git status --porcelain -- ':!artifacts/codex_runs' 2>/dev/null || true)" ]]; then
    echo "ERROR: git working tree is not clean." >&2
    echo "Commit, stash, or discard changes before running the sequence." >&2
    git status --short >&2 || true
    exit 1
  fi
}

append_result_header_if_needed() {
  mkdir -p "$ARTIFACT_ROOT"
  local csv="$ARTIFACT_ROOT/task_results.csv"
  if [[ ! -f "$csv" ]]; then
    echo "timestamp,task,codex_status,codex_rc,gate_status,gate_rc,artifact_dir" > "$csv"
  fi
}

build_codex_args() {
  local final_path="$1"
  local -n out_args_ref="$2"

  out_args_ref=(
    exec
    --cd "$REPO_ROOT"
    --sandbox "$CODEX_SANDBOX"
    --json
    --output-last-message "$final_path"
  )

  local help_text
  help_text="$($CODEX_BIN exec --help 2>&1 || true)"

  if grep -q -- "--ask-for-approval" <<<"$help_text"; then
    out_args_ref+=(--ask-for-approval "$CODEX_APPROVAL_POLICY")
  elif grep -q -- "--config" <<<"$help_text" || grep -q -- "-c" <<<"$help_text"; then
    out_args_ref+=(-c "approval_policy=$CODEX_APPROVAL_POLICY")
  else
    echo "WARN: codex exec help did not show --ask-for-approval or --config/-c." >&2
    echo "      Continuing without explicit approval policy." >&2
  fi
}

run_codex() {
  local prompt="$1"
  local artifact_dir="$2"
  local final_path="$artifact_dir/final.md"
  local events_path="$artifact_dir/events.jsonl"
  local stderr_path="$artifact_dir/stderr.log"

  local codex_args=()
  build_codex_args "$final_path" codex_args

  printf '%q ' "$CODEX_BIN" "${codex_args[@]}" - > "$artifact_dir/codex_command.txt"
  echo "< $prompt" >> "$artifact_dir/codex_command.txt"

  set +e
  if [[ "${CODEX_TEE_LOGS:-0}" == "1" ]]; then
    "$CODEX_BIN" "${codex_args[@]}" - < "$prompt" \
      > >(tee "$events_path") \
      2> >(tee "$stderr_path" >&2)
  else
    "$CODEX_BIN" "${codex_args[@]}" - < "$prompt" \
      > "$events_path" \
      2> "$stderr_path"
  fi
  local rc=$?
  set -e

  return "$rc"
}

run_gate() {
  local task="$1"
  local artifact_dir="$2"
  local stdout_path="$artifact_dir/gate.stdout.log"
  local stderr_path="$artifact_dir/gate.stderr.log"

  if [[ "${CODEX_SKIP_GATE:-0}" == "1" ]]; then
    echo "Gate skipped by CODEX_SKIP_GATE=1" > "$stdout_path"
    : > "$stderr_path"
    return 125
  fi

  if [[ ! -x "$REPO_ROOT/scripts/codex_gate.sh" ]]; then
    echo "ERROR: $REPO_ROOT/scripts/codex_gate.sh is missing or not executable" > "$stderr_path"
    return 126
  fi

  set +e
  "$REPO_ROOT/scripts/codex_gate.sh" "$task" \
    > "$stdout_path" \
    2> "$stderr_path"
  local rc=$?
  set -e
  return "$rc"
}

commit_if_needed() {
  local task="$1"
  local gate_status="$2"
  local codex_status="$3"
  local artifact_dir="$4"

  git add -A
  git reset -q -- artifacts/codex_runs 2>/dev/null || true

  if git diff --cached --quiet; then
    echo "No staged changes produced by $task. Skipping commit."
    return 0
  fi

  git diff --cached --stat | tee "$artifact_dir/diff.stat.txt"
  git commit -m "codex: implement ${task} [codex:${codex_status}] [gate:${gate_status}]"
  echo "Committed: codex: implement ${task} [codex:${codex_status}] [gate:${gate_status}]"
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

  local codex_rc=0
  local codex_status="passed"
  if run_codex "$prompt" "$artifact_dir"; then
    echo "Codex finished: $task"
  else
    codex_rc=$?
    codex_status="failed"
    echo "Codex failed: $task with exit code $codex_rc"
    if [[ "${CODEX_CONTINUE_ON_CODEX_FAIL:-0}" != "1" ]]; then
      echo "Stopping because Codex failed. To continue, run with CODEX_CONTINUE_ON_CODEX_FAIL=1"
      append_result_header_if_needed
      echo "$timestamp,$task,$codex_status,$codex_rc,not_run,0,$artifact_dir" >> "$ARTIFACT_ROOT/task_results.csv"
      exit "$codex_rc"
    fi
    echo "Continuing despite Codex failure because CODEX_CONTINUE_ON_CODEX_FAIL=1"
  fi

  local gate_status="not_run"
  local gate_rc=0

  if [[ "$codex_status" == "passed" ]]; then
    echo "Running deterministic gate: $task"
    set +e
    run_gate "$task" "$artifact_dir"
    gate_rc=$?
    set -e

    if [[ "$gate_rc" -eq 0 ]]; then
      gate_status="passed"
      echo "Gate passed: $task"
    elif [[ "$gate_rc" -eq 125 ]]; then
      gate_status="skipped"
      gate_rc=0
      echo "Gate skipped: $task"
    else
      gate_status="failed"
      echo "Gate failed: $task with exit code $gate_rc"
      if [[ "${CODEX_CONTINUE_ON_GATE_FAIL:-0}" != "1" ]]; then
        echo "Stopping because gate failed. To continue, run with CODEX_CONTINUE_ON_GATE_FAIL=1"
        append_result_header_if_needed
        echo "$timestamp,$task,$codex_status,$codex_rc,$gate_status,$gate_rc,$artifact_dir" >> "$ARTIFACT_ROOT/task_results.csv"
        exit "$gate_rc"
      fi
      echo "Continuing despite gate failure because CODEX_CONTINUE_ON_GATE_FAIL=1"
    fi
  else
    echo "Skipping gate because Codex failed for $task"
    echo "Gate skipped because Codex failed" > "$artifact_dir/gate.stdout.log"
    : > "$artifact_dir/gate.stderr.log"
  fi

  append_result_header_if_needed
  echo "$timestamp,$task,$codex_status,$codex_rc,$gate_status,$gate_rc,$artifact_dir" >> "$ARTIFACT_ROOT/task_results.csv"

  commit_if_needed "$task" "$gate_status" "$codex_status" "$artifact_dir"
}

main() {
  cd "$REPO_ROOT"
  require_clean_tree
  append_result_header_if_needed

  for task in "$@"; do
    run_one_task "$task"
  done

  echo "============================================================"
  echo "All requested tasks completed."
  echo "Tasks: $*"
  echo "Results: $ARTIFACT_ROOT/task_results.csv"
  echo "============================================================"
}

main "$@"