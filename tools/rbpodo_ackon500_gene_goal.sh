#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVER="${ROOT_DIR}/rb_servo_server/build/rbpodo_real_gate/rb_servo_server"
MATRIX=""
ARTIFACT_ROOT=""
PROFILE="best"
WITH_REQUIRED_ENV=0
DRY_RUN=0
SKIP_PLOTS=0
SKIP_NOOP=0
VERIFY_PGMODE=0
CONFIRM=0
CONFIRM_PGMODE=0
ALLOW_NO_REALTIME=0
ALLOW_LOCAL_DIFF=0
PGMODE_TIMEOUT_SEC="1.0"
NOOP_CONFIG=""
NOOP_COMMAND_TIMEOUT_SEC="0.05"

usage() {
  cat <<'EOF'
Usage: tools/rbpodo_ackon500_gene_goal.sh [options]

Run ACKON500-GENE-GOAL-01 against Rainbow controllers in pgmode simulation.
The wrapper refuses physical Cartesian real mode and never sets RB_ALLOW_*
variables unless --with-required-env is passed.

Default profile:
  --profile best  Run the named ACKON500 best controller-simulation profile.
                  This is controller-reference lower-bound evidence, not
                  physical real tracking.
  --profile repeatability
                  Run ACKON500-REPEATABILITY-VALIDATION-01: three left-arm
                  and three right-arm repetitions of the named best profile.

Required safety flags:
  --i-understand-this-connects-to-real-controller
  --i-confirm-controller-is-in-pgmode-simulation

Environment behavior:
  --with-required-env  Explicitly export:
    RB_ALLOW_REAL_ROBOT=1
    RB_ALLOW_REAL_MOTION=1
    RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
    RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
    RB_ALLOW_RBPODO_ASYNC_STREAMING=1
    RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
    RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1

Options:
  --profile NAME            best (default), matrix, or repeatability.
  --allow-local-diff        Allow local best profile to differ from tracked best profile.
  --artifact-root DIR        Default artifacts/rbpodo_circle_ablation/<timestamp>_<profile suffix>
  --server PATH              rb_servo_server binary path.
  --matrix PATH              Matrix path, default depends on --profile.
  --verify-pgmode-simulation Verify instead of setting pgmode simulation for each run.
  --pgmode-timeout-sec SEC   pgmode command timeout, default 1.0.
  --skip-noop                Skip the 500 Hz sdk_ack_worker no-op preflight.
  --skip-plots               Forward --skip-plots to acceptance and ablation.
  --allow-no-realtime        Allow missing realtime capabilities for explicit non-realtime testing.
  --dry-run                  Print commands without executing them.
  -h, --help                 Show this help.

Pass criteria are evaluated by scripts/generate_ackon500_gene_goal_report.py.
The final summary is written to ARTIFACT_ROOT/summary.json and the markdown
report to ARTIFACT_ROOT/gene_goal_report.md. Repeatability runs also write
repeatability_summary.csv/json and repeatability_report.md.
EOF
}

fail() {
  echo "rbpodo_ackon500_gene_goal: ERROR: $*" >&2
  exit 1
}

note() {
  echo "rbpodo_ackon500_gene_goal: $*"
}

abs_from_root() {
  local path="$1"
  if [[ "${path}" == /* ]]; then
    printf '%s\n' "${path}"
  else
    printf '%s/%s\n' "${ROOT_DIR}" "${path}"
  fi
}

print_command() {
  printf 'rbpodo_ackon500_gene_goal: command:'
  printf ' %q' "$@"
  printf '\n'
}

require_env() {
  local missing=()
  for key in \
    RB_ALLOW_REAL_ROBOT \
    RB_ALLOW_REAL_MOTION \
    RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION \
    RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN \
    RB_ALLOW_RBPODO_ASYNC_STREAMING \
    RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM \
    RB_RBPODO_PGMODE_SIMULATION_CONFIRMED
  do
    if [[ "${!key:-}" != "1" ]]; then
      missing+=("${key}=1")
    fi
  done
  if ((${#missing[@]} > 0)); then
    printf 'rbpodo_ackon500_gene_goal: missing required env:\n' >&2
    printf '  %s\n' "${missing[@]}" >&2
    printf 'Pass --with-required-env to set these explicitly in this wrapper.\n' >&2
    exit 1
  fi
}

check_realtime_caps() {
  [[ -f "${SERVER}" ]] || fail "server binary not found: ${SERVER}"
  if ! command -v getcap >/dev/null 2>&1; then
    [[ "${ALLOW_NO_REALTIME}" == "1" ]] && {
      note "warning: getcap unavailable; realtime capability not checked"
      return 0
    }
    fail "getcap unavailable; pass --allow-no-realtime only for explicit non-realtime testing"
  fi
  local caps
  caps="$(getcap "${SERVER}" 2>/dev/null || true)"
  if [[ "${caps}" != *"cap_sys_nice"* || "${caps}" != *"cap_ipc_lock"* ]]; then
    [[ "${ALLOW_NO_REALTIME}" == "1" ]] && {
      note "warning: realtime capabilities missing: ${caps:-<none>}"
      return 0
    }
    fail "server binary lacks cap_sys_nice and cap_ipc_lock; pass --allow-no-realtime only for explicit non-realtime testing"
  fi
  local namespaced_caps
  namespaced_caps="$(getcap -n "${SERVER}" 2>/dev/null || true)"
  if [[ "${namespaced_caps}" == *"[rootid="* ]]; then
    [[ "${ALLOW_NO_REALTIME}" == "1" ]] && {
      note "warning: realtime capabilities are namespaced and may not grant runtime SCHED_FIFO: ${namespaced_caps}"
      return 0
    }
    fail "server realtime capabilities are namespaced (${namespaced_caps}); set cap_sys_nice,cap_ipc_lock from the host namespace or pass --allow-no-realtime only for explicit non-realtime testing"
  fi
}

check_local_best_profile() {
  local tracked="${ROOT_DIR}/rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz_goal.example.yaml"
  local local_copy="${ROOT_DIR}/rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s_500hz_goal.yaml"

  [[ -f "${tracked}" ]] || fail "tracked best profile not found: ${tracked}"

  if [[ ! -f "${local_copy}" ]]; then
    note "local best profile not found; using tracked profile: ${tracked#${ROOT_DIR}/}"
    note "create a local copy with tools/create_rbpodo_circle_local_configs.sh --include-goal"
    return 0
  fi

  if cmp -s "${tracked}" "${local_copy}"; then
    note "local best profile matches tracked profile: ${local_copy#${ROOT_DIR}/}"
    return 0
  fi

  if [[ "${ALLOW_LOCAL_DIFF}" == "1" ]]; then
    note "warning: local best profile differs from tracked profile and --allow-local-diff was supplied"
    return 0
  fi

  fail "local best profile differs from tracked profile: ${local_copy#${ROOT_DIR}/}; pass --allow-local-diff only after reviewing operator-local edits"
}

while (($# > 0)); do
  case "$1" in
    --profile)
      [[ $# -ge 2 ]] || fail "--profile requires a value"
      PROFILE="$2"
      shift 2
      ;;
    --allow-local-diff)
      ALLOW_LOCAL_DIFF=1
      shift
      ;;
    --artifact-root)
      [[ $# -ge 2 ]] || fail "--artifact-root requires a directory"
      ARTIFACT_ROOT="$2"
      shift 2
      ;;
    --server)
      [[ $# -ge 2 ]] || fail "--server requires a path"
      SERVER="$2"
      shift 2
      ;;
    --matrix)
      [[ $# -ge 2 ]] || fail "--matrix requires a path"
      MATRIX="$2"
      shift 2
      ;;
    --with-required-env)
      WITH_REQUIRED_ENV=1
      shift
      ;;
    --verify-pgmode-simulation)
      VERIFY_PGMODE=1
      shift
      ;;
    --pgmode-timeout-sec)
      [[ $# -ge 2 ]] || fail "--pgmode-timeout-sec requires a value"
      PGMODE_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --skip-noop)
      SKIP_NOOP=1
      shift
      ;;
    --skip-plots)
      SKIP_PLOTS=1
      shift
      ;;
    --allow-no-realtime)
      ALLOW_NO_REALTIME=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --i-understand-this-connects-to-real-controller)
      CONFIRM=1
      shift
      ;;
    --i-confirm-controller-is-in-pgmode-simulation)
      CONFIRM_PGMODE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ "${CONFIRM}" == "1" ]] || fail "missing --i-understand-this-connects-to-real-controller"
[[ "${CONFIRM_PGMODE}" == "1" ]] || fail "missing --i-confirm-controller-is-in-pgmode-simulation"
if [[ "${RB_ALLOW_REAL_CARTESIAN:-}" == "1" ]]; then
  fail "RB_ALLOW_REAL_CARTESIAN must not be set for controller-simulation goal runs"
fi

case "${PROFILE}" in
  best)
    [[ -n "${MATRIX}" ]] || MATRIX="configs/rbpodo_circle_ablation/ackon500_gene_goal_best.yaml"
    NOOP_CONFIG="rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz_goal.example.yaml"
    NOOP_COMMAND_TIMEOUT_SEC="0.05"
    ARTIFACT_SUFFIX="ackon500_gene_goal_best"
    ;;
  matrix|full)
    [[ -n "${MATRIX}" ]] || MATRIX="configs/rbpodo_circle_ablation/ackon500_gene_goal.yaml"
    NOOP_CONFIG="rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz.example.yaml"
    NOOP_COMMAND_TIMEOUT_SEC="0.2"
    ARTIFACT_SUFFIX="ackon500_gene_goal"
    ;;
  repeatability)
    [[ -n "${MATRIX}" ]] || MATRIX="configs/rbpodo_circle_ablation/ackon500_gene_repeatability.yaml"
    NOOP_CONFIG="rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz_goal.example.yaml"
    NOOP_COMMAND_TIMEOUT_SEC="0.05"
    ARTIFACT_SUFFIX="ackon500_gene_repeatability"
    ;;
  *)
    fail "unknown profile: ${PROFILE}; expected best, matrix, or repeatability"
    ;;
esac

SERVER="$(abs_from_root "${SERVER}")"
MATRIX="$(abs_from_root "${MATRIX}")"
[[ -f "${MATRIX}" ]] || fail "matrix file not found: ${MATRIX}"
[[ -f "${ROOT_DIR}/${NOOP_CONFIG}" ]] || fail "no-op config not found: ${NOOP_CONFIG}"

if [[ "${PROFILE}" == "best" ]]; then
  check_local_best_profile
fi

note "This is controller-reference lower-bound evidence, not physical real tracking."

if [[ -z "${ARTIFACT_ROOT}" ]]; then
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  ARTIFACT_ROOT="${ROOT_DIR}/artifacts/rbpodo_circle_ablation/${timestamp}_${ARTIFACT_SUFFIX}"
else
  ARTIFACT_ROOT="$(abs_from_root "${ARTIFACT_ROOT}")"
fi

if [[ "${WITH_REQUIRED_ENV}" == "1" ]]; then
  export RB_ALLOW_REAL_ROBOT=1
  export RB_ALLOW_REAL_MOTION=1
  export RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
  export RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
  export RB_ALLOW_RBPODO_ASYNC_STREAMING=1
  export RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
  export RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1
else
  require_env
fi

PGMODE_FLAG="--set-pgmode-simulation"
[[ "${VERIFY_PGMODE}" == "1" ]] && PGMODE_FLAG="--verify-pgmode-simulation"

NOOP_ARTIFACT="${ARTIFACT_ROOT}/00_noop_500hz_sdk_ack_worker"
noop_cmd=(
  python3 scripts/rbpodo_500hz_acceptance.py
  --server "${SERVER}"
  --config "${NOOP_CONFIG}"
  --arm left
  --send-arms both
  --duration-sec 10
  --command-timeout-sec "${NOOP_COMMAND_TIMEOUT_SEC}"
  --async-mode sdk_ack_worker
  --min-controller-acceptance-ratio 0.98
  --min-send-count-ratio 0.98
  --max-state-age-us 5000
  --max-deadline-miss-count 0
  --max-worker-drop-count 100
  --max-drop-ratio 0.02
  "${PGMODE_FLAG}"
  --pgmode-timeout-sec "${PGMODE_TIMEOUT_SEC}"
  --artifact-dir "${NOOP_ARTIFACT}"
  --i-understand-this-connects-to-real-controller
  --i-confirm-controller-is-in-pgmode-simulation
)

ablation_cmd=(
  python3 scripts/run_rbpodo_circle_ablation.py
  --matrix "${MATRIX}"
  --artifact-root "${ARTIFACT_ROOT}"
  --server "${SERVER}"
  "${PGMODE_FLAG}"
  --pgmode-timeout-sec "${PGMODE_TIMEOUT_SEC}"
  --i-understand-this-connects-to-real-controller
  --i-confirm-controller-is-in-pgmode-simulation
)

report_cmd=(
  python3 scripts/generate_ackon500_gene_goal_report.py
  --artifact-root "${ARTIFACT_ROOT}"
)
if [[ "${PROFILE}" == "repeatability" ]]; then
  report_cmd+=(--require-repeatable)
else
  report_cmd+=(--require-pass)
fi

[[ "${SKIP_PLOTS}" == "1" ]] && noop_cmd+=(--skip-plots) && ablation_cmd+=(--skip-plots)

cd "${ROOT_DIR}"

if [[ "${DRY_RUN}" == "1" ]]; then
  [[ "${SKIP_NOOP}" == "1" ]] || print_command "${noop_cmd[@]}"
  print_command "${ablation_cmd[@]}"
  print_command "${report_cmd[@]}"
  note "dry-run only; commands were not executed"
  note "expected summary: ${ARTIFACT_ROOT}/summary.json"
  if [[ "${PROFILE}" == "repeatability" ]]; then
    note "expected repeatability summary: ${ARTIFACT_ROOT}/repeatability_summary.json"
    note "expected repeatability report: ${ARTIFACT_ROOT}/repeatability_report.md"
  fi
  exit 0
fi

check_realtime_caps

if [[ "${SKIP_NOOP}" != "1" ]]; then
  print_command "${noop_cmd[@]}"
  "${noop_cmd[@]}"
fi

print_command "${ablation_cmd[@]}"
"${ablation_cmd[@]}"

print_command "${report_cmd[@]}"
"${report_cmd[@]}"

note "goal summary: ${ARTIFACT_ROOT}/summary.json"
note "goal report: ${ARTIFACT_ROOT}/gene_goal_report.md"
if [[ "${PROFILE}" == "repeatability" ]]; then
  note "repeatability summary: ${ARTIFACT_ROOT}/repeatability_summary.json"
  note "repeatability report: ${ARTIFACT_ROOT}/repeatability_report.md"
fi
