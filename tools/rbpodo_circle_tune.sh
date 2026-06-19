#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVER="${ROOT_DIR}/rb_servo_server/build/rbpodo_real_gate/rb_servo_server"
MATRIX=""
MATRIX_FILE=""
MATRIX_LABEL=""
ARM=""
ARTIFACT_ROOT=""
WITH_REQUIRED_ENV=0
FORCE_LOCAL_CONFIGS=0
SET_PGMODE=0
CHECK_CONTROLLER=0
ALLOW_NO_REALTIME=0
DRY_RUN=0
SKIP_PLOTS=0
PORT_TIMEOUT_SEC="1.0"

CONTROLLER_IPS=("172.28.60.200" "172.28.60.201")
STANDARD_LOCAL_CONFIGS=(
  "${ROOT_DIR}/rb_servo_server/config/local/dual_real_rbpodo_circle_15cm16s.yaml"
  "${ROOT_DIR}/rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s.yaml"
)

usage() {
  cat <<'EOF'
Usage: tools/rbpodo_circle_tune.sh --matrix NAME --arm left|right [options]
       tools/rbpodo_circle_tune.sh --matrix-file PATH --arm left|right [options]

Run a safe rbpodo controller-simulation circle tuning matrix wrapper.

Supported matrices:
  stage2_gain_split
  stage2_pub_speed
  stage2_8s_middle

Environment behavior:
  --with-required-env  Explicitly export:
    RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1

Without --with-required-env, those env vars must already be set.

Options:
  --matrix NAME                 One supported stage-2 matrix name.
  --matrix-file PATH            Custom matrix YAML path.
  --arm left|right              Required; all enabled matrix rows must match.
  --artifact-root DIR           Default artifacts/rbpodo_circle_ablation/<timestamp>_<matrix>_<arm>.
  --server PATH                 rb_servo_server binary path.
  --force-local-configs         Regenerate local circle configs with --force before checks.
  --set-pgmode-simulation       Call tools/simulation_mode.sh before the matrix run.
  --check-controller            Check controller TCP ports 5000 and 5001 first.
  --port-timeout-sec SEC        TCP port and pgmode timeout, default 1.0.
  --allow-no-realtime           Allow missing getcap or missing realtime caps.
  --dry-run                     Print the resolved ablation command without running it.
  --skip-plots                  Forward --skip-plots to the ablation runner.
  -h, --help                    Show this help.

This wrapper never sets RB_ALLOW_* variables unless --with-required-env is
passed. It refuses physical-real Cartesian mode and stale local configs.
EOF
}

fail() {
  echo "rbpodo_circle_tune: ERROR: $*" >&2
  exit 1
}

note() {
  echo "rbpodo_circle_tune: $*"
}

abs_from_root() {
  local path="$1"
  if [[ "${path}" == /* ]]; then
    printf '%s\n' "${path}"
  else
    printf '%s/%s\n' "${ROOT_DIR}" "${path}"
  fi
}

sanitize_label() {
  printf '%s\n' "$1" | sed -E 's/[^A-Za-z0-9_.-]+/_/g'
}

while (($# > 0)); do
  case "$1" in
    --matrix)
      [[ $# -ge 2 ]] || fail "--matrix requires a name"
      MATRIX="$2"
      shift 2
      ;;
    --matrix-file)
      [[ $# -ge 2 ]] || fail "--matrix-file requires a path"
      MATRIX_FILE="$2"
      shift 2
      ;;
    --arm)
      [[ $# -ge 2 ]] || fail "--arm requires left or right"
      ARM="$2"
      shift 2
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
    --with-required-env)
      WITH_REQUIRED_ENV=1
      shift
      ;;
    --force-local-configs)
      FORCE_LOCAL_CONFIGS=1
      shift
      ;;
    --set-pgmode-simulation)
      SET_PGMODE=1
      shift
      ;;
    --check-controller)
      CHECK_CONTROLLER=1
      shift
      ;;
    --port-timeout-sec)
      [[ $# -ge 2 ]] || fail "--port-timeout-sec requires a value"
      PORT_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --allow-no-realtime)
      ALLOW_NO_REALTIME=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --skip-plots)
      SKIP_PLOTS=1
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

if [[ -n "${MATRIX}" && -n "${MATRIX_FILE}" ]]; then
  fail "--matrix and --matrix-file are mutually exclusive"
fi

case "${MATRIX}" in
  stage2_gain_split|stage2_pub_speed|stage2_8s_middle)
    MATRIX_FILE="${ROOT_DIR}/configs/rbpodo_circle_ablation/${MATRIX}.yaml"
    MATRIX_LABEL="${MATRIX}"
    ;;
  "")
    [[ -n "${MATRIX_FILE}" ]] || fail "--matrix NAME or --matrix-file PATH is required"
    MATRIX_FILE="$(abs_from_root "${MATRIX_FILE}")"
    MATRIX_LABEL="$(sanitize_label "$(basename "${MATRIX_FILE%.yaml}")")"
    ;;
  *)
    fail "unknown matrix: ${MATRIX}; supported: stage2_gain_split, stage2_pub_speed, stage2_8s_middle"
    ;;
esac

case "${ARM}" in
  left|right)
    ;;
  "")
    fail "--arm left|right is required"
    ;;
  *)
    fail "unknown arm: ${ARM}"
    ;;
esac

[[ -f "${MATRIX_FILE}" ]] || fail "matrix file not found: ${MATRIX_FILE}"
SERVER="$(abs_from_root "${SERVER}")"

if [[ -z "${ARTIFACT_ROOT}" ]]; then
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  ARTIFACT_ROOT="${ROOT_DIR}/artifacts/rbpodo_circle_ablation/${timestamp}_${MATRIX_LABEL}_${ARM}"
else
  ARTIFACT_ROOT="$(abs_from_root "${ARTIFACT_ROOT}")"
fi

require_env() {
  local missing=()
  for key in \
    RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM
  do
    if [[ "${!key:-}" != "1" ]]; then
      missing+=("${key}=1")
    fi
  done
  if ((${#missing[@]} > 0)); then
    printf 'rbpodo_circle_tune: missing required env:\n' >&2
    printf '  %s\n' "${missing[@]}" >&2
    printf 'Pass --with-required-env to set these explicitly in this wrapper.\n' >&2
    exit 1
  fi
}

validate_matrix_arm() {
  local arms=()
  mapfile -t arms < <(
    grep -E '^[[:space:]]*arm:[[:space:]]*' "${MATRIX_FILE}" \
      | sed -E 's/.*arm:[[:space:]]*([^[:space:]#]+).*/\1/' \
      | sort -u
  )
  ((${#arms[@]} > 0)) || fail "matrix has no arm entries: ${MATRIX_FILE}"
  local observed
  for observed in "${arms[@]}"; do
    if [[ "${observed}" != "${ARM}" ]]; then
      fail "matrix arm ${observed} does not match --arm ${ARM}; use a matching matrix file"
    fi
  done
}

referenced_configs() {
  {
    printf '%s\n' "${STANDARD_LOCAL_CONFIGS[@]}"
    grep -E '^[[:space:]]*config:[[:space:]]*' "${MATRIX_FILE}" \
      | sed -E 's/.*config:[[:space:]]*([^[:space:]#]+).*/\1/' \
      | while IFS= read -r config_path; do
          [[ -n "${config_path}" ]] || continue
          abs_from_root "${config_path}"
        done
  } | awk 'NF && !seen[$0]++'
}

validate_config() {
  local config="$1"
  [[ -f "${config}" ]] || fail "local config not found: ${config}; run tools/rbpodo_circle_prepare.sh --create-local-configs or pass --force-local-configs"
  if [[ "${config}" == *.example.yaml ]]; then
    fail "do not run tuning matrix from .example.yaml; copy to rb_servo_server/config/local first"
  fi
  if grep -Eq '^[[:space:]]*allow_in_real:[[:space:]]*true([[:space:]]*(#.*)?)?$' "${config}"; then
    fail "${config} has allow_in_real: true"
  fi
  grep -Eq '^[[:space:]]*allow_in_real:[[:space:]]*false([[:space:]]*(#.*)?)?$' "${config}" \
    || fail "${config} must contain allow_in_real: false"
  grep -Eq '^[[:space:]]*allow_in_controller_simulation:[[:space:]]*true([[:space:]]*(#.*)?)?$' "${config}" \
    || fail "${config} must contain allow_in_controller_simulation: true"
  grep -Eq '^[[:space:]]*controller_simulation_tracking_error_source:[[:space:]]*reference([[:space:]]*(#.*)?)?$' "${config}" \
    || fail "${config} must contain controller_simulation_tracking_error_source: reference"
  grep -Eq '^[[:space:]]*controller_simulation_servo_state_source:[[:space:]]*reference([[:space:]]*(#.*)?)?$' "${config}" \
    || fail "${config} must contain controller_simulation_servo_state_source: reference"
  grep -Eq '^[[:space:]]*state_pub_endpoints:[[:space:]]*$' "${config}" \
    || fail "${config} must contain network.state_pub_endpoints"
  local backend_count
  backend_count="$(grep -Ec '^[[:space:]]*backend_type:[[:space:]]*rbpodo([[:space:]]*(#.*)?)?$' "${config}")"
  [[ "${backend_count}" -ge 2 ]] || fail "${config} must set backend_type: rbpodo for both arms"
  local operation_count
  operation_count="$(grep -Ec '^[[:space:]]*operation_mode:[[:space:]]*simulation([[:space:]]*(#.*)?)?$' "${config}")"
  [[ "${operation_count}" -ge 2 ]] || fail "${config} must set operation_mode: simulation for both arms"
  if grep -Eq '^[[:space:]]*operation_mode:[[:space:]]*real([[:space:]]*(#.*)?)?$' "${config}"; then
    fail "${config} contains operation_mode: real"
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
    fail "server binary lacks cap_sys_nice and cap_ipc_lock; run tools/rbpodo_circle_prepare.sh --setcap or pass --allow-no-realtime"
  fi
}

check_tcp_port() {
  local ip="$1"
  local port="$2"
  if ! command -v timeout >/dev/null 2>&1; then
    fail "timeout command unavailable; cannot check ${ip}:${port}"
  fi
  if timeout "${PORT_TIMEOUT_SEC}" bash -c ":</dev/tcp/${ip}/${port}" 2>/dev/null; then
    note "reachable ${ip}:${port}"
  else
    fail "cannot connect to ${ip}:${port}"
  fi
}

print_command() {
  printf 'rbpodo_circle_tune: command:'
  printf ' %q' "$@"
  printf '\n'
}

if [[ "${WITH_REQUIRED_ENV}" == "1" ]]; then
  export RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
else
  require_env
fi

validate_matrix_arm

cmd=(
  python3 scripts/run_rbpodo_circle_ablation.py
  --matrix "${MATRIX_FILE}"
  --artifact-root "${ARTIFACT_ROOT}"
  --server "${SERVER}"
)

if [[ "${SET_PGMODE}" == "1" ]]; then
  PGMODE_SUMMARY="${ARTIFACT_ROOT}/pgmode_simulation.json"
  pgmode_cmd=(
    tools/simulation_mode.sh
    --summary-json "${PGMODE_SUMMARY}"
    --timeout-sec "${PORT_TIMEOUT_SEC}"
  )
  cmd+=(--pgmode-summary-json "${PGMODE_SUMMARY}")
else
  cmd+=(--verify-pgmode-simulation)
fi

[[ "${SKIP_PLOTS}" == "1" ]] && cmd+=(--skip-plots)
[[ "${DRY_RUN}" == "1" ]] && cmd+=(--dry-run)

cd "${ROOT_DIR}"

if [[ "${DRY_RUN}" == "1" ]]; then
  if [[ "${SET_PGMODE}" == "1" ]]; then
    print_command "${pgmode_cmd[@]}"
  fi
  print_command "${cmd[@]}"
  note "dry-run only; command was not executed"
  note "expected report path: ${ARTIFACT_ROOT}/ablation_report.md"
  exit 0
fi

if [[ "${FORCE_LOCAL_CONFIGS}" == "1" ]]; then
  tools/create_rbpodo_circle_local_configs.sh --force
fi

while IFS= read -r config; do
  validate_config "${config}"
done < <(referenced_configs)
note "local circle configs pass safety checks"

check_realtime_caps

if [[ "${CHECK_CONTROLLER}" == "1" ]]; then
  for ip in "${CONTROLLER_IPS[@]}"; do
    check_tcp_port "${ip}" 5000
    check_tcp_port "${ip}" 5001
  done
fi

if [[ "${SET_PGMODE}" == "1" ]]; then
  "${pgmode_cmd[@]}"
fi

print_command "${cmd[@]}"
"${cmd[@]}"

note "ablation report: ${ARTIFACT_ROOT}/ablation_report.md"
note "ablation summary: ${ARTIFACT_ROOT}/ablation_summary.csv"
