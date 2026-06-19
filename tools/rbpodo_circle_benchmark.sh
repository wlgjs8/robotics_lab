#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVER="${ROOT_DIR}/rb_servo_server/build/rbpodo_real_gate/rb_servo_server"
PROFILE=""
ARM=""
WITH_REQUIRED_ENV=0
VERIFY_PGMODE=0
ALLOW_NO_REALTIME=0
ARTIFACT_DIR=""
OVERLAY_ENDPOINT="udp://127.0.0.1:50261"
OVERLAY_RATE_HZ="20"
FEEDBACK_KP_POS=""
FEEDBACK_KP_ORI=""
FEEDBACK_MAX_LINEAR=""
FEEDBACK_MAX_ANGULAR=""
REPEAT_OVERRIDE=""
COMMAND_RATE_OVERRIDE=""
CONTROLLER_OVERRIDE=""
PREFLIGHT_ONLY=0
SKIP_PLOTS=0

usage() {
  cat <<'EOF'
Usage: tools/rbpodo_circle_benchmark.sh --profile stable|gene --arm left|right [options]

Run the rbpodo controller-simulation circle benchmark with short profile names.

Environment behavior:
  --with-required-env  Explicitly export:
    RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1

Without --with-required-env, those env vars must already be set.

Profiles:
  stable  local 15cm/16s config, twist_stand, repeat 3, 100 Hz
  gene    local 15cm/4s config, twist_stand_feedback, repeat 5, 100 Hz

Options:
  --server PATH
  --artifact-dir DIR
  --verify-pgmode-simulation       Verify instead of sending pgmode simulation.
  --allow-no-realtime              Allow missing setcap on server binary.
  --feedback-kp-pos VALUE          Gene feedback override, default 2.0.
  --feedback-kp-ori VALUE          Gene feedback override, default 2.0.
  --feedback-max-linear-m-s VALUE  Gene feedback override, default 0.15.
  --feedback-max-angular-rad-s VALUE
                                  Gene feedback override, default 0.4.
  --repeat N
  --command-rate-hz HZ
  --controller NAME
  --overlay-pub-endpoint ENDPOINT  Default udp://127.0.0.1:50261.
  --overlay-pub-rate-hz HZ         Default 20.
  --preflight-only
  --skip-plots
  -h, --help                       Show this help.

This wrapper does not set RB_ALLOW_* variables unless --with-required-env is
passed. It refuses physical-real Cartesian configs.
EOF
}

fail() {
  echo "rbpodo_circle_benchmark: ERROR: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --profile)
      [[ $# -ge 2 ]] || fail "--profile requires stable or gene"
      PROFILE="$2"
      shift 2
      ;;
    --arm)
      [[ $# -ge 2 ]] || fail "--arm requires left or right"
      ARM="$2"
      shift 2
      ;;
    --server)
      [[ $# -ge 2 ]] || fail "--server requires a path"
      SERVER="$2"
      shift 2
      ;;
    --artifact-dir)
      [[ $# -ge 2 ]] || fail "--artifact-dir requires a path"
      ARTIFACT_DIR="$2"
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
    --allow-no-realtime)
      ALLOW_NO_REALTIME=1
      shift
      ;;
    --feedback-kp-pos)
      [[ $# -ge 2 ]] || fail "--feedback-kp-pos requires a value"
      FEEDBACK_KP_POS="$2"
      shift 2
      ;;
    --feedback-kp-ori)
      [[ $# -ge 2 ]] || fail "--feedback-kp-ori requires a value"
      FEEDBACK_KP_ORI="$2"
      shift 2
      ;;
    --feedback-max-linear-m-s)
      [[ $# -ge 2 ]] || fail "--feedback-max-linear-m-s requires a value"
      FEEDBACK_MAX_LINEAR="$2"
      shift 2
      ;;
    --feedback-max-angular-rad-s)
      [[ $# -ge 2 ]] || fail "--feedback-max-angular-rad-s requires a value"
      FEEDBACK_MAX_ANGULAR="$2"
      shift 2
      ;;
    --repeat)
      [[ $# -ge 2 ]] || fail "--repeat requires a value"
      REPEAT_OVERRIDE="$2"
      shift 2
      ;;
    --command-rate-hz)
      [[ $# -ge 2 ]] || fail "--command-rate-hz requires a value"
      COMMAND_RATE_OVERRIDE="$2"
      shift 2
      ;;
    --controller)
      [[ $# -ge 2 ]] || fail "--controller requires a value"
      CONTROLLER_OVERRIDE="$2"
      shift 2
      ;;
    --overlay-pub-endpoint)
      [[ $# -ge 2 ]] || fail "--overlay-pub-endpoint requires a value"
      OVERLAY_ENDPOINT="$2"
      shift 2
      ;;
    --overlay-pub-rate-hz)
      [[ $# -ge 2 ]] || fail "--overlay-pub-rate-hz requires a value"
      OVERLAY_RATE_HZ="$2"
      shift 2
      ;;
    --preflight-only)
      PREFLIGHT_ONLY=1
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

case "${PROFILE}" in
  stable)
    CONFIG="${ROOT_DIR}/rb_servo_server/config/local/dual_real_rbpodo_circle_15cm16s.yaml"
    BENCH_PROFILE="circle_15cm_16s"
    CONTROLLER="${CONTROLLER_OVERRIDE:-twist_stand}"
    REPEAT="${REPEAT_OVERRIDE:-3}"
    COMMAND_RATE="${COMMAND_RATE_OVERRIDE:-100}"
    ;;
  gene)
    CONFIG="${ROOT_DIR}/rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s.yaml"
    BENCH_PROFILE="gene_15cm_4s"
    CONTROLLER="${CONTROLLER_OVERRIDE:-twist_stand_feedback}"
    REPEAT="${REPEAT_OVERRIDE:-5}"
    COMMAND_RATE="${COMMAND_RATE_OVERRIDE:-100}"
    FEEDBACK_KP_POS="${FEEDBACK_KP_POS:-2.0}"
    FEEDBACK_KP_ORI="${FEEDBACK_KP_ORI:-2.0}"
    FEEDBACK_MAX_LINEAR="${FEEDBACK_MAX_LINEAR:-0.15}"
    FEEDBACK_MAX_ANGULAR="${FEEDBACK_MAX_ANGULAR:-0.4}"
    ;;
  "")
    fail "--profile stable|gene is required"
    ;;
  *)
    fail "unknown profile: ${PROFILE}"
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

validate_config() {
  local config="$1"
  [[ -f "${config}" ]] || fail "local config not found: ${config}; run tools/rbpodo_circle_prepare.sh --create-local-configs"
  if [[ "${config}" == *.example.yaml ]]; then
    fail "do not run benchmark from .example.yaml; copy to rb_servo_server/config/local first"
  fi
  if grep -Eq '^[[:space:]]*allow_in_real:[[:space:]]*true([[:space:]]*(#.*)?)?$' "${config}"; then
    fail "${config} has cartesian_control.allow_in_real=true"
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
}

check_realtime_caps() {
  [[ -f "${SERVER}" ]] || fail "server binary not found: ${SERVER}"
  if ! command -v getcap >/dev/null 2>&1; then
    [[ "${ALLOW_NO_REALTIME}" == "1" ]] && return 0
    fail "getcap unavailable; pass --allow-no-realtime only for explicit non-realtime testing"
  fi
  local caps
  caps="$(getcap "${SERVER}" 2>/dev/null || true)"
  if [[ "${caps}" != *"cap_sys_nice"* || "${caps}" != *"cap_ipc_lock"* ]]; then
    [[ "${ALLOW_NO_REALTIME}" == "1" ]] && {
      echo "rbpodo_circle_benchmark: warning: realtime capabilities missing: ${caps:-<none>}" >&2
      return 0
    }
    fail "server binary lacks cap_sys_nice and cap_ipc_lock; run tools/rbpodo_circle_prepare.sh --setcap or pass --allow-no-realtime"
  fi
}

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
    printf 'rbpodo_circle_benchmark: missing required env:\n' >&2
    printf '  %s\n' "${missing[@]}" >&2
    printf 'Pass --with-required-env to set these explicitly in this wrapper.\n' >&2
    exit 1
  fi
}

if [[ "${WITH_REQUIRED_ENV}" == "1" ]]; then
  export RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
else
  require_env
fi

validate_config "${CONFIG}"
check_realtime_caps

if [[ -z "${ARTIFACT_DIR}" ]]; then
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  ARTIFACT_DIR="${ROOT_DIR}/artifacts/rbpodo_circle/${timestamp}_${PROFILE}_${ARM}"
fi

PGMODE_FLAG="--set-pgmode-simulation"
[[ "${VERIFY_PGMODE}" == "1" ]] && PGMODE_FLAG="--verify-pgmode-simulation"

cmd=(
  python3 scripts/rbpodo_circle_tracking_benchmark.py
  --server "${SERVER}"
  --server-config "${CONFIG}"
  --arm "${ARM}"
  --controller "${CONTROLLER}"
  --profile "${BENCH_PROFILE}"
  --repeat "${REPEAT}"
  --command-rate-hz "${COMMAND_RATE}"
  --tracking-source tcp_ref_stand
  --overlay-pub-endpoint "${OVERLAY_ENDPOINT}"
  --overlay-pub-rate-hz "${OVERLAY_RATE_HZ}"
  "${PGMODE_FLAG}"
  --artifact-dir "${ARTIFACT_DIR}"
)

if [[ "${PROFILE}" == "gene" ]]; then
  cmd+=(
    --feedback-kp-pos "${FEEDBACK_KP_POS}"
    --feedback-kp-ori "${FEEDBACK_KP_ORI}"
    --feedback-max-linear-m-s "${FEEDBACK_MAX_LINEAR}"
    --feedback-max-angular-rad-s "${FEEDBACK_MAX_ANGULAR}"
  )
fi

[[ "${PREFLIGHT_ONLY}" == "1" ]] && cmd+=(--preflight-only)
[[ "${SKIP_PLOTS}" == "1" ]] && cmd+=(--skip-plots)

cd "${ROOT_DIR}"
printf 'rbpodo_circle_benchmark: running:'
printf ' %q' "${cmd[@]}"
printf '\n'
exec "${cmd[@]}"
