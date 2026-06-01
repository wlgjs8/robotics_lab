#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/rb_servo_server/build/rbpodo_real_gate"
SERVER="${BUILD_DIR}/rb_servo_server"
CREATE_LOCAL=0
FORCE_LOCAL=0
DO_BUILD=0
DO_SETCAP=0
CHECK_PORTS=0
SET_PGMODE=0
VERIFY_PGMODE=0
CONFIRM=0
PORT_TIMEOUT_SEC="1.0"

CONTROLLER_IPS=("172.28.60.200" "172.28.60.201")
LOCAL_CONFIGS=(
  "${ROOT_DIR}/rb_servo_server/config/local/dual_real_rbpodo_circle_15cm16s.yaml"
  "${ROOT_DIR}/rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s.yaml"
)

usage() {
  cat <<'EOF'
Usage: tools/rbpodo_circle_prepare.sh [options]

Prepare rbpodo controller-simulation circle local configs and checks.

Options:
  --create-local-configs       Copy circle templates to rb_servo_server/config/local.
  --force-local-configs        Copy local configs with overwrite.
  --build                      Build rb_servo_server/build/rbpodo_real_gate.
  --setcap                     Apply cap_sys_nice,cap_ipc_lock+ep to server binary.
  --check-ports                Check controller TCP ports 5000 and 5001.
  --set-pgmode-simulation      Send pgmode simulation via tools/simulation_mode.sh.
  --verify-pgmode-simulation   Verify pgmode simulation without sending pgmode.
  --server PATH                rb_servo_server binary path.
  --port-timeout-sec SEC       TCP port check timeout, default 1.0.
  --i-understand-this-connects-to-real-controller
                               Required for controller port checks and pgmode commands.
  -h, --help                   Show this help.

This script never sets RB_ALLOW_* variables. Controller-touching options
require the explicit confirmation flag and RB_ALLOW_REAL_ROBOT=1.
EOF
}

fail() {
  echo "rbpodo_circle_prepare: ERROR: $*" >&2
  exit 1
}

note() {
  echo "rbpodo_circle_prepare: $*"
}

while (($# > 0)); do
  case "$1" in
    --create-local-configs)
      CREATE_LOCAL=1
      shift
      ;;
    --force-local-configs)
      CREATE_LOCAL=1
      FORCE_LOCAL=1
      shift
      ;;
    --build)
      DO_BUILD=1
      shift
      ;;
    --setcap)
      DO_SETCAP=1
      shift
      ;;
    --check-ports)
      CHECK_PORTS=1
      shift
      ;;
    --set-pgmode-simulation)
      SET_PGMODE=1
      shift
      ;;
    --verify-pgmode-simulation)
      VERIFY_PGMODE=1
      shift
      ;;
    --server)
      [[ $# -ge 2 ]] || fail "--server requires a path"
      SERVER="$2"
      shift 2
      ;;
    --port-timeout-sec)
      [[ $# -ge 2 ]] || fail "--port-timeout-sec requires a value"
      PORT_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --i-understand-this-connects-to-real-controller)
      CONFIRM=1
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

if [[ "${SET_PGMODE}" == "1" && "${VERIFY_PGMODE}" == "1" ]]; then
  fail "--set-pgmode-simulation and --verify-pgmode-simulation are mutually exclusive"
fi

require_controller_confirmation() {
  if [[ "${CONFIRM}" != "1" ]]; then
    fail "controller checks require --i-understand-this-connects-to-real-controller"
  fi
  if [[ "${RB_ALLOW_REAL_ROBOT:-}" != "1" ]]; then
    fail "controller checks require RB_ALLOW_REAL_ROBOT=1"
  fi
}

validate_config() {
  local config="$1"
  [[ -f "${config}" ]] || fail "local config not found: ${config}; run --create-local-configs or --force-local-configs"
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
  local strict="$1"
  if [[ ! -f "${SERVER}" ]]; then
    [[ "${strict}" == "0" ]] && return 0
    fail "server binary not found: ${SERVER}"
  fi
  if ! command -v getcap >/dev/null 2>&1; then
    [[ "${strict}" == "0" ]] && { note "getcap unavailable; realtime capability not checked"; return 0; }
    fail "getcap unavailable; cannot verify realtime capability"
  fi
  local caps
  caps="$(getcap "${SERVER}" 2>/dev/null || true)"
  if [[ "${caps}" != *"cap_sys_nice"* || "${caps}" != *"cap_ipc_lock"* ]]; then
    [[ "${strict}" == "0" ]] && { note "warning: realtime capabilities missing on ${SERVER}"; return 0; }
    fail "server binary lacks cap_sys_nice and cap_ipc_lock: ${caps:-<none>}"
  fi
  note "realtime capabilities ok: ${caps}"
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

cd "${ROOT_DIR}"

if [[ "${CREATE_LOCAL}" == "1" ]]; then
  args=()
  [[ "${FORCE_LOCAL}" == "1" ]] && args+=(--force)
  tools/create_rbpodo_circle_local_configs.sh "${args[@]}"
fi

for config in "${LOCAL_CONFIGS[@]}"; do
  validate_config "${config}"
done
note "local circle configs pass safety checks"

if [[ "${DO_BUILD}" == "1" ]]; then
  cmake -S rb_servo_server -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DRB_SERVO_ENABLE_RBPODO=ON \
    -DBUILD_TESTING=ON
  cmake --build "${BUILD_DIR}" -j
fi

if [[ "${DO_SETCAP}" == "1" ]]; then
  [[ -f "${SERVER}" ]] || fail "server binary not found: ${SERVER}"
  if [[ "${EUID}" -eq 0 ]]; then
    setcap cap_sys_nice,cap_ipc_lock+ep "${SERVER}"
  else
    sudo setcap cap_sys_nice,cap_ipc_lock+ep "${SERVER}"
  fi
  check_realtime_caps 1
elif [[ "${DO_BUILD}" == "1" ]]; then
  check_realtime_caps 1
else
  check_realtime_caps 0
fi

if [[ "${CHECK_PORTS}" == "1" ]]; then
  require_controller_confirmation
  for ip in "${CONTROLLER_IPS[@]}"; do
    check_tcp_port "${ip}" 5000
    check_tcp_port "${ip}" 5001
  done
fi

if [[ "${SET_PGMODE}" == "1" || "${VERIFY_PGMODE}" == "1" ]]; then
  require_controller_confirmation
  pgmode_args=(--summary-json artifacts/rbpodo_controller_sim_circle/pgmode_simulation.json)
  [[ "${VERIFY_PGMODE}" == "1" ]] && pgmode_args=(--verify-only --summary-json artifacts/rbpodo_controller_sim_circle/pgmode_verify.json)
  tools/simulation_mode.sh \
    "${pgmode_args[@]}" \
    --i-understand-this-connects-to-real-controller
fi

cat <<'EOF'

Next commands:
  tools/rbpodo_circle_gui.sh --profile stable

  tools/rbpodo_circle_tune.sh \
    --matrix stage2_gain_split \
    --arm left \
    --with-required-env \
    --i-understand-this-connects-to-real-controller \
    --i-confirm-controller-is-in-pgmode-simulation

  tools/rbpodo_circle_benchmark.sh \
    --profile stable \
    --arm left \
    --with-required-env \
    --i-understand-this-connects-to-real-controller \
    --i-confirm-controller-is-in-pgmode-simulation

Required env gates are not set by prepare:
  RB_ALLOW_REAL_ROBOT=1
  RB_ALLOW_REAL_MOTION=1
  RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
  RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
EOF
