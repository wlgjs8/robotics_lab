#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVER="${ROOT_DIR}/rb_servo_server/build/rbpodo_real_gate/rb_servo_server"
SERVER_TEMPLATE_REL="rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml"
SERVER_LOCAL_REL="rb_servo_server/config/local/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.yaml"
POLICY_CONFIG_REL="policy_runner/config/rbpodo_pgmode_spacemouse_500hz_ack.yaml"
OUTPUT_DIR="policy_runner/episodes"
TASK=""
OPERATOR=""
FORCE=0
WITH_REQUIRED_ENV=0
CONFIRM_CONNECTS=0
CONFIRM_PGMODE=0
DRY_RUN=0

usage() {
  cat <<'EOF'
Usage: tools/rbpodo_pgmode_spacemouse.sh ACTION [options]

Actions:
  prepare        Copy the tracked server template to rb_servo_server/config/local/.
  server         Launch rb_servo_server with the local ACKON500 pgmode SpaceMouse config.
  gui            Launch rb_gui/viser on the SpaceMouse pgmode state port.
  teleop-record  Run policy_runner dual SpaceMouse teleop and JSONL recording.
  hdf5-record    Run policy_runner dual SpaceMouse teleop and HDF5 recording.

Safety flags for server/teleop/hdf5-record:
  --i-understand-this-connects-to-real-controller
  --i-confirm-controller-is-in-pgmode-simulation

Environment behavior:
  --with-required-env  Explicitly export the controller-simulation and async
                       env gates for this process. The script never sets
                       RB_ALLOW_REAL_CARTESIAN.

Options:
  --server PATH       rb_servo_server binary path for the server action.
  --output-dir DIR    Recording output dir for teleop-record/hdf5-record.
  --task TEXT         Required task description for hdf5-record.
  --operator ID       Optional operator ID for hdf5-record.
  --force             Overwrite local config on prepare or skip GUI port check.
  --dry-run           Print the command instead of executing it.
  -h, --help          Show this help.
EOF
}

fail() {
  echo "rbpodo_pgmode_spacemouse: ERROR: $*" >&2
  exit 1
}

note() {
  echo "rbpodo_pgmode_spacemouse: $*"
}

print_command() {
  printf 'rbpodo_pgmode_spacemouse: command:'
  printf ' %q' "$@"
  printf '\n'
}

print_binary_fingerprint() {
  local binary="$1"
  [[ -f "${binary}" ]] || return 0
  local mtime size digest
  if command -v stat >/dev/null 2>&1; then
    mtime="$(stat -c '%y' "${binary}" 2>/dev/null || true)"
    size="$(stat -c '%s' "${binary}" 2>/dev/null || true)"
  fi
  if command -v sha256sum >/dev/null 2>&1; then
    digest="$(sha256sum "${binary}" | awk '{print substr($1, 1, 12)}')"
  else
    digest="unavailable"
  fi
  note "server binary: ${binary}"
  note "server binary mtime: ${mtime:-unknown}"
  note "server binary size: ${size:-unknown} sha256_12: ${digest}"
}

port_in_use() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -H -lun 2>/dev/null | awk '{print $5}' | grep -Eq "(^|[:.])${port}$"
    return $?
  fi
  if command -v lsof >/dev/null 2>&1; then
    lsof -nP -iUDP:"${port}" >/dev/null 2>&1
    return $?
  fi
  echo "rbpodo_pgmode_spacemouse: warning: ss/lsof unavailable; skipping UDP port check for ${port}" >&2
  return 1
}

require_confirmations() {
  [[ "${CONFIRM_CONNECTS}" == "1" ]] || fail "missing --i-understand-this-connects-to-real-controller"
  [[ "${CONFIRM_PGMODE}" == "1" ]] || fail "missing --i-confirm-controller-is-in-pgmode-simulation"
  if [[ "${RB_ALLOW_REAL_CARTESIAN:-}" == "1" ]]; then
    fail "RB_ALLOW_REAL_CARTESIAN must not be set for pgmode SpaceMouse simulation"
  fi
}

set_required_env_if_requested() {
  if [[ "${WITH_REQUIRED_ENV}" != "1" ]]; then
    return 0
  fi
  export RB_ALLOW_REAL_ROBOT=1
  export RB_ALLOW_REAL_MOTION=1
  export RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
  export RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
  export RB_ALLOW_RBPODO_ASYNC_STREAMING=1
  export RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
  export RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1
}

require_server_env() {
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
    printf 'rbpodo_pgmode_spacemouse: missing required env:\n' >&2
    printf '  %s\n' "${missing[@]}" >&2
    printf 'Pass --with-required-env to set these explicitly in this wrapper.\n' >&2
    exit 1
  fi
}

prepare_local_config() {
  local template="${ROOT_DIR}/${SERVER_TEMPLATE_REL}"
  local local_copy="${ROOT_DIR}/${SERVER_LOCAL_REL}"
  [[ -f "${template}" ]] || fail "template not found: ${SERVER_TEMPLATE_REL}"
  mkdir -p "$(dirname "${local_copy}")"
  if [[ -f "${local_copy}" && "${FORCE}" != "1" ]]; then
    note "local config already exists: ${SERVER_LOCAL_REL}"
    return 0
  fi
  cp "${template}" "${local_copy}"
  note "wrote ${SERVER_LOCAL_REL}"
}

run_server() {
  require_confirmations
  set_required_env_if_requested
  require_server_env
  if [[ -f "${ROOT_DIR}/${SERVER_LOCAL_REL}" ]]; then
    :
  elif [[ "${DRY_RUN}" == "1" ]]; then
    note "dry-run: local config missing; run prepare before a real server launch"
  else
    fail "local config missing; run prepare first"
  fi
  if [[ -f "${SERVER}" ]]; then
    :
  elif [[ "${DRY_RUN}" == "1" ]]; then
    note "dry-run: server binary not found: ${SERVER}"
  else
    fail "server binary not found: ${SERVER}"
  fi
  local cmd=("${SERVER}" --config "${ROOT_DIR}/${SERVER_LOCAL_REL}")
  print_binary_fingerprint "${SERVER}"
  print_command "${cmd[@]}"
  [[ "${DRY_RUN}" == "1" ]] && return 0
  cd "${ROOT_DIR}"
  exec "${cmd[@]}"
}

run_gui() {
  if [[ "${FORCE}" != "1" ]]; then
    port_in_use 50366 && fail "UDP state port 50366 is already in use; pass --force to skip this check"
  fi
  export PYTHONPATH="${ROOT_DIR}/rb_gui${PYTHONPATH:+:${PYTHONPATH}}"
  export RB_GUI_DESCRIPTIONS_DIR="${ROOT_DIR}/rb_servo_server/descriptions"
  export RB_GUI_STATE_BIND="0.0.0.0"
  export RB_GUI_STATE_PORT="50366"
  export RB_GUI_CIRCLE_OVERLAY_BIND="none"
  local cmd=(python3 -m rb_servo_gui.app)
  print_command "${cmd[@]}"
  [[ "${DRY_RUN}" == "1" ]] && return 0
  cd "${ROOT_DIR}"
  exec "${cmd[@]}"
}

run_teleop_record() {
  require_confirmations
  set_required_env_if_requested
  export PYTHONPATH="${ROOT_DIR}/policy_runner${PYTHONPATH:+:${PYTHONPATH}}"
  local cmd=(
    python3 -m policy_runner teleop-record
    --config "${ROOT_DIR}/${POLICY_CONFIG_REL}"
    --output-dir "${ROOT_DIR}/${OUTPUT_DIR}"
  )
  print_command "${cmd[@]}"
  [[ "${DRY_RUN}" == "1" ]] && return 0
  cd "${ROOT_DIR}"
  exec "${cmd[@]}"
}

run_hdf5_record() {
  require_confirmations
  [[ -n "${TASK}" ]] || fail "--task is required for hdf5-record"
  set_required_env_if_requested
  export PYTHONPATH="${ROOT_DIR}/policy_runner${PYTHONPATH:+:${PYTHONPATH}}"
  local cmd=(
    python3 -m policy_runner hdf5-record
    --config "${ROOT_DIR}/${POLICY_CONFIG_REL}"
    --output-dir "${ROOT_DIR}/${OUTPUT_DIR}"
    --task "${TASK}"
  )
  if [[ -n "${OPERATOR}" ]]; then
    cmd+=(--operator "${OPERATOR}")
  fi
  print_command "${cmd[@]}"
  [[ "${DRY_RUN}" == "1" ]] && return 0
  cd "${ROOT_DIR}"
  exec "${cmd[@]}"
}

ACTION="${1:-}"
if [[ -z "${ACTION}" ]]; then
  usage >&2
  exit 2
fi
shift

while (($# > 0)); do
  case "$1" in
    --server)
      [[ $# -ge 2 ]] || fail "--server requires a path"
      SERVER="$2"
      shift 2
      ;;
    --output-dir)
      [[ $# -ge 2 ]] || fail "--output-dir requires a directory"
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --task)
      [[ $# -ge 2 ]] || fail "--task requires text"
      TASK="$2"
      shift 2
      ;;
    --operator)
      [[ $# -ge 2 ]] || fail "--operator requires an ID"
      OPERATOR="$2"
      shift 2
      ;;
    --force)
      FORCE=1
      shift
      ;;
    --with-required-env)
      WITH_REQUIRED_ENV=1
      shift
      ;;
    --i-understand-this-connects-to-real-controller)
      CONFIRM_CONNECTS=1
      shift
      ;;
    --i-confirm-controller-is-in-pgmode-simulation)
      CONFIRM_PGMODE=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
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

case "${ACTION}" in
  prepare)
    prepare_local_config
    ;;
  server)
    run_server
    ;;
  gui)
    run_gui
    ;;
  teleop-record)
    run_teleop_record
    ;;
  hdf5-record)
    run_hdf5_record
    ;;
  -h|--help)
    usage
    ;;
  *)
    echo "unknown action: ${ACTION}" >&2
    usage >&2
    exit 2
    ;;
esac
