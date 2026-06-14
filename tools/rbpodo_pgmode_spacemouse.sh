#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVER="${ROOT_DIR}/rb_servo_server/build/rbpodo_real_gate/rb_servo_server"
SERVER_TEMPLATE_REL="rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml"
SERVER_LOCAL_REL="rb_servo_server/config/local/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.yaml"
POLICY_CONFIG_REL="policy_runner/config/rbpodo_pgmode_spacemouse_500hz_ack.yaml"
POLICY_DRY_RUN_CONFIG="${TMPDIR:-/tmp}/rbpodo_pgmode_spacemouse_policy_dry_run.yaml"
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
  prepare        Create the ignored local server config from the tracked template.
  server         Launch rb_servo_server with the local ACKON500 pgmode SpaceMouse config.
  server-dry-run Print the server launch command and required env gates; do not execute.
  policy-dry-run Print a mock-SpaceMouse policy_runner command; do not require HID devices.
  gui            Launch rb_gui/viser on the SpaceMouse pgmode state port.
  check          Print expected endpoints and required env gates; do not set them.
  teleop-record  Run policy_runner dual SpaceMouse teleop and JSONL recording.
  hdf5-record    Run policy_runner dual SpaceMouse teleop and HDF5 recording.

Safety flags for server/teleop/hdf5-record:
  --i-understand-this-connects-to-real-controller
  --i-confirm-controller-is-in-pgmode-simulation

Environment behavior:
  --with-required-env  Explicitly export the controller-simulation and async
                       env gates for this process.

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

print_required_env() {
  cat <<'EOF'
rbpodo_pgmode_spacemouse: required rbpodo pgmode simulation env flags:
  RB_ALLOW_RBPODO_ASYNC_STREAMING=1
  RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
EOF
}

print_endpoint_check() {
  cat <<'EOF'
rbpodo_pgmode_spacemouse: expected endpoints:
  servo command UDP endpoint: 127.0.0.1:50256
  policy safety readback: 0.0.0.0:50376
  viewer state fanout: 127.0.0.1:50366
EOF
  print_required_env
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
}

set_required_env_if_requested() {
  if [[ "${WITH_REQUIRED_ENV}" != "1" ]]; then
    return 0
  fi
  export RB_ALLOW_RBPODO_ASYNC_STREAMING=1
  export RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
}

require_server_env() {
  local missing=()
  for key in \
    RB_ALLOW_RBPODO_ASYNC_STREAMING \
    RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM
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
  local generator="${ROOT_DIR}/tools/create_rbpodo_pgmode_spacemouse_local_config.sh"
  [[ -f "${generator}" ]] || fail "generator not found: tools/create_rbpodo_pgmode_spacemouse_local_config.sh"
  local args=(--root "${ROOT_DIR}" --output "${SERVER_LOCAL_REL}")
  if [[ "${FORCE}" == "1" ]]; then
    args+=(--force)
  fi
  bash "${generator}" "${args[@]}"
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

run_server_dry_run() {
  if [[ -f "${ROOT_DIR}/${SERVER_LOCAL_REL}" ]]; then
    note "server config: ${SERVER_LOCAL_REL}"
  else
    note "dry-run: local config missing; run prepare before a real server launch"
  fi
  if [[ -f "${SERVER}" ]]; then
    print_binary_fingerprint "${SERVER}"
  else
    note "dry-run: server binary not found: ${SERVER}"
  fi
  local cmd=("${SERVER}" --config "${ROOT_DIR}/${SERVER_LOCAL_REL}")
  print_command "${cmd[@]}"
  print_required_env
}

run_gui() {
  if [[ "${FORCE}" != "1" ]]; then
    port_in_use 50366 && fail "UDP state port 50366 is already in use; pass --force to skip this check"
  fi
  local gui_host="${RB_GUI_HOST:-0.0.0.0}"
  local gui_port="${RB_GUI_PORT:-8080}"
  export PYTHONPATH="${ROOT_DIR}/rb_gui${PYTHONPATH:+:${PYTHONPATH}}"
  export RB_GUI_DESCRIPTIONS_DIR="${ROOT_DIR}/rb_servo_server/descriptions"
  export RB_GUI_STATE_BIND="0.0.0.0"
  export RB_GUI_STATE_PORT="50366"
  export RB_GUI_CIRCLE_OVERLAY_BIND="none"
  local cmd=(python3 -m rb_servo_gui.app)
  note "viewer URL: http://127.0.0.1:${gui_port} (bind ${gui_host}:${gui_port})"
  note "viewer state UDP: ${RB_GUI_STATE_BIND}:${RB_GUI_STATE_PORT}; circle overlay disabled"
  note "viewer is state-only for this workflow; SpaceMouse commands route through policy_runner"
  print_command "${cmd[@]}"
  [[ "${DRY_RUN}" == "1" ]] && return 0
  cd "${ROOT_DIR}"
  exec "${cmd[@]}"
}

write_policy_dry_run_config() {
  cat >"${POLICY_DRY_RUN_CONFIG}" <<'EOF'
schema: robotics_lab.policy_runner.v1
mode: real
action_source: dual_spacemouse_cartesian
runtime:
  startup_timeout_sec: 5.0
geometry:
  path: "calibration/active_calibration.yaml"
robot_state:
  bind: "udp://0.0.0.0:50376"
  stale_timeout_sec: 0.5
servo_command:
  endpoint: "udp://127.0.0.1:50256"
  timeout_sec: 0.05
  acquire_lease: true
  lease_readback_timeout_sec: 2.0
safety:
  allow_real_motion: false
  allow_rbpodo_controller_simulation_cartesian: true
  allow_configured_estimate_geometry_in_controller_simulation: true
  allow_configured_estimate_geometry_in_real: false
  require_valid_joint_state: true
command_rate_hz: 500
spacemouse_cartesian_dual:
  frame: local
  max_linear_velocity_m_s: 0.2
  max_angular_velocity_rad_s: 0.4
  deadband: 0.08
  response_curve_gamma: 3.0
  sample_hold_timeout_sec: 0.05
  left:
    mock_script: pgmode_spacemouse_smoke
    deadman_button: 0
  right:
    mock_script: pgmode_spacemouse_smoke
    deadman_button: 0
EOF
}

run_policy_dry_run() {
  write_policy_dry_run_config
  note "wrote mock SpaceMouse config: ${POLICY_DRY_RUN_CONFIG}"
  local cmd=(
    env "PYTHONPATH=${ROOT_DIR}/policy_runner${PYTHONPATH:+:${PYTHONPATH}}"
    python3 -m policy_runner
    --config "${POLICY_DRY_RUN_CONFIG}"
  )
  print_command "${cmd[@]}"
  note "dry-run only: command is not executed and no HID device is opened"
  print_endpoint_check
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
  server-dry-run)
    run_server_dry_run
    ;;
  policy-dry-run)
    run_policy_dry_run
    ;;
  gui)
    run_gui
    ;;
  check)
    print_endpoint_check
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
