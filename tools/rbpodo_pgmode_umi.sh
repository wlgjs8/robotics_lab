#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SPACEMOUSE_WRAPPER="${ROOT_DIR}/tools/rbpodo_pgmode_spacemouse.sh"
POLICY_CONFIG_REL="policy_runner/config/rbpodo_pgmode_umi_500hz_ack.yaml"
OUTPUT_DIR="policy_runner/episodes"
TASK=""
OPERATOR=""
DRY_RUN=0
CONFIRM_CONNECTS=0
CONFIRM_PGMODE=0

usage() {
  cat <<'EOF'
Usage: tools/rbpodo_pgmode_umi.sh ACTION [options]

Actions:
  prepare        Create the ignored local server config through the SpaceMouse pgmode wrapper.
  server         Launch rb_servo_server with the shared ACKON500 pgmode server config.
  server-dry-run Print the server launch command and required env gates; do not execute.
  policy         Run policy_runner with umi_dual_cartesian.
  policy-dry-run Print the mock-UMI policy_runner command; do not open tracker hardware.
  gui            Launch rb_gui/viser on the shared pgmode state port.
  check          Print expected endpoints and required env gates; do not set them.
  teleop-record  Run UMI teleop and JSONL recording.
  hdf5-record    Run UMI teleop and HDF5 recording.

Safety flags for server/policy/teleop/hdf5-record:
  --i-understand-this-connects-to-real-controller
  --i-confirm-controller-is-in-pgmode-simulation

Options:
  --with-required-env  Forward to the server action only; never sets RB_ALLOW_REAL_CARTESIAN.
  --server PATH        Forward server binary path to the server wrapper.
  --output-dir DIR     Recording output dir for teleop-record/hdf5-record.
  --task TEXT          Required task description for hdf5-record.
  --operator ID        Optional operator ID for hdf5-record.
  --force              Forward to prepare/gui server wrapper actions.
  --dry-run            Print the command instead of executing policy/record actions.
  -h, --help           Show this help.
EOF
}

fail() {
  echo "rbpodo_pgmode_umi: ERROR: $*" >&2
  exit 1
}

note() {
  echo "rbpodo_pgmode_umi: $*"
}

print_command() {
  printf 'rbpodo_pgmode_umi: command:'
  printf ' %q' "$@"
  printf '\n'
}

require_confirmations() {
  [[ "${CONFIRM_CONNECTS}" == "1" ]] || fail "missing --i-understand-this-connects-to-real-controller"
  [[ "${CONFIRM_PGMODE}" == "1" ]] || fail "missing --i-confirm-controller-is-in-pgmode-simulation"
  if [[ "${RB_ALLOW_REAL_CARTESIAN:-}" == "1" ]]; then
    fail "RB_ALLOW_REAL_CARTESIAN must not be set for pgmode UMI simulation"
  fi
}

print_policy_info() {
  cat <<'EOF'
rbpodo_pgmode_umi: policy config: policy_runner/config/rbpodo_pgmode_umi_500hz_ack.yaml
rbpodo_pgmode_umi: action_source: umi_dual_cartesian
rbpodo_pgmode_umi: default readers: mock_script pgmode_umi_smoke for left and right
rbpodo_pgmode_umi: policy safety keeps allow_real_motion=false and uses only the rbpodo controller-simulation Cartesian carve-out.
rbpodo_pgmode_umi: RB_ALLOW_REAL_CARTESIAN must remain unset for this workflow.
EOF
}

delegate_server_wrapper() {
  [[ -x "${SPACEMOUSE_WRAPPER}" ]] || fail "missing shared server wrapper: tools/rbpodo_pgmode_spacemouse.sh"
  exec "${SPACEMOUSE_WRAPPER}" "$@"
}

run_policy() {
  require_confirmations
  export PYTHONPATH="${ROOT_DIR}/policy_runner${PYTHONPATH:+:${PYTHONPATH}}"
  local cmd=(python3 -m policy_runner --config "${ROOT_DIR}/${POLICY_CONFIG_REL}")
  print_command "${cmd[@]}"
  [[ "${DRY_RUN}" == "1" ]] && return 0
  cd "${ROOT_DIR}"
  exec "${cmd[@]}"
}

run_policy_dry_run() {
  print_policy_info
  local cmd=(
    env "PYTHONPATH=${ROOT_DIR}/policy_runner${PYTHONPATH:+:${PYTHONPATH}}"
    python3 -m policy_runner
    --config "${ROOT_DIR}/${POLICY_CONFIG_REL}"
  )
  print_command "${cmd[@]}"
  note "dry-run only: command is not executed, UDP tracker sockets are not opened, and no env gates are set"
}

run_teleop_record() {
  require_confirmations
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

FORWARD_ARGS=()
while (($# > 0)); do
  case "$1" in
    --server|--output-dir|--task|--operator)
      [[ $# -ge 2 ]] || fail "$1 requires a value"
      if [[ "$1" == "--output-dir" ]]; then
        OUTPUT_DIR="$2"
      elif [[ "$1" == "--task" ]]; then
        TASK="$2"
      elif [[ "$1" == "--operator" ]]; then
        OPERATOR="$2"
      fi
      FORWARD_ARGS+=("$1" "$2")
      shift 2
      ;;
    --force|--with-required-env)
      FORWARD_ARGS+=("$1")
      shift
      ;;
    --i-understand-this-connects-to-real-controller)
      CONFIRM_CONNECTS=1
      FORWARD_ARGS+=("$1")
      shift
      ;;
    --i-confirm-controller-is-in-pgmode-simulation)
      CONFIRM_PGMODE=1
      FORWARD_ARGS+=("$1")
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      FORWARD_ARGS+=("$1")
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
  prepare|server|server-dry-run|gui|check)
    delegate_server_wrapper "${ACTION}" "${FORWARD_ARGS[@]}"
    ;;
  policy)
    run_policy
    ;;
  policy-dry-run)
    run_policy_dry_run
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
