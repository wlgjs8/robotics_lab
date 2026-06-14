#!/usr/bin/env bash
set -euo pipefail

LEFT_IP="${ROBOT_LEFT_IP:-192.168.56.101}"
RIGHT_IP="${ROBOT_RIGHT_IP:-192.168.56.102}"
MODE="closed"
EXEC_MODE=0
SINGLE=0
COMMAND=()

usage() {
  cat <<'EOF'
Usage: tools/vm/home_vm_env.sh [options] [--exec -- command ...]

Print or run with the environment for Rainbow VM controller-simulation parity.
Default mode is fail-closed: it sets only ROBOT_LEFT_IP/ROBOT_RIGHT_IP in
two-arm mode and opens no real/controller-simulation gates.

Options:
  --left-ip IP       Left VM IP, default $ROBOT_LEFT_IP or 192.168.56.101.
  --right-ip IP      Right VM IP, default $ROBOT_RIGHT_IP or 192.168.56.102.
  --single           Single-arm left-only mode; use/export only ROBOT_LEFT_IP.
  --readonly         Open read-only rbpodo connection gate only.
  --motion           Open controller-simulation Servo J motion gates.
  --cartesian        Open controller-simulation Cartesian gates.
  --exec -- CMD ...  Execute CMD with the selected environment instead of printing exports.
  -h, --help         Show this help.
EOF
}

fail() {
  echo "home_vm_env: ERROR: $*" >&2
  exit 2
}

while (($# > 0)); do
  case "$1" in
    --left-ip)
      [[ $# -ge 2 ]] || fail "--left-ip requires a value"
      LEFT_IP="$2"
      shift 2
      ;;
    --right-ip)
      [[ $# -ge 2 ]] || fail "--right-ip requires a value"
      RIGHT_IP="$2"
      shift 2
      ;;
    --single)
      SINGLE=1
      shift
      ;;
    --readonly)
      MODE="readonly"
      shift
      ;;
    --motion)
      MODE="motion"
      shift
      ;;
    --cartesian)
      MODE="cartesian"
      shift
      ;;
    --exec)
      EXEC_MODE=1
      shift
      [[ "${1:-}" == "--" ]] && shift
      COMMAND=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown argument: $1"
      ;;
  esac
done

[[ -n "${LEFT_IP}" ]] || fail "left IP is empty"
if [[ "${SINGLE}" == "0" ]]; then
  [[ -n "${RIGHT_IP}" ]] || fail "right IP is empty"
  [[ "${LEFT_IP}" != "${RIGHT_IP}" ]] || fail "left and right VM IPs must differ"
fi
if [[ "${EXEC_MODE}" == "1" && "${#COMMAND[@]}" -eq 0 ]]; then
  fail "--exec requires a command"
fi

apply_env() {
  export ROBOT_LEFT_IP="${LEFT_IP}"
  if [[ "${SINGLE}" == "1" ]]; then
    unset ROBOT_RIGHT_IP
  else
    export ROBOT_RIGHT_IP="${RIGHT_IP}"
  fi

  case "${MODE}" in
    closed)
      :
      ;;
    readonly)
      :
      ;;
    motion)
      :
      ;;
    cartesian)
      :
      ;;
    *)
      fail "unknown mode: ${MODE}"
      ;;
  esac
}

print_exports() {
  printf 'export ROBOT_LEFT_IP=%q\n' "${LEFT_IP}"
  if [[ "${SINGLE}" == "0" ]]; then
    printf 'export ROBOT_RIGHT_IP=%q\n' "${RIGHT_IP}"
  fi
  case "${MODE}" in
    closed)
      :
      ;;
    readonly)
      :
      ;;
    motion)
      :
      ;;
    cartesian)
      :
      ;;
  esac
}

if [[ "${EXEC_MODE}" == "1" ]]; then
  apply_env
  exec "${COMMAND[@]}"
fi

print_exports
