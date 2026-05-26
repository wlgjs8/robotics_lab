#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ARTIFACT_DIR=""
STACK_MODE="start-local"
CHECK_DEPS=1
ALLOW_MISSING_PINOCCHIO=0
SKIP_ESTOP_RESET=0
SERVER="${ROOT_DIR}/rb_servo_server/build/pinocchio_gate/rb_servo_server"
SERVER_CONFIG="${ROOT_DIR}/rb_servo_server/config/dual_simulator_tcp_acceptance.yaml"
LEFT_CONFIG="${ROOT_DIR}/rb_simulator/config/left_rb3_730e.yaml"
RIGHT_CONFIG="${ROOT_DIR}/rb_simulator/config/right_rb3_730e.yaml"
RBSIM_COMMAND="${RBSIM_COMMAND:-python3 -m rbsim}"
COMMAND_HOST="127.0.0.1"
COMMAND_PORT="50010"
STATE_HOST="127.0.0.1"
STATE_PORT="50110"
STARTUP_TIMEOUT_SEC="8.0"
CAPTURE_SEC="1.0"
LINEAR_DURATION_SEC="1.0"
POSITION_TOLERANCE_M="0.004"
ORIENTATION_TOLERANCE_RAD="0.005"
LINE_TOLERANCE_M="0.002"
REPEAT="${CODEX_CARTESIAN_ACCEPTANCE_REPEAT:-1}"
RECORD_RUN_LABEL="${CODEX_CARTESIAN_ACCEPTANCE_LABEL:-}"
SKIP_SLERP=0
REQUIRE_SLERP=0
RUN_PTP=0
RUN_LINEAR=0
RUN_TWIST_LOCAL=0
RUN_TWIST_STAND=0
RUN_NEAR_PI_PTP=0
RUN_ALL=0

usage() {
  cat <<'USAGE'
Usage: scripts/tcp_pose_simulator_acceptance.sh [options]

Simulator-only Cartesian acceptance for TcpPoseTarget, TcpLinearMove, and TcpTwist*.

Scenario selection:
  --run-ptp                  Run TcpPoseTarget final-pose acceptance.
  --run-linear               Run TcpLinearMove constant and slerp acceptance.
  --run-twist-local          Run TcpTwistLocal local +X orientation-hold acceptance.
  --run-twist-stand          Run TcpTwistStand stand +X orientation-hold acceptance.
  --run-near-pi-ptp          Run optional near-pi TcpPoseTarget orientation acceptance.
  --all                      Run all scenarios. Default when no scenario flag is supplied.

Runtime/options:
  --artifact-dir DIR         Artifact directory. Default: artifacts/cartesian_acceptance/<timestamp>
  --assume-running           Do not start local simulator/server processes; use configured host endpoints.
  --start-local              Start host-loopback simulators/server locally. Default.
  --skip-deps                Skip scripts/check_deps.sh --profile hardware-free.
  --allow-missing-pinocchio  Allow simulator-only config/tool preflight without runtime acceptance.
  --skip-estop-reset         Skip optional EmergencyStop/ResetFault check.
  --server PATH              rb_servo_server binary.
  --server-config PATH       FK/IK-enabled simulator server config.
  --left-config PATH         Left rb_simulator config.
  --right-config PATH        Right rb_simulator config.
  --rbsim-command COMMAND    Simulator launch command. Default: python3 -m rbsim
  --command-host HOST        UDP command host. Default: 127.0.0.1
  --command-port PORT        UDP command port. Default: 50010
  --state-host HOST          UDP state capture bind host. Default: 127.0.0.1
  --state-port PORT          UDP state capture bind port. Default: 50110
  --startup-timeout-sec SEC  Startup/state wait timeout. Default: 8.0
  --capture-sec SEC          Post-command capture duration. Default: 1.0
  --linear-duration-sec SEC  TcpLinearMove duration. Default: 1.0
  --orientation-tolerance-rad RAD
                             Quaternion angle tolerance. Default: 0.005
  --line-tolerance-m M       Linear path line-deviation tolerance. Default: 0.002
  --tcp-tolerance-m M        Alias for --position-tolerance-m. Default: 0.004
  --position-tolerance-m M   Final TCP position tolerance. Default: 0.004
  --repeat N                 Repeat selected scenarios N times. Default: ${CODEX_CARTESIAN_ACCEPTANCE_REPEAT:-1}
  --record-run-label LABEL   Add a label to summary/artifact files.
  --require-slerp            Fail if slerp linear acceptance is skipped.
  --skip-slerp               Run constant Linear acceptance without slerp.
  -h, --help                 Show this help.

This runner refuses real/rbpodo configs, cartesian_control.allow_in_real=true,
and endpoints containing the real robot IPs 172.28.60.200/172.28.60.201.
USAGE
}

fail() {
  echo "cartesian_acceptance: FAIL: $*" >&2
  exit 2
}

pinocchio_available() {
  "${ROOT_DIR}/scripts/check_deps.sh" --profile kinematics
}

require_file() {
  local path="$1"
  local label="$2"
  [[ -f "${path}" ]] || fail "missing ${label}: ${path}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --artifact-dir) [[ $# -ge 2 ]] || fail "--artifact-dir requires a value"; ARTIFACT_DIR="$2"; shift 2 ;;
    --assume-running) STACK_MODE="assume-running"; shift ;;
    --start-local) STACK_MODE="start-local"; shift ;;
    --skip-deps) CHECK_DEPS=0; shift ;;
    --allow-missing-pinocchio) ALLOW_MISSING_PINOCCHIO=1; shift ;;
    --skip-estop-reset) SKIP_ESTOP_RESET=1; shift ;;
    --server) [[ $# -ge 2 ]] || fail "--server requires a value"; SERVER="$2"; shift 2 ;;
    --server-config) [[ $# -ge 2 ]] || fail "--server-config requires a value"; SERVER_CONFIG="$2"; shift 2 ;;
    --left-config) [[ $# -ge 2 ]] || fail "--left-config requires a value"; LEFT_CONFIG="$2"; shift 2 ;;
    --right-config) [[ $# -ge 2 ]] || fail "--right-config requires a value"; RIGHT_CONFIG="$2"; shift 2 ;;
    --rbsim-command) [[ $# -ge 2 ]] || fail "--rbsim-command requires a value"; RBSIM_COMMAND="$2"; shift 2 ;;
    --command-host) [[ $# -ge 2 ]] || fail "--command-host requires a value"; COMMAND_HOST="$2"; shift 2 ;;
    --command-port) [[ $# -ge 2 ]] || fail "--command-port requires a value"; COMMAND_PORT="$2"; shift 2 ;;
    --state-host) [[ $# -ge 2 ]] || fail "--state-host requires a value"; STATE_HOST="$2"; shift 2 ;;
    --state-port) [[ $# -ge 2 ]] || fail "--state-port requires a value"; STATE_PORT="$2"; shift 2 ;;
    --startup-timeout-sec) [[ $# -ge 2 ]] || fail "--startup-timeout-sec requires a value"; STARTUP_TIMEOUT_SEC="$2"; shift 2 ;;
    --capture-sec) [[ $# -ge 2 ]] || fail "--capture-sec requires a value"; CAPTURE_SEC="$2"; shift 2 ;;
    --linear-duration-sec) [[ $# -ge 2 ]] || fail "--linear-duration-sec requires a value"; LINEAR_DURATION_SEC="$2"; shift 2 ;;
    --orientation-tolerance-rad|--tcp-orientation-tolerance-rad)
      [[ $# -ge 2 ]] || fail "$1 requires a value"; ORIENTATION_TOLERANCE_RAD="$2"; shift 2 ;;
    --line-tolerance-m) [[ $# -ge 2 ]] || fail "--line-tolerance-m requires a value"; LINE_TOLERANCE_M="$2"; shift 2 ;;
    --position-tolerance-m|--tcp-tolerance-m)
      [[ $# -ge 2 ]] || fail "$1 requires a value"; POSITION_TOLERANCE_M="$2"; shift 2 ;;
    --repeat) [[ $# -ge 2 ]] || fail "--repeat requires a value"; REPEAT="$2"; shift 2 ;;
    --record-run-label) [[ $# -ge 2 ]] || fail "--record-run-label requires a value"; RECORD_RUN_LABEL="$2"; shift 2 ;;
    --require-slerp) REQUIRE_SLERP=1; shift ;;
    --skip-slerp) SKIP_SLERP=1; shift ;;
    --run-ptp) RUN_PTP=1; shift ;;
    --run-linear) RUN_LINEAR=1; shift ;;
    --run-twist-local) RUN_TWIST_LOCAL=1; shift ;;
    --run-twist-stand) RUN_TWIST_STAND=1; shift ;;
    --run-near-pi-ptp) RUN_NEAR_PI_PTP=1; shift ;;
    --all) RUN_ALL=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "unknown argument: $1" ;;
  esac
done

if [[ "${RUN_ALL}" -eq 0 && "${RUN_PTP}" -eq 0 && "${RUN_LINEAR}" -eq 0 && "${RUN_TWIST_LOCAL}" -eq 0 && "${RUN_TWIST_STAND}" -eq 0 && "${RUN_NEAR_PI_PTP}" -eq 0 ]]; then
  RUN_ALL=1
fi

if [[ -z "${ARTIFACT_DIR}" ]]; then
  ARTIFACT_DIR="${ROOT_DIR}/artifacts/cartesian_acceptance/$(date -u +%Y%m%dT%H%M%SZ)"
fi

if [[ "${CHECK_DEPS}" -eq 1 ]]; then
  "${ROOT_DIR}/scripts/check_deps.sh" --profile hardware-free
fi

require_file "${ROOT_DIR}/scripts/cartesian_acceptance.py" "Cartesian acceptance helper"
require_file "${ROOT_DIR}/rb_servo_server/tools/send_tcp_pose_target.py" "TcpPoseTarget tool"
require_file "${ROOT_DIR}/rb_servo_server/tools/send_tcp_linear_move.py" "TcpLinearMove tool"
require_file "${ROOT_DIR}/rb_servo_server/tools/send_tcp_twist.py" "TcpTwist tool"
require_file "${SERVER_CONFIG}" "rb_servo_server simulator config"
require_file "${LEFT_CONFIG}" "left simulator config"
require_file "${RIGHT_CONFIG}" "right simulator config"

PY_ARGS=(
  --root "${ROOT_DIR}"
  --mode "${STACK_MODE}"
  --artifact-dir "${ARTIFACT_DIR}"
  --server "${SERVER}"
  --server-config "${SERVER_CONFIG}"
  --left-config "${LEFT_CONFIG}"
  --right-config "${RIGHT_CONFIG}"
  --rbsim-command "${RBSIM_COMMAND}"
  --command-host "${COMMAND_HOST}"
  --command-port "${COMMAND_PORT}"
  --state-host "${STATE_HOST}"
  --state-port "${STATE_PORT}"
  --startup-timeout-sec "${STARTUP_TIMEOUT_SEC}"
  --capture-sec "${CAPTURE_SEC}"
  --linear-duration-sec "${LINEAR_DURATION_SEC}"
  --position-tolerance-m "${POSITION_TOLERANCE_M}"
  --orientation-tolerance-rad "${ORIENTATION_TOLERANCE_RAD}"
  --line-tolerance-m "${LINE_TOLERANCE_M}"
  --repeat "${REPEAT}"
)

[[ -n "${RECORD_RUN_LABEL}" ]] && PY_ARGS+=(--record-run-label "${RECORD_RUN_LABEL}")
[[ "${REQUIRE_SLERP}" -eq 1 ]] && PY_ARGS+=(--require-slerp)
[[ "${SKIP_SLERP}" -eq 1 ]] && PY_ARGS+=(--skip-slerp)

[[ "${RUN_ALL}" -eq 1 ]] && PY_ARGS+=(--all)
[[ "${RUN_PTP}" -eq 1 ]] && PY_ARGS+=(--run-ptp)
[[ "${RUN_LINEAR}" -eq 1 ]] && PY_ARGS+=(--run-linear)
[[ "${RUN_TWIST_LOCAL}" -eq 1 ]] && PY_ARGS+=(--run-twist-local)
[[ "${RUN_TWIST_STAND}" -eq 1 ]] && PY_ARGS+=(--run-twist-stand)
[[ "${RUN_NEAR_PI_PTP}" -eq 1 ]] && PY_ARGS+=(--run-near-pi-ptp)
[[ "${SKIP_ESTOP_RESET}" -eq 1 ]] && PY_ARGS+=(--skip-estop-reset)

PYTHONPATH="${ROOT_DIR}/rb_simulator/src${PYTHONPATH:+:${PYTHONPATH}}" \
python3 "${ROOT_DIR}/scripts/cartesian_acceptance.py" "${PY_ARGS[@]}" --preflight-only >/dev/null

if ! pinocchio_available; then
  if [[ "${ALLOW_MISSING_PINOCCHIO}" -eq 1 ]]; then
    echo "cartesian_acceptance: Pinocchio unavailable; preflight passed and runtime acceptance skipped because --allow-missing-pinocchio was provided." >&2
    exit 0
  fi
  fail "Pinocchio is required for simulator Cartesian acceptance. Install the pinocchio CMake package or pass --allow-missing-pinocchio for preflight-only validation."
fi

if [[ "${STACK_MODE}" == "start-local" ]]; then
  [[ -x "${SERVER}" ]] || fail "rb_servo_server binary is not executable: ${SERVER}"
  server_cache="$(dirname "${SERVER}")/CMakeCache.txt"
  if [[ -f "${server_cache}" ]] && ! grep -q '^RB_SERVO_ENABLE_PINOCCHIO:BOOL=ON$' "${server_cache}"; then
    fail "rb_servo_server binary was not built with RB_SERVO_ENABLE_PINOCCHIO=ON: ${SERVER}"
  fi
fi

MATH_TEST="$(dirname "${SERVER}")/test_se3_math"
[[ -x "${MATH_TEST}" ]] || fail "near-pi Cartesian math test binary is missing: ${MATH_TEST}"
"${MATH_TEST}"
PY_ARGS+=(--near-pi-math-tests-run)

PYTHONPATH="${ROOT_DIR}/rb_simulator/src${PYTHONPATH:+:${PYTHONPATH}}" \
python3 "${ROOT_DIR}/scripts/cartesian_acceptance.py" "${PY_ARGS[@]}"
