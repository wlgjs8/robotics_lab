#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE=""
FORCE=0

usage() {
  cat <<'EOF'
Usage: tools/rbpodo_circle_gui.sh --profile stable|gene [--force]

Launch rb_gui for rbpodo controller-simulation circle live visualization.

Profiles:
  stable  state port 50161, overlay port 50261
  gene    state port 50162, overlay port 50261

Options:
  --force     Skip local UDP bind-port conflict checks.
  -h, --help  Show this help.

This script only launches the GUI. It does not send robot commands.
EOF
}

fail() {
  echo "rbpodo_circle_gui: ERROR: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --profile)
      [[ $# -ge 2 ]] || fail "--profile requires stable or gene"
      PROFILE="$2"
      shift 2
      ;;
    --force)
      FORCE=1
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
    STATE_PORT=50161
    ;;
  gene)
    STATE_PORT=50162
    ;;
  "")
    fail "--profile stable|gene is required"
    ;;
  *)
    fail "unknown profile: ${PROFILE}"
    ;;
esac

OVERLAY_PORT=50261

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
  echo "rbpodo_circle_gui: warning: ss/lsof unavailable; skipping UDP port check for ${port}" >&2
  return 1
}

if [[ "${FORCE}" != "1" ]]; then
  port_in_use "${STATE_PORT}" && fail "UDP state port ${STATE_PORT} is already in use; pass --force to skip this check"
  port_in_use "${OVERLAY_PORT}" && fail "UDP overlay port ${OVERLAY_PORT} is already in use; pass --force to skip this check"
fi

export PYTHONPATH="${ROOT_DIR}/rb_gui${PYTHONPATH:+:${PYTHONPATH}}"
export RB_GUI_STATE_BIND="0.0.0.0"
export RB_GUI_STATE_PORT="${STATE_PORT}"
export RB_GUI_CIRCLE_OVERLAY_BIND="udp://0.0.0.0:${OVERLAY_PORT}"

cd "${ROOT_DIR}"
exec python3 -m rb_servo_gui.app
