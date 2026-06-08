#!/usr/bin/env bash
# Bring up the rbpodo pgmode-simulation stack natively on this box:
# rb_servo_server (under sudo for realtime) + rb_gui (viser).
# Controller-simulation only; physical_motion_expected=false. The server config
# keeps cartesian_control.allow_in_real=false and this never sets
# RB_ALLOW_REAL_CARTESIAN. Reaches the VirtualBox host-only controller VMs
# directly (default WSL2 NAT routes to the host-only subnet).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

ENV_FILE="${PGMODE_SIM_ENV:-rb_servo_server/config/local/pgmode_sim.env}"
if [[ ! -f "$ENV_FILE" ]]; then
    echo "pgmode_sim_up: missing $ENV_FILE" >&2
    echo "  copy the template: cp rb_servo_server/config/local/pgmode_sim.env.example $ENV_FILE" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "$ENV_FILE"

BIN="${PGMODE_SIM_BIN:-rb_servo_server/build_rbpodo/rb_servo_server}"
PY="${PGMODE_SIM_PYTHON:-.venv/bin/python}"
CFG="${PGMODE_SIM_CONFIG:?PGMODE_SIM_CONFIG must be set in $ENV_FILE}"
[[ -x "$BIN" ]] || { echo "pgmode_sim_up: missing $BIN — run: make pgmode-sim-build" >&2; exit 1; }
[[ -x "$PY" ]]  || { echo "pgmode_sim_up: missing python $PY (the repo venv)" >&2; exit 1; }
[[ -f "$CFG" ]] || { echo "pgmode_sim_up: missing server config $CFG" >&2; exit 1; }

# Launch logs must be user-writable. The server runs as root and writes its CSV
# log into ./logs, so logs/ is often root-owned; make it writable (or fall back).
LOGDIR="logs"
mkdir -p "$LOGDIR" 2>/dev/null || true
[[ -w "$LOGDIR" ]] || sudo chown "$(id -un):$(id -gn)" "$LOGDIR" 2>/dev/null || true
[[ -w "$LOGDIR" ]] || { LOGDIR="/tmp/pgmode_sim"; mkdir -p "$LOGDIR"; }
SERVER_LOG="$LOGDIR/pgmode_sim_server.log"
GUI_LOG="$LOGDIR/pgmode_sim_gui.log"

wait_for() { # file pattern label
    local f="$1" pat="$2" lbl="$3" i
    for i in $(seq 1 40); do
        grep -q "$pat" "$f" 2>/dev/null && return 0
        grep -qi "FATAL" "$f" 2>/dev/null && { echo "pgmode_sim_up: $lbl FATAL:" >&2; grep -i FATAL "$f" | tail -3 >&2; return 1; }
        sleep 1
    done
    echo "pgmode_sim_up: timed out waiting for $lbl" >&2; return 1
}

# --- rb_servo_server (sudo: real mode requires realtime priority on WSL2) ---
if pgrep -f 'rb_servo[_]server --config' >/dev/null 2>&1; then
    echo "pgmode_sim_up: rb_servo_server already running"
else
    echo "pgmode_sim_up: starting rb_servo_server (sudo) with $CFG"
    # C-phase-1: the controller-simulation carve-out is config-derived now
    # (operation_mode=simulation + servo.allow_controller_simulation_* flags).
    # Only the real-connection tripwire env remains (retired in C-phase-2).
    nohup sudo env \
        RB_ALLOW_REAL_ROBOT="${RB_ALLOW_REAL_ROBOT:-1}" \
        RB_ALLOW_REAL_MOTION="${RB_ALLOW_REAL_MOTION:-1}" \
        "$BIN" --config "$CFG" > "$SERVER_LOG" 2>&1 &
    wait_for "$SERVER_LOG" "CommandServer listening" "rb_servo_server"
fi

# --- rb_gui (viser) — inherits RB_GUI_* exported by the env file ---
if pgrep -f '[r]b_servo_gui.app' >/dev/null 2>&1; then
    echo "pgmode_sim_up: rb_gui already running"
else
    echo "pgmode_sim_up: starting rb_gui (viser)"
    nohup "$PY" -m rb_servo_gui.app > "$GUI_LOG" 2>&1 &
    wait_for "$GUI_LOG" "listening on http" "rb_gui"
fi

echo "pgmode_sim_up: ready -> http://localhost:${RB_GUI_PORT:-8080}"
echo "  server log: $SERVER_LOG   gui log: $GUI_LOG"
echo "  stop with: make pgmode-sim-down"
