#!/usr/bin/env bash
# Launch the full teleop stack with one command:
#   rb_servo_server (pgmode real|sim) + viser GUI + policy_runner (spacemouse|umi)
#
# Usage:
#   tools/run_stack.sh [real|sim] [spacemouse|umi]
#   make run                    # real + spacemouse
#   make run MODE=sim SRC=umi
#
# Configs (the only two per layer — keep these the source of truth):
#   rb_servo_server/config/local/stack_real.yaml | stack_sim.yaml
#   policy_runner/config/stack_real.yaml         | stack_sim.yaml
#
# Notes:
#   - No sudo / RB_ALLOW_* envs needed (real/sim gates retired; rtprio via
#     limits.d). If RT setup fails, re-login or re-apply:
#       sudo setcap cap_sys_nice,cap_ipc_lock+ep rb_servo_server/build/rbpodo_real_gate/rb_servo_server
#   - Ctrl-C stops all three processes (lease is released by policy_runner).
set -euo pipefail
cd "$(dirname "$0")/.."

MODE="${1:-real}"
SRC="${2:-spacemouse}"

case "$MODE" in
  real|sim) ;;
  *) echo "usage: $0 [real|sim] [spacemouse|umi]" >&2; exit 2 ;;
esac
case "$SRC" in
  spacemouse) ACTION_SOURCE="dual_spacemouse_cartesian" ;;
  umi)        ACTION_SOURCE="umi_dual_cartesian" ;;
  *) echo "usage: $0 [real|sim] [spacemouse|umi]" >&2; exit 2 ;;
esac

SERVER_BIN="rb_servo_server/build/rbpodo_real_gate/rb_servo_server"
SERVER_CFG="rb_servo_server/config/local/stack_${MODE}.yaml"
POLICY_CFG="policy_runner/config/stack_${MODE}.yaml"
LOG_DIR="logs/stack"
mkdir -p "$LOG_DIR"

[ -x "$SERVER_BIN" ] || { echo "[stack] server binary missing: $SERVER_BIN (build rbpodo_real_gate first)" >&2; exit 1; }
[ -f "$SERVER_CFG" ] || { echo "[stack] missing $SERVER_CFG" >&2; exit 1; }
[ -f "$POLICY_CFG" ] || { echo "[stack] missing $POLICY_CFG" >&2; exit 1; }

if [ "$MODE" = "real" ]; then
  echo "============================================================"
  echo "[stack] PHYSICAL REAL MODE — THE ARMS WILL MOVE."
  echo "[stack] Pendant in real mode, workspace clear, E-stop manned."
  echo "============================================================"
fi

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  echo
  echo "[stack] stopping..."
  # Kill in reverse order (policy first so it releases the lease, then GUI, server).
  for ((i = ${#PIDS[@]} - 1; i >= 0; i--)); do
    kill "${PIDS[$i]}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  echo "[stack] stopped. logs in $LOG_DIR/"
}
trap cleanup EXIT INT TERM

echo "[stack] mode=$MODE source=$SRC"
echo "[stack] server: $SERVER_CFG"
"$SERVER_BIN" --config "$SERVER_CFG" >"$LOG_DIR/server.log" 2>&1 &
PIDS+=($!)

# Wait until the command server is up (or fail fast on config/RT errors).
# Generous deadline: an automatic pgmode switch (sim<->real) can take tens of
# seconds PER ARM on the controller before the server finishes initializing.
for i in $(seq 1 750); do
  if grep -q "CommandServer listening" "$LOG_DIR/server.log" 2>/dev/null; then break; fi
  if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
    echo "[stack] server exited during startup:" >&2
    tail -5 "$LOG_DIR/server.log" >&2
    exit 1
  fi
  if [ $((i % 25)) -eq 0 ]; then
    last_line=$(tail -1 "$LOG_DIR/server.log" 2>/dev/null)
    echo "[stack] waiting for server ($((i / 5))s)... ${last_line}"
  fi
  sleep 0.2
done
grep -q "CommandServer listening" "$LOG_DIR/server.log" || {
  echo "[stack] server did not come up in 150s:" >&2; tail -5 "$LOG_DIR/server.log" >&2; exit 1; }
echo "[stack] server up."

echo "[stack] viser GUI: http://127.0.0.1:8080"
PYTHONPATH=rb_gui \
  RB_GUI_DESCRIPTIONS_DIR="$PWD/rb_servo_server/descriptions" \
  RB_GUI_STATE_BIND=0.0.0.0 RB_GUI_STATE_PORT=50366 \
  RB_GUI_CIRCLE_OVERLAY_BIND=none \
  python3 -m rb_servo_gui.app >"$LOG_DIR/gui.log" 2>&1 &
PIDS+=($!)

VERBOSE_FLAG=""
if [ "${VERBOSE:-0}" = "1" ]; then VERBOSE_FLAG="--verbose"; fi
echo "[stack] policy_runner: $POLICY_CFG --action-source $ACTION_SOURCE $VERBOSE_FLAG"
echo "[stack] (VERBOSE=1 make run -> live input/loop stats; Ctrl-C stops everything)"
PYTHONPATH=policy_runner \
  python3 -u -m policy_runner --config "$POLICY_CFG" --action-source "$ACTION_SOURCE" $VERBOSE_FLAG \
  2>&1 | tee "$LOG_DIR/policy.log"
