#!/usr/bin/env bash
# Standalone rb_servo_server launcher (NO teleop_mux) for policy/replay sessions,
# with per-tick diagnostics auto-captured under logs/ — no env vars to type.
#
# Why this exists: isolated server-only diagnostics/replay. Live flow-infer no
# longer needs this path: `make run` keeps teleop_mux alive on state port 50376,
# and tools/flow_infer_real_policy.sh uses the external flow-infer state port
# 50378. For ground-truth replay or deliberately isolated debugging, this
# launcher still keeps the policy_runner command source out of the stack.
#
# On every launch this auto-sets (so you never type them on the command line):
#   RB_TWIST_PIPELINE_CSV = logs/twist_pipe_<ts>.csv   (per-tick model->applied
#       twist + qdot + IK conditioning: ik_sigma_min / ik_lambda / ik_jump_deg /
#       ik_branch_jump — the file to read when an arm freezes: singularity =
#       sigma_min small + qdot ~0 despite a commanded twist)
#   server stdout/stderr -> logs/server_<ts>.log
# Both are timestamped, so successive runs ACCUMULATE (nothing is overwritten).
# The companion action log is now opt-in for low-jitter live motion: set
# POLICY_RUNNER_ACTION_LOG=auto (or a path) on the flow-infer process when needed.
#
# Usage:
#   tools/run_servo.sh [real|sim]      # default: real
# Ctrl-C stops the server (and the CSV/log are flushed).
set -euo pipefail
cd "$(dirname "$0")/.."

MODE="${1:-real}"
case "$MODE" in
  real|sim) ;;
  *) echo "usage: $0 [real|sim]" >&2; exit 2 ;;
esac

SERVER_BIN="rb_servo_server/build/rbpodo_real_gate/rb_servo_server"
SERVER_CFG="rb_servo_server/config/local/stack_${MODE}.yaml"
LOG_DIR="logs"
TS="$(date +%Y%m%d_%H%M%S)"
SERVER_LOG="$LOG_DIR/server_${TS}.log"
TWIST_CSV="$LOG_DIR/twist_pipe_${TS}.csv"

[ -x "$SERVER_BIN" ] || { echo "[servo] server binary missing: $SERVER_BIN (build rbpodo_real_gate first)" >&2; exit 1; }
[ -f "$SERVER_CFG" ] || { echo "[servo] missing $SERVER_CFG" >&2; exit 1; }
mkdir -p "$LOG_DIR"

# Kill a stale standalone server (by executable name only, so we never match this
# script or a shell) to avoid "bind: Address already in use".
for pid in $(pgrep -f "$SERVER_BIN" 2>/dev/null || true); do
  [ "$(cat "/proc/$pid/comm" 2>/dev/null)" = "rb_servo_server" ] || continue
  echo "[servo] killing stale server pid=$pid"
  kill "$pid" 2>/dev/null || true
done
sleep 0.5

# Best-effort RT caps (cap_sys_nice + mlock); harmless to skip in sim.
RT_CAPS="cap_sys_nice,cap_ipc_lock+ep"
if command -v getcap >/dev/null 2>&1 && ! getcap "$SERVER_BIN" 2>/dev/null | grep -q cap_sys_nice; then
  sudo -n setcap "$RT_CAPS" "$SERVER_BIN" 2>/dev/null \
    && echo "[servo] setcap applied" \
    || echo "[servo] WARN: no RT caps (server runs without RT priority; fine for sim)" >&2
fi

echo "[servo] mode=$MODE config=$SERVER_CFG"
echo "[servo] twist CSV  -> $TWIST_CSV"
echo "[servo] server log -> $SERVER_LOG"
echo "[servo] (Ctrl-C to stop)"
env RB_TWIST_PIPELINE_CSV="$TWIST_CSV" \
  "$SERVER_BIN" --config "$SERVER_CFG" 2>&1 | tee "$SERVER_LOG"
