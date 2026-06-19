#!/usr/bin/env bash
# Run rb_servo_server AND open PlotJuggler showing TCP x,y,z + rx,ry,rz live.
#   ./scripts/run_server_plotjuggler.sh [server-config.yaml]
# Ctrl-C stops everything. If you already have a server running, instead run
# only the bridge + PlotJuggler (see the two background commands below).
set -uo pipefail
cd "$(cd "$(dirname "$0")/.." && pwd)"

SERVER_BIN="rb_servo_server/build/rbpodo_real_gate/rb_servo_server"
CFG="${1:-rb_servo_server/config/local/stack_real.yaml}"
LOG="/tmp/rb_servo_server.log"
PIDS=()
cleanup() { for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

# 1) server (console + log file for the readiness wait)
stdbuf -oL "$SERVER_BIN" --config "$CFG" 2>&1 | tee "$LOG" &
PIDS+=($!)
echo "[run] waiting for server to listen..."
for _ in $(seq 1 200); do
  grep -q "CommandServer listening" "$LOG" 2>/dev/null && { echo "[run] server up."; break; }
  sleep 0.2
done

# 2) pose bridge: state fanout 50356 -> PlotJuggler UDP-JSON 9870 (x,y,z,rx,ry,rz)
python3 scripts/tcp_pose_plotjuggler_bridge.py --in-port 50356 --out-port 9870 &
PIDS+=($!)

# 3) PlotJuggler
plotjuggler -n >/tmp/plotjuggler.log 2>&1 &
PIDS+=($!)

cat <<'EOF'
[run] PlotJuggler opened. ONE-TIME setup (it is remembered next launch):
  1. Left panel "Streaming" -> select "UDP Server" -> click Start.
  2. Dialog: port = 9870, message protocol = JSON -> OK.
  3. Drag right/x, right/y, right/z (and right/rx, right/ry, right/rz) onto plots.
     (left/* available too.) Use "t" as the X axis if prompted, else arrival time.
Ctrl-C here stops server + bridge + PlotJuggler.
EOF
wait "${PIDS[0]}"
