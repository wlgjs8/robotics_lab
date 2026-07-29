#!/usr/bin/env bash
# Launch the full teleop stack with one command:
#   rb_servo_server (pgmode real|sim) + viser GUI + policy_runner (teleop mux)
#   + joint scope dashboard on :8081 (per-arm q/velocity/acceleration/jerk)
#   + gripper_server: receives the arbitrated command stream on UDP 50410 and
#     publishes feedback on UDP 50420. Toggle with GRIPPER_SERVER=0|1; legacy
#     GRIPPER_FOLLOW is accepted only as a deprecated alias.
#
# Both teleop sources (SpaceMouse + UMI) run side by side: the first to engage
# owns the robot until it returns to idle (policy_runner action_source
# teleop_mux). A missing SpaceMouse degrades to UMI-only.
#
# Usage:
#   tools/run_stack.sh [real|sim]
#   make run                    # real
#   make run MODE=sim
#
# Configs (the only two per layer — keep these the source of truth):
#   rb_servo_server/config/stack_real.yaml       | stack_sim.yaml
#   policy_runner/config/stack_real.yaml         | stack_sim.yaml
#
# Notes:
#   - No sudo / RB_ALLOW_* envs needed (real/sim gates retired; rtprio via
#     limits.d). If RT setup fails, re-login or re-apply:
#       sudo setcap cap_sys_nice,cap_ipc_lock+ep rb_servo_server/build/rbpodo_real_gate/rb_servo_server
#   - Ctrl-C stops all three processes (lease is released by policy_runner).
#   - State fanout ports: 50356 scope dashboard, 50366 viser GUI,
#     50376 stack policy_runner/teleop_mux, 50378 external flow-infer.
set -euo pipefail
cd "$(dirname "$0")/.."

MODE="${1:-real}"

# Match tools/build_stack.sh so the interpreter receiving the editable installs
# is also the one that launches viser, policy_runner, and helper processes.
# Without this, `make build` can install into .venv while `make run` uses the
# system python3 and the GUI immediately dies on imports such as numpy/viser.
if [ -n "${STACK_PYTHON:-}" ]; then PY="$STACK_PYTHON"
elif [ -x ".venv/bin/python" ]; then PY=".venv/bin/python"
else PY="python3"; fi

case "$MODE" in
  real|sim) ;;
  *) echo "usage: $0 [real|sim]" >&2; exit 2 ;;
esac
# Single teleop entrypoint: SpaceMouse + UMI side by side (idle handoff).
# External flow-infer should run alongside this stack via
# tools/flow_infer_real_policy.sh / `make flow-infer-real`; no
# ACTION_SOURCE=none hand-edit is needed.
# 한 소스만 격리 디버그하려면 env 로 오버라이드:
#   ACTION_SOURCE=umi_dual_cartesian make run   (UMI 단독, SpaceMouse/mux 제외)
#   ACTION_SOURCE=dual_spacemouse_pose_target make run
#   ACTION_SOURCE=none make run                 (legacy replay/isolation only:
#                                                policy_runner 미기동)
ACTION_SOURCE="${ACTION_SOURCE:-teleop_mux}"

SERVER_BIN="rb_servo_server/build/rbpodo_real_gate/rb_servo_server"
SERVER_CFG="${SERVER_CONFIG:-rb_servo_server/config/stack_${MODE}.yaml}"
POLICY_CFG="policy_runner/config/stack_${MODE}.yaml"
LOG_DIR="logs/stack"
mkdir -p "$LOG_DIR"

# One folder per `make run` for recorded episodes: data_<KST timestamp>, fixed
# here at launch time so every episode from this session lands under the same
# folder. policy_runner writes episode_000.hdf5, episode_001.hdf5, ... inside it
# (under recording.output_dir). Override by exporting RB_RECORD_SESSION_DIR.
export RB_RECORD_SESSION_DIR="${RB_RECORD_SESSION_DIR:-data_$(TZ='Asia/Seoul' date +%Y%m%d_%H%M%S)}"

# Gripper server (docs/plans/gripper_server_design.md): single owner of the Pika
# grippers. rb_servo_server forwards the arbitrated per-arm gripper setpoint
# (left/right.gripper in the command packet) to it as gripper_cmd.v1 (:50410) and
# stamps its gripper_state.v1 feedback (:50420) into the published state JSON,
# which drives the viser gripper viz. This replaces the old direct serial bridge
# serial bridge: gripper now rides the same command stream + lease as arm motion.
# Backend: 'sim' (hardware-free; feedback eases toward the command, so the viz
# moves even without hardware) for mock/sim; 'pika' (local serial) for real.
# Toggle with GRIPPER_SERVER=auto|0|1 (GRIPPER_FOLLOW kept as a deprecated alias);
# default on in every mode. Endpoints match the server config's gripper block.
if [ "$MODE" = "real" ]; then
  GRIPPER_BACKEND="pika"
  # The AgileX Pika SDK ('pika' package) is NOT on this PC's default sys.path;
  # the copy from the SteamVR PC's conda env lives at PIKA_SDK_PATH (the same
  # convention/default used on this workstation). Without it the pika backend
  # dies at startup with "No module named 'pika'" and the gripper viz never
  # updates. Pass it through so `make run` alone works.
  PIKA_SDK_PATH="${PIKA_SDK_PATH:-/home/plaif/workspace/pika_sdk}"
  # --no-home-on-connect: do NOT actuate the grippers at startup. Homing drives
  # each jaw to its closed stop to re-zero, but the two Pika motors have opposite
  # power-on directions, so that startup motion lands asymmetrically (observed:
  # right opens, left closes). Hold each gripper at its current power-on position
  # instead. Re-enable with GRIPPER_SERVER_ARGS=... --home-on-connect.
  # The tracked D405 serial mapping + live camera.health physical_port are the
  # arm identity source. The gripper_server joins that identity to the CH340 on
  # the same xHCI root port and fails closed instead of trusting tty numbering or
  # the legacy fixed /dev/pika-* links. camera_server must already be running.
  GRIPPER_SERVER_ARGS="${GRIPPER_SERVER_ARGS---auto-pair-camera-config camera_server/config/dual_realsense_d405.yaml --camera-health-endpoint tcp://127.0.0.1:5600 --camera-health-topic camera.health --pairing-timeout-sec 5 --pika-sdk-path $PIKA_SDK_PATH --no-home-on-connect}"
else
  GRIPPER_BACKEND="sim"
  GRIPPER_SERVER_ARGS="${GRIPPER_SERVER_ARGS-}"
fi
case "${GRIPPER_SERVER:-${GRIPPER_FOLLOW:-auto}}" in
  0|no|off|false) GRIPPER_SERVER_ON=0 ;;
  *) GRIPPER_SERVER_ON=1 ;;
esac
case "${SCOPE_DASHBOARD:-1}" in
  0|no|off|false) SCOPE_DASHBOARD_ON=0 ;;
  *) SCOPE_DASHBOARD_ON=1 ;;
esac
SCOPE_DASHBOARD_PORT="${SCOPE_DASHBOARD_PORT:-8081}"
SCOPE_DASHBOARD_HOST="${SCOPE_DASHBOARD_HOST:-0.0.0.0}"
SCOPE_DASHBOARD_STATE_LISTEN="${SCOPE_DASHBOARD_STATE_LISTEN:-0.0.0.0:50356}"
SCOPE_DASHBOARD_CSV="${SCOPE_DASHBOARD_CSV:-$LOG_DIR/joint_kinematics.csv}"
GRIPPER_SERVER_DEBUG_ARGS=""
case "${GRIPPER_SERVER_DEBUG:-0}" in
  1|yes|on|true) GRIPPER_SERVER_DEBUG_ARGS="--debug-stats" ;;
esac

# Ensure the RT capabilities the servo loop needs (real-time scheduling +
# mlockall) are on the server binary. setcap requires root, and a rebuild
# strips the file caps, so we (re)apply them here. Order of preference:
#   1) already present              -> no-op
#   2) passwordless sudo (NOPASSWD) -> sudo -n
#   3) local ./.env SUDO_PASSWORD   -> piped to `sudo -S` over stdin (not argv,
#                                      so it never shows up in `ps`). .env is
#                                      chmod 600 and gitignored.
# If none work we WARN and continue: the server still runs, just without RT
# priority / locked memory (fine for sim, sub-optimal for real).
RT_CAPS="cap_sys_nice,cap_ipc_lock+ep"
ensure_rt_caps() {
  command -v getcap >/dev/null 2>&1 || return 0
  if getcap "$SERVER_BIN" 2>/dev/null | grep -q "cap_sys_nice"; then
    return 0
  fi
  echo "[stack] RT caps missing on $SERVER_BIN; applying setcap..."
  if sudo -n setcap "$RT_CAPS" "$SERVER_BIN" 2>/dev/null; then
    echo "[stack] setcap applied (passwordless sudo)"
    return 0
  fi
  local env_file="$PWD/.env" pw=""
  if [ -f "$env_file" ]; then
    pw=$(grep -E '^SUDO_PASSWORD=' "$env_file" | head -1 | cut -d= -f2-)
  fi
  if [ -n "$pw" ] && printf '%s\n' "$pw" | sudo -S -p '' setcap "$RT_CAPS" "$SERVER_BIN" 2>/dev/null; then
    echo "[stack] setcap applied (./.env password)"
    unset pw
    return 0
  fi
  unset pw
  echo "[stack] WARN: could not apply RT caps; server runs without RT priority/mlock." >&2
  echo "[stack]       fix: sudo setcap $RT_CAPS $SERVER_BIN  (or set SUDO_PASSWORD in ./.env)" >&2
}

[ -x "$SERVER_BIN" ] || { echo "[stack] server binary missing: $SERVER_BIN (build rbpodo_real_gate first)" >&2; exit 1; }
[ -f "$SERVER_CFG" ] || { echo "[stack] missing $SERVER_CFG" >&2; exit 1; }
[ -f "$POLICY_CFG" ] || { echo "[stack] missing $POLICY_CFG" >&2; exit 1; }
case "$SERVER_CFG" in
  /*) SERVER_CFG_ABS="$SERVER_CFG" ;;
  *) SERVER_CFG_ABS="$PWD/$SERVER_CFG" ;;
esac

# Preflight: a previous run interrupted mid-startup (e.g. Ctrl-C during a slow
# pgmode switch) can leave OUR processes alive holding the command/state ports
# -> "bind() failed: Address already in use". Kill stale instances of this
# stack's own components (matched by our binary/module paths only) first.
preflight_kill_stale() {
  local label="$1"; shift
  local pids
  pids=$(pgrep -f "$1" 2>/dev/null | grep -vw "$$" || true)
  [ -n "$pids" ] || return 0
  echo "[stack] killing stale $label from a previous run (pid: $(echo $pids | tr '\n' ' '))"
  kill $pids 2>/dev/null || true
  for _ in $(seq 1 50); do
    pgrep -f "$1" >/dev/null 2>&1 || return 0
    sleep 0.1
  done
  echo "[stack] stale $label ignored SIGTERM; escalating to SIGKILL" >&2
  pkill -9 -f "$1" 2>/dev/null || true
}
preflight_kill_stale "rb_servo_server" "$SERVER_BIN"
preflight_kill_stale "viser GUI"       "rb_servo_gui.app"
preflight_kill_stale "scope dashboard" "scripts/servo_scope_dashboard.py"
preflight_kill_stale "policy_runner"   "policy_runner --config policy_runner/config/stack_"
[ "$GRIPPER_SERVER_ON" = "1" ] && preflight_kill_stale "gripper_server" "policy_runner.gripper_server"

if [ "$MODE" = "real" ]; then
  echo "============================================================"
  echo "[stack] PHYSICAL REAL MODE — THE ARMS WILL MOVE."
  echo "[stack] Pendant in real mode, workspace clear, E-stop manned."
  echo "============================================================"
fi

PIDS=()
cleanup() {
  # Ignore further Ctrl-C: a second ^C must not interrupt the teardown and
  # leave a process holding the command port (-> bind Address already in use).
  trap '' INT TERM
  trap - EXIT
  echo
  echo "[stack] stopping..."
  # Kill in reverse order (policy first so it releases the lease, then GUI,
  # server, and finally the gripper process that was started first).
  for ((i = ${#PIDS[@]} - 1; i >= 0; i--)); do
    kill "${PIDS[$i]}" 2>/dev/null || true
  done
  # Bounded wait, then SIGKILL stragglers (the server now aborts its slow
  # pgmode/activation polls on SIGTERM, so this normally returns quickly).
  local deadline=$((SECONDS + 10))
  for pid in "${PIDS[@]}"; do
    while kill -0 "$pid" 2>/dev/null; do
      if [ "$SECONDS" -ge "$deadline" ]; then
        echo "[stack] pid $pid did not exit in 10s; SIGKILL" >&2
        kill -9 "$pid" 2>/dev/null || true
        break
      fi
      sleep 0.1
    done
  done
  wait 2>/dev/null || true
  echo "[stack] stopped. logs in $LOG_DIR/"
}
# Ctrl-C is the NORMAL way to stop the stack: clean up and exit 0 so make
# does not report "오류 130" (130 = 128+SIGINT). Error paths keep their
# nonzero exit codes via the plain EXIT trap.
trap 'cleanup; exit 0' INT TERM
trap cleanup EXIT

echo "[stack] mode=$MODE source=$ACTION_SOURCE (spacemouse + umi side by side)"
echo "[stack] server: $SERVER_CFG"
echo "[stack] python: $PY"
echo "[stack] external flow-infer state readback: udp://127.0.0.1:50378"
if [ "$SCOPE_DASHBOARD_ON" = "1" ]; then
  echo "[stack] joint scope dashboard state listen: ${SCOPE_DASHBOARD_STATE_LISTEN}"
fi

# Start the gripper owner before the arm server. In real mode this is also the
# camera-serial/USB-topology preflight: if camera_server is absent, the identity
# is ambiguous, or the Pika backend cannot connect, make run must fail before an
# arm command endpoint comes up. --no-home-on-connect keeps this startup check
# motionless. MODE=sim uses the hardware-free backend and has no camera dependency.
if [ "$GRIPPER_SERVER_ON" = "1" ]; then
  echo "[stack] gripper_server: backend=$GRIPPER_BACKEND cmd<-127.0.0.1:50410 feedback->127.0.0.1:50420 ${GRIPPER_SERVER_ARGS:-} ${GRIPPER_SERVER_DEBUG_ARGS:-}"
  "$PY" -u -m policy_runner.gripper_server \
    --backend "$GRIPPER_BACKEND" --bind 127.0.0.1:50410 --state-endpoint 127.0.0.1:50420 \
    ${GRIPPER_SERVER_DEBUG_ARGS:-} ${GRIPPER_SERVER_ARGS:-} >"$LOG_DIR/gripper_server.log" 2>&1 &
  GRIPPER_PID=$!
  PIDS+=("$GRIPPER_PID")
  for _ in $(seq 1 100); do
    if grep -q "gripper_server up:" "$LOG_DIR/gripper_server.log" 2>/dev/null; then break; fi
    if ! kill -0 "$GRIPPER_PID" 2>/dev/null; then
      echo "[stack] gripper_server exited during startup:" >&2
      tail -10 "$LOG_DIR/gripper_server.log" >&2
      exit 1
    fi
    sleep 0.1
  done
  grep -q "gripper_server up:" "$LOG_DIR/gripper_server.log" || {
    echo "[stack] gripper_server did not become ready in 10s:" >&2
    tail -10 "$LOG_DIR/gripper_server.log" >&2
    exit 1
  }
  echo "[stack] gripper_server up."
fi

ensure_rt_caps
"$SERVER_BIN" --config "$SERVER_CFG" >"$LOG_DIR/server.log" 2>&1 &
SERVER_PID=$!
PIDS+=("$SERVER_PID")

# Wait until the command server is up (or fail fast on config/RT errors).
# Generous deadline: an automatic pgmode switch (sim<->real) can take tens of
# seconds PER ARM on the controller before the server finishes initializing.
for i in $(seq 1 750); do
  if grep -q "CommandServer listening" "$LOG_DIR/server.log" 2>/dev/null; then break; fi
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
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

GUI_PORT="${RB_GUI_PORT:-8080}"
echo "[stack] viser GUI: http://127.0.0.1:${GUI_PORT}"
if [ "$SCOPE_DASHBOARD_ON" = "1" ]; then
  echo "[stack] joint scope dashboard: http://127.0.0.1:${SCOPE_DASHBOARD_PORT}"
fi
PYTHONPATH=rb_gui \
  RB_GUI_DESCRIPTIONS_DIR="$PWD/rb_servo_server/descriptions" \
  RB_GUI_STATE_BIND=0.0.0.0 RB_GUI_STATE_PORT=50366 \
  RB_GUI_COMMAND_HOST=127.0.0.1 RB_GUI_COMMAND_PORT=50256 \
  RB_GUI_CIRCLE_OVERLAY_BIND=none \
  RB_GUI_SERVER_CONFIG_PATH="$SERVER_CFG_ABS" \
  "$PY" -m rb_servo_gui.app >"$LOG_DIR/gui.log" 2>&1 &
PIDS+=($!)
sleep 1
if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
  echo "[stack] viser GUI exited during startup:" >&2
  tail -10 "$LOG_DIR/gui.log" >&2
  exit 1
fi

if [ "$SCOPE_DASHBOARD_ON" = "1" ]; then
  "$PY" -u scripts/servo_scope_dashboard.py \
    --listen "$SCOPE_DASHBOARD_STATE_LISTEN" \
    --host "$SCOPE_DASHBOARD_HOST" \
    --port "$SCOPE_DASHBOARD_PORT" \
    --csv "$SCOPE_DASHBOARD_CSV" \
    ${SCOPE_DASHBOARD_ARGS:-} >"$LOG_DIR/scope_dashboard.log" 2>&1 &
  PIDS+=($!)
  sleep 1
  if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    echo "[stack] WARNING: joint scope dashboard exited during startup:" >&2
    tail -5 "$LOG_DIR/scope_dashboard.log" >&2
    echo "[stack] (stack continues; set SCOPE_DASHBOARD=0 to skip this dashboard)" >&2
  fi
fi

VERBOSE_FLAG=""
if [ "${VERBOSE:-0}" = "1" ]; then VERBOSE_FLAG="--verbose"; fi
# ACTION_SOURCE=none|off|"" -> legacy replay/isolation mode: run the server
# (+GUI/gripper) WITHOUT the policy_runner command source. Live flow-infer no
# longer uses this path; keep `make run` up and start tools/flow_infer_real_policy.sh
# in another terminal. The stack stays up on `wait`; Ctrl-C tears it down via the
# same cleanup trap.
case "$ACTION_SOURCE" in
  none|off|"")
    echo "[stack] policy_runner: DISABLED (ACTION_SOURCE=$ACTION_SOURCE) — legacy replay/isolation mode."
    echo "[stack] For flow-infer, prefer normal make run + tools/flow_infer_real_policy.sh. Ctrl-C stops everything."
    wait
    ;;
  *)
    echo "[stack] policy_runner: $POLICY_CFG --action-source $ACTION_SOURCE $VERBOSE_FLAG"
    echo "[stack] (VERBOSE=1 make run -> live input/loop stats; Ctrl-C stops everything)"
    # UMI 텔레옵 수신 per-step 진단 로그(KST)를 실행마다 logs/ 하위에 남긴다.
    # 기본 auto; 경로를 직접 지정하거나 빈 값으로 비활성 가능 (POLICY_RUNNER_UMI_TELEOP_LOG=...).
    PYTHONPATH=policy_runner \
      POLICY_RUNNER_UMI_TELEOP_LOG="${POLICY_RUNNER_UMI_TELEOP_LOG-auto}" \
      POLICY_RUNNER_TELEOP_CAPTURE="${POLICY_RUNNER_TELEOP_CAPTURE-auto}" \
      "$PY" -u -m policy_runner --config "$POLICY_CFG" --action-source "$ACTION_SOURCE" $VERBOSE_FLAG \
      2>&1 | tee "$LOG_DIR/policy.log"
    ;;
esac
