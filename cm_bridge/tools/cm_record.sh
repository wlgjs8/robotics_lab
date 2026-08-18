#!/usr/bin/env bash
# cm_record.sh — start/stop a controller-manager `func write` capture on BOTH arms, and print
# the exact cm_replay command for what was captured.
#
#   ./cm_bridge/tools/cm_record.sh start        # both arms: data_left_*.bin + data_right_*.bin
#   ./cm_bridge/tools/cm_record.sh stop
#   ./cm_bridge/tools/cm_record.sh where        # where captures land, newest pair, matching sidecar
#
# The capture is the CONTROLLER's (DataRecorder, RT-safe ring -> writer thread); this script only
# sends the console command. The bridge sidecar (chunk content + gripper events) is always-on for
# the bridge process, so a capture window is automatically covered by the sidecar of the bridge
# that was running at the time - `where` pairs the newest capture with it.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CM="$ROOT/submodules/controller-manager"
PLATFORM="${CONTROL_MANAGER_PLATFORM:-monkey}"
MODE="${1:-}"
set +u; source "$CM/platforms/$PLATFORM/scripts/env.sh" >/dev/null 2>&1; set -u
CAPDIR="${CONTROL_MANAGER_LOG_DIR:-${CHIMPANZEE_LOG_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/plaif-chimpanzee/logs}}"

C() { ros2 service call "/$PLATFORM/$1/cmd/console" cell_msgs/srv/Command "{command: '$2'}" 2>&1 | grep -o "accepted=[A-Za-z]*" | head -1; }

case "$MODE" in
  start)
    echo "[cm-record] func write start: left $(C left 'func write start')  right $(C right 'func write start')"
    sleep 0.5
    echo "[cm-record] capture dir: $CAPDIR"
    ls -t "$CAPDIR"/data_*.bin 2>/dev/null | head -2 | sed 's/^/[cm-record]   /' || true
    ;;
  stop)
    echo "[cm-record] func write stop:  left $(C left 'func write stop')  right $(C right 'func write stop')"
    sleep 1
    ;&   # fall through to `where`
  where)
    # (each `|| true`: with set -e/pipefail an empty glob would abort the script silently)
    NEWEST=$(ls -t "$CAPDIR"/data_*.bin 2>/dev/null | head -2 | tr '\n' ' ') || true
    SIDECAR=$(ls -t "$ROOT"/logs/cm_bridge_sidecar_*.jsonl 2>/dev/null | head -1) || true
    OBSDUMP=$(ls -td "$ROOT"/outputs/obs_dump/*/ 2>/dev/null | head -1) || true
    if [ -z "$NEWEST" ]; then
      echo "[cm-record] NO CAPTURE YET in $CAPDIR - the controller recorder was never started."
      echo "[cm-record] Run  ./cm_bridge/tools/cm_record.sh start  BEFORE the rollout (func write start on both arms)."
    fi
    echo "[cm-record] capture dir : $CAPDIR"
    echo "[cm-record] newest pair : ${NEWEST:-<none>}"
    echo "[cm-record] sidecar     : ${SIDECAR:-<none - is the bridge running the sidecar build?>}"
    echo "[cm-record] obs dump    : ${OBSDUMP:-<none - flow_infer_sweep_run.sh writes outputs/obs_dump/<stamp>_<tag>/>}"
    if [ -n "$NEWEST" ]; then
      echo "[cm-record] replay:"
      echo "  .venv/bin/python cm_bridge/tools/cm_replay.py --bin $NEWEST${SIDECAR:+--sidecar $SIDECAR}${OBSDUMP:+ --obs-dump $OBSDUMP}"
      echo "[cm-record] re-infer one chunk's frame with a different velocity proprio, then overlay it:"
      echo "  /home/plaif/workspace/openpi/.venv/bin/python cm_bridge/tools/reinfer.py --dump ${OBSDUMP:-<obs_dump>} --seq <inference_seq> --samples 8 --vel-scale 0.5,1,1.5 --json /tmp/variants.json"
      echo "  ... cm_replay.py ... --variants /tmp/variants.json"
    fi
    ;;
  *)
    echo "usage: cm_record.sh {start|stop|where}" >&2; exit 2 ;;
esac
