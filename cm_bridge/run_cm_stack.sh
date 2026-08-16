#!/usr/bin/env bash
# cm_bridge stack launcher — `make run` (CONTROLLER=cm, the default).
#
#   make run            -> MODE=sim : controller-manager monkey platform in SILS
#   make run MODE=real  -> FAILS CLOSED until the P1 bridge command path and the
#                          P3 real device file land (see cm_bridge/docs/design.md §9).
#
# The legacy stack is always available: `make run CONTROLLER=legacy MODE=...`.
set -euo pipefail

MODE="${1:-sim}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CM="$ROOT/submodules/controller-manager"
COMPOSE=(docker compose -f "$CM/docker-compose.yaml" -f "$ROOT/cm_bridge/config/cm-compose.override.yaml")

seed_monkey_active_yaml() {
  if [ ! -f "$CM/platforms/monkey/active.yaml" ]; then
    sed -e '0,/ip: ""/s//ip: "127.0.0.1"/' -e '0,/ip: ""/s//ip: "127.0.0.2"/' \
      "$CM/platforms/monkey/params-presets/dual-arm.yaml" \
      > "$CM/platforms/monkey/active.yaml"
    echo "[cm-stack] seeded platforms/monkey/active.yaml (SILS loopback shell)"
  fi
}

case "$MODE" in
  sim)
    # Refuse to run two controllers against the same DDS/robot surface.
    if pgrep -f "rb_servo_server.*--config" >/dev/null 2>&1; then
      echo "[cm-stack] FATAL: rb_servo_server is running. Stop it first (single controller owner)." >&2
      exit 1
    fi
    seed_monkey_active_yaml
    "${COMPOSE[@]}" up -d monkey-sils
    echo "[cm-stack] monkey SILS up. Console:"
    echo "  docker exec monkey-sils bash -lc 'source /cm-ws/install/setup.bash && ros2 service call /monkey/cell/cmd/console cell_msgs/srv/Command \"{command: enable}\"'"
    echo "[cm-stack] NOTE: the cm_bridge command path (chunk -> follow) is P1 work in progress;"
    echo "           policy_runner/rb_gui are not wired to this controller yet."
    ;;
  real)
    # Single controller owner.
    if pgrep -f "rb_servo_server.*--config" >/dev/null 2>&1; then
      echo "[cm-stack] FATAL: rb_servo_server is running. Stop it first." >&2; exit 1
    fi
    DEV="$ROOT/cm_bridge/config/active.monkey.real.yaml"
    # Fail-closed: serials must be filled by the operator (nameplates).
    if grep -q 'serial_number: ""' "$DEV"; then
      echo "[cm-stack] FAIL-CLOSED: serial_number is blank in $DEV" >&2
      echo "  Fill both arms' serial_number from the nameplates, then retry." >&2
      exit 1
    fi
    cp "$DEV" "$CM/platforms/monkey/active.yaml"
    echo "[cm-stack] installed REAL device file -> platforms/monkey/active.yaml"
    "${COMPOSE[@]}" up -d monkey-real
    sleep 8
    if docker logs monkey-real 2>&1 | grep -aq "ConfigInvalid\|EXIT_FAILURE\|refus"; then
      echo "[cm-stack] FATAL: controller refused to init (firmware/config gate):" >&2
      docker logs monkey-real 2>&1 | tail -8 >&2
      "${COMPOSE[@]}" stop monkey-real >/dev/null; exit 1
    fi
    docker exec -d monkey-real bash -lc 'source /cm-ws/install/setup.bash && exec python3 /cm-bridge-src/cm_bridge_node.py --platform monkey > /tmp/cm_bridge.log 2>&1'
    nohup "$ROOT/.venv/bin/python" "$ROOT/cm_bridge/src/collision_monitor.py" \
      > "$ROOT/logs/collision_monitor_$(date +%Y%m%d_%H%M%S).log" 2>&1 &
    echo "[cm-stack] CM real up + bridge + collision monitor."
    echo "[cm-stack] Arms are NOT energized. Operator ladder (cm_bridge/docs/real_bringup.md):"
    echo "  docker exec monkey-real bash -lc 'source /cm-ws/install/setup.bash && ros2 service call /monkey/cell/cmd/console cell_msgs/srv/Command \"{command: enable}\"'"
    echo "  ... then {command: task on}, per-arm {command: task idle}"
    ;;
  gate)
    seed_monkey_active_yaml
    "${COMPOSE[@]}" up -d monkey-sils
    sleep 7
    docker exec monkey-sils bash -lc 'source /cm-ws/install/setup.bash
for c in enable "task on"; do ros2 service call /monkey/cell/cmd/console cell_msgs/srv/Command "{command: $c}" >/dev/null; sleep 2.5; done
ros2 service call /monkey/left/cmd/console cell_msgs/srv/Command "{command: task idle}" >/dev/null
ros2 service call /monkey/right/cmd/console cell_msgs/srv/Command "{command: task idle}" >/dev/null'
    docker exec monkey-sils bash -lc 'pkill -f "cm_bridge_nod[e]" 2>/dev/null; true'
    docker exec -d monkey-sils bash -lc 'source /cm-ws/install/setup.bash && exec python3 /cm-bridge-src/cm_bridge_node.py --platform monkey > /tmp/cm_bridge.log 2>&1'
    sleep 3
    python3 "$ROOT/cm_bridge/tests/cm_sils_gate.py"
    rc=$?
    "${COMPOSE[@]}" stop monkey-sils >/dev/null
    exit $rc
    ;;
  down|stop)
    "${COMPOSE[@]}" stop monkey-sils monkey-real chimp-sils 2>/dev/null || true
    pkill -f "collision_monito[r]" 2>/dev/null || true
    echo "[cm-stack] stopped"
    ;;
  *)
    echo "usage: run_cm_stack.sh {sim|real|gate|down}" >&2
    exit 2
    ;;
esac
