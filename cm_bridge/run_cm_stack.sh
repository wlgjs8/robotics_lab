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
    echo "[cm-stack] FAIL-CLOSED: CONTROLLER=cm MODE=real is not ready yet." >&2
    echo "  Missing: (a) cm_bridge command path (P1) — chunk UDP 50264 -> /monkey/<side>/cmd/follow," >&2
    echo "           state -> servo_state.v1 re-publish; (b) CollisionMonitor gate (P2);" >&2
    echo "           (c) platforms/monkey/active.yaml filled with the REAL box IPs/serials +" >&2
    echo "           this rig's calibrated mounts (P3). See cm_bridge/docs/design.md §9." >&2
    echo "  Until then: make run CONTROLLER=legacy MODE=real" >&2
    exit 1
    ;;
  down|stop)
    "${COMPOSE[@]}" stop monkey-sils chimp-sils 2>/dev/null || true
    echo "[cm-stack] stopped"
    ;;
  *)
    echo "usage: run_cm_stack.sh {sim|real|down}" >&2
    exit 2
    ;;
esac
