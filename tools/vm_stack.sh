#!/usr/bin/env bash
# Rainbow VIRTUAL control-box stack (vendor RBVirtualSimulator OVA in VirtualBox).
#
#   make vm-up      boot rb-cb-left/right headless and map them to the REAL
#                   controller addresses (172.28.60.200/.201 -> guests) so the
#                   unmodified stack configs / `make run MODE=sim` work on VMs
#   make vm-down    remove the mapping and power the VMs off
#   make vm-status  show VM + mapping + port state
#
# Topology:
#   vboxnet0 (host 10.0.2.1/24)
#     rb-cb-left  guest 10.0.2.7  <- DNAT 172.28.60.200
#     rb-cb-right guest 10.0.2.8  <- DNAT 172.28.60.201
#
# While the mapping is active, traffic to the REAL robots is intercepted —
# that is intentional (you cannot accidentally reach the physical controllers
# during VM testing). `vm-down` restores normal routing.
#
# NOTE: the virtual control box only supports pgmode SIMULATION (activation is
# impossible by design, [A187]) — use `make run MODE=sim`.
set -euo pipefail
cd "$(dirname "$0")/.."

# VBoxHeadless drops its frontend release log into the current working dir by
# default (which is the repo root). Redirect it into logs/vm/ instead. The
# --putenv flags below cover the VM process even when a long-lived VBoxSVC was
# started before this shell inherited VBOX_*.
VM_LOG_DIR="$PWD/logs/vm"
mkdir -p "$VM_LOG_DIR"
export VBOX_RELEASE_LOG_DEST="dir=$VM_LOG_DIR"
export VBOX_LOG_DEST="dir=$VM_LOG_DIR"

LEFT_VM=rb-cb-left;   LEFT_GUEST=10.0.2.7
RIGHT_VM=rb-cb-right; RIGHT_GUEST=10.0.2.8
HOST_IF=vboxnet0;     HOST_IP=10.0.2.1/24
REAL_LEFT=172.28.60.200
REAL_RIGHT=172.28.60.201

# Load SUDO_PASSWORD from .env (repo root, gitignored) so `make vm-*` does not
# prompt interactively. The password is fed to `sudo -A` through a private
# askpass helper (mode 0700, removed on exit) instead of the env/argv, so it
# never leaks into the process table or shell history.
if [ -z "${SUDO_PASSWORD:-}" ] && [ -f .env ]; then
  SUDO_PASSWORD="$(sed -n 's/^[[:space:]]*SUDO_PASSWORD[[:space:]]*=[[:space:]]*//p' .env | tail -n1)"
  # strip optional surrounding quotes
  SUDO_PASSWORD="${SUDO_PASSWORD%\"}"; SUDO_PASSWORD="${SUDO_PASSWORD#\"}"
  SUDO_PASSWORD="${SUDO_PASSWORD%\'}"; SUDO_PASSWORD="${SUDO_PASSWORD#\'}"
fi

SUDO="sudo"
if [ -n "${SUDO_PASSWORD:-}" ] && [ -z "${SUDO_ASKPASS:-}" ]; then
  ASKPASS_FILE="$(mktemp)"
  chmod 700 "$ASKPASS_FILE"
  trap 'rm -f "$ASKPASS_FILE"' EXIT
  printf '#!/usr/bin/env bash\nprintf %%s "$SUDO_PASSWORD"\n' >"$ASKPASS_FILE"
  export SUDO_PASSWORD
  export SUDO_ASKPASS="$ASKPASS_FILE"
fi
if [ -n "${SUDO_ASKPASS:-}" ]; then SUDO="sudo -A"; fi

dnat_rule() { # $1=-A|-D|-C  $2=real ip  $3=guest ip
  $SUDO iptables -t nat "$1" OUTPUT -d "$2" -j DNAT --to-destination "$3"
}

map_up() {
  $SUDO ip addr replace "$HOST_IP" dev "$HOST_IF"
  $SUDO ip link set "$HOST_IF" up
  $SUDO ip route replace "$REAL_LEFT/32" dev "$HOST_IF"
  $SUDO ip route replace "$REAL_RIGHT/32" dev "$HOST_IF"
  dnat_rule -C "$REAL_LEFT" "$LEFT_GUEST" 2>/dev/null || dnat_rule -A "$REAL_LEFT" "$LEFT_GUEST"
  dnat_rule -C "$REAL_RIGHT" "$RIGHT_GUEST" 2>/dev/null || dnat_rule -A "$REAL_RIGHT" "$RIGHT_GUEST"
}

map_down() {
  dnat_rule -D "$REAL_LEFT" "$LEFT_GUEST" 2>/dev/null || true
  dnat_rule -D "$REAL_RIGHT" "$RIGHT_GUEST" 2>/dev/null || true
  $SUDO ip route del "$REAL_LEFT/32" dev "$HOST_IF" 2>/dev/null || true
  $SUDO ip route del "$REAL_RIGHT/32" dev "$HOST_IF" 2>/dev/null || true
}

vm_running() { vboxmanage list runningvms | grep -q "\"$1\""; }

relocate_vbox_logs() {
  local log
  shopt -s nullglob
  for log in "$PWD"/*-VBoxHeadless-*.log; do
    [[ -f "$log" ]] || continue
    mv -n -- "$log" "$VM_LOG_DIR/"
  done
  shopt -u nullglob
}

start_vm() {
  if ! vm_running "$1"; then
    vboxmanage startvm \
      "--putenv=VBOX_RELEASE_LOG_DEST=dir=$VM_LOG_DIR" \
      "--putenv=VBOX_LOG_DEST=dir=$VM_LOG_DIR" \
      --type headless \
      "$1" >/dev/null
  fi
  relocate_vbox_logs
}

wait_port() { # $1=ip
  for _ in $(seq 1 120); do
    if timeout 1 bash -c "echo > /dev/tcp/$1/5000" 2>/dev/null; then return 0; fi
    sleep 1
  done
  return 1
}

case "${1:-}" in
  up)
    echo "[vm] mapping $REAL_LEFT->$LEFT_GUEST, $REAL_RIGHT->$RIGHT_GUEST"
    map_up
    echo "[vm] starting VMs (headless)"
    start_vm "$LEFT_VM"
    start_vm "$RIGHT_VM"
    echo "[vm] waiting for control boxes (rbpodo :5000)..."
    wait_port "$REAL_LEFT"  && echo "[vm] left  up at $REAL_LEFT ($LEFT_GUEST)"  || { echo "[vm] left did not come up" >&2; exit 1; }
    wait_port "$REAL_RIGHT" && echo "[vm] right up at $REAL_RIGHT ($RIGHT_GUEST)" || { echo "[vm] right did not come up" >&2; exit 1; }
    echo "[vm] ready — run: make run MODE=sim"
    ;;
  down)
    echo "[vm] removing mapping"
    map_down
    for vm in "$LEFT_VM" "$RIGHT_VM"; do
      if vm_running "$vm"; then
        echo "[vm] powering off $vm"
        vboxmanage controlvm "$vm" poweroff >/dev/null 2>&1 || true
      fi
    done
    echo "[vm] down."
    ;;
  status)
    echo "--- VMs ---"
    vboxmanage list runningvms | grep -E "rb-cb-(left|right)" || echo "(none running)"
    echo "--- mapping ---"
    $SUDO iptables -t nat -S OUTPUT 2>/dev/null | grep -E "172.28.60.20[01]" || echo "(no DNAT rules)"
    echo "--- ports ---"
    for ip in "$REAL_LEFT" "$REAL_RIGHT"; do
      if timeout 1 bash -c "echo > /dev/tcp/$ip/5000" 2>/dev/null; then
        echo "$ip:5000 OPEN"
      else
        echo "$ip:5000 closed"
      fi
    done
    ;;
  *)
    echo "usage: $0 up|down|status" >&2
    exit 2
    ;;
esac
