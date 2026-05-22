#!/usr/bin/env sh
set -eu

if [ "${RB_SIMULATOR_COMPOSE_BIND:-0}" != "1" ]; then
  exec python -m rbsim "$@"
fi

config=""
prev=""
for arg in "$@"; do
  if [ "${prev}" = "--config" ]; then
    config="${arg}"
    break
  fi
  prev="${arg}"
done

if [ -z "${config}" ]; then
  echo "rb_simulator_entrypoint: --config is required when RB_SIMULATOR_COMPOSE_BIND=1" >&2
  exit 2
fi

internal_control_port="${RB_SIMULATOR_INTERNAL_CONTROL_PORT:-15200}"
internal_admin_port="${RB_SIMULATOR_INTERNAL_ADMIN_PORT:-15201}"
public_control_port="${RB_SIMULATOR_PUBLIC_CONTROL_PORT:-50200}"
public_admin_port="${RB_SIMULATOR_PUBLIC_ADMIN_PORT:-50201}"
runtime_config="/tmp/rb_simulator_config.yaml"

python - "$config" "$runtime_config" "$internal_control_port" "$internal_admin_port" <<'PY'
from pathlib import Path
import sys

source = Path(sys.argv[1])
target = Path(sys.argv[2])
control_port = sys.argv[3]
admin_port = sys.argv[4]

text = source.read_text(encoding="utf-8")
for port in ("50200", "50210"):
    text = text.replace(f'tcp://127.0.0.1:{port}', f'tcp://127.0.0.1:{control_port}')
for port in ("50201", "50211"):
    text = text.replace(f'tcp://127.0.0.1:{port}', f'tcp://127.0.0.1:{admin_port}')
target.write_text(text, encoding="utf-8")
PY

python -m rbsim --config "${runtime_config}" &
simulator_pid="$!"

python - "$internal_control_port" <<'PY'
import socket
import sys
import time

port = int(sys.argv[1])
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.25):
            raise SystemExit(0)
    except OSError:
        time.sleep(0.05)
raise SystemExit("simulator internal control endpoint did not become ready")
PY

socat "TCP-LISTEN:${public_control_port},bind=0.0.0.0,fork,reuseaddr" "TCP:127.0.0.1:${internal_control_port}" &
control_proxy_pid="$!"
socat "TCP-LISTEN:${public_admin_port},bind=0.0.0.0,fork,reuseaddr" "TCP:127.0.0.1:${internal_admin_port}" &
admin_proxy_pid="$!"

cleanup() {
  kill "${control_proxy_pid}" "${admin_proxy_pid}" "${simulator_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait "${simulator_pid}"
