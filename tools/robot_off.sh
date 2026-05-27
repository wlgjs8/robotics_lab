#!/usr/bin/env bash
set -u
set -o pipefail

PORT="5000"

# 지금 1대, 나중에 2대 되면 여기에 IP만 추가
ROBOTS=(
  "172.28.60.200"
  "172.28.60.201"
)

send_shutdown() {
  local ip="$1"

  nc -w 2 "$ip" "$PORT" <<'EOF'
shutdown
EOF
}

ok=0
fail=0
failed_ips=()

for ip in "${ROBOTS[@]}"; do
  echo "[INFO] Sending shutdown to $ip:$PORT ..."
  if send_shutdown "$ip"; then
    echo "[OK]   $ip"
    ok=$((ok+1))
  else
    echo "[FAIL] $ip" >&2
    fail=$((fail+1))
    failed_ips+=("$ip")
  fi
done

echo "-----------------------------"
echo "[SUMMARY] ok=$ok fail=$fail total=$((ok+fail))"
if (( fail > 0 )); then
  echo "[FAILED IPs] ${failed_ips[*]}" >&2
  exit 1
fi