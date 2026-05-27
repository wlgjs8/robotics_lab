#!/usr/bin/env bash
set -u  # unbound 변수만 방지 (루프 계속을 위해 -e는 일부러 뺌)
set -o pipefail

PORT="5000"

# 지금 1대, 나중에 2대 되면 여기에 IP만 추가
ROBOTS=(
  "172.28.60.200"
  "172.28.60.201"
)

send_cmds() {
  local ip="$1"

  # init + real mode 전송
  nc -w 2 "$ip" "$PORT" <<'EOF'
mc jall init
pgmode real
EOF
}

ok=0
fail=0
failed_ips=()

for ip in "${ROBOTS[@]}"; do
  echo "[INFO] Sending to $ip:$PORT ..."
  if send_cmds "$ip"; then
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