#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 --interface IFACE --left-ip IP --right-ip IP --duration-sec N --output FILE.pcapng" >&2
}

interface=""
left_ip=""
right_ip=""
duration_sec=""
output=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --interface)
            interface="${2:-}"
            shift 2
            ;;
        --left-ip)
            left_ip="${2:-}"
            shift 2
            ;;
        --right-ip)
            right_ip="${2:-}"
            shift 2
            ;;
        --duration-sec)
            duration_sec="${2:-}"
            shift 2
            ;;
        --output)
            output="${2:-}"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage
            exit 2
            ;;
    esac
done

if [[ -z "$interface" || -z "$left_ip" || -z "$right_ip" || -z "$duration_sec" || -z "$output" ]]; then
    usage
    exit 2
fi
if [[ ! "$interface" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
    echo "Invalid interface: $interface" >&2
    exit 2
fi
if [[ ! "$left_ip" =~ ^[0-9.]+$ || ! "$right_ip" =~ ^[0-9.]+$ ]]; then
    echo "Controller IPs must be IPv4 address strings." >&2
    exit 2
fi
if [[ ! "$duration_sec" =~ ^[1-9][0-9]*$ ]]; then
    echo "--duration-sec must be a positive integer." >&2
    exit 2
fi
if ! command -v dumpcap >/dev/null 2>&1; then
    echo "dumpcap is required for passive packet capture." >&2
    exit 1
fi

echo "Passive capture only: TCP data port 5001 for $left_ip and $right_ip" >&2
exec dumpcap \
    -q \
    -i "$interface" \
    -s 128 \
    -f "tcp port 5001 and (host $left_ip or host $right_ip)" \
    -a "duration:$duration_sec" \
    -w "$output"
