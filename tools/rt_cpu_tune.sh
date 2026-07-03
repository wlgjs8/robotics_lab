#!/usr/bin/env bash
# RT CPU latency tuning for the 500 Hz servo stack (run with sudo).
#
#   sudo tools/rt_cpu_tune.sh            # apply now (until reboot)
#   sudo tools/rt_cpu_tune.sh --persist  # also install a boot-time systemd unit
#   tools/rt_cpu_tune.sh --status        # read-only report (no sudo needed)
#
# What it covers, in order of measured impact on this machine (AMD 5800X):
#
# 1) C-states — VERIFIED ALREADY DISABLED at the firmware level on this host:
#    the kernel cpuidle driver is "none" and /sys/.../cpuidle does not exist,
#    i.e. the BIOS (Global C-State Control) exposes no ACPI C-states, so the
#    OS idles with plain HLT (C1) and deep core sleeps (CC6) are never entered.
#    This script FAILS LOUDLY if C-states ever reappear (e.g. after a BIOS
#    reset/update) so the regression is caught before it shows up as servo
#    wake-latency jitter. If that happens: BIOS -> AMD CBS/Advanced ->
#    "Global C-state Control" = Disabled, or add "processor.max_cstate=1" to
#    GRUB_CMDLINE_LINUX_DEFAULT as the OS-side fallback.
#
# 2) cpufreq governor -> performance: with schedutil the core sags toward
#    2.2 GHz across idle gaps and ramps after wake; pinning the governor keeps
#    the servo core at full clock so the first instructions after sleep_until
#    run at speed. This is the remaining OS-side "core sleep" knob on this box.
#
# (idle=poll would shave the last ~1 us of HLT exit at the cost of running all
#  cores hot; measured wake p50 is already ~8 us so it is intentionally NOT
#  applied here. Revisit only if wake_latency_us p99 must go below ~10 us.)
set -euo pipefail

MODE="${1:-apply}"

cstate_report() {
    local driver="none(absent)"
    [ -r /sys/devices/system/cpu/cpuidle/current_driver ] &&
        driver="$(cat /sys/devices/system/cpu/cpuidle/current_driver)"
    echo "[rt-tune] cpuidle driver: ${driver}"
    if [ "${driver}" != "none" ] && [ "${driver}" != "none(absent)" ]; then
        echo "[rt-tune] FATAL: cpuidle driver '${driver}' registered — C-states are ENABLED again." >&2
        echo "[rt-tune]        Fix in BIOS (Global C-state Control = Disabled) or add" >&2
        echo "[rt-tune]        processor.max_cstate=1 to the kernel cmdline, then re-run." >&2
        return 1
    fi
    if compgen -G "/sys/devices/system/cpu/cpu0/cpuidle/state*" > /dev/null; then
        echo "[rt-tune] FATAL: per-cpu cpuidle states exist — C-states are ENABLED again." >&2
        return 1
    fi
    echo "[rt-tune] C-states: disabled (no ACPI idle states exposed; idle = HLT/C1 only)"
}

governor_report() {
    local govs
    govs="$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c | sed 's/^ *//')"
    echo "[rt-tune] governors: ${govs//$'\n'/, }"
    echo "[rt-tune] core3 freq: cur=$(cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq) kHz" \
         "min=$(cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_min_freq)" \
         "max=$(cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_max_freq)"
}

if [ "${MODE}" = "--status" ]; then
    cstate_report || true
    governor_report
    exit 0
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "usage: sudo $0 [--persist] | $0 --status" >&2
    exit 2
fi

cstate_report

for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance > "$g"
done
echo "[rt-tune] governor -> performance on all cores"
governor_report

if [ "${MODE}" = "--persist" ]; then
    UNIT=/etc/systemd/system/rt-cpu-tune.service
    SCRIPT_PATH="$(readlink -f "$0")"
    cat > "$UNIT" <<EOF
[Unit]
Description=RT CPU tuning for robotics_lab servo stack (governor=performance, C-state guard)
After=multi-user.target

[Service]
Type=oneshot
ExecStart=${SCRIPT_PATH}

[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    systemctl enable rt-cpu-tune.service
    echo "[rt-tune] installed + enabled ${UNIT} (applies at every boot)"
fi
