#!/usr/bin/env bash
# Kernel-level core isolation for the 500 Hz servo thread (AMD 5800X host).
#
#   tools/rt_core_isolation.sh --status   # read-only report (no sudo)
#   sudo tools/rt_core_isolation.sh       # apply GRUB + systemd persist + immediate steps
#
# Goal: dedicate physical core 3 (cpu3 = servo.cpu_core in the stack configs;
# its SMT sibling is cpu11) exclusively to the SCHED_FIFO servo thread:
#   - hyper-threading OFF (cpu8-15 offline; cpu0-7 numbering unchanged)
#   - cpu3 removed from the scheduler domains (isolcpus): the OS never places
#     a task there; only the servo thread lands on it via its own
#     sched_setaffinity (servo.cpu_core: 3). Collision-monitor pinning was
#     dropped to -1 (OS-scheduled) in the stack configs on this basis.
#   - INHERITANCE HARDENING: PID1 CPUAffinity=0-2,4-7 in systemd system.conf,
#     so every service/session starts with a default mask that EXCLUDES cpu3 —
#     a child process can only reach cpu3 by explicitly calling
#     sched_setaffinity itself (which is exactly and only what the servo
#     thread does). Without this, a parent that widened its own mask could
#     leak cpu3 to all of its children.
#   - device IRQs / scheduler tick / RCU callbacks kept off cpu3
#   - low-power stays disabled (C-states already off in BIOS — cpuidle driver
#     "none"; boot params below only guard against that regressing)
#
# Kernel cmdline added (REBOOT REQUIRED for isolcpus/nohz_full/rcu_nocbs):
#   nosmt isolcpus=domain,managed_irq,3 nohz_full=3 rcu_nocbs=3
#   irqaffinity=0-2,4-7 cpufreq.default_governor=performance processor.max_cstate=1
#
# Immediate (already effective before the reboot): SMT off via sysfs, default
# IRQ affinity mask off cpu3, existing IRQs migrated best-effort.
set -euo pipefail

ISOL_CPU=3
OTHER_CPUS="0-2,4-7"          # post-nosmt online set minus the isolated cpu
RUNTIME_IRQ_MASK="fff7"       # 16-thread hex mask with bit3 clear (pre-reboot)
GRUB_FILE=/etc/default/grub
SYSTEMD_CONF=/etc/systemd/system.conf
PARAMS=(
    "nosmt"
    "isolcpus=domain,managed_irq,${ISOL_CPU}"
    "nohz_full=${ISOL_CPU}"
    "rcu_nocbs=${ISOL_CPU}"
    "irqaffinity=${OTHER_CPUS}"
    "cpufreq.default_governor=performance"
    "processor.max_cstate=1"
)

status() {
    echo "[isolation] online cpus : $(cat /sys/devices/system/cpu/online)"
    echo "[isolation] smt control : $(cat /sys/devices/system/cpu/smt/control 2>/dev/null || echo n/a)"
    echo "[isolation] isolated    : '$(cat /sys/devices/system/cpu/isolated)'"
    echo "[isolation] nohz_full   : '$(cat /sys/devices/system/cpu/nohz_full 2>/dev/null | tr -d ' ' || echo n/a)'"
    echo "[isolation] cpuidle drv : $(cat /sys/devices/system/cpu/cpuidle/current_driver 2>/dev/null || echo none)"
    echo "[isolation] cpu${ISOL_CPU} governor: $(cat /sys/devices/system/cpu/cpu${ISOL_CPU}/cpufreq/scaling_governor)"
    echo "[isolation] rt throttle   : sched_rt_runtime_us=$(cat /proc/sys/kernel/sched_rt_runtime_us) (-1 = off; required for servo.spin_slack_us)"
    echo "[isolation] default irq affinity: $(cat /proc/irq/default_smp_affinity)"
    echo "[isolation] pid1 affinity: $(taskset -pc 1 2>/dev/null | sed 's/.*: //' || echo n/a)"
    echo "[isolation] cmdline     : $(cat /proc/cmdline)"
}

if [ "${1:-}" = "--status" ]; then
    status
    exit 0
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "usage: sudo $0   (or: $0 --status)" >&2
    exit 2
fi

# ---- 1) GRUB persist (idempotent, fail-closed on conflicting values) --------
current="$(grep -E '^GRUB_CMDLINE_LINUX_DEFAULT=' "$GRUB_FILE" | sed -E 's/^GRUB_CMDLINE_LINUX_DEFAULT="(.*)"$/\1/')"
new="$current"
for p in "${PARAMS[@]}"; do
    key="${p%%=*}"
    if [[ " $new " == *" $p "* ]]; then
        continue  # exact param already present
    fi
    if [[ "$p" == *=* ]] && [[ " $new " =~ [[:space:]]${key}= ]]; then
        echo "[isolation] FATAL: '$key=' already in GRUB cmdline with a different value:" >&2
        echo "[isolation]   $current" >&2
        echo "[isolation] resolve manually in $GRUB_FILE, then re-run." >&2
        exit 1
    fi
    new="$new $p"
done

if [ "$new" != "$current" ]; then
    backup="${GRUB_FILE}.bak-rt-isolation-$(date +%Y%m%d_%H%M%S)"
    cp "$GRUB_FILE" "$backup"
    sed -i -E "s|^GRUB_CMDLINE_LINUX_DEFAULT=\".*\"$|GRUB_CMDLINE_LINUX_DEFAULT=\"${new}\"|" "$GRUB_FILE"
    echo "[isolation] GRUB cmdline:"
    echo "[isolation]   old: $current"
    echo "[isolation]   new: $new"
    echo "[isolation]   backup: $backup"
    update-grub
else
    echo "[isolation] GRUB cmdline already contains every param — nothing to change"
fi

# ---- 2) systemd PID1 CPUAffinity (inheritance hardening, fail-closed) -------
# Every unit/session inherits this as its initial mask, so no child can land on
# cpu3 by inheritance; only an explicit sched_setaffinity (the servo pin) can.
want_affinity="CPUAffinity=${OTHER_CPUS}"
if grep -qE '^CPUAffinity=' "$SYSTEMD_CONF"; then
    have="$(grep -E '^CPUAffinity=' "$SYSTEMD_CONF")"
    if [ "$have" != "$want_affinity" ]; then
        echo "[isolation] FATAL: $SYSTEMD_CONF already sets '$have' (want '$want_affinity')." >&2
        echo "[isolation] resolve manually, then re-run." >&2
        exit 1
    fi
    echo "[isolation] systemd CPUAffinity already set"
else
    cp "$SYSTEMD_CONF" "${SYSTEMD_CONF}.bak-rt-isolation-$(date +%Y%m%d_%H%M%S)"
    if grep -qE '^#CPUAffinity=' "$SYSTEMD_CONF"; then
        sed -i -E "s|^#CPUAffinity=.*$|${want_affinity}|" "$SYSTEMD_CONF"
    else
        printf '\n%s\n' "$want_affinity" >> "$SYSTEMD_CONF"
    fi
    echo "[isolation] systemd system.conf: ${want_affinity} (effective from reboot)"
fi

# ---- 3) Immediate steps (effective now, superseded by the reboot) -----------
# Only toggle SMT at runtime if it is actually "on". When it is already off /
# forceoff (nosmt boot param) or notsupported (SMT disabled in BIOS / no siblings),
# writing "off" fails with ENODEV — the desired state (SMT off) is already met, so
# skip the write instead of emitting a scary error.
smt_ctl=/sys/devices/system/cpu/smt/control
if [ -w "$smt_ctl" ]; then
    smt_now="$(cat "$smt_ctl" 2>/dev/null || echo n/a)"
    if [ "$smt_now" = "on" ]; then
        echo off > "$smt_ctl" || true
    fi
    echo "[isolation] SMT: $(cat "$smt_ctl" 2>/dev/null || echo n/a) (immediate)"
fi

# Default affinity for NEW irqs + migrate existing ones off cpu3 (best effort;
# some (timer/IPI/managed) are immovable — expected, ignore failures).
printf '%s\n' "$RUNTIME_IRQ_MASK" > /proc/irq/default_smp_affinity || true
moved=0
for f in /proc/irq/[0-9]*/smp_affinity_list; do
    if grep -qw "${ISOL_CPU}" "$f" 2>/dev/null; then
        echo "$OTHER_CPUS" > "$f" 2>/dev/null && moved=$((moved+1)) || true
    fi
done
echo "[isolation] IRQs migrated off cpu${ISOL_CPU}: ${moved} (rest immovable/already off)"

# ---- 4) Disable RT throttling (persist + immediate) -------------------------
# The scheduler throttles SCHED_FIFO/RR to sched_rt_runtime_us per period
# (default 950000/1000000 = 95%): an RT task running >950 ms in any 1 s window is
# descheduled ~50 ms. The servo loop sleeps most of each tick so it normally
# never hits this — EXCEPT (a) a busy catch-up burst after a stall and (b)
# servo.spin_slack_us busy-spin, either of which can approach 100% duty and get
# throttled into a 50 ms stall. Disable it (-1). Safe here: cpu3 is isolated with
# the bounded servo loop as its only RT task, and there is no other high-duty RT
# task system-wide to run away without the throttle safety valve.
SYSCTL_DROPIN=/etc/sysctl.d/99-rb-servo-rt.conf
want_sysctl="kernel.sched_rt_runtime_us = -1"
if [ ! -f "$SYSCTL_DROPIN" ] || ! grep -qxF "$want_sysctl" "$SYSCTL_DROPIN"; then
    printf '# rb_servo: disable RT throttling for the dedicated-core 500 Hz servo loop\n%s\n' \
        "$want_sysctl" > "$SYSCTL_DROPIN"
    echo "[isolation] wrote $SYSCTL_DROPIN ($want_sysctl) — applied automatically every boot"
else
    echo "[isolation] $SYSCTL_DROPIN already sets $want_sysctl"
fi
sysctl -w kernel.sched_rt_runtime_us=-1 >/dev/null
echo "[isolation] rt throttle now: sched_rt_runtime_us=$(cat /proc/sys/kernel/sched_rt_runtime_us) (immediate)"

echo
echo "[isolation] DONE — reboot required for isolcpus/nohz_full/rcu_nocbs/CPUAffinity."
echo "[isolation] after reboot, verify with: tools/rt_core_isolation.sh --status"
echo "[isolation]   expect: online 0-7, smt off, isolated '${ISOL_CPU}', nohz_full '${ISOL_CPU}',"
echo "[isolation]           pid1 affinity ${OTHER_CPUS}"
