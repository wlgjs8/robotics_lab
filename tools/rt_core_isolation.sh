#!/usr/bin/env bash
# Kernel-level core isolation for the 500 Hz servo stack (AMD Ryzen 9 9950X host,
# 16 cores, SMT disabled in the BIOS).
#
#   tools/rt_core_isolation.sh --status   # read-only report (no sudo)
#   sudo tools/rt_core_isolation.sh       # apply GRUB + systemd persist + immediate steps
#
# Core ownership (every owner pins itself with sched_setaffinity + SCHED_FIFO;
# the kernel never places anything else on an isolated core):
#   cpu1  left  arm worker   (servo.worker_cpu_core_left,  FIFO 80)   -- controller-manager's
#   cpu2  right arm worker   (servo.worker_cpu_core_right, FIFO 80)      left/right RT loops
#   cpu3  500 Hz servo loop  (servo.cpu_core,              FIFO 80)      use cpu1/cpu2 too
#   cpu4  self-collision monitor (safety.self_collision.mesh.monitor_core, FIFO 50;
#         added 2026-09-04 -- it was the only safety-critical thread left to CFS)
#   cpu0, cpu5-15  housekeeping: IRQs, GNOME, the runner, the policy servers,
#         camera_server (docker-compose cpuset 5,6,7).
#
# What this script does:
#   - isolcpus=domain,managed_irq,1-4: the four cores leave the scheduler domains
#   - nohz_full / rcu_nocbs on the same set: no scheduler tick / RCU callbacks there
#   - irqaffinity=0,5-15: device IRQs stay off the isolated cores
#   - INHERITANCE HARDENING: PID1 CPUAffinity=0,5-15 in systemd system.conf, so every
#     service/session starts with a mask that EXCLUDES the isolated cores; a child
#     reaches one only by calling sched_setaffinity itself (exactly what the
#     owners above do)
#   - cpufreq performance governor, processor.max_cstate=1 (C-states are already off
#     in the BIOS; these only guard against that regressing)
#   - RT throttling off (sched_rt_runtime_us=-1): the servo loop's catch-up bursts
#     and spin_slack would otherwise be throttled into a 50 ms stall
#
# History: cpu3 only (2026-06, 5800X), cpu1-3 (2026-08-18, the workers), cpu1-4
# (2026-09-04, the collision monitor). REBOOT REQUIRED for the isolcpus /
# nohz_full / rcu_nocbs / CPUAffinity changes; the IRQ migration and the
# throttling switch take effect immediately.
set -euo pipefail

ISOL_LIST="1-4"
ISOL_CPUS="1 2 3 4"
OTHER_CPUS="0,5-15"
RUNTIME_IRQ_MASK="ffe1"       # 16-bit hex mask with bits 1-4 clear (pre-reboot)
GRUB_FILE=/etc/default/grub
SYSTEMD_CONF=/etc/systemd/system.conf
# Every `key=` below REPLACES an existing key of the same name in the cmdline (so
# the 2026-08-18 1-3 line migrates to 1-4 without manual editing); bare flags are
# added once.
PARAMS=(
    "isolcpus=domain,managed_irq,${ISOL_LIST}"
    "nohz_full=${ISOL_LIST}"
    "rcu_nocbs=${ISOL_LIST}"
    "irqaffinity=${OTHER_CPUS}"
    "cpufreq.default_governor=performance"
    "processor.max_cstate=1"
)

status() {
    echo "[isolation] online cpus : $(cat /sys/devices/system/cpu/online)"
    echo "[isolation] smt control : $(cat /sys/devices/system/cpu/smt/control 2>/dev/null || echo n/a)"
    echo "[isolation] isolated    : '$(cat /sys/devices/system/cpu/isolated)'   (want '${ISOL_LIST}')"
    echo "[isolation] nohz_full   : '$(cat /sys/devices/system/cpu/nohz_full 2>/dev/null | tr -d ' ' || echo n/a)'"
    echo "[isolation] cpuidle drv : $(cat /sys/devices/system/cpu/cpuidle/current_driver 2>/dev/null || echo none)"
    for c in ${ISOL_CPUS}; do
        echo "[isolation] cpu${c} governor: $(cat /sys/devices/system/cpu/cpu${c}/cpufreq/scaling_governor 2>/dev/null || echo n/a)"
    done
    echo "[isolation] rt throttle   : sched_rt_runtime_us=$(cat /proc/sys/kernel/sched_rt_runtime_us) (-1 = off; required for servo.spin_slack_us)"
    echo "[isolation] default irq affinity: $(cat /proc/irq/default_smp_affinity)"
    echo "[isolation] pid1 affinity: $(taskset -pc 1 2>/dev/null | sed 's/.*: //' || echo n/a)"
    echo "[isolation] cmdline     : $(cat /proc/cmdline)"
    echo "[isolation] grub file   : $(grep -E '^GRUB_CMDLINE_LINUX_DEFAULT=' "$GRUB_FILE" 2>/dev/null || echo n/a)"
    echo "[isolation] threads on the isolated cores (pid tid psr rtprio comm):"
    for t in /proc/[0-9]*/task/[0-9]*; do
        [ -r "$t/stat" ] || continue
        # field 39 of /proc/<pid>/task/<tid>/stat is the processor last run on
        read -r -a f < <(sed -E 's/^([0-9]+) \((.*)\) /\1 x /' "$t/stat" 2>/dev/null || true)
        [ "${#f[@]}" -ge 40 ] || continue
        psr="${f[38]}"
        case " ${ISOL_CPUS} " in
            *" ${psr} "*)
                pid="$(basename "$(dirname "$(dirname "$t")")")"
                tid="$(basename "$t")"
                comm="$(cat "$t/comm" 2>/dev/null || echo ?)"
                case "$comm" in kworker*|cpuhp*|ksoftirqd*|migration*|idle_inject*|rcu*|irq/*) continue;; esac
                echo "[isolation]   ${pid} ${tid} cpu${psr} rt=${f[39]} ${comm}" ;;
        esac
    done
}

if [ "${1:-}" = "--status" ]; then
    status
    exit 0
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "usage: sudo $0   (or: $0 --status)" >&2
    exit 2
fi

# ---- 1) GRUB persist: replace same-named keys, add missing ones -------------
current="$(grep -E '^GRUB_CMDLINE_LINUX_DEFAULT=' "$GRUB_FILE" | sed -E 's/^GRUB_CMDLINE_LINUX_DEFAULT="(.*)"$/\1/')"
new="$current"
for p in "${PARAMS[@]}"; do
    key="${p%%=*}"
    if [[ " $new " == *" $p "* ]]; then
        continue  # exact param already present
    fi
    if [[ "$p" == *=* ]] && [[ " $new " =~ [[:space:]]${key}=[^[:space:]]* ]]; then
        old_param="$(echo " $new " | grep -oE "[[:space:]]${key}=[^[:space:]]*" | head -1 | sed 's/^ //')"
        echo "[isolation] GRUB: replacing '${old_param}' with '${p}'"
        new="$(echo " $new " | sed -E "s|[[:space:]]${key}=[^[:space:]]*| ${p}|" | sed -E 's/^ //; s/ $//')"
        continue
    fi
    new="$new $p"
done
# nosmt is a leftover of the 5800X era; SMT is off in the BIOS on this host and a
# stale nosmt would only mask a BIOS regression. Leave it if present.

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

# ---- 2) systemd PID1 CPUAffinity (inheritance hardening) --------------------
want_affinity="CPUAffinity=${OTHER_CPUS}"
if grep -qE '^CPUAffinity=' "$SYSTEMD_CONF"; then
    have="$(grep -E '^CPUAffinity=' "$SYSTEMD_CONF")"
    if [ "$have" != "$want_affinity" ]; then
        cp "$SYSTEMD_CONF" "${SYSTEMD_CONF}.bak-rt-isolation-$(date +%Y%m%d_%H%M%S)"
        sed -i -E "s|^CPUAffinity=.*$|${want_affinity}|" "$SYSTEMD_CONF"
        echo "[isolation] systemd system.conf: '$have' -> '${want_affinity}' (effective from reboot)"
    else
        echo "[isolation] systemd CPUAffinity already set"
    fi
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
printf '%s\n' "$RUNTIME_IRQ_MASK" > /proc/irq/default_smp_affinity || true
moved=0
for f in /proc/irq/[0-9]*/smp_affinity_list; do
    for c in ${ISOL_CPUS}; do
        if grep -qw "${c}" "$f" 2>/dev/null; then
            echo "$OTHER_CPUS" > "$f" 2>/dev/null && moved=$((moved+1)) || true
            break
        fi
    done
done
echo "[isolation] IRQs migrated off cpu${ISOL_LIST}: ${moved} (rest immovable/already off)"

# ---- 4) Disable RT throttling (persist + immediate) -------------------------
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
echo "[isolation]   expect: isolated '${ISOL_LIST}', nohz_full '${ISOL_LIST}', pid1 affinity ${OTHER_CPUS},"
echo "[isolation]           and after 'make run': servo on cpu3, workers on cpu1/2, collision monitor on cpu4"
