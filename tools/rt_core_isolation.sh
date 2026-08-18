#!/usr/bin/env bash
# Kernel-level core isolation for the RT control threads.
#
#   tools/rt_core_isolation.sh --status          # read-only report (no sudo)
#   sudo tools/rt_core_isolation.sh              # apply GRUB + systemd persist + immediate steps
#   sudo tools/rt_core_isolation.sh --cpus 1,2,3 # isolate a different set
#
# HOST: AMD Ryzen 9 9950X — 16 PHYSICAL cores, SMT disabled in BIOS
# (/sys/devices/system/cpu/smt/control reads "notsupported"), so cpu0-15 are all
# real cores and there are no siblings to take offline. Retargeted 2026-08-18
# from the previous 8-core 5800X host, where cpu8-15 were SMT siblings and
# OTHER_CPUS was 0-2,4-7 — running that unchanged here would have confined every
# systemd unit and every IRQ to half the machine.
#
# ISOLATED SET (default 1,2,3) — 2026-08-18, widened from the single cpu3:
#   cpu3 = rb_servo_server's 500 Hz SCHED_FIFO servo thread (servo.cpu_core: 3
#          in the stack configs). The original and still-active reason.
#   cpu1 = controller-manager's LEFT-arm RT loop. That pin is HARDCODED in the
#          submodule (src/arm/Arm.cpp: Right ? 2 : 1) and its own
#          platforms/*/scripts/cpu-isolate.sh reserves 1-3 on the same grounds
#          (its wiki decision 0004). We consume controller-manager read-only, so
#          the host has to meet the pin rather than the pin move.
#   cpu2 = controller-manager's RIGHT-arm RT loop, same hardcoded pin. Isolated
#          for the same reason: the combined stack runs dual-arm.
#   (--cpus 3 restores the pre-2026-08-18 rb_servo-only set.)
# cpu0 is never isolated — timer tick / RCU / timekeeping housekeeping lives there.
#
# Goal, per isolated core:
#   - removed from the scheduler domains (isolcpus): the OS never places a task
#     there; only a thread that calls sched_setaffinity itself lands on it.
#     Collision-monitor pinning was dropped to -1 (OS-scheduled) on this basis.
#   - INHERITANCE HARDENING: PID1 CPUAffinity in systemd system.conf, so every
#     service/session starts with a default mask that EXCLUDES the isolated set —
#     a child can only reach one by explicitly calling sched_setaffinity (which is
#     exactly and only what the RT threads do). Without this, a parent that
#     widened its own mask could leak an isolated core to all of its children.
#   - device IRQs / scheduler tick / RCU callbacks kept off them. This matters
#     concretely here: measured 2026-08-18, the robot NIC's four busiest queues
#     (enp16s0f1-TxRx-0..3, IRQ 155-158, one of them with 143M interrupts) were
#     pinned to "3-4", i.e. ON the servo core, along with an nvidia IRQ.
#   - low-power / frequency scaling pinned down. NOT already handled by BIOS on
#     this host: measured cpuidle driver "acpi_idle" and cpu3 governor
#     "powersave", both of which add wake latency and clock variation to the tick.
#
# Kernel cmdline written (REBOOT REQUIRED for isolcpus/nohz_full/rcu_nocbs):
#   nosmt isolcpus=domain,managed_irq,<set> nohz_full=<set> rcu_nocbs=<set>
#   irqaffinity=<rest> cpufreq.default_governor=performance processor.max_cstate=1
#
# `nosmt` is a no-op while SMT is off in BIOS; it is kept as a guard so that
# re-enabling SMT in BIOS cannot silently hand an isolated core a sibling that
# shares its execution resources.
#
# Immediate (already effective before the reboot): default IRQ affinity mask off
# the isolated set, existing IRQs migrated best-effort, RT throttling disabled.
#
# Re-running with a DIFFERENT set rewrites the params this script owns (backing
# up /etc/default/grub and /etc/systemd/system.conf first) instead of refusing:
# widening the set is a normal operation now that two stacks pin cores. Any
# cmdline key this script does not own is left untouched.
set -euo pipefail

ISOL_CPUS="1,2,3"             # default set — see the header for who owns each core
GRUB_FILE=/etc/default/grub
SYSTEMD_CONF=/etc/systemd/system.conf
SYSCTL_DROPIN=/etc/sysctl.d/99-rb-servo-rt.conf

MODE=apply
while [ $# -gt 0 ]; do
    case "$1" in
        --status) MODE=status ;;
        --cpus)   shift; ISOL_CPUS="${1:-}" ;;
        -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

NCPU="$(nproc --all)"

# ---- cpu-list helpers ------------------------------------------------------
# expand "1,3" / "0-2,4-15" -> "1 3" / "0 1 2 4 ... 15"
expand_list() {
    local spec="$1" part lo hi i out=""
    for part in ${spec//,/ }; do
        case "$part" in
            *-*) lo="${part%%-*}"; hi="${part##*-}"
                 for ((i=lo; i<=hi; i++)); do out="$out $i"; done ;;
            '')  ;;
            *)   out="$out $part" ;;
        esac
    done
    echo "${out# }"
}

# compact "0 2 4 5 6" -> "0,2,4-6"
compact_list() {
    local n prev="" start="" out=""
    for n in $(printf '%s\n' "$@" | sort -n -u); do
        if [ -z "$start" ]; then start="$n"; prev="$n"; continue; fi
        if [ "$n" -eq $((prev + 1)) ]; then prev="$n"; continue; fi
        if [ "$start" = "$prev" ]; then out="$out,$start"
        elif [ "$prev" -eq $((start + 1)) ]; then out="$out,$start,$prev"
        else out="$out,$start-$prev"; fi
        start="$n"; prev="$n"
    done
    if [ -n "$start" ]; then
        if [ "$start" = "$prev" ]; then out="$out,$start"
        elif [ "$prev" -eq $((start + 1)) ]; then out="$out,$start,$prev"
        else out="$out,$start-$prev"; fi
    fi
    echo "${out#,}"
}

ISOL_ARR=($(expand_list "$ISOL_CPUS"))
[ "${#ISOL_ARR[@]}" -gt 0 ] || { echo "[isolation] FATAL: empty --cpus set" >&2; exit 2; }
for c in "${ISOL_ARR[@]}"; do
    case "$c" in ''|*[!0-9]*) echo "[isolation] FATAL: not a cpu number: '$c'" >&2; exit 2 ;; esac
    [ "$c" -lt "$NCPU" ] || { echo "[isolation] FATAL: cpu$c does not exist (nproc --all = $NCPU)" >&2; exit 2; }
    [ "$c" -ne 0 ] || { echo "[isolation] FATAL: cpu0 carries housekeeping — refusing to isolate it" >&2; exit 2; }
done
ISOL_SET=" ${ISOL_ARR[*]} "
is_isolated() { case "$ISOL_SET" in *" $1 "*) return 0 ;; *) return 1 ;; esac; }

OTHER_ARR=()
for ((i=0; i<NCPU; i++)); do is_isolated "$i" || OTHER_ARR+=("$i"); done
[ "${#OTHER_ARR[@]}" -ge 4 ] || { echo "[isolation] FATAL: only ${#OTHER_ARR[@]} housekeeping cpus left" >&2; exit 1; }

ISOL_LIST="$(compact_list "${ISOL_ARR[@]}")"      # e.g. 1,3
OTHER_CPUS="$(compact_list "${OTHER_ARR[@]}")"    # e.g. 0,2,4-15

# hex mask of the NON-isolated cpus, for /proc/irq/default_smp_affinity (pre-reboot)
mask=0
for c in "${OTHER_ARR[@]}"; do mask=$(( mask | (1 << c) )); done
RUNTIME_IRQ_MASK="$(printf '%x' "$mask")"

PARAMS=(
    "nosmt"
    "isolcpus=domain,managed_irq,${ISOL_LIST}"
    "nohz_full=${ISOL_LIST}"
    "rcu_nocbs=${ISOL_LIST}"
    "irqaffinity=${OTHER_CPUS}"
    "cpufreq.default_governor=performance"
    "processor.max_cstate=1"
)

status() {
    echo "[isolation] target set  : ${ISOL_LIST}  (housekeeping ${OTHER_CPUS})"
    echo "[isolation] online cpus : $(cat /sys/devices/system/cpu/online)"
    echo "[isolation] smt control : $(cat /sys/devices/system/cpu/smt/control 2>/dev/null || echo n/a)"
    echo "[isolation] isolated    : '$(cat /sys/devices/system/cpu/isolated)'"
    echo "[isolation] nohz_full   : '$(cat /sys/devices/system/cpu/nohz_full 2>/dev/null | tr -d ' ' || echo n/a)'"
    echo "[isolation] cpuidle drv : $(cat /sys/devices/system/cpu/cpuidle/current_driver 2>/dev/null || echo none)"
    for c in "${ISOL_ARR[@]}"; do
        echo "[isolation] cpu${c} governor: $(cat /sys/devices/system/cpu/cpu${c}/cpufreq/scaling_governor 2>/dev/null || echo n/a)"
    done
    echo "[isolation] rt throttle   : sched_rt_runtime_us=$(cat /proc/sys/kernel/sched_rt_runtime_us) (-1 = off; required for servo.spin_slack_us)"
    echo "[isolation] default irq affinity: $(cat /proc/irq/default_smp_affinity)"
    echo "[isolation] pid1 affinity: $(taskset -pc 1 2>/dev/null | sed 's/.*: //' || echo n/a)"
    echo "[isolation] cmdline     : $(cat /proc/cmdline)"
}

if [ "$MODE" = status ]; then
    status
    exit 0
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "usage: sudo $0 [--cpus ${ISOL_CPUS}]   (or: $0 --status)" >&2
    exit 2
fi

# ---- 1) GRUB persist (idempotent; rewrites only the keys this script owns) --
current="$(grep -E '^GRUB_CMDLINE_LINUX_DEFAULT=' "$GRUB_FILE" | sed -E 's/^GRUB_CMDLINE_LINUX_DEFAULT="(.*)"$/\1/')"
new="$current"
for p in "${PARAMS[@]}"; do
    key="${p%%=*}"
    if [[ " $new " == *" $p "* ]]; then
        continue  # exact param already present
    fi
    if [[ "$p" == *=* ]]; then
        # same key, different value: OURS to rewrite (the set changed) — say so loudly
        # NOTE the '#' sed delimiter: the alternation (^|[[:space:]]) contains a '|',
        # which a s|...|...| would parse as the end of the pattern.
        old="$(printf '%s' "$new" | grep -oE "(^|[[:space:]])${key}=[^[:space:]]*" | tr -d ' ' || true)"
        if [ -n "$old" ]; then
            echo "[isolation] rewriting ${old}  ->  ${p}"
            new="$(printf '%s' "$new" | sed -E "s#(^|[[:space:]])${key}=[^[:space:]]*#\1#g")"
        fi
    fi
    new="$new $p"
done
new="$(printf '%s' "$new" | tr -s ' ' | sed 's/^ //; s/ $//')"

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

# ---- 2) systemd PID1 CPUAffinity (inheritance hardening) -------------------
# Every unit/session inherits this as its initial mask, so no child can land on
# an isolated core by inheritance; only an explicit sched_setaffinity (the RT
# pins) can.
want_affinity="CPUAffinity=${OTHER_CPUS}"
if grep -qE '^CPUAffinity=' "$SYSTEMD_CONF"; then
    have="$(grep -E '^CPUAffinity=' "$SYSTEMD_CONF")"
    if [ "$have" != "$want_affinity" ]; then
        cp "$SYSTEMD_CONF" "${SYSTEMD_CONF}.bak-rt-isolation-$(date +%Y%m%d_%H%M%S)"
        sed -i -E "s|^CPUAffinity=.*$|${want_affinity}|" "$SYSTEMD_CONF"
        echo "[isolation] systemd CPUAffinity: '${have}' -> '${want_affinity}' (effective from reboot)"
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

# Default affinity for NEW irqs + migrate existing ones off the isolated set
# (best effort; some (timer/IPI/managed) are immovable — expected, ignore failures).
printf '%s\n' "$RUNTIME_IRQ_MASK" > /proc/irq/default_smp_affinity || true

# an IRQ is "on the set" if its affinity LIST (which may hold ranges) expands to
# include any isolated cpu — a plain grep -w misses "0-15".
irq_hits_set() {
    local f="$1" c
    for c in $(expand_list "$(tr -d ' ' < "$f" 2>/dev/null)"); do
        is_isolated "$c" && return 0
    done
    return 1
}

moved=0
for f in /proc/irq/[0-9]*/smp_affinity_list; do
    if irq_hits_set "$f"; then
        echo "$OTHER_CPUS" > "$f" 2>/dev/null && moved=$((moved+1)) || true
    fi
done
echo "[isolation] IRQs migrated off cpu{${ISOL_LIST}}: ${moved} (rest immovable/already off)"

# Report what is STILL on an isolated core. Silence here would read as success;
# a device whose affinity refused to move is exactly the thing that keeps
# injecting jitter into the tick, so name it.
stuck=""
for f in /proc/irq/[0-9]*/smp_affinity_list; do
    if irq_hits_set "$f"; then
        irq="${f#/proc/irq/}"; irq="${irq%/smp_affinity_list}"
        name="$(sed -nE "s/^ *${irq}:.*[0-9]+ +(.*)$/\1/p" /proc/interrupts | head -1)"
        stuck="${stuck}    IRQ ${irq}  $(cat "$f")  ${name}\n"
    fi
done
if [ -n "$stuck" ]; then
    echo "[isolation] STILL on cpu{${ISOL_LIST}} (immovable at runtime; the isolcpus=managed_irq"
    echo "[isolation] boot param should take these after the reboot — re-check with --status):"
    printf "%b" "$stuck"
else
    echo "[isolation] no IRQ remains on cpu{${ISOL_LIST}}"
fi

# ---- 4) Disable RT throttling (persist + immediate) -------------------------
# The scheduler throttles SCHED_FIFO/RR to sched_rt_runtime_us per period
# (default 950000/1000000 = 95%): an RT task running >950 ms in any 1 s window is
# descheduled ~50 ms. The servo loop sleeps most of each tick so it normally
# never hits this — EXCEPT (a) a busy catch-up burst after a stall and (b)
# servo.spin_slack_us busy-spin, either of which can approach 100% duty and get
# throttled into a 50 ms stall. Disable it (-1). Safe here: the isolated cores
# carry bounded RT loops as their only RT tasks, and there is no other high-duty
# RT task system-wide to run away without the throttle safety valve.
want_sysctl="kernel.sched_rt_runtime_us = -1"
if [ ! -f "$SYSCTL_DROPIN" ] || ! grep -qxF "$want_sysctl" "$SYSCTL_DROPIN"; then
    printf '# rb_servo: disable RT throttling for the dedicated-core RT control loops\n%s\n' \
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
echo "[isolation]   expect: online 0-$((NCPU-1)), isolated '${ISOL_LIST}', nohz_full '${ISOL_LIST}',"
echo "[isolation]           pid1 affinity ${OTHER_CPUS}, nproc $((NCPU - ${#ISOL_ARR[@]})),"
echo "[isolation]           governor performance on ${ISOL_LIST},"
echo "[isolation]           and no enp16s0f1-TxRx / nvidia IRQ left on cpu{${ISOL_LIST}}"
