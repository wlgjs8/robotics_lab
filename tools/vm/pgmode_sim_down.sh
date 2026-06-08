#!/usr/bin/env bash
# Stop the rbpodo pgmode-simulation stack (rb_servo_server + rb_gui).
# Uses bracket-glob patterns so this script's own command line never matches
# the pkill/pgrep pattern (a real footgun when the path is in the pattern).
set -uo pipefail

# rb_gui (no sudo)
gui_pids=$(ps -eo pid,cmd | grep '[r]b_servo_gui.app' | awk '{print $1}')
if [[ -n "${gui_pids}" ]]; then
    echo "pgmode_sim_down: stopping rb_gui (${gui_pids})"
    kill ${gui_pids} 2>/dev/null || true
fi

# rb_servo_server (started under sudo). Match 'rb_servo_server --config' so this
# script (which contains 'rb_servo[_]server') is not matched.
srv_pids=$(ps -eo pid,cmd | grep 'rb_servo[_]server --config' | grep -v sudo | awk '{print $1}')
sudo_pids=$(ps -eo pid,cmd | grep 'rb_servo[_]server --config' | grep sudo | awk '{print $1}')
if [[ -n "${srv_pids}${sudo_pids}" ]]; then
    echo "pgmode_sim_down: stopping rb_servo_server (${sudo_pids} ${srv_pids})"
    sudo kill ${sudo_pids} ${srv_pids} 2>/dev/null || true
fi

sleep 2
remaining=$(ps -eo cmd | grep -E '[r]b_servo_gui.app|rb_servo[_]server --config' | grep -v grep | wc -l)
if [[ "${remaining}" -gt 0 ]]; then
    echo "pgmode_sim_down: forcing remaining (${remaining})"
    pids=$(ps -eo pid,cmd | grep 'rb_servo[_]server --config' | grep -v grep | awk '{print $1}')
    [[ -n "${pids}" ]] && sudo kill -9 ${pids} 2>/dev/null || true
    pids=$(ps -eo pid,cmd | grep '[r]b_servo_gui.app' | awk '{print $1}')
    [[ -n "${pids}" ]] && kill -9 ${pids} 2>/dev/null || true
fi
echo "pgmode_sim_down: done"
