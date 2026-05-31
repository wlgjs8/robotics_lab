# rb_servo_gui

`rb_gui` is the browser GUI for `rb_servo_server` state visualization and
operator-facing simulator controls. Real robot motion gates remain outside the
GUI.

## rbpodo Circle Live View

For rbpodo controller-simulation circle benchmarks, the GUI consumes two UDP
telemetry streams:

```text
rb_servo_server state fanout
  udp://127.0.0.1:50151 -> benchmark recorder
  udp://127.0.0.1:50161 -> rb_gui state receiver

rbpodo_circle_tracking_benchmark overlay
  udp://127.0.0.1:50261 -> rb_gui circle overlay receiver
```

The state stream contains robot/server telemetry such as `tcp_actual_stand`,
`tcp_ref_stand`, Cartesian gate status, and
`physical_motion_expected=false`. The overlay stream contains desired circle
geometry and running benchmark metrics. The overlay is not robot state and does
not carry commands.

Run the GUI against the fanout state port and overlay port:

```bash
tools/rbpodo_circle_gui.sh --profile stable
```

The stable profile listens on state port `50161`; the GENE-style stress
profile uses:

```bash
tools/rbpodo_circle_gui.sh --profile gene
```

The equivalent manual command is:

```bash
PYTHONPATH=rb_gui \
RB_GUI_STATE_BIND=0.0.0.0 \
RB_GUI_STATE_PORT=50161 \
RB_GUI_CIRCLE_OVERLAY_BIND=udp://0.0.0.0:50261 \
python3 -m rb_servo_gui.app
```

After installing this package, the console entry point is:

```bash
RB_GUI_STATE_BIND=0.0.0.0 \
RB_GUI_STATE_PORT=50161 \
RB_GUI_CIRCLE_OVERLAY_BIND=udp://0.0.0.0:50261 \
rb-servo-gui
```

Bare `python3 -m rb_servo_gui` is not the current module entry point unless a
future `rb_servo_gui/__main__.py` is added.

For Rainbow controller `pgmode` simulation, leave TCP display on `Auto`; it
selects `tcp_ref_stand` when the server recommends it. `tcp_actual_stand`
remains visible for physical-state inspection, but pgmode simulation tracking
should be scored against `tcp_ref_stand`.

`policy_runner` is a separate command source. The GUI does not route benchmark
commands through `policy_runner`; it only receives server state and benchmark
overlay telemetry for this workflow.
