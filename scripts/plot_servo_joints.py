#!/usr/bin/env python3
"""Plot per-joint q_sent / q_ref / q_actual from a 500 Hz rb_servo_server log.

One panel per joint (6 rows) per arm (columns), x axis in servo ticks -- one
tick is one control period, 2 ms on the 500 Hz rbpodo real stack.

    q_sent    the servo_j target rb_servo_server handed to the control box
    q_ref     the box's own reference readback (rbpodo sdata.jnt_ref)
    q_actual  encoder position

Requires the q_ref columns; refuses to plot q_sent in their place if the log
predates them, since the sent/ref gap is the whole point of the comparison.

`--html` emits an interactive standalone page instead of a PNG: box-zoom or
scroll on any panel and every panel follows, down to individual 2 ms ticks.

Usage:
  scripts/plot_servo_joints.py [logs/servo_log.csv] [-o out.png]
      [--arm left|right|both] [--all | --tick-start N --tick-stop M]
      [--relative] [--error] [--html [--open]]
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ARMS = ("left", "right")
DOF = 6
KINDS = ("sent", "ref", "actual")
# sent -> ref -> actual are separated by only a few ticks, so they overlap at
# any useful zoom. Draw order and dashing keep all three readable instead of
# letting the last one painted hide the others.
STYLE = {
    "sent": {"color": "#2f6fdb", "lw": 1.6, "ls": "-", "label": "q_sent", "zorder": 2},
    "actual": {"color": "#e07b1a", "lw": 1.6, "ls": "-", "label": "q_actual", "zorder": 3},
    "ref": {"color": "#129c6e", "lw": 1.3, "ls": (0, (4, 2)), "label": "q_ref", "zorder": 4},
}
DRAW_ORDER = ("sent", "actual", "ref")
PHASE_SHADE = {"planning": "#ffd98a", "executing": "#b7e4c7"}
MOVING_DEG_PER_TICK = 1e-4


class PlotError(Exception):
    """The log cannot back the requested plot."""


def load(path: Path, arms: list[str]) -> dict:
    required = [f"{a}_q_{k}_{j}" for a in arms for k in KINDS for j in range(DOF)]
    optional = ["init_motion_aggregate_status", "loop_start_time_ns"] + [f"{a}_mode" for a in arms]

    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise PlotError(f"{path} is empty") from exc
        index = {name: i for i, name in enumerate(header)}
        missing = [c for c in required if c not in index]
        if missing:
            raise PlotError(
                "log is missing required columns: " + ", ".join(missing[:6])
                + (" ..." if len(missing) > 6 else "")
                + "\n\nIf q_ref columns are absent this log predates the servo-logger "
                  "q_ref change. Rebuild (tools/build_stack.sh) and re-record."
            )
        present_optional = [c for c in optional if c in index]
        picks = [index[c] for c in required + present_optional]
        width = len(header)
        rows = [[row[p] for p in picks] for row in reader if len(row) == width]

    if not rows:
        raise PlotError(f"{path} has a header but no data rows")

    raw = np.array(rows, dtype=object)
    out: dict = {"n": len(rows)}
    for i, name in enumerate(required):
        out[name] = raw[:, i].astype(np.float64)
    for k, name in enumerate(present_optional, start=len(required)):
        col = raw[:, k]
        try:
            out[name] = col.astype(np.float64)
        except ValueError:
            out[name] = col.astype(str)
    return out


def motion_window(data: dict, arms: list[str], pad: int) -> tuple[int, int]:
    """Ticks where any commanded joint moved, padded, else the whole log."""
    n = data["n"]
    moved = np.zeros(n, dtype=bool)
    for arm in arms:
        sent = np.column_stack([data[f"{arm}_q_sent_{j}"] for j in range(DOF)])
        step = np.zeros(n)
        step[1:] = np.abs(np.diff(sent, axis=0)).max(axis=1)
        moved |= step > MOVING_DEG_PER_TICK
    idx = np.flatnonzero(moved)
    if idx.size == 0:
        return 0, n
    return max(0, int(idx[0]) - pad), min(n, int(idx[-1]) + pad + 1)


def phase_spans(data: dict, lo: int, hi: int) -> list[tuple[int, int, str]]:
    status = data.get("init_motion_aggregate_status")
    if status is None or status.dtype.kind not in "US":
        return []
    window = status[lo:hi]
    spans = []
    start = 0
    for i in range(1, window.size + 1):
        if i == window.size or window[i] != window[start]:
            label = str(window[start])
            if label in PHASE_SHADE:
                spans.append((lo + start, lo + i, label))
            start = i
    return spans


def plot(data: dict, arms: list[str], lo: int, hi: int, *, relative: bool,
         error: bool, title: str) -> plt.Figure:
    ticks = np.arange(lo, hi)
    spans = phase_spans(data, lo, hi)

    fig, axes = plt.subplots(
        DOF, len(arms), figsize=(7.2 * len(arms), 12.5),
        sharex=True, squeeze=False,
    )

    for col, arm in enumerate(arms):
        for j in range(DOF):
            ax = axes[j][col]
            for span_lo, span_hi, label in spans:
                ax.axvspan(span_lo, span_hi, color=PHASE_SHADE[label], alpha=0.35, lw=0)

            series = {k: data[f"{arm}_q_{k}_{j}"][lo:hi] for k in KINDS}
            if error:
                ax.axhline(0.0, color="#999999", lw=0.8, zorder=1)
                ax.plot(ticks, series["sent"] - series["ref"], color=STYLE["ref"]["color"],
                        lw=1.3, label="q_sent - q_ref", zorder=3)
                ax.plot(ticks, series["ref"] - series["actual"], color=STYLE["actual"]["color"],
                        lw=1.1, label="q_ref - q_actual", zorder=2)
                unit = "deg (error)"
            else:
                base = {k: (v[0] if relative else 0.0) for k, v in series.items()}
                if relative:
                    # Share one origin so the three curves stay comparable; a
                    # per-series origin would hide the very offset being studied.
                    origin = series["sent"][0]
                    base = {k: origin for k in KINDS}
                for kind in DRAW_ORDER:
                    style = STYLE[kind]
                    ax.plot(ticks, series[kind] - base[kind], color=style["color"],
                            lw=style["lw"], ls=style["ls"], label=style["label"],
                            zorder=style["zorder"])
                unit = "deg (from window start)" if relative else "deg"

            ax.grid(True, alpha=0.25, lw=0.6)
            ax.set_ylabel(f"J{j}  [{unit}]" if col == 0 else f"J{j}", fontsize=9)
            ax.tick_params(labelsize=8)
            if j == 0:
                ax.set_title(f"{arm} arm", fontsize=12, pad=10)
                handles, labels = ax.get_legend_handles_labels()
                if spans:
                    seen = []
                    for _, _, label in spans:
                        if label not in seen:
                            seen.append(label)
                    for label in seen:
                        handles.append(plt.Rectangle((0, 0), 1, 1,
                                                     color=PHASE_SHADE[label], alpha=0.35))
                        labels.append(f"init_motion: {label}")
                ax.legend(handles, labels, fontsize=8, ncol=2, loc="best")
            if j == DOF - 1:
                ax.set_xlabel("servo tick  (1 tick = 2 ms @ 500 Hz)", fontsize=10)

    fig.suptitle(title, fontsize=13, y=0.997)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    return fig


# Beyond this many plotted points the browser starts to struggle. We never
# decimate -- the whole point is per-tick resolution -- so warn and let the
# operator narrow the window instead of silently dropping samples.
HTML_POINT_WARN = 1_500_000


def render_html(data: dict, arms: list[str], lo: int, hi: int, *, relative: bool,
                error: bool, title: str, out: Path) -> int:
    """Interactive page: shared x across every panel, zoomable to single ticks."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise PlotError(f"--html needs plotly ({exc}); pip install plotly") from exc

    ticks = np.arange(lo, hi)
    ms = ticks * 2.0
    points = (hi - lo) * len(arms) * DOF * (2 if error else 3)
    if points > HTML_POINT_WARN:
        print(f"[plot-servo-joints] warning: {points:,} points in one page; "
              f"narrow with --tick-start/--tick-stop if the browser lags",
              file=sys.stderr)

    fig = make_subplots(
        rows=DOF, cols=len(arms), shared_xaxes=True,
        subplot_titles=[f"{arm} arm" for arm in arms] + [""] * (DOF - 1) * len(arms),
        vertical_spacing=0.018, horizontal_spacing=0.06,
    )

    hover = ("tick %{x}  (%{customdata:.0f} ms)<br>%{y:.4f} deg<extra>%{fullData.name}</extra>")
    series_defs = (("sent", "ref"), ("ref", "actual")) if error else DRAW_ORDER
    first = True
    for col, arm in enumerate(arms, start=1):
        for j in range(DOF):
            row = j + 1
            raw = {k: data[f"{arm}_q_{k}_{j}"][lo:hi] for k in KINDS}
            if error:
                traces = [
                    ("q_sent - q_ref", raw["sent"] - raw["ref"], STYLE["ref"]),
                    ("q_ref - q_actual", raw["ref"] - raw["actual"], STYLE["actual"]),
                ]
            else:
                origin = raw["sent"][0] if relative else 0.0
                traces = [(STYLE[k]["label"], raw[k] - origin, STYLE[k]) for k in DRAW_ORDER]
            for name, values, style in traces:
                fig.add_trace(
                    go.Scattergl(
                        x=ticks, y=values, customdata=ms, name=name,
                        mode="lines+markers",
                        line=dict(color=style["color"], width=1.6,
                                  dash="dash" if style["ls"] != "-" else "solid"),
                        # Markers stay invisible until the zoom separates ticks,
                        # at which point each 2 ms sample becomes individually
                        # readable without a second render.
                        marker=dict(color=style["color"], size=3),
                        legendgroup=name, showlegend=first,
                        hovertemplate=hover,
                    ),
                    row=row, col=col,
                )
            first = False
            fig.update_yaxes(title_text=f"J{j} [deg]", row=row, col=col,
                             title_font=dict(size=10), tickfont=dict(size=9))

    for span_lo, span_hi, label in phase_spans(data, lo, hi):
        for col in range(1, len(arms) + 1):
            for row in range(1, DOF + 1):
                # Label the span once per column instead of on all six panels.
                extra = dict(annotation_text=label, annotation_font_size=9) if row == 1 else {}
                fig.add_vrect(x0=span_lo, x1=span_hi, fillcolor=PHASE_SHADE[label],
                              opacity=0.3, line_width=0, layer="below",
                              row=row, col=col, **extra)

    # Ticks are integers, so keep the axis on whole numbers however far the
    # operator zooms -- plotly's auto algorithm would otherwise offer 0.5-tick
    # steps that mean nothing here. Labels stay on every row so any panel can be
    # read without scrolling to the bottom axis.
    fig.update_xaxes(showspikes=True, spikemode="across", spikethickness=1,
                     tickformat="d", rangemode="normal", showticklabels=True,
                     tickfont=dict(size=9))
    for col in range(1, len(arms) + 1):
        fig.update_xaxes(title_text="servo tick  (1 tick = 2 ms @ 500 Hz)",
                         row=DOF, col=col, title_font=dict(size=11))
    fig.update_layout(
        title=dict(text=title, font=dict(size=14)),
        height=260 * DOF, width=820 * len(arms),
        hovermode="x unified", dragmode="zoom",
        legend=dict(orientation="h", y=1.03, x=0),
        margin=dict(l=70, r=30, t=110, b=60),
    )
    div_id = "servo-joints"
    # Plotly has no "integer ticks only" option, so clamp dtick on every zoom:
    # once the visible span is short enough that auto ticking would go
    # fractional, pin it to whole ticks (2 ms) instead.
    post = """
    var gd = document.getElementById('%s');
    var busy = false;
    function integerTicks() {
      if (busy) { return; }
      var upd = {}, dirty = false;
      Object.keys(gd.layout).filter(function (k) { return k.indexOf('xaxis') === 0; })
        .forEach(function (key) {
          var ax = gd.layout[key];
          if (!ax || !ax.range) { return; }
          var span = Math.abs(ax.range[1] - ax.range[0]);
          var want = span <= 12 ? 1 : (span <= 30 ? 2 : (span <= 60 ? 5 : null));
          // Auto dtick reads back as undefined, not null. Normalise both ends of
          // the comparison or zooming back out relayouts forever.
          var cur = (ax.dtick === undefined || ax.dtick === null) ? null : ax.dtick;
          if (cur !== want) { upd[key + '.dtick'] = want; dirty = true; }
        });
      if (!dirty) { return; }
      busy = true;
      Plotly.relayout(gd, upd).then(function () { busy = false; },
                                    function () { busy = false; });
    }
    gd.on('plotly_relayout', integerTicks);
    integerTicks();
    """ % div_id
    fig.write_html(
        out, include_plotlyjs="inline", div_id=div_id, post_script=post,
        config={"scrollZoom": True, "displaylogo": False,
                "modeBarButtonsToAdd": ["v1hovermode", "toggleSpikelines"]},
    )
    return points


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("log", nargs="?", default="logs/servo_log.csv")
    p.add_argument("-o", "--out", default="", help="output PNG (default: <log stem>_joints.png)")
    p.add_argument("--arm", choices=("left", "right", "both"), default="both")
    p.add_argument("--all", action="store_true", help="plot every tick instead of the motion window")
    p.add_argument("--tick-start", type=int, default=-1)
    p.add_argument("--tick-stop", type=int, default=-1)
    p.add_argument("--pad-ticks", type=int, default=100, help="margin around the auto motion window")
    p.add_argument("--relative", action="store_true",
                   help="plot displacement from the window start instead of absolute angle")
    p.add_argument("--error", action="store_true",
                   help="plot q_sent-q_ref and q_ref-q_actual instead of the raw signals")
    p.add_argument("--dpi", type=int, default=140)
    p.add_argument("--html", action="store_true",
                   help="write an interactive zoomable HTML page instead of a PNG")
    p.add_argument("--open", action="store_true", help="open the HTML in a browser")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    arms = list(ARMS) if args.arm == "both" else [args.arm]
    path = Path(args.log)
    try:
        data = load(path, arms)
        if args.all:
            lo, hi = 0, data["n"]
        elif args.tick_start >= 0 or args.tick_stop >= 0:
            lo = max(0, args.tick_start if args.tick_start >= 0 else 0)
            hi = min(data["n"], args.tick_stop if args.tick_stop >= 0 else data["n"])
            if hi - lo < 2:
                raise PlotError(f"tick window [{lo}, {hi}) is empty")
        else:
            lo, hi = motion_window(data, arms, args.pad_ticks)
        kind = "sent-ref / ref-actual error" if args.error else (
            "displacement" if args.relative else "absolute angle")
        title = (f"{path.name} — per-joint {kind}   "
                 f"ticks {lo}..{hi - 1}  ({(hi - lo) * 2 / 1000:.2f} s @ 500 Hz)")
        suffix = "_error" if args.error else ("_relative" if args.relative else "_joints")
        ext = ".html" if args.html else ".png"
        out = Path(args.out) if args.out else path.with_name(path.stem + suffix + ext)
        if args.html:
            points = render_html(data, arms, lo, hi, relative=args.relative,
                                 error=args.error, title=title, out=out)
            extra = f", {points:,} points"
        else:
            fig = plot(data, arms, lo, hi, relative=args.relative,
                       error=args.error, title=title)
            fig.savefig(out, dpi=args.dpi)
            plt.close(fig)
            extra = ""
    except PlotError as exc:
        print(f"[plot-servo-joints] {exc}", file=sys.stderr)
        return 2

    print(f"[plot-servo-joints] {out}  ({data['n']} ticks in log, plotted {hi - lo}{extra})")
    if args.open:
        import webbrowser
        webbrowser.open(out.resolve().as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
