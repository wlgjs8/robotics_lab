from __future__ import annotations

import collections
import math
import time
from typing import Any, Mapping, Sequence

from .scene import _pose_position

Point3 = tuple[float, float, float]

_ARMS = (("left", "L"), ("right", "R"))
_PLANES = (
    ("XY", (0, 1), ("x", "y")),
    ("XZ", (0, 2), ("x", "z")),
    ("YZ", (1, 2), ("y", "z")),
)
_PRED_COLOR = "#2ec4a0"
_ACTUAL_COLOR = "#ff7a1a"
_CURRENT_COLOR = "#ffd23a"


def _plotly_modules() -> tuple[Any, Any] | tuple[None, None]:
    try:
        from plotly import graph_objects as go
        from plotly.subplots import make_subplots
    except Exception:
        return None, None
    return go, make_subplots


def _coerce_points(points: Sequence[Sequence[float]] | None) -> list[Point3]:
    if not points:
        return []
    out: list[Point3] = []
    for point in points:
        try:
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            out.append((x, y, z))
    return out


def _points_for_arm(source: Mapping[str, Sequence[Sequence[float]] | None], arm: str) -> list[Point3]:
    try:
        points = source.get(arm)
    except AttributeError:
        points = None
    return _coerce_points(points)


def _axis_id(index: int) -> str:
    return "x" if index == 1 else f"x{index}"


def build_projection_figure(
    predicted: Mapping[str, Sequence[Sequence[float]] | None] | None,
    actual: Mapping[str, Sequence[Sequence[float]] | None] | None,
) -> Any:
    go, make_subplots = _plotly_modules()
    if go is None or make_subplots is None:
        return None

    predicted = predicted or {}
    actual = actual or {}
    titles = [f"{label} · {plane}" for _, label in _ARMS for plane, _, _ in _PLANES]
    fig = make_subplots(rows=2, cols=3, subplot_titles=titles)
    legend_seen = {"pred": False, "actual": False, "current": False}

    for row, (arm, _) in enumerate(_ARMS, start=1):
        pred_points = _points_for_arm(predicted, arm)
        actual_points = _points_for_arm(actual, arm)
        for col, (_, indices, labels) in enumerate(_PLANES, start=1):
            x_index, y_index = indices
            x_label, y_label = labels
            if pred_points:
                fig.add_trace(
                    go.Scatter(
                        x=[point[x_index] for point in pred_points],
                        y=[point[y_index] for point in pred_points],
                        mode="lines+markers",
                        name="Sent predicted",
                        legendgroup="pred",
                        showlegend=not legend_seen["pred"],
                        line={"color": _PRED_COLOR, "width": 2},
                        marker={"color": _PRED_COLOR, "size": 4},
                    ),
                    row=row,
                    col=col,
                )
                legend_seen["pred"] = True
            if actual_points:
                fig.add_trace(
                    go.Scatter(
                        x=[point[x_index] for point in actual_points],
                        y=[point[y_index] for point in actual_points],
                        mode="lines",
                        name="Actual followed",
                        legendgroup="actual",
                        showlegend=not legend_seen["actual"],
                        line={"color": _ACTUAL_COLOR, "width": 2},
                    ),
                    row=row,
                    col=col,
                )
                legend_seen["actual"] = True
                current = actual_points[-1]
                fig.add_trace(
                    go.Scatter(
                        x=[current[x_index]],
                        y=[current[y_index]],
                        mode="markers",
                        name="Current",
                        legendgroup="current",
                        showlegend=not legend_seen["current"],
                        marker={
                            "color": _CURRENT_COLOR,
                            "size": 9,
                            "line": {"color": "#1a1a1a", "width": 1},
                        },
                    ),
                    row=row,
                    col=col,
                )
                legend_seen["current"] = True
            axis_index = (row - 1) * len(_PLANES) + col
            fig.update_xaxes(title_text=f"{x_label} (m)", row=row, col=col)
            fig.update_yaxes(
                title_text=f"{y_label} (m)",
                scaleanchor=_axis_id(axis_index),
                scaleratio=1,
                row=row,
                col=col,
            )

    fig.update_layout(
        template="plotly_dark",
        margin={"l": 40, "r": 20, "t": 52, "b": 36},
        legend={
            "orientation": "h",
            "yanchor": "bottom",
            "y": 1.02,
            "xanchor": "right",
            "x": 1.0,
            "font": {"size": 10},
        },
        font={"size": 10},
        hovermode="closest",
    )
    return fig


class TrajectoryPlot2D:
    def __init__(self, host: str, port: int, *, actual_window: int = 600, label: str = "Trajectory 2D"):
        self._server: Any | None = None
        self._handle: Any | None = None
        self._port = int(port)
        self._last_redraw = 0.0
        self._warned_update_error = False
        self._enabled = False
        self._disabled_reason: str | None = None
        try:
            window = int(actual_window)
        except (TypeError, ValueError, OverflowError):
            window = 600
        if window <= 0:
            window = 600
        self._actual: dict[str, collections.deque[Point3]] = {
            "left": collections.deque(maxlen=window),
            "right": collections.deque(maxlen=window),
        }

        fig = build_projection_figure({"left": None, "right": None}, {"left": [], "right": []})
        if fig is None:
            self._disabled_reason = "plotly import failed"
            return

        import viser

        server = viser.ViserServer(host=host, port=port, label=label)
        try:
            handle = server.gui.add_plotly(fig, aspect=1.6)
        except Exception:
            try:
                server.stop()
            except Exception:
                pass
            raise
        self._server = server
        self._handle = handle
        # viser reassigns to the next free port if the requested one is busy, so report the
        # ACTUAL bound port (server._port) — not the requested one — or the URL is wrong.
        self._port = int(getattr(server, "_port", port) or port)
        self._enabled = True

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def port(self) -> int:
        return self._port

    @property
    def disabled_reason(self) -> str | None:
        return self._disabled_reason

    def update(self, chunk_store: Any, state_store: Any) -> None:
        try:
            if not self._enabled or self._handle is None:
                return
            now = time.monotonic()
            if now - self._last_redraw < 0.18:
                return
            self._last_redraw = now

            snapshot = state_store.latest() if state_store is not None else None
            for arm in ("left", "right"):
                arm_state = getattr(snapshot, arm, None) if snapshot is not None else None
                if (
                    arm_state is not None
                    and getattr(arm_state, "tcp_actual_valid", False)
                    and getattr(arm_state, "tcp_actual_stand", None) is not None
                ):
                    self._actual[arm].append(_pose_position(arm_state.tcp_actual_stand))

            chunk = chunk_store.latest() if chunk_store is not None else None
            predicted = {
                "left": getattr(chunk, "left_positions", None) if chunk is not None else None,
                "right": getattr(chunk, "right_positions", None) if chunk is not None else None,
            }
            actual = {
                "left": list(self._actual["left"]),
                "right": list(self._actual["right"]),
            }
            figure = build_projection_figure(predicted, actual)
            if figure is not None:
                self._handle.figure = figure
        except Exception as exc:
            if not self._warned_update_error:
                print(f"[rb_gui] 2D trajectory viewer update skipped: {type(exc).__name__}: {exc}", flush=True)
                self._warned_update_error = True

    def stop(self) -> None:
        try:
            if self._server is not None:
                self._server.stop()
        except Exception:
            pass
