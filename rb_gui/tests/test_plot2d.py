from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from plotly import graph_objects as go
except Exception as exc:  # pragma: no cover - exercised only on missing optional dep.
    raise unittest.SkipTest(f"plotly is not importable: {type(exc).__name__}: {exc}")

from rb_servo_gui.plot2d import build_projection_figure


class Plot2DFigureTest(unittest.TestCase):
    def test_build_projection_figure_contains_left_traces_and_handles_empty_inputs(self) -> None:
        predicted = {
            "left": [(0.0, 0.0, 0.0), (0.1, 0.0, 0.1)],
            "right": None,
        }
        actual = {
            "left": [(0.0, 0.0, 0.0), (0.05, 0.0, 0.05)],
            "right": [],
        }

        fig = build_projection_figure(predicted, actual)

        self.assertIsInstance(fig, go.Figure)
        self.assertEqual(
            [annotation.text for annotation in fig.layout.annotations],
            ["L · XY", "L · XZ", "L · YZ", "R · XY", "R · XZ", "R · YZ"],
        )
        self.assertEqual(len(fig.data), 9)
        self.assertEqual(sum(1 for trace in fig.data if trace.legendgroup == "pred"), 3)
        self.assertEqual(sum(1 for trace in fig.data if trace.legendgroup == "actual"), 3)
        self.assertEqual(sum(1 for trace in fig.data if trace.legendgroup == "current"), 3)

        empty = build_projection_figure(
            {"left": None, "right": None},
            {"left": [], "right": []},
        )
        self.assertIsInstance(empty, go.Figure)
        self.assertEqual(len(empty.layout.annotations), 6)
        self.assertEqual(len(empty.data), 0)

    def test_xy_projection_uses_x_and_y_coordinates(self) -> None:
        fig = build_projection_figure(
            {"left": [(0.0, 0.2, 0.4), (0.1, 0.3, 0.5)], "right": None},
            {"left": [], "right": []},
        )

        xy_predicted = next(
            trace
            for trace in fig.data
            if trace.legendgroup == "pred" and getattr(trace, "xaxis", "x") == "x"
        )
        self.assertEqual(tuple(float(value) for value in xy_predicted.x), (0.0, 0.1))
        self.assertEqual(tuple(float(value) for value in xy_predicted.y), (0.2, 0.3))

    def test_equal_aspect_is_configured(self) -> None:
        fig = build_projection_figure(
            {"left": [(0.0, 0.0, 0.0), (0.1, 0.0, 0.1)], "right": None},
            {"left": [], "right": []},
        )

        self.assertEqual(fig.layout.yaxis.scaleanchor, "x")
        self.assertEqual(float(fig.layout.yaxis.scaleratio), 1.0)


if __name__ == "__main__":
    unittest.main()
