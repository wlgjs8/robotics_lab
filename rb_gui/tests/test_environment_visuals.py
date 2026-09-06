"""Cell furniture (work table / stand riser) read out of the unified URDF.

The point of these tests is that the viewer owns NO furniture geometry: every
dimension and every pose comes from the URDF, so a measurement correction is a
URDF regeneration and never a GUI edit.  They also pin the two refusals that keep
a bad URDF from silently drawing furniture in the wrong place: a link that is not
rigidly fixed to the stand, and a visual with no usable geometry.
"""
from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rb_servo_gui.scene import (
    _ENVIRONMENT_DEFAULT_RGB,
    _add_environment_visuals,
    _environment_visuals_from_urdf,
    environment_riser_height_m,
    environment_riser_nominal_height_m,
    environment_table_top_z_m,
    set_environment_visible,
    set_riser_height_m,
)

URDF = """<?xml version="1.0"?>
<robot name="fixture">
  <link name="world"/>
  <link name="stand"/>
  <link name="env_work_table">
    <visual>
      <origin xyz="0.0 0.0 -0.009" rpy="0.0 0.0 0.0"/>
      <geometry><box size="1.2 0.8 0.018"/></geometry>
      <material name="table"><color rgba="0.55 0.45 0.30 1.0"/></material>
    </visual>
  </link>
  <link name="env_stand_riser">
    <visual>
      <geometry><box size="0.24 0.30 0.20"/></geometry>
    </visual>
  </link>
  <link name="env_tilted">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 1.5707963267948966"/>
      <geometry><box size="0.4 0.2 0.1"/></geometry>
    </visual>
  </link>
  <link name="env_floating"/>
  <link name="env_on_a_slider">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
  </link>
  <link name="env_no_geometry">
    <visual><geometry/></visual>
  </link>
  <joint name="stand_fixed" type="fixed">
    <parent link="world"/><child link="stand"/>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
  </joint>
  <joint name="env_work_table_fixed" type="fixed">
    <parent link="stand"/><child link="env_work_table"/>
    <origin xyz="0.6 0.0 -0.215" rpy="0.0 0.0 0.0"/>
  </joint>
  <joint name="env_stand_riser_fixed" type="fixed">
    <parent link="env_work_table"/><child link="env_stand_riser"/>
    <origin xyz="-0.6 0.0 0.1" rpy="0.0 0.0 0.0"/>
  </joint>
  <joint name="env_tilted_fixed" type="fixed">
    <parent link="stand"/><child link="env_tilted"/>
    <origin xyz="0.1 0.2 0.3" rpy="0.0 0.0 0.0"/>
  </joint>
  <joint name="env_on_a_slider_joint" type="prismatic">
    <parent link="stand"/><child link="env_on_a_slider"/>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="env_no_geometry_fixed" type="fixed">
    <parent link="stand"/><child link="env_no_geometry"/>
  </joint>
  <joint name="env_floating_fixed" type="fixed">
    <parent link="stand"/><child link="env_floating"/>
  </joint>
</robot>
"""


class _FakeHandle:
    def __init__(self, name, **kwargs):
        self.name = name
        self.kwargs = kwargs
        self.visible = True
        self.dimensions = kwargs.get("dimensions")
        self.position = kwargs.get("position")

    def remove(self):  # pragma: no cover - not exercised here
        pass


class _FakeScene:
    def __init__(self):
        self.boxes = []

    def add_box(self, name, **kwargs):
        handle = _FakeHandle(name, **kwargs)
        self.boxes.append(handle)
        return handle


class _FakeServer:
    def __init__(self):
        self.scene = _FakeScene()


def _write_fixture(tmp: Path) -> Path:
    path = tmp / "fixture.urdf"
    path.write_text(URDF)
    return path


class EnvironmentVisualsTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.urdf = _write_fixture(Path(self._tmp.name))
        self.entries = {e["name"]: e for e in _environment_visuals_from_urdf(self.urdf)}

    def tearDown(self):
        self._tmp.cleanup()

    def test_only_env_links_are_furniture(self):
        # `stand` and `world` are structure, not furniture, and must not be drawn here.
        self.assertNotIn("stand", self.entries)
        self.assertNotIn("world", self.entries)
        self.assertIn("env_work_table", self.entries)

    def test_box_dimensions_and_pose_come_from_the_urdf(self):
        table = self.entries["env_work_table"]
        self.assertEqual(table["shape"], "box")
        self.assertEqual(table["dimensions"], (1.2, 0.8, 0.018))
        # joint origin (0.6, 0, -0.215) composed with the visual origin (0, 0, -0.009).
        for got, want in zip(table["position"], (0.6, 0.0, -0.224)):
            self.assertAlmostEqual(got, want, places=9)

    def test_chained_fixed_joints_compose(self):
        # The riser hangs off the table, not off the stand: stand -> table (0.6, 0,
        # -0.215) -> riser (-0.6, 0, +0.1) must land back on the stand centre line.
        riser = self.entries["env_stand_riser"]
        for got, want in zip(riser["position"], (0.0, 0.0, -0.115)):
            self.assertAlmostEqual(got, want, places=9)

    def test_rotation_is_carried(self):
        tilted = self.entries["env_tilted"]
        w, x, y, z = tilted["wxyz"]
        self.assertAlmostEqual(w, math.cos(math.pi / 4), places=9)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        self.assertAlmostEqual(z, math.sin(math.pi / 4), places=9)

    def test_material_colour_is_used_and_defaulted(self):
        self.assertEqual(self.entries["env_work_table"]["rgb"], (140, 115, 76))
        self.assertEqual(self.entries["env_stand_riser"]["rgb"], _ENVIRONMENT_DEFAULT_RGB)

    def test_link_not_rigidly_fixed_to_the_stand_is_refused(self):
        # A prismatic joint would make the furniture pose configuration-dependent.
        entry = self.entries["env_on_a_slider"]
        self.assertIn("error", entry)
        self.assertNotIn("dimensions", entry)

    def test_visual_without_geometry_is_refused_not_guessed(self):
        entry = self.entries["env_no_geometry"]
        self.assertIn("error", entry)

    def test_link_without_visual_draws_nothing(self):
        self.assertNotIn("env_floating", self.entries)

    def test_missing_urdf_reports_instead_of_raising(self):
        entries = _environment_visuals_from_urdf(Path(self._tmp.name) / "nope.urdf")
        self.assertEqual(len(entries), 1)
        self.assertIn("error", entries[0])

    def test_scene_adds_boxes_and_toggles_visibility(self):
        server = _FakeServer()
        handles: dict = {}
        import rb_servo_gui.scene as scene

        original = scene._unified_urdf_path
        scene._unified_urdf_path = lambda: self.urdf
        try:
            _add_environment_visuals(server, handles)
        finally:
            scene._unified_urdf_path = original

        self.assertEqual(
            sorted(handles["environment_names"]),
            ["env_stand_riser", "env_tilted", "env_work_table"],
        )
        self.assertEqual(len(server.scene.boxes), 3)
        set_environment_visible(handles, False)
        self.assertTrue(all(not h.visible for h in server.scene.boxes))
        set_environment_visible(handles, True)
        self.assertTrue(all(h.visible for h in server.scene.boxes))
        # The two refusals above are surfaced, not swallowed.
        self.assertIn("environment_error_env_on_a_slider", handles)
        self.assertIn("environment_error_env_no_geometry", handles)


class RiserHeightTest(unittest.TestCase):
    """The riser is the one furniture dimension the operator retunes live."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.urdf = _write_fixture(Path(self._tmp.name))
        self.server = _FakeServer()
        self.handles: dict = {}
        import rb_servo_gui.scene as scene

        original = scene._unified_urdf_path
        scene._unified_urdf_path = lambda: self.urdf
        try:
            _add_environment_visuals(self.server, self.handles)
        finally:
            scene._unified_urdf_path = original

    def tearDown(self):
        self._tmp.cleanup()

    def _handle(self, name):
        return self.handles[f"environment_{name}"]

    def test_nominal_comes_from_the_urdf(self):
        self.assertAlmostEqual(environment_riser_nominal_height_m(self.handles), 0.20)
        self.assertAlmostEqual(environment_riser_height_m(self.handles), 0.20)
        # fixture: riser centre z -0.115, height 0.20 -> top -0.015, table top -0.215
        self.assertAlmostEqual(environment_table_top_z_m(self.handles), -0.215, places=9)

    def test_shortening_the_riser_keeps_its_top_and_lifts_the_tables(self):
        table_z0 = self._handle("env_work_table").position[2]
        self.assertEqual(set_riser_height_m(self.handles, 0.185), "")
        riser = self._handle("env_stand_riser")
        self.assertAlmostEqual(riser.dimensions[2], 0.185, places=9)
        # top face pinned to the stand base plate underside (-0.015 in this fixture)
        self.assertAlmostEqual(riser.position[2] + riser.dimensions[2] / 2.0, -0.015, places=9)
        # 15 mm shorter riser -> everything it carries comes up 15 mm
        self.assertAlmostEqual(self._handle("env_work_table").position[2], table_z0 + 0.015, places=9)
        self.assertAlmostEqual(environment_table_top_z_m(self.handles), -0.200, places=9)

    def test_repeated_adjustments_do_not_compound(self):
        set_riser_height_m(self.handles, 0.185)
        set_riser_height_m(self.handles, 0.185)
        set_riser_height_m(self.handles, 0.20)
        riser = self._handle("env_stand_riser")
        self.assertAlmostEqual(riser.dimensions[2], 0.20, places=9)
        self.assertAlmostEqual(riser.position[2], -0.115, places=9)
        self.assertAlmostEqual(self._handle("env_work_table").position[2], -0.224, places=9)

    def test_absurd_height_is_refused_and_nothing_moves(self):
        before = (self._handle("env_stand_riser").position, self._handle("env_work_table").position)
        for bad in (0.0, -0.2, 12.0, float("nan"), "tall"):
            self.assertNotEqual(set_riser_height_m(self.handles, bad), "")
        self.assertEqual(self._handle("env_stand_riser").position, before[0])
        self.assertEqual(self._handle("env_work_table").position, before[1])
        self.assertAlmostEqual(environment_riser_height_m(self.handles), 0.20)

    def test_missing_riser_link_reports(self):
        self.assertNotEqual(set_riser_height_m({}, 0.3), "")


if __name__ == "__main__":
    unittest.main()
