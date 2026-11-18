"""Check the written scene file against the cup model, without a simulator."""

import math
from pathlib import Path
import re
import unittest

import numpy as np

from rdb_nested_cups import build_scene, cup_model as cup

USD = Path(build_scene.__file__).with_name("cups.usda")
YAM_GRIPPER_STROKE = 0.094  # yam_arena.constants: pads 94 mm apart when open
YAM_GRIP_LIMIT_N = 50.  # yam_arena.robot: the driver caps grip force at 50 N


def panels(text, body, layer="wall"):
    """(centre, WXYZ quaternion, half extents) of each collision panel of one cup: the compliant fit layer
    ("wall") or the rigid core ("core")."""
    found = []
    for name, block in re.findall(r'def Cube "(\w+)" \((.*?)\n            \}', text, flags=re.S):
        if not name.startswith(f"{body}_{layer}"):
            continue
        value = {key: np.array([float(v) for v in numbers.split(",")]) for key, numbers in
                 re.findall(r"xformOp:(orient|scale|translate) = \(([^)]*)\)", block)}
        found.append((value["translate"], value["orient"], value["scale"]))
    return found


def rotate(quat_wxyz, vector):
    w, x, y, z = quat_wxyz
    u = np.array((x, y, z))
    return vector + 2. * w * np.cross(u, vector) + 2. * np.cross(u, np.cross(u, vector))


class SceneFileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = USD.read_text()

    def test_file_matches_the_generator(self):
        self.assertEqual(self.text, build_scene.text())

    def test_true_size_and_mass(self):
        self.assertAlmostEqual(2 * cup.RIM_RADIUS, .080)
        self.assertAlmostEqual(cup.HEIGHT, .081)
        self.assertAlmostEqual(cup.MASS, .030)
        for body in cup.CUPS:
            start = self.text.index(f'def Xform "{body}"')
            end = min(i for i in (self.text.find('def Xform "', start + 10), self.text.index('def Scope "Static"')) if i > 0)
            masses = [float(m) for m in re.findall(r"physics:mass = ([0-9.e-]+)", self.text[start:end])]
            self.assertAlmostEqual(masses[0], cup.MASS)  # the body's own mass
            self.assertAlmostEqual(sum(masses[1:]), cup.MASS, places=7)  # its 24 panels and base disc
            self.assertEqual(len(panels(self.text, body)), cup.SEGMENTS)

    def test_rim_fits_the_open_gripper(self):
        self.assertGreaterEqual(YAM_GRIPPER_STROKE - 2 * cup.RIM_RADIUS, .012)  # 7 mm spare each side

    def test_panels_follow_the_stated_cone(self):
        for body in cup.CUPS:
            for centre, quat, half in panels(self.text, body):
                outward = rotate(quat, np.array((1., 0., 0.)))
                along = rotate(quat, np.array((0., 0., 1.)))
                self.assertAlmostEqual(math.degrees(math.acos(along[2])), math.degrees(cup.HALF_ANGLE), places=4)
                for end in (-1., 1.):
                    point = centre + half[0] * outward + end * half[2] * along  # middle of the outside face's edge
                    self.assertAlmostEqual(float(np.hypot(point[0], point[1])), cup.outside_radius(point[2]), places=6)
                # A panel's outside corners must not stick out past its neighbours' faces.
                narrowest = cup.outside_radius(centre[2] - half[2] * along[2])
                self.assertLessEqual(half[1], narrowest * math.tan(math.pi / cup.SEGMENTS) + 1e-9)

    def test_cup_stands_on_its_base_disc(self):
        for body in cup.CUPS:
            lowest = min((centre + sx * half[0] * rotate(quat, np.array((1., 0., 0.)))
                          - half[2] * rotate(quat, np.array((0., 0., 1.))))[2]
                         for centre, quat, half in panels(self.text, body) for sx in (-1., 1.))
            self.assertGreater(lowest, 0.)

    def test_bonded_start_leaves_the_walls_clear_of_each_other(self):
        outer_z = float(re.search(r'def Xform "outer".*?translate = \(([^)]*)\)', self.text, re.S).group(1).split(",")[2])
        inner_z = float(re.search(r'def Xform "inner".*?translate = \(([^)]*)\)', self.text, re.S).group(1).split(",")[2])
        pitch = inner_z - outer_z
        self.assertAlmostEqual(pitch, cup.SEATED_PITCH, places=7)
        self.assertGreater(pitch, cup.FREE_PITCH)
        centre, quat, half = panels(self.text, "outer")[0]
        normal = rotate(quat, np.array((1., 0., 0.)))
        outer_inside_face = (centre - half[0] * normal) @ normal
        centre, quat, half = panels(self.text, "inner")[0]
        inner_outside_face = (centre + (0., 0., pitch) + half[0] * normal) @ normal
        gap = outer_inside_face - inner_outside_face
        self.assertGreater(gap, .00002)  # rigid walls, not touching
        self.assertLess(gap, .0001)
        # The inner cup's base disc and its solid inside stay clear of the outer cup's wall.
        cavity = cup.outside_radius(pitch) - cup.WALL / math.cos(cup.HALF_ANGLE)
        self.assertGreater(cavity - build_scene.BASE_DISC_RADIUS, .001)

    def test_the_fit_is_one_breakable_fixed_joint_between_the_cups(self):
        joints = re.findall(r'def Physics(\w+)Joint "(\w+)"\s*\{(.*?)\n        \}', self.text, re.S)
        self.assertEqual([(kind, name) for kind, name, _ in joints], [("Fixed", "fit")])
        body = joints[0][2]
        self.assertIn("rel physics:body0 = </Fixture/Actors/outer>", body)
        self.assertIn("rel physics:body1 = </Fixture/Actors/inner>", body)
        self.assertIn(f"float physics:breakForce = {cup.FIT_BREAK_FORCE_N:.9g}", body)
        self.assertIn(f"float physics:breakTorque = {cup.FIT_BREAK_TORQUE_NM:.9g}", body)
        # The two anchor points coincide at the start pose.
        z0 = float(re.search(r"localPos0 = \(([^)]*)\)", body).group(1).split(",")[2])
        z1 = float(re.search(r"localPos1 = \(([^)]*)\)", body).group(1).split(",")[2])
        self.assertAlmostEqual(z0, z1 + cup.SEATED_PITCH, places=7)
        # The bond holds many cup weights, a straight pull at the rim breaks it by force before moment, and
        # the rim pinch (30 N per finger) can carry the pull with room to spare.
        self.assertGreater(cup.FIT_BREAK_FORCE_N, 15 * cup.MASS * 9.81)
        self.assertLess(cup.FIT_BREAK_FORCE_N * (cup.RIM_RADIUS - cup.BAND_THICKNESS / 2.), cup.FIT_BREAK_TORQUE_NM)
        self.assertLess(cup.FIT_BREAK_FORCE_N + 5., 2 * cup.FRICTION * 5000. * cup.BAND_THICKNESS / 2.)

    def test_every_collider_is_rigid_and_probe_mode_is_off(self):
        self.assertNotIn("compliant", self.text)
        self.assertNotIn("probe", self.text)
        self.assertFalse(cup.PROBE)

    def test_inner_cup_is_solid_below_its_rim_band(self):
        plugs = re.findall(r'def Cylinder "inner_plug(\d)" \((.*?)\n            \}', self.text, re.S)
        self.assertEqual(len(plugs), cup.PLUG_LAYERS)
        top = 0.
        for _, body in plugs:
            height = float(re.search(r"double height = ([0-9.e-]+)", body).group(1))
            radius = float(re.search(r"double radius = ([0-9.e-]+)", body).group(1))
            z = float(re.search(r"translate = \(([^)]*)\)", body).group(1).split(",")[2])
            inside = cup.outside_radius(z - height / 2.) - cup.WALL / math.cos(cup.HALF_ANGLE)
            self.assertLess(radius, inside)  # within the wall's inside surface
            self.assertGreater(radius, inside - .001)
            top = max(top, z + height / 2.)
        self.assertAlmostEqual(top, cup.PLUG_TOP, places=6)
        # A pinching fingertip goes 13 mm into the cup; the solid starts deeper than that.
        self.assertGreater(cup.HEIGHT - top, .0145)
        self.assertNotIn("outer_plug", self.text)

    def test_inner_rim_band_is_rigid_thick_inward_and_clear_of_the_outer_cup(self):
        band = panels(self.text, "inner", "band")
        self.assertEqual(len(band), cup.SEGMENTS)
        self.assertEqual(panels(self.text, "outer", "band"), [])
        for centre, quat, half in band:
            outward, along = rotate(quat, np.array((1., 0., 0.))), rotate(quat, np.array((0., 0., 1.)))
            self.assertAlmostEqual(2 * half[0], cup.BAND_THICKNESS)
            for end in (-1., 1.):  # the outside face is the cup's true surface
                point = centre + half[0] * outward + end * half[2] * along
                self.assertAlmostEqual(float(np.hypot(point[0], point[1])), cup.outside_radius(point[2]), places=6)
            self.assertLess(abs((centre + half[0] * outward + half[2] * along)[2] - cup.HEIGHT), .0006)  # reaches the rim
            self.assertLessEqual(half[1], cup.outside_radius(cup.BAND_START) * math.tan(math.pi / cup.SEGMENTS) + 1e-9)
        # Rigid material on the band.
        self.assertIn("rel material:binding:physics = </Fixture/Materials/inner_base>",
                      self.text[self.text.index('def Cube "inner_band0"'):][:500])
        # The band starts above the outer cup's rim, and the compliant wall below it still covers the nesting.
        outer_rim = cup.OUTER_START[2] + cup.HEIGHT
        self.assertGreater(cup.INNER_START[2] + cup.BAND_START - outer_rim, .0012)
        wall_top = max((c + h[2] * rotate(q, np.array((0., 0., 1.))))[2] for c, q, h in panels(self.text, "inner"))
        self.assertGreaterEqual(cup.INNER_START[2] + wall_top, outer_rim)
        self.assertLess(wall_top, cup.BAND_START)
        # The outer cup's wall is full height.
        top = max((c + h[2] * rotate(q, np.array((0., 0., 1.))))[2] for c, q, h in panels(self.text, "outer"))
        self.assertAlmostEqual(top, cup.HEIGHT, places=4)

    def test_model_reproduces_the_published_stack_height(self):
        six_stacked = cup.HEIGHT + 5 * cup.FREE_PITCH
        self.assertGreater(six_stacked, .160)  # the retailer lists the six-pack as 16 to 17 cm tall
        self.assertLess(six_stacked, .172)


if __name__ == "__main__":
    unittest.main()
