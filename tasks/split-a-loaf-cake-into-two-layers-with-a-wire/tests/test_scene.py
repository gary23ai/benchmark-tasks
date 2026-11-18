"""Check the scene file, the constants and the submission text against each other, without a simulator."""

import json
from pathlib import Path
import re
import unittest

import numpy as np

import conftest  # noqa: F401
from rdb_cake_wire import build_scene, cake_model as cake, grading

ROOT = Path(__file__).resolve().parents[1]
USD = ROOT / "rdb_cake_wire" / "cake.usda"
TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
YAM_GRIPPER_STROKE = .094     # yam_arena.constants: pads 94 mm apart when open
YAM_GRIP_LIMIT_N = 50.        # yam_arena.constants: the driver caps grip force at 50 N
FINGER_STIFFNESS = 5000.      # N/m, yam_arena.constants.GRIPPER


def block(text, kind, name):
    return re.search(rf'def {kind} "{name}" \((.*?)\n        \}}\n', text, flags=re.S).group(1)


def vector(text, key):
    return np.array([float(v) for v in re.search(rf"{key} = \(([^)]*)\)", text).group(1).split(",")])


class SceneFileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = USD.read_text()

    def test_file_matches_the_generator(self):
        self.assertEqual(self.text, build_scene.scene())

    def test_bodies(self):
        for name in cake.ACTORS:
            body = block(self.text, "Xform", name)
            self.assertIn("PhysicsRigidBodyAPI", body)
            self.assertAlmostEqual(float(re.search(r"physics:mass = ([\d.e-]+)", body).group(1)), cake.MASS[name], places=4)
            np.testing.assert_allclose(vector(body, "xformOp:translate") - (0., 0., cake.TABLE_TOP_WORLD_Z),
                                       cake.START[name], atol=1e-6)

    def test_cake_is_true_size_and_stands_on_the_stand(self):
        base, top_layer = block(self.text, "Xform", "base"), block(self.text, "Xform", "top_layer")
        np.testing.assert_allclose(2 * vector(base, "xformOp:scale"), (*cake.CAKE_SIZE[:2], cake.BASE_H), atol=1e-6)
        np.testing.assert_allclose(2 * vector(top_layer, "xformOp:scale"), (*cake.CAKE_SIZE[:2], cake.TOP_LAYER_H), atol=1e-6)
        self.assertAlmostEqual(cake.BASE_CENTRE[2] - cake.BASE_H / 2, cake.STAND_TOP)
        self.assertAlmostEqual(cake.TOP_LAYER_CENTRE[2] - cake.TOP_LAYER_H / 2, cake.BASE_CENTRE[2] + cake.BASE_H / 2)
        self.assertAlmostEqual(cake.BASE_MASS + cake.TOP_LAYER_MASS, .326, places=3)
        self.assertAlmostEqual(cake.SEAM_Z, .070)
        # The box colliders are not drawn; what is drawn is a loaf of meshes, none of which collides.
        for body in (base, top_layer):
            cube = re.search(r'def Cube "cake" \((.*?)\n            \}', body, flags=re.S).group(1)
            self.assertIn("PhysicsCollisionAPI", cube)
            self.assertIn('token visibility = "invisible"', cube)
            meshes = re.findall(r'def Mesh "\w+" \((.*?)\n            \}', body, flags=re.S)
            self.assertGreaterEqual(len(meshes), 4)
            for drawn in meshes:
                self.assertNotIn("PhysicsCollisionAPI", drawn)
                self.assertNotIn("visibility", drawn)

    def test_drawn_cake_is_a_loaf_inside_its_box_collider(self):
        rings, bands = build_scene.loaf()
        points = np.concatenate(rings)
        # Wholly inside the 185 x 90 x 60 box, so whatever touches the drawn cake has touched the collider.
        self.assertTrue(np.all(np.abs(points[:, :2]) <= cake.CAKE_HALF[:2] + 1e-9))
        self.assertGreaterEqual(points[:, 2].min(), 0.)
        self.assertLessEqual(points[:, 2].max(), cake.CAKE_SIZE[2])
        self.assertGreater(points[:, 2].max(), cake.CAKE_SIZE[2] - .0015)       # the ridge reaches the box's top
        # Where the wire cuts, the drawn cake is within 1.5 mm of the box all along both sides and both ends
        # (the model's cut passes through drawn cake), and within 6 mm where the upright corners are rounded.
        self.assertLess(build_scene.side_inset(cake.BASE_H), .0015)
        seam = rings[bands.index("crumb") + 1]                                   # the bottom of the score
        self.assertAlmostEqual(float(seam[0, 2]), cake.BASE_H)
        gap = np.min(cake.CAKE_HALF[:2] - np.abs(seam[:, :2]), axis=1)           # to the nearest face of the box
        straight = np.max(cake.CAKE_HALF[:2] - np.abs(seam[:, :2]), axis=1) > cake.CAKE_CORNER
        self.assertLess(gap[straight].max(), .0015 + cake.LINE_DEPTH)
        self.assertLess(gap.max(), .006)
        # A loaf: rounded upright corners, a shoulder, a top that rises 8 mm or more from shoulder to ridge.
        self.assertGreater(cake.CAKE_CORNER, .010)
        shoulder = cake.CAKE_SHOULDER_Z + cake.CAKE_SHOULDER
        self.assertGreater(cake.CAKE_CREST - shoulder, .008)
        self.assertGreater(shoulder, cake.BASE_H + .015)                         # the top layer is cake all the way across
        # The score is the only mark of the cut's height: a nick 1.2 mm wide in the crumb's colour, no band.
        self.assertEqual(bands.count("crumb"), 4)                               # two for the score, two for the split
        low, high = rings[bands.index("crumb")], rings[bands.index("crumb") + 2]
        self.assertAlmostEqual(float(high[0, 2] - low[0, 2]), cake.LINE_WIDTH)
        self.assertLessEqual(cake.LINE_WIDTH, .0015)
        self.assertNotIn("score_", self.text)
        # Browned on top, paler on the sides, palest in the crumb.
        lum = lambda name: float(np.dot((.2126, .7152, .0722), build_scene.COLOURS[name]))
        self.assertLess(lum("crust"), lum("crust_mid"))
        self.assertLess(lum("crust_mid"), lum("crust_edge"))
        self.assertLess(lum("crust_edge"), lum("skin"))
        self.assertLess(lum("skin"), lum("crumb"))
        self.assertEqual(bands[-1], "crumb")                                     # the split along the ridge
        self.assertEqual(set(bands[:3]), {"skin"})

    def test_layers_meet_at_the_score_and_show_crumb_when_parted(self):
        base, top_layer = block(self.text, "Xform", "base"), block(self.text, "Xform", "top_layer")
        for body, centre in ((base, cake.BASE_H / 2), (top_layer, cake.BASE_H + cake.TOP_LAYER_H / 2)):
            face = re.search(r'def Mesh "cut_face" \((.*?)\n            \}', body, flags=re.S).group(1)
            self.assertIn("</Fixture/Materials/crumb>", face)
            heights = {round(float(z) + centre, 6) for z in re.findall(r"\([^()]*, ([-\d.e]+)\)", face.split("point3f[] points")[1])}
            self.assertEqual(heights, {round(cake.BASE_H, 6)})
        self.assertNotIn("crust", base)                                          # the bottom layer has no top crust
        self.assertIn("crust_shell", top_layer)

    def test_every_mesh_is_well_formed(self):
        meshes = re.findall(r'def Mesh "(\w+)" \((.*?)uniform token subdivisionScheme = "none"', self.text, flags=re.S)
        self.assertEqual(len(meshes), 14)
        for name, body in meshes:
            counts = [int(v) for v in re.search(r"faceVertexCounts = \[(.*?)\]", body).group(1).split(",")]
            index = [int(v) for v in re.search(r"faceVertexIndices = \[(.*?)\]", body).group(1).split(",")]
            points = re.search(r"point3f\[\] points = \[(.*?)\]\n", body, flags=re.S).group(1).count("(")
            normals = re.search(r"normals = \[(.*?)\] \(", body, flags=re.S).group(1).count("(")
            self.assertEqual(sum(counts), len(index), name)
            self.assertEqual((min(index), max(index)), (0, points - 1), name)
            self.assertEqual(normals, points, name)
            self.assertIn('interpolation = "vertex"', body)
        self.assertEqual(self.text.count("{"), self.text.count("}"))
        self.assertNotIn("nan", self.text)

    def test_handles_lie_flat_pointing_away_from_the_robot(self):
        for side in ("left", "right"):
            body = block(self.text, "Xform", f"handle_{side}")
            self.assertIn('uniform token axis = "X"', body)
            self.assertIn(f"double height = {cake.HANDLE_LENGTH:.6g}", body)
            self.assertIn(f"double radius = {cake.HANDLE_RADIUS:.6g}", body)
            self.assertAlmostEqual(cake.HANDLE_START[side][2], cake.HANDLE_RADIUS)
        self.assertAlmostEqual(cake.HANDLE_START["left"][1], -cake.HANDLE_START["right"][1])
        span = np.linalg.norm(cake.HANDLE_START["left"] - cake.HANDLE_START["right"])
        self.assertLess(span, cake.WIRE_LENGTH)            # the wire starts slack

    def test_stand_is_fixed(self):
        static = self.text.split('def Scope "Static"')[1].split('def Scope "Materials"')[0]
        self.assertNotIn("PhysicsRigidBodyAPI", static)
        self.assertIn("PhysicsCollisionAPI", static)
        np.testing.assert_allclose(2 * vector(static, "xformOp:scale"), cake.STAND_SIZE, atol=1e-6)
        np.testing.assert_allclose(vector(static, "xformOp:translate") - (0., 0., cake.TABLE_TOP_WORLD_Z),
                                   cake.STAND_CENTRE, atol=1e-6)

    def test_stand_is_drawn_as_a_wooden_block_with_a_mat_on_it(self):
        static = self.text.split('def Scope "Static"')[1].split('def Scope "Materials"')[0]
        cube = re.search(r'def Cube "stand" \((.*?)\n        \}', static, flags=re.S).group(1)
        self.assertIn('token visibility = "invisible"', cube)
        drawn = dict(re.findall(r'def Mesh "(\w+)" \((.*?)\n        \}', static, flags=re.S))
        self.assertEqual(set(drawn), {"block", "block_top", "mat_edge", "mat"})
        for name, body in drawn.items():
            self.assertNotIn("PhysicsCollisionAPI", body)
            self.assertIn("</Fixture/Materials/mat>" if name.startswith("mat") else "</Fixture/Materials/stand>", body)
            xyz = np.array([[float(v) for v in p.split(",")] for p in
                            re.findall(r"\(([^()]*)\)", body.split("point3f[] points")[1])])
            xyz -= (*cake.STAND_CENTRE[:2], cake.TABLE_TOP_WORLD_Z)
            # Nothing drawn is outside the collider, and the mat's top is the collider's top.
            self.assertTrue(np.all(np.abs(xyz[:, :2]) <= cake.STAND_SIZE[:2] / 2 + 1e-6), name)
            self.assertLessEqual(xyz[:, 2].max(), cake.STAND_TOP + 1e-6)
            if name == "mat":
                np.testing.assert_allclose(xyz[:, 2], cake.STAND_TOP, atol=1e-6)
                np.testing.assert_allclose(2 * np.abs(xyz[:, :2]).max(axis=0), cake.MAT_SIZE, atol=1e-6)
        # The cake stands wholly on the mat, and the wood shows as a rim round it.
        self.assertTrue(np.all(cake.MAT_SIZE > cake.CAKE_SIZE[:2] + .04))
        self.assertTrue(np.all(cake.MAT_SIZE < cake.STAND_SIZE[:2]))
        lum = lambda name: float(np.dot((.2126, .7152, .0722), build_scene.COLOURS[name]))
        self.assertGreater(lum("stand"), 2 * lum("mat"))                         # brown wood, not a black box

    def test_film_is_colour_only(self):
        film = re.search(r'def Cube "film" \((.*?)\n        \}', self.text, flags=re.S).group(1)
        self.assertNotIn("PhysicsCollisionAPI", film)
        size, centre = 2 * vector(film, "xformOp:scale"), vector(film, "xformOp:translate")
        np.testing.assert_allclose(size[:2], (cake.FILM_X[1] - cake.FILM_X[0], cake.FILM_Y[1] - cake.FILM_Y[0]), atol=1e-6)
        np.testing.assert_allclose(centre[:2], (sum(cake.FILM_X) / 2, sum(cake.FILM_Y) / 2), atol=1e-6)
        self.assertLessEqual(centre[2] + size[2] / 2 - cake.TABLE_TOP_WORLD_Z, .0005 + 1e-9)   # under the wire and handles

    def test_friction(self):
        self.assertIn(f"rel material:binding:physics = </Fixture/Materials/{build_scene.CAKE_COLLIDER_MATERIAL}>",
                      block(self.text, "Xform", "base"))
        for name, value in ((build_scene.CAKE_COLLIDER_MATERIAL, cake.CAKE_FRICTION), ("stand", cake.CAKE_FRICTION),
                            ("handle", cake.HANDLE_FRICTION)):
            material = self.text.split(f'def Material "{name}"')[1].split("def Shader")[0]
            self.assertIn(f"physics:staticFriction = {value:.6g}", material)
            self.assertIn('frictionCombineMode = "average"', material)

    def test_no_third_party_files(self):
        self.assertNotIn("@", self.text)                    # no texture or mesh references
        self.assertEqual(sorted(p.suffix for p in (ROOT / "rdb_cake_wire").iterdir() if p.is_file() and p.suffix != ".py"),
                         [".usda"])


class RobotFitTest(unittest.TestCase):
    def test_true_size_objects_fit_the_gripper_and_its_strength(self):
        self.assertLess(2 * cake.HANDLE_RADIUS, YAM_GRIPPER_STROKE)
        squeeze = FINGER_STIFFNESS * cake.HANDLE_RADIUS       # fully closed command on a 12 mm handle
        self.assertLess(squeeze, YAM_GRIP_LIMIT_N)
        self.assertLess(squeeze, 40.)
        # Wire tension at the demonstration's 25 degree wrap: cut force / (2 sin 25) = 1.7 N at yield.
        tension = cake.CUT_FORCE / (2 * np.sin(np.radians(25.)))
        self.assertLess(tension, .5 * squeeze)

    def test_block_holds_against_a_slow_cut_by_friction_alone(self):
        self.assertAlmostEqual(cake.CUT_FORCE, 1.44, places=2)
        self.assertAlmostEqual(cake.CAKE_SLIDE_FORCE, 3.2, places=1)
        self.assertLess(cake.CUT_FORCE, .5 * cake.CAKE_SLIDE_FORCE)
        # It slides before it tips: the cut's pull acts 30 mm up, the weight 92 mm from the edge.
        weight = (cake.BASE_MASS + cake.TOP_LAYER_MASS) * 9.81
        self.assertGreater(weight * cake.CAKE_HALF[0] / cake.BASE_H, cake.CAKE_SLIDE_FORCE)

    def test_yield_follows_from_the_stated_strength_and_wire(self):
        self.assertAlmostEqual(10.5 * cake.CAKE_SHEAR_STRENGTH * cake.WIRE_DIAMETER, cake.CAKE_YIELD, delta=1.)


class TaskTextTest(unittest.TestCase):
    def test_format(self):
        required = {"title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"}
        self.assertTrue(required <= set(TASK))
        self.assertNotIn("media", TASK)
        self.assertIn(TASK["category"], {"Impact on jobs", "Impact on AI timelines", "Safety and alignment",
                                         "Societal relatability", "Other"})
        self.assertEqual(TASK["scores"], "Dream-1 run: 0.0; demo: 1.0")

    def test_text_matches_the_code(self):
        try:
            from rdb_cake_wire import fixed_scene as fixture
        except ImportError as error:  # pragma: no cover
            self.skipTest(f"needs yam_arena: {error}")
        self.assertEqual(TASK["instruction"], fixture.INSTRUCTION)
        self.assertEqual(TASK["run_limit"], f"{fixture.MAX_STEPS:,} steps ({fixture.MAX_STEPS // fixture.CONTROL_HZ} seconds "
                                            f"at {fixture.CONTROL_HZ} Hz)")
        self.assertEqual((grading.CONTROL_HZ, grading.HOLD_S), (fixture.CONTROL_HZ, fixture.SUCCESS_HOLD_S))
        fidelity = TASK["fidelity"]
        for phrase in (f"chain of {fixture.WIRE_SEGMENTS} stretch-only springs", f"{cake.CAKE_YIELD:.0f} N per metre",
                       f"{cake.CUT_FORCE:.1f} N across the cake", f"{cake.CAKE_VISCOUS:.0f} N s/m2",
                       f"{1000 * fixture.CUT_CELL:.1f} mm grid", f"{fixture.WELD_STRENGTH:.0f} N",
                       f"{fixture.WIRE_STIFFNESS:,.0f} N/m", f"friction {cake.CAKE_FRICTION:.1f}", f"{cake.CAKE_SLIDE_FORCE:.1f} N", "240 Hz", "20 Hz"):
            self.assertIn(phrase, fidelity)
        for phrase in (f"no spring carries more than {fixture.END_PULL_LIMIT:.0f} N",
                       f"none below {1000 * fixture.HANDLE_REST_PULL:.0f} mN", "a tenth of the wire's pull or less",
                       "lies wholly inside the boxes", "within about 2 mm", "(up to 6 mm)"):
            self.assertIn(phrase, fidelity)
        self.assertAlmostEqual(fixture.LOOSE_STIFFNESS / fixture.WIRE_STIFFNESS, .1, delta=.005)
        nodes = fixture.wire_start()
        self.assertIn(f"x = {1000 * nodes[:, 0].max():.0f}", TASK["setup"])

    def test_setup_lists_the_modelled_objects(self):
        mm = lambda v: f"{1000 * v:g}"
        setup = TASK["setup"]
        size, stand = cake.CAKE_SIZE, cake.STAND_SIZE
        for phrase in (
            f"{mm(stand[0])} (x) x {mm(stand[1])} (y) x {mm(stand[2])} tall",
            f"centred at ({mm(cake.STAND_CENTRE[0])}, 0)",
            f"about {mm(size[0])} long x {mm(size[1])} wide x {mm(size[2])} tall",
            f"({1000 * cake.CAKE_MASS:.0f} g)",
            f"{mm(cake.BASE_H)} above the mat ({mm(cake.SEAM_Z)} above the table)",
            f"{mm(cake.WIRE_LENGTH)} of fine stainless steel wire", f"about {mm(cake.WIRE_DIAMETER)} thick",
            f"about {mm(cake.HANDLE_LENGTH)} long, {mm(2 * cake.HANDLE_RADIUS)} thick and {1000 * cake.HANDLE_MASS:g} g",
            f"({mm(cake.HANDLE_START['left'][0])}, +{mm(cake.HANDLE_START['left'][1])})",
            f"({mm(cake.HANDLE_START['right'][0])}, -{mm(-cake.HANDLE_START['right'][1])})",
            f"about {cake.CAKE_SLIDE_FORCE:.0f} N", f"about {cake.CUT_FORCE:.1f} N",
            f"{mm(cake.FILM_X[1] - cake.FILM_X[0])} (x) x {mm(cake.FILM_Y[1] - cake.FILM_Y[0])} (y)",
            f"from x = {mm(cake.FILM_X[0])} to {mm(cake.FILM_X[1])}",
            f"drawn {mm(cake.WIRE_DRAWN_DIAMETER)} thick",
            f"cut to {mm(cake.MAT_SIZE[0])} x {mm(cake.MAT_SIZE[1])}",
            "about 1 wide and 1 deep", "a fine pale line in the golden side, not as a gap",
        ):
            self.assertIn(phrase, setup)

    def test_scoring_text_matches_the_scorer(self):
        scoring = TASK["scoring"]
        plan_cm2 = 1e4 * cake.CAKE_SIZE[0] * cake.CAKE_SIZE[1]
        for phrase in (
            f"less than {1000 * grading.STARTED_SIDE_CUT:.0f} mm along either long side",
            f"{100 * grading.THROUGH:.0f} percent of the cake's plan",
            f"more than {100 * grading.SPOILED:.0f} percent of the cake's plan ({grading.SPOILED * plan_cm2:.0f} cm2)",
            f"within {1000 * grading.CAKE_SHIFT:.0f} mm and {grading.CAKE_TURN_DEG:.0f} degrees of its chalk marks",
            f"within {1000 * grading.TOP_LAYER_SHIFT:.0f} mm and {grading.TOP_LAYER_TILT_DEG:.0f} degrees of where it was cut",
            f"at least {100 * grading.CLEAR:.0f} cm from both handles and from the cake",
            f"last {grading.HOLD_S} s", "Possible scores: 0, 0.25, 0.5, 0.75, 1.0",
            f"{1000 * (cake.TOP_LAYER_H - .005):.0f} to {1000 * (cake.TOP_LAYER_H + .005):.0f} mm thick", f"more than {grading.TOUCH_N} N", "or if the cake was touched",
        ):
            self.assertIn(phrase, scoring)

    def test_blind_review_text(self):
        for name in ("task.json", "README.md"):
            text = (ROOT / name).read_text()
            for word in ("/Users/", "/home/", "Robot-Discovery", "university", "University", "our lab", "Hopkins", "clay", "pottery"):
                self.assertNotIn(word, text, f"{word} in {name}")


if __name__ == "__main__":
    unittest.main()
