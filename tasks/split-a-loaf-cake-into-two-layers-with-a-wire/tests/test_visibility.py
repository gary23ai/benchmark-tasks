"""What the three policy cameras (320 x 176) can see of the handles, the wire and the marked line.

Pinhole projection with the station's calibrated cameras; no renderer. It shows sizes in pixels and the
contrast of the authored colours, not lighting, shadows or anti-aliasing.
"""

import unittest

import numpy as np

import conftest  # noqa: F401

try:
    from yam_arena import ik
    from yam_arena.constants import READY
    import camera_views as views
    from rdb_cake_wire import build_scene, cake_model as cake, fixed_scene as fixture, scripted, skin
except ImportError as error:  # pragma: no cover
    raise unittest.SkipTest(f"needs yam_arena: {error}")

WIRE_COLOUR = (.78, .79, .80)      # skin.WireSkin's default


def luminance(rgb):
    return float(np.dot((.2126, .7152, .0722), rgb))


def contrast(a, b):
    """Contrast ratio of two linear colours, 1 (none) to 21 (white on black)."""
    hi, lo = sorted((luminance(a), luminance(b)), reverse=True)
    return (hi + .05) / (lo + .05)


def wrist_at(point, rotation):
    joints, error, _ = ik.solve(np.asarray(point), ik.matrix_to_quat_xyzw(rotation), np.asarray(READY, float),
                                fixture.BASES["left"], iters=300)
    assert error < 1e-3
    return views.wrist("left", joints, fixture.BASES["left"])


def sizes():
    """The numbers quoted in task.json and the README, in pixels of a 320 x 176 image."""
    top = views.overhead()
    handle = cake.HANDLE_START["left"]
    end = np.array((cake.HANDLE_LENGTH / 2, 0., 0.))
    middle = fixture.wire_start()[fixture.WIRE_SEGMENTS // 2] * (1., 1., 0.)
    near = np.array((cake.CAKE_CENTRE[0] - cake.CAKE_HALF[0], 0., cake.SEAM_Z))
    over = wrist_at(scripted.GRASP["left"] + (0., 0., .065), scripted.DOWN)
    grasp = wrist_at(scripted.GRASP["left"], scripted.DOWN)
    beside_wire = handle + (0., -.030, 0.)
    return {
        "top_handle_across": top.best_thickness(handle, (1., 0., 0.), 2 * cake.HANDLE_RADIUS),
        "top_handle_length": top.length(handle - end, handle + end),
        "top_wire_slack": top.best_thickness(middle, (0., 1., 0.), cake.WIRE_DRAWN_DIAMETER),
        "top_wire_at_line": top.best_thickness((.40, .12, cake.SEAM_Z), (0., 1., 0.), cake.WIRE_DRAWN_DIAMETER),
        "top_real_wire": top.best_thickness(middle, (0., 1., 0.), cake.WIRE_DIAMETER),
        "top_line_near_face": top.thickness(near, (0., 0., 1.), cake.LINE_WIDTH),
        "top_top_layer_near_face": top.thickness(near + (0., 0., cake.TOP_LAYER_H / 2), (0., 0., 1.), cake.TOP_LAYER_H),
        "top_block_length": top.length(cake.CAKE_CENTRE + (-.09, 0., .06), cake.CAKE_CENTRE + (.09, 0., .06)),
        "top_block_width": top.length(cake.CAKE_CENTRE + (0., -.04, .06), cake.CAKE_CENTRE + (0., .04, .06)),
        "top_split_across": top.thickness(cake.CAKE_CENTRE + (0., 0., cake.CAKE_CREST - cake.CAKE_HALF[2]), (0., 1., 0.),
                                          sum(cake.CAKE_SPLIT_HALF_WIDTH)),
        "top_split_length": top.length(cake.CAKE_CENTRE + (-.04, 0., .0285), cake.CAKE_CENTRE + (.04, 0., .0285)),
        "top_stand_front": top.thickness((cake.STAND_CENTRE[0] - cake.STAND_SIZE[0] / 2, 0., cake.STAND_TOP / 2),
                                         (0., 0., 1.), cake.STAND_TOP),
        "wrist_score_at_120_mm": over.lens.fx * cake.LINE_WIDTH / .12,
        "wrist_score_at_200_mm": over.lens.fx * cake.LINE_WIDTH / .20,
        "wrist_over_handle_across": over.best_thickness(handle, (1., 0., 0.), 2 * cake.HANDLE_RADIUS),
        "wrist_over_wire": over.best_thickness(beside_wire, (0., 1., 0.), cake.WIRE_DRAWN_DIAMETER),
        "wrist_grasp_handle_across": grasp.best_thickness(handle, (1., 0., 0.), 2 * cake.HANDLE_RADIUS),
        "wrist_grasp_wire": grasp.best_thickness(beside_wire, (0., 1., 0.), cake.WIRE_DRAWN_DIAMETER),
    }


class VisibilityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.px = sizes()
        cls.top = views.overhead()

    def test_everything_starts_inside_the_overhead_image(self):
        for arm in fixture.ARMS:
            self.assertTrue(self.top.in_view(cake.HANDLE_START[arm]))
        for node in fixture.wire_start():
            self.assertTrue(self.top.in_view(node))
        for sx in (-1, 1):
            for sy in (-1, 1):
                self.assertTrue(self.top.in_view(cake.CAKE_CENTRE + cake.CAKE_HALF * (sx, sy, 1)))

    def test_overhead_sizes(self):
        px = self.px
        self.assertGreater(px["top_handle_across"], 2.)
        self.assertGreater(px["top_handle_length"], 12.)
        self.assertGreater(px["top_wire_slack"], .9)             # the drawn wire covers about a pixel
        self.assertGreater(px["top_wire_at_line"], .9)
        self.assertLess(px["top_real_wire"], .2)                 # at its real 0.64 mm it would not show at all
        self.assertGreater(px["top_block_length"], 25.)
        self.assertGreater(px["top_block_width"], 15.)
        # The overhead camera looks 60 degrees down: the marked line on the near face is far below a pixel,
        # and it cannot see the long side faces at all. The instruction therefore states the height.
        self.assertLess(px["top_line_near_face"], .5)
        # What it does see of the cake: the browned top with the pale split along it, and the near end.
        self.assertGreater(px["top_split_across"], 1.)
        self.assertGreater(px["top_split_length"], 10.)
        self.assertGreater(px["top_top_layer_near_face"], 2.)
        self.assertGreater(px["top_stand_front"], 3.)                # the wooden front of the stand
        # A wrist camera beside the cake resolves the score: 1 to 2 pixels at 200 to 120 mm.
        self.assertGreater(px["wrist_score_at_120_mm"], 1.5)
        self.assertGreater(px["wrist_score_at_200_mm"], 1.)
        self.assertIn("30 mm above the mat", fixture.INSTRUCTION)
        for side in (-1., 1.):
            face = cake.CAKE_CENTRE + (0., side * cake.CAKE_HALF[1], 0.)
            self.assertLess(np.dot((0., side, 0.), self.top.position - face), 0.)

    def test_wrist_cameras_see_a_handle_and_its_wire_on_the_way_down_to_it(self):
        px = self.px
        self.assertGreater(px["wrist_over_handle_across"], 12.)
        self.assertGreater(px["wrist_over_wire"], 5.)
        self.assertGreater(px["wrist_grasp_handle_across"], 20.)
        self.assertGreater(px["wrist_grasp_wire"], 8.)
        ready = views.wrist("left", READY, fixture.BASES["left"])
        self.assertFalse(ready.in_view(cake.HANDLE_START["left"]))   # not at the start: the arm has to move first
        # Of the cake a wrist camera has, at the start, only the far end of the side that faces it, at the edge
        # of its image (within 25 pixels of it); not the middle of that side or its near end.
        marks = {(sx, sy, sz): cake.CAKE_CENTRE + cake.CAKE_HALF * (sx, sy, sz)
                 for sx in (-1, 0, 1) for sy in (-1, 0, 1) for sz in (-1, 0, 1)}
        for arm, side in zip(fixture.ARMS, (1, -1)):
            view = views.wrist(arm, READY, fixture.BASES[arm])
            seen = [key for key, point in marks.items() if view.in_view(point)]
            self.assertIn((1, side, -1), seen)
            for key in seen:
                self.assertEqual(key[:2], (1, side), arm)
                u = view.pixel(marks[key])[0][0]
                self.assertLess(min(u, views.WIDTH - u), 25.)

    def test_colours_stand_out_from_what_is_behind_them(self):
        c = build_scene.COLOURS
        self.assertGreater(contrast(WIRE_COLOUR, c["film"]), 7.)      # the wire lying on the black film
        self.assertGreater(contrast(c["handle"], c["film"]), 7.)      # the handles on the film
        self.assertGreater(contrast(WIRE_COLOUR, c["mat"]), 7.)       # the wire over the black mat
        self.assertGreater(contrast(WIRE_COLOUR, c["stand"]), 4.)     # and against the wooden block
        for part in ("crust", "crust_mid", "crust_edge", "skin"):     # the cake on its black mat
            self.assertGreater(contrast(c[part], c["mat"]), 2.5)
        self.assertGreater(contrast(c["crumb"], c["crust"]), 3.)      # the split in the top crust
        self.assertGreater(contrast(WIRE_COLOUR, c["crust"]), 3.)     # the wire against the browned top
        # Weakest, and meant to be: the score is a light nick in the side, as a real one is, not a dark band.
        self.assertGreater(contrast(c["crumb"], c["skin"]), 1.5)
        self.assertLess(contrast(c["crumb"], c["skin"]), 2.5)
        self.assertGreater(contrast(WIRE_COLOUR, c["skin"]), 1.5)     # the wire against the cake's side
        self.assertGreater(contrast(c["stand"], c["film"]), 1.4)      # the wooden block on the black film
        self.assertEqual(skin.WireSkin.__init__.__defaults__[0], WIRE_COLOUR)

    def test_film_covers_the_wire_and_the_handles_and_lies_under_the_stand(self):
        nodes = fixture.wire_start()
        self.assertGreater(nodes[:, 0].min() - cake.HANDLE_LENGTH / 2, cake.FILM_X[0])
        self.assertLess(np.abs(nodes[:, 1]).max() + .03, cake.FILM_Y[1])
        self.assertLess(cake.STAND_CENTRE[0] + cake.STAND_SIZE[0] / 2, cake.FILM_X[1])
        self.assertGreater(cake.FILM_X[0], .04)                       # clear of the arms' base plates


if __name__ == "__main__":
    for name, value in sizes().items():
        print(f"{name:28s} {value:6.2f} px")
