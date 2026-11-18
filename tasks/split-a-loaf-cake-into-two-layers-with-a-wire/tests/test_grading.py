"""The scorer's rules, and strategies that must not score, without a simulator."""

import unittest

import numpy as np

import conftest  # noqa: F401
from rdb_cake_wire import cake_model as cake, grading

GOOD = dict(on_line=1., off_line=0., side_cut=(.185, .185), cake_shift=.001, cake_turn_deg=.2, top_layer_shift=.002,
            top_layer_tilt_deg=.1, top_layer_rise=0., handles_lying=[True, True], jaws=[1., 1.], gripper_gap=.2, speed=0.)


def final(ticks=grading.HOLD_TICKS, **changes):
    grade = grading.CakeCutGrade()
    for _ in range(ticks):
        result = grade.update(**{**GOOD, **changes})
    return result["score"]


class RulesTest(unittest.TestCase):
    def test_full_score_needs_the_two_second_hold(self):
        self.assertEqual(final(), 1.)
        self.assertEqual(final(ticks=grading.HOLD_TICKS - 1), .75)

    def test_no_cut_scores_nothing_whatever_else_is_true(self):
        self.assertEqual(final(on_line=0., side_cut=(0., 0.)), 0.)
        self.assertEqual(final(on_line=.2, side_cut=(.059, .18)), 0.)     # one side short of 60 mm

    def test_partial_cut(self):
        self.assertEqual(final(on_line=.5, side_cut=(.09, .09)), .25)
        self.assertEqual(final(on_line=.98, side_cut=(.18, .18)), .25)    # a strip still holds the top_layer

    def test_spoiled_block_is_capped(self):
        self.assertEqual(final(off_line=.06), .25)
        self.assertEqual(final(off_line=.05), 1.)
        self.assertEqual(final(off_line=.5, on_line=0., side_cut=(0., 0.)), 0.)   # a cut at the wrong height only

    def test_block_or_top_layer_moved(self):
        for change in (dict(cake_shift=.011), dict(cake_turn_deg=6.), dict(top_layer_shift=.011), dict(top_layer_tilt_deg=6.),
                       dict(top_layer_rise=.006), dict(top_layer_rise=-.02)):
            self.assertEqual(final(**change), .5, change)

    def test_touching_the_cake_scores_nothing_for_the_rest_of_the_trial(self):
        grade = grading.CakeCutGrade()
        grade.update(**{**GOOD, "touching": True})
        for _ in range(grading.HOLD_TICKS + 5):
            result = grade.update(**GOOD)
        self.assertTrue(result["touched"])
        self.assertEqual(result["score"], 0.)

    def test_wire_not_put_down(self):
        for change in (dict(handles_lying=[True, False]), dict(jaws=[1., .5]), dict(gripper_gap=.049), dict(speed=.03)):
            self.assertEqual(final(**change), .75, change)

    def test_hold_restarts(self):
        grade = grading.CakeCutGrade()
        for _ in range(30):
            grade.update(**GOOD)
        grade.update(**{**GOOD, "speed": .1})
        for _ in range(grading.HOLD_TICKS - 1):
            result = grade.update(**GOOD)
        self.assertEqual(result["score"], .75)
        self.assertEqual(grade.update(**GOOD)["score"], 1.)

    def test_only_listed_scores(self):
        seen = set()
        for on, side in ((0., 0.), (.5, .1), (1., .18)):
            for off in (0., .2):
                for shift in (0., .05):
                    for lying in (True, False):
                        seen.add(final(on_line=on, side_cut=(side, side), off_line=off, cake_shift=shift,
                                       handles_lying=[lying, lying]))
        self.assertEqual(seen, {0., .25, .5, .75, 1.})


class MeasureTest(unittest.TestCase):
    """measure() turns poses into the quantities above."""

    class Wire:
        def cut_fraction(self):
            return 1., 0.

        def side_cut(self):
            return .185, .185

    def measure(self, **moved):
        poses = {name: np.array(cake.START[name], dtype=float) for name in cake.ACTORS}
        rotations = {name: np.eye(3) for name in cake.ACTORS}
        for name, value in moved.items():
            if np.shape(value) == (3,):
                poses[name] = poses[name] + value
            else:
                rotations[name] = value
        tcp = [np.array((.14, .34, .19)), np.array((.14, -.34, .19))]
        return grading.measure(self.Wire(), poses, rotations, [0., 0., 0., 0.], tcp, [1., 1.])

    def test_start_layout(self):
        m = self.measure()
        self.assertAlmostEqual(m["cake_shift"], 0.)
        self.assertAlmostEqual(m["top_layer_shift"], 0.)
        self.assertAlmostEqual(m["top_layer_rise"], 0.)
        self.assertEqual(m["handles_lying"], [True, True])
        self.assertGreater(m["gripper_gap"], .2)
        self.assertEqual(grading.CakeCutGrade().update(**m)["score"], .75)

    def test_dragged_block(self):
        m = self.measure(base=np.array((-.02, 0., 0.)), top_layer=np.array((-.02, 0., 0.)))
        self.assertAlmostEqual(m["cake_shift"], .02)
        self.assertAlmostEqual(m["top_layer_shift"], 0.)
        self.assertEqual(grading.CakeCutGrade().update(**m)["score"], .5)

    def test_top_layer_pulled_off_or_lifted(self):
        self.assertAlmostEqual(self.measure(top_layer=np.array((0., .03, 0.)))["top_layer_shift"], .03)
        self.assertAlmostEqual(self.measure(top_layer=np.array((0., 0., .04)))["top_layer_rise"], .04)

    def test_handle_left_on_the_stand_the_cake_or_in_the_air(self):
        on_stand = np.array((.40, .06, cake.STAND_TOP + cake.HANDLE_RADIUS)) - cake.HANDLE_START["left"]
        self.assertEqual(self.measure(handle_left=on_stand)["handles_lying"], [False, True])
        self.assertEqual(self.measure(handle_right=np.array((0., 0., .03)))["handles_lying"], [True, False])
        self.assertEqual(self.measure(handle_right=np.array((0., 0., -.5)))["handles_lying"], [True, False])   # fell off

    def test_gripper_gap_is_to_the_nearest_handle_or_cake_surface(self):
        poses = {name: np.array(cake.START[name], dtype=float) for name in cake.ACTORS}
        rotations = {name: np.eye(3) for name in cake.ACTORS}
        above = cake.HANDLE_START["left"] + (.03, 0., .04)        # 40 mm above the handle, over its far half
        m = grading.measure(self.Wire(), poses, rotations, [0.] * 4, [above, np.array((.14, -.5, .2))], [1., 1.])
        self.assertAlmostEqual(m["gripper_gap"], .04, places=6)
        beside = cake.CAKE_CENTRE + (0., cake.CAKE_HALF[1] + .03, 0.)
        m = grading.measure(self.Wire(), poses, rotations, [0.] * 4, [beside, np.array((.14, -.5, .2))], [1., 1.])
        self.assertAlmostEqual(m["gripper_gap"], .03, places=6)


if __name__ == "__main__":
    unittest.main()
