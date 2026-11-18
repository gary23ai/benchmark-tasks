import unittest

import numpy as np

from rdb_nested_cups import cup_model as cup
from rdb_nested_cups.grading import HOLD_TICKS, StuckCups, nested

UPRIGHT = (0., 0., 0., 1.)
ON_SIDE = (0., np.sqrt(.5), 0., np.sqrt(.5))  # axis along +x
INVERTED = (1., 0., 0., 0.)
PARKED = np.array([[.14, .34, .19], [.14, -.34, .19]])


def poses(outer_xyz, inner_xyz, outer_q=UPRIGHT, inner_q=UPRIGHT):
    return np.array([[*outer_xyz, *outer_q], [*inner_xyz, *inner_q]], dtype=float)


class StuckCupsTest(unittest.TestCase):
    def setUp(self):
        self.grader = StuckCups()
        self.start = poses(cup.OUTER_START, cup.INNER_START)
        self.done = poses((.275, .035, 0.), (.275, .185, 0.))
        self.velocity = np.zeros((2, 6))
        self.tcp = PARKED.copy()
        self.touching = np.array([False, False])

    def step(self, pose, n=1):
        for _ in range(n):
            grade = self.grader.update(pose, self.velocity, self.tcp, self.touching)
        return grade

    # -- the intended outcome ----------------------------------------------------------
    def test_start_state_scores_zero(self):
        self.assertTrue(nested(self.start))
        self.assertEqual(self.step(self.start, 3 * HOLD_TICKS)["score"], 0.)

    def test_full_score_needs_both_cups_standing_for_the_whole_hold(self):
        self.assertEqual(self.step(self.done, HOLD_TICKS - 1)["score"], .5)
        self.assertEqual(self.step(self.done)["score"], 1.)

    def test_upright_cups_side_by_side_and_touching_are_apart(self):
        touching_rims = poses((.275, .035, 0.), (.275, .035 + 2 * cup.RIM_RADIUS, 0.))
        self.assertEqual(self.step(touching_rims, HOLD_TICKS)["score"], 1.)

    # -- partial credit ----------------------------------------------------------------
    def test_apart_but_still_held_in_the_air_scores_half(self):
        held = poses((.275, .035, 0.), (.275, .185, .12))
        self.tcp[0] = (.275, .185, .12 + cup.HEIGHT)
        self.touching[:] = True
        self.assertEqual(self.step(held, HOLD_TICKS)["score"], .5)

    def test_one_cup_knocked_over_after_separation_scores_three_quarters(self):
        lying = poses((.275, .035, 0.), (.275, .185, cup.RIM_RADIUS), inner_q=ON_SIDE)
        self.assertEqual(self.step(lying, HOLD_TICKS)["score"], .75)

    def test_inverted_cup_is_not_standing(self):
        upside_down = poses((.275, .035, 0.), (.275, .185, cup.HEIGHT), inner_q=INVERTED)
        self.assertEqual(self.step(upside_down, HOLD_TICKS)["score"], .75)

    def test_gripper_touching_or_close_to_a_cup_withholds_that_cup(self):
        self.touching[1] = True
        self.assertEqual(self.step(self.done, HOLD_TICKS)["score"], .75)
        self.touching[1] = False
        self.tcp[1] = (.275, .035 - cup.RIM_RADIUS - .049, .04)  # 49 mm from the outer cup's surface
        self.assertEqual(self.step(self.done, HOLD_TICKS)["score"], .75)
        self.tcp[1] = (.275, .035 - cup.RIM_RADIUS - .051, .04)
        self.assertEqual(self.step(self.done, HOLD_TICKS)["score"], 1.)

    def test_a_moving_cup_restarts_its_hold(self):
        self.step(self.done, HOLD_TICKS)
        self.velocity[0, 0] = .011
        self.assertEqual(self.step(self.done)["score"], .75)
        self.velocity[0, 0] = 0.
        self.assertEqual(self.step(self.done, HOLD_TICKS - 1)["score"], .75)
        self.assertEqual(self.step(self.done)["score"], 1.)

    def test_tilt_and_height_limits(self):
        tilt = np.radians(11.)
        leaning = poses((.275, .035, 0.), (.275, .185, 0.), inner_q=(np.sin(tilt / 2), 0., 0., np.cos(tilt / 2)))
        self.assertEqual(self.step(leaning, HOLD_TICKS)["score"], .75)
        raised = poses((.275, .035, 0.), (.275, .185, .006))
        self.assertEqual(StuckCups().update(raised, self.velocity, self.tcp, self.touching)["standing"], [False, False])

    # -- unintended strategies and failures ---------------------------------------------
    def test_knocking_the_stuck_pair_over_scores_zero(self):
        pitch = cup.SEATED_PITCH
        toppled = poses((.275, .035, cup.RIM_RADIUS), (.275 + pitch, .035, cup.RIM_RADIUS), ON_SIDE, ON_SIDE)
        self.assertTrue(nested(toppled))
        self.assertEqual(self.step(toppled, 3 * HOLD_TICKS)["score"], 0.)

    def test_loosened_but_still_inside_scores_zero(self):
        loose = poses(cup.OUTER_START, (.275, .035, .050))
        self.assertEqual(self.step(loose, HOLD_TICKS)["score"], 0.)
        tilted_in_mouth = poses(cup.OUTER_START, (.275, .040, .060), inner_q=(np.sin(.2), 0., 0., np.cos(.2)))
        self.assertEqual(self.step(tilted_in_mouth, HOLD_TICKS)["score"], 0.)

    def test_cup_capping_the_other_scores_zero(self):
        capped = poses((.275, .035, 0.), (.275, .035, cup.HEIGHT + .03), inner_q=INVERTED)
        self.assertEqual(self.step(capped, HOLD_TICKS)["score"], 0.)

    def test_a_thrown_cup_zeroes_the_trial_even_if_it_lands_upright(self):
        flying = poses((.275, .035, 0.), (.40, .185, .15))
        self.velocity[1, :3] = (.8, 0., -.5)
        grade = self.step(flying)
        self.assertEqual(grade["score"], 0.)
        self.assertIn("thrown", grade["violation"])
        self.velocity[:] = 0.
        self.assertEqual(self.step(self.done, 3 * HOLD_TICKS)["score"], 0.)

    def test_the_stuck_pair_flung_or_dropped_from_a_height_zeroes_the_trial(self):
        high = poses((.275, .035, .20), (.275, .035, .20 + cup.SEATED_PITCH))
        self.velocity[:, 2] = -1.5
        self.assertEqual(self.step(high)["score"], 0.)
        self.velocity[:] = 0.
        self.assertEqual(self.step(self.done, 3 * HOLD_TICKS)["score"], 0.)

    def test_a_cup_carried_by_a_gripper_is_not_thrown(self):
        carried = poses((.275, .035, 0.), (.275, .185, .12))
        self.velocity[1, :3] = (0., .3, 0.)
        self.tcp[0] = (.275, .185, .12 + cup.HEIGHT)
        self.touching[1] = True
        self.assertIsNone(self.step(carried)["violation"])

    def test_a_cup_resting_high_on_something_is_not_thrown(self):
        resting = poses((.275, .035, 0.), (.275, .185, .09))
        self.assertIsNone(self.step(resting, 5)["violation"])

    def test_a_cup_off_the_table_zeroes_the_trial(self):
        off_edge = poses((.275, .035, 0.), (.275, .70, 0.))
        grade = self.step(off_edge)
        self.assertEqual(grade["score"], 0.)
        self.assertIn("left the table", grade["violation"])
        self.assertEqual(self.step(self.done, 3 * HOLD_TICKS)["score"], 0.)
        fallen = StuckCups().update(poses((.275, .035, 0.), (.275, .185, -.2)), self.velocity, self.tcp, self.touching)
        self.assertEqual(fallen["score"], 0.)

    def test_non_finite_state_scores_zero(self):
        bad = self.done.copy()
        bad[0, 0] = np.nan
        self.assertEqual(self.step(bad)["score"], 0.)
        self.assertEqual(self.step(self.done, 3 * HOLD_TICKS)["score"], 0.)

    def test_possible_scores(self):
        seen = set()
        for pose in (self.start, self.done):
            grader = StuckCups()
            for _ in range(HOLD_TICKS):
                seen.add(grader.update(pose, self.velocity, self.tcp, self.touching)["score"])
        self.assertEqual(seen, {0., .5, 1.})


if __name__ == "__main__":
    unittest.main()
