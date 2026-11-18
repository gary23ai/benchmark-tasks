"""The scorer's rules, including the ways of scoring that must not work. No simulator needed."""

import sys
from pathlib import Path
import unittest

import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parent)]
from helpers import FAR, grip_point, on_glass, on_table  # noqa: E402
from rdb_window_wall import cleaner_model as m, grading as g  # noqa: E402

STILL = np.zeros((2, 3))
FREE = (False, False)
A = .170  # where the tests pair the halves, near the stile
END = m.BAND_U[1] - .040  # where a full stroke ends


def hold(grade, pose, ticks=g.HOLD_TICKS, tcp=FAR, touching=FREE):
    out = None
    for _ in range(ticks):
        out = grade.update(pose, STILL, tcp, touching)
    return out


def slide(grade, x0, x1, *, touching=FREE, tcp=None, speed=.04, z=m.WORK_Z, lag=.003, wiper_touched=False):
    """Slide the pair from x0 to x1; the inside half is held by the left gripper, the outside half follows."""
    out = None
    step = speed / g.CONTROL_HZ
    for x in np.arange(x0, x1 + 1e-9, step):
        pose = np.array((on_glass("inside", x, z), on_glass("outside", x - lag, z)))
        hands = np.array((grip_point(pose[0]), FAR[1])) if tcp is None else tcp
        out = grade.update(pose, np.array(((speed, 0, 0), (speed, 0, 0))), hands, (True, wiper_touched))
    return out


class GradingTest(unittest.TestCase):
    def test_nothing_done_scores_zero(self):
        grade = g.WindowGrade()
        out = hold(grade, np.array((on_table("inside"), on_table("outside"))), 200)
        self.assertEqual(out["score"], 0.)
        self.assertFalse(out["ever_paired"])

    def test_halves_stuck_together_without_the_glass_score_zero(self):
        # Pad to pad on the table, and pad to pad in the air beside the pane.
        stacked = np.array((on_table("inside"), np.r_[m.START["inside"][:2], .0, 1., 0., 0., 0.]))
        beside = np.array((on_glass("inside", .30, gap=.050), on_glass("outside", .30, gap=-.050 - m.PANE_T)))
        for pose in (stacked, beside):
            self.assertEqual(hold(g.WindowGrade(), pose, 100)["score"], 0.)

    def test_both_halves_on_the_same_face_score_zero(self):
        pose = np.array((on_glass("inside", .25), on_glass("outside", .40, face="inside")))
        self.assertEqual(hold(g.WindowGrade(), pose, 100)["score"], 0.)

    def test_one_half_on_the_glass_scores_zero(self):
        pose = np.array((on_glass("inside", .25), on_table("outside")))
        self.assertEqual(hold(g.WindowGrade(), pose, 100)["score"], 0.)

    def test_caught_one_magnet_off_is_not_paired(self):
        # The halves also hold weakly when shifted by the 70 mm magnet spacing; that is not lined up.
        pose = np.array((on_glass("inside", .25), on_glass("outside", .25 + 2 * m.MAGNET_X)))
        self.assertEqual(hold(g.WindowGrade(), pose, 100)["score"], 0.)

    def test_tilted_or_lifted_half_is_not_on_the_glass(self):
        lifted = np.array((on_glass("inside", .25), on_glass("outside", .25, gap=.004)))
        self.assertEqual(hold(g.WindowGrade(), lifted, 100)["score"], 0.)
        tilted = np.array((on_glass("inside", .25), on_glass("outside", .25)))
        c, s = np.cos(np.radians(8.)), np.sin(np.radians(8.))
        tilted[1] = on_glass("outside", .25, turn=np.array(((1., 0., 0.), (0., c, -s), (0., s, c))))
        self.assertEqual(hold(g.WindowGrade(), tilted, 100)["score"], 0.)

    def test_paired_while_still_held_scores_a_quarter(self):
        pose = np.array((on_glass("inside", .25), on_glass("outside", .25)))
        tcp = np.array((grip_point(pose[0]), grip_point(pose[1])))
        self.assertEqual(hold(g.WindowGrade(), pose, 200, tcp=tcp, touching=(True, True))["score"], .25)
        # Open fingers that no longer touch but have not moved away do not count as let go either.
        self.assertEqual(hold(g.WindowGrade(), pose, 200, tcp=tcp)["score"], .25)
        near = np.array((grip_point(pose[0]) + (0., .07, 0.), FAR[1]))
        self.assertEqual(hold(g.WindowGrade(), pose, 200, tcp=near)["score"], .25)

    def test_paired_and_let_go_scores_half_after_two_seconds(self):
        pose = np.array((on_glass("inside", .25), on_glass("outside", .25)))
        grade = g.WindowGrade()
        self.assertEqual(hold(grade, pose, g.HOLD_TICKS - 1)["score"], .25)
        self.assertEqual(hold(grade, pose, 1)["score"], .5)

    def test_swapped_halves_are_accepted(self):
        pose = np.array((on_glass("inside", .25, face="outside"), on_glass("outside", .25, face="inside")))
        self.assertEqual(hold(g.WindowGrade(), pose)["score"], .5)

    def test_outside_half_falling_off_afterwards_drops_back_to_a_quarter(self):
        grade = g.WindowGrade()
        pose = np.array((on_glass("inside", .25), on_glass("outside", .25)))
        self.assertEqual(hold(grade, pose)["score"], .5)
        fallen = np.array((pose[0], on_table("outside", (.25, -.10))))  # on the table outdoors
        self.assertEqual(hold(grade, fallen, 100)["score"], .25)

    def test_still_sliding_at_the_end_is_not_finished(self):
        grade = g.WindowGrade()
        pose = np.array((on_glass("inside", .25), on_glass("outside", .25)))
        out = None
        for _ in range(100):
            out = grade.update(pose, np.array(((0., 0., -.03), (0., 0., -.03))), FAR, FREE)
        self.assertEqual(out["score"], .25)

    def test_full_stroke_then_let_go_scores_one(self):
        grade = g.WindowGrade()
        out = slide(grade, A, END)
        self.assertEqual(out["area_cleaned_fraction"], 1.)
        self.assertEqual(out["area_cleaned_mm2"], 150. * 30.)
        self.assertEqual(out["score"], .25)  # the left gripper is still on the inside half
        pose = np.array((on_glass("inside", END), on_glass("outside", END - .002)))
        self.assertEqual(hold(grade, pose)["score"], 1.)

    def test_half_a_stroke_scores_three_quarters(self):
        grade = g.WindowGrade()
        out = slide(grade, A, .45)
        self.assertGreaterEqual(out["area_cleaned_fraction"], g.HALF_AREA)
        self.assertLess(out["area_cleaned_fraction"], g.FULL_AREA)
        pose = np.array((on_glass("inside", .45), on_glass("outside", .448)))
        self.assertEqual(hold(grade, pose)["score"], .75)

    def test_short_stroke_stays_at_half(self):
        grade = g.WindowGrade()
        out = slide(grade, A, .36)
        self.assertLess(out["area_cleaned_fraction"], g.HALF_AREA)
        pose = np.array((on_glass("inside", .36), on_glass("outside", .358)))
        self.assertEqual(hold(grade, pose)["score"], .5)

    def test_area_is_counted_square_by_square_not_along_a_centre_line(self):
        # A full-length stroke 30 mm too high wipes only the band's top three rows of squares: half its
        # area, so 0.75 and not 1.0. (Scoring along the band's centre line would have called this clean.)
        grade = g.WindowGrade()
        out = slide(grade, A, END, z=m.WORK_Z + .030)
        cells = np.array(out["cells_cleaned"]).reshape(g.GRID_SHAPE)
        self.assertFalse(cells[:3].any())  # the bottom three rows are untouched
        self.assertTrue(cells[3:].all())
        self.assertEqual(out["area_cleaned_fraction"], .5)
        pose = np.array((on_glass("inside", END, m.WORK_Z + .030), on_glass("outside", END - .002, m.WORK_Z + .030)))
        self.assertEqual(hold(grade, pose)["score"], .75)
        # One row less and it is under half.
        grade = g.WindowGrade()
        self.assertLess(slide(grade, A, END, z=m.WORK_Z + .036)["area_cleaned_fraction"], g.HALF_AREA)

    def test_grid_is_the_band(self):
        self.assertEqual(g.GRID_SHAPE, (6, 30))
        self.assertEqual(g.GRID_POINTS.shape, (m.CELLS, 3))
        np.testing.assert_allclose(g.GRID_POINTS[:, 0].min() - m.CELL / 2, m.BAND_U[0])
        np.testing.assert_allclose(g.GRID_POINTS[:, 0].max() + m.CELL / 2, m.BAND_U[1])
        np.testing.assert_allclose(g.GRID_POINTS[:, 2].min() - m.CELL / 2, m.BAND_Z[0])
        np.testing.assert_allclose(g.GRID_POINTS[:, 2].max() + m.CELL / 2, m.BAND_Z[1])
        np.testing.assert_allclose(g.GRID_POINTS[:, 1], m.FACE_Y["outside"])
        self.assertEqual(g.CELL_MM2 * m.CELLS, 150. * 30.)
        self.assertEqual((g.HALF_AREA, g.FULL_AREA), (.5, .9))

    def test_wiping_with_the_outside_half_in_the_gripper_does_not_count(self):
        grade = g.WindowGrade()
        out = slide(grade, A, END, wiper_touched=True)
        self.assertEqual(out["area_cleaned_fraction"], 0.)
        pose = np.array((on_glass("inside", END), on_glass("outside", END - .002)))
        self.assertEqual(hold(grade, pose)["score"], .5)

    def test_wiping_with_the_outside_half_alone_does_not_count(self):
        # A gripper rubs the band with the outside half before the halves are paired.
        grade = g.WindowGrade()
        for x in np.arange(A, END, .002):
            pose = np.array((on_table("inside"), on_glass("outside", x)))
            out = grade.update(pose, np.array(((0, 0, 0), (.04, 0, 0))), np.array((FAR[0], grip_point(pose[1]))), (False, True))
        self.assertEqual(out["area_cleaned_fraction"], 0.)
        self.assertEqual(out["score"], 0.)

    def test_pad_resting_on_the_band_without_sliding_does_not_clean_it(self):
        grade = g.WindowGrade()
        pose = np.array((on_glass("inside", .48), on_glass("outside", .48)))
        out = hold(grade, pose, 200)
        self.assertEqual(out["area_cleaned_fraction"], 0.)
        self.assertEqual(out["score"], .5)

    def test_stroke_below_the_band_misses_it(self):
        grade = g.WindowGrade()
        out = slide(grade, A, END, z=m.WORK_Z - .130)  # the pad's apex passes 13 mm under the band
        self.assertEqual(out["area_cleaned_fraction"], 0.)

    def test_going_over_or_round_the_wall_scores_zero_for_good(self):
        pose = np.array((on_glass("inside", A), on_glass("outside", A)))
        indoors, outdoors = m.WALL_MID_N + .05, m.WALL_MID_N - .05
        for u, z in ((0., m.WALL_H + .03),  # over the top
                     (m.WALL_U[1] + .03, .20), (m.WALL_U[0] - .03, .20)):  # round either end
            grade = g.WindowGrade()
            grade.update(pose, STILL, np.array((m.to_world((u, indoors, z)), FAR[1])), FREE)
            out = grade.update(pose, STILL, np.array((m.to_world((u, outdoors, z)), FAR[1])), FREE)
            self.assertIn("crossed the wall", out["violation"])
            self.assertEqual(out["score"], 0.)
            slide(grade, A, END)  # nothing done afterwards brings the score back
            done = np.array((on_glass("inside", END), on_glass("outside", END - .002)))
            self.assertEqual(hold(grade, done)["score"], 0.)

    def test_reaching_out_through_the_open_light_is_allowed(self):
        grade = g.WindowGrade()
        pose = np.array((on_table("inside"), on_table("outside")))
        u, z = sum(m.OPENING_U) / 2, sum(m.OPENING_Z) / 2
        for n in (.05, .02, -.02, -.08, -.02, .02, .05):  # out and back in
            out = grade.update(pose, STILL, np.array((FAR[0], m.to_world((u, m.WALL_MID_N + n, z)))), FREE)
        self.assertIsNone(out["violation"])
        self.assertTrue(g.in_opening((m.OPENING_U[1] - .001, 0., z)))
        self.assertFalse(g.in_opening((m.OPENING_U[1] + .001, 0., z)))  # the stile is not the opening
        self.assertFalse(g.in_opening((u, 0., m.OPENING_Z[1] + .001)))

    def test_bad_numbers_do_not_crash_or_score(self):
        grade = g.WindowGrade()
        pose = np.array((on_glass("inside", .25), on_glass("outside", .25)))
        pose[1, 0] = np.nan
        self.assertEqual(grade.update(pose, STILL, FAR, FREE)["score"], 0.)

    def test_scores_are_only_the_five_listed(self):
        seen = set()
        grade = g.WindowGrade()
        seen.add(hold(grade, np.array((on_table("inside"), on_table("outside"))), 5)["score"])
        seen.add(slide(grade, A, .30)["score"])
        seen.add(hold(grade, np.array((on_glass("inside", .30), on_glass("outside", .298))))["score"])
        slide(grade, .30, .45)
        seen.add(hold(grade, np.array((on_glass("inside", .45), on_glass("outside", .448))))["score"])
        slide(grade, .45, END)
        seen.add(hold(grade, np.array((on_glass("inside", END), on_glass("outside", END - .002))))["score"])
        self.assertEqual(seen, {0., .25, .5, .75, 1.})


if __name__ == "__main__":
    unittest.main()
