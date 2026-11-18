"""Dry run of the scripted demonstration through the real wire model and the real scorer, without a simulator.

Needs ``yam_arena`` (pure-Python parts) and ``inspect_robots`` (or a stand-in providing Action and
ActionChunk) on the path; skipped otherwise. See kinematic_world.py for what this does and does not show.
"""

from pathlib import Path
import unittest
from unittest import mock

import numpy as np

import conftest  # noqa: F401

try:
    from yam_arena import ik
    from yam_arena.constants import PAD_TIP_PAST_TCP
    from rdb_cake_wire import cake_model as cake, fixed_scene as fixture, scripted
    from kinematic_world import KinematicWorld, play
except ImportError as error:  # pragma: no cover
    raise unittest.SkipTest(f"dry run needs yam_arena and inspect_robots: {error}")


def run(probe=True, **world_options):
    world = KinematicWorld(**world_options)
    reference = scripted.WireCutReference(world if probe else None)
    result, trace = play(reference, world, fixture.MAX_STEPS)
    return reference, world, result, trace


class DemoDryRunTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference, cls.world, cls.result, cls.trace = run()
        cls.tcp = {arm: np.array([ik.tcp_pose(row[7 * i:7 * i + 6], fixture.BASES[arm])[0] for row in cls.trace])
                   for i, arm in enumerate(fixture.ARMS)}

    def test_script_only_reads_the_simulator(self):
        source = Path(scripted.__file__).read_text()
        for word in ("set_transforms", "set_velocities", "apply_force", "write_", ".grade", "import grading"):
            self.assertNotIn(word, source, word)
        calls = set(__import__("re").findall(r"self\.probe\.(\w+)", source))
        self.assertEqual(calls, {"pos", "tension", "cut"})        # the three read-only calls

    def test_full_score_through_the_real_scorer(self):
        self.assertEqual(self.result["score"], 1.)
        self.assertGreaterEqual(self.result["on_line"], .99)
        self.assertEqual(self.result["off_line"], 0.)
        self.assertEqual(self.result["side_cut_mm"], [185., 185.])
        self.assertFalse(self.world.touched)

    def test_fits_the_run_limit_with_the_hold(self):
        self.assertLess(len(self.trace), fixture.MAX_STEPS - 400)       # 20 s to spare, the 2 s hold included

    def test_every_target_is_reachable(self):
        self.assertLess(self.reference.worst_ik, .001)
        for i in (0, 7):
            self.assertTrue(np.all(self.trace[:, i:i + 6] >= ik.LOWER - 1e-9))
            self.assertTrue(np.all(self.trace[:, i:i + 6] <= ik.UPPER + 1e-9))
            # Per 50 ms step: shoulder joints under 0.12 rad (2.4 of their 5.5 rad/s), wrist joints under 0.2 rad.
            step = np.abs(np.diff(self.trace[:, i:i + 6], axis=0)).max(axis=0)
            self.assertLess(step[:3].max(), .12)
            self.assertLess(step[3:].max(), .20)
        self.assertTrue(np.all((self.trace[:, [6, 13]] >= 0.) & (self.trace[:, [6, 13]] <= 1.)))

    def test_grasp_takes_each_handle_beside_the_wire(self):
        for i, arm in enumerate(fixture.ARMS):
            closing = np.nonzero(self.trace[:, 7 * i + 6] < .5)[0][0]
            np.testing.assert_allclose(self.tcp[arm][closing], scripted.GRASP[arm], atol=.0005)
            self.assertTrue(self.reference.report[f"{arm}_holding"])
        self.assertGreater(scripted.GRASP_Z - PAD_TIP_PAST_TCP, .002)             # pad tips clear of the table
        self.assertLess(scripted.GRASP_Z - PAD_TIP_PAST_TCP, cake.HANDLE_RADIUS)   # and below the handle's middle
        # The pads (15 mm wide) sit between the handle's middle, where the wire is fixed, and its near end.
        self.assertGreater(scripted.GRIP_BACK - .0077, .008)
        self.assertLess(scripted.GRIP_BACK + .0077, cake.HANDLE_LENGTH / 2)

    def test_forces_stay_inside_what_the_block_and_the_fingers_can_take(self):
        self.assertLess(self.world.peak["push"], .6 * cake.CAKE_SLIDE_FORCE)     # the block is not dragged
        self.assertLess(self.world.peak["tension"], 6.)                           # N; each finger squeezes with 30 N
        self.assertLess(self.world.peak["tension"], fixture.END_PULL_LIMIT)

    def test_wire_stays_clear_of_the_fingers_and_of_the_block_top(self):
        self.assertGreater(self.world.min_wire_finger_gap, .003)
        self.assertGreater(self.world.min_over_block, .025)

    def test_ends_with_the_handles_on_the_table_and_the_arms_parked(self):
        for arm in fixture.ARMS:
            self.assertIsNone(self.world.held[arm])
            self.assertAlmostEqual(self.world.handle[arm][0][2], cake.HANDLE_RADIUS)
            np.testing.assert_allclose(self.tcp[arm][-1], fixture.PARK[arm], atol=.002)
        np.testing.assert_allclose(self.trace[-1, [6, 13]], 1.)


class DemoRobustnessTest(unittest.TestCase):
    def test_arms_that_give_way_under_the_wires_pull(self):
        for stiffness in (400., 150.):      # N/m at the fingertips; the YAM's joint gains give roughly 400
            _, world, result, trace = run(arm_stiffness=stiffness)
            self.assertEqual(result["score"], 1., stiffness)
            self.assertEqual(result["off_line"], 0.)
            self.assertLess(len(trace), fixture.MAX_STEPS - 400)

    def test_without_reading_the_simulator(self):
        _, world, result, _ = run(probe=False)
        self.assertEqual(result["score"], 1.)

    def test_drooping_shoulders(self):
        reference, world, result, _ = run(sag=-.01)
        self.assertEqual(result["score"], 1.)
        self.assertLess(reference.worst_ik, .002)

    def test_the_same_script_8_mm_too_high_frees_nothing(self):
        with mock.patch.object(scripted, "SEAM_Z", cake.SEAM_Z + .008):
            _, _, result, _ = run()
        self.assertLessEqual(result["score"], .25)
        self.assertFalse(result["through"])

    def test_the_same_script_without_closing_the_grippers_scores_nothing(self):
        world = KinematicWorld()
        reference = scripted.WireCutReference()
        result = None
        for count, chunk in enumerate(reference.sequence()):
            values = np.asarray(chunk.actions[0].data, dtype=float).copy()
            values[[6, 13]] = 1.
            result = world.step(values)
            reference.measured = world.measured
            if count >= 1000:
                break
        self.assertEqual(result["score"], 0.)
        self.assertEqual(result["on_line"], 0.)


if __name__ == "__main__":
    unittest.main()
