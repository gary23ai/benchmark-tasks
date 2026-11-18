"""Dry run of the closed-loop demonstration against a kinematic stand-in for the simulator.

Needs ``yam_arena`` (pure-Python parts) and ``inspect_robots`` on the path; skipped otherwise. It checks the
control flow (grasp placement from cup poses, grip checks, pull until out, regrip, set-down), reach, joint
limits and timing. It says nothing about contact physics.
"""

import unittest

import numpy as np

try:
    from yam_arena import ik
    from yam_arena.constants import PAD_TIP_PAST_TCP
    from rdb_nested_cups import adapter, cup_model as cup, fixed_scene as fixture, grading, scripted
    from fake_world import FakeWorld, play
except ImportError as error:  # pragma: no cover
    raise unittest.SkipTest(f"dry run needs yam_arena and inspect_robots: {error}")

HOLD = fixture.CONTROL_HZ * fixture.SUCCESS_HOLD_S


def run(**options):
    world = FakeWorld(**options)
    reference = scripted.CupReference(world)
    return world, reference, play(world, reference)


class DemoDryRunTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world, cls.reference, cls.trace = run()
        cls.tcp = {arm: np.array([ik.tcp_pose(row[7 * i:7 * i + 6], fixture.BASES[arm])[0] for row in cls.trace])
                   for i, arm in enumerate(fixture.ARMS)}
        cls.phases = [p["phase"] for p in cls.reference.phases]

    def test_separates_and_sets_down_in_the_stand_in_world(self):
        self.assertEqual(self.reference.report["regrips"], 0)
        self.assertTrue(self.reference.report["right_holding"] and self.reference.report["left_holding"])
        self.assertTrue(self.reference.report["lift_test"]["pair_lifted_together"])
        for phase in ("lift_test_lifted", "right_closed_on_outer_cup", "left_closed_on_inner_rim", "pulled_out", "left_released",
                      "right_parked"):
            self.assertIn(phase, self.phases)
        self.assertFalse(grading.nested(self.world.cups))
        np.testing.assert_allclose(self.world.cups[1, :3], (cup.STACK_XY[0], cup.STACK_XY[1] + scripted.CARRY_Y, 0.),
                                   atol=.003)
        np.testing.assert_allclose(self.world.cups[0, :3], cup.OUTER_START, atol=.0005)

    def test_fits_the_run_limit_with_the_hold(self):
        self.assertLess(len(self.trace) + HOLD, .55 * fixture.MAX_STEPS)

    def test_every_target_is_reachable(self):
        self.assertLess(self.reference.worst_ik, .001)
        for i in (0, 7):
            self.assertTrue(np.all(self.trace[:, i:i + 6] >= ik.LOWER - 1e-9))
            self.assertTrue(np.all(self.trace[:, i:i + 6] <= ik.UPPER + 1e-9))
            # Per 50 ms step: shoulder joints under 0.12 rad, wrist joints under 0.2 rad.
            step = np.abs(np.diff(self.trace[:, i:i + 6], axis=0)).max(axis=0)
            self.assertLess(step[:3].max(), .12)
            self.assertLess(step[3:].max(), .20)
        self.assertTrue(np.all((self.trace[:, [6, 13]] >= 0.) & (self.trace[:, [6, 13]] <= 1.)))

    def test_grasps_are_placed_from_the_cup_poses(self):
        world, reference, _ = run(nudge=(.004, -.003, 0.))  # the pair stands 5 mm off its written place
        self.assertTrue(reference.report["left_holding"])
        self.assertFalse(grading.nested(world.cups))
        at = {p["phase"]: p for p in reference.phases}  # the last time each phase was logged
        rim = np.array(cup.INNER_START) + (.004, -.003, cup.HEIGHT)
        pinch = rim - (scripted.PINCH_RADIUS, 0., 0.)  # the rim point nearest the robot, on the right gripper's grip axis
        np.testing.assert_allclose(at["left_at_inner_rim_open"]["left"]["tcp_measured_mm"], 1000 * pinch, atol=.6)
        # The fingertips stop above the outer cup's rim and inside the height of the inner cup's rim band; the
        # open inside finger clears the band's inner face and the open outside finger is outside the cup.
        tips = rim[2] - PAD_TIP_PAST_TCP
        self.assertGreater(tips - (cup.OUTER_START[2] + cup.HEIGHT), .0015)
        self.assertGreater(tips - (cup.INNER_START[2] + cup.BAND_START), .0005)
        half_open = abs(scripted.OPEN)
        self.assertLess(abs(scripted.PINCH_RADIUS - half_open), cup.RIM_RADIUS - cup.BAND_THICKNESS - .010)
        self.assertGreater(scripted.PINCH_RADIUS + half_open, cup.RIM_RADIUS + .010)

    def test_closes_slowly_and_within_the_grip_limit(self):
        closing = np.nonzero(np.diff(self.trace[:, 6]) < 0)[0]
        per_step = -np.diff(self.trace[:, 6])[closing].max() * abs(scripted.OPEN)
        self.assertLess(per_step, .0005)  # each finger moves under 0.5 mm per control step
        self.assertLess(5000. * scripted.SQUEEZE, 50.)
        # The pinch squeezes with 5000 N/m x half the band's thickness per finger: 30 N, holding 21 N of pull.
        pinch_force = 5000. * cup.BAND_THICKNESS / 2.
        self.assertLess(pinch_force, 40.)
        self.assertGreater(2 * cup.FRICTION * pinch_force, 15. + 5.)  # the largest expected fit plus the right hand's drag

    def test_pull_stops_once_the_inner_cup_is_clear(self):
        at = {p["phase"]: p for p in self.reference.phases}
        self.assertGreater(at["pulled_out"]["inner_above_outer_mm"], 1000 * (cup.HEIGHT + scripted.CLEAR_OF_RIM) - .5)
        self.assertLess(at["pulled_out"]["left"]["tcp_measured_mm"][2], 1000 * scripted.HIGHEST_TCP + .5)
        # The right gripper lets go only after the left one has.
        self.assertGreater(at["right_released"]["step"], at["left_released"]["step"])

    def test_ends_parked_and_open(self):
        for arm in fixture.ARMS:
            np.testing.assert_allclose(self.tcp[arm][-1], fixture.PARK[arm], atol=.001)
        np.testing.assert_allclose(self.trace[-1, [6, 13]], 1.)

    def test_five_regrips_still_fit_the_run_limit(self):
        world, reference, trace = run(failed_grasps=6)  # the lift test's pinch and five more
        self.assertEqual(reference.report["regrips"], 5)
        self.assertFalse(grading.nested(world.cups))
        self.assertLess(len(trace) + HOLD, fixture.MAX_STEPS - 200)

    def test_regrips_when_a_grasp_closes_on_nothing(self):
        world, reference, trace = run(failed_grasps=3)  # the lift test's pinch and two more
        self.assertEqual(reference.report["regrips"], 2)
        self.assertFalse(grading.nested(world.cups))
        self.assertEqual(reference.report["left_pinch_turn_deg"], 110.)  # third try, third rim point
        self.assertLess(len(trace) + HOLD, fixture.MAX_STEPS)

    def test_gives_up_cleanly_when_nothing_can_be_gripped(self):
        world, reference, trace = run(never_hold=True)
        self.assertEqual(reference.report["regrips"], scripted.MAX_REGRIPS)
        self.assertIn("gave_up", [p["phase"] for p in reference.phases])
        self.assertTrue(grading.nested(world.cups))
        self.assertLess(len(trace), fixture.MAX_STEPS)
        np.testing.assert_allclose(trace[-1, [6, 13]], 1.)

    def test_policy_reads_cup_poses_through_the_embodiment(self):
        policy = adapter.make_scripted()
        info = type("Info", (), {"environment_id": "stand-in"})()
        adapter._RIGS["stand-in"] = self.world
        policy.bind(info)
        policy.reset(None)
        self.assertIs(policy.reference.rig, self.world)


if __name__ == "__main__":
    unittest.main()
