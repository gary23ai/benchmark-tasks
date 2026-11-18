"""Kinematic dry run of the demonstration through the real scorer, without a simulator.

Needs ``yam_arena`` (pure-Python parts), ``scipy`` and ``inspect_robots`` on the path; skipped otherwise.
The arms go exactly where they are commanded. A half moves rigidly with a gripper that closed on its handle
bar, cannot pass through the glass, stays on the glass when it is let go opposite the other half and then
trails that half by 3 mm; a half let go anywhere else drops to the table on its own side of the wall. The
stand-in works in pane coordinates and hands the scorer tabletop poses, and the demonstration the same
readings of the halves, as the simulator does. This checks reach, joint limits, timing, the order of events,
the way through the open light, the demonstration's checks and second attempts, and the scorer's reading of
it all. It says nothing about contact physics. Clearances of the arms' collision hulls from the wall are a
separate, slower check reported in the README.
"""

import sys
from pathlib import Path
import unittest

import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parent)]
try:
    from yam_arena import ik
    from helpers import quat
    from rdb_window_wall import cleaner_model as m, fixed_scene as fixture, grading, scripted
except ImportError as error:  # pragma: no cover
    raise unittest.SkipTest(f"dry run needs yam_arena, scipy and inspect_robots: {error}")

LAG = .003
IDLE_STOP = 80


class World:
    """The stand-in for the simulator described in the module docstring. Its state is in pane coordinates."""

    def __init__(self, lag=LAG, miss=()):
        self.lag = lag
        self.miss = set(miss)  # arms whose first close on a handle bar fails to take hold
        self.empty = set()  # arms closed on nothing
        self.origin = {h: np.array((*m.START_UN[h], 0.)) for h in m.HALVES}
        self.rot = {h: m.rotation(m.START_QUAT_PANE[h]) for h in m.HALVES}
        self.held = dict.fromkeys(fixture.ARMS)  # arm -> (half, origin in gripper frame, rotation in gripper frame)
        self.events = []
        self.grade = grading.WindowGrade()
        self.out = None
        self.tcp = None

    def on_glass(self, half):
        return grading.on_glass(self.origin[half], self.rot[half]) != 0

    def step(self, values, tick):
        before = {h: self.origin[h].copy() for h in m.HALVES}
        tcps = []
        for i, arm in enumerate(fixture.ARMS):
            world_tcp, world_rot = ik.tcp_pose(values[7 * i:7 * i + 6], fixture.BASES[arm])
            tcps.append(world_tcp)
            tcp, rot = m.to_pane(world_tcp), m.PANE_ROT.T @ world_rot
            closed = values[7 * i + 6] < .5
            if not closed:
                self.empty.discard(arm)
            if closed and self.held[arm] is None and arm not in self.empty:
                self.empty.add(arm)
                for half in m.HALVES:
                    local = self.rot[half].T @ (tcp - self.origin[half]) - m.GRIP_POINT  # from the bar's middle
                    if abs(local[0]) < .030 and np.hypot(local[1], local[2]) < .012:  # the pads are 20 mm wide
                        if arm in self.miss:
                            self.miss.discard(arm)
                            self.events.append((tick, f"{arm} closes on {half} but does not take hold"))
                            continue
                        self.empty.discard(arm)
                        self.held[arm] = (half, rot.T @ (self.origin[half] - tcp), rot.T @ self.rot[half])
                        self.events.append((tick, f"{arm} grasps {half}"))
            if not closed and self.held[arm] is not None:
                half = self.held[arm][0]
                self.held[arm] = None
                other = next(h for h in m.HALVES if h != half)
                paired = self.on_glass(half) and self.on_glass(other) and \
                    np.linalg.norm((self.origin[half] - self.origin[other])[[0, 2]]) < grading.PAIR_OFFSET
                down = self.origin[half][2] < .010 and self.rot[half][2, 2] > .99  # set down flat on the table
                self.events.append((tick, f"{arm} lays the {half} half down" if down else
                                    f"{arm} lets go of {half}" + ("" if paired else " and it drops")))
                if down:
                    self.origin[half][2] = 0.
                elif not paired:
                    indoors = self.origin[half][1] > m.WALL_MID_N  # it lands on the side of the wall it was let go on
                    self.origin[half] = np.array((self.origin[half][0], max(self.origin[half][1], .10) if indoors
                                                  else min(self.origin[half][1], -.10), 0.))
                    self.rot[half] = m.rotation(m.START_QUAT_PANE[half])
            if self.held[arm] is not None:
                half, offset, turn = self.held[arm]
                origin, self.rot[half] = tcp + rot @ offset, rot @ turn
                side = m.SIDE[half]
                if side * origin[1] < m.PANE_T / 2 and m.PANE_X[0] < origin[0] < m.PANE_X[1]:
                    origin[1] = side * m.PANE_T / 2  # the pad stops on the glass
                self.origin[half] = origin
        holders = {held[0] for held in self.held.values() if held}
        for half in m.HALVES:  # a let-go half on the glass trails the held one
            other = next(h for h in m.HALVES if h != half)
            if half not in holders and other in holders and self.on_glass(half) and self.on_glass(other):
                way = np.sign(self.origin[other][0] - before[other][0])  # dragged along behind the held half
                target = self.origin[other] - (self.lag * way, 0., 0.)
                if way > 0:
                    target[0] = max(target[0], self.origin[half][0])
                elif way < 0:
                    target[0] = min(target[0], self.origin[half][0])
                else:
                    target[0] = self.origin[half][0]
                if np.linalg.norm((target - self.origin[half])[[0, 2]]) < .03:
                    self.origin[half][[0, 2]] = target[[0, 2]]
        pose = np.array([np.r_[m.to_world(self.origin[h]), quat(m.PANE_ROT @ self.rot[h])] for h in m.HALVES])
        velocity = np.array([(self.origin[h] - before[h]) * fixture.CONTROL_HZ for h in m.HALVES])
        touching = [h in holders for h in m.HALVES]
        self.tcp = np.array(tcps)
        self.out = self.grade.update(pose, velocity, self.tcp, touching)
        return self.out


def reader(world):
    """What the adapter's ``state`` gives the demonstration, from the stand-in."""
    def read():
        out = {h: (world.origin[h].copy(), world.rot[h].copy()) for h in m.HALVES}
        out["paired"] = bool(world.out["paired"]) if world.out else False
        return out
    return read


def play(reference=None, lag=0., world=None, blind=False):
    world = world or World()
    reference = reference or scripted.WindowReference()
    if not blind:
        reference.read = reader(world)
    trace, scores, last, idle = [], [], None, 0
    for chunk in reference.sequence():
        values = np.asarray(chunk.actions[0].data, dtype=float)
        measured = values.copy()
        measured[[1, 8]] += lag
        reference.measured = measured
        trace.append(values)
        scores.append(world.step(values, len(trace))["score"])
        idle = idle + 1 if last is not None and np.allclose(values, last) else 0
        last = values
        if scores[-1] == 1. or idle >= IDLE_STOP or len(trace) >= fixture.MAX_STEPS:
            break
    return reference, world, np.array(trace), scores


PHASES = ["reach", "grasp", "lift", "turn", "seat_inside", "through_opening", "along_outside",
          "set_outside_on_glass", "slide_outside_on", "release_outside", "stroke", "release_inside"]
EVENTS = ["left grasps inside", "right grasps outside", "right lets go of outside", "left lets go of inside"]
TILE = np.c_[m.TRIANGLE, np.zeros(3)]


class DemoDryRunTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference, cls.world, cls.trace, cls.scores = play()
        cls.tcp = {arm: np.array([ik.tcp_pose(row[7 * i:7 * i + 6], fixture.BASES[arm])[0] for row in cls.trace])
                   for i, arm in enumerate(fixture.ARMS)}
        cls.tcp_pane = {arm: m.to_pane(cls.tcp[arm]) for arm in fixture.ARMS}

    def test_script_only_reads_the_simulator(self):
        # It may read the halves' poses (through ``read``); it has no handle with which to move anything.
        source = Path(scripted.__file__).read_text()
        for word in ("set_transforms", "set_velocities", "apply_forces", "body_view", "torch", "omni", "pxr", "scene["):
            self.assertNotIn(word, source)
        self.assertEqual(scripted.WindowReference.__init__.__code__.co_varnames[:2], ("self", "read"))

    def test_full_score_through_the_real_scorer(self):
        self.assertEqual(self.scores[-1], 1.)
        self.assertEqual(self.world.out["area_cleaned_fraction"], 1.)
        self.assertIsNone(self.world.out["violation"])
        self.assertEqual(sorted(set(self.scores)), [0., .25, 1.])
        self.assertEqual([text for _, text in self.world.events], EVENTS)
        report = self.reference.report
        self.assertTrue(report["left_lifted"] and report["right_lifted"])
        self.assertTrue(report["paired_before_release"] and report["paired_after_release"] and report["paired_after_stroke"])
        for key in ("left_grasped_again", "right_grasped_again", "slid_on_again"):
            self.assertNotIn(key, report)

    def test_full_score_without_reading_the_simulator(self):
        reference, world, trace, scores = play(blind=True)
        self.assertEqual(scores[-1], 1.)
        self.assertEqual([text for _, text in world.events], EVENTS)
        self.assertGreaterEqual(world.out["area_cleaned_fraction"], grading.FULL_AREA)

    def test_fits_the_run_limit(self):
        self.assertLess(len(self.trace), fixture.MAX_STEPS - 1000)  # at least 50 s to spare

    def test_every_target_is_reachable(self):
        self.assertLess(self.reference.worst_ik, .001)
        for i in (0, 7):
            self.assertTrue(np.all(self.trace[:, i:i + 6] >= ik.LOWER - 1e-9))
            self.assertTrue(np.all(self.trace[:, i:i + 6] <= ik.UPPER + 1e-9))
            # Per 50 ms step: shoulder joints under 0.06 rad (1.2 of their 5.5 rad/s), wrist joints under
            # 0.12 rad (2.4 of their 20 rad/s). There is no wrist flip anywhere in the script.
            step = np.abs(np.diff(self.trace[:, i:i + 6], axis=0)).max(axis=0)
            self.assertLess(step[:3].max(), .06)
            self.assertLess(step[3:].max(), .12)
        self.assertTrue(np.all((self.trace[:, [6, 13]] >= 0.) & (self.trace[:, [6, 13]] <= 1.)))

    def test_every_joint_keeps_clear_of_its_limits(self):
        # 0.20 rad or more on every joint of both arms for the whole script.
        for i in (0, 7):
            margin = np.minimum(self.trace[:, i:i + 6] - ik.LOWER, ik.UPPER - self.trace[:, i:i + 6])
            self.assertGreater(margin.min(), .20)

    def test_phases_are_recorded_in_order(self):
        names = [name for name, _ in self.reference.phases]
        self.assertEqual(names, PHASES)
        starts = [start for _, start in self.reference.phases]
        self.assertEqual(starts, sorted(starts))
        self.assertEqual(starts[0], 0)

    def test_grasp_and_seat_points(self):
        for arm in fixture.ARMS:
            half = scripted.HALF[arm]
            self.assertLess(np.linalg.norm(self.tcp[arm] - scripted.PICK[arm], axis=1).min(), .0005)
            seat = scripted.glass_point_pane(arm, scripted.MATE_U, -scripted.PRESS)
            self.assertLess(np.linalg.norm(self.tcp_pane[arm] - seat, axis=1).min(), .0005)
            self.assertAlmostEqual(abs(seat[1]), m.PANE_T / 2 + m.GRIP_POINT[2] - scripted.PRESS)
            # The fingers always close across the bar's 24 mm width: level on the table, vertical on the glass.
            table = m.PANE_ROT.T @ scripted.TABLE_GRIP[arm]
            glass = m.PANE_ROT.T @ scripted.GLASS_GRIP[arm]
            np.testing.assert_allclose(np.abs(table[:, 0]), (0., 1., 0.), atol=1e-9)
            np.testing.assert_allclose(np.abs(glass[:, 0]), (0., 0., 1.), atol=1e-9)
            for rot in (scripted.TABLE_GRIP[arm], scripted.GLASS_GRIP[arm], scripted.PASS_GRIP, scripted.SWING_GRIP):
                np.testing.assert_allclose(rot.T @ rot, np.eye(3), atol=1e-9)
                self.assertAlmostEqual(np.linalg.det(rot), 1.)
        # Left: straight down on the table, square at the indoor face on the glass.
        np.testing.assert_allclose((m.PANE_ROT.T @ scripted.TABLE_GRIP["left"])[:, 2], (0., 0., -1.), atol=1e-9)
        np.testing.assert_allclose((m.PANE_ROT.T @ scripted.GLASS_GRIP["left"])[:, 2], (0., -1., 0.), atol=1e-9)
        # Right: along the bar towards +u, 10 degrees down on the table and 10 degrees into the outdoor face
        # on the glass, holding the bar 25 mm from its middle towards the end the hand comes from.
        c, s = np.cos(scripted.RIGHT_PITCH), np.sin(scripted.RIGHT_PITCH)
        self.assertAlmostEqual(np.degrees(scripted.RIGHT_PITCH), 10.)
        np.testing.assert_allclose((m.PANE_ROT.T @ scripted.TABLE_GRIP["right"])[:, 2], (c, 0., -s), atol=1e-9)
        np.testing.assert_allclose((m.PANE_ROT.T @ scripted.GLASS_GRIP["right"])[:, 2], (c, s, 0.), atol=1e-9)
        np.testing.assert_allclose(scripted.GRIP_LOCAL["right"] - m.GRIP_POINT, (-.025, 0., 0.), atol=1e-12)
        # The fingertips, 13 mm past the centre point and 10 mm either side of it, pass above the shell.
        self.assertGreater(m.GRIP_POINT[2] - .013 * s - .010 - m.BODY_T, .004)
        # Both closing targets ask for less than the 50 N grip limit (5000 N/m finger drives).
        self.assertLess(5000. * scripted.SQUEEZE, 50.)
        # The halves meet with the whole pad on the glass, 20 mm from the stile, and well short of the band.
        self.assertGreaterEqual(scripted.MATE_U - m.PAD_BASE / 2 - m.PANE_X[0], .020 - 1e-9)
        self.assertLess(scripted.MATE_U + m.PAD_BASE / 2 + .150, m.BAND_U[0])
        # The stroke ends with the pad's 100 mm wide part past the band's end and the whole pad on the glass.
        self.assertGreater(scripted.END_U + .050, m.BAND_U[1] + .005)
        self.assertLess(scripted.END_U + m.PAD_BASE / 2 + .010, m.PANE_X[1])

    def test_outside_half_is_set_on_the_glass_away_from_the_other_and_slid_on(self):
        start, stop = (dict(self.reference.phases)[name] for name in ("slide_outside_on", "release_outside"))
        right = self.tcp_pane["right"]
        # When it first touches the glass it is 55 mm below the inside half, where the pull is under 0.5 N.
        self.assertAlmostEqual(scripted.SET_ON_Z, m.WORK_Z - scripted.SLIDE_ON)
        first = scripted.glass_point_pane("right", scripted.MATE_U, -scripted.PRESS, scripted.SET_ON_Z)
        np.testing.assert_allclose(right[start - 1], first, atol=.0005)
        self.assertLess(abs(m.hold_force(dz=-scripted.SLIDE_ON)[0]), .5)
        self.assertGreater(scripted.SET_ON_Z - m.PAD_HEIGHT / 3, m.PANE_CLEAR_Z[0] + .05)  # all on the glass
        # Then it only moves up the glass: no motion along the wall or off the glass, never faster than 3 cm/s.
        slide = right[start - 1:stop]
        self.assertLess(np.ptp(slide[:, 0]), .0005)
        self.assertLess(np.ptp(slide[:, 1]), .0005)
        self.assertTrue(np.all(np.diff(slide[:, 2]) >= -1e-9))
        self.assertLess(np.abs(np.diff(slide[:, 2])).max() * fixture.CONTROL_HZ, .030)
        self.assertAlmostEqual(slide[-1, 2], m.WORK_Z, places=3)
        # The inside half was already seated when the outside half reached the glass.
        left = self.tcp_pane["left"]
        np.testing.assert_allclose(left[dict(self.reference.phases)["set_outside_on_glass"]],
                                   scripted.glass_point_pane("left", scripted.MATE_U, -scripted.PRESS), atol=.0005)

    def test_only_the_right_gripper_goes_outdoors_and_only_through_the_open_light(self):
        left, right = self.tcp_pane["left"], self.tcp_pane["right"]
        self.assertGreater(left[:, 1].min(), m.WALL_N[1] + .020)  # the left gripper's centre stays indoors
        side = np.sign(right[:, 1] - m.WALL_MID_N)
        crossings = np.flatnonzero(np.diff(side) != 0)
        self.assertEqual(len(crossings), 2)  # out once, back in once
        for i in crossings:
            for point in (right[i], right[i + 1]):
                self.assertTrue(grading.in_opening(point))
                self.assertGreater(min(point[0] - m.OPENING_U[0], m.OPENING_U[1] - point[0]), .080)
                self.assertGreater(min(point[2] - m.OPENING_Z[0], m.OPENING_Z[1] - point[2]), .080)
        self.assertIsNone(self.world.out["violation"])
        # Outdoors the gripper's centre goes no further along the wall than where it sets the half down.
        outdoors = right[side < 0]
        self.assertLess(outdoors[:, 0].max(), scripted.MATE_U)
        self.assertLess(scripted.MATE_U + m.PAD_BASE / 2 + .150, m.BAND_U[0])

    def test_held_halves_clear_the_table_and_the_window_frame(self):
        reference, world = scripted.WindowReference(), World()
        reference.read = reader(world)
        lowest, tightest = 1., 1.
        for tick, chunk in enumerate(reference.sequence()):
            values = np.asarray(chunk.actions[0].data, dtype=float)
            reference.measured = values
            out = world.step(values, tick)
            for half in m.HALVES:
                points = world.origin[half] + TILE @ world.rot[half].T  # the pad's corners, pane coordinates
                lowest = min(lowest, points[:, 2].min())
                edge = np.array([points[i] + t * (points[(i + 1) % 3] - points[i]) for i in range(3) for t in np.linspace(0., 1., 16)])
                in_wall = edge[(edge[:, 1] > m.WALL_N[0] - .010) & (edge[:, 1] < m.WALL_N[1] + .010)]
                over_glass = (in_wall[:, 0] > m.PANE_X[0]) & (in_wall[:, 0] < m.PANE_X[1])
                for point in in_wall[~over_glass]:  # the pad's outline, where it is within the wall's thickness
                    tightest = min(tightest, point[0] - m.OPENING_U[0], m.OPENING_U[1] - point[0],
                                   point[2] - m.OPENING_Z[0], m.OPENING_Z[1] - point[2])
            if out["score"] == 1.:
                break
        self.assertGreater(lowest, -1e-6)
        self.assertGreater(tightest, .030)  # the half's outline passes 30 mm or more inside the open light

    def test_ends_open_and_clear(self):
        # The trial ends as soon as the cleaner has been let go, paired and still for 2 s; the right arm is
        # at rest by then and the left arm is on its way back.
        np.testing.assert_allclose(self.tcp["right"][-1], scripted.REST["right"], atol=.001)
        np.testing.assert_allclose(self.trace[-1, [6, 13]], 1.)
        self.assertTrue(self.world.out["hands_free"])
        for half in m.HALVES:
            grip = self.world.origin[half] + self.world.rot[half] @ m.GRIP_POINT
            for arm in fixture.ARMS:
                self.assertGreater(np.linalg.norm(self.tcp_pane[arm][-1] - grip), grading.CLEAR_OF_GRIPPER + .02)

    def test_script_rests_both_arms_if_left_to_run(self):
        reference = scripted.WindowReference()
        values = None
        for tick, chunk in enumerate(reference.sequence()):
            values = np.asarray(chunk.actions[0].data, dtype=float)
            reference.measured = values
            if tick > 1200:
                break
        for i, arm in enumerate(fixture.ARMS):
            np.testing.assert_allclose(ik.tcp_pose(values[7 * i:7 * i + 6], fixture.BASES[arm])[0], scripted.REST[arm], atol=.001)
            self.assertLess(m.to_pane(scripted.REST[arm])[1], .35)
            self.assertGreater(m.to_pane(scripted.REST[arm])[1], m.WALL_N[1] + .05)  # indoors
            for u in (scripted.MATE_U, scripted.END_U):  # well clear of the cleaner wherever it was left
                handle = m.to_world(scripted.glass_point_pane(arm, u, 0.))
                self.assertGreater(np.linalg.norm(scripted.REST[arm] - handle), .15)

    def test_tracking_correction_stays_bounded_when_the_arm_sags(self):
        reference, world, trace, scores = play(lag=-.02)
        self.assertLess(reference.worst_ik, .002)
        self.assertLess(len(trace), fixture.MAX_STEPS - 1000)
        self.assertIsNone(world.out["violation"])
        # Pressing on the glass is never integrated: the commanded point stays within the press depth.
        for i, arm in enumerate(fixture.ARMS):
            pane = m.to_pane(np.array([ik.tcp_pose(row[7 * i:7 * i + 6], fixture.BASES[arm])[0] for row in trace]))
            over_glass = pane[:, 0] > m.PANE_X[0]
            reach = np.abs(pane[over_glass, 1]).min() if arm == "left" else np.abs(pane[over_glass & (pane[:, 1] < 0), 1]).min()
            self.assertGreater(reach, m.PANE_T / 2 + m.GRIP_POINT[2] - scripted.PRESS - .0005)

    def test_without_the_stroke_the_score_stops_at_half(self):
        reference = scripted.WindowReference()
        reference.stroke = lambda: iter(())
        _, world, _, scores = play(reference)
        self.assertEqual(scores[-1], .5)
        self.assertEqual(world.out["area_cleaned_fraction"], 0.)

    def test_a_stroke_cut_short_gets_partial_credit(self):
        reference = scripted.WindowReference()
        full = scripted.END_U
        try:
            scripted.END_U = m.BAND_U[0] + .040  # stop with about half the band wiped
            _, world, _, scores = play(reference)
        finally:
            scripted.END_U = full
        self.assertEqual(scores[-1], .75)
        self.assertGreaterEqual(world.out["area_cleaned_fraction"], grading.HALF_AREA)
        self.assertLess(world.out["area_cleaned_fraction"], grading.FULL_AREA)

    def test_outside_half_alone_is_not_let_go(self):
        # The inside half is never put on the glass, so the halves cannot pair.
        def without_inside(reference):
            set_on_glass = reference.set_on_glass
            reference.set_on_glass = lambda arm, z, seconds: set_on_glass(arm, z, seconds) if arm == "right" else iter(())
            return reference
        # Reading the simulator, the demonstration sees that and brings the outside half back in.
        reference, world, _, scores = play(without_inside(scripted.WindowReference()))
        self.assertEqual(scores[-1], 0.)
        self.assertEqual(reference.report["gave_up"], "the halves did not pair")
        self.assertEqual({text for _, text in world.events[-2:]}, {"left lays the inside half down", "right lays the outside half down"})
        self.assertGreater(world.origin["outside"][1], m.WALL_N[1])  # indoors
        # Without the readings it lets go, and the half drops to the table outdoors: nothing holds it up.
        reference, world, _, scores = play(without_inside(scripted.WindowReference()), blind=True)
        self.assertEqual(scores[-1], 0.)
        self.assertIn("right lets go of outside and it drops", [text for _, text in world.events])
        self.assertLess(world.origin["outside"][1], m.WALL_N[0])  # it lies outdoors, beyond the wall

    def test_a_grasp_that_did_not_take_hold_is_made_again(self):
        for arm, half in (("right", "outside"), ("left", "inside")):
            reference, world, trace, scores = play(world=World(miss=(arm,)))
            self.assertEqual(scores[-1], 1.)
            self.assertTrue(reference.report[f"{arm}_grasped_again"])
            self.assertTrue(reference.report[f"{arm}_lifted"])
            texts = [text for _, text in world.events]
            self.assertIn(f"{arm} closes on {half} but does not take hold", texts)
            self.assertIn(f"{arm} grasps {half}", texts)
            self.assertLess(len(trace), fixture.MAX_STEPS - 800)

    def test_halves_that_did_not_pair_are_slid_on_again(self):
        world, reference = World(), scripted.WindowReference()
        read, told = reader(world), []

        def doubtful():  # the first check after the slide is told the halves are not paired
            state = read()
            if reference.phases[-1][0] == "slide_outside_on" and "slid_on_again" not in reference.report:
                told.append(reference.steps)
                state["paired"] = False
            return state
        reference.read = doubtful
        _, world, trace, scores = play(reference, world=world, blind=True)
        self.assertTrue(told and reference.report["slid_on_again"])
        self.assertTrue(reference.report["paired_before_release"])
        self.assertNotIn("gave_up", reference.report)
        self.assertEqual(scores[-1], 1.)

    def test_stroke_waits_for_an_outside_half_that_falls_behind(self):
        reference, world, trace, scores = play(world=World(lag=.012))
        self.assertGreater(reference.report["stroke_waits"], 0)
        self.assertEqual(scores[-1], 1.)
        self.assertAlmostEqual(reference.report["outside_u_after_stroke_mm"], 1000 * scripted.END_U, delta=2.5)
        self.assertEqual(self.reference.report["stroke_waits"], 0)  # none at the usual 3 mm

    def test_gives_up_tidily_when_a_half_cannot_be_picked_up(self):
        class Slippery(World):  # the right gripper never takes hold
            def step(self, values, tick):
                self.miss.add("right")
                return super().step(values, tick)
        reference, world, trace, scores = play(world=Slippery())
        self.assertEqual(reference.report["gave_up"], "could not pick up the outside half")
        self.assertEqual(scores[-1], 0.)
        self.assertLess(reference.worst_ik, .001)
        self.assertEqual(reference.phases[-1][0], "gave_up")
        np.testing.assert_allclose(trace[-1, [6, 13]], 1.)  # both grippers open at the end
        np.testing.assert_allclose(world.origin["inside"][:2], m.START_UN["inside"], atol=.002)  # laid back down
        self.assertIsNone(world.out["violation"])

    def test_gives_up_and_brings_the_outside_half_back_if_the_halves_will_not_pair(self):
        world, reference = World(), scripted.WindowReference()
        read = reader(world)

        def never():
            state = read()
            state["paired"] = False
            return state
        reference.read = never
        _, world, trace, scores = play(reference, world=world, blind=True)
        self.assertEqual(reference.report["gave_up"], "the halves did not pair")
        self.assertTrue(reference.report["slid_on_again"])
        self.assertLess(reference.worst_ik, .001)
        for half in m.HALVES:  # both are back indoors where they lay, not dropped outdoors
            np.testing.assert_allclose(world.origin[half][:2], m.START_UN[half], atol=.002)
        self.assertEqual({text for _, text in world.events[-2:]}, {"left lays the inside half down", "right lays the outside half down"})
        self.assertIsNone(world.out["violation"])

    def test_phase_diagnostics_table(self):
        try:
            from rdb_window_wall import run
        except ImportError as error:  # pragma: no cover
            self.skipTest(f"needs inspect_robots: {error}")
        trace = [{"score": 0., "magnet_force_n": float(i), "pair_offset_mm": 1., "cells_cleaned": []} for i in range(10)]
        rows = run.phase_diagnostics(trace, [("a", 0), ("b", 4), ("never", 20)])
        self.assertEqual([(r["phase"], r["first_step"], r["last_step"], r["peak_magnet_force_n"]) for r in rows],
                         [("a", 1, 4, 3.), ("b", 5, 10, 9.)])
        self.assertNotIn("cells_cleaned", rows[0])
        self.assertEqual(run.phase_diagnostics([], [("a", 0)]), [])
        # Without phases (a model's trial) there is one row per 15 s.
        self.assertEqual(len(run.phase_diagnostics(trace * 70, None)), 3)

    def test_gripper_reading_check(self):
        reference = scripted.WindowReference()
        reference.measured[6] = scripted.BAR_FINGER / scripted.OPEN  # closed all the way to the target: empty
        reference.note_grip("left")
        self.assertFalse(reference.report["left_holding"])
        reference.measured[6] = (m.BAR_SIZE[1] / 2) / abs(scripted.OPEN)  # stopped on the 24 mm bar
        reference.note_grip("left")
        self.assertTrue(reference.report["left_holding"])


if __name__ == "__main__":
    unittest.main()
