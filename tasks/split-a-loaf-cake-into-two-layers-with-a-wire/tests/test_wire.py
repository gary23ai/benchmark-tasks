"""The wire and cake model on its own: what cuts, what does not, and what the wire cannot pass through."""

import unittest

import numpy as np

import conftest  # noqa: F401
from rdb_cake_wire import cake_model as cake

try:
    from rdb_cake_wire import fixed_scene as fixture
except ImportError as error:  # pragma: no cover
    raise unittest.SkipTest(f"needs yam_arena for the station constants: {error}")

from loose_handles import LooseHandles, first_segment_tenth

EYE = np.eye(3)
WHOLE = ((cake.CAKE_CENTRE, EYE), (cake.TOP_LAYER_CENTRE, EYE), True)
FAR = cake.CAKE_CENTRE[0] + cake.CAKE_HALF[0]
NEAR = cake.CAKE_CENTRE[0] - cake.CAKE_HALF[0]
HALF = cake.WIRE_LENGTH / 2


class Hands:
    """Two wire ends moved by hand, mirrored about the midline."""

    def __init__(self, x, y, z, boxes=(), capsules=()):
        self.wire = fixture.make_wire()
        s = np.linspace(0., 1., fixture.WIRE_SEGMENTS + 1)[:, None]
        self.at = np.array((x, y, z), dtype=float)
        self.wire.reset((1. - s) * self.at + s * self.at * (1., -1., 1.))
        self.boxes, self.capsules = boxes, capsules
        self.peak_push, self.peak_tension = 0., 0.

    def to(self, x, y, z, seconds, substeps=3):
        start, end = self.at.copy(), np.array((x, y, z), dtype=float)
        steps = max(1, round(seconds * 20))
        for i in range(1, steps + 1):
            self.at = start + i / steps * (end - start)
            for _ in range(substeps):
                _, _, push = self.wire.step(self.at, self.at * (1., -1., 1.), WHOLE, fixture.floor,
                                            1 / (20 * substeps), self.boxes, self.capsules)
            self.peak_push = max(self.peak_push, float(np.linalg.norm(push.sum(axis=0)[:2])))
            self.peak_tension = max(self.peak_tension, float(self.wire.tension.max()))
        return self

    def draw_through(self, z, speed=.015):
        """Lay the straight wire against the far face at height z and draw it out of the near face."""
        self.to(FAR - .004, HALF + .0025, z, 1.)
        radius = HALF - cake.CAKE_HALF[1] + .002           # swing round the far corners, the wire kept tight
        for angle in np.radians(np.linspace(0., 25., 11)[1:]):
            self.to(FAR - radius * np.sin(angle), cake.CAKE_HALF[1] + radius * np.cos(angle), z, .4)
        x = self.at[0]
        self.to(NEAR - .10, self.at[1], z, (x - NEAR + .10) / speed)
        return self


class CutTest(unittest.TestCase):
    def test_a_taut_level_draw_cuts_the_top_layer_free_at_the_modelled_force(self):
        hands = Hands(FAR + .03, HALF + .0025, cake.SEAM_Z).draw_through(cake.SEAM_Z)
        on_line, off_line = hands.wire.cut_fraction()
        self.assertGreaterEqual(on_line, .99)
        self.assertEqual(off_line, 0.)
        self.assertEqual(hands.wire.side_cut(), (.185, .185))
        # Yield 16 N/m over 90 mm, plus the rate term at 15 mm/s.
        self.assertGreater(hands.peak_push, cake.CUT_FORCE)
        self.assertLess(hands.peak_push, 1.35 * cake.CUT_FORCE)
        self.assertLess(hands.peak_push, .6 * cake.CAKE_SLIDE_FORCE)      # the block stays put
        self.assertLess(hands.peak_tension, 8.)                            # well inside a 30 N finger squeeze

    def test_a_cut_4_mm_off_the_line_counts_and_one_8_mm_off_does_not(self):
        on_line, off_line = Hands(FAR + .03, HALF + .0025, cake.SEAM_Z + .004).draw_through(cake.SEAM_Z + .004).wire.cut_fraction()
        self.assertGreaterEqual(on_line, .99)
        on_line, off_line = Hands(FAR + .03, HALF + .0025, cake.SEAM_Z - .008).draw_through(cake.SEAM_Z - .008).wire.cut_fraction()
        self.assertEqual(on_line, 0.)
        self.assertGreaterEqual(off_line, .99)

    def test_a_fast_pull_would_drag_the_block(self):
        hands = Hands(FAR + .03, HALF + .0025, cake.SEAM_Z).draw_through(cake.SEAM_Z, speed=.25)
        self.assertGreater(hands.peak_push, cake.CAKE_SLIDE_FORCE)

    def test_a_wire_pressed_gently_against_the_cake_does_not_cut(self):
        hands = Hands(FAR + .03, HALF + .0005, cake.SEAM_Z)      # about 1.5 N of tension
        hands.to(FAR - .003, HALF + .0005, cake.SEAM_Z, 2.)      # 3 mm past the face, hands still wide apart
        hands.to(FAR - .003, HALF + .0005, cake.SEAM_Z, 2.)
        self.assertEqual(hands.wire.cut_fraction(), (0., 0.))
        local_x = hands.wire.x[:, 0]
        inside = np.abs(hands.wire.x[:, 1]) < cake.CAKE_HALF[1] - .005
        self.assertGreater(local_x[inside].min(), FAR - .0005)   # held at the surface, not inside the cake

    def test_a_slack_wire_does_not_cut(self):
        hands = Hands(FAR + .03, .10, cake.SEAM_Z)               # hands 105 mm closer than the wire is long
        hands.to(FAR + .03, .10, cake.SEAM_Z, 1.)
        hands.to(FAR - .03, .10, cake.SEAM_Z, 3.)                # not far enough to draw it tight round the block
        self.assertEqual(hands.wire.cut_fraction()[0], 0.)
        self.assertLess(hands.peak_push, .5 * cake.CUT_FORCE)

    def test_a_cut_through_the_top_layer_spoils_the_cake_and_frees_nothing(self):
        top = cake.CAKE_CENTRE[2] + cake.CAKE_HALF[2]
        hands = Hands(cake.CAKE_CENTRE[0], HALF + .0025, top + .02)
        hands.to(cake.CAKE_CENTRE[0], HALF + .0025, top - .012, 4.)
        hands.to(NEAR - .12, HALF, top - .012, 20.)
        on_line, off_line = hands.wire.cut_fraction()
        self.assertLess(on_line, .1)
        self.assertGreater(off_line, .05)

    def test_cut_height_is_recorded_per_cell_and_the_kerf_is_free(self):
        hands = Hands(FAR + .03, HALF + .0025, cake.SEAM_Z).draw_through(cake.SEAM_Z)
        wire = hands.wire
        self.assertTrue(np.allclose(wire.on_z[wire.on], cake.SEAM_LOCAL, atol=fixture.LINE_TOLERANCE))
        # Drawn straight again and passed back through the kerf, the wire meets no cake.
        hands.to(NEAR - .10, HALF + .0025, cake.SEAM_Z, 1.)
        hands.peak_push = 0.
        hands.to(FAR + .03, HALF + .0025, cake.SEAM_Z, 4.)
        self.assertLess(hands.peak_push, .2)


class SolidsTest(unittest.TestCase):
    def test_wire_rests_on_the_table_and_the_stand(self):
        wire = fixture.make_wire()
        wire.reset(fixture.wire_start() + (0., 0., .05))
        a, b = (cake.HANDLE_START[arm] for arm in fixture.ARMS)
        for _ in range(400):
            wire.step(a, b, WHOLE, fixture.floor, 1 / 240)
        self.assertGreater(wire.x[:, 2].min(), -.0005)
        self.assertLess(wire.x[1:-1, 2].max(), .002)
        self.assertLess(wire.x[:, 0].max(), cake.STAND_CENTRE[0] - cake.STAND_SIZE[0] / 2)    # clear of the stand
        self.assertGreater(wire.x[:, 0].min(), cake.HANDLE_START["left"][0] - 1e-6)           # bowed away from the robot
        # Dropped across the stand it lies on top of it.
        nodes = np.linspace((.40, .15, .08), (.40, -.15, .08), fixture.WIRE_SEGMENTS + 1)
        wire.reset(nodes)
        far = ((np.array((9., 9., 9.)), EYE), (np.array((9., 9., 9.)), EYE), True)
        for _ in range(400):
            wire.step(nodes[0] * (1., 1., 0.) + (0., 0., .0005), nodes[-1] * (1., 1., 0.) + (0., 0., .0005), far,
                      fixture.floor, 1 / 240)
        middle = np.abs(wire.x[:, 1]) < .06
        self.assertTrue(np.all(wire.x[middle, 2] > cake.STAND_TOP - .0005))

    def test_wire_is_turned_aside_by_a_box_and_a_capsule(self):
        box = (np.array((.30, 0., .10)), EYE, np.array((.003, .008, .04)))       # the size of a finger
        capsule = (np.array((.30, -.03, .10)), np.array((.30, .03, .10)), cake.HANDLE_RADIUS, slice(0, 0))
        for boxes, capsules in (([box], ()), ((), [capsule])):
            hands = Hands(.36, .145, .10, boxes, capsules)
            hands.to(.27, .145, .10, 3.)                                          # sweep the wire across the solid
            wire = hands.wire
            self.assertGreater(wire.x[fixture.WIRE_SEGMENTS // 2, 0], .30)        # the middle is caught on it
            if boxes:
                q = np.abs(wire.x - box[0]) - box[2]
                gap = np.linalg.norm(np.maximum(q, 0.), axis=1) + np.minimum(q.max(axis=1), 0.)
            else:
                gap = np.linalg.norm(wire.x[:, [0, 2]] - (.30, .10), axis=1) - cake.HANDLE_RADIUS
                gap = gap[np.abs(wire.x[:, 1]) < .03]
            self.assertGreater(gap.min(), .002)

    def test_a_moved_top_layer_is_still_cake(self):
        wire = fixture.make_wire()
        moved = cake.TOP_LAYER_CENTRE + (0., .10, 0.)
        state = ((cake.CAKE_CENTRE, EYE), (moved, EYE), False)
        points = np.array((moved, cake.TOP_LAYER_CENTRE, cake.BASE_CENTRE))
        wire.reset(np.zeros((fixture.WIRE_SEGMENTS + 1, 3)))
        _, in_cake, _, in_top_layer = wire.state(points, state)
        self.assertEqual(in_cake.tolist(), [True, False, True])      # where the top_layer was is now empty
        self.assertEqual(in_top_layer.tolist(), [True, False, False])


class ForceLimitTest(unittest.TestCase):
    """Nothing the wire touches is pulled or pushed harder than fixed_scene.END_PULL_LIMIT."""

    def test_no_segment_carries_more_than_the_limit(self):
        hands = Hands(.25, HALF, .15)
        pulls = None
        for stretch in np.linspace(0., .04, 41)[1:]:                 # to 80 mm over length: 120 N in a plain spring
            for _ in range(3):
                pulls = hands.wire.step(np.array((.25, HALF + stretch, .15)), np.array((.25, -HALF - stretch, .15)),
                                        WHOLE, fixture.floor, 1 / 240)
            self.assertLessEqual(hands.wire.tension.max(), fixture.END_PULL_LIMIT + 1e-9)
            self.assertTrue(np.isfinite(hands.wire.x).all())
        self.assertGreater(float(hands.wire.tension.min()), fixture.END_PULL_LIMIT - .01)   # it did reach it
        for pull in pulls[:2]:
            self.assertAlmostEqual(float(np.linalg.norm(pull)), fixture.END_PULL_LIMIT, delta=.01)
            self.assertLessEqual(float(np.linalg.norm(pull)), fixture.END_PULL_LIMIT + 1e-9)
        # Below the limit the spring is the plain one: 10 mm over length at 1,500 N/m is 15 N.
        hands = Hands(.25, HALF + .005, .15)
        hands.to(.25, HALF + .005, .15, .2)
        self.assertAlmostEqual(float(hands.wire.tension.max()), fixture.WIRE_STIFFNESS * .010, delta=.05)

    def test_force_passed_to_a_handle(self):
        limit, whole = fixture.END_PULL_LIMIT, fixture.WIRE_SEGMENTS
        force = lambda newtons, run, held: float(np.linalg.norm(fixture.handle_force(np.array((0., newtons, 0.)), run, held)))
        self.assertEqual(force(5., whole, True), 5.)                 # a held handle gets the whole pull
        self.assertEqual(force(500., whole, True), limit)            # up to the limit
        self.assertEqual(force(500., 1, False), force(40., 1, False))
        # A loose handle on a wire taut from end to end gets about a tenth (a 144 N/m wire).
        self.assertAlmostEqual(force(5., whole, False) / 5., fixture.LOOSE_STIFFNESS / fixture.WIRE_STIFFNESS, delta=.001)
        self.assertAlmostEqual(fixture.LOOSE_STIFFNESS, 144.)
        # A short taut length is stiffer, so less of its pull is passed on: explicit steps of the 5 g handle stay
        # stable whatever length of wire it hangs on (stiffness x step^2 / mass no more than a half).
        for run in (1, 3, 16, 64):
            passed = force(5., run, False) / 5.
            stiffness = fixture.WIRE_STIFFNESS * whole / run * passed
            self.assertLessEqual(stiffness / fixture.PHYSICS_HZ ** 2 / cake.HANDLE_MASS, .5 + 1e-9)
        # The weight of the wire hanging from a loose handle's middle (0.1 to 0.3 mN) is not passed on at all.
        self.assertEqual(force(.0003, 2, False), 0.)
        self.assertEqual(force(.5 * fixture.HANDLE_REST_PULL, whole, False), 0.)
        self.assertGreater(force(.5, whole, False), cake.HANDLE_FRICTION * cake.HANDLE_MASS * 9.81)   # 0.5 N still drags it
        self.assertEqual(force(0., 0, False), 0.)
        # The push on the cake is limited in the same way.
        push = np.tile((30., 0., 0.), (4, 1))
        self.assertAlmostEqual(float(np.linalg.norm(fixture.limited(push).sum(axis=0))), limit)
        np.testing.assert_array_equal(fixture.limited(push / 100.), push / 100.)

    def test_pull_is_taken_where_the_wire_leaves_its_own_handle(self):
        # The wire is fixed to the middle of a handle and bent sharply back over the handle's own length, so
        # that its first nodes are pressed against the handle. The first segment is then taut between the middle
        # and the node the handle pushes away, but the hand feels only the wire beyond that node.
        a, b = np.array((.25, .10, .15)), np.array((.25, -.10, .15))
        axis = np.array((0., 1., 0.))                                # the handle lies along the wire's line
        capsule = (a - .0265 * axis, a + .0265 * axis, cake.HANDLE_RADIUS, slice(0, 2))
        wire = fixture.make_wire()
        s = np.linspace(0., 1., fixture.WIRE_SEGMENTS + 1)[:, None]
        wire.reset((1. - s) * a + s * b)
        for _ in range(60):
            pull, _, _ = wire.step(a, b, WHOLE, fixture.floor, 1 / 240, (), [capsule])
        self.assertGreater(wire.leave[0], 0)                         # nodes 2.. lie against the handle
        self.assertEqual(wire.leave[1], fixture.WIRE_SEGMENTS)
        first = wire.tension[0]
        self.assertGreater(first, 10. * max(float(np.linalg.norm(pull)), 1e-6))   # the internal pull is not passed on
        self.assertLess(float(np.linalg.norm(pull)), .01)


class LooseHandleTest(unittest.TestCase):
    """The wire and the two handles with nobody holding them (see loose_handles.py)."""

    def test_an_untouched_wire_lies_still_for_90_s(self):
        world = LooseHandles().run(60.)
        handles, wire = world.moved()
        nodes = world.wire.x.copy()
        world.run(30.)
        self.assertAlmostEqual(world.time, 90., places=6)
        self.assertEqual(world.moved()[0], 0.)                       # neither handle moved at all
        self.assertEqual(world.peak_force, 0.)                       # because no force was passed to them
        self.assertLessEqual(world.peak_force, fixture.END_PULL_LIMIT)
        self.assertLess(world.peak_tension, .001)                    # N: the weight of the wire beside each handle
        self.assertLessEqual(world.peak_tension, fixture.END_PULL_LIMIT)
        # Nor did the wire: no node is 0.05 mm from where it lay at the start, or moved 0.01 mm in the last 30 s.
        self.assertLess(world.moved()[1], .00005)
        self.assertLess(np.linalg.norm(world.wire.x - nodes, axis=1).max(), .00001)
        self.assertLess(wire, .00005)
        self.assertEqual(handles, 0.)
        self.assertEqual(world.wire.cut_fraction(), (0., 0.))

    def test_a_nudged_handle_does_not_pull_itself_along_its_own_wire(self):
        # A steady 0.2 mN sideways (the weight of 40 mm of the wire) rolls the left handle toward the right one
        # and over its own wire. With the coupling as first built (the first segment's pull, a tenth of it,
        # no limit inside the wire) the handle's push on the wire came back as a pull on the handle: within 6 s
        # it ran away and threw both handles, as happened at step 787 of the first Dream-1 trial.
        before = LooseHandles(first_segment_tenth, np.inf).run(6., push=(-2e-4, 0.))
        self.assertGreater(before.peak_tension, 10.)                 # N, in a wire nobody is pulling
        self.assertGreater(before.peak_force, 1.)                    # N, on a 5 g handle: 20 times its weight
        self.assertGreater(float(np.linalg.norm(before.p - before.start, axis=1).min()), .15)   # both thrown
        # As shipped, the same nudge rolls the handle against the wire and nothing more.
        world = LooseHandles().run(6., push=(-2e-4, 0.))
        self.assertLess(world.peak_tension, .1)
        self.assertLess(world.peak_force, .002)
        self.assertLess(np.abs(world.v).max(), .06)                  # m/s: rolling under the nudge, not thrown
        self.assertEqual(float(np.linalg.norm(world.p[1] - world.start[1])), 0.)   # the other handle is left alone


if __name__ == "__main__":
    unittest.main()
