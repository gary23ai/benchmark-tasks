"""Diagnostic probe: a series of slow test closes, to find out what this gripper can hold and how.

Used only when cup_model.PROBE is on. Like the demonstration it follows fixed waypoints and reads only the
robot's joint angles and gripper openings; all simulator-side logging happens in adapter.py. Each test:
move above the target, come down open, close slowly (4 s unless stated), hold 2 s, lift 15 mm, hold 1 s,
lower, open, go back up. Phase names are ``<test>:<stage>``.

Tests, in order:
  right_side_outer   right gripper, side-on, on the outer cup 40 mm up (the grip that has held in every run)
  cyl80_slow         left gripper, top-down, across a rigid 80 mm cylinder
  cyl80_fast         the same with the demonstration's 0.62 s close
  cyl66_slow         left gripper, top-down, across a rigid 66 mm cylinder
  box80_slow         left gripper, top-down, across a rigid box 80 mm wide (flat faces)
  nested_slow        left gripper, top-down, across the OUTER cup's top 19 mm, where the inner cup backs its wall
  rim_y_slow         left gripper, top-down, across the inner cup's rim band, closing along y (panel flats)
  rim_x_slow         the same turned 90 degrees, closing along x
  rim_diag_slow      the same turned 7.5 degrees, so the pads meet the joints between panels
  pinch_slow         left gripper, top-down, one finger inside the inner cup and one outside, closing on its wall
"""

import numpy as np
from yam_arena.constants import PAD_TIP_PAST_TCP
from yam_arena.demo import grip_rotation

from . import cup_model as cup
from .scripted import DOWN, LEFT, OPEN, OUTER_FINGER, RIGHT, SIDE_GRIP, SQUEEZE, CupReference
from .fixed_scene import ARMS, PARK

STACK = np.array((*cup.STACK_XY, 0.))
INNER_GRIP = STACK + (0., 0., cup.INNER_START[2] + cup.HEIGHT)
OUTER_GRIP = STACK + (0., -.003, .040)
OUTER_STAGE = np.array((cup.STACK_XY[0], -.125, .105))
HOVER = .055
LIFT = .015


def top_of(name):
    half, radius, height, _, base = cup.PROBE_BODIES[name]
    return np.array((base[0], base[1], height)), (radius if half is None else half[1])


class ProbeReference(CupReference):
    def close_test(self, name, arm, point, rotation, half_width, *, close_s=4., lift=True):
        """One test close of ``arm`` with its gripper centre at ``point``; ``half_width`` is half the object's
        width between the pads (the finger target is SQUEEZE inside it, or fully closed for a thin wall)."""
        finger = -max(half_width - SQUEEZE, 0.)
        self.report[f"{name}_finger_target_opening"] = round(float(finger / OPEN), 4)
        self.report[f"{name}_expected_opening_if_held"] = round(float(half_width / abs(OPEN)), 4)
        yield from self.move(1.2, {arm: point + (0., 0., HOVER)}, orientation={arm: rotation}, jaws={arm: OPEN})
        yield from self.move(.3)
        for clearance in (.030, .012, 0.):
            yield from self.track(arm, point + (0., 0., clearance), .020)
        self.mark(f"{name}:open_at_target")
        yield from self.move(close_s, jaws={arm: finger})
        self.mark(f"{name}:closed")
        yield from self.move(2.)
        self.mark(f"{name}:held_2s")
        self.report[f"{name}_opening_after_hold"] = round(self.opening(arm), 4)
        if lift:
            yield from self.track(arm, point + (0., 0., LIFT), .020)
            yield from self.move(1.)
            self.mark(f"{name}:lifted_15mm")
            self.report[f"{name}_opening_after_lift"] = round(self.opening(arm), 4)
            yield from self.track(arm, point, .020)
        yield from self.move(.5, jaws={arm: OPEN})
        yield from self.move(.3)
        self.mark(f"{name}:released")
        yield from self.move(.9, {arm: self.tcp[arm] + (0., 0., HOVER)})

    def right_side_test(self):
        name = "right_side_outer"
        self.report[f"{name}_finger_target_opening"] = round(float(OUTER_FINGER / OPEN), 4)
        yield from self.move(1.1, {RIGHT: OUTER_STAGE})
        yield from self.move(1.8, orientation={RIGHT: SIDE_GRIP})
        yield from self.move(.3)
        yield from self.move(1.0, {RIGHT: OUTER_GRIP - .052 * SIDE_GRIP[:, 2]})
        for distance in (.052, .040, .028, .018, .009, 0.):
            yield from self.track(RIGHT, OUTER_GRIP - distance * SIDE_GRIP[:, 2], .040)
        self.mark(f"{name}:open_at_target")
        yield from self.move(4., jaws={RIGHT: OUTER_FINGER})
        self.mark(f"{name}:closed")
        yield from self.move(2.)
        self.mark(f"{name}:held_2s")
        self.report[f"{name}_opening_after_hold"] = round(self.opening(RIGHT), 4)
        yield from self.move(.5, jaws={RIGHT: OPEN})
        yield from self.move(.3)
        self.mark(f"{name}:released")
        yield from self.withdraw(RIGHT, .065, .65)
        yield from self.move(2.0, {RIGHT: PARK[RIGHT]}, orientation={RIGHT: DOWN})

    def sequence(self):
        yield from self.move(1.2, PARK, orientation=dict.fromkeys(ARMS, DOWN))
        yield from self.move(.55)
        self.mark("parked")
        yield from self.right_side_test()

        # Rigid test bodies: gripper centre 6 mm below the top, so the pads' flat lowest 18 mm are all on it.
        top, half = top_of("probe80")
        yield from self.close_test("cyl80_slow", LEFT, top - (0., 0., .006), DOWN, half)
        yield from self.close_test("cyl80_fast", LEFT, top - (0., 0., .006), DOWN, half, close_s=.62)
        top, half = top_of("probe66")
        yield from self.close_test("cyl66_slow", LEFT, top - (0., 0., .006), DOWN, half)
        top, half = top_of("probebox")
        yield from self.close_test("box80_slow", LEFT, top - (0., 0., .006), DOWN, half)

        # The cup pair, not held by the other hand.
        outer_rim = cup.OUTER_START[2] + cup.HEIGHT
        nested = STACK + (0., 0., outer_rim - .019 + PAD_TIP_PAST_TCP)  # pad tips 19 mm below the outer rim
        yield from self.close_test("nested_slow", LEFT, nested, DOWN, cup.outside_radius(cup.HEIGHT - .010))
        yield from self.close_test("rim_y_slow", LEFT, INNER_GRIP, DOWN, cup.RIM_RADIUS)
        yield from self.close_test("rim_x_slow", LEFT, INNER_GRIP, grip_rotation(-np.pi / 2), cup.RIM_RADIUS)
        yield from self.close_test("rim_diag_slow", LEFT, INNER_GRIP, grip_rotation(-np.pi / cup.SEGMENTS), cup.RIM_RADIUS)
        # One finger inside the inner cup, one outside: the gripper centre over the middle of the rim band's wall.
        wall_middle = cup.RIM_RADIUS - cup.BAND_THICKNESS / 2.
        yield from self.close_test("pinch_slow", LEFT, INNER_GRIP + (0., wall_middle, 0.), DOWN, cup.BAND_THICKNESS / 2.)

        yield from self.move(1.2, {LEFT: PARK[LEFT]}, orientation={LEFT: DOWN})
        self.mark("done")
        while True:
            yield from self.move(1.)
