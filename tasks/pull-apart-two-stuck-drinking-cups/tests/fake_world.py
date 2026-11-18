"""A kinematic stand-in for the simulator, for dry runs of the demonstration. No contact physics: a cup
follows a gripper that has closed on it and stays where it is let go."""

import numpy as np
from yam_arena import ik

from rdb_nested_cups import cup_model as cup, fixed_scene as fixture, scripted
from rdb_nested_cups.grading import nested as grading_nested

OPEN = abs(scripted.OPEN)


class FakeWorld:
    def __init__(self, *, failed_grasps=0, never_hold=False, nudge=(0., 0., 0.)):
        self.cups = np.array([[*cup.OUTER_START, 0., 0., 0., 1.], [*cup.INNER_START, 0., 0., 0., 1.]])
        self.cups[:, :3] += nudge  # the pair is not exactly where the setup says
        self.failed_grasps, self.never_hold = failed_grasps, never_hold
        self.left_holds = self.right_holds = False
        self.grab = None
        self.left_closed_before = False

    def step(self, commanded):
        """Apply one 14-D action; return the 14-D joint reading the robot would report."""
        measured = np.array(commanded, dtype=float)
        left = ik.tcp_pose(commanded[:6], fixture.BASES["left"])[0]
        right = ik.tcp_pose(commanded[7:13], fixture.BASES["right"])[0]
        # right gripper on the outer cup
        on_outer = np.linalg.norm(right[:2] - self.cups[0, :2]) < .008 and abs(right[2] - scripted.OUTER_GRIP_HEIGHT) < .01
        surface = cup.outside_radius(scripted.OUTER_GRIP_HEIGHT) / OPEN
        self.right_holds = bool(on_outer and commanded[13] < surface)
        if self.right_holds:
            measured[13] = surface
        # left gripper pinching the inner cup's rim wall: its centre point over the middle of the wall
        rim = self.cups[1, :3] + (0., 0., cup.HEIGHT)
        wall = cup.BAND_THICKNESS / 2. / OPEN
        closing = commanded[6] < wall
        if not self.left_holds and closing and not self.left_closed_before:
            over_wall = abs(np.linalg.norm(left[:2] - rim[:2]) - scripted.PINCH_RADIUS) < .004 and abs(left[2] - rim[2]) < .008
            if over_wall and not self.never_hold:
                if self.failed_grasps > 0:
                    self.failed_grasps -= 1
                else:
                    self.left_holds, self.grab = True, self.cups[1, :3] - left
        self.left_closed_before = closing
        if self.left_holds and commanded[6] > .95:
            self.left_holds = False
            if not self.right_holds and grading_nested(self.cups):  # a lifted stuck pair settles back on the table
                self.cups[:, 2] -= self.cups[0, 2] - cup.OUTER_START[2]
            if self.cups[1, 2] < .006:
                self.cups[1, 2] = 0.
        if self.left_holds:
            measured[6] = cup.BAND_THICKNESS / 2. / OPEN
            target = left + self.grab
            if not self.right_holds:  # nothing holds the outer cup back: the stuck pair comes up together
                self.cups[0, :3] += target - self.cups[1, :3]
            self.cups[1, :3] = target
        return measured


def play(world, reference, limit=fixture.MAX_STEPS):
    trace = []
    for chunk in reference.sequence():
        values = np.asarray(chunk.actions[0].data, dtype=float)
        reference.measured = world.step(values)
        trace.append(values)
        if (reference.phases and reference.phases[-1]["phase"] == "right_parked") or len(trace) >= limit:
            break
    return np.array(trace)
