"""A kinematic stand-in for the simulator, for dry runs of the demonstration and of the scorer. No contact physics.

- Each arm follows its joint commands with a first-order lag; fingertip points come from the arm's kinematics.
- The drawer moves only while the right hand's closed jaws are on the pull's lip; it then follows the hand.
- The latch arm goes down as far as a fingertip pressing on it from above pushes it (never past its stop) and
  is back up at once when nothing presses it. While the hook is up, the drawer cannot open past the catch.

This checks reach, joint limits, control flow and the scorer's rules. It says nothing about contact forces.
"""

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

from rdb_child_latch import fixed_scene as fixture
from rdb_child_latch import layout as L
from rdb_child_latch.scripted import GRIP_BACK, TIP

TIP_HALF_WIDTH = .006        # a fingertip's end is about 12 mm wide; tipped, its lowest corner leads the middle
FINGER_HALF = .0025           # a fingertip's middle is this far outside its inner face
CLOSED = .3                   # a gripper command below this counts as closed


class World:
    def __init__(self, *, lag_s=.12, press_offset=(0., 0., 0.), pull_offset=0., hold_angle=None, sag=0.):
        self.blend = 1. - np.exp(-1. / fixture.CONTROL_HZ / lag_s)
        self.press_offset = np.asarray(press_offset, dtype=float)     # where the pressing hand really is vs its joints
        self.pull_offset = pull_offset                                # the same for the pulling hand, along x
        self.hold_angle = hold_angle                                  # if set: the hook frees only past this angle
        self.joints = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        # Joint tracking error that the joint readings show (the arms sag under their weight): the demonstration
        # sees it through the arm's kinematics and must servo it out.
        self.bias = np.zeros(14)
        self.bias[[2, 9]] = sag
        self.target = self.joints.copy()
        self.open = L.OPEN_START
        self.angle = 0.
        self.gripped = False
        self.touching = False
        self.bottomed = 0.            # deepest a fingertip went below the arm at its stop (m), for the report

    # -- geometry ---------------------------------------------------------------------------
    def front(self):
        return L.drawer_x(self.open)

    def hand(self, index):
        return ik.tcp_pose(self.joints[7 * index:7 * index + 6], fixture.BASES[fixture.ARMS[index]])

    def fingertip_points(self, index):
        """Ends of the two fingertips of one hand."""
        point, rotation = self.hand(index)
        middle = point + TIP * rotation[:, 2]
        half = abs(float(self.joints[7 * index + 6]) * FINGER_OPEN) + FINGER_HALF
        return [middle + sign * half * rotation[:, 0] for sign in (1., -1.)]

    def boxes(self):
        """(low, high) of the drawer's boxes and the latch arm where they are now, tabletop frame (the latch
        arm as the box round it at its present angle)."""
        from build_scene import drawer_boxes, latch_boxes

        front = np.array((self.front(), 0., 0.))
        out = [(np.asarray(low) + front, np.asarray(high) + front) for _, low, high in drawer_boxes()]
        hinge = L.hinge_point(self.open)
        c, s = np.cos(self.angle), np.sin(self.angle)
        turn = np.array(((c, 0., s), (0., 1., 0.), (-s, 0., c)))
        for _, low, high in latch_boxes():
            corners = np.array([(x, y, z) for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])])
            points = corners @ turn.T + hinge
            out.append((points.min(axis=0), points.max(axis=0)))
        return out

    def state(self):
        return {"open": self.open, "latch_angle": self.angle}

    # -- one control step -------------------------------------------------------------------
    def _press(self):
        """Latch angle from the pressing fingertips (left hand)."""
        hinge = L.hinge_point(self.open)
        angle = 0.
        self.touching = False
        width = self.hand(0)[1][:, 1]
        corner = -TIP_HALF_WIDTH * width * np.sign(width[2])           # toward the lower end of the tip
        for p in self.fingertip_points(0):
            p = p + self.press_offset + corner
            reach = p[0] - hinge[0]
            if not (0. < reach < L.LATCH_LENGTH) or abs(p[1] - L.LATCH_Y) > L.LATCH_WIDTH / 2:
                continue
            top = hinge[2] + L.ARM_T / 2
            if p[2] < top:
                self.touching = True
                wanted = float(np.arcsin(np.clip((top - p[2]) / reach, 0., 1.)))
                if wanted > L.LATCH_TRAVEL:
                    self.bottomed = max(self.bottomed, top - reach * np.sin(L.LATCH_TRAVEL) - p[2])
                angle = max(angle, min(wanted, L.LATCH_TRAVEL))
        self.angle = angle

    def free(self):
        """Whether the hook lets the drawer past the catch."""
        need = L.RELEASE_ANGLE if self.hold_angle is None else self.hold_angle
        return self.angle >= need or self.open > L.catch_open() + .001

    def step(self, command):
        command = np.asarray(command, dtype=float)
        self.target = self.target + self.blend * (command - self.target)
        self.joints = self.target + self.bias
        self._press()
        tips = self.fingertip_points(1)
        middle = (tips[0] + tips[1]) / 2 + (self.pull_offset, 0., 0.)
        closed = float(self.joints[13]) < CLOSED
        on_lip = (self.front() - L.PULL_SIZE[0] - .004 <= middle[0] <= self.front() + .001
                  and abs(middle[1]) <= L.PULL_SIZE[1] / 2 and abs(middle[2] - L.PULL_Z) <= .008)
        self.gripped = closed and (self.gripped or on_lip)
        if self.gripped and not (closed and abs(middle[2] - L.PULL_Z) <= .012):
            self.gripped = False
        if self.gripped:
            self.joints[13] = max(self.joints[13], L.PULL_SIZE[2] / .094)     # the lip keeps the jaws apart
            # The lip is held in the jaws: the drawer goes where the hand is. The arm's give under the runners'
            # friction is between the command and the hand, and the lag stands in for it.
            self.touching = True
            target = float(np.clip(L.FRAME_FRONT_X - (middle[0] + GRIP_BACK), 0., L.OPEN_LIMIT))
            if target > L.catch_open() and self.open <= L.catch_open() + .001 and not self.free():
                target = min(target, L.catch_open())
            self.open = target
        return self.joints.copy()

    def grade_inputs(self):
        tcp = [self.hand(index)[0] for index in range(2)]
        return self.open, tcp, self.touching
