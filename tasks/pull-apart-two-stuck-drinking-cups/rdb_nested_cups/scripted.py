"""Full-score demonstration, closed loop on simulator state.

The demonstration reads both cups' poses from the simulator (as a camera could) and the robot's own joint
angles and gripper openings. It uses them to place each grasp, to check that each grip holds, to pull until
the inner cup is actually out, to regrip if it slips, and to set the inner cup down upright. It never moves
an object except through the grippers.

Plan: first the left gripper alone pinches the inner cup's rim and lifts 10 mm, which lifts both cups (logged
as evidence that the pair is stuck), and sets them down again. Then the right gripper holds the outer cup
round its side; the left gripper pinches the inner cup's rim wall
from above, one finger inside the cup and one outside, at the rim point nearest the robot, which lies on the right gripper's grip axis; it pulls the
inner cup up and out, sets it down 15 cm to the left and lets go; then the right gripper lets go of the outer
cup and both arms park.

Why a wall pinch and not a squeeze across the rim: in five simulator runs the across-the-rim squeeze sank
into the cup and slipped off at about 10 N per finger, whatever the cup's colliders were, while a probe run
showed this wall pinch holding steadily.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import grip_rotation

from . import cup_model as cup
from .fixed_scene import ARMS, BASES, CONTROL_HZ, PARK

LEFT, RIGHT = ARMS
OUTER, INNER = 0, 1
OPEN = float(FINGER_OPEN)
DOWN = grip_rotation()
# The right gripper comes in from the robot's right, 7 degrees below horizontal, fingers either side of the cup.
# (At this angle the open left fingers pass 10 mm from the right hand on their way down; steeper is tighter.)
SIDE_GRIP = (Rotation.from_rotvec((1.45, 0, 0)) * Rotation.from_rotvec((0, 0, np.pi / 2))).as_matrix() @ DOWN

OUTER_GRIP_HEIGHT = .040  # up the outer cup's wall (72.9 mm across there), below the inner cup's rim band
OUTER_GRIP_BACK = .003  # the right gripper's centre point stops this far short of the cup's axis
STAGE_Y, STAGE_Z = -.160, .105  # where the right gripper turns side-on, relative to the cup in y
CLOSE_S = 3.  # a slow close: in the simulator a fast one can drive a finger through a cup wall
# Finger targets (m, each finger; 0 is closed). Each finger is a 5000 N/m position drive, so a target 7 mm
# inside the cup's surface squeezes with about 35 N (measured: 34 N), under the gripper's 50 N limit.
SQUEEZE = .007
OUTER_FINGER = -(cup.outside_radius(OUTER_GRIP_HEIGHT) - SQUEEZE)
# The left gripper closes fully on the inner cup's rim wall; the wall stops each finger half its thickness out.
INNER_FINGER = 0.
PINCH_RADIUS = cup.RIM_RADIUS - cup.BAND_THICKNESS / 2.  # middle of the rim band's wall, from the cup's axis
# Where on the rim to pinch, as an angle about the vertical from the rim's far-left point (+y); +90 degrees
# is the rim point nearest the robot (-x). The right gripper holds the outer cup by two pads on the x axis
# through the cup, and the cup can pivot about that axis in its grip. A pull at the far-left rim point tips
# the whole pair about it (seen in the simulator: 9 degrees, and the pinch slipped); a pull at a rim point ON
# that axis does not. So the first pinch is at +90 degrees, which is also where the arm reaches highest; a
# regrip moves on to the next entry.
PINCH_AT = tuple(np.radians((90., 70., 110., -25., 0., 90.)))
# A gripper that closed on nothing reaches its target; one that holds a cup stops at least this far short of it.
HOLDING_MARGIN = .003 / abs(OPEN)
CLEAR_OF_RIM = .015  # the inner cup's base is lifted this far above the outer cup's rim before it is carried
PULL_STEP = .005
SLIP_LIMIT = .012  # the hand has risen this much more than the cup: the pinch is slipping
HIGHEST_TCP = .192  # as high as the left arm reaches over the cups with its gripper pointing straight down
CARRY_Y = .150
SET_DOWN_GAP = .002  # the inner cup's base is let go this far above the table
MAX_REGRIPS = 5


class CupReference:
    """Emit one absolute 14-D joint action per control step.

    ``rig`` is the embodiment; ``rig.cups`` holds both cups' base centres (tabletop coordinates) and XYZW
    quaternions as of the last step. ``measured`` is the latest joint observation."""

    def __init__(self, rig):
        self.rig = rig
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.tcp, self.rotation = {}, {}
        for arm in ARMS:
            self.tcp[arm], self.rotation[arm] = ik.tcp_pose(self.joints[arm], BASES[arm])
        self.aperture = dict.fromkeys(ARMS, OPEN)
        self.worst_ik = 0.
        self.report = {"regrips": 0}
        self.steps = 0  # actions emitted so far
        self.phases = []  # one entry at each phase boundary

    # -- readings ------------------------------------------------------------------------
    def actual_tcp(self, arm):
        start = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.measured[start:start + 6], BASES[arm])[0]

    def opening(self, arm):
        """Measured gripper opening, 0 (closed) to 1 (open)."""
        return float(self.measured[7 * ARMS.index(arm) + 6])

    def holding(self, arm, finger_target):
        return self.opening(arm) > finger_target / OPEN + HOLDING_MARGIN

    def base(self, index):
        return np.array(self.rig.cups[index][:3], dtype=float)

    def axis(self, index):
        return Rotation.from_quat(np.asarray(self.rig.cups[index][3:], dtype=float)).apply((0., 0., 1.))

    def rim_centre(self, index):
        return self.base(index) + cup.HEIGHT * self.axis(index)

    def inner_above_outer(self):
        """Height of the inner cup's base above the outer cup's base, along the outer cup's axis."""
        return float((self.base(INNER) - self.base(OUTER)) @ self.axis(OUTER))

    def inner_is_out(self):
        return self.inner_above_outer() > cup.HEIGHT + .005

    def lowest_base_point(self, index):
        """Height above the table of the lowest point of a cup's base, allowing for tilt."""
        up = self.axis(index)
        return float(self.base(index)[2] - cup.BASE_RADIUS * np.hypot(up[0], up[1]))

    def mark(self, phase):
        entry = {"phase": phase, "step": self.steps,
                 "inner_above_outer_mm": round(1000 * self.inner_above_outer(), 2),
                 "inner_base_mm": (1000 * self.base(INNER)).round(1).tolist(),
                 "outer_base_mm": (1000 * self.base(OUTER)).round(1).tolist()}
        for arm in ARMS:
            entry[arm] = {"opening_measured": round(self.opening(arm), 4),
                          "opening_commanded": round(float(np.clip(self.aperture[arm] / OPEN, 0., 1.)), 4),
                          "tcp_measured_mm": (1000 * self.actual_tcp(arm)).round(2).tolist(),
                          "tcp_commanded_mm": (1000 * np.asarray(self.tcp[arm])).round(2).tolist()}
        self.phases.append(entry)

    # -- motion --------------------------------------------------------------------------
    def move(self, seconds, targets=None, *, orientation=None, jaws=None):
        targets, orientation, jaws = targets or {}, orientation or {}, jaws or {}
        arms = targets.keys() | orientation.keys() | jaws.keys()
        start = {a: (self.tcp[a], ik.matrix_to_quat_xyzw(self.rotation[a]), self.aperture[a]) for a in arms}
        end = {}
        for arm, (point, quat, aperture) in start.items():
            q = ik.matrix_to_quat_xyzw(orientation[arm]) if arm in orientation else quat
            end[arm] = (np.asarray(targets.get(arm, point)), q if quat @ q >= 0 else -q, jaws.get(arm, aperture))
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm, (point, quat, aperture) in start.items():
                target, q, opening = end[arm]
                self.tcp[arm] = point + alpha * (target - point)
                rotation = quat + alpha * (q - quat)
                rotation /= np.linalg.norm(rotation)
                self.rotation[arm] = Rotation.from_quat(rotation).as_matrix()
                self.joints[arm], error, _ = ik.solve(self.tcp[arm], rotation, self.joints[arm], BASES[arm], iters=35)
                self.worst_ik = max(self.worst_ik, error)
                self.aperture[arm] = aperture + alpha * (opening - aperture)
            values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
            self.steps += 1
            yield ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def track(self, arm, target, limit):
        """Two short moves that shift the command by the arm's measured error, at most ``limit`` each."""
        for _ in range(2):
            delta = np.clip(np.asarray(target) - self.actual_tcp(arm), -limit, limit)
            yield from self.move(.35, {arm: self.tcp[arm] + delta})

    def withdraw(self, arm, distance, seconds):
        yield from self.move(seconds, {arm: self.tcp[arm] - self.rotation[arm][:, 2] * distance})

    # -- the demonstration ---------------------------------------------------------------
    def hold_outer(self):
        """Right gripper: side-on grip of the outer cup where the simulator says it stands."""
        centre = self.base(OUTER)
        grip = np.array((centre[0], centre[1] - OUTER_GRIP_BACK, OUTER_GRIP_HEIGHT))
        self.mark("parked")
        yield from self.move(1.1, {RIGHT: (centre[0], centre[1] + STAGE_Y, STAGE_Z)})
        yield from self.move(1.8, orientation={RIGHT: SIDE_GRIP})
        yield from self.move(.30)
        yield from self.move(1.0, {RIGHT: grip - .052 * SIDE_GRIP[:, 2]})
        for distance in (.052, .040, .028, .018, .009, 0.):
            yield from self.track(RIGHT, grip - distance * SIDE_GRIP[:, 2], .040)
        self.mark("right_at_outer_cup_open")
        yield from self.move(CLOSE_S, jaws={RIGHT: OUTER_FINGER})
        yield from self.move(.50)
        self.report["right_opening_after_close"] = round(self.opening(RIGHT), 4)
        self.report["right_holding"] = bool(self.holding(RIGHT, OUTER_FINGER))
        self.mark("right_closed_on_outer_cup")

    def take_inner(self):
        """Left gripper: pinch the inner cup's rim wall from above, one finger inside the cup and one outside,
        at the rim point the simulator's pose gives (the right gripper's squeeze shifts the pair a little)."""
        turn = PINCH_AT[min(self.report["regrips"], len(PINCH_AT) - 1)]
        outward = np.array((-np.sin(turn), np.cos(turn), 0.))  # from the cup's axis to the pinch point
        # The fingers close along ``outward``; the gripper is symmetric, so use the half turn the wrist reaches.
        rotation = grip_rotation(turn if turn <= np.pi / 4 else turn - np.pi)

        def point():
            return self.rim_centre(INNER) + PINCH_RADIUS * outward

        yield from self.move(1.12, {LEFT: point() + (0., 0., .055)}, orientation={LEFT: rotation}, jaws={LEFT: OPEN})
        yield from self.move(.32)
        for clearance in (.045, .030, .018, .008, 0.):
            yield from self.track(LEFT, point() + (0., 0., clearance), .018)
        self.mark("left_at_inner_rim_open")
        yield from self.move(CLOSE_S, jaws={LEFT: INNER_FINGER})
        yield from self.move(.50)
        self.report["left_pinch_turn_deg"] = round(float(np.degrees(turn)), 1)
        self.report["left_opening_after_close"] = round(self.opening(LEFT), 4)
        self.report["left_holding"] = bool(self.holding(LEFT, INNER_FINGER))
        self.mark("left_closed_on_inner_rim")

    def lift_test(self):
        """Before the other hand holds anything: pinch the inner cup's rim, lift 10 mm, and read from the
        simulator whether the outer cup came up with it. This shows that the pair is stuck (one hand alone
        only lifts both cups). Then set the pair back down and let go."""
        yield from self.take_inner()
        self.mark("lift_test_pinched")
        if not self.holding(LEFT, INNER_FINGER):
            self.report["lift_test"] = "pinch did not hold"
        else:
            hand, inner, outer, height = self.actual_tcp(LEFT), self.base(INNER)[2], self.base(OUTER)[2], self.inner_above_outer()
            for rise in (.005, .010):
                yield from self.track(LEFT, hand + (0., 0., rise), .008)
            yield from self.move(.5)
            self.report["lift_test"] = {
                "inner_rise_mm": round(float(1000 * (self.base(INNER)[2] - inner)), 1),
                "outer_rise_mm": round(float(1000 * (self.base(OUTER)[2] - outer)), 1),
                "inner_moved_in_outer_mm": round(float(1000 * (self.inner_above_outer() - height)), 2),
                "pair_lifted_together": bool(self.base(OUTER)[2] - outer > .006 and abs(self.inner_above_outer() - height) < .0005)}
            self.mark("lift_test_lifted")
            for rise in (.005, .001):
                yield from self.track(LEFT, hand + (0., 0., rise), .008)
        yield from self.move(1.0, jaws={LEFT: OPEN})
        yield from self.move(.3)
        yield from self.move(.9, {LEFT: self.tcp[LEFT] + (0., 0., .045)})
        for key in ("left_opening_after_close", "left_holding"):
            self.report[f"lift_test_{key}"] = self.report.pop(key)
        self.mark("lift_test_released")

    def regrip(self, reason):
        self.report["regrips"] += 1
        self.mark(reason)
        yield from self.move(.6, jaws={LEFT: OPEN})
        yield from self.move(.8, {LEFT: self.tcp[LEFT] + (0., 0., .030)})
        yield from self.take_inner()

    def pull_out(self):
        """Raise the left gripper 5 mm at a time until the inner cup's base is CLEAR_OF_RIM above the outer
        cup's rim. If the pinch is lost (the fingers have closed up) or is slipping (the hand has risen more
        than the cup), open, pinch again at the next rim point and go on, at most MAX_REGRIPS times."""
        hand_start, cup_start = self.actual_tcp(LEFT)[2], self.base(INNER)[2]
        while self.inner_above_outer() < cup.HEIGHT + CLEAR_OF_RIM:
            slipping = (self.actual_tcp(LEFT)[2] - hand_start) - (self.base(INNER)[2] - cup_start) > SLIP_LIMIT
            if slipping or not self.holding(LEFT, INNER_FINGER):
                if self.report["regrips"] >= MAX_REGRIPS:
                    self.mark("gave_up")
                    return
                yield from self.regrip("slipping" if slipping else "lost_grip")
                hand_start, cup_start = self.actual_tcp(LEFT)[2], self.base(INNER)[2]
                continue
            if self.actual_tcp(LEFT)[2] + PULL_STEP > HIGHEST_TCP:
                break
            yield from self.track(LEFT, self.actual_tcp(LEFT) + (0., 0., PULL_STEP), .008)
        self.report["left_opening_after_pull"] = round(self.opening(LEFT), 4)
        self.report["inner_above_outer_after_pull_mm"] = round(1000 * self.inner_above_outer(), 1)
        self.report["inner_tilt_after_pull_deg"] = round(float(np.degrees(np.arccos(np.clip(self.axis(INNER)[2], -1., 1.)))), 1)
        self.mark("pulled_out" if self.inner_is_out() else "pull_ended_not_out")

    def set_down(self):
        """Carry the inner cup 15 cm to the left of the outer cup and lower it until the simulator shows its
        base just above the table, then let go."""
        shift = np.r_[self.base(OUTER)[:2] + (0., CARRY_Y) - self.base(INNER)[:2], 0.]
        yield from self.move(1.5, {LEFT: self.tcp[LEFT] + shift})
        self.mark("left_carried_aside")
        for _ in range(12):
            height = self.lowest_base_point(INNER)
            if height <= SET_DOWN_GAP + .001 or not self.holding(LEFT, INNER_FINGER):
                break
            yield from self.track(LEFT, self.actual_tcp(LEFT) - (0., 0., min(height - SET_DOWN_GAP, .030)), .030)
        self.mark("left_lowered_to_table")
        yield from self.move(1.0, jaws={LEFT: OPEN})
        yield from self.move(.4)
        self.mark("left_released")
        yield from self.withdraw(LEFT, .07, 1.0)  # straight up: one finger is still inside the cup
        yield from self.move(.85, {LEFT: PARK[LEFT]}, orientation={LEFT: DOWN})
        self.mark("left_parked")

    def sequence(self):
        yield from self.move(1.2, PARK, orientation=dict.fromkeys(ARMS, DOWN))
        yield from self.move(.55)
        yield from self.lift_test()
        yield from self.hold_outer()
        yield from self.take_inner()
        yield from self.pull_out()
        if self.inner_is_out() and self.holding(LEFT, INNER_FINGER):
            yield from self.set_down()
        else:  # nothing useful in hand: open and park
            yield from self.move(.5, jaws={LEFT: OPEN})
            yield from self.move(1.2, {LEFT: self.tcp[LEFT] + (0., 0., .04)})
            yield from self.move(1.2, {LEFT: PARK[LEFT]}, orientation={LEFT: DOWN})
        yield from self.move(1.0, jaws={RIGHT: OPEN})
        self.mark("right_released")
        yield from self.withdraw(RIGHT, .065, .65)
        yield from self.move(2.0, {RIGHT: PARK[RIGHT]}, orientation={RIGHT: DOWN})
        self.mark("right_parked")
        while True:
            yield from self.move(1.)
