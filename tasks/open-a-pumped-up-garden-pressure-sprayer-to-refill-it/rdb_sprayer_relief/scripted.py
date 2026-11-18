"""Full-score demonstration: hold the sprayer down, let the pressure out through the safety valve, then unscrew
the spray head one and a half turns by its nozzle, half a turn at a time, the arms taking turns.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a
scripted demonstration, simulator state: the pressure number, how far the valve is out, how far the head is
unscrewed and where the tank stands. It changes nothing in the simulation: the valve moves only because the
gripper pulls its cap, the head turns only because a gripper leads its nozzle round.

One arm holds the sprayer: it pinches the pump knob from the side, the hand pitched down 35 degrees so its
fingers stay well above the trigger's sweep, and presses down (the knob spins freely on the pump rod, so this
holds the sprayer's top in place and loads it onto the mat without holding the head). The left arm pinches the valve's red cap, pulls it out and keeps it out until the pressure
is gone. Then, half a turn at a time, an arm pinches the nozzle from above and below (so the nozzle can turn
between the pads) and leads it round the sprayer's axis; the left arm works the left half of the circle, the
right arm the right half, and whichever arm is free holds the knob. Each stage is checked from the state and
repeated if it did not happen.

``variant`` (or RDB_SPRAYER_DEMO in the environment) selects a control: "no_vent" skips the valve and must
score 0 (the hazard); "vent_only" stops after the pressure is out and must score 0.3; "no_hold" does everything
except hold the sprayer (a physics control: the 5 N push on the nozzle, 265 mm up, tips the loose sprayer).
"""

import os

from inspect_robots import Action, ActionChunk
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY, SHOULDER, WRIST

from . import layout as L
from .fixed_scene import ARMS, BASES, CONTROL_HZ

OPEN = float(FINGER_OPEN)
DT = 1. / CONTROL_HZ
PAD_TIP = .013                         # the pads' tips are this far past the tool point
SIDE = {"left": 1., "right": -1.}
SEEDS = [np.array(s, dtype=float) for s in (
    (0, 1, 1, -.6, 0, 0), (-.35, 2.2, .8, 1.4, 1.2, -1.57), (.35, 2.2, .8, 1.4, -1.2, 0), (0, 2.1, 1.0, -.4, 1.5, -.1),
    (.4, 2.5, 1.4, -1.6, 1.1, 0), (0, 1.6, 1.3, -1.2, 0, 0), (0, 1.2, 1.6, .3, 0, 0))]

# holding the sprayer by its pump knob
BRACE_HEADING = np.radians(50.)        # the holding hand points across the sprayer, 50 degrees off straight ahead ...
BRACE_PITCH = np.radians(35.)          # ... and 35 degrees down, so its fingers stay at least 15 mm above the trigger
KNOB_Z = (L.KNOB_BOTTOM + L.HEIGHT) / 2          # pads on the middle of the knob's 13 mm rim
KNOB_PINCH = -(L.KNOB_R - 20. / 5000.)           # about 20 N a finger on the 48 mm knob
KNOB_HELD_MM = 2 * L.KNOB_R - .004               # pads stopped at least this far apart: the knob is between them
BRACE_FORCE = 7.                       # then pushed down this hard: the command is moved by the arm's compliance times
                                       # this force (about 60 mm down and 14 mm sideways), so the push is straight down
                                       # and does not itself shove the sprayer's top sideways

# the valve
VALVE_TIP_GAP = .0025                  # pad tips stop this far short of the shoulder
VALVE_PINCH = -(L.VALVE_CAP / 2 - 15. / 5000.)       # about 15 N a finger on the 16 mm cap
VALVE_APPROACH = -.020                 # pads 40 mm apart
VALVE_PULL = L.VALVE_TRAVEL + .002
VALVE_HELD_MM = .013                   # pads stopped at least this far apart: the cap is between them

# the nozzle
PINCH_R = .068                         # where on the nozzle it is pinched (its tip is at 78 mm)
NOZZLE_PINCH = -(L.NOZZLE_W / 2 - 15. / 5000.)       # about 15 N a finger on the 12 mm nozzle (it carries a 5 N push)
NOZZLE_APPROACH = -.018                # pads 36 mm apart, above and below
NOZZLE_RELEASE = -.012                 # pads 24 mm apart to let go (the other hand is just above)
NOZZLE_HELD_MM = .0095                # pads stopped at least this far apart: the nozzle is between them
TURN_RATE = np.radians(30.)            # how fast the nozzle is led round
LEAD = np.radians(15.)                 # the position command never runs further ahead of the nozzle than this (the arm
                                       # gives 8 to 11 mm under the 5 N it takes to turn the head)
# How the turning hand is set to the nozzle. The hand turns with the nozzle for the whole stroke, keeping the
# angle between its pointing direction and the nozzle's (ALPHA) fixed, so the nozzle never twists between the
# pads (run 2: a hand that did not turn with it dragged the tank round with the head). In the left arm's frame
# (the right arm's is its mirror image): the hand may point from 85 to 10 degrees right of straight ahead
# (HAND_RANGE), and the nozzle may be anywhere from 25 degrees right of straight ahead round the left to
# 205 degrees (NOZZLE_RANGE); these are where inverse kinematics finds the pose with 10 degrees or more to spare. The left arm leads the nozzle anticlockwise through its range, the right arm
# through the mirror image of it.
HAND_RANGE = (np.radians(-85.), np.radians(-10.))
NOZZLE_RANGE = (np.radians(-25.), np.radians(205.))
ALPHA_RANGE = (np.radians(78.), np.radians(280.))     # beyond these the palm swings in toward the head
MIN_STROKE = np.radians(12.)


def grip_frame(approach, closing):
    """Gripper rotation (columns: closing direction, pad width, approach) from two directions."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


def level(heading):
    """Level gripper pointing along ``heading`` (from +x, anticlockwise), one pad above the other."""
    return grip_frame((np.cos(heading), np.sin(heading), 0.), (0., 0., 1.))


JOINT_STIFFNESS = np.diag([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)     # N m/rad, the arm's joint drives


def compliance(joints, base):
    """Tool-point compliance (m/N) of an arm held at ``joints`` by its joint drives: J K^-1 J^T."""
    p0 = ik.tcp_pose(joints, base)[0]
    jac = np.zeros((3, 6))
    for i in range(6):
        step = np.zeros(6)
        step[i] = 1e-6
        jac[:, i] = (ik.tcp_pose(joints + step, base)[0] - p0) / 1e-6
    return jac @ np.linalg.inv(JOINT_STIFFNESS) @ jac.T


def wrap(angle, low):
    """``angle`` brought into [low, low + 360 degrees)."""
    return (angle - low) % (2 * np.pi) + low


def mirrored(arm, azimuth):
    """A world direction in the arm's own frame (the right arm's frame is the mirror image of the left's)."""
    return wrap(SIDE[arm] * azimuth, np.radians(-60.))


def stroke_plan(arm, azimuth):
    """(hand heading at the start in the arm's frame, ALPHA, how far the nozzle can be led) for a stroke that starts
    with the nozzle pointing along ``azimuth`` (world), or None if this arm cannot make a useful stroke from there."""
    m = mirrored(arm, azimuth)
    (h_lo, h_hi), (m_lo, m_hi), (a_lo, a_hi) = HAND_RANGE, NOZZLE_RANGE, ALPHA_RANGE
    if not m_lo <= m <= m_hi:
        return None
    if arm == "left":                                  # nozzle and hand both turn anticlockwise in this frame
        h0 = max(h_lo, m - a_hi)
        if m - h0 < a_lo:
            h0 = m - a_lo
        length = min(h_hi - h0, m_hi - m)
    else:                                              # in the right arm's (mirrored) frame they turn clockwise
        h0 = min(h_hi, m - a_lo)
        if m - h0 > a_hi:
            h0 = m - a_hi
        length = min(h0 - h_lo, m - m_lo)
    if not h_lo - 1e-9 <= h0 <= h_hi + 1e-9 or length < MIN_STROKE:
        return None
    return h0, m - h0, length


class Hand:
    def __init__(self, name):
        self.name = name
        self.base = BASES[name]
        self.joints = np.asarray(READY, dtype=float)
        self.aperture = OPEN
        self.point = None
        self.rot = None
        self.press = np.zeros(3)
        self.lag0 = np.zeros(3)
        self.stiff = np.zeros((3, 3))


class SprayerReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant=None):
        self.variant = variant or os.environ.get("RDB_SPRAYER_DEMO", "full")
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.hands = {arm: Hand(arm) for arm in ARMS}
        self.bracing = None
        self.worst_ik = 0.
        self.phases = []
        self.tick = 0
        self.log = {"events": [], "variant": self.variant}
        self.done = False

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        self.state = state

    def actual(self, arm):
        start = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.measured[start:start + 6], BASES[arm])[0]

    def opening(self, arm):
        """Distance between the pads from the gripper reading, metres."""
        return float(self.measured[7 * ARMS.index(arm) + 6]) * 2 * abs(OPEN)

    def world(self, local):
        """A point of the sprayer's frame in the tabletop frame, from the tank's measured pose."""
        return np.asarray(self.state["tank_pos"], dtype=float) + np.asarray(self.state["tank_rot"], dtype=float) @ np.asarray(local, dtype=float)

    def turn(self, direction):
        return np.asarray(self.state["tank_rot"], dtype=float) @ np.asarray(direction, dtype=float)

    def tank_yaw(self):
        rot = np.asarray(self.state["tank_rot"], dtype=float)
        return float(np.arctan2(rot[1, 0], rot[0, 0]))

    def nozzle_azimuth(self):
        return self.tank_yaw() + L.nozzle_azimuth(float(self.state["head"]))

    def note(self, name, **values):
        self.log["events"].append({"tick": self.tick, "event": name,
                                   **{k: (v if isinstance(v, str) else round(float(v), 4) if np.isscalar(v) else [round(float(x), 4) for x in v])
                                      for k, v in values.items()}})

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- motion ---------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.hands[a].joints, np.clip(self.hands[a].aperture / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def solve(self, arm, iters=40):
        hand = self.hands[arm]
        joints, error, _ = ik.solve(hand.point, ik.matrix_to_quat_xyzw(hand.rot), hand.joints, hand.base, iters=iters)
        self.worst_ik = max(self.worst_ik, error)
        hand.joints = joints

    def pose_joints(self, arm, point, rot):
        """Joint angles for a pose, from several starting guesses: the solution furthest from the joint limits."""
        best = None
        for seed in SEEDS:
            seed = seed * np.array((SIDE[arm], 1, 1, 1, SIDE[arm], SIDE[arm]))
            joints, error, turn = ik.solve(point, ik.matrix_to_quat_xyzw(rot), seed, BASES[arm], iters=300)
            if error < 5e-4 and turn < 5e-3:
                margin = float(np.minimum(joints - ik.LOWER, ik.UPPER - joints).min())
                if best is None or margin > best[1]:
                    best = (joints, margin)
        if best is None:
            raise RuntimeError(f"{arm} arm cannot reach {np.round(point, 3)}")
        return best[0]

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def glide(self, arm, seconds, goal):
        """Move one arm in joint space to the given joint angles (smooth start and stop)."""
        hand = self.hands[arm]
        start = hand.joints.copy()
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            hand.joints = start + alpha * (np.asarray(goal) - start)
            yield self.emit()

    def go(self, arm, seconds, point, rot):
        """Glide in joint space to a pose and make it the arm's commanded pose."""
        hand = self.hands[arm]
        yield from self.glide(arm, seconds, self.pose_joints(arm, point, rot))
        hand.point, hand.rot = np.asarray(point, dtype=float).copy(), np.asarray(rot, dtype=float).copy()

    def home(self, arm, seconds=2.2):
        yield from self.glide(arm, seconds, np.asarray(READY, dtype=float))
        self.hands[arm].point = self.hands[arm].rot = None

    def move(self, arm, seconds, point=None, rot=None):
        """Move the tool point in a straight line and turn the gripper evenly (smooth start and stop)."""
        hand = self.hands[arm]
        p0, r0 = hand.point.copy(), hand.rot.copy()
        p1 = p0 if point is None else np.asarray(point, dtype=float)
        turn = None if rot is None else Slerp([0., 1.], Rotation.from_matrix(np.stack((r0, rot))))
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            hand.point = p0 + alpha * (p1 - p0)
            if turn is not None:
                hand.rot = turn(alpha).as_matrix()
            self.solve(arm)
            yield self.emit()

    def trim(self, arm, want, seconds=.8, tolerance=.001):
        """Shift the command until the tool point measured from the joint readings is on ``want`` (the arm sags
        a little under its own weight)."""
        hand = self.hands[arm]
        want = np.asarray(want, dtype=float)
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            error = want - self.actual(arm)
            if np.linalg.norm(error) < tolerance:
                break
            hand.point = hand.point + np.clip(.3 * error, -.002, .002)
            self.solve(arm)
            yield self.emit()

    def set_gripper(self, arm, seconds, value):
        hand = self.hands[arm]
        start = hand.aperture
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            hand.aperture = start + step / steps * (value - start)
            yield self.emit()

    # -- holding the sprayer by its pump knob ------------------------------------------------------
    def knob_pose(self, arm):
        heading = -SIDE[arm] * BRACE_HEADING
        a = np.array((np.cos(heading) * np.cos(BRACE_PITCH), np.sin(heading) * np.cos(BRACE_PITCH), -np.sin(BRACE_PITCH)))
        closing = np.array((np.sin(heading), -np.cos(heading), 0.))
        centre = self.world((0., 0., KNOB_Z))
        return centre - a * (PAD_TIP / 2), grip_frame(a, closing), a

    def brace(self, arm):
        if self.variant == "no_hold":
            self.note("control: the sprayer is not held", arm=arm)
            return
        hand = self.hands[arm]
        for attempt in range(3):
            self.phase(f"hold the knob ({arm}) {attempt + 1}")
            grip, rot, a = self.knob_pose(arm)
            before = grip - a * .060
            yield from self.set_gripper(arm, .3, OPEN)
            if hand.point is None:
                yield from self.go(arm, 2.4, before, rot)
            else:
                yield from self.move(arm, 1., before, rot)
            yield from self.hold(.2)
            yield from self.trim(arm, before)
            yield from self.move(arm, 1.2, hand.point + (grip - before))
            yield from self.trim(arm, grip, tolerance=.001)
            yield from self.set_gripper(arm, .8, KNOB_PINCH)
            yield from self.hold(.4)
            held = self.opening(arm) >= KNOB_HELD_MM
            self.note("knob pinched", arm=arm, pads_mm=1000 * self.opening(arm), held=float(held),
                      tool_minus_wanted_mm=1000 * (self.actual(arm) - grip))
            if held:
                break
            yield from self.set_gripper(arm, .4, OPEN)
            yield from self.move(arm, .8, hand.point - a * .060)
        else:
            self.note("could not pinch the knob", arm=arm)
            return
        before_press = self.actual(arm)
        lag_before = hand.point - before_press
        stiff = np.linalg.inv(compliance(hand.joints, hand.base))
        hand.press = compliance(hand.joints, hand.base) @ np.array((0., 0., -BRACE_FORCE))
        yield from self.move(arm, 1.5, hand.point + hand.press)
        yield from self.hold(.3)
        self.bracing = arm
        hand.lag0, hand.stiff = lag_before, stiff
        self.note("holding the knob", arm=arm, press_command_mm=1000 * hand.press,
                  push_from_joint_readings_n=stiff @ ((hand.point - self.actual(arm)) - lag_before),
                  hand_went_down_mm=1000 * float(before_press[2] - self.actual(arm)[2]),
                  pads_mm=1000 * self.opening(arm), tank_tilt_deg=np.degrees(np.arccos(np.clip(self.state["tank_rot"][2, 2], -1., 1.))))

    def unbrace(self):
        arm = self.bracing
        if arm is None:
            return
        self.phase(f"let go of the knob ({arm})")
        hand = self.hands[arm]
        _, _, a = self.knob_pose(arm)
        yield from self.move(arm, .8, hand.point - hand.press)
        yield from self.set_gripper(arm, .5, OPEN)
        yield from self.hold(.2)
        yield from self.move(arm, 1., hand.point - a * .070)
        yield from self.home(arm, 2.0)
        self.bracing = None

    # -- the valve --------------------------------------------------------------------------------
    def vent(self, arm="left"):
        hand = self.hands[arm]
        for attempt in range(3):
            self.phase(f"to the valve {attempt + 1}")
            out = self.turn(L.valve_axis())
            across = self.turn((-np.sin(L.VALVE_AZ), np.cos(L.VALVE_AZ), 0.))
            rot = grip_frame(-out, across)
            grip = self.world(L.valve_base()) + out * (VALVE_TIP_GAP + PAD_TIP)
            before = grip + out * .050
            yield from self.set_gripper(arm, .3, VALVE_APPROACH)
            if hand.point is None:
                yield from self.go(arm, 2.6, before, rot)
            else:
                yield from self.move(arm, 1., before, rot)
            yield from self.hold(.2)
            yield from self.trim(arm, before)
            yield from self.move(arm, 1.2, hand.point + (grip - before))
            yield from self.trim(arm, grip, tolerance=.0008)
            self.note("at the valve", tool_minus_wanted_mm=1000 * (self.actual(arm) - grip))
            self.phase(f"pinch the cap {attempt + 1}")
            yield from self.set_gripper(arm, .8, VALVE_PINCH)
            yield from self.hold(.4)
            held = self.opening(arm) >= VALVE_HELD_MM
            self.note("cap pinched", pads_mm=1000 * self.opening(arm), held=float(held))
            self.phase(f"pull and hold {attempt + 1}")
            yield from self.move(arm, .8, hand.point + out * VALVE_PULL)
            waited, lost = 0., 0
            self.note("pulled", valve_mm=1000 * self.state["valve"], bar=self.state["bar"])
            while self.state["bar"] > L.SAFE_BAR / 2 and waited < 15. and lost < 6:
                lost = lost + 1 if self.state["valve"] < L.VALVE_OPEN else 0
                waited += DT
                yield self.emit()
            yield from self.hold(.4)
            self.note("held", seconds=waited, bar=self.state["bar"], valve_mm=1000 * self.state["valve"])
            self.phase(f"let the valve go {attempt + 1}")
            yield from self.move(arm, .5, hand.point - out * VALVE_PULL)
            yield from self.set_gripper(arm, .5, VALVE_APPROACH)
            yield from self.hold(.2)
            yield from self.move(arm, 1., hand.point + out * .050)
            if self.state["bar"] <= L.SAFE_BAR / 2:
                yield from self.home(arm)
                yield from self.set_gripper(arm, .2, OPEN)
                return True
        yield from self.home(arm)
        return False

    # -- the head -----------------------------------------------------------------------------------
    def pinch_pose(self, arm, azimuth, alpha, radius=PINCH_R):
        """Tool pose for pinching the nozzle at ``radius`` with it pointing along ``azimuth`` (world): the pads'
        middle on the nozzle, the hand level and pointing ALPHA away from the nozzle's direction."""
        yaw = self.tank_yaw()
        local = azimuth - yaw
        point = self.world((radius * np.cos(local), radius * np.sin(local), L.NOZZLE_Z))
        heading = SIDE[arm] * (mirrored(arm, azimuth) - alpha)
        along = np.array((np.cos(heading), np.sin(heading), 0.))
        return point - along * (PAD_TIP / 2), level(heading), along

    def stroke(self, arm, plan):
        """Pinch the nozzle and lead it round as far as ``plan`` allows, the hand turning with it. Returns True
        when the head moved."""
        hand = self.hands[arm]
        start_head = float(self.state["head"])
        _, alpha, length = plan
        for attempt in range(3):
            azimuth = self.nozzle_azimuth()
            self.phase(f"to the nozzle ({arm}) {attempt + 1}")
            grip, rot, along = self.pinch_pose(arm, azimuth, alpha)
            before = grip - along * .060
            yield from self.set_gripper(arm, .3, NOZZLE_APPROACH)
            # Between strokes the hand turns by up to 85 degrees. It goes back to the ready pose and comes in again in
            # joint space (to the pose with most room at the joint limits), as for the first stroke: straight-line
            # moves took the wrist to its stop, and shortcuts in joint space swept the fingers past the other hand.
            if hand.point is not None:
                yield from self.home(arm, 1.6)
            yield from self.go(arm, 2.4, before, rot)
            yield from self.hold(.2)
            yield from self.trim(arm, before)
            grip, rot, along = self.pinch_pose(arm, self.nozzle_azimuth(), alpha)     # re-read: the sprayer may have moved
            yield from self.move(arm, 1.2, grip, rot)
            yield from self.trim(arm, grip, tolerance=.0015)
            self.note("at the nozzle", arm=arm, nozzle_deg=np.degrees(azimuth), alpha_deg=np.degrees(alpha),
                      tool_minus_wanted_mm=1000 * (self.actual(arm) - grip))
            self.phase(f"pinch the nozzle ({arm}) {attempt + 1}")
            yield from self.set_gripper(arm, .8, NOZZLE_PINCH)
            yield from self.hold(.3)
            held = self.opening(arm) >= NOZZLE_HELD_MM
            self.note("nozzle pinched", arm=arm, pads_mm=1000 * self.opening(arm), held=float(held))
            moved = False
            if held:
                self.phase(f"turn the head ({arm}) {attempt + 1}")
                start = self.nozzle_azimuth()
                command, best, stalled = start, float(self.state["head"]), 0
                offset = hand.point - grip                 # the trim found at the pinch, kept for the whole stroke
                yaw0 = self.tank_yaw()
                while True:
                    head = float(self.state["head"])
                    now = self.nozzle_azimuth()
                    if head >= L.HEAD_FREE - np.radians(1.) or now - start >= length - np.radians(2.):
                        break
                    command = min(command + TURN_RATE * DT, now + LEAD, start + length)
                    # position on the commanded direction, pointing set from where the nozzle really is
                    point, _, _ = self.pinch_pose(arm, command, alpha)
                    _, hand.rot, _ = self.pinch_pose(arm, now, alpha)
                    hand.point = point + offset
                    self.solve(arm)
                    yield self.emit()
                    stalled = stalled + 1 if head <= best + np.radians(.3) else 0
                    best = max(best, head)
                    if stalled > 1.5 * CONTROL_HZ or self.opening(arm) < NOZZLE_HELD_MM - .002:
                        break
                yield from self.hold(.2)
                holder = self.hands[self.bracing] if self.bracing else None
                moved = float(self.state["head"]) > best - np.radians(1.) and float(self.state["head"]) > start_head + np.radians(5.)
                self.note("turned", holder_push_n=(holder.stiff @ ((holder.point - self.actual(self.bracing)) - holder.lag0)) if holder else [0., 0., 0.],
                          arm=arm, head_deg=np.degrees(self.state["head"]), nozzle_deg=np.degrees(self.nozzle_azimuth()),
                          led_deg=np.degrees(self.nozzle_azimuth() - start), pads_mm=1000 * self.opening(arm),
                          tank_yaw_deg=np.degrees(self.tank_yaw()), tank_turned_in_stroke_deg=np.degrees(self.tank_yaw() - yaw0))
            self.phase(f"let the nozzle go ({arm}) {attempt + 1}")
            yield from self.set_gripper(arm, .5, NOZZLE_RELEASE)
            yield from self.hold(.2)
            yield from self.move(arm, .9, hand.point - .060 * hand.rot[:, 2] + (0., 0., .012))
            if held:
                break
        return float(self.state["head"]) > start_head + np.radians(5.)

    def unscrew(self):
        arm, failures = "left", 0
        for _ in range(16):
            if float(self.state["head"]) >= L.HEAD_FREE - np.radians(2.):
                break
            plan = stroke_plan(arm, self.nozzle_azimuth())
            if plan is None:                           # past this arm's reach: the other arm takes over
                yield from self.home(arm)
                arm = ARMS[1 - ARMS.index(arm)]
                plan = stroke_plan(arm, self.nozzle_azimuth())
                if plan is None:
                    self.note("neither arm can reach the nozzle", nozzle_deg=np.degrees(self.nozzle_azimuth()))
                    break
            other = ARMS[1 - ARMS.index(arm)]
            if self.bracing != other and self.variant != "no_hold":
                yield from self.unbrace()
                yield from self.brace(other)
            moved = yield from self.stroke(arm, plan)
            failures = 0 if moved else failures + 1
            if not moved:
                self.note("the head did not turn", arm=arm, head_deg=np.degrees(self.state["head"]))
            if failures >= 2:
                break
        yield from self.home(arm)
        yield from self.set_gripper(arm, .2, OPEN)
        return float(self.state["head"]) >= L.HEAD_FREE - np.radians(8.)

    def sequence(self):
        while self.state is None:
            yield self.emit()
        self.note("start", bar=self.state["bar"], head_deg=np.degrees(self.state["head"]))
        yield from self.brace("right")
        vented = True
        if self.variant != "no_vent" and self.state["bar"] > L.SAFE_BAR:
            vented = yield from self.vent("left")
            self.log["vented"] = bool(vented)
        if self.variant == "vent_only":
            yield from self.unbrace()
        elif vented:
            self.log["unscrewed"] = bool((yield from self.unscrew()))
            yield from self.unbrace()
        else:
            self.note("gave up: the pressure could not be let out")
            yield from self.unbrace()
        self.done = True
        self.phase("rest")
        while True:
            yield self.emit()
