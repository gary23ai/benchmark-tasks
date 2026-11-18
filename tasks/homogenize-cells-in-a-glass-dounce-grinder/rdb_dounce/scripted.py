"""Full-score demonstration: the left hand holds the tube down in its well, the right hand strokes the pestle.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a scripted
demonstration, simulator state: the tube's and the pestle's poses, and the scorer's running stroke count. It
changes nothing in the simulation; the glass moves only because the fingers hold it and the liquid drags it.

Grips (both level, which the reach study found reachable with joint margins of 13 degrees or more):
- left: fingers pointing forward and to the right (60 degrees right of straight ahead), pads closing on two flats
  of the 13 mm tube 84 mm above the jack's platform (159 mm above the bench), about 12 N a finger;
- right: fingers pointing forward and to the left (45 degrees), pads closing on the 14 mm glass handle at its
  middle, about 20 N a finger.
Both arms move at once: the reaches and grips together, then for every stroke the right hand draws the pestle up
and pushes it down while the left hand presses the tube onto the well's floor and, by feedback on the tube's
measured height, pushes harder as soon as it rises. Strokes are closed on the pestle's measured depth in the tube,
because the arm sags and lags under the liquid's drag.
"""

import os

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

from . import fixed_scene as F
from . import layout as L

OPEN = float(FINGER_OPEN)
ARM_INDEX = {"left": 0, "right": 1}
YAW = {"left": np.radians(-60.), "right": np.radians(45.)}
SEED = {"left": np.radians((-30., 109., 38., 70., 30., 0.)), "right": np.radians((38., 85., 48., 37., -7., 0.))}
GRIP_Z = L.GRIP_Z                              # left fingertip centre height on the tube (159 mm)
STANDOFF = .060                                # pre-grasp: this far back along the fingers
TUBE_FINGER = -(L.TUBE_OD / 2 - 12. / 5000.)   # about 12 N a finger on the 13 mm tube
KNOB_FINGER = -(L.KNOB_D / 2 - 20. / 5000.)    # about 20 N a finger on the 14 mm handle
AROUND = -.020                                 # each pad 20 mm from the axis on the way in
HOLDING = {"left": (L.TUBE_OD / 2 - .0012) / abs(OPEN), "right": (L.KNOB_D / 2 - .0012) / abs(OPEN)}
PRESS = .008                                   # the left hand is commanded this far below where it gripped
PRESS_MAX = .030
LOW, HIGH = .002, .031                         # stroke between these depths of the head (bands: <= 5, >= 27 mm)
SPEED = .018                                   # m/s along the tube; the drag is then 4.5 N
DWELL = .15
ACCEL = .15                                    # m/s2: the wanted depth's starts and stops
LEAD0, LEAD_GAIN, LEAD_MAX, LEAD_STEP = .30, .6, .8, .08   # s: how far the height command leads the wanted height
LIFT_SLOW = .0015                              # tube rose this far on an up-stroke: 15 % slower from then on
SPEED_MIN = .009
BIAS_GAIN, BIAS_MAX = .5, .010
SETTLED, SETTLE_STEPS = .0015, 16              # a pass ends when the head is within 1.5 mm, or after 0.8 s more
XY_GAIN = .3
XY_MAX = .004                                  # sideways correction of the right hand at most 4 mm from its grip
SLIP_STOP = .005
VARIANT = os.environ.get("DEMO_VARIANT", "") or None      # None, "no_hold" (control) or "fast" (control)


def level(yaw):
    """Rotation of a level grip: fingers along ``yaw`` in the horizontal plane, pads closing horizontally."""
    a = np.array((np.cos(yaw), np.sin(yaw), 0.))
    c = np.array((-np.sin(yaw), np.cos(yaw), 0.))
    x = -c                                                     # the reachable roll (the reach study's 180 degrees)
    return np.column_stack((x, np.cross(a, x), a))


class DounceReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant=VARIANT):
        self.variant = variant
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in F.ARMS}
        self.aperture = dict.fromkeys(F.ARMS, OPEN)
        self.target = dict.fromkeys(F.ARMS)
        self.rot = {arm: level(YAW[arm]) for arm in F.ARMS}
        self.worst_ik = 0.
        self.phases = []
        self.tick = 0
        self.press = 0.
        # trace rows (every 2nd step of a stroke): tick, wanted depth, measured depth (mm), tube lift (mm), left
        # press (mm), lead up, lead down (s), height bias (mm), right fingertip centre to handle sideways (mm),
        # both openings
        self.log = {"events": [], "grasps": [], "strokes": [], "trace": []}

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        if state is not None:
            self.state = state

    def hand(self, arm):
        i = 7 * ARM_INDEX[arm]
        return ik.tcp_pose(self.measured[i:i + 6], F.BASES[arm])

    def opening(self, arm):
        return float(self.measured[7 * ARM_INDEX[arm] + 6])

    def tube(self):
        return np.asarray(self.state["tube_pos"], float), np.asarray(self.state["tube_rot"], float)

    def knob(self):
        p, r = np.asarray(self.state["pestle_pos"], float), np.asarray(self.state["pestle_rot"], float)
        return p + r[:, 2] * L.KNOB_Z

    def depth(self):
        return float(self.state["depth"])

    def lift(self):
        return float(self.state["tube_lift"])

    def grip_point(self, arm):
        if arm == "left":
            p, r = self.tube()
            return p + r[:, 2] * (GRIP_Z - L.SEAT_Z)
        return self.knob()

    def event(self, text, **fields):
        self.log["events"].append({"tick": self.tick, "event": text, **fields})

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- motion ---------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in F.ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=F.CONTROL_HZ)

    def command(self, arm, point, seed=None, iters=40):
        self.target[arm] = np.asarray(point, dtype=float).copy()
        q, err, _ = ik.solve(point, ik.matrix_to_quat_xyzw(self.rot[arm]),
                             self.joints[arm] if seed is None else seed, F.BASES[arm], iters=iters)
        self.worst_ik = max(self.worst_ik, err)
        self.joints[arm] = q

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * F.CONTROL_HZ))):
            yield self.emit()

    def glide(self, seconds, goals):
        """Joint-space moves of the arms named in ``goals`` (smooth start and stop), at the same time."""
        start = {arm: self.joints[arm].copy() for arm in goals}
        steps = max(1, round(seconds * F.CONTROL_HZ))
        for step in range(1, steps + 1):
            a = step / steps
            a *= a * (3. - 2. * a)
            for arm, goal in goals.items():
                self.joints[arm] = start[arm] + a * (np.asarray(goal) - start[arm])
            yield self.emit()
        for arm in goals:
            self.target[arm] = None

    def lines(self, seconds, goals):
        """Straight-line moves of the commanded fingertip centres of the arms named in ``goals``, at the same time."""
        start = {arm: (self.target[arm] if self.target[arm] is not None else self.hand(arm)[0]).copy() for arm in goals}
        steps = max(1, round(seconds * F.CONTROL_HZ))
        for step in range(1, steps + 1):
            a = step / steps
            a *= a * (3. - 2. * a)
            for arm, goal in goals.items():
                self.command(arm, start[arm] + a * (np.asarray(goal) - start[arm]))
            yield self.emit()

    def trim(self, goals, repeats=3):
        """Shift the commands until each hand, by its joint readings, is at its goal (the arms sag a little)."""
        for _ in range(repeats):
            moved = {}
            for arm, goal in goals.items():
                shift = np.clip(np.asarray(goal) - self.hand(arm)[0], -.008, .008)
                if np.linalg.norm(shift) > .0008:
                    moved[arm] = self.target[arm] + shift
            if not moved:
                return
            yield from self.lines(.35, moved)
            yield from self.hold(.1)

    def grip(self, fingers, seconds):
        start = dict(self.aperture)
        steps = max(1, round(seconds * F.CONTROL_HZ))
        for step in range(1, steps + 1):
            for arm, finger in fingers.items():
                self.aperture[arm] = start[arm] + step / steps * (finger - start[arm])
            yield self.emit()

    # -- the demonstration ------------------------------------------------------------------
    def take_hold(self, arms):
        """Reach both hands round their parts and close. Returns the arms whose pads stopped on their part."""
        fingers = {"left": TUBE_FINGER, "right": KNOB_FINGER}
        holding = set()
        for attempt in (1, 2):
            todo = [arm for arm in arms if arm not in holding]
            if not todo:
                break
            self.phase(f"reach {attempt}")
            yield from self.grip({arm: AROUND for arm in todo}, .4)
            backs = {arm: self.grip_point(arm) - self.rot[arm][:, 2] * STANDOFF for arm in todo}
            goals = {}
            for arm in todo:
                q, err, _ = ik.solve(backs[arm], ik.matrix_to_quat_xyzw(self.rot[arm]), SEED[arm], F.BASES[arm], iters=300)
                self.worst_ik = max(self.worst_ik, err)
                goals[arm] = q
            yield from self.glide(2.5 if attempt == 1 else 1.5, goals)
            for arm in todo:
                self.target[arm] = backs[arm].copy()
            yield from self.hold(.3)
            yield from self.trim(backs)
            self.phase(f"approach {attempt}")
            yield from self.lines(1.4, {arm: self.grip_point(arm) + (self.target[arm] - self.hand(arm)[0])
                                        for arm in todo})
            yield from self.trim({arm: self.grip_point(arm) for arm in todo}, repeats=2)
            self.phase(f"close {attempt}")
            before = {arm: (self.hand(arm)[1].T @ (self.grip_point(arm) - self.hand(arm)[0])) for arm in todo}
            yield from self.grip({arm: fingers[arm] for arm in todo}, .9)
            yield from self.hold(.4)
            for arm in todo:
                ok = self.opening(arm) > HOLDING[arm]
                self.log["grasps"].append({"arm": arm, "attempt": attempt, "tick": self.tick, "holding": bool(ok),
                                           "opening": round(self.opening(arm), 4),
                                           "part_in_hand_before_mm": [round(1000 * float(v), 2) for v in before[arm]]})
                if ok:
                    holding.add(arm)
                else:
                    self.event("grip closed on nothing; trying again", arm=arm)
                    self.aperture[arm] = AROUND
        return holding

    def pressed_point(self):
        p = self.grip_point("left")
        return p - np.array((0., 0., self.press))

    def stroke_to(self, depth_goal, speed):
        """Move the pestle's head to ``depth_goal`` by feedback on its measured depth, while the left hand keeps
        the tube pressed down (harder as soon as it rises).

        The wanted depth moves at ``speed`` with gentle starts and stops. The soft arm trails a moving pestle by
        (drag / arm stiffness), so the right hand's height command leads the wanted height by ``lead`` seconds of
        the wanted speed; the lead for each direction is learned from how far the pestle trailed on the last pass
        in that direction. A slow offset soaks up what is left at the ends. Nothing integrates fast, so the loop
        cannot ring or wind up against the bore's bottom."""
        dt = 1. / F.CONTROL_HZ
        way = "up" if depth_goal > self.depth() else "down"
        wanted, rate = self.depth(), 0.
        trails = []
        settle = cruise = 0
        self.peak_lift = 0.
        for count in range(int(6. / dt)):
            gap = depth_goal - wanted
            brake = np.sqrt(2 * ACCEL * abs(gap))
            rate = float(np.clip(np.sign(gap) * min(speed, brake), rate - ACCEL * dt, rate + ACCEL * dt))
            wanted = depth_goal if abs(gap) <= abs(rate) * dt else wanted + rate * dt
            if abs(rate) >= .98 * speed:           # cruising: how many seconds the pestle trails the wanted depth
                cruise += 1
                if cruise > 6:
                    trails.append((wanted - self.depth()) / rate)
            tube_p, tube_r = self.tube()
            knob = self.knob()
            want = tube_p + tube_r[:, 2] * (L.BOTTOM_T + wanted + L.KNOB_Z)     # handle centre on the tube's axis
            point = self.target["right"].copy()
            point[:2] += XY_GAIN * (want[:2] - knob[:2])
            point[:2] = self._xy0 + np.clip(point[:2] - self._xy0, -XY_MAX, XY_MAX)
            point[2] = want[2] + self._grip_dz + self._bias + self._lead[way] * rate
            self.command("right", point)
            self._press_tube()
            self.peak_lift = max(self.peak_lift, self.lift())
            if self.tick % 2 == 0:
                self._trace(wanted, knob)
            yield self.emit()
            if (self.state.get("grade") or {}).get("failed"):
                return
            if wanted == depth_goal:
                settle += 1
                if abs(self.depth() - depth_goal) < SETTLED or settle > SETTLE_STEPS:
                    break
        if len(trails) >= 3:
            step = float(np.clip(LEAD_GAIN * np.median(trails), -LEAD_STEP, LEAD_STEP))
            self._lead[way] = float(np.clip(self._lead[way] + step, 0., LEAD_MAX))
        if way == "up" and self.peak_lift > LIFT_SLOW and self.variant != "fast":      # the tube started to rise: stroke more gently
            self.speed = max(SPEED_MIN, self.speed * .85)
            self.event("tube rose on the up-stroke; slowing down", lift_mm=round(1000 * self.peak_lift, 2),
                       speed_mm_s=round(1000 * self.speed, 1))
        self._bias = float(np.clip(self._bias + BIAS_GAIN * (depth_goal - self.depth()), -BIAS_MAX, BIAS_MAX))

    def _press_tube(self):
        if "left" not in self.held:
            return
        lift = self.lift()
        if lift > .0003:
            self.press = min(PRESS_MAX, self.press + .8 * lift)
        slipped = self.grip_point("left")[2] - self.hand("left")[0][2]
        if slipped > SLIP_STOP:          # the fingers are sliding down the tube: press no harder
            self.press = max(PRESS, self.press - .0005)
        self.command("left", self.pressed_point() + self._left_trim)

    def _trace(self, wanted, knob):
        hp = self.hand("right")[0]
        self.log["trace"].append([self.tick, round(1000 * wanted, 2), round(1000 * self.depth(), 2),
                                  round(1000 * self.lift(), 2), round(1000 * self.press, 1),
                                  round(self._lead["up"], 3), round(self._lead["down"], 3), round(1000 * self._bias, 2),
                                  round(1000 * float(np.linalg.norm(hp[:2] - knob[:2])), 1),
                                  round(self.opening("left"), 4), round(self.opening("right"), 4)])

    def dwell(self, seconds):
        for _ in range(max(1, round(seconds * F.CONTROL_HZ))):
            self._press_tube()
            yield self.emit()

    def sequence(self):
        yield from self.hold(.5)
        arms = ("right",) if self.variant == "no_hold" else ("left", "right")
        self.held = yield from self.take_hold(arms)
        if "right" not in self.held or (self.variant != "no_hold" and "left" not in self.held):
            self.event("no grip; giving up", held=sorted(self.held))
            yield from self.glide(2.5, {arm: np.asarray(READY, float) for arm in F.ARMS})
            while True:
                yield self.emit()
        # what each grip settled to, kept as offsets so the commands start where the hands are
        knob = self.knob()
        self._grip_dz = float(self.target["right"][2] - knob[2])
        self._left_trim = self.target["left"] - self.grip_point("left") if "left" in self.held else 0.
        self._xy0 = self.target["right"][:2].copy()
        self._lead = {"up": LEAD0, "down": LEAD0}
        self._bias = 0.
        if "left" in self.held:
            self.phase("press")
            self.press = PRESS
            yield from self.lines(.6, {"left": self.pressed_point() + self._left_trim})
        self.speed = .045 if self.variant == "fast" else SPEED
        self.phase("first descent")
        yield from self.stroke_to(LOW, self.speed)
        n = 0
        while True:
            grade = self.state.get("grade") or {}
            good, total = grade.get("good_strokes", n), grade.get("strokes", n)
            if good >= 15 or total >= 16 or grade.get("failed"):
                break
            n += 1
            self.phase(f"stroke {n}")
            yield from self.stroke_to(HIGH, self.speed)
            yield from self.dwell(DWELL)
            yield from self.stroke_to(LOW, self.speed)
            yield from self.dwell(DWELL)
            grade = self.state.get("grade") or {}
            self.log["strokes"].append({"n": n, "tick": self.tick, "good": grade.get("good_strokes"),
                                        "total": grade.get("strokes"), "peak_lift_mm": grade.get("peak_lift_mm")})
            if n > 20:
                break
        self.phase("release")
        # the right hand lets go of the pestle, now on the bottom, and backs out; then the left hand
        yield from self.grip({"right": AROUND}, .5)
        yield from self.hold(.3)
        yield from self.lines(1.0, {"right": self.target["right"] - self.rot["right"][:, 2] * .07})
        if "left" in self.held:
            # ease the press off by commanding the hand where its joints say it is (no force, no motion), then open
            yield from self.lines(.5, {"left": self.hand("left")[0]})
            yield from self.grip({"left": AROUND}, .5)
            yield from self.hold(.3)
            yield from self.lines(1.0, {"left": self.target["left"] - self.rot["left"][:, 2] * .07})
        self.phase("home")
        yield from self.glide(2.5, {arm: np.asarray(READY, float) for arm in F.ARMS})
        yield from self.grip({arm: OPEN for arm in F.ARMS}, .3)
        while True:
            yield self.emit()
