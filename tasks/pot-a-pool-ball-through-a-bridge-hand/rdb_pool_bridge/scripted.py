"""Full-score demonstration: both arms take the cue off the rail, the left hand closes a loop round the shaft on
the cloth (the bridge), the right hand strokes the cue through it into the white ball.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a
scripted demonstration, simulator state: the cue's pose (its tip and axis) and the balls' positions. It changes
nothing in the simulation; the cue moves only because the fingers hold it or the loop guides it.

How the bridge works. The left gripper points straight down with its fingertips on the cloth and its pads
LOOP_GAP (8 mm) apart, closer than the 13 mm shaft is thick. Between the fingers, 74 mm above the fingertips,
the undersides of the two finger carriages form a flat ceiling. The right hand holds the cue 0.80 m from the
tip, about 0.2 m in front of its balance point, so the cue's weight lifts its front end (about 1 N m about the
grip; a pinch resists only about 0.1 to 0.3 N m about its pad line) and presses the shaft up against that
ceiling with about 2 N. The fingers' sides keep it in line (1.75 mm of play each side); the pads below keep it
from dropping out. So the left hand sets the height and line of the shaft and the right hand sets its speed.

Each stage is closed on what the simulator reports: the hands are trimmed until their joint readings are on
target, the cue's measured line is servoed onto the line through the white ball, and a grip is checked by the
measured opening before the cue is moved.
"""

import math

from inspect_robots import Action, ActionChunk
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

from . import layout as L
from .fixed_scene import ARMS, BASES, CONTROL_HZ

OPEN = abs(float(FINGER_OPEN))
DT = 1. / CONTROL_HZ
TILT_R = math.radians(40.)                       # the right hand's fingers lean 40 degrees toward -y
DOWN = np.array((0., 0., -1.))
APPROACH = {"left": DOWN, "right": np.array((0., -math.sin(TILT_R), -math.cos(TILT_R)))}
CLOSING = {"left": np.array((1., 0., 0.)), "right": np.array((1., 0., 0.))}
# the cue's axis this far past the TCP along the fingers in a pinch: the fingertips then stay 1.5 mm or more above
# the rail's cap when the cue lies on it (the right hand's leaning pads reach lower at their corners)
PINCH_DEPTH = {"left": .007, "right": .009}
SQUEEZE_N = 25.                                  # force-sized close: each finger 5 mm inside the surface
LIFT = .045
HOVER = .003                                     # fingertips above the cloth while the loop is aligned
PRESS = .022                                     # then the command goes this far below the cloth: about 10 N
LOOP_OPEN = .025                                 # pads apart while the shaft rises to the ceiling
BACKSWING = .130
STRIKE_TARGET = .60                              # cue speed wanted at the white: it leaves at about 0.8 m/s
BACK_SPEED = .25                                 # peak speed of the backswing
PRACTICE_STROKES = 1                             # one practice stroke, made without any correction (run 3: a
                                                 # corrected second practice jerked the cue out of the grip)
PRACTICE_SHIFT = .130                            # started this much further back, a practice stroke ends 50 mm
                                                 # short of the white (the stroke runs on 80 mm past contact;
                                                 # run 3's practice, 100 mm back, ended only 3 mm short)
OVERRUN = .080                                   # the stroke's command runs on this far past the contact point
CRUISE = .080                                    # at a steady speed from 40 mm before to 40 mm after it: the arm
                                                 # lags its command, so the cue is still at speed at the white
SERVO_GAIN = .35
SEEDS = {"left": [np.array((-.6, 1.4, 1.2, -.9, 0., 0.)), np.array((-.3, 1.9, 1.6, -.9, 0., -1.5)),
                  np.array((0., 1., 1., -.6, 0., 0.)), np.array((-.5, 2.2, 1.2, -1.2, 0., 1.5))],
         "right": [np.array((.4, 1.6, 1.3, -.6, 0., 0.)), np.array((.3, 1.9, 1.6, -.9, 0., 1.5)),
                   np.array((0., 1., 1., -.6, 0., 0.)), np.array((.5, 2.2, 1.2, -1.2, 0., -1.5))]}


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def hand_rotation(arm, sign=1.):
    """Gripper rotation: columns (closing axis, pad-width axis, finger direction)."""
    a = APPROACH[arm]
    c = sign * CLOSING[arm]
    c = unit(c - (c @ a) * a)
    return np.column_stack((c, np.cross(a, c), a))


def finger_for(width, squeeze_n=0.):
    """Finger joint command (0 closed .. OPEN) for pads ``width`` apart, minus a force-sized squeeze."""
    return float(np.clip(width / 2 - squeeze_n / 5000., 0., OPEN))


class PoolReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant=""):
        self.variant = variant
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.finger = dict.fromkeys(ARMS, OPEN)
        self.target = dict.fromkeys(ARMS)
        self.sign = {"left": 1., "right": 1.}
        self.worst_ik = 0.
        self.phases = []
        self.tick = 0
        self.log = {"events": [], "grips": [], "servo": [], "stroke": [], "trims": []}

    # -- readings -------------------------------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        if state is not None:
            self.state = state

    def hand(self, arm):
        i = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.measured[i:i + 6], BASES[arm])

    def opening(self, arm):
        return float(self.measured[7 * ARMS.index(arm) + 6]) * 2 * OPEN

    def cue(self):
        return np.asarray(self.state["cue_tip"], dtype=float), unit(self.state["cue_axis"])

    def ball(self, name):
        return np.asarray(self.state["balls"][name], dtype=float)

    def event(self, text, **fields):
        self.log["events"].append({"tick": self.tick, "event": text, **fields})

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- commands -------------------------------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.finger[a] / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def solve(self, arm, point, rotation, seed=None, iters=40):
        joints, error, _ = ik.solve(point, ik.matrix_to_quat_xyzw(rotation),
                                    self.joints[arm] if seed is None else seed, BASES[arm], iters=iters)
        self.worst_ik = max(self.worst_ik, error)
        return joints

    def command(self, arm, point, rotation=None):
        rotation = hand_rotation(arm, self.sign[arm]) if rotation is None else rotation
        self.target[arm] = (np.asarray(point, dtype=float).copy(), rotation)
        self.joints[arm] = self.solve(arm, point, rotation)

    def best_start(self, arm, points):
        """Joint angles for the first of ``points``: the seed and closing sign whose solutions, followed from one
        point to the next (as the demonstration will move), keep the most joint margin."""
        best = None
        dense = [points[0]] + [a + (b - a) * k / 6 for a, b in zip(points, points[1:]) for k in range(1, 7)]
        for sign in (1., -1.):
            rotation = hand_rotation(arm, sign)
            for seed in SEEDS[arm]:
                q, first, margin = seed, None, np.inf
                for point in dense:
                    q, err, rerr = ik.solve(point, ik.matrix_to_quat_xyzw(rotation), q, BASES[arm],
                                            iters=300 if first is None else 80)
                    if err > 5e-4 or rerr > 5e-3:
                        margin = -np.inf
                        break
                    first = q if first is None else first
                    margin = min(margin, float(np.minimum(q - ik.LOWER, ik.UPPER - q).min()))
                if margin > -np.inf and (best is None or margin > best[0]):
                    best = (margin, sign, first)
        if best is None:
            raise RuntimeError(f"No IK for the {arm} hand along {points}")
        self.sign[arm] = best[1]
        self.log["events"].append({"tick": self.tick, "event": f"{arm} hand plan", "closing_sign": best[1],
                                   "path_margin_deg": round(float(np.degrees(best[0])), 1)})
        return best[2]

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def glide(self, seconds, goals):
        """Joint-space moves to ``goals`` ({arm: joints}), smooth start and stop."""
        start = {arm: self.joints[arm].copy() for arm in goals}
        steps = max(1, round(seconds * CONTROL_HZ))
        for k in range(1, steps + 1):
            a = k / steps
            a *= a * (3. - 2. * a)
            for arm, goal in goals.items():
                self.joints[arm] = start[arm] + a * (np.asarray(goal) - start[arm])
            yield self.emit()
        for arm in goals:
            self.target[arm] = None

    def lines(self, seconds, goals, fingers=None):
        """Straight-line moves of the commanded TCPs to ``goals`` ({arm: point}), fingers ramped meanwhile."""
        start = {}
        for arm in goals:
            if self.target[arm] is None:
                self.target[arm] = (self.hand(arm)[0], hand_rotation(arm, self.sign[arm]))
            start[arm] = self.target[arm]
        f0 = {arm: self.finger[arm] for arm in (fingers or {})}
        steps = max(1, round(seconds * CONTROL_HZ))
        for k in range(1, steps + 1):
            a = k / steps
            a *= a * (3. - 2. * a)
            for arm, goal in goals.items():
                p0, r0 = start[arm]
                self.command(arm, p0 + a * (np.asarray(goal) - p0), r0)
            for arm, f in (fingers or {}).items():
                self.finger[arm] = f0[arm] + min(1., 2. * k / steps) * (f - f0[arm])
            yield self.emit()

    def trim(self, goals, repeats=3, tolerance=.002):
        """Shift each command until the TCP, by its joint readings, is on its goal (the arms sag)."""
        for _ in range(repeats):
            shifted = {}
            for arm, goal in goals.items():
                error = np.asarray(goal) - self.hand(arm)[0]
                if np.linalg.norm(error) > tolerance:
                    shifted[arm] = self.target[arm][0] + np.clip(error, -.012, .012)
            if not shifted:
                return
            self.log["trims"].append([self.tick, {a: [round(1000 * float(v), 1) for v in shifted[a] - goals[a]] for a in shifted}])
            yield from self.lines(.35, shifted)
            yield from self.hold(.1)

    # -- the cue's geometry -----------------------------------------------------------------------------------------
    @staticmethod
    def pinch_tcp(arm, point):
        """TCP that puts a cue point between the pads, PINCH_DEPTH past the TCP along the fingers."""
        return np.asarray(point, dtype=float) - PINCH_DEPTH[arm] * APPROACH[arm]

    def carry_targets(self, tip, axis):
        return {"left": self.pinch_tcp("left", L.cue_point(tip, axis, L.BRIDGE_S)),
                "right": self.pinch_tcp("right", L.cue_point(tip, axis, L.GRIP_S))}

    # -- the demonstration ------------------------------------------------------------------------------------------
    def take_cue(self):
        """Both hands pinch the cue where it lies on the rail. Returns True when both have closed on it."""
        tip, axis = self.cue()
        grasp = self.carry_targets(tip, axis)
        width = {"left": L.cue_diameter(L.BRIDGE_S), "right": L.cue_diameter(L.GRIP_S)}
        above = {arm: grasp[arm] + np.array((0., 0., .05)) for arm in ARMS}      # come straight down onto the rail
        self.phase("reach")
        # the poses this demonstration will take, to choose each arm's configuration once for all of them
        slid = (np.array((tip[0], L.TIP_ADDRESS[1], tip[2] + LIFT)), axis)
        plan = [self.carry_targets(t, a) for t, a in ((tip, axis), slid, (L.TIP_ADDRESS, L.CUE_AXIS),
                                                      (L.TIP_ADDRESS - BACKSWING * L.CUE_AXIS, L.CUE_AXIS))]
        loop = plan[2]["left"].copy()
        loop[2] = L.BED_Z + L.PAD_TIP_PAST_TCP - PRESS
        paths = {"left": [above["left"]] + [p["left"] for p in plan[:3]] + [loop],
                 "right": [above["right"]] + [p["right"] for p in plan]}
        yield from self.glide(2.4, {arm: self.best_start(arm, paths[arm]) for arm in ARMS})
        for arm in ARMS:
            self.finger[arm] = finger_for(width[arm] + .020)
            self.target[arm] = (above[arm], hand_rotation(arm, self.sign[arm]))
        yield from self.hold(.3)
        yield from self.trim(above)
        self.phase("descend")
        tip, axis = self.cue()
        grasp = self.carry_targets(tip, axis)
        yield from self.lines(1.0, grasp)
        yield from self.trim(grasp, repeats=2, tolerance=.0015)
        self.phase("close")
        steps = round(.8 * CONTROL_HZ)
        f0 = dict(self.finger)
        for k in range(1, steps + 1):
            for arm in ARMS:
                self.finger[arm] = f0[arm] + k / steps * (finger_for(width[arm], SQUEEZE_N) - f0[arm])
            yield self.emit()
        yield from self.hold(.4)
        ok = True
        for arm in ARMS:
            held = self.opening(arm) > width[arm] - .003
            self.log["grips"].append({"arm": arm, "tick": self.tick, "opening_mm": round(1000 * self.opening(arm), 2),
                                      "width_mm": round(1000 * width[arm], 1), "held": bool(held)})
            ok &= held
        return ok

    def carry(self, path, seconds):
        """Move the cue through a list of (tip, axis) poses, both hands keeping their pinch points, then trim
        both hands onto the last pose using the cue's measured position."""
        tip0, axis0 = self.cue()
        held = {arm: self.target[arm][0] - self.carry_targets(tip0, axis0)[arm] for arm in ARMS}   # sag offsets
        poses = [(tip0, axis0)] + path
        for (ta, aa), (tb, ab) in zip(poses, poses[1:]):
            rot = Slerp([0., 1.], Rotation.from_matrix(np.stack((L.cue_rotation(aa), L.cue_rotation(ab)))))
            steps = max(1, round(seconds / len(path) * CONTROL_HZ))
            for k in range(1, steps + 1):
                a = k / steps
                a *= a * (3. - 2. * a)
                tip = ta + a * (tb - ta)
                axis = rot(a).as_matrix()[:, 0]
                for arm, point in self.carry_targets(tip, axis).items():
                    self.command(arm, point + held[arm], self.target[arm][1])
                yield self.emit()
        # trim: the cue's measured pinch points onto the wanted ones
        for _ in range(3):
            tip, axis = self.cue()
            now = {"left": L.cue_point(tip, axis, L.BRIDGE_S), "right": L.cue_point(tip, axis, L.GRIP_S)}
            want = {"left": L.cue_point(*path[-1], L.BRIDGE_S), "right": L.cue_point(*path[-1], L.GRIP_S)}
            error = {arm: np.clip(want[arm] - now[arm], -.01, .01) for arm in ARMS}
            if max(np.linalg.norm(e) for e in error.values()) < .002:
                break
            yield from self.lines(.4, {arm: self.target[arm][0] + error[arm] for arm in ARMS})
            yield from self.hold(.15)

    def make_bridge(self):
        """Open the left hand a little and lower it so the shaft rises to the loop's ceiling, then close the
        pads under the shaft and put the fingertips down by the cloth."""
        self.phase("open loop")
        tcp_loop = self.target["left"][0].copy()
        tcp_loop[2] = L.BED_Z + HOVER + L.PAD_TIP_PAST_TCP
        yield from self.lines(.9, {"left": tcp_loop}, fingers={"left": finger_for(LOOP_OPEN)})
        yield from self.hold(.5)
        self.phase("close loop")
        yield from self.lines(.6, {"left": tcp_loop}, fingers={"left": finger_for(L.LOOP_GAP)})
        yield from self.hold(.3)

    def wanted_line(self):
        """The line the cue must lie on: through the shaft's measured point in the loop and the white ball's
        centre (a centre-ball hit), its horizontal direction from the white toward the ghost-ball position."""
        tip, axis = self.cue()
        white = self.ball("cue")
        tcp, rot = self.hand("left")
        from .grading import axis_near
        loop = axis_near(tip, axis, tcp, rot)
        direction = unit(white - loop)
        return loop, white, direction

    def align(self, seconds, *, hover=True):
        """Servo the cue's measured line: the left hand's x so that the shaft in the loop is on the shot's line,
        the right hand so that the axis runs through the white's centre with the tip ADDRESS_GAP behind it."""
        good = 0
        for count in range(max(1, round(seconds * CONTROL_HZ))):
            tip, axis = self.cue()
            white = self.ball("cue")
            loop, _, _ = self.wanted_line()
            # the shot line, horizontally: from the white's measured centre to the ghost-ball position of the
            # 1-ball's measured centre for a cut toward the marked pocket's aim point
            one = self.ball("one")
            w = unit(L.AIM - one[:2])
            u = unit(one[:2] - 2 * L.BALL_R * w - white[:2])
            n = np.array((-u[1], u[0]))
            # left hand moved along x only, by what puts the loop point on that line
            ex_loop = -float((loop[:2] - white[:2]) @ n) / n[0] if abs(n[0]) > .3 else 0.
            corrected = loop + (np.array((ex_loop, 0., 0.)) if hover else 0.)
            desired_dir = unit(white - corrected)
            grip_now = L.cue_point(tip, axis, L.GRIP_S)
            # where the grip point belongs: on the line from the (corrected) loop point through the white,
            # GRIP_S behind the tip, with the tip ADDRESS_GAP behind the white's surface
            tip_want = white - (L.BALL_R + L.ADDRESS_GAP) * desired_dir
            grip_want = tip_want - L.GRIP_S * desired_dir
            e_grip = grip_want - grip_now
            e_left = np.array((ex_loop, 0., 0.))
            if hover is False:
                e_left[:] = 0.                    # pressed on the cloth: the left hand stays where it is
            err = max(abs(ex_loop) if hover else 0., float(np.linalg.norm(e_grip)))
            if count % 2 == 0:
                self.log["servo"].append([self.tick, round(1000 * float(ex_loop), 2),
                                          *(round(1000 * float(v), 2) for v in e_grip)])
            good = good + 1 if err < .0012 else 0
            if good >= 4:
                return True
            for arm, e in (("left", e_left), ("right", e_grip)):
                step = SERVO_GAIN * e
                size = float(np.linalg.norm(step))
                if size > .02 * DT:
                    step *= .02 * DT / size
                self.command(arm, self.target[arm][0] + step, self.target[arm][1])
            yield self.emit()
        return False

    def press(self):
        self.phase("press")
        p = self.target["left"][0].copy()
        p[2] = L.BED_Z + L.PAD_TIP_PAST_TCP - PRESS
        yield from self.lines(.6, {"left": p})
        yield from self.hold(.4)

    @staticmethod
    def stroke_profile(speed, rise, fall, cruise=0.):
        """Commanded distance along the cue at each control step: a raised-cosine rise to ``speed`` over
        ``rise``, ``cruise`` at that speed, then a raised-cosine fall to rest over ``fall``."""
        t_rise, t_cruise, t_fall = 2 * rise / speed, cruise / speed, 2 * fall / speed
        out = []
        for k in range(1, round((t_rise + t_cruise + t_fall) * CONTROL_HZ) + 1):
            t = k * DT
            if t <= t_rise:
                u = t / t_rise
                out.append(rise * (u - math.sin(math.pi * u) / math.pi))
            elif t <= t_rise + t_cruise:
                out.append(rise + speed * (t - t_rise))
            else:
                u = min(1., (t - t_rise - t_cruise) / t_fall)
                out.append(rise + cruise + fall * (u + math.sin(math.pi * u) / math.pi))
        return out

    @staticmethod
    def follow(profile, lag, extra=20):
        """Positions, at the end of each control step, of a hand that follows the stepped command ``profile``
        with a first-order lag (12 sub-steps per control step)."""
        x, sub, out = 0., 12, []
        h = DT / sub
        for target in list(profile) + [profile[-1]] * extra:
            for _ in range(sub):
                x += (target - x) / lag * h
            out.append(x)
        return out

    def contact_speed(self, profile, contact, lag):
        """Speed at ``contact`` (averaged over the control step in which it is reached) of that hand."""
        xs = [0.] + self.follow(profile, lag)
        for a, b in zip(xs, xs[1:]):
            if b >= contact:
                return (b - a) / DT
        return 0.

    def fit_lag(self, profile, behind):
        """The lag that best explains how far the hand trailed its command during the backswing."""
        best = (np.inf, .1)
        for lag in np.arange(.02, .40, .005):
            model = np.asarray(profile) - np.asarray(self.follow(profile, lag, extra=0))
            err = float(np.sum((model[:len(behind)] - np.asarray(behind)) ** 2))
            best = min(best, (err, float(lag)))
        return best[1]

    def plan_speed(self, rise, cruise, fall, contact, lag):
        """Commanded cruise speed that brings the cue to the white at STRIKE_TARGET (bisection on the lag model)."""
        lo, hi = .3, 1.6
        for _ in range(30):
            mid = (lo + hi) / 2
            if self.contact_speed(self.stroke_profile(mid, rise, fall, cruise), contact, lag) < STRIKE_TARGET:
                lo = mid
            else:
                hi = mid
        return hi

    def stroke(self):
        """Backswing at a steady speed (which measures how far the arm lags its command), pause, then a forward
        stroke along the cue's measured axis, planned on that lag so the cue meets the white at STRIKE_TARGET,
        running on OVERRUN past the contact point. The left hand does not move."""
        self.phase("backswing")
        tip, axis = self.cue()
        white = self.ball("cue")
        start = self.target["right"][0].copy()
        rot = self.target["right"][1]
        gap = float(np.linalg.norm(white - tip) - L.BALL_R)           # tip to the white's surface along the line
        rest = self.hand("right")[0]
        back = self.stroke_profile(BACK_SPEED, BACKSWING / 2, BACKSWING / 2)
        behind = []
        for s in back:
            self.command("right", start - s * axis, rot)
            yield self.emit()
            behind.append(s - float((rest - self.hand("right")[0]) @ axis))     # commanded minus measured travel
        yield from self.hold(.4)
        lag = self.fit_lag(back, behind)
        # the distance the cue must travel to the white, from where the hand really is after the backswing
        contact = gap + float((rest - self.hand("right")[0]) @ axis)
        rise = contact - CRUISE / 2
        speed = self.plan_speed(rise, CRUISE, OVERRUN - CRUISE / 2, contact, lag)
        self.log["stroke_plan"] = {"lag_s": round(lag, 4), "commanded_peak_m_s": round(speed, 3),
                                   "gap_mm": round(1000 * gap, 1), "contact_travel_mm": round(1000 * contact, 1),
                                   "target_m_s": STRIKE_TARGET, "model_contact_m_s": round(self.contact_speed(
                                       self.stroke_profile(speed, rise, OVERRUN - CRUISE / 2, CRUISE), contact, lag), 3)}
        profile = self.stroke_profile(speed, rise, OVERRUN - CRUISE / 2, CRUISE)
        # Practice strokes, as a player makes them: the same stroke started PRACTICE_SHIFT further back, so it ends
        # short of the white. At speed the arm's hand swings off the straight line (its joints lag by different
        # amounts; in our first run 16 mm sideways, turning the cue 1.7 degrees about the bridge). The sideways
        # and vertical swing measured from the joint readings is fed forward, negated, into the next stroke.
        correction = [np.zeros(3) for _ in profile]
        shift = max(0, round(lag * CONTROL_HZ))
        # the control step in which the cue should reach the white (lag model); the correction stops changing
        # there, so nothing jerks the hand after the strike
        reach = [0.] + self.follow(profile, lag, extra=0)
        contact_k = next((k for k, x in enumerate(reach[1:]) if x >= contact), len(profile) - 1)
        self.log["practice"] = []
        for attempt in range(PRACTICE_STROKES):
            self.phase(f"practice {attempt + 1}")
            base = start - (BACKSWING + PRACTICE_SHIFT) * axis
            yield from self.lines(.8, {"right": base})
            yield from self.hold(.4)
            offset = self.hand("right")[0] - self.target["right"][0]          # static sag, removed below
            swing = []
            for k, s in enumerate(profile):
                self.command("right", base + s * axis + correction[k], rot)
                yield self.emit()
                d = self.hand("right")[0] - offset - (base + s * axis)      # off the commanded straight line
                swing.append(d - (d @ axis) * axis)                          # its part across the cue
            swing += [swing[-1]] * shift
            worst = max(float(np.linalg.norm(v)) for v in swing)
            held = self.opening("right") > L.cue_diameter(L.GRIP_S) - .004
            for k in range(len(profile)):
                j = min(k, contact_k)
                correction[k] = np.clip(correction[j] - swing[j + shift], -.03, .03) if k <= contact_k else correction[contact_k]
            self.log["practice"].append({"attempt": attempt + 1, "worst_swing_mm": round(1000 * worst, 1),
                                         "contact_step": contact_k, "shift_steps": shift, "grip_held": bool(held),
                                         "swing_mm": [[round(1000 * float(x), 1) for x in v] for v in swing],
                                         "correction_mm": [[round(1000 * float(x), 1) for x in v] for v in correction]})
            yield from self.hold(.3)
            yield from self.lines(.8, {"right": start - BACKSWING * axis})
            yield from self.hold(.5)
        self.phase("stroke")
        base = start - BACKSWING * axis
        for k, s in enumerate(profile):
            self.command("right", base + s * axis + correction[k], rot)
            self.log["stroke"].append([self.tick, round(1000 * s, 1), round(float(np.linalg.norm(self.state["cue_velocity"])), 3)
                                       if self.state is not None else None])
            yield self.emit()
        self.phase("after the strike")
        yield from self.hold(.4)

    def one_hand_stroke(self):
        """Control: the left hand lets go and moves away; the right hand alone holds and strokes the cue."""
        self.phase("left lets go")
        yield from self.lines(.5, {"left": self.target["left"][0]}, fingers={"left": OPEN})
        up = self.target["left"][0] + (0., 0., .06)
        yield from self.lines(1.0, {"left": up})
        yield from self.glide(1.5, {"left": np.asarray(READY, dtype=float)})
        yield from self.stroke()

    def sequence(self):
        yield from self.hold(.5)
        self.log["stroke_plan"] = None
        if not (yield from self.take_cue()):
            self.event("a hand did not close on the cue")
        tip0, axis0 = self.cue()
        lifted = (tip0 + (0., 0., LIFT), axis0)
        slid = (np.array((tip0[0], L.TIP_ADDRESS[1], tip0[2] + LIFT)), axis0)
        address = (L.TIP_ADDRESS.copy(), L.CUE_AXIS.copy())
        self.phase("lift")
        yield from self.carry([lifted], 1.0)
        self.phase("slide along the rail")
        yield from self.carry([slid], 1.2)
        self.phase("to address")
        yield from self.carry([address], 2.6)
        if self.variant == "one_hand":
            yield from self.one_hand_stroke()
        elif self.variant == "no_slide":
            self.phase("both hands push")            # control: the left hand keeps its pinch and moves with the cue
            tip, axis = self.cue()
            pl, pr = self.target["left"], self.target["right"]
            yield from self.lines(1.1, {"left": pl[0] - BACKSWING * axis, "right": pr[0] - BACKSWING * axis})
            yield from self.lines(.5, {"left": pl[0] + .02 * axis, "right": pr[0] + .02 * axis})
        else:
            yield from self.make_bridge()
            self.phase("align")
            yield from self.align(3.0, hover=True)
            yield from self.press()
            self.phase("align pressed")
            yield from self.align(1.5, hover=False)
            yield from self.stroke()
        self.phase("wait for the balls")
        while True:
            yield self.emit()
