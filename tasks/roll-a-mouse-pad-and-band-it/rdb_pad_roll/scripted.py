"""Full-score demonstration: roll the pad up, keep the roll shut, slide the band over it, put the banded roll down.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a scripted
demonstration, simulator state: where the pad's strips and the band are. It changes nothing in the simulation:
the pad and the band move only because the grippers move them.

How the pad is rolled (the right gripper winds, the left gripper holds)
  The right gripper reaches in along the roll's axis from the pad's right-hand side, so that turning its wrist
  turns the roll. It pinches the first two strips of the near short edge between its fingertips and lays them
  over. From then on turning those two strips winds the pad up round them like a clock spring.

  Its wrist turns 240 degrees and no further, so twice it must let go and take hold of the edge again further
  back. The left gripper comes down on the curl from above, leaning forward, and stays with the roll: it holds
  the roll shut whenever the right gripper lets go and is a loose fence round it while the right gripper turns.
  The second time the edge is underneath; the left gripper lifts the roll 20 mm so that a fingertip can come
  under it. What a fingertip drags out of the roll as it is drawn out, the closed fingertips push back, end on.

How the band goes on
  After the third stroke the whole pad is on the roll and the right gripper leaves. From then on the left gripper
  neither eases its hold nor lifts the roll (either lets the roll come open): it slides the roll along its own
  length on the carton until part of it sticks out over the carton's right-hand edge. The right gripper takes the
  band from its cradle, slides it over that part as far as the carton's edge, and only then does the left gripper
  let go. The right gripper lays the banded roll in the cradle and both arms go back to ready.

Kept but not used by the recorded demonstration (TIGHTEN = False): tightening the first turn by winding it against
the pinned pad, and holding the roll by its wall; see TECHNICAL.md, "What was tried and dropped".
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY

from . import pad_model as m
from .fixed_scene import ARMS, BASES, CONTROL_HZ

OPEN = float(FINGER_OPEN)
DT = 1. / CONTROL_HZ

# --- the winding gripper (right) ----------------------------------------------------------------------
BITE = .010                               # the fingertips reach this far in from the roll's right-hand end
OUT = .060                                # drawn out along the roll this far, the fingers are clear of its end
EDGE_OPEN = .014                          # fingertips this far apart round the edge before they close on it
CLEAR = .005                              # an open fingertip stands this far off what it held
WALL_HALF = .012                          # the roll's wall is taken where its strips are within this of the roll's middle
TAKE_CLEAR = .0035                        # ... and this far off what it is about to take
WIND_RATE = np.radians(90.)               # rad/s
FIRST_TURN = np.radians(200.)
TIGHTEN = False                           # wind the first turn tight against the pinned pad before rolling on (see tighten)
PUSH_SPEED, PUSH_MOST = .020, .050        # the finished roll is rolled on by the holding gripper: how fast, how far at most
TIP_THICK, TOP_PRESS = .004, .005         # a fingertip's thickness; the fingertips bear this far down on the roll's top
PUSH_SLACK = .001                         # ... its fingers this far off the roll
SQUARE_PATS, SQUARE_OVER = 5, .003        # squaring the roll's end: at most so many strips, those standing this far out
WRIST = (np.radians(-30.), np.radians(210.))      # the closing axis' turn about the roll's axis that the wrist covers
SECOND_TO = WRIST[1]                      # the second stroke ends with the wrist here: at the end of its travel
HOLD_GAIN = .15                           # share of the roll's creep along the pad that is added, each step, to the pull against it
DIE_SLACK = .003                          # the fence stands this far outside the tight roll's size
LIFT = .020                               # the holding gripper lifts the roll this far for a fingertip to come under it
PIN_STRIP = 22                            # the holding gripper pins the pad by this strip
PIN_Y = .040                              # ... this far in from the pad's right-hand edge: near the line the winding gripper pulls on
PIN_PRESS = .020                          # ... sent this far below the pad's top (about 10 N)
PIN_SLIP = .012                           # the pinned pad has slipped if its far edge has come this far
SKEW = np.radians(5.)                     # control "skewed_start": the winding gripper's drift off square
SLEEVE_STALL = .025                       # the band will not go on if it falls this far behind the gripper's command
TAIL_PAST = np.radians(15.)               # the far edge may be this far past a finger and still be under it
PRESS = .003                              # while it turns the roll its centre point aims this far below where it is
CINCH_TO, CINCH_RATE = .001, np.radians(60.)          # tighten the first turn to within this of tight; how fast
PAT_OVER, PAT_LEFT, PAT_SPEED = .004, .001, .020      # pat a strip back when it is this far out; until; how fast
# --- the holding gripper (left) -------------------------------------------------------------------------
JAW_TILT = np.radians(20.)                # it leans this far forward of straight down
JAW_Y = .105                              # along the roll from its right-hand end: the middle
JAW_TIP = .006                            # the roll sits this far up the fingers from their tips
JAW_SQUEEZE = .003                        # each finger is sent this far inside the tight roll's size to hold it firm
GUIDE_SLACK = .002                        # as a fence round the turning roll the fingers stand this far off it
SNUG = .001                               # just touching the roll: this far inside its size
CARRY_SQUEEZE = .0045                     # ... and this far to carry it
# --- the band ---------------------------------------------------------------------------------------------
JAW_ALONG = .045                          # the holding gripper holds the roll this far left of its middle
BAND_OVERHANG = .115                      # the roll is slid along the carton until this much of it sticks out over the
                                          # carton's right-hand edge: the band goes on that part
SLIDE_SPEED, SLIDE_PRESS = .030, .0       # how fast it is slid; the holding gripper bears this far down meanwhile
EDGE_GAP = .003                           # the band's rim stops this far short of the carton's edge
BAND_LEAN = np.radians(55.)               # the band gripper leans this far back along the band: it reaches further so, and the
                                          # two wrists have room
BAND_HOLD = .020                          # ... and holds the band this far from its trailing rim
BAND_GRIP_UP = .004                       # its centre point above the band's axis
BAND_LEAD = .015                          # the band's mouth starts this far from the roll's end
BAND_SHORT = .004                         # ... and stops this far short of the holding gripper's fingers
FINGER_HALF = .016                        # half a finger's width along the roll, where the band's rim would meet it
SLEEVE_SPEED = .035                       # m/s
SLEEVE_STALL = .025                       # the band will not go on if it falls this far behind the gripper's command


def band_rotation(along):
    """The band gripper's orientation for a band whose axis points ``along`` (level; toward its leading end): closing
    across the band, leaning BAND_LEAN back from straight down."""
    u = np.asarray(along, dtype=float)
    a = np.sin(BAND_LEAN) * u - np.cos(BAND_LEAN) * np.array((0., 0., 1.))
    c = np.cross(u, (0., 0., 1.))
    return np.column_stack((c, np.cross(a, c), a))


R_BAND_X = band_rotation((1., 0., 0.))    # the band lying along x, as in its cradle
R_BAND_Y = band_rotation((0., 1., 0.))    # the band along the roll

SEED = {"wind": (.42, 1.95, .49, 1.46, -1.16, -1.57), "clamp": (-.6, 1.53, 1.29, -1.04, -.2, 1.0),

        "band": (-.29, 1.04, .53, -.11, -.24, -.17)}

A_J = np.array((np.sin(JAW_TILT), 0., -np.cos(JAW_TILT)))           # down the holding gripper's fingers
C_J = np.array((np.cos(JAW_TILT), 0., np.sin(JAW_TILT)))            # from its rear finger to its front one
R_JAW = np.column_stack((-C_J, np.cross(A_J, -C_J), A_J))
REAR_JAW_AT = np.arctan2(-C_J[2], -C_J[0]) % (2 * np.pi)


def wind_rotation(turn):
    """The winding gripper: pointing along +y (the roll's axis), its closing axis turned ``turn`` about it from up."""
    a = np.array((0., 1., 0.))
    c = np.array((np.sin(turn), 0., np.cos(turn)))
    return np.column_stack((c, np.cross(a, c), a))


def tcp_of(joint_pos, arm):
    start = 7 * ARMS.index(arm)
    return ik.tcp_pose(np.asarray(joint_pos, dtype=float)[start:start + 6], BASES[arm])[0]


def into_wrist(angle):
    """The same pinch, half a turn round if need be, so that the wrist's travel covers it."""
    return (angle - WRIST[0]) % np.pi + WRIST[0]


def enclosing_circle(points, rounds=200):
    centre = points.mean(axis=0)
    for i in range(1, rounds + 1):
        far = points[np.linalg.norm(points - centre, axis=1).argmax()]
        centre = centre + (far - centre) / (i + 1)
    return centre, float(np.linalg.norm(points - centre, axis=1).max())


class PadReference:
    """Emit one absolute 14-D joint action per control step."""

    # None is the demonstration. The others are controls, each the demonstration with one thing left out:
    #   one_hand      the holding (left) arm is not used at all
    #   let_go        the holding gripper lets go of the finished roll before the band is on
    #   loose_roll    the first turn is not tightened against the pinned pad
    #   skewed_start  the winding gripper drifts 5 degrees off square as it rolls, and nothing is pushed back
    #   roll_only     development: stop when the roll is finished
    VARIANTS = (None, "roll_only", "one_hand", "let_go", "loose_roll", "skewed_start")

    def __init__(self, variant=None):
        if variant not in self.VARIANTS:
            raise ValueError(f"unknown demonstration variant {variant!r}")
        self.variant = variant
        self.skew = np.tan(SKEW) if variant == "skewed_start" else 0.
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.aperture = dict.fromkeys(ARMS, OPEN)
        self.offset = {arm: np.zeros(3) for arm in ARMS}      # correction of each arm's commanded point
        self.pose = dict.fromkeys(ARMS)                       # (point, rotation) each arm was last sent to
        self.worst_ik = 0.
        self.phases = []
        self.tick = 0
        self.turned = 0.                                      # how far the roll has been turned in all
        self.turn = 0.                                        # the winding gripper's closing axis, about the roll's axis
        self.strokes = 0
        self.off_centre = np.zeros(3)                         # the two strips' middle, in the winding gripper's frame
        self.path_from = np.zeros(2)
        self.thick = m.PAD_T                                  # what the winding gripper holds: how thick
        self.grip_y = 0.                                      # ... and where along the roll it took hold
        self.none_flat = 0
        self.stage = None                                     # the stage of plan() now running
        self.went = True                                      # whether the band went over the roll
        self.log = {"events": []}

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        self.state = state
        if state is not None:
            self.none_flat = self.none_flat + 1 if self.flat_left() == 0 else 0

    def actual_tcp(self, arm):
        return tcp_of(self.measured, arm)

    def gap(self, arm):
        return float(self.measured[7 * ARMS.index(arm) + 6]) * -2. * OPEN

    def note(self, what, **values):
        row = {"tick": self.tick, "event": what}
        for key, value in values.items():
            row[key] = np.round(np.asarray(value, dtype=float), 4).tolist() if not isinstance(value, (bool, str)) else value
        self.log["events"].append(row)

    def phase(self, name):
        self.phases.append((self.tick, name))

    # the pad, read from the simulator
    def strips(self):
        return self.state["strips"], self.state["strip_rot"]

    def right_end(self):
        """y of the roll's right-hand end (the end the winding gripper works at)."""
        return float(self.state["strips"][:, 1].min() - m.PAD_W / 2)

    def flat_left(self):
        """Strips still lying flat on the carton, counted from the far end."""
        pos, rot = self.strips()
        n = 0
        for i in range(m.STRIPS - 1, -1, -1):
            level = abs(rot[i][2, 0]) < np.sin(np.radians(12.)) and rot[i][2, 2] > 0
            if not (level and abs(pos[i][2] - m.PAD_Z) < .003):
                break
            n += 1
        return n

    def section(self, count=None):
        """Cross-section points (x, z) of the first ``count`` strips' corners."""
        pos, rot = self.strips()
        out = []
        for c, r in list(zip(pos, rot))[:count]:
            for sx in (-1, 1):
                for sz in (-1, 1):
                    p = c + r[:, 0] * sx * m.COLLIDER_W / 2 + r[:, 2] * sz * m.PAD_T / 2
                    out.append((p[0], p[2]))
        return np.array(out)

    def ring(self):
        """Middle (x, z) and outer radius of what has been rolled so far."""
        return enclosing_circle(self.section(max(m.STRIPS - self.flat_left(), 2)))

    # -- motion ---------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def solve(self, arm, point, rotation, seed=None, iters=40):
        joints, error, _ = ik.solve(np.asarray(point) + self.offset[arm], ik.matrix_to_quat_xyzw(rotation),
                                    self.joints[arm] if seed is None else seed, BASES[arm], iters=iters)
        self.worst_ik = max(self.worst_ik, error)
        self.pose[arm] = (np.asarray(point, dtype=float).copy(), np.asarray(rotation))
        return joints

    def aim(self, arm, point, rotation):
        """Send ``arm`` toward a pose this control step."""
        self.joints[arm] = self.solve(arm, point, rotation)

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def glide(self, seconds, targets):
        """Move in joint space to the given joint angles (smooth start and stop)."""
        start = {arm: self.joints[arm].copy() for arm in targets}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = ik.smoothstep(step / steps)
            for arm, goal in targets.items():
                self.joints[arm] = start[arm] + alpha * (np.asarray(goal) - start[arm])
            yield self.emit()

    def first(self, arm, point, rotation, seed, seconds):
        """From anywhere, in joint space, to a pose found from a seed."""
        goal = self.solve(arm, point, rotation, seed=np.asarray(seed, dtype=float), iters=300)
        yield from self.glide(seconds, {arm: goal})

    def move(self, seconds, targets):
        """Straight lines of the grippers' centre points, at fixed or smoothly turning orientation."""
        start = {arm: self.pose[arm] for arm in targets}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = ik.smoothstep(step / steps)
            for arm, (point, rotation) in targets.items():
                p0, r0 = start[arm]
                turn = rotation if r0 is rotation or np.allclose(r0, rotation) else ik.slerp_matrix(r0, rotation, alpha)
                self.joints[arm] = self.solve(arm, p0 + alpha * (np.asarray(point) - p0), turn)
            yield self.emit()

    def trim(self, arm, repeats=3, limit=.025):
        """Shift the command by the measured error (the arms sag under their own weight and under load)."""
        for _ in range(repeats):
            point, rotation = self.pose[arm]
            error = point - self.actual_tcp(arm)
            self.offset[arm] = np.clip(self.offset[arm] + np.clip(error, -.006, .006), -limit, limit)
            self.joints[arm] = self.solve(arm, point, rotation)
            yield from self.hold(.3)
        self.note(f"trim {arm}", offset=self.offset[arm], left_over=self.pose[arm][0] - self.actual_tcp(arm))

    def grip(self, arm, finger, seconds):
        start = self.aperture[arm]
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            self.aperture[arm] = start + step / steps * (finger - start)
            yield self.emit()

    def home(self, arms, seconds):
        yield from self.glide(seconds, {arm: np.asarray(READY, dtype=float) for arm in arms})
        for arm in arms:
            self.pose[arm] = None
            self.offset[arm] = np.zeros(3)

    # -- the two grippers at the roll ---------------------------------------------------------
    def wind_point(self, held, strip=0, out=0.):
        """Where the winding gripper's centre point goes to hold (x, z) = ``held`` with its fingertips BITE in
        from the right-hand end of ``strip`` (``out``: drawn back along the roll by that much)."""
        end = float(self.state["strips"][strip][1]) - m.PAD_W / 2
        return np.array((held[0], end + BITE - PAD_TIP_PAST_TCP - out, held[-1]))

    # -- 1. the edge ---------------------------------------------------------------------------
    def take_edge(self):
        self.phase("reach the pad's near edge")
        held = np.array((m.PAD_X0 + m.STRIP_W, m.PAD_Z))
        self.aperture["right"] = -EDGE_OPEN / 2
        for attempt in range(3):
            pos, _ = self.strips()
            held = np.array(((pos[0][0] + pos[1][0]) / 2, (pos[0][2] + pos[1][2]) / 2))
            if attempt == 0:
                yield from self.first("right", self.wind_point(held, 0, OUT) + (0., 0., .04), wind_rotation(0.), SEED["wind"], 2.0)
            yield from self.move(.8, {"right": (self.wind_point(held, 0, OUT), wind_rotation(0.))})
            yield from self.trim("right")
            yield from self.move(1.0, {"right": (self.wind_point(held), wind_rotation(0.))})
            yield from self.trim("right", 2)
            yield from self.grip("right", 0., .5)
            yield from self.hold(.3)
            took = self.gap("right") > m.PAD_T - .0015
            self.note("edge pinch", attempt=attempt, opening_mm=1000 * self.gap("right"), held=bool(took),
                      fingertips_from_edge_mm=1000 * (self.actual_tcp("right") - self.wind_point(held)))
            if took:
                break
            yield from self.grip("right", -EDGE_OPEN / 2, .3)
        self.phase("lay the edge over")
        steps = round(FIRST_TURN / WIND_RATE * CONTROL_HZ)
        self.grip_y = float(self.pose["right"][0][1])
        for step in range(1, steps + 1):
            turn = FIRST_TURN * step / steps
            self.grip_y -= self.skew * m.CORE_R * FIRST_TURN / steps
            x, z = m.pinch_pose(turn)
            self.aim("right", (m.PAD_X0 + x, self.grip_y, m.PAD_Z + z), wind_rotation(turn))
            yield self.emit()
        self.turned, self.turn, self.strokes = FIRST_TURN, FIRST_TURN, 1
        yield from self.hold(.3)
        pos, rot = self.strips()
        self.note("edge laid over", turned_deg=np.degrees(self.turned), first_strip_mm=1000 * pos[0],
                  folds_deg=np.degrees(self.state["folds"][:8]), strips_rolled=m.STRIPS - self.flat_left())

    # -- 2. the holding gripper: down on the roll from above, and with it from then on ----------------------
    def all_on(self):
        """The whole pad is on the roll: no strip has lain flat on the carton for three control steps."""
        return self.none_flat >= 3

    def edge(self):
        """Middle (x, z) of the first two strips, and their turn about the roll's axis."""
        pos, rot = self.strips()
        return (pos[0] + pos[1])[[0, 2]] / 2, float(np.arctan2(rot[0][0, 2], rot[0][0, 0]))

    def jaw_point(self, centre, lift=0.):
        """Where the holding gripper's centre point goes for a roll whose middle is at ``centre`` (x, z): over the
        middle of the roll's length."""
        along = float(self.state["strips"][:, 1].mean()) + JAW_ALONG
        return np.array((centre[0], along, centre[1] + lift)) - A_J * (PAD_TIP_PAST_TCP - JAW_TIP)

    def bring_jaws(self):
        self.phase("holding gripper comes down on the curl")
        centre, radius = self.ring()
        self.aperture["left"] = -(2 * radius + .030) / 2
        yield from self.first("left", self.jaw_point(centre, .06), R_JAW, SEED["clamp"], 2.0)
        yield from self.trim("left", 2)
        centre, radius = self.ring()
        yield from self.move(.8, {"left": (self.jaw_point(centre), R_JAW)})
        yield from self.trim("left", 2)
        yield from self.jaws_firm()

    def jaws_firm(self, squeeze=None):
        """Close on the roll where it is, firmly: each finger is sent JAW_SQUEEZE into it."""
        centre, radius = self.ring()
        yield from self.move(.3, {"left": (self.jaw_point(centre), R_JAW)})
        yield from self.grip("left", -max(0., 2 * radius - 2 * (JAW_SQUEEZE if squeeze is None else squeeze)) / 2, .4)

    def jaws_guide(self, sized=False):
        """This control step: stay round the roll where it now is, as a fence. ``sized``: the fingers stand at the
        tight roll's size, so the roll cannot grow as it turns; otherwise just clear of the roll as it is."""
        centre, radius = self.ring()
        self.aim("left", self.jaw_point(centre), R_JAW)
        self.aperture["left"] = -(self.tight() + DIE_SLACK if sized else 2 * radius + GUIDE_SLACK) / 2
        return centre, radius

    def tail_short(self, centre, radius):
        """How much further (rad) the roll must turn to bring the pad's far edge just under one of the holding
        gripper's two fingers (negative: it is that far past one)."""
        pos, rot = self.strips()
        edge = pos[-1] + rot[-1][:, 0] * m.STRIP_W / 2
        at = np.arctan2(edge[2] - centre[1], edge[0] - centre[0])
        short = (at - REAR_JAW_AT - m.STRIP_W / 2 / radius) % np.pi
        return short if short < np.pi - TAIL_PAST else short - np.pi

    # -- 3. the winding gripper: let go, pat the core back, take hold again half a turn back, stroke ------
    def wind_to(self, seconds, held, turn, strip=0, out=0.):
        """Move the winding gripper to hold (x, z) = ``held`` at closing-axis turn ``turn``, fingertips BITE in from
        the end of ``strip``; the turn is taken the direct way (the short way round could leave the wrist's travel)."""
        p0 = self.pose["right"][0].copy()
        a0 = self.turn
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            s = ik.smoothstep(step / steps)
            goal = self.wind_point(held, strip, out)
            self.turn = a0 + s * (turn - a0)
            self.aim("right", p0 + s * (goal - p0), wind_rotation(self.turn))
            yield self.emit()

    def leave(self):
        """Open the fingertips clear of what they hold and draw them out along the roll. Where a fingertip was
        between two turns of the roll it drags them out a little; push those strips back in to where they were."""
        before = self.state["strips"][:, 1].copy()
        point, _ = self.pose["right"]
        yield from self.grip("right", -(self.thick + 2 * CLEAR) / 2, .3)
        for s in range(1, round(.5 * CONTROL_HZ) + 1):
            self.aim("right", point - (0., OUT * ik.smoothstep(s / round(.5 * CONTROL_HZ)), 0.), wind_rotation(self.turn))
            yield self.emit()
        dragged = before - self.state["strips"][:, 1]
        self.note("fingers drawn out", stroke=self.strokes, dragged_out_mm=1000 * float(dragged.max()), strip=float(dragged.argmax()))
        yield from self.grip("right", 0., .3)                   # closed fingertips, end on, are what pats
        if self.skew:
            return
        done = set()
        for _ in range(3):
            out = before - self.state["strips"][:, 1]
            out[sorted(done)] = 0.
            worst = int(out.argmax())
            if out[worst] < PAT_OVER:
                break
            done.update((worst - 1, worst, worst + 1) if 0 < worst < m.STRIPS - 1 else (worst,))
            yield from self.pat(worst, before[worst])

    def pat(self, strip, back_to):
        """Push one strip back along the roll with the closed fingertips, end on against its right-hand end, until
        its middle is at y = ``back_to``."""
        pos, rot = self.strips()
        turn = into_wrist(float(np.arctan2(rot[strip][0, 2], rot[strip][0, 0])))
        yield from self.wind_to(.6, pos[strip][[0, 2]], turn, strip, out=OUT)
        yield from self.wind_to(.5, pos[strip][[0, 2]], turn, strip, out=BITE + .004)     # 4 mm short of that strip's end
        point = self.pose["right"][0].copy()
        start = float(back_to - self.state["strips"][strip, 1])
        for _ in range(round((start + .010) / PAT_SPEED * CONTROL_HZ)):
            if back_to - self.state["strips"][strip, 1] <= PAT_LEFT:
                break
            point = point + (0., PAT_SPEED * DT, 0.)
            self.aim("right", point, wind_rotation(self.turn))
            yield self.emit()
        self.note("patted back", strip=float(strip), out_mm=1000 * start,
                  left_mm=1000 * float(back_to - self.state["strips"][strip, 1]))
        yield from self.wind_to(.4, self.state["strips"][strip][[0, 2]], self.turn, strip, out=OUT)

    def square_end(self):
        """The winding gripper draws the turns it holds a little way out of the roll at every stroke. With the roll
        held, push the strips that stand out of its right-hand end back level with the rest."""
        self.phase("square the roll's end")
        yield from self.grip("right", 0., .2)
        before = float(np.ptp(self.state["strips"][:, 1]))
        done = set()
        for _ in range(SQUARE_PATS):
            y = self.state["strips"][:, 1]
            level = float(np.median(y))
            out = level - y
            out[sorted(done)] = 0.
            worst = int(out.argmax())
            if out[worst] < SQUARE_OVER:
                break
            done.update((worst - 1, worst, worst + 1) if 0 < worst < m.STRIPS - 1 else (worst,))
            yield from self.pat(worst, level)
        self.note("end squared", turns_offset_before_mm=1000 * before, turns_offset_mm=1000 * float(np.ptp(self.state["strips"][:, 1])))

    def wall(self):
        """The roll's wall at its top, every layer of it: what the winding gripper takes hold of once the first turn
        has been tightened, one fingertip inside the roll and one above it, so that it never has to find a gap
        between layers. Returns where the fingers' middle goes (x, z), the closing-axis turn, how thick the wall is
        there (gaps between its layers included), and the strip whose right-hand end stands furthest out (the
        fingertips reach BITE in from the others' ends at least)."""
        pos, rot = self.strips()
        centre, _ = self.ring()
        rolled = max(m.STRIPS - self.flat_left(), 2)
        # every strip any part of which is under the fingertip's 15 mm width, above the roll's middle
        top = [i for i in range(rolled) if abs(pos[i][0] - centre[0]) < WALL_HALF and pos[i][2] > centre[1]]
        outer = max(top, key=lambda i: pos[i][2])
        low = min(pos[i][2] for i in top)
        turn = into_wrist(float(np.arctan2(rot[outer][0, 2], rot[outer][0, 0])))
        return (np.array((centre[0], (pos[outer][2] + low) / 2)), turn, float(pos[outer][2] - low) + m.PAD_T,
                min(top, key=lambda i: pos[i][1]), [(i, round(1000 * float(pos[i][0] - centre[0]), 1),
                                                     round(1000 * float(pos[outer][2] - pos[i][2]), 1)) for i in top])

    def take(self, wall=False):
        """Come back round and take hold again: lined up clear of the roll's end, then straight in. Before the first
        turn has been tightened it is the pad's first two strips that are taken; after that, the roll's wall at its
        top. If the fingertips do not go in, or close on nothing, they are drawn back and tried again wider."""
        def target():
            if wall:
                return self.wall()
            held, angle = self.edge()
            return held, into_wrist(angle), m.PAD_T, 0, []

        for attempt in range(3):
            held, turn, thick, strip, layers = target()
            self.thick = thick + attempt * .004
            self.aperture["right"] = -(self.thick + 2 * TAKE_CLEAR) / 2
            yield from self.wind_to(.9, held, turn, strip, out=OUT)
            yield from self.trim("right", 3)
            held, turn, thick, strip, layers = target()
            yield from self.wind_to(.3, held, turn, strip, out=OUT)
            yield from self.trim("right", 1)
            yield from self.wind_to(.8, held, turn, strip)
            yield from self.hold(.2)
            short = self.pose["right"][0] - self.actual_tcp("right")          # how far the fingertips are from where they were sent
            yield from self.grip("right", 0., .4)
            yield from self.hold(.2)
            took = self.gap("right") > max(.0015, thick / 2) and abs(short[1]) < .005   # the wall's layers close up under the pinch
            self.note("taken again", after_stroke=self.strokes, attempt=attempt,
                      taken="the roll's wall at its top" if wall else "the pad's edge",
                      wrist_turn_deg=np.degrees(self.turn), thickness_mm=1000 * thick, opening_mm=1000 * self.gap("right"),
                      fingertips_short_mm=1000 * short, held=bool(took), strips_rolled=m.STRIPS - self.flat_left(),
                      layers=str(layers))
            if took:
                break
            yield from self.grip("right", -(self.thick + 2 * TAKE_CLEAR + .004) / 2, .3)
            yield from self.wind_to(.5, held, turn, strip, out=OUT)
        else:
            raise RuntimeError("the winding gripper closed on nothing")
        self.grip_y = float(self.pose["right"][0][1])
        # where the two strips' middle is in the gripper's own (turning) frame: the second stroke steers by it
        edge, _ = self.edge()
        tcp = self.actual_tcp("right")
        self.off_centre = wind_rotation(self.turn).T @ np.array((edge[0] - tcp[0], 0., edge[1] - tcp[2]))
        self.path_from = edge - m.pinch_pose(self.turned)

    def shift_grip(self, wall=True, lift=False):
        """The winding gripper lets go and takes hold again further back round the roll, the holding gripper keeping
        the roll shut meanwhile. ``lift``: the pad's edge is on the underside of the roll, so the holding gripper
        lifts the roll first, for a fingertip to come under it."""
        self.phase(f"winding gripper shifts its grip (after stroke {self.strokes})")
        rotation = self.pose["left"][1]
        if lift:
            yield from self.move(.6, {"left": (self.pose["left"][0] + (0., 0., LIFT), rotation)})
        yield from self.leave()
        yield from self.take(wall)
        if lift:
            yield from self.move(.6, {"left": (self.pose["left"][0] - (0., 0., LIFT), rotation)})

    def stroke(self):
        """Second stroke: carry the two strips on round until the wrist runs out. That closes the first turn."""
        self.strokes += 1
        self.phase(f"turn the roll (stroke {self.strokes})")
        while self.turn < SECOND_TO - 1e-6:
            self.turn += WIND_RATE * DT
            self.turned += WIND_RATE * DT
            self.grip_y -= self.skew * self.ring()[1] * WIND_RATE * DT
            aim = self.path_from + m.pinch_pose(self.turned)
            shift = wind_rotation(self.turn) @ self.off_centre
            self.aim("right", (aim[0] - shift[0], self.grip_y, aim[1] - shift[2]), wind_rotation(self.turn))
            self.jaws_guide()
            yield self.emit()
        for _ in range(round(.2 * CONTROL_HZ)):
            self.jaws_guide()
            yield self.emit()
        self.note_roll("first turn closed")

    def note_roll(self, what, **more):
        pos, _ = self.strips()
        centre, radius = self.ring()
        self.note(what, stroke=self.strokes, turned_deg=np.degrees(self.turned), wrist_turn_deg=np.degrees(self.turn),
                  strips_rolled=m.STRIPS - self.flat_left(), ring_mm=1000 * np.r_[centre, radius], tight_mm=1000 * self.tight(),
                  turns=float(np.abs(self.state["folds"]).sum() / (2 * np.pi)),
                  turns_offset_mm=1000 * float(np.ptp(pos[:, 1])), all_on=self.all_on(), **more)

    def tight(self):
        return m.tight_diameter(m.STRIPS - self.flat_left())

    # -- 4. tighten the first turn against the pinned pad, then roll on ------------------------------------
    def wind(self, done, rate, hold=False, fence=None):
        """Turn what the winding gripper holds until ``done()`` or the wrist runs out. A closed turn winds up like a
        clock spring when its inner end is turned, so only the wrist is driven: the centre point is sent, each step,
        to where it is, PRESS lower, so that the arm bears the roll down on the pad and otherwise goes where the
        roll takes it. ``hold``: the roll is also kept from rolling on (its middle is held where it is along the
        pad), so that turning its inner end has to tighten it. ``fence``: what the holding gripper does meanwhile
        ("sized": a fence at the tight roll's size that goes with the roll; "still": that fence, staying put)."""
        middle = float(self.ring()[0][0])
        pull = 0.
        while self.turn < WRIST[1] - 1e-6 and not done():
            self.turn += rate * DT
            self.turned += rate * DT
            self.grip_y -= self.skew * self.ring()[1] * rate * DT
            tcp = self.actual_tcp("right")
            if hold:
                pull = float(np.clip(pull - HOLD_GAIN * (self.ring()[0][0] - middle), -.025, .025))
            self.aim("right", (tcp[0] + pull, self.grip_y, tcp[2] - PRESS), wind_rotation(self.turn))
            if fence == "loose":
                self.jaws_guide()
            elif fence == "sized":
                self.jaws_guide(sized=self.variant != "loose_roll")
            elif fence == "still" and self.variant != "loose_roll":
                self.aperture["left"] = -(self.tight() + DIE_SLACK) / 2
            yield self.emit()

    def pin_pad(self):
        """The holding gripper leaves the roll to the winding gripper and bears down, fingers closed, on the part
        of the pad still lying flat: held there, the pad can be wound against."""
        self.phase("holding gripper pins the flat part of the pad")
        _, radius = self.ring()
        yield from self.grip("left", -(2 * radius + .030) / 2, .3)
        point, _ = self.pose["left"]
        yield from self.move(.5, {"left": (point - A_J * .06, R_JAW)})
        yield from self.grip("left", 0., .3)
        pos, _ = self.strips()
        tip = pos[PIN_STRIP] + (0., PIN_Y - m.PAD_W / 2, m.PAD_T / 2)
        over = tip - A_J * PAD_TIP_PAST_TCP + (0., 0., .04)
        yield from self.move(.8, {"left": (over, R_JAW)})
        yield from self.move(.6, {"left": (over - (0., 0., .04 + PIN_PRESS), R_JAW)})
        yield from self.hold(.2)
        self.note("pad pinned", strip=PIN_STRIP, fingertips_short_of_command_mm=1000 * (self.actual_tcp("left") - self.pose["left"][0]),
                  flat_strips=self.flat_left())

    def tighten(self):
        """Wind the closed first turn tight, on the spot, against the pinned pad."""
        yield from self.pin_pad()
        self.phase("tighten the first turn")
        before, far = 2 * self.ring()[1], float(self.state["strips"][-1][0])

        def done():
            return 2 * self.ring()[1] <= self.tight() + CINCH_TO or abs(self.state["strips"][-1][0] - far) > PIN_SLIP

        yield from self.wind(done, CINCH_RATE, hold=True)
        yield from self.hold(.2)
        self.strokes += 1
        self.note_roll("first turn tightened", from_mm=1000 * before, pad_pulled_mm=1000 * (far - float(self.state["strips"][-1][0])))
        self.phase("holding gripper comes back to the roll")
        point, _ = self.pose["left"]
        yield from self.move(.4, {"left": (point + (0., 0., .05), R_JAW)})
        yield from self.bring_jaws()

    def under_finger(self):
        """The whole pad is on the roll and its far edge is under one of the holding gripper's fingers."""
        return self.all_on() and self.tail_short(*self.ring()) <= 0.

    def tuck(self):
        """The whole pad is on the roll, but its far edge is still short of the holding gripper's rear finger, and a
        roll lifted like that lets the edge hang open. The winding gripper lays its closed fingertips on top of the
        roll at its right-hand end and bears down a little: with the carton under the roll and the holding
        gripper's two fingers fore and aft, the roll is shut in on all four sides and cannot grow. The holding
        gripper eases off to a fence of that width and moves on along the carton; its rear finger rolls the roll
        along under the bearing fingertips until the pad's far edge has come up under that finger. Then it closes."""
        self.phase("roll the roll on, shut in, until the pad's far edge is under a finger")
        pos, _ = self.strips()
        centre, radius = self.ring()
        top = int(np.argmax(pos[:, 2]))
        height = float(pos[top][2]) + m.PAD_T / 2 + TIP_THICK       # the centre point with the lower fingertip lying on the roll
        yield from self.grip("right", 0., .2)
        yield from self.wind_to(1.0, (centre[0], height + .030), 0., top, out=OUT)
        yield from self.wind_to(.6, (centre[0], height + .030), 0., top)
        yield from self.wind_to(.4, (centre[0], height - TOP_PRESS), 0., top)
        along = float(self.pose["right"][0][1])
        start = self.pose["left"][0].copy()
        yield from self.grip("left", -(2 * radius + PUSH_SLACK) / 2, .3)
        moved = 0.
        while moved < PUSH_MOST and not self.under_finger():
            moved += PUSH_SPEED * DT
            self.aim("left", start + (moved, 0., 0.), R_JAW)
            self.aim("right", (float(self.ring()[0][0]), along, height - TOP_PRESS), wind_rotation(0.))
            yield self.emit()
        yield from self.jaws_firm()
        self.note_roll("far edge brought round", pushed_mm=1000 * moved, far_edge_short_deg=np.degrees(self.tail_short(*self.ring())),
                       roll_moved_mm=1000 * (self.ring()[0][0] - centre[0]), fence_mm=1000 * (2 * radius + PUSH_SLACK),
                       across_before_mm=2000 * radius)
        point = self.pose["right"][0]
        yield from self.move(.4, {"right": (point + (0., 0., .030), wind_rotation(0.))})
        yield from self.move(.5, {"right": (point + (0., -OUT, .030), wind_rotation(0.))})

    def second_stroke(self):
        """The holding gripper comes down on the curl; the winding gripper shifts its grip on the pad's edge and
        turns it on until the wrist runs out. That closes the first turn, loosely."""
        yield from self.bring_jaws()
        yield from self.shift_grip(wall=False)
        yield from self.stroke()
        yield from self.jaws_firm()

    def third_stroke(self):
        """The edge is taken once more (it is underneath now) and the roll rolls on over the rest of the pad."""
        loose = self.variant == "loose_roll"
        yield from self.shift_grip(wall=False, lift=True)
        if TIGHTEN and not loose:
            # the turn is tightened on the spot against the pinned pad; the rest of the pad is then rolled on inside
            # a fence at the tight roll's size, the winding gripper holding the roll by its wall
            yield from self.tighten()
            while not self.under_finger():
                yield from self.shift_grip()
                self.strokes += 1
                self.phase(f"roll on (stroke {self.strokes})")
                self.none_flat = 0
                yield from self.wind(self.all_on, WIND_RATE, fence="sized")
                if self.all_on():
                    self.phase("turn the far edge up under a finger")
                    yield from self.wind(self.under_finger, WIND_RATE, hold=True, fence="still")
                yield from self.jaws_firm()
                self.note_roll("stroke done", far_edge_short_deg=np.degrees(self.tail_short(*self.ring())))
                if self.strokes > 8:
                    raise RuntimeError("still rolling after 8 strokes")
            return
        self.strokes += 1
        self.phase(f"roll on (stroke {self.strokes})")
        self.none_flat = 0
        yield from self.wind(self.all_on, WIND_RATE, fence="loose")
        yield from self.jaws_firm()
        self.note_roll("stroke done", far_edge_short_deg=np.degrees(self.tail_short(*self.ring())))
        if not self.all_on():
            self.note("the wrist ran out before the whole pad was on the roll", flat_strips=self.flat_left())

    def finish_roll(self):
        """The winding gripper lets go (what its fingertip drags out of the roll it pushes back) and its arm leaves.
        The holding gripper keeps the roll where it is."""
        self.phase("winding gripper leaves the roll")
        yield from self.leave()
        yield from self.home(("right",), 1.4)
        self.note_roll("roll finished", across_mm=2000 * self.ring()[1], strokes=self.strokes,
                       far_edge_short_deg=np.degrees(self.tail_short(*self.ring())), opening_mm=1000 * self.gap("left"))

    # -- 5. the band ---------------------------------------------------------------------------------
    def roll_axis(self):
        """Middle (x, z) of the finished roll, its radius and its two ends along y."""
        pos, _ = self.strips()
        centre, radius = enclosing_circle(self.section())
        return centre, radius, float(pos[:, 1].min() - m.PAD_W / 2), float(pos[:, 1].max() + m.PAD_W / 2)

    def carry_roll(self):
        """The holding gripper never lets go of the finished roll and never lifts it: lifted, or held any less
        firmly, the roll opens (the pad's far edge is not under a finger). It tightens its hold and slides the roll
        along its own length on the carton until BAND_OVERHANG of it sticks out over the carton's right-hand edge.
        The carton still holds the pad's far edge shut under the roll; the band goes on the part that sticks out."""
        self.phase("slide the roll out over the carton's edge")
        centre, radius, right, _ = self.roll_axis()
        yield from self.grip("left", -max(0., 2 * radius - 2 * CARRY_SQUEEZE) / 2, .4)
        start = self.pose["left"][0].copy()
        shift = (m.CARTON_Y0 - BAND_OVERHANG) - right
        yield from self.move(.3, {"left": (start - (0., 0., SLIDE_PRESS), R_JAW)})
        yield from self.move(abs(shift) / SLIDE_SPEED, {"left": (start + (0., shift, -SLIDE_PRESS), R_JAW)})
        for _ in range(2):                                      # the roll gives a little in the fingers: make up what is short
            yield from self.hold(.3)
            short = (m.CARTON_Y0 - BAND_OVERHANG) - self.roll_axis()[2]
            point = self.pose["left"][0]
            yield from self.move(max(.2, abs(short) / SLIDE_SPEED), {"left": (point + (0., float(np.clip(short, -.02, .02)), 0.), R_JAW)})
        yield from self.hold(.4)
        centre, radius, right, left = self.roll_axis()
        pos, rot = self.strips()
        self.note("roll held for the band", axis_mm=1000 * centre, ends_y_mm=1000 * np.array((right, left)),
                  over_the_edge_mm=1000 * (m.CARTON_Y0 - right), across_mm=2000 * radius,
                  turns_offset_mm=1000 * float(np.ptp(pos[:, 1])), opening_mm=1000 * self.gap("left"),
                  turns=float(np.abs(self.state["folds"]).sum() / (2 * np.pi)),
                  far_edge_short_deg=np.degrees(self.tail_short(centre, radius)),
                  droop_deg=np.degrees(np.arcsin(np.clip(np.abs(rot[:, 2, 1]).max(), 0., 1.))))

    def band_point(self, middle, along):
        """Where the band gripper's centre point is for a band whose middle is at ``middle`` and whose axis points
        ``along``: BAND_HOLD from the band's trailing rim, just above its axis."""
        return np.asarray(middle, dtype=float) - np.asarray(along) * (m.BAND_L / 2 - BAND_HOLD) + (0., 0., BAND_GRIP_UP)

    def fetch_band(self):
        self.phase("take the band from its cradle")
        x, y = np.array((1., 0., 0.)), np.array((0., 1., 0.))
        self.aperture["right"] = -(m.BAND_OD + .024) / 2
        yield from self.first("right", self.band_point(self.state["band"], x) + (0., 0., .08), R_BAND_X, SEED["band"], 2.0)
        yield from self.trim("right", 2)
        yield from self.move(1.0, {"right": (self.band_point(self.state["band"], x), R_BAND_X)})
        yield from self.trim("right", 2)
        yield from self.grip("right", 0., .6)                   # full grip: the band must not swing when the roll hangs in it
        yield from self.hold(.3)
        self.note("band gripped", opening_mm=1000 * self.gap("right"),
                  band_from_gripper_mm=1000 * (self.state["band"] - self.actual_tcp("right")))
        self.phase("bring the band to the roll's end")
        centre, radius, right, left = self.roll_axis()
        ready = np.array((centre[0], right - BAND_LEAD - m.BAND_L / 2, centre[1]))
        lifted = self.pose["right"][0] + (0., 0., .10)
        yield from self.move(1.0, {"right": (lifted, R_BAND_X)})
        yield from self.move(2.2, {"right": (self.band_point(ready, y) + (0., 0., .04), R_BAND_Y)})
        yield from self.move(.8, {"right": (self.band_point(ready, y), R_BAND_Y)})
        yield from self.line_up(ready)

    def line_up(self, wanted, rounds=4):
        """Trim the band gripper until the band's own middle (not the gripper's) is at ``wanted``."""
        for _ in range(rounds):
            error = np.asarray(wanted) - self.state["band"]
            point, rotation = self.pose["right"]
            yield from self.move(.3, {"right": (point + np.clip(error, -.008, .008), rotation)})
            yield from self.hold(.2)
        self.note("band lined up", band_mm=1000 * self.state["band"], off_mm=1000 * (np.asarray(wanted) - self.state["band"]),
                  band_axis=self.state["band_rot"][:, 0])

    def sleeve(self):
        """Slide the band over the roll from its right-hand end as far as the holding gripper's fingers, steering
        the band's mouth onto the roll's axis as it goes. Returns whether it went on."""
        self.phase("slide the band over the roll")
        centre, radius, right, left = self.roll_axis()
        jaws_y = float(self.actual_tcp("left")[1])
        # the band's left rim stops short of the holding fingers and of the carton's edge, whichever comes first
        stop = min(jaws_y - FINGER_HALF - BAND_SHORT, m.CARTON_Y0 - EDGE_GAP)
        start = self.state["band"].copy()
        travel = stop - (start[1] + m.BAND_L / 2)
        steps = round(travel / SLEEVE_SPEED * CONTROL_HZ)
        point, rotation = self.pose["right"]
        point = point.copy()
        start_point, went = float(point[1]), True
        for step in range(1, steps + 1):
            centre, _, _, _ = self.roll_axis()
            band = self.state["band"]
            point[1] += travel / steps
            point[0] += float(np.clip(.3 * (centre[0] - band[0]), -.0015, .0015))
            point[2] += float(np.clip(.3 * (centre[1] - band[2]), -.0015, .0015))
            self.aim("right", point, rotation)
            yield self.emit()
            if point[1] - start_point - (float(self.state["band"][1]) - start[1]) > SLEEVE_STALL:
                went = False                                    # the band has stopped against the roll's end
                break
        yield from self.hold(.4)
        band = self.state["band"]
        self.note("band on the roll" if went else "the band will not go over the roll",
                  band_left_rim_y_mm=1000 * (band[1] + m.BAND_L / 2), stop_y_mm=1000 * stop,
                  roll_out_of_band_mm=1000 * np.array((band[1] - m.BAND_L / 2 - right, left - band[1] - m.BAND_L / 2)),
                  band_off_axis_mm=1000 * (band[[0, 2]] - centre), roll_across_mm=2000 * radius, bore_mm=1000 * m.BAND_ID)
        self.went = went

    def hand_over(self):
        """The holding gripper lets go and leaves: from here the band is all that holds the roll shut."""
        if not self.went:
            if self.variant is None:
                raise RuntimeError("the band will not go over the roll")
            return                                              # a control: the band goes back, then the roll is let go
        self.phase("hand over: holding gripper lets go")
        _, radius, _, _ = self.roll_axis()
        yield from self.grip("left", -(2 * radius + .040) / 2, .5)
        point, rotation = self.pose["left"]
        yield from self.move(.6, {"left": (point - A_J * .06, rotation)})
        yield from self.home(("left",), 1.6)
        yield from self.hold(.5)
        pos, rot = self.strips()
        self.note("roll held by the band alone", across_mm=2000 * self.roll_axis()[1], turns_offset_mm=1000 * float(np.ptp(pos[:, 1])),
                  roll_tilt_in_band_deg=np.degrees(np.arccos(np.clip(abs(float(np.mean(rot[:, :, 1], axis=0) @ self.state["band_rot"][:, 0])), 0., 1.))))

    def return_band(self):
        self.phase("lay the banded roll in the cradle" if self.went else "put the band back in the cradle")
        x = np.array((1., 0., 0.))
        point, rotation = self.pose["right"]
        if not self.went:
            yield from self.move(1.4, {"right": (point - (0., .12, 0.), rotation)})
            point, rotation = self.pose["right"]
        # up off the carton first (dragged along it, the roll is drawn out of the band), then straight back along the roll
        yield from self.move(.8, {"right": (point + (0., 0., .030), rotation)})
        yield from self.move(1.6, {"right": (point + (0., -.14, .030), rotation)})
        over = self.band_point(m.BAND_START, x) + (0., 0., .10)
        yield from self.move(2.6, {"right": (over, R_BAND_X)})
        yield from self.hold(.3)
        for _ in range(3):                                      # put the band's own middle over the cradle's
            error = (m.BAND_START - self.state["band"]) * (1., 1., 0.)
            p, r = self.pose["right"]
            yield from self.move(.3, {"right": (p + np.clip(error, -.008, .008), r)})
            yield from self.hold(.2)
        p, r = self.pose["right"]
        yield from self.move(1.4, {"right": (p - (0., 0., .10 - .002), r)})
        yield from self.hold(.3)
        yield from self.grip("right", -(m.BAND_OD + .024) / 2, .5)
        yield from self.hold(.3)
        self.note("banded roll laid down", band_mm=1000 * self.state["band"], from_cradle_mm=1000 * (self.state["band"] - m.BAND_START),
                  band_axis=self.state["band_rot"][:, 0])
        self.phase("away")
        p, r = self.pose["right"]
        yield from self.move(.8, {"right": (p - r[:, 2] * .10, r)})
        yield from self.home(("right",), 1.8)
        if not self.went:
            yield from self.let_go()

    # -- controls -----------------------------------------------------------------------------------------
    def alone(self):
        """Control: the other arm is not used. Nothing holds the curl when the winding gripper lets go to shift
        its grip, so the curl is lost each time; it is tried twice."""
        for attempt in range(2):
            rolled = m.STRIPS - self.flat_left()
            self.phase("winding gripper lets go with nothing holding the curl")
            yield from self.leave()
            yield from self.hold(1.5)
            self.note("let go with the other arm still", attempt=attempt, strips_rolled_before=rolled,
                      strips_rolled_after=m.STRIPS - self.flat_left(), folds_deg=np.degrees(self.state["folds"][:8]))
            if attempt == 0:
                yield from self.take_edge()
        yield from self.home(("right",), 1.6)

    def let_go(self):
        """The holding gripper opens and leaves the roll where it is."""
        self.phase("holding gripper lets go of the roll")
        across = 2 * self.ring()[1]
        yield from self.grip("left", OPEN, .5)
        point, rotation = self.pose["left"]
        yield from self.move(.8, {"left": (point - A_J * .09, rotation)})
        yield from self.home(("left",), 1.6)
        yield from self.hold(2.)
        self.note_roll("roll let go", across_before_mm=1000 * across, across_after_2s_mm=2000 * self.ring()[1])

    def plan(self):
        """The stages of the demonstration, or of the control being run, in order."""
        if self.variant == "one_hand":
            return ("take_edge", "alone")
        rolled = ("take_edge", "second_stroke", "third_stroke", "finish_roll")
        if self.variant == "roll_only":
            return rolled
        if self.variant == "let_go":
            return rolled + ("let_go",)
        return rolled + ("carry_roll", "fetch_band", "sleeve", "hand_over", "return_band")

    def sequence(self, start=None):
        """One action per control step. ``start`` (development only): begin at that stage, the simulator and this
        object having been put back as they were when it began."""
        stages = self.plan()
        for name in stages[stages.index(start) if start else 0:]:
            self.stage = name
            yield from getattr(self, name)()
        self.phase("done")
        if self.variant:                                        # a control or a development run stops here
            yield from self.hold(3.)
            return
        while True:
            yield self.emit()
