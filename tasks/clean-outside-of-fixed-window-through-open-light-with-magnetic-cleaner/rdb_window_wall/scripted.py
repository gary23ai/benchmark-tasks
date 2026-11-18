"""The demonstration: closed loop on the robot's joint readings and on the simulator's poses of the two halves.

Plan. The left arm takes the blue (inside) half from above by its handle bar, turns it up to face the fixed
light and sets it on the indoor face of the glass near the stile. The right arm takes the white (outside)
half by the near end of its handle bar, hand lying along the bar and 10 degrees down, turns it up, carries
it out through the open light with the hand pointing outwards, swings it round to lie along the outdoor face
past the stile, sets it on the glass 55 mm below the inside half, where the magnets barely act, and slides it
up until the halves pair. It lets go and comes back in through the opening. The left arm then slides the
inside half along the glass, so the outside half is drawn across the marker band, and lets go.

What it reads. Joint angles and gripper openings (to correct each arm's tracking error by forward kinematics
and to check each grip), and, through ``read``, the simulator's pose of each half and whether the scorer sees
them paired. The poses are used to aim each grasp at where the half really lies, to check that a lifted half
came up with the gripper (and to grasp again if it did not), to line the outside half up with the inside
half as it is slid on (and to slide it on again if they did not pair), to wait for the outside half when it
falls behind in the stroke, and to end the stroke when the outside half itself has passed the band. If a half
cannot be picked up, or the halves still do not pair, it lays down what it holds and stops (`give_up`). With
``read`` left as None (a kinematic dry run without a stand-in) it follows the nominal waypoints.

Targets are poses of a half in pane coordinates (u along the wall, n square to it and positive indoors, z up;
see cleaner_model.py); `hold` turns one into the gripper pose that holds the half there.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

from . import cleaner_model as model
from .fixed_scene import ARMS, BASES, CONTROL_HZ

LEFT, RIGHT = ARMS
HALF = {LEFT: "inside", RIGHT: "outside"}
OPEN = float(FINGER_OPEN)


def _grip(approach, fingers):
    approach, fingers = np.asarray(approach, dtype=float), np.asarray(fingers, dtype=float)
    return np.column_stack((fingers, np.cross(approach, fingers), approach))


def _about_z(degrees):
    return Rotation.from_euler("z", degrees, degrees=True).as_matrix()


# -- how each gripper holds its half -------------------------------------------------------------
# In the half's own frame (x along the handle bar, y towards the apex, z out of the back). The fingers close
# across the bar's 24 mm width. The left hand points at the back of its half, LEFT_PITCH from the bar. The
# right hand lies along the bar, tipped 10 degrees towards the pad, and holds the bar 25 mm from its middle,
# towards the end it comes from, so that the hand's wider parts stay clear of the half's shell. On the glass
# that puts the right hand along the outdoor face, which is the only way the wrist can hold a half out
# there: pointing the hand more squarely at the glass would take joint 5 past its limit.
LEFT_PITCH = np.radians(90.)
RIGHT_PITCH = np.radians(10.)
GRIP_IN_HALF = {LEFT: _grip((np.cos(LEFT_PITCH), 0., -np.sin(LEFT_PITCH)), (0., 1., 0.)),
                RIGHT: _grip((np.cos(RIGHT_PITCH), 0., -np.sin(RIGHT_PITCH)), (0., -1., 0.))}
GRIP_OFFSET = 0.025
GRIP_LOCAL = {LEFT: model.GRIP_POINT.copy(), RIGHT: model.GRIP_POINT + (-GRIP_OFFSET, 0., 0.)}
GRIP_Z = float(model.GRIP_POINT[2])


def hold(arm, origin, rot):
    """Gripper centre point and rotation (tabletop frame) that hold the arm's half with its pad centre at
    ``origin`` and its rotation ``rot``, both in pane coordinates."""
    origin, rot = np.asarray(origin, dtype=float), np.asarray(rot, dtype=float)
    return model.to_world(origin + rot @ GRIP_LOCAL[arm]), model.PANE_ROT @ rot @ GRIP_IN_HALF[arm]


# -- fixed targets (m), pane coordinates ----------------------------------------------------------
START_ROT = {half: model.rotation(model.START_QUAT_PANE[half]) for half in model.HALVES}
START_ORIGIN = {half: np.array((*model.START_UN[half], 0.)) for half in model.HALVES}
ON_GLASS = {half: model.on_glass_rotation(half) for half in model.HALVES}
# The outside half goes out through the open light with its handle bar, and the hand along it, turned 40
# degrees outwards from the wall's line (the wrist cannot lie along the wall that far to the right); it
# is swung round to lie along the glass once it is outdoors.
PASS_YAW_DEG = -40.
PASS_ROT = _about_z(PASS_YAW_DEG) @ ON_GLASS["outside"]
LIFT = {LEFT: 0.100, RIGHT: 0.140}
MATE_U = 0.170  # where along the glass the two halves are brought together (pad centre)
END_U = model.BAND_U[1] - 0.040  # where the stroke ends: the pad's 100 mm wide part is then past the band
STROKE_STEP = 0.021
PRESS = 0.002  # the seated targets aim the pad this far into the glass, so the pad stays flat on it
STANDOFF = {LEFT: 0.045, RIGHT: 0.025}  # pad face to glass while a half waits in front of it (the right arm is
# near the end of its reach out there, so its half waits closer)
SLIDE_ON = 0.055  # the outside half is set on the glass this far below the inside half, then slid up (the
# right arm, near the end of its reach out there, reaches lower more easily than higher)
SET_ON_Z = model.WORK_Z - SLIDE_ON
SLIDE_STEP = 0.011
STAGE = np.array((-0.150, 0.130, 0.270))  # outside half's pad centre: indoors, turned up, facing the opening
OUTDOORS = np.array((-0.060, -0.120, 0.270))  # the same, carried out through the opening
# Half-way round the swing: off the stile, 55 mm from the outdoor face, at the height it will be set on at.
SWING = np.array(((OUTDOORS[0] + MATE_U) / 2, -(model.PANE_T / 2 + 0.055), SET_ON_Z))
SWING_ROT = _about_z(PASS_YAW_DEG / 2) @ ON_GLASS["outside"]
BACK_OFF = 0.060  # how far each open gripper draws back along its own axis after letting go
LAG_WAIT = 0.009  # the stroke waits when the outside half is this far behind the inside half
PAIRED_WITHIN = .0015  # the slide-on stops when the two halves' centres are this close, seen through the glass


def glass_origin(half, u, gap, z=model.WORK_Z):
    """Pad centre, in pane coordinates, of a half whose pad is ``gap`` from its face of the glass at (u, z)."""
    return np.array((u, model.SIDE[half] * (model.PANE_T / 2 + gap), z))


def glass_point_pane(arm, u, gap, z=model.WORK_Z):
    """Gripper centre point, in pane coordinates, that holds the arm's half there."""
    return glass_origin(HALF[arm], u, gap, z) + ON_GLASS[HALF[arm]] @ GRIP_LOCAL[arm]


def glass_point(arm, u, gap, z=model.WORK_Z):
    """The same point in tabletop coordinates."""
    return model.to_world(glass_point_pane(arm, u, gap, z))


TABLE_GRIP = {arm: hold(arm, START_ORIGIN[HALF[arm]], START_ROT[HALF[arm]])[1] for arm in ARMS}
GLASS_GRIP = {arm: hold(arm, np.zeros(3), ON_GLASS[HALF[arm]])[1] for arm in ARMS}
PASS_GRIP = hold(RIGHT, np.zeros(3), PASS_ROT)[1]
PICK = {arm: hold(arm, START_ORIGIN[HALF[arm]], START_ROT[HALF[arm]])[0] for arm in ARMS}
WAIT_LEFT = glass_point(LEFT, MATE_U, STANDOFF[LEFT])
STAGE_TCP = hold(RIGHT, STAGE, PASS_ROT)[0]
OUTDOORS_TCP = hold(RIGHT, OUTDOORS, PASS_ROT)[0]
SWING_TCP, SWING_GRIP = hold(RIGHT, SWING, SWING_ROT)
# Each arm ends resting above where its half lay.
REST = {arm: PICK[arm] - .080 * TABLE_GRIP[arm][:, 2] + (0., 0., LIFT[arm] - .060) for arm in ARMS}

# Finger targets (m, each finger; 0 is closed). Each finger is a 5000 N/m position drive, so a target 7 mm
# inside the bar's surface squeezes with about 35 N, under the gripper's 50 N limit.
SQUEEZE = .007
BAR_FINGER = -(model.BAR_SIZE[1] / 2 - SQUEEZE)
# A gripper that closed on nothing reaches its target; one that holds the bar stops this far short of it.
HOLDING_MARGIN = .003 / abs(OPEN)
ALONG_GLASS = (True, False, True)  # correct u and z from the arm's readings; never push harder along n


class WindowReference:
    """Emit one absolute 14-D joint action per control step. ``measured`` is the latest joint observation;
    ``read``, if given, returns {"inside": (origin, rotation), "outside": (origin, rotation), "paired": bool}
    in pane coordinates."""

    def __init__(self, read=None):
        self.read = read
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.tcp, self.rotation = {}, {}
        for arm in ARMS:
            self.tcp[arm], self.rotation[arm] = ik.tcp_pose(self.joints[arm], BASES[arm])
        self.aperture = dict.fromkeys(ARMS, OPEN)
        self.worst_ik = 0.
        self.report = {}
        self.steps = 0
        self.phases = []  # (name, first control step of the phase)

    def phase(self, name):
        self.phases.append((name, self.steps))

    # -- readings --------------------------------------------------------------------------
    def actual_tcp(self, arm):
        start = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.measured[start:start + 6], BASES[arm])[0]

    def opening(self, arm):
        """Measured gripper opening, 0 (closed) to 1 (open)."""
        return float(self.measured[7 * ARMS.index(arm) + 6])

    def note_grip(self, arm):
        commanded = BAR_FINGER / OPEN
        self.report[f"{arm}_opening_after_close"] = round(self.opening(arm), 4)
        self.report[f"{arm}_holding"] = bool(self.opening(arm) > commanded + HOLDING_MARGIN)

    def halves(self):
        """Pad centres of the inside and outside half in pane coordinates, and whether they are paired; None
        without a simulator."""
        if self.read is None:
            return None
        state = self.read()
        return np.asarray(state["inside"][0], dtype=float), np.asarray(state["outside"][0], dtype=float), bool(state["paired"])

    def lying(self, half):
        """Where a half lies on the table (pad centre, rotation), from the simulator if it can be read. A half
        that is not lying flat within 60 mm of its start is taken to be at its start."""
        if self.read is not None:
            origin, rot = (np.asarray(v, dtype=float) for v in self.read()[half])
            flat = rot[2, 2] > .995 and abs(origin[2]) < .004
            if flat and np.linalg.norm((origin - START_ORIGIN[half])[:2]) < .060:
                return np.array((origin[0], origin[1], 0.)), _about_z(np.degrees(np.arctan2(rot[1, 0], rot[0, 0])))
        return START_ORIGIN[half], START_ROT[half]

    def in_hand(self, arm, origin):
        """Whether a half's pad centre is where the arm's commanded gripper would be holding it (within 40 mm)."""
        expected = model.to_pane(self.tcp[arm]) - model.PANE_ROT.T @ self.rotation[arm] @ GRIP_IN_HALF[arm].T @ GRIP_LOCAL[arm]
        return bool(np.linalg.norm(np.asarray(origin) - expected) < .040)

    def lifted(self, half):
        """True unless the simulator shows the half still down on the table."""
        return self.read is None or float(self.read()[half][0][2]) > .040

    # -- motion ----------------------------------------------------------------------------
    def move(self, seconds, targets=None, *, orientation=None, jaws=None):
        targets, orientation, jaws = targets or {}, orientation or {}, jaws or {}
        arms = targets.keys() | orientation.keys() | jaws.keys()
        start = {a: (self.tcp[a], ik.matrix_to_quat_xyzw(self.rotation[a]), self.aperture[a]) for a in arms}
        end = {}
        for arm, (point, quat, aperture) in start.items():
            q = ik.matrix_to_quat_xyzw(orientation[arm]) if arm in orientation else quat
            end[arm] = (np.asarray(targets.get(arm, point), dtype=float), q if quat @ q >= 0 else -q,
                        jaws.get(arm, aperture))
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

    def track(self, targets, limit, *, free=(True, True, True), seconds=.35, repeats=2):
        """Short moves that shift each arm's command by its measured error, at most ``limit`` per move.

        ``targets`` are in tabletop coordinates; ``free`` is per pane axis (u, n, z). Along an axis that is not
        free the arm may be pressing on the glass, so there the command simply goes to the target and the
        measured error is ignored."""
        free = np.asarray(free)
        for _ in range(repeats):
            goal = {}
            for arm, target in targets.items():
                target = model.to_pane(target)
                delta = np.clip(target - model.to_pane(self.actual_tcp(arm)), -limit, limit)
                goal[arm] = model.to_world(np.where(free, model.to_pane(self.tcp[arm]) + delta, target))
            yield from self.move(seconds, goal)

    def go(self, arm, target, seconds, *, orientation=None, limit=.020, repeats=2):
        """Move one arm to a point in tabletop coordinates, then correct its tracking error."""
        yield from self.move(seconds, {arm: target}, orientation={arm: orientation} if orientation is not None else None)
        yield from self.track({arm: target}, limit, repeats=repeats)

    # -- the demonstration -------------------------------------------------------------------
    def grasp(self, arms, seconds=2.4):
        """Reach for each arm's half where it lies and close on its handle bar. Returns the grasp points."""
        pose = {arm: hold(arm, *self.lying(HALF[arm])) for arm in arms}
        point = {arm: pose[arm][0] for arm in arms}
        back = {arm: pose[arm][1][:, 2] for arm in arms}  # each gripper comes in along its own axis
        yield from self.move(seconds, {arm: point[arm] - .080 * back[arm] for arm in arms},
                             orientation={arm: pose[arm][1] for arm in arms}, jaws=dict.fromkeys(arms, OPEN))
        yield from self.move(.3)
        for distance in (.050, .025, .010, 0.):
            yield from self.track({arm: point[arm] - distance * back[arm] for arm in arms}, .030)
        if self.phases[-1][0] == "reach":
            self.phase("grasp")
        yield from self.move(.6, jaws=dict.fromkeys(arms, BAR_FINGER))
        yield from self.move(.3)
        for arm in arms:
            self.note_grip(arm)
        return point

    def pick_up_both(self):
        self.phase("reach")
        point = yield from self.grasp(ARMS)
        self.phase("lift")
        yield from self.move(1.0, {arm: point[arm] + (0., 0., LIFT[arm]) for arm in ARMS})
        yield from self.move(.2)
        for arm in ARMS:  # a half that stayed on the table is grasped once more, where it now lies
            if not self.lifted(HALF[arm]):
                self.report[f"{arm}_grasped_again"] = True
                yield from self.move(.4, jaws={arm: OPEN})
                again = yield from self.grasp((arm,), seconds=1.2)
                yield from self.move(1.0, {arm: again[arm] + (0., 0., LIFT[arm])})
                yield from self.move(.2)
            self.report[f"{arm}_lifted"] = self.lifted(HALF[arm])

    def face_the_glass(self):
        """Turn both halves up about their handle bars. The inside half goes to its standoff, 45 mm from the
        indoor face; the outside half waits indoors in front of the open light."""
        self.phase("turn")
        goal = {LEFT: WAIT_LEFT, RIGHT: STAGE_TCP}
        grip = {LEFT: GLASS_GRIP[LEFT], RIGHT: PASS_GRIP}
        middle = {arm: (self.tcp[arm] + goal[arm]) / 2 + (0., 0., .03 if arm == LEFT else 0.) for arm in ARMS}
        halfway = {arm: Rotation.from_rotvec(Rotation.from_matrix(grip[arm] @ self.rotation[arm].T).as_rotvec() / 2).as_matrix()
                   @ self.rotation[arm] for arm in ARMS}
        yield from self.move(1.6, middle, orientation=halfway)
        yield from self.move(2.0, goal, orientation=grip)
        yield from self.move(.4)
        yield from self.track(goal, .020, repeats=3)

    def set_on_glass(self, arm, z, seconds):
        """Bring one half straight up to the glass and press its pad lightly on it. Square to the glass the
        command is not corrected by the measured arm, so the press never builds up."""
        for gap in (.025, .010)[arm == RIGHT:]:
            yield from self.track({arm: glass_point(arm, MATE_U, gap, z)}, .030, free=ALONG_GLASS, repeats=1)
        yield from self.move(seconds, {arm: glass_point(arm, MATE_U, -PRESS, z)})
        yield from self.move(.5)

    def out_through_the_opening(self):
        """Carry the outside half straight out through the open light, then swing it round to lie along the
        outdoor face, past the stile, at its standoff 55 mm below the inside half."""
        self.phase("through_opening")
        yield from self.go(RIGHT, OUTDOORS_TCP, 2.4)
        self.phase("along_outside")
        yield from self.move(1.8, {RIGHT: SWING_TCP}, orientation={RIGHT: SWING_GRIP})
        yield from self.go(RIGHT, glass_point(RIGHT, MATE_U, STANDOFF[RIGHT], SET_ON_Z), 1.8,
                           orientation=GLASS_GRIP[RIGHT], repeats=3)

    def slide_up(self):
        """Slide the outside half up the glass until it is opposite the inside half. With the simulator's
        poses to hand, each step aims the outside half's centre at the inside half's."""
        aim = model.to_pane(self.tcp[RIGHT])
        first = aim.copy()
        for _ in range(14):
            state = self.halves()
            if state is None:
                error = np.array((0., 0., glass_point_pane(RIGHT, MATE_U, 0.)[2] - aim[2]))  # nominal: straight up
            else:
                inside, outside, paired = state
                error = inside - outside
                if paired and np.hypot(error[0], error[2]) < PAIRED_WITHIN:
                    break
                if not self.in_hand(RIGHT, outside):
                    self.report["outside_half_not_in_hand"] = True
                    break
            if abs(error[0]) < 1e-4 and abs(error[2]) < 1e-4:
                break
            aim = aim + (np.clip(error[0], -.004, .004), 0., np.clip(error[2], -SLIDE_STEP, SLIDE_STEP))
            # The corrections stay within 20 mm along the glass and 15 mm past the planned slide.
            aim[0] = np.clip(aim[0], first[0] - .020, first[0] + .020)
            aim[2] = np.clip(aim[2], first[2] - .015, first[2] + SLIDE_ON + .015)
            yield from self.track({RIGHT: model.to_world(aim)}, .020, free=ALONG_GLASS, seconds=.6, repeats=1)
        yield from self.move(.5)

    def slide_on(self):
        """The magnets take hold over the last 25 mm of the slide; the half is on the glass the whole way, so
        nothing snaps onto the pane. If the halves have not paired, the outside half is slid back down the glass
        and up again, once."""
        yield from self.slide_up()
        state = self.halves()
        if state is not None and not state[2]:
            self.report["slid_on_again"] = True
            down = model.to_pane(self.tcp[RIGHT]) - (0., 0., SLIDE_ON)
            yield from self.track({RIGHT: model.to_world(down)}, .030, free=ALONG_GLASS, seconds=1.2, repeats=2)
            yield from self.slide_up()
            state = self.halves()
        if state is not None:
            self.report["offset_before_release_mm"] = [round(1000 * float(v), 1) for v in (state[0] - state[1])[[0, 2]]]
            self.report["paired_before_release"] = state[2]
        return state is None or state[2]

    def come_back_in(self):
        """Bring the right arm back in through the open light the way it went out, to above where its half lay."""
        yield from self.move(1.6, {RIGHT: SWING_TCP}, orientation={RIGHT: SWING_GRIP})
        yield from self.go(RIGHT, OUTDOORS_TCP, 1.6, orientation=PASS_GRIP, repeats=1)
        yield from self.go(RIGHT, STAGE_TCP, 2.2, repeats=1)
        yield from self.move(1.6, {RIGHT: PICK[RIGHT] + (0., 0., LIFT[RIGHT])}, orientation={RIGHT: TABLE_GRIP[RIGHT]})

    def let_go_outside(self):
        """Open, draw back along the hand's own axis, and come back in."""
        yield from self.move(.35, jaws={RIGHT: OPEN})
        yield from self.move(.35)
        yield from self.move(1.0, {RIGHT: self.tcp[RIGHT] - self.rotation[RIGHT][:, 2] * BACK_OFF})
        state = self.halves()
        if state is not None:
            self.report["paired_after_release"] = state[2]
        yield from self.come_back_in()
        yield from self.move(1.0, {RIGHT: REST[RIGHT]})

    def lay_down(self, arms):
        """Put each arm's half back where it lay and let go of it."""
        yield from self.move(2.0, {arm: PICK[arm] + (0., 0., LIFT[arm]) for arm in arms}, orientation={arm: TABLE_GRIP[arm] for arm in arms})
        yield from self.move(1.2, {arm: PICK[arm] + (0., 0., .003) for arm in arms})
        yield from self.move(.4, jaws=dict.fromkeys(arms, OPEN))
        yield from self.move(1.0, {arm: REST[arm] for arm in arms})

    def give_up(self, why, held=()):
        """Stop tidily: lay down whatever is still held, open both grippers and rest. The score stays as it is."""
        self.report["gave_up"] = why
        self.phase("gave_up")
        if RIGHT in held:  # the outside half is still in the right gripper, out on the glass: bring it back in
            yield from self.move(1.0, {RIGHT: glass_point(RIGHT, MATE_U, STANDOFF[RIGHT], SET_ON_Z)})
            yield from self.come_back_in()
        if LEFT in held and any(name == "seat_inside" for name, _ in self.phases):  # take the inside half off the glass
            yield from self.move(1.0, {LEFT: WAIT_LEFT})
        if held:
            yield from self.lay_down(tuple(held))
        yield from self.move(.4, jaws=dict.fromkeys(ARMS, OPEN))
        yield from self.move(1.5, REST, orientation=TABLE_GRIP)
        while True:
            yield from self.move(1.)

    def stroke(self):
        """Slide the inside half along the glass. Only u and z follow the measured arm; n keeps pressing. The
        stroke keeps the height at which the pair actually sits (within 6 mm of the planned height), waits
        when the outside half falls behind, and ends when the outside half itself is at END_U."""
        state = self.halves()
        z = model.WORK_Z if state is None else float(np.clip(state[0][2], model.WORK_Z - .006, model.WORK_Z + .006))
        self.report["stroke_height_mm"] = round(1000 * z, 1)
        waits = 0
        for u in np.arange(MATE_U + STROKE_STEP, END_U + 1e-9, STROKE_STEP):
            yield from self.track({LEFT: glass_point(LEFT, u, -PRESS, z)}, .030, free=ALONG_GLASS)
            state = self.halves()
            if state is not None and state[0][0] - state[1][0] > LAG_WAIT and waits < 8:
                waits += 1
                yield from self.move(.5)
        self.report["stroke_waits"] = waits
        for _ in range(8):  # carry on in small steps until the outside half itself has arrived
            state = self.halves()
            behind = END_U - (model.to_pane(self.actual_tcp(LEFT))[0] if state is None else state[1][0])
            if behind < .002:
                break
            aim = model.to_pane(self.tcp[LEFT]) + (min(behind, .010), 0., 0.)
            yield from self.track({LEFT: model.to_world(aim)}, .010, free=ALONG_GLASS, repeats=1)
        self.report["left_u_after_stroke_mm"] = round(1000 * float(model.to_pane(self.actual_tcp(LEFT))[0]), 1)
        state = self.halves()
        if state is not None:
            self.report["outside_u_after_stroke_mm"] = round(1000 * float(state[1][0]), 1)
            self.report["paired_after_stroke"] = state[2]
        yield from self.move(.5)

    def let_go_inside(self):
        yield from self.move(.35, jaws={LEFT: OPEN})
        yield from self.move(.35)
        yield from self.move(1.0, {LEFT: self.tcp[LEFT] - self.rotation[LEFT][:, 2] * (BACK_OFF + .030)})
        yield from self.move(2.6, {LEFT: REST[LEFT]}, orientation={LEFT: TABLE_GRIP[LEFT]})

    def sequence(self):
        yield from self.pick_up_both()
        missing = [arm for arm in ARMS if not self.report[f"{arm}_lifted"]]
        if missing:
            yield from self.give_up(f"could not pick up the {HALF[missing[0]]} half", [arm for arm in ARMS if arm not in missing])
        yield from self.face_the_glass()
        self.phase("seat_inside")
        yield from self.set_on_glass(LEFT, model.WORK_Z, .8)
        yield from self.out_through_the_opening()
        self.phase("set_outside_on_glass")
        yield from self.set_on_glass(RIGHT, SET_ON_Z, 1.0)
        self.phase("slide_outside_on")
        paired = yield from self.slide_on()
        if not paired:  # letting go now would drop the outside half outdoors, out of reach
            yield from self.give_up("the halves did not pair", ARMS)
        self.report["right_opening_before_release"] = round(self.opening(RIGHT), 4)
        self.phase("release_outside")
        yield from self.let_go_outside()
        self.phase("stroke")
        yield from self.stroke()
        self.report["left_opening_before_release"] = round(self.opening(LEFT), 4)
        self.phase("release_inside")
        yield from self.let_go_inside()
        self.phase("done")
        while True:
            yield from self.move(1.)
