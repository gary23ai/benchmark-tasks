"""Full-score demonstration: one hand pulls the drawer to the catch and holds it, the other reaches into the
gap and presses the latch arm down, the drawer is pulled past the catch and then opened the rest of the way.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a
scripted demonstration, simulator state: how far the drawer is open and how far the latch arm is pressed
down. It changes nothing in the simulation. The drawer moves only because the right hand's pads hold its
moulded pull, and the latch goes down only where the left fingertips press it.

Plan.
1. The right hand, jaws part open and pointing forward (10 degrees below level, jaws closing up and down), takes the
   pull by its lip and closes on it. The left hand waits above, fingers closed.
2. The right hand pulls until the drawer stops on the catch (38 mm), then eases it back 1.5 mm so the hook
   no longer bears on the catch.
3. The left hand, fingers closed and jaws closing front to back so that its two fingertips together are
   about 13 mm thick along the drawer, tipped 20 degrees forward and 25 degrees to the side so that the
   fingers rise away from the frame's top, comes down into the 30 mm gap between the latch's base and the
   frame and presses the latch arm 21 mm behind its pin until the arm is 2 degrees past the angle that frees
   the hook.
4. Still pressing, the right hand pulls the drawer past the catch to 66 mm; the left hand then lifts out.
5. The right hand opens the drawer to 165 mm, lets go and backs off; both hands park clear and wait.

``variant`` gives two controls that must not get past the catch: ``"no_press"`` only pulls;
``"early_release"`` presses the latch but lifts the hand before pulling.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY

from . import layout
from .fixed_scene import ARMS, BASES, CONTROL_HZ

PRESS, PULL = ARMS                      # the left hand presses the latch, the right hand pulls the drawer
OPEN = float(FINGER_OPEN)
TIP = float(PAD_TIP_PAST_TCP)           # the fingertips end this far beyond the gripper's centre point
DT = 1. / CONTROL_HZ

PULL_PITCH = np.radians(80.)            # the pulling hand points forward, 10 degrees below level
PULL_Y = -.050                          # where on the pull's lip it takes hold (the lip spans y -60 to 60 mm)
GRIP_BACK = .0025                       # fingertips this far short of the drawer front: 9.5 mm of the 12 mm lip in the jaws
PRE_GRASP = .060                        # first this far in front of that
PRE_JAW = .30                           # jaws part open on the way in (22 mm between the pads; the lip is 8 mm)
PULL_SPEED = .040                       # m/s of the command
PULL_LEAD = .016                        # the command may run this far ahead of the drawer (the arm's springiness pulls)
STALL_LEAD = .012
STALL_STEPS = 14
EASE = .0015                            # push the drawer back this far off the catch before pressing
SERVO_GAIN = .3                         # share of the measured fingertip error added to the trim each step
TRIM_MAX = .030
HOLDING = .04                           # gripper reading (0 closed, 1 open) above which the jaws are on the lip

PRESS_FORWARD = np.radians(20.)         # the pressing hand points this far forward (+x) from straight down ...
PRESS_SIDE = np.radians(25.)            # ... and this far toward -y: its fingers rise away from the frame's top
PRESS_Y = layout.LATCH_Y
PRESS_A = .021                          # press the arm this far behind its pin
FRAME_MARGIN = .009                     # keep the fingertips' middle this far from the frame's front face
PRESS_HOVER_Z = .200                    # fingertips' height above the gap before going in
PRESS_TOUCH_Z = layout.ARM_TOP_Z + .006
PRESS_SPEED = .015                      # m/s while pressing
PRESS_FLOOR = layout.ARM_TOP_Z - .016   # never command the fingertips deeper than this
PRESS_PAST = np.radians(2.)             # press this far past the angle that frees the hook
PAST_OPEN = .066                        # pull past the catch to here before lifting the pressing hand
OPEN_TO = .165                          # then open the drawer to here
WAIT_HIGH = (.300, .060, .250)          # the pressing hand waits here while the other pulls
APPROACH = (.260, PULL_Y, .190)         # the pulling hand's way in
BACK_OFF = (-.040, 0., .020)            # after letting go, the pulling hand first moves this far
PARK = {PRESS: WAIT_HIGH, PULL: (.220, -.150, .240)}       # fingertips, at the end
# Joint angles near the two working poses (from the reach study); they seed the inverse kinematics.
SEED = {PRESS: (-.7, 1.75, 1.5, -.8, -.6, -.6), PULL: (.93, 1.8, .71, .8, .91, -1.34)}


def frame(approach, closing):
    """Gripper orientation from its approach axis (fingers point along it) and closing axis."""
    a = np.asarray(approach, dtype=float)
    a /= np.linalg.norm(a)
    c = np.asarray(closing, dtype=float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


ROTATION = {
    # tipped forward and toward -y, jaws closing front to back: the closed fingertips are thin along x
    PRESS: frame((np.tan(PRESS_FORWARD), -np.tan(PRESS_SIDE), -1.), (-1., 0., 0.)),
    # pointing forward and a little down, jaws closing nearly up and down across the pull's lip
    PULL: frame((np.sin(PULL_PITCH), 0., -np.cos(PULL_PITCH)), (np.cos(PULL_PITCH), 0., np.sin(PULL_PITCH))),
}
QUAT = {arm: ik.matrix_to_quat_xyzw(ROTATION[arm]) for arm in ARMS}


def fingertips(joint_pos, arm):
    """Middle of the two fingertips' ends, from joint readings (tabletop frame)."""
    start = 7 * ARMS.index(arm)
    point, rotation = ik.tcp_pose(np.asarray(joint_pos, dtype=float)[start:start + 6], BASES[arm])
    return point + TIP * rotation[:, 2]


def press_x(open_m):
    """Tabletop x of the pressing point on the latch arm for a drawer open ``open_m``."""
    return min(layout.drawer_x(open_m) + layout.HINGE_X + PRESS_A, layout.FRAME_FRONT_X - FRAME_MARGIN)


class LatchReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant="full"):
        if variant not in ("full", "no_press", "early_release"):
            raise ValueError(variant)
        self.variant = variant
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.aperture = {PRESS: OPEN, PULL: OPEN}
        self.tip = {}                                   # commanded fingertip points
        self.trim = {arm: np.zeros(3) for arm in ARMS}  # added to a command so the measured fingertips land on it
        self.still = 0
        self.last_open = None
        self.worst_ik = 0.
        self.tick = 0
        self.phases = []
        self.log = {"variant": variant, "press_tries": []}
        self.report = self.log

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        if state is not None:
            self.state = state

    def measured_tip(self, arm):
        return fingertips(self.measured, arm)

    def opening(self, arm):
        return float(self.measured[7 * ARMS.index(arm) + 6])

    def drawer_open(self):
        return float(self.state["open"])

    def angle(self):
        return float(self.state["latch_angle"])

    def front_x(self):
        return layout.drawer_x(self.drawer_open())

    # -- motion ---------------------------------------------------------------------------
    def solve(self, arm, point, seed=None, iters=40):
        target = np.asarray(point, dtype=float) + self.trim[arm] - TIP * ROTATION[arm][:, 2]
        joints, error, _ = ik.solve(target, QUAT[arm], self.joints[arm] if seed is None else seed, BASES[arm],
                                    iters=iters)
        self.worst_ik = max(self.worst_ik, error)
        return joints

    def emit(self):
        for arm, point in self.tip.items():
            self.joints[arm] = self.solve(arm, point)
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def glide(self, seconds, targets, aperture=None):
        """Move in joint space to the given fingertip points (smooth start and stop)."""
        aperture = aperture or {}
        goal = {arm: self.solve(arm, point, seed=np.asarray(SEED[arm]), iters=400) for arm, point in targets.items()}
        start = {arm: self.joints[arm].copy() for arm in targets}
        jaws = {arm: self.aperture[arm] for arm in aperture}
        self.tip = {arm: point for arm, point in self.tip.items() if arm not in targets}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm in targets:
                self.joints[arm] = start[arm] + alpha * (goal[arm] - start[arm])
            for arm, value in aperture.items():
                self.aperture[arm] = jaws[arm] + alpha * (value - jaws[arm])
            yield self.emit()
        for arm, point in targets.items():
            self.tip[arm] = np.asarray(point, dtype=float).copy()

    def line(self, seconds, targets, aperture=None):
        """Move fingertips in straight lines (smooth start and stop), optionally moving the jaws too."""
        aperture = aperture or {}
        start = {arm: self.tip[arm].copy() for arm in targets}
        jaws = {arm: self.aperture[arm] for arm in aperture}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm, goal in targets.items():
                self.tip[arm] = start[arm] + alpha * (np.asarray(goal, dtype=float) - start[arm])
            for arm, value in aperture.items():
                self.aperture[arm] = jaws[arm] + alpha * (value - jaws[arm])
            yield self.emit()

    def run(self, seconds, *, pull_to=None, press=False, done=None, settle=3):
        """Servo for up to ``seconds``: move the drawer toward an opening and/or keep the latch pressed. Stops
        early once ``done()`` has held for ``settle`` steps. Returns True if it did."""
        count = 0
        self.still = 0
        self.last_open = None
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            if pull_to is not None:
                self._pull_step(pull_to)
            if press:
                self._press_step()
            yield self.emit()
            count = count + 1 if done is not None and done() else 0
            if count >= settle:
                return True
        return False

    def _pull_step(self, target_open):
        """Run the pulling hand's command toward the opening wanted, at most PULL_LEAD ahead of the drawer."""
        held = self.front_x() - GRIP_BACK               # where the fingertips sit on the lip
        tip = self.tip[PULL]
        moved = self.last_open is None or abs(self.drawer_open() - self.last_open) > .0002
        self.last_open = self.drawer_open()
        error = target_open - self.drawer_open()
        goal = layout.drawer_x(target_open) - GRIP_BACK
        step = PULL_SPEED * DT
        if error > .0005:          # outward: lead the drawer
            tip[0] = max(tip[0] - step, goal - .5 * PULL_LEAD, held - PULL_LEAD)
        elif error < -.0005:       # inward
            tip[0] = min(tip[0] + step, goal + .5 * PULL_LEAD, held + PULL_LEAD)
        else:                      # there: take the force off
            tip[0] += float(np.clip(held - tip[0], -step, step))
        leading = held - tip[0] if error > 0 else tip[0] - held
        self.still = 0 if moved or leading < STALL_LEAD else self.still + 1
        tip[1], tip[2] = PULL_Y, layout.PULL_Z

    def servo(self, arm, goal, seconds=2., tol=.0015, settle=3):
        """Hold the command at ``goal`` and adjust the arm's trim until its measured fingertips (joint readings
        through the arm's kinematics) are within ``tol`` of it: the arm sags 10 to 20 mm under its own weight."""
        goal = np.asarray(goal, dtype=float)
        self.tip[arm] = goal.copy()
        count = 0
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            error = goal - self.measured_tip(arm)
            count = count + 1 if np.linalg.norm(error) < tol else 0
            if count >= settle:
                return True
            self.trim[arm] = np.clip(self.trim[arm] + SERVO_GAIN * error, -TRIM_MAX, TRIM_MAX)
            yield self.emit()
        return False

    def _stalled(self):
        return self.still >= STALL_STEPS

    def _press_step(self):
        """Press the latch arm down until it is PRESS_PAST beyond the angle that frees the hook, keep it there."""
        tip = self.tip[PRESS]
        if self.angle() < layout.RELEASE_ANGLE + PRESS_PAST:
            tip[2] = max(tip[2] - PRESS_SPEED * DT, PRESS_FLOOR)
        elif self.angle() > layout.RELEASE_ANGLE + 2. * PRESS_PAST:
            tip[2] += .0003

    def phase(self, name):
        self.phases.append((self.tick, name))

    def snapshot(self, **extra):
        """What the demonstration saw at this moment, for the run log (millimetres, degrees)."""
        row = {"tick": self.tick, "open_mm": round(1000 * self.drawer_open(), 1),
               "latch_angle_deg": round(float(np.degrees(self.angle())), 2)}
        for arm in ARMS:
            if arm in self.tip:
                row[f"{arm}_tip_command_mm"] = [round(1000 * float(v), 1) for v in self.tip[arm]]
            row[f"{arm}_tip_measured_mm"] = [round(1000 * float(v), 1) for v in self.measured_tip(arm)]
        row["gripper_opening"] = [round(self.opening(arm), 4) for arm in ARMS]
        row.update(extra)
        return row

    # -- the demonstration ------------------------------------------------------------------
    def press(self):
        """Bring the left fingertips into the gap and press the latch arm down. Returns True once it is down."""
        x = press_x(self.drawer_open())
        yield from self.glide(1.6, {PRESS: (x, PRESS_Y, PRESS_HOVER_Z)}, {PRESS: 0.})
        aimed = yield from self.servo(PRESS, (x, PRESS_Y, PRESS_HOVER_Z))
        yield from self.line(.8, {PRESS: (x, PRESS_Y, PRESS_TOUCH_Z + .010)})
        aimed = (yield from self.servo(PRESS, (x, PRESS_Y, PRESS_TOUCH_Z + .010), seconds=1.)) and aimed
        yield from self.line(.4, {PRESS: (x, PRESS_Y, PRESS_TOUCH_Z)})
        self.log["press_aim"] = self.snapshot(aimed=bool(aimed), trim_mm=[round(1000 * float(v), 1) for v in self.trim[PRESS]])
        pressed = yield from self.run(5., press=True,
                                      done=lambda: self.angle() >= layout.RELEASE_ANGLE + .8 * PRESS_PAST)
        return pressed

    def take_pull(self, shift_z=0.):
        """Bring the right jaws round the pull's lip and close them. Returns True if the reading says they hold it."""
        front = self.front_x()
        z = layout.PULL_Z + shift_z
        self.aperture[PULL] = PRE_JAW * OPEN
        yield from self.line(1., {PULL: (front - GRIP_BACK - PRE_GRASP, PULL_Y, z)})
        yield from self.servo(PULL, (front - GRIP_BACK - PRE_GRASP, PULL_Y, z))
        yield from self.line(1.2, {PULL: (front - GRIP_BACK, PULL_Y, z)})
        yield from self.servo(PULL, (front - GRIP_BACK, PULL_Y, z), seconds=1., tol=.002)
        yield from self.line(1., {}, {PULL: 0.})
        yield from self.run(.4)
        return self.opening(PULL) > HOLDING

    def lift(self):
        x = float(self.tip[PRESS][0])
        yield from self.line(.8, {PRESS: (x, PRESS_Y, PRESS_HOVER_Z)})

    def sequence(self):
        self.phase("reach")
        front = layout.drawer_x(layout.OPEN_START)
        # The pulling hand first comes in high and in front of the unit, then down in front of the pull.
        yield from self.glide(2.5, {PRESS: WAIT_HIGH, PULL: APPROACH}, {PRESS: 0., PULL: PRE_JAW * OPEN})
        self.phase("take the pull")
        held = yield from self.take_pull()
        self.log["grasped"] = self.snapshot(holding=bool(held))
        for retry in (.004, -.004):
            if held:
                break
            self.phase("take the pull again")
            yield from self.line(.6, {}, {PULL: PRE_JAW * OPEN})
            held = yield from self.take_pull(shift_z=retry)
            self.log[f"grasped_again_{retry:+.3f}"] = self.snapshot(holding=bool(held))
        self.phase("pull to the catch")
        self.log["stopped"] = yield from self.run(6., pull_to=layout.catch_open() + .030,
                                                  done=lambda: self._stalled() or self.drawer_open() >= .060)
        self.log["at_catch"] = self.snapshot()
        caught = self.drawer_open()
        for attempt in range(3):
            if self.drawer_open() >= PAST_OPEN - .004 or self.variant == "no_press":
                break
            entry = {"attempt": attempt}
            self.phase(f"ease off the catch ({attempt})")
            yield from self.run(1., pull_to=caught - EASE * (1 + attempt),
                                done=lambda: self.drawer_open() <= caught - EASE * (1 + attempt) + .0007)
            self.phase(f"press the latch ({attempt})")
            entry["pressed"] = yield from self.press()
            entry["after_press"] = self.snapshot()
            if self.variant == "early_release":
                self.phase("lift before pulling")
                yield from self.lift()
            self.phase(f"pull past the catch ({attempt})")
            keep = self.variant == "full"
            entry["past"] = yield from self.run(4., pull_to=PAST_OPEN + .004, press=keep,
                                                done=lambda: self.drawer_open() >= PAST_OPEN or self._stalled())
            entry["after_pull"] = self.snapshot()
            self.log["press_tries"].append(entry)
            if keep or self.drawer_open() < PAST_OPEN:
                self.phase(f"lift out of the gap ({attempt})")
                yield from self.lift()
            if self.variant == "early_release":
                break
        self.phase("park the pressing hand")
        yield from self.glide(1.5, {PRESS: PARK[PRESS]})
        self.phase("open the drawer")
        self.log["opened"] = yield from self.run(8., pull_to=OPEN_TO,
                                                 done=lambda: self.drawer_open() >= OPEN_TO - .006 or self._stalled())
        yield from self.run(.4, pull_to=self.drawer_open())
        self.log["before_letting_go"] = self.snapshot()
        self.phase("let go")
        yield from self.line(.8, {}, {PULL: OPEN})
        tip = self.tip[PULL]
        yield from self.line(1., {PULL: tip + BACK_OFF})
        yield from self.glide(1.5, {PULL: PARK[PULL]})
        self.log["parked"] = self.snapshot()
        self.phase("wait")
        while True:
            yield self.emit()
