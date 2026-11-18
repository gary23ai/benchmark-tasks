"""Full-score demonstration.

The scene is the same in every trial (seed 0, no jitter), so the plan is a fixed sequence of waypoints from the
written setup. The demonstration is closed loop where that makes it robust. From the robot it reads the twelve
joint angles and the two gripper openings. From the simulator (through ``probe``, when one is given) it reads
the two handles' positions, the wire's tension and how much of the cake is cut, and uses them to aim each
grasp at where the handle really lies, to hold each handle at the height of the cut, to draw the wire just
taut, and to stop drawing once the wire is through. It only reads: it never moves an object, changes the
simulation or touches the score. Without a probe it runs from the robot's own readings alone.

Plan: each gripper picks up one handle by its near half, so that the wire leaves the handle beside the
fingers. The arms draw the wire straight, carry it over the cake, lower it behind the cake to the height
of the cut, lay it against the far face, swing round so that the wire wraps the two far corners, and
draw it toward the robot at 15 mm/s until it comes out of the near face. Then both handles are laid on the
table and the arms park. The grippers stay 45 mm or more from the cake throughout.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import grip_rotation

from . import cake_model as cake
from .fixed_scene import ARMS, BASES, CONTROL_HZ, PARK

LEFT, RIGHT = ARMS
SIDE = {LEFT: 1., RIGHT: -1.}
OPEN = float(FINGER_OPEN)
DOWN = grip_rotation()                       # pointing straight down, fingers closing along y
# While the wire is carried and drawn the grippers lean 35 degrees forward, which lets the arms reach the far
# side of the cake (straight down they reach x = 0.42 m at this height, leaning 0.62 m).
LEAN = Rotation.from_euler("y", -35., degrees=True).as_matrix() @ DOWN

# -- fixed targets, tabletop coordinates (m) -------------------------------------------------------------
GRIP_BACK = .020                             # the fingers take the handle this far toward the robot from its middle
GRASP_Z = .0155                              # gripper centre point above the table: pad tips 2.5 mm clear of it
HANDLE = {arm: cake.HANDLE_START[arm] for arm in ARMS}
GRASP = {arm: HANDLE[arm] + (-GRIP_BACK, 0., GRASP_Z - cake.HANDLE_RADIUS) for arm in ARMS}
# Where the wire leaves the held handle (the handle's middle), in the gripper's own frame.
WIRE_END = DOWN.T @ (GRIP_BACK, 0., cake.HANDLE_RADIUS - GRASP_Z)
HALF_WIRE = cake.WIRE_LENGTH / 2
TAUT_Y = HALF_WIRE + .0015                   # hands 3 mm further apart than the wire is long: about 4.5 N
CARRY_Z = cake.CAKE_CENTRE[2] + cake.CAKE_HALF[2] + .038   # the straight wire passes 38 mm above the cake
SEAM_Z = cake.SEAM_Z
FAR, NEAR = cake.CAKE_CENTRE[0] + cake.CAKE_HALF[0], cake.CAKE_CENTRE[0] - cake.CAKE_HALF[0]
BEHIND_X = FAR + .035
TOUCH_X = FAR - .004                         # the straight wire pressed 4 mm against the far face
# Swing round the far corners: each hand keeps 2 mm more than the free wire's length from its corner.
CORNER_Y = cake.CAKE_HALF[1]
SWING_RADIUS = HALF_WIRE - CORNER_Y + .002
SWING_DEG = 25.
DRAW_Y = CORNER_Y + SWING_RADIUS * np.cos(np.radians(SWING_DEG))
DRAW_START_X = FAR - SWING_RADIUS * np.sin(np.radians(SWING_DEG))
DRAW_END_X = NEAR - .100                     # the wire leaves the near face when the hands are about 50 mm past it
DRAW_SPEED = .015                            # m/s
LAY_Z = GRASP_Z + .002
# A gripper that closed on nothing reads 0; one that holds a 12 mm handle stops at about 0.13.
HOLDING_MARGIN = .05
SERVO_GAIN, SERVO_LIMIT = .15, .045          # share of the measured error added per step; largest correction, m
TAUT_N = (1., 3.5)                           # N; while the wire is held, the hands are eased together above this
TRIM_STEP, TRIM_MAX = .0002, .012            # m per control step, and at most, each hand toward the midline
CUT_DONE = .99                               # share of the cake's plan cut on the line at which drawing stops


class WireCutReference:
    """Emit one absolute 14-D joint action per control step. ``measured`` is the latest joint observation."""

    def __init__(self, probe=None):
        self.probe = probe                       # read-only view of the simulator, or None
        self.holding = False                     # the guided points are the handles' middles
        self.trim = 0.                           # how far each hand is eased toward the midline, m
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.point, self.rotation = {}, {}
        for arm in ARMS:
            self.point[arm], self.rotation[arm] = ik.tcp_pose(self.joints[arm], BASES[arm])
        self.offset = {arm: np.zeros(3) for arm in ARMS}     # the guided point, in the gripper's frame
        self.bias = {arm: np.zeros(3) for arm in ARMS}
        self.aperture = dict.fromkeys(ARMS, OPEN)
        self.worst_ik = 0.
        self.report = {}

    # -- robot readings ----------------------------------------------------------------------------------
    def actual(self, arm):
        """Where the guided point of one arm is, from its measured joint angles."""
        if self.probe is not None and self.holding:
            return self.probe.pos(f"handle_{arm}")       # where the handle's middle really is
        start = 7 * ARMS.index(arm)
        point, rotation = ik.tcp_pose(self.measured[start:start + 6], BASES[arm])
        return point + rotation @ self.offset[arm]

    def opening(self, arm):
        """Measured gripper opening, 0 (closed) to 1 (open)."""
        return float(self.measured[7 * ARMS.index(arm) + 6])

    def note_grip(self, arm):
        self.report[f"{arm}_opening_after_close"] = round(self.opening(arm), 4)
        self.report[f"{arm}_holding"] = bool(self.opening(arm) > HOLDING_MARGIN)

    def guide(self, offset):
        """Guide another point of each gripper (the centre point, or where the wire leaves the handle)."""
        for arm in ARMS:
            self.point[arm] = self.point[arm] + self.rotation[arm] @ (offset - self.offset[arm])
            self.offset[arm] = np.array(offset, dtype=float)
            self.bias[arm] = np.zeros(3)

    # -- motion ------------------------------------------------------------------------------------------
    def move(self, seconds, targets=None, *, orientation=None, jaws=None, servo=False):
        """Move the guided points in straight lines. With ``servo`` the command is shifted, step by step,
        by the error between where each guided point should be and where the joint readings say it is."""
        targets, orientation, jaws = targets or {}, orientation or {}, jaws or {}
        arms = [arm for arm in ARMS if servo or arm in targets or arm in orientation or arm in jaws]
        start = {a: (self.point[a], ik.matrix_to_quat_xyzw(self.rotation[a]), self.aperture[a]) for a in arms}
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
                self.point[arm] = point + alpha * (target - point)
                rotation = quat + alpha * (q - quat)
                rotation /= np.linalg.norm(rotation)
                self.rotation[arm] = Rotation.from_quat(rotation).as_matrix()
                tcp = self.aim(arm) + self.bias[arm] - self.rotation[arm] @ self.offset[arm]
                self.joints[arm], error, _ = ik.solve(tcp, rotation, self.joints[arm], BASES[arm], iters=35)
                self.worst_ik = max(self.worst_ik, error)
                self.aperture[arm] = aperture + alpha * (opening - aperture)
            values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
            yield ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)
            if servo:
                for arm in arms:
                    bias = self.bias[arm] + SERVO_GAIN * (self.aim(arm) - self.actual(arm))
                    self.bias[arm] = bias * min(1., SERVO_LIMIT / max(np.linalg.norm(bias), 1e-9))
                if self.probe is not None and self.holding:      # keep the wire's pull gentle
                    tension = self.probe.tension()
                    self.peak_tension = max(getattr(self, "peak_tension", 0.), tension)
                    if tension > TAUT_N[1]:
                        self.trim = min(TRIM_MAX, self.trim + TRIM_STEP)
                    elif tension < TAUT_N[0]:
                        self.trim = max(0., self.trim - TRIM_STEP)
        if not servo:
            for arm in arms:
                self.bias[arm] = np.zeros(3)

    def aim(self, arm):
        """Where the guided point is wanted: the planned point, eased toward the midline by the trim."""
        return self.point[arm] - (0., SIDE[arm] * self.trim, 0.)

    def hands(self, seconds, x, y, z, **kwargs):
        """Both guided points to (x, +-y, z), mirrored about the robot's midline."""
        yield from self.move(seconds, {arm: (x, SIDE[arm] * y, z) for arm in ARMS}, servo=True, **kwargs)

    # -- the demonstration -------------------------------------------------------------------------------
    def pick_up_handles(self):
        grasp = dict(GRASP)
        if self.probe is not None:               # aim at where each handle really lies
            for arm in ARMS:
                found = self.probe.pos(f"handle_{arm}")
                grasp[arm] = np.array((found[0] - GRIP_BACK, found[1], GRASP_Z))
            self.report["handles_found_mm"] = {arm: (1000 * self.probe.pos(f"handle_{arm}")).round(1).tolist() for arm in ARMS}
        yield from self.move(1.4, {arm: grasp[arm] + (0., 0., .065) for arm in ARMS})
        yield from self.move(1.2, grasp, servo=True)
        yield from self.move(.4, servo=True)
        yield from self.move(.5, jaws=dict.fromkeys(ARMS, 0.))
        yield from self.move(.4)
        for arm in ARMS:
            self.note_grip(arm)
        yield from self.move(1.0, {arm: grasp[arm] + (0., 0., .060) for arm in ARMS})
        if self.probe is not None:
            lifted = {arm: float(self.probe.pos(f"handle_{arm}")[2]) for arm in ARMS}
            self.report["handles_lifted_mm"] = {arm: round(1000 * z, 1) for arm, z in lifted.items()}
            if min(lifted.values()) < .030:
                raise RuntimeError(f"A handle was not picked up: heights {lifted}")

    def wire_behind_the_cake(self):
        self.guide(WIRE_END)
        lean = dict.fromkeys(ARMS, LEAN)
        yield from self.hands(2.0, .160, HALF_WIRE - .015, CARRY_Z, orientation=lean)
        self.holding = True
        self.taut_y = TAUT_Y
        yield from self.hands(1.2, .160, self.taut_y, CARRY_Z)     # draw the wire straight
        yield from self.hands(.6, .160, self.taut_y, CARRY_Z)
        if self.probe is not None:
            self.report["taut_tension_n"] = round(self.probe.tension(), 2)
        yield from self.hands(4.0, BEHIND_X, self.taut_y, CARRY_Z)      # over the cake
        yield from self.hands(2.0, BEHIND_X, self.taut_y, SEAM_Z)       # down to the scored line
        yield from self.hands(.5, BEHIND_X, self.taut_y, SEAM_Z)

    def cut(self):
        yield from self.hands(2.0, TOUCH_X, self.taut_y, SEAM_Z)        # the straight wire against the far face
        for angle in np.radians(np.linspace(0., SWING_DEG, 11)[1:]):
            radius = SWING_RADIUS + self.taut_y - TAUT_Y
            yield from self.hands(.5, FAR - radius * np.sin(angle), CORNER_Y + radius * np.cos(angle), SEAM_Z)
        x = DRAW_START_X
        end = DRAW_END_X
        while x > end + 1e-9:
            x = max(end, x - DRAW_SPEED * .25)
            yield from self.hands(.25, x, DRAW_Y, SEAM_Z)
            if self.probe is not None and end == DRAW_END_X and self.probe.cut()[0] >= CUT_DONE:
                end = max(DRAW_END_X, x - .020)          # the wire is through: 20 mm more, then stop
                self.report["cut_done_at_x_mm"] = round(float(1000 * x), 1)
        yield from self.hands(.5, x, DRAW_Y, SEAM_Z)
        if self.probe is not None:
            self.report["cut_on_line_off_line"] = [round(v, 3) for v in self.probe.cut()]
        self.draw_end = x
        self.report["openings_after_cut"] = [round(self.opening(arm), 4) for arm in ARMS]

    def put_down_handles(self):
        self.holding = False
        self.report["peak_tension_seen_n"] = round(getattr(self, "peak_tension", 0.), 2)
        self.report["final_trim_mm"] = round(1000 * self.trim, 2)
        for arm in ARMS:                         # carry the trim over into the planned points
            self.point[arm] = self.aim(arm)
        self.trim = 0.
        self.guide(np.zeros(3))
        place = {arm: np.array((self.draw_end - GRIP_BACK, SIDE[arm] * DRAW_Y, LAY_Z)) for arm in ARMS}
        yield from self.move(2.0, {arm: place[arm] + (0., 0., .030) for arm in ARMS},
                             orientation=dict.fromkeys(ARMS, DOWN))
        yield from self.move(1.0, place, servo=True)
        yield from self.move(.4, jaws=dict.fromkeys(ARMS, OPEN))
        yield from self.move(.8, {arm: place[arm] + (0., 0., .080) for arm in ARMS})
        yield from self.move(1.2, PARK)

    def sequence(self):
        yield from self.move(1.2, PARK, orientation=dict.fromkeys(ARMS, DOWN), jaws=dict.fromkeys(ARMS, OPEN))
        yield from self.move(.5)
        yield from self.pick_up_handles()
        yield from self.wire_behind_the_cake()
        yield from self.cut()
        yield from self.put_down_handles()
        while True:
            yield from self.move(1.)
