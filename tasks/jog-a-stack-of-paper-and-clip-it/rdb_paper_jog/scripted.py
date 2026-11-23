"""Full-score demonstration: both hands take the skewed stack off the block by its overhanging long edge and
stand it on that edge on the table. Then one hand goes to each end of the standing stack and the two jog it the
way a person does: loosely held, lifted and dropped so the long edges come flush on the table, and patted
together and shaken from side to side so the short edges come flush between the two hands. The right hand then
keeps the stack pinched while the left hand fetches the binder clip, squeezes it open and puts it on the middle
of the free short edge; both hands lay the clipped stack back on the block and let go.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a
scripted demonstration, simulator state: where the nine plates and the clip's two halves are. It changes
nothing in the simulation: paper moves only where fingers, the table and the block push it, and the clip opens
only because its two handles are squeezed.

Plan (R, L: right and left hand; "gripper point": the middle between the finger pads).

 1. reach, pinch: each hand comes in level, toed in 30 degrees, pads one above the other, onto the long edge
    that overhangs the block, 15 mm in from the most set-back sheet, and closes. Measured opening checked.
 2. draw off and stand: the hands lift the pinched edge while drawing the stack toward the robot, its far edge
    sliding back over the block, then turn it to hang from the pinched edge in the plane x = JOG_X, 12 mm
    above the table, and open to the stack's thickness plus 3 mm: the sheets drop onto the table.
 3. a hand to each end: first L, then R leaves the top edge and comes down at 30 degrees over the stack's upper
    corner on its own side, fingers loosely on either side of the paper. Behind the finger pads the gripper
    has a cross-bar; it stops 61 mm behind the gripper point and is what the sheets' ends meet.
 4. jog: both hands pinch, lift 15 mm and open, at least twice and until the long edges are flush within 0.5 mm
    (at most five times).
 5. pat and shake: with the fingers loose, both hands move toward each other until their cross-bars are a
    sheet's length plus 2.5 mm apart (the paper is gathered, never squeezed), then shake together from side
    to side, 12 mm, four strokes. After each stroke every sheet stands against the bar that pushed it; the
    last stroke leaves them against the right hand.
 6. clamp: R pinches where it is. From here until the clip is on, R does not let go.
 7. fetch the clip: L leaves its end, comes down over the clip on its fin, closes on its two handles, which
    opens the jaws, and lifts it off.
 8. clip: L turns the clip nearly level, moves it onto the middle of the free short edge until the lips are
    15 mm past the edge (further, up to 19 mm, if the sheets' ends are not flush), and opens: the clip shuts on
    the stack. L backs off.
 9. lay back: L takes the top edge, then R moves from the end to the top edge; both lay the stack back on the
    block the way it came off, measuring how far it hangs below the hands, let go, back off and go home.

``variant`` gives three controls that must not reach full score: ``"no_jog"`` never opens the pinch;
``"one_direction"`` lets the sheets drop onto the table and jogs the long edges, but no hand goes to the ends
and the short edges are never patted; ``"release_before_clip"`` squares the stack, then R lets go before the
clip is on.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

from . import grading, paper_model as m
from .fixed_scene import ARMS, BASES, CONTROL_HZ

LEFT, RIGHT = ARMS
JAW = 2. * abs(float(FINGER_OPEN))      # pad faces this far apart with the gripper fully open
DT = 1. / CONTROL_HZ
VARIANTS = ("full", "no_jog", "one_direction", "release_before_clip")

TOE = np.radians(30.)                   # the hands come in toed in by this
PINCH_Y = {RIGHT: -.060, LEFT: .070}    # where each hand takes the long edge, from the stack's middle
PINCH_IN = .015                         # gripper point this far in from the paper's edge
PRE = .060                              # hands stop this far short before going in
WIDE = .030                             # opening for going onto the paper
CRADLE = m.STACK_T + .003               # the loose cradle
END_TILT = np.radians(30.)              # a hand on the stack's end points this far below level
THROAT = (.0624, .0140)                 # the fingers' cross-bars: behind the gripper point, and their inner ends off its axis
END_BACK = THROAT[0] * np.cos(END_TILT) + THROAT[1] * np.sin(END_TILT)    # the bar's corner behind the gripper point, level
END_UP = THROAT[0] * np.sin(END_TILT) - THROAT[1] * np.cos(END_TILT)      # and above it
END_Z = m.SHEET_U - .032                # gripper-point height at an end: bar corner 13 mm below the sheets' top edge
END_ABOVE = .060                        # a hand comes down to its end, and leaves it, from this far straight above
TOP_PRE = .036                          # a hand comes onto the top edge from this far back along its fingers
END_OUT = .004                          # a hand arrives at its end with its bar this far outside the furthest sheet
PAT_STEP = .0004                        # each hand closes in by this per control step (8 mm/s)
PAT_PLAY = .0025                        # until the two bars are a sheet's length apart plus this: the paper is never squeezed
PAT_TRAVEL = .040                       # or each has closed in by this
SHAKE = .006                            # the side-to-side shake: more than the play, so every sheet ends against one bar
PAT_Y = -.030                           # the stack's middle while a hand is at each end (both ends in easy reach)
CARRY_LIFT = .004                       # the squared stack is carried this far above the table to where it is clipped
GRASP_OK = (.0020, .0075)               # measured opening that means the whole stack is between the pads
TILT = np.radians(35.)                  # the stack's slope when its far edge leaves the block
OFF_BLOCK = .010                        # the far edge is drawn this far past the block's near face before turning
HANG = .012                             # the hanging stack's lower edge above the table
LAY_CLEAR = .030                        # laying back: the far edge is first held this far above the block's top
LAY_SAG_MAX = .030                      # and the hands are raised by up to this for the measured sag
JOG_LIFT = .015
JOG_FLUSH = .0005
CLIP_WIDE = .045                        # opening for going over the clip's handles
CLIP_GRIP = .0145                       # gripper point this far behind the clip's back
CLIP_OPEN_DEG = 30.                     # the clip counts as squeezed open beyond this
LIPS_PAST = (.015, .019)                # the lips go this far past the paper's edge, at least and at most
CLIP_TILT = np.radians(10.)              # the clip goes on with the left hand pointing this far below level
# Joint seeds for the postures the arms glide into (found with tools/find_seeds.py): each one stays clear of the
# joint limits along the whole move that follows it.
SEED = {
    (RIGHT, "level"): (.15, 2.34, 1.03, 1.32, -.37, -1.57), (LEFT, "level"): (-.57, 2.49, 1.37, 1.11, -.04, 1.57),
    (LEFT, "down"): (-.02, 1.54, .95, -.98, 0., -.02), (LEFT, "clip"): (-.25, 1.84, .77, .45, 1.27, .6),
    (LEFT, "hang"): (-.64, 1.99, 2.13, -1.37, .41, .86), (RIGHT, "hang"): (.51, 1.86, 1.95, -1.39, -.45, -1.),
    (LEFT, "end"): (-.27, 1.64, 1.47, -.96, .99, 1.06), (RIGHT, "end"): (.09, 1.68, 1.53, -1.28, -1.04, -1.4),
}
END_CLOSING = {RIGHT: 1., LEFT: -1.}     # which way round each wrist is at the stack's end (tools/find_seeds.py)


def frame(approach, closing):
    """Gripper orientation: fingers point along ``approach``; they open and close along ``closing``."""
    a = np.asarray(approach, dtype=float)
    a /= np.linalg.norm(a)
    c = np.asarray(closing, dtype=float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


def hold_rot(arm, tilt):
    """A hand on the stack's long edge, toed in, with the stack sloping down away from it by ``tilt``
    (0: flat on the block; 90 degrees: hanging)."""
    side = 1. if arm == RIGHT else -1.
    approach = (np.cos(TOE) * np.cos(tilt), side * np.sin(TOE), -np.cos(TOE) * np.sin(tilt))
    # which finger is on which face of the paper is chosen per arm so that the wrist can follow the whole turn
    return frame(approach, side * np.array((np.sin(tilt), 0., np.cos(tilt))))


def end_rot(arm):
    """A hand over the stack's upper corner on its own side: fingers pointing inward and END_TILT below level,
    in the stack's plane, one pad on each face."""
    side = 1. if arm == RIGHT else -1.
    return frame((0., side * np.cos(END_TILT), -np.sin(END_TILT)), (END_CLOSING[arm], 0., 0.))


CLIP_ROT = frame((0., -np.cos(CLIP_TILT), -np.sin(CLIP_TILT)), (-1., 0., 0.))


def tcp_pose(joint_pos, arm):
    start = 7 * ARMS.index(arm)
    return ik.tcp_pose(np.asarray(joint_pos, dtype=float)[start:start + 6], BASES[arm])


class PaperReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant="full"):
        if variant not in VARIANTS:
            raise ValueError(variant)
        self.variant = variant
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.gap = {arm: JAW for arm in ARMS}            # commanded opening
        self.point = {}                                  # commanded gripper points (Cartesian moves)
        self.rot = {}
        self.worst_ik = 0.
        self.tick = 0
        self.phases = []
        self.log = {"variant": variant, "events": [], "ik_error_mm": {}}
        self.grip_offset = np.array((0., 0., CLIP_GRIP))
        self.on_top, self.at_end, self.end_point = set(), set(), {}
        self.squared_end = 0.
        self.far = m.SHEET_U - PINCH_IN                  # paper beyond the gripper points

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        if state is not None:
            self.state = state

    def opening(self, arm):
        return float(self.measured[7 * ARMS.index(arm) + 6]) * JAW

    def plates(self):
        return self.state["plates"]

    def corner_points(self):
        return np.array([m.corners(p, r) for p, r in self.plates()])

    def middle(self):
        return np.mean([p for p, _ in self.plates()], axis=0)

    def upright(self):
        """The stack stands in the jogging plane (its sheets' faces square to x)."""
        return bool(all(abs(r[0, 2]) > .9 for _, r in self.plates()))

    def flush(self):
        return m.misalignment(self.plates())

    def clip(self):
        return self.state["clip"]

    # -- motion ---------------------------------------------------------------------------
    def solve(self, arm, point, rotation, seed=None, iters=60):
        joints, error, turn = ik.solve(np.asarray(point, dtype=float), ik.matrix_to_quat_xyzw(rotation),
                                       self.joints[arm] if seed is None else np.asarray(seed, dtype=float), BASES[arm], iters=iters)
        self.worst_ik = max(self.worst_ik, error + .05 * turn)
        if self.phases:
            worst = self.log["ik_error_mm"]
            key = f"{self.phases[-1][1]} [{arm}]"
            worst[key] = max(worst.get(key, 0.), round(1000 * (error + .05 * turn), 2))
        return joints

    def emit(self):
        for arm, point in self.point.items():
            self.joints[arm] = self.solve(arm, point, self.rot[arm])
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.gap[a] / JAW, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def glide(self, seconds, targets, gap=None, seeds=None):
        """Move in joint space to gripper poses {arm: (point, rotation)} (smooth start and stop)."""
        seeds = seeds or {}
        goal = {}
        for arm, (p, r) in targets.items():
            seed = SEED.get((arm, seeds.get(arm))) if arm in seeds else None
            goal[arm] = self.solve(arm, p, r, seed=seed, iters=400)
        start = {arm: self.joints[arm].copy() for arm in targets}
        self.gap.update(gap or {})
        self.point = {arm: point for arm, point in self.point.items() if arm not in targets}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm in targets:
                self.joints[arm] = start[arm] + alpha * (goal[arm] - start[arm])
            yield self.emit()
        for arm, (point, rotation) in targets.items():
            self.point[arm] = np.asarray(point, dtype=float).copy()
            self.rot[arm] = rotation

    def follow(self, seconds, path, gap=None):
        """Follow ``path(s)`` -> {arm: (point, rotation)}, s running smoothly from 0 to 1."""
        self.gap.update(gap or {})
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm, (point, rotation) in path(alpha).items():
                self.point[arm] = np.asarray(point, dtype=float)
                self.rot[arm] = rotation
            yield self.emit()

    def line(self, seconds, targets, gap=None):
        """Move gripper points in straight lines to {arm: point}, orientation kept."""
        start = {arm: self.point[arm].copy() for arm in targets}
        goal = {arm: np.asarray(p, dtype=float) for arm, p in targets.items()}
        rot = dict(self.rot)
        yield from self.follow(seconds, lambda s: {arm: (start[arm] + s * (goal[arm] - start[arm]), rot[arm]) for arm in goal}, gap)

    def home(self, arms, seconds):
        """Open the fingers and swing the arms back to their ready pose in joint space."""
        for arm in arms:
            self.point.pop(arm, None)
        start = {arm: self.joints[arm].copy() for arm in arms}
        jaw = {arm: self.gap[arm] for arm in arms}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm in arms:
                self.joints[arm] = start[arm] + alpha * (np.asarray(READY) - start[arm])
                self.gap[arm] = jaw[arm] + min(1., 3. * alpha) * (JAW - jaw[arm])
            yield self.emit()

    def wait(self, seconds, gap=None):
        self.gap.update(gap or {})
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def event(self, name, **extra):
        long_side, short_side = self.flush()
        clip = self.clip()
        row = {"tick": self.tick, "event": name, "long_edges_mm": round(1000 * long_side, 2),
               "short_edges_mm": round(1000 * short_side, 2), "stack_middle_mm": [round(1000 * float(v), 1) for v in self.middle()],
               "lowest_corner_mm": round(1000 * float(self.corner_points()[:, :, 2].min()), 1),
               "clip_opening_deg": round(float(np.degrees(clip["opening"])), 1),
               "sheets_in_jaws": m.PER_PLATE * sum(grading.in_jaws(self.plates(), clip)),
               "gripper_opening_mm": {arm: round(1000 * self.opening(arm), 2) for arm in ARMS},
               "gripper_points_mm": {arm: [round(1000 * float(v), 1) for v in tcp_pose(self.measured, arm)[0]] for arm in ARMS}}
        row.update(extra)
        self.log["events"].append(row)

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- geometry of the plan ---------------------------------------------------------------
    def edge_at(self, y):
        """Flat stack: x of every plate's near long edge where it crosses ``y``."""
        out = []
        for position, rotation in self.plates():
            near = position - rotation[:, 0] * m.SHEET_U / 2
            t = (y - near[1]) / rotation[1, 1]
            out.append(near[0] + t * rotation[0, 1])
        return np.array(out)

    def edge_pose(self, far_x, tilt, lift=0.):
        """Both hands' gripper poses with the stack sloping by ``tilt`` and its far edge at ``far_x``, a
        sheet's thickness above the block's top plus ``lift``."""
        z = m.BLOCK[2] + m.STACK_T / 2 + .0015 + lift
        return self.pinch_pose(far_x - self.far * np.cos(tilt), z + self.far * np.sin(tilt), tilt)

    def pinch_pose(self, x, z, tilt, shift=0.):
        return {arm: (np.array((x, self.pinch_y[arm] + shift, z)), hold_rot(arm, tilt)) for arm in ARMS}

    def draw_off(self, reverse=False):
        """Path between the stack flat on the block (s = 0) and sloping TILT with its far edge just off the
        block (s = 1)."""
        far0, far1 = self.flat_far, m.BLOCK_NEAR_X - OFF_BLOCK

        def path(s):
            s = 1. - s if reverse else s
            return self.edge_pose(far0 + s * (far1 - far0), s * TILT, lift=.004 * s)
        return path

    def swing(self, shift=0.):
        """Path from sloping TILT just off the block (s = 0) to hanging in the jogging plane (s = 1), moved
        ``shift`` along y on the way."""
        start = self.edge_pose(m.BLOCK_NEAR_X - OFF_BLOCK, TILT, lift=.004)[RIGHT][0]
        end = np.array((m.JOG_X, 0., self.far + HANG))

        def path(s):
            x, z = start[0] + s * (end[0] - start[0]), start[2] + s * (end[2] - start[2])
            return self.pinch_pose(x, z, TILT + s * (np.pi / 2 - TILT), s * shift)
        return path

    # -- the demonstration ------------------------------------------------------------------
    def sequence(self):
        yield from self.pick_up()
        yield from self.stand()
        if self.variant == "no_jog":
            yield from self.set_down_pinched()
            yield from self.leave_top(LEFT)
        elif self.variant == "one_direction":
            yield from self.first_drop()
            yield from self.jog()
            yield from self.leave_top(LEFT)
            yield from self.clamp()
        else:
            yield from self.first_drop()
            yield from self.to_ends()
            yield from self.jog()
            yield from self.pat_and_shake()
            yield from self.clamp()
            yield from self.carry_to_clip()
            if self.variant == "release_before_clip":
                yield from self.let_go_early()
        yield from self.fetch_clip()
        yield from self.put_clip_on()
        yield from self.lay_back()
        self.phase("wait")
        while True:
            yield self.emit()

    def pick_up(self):
        self.phase("reach")
        middle = self.middle()
        self.pinch_y = {arm: float(middle[1] + PINCH_Y[arm]) for arm in ARMS}
        self.on_top, self.at_end, self.end_point = set(ARMS), set(), {}     # which hands are where on the standing stack
        targets, back = {}, {}
        for arm in ARMS:
            rotation = hold_rot(arm, 0.)
            x = float(self.edge_at(self.pinch_y[arm]).max()) + PINCH_IN
            targets[arm] = np.array((x, self.pinch_y[arm], middle[2]))
            back[arm] = (targets[arm] - PRE * rotation[:, 2], rotation)
        self.flat_far = float(np.mean([targets[arm][0] for arm in ARMS])) + self.far

        def come_down(s):
            """From above and behind the edge, sloping as the stack will when it is lifted (s = 0), down to
            level just short of the edge (s = 1): the fingertips never pass over the paper."""
            return {arm: (back[arm][0] + (1. - s) * np.array((-.040, 0., .110)), hold_rot(arm, (1. - s) * TILT)) for arm in ARMS}

        yield from self.glide(2., come_down(0.), {arm: WIDE for arm in ARMS}, seeds={arm: "level" for arm in ARMS})
        yield from self.follow(1.5, come_down)
        for attempt in range(3):
            self.phase(f"pinch the overhanging edge (attempt {attempt + 1})")
            yield from self.line(1.2, targets)
            yield from self.wait(.7, {arm: 0. for arm in ARMS})
            got = {arm: GRASP_OK[0] <= self.opening(arm) <= GRASP_OK[1] for arm in ARMS}
            self.event("pinched", attempt=attempt + 1, whole_stack=got)
            if all(got.values()) or attempt == 2:
                break
            yield from self.wait(.4, {arm: WIDE for arm in ARMS})
            yield from self.line(.8, {arm: back[arm][0] for arm in ARMS})

    def stand(self):
        self.phase("draw the stack off the block")
        yield from self.follow(4., self.draw_off())
        self.event("far edge off the block")
        self.phase("turn the stack to hang")
        shift = PAT_Y - float(self.middle()[1]) if self.variant in ("full", "release_before_clip") else 0.
        yield from self.follow(3., self.swing(shift))
        for arm in ARMS:
            self.pinch_y[arm] += shift
        yield from self.wait(.3)
        self.event("hanging over the table")

    def top_edge_z(self):
        """Gripper-point height for a hand on the top edge of the standing stack."""
        return float(self.corner_points()[:, :, 2].max(axis=1).min()) - PINCH_IN

    def hand_points(self, lift=0.):
        """Where each hand that is on the stack belongs now: on the top edge, or at its end."""
        out = {}
        for arm in self.on_top:
            out[arm] = np.array((m.JOG_X, self.pinch_y[arm], self.top_edge_z() + lift))
        for arm in self.at_end:
            out[arm] = self.end_point[arm] + (0., 0., lift)
        return out

    def set_down_pinched(self):
        """Control: stand the stack on the table without ever opening the pinch."""
        self.phase("set the pinched stack down")
        yield from self.line(1., {arm: self.point[arm] - (0., 0., HANG - .002) for arm in ARMS})
        yield from self.wait(.3)
        self.event("set down, never opened")

    def first_drop(self):
        self.phase("open the cradle: the sheets drop onto the table")
        yield from self.wait(.6, {arm: CRADLE for arm in ARMS})
        self.event("first drop")
        yield from self.line(.6, self.hand_points())

    def jog(self):
        """Pinch, lift, open: with the hands wherever they are on the stack."""
        for cycle in range(1, 6):
            self.phase(f"jog: pinch, lift, drop ({cycle})")
            hands = list(self.hand_points())
            yield from self.wait(.35, {arm: 0. for arm in hands})
            yield from self.line(.4, self.hand_points(JOG_LIFT))
            yield from self.wait(.1, {arm: CRADLE for arm in hands})
            yield from self.wait(.4)
            yield from self.line(.4, self.hand_points())
            self.event("dropped", cycle=cycle)
            if cycle >= 2 and self.flush()[0] <= JOG_FLUSH:
                break

    def leave_top(self, arm):
        """A hand opens and backs off the top edge the way its fingers point."""
        self.on_top.discard(arm)
        yield from self.wait(.3, {arm: WIDE})
        yield from self.line(.6, {arm: self.point[arm] - .070 * self.rot[arm][:, 2]})

    def leave_end(self, arm):
        self.phase(f"{arm} hand leaves its end")
        self.at_end.discard(arm)
        yield from self.wait(.3, {arm: WIDE})
        yield from self.line(.7, {arm: self.point[arm] + (0., 0., END_ABOVE)})

    def end_y(self, arm, stop):
        """Gripper-point y for a hand whose cross-bar is to be at ``stop``."""
        return stop + (END_BACK if arm == RIGHT else -END_BACK)

    def to_ends(self):
        """One hand at a time leaves the top edge and takes the stack's end on its own side; the other keeps
        the stack from falling."""
        for arm in (LEFT, RIGHT):
            self.phase(f"{arm} hand goes to its end of the stack")
            yield from self.leave_top(arm)
            ends = self.corner_points()[:, :, 1]
            stop = float(ends.max()) + END_OUT if arm == LEFT else float(ends.min()) - END_OUT
            rotation = end_rot(arm)
            point = np.array((float(self.middle()[0]), self.end_y(arm, stop), END_Z))
            yield from self.glide(1.8, {arm: (point + (0., 0., END_ABOVE), rotation)}, seeds={arm: "end"})
            yield from self.line(.9, {arm: point})           # straight down over the corner, a finger on each face
            yield from self.wait(.3, {arm: CRADLE})
            self.end_point[arm] = point
            self.at_end.add(arm)
            self.event(f"{arm} hand at its end")

    def pat_and_shake(self):
        self.phase("pat the ends together")
        yield from self.pat()
        self.event("ends patted together", bars_apart_mm=round(1000 * self.bars_apart(), 2))
        self.phase("shake from side to side")
        # Both hands move together. Whichever bar is behind pushes every sheet that touches it, so after each
        # stroke all the sheets stand against that one bar: flush. The last stroke is toward the left, which
        # leaves them against the right hand's bar, the hand that then pinches.
        for shift in (-SHAKE, 2 * SHAKE, -2 * SHAKE, 2 * SHAKE):
            for arm in ARMS:
                self.end_point[arm] = self.end_point[arm] + (0., shift, 0.)
            yield from self.line(.6 if abs(shift) == SHAKE else .9, self.hand_points())
            yield from self.wait(.15)
            self.event("shaken to one side", bars_apart_mm=round(1000 * self.bars_apart(), 2))
        self.event("shaken")

    def bars_apart(self):
        """Distance between the two hands' cross-bars, from the measured gripper points."""
        return float(tcp_pose(self.measured, LEFT)[0][1] - tcp_pose(self.measured, RIGHT)[0][1]) + 2 * END_BACK

    def pat(self):
        """Both hands close in on each other until their cross-bars are a sheet's length apart plus a little
        play, then stop where they are: the sheets are gathered between them, not squeezed."""
        for _ in range(round(PAT_TRAVEL / PAT_STEP)):
            if self.bars_apart() <= m.SHEET_V + PAT_PLAY:
                break
            for arm in ARMS:
                self.end_point[arm] = self.end_point[arm] + (0., PAT_STEP if arm == RIGHT else -PAT_STEP, 0.)
                self.point[arm] = self.end_point[arm].copy()
            yield self.emit()
        for arm in ARMS:                             # stop here: do not run on by the arms' lag
            self.end_point[arm][1] = float(tcp_pose(self.measured, arm)[0][1])
            self.point[arm] = self.end_point[arm].copy()
        yield from self.wait(.3)

    def clamp(self):
        hands = sorted(self.at_end) or [RIGHT]
        self.phase("pinch and hold")
        yield from self.wait(.5, {arm: 0. for arm in hands})
        self.event("clamped", whole_stack={arm: bool(GRASP_OK[0] <= self.opening(arm) <= GRASP_OK[1]) for arm in hands})

    def carry_to_clip(self):
        """With a hand at each end the stack stands where both ends are in easy reach, which puts its free end
        out of the left hand's level reach. The right hand therefore changes to the middle of the top edge
        (the left hand holding meanwhile), the left hand leaves, and the right hand carries the squared stack
        to where the left hand can put the clip on. At every moment at least one hand has it pinched."""
        yield from self.leave_end(RIGHT)
        yield from self.take_top(RIGHT, 0.)
        yield from self.leave_end(LEFT)
        self.phase("carry the squared stack to where it is clipped")
        shift = m.BLOCK_Y - float(self.middle()[1])
        up = self.point[RIGHT] + (0., 0., CARRY_LIFT)
        yield from self.line(.4, {RIGHT: up})
        yield from self.line(1.2, {RIGHT: up + (0., shift, 0.)})
        yield from self.line(.4, {RIGHT: self.point[RIGHT] - (0., 0., CARRY_LIFT)})
        self.pinch_y[RIGHT] += shift
        self.event("carried", whole_stack=bool(GRASP_OK[0] <= self.opening(RIGHT) <= GRASP_OK[1]))

    def let_go_early(self):
        """Control: the holding hand lets go before the clip is on."""
        self.phase("right hand lets go early")
        yield from self.wait(.4, {RIGHT: WIDE})
        yield from self.line(.6, {RIGHT: self.point[RIGHT] - .040 * self.rot[RIGHT][:, 2]})
        yield from self.home([RIGHT], 2.)
        self.at_end.discard(RIGHT)
        self.on_top.discard(RIGHT)
        self.event("let go before the clip")

    def clip_hold(self):
        """Left gripper pose for taking the clip by its handles where it is."""
        clip = self.clip()
        rotation = frame(clip["rotation"][:, 2], -clip["rotation"][:, 0])
        return clip["origin"] - CLIP_GRIP * clip["rotation"][:, 2], rotation

    def fetch_clip(self):
        self.phase("reach the clip")
        point, rotation = self.clip_hold()
        yield from self.glide(2.3, {LEFT: (point - PRE * rotation[:, 2], rotation)}, {LEFT: CLIP_WIDE}, seeds={LEFT: "down"})
        for attempt in range(3):
            self.phase(f"squeeze the clip's handles (attempt {attempt + 1})")
            yield from self.line(1., {LEFT: point})
            yield from self.wait(.7, {LEFT: 0.})
            opened = float(np.degrees(self.clip()["opening"]))
            self.event("handles squeezed", attempt=attempt + 1, opened=bool(opened >= CLIP_OPEN_DEG))
            if opened >= CLIP_OPEN_DEG or attempt == 2:
                break
            yield from self.wait(.4, {LEFT: CLIP_WIDE})
            yield from self.line(.6, {LEFT: point - .030 * rotation[:, 2]})
            point, rotation = self.clip_hold()
        self.phase("lift the clip off its fin")
        yield from self.line(.8, {LEFT: point - .090 * rotation[:, 2]})
        hand_point, hand_rot = tcp_pose(self.measured, LEFT)
        self.grip_offset = hand_rot.T @ (self.clip()["origin"] - hand_point)
        self.event("clip in hand", clip_in_hand_mm=[round(1000 * float(v), 1) for v in self.grip_offset])

    def put_clip_on(self):
        self.phase("bring the clip to the short edge")
        corners = self.corner_points()
        past = LIPS_PAST[0]
        if self.upright():
            ends = corners[:, :, 1].max(axis=1)                      # every plate's free short edge
            edge = float(ends.max())
            past = float(np.clip(np.ptp(ends) + .006, *LIPS_PAST))   # far enough on to take the most set-back sheet
            x, z = float(self.middle()[0]), float(self.middle()[2])
        else:                                   # control: the stack has fallen; go where it was squared
            edge, x, z = self.squared_end, m.JOG_X, m.SHEET_U / 2
        push = CLIP_ROT[:, 2]                    # the way the clip goes on
        on = np.array((x, edge, z)) + (m.JAW_L - past) * -push - CLIP_ROT @ self.grip_offset
        yield from self.glide(2.5, {LEFT: (on - .035 * push, CLIP_ROT)}, seeds={LEFT: "clip"})
        yield from self.wait(.3)
        self.phase("push the clip on and let its handles go")
        yield from self.line(1.2, {LEFT: on})
        yield from self.wait(.6, {LEFT: CLIP_WIDE})
        self.event("clip let go")
        yield from self.line(.8, {LEFT: on - .055 * push})
        self.event("left hand off the clip")
        yield from self.line(.8, {LEFT: on - .055 * push + (0., .020, .150)})      # up, clear of the clip and the stack's corner

    def take_top(self, arm, offset=None):
        """A hand comes down on the top edge of the standing stack, ``offset`` from its middle, and pinches."""
        self.phase(f"{arm} hand takes the top edge")
        rotation = hold_rot(arm, np.pi / 2)
        self.pinch_y[arm] = float(self.middle()[1] + (PINCH_Y[arm] if offset is None else offset))
        point = np.array((m.JOG_X, self.pinch_y[arm], self.top_edge_z()))
        yield from self.glide(2., {arm: (point - TOP_PRE * rotation[:, 2], rotation)}, {arm: WIDE}, seeds={arm: "hang"})
        yield from self.line(.8, {arm: point})
        yield from self.wait(.6, {arm: 0.})
        self.on_top.add(arm)

    def lay_back(self):
        if not (self.upright() and RIGHT in self.on_top):
            self.phase("hands away (nothing to lay back)")
            yield from self.home(list(ARMS), 2.5)
            return
        yield from self.take_top(LEFT)
        wanted = float(self.middle()[1] + PINCH_Y[RIGHT])
        if abs(self.pinch_y[RIGHT] - wanted) > .005:      # the clip is on: the right hand may slide along the top edge
            self.phase("right hand slides along the top edge to its own place")
            yield from self.wait(.3, {RIGHT: CRADLE})
            self.pinch_y[RIGHT] = wanted
            yield from self.line(1., {RIGHT: np.array((m.JOG_X, wanted, self.top_edge_z()))})
            yield from self.wait(.5, {RIGHT: 0.})
        self.far = m.SHEET_U - PINCH_IN
        self.event("both hands on the top edge", whole_stack={arm: bool(GRASP_OK[0] <= self.opening(arm) <= GRASP_OK[1]) for arm in ARMS})
        self.phase("lay the clipped stack back on the block")
        top = m.BLOCK[2]

        def held(far_x, far_z, tilt):
            """Both hands' poses with the stack's far edge at (far_x, far_z) and the stack sloping by ``tilt``."""
            return self.pinch_pose(far_x - self.far * np.cos(tilt), far_z + self.far * np.sin(tilt), tilt)

        def between(a, b):
            return lambda s: held(*(np.asarray(a) + s * (np.asarray(b) - a)))

        hanging = (m.JOG_X, HANG, np.pi / 2)
        before = (m.BLOCK_NEAR_X - .030, top + LAY_CLEAR, TILT)
        yield from self.line(.8, {arm: held(*hanging)[arm][0] for arm in ARMS})
        yield from self.follow(3., between(hanging, before))
        yield from self.wait(.4)
        # Held out by one edge, the stack hangs lower than the hands say. Measure by how much and allow for it.
        sag = float(np.clip(before[1] - self.corner_points()[:, :, 2].min(), 0., LAY_SAG_MAX))
        over = (m.BLOCK_NEAR_X + .025, top + .012 + sag, TILT)
        self.event("held out in front of the block", sag_mm=round(1000 * sag, 1))
        yield from self.follow(1.2, between(before, (before[0], over[1], TILT)))
        yield from self.follow(1.5, between((before[0], over[1], TILT), over))
        resting = (over[0], top + m.STACK_T / 2 + .0015 + .004, TILT)
        yield from self.follow(1.2, between(over, resting))
        self.event("far edge on the block")
        yield from self.follow(4., between(resting, (self.flat_far, top + m.STACK_T / 2 + .0015, 0.)))
        yield from self.wait(.3)
        self.event("laid on the block")
        self.phase("let go and move away")
        yield from self.wait(.5, {arm: WIDE for arm in ARMS})
        yield from self.line(1., {arm: self.point[arm] - .045 * self.rot[arm][:, 2] for arm in ARMS})
        yield from self.home(list(ARMS), 2.)
        self.event("parked")
