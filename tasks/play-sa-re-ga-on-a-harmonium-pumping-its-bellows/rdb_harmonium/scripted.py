"""Full-score demonstration: the left hand pumps the bellows board at the back while the right hand plays Sa, Re
and Ga, each for at least 4.4 s, legato (each next key down before the last one comes up).

The demonstration reads the robot's own joint readings and, as the review rules allow for a scripted
demonstration, simulator state: the bellows board's angle, how much air the reservoir holds, how far each key is
down and which reeds are sounding. It changes nothing in the simulation: the board moves only because the left
hand pushes it, and a key only because the right hand presses it.

Both grippers are closed throughout and used as paddles, the hand tipped forward along the line from its arm's
base (the YAM reaches these heights, 0.18 to 0.21 m above the table, only tipped):

- Left hand (pump): its closed fingers reach over the board's top edge from above and behind, near the board's
  left end, and pull toward the robot: the lower finger's outer face, 22 mm up from its tip, pushes the top edge
  shut. The board is servoed on its measured angle (the arm is compliant, so the command leads the board by a bias
  that grows while the board lags, aimed along the finger face's normal; see FACE_SLOPE). If the hand slips off
  the edge it goes back behind the board and tries again further up the finger. A stroke closes the board from open (0.30 rad) to 0.12 rad (1.7 L of air, the rest let out by
  the relief valve) whenever the reservoir is less than half full; then the hand backs off and the board's spring
  reopens it.
- Right hand (keys), legato as players do it: tipped 55 degrees and turned 35 degrees about the vertical so that
  the lower pad tip's 15 mm end edge lies nearly along the keys. A twist of joint 6 alone (+-30 degrees) tilts the
  edge so one end is lower; a note is held at a straddle (low end in the held key, raised end over the next), and
  the change of note is the twist the other way: the next key goes down as the held one comes up. Two legs then
  carry the edge across the new key to the next straddle (see key_track). Each note sounds at least 4.4 s.
Sa waits until the reservoir is primed. After Ga both hands back away and return to ready.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import YAM_URDF, ik
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, READY, TCP_OFFSET
from yam_arena.urdf import axis_angle_matrix, parse_joints, rpy_matrix

from . import layout as L
from .fixed_scene import ARMS, BASES, CONTROL_HZ, flap_corner, key_top
from .grading import HOLD_S, PHRASE

OPEN = float(FINGER_OPEN)
DT = 1. / CONTROL_HZ
PUMP_TILT = np.radians(25.)        # tip of the pumping hand from straight down, along the line from its base
KEY_TILT = np.radians(55.)
FACE = .004                        # closed fingers: each pad's outer face is 4 mm from the TCP axis (tip hulls)
PUMP_DEPTH = -.009                 # the board's edge meets the finger 9 mm above the TCP, 22 mm up from the pad tips,
PUMP_FACE = .0075                  # where the closed fingers' outer face is 7.5 mm from the axis (tip hulls)
FACE_SLOPE = np.radians(11.5)      # that face widens 11.5 degrees up the finger (3.9 mm at the tip, 10 mm 35 mm up), so
                                   # in the 25-degree tipped hand it leans 36 degrees from vertical: run 1's finger rode
                                   # up and over the edge under 3 N (friction needs tan 36 = 0.73). The push is aimed
                                   # along the face's normal instead (back and 37 degrees down), so it needs no friction
SINK_MAX = .012                    # at most 12 mm of the lead goes downward; the rest is horizontal
RIDE_UP = .015                     # measured hand this far above its command, or 20 mm in front of the edge: it slipped
RETRY_DEEPER = .008                # each retry meets the edge 8 mm further up the finger
MAX_RECOVERIES = 3                 # after that the hand keeps pumping as it is, without slip checks
TRACK_MM, TRACK_DEG = .0025, 3.    # a key-hand move steps on only when the measured hand's error has changed less than
STEP_WAIT = 10                     # this since the move began (at most 10 ticks of waiting per step), so the executed
                                   # path stays on the planned one whatever the joints' lags
XY_GAIN, XY_MAX = .08, .012        # key hand: servo its pad edge's place on the measured hand (run 2: the hand swung
                                   # 20 mm along the keyboard during a change of note and pressed the wrong key)
KEY_TIP = .013                     # the pad tips are 13 mm past the TCP
CLEAR = .006                       # waiting: the finger face 6 mm behind the open board's edge
SIDE_Y = .31                       # the left hand comes down here, beyond the board's left end (y 0.265)
TH_LOW = .12                       # a stroke closes the board to this angle (rad)
PUSH_RATE = .25                    # rad/s of the board while pushing (47 mm/s at the top edge)
RETURN_RATE = .5
PUMP_BELOW = .5                    # start a stroke when the reservoir is less than half full
PRIMED = .85                       # Sa waits for this much
BIAS_GAIN, BIAS_MAX = .35, .070    # m per m of lag per tick; at most 70 mm of lead
KEY_HOVER = .015
NOTE_S = HOLD_S + .4
KEY_YAW = np.radians(-35.)         # the key hand turned about the vertical: its pad edge then runs nearly along the keys
TWIST = np.radians(30.)            # rocking is a twist of joint 6 alone (about the hand's own axis) by +-30 degrees:
                                   # the pad edge's ends then differ 6.3 mm in height and the low end moves 8.9 mm
                                   # along the keyboard from one key into the next. Runs 3 and 4 rocked by a roll
                                   # that also moved the shoulder; with the wrist lagging the shoulder (0.15 s against
                                   # 0.07 s, fitted to both runs) the edge swung 12 to 21 mm along the keyboard
EDGE = .0077                       # half the pad tip's end edge (the real pad is 15.4 mm wide)
KEY_DEPTH = .0033                  # the held key is kept at least this far down (it speaks from 2.5 mm, stops at 7 mm);
                                   # shallow, so that the part of the edge just across the key boundary, rising 0.41 mm
                                   # per mm along it, stays under the next key's 2.5 mm
DEPTH_GAINS = (.3, .05)            # servo on the measured depth: quick to press deeper, slow to back off
GUARD_DEPTH = .0018                # any other key this far down stops the depth servo pressing deeper
DZ_RANGE = (-.016, .0015)          # run 5: the pressing hand rode 6 to 7 mm above its plan (runs 4 and 5 alike); the
                                   # servo was capped at 6 mm, so Sa and Re stayed 1.9 mm down and never sounded
LEGATO = (.6, 1.2, 1.2)            # rock (joint 6 only, not gated: nothing else moves), then two gated legs that carry
                                   # the held key's edge across it to the next straddle
TRACK_MM, TRACK_DEG = .0025, 3.    # a key-hand move steps on only when the measured hand's error has changed less than
STEP_WAIT = 10                     # this since the move began (at most 10 ticks of waiting per step), so the executed
                                   # path stays on the planned one whatever the joints' lags
SEED = {"left": (.35, 1.9, 1.7, -.5, .3, 0.), "right": (-.6, 1.6, 1.3, -.2, -.3, 0.)}
SEEDS = [np.array(s, dtype=float) for s in ([0, 1, 1, -.6, 0, 0], [0, 1.6, 1.3, -1.2, 0, 0], [0, 1.2, .6, .3, 0, 0],
                                            [.61, 1.99, 1.47, -1.03, 0, 0], [-.61, 1.99, 1.47, -1.03, 0, 0],
                                            [0, 2.1, 1.0, -.4, 1.5, -.1])]


class Chain:
    """Fast forward kinematics and damped-least-squares IK for one YAM arm (same URDF, TCP and weighting as
    yam_arena.ik, with a geometric Jacobian so that a per-tick solve is cheap)."""

    def __init__(self):
        joints = parse_joints(YAM_URDF)
        by_child = {j.child: j for j in joints.values()}
        path, link = [], "gripper"
        while link in by_child:
            j = by_child[link]
            path.append(j)
            link = j.parent
        self.steps = []
        for j in reversed(path):
            origin = np.eye(4)
            origin[:3, :3] = rpy_matrix(j.rpy)
            origin[:3, 3] = j.xyz
            axis = np.asarray(j.axis, dtype=float) / np.linalg.norm(j.axis)
            index = ARM_JOINTS.index(j.name) if j.name in ARM_JOINTS else None
            self.steps.append((origin, axis, index))
        self.tcp = np.asarray(TCP_OFFSET, dtype=float)

    def fk(self, q, base):
        t = np.eye(4)
        t[:3, 3] = base
        axes, points = [], []
        for origin, axis, index in self.steps:
            t = t @ origin
            if index is not None:
                axes.append(t[:3, :3] @ axis)
                points.append(t[:3, 3].copy())
                r = np.eye(4)
                r[:3, :3] = axis_angle_matrix(axis, q[index])
                t = t @ r
        return t[:3, :3] @ self.tcp + t[:3, 3], t[:3, :3], axes, points

    def solve(self, target, rotation, q0, base, iters=40, damping=.05, rot_weight=.3, tol=1e-5):
        q = np.array(q0, dtype=float)
        for _ in range(iters):
            p, r, axes, points = self.fk(q, base)
            err = np.r_[target - p, rot_weight * ik._rot_error(rotation, r)]
            if np.linalg.norm(err) < tol:
                break
            jac = np.zeros((6, 6))
            for i, (z, o) in enumerate(zip(axes, points, strict=True)):
                jac[:3, i] = np.cross(z, p - o)
                jac[3:, i] = rot_weight * z
            step = jac.T @ np.linalg.solve(jac @ jac.T + damping ** 2 * np.eye(6), err)
            q = np.clip(q + step, ik.LOWER, ik.UPPER)
        p, r, _, _ = self.fk(q, base)
        return q, float(np.linalg.norm(target - p)), float(np.linalg.norm(ik._rot_error(rotation, r)))


CHAIN = Chain()


def tipped(arm, point, tilt):
    """Hand rotation: approach tipped ``tilt`` from straight down along the line from the arm's base to ``point``;
    the fingers close in that vertical plane (x axis forward and up)."""
    d = np.asarray(point[:2], dtype=float) - BASES[arm][:2]
    d /= np.linalg.norm(d)
    a = np.array((d[0] * np.sin(tilt), d[1] * np.sin(tilt), -np.cos(tilt)))
    x = np.array((d[0] * np.cos(tilt), d[1] * np.cos(tilt), np.sin(tilt)))
    return np.column_stack((x, np.cross(a, x), a))


class HarmoniumReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self):
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.aperture = dict.fromkeys(ARMS, OPEN)
        self.rotation = {"left": tipped("left", flap_corner(L.FLAP_OPEN), PUMP_TILT)}
        self.twist, self.key_dz, self.runs = -TWIST, 0., {}
        self.rotation["right"] = self.key_pose(-TWIST)
        self.deeper, self.recoveries = 0., 0
        self.xy_trim, self.target_prev, self.twist_prev = np.zeros(2), None, None
        self.waited, self.sag, self.rocking = 0, (np.zeros(3), np.zeros(3)), False
        self.offset = {arm: np.zeros(3) for arm in ARMS}         # trims from measured FK
        self.target = {arm: None for arm in ARMS}                # current contact-point target
        self.worst_ik = 0.
        self.phases = []
        self.tick = 0
        self.stop_pumping = False
        self.keys_done = False
        self.bias = 0.
        self.log = {"strokes": [], "notes": [], "trim_mm": []}

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        self.state = state

    def actual_tcp(self, arm):
        start = 7 * ARMS.index(arm)
        return CHAIN.fk(self.measured[start:start + 6], BASES[arm])[0]

    def read(self, key, default):
        return self.state[key] if self.state and key in self.state else default

    def flap(self):
        return float(self.read("flap", L.FLAP_OPEN))

    def volume(self):
        return float(self.read("volume", L.RESERVOIR_M3))

    def depth(self, name):
        return float(self.read("depths", {}).get(name, 0.))

    def sounding(self):
        return tuple(self.read("sounding", ()))

    # -- motion ---------------------------------------------------------------------------
    def tcp_for(self, arm, contact):
        """TCP that puts the hand's working point on ``contact``: for the left hand the lower finger's outer face
        6 mm past the TCP, for the right hand the lower finger's tip."""
        r = self.rotation[arm]
        x, a = r[:, 0], r[:, 2]
        if arm == "left":
            return np.asarray(contact) - (PUMP_DEPTH - self.deeper) * a + PUMP_FACE * x
        return np.asarray(contact)                     # the key hand is planned by its TCP (key_track)

    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def phase(self, name):
        self.phases.append((self.tick, name))

    def solve(self, arm, contact, *, fresh=False):
        target = self.tcp_for(arm, contact) + self.offset[arm]
        if arm == "right":
            target[:2] += self.xy_trim
        if fresh:                            # a full solve from several seeds; keep the one with most joint margin
            best = None
            for seed in [np.asarray(SEED[arm])] + SEEDS:
                q, error, rerr = ik.solve(target, ik.matrix_to_quat_xyzw(self.rotation[arm]), seed, BASES[arm],
                                          iters=300)
                if error < 5e-4 and rerr < 5e-3:
                    margin = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
                    if best is None or margin > best[1]:
                        best = (q, margin, error)
            if best is None:
                raise RuntimeError(f"No inverse kinematics for the {arm} hand at {np.round(contact, 4)}")
            q, error = best[0], best[2]
        else:
            q, error, _ = CHAIN.solve(target, self.rotation[arm], self.joints[arm], BASES[arm])
        self.worst_ik = max(self.worst_ik, error)
        return q

    def glide_joints(self, seconds, goals, apertures=None):
        start = {arm: self.joints[arm].copy() for arm in goals}
        first = {arm: self.aperture[arm] for arm in (apertures or {})}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm, goal in goals.items():
                self.joints[arm] = start[arm] + alpha * (np.asarray(goal) - start[arm])
            for arm, goal in (apertures or {}).items():
                self.aperture[arm] = first[arm] + alpha * (goal - first[arm])
            yield self.emit()

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def trim(self, arm, tries=2):
        """Shift the arm's command by its measured error at the current target (the arms sag a little)."""
        for _ in range(tries):
            yield from self.hold(.3)
            want = self.tcp_for(arm, self.target[arm])
            error = want - self.actual_tcp(arm)
            self.log["trim_mm"].append([self.tick, arm, *(round(1000 * float(v), 2) for v in error)])
            if np.linalg.norm(error) < .0015 or self.state is None:
                break
            self.offset[arm] = np.clip(self.offset[arm] + np.clip(error, -.01, .01), -.03, .03)
            self.joints[arm] = self.solve(arm, self.target[arm])

    # -- the two hands' tracks: each yields its contact-point target for one tick ------------
    def lead(self):
        """The command's lead over the board edge for the current bias: along the finger face's outward normal
        (toward the robot and down) for its first part, horizontal beyond SINK_MAX of downward travel."""
        r = self.rotation["left"]
        n = -np.cos(FACE_SLOPE) * r[:, 0] + np.sin(FACE_SLOPE) * r[:, 2]
        n /= np.linalg.norm(n)
        down = min(-n[2] * self.bias, SINK_MAX)
        return np.array((-self.bias * abs(n[0]) / np.hypot(n[0], n[1]), 0., -down))

    def servo_xy(self):
        """Shift the key hand's command by its measured horizontal error (integral), only while the hand is meant
        to be still. Run 3 integrated during the rocks too: the lag of the moving hand wound the trim up, it then
        overshot 15 mm along the keyboard and pressed F4 for 0.5 s."""
        steady = (self.target_prev is not None and self.twist_prev == self.twist
                  and np.linalg.norm(self.target["right"] - self.target_prev) < 2e-4)
        if self.state is not None and steady:
            error = self.tcp_for("right", self.target["right"]) - self.actual_tcp("right")
            self.xy_trim = np.clip(self.xy_trim + XY_GAIN * error[:2], -XY_MAX, XY_MAX)
            dp, dr = self.tracking_error()                   # the standing error of a still hand (sag)
            self.sag = (.9 * self.sag[0] + .1 * dp, .9 * self.sag[1] + .1 * dr)
        self.target_prev, self.twist_prev = self.target["right"].copy(), self.twist

    def tracking_error(self):
        """(position error m, rotation error rad vector) of the measured key hand against its command."""
        p, r, _, _ = CHAIN.fk(self.measured[7:13], BASES["right"])
        q, rq, _, _ = CHAIN.fk(self.joints["right"], BASES["right"])
        return q - p, ik._rot_error(rq, r)

    def tracking_ok(self):
        """The measured key hand's error differs by less than TRACK_MM and TRACK_DEG from its standing error when
        still (the sag under gravity and key load is not a lag)."""
        if self.state is None:
            return True
        dp, dr = self.tracking_error()
        return (np.linalg.norm(dp - self.sag[0]) < TRACK_MM
                and np.degrees(np.linalg.norm(dr - self.sag[1])) < TRACK_DEG)

    def slipped(self):
        """The measured left hand has ridden up off the board's edge or passed in front of it. Compared with where
        the hand would be with the edge on its finger at the board's measured angle, not with the command, which
        leads the board and sinks below it (run 2: comparing with the command flagged every good stroke)."""
        if self.state is None or self.recoveries >= MAX_RECOVERIES:
            return False
        tcp = self.actual_tcp("left")
        on_edge = self.tcp_for("left", flap_corner(self.flap())) + self.offset["left"]
        return tcp[2] - on_edge[2] > RIDE_UP or tcp[0] < flap_corner(self.flap())[0] - .020

    def recover(self):
        """Back behind the board by the way the hand first came in: up, out past the board's left end, back,
        down, and along the edge again; then meet the edge further up the finger."""
        self.recoveries += 1
        self.phase(f"pump recovery {self.recoveries}")
        here = self.actual_tcp("left") - self.offset["left"]
        start = self.target["left"].copy()
        start[0] = min(start[0], here[0])
        corner = flap_corner(L.FLAP_OPEN)
        side_hi = flap_corner(L.FLAP_OPEN, SIDE_Y) + (CLEAR, 0., .02)
        up = np.array((start[0], start[1], corner[2] + .025))
        out = np.array((start[0], SIDE_Y, corner[2] + .025))
        self.bias = 0.
        self.deeper = min(self.deeper + RETRY_DEEPER, 2 * RETRY_DEEPER)
        for a, b, n in ((start, up, 8), (up, out, 12), (out, side_hi, 12), (side_hi, side_hi - (0., 0., .02), 8),
                        (side_hi - (0., 0., .02), corner + (CLEAR, 0., 0.), 16)):
            for k in range(1, n + 1):
                yield a + (b - a) * k / n

    def pump_track(self):
        wait = flap_corner(L.FLAP_OPEN) + (CLEAR, 0., 0.)
        while not self.stop_pumping:
            if self.volume() >= PUMP_BELOW * L.RESERVOIR_M3:
                self.bias *= .8
                yield wait + self.lead()
                continue
            start, theta_d = self.tick, min(self.flap(), L.FLAP_OPEN)
            lost = False
            # close in from the waiting place until the finger meets the edge, then push the board shut
            while True:
                theta = self.flap()
                theta_d = max(theta_d - PUSH_RATE * DT, TH_LOW)
                lag = max(0., flap_corner(theta)[0] - flap_corner(theta_d)[0])
                self.bias = float(np.clip(self.bias + BIAS_GAIN * lag, 0., BIAS_MAX))
                target = flap_corner(theta_d) + self.lead()
                yield target
                if self.slipped():
                    lost = True
                    break
                if self.flap() <= TH_LOW + .01 or self.tick - start > 3 * CONTROL_HZ:
                    break
            self.log["strokes"].append([start, self.tick, round(float(self.flap()), 3),
                                        round(1000 * self.volume(), 3), round(1000 * self.bias, 1), lost])
            if lost or self.flap() > L.FLAP_OPEN - .03:
                if self.recoveries < MAX_RECOVERIES:
                    yield from self.recover()
                    continue
            # back off; the board's spring reopens it against the finger
            theta_d = self.flap()
            while theta_d < L.FLAP_OPEN:
                theta_d = min(theta_d + RETURN_RATE * DT, L.FLAP_OPEN)
                self.bias *= .7
                yield flap_corner(theta_d) + (CLEAR * (theta_d >= L.FLAP_OPEN), 0., 0.) + self.lead()
        self.bias = 0.
        while True:
            yield wait

    def key_rotation(self, roll):
        """The key hand: tipped KEY_TILT along the line from its base, turned KEY_YAW about the vertical, then rolled
        ``roll`` about the horizontal line across its pad edge (positive: the edge's -y end up)."""
        r0 = axis_angle_matrix(np.array((0., 0., 1.)), KEY_YAW) @ tipped("right", key_top("D4"), KEY_TILT)
        across = np.cross(r0[:, 1], (0., 0., 1.))
        return axis_angle_matrix(across / np.linalg.norm(across), roll) @ r0

    def key_pose(self, twist):
        """The key hand's rotation with joint 6 twisted ``twist`` from the reference pose (about the hand's axis)."""
        r0 = self.key_rotation(0.)
        return axis_angle_matrix(r0[:, 2], twist) @ r0

    def edge_ends(self, twist):
        """The pad tip's end edge's two ends relative to the TCP: (plus end, minus end), for a twist."""
        r = self.key_pose(twist)
        mid = KEY_TIP * r[:, 2] - FACE * r[:, 0]
        return mid + EDGE * r[:, 1], mid - EDGE * r[:, 1]

    def key_track(self):
        """Yields the key hand's TCP each tick and sets ``self.twist``.

        A note is held at a straddle: the TCP placed so that the edge's lower end is 4.7 mm inside the held key and
        its raised end over the next key. Twisting joint 6 from -TWIST to +TWIST swaps which end is low: the next
        key goes down as the held one comes up (legato). Two legs then carry the edge across the new key to the
        next straddle: to its middle with the edge level (both ends on that key), then on with the twist back."""
        top = {name: key_top(name) for name in PHRASE}
        z0 = top[PHRASE[0]][2]
        x = top[PHRASE[0]][0] - .007
        (pa, ma), (pb, mb) = self.edge_ends(-TWIST), self.edge_ends(TWIST)
        low_a = ma if ma[2] < pa[2] else pa                 # the low end at -TWIST (on the +y side)
        low_b = pb if pb[2] < mb[2] else mb                 # the low end at +TWIST (on the -y side)
        offset = (low_a[1] + low_b[1]) / 2                  # straddle: the boundary halfway between the two
        p0, m0 = self.edge_ends(0.)
        centre = (p0[1] + m0[1]) / 2
        self.log["edge_geometry_mm"] = {"low_ends_apart": round(1000 * (low_a[1] - low_b[1]), 2),
                                        "height_difference": round(1000 * abs(pa[2] - ma[2]), 2)}

        def boundary(a, b):
            return (top[a][1] + top[b][1]) / 2

        def pose(y_tcp, twist, held):
            """TCP that puts the edge's lower end KEY_DEPTH into the held key (servoed on its measured depth)."""
            if held is not None and self.state is not None:
                short = KEY_DEPTH - self.depth(held)
                gain = DEPTH_GAINS[0] if short > 0. else DEPTH_GAINS[1]
                others = max((d for k, d in self.read("depths", {}).items() if k != held), default=0.)
                if not self.rocking and others > GUARD_DEPTH:
                    # another key is going down (the hand sits too far toward it): never press deeper now; back
                    # off a little instead (the dry run with the hand 3 mm off made a chord by pressing on)
                    short = min(short, 0.)
                    gain = DEPTH_GAINS[1]
                    self.key_dz = float(np.clip(self.key_dz + DEPTH_GAINS[1] * (others - GUARD_DEPTH), *DZ_RANGE))
                self.key_dz = float(np.clip(self.key_dz - gain * short, *DZ_RANGE))
            low = min(e[2] for e in self.edge_ends(twist))
            return np.array((x, y_tcp, z0 - KEY_DEPTH + self.key_dz - low))

        def move(y0, y1, t0, t1, seconds, held, switch_at=None, then=None, gate=True):
            steps = max(1, round(seconds * CONTROL_HZ))
            for k in range(1, steps + 1):
                u = k / steps
                s = u * u * (3. - 2. * u)
                self.twist = t0 + (t1 - t0) * s
                h = then if switch_at is not None and u >= switch_at else held
                yield pose(y0 + (y1 - y0) * s, self.twist, h)
                for _ in range(STEP_WAIT if gate else 0):    # wait for the hand to catch up with this step
                    if self.tracking_ok():
                        break
                    self.waited += 1
                    yield pose(y0 + (y1 - y0) * s, self.twist, h)

        first_y = boundary(PHRASE[0], PHRASE[1]) - offset
        self.twist = -TWIST
        hover = pose(first_y, -TWIST, None) + (0., 0., KEY_HOVER)
        while self.volume() < PRIMED * L.RESERVOIR_M3:
            yield hover
        self.phase(f"note {PHRASE[0]}")
        start = pose(first_y, -TWIST, None)
        for k in range(1, 9):
            yield hover + (start - hover) * k / 8
        first = {PHRASE[0]: self.tick}
        y = first_y
        for i, name in enumerate(PHRASE):
            last = i == len(PHRASE) - 1
            need = NOTE_S if last else NOTE_S - LEGATO[0] / 2
            while self.runs.get(name, 0) < need * CONTROL_HZ and self.tick - first[name] < 10 * CONTROL_HZ:
                yield pose(y, self.twist, name)
            self.log["notes"].append([name, first[name], self.tick, self.runs.get(name, 0)])
            if last:
                break
            nxt = PHRASE[i + 1]
            self.phase(f"note {nxt}")
            first[nxt] = self.tick
            self.rocking = True
            yield from move(y, y, -TWIST, TWIST, LEGATO[0], name, .5, nxt, gate=False)     # rock: next down, then up
            self.rocking = False
            if i + 2 < len(PHRASE):
                mid_y = top[nxt][1] - centre
                y_next = boundary(nxt, PHRASE[i + 2]) - offset
                yield from move(y, mid_y, TWIST, 0., LEGATO[1], nxt)                     # level, both ends on nxt
                yield from move(mid_y, y_next, 0., -TWIST, LEGATO[2], nxt)               # on to the next straddle
                y = y_next
        self.phase("keys released")
        self.log["key_dz_mm"] = round(1000 * self.key_dz, 2)
        here = pose(y, self.twist, None)
        up = here + (0., 0., .03 - self.key_dz)
        for k in range(1, 9):
            yield here + (up - here) * k / 8
        self.keys_done = True
        while True:
            yield up

    # -- the demonstration ------------------------------------------------------------------
    def reach(self):
        self.phase("reach")
        # the left hand comes down beside the board's left end, behind it, and slides in along the edge
        side = flap_corner(L.FLAP_OPEN, SIDE_Y) + (CLEAR, 0., 0.)
        self.target["left"] = side + (0., 0., .02)
        self.target["right"] = key_top(PHRASE[0]) + (0., 0., .03)
        goals = {arm: self.solve(arm, self.target[arm], fresh=True) for arm in ARMS}
        yield from self.glide_joints(3.0, goals, dict.fromkeys(ARMS, 0.))
        for arm in ARMS:
            yield from self.trim(arm)
        self.phase("into place")
        right_to = key_top(PHRASE[0]) + (0., 0., KEY_HOVER)
        left_from, right_from = self.target["left"].copy(), self.target["right"].copy()
        for k in range(1, 11):
            self.target["left"] = left_from + (side - left_from) * k / 10
            self.target["right"] = right_from + (right_to - right_from) * k / 10
            for arm in ARMS:
                self.joints[arm] = self.solve(arm, self.target[arm])
            yield self.emit()
        wait = flap_corner(L.FLAP_OPEN) + (CLEAR, 0., 0.)
        for k in range(1, 21):
            self.target["left"] = side + (wait - side) * k / 20
            self.joints["left"] = self.solve("left", self.target["left"])
            yield self.emit()
        for arm in ARMS:
            yield from self.trim(arm)

    def play(self):
        self.phase("pump and play")
        pump, keys = self.pump_track(), self.key_track()
        while True:
            sounding = self.sounding()
            for name in PHRASE:
                self.runs[name] = self.runs.get(name, 0) + 1 if name in sounding else 0
            for arm, track in (("left", pump), ("right", keys)):
                self.target[arm] = next(track)
                if arm == "right":
                    self.rotation["right"] = self.key_pose(self.twist)
                    self.servo_xy()
                self.joints[arm] = self.solve(arm, self.target[arm])
            yield self.emit()
            if self.keys_done:
                self.stop_pumping = True
                if self.flap() >= L.FLAP_OPEN - .02:
                    break

    def let_go(self):
        self.phase("let go")
        # the left hand slides out past the board's left end the way it came in, then both rise
        start = {arm: self.target[arm].copy() for arm in ARMS}
        out = flap_corner(L.FLAP_OPEN, SIDE_Y) + (CLEAR, 0., 0.)
        for k in range(1, 13):
            self.target["left"] = start["left"] + (out - start["left"]) * k / 12
            self.target["right"] = start["right"] + np.array((0., 0., .04)) * k / 12
            for arm in ARMS:
                self.joints[arm] = self.solve(arm, self.target[arm])
            yield self.emit()
        for k in range(1, 7):
            self.target["left"] = out + np.array((0., 0., .02)) * k / 6
            self.joints["left"] = self.solve("left", self.target["left"])
            yield self.emit()
        # run 2: opening the fingers during the swing to ready brushed the right hand against the case (50 N):
        # swing closed, open at ready
        yield from self.glide_joints(2.5, {arm: np.asarray(READY, dtype=float) for arm in ARMS})
        yield from self.glide_joints(.6, {}, dict.fromkeys(ARMS, OPEN))
        while True:
            yield from self.hold(1.)

    def sequence(self):
        yield from self.reach()
        yield from self.play()
        yield from self.let_go()
