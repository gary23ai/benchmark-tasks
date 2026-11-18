"""A world without contact physics for dry runs of the demonstration and of the scorer. NumPy only.

The arms follow their joint commands with a first-order lag plus a constant joint offset (sag). The cue lies on
the rail until fingers close on it. Held by two pinches it lies on the line through the two pinch points; held
by the right hand with the left hand round it (open, or closed to the loop) it lies on the line through the
right pinch and the left loop's ceiling, sliding through the loop; held by one hand only it droops 20 degrees
tip-up about the grip (the cue is front-light in front of that grip). The white ball is struck when the tip
reaches its surface moving toward it: it leaves at 1.31 times the cue's speed along the axis (leather tip
e = 0.72, cue 0.539 kg, ball 0.163 kg). Balls slide and roll on the cloth (sliding friction 0.2, rolling
resistance 0.01, spin friction ignored), collide with each other (e = 0.93, no friction), bounce off cushion
noses (e = 0.75) and drop when their centre passes over a pocket hole. Jaw facings are not modelled. This checks
reach, joint limits, control flow and the scorer's logic, not the contact physics of the real simulator.
"""

import numpy as np

from rdb_pool_bridge import fixed_scene as fixture
from rdb_pool_bridge import layout as L

OPEN = .04695
G = 9.81
SLIDE_MU = .2
SUBSTEPS = 12
from rdb_pool_bridge.scripted import PINCH_DEPTH  # noqa: E402


class Balls:
    def __init__(self):
        self.p = {n: L.BALL_START[n][:2].copy() for n in L.BALLS}
        self.v = {n: np.zeros(2) for n in L.BALLS}
        self.w = {n: np.zeros(2) for n in L.BALLS}      # rolling velocity R * omega (horizontal)
        self.potted = {}
        self.hits = []                                    # (step, name) of the white's ball contacts

    def position(self, n):
        if n in self.potted:
            h = L.POCKETS[self.potted[n]]["hole"]
            return np.r_[h, L.BED_Z - L.POCKET_DEPTH + L.BALL_R]
        return np.r_[self.p[n], L.BALL_Z]

    def velocity(self, n):
        return np.r_[self.v[n], 0.] if n not in self.potted else np.zeros(3)

    def step(self, dt, events):
        live = [n for n in L.BALLS if n not in self.potted]
        for n in live:
            slip = self.v[n] - self.w[n]
            s = np.linalg.norm(slip)
            if s > 1e-3:                                  # sliding: friction slows the ball and spins it up
                d = slip / s
                dv = min(SLIDE_MU * G * dt, s * 2 / 7)
                self.v[n] -= d * dv
                self.w[n] += d * dv * 2.5
            else:                                         # rolling: rolling resistance
                sp = np.linalg.norm(self.v[n])
                if sp > 0:
                    self.v[n] *= max(0., sp - .01 * G * dt) / sp
                self.w[n] = self.v[n].copy()
            self.p[n] = self.p[n] + self.v[n] * dt
        for i, a in enumerate(live):
            for b in live[i + 1:]:
                d = self.p[b] - self.p[a]
                dist = np.linalg.norm(d)
                if dist < 2 * L.BALL_R and dist > 0:
                    nrm = d / dist
                    rel = (self.v[a] - self.v[b]) @ nrm
                    if rel > 0:
                        j = (1 + .93) / 2 * rel
                        self.v[a] -= j * nrm
                        self.v[b] += j * nrm
                        if "cue" in (a, b):
                            other = b if a == "cue" else a
                            events["cue_ball_hits"] = events["cue_ball_hits"] + (other,)
        for n in live:
            for name, pk in L.POCKETS.items():
                if np.linalg.norm(self.p[n] - pk["hole"]) < pk["hole_r"]:
                    self.potted[n] = name
            if n in self.potted:
                continue
            x, y = self.p[n]
            for lo, hi, axis, line, sign in self._rails():
                coord = (x, y)[axis]
                along = (y, x)[axis]
                if lo <= along <= hi and sign * (coord - line) < L.BALL_R:
                    if sign * self.v[n][axis] < 0:
                        self.v[n][axis] *= -.75
                        self.w[n][axis] *= -.5

    @staticmethod
    def _rails():
        p = L.POCKETS
        ny0, ny1 = p["near_right"]["jaws"][0][1], p["near_left"]["jaws"][0][1]
        side = (p["near_side"]["jaws"][1][1], p["near_side"]["jaws"][0][1])
        lx0, lx1 = p["near_left"]["jaws"][1][0], p["far_left"]["jaws"][1][0]
        return [(ny0, side[0], 0, L.X_N, 1.), (side[1], ny1, 0, L.X_N, 1.), (ny0, side[0], 0, L.X_F, -1.),
                (side[1], ny1, 0, L.X_F, -1.), (lx0, lx1, 1, L.Y_L, -1.), (lx0, lx1, 1, L.Y_R, 1.)]


class World:
    def __init__(self, ik, *, lag_s=.12, sag=(0., .02, 0., .05, 0., 0.), speed_scale=1., swerve=0., clearance=None):
        from yam_arena.constants import READY

        self.ik = ik
        self.blend = 1 - np.exp(-(1 / fixture.CONTROL_HZ) / lag_s)
        self.sag = np.asarray(sag, dtype=float)
        self.speed_scale = speed_scale
        self.swerve = swerve                              # right hand swings sideways by this many m per m/s
        self.swing = np.zeros(3)
        self.q_nominal = np.asarray(READY, dtype=float) + self.sag
        self.q = {arm: np.asarray(READY, dtype=float) + self.sag for arm in fixture.ARMS}
        self.finger = dict.fromkeys(fixture.ARMS, OPEN)
        self.finger_cmd = dict.fromkeys(fixture.ARMS, OPEN)
        self.tip = L.CUE_REST_TIP.copy()
        self.axis = L.CUE_REST_AXIS.copy()
        self.tip_velocity = np.zeros(3)
        self.held = {}                                    # arm -> s along the cue where it pinches
        self.balls = Balls()
        self.clearance = clearance
        self.separated = False
        self.struck = 0
        self.tick = 0
        self.report = {"worst_limit_margin_deg": 180., "min_gap_table_mm": np.inf, "min_gap_arms_mm": np.inf,
                       "grasped": {}, "strike_tick": None, "cue_speed_at_strike": None, "white_speed": None}

    # -- what the demonstration and the scorer read ------------------------------------------------------------
    def joint_pos(self):
        return np.concatenate([np.r_[self.q[a], self.finger[a] / OPEN] for a in fixture.ARMS])

    def hand(self, arm):
        return self.ik.tcp_pose(self.q[arm], fixture.BASES[arm])

    def state(self):
        return {"cue_tip": self.tip.copy(), "cue_axis": self.axis.copy(), "cue_velocity": self.tip_velocity.copy(),
                "balls": {n: self.balls.position(n) for n in L.BALLS}, "struck": self.struck > 0}

    def sample(self):
        return {"balls": {n: (self.balls.position(n), self.balls.velocity(n)) for n in L.BALLS},
                "cue_tip": self.tip.copy(), "cue_axis": self.axis.copy(), "cue_velocity": self.tip_velocity.copy(),
                "tcp": {a: self.hand(a) for a in fixture.ARMS},
                "opening": {a: 2 * self.finger[a] for a in fixture.ARMS}}

    # -- one control step ---------------------------------------------------------------------------------------
    def step(self, command):
        from rdb_pool_bridge.grading import EMPTY_EVENTS

        self.tick += 1
        events = {k: (() if isinstance(v, tuple) else v) for k, v in EMPTY_EVENTS.items()}
        command = np.asarray(command, dtype=float)
        for i, arm in enumerate(fixture.ARMS):
            q = command[7 * i:7 * i + 6]
            if np.any(q < self.ik.LOWER - 1e-6) or np.any(q > self.ik.UPPER + 1e-6):
                raise RuntimeError(f"{arm} arm commanded past a joint limit at tick {self.tick}: {q}")
            margin = float(np.degrees(np.minimum(q - self.ik.LOWER, self.ik.UPPER - q).min()))
            if margin < self.report["worst_limit_margin_deg"]:
                self.report["worst_limit_margin_deg"] = margin
                self.report["tick_of_worst_margin"] = (self.tick, arm)
            if arm == "right" and self.swerve:              # what run 2 showed: the hand swings +x at speed,
                before = self.ik.tcp_pose(self.q_nominal, fixture.BASES[arm])[0]    # in the joints themselves
                self.q_nominal = self.q_nominal + self.blend * (q + self.sag - self.q_nominal)
                p, r = self.ik.tcp_pose(self.q_nominal, fixture.BASES[arm])
                v = (p - before) * fixture.CONTROL_HZ
                self.swing = np.array((self.swerve * max(0., v[1]), 0., 0.))
                self.q[arm], _, _ = self.ik.solve(p + self.swing, self.ik.matrix_to_quat_xyzw(r), self.q_nominal,
                                                  fixture.BASES[arm], iters=30)
            else:
                self.q[arm] = self.q[arm] + self.blend * (q + self.sag - self.q[arm])
            if arm == "left":                             # the cloth stops the fingertips
                p, r = self.hand(arm)
                below = L.BED_Z - (p + L.PAD_TIP_PAST_TCP * r[:, 2])[2]
                if below > 0:
                    self.q[arm], _, _ = self.ik.solve(p + (0., 0., below), self.ik.matrix_to_quat_xyzw(r), self.q[arm],
                                                      fixture.BASES[arm], iters=30)
            self.finger_cmd[arm] = command[7 * i + 6] * OPEN
        tip_before = self.tip.copy()
        self._grip()
        self._place_cue()
        events["left_on_cue"] = SUBSTEPS if self.touching else 0
        self.tip_velocity = (self.tip - tip_before) * fixture.CONTROL_HZ
        self._balls(events, tip_before)
        if self.clearance is not None and self.tick % 3 == 0:
            self._gaps()
        return events

    def _pinch_point(self, arm):
        p, r = self.hand(arm)
        return p + PINCH_DEPTH[arm] * r[:, 2]

    def _grip(self):
        for arm in fixture.ARMS:
            point = self._pinch_point(arm)
            s = float(-(point - self.tip) @ self.axis)
            width = L.cue_diameter(s)
            off = np.linalg.norm((point - self.tip) + s * self.axis)
            want = self.finger_cmd[arm]
            if arm in self.held:
                self.finger[arm] = max(want, width / 2)
                if want > width / 2 + .0005:
                    del self.held[arm]
            else:
                closing = want < width / 2 and self.finger[arm] >= width / 2 - .0005
                if closing and off < .006 and 0 < s < L.CUE_L:
                    self.held[arm] = s
                    self.report["grasped"][arm] = self.tick
                    self.finger[arm] = width / 2
                else:
                    self.finger[arm] = self.finger[arm] + .6 * (want - self.finger[arm])

    def _loop_height(self, arm):
        """For the left hand around the shaft: the loop's ceiling point if the shaft is between its fingers
        above the pads, else None."""
        if arm != "left":
            return None
        p, r = self.hand(arm)
        ceiling = p - (L.LOOP_CEILING - L.PAD_TIP_PAST_TCP - .0065) * r[:, 2]
        s = float(-(ceiling - self.tip) @ self.axis)
        rel = ceiling - (self.tip - s * self.axis)
        if abs(rel @ r[:, 0]) < .012 and -.07 < rel @ r[:, 2] < .03 and abs(rel @ r[:, 1]) < .04:
            return ceiling
        return None

    def _place_cue(self):
        self.touching = "left" in self.held
        if "right" in self.held and "left" in self.held:
            pl, pr = self._pinch_point("left"), self._pinch_point("right")
            self.axis = (pl - pr) / np.linalg.norm(pl - pr)
            self.tip = pr + self.held["right"] * self.axis
        elif "right" in self.held:
            pr = self._pinch_point("right")
            ceiling = self._loop_height("left")
            if ceiling is not None:
                self.axis = (ceiling - pr) / np.linalg.norm(ceiling - pr)
                self.touching = True
            else:                                         # front-light cue on one pinch: it tips up
                level = np.array((self.axis[0], self.axis[1], 0.))
                level /= np.linalg.norm(level)
                self.axis = np.cos(np.radians(20.)) * level + np.sin(np.radians(20.)) * np.array((0., 0., 1.))
            self.tip = pr + self.held["right"] * self.axis
        elif "left" in self.held:
            pl = self._pinch_point("left")
            self.tip = pl + self.held["left"] * self.axis

    def _balls(self, events, tip_before):
        white = self.balls.position("cue")
        speed_along = float(self.tip_velocity @ self.axis)
        dist = np.linalg.norm(self.tip - white) - L.BALL_R
        # the tip moves up to 40 mm in a control step: test the whole segment it swept, not just its end
        seg = self.tip - tip_before
        t = float(np.clip((white - tip_before) @ seg / max(seg @ seg, 1e-12), 0., 1.))
        swept = np.linalg.norm(tip_before + t * seg - white) - L.BALL_R
        if "cue" not in self.balls.potted and min(dist, swept) <= .0005 and speed_along > 0 and (white - tip_before) @ self.axis > 0:
            if self.struck == 0 or self.separated:
                self.struck += 1
                events["tip_cue_ball"] = 1
                events["impulse_ns"] = L.BALL_MASS * 1.31 * speed_along * self.speed_scale
                # the white leaves along the line from the tip's contact point through its centre: an
                # off-centre hit sends it off the cue's line (as in run 2: tip 10 mm left, white 10 deg right)
                f = tip_before - white                    # first point of the swept segment on the ball's surface
                a, b, c = seg @ seg, 2 * f @ seg, f @ f - L.BALL_R ** 2
                disc = b * b - 4 * a * c
                t0 = (-b - np.sqrt(max(disc, 0.))) / (2 * a) if a > 1e-12 else 0.
                hit = tip_before + float(np.clip(t0, 0., 1.)) * seg
                horiz = (white - hit)[:2]
                horiz /= np.linalg.norm(horiz)
                self.balls.v["cue"] = horiz * 1.31 * speed_along * self.speed_scale * np.hypot(self.axis[0], self.axis[1]) \
                    * float(np.clip(horiz @ (self.axis[:2] / np.linalg.norm(self.axis[:2])), 0., 1.))
                self.balls.w["cue"] = np.zeros(2)
                self.separated = False
                if self.report["strike_tick"] is None:
                    self.report.update(strike_tick=self.tick, cue_speed_at_strike=round(speed_along, 3),
                                       white_speed=round(float(np.linalg.norm(self.balls.v["cue"])), 3))
        elif dist > .005:
            self.separated = self.struck > 0
        # the cue's side or tip on the object balls, the fingers on any ball
        for n in L.BALLS:
            c = self.balls.position(n)
            s = float(np.clip(-(c - self.tip) @ self.axis, 0., L.CUE_L))
            d = np.linalg.norm(c - (self.tip - s * self.axis))
            if d < L.BALL_R + L.cue_diameter(s) / 2 - .0005:
                if s < .01:
                    if n != "cue":
                        events["tip_object_balls"] += (n,)
                else:
                    events["cue_side_balls"] += (n,)
            for arm in fixture.ARMS:
                p, r = self.hand(arm)
                tips = p + L.PAD_TIP_PAST_TCP * r[:, 2]
                if np.linalg.norm(c - tips) < L.BALL_R + .01:
                    events["robot_balls"] += (n,)
        for _ in range(SUBSTEPS):
            self.balls.step(1 / (fixture.CONTROL_HZ * SUBSTEPS), events)

    def _gaps(self):
        """Arm hulls above the table: rails and bed; the left fingertips may touch the cloth (the bridge)."""
        clouds = {}
        for arm in fixture.ARMS:
            links = ("gripper", "link5", "link4", "link3", "link2", "tip_left", "tip_right")
            clouds[arm] = self.clearance.arm_cloud(self.q[arm], self.finger[arm] / OPEN, fixture.BASES[arm], links=links)
        x0, x1, y0, y1 = L.OUTER
        for arm, cloud in clouds.items():
            inside = (cloud[:, 0] > x0) & (cloud[:, 0] < x1) & (cloud[:, 1] > y0) & (cloud[:, 1] < y1)
            pts = cloud[inside]
            if len(pts):
                rail = (pts[:, 0] < L.X_N) | (pts[:, 0] > L.X_F) | (pts[:, 1] > L.Y_L) | (pts[:, 1] < L.Y_R)
                surface = np.where(rail, L.RAIL_TOP, L.BED_Z - (.0015 if arm == "left" else 0.))   # the bridge touches the cloth
                gap = 1000 * float((pts[:, 2] - surface).min())
                if gap < self.report["min_gap_table_mm"]:
                    self.report["min_gap_table_mm"] = gap
                    self.report["tick_of_min_gap_table"] = (self.tick, arm)
        gap = 1000 * self.clearance.arm_gap(clouds["left"], clouds["right"])
        self.report["min_gap_arms_mm"] = min(self.report["min_gap_arms_mm"], gap)
