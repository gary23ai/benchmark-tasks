"""A stand-in world for dry runs of the demonstration and of the scorer, with a one-dimensional model of the glass
along the tube's axis. NumPy only.

Arms: each follows its joint commands with a first-order lag plus a constant joint offset (sag). A hand closed on
its part (left: the tube, right: the pestle's handle) carries it: vertically the hand is a 0.4 kg point on a
spring-damper (600 N/m, 0.7 of critical damping) toward where its joints put it, so the liquid's drag deflects it
as the soft real arm is deflected; across, the part follows the hand within the play the tube allows.
Glass, along the axis only: the tube (12.4 g with its sample) stands on the well's floor (a one-sided contact);
the pestle (20.8 g) rests on the bore's bottom when pushed down (one-sided); the liquid law of
``layout.fluid_force`` (implicit in time, so that the light parts cannot ring) acts between them, its damping
scaled by the head's share under the surface; gravity on both. A tube that rises out of its well falls over (the
sample is spilled). This checks reach, joint limits, timing, the control flow and the scorer's rules, and shows
whether an unheld tube rides up with the pestle; it says nothing about finger contact or tilting.
"""

import numpy as np

from rdb_dounce import fixed_scene as F
from rdb_dounce import layout as L

OPEN = .04695
G = 9.81
HAND_KG, ARM_K = .4, 600.
ARM_B = .7 * 2 * np.sqrt(ARM_K * HAND_KG)
SUB = 120                                         # substeps per control step (2,400 Hz)


class World:
    def __init__(self, ik, *, lag_s=.12, sag=(0., .02, 0., .05, 0., 0.), drag=None, clearance=None):
        from yam_arena.constants import READY

        self.ik = ik
        self.blend = 1 - np.exp(-(1 / F.CONTROL_HZ) / lag_s)
        self.sag = np.asarray(sag, dtype=float)
        self.drag = L.DRAG if drag is None else drag
        self.q = {arm: np.asarray(READY, dtype=float) + self.sag for arm in F.ARMS}
        self.finger = dict.fromkeys(F.ARMS, OPEN)
        self.zt, self.vt = float(L.SEAT_Z), 0.              # tube frame origin height, speed
        self.zp, self.vp = float(F.PESTLE_START[2]), 0.     # pestle frame origin (head's bottom)
        self.held = dict.fromkeys(F.ARMS, False)
        self.attach = dict.fromkeys(F.ARMS, 0.)             # part height minus hand height when gripped
        self.zh = {arm: None for arm in F.ARMS}             # hand heights (when holding)
        self.vh = dict.fromkeys(F.ARMS, 0.)
        self.spilled = False
        self.clearance = clearance
        self.grade = None
        self.tick = 0
        self.report = {"worst_limit_margin_deg": 180., "peak_tube_lift_mm": 0., "peak_fluid_n": 0.,
                       "peak_hand_force_n": dict.fromkeys(F.ARMS, 0.), "grips": [], "min_arm_gap_mm": np.inf,
                       "min_gap_block_mm": np.inf}

    # -- what the demonstration and the scorer read ----------------------------------------------------------
    def joint_pos(self):
        out = []
        for arm in F.ARMS:
            shown = self.finger[arm]
            if self.held[arm]:
                shown = max(shown, (L.TUBE_OD if arm == "left" else L.KNOB_D) / 2)
            out.append(np.r_[self.q[arm], shown / OPEN])
        return np.concatenate(out)

    def hand(self, arm):
        p, r = self.ik.tcp_pose(self.q[arm], F.BASES[arm])
        if self.held[arm] and self.zh[arm] is not None:
            p = p.copy()
            p[2] = self.zh[arm]
        return p, r

    def tube_rot(self):
        if not self.spilled:
            return np.eye(3)
        return np.array(((1., 0., 0.), (0., 0., -1.), (0., 1., 0.)))

    def state(self):
        tube = np.array((*F.AXIS_XY, self.zt))
        pestle = np.array((*F.AXIS_XY, self.zp))
        return {"tube_pos": tube, "tube_rot": self.tube_rot(), "pestle_pos": pestle, "pestle_rot": np.eye(3),
                "depth": self.zp - self.zt - L.BOTTOM_T, "tube_lift": self.zt - L.SEAT_Z, "grade": self.grade}

    # -- one control step ----------------------------------------------------------------------------------
    def step(self, command):
        self.tick += 1
        command = np.asarray(command, dtype=float)
        for i, arm in enumerate(F.ARMS):
            q = command[7 * i:7 * i + 6]
            if np.any(q < self.ik.LOWER - 1e-6) or np.any(q > self.ik.UPPER + 1e-6):
                raise RuntimeError(f"{arm} arm commanded past a joint limit at tick {self.tick}: {q}")
            self.report["worst_limit_margin_deg"] = min(self.report["worst_limit_margin_deg"], float(
                np.degrees(np.minimum(q - self.ik.LOWER, self.ik.UPPER - q).min())))
            self.q[arm] = self.q[arm] + self.blend * (q + self.sag - self.q[arm])
            self.finger[arm] = self.finger[arm] + .6 * (command[7 * i + 6] * OPEN - self.finger[arm])
        self._grips()
        self._physics()
        if self.clearance is not None and self.tick % 4 == 0:
            self._clearance()

    def _grips(self):
        for arm, width in (("left", L.TUBE_OD), ("right", L.KNOB_D)):
            p, _ = self.ik.tcp_pose(self.q[arm], F.BASES[arm])
            if arm == "left":
                on = (np.linalg.norm(p[:2] - F.AXIS_XY) < .004 and self.zt + L.WELL_DEPTH + .010 < p[2] < self.zt + L.TUBE_L - .008)
            else:
                knob = self.zp + L.KNOB_Z
                on = np.linalg.norm(p[:2] - F.AXIS_XY) < .004 and abs(p[2] - knob) < L.KNOB_L / 2 - .006
            closed = self.finger[arm] < width / 2 + .0003
            if not self.held[arm] and closed and on and not self.spilled:
                self.held[arm] = True
                part = self.zt if arm == "left" else self.zp
                self.zh[arm], self.vh[arm] = float(p[2]), 0.
                self.attach[arm] = part - p[2]
                self.report["grips"].append([self.tick, arm])
            elif self.held[arm] and self.finger[arm] > width / 2 + .0015:
                self.held[arm] = False
                self.zh[arm] = None

    def _physics(self):
        dt = 1. / F.CONTROL_HZ / SUB
        cmd = {arm: float(self.ik.tcp_pose(self.q[arm], F.BASES[arm])[0][2]) for arm in F.ARMS}
        for _ in range(SUB):
            m_t = L.TUBE_KG + (HAND_KG if self.held["left"] else 0.)
            m_p = L.PESTLE_KG + (HAND_KG if self.held["right"] else 0.)
            f_t = -L.TUBE_KG * G
            f_p = -L.PESTLE_KG * G
            for arm, sign in (("left", "t"), ("right", "p")):
                if self.held[arm]:
                    v = self.vt if sign == "t" else self.vp
                    z = (self.zt if sign == "t" else self.zp) - self.attach[arm]
                    f = ARM_K * (cmd[arm] - z) - ARM_B * v       # the hand's own weight is carried by its joints
                    if sign == "t":
                        f_t += f
                    else:
                        f_p += f
                    self.report["peak_hand_force_n"][arm] = max(self.report["peak_hand_force_n"][arm], abs(f))
            depth = self.zp - self.zt - L.BOTTOM_T
            c = self.drag * L.submerged_share(depth) if not self.spilled else 0.
            # implicit drag: solve for the new speeds with the drag evaluated at them
            a = np.array(((m_p + dt * c, -dt * c), (-dt * c, m_t + dt * c)))
            b = np.array((m_p * self.vp + dt * f_p, m_t * self.vt + dt * f_t))
            vp, vt = np.linalg.solve(a, b)
            fluid = -c * (vp - vt)
            if abs(fluid) > L.SUCTION_CAP:                  # the suction is capped: the liquid boils
                fluid = float(np.sign(fluid) * L.SUCTION_CAP)
                vp = self.vp + dt * (f_p + fluid) / m_p
                vt = self.vt + dt * (f_t - fluid) / m_t
            self.report["peak_fluid_n"] = max(self.report["peak_fluid_n"], abs(fluid))
            # one-sided contacts: tube on the well's floor; head on the bore's bottom
            if self.zt <= L.SEAT_Z + 1e-9 and vt < 0. and not self.spilled:
                vt = 0.
                vp = (m_p * self.vp + dt * f_p) / (m_p + dt * c)        # implicit drag against a standing tube
            if depth <= 0. and vp - vt < 0.:
                v = (m_p * vp + m_t * vt) / (m_p + m_t)
                vp = vt = v if not (self.zt <= L.SEAT_Z + 1e-9 and v < 0.) else 0.
            self.vp, self.vt = float(vp), float(vt)
            self.zp += self.vp * dt
            self.zt += self.vt * dt
            if self.zt < L.SEAT_Z:
                self.zt = float(L.SEAT_Z)
            if self.zp - self.zt - L.BOTTOM_T < 0.:
                self.zp = self.zt + L.BOTTOM_T
        for arm, part in (("left", self.zt), ("right", self.zp)):
            if self.held[arm]:
                self.zh[arm] = part - self.attach[arm]
        self.report["peak_tube_lift_mm"] = max(self.report["peak_tube_lift_mm"], 1000 * (self.zt - L.SEAT_Z))
        if self.zt - L.SEAT_Z > L.WELL_DEPTH and not self.spilled:
            self.spilled = True                          # out of its well, the tube falls over
            self.held = dict.fromkeys(F.ARMS, False)

    def _clearance(self):
        c = self.clearance
        clouds = {arm: c.arm_cloud(self.q[arm], self.finger[arm] / OPEN, F.BASES[arm]) for arm in F.ARMS}
        self.report["min_arm_gap_mm"] = min(self.report["min_arm_gap_mm"], 1000 * c.arm_gap(clouds["left"], clouds["right"]))
        bx, by, bz = L.BLOCK
        centre = F.BLOCK_CENTRE
        for arm in F.ARMS:
            self.report["min_gap_block_mm"] = min(self.report["min_gap_block_mm"],
                                                  1000 * c.box_gap(clouds[arm], centre, (bx / 2, by / 2, bz / 2)))
