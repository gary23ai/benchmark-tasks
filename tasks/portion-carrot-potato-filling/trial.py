"""20 Hz position control and an outcome scorer shared by both policies."""

import mujoco as mj
import numpy as np
from scoring import grade
from scene import build, FOODS, SIDES, READY, LIMIT


class Trial:
    def __init__(self):
        self.model = m = build()
        self.data = d = mj.MjData(m)
        self.joints = {}
        for s in SIDES:
            ids = [m.joint(f"{s}_joint{i}").id for i in range(1, 9)]
            self.joints[s] = m.jnt_qposadr[ids]
            d.qpos[self.joints[s]] = [*READY, 0.035, 0.035]
            d.ctrl[[m.actuator(f"{s}_joint{i}").id for i in range(1, 8)]] = [
                *READY,
                0.035,
            ]
        for _ in range(200):
            mj.mj_step(m, d)
        self.start = self.position("pan").copy()
        self.spilled = self.bare_hand = False
        self.pan_displaced = False
        self.touched = set()
        self.step_count = 0
        self.food_geoms = {m.geom(name).id: i for i, name in enumerate(FOODS)}
        self.arm_geoms = {
            i
            for i in range(m.ngeom)
            if mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, m.geom_bodyid[i]).startswith(
                ("left_", "right_")
            )
        }
        self.blade = m.geom("blade").id

    def position(self, name):
        return self.data.xpos[self.model.body(name).id]

    def action(self):
        q = self.data.qpos
        return np.array(
            [[*q[self.joints[s][:6]], q[self.joints[s][6]] / 0.0475] for s in SIDES]
        )

    def step(self, action):
        assert self.step_count < LIMIT
        m, d = self.model, self.data
        a = np.asarray(action, dtype=float).reshape(2, 7).copy()
        if not np.isfinite(a).all():
            raise ValueError("nonfinite action")
        a[:, 6] = np.clip(a[:, 6], 0, 1) * 0.0475
        d.ctrl[:] = np.clip(
            a.ravel(), m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
        )
        for _ in range(25):
            mj.mj_step(m, d)
            if np.linalg.norm(
                self.position("pan")[:2] - self.start[:2]
            ) >= 0.025 or d.xmat[m.body("pan").id, 8] <= np.cos(np.deg2rad(10)):
                self.pan_displaced = True
            for c in d.contact:
                g, h = c.geom
                if g not in self.food_geoms and h not in self.food_geoms:
                    continue
                food, other = (g, h) if g in self.food_geoms else (h, g)
                if other == self.blade:
                    self.touched.add(self.food_geoms[food])
                if other in self.arm_geoms:
                    self.bare_hand = True
            if any(
                self.position(n)[2] < -0.001
                or np.linalg.norm(self.position(n)[:2] - self.position("pan")[:2])
                > 0.083
                for n in FOODS
            ):
                self.spilled = True
        self.step_count += 1

    def score(self):
        foods = np.array([self.position(n) for n in FOODS])
        pan = self.position("pan")
        blade = self.model.geom("blade").id
        bounds = self.model.geom_size[blade]
        corners = (
            np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
            * bounds
        )
        corners = (
            corners @ self.data.geom_xmat[blade].reshape(3, 3).T
            + self.data.geom_xpos[blade]
        )
        clear = (
            np.all(corners[:, 2] > pan[2] + 0.072)
            or np.linalg.norm(self.data.geom_xpos[blade, :2] - pan[:2]) > 0.11
        )
        stable = not self.pan_displaced
        rotations = np.array([self.data.body(n).xmat.reshape(3, 3) for n in FOODS])
        one_layer = bool(np.all(foods[:, 2] - pan[2] <= 0.015))
        result = grade(
            foods,
            rotations,
            clear,
            stable,
            self.spilled,
            self.bare_hand,
            bool(self.touched),
            one_layer,
        )
        result["blade_contacts"] = len(self.touched)
        rest = all(
            np.linalg.norm(
                self.data.qvel[self.model.jnt_dofadr[self.model.joint(n).id] :][:3]
            )
            < 0.025
            for n in FOODS
        )
        result["settled_in_pan"] = bool(rest and np.all(foods[:, 2] - pan[2] < 0.072))
        if not result["settled_in_pan"]:
            result["score"] = 0.0
        return result
