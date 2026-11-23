"""Robot joint targets only. No controller writes to a task object."""

import mujoco as mj
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scene import CROCK, PAN, READY, GRIP_Z, LIMIT


def targets(trial):
    m = trial.model
    kd = mj.MjData(m)
    kd.qpos[:] = trial.data.qpos
    site = m.site("left_grasp_site").id
    addrs = trial.joints["left"][:6]
    lo, hi = m.jnt_range[[m.joint(f"left_joint{i}").id for i in range(1, 7)]].T
    orientation = Rotation.from_euler("y", -35, degrees=True).as_matrix() @ np.diag(
        [-1, 1, -1]
    )
    last = READY.copy()

    def solve(point, rotation=None):
        desired = orientation if rotation is None else rotation

        def error(q):
            kd.qpos[addrs] = q
            mj.mj_kinematics(m, kd)
            R = kd.site_xmat[site].reshape(3, 3)
            return np.r_[
                kd.site_xpos[site] - point,
                0.11 * Rotation.from_matrix(desired.T @ R).as_rotvec(),
            ]

        result = least_squares(
            error,
            last,
            bounds=(lo + 0.001, hi - 0.001),
            xtol=1e-9,
            ftol=1e-9,
            gtol=1e-9,
        )
        if np.linalg.norm(error(result.x)[:3]) > 0.003:
            raise ValueError(f"unreachable {point}: {error(result.x)}")
        return result.x

    # Lift from the crock, enter behind the carrots, draw the vegetables together, lift clear.
    pickup = CROCK + [0, 0, 0.006 + GRIP_Z]
    path = [
        (26, pickup + [0, 0, 0.095], 0.95),
        (22, pickup, 0.95),
        (12, pickup, 0),
        (30, pickup + [0, 0, 0.105], 0),
        (30, PAN + [-0.063, 0, 0.252], 0),
        (32, PAN + [-0.063, 0, 0.149], 0),
        (48, PAN + [0.017, 0, 0.149], 0),
        (20, PAN + [0.017, 0, 0.245], 0),
    ]
    grip = 0.74
    t = 0
    for duration, point, g in path:
        q = solve(point)
        for j in range(1, duration + 1):
            u = j / duration
            f = u * u * (3 - 2 * u)
            yield np.array(
                [[*(last * (1 - f) + q * f), grip * (1 - f) + g * f], [*READY, 0.74]]
            )
            t += 1
        last = q
        grip = g
    # Use measured blade pose to correct for slip at the pads during the division.
    # The spoon end is never attached to the robot. Only the joint controller acts.
    tool = m.body("spatula").id
    stages = [
        (55, [0.024, 0, 0.097], -90),
        (38, [0.024, 0, 0.006], -90),
        (32, [0.024, 0.032, 0.006], -90),
        (24, [0.024, 0.032, 0.100], -90),
        (24, [0.024, 0, 0.100], -90),
        (34, [0.024, 0, 0.006], -90),
        (34, [0.024, -0.032, 0.006], -90),
        (26, [0.024, -0.032, 0.100], -90),
        (35, [0, 0.096, 0.100], -45),
    ]
    for duration, xyz, angle in stages:
        start = trial.data.xpos[tool].copy()
        R0 = Rotation.from_matrix(trial.data.xmat[tool].reshape(3, 3))
        R1 = Rotation.from_euler("z", angle, degrees=True)
        delta = (R1 * R0.inv()).as_rotvec()
        for k in range(0, duration, 3):
            steps = min(3, duration - k)
            u = (k + steps) / duration
            f = u * u * (3 - 2 * u)
            goal = start * (1 - f) + (trial.start + xyz) * f
            targetR = (Rotation.from_rotvec(delta * f) * R0).as_matrix()
            currentR = trial.data.xmat[tool].reshape(3, 3)
            siteR = trial.data.site_xmat[site].reshape(3, 3)
            relative = currentR.T @ (trial.data.site_xpos[site] - trial.data.xpos[tool])
            wrist = goal + targetR @ relative
            cmd = solve(wrist, targetR @ currentR.T @ siteR)
            for s in range(1, steps + 1):
                weight = s / steps
                yield np.array(
                    [[*(last * (1 - weight) + cmd * weight), 0.0], [*READY, 0.74]]
                )
                t += 1
            last = cmd
    while t < LIMIT:
        yield np.array([[*last, grip], [*READY, 0.74]])
        t += 1
