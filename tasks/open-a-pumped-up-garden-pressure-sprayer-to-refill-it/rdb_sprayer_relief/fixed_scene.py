"""Fixed scene: a pumped-up 2 litre pressure sprayer standing on a rubber mat on the calibrated dual-arm YAM
station. Tabletop frame: x forward from the line joining the arm bases, y to the robot's left, z up from the
table top; metres."""

import os
from pathlib import Path

import numpy as np

from . import layout as L

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only NumPy
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

USD = Path(__file__).with_name("sprayer.usda")
CONTROL_HZ = 20
MAX_STEPS = 5000
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table

MARK = np.array((.300, 0.))                      # the sprayer's mark: the middle of the mat
ORIGIN = np.array((MARK[0], MARK[1], L.MAT[2]))  # the sprayer frame's origin: base centre, on the mat

LINKS = ("tank", "head", "knob", "valve")      # one articulation rooted at the tank (it stands loose)
LINK_MASSES = (L.TANK_MASS, L.HEAD_MASS, L.KNOB_MASS, L.VALVE_MASS)
JOINTS = ("head", "knob", "valve")     # head: 0..1.5 turns; knob: free; valve: 0..5 mm

# The official scene starts pumped up. START_BAR=0 in the environment gives the control scene (already vented).
START_BAR = float(os.environ.get("RDB_SPRAYER_START_BAR", L.START_BAR))

INSTRUCTION = "The sprayer has run low. Unscrew its spray head all the way so it can be refilled, then let go."


def to_station(point):
    return ORIGIN + np.asarray(point, dtype=float)


def side_camera():
    """Review camera: in front of the sprayer, a little to the robot's right and above its shoulder, so that the valve,
    the spray head and both grippers are in view. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((ORIGIN[0] - .42, ORIGIN[1] - .12, TOP + .36))
    back = eye - (ORIGIN[0], ORIGIN[1], TOP + .21)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(22.))
