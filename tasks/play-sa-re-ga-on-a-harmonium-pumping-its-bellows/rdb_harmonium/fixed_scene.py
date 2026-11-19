"""Fixed scene: one hand-pumped harmonium on the calibrated dual-arm YAM station, keyboard toward the robot."""

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

USD = Path(__file__).with_name("harmonium.usda")
CONTROL_HZ = 20
MAX_STEPS = 2400
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table

FRONT_X = .140                                  # the case's front edge, 140 mm in front of the arm bases' line
CASE_START = (FRONT_X, 0., 0.)                  # the instrument frame's origin, tabletop frame
PUMP_Y = .225                                   # the pumping hand pushes the board's top edge here (near its left end)
LINKS = ("case", "flap") + tuple(f"key_{L.prim_name(name)}" for name in L.KEY_NAMES)
FLAP_JOINT = "flap_hinge"
KEY_JOINTS = tuple(f"key_{L.prim_name(name)}_slide" for name in L.KEY_NAMES)
JOINTS = (FLAP_JOINT,) + KEY_JOINTS


def to_station(point):
    """A point of the instrument frame (x, y, z) in the tabletop frame, with the case on its marks."""
    return np.asarray(CASE_START, dtype=float) + np.asarray(point, dtype=float)


def key_top(name, depth=0.):
    """Middle of a key's playing point (22 mm behind a white key's front end) on its top face, tabletop frame."""
    x, y, top, sx, _, _, _, white = L.key_box(L.key_by_name(name))
    front = x - sx / 2
    return to_station((front + (L.PLAY_FROM_FRONT if white else .015), y, top - depth))


def flap_corner(theta, y=PUMP_Y):
    """The bellows board's outer top edge at ``y``, tabletop frame."""
    x, z = L.flap_corner(theta)
    return to_station((x, y, z))


INSTRUCTION = (
    "Play the three marked keys on the harmonium in order, Sa (yellow dot), Re (green dot), Ga (blue dot), each "
    "held down for at least 4 seconds, going from one to the next without a gap in the sound. Pump the bellows at "
    "the back with your other hand the whole time so that the notes never die away. Then let go of the keys and "
    "the bellows."
)


def side_camera():
    """Review camera: to the robot's left of the harmonium and above it, looking across the keyboard at the
    bellows and both hands. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    target = np.array((FRONT_X + .17, 0., TOP + .14))
    eye = np.array((FRONT_X - .25, .62, TOP + .55))
    back = eye - target
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(18.))
