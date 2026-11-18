"""Fixed scene: one storage drawer with a child-safety latch on the calibrated dual-arm YAM station."""

from pathlib import Path

import numpy as np

from . import layout

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only NumPy
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

USD = Path(__file__).with_name("drawer.usda")
CONTROL_HZ = 20
MAX_STEPS = 1600
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table
ACTORS = layout.ACTORS
START = layout.start_poses()                    # tabletop frame
WORLD_START = {name: (x, y, z + TOP) for name, (x, y, z) in START.items()}

INSTRUCTION = (
    "Open the drawer at least 150 mm, then let go of it. A child-safety latch inside stops it after about 38 mm: "
    "hold the drawer there, reach into the gap above the drawer front with the other hand and press the latch "
    "down, pull the drawer past the catch, then open it the rest of the way."
)


def side_camera():
    """Review camera: in front of the unit on the left, above, looking down into the gap where the latch is.
    Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.10, .36, TOP + .40))
    back = eye - (layout.FRAME_FRONT_X - .02, layout.LATCH_Y, TOP + .13)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(24.))
