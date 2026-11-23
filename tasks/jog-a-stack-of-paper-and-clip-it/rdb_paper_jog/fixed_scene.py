"""Fixed scene: a skewed stack of 36 sheets on a riser block and a binder clip on its stand."""

from pathlib import Path

import numpy as np

from . import paper_model as model

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only NumPy
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

USD = Path(__file__).with_name("paper_jog.usda")
CONTROL_HZ = 20
PHYSICS_HZ = model.PHYSICS_HZ
MAX_STEPS = 2400
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table
ACTORS = model.ACTORS

INSTRUCTION = (
    "Square up the stack of paper so that all its edges are flush, and fasten it with the binder clip on the "
    "middle of one short edge. Then let go and move both arms away."
)


def side_camera():
    """Review camera: beyond the block and to the robot's left, looking back across the block and the jogging
    area, so the stack is seen on the block, standing on the table with both hands on its top edge, and from
    its free short edge where the clip goes on. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.92, .56, TOP + .66))
    back = eye - (.36, -.06, TOP + .13)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(18.))
