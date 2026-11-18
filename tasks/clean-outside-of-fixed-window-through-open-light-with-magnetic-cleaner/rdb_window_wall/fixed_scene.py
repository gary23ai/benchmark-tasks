"""A wall with a window and a double-sided magnetic window cleaner on the calibrated dual-arm YAM station."""

from pathlib import Path

import numpy as np
from yam_arena import station
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens

from . import cleaner_model as model

TASK_ID = "clean-outside-of-fixed-window-through-open-light-with-magnetic-cleaner"
SCORER = "window_wall_score"
USD = Path(__file__).with_name("window.usda")
TOP = station.TABLE_TOP
if abs(TOP - model.TABLE_TOP_WORLD_Z) > 1e-9:
    raise RuntimeError("cleaner_model.TABLE_TOP_WORLD_Z no longer matches the station's table top")
CONTROL_HZ = 20
SUCCESS_HOLD_S = 2
MAX_STEPS = 2400
ARMS = ("left", "right")
HALVES = model.HALVES  # the left arm works the indoor face; the right arm reaches out through the open light
BASES = {arm: np.asarray(station.BASES[arm]) - (0., 0., TOP) for arm in ARMS}
PARK = {"left": (.14, .34, .19), "right": (.14, -.34, .19)}
# World start poses (x, y, z, qx, qy, qz, qw) of the two halves; resets never depend on a previous trial.
WORLD_START = {half: (model.START[half][0], model.START[half][1], model.START[half][2] + TOP, *model.START_QUAT[half])
               for half in HALVES}
PANE_YAW_DEG = model.PANE_YAW_DEG
ACTORS = tuple(WORLD_START)
RESET_MASS = model.MASS
INSTRUCTION = (
    "You are indoors. The black mark is on the outside of the fixed window pane, too far along to reach from "
    "the open window beside it. Clean it off from the inside with the magnetic window cleaner, then let go and "
    "leave the cleaner holding on the glass."
)


def side_camera():
    """Review camera: outdoors, beyond the fixed light's far end, looking back along the outdoor face."""
    eye = model.to_world((.86, -.52, .42)) + (0., 0., TOP)
    back = eye - (model.to_world((.22, 0., .17)) + (0., 0., TOP))
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(20.))
