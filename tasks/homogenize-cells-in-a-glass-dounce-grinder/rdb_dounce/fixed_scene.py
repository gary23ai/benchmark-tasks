"""Fixed scene: a glass Dounce grinder standing in an aluminium cold block on a lab jack, on the calibrated
dual-arm YAM station."""

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

USD = Path(__file__).with_name("dounce.usda")
CONTROL_HZ = 20
MAX_STEPS = 3000
SUCCESS_HOLD_S = 1.
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table

# Tabletop frame: x forward from the line through the arm bases, y to the robot's left, z up from the table.
AXIS_XY = np.array((.360, 0.))                  # the well in use (and so the tube's axis)
# The well in use is the middle one of the block's row nearest the robot (4 rows of 5); jack under the block.
BLOCK_CENTRE = np.array((AXIS_XY[0] + 1.5 * L.WELL_PITCH, AXIS_XY[1], L.JACK_H + L.BLOCK[2] / 2))
JACK_START = np.array((BLOCK_CENTRE[0], BLOCK_CENTRE[1], 0.))
WELLS = [(BLOCK_CENTRE[0] + (i - 1.5) * L.WELL_PITCH, BLOCK_CENTRE[1] + (j - 2) * L.WELL_PITCH)
         for i in range(4) for j in range(5)]
TUBE_START = np.array((*AXIS_XY, L.SEAT_Z))     # tube frame origin: its outside bottom, on the well's floor
PESTLE_START = TUBE_START + (0., 0., L.BOTTOM_T + L.START_DEPTH)     # pestle frame origin: head's bottom
ACTORS = ("tube", "pestle")
FLUID_JOINT = "dounce_fluid"
BORE_BOTTOM_Z = L.SEAT_Z + L.BOTTOM_T

INSTRUCTION = (
    "Homogenize the cells in the glass Dounce grinder with the tight pestle standing in it. Push the pestle down to "
    "the bottom, then give it 15 full strokes: each time draw it up until the blue ring on its shaft is above the "
    "tube's rim, keeping the red ring below the rim so that the head stays in the liquid, and push it back down to "
    "the bottom. Hold the tube down in its cold block all the while, or the pestle will pull it out. After the 15th "
    "stroke leave the pestle on the bottom and let go of both."
)


def side_camera():
    """Review camera level with the tube, in front of the block and to the right: the rim, the shaft's tape rings,
    the liquid and the top of the well in one view. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.66, -.20, TOP + .20))
    back = eye - (AXIS_XY[0], AXIS_XY[1], TOP + .185)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(24.))
