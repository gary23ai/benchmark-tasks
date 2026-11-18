"""Fixed scene: a 7-ft home pool table at the dual-arm YAM station, a cue on the near rail and three balls."""

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

USD = Path(__file__).with_name("pool_table.usda")
CONTROL_HZ = 20
MAX_STEPS = 1600
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the bench
ACTORS = ("cue_stick", "ball_cue", "ball_one", "ball_three")                     # free rigid bodies, USD names
BALL_BODY = {"cue": "ball_cue", "one": "ball_one", "three": "ball_three"}
BOUNCE_THRESHOLD = .05          # m/s: PhysX treats slower impacts as inelastic; the default 0.5 would kill the
                                # cushion rebound of a slow ball (set on the scene, see README)
ROLL_DECEL = .01 * 9.81         # rolling resistance 0.01 (Dr Dave: 0.005 to 0.015), applied as a force
SPIN_DECEL = 10.                # rad/s^2 about the vertical (Dr Dave: 5 to 15)

INSTRUCTION = (
    "Pot the yellow 1-ball in the corner pocket marked by the chalk cube. Take the cue from the rail, bridge it on "
    "the cloth with your left hand, hold it further back with your right and stroke the white into the 1-ball. "
    "Touch no ball except with the tip on the white."
)


def side_camera():
    """Review camera low over the table's left end, looking back along the shot at the bridge, the cue ball, the
    1-ball and the marked pocket. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.86, .95, TOP + .42))
    target = np.array((.30, .30, TOP + .03))
    back = eye - target
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(18.))
