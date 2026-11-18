"""Two stuck drinking cups on the calibrated dual-arm YAM workcell: station-bound constants."""

from pathlib import Path

import numpy as np
from yam_arena import station
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens

from . import cup_model as cup

USD = Path(__file__).with_name("cups.usda")
TOP = station.TABLE_TOP
if abs(TOP - cup.TABLE_TOP_WORLD_Z) > 1e-9:
    raise RuntimeError("cup_model.TABLE_TOP_WORLD_Z no longer matches the station's table top")
CONTROL_HZ = 20
SUCCESS_HOLD_S = 2
MAX_STEPS = cup.PROBE_MAX_STEPS if cup.PROBE else 2400
ARMS = ("left", "right")
CUPS = cup.CUPS
BASES = {arm: np.asarray(station.BASES[arm]) - (0., 0., TOP) for arm in ARMS}
PARK = {"left": (.14, .34, .19), "right": (.14, -.34, .19)}
# World start positions of the cup base centres (outer, inner) and their masses.
RESET_POS = cup.START + (0., 0., TOP)
RESET_MASS = (cup.MASS, cup.MASS)
# Table top rectangle in tabletop coordinates: (x min, x max, y min, y max).
TABLE_BOUNDS = (station.TABLE_CENTER[0] - station.TABLE_SIZE[0] / 2, station.TABLE_CENTER[0] + station.TABLE_SIZE[0] / 2,
                station.TABLE_CENTER[1] - station.TABLE_SIZE[1] / 2, station.TABLE_CENTER[1] + station.TABLE_SIZE[1] / 2)
INSTRUCTION = ("The two cups are stuck together. Get them apart and leave both standing upright on the table.")


def side_camera():
    eye = np.array((.57, -.40, TOP + .23))
    back = eye - np.array((.275, .085, TOP + .055))
    back /= np.linalg.norm(back)
    right = np.cross((0, 0, 1), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(26.))
