"""Fixed scene: a mouse pad lying flat on a carton, and a paper band in a cradle, on the YAM station."""

from pathlib import Path

import numpy as np

from . import pad_model as m

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only NumPy
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

TASK_ID = "roll-a-mouse-pad-and-band-it"
SCORER = "pad_roll_score"
USD = Path(__file__).with_name("pad.usda")
CONTROL_HZ = 20
PHYSICS_HZ = 240
MAX_STEPS = 2400
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table

STRIPS = tuple(f"strip_{i}" for i in range(m.STRIPS))
LINKS = tuple(f"link_{i}" for i in range(1, m.STRIPS))          # link_i sits between strip_{i-1} and strip_i
FOLDS = tuple(f"fold_{i}" for i in range(1, m.STRIPS))          # hinge between strip_{i-1} and strip_i
SLIDES = tuple(f"slide_{i}" for i in range(1, m.STRIPS))        # shear slide between them
ROOT = f"strip_{m.ROOT}"

INSTRUCTION = (
    "Roll the mouse pad up from one short edge and slide the paper band over the roll, so that the band holds it "
    "rolled. Then put the banded roll down, let go and move both arms away."
)
