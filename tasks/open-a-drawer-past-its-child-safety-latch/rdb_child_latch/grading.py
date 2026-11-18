"""Score the trial from how far the drawer is open and where the grippers are at the end. NumPy only.

Every rule can be judged with a ruler, a photo and the trial video:

- open: the gap between the frame's front face and the outer face of the drawer front, in mm;
- hands clear: no part of the robot touches the unit or the latch, and each gripper's centre point (midway
  between its fingertips) is at least 5 cm from the frame, the drawer and its pull.

Score at the end of the trial: 0 if the unit has been shifted more than 10 mm on the table or the latch or its
catch is broken or off its screws; otherwise 1.0 if the drawer is open 150 mm or more with the hands clear for
2 s, 0.8 if it is open 150 mm or more, 0.6 if it is open 60 mm or more (the hook is past the catch, which it can
only pass pressed down), 0.25 if it is open 30 mm or more (pulled out to the catch), else 0.
"""

from dataclasses import dataclass

import numpy as np

from . import layout

try:  # the Inspect Robots scorer wrapper is only needed inside a trial
    from inspect_robots.scorer import Score
except ImportError:  # pragma: no cover
    Score = None

CONTROL_HZ = 20
HOLD_S = 2
HOLD_TICKS = CONTROL_HZ * HOLD_S
OPEN_M = .150                # opened the rest of the way
PAST_M = .060                # the hook is past the catch (every part of it above the catch's underside is in front of it)
AT_CATCH_M = .030            # pulled out to the catch (it stops the drawer at 38 mm)
CLEAR_M = .050               # gripper centre point to the unit
TOUCH_N = .05                # contact force that counts as touching
UNIT_SHIFT_M = .010          # the unit pushed or pulled this far on the table: 0

assert layout.catch_open() < PAST_M - .001 and layout.hook_clear_open() < PAST_M
assert AT_CATCH_M < layout.catch_open() - .005


def clearance(tcp, open_m):
    """Smallest distance from a gripper centre point to the box round the frame, the drawer and its pull."""
    low, high = layout.unit_box(open_m)
    tcp = np.atleast_2d(np.asarray(tcp, dtype=float))
    return float(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.), axis=1).min())


@dataclass
class DrawerGrade:
    """Running grade of one trial. Call :meth:`update` once per control step; the last value is the score."""
    hold: int = 0

    def update(self, open_m, tcp, touching, unit_shift_m=0., latch_broken=False):
        """``open_m``: drawer opening. ``tcp``: both gripper centre points. ``touching``: whether any part of the
        robot touches the drawer or the latch. ``unit_shift_m``: how far the frame has moved on the table, and
        ``latch_broken``: latch or catch broken or off its screws (both only arise in real trials: the frame and
        the catch are fixed in the simulator and the latch's pin and stops cannot fail)."""
        values = [np.asarray(v, dtype=float) for v in (open_m, tcp, unit_shift_m)]
        if not all(np.isfinite(v).all() for v in values):
            self.hold = 0
            return self._grade(float("nan"), False, 0.)
        open_m = float(open_m)
        gap = clearance(tcp, open_m)
        hands_clear = bool(gap >= CLEAR_M and not touching)
        void = bool(unit_shift_m > UNIT_SHIFT_M or latch_broken)
        opened = open_m >= OPEN_M
        self.hold = self.hold + 1 if opened and hands_clear and not void else 0
        if void:
            score = 0.
        elif opened:
            score = 1. if self.hold >= HOLD_TICKS else .8
        elif open_m >= PAST_M:
            score = .6
        elif open_m >= AT_CATCH_M:
            score = .25
        else:
            score = 0.
        return self._grade(open_m, hands_clear, score, gap, void)

    def _grade(self, open_m, hands_clear, score, gap=float("nan"), void=False):
        return {"score": score, "open_mm": round(1000. * open_m, 1), "hands_clear": hands_clear,
                "gripper_clearance_mm": round(1000. * gap, 1), "void": void, "hold_ticks": self.hold}


@dataclass(frozen=True)
class ChildLatchScorer:
    name: str = "child_latch_score"

    def __call__(self, record, target):
        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["drawer_grade"]
        if grade["void"]:
            text = "0: the unit was moved or the latch broken"
        elif grade["score"] == 1.:
            text = f'drawer open {grade["open_mm"]:.0f} mm, hands clear'
        else:
            text = f'drawer open {grade["open_mm"]:.0f} mm' + (", a hand still on or near it" if grade["score"] == .8 else "")
        return Score(value=grade["score"], explanation=text)


def make_scorer():
    return ChildLatchScorer()
