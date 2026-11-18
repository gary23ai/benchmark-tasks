"""The scoring rules, tier by tier, and the unintended strategies."""

import numpy as np
import pytest

from rdb_child_latch import layout as L
from rdb_child_latch.grading import (AT_CATCH_M, HOLD_TICKS, OPEN_M, PAST_M, DrawerGrade, clearance)

FAR = [(.30, .06, .26), (.22, -.15, .25)]          # both grippers parked away from the unit


def final(open_m, *, ticks=HOLD_TICKS, tcp=FAR, touching=False, shift=0., broken=False):
    grade, result = DrawerGrade(), None
    for _ in range(ticks):
        result = grade.update(open_m, tcp, touching, shift, broken)
    return result


@pytest.mark.parametrize("open_mm, score", [(0, 0.), (29, 0.), (30, .25), (38, .25), (59, .25), (60, .6),
                                            (149, .6), (150, 1.), (165, 1.), (300, 1.)])
def test_tiers_by_opening(open_mm, score):
    assert final(open_mm / 1000.)["score"] == score


def test_thresholds_sit_where_the_latch_decides():
    # 0.25 is reached at the catch, which stops the drawer at 1.5 in; 0.6 lies beyond the point where every part
    # of the hook that reaches above the catch's underside is in front of it, so it needs the hook pressed down.
    assert AT_CATCH_M < L.catch_open() < L.hook_clear_open() < PAST_M < OPEN_M
    assert abs(L.catch_open() - 1.5 * .0254) < 1e-6
    assert PAST_M - L.hook_clear_open() < .003


def test_full_score_needs_two_seconds_with_hands_clear():
    assert final(.16, ticks=HOLD_TICKS - 1)["score"] == .8
    assert final(.16)["score"] == 1.
    assert final(.16, touching=True)["score"] == .8                 # a hand still on the pull
    near = [FAR[0], (L.drawer_x(.16) - L.PULL_SIZE[0] - .030, -.05, L.PULL_Z)]
    assert clearance(near, .16) < .05 and final(.16, tcp=near)["score"] == .8
    above = [(L.FRAME_FRONT_X + .05, 0., L.FRAME_SIZE[2] + .04), FAR[1]]
    assert final(.16, tcp=above)["score"] == .8                     # hovering 4 cm over the frame is not clear


def test_hold_restarts_when_interrupted():
    grade = DrawerGrade()
    for _ in range(HOLD_TICKS - 5):
        grade.update(.16, FAR, False)
    grade.update(.16, FAR, True)
    for _ in range(HOLD_TICKS - 1):
        result = grade.update(.16, FAR, False)
    assert result["score"] == .8
    assert grade.update(.16, FAR, False)["score"] == 1.


def test_only_the_end_counts():
    grade = DrawerGrade()
    for _ in range(HOLD_TICKS):
        grade.update(.16, FAR, False)
    # opened fully, then pushed back shut: scored where it ends
    assert grade.update(0., FAR, False)["score"] == 0.
    assert grade.update(.045, FAR, False)["score"] == .25


def test_moving_the_unit_or_breaking_the_latch_scores_zero():
    assert final(.20, shift=.011)["score"] == 0.
    assert final(.20, shift=.009)["score"] == 1.
    assert final(.20, broken=True)["score"] == 0.                   # e.g. the latch prised off its screws
    assert final(.04, broken=True)["void"]


def test_nonfinite_input_scores_zero():
    assert final(float("nan"))["score"] == 0.
    assert final(.16, tcp=[(np.nan, 0., 0.), FAR[1]])["score"] == 0.


def test_latch_numbers_hold_together():
    # The hook overlaps the catch by 5 mm and is freed by pressing the arm about 5 degrees, well inside its
    # 10-degree travel; the press needed 21 mm behind the pin is a few newtons, within what a hand can push.
    assert abs(L.hook_drop(L.RELEASE_ANGLE) - L.ENGAGE) < 1e-6
    assert np.radians(4.) < L.RELEASE_ANGLE < np.radians(6.)
    force = L.press_force(L.RELEASE_ANGLE + np.radians(2.), L.HINGE_X + .021)
    assert 3. < force < 8.
    # at rest the spring holds the hook up with about 1 N at the hook
    assert .8 < L.spring_torque(0.) / (L.HOOK_FACE_X - L.HINGE_X) < 1.5
