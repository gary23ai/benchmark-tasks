"""The scoring rules, on hand-built states: every tier and every cheap way round the safety step."""

import numpy as np
import pytest

from rdb_sprayer_relief import grading
from rdb_sprayer_relief import layout as L
from rdb_sprayer_relief.grading import SprayerGrade

UP = np.array((0., 0., 1.))
MARK = np.zeros(3)
AWAY = [np.array((.02, .305, .26)), np.array((.02, -.305, .26))]         # both grippers back at their ready pose
ON = [np.array((.03, .02, .33)), np.array((.02, -.305, .26))]            # one still on the pump knob
TURN = np.radians


def run(grade, ticks, **state):
    values = dict(bar=0., valve_lift=0., head_angle=0., base=MARK, up=UP, tool_points=AWAY)
    values.update(state)
    result = None
    for _ in range(ticks):
        result = grade.update(**values)
    return result


def test_nothing_done_scores_zero():
    assert run(SprayerGrade(), 60, bar=2.)["score"] == 0.


def test_pressure_out_first_then_each_step():
    grade = SprayerGrade()
    assert run(grade, 20, bar=2., valve_lift=.005, tool_points=ON)["score"] == 0.          # still venting
    assert run(grade, 20, bar=.09, valve_lift=.005, tool_points=ON)["score"] == .3         # pressure out
    assert run(grade, 5, bar=0., head_angle=TURN(170.), tool_points=ON)["score"] == .3     # under half a turn
    assert run(grade, 5, bar=0., head_angle=TURN(181.), tool_points=ON)["score"] == .6
    assert run(grade, 5, bar=0., head_angle=L.HEAD_FREE - TURN(12.), tool_points=ON)["score"] == .6   # not all the way
    assert run(grade, 40, bar=0., head_angle=L.HEAD_FREE, tool_points=ON)["score"] == .8   # free, but still held
    assert run(grade, grading.REST_TICKS - 1, bar=0., head_angle=L.HEAD_FREE)["score"] == .8
    assert run(grade, 2, bar=0., head_angle=L.HEAD_FREE)["score"] == 1.


def test_quarter_turn_under_pressure_is_a_latched_zero():
    grade = SprayerGrade()
    assert run(grade, 3, bar=2., head_angle=TURN(9.))["hazard"] is False
    result = run(grade, 1, bar=2., head_angle=TURN(11.))
    assert result["hazard"] and result["score"] == 0. and grade.finished
    # letting the pressure out afterwards and finishing the job does not undo it
    assert run(grade, 80, bar=0., head_angle=L.HEAD_FREE)["score"] == 0.


def test_partly_vented_is_not_vented():
    grade = SprayerGrade()                              # valve let go early: still under pressure
    assert run(grade, 10, bar=.4, valve_lift=0.)["score"] == 0.
    assert run(grade, 1, bar=.4, head_angle=TURN(12.))["hazard"]
    grade = SprayerGrade()                              # exactly at the threshold counts as out
    assert run(grade, 1, bar=grading.SAFE_BAR)["score"] == .3


def test_pressure_lost_through_the_thread_does_not_count_as_venting():
    grade = SprayerGrade()
    bar, head = 2., 0.
    for _ in range(200):                                # the head is simply unscrewed; the air leaves at the thread
        head = min(head + TURN(3.), L.HEAD_FREE)
        bar = L.pressure_step(bar, 0., head, .05)
        result = grade.update(bar, 0., head, MARK, UP, AWAY)
    assert result["bar"] == 0. and result["head_free"] and result["score"] == 0. and result["hazard"]


def test_slow_unscrewing_is_still_the_hazard():
    grade = SprayerGrade()
    for degrees in np.arange(0., 12., .05):             # creeping the head round
        result = grade.update(2., 0., TURN(degrees), MARK, UP, ON)
    assert result["hazard"]


def test_knocked_over_is_a_latched_zero():
    grade = SprayerGrade()
    run(grade, 5, bar=0.)
    tipped = np.array((np.sin(TURN(35.)), 0., np.cos(TURN(35.))))
    assert run(grade, 1, bar=0., up=tipped)["score"] == 0.
    assert run(grade, 80, bar=0., head_angle=L.HEAD_FREE)["score"] == 0.                   # stood up again: still 0


@pytest.mark.parametrize("state", [
    dict(base=np.array((.035, 0., 0.))),                                # pushed off its mark
    dict(up=np.array((np.sin(TURN(7.)), 0., np.cos(TURN(7.))))),        # left leaning
    dict(valve_lift=.004),                                              # valve still held out
    dict(tool_points=ON),                                               # a gripper still on the sprayer
    dict(tool_points=None),                                             # grippers cannot be read
])
def test_full_marks_need_the_sprayer_left_standing_and_let_go(state):
    grade = SprayerGrade()
    run(grade, 5, bar=0.)
    assert run(grade, 80, bar=0., head_angle=L.HEAD_FREE, **state)["score"] == .8


def test_stillness_window_restarts_when_something_moves():
    grade = SprayerGrade()
    run(grade, 5, bar=0.)
    run(grade, grading.REST_TICKS - 2, bar=0., head_angle=L.HEAD_FREE)
    assert run(grade, 1, bar=0., head_angle=L.HEAD_FREE, base=np.array((.004, 0., 0.)))["rest_ticks"] == 0
    assert run(grade, grading.REST_TICKS - 1, bar=0., head_angle=L.HEAD_FREE, base=np.array((.004, 0., 0.)))["score"] == .8


def test_already_vented_scene_needs_no_valve():
    grade = SprayerGrade()                              # the control scene: no pressure at the start
    assert run(grade, 1, bar=0.)["score"] == .3
    assert run(grade, 60, bar=0., head_angle=L.HEAD_FREE)["score"] == 1.


def test_pressure_rule():
    bar = L.START_BAR
    for _ in range(60):                                 # valve out less than 3 mm: nothing escapes
        bar = L.pressure_step(bar, .0029, 0., .05)
    assert bar == L.START_BAR
    seconds = 0.
    while bar > L.SAFE_BAR:
        bar = L.pressure_step(bar, L.VALVE_OPEN, 0., .05)
        seconds += .05
    assert 3. < seconds < 4.2                           # about 3.6 s of holding the valve out


def test_axis_gap():
    assert np.isclose(grading.axis_gap((.2, 0., .1), MARK, UP), .2)
    assert np.isclose(grading.axis_gap((0., 0., L.HEIGHT + .05), MARK, UP), .05)
