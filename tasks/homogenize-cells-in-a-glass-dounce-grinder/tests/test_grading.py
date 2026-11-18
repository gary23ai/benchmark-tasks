"""Every scoring tier and the cheap strategies, on hand-built states (NumPy only)."""

import numpy as np
import pytest

from rdb_dounce import fixed_scene as F
from rdb_dounce import grading as G
from rdb_dounce import layout as L

I3 = np.eye(3)
FAR = [np.array((.10, .25, .30)), np.array((.10, -.25, .30))]
NEAR = [F.TUBE_START + (0., 0., .07), F.TUBE_START + (0., 0., .25)]       # hands on the tube and the handle
DT = 1. / G.CONTROL_HZ


def state(depth, lift=0., tilt_deg=0., tcp=NEAR):
    t = np.radians(tilt_deg)
    rot = np.array(((1., 0., 0.), (0., np.cos(t), -np.sin(t)), (0., np.sin(t), np.cos(t))))
    tube = F.TUBE_START + (0., 0., lift)
    pestle = tube + rot[:, 2] * (L.BOTTOM_T + depth)
    return {"tube_pos": tube, "tube_rot": rot, "pestle_pos": pestle, "pestle_rot": rot, "tcp": tcp}


def ramp(a, b, seconds, **kw):
    n = max(1, round(seconds / DT))
    return [state(a + (b - a) * (k + 1) / n, **kw) for k in range(n)]


def strokes(n, *, up_s=1.6, down_s=1.6, low=.002, high=.031, lift_on_up=0., dwell=3):
    out = ramp(L.START_DEPTH, low, 2.) + [state(low)] * dwell              # the first descent
    for _ in range(n):
        up = ramp(low, high, up_s)
        if lift_on_up:
            up = [state(s_depth(s), lift=lift_on_up) for s in up]
        out += up + [state(high)] * dwell + ramp(high, low, down_s) + [state(low)] * dwell
    return out


def s_depth(s):
    return G.measure(s["tube_pos"], s["tube_rot"], s["pestle_pos"], L.SEAT_Z)[0]


def run(states, g=None):
    g = g or G.DounceGrade()
    out = None
    for s in states:
        out = g.update(s)
    return out, g


def release(seconds=1.3, depth=.002):
    return [state(depth, tcp=FAR)] * round(seconds / DT)


def test_full_marks_after_fifteen_good_strokes_and_letting_go():
    out, g = run(strokes(15) + release())
    assert out["strokes"] == out["good_strokes"] == 15 and out["score"] == 1. and g.finished


def test_first_descent_is_not_a_stroke():
    out, _ = run(ramp(L.START_DEPTH, .002, 2.) + [state(.002)] * 5)
    assert out["strokes"] == 0 and out["score"] == 0.


@pytest.mark.parametrize("n, tier", [(0, 0.), (4, 0.), (5, .2), (9, .2), (10, .4), (14, .4), (15, .8)])
def test_partial_credit_by_good_strokes(n, tier):
    out, _ = run(strokes(n))
    assert out["score"] == tier


def test_still_holding_at_the_end_caps_at_point_eight():
    out, _ = run(strokes(15) + [state(.002)] * 40)
    assert out["score"] == .8


def test_pestle_left_up_is_not_done():
    out, _ = run(strokes(15) + ramp(.002, .020, 1.) + [state(.020, tcp=FAR)] * 30)
    assert out["score"] == .8


def test_tube_lifted_on_the_up_strokes_counts_nothing():
    out, _ = run(strokes(15, lift_on_up=.004) + release())
    assert out["strokes"] == 15 and out["good_strokes"] == 0 and out["score"] == 0.


def test_slight_lift_within_the_seat_is_fine():
    out, _ = run(strokes(15, lift_on_up=.002) + release())
    assert out["good_strokes"] == 15 and out["score"] == 1.


def test_slow_passes_do_not_count():
    # the creep that would leave an unheld tube in place (drag under the tube's weight) takes far longer than 4 s
    out, _ = run(strokes(15, up_s=6.))      # 4.6 s from band to band
    assert out["good_strokes"] == 0 and out["score"] == 0.


def test_short_strokes_are_not_strokes():
    out, _ = run(strokes(15, high=.024))
    assert out["strokes"] == 0
    out, _ = run(strokes(15, low=.007))
    assert out["strokes"] == 0


def test_head_out_of_the_liquid():
    out, _ = run(strokes(15, high=L.LIMIT_DEPTH + .002) + release())
    assert out["head_out"] and out["good_strokes"] == 0 and out["score"] == 0.
    out, _ = run(strokes(16)[:-200] + ramp(.002, L.LIMIT_DEPTH + .002, 1.5) + ramp(L.LIMIT_DEPTH + .002, .002, 1.5)
                 + strokes(15)[45:] + release())
    assert out["head_out"] and out["score"] <= .6


def test_fast_strokes_foam_and_cap_at_point_six():
    out, _ = run(strokes(15, up_s=.6, down_s=.6) + release())
    assert out["foamed"] and out["good_strokes"] == 15 and out["score"] == .6


def test_too_many_strokes_cap_at_point_six():
    out, _ = run(strokes(17) + release())
    assert out["strokes"] == 17 and out["score"] == .6
    out, _ = run(strokes(16) + release())
    assert out["score"] == 1.


def test_spill_is_a_latched_zero():
    out, g = run(strokes(15) + [state(.002, lift=.031)] + release())
    assert out["failed"] == "tube pulled out of its well" and out["score"] == 0.
    out, g = run(strokes(15) + [state(.002, tilt_deg=35.)] + release())
    assert out["failed"] == "tube tipped over" and out["score"] == 0.


def test_unreadable_grippers_are_never_clear():
    out, _ = run(strokes(15) + [state(.002, tcp=None)] * 40)
    assert out["score"] == .8 and not out["clear"]


def test_rings_and_bands_agree():
    assert L.RING_BOTTOM > L.RING_TOP > L.RING_LIMIT > 0.
    assert L.TOP_BAND + .004 < L.LIMIT_DEPTH
    assert 1000 * L.head_cover(L.TOP_BAND) > 10.
