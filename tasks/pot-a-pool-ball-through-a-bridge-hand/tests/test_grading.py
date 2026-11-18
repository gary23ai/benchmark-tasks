"""The scorer on hand-built trials: every tier, every foul, and the one-handed and pinched shortcuts."""

import numpy as np
import pytest

from rdb_pool_bridge import grading as G
from rdb_pool_bridge import layout as L

DOWN = np.column_stack(((1., 0., 0.), (0., -1., 0.), (0., 0., -1.)))     # left gripper pointing down
TILT = np.radians(40.)
_a = np.array((0., -np.sin(TILT), -np.cos(TILT)))
RIGHT = np.column_stack(((1., 0., 0.), np.cross(_a, (1., 0., 0.)), _a))


def balls(**moved):
    out = {n: (L.BALL_START[n].copy(), np.zeros(3)) for n in L.BALLS}
    out.update(moved)
    return out


def sample(advance=0., *, left=None, right_off=np.zeros(3), left_open=L.LOOP_GAP, right_open=.0215, ball_state=None,
           tip_offset=np.zeros(3)):
    """The cue at address moved ``advance`` toward the white; the left hand bridging, the right holding."""
    axis = L.CUE_AXIS
    tip = L.TIP_ADDRESS + advance * axis + tip_offset
    loop_point = L.BRIDGE_ADDRESS                       # the axis point in the loop (the hand does not move)
    left_tcp = loop_point + (L.LOOP_CEILING - L.PAD_TIP_PAST_TCP - .0065) * DOWN[:, 2] if left is None else left
    right_tcp = L.cue_point(tip, axis, L.GRIP_S) + right_off
    return {"balls": ball_state or balls(), "cue_tip": tip, "cue_axis": axis, "cue_velocity": np.zeros(3),
            "tcp": {"left": (left_tcp, DOWN), "right": (right_tcp, RIGHT)},
            "opening": {"left": left_open, "right": right_open}}


def strike(grade, *, steps=12, per_step=.006, touch=12, **kw):
    """Approach for ``steps`` control steps (the shaft touching the left hand in ``touch`` physics steps of
    each), then report the tip on the white."""
    out = None
    for k in range(steps):
        out = grade.update(sample(-per_step * (steps - k), **kw), {"left_on_cue": touch})
    return grade.update(sample(0., **kw), {"tip_cue_ball": 2, "impulse_ns": .13, "peak_tip_force_n": 60.,
                                           "left_on_cue": touch})


def finish(grade, ball_state, extra=None, steps=25):
    out = None
    for k in range(steps):
        out = grade.update(sample(.01, ball_state=ball_state), extra if k == 0 else None)
    return out


def test_bridge_geometry_matches_the_loop():
    check = G.bridge_check(sample())
    assert check["on_cloth"] and check["in_loop"] and check["resting"]
    assert abs(check["axis_above_tips_mm"] - 1000 * (L.LOOP_CEILING - .0065)) < 1.


def test_full_marks_for_a_legal_pot():
    g = G.PoolGrade()
    r = strike(g)
    assert r["struck"] and r["legal"] and r["score"] == .1
    potted = balls(one=(np.r_[L.POCKETS["near_left"]["hole"], L.BED_Z - .07], np.zeros(3)),
                   cue=(np.r_[L.GHOST_XY + (.05, .05), L.BALL_Z], np.zeros(3)))
    r = finish(g, potted, {"cue_ball_hits": ("one",)})
    assert r["first_hit"] == "one" and r["potted"] == {"one": "near_left"} and r["score"] == 1. and r["finished"]


def test_tiers_below_full_marks():
    g = G.PoolGrade()
    strike(g)
    in_jaws = balls(one=(np.r_[L.POCKETS["near_left"]["mouth"] + .01 * L.POCKETS["near_left"]["out"], L.BALL_Z], np.zeros(3)))
    assert finish(g, in_jaws, {"cue_ball_hits": ("one",)})["score"] == .7
    g = G.PoolGrade()
    strike(g)
    assert finish(g, balls(one=(np.r_[.5, .6, L.BALL_Z], np.zeros(3))), {"cue_ball_hits": ("one",)})["score"] == .4
    g = G.PoolGrade()
    strike(g)
    assert finish(g, balls(), {"cue_ball_hits": ("three",)})["score"] == .1
    g = G.PoolGrade()
    strike(g)
    wrong = balls(one=(np.r_[L.POCKETS["near_side"]["hole"], L.BED_Z - .07], np.zeros(3)))
    assert finish(g, wrong, {"cue_ball_hits": ("one",)})["score"] == .4


@pytest.mark.parametrize("kw, failing", [
    (dict(left=L.BRIDGE_ADDRESS + (0., 0., .004)), "in_loop"),                     # pinched at the pads
    (dict(left_open=.020), "resting"),                                              # pads wider than the shaft
    (dict(left=L.BRIDGE_ADDRESS + (0., 0., .03)), "in_loop"),                      # hand not round the shaft
    (dict(right_off=np.array((0., 0., .035))), "on_axis"),                           # right hand off the cue
    (dict(right_open=.06), "closed"),                                               # right hand open
])
def test_illegal_strikes_score_zero(kw, failing):
    g = G.PoolGrade()
    r = strike(g, **kw)
    assert not r["legal"] and r["score"] == 0.
    flags = {**r["strike"]["bridge"], **r["strike"]["grip"]}
    assert flags[failing] is False


def test_a_bridge_off_the_cloth_is_illegal():
    raised = L.BRIDGE_ADDRESS + (L.LOOP_CEILING - L.PAD_TIP_PAST_TCP - .0065) * DOWN[:, 2] + (0., 0., .02)
    g = G.PoolGrade()
    r = strike(g, left=raised, tip_offset=np.array((0., 0., .0)))
    assert r["strike"]["bridge"]["on_cloth"] is False and r["score"] == 0.


def test_cue_must_slide_through_the_bridge():
    g = G.PoolGrade()
    r = strike(g, per_step=.002)                      # 24 mm in the last 0.5 s: a push, not a stroke
    assert r["strike"]["slid_mm"] < 1000 * G.SLIDE_MIN and not r["legal"]


def test_shaft_must_touch_the_bridge():
    g = G.PoolGrade()
    r = strike(g, touch=0)
    assert r["strike"]["bridge_touch_share"] == 0. and not r["legal"] and r["score"] == 0.


def test_two_hand_shove_without_a_bridge_is_illegal():
    """Both hands move with the cue (the left pinches it): it does not slide through the left hand."""
    g = G.PoolGrade()
    r = None
    for k in range(13):
        adv = -.006 * (12 - k)
        s = sample(adv)
        pinch = L.cue_point(s["cue_tip"], s["cue_axis"], L.BRIDGE_S) - .007 * DOWN[:, 2]
        s["tcp"]["left"] = (pinch, DOWN)
        s["opening"]["left"] = .0128
        r = g.update(s, {"tip_cue_ball": 1} if k == 12 else None)
    assert r["struck"] and not r["legal"] and r["score"] == 0.


def test_one_hand_strike_is_illegal():
    g = G.PoolGrade()
    r = strike(g, left=np.array((0., .3, .3)))
    assert not r["legal"] and r["score"] == 0.


@pytest.mark.parametrize("events, text", [
    ({"robot_balls": ("one",)}, "gripper or arm"),
    ({"cue_side_balls": ("cue",)}, "side touched"),
    ({"tip_object_balls": ("three",)}, "tip touched the three"),
])
def test_touch_fouls_are_latched(events, text):
    g = G.PoolGrade()
    strike(g)
    potted = balls(one=(np.r_[L.POCKETS["near_left"]["hole"], L.BED_Z - .07], np.zeros(3)))
    r = finish(g, potted, {"cue_ball_hits": ("one",), **events})
    assert r["score"] == 0. and any(text in f for f in r["fouls"])


def test_double_hit_and_scratch_are_fouls():
    g = G.PoolGrade()
    strike(g)
    away = balls(cue=(L.BALL_START["cue"] + (0., .03, 0.), np.array((0., .5, 0.))))
    g.update(sample(0., ball_state=away))
    r = g.update(sample(.03, ball_state=away), {"tip_cue_ball": 1})
    assert any("second time" in f for f in r["fouls"]) and r["score"] == 0.
    g = G.PoolGrade()
    strike(g)
    scratch = balls(one=(np.r_[L.POCKETS["near_left"]["hole"], L.BED_Z - .07], np.zeros(3)),
                    cue=(np.r_[L.POCKETS["far_left"]["hole"], L.BED_Z - .07], np.zeros(3)))
    r = finish(g, scratch, {"cue_ball_hits": ("one",)})
    assert r["score"] == 0. and any("white ball was pocketed" in f for f in r["fouls"])


def test_disturbing_a_ball_before_the_strike_is_a_foul():
    g = G.PoolGrade()
    g.update(sample(-.05))
    r = g.update(sample(-.05, ball_state=balls(one=(L.BALL_START["one"] + (.01, 0., 0.), np.zeros(3)))))
    assert any("moved before the strike" in f for f in r["fouls"])


def test_no_strike_scores_zero_and_never_finishes():
    g = G.PoolGrade()
    for _ in range(60):
        r = g.update(sample(-.05))
    assert r["score"] == 0. and not r["struck"] and not r["finished"]


def test_one_pinch_cannot_hold_the_cue_level():
    """The physics behind the second hand: held 0.80 m from the tip (0.2 m in front of the balance point) the
    cue's weight turns it with about 1 N m about the grip, several times what a pad pinch resists (0.1 to 0.3 N m)."""
    moment = L.CUE_MASS * 9.81 * (L.CUE_CM_S - L.GRIP_S)
    assert 1.0 < moment < 1.2
    assert moment > 3 * .3
