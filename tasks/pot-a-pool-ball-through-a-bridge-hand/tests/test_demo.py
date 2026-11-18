"""The scripted demonstration in the kinematic stand-in world, scored by the real scorer."""

import numpy as np
import pytest

pytest.importorskip("scipy")
ik = pytest.importorskip("yam_arena.ik")

import dryrun_demo  # noqa: E402
from rdb_pool_bridge import layout as L  # noqa: E402
from rdb_pool_bridge.scripted import PoolReference, hand_rotation  # noqa: E402


def test_hand_rotations_are_proper():
    for arm in ("left", "right"):
        for sign in (1., -1.):
            r = hand_rotation(arm, sign)
            assert np.allclose(r @ r.T, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(r), 1.)
            assert abs(r[:, 0] @ L.CUE_AXIS) < .13          # the pads close across the cue


@pytest.mark.parametrize("kw", [{}, dict(lag_s=.06), dict(lag_s=.2), dict(speed_scale=.75), dict(speed_scale=1.3),
                                dict(sag=(0.,) * 6, lag_s=.06, speed_scale=1.15), dict(swerve=.025, lag_s=.075),
                                dict(swerve=.04, lag_s=.075)])
def test_demonstration_scores_full_marks(kw):
    report = dryrun_demo.dry_run(**kw)
    assert report["score"] == 1. and report["legal"] and report["fouls"] == []
    assert report["potted"] == {"one": "near_left"} and report["steps"] < 650
    assert report["worst_limit_margin_deg"] > 5. and report["worst_ik_mm"] < .5
    assert report["strike"]["slid_mm"] > 100. and report["strike"]["bridge_moved_mm"] < 2.


def test_practice_strokes_cancel_the_hands_swing(monkeypatch):
    """Run 2 in Isaac: the right hand swung 16 mm sideways at speed and the 1-ball was cut too thin (0.4). The
    stand-in world reproduces that swing (``swerve``); without the practice strokes the shot misses the pocket."""
    from rdb_pool_bridge import scripted

    monkeypatch.setattr(scripted, "PRACTICE_STROKES", 0)
    assert dryrun_demo.dry_run(swerve=.025, lag_s=.075)["score"] < 1.


def test_slow_heavy_arm_still_strikes_legally():
    """Lag 0.2 s, 15 % weaker strike and heavy sag: the stroke is planned on the measured lag but saturates;
    the shot is legal and hits the 1-ball first (0.4) even when the 1-ball stops short."""
    report = dryrun_demo.dry_run(sag=(0., .04, 0., .09, .02, 0.), lag_s=.2, speed_scale=.85)
    assert report["legal"] and report["first_hit"] == "one" and report["score"] >= .4


@pytest.mark.parametrize("variant", ["one_hand", "no_slide"])
def test_shortcuts_score_zero(variant):
    report = dryrun_demo.dry_run(variant=variant)
    assert report["score"] == 0. and not report["legal"]


def test_idle_robot_scores_zero():
    assert dryrun_demo.dry_run(idle=True, max_steps=200)["score"] == 0.


def test_lag_model_round_trip():
    ref = PoolReference()
    profile = ref.stroke_profile(.25, .065, .065)
    for lag in (.05, .1, .18):
        behind = list(np.asarray(profile) - np.asarray(ref.follow(profile, lag, extra=0)))
        assert abs(ref.fit_lag(profile, behind) - lag) < .006
