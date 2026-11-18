"""The scripted demonstration in the stand-in world (one-dimensional glass physics), scored by the real scorer."""

import pytest

pytest.importorskip("scipy")
pytest.importorskip("yam_arena.ik")

import dryrun_demo  # noqa: E402


@pytest.mark.parametrize("kw", [{}, dict(sag=(0., .04, 0., .09, .02, 0.), lag_s=.2, drag=400.),
                                dict(sag=(0.,) * 6, lag_s=.06, drag=120.)])
def test_demonstration_scores_full_marks(kw):
    r = dryrun_demo.dry_run(**kw)
    assert r["score"] == 1. and r["strokes"] == r["good"] == 15 and r["steps"] < 2000
    assert r["worst_limit_margin_deg"] > 10. and r["worst_ik_mm"] < .5
    assert not r["foamed"] and not r["head_out"] and r["peak_lift_mm"] < 3.
    assert r["peak_fluid_n"] < 8.62 or kw.get("drag", 0) > 300


def test_unheld_tube_is_pulled_out_of_its_well():
    r = dryrun_demo.dry_run(variant="no_hold")
    assert r["score"] == 0. and r["failed"] and r["peak_lift_mm"] > 25.
    first = r["stroke_log"]
    assert not any(row["good"] for row in first)


def test_idle_robot_scores_zero():
    assert dryrun_demo.dry_run(idle=True, max_steps=300)["score"] == 0.
