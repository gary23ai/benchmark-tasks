"""Dry runs of the scripted demonstration in the kinematic world, scored by the real scorer."""

import numpy as np
import pytest

pytest.importorskip("yam_arena")
pytest.importorskip("scipy")

from dryrun_demo import dry_run  # noqa: E402
from rdb_child_latch import fixed_scene as fixture  # noqa: E402
from rdb_child_latch import layout as L  # noqa: E402


def test_demonstration_reaches_full_score_within_the_limit():
    report, reference = dry_run("full")
    assert report["score"] == 1. and report["steps"] < fixture.MAX_STEPS / 2
    assert report["tries"] == [(0, True, True)]
    assert report["worst_limit_margin_deg"] > 8. and report["worst_ik_mm"] < .5
    assert reference.phases[-1][1] == "wait" and reference.log["grasped"]["holding"]


@pytest.mark.parametrize("options", [
    dict(lag_s=.25, press_offset=(-.004, 0., .006), pull_offset=.004),
    dict(lag_s=.08, press_offset=(.004, .005, -.005), hold_angle=np.radians(8.)),
    dict(sag=.03),
])
def test_demonstration_tolerates_arm_errors(options):
    report, _ = dry_run("full", **options)
    assert report["score"] == 1.


@pytest.mark.parametrize("variant", ["no_press", "early_release"])
def test_controls_stay_on_the_catch(variant):
    report, _ = dry_run(variant, max_steps=900)
    assert report["peak_score"] == .25 and report["open_mm"] <= 1000 * L.catch_open() + .5
