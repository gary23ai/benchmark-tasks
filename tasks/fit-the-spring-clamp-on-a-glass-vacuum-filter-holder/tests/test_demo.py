"""The demonstration in the stand-in world, scored by the real scorer (needs yam_arena and SciPy)."""

import os
import sys

import pytest

pytest.importorskip("scipy")
if os.environ.get("YAM_ARENA"):
    sys.path.insert(0, os.environ["YAM_ARENA"])
pytest.importorskip("yam_arena")

from dryrun_demo import dry_run  # noqa: E402


def test_demonstration_scores_one():
    report = dry_run()
    assert report["score"] == 1. and report["steps"] < 1200
    assert report["worst_ik_mm"] < 1. and report["worst_limit_margin_deg"] > 3.
    assert report["final"]["held_fitting_ticks"] == report["final"]["fitting_ticks"] > 0


def test_control_without_the_holding_hand_scores_at_most_06():
    report = dry_run(variant="no_hold")
    assert report["score"] <= .6 and report["final"]["unheld_fitting"]


def test_idle_robot_scores_zero():
    assert dry_run(idle=True, max_steps=100)["score"] == 0.
