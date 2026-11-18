"""The scripted demonstration, played without the simulator through the real scorer (needs yam_arena)."""

import pytest

pytest.importorskip("yam_arena")


def test_demonstration_reaches_full_marks_in_the_dry_run():
    import dryrun_demo

    report = dryrun_demo.dry_run()
    assert report["score"] == 1. and report["steps"] < 1600 and report["worst_ik_mm"] < 1.
    assert report["worst_limit_margin_deg"] > 5. and report["in_cup"] == 2 and report["strays"] == 0
