"""Kinematic dry run of the scripted demonstration through the real air rule and scorer (needs yam_arena)."""

import numpy as np
import pytest

pytest.importorskip("yam_arena")

from dryrun_demo import dry_run  # noqa: E402
from rdb_harmonium import fixed_scene as fixture  # noqa: E402
from rdb_harmonium import layout as L  # noqa: E402


def test_nominal_dry_run_scores_one_within_the_run_limit():
    report = dry_run()
    assert report["score"] == 1. and report["steps"] < .5 * fixture.MAX_STEPS
    assert all(v >= 4.4 for v in report["longest_s"].values()) and report["other_keys_s"] == 0.
    assert all(-.8 <= j <= .1 for j in report["joins_s"])                 # legato: the sound never breaks
    assert report["worst_ik_mm"] < .5 and report["worst_limit_margin_deg"] > 15.
    assert report["min_volume_while_playing_l"] > .3 and report["strokes"] >= 6


@pytest.mark.parametrize("kw", [dict(arm_lag_s=.25), dict(load_give=(.030, .008)),
                                dict(arm_lag_s=.07, wrist_lag_s=.15), dict(arm_lag_s=.05, wrist_lag_s=.25),
                                dict(arm_lag_s=.07, wrist_lag_s=.15, key_hand_y=.003),
                                dict(arm_lag_s=.07, wrist_lag_s=.15, key_hand_y=-.003),
                                dict(arm_lag_s=.07, wrist_lag_s=.15, key_hand_z=.007),
                                dict(arm_lag_s=.07, wrist_lag_s=.15, key_hand_z=.010),
                                dict(arm_lag_s=.07, wrist_lag_s=.15, key_hand_z=.007, key_hand_y=.003)])
def test_slow_lagging_or_yielding_arms_still_score_one(kw):
    report = dry_run(**kw)
    assert report["score"] == 1. and report["worst_ik_mm"] < .5 and report["other_keys_s"] == 0.


@pytest.mark.parametrize("variant", ["no_pump", "pump_only", "prime_then_stop"])
def test_neither_hand_alone_scores(variant):
    """No pumping: the reservoir stays empty and no reed speaks. No keys: nothing sounds. One stroke to fill the
    reservoir and then no more pumping: Sa dies away under 3 s, short of the 4 s hold."""
    report = dry_run(variant=variant)
    assert report["score"] == 0. and report["played"] == 0.
    assert max(report["longest_s"].values()) < 3.2


def test_both_hands_are_working_at_the_same_time():
    """During every held note the left hand pushed the board shut at least once."""
    from rdb_harmonium.scripted import HarmoniumReference

    report = dry_run()
    notes = [(t, name) for t, name in report["phases"] if name.startswith("note")]
    ends = [t for t, _ in notes[1:]] + [dict((n, t) for t, n in report["phases"])["keys released"]]
    strokes = report["stroke_ticks"]
    for (start, name), end in zip(notes, ends, strict=True):
        assert any(start <= s < end for s in strokes), name
    assert isinstance(HarmoniumReference().rotation["left"], np.ndarray) and L.FLAP_OPEN > 0.


def test_fast_ik_matches_yam_arena():
    from yam_arena import ik

    from rdb_harmonium.scripted import CHAIN

    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform(ik.LOWER + .2, ik.UPPER - .2)
        p, r = ik.tcp_pose(q, fixture.BASES["left"])
        p2, r2, _, _ = CHAIN.fk(q, fixture.BASES["left"])
        assert np.allclose(p, p2, atol=1e-9) and np.allclose(r, r2, atol=1e-9)


def test_pump_recovery_path_is_reachable():
    """Run 1: the finger rode up over the board's edge and came to rest on the case top at about (412, 223, 217)
    mm. The recovery path from there back behind the board must be reachable throughout."""
    from rdb_harmonium.scripted import CHAIN, HarmoniumReference

    reference = HarmoniumReference()
    slipped = np.array((.412, .223, .217))
    reference.target["left"] = slipped
    q = reference.solve("left", slipped, fresh=True)
    reference.joints["left"] = q
    reference.measured[:6] = q
    reference.observe(reference.measured, {"flap": L.FLAP_OPEN, "volume": 0., "depths": {}, "sounding": ()})
    assert reference.slipped()
    worst = 0.
    for target in reference.recover():
        reference.joints["left"] = reference.solve("left", target)
        worst = max(worst, reference.worst_ik)
    assert worst < 1e-3 and reference.deeper > 0.
    tcp = CHAIN.fk(reference.joints["left"], fixture.BASES["left"])[0]
    assert tcp[0] > fixture.flap_corner(L.FLAP_OPEN)[0]                  # back behind the board


def test_a_good_stroke_is_not_taken_for_a_slip():
    """Run 2: the slip check compared the hand with its command, which leads the board and sinks 12 mm below it,
    so every good stroke was flagged. With the edge on the finger at the board's measured angle there is no slip."""
    from rdb_harmonium.scripted import HarmoniumReference

    reference = HarmoniumReference()
    for theta in (L.FLAP_OPEN, .2, .12):
        on_edge = fixture.flap_corner(theta)
        q = reference.solve("left", on_edge, fresh=True)
        reference.measured[:6] = q
        reference.observe(reference.measured, {"flap": theta, "volume": .5e-3, "depths": {}, "sounding": ()})
        reference.bias = .07
        assert not reference.slipped(), theta


def test_run5_depth_cap_is_reproduced_and_fixed(monkeypatch):
    """Run 5: the pressing hand rode about 7 mm above its plan and the depth servo was capped at 6 mm, so Sa and Re
    stayed under 2.5 mm. The dry run with that cap and that height error fails the same way; the current cap works."""
    import rdb_harmonium.scripted as scripted

    lag = dict(arm_lag_s=.07, wrist_lag_s=.15, key_hand_z=.007)
    assert dry_run(**lag)["score"] == 1.
    monkeypatch.setattr(scripted, "DZ_RANGE", (-.006, .0015))
    report = dry_run(**lag)
    assert report["score"] == 0. and report["longest_s"]["C4"] < 1.
