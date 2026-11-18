"""The scripted demonstration, run through the arms' kinematics and scored by the real scorer (no contact physics)."""

import numpy as np
import pytest

pytest.importorskip("scipy")
pytest.importorskip("yam_arena")

import dryrun_demo  # noqa: E402
from rdb_sprayer_relief import fixed_scene as fixture  # noqa: E402
from rdb_sprayer_relief import layout as L  # noqa: E402
from rdb_sprayer_relief import scripted  # noqa: E402


def test_grip_frames():
    for heading in np.radians((-80., -30., 60.)):
        rotation = scripted.level(heading)
        assert np.isclose(np.linalg.det(rotation), 1.) and np.allclose(rotation[:, 0], (0., 0., 1.))
        assert np.allclose(rotation[:, 2], (np.cos(heading), np.sin(heading), 0.))


def test_stroke_plans_turn_the_hand_with_the_nozzle_and_cover_the_circle():
    lo, hi = scripted.HAND_RANGE
    covered = []
    for degrees in range(0, 360, 5):
        az = np.radians(degrees)
        plans = {arm: scripted.stroke_plan(arm, az) for arm in ("left", "right")}
        covered.append(any(plans.values()))
        for arm, plan in plans.items():
            if plan is None:
                continue
            h0, alpha, length = plan
            assert lo - 1e-9 <= h0 <= hi + 1e-9 and scripted.ALPHA_RANGE[0] - 1e-9 <= alpha <= scripted.ALPHA_RANGE[1] + 1e-9
            end = h0 + (length if arm == "left" else -length)        # the hand's heading at the end of the stroke
            assert lo - 1e-9 <= end <= hi + 1e-9
    assert all(covered)


@pytest.mark.parametrize("case", [
    {},
    dict(tank_yaw_deg=8., arm_lag_s=.2, miss_first_pinch=True, valve_slips_once=True),
    dict(tank_yaw_deg=-6., sag=(.002, -.002, -.004)),
])
def test_demo_scores_full_marks_in_the_dry_run(case):
    report = dryrun_demo.dry_run(**case)
    assert report["score"] == 1. and report["steps"] < .95 * fixture.MAX_STEPS
    assert report["worst_limit_margin_deg"] >= 4. and report["worst_ik_mm"] < .5
    assert report["grade"]["vented_first"] and not report["grade"]["hazard"]
    turned = [e for e in report["log"]["events"] if e["event"] == "turned"]
    assert len(turned) >= 3 and turned[-1]["head_deg"] >= np.degrees(L.HEAD_FREE) - 5.


def test_controls():
    skipped = dryrun_demo.dry_run(variant="no_vent")                 # head unscrewed without touching the valve
    assert skipped["score"] == 0. and skipped["grade"]["hazard"] and skipped["grade"]["head_deg"] < 100.
    vented = dryrun_demo.dry_run(variant="vent_only")                # pressure out, then stop: the safe partial result
    assert vented["score"] == .3 and vented["grade"]["bar"] <= L.SAFE_BAR
    empty = dryrun_demo.dry_run(start_bar=0.)                        # control scene: nothing to vent, valve not needed
    assert empty["score"] == 1. and not any(e["event"] == "pulled" for e in empty["log"]["events"])
    unheld = dryrun_demo.dry_run(variant="no_hold")                  # nobody holds the knob: the push tips the sprayer
    assert unheld["score"] == 0. and unheld["grade"]["head_deg"] < 5. and unheld["tank_tipped_deg"] > 30.


def test_both_hands_work_together_on_the_head():
    report = dryrun_demo.dry_run()
    events = [e for e in report["events"]]
    holds = [t for t, e in events if "pads on the pump knob" in e]
    lets = [t for t, e in events if "let go of the pump knob" in e]
    turns = [t for t, e in events if "pads on the nozzle" in e]
    assert len(holds) == 3 and len(lets) == 3 and len(turns) >= 6
    for turn in turns:                                  # every stroke on the nozzle happens inside a hold of the knob
        assert any(hold < turn < let for hold, let in zip(holds, lets, strict=True))
    for event in report["log"]["events"]:               # and the hand turns with the nozzle: the tank never turns
        if event["event"] == "turned":
            assert event["tank_turned_in_stroke_deg"] == 0.
    assert report["tank_tipped_deg"] == 0.


def test_demo_stopped_early_earns_partial_credit_only():
    assert dryrun_demo.dry_run(stop_after="pull and hold")["score"] == 0.          # valve pinched, pressure still in
    assert dryrun_demo.dry_run(stop_after="to the nozzle (left)")["score"] == .3   # vented, head untouched
    assert dryrun_demo.dry_run(stop_after="to the nozzle (right)")["score"] == .6  # half a turn loose
