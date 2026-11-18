"""task.json, README and the code say the same thing."""

import json
import re
from pathlib import Path

from rdb_dounce import fixed_scene as F
from rdb_dounce import grading as G
from rdb_dounce import layout as L

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()


def test_required_fields():
    for key in ("title", "category", "instruction", "context", "setup", "scoring", "run_limit", "fidelity", "scores",
                "reset_notes"):
        assert TASK[key]
    assert TASK["category"] == "Other"
    assert len(TASK["fidelity"].split()) <= 100


def test_instruction_and_run_limit_match_the_code():
    assert TASK["instruction"] == F.INSTRUCTION
    assert TASK["run_limit"].startswith(f"{F.MAX_STEPS:,} steps")
    assert F.CONTROL_HZ == G.CONTROL_HZ == 20
    assert f"{F.MAX_STEPS:,} actions" in README


def test_setup_numbers_match_the_code():
    s = TASK["setup"]
    assert f"({1000 * F.AXIS_XY[0]:.0f}, {1000 * F.AXIS_XY[1]:.0f})" in s
    assert f"({1000 * F.JACK_START[0]:.0f}, {1000 * F.JACK_START[1]:.0f})" in s
    assert f"set {1000 * L.JACK_H:.0f} high" in s
    assert f"wells {1000 * L.WELL_D:.0f} across, {1000 * L.WELL_DEPTH:.0f} deep" in s
    assert f"rim {1000 * (L.SEAT_Z + L.TUBE_L):.0f} above the bench" in s
    assert f"{L.LIQUID_ML:.1f} mL" in s and "Wheaton 357542" in s and "13 OD x 82 long" in s
    for ring in (L.RING_BOTTOM, L.RING_TOP, L.RING_LIMIT):
        assert f"at {1000 * ring:.0f}" in s
    assert f"head {1000 * L.START_DEPTH:.0f} above the bore's bottom" in s


def test_scoring_numbers_match_the_code():
    s = TASK["scoring"]
    assert "Possible scores: 0, 0.2, 0.4, 0.6, 0.8, 1.0" in s and list(G.TIERS.values()) == [.2, .4, .6, .8, 1.]
    assert f"within {1000 * L.BOTTOM_BAND:.0f} of the bottom" in s
    assert f"within {1000 * L.SEAT_LIFT:.0f} of the block's top" in s and f"within {L.SEAT_TILT_DEG:.0f} degrees" in s
    assert f"at most {G.PASS_S:.0f} s" in s and f"at most {G.MAX_STROKES} strokes" in s
    assert f"faster than {1000 * L.FOAM_SPEED:.0f} mm/s" in s and f"for {G.FOAM_HOLD_S} s" in s
    assert f"at least {1000 * G.CLEAR:.0f} mm" in s and f"past {L.SPILL_TILT_DEG:.0f} degrees" in s
    assert f"lifted {1000 * L.OUT_LIFT:.0f}" in s


def test_fidelity_states_the_law_as_coded_and_as_measured():
    f = TASK["fidelity"]
    assert f"Authored {L.DRAG:.0f} N s/m" in f and f"about {(L.PESTLE_KG * 9.81 / .003785):.0f} N s/m applied" in f
    assert f"{L.SUCTION_CAP:.1f} N" in f and f"{1000 * (L.WALL_IN - L.HEAD_R_SIM):.2f} mm" in f


def test_measured_results_as_reported():
    # run 1 (RESULT.json of the scripted and Dream-1 trials): numbers the text reports
    assert TASK["scores"] == "Dream-1 run: 0.0; demo: 1.0"
    assert abs((L.PESTLE_KG * 9.81 / .003785) - 54.) < 1. and abs(L.DRAG * .815e-3 / (L.PESTLE_KG * 9.81) - 1.) < .01
    for text in ("1,451 steps", "3.79 mm/s", "about **54 N s/m**", "15 good strokes of\n15", "0.27 mm", "54.7 mm",
                 "343 s of wall time", "747 s", "87 requests", "32.1 mm/s"):
        assert text in README, text
    assert "PENDING" not in README + json.dumps(TASK)


def test_text_does_not_identify_anyone():
    for text in (json.dumps(TASK), README):
        home_dirs = "|".join("/" + d + "/" for d in ("Users", "home"))     # spelled so this file itself stays clean
        assert not re.search(home_dirs + r"|@[a-z]+\.(edu|com)\b", text)
