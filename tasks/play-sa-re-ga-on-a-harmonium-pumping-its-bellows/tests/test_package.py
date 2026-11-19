"""task.json, README and the code say the same thing."""

import json
import math
import re
from pathlib import Path

import numpy as np

from rdb_harmonium import audio, fixed_scene as fixture
from rdb_harmonium import grading, layout as L
from rdb_harmonium.air import Air

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "task.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()


def test_required_fields_and_measured_scores():
    for key in ("title", "category", "instruction", "context", "setup", "scoring", "run_limit", "fidelity",
                "scores", "reset_notes"):
        assert TASK[key]
    assert TASK["category"] == "Societal relatability"
    if "media" in TASK: assert re.fullmatch(r"M-[A-Z0-9]+", TASK["media"])
    assert len(TASK["context"]) <= 1000 and len(TASK["fidelity"]) <= 500
    assert TASK["scores"].startswith("Dream-1 run: 0.0 (2,400 steps;") and "demo: 1.0 (464 steps)" in TASK["scores"]
    assert "was written just before" not in README and "renamed for clarity" in README
    for text in (json.dumps(TASK), README):
        assert "PENDING" not in text and "not run yet" not in text and "has not run" not in text


def test_instruction_and_run_limit_match_the_code():
    assert TASK["instruction"] == fixture.INSTRUCTION
    assert TASK["run_limit"] == f"{fixture.MAX_STEPS:,} steps ({fixture.MAX_STEPS // fixture.CONTROL_HZ} s at 20 Hz)"
    assert fixture.CONTROL_HZ == grading.CONTROL_HZ == 20
    assert f"at most {fixture.MAX_STEPS:,} actions" in README


def test_setup_numbers_match_the_code():
    setup = TASK["setup"]
    assert f"front edge along x = {1000 * fixture.FRONT_X:.0f}" in setup
    assert f"case is {1000 * L.CASE_H:.0f} tall" in setup and "about 8 kg" in setup and L.MASS == 8.
    assert f"White keys {1000 * L.WHITE_PITCH:.0f} apart, {1000 * (L.WHITE_X[1] - L.WHITE_X[0]):.0f} long, tops " \
           f"{1000 * L.WHITE_TOP:.0f} above the table" in setup
    assert f"goes down {1000 * L.KEY_TRAVEL:.0f}" in setup and f"once it is {1000 * L.KEY_SPEAKS:.1f} down" in setup
    lo, hi = L.KEY_RATE * L.KEY_PRELOAD, L.KEY_RATE * (L.KEY_PRELOAD + L.KEY_TRAVEL)
    assert f"about {lo:.1f} to {hi:.1f} N" in setup
    ys = [1000 * fixture.key_top(n)[1] for n in L.PHRASE]
    assert "y = {:.0f}, {:.0f} and {:.0f}".format(*ys) in setup
    assert f"{1000 * L.FLAP_T:.0f} thick wooden board {1000 * L.FLAP_W:.0f} wide and {1000 * L.FLAP_H:.0f} tall" in setup
    hinge_x = 1000 * (fixture.FRONT_X + L.HINGE[0])
    assert f"hinged along its bottom edge at x = {hinge_x:.0f}, z = {1000 * L.HINGE[1]:.0f}" in setup
    open_ = 1000 * fixture.flap_corner(L.FLAP_OPEN)
    shut = 1000 * fixture.flap_corner(0.)
    assert f"open at x = {open_[0]:.0f}, z = {open_[2]:.0f}" in setup and f"(to x = {shut[0]:.0f})" in setup
    assert f"about {L.TOP_EDGE_PUSH_N:.0f} N at the top edge" in setup
    assert f"holds about {1000 * L.RESERVOIR_M3:.1f} litres" in setup
    assert f"for about {Air(volume=L.RESERVOIR_M3).seconds_left(1):.0f} s" in setup
    assert "C2 to D5" in setup and L.KEY_NAMES[0] == "C2" and L.KEY_NAMES[-1] == "D5"
    assert f"pulled out {1000 * L.MAIN_STOP_PULL:.0f}" in setup


def test_scoring_numbers_match_the_code():
    scoring = TASK["scoring"]
    assert f"within {audio.CENTS:.0f} cents" in scoring and f"within {audio.WITHIN_DB:.0f} dB" in scoring
    assert f"at least {audio.ABOVE_NOISE_DB:.0f} dB above" in scoring
    assert f"{audio.WINDOW / audio.RATE:.3f} s analysis windows every 50 ms" in scoring
    assert "every key from C2 to D5 (all 39)" in scoring and audio.CALIBRATION_KEYS == L.KEY_NAMES
    assert f"for {audio.CAL_NOTE_S:.1f} s in turn, {audio.CAL_GAP_S:.1f} s apart" in scoring
    assert f"at least {audio.CROSS_MARGIN_DB:.0f} dB above what the keys already counted" in scoring
    assert f"H below is {grading.HOLD_S:.1f} s" in scoring and f"gaps of {grading.BRIDGE_S:.1f} s or less" in scoring
    assert f"at most {grading.MAX_GAP_S:.1f} s (two frames) after" in scoring
    assert f"at most {grading.DETACHED_GAP_S:.1f} s of silence" in scoring
    assert f"by at most {grading.MAX_OVERLAP_S:.1f} s" in scoring
    assert f"more than {audio.SILENT_DB:.0f} dB below" in scoring
    assert f"more than {grading.OTHER_KEYS_S:.1f} s in total" in scoring
    assert f"more than {1000 * grading.MOVED_M:.0f} mm from its marks" in scoring
    assert f"more than {1000 * grading.VOID_MOVED_M:.0f} mm" in scoring
    assert f"tipped more than {math.degrees(grading.VOID_TILT_RAD):.0f} degrees" in scoring
    assert f"for at least {grading.RELEASE_S:.0f} s" in scoring
    assert f"at least {1000 * L.KEY_SPEAKS:.1f} mm down" in scoring
    assert "0, 0.25, 0.5, 0.75, 0.85, 1.0" in scoring and grading.TIERS == (.25, .5, .75, .85, 1.)


def test_context_and_fidelity_numbers_match_the_code():
    context, fidelity = TASK["context"], TASK["fidelity"]
    assert f"about {Air(volume=L.RESERVOIR_M3).seconds_left(1):.0f} s" in context and "under 3.1 s" in context
    assert Air(volume=L.RESERVOIR_M3).seconds_left(1) < 3.1
    assert f"about {L.TOP_EDGE_PUSH_N:.0f} N" in context
    stroke = 1000 * (fixture.flap_corner(L.FLAP_OPEN)[0] - fixture.flap_corner(.12)[0])
    assert 28. < stroke < 35. and "about 30 mm shut" in context
    assert f"constant {L.FLAP_TORQUE_NM:.2f} N m" in fidelity and f"{L.WORK_PA:.0f} Pa" in fidelity
    assert f"({1000 * L.FEEDER_M3_PER_RAD:.1f} L)" in fidelity and f"{1000 * L.RESERVOIR_M3:.1f} L reservoir" in fidelity
    assert f"{L.EMPTY_PA:.0f} to {L.FULL_PA:.0f} Pa" in fidelity
    assert f"about {1000 * L.REED_FLOW_M3S:.1f} L/s" in fidelity
    assert "all 39 keys" in fidelity and "no loop joints" in fidelity


def test_readme_numbers_match_the_code():
    assert f"{L.FLAP_TORQUE_NM:.2f} N m" in README and f"{L.TOP_EDGE_PUSH_N:.1f} N" in README
    assert f"{Air(volume=L.RESERVOIR_M3).seconds_left(1):.2f} s in the model" in README
    for name in L.PHRASE:
        x, y, z = 1000 * fixture.key_top(name)
        assert f"({x:.0f}, {y:.0f}, {z:.0f})" in README
    assert f"y = {1000 * fixture.PUMP_Y:.0f}" in README


def test_text_does_not_identify_anyone_or_any_machine():
    for path in ROOT.rglob("*"):
        if path.is_file() and path.suffix in (".py", ".json", ".md", ".toml", ".usda"):
            text = path.read_text()
            assert not re.search(r"/Us" r"ers/|/ho" r"me/|@[a-z]+\.(edu|com)\b", text), path


def test_dots_are_on_the_phrase_keys():
    assert tuple(L.DOT_COLOURS) == L.PHRASE == grading.PHRASE
    assert np.isclose(L.frequency("E4"), 329.63, atol=.01)


def test_readme_reports_the_final_isaac_trials():
    for text in ("run 8 on the final package files", "success at step 464 of 2,400", "4.7, 4.7, 4.4 s", "0.35 s before",
                 "0.81 mm", "555 s (58 model requests)", "14.2 L in 8 strokes", "0.0** (all 2,400 steps)"):
        assert text in README, text
