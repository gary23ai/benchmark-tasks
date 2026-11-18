"""task.json, README and the code say the same thing."""

import json
import re
from pathlib import Path

import numpy as np

from rdb_child_latch import fixed_scene as fixture
from rdb_child_latch import grading, layout as L

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()


def mm(value):
    return f"{1000 * value:.0f}"


def test_required_fields_and_media_when_uploaded():
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "reset_notes", "fidelity", "scores"):
        assert TASK[key]
    assert TASK["category"] == "Societal relatability"
    if "media" in TASK:
        assert re.fullmatch(r"M-[A-Z0-9]+", TASK["media"])
    assert TASK["scores"] == "Dream-1 run: 0.0; demo: 1.0"
    record = json.loads((ROOT / "validation.json").read_text())
    assert record["scripted"]["score"] == 1.0 and record["scripted"]["steps"] == 532
    assert record["dream_1"]["score"] == 0.0 and record["dream_1"]["steps"] == 1600
    assert not re.search(r"PENDING|not been run|unverified|to be replaced", README + json.dumps(TASK))


def test_instruction_and_run_limit_match_the_code():
    assert TASK["instruction"] == fixture.INSTRUCTION
    assert TASK["run_limit"].startswith(f"{fixture.MAX_STEPS:,} steps")
    assert fixture.CONTROL_HZ == grading.CONTROL_HZ == 20
    assert f"{fixture.MAX_STEPS:,} steps" in README
    assert f"about {mm(L.catch_open())} mm" in fixture.INSTRUCTION and f"at least {mm(grading.OPEN_M)} mm" in fixture.INSTRUCTION


def test_setup_numbers_match_the_code():
    setup = TASK["setup"]
    assert "{} front to back x {} wide x {} high".format(*map(mm, L.FRAME_SIZE)) in setup
    assert "{} x {} x {} deep inside".format(*map(mm, L.DRAWER_INNER)) in setup
    assert f"front face at x = {mm(L.FRAME_FRONT_X)}, centred on y = 0" in setup
    assert f"a built-in stop at {mm(L.OPEN_LIMIT)} open" in setup and f"about {L.RUNNER_FRICTION_N:.0f} N to move it" in setup
    assert f"{mm(L.FRONT_T)} thick and runs from {mm(L.FRONT_Z[0])} to {mm(L.FRONT_Z[1])}" in setup
    assert f"a lip {mm(L.PULL_SIZE[1])} wide (y -60 to +60), {mm(L.PULL_SIZE[2])} thick" in setup
    assert f"standing {mm(L.PULL_SIZE[0])} proud" in setup and f"middle {mm(L.PULL_Z)} above" in setup
    assert f"top is {mm(L.RAIL_Z)} above the table" in setup
    assert "2.5 in long x 1 in high x 1.5 in wide, 64 x 25 x 38" in setup and "model 48392" in setup
    assert f"{mm(L.CATCH_WIDTH)} wide and hanging {mm(L.CATCH_DROP)} below it (its underside {mm(L.CATCH_LOW_Z)}" in setup
    assert f"catching face {mm(L.CATCH_FACE_X - L.FRAME_FRONT_X)} behind the frame's front face" in setup
    assert f"arm's top is {mm(L.ARM_TOP_Z)} above the table" in setup
    assert f"{mm(L.LATCH_LENGTH)} behind the front's inner face, a hook rises to {mm(L.HOOK_TOP_Z)}" in setup
    assert f"({mm(L.ENGAGE)} mm higher than the catch's underside)" in setup
    assert f"about {np.degrees(L.RELEASE_ANGLE):.0f} degrees" in setup
    assert f"go down {np.degrees(L.LATCH_TRAVEL):.0f} degrees" in setup
    o = L.catch_open()
    gap = L.FRAME_FRONT_X - (L.drawer_x(o) + L.HINGE_X)
    assert f"pulled out {mm(o)} (1.5 in" in setup and f"then {mm(gap)} wide" in setup
    assert f"{mm(L.FRONT_Z[1] - L.ARM_TOP_Z)} below the top edge of the drawer front" in setup


def test_scoring_and_context_numbers_match_the_code():
    scoring = TASK["scoring"]
    assert f"open {mm(grading.OPEN_M)} mm or more" in scoring and f"at least {mm(grading.CLEAR_M)} mm from" in scoring
    assert f"open {mm(grading.PAST_M)} mm or more" in scoring and f"open {mm(grading.AT_CATCH_M)} mm or more" in scoring
    assert f"for the last {grading.HOLD_S} s" in scoring and f"more than {mm(grading.UNIT_SHIFT_M)} mm" in scoring
    assert f"stops it at {mm(L.catch_open())} mm" in scoring
    context = TASK["context"]
    assert len(context.split()) <= 50 and "latch" in context


def test_fidelity_states_the_model():
    fidelity = TASK["fidelity"]
    assert f"{L.LATCH_STIFFNESS:.2f} N m/rad" in fidelity and f"wound {L.LATCH_PRELOAD:.1f} rad" in fidelity
    assert f"about {L.spring_torque(0.) / (L.HOOK_FACE_X - L.HINGE_X):.1f} N at the hook" in fidelity
    assert f"stop {np.degrees(L.LATCH_TRAVEL):.0f} degrees down" in fidelity
    assert "No software rule moves, holds or releases anything" in fidelity


def test_text_does_not_identify_anyone():
    for text in (json.dumps(TASK), README):
        homes = "|".join("/" + name + "/" for name in ("Us" + "ers", "ho" + "me"))     # no local paths in the text
        assert not re.search(homes + r"|@[a-z]+\.(edu|com)\b", text)
