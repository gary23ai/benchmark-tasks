"""task.json, README and the code say the same thing."""

import json
import re
from pathlib import Path

from rdb_filter_clamp import fixed_scene as F
from rdb_filter_clamp import grading as G
from rdb_filter_clamp import layout as L

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()


def test_required_fields():
    for key in ("title", "category", "instruction", "context", "setup", "scoring", "run_limit", "fidelity", "scores", "reset_notes"):
        assert TASK[key]
    assert TASK["category"] == "Other"
    assert TASK["scores"].startswith("Dream-1 run: 0.0 (2,400 steps;") and "demo: 1.0 (993 steps)" in TASK["scores"]
    for text in (json.dumps(TASK), README):
        assert "PENDING" not in text and "unverified" not in text.lower()


def test_instruction_and_run_limit_match_the_code():
    assert TASK["instruction"] == F.INSTRUCTION
    assert TASK["run_limit"].startswith(f"{F.MAX_STEPS:,} steps")
    assert F.CONTROL_HZ == G.CONTROL_HZ == 20
    assert f"{F.MAX_STEPS:,} actions" in README


def test_setup_numbers_match_the_code():
    setup = TASK["setup"]
    for p in (F.HOLDER_START, F.FUNNEL_START):
        assert f"({1000 * p[0]:.0f}, {1000 * p[1]:.0f})" in setup
    assert f"({1000 * F.CLAMP_START[0]:.0f}, {1000 * F.CLAMP_START[1]:.0f})" in setup
    assert f"{1000 * L.JOINT_Z:.0f} above the table" in setup
    assert f"{1000 * 2 * L.BODY_R:.0f} across" in setup and f"{1000 * L.FUNNEL_H:.0f} tall" in setup


def test_scoring_numbers_match_the_code():
    s = TASK["scoring"]
    assert list(G.TIERS.values()) == [.2, .4, .6, .8, 1.] and "Possible scores: 0, 0.2, 0.4, 0.6, 0.8, 1.0" in s
    assert f"within {1000 * G.SEAT_GAP:.0f} mm" in s and f"{G.SEAT_TILT_DEG} degrees" in s
    assert f"within {1000 * G.CENTRED:.1f} mm" in s and f"within {1000 * G.MEMBRANE_LIMIT:.0f} mm of its tick" in s
    assert f"within {1000 * G.CLAMP_CENTRED:.0f} mm" in s and f"{1000 * G.CLEAR_AXIS:.0f} mm" in s
    assert f"past {G.FLASK_TIPPED_DEG:.0f} degrees" in s and f"more than {100 * G.FLASK_PUSHED:.0f} cm" in s


def test_text_does_not_identify_anyone():
    for text in (json.dumps(TASK), README):
        assert not re.search(r"/Users/|/home/|@[a-z]+\.(edu|com)\b", text)


def test_readme_reports_the_measured_runs():
    for text in ("| Scripted demonstration | 1.0 | 993 of 2,400", "| Dream-1 (`dreamlabs/dream-1-beta`) | 0.0 | 2,400 of 2,400",
                 "250 s of wall time for its 993", "658 s for the full 2,400 steps, with 90 requests",
                 "`membrane_offset_mm` 0.57 at the end", "`membrane_peak_mm` 0.58", "**Run 4 (final scene and scorer): demo 1.0, Dream-1 0.0.**",
                 "**Run 3 (previous scorer): demo 1.0 (993 steps), Dream-1 0.0.**",
                 "all 101 steps of the fitting", "1.39 and 1.59 degrees",
                 "**Run 1 (scripted, 2,400 steps): 0.0.**", "**Run 2 (scripted, 2,400 steps): 0.0, but the task itself was done.**",
                 "counted only while the ground faces touch"):
        assert text in README


def test_run_4_numbers_sit_inside_the_scorer_limits():
    lo, hi = L.SEATED_DEG
    assert lo <= 1.39 <= hi and lo <= 1.59 <= hi            # jaws after release
    assert .57 / 1000 < G.MEMBRANE_LIMIT                     # run 4's membrane offset at the end
    assert 1.29 / 1000 <= G.CENTRED                          # funnel off centre at the end
    assert .65 / 1000 <= G.CLAMP_CENTRED and 1.15 <= G.CLAMP_TILT_DEG
    assert 63.92 >= G.HOLD_OPENING_MM[0] and 63.92 <= G.HOLD_OPENING_MM[1]   # the measured pad opening on the funnel
