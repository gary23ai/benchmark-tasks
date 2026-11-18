"""task.json, README and the code say the same thing."""

import json
import re
from pathlib import Path

from rdb_pool_bridge import fixed_scene as F
from rdb_pool_bridge import grading as G
from rdb_pool_bridge import layout as L

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "task.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()


def test_required_fields():
    for key in ("title", "category", "instruction", "context", "setup", "scoring", "run_limit", "fidelity", "scores",
                "reset_notes"):
        assert TASK[key]
    assert TASK["category"] == "Societal relatability"
    assert TASK["scores"].startswith("Dream-1 run: 0.0 (1,600 steps") and TASK["scores"].endswith("demo: 1.0 (620 steps)")
    assert "PENDING" not in json.dumps(TASK) and "PENDING" not in README


def test_instruction_and_run_limit_match_the_code():
    assert TASK["instruction"] == F.INSTRUCTION
    assert TASK["run_limit"].startswith(f"{F.MAX_STEPS:,} steps")
    assert F.CONTROL_HZ == G.CONTROL_HZ == 20


def test_numbers_in_the_text_match_the_code():
    setup, scoring = TASK["setup"], TASK["scoring"]
    assert f"cushion noses at x = {1000 * L.X_N:.0f}" in setup and f"y = {1000 * L.Y_L:.0f}" in setup
    for n, label in (("cue", "white"), ("one", "1-ball"), ("three", "3-ball")):
        x, y, _ = L.BALL_START[n]
        assert f"({1000 * x:.0f}, {1000 * y:.0f})" in setup, label
    assert f"{1000 * L.LOOP_GAP:.0f} mm" in scoring and f"{1000 * G.TIPS_ON_CLOTH:.0f} mm" in scoring
    assert f"{1000 * G.SLIDE_MIN:.0f} mm" in scoring and f"{1000 * G.BRIDGE_STILL:.0f} mm" in scoring
    assert f"{1000 * G.GRIP_ON_AXIS:.0f} mm" in scoring and f"{1000 * G.DISTURB:.0f} mm" in scoring
    assert list(G.TIERS.values()) == [.1, .4, .7, 1.] and "0, 0.1, 0.4, 0.7, 1.0" in scoring


def test_readme_reports_the_final_runs():
    for text in ("demonstration 1.0 in 620 steps", "Dream-1: 0.0", "slid 157 mm", "159 s for the demonstration"):
        assert text in README


def test_text_does_not_identify_anyone():
    home = "/" + "Users" + "/"                   # spelled apart so this file holds no local path itself
    for text in (json.dumps(TASK), README):
        assert home not in text and "/home/" not in text and not re.search(r"@[a-z]+\.(edu|com)\b", text)
