"""task.json, README and code must say the same thing, in the official format, without naming the authors."""

import json
import sys
from pathlib import Path
import re
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(Path(__file__).resolve().parent)]
from rdb_window_wall import cleaner_model as m, grading  # noqa: E402

TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()
FIXED_SCENE = (ROOT / "rdb_window_wall" / "fixed_scene.py").read_text()


def mm(value):
    return f"{1000 * value:.0f}"


def sy(value):
    return f"{1000 * value:+.0f}"


def xy(un):
    world = m.to_world((*un, 0.))
    return f"({1000 * world[0]:.0f}, {1000 * world[1]:+.0f})"


def fixed_hz():
    return int(re.search(r"CONTROL_HZ = (\d+)", FIXED_SCENE).group(1))


AREA = 1e6 * (m.BAND_U[1] - m.BAND_U[0]) * (m.BAND_Z[1] - m.BAND_Z[0])
REACH_TCP = .220  # the reach search quoted in the README: furthest gripper centre point along the outdoor face
PAD_PAST_GRIP = .130


class TaskTextTest(unittest.TestCase):
    def test_official_fields(self):
        self.assertEqual(list(TASK), ["title", "category", "keywords", "instruction", "context", "setup", "scoring",
                                      "run_limit", "reset_notes", "fidelity", "scores"])
        self.assertNotIn("media", TASK)
        self.assertEqual(TASK["category"], "Societal relatability")
        self.assertTrue(README.startswith("# " + TASK["title"]))

    def test_scores_are_the_measured_ones(self):
        # Run 2 (seed 0, this scene as drawn): demonstration 1.0 at step 1,174 of 2,400 (271 s), Dream-1 0.0 over
        # all 2,400 steps (416 s, 56 requests). Run 1 (earlier drawing, same physics) is kept as history.
        self.assertEqual(TASK["scores"], "Dream-1 run: 0.0; demo: 1.0")
        record = json.loads((ROOT / "validation.json").read_text())
        self.assertEqual(record["scripted"]["steps"], 1174)
        self.assertEqual(record["dream_1"]["steps"], 2400)
        self.assertEqual(record["scripted"]["score"], 1.0)
        self.assertEqual(record["dream_1"]["score"], 0.0)
        self.assertIn("271.16 s", README)
        self.assertIn("415.65 s", README)
        self.assertEqual(1174 / fixed_hz(), 58.7)
        self.assertIn("display only", TASK["fidelity"])
        # Nothing may describe the current version as unrun or unchecked.
        for text in (README, json.dumps(TASK)):
            lowered = text.lower()
            for phrase in ("pending", "not yet run", "not yet in the simulator", "has not been seen", "unverified",
                           "must be run again", "has not been run", "not been rendered"):
                self.assertNotIn(phrase, lowered)

    def test_instruction_and_limits_match_the_code(self):
        instruction = "".join(re.findall(r'"([^"]*)"', FIXED_SCENE[FIXED_SCENE.index("INSTRUCTION = ("):].split(")")[0]))
        self.assertEqual(TASK["instruction"], instruction)
        steps = int(re.search(r"MAX_STEPS = (\d+)", FIXED_SCENE).group(1))
        hz = int(re.search(r"CONTROL_HZ = (\d+)", FIXED_SCENE).group(1))
        self.assertEqual(TASK["run_limit"], f"{steps:,} steps ({steps // hz} seconds at {hz} Hz)")
        self.assertEqual(hz, grading.CONTROL_HZ)
        self.assertEqual(int(re.search(r"SUCCESS_HOLD_S = (\d+)", FIXED_SCENE).group(1)), grading.HOLD_S)
        self.assertIn(f"{steps:,} steps", README)

    def test_instruction_does_not_prescribe_the_method(self):
        text = TASK["instruction"].lower()
        for word in ("left", "right", "grip", "grasp", "first", "slide", "handle", "hold the", "through"):
            self.assertNotIn(word, text)

    def test_setup_numbers_match_the_model(self):
        setup = TASK["setup"]
        for phrase in (
            f"plywood {mm(m.WALL_T)} thick, {mm(m.WALL_U[1] - m.WALL_U[0])} wide (y {sy(m.WALL_U[0])} to {sy(m.WALL_U[1])}) and {mm(m.WALL_H)} tall",
            f"indoor face at x = {mm(m.WALL_X)}",
            f"beyond x = {mm(m.WALL_X + m.WALL_T)} is outdoors",
            f"y {sy(m.OPENING_U[0])} to {sy(m.PANE_X[1])} and z {mm(m.OPENING_Z[0])} to {mm(m.OPENING_Z[1])}",
            f"a clear opening {mm(m.OPENING_U[1] - m.OPENING_U[0])} wide x {mm(m.OPENING_Z[1] - m.OPENING_Z[0])} tall",
            f"A {mm(m.STILE_U[1] - m.STILE_U[0])} wide stile at y {sy(m.STILE_U[0])} to {sy(m.STILE_U[1])}",
            f"clear glass {mm(m.PANE_X[1] - m.PANE_X[0])} wide x {mm(m.PANE_Z[1] - m.PANE_Z[0])} tall x {mm(m.PANE_T)} thick, {m.PANE_MASS:.2f} kg",
            f"y {sy(m.PANE_X[0])} to {sy(m.PANE_X[1])}, z {mm(m.PANE_Z[0])} to {mm(m.PANE_Z[1])}",
            f"glass from x {mm(m.WALL_X + m.WALL_T - m.PANE_T)} to {mm(m.WALL_X + m.WALL_T)}",
            f"{mm(m.BAND_U[1] - m.BAND_U[0])} long and {mm(m.BAND_Z[1] - m.BAND_Z[0])} tall, y {sy(m.BAND_U[0])} to {sy(m.BAND_U[1])}, z {mm(m.BAND_Z[0])} to {mm(m.BAND_Z[1])}",
            f"its near end is {mm(m.BAND_U[0] - m.STILE_U[1])} from the stile",
            f"{mm(m.PAD_BASE)} base and {mm(m.PAD_HEIGHT)} height",
            f"{mm(m.BODY_T)} thick",
            f"{mm(m.BAR_SIZE[0])} long x {mm(m.BAR_SIZE[1])} wide x {mm(m.BAR_SIZE[2])} thick",
            f"its top {mm(m.HALF_THICK)} from the pad face and a {mm(m.BAR_Z[0] - m.BODY_T)} gap under it",
            f"{mm(m.MASS)} g each",
            f"{mm(2 * m.MAGNET_X)} apart",
            f"at {xy(m.START_UN['inside'])}, in front of the fixed light, apex towards the wall",
            f"at {xy(m.START_UN['outside'])}, in front of the stile, apex towards the robot",
            f"than y {sy(REACH_TCP)}",
            f"more than {mm(PAD_PAST_GRIP)} from the centre point",
            f"at least {mm(m.BAND_U[0] - REACH_TCP - PAD_PAST_GRIP)} short of the band",
            "a search, not a simulator run", "read off the first frames of the Dream-1 trial", "safety cord is taken off",
            "3 to 8 mm single glazing", "What the policy's cameras see", "Overhead camera", "Left wrist camera", "Right wrist camera",
        ):
            self.assertIn(phrase, setup)
        for phrase in (xy(m.START_UN["inside"]), xy(m.START_UN["outside"]), "y = +220", "60 short of the band", "y -380 to +50",
                       "y +410 to +560, z 280 to 310", "550 x 320 x 4", "1280 wide, 560 tall"):
            self.assertIn(phrase, README)
        self.assertAlmostEqual(m.PANE_MASS, 2500 * .55 * .32 * .004, places=2)
        # The reach figures hang together: the band begins beyond anything a hand-held pad can reach, and no
        # point of a pad is further from a gripper that holds its half by the bar than the figure quoted.
        corners = np.c_[m.TRIANGLE, np.zeros(3)]
        ends = np.array(((-m.BAR_SIZE[0] / 2, 0., m.GRIP_POINT[2]), (m.BAR_SIZE[0] / 2, 0., m.GRIP_POINT[2])))
        furthest = max(np.linalg.norm((c - e)[:2]) for c in corners for e in ends)
        self.assertLess(furthest, PAD_PAST_GRIP)
        self.assertGreaterEqual(m.BAND_U[0] - REACH_TCP - PAD_PAST_GRIP, .050)
        try:
            from rdb_window_wall import scripted
        except ImportError:  # pragma: no cover
            return
        self.assertIn(f"its centre at y {sy(scripted.MATE_U)}", setup)
        self.assertAlmostEqual(REACH_TCP - (scripted.MATE_U - scripted.GRIP_OFFSET), .075)  # "75 short of the limit"
        self.assertIn(f"gripper centre at {sy(scripted.MATE_U - scripted.GRIP_OFFSET)}, 75 short", README)

    def test_overhead_camera_sees_the_band_and_both_halves(self):
        """The figures quoted for the overhead camera, recomputed from the station's calibration. (The check
        that the left arm at its ready pose does not hide the band needs the arm's hulls and is in the README.)"""
        try:
            from yam_arena import station
            from yam_arena.urdf import quat_xyzw_to_matrix
        except ImportError as error:  # pragma: no cover
            self.skipTest(f"needs yam_arena: {error}")
        camera = station.top_camera()
        rot, eye, lens = quat_xyzw_to_matrix(np.array(camera.rot_xyzw)), np.array(camera.pos), camera.lens

        def pixel(point):  # in the 320 x 176 policy image
            c = rot.T @ (m.to_world(point) + (0., 0., station.TABLE_TOP) - eye)
            self.assertLess(c[2], 0.)  # in front of the camera
            return np.array(((lens.cx + lens.fx * c[0] / -c[2]) * 320 / lens.width, (lens.cy - lens.fy * c[1] / -c[2]) * 176 / lens.height))

        n = m.FACE_Y["outside"]
        corners = np.array([pixel((u, n, z)) for u in m.BAND_U for z in m.BAND_Z])
        self.assertAlmostEqual(np.ptp(corners[:, 0]), 38., delta=1.5)  # "about 38 pixels long and 4 tall"
        self.assertAlmostEqual(np.ptp(corners[:, 1]), 4., delta=1.)
        self.assertIn("about 38 pixels long and 4 tall", TASK["setup"])
        self.assertIn(f"columns {corners[:, 0].min():.0f} to {corners[:, 0].max():.0f}, rows {corners[:, 1].min():.0f} to {corners[:, 1].max():.0f}", README)
        for point in ((*m.START_UN["inside"], .02), (*m.START_UN["outside"], .02), *((u, n, z) for u in m.BAND_U for z in m.BAND_Z)):
            u, v = pixel(point)
            self.assertTrue(10 < u < 310 and 10 < v < 166)  # well inside the image
        # The camera is indoors: it sees the band through the glass, and the sight line passes through the glass
        # itself, not through the wall beside, above or below it.
        cam = m.to_pane(eye - (0., 0., station.TABLE_TOP))
        self.assertGreater(cam[1], 0.)
        for u in m.BAND_U:
            for z in m.BAND_Z:
                t = (m.WALL_N[1] - n) / (cam[1] - n)
                cross = np.array((u, n, z)) + t * (cam - (u, n, z))  # where the sight line leaves the wall's thickness
                self.assertTrue(m.PANE_X[0] < cross[0] < m.PANE_X[1] and m.PANE_Z[0] < cross[2] < m.PANE_Z[1])
        v_top = pixel((0., m.WALL_MID_N, m.WALL_H))[1]
        self.assertLess(v_top, 0.)  # "its top edge is out of view"

    def test_scoring_numbers_match_the_scorer(self):
        scoring = TASK["scoring"]
        for phrase in (
            f"within {mm(grading.PAIR_OFFSET)} mm of each other",
            f"at least {100 * grading.CLEAR_OF_GRIPPER:.0f} cm from the middle of each half's handle bar",
            f"moving at {mm(grading.SLIDING)} mm/s or more",
            f"under {100 * grading.STILL:.0f} cm/s",
            f"for the last {grading.HOLD_S} s",
            f"{mm(m.BAND_U[1] - m.BAND_U[0])} x {mm(m.BAND_Z[1] - m.BAND_Z[0])} mm, {AREA:,.0f} mm2",
            f"{m.CELLS} squares of {mm(m.CELL)} mm",
            f"at least {100 * grading.HALF_AREA:.0f} % of the band's area ({AREA * grading.HALF_AREA:,.0f} mm2)",
            f"at least {100 * grading.FULL_AREA:.0f} % ({AREA * grading.FULL_AREA:,.0f} mm2)",
            "where no marker remains", "over the top of the wall or round its ends",
            "within 5 degrees", "no gap over 2 mm", "Possible scores: 0, 0.25, 0.5, 0.75, 1.0.",
        ):
            self.assertIn(phrase, scoring)
        self.assertNotIn("centre line", scoring)
        self.assertAlmostEqual(grading.FLAT_COS, 0.9961947, places=6)
        self.assertAlmostEqual(grading.PAD_GAP_RANGE[1], .002)
        # The README's real-trial square counts are the same thresholds.
        self.assertIn(f"Under {m.CELLS * grading.HALF_AREA:.0f} squares: 0.5", README)
        self.assertIn(f"{m.CELLS * grading.FULL_AREA:.0f} or more: 1.0", README)

    def test_fidelity_and_context_numbers_match_the_model(self):
        fidelity = TASK["fidelity"]
        self.assertIn(f"{m.HOLD_AT_RATED_N:.0f} N through the thickest rated glass ({mm(m.RATED_GLASS)} mm)", fidelity)
        self.assertIn(f"{m.hold_force()[0]:.1f} N through this {mm(m.PANE_T)} mm pane", fidelity)
        self.assertIn(f"{m.hold_force(0.)[0]:.0f} N pad to pad", fidelity)
        self.assertIn(f"{m.hold_force(.030)[0]:.0f} N at a 30 mm gap", fidelity)
        self.assertIn(f"friction is {m.PAD_FRICTION}", fidelity)
        self.assertIn(f"{m.CELLS} squares of {mm(m.CELL)} mm", fidelity)
        self.assertIn("estimate", fidelity)
        self.assertIn("reads the simulator's poses", fidelity)
        self.assertIn("55 mm below the inside half", fidelity)
        context = TASK["context"]
        self.assertLessEqual(len(context.split()), 50)
        self.assertIn("magnetic window cleaner", context.lower())
        # "lifted about 6 mm off the glass ... it drops": friction no longer carries the half's weight there.
        self.assertLess(m.PAD_FRICTION * m.hold_force(m.PANE_T + .006)[0], m.MASS * 9.81)
        self.assertGreater(m.PAD_FRICTION * m.hold_force(m.PANE_T + .004)[0], m.MASS * 9.81)

    def test_readme_tables_match_the_model(self):
        for gap, text in ((0., "32.4 N"), (.004, "17.7 N"), (.008, "10.0 N"), (.012, "6.0 N"), (.020, "2.5 N"), (.030, "1.0 N")):
            self.assertEqual(f"{m.hold_force(gap)[0]:.1f} N", text)
            self.assertIn(text, README)
        for phrase in ("ESTIMATE", "Third-party assets and licences", "seed 0",
                       "python -m rdb_window_wall.run scripted", "Real-world build", "Scoring a real trial",
                       "Reach: why the band cannot be wiped with a hand-held half", "What the policy's cameras see at the start",
                       "This is a numerical search, not a proof"):
            self.assertIn(phrase, README)

    def test_no_text_identifies_the_authors(self):
        for name, text in (("task.json", json.dumps(TASK)), ("README.md", README)):
            lowered = text.lower()
            for word in ("/users/", "/home/", "worklocal", "university", "institute", "our lab", "laborator"):
                self.assertFalse(word in lowered, f"{word} in {name}")
            self.assertIsNone(re.search(r"[\w.]+@[\w-]+\.\w+", text), f"e-mail address in {name}")

    def test_package_files(self):
        pyproject = (ROOT / "pyproject.toml").read_text()
        self.assertIn('rdb_window_wall = ["window.usda"]', pyproject)
        for name in ("__init__", "adapter", "build_scene", "cleaner_model", "dream_audit", "fixed_scene", "grading",
                     "magnet", "recording", "run", "scripted"):
            self.assertTrue((ROOT / "rdb_window_wall" / f"{name}.py").is_file(), name)
        self.assertTrue((ROOT / "rdb_window_wall" / "window.usda").is_file())
        self.assertFalse((ROOT / "evidence").exists())
        run = (ROOT / "rdb_window_wall" / "run.py").read_text()
        self.assertIn('"peak_interarm_contact_n"', run)  # read by the batch harness; a diagnostic only
        self.assertIn('return int(args.policy == "scripted" and score != 1.)', run)
        self.assertIn('"phase_diagnostics"', run)
        self.assertNotIn("interarm", (ROOT / "rdb_window_wall" / "grading.py").read_text())


if __name__ == "__main__":
    unittest.main()
