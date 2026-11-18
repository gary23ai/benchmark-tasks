"""Kinematic dry run of the diagnostic probe (needs yam_arena and inspect_robots; skipped otherwise)."""

import json
import unittest

import numpy as np

try:
    from yam_arena import ik
    from rdb_nested_cups import adapter, cup_model as cup, fixed_scene as fixture, probe
except ImportError as error:  # pragma: no cover
    raise unittest.SkipTest(f"needs yam_arena and inspect_robots: {error}")


@unittest.skipUnless(cup.PROBE, "probe mode is off")
class ProbeDryRunTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from fake_world import FakeWorld
        cls.reference, trace = probe.ProbeReference(FakeWorld()), []
        for chunk in cls.reference.sequence():
            values = np.asarray(chunk.actions[0].data, dtype=float)
            cls.reference.measured = values
            trace.append(values)
            if (cls.reference.phases and cls.reference.phases[-1]["phase"] == "done") or len(trace) > 5000:
                break
        cls.trace = np.array(trace)

    def test_all_ten_tests_fit_the_step_limit(self):
        self.assertLess(len(self.trace), fixture.MAX_STEPS - 200)
        names = [p["phase"] for p in self.reference.phases]
        for test in ("right_side_outer", "cyl80_slow", "cyl80_fast", "cyl66_slow", "box80_slow", "nested_slow",
                     "rim_y_slow", "rim_x_slow", "rim_diag_slow", "pinch_slow"):
            for stage in ("open_at_target", "closed", "held_2s", "released"):
                self.assertIn(f"{test}:{stage}", names)
        json.dumps({"phases": self.reference.phases, "report": self.reference.report}, allow_nan=False)

    def test_reachable_and_gentle(self):
        self.assertLess(self.reference.worst_ik, .001)
        for i in (0, 7):
            joints = self.trace[:, i:i + 6]
            self.assertTrue(np.all(joints >= ik.LOWER - 1e-9) and np.all(joints <= ik.UPPER + 1e-9))
            step = np.abs(np.diff(joints, axis=0)).max(axis=0)
            self.assertLess(step[:3].max(), .12)
            self.assertLess(step[3:].max(), .20)

    def test_targets(self):
        at = {p["phase"]: p for p in self.reference.phases}
        for name, body in (("cyl80_slow", "probe80"), ("cyl66_slow", "probe66"), ("box80_slow", "probebox")):
            _, _, height, _, base = cup.PROBE_BODIES[body]
            np.testing.assert_allclose(at[f"{name}:open_at_target"]["left"]["tcp_measured_mm"],
                                       [1000 * base[0], 1000 * base[1], 1000 * (height - .006)], atol=.6)
        self.assertEqual(self.reference.report["pinch_slow_finger_target_opening"], 0.)
        self.assertAlmostEqual(self.reference.report["cyl80_slow_expected_opening_if_held"], .852, places=3)
        # The slow close advances each finger under 0.2 mm per control step.
        closing = at["cyl80_slow:closed"]["step"] - at["cyl80_slow:open_at_target"]["step"]
        self.assertEqual(closing, 80)


if __name__ == "__main__":
    unittest.main()
