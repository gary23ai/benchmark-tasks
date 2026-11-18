"""The result-file diagnostics, exercised without a simulator (needs inspect_robots and yam_arena)."""

import json
import unittest

import numpy as np

try:
    from rdb_nested_cups import adapter, cup_model as cup, fixed_scene as fixture, run, scripted
    from fake_world import FakeWorld
except ImportError as error:  # pragma: no cover
    raise unittest.SkipTest(f"needs inspect_robots and yam_arena: {error}")

LINKS = ["base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right"]


def fake_arena():
    arena = object.__new__(adapter.CupArena)
    arena.link_names, arena.tick, arena.history = LINKS, 0, []
    arena.fit, arena.cup_on_cup = {"bonded": True}, None
    arena.cups = np.array([[*cup.OUTER_START, 0., 0., 0., 1.], [*cup.INNER_START, 0., 0., 0., 1.]])
    arena.velocity = np.zeros((2, 6))
    return arena


class DiagnosticsTest(unittest.TestCase):
    def test_step_row_phase_snapshots_and_peaks(self):
        arena = fake_arena()
        world = FakeWorld()
        reference = scripted.CupReference(world)
        actions = reference.sequence()
        forces = np.zeros((2 * len(LINKS), 2, 3))
        forces[LINKS.index("tip_left"), 1] = (0., 30., 4.)  # left arm, inner cup
        forces[LINKS.index("tip_right"), 1] = (0., -30., 4.)
        for _ in range(1500):
            commanded = np.asarray(next(actions).actions[0].data, dtype=float)
            reference.measured = world.step(commanded)
            tcp = np.array([adapter.ik.tcp_pose(commanded[i:i + 6], fixture.BASES[arm])[0]
                            for arm, i in zip(fixture.ARMS, (0, 7), strict=True)])
            arena.history.append(arena._diagnostics(commanded, commanded, tcp, forces))
            arena.tick += 1
        row = arena.history[0]
        self.assertEqual(row["tick"], 1)
        self.assertAlmostEqual(row["inner_above_outer_mm"], 1000 * cup.SEATED_PITCH, places=2)
        self.assertEqual(row["arms"]["left"]["cup_contact"]["inner"]["net_n"], [0., 0., 8.])
        self.assertEqual(set(row["arms"]["left"]["cup_contact"]["inner"]["links_n"]), {"tip_left", "tip_right"})
        self.assertEqual(row["arms"]["right"]["cup_contact"]["outer"]["links_n"], {})
        snapshots = run.phase_snapshots(reference, arena)
        names = [s["phase"] for s in snapshots]
        for expected in ("parked", "right_closed_on_outer_cup", "left_at_inner_rim_open", "left_closed_on_inner_rim",
                         "pulled_out", "left_released", "right_parked"):
            self.assertIn(expected, names)
        closed = snapshots[names.index("left_closed_on_inner_rim")]
        self.assertEqual(closed["simulator"]["tick"], closed["step"])
        rim = np.array(cup.INNER_START) + (-scripted.PINCH_RADIUS, 0., cup.HEIGHT)
        np.testing.assert_allclose(closed["robot"]["left"]["tcp_measured_mm"], 1000 * rim, atol=.6)
        self.assertAlmostEqual(closed["simulator"]["arms"]["left"]["opening_commanded"],
                               scripted.INNER_FINGER / scripted.OPEN, places=3)
        peak = run.peaks(arena.history)
        self.assertAlmostEqual(peak["left_on_inner"]["peak_vertical_net_n"], 8.)
        self.assertGreater(peak["left_on_inner"]["peak_total_contact_n"], 60.)
        json.dumps({"phases": snapshots, "peaks": peak, "row": row}, allow_nan=False)

    def test_bond_state_is_read_from_the_cups_relative_pose(self):
        arena = fake_arena()
        arena.reset_height = cup.SEATED_PITCH
        fit = arena._read_fit()
        self.assertTrue(fit["bonded"])
        self.assertAlmostEqual(fit["inner_above_outer_mm"], 1000 * cup.SEATED_PITCH, places=2)
        arena.cups[1, 2] += .0005  # the inner cup has moved half a millimetre in the outer one
        self.assertFalse(arena._read_fit()["bonded"])
        arena.cups[1, 2] -= .0005
        arena.cups[:, 2] += .05  # the pair lifted together is still bonded
        self.assertTrue(arena._read_fit()["bonded"])
        self.assertIn("error", arena._fit_joint())  # no simulator stage here: reported, not raised


if __name__ == "__main__":
    unittest.main()
