"""Regression cases for the distinction between mixing and portioning."""

import unittest

import numpy as np

from scoring import grade


class PortionScoring(unittest.TestCase):
    def setUp(self):
        # Each color contributes one row of three to each serving.
        self.points = (
            np.array(
                [[x, y, 9] for y in (-40, 40, -29, 29) for x in (-11, 0, 11)], float
            )
            / 1000
        )
        self.rotations = np.broadcast_to(np.eye(3), (12, 3, 3))
        self.flags = dict(
            tool_clear=True,
            pan_stable=True,
            spilled=False,
            bare_hand=False,
            tool_used=True,
            one_layer=True,
        )

    def judge(self, points=None, **flags):
        return grade(
            self.points if points is None else points,
            self.rotations,
            **(self.flags | flags),
        )["score"]

    def test_equal_servings(self):
        self.assertEqual(self.judge(), 1.0)
        # Turning the pan or swapping which pile is called the first makes no difference.
        turned = self.points.copy()
        turned[:, :2] = turned[:, [1, 0]] * [-1, 1]
        self.assertEqual(self.judge(turned), 1.0)

    def test_intermediate(self):
        nearby = self.points.copy()
        nearby[:, 1] -= 0.012 * np.sign(nearby[:, 1])
        self.assertEqual(self.judge(nearby), 0.5)
        nearby[:, 1] -= 0.009 * np.sign(nearby[:, 1])
        self.assertEqual(self.judge(nearby), 0.25)

    def test_separated_colors_and_invalid_trials(self):
        segregated = self.points[[0, 1, 2, 6, 7, 8, 3, 4, 5, 9, 10, 11]]
        self.assertEqual(self.judge(segregated), 0.0)
        for flags in (
            dict(spilled=True),
            dict(bare_hand=True),
            dict(tool_used=False),
            dict(pan_stable=False),
        ):
            self.assertEqual(self.judge(**flags), 0.0)
        self.assertEqual(self.judge(tool_clear=False), 0.5)


if __name__ == "__main__":
    unittest.main()
