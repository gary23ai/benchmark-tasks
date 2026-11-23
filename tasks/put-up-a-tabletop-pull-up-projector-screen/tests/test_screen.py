import numpy as np
import pytest
from screen_checks import distance_to_segment, release_time


def test_distance_includes_middle_and_clamps_endpoints():
    a = np.zeros(3)
    b = np.array([0., 0.43, 0.])
    assert distance_to_segment(np.array([0.04, 0.033, 0.]), a, b) == pytest.approx(0.04)
    assert distance_to_segment(np.array([0., 0.46, 0.]), a, b) == pytest.approx(0.03)


def test_two_seconds_must_be_continuous():
    assert release_time(True, 3., None) == (3., 0.)
    assert release_time(True, 4.95, 3.)[1] < 2.
    assert release_time(False, 5., 3.) == (None, 0.)
    assert release_time(True, 5.05, None) == (5.05, 0.)
    assert release_time(True, 7.05, 5.05)[1] == pytest.approx(2.)
