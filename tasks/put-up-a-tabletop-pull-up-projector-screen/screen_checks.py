"""Geometric and timing checks for a released tabletop screen."""
import numpy as np


def distance_to_segment(point, start, end):
    axis = end - start
    length2 = axis @ axis
    fraction = np.clip((point - start) @ axis / length2, 0.0, 1.0) if length2 else 0.0
    return float(np.linalg.norm(point - (start + fraction * axis)))


def release_time(qualified, now, started):
    if not qualified:
        return None, 0.0
    if started is None:
        return now, 0.0
    return started, now - started
