"""Hand-built pad shapes for the scorer's tests: a chain of 25 strips with given fold angles, put anywhere. NumPy only."""

import numpy as np

from rdb_pad_roll import pad_model as m


def chain(bends):
    """Hinge points (26, 2) of a pad whose hinges 1..24 are folded by ``bends`` (rad), strip 0 from the origin along +u."""
    points, angle = [np.zeros(2)], 0.
    for i in range(m.STRIPS):
        if i:
            angle += bends[i - 1]
        points.append(points[-1] + m.STRIP_W * np.array((np.cos(angle), np.sin(angle))))
    return np.array(points)


def section_middle(points):
    """Middle of the smallest circle round a cross-section's hinge points, near enough."""
    centre = points.mean(axis=0)
    for i in range(1, 400):
        far = points[np.linalg.norm(points - centre, axis=1).argmax()]
        centre = centre + (far - centre) / (i + 1)
    return centre


def pad(bends, middle, along=(1., 0., 0.), up=(0., 0., 1.), offsets=None):
    """Strip centres (25, 3) and rotations (25, 3, 3) of a pad with fold angles ``bends``, the middle of its
    cross-section at ``middle``, its strips lying along ``along``; ``offsets`` shifts each strip along its length."""
    a = np.asarray(along, dtype=float) / np.linalg.norm(along)
    v = np.asarray(up, dtype=float)
    v = v - a * (v @ a)
    v /= np.linalg.norm(v)
    u = np.cross(a, v)                       # (u, a, v) is right-handed: the strip's x, y and z when it lies flat
    points = chain(bends)
    points = points - section_middle(points)
    offsets = np.zeros(m.STRIPS) if offsets is None else np.asarray(offsets, dtype=float)
    centres, rotations = [], []
    for i in range(m.STRIPS):
        mid, d = (points[i] + points[i + 1]) / 2, (points[i + 1] - points[i]) / m.STRIP_W
        x = d[0] * u + d[1] * v
        centres.append(np.asarray(middle, dtype=float) + mid[0] * u + mid[1] * v + offsets[i] * a)
        rotations.append(np.column_stack((x, a, np.cross(x, a))))
    return np.array(centres), np.array(rotations)


def roll_bends(turns):
    """Fold angles of a roll wound evenly to ``turns`` turns (the first two strips flat together, as the gripper leaves them)."""
    tight = m.tight_bends()
    return tight * (turns * 2 * np.pi / tight.sum())


FLAT = np.zeros(m.STRIPS - 1)
