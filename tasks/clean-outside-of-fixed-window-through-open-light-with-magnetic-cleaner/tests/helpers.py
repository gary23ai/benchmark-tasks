"""Poses for the tests. NumPy only."""

import numpy as np

from rdb_window_wall import cleaner_model as m

FAR = np.array(((.05, .40, .25), (.05, -.40, .25)))  # both grippers well away from the pane (tabletop frame)


def quat(rot):
    """XYZW quaternion of a rotation matrix."""
    w = np.sqrt(max(0., 1. + rot[0, 0] + rot[1, 1] + rot[2, 2])) / 2.
    if w > 1e-6:
        return np.array((rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1], 4 * w * w)) / (4 * w)
    i = int(np.argmax(np.diag(rot)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1. + rot[i, i] - rot[j, j] - rot[k, k]) * 2.
    out = np.zeros(4)
    out[i], out[j], out[k], out[3] = s / 4., (rot[j, i] + rot[i, j]) / s, (rot[k, i] + rot[i, k]) / s, (rot[k, j] - rot[j, k]) / s
    return out


def on_glass(half, u, z=m.WORK_Z, *, gap=0., face=None, turn=None):
    """Tabletop pose row of a half sitting flat on a face of the pane (its own unless ``face`` names the
    other), its centre at pane coordinate ``u`` and height ``z``. ``turn`` is an extra rotation applied in
    the pane frame."""
    face = face or half
    rot = m.on_glass_rotation(face) if turn is None else turn @ m.on_glass_rotation(face)
    return np.r_[m.to_world((u, m.SIDE[face] * (m.PANE_T / 2 + gap), z)), quat(m.PANE_ROT @ rot)]


def on_table(half, un=None):
    """Tabletop pose row of a half lying pad down at pane coordinates ``un`` (its start by default)."""
    return np.r_[m.to_world((*(un if un is not None else m.START_UN[half]), 0.)), m.START_QUAT[half]]


def grip_point(row):
    return row[:3] + m.rotation(row[3:7]) @ m.GRIP_POINT
