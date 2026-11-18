"""Magnetic pull between the two halves of the cleaner, evaluated at every 240 Hz physics substep.

Each half carries two block magnets; each magnet is two softened point poles (Gilbert model), one at its
face nearer the pad and one at its far face. Unlike poles attract. Nothing here attaches the halves: the
result is a force and a torque on each rigid body, and contact with the glass does the rest.
"""

import numpy as np

from . import cleaner_model as model


def interaction(p_inside, r_inside, p_outside, r_outside, strength=None):
    """Force and torque about each centre of mass (world frame) on the inside half and on the outside half.

    ``p_*`` are the origins of the halves' frames and ``r_*`` their rotation matrices."""
    strength = model.POLE_STRENGTH if strength is None else strength
    p1, p2 = np.asarray(p_inside, float), np.asarray(p_outside, float)
    r1, r2 = np.asarray(r_inside, float), np.asarray(r_outside, float)
    zero = np.zeros(3)
    if np.linalg.norm(p2 - p1) > model.FIELD_RANGE:
        return zero, zero, zero, zero
    c1, c2 = p1 + r1 @ model.COM, p2 + r2 @ model.COM
    f1, t1, f2, t2 = zero.copy(), zero.copy(), zero.copy(), zero.copy()
    for a_local, s1 in model.poles("inside"):
        a = p1 + r1 @ a_local
        for b_local, s2 in model.poles("outside"):
            b = p2 + r2 @ b_local
            r = b - a
            # Force on pole b from pole a: like poles (s1 s2 > 0) push b away from a.
            force = model.MU0 * strength ** 2 * s1 * s2 / (4 * np.pi) * r / (r @ r + model.SOFTENING ** 2) ** 1.5
            force *= min(1., model.FORCE_CAP / max(np.linalg.norm(force), 1e-12))
            f2 += force
            t2 += np.cross(b - c2, force)
            f1 -= force
            t1 += np.cross(a - c1, -force)
    return f1, t1, f2, t2
