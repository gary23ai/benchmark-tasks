"""Two handles that nobody holds, lying on the table with the wire between them. NumPy only.

A stand-in for what PhysX does with the two loose 5 g handles, for testing the wire at rest. Each handle is a
cylinder lying on the table with its axis along x. Across its axis it rolls without slipping and without any
rolling resistance, which is the worst case and what PhysX gives it (effective mass 1.5 m, slowed only by the
scene file's angular damping of 2 per second); along its axis it slides against friction. It is advanced
at the simulator's 240 Hz by the same explicit step, with the force the adapter would apply
(``fixed_scene.handle_force``). The wire is the real model. Yaw and contact between the handles are left out.
"""

import numpy as np

from rdb_cake_wire import cake_model as cake, fixed_scene as fixture

DT = 1. / fixture.PHYSICS_HZ
EYE = np.eye(3)
WHOLE = ((cake.CAKE_CENTRE, EYE), (cake.TOP_LAYER_CENTRE, EYE), True)
ENDS = (slice(0, 4), slice(-4, None))                  # as adapter.WIRE_ENDS
ANGULAR_DAMPING = 2.                                   # build_scene.py, per second
SLIDE = cake.HANDLE_FRICTION * cake.HANDLE_MASS * 9.81


def first_segment_tenth(wire, pulls, i):
    """The coupling before the fix: the first segment's pull, a tenth of it for a loose handle."""
    u, _, tension = wire.springs()
    pull = tension[0] * u[0] if i == 0 else -tension[-1] * u[-1]
    pull = pull * min(1., fixture.END_PULL_LIMIT / max(np.linalg.norm(pull), 1e-9))
    return pull * fixture.LOOSE_STIFFNESS / fixture.WIRE_STIFFNESS


def as_shipped(wire, pulls, i):
    return fixture.handle_force(pulls[i], wire.taut_run[i], False)


class LooseHandles:
    def __init__(self, coupling=as_shipped, max_tension=fixture.END_PULL_LIMIT):
        self.coupling = coupling
        self.wire = fixture.make_wire()
        self.wire.max_tension = max_tension
        self.p = np.array([cake.HANDLE_START[arm] for arm in fixture.ARMS])
        self.v = np.zeros((2, 2))
        fixture.lay_wire(self.wire, self.capsules())   # the adapter's reset: let the wire down onto the table
        self.start = self.p.copy()
        self.nodes = self.wire.x.copy()
        self.peak_force = self.peak_tension = 0.
        self.time = 0.

    def capsules(self):
        return [(*fixture.handle_capsule(p, EYE), cake.HANDLE_RADIUS, skip) for p, skip in zip(self.p, ENDS)]

    def run(self, seconds, push=(0., 0.), stop_at_tension=np.inf):
        """Advance; ``push`` is a steady sideways force (N) on each handle, for disturbing them."""
        for _ in range(round(seconds * fixture.PHYSICS_HZ)):
            pulls = self.wire.step(self.p[0], self.p[1], WHOLE, fixture.floor, DT, (), self.capsules())
            self.peak_tension = max(self.peak_tension, float(self.wire.tension.max()))
            for i in range(2):
                force = self.coupling(self.wire, pulls, i)
                self.peak_force = max(self.peak_force, float(np.linalg.norm(force)))
                along, across = self.v[i]
                if along == 0. and abs(force[0]) <= SLIDE:
                    after = 0.
                else:
                    after = along + (force[0] - SLIDE * np.sign(along if along else force[0])) / cake.HANDLE_MASS * DT
                    after = 0. if along * after < 0. else after
                across += ((force[1] + push[i]) / (1.5 * cake.HANDLE_MASS) - ANGULAR_DAMPING / 3. * across) * DT
                self.v[i] = after, across
                self.p[i, :2] += self.v[i] * DT
            self.time += DT
            if self.wire.tension.max() > stop_at_tension:
                break
        return self

    def moved(self):
        """Furthest either handle, and any node of the wire, is from where it lay at the start, m."""
        return (float(np.linalg.norm(self.p - self.start, axis=1).max()),
                float(np.linalg.norm(self.wire.x - self.nodes, axis=1).max()))
