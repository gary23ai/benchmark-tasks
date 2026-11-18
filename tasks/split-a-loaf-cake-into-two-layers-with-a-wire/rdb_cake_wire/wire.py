"""A cutting wire drawn through plastic cake. NumPy only.

The wire is a chain of nodes joined by stretch-only springs. It carries no inertia: every node moves
against a drag, and the positions are advanced with one implicit step per call (the minimum of a convex
energy, found by Newton steps with a line search).

Cake. A node at or inside uncut cake is held by the cake until the wire's sideways pull on it exceeds a
yield force, then creeps at a rate set by a viscous term (a Bingham material). Along the wire the cake
resists much less than across it. The same rule applies at the cake's surface, so a wire laid against the
cake does not sink in until it is pulled hard enough. Cake the wire has left behind is recorded as cut on
a plan grid of the cake, with the height of the cut, and offers no resistance to a wire that stays in
that kerf. Cuts within ``line_tolerance`` of the scored line are kept apart from cuts anywhere else.

Everything else the wire touches is a solid it cannot enter: the table and the stand (a height map), and
boxes and capsules given by the caller for the gripper fingers, the arm links and the two handles.

Limits. No spring carries more than ``max_tension``: beyond it the segment stretches at that constant pull
(the energy stays convex), so nothing the wire touches is ever pushed or pulled harder than that. The pull
returned for each end is the wire's pull where it leaves that end's handle: where the first nodes of the wire
are pressed against their own handle, the handle's push on them and their pull on its middle are two halves
of one internal force and cancel, and only the tension beyond the last such node acts on the handle.
"""

import numpy as np

GRAVITY = 9.81
EYE = np.eye(3)


class CuttingWire:
    def __init__(self, *, length, segments, stiffness, linear_mass, yield_per_m, viscous_per_m, along,
                 half, seam, top_layer_half, cell, line_tolerance, kerf=.0015, radius=.0003, air_drag=.002,
                 margin=.004, band=.006, max_tension=np.inf, near=12):
        self.n = segments
        self.rest = length / segments
        self.k = stiffness * segments                 # each segment; `stiffness` is end to end
        self.weight = linear_mass * self.rest * GRAVITY
        self.fy = yield_per_m * self.rest
        self.cv = viscous_per_m * self.rest
        self.along, self.air = along, air_drag
        self.half = np.asarray(half, float)           # whole cake (both layers) half extents, cake frame
        self.seam = float(seam)                       # height of the scored line in the cake frame
        self.top_layer_half = np.asarray(top_layer_half, float)
        self.cell = cell
        self.shape = tuple(int(round(2 * h / cell)) for h in self.half[:2])
        self.tolerance, self.kerf, self.radius = line_tolerance, kerf, radius
        self.margin, self.band = margin, band         # solids are padded by `margin`; nodes within `band` are checked
        self.table = .05                              # N s/m, a node dragged along the surface it lies on
        self.k_solid = 2.e4                           # N/m, a node pressed into a solid
        self.max_tension = max_tension                # N, the most any segment carries
        self.near = near                              # nodes from an end that can lie against that end's handle

    def reset(self, nodes):
        self.x = np.array(nodes, float)
        self.on = np.zeros(self.shape, bool)          # plan cells cut on the scored line
        self.on_z = np.full(self.shape, np.nan)       # and the height of that cut, cake frame
        self.off = np.zeros(self.shape, bool)         # plan cells cut anywhere else
        self.off_z = np.full(self.shape, np.nan)
        self.inside = {}                              # plan cell -> height of the wire now inside it
        self.tension = np.zeros(self.n)
        self.cake_force = np.zeros_like(self.x)
        self.in_top_layer = np.zeros(len(self.x), bool)
        self.leave = [0, self.n]                      # node at which the wire leaves each end's handle
        self.taut_run = [0, 0]                        # taut segments from there to the first node the cake holds
        self.solves = 0

    # --- cake geometry ------------------------------------------------------------------------------
    def cells(self, local):
        index = np.floor((local[:, :2] + self.half[:2]) / self.cell).astype(int)
        return np.clip(index, 0, np.array(self.shape) - 1)

    def state(self, world, cake):
        """For each point: cake-frame position, in uncut cake, in a kerf, in the top_layer."""
        (base_p, base_r), (top_layer_p, top_layer_r), seated = cake
        local = (world - base_p) @ base_r
        whole = np.all(np.abs(local) < self.half, axis=1)
        in_base = whole & (local[:, 2] < self.seam)
        if seated:
            in_top_layer = whole & ~in_base
        else:
            in_top_layer = np.all(np.abs((world - top_layer_p) @ top_layer_r) < self.top_layer_half, axis=1)
        index = self.cells(local)
        i, j = index[:, 0], index[:, 1]
        with np.errstate(invalid="ignore"):
            near = ((self.on[i, j] & (np.abs(local[:, 2] - self.on_z[i, j]) < self.kerf))
                    | (self.off[i, j] & (np.abs(local[:, 2] - self.off_z[i, j]) < self.kerf)))
        kerf = whole & near
        if not seated:
            kerf &= local[:, 2] < self.seam + self.kerf
        return local, (in_base | in_top_layer) & ~kerf, kerf, in_top_layer

    # --- solids -------------------------------------------------------------------------------------
    def half_spaces(self, x, boxes, capsules):
        """Planes (node, normal, offset) that keep nodes out of the solids near them: normal . x >= offset."""
        who, normals, offsets, owners = [], [], [], []
        inner = np.arange(1, self.n)
        for centre, rotation, half in boxes:
            q = (x[inner] - centre) @ rotation
            reach = np.asarray(half) + self.margin
            s = np.abs(q) - reach
            near = np.all(s < self.band, axis=1)
            if not near.any():
                continue
            axis = np.argmax(s[near], axis=1)
            sign = np.where(q[near, axis] >= 0., 1., -1.)
            normal = sign[:, None] * rotation.T[axis]
            who.append(inner[near])
            normals.append(normal)
            offsets.append(normal @ centre + reach[axis])
            owners.append(np.full(int(near.sum()), -1))
        for p0, p1, radius, skip in capsules:
            nodes = np.setdiff1d(inner, np.arange(self.n + 1)[skip])
            axis = p1 - p0
            t = np.clip((x[nodes] - p0) @ axis / (axis @ axis), 0., 1.)
            closest = p0 + t[:, None] * axis
            away = x[nodes] - closest
            dist = np.linalg.norm(away, axis=1)
            reach = radius + self.margin
            near = dist < reach + self.band
            if not near.any():
                continue
            normal = away[near] / np.maximum(dist[near], 1e-9)[:, None]
            normal[dist[near] < 1e-9] = (0., 0., 1.)
            who.append(nodes[near])
            normals.append(normal)
            offsets.append(np.einsum("ij,ij->i", normal, closest[near]) + reach)
            # A capsule that skips the nodes at one end of the wire is the handle that end is fixed to.
            skipped = np.arange(self.n + 1)[skip]
            owners.append(np.full(int(near.sum()), 0 if 0 in skipped else 1 if self.n in skipped else -1))
        if not who:
            return np.zeros(0, int), np.zeros((0, 3)), np.zeros(0), np.zeros(0, int)
        return np.concatenate(who), np.concatenate(normals), np.concatenate(offsets), np.concatenate(owners)

    # --- one step -----------------------------------------------------------------------------------
    def springs(self):
        d = np.diff(self.x, axis=0)
        length = np.linalg.norm(d, axis=1)
        u = d / np.maximum(length, 1e-9)[:, None]
        return u, length, np.minimum(self.k * np.maximum(length - self.rest, 0.), self.max_tension)

    def net(self, u, tension):
        force = np.zeros_like(self.x)
        force[:-1] += tension[:, None] * u
        force[1:] -= tension[:, None] * u
        return force

    def step(self, a, b, cake, floor, dt, boxes=(), capsules=(), iterations=30):
        """Advance by dt with the ends at a and b.

        ``cake`` is ((cake position, rotation), (top layer position, rotation), top layer still seated); the cake
        pose is that of the centre of the whole cake, top layer included. Returns the pulls on the two ends and
        the wire's push on the cake at each node. The pull on an end is taken where the wire leaves that end's
        handle (see the module text); ``leave`` and ``taut_run`` say where that is and how many taut segments
        lie between there and the first node the cake holds (the whole wire, if it is taut from end to end).
        """
        self.x[0], self.x[-1] = a, b
        start = self.x.copy()
        _, in_cake, kerf, _ = self.state(start, cake)
        in_cake[[0, -1]] = False
        ground = floor(start) + self.radius
        *planes, owners = self.half_spaces(start, boxes, capsules)
        for _ in range(3):
            self.x[:] = start
            self.solve(start, in_cake, kerf, ground, planes, dt, iterations)
            _, now, _, _ = self.state(self.x, cake)
            now[[0, -1]] = False
            entered = now & ~in_cake
            if not entered.any():
                break
            in_cake |= entered                        # the cake's surface resists like its inside
        self.record(cake)
        u, _, tension = self.springs()
        self.tension = tension
        _, cake_now, _, self.in_top_layer = self.state(self.x, cake)
        cake_now |= in_cake                           # a node held at the surface also pushes on the cake
        cake_now[[0, -1]] = False
        self.cake_force = np.where(cake_now[:, None], self.net(u, tension), 0.)
        who, normals, offsets = planes
        pressed = offsets - np.einsum("ij,ij->i", normals, self.x[who]) > 0.
        on_a = who[pressed & (owners == 0) & (who <= self.near)]
        on_b = who[pressed & (owners == 1) & (who >= self.n - self.near)]
        a_from, b_from = int(on_a.max(initial=0)), int(on_b.min(initial=self.n))
        self.leave = [a_from, b_from]
        taut, runs = tension > 0., [0, 0]
        for k in range(a_from, self.n):               # segment k joins nodes k and k + 1
            if not taut[k]:
                break
            runs[0] += 1
            if cake_now[k + 1]:
                break
        for k in range(b_from - 1, -1, -1):
            if not taut[k]:
                break
            runs[1] += 1
            if cake_now[k]:
                break
        self.taut_run = runs
        pull_a = tension[a_from] * u[a_from] if a_from < self.n else np.zeros(3)
        pull_b = -tension[b_from - 1] * u[b_from - 1] if b_from > 0 else np.zeros(3)
        return pull_a, pull_b, self.cake_force

    def solve(self, start, in_cake, kerf, ground, planes, dt, iterations):
        x = self.x
        m = self.n - 1                                # interior nodes 1..n-1
        inner = x[1:-1]
        cake = in_cake[1:-1]
        loose = (~in_cake & ~kerf)[1:-1]
        tangent = start[2:] - start[:-2]
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1), 1e-9)[:, None]
        tt = tangent[:, :, None] * tangent[:, None, :]
        # What the cake does to each node is settled from the pull at the start of the step: it holds the
        # node, or yields and resists with the yield force plus a rate term.
        u, length, tension = self.springs()
        pull = self.net(u, tension)[1:-1]
        f_t = np.einsum("ij,ij->i", pull, tangent)[:, None] * tangent
        f_n = pull - f_t
        n_mag, t_mag = np.linalg.norm(f_n, axis=1), np.linalg.norm(f_t, axis=1)
        held = cake & (n_mag <= self.fy) & (t_mag <= self.along * self.fy)
        load = np.zeros((m, 3))
        load[cake] = (-f_n * np.minimum(1., self.fy / np.maximum(n_mag, 1e-12))[:, None]
                      - f_t * np.minimum(1., self.along * self.fy / np.maximum(t_mag, 1e-12))[:, None])[cake]
        drags = np.where(cake[:, None, None], self.cv / dt * ((EYE - tt) + self.along * tt), self.air / dt * EYE)
        load[loose, 2] = -self.weight
        lying = loose & (start[1:-1, 2] <= ground[1:-1] + 1e-5)
        drags[lying] += self.table / dt * np.diag((1., 1., 0.))
        # Solids: the surface under each loose node, and the planes of the boxes and capsules near it.
        who, normals, offsets = planes
        who = np.concatenate((np.nonzero(loose)[0], who - 1))
        normals = np.concatenate((np.tile((0., 0., 1.), (int(loose.sum()), 1)), normals))
        offsets = np.concatenate((ground[1:-1][loose], offsets))

        def sink():
            return np.maximum(offsets - np.einsum("ij,ij->i", normals, inner[who]), 0.)

        def energy():
            d = inner - start[1:-1]
            stretch = np.maximum(np.linalg.norm(np.diff(x, axis=0), axis=1) - self.rest, 0.)
            depth = sink()
            # A spring up to max_tension, a constant pull beyond it.
            most = self.max_tension / self.k
            spring = np.where(stretch <= most, .5 * self.k * stretch ** 2, self.max_tension * (stretch - .5 * most))
            return (.5 * np.einsum("ni,nij,nj->", d, drags, d) - np.sum(load * inner)
                    + spring.sum() + .5 * self.k_solid * depth @ depth)

        rows = np.arange(m)
        for _ in range(iterations):
            self.solves += 1
            u, length, tension = self.springs()
            force = self.net(u, tension)[1:-1]
            depth = sink()
            np.add.at(force, who, self.k_solid * depth[:, None] * normals)
            blocks = np.zeros((m, m, 3, 3))
            blocks[rows, rows] = drags
            pressed = depth > 0.
            np.add.at(blocks, (who[pressed], who[pressed]),
                      self.k_solid * normals[pressed][:, :, None] * normals[pressed][:, None, :])
            blocks[rows[held], rows[held]] += 1e12 * EYE
            taut = np.nonzero(tension > 0.)[0]
            uu = u[taut][:, :, None] * u[taut][:, None, :]
            # (A spring at max_tension has no stiffness along itself. It is given the plain spring's here, which
            # only shortens the Newton step: with none the step is unbounded and the line search stalls.)
            spring = self.k * uu + (tension[taut] / length[taut])[:, None, None] * (EYE - uu)
            lo, hi = taut - 1, taut                   # interior indices of each segment's two nodes
            ok_lo, ok_hi = lo >= 0, hi < m
            np.add.at(blocks, (lo[ok_lo], lo[ok_lo]), spring[ok_lo])
            np.add.at(blocks, (hi[ok_hi], hi[ok_hi]), spring[ok_hi])
            both = ok_lo & ok_hi
            np.add.at(blocks, (lo[both], hi[both]), -spring[both])
            np.add.at(blocks, (hi[both], lo[both]), -spring[both])
            rhs = force + load - np.einsum("nij,nj->ni", drags, inner - start[1:-1])
            rhs[held] = 0.
            move = np.linalg.solve(blocks.transpose(0, 2, 1, 3).reshape(3 * m, 3 * m), rhs.reshape(-1)).reshape(m, 3)
            slope = float(np.sum(rhs * move))
            if np.abs(move).max() < 1e-8 or slope <= 0.:
                break
            before, here, scale = energy(), inner.copy(), 1.
            for _ in range(20):
                inner[:] = here + scale * move
                if energy() <= before - 1e-4 * scale * slope:
                    break
                scale *= .5
            else:
                inner[:] = here
                break

    # --- the cut ------------------------------------------------------------------------------------
    def record(self, cake):
        """Mark as cut every plan cell the wire was in at the last step and has now left."""
        (base_p, base_r), _, seated = cake
        samples = np.concatenate([self.x[:-1] + f * np.diff(self.x, axis=0) for f in (0., .25, .5, .75)])
        local = (samples - base_p) @ base_r
        occupied = np.all(np.abs(local) < self.half, axis=1)
        if not seated:
            occupied &= local[:, 2] < self.seam + self.kerf
        index = self.cells(local[occupied])
        now = {(int(i), int(j)): float(z) for (i, j), z in zip(index, local[occupied, 2])}
        for key, z in self.inside.items():
            if key in now:
                continue
            if abs(z - self.seam) <= self.tolerance:
                self.on[key], self.on_z[key] = True, z
            else:
                self.off[key], self.off_z[key] = True, z
        self.inside = now

    def cut_fraction(self):
        """Shares of the cake's plan cut on the scored line, and cut anywhere else."""
        return float(self.on.mean()), float(self.off.mean())

    def side_cut(self):
        """Length of the cut on the line along each long side face of the cake, m (-y side, +y side)."""
        return float(self.on[:, 0].sum() * self.cell), float(self.on[:, -1].sum() * self.cell)
