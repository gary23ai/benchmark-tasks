"""Regenerate ``rdb_pool_bridge/pool_table.usda`` from ``layout.py`` and ``fixed_scene.py``.

Run from the task folder: ``python tools/build_assets.py``. Needs NumPy and SciPy (convex hulls).

Scopes:
- ``Table`` (static colliders): the slate bed as boxes with six real pocket holes (the slate round each hole is
  convex wedges), six cushions with the nose 36.3 mm above the cloth and their ends cut at the WPA pocket angles
  (142 degrees at the corners, 104 at the sides), the wooden rail caps cut on the same lines, the pockets' back
  walls and drop pockets, the robot's bench and a chalk cube on the rail.
- ``Actors`` (free rigid bodies): the cue (nine convex frustums along a 58 in two-piece cue, 19 oz, balance point
  18 1/2 in from the butt) and three balls (spheres 57.15 mm, 163 g).
- ``Drawn`` (no collision): apron, legs, sights, number circles on the balls.
"""

import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_pool_bridge import fixed_scene as F  # noqa: E402
from rdb_pool_bridge import layout as L  # noqa: E402

# name: colour (linear RGB), roughness, metallic, friction (static = dynamic), restitution
# Friction combines by averaging, restitution by the minimum: ball-ball 0.06 / 0.93, ball-cloth 0.20 / 0.5,
# ball-cushion 0.20 / 0.75, tip-ball 0.60 / 0.72 (Dr Dave's table). The arm's pads carry PhysX's default 0.5.
MATERIALS = {
    "cloth": ((.020, .16, .42), .9, 0., .34, .5),
    "cushion": ((.018, .15, .40), .9, 0., .34, .75),
    "rail_wood": ((.16, .055, .022), .45, 0., .4, .5),
    "leather": ((.05, .025, .012), .7, 0., .6, .1),
    "liner": ((.02, .02, .02), .8, 0., .6, .1),
    "bench": ((.32, .33, .35), .6, .3, .5, .2),
    "chalk": ((.05, .25, .75), .95, 0., .8, .1),
    "ball_cue": ((.86, .85, .78), .15, 0., .06, .93),
    "ball_one": ((.90, .55, .01), .15, 0., .06, .93),
    "ball_three": ((.62, .015, .01), .15, 0., .06, .93),
    "tip_leather": ((.05, .22, .62), .9, 0., 1.14, .72),       # blue chalk on the tip
    "ferrule": ((.88, .88, .84), .3, 0., .3, .6),
    "shaft_maple": ((.80, .62, .38), .35, 0., .10, .3),      # waxed maple through a dusted bridge
    "joint_ring": ((.03, .03, .03), .3, .5, .5, .3),
    "forearm": ((.30, .10, .04), .4, 0., .7, .3),
    "wrap": ((.03, .03, .035), .9, 0., .9, .3),
    "sleeve": ((.20, .06, .025), .35, 0., .5, .3),
    "bumper": ((.02, .02, .02), .9, 0., 1.0, .3),
}
LOOKS = {"decal": ((.92, .92, .9), .3), "glyph": ((.01, .01, .01), .4), "sight": ((.85, .84, .78), .2),
         "apron": ((.13, .045, .018), .5), "bench_leg": ((.15, .15, .16), .5), "iron": ((.55, .50, .40), .3)}
_M = "/Fixture/Materials/"
SECTORS = 24


def _f(values):
    return "(" + ", ".join(f"{float(v) + 0.:.7g}" for v in values) + ")"


def _schemas(collide):
    return ('"MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"' if collide else '"MaterialBindingAPI"')


def _bind(material, collide, offset=.001):
    text = f"            rel material:binding = <{_M}{material}>\n"
    if collide:
        text += (f"            rel material:binding:physics = <{_M}{material}>\n"
                 f"            float physxCollision:contactOffset = {offset:g}\n"
                 "            float physxCollision:restOffset = 0\n")
    return text


def box(name, material, lo, hi, *, collide=True, yaw=0., offset=.001):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, h = (lo + hi) / 2, (hi - lo) / 2
    q = (math.cos(yaw / 2), 0., 0., math.sin(yaw / 2))
    return (f'        def Cube "{name}" (\n            prepend apiSchemas = [{_schemas(collide)}]\n        )\n        {{\n'
            + _bind(material, collide, offset) + "            double size = 2\n"
            f"            quatf xformOp:orient = {_f(q)}\n            float3 xformOp:scale = {_f(h)}\n"
            f"            double3 xformOp:translate = {_f(c)}\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]\n'
            "        }\n\n")


def cylinder(name, material, centre, radius, height, *, axis="Z", collide=False, quat_wxyz=(1., 0., 0., 0.)):
    return (f'        def Cylinder "{name}" (\n            prepend apiSchemas = [{_schemas(collide)}]\n        )\n        {{\n'
            + _bind(material, collide) + f'            uniform token axis = "{axis}"\n'
            f"            double height = {height:.7g}\n            double radius = {radius:.7g}\n"
            f"            quatf xformOp:orient = {_f(quat_wxyz)}\n            double3 xformOp:translate = {_f(centre)}\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n        }\n\n')


def mesh(name, material, points, *, collide=True, faces=None, offset=.001):
    """A convex hull (collider when ``collide``) or, with ``faces``, a display surface."""
    points = np.asarray(points, float)
    if faces is None:
        hull = ConvexHull(points)
        centre = points.mean(axis=0)
        faces = []
        for simplex, eq in zip(hull.simplices, hull.equations):
            a, b, c = points[simplex]
            if np.dot(np.cross(b - a, c - a), eq[:3]) < 0:
                simplex = simplex[[0, 2, 1]]
            faces.append(simplex)
        used = sorted(set(int(i) for f in faces for i in f))
        remap = {old: new for new, old in enumerate(used)}
        points = points[used]
        faces = [[remap[int(i)] for i in f] for f in faces]
        if len(points) > 64:
            raise ValueError(f"{name}: {len(points)} hull vertices (PhysX keeps 64)")
        del centre
    schemas = (_schemas(True) + ', "PhysicsMeshCollisionAPI"') if collide else _schemas(False)
    lo, hi = points.min(axis=0), points.max(axis=0)
    text = (f'        def Mesh "{name}" (\n            prepend apiSchemas = [{schemas}]\n        )\n        {{\n'
            + _bind(material, collide, offset))
    if collide:
        text += '            uniform token physics:approximation = "convexHull"\n'
    return text + ('            uniform token subdivisionScheme = "none"\n'
                   f"            float3[] extent = [{_f(lo)}, {_f(hi)}]\n"
                   f"            int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]\n"
                   f"            int[] faceVertexIndices = [{', '.join(str(int(i)) for f in faces for i in f)}]\n"
                   "            point3f[] points = [" + ", ".join(_f(p) for p in points) + "]\n        }\n\n")


def indent(text, spaces=4):
    pad = " " * spaces
    return "".join(pad + line if line.strip() else line for line in text.splitlines(True))


# -- geometry helpers -----------------------------------------------------------------------------------------------
def clip(poly, normal, offset):
    """Keep the part of a convex polygon where normal . p >= offset (Sutherland-Hodgman)."""
    out = []
    for i, p in enumerate(poly):
        q = poly[(i + 1) % len(poly)]
        dp, dq = normal @ p - offset, normal @ q - offset
        if dp >= 0:
            out.append(p)
        if dp * dq < 0:
            out.append(p + (q - p) * (dp / (dp - dq)))
    return out


def area(poly):
    if len(poly) < 3:
        return 0.
    xs, ys = np.array(poly).T
    return .5 * abs(np.dot(xs, np.roll(ys, -1)) - np.dot(ys, np.roll(xs, -1)))


def ring_pieces(rect, centre, radius, *, keep=None):
    """Convex polygons tiling ``rect`` minus a disk (chords for the arc), one per sector."""
    x0, x1, y0, y1 = rect
    pieces = []
    for k in range(SECTORS):
        a0, a1 = 2 * math.pi * k / SECTORS, 2 * math.pi * (k + 1) / SECTORS
        mid = (a0 + a1) / 2
        if keep is not None and not keep(mid):
            continue
        poly = [np.array(p, float) for p in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
        d0, d1 = np.array((math.cos(a0), math.sin(a0))), np.array((math.cos(a1), math.sin(a1)))
        poly = clip(poly, np.array((-d0[1], d0[0])), np.array((-d0[1], d0[0])) @ centre)       # left of ray a0
        poly = clip(poly, np.array((d1[1], -d1[0])), np.array((d1[1], -d1[0])) @ centre)       # right of ray a1
        m = np.array((math.cos(mid), math.sin(mid)))
        poly = clip(poly, m, m @ centre + radius * math.cos((a1 - a0) / 2))                     # beyond the chord
        if area(poly) > 2e-6:
            pieces.append(poly)
    return pieces


def prism(poly, z0, z1):
    return [(p[0], p[1], z) for z in (z0, z1) for p in poly]


# -- the table ---------------------------------------------------------------------------------------------------------
S = .20     # side of the pocket notches cut out of the bed boxes


def bed_and_pockets():
    x0, x1, y0, y1 = L.OUTER
    z0, z1 = L.BED_Z - L.SLATE_T, L.BED_Z
    ys = L.Y_SIDE
    text = box("bed_middle", "cloth", (x0 + S, y0, z0), (x1 - S, y1, z1))
    for tag, (xa, xb) in (("near", (x0, x0 + S)), ("far", (x1 - S, x1))):
        text += box(f"bed_{tag}_a", "cloth", (xa, y0 + S, z0), (xb, ys - S / 2, z1))
        text += box(f"bed_{tag}_b", "cloth", (xa, ys + S / 2, z0), (xb, y1 - S, z1))
    rects = {"near_left": (x0, x0 + S, y1 - S, y1), "near_side": (x0, x0 + S, ys - S / 2, ys + S / 2),
             "near_right": (x0, x0 + S, y0, y0 + S), "far_left": (x1 - S, x1, y1 - S, y1),
             "far_side": (x1 - S, x1, ys - S / 2, ys + S / 2), "far_right": (x1 - S, x1, y0, y0 + S)}
    for name, pocket in L.POCKETS.items():
        h, r = pocket["hole"], pocket["hole_r"]
        rect = rects[name]
        assert rect[0] < h[0] - r and h[0] + r < rect[1] and rect[2] < h[1] - r and h[1] + r < rect[3], name
        for k, poly in enumerate(ring_pieces(rect, h, r)):
            text += mesh(f"slate_{name}_{k:02d}", "cloth", prism(poly, z0, z1))
        # drop pocket: floor and walls under the hole, leather
        depth = L.BED_Z - L.POCKET_DEPTH
        text += box(f"pocket_floor_{name}", "leather", (h[0] - r - .01, h[1] - r - .01, depth - .01),
                    (h[0] + r + .01, h[1] + r + .01, depth))
        for k in range(12):
            a = 2 * math.pi * k / 12
            c = h + (r + .006) * np.array((math.cos(a), math.sin(a)))
            half = (r + .006) * math.tan(math.pi / 12) + .002
            text += box(f"pocket_wall_{name}_{k:02d}", "leather", (c[0] - .004, c[1] - half, depth),
                        (c[0] + .004, c[1] + half, z0), yaw=a)
        # back of the pocket at rail height: the rail's corner block behind the hole (wood), and its liner
        out = pocket["out"]
        out_angle = math.atan2(out[1], out[0])
        if pocket["kind"] == "corner":
            sx, sy = np.sign(out)
            cx = x0 if sx < 0 else x1
            cy = y1 if sy > 0 else y0
            crect = (min(cx, cx - sx * L.RAIL_W), max(cx, cx - sx * L.RAIL_W) + 0.,
                     min(cy, cy - sy * L.RAIL_W), max(cy, cy - sy * L.RAIL_W))
            crect = (crect[0] if sx < 0 else crect[0] - .02, crect[1] + (.02 if sx < 0 else 0.),
                     crect[2] - (.02 if sy > 0 else 0.), crect[3] + (0. if sy > 0 else .02))
        else:
            sx = np.sign(out[0])
            cx = x0 if sx < 0 else x1
            crect = (min(cx, cx - sx * (L.RAIL_W - L.CUSHION_W)), max(cx, cx - sx * (L.RAIL_W - L.CUSHION_W)),
                     ys - pocket["width"] / 2 - .03, ys + pocket["width"] / 2 + .03)

        def behind(mid, out_angle=out_angle):
            return abs((mid - out_angle + math.pi) % (2 * math.pi) - math.pi) < math.radians(80)

        for k, poly in enumerate(ring_pieces(crect, h, r + .008, keep=behind)):
            text += mesh(f"crown_{name}_{k:02d}", "rail_wood", prism(poly, z1, L.RAIL_TOP))
    return text


def rails_and_cushions():
    """Cushions and wooden caps between the pockets, their ends cut along the pockets' facings."""
    p = L.POCKETS
    rails = [  # (name, start jaw, start facing, end jaw, end facing, outward normal)
        ("near_a", p["near_left"]["jaws"][0], p["near_left"]["facings"][0], p["near_side"]["jaws"][0], p["near_side"]["facings"][0], (-1., 0.)),
        ("near_b", p["near_side"]["jaws"][1], p["near_side"]["facings"][1], p["near_right"]["jaws"][0], p["near_right"]["facings"][0], (-1., 0.)),
        ("far_a", p["far_left"]["jaws"][0], p["far_left"]["facings"][0], p["far_side"]["jaws"][0], p["far_side"]["facings"][0], (1., 0.)),
        ("far_b", p["far_side"]["jaws"][1], p["far_side"]["facings"][1], p["far_right"]["jaws"][0], p["far_right"]["facings"][0], (1., 0.)),
        ("left", p["near_left"]["jaws"][1], p["near_left"]["facings"][1], p["far_left"]["jaws"][1], p["far_left"]["facings"][1], (0., 1.)),
        ("right", p["near_right"]["jaws"][1], p["near_right"]["facings"][1], p["far_right"]["jaws"][1], p["far_right"]["facings"][1], (0., -1.)),
    ]
    cushion = ((0., L.NOSE_H), (.012, 0.), (L.CUSHION_W, 0.), (L.CUSHION_W, L.RAIL_H))
    cap = ((L.CUSHION_W, 0.), (L.RAIL_W, 0.), (L.RAIL_W, L.RAIL_H), (L.CUSHION_W, L.RAIL_H))
    text = ""
    sights = []
    for name, ja, fa, jb, fb, n in rails:
        ja, jb, n = np.asarray(ja), np.asarray(jb), np.asarray(n)
        e = L.unit(jb - ja)
        for part, section, material in (("cushion", cushion, "cushion"), ("cap", cap, "rail_wood")):
            pts = []
            for b, h in section:
                for jaw, f, sign in ((ja, fa, -1.), (jb, fb, 1.)):
                    shift = b * (sign * (f @ e)) / (f @ n)
                    xy = jaw + b * n + sign * shift * e
                    pts.append((xy[0], xy[1], L.BED_Z + h))
            text += mesh(f"{part}_{name}", material, pts)
        sights.append((ja, jb, n, e))
    return text, sights


def drawn_table(sights):
    x0, x1, y0, y1 = L.OUTER
    text = ""
    # sights (round, WPA 7/16 to 1/2 in) 3 11/16 in from the noses, every eighth of the long rails / quarter of the short
    k = 0
    for ja, jb, n, e in sights:
        horizontal = abs(n[0]) > .5
        span = L.PLAY_L / 8 if horizontal else L.PLAY_W / 4
        line = (L.X_N if n[0] < 0 else L.X_F) if horizontal else (L.Y_L if n[1] > 0 else L.Y_R)
        for i in range(1, 8 if horizontal else 4):
            if horizontal:
                pos = np.array((line, L.Y_L - i * span))
                if not (min(ja[1], jb[1]) < pos[1] < max(ja[1], jb[1])):
                    continue
            else:
                pos = np.array((L.X_N + i * span, line))
            c = pos + n * (3 + 11 / 16) * L.IN
            text += cylinder(f"sight_{k:02d}", "sight", (c[0], c[1], L.RAIL_TOP + .0002), .006, .0006)
            k += 1
    # apron under the rail edges, legs, a shelf: what the table looks like below the rails
    zt, zb = L.BED_Z - L.SLATE_T, L.BED_Z - .23
    text += box("apron_near", "apron", (x0, y0, zb), (x0 + .03, y1, zt), collide=False)
    text += box("apron_far", "apron", (x1 - .03, y0, zb), (x1, y1, zt), collide=False)
    text += box("apron_left", "apron", (x0, y1 - .03, zb), (x1, y1, zt), collide=False)
    text += box("apron_right", "apron", (x0, y0, zb), (x1, y0 + .03, zt), collide=False)
    for i, (x, y) in enumerate(((x0 + .08, y0 + .08), (x0 + .08, y1 - .08), (x1 - .08, y0 + .08), (x1 - .08, y1 - .08))):
        text += box(f"leg_{i}", "apron", (x - .06, y - .06, L.FLOOR_Z), (x + .06, y + .06, zb), collide=False)
    # the bench the robot stands on (its top collides; the arms are bolted to it)
    text += box("bench_top", "bench", (-.32, -.70, -.04), (.065, .70, 0.), offset=.002)
    for i, (x, y) in enumerate(((-.29, -.67), (-.29, .67), (.035, -.67), (.035, .67))):
        text += box(f"bench_leg_{i}", "bench_leg", (x - .025, y - .025, L.FLOOR_Z), (x + .025, y + .025, -.04), collide=False)
    # chalk cube on the rail beside the marked pocket (WPA: blue or green chalk)
    c = L.CHALK_XYZ
    text += box("chalk_cube", "chalk", c - .011, c + .011)
    return text


# -- the cue and the balls -----------------------------------------------------------------------------------------------
def ring(s, radius, n=20):
    return [(-s, radius * math.cos(2 * math.pi * k / n), radius * math.sin(2 * math.pi * k / n)) for k in range(n)]


def cue_parts():
    text = ""
    for name, s0, s1, material in L.CUE_SEGMENTS:
        if name == "tip":
            r = L.cue_diameter(0.) / 2
            cap_h = L.TIP_DOME_R - math.sqrt(L.TIP_DOME_R ** 2 - r ** 2)
            pts = [(0., 0., 0.)] + ring(cap_h * .35, math.sqrt(L.TIP_DOME_R ** 2 - (L.TIP_DOME_R - cap_h * .35) ** 2), 12) \
                + ring(cap_h, r) + ring(s1, r)
        else:
            pts = ring(s0, L.cue_diameter(s0 + 1e-6) / 2) + ring(s1, L.cue_diameter(s1 - 1e-6) / 2)
        text += mesh(name, material, pts, offset=.001)
    # display only: diamond-shaped inlays on the sleeve
    for k in range(4):
        a = math.pi / 2 * k
        r = L.cue_diameter(1.36) / 2 + .0002
        text += cylinder(f"inlay_{k}", "decal", (-1.36, r * math.cos(a), r * math.sin(a)), .003, .0004, axis="X")
    return text


def cue_inertia():
    """Transverse moment about the balance point, from the cue's volume with a heavier butt (estimate)."""
    s = np.linspace(0., L.CUE_L, 600)
    area_s = np.pi * np.array([L.cue_diameter(v) for v in s]) ** 2 / 4
    # density chosen so that the mass is 19 oz and the balance point is 18 1/2 in from the butt: two densities
    for split in np.linspace(.5, 1.2, 701):
        rho = np.where(s < split, 1., 2.2)
        m = area_s * rho
        if abs((m * s).sum() / m.sum() - L.CUE_CM_S) < .002:
            break
    w = m / m.sum() * L.CUE_MASS
    transverse = float((w * (s - L.CUE_CM_S) ** 2).sum())
    axial = float((w * np.array([L.cue_diameter(v) for v in s]) ** 2 / 8).sum())
    return axial, transverse


def sphere_patch(normal, radius, angle, lift, n=10, rings=3):
    """Display triangles of a spherical cap of half-angle ``angle`` around ``normal``, ``lift`` off the ball."""
    normal = L.unit(normal)
    a = L.unit(np.cross(normal, (1., 0., 0.) if abs(normal[0]) < .9 else (0., 1., 0.)))
    b = np.cross(normal, a)
    pts = [normal * (radius + lift)]
    for r in range(1, rings + 1):
        t = angle * r / rings
        for k in range(n):
            phi = 2 * math.pi * k / n
            d = math.cos(t) * normal + math.sin(t) * (math.cos(phi) * a + math.sin(phi) * b)
            pts.append(d * (radius + lift))
    faces = [[0, 1 + k, 1 + (k + 1) % n] for k in range(n)]
    for r in range(rings - 1):
        base, nxt = 1 + r * n, 1 + (r + 1) * n
        for k in range(n):
            k1 = (k + 1) % n
            faces += [[base + k, nxt + k, nxt + k1], [base + k, nxt + k1, base + k1]]
    return pts, faces


def glyph(number, normal, radius):
    """The ball's number as thin black bars on its circle (display only)."""
    normal = L.unit(normal)
    up = L.unit(np.cross(normal, (1., 0., 0.) if abs(normal[0]) < .9 else (0., 1., 0.)))
    side = np.cross(up, normal)
    h, w, t = .0058, .0013, .0004
    bars = {1: [((0., 0.), (w, h))],
            3: [((-.0005, .0026), (.0032, w)), ((-.0002, 0.), (.0026, w)), ((-.0005, -.0026), (.0032, w)),
                ((.0012, 0.), (w, h))]}[number]
    out = []
    for (u, v), (bw, bh) in bars:
        centre = normal * (radius + .0006) + side * u + up * v
        pts = [centre + side * su * bw / 2 + up * sv * bh / 2 + normal * sn * t / 2
               for su in (-1, 1) for sv in (-1, 1) for sn in (-1, 1)]
        out.append(pts)
    return out


def ball_parts(name):
    text = (f'        def Sphere "collider" (\n            prepend apiSchemas = [{_schemas(True)}]\n        )\n        {{\n'
            + _bind(f"ball_{name}", True) + f"            double radius = {L.BALL_R:.7g}\n        }}\n\n")
    if name != "cue":
        number = {"one": 1, "three": 3}[name]
        for tag, normal in (("a", (0., 0., 1.)), ("b", (0., 0., -1.))):
            pts, faces = sphere_patch(normal, L.BALL_R, math.asin(.0095 / L.BALL_R), .00025)
            text += mesh(f"circle_{tag}", "decal", pts, collide=False, faces=faces)
            for k, bar in enumerate(glyph(number, normal, L.BALL_R)):
                text += mesh(f"number_{tag}_{k}", "glyph", bar, collide=False,
                             faces=[[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]])
    return text


def body(name, mass, position, rotation, children, *, com=None, inertia=None, iterations=(16, 4)):
    from yam_arena.ik import matrix_to_quat_xyzw

    x, y, z, w = matrix_to_quat_xyzw(rotation)
    extra = ""
    if com is not None:
        extra = (f"        point3f physics:centerOfMass = {_f(com)}\n"
                 f"        float3 physics:diagonalInertia = {_f(inertia)}\n"
                 "        quatf physics:principalAxes = (1, 0, 0, 0)\n")
    return (f'    def Xform "{name}" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", '
            '"PhysxContactReportAPI", "PhysicsMassAPI"]\n    )\n    {\n'
            f"        float physics:mass = {mass:.7g}\n" + extra +
            "        bool physics:rigidBodyEnabled = 1\n        float physxContactReport:threshold = 0\n"
            "        float physxRigidBody:maxDepenetrationVelocity = 1.0\n        float physxRigidBody:sleepThreshold = 0\n"
            f"        int physxRigidBody:solverPositionIterationCount = {iterations[0]}\n"
            f"        int physxRigidBody:solverVelocityIterationCount = {iterations[1]}\n"
            f"        quatf xformOp:orient = {_f((w, x, y, z))}\n        double3 xformOp:translate = {_f(position)}\n"
            '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            + indent(children, 0).replace("\n        def ", "\n        def ") + "    }\n\n")


def material(name):
    colour, rough, metal, friction, restitution = MATERIALS[name]
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = <{_M}{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction:g}\n            float physics:restitution = {restitution:g}\n"
            f"            float physics:staticFriction = {friction:g}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = {_f(colour)}\n                float inputs:metallic = {metal:g}\n"
            f"                float inputs:roughness = {rough:g}\n                token outputs:surface\n            }}\n        }}\n\n")


def look(name):
    colour, rough = LOOKS[name]
    return (f'        def Material "{name}"\n        {{\n'
            f"            token outputs:surface.connect = <{_M}{name}/Surface.outputs:surface>\n\n"
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = {_f(colour)}\n                float inputs:roughness = {rough:g}\n"
            "                token outputs:surface\n            }\n        }\n\n")


def usd():
    top = np.array((0., 0., F.TOP))

    table = bed_and_pockets()
    rail_text, sights = rails_and_cushions()
    table += rail_text
    drawn = drawn_table(sights)
    axial, transverse = cue_inertia()
    cue = body("cue_stick", L.CUE_MASS, L.CUE_REST_TIP + top, L.cue_rotation(L.CUE_REST_AXIS), cue_parts(),
               com=(-L.CUE_CM_S, 0., 0.), inertia=(axial, transverse, transverse), iterations=(32, 8))
    balls = "".join(body(F.BALL_BODY[n], L.BALL_MASS, L.BALL_START[n] + top + (0., 0., .0002), np.eye(3), ball_parts(n))
                    for n in L.BALLS)
    lift = f"        double3 xformOp:translate = {_f(top)}\n        uniform token[] xformOpOrder = [\"xformOp:translate\"]\n\n"
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n'
            '    def Xform "Table"\n    {\n' + lift + table + "    }\n\n"
            '    def Xform "Drawn"\n    {\n' + lift + drawn + "    }\n\n"
            '    def Scope "Actors"\n    {\n' + indent(cue + balls, 4) + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(n) for n in MATERIALS)
            + "".join(look(n) for n in LOOKS) + "    }\n}\n")


if __name__ == "__main__":
    text = usd()
    F.USD.write_text(text)
    print("wrote", F.USD, len(text.splitlines()), "lines", f"cue inertia {cue_inertia()}")
