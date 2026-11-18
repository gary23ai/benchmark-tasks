"""Write cake.usda from cake_model.py:  python -m rdb_cake_wire.build_scene"""

from pathlib import Path

import numpy as np

from . import cake_model as cake

USD = Path(__file__).with_name("cake.usda")
BODY_API = '["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI", "PhysicsMassAPI"]'
SHAPE_API = '["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMassAPI"]'


def linear(*srgb):
    """Linear RGB of an 8-bit sRGB colour."""
    c = np.array(srgb) / 255.
    return tuple(float(v) for v in np.where(c <= .04045, c / 12.92, ((c + .055) / 1.055) ** 2.4))


# Plain colours, chosen as 8-bit sRGB and written as linear RGB. The cake: a golden-brown top crust in three
# shades, darkest along the ridge and lightest at the shoulder; paler golden sides and ends, where the paper tray kept the loaf from
# browning; pale yellow crumb in the split along the top, in the knife score and on the cut faces. The stand:
# a walnut-brown wooden block with a black silicone mat on it. Pale hardwood handles; matt black film.
COLOURS = {"crust": linear(158, 90, 32), "crust_mid": linear(174, 106, 42), "crust_edge": linear(192, 128, 56),
           "skin": linear(212, 160, 84),
           "crumb": linear(240, 214, 140), "stand": linear(96, 60, 38), "mat": (.02, .02, .02),
           "handle": (.72, .56, .36), "film": (.02, .02, .02)}
ROUGHNESS = {"crust": .6, "crust_mid": .6, "crust_edge": .65, "skin": .75, "crumb": .95, "stand": .55, "mat": .9, "handle": .8, "film": .8}
FRICTION = {name: cake.HANDLE_FRICTION if name in ("handle", "film") else cake.CAKE_FRICTION for name in COLOURS}
CAKE_COLLIDER_MATERIAL = "skin"      # the cake's box collider takes its friction from this material


def fmt(values):
    return "(" + ", ".join(f"{float(v):.6g}" for v in values) + ")"


def world(position):
    return (position[0], position[1], position[2] + cake.TABLE_TOP_WORLD_Z)


def body(name, mass, position, shapes, *, iterations=32, angular_damping=None):
    extra = f"            float physxRigidBody:angularDamping = {angular_damping}\n" if angular_damping else ""
    return f'''        def Xform "{name}" (
            prepend apiSchemas = {BODY_API}
        )
        {{
            float physics:mass = {mass:.6g}
            bool physics:rigidBodyEnabled = 1
{extra}            float physxContactReport:threshold = 0
            float physxRigidBody:maxDepenetrationVelocity = 0.75
            int physxRigidBody:solverPositionIterationCount = {iterations}
            int physxRigidBody:solverVelocityIterationCount = 8
            quatf xformOp:orient = (1, 0, 0, 0)
            double3 xformOp:translate = {fmt(world(position))}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]

{shapes}        }}
'''


def box(name, material, half, *, mass=None, offset=(0., 0., 0.), collide=True, shown=True, indent="            "):
    api = SHAPE_API if collide else '["MaterialBindingAPI"]'
    physics = (f'''{indent}    rel material:binding:physics = </Fixture/Materials/{material}>
{indent}    float physxCollision:contactOffset = 0.0015
{indent}    float physxCollision:restOffset = 0
''' if collide else "")
    weight = f"{indent}    float physics:mass = {mass:.6g}\n" if mass is not None else ""
    weight += "" if shown else f'{indent}    token visibility = "invisible"\n'
    return f'''{indent}def Cube "{name}" (
{indent}    prepend apiSchemas = {api}
{indent})
{indent}{{
{indent}    rel material:binding = </Fixture/Materials/{material}>
{physics}{weight}{indent}    double size = 2
{indent}    float3 xformOp:scale = {fmt(half)}
{indent}    double3 xformOp:translate = {fmt(offset)}
{indent}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
{indent}}}
'''


def dowel(mass):
    return f'''            def Cylinder "wood" (
                prepend apiSchemas = {SHAPE_API}
            )
            {{
                uniform token axis = "X"
                double height = {cake.HANDLE_LENGTH:.6g}
                double radius = {cake.HANDLE_RADIUS:.6g}
                rel material:binding = </Fixture/Materials/handle>
                rel material:binding:physics = </Fixture/Materials/handle>
                float physxCollision:contactOffset = 0.0015
                float physxCollision:restOffset = 0
                float physics:mass = {mass:.6g}
            }}
'''


def material(name):
    return f'''        def Material "{name}" (
            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
        )
        {{
            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>
            float physics:dynamicFriction = {FRICTION[name]:.6g}
            float physics:restitution = 0
            float physics:staticFriction = {FRICTION[name]:.6g}
            uniform token physxMaterial:frictionCombineMode = "average"

            def Shader "Surface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = {fmt(COLOURS[name])}
                float inputs:roughness = {ROUGHNESS[name]:.6g}
                float inputs:metallic = 0
                token outputs:surface
            }}
        }}
'''


# --- drawn shapes: meshes without colliders ---------------------------------------------------------------
def ring(half, corner, z, *, long=20, short=6, arc=6):
    """A rounded rectangle at height z, anticlockwise seen from above, always the same number of points."""
    hx, hy = half
    corner = min(max(corner, 1e-4), hx - 1e-4, hy - 1e-4)
    points = []
    turn = np.linspace(0., np.pi / 2, arc + 1)[:-1]
    for k, (cx, cy) in enumerate(((hx - corner, hy - corner), (corner - hx, hy - corner),
                                  (corner - hx, corner - hy), (hx - corner, corner - hy))):
        if k in (0, 2):                                # the straight that leads to this corner, then its arc
            points += [(cx + np.sign(cx) * corner, v) for v in np.linspace(-cy, cy, short + 1)[:-1]]
        else:
            points += [(v, cy + np.sign(cy) * corner) for v in np.linspace(-cx, cx, long + 1)[:-1]]
        points += [(cx + corner * np.cos(k * np.pi / 2 + s), cy + corner * np.sin(k * np.pi / 2 + s)) for s in turn]
    return np.column_stack((np.array(points), np.full(len(points), z)))


def side_inset(z):
    """How far inside its box collider the cake's drawn side stands at height z above the mat."""
    return float(np.interp(z, (0., .0015, .004, cake.CAKE_SHOULDER_Z),
                           (cake.CAKE_BASE_INSET + .002, cake.CAKE_BASE_INSET + .0006, cake.CAKE_BASE_INSET,
                            cake.CAKE_TOP_INSET)))


def loaf():
    """The drawn cake: rings of points from the mat up to the ridge, and which material lies between them.

    Heights are above the mat. Returns (rings, bands): bands[k] is the material between rings k and k + 1.
    """
    half = cake.CAKE_HALF[:2]

    def wall(z, extra=0.):
        inset = side_inset(z) + extra
        return ring(half - inset, cake.CAKE_CORNER - inset, z)

    seam, w = cake.BASE_H, cake.LINE_WIDTH / 2
    rings = [wall(z) for z in (0., .0015, .004, seam - w)] + [wall(seam, cake.LINE_DEPTH)]
    bands = ["skin", "skin", "skin", "crumb"]
    rings += [wall(seam + w), wall(cake.CAKE_SHOULDER_Z)]
    bands += ["crumb", "skin"]
    # The rounded shoulder, a quarter circle from the side to the start of the top.
    for angle in np.radians((30., 60., 90.)):
        inset = cake.CAKE_TOP_INSET + cake.CAKE_SHOULDER * (1. - np.cos(angle))
        rings.append(ring(half - inset, cake.CAKE_CORNER - inset, cake.CAKE_SHOULDER_Z + cake.CAKE_SHOULDER * np.sin(angle)))
        bands.append("crust_edge")
    # The top: from that last ring each point runs in to the nearest point of the ridge along the middle.
    rim = rings[-1]
    reach = float(half[0] - half[1])
    ridge = np.column_stack((np.clip(rim[:, 0], -reach, reach), np.zeros(len(rim)), np.zeros(len(rim))))
    span = np.linalg.norm((rim - ridge)[:, :2], axis=1)
    around = np.linspace(0., 2 * np.pi, len(rim), endpoint=False)
    lo, hi = cake.CAKE_SPLIT_HALF_WIDTH
    # The split is widest near the middle of the loaf and closes toward both ends; its two edges are ragged
    # and not alike.
    along, side = ridge[:, 0] / reach, np.sign(rim[:, 1])
    ragged = .55 + .27 * np.sin(9. * along + 1.1 + 1.9 * side) + .18 * np.sin(23. * along + .4 - 1.3 * side)
    lip = 1. - (lo + (hi - lo) * ragged * (1. - along ** 2) ** .7) / span                # where the split opens
    wavy = lambda share, k: share * (1. + .12 * np.sin(k * around + share * 9.))         # uneven browning
    rise = cake.CAKE_CREST - rim[0, 2]

    def top(s, drop=0.):
        points = rim + s[:, None] * (ridge - rim)
        height = rise * np.sin(np.pi / 2 * s) ** .85 * (1. - .15 * (points[:, 0] / half[0]) ** 2)
        lumps = .0004 * np.sin(55. * points[:, 0] + 1.3) * np.sin(70. * points[:, 1] + .7) * np.sin(np.pi * s)
        points[:, 2] = np.minimum(rim[:, 2] + height + lumps - drop, cake.CAKE_SIZE[2] - .0002)
        return points

    for share, material in ((.22, "crust_edge"), (.5, "crust_mid"), (.78, "crust")):
        rings.append(top(wavy(share, 7 if share < .4 else 5) * lip))
        bands.append(material)
    rings.append(top(lip, drop=-.0008))                # the lip of the split stands a little proud
    bands.append("crust")
    rings.append(top(lip + .4 * (1. - lip), drop=.7 * cake.CAKE_SPLIT_DEPTH))
    bands.append("crumb")
    rings.append(top(np.ones(len(rim)), drop=cake.CAKE_SPLIT_DEPTH))
    bands.append("crumb")
    return rings, bands


def shell(rings, bands):
    """Points, outward normals and, for each material, the quads of a stack of rings."""
    count = len(rings[0])
    points = np.concatenate(rings)
    quads = {}
    for k, material in enumerate(bands):
        for i in range(count):
            j = (i + 1) % count
            quads.setdefault(material, []).append((k * count + i, k * count + j, (k + 1) * count + j, (k + 1) * count + i))
    normals = np.zeros_like(points)
    for faces in quads.values():
        f = np.array(faces)
        n = np.cross(points[f[:, 2]] - points[f[:, 0]], points[f[:, 3]] - points[f[:, 1]])
        for corner in range(4):
            np.add.at(normals, f[:, corner], n)
    normals /= np.maximum(np.linalg.norm(normals, axis=1), 1e-12)[:, None]
    return points, normals, quads


def cap(loop, up):
    """A flat face closing a ring: points, normals and one polygon, facing up or down."""
    order = np.arange(len(loop)) if up else np.arange(len(loop))[::-1]
    return loop, np.tile((0., 0., 1. if up else -1.), (len(loop), 1)), [tuple(int(i) for i in order)]


def mesh(name, material, points, normals, faces, *, offset=(0., 0., 0.), indent="            "):
    """A drawn mesh with no collider, keeping only the points its faces use."""
    used = sorted({i for face in faces for i in face})
    index = {old: new for new, old in enumerate(used)}
    points = np.asarray(points)[used] + offset
    normals = np.asarray(normals)[used]
    triple = lambda rows, digits: "[" + ", ".join("(" + ", ".join(f"{v:.{digits}g}" for v in row) + ")" for row in rows) + "]"
    return f'''{indent}def Mesh "{name}" (
{indent}    prepend apiSchemas = ["MaterialBindingAPI"]
{indent})
{indent}{{
{indent}    uniform bool doubleSided = 1
{indent}    int[] faceVertexCounts = [{", ".join(str(len(face)) for face in faces)}]
{indent}    int[] faceVertexIndices = [{", ".join(str(index[i]) for face in faces for i in face)}]
{indent}    rel material:binding = </Fixture/Materials/{material}>
{indent}    normal3f[] normals = {triple(normals, 4)} (
{indent}        interpolation = "vertex"
{indent}    )
{indent}    point3f[] points = {triple(points, 7)}
{indent}    uniform token subdivisionScheme = "none"
{indent}}}
'''


def cake_shapes():
    """The drawn meshes of the two layers, each in its own body's frame."""
    rings, bands = loaf()
    points, normals, quads = shell(rings, bands)
    count = len(rings[0])
    seam_ring = bands.index("crumb") + 1               # the ring at the bottom of the score, where the layers meet
    parts = {"base": "", "top_layer": ""}
    centre = {"base": cake.BASE_H / 2, "top_layer": cake.BASE_H + cake.TOP_LAYER_H / 2}
    for material, faces in quads.items():
        for layer, keep in (("base", lambda f: f[0] < seam_ring * count), ("top_layer", lambda f: f[0] >= seam_ring * count)):
            mine = [f for f in faces if keep(f)]
            if mine:
                parts[layer] += mesh(f"{material}_shell", material, points, normals, mine, offset=(0., 0., -centre[layer]))
    seam = rings[seam_ring]
    parts["base"] += mesh("underside", "skin", *cap(rings[0], False), offset=(0., 0., -centre["base"]))
    parts["base"] += mesh("cut_face", "crumb", *cap(seam, True), offset=(0., 0., -centre["base"]))
    parts["top_layer"] += mesh("cut_face", "crumb", *cap(seam, False), offset=(0., 0., -centre["top_layer"]))
    return parts


def stand_shapes():
    """The drawn block and mat, in world coordinates."""
    half, top = cake.STAND_SIZE[:2] / 2, cake.STAND_TOP - cake.MAT_THICKNESS
    at = world(cake.STAND_CENTRE * (1., 1., 0.))
    block = [ring(half, cake.STAND_CORNER, 0.), ring(half, cake.STAND_CORNER, top - cake.STAND_CHAMFER),
             ring(half - cake.STAND_CHAMFER, cake.STAND_CORNER - cake.STAND_CHAMFER, top)]
    points, normals, quads = shell(block, ["stand", "stand"])
    text = mesh("block", "stand", points, normals, quads["stand"], offset=at, indent="        ")
    text += mesh("block_top", "stand", *cap(block[-1], True), offset=at, indent="        ")
    mat = [ring(cake.MAT_SIZE / 2, cake.MAT_CORNER, top), ring(cake.MAT_SIZE / 2, cake.MAT_CORNER, cake.STAND_TOP)]
    points, normals, quads = shell(mat, ["mat"])
    text += mesh("mat_edge", "mat", points, normals, quads["mat"], offset=at, indent="        ")
    text += mesh("mat", "mat", *cap(mat[-1], True), offset=at, indent="        ")
    return text


def scene():
    hx, hy = cake.CAKE_HALF[:2]
    drawn = cake_shapes()
    # Each layer: its box collider, not drawn, and the drawn loaf inside it.
    base = box("cake", CAKE_COLLIDER_MATERIAL, (hx, hy, cake.BASE_H / 2), mass=cake.BASE_MASS, shown=False) + drawn["base"]
    top_layer = (box("cake", CAKE_COLLIDER_MATERIAL, (hx, hy, cake.TOP_LAYER_H / 2), mass=cake.TOP_LAYER_MASS, shown=False)
                 + drawn["top_layer"])
    actors = body("base", cake.BASE_MASS, cake.BASE_CENTRE, base)
    actors += body("top_layer", cake.TOP_LAYER_MASS, cake.TOP_LAYER_CENTRE, top_layer)
    for side in ("left", "right"):
        actors += body(f"handle_{side}", cake.HANDLE_MASS, cake.HANDLE_START[side], dowel(cake.HANDLE_MASS),
                       iterations=48, angular_damping=2.0)
    stand = box("stand", "stand", cake.STAND_SIZE / 2, offset=world(cake.STAND_CENTRE), shown=False, indent="        ")
    stand += stand_shapes()
    film_half = ((cake.FILM_X[1] - cake.FILM_X[0]) / 2, (cake.FILM_Y[1] - cake.FILM_Y[0]) / 2, cake.FILM_DRAWN_THICKNESS / 2)
    film_centre = (sum(cake.FILM_X) / 2, sum(cake.FILM_Y) / 2, cake.FILM_DRAWN_THICKNESS / 2)
    stand += box("film", "film", film_half, offset=world(film_centre), collide=False, indent="        ")
    return f'''#usda 1.0
(
    defaultPrim = "Fixture"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Fixture"
{{
    def Scope "Actors"
    {{
{actors}    }}

    def Scope "Static"
    {{
{stand}    }}

    def Scope "Materials"
    {{
{"".join(material(name) for name in COLOURS)}    }}
}}
'''


if __name__ == "__main__":
    USD.write_text(scene())
    print(f"wrote {USD}")
