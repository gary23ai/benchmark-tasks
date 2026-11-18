"""Regenerate ``rdb_dounce/dounce.usda`` from ``layout.py`` and ``fixed_scene.py``.

Run from the task folder: ``python tools/build_assets.py``. Needs NumPy and SciPy (for the convex hulls).

- ``Actors/tube``: the grinder's glass tube with its 3.5 mL of sample, one loose rigid body standing in its well.
  Collision: 16 wedge hulls (the wall, inner faces on the bore, outer faces on the outside) and a 16-sided bottom.
- ``Actors/pestle``: the tight pestle B, one loose rigid body: head (capsule), shaft (cylinder), handle (cylinder).
- ``Joints/fluid``: the liquid between them, a PhysX D6 joint with every axis free (it holds nothing in place) and
  one damping drive along the tube's axis: the drag law of ``layout.fluid_force``; the adapter scales its damping
  with how much of the head is under the surface. The head and the bore still collide.
- ``Fixture``: the aluminium cold block and the lab jack, static colliders (boxes) under drawn surfaces.
Glass is drawn as smooth transparent surfaces of revolution with no collision; the colliders are hidden. The
sample, the tape rings and the line on the tube are drawn only.
"""

import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_dounce import fixed_scene as scene  # noqa: E402
from rdb_dounce import layout as L  # noqa: E402

# colour (linear RGB), roughness, metallic, opacity, static friction, dynamic friction
MATERIALS = {
    "glass": ((.86, .93, .96), .03, 0., .18, .50, .45),          # borosilicate; cold, slightly damp outside
    "glass_edge": ((.72, .84, .88), .08, 0., .40, .50, .45),
    "glass_ground": ((.90, .92, .93), .45, 0., .55, .50, .45),   # the ground head of the pestle (frosted)
    "glass_handle": ((.86, .93, .96), .05, 0., .30, .60, .50),   # dry handle
    "sample": ((.80, .70, .50), .35, 0., .80, .50, .45),        # cloudy yeast suspension
    "aluminium": ((.62, .64, .66), .35, 1., 1., .40, .35),
    "aluminium_dark": ((.30, .31, .33), .5, 1., 1., .40, .35),
    "steel": ((.75, .76, .78), .3, 1., 1., .30, .25),
    "rubber": ((.05, .05, .05), .8, 0., 1., .90, .80),
    "tape_black": ((.02, .02, .02), .6, 0., 1., .50, .45),
    "tape_blue": ((.02, .15, .65), .6, 0., 1., .50, .45),
    "tape_red": ((.75, .03, .03), .6, 0., 1., .50, .45),
    "marker": ((.97, .97, .97), .6, 0., 1., .50, .45),
}
_M = "</Dounce/Materials/"
I = "    "


def fmt(v):
    return ", ".join(f"{float(x):.6g}" for x in v)


def hull_mesh(name, points, material, *, visible=False, offset=L.CONTACT_OFFSET, indent=4):
    """A convex-hull collider (at most 64 vertices). The hull's own triangles are written so it can be drawn."""
    pts = np.asarray(points, dtype=float)
    hull = ConvexHull(pts)
    used = sorted(set(hull.simplices.ravel()))
    index = {old: new for new, old in enumerate(used)}
    faces = []
    centre = pts[used].mean(axis=0)
    for tri in hull.simplices:
        a, b, c = (pts[i] for i in tri)
        if np.cross(b - a, c - a) @ (a - centre) < 0:
            tri = tri[::-1]
        faces.extend(index[i] for i in tri)
    p = I * indent
    return (f'{p}def Mesh "{name}" (\n{p}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", '
            f'"PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxConvexHullCollisionAPI"]\n{p})\n{p}{{\n'
            f"{p}    rel material:binding = {_M}{material}>\n"
            f"{p}    rel material:binding:physics = {_M}{material}>\n"
            f'{p}    uniform token physics:approximation = "convexHull"\n'
            f"{p}    int physxConvexHullCollision:hullVertexLimit = 64\n"
            f"{p}    float physxCollision:contactOffset = {offset:g}\n"
            f"{p}    float physxCollision:restOffset = 0\n"
            + ("" if visible else f'{p}    token visibility = "invisible"\n') +
            f"{p}    int[] faceVertexCounts = [{', '.join(['3'] * (len(faces) // 3))}]\n"
            f"{p}    int[] faceVertexIndices = [{', '.join(map(str, faces))}]\n"
            f"{p}    point3f[] points = [{', '.join('(' + fmt(pts[i]) + ')' for i in used)}]\n"
            f'{p}    uniform token subdivisionScheme = "none"\n'
            f"{p}}}\n\n")


def shape(kind, name, centre, material, *, size=None, radius=None, height=None, visible=False,
          offset=L.CONTACT_OFFSET, indent=4):
    """A native collider: Cube (size = full edge lengths), Cylinder or Capsule (axis Z)."""
    p = I * indent
    text = (f'{p}def {kind} "{name}" (\n{p}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", '
            f'"PhysxCollisionAPI"]\n{p})\n{p}{{\n'
            f"{p}    rel material:binding = {_M}{material}>\n"
            f"{p}    rel material:binding:physics = {_M}{material}>\n"
            f"{p}    float physxCollision:contactOffset = {offset:g}\n"
            f"{p}    float physxCollision:restOffset = 0\n"
            + ("" if visible else f'{p}    token visibility = "invisible"\n'))
    if kind == "Cube":
        text += (f"{p}    double size = 1\n{p}    double3 xformOp:translate = ({fmt(centre)})\n"
                 f"{p}    float3 xformOp:scale = ({fmt(size)})\n"
                 f'{p}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n')
    else:
        text += (f'{p}    uniform token axis = "Z"\n{p}    double radius = {radius:.6g}\n{p}    double height = {height:.6g}\n'
                 f"{p}    double3 xformOp:translate = ({fmt(centre)})\n"
                 f'{p}    uniform token[] xformOpOrder = ["xformOp:translate"]\n')
    return text + f"{p}}}\n\n"


def display_mesh(name, points, faces, material, *, indent=4, double_sided=True, extra=""):
    p = I * indent
    counts = ", ".join(str(len(f)) for f in faces)
    flat = ", ".join(str(i) for f in faces for i in f)
    return (f'{p}def Mesh "{name}" (\n{p}    prepend apiSchemas = ["MaterialBindingAPI"]\n{p})\n{p}{{\n'
            f"{p}    rel material:binding = {_M}{material}>\n"
            f"{p}    uniform bool doubleSided = {1 if double_sided else 0}\n"
            f"{p}    int[] faceVertexCounts = [{counts}]\n"
            f"{p}    int[] faceVertexIndices = [{flat}]\n"
            f"{p}    point3f[] points = [{', '.join('(' + fmt(v) + ')' for v in points)}]\n"
            f'{p}    uniform token subdivisionScheme = "none"\n' + extra +
            f"{p}}}\n\n")


def revolve(name, profile, material, *, n=48, centre=(0., 0., 0.), indent=4, extra=""):
    """A surface of revolution about z through ``centre`` from an (r, z) polyline. Display only."""
    profile = np.asarray(profile, dtype=float)
    pts, faces = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        for r, z in profile:
            pts.append((centre[0] + r * math.cos(a), centre[1] + r * math.sin(a), centre[2] + z))
    m = len(profile)
    for k in range(n):
        k2 = (k + 1) % n
        for j in range(m - 1):
            faces.append((k * m + j, k2 * m + j, k2 * m + j + 1, k * m + j + 1))
    return display_mesh(name, pts, faces, material, indent=indent, extra=extra)


def box_display(name, centre, half, material, *, indent=4, rotation=None):
    c = np.asarray(centre, dtype=float)
    h = np.asarray(half, dtype=float)
    r = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
    corners = [c + r @ (h * np.array((sx, sy, sz))) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return display_mesh(name, corners, faces, material, indent=indent)


def band(name, radius, z0, z1, material, *, n=48, indent=4):
    """A thin painted band round an axis: tape on the shaft, a marker line on the tube."""
    return revolve(name, [(radius, z0), (radius, z1)], material, n=n, indent=indent)


def rigid_body(name, mass, position, children, *, com, inertia, indent=2):
    p = I * indent
    return (f'{p}def Xform "{name}" (\n{p}    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", '
            f'"PhysicsMassAPI", "PhysxContactReportAPI"]\n{p})\n{p}{{\n'
            f"{p}    float physics:mass = {mass:.6g}\n"
            f"{p}    point3f physics:centerOfMass = ({fmt(com)})\n"
            f"{p}    float3 physics:diagonalInertia = ({fmt(inertia)})\n"
            f"{p}    bool physics:rigidBodyEnabled = 1\n"
            f"{p}    float physxContactReport:threshold = 0\n"
            f"{p}    float physxRigidBody:maxDepenetrationVelocity = 1\n"
            f"{p}    float physxRigidBody:sleepThreshold = 0\n"
            f"{p}    float physxRigidBody:stabilizationThreshold = 0\n"
            f"{p}    int physxRigidBody:solverPositionIterationCount = 32\n"
            f"{p}    int physxRigidBody:solverVelocityIterationCount = 8\n"
            f"{p}    quatf xformOp:orient = (1, 0, 0, 0)\n"
            f"{p}    double3 xformOp:translate = ({fmt(position)})\n"
            f'{p}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            + children + f"{p}}}\n\n")


def material(name):
    color, roughness, metallic, opacity, static, dynamic = MATERIALS[name]
    extra = f"                float inputs:opacity = {opacity}\n" if opacity < 1 else ""
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Dounce/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {dynamic}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {static}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({fmt(color)})\n"
            f"                float inputs:metallic = {metallic}\n"
            f"                float inputs:roughness = {roughness}\n" + extra +
            ("                float inputs:ior = 1.47\n" if name.startswith("glass") else "") +
            "                token outputs:surface\n            }\n        }\n\n")


def rod_inertia(mass, radius, length):
    side = mass * (3 * radius * radius + length * length) / 12
    return (side, side, mass * radius * radius / 2)


# ---- the glass parts ---------------------------------------------------------------------------------------
def tube_children():
    out = "".join(hull_mesh(n, v, "glass") for n, v in L.tube_pieces().items())
    ro, ri, top = L.TUBE_OD / 2, L.TUBE_ID / 2, L.TUBE_L
    rb = L.RIM_OD / 2
    # outside: a rounded bottom, the wall, the fire-polished bead at the rim; inside: the bore back down
    outer = [(0., 0.), (ro * .55, .0003), (ro * .9, .0015), (ro, .0035), (ro, top - L.RIM_H), (rb, top - L.RIM_H * .6),
             (rb, top - L.RIM_H * .2), (ri + .0004, top), (ri, top - .0004), (ri, L.BOTTOM_T + .0012),
             (ri * .7, L.BOTTOM_T + .0002), (0., L.BOTTOM_T)]
    out += revolve("tube_glass", outer, "glass", n=64)
    out += revolve("tube_rim", [(rb + .00005, top - L.RIM_H * .6), (rb + .00005, top - L.RIM_H * .2)], "glass_edge", n=64)
    # the sample: a column from the bore's bottom to the surface, its top at the surface (height changed at run time)
    rs = ri - .00005
    sample = [(0., 0.), (rs, 0.), (rs, 1.), (0., 1.)]
    out += ('    def Xform "dounce_sample" (\n    )\n    {\n'
            f"        double3 xformOp:translate = (0, 0, {L.BOTTOM_T:.6g})\n"
            f"        float3 xformOp:scale = (1, 1, {L.surface_height(L.START_DEPTH):.6g})\n"
            '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n\n'
            + revolve("liquid", sample, "sample", n=48, indent=8) + "    }\n\n")
    # a marker line round the tube, level with the block's top when the tube sits on the well's floor
    out += band("seat_line", ro + .0001, L.WELL_DEPTH, L.WELL_DEPTH + .001, "marker", n=64)
    out += band("seat_line_2", ro + .0001, L.WELL_DEPTH + L.SEAT_LIFT, L.WELL_DEPTH + L.SEAT_LIFT + .0005, "tape_black", n=64)
    return out


def pestle_children():
    hr, sr, kr = L.HEAD_R_SIM, L.SHAFT_R_SIM, L.KNOB_R_SIM
    out = shape("Capsule", "head", (0., 0., L.HEAD_L / 2), "glass_ground", radius=hr, height=L.HEAD_L - 2 * hr)
    out += shape("Cylinder", "shaft", (0., 0., L.HEAD_L + L.SHAFT_L / 2), "glass", radius=sr, height=L.SHAFT_L + .002)
    out += shape("Cylinder", "handle", (0., 0., L.KNOB_Z), "glass_handle", radius=kr, height=L.KNOB_L)
    rh = L.HEAD_D / 2
    head = [(0., 0.)] + [(rh * math.sin(t), rh * (1 - math.cos(t))) for t in np.linspace(.2, math.pi / 2, 8)] + \
           [(rh, L.HEAD_L - .002), (rh * .8, L.HEAD_L - .0005), (L.SHAFT_D / 2 + .0008, L.HEAD_L + .002),
            (L.SHAFT_D / 2, L.HEAD_L + .004)]
    out += revolve("head_glass", head, "glass_ground", n=48)
    k0, k1 = L.PESTLE_L - L.KNOB_L, L.PESTLE_L
    rk = L.KNOB_D / 2
    shaft_handle = [(L.SHAFT_D / 2, L.HEAD_L + .004), (L.SHAFT_D / 2, k0 - .004), (rk * .8, k0 - .0012), (rk, k0 + .001),
                    (rk, k1 - .004)] + [(rk * math.cos(t), k1 - .004 + .004 * math.sin(t)) for t in np.linspace(.3, math.pi / 2, 6)]
    out += revolve("shaft_glass", shaft_handle, "glass_handle", n=48)
    # three tape rings on the shaft (each 1.5 mm wide, its lower edge at the stated height above the head's bottom)
    for name, z, mat in (("ring_bottom", L.RING_BOTTOM, "tape_black"), ("ring_top", L.RING_TOP, "tape_blue"),
                         ("ring_limit", L.RING_LIMIT, "tape_red")):
        out += band(name, L.SHAFT_D / 2 + .00015, z, z + .0015, mat, n=32)
    return out


def fluid_joint():
    """Every axis free: the joint only carries the liquid's drag along the tube's axis (its x axis here)."""
    q = (math.cos(math.pi / 4), 0., -math.sin(math.pi / 4), 0.)      # joint x along the bodies' z
    return ('    def Scope "Joints"\n    {\n'
            f'        def PhysicsJoint "{scene.FLUID_JOINT}" (\n'
            '            prepend apiSchemas = ["PhysicsDriveAPI:transX", "PhysxJointAPI"]\n        )\n        {\n'
            "            rel physics:body0 = </Dounce/Actors/tube>\n"
            "            rel physics:body1 = </Dounce/Actors/pestle>\n"
            "            point3f physics:localPos0 = (0, 0, 0)\n"
            "            point3f physics:localPos1 = (0, 0, 0)\n"
            f"            quatf physics:localRot0 = ({fmt(q)})\n"
            f"            quatf physics:localRot1 = ({fmt(q)})\n"
            "            bool physics:collisionEnabled = 1\n"
            "            bool physics:excludeFromArticulation = 1\n"
            "            bool physics:jointEnabled = 1\n"
            '            uniform token drive:transX:physics:type = "force"\n'
            "            float drive:transX:physics:stiffness = 0\n"
            f"            float drive:transX:physics:damping = {L.DRAG:.6g}\n"
            "            float drive:transX:physics:targetVelocity = 0\n"
            f"            float drive:transX:physics:maxForce = {L.SUCTION_CAP:.6g}\n"
            "        }\n    }\n\n")


# ---- the block and the jack (static) -----------------------------------------------------------------------
def well_top(centre, z, cell, radius, n=32):
    """The block's top face over one well: a square cell with a round hole (display)."""
    cx, cy = centre
    pts, faces = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        t = cell / 2 / max(abs(c), abs(s))
        pts += [(cx + radius * c, cy + radius * s, z), (cx + t * c, cy + t * s, z)]
    for k in range(n):
        k2 = (k + 1) % n
        faces.append((2 * k, 2 * k2, 2 * k2 + 1, 2 * k + 1))
    return pts, faces


def fixture():
    bx, by, bz = L.BLOCK
    c = scene.BLOCK_CENTRE
    top, floor = L.JACK_H + bz, L.SEAT_Z
    ax, ay = scene.AXIS_XY
    w = L.WELL_D / 2
    out = ""
    # colliders: the block below the wells' floors, and its top layer as four boxes round the well in use
    out += shape("Cube", "block_base", (c[0], c[1], (L.JACK_H + floor) / 2), "aluminium", size=(bx, by, floor - L.JACK_H))
    x0, x1, y0, y1 = c[0] - bx / 2, c[0] + bx / 2, c[1] - by / 2, c[1] + by / 2
    zc, h = (floor + top) / 2, top - floor
    out += shape("Cube", "block_front", ((x0 + ax - w) / 2, c[1], zc), "aluminium", size=(ax - w - x0, by, h))
    out += shape("Cube", "block_back", ((ax + w + x1) / 2, c[1], zc), "aluminium", size=(x1 - ax - w, by, h))
    out += shape("Cube", "block_left", (ax, (ay + w + y1) / 2, zc), "aluminium", size=(2 * w, y1 - ay - w, h))
    out += shape("Cube", "block_right", (ax, (y0 + ay - w) / 2, zc), "aluminium", size=(2 * w, ay - w - y0, h))
    jx, jy = L.JACK
    out += shape("Cube", "jack", (scene.JACK_START[0], scene.JACK_START[1], L.JACK_H / 2), "aluminium_dark",
                 size=(jx, jy, L.JACK_H))
    # drawn: block sides and bottom, its top with 20 wells, each well's wall and floor, a stamped label plate
    sides = [(x0, y0, L.JACK_H), (x1, y0, L.JACK_H), (x1, y1, L.JACK_H), (x0, y1, L.JACK_H),
             (x0, y0, top), (x1, y0, top), (x1, y1, top), (x0, y1, top)]
    out += display_mesh("block_sides", sides, [(0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7), (0, 3, 2, 1)],
                        "aluminium", indent=2)
    pts, faces = [], []
    for well in scene.WELLS:
        p, f = well_top(well, top, L.WELL_PITCH, w)
        faces += [tuple(i + len(pts) for i in face) for face in f]
        pts += p
    # the margins of the top face outside the grid of cells
    gx0, gx1 = c[0] - 2 * L.WELL_PITCH, c[0] + 2 * L.WELL_PITCH
    gy0, gy1 = c[1] - 2.5 * L.WELL_PITCH, c[1] + 2.5 * L.WELL_PITCH
    for (ax0, ax1, ay0, ay1) in ((x0, gx0, y0, y1), (gx1, x1, y0, y1), (gx0, gx1, y0, gy0), (gx0, gx1, gy1, y1)):
        base = len(pts)
        pts += [(ax0, ay0, top), (ax1, ay0, top), (ax1, ay1, top), (ax0, ay1, top)]
        faces.append((base, base + 1, base + 2, base + 3))
    out += display_mesh("block_top", pts, faces, "aluminium", indent=2)
    for i, well in enumerate(scene.WELLS):
        out += revolve(f"well_{i:02d}", [(w, top), (w, floor + .0005), (w * .6, floor), (0., floor)], "aluminium_dark",
                       n=32, centre=(*well, 0.), indent=2)
    out += box_display("block_label", (x0 - .0002, c[1], L.JACK_H + bz * .5), (.0002, .030, .008), "marker", indent=2)
    # the jack: base plate, platform, scissor bars, the screw and its knob, rubber feet
    jc = scene.JACK_START
    out += box_display("jack_base", (jc[0], jc[1], .004), (jx / 2, jy / 2, .004), "aluminium_dark", indent=2)
    out += box_display("jack_platform", (jc[0], jc[1], L.JACK_H - .003), (jx / 2, jy / 2, .003), "aluminium_dark", indent=2)
    span = L.JACK_H - .014
    tilt = math.atan2(span, jx * .8)
    length = math.hypot(span, jx * .8)
    for side in (-1, 1):
        for sign in (-1, 1):
            rot = np.array(((math.cos(tilt), 0., -sign * math.sin(tilt)), (0., 1., 0.), (sign * math.sin(tilt), 0., math.cos(tilt))))
            out += box_display(f"jack_bar_{'lr'[side > 0]}{'ab'[sign > 0]}", (jc[0], jc[1] + side * (jy / 2 - .006), L.JACK_H / 2),
                               (length / 2, .003, .005), "steel", indent=2, rotation=rot)
    out += box_display("jack_screw", (jc[0] - jx / 2 - .010, jc[1], L.JACK_H / 2), (.010 + jx / 2, .003, .003), "steel", indent=2)
    out += revolve("jack_knob", [(0., -.012), (.011, -.012), (.011, 0.), (0., 0.)], "rubber", n=24,
                   centre=(jc[0] - jx / 2 - .010, jc[1], L.JACK_H / 2), indent=2)
    return out


def usd():
    top = np.array((0., 0., scene.TOP))
    tube_com = (0., 0., (L.TUBE_GLASS_KG * L.TUBE_L / 2 + L.LIQUID_KG * (L.BOTTOM_T + L.surface_height(L.START_DEPTH) / 2)) / L.TUBE_KG)
    actors = rigid_body("tube", L.TUBE_KG, scene.TUBE_START + top, tube_children(), com=tube_com,
                        inertia=rod_inertia(L.TUBE_KG, L.TUBE_OD / 2, L.TUBE_L))
    actors += rigid_body("pestle", L.PESTLE_KG, scene.PESTLE_START + top, pestle_children(), com=(0., 0., L.PESTLE_COM_Z),
                         inertia=rod_inertia(L.PESTLE_KG, L.KNOB_D / 2 * .6, L.PESTLE_L))
    return ('#usda 1.0\n(\n    defaultPrim = "Dounce"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Dounce"\n{\n'
            '    def Scope "Actors"\n    {\n' + actors + "    }\n\n" + fluid_joint()
            + '    def Xform "Fixture"\n    {\n'
            + f"        double3 xformOp:translate = (0, 0, {scene.TOP:.6g})\n"
            + '        uniform token[] xformOpOrder = ["xformOp:translate"]\n\n'
            + fixture() + "    }\n\n"
            + '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    text = usd()
    scene.USD.write_text(text)
    print("wrote", scene.USD, len(text.splitlines()), "lines", len(text) // 1024, "KiB")
